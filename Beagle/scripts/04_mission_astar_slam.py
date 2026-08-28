from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""receiving zone <-> defect zone 왕복 배달만 하는 미션을 matplotlib 창에서 확인합니다
(OMX box-placed 신호로 구동 -- YOLO class 신호/start/normal zone 없음).
신호는 TriggerServer(TCP+JSON)로 받습니다 -- scripts\\05_send_trigger.py --box-placed로
테스트하거나, 나중에는 실제 OMX 코드가 같은 형식으로 메시지를 보내면 됩니다.

이동 방법: A* + pure pursuit (beagle_sim.py의 set_goal() 그대로 사용).
같은 미션을 line-following으로 돌려보려면 scripts\\07_mission_visual_line.py를 실행하세요.

각 zone은 점이 아니라 실제 크기의 사각형(ZONE_SIZE)으로 그려서 로봇이 그 안에 실제로
들어갔는지 눈으로 바로 확인할 수 있습니다("앞에 멈춤"과 "안에 들어감"을 구분하기 위함).

동작: 항상 receiving zone에서 대기 -> box-placed 신호 -> defect zone 이동 -> 5초 대기
-> receiving zone 복귀 -> 대기, 반복.

--dry-run 플래그로 시뮬레이션과 실물을 전환합니다 (코드 주석 처리 없이).
  --dry-run 있음: 지금까지와 동일한 시뮬레이션 (MockBeagle) + 시각화.
  --dry-run 없음: 실물 로봇(COM 포트)에 연결해서 실제로 주행하면서, 같은 창에
    실시간으로 위치를 보여줍니다. 실물은 ground truth가 없으므로 True/Estimated
    Trajectory가 같은 선으로 겹쳐 보입니다 (beagle_sim.py의 BeagleSimulator가
    dry_run=False일 때 추정 위치를 그대로 미러링하기 때문).
"""

import argparse
from dataclasses import dataclass, field
import math
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from common.comm import TriggerServer
from common.geometry import Pose2D, polar_to_xy
from common.lidar import cardinal_distances, valid_fraction
from common.localization import localize, resolve_180_ambiguity, scan_multiple
from common.motion import align_to_heading_command, dock_to_pose
from common.robot import rectangle_segments
from simulator.beagle_sim import BeagleSimulator, draw_planned_path, draw_pursuit_target

ZONE_SIZE = 0.21  # 각 zone 사각형의 한 변 길이(m) -- 실측값
ZONES = {
    "receiving": (0.35, 0.34),  # measured 2026-08-28: 35cm from left wall, 34cm from bottom wall
    "defect": (0.73, 0.135),  # measured 2026-08-28: 73cm from left wall, 13.5cm from bottom wall
}
ZONE_COLORS = {
    "receiving": "#F5A623",
    "defect": "#D0021B",
}
ROOM_WIDTH_M = 0.90
ROOM_HEIGHT_M = 0.70
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, ROOM_WIDTH_M, ROOM_HEIGHT_M)  # 실측 방 치수 (90cm x 70cm)
# OMX_CENTER는 실측 오차가 있을 수 있어 localize()/dock_to_pose()의 매칭 기준에 넣으면 오히려
# 국소 최적점으로 끌려갈 수 있음 (2026-08-27 확인) -- 방 벽 4개만 기준으로 삼는다.
LOCALIZE_SEGMENTS = ROOM_BOUNDARY
# 사각형 벽만으로는 방 중심 기준 180도 대칭이라 LiDAR 매칭만으로는 실제 위치와 그 반대편
# (예: (0,0) 근처를 (0.9,0.7) 근처로 착각)을 구분할 수 없음 -- OMX 로봇팔(15번 스크립트와
# 동일 실측 위치)을 비대칭 기준물로 넣어야 localize_robot()이 어느 쪽인지 제대로 구분함.
OMX_CENTER = (0.295, 0.34)  # measured 2026-08-28: 5.5cm left of receiving zone, same y row (0.34)
OMX_RADIUS = 0.065
OBSTACLE = rectangle_segments(
    OMX_CENTER[0] - OMX_RADIUS, OMX_CENTER[1] - OMX_RADIUS,
    OMX_CENTER[0] + OMX_RADIUS, OMX_CENTER[1] + OMX_RADIUS,
)
SEGMENTS = ROOM_BOUNDARY + OBSTACLE


@dataclass
class DeliveryMission:
    """receiving zone에서 대기 -> box_placed 신호 -> defect zone 이동 -> 도착 시
    DWELL_SECONDS 대기 -> receiving zone 복귀 -> 대기, 무한 반복.

    상태: WAIT_FOR_BOX -> MOVE_TO_ZONE -> AT_DESTINATION -> RETURN_TO_RECEIVING -> WAIT_FOR_BOX
    """

    zones: dict[str, tuple[float, float]]
    state: str = "WAIT_FOR_BOX"
    state_started: float = field(default_factory=time.monotonic)
    DWELL_SECONDS: float = 5.0

    def on_box_placed(self) -> None:
        if self.state != "WAIT_FOR_BOX":
            return
        self._set_state("MOVE_TO_ZONE")

    def target_zone(self) -> tuple[float, float] | None:
        if self.state == "MOVE_TO_ZONE":
            return self.zones["defect"]
        if self.state == "RETURN_TO_RECEIVING":
            return self.zones["receiving"]
        return None

    def notify_arrived(self) -> None:
        if self.state == "MOVE_TO_ZONE":
            self._set_state("AT_DESTINATION")
        elif self.state == "RETURN_TO_RECEIVING":
            self._set_state("WAIT_FOR_BOX")

    def tick(self) -> None:
        if self.state == "AT_DESTINATION" and time.monotonic() - self.state_started >= self.DWELL_SECONDS:
            self._set_state("RETURN_TO_RECEIVING")

    def dwell_remaining(self) -> float:
        if self.state != "AT_DESTINATION":
            return 0.0
        return max(0.0, self.DWELL_SECONDS - (time.monotonic() - self.state_started))

    def _set_state(self, new_state: str) -> None:
        self.state = new_state
        self.state_started = time.monotonic()


def draw_lidar_view(ax, scan_mm: list[float]) -> None:
    """지금 이 순간 LiDAR가 실제로 보고 있는 것 -- 로봇 기준 상대 좌표(위=전방)로 표시합니다."""
    ax.clear()
    points_mm = polar_to_xy(scan_mm)  # 0도=전방(+x), 90도=좌측(+y), 로봇 로컬 좌표
    if points_mm:
        xs = [p[1] / 1000.0 for p in points_mm]  # 화면상 위쪽이 전방이 되도록 x<->y 교체
        ys = [p[0] / 1000.0 for p in points_mm]
        dists = [math.hypot(p[0], p[1]) for p in points_mm]
        ax.scatter(xs, ys, s=6, c=dists, cmap="RdYlGn", vmin=100, vmax=1500)
    ax.plot(0, 0, "^", color="#20242C", markersize=14)  # robot, pointing up (front)
    cd = cardinal_distances(scan_mm)
    vf = valid_fraction(scan_mm)
    ax.text(
        0.02, 0.98,
        f"front={cd['front']:.0f}mm  rear={cd['rear']:.0f}mm\n"
        f"left={cd['left']:.0f}mm  right={cd['right']:.0f}mm\n"
        f"valid={vf * 100:.0f}%",
        transform=ax.transAxes, va="top", ha="left", fontsize=9,
        bbox=dict(facecolor="white", alpha=0.8, edgecolor="#33415F"),
    )
    ax.set_xlim(-2.0, 2.0)
    ax.set_ylim(-2.0, 2.0)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title("LiDAR (robot-relative, front=up)")


def draw_zones(ax) -> None:
    half = ZONE_SIZE / 2.0
    for zone_name, (zx, zy) in ZONES.items():
        rect = Rectangle(
            (zx - half, zy - half), ZONE_SIZE, ZONE_SIZE,
            facecolor=ZONE_COLORS[zone_name], edgecolor="#20242C", linewidth=1.5, alpha=0.85,
        )
        ax.add_patch(rect)
        ax.text(zx, zy, zone_name, ha="center", va="center", fontsize=9, color="white", weight="bold")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--start-theta", type=float, default=None,
                        help="Override the LiDAR-detected starting heading in degrees "
                             "(use if localize()/resolve_180_ambiguity() picks the wrong heading)")
    parser.add_argument("--known-start", action="store_true",
                        help="Skip LiDAR localize() entirely and assume the robot is placed "
                             "exactly at ZONES['receiving'] with heading --start-theta (default 0). "
                             "Use when you physically place the robot at a fixed, known pose every "
                             "time, to remove localization error/instability as a variable.")
    parser.add_argument("--dock", action="store_true",
                        help="Before starting the mission, iteratively re-scan+localize and nudge "
                             "the robot (turn/drive via gyro+encoder) until it sits within 1cm/3deg "
                             "of ZONES['receiving'] facing 3 o'clock (heading=0deg). Use this instead "
                             "of relying on hand-placement accuracy. Real hardware only.")
    args = parser.parse_args()

    sim = BeagleSimulator(
        SEGMENTS,
        Pose2D(*ZONES["receiving"], math.atan2(
            ZONES["defect"][1] - ZONES["receiving"][1], ZONES["defect"][0] - ZONES["receiving"][0]
        )),
        # Confirmed 2026-08-28 via [formula debug]/[distance debug] logs: encoder+gyro
        # dead-reckoning (integrate_wheel_distances) is correct -- predicted dx,dy from the
        # mid_theta formula matched the actual pose delta exactly, every step. But the pos
        # printed right after (post scan_match()) kept diverging from that in the OPPOSITE
        # direction by several cm per frame. scan_match() runs every frame against a grid
        # built from this same uncorrected pose estimate (self-referential), and was yanking
        # est_pose backward relative to the correct odometry result -- this, not theta or the
        # dead-reckoning math, was the real source of "moving opposite of theta" on real
        # hardware. Disabled here; drift correction now comes from relocalize() instead (see
        # 04_mission_astar_slam_relocalize.py), which re-localizes against the full LiDAR scan
        # only at zone arrival instead of nudging the pose every single frame.
        odom_noise=0.06, use_slam=False, dry_run=args.dry_run,
    )

    # 실물은 로봇이 receiving zone 안 정확히 어디에, 어느 방향으로 놓였는지 알 수 없으므로
    # (dry-run처럼 정확한 시작 pose를 가정할 수 없음) LiDAR로 실제 위치/heading을 먼저
    # 찾습니다. receiving에서 충분히 떨어져 있으면 미션을 "WAIT_FOR_BOX"가 아니라
    # "RETURN_TO_RECEIVING"으로 시작해서, 기존 루프가 그대로 그쪽으로 이동시킨 뒤 자동으로
    # WAIT_FOR_BOX로 넘어가게 합니다.
    initial_state = "WAIT_FOR_BOX"
    if args.known_start:
        theta = math.radians(args.start_theta) if args.start_theta is not None else 0.0
        detected_pose = Pose2D(*ZONES["receiving"], theta)
        print(f"[--known-start] Skipping localize() -- assuming pose ({detected_pose.x:.3f}, "
              f"{detected_pose.y:.3f}) heading={math.degrees(theta):.0f}deg")
        sim.est_pose = detected_pose
        sim.robot.pose = detected_pose
        print("이미 receiving zone 근처입니다.")
    elif not args.dry_run:
        print("실제 시작 위치 파악 중 (제자리에서 LiDAR 스캔)...")
        scan = scan_multiple(sim.robot)
        # OMX_CENTER is a rough hand-measurement, not surveyed -- feeding an uncertain
        # obstacle position into the match score can pull localize() toward a wrong local
        # optimum (seen 2026-08-27: 14cm position error while placed exactly at receiving,
        # with a deceptively low 8mm match error). Matching against the room rectangle only
        # removes that risk; the tradeoff is losing OMX as a 180-degree-ambiguity landmark,
        # so resolve_180_ambiguity() may pick the wrong (mirrored) heading. The real OMX
        # obstacle stays in SEGMENTS for A* planning/collision-avoidance -- only localization
        # skips it.
        detected_pose, match_err_m = localize(scan, LOCALIZE_SEGMENTS, *ZONES["receiving"], search_radius=0.3)
        # 격자 탐색 해상도 때문에 180도 대칭 쌍(예: (0,0) 근처 vs (0.9,0.7) 근처) 중 더 잘
        # 맞는 쪽을 놓쳤을 수 있으니, 그 두 후보만 정밀하게 다시 채점해서 최종 확정합니다.
        # Confirmed 2026-08-28: matching against LOCALIZE_SEGMENTS (walls only) here always
        # ties exactly (a plain rectangle is perfectly 180-degree symmetric), so this pick was
        # pure floating-point luck -- seen picking the wrong twin (38cm off). SEGMENTS (which
        # includes OMX, now measured precisely as 5.5cm from receiving zone -- see OMX_CENTER)
        # is the one place OMX's asymmetry is actually needed, so use it only for this
        # disambiguation step; the coarse localize() search above still uses walls only to
        # stay robust if OMX_CENTER ever drifts again.
        detected_pose, match_err_m = resolve_180_ambiguity(
            scan, SEGMENTS, detected_pose, ROOM_WIDTH_M, ROOM_HEIGHT_M
        )
        if args.start_theta is not None:
            print(f"[--start-theta override] localize() said {math.degrees(detected_pose.theta) % 360.0:.0f}deg, "
                  f"forcing {args.start_theta:.0f}deg instead")
            detected_pose = Pose2D(detected_pose.x, detected_pose.y, math.radians(args.start_theta))
        heading_deg = math.degrees(detected_pose.theta) % 360.0
        print(
            f"추정 위치: ({detected_pose.x:.3f}, {detected_pose.y:.3f}) heading={heading_deg:.0f}deg "
            f"(매칭오차 {match_err_m * 1000:.0f}mm)"
        )
        sim.est_pose = detected_pose
        sim.robot.pose = detected_pose  # ground truth 없음 -- 추정치를 그대로 미러링
        dist_to_receiving = math.hypot(
            detected_pose.x - ZONES["receiving"][0], detected_pose.y - ZONES["receiving"][1]
        )
        if dist_to_receiving > 0.05:
            print(f"receiving zone과 {dist_to_receiving * 100:.0f}cm 떨어져 있습니다 -- 먼저 그쪽으로 이동합니다.")
            initial_state = "RETURN_TO_RECEIVING"
        else:
            print("이미 receiving zone 근처입니다.")

    if args.dock and not args.dry_run:
        # 3시 방향(clock-position 표현) == 방 x축(+x) 방향 == theta=0deg.
        print("receiving zone에 정밀 도킹 중 (목표: 오차 1cm/3deg 이내, heading=3시 방향)...")
        docked_pose, docked_ok = dock_to_pose(
            sim.robot, LOCALIZE_SEGMENTS, ROOM_WIDTH_M, ROOM_HEIGHT_M,
            *ZONES["receiving"], 0.0, disambiguation_segments=SEGMENTS, pos_tol_m=0.01,
        )
        sim.est_pose = docked_pose
        sim.robot.pose = docked_pose
        status = "성공" if docked_ok else "실패 (max_iters 소진 -- 그래도 가장 가까웠던 pose로 진행)"
        print(f"[dock] {status}: ({docked_pose.x:.3f}, {docked_pose.y:.3f}) "
              f"heading={math.degrees(docked_pose.theta):.1f}deg")
        initial_state = "WAIT_FOR_BOX"

    mission = DeliveryMission(zones=ZONES, state=initial_state)
    server = TriggerServer(host="0.0.0.0", port=8765)
    server.start()

    plt.ion()
    fig, (ax, ax_lidar) = plt.subplots(1, 2, figsize=(13, 6.5))
    fig.canvas.manager.set_window_title("Mission Test (astar+slam)")

    running = {"value": True}

    def on_key(event) -> None:
        if (event.key or "").lower() == "q":
            running["value"] = False

    fig.canvas.mpl_connect("key_press_event", on_key)

    true_trail: list[tuple[float, float]] = []
    est_trail: list[tuple[float, float]] = []
    last_mission_state = None
    all_paths: list[list[tuple[float, float]]] = []  # every leg's planned path, kept for the whole run
    leg_phase = "TRACKING"  # "ALIGNING" (turn in place first) or "TRACKING" (pure pursuit)
    align_heading = 0.0
    previous = time.monotonic()
    last_print_time = 0.0

    try:
        while running["value"] and plt.fignum_exists(fig.number):
            now = time.monotonic()
            # Confirmed 2026-08-27 (in beagle_sim.py's own loop): matplotlib rendering
            # routinely takes 0.28-0.4s+ per frame on real hardware, well over the old 0.15s
            # cap, which silently discarded most of dt every frame and made gyro-based
            # rotation tracking undercount real rotation by ~3-4x. This loop has its own
            # separate dt calculation (doesn't go through beagle_sim.py's _run_loop()), so it
            # needed the same fix independently.
            # Lowered 1.0->0.5 on 2026-08-28 (see beagle_sim.py's matching fix note) -- a rare
            # multi-second dt outlier could integrate a large single-step position jump.
            dt = min(0.5, max(0.0, now - previous))
            previous = now

            for message in server.poll():
                if message.get("event") == "box_placed":
                    mission.on_box_placed()

            mission.tick()

            if mission.state != last_mission_state:
                target = mission.target_zone()
                if target is not None:
                    sim.set_goal(*target)  # computes/stores the path; auto_on flipped back off below
                    all_paths.append(list(sim.auto_path))
                    align_heading = math.atan2(target[1] - sim.est_pose.y, target[0] - sim.est_pose.x)
                    leg_phase = "ALIGNING"
                    sim.auto_on = False  # hold off pure pursuit until roughly facing the target
                else:
                    sim.auto_on = False
                    sim.cmd = (0.0, 0.0)
                last_mission_state = mission.state

            if leg_phase == "ALIGNING":
                # turn_speed raised from the 10.0 default on 2026-08-27 to match the faster
                # TRACKING speed (see beagle_sim.py's pure_pursuit speed_mps/wheel-clamp bump).
                left, right, aligned = align_to_heading_command(sim.est_pose, align_heading, turn_speed=13.0)
                sim.cmd = (left, right)
                if aligned:
                    leg_phase = "TRACKING"
                    sim.auto_on = True  # hand off to set_goal()'s pure pursuit on the path already computed above

            sim.step(dt)
            scan = sim.robot.lidar()

            if sim.status == "GOAL!" and mission.state in {"MOVE_TO_ZONE", "RETURN_TO_RECEIVING"}:
                mission.notify_arrived()

            true_trail.append((sim.robot.pose.x, sim.robot.pose.y))
            est_trail.append((sim.est_pose.x, sim.est_pose.y))

            if now - last_print_time >= 0.5:
                cd = cardinal_distances(scan)
                vf = valid_fraction(scan)
                print(
                    f"[{mission.state}/{leg_phase}] pos=({sim.est_pose.x:.3f},{sim.est_pose.y:.3f}) "
                    f"heading={math.degrees(sim.est_pose.theta):.1f}deg cmd={sim.cmd} status={sim.status} | "
                    f"lidar front={cd['front']:.0f} rear={cd['rear']:.0f} left={cd['left']:.0f} "
                    f"right={cd['right']:.0f}mm valid={vf * 100:.0f}%"
                )
                if vf < 0.5:
                    print(f"  [WARN] LiDAR 유효 비율이 낮습니다 ({vf * 100:.0f}%) -- 센서/반사 문제 의심")
                last_print_time = now

            ax.clear()
            for sx, sy, ex, ey in ROOM_BOUNDARY:
                ax.plot([sx, ex], [sy, ey], color="#33415F", linewidth=3)
            draw_zones(ax)
            for i, leg_path in enumerate(all_paths):
                draw_planned_path(ax, leg_path, label="A* Path" if i == 0 else "_nolegend_")
            draw_pursuit_target(ax, (sim.est_pose.x, sim.est_pose.y), sim.pp_target_point)
            if true_trail:
                xs, ys = zip(*true_trail)
                ax.plot(xs, ys, color="#D9534F", linewidth=1.4, label="True Trajectory")
            if est_trail:
                xs, ys = zip(*est_trail)
                ax.plot(xs, ys, "--", color="#2C74F5", linewidth=1.4, label="Estimated Trajectory")
            ax.plot(sim.robot.pose.x, sim.robot.pose.y, "o", color="#D9534F", markersize=10)
            ax.plot(sim.est_pose.x, sim.est_pose.y, "x", color="#2C74F5", markersize=9)
            ax.set_aspect("equal")
            ax.grid(True, alpha=0.3)
            dwell_tag = f" | waiting {mission.dwell_remaining():.1f}s" if mission.state == "AT_DESTINATION" else ""
            phase_tag = f" [{leg_phase}]" if mission.target_zone() is not None else ""
            mode_tag = "DRY-RUN" if args.dry_run else "REAL"
            ax.set_title(
                f"Mission: {mission.state}{dwell_tag}{phase_tag} | {mode_tag} | listening on :8765 | Q=quit"
            )
            ax.legend(loc="lower left", fontsize=8)

            draw_lidar_view(ax_lidar, scan)

            fig.canvas.draw_idle()
            plt.pause(0.03)
    finally:
        server.stop()
        sim.robot.stop()

    plt.ioff()
    if plt.fignum_exists(fig.number):
        plt.show()


if __name__ == "__main__":
    main()
