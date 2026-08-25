from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""receiving zone <-> defect zone 왕복 배달만 하는 단순 미션 (YOLO 클래스 구분/start/normal
zone 없음 -- 지금 당장 실행할 시나리오에 맞춘 축소판). common/mission.py의 Mission은 07~10번
스크립트가 여전히 그 4-zone API에 의존하므로 건드리지 않고, 이 파일 안에 별도의 2-zone
DeliveryMission을 둡니다.

동작:
  1) 항상 receiving zone에서 대기 (WAIT_FOR_BOX).
  2) OMX가 TriggerServer로 {"event": "box_placed"}를 보내면 defect zone으로 출발
     -- 이 순간 상태 "출발"을 (있다면) 대시보드로 전송.
  3) 이동 중 장애물(OMX 로봇팔 실측 위치)을 만나면 beagle_sim.py에 이미 있는
     backup-turn-replan 복구 로직이 자동으로 처리 (이 파일은 별도 코드 불필요).
  4) 도착하면 상태 "도착" 전송, DWELL_SECONDS(5초) 대기 후 receiving zone으로 복귀.
  5) receiving zone에 도착하면 상태 "대기 중" 전송, 1)로 되돌아가서 반복.

실물 로봇은 시작 시 정확한 위치를 모르므로, --dry-run 없이 실행하면 먼저 LiDAR로 실제
위치를 찾고 receiving zone과 떨어져 있으면 그쪽으로 이동한 뒤 대기를 시작합니다
(04_mission_astar_slam.py와 동일한 방식).

--status-host를 주면 상태를 Ubuntu 쪽 16_status_dashboard.py로 실시간 전송합니다
(생략하면 전송하지 않고 로컬 창에만 표시 -- 대시보드 없이도 그냥 실행 가능).

사용법:
  python scripts\\15_mission_defect_delivery.py --dry-run                              # 시뮬레이션만
  python scripts\\15_mission_defect_delivery.py                                        # 실물 로봇
  python scripts\\15_mission_defect_delivery.py --status-host 100.x.y.z                # 실물 + 대시보드 전송
  python scripts\\05_send_trigger.py --box-placed                                      # 박스 배치 신호 테스트
"""

import argparse
from dataclasses import dataclass, field
import math
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from common.comm import StatusClient, TriggerServer
from common.geometry import Pose2D, polar_to_xy
from common.lidar import cardinal_distances, valid_fraction
from common.localization import localize, resolve_180_ambiguity, scan_multiple
from common.motion import align_to_heading_command
from common.robot import rectangle_segments
from simulator.beagle_sim import BeagleSimulator, draw_planned_path, draw_pursuit_target

ZONE_SIZE = 0.21  # 각 zone 사각형의 한 변 길이(m) -- 실측값
ZONES = {
    "receiving": (0.41, 0.38),
    "defect": (0.75, 0.13),
}
ZONE_COLORS = {
    "receiving": "#F5A623",
    "defect": "#D0021B",
}
ROOM_WIDTH_M = 0.90
ROOM_HEIGHT_M = 0.70
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, ROOM_WIDTH_M, ROOM_HEIGHT_M)  # 실측 방 치수 (90cm x 70cm)
# 시작 시 로봇이 방 어디에 있을지 모름(반드시 receiving zone 근처라는 보장 없음) -- 방
# 중심에서 대각선 절반 거리(약 0.57m) + 여유를 반경으로 잡아 방 전체를 덮습니다.
LOCALIZE_CENTER = (ROOM_WIDTH_M / 2.0, ROOM_HEIGHT_M / 2.0)
LOCALIZE_RADIUS = math.hypot(ROOM_WIDTH_M, ROOM_HEIGHT_M) / 2.0 + 0.05
# 대부분은 실제로 receiving zone 근처에서 시작하므로, 그 좁은 범위부터 먼저(빠르게) 찾아보고
# 잘 안 맞을 때만 위의 방 전체 탐색으로 확대합니다 (풀 스캔은 방 전체 기준 ~1분 걸림).
QUICK_LOCALIZE_RADIUS = 0.3
QUICK_LOCALIZE_OK_ERROR_M = 0.03
OMX_CENTER = (0.17, 0.37)
OMX_RADIUS = 0.065
OBSTACLE = rectangle_segments(
    OMX_CENTER[0] - OMX_RADIUS, OMX_CENTER[1] - OMX_RADIUS,
    OMX_CENTER[0] + OMX_RADIUS, OMX_CENTER[1] + OMX_RADIUS,
)  # OMX 로봇팔 실측 위치를 감싸는 사각형
SEGMENTS = ROOM_BOUNDARY + OBSTACLE
PORT = 8769  # 8765~8768은 04/07/08/09가 이미 사용 중

STATUS_TEXT = {
    "MOVE_TO_ZONE": "출발",
    "AT_DESTINATION": "도착",
    "WAIT_FOR_BOX": "대기 중",
}  # RETURN_TO_RECEIVING은 매핑 없음 -- 복귀 이동 자체는 신호를 보내지 않음 (요청된 사양)


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
    points_mm = polar_to_xy(scan_mm)
    if points_mm:
        xs = [p[1] / 1000.0 for p in points_mm]
        ys = [p[0] / 1000.0 for p in points_mm]
        dists = [math.hypot(p[0], p[1]) for p in points_mm]
        ax.scatter(xs, ys, s=6, c=dists, cmap="RdYlGn", vmin=100, vmax=1500)
    ax.plot(0, 0, "^", color="#20242C", markersize=14)
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
    parser.add_argument("--status-host", default=None, help="상태를 실시간으로 받을 Ubuntu 대시보드 주소 (Tailscale IP 등)")
    parser.add_argument("--status-port", type=int, default=8770)
    args = parser.parse_args()

    start_heading = math.atan2(
        ZONES["defect"][1] - ZONES["receiving"][1], ZONES["defect"][0] - ZONES["receiving"][0]
    )
    sim = BeagleSimulator(
        SEGMENTS, Pose2D(*ZONES["receiving"], start_heading),
        odom_noise=0.06, use_slam=True, dry_run=args.dry_run,
    )

    # 실물은 로봇이 receiving zone 안 정확히 어디에, 어느 방향으로 놓였는지 알 수 없으므로
    # LiDAR로 실제 위치/heading을 먼저 찾습니다. receiving에서 충분히 떨어져 있으면
    # RETURN_TO_RECEIVING으로 시작해서 그쪽으로 먼저 이동한 뒤 자동으로 대기로 넘어갑니다.
    initial_state = "WAIT_FOR_BOX"
    if not args.dry_run:
        print("실제 시작 위치 파악 중 (제자리에서 LiDAR 스캔)...")
        scan = scan_multiple(sim.robot)

        print(f"빠른 탐색 중 (receiving zone 주변 {QUICK_LOCALIZE_RADIUS * 100:.0f}cm)...")
        # ROOM_BOUNDARY(사각형)만 보고 매칭하면 방 중심 기준 180도 회전 대칭이라 실제
        # pose와 그 180도 뒤집힌 pose가 매칭 점수가 완전히 같아서 heading이 반대로 나올 수
        # 있음 -- OMX 장애물(SEGMENTS)까지 포함해서 매칭해야 그 대칭이 깨져 정확히 구분됨.
        detected_pose, match_err_m = localize(
            scan, SEGMENTS, *ZONES["receiving"], QUICK_LOCALIZE_RADIUS, verbose=False
        )
        if match_err_m > QUICK_LOCALIZE_OK_ERROR_M:
            print(
                f"빠른 탐색 매칭오차 {match_err_m * 1000:.0f}mm -- receiving zone 근처가 아닌 듯합니다. "
                "방 전체로 확대 탐색합니다 (시간이 더 걸립니다)..."
            )
            detected_pose, match_err_m = localize(scan, SEGMENTS, *LOCALIZE_CENTER, LOCALIZE_RADIUS)

        # 그리드 탐색 해상도 때문에 180도 대칭 쌍 중 더 잘 맞는 쪽을 놓쳤을 수 있으니,
        # 그 두 후보만 정밀하게(그리드에 얽매이지 않고) 다시 채점해서 최종 확정합니다.
        detected_pose, match_err_m = resolve_180_ambiguity(
            scan, SEGMENTS, detected_pose, ROOM_WIDTH_M, ROOM_HEIGHT_M
        )

        heading_deg = math.degrees(detected_pose.theta) % 360.0
        print(
            f"추정 위치: ({detected_pose.x:.3f}, {detected_pose.y:.3f}) heading={heading_deg:.0f}deg "
            f"(매칭오차 {match_err_m * 1000:.0f}mm)"
        )
        sim.est_pose = detected_pose
        sim.robot.pose = detected_pose
        dist_to_receiving = math.hypot(
            detected_pose.x - ZONES["receiving"][0], detected_pose.y - ZONES["receiving"][1]
        )
        if dist_to_receiving > 0.05:
            print(f"receiving zone과 {dist_to_receiving * 100:.0f}cm 떨어져 있습니다 -- 먼저 그쪽으로 이동합니다.")
            initial_state = "RETURN_TO_RECEIVING"
        else:
            print("이미 receiving zone 근처입니다.")

    mission = DeliveryMission(zones=ZONES, state=initial_state)

    status_client: StatusClient | None = None
    if args.status_host:
        status_client = StatusClient(args.status_host, args.status_port)
        status_client.start()
        initial_status = STATUS_TEXT.get(mission.state)
        if initial_status is not None:
            status_client.send_status(initial_status, mission_state=mission.state)

    server = TriggerServer(host="0.0.0.0", port=PORT)
    server.start()

    plt.ion()
    fig, (ax, ax_lidar) = plt.subplots(1, 2, figsize=(13, 6.5))
    fig.canvas.manager.set_window_title("Mission: Defect Delivery")

    running = {"value": True}

    def on_key(event) -> None:
        if (event.key or "").lower() == "q":
            running["value"] = False

    fig.canvas.mpl_connect("key_press_event", on_key)

    true_trail: list[tuple[float, float]] = []
    est_trail: list[tuple[float, float]] = []
    # None으로 시작해야 첫 루프에서 "전이"로 감지되어 초기 state의 target으로 set_goal()이
    # 걸림 (RETURN_TO_RECEIVING으로 시작하는 경우 등) -- mission.state로 미리 채우면 첫
    # set_goal() 호출 자체가 스킵되어 로봇이 아예 움직이지 않음. 상태 전송 중복은 아래
    # 루프에서 last_mission_state is not None으로 따로 막음 (시작 상태는 위에서 이미 전송).
    last_mission_state = None
    all_paths: list[list[tuple[float, float]]] = []
    leg_phase = "TRACKING"
    align_heading = 0.0
    previous = time.monotonic()
    last_print_time = 0.0

    try:
        while running["value"] and plt.fignum_exists(fig.number):
            now = time.monotonic()
            dt = min(0.15, max(0.0, now - previous))
            previous = now

            for message in server.poll():
                if message.get("event") == "box_placed":
                    mission.on_box_placed()

            mission.tick()

            if mission.state != last_mission_state:
                target = mission.target_zone()
                if target is not None:
                    sim.set_goal(*target)
                    all_paths.append(list(sim.auto_path))
                    align_heading = math.atan2(target[1] - sim.est_pose.y, target[0] - sim.est_pose.x)
                    leg_phase = "ALIGNING"
                    sim.auto_on = False
                else:
                    sim.auto_on = False
                    sim.cmd = (0.0, 0.0)
                if status_client is not None and last_mission_state is not None:
                    status = STATUS_TEXT.get(mission.state)
                    if status is not None:
                        status_client.send_status(status, mission_state=mission.state)
                last_mission_state = mission.state

            if leg_phase == "ALIGNING":
                left, right, aligned = align_to_heading_command(sim.est_pose, align_heading)
                sim.cmd = (left, right)
                if aligned:
                    leg_phase = "TRACKING"
                    sim.auto_on = True

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
            for i, (sx, sy, ex, ey) in enumerate(OBSTACLE):
                ax.plot([sx, ex], [sy, ey], color="#E8590C", linewidth=5,
                        label="Obstacle (OMX)" if i == 0 else "_nolegend_")
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
            status_tag = f" | status={STATUS_TEXT.get(mission.state, '-')}"
            ax.set_title(
                f"Mission: {mission.state}{dwell_tag}{phase_tag}{status_tag} | {mode_tag} | "
                f"listening on :{PORT} | Q=quit"
            )
            ax.legend(loc="lower left", fontsize=8)

            draw_lidar_view(ax_lidar, scan)

            fig.canvas.draw_idle()
            plt.pause(0.03)
    finally:
        server.stop()
        if status_client is not None:
            status_client.stop()
        sim.robot.stop()

    plt.ioff()
    if plt.fignum_exists(fig.number):
        plt.show()


if __name__ == "__main__":
    main()
