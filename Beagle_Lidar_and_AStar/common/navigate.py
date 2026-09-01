from __future__ import annotations

import math
import time

from common.dock import REALIGN_TOL_DEG, SANITY_MATCH_ERR_M, find_pose, find_pose_via_map, realign_heading
from common.geometry import Pose2D, wrap_angle
from common.lidar import Segment, rectangle_segments
from common.localize import checkpoint_correct
from common.mapping import DistanceField, localize_from_map
from common.motion import integrate_real_dead_reckoning
from common.obstacles import detect_obstacle_points, obstacle_rects_from_points
from common.planning import Point, Rect, astar_path, pure_pursuit_command, pure_pursuit_target
from common.scan_align import mask_from_angle_range

# How often (wall-clock seconds) drive_with_localization() pauses to correct
# against the fixed map via common/mapping.py's localize_from_map() --
# continuous odometry runs every control tick in between (integrate_real_dead_reckoning()),
# this just bounds how far it can drift before the next correction. Tuned
# 2026-08-31 once localize_from_map() got fast (~0.65s/call, DistanceField) --
# not practical at ~80s/call. Raised from 1.5 to 3.0 (2026-09-01) to cut the
# number of stop-scan-resume interruptions roughly in half for smoother
# driving -- real-hardware corrections at 1.5s intervals were consistently
# small (~1-5cm, ~1-5deg per checkpoint, see mission logs), so doubling the
# gap between corrections shouldn't let drift grow past what the 5cm search
# radius below can still recover.
LOCALIZE_INTERVAL_S = 3.0
LOCALIZE_SEARCH_RADIUS_M = 0.05  # only needs to cover drift since the LAST correction, not a cold search
LOCALIZE_THETA_STEPS = 90
# This room is a rectangle (180deg rotational symmetry) -- a full-circle
# theta search can occasionally lock onto the exact 180deg-flipped heading
# instead of the true one when the robot's out in open floor space, away
# from any single asymmetric feature (confirmed 2026-09-01: a live mission
# run's heading flipped ~170deg in one localize_from_map() call mid-drive,
# and the robot then drove on that wrong belief). Odometry only drifts a few
# degrees between corrections (see LOCALIZE_INTERVAL_S above), so it's a
# trustworthy-enough prior to search a window around instead of the full
# circle -- wide enough (90deg) to still catch a real, larger heading error,
# narrow enough that the 180deg-away candidate is never even considered.
LOCALIZE_THETA_WINDOW_DEG = 90.0

# Extra pause between drive_with_localization() finishing and find_pose()
# taking its first scan -- separate from (and on top of) common/dock.py's own
# SETTLE_S. Tried 2.0s (2026-09-01) on the theory the OMX arm might not be
# done settling into its resting position the instant the drive leg ends --
# but match_err at defect stayed just as bad (150-340mm) with the wait as
# without it, so that wasn't the real fix either. Kept short rather than
# removed entirely -- still cheap, and the actual fix needs to target the
# drive's own arrival position accuracy instead (see goto_zone()'s docstring
# discussion / this session's ongoing diagnosis), not scan timing.
ARRIVAL_SETTLE_S = 0.5

# Dynamic obstacle detection: this is the whole reason this session chose
# LiDAR+map+A* over plain dead-reckoning instead of just driving a fixed
# straight/L-shaped line between two known points -- a mapped-only obstacle
# list can't react to something new placed in the room after
# data/obstacle_map.json was built. Reuses the LiDAR scan already taken at
# each periodic localize_from_map() checkpoint above (no extra stop/scan) via
# common/obstacles.py's detect_obstacle_points() -- same function
# scripts/05_map_obstacles.py uses for the one-time static mapping, just
# pointed at (walls + already-known obstacles) instead of walls alone, so
# only genuinely NEW solid points get flagged, not the OMX arm base that's
# already in obstacles. Only reacts if the new obstacle actually blocks the
# path ahead (checked via _path_hits_rects() below) -- clutter off to the
# side of the route is ignored rather than triggering a needless replan.
NEW_OBSTACLE_CLEARANCE_M = 0.04
NEW_OBSTACLE_MIN_POINTS = 3
NEW_OBSTACLE_PATH_MARGIN_M = 0.05
PATH_SAMPLE_STEP_M = 0.01

WHEEL_BASE_M = 0.0956

# How often (meters of dead-reckoned travel) to pause and correct against the
# static map via common/localize.py's checkpoint_correct() -- keeps drift
# from accumulating unbounded over a long drive (confirmed 2026-08-31: dead
# reckoning alone drifted ~15cm+ over a single ~40cm continuous curve).
# Lowered from 0.15 the same day -- a 15cm gap between checkpoints let drift
# grow past checkpoint_correct()'s own 100mm sanity threshold before the next
# check could catch it (curved, continuously-varying wheel speeds drift faster
# per cm traveled than the straight/turn-only motions the encoder+gyro
# calibration constants were measured from).
CHECKPOINT_INTERVAL_M = 0.08
# Lowered from 0.15 to 0.1 (2026-09-01) to shave a bit off each stop-scan-resume
# pause during drive_with_localization()'s periodic map checkpoints -- smaller
# win than LOCALIZE_INTERVAL_S above, but free (this is just a wait before
# trusting the scan post-stop, not a measurement itself).
CHECKPOINT_SETTLE_S = 0.1

# Measured 2026-08-31 via scripts/00b_check_drive_direction.py: 10% wheel
# command for 1.0s produced ~3.35cm forward per wheel -- roughly 0.00335 m/s
# per percent of wheel command. This is a rough real-hardware calibration for
# CONTROL only -- pose tracking below uses real measured encoder distances
# every loop tick, not this conversion, so an imperfect calibration here
# doesn't bias the position estimate, only how smoothly it tracks the path.
MPS_PER_PERCENT = 0.00335
DRIVE_PERCENT_CAP = 15.0
CONTROL_DT_S = 0.15


def mps_to_percent(mps: float) -> float:
    percent = mps / MPS_PER_PERCENT
    return max(-DRIVE_PERCENT_CAP, min(DRIVE_PERCENT_CAP, percent))


TURN_STEP_RAD = math.radians(12.0)  # finer than common/dock.py's 18deg MAX_SINGLE_TURN_RAD --
# no LiDAR re-check between chunks here, so smaller steps matter more for
# controlling overshoot (confirmed 2026-08-31: even chunked at 18deg, a plain
# 90deg turn still contributed noticeably to a ~9cm drift over one leg).
TURN_SETTLE_S = 0.15


def turn_by_angle_chunked(hw, delta_rad: float, step_rad: float = TURN_STEP_RAD,
                           turn_percent: float = 10.0, settle_s: float = TURN_SETTLE_S) -> float:
    """Large in-place turn broken into several small gyro-integrated steps
    with a brief stop-and-settle between each, instead of one continuous
    high-momentum spin. A single continuous turn_by_angle() call was confirmed
    2026-08-31 to overshoot by a large fraction of its own commanded angle
    (~14deg off on a ~33deg turn, common/dock.py's find_pose()) -- and a plain
    90deg turn_by_angle() call in scripts/07_goto_defect_checkpointed.py's
    route (no chunking) was a likely contributor to a ~9cm position drift over
    just a 20.5cm leg the same day. Stopping between smaller chunks lets
    momentum settle instead of carrying into the next chunk. No LiDAR re-check
    between chunks here (there usually isn't a reference scan for an arbitrary
    intermediate pose) -- still gyro-only end to end, just chunked for less
    single-shot overshoot. Returns the gyro-integrated total turned."""
    remaining = delta_rad
    total = 0.0
    while abs(remaining) > math.radians(1.0):
        step = math.copysign(min(abs(remaining), step_rad), remaining)
        turned = hw.turn_by_angle(step, turn_percent=turn_percent)
        total += turned
        remaining = delta_rad - total
        time.sleep(settle_s)
    return total


def _checkpoint_step(hw, pose: Pose2D, checkpoint_segments: list[Segment]) -> Pose2D:
    """Stop, scan, correct `pose` against the static map (checkpoint_correct()),
    print what happened, and return the (possibly corrected) pose."""
    time.sleep(CHECKPOINT_SETTLE_S)
    scan = hw.scan()
    corrected, match_err, trustworthy = checkpoint_correct(scan, pose, checkpoint_segments)
    if trustworthy:
        dx_cm = (corrected.x - pose.x) * 100.0
        dy_cm = (corrected.y - pose.y) * 100.0
        dtheta_deg = math.degrees(wrap_angle(corrected.theta - pose.theta))
        print(f"    [checkpoint] correction=({dx_cm:+.1f},{dy_cm:+.1f})cm,{dtheta_deg:+.1f}deg "
              f"match_err={match_err * 1000:.0f}mm")
        return corrected
    print(f"    [checkpoint] match_err={match_err * 1000:.0f}mm too high -- keeping dead-reckoning estimate")
    return pose


def turn_to_heading_checkpointed(
    hw, target_theta_rad: float, pose: Pose2D, checkpoint_segments: list[Segment], turn_percent: float = 10.0,
) -> Pose2D:
    """Turn in place from `pose`'s heading to the absolute `target_theta_rad`
    (room frame), then verify/correct against the static map -- not a
    captured reference scan (see common/localize.py's checkpoint_correct()),
    since there isn't one for an arbitrary intermediate pose."""
    delta = wrap_angle(target_theta_rad - pose.theta)
    hw.turn_by_angle(delta, turn_percent=turn_percent)
    pose = Pose2D(pose.x, pose.y, target_theta_rad)
    print(f"  [turn] -> heading={math.degrees(target_theta_rad):.1f}deg (commanded)")
    return _checkpoint_step(hw, pose, checkpoint_segments)


def drive_straight_checkpointed(
    hw, distance_m: float, pose: Pose2D, checkpoint_segments: list[Segment],
    forward_percent: float = 10.0, checkpoint_interval_m: float = CHECKPOINT_INTERVAL_M,
) -> Pose2D:
    """Drive `distance_m` straight in `pose`'s current heading direction, in
    encoder-closed sub-segments of at most checkpoint_interval_m each,
    correcting against the static map between sub-segments (see
    common/localize.py's checkpoint_correct()). Individually more accurate
    than common/navigate.py's continuous pure-pursuit drive_path_real()
    (confirmed 2026-08-31: a curved trip drifted enough that checkpoints alone
    couldn't fully recover it -- straight-line driving and in-place turning
    are each individually much better-behaved than continuously varying wheel
    speeds). Returns the corrected pose estimate after the full leg."""
    remaining = distance_m
    while remaining > 1e-6:
        step = min(remaining, checkpoint_interval_m)
        driven = hw.drive_forward(step, forward_percent=forward_percent)
        pose = Pose2D(pose.x + driven * math.cos(pose.theta), pose.y + driven * math.sin(pose.theta), pose.theta)
        remaining -= step
        print(f"  [leg] pos=({pose.x:.3f},{pose.y:.3f}) heading={math.degrees(pose.theta):.1f}deg "
              f"remaining={remaining * 100:.1f}cm")
        pose = _checkpoint_step(hw, pose, checkpoint_segments)
    return pose


def drive_path_real(
    hw, path: list[Point], start_pose: Pose2D,
    speed_mps: float = 0.03, lookahead_m: float = 0.15, goal_tolerance_m: float = 0.03,
    max_time_s: float = 60.0, checkpoint_segments: list[Segment] | None = None,
    checkpoint_interval_m: float = CHECKPOINT_INTERVAL_M,
) -> Pose2D:
    """Continuously follow `path` (A* waypoints) using pure pursuit, tracking
    pose via real encoder+gyro dead-reckoning every control loop tick (see
    common/motion.py's integrate_real_dead_reckoning()) -- not the commanded
    wheel speeds.

    `checkpoint_segments` (walls + common/localize.py's obstacle_segments(),
    built once ahead of time -- never updated here, so this carries none of
    continuous SLAM's self-referential feedback-loop risk): if given, every
    `checkpoint_interval_m` of dead-reckoned travel the robot briefly stops,
    scans, and corrects its pose estimate against this static map via
    checkpoint_correct(), instead of letting dead-reckoning drift accumulate
    for the whole trip unchecked. Pass None to skip checkpoints entirely.

    Even with checkpoints this is not guaranteed pixel-perfect -- call
    common/dock.py's find_pose() at the destination afterward for the
    authoritative, LiDAR-corrected final pose.
    """
    max_wheel_mps = DRIVE_PERCENT_CAP * MPS_PER_PERCENT
    pose = start_pose
    hw.encoder_delta_m()  # reset the running baseline before starting to move
    start_time = time.monotonic()
    previous = start_time
    goal = path[-1]
    distance_since_checkpoint = 0.0
    try:
        while time.monotonic() - start_time < max_time_s:
            if pose.distance_to(*goal) <= goal_tolerance_m:
                break
            target = pure_pursuit_target(pose, path, lookahead_m)
            left_mps, right_mps = pure_pursuit_command(pose, target, speed_mps, max_wheel_mps, WHEEL_BASE_M)
            hw.wheels(mps_to_percent(left_mps), mps_to_percent(right_mps))
            time.sleep(CONTROL_DT_S)
            now = time.monotonic()
            dt = now - previous
            previous = now
            d_left, d_right = hw.encoder_delta_m()
            gyro_delta_rad = math.radians(hw.gyro_z_dps() * dt)
            pose = integrate_real_dead_reckoning(pose, d_left, d_right, gyro_delta_rad, WHEEL_BASE_M)
            distance_since_checkpoint += (d_left + d_right) / 2.0
            print(f"  [drive] pos=({pose.x:.3f},{pose.y:.3f}) heading={math.degrees(pose.theta):.1f}deg "
                  f"dist_to_goal={pose.distance_to(*goal) * 100:.1f}cm")

            if checkpoint_segments is not None and distance_since_checkpoint >= checkpoint_interval_m:
                hw.stop()
                time.sleep(CHECKPOINT_SETTLE_S)
                scan = hw.scan()
                corrected, match_err, trustworthy = checkpoint_correct(scan, pose, checkpoint_segments)
                if trustworthy:
                    dx_cm = (corrected.x - pose.x) * 100.0
                    dy_cm = (corrected.y - pose.y) * 100.0
                    dtheta_deg = math.degrees(wrap_angle(corrected.theta - pose.theta))
                    print(f"  [checkpoint] correction=({dx_cm:+.1f},{dy_cm:+.1f})cm,{dtheta_deg:+.1f}deg "
                          f"match_err={match_err * 1000:.0f}mm")
                    pose = corrected
                else:
                    print(f"  [checkpoint] match_err={match_err * 1000:.0f}mm too high -- "
                          "keeping dead-reckoning estimate")
                distance_since_checkpoint = 0.0
                previous = time.monotonic()  # exclude the stopped checkpoint time from the next dt
    finally:
        hw.stop()
    return pose


def _path_hits_rects(path: list[Point], rects: list[Rect], margin_m: float = 0.0,
                      sample_step_m: float = PATH_SAMPLE_STEP_M) -> bool:
    """True if any point sampled along `path`'s segments (every sample_step_m)
    falls inside (or within margin_m of) any rect in `rects`. Simple sampling
    instead of exact segment/rect intersection math -- path segments here are
    short (A* grid cell size, 2cm) and rects are obstacle-scale, so 1cm
    sampling reliably catches a real crossing without needing full
    computational geometry. Used both to decide whether a newly detected
    obstacle actually blocks the route (trigger to replan) and to sanity-check
    a freshly replanned path isn't astar_path()'s straight-line fallback
    cutting through something when no real route exists."""
    for (x1, y1), (x2, y2) in zip(path, path[1:]):
        length = math.hypot(x2 - x1, y2 - y1)
        steps = max(1, int(length / sample_step_m))
        for s in range(steps + 1):
            t = s / steps
            x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            for xmin, ymin, xmax, ymax in rects:
                if xmin - margin_m <= x <= xmax + margin_m and ymin - margin_m <= y <= ymax + margin_m:
                    return True
    return False


def drive_with_localization(
    hw, path: list[Point], start_pose: Pose2D, distance_field: DistanceField,
    speed_mps: float = 0.03, lookahead_m: float = 0.15, goal_tolerance_m: float = 0.03,
    max_time_s: float = 90.0, localize_interval_s: float = LOCALIZE_INTERVAL_S,
    localize_search_radius_m: float = LOCALIZE_SEARCH_RADIUS_M, localize_theta_steps: int = LOCALIZE_THETA_STEPS,
    boundary_w_m: float | None = None, boundary_h_m: float | None = None,
    known_obstacles: list[Rect] | None = None,
) -> tuple[Pose2D, bool]:
    """Continuous pure-pursuit drive with periodic map-based localization,
    matching this session's chosen architecture: continuous odometry
    (encoder+gyro dead-reckoning, integrate_real_dead_reckoning(), every
    control tick) PLUS periodic LiDAR localization against the fixed map
    (common/mapping.py's localize_from_map(), roughly every
    `localize_interval_s`) -- neither alone. The map (`distance_field`, see
    scripts/08_build_map.py) is built once ahead of time and never updated
    here, so this carries none of continuous SLAM's self-referential
    feedback-loop risk.

    `localize_search_radius_m` only needs to cover how far odometry could have
    drifted since the LAST correction (not a cold, room-wide search), which is
    what makes each periodic call fast enough to matter here.

    This gets the robot CLOSE, not exact -- call common/dock.py's find_pose()
    at the destination afterward (against a captured reference scan) for the
    final precise alignment; that is a deliberately separate, tighter step
    (see this session's architecture discussion), not something this function
    tries to do itself.

    If `boundary_w_m`/`boundary_h_m`/`known_obstacles` are given, each
    periodic localization checkpoint also reuses that same scan to look for a
    NEW obstacle (not already in `known_obstacles`) blocking the remaining
    path, and replans around it with astar_path() if so -- see the
    NEW_OBSTACLE_* constants above. Pass known_obstacles=None (the default) to
    skip this and keep the old fixed-path-only behavior. Returns
    (final_pose, blocked) -- blocked is True only if a new obstacle left no
    way through to the goal at all.
    """
    max_wheel_mps = DRIVE_PERCENT_CAP * MPS_PER_PERCENT
    pose = start_pose
    hw.encoder_delta_m()  # reset the running baseline before starting to move
    start_time = time.monotonic()
    previous = start_time
    last_localize = start_time
    goal = path[-1]
    obstacles = list(known_obstacles) if known_obstacles is not None else None
    try:
        while time.monotonic() - start_time < max_time_s:
            if pose.distance_to(*goal) <= goal_tolerance_m:
                break
            target = pure_pursuit_target(pose, path, lookahead_m)
            left_mps, right_mps = pure_pursuit_command(pose, target, speed_mps, max_wheel_mps, WHEEL_BASE_M)
            hw.wheels(mps_to_percent(left_mps), mps_to_percent(right_mps))
            time.sleep(CONTROL_DT_S)
            now = time.monotonic()
            dt = now - previous
            previous = now
            d_left, d_right = hw.encoder_delta_m()
            gyro_delta_rad = math.radians(hw.gyro_z_dps() * dt)
            pose = integrate_real_dead_reckoning(pose, d_left, d_right, gyro_delta_rad, WHEEL_BASE_M)
            print(f"  [odom] pos=({pose.x:.3f},{pose.y:.3f}) heading={math.degrees(pose.theta):.1f}deg "
                  f"dist_to_goal={pose.distance_to(*goal) * 100:.1f}cm")

            if now - last_localize >= localize_interval_s:
                hw.stop()
                time.sleep(CHECKPOINT_SETTLE_S)
                scan = hw.scan()
                localized, match_err = localize_from_map(
                    scan, distance_field, pose.x, pose.y,
                    pos_search_radius_m=localize_search_radius_m, theta_steps=localize_theta_steps,
                    theta_center=pose.theta, theta_range=math.radians(LOCALIZE_THETA_WINDOW_DEG),
                )
                dx_cm = (localized.x - pose.x) * 100.0
                dy_cm = (localized.y - pose.y) * 100.0
                dtheta_deg = math.degrees(wrap_angle(localized.theta - pose.theta))
                print(f"  [localize] pos=({localized.x:.3f},{localized.y:.3f}) "
                      f"heading={math.degrees(localized.theta):.1f}deg match_err={match_err * 1000:.0f}mm "
                      f"(odometry was off by {dx_cm:+.1f},{dy_cm:+.1f}cm,{dtheta_deg:+.1f}deg)")
                pose = localized
                last_localize = time.monotonic()
                previous = last_localize

                if obstacles is not None:
                    known_segments = (
                        rectangle_segments(0.0, 0.0, boundary_w_m, boundary_h_m)
                        + [seg for rect in obstacles for seg in rectangle_segments(*rect)]
                    )
                    new_points = detect_obstacle_points(pose, scan, known_segments,
                                                          clearance_m=NEW_OBSTACLE_CLEARANCE_M)
                    new_rects = obstacle_rects_from_points(new_points, min_points=NEW_OBSTACLE_MIN_POINTS)
                    if new_rects:
                        nearest_i = min(range(len(path)), key=lambda i: pose.distance_to(*path[i]))
                        remaining_path = path[nearest_i:]
                        if _path_hits_rects(remaining_path, new_rects, margin_m=NEW_OBSTACLE_PATH_MARGIN_M):
                            print(f"  [obstacle] {len(new_rects)} new obstacle(s) blocking the path -- replanning...")
                            obstacles = obstacles + new_rects
                            new_path = astar_path((pose.x, pose.y), goal, boundary_w_m, boundary_h_m, obstacles)
                            if _path_hits_rects(new_path, obstacles):
                                print("  [obstacle] no path around the new obstacle(s) -- stopping.")
                                hw.stop()
                                return pose, True
                            path = new_path
                            print(f"  [obstacle] replanned: {len(path)} new waypoints")
                        else:
                            print(f"  [obstacle] {len(new_rects)} new obstacle(s) detected, "
                                  "not on the remaining path -- ignoring.")
    finally:
        hw.stop()
    return pose, False


def goto_zone(
    hw, cfg: dict, distance_field: DistanceField, obstacles: list,
    from_name: str, to_name: str, to_reference_scan: list[float],
    dynamic_obstacles: bool = False, align: bool = False, align_map: bool = True,
    align_heading: bool = False,
) -> bool:
    """One full leg between two named zones in `cfg`'s "zones" (course_config.json):
    plans an A* path (avoiding `obstacles`, see data/obstacle_map.json), drives
    it with drive_with_localization() (continuous odometry + periodic map
    localization), then finishes with an alignment stage (see
    align/align_map/align_heading below) against `to_name`'s zone. Shared by
    scripts/09_goto_zone_slam.py and scripts/10_shuttle_mission.py so the
    drive-then-align sequence only lives in one place. Checked in this
    priority order if more than one is True: align, then align_map, then
    align_heading.

    align_map (common/dock.py's find_pose_via_map(), heading AND position via
    localize_from_map()'s wide-radius grid search against the frozen
    point-cloud map -- NOT `to_reference_scan`) is the default alignment
    (2026-09-01). Supersedes align_heading as the load-bearing default (every
    leg's start_pose below is taken from `to_name`'s CONFIGURED heading_deg,
    so leaving the robot's actual heading uncorrected breaks the NEXT leg's
    dead-reckoning, confirmed 2026-09-01 to drive it toward a room corner)
    AND gives real position precision align_heading alone doesn't. Replaced
    align as the default the same day: find_pose_via_map() converged in a
    single iteration at both zones (match_err ~5-6mm, 0cm position error) in
    real-hardware testing, where align (find_pose(), the linearized estimate)
    was observed to actively diverge at defect (position error growing
    7.8cm -> 12.1cm across repeated "corrections").

    align (common/dock.py's find_pose()) and align_heading
    (common/dock.py's realign_heading(), heading ONLY, no position) are kept
    for comparison/fallback, both default OFF. dynamic_obstacles also
    defaults OFF -- this is the last configuration confirmed working
    end-to-end on real hardware (plain A* over the static obstacle map +
    drive_with_localization). Re-enable explicitly via --dynamic-obstacles/
    --align on scripts 09/10:
      - dynamic_obstacles: caused a severe regression the same day -- a
        common/mapping.py localize_from_map() bug (a full-circle theta search
        can lock onto the exact 180deg-flipped heading in this rectangular,
        180deg-symmetric room) flipped the believed heading ~180deg mid-drive,
        and dynamic obstacle detection then trusted that corrupted pose,
        replanning repeatedly against phantom "obstacles" until it reported
        the path fully blocked. The 180deg bug itself was fixed the same day
        (localize_from_map() gained an optional theta_center/theta_range to
        search a window around a trusted heading prior instead of the full
        circle; drive_with_localization()'s own periodic checkpoint now
        passes its tracked odometry heading as that prior -- see
        LOCALIZE_THETA_WINDOW_DEG below), but dynamic_obstacles hasn't been
        re-tested against that fix yet, so it stays off until it has.

    `dynamic_obstacles=True` has drive_with_localization() also
    watch for a NEW obstacle (not in `obstacles`) blocking the path and
    replan around it live -- see drive_with_localization()'s docstring.
    Pass False to skip that and only avoid what's in `obstacles`.

    Returns whether the leg fully succeeded (the chosen alignment stage
    converged, or the drive alone if align/align_map/align_heading are all
    False -- AND no new obstacle left the robot with no way through)."""
    boundary = cfg["boundary"]
    from_zone = cfg["zones"][from_name]
    to_zone = cfg["zones"][to_name]

    path = astar_path(
        (from_zone["x_m"], from_zone["y_m"]), (to_zone["x_m"], to_zone["y_m"]),
        boundary["x_m"], boundary["y_m"], obstacles,
    )
    print(f"[plan] {from_name} -> {to_name}: {len(path)} waypoints")

    start_pose = Pose2D(from_zone["x_m"], from_zone["y_m"], math.radians(from_zone["heading_deg"]))

    print("[drive] continuous odometry + periodic map localization"
          + (" + dynamic obstacle avoidance..." if dynamic_obstacles else "..."))
    arrival_pose, blocked = drive_with_localization(
        hw, path, start_pose, distance_field,
        boundary_w_m=boundary["x_m"] if dynamic_obstacles else None,
        boundary_h_m=boundary["y_m"] if dynamic_obstacles else None,
        known_obstacles=obstacles if dynamic_obstacles else None,
    )
    if blocked:
        print(f"RESULT: blocked -- a new obstacle left no path to {to_name}")
        return False
    print(f"[drive] arrival pose (map-corrected) = ({arrival_pose.x:.3f}, {arrival_pose.y:.3f}) "
          f"heading={math.degrees(arrival_pose.theta):.1f}deg")

    exclude_deg_range = to_zone.get("exclude_deg_range")
    mask = mask_from_angle_range(len(to_reference_scan), *exclude_deg_range) if exclude_deg_range else None

    if align:
        time.sleep(ARRIVAL_SETTLE_S)
        sanity_match_err_m = to_zone.get("match_sanity_mm", SANITY_MATCH_ERR_M * 1000.0) / 1000.0
        print(f"[align] final precise alignment at {to_name} via find_pose() "
              f"(sanity={sanity_match_err_m * 1000:.0f}mm)...")
        converged = find_pose(hw, to_reference_scan, mask=mask, sanity_match_err_m=sanity_match_err_m)
        print("RESULT:", f"arrived and converged at {to_name}" if converged
              else f"drove to {to_name} but did NOT fully converge there -- see log above")
        return converged

    if align_map:
        time.sleep(ARRIVAL_SETTLE_S)
        print(f"[align] final precise alignment at {to_name} via find_pose_via_map()...")
        converged = find_pose_via_map(
            hw, distance_field, to_zone["x_m"], to_zone["y_m"], math.radians(to_zone["heading_deg"]),
        )
        print("RESULT:", f"arrived and converged at {to_name}" if converged
              else f"drove to {to_name} but did NOT fully converge there -- see log above")
        return converged

    if align_heading:
        time.sleep(ARRIVAL_SETTLE_S)
        heading_tol_deg = to_zone.get("heading_tol_deg", REALIGN_TOL_DEG)
        print(f"[align] heading-only realignment at {to_name} via realign_heading() "
              f"(tol={heading_tol_deg:.0f}deg)...")
        converged, match_err = realign_heading(hw, to_reference_scan, mask=mask, tol_deg=heading_tol_deg)
        print("RESULT:", f"arrived, heading aligned at {to_name} (match_err={match_err * 1000:.0f}mm)"
              if converged else
              f"arrived at {to_name} but heading did NOT converge (match_err={match_err * 1000:.0f}mm) "
              "-- see log above")
        return converged

    print(f"RESULT: arrived at {to_name} (align=False, align_heading=False -- using drive's own "
          "arrival pose as-is, heading NOT corrected)")
    return True
