from __future__ import annotations

import math
import time

from common.dock import find_pose
from common.geometry import Pose2D, wrap_angle
from common.lidar import Segment
from common.localize import checkpoint_correct
from common.mapping import DistanceField, localize_from_map
from common.motion import integrate_real_dead_reckoning
from common.planning import Point, astar_path, pure_pursuit_command, pure_pursuit_target

# How often (wall-clock seconds) drive_with_localization() pauses to correct
# against the fixed map via common/mapping.py's localize_from_map() --
# continuous odometry runs every control tick in between (integrate_real_dead_reckoning()),
# this just bounds how far it can drift before the next correction. Tuned
# 2026-08-31 once localize_from_map() got fast (~0.65s/call, DistanceField) --
# not practical at ~80s/call.
LOCALIZE_INTERVAL_S = 1.5
LOCALIZE_SEARCH_RADIUS_M = 0.05  # only needs to cover drift since the LAST correction, not a cold search
LOCALIZE_THETA_STEPS = 90

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
CHECKPOINT_SETTLE_S = 0.15

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


def drive_with_localization(
    hw, path: list[Point], start_pose: Pose2D, distance_field: DistanceField,
    speed_mps: float = 0.03, lookahead_m: float = 0.15, goal_tolerance_m: float = 0.03,
    max_time_s: float = 90.0, localize_interval_s: float = LOCALIZE_INTERVAL_S,
    localize_search_radius_m: float = LOCALIZE_SEARCH_RADIUS_M, localize_theta_steps: int = LOCALIZE_THETA_STEPS,
) -> Pose2D:
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
    """
    max_wheel_mps = DRIVE_PERCENT_CAP * MPS_PER_PERCENT
    pose = start_pose
    hw.encoder_delta_m()  # reset the running baseline before starting to move
    start_time = time.monotonic()
    previous = start_time
    last_localize = start_time
    goal = path[-1]
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
    finally:
        hw.stop()
    return pose


def goto_zone(
    hw, cfg: dict, distance_field: DistanceField, obstacles: list,
    from_name: str, to_name: str, to_reference_scan: list[float],
) -> bool:
    """One full leg between two named zones in `cfg`'s "zones" (course_config.json):
    plans an A* path (avoiding `obstacles`, see data/obstacle_map.json), drives
    it with drive_with_localization() (continuous odometry + periodic map
    localization), then finishes with common/dock.py's find_pose() against
    `to_reference_scan` for the precise final position + heading. Shared by
    scripts/09_goto_zone_slam.py and scripts/10_shuttle_mission.py so the
    drive-then-align sequence only lives in one place. Returns whether
    find_pose() converged."""
    boundary = cfg["boundary"]
    from_zone = cfg["zones"][from_name]
    to_zone = cfg["zones"][to_name]

    path = astar_path(
        (from_zone["x_m"], from_zone["y_m"]), (to_zone["x_m"], to_zone["y_m"]),
        boundary["x_m"], boundary["y_m"], obstacles,
    )
    print(f"[plan] {from_name} -> {to_name}: {len(path)} waypoints")

    start_pose = Pose2D(from_zone["x_m"], from_zone["y_m"], math.radians(from_zone["heading_deg"]))

    print("[drive] continuous odometry + periodic map localization...")
    arrival_pose = drive_with_localization(hw, path, start_pose, distance_field)
    print(f"[drive] arrival pose (map-corrected) = ({arrival_pose.x:.3f}, {arrival_pose.y:.3f}) "
          f"heading={math.degrees(arrival_pose.theta):.1f}deg")

    print(f"[align] final precise alignment at {to_name} via find_pose()...")
    converged = find_pose(hw, to_reference_scan)
    print("RESULT:", f"arrived and converged at {to_name}" if converged
          else f"drove to {to_name} but did NOT fully converge there -- see log above")
    return converged
