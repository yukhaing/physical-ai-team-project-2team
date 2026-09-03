from __future__ import annotations

import math
import time

from common.geometry import wrap_angle
from common.mapping import DistanceField, localize_from_map
from common.scan_align import best_rotation_offset, estimate_pose_offset, mask_from_angle_range

# How close (in degrees) the measured heading must be to the target before a
# turn is considered "done". Real turns aren't perfectly precise, so a too-
# tight tolerance causes endless small corrections that never settle.
REALIGN_TOL_DEG = 6.5
REALIGN_MAX_ITERS = 30
# Wheel power (%) used for every turn during zone alignment
# (find_pose()/find_pose_via_map()/realign_heading()) -- kept low and slow
# to reduce turn overshoot, separate from drive_with_localization()'s faster
# cruise speed for the zone-to-zone drive itself.
REALIGN_TURN_PERCENT = 7.5
# Rough estimate of turn speed (wheel percent -> deg/s), only used to size
# MAX_SINGLE_TURN_RAD below -- doesn't need to be precise, since every turn
# is re-measured with LiDAR afterward instead of trusted blindly.
REALIGN_TURN_RATE_DEG_PER_S = 35.0
# Pause after stopping, before trusting a fresh scan -- lets the robot settle
# physically first.
SETTLE_S = 0.15

# turn_by_angle() only checks the gyro while turning, not LiDAR -- a single
# large turn can overshoot due to motor/wheel momentum. Capping every
# physical turn to this size and re-checking with LiDAR before turning
# further (see _turn_to_heading() below) keeps overshoot from building up.
MAX_SINGLE_TURN_RAD = math.radians(REALIGN_TURN_RATE_DEG_PER_S * 0.5)
MAX_TURN_STEPS = 20

# find_pose()'s "turn toward position target" step only needs to point
# roughly the right way before a short drive -- the next iteration
# re-measures heading fresh against the reference anyway. A loose tolerance
# and a step cap stop it from chasing noise in a poor-quality match.
POSITION_TURN_TOL_DEG = 7
POSITION_TURN_MAX_STEPS = 6

# How close (in meters) the estimated position must be to the target before
# a correction move is considered "done". In-place turns aren't a perfect
# pivot on real hardware, so each turn adds a little real drift -- a looser
# tolerance means fewer correction cycles (and fewer turns) rather than
# chasing precision the hardware can't reliably deliver.
POSITION_TOL_M = 0.02
# Wheel power (%) for short drives during position correction -- kept low,
# same reasoning as REALIGN_TURN_PERCENT above.
POSITION_DRIVE_PERCENT = 7

# estimate_pose_offset()'s linear model only holds for small rotations
# (roughly +/-10-15deg). If heading is off by more than this, find_pose()
# turns toward the reference heading first instead of trusting that estimate.
COARSE_ROTATION_THRESHOLD_DEG = 15.0
POSE_MAX_ITERS = 15
# Minimum match quality (meters) required before find_pose() trusts a
# position/heading estimate at all. Both best_rotation_offset() and
# estimate_pose_offset() assume the robot is close to where reference_scan
# was captured -- too far away and the fitted values become unreliable
# noise, not a real reading, so refuse to act on them rather than chase a
# number that isn't measuring what it looks like it's measuring.
SANITY_MATCH_ERR_M = 0.12


def _turn_to_heading(hw, reference_scan: list[float], target_heading_rad: float, tol_rad: float,
                      max_steps: int = MAX_TURN_STEPS, mask: set[int] | None = None) -> bool:
    """Rotate in place, LiDAR-verifying after every step, until the robot faces
    `target_heading_rad` (absolute, in the reference scan's frame -- 0 = the
    heading the reference scan was captured at) within `tol_rad`. Every physical
    turn is capped to MAX_SINGLE_TURN_RAD and re-checked via best_rotation_offset()
    (robust for any angle, not just small ones) before turning further -- never
    trusts a single large gyro-only spin. `mask` excludes ray indices from the
    comparison (see scan_align.mask_from_angle_range()) -- for a zone where
    part of the scene isn't static. Returns True if it converges."""
    for _ in range(max_steps):
        time.sleep(SETTLE_S)
        scan = hw.scan()
        rotation_to_ref, match_err = best_rotation_offset(scan, reference_scan, mask=mask)
        current_heading = -rotation_to_ref  # best_rotation_offset's convention: rotation_to_ref = -current_heading
        needed = wrap_angle(target_heading_rad - current_heading)
        print(f"  [turn] current~={math.degrees(current_heading):+.1f}deg target={math.degrees(target_heading_rad):+.1f}deg "
              f"needed={math.degrees(needed):+.1f}deg match_err={match_err * 1000:.1f}mm")
        if abs(needed) <= tol_rad:
            return True
        step = needed if abs(needed) <= MAX_SINGLE_TURN_RAD else math.copysign(MAX_SINGLE_TURN_RAD, needed)
        hw.turn_by_angle(step, turn_percent=REALIGN_TURN_PERCENT)
    return False


def realign_heading(hw, reference_scan: list[float], mask: set[int] | None = None,
                     tol_deg: float = REALIGN_TOL_DEG) -> tuple[bool, float]:
    """Rotate in place until the live scan's heading matches reference_scan's
    within `tol_deg` (REALIGN_TOL_DEG by default). Heading-only; used
    standalone by scripts/03_calibrate_and_realign.py, and by goto_zone()'s
    --align-heading fallback. For heading+position together, see find_pose()
    below (or find_pose_via_map(), the actual default alignment method).

    `tol_deg` is overridable per zone because some zones have noisier scan
    matching than others (e.g. a nearby object that isn't perfectly static)
    -- chasing a tight tolerance against that noise just causes endless
    small corrections that never settle. See course_config.json's per-zone
    "heading_tol_deg".

    Returns (converged, match_err_m)."""
    ok = _turn_to_heading(hw, reference_scan, 0.0, math.radians(tol_deg), mask=mask)
    scan = hw.scan()
    _, match_err = best_rotation_offset(scan, reference_scan, mask=mask)
    return ok, match_err


def find_pose(hw, reference_scan: list[float], max_iters: int = POSE_MAX_ITERS,
              mask: set[int] | None = None, sanity_match_err_m: float = SANITY_MATCH_ERR_M) -> bool:
    """Older heading+position alignment method: fits (dtheta, dx, dy) from a
    single LiDAR scan against `reference_scan`, using a linearized model that
    only holds for small offsets. NOT the default -- goto_zone() uses
    find_pose_via_map() instead, since this linear fit was found to diverge
    (get worse, not better) on large real-hardware offsets. Kept for
    comparison via scripts/10's --align flag.

    Every iteration takes exactly one fresh scan and acts on it -- nothing
    from a previous turn/drive is trusted, it's always re-measured. If
    heading is far off (more than COARSE_ROTATION_THRESHOLD_DEG), it turns
    toward the reference heading first (via _turn_to_heading()), since the
    linear fit below only holds for small rotation. Once heading is roughly
    right, estimate_pose_offset() reads (dtheta, dx, dy) from the scan; if
    position also needs fixing, it turns toward the target direction and
    drives the encoder-measured distance. Heading is re-measured fresh next
    iteration rather than trusted from the turn. Returns True if both
    heading and position land within tolerance within max_iters.

    `mask` excludes reference-frame ray indices (see
    scan_align.mask_from_angle_range()) from every comparison -- for a zone
    where part of the scene isn't static (e.g. a nearby robot arm), so
    matching relies only on geometry that's actually fixed. See
    course_config.json's per-zone "exclude_deg_range".

    `sanity_match_err_m` gates whether an estimate is trustworthy enough to
    act on, for both the coarse rotation check and the fine position fit --
    SANITY_MATCH_ERR_M by default, overridable per zone via
    course_config.json's "match_sanity_mm" for a zone with persistently
    worse baseline match quality."""
    heading_tol_rad = math.radians(REALIGN_TOL_DEG)
    coarse_tol_rad = math.radians(COARSE_ROTATION_THRESHOLD_DEG)

    for i in range(max_iters):
        time.sleep(SETTLE_S)
        scan = hw.scan()

        # Heading correction runs first regardless of match quality --
        # best_rotation_offset() picks the best-fitting rotation even with
        # some position error present, so it stays roughly trustworthy even
        # before position is fixed. Gating heading behind the same sanity
        # check as position (below) would block heading correction whenever
        # position alone is bad, even though heading might already be
        # measurable.
        rotation_rad, rot_match_err = best_rotation_offset(scan, reference_scan, mask=mask)
        if abs(rotation_rad) > coarse_tol_rad:
            print(f"[pose {i}] heading far off (needed={math.degrees(rotation_rad):+.1f}deg, "
                  f"match_err={rot_match_err * 1000:.1f}mm) -- turning toward reference heading:")
            _turn_to_heading(hw, reference_scan, 0.0, coarse_tol_rad, mask=mask)
            continue

        # Heading is roughly aligned now -- only the FINE position estimate
        # below needs a good match (its linearization assumes small offsets).
        if rot_match_err > sanity_match_err_m:
            print(f"[pose {i}] heading aligned (match_err={rot_match_err * 1000:.0f}mm), but still too "
                  f"high to trust a position correction -- stopping before that; please physically "
                  f"place the robot closer to the zone and re-run for full convergence.")
            return False

        dtheta, dx, dy, residual = estimate_pose_offset(scan, reference_scan, mask=mask)
        pos_err_m = math.hypot(dx, dy)
        print(f"[pose {i}] heading_err={math.degrees(dtheta):+.1f}deg "
              f"position_err=({dx * 100:+.1f},{dy * 100:+.1f})cm={pos_err_m * 100:.1f}cm "
              f"residual={residual * 1000:.1f}mm")

        # Passing the rotation check above only means best_rotation_offset()
        # found a confident angle -- it doesn't guarantee the separate
        # linear (dtheta, dx, dy) fit here is trustworthy too (fewer rays
        # after masking makes this fit less constrained). Gate on residual
        # before trusting or acting on this estimate, including declaring
        # done.
        if residual > sanity_match_err_m:
            print(f"  fit residual ({residual * 1000:.0f}mm) too high to trust this estimate -- "
                  "re-scanning instead of acting on it.")
            continue

        if abs(dtheta) <= heading_tol_rad and pos_err_m <= POSITION_TOL_M:
            print("[pose] heading and position both within tolerance, done.")
            return True

        if pos_err_m > POSITION_TOL_M:
            # dx, dy are in the reference scan's world frame (0 = reference
            # heading). Facing correction_heading and driving forward reaches
            # the target, but so does facing the opposite way and driving
            # backward -- pick whichever needs less turning from the current
            # heading (-dtheta), since every turn on real hardware adds real
            # positional drift (imperfect in-place pivot).
            correction_heading = math.atan2(-dy, -dx)
            current_heading = -dtheta
            forward_needed = wrap_angle(correction_heading - current_heading)
            backward_needed = wrap_angle(correction_heading + math.pi - current_heading)
            drive_backward = abs(backward_needed) < abs(forward_needed)
            target_heading = wrap_angle(correction_heading + math.pi) if drive_backward else correction_heading
            print(f"  orienting toward position target ({math.degrees(target_heading):+.1f}deg, "
                  f"will drive {'backward' if drive_backward else 'forward'}):")
            oriented = _turn_to_heading(hw, reference_scan, target_heading,
                                         math.radians(POSITION_TURN_TOL_DEG),
                                         max_steps=POSITION_TURN_MAX_STEPS, mask=mask)
            if not oriented:
                print("  could not orient toward target -- skipping drive this round, re-scanning.")
                continue
            if drive_backward:
                driven = hw.drive_backward(pos_err_m, backward_percent=POSITION_DRIVE_PERCENT)
            else:
                driven = hw.drive_forward(pos_err_m, forward_percent=POSITION_DRIVE_PERCENT)
            print(f"  drove {driven * 100:.1f}cm toward target (heading re-checked next scan)")
        else:
            hw.turn_by_angle(dtheta, turn_percent=REALIGN_TURN_PERCENT)
            print(f"  moved: heading-only turn {math.degrees(dtheta):+.1f}deg")

    print("[pose] max iterations reached without converging.")
    return False


# Search radius for find_pose_via_map()'s position search -- wider than
# drive_with_localization()'s in-transit checks (common/navigate.py), since
# this is a one-shot "where am I, really" check at a zone, not a per-tick
# check of small drift since the last correction.
MAP_POSITION_SEARCH_RADIUS_M = 0.15
# Heading search resolution (steps across the search window below). Needs to
# be clearly finer than REALIGN_TOL_DEG, or a converged search can only ever
# land a full grid-step away from the true heading and never settle within
# tolerance.
MAP_THETA_STEPS = 180
# Caps a single turn_by_angle() call in find_pose_via_map()'s heading-only
# correction branch -- turn_by_angle() is gyro-only with no LiDAR check
# until it's done, and a large single turn can overshoot on real hardware.
# Capping means a big correction takes two or more outer-loop iterations
# (each re-measured from scratch) instead of betting everything on one
# large, unverified spin.
MAP_MAX_SINGLE_TURN_RAD = math.radians(90.0)

# Search window (see common/mapping.py's localize_from_map()
# theta_center/theta_range) for find_pose_via_map()'s own heading search. A
# full 360deg search can hop between several similarly-scoring heading
# candidates from call to call, even when match quality looks fine on all of
# them -- narrowing to a window around a trusted prior heading avoids that
# ambiguity. See find_pose_via_map()'s docstring for how the prior is chosen.
MAP_HEADING_WINDOW_DEG = 120.0


def find_pose_via_map(
    hw, distance_field: DistanceField, target_x: float, target_y: float, target_heading_rad: float,
    max_iters: int = 12, search_radius_m: float = MAP_POSITION_SEARCH_RADIUS_M, theta_steps: int = MAP_THETA_STEPS,
    heading_tol_rad: float | None = None, position_tol_m: float = POSITION_TOL_M,
    initial_heading_rad: float | None = None, heading_window_deg: float = MAP_HEADING_WINDOW_DEG,
) -> bool:
    """Default heading+position alignment method for goto_zone(), built
    around common/mapping.py's localize_from_map() (a grid search against
    the frozen point-cloud map) instead of find_pose()'s linearized single-
    scan fit, which was found to diverge (get worse, not better) on large
    real-hardware offsets. `distance_field` is the same frozen map (see
    scripts/08_build_map.py) drive_with_localization() uses, not a single
    reference scan -- no mask needed, since a non-static element like the
    OMX arm only contaminates a few of the many map points, not the whole
    comparison.

    Each iteration takes one scan, localizes against the map, and either
    turns+drives toward the target position or does a heading-only turn,
    depending on which is off. Requires TWO CONSECUTIVE scans within
    tolerance before declaring done (not just one) -- scan-to-scan
    measurement noise here is comparable to the tolerance itself, so a
    single lucky reading isn't strong enough evidence on its own. The
    second confirming scan is taken without moving.

    `initial_heading_rad`/`heading_window_deg`: search only
    `heading_window_deg` (120deg default) centered on a tracked heading
    prior, instead of the full circle -- a full-circle search can hop
    between several similarly-scoring heading candidates and never settle.
    Pass `initial_heading_rad` when a trustworthy prior exists (e.g.
    goto_zone() passes its own arrival_pose.theta, the drive phase's
    odometry-tracked heading) -- arrival heading can legitimately be far
    from the zone's target heading, so centering the window on the target
    instead would wrongly exclude the true heading. Leave it as None for a
    cold start with no prior (e.g. standalone testing) -- the search then
    falls back to the full circle instead of narrowing around a guess that
    might be wrong. The tracked prior updates every iteration to the
    heading just commanded (or measured, if nothing moved).

    Returns True if it converges (twice in a row) within max_iters."""
    if heading_tol_rad is None:
        heading_tol_rad = math.radians(REALIGN_TOL_DEG)
    guess_x, guess_y = target_x, target_y
    # Only narrow the search window when a real prior is given -- narrowing
    # around target_heading_rad with no real prior can exclude the true
    # heading entirely if the robot's actual heading is far from the target,
    # leaving the search wandering forever inside the wrong region.
    if initial_heading_rad is None:
        guess_theta = target_heading_rad
        heading_range_rad = 2.0 * math.pi
    else:
        guess_theta = initial_heading_rad
        heading_range_rad = math.radians(heading_window_deg)
    consecutive_ok = 0
    for i in range(max_iters):
        time.sleep(SETTLE_S)
        scan = hw.scan()
        localized, match_err = localize_from_map(
            scan, distance_field, guess_x, guess_y,
            pos_search_radius_m=search_radius_m, theta_steps=theta_steps,
            theta_center=guess_theta, theta_range=heading_range_rad,
        )
        dtheta = wrap_angle(target_heading_rad - localized.theta)
        dx = target_x - localized.x
        dy = target_y - localized.y
        pos_err_m = math.hypot(dx, dy)
        print(f"[pose-map {i}] pos=({localized.x:.3f},{localized.y:.3f}) "
              f"heading={math.degrees(localized.theta):.1f}deg match_err={match_err * 1000:.0f}mm "
              f"heading_err={math.degrees(dtheta):+.1f}deg position_err={pos_err_m * 100:.1f}cm")

        if abs(dtheta) <= heading_tol_rad and pos_err_m <= position_tol_m:
            consecutive_ok += 1
            if consecutive_ok >= 2:
                print("[pose-map] heading and position within tolerance on 2 consecutive checks, done.")
                return True
            print(f"  within tolerance ({consecutive_ok}/2 consecutive) -- re-scanning to confirm, no move.")
            guess_x, guess_y = localized.x, localized.y
            guess_theta = localized.theta
            continue
        consecutive_ok = 0

        if pos_err_m > position_tol_m:
            # World-frame direction from the robot's actual (localized)
            # position to the target -- facing it and driving forward reaches
            # the target, but so does facing away and driving backward; pick
            # whichever needs the smaller turn (every turn adds real
            # positional drift on real hardware), same reasoning as find_pose().
            correction_heading = math.atan2(dy, dx)
            forward_needed = wrap_angle(correction_heading - localized.theta)
            backward_needed = wrap_angle(correction_heading + math.pi - localized.theta)
            drive_backward = abs(backward_needed) < abs(forward_needed)
            needed = backward_needed if drive_backward else forward_needed
            hw.turn_by_angle(needed, turn_percent=REALIGN_TURN_PERCENT)
            if drive_backward:
                driven = hw.drive_backward(pos_err_m, backward_percent=POSITION_DRIVE_PERCENT)
            else:
                driven = hw.drive_forward(pos_err_m, forward_percent=POSITION_DRIVE_PERCENT)
            print(f"  drove {driven * 100:.1f}cm toward target (pose re-checked next scan)")
            guess_x, guess_y = target_x, target_y  # expect to be near the target now
            guess_theta = wrap_angle(localized.theta + needed)  # heading after the turn just commanded
        else:
            # Cap the single commanded turn -- unlike the position branch
            # above (whose forward/backward choice keeps `needed` under
            # 90deg by construction), `dtheta` here can be up to 180deg, and
            # a large single turn can overshoot on real hardware. Capping
            # means a large correction takes two or more iterations, each
            # re-measured from scratch, instead of betting everything on one
            # large, unverified spin.
            step = dtheta if abs(dtheta) <= MAP_MAX_SINGLE_TURN_RAD else math.copysign(MAP_MAX_SINGLE_TURN_RAD, dtheta)
            hw.turn_by_angle(step, turn_percent=REALIGN_TURN_PERCENT)
            print(f"  heading-only turn {math.degrees(step):+.1f}deg (needed {math.degrees(dtheta):+.1f}deg)")
            guess_x, guess_y = localized.x, localized.y  # position unchanged by a pure turn
            guess_theta = wrap_angle(localized.theta + step)  # heading after the turn just commanded

    print("[pose-map] max iterations reached without converging.")
    return False
