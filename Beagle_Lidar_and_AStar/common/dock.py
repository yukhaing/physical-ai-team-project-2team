from __future__ import annotations

import math
import time

from common.geometry import wrap_angle
from common.mapping import DistanceField, localize_from_map
from common.scan_align import best_rotation_offset, estimate_pose_offset, mask_from_angle_range

# Loosened from 3.0 (2026-09-01): even with MAP_THETA_STEPS raised to 360
# (1deg/step, well finer than 3deg) below, find_pose_via_map() still showed
# some oscillation on real hardware chasing exactly 3deg -- turn_by_angle()
# isn't a perfectly precise physical pivot, so the live heading reading
# itself has some real jitter beyond pure grid quantization. Raised to 6.0
# for more margin against that same jitter.
REALIGN_TOL_DEG = 6.0
REALIGN_MAX_ITERS = 30
# Lowered from 10.0 (2026-09-02) to move slower/more precisely during
# alignment at a zone (find_pose()/find_pose_via_map()/realign_heading()) --
# separate from drive_with_localization()'s cruise speed (10%,
# common/navigate.py), which stays faster for the zone-to-zone leg itself.
# Slower turns should reduce the overshoot/skid that's been a repeated
# source of oscillation this session. Matched to POSITION_DRIVE_PERCENT
# below (8.0) so every alignment-phase move uses the same speed.
REALIGN_TURN_PERCENT = 7.5
# Rough estimate only (wheel_percent -> deg/s) -- exact rate doesn't matter much
# since every step re-measures the true heading with LiDAR afterward instead of
# trusting this number; it only affects how many iterations convergence takes.
REALIGN_TURN_RATE_DEG_PER_S = 35.0
# Tried raising this to 1.0 (2026-09-01) on the theory that residual
# motion/vibration right after a turn was corrupting the next scan -- one
# isolated scripts/04_find_pose.py run at defect did converge cleanly right
# after that change, but the SAME leg driven as part of the actual shuttle
# mission (scripts/10) kept landing on match_err 150-340mm regardless,
# settle time on top or not -- so settle time was not the real fix for that,
# just added real seconds per correction step for no consistent benefit.
# Kept modestly above the original 0.15 rather than a full revert, since a
# short pause after stopping is still cheap and can't hurt.
SETTLE_S = 0.3  # pause after stopping before trusting a scan

# turn_by_angle() is only gyro-verified while it's turning, with no LiDAR check
# until it's done -- for a single large continuous spin, real motor/wheel
# momentum (it doesn't stop instantly when commanded) turned out to add real
# overshoot (~14deg seen on a ~33deg commanded turn), not just small
# integration noise. Capping every single physical turn to this size and
# re-verifying via LiDAR before turning further keeps overshoot from
# accumulating -- proven reliable for the reference-heading case, and used
# below for turning toward ANY target heading, not just the reference's.
MAX_SINGLE_TURN_RAD = math.radians(REALIGN_TURN_RATE_DEG_PER_S * 0.5)
MAX_TURN_STEPS = 20

# The "orient toward position-correction target" turn inside find_pose() aims
# at an arbitrary intermediate heading, not the reference heading -- its only
# job is to point roughly the right way before a short drive; the *next*
# find_pose() iteration re-measures heading fresh against the reference
# afterward anyway. Confirmed 2026-08-31 (shuttle mission, GOTO_RECEIVING
# leg): reusing the tight REALIGN_TOL_DEG/MAX_TURN_STEPS here made it thrash
# for 17 steps, bouncing between wildly different `needed` values (e.g.
# +5/+25/+55/+70/+55/+60...), because match_err was elevated (33-90mm, since
# the robot was still meaningfully off-position) and best_rotation_offset()'s
# heading reading gets noisier as match_err rises -- chasing a noisy signal
# down to a tight tolerance just means chasing the noise. A looser tolerance
# and a hard cap on steps stop it from thrashing; residual heading error left
# over just shows up as a bit of lateral drift in the drive, which the outer
# find_pose() loop catches next iteration anyway.
POSITION_TURN_TOL_DEG = 7.5
POSITION_TURN_MAX_STEPS = 6

# Chasing 1cm needs several correction cycles, each requiring several small
# in-place turns to get there -- and on real hardware, in-place turning isn't a
# perfect pivot (wheel slip/skid), so each turn adds a bit of real, uncontrolled
# linear drift. That was accumulating faster than the position correction was
# fixing it (confirmed 2026-08-31: position error grew each correction cycle
# even when scripts/04b_measure_only.py showed a stable, repeatable reading
# with the robot not moving at all -- i.e. the measurement was trustworthy, the
# repeated turning to execute the correction was not). Loosening the tolerance
# means fewer correction cycles -- and fewer turns -- rather than chasing a
# precision this mechanism can't reliably deliver.
#
# Tightened to 0.02 (2026-09-01) now that find_pose_via_map() is the default
# alignment (see common/navigate.py's goto_zone()) instead of the find_pose()
# this was originally loosened for: find_pose_via_map() measures heading AND
# position jointly from the same wide-radius map search every iteration
# (match_err ~5-10mm in working real-hardware runs), not a linearized
# single-scan fit, so it isn't the same "turning to chase noise" failure
# mode. But 0.02 turned out too tight for a DIFFERENT reason, still real:
# find_pose_via_map()'s position-correction turn re-orients toward whatever
# ARBITRARY heading the correction needs, ignoring target heading entirely
# (see the branch below) -- confirmed 2026-09-01, a GOTO_RECEIVING run
# oscillated through all 8 iterations without ever landing position AND
# heading within tolerance at the same time, even though each individual
# match_err stayed a trustworthy 8-14mm throughout (i.e. genuinely an
# execution-precision problem, not a measurement one -- each turn-then-drive
# to fix position knocks heading off, then fixing heading knocks position
# off again). Backed off to 0.025 as a middle ground between that and the
# original 0.03 -- still real precision, less prone to this back-and-forth.
#
# Tightened again to 0.015 (2026-09-01): a converged run left a visible
# residual (pos=(0.330,0.340) vs target (0.35,0.34) -- 2.0cm, all of it along
# the heading-0deg/3-o'clock axis, none lateral). A residual that's purely
# along the target heading needs close to ZERO turn to correct (the
# correction_heading below comes out near the current heading already), so
# it doesn't carry the same oscillation risk the 0.02 case above did (that
# one needed a real reorientation). If oscillation reappears on a
# non-axis-aligned residual, back off toward 0.02-0.025 again rather than
# assuming this case generalizes.
POSITION_TOL_M = 0.015
# Lowered from 10.0 (2026-09-02) alongside REALIGN_TURN_PERCENT above, same
# reasoning -- slower, more precise short drives during position correction.
POSITION_DRIVE_PERCENT = 7.5

# estimate_pose_offset()'s linearization only holds for small rotation (roughly
# +/-10-15deg). If best_rotation_offset() (robust for any angle) says heading is
# off by more than this, do a coarse turn toward it first instead of trusting
# the joint estimate.
COARSE_ROTATION_THRESHOLD_DEG = 15.0
POSE_MAX_ITERS = 15
# On a correctly-placed real scan, best_rotation_offset()'s match_err is
# normally ~40-60mm (see scripts/03_calibrate_and_realign.py's working runs).
# Both best_rotation_offset() and estimate_pose_offset() assume the robot is
# close to where reference_scan was captured -- far enough away and the scan
# shape itself differs (not just rotated), so match_err stays high and the
# fitted (dtheta, dx, dy) become unreliable noise, not a real reading. Refuse
# to "correct" from there instead of chasing a number that isn't measuring
# what it looks like it's measuring. Raised from 0.10 (2026-08-31): a genuine,
# recoverable arrival (just past the end of a checkpointed drive leg) landed
# at 104mm -- barely over the old threshold -- while every confirmed-hopeless
# case seen that day was 140mm+. 0.12 still rejects those, not this.
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
    standalone by scripts/03_calibrate_and_realign.py. For heading+position
    together in one pass, see find_pose() below.

    `tol_deg` is overridable per call because REALIGN_TOL_DEG=3deg isn't
    always achievable: confirmed 2026-09-01 at the defect zone (match_err
    stuck around 80-100mm even with exclude_deg_range masking, versus
    receiving's clean ~40-50mm) -- current~ oscillated between roughly +10deg
    and -10deg for the full MAX_TURN_STEPS without ever settling inside 3deg,
    the same limit-cycle pattern find_pose()'s position-correction turn hit
    against noisy match_err (see POSITION_TURN_TOL_DEG above). Pass a looser
    `tol_deg` (see course_config.json's per-zone "heading_tol_deg") for a zone
    where matching is inherently noisier instead of chasing precision the
    measurement can't deliver there.

    Returns (converged, match_err_m)."""
    ok = _turn_to_heading(hw, reference_scan, 0.0, math.radians(tol_deg), mask=mask)
    scan = hw.scan()
    _, match_err = best_rotation_offset(scan, reference_scan, mask=mask)
    return ok, match_err


def find_pose(hw, reference_scan: list[float], max_iters: int = POSE_MAX_ITERS,
              mask: set[int] | None = None, sanity_match_err_m: float = SANITY_MATCH_ERR_M) -> bool:
    """Closed loop that finds and fixes heading AND position together: every
    iteration takes exactly ONE fresh LiDAR scan, compares it to
    `reference_scan`, and moves accordingly -- nothing from a previous
    iteration's turn/drive is trusted going forward, it's always re-measured.

    If heading is far off (more than COARSE_ROTATION_THRESHOLD_DEG), turns
    toward the reference heading first (via _turn_to_heading(), small
    LiDAR-verified steps) -- the joint linear estimate below only holds for
    small rotation. Once heading is roughly right, estimate_pose_offset() reads
    (dtheta, dx, dy) from that single scan; if position also needs fixing, this
    turns toward the (world-frame) direction the target is in -- again via
    _turn_to_heading(), so a big reorientation is done in small re-verified
    steps instead of one large untrusted gyro-only spin -- then drives the
    encoder-measured distance. Heading is left wherever it ends up afterward
    and gets re-measured/re-corrected fresh via LiDAR next iteration, rather
    than "returned" by an unverified dead-reckoning turn. Returns True if both
    heading and position land within tolerance within max_iters.

    `mask` excludes reference-frame ray indices (see
    scan_align.mask_from_angle_range()) from every comparison in this
    function -- for a zone where part of the scene isn't static (e.g. a
    nearby robot arm that moves between visits), so matching relies only on
    geometry that's actually fixed. See course_config.json's per-zone
    "exclude_deg_range".

    `sanity_match_err_m` gates both the coarse rot_match_err check below AND
    the fine estimate_pose_offset() residual -- SANITY_MATCH_ERR_M (120mm) by
    default, overridable per zone (course_config.json's "match_sanity_mm")
    for a zone whose baseline match quality is persistently worse. Confirmed
    2026-09-01 at defect: even a real, working partial correction (8.6cm
    position error successfully driven down to 4.4cm) left residual stuck
    around 150mm on every subsequent scan -- consistent across many repeated
    stationary reads, so not just noise -- meaning the 120mm default rejected
    a legitimately trustworthy, still-actionable estimate every iteration and
    the loop could never finish the last cm of correction."""
    heading_tol_rad = math.radians(REALIGN_TOL_DEG)
    coarse_tol_rad = math.radians(COARSE_ROTATION_THRESHOLD_DEG)

    for i in range(max_iters):
        time.sleep(SETTLE_S)
        scan = hw.scan()

        # Coarse heading correction always runs first, regardless of match_err
        # -- best_rotation_offset() searches every possible rotation and picks
        # whichever fits best, and that pick tends to stay roughly right even
        # with some position error thrown in (a position offset degrades match
        # quality at every rotation fairly evenly; a genuinely wrong rotation
        # shows up as much worse than the true one regardless). Confirmed
        # 2026-08-31: after a drive that landed ~4cm off (match_err
        # 129-143mm), the robot's heading had drifted to roughly -30deg from a
        # 180deg target and never got a chance to correct because the old code
        # gated heading behind the same sanity check as the position estimate
        # below (which genuinely does need a good match, due to its linear
        # approximation) -- so a position error alone was silently blocking
        # heading alignment too.
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

        # rot_match_err passing SANITY_MATCH_ERR_M above only means the ROTATION
        # search found a confident best-fit angle -- it doesn't guarantee the
        # separate linear (dtheta, dx, dy) fit here is trustworthy too (fewer
        # rays after masking makes this fit less constrained, so it can land on
        # a small-looking (dtheta, dx, dy) that doesn't actually explain the
        # scan well). Confirmed 2026-09-01: a deliberately WRONG placement at
        # defect still reported "converged" -- residual was printed but never
        # checked against anything, so a coincidentally-small fit with a bad
        # residual sailed through. Gate on residual too before trusting this
        # estimate for anything, including declaring done.
        if residual > sanity_match_err_m:
            print(f"  fit residual ({residual * 1000:.0f}mm) too high to trust this estimate -- "
                  "re-scanning instead of acting on it.")
            continue

        if abs(dtheta) <= heading_tol_rad and pos_err_m <= POSITION_TOL_M:
            print("[pose] heading and position both within tolerance, done.")
            return True

        if pos_err_m > POSITION_TOL_M:
            # dx, dy are in the reference scan's world frame (0 = reference
            # heading). Facing correction_heading and driving FORWARD reaches
            # the target, but so does facing the opposite direction and driving
            # BACKWARD -- pick whichever needs the smaller turn from the
            # current heading (-dtheta), since every turn on real hardware adds
            # real positional drift (imperfect in-place pivot), so minimizing
            # total rotation per correction matters as much as the direction
            # being right.
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



# Search radius/resolution for find_pose_via_map() below -- wider and finer
# than drive_with_localization()'s periodic in-transit checks
# (LOCALIZE_SEARCH_RADIUS_M=0.05, theta_steps=90 in common/navigate.py),
# which only need to catch small drift since the last correction. This is a
# one-shot "where am I, really" check at a zone, so it can afford to search
# wider (covers the 7-12cm errors seen in real find_pose() runs at defect)
# and doesn't run often enough for the extra cost per call to matter the way
# it would every drive tick.
MAP_POSITION_SEARCH_RADIUS_M = 0.15
# Raised from 90 to 360 (2026-09-01): 90 steps over the full 360deg circle is
# 4deg/step -- coarser than REALIGN_TOL_DEG's original 3deg default heading
# tolerance, so a converged search could only ever land on a candidate 4deg
# away from true, never within 3deg. Confirmed on real hardware: heading_err
# oscillated between exactly +4deg and -4deg (176/184, 356/4, etc. -- always
# a multiple of the 4deg grid) for many iterations without ever settling,
# and each extra "heading-only turn" iteration added real physical drift
# (in-place pivots aren't a perfect pivot), compounding rather than
# converging.
#
# Backed off to 180 (2026-09-01) now that REALIGN_TOL_DEG has since been
# raised to 6.0 -- 180 steps = 2deg/step, still comfortably (3x) finer than
# that tolerance, so no quantization-oscillation risk reappears; halves the
# per-call cost (~1.34s -> ~0.62s measured against real captured data),
# which matters since find_pose_via_map() typically takes a few iterations
# to converge.
#
# Backed off further to 90 (2026-09-02): with the heading-only turn now
# capped (MAP_MAX_SINGLE_TURN_RAD below) and REALIGN_TOL_DEG at 6.0, 90
# steps = 4deg/step is still comfortably (1.5x) finer than tolerance, and a
# full real-hardware mission (4 round trips, dynamic obstacles included)
# converged reliably at this resolution. Halves per-call search cost again
# (~0.62s -> ~0.33s) on top of the 360->180 cut -- matters more here than
# find_pose_via_map()'s target-heading precision does, since find_pose_via_map()
# typically needs 5-9 iterations per leg to converge and each one pays this cost.
MAP_THETA_STEPS = 90
# Caps a single turn_by_angle() call in find_pose_via_map()'s heading-only
# correction branch (see there for the full reasoning) -- confirmed
# 2026-09-02 that an uncapped 176deg single turn overshot by ~32deg on real
# hardware. 90deg is generous enough that most real corrections still take
# just one step, while a near-180deg correction takes two (each re-measured
# by the outer loop) instead of betting everything on one large,
# gyro-only-verified spin.
MAP_MAX_SINGLE_TURN_RAD = math.radians(90.0)


def find_pose_via_map(
    hw, distance_field: DistanceField, target_x: float, target_y: float, target_heading_rad: float,
    max_iters: int = 12, search_radius_m: float = MAP_POSITION_SEARCH_RADIUS_M, theta_steps: int = MAP_THETA_STEPS,
    heading_tol_rad: float | None = None, position_tol_m: float = POSITION_TOL_M,
) -> bool:
    """Alternative to find_pose() for heading+position alignment, built around
    common/mapping.py's localize_from_map() (wide-radius grid+360deg search
    against the frozen point-cloud map) instead of common/scan_align.py's
    estimate_pose_offset() (a fast linearized fit that only holds for small
    offsets from an already-known pose).

    Added 2026-09-01: find_pose() at the defect zone was observed to diverge,
    not converge -- position_err went 7.8 -> 4.7 -> 4.4 -> 6.9 -> 12.1cm
    across repeated correction attempts on real hardware, getting WORSE than
    where it started. estimate_pose_offset()'s linear model degrades as the
    true offset (position AND heading, which drifts a bit on every real
    turn) grows, and defect's large, noisy corrections kept landing outside
    where that linearization still holds. localize_from_map() carries no such
    assumption -- it's the same tool that reliably tracked the robot through
    ~40cm of continuous curved driving in drive_with_localization() -- so
    this uses it for the final zone alignment too, at both heading and
    position simultaneously from one search per iteration, no separate
    coarse-heading-then-linear-position phases needed. `distance_field` is
    the same frozen map (see scripts/08_build_map.py) drive_with_localization()
    uses, not a single captured reference scan -- no `mask`/exclude_deg_range
    needed either, since a non-static element like the OMX arm only
    contaminates a few of the many map points, not the whole comparison.

    Requires TWO CONSECUTIVE scans within tolerance before declaring done, not
    just one -- confirmed 2026-09-02: a mission run converged and reported
    position_err=1.0cm, but a fresh standalone check moments later (robot
    physically untouched in between) read 2.2cm at the very same spot --
    i.e. scan-to-scan measurement noise here is comparable to the tolerance
    itself, so a single lucky low reading isn't strong enough evidence on its
    own. The second confirming scan is taken without moving (a real
    correction move only happens after a check FAILS), so it's a genuine
    repeat measurement of the same physical pose, not a new target.

    `max_iters` raised 8 -> 12 (2026-09-02) alongside the heading-only turn
    cap above: a large initial correction now takes 2+ iterations to work
    through instead of 1, and reaching tolerance for the first time still
    needs a second confirming iteration on top of that -- 8 wasn't always
    enough headroom for both (confirmed: one run reached tolerance exactly on
    iteration 7/8, the last one, with no iteration left for the confirm).

    Returns True if it converges (twice in a row) within max_iters."""
    if heading_tol_rad is None:
        heading_tol_rad = math.radians(REALIGN_TOL_DEG)
    guess_x, guess_y = target_x, target_y
    consecutive_ok = 0
    for i in range(max_iters):
        time.sleep(SETTLE_S)
        scan = hw.scan()
        localized, match_err = localize_from_map(
            scan, distance_field, guess_x, guess_y,
            pos_search_radius_m=search_radius_m, theta_steps=theta_steps,
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
        else:
            # Cap the single commanded turn -- unlike the position branch
            # above (whose forward/backward choice keeps `needed` under
            # 90deg by construction), `dtheta` here can be up to 180deg, and
            # turn_by_angle() is gyro-only with no LiDAR check until it's
            # done. Confirmed 2026-09-02: a single 176deg commanded turn
            # landed ~32deg off target on real hardware, and that error then
            # needed its own correction, causing oscillation instead of
            # convergence. Capping means a very large correction takes 2+
            # outer-loop iterations instead of 1 -- each already re-scans
            # and re-measures from scratch, so this is a smaller, safer
            # version of the chunked-turn approach tried and reverted
            # earlier (that one added a whole extra re-verify sub-loop per
            # step; this just bounds what a single step can ask for).
            step = dtheta if abs(dtheta) <= MAP_MAX_SINGLE_TURN_RAD else math.copysign(MAP_MAX_SINGLE_TURN_RAD, dtheta)
            hw.turn_by_angle(step, turn_percent=REALIGN_TURN_PERCENT)
            print(f"  heading-only turn {math.degrees(step):+.1f}deg (needed {math.degrees(dtheta):+.1f}deg)")
            guess_x, guess_y = localized.x, localized.y  # position unchanged by a pure turn

    print("[pose-map] max iterations reached without converging.")
    return False
