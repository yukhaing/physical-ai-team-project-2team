from __future__ import annotations

import math

Scan = list[float]


def mask_from_angle_range(n: int, start_deg: float, end_deg: float) -> set[int]:
    """Ray indices covering the angular window [start_deg, end_deg] (in the
    reference scan's own frame -- 0deg = the direction the reference was
    captured facing, increasing counter-clockwise, same convention as
    common/lidar.py's simulate_scan()). Handles a window that wraps past
    +/-180deg. Used to exclude a scene element that moves (e.g. an OMX arm at
    a roughly-known bearing but variable reach/position) from scan matching,
    so alignment relies only on geometry that's actually static."""
    step_deg = 360.0 / n
    span = (end_deg - start_deg) % 360.0
    start_i = round(start_deg / step_deg)
    count = max(1, round(span / step_deg))
    return {(start_i + k) % n for k in range(count)}


def best_rotation_offset(scan: Scan, reference: Scan, mask: set[int] | None = None) -> tuple[float, float]:
    """How far the robot has rotated since `reference` was captured, found by
    comparing the two 360-degree LiDAR scans directly -- no wall/map geometry
    needed, just the two scans.

    Both scans are index-based (index 0 = straight ahead of the robot's heading
    at the moment it was captured, index increasing in the +theta direction --
    see common/lidar.py's simulate_scan()). Tries every circular shift between
    them and keeps the one with the lowest mean absolute difference: that shift
    is how many scan-steps the world has "rotated" in the new scan relative to
    the reference, which is exactly how much the robot itself rotated.

    `mask`, if given, is a set of reference-frame ray indices (see
    mask_from_angle_range()) to exclude from the comparison entirely -- for a
    scene element whose position isn't static (e.g. an OMX arm), so the match
    is judged only on geometry that can't have changed since `reference` was
    captured.

    Returns (rotation_rad, match_err_m):
      rotation_rad -- turn the robot by this much (in the +theta direction) to
      bring it back to the pose it was in when `reference` was captured.
      match_err_m -- residual mean-abs-difference at the best-fit shift (small
      = confident match, large = scans don't line up well at any rotation).
    """
    n = len(scan)
    if n == 0 or len(reference) != n:
        raise ValueError("scan and reference must be the same non-zero length")
    indices = [i for i in range(n) if not mask or i not in mask]
    best_shift, best_err = 0, math.inf
    for k in range(n):
        err = sum(abs(scan[(i + k) % n] - reference[i]) for i in indices) / len(indices)
        if err < best_err:
            best_err = err
            best_shift = k
    rotation_rad = (best_shift / n) * 2.0 * math.pi
    if rotation_rad > math.pi:
        rotation_rad -= 2.0 * math.pi
    return rotation_rad, best_err


def estimate_translation_offset(
    scan: Scan, reference: Scan, max_range_m: float = 5.0, mask: set[int] | None = None
) -> tuple[float, float, float]:
    """How far (dx, dy), in the room's world frame, the robot has drifted from
    the pose `reference` was captured at -- assuming heading is already aligned
    (call this only after best_rotation_offset() is within tolerance).

    Unlike rotation, translation can't be found by trying shifts -- instead this
    uses a linear approximation: for a ray hitting a surface close to head-on,
    moving the robot by (dx, dy) changes that ray's measured range by
    approximately -(dx*cos(angle) + dy*sin(angle)), where angle is the ray's
    world-frame angle (valid because heading is assumed aligned, so robot-frame
    angle == world-frame angle here). Fitting that model to every ray via least
    squares gives (dx, dy) directly, using only the two scans -- no wall/map
    model needed.

    Rays where either scan is at max range (no real return, e.g. pointing out
    through a gap) are skipped -- the head-on-surface assumption doesn't hold
    for them and they would just add noise.

    Returns (dx, dy, residual_err_m): (dx, dy) is the robot's current position
    minus its position when `reference` was captured (drive by (-dx, -dy) to
    return to it); residual_err_m is the leftover mean-abs range error after
    accounting for that translation (large = fit is unreliable, e.g. too few
    usable rays or the mismatch isn't actually a small translation).
    """
    n = len(scan)
    if n == 0 or len(reference) != n:
        raise ValueError("scan and reference must be the same non-zero length")
    valid_cutoff = max_range_m - 0.05
    sum_cc = sum_cs = sum_ss = 0.0
    sum_c_dr = sum_s_dr = 0.0
    rows: list[tuple[float, float, float]] = []  # (cos, sin, delta_r) for residual calc
    for i in range(n):
        if mask and i in mask:
            continue
        r_cur, r_ref = scan[i], reference[i]
        if r_cur >= valid_cutoff or r_ref >= valid_cutoff:
            continue
        angle = i * 2.0 * math.pi / n
        c, s = math.cos(angle), math.sin(angle)
        delta_r = r_cur - r_ref
        sum_cc += c * c
        sum_cs += c * s
        sum_ss += s * s
        sum_c_dr += c * delta_r
        sum_s_dr += s * delta_r
        rows.append((c, s, delta_r))

    if len(rows) < 8:
        return 0.0, 0.0, math.inf  # too few usable rays to trust an estimate

    det = sum_cc * sum_ss - sum_cs * sum_cs
    if abs(det) < 1e-9:
        return 0.0, 0.0, math.inf  # rays too clustered in angle to separate dx from dy

    dx = (-sum_c_dr * sum_ss + sum_cs * sum_s_dr) / det
    dy = (-sum_cc * sum_s_dr + sum_cs * sum_c_dr) / det

    residual = sum(abs(delta_r + dx * c + dy * s) for c, s, delta_r in rows) / len(rows)
    return dx, dy, residual


def _solve_3x3(a: list[list[float]], b: list[float]) -> list[float] | None:
    """Cramer's rule for a 3x3 linear system a @ x = b. None if singular."""

    def det3(m: list[list[float]]) -> float:
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    d = det3(a)
    if abs(d) < 1e-12:
        return None
    x = []
    for col in range(3):
        m = [row[:] for row in a]
        for row in range(3):
            m[row][col] = b[row]
        x.append(det3(m) / d)
    return x


def estimate_pose_offset(
    scan: Scan, reference: Scan, max_range_m: float = 5.0, mask: set[int] | None = None
) -> tuple[float, float, float, float]:
    """Joint (dtheta, dx, dy) estimate from ONE scan comparison -- for small
    offsets (rotation within roughly +/-10deg; call best_rotation_offset() first
    and do a coarse turn if it's larger than that, this linearization doesn't
    hold for big rotations the way the circular-shift search does).

    Extends estimate_translation_offset()'s model with a rotation term: a small
    heading offset dtheta shifts each ray's angle slightly, which (for a locally
    smooth scan) changes its range by approximately -dtheta times the scan's
    angular derivative at that ray (estimated from `reference` via a central
    difference). Combined with the translation terms:

        r_obs(i) - r_ref(i) ~= -dtheta*dref/dtheta(i) - dx*cos(angle_i) - dy*sin(angle_i)

    Solving all three simultaneously via least squares means heading and
    position come from the SAME single scan comparison instead of being solved
    in separate passes -- no assumption that heading is already exactly right,
    just roughly (small-angle) right.

    Returns (dtheta_rad, dx, dy, residual_err_m) -- dtheta_rad is the corrective
    turn (same convention as best_rotation_offset()'s rotation_rad: turn the
    robot by this much, in the +theta direction, to undo the heading drift --
    NOT the drift itself, which is -dtheta_rad); (dx, dy) is the position drift
    as in estimate_translation_offset(); residual_err_m is the leftover mean-abs
    range error after accounting for all three (large = this scan doesn't fit a
    small combined offset well -- rotation may still be too big for this
    linearization, run best_rotation_offset() + a coarse turn first).
    """
    n = len(scan)
    if n == 0 or len(reference) != n:
        raise ValueError("scan and reference must be the same non-zero length")
    valid_cutoff = max_range_m - 0.05
    step = 2.0 * math.pi / n

    sum_aa = sum_ab = sum_ac = sum_bb = sum_bc = sum_cc = 0.0
    sum_ae = sum_be = sum_ce = 0.0
    rows: list[tuple[float, float, float, float]] = []  # (deriv, cos, sin, delta_r)
    for i in range(n):
        if mask and i in mask:
            continue
        r_cur, r_ref = scan[i], reference[i]
        r_prev, r_next = reference[(i - 1) % n], reference[(i + 1) % n]
        if r_cur >= valid_cutoff or r_ref >= valid_cutoff or r_prev >= valid_cutoff or r_next >= valid_cutoff:
            continue
        angle = i * step
        # d(reference)/d(angle) at this ray. If the robot's actual heading is
        # theta_cur away from the reference heading, r_obs(i) ~= r_ref(i +
        # theta_cur/step) ~= r_ref(i) + theta_cur * d(reference)/d(angle), i.e.
        # delta_r ~= theta_cur*a - dx*b - dy*c. We solve for the *corrective*
        # turn instead (dtheta = -theta_cur, matching best_rotation_offset()'s
        # convention -- turn the robot by the returned amount to fix it), which
        # is the same as solving delta_r ~= -dtheta*a - dx*b - dy*c for dtheta.
        a = (r_next - r_prev) / (2.0 * step)
        b, c = math.cos(angle), math.sin(angle)
        delta_r = r_cur - r_ref
        sum_aa += a * a
        sum_ab += a * b
        sum_ac += a * c
        sum_bb += b * b
        sum_bc += b * c
        sum_cc += c * c
        sum_ae += a * delta_r
        sum_be += b * delta_r
        sum_ce += c * delta_r
        rows.append((a, b, c, delta_r))

    if len(rows) < 12:
        return 0.0, 0.0, 0.0, math.inf  # too few usable rays to trust an estimate

    solution = _solve_3x3(
        [[sum_aa, sum_ab, sum_ac], [sum_ab, sum_bb, sum_bc], [sum_ac, sum_bc, sum_cc]],
        [-sum_ae, -sum_be, -sum_ce],
    )
    if solution is None:
        return 0.0, 0.0, 0.0, math.inf
    dtheta, dx, dy = solution

    residual = sum(abs(delta_r + dtheta * a + dx * b + dy * c) for a, b, c, delta_r in rows) / len(rows)
    return dtheta, dx, dy, residual
