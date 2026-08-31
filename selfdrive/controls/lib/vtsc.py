"""
Vision Turn Speed Control (VTSC) — SHADOW-MODE-ONLY curve-speed estimator.

This module computes a "would-be" comfortable curve-entry speed target from the
driving model's predicted yaw-rate / velocity trajectory. It is a PURE, log-only
estimator: NOTHING in this module actuates. The caller LOGS the returned target
and never feeds it into v_cruise, the MPC, accel clips, or any command.

Design invariants (from a safety review):
  1. ZERO ACTUATION. This file only computes and returns numbers to be logged.
  2. DECOUPLED lateral-accel budget. VTSC uses its own conservative
     VTSC_TARGET_LAT_ACCEL, deliberately NOT the reactive lateral-accel clamp
     MAX_LATERAL_ACCEL_NO_ROLL (4.5 m/s^2) in drive_helpers.py. Planning a
     curve at 4.5 m/s^2 assumes a harsher, less-comfortable corner and would
     systematically UNDER-slow (target too high). VTSC targets ride comfort, so
     it uses a lower budget.
  3. FAIL TOWARD DOING NOTHING. Any missing / short / NaN / low-confidence input
     yields the NO_CONSTRAINT sentinel (a very high speed), never a garbage-low
     number that would look like a spurious slow-down request.

Everything here takes plain python lists / numpy arrays. No cereal, no car
imports, so it is trivially unit-testable off-vehicle.
"""

import math

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Lateral-accel budget used to PLAN a curve entry speed.
#
# Deliberately a SEPARATE constant from drive_helpers.MAX_LATERAL_ACCEL_NO_ROLL
# (currently 5.0 m/s^2). That 5.0 is a *reactive* clamp on the curvature the lateral
# controller may command RIGHT NOW. This one is *predictive*: it answers "how fast may
# I still be going when I reach that corner?".
#
# Set to 4.5 at Caspar's request. Rationale/calibration:
#   - Higher budget  -> higher v_safe -> VTSC intervenes LESS (only on genuinely tight
#     corners). Lower budget -> slows more often, more nanny-ish.
#   - Reference point from real driving: the ECP loop ramp is R=66.35 m (curv 0.0151).
#     Caspar takes it comfortably at 55-60 km/h, which back-solves to ~3.5 m/s^2.
#     At 4.5 this yields v_safe = sqrt(4.5/0.0151) = 17.3 m/s = 62 km/h, i.e. ABOVE the
#     speed he actually uses -> VTSC would NOT intervene there. So 4.5 behaves as a
#     safety net for corners tighter/faster than he'd take naturally, not a comfort limiter.
#   - It sits just UNDER the 5.0 execution clamp, so VTSC always plans slightly inside
#     what the lateral controller can actually deliver. Do not raise it above that clamp.
VTSC_TARGET_LAT_ACCEL = 2.0  # m/s^2

# Sentinel meaning "no curve constraint" — a speed so high nothing would ever be
# reduced to it. Failing to this value guarantees VTSC never fabricates a slow-down.
NO_CONSTRAINT_V = 1.0e9  # m/s (effectively +inf, but JSON/log friendly)


def _percentile(sorted_vals, pct):
  """Linear-interpolation percentile over a PRE-SORTED ascending list. No numpy, so this
  module stays importable/testable anywhere. Matches numpy.percentile's default method."""
  if not sorted_vals:
    return None
  if len(sorted_vals) == 1:
    return sorted_vals[0]
  k = (len(sorted_vals) - 1) * (max(0.0, min(100.0, pct)) / 100.0)
  lo = int(math.floor(k))
  hi = int(math.ceil(k))
  if lo == hi:
    return sorted_vals[lo]
  return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


def _default_params():
  """Defaults so tests / callers can pass a partial dict (or None)."""
  return {
    "target_lat_accel":   VTSC_TARGET_LAT_ACCEL,  # m/s^2, comfort budget
    "lookahead_s":        6.0,    # only consider model points with t <= this
    "min_speed":          8.3,    # m/s (~30 km/h): below this VTSC does nothing
    "min_curvature":      0.002,  # 1/m: near-straight road -> ignore (R > 500 m)
    "max_std":            0.20,   # rad/s: discard yaw-rate points more uncertain than this
    "vel_floor":          1.0,    # m/s: floor used in curvature denominator (== MIN_SPEED)
    "curv_eps":           1e-6,   # guard against divide-by-zero in sqrt
    "curv_percentile":    97.0,   # use the Nth-percentile curvature, not the strict max.
                                  # sunnypilot/SCC-Vision does the same (np.percentile(...,97)):
                                  # a single noisy model point must not define the whole target.
                                  # Our max_std gate cannot be relied on for this because cereal
                                  # often leaves orientationRate.zStd empty.
  }


def _finite(x):
  try:
    return math.isfinite(float(x))
  except (TypeError, ValueError):
    return False


def compute_curve_speed_target(orientation_rate_z, velocity_x, t_idxs, stds=None, params=None):
  """
  PURE curve-speed estimator. LOG-ONLY — the caller must not actuate on this.

  Args:
    orientation_rate_z: list/array of predicted YAW RATE (rad/s) per future point
                        (modelV2.orientationRate.z).
    velocity_x:         list/array of predicted forward SPEED (m/s) per future point
                        (modelV2.velocity.x).
    t_idxs:             list/array of the time (s) of each future point
                        (ModelConstants.T_IDXS). Same length as the arrays above.
    stds:               OPTIONAL list/array of yaw-rate uncertainty (rad/s)
                        (modelV2.orientationRate.zStd). May be None or empty
                        (cereal often leaves it empty) -> treated as "unknown",
                        which does NOT block a target (we just can't gate on it).
    params:             OPTIONAL dict overriding _default_params().

  Returns a dict:
    {
      "v_target":          float,  # comfortable curve speed (m/s); NO_CONSTRAINT_V if none
      "would_reduce":      bool,   # v_target < NO_CONSTRAINT_V (a real constraint found)
      "valid":             bool,   # inputs were usable at all
      "limiting_index":    int,    # index of the tightest point (-1 if none)
      "limiting_curvature":float,  # |curvature| (1/m) at the limiting point (0 if none)
      "limiting_t":        float,  # t (s) of the limiting point (-1 if none)
      "min_v_safe":        float,  # == v_target (kept for clarity in logs)
      "limiting_std":      float,  # yaw-rate uncertainty at the limiting point
                                   #   (NaN if std unavailable) -- lets us later
                                   #   separate confident vs unconfident predictions
      "n_considered":      int,    # how many points passed all gates
      "reason":            str,    # short human tag for why (debug)
    }

  Failure philosophy: on ANY problem, return v_target = NO_CONSTRAINT_V with
  would_reduce=False. NEVER return a small number on bad data.
  """
  p = _default_params()
  if params:
    p.update(params)

  no_constraint = {
    "v_target":          NO_CONSTRAINT_V,
    "would_reduce":      False,
    "valid":             False,
    "limiting_index":    -1,
    "limiting_curvature": 0.0,
    "limiting_t":        -1.0,
    "min_v_safe":        NO_CONSTRAINT_V,
    "limiting_std":      float("nan"),
    "max_pred_lat_acc":  0.0,
    "n_considered":      0,
    "reason":            "no_constraint",
  }

  # --- Structural validation: fail toward doing nothing -------------------
  if orientation_rate_z is None or velocity_x is None or t_idxs is None:
    return {**no_constraint, "reason": "none_input"}

  z = list(orientation_rate_z)
  vx = list(velocity_x)
  t = list(t_idxs)

  n = min(len(z), len(vx), len(t))
  if n < 2:
    return {**no_constraint, "reason": "too_short"}

  have_std = stds is not None and len(stds) >= n
  s = list(stds) if have_std else None

  target_lat_accel = float(p["target_lat_accel"])
  lookahead_s      = float(p["lookahead_s"])
  curv_pct         = float(p["curv_percentile"])
  min_speed        = float(p["min_speed"])
  min_curv         = float(p["min_curvature"])
  max_std          = float(p["max_std"])
  vel_floor        = float(p["vel_floor"])
  curv_eps         = float(p["curv_eps"])

  # Minimum-speed floor: don't do anything below ~30 km/h.
  # Use the model's own current predicted speed (first point) as the "ego speed" proxy.
  v_now = vx[0]
  if not _finite(v_now):
    return {**no_constraint, "reason": "v_now_nan"}
  if v_now < min_speed:
    return {**no_constraint, "valid": True, "reason": "below_min_speed"}

  cand = []           # (abs_curv, lat_acc, idx, t, std) for every point that passes all gates
  saw_any_finite = False

  for i in range(n):
    ti = t[i]
    zi = z[i]
    vi = vx[i]

    # Skip non-finite points (fail toward nothing, per-point).
    if not (_finite(ti) and _finite(zi) and _finite(vi)):
      continue
    saw_any_finite = True

    # Lookahead window: only the next ~6 s matter for an approaching corner.
    if ti > lookahead_s:
      continue

    # Uncertainty gate (only if std is available for this point).
    std_i = float("nan")
    if have_std:
      std_i = s[i]
      if not _finite(std_i):
        continue
      if std_i > max_std:
        continue  # too unconfident -> discard this point

    # curvature = yaw_rate / speed ; floor the speed so we never blow up.
    denom = vi if vi > vel_floor else vel_floor
    abs_curv = abs(zi / denom)

    # Minimum-curvature floor: ignore near-straight road.
    if abs_curv < min_curv:
      continue

    # predicted lateral accel at this point = |yaw_rate| * speed
    cand.append((abs_curv, abs(zi) * denom, i, ti, std_i))

  if not saw_any_finite:
    return {**no_constraint, "reason": "all_nan"}

  if not cand:
    # Everything was straight / too-uncertain / out-of-window -> no reduction.
    return {**no_constraint, "valid": True, "reason": "no_tight_curve",
            "n_considered": 0}

  # PERCENTILE, not strict max: one bad model point must not define the target.
  curvs = sorted(c[0] for c in cand)
  lat_accels = sorted(c[1] for c in cand)
  curv_limit = _percentile(curvs, curv_pct)
  max_pred_lat_acc = _percentile(lat_accels, curv_pct)

  # Attribute the result to the real candidate closest to the percentile curvature,
  # so limiting_index / _t / _std still point at an actual model point.
  best = min(cand, key=lambda c: abs(c[0] - curv_limit))

  best_v = math.sqrt(target_lat_accel / max(curv_limit, curv_eps))

  return {
    "v_target":          float(best_v),
    "would_reduce":      True,
    "valid":             True,
    "limiting_index":    int(best[2]),
    "limiting_curvature": float(curv_limit),
    "limiting_t":        float(best[3]),
    "min_v_safe":        float(best_v),
    "limiting_std":      float(best[4]),
    "max_pred_lat_acc":  float(max_pred_lat_acc),
    "n_considered":      int(len(cand)),
    "reason":            "curve_found",
  }