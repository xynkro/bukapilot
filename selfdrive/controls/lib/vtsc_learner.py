"""
Learned lateral-acceleration budget for VTSC.

VTSC needs one number: how much lateral g is acceptable through a corner. That number is
personal -- it is how hard THIS driver is willing to corner -- and we were previously
guessing it (a fixed 4.5, back-solved from a single ramp).

This learns it instead, from the driver's own cornering, entirely on-device. No network,
no map data, no uploads: samples live in openpilot Params under VtscCurvatureData.

Approach follows FrogPilot's CurveSpeedController (bucket by road curvature, record the
lateral accel the DRIVER chose, take a high percentile) with one deliberate change:

  FrogPilot keeps a CUMULATIVE mean --  avg = ((avg * n) + x) / (n + 1)
  As n grows each new sample moves the average less, so after a few thousand corners it is
  effectively frozen and stops tracking. We cap the effective sample count (DECAY_N) so a
  new sample always carries at least 1/DECAY_N weight and the estimate keeps adapting.

Everything fails toward the fixed fallback: bad inputs, too few samples, or a corrupt
param all yield DEFAULT_LAT_ACCEL rather than a garbage budget.
"""

import json

# Only learn from the driver's OWN speed choice, so we require openpilot to NOT be
# driving longitudinally. Caspar drives on ACC most of the time, so samples accrue
# slowly -- that is expected, and why the fallback has to be sane on its own.
DEFAULT_LAT_ACCEL = 4.5   # m/s^2, used until enough has been learned

CURV_MIN = 0.002          # 1/m (R = 500 m) -- below this it is not a corner
CURV_MAX = 0.050          # 1/m (R = 20 m)  -- above this is a car park, not a road
CURV_STEP = 0.002         # bucket width -> 24 buckets

MIN_SPEED = 11.0          # m/s (~40 km/h): slow corners tell us nothing about highway comfort
MIN_LAT_ACCEL = 0.5       # m/s^2: ignore near-straight drift
MAX_LAT_ACCEL = 8.0       # m/s^2: reject implausible samples outright

DECAY_N = 40              # effective sample cap per bucket -> keeps adapting
PERCENTILE = 90.0         # of the per-bucket averages
MIN_BUCKETS = 4           # need this many populated buckets before trusting the estimate
MIN_SAMPLES = 20          # and this many samples in total

# Never let a learned value wander somewhere unsafe, however odd the data.
#
# LEARNED_MAX must not exceed drive_helpers.MAX_LATERAL_ACCEL_NO_ROLL (currently 5.0):
# that clamp is what the lateral controller can actually EXECUTE. Planning a corner entry
# speed against a budget the car will never deliver systematically UNDER-slows -- the exact
# failure the module docstring in vtsc.py warns about. Keep this <= the execution clamp.
LEARNED_MIN = 2.5
LEARNED_MAX = 5.0


def _percentile(sorted_vals, pct):
  """Linear-interpolation percentile over a PRE-SORTED ascending list (no numpy)."""
  if not sorted_vals:
    return None
  if len(sorted_vals) == 1:
    return sorted_vals[0]
  k = (len(sorted_vals) - 1) * (max(0.0, min(100.0, pct)) / 100.0)
  lo, hi = int(k), min(int(k) + 1, len(sorted_vals) - 1)
  if lo == hi:
    return sorted_vals[lo]
  return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


class LatAccelLearner:
  """Bucketed, decaying-average learner for the VTSC lateral-accel budget."""

  def __init__(self, params=None, param_key="VtscCurvatureData"):
    self.params = params
    self.param_key = param_key
    self.buckets = {}          # "curv_key" -> {"avg": float, "n": int}
    self.budget = DEFAULT_LAT_ACCEL
    self.learned = False       # False => still using the fallback
    self.total_samples = 0
    self._dirty = False
    self._load()
    self._recompute()

  # ---------- persistence ----------
  def _load(self):
    if self.params is None:
      return
    try:
      raw = self.params.get(self.param_key)
      if not raw:
        return
      data = json.loads(raw)
      if not isinstance(data, dict):
        return
      for k, v in data.items():
        if isinstance(v, dict) and "avg" in v and "n" in v:
          avg, n = float(v["avg"]), int(v["n"])
          if MIN_LAT_ACCEL <= avg <= MAX_LAT_ACCEL and n > 0:
            self.buckets[k] = {"avg": avg, "n": n}
    except Exception:
      self.buckets = {}   # corrupt param -> start clean, never crash

  def save(self):
    """Call sparingly (not every cycle) -- Params writes hit flash."""
    if self.params is None or not self._dirty:
      return
    try:
      self.params.put_nonblocking(self.param_key, json.dumps(self.buckets))
      self._dirty = False
    except Exception:
      pass

  # ---------- learning ----------
  def update(self, v_ego, curvature, long_active, blinker, has_lead):
    """Feed one cycle. Records a sample only when the DRIVER is choosing the speed."""
    try:
      if long_active or blinker or has_lead:
        return                      # not the driver's own free-flow speed choice
      if v_ego is None or curvature is None or v_ego < MIN_SPEED:
        return

      abs_curv = abs(float(curvature))
      if not (CURV_MIN <= abs_curv <= CURV_MAX):
        return

      lat_accel = (float(v_ego) ** 2) * abs_curv
      if not (MIN_LAT_ACCEL <= lat_accel <= MAX_LAT_ACCEL):
        return

      key = f"{round(abs_curv / CURV_STEP) * CURV_STEP:.3f}"
      b = self.buckets.get(key)
      if b is None:
        self.buckets[key] = {"avg": lat_accel, "n": 1}
      else:
        # DECAYING mean: cap the effective count so recent driving keeps moving it
        n = min(b["n"] + 1, DECAY_N)
        b["avg"] += (lat_accel - b["avg"]) / n
        b["n"] = n
      self.total_samples += 1
      self._dirty = True
      self._recompute()
    except Exception:
      return   # never let learning break the planner

  def _recompute(self):
    """Budget = PERCENTILE of the per-bucket averages, clamped to a sane band."""
    try:
      usable = [b["avg"] for b in self.buckets.values() if b["n"] >= 2]
      samples = sum(b["n"] for b in self.buckets.values())
      if len(usable) < MIN_BUCKETS or samples < MIN_SAMPLES:
        self.budget = DEFAULT_LAT_ACCEL
        self.learned = False
        return
      val = _percentile(sorted(usable), PERCENTILE)
      if val is None:
        self.budget = DEFAULT_LAT_ACCEL
        self.learned = False
        return
      self.budget = float(max(LEARNED_MIN, min(LEARNED_MAX, val)))
      self.learned = True
    except Exception:
      self.budget = DEFAULT_LAT_ACCEL
      self.learned = False

  # ---------- introspection ----------
  def progress(self):
    """0.0-1.0, how far toward a trusted estimate."""
    usable = sum(1 for b in self.buckets.values() if b["n"] >= 2)
    samples = sum(b["n"] for b in self.buckets.values())
    return min(1.0, min(usable / MIN_BUCKETS, samples / MIN_SAMPLES))