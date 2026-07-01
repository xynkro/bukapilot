#!/usr/bin/env python3
"""
Standalone unit test for the PURE VTSC curve-speed estimator.

numpy only; NO car / cereal deps. Builds a synthetic constant-curvature corner and
asserts v_safe ~= sqrt(a_lat * R), plus edge cases (empty/short/NaN -> no-constraint
sentinel; straight road -> no reduction; below min speed -> no reduction; uncertainty
gate works).

Run:  python3 test_vtsc.py
"""

import math
import os
import sys

import numpy as np

# --- Import the module under test, working both in-repo and standalone --------
# In the repo the module lives at selfdrive/controls/lib/vtsc.py (this test lives
# at selfdrive/controls/tests/test_vtsc.py). For a standalone run we also allow
# vtsc.py to sit right next to this file. Neither path pulls in any car/cereal deps.
_HERE = os.path.dirname(os.path.abspath(__file__))
# candidate 1: sibling file (standalone scratch layout)
sys.path.insert(0, _HERE)
# candidate 2: ../lib relative to selfdrive/controls/tests (in-repo layout)
sys.path.insert(0, os.path.join(_HERE, os.pardir, "lib"))
try:
  from vtsc import compute_curve_speed_target, NO_CONSTRAINT_V, VTSC_TARGET_LAT_ACCEL  # noqa: E402
except ImportError:
  # in-repo package import as a last resort
  from openpilot.selfdrive.controls.lib.vtsc import (  # noqa: E402
    compute_curve_speed_target, NO_CONSTRAINT_V, VTSC_TARGET_LAT_ACCEL,
  )

# Quadratic time grid identical in shape to ModelConstants.T_IDXS (33 pts to 10 s).
IDX_N = 33
T_IDXS = [10.0 * ((i / 32.0) ** 2) for i in range(IDX_N)]

PASS = 0
FAIL = 0


def check(name, cond, extra=""):
  global PASS, FAIL
  if cond:
    PASS += 1
    print(f"  PASS  {name}")
  else:
    FAIL += 1
    print(f"  FAIL  {name}  {extra}")


def const_curvature_corner(radius_m, speed_ms, n=IDX_N, std=None):
  """A steady corner: yaw_rate = v / R at every point, constant forward speed."""
  yaw_rate = speed_ms / radius_m           # rad/s
  z = [yaw_rate] * n
  vx = [speed_ms] * n
  stds = None if std is None else [std] * n
  return z, vx, list(T_IDXS), stds


print("=" * 70)
print("VTSC pure-function tests")
print(f"VTSC_TARGET_LAT_ACCEL = {VTSC_TARGET_LAT_ACCEL} m/s^2")
print("=" * 70)

# ---------------------------------------------------------------------------
# 1. CORE MATH: 100 m radius corner at 25 m/s -> v_safe = sqrt(a_lat * R)
# ---------------------------------------------------------------------------
print("\n[1] Constant-curvature corner, R=100 m @ 25 m/s")
R = 100.0
V = 25.0
z, vx, t, stds = const_curvature_corner(R, V)
res = compute_curve_speed_target(z, vx, t, stds)
expected = math.sqrt(VTSC_TARGET_LAT_ACCEL * R)  # = sqrt(2.7*100) ~ 16.43 m/s
print(f"      v_target={res['v_target']:.4f}  expected={expected:.4f}  "
      f"reason={res['reason']}")
check("would_reduce is True", res["would_reduce"] is True)
check("v_safe ~= sqrt(a_lat*R)", abs(res["v_target"] - expected) < 1e-3,
      f"got {res['v_target']:.4f} want {expected:.4f}")
check("recovered curvature ~= 1/R", abs(res["limiting_curvature"] - 1.0 / R) < 1e-6,
      f"got {res['limiting_curvature']:.6f}")
check("limiting_t within lookahead", res["limiting_t"] <= 6.0 + 1e-9)

# Second radius to prove the sqrt relationship holds generally.
print("\n[1b] Constant-curvature corner, R=250 m @ 25 m/s")
z, vx, t, stds = const_curvature_corner(250.0, 25.0)
res = compute_curve_speed_target(z, vx, t, stds)
expected = math.sqrt(VTSC_TARGET_LAT_ACCEL * 250.0)
print(f"      v_target={res['v_target']:.4f}  expected={expected:.4f}")
check("v_safe ~= sqrt(a_lat*R) (R=250)", abs(res["v_target"] - expected) < 1e-3)

# ---------------------------------------------------------------------------
# 2. STRAIGHT ROAD -> no reduction (curvature under floor)
# ---------------------------------------------------------------------------
print("\n[2] Straight road (yaw_rate = 0)")
z = [0.0] * IDX_N
vx = [25.0] * IDX_N
res = compute_curve_speed_target(z, vx, list(T_IDXS), None)
print(f"      v_target={res['v_target']:.1f}  would_reduce={res['would_reduce']}  "
      f"reason={res['reason']}")
check("straight -> NO_CONSTRAINT_V", res["v_target"] == NO_CONSTRAINT_V)
check("straight -> would_reduce False", res["would_reduce"] is False)
check("straight -> valid True (ran, found nothing)", res["valid"] is True)

# ---------------------------------------------------------------------------
# 3. EDGE CASES -> no-constraint sentinel, never a garbage-low number
# ---------------------------------------------------------------------------
print("\n[3] Degenerate inputs -> no-constraint sentinel")

# empty
res = compute_curve_speed_target([], [], [], None)
check("empty lists -> sentinel", res["v_target"] == NO_CONSTRAINT_V and not res["would_reduce"])

# None
res = compute_curve_speed_target(None, None, None, None)
check("None inputs -> sentinel", res["v_target"] == NO_CONSTRAINT_V and not res["would_reduce"])

# too short (1 element)
res = compute_curve_speed_target([0.1], [25.0], [0.0], None)
check("length-1 -> sentinel", res["v_target"] == NO_CONSTRAINT_V and res["reason"] == "too_short")

# NaN everywhere in a real corner shape
z, vx, t, _ = const_curvature_corner(100.0, 25.0)
z = [float("nan")] * IDX_N
res = compute_curve_speed_target(z, vx, t, None)
check("all-NaN yaw -> sentinel", res["v_target"] == NO_CONSTRAINT_V and not res["would_reduce"],
      f"reason={res['reason']}")

# NaN speed at point 0 (v_now)
z, vx, t, _ = const_curvature_corner(100.0, 25.0)
vx[0] = float("nan")
res = compute_curve_speed_target(z, vx, t, None)
check("NaN v_now -> sentinel", res["v_target"] == NO_CONSTRAINT_V)

# A single NaN mid-array should be skipped, corner still detected.
z, vx, t, stds = const_curvature_corner(100.0, 25.0)
z[5] = float("nan")
res = compute_curve_speed_target(z, vx, t, stds)
check("single NaN skipped, corner still found", res["would_reduce"] is True,
      f"reason={res['reason']}")

# ---------------------------------------------------------------------------
# 4. MIN SPEED FLOOR -> below ~30 km/h do nothing
# ---------------------------------------------------------------------------
print("\n[4] Below min speed (5 m/s = 18 km/h)")
z, vx, t, stds = const_curvature_corner(100.0, 5.0)  # tight-ish but slow
res = compute_curve_speed_target(z, vx, t, stds)
print(f"      v_target={res['v_target']:.1f}  reason={res['reason']}")
check("below min speed -> no reduction", res["v_target"] == NO_CONSTRAINT_V and not res["would_reduce"])
check("below min speed -> valid True", res["valid"] is True)

# ---------------------------------------------------------------------------
# 5. UNCERTAINTY GATE -> high zStd discards points
# ---------------------------------------------------------------------------
print("\n[5] Uncertainty gate")
# Real corner but every point flagged high-uncertainty -> discarded -> no reduction.
z, vx, t, _ = const_curvature_corner(100.0, 25.0)
stds_hi = [5.0] * IDX_N  # way above max_std 0.20
res = compute_curve_speed_target(z, vx, t, stds_hi)
print(f"      high-std: v_target={res['v_target']:.1f}  reason={res['reason']}")
check("all high-std -> no reduction", res["v_target"] == NO_CONSTRAINT_V and not res["would_reduce"])

# Low std -> corner IS used, and limiting_std is recorded (survivorship-bias log).
z, vx, t, _ = const_curvature_corner(100.0, 25.0)
stds_lo = [0.01] * IDX_N
res = compute_curve_speed_target(z, vx, t, stds_lo)
check("low-std -> corner used", res["would_reduce"] is True)
check("limiting_std recorded", abs(res["limiting_std"] - 0.01) < 1e-9,
      f"got {res['limiting_std']}")

# std absent (empty list) -> we still find the corner, limiting_std is NaN.
z, vx, t, _ = const_curvature_corner(100.0, 25.0)
res = compute_curve_speed_target(z, vx, t, [])  # empty std => unavailable
check("empty std -> still finds corner", res["would_reduce"] is True)
check("empty std -> limiting_std is NaN", math.isnan(res["limiting_std"]))

# ---------------------------------------------------------------------------
# 6. TIGHTEST-WINS -> min across the window is picked
# ---------------------------------------------------------------------------
print("\n[6] Picks the TIGHTEST curve in the window")
# Straight, then a tightening bend inside the lookahead window.
z = [0.0] * IDX_N
vx = [25.0] * IDX_N
# points around t~2-4s get progressively tighter yaw rate
for i in range(IDX_N):
  if 2.0 <= T_IDXS[i] <= 4.0:
    z[i] = 25.0 / 80.0  # R=80 m -> tightest
res = compute_curve_speed_target(z, vx, list(T_IDXS), None)
expected = math.sqrt(VTSC_TARGET_LAT_ACCEL * 80.0)
print(f"      v_target={res['v_target']:.4f}  expected(R=80)={expected:.4f}  "
      f"t={res['limiting_t']:.2f}")
check("tightest (R=80) selected", abs(res["v_target"] - expected) < 1e-3)
check("limiting point inside bend window", 2.0 <= res["limiting_t"] <= 4.0)

# ---------------------------------------------------------------------------
# 7. LOOKAHEAD WINDOW -> a corner beyond ~6 s is ignored
# ---------------------------------------------------------------------------
print("\n[7] Corner beyond lookahead (t > 6 s) is ignored")
z = [0.0] * IDX_N
vx = [25.0] * IDX_N
for i in range(IDX_N):
  if T_IDXS[i] > 7.0:      # only far-future points curve
    z[i] = 25.0 / 60.0
res = compute_curve_speed_target(z, vx, list(T_IDXS), None)
print(f"      v_target={res['v_target']:.1f}  reason={res['reason']}")
check("far corner ignored -> no reduction", res["v_target"] == NO_CONSTRAINT_V and not res["would_reduce"])

# ---------------------------------------------------------------------------
print("\n" + "=" * 70)
print(f"RESULT: {PASS} passed, {FAIL} failed")
print("=" * 70)
sys.exit(1 if FAIL else 0)
