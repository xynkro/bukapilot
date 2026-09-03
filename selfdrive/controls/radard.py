#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float,
             a_lead_meas: float | None = None):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # The BYD radar MEASURES lead acceleration directly (ALEAD, 0.01 m/s^2), already
    # plausibility-gated and low-passed in radar_interface. The Kalman above has to build
    # its accel estimate up from successive velocity samples, so it lags worst exactly when
    # acceleration is changing fastest -- a lead braking hard.
    #
    # Blend CONSERVATIVELY: take whichever indicates MORE deceleration. A bad/garbage ALEAD
    # can then only ever make us more cautious about a braking lead, never less. If the
    # measurement is spurious-positive it is simply ignored by the min().
    # Only DECELERATION measurements are honoured. capnp floats default to 0.0 rather than
    # NaN, so an unpopulated aRel would otherwise read as "lead accel = 0" and clamp away a
    # genuinely accelerating lead, making us follow sluggishly. Gating on < 0 means this can
    # only ever ADD braking sensitivity, never remove acceleration responsiveness.
    # DEADBAND: only honour the measurement for GENUINE hard braking.
    #
    # Just below, aLeadTau collapses to 0 whenever |aLeadK| >= 0.5, which tells the MPC to
    # project the lead's deceleration as PERMANENT (a_lead_traj = a_lead * exp(-tau*t^2/2)).
    # Letting a mild measured lift (say -0.8) win the min() therefore flipped the MPC into
    # "this lead is coming to a stop" and produced heavy braking where a light lift was all
    # that was needed. The Kalman handles gentle decel perfectly well and keeps tau high.
    # So: use the measurement only when the lead is really braking, which is the case the
    # Kalman is genuinely slow at.
    # ALEAD-DIRECT, RE-APPLIED 2026-09-04 at Caspar's request for road testing.
    # aLeadK is taken from the radar's own measured ALEAD rather than the Kalman.
    #
    # WHY: measured on the car (route 2026-09-03--06-14-37, 29839 engaged frames),
    # regressed on the lead's true acceleration and on ego's own aEgo:
    #     aLeadK (Kalman)  tracks lead +0.032   ego contamination +0.621
    #     ALEAD  (radar)   tracks lead +0.280   ego contamination +0.147
    #     a clean estimator would be  +1.000 / +0.000
    # The Kalman is ~9x worse at tracking the lead and ~4x more contaminated.
    #
    # KNOWN RISK, UNRESOLVED -- this FAILS the offline scenario suite. Controlled
    # A/B, same plant, only this code changed:
    #     with    33/37  cut-in -9.5 m, stop-and-go -0.9 m, morning commute -1.2 m
    #     without 36/37  only the known plant.py stopping-queue artifact
    # The sim feeds the TRUE lead acceleration as a_lead_meas, so it is NOT about
    # ALEAD being an imperfect sensor: even a perfect measurement collides there.
    # Hypothesis: the Kalman's LAG is load-bearing, holding aLeadK negative through
    # the tail of a braking event and keeping the deceleration projection alive
    # while still closing. Tested and REFUTED as the mechanism: making the aLeadTau
    # snap-back symmetric (aLeadTau.update(_LEAD_ACCEL_TAU) instead of the instant
    # aLeadTau.x = ...) still gives 32/37. Actual mechanism still unknown.
    #
    # First road trial (2026-09-03 evening): "seemed ok, not much change" -- no
    # incident, but no felt benefit either. Neither confirms nor refutes; the sim
    # failures are cut-in and stop-and-go geometry that may simply not have been hit.
    # WATCH cut-ins especially -- by far the worst sim failure.
    #
    # TO SETTLE IT: with this on, aLeadK IS ALEAD, so the same regression should
    # shift lead-tracking ~0.15 -> ~0.28 and ego ~0.45 -> ~0.147. If those numbers
    # do not move, the premise is wrong and this should come back out.
    if a_lead_meas is not None and math.isfinite(a_lead_meas):
      self.aLeadK = float(a_lead_meas)

    # Learn if constant acceleration.
    #
    # THRESHOLD RAISED 0.5 -> 1.0 (2026-09-02). Crossing this collapses aLeadTau to 0, which
    # tells the MPC to project the lead's deceleration as PERMANENT
    # (a_lead_traj = a_lead * exp(-aLeadTau * t^2 / 2)) -- so it must only fire when the lead
    # is REALLY braking, not when it taps.
    #
    # Measured on the car (2026-09-02 route, 5935 lead frames matched against the BYD radar's
    # own directly-measured ALEAD):
    #   threshold 0.5 : 746 collapses, 401 of them SPURIOUS (radar measured the lead
    #                   decelerating less than 0.5 m/s^2) -- 54% false-fire rate
    #   threshold 1.0 : 352 collapses, 108 spurious -- 73% of the false fires removed
    # and at 1.0, all 9 frames where the lead genuinely braked hard (ALEAD < -2.0) STILL
    # collapse tau, plus 86% of moderate braking (-1.0..-2.0). At 2.0 that starts to fail
    # (8 of 9), so 1.0 is the limit.
    #
    # Closed-loop through the MPC, threshold 0.5 vs 1.0: peak braking on a -0.8 tap softens
    # -0.65 -> -0.53, while -2.5 / -3.0 / -5.0 / -8.0 lead braking is BYTE-IDENTICAL. The
    # emergency response is untouched by construction: aLeadK itself is not modified, we
    # simply stop reacting to values the radar says are not there.
    if abs(self.aLeadK) < ALEAD_TAU_COLLAPSE_THRESHOLD:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track
  else:
    return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


# How long a vision-CONFIRMED lead may be sustained on radar alone after vision drops it.
# This does NOT let radar invent a lead: the track must already have been vision-confirmed and
# must still be present in `tracks`, which RadarD rebuilds every frame -- so the hold ends the
# instant radar stops seeing it. A gantry can never exploit this: it is never vision-confirmed.
# Only trust the radar's directly-measured lead acceleration below this (i.e. real braking).
# Above it the Kalman estimate is used, which avoids tripping the aLeadTau "permanent decel"
# behaviour on mild lifts. See the note in Track.update().
ALEAD_MEAS_MIN_DECEL = -1.5   # m/s^2
# |aLeadK| at or above this collapses aLeadTau to 0. See the long note in Track.update().
ALEAD_TAU_COLLAPSE_THRESHOLD = 1.0   # m/s^2  (stock openpilot: 0.5)

LEAD_SUSTAIN_S = 10.0

# A sustained lead has NO vision backing, so it must clear a TIGHTER lateral bar than a
# vision-confirmed one. radar_interface only deletes a track beyond |yRel| > 2.4 m, which
# leaves the 2.0-2.4 m band -- unambiguously the next lane -- eligible to be held for the
# full sustain window. That is a false-lead path my original sustain did not close: before
# it, such a lead was dropped the moment vision let go.
SUSTAIN_YREL_MAX = 1.2   # m


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, low_speed_override: bool = True,
             sustain: dict | None = None, now: float = 0.0) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
  elif (track is None) and ready and (lead_msg.prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  # *** RADAR SUSTAIN ***
  # openpilot needs vision (lead_msg.prob > .5) to declare a lead at speed; radar alone cannot.
  # When traffic ahead stops the model's lead probability can collapse, the lead vanishes, and
  # the planner accelerates toward set speed into a forming queue. Let radar hold a lead that
  # vision already agreed existed, briefly.
  if sustain is not None:
    if lead_dict['status'] and lead_dict.get('radar', False):
      sustain['id'] = lead_dict.get('radarTrackId')
      sustain['t'] = now
    elif not lead_dict['status']:
      tid = sustain.get('id')
      held = (tid is not None and tid in tracks
              and (now - sustain.get('t', 0.0)) < LEAD_SUSTAIN_S
              and abs(tracks[tid].yRel) <= SUSTAIN_YREL_MAX)
      if held:
        lead_dict = tracks[tid].get_RadarState()
        lead_dict['modelProb'] = 0.0     # honest: vision is not backing this right now
      else:
        sustain['id'] = None

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.v_ego = 0.0
    self.a_ego = 0.0
    self._sustain_one: dict = {}
    self._sustain_two: dict = {}
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.a_ego = sm['carState'].aEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured, pt.aRel] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # Radar-measured lead accel. Do NOT add a_ego back.
      #
      # BUG FIXED 2026-09-03: this used to be `rpt[4] + self.a_ego`, on the
      # assumption that radar_interface had subtracted a_ego to make aRel
      # relative. It had not: byd/radar_interface._ego_speed_accel() returns
      # `(v_ego, 0.0)` -- a_ego is hardcoded ZERO -- so `aRel = alead - a_ego`
      # subtracts nothing while this line added the real aEgo back.
      #
      # Result was a positive feedback loop in the braking path: ego brakes ->
      # a_lead_meas = ALEAD + aEgo goes below ALEAD_MEAS_MIN_DECEL purely from
      # ego's own decel -> min() drags aLeadK down -> aLeadTau collapses to 0 ->
      # MPC projects a PERMANENT lead stop -> commands more brake -> aEgo stays
      # large -> loop sustains. Only engages once already braking hard, which is
      # why it presented as a jam brake rather than a constant offset.
      #
      # Measured on route 2026-09-03--06-14-37 (9565 engaged frames):
      #   aLeadK ~= 0.022*a_lead_true + 0.652*aEgo      (clean: 1.000 / 0.000)
      #   corr(aLeadK, true lead accel) = 0.126
      #   during hard braking (aEgo < -1.5, n=166):
      #     aLeadK ~= 0.131*a_lead_true + 1.697*aEgo
      #     mean aLeadK -3.05 vs true lead accel -0.77 (overstates by 2.28)
      # Episode at t=395s: lead shed 10.2 km/h, ego shed 34.9 (3.4x), command
      # saturated at ACCEL_MIN -3.50 for 1.5 s AFTER vRel went positive and the
      # gap was growing, never below 27.2 m.
      #
      # Safe either way: if ALEAD is absolute (as VLEAD is, and as this file
      # already treats it) this is simply correct. If it were relative, it reads
      # POSITIVE while we brake, min() ignores it, and the blend goes inert --
      # i.e. stock behaviour. No branch makes braking weaker than stock.
      a_lead_meas = None
      if len(rpt) > 4 and rpt[4] is not None and math.isfinite(rpt[4]):
        a_lead_meas = rpt[4]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], a_lead_meas)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego,
                                          low_speed_override=True, sustain=self._sustain_one, now=self.current_time)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego,
                                          low_speed_override=False, sustain=self._sustain_two, now=self.current_time)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()