"""
Cherry port smoke + regression tests (offline, no vehicle logs).

Run: pytest opendbc/car/cherry/tests/test_cherry_smoke.py -q

Phase 2 — replay against real CAN (when available):
  Use opendbc/car/tests/car_diff.py::replay_segment(platform, can_msgs) with CAN from a
  comma segment or local rlog. Generate a reference pickle once:
    python opendbc/car/tests/car_diff.py --platform CHERRY_JAECOO_J7_PHEV --update ...
  Then CI compares CarState dicts to the committed .zst under car_diff bucket / repo refs.
"""

import copy
import math

import pytest
from cereal import car as cereal_car

from opendbc.can import CANParser, CANPacker
from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.cherry.carcontroller import CarController
from opendbc.car.cherry.carstate import CarState
from opendbc.car.cherry.cherrycan import (
  cherry_checksum,
  create_lane_keep_command,
  create_lkas_info_torque_spoof,
  create_pcm_icc_toggle_press,
)
from opendbc.car.cherry.fingerprints import FINGERPRINTS
from opendbc.car.cherry.interface import CarInterface
from opendbc.car.cherry.radar_interface import RadarInterface
from opendbc.car.cherry.values import CANBUS, CAR, DBC, cherry_steering_deg_sign


DBC_NAME = "cherry_general_pt"
# 0x391 STALK: logged door-shut frame; packer cannot set byte2 (0xa8) without overlapping blinker bits.
STALK_391_DOOR_SHUT_HEX = "cceaa80810404900"


def _cp():
  fp = FINGERPRINTS[CAR.CHERRY_JAECOO_J7_PHEV][0]
  return CarInterface.get_params(
    CAR.CHERRY_JAECOO_J7_PHEV,
    {0: fp},
    [],
    False,
    False,
    False,
  )


class TestCherryParams:
  def test_get_params(self):
    CP = _cp()
    assert CP.carFingerprint == CAR.CHERRY_JAECOO_J7_PHEV
    assert CP.brand == "cherry"
    assert CP.radarUnavailable is True
    assert CP.openpilotLongitudinalControl is False
    assert CP.steerControlType == cereal_car.CarParams.SteerControlType.angle
    assert CP.safetyConfigs[0].safetyModel == cereal_car.CarParams.SafetyModel.cherry
    assert CP.safetyConfigs[0].safetyParam == 0
    assert math.isclose(CP.wheelSpeedFactor, 0.832, rel_tol=0.0, abs_tol=1e-5)
    assert CP.dashcamOnly is False


class TestCherryChecksum:
  """CRC over bytes 0..6 with xorOut 0x0A matches vehicle BO_ 467 (0x1D3) logs (Jaecoo J7)."""

  # Packed LANE_KEEP (addr 837) with steer=0, LKAS on, default padding, COUNTER=0 — use to
  # compare against `can_table.py 0x345 2` on stock LKA when investigating ECU rejects.
  LANE_KEEP_GOLDEN_HEX = "79e6fffcf463f095"

  # Stock LANE_KEEP frames captured live from Jaecoo J7 bus 2 (car on, stationary, LKA off).
  # Locks padding (ff fc f4 63), upper-nibble SET_ME_XF (0xF), and cherry_checksum formula
  # against the real ECU. If any of these drift, the ECU will reject openpilot's LANE_KEEP.
  STOCK_LANE_KEEP_HEX = [
    "7abcfffcf463f397",  # counter 3
    "7abcfffcf463f4c4",  # counter 4
    "7abcfffcf463f5d9",  # counter 5
    "7abcfffcf463f858",  # counter 8 (covers 0x58 = stock CRC)
    "7abcfffcf463ff0b",  # counter 15 (wrap)
    "7abcfffcf463f0b0",  # counter 0 (after wrap)
  ]

  def test_lane_keep_golden_padding_and_checksum(self):
    packer = CANPacker(DBC_NAME)
    addr, dat, bus = packer.make_can_msg(
      "LANE_KEEP",
      CANBUS.main_bus,
      {
        "STEER_CMD_ANGLE": 0.0,
        "LKAS_ENABLE": 1,
        "SET_ME_XFF": 255,
        "SET_ME_XFC": 252,
        "SET_ME_XF4": 244,
        "SET_ME_X63": 99,
        "SET_ME_XF": 15,
        "COUNTER": 0,
      },
    )
    assert addr == 837
    assert bus == CANBUS.main_bus
    assert dat.hex() == self.LANE_KEEP_GOLDEN_HEX
    b = bytearray(dat)
    assert b[2:6] == bytes([0xFF, 0xFC, 0xF4, 0x63])
    assert cherry_checksum(addr, None, b) == b[7]

  @pytest.mark.parametrize("stock_hex", STOCK_LANE_KEEP_HEX)
  def test_lane_keep_stock_frame_checksum(self, stock_hex):
    d = bytearray(bytes.fromhex(stock_hex))
    assert d[2:6] == bytes([0xFF, 0xFC, 0xF4, 0x63]), "stock padding bytes drifted"
    assert (d[6] >> 4) & 0x0F == 0xF, "SET_ME_XF (upper nibble of byte 6) must be 0xF"
    assert cherry_checksum(837, None, d) == d[7], \
      f"cherry_checksum mismatch on stock frame {stock_hex}"

  def test_cherry_checksum_matches_packer_eps(self):
    packer = CANPacker(DBC_NAME)
    _, dat, _ = packer.make_can_msg("EPS", 0, {"STEERING_ANGLE": 12.3, "DRIVER_TORQUE": 5})
    d = bytearray(dat)
    assert cherry_checksum(467, None, d) == d[7]

  def test_eps_packed_snapshot_matches_formula(self):
    """Locks one deterministic EPS frame (steer=0, torque=0, explicit counter). Replace hex with a stock frame when available."""
    packer = CANPacker(DBC_NAME)
    addr, dat, _ = packer.make_can_msg(
      "EPS", 0, {"STEERING_ANGLE": 0.0, "DRIVER_TORQUE": 0, "COUNTER": 5},
    )
    assert addr == 467
    d = bytearray(dat)
    assert d[7] == cherry_checksum(addr, None, d)
    assert (d[6] & 0x0F) == 5

  def test_lkas_info_tx_on_vehicle_bus(self):
    packer = CANPacker(DBC_NAME)
    msg = create_lkas_info_torque_spoof(packer, lat_active=True, main_torque=12.0, spoof_active=True)
    assert msg[0] == 0x394
    assert msg[2] == CANBUS.main_bus, "LKAS_INFO TX must be bus 0 (PT), not camera bus 2"

  def test_lkas_info_checksum_matches_packer(self):
    packer = CANPacker(DBC_NAME)
    msg = create_lkas_info_torque_spoof(packer, lat_active=True, main_torque=12.0, spoof_active=True)
    d = bytearray(msg[1])
    assert msg[2] == CANBUS.main_bus
    assert cherry_checksum(0x394, None, d) == d[7]

  def test_lkas_info_counter_increments(self):
    packer = CANPacker(DBC_NAME)
    parser = CANParser(DBC_NAME, [("LKAS_INFO", 50)], CANBUS.main_bus)
    counters = []
    for _ in range(4):
      msg = create_lkas_info_torque_spoof(packer, lat_active=False, main_torque=0.0, spoof_active=False)
      parser.update([0, [msg]])
      counters.append(int(parser.vl["LKAS_INFO"]["COUNTER"]))
    assert counters == [0, 1, 2, 3]

  def test_lane_keep_roundtrip_checksum(self, subtests):
    parser = CANParser(DBC_NAME, [("LANE_KEEP", 2)], CANBUS.cam_bus)
    packer = CANPacker(DBC_NAME)

    seed = packer.make_can_msg(
      "LANE_KEEP",
      CANBUS.cam_bus,
      {
        "STEER_CMD_ANGLE": 4.2,
        "LKAS_ENABLE": 1,
        "SET_ME_XFF": 255,
        "SET_ME_XFC": 252,
        "SET_ME_XF4": 244,
        "SET_ME_X63": 99,
        "SET_ME_XF": 15,
      },
    )
    parser.update([0, [seed]])
    expected = copy.deepcopy(parser.vl["LANE_KEEP"])

    modified = copy.deepcopy(expected)
    modified.pop("CHECKSUM", None)
    repacked = packer.make_can_msg("LANE_KEEP", CANBUS.cam_bus, modified)

    parser.update([0, [repacked]])
    tested = parser.vl["LANE_KEEP"]
    with subtests.test(msg="checksum_roundtrip"):
      assert tested["CHECKSUM"] == expected["CHECKSUM"]


class TestCherrySyntheticCan:
  def _frames_one_tick(self, packer: CANPacker) -> list[CanData]:
    # Physically plausible defaults so CarState paths execute without KeyErrors.
    kph = 40.0  # decoded wheel domain uses DBC factor 0.01 on raw
    wh = int(round(kph / 0.01))
    pt = [
      packer.make_can_msg("WHEELSPEED_1", 0, {"WHEEL_BL": wh, "WHEEL_BR": wh, "ACC_ENABLED": 0}),
      packer.make_can_msg("WHEELSPEED_2", 0, {"WHEEL_FL": wh, "WHEEL_FR": wh}),
      packer.make_can_msg("EPS", 0, {"STEERING_ANGLE": 5.0, "DRIVER_TORQUE": 0}),
      packer.make_can_msg("GAS", 0, {"GAS_PEDAL_PRESSURE": 0.0, "GAS_THROTTLE": 0.0}),
      packer.make_can_msg("TRANSMISSION", 0, {"GEAR": 4}),
      packer.make_can_msg("BRAKE_PEDAL", 0, {"BRAKE_PRESSURE": 0.0, "AUTO_HOLD": 0}),
      CanData(0x391, bytes.fromhex(STALK_391_DOOR_SHUT_HEX), 0),
      packer.make_can_msg("PCM_BUTTONS", 0, {"ICC_TOGGLE": 0, "CRUISE_BUTTON": 0}),
      packer.make_can_msg("ADAS_RELATED", 0, {"NEW_SIGNAL_1": 0, "NEW_SIGNAL_2": 0, "NEW_SIGNAL_3": 0}),
      packer.make_can_msg("SPEED_RELATED", 0, {"NON_CAL_SPEED1": 0, "NON_CAL_SPEED2": 0}),
      packer.make_can_msg("STEER_RELATED", 0, {"STEERING_ANGLE_NOT_CALIBRATED": 0}),
      packer.make_can_msg("LKAS_INFO", CANBUS.main_bus, {"MAIN_TORQUE": 0, "LKAS_ENABLE": 1, "STEER_RELATED": 0}),
    ]
    cam = [
      packer.make_can_msg(
        "HUD",
        CANBUS.cam_bus,
        {
          "AEB": 0,
          "CANCEL_CRUISE_UNCERTAIN": 0,
          "GAS_RESUME_UNCERTAIN": 0,
          "FOLLOW_DISTANCE": 1,
          "NEW_SIGNAL_1": 0,
          "PCW": 0,
          "CRUISE_STATE": 3,
          "GAS_OVERRIDE": 0,
          "AEB_RELATED": 0,
          "SET_SPEED": 80,
        },
      ),
      packer.make_can_msg(
        "LANE_KEEP",
        CANBUS.cam_bus,
        {
          "STEER_CMD_ANGLE": 5.0,
          "LKAS_ENABLE": 1,
          "SET_ME_XFF": 255,
          "SET_ME_XFC": 252,
          "SET_ME_XF4": 244,
          "SET_ME_X63": 99,
          "SET_ME_XF": 15,
        },
      ),
      packer.make_can_msg("ACC_UNCERTAIN", CANBUS.cam_bus, {"NEW_SIGNAL_2": 260.0}),
    ]
    out: list[CanData] = []
    for item in pt + cam:
      if isinstance(item, CanData):
        out.append(item)
      else:
        addr, dat, bus = item
        out.append(CanData(addr, dat, bus))
    return out

  def test_car_interface_update(self):
    CP = _cp()
    CI = CarInterface(CP)
    packer = CANPacker(DBC_NAME)

    for i in range(3):
      t = i * 20_000_000
      frames = self._frames_one_tick(packer)
      st = CI.update([(t, frames)])
      assert math.isfinite(st.vEgo)
      assert math.isfinite(st.steeringAngleDeg)
      assert st.cruiseState.enabled is True
      assert st.cruiseState.speedCluster > 0.0
      assert st.cruiseState.standstill is False
      assert CI.CS.lkas_enable_lane is True
      assert CI.CS.lkas_enable_info is True
      assert st.doorOpen is False
      assert st.genericToggle is False

  def test_hud_follow_distance_five_bars_from_stock_frames(self):
    """3-bit FOLLOW_DISTANCE (bits 9-11): route log 1..5 = 5..1 ACC gap bars."""
    stock = {
      1: "000200128c078f2a",
      2: "0004001288050c3c",
      3: "000600128c062aab",
      4: "000800128805c4ee",
      5: "000a00128c05cbdf",
    }
    for bars, hx in stock.items():
      d = bytes.fromhex(hx)
      p = CANParser(DBC_NAME, [("HUD", 20)], CANBUS.cam_bus)
      p.update([(0, [(0x387, d, CANBUS.cam_bus)])])
      assert int(p.vl["HUD"]["FOLLOW_DISTANCE"]) == bars

  def test_personality_from_hud_follow_distance(self):
    """HUD FOLLOW_DISTANCE maps to carState.personality; unknown raw stays -1 (not forced to 0)."""
    from opendbc.car.cherry.values import CHERRY_FOLLOW_RAW_TO_PERSONALITY

    CP = _cp()
    packer = CANPacker(DBC_NAME)
    hud_counter = 0

    for raw, want in [(0, -1), (1, 2), (2, 2), (3, 1), (4, 0), (5, 0), (6, -1), (7, -1)]:
      CI = CarInterface(CP)
      frames = self._frames_one_tick(packer)
      patched = []
      for f in frames:
        if f.address == 0x387:
          continue
        patched.append(f)
      _, dat, _ = packer.make_can_msg(
        "HUD",
        CANBUS.cam_bus,
        {
          "AEB": 0,
          "CANCEL_CRUISE_UNCERTAIN": 0,
          "GAS_RESUME_UNCERTAIN": 0,
          "FOLLOW_DISTANCE": raw,
          "NEW_SIGNAL_1": 0,
          "PCW": 0,
          "CRUISE_STATE": 3,
          "GAS_OVERRIDE": 0,
          "AEB_RELATED": 0,
          "SET_SPEED": 80,
          "COUNTER": hud_counter,
        },
      )
      hud_counter = (hud_counter + 1) % 16
      patched.append(CanData(0x387, dat, CANBUS.cam_bus))
      st = CI.update([(0, patched)])
      assert st.personality == want, f"FOLLOW_DISTANCE raw={raw} -> personality {st.personality} != {want}"
      assert int(CI.CS.distance_val) == raw
      if raw in CHERRY_FOLLOW_RAW_TO_PERSONALITY:
        assert want == CHERRY_FOLLOW_RAW_TO_PERSONALITY[raw]

  def test_door_ajar_from_payload391_b3_lsb(self):
    parser = CANParser(DBC_NAME, [("STALK", 50)], 0)
    parser.update([(0, [(0x391, bytes.fromhex("cceaa80810404900"), 0)])])
    assert parser.vl["STALK"]["PAYLOAD391_B3"] == 8.0
    assert int(parser.vl["STALK"]["PAYLOAD391_B3"]) & 1 == 0
    parser.update([(0, [(0x391, bytes.fromhex("cce9a80910404900"), 0)])])
    assert parser.vl["STALK"]["PAYLOAD391_B3"] == 9.0
    assert int(parser.vl["STALK"]["PAYLOAD391_B3"]) & 1 == 1

  def test_door_open_when_stalk391_b3_ajar(self):
    CP = _cp()
    CI = CarInterface(CP)
    packer = CANPacker(DBC_NAME)
    frames = self._frames_one_tick(packer)
    door_ajar = bytearray(bytes.fromhex(STALK_391_DOOR_SHUT_HEX))
    door_ajar[3] = 0x09
    patched: list[CanData] = []
    for f in frames:
      if f.address == 0x391:
        patched.append(CanData(0x391, bytes(door_ajar), f.src))
      else:
        patched.append(f)
    st = CI.update([(0, patched)])
    assert st.doorOpen is True

  def test_radar_interface(self):
    CP = _cp()
    ri = RadarInterface(CP)
    assert ri.update([]) is None


class TestCherryCarController:
  def test_lane_keep_tx_and_parse(self):
    CP = _cp()
    cc_state = CarState(CP)
    dbc_names = {Bus.pt: DBC_NAME, Bus.cam: DBC_NAME}
    CC = CarController(dbc_names, CP)

    CS = cc_state
    CS.out = structs.CarState.new_message()
    CS.out.steeringAngleDeg = 3.0
    CS.out.vEgo = 12.0
    CS.out.standstill = False
    CS.lkas_enable_lane = True

    ctl = structs.CarControl()
    ctl.latActive = True
    ctl.actuators.steeringAngleDeg = 7.0
    ctl = ctl.as_reader()

    can_sends: list = []
    for _ in range(40):
      _, sends = CC.update(ctl, CS, 0)
      can_sends.extend(sends)

    lane_keep = [x for x in can_sends if x[0] == 0x345]
    assert len(lane_keep) >= 1
    addr, dat, bus = lane_keep[-1]
    assert bus == CANBUS.main_bus, "LANE_KEEP must TX on vehicle bus 0 (ECU side); bus 2 is the stock-cam leg"
    parser = CANParser(DBC_NAME, [("LANE_KEEP", 2)], CANBUS.main_bus)
    parser.update([0, [(addr, dat, bus)]])
    vl = parser.vl["LANE_KEEP"]
    assert vl["LKAS_ENABLE"] == 1
    assert math.isclose(vl["STEER_CMD_ANGLE"], 7.0, abs_tol=0.05)

  def test_steering_lowpass_smooths_command(self):
    """Same 2 Hz one-pole as BYD: large step is filtered before rate limits."""
    from opendbc.car.cherry.values import lowpass_steer_cmd

    target = 30.0
    out = lowpass_steer_cmd(target, 0.0)
    assert abs(out) < abs(target)
    assert math.isfinite(out)

  def test_compute_apply_angle_no_raise(self):
    CP = _cp()
    CC = CarController({Bus.pt: DBC_NAME, Bus.cam: DBC_NAME}, CP)
    CS = CarState(CP)
    CS.out = structs.CarState.new_message()

    act = structs.CarControl.Actuators()
    for lat_active in (False, True):
      for v in (0.0, 2.0, 25.0):
        for steer_meas in (-180.0, 0.0, 90.0):
          for steer_cmd in (-400.0, 0.0, 50.0, 400.0):
            CS.out.steeringAngleDeg = steer_meas
            CS.out.vEgo = v
            CS.out.standstill = v < 0.01
            act.steeringAngleDeg = steer_cmd
            out = CC._compute_apply_angle(CS, act, lat_active)
            assert math.isfinite(out)

  def test_lane_keep_lkas_off_when_driver_steering(self):
    """Driver torque must drop LKAS_ENABLE to avoid EPS fault (route 2026-05-14--07-49-04)."""
    CP = _cp()
    CC = CarController({Bus.pt: DBC_NAME, Bus.cam: DBC_NAME}, CP)
    CS = CarState(CP)
    CS.out = structs.CarState.new_message()
    CS.out.steeringAngleDeg = 2.0
    CS.out.steeringTorque = 25.0
    CS.out.vEgo = 12.0
    CS.out.standstill = False
    CS.out.steeringPressed = True
    CS.steer_related_intervention = False

    ctl = structs.CarControl()
    ctl.latActive = True
    ctl.actuators.steeringAngleDeg = 9.0
    ctl = ctl.as_reader()

    can_sends: list = []
    for _ in range(40):
      _, sends = CC.update(ctl, CS, 0)
      can_sends.extend(sends)

    lane_keep = [x for x in can_sends if x[0] == 0x345]
    assert len(lane_keep) >= 1
    parser = CANParser(DBC_NAME, [("LANE_KEEP", 2)], CANBUS.main_bus)
    parser.update([0, [lane_keep[-1]]])
    assert parser.vl["LANE_KEEP"]["LKAS_ENABLE"] == 0

  def test_pcm_icc_press_counter_and_checksum(self):
    """PCM_BUTTONS: pack ICC_TOGGLE + COUNTER + CHECKSUM_BUTTONS (Chrysler CRC over bytes 1..5)."""
    from opendbc.car.cherry.cherrycan import create_pcm_icc_toggle_press

    packer = CANPacker(DBC_NAME)
    stock_icc = bytes.fromhex("529000010000")
    stock_idle = bytes.fromhex("dd9000000000")

    _, packed_icc, _ = create_pcm_icc_toggle_press(packer, 9)
    assert bytes(packed_icc) == stock_icc

    _, packed_idle, _ = packer.make_can_msg(
      "PCM_BUTTONS", CANBUS.main_bus, {"ICC_TOGGLE": 0, "CRUISE_BUTTON": 0, "COUNTER": 9}
    )
    assert bytes(packed_idle) == stock_idle

    parser = CANParser(DBC_NAME, [("PCM_BUTTONS", 20)], CANBUS.main_bus)
    parser.update([0, [(0x360, packed_icc, CANBUS.main_bus)]])
    assert int(parser.vl["PCM_BUTTONS"]["ICC_TOGGLE"]) == 1
    assert int(parser.vl["PCM_BUTTONS"]["COUNTER"]) == 9
    assert int(parser.vl["PCM_BUTTONS"]["CHECKSUM_BUTTONS"]) == 0x52

  def test_stock_acc_resume_sends_icc_toggle_press(self):
    CP = _cp()
    CC = CarController({Bus.pt: DBC_NAME, Bus.cam: DBC_NAME}, CP)
    CS = CarState(CP)
    CS.out = structs.CarState.new_message()
    CS.out.vEgo = 0.0
    CS.out.standstill = True
    CS.out.brakePressed = False

    ctl = structs.CarControl()
    ctl.enabled = True
    ctl.latActive = True
    ctl.cruiseControl.resume = True
    ctl.actuators.steeringAngleDeg = 0.0
    ctl = ctl.as_reader()

    can_sends: list = []
    for _ in range(100):
      _, sends = CC.update(ctl, CS, 0)
      can_sends.extend(sends)

    pcm = [x for x in can_sends if x[0] == 0x360]
    assert len(pcm) >= 1
    assert pcm[0][2] == CANBUS.main_bus
    parser = CANParser(DBC_NAME, [("PCM_BUTTONS", 20)], CANBUS.main_bus)
    parser.update([0, [pcm[0]]])
    vl = parser.vl["PCM_BUTTONS"]
    assert int(vl["ICC_TOGGLE"]) == 1
    assert int(vl["CRUISE_BUTTON"]) == 0
    assert int(vl["COUNTER"]) == 1
    assert int(vl["CHECKSUM_BUTTONS"]) != 0

  def test_lat_control_not_blocked_by_stock_lka_bit(self):
    CP = _cp()
    cc_state = CarState(CP)
    CC = CarController({Bus.pt: DBC_NAME, Bus.cam: DBC_NAME}, CP)

    CS = cc_state
    CS.out = structs.CarState.new_message()
    CS.out.steeringAngleDeg = 2.0
    CS.out.vEgo = 12.0
    CS.out.standstill = False
    CS.lkas_enable_lane = False
    CS.lkas_enable_info = False

    ctl = structs.CarControl()
    ctl.latActive = True
    ctl.actuators.steeringAngleDeg = 9.0
    ctl = ctl.as_reader()

    can_sends: list = []
    for _ in range(40):
      _, sends = CC.update(ctl, CS, 0)
      can_sends.extend(sends)

    lane_keep = [x for x in can_sends if x[0] == 0x345]
    assert len(lane_keep) >= 1
    parser = CANParser(DBC_NAME, [("LANE_KEEP", 2)], CANBUS.main_bus)
    parser.update([0, [lane_keep[-1]]])
    vl = parser.vl["LANE_KEEP"]
    assert vl["LKAS_ENABLE"] == 1


class TestCherryCreateLaneKeepCommand:
  def test_matches_parser(self):
    CP = _cp()
    s = cherry_steering_deg_sign(CP)
    packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    msg = create_lane_keep_command(packer, 6.5, True, 3.0, steering_deg_sign=s)
    assert msg[2] == CANBUS.main_bus, "LANE_KEEP TX must target vehicle bus 0, not the camera leg"
    parser = CANParser(DBC_NAME, [("LANE_KEEP", 2)], CANBUS.main_bus)
    parser.update([0, [msg]])
    vl = parser.vl["LANE_KEEP"]
    assert vl["LKAS_ENABLE"] == 1
    # CAN engineering value = s * OP_command (Jaecoo J7: s=+1 so OP and wire match).
    assert math.isclose(vl["STEER_CMD_ANGLE"], s * 6.5, abs_tol=0.05)

    msg2 = create_lane_keep_command(packer, 0.0, False, 12.0, steering_deg_sign=s)
    assert msg2[2] == CANBUS.main_bus
    parser.update([0, [msg2]])
    vl2 = parser.vl["LANE_KEEP"]
    assert vl2["LKAS_ENABLE"] == 0
    assert math.isclose(vl2["STEER_CMD_ANGLE"], s * 12.0, abs_tol=0.05)


class TestCherrySteeringSignConsistency:
  """OP convention (+ = left / CCW) vs CAN (DBC engineering units on EPS / LANE_KEEP).

  CarState: steeringAngleDeg_OP = s * EPS_decode.
  TX: STEER_CMD_ANGLE packed with s * OP_angle so the bus uses the same units as EPS.

  On-car sanity: turn wheel slowly left; carState.steeringAngleDeg should correlate with
  yaw rate as expected for paramsd (Jaecoo J7 uses s=+1).
  """

  def test_lane_keep_track_meas_matches_eps_angle_on_bus(self):
    """steer_req False: cmd on CAN should equal EPS physical angle implied by OP meas."""
    CP = _cp()
    s = cherry_steering_deg_sign(CP)
    packer = CANPacker(DBC_NAME)
    v_can = -4.2  # arbitrary CAN-side deg
    op_meas = s * v_can
    msg = create_lane_keep_command(packer, 0.0, False, op_meas, steering_deg_sign=s)
    p_lk = CANParser(DBC_NAME, [("LANE_KEEP", 2)], CANBUS.main_bus)
    p_lk.update([0, [msg]])
    assert math.isclose(p_lk.vl["LANE_KEEP"]["STEER_CMD_ANGLE"], v_can, abs_tol=0.05)

    _, eps_dat, _ = packer.make_can_msg("EPS", CANBUS.main_bus, {"STEERING_ANGLE": v_can, "DRIVER_TORQUE": 0})
    p_eps = CANParser(DBC_NAME, [("EPS", 100)], CANBUS.main_bus)
    p_eps.update([0, [(467, eps_dat, CANBUS.main_bus)]])
    assert math.isclose(p_eps.vl["EPS"]["STEERING_ANGLE"], v_can, abs_tol=0.05)

  def test_lane_keep_steer_req_scales_like_eps(self):
    CP = _cp()
    s = cherry_steering_deg_sign(CP)
    packer = CANPacker(DBC_NAME)
    for op_cmd in (-15.0, 0.0, 22.5):
      v_expected = s * op_cmd
      msg = create_lane_keep_command(packer, op_cmd, True, 0.0, steering_deg_sign=s)
      p = CANParser(DBC_NAME, [("LANE_KEEP", 2)], CANBUS.main_bus)
      p.update([0, [msg]])
      assert math.isclose(p.vl["LANE_KEEP"]["STEER_CMD_ANGLE"], v_expected, abs_tol=0.05)
