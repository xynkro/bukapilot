from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.cam_lka.carcontroller import CarController as CamLkaCarController
from opendbc.car.byd.cam_lka.carstate import CarState as CamLkaCarState
from opendbc.car.byd.mpc_lka.carcontroller import CarController as MpcLkaCarController
from opendbc.car.byd.mpc_lka.carstate import CarState as MpcLkaCarState
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.values import CAR, PLATFORM_MPC_LKA, BydFlags


class CarInterface(CarInterfaceBase):
  RadarInterface = RadarInterface

  def __init__(self, CP):
    if CP.carFingerprint in PLATFORM_MPC_LKA:
      self.CarState = MpcLkaCarState
      self.CarController = MpcLkaCarController
    else:
      self.CarState = CamLkaCarState
      self.CarController = CamLkaCarController
    super().__init__(CP)

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "byd"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.byd)]
    # Default 0: no torque spoof/block. Platforms that need spoof set their own param.
    ret.safetyConfigs[0].safetyParam = 0

    ret.steerLimitTimer = 0.6
    ret.steerActuatorDelay = 0.01
    ret.lateralTuning.init("pid")
    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.9871
    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [530]]
    ret.lateralTuning.pid.kpBP = [0.0, 5.0, 20.0]
    ret.lateralTuning.pid.kiBP = [0.0, 5.0, 20.0]
    ret.lateralTuning.pid.kf = 0.00015
    ret.wheelSpeedFactor = 0.66
    ret.startingState = True
    ret.startAccel = 3.0
    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.2
    # MEASURED on this car, not inherited. Cross-correlating carControl.actuators.accel
    # against carState.aEgo over 10692 engaged frames (2026-09-02/03 route, pedal-free,
    # v_ego > 3 m/s) peaks at 0.50 s:
    #     lag 0.30 s -> correlation 0.948, slope 0.985
    #     lag 0.40 s -> correlation 0.954, slope 1.001
    #     lag 0.50 s -> correlation 0.957, slope 1.014   <-- peak
    # It was 0.5 here and I changed it to kommuai's 0.3 on the grounds that 0.3 was "stock".
    # That was wrong: 0.3 is upstream's generic placeholder (interfaces.py notes "TODO
    # estimate car specific lag, use .15s for now"), while 0.5 matches this car's actual
    # actuation. Undershooting it makes the planner sample its trajectory 0.15 s shy of when
    # the command really lands, so every correction arrives late and the next one is sharper.
    ret.longitudinalActuatorDelay = 0.5

    if candidate in PLATFORM_MPC_LKA:
      ret.steerControlType = car.CarParams.SteerControlType.torque
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
      ret.flags |= int(BydFlags.MPC_LKA | BydFlags.ACC_ON_ESC)
      ret.safetyConfigs[0].safetyParam = 4
      ret.openpilotLongitudinalControl = False
      ret.pcmCruise = True  # stock ACC rising edge triggers pcmEnable → lateral
      ret.radarUnavailable = True
      ret.wheelSpeedFactor = 0.66  # Song Plus wheel speed calibration (1.0 was ~40-50% high vs GPS)
      ret.minSteerSpeed = 0.0
    elif candidate in (CAR.BYD_ATTO3, CAR.BYD_M6, CAR.BYD_SEAL6):
      ret.steerControlType = car.CarParams.SteerControlType.angle
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
      if candidate == CAR.BYD_ATTO3:
        # safetyParam 1: ACC_CMD(814) engage + STEERING_TORQUE spoof (block PT→cam 0x1FC)
        ret.safetyConfigs[0].safetyParam = 1
      elif candidate in (CAR.BYD_M6, CAR.BYD_SEAL6):
        ret.safetyConfigs[0].safetyParam = 3
      if candidate == CAR.BYD_SEAL6:
        ret.pcmCruise = True
        ret.wheelSpeedFactor = 0.6336
        ret.enableBsm = False
    elif candidate in (CAR.BYD_SEAL, CAR.BYD_SEALION7, CAR.BYD_SHARK):
      ret.steerControlType = car.CarParams.SteerControlType.angle
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
      ret.safetyConfigs[0].safetyParam = 2
      ret.openpilotLongitudinalControl = False
      ret.radarUnavailable = True
      ret.wheelSpeedFactor = 0.6336
    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    # Caspar diagnostic: FORCE radar + openpilot longitudinal ON for every BYD variant,
    # so a fingerprint mismatch can never silently disable the Atto3 radar.
    ret.radarUnavailable = False
    ret.openpilotLongitudinalControl = True

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    events = self.create_common_events(ret)
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
