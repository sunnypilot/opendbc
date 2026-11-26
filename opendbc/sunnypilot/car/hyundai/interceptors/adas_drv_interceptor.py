import math
from enum import IntEnum

from opendbc.can.dbc import SignalType
from opendbc.can.packer import set_value
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags

from opendbc.sunnypilot.car.hyundai.interceptors.ecu_interceptor import EcuInterceptorBase, \
  EcuInterceptorCarStateBase, EcuInterceptorCarControllerBase, EcuInterceptorRadarInterfaceBase
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

SAFETY_MODE_ADAS_SAFETY_HKG_ADAS_DRV_INTERCEPTOR = 5
ADAS_DRV_INTERCEPTOR_PROTOCOL_VERSION = 1
ADAS_INTERCEPTOR_HEARTBEAT_MSG = 0x1258
LFA_PRESS_FRAMES = 2  # 20 ms, as the camera pulses LFA_BUTTON
LFA_PRESS_PERIOD = 50  # 0.5 s: the ADAS DRV shows its LFA icon 120-140 ms after a pulse, a second one would toggle it off
DCU_161_TIMEOUT_FRAMES = 15  # 3 periods of its 0x161, or of its 0x1E0 without ccNC
DCU_RESUME_FRAMES = 2  # the panda refuses our first steering frame after a gap and restarts its limits, the second passes


class ActvACISta(IntEnum):
  INIT = 0
  INACTIVE = 1
  ACTIVE = 2


class AdasDrvEcuInterceptor(EcuInterceptorBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.dcu_lfa_on = False
    self.frame = 0
    self.dcu_161_counter = None
    self.dcu_161_frame = -DCU_161_TIMEOUT_FRAMES
    self.last_packed = {}

  @property
  def available(self):
    return self.CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR

  @property
  def trigger_msg(self):
    return 0x1A0

  @property
  def lfa_alt_steering(self):
    # the MDPS on angle steering cars only listens to LFA_ALT, so with the ADAS DRV intercepted we steer it directly
    return bool(self.available and self.CP_SP.flags & HyundaiFlagsSP.CANFD_ANGLE_STEERING and self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)

  @staticmethod
  def create_lfa_alt(packer, CAN, lat_active, apply_angle, apply_torque):
    values = {
      "ADAS_ActvACISta": ActvACISta.INIT,
      "ADAS_ActvACILvl2Sta": ActvACISta.ACTIVE if lat_active else ActvACISta.INACTIVE,
      "ADAS_StrAnglReqVal": apply_angle,
      "ADAS_ACIAnglTqRedcGainVal": apply_torque if lat_active else 0,
      "FCA_ESA_ActvSta": 0,
      "FCA_ESA_TqBstGainVal": 0,
    }
    return packer.make_can_msg("LFA_ALT", CAN.ECAN, values)

  def lfa_press(self, msg_161):
    # call once per frame. The ADAS DRV toggles LFA on LFA_BUTTON in 0x50/0x110, not on CRUISE_BUTTONS: we press it until
    # its LFA icon shows, then own the button for the drive so its LFA stays on for HDA and the side cars. Its 0x161
    # (0x1E0 without ccNC) reaches us only once the interceptor diverts it
    self.frame += 1
    counter = msg_161.get("COUNTER")
    if counter != self.dcu_161_counter:
      if None not in (counter, self.dcu_161_counter):
        self.dcu_161_frame = self.frame
      self.dcu_161_counter = counter
    dcu_161_fresh = self.frame - self.dcu_161_frame < DCU_161_TIMEOUT_FRAMES
    self.dcu_lfa_on = self.dcu_lfa_on or (dcu_161_fresh and msg_161.get("LFA_ICON", 0) != 0)
    return dcu_161_fresh and not self.dcu_lfa_on and self.frame % LFA_PRESS_PERIOD < LFA_PRESS_FRAMES

  def create_lkas_alt(self, packer, CAN, lkas_alt, msg_161, enabled, apply_angle):
    # the camera's copy, never engaged, so the ADAS DRV is never asked to steer. The rest passes through
    if not self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT:
      return []
    press = self.lfa_press(msg_161)
    values = {k: v for k, v in lkas_alt.items() if k not in ("CHECKSUM", "COUNTER")}
    values.update({
      "LKA_OptUsmSta": 0,
      "LKA_SysIndReq": 2 if enabled else 1,
      "StrTqReqVal": 0,
      "LKA_SysWrn": 0,
      "ActToiSta": 0,
      "LKA_UsmMod": 0,
      "LKA_RcgSta": 0,
      "Damping_Gain": 100,
      "ADAS_StrAnglReqVal": apply_angle,
      "LKAS_ANGLE_ACTIVE": 1,
      "ADAS_ACIAnglTqRedcGainVal": 0,
    })
    values["LFA_BUTTON"] = 1 if press else 0
    return [self.make_can_msg_changed(packer, "LKAS_ALT", CAN.ACAN, values)]

  def create_lkas(self, packer, CAN, lkas, lfa_button, lat_active, apply_torque):
    # torque cars: the camera's own frame with our steering request and button in place of its own
    values = {k: v for k, v in lkas.items() if k not in ("CHECKSUM", "COUNTER")}
    values |= {"StrTqReqVal": apply_torque, "ActToiSta": 1 if lat_active else 0, "LFA_BUTTON": 1 if lfa_button else 0}
    return self.make_can_msg_changed(packer, "LKAS_ALT" if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS", CAN.ACAN, values)

  def make_can_msg_changed(self, packer, name, bus, values):
    # repacking all ~30 signals of the camera's copy every frame costs more than the few that change
    msg = packer.dbc.name_to_msg[name]
    if name not in self.last_packed or msg.address not in packer.counters:
      _, dat, _ = packer.make_can_msg(name, bus, values)
      self.last_packed[name] = (dict(values), bytearray(dat))
      return msg.address, dat, bus

    last_values, dat = self.last_packed[name]
    for sig_name, value in values.items():
      if last_values.get(sig_name) != value:
        sig = msg.sigs[sig_name]
        ival = int(math.floor((value - sig.offset) / sig.factor + 0.5))
        set_value(dat, sig, ival + (1 << sig.size) if ival < 0 else ival)
        last_values[sig_name] = value

    counter = next(s for s in msg.sigs.values() if s.type == SignalType.COUNTER or s.name == "COUNTER")
    set_value(dat, counter, packer.counters[msg.address])
    packer.counters[msg.address] = (packer.counters[msg.address] + 1) % (1 << counter.size)
    checksum = next(s for s in msg.sigs.values() if s.type > SignalType.COUNTER)
    set_value(dat, checksum, checksum.calc_checksum(msg.address, checksum, dat))
    return msg.address, bytes(dat), bus

  def create_adas_drv_intercept_msg(self, packer, CAN):
    nav = int(self.CP.openpilotLongitudinalControl)
    values = {
      "REQUESTED_SAFETY_MODE": SAFETY_MODE_ADAS_SAFETY_HKG_ADAS_DRV_INTERCEPTOR,
      "REQUESTED_SAFETY_PARAM": ADAS_DRV_INTERCEPTOR_PROTOCOL_VERSION,
      "FEATURE_1": 1,
      "FEATURE_2": 1,
      # the ADAS DRV shows HDA and fills the side cars only on a freeway: every road is sent to it as one, with the
      # road's own speed limit
      "FEATURE_NAV_SEGMENT": nav,
      "FEATURE_NAV_SEGMENT_UPDATE": nav,
      "FEATURE_NAV_SEGMENT_LIMIT": 0,
      "FEATURE_NAV_SUMMARY": 0,
      # its curve slowdown follows the head unit's predicted path, often a turn not taken off a freeway
      "FEATURE_NAV_PROFILE": nav,
    }
    return [packer.make_can_msg("ADAS_DRV_INTERCEPT_OPT", CAN.ACAN, values)]


# VERY LIKELY NOT NEEDED AS WE DON'T KEEP A STATE ABOUT THE INTERCEPTOR, SINCE WE PASS MSGS AS-IS
class AdasDrvEcuInterceptorCarState(EcuInterceptorCarStateBase):
  pass


class AdasDrvEcuInterceptorCarController(EcuInterceptorCarControllerBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.dcu_resume_frames = 0

  def update(self, car_state):
    super().update(car_state)
    # our steering frame is held back while the ADAS DRV steers in an emergency. After it, our command restarts where the
    # panda's limits do: at the wheel angle, or at no torque
    if car_state.dcu_passthrough:
      self.dcu_resume_frames = DCU_RESUME_FRAMES
    elif self.dcu_resume_frames > 0:
      self.dcu_resume_frames -= 1
      if self.interceptor.lfa_alt_steering:
        self.apply_angle_last = car_state.out.steeringAngleDeg
        self.angle_filter.x = self.apply_angle_last
      else:
        self.apply_torque_last = 0

  def initialize(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> AdasDrvEcuInterceptor:
    """
      This method is invoked to initialize the ESCC object.
      It should be called before any other methods are invoked.
    """
    return AdasDrvEcuInterceptor(CP, CP_SP)


class AdasDrvEcuInterceptorRadarInterface(EcuInterceptorRadarInterfaceBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

  def initialize(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> AdasDrvEcuInterceptor:
    """
      This method is invoked to initialize the ESCC object.
      It should be called before any other methods are invoked.
    """
    return AdasDrvEcuInterceptor(CP, CP_SP)
