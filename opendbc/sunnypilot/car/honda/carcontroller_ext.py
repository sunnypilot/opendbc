from opendbc.car import DT_CTRL, rate_limit, structs
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP


class CarControllerExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

  def rate_limit_ext(self, new_value, last_value, dw_step, up_step):
    if (self.CP_SP.flags & HondaFlagsSP.EPS_MODIFIED) and self.CP.lateralTuning.which() == "pid":
        # Speed params can be adjusted if needed
        base_tau = 0.2  # Time constant in seconds
        alpha = DT_CTRL / (base_tau + DT_CTRL)  # Alpha for first-order low-pass

        # Simple low-pass filter
        return alpha * new_value + (1 - alpha) * last_value
    else:
        return rate_limit(new_value, last_value, dw_step, up_step)
