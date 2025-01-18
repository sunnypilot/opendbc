from opendbc.car import structs
from opendbc.sunnypilot import SunnypilotParamFlags


class MadsCarController:
  def __init__(self):
    super().__init__()
    self.lat_paused = False

  def update(self, CC: structs.CarControl) -> None:
    enable_mads = CC.sunnypilotParams & SunnypilotParamFlags.ENABLE_MADS

    if enable_mads:
      self.lat_paused = CC.madsEnabled and not CC.latActive
    else:
      self.lat_paused = CC.hudControl.lanesVisible
