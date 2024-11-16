from opendbc.can.parser import CANParser
from opendbc.car.hyundai.values import DBC
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.car.structs import CarParams


class Escc:
  def __init__(self, car_params: CarParams):
    self.car_params = car_params

  @property
  def enabled(self):
    return self.car_params.sunnypilotCarParams.flags & HyundaiFlagsSP.SP_ENHANCED_SCC.value

  @property
  def ESCC_MSG_ID(self):
    return 0x2AB

  def refresh_car_state(self, car_state):
    """
    This method is called by the CarController to update the car state on the ESCC object.
    The new state is used to update the SCC12 message with the current values of the car state received via ESCC.
    :param car_state:
    :return:
    """
    self.car_state = car_state

  def update_scc12_message(self, scc12_message):
    """
    Update the scc12 message with the current values of the car state received via ESCC.
    These values come straight from the on-board radar, and are used as a more reliable source for
    AEB / FCA alerts.
    :param scc12_message: the SCC12 message to be sent in dictionary form before being packed
    :return: Nothing. The scc12_message is updated in place.
    """
    scc12_message["AEB_CmdAct"] = self.car_state.escc_cmd_act
    scc12_message["CF_VSM_Warn"] = self.car_state.escc_aeb_warning
    scc12_message["CF_VSM_DecCmdAct"] = self.car_state.escc_aeb_dec_cmd_act
    scc12_message["CR_VSM_DecCmd"] = self.car_state.escc_aeb_dec_cmd
    # Active AEB. Although I don't think we should set it here ourselves.
    # It might differ from the user's config. Instead we should try to read it from the car and use that.
    # I saw flickering on the dashboard settings where it went to "deactivated" to "active assistance" when sengin AEB_Status 1.
    # Which means likely that SCC12 shows it on the dashboard, but also another FCA message makes it show up
    scc12_message["AEB_Status"] = 2

  def get_radar_escc_parser(self):
    lead_src, bus = "ESCC", 0
    messages = [(lead_src, 50)]
    return CANParser(DBC[self.car_params.carFingerprint]['pt'], messages, bus)


class EsccController:
  def __init__(self, CP):
    self.CP = CP
    self.ESCC = Escc(self.CP)

  def update(self, CC, car_state, now_nanos):
      self.ESCC.refresh_car_state(car_state)
