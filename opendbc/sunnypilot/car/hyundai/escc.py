from abc import ABC

from opendbc.can.parser import CANParser
from opendbc.car.hyundai.values import DBC
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.car import structs

ESCC_MSG = 0x2AB


class EsccCarStateBase:
  def __init__(self):
    self.escc_aeb_warning = 0
    self.escc_aeb_dec_cmd_act = 0
    self.escc_cmd_act = 0
    self.escc_aeb_dec_cmd = 0


class EnhancedSmartCruiseControl:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP

  @property
  def enabled(self):
    return self.CP.sunnypilotCarParams.flags & HyundaiFlagsSP.ENHANCED_SCC

  @property
  def trigger_msg(self):
    return ESCC_MSG

  def refresh_car_state(self, CS):
    """
    This method is called by the CarController to update the car state on the ESCC object.
    The new state is used to update the SCC12 message with the current values of the car state received via ESCC.
    :param CS:
    :return:
    """
    self.CS = CS

  def update_scc12_message(self, values):
    """
    Update the scc12 message with the current values of the car state received via ESCC.
    These values come straight from the on-board radar, and are used as a more reliable source for
    AEB / FCA alerts.
    :param values: the SCC12 message to be sent in dictionary form before being packed
    :return: Nothing. The scc12_message is updated in place.
    """
    values["AEB_CmdAct"] = self.CS.escc_cmd_act
    values["CF_VSM_Warn"] = self.CS.escc_aeb_warning
    values["CF_VSM_DecCmdAct"] = self.CS.escc_aeb_dec_cmd_act
    values["CR_VSM_DecCmd"] = self.CS.escc_aeb_dec_cmd
    # Active AEB. Although I don't think we should set it here ourselves.
    # It might differ from the user's config. Instead we should try to read it from the car and use that.
    # I saw flickering on the dashboard settings where it went to "deactivated" to "active assistance" when sengin AEB_Status 1.
    # Which means likely that SCC12 shows it on the dashboard, but also another FCA message makes it show up
    values["AEB_Status"] = 2

  def get_radar_parser_escc(self):
    lead_src, bus = "ESCC", 0
    messages = [(lead_src, 50)]
    return CANParser(DBC[self.CP.carFingerprint]['pt'], messages, bus)


class EsccCarController:
  def __init__(self, CP):
    self.ESCC = EnhancedSmartCruiseControl(CP)

  def update(self, CS):
    self.ESCC.refresh_car_state(CS)
