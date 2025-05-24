from opendbc.car.secoc import add_mac
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.car import create_button_events, structs

ButtonType = structs.CarState.ButtonEvent.Type

class SecOCLong:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.reset_counter()

  def reset_counter(self):
    self.secoc_acc_message_counter = 0

  @property
  def enabled(self):
    return self.CP.openpilotLongitudinalControl and (self.CP.flags & ToyotaFlags.SECOC)

  def update_car_state(self, car_state):
    """
      This method is invoked by the CarController to update the car state on the SECOC_LONG object.
      The updated state is then used to update create the secoc signed accel command.
      :param car_state:
      :return:
    """
    self.car_state = car_state

  def set_can_sends(self, can_sends):
    """
      This method is invoked by the CarController to set a reference to the can_sends list.
      The can_sends list is then used to append the secoc signed accel command.
      :param can_sends:
      :return:
    """
    self.can_sends = can_sends

  def update_accel_command(self, packer, values):
    """
      Set ACC_CONTROL's ACCEL_CMD value to 0. On SecOC, stock behavior does not send
      acceleration on ACCEL_CMD. Instead it is send on ACC_CONTROL_2.
      :param values: ACC_CONTROL to be sent in dictionary form before being packed
      :return: Nothing. ACC_CONTROL is updated in place.
    """
    if self.enabled:
      acc_cmd_2 = self.create_accel_2_command(packer, values["ACCEL_CMD"])
      self.can_sends.append(acc_cmd_2)
      values["ACCEL_CMD"] = 0

  def create_accel_2_command(self, packer, accel):
    values = {
      "ACCEL_CMD": accel,
    }
    acc_cmd_2 = packer.make_can_msg("ACC_CONTROL_2", 0, values)
    acc_cmd_2 = add_mac(self.car_state.secoc_key,
                        int(self.car_state.secoc_synchronization['TRIP_CNT']),
                        int(self.car_state.secoc_synchronization['RESET_CNT']),
                        self.secoc_acc_message_counter,
                        acc_cmd_2)
    self.secoc_acc_message_counter += 1
    return acc_cmd_2

class SecOCLongCarState:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.desired_distance = 0
    self.distance_button = 0

  def set_desired_distance(self, desired_distance):
    self.desired_distance = desired_distance

  def update(self, ret: structs.CarState, cp):
    if self.CP.flags & ToyotaFlags.SECOC:
      pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

      prev_distance_button = self.distance_button
      if self.desired_distance != pcm_follow_distance:
        self.distance_button = not self.distance_button
      elif self.distance_button != 0:
        self.distance_button = 0

      ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

class SecOCLongCarController:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.SECOC_LONG = SecOCLong(CP)

  def update(self, CS, CC, can_sends, prev_reset_timer):
    self.SECOC_LONG.update_car_state(CS)
    self.SECOC_LONG.set_can_sends(can_sends)
    if self.CP.flags & ToyotaFlags.SECOC:
      CS.SECOC_LONG_CAR_STATE.set_desired_distance(4 - CC.hudControl.leadDistanceBars)
      if CS.secoc_synchronization['RESET_CNT'] != prev_reset_timer:
        self.SECOC_LONG.reset_counter()
