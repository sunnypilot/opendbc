from opendbc.car import structs
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, LEFT_BLINDSPOT, RIGHT_BLINDSPOT, create_set_bsm_debug_mode, create_bsm_polling_status

class EnhancedBSMController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    """
    Initialize Enhanced Blind Spot Monitoring Controller

    :param CP: Car Parameters
    """
    self.CP = CP
    self.CP_SP = CP_SP

    # Blindspot monitoring state tracking
    self.left_blindspot_debug_enabled = False
    self.right_blindspot_debug_enabled = False
    self.last_blindspot_frame = 0

    # New detection tracking variables
    self._left_blindspot = False
    self._left_blindspot_d1 = 0
    self._left_blindspot_d2 = 0
    self._left_blindspot_counter = 0

    self._right_blindspot = False
    self._right_blindspot_d1 = 0
    self._right_blindspot_d2 = 0
    self._right_blindspot_counter = 0
  @property
  def enabled(self):
    return self.CP_SP.flags & ToyotaFlagsSP.SP_ENHANCED_BSM
    # Frame counter
    self.frame = 0

  def update(self, CS: structs.CarState, frame: int):
    """
    Update method for Enhanced BSM

    :param CS: Car State
    :param frame: Current frame number
    :return: List of CAN messages for BSM
    """


    can_sends = []
    self.frame = frame

    # Process Left Blindspot
    can_sends.extend(self._process_side_blindspot(
      CS,
      LEFT_BLINDSPOT,
      self.left_blindspot_debug_enabled,
      is_left=True
    ))

    # Process Right Blindspot
    can_sends.extend(self._process_side_blindspot(
      CS,
      RIGHT_BLINDSPOT,
      self.right_blindspot_debug_enabled,
      is_left=False
    ))

    return can_sends

  def _process_side_blindspot(self, CS, side_marker, current_debug_state, is_left=True,
                              min_speed=6.0, debug_rate=20, always_on=True):
    """
    Process blindspot monitoring for a specific side of the vehicle

    :param CS: Car State
    :param side_marker: Byte marker for left or right side
    :param current_debug_state: Current debug state for the side
    :param is_left: Whether processing left or right side
    :param min_speed: Minimum speed for BSM activation
    :param debug_rate: Rate of debug message sending
    :param always_on: Whether BSM should always be on
    :return: List of CAN messages
    """
    can_sends = []

    # Activate BSM debug mode
    if not current_debug_state:
      if always_on or CS.out.vEgo > min_speed:
        can_sends.append(create_set_bsm_debug_mode(side_marker, True))
        if is_left:
          self.left_blindspot_debug_enabled = True
        else:
          self.right_blindspot_debug_enabled = True

    # Deactivate or poll BSM
    else:
      # Deactivate if not always on and no recent activity
      if not always_on and self.frame - self.last_blindspot_frame > 50:
        can_sends.append(create_set_bsm_debug_mode(side_marker, False))
        if is_left:
          self.left_blindspot_debug_enabled = False
        else:
          self.right_blindspot_debug_enabled = False

      # Polling logic - alternate between left and right
      poll_condition = (is_left and self.frame % debug_rate == 0) or \
                       (not is_left and self.frame % debug_rate == debug_rate // 2)

      if poll_condition:
        can_sends.append(create_bsm_polling_status(side_marker))
        self.last_blindspot_frame = self.frame

    return can_sends

  def sp_get_enhanced_bsm(self, cp):
    """
    Enhanced Blind Spot Monitoring status retrieval

    :param cp: CAN parser
    :return: Tuple of (left_blindspot, right_blindspot)
    """
    # Let's keep all the commented out code for easy debug purposes in the future.
    distance_1 = cp.vl["DEBUG"].get('BLINDSPOTD1')
    distance_2 = cp.vl["DEBUG"].get('BLINDSPOTD2')
    side = cp.vl["DEBUG"].get('BLINDSPOTSIDE')

    if all(val is not None for val in [distance_1, distance_2, side]):
      if side == 65:  # left blind spot
        if distance_1 != self._left_blindspot_d1 or distance_2 != self._left_blindspot_d2:
          self._left_blindspot_d1 = distance_1
          self._left_blindspot_d2 = distance_2
          self._left_blindspot_counter = 100
          self._left_blindspot = distance_1 > 10 or distance_2 > 10

      elif side == 66:  # right blind spot
        if distance_1 != self._right_blindspot_d1 or distance_2 != self._right_blindspot_d2:
          self._right_blindspot_d1 = distance_1
          self._right_blindspot_d2 = distance_2
          self._right_blindspot_counter = 100
          self._right_blindspot = distance_1 > 10 or distance_2 > 10

      # update counters
      self._left_blindspot_counter = max(0, self._left_blindspot_counter - 1)
      self._right_blindspot_counter = max(0, self._right_blindspot_counter - 1)

      # reset blind spot status if counter reaches 0
      if self._left_blindspot_counter == 0:
        self._left_blindspot = False
        self._left_blindspot_d1 = self._left_blindspot_d2 = 0

      if self._right_blindspot_counter == 0:
        self._right_blindspot = False
        self._right_blindspot_d1 = self._right_blindspot_d2 = 0

    return self._left_blindspot, self._right_blindspot