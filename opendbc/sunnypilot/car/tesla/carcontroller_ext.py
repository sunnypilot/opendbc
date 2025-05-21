"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car.interfaces import CarStateBase

TORQUE_TO_ANGLE_MULTIPLIER_OUTER = 4  # Higher = easier to influence when manually steering more than OP
TORQUE_TO_ANGLE_MULTIPLIER_INNER = 8  # Higher = easier to influence when manually steering less than OP
TORQUE_TO_ANGLE_DEADZONE = 0.5  # This equates to hands-on level 1, so we don't allow override if not hands-on
TORQUE_TO_ANGLE_CLIP = 10.  # Steering (usually) disengages at 2.5 Nm, this limit exists only in case the EPAS gives bad data
CONTINUED_OVERRIDE_ANGLE = 10.  # The angle difference between OP and user to continue overriding steering (prevents oscillation)


class CarControllerExt:
  def __init__(self):
    self.enabled = True  # TODO-SP: always on for now, couple with toggle
    self.steering_override = False
    self.last_hands_nanos = 0

  @staticmethod
  def torque_blended_angle(apply_angle, torsion_bar_torque):
    deadzone = TORQUE_TO_ANGLE_DEADZONE
    if abs(torsion_bar_torque) < deadzone:
      return apply_angle

    limit = TORQUE_TO_ANGLE_CLIP
    if apply_angle * torsion_bar_torque >= 0:
      # user override in the same direction
      strength = TORQUE_TO_ANGLE_MULTIPLIER_OUTER
    else:
      # user override in the opposite direction
      strength = TORQUE_TO_ANGLE_MULTIPLIER_INNER

    torque = torsion_bar_torque - deadzone if torsion_bar_torque > 0 else torsion_bar_torque + deadzone
    return apply_angle + np.clip(torque, -limit, limit) * strength

  def update_torque_blending(self, CS: CarStateBase, lat_active: bool, apply_angle: float, now_nanos: int) -> tuple[bool, float]:
    if not self.enabled:
      return lat_active, apply_angle

    # Detect user override of the steering wheel
    self.steering_override = CS.hands_on_level >= 3 or \
                             (self.steering_override and
                              abs(CS.out.steeringAngleDeg - apply_angle) > CONTINUED_OVERRIDE_ANGLE and
                              not CS.out.standstill)

    if CS.hands_on_level > 0:
      self.last_hands_nanos = now_nanos

    # Reset steering override when lateral control is inactive, OR
    # when hands have been off the wheel for more than 1 second
    if not lat_active or (now_nanos - self.last_hands_nanos > 1e9):
      self.steering_override = False

    lat_active = lat_active and not self.steering_override

    apply_angle = self.torque_blended_angle(apply_angle, CS.out.steeringTorque)

    return lat_active, apply_angle
