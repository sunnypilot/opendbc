"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from abc import ABC, abstractmethod
import math
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags


CLUSTER_TRACK_SLOTS = ("lead", "lead_left", "lead_right")


class ClusterRadarTrackSelector:
  CENTER_HALF_WIDTH = 1.75
  STATIONARY_SPEED_THRESHOLD = 1.0
  SWITCH_DISTANCE_MARGIN = 3.0

  def __init__(self):
    self._track_ids = {slot: None for slot in CLUSTER_TRACK_SLOTS}

  def reset(self) -> dict[str, structs.CarControlSP.RadarTrack]:
    self._track_ids = {slot: None for slot in CLUSTER_TRACK_SLOTS}
    return {}

  @staticmethod
  def _range(track: structs.CarControlSP.RadarTrack) -> float:
    return abs(track.dRel)

  def _select(self, slot: str, candidates: list[structs.CarControlSP.RadarTrack], used_ids: set[int]):
    available = [track for track in candidates if track.trackId not in used_ids]
    if not available:
      self._track_ids[slot] = None
      return None

    closest = min(available, key=self._range)
    current = next((track for track in available if track.trackId == self._track_ids[slot]), None)
    selected = current if current is not None and self._range(current) <= self._range(closest) + self.SWITCH_DISTANCE_MARGIN else closest
    self._track_ids[slot] = selected.trackId
    used_ids.add(selected.trackId)
    return selected

  def update(self, tracks: list[structs.CarControlSP.RadarTrack], v_ego: float) -> dict[str, structs.CarControlSP.RadarTrack]:
    moving_tracks = [
      track for track in tracks
      if math.isfinite(track.dRel) and math.isfinite(track.yRel) and math.isfinite(track.vRel)
      and abs(v_ego + track.vRel) > self.STATIONARY_SPEED_THRESHOLD
    ]
    center = [track for track in moving_tracks if track.dRel >= 0 and abs(track.yRel) <= self.CENTER_HALF_WIDTH]
    left = [track for track in moving_tracks if track.dRel >= 0 and track.yRel > self.CENTER_HALF_WIDTH]
    right = [track for track in moving_tracks if track.dRel >= 0 and track.yRel < -self.CENTER_HALF_WIDTH]

    used_ids: set[int] = set()
    candidates = {
      "lead": center,
      "lead_left": left,
      "lead_right": right,
    }
    selected = {slot: self._select(slot, candidates[slot], used_ids) for slot in CLUSTER_TRACK_SLOTS}
    return {slot: track for slot, track in selected.items() if track is not None}


class LeadData(ABC):
  def __init__(self, object_gap: int, lead_distance: float, lead_rel_speed: float, lead_visible: bool):
    self.object_gap = object_gap
    self.lead_distance = lead_distance
    self.lead_rel_speed = lead_rel_speed
    self.lead_visible = lead_visible

  @property
  @abstractmethod
  def object_rel_gap(self) -> int:
    raise NotImplementedError("Subclasses must implement this method")


class CanLeadData(LeadData):
  @property
  def object_rel_gap(self) -> int:
    return 0 if self.lead_distance == 0 else 2 if self.lead_rel_speed < -0.2 else 1


class CanFdLeadData(LeadData):
  @property
  def object_rel_gap(self) -> int:
    return 0 if not self.lead_visible else 2 if self.lead_rel_speed < 0 else 1


def _hysteresis_update(current, new_value, counter, threshold):
  """
  Updates a value based on a hysteresis threshold mechanism. This function
  compares a new value against the current value and uses a counter to detect
  when a transition should occur, avoiding rapid oscillations between states.
  A new value will only be adopted if it differs from the current value and
  the counter reaches the specified threshold.

  :param current: The current value being tracked.
  :param new_value: The potential new value to compare against the current.
  :param counter: The count of consecutive different values encountered.
  :param threshold: The minimum count required before switching to the new value.
  :return: A tuple containing:
           - The updated current value, which is either the original current
             value or the new value if the hysteresis condition was met.
           - The updated counter, reset to 0 if the new value was adopted,
             or incremented by 1 otherwise.
  """
  if new_value == current:
    return current, 0
  counter += 1
  return (new_value, 0) if counter >= threshold else (current, counter)


class LeadDataCarController:
  # Hysteresis parameters
  LEAD_HYSTERESIS_FRAMES: int = 50

  def __init__(self, CP: structs.CarParams):
    self.CP = CP

    self.lead_one = {}
    self.lead_two = {}

    self._lead_on_counter = 0
    self._lead_off_counter = 0
    self.lead_visible = False
    self.gap_counter = 0
    self.object_gap = 0
    self.lead_distance = 0
    self.lead_rel_speed = 0
    self.radar_tracks_active = False
    self.cluster_track_slots: dict[str, structs.CarControlSP.RadarTrack] = {}
    self.cluster_track_selector = ClusterRadarTrackSelector()

  def _update_object_gap(self, lead_distance: float | None):
    new_gap = 5  # Default gap value if no lead distance is provided
    if lead_distance is None or lead_distance == 0:
      new_gap = 0
    elif lead_distance < 20:
      new_gap = 2
    elif lead_distance < 25:
      new_gap = 3
    elif lead_distance < 30:
      new_gap = 4

    self.object_gap, self.gap_counter = _hysteresis_update(self.object_gap, new_gap, self.gap_counter, self.LEAD_HYSTERESIS_FRAMES)

  def _update_lead_visible_hysteresis(self, raw_lead_visible: bool):
    counter = self._lead_on_counter if raw_lead_visible else self._lead_off_counter
    self.lead_visible, counter = _hysteresis_update(self.lead_visible, raw_lead_visible, counter, self.LEAD_HYSTERESIS_FRAMES)

    if raw_lead_visible:
      self._lead_on_counter = counter
      self._lead_off_counter = 0  # reset opposite counter
    else:
      self._lead_off_counter = counter
      self._lead_on_counter = 0  # reset opposite counter

  def update(self, CC_SP: structs.CarControlSP, v_ego: float = 0.0) -> None:
    self.lead_one = CC_SP.leadOne
    self.lead_two = CC_SP.leadTwo
    self.radar_tracks_active = CC_SP.radarTracksActive
    self.cluster_track_slots = self.cluster_track_selector.update(CC_SP.radarTracks, v_ego) \
      if self.radar_tracks_active else self.cluster_track_selector.reset()

    self.lead_distance = self.lead_one.dRel
    self.lead_rel_speed = self.lead_one.vRel
    self._update_lead_visible_hysteresis(self.lead_one.status)
    self._update_object_gap(self.lead_distance)

  @property
  def lead_data(self) -> CanLeadData | CanFdLeadData:
    if self.CP.flags & HyundaiFlags.CANFD:
      return CanFdLeadData(self.object_gap, self.lead_distance, self.lead_rel_speed, self.lead_visible)

    return CanLeadData(self.object_gap, self.lead_distance, self.lead_rel_speed, self.lead_visible)
