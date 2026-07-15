import unittest
from enum import IntFlag

from opendbc.sunnypilot.car.hyundai.lead_data_ext import (
  CanFdLeadData,
  CanLeadData,
  ClusterRadarTrackSelector,
  LeadDataCarController,
)
from opendbc.car import structs
from opendbc.car.hyundai.hyundaicanfd import update_ccnc_cluster_tracks
from opendbc.car.hyundai.values import HyundaiFlags


def make_carparams(flags: IntFlag = HyundaiFlags.LEGACY):
  cp = structs.CarParams()
  cp.carFingerprint = "HYUNDAI_SONATA"
  cp.flags = flags.value
  return cp


def make_carcontrolsp(leadDistance=10.0, leadRelSpeed=0.0, leadVisible=True):
  c = structs.CarControlSP()
  c.leadOne.dRel = leadDistance
  c.leadOne.vRel = leadRelSpeed
  c.leadOne.status = leadVisible
  return c


def make_radar_track(track_id, d_rel, y_rel, v_rel=0.0):
  return structs.CarControlSP.RadarTrack(trackId=track_id, dRel=d_rel, yRel=y_rel, vRel=v_rel)


class TestLeadDataCarController(unittest.TestCase):
  def test_update_object_gap(self):
    ctrl = LeadDataCarController(make_carparams())
    # Initial value should be 0
    self.assertEqual(ctrl.object_gap, 0)

    # Set to 15 (should become 2 after hysteresis)
    for _ in range(ctrl.LEAD_HYSTERESIS_FRAMES):
      ctrl._update_object_gap(15)
    self.assertEqual(ctrl.object_gap, 2)

    # Set to 22 (should become 3 after hysteresis)
    for _ in range(ctrl.LEAD_HYSTERESIS_FRAMES):
      ctrl._update_object_gap(22)
    self.assertEqual(ctrl.object_gap, 3)

    # Set to 0 (should become 0 after hysteresis)
    for _ in range(ctrl.LEAD_HYSTERESIS_FRAMES):
      ctrl._update_object_gap(0)
    self.assertEqual(ctrl.object_gap, 0)

  def test_update_lead_visible_hysteresis(self):
    ctrl = LeadDataCarController(make_carparams())
    ctrl._update_lead_visible_hysteresis(True)
    self.assertIsInstance(ctrl.lead_visible, bool)
    ctrl._update_lead_visible_hysteresis(False)
    self.assertIsInstance(ctrl.lead_visible, bool)

  def test_update(self):
    ctrl = LeadDataCarController(make_carparams())
    sp = make_carcontrolsp(leadDistance=25, leadRelSpeed=-0.5, leadVisible=True)
    ctrl.update(sp)
    self.assertEqual(ctrl.lead_distance, 25)
    self.assertEqual(ctrl.lead_rel_speed, -0.5)
    self.assertIsInstance(ctrl.lead_visible, bool)

  def test_lead_data_can(self):
    ctrl = LeadDataCarController(make_carparams())
    ctrl.object_gap = 1
    ctrl.lead_distance = 10
    ctrl.lead_rel_speed = -0.3
    ctrl.lead_visible = True
    ld = ctrl.lead_data
    self.assertIsInstance(ld, CanLeadData)
    self.assertEqual(ld.object_rel_gap, 2)

  def test_lead_data_canfd(self):
    ctrl = LeadDataCarController(make_carparams(HyundaiFlags.CANFD))
    ctrl.object_gap = 1
    ctrl.lead_distance = 10
    ctrl.lead_rel_speed = 1.0
    ctrl.lead_visible = True
    ld = ctrl.lead_data
    self.assertIsInstance(ld, CanFdLeadData)
    self.assertEqual(ld.object_rel_gap, 1)

  def test_cluster_radar_track_regions_and_stationary_filter(self):
    selector = ClusterRadarTrackSelector()
    tracks = [
      make_radar_track(1, 15, 0),
      make_radar_track(2, 25, 1),
      make_radar_track(3, 12, -3),
      make_radar_track(4, 14, 3),
      make_radar_track(5, -8, -2),
      make_radar_track(6, -9, 2),
      make_radar_track(7, 5, 0, -20),
    ]

    slots = selector.update(tracks, v_ego=20)

    self.assertEqual({slot: track.trackId for slot, track in slots.items()}, {
      "lead": 1,
      "lead_alt": 2,
      "lead_left": 3,
      "lead_right": 4,
      "lead_left_rear": 5,
      "lead_right_rear": 6,
    })

  def test_cluster_radar_track_selection_is_stable(self):
    selector = ClusterRadarTrackSelector()
    self.assertEqual(selector.update([make_radar_track(1, 10, 0)], 20)["lead"].trackId, 1)
    self.assertEqual(selector.update([make_radar_track(1, 12, 0), make_radar_track(2, 10, 0)], 20)["lead"].trackId, 1)
    self.assertEqual(selector.update([make_radar_track(1, 12, 0), make_radar_track(2, 5, 0)], 20)["lead"].trackId, 2)

  def test_cluster_radar_track_selection_prioritizes_matched_lead(self):
    selector = ClusterRadarTrackSelector()
    tracks = [make_radar_track(1, 10, 0), make_radar_track(2, 20, 0), make_radar_track(3, 30, 0)]

    slots = selector.update(tracks, v_ego=20, preferred_ids=(3,))

    self.assertEqual(slots["lead"].trackId, 3)
    self.assertEqual(slots["lead_alt"].trackId, 1)

  def test_update_ccnc_cluster_tracks(self):
    msg_162 = {}
    slots = {
      "lead": make_radar_track(1, 20, 0.5),
      "lead_left": make_radar_track(2, 30, -3),
      "lead_right_rear": make_radar_track(3, -40, 4),
    }

    update_ccnc_cluster_tracks(msg_162, slots, matched_track_ids={1})

    self.assertEqual(msg_162["LEAD"], 4)
    self.assertEqual(msg_162["LEAD_DISTANCE"], 20)
    self.assertEqual(msg_162["LEAD_LATERAL"], 0.5)
    self.assertEqual(msg_162["LEAD_LEFT"], 3)
    self.assertEqual(msg_162["LEAD_LEFT_LATERAL"], 3)
    self.assertEqual(msg_162["LEAD_RIGHT_REAR_STATUS"], 3)
    self.assertEqual(msg_162["LEAD_RIGHT_REAR_DISTANCE"], 25.5)
    self.assertEqual(msg_162["LEAD_ALT"], 0)
