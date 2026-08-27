import math
import random
from types import SimpleNamespace
import unittest

from opendbc.can import CANParser
from opendbc.car.structs import CarControl, CarControlSP, CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC, CarControllerParams, FW_QUERY_CONFIG, FW_PATTERN, get_platform_codes
from opendbc.car.ford.fingerprints import FW_VERSIONS
from opendbc.testing import fuzzy_test, parameterized

Ecu = CarParams.Ecu


ECU_ADDRESSES = {
  Ecu.eps: 0x730,          # Power Steering Control Module (PSCM)
  Ecu.abs: 0x760,          # Anti-Lock Brake System (ABS)
  Ecu.fwdRadar: 0x764,     # Cruise Control Module (CCM)
  Ecu.fwdCamera: 0x706,    # Image Processing Module A (IPMA)
  Ecu.engine: 0x7E0,       # Powertrain Control Module (PCM)
  Ecu.shiftByWire: 0x732,  # Gear Shift Module (GSM)
  Ecu.debug: 0x7D0,        # Accessory Protocol Interface Module (APIM)
}


ECU_PART_NUMBER = {
  Ecu.eps: [
    b"14D003",
  ],
  Ecu.abs: [
    b"2D053",
  ],
  Ecu.fwdRadar: [
    b"14D049",
  ],
  Ecu.fwdCamera: [
    b"14F397",  # Ford Q3
    b"14H102",  # Ford Q4
  ],
}


class TestFordFW(unittest.TestCase):
  def test_fw_query_config(self):
    for (ecu, addr, subaddr) in FW_QUERY_CONFIG.extra_ecus:
      assert ecu in ECU_ADDRESSES, "Unknown ECU"
      assert addr == ECU_ADDRESSES[ecu], "ECU address mismatch"
      assert subaddr is None, "Unexpected ECU subaddress"

  @parameterized("car_model, fw_versions", FW_VERSIONS.items())
  def test_fw_versions(self, car_model, fw_versions):
    for (ecu, addr, subaddr), fws in fw_versions.items():
      assert ecu in ECU_PART_NUMBER, "Unexpected ECU"
      assert addr == ECU_ADDRESSES[ecu], "ECU address mismatch"
      assert subaddr is None, "Unexpected ECU subaddress"

      for fw in fws:
        assert len(fw) == 24, "Expected ECU response to be 24 bytes"

        match = FW_PATTERN.match(fw)
        assert match is not None, f"Unable to parse FW: {fw!r}"
        if match:
          part_number = match.group("part_number")
          assert part_number in ECU_PART_NUMBER[ecu], f"Unexpected part number for {fw!r}"

        codes = get_platform_codes([fw])
        assert 1 == len(codes), f"Unable to parse FW: {fw!r}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([
      b"JX6A-14C204-BPL\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"NZ6T-14F397-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"PJ6T-14H102-ABJ\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"LB5A-14C204-EAC\x00\x00\x00\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {(b"X6A", b"J"), (b"Z6T", b"N"), (b"J6T", b"P"), (b"B5A", b"L")}

  def test_fuzzy_match(self):
    for platform, fw_by_addr in FW_VERSIONS.items():
      # Ensure there's no overlaps in platform codes
      for _ in range(20):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(fw_versions)
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
        assert matches == {platform}

  def test_match_fw_fuzzy(self):
    offline_fw = {
      (Ecu.eps, 0x730, None): [
        b"L1MC-14D003-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"L1MC-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      (Ecu.abs, 0x760, None): [
        b"L1MC-2D053-BA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"L1MC-2D053-BD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      (Ecu.fwdRadar, 0x764, None): [
        b"LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"LB5T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      # We consider all model year hints for ECU, even with different platform codes
      (Ecu.fwdCamera, 0x706, None): [
        b"LB5T-14F397-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"NC5T-14F397-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
    }
    expected_fingerprint = CAR.FORD_EXPLORER_MK6

    # ensure that we fuzzy match on all non-exact FW with changed revisions
    live_fw = {
      (0x730, None): {b"L1MC-14D003-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x760, None): {b"L1MC-2D053-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x764, None): {b"LB5T-14D049-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x706, None): {b"LB5T-14F397-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
    }
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    assert candidates == {expected_fingerprint}

    # model year hint in between the range should match
    live_fw[(0x706, None)] = {b"MB5T-14F397-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw,})
    assert candidates == {expected_fingerprint}

    # unseen model year hint should not match
    live_fw[(0x760, None)] = {b"M1MC-2D053-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    assert len(candidates) == 0, "Should not match new model year hint"


class TestFordPathActuators(unittest.TestCase):
  def test_lateral_path_round_trip(self):
    path = CarControlSP().fordLateralPath
    path.pathOffset = 0.3
    path.pathAngle = -0.2
    path.curvature = 0.01
    path.curvatureRate = -0.0004
    path.valid = True
    assert math.isclose(path.pathOffset, 0.3, rel_tol=1e-6)
    assert math.isclose(path.pathAngle, -0.2, rel_tol=1e-6)
    assert math.isclose(path.curvature, 0.01, rel_tol=1e-6)
    assert math.isclose(path.curvatureRate, -0.0004, rel_tol=1e-6)
    assert path.valid

  def test_controller_packs_path(self):
    CP = CarInterface.get_non_essential_params(CAR.FORD_F_150_LIGHTNING_MK1)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.FORD_F_150_LIGHTNING_MK1)
    controller = CarController(DBC[CP.carFingerprint], CP, CP_SP)
    controller.frame = CarControllerParams.STEER_STEP
    controller.apply_curvature_last = 0.001

    CC = CarControl(latActive=True)
    CC_SP = CarControlSP()
    CC_SP.fordLateralPath.valid = True
    CC_SP.fordLateralPath.pathOffset = 0.4
    CC_SP.fordLateralPath.pathAngle = 0.1
    CC_SP.fordLateralPath.curvature = 0.001
    CC_SP.fordLateralPath.curvatureRate = 0.0002
    CC.hudControl.leadDistanceBars = 0

    CS = SimpleNamespace(
      out=SimpleNamespace(
        cruiseState=SimpleNamespace(available=False, standstill=False),
        vEgoRaw=7.0,
        vEgo=7.0,
        yawRate=0.0,
      ),
      buttons_stock_values={},
      acc_tja_status_stock_values={"Tja_D_Stat": 0},
      lkas_status_stock_values={},
    )
    controller.lkas_enabled_last = True
    controller.lead_distance_bars_last = 0

    output, can_sends = controller.update(CC.as_reader(), CC_SP, CS, 0)
    assert math.isclose(output.curvature, 0.001, rel_tol=1e-6)

    parser = CANParser("ford_lincoln_base_pt", [("LateralMotionControl2", 0)], 0)
    parser.update([0, can_sends])
    msg = parser.vl["LateralMotionControl2"]
    assert msg["LatCtl_D2_Rq"] == 2
    assert math.isclose(msg["LatCtlPathOffst_L_Actl"], -0.4, abs_tol=0.01)
    assert math.isclose(msg["LatCtlPath_An_Actl"], -0.1, abs_tol=0.001)
    assert math.isclose(msg["LatCtlCurv_No_Actl"], -0.001, abs_tol=1e-5)
    assert math.isclose(msg["LatCtlCrv_NoRate2_Actl"], -0.0002, abs_tol=2e-6)

    CC_SP.fordLateralPath.valid = False
    controller.frame = 2 * CarControllerParams.STEER_STEP
    output, can_sends = controller.update(CC.as_reader(), CC_SP, CS, 0)
    assert output.curvature == 0.0

    parser.update([0, can_sends])
    msg = parser.vl["LateralMotionControl2"]
    assert msg["LatCtl_D2_Rq"] == 2
    assert msg["LatCtlPathOffst_L_Actl"] == 0.0
    assert msg["LatCtlPath_An_Actl"] == 0.0
    assert msg["LatCtlCurv_No_Actl"] == 0.0
    assert msg["LatCtlCrv_NoRate2_Actl"] == 0.0

    CC.latActive = False
    controller.frame = 3 * CarControllerParams.STEER_STEP
    controller.lkas_enabled_last = False
    output, can_sends = controller.update(CC.as_reader(), CC_SP, CS, 0)
    parser.update([0, can_sends])
    msg = parser.vl["LateralMotionControl2"]
    assert msg["LatCtl_D2_Rq"] == 3

  def test_controller_unloads_c2_without_dropping_path(self):
    CP = CarInterface.get_non_essential_params(CAR.FORD_F_150_LIGHTNING_MK1)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.FORD_F_150_LIGHTNING_MK1)
    controller = CarController(DBC[CP.carFingerprint], CP, CP_SP)
    controller.frame = CarControllerParams.STEER_STEP
    controller.apply_curvature_last = 0.001

    CC = CarControl(latActive=True)
    CC_SP = CarControlSP()
    CC_SP.fordLateralPath.valid = True
    CC_SP.fordLateralPath.pathOffset = 0.4
    CC_SP.fordLateralPath.pathAngle = 0.1
    CC_SP.fordLateralPath.curvature = 0.0
    CC.hudControl.leadDistanceBars = 0

    CS = SimpleNamespace(
      out=SimpleNamespace(
        cruiseState=SimpleNamespace(available=False, standstill=False),
        vEgoRaw=35.0,
        vEgo=35.0,
        yawRate=0.0,
      ),
      buttons_stock_values={},
      acc_tja_status_stock_values={"Tja_D_Stat": 0},
      lkas_status_stock_values={},
    )
    controller.lkas_enabled_last = True
    controller.lead_distance_bars_last = 0

    output, can_sends = controller.update(CC.as_reader(), CC_SP, CS, 0)
    assert 0.0 < output.curvature < 0.001

    parser = CANParser("ford_lincoln_base_pt", [("LateralMotionControl2", 0)], 0)
    parser.update([0, can_sends])
    msg = parser.vl["LateralMotionControl2"]
    assert math.isclose(msg["LatCtlPathOffst_L_Actl"], -0.4, abs_tol=0.01)
    assert math.isclose(msg["LatCtlPath_An_Actl"], -0.1, abs_tol=0.001)
    assert 0.0 < -msg["LatCtlCurv_No_Actl"] < 0.001
