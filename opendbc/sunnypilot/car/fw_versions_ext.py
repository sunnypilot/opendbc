import os
from enum import StrEnum
from typing import Any

from opendbc.car.common.basedir import BASEDIR

INTERFACE_EXT_ATTR_FILE = {
  "FW_VERSIONS_EXT": "fingerprints",
}


def get_interface_ext_attr(attr: str, combine_brands: bool = False, ignore_none: bool = False) -> dict[str | StrEnum, Any]:
  # read all the folders in opendbc/car and return a dict where:
  # - keys are all the car models or brand names
  # - values are attr values from all car folders
  result = {}
  for car_folder in sorted([x[0] for x in os.walk(BASEDIR)]):
    try:
      brand_name = car_folder.split('/')[-1]
      brand_values = __import__(f'opendbc.sunnypilot.car.{brand_name}.{INTERFACE_EXT_ATTR_FILE.get(attr, "values")}', fromlist=[attr])
      if hasattr(brand_values, attr) or not ignore_none:
        attr_data = getattr(brand_values, attr, None)
      else:
        continue

      if combine_brands:
        if isinstance(attr_data, dict):
          for f, v in attr_data.items():
            result[f] = v
      else:
        result[brand_name] = attr_data
    except (ImportError, OSError):
      pass

  return result


def merge_fw_versions(fw_versions):
  fw_versions_ext = get_interface_ext_attr("FW_VERSIONS_EXT", ignore_none=True)
  for c, e in fw_versions_ext.items():
    if c not in fw_versions:
      fw_versions[c] = {}

    for ecu, versions in e.items():
      if ecu not in fw_versions[c]:
        fw_versions[c][ecu] = []

      for v in versions:
        if v not in fw_versions[c][ecu]:
          fw_versions[c][ecu].append(v)


if __name__ == "__main__":
  from opendbc.car.hyundai.values import CAR
  from opendbc.car.hyundai.fingerprints import FW_VERSIONS

  # Merge the extensions into FW_VERSIONS
  merge_fw_versions(FW_VERSIONS)

  # Specify which car you want to print
  target_car = CAR.KIA_NIRO_EV

  if target_car in FW_VERSIONS:
    print(f"\n--- Firmware versions for {target_car} ---")
    for ecu, versions in FW_VERSIONS[target_car].items():
      print(f"\nECU: {ecu}")
      for v in versions:
        # Printing as hex and str for better readability
        print(f"  {v} ({v.hex()})")
    print(f"\nTotal ECUs: {len(FW_VERSIONS[target_car])}")
  else:
    print(f"Car {target_car} not found in FW_VERSIONS")

  # Print available cars for reference
  print(f"\nAvailable cars in FW_VERSIONS: {sorted(list(FW_VERSIONS.keys()))}")
