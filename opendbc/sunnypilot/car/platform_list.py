import json
import os
from natsort import natsorted

from opendbc.car.common.basedir import BASEDIR
from opendbc.car.docs import get_all_footnotes, get_params_for_docs
from opendbc.car.values import PLATFORMS

CAR_LIST_JSON_OUT = os.path.join(BASEDIR, "../", "sunnypilot", "car", "car_list.json")


def get_car_list() -> dict[str, str]:
  collected_footnote = get_all_footnotes()
  sorted_list: dict[str, str] = build_sorted_car_list(PLATFORMS, collected_footnote)
  return sorted_list


def build_sorted_car_list(platforms, footnotes) -> dict[str, str]:
  cars: dict[str, str] = {}
  for model, platform in platforms.items():
    car_docs = platform.config.car_docs
    CP = get_params_for_docs(model, platform)

    if CP.dashcamOnly or not len(car_docs):
      continue

    # A platform can include multiple car models
    for _car_docs in car_docs:
      if not hasattr(_car_docs, "row"):
        _car_docs.init_make(CP)
        _car_docs.init(CP, footnotes)
      cars[_car_docs.name] = model

  # Sort cars by make and model + year
  sorted_cars = natsorted(cars.keys(), key=lambda car: car.lower())
  sorted_car_list = {car: cars[car] for car in sorted_cars}
  return sorted_car_list


if __name__ == "__main__":
  platform_dict = get_car_list()

  with open(CAR_LIST_JSON_OUT, "w") as json_file:
    json.dump(platform_dict, json_file, indent=2, ensure_ascii=False)
  print(f"Generated and written to {CAR_LIST_JSON_OUT}")
