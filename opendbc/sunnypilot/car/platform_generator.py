import json
import os
from natsort import natsorted

from common.basedir import BASEDIR
from opendbc.car.docs import get_all_footnotes, get_params_for_docs
from opendbc.car.values import PLATFORMS


def build_sorted_car_list(platforms) -> dict[str, str]:
  cars: dict[str, str] = {}
  footnotes = get_all_footnotes()
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
  car_name_dict = build_sorted_car_list(PLATFORMS)
  car_list_file = os.path.join(BASEDIR, "opendbc/sunnypilot/car/car_list.json")

  with open(car_list_file, "w") as json_file:
    json.dump(car_name_dict, json_file, indent=2, ensure_ascii=False)
    json_file.write('\n')
  print(f"Generated and written to {car_list_file}")
