import numpy as np


class Conversions:
  # Speed
  MPH_TO_KPH = 1.609344
  KPH_TO_MPH = 1. / MPH_TO_KPH # = 0.621371
  MS_TO_KPH = 3.6
  KPH_TO_MS = 1. / MS_TO_KPH # = 0.277778
  MS_TO_MPH = MS_TO_KPH * KPH_TO_MPH # = 2.236936
  MPH_TO_MS = MPH_TO_KPH * KPH_TO_MS # = 0.44704
  MS_TO_KNOTS = 1.9438
  KNOTS_TO_MS = 1. / MS_TO_KNOTS # = 0.514444

  # Angle
  DEG_TO_RAD = np.pi / 180.
  RAD_TO_DEG = 1. / DEG_TO_RAD

  # Mass
  LB_TO_KG = 0.453592
