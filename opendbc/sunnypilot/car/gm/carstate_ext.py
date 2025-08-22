"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.gm.values import CAR

# FIXME-SP: use GMFlagsSP
CC_ONLY_CAR = {CAR.CHEVROLET_BOLT_2017, CAR.CHEVROLET_BOLT_2018, CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_EQUINOX_CC,
               CAR.CHEVROLET_SUBURBAN_CC, CAR.CADILLAC_CT6_CC, CAR.CHEVROLET_TRAILBLAZER_CC, CAR.CHEVROLET_MALIBU_CC,
               CAR.CADILLAC_XT5_CC}