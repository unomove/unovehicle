# ruff: noqa: E501
""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.ecar.values import CAR

Ecu = CarParams.Ecu

FINGERPRINTS = {
  CAR.ECAR_E70: [
  #   {
  #   273: 8, 274: 8, 275: 8, 276: 8, 285: 8, \
  #     535: 8, 592: 8,  529: 8, 533: 8
  # }
  {576: 8, 592: 8, 593: 8, 594: 8, 624: 8, 625: 8, 626: 8, 630: 8, 602: 8, 603: 8, 529: 8, 531: 8, \
  533: 8, 535: 8, 545: 8, 640: 8, 641: 8, 530: 8, 532: 8, \
  534: 8, 536: 8, 537: 8, 544: 8, 627: 8, 628: 8, 629: 8}
  ],
}

FW_VERSIONS = {
  CAR.ECAR_E70: {
    (Ecu.engine, 0x720, None): [
      b'E70.0.0',
    ],
    (Ecu.debug, 0x721, None): [
      b'166bd860',
    ],
  },
}
