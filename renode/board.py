"""The VCU rev5 pedal inputs, from the schematic (docs/hardware.md), as the
ADCs see them: give a voltage at the J1 connector, or a fault, get counts.

  APPS1 (J1.11, net APPS_2*)  R50 100K to GND, R48 4K7, low-pass buffer ->
                              ADS1018 AIN0, +-2.048 V range: 1 mV/count
  APPS2 (J1.12, net APPS_1*)  R46 100K to GND, R43 4K7, ... -> AIN1
  brake (J1.10)               R115 4K7 + R116 2K0 series, 12K0 to GND,
                              BAV199 clamp -> Teensy A10, 10-bit at 3.3 V

  * the schematic's APPS names are swapped relative to the firmware's; these
    are the firmware's names (ADS channel 0 is what it calls APPS1)

The 100K pull-downs are what make an unplugged APPS read 0 V (T.4.2.3); the
brake's divider does the same for J1.10. Loading of the sensors by the
pull-downs is ignored (their outputs are low impedance).
"""
from __future__ import annotations

from dataclasses import dataclass

ADS_FULL_SCALE = 2047        # counts, +-2.048 V range
TEENSY_ADC_MAX = 1023        # 10 bit
TEENSY_VREF = 3.3
A10_DIVIDER = 12.0 / (4.7 + 2.0 + 12.0)

# The firmware's calibration (include/parameters.hpp), as sensor volts
APPS1_V = (0.670, 1.100)     # 0% .. 100% travel
APPS2_V = (0.450, 0.710)
BRAKE_COUNTS = (130, 922)


@dataclass(frozen=True)
class Input:
    """What's at the connector pin: a sensor voltage, or a wiring fault"""
    volts: float | None  # None: nothing connected

    def __str__(self):
        return "open" if self.volts is None else f"{self.volts:.3f} V"


UNPLUGGED = Input(None)
SHORT_TO_GROUND = Input(0.0)
SHORT_TO_5V = Input(5.0)      # the sensor supply
SHORT_TO_GLV = Input(12.0)    # the low voltage battery


def ads1018_counts(pin: Input) -> int:
    # Open: the 100K pull-down takes the input to 0 V
    v = 0.0 if pin.volts is None else pin.volts
    return max(0, min(ADS_FULL_SCALE, round(v * 1000)))


def a10_counts(pin: Input) -> int:
    v = 0.0 if pin.volts is None else pin.volts * A10_DIVIDER
    return max(0, min(TEENSY_ADC_MAX, round(v / TEENSY_VREF * TEENSY_ADC_MAX)))


def apps1(travel: float) -> Input:
    lo, hi = APPS1_V
    return Input(lo + travel * (hi - lo))


def apps2(travel: float) -> Input:
    lo, hi = APPS2_V
    return Input(lo + travel * (hi - lo))


def brake(travel: float) -> Input:
    lo, hi = BRAKE_COUNTS
    counts = lo + travel * (hi - lo)
    return Input(counts / TEENSY_ADC_MAX * TEENSY_VREF / A10_DIVIDER)
