# VCU rev5 (KS9e) hardware vs firmware

Extracted from the Altium project `VCU rev5 for KS9e` with `tools/schematic`
(netlist built from the schematic geometry, not the PCB). Worth a second pair
of eyes against the real board before acting on it.

```sh
cd tools/schematic
python3 netlist.py ../../schematic/*.SchDoc > nets.json
python3 pins.py nets.json U14          # every Teensy pin
python3 trace.py nets.json A10_Buff    # follow a net out to the J1 harness
```

## Needs a decision

| | Schematic | Firmware (`parameters.hpp` / `main.hpp`) |
|---|---|---|
| **Pump low-side outputs** | `LOWSide1` = Teensy **6** -> Q16 -> J1.13, `LOWSide2` = **7** -> Q15 -> J1.25. Pin 5 is unconnected | `LOWSIDE1 5`, `LOWSIDE2 6`: one pump output does nothing, the other drives the wrong one. Rev4 leftover? |
| **Brake sensor** | J1.21 `BSE` -> filter -> ADS1018 **AIN2** (never read). J1.10 is a generic input -> Teensy **A10** | `adc bse(avr, A10)`: reads J1.10. Fine if the transducer is wired to J1.10, otherwise brake reads nothing |
| **BSE short-to-supply fault** | A10 front end: 4K7 + 2K0 series, 12K0 to GND (gain 0.642), BAV199 clamp. 5 V reads ~995 counts, a GLV short clamps at ~1023 | `MAX_BRAKE_PEDAL 1025` can never be reached, so T.4.3.4 short-high never trips. Needs a threshold between real full-press and ~995 (KNOWN ISSUE test in `test_pedals`) |
| **APPS names** | ADS AIN0 = `APPS_2` (J1.11), AIN1 = `APPS_1` (J1.12) | Channel 0 called APPS1, 1 called APPS2. Calibration was tuned on the car so it works; just swapped names |

## Pin map (Teensy U14)

| Teensy | Net | Goes to | Firmware |
|---|---|---|---|
| 0 / 1 | RX2 / TX2 | MAX33041 U13 -> J1.18/19 `2CANL/H` | CAN2, `inv_can` |
| 22 / 23 | TX1 / RX1 | MAX33041 U10 -> J1.17/16 `1CANL/H` | CAN1, `acc_can` |
| 31 / 30 | TX3 / RX3 | MAX33041 U15 -> J1.28/27 `3CANL/H` | CAN3, `daq_can` (OTA) |
| 6 / 7 | LOWSide1 / 2 | NMOS -> J1.13 / J1.25 | see above |
| 9 | RTD_Buzz_EN | ITS4100 high side -> J1.26 `RTDBuzz` | `BUZZER 9` ✓ |
| 10-13 | CS/MOSI/MISO/SCK | TXB0104 level shifter -> ADS1018 (5 V) | `ADC_CS 10` ✓ |
| 14 / 15 | A0 / A1 | divider + buffer <- J1.2 `SDC_NO` / J1.3 `SDC_COMMON` | not read (shutdown circuit sense) |
| 16 | A2 | 6K8/12K0 <- BSPD `FAULT_HIGH` | `BSPD_SENSE A2` ✓ |
| 20 | A6 | 6K8/12K0 <- BSPD `Hall_OK_HIGH` | `BSPD_HALL_SENSE A6` ✓ |
| 21 | A7 | 6K8/12K0 <- BSPD `BSE_OK_HIGH` | `BSPD_BSE_SENSE A7` ✓ |
| 24 | A10 | divider + buffer <- J1.10 | brake, see above |
| 25, 38-41 | A11, A14-A17 | divider + buffer <- J1.9, 23, 22, 31, 30 | not read (old `VCU_REV4` sense pins don't match) |
| 36 / 37 | CS2 / CS3 | open-drain outputs -> J1.8 / J1.29 | not used |

ADS1018 (U12): AIN0 `APPS_2` (J1.11), AIN1 `APPS_1` (J1.12), AIN2 `BSE`
(J1.21), AIN3 `Steering_Angle` (J1.20). J1.15 `HALL` goes to the BSPD only;
J1.4 `Reset_Button` resets the BSPD latch in hardware, the Teensy never sees it.

BSPD polarity check: the senses are `*_OK_HIGH` / `FAULT_HIGH` 5 V logic
through 6K8/12K0, so high reads ~990 counts, over the firmware's 800
threshold, and `update_bspd`'s mapping (A2 relay, A7 brake, A6 current) and
polarities line up.
