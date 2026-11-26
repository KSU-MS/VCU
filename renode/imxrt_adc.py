# Renode PythonPeripheral standing in for i.MX RT ADC1 (0x400C4000).
#
# Renode's IMXRT_ADC crashes on any channel above 7: TriggerConversion
# stores the result in cdata[channel] but cdata has one entry per trigger
# (8), not per channel (16). The Teensy reads A2/A6 on channels 12/15, so
# this replaces it with just what the Teensy core's analogRead() uses:
# calibration (GC.CAL), HC0 to start a conversion, HS.COCO0 and R0.
#
# Inputs are 10 bit counts (the Teensy default resolution), settable from
# the monitor at offset 0x1000 + 4*channel, e.g. press the brake:
#   sysbus WriteDoubleWord 0x400C4004 900        (A10 is channel 1)
HC0, HS, R0, GC, GS = 0x00, 0x20, 0x24, 0x48, 0x4C
CAL = 1 << 7
COCO0 = 1 << 0
INPUTS = 0x1000

if request.IsInit:
    regs = {}
    # VCU rev5 defaults: BSPD healthy, brake released
    inputs = {
        12: 0,    # A2  BSPD FAULT_HIGH, low = no fault
        15: 990,  # A6  BSPD Hall_OK_HIGH
        0: 990,   # A7  BSPD BSE_OK_HIGH
        1: 130,   # A10 brake, START_BRAKE_PEDAL
    }
elif request.IsRead:
    off = request.Offset
    if off >= INPUTS:
        request.Value = inputs.get((off - INPUTS) // 4, 0)
    elif off == R0:
        regs[HS] = regs.get(HS, 0) & ~COCO0
        request.Value = regs.get(R0, 0)
    else:
        request.Value = regs.get(off, 0)
elif request.IsWrite:
    off, val = request.Offset, request.Value
    if off >= INPUTS:
        inputs[(off - INPUTS) // 4] = val
    elif off == HC0:
        regs[HC0] = val
        channel = val & 0x1F
        if channel != 0x1F:  # 0x1F = conversion disabled
            regs[R0] = inputs.get(channel, 0) & 0xFFF
            regs[HS] = regs.get(HS, 0) | COCO0
    elif off == GC:
        regs[GC] = val & ~CAL  # calibration finishes instantly
        regs[GS] = 0           # and never fails
    else:
        regs[off] = val
