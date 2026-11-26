# Renode PythonPeripheral for the Cortex-M7 DWT block (0xE0001000), which
# Renode doesn't model. The Teensy core's delayMicroseconds() and micros()
# spin on CYCCNT (offset 0x4), so with it stuck at 0 they never return.
#
# CYCCNT advances ~1 us worth of 600 MHz cycles on every read. Not tied to
# emulated time, so delays are only roughly right, but they terminate.
CYCLES_PER_READ = 600

if request.IsInit:
    regs = {}
    cyccnt = 0
elif request.IsRead:
    if request.Offset == 0x4:
        cyccnt = (cyccnt + CYCLES_PER_READ) & 0xFFFFFFFF
        request.Value = cyccnt
    else:
        request.Value = regs.get(request.Offset, 0)
elif request.IsWrite:
    if request.Offset == 0x4:
        cyccnt = request.Value
    else:
        regs[request.Offset] = request.Value
