# Renode PythonPeripheral for the i.MX RT CCM_ANALOG PLL registers
# (0x400D8000..0x400D802F). Unlike flipflop.py it remembers what was written,
# so the Teensy core's PLL bring-up loops (which check several bits at once)
# terminate.
#
# Every register has SET/CLR/TOG aliases at +4/+8/+C, and PLLs report LOCK
# (bit 31) as soon as they're asked for.
LOCK = 1 << 31

if request.IsInit:
    regs = {}
elif request.IsRead:
    base = request.Offset & ~0xF
    request.Value = regs.get(base, 0) | LOCK
elif request.IsWrite:
    base = request.Offset & ~0xF
    alias = request.Offset & 0xF
    cur = regs.get(base, 0)
    if alias == 0x0:
        cur = request.Value
    elif alias == 0x4:
        cur |= request.Value
    elif alias == 0x8:
        cur &= ~request.Value
    elif alias == 0xC:
        cur ^= request.Value
    regs[base] = cur & 0xFFFFFFFF
