// The i.MX RT1062's RTWDOG (WDOG3) and the reset cause bit in the SRC, which
// the 1064 platform only has as a "ticker" stub and a tag.
//
// RTWDOG (i.MX RT1060 RM chapter 58), as much as the bootloader and firmware
// use: unlock (0xD928C520, or 0xC520 then 0xD928) opens the registers for
// one configuration; CS[UPDATE] = 0 makes that configuration final until a
// reset; refresh (0xB480A602, or 0xA602 then 0xB480) restarts the count. The
// counter runs from the 32.768 kHz LPO (CS[PRES] divides by 256) and on
// reaching TOVAL resets the machine. Out of reset it's disabled, as the boot
// ROM leaves it when it starts code from flash on a Teensy.
//
// IMXRT_SRC: SRSR (0x008) bit 7, WDOG3_RST_B, says the last reset was the
// watchdog. Write 1 to clear. It survives the reset it describes.
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class IMXRT_RTWDOG : IDoubleWordPeripheral, IKnownSize
    {
        public IMXRT_RTWDOG(IMachine machine)
        {
            this.machine = machine;
            counter = new LimitTimer(machine.ClockSource, LpoHz, this, "counter",
                limit: 0x400, direction: Direction.Ascending, enabled: false,
                workMode: WorkMode.OneShot, eventEnabled: true);
            counter.LimitReached += TimedOut;
            Reset();
        }

        public long Size => 0x4000;

        // The SRC's WDOG3_RST_B: not cleared by Reset(), like the hardware
        public bool ResetByWatchdog { get; set; }

        public void Reset()
        {
            counter.Enabled = false;
            counter.Value = 0;
            cs = CmdEn32; // disabled, as the boot ROM leaves it
            toval = 0x400;
            win = 0;
            configured = false;
            unlocked = true; // all registers are unlocked by reset
            lastCntWrite = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Cs:
                return cs | (unlocked ? Ulk : 0) | (configured ? Rcs : 0);
            case Cnt:
                return (uint)counter.Value;
            case Toval:
                return toval;
            case Win:
                return win;
            default:
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Cnt:
                WriteCnt(value);
                return;
            case Toval:
                if(CanConfigure("TOVAL"))
                {
                    toval = value & 0xFFFF;
                }
                return;
            case Win:
                if(CanConfigure("WIN"))
                {
                    win = value & 0xFFFF;
                }
                return;
            case Cs:
                if(!CanConfigure("CS"))
                {
                    return;
                }
                // FLG is write 1 to clear; ULK and RCS are read only
                cs = value & ~(Ulk | Rcs | Flg);
                configured = true;
                unlocked = false;
                Apply();
                return;
            }
        }

        private void WriteCnt(uint value)
        {
            var previous = lastCntWrite;
            lastCntWrite = value;
            if(value == 0xD928C520 || (previous == 0xC520 && value == 0xD928))
            {
                unlocked = true;
            }
            else if(value == 0xB480A602 || (previous == 0xA602 && value == 0xB480))
            {
                counter.Value = 0;
            }
        }

        private bool CanConfigure(string register)
        {
            if(unlocked && (!configured || (cs & Update) != 0))
            {
                return true;
            }
            this.Log(LogLevel.Warning, "Write to {0} ignored: {1}", register,
                unlocked ? "configured with UPDATE = 0, until a reset" : "not unlocked");
            return false;
        }

        private void Apply()
        {
            if(((cs >> 8) & 3) != 1)
            {
                this.Log(LogLevel.Warning, "Clock source {0} isn't modelled, counting at the LPO's 32.768 kHz", (cs >> 8) & 3);
            }
            counter.Frequency = (cs & Pres) != 0 ? LpoHz / 256 : LpoHz;
            counter.Limit = toval == 0 ? 1 : toval;
            counter.Value = 0;
            counter.Enabled = (cs & En) != 0;
            this.Log(LogLevel.Info, "{0}, timeout {1} ms", counter.Enabled ? "Running" : "Disabled",
                counter.Limit * 1000 / counter.Frequency);
        }

        private void TimedOut()
        {
            this.Log(LogLevel.Warning, "Not refreshed in time: resetting the machine");
            ResetByWatchdog = true;
            machine.RequestReset();
        }

        private readonly IMachine machine;
        private readonly LimitTimer counter;
        private uint cs;
        private uint toval;
        private uint win;
        private bool configured;
        private bool unlocked;
        private uint lastCntWrite;

        private const ulong LpoHz = 32768;
        private const long Cs = 0x0, Cnt = 0x4, Toval = 0x8, Win = 0xC;
        private const uint Update = 1u << 5, En = 1u << 7, Rcs = 1u << 10,
            Ulk = 1u << 11, Pres = 1u << 12, CmdEn32 = 1u << 13, Flg = 1u << 14;
    }
}

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class IMXRT_SRC : IDoubleWordPeripheral, IKnownSize
    {
        public IMXRT_SRC(Timers.IMXRT_RTWDOG rtwdog)
        {
            this.rtwdog = rtwdog;
        }

        public long Size => 0x4000;

        public void Reset()
        {
            regs.Clear();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == Srsr)
            {
                return rtwdog.ResetByWatchdog ? Wdog3RstB : 0;
            }
            return regs.TryGetValue(offset, out var v) ? v : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == Srsr)
            {
                if((value & Wdog3RstB) != 0)
                {
                    rtwdog.ResetByWatchdog = false;
                }
                return;
            }
            regs[offset] = value;
        }

        private readonly Timers.IMXRT_RTWDOG rtwdog;
        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();

        private const long Srsr = 0x008;
        private const uint Wdog3RstB = 1u << 7;
    }
}
