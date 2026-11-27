// Cortex-M7 DWT (0xE0001000), which Renode doesn't model. The Teensy core's
// delayMicroseconds() and micros() spin on CYCCNT, so without it they hang.
//
// CYCCNT is derived from the CPU's executed instruction count, scaled so it
// ticks at the Teensy's 600 MHz relative to emulated time:
// cyclesPerInstruction = 600 / cpu.PerformanceInMips (100 by default).
// That keeps delays correct in emulated time, to within SkipMicroseconds
// (see FastForwardBusyWait). C# rather than a Python peripheral because
// delay loops read it constantly.
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class CortexM7_DWT : IDoubleWordPeripheral, IKnownSize
    {
        public CortexM7_DWT(TranslationCPU cpu, uint cyclesPerInstruction = 6)
        {
            this.cpu = cpu;
            this.cyclesPerInstruction = cyclesPerInstruction;
            Reset();
        }

        public long Size => 0x1000;

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case CtrlOffset:
                    return ctrl;
                case CyccntOffset:
                    FastForwardBusyWait();
                    return Cycles();
                default:
                    return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case CtrlOffset:
                    ctrl = value;
                    break;
                case CyccntOffset:
                    // Writing CYCCNT sets the current count
                    bias = value - RawCycles() - skippedCycles;
                    break;
            }
        }

        public void Reset()
        {
            ctrl = 0;
            bias = 0;
        }

        // A delay loop polls CYCCNT every few instructions until it passes a
        // target. Emulating that costs one slow peripheral access per
        // iteration (nearly all of the VCU's time: the ADS1018 driver waits
        // 4 x 500 us per loop). So when CYCCNT is read again within a few
        // instructions, jump the CPU ahead SkipMicroseconds instead: the loop
        // ends at most that late, and timers stay consistent because it's
        // real virtual time. SkipTime doesn't count as executed
        // instructions, so CYCCNT adds it itself.
        private void FastForwardBusyWait()
        {
            var executed = cpu.ExecutedInstructions;
            var polling = executed - lastRead < TightLoopInstructions;
            lastRead = executed;
            if(!polling)
            {
                return;
            }
            cpu.SkipTime(TimeInterval.FromMicroseconds(SkipMicroseconds));
            skippedCycles += (uint)(SkipMicroseconds * CpuMhz);
        }

        private uint RawCycles()
        {
            return (uint)(cpu.ExecutedInstructions * cyclesPerInstruction);
        }

        private uint Cycles()
        {
            return RawCycles() + skippedCycles + bias;
        }

        private readonly TranslationCPU cpu;
        private readonly uint cyclesPerInstruction;
        private uint ctrl;
        private uint bias;
        private ulong lastRead;
        private uint skippedCycles;
        private const ulong TightLoopInstructions = 32;
        private const ulong SkipMicroseconds = 20;
        private const ulong CpuMhz = 600;

        private const long CtrlOffset = 0x0;
        private const long CyccntOffset = 0x4;
    }
}
