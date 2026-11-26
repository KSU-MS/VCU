// Cortex-M7 DWT (0xE0001000), which Renode doesn't model. The Teensy core's
// delayMicroseconds() and micros() spin on CYCCNT, so without it they hang.
//
// CYCCNT is derived from the CPU's executed instruction count, scaled so it
// ticks at the Teensy's 600 MHz relative to emulated time:
// cyclesPerInstruction = 600 / cpu.PerformanceInMips (100 by default).
// That keeps delays correct in emulated time. C# rather than a Python
// peripheral because delay loops read it constantly.
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

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
                    bias = value - RawCycles();
                    break;
            }
        }

        public void Reset()
        {
            ctrl = 0;
            bias = 0;
        }

        private uint RawCycles()
        {
            return (uint)(cpu.ExecutedInstructions * cyclesPerInstruction);
        }

        private uint Cycles()
        {
            return RawCycles() + bias;
        }

        private readonly TranslationCPU cpu;
        private readonly uint cyclesPerInstruction;
        private uint ctrl;
        private uint bias;

        private const long CtrlOffset = 0x0;
        private const long CyccntOffset = 0x4;
    }
}
