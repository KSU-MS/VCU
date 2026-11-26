// TI ADS1018 12-bit SPI ADC, enough of it for the VCU's pedal/steering reads.
// Loaded at runtime by teensy41.resc (Renode compiles it) and attached to
// LPSPI4 chip select 0, which is how VCU rev5 wires it (through a TXB0104).
//
// Every 16 bit frame clocks a config word in and the last conversion out. A
// config with OS (bit 15) set starts a conversion on the channel in MUX
// (bits 14:12, 4..7 = single-ended AIN0..3), whose result comes back on the
// next frame. That's exactly what ads_helper() in adc_new.hpp does.
//
// Channel values are 12 bit codes (0..2047), as ads_helper returns them, so
// they compare directly with the calibration in parameters.hpp:
//   sysbus.lpspi4.ads1018 AIN0 900
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class ADS1018 : ISPIPeripheral
    {
        public ADS1018()
        {
            Reset();
        }

        // Defaults: pedals released, steering centred (VCU parameters.hpp)
        public int AIN0 { get; set; } = 670;  // APPS (firmware "APPS1")
        public int AIN1 { get; set; } = 450;  // APPS (firmware "APPS2")
        public int AIN2 { get; set; } = 0;    // BSE (not read by firmware)
        public int AIN3 { get; set; } = 1024; // steering

        public byte Transmit(byte data)
        {
            // Renode's LPSPI shifts a frame out least significant byte first
            // and assembles the response the same way, so stay consistent
            var response = (byte)(output >> (8 * index));
            if(index < 2)
            {
                input |= (uint)data << (8 * index);
            }
            index++;
            return response;
        }

        public void FinishTransmission()
        {
            if(index >= 2)
            {
                var config = (ushort)input;
                var mux = (config >> 12) & 0x7;
                if((config & 0x8000) != 0 && mux >= 4)
                {
                    conversion = (ushort)(Clamp(Channel(mux - 4)) << 4);
                    this.Log(LogLevel.Noisy, "Conversion on AIN{0}: 0x{1:X4}", mux - 4, conversion);
                }
                output = conversion;
            }
            index = 0;
            input = 0;
        }

        public void Reset()
        {
            index = 0;
            input = 0;
            output = 0;
            conversion = 0;
        }

        private int Channel(int n)
        {
            switch(n)
            {
                case 0: return AIN0;
                case 1: return AIN1;
                case 2: return AIN2;
                default: return AIN3;
            }
        }

        // 12 bit two's complement, left justified; the VCU clamps negatives to 0
        private static int Clamp(int code)
        {
            return code < -2048 ? -2048 : (code > 2047 ? 2047 : code);
        }

        private int index;
        private uint input;
        private uint output;
        private ushort conversion;
    }
}
