// i.MX RT FlexSPI controller, just the IP-command path the Teensy core uses
// to erase and program its own QSPI flash (eeprom.c: eepromemu_flash_write,
// eepromemu_flash_erase_sector, flash_wait), which is also what FlasherX and
// the VCU's OTA staging go through.
//
// Commands run against plain memory at the FlexSPI's AHB window (where the
// CPU reads/executes the flash from), with NOR rules: sector erase sets 4 KB
// to 0xFF, page program can only clear bits. Only the first CMD_SDR
// instruction of the LUT sequence matters (the flash command byte):
//
//   06 write enable    05 read status (never busy)    20 sector erase
//   02/32 page program (data pushed through TFDR, 8 bytes per IPTXWE)
//
// With hasDevice false every command completes and reads return 0, which
// is how the Teensy's PSRAM probe on FlexSPI2 concludes there's no PSRAM.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class TeensyFlexSPI : IDoubleWordPeripheral, IBytePeripheral, IKnownSize
    {
        public TeensyFlexSPI(IMachine machine, ulong ahbBase, bool hasDevice = true)
        {
            this.machine = machine;
            this.ahbBase = ahbBase;
            this.hasDevice = hasDevice;
            Reset();
        }

        public long Size => 0x4000;

        public void Reset()
        {
            regs.Clear();
            lut = new uint[64];
            txFifo.Clear();
            rxData = 0;
            intr = 0;
            programRemaining = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == Intr)
            {
                return intr;
            }
            if(offset == Sts0)
            {
                return 0x3; // sequencer and arbitrator idle
            }
            if(offset == Mcr0)
            {
                return Get(Mcr0) & ~1u; // SWRESET completes instantly
            }
            if(offset >= Rfdr && offset < Rfdr + 0x80)
            {
                return offset == Rfdr ? rxData : 0;
            }
            if(offset >= Lut && offset < Lut + 0x100)
            {
                return lut[(offset - Lut) / 4];
            }
            return Get(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == Intr)
            {
                intr &= ~value; // write one to clear
                if((value & IpTxWe) != 0)
                {
                    TxWatermarkConsumed();
                }
                return;
            }
            if(offset >= Tfdr && offset < Tfdr + 0x80)
            {
                for(var i = 0; i < 4; i++)
                {
                    txFifo.Add((byte)(value >> (8 * i)));
                }
                return;
            }
            if(offset >= Lut && offset < Lut + 0x100)
            {
                lut[(offset - Lut) / 4] = value;
                return;
            }
            if(offset == IpTxFcr && (value & 1) != 0)
            {
                txFifo.Clear();
            }
            if(offset == IpCmd && (value & 1) != 0)
            {
                Execute();
                return;
            }
            regs[offset] = value;
        }

        // memcpy into TFDR may use byte stores
        public byte ReadByte(long offset)
        {
            return (byte)(ReadDoubleWord(offset & ~3) >> (int)(8 * (offset & 3)));
        }

        public void WriteByte(long offset, byte value)
        {
            if(offset >= Tfdr && offset < Tfdr + 0x80)
            {
                txFifo.Add(value);
                return;
            }
            var word = ReadDoubleWord(offset & ~3);
            var shift = (int)(8 * (offset & 3));
            word = (word & ~(0xFFu << shift)) | ((uint)value << shift);
            WriteDoubleWord(offset & ~3, word);
        }

        private void Execute()
        {
            var ipcr1 = Get(IpCr1);
            var seq = (int)((ipcr1 >> 16) & 0xF);
            var size = (int)(ipcr1 & 0xFFFF);
            var address = Get(IpCr0) & 0xFFFFFF;
            var command = CommandByte(seq);

            if(!hasDevice)
            {
                rxData = 0;
                intr |= IpCmdDone;
                return;
            }

            switch(command)
            {
                case 0x06: // write enable
                    break;
                case 0x05: // read status: never busy
                    rxData = 0;
                    break;
                case 0x20: // 4 KB sector erase
                    var erased = new byte[4096];
                    for(var i = 0; i < erased.Length; i++)
                    {
                        erased[i] = 0xFF;
                    }
                    machine.SystemBus.WriteBytes(erased, ahbBase + (address & ~0xFFFu));
                    this.Log(LogLevel.Debug, "Erased sector at 0x{0:X6}", address & ~0xFFFu);
                    break;
                case 0x02:
                case 0x32: // page program, data arrives through TFDR
                    programAddress = address;
                    programRemaining = size;
                    txFifo.Clear();
                    intr |= IpTxWe; // ask for data
                    return; // IPCMDDONE once all of it has been written
                default:
                    this.Log(LogLevel.Debug, "Ignoring flash command 0x{0:X2}", command);
                    break;
            }
            intr |= IpCmdDone;
        }

        private void TxWatermarkConsumed()
        {
            if(programRemaining <= 0)
            {
                return;
            }
            var n = Math.Min(Math.Min(txFifo.Count, 8), programRemaining);
            if(n > 0)
            {
                // NOR program: can only clear bits
                var target = ahbBase + programAddress;
                var current = machine.SystemBus.ReadBytes(target, n);
                for(var i = 0; i < n; i++)
                {
                    current[i] &= txFifo[i];
                }
                machine.SystemBus.WriteBytes(current, target);
                txFifo.RemoveRange(0, Math.Min(8, txFifo.Count));
                programAddress += (uint)n;
                programRemaining -= n;
            }
            if(programRemaining > 0)
            {
                intr |= IpTxWe;
            }
            else
            {
                intr |= IpCmdDone;
            }
        }

        // First CMD_SDR (opcode 1) operand in the LUT sequence
        private byte CommandByte(int seq)
        {
            for(var i = 0; i < 8; i++)
            {
                var instr = (lut[seq * 4 + i / 2] >> (16 * (i % 2))) & 0xFFFF;
                if(((instr >> 10) & 0x3F) == 0x01)
                {
                    return (byte)(instr & 0xFF);
                }
            }
            return 0;
        }

        private uint Get(long offset)
        {
            return regs.TryGetValue(offset, out var v) ? v : 0;
        }

        private readonly IMachine machine;
        private readonly ulong ahbBase;
        private readonly bool hasDevice;
        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly List<byte> txFifo = new List<byte>();
        private uint[] lut;
        private uint rxData;
        private uint intr;
        private uint programAddress;
        private int programRemaining;

        private const long Mcr0 = 0x000;
        private const long Intr = 0x014;
        private const long IpCr0 = 0x0A0;
        private const long IpCr1 = 0x0A4;
        private const long IpCmd = 0x0B0;
        private const long IpTxFcr = 0x0BC;
        private const long Sts0 = 0x0E0;
        private const long Rfdr = 0x100;
        private const long Tfdr = 0x180;
        private const long Lut = 0x200;

        private const uint IpCmdDone = 1u << 0;
        private const uint IpTxWe = 1u << 6;
    }
}
