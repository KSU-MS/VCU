// i.MX RT FlexSPI controller, just the IP-command path the Teensy core uses
// to erase and program its own QSPI flash (eeprom.c: eepromemu_flash_write,
// eepromemu_flash_erase_sector, flash_wait), which is also what FlasherX and
// the VCU's OTA staging go through.
//
// Commands run against plain memory at the FlexSPI's AHB window (where the
// CPU reads/executes the flash from), behaving like the Teensy 4.1's Winbond
// W25Q64JV (datasheet rev J): erase sets bits to 1, page program can only
// clear them and wraps within its 256-byte page, both need a write enable
// first and leave the chip busy (status bit 0) for the datasheet time, in
// virtual time. Only the first CMD_SDR instruction of the LUT sequence
// matters (the flash command byte):
//
//   06 write enable    04 write disable    05 read status    9F JEDEC ID
//   20 / 52 / D8 erase 4 / 32 / 64 KB
//   02 / 32 page program (data pushed through TFDR, 8 bytes per IPTXWE)
//
// A command other than read status while busy, or an erase/program without
// write enable, is ignored by the real chip; here it's also logged as an
// error, so a firmware that doesn't wait shows up in the log.
//
//   tSE 45 / 400 ms, tBE1 120 / 1600 ms, tBE2 150 / 2000 ms, tPP 0.4 / 3 ms
//
// typical / max (WorstCaseTiming). The JV datasheet only gives tPP for a
// full page; the Teensy core programs 4 bytes at a time, so shorter
// programs take tBP1 + tBP2 per extra byte (30 + 2.5 us, max 50 + 12 us),
// from earlier W25Q64 datasheets, capped at tPP.
//
// With hasDevice false every command completes and reads return 0, which
// is how the Teensy's PSRAM probe on FlexSPI2 concludes there's no PSRAM.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Time;

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

        // Datasheet max instead of typical times, from the monitor:
        //   sysbus.flexspi1 WorstCaseTiming true
        public bool WorstCaseTiming { get; set; }

        // Chip erase (C7), instantly: the state of a new chip, before the
        // firmware is loaded
        public void ChipErase()
        {
            var erased = new byte[64 * 1024];
            for(var i = 0; i < erased.Length; i++)
            {
                erased[i] = 0xFF;
            }
            for(ulong offset = 0; offset < FlashSize; offset += (ulong)erased.Length)
            {
                machine.SystemBus.WriteBytes(erased, ahbBase + offset);
            }
        }

        // Commands the real chip would have ignored, see above
        public ulong Violations { get; private set; }

        public long Size => 0x4000;

        public void Reset()
        {
            regs.Clear();
            lut = new uint[64];
            txFifo.Clear();
            rxData = 0;
            intr = 0;
            programRemaining = 0;
            writeEnabled = false;
            busyUntil = TimeInterval.Empty;
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

            if(command == 0x05) // read status: WIP, WEL
            {
                rxData = Busy ? 0x3u : (writeEnabled ? 0x2u : 0x0u);
                if(Busy)
                {
                    // The firmware is spinning on WIP: jump the CPU to the end of
                    // the busy time instead of emulating the spin, which is
                    // MMIO-bound and would take minutes of wall time per second
                    // of erase. It still sees busy at least once.
                    var cpu = machine.SystemBus.GetCurrentCPU() as TranslationCPU;
                    cpu?.SkipTime(busyUntil - Now);
                }
                intr |= IpCmdDone;
                return;
            }
            if(Busy)
            {
                Violation("command 0x{0:X2} at 0x{1:X6} while busy for another {2} us",
                    command, address, (busyUntil - Now).TotalMicroseconds);
                intr |= IpCmdDone;
                return;
            }

            switch(command)
            {
                case 0x06: // write enable
                    writeEnabled = true;
                    break;
                case 0x04: // write disable
                    writeEnabled = false;
                    break;
                case 0x9F: // JEDEC ID: Winbond, W25Q64JV-IQ/JQ
                    rxData = 0x1740EF;
                    break;
                case 0x20:
                    Erase(address, 4 * 1024, 45, 400);
                    break;
                case 0x52:
                    Erase(address, 32 * 1024, 120, 1600);
                    break;
                case 0xD8:
                    Erase(address, 64 * 1024, 150, 2000);
                    break;
                case 0x02:
                case 0x32: // page program, data arrives through TFDR
                    if(!TakeWriteEnable(command, address))
                    {
                        break;
                    }
                    programAddress = address;
                    programRemaining = size;
                    programmed = 0;
                    txFifo.Clear();
                    intr |= IpTxWe; // ask for data
                    return; // IPCMDDONE once all of it has been written
                default:
                    this.Log(LogLevel.Debug, "Ignoring flash command 0x{0:X2}", command);
                    break;
            }
            intr |= IpCmdDone;
        }

        private void Erase(uint address, uint size, ulong typicalMs, ulong maxMs)
        {
            if(!TakeWriteEnable(0, address))
            {
                return;
            }
            var start = address & ~(size - 1);
            var erased = new byte[size];
            for(var i = 0; i < erased.Length; i++)
            {
                erased[i] = 0xFF;
            }
            machine.SystemBus.WriteBytes(erased, ahbBase + start);
            BusyFor(TimeInterval.FromMilliseconds(WorstCaseTiming ? maxMs : typicalMs));
            this.Log(LogLevel.Debug, "Erased {0} KB at 0x{1:X6}", size / 1024, start);
        }

        // Erase and program need a write enable, which they use up
        private bool TakeWriteEnable(byte command, uint address)
        {
            if(!writeEnabled)
            {
                Violation("erase/program 0x{0:X2} at 0x{1:X6} without write enable", command, address);
                return false;
            }
            writeEnabled = false;
            return true;
        }

        private void BusyFor(TimeInterval duration)
        {
            busyUntil = Now + duration;
        }

        private void Violation(string format, params object[] args)
        {
            Violations++;
            this.Log(LogLevel.Error, "W25Q64 would ignore " + format, args);
        }

        private TimeInterval Now => machine.LocalTimeSource.ElapsedVirtualTime;

        private bool Busy => Now < busyUntil;

        private void TxWatermarkConsumed()
        {
            if(programRemaining <= 0)
            {
                return;
            }
            var n = Math.Min(Math.Min(txFifo.Count, 8), programRemaining);
            if(n > 0)
            {
                // NOR program: can only clear bits; past the end of the
                // 256-byte page the address wraps to the start of it
                for(var i = 0; i < n; i++)
                {
                    var target = ahbBase + (programAddress & ~0xFFu) + ((programAddress + (uint)i) & 0xFFu);
                    var current = machine.SystemBus.ReadByte(target);
                    machine.SystemBus.WriteByte(target, (byte)(current & txFifo[i]));
                }
                txFifo.RemoveRange(0, Math.Min(8, txFifo.Count));
                programAddress = (programAddress & ~0xFFu) + ((programAddress + (uint)n) & 0xFFu);
                programRemaining -= n;
                programmed += n;
            }
            if(programRemaining > 0)
            {
                intr |= IpTxWe;
            }
            else
            {
                var us = WorstCaseTiming ? 50 + 12 * (programmed - 1) : 30 + 2.5 * (programmed - 1);
                BusyFor(TimeInterval.FromMicroseconds((ulong)Math.Min(us, WorstCaseTiming ? 3000 : 400)));
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
        private bool writeEnabled;
        private TimeInterval busyUntil;
        private int programmed;
        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly List<byte> txFifo = new List<byte>();
        private uint[] lut;
        private uint rxData;
        private uint intr;
        private uint programAddress;
        private int programRemaining;

        private const ulong FlashSize = 8 * 1024 * 1024; // W25Q64
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
