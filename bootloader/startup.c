// What the i.MX RT1062 boot ROM needs at the start of flash, and the reset
// path up to main(). Bare metal: no Teensy core, so clocks, caches and
// peripherals are still as the ROM left them when we jump to the firmware,
// and its own ResetHandler runs exactly as it does from a cold boot.
//
// The FlexSPI configuration block is PJRC's (Teensyduino cores/teensy4/
// bootdata.c, Copyright (c) 2019 PJRC.COM, LLC, MIT-style licence: the
// notice in that file applies to this table).
#include <stdint.h>

extern uint32_t _estack, _flashimagelen;
extern uint32_t _sramfunc, _eramfunc, _sramfuncload;
extern uint32_t _sdata, _edata, _sdataload, _sbss, _ebss;

int main(void);

__attribute__((section(".bootdata"), used))
const uint32_t BootData[3] = {
    0x60000000,
    (uint32_t)&_flashimagelen,
    0,
};

void Reset_Handler(void);

__attribute__((section(".ivt"), used))
const uint32_t ImageVectorTable[8] = {
    0x432000D1,                 // header
    (uint32_t)&Reset_Handler,   // entry
    0,                          // reserved
    0,                          // DCD
    (uint32_t)BootData,         // boot data
    (uint32_t)ImageVectorTable, // self
    0,                          // CSF (no HAB)
    0,                          // reserved
};

// Serial NOR configuration for the W25Q64JV (quad read, 8 MB), as Teensy 4.1
__attribute__((section(".flashconfig"), used))
const uint32_t FlexSPI_NOR_Config[128] = {
    0x42464346, 0x56010000, 0, 0x00020101, // tag, version, -, sample clock
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0x01060401, 0, 0, // -, serial clock/pad type/device type
    0x00800000, 0, 0, 0, // sflashA1Size: 8 MB
    0, 0, 0, 0,
    0, 0, 0, 0,
    0x0A1804EB, 0x32041EFF, 0x00002601, 0, // LUT 0: quad I/O fast read
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // LUT index / reserved
    256, 4096, 1, 0,       // page size, sector size, IP clock
    0x00010000, 0, 0, 0,   // block size
    0, 0, 0, 0,
    0, 0, 0, 0,
};

static void Default_Handler(void) {
  for (;;)
    ;
}

// Only the reset vector matters: nothing here enables an interrupt, and a
// fault spins (the firmware never gets started, the USB bootloader still
// works)
__attribute__((section(".vectors"), used, aligned(128)))
const void *const Vectors[16] = {
    &_estack,        &Reset_Handler,   &Default_Handler, &Default_Handler,
    &Default_Handler, &Default_Handler, &Default_Handler, 0,
    0,               0,               0,               &Default_Handler,
    &Default_Handler, 0,               &Default_Handler, &Default_Handler,
};

__attribute__((section(".startup"), naked, noreturn))
void Reset_Handler(void) {
  __asm__ volatile("ldr sp, =_estack\n"
                   "b reset_c\n");
}

__attribute__((section(".startup"), used, noreturn))
void reset_c(void) {
  __asm__ volatile("cpsid i");
  *(volatile uint32_t *)0xE000ED08 = (uint32_t)Vectors; // VTOR

  // Flash erase/program code runs from ITCM, see flexspi.cpp
  for (uint32_t *src = &_sramfuncload, *dst = &_sramfunc; dst < &_eramfunc;)
    *dst++ = *src++;
  for (uint32_t *src = &_sdataload, *dst = &_sdata; dst < &_edata;)
    *dst++ = *src++;
  for (uint32_t *dst = &_sbss; dst < &_ebss;)
    *dst++ = 0;
  __asm__ volatile("dsb\nisb" ::: "memory");

  main();
  for (;;)
    ;
}

// Monocypher and the boot code have virtual functions but no C++ runtime
void __cxa_pure_virtual(void) { Default_Handler(); }
