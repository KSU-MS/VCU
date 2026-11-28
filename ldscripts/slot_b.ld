/* Firmware for slot B: boot::slot_firmware_address(SLOT_B), up to
 * boot::SLOT_FIRMWARE_MAX (0x1DF000). Everything else as the Teensy core. */
MEMORY
{
	ITCM (rx):  ORIGIN = 0x00000000, LENGTH = 512K
	DTCM (rw):  ORIGIN = 0x20000000, LENGTH = 512K
	RAM (rw):   ORIGIN = 0x20200000, LENGTH = 512K
	FLASH (rx): ORIGIN = 0x60201000, LENGTH = 0x1DF000
	ERAM (rw):  ORIGIN = 0x70000000, LENGTH = 32768K
}

INCLUDE vcu_sections.ld
