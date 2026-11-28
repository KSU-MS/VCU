/* The bootloader: the first 64 KB of flash (boot::BOOTLOADER_SIZE), runs in
 * place from flash, a little ITCM for the flash routines and DTCM for data
 * and stack. Only the low 128 KB of each TCM is used, which exists whatever
 * the FlexRAM fuses say. */
MEMORY
{
	ITCM (rwx): ORIGIN = 0x00000400, LENGTH = 31K
	DTCM (rw):  ORIGIN = 0x20000000, LENGTH = 64K
	FLASH (rx): ORIGIN = 0x60000000, LENGTH = 64K
}

ENTRY(ImageVectorTable)

SECTIONS
{
	.text.headers : {
		KEEP(*(.flashconfig))
		FILL(0xFF)
		. = ORIGIN(FLASH) + 0x1000;
		KEEP(*(.ivt))
		KEEP(*(.bootdata))
		. = ALIGN(128);
		KEEP(*(.vectors))
	} > FLASH

	.text : {
		*(.startup)
		*(.text*)
		*(.rodata*)
		. = ALIGN(4);
	} > FLASH

	.ARM.exidx : {
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > FLASH

	.ramfunc : {
		. = ALIGN(4);
		*(.ramfunc*)
		. = ALIGN(4);
	} > ITCM AT> FLASH

	.data : {
		. = ALIGN(4);
		*(.data*)
		. = ALIGN(4);
	} > DTCM AT> FLASH

	.bss (NOLOAD) : {
		. = ALIGN(4);
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
	} > DTCM

	_sramfunc = ADDR(.ramfunc);
	_eramfunc = ADDR(.ramfunc) + SIZEOF(.ramfunc);
	_sramfuncload = LOADADDR(.ramfunc);
	_sdata = ADDR(.data);
	_edata = ADDR(.data) + SIZEOF(.data);
	_sdataload = LOADADDR(.data);
	_sbss = ADDR(.bss);
	_ebss = ADDR(.bss) + SIZEOF(.bss);
	_estack = ORIGIN(DTCM) + LENGTH(DTCM);
	_flashimagelen = LOADADDR(.data) + SIZEOF(.data) - ORIGIN(FLASH);
}
