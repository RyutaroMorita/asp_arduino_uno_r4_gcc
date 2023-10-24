/*
                  Linker File for RA4M1 MCU
*/

/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx)         : ORIGIN = 0x00000000, LENGTH = 0x0040000  /* 256K */
  RAM (rwx)          : ORIGIN = 0x20000000, LENGTH = 0x0008000  /* 32K */
  DATA_FLASH (rx)    : ORIGIN = 0x40100000, LENGTH = 0x0002000  /*  8K */

  ID_CODE (rx)       : ORIGIN = 0x01010018, LENGTH = 0x20      /* 32 bytes */
}

PROVIDE(hardware_init_hook = 0);
PROVIDE(software_init_hook = 0);
PROVIDE(software_term_hook = 0);

ENTRY(_kernel__start)

SECTIONS
{
	.text :
	{
		__text = .;
		KEEP(*(.vector))
        /* ROM Registers start at address 0x00000400 */
        . = __text + 0x400;
        KEEP(*(.rom_registers*))
        /* Reserving 0x100 bytes of space for ROM registers. */
        . = __text + 0x500;
		*(.text)
		*(.text.*)
	} > FLASH
	_etext = .;
	PROVIDE(etext = .);

	.rodata :
	{
		*(.rodata)
		*(.rodata.*)
	} > FLASH

	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) }
	__exidx_start = .;
	.ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) }
	__exidx_end = .;

	__idata_start = .;
	.data : AT (__idata_start)
	{
		__data_start = .;
		*(.data)
		*(.data.*)
	} > RAM
	__idata_end = __idata_start + SIZEOF(.data);
	_edata = .;
	PROVIDE(edata = .);

	.heap (NOLOAD):
	{
		. = ALIGN(8);
		__HeapBase = .;
		__end__ = .;
		end = __end__;
		/* Place the STD heap here. */
		KEEP(*(.heap))
		__HeapLimit = .;
	} > RAM

	. = ALIGN(4);

	.bss :
	{
		__bss_start  = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
	} > RAM
	. = ALIGN(4);
	_end = .;
	__bss_end = .;
	PROVIDE(end = .);

	.id_code :
	{
		__ID_Code_Start = .;
		KEEP(*(.id_code*))
		__ID_Code_End = .;
	} > ID_CODE
}
