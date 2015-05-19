/*
*****************************************************************************
**
** Linker script for STM32F072RB(128KByte FLASH, 16KByte RAM)
** 	- tout en RAM @ 0x2000 0000, pas de remap. 
**	- no heap
**
From GNU ld docs : 
"
Every loadable or allocatable output section has two addresses. The ?rst is the VMA, or
virtual memory address. This is the address the section will have when the output ?le is
run. The second is the LMA, or load memory address. This is the address at which the
section will be loaded.
"
ADDR(section) returns the VMA of <section>.
LOADADDR(section) returns the LMA of <section>

.section_name [vma] : {....} [>VMA_REGION] [AT>LMA_REGION]
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Specify the memory areas */
MEMORY {
	FLASH (rx)	: ORIGIN = 0x08000000, LENGTH = 128K
	RAM (xrw)	: ORIGIN = 0x20000000, LENGTH = 16K
	MAP0 (xrw)	: ORIGIN = 0x0, LENGTH = 16k
}
REGION_ALIAS("TGT", RAM);

/* Highest address of the user mode stack */
_stackinit =  ORIGIN(RAM)+LENGTH(RAM)-4;    /* end of RAM -4 !*/
/* TODO : stacksize + limit check */

/* Define output sections */
SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >RAM AT>TGT

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >RAM AT>TGT

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >RAM AT>TGT


  /* used by the startup to initialize data */
  /* i.e. memcpy(_sdata, _sidata, _idatalen) */
  _sidata = LOADADDR(.data);
  _idatalen = SIZEOF(.data);
  _eidata = _sidata + _idatalen;

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT>RAM

  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    _sbss = .;         /* define a global symbol at bss start */
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
  } >RAM AT>TGT
  _bsslen = SIZEOF(.bss);

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

