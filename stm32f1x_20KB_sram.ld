ENTRY(Reset_Handler)
MEMORY
{
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x400;
SECTIONS
{
  .isr_vector ORIGIN(RAM) :
  {
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >RAM
  .text :
  {
    . = ALIGN(4);
    *(.text .text*)
    *(.rodata .rodata*)
    *(.glue_7 .glue_7t)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >RAM
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >RAM
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >RAM
  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >RAM
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >RAM
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >RAM
  /* Keep a separate SRAM copy of initial values for subsequent warm resets.
     The loader writes the initial values at the LMA (_sidata); startup copies
     them into the writable VMA (_sdata), immediately after that template. */
  _sidata = ALIGN(4);
  .data (_sidata + SIZEOF(.data)) : AT(_sidata)
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = .;
    *(.bss .bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = .;
  } >RAM
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . += _Min_Heap_Size + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
ASSERT(ADDR(.isr_vector) == 0x20000000, "SRAM vectors must start at 0x20000000")
ASSERT(SIZEOF(.isr_vector) == 0x10C, "Medium-density BootRAM must be at offset 0x108")
ASSERT(_sidata + SIZEOF(.data) == _sdata, "SRAM data template must not overlap writable data")
ASSERT(_edata <= _sbss, "SRAM data must not overlap BSS")
ASSERT(ADDR(._user_heap_stack) + SIZEOF(._user_heap_stack) <= _estack, "SRAM image leaves insufficient stack space")
