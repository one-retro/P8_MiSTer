ENTRY(_start)
SECTIONS
{
  . = 0;
  .text : { *(.text.start) *(.text .text.*) }
  .rodata : ALIGN(4) { *(.rodata .rodata.* .srodata .srodata.*) }
  .data : ALIGN(4) { *(.data .data.* .sdata .sdata.*) }
  .bss : ALIGN(4) { *(.bss .bss.* .sbss .sbss.* COMMON) }
  _end = .;
}
