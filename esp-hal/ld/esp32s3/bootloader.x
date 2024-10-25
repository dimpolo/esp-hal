ENTRY(BootloaderReset)

INCLUDE "bootloader_memory.x"
/*INCLUDE "esp32s3.x" TODO include fixup*/
INCLUDE "rom-functions.x"

SECTIONS {
  .section_mc_section_face_text : ALIGN(4)
  {
    *(.literal .text .literal.* .text.*)
    . = ALIGN (4);
    *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)

    /** CPU will try to prefetch up to 16 bytes of
      * of instructions. This means that any configuration (e.g. MMU, PMS) must allow
      * safe access to up to 16 bytes after the last real instruction, add
      * dummy bytes to ensure this
      */
    . += 16;
  } > iram_seg
}

SECTIONS {
  .section_mc_section_face_data : ALIGN(4)
  {
    *(.data .data.*)
    *(.rodata .rodata.*)
  } > dram_seg
}

SECTIONS {
  .section_mc_section_face_bss : ALIGN(4)
  {
    _bss_start = ABSOLUTE(.);
    *(.bss .bss.*)
    . = ALIGN(4);
    _bss_end = ABSOLUTE(.);
  } > dram_seg
}

PROVIDE(_stack_start_cpu0 = ORIGIN(dram_seg) + LENGTH(dram_seg));

