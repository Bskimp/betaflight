/*
*****************************************************************************
**
**  File        : stm32_h750_brainfpv_xip.ld
**
**  Abstract    : Linker script for STM32H750xB on the BrainFPV RADIX 2 HD,
**                running in XIP (eXecute In Place) directly from QSPI flash.
**
**                Forked from stm32_ram_h750_exst.ld with the key difference
**                that .text/.rodata live in QSPI at 0x90400000+ and are
**                fetched by the CPU in-place. The standard EXST flow (where
**                the bootloader copies the entire firmware into RAM at
**                0x24010000 first) is NOT used here. The BrainFPV bootloader
**                sets up memory-mapped QSPI and jumps directly to firmware.
**
**                .data / .bss / .fastram_* still live in RAM as usual; their
**                LMAs (initial values) live in QSPI and are copied to RAM by
**                startup code, same as a normal Cortex-M program.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x00000000 to 0x0000FFFF   64K ITCM
0x20000000 to 0x2001FFFF  128K DTCM, main RAM
0x24000000 to 0x2407FFFF  512K AXI SRAM, D1 domain   (full size — no CODE_RAM carve-out)
0x30000000 to 0x3001FFFF  128K SRAM1, D2 domain
0x30020000 to 0x3003FFFF  128K SRAM2, D2 domain
0x30040000 to 0x30047FFF   32K SRAM3, D2 domain, unused
0x38000000 to 0x3800FFFF   64K SRAM4, D3 domain, unused
0x38800000 to 0x38800FFF    4K BACKUP SRAM, Backup domain, unused

0x08000000 to 0x0801FFFF  128K bootloader region (BrainFPV-owned, no firmware here)
0x90000000 to 0x903FFFFF    4M QSPI region reserved for bootloader + bootloader settings
0x90400000 to 0x907FFFFF    4M QSPI region for firmware (us)
0x90800000 to 0x90FFFFFF    8M QSPI region for blackbox / flashfs
*/

/* Specify the memory areas */
MEMORY
{
    ITCM_RAM (rwx)    : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM_RAM (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)         : ORIGIN = 0x24000000, LENGTH = 512K   /* full AXI SRAM */

    D2_RAM (rwx)      : ORIGIN = 0x30000000, LENGTH = 256K   /* SRAM1 + SRAM2 */

    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
    QUADSPI (rx)      : ORIGIN = 0x90400000, LENGTH = 4M     /* firmware region post-bootloader */
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)
REGION_ALIAS("MAIN", QUADSPI)

/* ------------------------------------------------------------------------
 * BrainFPV BL header — must be the FIRST thing in QUADSPI so the bootloader
 * finds it at the firmware load address (0x90400000). Declared in our own
 * SECTIONS block before the common.ld include so it lands ahead of
 * .isr_vector. The struct itself is defined in
 * src/config/configs/RADIX2HD/config.c with section attribute.
 *
 * After the 8-byte struct the linker pads up to the 512-byte alignment that
 * common.ld's .isr_vector requires, so the vector table lands at a clean
 * boundary (0x90400200) and the symbol `isr_vector_table_base` (also
 * provided by common.ld) gives us its address for the BL header's
 * isr_vector_base field.
 * ------------------------------------------------------------------------ */
SECTIONS
{
    .bl_header_section : ALIGN(4)
    {
        KEEP(*(.bl_header_section))
    } >MAIN
}

INCLUDE "stm32_h750_common.ld"

/* ------------------------------------------------------------------------
 * DMA-capable RAM sections, identical to stm32_ram_h750_exst.ld. dmaram_data
 * lives at runtime in RAM (D1 AXI SRAM) but is loaded from QUADSPI; startup
 * code uses _sdmaram_idata / _sdmaram_data / _edmaram_data to copy it.
 * ------------------------------------------------------------------------ */
SECTIONS
{
  /* used during startup to initialize dmaram_data */
  _sdmaram_idata = LOADADDR(.dmaram_data);

  . = ALIGN(32);
  .dmaram_data :
  {
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    _sdmaram_data = .;        /* create a global symbol at data start */
    *(.dmaram_data)           /* .data sections */
    *(.dmaram_data*)          /* .data* sections */
    . = ALIGN(32);
    _edmaram_data = .;        /* define a global symbol at data end */
  } >RAM AT >MAIN

  . = ALIGN(32);
  .dmaram_bss (NOLOAD) :
  {
    _sdmaram_bss = .;
    __dmaram_bss_start__ = _sdmaram_bss;
    *(.dmaram_bss)
    *(SORT_BY_ALIGNMENT(.dmaram_bss*))
    . = ALIGN(32);
    _edmaram_bss = .;
    __dmaram_bss_end__ = _edmaram_bss;
  } >RAM

  . = ALIGN(32);
  .DMA_RAM (NOLOAD) :
  {
    KEEP(*(.DMA_RAM))
    PROVIDE(dmaram_end = .);
    _edmaram = .;
    _dmaram_end__ = _edmaram;
  } >RAM

  .DMA_RW_D2 (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarw_start = .);
    _sdmarw = .;
    _dmarw_start__ = _sdmarw;
    KEEP(*(.DMA_RW))
    PROVIDE(dmarw_end = .);
    _edmarw = .;
    _dmarw_end__ = _edmarw;
  } >D2_RAM

  .DMA_RW_AXI (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarwaxi_start = .);
    _sdmarwaxi = .;
    _dmarwaxi_start__ = _sdmarwaxi;
    KEEP(*(.DMA_RW_AXI))
    PROVIDE(dmarwaxi_end = .);
    _edmarwaxi = .;
    _dmarwaxi_end__ = _edmarwaxi;
  } >RAM
}

INCLUDE "stm32_h750_common_post.ld"

/* No INCLUDE of stm32_ram_h750_exst_post.ld — its .exst_hash section is for
   SPRACING's MD5-hash bootloader handoff. Our bootloader validates firmware
   via the .bl_header_section magic above, not a trailing hash. */
