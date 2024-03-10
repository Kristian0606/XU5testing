/*   _____ _____ _____
 *  |   | |__   |   | | cpu0/src/lscript.ld
 *  | | | |   __| | | | Nikolai Nymo
 *  |_|___|_____|_|___| 16-04-19
 */

_STACK_SIZE = DEFINED(_STACK_SIZE) ? _STACK_SIZE :  0x800;
_HEAP_SIZE  = DEFINED(_HEAP_SIZE)  ? _HEAP_SIZE  :    0x0;

_ABORT_STACK_SIZE      = DEFINED(_ABORT_STACK_SIZE)      ? _ABORT_STACK_SIZE      :   0x0;
_SUPERVISOR_STACK_SIZE = DEFINED(_SUPERVISOR_STACK_SIZE) ? _SUPERVISOR_STACK_SIZE :   0x0;
_IRQ_STACK_SIZE        = DEFINED(_IRQ_STACK_SIZE)        ? _IRQ_STACK_SIZE        : 0x400;
_FIQ_STACK_SIZE        = DEFINED(_FIQ_STACK_SIZE)        ? _FIQ_STACK_SIZE        :   0x0;
_UNDEF_STACK_SIZE      = DEFINED(_UNDEF_STACK_SIZE)      ? _UNDEF_STACK_SIZE      :   0x0;

/* Availiable Memories */
MEMORY
{
 /* FLASH   : ORIGIN = 0xFC000000, LENGTH = 0x01000000     16MB - Reserved for FSBL */
    OCM_LO0 : ORIGIN = 0x00000000, LENGTH = 0x00030000 /* 192kB - Reserved for CPU0 */
    OCM_HI0 : ORIGIN = 0xFFFF0000, LENGTH = 0x0000FE00 /*  64kB - Reserved for CPU0 */

}

/* Application Entry Point */
ENTRY(_vector_table)


/* Memory Sections */
SECTIONS
{ 
/*
    /DISCARD/ :
    {
        *(.ARM.exidx)
    }
*/
    .text :
    {
        KEEP (*(.vectors))
        *(.boot)
        *(.text)
        *(.text.*)
        *(.gnu.linkonce.t.*)
        *(.plt)
        *(.gnu_warning)
        *(.gcc_execpt_table)
        *(.glue_7)
        *(.glue_7t)
        *(.vfp11_veneer)
        *(.ARM.extab)
        *(.gnu.linkonce.armextab.*)
    } > OCM_LO0

    .init :
    {
        KEEP (*(.init))
    } > OCM_LO0

    .fini :
    {
        KEEP (*(.fini))
    } > OCM_LO0

    .rodata :
    {
        __rodata_start = .;
        *(.rodata)
        *(.rodata.*)
        *(.gnu.linkonce.r.*)
        __rodata_end = .;
    } > OCM_LO0

    .rodata1 :
    {
        __rodata1_start = .;
        *(.rodata1)
        *(.rodata1.*)
        __rodata1_end = .;
    } > OCM_LO0

    .sdata2 :
    {
        __sdata2_start = .;
        *(.sdata2)
        *(.sdata2.*)
        *(.gnu.linkonce.s2.*)
        __sdata2_end = .;
    } > OCM_LO0

    .sbss2 :
    {
        __sbss2_start = .;
        *(.sbss2)
        *(.sbss2.*)
        *(.gnu.linkonce.sb2.*)
        __sbss2_end = .;
    } > OCM_LO0

    .data :
    {
        __data_start = .;
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        *(.jcr)
        *(.got)
        *(.got.plt)
        __data_end = .;
    } > OCM_LO0

    .data1 :
    {
        __data1_start = .;
        *(.data1)
        *(.data1.*)
        __data1_end = .;
    } > OCM_LO0

    .got :
    {
        *(.got)
    } > OCM_LO0

    .ctors :
    {
        __CTOR_LIST__ = .;
        ___CTORS_LIST___ = .;
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE(*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*(.ctors))
        __CTOR_END__ = .;
        ___CTORS_END___ = .;
    } > OCM_LO0

    .dtors :
    {
        __DTOR_LIST__ = .;
        ___DTORS_LIST___ = .;
        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE(*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*(.dtors))
        __DTOR_END__ = .;
        ___DTORS_END___ = .;
    } > OCM_LO0

    .fixup :
    {
        __fixup_start = .;
        *(.fixup)
        __fixup_end = .;
    } > OCM_LO0

    .eh_frame :
    {
        *(.eh_frame)
    } > OCM_LO0

    .eh_framehdr :
    {
        __eh_framehdr_start = .;
        *(.eh_framehdr)
        __eh_framehdr_end = .;
    } > OCM_LO0
    

    .gcc_except_table :
    {
        *(.gcc_except_table)
    } > OCM_LO0


    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        *(.gnu.linkonce.armexidix.*.*)
        __exidx_end = .;
    } > OCM_LO0

    .preinit_array :
    {
        __preinit_array_start = .;
        KEEP (*(SORT(.preinit_array.*)))
        KEEP (*(.preinit_array))
        __preinit_array_end = .;
    } > OCM_LO0

    .init_array :
    {
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;
    } > OCM_LO0

    .fini_array :
    {
        __fini_array_start = .;
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array))
        __fini_array_end = .;
    } > OCM_LO0

    .ARM.attributes :
    {
        __ARM.attributes_start = .;
        *(.ARM.attributes)
        __ARM.attributes_end = .;
    } > OCM_LO0

    .sdata :
    {
        __sdata_start = .;
        *(.sdata)
        *(.sdata.*)
        *(.gnu.linkonce.s.*)
        __sdata_end = .;
    } > OCM_LO0

    .tdata :
    {
        __tdata_start = .;
        *(.tdata)
        *(.tdata.*)
        *(.gnu.linkonce.td.*)
        __tdata_end = .;
    } > OCM_LO0


    .tbss :
    {
        __tbss_start = .;
        *(.tbss)
        *(.tbss.*)
        *(.gnu.linkonce.tb.*)
        __tbss_end = .;
    } > OCM_LO0
    
    .mmu_tbl (ALIGN(16384)) :
    {
        __mmu_tbl_start = .;
        *(.mmu_tbl)
        __mmu_tbl_end = .;
    } > OCM_LO0

    .sbss (NOLOAD) :
    {
        __sbss_start = .;
        *(.sbss)
        *(.sbss.*)
        *(.gnu.linkonce.sb.*)
        __sbss_end = .;
    } > OCM_HI0

    .bss (NOLOAD) :
    {
        __bss_start = .;
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        __bss_end = .;
    } > OCM_HI0
    
    .heap (NOLOAD) :
    {
        . = ALIGN(16);
        _heap = .;
        HeapBase = .;
        _heap_start = .;
        . += _HEAP_SIZE;
        _heap_end = .;
        HeapLimit = .;
    } > OCM_HI0

    .stack (NOLOAD) :
    {
        . = ALIGN(16);
        _stack_end = .;
        . += _STACK_SIZE;
        . = ALIGN(16);
        _stack = .;
        __stack = _stack;
        . = ALIGN(16);
        _irq_stack_end = .;
        . += _IRQ_STACK_SIZE;
        . = ALIGN(16);
        __irq_stack = .;
        _supervisor_stack_end = .;
        . += _SUPERVISOR_STACK_SIZE;
        . = ALIGN(16);
        __supervisor_stack = .;
        _abort_stack_end = .;
        . += _ABORT_STACK_SIZE;
        . = ALIGN(16);
        __abort_stack = .;
        _fiq_stack_end = .;
        . += _FIQ_STACK_SIZE;
        . = ALIGN(16);
        __fiq_stack = .;
        _undef_stack_end = .;
        . += _UNDEF_STACK_SIZE;
        . = ALIGN(16);
        __undef_stack = .;
    } > OCM_HI0

    _end = .;
}

