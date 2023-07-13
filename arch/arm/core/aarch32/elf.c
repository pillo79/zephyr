#include <zephyr/kernel.h>
#include <zephyr/modules/module.h>
#include <zephyr/modules/elf.h>

#define R_ARM_ABS32 2
#define R_ARM_CALL 28

/**
 * @brief Architecture specific function for relocating partially linked (static) elf
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting modules
 * must implement this.
 *
 * The relocation codes for arm are well documented
 * https://github.com/ARM-software/abi-aa/blob/main/aaelf32/aaelf32.rst#relocation
 * */
void arch_elf_relocate(elf_rel_t *rel, uintptr_t opaddr, uintptr_t opval)
{
	elf_word reloc_type = ELF32_R_TYPE(rel->r_info);

	switch(reloc_type) {
		case R_ARM_ABS32:
			/* Update the absolute address of a load/store instruction */
			*((uint32_t*)opaddr) = (uint32_t)opval;
			break;
		case R_ARM_CALL:
			/* Update the address of the call (BL/BLX) instruction */
			*((uint32_t*)opaddr) = ((uint32_t)opval & 0x03FFFFFE);
			break;
		default:
			printk("Unsupported relocation type %d\n", reloc_type);
			break;
	}
}
