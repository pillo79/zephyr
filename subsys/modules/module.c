/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "zephyr/sys/slist.h"
#include "zephyr/sys/util.h"
#include <zephyr/modules/elf.h>
#include <zephyr/modules/module.h>
#include <zephyr/modules/buf_stream.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modules, CONFIG_MODULES_LOG_LEVEL);

#include <string.h>

/* TODO different allocator pools for metadata, code sections, and data sections */
K_HEAP_DEFINE(module_heap, CONFIG_MODULES_HEAP_SIZE * 1024);

static const char ELF_MAGIC[] = {0x7f, 'E', 'L', 'F'};

int module_buf_read(struct module_stream *s, void *buf, size_t len)
{
	struct module_buf_stream *buf_s = CONTAINER_OF(s, struct module_buf_stream, stream);
	size_t end = MIN(buf_s->pos + len, buf_s->len);
	size_t read_len = end - buf_s->pos;

	LOG_DBG("reading from pos %ld (%p) len %d, new pos %d (%p), read_len %d",
		buf_s->pos, (void *)&buf_s->buf[buf_s->pos],
		len, end, (void *)&buf_s->buf[end],
		read_len);

	memcpy(buf, buf_s->buf + buf_s->pos, read_len);
	LOG_HEXDUMP_DBG(buf_s->buf + buf_s->pos, read_len, "Read from buffer");
	buf_s->pos = end;

	return read_len;
}

int module_buf_seek(struct module_stream *s, size_t pos)
{
	struct module_buf_stream *buf_s = CONTAINER_OF(s, struct module_buf_stream, stream);

	buf_s->pos = MIN(pos, buf_s->len);

	return 0;
}


static inline int mod_read(struct module_stream *s, void *buf, size_t len)
{
	return s->read(s, buf, len);
}

static inline int mod_seek(struct module_stream *s, size_t pos)
{
	return s->seek(s, pos);
}

static sys_slist_t _module_list = SYS_SLIST_STATIC_INIT(&_module_list);

sys_slist_t *module_list(void)
{
	return &_module_list;
}

struct module *module_from_name(const char *name) {
	sys_slist_t *mlist = module_list();
	sys_snode_t *node = sys_slist_peek_head(mlist);
	struct module *m = CONTAINER_OF(node, struct module, _mod_list);

	while (node != NULL) {
		if (strncmp(m->name, name, sizeof(m->name)) == 0){
			return m;
		}
		node = sys_slist_peek_next(node);
		m = CONTAINER_OF(node, struct module, _mod_list);
	}

	return NULL;
}


/* find arbitrary symbol's address according to its name in a module */
void *module_find_sym(struct module_symtable *sym_table, const char *sym_name)
{
	elf_word i;

	/* find symbols in module */
	for (i = 0; i < sym_table->sym_cnt; i++) {
		if (strcmp(sym_table->syms[i].name, sym_name) == 0) {
			return (void *)sym_table->syms[i].addr;
		}
	}

	return NULL;
}

/**
 * @brief load a relocatable object file.
 *
 * An unlinked or partially linked elf will have symbols that have yet to be
 * determined and must be linked in effect. This is similiar, but not exactly like,
 * a dynamic elf. Typically the code and addresses *are* position dependent.
 */
static int module_load_rel(struct module_stream *ms, struct module *m)
{
	elf_ehdr_t ehdr;
	elf_word i, j, sym_cnt, rel_cnt;
	bool flag;
	elf_addr ptr, sym_addr;
	elf_shdr_t *shdr_array;
	elf_sym_t *sym;
	elf_rel_t rel;
	char str_name[50];

	m->mem_sz = 0;

	mod_seek(ms, 0);
	mod_read(ms, (void *)&ehdr, sizeof(ehdr));

	size_t shdr_array_sz = ehdr.e_shnum * sizeof(elf_shdr_t);
	LOG_DBG("allocating section header array for %d sections, size %d", ehdr.e_shnum, shdr_array_sz);

	shdr_array = (elf_shdr_t *)k_heap_alloc(&module_heap,shdr_array_sz, K_NO_WAIT);

	/* iterate all sections and get total necessary memory size */
	flag = false;
	for (i = 0; i < ehdr.e_shnum; i++) {
		size_t pos = ehdr.e_shoff + i*ehdr.e_shentsize;
		mod_seek(ms, pos);
		mod_read(ms, (void *)&shdr_array[i], sizeof(elf_shdr_t));

		LOG_DBG("section %d at %x: name %d, type %d, flags %x, addr %x, size %d",
			i,
			ehdr.e_shoff+i*ehdr.e_shentsize,
			shdr_array[i].sh_name,
			shdr_array[i].sh_type,
			shdr_array[i].sh_flags,
			shdr_array[i].sh_addr,
			shdr_array[i].sh_size);

		/* get symtab and strtab sections */
		if (shdr_array[i].sh_type == SHT_SYMTAB) {
			LOG_DBG("symbol table at section %d", i);
			ms->symtab = shdr_array[i];
		} else if (shdr_array[i].sh_type == SHT_STRTAB) {
			LOG_DBG("string table at section %d", i);
			ms->strtab = shdr_array[i];
		}
		if ((shdr_array[i].sh_flags & SHF_ALLOC)
				&& (shdr_array[i].sh_size > 0)) {
			if (!flag) {
				LOG_DBG("updating virt start addr, virt addr sz %d, section addr sz %d",
					sizeof(m->virt_start_addr), sizeof(shdr_array[i].sh_addr));
				/*
				m->virt_start_addr = shdr_array[i].sh_addr;
				*/
				LOG_DBG("updating module size, adding %d to %d", shdr_array[i].sh_size, m->mem_sz);
				m->mem_sz += shdr_array[i].sh_size;
				flag = true;
			} else {
				LOG_DBG("updating module size, adding %d to %d", shdr_array[i].sh_size, m->mem_sz);
				m->mem_sz += shdr_array[i].sh_size;
			}
		}
	}


	LOG_DBG("module size (total section sizes) %d", m->mem_sz);

	/* allocate memory */
	m->load_start_addr = (elf_addr)k_heap_alloc(&module_heap,
			(size_t)m->mem_sz, K_NO_WAIT);
	if ((void *)m->load_start_addr == NULL) {
		/* no available memory to load module */
		LOG_ERR("Not enough memory for module");
		return -ENOMEM;
	}
	memset((void *)m->load_start_addr, 0, sizeof(m->mem_sz));

	/* copy sections into memory */
	ptr = m->load_start_addr;
	for (i = 0; i < ehdr.e_shnum - 1; i++) {
		if ((shdr_array[i].sh_flags & SHF_ALLOC)
				&& (shdr_array[i].sh_size > 0)) {
			memcpy((void *)ptr, (void *)&shdr_array[i],
					(size_t)shdr_array[i].sh_size);
			shdr_array[i].sh_addr = ptr;
			ptr += shdr_array[i].sh_size;
		}
	}

	/* Section names */
	for (i = 0; i < ehdr.e_shnum; i++) {
		elf32_word str_idx = shdr_array[i].sh_name;
		char sec_name[32];
		mod_seek(ms, ms->strtab.sh_offset + str_idx);
		mod_read(ms, sec_name, sizeof(sec_name));
		if (strncmp(sec_name, ".text", sizeof(sec_name)) == 0) {
			ms->text = shdr_array[i];
		} else if (strncmp(sec_name, ".data", sizeof(sec_name)) == 0) {
			ms->data = shdr_array[i];
		} else if(strncmp(sec_name, ".rodata", sizeof(sec_name)) == 0) {
			ms->rodata = shdr_array[i];
		} else if (strncmp(sec_name, ".bss", sizeof(sec_name)) == 0) {
			ms->bss = shdr_array[i];
		}
		LOG_DBG("section %d at addr %x, name %s", i, shdr_array[i].sh_addr, sec_name);
	}


	/* Iterate all symbols in symtab and update its st_value,
	 * for sections, using its loading address,
	 * for undef functions or variables, find it's address globally.
	 */
	sym_cnt = ms->symtab.sh_size / sizeof(elf_sym_t);
	sym = (elf_sym_t *)ms->symtab.sh_addr;
	for (i = 0; i < sym_cnt; i++) {
		mod_seek(ms, ms->strtab.sh_offset + sym[i].st_name);
		mod_read(ms, str_name, sizeof(str_name));
		switch (sym[i].st_shndx) {
		case SHN_UNDEF:
			LOG_DBG("Map symbol: %s\n", str_name);
			/*TODO
			sym_addr = (elf_addr)module_find_sym(sym_table, str_name);
			sym[i].st_value = (elf_addr)sym_addr;
			*/
			break;
		case SHN_ABS:
			break;
		case SHN_COMMON:
			break;
		default:
			sym[i].st_value += shdr_array[sym[i].st_shndx].sh_addr;
			break;
		}
	}

	/* symbols relocation */
	for (i = 0; i < ehdr.e_shnum - 1; i++) {
		/* find out relocation sections */
		if ((shdr_array[i].sh_type == SHT_REL)
				|| (shdr_array[i].sh_type == SHT_RELA)) {
			rel_cnt = shdr_array[i].sh_size / sizeof(elf_rel_t);

			for (j = 0; j < rel_cnt; j++) {
				/* get each relocation entry */
				mod_seek(ms, shdr_array[i].sh_offset
						+ j * sizeof(elf_rel_t));
				mod_read(ms, (void *)&rel, sizeof(elf_rel_t));

				/* get corresponding symbol */
				mod_seek(ms, ms->symtab.sh_offset
						+ ELF_R_SYM(rel.r_info)
						* sizeof(elf_sym_t));
				mod_read(ms, (void *)&sym, sizeof(sym));

				/* relocation */
				arch_elf_relocate_rel(m, &rel,
					&shdr_array[shdr_array[i].sh_info],
					sym);
			}
		}
	}

	return 0;
}

/* load a shared object file */
static int module_load_dyn(struct module_stream *ms, struct module *m)
{
	elf_ehdr_t ehdr;
	elf_phdr_t phdr;
	elf_shdr_t shdr, dynsym_shdr, dynstr_shdr;
	elf_word i, j, rel_cnt, dynsym_cnt, count, len;
	elf_addr end_addr, sym_addr;
	elf_rel_t rel;
	elf_sym_t sym;
	bool flag;
	char str_name[50];

	mod_seek(ms, 0);
	mod_read(ms, (void *)&ehdr, sizeof(ehdr));

	/* iterate all program segments to get total necessary memory size */
	flag = false;
	end_addr = 0;
	for (i = 0; i < ehdr.e_phnum; i++) {
		mod_seek(ms, ehdr.e_phoff + i * ehdr.e_phentsize);
		mod_read(ms, (void *)&phdr, sizeof(phdr));
		if (phdr.p_type == PT_LOAD) {
			if (!flag) {
				m->virt_start_addr = phdr.p_vaddr;
				end_addr = phdr.p_vaddr + phdr.p_memsz;
				flag = true;
			} else {
				end_addr = phdr.p_vaddr + phdr.p_memsz;
			}
		}
	}
	LOG_DBG("updating module size, subtracting end_addr %ul from virt start addr %ul", end_addr, m->virt_start_addr);
	m->mem_sz = end_addr - m->virt_start_addr;
	LOG_DBG("module size %d", m->mem_sz);

	/* allocate memory */
	m->load_start_addr = (elf_addr)k_heap_alloc(&module_heap,
			(size_t)m->mem_sz, K_NO_WAIT);
	if ((void *)m->load_start_addr == NULL) {
		/* no available memory to load module */
		LOG_ERR("Not enough memory for module");
		return -ENOMEM;
	}
	memset((void *)m->load_start_addr, 0, sizeof(m->mem_sz));

	/* copy segments into memory */
	for (i = 0; i < ehdr.e_phnum; i++) {
		mod_seek(ms, ehdr.e_phoff + i * ehdr.e_phentsize);
		mod_read(ms, (void *)&phdr, sizeof(phdr));
		if (phdr.p_type == PT_LOAD) {
			mod_seek(ms, phdr.p_offset);
			mod_read(ms, (void *)(m->load_start_addr
						+ phdr.p_vaddr
						- m->virt_start_addr),
					phdr.p_filesz);
		}
	}

	/* doing symbols relocation */
	for (i = 0; i < ehdr.e_shnum; i++) {
		mod_seek(ms, ehdr.e_shoff + i * ehdr.e_shentsize);
		mod_read(ms, (void *)&shdr, sizeof(shdr));
		/* get .dynsym and .dynstr section, they will be used
		 * for symbols relocation and finding exported symbols
		 * of module.
		 */
		if (shdr.sh_type == SHT_DYNSYM) {
			/* get .dynsym section */
			dynsym_shdr = shdr;
			/* get .dynstr section */
			mod_seek(ms, ehdr.e_shoff + dynsym_shdr.sh_link
					* ehdr.e_shentsize);
			mod_read(ms, (void *)&dynstr_shdr,
					sizeof(dynstr_shdr));
		}
		/* find out relocation sections */
		else if ((shdr.sh_type == SHT_REL)
				|| (shdr.sh_type == SHT_RELA)) {
			rel_cnt = shdr.sh_size / sizeof(elf_rel_t);
			for (j = 0; j < rel_cnt; j++) {
				/* get each relocation entry */
				mod_seek(ms, shdr.sh_offset
						+ j * sizeof(elf_rel_t));
				mod_read(ms, (void *)&rel, sizeof(elf_rel_t));

				/* get corresponding symbol */
				mod_seek(ms, dynsym_shdr.sh_offset
						+ ELF_R_SYM(rel.r_info)
						* sizeof(elf_sym_t));
				mod_read(ms, (void *)&sym, sizeof(sym));

				/* get corresponding symbol str name */
				mod_seek(ms, dynstr_shdr.sh_offset
						+ sym.st_name);
				mod_read(ms, str_name, sizeof(str_name));

				/* find out symbol's real address */
				if (sym.st_shndx == SHN_UNDEF) {
					/* this symbol needs to be found
					 * globally.
					 */
					LOG_DBG("Looking up symbol %s\n", str_name);
					/* TODO map symbols with a provided set of tables */
					/*
					sym_addr = (elf_addr)module_find_sym(
							KERNEL_MODULE,
							str_name);
					*/
				} else {
					/* this symbol could be found locally */
					sym_addr = m->load_start_addr
						+ sym.st_value
						- m->virt_start_addr;
				}

				/* doing relocation */
				/* TODO  */
				/*
				arch_elf_relocate_dyn(module, &rel,
						sym_addr);
				*/
			}
		}
	}

	/* get all exported symbols of module.
	 * iterate all symbols in .dynsym section and find out those symbols
	 * with global, function attributes.
	 */
	/* get total count of exported symbols */
	count = 0;
	dynsym_cnt = dynsym_shdr.sh_size / sizeof(elf_sym_t);
	for (j = 0; j < dynsym_cnt; j++) {
		mod_seek(ms, dynsym_shdr.sh_offset + j * sizeof(elf_sym_t));
		mod_read(ms, (void *)&sym, sizeof(sym));
		if ((ELF_ST_BIND(sym.st_info) == STB_GLOBAL)
				&& (ELF_ST_TYPE(sym.st_info) == STT_FUNC)
				&& (sym.st_shndx != SHN_UNDEF)) {
			count++;
		}
	}
	m->sym_tab.syms = k_heap_alloc(&module_heap, count * sizeof(struct module_symbol), K_NO_WAIT);
	if (m->sym_tab.syms == NULL) {
		LOG_ERR("Not enough memory for module symbol table");
		/* TODO CLEANUP! */
		return -ENOMEM;
	}

	m->sym_tab.sym_cnt = count;

	/* get each exported symbols's address and string name */
	for (j = 0, count = 0; j < dynsym_cnt; j++) {
		mod_seek(ms, dynsym_shdr.sh_offset
				+ j * sizeof(elf_sym_t));
		mod_read(ms, (void *)&sym, sizeof(sym));
		if ((ELF_ST_BIND(sym.st_info) == STB_GLOBAL)
				&& (ELF_ST_TYPE(sym.st_info) == STT_FUNC)
				&& (sym.st_shndx != SHN_UNDEF)) {

			/* get symbol's address */
			m->sym_tab.syms[count].addr = sym.st_value
				- m->virt_start_addr
				+ m->load_start_addr;

			/* get symbol's string name */
			mod_seek(ms, dynstr_shdr.sh_offset
					+ sym.st_name);
			mod_read(ms, str_name, sizeof(str_name));
			len = strlen(str_name) + 1;
			m->sym_tab.syms[count].name = k_heap_alloc(&module_heap, len, K_NO_WAIT);
			if (m->sym_tab.syms[count].name == NULL) {
				LOG_ERR("Not enough memory for symbol name");
				/* TODO cleanup */
				return -ENOMEM;
			}
			memcpy((void *)m->sym_tab.syms[count].name,
					(void *)str_name, len);
			count++;
		}
	}

	return 0;
}

int module_load(struct module_stream *ms, const char name[16], struct module **m)
{
	int ret;
	elf_ehdr_t ehdr;

	mod_seek(ms, 0);
	mod_read(ms, (void *)&ehdr, sizeof(ehdr));

	/* check whether this is an valid elf file */
	if (memcmp(ehdr.e_ident, ELF_MAGIC, sizeof(ELF_MAGIC)) != 0) {
		LOG_HEXDUMP_ERR(ehdr.e_ident, 16, "Invalid ELF, magic does not match");
		return -EINVAL;
	}

	if (ehdr.e_type == ET_REL) {
		LOG_DBG("Loading relocatable elf");
		*m = k_heap_alloc(&module_heap, sizeof(struct module), K_NO_WAIT);
		if (m == NULL) {
			LOG_ERR("Not enough memory for module metadata");
			ret = -ENOMEM;
		}
		ret = module_load_rel(ms, *m);
	} else if (ehdr.e_type == ET_DYN) {
		LOG_DBG("Loading dynamic elf");
		*m = k_heap_alloc(&module_heap, sizeof(struct module), K_NO_WAIT);
		if (m == NULL) {
			LOG_ERR("Not enough memory for module metadata");
			ret = -ENOMEM;
		}
		ret = module_load_dyn(ms, *m);
	} else {
		LOG_ERR("Unsupported elf file type %x", ehdr.e_type);
		/* unsupported ELF file type */
		*m = NULL;
	}

	if (ret != 0) {
		if(*m != NULL) {
			module_unload(*m);
		}
		*m = NULL;
	} else {
		strncpy((*m)->name, name, sizeof((*m)->name));
		sys_slist_append(&_module_list, &(*m)->_mod_list);
	}

	return ret;
}

void module_unload(struct module *m)
{
	__ASSERT(m, "Expected non-null module");

	sys_slist_find_and_remove(&_module_list, &m->_mod_list);

	k_heap_free(&module_heap, (void *)m->sym_tab.syms);
	k_heap_free(&module_heap, (void *)m->load_start_addr);
	k_heap_free(&module_heap, (void *)m);
}
