/*
 * os.c
 *
 * ADSP OS management
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Copyright (C) 2014 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/tegra_nvadsp.h>
#include <linux/elf.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/irqchip/tegra-agic.h>
#include <linux/interrupt.h>

#include "os.h"
#include "dev.h"

#define APE_FPGA_MISC_RST_DEVICES 0x702dc800 /*1882048512*/
#define APE_RESET (1 << 6)

#define ADSP_SMMU_LOAD_ADDR	0x80300000
#define ADSP_SMMU_SIZE		SZ_8M

#define AMC_EVP_RESET_VEC_0		0x700
#define AMC_EVP_UNDEF_VEC_0		0x704
#define AMC_EVP_SWI_VEC_0		0x708
#define AMC_EVP_PREFETCH_ABORT_VEC_0	0x70c
#define AMC_EVP_DATA_ABORT_VEC_0	0x710
#define AMC_EVP_RSVD_VEC_0		0x714
#define AMC_EVP_IRQ_VEC_0		0x718
#define AMC_EVP_FIQ_VEC_0		0x71c
#define AMC_EVP_RESET_ADDR_0		0x720
#define AMC_EVP_UNDEF_ADDR_0		0x724
#define AMC_EVP_SWI_ADDR_0		0x728
#define AMC_EVP_PREFETCH_ABORT_ADDR_0	0x72c
#define AMC_EVP_DATA_ABORT_ADDR_0	0x730
#define AMC_EVP_RSVD_ADDR_0		0x734
#define AMC_EVP_IRQ_ADDR_0		0x738
#define AMC_EVP_FIQ_ADDR_0		0x73c

#define AMC_EVP_SIZE (AMC_EVP_FIQ_ADDR_0 - AMC_EVP_RESET_VEC_0 + 4)

#define ADSP_CONFIG	0x04
#define MAXCLKLATENCY   (3 << 8)

#define NVADSP_ELF "adsp.elf"
#define NVADSP_FIRMWARE NVADSP_ELF

#define MAILBOX_REGION ".mbox_shared_data"

/* Maximum number of LOAD MAPPINGS supported */
#define NM_LOAD_MAPPINGS 20

#define UART_BAUD_RATE	9600

struct nvadsp_os_data {
#if !CONFIG_SYSTEM_FPGA
	void __iomem *reset_reg;
#endif
	struct platform_device *pdev;
	struct global_sym_info *adsp_glo_sym_tbl;
	void __iomem *misc_base;
	struct resource **dram_region;
};

static struct nvadsp_os_data priv;

struct nvadsp_mappings {
	phys_addr_t da;
	void *va;
	int len;
};

static struct nvadsp_mappings adsp_map[NM_LOAD_MAPPINGS];
static int map_idx;

bool is_adsp_dram_addr(u64 addr)
{
	int i;
	struct resource **dram = priv.dram_region;

	for (i = 0; i < ADSP_MAX_DRAM_MAP; i++) {
		if ((addr >= dram[i]->start) &&
				(addr <= dram[i]->end)) {
			return true;
		}
	}
	return false;
}

int adsp_add_load_mappings(phys_addr_t pa, void *mapping, int len)
{
	if (map_idx >= NM_LOAD_MAPPINGS)
		return -EINVAL;

	adsp_map[map_idx].da = pa;
	adsp_map[map_idx].va = mapping;
	adsp_map[map_idx].len = len;
	map_idx++;
	return 0;
}

void *nvadsp_da_to_va_mappings(u64 da, int len)
{
	void *ptr = NULL;
	int i;

	for (i = 0; i < map_idx; i++) {
		int offset = da - adsp_map[i].da;

		/* try next carveout if da is too small */
		if (offset < 0)
			continue;

		/* try next carveout if da is too large */
		if (offset + len > adsp_map[i].len)
			continue;

		ptr = adsp_map[i].va + offset;
		break;
	}
	return ptr;
}
EXPORT_SYMBOL(nvadsp_da_to_va_mappings);

void *nvadsp_alloc_coherent(size_t size, dma_addr_t *da, gfp_t flags)
{
	struct device *dev;
	void *va = NULL;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		goto end;
	}

	dev = &priv.pdev->dev;
	va = dma_alloc_coherent(dev, size, da, flags);
	if (!va) {
		dev_err(dev,
			"unable to allocate the memory for size %u\n",
							(u32)size);
		goto end;
	}
	WARN(!is_adsp_dram_addr(*da),
			"bus addr %llx beyond %x\n", *da, UINT_MAX);
end:
	return va;
}
EXPORT_SYMBOL(nvadsp_alloc_coherent);

void nvadsp_free_coherent(size_t size, void *va, dma_addr_t da)
{
	struct device *dev;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}
	dev = &priv.pdev->dev;
	dma_free_coherent(dev, size, va, da);
}
EXPORT_SYMBOL(nvadsp_free_coherent);

struct elf32_shdr *
nvadsp_get_section(const struct firmware *fw, char *sec_name)
{
	int i;
	struct device *dev = &priv.pdev->dev;
	const u8 *elf_data = fw->data;
	struct elf32_hdr *ehdr = (struct elf32_hdr *)elf_data;
	struct elf32_shdr *shdr;
	const char *name_table;

	/* look for the resource table and handle it */
	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);
	name_table = elf_data + shdr[ehdr->e_shstrndx].sh_offset;

	for (i = 0; i < ehdr->e_shnum; i++, shdr++)
		if (!strcmp(name_table + shdr->sh_name, sec_name)) {
			dev_dbg(dev, "found the section %s\n",
					name_table + shdr->sh_name);
			return shdr;
		}
	return NULL;
}

static inline void dump_global_symbol_table(void)
{
	struct device *dev = &priv.pdev->dev;
	struct global_sym_info *table = priv.adsp_glo_sym_tbl;
	int num_ent;
	int i;

	if (!table) {
		dev_err(dev, "no table not created\n");
		return;
	}
	num_ent = table[0].addr;
	dev_info(dev,
	    "total number of entries in global symbol table %d\n", num_ent);

	pr_info("NAME ADDRESS TYPE\n");
	for (i = 1; i < num_ent; i++)
		pr_info("%s %x %s\n", table[i].name, table[i].addr,
				ELF32_ST_TYPE(table[i].info) == STT_FUNC ?
						    "STT_FUNC" : "STT_OBJECT");
}

static int
create_global_symbol_table(const struct firmware *fw)
{
	int i;
	struct device *dev = &priv.pdev->dev;
	struct elf32_shdr *sym_shdr =
				nvadsp_get_section(fw, ".symtab");
	struct elf32_shdr *str_shdr =
				nvadsp_get_section(fw, ".strtab");
	const u8 *elf_data = fw->data;
	const char *name_table;
	/* The first entry stores the number of entries in the array */
	int num_ent = 1;
	struct elf32_sym *sym;
	struct elf32_sym *last_sym;

	sym = (struct elf32_sym *)(elf_data + sym_shdr->sh_offset);
	name_table = elf_data + str_shdr->sh_offset;

	num_ent += sym_shdr->sh_size / sizeof(struct elf32_sym);
	priv.adsp_glo_sym_tbl =
		devm_kzalloc(dev,
			sizeof(struct global_sym_info) * num_ent,
			GFP_KERNEL);
	if (!priv.adsp_glo_sym_tbl)
		return -ENOMEM;

	last_sym = sym + num_ent;

	for (i = 1; sym < last_sym; sym++) {
		unsigned char info = sym->st_info;
		unsigned char type = ELF32_ST_TYPE(info);
		if ((ELF32_ST_BIND(sym->st_info) == STB_GLOBAL) &&
		((type == STT_OBJECT) || (type == STT_FUNC))) {
			char *name = priv.adsp_glo_sym_tbl[i].name;
			strncpy(name, name_table + sym->st_name, SYM_NAME_SZ);
			priv.adsp_glo_sym_tbl[i].addr = sym->st_value;
			priv.adsp_glo_sym_tbl[i].info = info;
			i++;
		}
	}
	priv.adsp_glo_sym_tbl[0].addr = i;
	return 0;
}

struct global_sym_info *find_global_symbol(const char *sym_name)
{
	struct device *dev = &priv.pdev->dev;
	struct global_sym_info *table = priv.adsp_glo_sym_tbl;
	int num_ent;
	int i;

	if (unlikely(!table)) {
		dev_info(dev, "symbol table not present\n");
		return 0;
	}
	num_ent = table[0].addr;

	for (i = 1; i < num_ent; i++) {
		if (!strncmp(table[i].name, sym_name, SYM_NAME_SZ))
			return &table[i];
	}
	return NULL;
}

void *get_mailbox_shared_region(void)
{
	const struct firmware *fw;
	struct device *dev;
	struct elf32_shdr *shdr;
	int addr;
	int size;
	int ret;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return ERR_PTR(-EINVAL);
	}

	dev = &priv.pdev->dev;

	ret = request_firmware(&fw, NVADSP_FIRMWARE, dev);
	if (ret < 0) {
		dev_info(dev,
			"reqest firmware for %s failed with %d\n",
					NVADSP_FIRMWARE, ret);
		return ERR_PTR(ret);
	}

	shdr = nvadsp_get_section(fw, MAILBOX_REGION);
	if (!shdr) {
		dev_info(dev, "section %s not found\n", MAILBOX_REGION);
		return ERR_PTR(-EINVAL);
	}

	dev_dbg(dev,
		"the shared section is present at 0x%x\n",
						shdr->sh_addr);
	addr = shdr->sh_addr;
	size = shdr->sh_size;
	return nvadsp_da_to_va_mappings(addr, size);
}

static void copy_io_in_l(void *to, const void *from, int sz)
{
	int i;
	for (i = 0; i <= sz; i += 4) {
		int val = *(int *)(from + i);
		writel(val, to + i);
	}
}

static int nvadsp_os_elf_load(const struct firmware *fw)
{
	struct device *dev = &priv.pdev->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		void *va;
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_dbg(dev,
		"phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
				phdr->p_type, da, memsz, filesz);

		va = nvadsp_da_to_va_mappings(da, filesz);
		if (!va) {
			dev_err(dev, "no va for da 0x%x filesz 0x%x\n",
							da, filesz);
			ret = -EINVAL;
			break;
		}

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
							filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
					offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (filesz) {
			if (!is_adsp_dram_addr(da))
				copy_io_in_l(va,
					elf_data + offset, filesz);
			else
				memcpy(va, elf_data + offset,
							filesz);
		}
	}

	return ret;
}

static int allocate_memory_for_adsp_os(void)
{
	struct platform_device *pdev = priv.pdev;
	struct device *dev = &pdev->dev;
	void *dram_va;
	size_t size;
#if defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	dma_addr_t addr = ADSP_SMMU_LOAD_ADDR;

	size = ADSP_SMMU_SIZE;
	dram_va = dma_alloc_at_coherent(dev, size, &addr, GFP_KERNEL);
	if (!dram_va) {
		dev_info(dev, "unable to allocate SMMU pages\n");
		return PTR_ERR(dram_va);
	}
#else
	phys_addr_t addr;
	struct nvadsp_platform_data *plat_data = pdev->dev.platform_data;

	if (IS_ERR_OR_NULL(plat_data)) {
		dev_info(dev, "carvout is NULL\n");
		return PTR_ERR(plat_data);
	}

	addr = plat_data->co_pa;
	size = plat_data->co_size;
	dram_va = ioremap_nocache(addr, plat_data->co_size);
	if (!dram_va) {
		dev_info(dev, "remap failed for addr %lx\n",
					(long)plat_data->co_pa);
		return PTR_ERR(dram_va);
	}
#endif
	adsp_add_load_mappings(addr, dram_va, size);
	return 0;
}

int nvadsp_os_load(void)
{
	const struct firmware *fw;
	int ret;
	struct device *dev;
	void *ptr;
	struct clk *clk_ape;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		ret = -EINVAL;
		goto end;
	}

	dev = &priv.pdev->dev;

	ret = request_firmware(&fw, NVADSP_FIRMWARE, dev);
	if (ret < 0) {
		dev_info(dev,
			"reqest firmware for %s failed with %d\n",
					NVADSP_FIRMWARE, ret);
		goto end;
	}

	ret = create_global_symbol_table(fw);
	if (ret) {
		dev_info(dev,
			"unable to create global symbol table\n");
		goto end;
	}

	ret = allocate_memory_for_adsp_os();
	if (ret < 0) {
		dev_info(dev,
			"unable to allocate memory for adsp os\n");
		goto end;
	}

	dev_info(dev, "Loading ADSP OS firmware %s\n",
						NVADSP_FIRMWARE);
	clk_ape = clk_get_sys(NULL, "ape");
	if (IS_ERR_OR_NULL(clk_ape)) {
		dev_info(dev, "unable to find ape clock\n");
		goto end;
	}
	tegra_periph_reset_deassert(clk_ape);

	ptr = get_mailbox_shared_region();
	update_nvadsp_app_shared_ptr(ptr);

	ret = nvadsp_os_elf_load(fw);
	if (ret)
		dev_info(dev, "failed to load %s\n", NVADSP_FIRMWARE);
end:
	return ret;
}
EXPORT_SYMBOL(nvadsp_os_load);

int nvadsp_os_start(void)
{
	struct device *dev;
	struct clk *adsp_clk;
	struct clk *ape_uart;
	int val;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return -EINVAL;
	}

	dev = &priv.pdev->dev;

	/*FIXME:this will be replaced by pm_runtime API */
	adsp_clk = clk_get_sys(NULL, "adsp");
	if (IS_ERR_OR_NULL(adsp_clk)) {
		dev_info(dev, "unable to find adsp clock\n");
		return PTR_ERR(adsp_clk);
	}
	tegra_periph_reset_assert(adsp_clk);
	udelay(10);

	val = readl(priv.misc_base + ADSP_CONFIG);
	writel(val | MAXCLKLATENCY, priv.misc_base + ADSP_CONFIG);

	/* TODO: enable ape2apb clock */
	ape_uart = clk_get_sys("uartape", NULL);
	if (IS_ERR_OR_NULL(ape_uart)) {
		dev_info(dev, "unable to find uart ape clk\n");
		return PTR_ERR(ape_uart);
	}

	clk_prepare_enable(ape_uart);
	clk_set_rate(ape_uart, UART_BAUD_RATE * 16);

	dev_info(dev, "starting ADSP OS ....\n");
	tegra_periph_reset_deassert(adsp_clk);

#if !CONFIG_SYSTEM_FPGA
	writel(APE_RESET, priv.reset_reg);
#endif
	wait_for_adsp_os_load_complete();
	return 0;
}
EXPORT_SYMBOL(nvadsp_os_start);


static  irqreturn_t adsp_wdt_handler(int irq, void *arg)
{
	struct device *dev = arg;
	dev_crit(dev, "ADSP OS crashed .... Restarting ADSP OS\n");
#if CONFIG_SYSTEM_FPGA
	if (nvadsp_os_start())
		dev_crit(dev, "Unable to restart ADSP OS\n");
#endif
	return 0;
}

int nvadsp_os_probe(struct platform_device *pdev)
{

	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int virq = tegra_agic_irq_get_virq(INT_ADSP_WDT);
	struct device *dev = &pdev->dev;
	int ret = 0;
#if !CONFIG_SYSTEM_FPGA
	priv.reset_reg = ioremap(APE_FPGA_MISC_RST_DEVICES, 1);
	if (!priv.reset_reg) {
		dev_info(dev, "unable to map reset addr\n");
		ret = -EINVAL;
		goto end;
	}
#endif
	priv.pdev = pdev;
	priv.misc_base = drv_data->base_regs[AMISC];
	priv.dram_region = drv_data->dram_region;

	ret = request_irq(virq, adsp_wdt_handler,
			IRQF_TRIGGER_RISING, "adsp watchdog", dev);
	if (ret)
		dev_err(dev, "failed to get adsp watchdog interrupt\n");

#if !CONFIG_SYSTEM_FPGA
end:
#endif
	return ret;
}
