/*
 * arch/arm/mach-tegra/common.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/irqchip.h>
#include <linux/clk/tegra.h>
#include <linux/highmem.h>
#include <linux/memblock.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/system_info.h>

#include <mach/powergate.h>

#include "board.h"
#include "common.h"
#include "dvfs.h"
#include "fuse.h"
#include "iomap.h"
#include "pm.h"
#include "pmc.h"
#include "apbio.h"
#include "sleep.h"

#define MC_SECURITY_CFG2	0x7c

unsigned long tegra_bootloader_fb_start;
unsigned long tegra_bootloader_fb_size;
unsigned long tegra_fb_start;
unsigned long tegra_fb_size;
unsigned long tegra_fb2_start;
unsigned long tegra_fb2_size;
unsigned long tegra_carveout_start;
unsigned long tegra_carveout_size;
unsigned long tegra_lp0_vec_start;
unsigned long tegra_lp0_vec_size;
unsigned long tegra_grhost_aperture = ~0ul;
static   bool is_tegra_debug_uart_hsport;

/*
 * Storage for debug-macro.S's state.
 *
 * This must be in .data not .bss so that it gets initialized each time the
 * kernel is loaded. The data is declared here rather than debug-macro.S so
 * that multiple inclusions of debug-macro.S point at the same data.
 */
u32 tegra_uart_config[4] = {
	/* Debug UART initialization required */
	1,
	/* Debug UART physical address */
	0,
	/* Debug UART virtual address */
	0,
	/* Scratch space for debug macro */
	0,
};

#ifdef CONFIG_OF
void __init tegra_dt_init_irq(void)
{
	tegra_clocks_init();
	tegra_pmc_init();
	tegra_init_irq();
	irqchip_init();
}
#endif

void tegra_assert_system_reset(char mode, const char *cmd)
{
	void __iomem *reset = IO_ADDRESS(TEGRA_PMC_BASE + 0);
	u32 reg;

	reg = readl_relaxed(reset);
	reg |= 0x10;
	writel_relaxed(reg, reset);
}

void tegra_init_cache(u32 tag_latency, u32 data_latency)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;
	u32 aux_ctrl, cache_type;
	u32 tag_latency, data_latency;

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	tag_latency = 0x331;
	data_latency = 0x441;
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
#ifdef CONFIG_TEGRA_FPGA_PLATFORM
	tag_latency = 0x770;
	data_latency = 0x770;
#else
	if (is_lp_cluster()) {
		tag_latency = 0x221;
		data_latency = 0x221;
	} else {
		tag_latency = 0x331;
		data_latency = 0x441;
	}
#endif
#endif
	writel_relaxed(tag_latency, p + L2X0_TAG_LATENCY_CTRL);
	writel_relaxed(data_latency, p + L2X0_DATA_LATENCY_CTRL);

#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
#ifndef CONFIG_TEGRA_FPGA_PLATFORM
	writel(7, p + L2X0_PREFETCH_CTRL);
	writel(2, p + L2X0_POWER_CTRL);
#endif	

	/* Enable PL310 double line fill feature. */
	writel(((1<<30) | 7), p + L2X0_PREFETCH_CTRL);
#endif

	cache_type = readl(p + L2X0_CACHE_TYPE);
	aux_ctrl = (cache_type & 0x700) << (17-8);
	aux_ctrl |= 0x7C400001;

	l2x0_init(p, aux_ctrl, 0x8200c3fe);
#endif
}

static void __init tegra_init_power(void)
{
	tegra_powergate_power_off(TEGRA_POWERGATE_MPE);
	tegra_powergate_power_off(TEGRA_POWERGATE_3D);
	tegra_powergate_power_off(TEGRA_POWERGATE_PCIE);
}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
void __init tegra20_init_early(void)
{
	tegra_apb_io_init();
	tegra_init_fuse();
	tegra_init_cache();
	tegra_powergate_init();
	tegra20_hotplug_init();
	tegra_init_power();
	tegra20_mc_init(); /* !!!FIXME!!! Change Tegra3 behavior to match Tegra2 */
}
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
void __init tegra30_init_early(void)
{
	tegra_apb_io_init();
	tegra_init_fuse();
	tegra_init_cache();
	tegra_pmc_init();
	tegra_powergate_init();
	tegra30_hotplug_init();
	tegra_init_power();
	tegra30_mc_init();
}
#endif

static int __init tegra_lp0_vec_arg(char *options)
{
	char *p = options;

	tegra_lp0_vec_size = memparse(p, &p);
	if (*p == '@')
		tegra_lp0_vec_start = memparse(p+1, &p);
	if (!tegra_lp0_vec_size || !tegra_lp0_vec_start) {
		tegra_lp0_vec_size = 0;
		tegra_lp0_vec_start = 0;
	}

	return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);

static int __init tegra_bootloader_fb_arg(char *options)
{
	char *p = options;
	unsigned long start;
	unsigned long size;

	tegra_bootloader_fb_size = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_fb_start = memparse(p+1, &p);

	pr_info("Found tegra_fbmem: %08lx@%08lx\n",
		tegra_bootloader_fb_size, tegra_bootloader_fb_start);

	return 0;
}
early_param("tegra_fbmem", tegra_bootloader_fb_arg);

static int __init tegra_debug_uartport(char *info)
{
	if (!strcmp(info, "hsport"))
		is_tegra_debug_uart_hsport = true;
	else if (!strcmp(info, "lsport"))
		is_tegra_debug_uart_hsport = false;

	return 1;
}

bool is_tegra_debug_uartport_hs(void)
{
	return is_tegra_debug_uart_hsport;
}

__setup("debug_uartport=", tegra_debug_uartport);

void tegra_get_board_info(struct board_info *bi)
{
	bi->board_id = (system_serial_high >> 16) & 0xFFFF;
	bi->sku = (system_serial_high) & 0xFFFF;
	bi->fab = (system_serial_low >> 24) & 0xFF;
	bi->major_revision = (system_serial_low >> 16) & 0xFF;
	bi->minor_revision = (system_serial_low >> 8) & 0xFF;
}

/*
 * Tegra has a protected aperture that prevents access by most non-CPU
 * memory masters to addresses above the aperture value.  Enabling it
 * secures the CPU's memory from the GPU, except through the GART.
 */
void __init tegra_protected_aperture_init(unsigned long aperture)
{
#ifndef CONFIG_NVMAP_ALLOW_SYSMEM
	void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);
	pr_info("Enabling Tegra protected aperture at 0x%08lx\n", aperture);
	writel(aperture, mc_base + MC_SECURITY_CFG2);
#else
	pr_err("Tegra protected aperture disabled because nvmap is using "
		"system memory\n");
#endif
}

/*
 * Due to conflicting restrictions on the placement of the framebuffer,
 * the bootloader is likely to leave the framebuffer pointed at a location
 * in memory that is outside the grhost aperture.  This function will move
 * the framebuffer contents from a physical address that is anywher (lowmem,
 * highmem, or outside the memory map) to a physical address that is outside
 * the memory map.
 */
void tegra_move_framebuffer(unsigned long to, unsigned long from,
	unsigned long size)
{
	struct page *page;
	void __iomem *to_io;
	void *from_virt;
	unsigned long i;

	BUG_ON(PAGE_ALIGN((unsigned long)to) != (unsigned long)to);
	BUG_ON(PAGE_ALIGN(from) != from);
	BUG_ON(PAGE_ALIGN(size) != size);

	to_io = ioremap(to, size);
	if (!to_io) {
		pr_err("%s: Failed to map target framebuffer\n", __func__);
		return;
	}

	if (pfn_valid(page_to_pfn(phys_to_page(from)))) {
		for (i = 0 ; i < size; i += PAGE_SIZE) {
			page = phys_to_page(from + i);
			from_virt = kmap(page);
			memcpy_toio(to_io + i, from_virt, PAGE_SIZE);
			kunmap(page);
		}
	} else {
		void __iomem *from_io = ioremap(from, size);
		if (!from_io) {
			pr_err("%s: Failed to map source framebuffer\n",
				__func__);
			goto out;
		}

		for (i = 0; i < size; i+= 4)
			writel(readl(from_io + i), to_io + i);

		iounmap(from_io);
	}
out:
	iounmap(to_io);
}

void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
	unsigned long fb2_size)
{
	if (tegra_lp0_vec_size)
		if (memblock_reserve(tegra_lp0_vec_start, tegra_lp0_vec_size)) {
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);
			tegra_lp0_vec_start = 0;
			tegra_lp0_vec_size = 0;
		}

	if (carveout_size) {
		tegra_carveout_start = memblock_end_of_DRAM() - carveout_size;
		if (memblock_remove(tegra_carveout_start, carveout_size)) {
			pr_err("Failed to remove carveout %08lx@%08lx "
				"from memory map\n",
				tegra_carveout_start, carveout_size);
			tegra_carveout_start = 0;
			tegra_carveout_size = 0;
		}
		else
			tegra_carveout_size = carveout_size;
	}

	if (fb2_size) {
		tegra_fb2_start = memblock_end_of_DRAM() - fb2_size;
		if (memblock_remove(tegra_fb2_start, fb2_size)) {
			pr_err("Failed to remove second framebuffer %08lx@%08lx "
				"from memory map\n",
				tegra_fb2_start, fb2_size);
			tegra_fb2_start = 0;
			tegra_fb2_size = 0;
		} else
			tegra_fb2_size = fb2_size;
	}

	if (fb_size) {
		tegra_fb_start = memblock_end_of_DRAM() - fb_size;
		if (memblock_remove(tegra_fb_start, fb_size)) {
			pr_err("Failed to remove framebuffer %08lx@%08lx "
				"from memory map\n",
				tegra_fb_start, fb_size);
			tegra_fb_start = 0;
			tegra_fb_size = 0;
		} else
			tegra_fb_size = fb_size;
	}

	if (tegra_fb_size)
		tegra_grhost_aperture = tegra_fb_start;

	if (tegra_fb2_size && tegra_fb2_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_fb2_start;

	if (tegra_carveout_size && tegra_carveout_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_carveout_start;

#ifdef CONFIG_TEGRA_IOVMM_SMMU
	if (memblock_reserve(TEGRA_SMMU_BASE, TEGRA_SMMU_SIZE)) {
		pr_err("Failed to reserve SMMU I/O window %08x@%08x\n",
			TEGRA_SMMU_BASE, TEGRA_SMMU_SIZE);
	}
#endif

	/*
	 * TODO: We should copy the bootloader's framebuffer to the framebuffer
	 * allocated above, and then free this one.
	 */
	if (tegra_bootloader_fb_size)
		if (memblock_reserve(tegra_bootloader_fb_start,
				tegra_bootloader_fb_size)) {
			pr_err("Failed to reserve bootloader frame buffer %08lx@%08lx\n",
				tegra_bootloader_fb_size, tegra_bootloader_fb_start);
			tegra_bootloader_fb_start = 0;
			tegra_bootloader_fb_size = 0;
		}

	pr_info("Tegra reserved memory:\n"
		"LP0:                    %08lx - %08lx\n"
		"Bootloader framebuffer: %08lx - %08lx\n"
		"Framebuffer:            %08lx - %08lx\n"
		"2nd Framebuffer:        %08lx - %08lx\n"
		"Carveout:               %08lx - %08lx\n",
		tegra_lp0_vec_start,
		tegra_lp0_vec_size ?
			tegra_lp0_vec_start + tegra_lp0_vec_size - 1 : 0,
		tegra_bootloader_fb_start,
		tegra_bootloader_fb_size ?
			tegra_bootloader_fb_start + tegra_bootloader_fb_size - 1 : 0,
		tegra_fb_start,
		tegra_fb_size ?
			tegra_fb_start + tegra_fb_size - 1 : 0,
		tegra_fb2_start,
		tegra_fb2_size ?
			tegra_fb2_start + tegra_fb2_size - 1 : 0,
		tegra_carveout_start,
		tegra_carveout_size ?
			tegra_carveout_start + tegra_carveout_size - 1 : 0);

#ifdef CONFIG_TEGRA_IOVMM_SMMU
	pr_info("SMMU:                   %08x - %08x\n",
		TEGRA_SMMU_BASE, TEGRA_SMMU_BASE + TEGRA_SMMU_SIZE - 1);
#endif
}

void __init tegra_init_late(void)
{
#ifndef CONFIG_COMMON_CLK
	tegra_clk_debugfs_init();
#endif
	tegra_powergate_debugfs_init();
}
