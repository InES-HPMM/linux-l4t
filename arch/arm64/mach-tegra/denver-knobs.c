/*
 * Copyright (c) 2013-2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>

#include <mach/hardware.h>

#include "denver-knobs.h"

#define BG_CLR_SHIFT	16
#define BG_STATUS_SHIFT	32

#ifdef CONFIG_DEBUG_FS
struct denver_creg {
	unsigned long offset;
	const char *name;
};

struct denver_creg denver_cregs[] = {
	{0x00000000, "ifu.mcstatinfo"},
	{0x00000001, "jsr.ret_mcstatinfo"},
	{0x00000002, "jsr.mts_mcstatinfo"},
	{0x00000003, "l2i.mcstatinfo"},
	{0x00000004, "dcc.mcstatinfo1"},
	{0x00000005, "dcc.mcstatinfo2"},
	{0x00000006, "lvb.mcstatinfo3"},
	{0x00000010, "jsr.pending_traps_set"},
	{0x00000011, "jsr.int_sw"},
	{0x0000001e, "dec.pcr_type"},
	{0x0000001f, "dec.pcr_valid"},
	{0x00000020, "dec.pcr_match_0"},
	{0x00000021, "dec.pcr_match_1"},
	{0x00000022, "dec.pcr_match_2"},
	{0x00000023, "dec.pcr_match_3"},
	{0x00000024, "dec.pcr_match_4"},
	{0x00000025, "dec.pcr_match_5"},
	{0x00000026, "dec.pcr_match_6"},
	{0x00000027, "dec.pcr_match_7"},
	{0x00000028, "dec.pcr_mask_0"},
	{0x00000029, "dec.pcr_mask_1"},
	{0x0000002a, "dec.pcr_mask_2"},
	{0x0000002b, "dec.pcr_mask_3"},
	{0x0000002c, "dec.pcr_mask_4"},
	{0x0000002d, "dec.pcr_mask_5"},
	{0x0000002e, "dec.pcr_mask_6"},
	{0x0000002f, "dec.pcr_mask_7"},
	{0x00000030, "mm.tom1"},
	{0x00000031, "mm.tom2"},
	{0x00000032, "mm.tom3"},
	{0x00000033, "mm.tom4"},
	{0x00000034, "mm.bom1"},
	{0x00000035, "mm.bom2"},
	{0x00000036, "mm.bom3"},
	{0x00000037, "mm.bom4"},
	{0x00000038, "mm.cheese"},
	{0x00000040, "mm.dmtrr_base0"},
	{0x00000041, "mm.dmtrr_base1"},
	{0x00000042, "mm.dmtrr_base2"},
	{0x00000043, "mm.dmtrr_base3"},
	{0x00000044, "mm.dmtrr_base4"},
	{0x00000045, "mm.dmtrr_base5"},
	{0x00000046, "mm.dmtrr_base6"},
	{0x00000047, "mm.dmtrr_base7"},
	{0x00000048, "mm.dmtrr_mask0"},
	{0x00000049, "mm.dmtrr_mask1"},
	{0x0000004a, "mm.dmtrr_mask2"},
	{0x0000004b, "mm.dmtrr_mask3"},
	{0x0000004c, "mm.dmtrr_mask4"},
	{0x0000004d, "mm.dmtrr_mask5"},
	{0x0000004e, "mm.dmtrr_mask6"},
	{0x0000004f, "mm.dmtrr_mask7"}
};

#define NR_CREGS (sizeof(denver_cregs) / sizeof(struct denver_creg))

enum creg_command {
	CREG_INDEX = 1,
	CREG_READ,
	CREG_WRITE
};
#endif

bool denver_get_bg_allowed(int cpu)
{
	unsigned long regval;

	asm volatile("mrs %0, s3_0_c15_c0_2" : "=r" (regval) : );
	regval = (regval >> BG_STATUS_SHIFT) & 0xffff;

	return !!(regval & (1 << cpu));
}

void denver_set_bg_allowed(int cpu, bool enable)
{
	unsigned long regval;

	BUG_ON(cpu >= num_present_cpus());

	regval = 1 << cpu;
	if (!enable)
		regval <<= BG_CLR_SHIFT;

	asm volatile("msr s3_0_c15_c0_2, %0" : : "r" (regval));
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *denver_debugfs_root;

static int bgallowed_get(void *data, u64 *val)
{
	int cpu = (int)((s64)data);
	*val = denver_get_bg_allowed(cpu);
	return 0;
}

static int bgallowed_set(void *data, u64 val)
{
	int cpu = (int)((s64)data);
	denver_set_bg_allowed(cpu, (bool)val);
	pr_debug("CPU%d: BGALLOWED is %s.\n",
			cpu, val ? "enabled" : "disabled");
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bgallowed_fops, bgallowed_get,
			bgallowed_set, "%llu\n");

static int __init create_denver_bgallowed(void)
{
	int cpu;
	char name[30];

	for_each_present_cpu(cpu) {
		snprintf(name, 30, "bgallowed_cpu%d", cpu);
		if (!debugfs_create_file(
			name, S_IRUGO, denver_debugfs_root,
			(void *)((s64)cpu), &bgallowed_fops)) {
			pr_info("ERROR: failed to create bgallow_fs");
			return -ENOMEM;
		}
	}

	return 0;
}

static struct dentry *denver_creg_root;

static int denver_creg_get(void *data, u64 *val)
{
	struct denver_creg *reg = (struct denver_creg *)data;

	asm volatile (
	"	sys 0, c11, c0, 1, %1\n"
	"	sys 0, c11, c0, 0, %2\n"
	"	sys 0, c11, c0, 0, %3\n"
	"	sysl %0, 0, c11, c0, 0\n"
	: "=r" (*val)
	: "r" (reg->offset), "r" (CREG_INDEX), "r" (CREG_READ)
	);

	return 0;
}

static int denver_creg_set(void *data, u64 val)
{
	struct denver_creg *reg = (struct denver_creg *)data;
	pr_debug("CREG: write %s @ 0x%lx =  0x%llx\n",
			reg->name, reg->offset, val);

	asm volatile (
	"	sys 0, c11, c0, 1, %0\n"
	"	sys 0, c11, c0, 0, %1\n"
	"	sys 0, c11, c0, 1, %2\n"
	"	sys 0, c11, c0, 0, %3\n"
	:
	: "r" (reg->offset), "r" (CREG_INDEX),
	  "r" (val), "r" (CREG_WRITE)
	);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(denver_creg_fops, denver_creg_get,
			denver_creg_set, "%llu\n");

static int __init create_denver_cregs(void)
{
	int i;
	struct denver_creg *reg;

	denver_creg_root = debugfs_create_dir(
			"cregs", denver_debugfs_root);

	for (i = 0; i < NR_CREGS; ++i) {
		reg = &denver_cregs[i];
		if (!debugfs_create_file(
			reg->name, S_IRUGO, denver_creg_root,
			reg, &denver_creg_fops))
			return -ENOMEM;
	}

	return 0;
}

static u64 nvmstat0_pg_agg;
static u64 nvmstat0_tot_agg;

static u64 nvmstat1_bg_agg;
static u64 nvmstat1_fg_agg;

static int inst_stats_show(struct seq_file *s, void *data)
{
	u64 nvmstat0;
	u32 nvmstat0_pg;
	u32 nvmstat0_tot;

	u64 nvmstat1;
	u32 nvmstat1_bg;
	u32 nvmstat1_fg;

	/* read nvmstat0 */
	asm volatile("mrs %0, s3_0_c15_c0_0" : "=r" (nvmstat0) : );
	nvmstat0_pg = (u32)(nvmstat0);
	nvmstat0_pg_agg += nvmstat0_pg;
	nvmstat0_tot = (u32)(nvmstat0 >> 32);
	nvmstat0_tot_agg += nvmstat0_tot;

	/* read nvmstat1 */
	asm volatile("mrs %0, s3_0_c15_c0_1" : "=r" (nvmstat1) : );
	nvmstat1_bg = (u32)(nvmstat1);
	nvmstat1_bg_agg += nvmstat1_bg;
	nvmstat1_fg = (u32)(nvmstat1 >> 32);
	nvmstat1_fg_agg += nvmstat1_fg;

	seq_printf(s, "nvmstat0 = %llu\n", nvmstat0);
	seq_printf(s, "nvmstat0_pg = %u\n", nvmstat0_pg);
	seq_printf(s, "nvmstat0_tot = %u\n", nvmstat0_tot);

	seq_printf(s, "nvmstat1 = %llu\n", nvmstat1);
	seq_printf(s, "nvmstat1_bg = %u\n", nvmstat1_bg);
	seq_printf(s, "nvmstat1_fg = %u\n", nvmstat1_fg);

	/* reset nvmstat0 */
	nvmstat0 = 0;
	asm volatile("msr s3_0_c15_c0_0, %0" : : "r" (nvmstat0));

	return 0;
}

static int inst_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, inst_stats_show, inode->i_private);
}

static const struct file_operations inst_stats_fops = {
	.open		= inst_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int agg_stats_show(struct seq_file *s, void *data)
{
	seq_printf(s, "nvmstat0_pg_agg = %llu\n", nvmstat0_pg_agg);
	seq_printf(s, "nvmstat0_tot_agg = %llu\n", nvmstat0_tot_agg);

	seq_printf(s, "nvmstat1_bg_agg = %llu\n", nvmstat1_bg_agg);
	seq_printf(s, "nvmstat1_fg_agg = %llu\n", nvmstat1_fg_agg);

	return 0;
}

static int agg_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, agg_stats_show, inode->i_private);
}

static const struct file_operations agg_stats_fops = {
	.open		= agg_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init create_denver_nvmstats(void)
{
	struct dentry *nvmstats_dir;

	nvmstats_dir = debugfs_create_dir("nvmstats", denver_debugfs_root);
	if (!nvmstats_dir) {
		pr_err("%s: Couldn't create the \"nvmstats\" debugfs node.\n",
			__func__);
		return -1;
	}

	if (!debugfs_create_file("instantaneous_stats",
				S_IRUGO,
				nvmstats_dir,
				NULL,
				&inst_stats_fops)) {
		pr_err("%s: Couldn't create the \"instantaneous_stats\" debugfs node.\n",
			__func__);
		return -1;
	}

	if (!debugfs_create_file("aggregated_stats",
				S_IRUGO,
				nvmstats_dir,
				NULL,
				&agg_stats_fops)) {
		pr_err("%s: Couldn't create the \"aggregated_stats\" debugfs node.\n",
			__func__);
		return -1;
	}

	return 0;
}

static int __init denver_knobs_init(void)
{
	int error;

	if (tegra_cpu_is_asim())
		return 0;

	denver_debugfs_root = debugfs_create_dir("tegra_denver", NULL);

	error = create_denver_bgallowed();
	if (error)
		return error;

	error = create_denver_cregs();
	if (error)
		return error;

	error = create_denver_nvmstats();
	if (error)
		return error;

	return 0;
}
device_initcall(denver_knobs_init);

#endif
