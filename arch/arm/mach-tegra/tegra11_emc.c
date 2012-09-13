/*
 * arch/arm/mach-tegra/tegra11_emc.c
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_emc.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/hrtimer.h>

#include <asm/cputime.h>

#include <mach/iomap.h>

#include "clock.h"
#include "dvfs.h"
#include "tegra11_emc.h"

#ifdef CONFIG_TEGRA_EMC_SCALING_ENABLE
static bool emc_enable = true;
#else
static bool emc_enable;
#endif
module_param(emc_enable, bool, 0644);

u8 tegra_emc_bw_efficiency = 100;

#define PLL_C_DIRECT_FLOOR		333500000
#define EMC_STATUS_UPDATE_TIMEOUT	100
#define TEGRA_EMC_TABLE_MAX_SIZE	16


#define EMC_CLK_DIV_SHIFT		0
#define EMC_CLK_DIV_MAX_VALUE		0xFF
#define EMC_CLK_DIV_MASK		(0xFF << EMC_CLK_DIV_SHIFT)
#define EMC_CLK_SOURCE_SHIFT		29
#define EMC_CLK_SOURCE_MAX_VALUE	3
#define EMC_CLK_LOW_JITTER_ENABLE	(0x1 << 31)
#define	EMC_CLK_MC_SAME_FREQ		(0x1 << 16)

/* FIXME: actual Tegar11 list */
#define BURST_REG_LIST \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RC),			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RFC),			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RAS),			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RP),			\
								\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_MISC0),

#define DEFINE_REG(base, reg) ((base) ? (IO_ADDRESS((base)) + (reg)) : 0)
static void __iomem *burst_reg_addr[TEGRA11_EMC_MAX_NUM_REGS] = {
	BURST_REG_LIST
};
#undef DEFINE_REG

#define DEFINE_REG(base, reg)	reg##_INDEX
enum {
	BURST_REG_LIST
};
#undef DEFINE_REG

static int emc_num_burst_regs;

struct emc_sel {
	struct clk	*input;
	u32		value;
	unsigned long	input_rate;
};
static struct emc_sel tegra_emc_clk_sel[TEGRA_EMC_TABLE_MAX_SIZE];
static struct tegra11_emc_table start_timing;
static const struct tegra11_emc_table *emc_timing;

static ktime_t clkchange_time;
static int clkchange_delay = 100;

static const struct tegra11_emc_table *tegra_emc_table;
static int tegra_emc_table_size;

static u32 dram_dev_num;
static u32 dram_type = -1;

static struct clk *emc;

static struct {
	cputime64_t time_at_clock[TEGRA_EMC_TABLE_MAX_SIZE];
	int last_sel;
	u64 last_update;
	u64 clkchange_count;
	spinlock_t spinlock;
} emc_stats;

static DEFINE_SPINLOCK(emc_access_lock);

static void __iomem *emc_base = IO_ADDRESS(TEGRA_EMC_BASE);
static void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);

static inline void emc_writel(u32 val, unsigned long addr)
{
	writel(val, (u32)emc_base + addr);
}
static inline u32 emc_readl(unsigned long addr)
{
	return readl((u32)emc_base + addr);
}
static inline void mc_writel(u32 val, unsigned long addr)
{
	writel(val, (u32)mc_base + addr);
}
static inline u32 mc_readl(unsigned long addr)
{
	return readl((u32)mc_base + addr);
}

static void emc_last_stats_update(int last_sel)
{
	unsigned long flags;
	u64 cur_jiffies = get_jiffies_64();

	spin_lock_irqsave(&emc_stats.spinlock, flags);

	if (emc_stats.last_sel < TEGRA_EMC_TABLE_MAX_SIZE)
		emc_stats.time_at_clock[emc_stats.last_sel] =
			emc_stats.time_at_clock[emc_stats.last_sel] +
			(cur_jiffies - emc_stats.last_update);

	emc_stats.last_update = cur_jiffies;

	if (last_sel < TEGRA_EMC_TABLE_MAX_SIZE) {
		emc_stats.clkchange_count++;
		emc_stats.last_sel = last_sel;
	}
	spin_unlock_irqrestore(&emc_stats.spinlock, flags);
}

static noinline void emc_set_clock(const struct tegra11_emc_table *next_timing,
				   const struct tegra11_emc_table *last_timing,
				   u32 clk_setting)
{
	/* FIXME: implement */
	pr_info("tegra11_emc: Configuring EMC rate %lu (setting: 0x%x)\n",
		next_timing->rate, clk_setting);
}

static inline void emc_get_timing(struct tegra11_emc_table *timing)
{
	int i;

	for (i = 0; i < emc_num_burst_regs; i++) {
		if (burst_reg_addr[i])
			timing->burst_regs[i] = __raw_readl(burst_reg_addr[i]);
		else
			timing->burst_regs[i] = 0;
	}
	timing->emc_acal_interval = 0;
	timing->emc_zcal_cnt_long = 0;
	timing->emc_mode_reset = 0;
	timing->emc_mode_1 = 0;
	timing->emc_mode_2 = 0;
	timing->emc_periodic_qrst = (emc_readl(EMC_CFG) &
				     EMC_CFG_PERIODIC_QRST) ? 1 : 0;
	timing->rate = clk_get_rate_locked(emc);
}

/* The EMC registers have shadow registers. When the EMC clock is updated
 * in the clock controller, the shadow registers are copied to the active
 * registers, allowing glitchless memory bus frequency changes.
 * This function updates the shadow registers for a new clock frequency,
 * and relies on the clock lock on the emc clock to avoid races between
 * multiple frequency changes. In addition access lock prevents concurrent
 * access to EMC registers from reading MRR registers */
int tegra_emc_set_rate(unsigned long rate)
{
	int i;
	u32 clk_setting;
	const struct tegra11_emc_table *last_timing;
	unsigned long flags;
	s64 last_change_delay;

	if (!tegra_emc_table)
		return -EINVAL;

	/* Table entries specify rate in kHz */
	rate = rate / 1000;

	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_clk_sel[i].input == NULL)
			continue;	/* invalid entry */

		if (tegra_emc_table[i].rate == rate)
			break;
	}

	if (i >= tegra_emc_table_size)
		return -EINVAL;

	if (!emc_timing) {
		/* can not assume that boot timing matches dfs table even
		   if boot frequency matches one of the table nodes */
		emc_get_timing(&start_timing);
		last_timing = &start_timing;
	}
	else
		last_timing = emc_timing;

	clk_setting = tegra_emc_clk_sel[i].value;

	last_change_delay = ktime_us_delta(ktime_get(), clkchange_time);
	if ((last_change_delay >= 0) && (last_change_delay < clkchange_delay))
		udelay(clkchange_delay - (int)last_change_delay);

	spin_lock_irqsave(&emc_access_lock, flags);
	emc_set_clock(&tegra_emc_table[i], last_timing, clk_setting);
	clkchange_time = ktime_get();
	emc_timing = &tegra_emc_table[i];
	spin_unlock_irqrestore(&emc_access_lock, flags);

	emc_last_stats_update(i);

	pr_debug("%s: rate %lu setting 0x%x\n", __func__, rate, clk_setting);

	return 0;
}

long tegra_emc_round_rate(unsigned long rate)
{
	int i;

	if (!tegra_emc_table)
		return clk_get_rate_locked(emc); /* no table - no rate change */

	if (!emc_enable)
		return -EINVAL;

	pr_debug("%s: %lu\n", __func__, rate);

	/* Table entries specify rate in kHz */
	rate = rate / 1000;

	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_clk_sel[i].input == NULL)
			continue;	/* invalid entry */

		if (tegra_emc_table[i].rate >= rate) {
			pr_debug("%s: using %lu\n",
				 __func__, tegra_emc_table[i].rate);
			return tegra_emc_table[i].rate * 1000;
		}
	}

	return -EINVAL;
}

struct clk *tegra_emc_predict_parent(unsigned long rate, u32 *div_value)
{
	int i;

	if (!tegra_emc_table) {
		if (rate == clk_get_rate_locked(emc)) {
			*div_value = emc->div - 2;
			return emc->parent;
		}
		return NULL;
	}

	pr_debug("%s: %lu\n", __func__, rate);

	/* Table entries specify rate in kHz */
	rate = rate / 1000;

	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_table[i].rate == rate) {
			struct clk *p = tegra_emc_clk_sel[i].input;

			if (p && (tegra_emc_clk_sel[i].input_rate ==
				  clk_get_rate(p))) {
				*div_value = (tegra_emc_clk_sel[i].value &
					EMC_CLK_DIV_MASK) >> EMC_CLK_DIV_SHIFT;
				return p;
			}
		}
	}
	return NULL;
}

bool tegra_emc_is_parent_ready(unsigned long rate, struct clk **parent,
		unsigned long *parent_rate, unsigned long *backup_rate)
{

	int i;
	struct clk *p = NULL;
	unsigned long p_rate = 0;

	if (!tegra_emc_table || !emc_enable)
		return true;

	pr_debug("%s: %lu\n", __func__, rate);

	/* Table entries specify rate in kHz */
	rate = rate / 1000;

	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_table[i].rate == rate) {
			p = tegra_emc_clk_sel[i].input;
			if (!p)
				continue;	/* invalid entry */

			p_rate = tegra_emc_clk_sel[i].input_rate;
			if (p_rate == clk_get_rate(p))
				return true;
			break;
		}
	}

	/* Table match not found - "non existing parent" is ready */
	if (!p)
		return true;

	/*
	 * Table match found, but parent is not ready - continue search
	 * for backup rate: min rate above requested that has different
	 * parent source (since only pll_c is scaled and may not be ready,
	 * any other parent can provide backup)
	 */
	*parent = p;
	*parent_rate = p_rate;

	for (i++; i < tegra_emc_table_size; i++) {
		p = tegra_emc_clk_sel[i].input;
		if (!p)
			continue;	/* invalid entry */

		if (p != (*parent)) {
			*backup_rate = tegra_emc_table[i].rate * 1000;
			return false;
		}
	}

	/* Parent is not ready, and no backup found */
	*backup_rate = -EINVAL;
	return false;
}

static int find_matching_input(const struct tegra11_emc_table *table,
			struct clk *pll_c, struct emc_sel *emc_clk_sel)
{
	u32 div_value = 0;
	unsigned long input_rate = 0;
	unsigned long table_rate = table->rate * 1000; /* table rate in kHz */
	struct clk *src = tegra_get_clock_by_name(table->src_name);
	const struct clk_mux_sel *sel;

	for (sel = emc->inputs; sel->input != NULL; sel++) {
		if (sel->input != src)
			continue;
		/*
		 * PLLC is a scalable source. For rates below PLL_C_DIRECT_FLOOR
		 * configure PLLC at double rate and set 1:2 divider, otherwise
		 * configure PLLC at target rate with divider 1:1.
		 */
		if (src == pll_c) {
#ifdef CONFIG_TEGRA_DUAL_CBUS
			if (table_rate < PLL_C_DIRECT_FLOOR) {
				input_rate = 2 * table_rate;
				div_value = 2;
			} else {
				input_rate = table_rate;
				div_value = 0;
			}
			break;
#else
			continue;	/* pll_c is used for cbus - skip */
#endif
		}

		/*
		 * All other clock sources are fixed rate sources, and must
		 * run at rate that is an exact multiple of the target.
		 */
		input_rate = clk_get_rate(src);

		if ((input_rate >= table_rate) &&
		     (input_rate % table_rate == 0)) {
			div_value = 2 * input_rate / table_rate - 2;
			break;
		}
	}

	if (!sel->input || (sel->value > EMC_CLK_SOURCE_MAX_VALUE) ||
	    (div_value > EMC_CLK_DIV_MAX_VALUE)) {
		pr_warn("tegra: no matching input found for EMC rate %lu\n",
			table_rate);
		return -EINVAL;
	}

	emc_clk_sel->input = sel->input;
	emc_clk_sel->input_rate = input_rate;

	/* Get ready emc clock selection settings for this table rate */
	emc_clk_sel->value = sel->value << EMC_CLK_SOURCE_SHIFT;
	emc_clk_sel->value |= (div_value << EMC_CLK_DIV_SHIFT);
	if ((div_value == 0) && (emc_clk_sel->input == emc->parent))
		emc_clk_sel->value |= EMC_CLK_LOW_JITTER_ENABLE;

	if (MC_EMEM_ARB_MISC0_EMC_SAME_FREQ &
	    table->burst_regs[MC_EMEM_ARB_MISC0_INDEX])
		emc_clk_sel->value |= EMC_CLK_MC_SAME_FREQ;

	return 0;
}

static void adjust_emc_dvfs_table(const struct tegra11_emc_table *table,
				  int table_size)
{
	int i, j;
	unsigned long rate;

	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		int mv = emc->dvfs->millivolts[i];
		if (!mv)
			break;

		/* For each dvfs voltage find maximum supported rate;
		   use 1MHz placeholder if not found */
		for (rate = 1000, j = 0; j < table_size; j++) {
			if (tegra_emc_clk_sel[j].input == NULL)
				continue;	/* invalid entry */

			if ((mv >= table[j].emc_min_mv) &&
			    (rate < table[j].rate))
				rate = table[j].rate;
		}
		/* Table entries specify rate in kHz */
		emc->dvfs->freqs[i] = rate * 1000;
	}
}

static int init_emc_table(const struct tegra11_emc_table *table, int table_size)
{
	int i, mv;
	u32 reg;
	bool max_entry = false;
	unsigned long boot_rate, max_rate;
	struct clk *pll_c = tegra_get_clock_by_name("pll_c");

	emc_stats.clkchange_count = 0;
	spin_lock_init(&emc_stats.spinlock);
	emc_stats.last_update = get_jiffies_64();
	emc_stats.last_sel = TEGRA_EMC_TABLE_MAX_SIZE;

	boot_rate = clk_get_rate(emc) / 1000;
	max_rate = clk_get_max_rate(emc) / 1000;

	if ((dram_type != DRAM_TYPE_DDR3) && (dram_type != DRAM_TYPE_LPDDR2)) {
		pr_err("tegra: not supported DRAM type %u\n", dram_type);
		return -ENODATA;
	}

	if (emc->parent != tegra_get_clock_by_name("pll_m")) {
		pr_err("tegra: boot parent %s is not supported by EMC DFS\n",
			emc->parent->name);
		return -ENODATA;
	}

	if (!table || !table_size) {
		pr_err("tegra: EMC DFS table is empty\n");
		return -ENODATA;
	}

	tegra_emc_table_size = min(table_size, TEGRA_EMC_TABLE_MAX_SIZE);
	switch (table[0].rev) {
	case 0x40:
		emc_num_burst_regs = 105; /* FIXME: actual number */
		break;
	default:
		pr_err("tegra: invalid EMC DFS table: unknown rev 0x%x\n",
			table[0].rev);
		return -ENODATA;
	}

	/* Match EMC source/divider settings with table entries */
	for (i = 0; i < tegra_emc_table_size; i++) {
		unsigned long table_rate = table[i].rate;

		/* Skip "no-rate" entry, or entry violating ascending order */
		if (!table_rate ||
		    (i && (table_rate <= table[i-1].rate)))
			continue;

		BUG_ON(table[i].rev != table[0].rev);

		if (find_matching_input(&table[i], pll_c,
					&tegra_emc_clk_sel[i]))
			continue;

		if (table_rate == boot_rate)
			emc_stats.last_sel = i;

		if (table_rate == max_rate)
			max_entry = true;
	}

	/* Validate EMC rate and voltage limits */
	if (!max_entry) {
		pr_err("tegra: invalid EMC DFS table: entry for max rate"
		       " %lu kHz is not found\n", max_rate);
		return -ENODATA;
	}

	tegra_emc_table = table;

	if (emc->dvfs) {
		adjust_emc_dvfs_table(tegra_emc_table, tegra_emc_table_size);
		mv = tegra_dvfs_predict_millivolts(emc, max_rate * 1000);
		if ((mv <= 0) || (mv > emc->dvfs->max_millivolts)) {
			tegra_emc_table = NULL;
			pr_err("tegra: invalid EMC DFS table: maximum rate %lu"
			       " kHz does not match nominal voltage %d\n",
			       max_rate, emc->dvfs->max_millivolts);
			return -ENODATA;
		}
	}

	pr_info("tegra: validated EMC DFS table\n");

	/* Configure clock change mode according to dram type */
	reg = emc_readl(EMC_CFG_2) & (~EMC_CFG_2_MODE_MASK);
	reg |= ((dram_type == DRAM_TYPE_LPDDR2) ? EMC_CFG_2_PD_MODE :
		EMC_CFG_2_SREF_MODE) << EMC_CFG_2_MODE_SHIFT;
	emc_writel(reg, EMC_CFG_2);
	return 0;
}

static int __devinit tegra11_emc_probe(struct platform_device *pdev)
{
	struct tegra11_emc_pdata *pdata;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing register base\n");
		return -ENOMEM;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -ENODATA;
	}

	return init_emc_table(pdata->tables, pdata->num_tables);
}

static struct platform_driver tegra11_emc_driver = {
	.driver         = {
		.name   = "tegra-emc",
		.owner  = THIS_MODULE,
	},
	.probe          = tegra11_emc_probe,
};

int __init tegra11_emc_init(void)
{
	return platform_driver_register(&tegra11_emc_driver);
}

void tegra_emc_timing_invalidate(void)
{
	emc_timing = NULL;
}

void tegra_emc_dram_type_init(struct clk *c)
{
	emc = c;

	dram_type = (emc_readl(EMC_FBIO_CFG5) &
		     EMC_CFG5_TYPE_MASK) >> EMC_CFG5_TYPE_SHIFT;

	dram_dev_num = (mc_readl(MC_EMEM_ADR_CFG) & 0x1) + 1; /* 2 dev max */
}

int tegra_emc_get_dram_type(void)
{
	return dram_type;
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *emc_debugfs_root;

static int emc_stats_show(struct seq_file *s, void *data)
{
	int i;

	emc_last_stats_update(TEGRA_EMC_TABLE_MAX_SIZE);

	seq_printf(s, "%-10s %-10s \n", "rate kHz", "time");
	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_clk_sel[i].input == NULL)
			continue;	/* invalid entry */

		seq_printf(s, "%-10lu %-10llu \n", tegra_emc_table[i].rate,
			   cputime64_to_clock_t(emc_stats.time_at_clock[i]));
	}
	seq_printf(s, "%-15s %llu\n", "transitions:",
		   emc_stats.clkchange_count);
	seq_printf(s, "%-15s %llu\n", "time-stamp:",
		   cputime64_to_clock_t(emc_stats.last_update));

	return 0;
}

static int emc_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, emc_stats_show, inode->i_private);
}

static const struct file_operations emc_stats_fops = {
	.open		= emc_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int efficiency_get(void *data, u64 *val)
{
	*val = tegra_emc_bw_efficiency;
	return 0;
}
static int efficiency_set(void *data, u64 val)
{
	tegra_emc_bw_efficiency = (val > 100) ? 100 : val;
	if (emc)
		tegra_clk_shared_bus_update(emc);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(efficiency_fops, efficiency_get,
			efficiency_set, "%llu\n");

static int __init tegra_emc_debug_init(void)
{
	if (!tegra_emc_table)
		return 0;

	emc_debugfs_root = debugfs_create_dir("tegra_emc", NULL);
	if (!emc_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file(
		"stats", S_IRUGO, emc_debugfs_root, NULL, &emc_stats_fops))
		goto err_out;

	if (!debugfs_create_u32("clkchange_delay", S_IRUGO | S_IWUSR,
		emc_debugfs_root, (u32 *)&clkchange_delay))
		goto err_out;

	if (!debugfs_create_file("efficiency", S_IRUGO | S_IWUSR,
				 emc_debugfs_root, NULL, &efficiency_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(emc_debugfs_root);
	return -ENOMEM;
}

late_initcall(tegra_emc_debug_init);
#endif
