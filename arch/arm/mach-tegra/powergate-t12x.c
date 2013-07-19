/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <asm/atomic.h>

#include <mach/powergate.h>

#include "powergate-priv.h"
#include "powergate-ops-txx.h"
#include "powergate-ops-t1xx.h"

enum mc_client {
	MC_CLIENT_AFI		= 0,
	MC_CLIENT_DC		= 2,
	MC_CLIENT_DCB		= 3,
	MC_CLIENT_ISP		= 8,
	MC_CLIENT_MSENC		= 11,
	MC_CLIENT_SATA		= 15,
	MC_CLIENT_VDE		= 16,
	MC_CLIENT_VI		= 17,
	MC_CLIENT_VIC		= 18,
	MC_CLIENT_XUSB_HOST	= 19,
	MC_CLIENT_XUSB_DEV	= 20,
	MC_CLIENT_GPU		= 34,
	MC_CLIENT_LAST		= -1,
};

struct tegra12x_powergate_mc_client_info {
	enum mc_client hot_reset_clients[MAX_HOTRESET_CLIENT_NUM];
};

static struct tegra12x_powergate_mc_client_info tegra12x_pg_mc_info[] = {
	[TEGRA_POWERGATE_GPU] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_GPU,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_VDEC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_VDE,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_MPE] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_MSENC,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_VENC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_ISP,
			[1] = MC_CLIENT_VI,
			[2] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_DISA] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_DC,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_DISB] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_DCB,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_XUSBA] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_XUSBB] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_XUSB_DEV,
			[1] = MC_CLIENT_LAST
		},
	},
	[TEGRA_POWERGATE_XUSBC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_XUSB_HOST,
			[1] = MC_CLIENT_LAST,
		},
	},
#ifdef CONFIG_ARCH_TEGRA_HAS_PCIE
	[TEGRA_POWERGATE_PCIE] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_AFI,
			[1] = MC_CLIENT_LAST,
		},
	},
#endif
#ifdef CONFIG_ARCH_TEGRA_HAS_SATA
	[TEGRA_POWERGATE_SATA] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_SATA,
			[1] = MC_CLIENT_LAST,
		},
	},
#endif
	[TEGRA_POWERGATE_SOR] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_LAST,
		},
	},
#ifdef CONFIG_ARCH_TEGRA_VIC
	[TEGRA_POWERGATE_VIC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_VIC,
			[1] = MC_CLIENT_LAST,
		},
	},
#endif
};

static struct powergate_partition_info tegra12x_powergate_partition_info[] = {
	[TEGRA_POWERGATE_GPU] = {
		.name = "gpu",
		.clk_info = {
			[0] = { .clk_name = "gpu", .clk_type = CLK_AND_RST },
		},
	},
	[TEGRA_POWERGATE_VDEC] = {
		.name = "vde",
		.clk_info = {
			[0] = { .clk_name = "vde", .clk_type = CLK_AND_RST },
		},
	},
	[TEGRA_POWERGATE_MPE] = {
		.name = "mpe",
		.clk_info = {
			[0] = { .clk_name = "msenc.cbus", .clk_type = CLK_AND_RST },
		},
	},
	[TEGRA_POWERGATE_VENC] = {
		.name = "ve",
		.clk_info = {
			[0] = { .clk_name = "isp", .clk_type = CLK_AND_RST },
			[0] = { .clk_name = "ispb", .clk_type = CLK_AND_RST },
			[1] = { .clk_name = "vi", .clk_type = CLK_AND_RST },
			[2] = { .clk_name = "csi", .clk_type = CLK_AND_RST },
		},
	},
	[TEGRA_POWERGATE_DISA] = {
		.name = "disa",
		.clk_info = {
			[0] = { .clk_name = "disp1", .clk_type = CLK_AND_RST },
		},
	},
	[TEGRA_POWERGATE_DISB] = {
		.name = "disb",
		.clk_info = {
			[0] = { .clk_name = "disp2", .clk_type = CLK_AND_RST },
		},
	},
	[TEGRA_POWERGATE_XUSBA] = {
		.name = "xusba",
		.clk_info = {
			[0] = { .clk_name = "xusb_ss", .clk_type = CLK_AND_RST },
		},
	},
	[TEGRA_POWERGATE_XUSBB] = {
		.name = "xusbb",
		.clk_info = {
			[0] = { .clk_name = "xusb_dev", .clk_type = CLK_AND_RST },
		},
	},
	[TEGRA_POWERGATE_XUSBC] = {
		.name = "xusbc",
		.clk_info = {
			[0] = { .clk_name = "xusb_host", .clk_type = CLK_AND_RST },
		},
	},
#ifdef CONFIG_ARCH_TEGRA_HAS_PCIE
	[TEGRA_POWERGATE_PCIE] = {
		.name = "pcie",
		.clk_info = {
			[0] = { .clk_name = "afi", .clk_type = CLK_AND_RST },
			[1] = { .clk_name = "pcie", .clk_type = CLK_AND_RST },
			[2] = { .clk_name = "cml0", .clk_type = CLK_ONLY },
			[3] = { .clk_name = "pciex", .clk_type = RST_ONLY },
		},
	},
#endif
#ifdef CONFIG_ARCH_TEGRA_HAS_SATA
	[TEGRA_POWERGATE_SATA] = {
		.name = "sata",
		.clk_info = {
			[0] = { .clk_name = "sata", .clk_type = CLK_AND_RST },
			[1] = { .clk_name = "sata_oob", .clk_type = CLK_AND_RST },
			[2] = { .clk_name = "cml1", .clk_type = CLK_ONLY },
			[3] = { .clk_name = "sata_cold", .clk_type = RST_ONLY },
		},
	},
#endif
	[TEGRA_POWERGATE_SOR] = {
		.name = "sor",
		.clk_info = {
			[0] = { .clk_name = "sor0", .clk_type = CLK_AND_RST },
			[1] = { .clk_name = "dsia", .clk_type = CLK_AND_RST },
			[2] = { .clk_name = "dsib", .clk_type = CLK_AND_RST },
			[3] = { .clk_name = "hdmi", .clk_type = CLK_AND_RST },
			[4] = { .clk_name = "mipi-cal", .clk_type = CLK_AND_RST },
			[5] = { .clk_name = "dpaux", .clk_type = CLK_ONLY },
		},
	},
#ifdef CONFIG_ARCH_TEGRA_VIC
	[TEGRA_POWERGATE_VIC] = {
		.name = "vic",
		.clk_info = {
			[0] = { .clk_name = "vic03.cbus", .clk_type = CLK_AND_RST },
		},
	},
#endif
};

#define MC_CLIENT_HOTRESET_CTRL		0x200
#define MC_CLIENT_HOTRESET_STAT		0x204
#define MC_CLIENT_HOTRESET_CTRL_1	0x970
#define MC_CLIENT_HOTRESET_STAT_1	0x974

#define PMC_GPU_RG_CNTRL_0		0x2d4

static DEFINE_SPINLOCK(tegra12x_powergate_lock);

#define HOTRESET_READ_COUNT	5
static bool tegra12x_stable_hotreset_check(u32 stat_reg, u32 *stat)
{
	int i;
	u32 cur_stat;
	u32 prv_stat;
	unsigned long flags;

	spin_lock_irqsave(&tegra12x_powergate_lock, flags);
	prv_stat = mc_read(stat_reg);
	for (i = 0; i < HOTRESET_READ_COUNT; i++) {
		cur_stat = mc_read(stat_reg);
		if (cur_stat != prv_stat) {
			spin_unlock_irqrestore(&tegra12x_powergate_lock, flags);
			return false;
		}
	}
	*stat = cur_stat;
	spin_unlock_irqrestore(&tegra12x_powergate_lock, flags);
	return true;
}

int tegra12x_powergate_mc_enable(int id)
{
	return 0;
}

int tegra12x_powergate_mc_disable(int id)
{
	return 0;
}

int tegra12x_powergate_mc_flush(int id)
{
	u32 idx, rst_ctrl, rst_stat;
	u32 rst_ctrl_reg, rst_stat_reg;
	enum mc_client mcClientBit;
	unsigned long flags;
	bool ret;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			tegra12x_pg_mc_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		if (mcClientBit < 32) {
			rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL;
			rst_stat_reg = MC_CLIENT_HOTRESET_STAT;
		} else {
			mcClientBit %= 32;
			rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL_1;
			rst_stat_reg = MC_CLIENT_HOTRESET_STAT_1;
		}

		spin_lock_irqsave(&tegra12x_powergate_lock, flags);

		rst_ctrl = mc_read(rst_ctrl_reg);
		rst_ctrl |= (1 << mcClientBit);
		mc_write(rst_ctrl, rst_ctrl_reg);

		spin_unlock_irqrestore(&tegra12x_powergate_lock, flags);

		do {
			udelay(10);
			rst_stat = 0;
			ret = tegra12x_stable_hotreset_check(rst_stat_reg, &rst_stat);
			if (!ret)
				continue;
		} while (!(rst_stat & (1 << mcClientBit)));
	}

	return 0;
}

int tegra12x_powergate_mc_flush_done(int id)
{
	u32 idx, rst_ctrl, rst_ctrl_reg;
	enum mc_client mcClientBit;
	unsigned long flags;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			tegra12x_pg_mc_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		if (mcClientBit < 32)
			rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL;
		else {
			mcClientBit %= 32;
			rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL_1;
		}

		spin_lock_irqsave(&tegra12x_powergate_lock, flags);

		rst_ctrl = mc_read(rst_ctrl_reg);
		rst_ctrl &= ~(1 << mcClientBit);
		mc_write(rst_ctrl, rst_ctrl_reg);

		spin_unlock_irqrestore(&tegra12x_powergate_lock, flags);
	}

	wmb();

	return 0;
}

static int tegra12x_gpu_powergate(int id, struct powergate_partition_info *pg_info)
{
	int ret;

	/* If first clk_ptr is null, fill clk info for the partition */
	if (!pg_info->clk_info[0].clk_ptr)
		get_clk_info(pg_info);

	ret = partition_clk_enable(pg_info);
	if (ret)
		WARN(1, "Couldn't enable clock");

	udelay(10);

	tegra_powergate_mc_flush(id);

	udelay(10);

	/* enable clamp */
	pmc_write(0x1, PMC_GPU_RG_CNTRL_0);

	udelay(10);

	powergate_partition_assert_reset(pg_info);

	udelay(10);

	/* Powergating is done only if refcnt of all clks is 0 */
	partition_clk_disable(pg_info);

	udelay(10);

	/* TBD: call regulator api to turn off VDD_GPU */
	if (0)
		goto err_power_off;

	return 0;

err_power_off:
	WARN(1, "Could not Railgate Partition %d", id);
	return ret;
}

int hack_tegra12x_gpu_unpowergate(void)
{
	int ret;
	int id = TEGRA_POWERGATE_GPU;
	struct powergate_partition_info *pg_info =
					&tegra12x_powergate_partition_info[id];
	struct regulator * gpu_reg;

	if (!tegra_platform_is_silicon())
		return 0;

	printk("%s(): start\n", __func__);
	gpu_reg = regulator_get(NULL, "vdd_gpu");
	if (IS_ERR_OR_NULL(gpu_reg))
		BUG_ON(1);

	regulator_set_voltage(gpu_reg, 1000000, 1000000);
	ret = regulator_enable(gpu_reg);
	if (ret)
		BUG_ON(1);

	/* TBD: call regulator api to turn on VDD_GPU */
	if (0)
               goto err_power;

	/* If first clk_ptr is null, fill clk info for the partition */
	if (!pg_info->clk_info[0].clk_ptr)
		get_clk_info(pg_info);

	/* Un-Powergating fails if all clks are not enabled */
	ret = partition_clk_enable(pg_info);
	if (ret)
		goto err_clk_on;

	udelay(10);

	/* disable clamp */
	pmc_write(0, PMC_GPU_RG_CNTRL_0);

	udelay(10);

	powergate_partition_assert_reset(pg_info);
	udelay(10);
	powergate_partition_deassert_reset(pg_info);

	udelay(10);

	tegra_powergate_mc_flush_done(id);

	udelay(10);

	/* Disable all clks enabled earlier. Drivers should enable clks */
	// partition_clk_disable(pg_info);

	printk("%s(): end\n", __func__);
	return 0;

err_clk_on:
	powergate_module(id);
err_power:
	WARN(1, "Could not Un-Railgate %d", id);
	return ret;
}
EXPORT_SYMBOL(hack_tegra12x_gpu_unpowergate);

static int tegra12x_gpu_unpowergate(int id, struct powergate_partition_info *pg_info)
{
	int ret;

	/* TBD: call regulator api to turn on VDD_GPU */
	if (0)
		goto err_power;

	/* If first clk_ptr is null, fill clk info for the partition */
	if (!pg_info->clk_info[0].clk_ptr)
		get_clk_info(pg_info);

	/* Un-Powergating fails if all clks are not enabled */
	ret = partition_clk_enable(pg_info);
	if (ret)
		goto err_clk_on;

	udelay(10);

	/* disable clamp */
	pmc_write(0, PMC_GPU_RG_CNTRL_0);

	udelay(10);

	powergate_partition_deassert_reset(pg_info);

	udelay(10);

	tegra_powergate_mc_flush_done(id);

	udelay(10);

	/* Disable all clks enabled earlier. Drivers should enable clks */
	partition_clk_disable(pg_info);

	return 0;

err_clk_on:
	powergate_module(id);
err_power:
	WARN(1, "Could not Un-Railgate %d", id);
	return ret;
}

static atomic_t ref_count_disp = ATOMIC_INIT(0);

#define CHECK_RET(x)			\
	do {				\
		ret = (x);		\
		if (ret != 0)		\
			return ret;	\
	} while (0)


static inline int tegra12x_powergate(int id)
{
	if (tegra_powergate_is_powered(id))
		return tegra1xx_powergate(id,
			&tegra12x_powergate_partition_info[id]);
	return 0;
}

static inline int tegra12x_unpowergate(int id)
{
	if (!tegra_powergate_is_powered(id))
		return tegra1xx_unpowergate(id,
			&tegra12x_powergate_partition_info[id]);
	return 0;
}


static int tegra12x_disp_powergate(int id)
{
	int ret = 0;
	int ref_count = atomic_read(&ref_count_disp);

	if (!TEGRA_IS_DISP_POWERGATE_ID(id))
		return -EINVAL;

	if (id == TEGRA_POWERGATE_DISA) {
		ref_count = atomic_dec_return(&ref_count_disp);
		WARN(ref_count < 0, "DISP ref count underflow");
	} else
		CHECK_RET(tegra12x_powergate(TEGRA_POWERGATE_DISB));

	if (ref_count <= 0 &&
		!tegra_powergate_is_powered(TEGRA_POWERGATE_DISB)) {
		CHECK_RET(tegra12x_powergate(TEGRA_POWERGATE_SOR));
		CHECK_RET(tegra12x_powergate(TEGRA_POWERGATE_DISA));
	}
	return ret;
}

static int tegra12x_disp_unpowergate(int id)
{
	int ret;

	if (!TEGRA_IS_DISP_POWERGATE_ID(id))
		return -EINVAL;

	/* always unpowergate dispA and SOR partition */
	CHECK_RET(tegra12x_unpowergate(TEGRA_POWERGATE_DISA));
	CHECK_RET(tegra12x_unpowergate(TEGRA_POWERGATE_SOR));

	if (id == TEGRA_POWERGATE_DISA)
		WARN_ONCE(atomic_inc_return(&ref_count_disp) > 1,
			"disp ref count overflow");
	else
		ret = tegra12x_unpowergate(TEGRA_POWERGATE_DISB);

	return ret;
}


int tegra12x_powergate_partition(int id)
{
	int ret;

	if (TEGRA_IS_GPU_POWERGATE_ID(id)) {
		ret = tegra12x_gpu_powergate(id,
			&tegra12x_powergate_partition_info[id]);
	} else if (TEGRA_IS_DISP_POWERGATE_ID(id))
		ret = tegra12x_disp_powergate(id);
	else {
		/* call common power-gate API for t1xx */
		ret = tegra1xx_powergate(id,
			&tegra12x_powergate_partition_info[id]);
	}

	return ret;
}

int tegra12x_unpowergate_partition(int id)
{
	int ret;

	if (TEGRA_IS_GPU_POWERGATE_ID(id)) {
		ret = tegra12x_gpu_unpowergate(id,
			&tegra12x_powergate_partition_info[id]);
	} else if (TEGRA_IS_DISP_POWERGATE_ID(id))
		ret = tegra12x_disp_unpowergate(id);
	else {
		ret = tegra1xx_unpowergate(id,
			&tegra12x_powergate_partition_info[id]);
	}

	return ret;
}

int tegra12x_powergate_partition_with_clk_off(int id)
{
	BUG_ON(TEGRA_IS_GPU_POWERGATE_ID(id));

	return tegraxx_powergate_partition_with_clk_off(id,
		&tegra12x_powergate_partition_info[id]);
}

int tegra12x_unpowergate_partition_with_clk_on(int id)
{
	BUG_ON(TEGRA_IS_GPU_POWERGATE_ID(id));

	return tegraxx_unpowergate_partition_with_clk_on(id,
		&tegra12x_powergate_partition_info[id]);
}

const char *tegra12x_get_powergate_domain_name(int id)
{
	return tegra12x_powergate_partition_info[id].name;
}

spinlock_t *tegra12x_get_powergate_lock(void)
{
	return &tegra12x_powergate_lock;
}

bool tegra12x_powergate_skip(int id)
{
	switch (id) {
	case TEGRA_POWERGATE_VDEC:
	case TEGRA_POWERGATE_VENC:
	case TEGRA_POWERGATE_XUSBA:
	case TEGRA_POWERGATE_XUSBB:
	case TEGRA_POWERGATE_XUSBC:
#ifdef CONFIG_ARCH_TEGRA_HAS_SATA
	case TEGRA_POWERGATE_SATA:
#endif
		return true;

	default:
		return false;
	}
}

static struct powergate_ops tegra12x_powergate_ops = {
	.soc_name = "tegra12x",

	.num_powerdomains = TEGRA_NUM_POWERGATE,

	.get_powergate_lock = tegra12x_get_powergate_lock,
	.get_powergate_domain_name = tegra12x_get_powergate_domain_name,

	.powergate_partition = tegra12x_powergate_partition,
	.unpowergate_partition = tegra12x_unpowergate_partition,

	.powergate_partition_with_clk_off =  tegra12x_powergate_partition_with_clk_off,
	.unpowergate_partition_with_clk_on = tegra12x_unpowergate_partition_with_clk_on,

	.powergate_mc_enable = tegra12x_powergate_mc_enable,
	.powergate_mc_disable = tegra12x_powergate_mc_disable,

	.powergate_mc_flush = tegra12x_powergate_mc_flush,
	.powergate_mc_flush_done = tegra12x_powergate_mc_flush_done,

	.powergate_skip = tegra12x_powergate_skip,
};

struct powergate_ops *tegra12x_powergate_init_chip_support(void)
{
	return &tegra12x_powergate_ops;
}
