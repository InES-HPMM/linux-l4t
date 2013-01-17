/*
 * drivers/video/tegra/camera/camera_clk.c
 *
 * Copyright (C) 2013 Nvidia Corp
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

#include "camera_clk.h"

int tegra_camera_enable_clk(struct tegra_camera *camera)
{
	int i;
	for (i = 0; i < CAMERA_CLK_MAX; i++)
		if (camera->clock[i].on)
			clk_prepare_enable(camera->clock[i].clk);

	return 0;
}

int tegra_camera_disable_clk(struct tegra_camera *camera)
{
	int i;
	for (i = CAMERA_CLK_MAX; i > 0; i--)
		if (camera->clock[i-1].on)
			clk_disable_unprepare(camera->clock[i-1].clk);

	return 0;
}

int tegra_camera_init_clk(struct tegra_camera *camera,
		struct clock_data *clock_init)
{
	int i;
	for (i = 0; i < CAMERA_CLK_MAX; i++)
		camera->clock[clock_init[i].index].on = clock_init[i].init;
	return 0;
}

int tegra_camera_clk_set_rate(struct tegra_camera *camera)
{
	struct clk *clk, *clk_parent;
	struct tegra_camera_clk_info *info = &camera->info;
	unsigned long parent_rate, parent_div_rate, parent_div_rate_pre;

	if (!info) {
		dev_err(camera->dev,
				"%s: no clock info %d\n",
				__func__, info->id);
		return -EINVAL;
	}

	if (info->id != TEGRA_CAMERA_MODULE_VI &&
		info->id != TEGRA_CAMERA_MODULE_EMC) {
		dev_err(camera->dev,
				"%s: set rate only aplies to vi module %d\n",
				__func__, info->id);
		return -EINVAL;
	}

	switch (info->clk_id) {
	case TEGRA_CAMERA_VI_CLK:
#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
		if (info->flag == TEGRA_CAMERA_ENABLE_PD2VI_CLK) {
			if (camera->clock[CAMERA_PLL_D2_CLK].clk) {
				clk_prepare_enable(
					camera->clock[CAMERA_PLL_D2_CLK].clk);
				clk = camera->clock[CAMERA_PLL_D2_CLK].clk;
				camera->clock[CAMERA_PLL_D2_CLK].on = true;
			} else {
				/*
				 * PowerSaving: enable pll_d2 for camera only
				 * when required. pll_d2 will be disabled when
				 * camera will be released.
				 */
				return -EINVAL;
			}
		} else
#endif
		{
			clk = camera->clock[CAMERA_VI_CLK].clk;
		}
		break;
	case TEGRA_CAMERA_VI_SENSOR_CLK:
		clk = camera->clock[CAMERA_VI_SENSOR_CLK].clk;
		break;
	case TEGRA_CAMERA_EMC_CLK:
		clk = camera->clock[CAMERA_EMC_CLK].clk;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		{
			/*
			 * User space assumes that HW emc controller is 4
			 * byte-wide DDR controller.
			 * Emc bandwidth needs to be calcaluated using input emc
			 * freq first, and then real emc freq will
			 * be calculated using tegra_emc API.
			 * tegra_emc_bw_to_freq_req takes HW difference
			 * into consideration.
			 * bw param in tegra_emc_bw_to_freq_req() is in KHz.
			 */
			unsigned long bw = (info->rate * 8) >> 10;
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
			int ret = 0;
#endif
			dev_dbg(camera->dev, "%s: emc_clk rate=%lu\n",
				__func__, info->rate);
			clk_set_rate(clk,
					tegra_emc_bw_to_freq_req(bw) << 10);
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
			/*
			 * There is no way to figure out what latency
			 * can be tolerated in VI without reading VI
			 * registers for now. 3 usec is minimum time
			 * to switch PLL source. Let's put 4 usec as
			 * latency for now.
			 */
			ret = tegra_isomgr_reserve(camera->isomgr_handle,
					bw,	/* KB/sec */
					4);	/* usec */
			if (!ret)
				return -ENOMEM;

			ret = tegra_isomgr_realize(camera->isomgr_handle);
			if (!ret)
				return -ENOMEM;
#endif
		}
#endif
		goto set_rate_end;
	default:
		dev_err(camera->dev,
				"%s: invalid clk id for set rate %d\n",
				__func__, info->clk_id);
		return -EINVAL;
	}

	clk_parent = clk_get_parent(clk);
	parent_rate = clk_get_rate(clk_parent);
	dev_dbg(camera->dev, "%s: clk_id=%d, parent_rate=%lu, clk_rate=%lu\n",
			__func__, info->clk_id, parent_rate, info->rate);
	parent_div_rate = parent_rate;
	parent_div_rate_pre = parent_rate;

	if (info->flag != TEGRA_CAMERA_ENABLE_PD2VI_CLK) {
		/*
		 * The requested clock rate from user space should be respected.
		 * This loop is to search the clock rate that is higher than
		 * requested clock.
		 * However, for camera pattern generator, since we share the
		 * clk source with display, we would not want to change the
		 * display clock.
		 */
		while (parent_div_rate >= info->rate) {
			parent_div_rate_pre = parent_div_rate;
			parent_div_rate = clk_round_rate(clk,
				parent_div_rate-1);
		}
	}
	dev_dbg(camera->dev, "%s: set_rate=%lu",
			__func__, parent_div_rate_pre);

	clk_set_rate(clk, parent_div_rate_pre);

	if (info->clk_id == TEGRA_CAMERA_VI_CLK) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		{
			u32 val;
			void __iomem *apb_misc =
				IO_ADDRESS(TEGRA_APB_MISC_BASE);
			val = readl(apb_misc + 0x42c);
			writel(val | 0x1, apb_misc + 0x42c);
		}
#endif
		if (info->flag == TEGRA_CAMERA_ENABLE_PD2VI_CLK) {
#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
			tegra_clk_cfg_ex(camera->clock[CAMERA_PLL_D2_CLK].clk,
						TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(camera->clock[CAMERA_PLL_D2_CLK].clk,
						TEGRA_CLK_PLLD_DSI_OUT_ENB, 1);
#else
		/*
		 * bit 25: 0 = pd2vi_Clk, 1 = vi_sensor_clk
		 * bit 24: 0 = internal clock, 1 = external clock(pd2vi_clk)
		 */
			tegra_clk_cfg_ex(clk, TEGRA_CLK_VI_INP_SEL, 2);
#endif
		}
#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
		else {
			tegra_clk_cfg_ex(camera->clock[CAMERA_PLL_D2_CLK].clk,
						TEGRA_CLK_PLLD_CSI_OUT_ENB, 0);
			tegra_clk_cfg_ex(camera->clock[CAMERA_PLL_D2_CLK].clk,
						TEGRA_CLK_PLLD_DSI_OUT_ENB, 0);
		}
#endif
	}

set_rate_end:
	info->rate = clk_get_rate(clk);
	dev_dbg(camera->dev, "%s: get_rate=%lu",
			__func__, info->rate);
	return 0;
}
