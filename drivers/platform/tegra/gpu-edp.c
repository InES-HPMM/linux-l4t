/*
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra_ppm.h>

#include <linux/platform/tegra/dvfs.h>
#include <linux/platform/tegra/clock.h>

struct gpu_edp_platform_data {
	const char *name;
	char *cdev_name;
	char *cap_clk_name;
	char *clk_name;
	int freq_step;
	int imax;
};

struct gpu_edp {
	struct tegra_ppm *ppm;
	struct mutex edp_lock;
	struct clk *cap_clk;
	struct thermal_cooling_device *cdev;

	int imax;
	int temperature_now;
};

#define C_TO_K(c) (c+273)
#define K_TO_C(k) (k-273)
#define MELT_SILICON_K 1687 /* melting point of silicon */

static int edp_get_cdev_max_state(struct thermal_cooling_device *cdev,
				       unsigned long *max_state)
{
	*max_state = MELT_SILICON_K;
	return 0;
}

static int edp_get_cdev_cur_state(struct thermal_cooling_device *cdev,
				       unsigned long *cur_state)
{
	struct gpu_edp *ctx = cdev->devdata;
	*cur_state = C_TO_K(ctx->temperature_now);
	return 0;
}

static int edp_set_cdev_state(struct thermal_cooling_device *cdev,
				   unsigned long cur_state)
{
	unsigned clk_rate;
	struct gpu_edp *ctx = cdev->devdata;

	if (cur_state == 0)
		return 0;

	mutex_lock(&ctx->edp_lock);

	BUG_ON(cur_state >= MELT_SILICON_K);

	ctx->temperature_now = K_TO_C(cur_state);
	clk_rate = tegra_ppm_get_maxf(ctx->ppm, ctx->imax,
				      TEGRA_PPM_UNITS_MILLIAMPS,
				      ctx->temperature_now, 1);

	clk_set_rate(ctx->cap_clk, clk_rate * 1000);

	mutex_unlock(&ctx->edp_lock);

	return 0;
}

/* cooling devices using these ops interpret the "cur_state" as
 * a temperature in Kelvin
 */
static struct thermal_cooling_device_ops edp_cooling_ops = {
	.get_max_state = edp_get_cdev_max_state,
	.get_cur_state = edp_get_cdev_cur_state,
	.set_cur_state = edp_set_cdev_state,
};

static int __init tegra_gpu_edp_parse_dt(struct platform_device *pdev,
					 struct gpu_edp_platform_data *pdata)
{

	struct device_node *np = pdev->dev.of_node;

	if (WARN(of_property_read_u32(np, "nvidia,freq_step",
				      &pdata->freq_step),
		 "missing required parameter: nvidia,freq_step\n"))
		return -ENODATA;

	if (WARN(of_property_read_u32(np, "nvidia,edp_limit",
				      &pdata->imax),
		 "missing required parameter: nvidia,edp_limit\n"))
		return -ENODATA;

	if (WARN(of_property_read_string(np, "nvidia,edp_cap_clk",
					 (const char **)&pdata->cap_clk_name),
		 "missing required parameter: nvidia,edp_cap_clk\n"))
		return -ENODATA;

	if (WARN(of_property_read_string(np, "nvidia,edp_clk",
				      (const char **)&pdata->clk_name),
		 "missing required parameter: nvidia,edp_clk\n"))
		return -ENODATA;

	pdata->cdev_name = "gpu_edp";
	pdata->name = dev_name(&pdev->dev);

	return 0;
}

static int __init tegra_gpu_edp_probe(struct platform_device *pdev)
{
	struct clk *gpu_clk;
	struct fv_relation *fv = NULL;
	struct tegra_ppm_params *params;
	unsigned iddq_ma;
	struct gpu_edp_platform_data pdata;
	struct gpu_edp *ctx;
	int ret;

	if (WARN(!pdev || !pdev->dev.of_node,
		 "DT node required but absent\n"))
		return -EINVAL;

	ret = tegra_gpu_edp_parse_dt(pdev, &pdata);
	if (WARN(ret, "GPU EDP management initialization failed\n"))
		return ret;

	params = of_read_tegra_ppm_params(pdev->dev.of_node);
	if (WARN(IS_ERR_OR_NULL(params),
		 "GPU EDP management initialization failed\n"))
		return PTR_ERR(params);

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (WARN(!ctx,
		 "Failed GPU EDP mgmt init. Allocation failure\n")) {
		ret = -ENOMEM;
		goto out;
	}

	gpu_clk = tegra_get_clock_by_name(pdata.clk_name);
	fv = fv_relation_create(gpu_clk, pdata.freq_step, 150,
				      tegra_dvfs_predict_peak_millivolts);
	if (WARN(IS_ERR_OR_NULL(fv),
		 "Failed GPU EDP mgmt init. freq/volt data unavailable\n")) {
		ret = PTR_ERR(fv);
		fv = NULL;
		goto out;
	}

	iddq_ma = tegra_get_gpu_iddq_value();
	pr_debug("%s: %s IDDQ value %d\n",
		 __func__, pdata.name, iddq_ma);


	ctx->cap_clk = tegra_get_clock_by_name(pdata.cap_clk_name);
	if (!ctx->cap_clk)
		pr_err("%s: cannot get clock:%s\n",
		       __func__, pdata.cap_clk_name);

	ctx->ppm = tegra_ppm_create(pdata.name, fv, params, iddq_ma, NULL);
	if (WARN(IS_ERR_OR_NULL(ctx->ppm),
		 "Failed GPU EDP mgmt init. Couldn't create power model\n")) {
		ret = PTR_ERR(ctx->ppm);
		ctx->ppm = NULL;
		goto out;

	}

	ctx->temperature_now = -273; /* HACK */
	ctx->imax = pdata.imax;

	mutex_init(&ctx->edp_lock);

	ctx->cdev = thermal_of_cooling_device_register(
		pdev->dev.of_node, pdata.cdev_name, ctx, &edp_cooling_ops);
	if (IS_ERR_OR_NULL(ctx->cdev)) {
		pr_err("Failed to register '%s' cooling device\n",
			pdata.cdev_name);
		ctx->cdev = NULL;
		ret = PTR_ERR(ctx->cdev);
		goto out;
	}

out:
	if (ret) {
		if (ctx) {
			thermal_cooling_device_unregister(ctx->cdev);
			tegra_ppm_destroy(ctx->ppm, NULL, NULL);
		}
		fv_relation_destroy(fv);
		kfree(params);
	}
	return ret;
}

static struct of_device_id tegra_gpu_edp_of_match[] = {
	{ .compatible = "nvidia,tegra124-gpu-edp-capping" },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_gpu_edp_of_match);

static struct platform_driver tegra_gpu_edp_driver = {
	.driver = {
		.name = "tegra_gpu_edp",
		.owner = THIS_MODULE,
		.of_match_table = tegra_gpu_edp_of_match,
	},
};

static int __init tegra_gpu_edp_driver_init(void)
{
	return platform_driver_probe(&tegra_gpu_edp_driver,
				     tegra_gpu_edp_probe);
}
module_init(tegra_gpu_edp_driver_init);

MODULE_AUTHOR("NVIDIA Corp.");
MODULE_DESCRIPTION("Tegra GPU EDP management");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tegra-gpu-edp");
