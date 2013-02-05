/*
 * drivers/media/video/tegra/tegra_camera.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2012 Nvidia Corp
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

#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>

#include <mach/iomap.h>
#include <mach/clk.h>
#include <mach/powergate.h>
#include <mach/mc.h>

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
#include <mach/isomgr.h>
#endif

#include <media/tegra_camera.h>

/* Eventually this should handle all clock and reset calls for the isp, vi,
 * vi_sensor, and csi modules, replacing nvrm and nvos completely for camera
 */
#define TEGRA_CAMERA_NAME "tegra_camera"

struct tegra_camera_dev {
	struct device *dev;
	struct miscdevice misc_dev;
	struct clk *isp_clk;
	struct clk *vi_clk;
	struct clk *vi_sensor_clk;
	struct clk *csus_clk;
	struct clk *csi_clk;
	struct clk *emc_clk;
#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
	struct clk *pll_d2_clk;
	struct clk *cilab_clk;
	struct clk *cilcd_clk;
	struct clk *cile_clk;
#endif
#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
	struct clk *vim2_clk;
#endif
	struct regulator *reg;
	struct tegra_camera_clk_info info;
	struct mutex tegra_camera_lock;
	atomic_t in_use;
	int power_on;
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	tegra_isomgr_handle isomgr_handle;
#endif
};

struct tegra_camera_block {
	int (*enable) (struct tegra_camera_dev *dev);
	int (*disable) (struct tegra_camera_dev *dev);
	bool is_enabled;
};

static int tegra_camera_enable_clk(struct tegra_camera_dev *dev)
{
	clk_prepare_enable(dev->vi_clk);
	clk_prepare_enable(dev->vi_sensor_clk);
	clk_prepare_enable(dev->csus_clk);
	clk_prepare_enable(dev->isp_clk);
	clk_prepare_enable(dev->csi_clk);

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	clk_prepare_enable(dev->cilab_clk);
	clk_prepare_enable(dev->cilcd_clk);
	clk_prepare_enable(dev->cile_clk);
#endif
#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
	clk_prepare_enable(dev->vim2_clk);
#endif
	return 0;
}

static int tegra_camera_disable_clk(struct tegra_camera_dev *dev)
{
	clk_disable_unprepare(dev->csi_clk);
	clk_disable_unprepare(dev->isp_clk);
	clk_disable_unprepare(dev->csus_clk);
	clk_disable_unprepare(dev->vi_sensor_clk);
	clk_disable_unprepare(dev->vi_clk);

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	clk_disable_unprepare(dev->cilab_clk);
	clk_disable_unprepare(dev->cilcd_clk);
	clk_disable_unprepare(dev->cile_clk);
	if (tegra_is_clk_enabled(dev->pll_d2_clk))
		clk_disable_unprepare(dev->pll_d2_clk);
#endif
#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
	clk_disable_unprepare(dev->vim2_clk);
#endif
	return 0;
}

static int tegra_camera_enable_emc(struct tegra_camera_dev *dev)
{
	int ret = tegra_emc_disable_eack();
	clk_prepare_enable(dev->emc_clk);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	clk_set_rate(dev->emc_clk, 300000000);
#endif
	return ret;
}

static int tegra_camera_disable_emc(struct tegra_camera_dev *dev)
{
	clk_disable_unprepare(dev->emc_clk);
	return tegra_emc_enable_eack();
}

static int tegra_camera_clk_set_rate(struct tegra_camera_dev *dev)
{
	struct clk *clk, *clk_parent;
	struct tegra_camera_clk_info *info = &dev->info;
	unsigned long parent_rate, parent_div_rate, parent_div_rate_pre;

	if (!info) {
		dev_err(dev->dev,
				"%s: no clock info %d\n",
				__func__, info->id);
		return -EINVAL;
	}

	if (info->id != TEGRA_CAMERA_MODULE_VI &&
		info->id != TEGRA_CAMERA_MODULE_EMC) {
		dev_err(dev->dev,
				"%s: set rate only aplies to vi module %d\n",
				__func__, info->id);
		return -EINVAL;
	}

	switch (info->clk_id) {
	case TEGRA_CAMERA_VI_CLK:
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
		if (info->flag == TEGRA_CAMERA_ENABLE_PD2VI_CLK) {
			if (dev->pll_d2_clk) {
				if (!tegra_is_clk_enabled(dev->pll_d2_clk))
					clk_prepare_enable(dev->pll_d2_clk);
				clk = dev->pll_d2_clk;
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
			clk = dev->vi_clk;
		}
		break;
	case TEGRA_CAMERA_VI_SENSOR_CLK:
		clk = dev->vi_sensor_clk;
		break;
	case TEGRA_CAMERA_EMC_CLK:
		clk = dev->emc_clk;
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
			dev_dbg(dev->dev, "%s: emc_clk rate=%lu\n",
				__func__, info->rate);
			clk_set_rate(dev->emc_clk,
					tegra_emc_bw_to_freq_req(bw) << 10);
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
			/*
			 * There is no way to figure out what latency
			 * can be tolerated in VI without reading VI
			 * registers for now. 3 usec is minimum time
			 * to switch PLL source. Let's put 4 usec as
			 * latency for now.
			 */
			ret = tegra_isomgr_reserve(dev->isomgr_handle,
					bw,	/* KB/sec */
					4);	/* usec */
			if (!ret)
				return -ENOMEM;

			ret = tegra_isomgr_realize(dev->isomgr_handle);
			if (!ret)
				return -ENOMEM;
#endif
		}
#endif
		goto set_rate_end;
	default:
		dev_err(dev->dev,
				"%s: invalid clk id for set rate %d\n",
				__func__, info->clk_id);
		return -EINVAL;
	}

	clk_parent = clk_get_parent(clk);
	parent_rate = clk_get_rate(clk_parent);
	dev_dbg(dev->dev, "%s: clk_id=%d, parent_rate=%lu, clk_rate=%lu\n",
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
			parent_div_rate =
				clk_round_rate(clk, parent_div_rate-1);
		}
	}

	dev_dbg(dev->dev, "%s: set_rate=%lu",
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
			tegra_clk_cfg_ex(dev->pll_d2_clk,
						TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(dev->pll_d2_clk,
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
			tegra_clk_cfg_ex(dev->pll_d2_clk,
						TEGRA_CLK_PLLD_CSI_OUT_ENB, 0);
			tegra_clk_cfg_ex(dev->pll_d2_clk,
						TEGRA_CLK_PLLD_DSI_OUT_ENB, 0);
		}
#endif
	}

set_rate_end:
	info->rate = clk_get_rate(clk);
	dev_dbg(dev->dev, "%s: get_rate=%lu",
			__func__, info->rate);
	return 0;

}

static int tegra_camera_power_on(struct tegra_camera_dev *dev)
{
	int ret = 0;
	dev_dbg(dev->dev, "%s++\n", __func__);

	/* Enable external power */
	if (dev->reg) {
		ret = regulator_enable(dev->reg);
		if (ret) {
			dev_err(dev->dev,
				"%s: enable csi regulator failed.\n",
				__func__);
			return ret;
		}
	}
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Unpowergate VE */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);
	if (ret)
		dev_err(dev->dev,
			"%s: unpowergate failed.\n",
			__func__);
#endif
	dev->power_on = 1;
	return ret;
}

static int tegra_camera_power_off(struct tegra_camera_dev *dev)
{
	int ret = 0;

	dev_dbg(dev->dev, "%s++\n", __func__);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Powergate VE */
	ret = tegra_powergate_partition(TEGRA_POWERGATE_VENC);
	if (ret)
		dev_err(dev->dev,
			"%s: powergate failed.\n",
			__func__);
#endif
	/* Disable external power */
	if (dev->reg) {
		ret = regulator_disable(dev->reg);
		if (ret) {
			dev_err(dev->dev,
				"%s: disable csi regulator failed.\n",
				__func__);
			return ret;
		}
	}
	dev->power_on = 0;
	return ret;
}

static long tegra_camera_ioctl(struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	uint id;
	struct tegra_camera_dev *dev = file->private_data;

	/* first element of arg must be u32 with id of module to talk to */
	if (copy_from_user(&id, (const void __user *)arg, sizeof(uint))) {
		dev_err(dev->dev,
				"%s: Failed to copy arg from user", __func__);
		return -EFAULT;
	}

	if (id >= TEGRA_CAMERA_MODULE_MAX) {
		dev_err(dev->dev,
				"%s: Invalid id to tegra isp ioctl%d\n",
				__func__, id);
		return -EINVAL;
	}

	switch (cmd) {
	/*
	 * Clock enable/disable and reset should be handled in kernel.
	 * In order to support legacy code in user space, we don't remove
	 * these IOCTL.
	 */
	case TEGRA_CAMERA_IOCTL_ENABLE:
	case TEGRA_CAMERA_IOCTL_DISABLE:
	case TEGRA_CAMERA_IOCTL_RESET:
		return 0;
	case TEGRA_CAMERA_IOCTL_CLK_SET_RATE:
	{
		int ret;

		if (copy_from_user(&dev->info, (const void __user *)arg,
				   sizeof(struct tegra_camera_clk_info))) {
			dev_err(dev->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
		}
		ret = tegra_camera_clk_set_rate(dev);
		if (ret)
			return ret;
		if (copy_to_user((void __user *)arg, &dev->info,
				 sizeof(struct tegra_camera_clk_info))) {
			dev_err(dev->dev,
				"%s: Failed to copy arg to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	default:
		dev_err(dev->dev,
				"%s: Unknown tegra_camera ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int tegra_camera_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct tegra_camera_dev *dev = container_of(miscdev,
						struct tegra_camera_dev,
						misc_dev);
	int ret = 0;

	dev_info(dev->dev, "%s\n", __func__);

	if (atomic_xchg(&dev->in_use, 1))
		return -EBUSY;

	file->private_data = dev;

	pm_runtime_get_sync(dev->dev);

	mutex_lock(&dev->tegra_camera_lock);
	/* turn on CSI regulator */
	ret = tegra_camera_power_on(dev);
	if (ret)
		goto power_on_err;
	/* set EMC request */
	ret = tegra_camera_enable_emc(dev);
	if (ret)
		goto enable_emc_err;
	/* enable camera HW clock */
	ret = tegra_camera_enable_clk(dev);
	if (ret)
		goto enable_clk_err;

	mutex_unlock(&dev->tegra_camera_lock);

	return 0;

enable_clk_err:
	tegra_camera_disable_emc(dev);
enable_emc_err:
	tegra_camera_power_off(dev);
power_on_err:
	mutex_unlock(&dev->tegra_camera_lock);
	pm_runtime_put(dev->dev);
	return ret;
}

static int tegra_camera_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct tegra_camera_dev *dev = file->private_data;

	dev_info(dev->dev, "%s\n", __func__);


	mutex_lock(&dev->tegra_camera_lock);
	/* disable HW clock */
	ret = tegra_camera_disable_clk(dev);
	if (ret)
		goto release_exit;
	/* nullify EMC request */
	ret = tegra_camera_disable_emc(dev);
	if (ret)
		goto release_exit;
	/* turn off CSI regulator */
	ret = tegra_camera_power_off(dev);
	if (ret)
		goto release_exit;

release_exit:
	mutex_unlock(&dev->tegra_camera_lock);
	WARN_ON(!atomic_xchg(&dev->in_use, 0));
	pm_runtime_put(dev->dev);
	return 0;
}

static const struct file_operations tegra_camera_fops = {
	.owner = THIS_MODULE,
	.open = tegra_camera_open,
	.unlocked_ioctl = tegra_camera_ioctl,
	.release = tegra_camera_release,
};

static int tegra_camera_clk_get(struct platform_device *pdev, const char *name,
				struct clk **clk)
{
	*clk = clk_get(&pdev->dev, name);
	if (IS_ERR_OR_NULL(*clk)) {
		dev_err(&pdev->dev, "%s: unable to get clock for %s\n",
			__func__, name);
		*clk = NULL;
		return PTR_ERR(*clk);
	}
	return 0;
}

static const struct of_device_id tegra_camera_of_match[] __devinitdata = {
	{ .compatible = "nvidia,tegra20-camera", },
	{ .compatible = "nvidia,tegra30-camera", },
	{ .compatible = "nvidia,tegra114-camera", },
	{}
};

MODULE_DEVICE_TABLE(of, tegra_camera_of_match);

static int tegra_camera_probe(struct platform_device *pdev)
{
	int err;
	struct tegra_camera_dev *dev;

	dev_info(&pdev->dev, "%s\n", __func__);
	dev = devm_kzalloc(&pdev->dev, sizeof(struct tegra_camera_dev),
			GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "%s: unable to allocate memory\n",
			__func__);
		goto alloc_err;
	}

	mutex_init(&dev->tegra_camera_lock);

	/* Powergate VE when boot */
	mutex_lock(&dev->tegra_camera_lock);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra_powergate_partition(TEGRA_POWERGATE_VENC);
	if (err)
		dev_err(&pdev->dev, "%s: powergate failed.\n", __func__);
#endif
	mutex_unlock(&dev->tegra_camera_lock);

	dev->dev = &pdev->dev;

	/* Get regulator pointer */
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	dev->reg = regulator_get(&pdev->dev, "vcsi");
#else
	dev->reg = regulator_get(&pdev->dev, "avdd_dsi_csi");
#endif
	if (IS_ERR_OR_NULL(dev->reg)) {
		if (dev->reg == ERR_PTR(-ENODEV)) {
			dev->reg = NULL;
			dev_info(&pdev->dev, "%s: no regulator device, overriding\n",
							__func__);
		} else {
			dev_err(&pdev->dev, "%s: couldn't get regulator\n",
							__func__);
			return PTR_ERR(dev->reg);
		}
	}

	dev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	dev->misc_dev.name = TEGRA_CAMERA_NAME;
	dev->misc_dev.fops = &tegra_camera_fops;
	dev->misc_dev.parent = &pdev->dev;

	err = misc_register(&dev->misc_dev);
	if (err) {
		dev_err(&pdev->dev, "%s: Unable to register misc device!\n",
		       TEGRA_CAMERA_NAME);
		goto misc_register_err;
	}

	err = tegra_camera_clk_get(pdev, "isp", &dev->isp_clk);
	if (err)
		goto isp_clk_get_err;
	err = tegra_camera_clk_get(pdev, "vi", &dev->vi_clk);
	if (err)
		goto vi_clk_get_err;
#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
	err = tegra_camera_clk_get(pdev, "vi_sensor2", &dev->vi_sensor_clk);
#else
	err = tegra_camera_clk_get(pdev, "vi_sensor", &dev->vi_sensor_clk);
#endif
	if (err)
		goto vi_sensor_clk_get_err;
	err = tegra_camera_clk_get(pdev, "csus", &dev->csus_clk);
	if (err)
		goto csus_clk_get_err;
	err = tegra_camera_clk_get(pdev, "csi", &dev->csi_clk);
	if (err)
		goto csi_clk_get_err;
	err = tegra_camera_clk_get(pdev, "emc", &dev->emc_clk);
	if (err)
		goto emc_clk_get_err;

#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
	err = tegra_camera_clk_get(pdev, "cilab", &dev->cilab_clk);
	if (err)
		goto cilab_clk_get_err;
	err = tegra_camera_clk_get(pdev, "cilcd", &dev->cilcd_clk);
	if (err)
		goto cilcd_clk_get_err;
	err = tegra_camera_clk_get(pdev, "cile", &dev->cile_clk);
	if (err)
		goto cile_clk_get_err;
	err = tegra_camera_clk_get(pdev, "pll_d2", &dev->pll_d2_clk);
	if (err)
		goto pll_d2_clk_get_err;
#endif

#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
	err = tegra_camera_clk_get(pdev, "vim2_clk", &dev->vim2_clk);
	if (err)
		goto vim2_clk_get_err;
#endif

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	/*
	 * Dedicated bw is what VI could ask for at most.
	 * Assume that preview has 3M@30fps and video has 8M@30fps
	 * Total = 3M * 30fps * 2Bpp + 8M * 30fps * 1.5Bpp
	 *       = 540MBps
	 * This number can be changed if VI requests more than this.
	 *
	 */
	dev->isomgr_handle = tegra_isomgr_register(TEGRA_ISO_CLIENT_VI_0,
				540000,	/* dedicated bw, KBps*/
				NULL,	/* tegra_isomgr_renegotiate */
				NULL);	/* *priv */
	if (!dev->isomgr_handle)
		goto isomgr_reg_err;
#endif

	pm_runtime_enable(&pdev->dev);
	/* dev is set in order to restore in _remove */
	platform_set_drvdata(pdev, dev);

	return 0;

#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
isomgr_reg_err:
	clk_put(dev->pll_d2_clk);
#endif

#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
vim2_clk_get_err:
	clk_put(dev->pll_d2_clk);
#endif

#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
pll_d2_clk_get_err:
	clk_put(dev->cile_clk);
cile_clk_get_err:
	clk_put(dev->cilcd_clk);
cilcd_clk_get_err:
	clk_put(dev->cilab_clk);
cilab_clk_get_err:
	clk_put(dev->emc_clk);
#endif
emc_clk_get_err:
	clk_put(dev->csi_clk);
csi_clk_get_err:
	clk_put(dev->csus_clk);
csus_clk_get_err:
	clk_put(dev->vi_sensor_clk);
vi_sensor_clk_get_err:
	clk_put(dev->vi_clk);
vi_clk_get_err:
	clk_put(dev->isp_clk);
isp_clk_get_err:
	misc_deregister(&dev->misc_dev);
misc_register_err:
	regulator_put(dev->reg);
alloc_err:
	return err;
}

static int tegra_camera_remove(struct platform_device *pdev)
{
	struct tegra_camera_dev *dev = platform_get_drvdata(pdev);


	clk_put(dev->isp_clk);
	clk_put(dev->vi_clk);
	clk_put(dev->vi_sensor_clk);
	clk_put(dev->csus_clk);
	clk_put(dev->csi_clk);
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	clk_put(dev->cilab_clk);
	clk_put(dev->cilcd_clk);
	clk_put(dev->cile_clk);
	clk_put(dev->pll_d2_clk);
#endif

#if defined(CONFIG_ARCH_TEGRA_14x_SOC)
	clk_put(dev->vim2_clk);
#endif

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	/*
	 * Return memory bandwidth to isomgr.
	 * If bandwidth is zero, then latency will be ignored
	 * in tegra_isomgr_reserve().
	 */
	{
		int ret = 0;
		ret = tegra_isomgr_reserve(dev->isomgr_handle,
					0,	/* KB/sec */
					0);	/* usec */
		if (!ret)
			return -ENOMEM;

		tegra_isomgr_unregister(dev->isomgr_handle);
		dev->isomgr_handle = NULL;
	}
#endif

	misc_deregister(&dev->misc_dev);
	regulator_put(dev->reg);
	mutex_destroy(&dev->tegra_camera_lock);

	return 0;
}

static int tegra_camera_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_camera_dev *dev = platform_get_drvdata(pdev);
	int ret = 0;

	mutex_lock(&dev->tegra_camera_lock);
	if (dev->power_on) {
		ret = -EBUSY;
		dev_err(&pdev->dev,
		"tegra_camera cannot suspend, "
		"application is holding on to camera. \n");
	}
	mutex_unlock(&dev->tegra_camera_lock);

	return ret;
}

static int tegra_camera_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver tegra_camera_driver = {
	.probe = tegra_camera_probe,
	.remove = tegra_camera_remove,
	.suspend = tegra_camera_suspend,
	.resume = tegra_camera_resume,
	.driver = {
		.name = TEGRA_CAMERA_NAME,
		.of_match_table = tegra_camera_of_match,
	}
};

static int __init tegra_camera_init(void)
{
	return platform_driver_register(&tegra_camera_driver);
}

static void __exit tegra_camera_exit(void)
{
	platform_driver_unregister(&tegra_camera_driver);
}

module_init(tegra_camera_init);
module_exit(tegra_camera_exit);

