/*
 * Copyright 2014 Audience, Inc.
 *
 * Author: Steven Tarr  <starr@audience.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * For the time being, the default actions are those required for the
 * ES755 as decibed in "ES755 ENgineering API Guide" version 0.31
 */
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include "escore.h"
#include "escore-vs.h"
#include "escore-uart-common.h"
#include "escore-slim.h"

#define ES_PM_SLEEP_DELAY		30 /* 30 ms */
#define ES_PM_RESUME_TIME		30 /* 30ms */
#define ES_PM_AUTOSUSPEND_DELAY		3000 /* 3 sec */

static inline bool es_ready_to_suspend(struct escore_priv *escore)
{
	bool is_active;

	is_active = escore->flag.rx1_route_enable || \
		    escore->flag.rx2_route_enable || \
		    escore->flag.tx1_route_enable || \
		    atomic_read(&escore->active_streams);

	return !is_active;
}

static int escore_non_vs_suspend(struct device *dev);
static int escore_non_vs_resume(struct device *dev);

static int escore_vs_suspend(struct device *dev)
{
	struct escore_priv *escore = &escore_priv;
	int ret;

	dev_dbg(dev, "%s()\n", __func__);

	/* Already in low power mode because of runtime CVQ suspend.
	 */
	if (escore_priv.escore_power_state == ES_SET_POWER_STATE_VS_LOWPWR) {
		dev_dbg(dev, "%s() Already in VS low power mode\n", __func__);
		return 0;
	}

	/*
	 * Do not suspend in runtime if CVQ training is going on.
	 * For System suspend, application onPause() have already
	 * put chip in normal mode.
	 */
	if (escore_priv.mode == VOICESENSE_PENDING || \
				escore_priv.mode == VOICESENSE) {
		dev_dbg(dev, "%s() Suspend Deferred\n", __func__);
		return -EBUSY;
	}

	ret = escore->vs_ops.escore_voicesense_sleep(escore);

	return ret;
}

static int escore_vs_resume(struct device *dev)
{
	struct escore_priv *escore = &escore_priv;
	int ret;

	dev_dbg(dev, "%s()\n", __func__);

	ret = escore->vs_ops.escore_voicesense_wakeup(escore);

	if (!escore_priv.defer_intr_config)
		blocking_notifier_call_chain(escore->irq_notifier_list,
				ES_RECONFIG_INTR_EVENT, escore);

	return ret;
}

static int escore_non_vs_suspend(struct device *dev)
{
	int ret = 0;
	int rsp;
	u32 cmd;

	dev_dbg(dev, "%s()\n", __func__);

	/*
	 * Do not suspend in runtime if CVQ training is going on.
	 * For System suspend, application onPause() have already
	 * put chip in normal mode.
	 */
	if (escore_priv.escore_power_state != ES_SET_POWER_STATE_NORMAL
		|| escore_priv.mode == VOICESENSE_PENDING) {
		dev_dbg(dev, "%s() Suspend Deferred\n", __func__);
		return -EBUSY;
	}

	/* It is assumed that a channels have already been muted */
	/* Send a SetPowerState command - no respnse */
	cmd = (ES_SET_POWER_STATE << 16) |
		escore_priv.non_vs_sleep_state;
	ret = escore_priv.bus.ops.cmd(&escore_priv, cmd,
			&rsp);
	if (ret < 0) {
		dev_err(dev, "%s() - Chip dead.....\n", __func__);
		goto suspend_out;
	}

	if (escore_priv.pdata->gpioa_gpio != -1)
		escore_priv.cmd_compl_mode = ES_CMD_COMP_POLL;

	/* Set delay time time */
	msleep(ES_PM_SLEEP_DELAY);

	escore_priv.escore_power_state = escore_priv.non_vs_sleep_state;

suspend_out:

	return ret;
}

static int escore_non_vs_resume(struct device *dev)
{
	struct escore_priv *escore = &escore_priv;
	int ret = 0;
	dev_dbg(dev, "%s()\n", __func__);

	ret = escore_wakeup(escore);
	if (ret) {
		dev_err(dev, "%s() wakeup failed ret = %d\n", __func__, ret);
		goto escore_wakeup_fail_recovery;
	}

	if (!escore->defer_intr_config)
		blocking_notifier_call_chain(escore->irq_notifier_list,
				ES_RECONFIG_INTR_EVENT, escore);

	dev_dbg(dev, "%s() - out rc =%d\n", __func__, ret);

	escore->escore_power_state = ES_SET_POWER_STATE_NORMAL;
	return ret;

escore_wakeup_fail_recovery:
	escore_gpio_reset(&escore_priv);
	ret = escore_priv.boot_ops.bootup(&escore_priv);
	escore_priv.escore_power_state = ES_SET_POWER_STATE_NORMAL;

	return ret;
}

static int escore_runtime_suspend(struct device *dev)
{
	int ret = 0;
	unsigned long time_left;
	struct escore_priv *escore = &escore_priv;

	dev_dbg(dev, "%s()\n", __func__);

	if (!es_ready_to_suspend(escore)) {
		dev_dbg(dev, "%s() - Not ready for suspend\n", __func__);
		return 0;
	}

	time_left = pm_runtime_autosuspend_expiration(escore->dev);
	if (time_left) {
		/*
		 * If pm_runtime_suspend() is called without RPM_AUTO flag,
		 * this condition may be hit. This condition may happen if
		 * the suspend() is initiated outside of this driver, e.g.
		 * ASoC subsystem which calls pm_runtime_put() which internally
		 * calls pm_runtime_idle() which does not call suspend() with
		 * RPM_AUTO flag.
		 *
		 * Ideally, the function should check for the autosuspend
		 * timer expiration, and if the timer is not expiring, -EBUSY
		 * should be returned. But kernel does not handle this
		 * error code if suspend() was called without RPM_AUTO flag,
		 * so here this condition is ignored and chip would enter into
		 * suspend mode.
		 */

		pm_runtime_mark_last_busy(escore->dev);
		dev_dbg(escore->dev, "%s() Autosuspend timer not expired yet\n",
				__func__);
		return -EBUSY;
	}

	/*
	 * If the user has selected MP_SLEEP playback mode, the chip will not
	 * enter into normal mode once the stream is shutdown. We need to
	 * bring chip into normal mode to enter into desired runtime suspend
	 * state.
	 */
	if (escore->escore_power_state == ES_SET_POWER_STATE_MP_SLEEP) {
		ret = escore_wakeup(escore);
		if (ret) {
			dev_err(dev, "%s() wakeup failed ret = %d\n",
					__func__, ret);
			goto out;
		}
	}
#ifdef CONFIG_SND_SOC_ES_RUNTIME_SUSPEND_MODE_SLEEP
	ret = escore_non_vs_suspend(dev);
#elif defined(CONFIG_SND_SOC_ES_RUNTIME_SUSPEND_MODE_CVQ)
	if (escore->voice_sense &&
		escore->vs_ops.escore_is_voicesense_sleep_enable(escore))
		ret = escore_vs_suspend(dev);
	else
		ret = escore_non_vs_suspend(dev);
#endif

	if (ret)
		goto out;

	escore->pm_state = ES_PM_RUNTIME_SLEEP;

	/* Disable the clocks */
	if (escore_priv.pdata->esxxx_clk_cb)
		escore_priv.pdata->esxxx_clk_cb(0);

	if (escore_priv.dev && device_may_wakeup(escore_priv.dev))
		enable_irq_wake(gpio_to_irq(escore_priv.pdata->gpiob_gpio));

out:
	return ret;
}

static int escore_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct escore_priv *escore = &escore_priv;
	struct device *p = dev->parent;

	dev_dbg(dev, "%s()\n", __func__);
	if (p && pm_runtime_status_suspended(p)) {
		dev_err(dev, "%s() - parent is suspended\n", __func__);
		pm_runtime_resume(p);
	}

	if (escore->pm_state == ES_PM_NORMAL) {
		dev_dbg(dev, "%s() - already awake\n", __func__);
		return 0;
	}
#ifdef CONFIG_SND_SOC_ES_RUNTIME_SUSPEND_MODE_SLEEP
	ret = escore_non_vs_resume(dev);
#elif defined(CONFIG_SND_SOC_ES_RUNTIME_SUSPEND_MODE_CVQ)
	if (escore->voice_sense &&
		escore->vs_ops.escore_is_voicesense_sleep_enable(escore))
		ret = escore_vs_resume(dev);
	else
		ret = escore_non_vs_resume(dev);
#endif
	if (!ret)
		escore->pm_state = ES_PM_NORMAL;

	pm_runtime_mark_last_busy(escore->dev);

	if (escore_priv.dev && device_may_wakeup(escore_priv.dev))
		disable_irq_wake(gpio_to_irq(escore_priv.pdata->gpiob_gpio));

	dev_dbg(dev, "%s() complete %d\n", __func__, ret);

	return ret;
}

int escore_system_suspend(struct device *dev)
{
	int ret = 0;
	struct escore_priv *escore = &escore_priv;

	dev_dbg(dev, "%s()\n", __func__);

	if (escore->pm_state == ES_PM_ASLEEP) {
		dev_dbg(dev, "%s() - already suspended\n", __func__);
		return 0;
	}

	if (!es_ready_to_suspend(escore)) {
		dev_dbg(dev, "%s() - Not ready for suspend\n", __func__);
		return 0;
	}

	if (escore->voice_sense &&
		escore->vs_ops.escore_is_voicesense_sleep_enable(escore)) {
		if (escore->escore_power_state == escore->non_vs_sleep_state) {
			ret = escore_non_vs_resume(dev);
			if (ret) {
				dev_err(escore->dev,
					"%s() non VS resume failed ret = %d\n",
					__func__, ret);
					goto out;
			}
		}
		ret = escore_vs_suspend(dev);
	} else {
		if (escore->escore_power_state == escore->non_vs_sleep_state) {
			dev_dbg(dev, "%s() - already suspended in runtime\n",
					__func__);
			ret = 0;
			escore->pm_state = ES_PM_ASLEEP;
			goto out;
		} else if (escore->escore_power_state ==
				ES_SET_POWER_STATE_MP_SLEEP) {
			/*
			 * Case where playback was happening into MP_SLEEP mode
			 * and stream is shutdown now. But system suspend
			 * initiated before runtime suspend got a chance.
			 * Need to bring chip into normal mode before putting
			 * into suspend state.
			 */
			ret = escore_wakeup(escore);
			if (ret) {
				dev_err(dev, "%s() wakeup failed ret = %d\n",
						__func__, ret);
				goto out;
			}
		}
		ret = escore_non_vs_suspend(dev);
	}

	if (ret)
		goto out;

	escore->pm_state = ES_PM_ASLEEP;

	/* Disable the clocks */
	if (escore_priv.pdata->esxxx_clk_cb)
		escore_priv.pdata->esxxx_clk_cb(0);

	if (escore_priv.dev && device_may_wakeup(escore_priv.dev))
		enable_irq_wake(gpio_to_irq(escore_priv.pdata->gpiob_gpio));
out:
	return ret;
}
int escore_system_resume(struct device *dev)
{
	int ret = 0;
	struct escore_priv *escore = &escore_priv;

	dev_dbg(dev, "%s()\n", __func__);

	/* Voice Wakeup case */
	if (escore->pm_state == ES_PM_NORMAL) {
		dev_dbg(dev, "%s() - already awake\n", __func__);
		return 0;
	}

	if (escore->voice_sense &&
		escore->vs_ops.escore_is_voicesense_sleep_enable(escore))
		ret = escore_vs_resume(dev);
	else
		ret = escore_non_vs_resume(dev);

	if (!ret)
		escore->pm_state = ES_PM_NORMAL;

	if (escore_priv.dev && device_may_wakeup(escore_priv.dev))
		disable_irq_wake(gpio_to_irq(escore_priv.pdata->gpiob_gpio));

	/* Bring the device to full powered state upon system resume */
	pm_runtime_disable(dev);
	pm_runtime_mark_last_busy(escore_priv.dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	dev_dbg(dev, "%s() complete %d\n", __func__, ret);

	return ret;
}

static int escore_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s()\n", __func__);
	pm_request_autosuspend(dev);
	return -EAGAIN;
}

int escore_generic_suspend(struct device *dev)
{
	int ret = 0;
	struct escore_priv *escore = &escore_priv;

	dev_dbg(dev, "%s()\n", __func__);

	if (escore->dev != dev) {
		dev_dbg(dev, "%s() Invalid device\n", __func__);
		return 0;
	}

	if (escore->system_suspend)
		escore_system_suspend(dev);
	else
		escore_runtime_suspend(dev);

	return ret;
}

int escore_generic_resume(struct device *dev)
{
	int ret = 0;
	struct escore_priv *escore = &escore_priv;

	dev_dbg(dev, "%s()\n", __func__);

	if (escore->dev != dev) {
		dev_dbg(dev, "%s() Invalid device\n", __func__);
		return 0;
	}

	if (escore->system_suspend)
		escore_system_resume(dev);
	else
		escore_runtime_resume(dev);

	return ret;
}

int escore_prepare(struct device *dev)
{
	struct escore_priv *escore = &escore_priv;

	dev_dbg(dev, "%s()\n", __func__);

	if (escore->dev != dev)
		dev_dbg(dev, "%s() Invalid device\n", __func__);
	else
		escore->system_suspend = 1;

	return 0;
}

void escore_complete(struct device *dev)
{
	struct escore_priv *escore = &escore_priv;

	dev_dbg(dev, "%s()\n", __func__);

	if (escore->dev != dev)
		dev_dbg(dev, "%s() Invalid device\n", __func__);
	else
		escore->system_suspend = 0;
}

const struct dev_pm_ops escore_pm_ops = {
	.suspend = escore_generic_suspend,
	.resume = escore_generic_resume,
	.prepare = escore_prepare,
	.complete = escore_complete,
	.runtime_suspend = escore_runtime_suspend,
	.runtime_resume = escore_runtime_resume,
	.runtime_idle = escore_runtime_idle,
};

void escore_pm_init(void)
{

	if (escore_priv.voice_sense) {
		escore_priv.vs_pm_ops.suspend = escore_vs_suspend;
		escore_priv.vs_pm_ops.resume = escore_vs_resume;
	}
	escore_priv.non_vs_pm_ops.suspend = escore_non_vs_suspend;
	escore_priv.non_vs_pm_ops.resume = escore_non_vs_resume;

	return;
}

void escore_pm_enable(void)
{
	dev_dbg(escore_priv.dev, "%s()\n", __func__);
	escore_priv.pm_enable = 1;
	pm_runtime_set_active(escore_priv.dev);
	pm_runtime_enable(escore_priv.dev);
	pm_runtime_set_autosuspend_delay(escore_priv.dev,
					ES_PM_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(escore_priv.dev);
	device_init_wakeup(escore_priv.dev, true);
	return;
}

void escore_pm_disable(void)
{
	struct escore_priv *escore = &escore_priv;
	struct device *xdev = escore->dev;

	escore->pm_enable = 0;
	if (escore->pm_state == ES_PM_RUNTIME_SLEEP ||
			escore->pm_state == ES_PM_ASLEEP) {
		dev_dbg(xdev, "%s(): Wakeup chip before Runtime PM disable\n",
				__func__);
		escore_runtime_resume(xdev);
	}
	pm_runtime_disable(xdev);
	return;
}

void escore_pm_vs_enable(struct escore_priv *escore, bool value)
{
	struct device *xdev = escore->dev;
	dev_dbg(escore->dev, "%s()\n", __func__);
	if (xdev) {
		if (value == true)
			device_wakeup_enable(xdev);
		else
			device_wakeup_disable(xdev);
	}
	return;
}

int escore_pm_get_sync(void)
{
	int ret = 0;
	dev_dbg(escore_priv.dev, "%s()\n", __func__);
	if (escore_priv.pm_enable == 1) {
		if (!escore_priv.system_suspend)
			ret = pm_runtime_get_sync(escore_priv.dev);
	}
	return ret;
}

void escore_pm_put_autosuspend(void)
{
	int ret = 0;
	dev_dbg(escore_priv.dev, "%s()\n", __func__);
	if (escore_priv.pm_enable == 1 && !escore_priv.system_suspend) {
		pm_runtime_mark_last_busy(escore_priv.dev);
		ret = pm_runtime_put_sync_autosuspend(escore_priv.dev);
		if (ret)
			dev_err(escore_priv.dev, "%s() - failed\n", __func__);
	}

	return;
}


bool escore_is_probe_error(void)
{
	struct escore_priv *escore = &escore_priv;

	return escore->is_probe_error;
}
