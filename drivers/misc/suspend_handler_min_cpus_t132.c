/*
 * drivers/misc/suspend_handler_min_cpus_t132.c
 *
 * Copyright (c) 2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/pm_qos.h>
#include <linux/suspend.h>

static struct pm_qos_request suspend_min_cpus_req;

/*
 * XXX: LP0 WAR: If CPU1 is offline on LP0, the device will fail to go into LP0
 * because the CPU gets put into C6 during normal hotplug. This causes LP0 (C7)
 * to fail thereby hanging the system. Work around this by bringing back CPU1
 * online and putting it in C7 before requesting LP0 from CPU0
 */
static int suspend_pm_notify(struct notifier_block *nb,
					unsigned long event, void *data)
{
	if (event == PM_SUSPEND_PREPARE) {
		pm_qos_update_request(&suspend_min_cpus_req,
					num_possible_cpus());
		if (num_online_cpus() != num_possible_cpus()) {
			pr_err("%s: min online cpu request failed\n",
					__func__);
			return NOTIFY_BAD;
		}
	} else if (event == PM_POST_SUSPEND)
		pm_qos_update_request(&suspend_min_cpus_req,
				PM_QOS_MIN_ONLINE_CPUS_DEFAULT_VALUE);

	return NOTIFY_OK;
}

/*
 * The priority of this suspend notifier should always be higher than
 * cpu_hotplug's pm_notifier. Otherwise cpu_hotplug will disable hotplug
 * operations on suspend causing the following cpu_up call in
 * suspend_pm_notify to fail
 */
static struct notifier_block suspend_pm_notifier = {
	.notifier_call = suspend_pm_notify,
	.priority = 1,
};

static int __init suspend_handler_min_cpus_init(void)
{
	int err = register_pm_notifier(&suspend_pm_notifier);

	if (err)
		pr_err("%s: Failed to register pm notifier\n",
			__func__);

	pm_qos_add_request(&suspend_min_cpus_req, PM_QOS_MIN_ONLINE_CPUS,
			PM_QOS_MIN_ONLINE_CPUS_DEFAULT_VALUE);

	return err;
}
module_init(suspend_handler_min_cpus_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("T132 Suspend handler for min online cpus constraint");
