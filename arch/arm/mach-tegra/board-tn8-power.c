/*
 * arch/arm/mach-tegra/board-tn8-power.c
 *
 * Copyright (c) 2014 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/edp.h>
#include <linux/edpdev.h>
#include <mach/edp.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>

#include <linux/gpio.h>
#include <linux/power/bq2419x-charger.h>
#include <linux/max17048_battery.h>
#include <linux/tegra-soc.h>

#include <mach/irqs.h>

#include <asm/mach-types.h>
#include <linux/power/sbs-battery.h>

#include "pm.h"
#include "board.h"
#include "tegra-board-id.h"
#include "board-common.h"
#include "board-ardbeg.h"
#include "board-pmu-defines.h"
#include "devices.h"
#include <linux/platform/tegra/common.h>
#include "iomap.h"
#include "tegra-board-id.h"
#include "battery-ini-model-data.h"


static u32 tegra_chip_id;
#define IS_T13X			(tegra_chip_id == TEGRA_CHIPID_TEGRA13)

int __init tn8_edp_init(void)
{
	unsigned int regulator_mA;

	if (!tegra_chip_id)
		tegra_chip_id = tegra_get_chip_id();

	/* gpu maximum current */
	regulator_mA = 11200;
	pr_info("%s: GPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_gpu_edp_limits(regulator_mA);

	return 0;
}
