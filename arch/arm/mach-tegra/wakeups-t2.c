/*
 * Copyright (c) 2011, Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>

#include <mach/gpio-tegra.h>
#include <mach/irqs.h>

#include "gpio-names.h"
#include "iomap.h"

#define NUM_WAKE_EVENTS 31

/* TODO: We could populate the other table from this one at runtime
 * instead of always searching twice */
static int tegra_gpio_wakes[] = {
	[0]  = TEGRA_GPIO_PO5,
	[1]  = TEGRA_GPIO_PV3,
	[2]  = TEGRA_GPIO_PL1,
	[3]  = TEGRA_GPIO_PB6,
	[4]  = TEGRA_GPIO_PN7,
	[5]  = TEGRA_GPIO_PA0,
	[6]  = TEGRA_GPIO_PU5,
	[7]  = TEGRA_GPIO_PU6,
	[8]  = TEGRA_GPIO_PC7,
	[9]  = TEGRA_GPIO_PS2,
	[10] = TEGRA_GPIO_PAA1,
	[11] = TEGRA_GPIO_PW3,
	[12] = TEGRA_GPIO_PW2,
	[13] = TEGRA_GPIO_PY6,
	[14] = TEGRA_GPIO_PV6,
	[15] = TEGRA_GPIO_PJ7,
	[16] = -EINVAL,
	[17] = -EINVAL,
	[18] = -EINVAL,
	[19] = -EINVAL,	 /* TEGRA_USB1_VBUS, */
	[20] = -EINVAL, /* TEGRA_USB3_VBUS, */
	[21] = -EINVAL, /* TEGRA_USB1_ID, */
	[22] = -EINVAL, /* TEGRA_USB3_ID, */
	[23] = TEGRA_GPIO_PI5,
	[24] = TEGRA_GPIO_PV2,
	[25] = TEGRA_GPIO_PS4,
	[26] = TEGRA_GPIO_PS5,
	[27] = TEGRA_GPIO_PS0,
	[28] = TEGRA_GPIO_PQ6,
	[29] = TEGRA_GPIO_PQ7,
	[30] = TEGRA_GPIO_PN2,
	[31] = -EINVAL,
	[32] = -EINVAL,
	[33] = -EINVAL,
	[34] = -EINVAL,
	[35] = -EINVAL,
	[36] = -EINVAL,
	[37] = -EINVAL,
	[38] = -EINVAL,
};

static int tegra_wake_event_irq[] = {
	[0]  = -EAGAIN, /* Search the GPIO table */
	[1]  = -EAGAIN,
	[2]  = -EAGAIN,
	[3]  = -EAGAIN,
	[4]  = -EAGAIN,
	[5]  = -EAGAIN,
	[6]  = -EAGAIN,
	[7]  = -EAGAIN,
	[8]  = -EAGAIN,
	[9]  = -EAGAIN,
	[10] = -EAGAIN,
	[11] = -EAGAIN,
	[12] = -EAGAIN,
	[13] = -EAGAIN,
	[14] = -EAGAIN,
	[15] = -EAGAIN,
	[16] = INT_RTC,
	[17] = INT_KBC,
	[18] = INT_EXTERNAL_PMU,
	[19] = -EINVAL,	 /* TEGRA_USB1_VBUS, */
	[20] = -EINVAL, /* TEGRA_USB3_VBUS, */
	[21] = -EINVAL, /* TEGRA_USB1_ID, */
	[22] = -EINVAL, /* TEGRA_USB3_ID, */
	[23] = -EAGAIN,
	[24] = -EAGAIN,
	[25] = -EAGAIN,
	[26] = -EAGAIN,
	[27] = -EAGAIN,
	[28] = -EAGAIN,
	[29] = -EAGAIN,
	[30] = -EAGAIN,
	[31] = -EINVAL,
	/*
	 * The gpio bank irqs aren't actually wake sources, but they don't
	 * prevent lp0 because the gpio chained irq is requested directly
	 */
	[32] = INT_GPIO1,
	[33] = INT_GPIO2,
	[34] = INT_GPIO3,
	[35] = INT_GPIO4,
	[36] = INT_GPIO5,
	[37] = INT_GPIO6,
	[38] = INT_GPIO7,
};

int tegra_irq_to_wake(int irq)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(tegra_wake_event_irq); i++)
		if (tegra_wake_event_irq[i] == irq)
			break;

	if (i == ARRAY_SIZE(tegra_wake_event_irq)) {
		for (i = 0; i < ARRAY_SIZE(tegra_gpio_wakes); i++)
			if (gpio_to_irq(tegra_gpio_wakes[i]) == irq)
				break;

		if (i == ARRAY_SIZE(tegra_gpio_wakes))
			return -ENOTSUPP;
	}

	if (i > NUM_WAKE_EVENTS)
		return -EALREADY;

	return i;
}

int tegra_wake_to_irq(int wake)
{
	int ret;

	if (wake < 0)
		return -EINVAL;

	if (wake >= NUM_WAKE_EVENTS)
		return -EINVAL;

	ret = tegra_wake_event_irq[wake];
	if (ret == -EAGAIN)
		ret = gpio_to_irq(tegra_gpio_wakes[wake]);

	return ret;
}
