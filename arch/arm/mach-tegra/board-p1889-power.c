/*
 * arch/arm/mach-tegra/board-p1889-power.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>
#include <linux/tegra-pmc.h>

#include "pm.h"
#include "board.h"
#include "gpio-names.h"
#include "board-common.h"
#include "board-p1889.h"
#include "devices.h"
#include <mach/board_id.h>
#include "vcm30_t124.h"

/*
 * GPIO init table for PCA9539 MISC IO GPIOs
 * related to DAP_D_SEL and DAP_D_EN.
 */
static struct gpio p1889_system_1_gpios[] = {
	{MISCIO_MUX_DAP_D_SEL,	GPIOF_OUT_INIT_LOW,	"dap_d_sel"},
	{MISCIO_MUX_DAP_D_EN,	GPIOF_OUT_INIT_LOW,	"dap_d_en"},
};

static struct gpio p1889_system_2_gpios[] = {
	{MISCIO_MDM_EN,		GPIOF_OUT_INIT_HIGH,	"mdm_en"},
	{MISCIO_MDM_COLDBOOT,	GPIOF_IN,		"mdm_coldboot"},
};

static struct gpio p1889_system_2_nofree_gpios[] = {
	{MISCIO_AP_MDM_RESET,	GPIOF_OUT_INIT_HIGH|GPIOF_EXPORT, "ap_mdm_rst"}
};

static int __init p1889_system_gpio_init(struct gpio *gpios_info, int pin_count)
{
	int ret;

	/* Set required system GPIOs to initial bootup values */
	ret = gpio_request_array(gpios_info, pin_count);

	if (ret)
		pr_err("%s gpio_request_array failed(%d)\r\n",
				 __func__, ret);

	gpio_free_array(gpios_info, pin_count);

	return ret;
}

/*
 * TODO: Check for the correct pca953x before invoking client
 *  init functions
 */
static int pca953x_client_setup(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context)
{
	int ret = 0;
	int system = (int)context;

	switch (system) {
	case 1:
		ret = p1889_system_gpio_init(p1889_system_1_gpios,
				ARRAY_SIZE(p1889_system_1_gpios));
		break;
	case 2:
		ret = p1889_system_gpio_init(p1889_system_2_gpios,
				ARRAY_SIZE(p1889_system_2_gpios));
		if (!ret) {
			ret = gpio_request_array(p1889_system_2_nofree_gpios,
				ARRAY_SIZE(p1889_system_2_nofree_gpios));
			if (ret)
				pr_err("%s gpio_request_array failed(%d)\r\n",
						__func__, ret);
		}
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0)
		goto fail;

	return 0;
fail:
	pr_err("%s failed(%d)\r\n", __func__, ret);
	return ret;
}

static struct pca953x_platform_data p1889_miscio_1_pca9539_data = {
	.gpio_base  = PCA953X_MISCIO_1_GPIO_BASE,
	.setup = pca953x_client_setup,
	.context = (void *)1,
};

static struct pca953x_platform_data p1889_miscio_2_pca9539_data = {
	.gpio_base  = PCA953X_MISCIO_2_GPIO_BASE,
	.setup = pca953x_client_setup,
	.context = (void *)2,
};

static struct i2c_board_info p1889_i2c2_board_info_pca9539_1 = {
	I2C_BOARD_INFO("pca9539", PCA953X_MISCIO_1_ADDR),
	.platform_data = &p1889_miscio_1_pca9539_data,
};

static struct i2c_board_info p1889_i2c2_board_info_pca9539_2 = {
	I2C_BOARD_INFO("pca9539", PCA953X_MISCIO_2_ADDR),
	.platform_data = &p1889_miscio_2_pca9539_data,
};

int __init p1889_pca953x_init(void)
{
	int is_e1860_b00 = 0;

	is_e1860_b00 = tegra_is_board(NULL, "61860", NULL, "300", NULL);

	if (is_e1860_b00) {

		i2c_register_board_info(1,
				&p1889_i2c2_board_info_pca9539_1, 1);

		/*
		 * some pins of p1889_i2c2_board_info_pca9539_2 may be used
		 * later on some other board versions, not only b00
		 */
		i2c_register_board_info(1,
				&p1889_i2c2_board_info_pca9539_2, 1);
	}
	return 0;
}
