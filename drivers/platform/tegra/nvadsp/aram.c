/*
 * aram.c
 *
 * ARAM Bookkepping
 *
 * Copyright (C) 2014, NVIDIA Corporation. All rights reserved.
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

#include <linux/tegra_nvadsp.h>
#include <linux/irqchip/tegra-agic.h>
#include <linux/interrupt.h>

#include "dev.h"
#include "amc.h"

static struct platform_device *nvadsp_pdev;
static struct nvadsp_drv_data *nvadsp_drv_data;

bool nvadsp_aram_request(char *start, size_t size, char *id)
{
	return false;
}

void nvadsp_aram_release(char *start, size_t size)
{
	return;
}

static inline u32 amc_readl(u32 reg)
{
	return readl(nvadsp_drv_data->base_regs[AMC] + reg);
}

static inline void amc_writel(u32 val, u32 reg)
{
	writel(val, nvadsp_drv_data->base_regs[AMC] + reg);
}

static irqreturn_t nvadsp_amc_error_int_handler(int irq, void *devid)
{
	u32 val, addr, status, intr = 0;

	status = amc_readl(AMC_INT_STATUS);
	addr = amc_readl(AMC_ERROR_ADDR);

	if (status & AMC_INT_STATUS_ARAM) {
		/*
		 * Ignore addresses lesser than AMC_ERROR_ADDR_IGNORE (4k)
		 * as those are spurious ones due a hardware issue.
		 */
		if (addr > AMC_ERROR_ADDR_IGNORE)
			pr_info("nvadsp: invalid ARAM access. address: 0x%x\n",
				addr);

		intr |= AMC_INT_INVALID_ARAM_ACCESS;
	}

	if (status & AMC_INT_STATUS_REG) {
		pr_info("nvadsp: invalid AMC reg access. address: 0x%x\n",
			addr);
		intr |= AMC_INT_INVALID_REG_ACCESS;
	}

	val = amc_readl(AMC_INT_CLR);
	val |= intr;
	amc_writel(val, AMC_INT_CLR);

	return IRQ_HANDLED;
}

status_t nvadsp_aram_init(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);
	int ret = 0;
	int virq;

	nvadsp_pdev = pdev;
	nvadsp_drv_data = drv;

	virq = tegra_agic_irq_get_virq(INT_AMC_ERR);
	if (!virq)
		goto error;

	ret = request_irq(virq, nvadsp_amc_error_int_handler,
			  0, "AMC error int", pdev);

	dev_info(&pdev->dev, "AMC/ARAM initialized.\n");

 error:
	return ret;
}
