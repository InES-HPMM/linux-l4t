/*
 * drivers/clocksource/tegra210_timer.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/tegra-timer.h>
#include <linux/tick.h>
#include <asm/mach/time.h>

static void __iomem *rtc_base;

/* set to 1 = busy every eight 32kHz clocks during copy of sec+msec to AHB */
#define TEGRA_RTC_REG_BUSY			0x004
#define TEGRA_RTC_REG_SECONDS			0x008
#define TEGRA_RTC_REG_MSEC_CDN_ALARM0		0x024
#define TEGRA_RTC_REG_INTR_MASK			0x028
/* write 1 bits to clear status bits */
#define TEGRA_RTC_REG_INTR_STATUS		0x02c

/* bits in INTR_MASK */
#define TEGRA_RTC_INTR_MASK_MSEC_CDN_ALARM	(1<<4)

/* bits in INTR_STATUS */
#define TEGRA_RTC_INTR_STATUS_MSEC_CDN_ALARM	(1<<4)

static u64 persistent_ms, last_persistent_ms;
static struct timespec persistent_ts;

/*
 * tegra_rtc_read - Reads the Tegra RTC registers
 * Care must be taken that this funciton is not called while the
 * tegra_rtc driver could be executing to avoid race conditions
 * on the RTC shadow register
 */
u64 tegra_rtc_read_ms(void)
{
	u32 ms = readl(rtc_base + RTC_MILLISECONDS);
	u32 s = readl(rtc_base + RTC_SHADOW_SECONDS);
	return (u64)s * MSEC_PER_SEC + ms;
}

#ifdef CONFIG_TEGRA_LP0_IN_IDLE
/* RTC hardware is busy when it is updating its values over AHB once
 * every eight 32kHz clocks (~250uS).
 * outside of these updates the CPU is free to write.
 * CPU is always free to read.
 */
static inline u32 tegra_rtc_check_busy(void)
{
	return readl(rtc_base + TEGRA_RTC_REG_BUSY) & 1;
}

/* Wait for hardware to be ready for writing.
 * This function tries to maximize the amount of time before the next update.
 * It does this by waiting for the RTC to become busy with its periodic update,
 * then returning once the RTC first becomes not busy.
 * This periodic update (where the seconds and milliseconds are copied to the
 * AHB side) occurs every eight 32kHz clocks (~250uS).
 * The behavior of this function allows us to make some assumptions without
 * introducing a race, because 250uS is plenty of time to read/write a value.
 */
static int tegra_rtc_wait_while_busy(void)
{
	int retries = 500; /* ~490 us is the worst case, ~250 us is best. */

	/* first wait for the RTC to become busy. this is when it
	 * posts its updated seconds+msec registers to AHB side. */
	while (tegra_rtc_check_busy()) {
		if (!retries--)
			goto retry_failed;
		udelay(1);
	}

	/* now we have about 250 us to manipulate registers */
	return 0;

retry_failed:
	pr_err("RTC: write failed:retry count exceeded.\n");
	return -ETIMEDOUT;
}

static int tegra_rtc_alarm_irq_enable(unsigned int enable)
{
	u32 status;

	/* read the original value, and OR in the flag. */
	status = readl(rtc_base + TEGRA_RTC_REG_INTR_MASK);
	if (enable)
		status |= TEGRA_RTC_INTR_MASK_MSEC_CDN_ALARM; /* set it */
	else
		status &= ~TEGRA_RTC_INTR_MASK_MSEC_CDN_ALARM; /* clear it */

	writel(status, rtc_base + TEGRA_RTC_REG_INTR_MASK);

	return 0;
}

static irqreturn_t tegra_rtc_interrupt(int irq, void *dev_id)
{
	u32 status;

	status = readl(rtc_base + TEGRA_RTC_REG_INTR_STATUS);
	if (status) {
		/* clear the interrupt masks and status on any irq. */
		tegra_rtc_wait_while_busy();
		writel(0, rtc_base + TEGRA_RTC_REG_INTR_MASK);
		writel(status, rtc_base + TEGRA_RTC_REG_INTR_STATUS);
	}

	return IRQ_HANDLED;
}

static struct irqaction tegra_rtc_irq = {
	.name		= "tegra_rtc",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
	.handler	= tegra_rtc_interrupt,
	.irq		= INT_RTC,
};

void tegra_rtc_set_trigger(unsigned long cycles)
{
	unsigned long msec;

	/* Convert to msec */
	msec = cycles / 1000;

	if (msec)
		msec = 0x80000000 | (0x0fffffff & msec);

	tegra_rtc_wait_while_busy();

	writel(msec, rtc_base + TEGRA_RTC_REG_MSEC_CDN_ALARM0);

	tegra_rtc_wait_while_busy();

	if (msec)
		tegra_rtc_alarm_irq_enable(1);
	else
		tegra_rtc_alarm_irq_enable(0);
}
#endif

/*
 * tegra_read_persistent_clock -  Return time from a persistent clock.
 *
 * Reads the time from a source which isn't disabled during PM, the
 * 32k sync timer.  Convert the cycles elapsed since last read into
 * nsecs and adds to a monotonically increasing timespec.
 * Care must be taken that this funciton is not called while the
 * tegra_rtc driver could be executing to avoid race conditions
 * on the RTC shadow register
 */
void tegra_read_persistent_clock(struct timespec *ts)
{
	u64 delta;
	struct timespec *tsp = &persistent_ts;

	last_persistent_ms = persistent_ms;
	persistent_ms = tegra_rtc_read_ms();
	delta = persistent_ms - last_persistent_ms;

	timespec_add_ns(tsp, delta * NSEC_PER_MSEC);
	*ts = *tsp;
}

static void __init tegra_init_rtc(struct device_node *np)
{
	struct clk *clk;
#if defined(CONFIG_TEGRA_LP0_IN_IDLE)
	int ret = 0;
#endif

	/*
	 * rtc registers are used by read_persistent_clock, keep the rtc clock
	 * enabled
	 */
	rtc_base = of_iomap(np, 0);
	if (!rtc_base) {
		pr_err("%s: Can't map RTC registers", __func__);
		BUG();
	}

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk))
		clk = clk_get_sys("rtc-tegra", NULL);

#if defined(CONFIG_TEGRA_LP0_IN_IDLE)
	/* clear out the hardware. */
	writel(0, rtc_base + TEGRA_RTC_REG_MSEC_CDN_ALARM0);
	writel(0xffffffff, rtc_base + TEGRA_RTC_REG_INTR_STATUS);
	writel(0, rtc_base + TEGRA_RTC_REG_INTR_MASK);

	ret = setup_irq(tegra_rtc_irq.irq, &tegra_rtc_irq);
	if (ret) {
		pr_err("Failed to register RTC IRQ: %d\n", ret);
		BUG();
	}

	enable_irq_wake(tegra_rtc_irq.irq);
#endif

	of_node_put(np);

	if (IS_ERR(clk))
		pr_warn("Unable to get rtc-tegra clock\n");
	else
		clk_prepare_enable(clk);

	register_persistent_clock(NULL, tegra_read_persistent_clock);
}
CLOCKSOURCE_OF_DECLARE(tegra_rtc, "nvidia,tegra-rtc", tegra_init_rtc);
