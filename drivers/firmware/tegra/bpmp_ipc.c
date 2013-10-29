/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_data/tegra_bpmp.h>
#include <linux/spinlock.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp_private.h"

#define RES_SEMA_SHRD_INBOX	IO_ADDRESS(TEGRA_RES_SEMA_BASE + 0x10)
#define INBOX_TAG		(1 << 29)
#define RES_SEMA_SHRD_OUTBOX	IO_ADDRESS(TEGRA_RES_SEMA_BASE + 0x20)
#define OUTBOX_IE_OBF		(1 << 31)
#define OUTBOX_TAG		(1 << 29)
#define MRQ_LIMIT		((1 << 28) - 1)
#define TEGRA_ATOMICS_BASE	0x70016000

#define ATOMICS_AP0_TRIGGER	IO_ADDRESS(TEGRA_ATOMICS_BASE + 0x000)
#define TRIGGER_ID_SHIFT	16
#define TRIGGER_CMD_PUT		5
#define TRIGGER_CMD_GET		4
#define ATOMICS_AP0_SETUP_V(id)	IO_ADDRESS(TEGRA_ATOMICS_BASE + 0x400 + id * 4)
#define ATOMICS_AP0_RESULT(id)	IO_ADDRESS(TEGRA_ATOMICS_BASE + 0xc00 + id * 4)
#define ATOMICS_NUM_OUTBOX	64
#define ATOMICS_NUM_INBOX	64
#define ATOMICS_OUTBOX_ID_OFF	0
#define ATOMICS_INBOX_ID_OFF	ATOMICS_NUM_OUTBOX

DEFINE_SPINLOCK(lock);

static void bpmp_write_mbexdata(int off, void *data, int sz)
{
	int id;
	int i;
	int num;
	u32 *p;

	if (!data)
		return;

	num = DIV_ROUND_UP(sz, 4);
	p = data;

	if (off + num > ATOMICS_NUM_OUTBOX) {
		WARN_ON(1);
		return;
	}

	for (i = 0; i < num; i++) {
		id = ATOMICS_OUTBOX_ID_OFF + off + i;
		writel(p[i], ATOMICS_AP0_SETUP_V(id));
		writel(id << TRIGGER_ID_SHIFT | TRIGGER_CMD_PUT,
				ATOMICS_AP0_TRIGGER);
	}
}

static void bpmp_read_mbexdata(int off, void *data, int sz)
{
	int id;
	int i;
	int num;
	u32 *p;

	if (!data)
		return;

	num = DIV_ROUND_UP(sz, 4);
	p = data;

	if (off + num >= ATOMICS_NUM_INBOX) {
		WARN_ON(1);
		return;
	}

	for (i = 0; i < num; i++) {
		id = ATOMICS_INBOX_ID_OFF + off + i;
		writel(id << TRIGGER_ID_SHIFT | TRIGGER_CMD_GET,
				ATOMICS_AP0_TRIGGER);
		p[i] = readl(ATOMICS_AP0_RESULT(id));
	}
}

static void bpmp_write_completion(struct completion *work)
{
	bpmp_write_mbexdata(0, &work, sizeof(work));
}

static struct completion *bpmp_read_completion(void)
{
	struct completion *work = 0;
	bpmp_read_mbexdata(0, &work, sizeof(work));
	return work;
}

static void bpmp_write_func_data(void *data, int sz)
{
	bpmp_write_mbexdata(2, data, sz);
}

static void bpmp_read_func_data(void *data, int sz)
{
	bpmp_read_mbexdata(2, data, sz);
}

static void bpmp_empty_ib(void)
{
	writel(INBOX_TAG, RES_SEMA_SHRD_INBOX);
}

irqreturn_t bpmp_inbox_irq(int irq, void *data)
{
	struct completion *work;

	/* clear interrupt */
	writel(0, RES_SEMA_SHRD_INBOX);

	work = bpmp_read_completion();

	if (work)
		complete(work);

	return IRQ_HANDLED;
}

static void bpmp_raise_mrq(int mrq)
{
	writel(OUTBOX_IE_OBF | OUTBOX_TAG | mrq, RES_SEMA_SHRD_OUTBOX);
}

static void bpmp_wait_for_ob_empty(void)
{
	while (readl(RES_SEMA_SHRD_OUTBOX))
		;
}

/* IPC without ACK */
static int bpmp_send(int mrq, void *data, int sz)
{
	unsigned long flags;

	if (mrq >= MRQ_LIMIT)
		return -EINVAL;

	spin_lock_irqsave(&lock, flags);

	bpmp_wait_for_ob_empty();
	bpmp_write_func_data(data, sz);
	bpmp_raise_mrq(mrq);

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static bool bpmp_ib_empty(void)
{
	return !(readl(RES_SEMA_SHRD_INBOX) & INBOX_TAG);
}

/*
 * Make sure that this INBOX is meant for us by checking the completion
 * value (should be NULL)
 * To be used only by the _spin version.
 */
static void bpmp_wait_for_ib_full(void)
{
	while (bpmp_ib_empty() || bpmp_read_completion())
		;
}

/* IPC with ACK poll */
static int bpmp_send_receive_spin(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz)
{
	unsigned long flags;

	if (mrq >= MRQ_LIMIT)
		return -EINVAL;

	spin_lock_irqsave(&lock, flags);

	bpmp_wait_for_ob_empty();
	bpmp_write_completion(NULL);
	bpmp_write_func_data(ob_data, ob_sz);
	bpmp_raise_mrq(mrq);

	bpmp_wait_for_ib_full();
	bpmp_read_func_data(ib_data, ib_sz);
	bpmp_empty_ib();

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

/* IPC with ACK */
static int bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz)
{
	unsigned long flags;
	struct completion work;

	if (mrq >= MRQ_LIMIT)
		return -EINVAL;

	init_completion(&work);

	spin_lock_irqsave(&lock, flags);

	bpmp_wait_for_ob_empty();
	bpmp_write_completion(&work);
	bpmp_write_func_data(ob_data, ob_sz);
	bpmp_raise_mrq(mrq);

	spin_unlock_irqrestore(&lock, flags);

	wait_for_completion(&work);
	bpmp_read_func_data(ib_data, ib_sz);
	bpmp_empty_ib();

	return 0;
}
