/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <soc/tegra/tegra_bpmp.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp.h"

#define DO_ACK			(1 << 0)
#define RING_DOORBELL		(1 << 1)

/* FIXME: reduce this to 1 sec (WAR for linsim irq bug) */
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#define CHANNEL_TIMEOUT		(60 * USEC_PER_SEC)
#define THREAD_CH_TIMEOUT	(600 * USEC_PER_SEC)
#else
#define CHANNEL_TIMEOUT		USEC_PER_SEC
#define THREAD_CH_TIMEOUT	USEC_PER_SEC
#endif

struct channel_data channel_area[NR_CHANNELS];
static struct completion completion[NR_THREAD_CH];
int connected;
static DEFINE_SPINLOCK(lock);

/*
 * How the token bits are interpretted
 *
 * SL_SIGL (b00): slave ch in signalled state
 * SL_QUED (b01): slave ch is in queue
 * MA_FREE (b10): master ch is free
 * MA_ACKD (b11): master ch is acked
 *
 * Ideally, the slave should only set bits while the
 * master do only clear them. But there is an exception -
 * see bpmp_ack_master()
 */
#define CH_MASK(ch)	(0x3 << ((ch) * 2))
#define SL_SIGL(ch)	(0x0 << ((ch) * 2))
#define SL_QUED(ch)	(0x1 << ((ch) * 2))
#define MA_FREE(ch)	(0x2 << ((ch) * 2))
#define MA_ACKD(ch)	(0x3 << ((ch) * 2))

static u32 bpmp_ch_sta(int ch)
{
	return bpmp_mail_token() & CH_MASK(ch);
}

static bool bpmp_master_free(int ch)
{
	return bpmp_ch_sta(ch) == MA_FREE(ch);
}

static bool bpmp_slave_signalled(int ch)
{
	return bpmp_ch_sta(ch) == SL_SIGL(ch);
}

static bool bpmp_master_acked(int ch)
{
	return bpmp_ch_sta(ch) == MA_ACKD(ch);
}

static void bpmp_signal_slave(int ch)
{
	bpmp_mail_token_clr(CH_MASK(ch));
}

static void bpmp_ack_master(int ch, int flags)
{
	bpmp_mail_token_set(MA_ACKD(ch));

	/*
	 * We have to violate the bit modification rule while
	 * moving from SL_QUED to MA_FREE (DO_ACK not set) so that
	 * the channel won't be in ACKD state forever.
	 */
	if (!(flags & DO_ACK))
		bpmp_mail_token_clr(MA_ACKD(ch) ^ MA_FREE(ch));
}

/* MA_ACKD to MA_FREE */
static void bpmp_free_master(int ch)
{
	bpmp_mail_token_clr(MA_ACKD(ch) ^ MA_FREE(ch));
}

uint32_t tegra_bpmp_mail_readl(int ch, int offset)
{
	u32 *data = (u32 *)(channel_area[ch].ib->data + offset);
	return *data;
}
EXPORT_SYMBOL(tegra_bpmp_mail_readl);

void tegra_bpmp_mail_return_data(int ch, int code, void *data, int sz)
{
	struct mb_data *p;
	int flags;

	if (sz > MSG_DATA_SZ) {
		WARN_ON(1);
		return;
	}

	p = channel_area[ch].ob;
	p->code = code;
	memcpy(p->data, data, sz);

	flags = channel_area[ch].ib->flags;
	bpmp_ack_master(ch, flags);
	if (flags & RING_DOORBELL)
		bpmp_ring_doorbell();
}
EXPORT_SYMBOL(tegra_bpmp_mail_return_data);

void tegra_bpmp_mail_return(int ch, int code, int v)
{
	tegra_bpmp_mail_return_data(ch, code, &v, sizeof(v));
}
EXPORT_SYMBOL(tegra_bpmp_mail_return);

static struct completion *bpmp_completion_obj(int ch)
{
	int i = bpmp_thread_ch_index(ch);
	return i < 0 ? NULL : completion + i;
}

static void bpmp_signal_thread(int ch)
{
	struct mb_data *p = channel_area[ch].ib;
	struct completion *w;

	if (!(p->flags & RING_DOORBELL))
		return;

	w = bpmp_completion_obj(ch);
	if (!w) {
		WARN_ON(1);
		return;
	}

	complete(w);
}

/* bit mask of thread channels waiting for completion */
static unsigned int to_complete;

void bpmp_handle_irq(int ch)
{
	int i;

	if (!connected)
		return;

	if (bpmp_slave_signalled(ch))
		bpmp_handle_mail(channel_area[ch].ib->code, ch);

	spin_lock(&lock);

	for (i = 0; i < NR_THREAD_CH && to_complete; i++) {
		ch = bpmp_thread_ch(i);
		if (bpmp_master_acked(ch) && (to_complete & 1 << ch)) {
			to_complete &= ~(1 << ch);
			bpmp_signal_thread(ch);
		}
	}

	spin_unlock(&lock);
}

static int bpmp_wait_master_free(int ch)
{
	ktime_t start;

	if (bpmp_master_free(ch))
		return 0;

	start = ktime_get();

	while (ktime_us_delta(ktime_get(), start) < CHANNEL_TIMEOUT) {
		if (bpmp_master_free(ch))
			return 0;
	}

	return -ETIMEDOUT;
}

static void __bpmp_write_ch(int ch, int mrq, int flags, void *data, int sz)
{
	struct mb_data *p = channel_area[ch].ib;

	p->code = mrq;
	p->flags = flags;
	if (data)
		memcpy(p->data, data, sz);
	bpmp_signal_slave(ch);
}

static int bpmp_write_ch(int ch, int mrq, int flags, void *data, int sz)
{
	int r;

	r = bpmp_wait_master_free(ch);
	if (r)
		return r;

	__bpmp_write_ch(ch, mrq, flags, data, sz);
	return 0;
}

static int tch_free = (1 << NR_THREAD_CH) - 1;
static struct semaphore tch_sem =
		__SEMAPHORE_INITIALIZER(tch_sem, NR_THREAD_CH);

static int bpmp_write_threaded_ch(int *ch, int mrq, void *data, int sz)
{
	unsigned long flags;
	int ret;
	int i;

	ret = down_timeout(&tch_sem, usecs_to_jiffies(THREAD_CH_TIMEOUT));
	if (ret)
		return ret;

	spin_lock_irqsave(&lock, flags);

	i = __ffs(tch_free);
	*ch = bpmp_thread_ch(i);

	ret = bpmp_master_free(*ch) ? 0 : -EFAULT;
	if (!ret) {
		tch_free &= ~(1 << i);
		__bpmp_write_ch(*ch, mrq, DO_ACK | RING_DOORBELL, data, sz);
		to_complete |= 1 << *ch;
	}

	spin_unlock_irqrestore(&lock, flags);
	return ret;
}

static int __bpmp_read_ch(int ch, void *data, int sz)
{
	struct mb_data *p = channel_area[ch].ob;
	if (data)
		memcpy(data, p->data, sz);
	bpmp_free_master(ch);
	return p->code;
}

static int bpmp_read_ch(int ch, void *data, int sz)
{
	unsigned long flags;
	int tchi;
	int r;

	tchi = bpmp_thread_ch_index(ch);

	spin_lock_irqsave(&lock, flags);
	r = __bpmp_read_ch(ch, data, sz);
	tch_free |= (1 << tchi);
	spin_unlock_irqrestore(&lock, flags);

	up(&tch_sem);
	return r;
}

static int bpmp_wait_ack(int ch)
{
	ktime_t start;

	if (bpmp_master_acked(ch))
		return 0;

	start = ktime_get();

	while (ktime_us_delta(ktime_get(), start) < CHANNEL_TIMEOUT) {
		if (bpmp_master_acked(ch))
			return 0;
	}

	return -ETIMEDOUT;
}

int tegra_bpmp_send(int mrq, void *data, int sz)
{
	unsigned long flags;
	int ch;
	int r;

	if (!connected)
		return -ENODEV;

	local_irq_save(flags);

	ch = bpmp_ob_channel();
	r = bpmp_write_ch(ch, mrq, 0, data, sz);
	if (!r)
		bpmp_ring_doorbell();

	local_irq_restore(flags);
	return r;
}
EXPORT_SYMBOL(tegra_bpmp_send);

/* should be called with local irqs disabled */
int tegra_bpmp_send_receive_atomic(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz)
{
	int ch;
	int r;

	if (!connected)
		return -ENODEV;

	ch = bpmp_ob_channel();
	r = bpmp_write_ch(ch, mrq, DO_ACK, ob_data, ob_sz);
	if (r)
		return r;

	bpmp_ring_doorbell();
	r = bpmp_wait_ack(ch);
	if (r)
		return r;

	return __bpmp_read_ch(ch, ib_data, ib_sz);
}
EXPORT_SYMBOL(tegra_bpmp_send_receive_atomic);

int tegra_bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz)
{
	struct completion *w;
	unsigned long timeout;
	int ch;
	int r;

	if (!connected)
		return -ENODEV;

	r = bpmp_write_threaded_ch(&ch, mrq, ob_data, ob_sz);
	if (r)
		return r;

	bpmp_ring_doorbell();
	w = bpmp_completion_obj(ch);
	timeout = usecs_to_jiffies(THREAD_CH_TIMEOUT);
	if (!wait_for_completion_timeout(w, timeout))
		return -ETIMEDOUT;

	return bpmp_read_ch(ch, ib_data, ib_sz);
}
EXPORT_SYMBOL(tegra_bpmp_send_receive);

static void bpmp_init_completion(void)
{
	int i;
	for (i = 0; i < NR_THREAD_CH; i++)
		init_completion(completion + i);
}

int bpmp_mail_init(struct platform_device *pdev)
{
	int r;

	bpmp_init_completion();
	r = bpmp_init_irq(pdev);
	if (r) {
		dev_err(&pdev->dev, "irq init failed (%d)\n", r);
		return r;
	}

	r = bpmp_mailman_init();
	if (r) {
		dev_err(&pdev->dev, "mailman init failed (%d)\n", r);
		return r;
	}

	r = bpmp_connect();
	dev_info(&pdev->dev, "bpmp_connect returned %d\n", r);
	return r;
}

void tegra_bpmp_init_early(void)
{
	bpmp_connect();
}
