/*
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

#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/platform_data/tegra_bpmp.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp_abi.h"
#include "bpmp_private.h"

#define TEGRA_ATOMICS_BASE	0x70016000
#define ATOMICS_AP0_TRIGGER	IO_ADDRESS(TEGRA_ATOMICS_BASE + 0x000)
#define ATOMICS_AP0_RESULT(id)	IO_ADDRESS(TEGRA_ATOMICS_BASE + 0xc00 + id * 4)
#define TRIGGER_ID_SHIFT	16
#define TRIGGER_CMD_GET		4

#define ICTLR_REG_BASE(irq)	IO_ADDRESS(TEGRA_PRIMARY_ICTLR_BASE + (((irq) - 32) >> 5) * 0x100)
#define ICTLR_FIR_SET(irq)	(ICTLR_REG_BASE(irq) + 0x18)
#define ICTLR_FIR_CLR(irq)	(ICTLR_REG_BASE(irq) + 0x1c)
#define FIR_BIT(irq)		(1 << ((irq) & 0x1f))

#define TIMERUS_CNTR_1US	IO_ADDRESS(TEGRA_TMR1_BASE + 0x10)

#define RES_SEMA_SHRD_SMP_STA	IO_ADDRESS(TEGRA_RES_SEMA_BASE + 0x00)
#define RES_SEMA_SHRD_SMP_SET	IO_ADDRESS(TEGRA_RES_SEMA_BASE + 0x04)
#define RES_SEMA_SHRD_SMP_CLR	IO_ADDRESS(TEGRA_RES_SEMA_BASE + 0x08)

#define THREAD_CH(i)		(CPU0_OB_CH1 + i)
#define THREAD_CH_INDEX(i)	(i - CPU0_OB_CH1)
#define PER_CPU_IB_CH(i)	(CPU0_IB_CH + i)

#define NR_THREAD_CH		4
#define ALL_FREE		0xaaaaaaaa
#define CHANNEL_TIMEOUT		USEC_PER_SEC
#define THREAD_CH_TIMEOUT	USEC_PER_SEC

#define __MRQ_INDEX(id)		((id) & ~__MRQ_ATTRS)

/*
 * IPC message format
 * @code: either an MRQ number (outgoing) or an error code (incoming)
 * @flags: message flags
 * @data: MRQ specifc args (outgoing) or result (incoming)
 */
struct mb_data {
	int code;
	int flags;
	u8 data[MSG_DATA_SZ];
};

struct mrq_handler_data {
	bpmp_mrq_handler handler;
	void *data;
};

static int cpu_mrqs[] = {
	MRQ_PING,
	MRQ_MODULE_MAIL
};

static struct mrq_handler_data mrq_handlers[ARRAY_SIZE(cpu_mrqs)];
static uint8_t mrq_handler_index[NR_MRQS];
static struct mb_data *channel_area[NR_CHANNELS];
struct completion completion[NR_THREAD_CH];
static int connected;
static DEFINE_SPINLOCK(lock);

/*
 * How the RES_SEMA_SHRD_SMP bits are interpretted
 * as token states.
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
	return __raw_readl(RES_SEMA_SHRD_SMP_STA) & CH_MASK(ch);
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
	__raw_writel(CH_MASK(ch), RES_SEMA_SHRD_SMP_CLR);
}

static void bpmp_ack_master(int ch, int flags)
{
	writel(MA_ACKD(ch), RES_SEMA_SHRD_SMP_SET);

	/*
	 * We have to violate the bit modification rule while
	 * moving from SL_QUED to MA_FREE (DO_ACK not set) so that
	 * the channel won't be in ACKD state forever.
	 */
	if (!(flags & DO_ACK))
		writel(MA_ACKD(ch) ^ MA_FREE(ch), RES_SEMA_SHRD_SMP_CLR);
}

/* MA_ACKD to MA_FREE */
static void bpmp_free_master(int ch)
{
	writel(MA_ACKD(ch) ^ MA_FREE(ch), RES_SEMA_SHRD_SMP_CLR);
}

static void bpmp_ring_doorbell(void)
{
	writel(FIR_BIT(CPU_OB_IRQ), ICTLR_FIR_SET(CPU_OB_IRQ));
}

uint32_t tegra_bpmp_mail_readl(int ch, int offset)
{
	u32 *data = (u32 *)(channel_area[ch]->data + offset);
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

	p = channel_area[ch];
	p->code = code;
	memcpy(p->data, data, sz);

	flags = p->flags;
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

static void bpmp_mrq_ping(int code, void *data, int ch)
{
	int challenge;
	int reply;
	challenge = tegra_bpmp_mail_readl(ch, 0);
	reply = challenge  << (smp_processor_id() + 1);
	tegra_bpmp_mail_return(ch, 0, reply);
}

static void bpmp_dispatch_mrq(int ch)
{
	struct mb_data *p;
	struct mrq_handler_data *h;
	unsigned int i;

	p = channel_area[ch];
	i = __MRQ_INDEX(p->code);

	if (i >= NR_MRQS)
		goto err_out;

	if (!mrq_handler_index[i])
		goto err_out;

	h = mrq_handlers + mrq_handler_index[i] - 1;
	if (!h->handler)
		goto err_out;

	h->handler(p->code, h->data, ch);
	return;

err_out:
	tegra_bpmp_mail_return(ch, -EINVAL, 0);
}

static struct completion *bpmp_completion_obj(int ch)
{
	int i = ch - CPU0_OB_CH1;
	if (i < 0 || i >= NR_THREAD_CH)
		return NULL;
	return completion + i;
}

static void bpmp_signal_thread(int ch)
{
	struct mb_data *p = channel_area[ch];
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

static void bpmp_ack_doorbell(int irq)
{
	writel(FIR_BIT(irq), ICTLR_FIR_CLR(irq));
}

irqreturn_t bpmp_inbox_irq(int irq, void *data)
{
	unsigned long flags;
	int ch;
	int i;

	if (!connected)
		return IRQ_HANDLED;

	ch = (long)data;
	bpmp_ack_doorbell(irq);

	if (bpmp_slave_signalled(ch))
		bpmp_dispatch_mrq(ch);

	spin_lock_irqsave(&lock, flags);

	for (i = 0; i < NR_THREAD_CH && to_complete; i++) {
		ch = THREAD_CH(i);
		if (bpmp_master_acked(ch) && (to_complete & 1 << ch)) {
			to_complete &= ~(1 << ch);
			bpmp_signal_thread(ch);
		}
	}

	spin_unlock_irqrestore(&lock, flags);

	return IRQ_HANDLED;
}

static unsigned int usec_counter(void)
{
	return __raw_readl(TIMERUS_CNTR_1US);
}

static int bpmp_wait_master_free(int ch)
{
	unsigned int start;

	if (bpmp_master_free(ch))
		return 0;

	start = usec_counter();

	while (usec_counter() - start < CHANNEL_TIMEOUT) {
		if (bpmp_master_free(ch))
			return 0;
	}

	return -ETIMEDOUT;
}

static void __bpmp_write_ch(int ch, int mrq, int flags, void *data, int sz)
{
	struct mb_data *p = channel_area[ch];

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

static int bpmp_ob_channel(void)
{
	return smp_processor_id() + CPU0_OB_CH0;
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
	*ch = THREAD_CH(i);

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
	struct mb_data *p = channel_area[ch];
	if (data)
		memcpy(data, p->data, sz);
	bpmp_free_master(ch);
	return p->code;
}

static int bpmp_read_ch(int ch, void *data, int sz)
{
	unsigned long flags;
	int r;

	spin_lock_irqsave(&lock, flags);
	r = __bpmp_read_ch(ch, data, sz);
	tch_free |= (1 << THREAD_CH_INDEX(ch));
	spin_unlock_irqrestore(&lock, flags);

	up(&tch_sem);
	return r;
}

static int bpmp_wait_ack(int ch)
{
	unsigned int start = usec_counter();

	while (usec_counter() - start < CHANNEL_TIMEOUT) {
		if (bpmp_master_acked(ch))
			return 0;
	}

	return -ETIMEDOUT;
}

/* One-way: do not wait for ACK */
int bpmp_post(int mrq, void *data, int sz)
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

/* Atomic */
int bpmp_rpc(int mrq, void *ob_data, int ob_sz, void *ib_data, int ib_sz)
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

/* Non atomic */
int bpmp_threaded_rpc(int mrq, void *ob_data, int ob_sz,
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

int __bpmp_rpc(int mrq, void *ob_data, int ob_sz, void *ib_data, int ib_sz)
{
	if (irqs_disabled())
		return bpmp_rpc(mrq, ob_data, ob_sz, ib_data, ib_sz);

	return bpmp_threaded_rpc(mrq, ob_data, ob_sz, ib_data, ib_sz);
}

int tegra_bpmp_rpc(int mrq, void *ob_data, int ob_sz, void *ib_data, int ib_sz)
{
	if (!(mrq & __MRQ_PUBLIC))
		return -EPERM;

	return __bpmp_rpc(mrq, ob_data, ob_sz, ib_data, ib_sz);
}
EXPORT_SYMBOL(tegra_bpmp_rpc);

static int bpmp_request_mrq(int mrq, bpmp_mrq_handler handler, void *data)
{
	struct mrq_handler_data *ph;
	unsigned long f;
	unsigned int i;

	i = __MRQ_INDEX(mrq);
	if (i >= NR_MRQS)
		return -EINVAL;

	if (!mrq_handler_index[i])
		return -EINVAL;

	spin_lock_irqsave(&lock, f);

	ph = mrq_handlers + mrq_handler_index[i] - 1;
	if (ph->handler) {
		spin_unlock_irqrestore(&lock, f);
		return -EEXIST;
	}

	ph->handler = handler;
	ph->data = data;

	spin_unlock_irqrestore(&lock, f);
	return 0;
}

static LIST_HEAD(module_mrq_list);

struct module_mrq {
	struct list_head link;
	uint32_t base;
	bpmp_mrq_handler handler;
	void *data;
};

static struct module_mrq *bpmp_find_module_mrq(uint32_t module_base)
{
	struct module_mrq *item;

	list_for_each_entry(item, &module_mrq_list, link) {
		if (item->base == module_base)
			return item;
	}

	return NULL;
}

static void bpmp_mrq_module_mail(int code, void *data, int ch)
{
	unsigned long flags;
	struct module_mrq *item;
	uint32_t base;

	base = tegra_bpmp_mail_readl(ch, 0);

	spin_lock_irqsave(&lock, flags);
	item = bpmp_find_module_mrq(base);
	if (item)
		item->handler(code, item->data, ch);
	else
		tegra_bpmp_mail_return(ch, -ENODEV, 0);
	spin_unlock_irqrestore(&lock, flags);
}

int tegra_bpmp_request_module_mrq(uint32_t module_base,
		bpmp_mrq_handler handler, void *data)
{
	struct module_mrq *item;
	unsigned long flags;

	if (!module_base || !handler)
		return -EINVAL;

	item = kmalloc(sizeof(*item), GFP_KERNEL);
	if (!item)
		return -ENOMEM;

	item->base = module_base;
	item->handler = handler;
	item->data = data;

	spin_lock_irqsave(&lock, flags);

	if (bpmp_find_module_mrq(module_base)) {
		spin_unlock_irqrestore(&lock, flags);
		kfree(item);
		return -EEXIST;
	}

	list_add(&item->link, &module_mrq_list);
	spin_unlock_irqrestore(&lock, flags);
	return 0;
}
EXPORT_SYMBOL(tegra_bpmp_request_module_mrq);

void tegra_bpmp_cancel_module_mrq(uint32_t module_base)
{
	struct module_mrq *item;
	unsigned long flags;

	if (!module_base)
		return;

	spin_lock_irqsave(&lock, flags);
	item = bpmp_find_module_mrq(module_base);
	if (item)
		list_del(&item->link);
	spin_unlock_irqrestore(&lock, flags);

	kfree(item);
}
EXPORT_SYMBOL(tegra_bpmp_cancel_module_mrq);

static int bpmp_mrq_init(void)
{
	int i;
	int j;
	int r;

	for (i = 0; i < ARRAY_SIZE(cpu_mrqs); i++) {
		j = __MRQ_INDEX(cpu_mrqs[i]);
		mrq_handler_index[j] = i + 1;
	}

	r = bpmp_request_mrq(MRQ_PING, bpmp_mrq_ping, NULL);
	if (r)
		return r;

	return bpmp_request_mrq(MRQ_MODULE_MAIL, bpmp_mrq_module_mail, NULL);
}

static void bpmp_init_completion(void)
{
	int i;
	for (i = 0; i < NR_THREAD_CH; i++)
		init_completion(completion + i);
}

static int cpu_irqs[] = { CPU0_IB_IRQ, CPU1_IB_IRQ, CPU2_IB_IRQ, CPU3_IB_IRQ };

static void bpmp_irq_set_affinity(int cpu)
{
	int nr_cpus = num_present_cpus();
	int r;
	int i;

	for (i = cpu; i < ARRAY_SIZE(cpu_irqs); i += nr_cpus) {
		r = irq_set_affinity(cpu_irqs[i], cpumask_of(cpu));
		WARN_ON(r);
	}
}

static void bpmp_irq_clr_affinity(int cpu)
{
	int nr_cpus = num_present_cpus();
	int new_cpu;
	int r;
	int i;

	for (i = cpu; i < ARRAY_SIZE(cpu_irqs); i += nr_cpus) {
		new_cpu = cpumask_any_but(cpu_online_mask, cpu);
		r = irq_set_affinity(cpu_irqs[i], cpumask_of(new_cpu));
		WARN_ON(r);
	}
}

/*
 * When a CPU is being hot unplugged, the incoming
 * doorbell irqs must be moved to another CPU
 */
static int bpmp_cpu_notify(struct notifier_block *nb, unsigned long action,
		void *data)
{
	int cpu = (long)data;

	switch (action) {
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		bpmp_irq_clr_affinity(cpu);
		break;
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		bpmp_irq_set_affinity(cpu);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block bpmp_cpu_nb = {
	.notifier_call = bpmp_cpu_notify
};

static int bpmp_init_irq(struct platform_device *pdev)
{
	const char *n = dev_name(&pdev->dev);
	long ch;
	int r;
	int i;

	for (i = 0; i < ARRAY_SIZE(cpu_irqs); i++) {
		ch = PER_CPU_IB_CH(i);
		r = request_irq(cpu_irqs[i], bpmp_inbox_irq, 0, n, (void *)ch);
		if (r)
			return r;
	}

	r = register_cpu_notifier(&bpmp_cpu_nb);
	if (r)
		return r;

	for_each_present_cpu(i)
		bpmp_irq_set_affinity(i);

	return 0;
}

/* Channel area is setup by BPMP before signalling handshake */
static void *bpmp_channel_area(int ch)
{
	u32 a;
	writel(ch << TRIGGER_ID_SHIFT | TRIGGER_CMD_GET, ATOMICS_AP0_TRIGGER);
	a = readl(ATOMICS_AP0_RESULT(ch));
	return a ? IO_ADDRESS(a) : NULL;
}

static int bpmp_connect(void)
{
	int i;

	if (connected)
		return 0;

	/* handshake */
	if (!readl(RES_SEMA_SHRD_SMP_STA))
		return -ENODEV;

	for (i = 0; i < NR_CHANNELS; i++) {
		channel_area[i] = bpmp_channel_area(i);
		if (!channel_area[i])
			return -EFAULT;
	}

	connected = 1;

	return 0;
}

void bpmp_detach(void)
{
	int i;

	connected = 0;
	writel(0xffffffff, RES_SEMA_SHRD_SMP_CLR);

	for (i = 0; i < NR_CHANNELS; i++)
		channel_area[i] = 0;
}

int bpmp_attach(void)
{
	int i;

	WARN_ON(connected);

	for (i = 0; i < MSEC_PER_SEC * 60; i += 20) {
		if (!bpmp_connect())
			return 0;
		msleep(20);
	}

	return -ETIMEDOUT;
}

int bpmp_ipc_init(struct platform_device *pdev)
{
	int r;

	bpmp_init_completion();
	r = bpmp_init_irq(pdev);
	if (r) {
		dev_err(&pdev->dev, "irq init failed (%d)\n", r);
		return r;
	}

	r = bpmp_mrq_init();
	if (r) {
		dev_err(&pdev->dev, "mrq init failed (%d)\n", r);
		return r;
	}

	r = bpmp_connect();
	dev_info(&pdev->dev, "bpmp_connect returned %d\n", r);

	/* Ignore connection failures - bpmp can be loaded post boot
	 * TODO: remove this after POR bootflow is ready
	 */
	return 0;
}

void tegra_bpmp_init_early(void)
{
	bpmp_connect();
}
