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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <soc/tegra/tegra_bpmp.h>
#include "bpmp.h"
#include "bpmp_abi.h"

struct mrq_handler_data {
	bpmp_mrq_handler handler;
	void *data;
};

static int cpu_mrqs[] = {
	MRQ_PING,
	MRQ_MODULE_MAIL
};

struct module_mrq {
	struct list_head link;
	uint32_t base;
	bpmp_mrq_handler handler;
	void *data;
};

static LIST_HEAD(module_mrq_list);
static struct mrq_handler_data mrq_handlers[ARRAY_SIZE(cpu_mrqs)];
static uint8_t mrq_handler_index[NR_MRQS];
static DEFINE_SPINLOCK(lock);

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

static void bpmp_mrq_ping(int code, void *data, int ch)
{
	int challenge;
	int reply;
	challenge = tegra_bpmp_mail_readl(ch, 0);
	reply = challenge  << (smp_processor_id() + 1);
	tegra_bpmp_mail_return(ch, 0, reply);
}

void bpmp_handle_mail(int mrq, int ch)
{
	struct mb_data *p;
	struct mrq_handler_data *h;
	unsigned int i;

	p = channel_area[ch].ib;
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

int bpmp_mailman_init(void)
{
	int i;
	int j;
	int r;

	for (i = 0; i < ARRAY_SIZE(cpu_mrqs); i++) {
		j = __MRQ_INDEX(cpu_mrqs[i]);
		mrq_handler_index[j] = i + 1;
	}

	r = bpmp_request_mrq(MRQ_PING, bpmp_mrq_ping, NULL);
	r = r ?: bpmp_request_mrq(MRQ_MODULE_MAIL,
			bpmp_mrq_module_mail, NULL);
	return r;
}
