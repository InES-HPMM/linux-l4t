/*
 *  linux/arch/arm/kernel/fiq.c
 *
 *  Copyright (C) 1998 Russell King
 *  Copyright (C) 1998, 1999 Phil Blundell
 *
 *  FIQ support written by Philip Blundell <philb@gnu.org>, 1998.
 *
 *  FIQ support re-written by Russell King to be more generic
 *
 * We now properly support a method by which the FIQ handlers can
 * be stacked onto the vector.  We still do not support sharing
 * the FIQ vector itself.
 *
 * Operation is as follows:
 *  1. Owner A claims FIQ:
 *     - default_fiq relinquishes control.
 *  2. Owner A:
 *     - inserts code.
 *     - sets any registers,
 *     - enables FIQ.
 *  3. Owner B claims FIQ:
 *     - if owner A has a relinquish function.
 *       - disable FIQs.
 *       - saves any registers.
 *       - returns zero.
 *  4. Owner B:
 *     - inserts code.
 *     - sets any registers,
 *     - enables FIQ.
 *  5. Owner B releases FIQ:
 *     - Owner A is asked to reacquire FIQ:
 *	 - inserts code.
 *	 - restores saved registers.
 *	 - enables FIQ.
 *  6. Goto 3
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/radix-tree.h>
#include <linux/slab.h>

#include <asm/cacheflush.h>
#include <asm/cp15.h>
#include <asm/fiq.h>
#include <asm/irq.h>
#include <asm/traps.h>

#define FIQ_OFFSET ({					\
		extern void *vector_fiq_offset;		\
		(unsigned)&vector_fiq_offset;		\
	})

struct fiq_data {
	struct fiq_chip *fiq_chip;
	struct irq_data *irq_data;
};

static unsigned long no_fiq_insn;
static int fiq_start = -1;
static RADIX_TREE(fiq_data_tree, GFP_KERNEL);
static DEFINE_MUTEX(fiq_data_mutex);

/* Default reacquire function
 * - we always relinquish FIQ control
 * - we always reacquire FIQ control
 */
static int fiq_def_op(void *ref, int relinquish)
{
	if (!relinquish)
		set_fiq_handler(&no_fiq_insn, sizeof(no_fiq_insn));

	return 0;
}

static struct fiq_handler default_owner = {
	.name	= "default",
	.fiq_op = fiq_def_op,
};

static struct fiq_handler *current_fiq = &default_owner;

int show_fiq_list(struct seq_file *p, int prec)
{
	if (current_fiq != &default_owner)
		seq_printf(p, "%*s:              %s\n", prec, "FIQ",
			current_fiq->name);

	return 0;
}

void set_fiq_handler(void *start, unsigned int length)
{
	void *base = vectors_page;
	unsigned offset = FIQ_OFFSET;

	memcpy(base + offset, start, length);
	if (!cache_is_vipt_nonaliasing())
		flush_icache_range((unsigned long)base + offset, offset +
				   length);
	flush_icache_range(0xffff0000 + offset, 0xffff0000 + offset + length);
}

int claim_fiq(struct fiq_handler *f)
{
	int ret = 0;

	if (current_fiq) {
		ret = -EBUSY;

		if (current_fiq->fiq_op != NULL)
			ret = current_fiq->fiq_op(current_fiq->dev_id, 1);
	}

	if (!ret) {
		f->next = current_fiq;
		current_fiq = f;
	}

	return ret;
}

void release_fiq(struct fiq_handler *f)
{
	if (current_fiq != f) {
		printk(KERN_ERR "%s FIQ trying to release %s FIQ\n",
		       f->name, current_fiq->name);
		dump_stack();
		return;
	}

	do
		current_fiq = current_fiq->next;
	while (current_fiq->fiq_op(current_fiq->dev_id, 0));
}

static struct fiq_data *lookup_fiq_data(int fiq)
{
	struct fiq_data *data;

	rcu_read_lock();
	data = radix_tree_lookup(&fiq_data_tree, fiq);
	rcu_read_unlock();

	return data;
}

void enable_fiq(int fiq)
{
	struct fiq_data *data = lookup_fiq_data(fiq);

	if (data) {
		if (data->fiq_chip->fiq_enable)
			data->fiq_chip->fiq_enable(data->irq_data);
		enable_irq(fiq);
		return;
	}

	if (WARN_ON(fiq_start == -1))
		return;

	enable_irq(fiq + fiq_start);
}

void disable_fiq(int fiq)
{
	struct fiq_data *data = lookup_fiq_data(fiq);

	if (data) {
		if (data->fiq_chip->fiq_disable)
			data->fiq_chip->fiq_disable(data->irq_data);
		disable_irq(fiq);
		return;
	}

	if (WARN_ON(fiq_start == -1))
		return;

	disable_irq(fiq + fiq_start);
}

int ack_fiq(int fiq)
{
	struct fiq_data *data = lookup_fiq_data(fiq);

	if (data && data->fiq_chip->fiq_ack)
		return data->fiq_chip->fiq_ack(data->irq_data);

	return fiq;
}

void eoi_fiq(int fiq)
{
	struct fiq_data *data = lookup_fiq_data(fiq);

	if (data && data->fiq_chip->fiq_eoi)
		data->fiq_chip->fiq_eoi(data->irq_data);
}
EXPORT_SYMBOL(eoi_fiq);

bool has_fiq(int fiq)
{
	struct fiq_data *data = lookup_fiq_data(fiq);

	if (data)
		return true;

	if (fiq_start == -1)
		return false;

	return fiq >= fiq_start;
}
EXPORT_SYMBOL(has_fiq);

EXPORT_SYMBOL(set_fiq_handler);
EXPORT_SYMBOL(__set_fiq_regs);	/* defined in fiqasm.S */
EXPORT_SYMBOL(__get_fiq_regs);	/* defined in fiqasm.S */
EXPORT_SYMBOL(claim_fiq);
EXPORT_SYMBOL(release_fiq);
EXPORT_SYMBOL(enable_fiq);
EXPORT_SYMBOL(disable_fiq);

/*
 * Add a mapping from a Linux irq to the fiq data.
 */
void fiq_register_mapping(int irq, struct fiq_chip *chip)
{
	struct fiq_data *fiq_data = NULL;
	int res;

	/* fiq_register_mapping can't be mixed with init_FIQ */
	BUG_ON(fiq_start != -1);

	fiq_data = kmalloc(sizeof(*fiq_data), GFP_KERNEL);
	if (!fiq_data)
		goto err;

	fiq_data->fiq_chip = chip;
	fiq_data->irq_data = irq_get_irq_data(irq);
	BUG_ON(!fiq_data->irq_data);

	mutex_lock(&fiq_data_mutex);
	res = radix_tree_insert(&fiq_data_tree, irq, fiq_data);
	mutex_unlock(&fiq_data_mutex);
	if (res)
		goto err;

	return;

err:
	kfree(fiq_data);
	pr_err("fiq: Cannot register mapping %d\n", irq);
}

/*
 * Set the offset between normal IRQs and their FIQ shadows.
 */
void __init init_FIQ(int start)
{
	fiq_start = start;
}

static int __init init_default_fiq_handler(void)
{
	unsigned offset = FIQ_OFFSET;
	no_fiq_insn = *(unsigned long *)(0xffff0000 + offset);
	return 0;
}
pure_initcall(init_default_fiq_handler);
