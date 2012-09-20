/*
 * arch/arm/mach-tegra/isomgr.c
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/printk.h>
#include <linux/clk.h>
#include <mach/isomgr.h>

#define ISOMGR_SYSFS_VERSION 0	/* increment on change */

/* get minimum MC frequency for client that can support this BW and LT */
static inline u32 mc_min_freq(u32 ubw, u32 ult) /* in KB/sec and usec */
{
	/* should take MC efficiency into account (order of 75-80%) */
	/* ult==0 means ignore LT (effectively infinite) */
	return 1; /* !!!FIXME!!! this function should reside in MC */
	/* return value in KHz - 0 means impossible/failure */
}

/* get dvfs switch latency for client requiring this frequency */
static inline u32 mc_dvfs_latency(u32 ufreq)
{
	return 4; /* !!!FIXME!!! this function should reside in MC */
	/* return value in usec */
}

static struct isomgr_client {
	bool busy;		/* already registered */
	s32 dedi_bw;		/* BW dedicated to this client	(KB/sec) */
	s32 rsvd_bw;		/* BW reserved for this client	(KB/sec) */
	s32 real_bw;		/* BW realized for this client	(KB/sec) */
	s32 lti;		/* Client spec'd Latency Tolerance (usec) */
	s32 lto;		/* MC calculated Latency Tolerance (usec) */
	s32 rsvd_mf;		/* reserved minimum freq in support of LT */
	s32 real_mf;		/* realized minimum freq in support of LT */
	s32 alloc_delta;	/* amt to increment allocation	(KB/sec) */
	s32 avail_delta;	/* amt to decrement availability(KB/sec) */
	bool sleeping;		/* sleeping until avail_delta is met */
	tegra_isomgr_renegotiate renegotiate;	/* ask client to renegotiate */
	void *priv;		/* client driver's private data */
	struct completion cmpl;	/* so we can sleep waiting for delta BW */
#ifdef CONFIG_TEGRA_ISOMGR_SYSFS
	struct kobject *client_kobj;
	struct isomgr_client_attrs {
		struct kobj_attribute busy;
		struct kobj_attribute dedi_bw;
		struct kobj_attribute rsvd_bw;
		struct kobj_attribute real_bw;
		struct kobj_attribute need_bw;
		struct kobj_attribute lti;
		struct kobj_attribute lto;
		struct kobj_attribute rsvd_mf;
		struct kobj_attribute real_mf;
	} client_attrs;
#ifdef CONFIG_TEGRA_ISOMGR_DEBUG
	u32 __arg0;		/* args to inject stimulus */
	u32 __arg1;		/* args to inject stimulus */
	u32 __arg2;		/* args to inject stimulus */
	u32 reneg_seqnum;	/* renegotiation() callback count */
	u32 dvfs_latency;	/* retained value (in usec) */
	struct kobject *debug_kobj;
	struct isomgr_debug_attrs {
		struct kobj_attribute __arg0;
		struct kobj_attribute __arg1;
		struct kobj_attribute __arg2;
		struct kobj_attribute _register;
		struct kobj_attribute _unregister;
		struct kobj_attribute _reserve;
		struct kobj_attribute _realize;
		struct kobj_attribute reneg_seqnum;
		struct kobj_attribute dvfs_latency;
	} debug_attrs;
#endif /* CONFIG_TEGRA_ISOMGR_DEBUG */
#endif /* CONFIG_TEGRA_ISOMGR_SYSFS */
} isomgr_clients[ISOMGR_CLIENT_COUNT];

static struct {
	struct mutex lock;		/* to lock ALL isomgr state */
	struct task_struct *task;	/* check reentrant/mismatched locks */
	struct clk *emc_clk;		/* emc clock handle */
	s32 bw_mf;			/* min freq to support aggregate bw */
	s32 lt_mf;			/* min freq to support worst LT */
	s32 iso_mf;			/* min freq to support ISO clients */
	s32 alloc_bw;			/* globally allocated MC BW */
	s32 avail_bw;			/* globally available MC BW */
	s32 sleep_bw;			/* total BW needed by sleepers */
	s32 dedi_bw;			/* total BW 'dedicated' to clients */
	u32 max_iso_bw;			/* max ISO BW MC can accomodate */
	u32 seqid;			/* incrementing seqid per change */
	struct kobject *kobj;		/* for sysfs linkage */
} isomgr = {
	.max_iso_bw = CONFIG_TEGRA_ISOMGR_POOL_KB_PER_SEC,
	.avail_bw = CONFIG_TEGRA_ISOMGR_POOL_KB_PER_SEC,
};

static inline void isomgr_lock(void)
{
	BUG_ON(isomgr.task == current); /* disallow rentrance, avoid deadlock */
	mutex_lock(&isomgr.lock);
	isomgr.task = current;
}

static inline void isomgr_unlock(void)
{
	BUG_ON(isomgr.task != current); /* detact mismatched calls */
	isomgr.task = 0;
	mutex_unlock(&isomgr.lock);
}

static inline void isomgr_scavenge(void)
{
	struct isomgr_client *cp;
	int i;

	/* ask flexible clients above dedicated BW levels to pitch in */
	for (i = 0; i < ISOMGR_CLIENT_COUNT; ++i) {
		cp = &isomgr_clients[i];
		if (cp->busy && cp->renegotiate)
			if ((cp->real_bw > cp->dedi_bw) ||
			    (cp->rsvd_bw > cp->dedi_bw))
				cp->renegotiate(cp->priv);
	}
}

static inline void isomgr_scatter(void)
{
	struct isomgr_client *cp;
	int i;

	for (i = 0; i < ISOMGR_CLIENT_COUNT; ++i) {
		cp = &isomgr_clients[i];
		if (cp->busy) {
			if (cp->sleeping) {
				if (cp->avail_delta <= isomgr.avail_bw)
					complete(&cp->cmpl); /* awaken */
			} else if (cp->renegotiate) {
				cp->renegotiate(cp->priv); /* poke flexibles */
			}
		}
	}
}

/* register an ISO BW client */
tegra_isomgr_handle tegra_isomgr_register(int client, u32 udedi_bw,
	tegra_isomgr_renegotiate renegotiate, void *priv)
{
	struct isomgr_client *cp = &isomgr_clients[client];
	s32 dedi_bw = udedi_bw;
	int err;

	if (unlikely((client < 0) || (client >= ISOMGR_CLIENT_COUNT))) {
		pr_err("tegra_isomgr_register: invalid client (%d)\n", client);
		return 0;
	}

	isomgr_lock();
	if (unlikely(udedi_bw > (isomgr.max_iso_bw - isomgr.dedi_bw))) {
		pr_err("tegra_isomgr_register: invalid bandwidth (%u)\n",
			udedi_bw);
		isomgr_unlock();
		return 0;
	}

	if (unlikely(cp->busy)) {
		pr_err("tegra_isomgr_register: client (%d) already registered",
		       client);
		isomgr_unlock();
		return 0;
	}
	cp->busy = true;

	/* arrange for dedicated ISO BW if necessary (e.g. disp min config) */
	/* regarding dedi_bw, check for obvious fail and scavenge */
	if (dedi_bw <= isomgr.avail_bw) {
		isomgr.sleep_bw += dedi_bw;
		isomgr_scavenge();
		isomgr.sleep_bw -= dedi_bw;
	}
retry:
	if (dedi_bw <= isomgr.avail_bw) {
		/* got it! */
		cp->dedi_bw = dedi_bw;
		cp->renegotiate = renegotiate;
		cp->priv = priv;
		isomgr.avail_bw -= dedi_bw;
		isomgr.dedi_bw += dedi_bw;
	} else {
		cp->sleeping = true;
		isomgr.sleep_bw += dedi_bw;
		cp->avail_delta = dedi_bw;
		++isomgr.seqid;
		isomgr_unlock();
		err = wait_for_completion_killable(&cp->cmpl);
		isomgr_lock();
		cp->avail_delta = 0;
		isomgr.sleep_bw -= dedi_bw;
		cp->sleeping = false;
		if (unlikely(err != -EINTR))
			goto retry;
		cp->busy = false;
		cp = 0;
	}
	++isomgr.seqid;
	isomgr_unlock();
	return (tegra_isomgr_handle)cp;
}
EXPORT_SYMBOL(tegra_isomgr_register);

/* unregister an ISO BW client */
void tegra_isomgr_unregister(tegra_isomgr_handle handle)
{
	struct isomgr_client *cp = (struct isomgr_client *)handle;
	int client = cp - &isomgr_clients[0];

	if (unlikely((client < 0) || (client >= ISOMGR_CLIENT_COUNT) ||
		     !cp->busy)) {
		pr_err("tegra_isomgr_unregister: bad handle (%p)\n", handle);
		return;
	}

	BUG_ON(cp->sleeping);

	/* relinquish lingering client resources */
	if (cp->rsvd_bw || cp->real_bw) {
		tegra_isomgr_reserve(handle, 0, 0);
		tegra_isomgr_realize(handle);
	}

	/* unregister client and relinquish dedicated BW */
	isomgr_lock();
	isomgr.avail_bw += cp->dedi_bw;
	isomgr.dedi_bw += cp->dedi_bw;
	cp->dedi_bw = 0;
	cp->busy = false;
	cp->renegotiate = 0;
	cp->priv = 0;
	BUG_ON(cp->rsvd_bw);
	BUG_ON(cp->real_bw);
	BUG_ON(cp->alloc_delta);
	BUG_ON(cp->avail_delta);
	BUG_ON(cp->sleeping);
	isomgr_scatter(); /* share the excess */
	++isomgr.seqid;
	isomgr_unlock();
}
EXPORT_SYMBOL(tegra_isomgr_unregister);

/* reserve ISO BW for an ISO client - rval is dvfs thresh in usec */
u32 tegra_isomgr_reserve(tegra_isomgr_handle handle, u32 ubw, u32 ult)
{
	struct isomgr_client *cp = (struct isomgr_client *) handle;
	s32 avail_delta;
	s32 bw = ubw;
	s32 bias = 0;
	u32 mf, dvfs_latency = 0;
	int err;
	int client = cp - &isomgr_clients[0];

	if (unlikely((client < 0) || (client >= ISOMGR_CLIENT_COUNT) ||
		     !cp->busy)) {
		pr_err("tegra_isomgr_reserve: bad handle (%p)\n", handle);
		return dvfs_latency;
	}

	if (unlikely(ubw > (isomgr.max_iso_bw - isomgr.dedi_bw))) {
		pr_err("tegra_isomgr_reserve: invalid BW (%u Kb/sec)\n", ubw);
		return dvfs_latency;
	}

	/* Look up MC's min freq that could satisfy requested BW and LT */
	mf = mc_min_freq(ubw, ult);
	if (unlikely(!mf)) {
		pr_err("tegra_isomgr_reserve: invalid LT (%u usec)\n", ult);
		return dvfs_latency;
	}
	/* Look up MC's dvfs latency at min freq */
	dvfs_latency = mc_dvfs_latency(max(ubw, mf));

	isomgr_lock();
	cp->lti = ult;		/* remember client spec'd LT (usec) */
	cp->lto = dvfs_latency;	/* remember MC calculated LT (usec) */
	cp->rsvd_mf = mf;	/* remember associated min freq */
	if (cp->avail_delta > 0) {
		/* a reservation to increase BW lapsed */
		isomgr.avail_bw += cp->avail_delta;
		cp->avail_delta = 0;
		isomgr_scatter(); /* share the excess */
	}

	/* calculate affect on availability, given this client's dedicated BW */
	avail_delta = max(bw, cp->dedi_bw) - max(cp->real_bw, cp->dedi_bw);

	if (cp->renegotiate) {
		bias = isomgr.sleep_bw; /* don't let flex clients thrash */
	} else {

		/* inflexible clients check for obvious fail and scavenge */
		if (avail_delta > isomgr.avail_bw) {
			isomgr.sleep_bw += avail_delta;
			isomgr_scavenge();
			isomgr.sleep_bw -= avail_delta;
		}
	}
retry:
	if (avail_delta <= (isomgr.avail_bw - bias)) {
		/* got it! */
		cp->rsvd_bw = bw;
		cp->avail_delta = avail_delta;
		cp->alloc_delta = bw - cp->real_bw;
		if (avail_delta > 0) {
			/* account for BW increases upon reservation */
			isomgr.avail_bw -= avail_delta;
		}
	} else if (!cp->renegotiate) {
		cp->sleeping = true;
		isomgr.sleep_bw += avail_delta;
		++isomgr.seqid;
		isomgr_unlock();
		err = wait_for_completion_killable(&cp->cmpl);
		isomgr_lock();
		isomgr.sleep_bw -= avail_delta;
		cp->sleeping = false;
		if (unlikely(err != -EINTR))
			goto retry;
	}
	++isomgr.seqid;
	isomgr_unlock();
	return dvfs_latency;
}
EXPORT_SYMBOL(tegra_isomgr_reserve);

u32 tegra_isomgr_realize(tegra_isomgr_handle handle)
{
	struct isomgr_client *cp = (struct isomgr_client *) handle;
	int client = cp - &isomgr_clients[0];
	int i;
	u32 rval = 0;

	if (unlikely((client < 0) || (client >= ISOMGR_CLIENT_COUNT) ||
		     !cp->busy)) {
		pr_err("tegra_isomgr_realize: bad handle (%p)\n", handle);
		return rval;
	}

	isomgr_lock();
	cp->real_bw = cp->rsvd_bw;	/* reservation has been realized */
	cp->real_mf = cp->real_mf;	/* minimum frequency realized */
	if (likely(cp->alloc_delta)) {
		isomgr.alloc_bw += cp->alloc_delta;
		cp->alloc_delta = 0;
		rval = (u32)cp->lto;
		/* !!!FIXME!!! */
		/*mc_set_ptsa(cp-&isomgr_clients[0], cp->real_bw * 1000);*/

		/* determine worst case freq to satisfy LT */
		isomgr.lt_mf = 0;
		for (i = 0; i < ISOMGR_CLIENT_COUNT; ++i) {
			if (isomgr_clients[i].busy)
				isomgr.lt_mf = max(isomgr.lt_mf,
						   isomgr_clients[i].real_mf);
		}

		/* determine worst case freq to satisfy BW */
		isomgr.bw_mf = mc_min_freq(isomgr.alloc_bw, 0);

		/* combine them and set the MC clock */
		isomgr.iso_mf = max(isomgr.lt_mf, isomgr.bw_mf);
		clk_set_rate(isomgr.emc_clk, isomgr.iso_mf * 1000);
	}
	if (cp->avail_delta < 0) {
		/* account for BW decreases upon realization */
		isomgr.avail_bw -= cp->avail_delta;
		cp->avail_delta = 0;
		isomgr_scatter(); /* share the excess */
	}
	++isomgr.seqid;
	isomgr_unlock();
	return rval;
}
EXPORT_SYMBOL(tegra_isomgr_realize);

#ifdef CONFIG_TEGRA_ISOMGR_SYSFS
static ssize_t isomgr_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);

static const struct kobj_attribute seqid_attr =
	__ATTR(seqid, 0444, isomgr_show, 0);
static const struct kobj_attribute bw_mf_attr =
	__ATTR(bw_mf, 0444, isomgr_show, 0);
static const struct kobj_attribute lt_mf_attr =
	__ATTR(lt_mf, 0444, isomgr_show, 0);
static const struct kobj_attribute iso_mf_attr =
	__ATTR(iso_mf, 0444, isomgr_show, 0);
static const struct kobj_attribute alloc_bw_attr =
	__ATTR(alloc_bw, 0444, isomgr_show, 0);
static const struct kobj_attribute avail_bw_attr =
	__ATTR(avail_bw, 0444, isomgr_show, 0);
static const struct kobj_attribute sleep_bw_attr =
	__ATTR(sleep_bw, 0444, isomgr_show, 0);
static const struct kobj_attribute version_attr =
	__ATTR(version, 0444, isomgr_show, 0);

static const struct attribute *isomgr_attrs[] = {
	&seqid_attr.attr,
	&bw_mf_attr.attr,
	&lt_mf_attr.attr,
	&iso_mf_attr.attr,
	&alloc_bw_attr.attr,
	&avail_bw_attr.attr,
	&sleep_bw_attr.attr,
	&version_attr.attr,
	NULL
};

static ssize_t isomgr_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t rval = 0;

	if (attr == &seqid_attr)
		rval = sprintf(buf, "%d\n", isomgr.seqid);
	else if (attr == &bw_mf_attr)
		rval = sprintf(buf, "%d\n", isomgr.bw_mf);
	else if (attr == &lt_mf_attr)
		rval = sprintf(buf, "%d\n", isomgr.lt_mf);
	else if (attr == &iso_mf_attr)
		rval = sprintf(buf, "%d\n", isomgr.iso_mf);
	else if (attr == &alloc_bw_attr)
		rval = sprintf(buf, "%d\n", isomgr.alloc_bw);
	else if (attr == &avail_bw_attr)
		rval = sprintf(buf, "%d\n", isomgr.avail_bw);
	else if (attr == &sleep_bw_attr)
		rval = sprintf(buf, "%d\n", isomgr.sleep_bw);
	else if (attr == &version_attr)
		rval = sprintf(buf, "%d\n", ISOMGR_SYSFS_VERSION);
	return rval;
}

#ifdef CONFIG_TEGRA_ISOMGR_DEBUG
static void isomgr_client_reneg(void *priv)
{
	struct isomgr_client *cp = (struct isomgr_client *)priv;

	++cp->reneg_seqnum;
}

static ssize_t isomgr_debug_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int client = ((char *)attr - (char *)isomgr_clients) /
			sizeof(struct isomgr_client);
	struct isomgr_client *cp =
		(struct isomgr_client *)&isomgr_clients[client];

	if (attr == &cp->debug_attrs.__arg0)
		sscanf(buf, "%d\n", &cp->__arg0);
	else if (attr == &cp->debug_attrs.__arg1)
		sscanf(buf, "%d\n", &cp->__arg1);
	else if (attr == &cp->debug_attrs.__arg2)
		sscanf(buf, "%d\n", &cp->__arg2);
	return size;
}

static ssize_t isomgr_debug_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int client = ((char *)attr - (char *)isomgr_clients) /
			sizeof(struct isomgr_client);
	struct isomgr_client *cp =
			(struct isomgr_client *)&isomgr_clients[client];
	ssize_t rval = 0;
	bool b;
	tegra_isomgr_handle h;

	if (attr == &cp->debug_attrs.__arg0)
		rval = sprintf(buf, "%d\n", cp->__arg0);
	else if (attr == &cp->debug_attrs.__arg1)
		rval = sprintf(buf, "%d\n", cp->__arg1);
	else if (attr == &cp->debug_attrs.__arg2)
		rval = sprintf(buf, "%d\n", cp->__arg2);
	else if (attr == &cp->debug_attrs._register) {
		h = tegra_isomgr_register(client,
					  cp->__arg0, /* dedi_bw */
					  cp->__arg1 ? isomgr_client_reneg : 0,
					  &isomgr_clients[client]);
		rval = sprintf(buf, "%p\n", h);
	} else if (attr == &cp->debug_attrs._unregister) {
		tegra_isomgr_unregister((tegra_isomgr_handle)cp);
		rval = sprintf(buf, "\n");
	} else if (attr == &cp->debug_attrs._reserve) {
		b = tegra_isomgr_reserve((tegra_isomgr_handle)cp,
					 (u32)cp->__arg0,
					 (u32)cp->__arg1);
		rval = sprintf(buf, "%d\n", b ? 1 : 0);
	} else if (attr == &cp->debug_attrs._realize) {
		tegra_isomgr_realize((tegra_isomgr_handle)cp);
		rval = sprintf(buf, "\n");
	} else if (attr == &cp->debug_attrs.reneg_seqnum)
		rval = sprintf(buf, "%d\n", cp->reneg_seqnum);
	else if (attr == &cp->debug_attrs.dvfs_latency)
		rval = sprintf(buf, "%d\n", cp->dvfs_latency);
	return rval;
}

static const struct isomgr_debug_attrs debug_attrs = {
	__ATTR(__arg0, 0644, isomgr_debug_show, isomgr_debug_store),
	__ATTR(__arg1, 0644, isomgr_debug_show, isomgr_debug_store),
	__ATTR(__arg2, 0644, isomgr_debug_show, isomgr_debug_store),
	__ATTR(_register, 0400, isomgr_debug_show, 0),
	__ATTR(_unregister, 0400, isomgr_debug_show, 0),
	__ATTR(_reserve, 0400, isomgr_debug_show, 0),
	__ATTR(_realize, 0400, isomgr_debug_show, 0),
	__ATTR(reneg_seqnum, 0444, isomgr_debug_show, 0),
	__ATTR(dvfs_latency, 0444, isomgr_debug_show, 0),
};

#define NDATTRS (sizeof(debug_attrs) / sizeof(struct kobj_attribute))
static const struct attribute *debug_attr_list[][NDATTRS+1] = {
#define DEBUG_ATTR(i)\
	{\
		&isomgr_clients[i].debug_attrs.__arg0.attr,\
		&isomgr_clients[i].debug_attrs.__arg1.attr,\
		&isomgr_clients[i].debug_attrs.__arg2.attr,\
		&isomgr_clients[i].debug_attrs._register.attr,\
		&isomgr_clients[i].debug_attrs._unregister.attr,\
		&isomgr_clients[i].debug_attrs._reserve.attr,\
		&isomgr_clients[i].debug_attrs._realize.attr,\
		&isomgr_clients[i].debug_attrs.reneg_seqnum.attr,\
		&isomgr_clients[i].debug_attrs.dvfs_latency.attr,\
		NULL\
	},
	DEBUG_ATTR(0)
	DEBUG_ATTR(1)
	DEBUG_ATTR(2)
	DEBUG_ATTR(3)
	DEBUG_ATTR(4)
	DEBUG_ATTR(5)
};

static void isomgr_create_debug(int client)
{
	struct isomgr_client *cp = &isomgr_clients[client];

	BUG_ON(!cp->client_kobj);
	BUG_ON(cp->debug_kobj);
	cp->debug_kobj = kobject_create_and_add("debug", cp->client_kobj);
	if (!cp->debug_kobj) {
		pr_err("isomgr: failed to create sysfs debug client dir");
		return;
	}
	cp->debug_attrs = debug_attrs;
	if (sysfs_create_files(cp->debug_kobj, &debug_attr_list[client][0])) {
		pr_err("isomgr: failed to create sysfs debug files");
		kobject_del(cp->debug_kobj);
		return;
	}
}
#else
static inline void isomgr_create_debug(int client) {}
#endif

static ssize_t isomgr_client_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int client = ((char *)attr - (char *)isomgr_clients) /
			sizeof(struct isomgr_client);
	struct isomgr_client *cp =
			(struct isomgr_client *)&isomgr_clients[client];
	ssize_t rval = 0;

	if (attr == &cp->client_attrs.busy)
		rval = sprintf(buf, "%d\n", cp->busy ? 1 : 0);
	else if (attr == &cp->client_attrs.dedi_bw)
		rval = sprintf(buf, "%d\n", cp->dedi_bw);
	else if (attr == &cp->client_attrs.rsvd_bw)
		rval = sprintf(buf, "%d\n", cp->rsvd_bw);
	else if (attr == &cp->client_attrs.real_bw)
		rval = sprintf(buf, "%d\n", cp->real_bw);
	else if (attr == &cp->client_attrs.need_bw)
		rval = sprintf(buf, "%d\n", cp->avail_delta);
	else if (attr == &cp->client_attrs.lti)
		rval = sprintf(buf, "%d\n", cp->lti);
	else if (attr == &cp->client_attrs.lto)
		rval = sprintf(buf, "%d\n", cp->lto);
	else if (attr == &cp->client_attrs.rsvd_mf)
		rval = sprintf(buf, "%d\n", cp->rsvd_mf);
	else if (attr == &cp->client_attrs.real_mf)
		rval = sprintf(buf, "%d\n", cp->real_mf);
	return rval;
}

static const struct isomgr_client_attrs client_attrs = {
	__ATTR(busy,    0444, isomgr_client_show, 0),
	__ATTR(dedi_bw, 0444, isomgr_client_show, 0),
	__ATTR(rsvd_bw, 0444, isomgr_client_show, 0),
	__ATTR(real_bw, 0444, isomgr_client_show, 0),
	__ATTR(need_bw, 0444, isomgr_client_show, 0),
	__ATTR(lti,     0444, isomgr_client_show, 0),
	__ATTR(lto,     0444, isomgr_client_show, 0),
	__ATTR(rsvd_mf, 0444, isomgr_client_show, 0),
	__ATTR(real_mf, 0444, isomgr_client_show, 0),
};

#define NCATTRS (sizeof(client_attrs) / sizeof(struct kobj_attribute))
static const struct attribute *client_attr_list[][NCATTRS+1] = {
#define CLIENT_ATTR(i)\
	{\
		&isomgr_clients[i].client_attrs.busy.attr,\
		&isomgr_clients[i].client_attrs.dedi_bw.attr,\
		&isomgr_clients[i].client_attrs.rsvd_bw.attr,\
		&isomgr_clients[i].client_attrs.real_bw.attr,\
		&isomgr_clients[i].client_attrs.need_bw.attr,\
		&isomgr_clients[i].client_attrs.lti.attr,\
		&isomgr_clients[i].client_attrs.lto.attr,\
		&isomgr_clients[i].client_attrs.rsvd_mf.attr,\
		&isomgr_clients[i].client_attrs.real_mf.attr,\
		NULL\
	},
	CLIENT_ATTR(0)
	CLIENT_ATTR(1)
	CLIENT_ATTR(2)
	CLIENT_ATTR(3)
	CLIENT_ATTR(4)
	CLIENT_ATTR(5)
};

static void isomgr_create_client(int client, const char *name)
{
	struct isomgr_client *cp = &isomgr_clients[client];

	BUG_ON(!isomgr.kobj);
	BUG_ON(cp->client_kobj);
	cp->client_kobj = kobject_create_and_add(name, isomgr.kobj);
	if (!cp->client_kobj) {
		pr_err("isomgr: failed to create sysfs client dir");
		return;
	}
	cp->client_attrs = client_attrs;
	if (sysfs_create_files(cp->client_kobj, &client_attr_list[client][0])) {
		pr_err("isomgr: failed to create sysfs client files");
		kobject_del(cp->client_kobj);
		return;
	}
	isomgr_create_debug(client);
}

static void isomgr_create_sysfs(void)
{
	BUG_ON(isomgr.kobj);
	isomgr.kobj = kobject_create_and_add("isomgr", kernel_kobj);
	if (!isomgr.kobj) {
		pr_err("isomgr: failed to create kobject");
		return;
	}
	if (sysfs_create_files(isomgr.kobj, isomgr_attrs)) {
		pr_err("isomgr: failed to create sysfs files");
		kobject_del(isomgr.kobj);
		isomgr.kobj = 0;
		return;
	}
	isomgr_create_client(ISOMGR_CLIENT_DISP_0, "disp_0");
	isomgr_create_client(ISOMGR_CLIENT_DISP_1, "disp_1");
	isomgr_create_client(ISOMGR_CLIENT_VI_0,   "vi_0");
	isomgr_create_client(ISOMGR_CLIENT_VI_1,   "vi_1");
	isomgr_create_client(ISOMGR_CLIENT_ISP_0,  "isp_0");
	isomgr_create_client(ISOMGR_CLIENT_ISP_1,  "isp_1");
}
#else
static inline void isomgr_create_sysfs(void) {};
#endif /* CONFIG_TEGRA_ISOMGR_SYSFS */

static int __init isomgr_init(void)
{
	int i;

	mutex_init(&isomgr.lock);
	isomgr.emc_clk = clk_get_sys("iso", "emc");
	for (i = 0; i < ISOMGR_CLIENT_COUNT; ++i)
		init_completion(&isomgr_clients[i].cmpl);
	isomgr_create_sysfs();
	return 0;
}
late_initcall(isomgr_init);
