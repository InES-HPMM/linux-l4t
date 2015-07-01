/*
 * drivers/video/tegra/dc/dphdcp.c
 *
 * Copyright (c) 2015, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <mach/dc.h>
#include <mach/kfuse.h>

#include "dphdcp.h"
#include "dp.h"
#include "edid.h"
#include "sor.h"
#include "sor_regs.h"
#include "dpaux_regs.h"
#include "tsec_drv.h"
#include "tsec/tsec_methods.h"
#include "nvhdcp_hdcp22_methods.h"
#include "tsec/tsec.h"

static DECLARE_WAIT_QUEUE_HEAD(wq_worker);

/* Bcaps register bits */
#define BCAPS_REPEATER (1 << 1)
#define BCAPS_HDCP_CAPABLE (1 << 0)

/* Bstatus register bits */
#define BSTATUS_REAUTH_REQ	(1 << 3)
#define BSTATUS_LINK_INTEG_FAIL	(1 << 2)
#define BSTATUS_R0_PRIME_SET	(1 << 1)
#define BSTATUS_READY		(1 << 0)

/* Binfo register bits */
#define BINFO_MAX_DEVS_EXCEEDED (1 << 7)
#define BINFO_MAX_CASCADE_EXCEEDED (1 << 11)

/* for hdcp 2.2 */
#define HDCP22_PROTOCOL		1
#define HDCP1X_PROTOCOL		0
#define HDCP_DEBUG		1
#define HDCP_READY		1
#define HDCP_REAUTH		2
#define HDCP_REAUTH_MASK	(1 << 3)

#define MAX_AUX_SIZE		15
/* logging */
#ifdef VERBOSE_DEBUG
#define dphdcp_vdbg(...)	\
		pr_debug("dphdcp: " __VA_ARGS__)
#else
#define dphdcp_vdbg(...)		\
({						\
	if (0)					\
		pr_debug("dphdcp: " __VA_ARGS__); \
	0;					\
})
#endif
#define dphdcp_debug(...)	\
		pr_debug("dphdcp: " __VA_ARGS__)
#define	dphdcp_err(...)	\
		pr_err("dphdcp: Error: " __VA_ARGS__)
#define dphdcp_info(...)	\
		pr_info("dphdcp: " __VA_ARGS__)

static int tegra_dpcd_hdcp_read(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 *data_ptr, u32 size, u32 *aux_status)
{
	u32 status = 0;
	u32 cursize = 0;
	int ret = 0;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return -EIO;

	cursize = size;
	mutex_lock(&dp->dpaux_lock);
	ret = tegra_dc_dpaux_read_chunk_locked(dp, DPAUX_DP_AUXCTL_CMD_AUXRD,
		cmd, data_ptr, &cursize, &status);
	mutex_unlock(&dp->dpaux_lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to read DPCD data. CMD 0x%x, Status 0x%x\n",
			cmd, status);
	*aux_status = status;
	return ret;
}

static int tegra_dpcd_hdcp_write(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 *data, u32 size)
{
	u32 status = 0;
	int ret;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return -EIO;

	mutex_lock(&dp->dpaux_lock);
	ret = tegra_dc_dpaux_write_chunk_locked(dp, DPAUX_DP_AUXCTL_CMD_AUXWR,
		cmd, data, &size, &status);
	mutex_unlock(&dp->dpaux_lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to write DPCD data. CMD 0x%x, Status 0x%x\n",
			cmd, status);
	return ret;
}

/* read 5 bytes of data */
static int tegra_dpcd_hdcp_read40(struct tegra_dc_dp_data *dp, u32 cmd,
			u64 *data)
{
	u8 buf[5];
	int i;
	u64 n;
	int e;
	u32 status;
	e = tegra_dpcd_hdcp_read(dp, cmd, buf, sizeof(buf), &status);
	if (e)
		return e;

	/* assign the value read from aux to data */
	for (i = 0, n = 0; i < 5; i++) {
		n <<= 8;
		n |= buf[4 - i];
	}
	if (data)
		*data = n;
	return 0;
}

/* read 2 bytes of data */
static int tegra_dpcd_hdcp_read16(struct tegra_dc_dp_data *dp, u32 cmd,
				u64 *data)
{
	u8 buf[2];
	int e;
	u32 status;

	e = tegra_dpcd_hdcp_read(dp, cmd, buf, sizeof(buf), &status);
	if (e)
		return e;

	if (data)
		*data = buf[0] | (u16)buf[1] << 8;
	return 0;
}

/* write 1 byte of data */
static int tegra_dpcd_hdcp_write8(struct tegra_dc_dp_data *dp, u32 reg,
	u64 data)
{
	char buf[1];
	memcpy(buf, (char *)&data, sizeof(buf));
	return tegra_dpcd_hdcp_write(dp, reg, buf, sizeof(buf));
}

/* write 8 bytes of data */
static int tegra_dpcd_hdcp_write64(struct tegra_dc_dp_data *dp, u32 reg,
	u64 data)
{
	char buf[8];
	memcpy(buf, (char *)&data, sizeof(buf));
	return tegra_dpcd_hdcp_write(dp, reg, buf, sizeof(buf));
}

/* write 5 bytes of data */
static int tegra_dpcd_hdcp_write40(struct tegra_dc_dp_data *dp, u32 reg,
	u64 data)
{
	char buf[5];
	memcpy(buf, (char *)&data, sizeof(buf));
	return tegra_dpcd_hdcp_write(dp, reg, buf, sizeof(buf));
}

/* wait for bits in mask to be set to value in NV_SOR_KEY_CTRL
* waits upto 100 ms */
static int wait_key_ctrl(struct tegra_dc_sor_data *sor, u32 mask, u32 value)
{
	int retries = 101;
	u32 ctrl;

	do {
		usleep_range(1, 2);
		ctrl = tegra_sor_readl(sor, NV_SOR_KEY_CTRL);
		if (((ctrl ^ value) & mask) == 0)
			break;
	} while (--retries);
	if (!retries) {
		dphdcp_err("key ctrl read timeout (mask=0x%x)\n", mask);
		return -EIO;
	}
	return 0;
}

/* set or clear RUN_YES */
static void hdcp_ctrl_run(struct tegra_dc_sor_data *sor, bool v)
{
	u32 ctrl;

	if (v) {
		ctrl = tegra_sor_readl(sor, NV_SOR_DP_HDCP_CTRL);
		ctrl |= HDCP_RUN_YES;
	} else {
		ctrl = 0;
	}

	tegra_sor_writel(sor, NV_SOR_DP_HDCP_CTRL, ctrl);
}

/* wait for any bits in mask to be set in NV_SOR_DP_HDCP_CTRL
 * sleeps up to 120 ms */
static int wait_hdcp_ctrl(struct tegra_dc_sor_data *sor, u32 mask, u32 *v)
{
	int retries = 13;
	u32 ctrl;

	do {
		ctrl = tegra_sor_readl(sor, NV_SOR_DP_HDCP_CTRL);
		if ((ctrl & mask)) {
			if (v)
				*v = ctrl;
			break;
		}
		if (retries > 1)
			usleep_range(10, 15);
	} while (--retries);
	if (!retries) {
		dphdcp_err("ctrl read timeout (mask=0x%x)\n", mask);
		return -EIO;
	}
	return 0;
}

/* 64-bit link encryption session random number */
static inline u64 get_an(struct tegra_dc_sor_data *sor)
{
	u64 r;
	r = (u64)tegra_sor_readl(sor, NV_SOR_DP_HDCP_AN_MSB) << 32;
	r |= tegra_sor_readl(sor, NV_SOR_DP_HDCP_AN_LSB);
	return r;
}

/* 40-bit transmitter's key selection vector */
static inline u64 get_aksv(struct tegra_dc_sor_data *sor)
{
	u64 r;
	r = (u64)tegra_sor_readl(sor, NV_SOR_DP_HDCP_AKSV_MSB) << 32;
	r |= tegra_sor_readl(sor, NV_SOR_DP_HDCP_AKSV_LSB);
	return r;
}

/* 40-bit receiver's key selection vector */
static inline void set_bksv(struct tegra_dc_sor_data *sor, u64 b_ksv,
							bool repeater)
{
	if (repeater)
		b_ksv |= (u64)REPEATER << 32;
	tegra_sor_writel(sor, NV_SOR_DP_HDCP_BKSV_LSB, (u32)b_ksv);
	tegra_sor_writel(sor, NV_SOR_DP_HDCP_BKSV_MSB, b_ksv >> 32);
}

/* check if the KSV values returned are valid,
	i.e a combination of 20 ones and 20 zeroes */
static int verify_ksv(u64 k)
{
	unsigned i;
	/* count set bits, must be exactly 20 set to be valid */
	for (i = 0; k; i++)
		k ^= k & -k;

	return  (i != 20) ? -EINVAL : 0;
}

static int get_bcaps(struct tegra_dc_dp_data *dp, u8 *b_caps)
{
	u32 status;
	return tegra_dpcd_hdcp_read(dp, NV_DPCD_HDCP_BCAPS_OFFSET,
						b_caps, 1, &status);
}

static int get_bstatus(struct tegra_dc_dp_data *dp, u8 *bstatus)
{
	u32 status;
	return tegra_dpcd_hdcp_read(dp, NV_DPCD_HDCP_BSTATUS_OFFSET,
						bstatus, 1, &status);
}

static inline bool dphdcp_is_plugged(struct tegra_dphdcp *dphdcp)
{
	rmb();
	return dphdcp->plugged;
}

static inline bool dphdcp_set_plugged(struct tegra_dphdcp *dphdcp,
						bool plugged)
{
	dphdcp->plugged = plugged;
	wmb();
	return plugged;
}

static int load_kfuse(struct tegra_dc_dp_data *dp)
{
	u32 ctrl;
	u32 tmp;
	int retries;
	int e;
	int i;
	unsigned buf[KFUSE_DATA_SZ/4];
	struct tegra_dc_sor_data *sor = dp->sor;
	memset(buf, 0, sizeof(buf));

	/* load kfuse */
	dphdcp_vdbg("loading kfuse\n");
	/* copy load kfuse into buffer - only needed for early Tegra parts */
	e = tegra_kfuse_read(buf, sizeof(buf));
	if (e) {
		dphdcp_err("Kfuse read failure\n");
		return e;
	}

	/* write the kfuse to the DP SRAM */
	tegra_sor_writel(sor, NV_SOR_KEY_CTRL, 1);

	/* issue a reload */
	ctrl = tegra_sor_readl(sor, NV_SOR_KEY_CTRL);
	tegra_sor_writel(sor, NV_SOR_KEY_CTRL, ctrl | PKEY_RELOAD_TRIGGER
					| LOCAL_KEYS);
	e = wait_key_ctrl(sor, PKEY_LOADED, PKEY_LOADED);
	if (e) {
		dphdcp_err("key reload timeout\n");
		return e;
	}

	tegra_sor_writel(sor, NV_SOR_KEY_SKEY_INDEX, 0);

	/* wait for SRAM to be cleared */
	retries = 6;
	do {
		tmp = tegra_sor_readl(sor, NV_SOR_KEY_DEBUG0);
		if ((tmp & 1) == 0)
			break;
		if (retries > 1)
			mdelay(1);
	} while (--retries);
	if (!retries) {
		dphdcp_err("key SRAM clear timeout\n");
		return -EIO;
	}

	for (i = 0; i < KFUSE_DATA_SZ / 4; i += 4) {

		/* load 128-bits*/
		tegra_sor_writel(sor, NV_SOR_KEY_HDCP_KEY_0, buf[i]);
		tegra_sor_writel(sor, NV_SOR_KEY_HDCP_KEY_1, buf[i+1]);
		tegra_sor_writel(sor, NV_SOR_KEY_HDCP_KEY_2, buf[i+2]);
		tegra_sor_writel(sor, NV_SOR_KEY_HDCP_KEY_3, buf[i+3]);

		/* trigger LOAD_HDCP_KEY */
		tegra_sor_writel(sor, NV_SOR_KEY_HDCP_KEY_TRIG, 0x100);
		tmp = LOCAL_KEYS | WRITE16;
		if (i)
			tmp |= AUTOINC;
		tegra_sor_writel(sor, NV_SOR_KEY_CTRL, tmp);

		/* wait for WRITE16 to complete */
		e = wait_key_ctrl(sor, 0x10, 0); /* WRITE16 */
		if (e) {
			dphdcp_err("key write timeout\n");
			return -EIO;
		}
	}
	return 0;
}

/* check the 16 bit link integrity value */
static inline u64 get_transmitter_ro_prime(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	return tegra_sor_readl(sor, NV_SOR_DP_HDCP_RI);
}

/* R0' prime value generated from the receiver */
static inline int get_receiver_ro_prime(struct tegra_dc_dp_data *dp, u64 *r)
{
	return tegra_dpcd_hdcp_read16(dp, NV_DPCD_HDCP_RPRIME_OFFSET, r);
}

static int validate_rx(struct tegra_dphdcp *dphdcp)
{
	int retries = 3;
	u64 rx = 0, tx = 0;
	int e;
	struct tegra_dc_dp_data *dp = dphdcp->dp;

	/* try 3 times for possible link errors */
	do {
		tx = get_transmitter_ro_prime(dp);
		e = get_receiver_ro_prime(dp, &rx);
	} while (--retries && rx != tx);
	dphdcp_vdbg("rx=0x%016llx tx=0x%016llx\n", rx, tx);

	if (rx != tx)
		return -EINVAL;
	return 0;
}

/* get V' 160-bit SHA-1 hash from repeater */
static int get_vprime(struct tegra_dc_dp_data *dp, u8 *v_prime)
{
	int e, i;
	u32 status;

	for (i = 0; i < 20; i += 4) {
		e = tegra_dpcd_hdcp_read(dp,
			NV_DPCP_HDCP_SHA_H0_OFFSET+i, v_prime+i, 4, &status);
		if (e) {
			dphdcp_err("Error reading V'\n");
			return e;
		}
	}
	return 0;
}

static int get_ksvfifo(struct tegra_dc_dp_data *dp,
					unsigned num_bksv_list, u64 *ksv_list)
{
	u8 *buf;
	u8 *orig_buf;
	int e;
	unsigned int dp_retries = 10;
	u32 status;
	size_t buf_len = num_bksv_list * 5;


	if (!ksv_list || num_bksv_list > TEGRA_NVHDCP_MAX_DEVS)
		return -EINVAL;

	if (num_bksv_list == 0)
		return 0;

	buf = kmalloc(buf_len, GFP_KERNEL);
	if (IS_ERR_OR_NULL(buf))
		return -ENOMEM;

	orig_buf = buf;
	while (buf_len > 15) {
aux_read:
			e = tegra_dpcd_hdcp_read(dp,
			NV_DPCD_HDCP_KSV_FIFO_OFFSET, buf, 15, &status);
			if (e) {
				dphdcp_err("Error reading KSV\n");
				kfree(orig_buf);
				return e;
			}
			if ((status & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CDEFER) ||
			(status & DPAUX_DP_AUXSTAT_REPLYTYPE_DEFER)) {
				if (--dp_retries)
					goto aux_read;
			}
			buf_len -= 15;
			buf += 15;
	}

	if (buf_len)
		e = tegra_dpcd_hdcp_read(dp,
		NV_DPCD_HDCP_KSV_FIFO_OFFSET, buf, buf_len, &status);
	if (e) {
		dphdcp_err("Error reading KSV\n");
		kfree(orig_buf);
		return e;
	}
	memcpy(ksv_list, orig_buf, num_bksv_list*5);
	kfree(orig_buf);
	return 0;
}

int tsec_hdcp_dp_verify_vprime(struct tegra_dphdcp *dphdcp)
{
	struct hdcp_context_t hdcp_context;
	struct hdcp_verify_vprime_param verify_vprime_param;
	int e = 0;
	e = tsec_hdcp_create_context(&hdcp_context);
	if (e)
		dphdcp_err("Error creating hdcp context\n");
	e = tsec_hdcp_init(&hdcp_context);
	if (e)
		dphdcp_err("error in tsec init\n");
	memset(&verify_vprime_param, 0x0,
		sizeof(struct hdcp_verify_vprime_param));
	memset(hdcp_context.cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	/* revocation check */
	verify_vprime_param.srm_size =
		tsec_dp_hdcp_revocation_check(&hdcp_context);
	/* get receiver id list into the buffer */
	memcpy(hdcp_context.cpuvaddr_rcvr_id_list, dphdcp->bksv_list,
			(dphdcp->num_bksv_list)*5);
	memcpy(verify_vprime_param.vprime, dphdcp->v_prime,
			HDCP_SIZE_VPRIME_1X_8);
	verify_vprime_param.trans_id.session_id = 0;
	verify_vprime_param.is_ver_hdcp2x = 0; /* hdcp 1.x */
	verify_vprime_param.bstatus = dphdcp->binfo;
	verify_vprime_param.depth = 0; /* depth not used */
	verify_vprime_param.device_count = dphdcp->num_bksv_list;
	verify_vprime_param.has_hdcp2_repeater = 0;
	verify_vprime_param.has_hdcp1_device = 0;
	memcpy(hdcp_context.cpuvaddr_mthd_buf_aligned,
		&verify_vprime_param,
		sizeof(struct hdcp_verify_vprime_param));
	tsec_send_method(&hdcp_context,
	HDCP_VERIFY_VPRIME,
	HDCP_MTHD_FLAGS_SB|HDCP_MTHD_FLAGS_RECV_ID_LIST|HDCP_MTHD_FLAGS_SRM);
	memcpy(&verify_vprime_param,
		hdcp_context.cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_vprime_param));
	if (verify_vprime_param.ret_code) {
		dphdcp_err("tsec_hdcp_verify_vprime: failed with error:%d\n",
		verify_vprime_param.ret_code);
	}
	e = verify_vprime_param.ret_code;
	return e;
}

static int get_repeater_info(struct tegra_dphdcp *dphdcp)
{
	int e, retries;
	int err = 0;
	int vcheck_tries = 3;
	u8 bstatus;
	u64 binfo;
	struct tegra_dc_dp_data *dp = dphdcp->dp;

	dphdcp_vdbg("repeater found:fetching repeater info\n");
	/* wait up to 5 seconds for READY on repeater */
	retries = 51;
	do {
		mutex_lock(&dphdcp->lock);
		if (!dphdcp_is_plugged(dphdcp)) {
			dphdcp_err("disconnect while waiting for repeater\n");
			mutex_unlock(&dphdcp->lock);
			return -EIO;
		}
		mutex_unlock(&dphdcp->lock);
		/* wait till receiver computes V' */
		e = get_bstatus(dp, &bstatus);
		if (!e && (bstatus & BSTATUS_READY)) {
			dphdcp_vdbg("Bstatus READY from repeater\n");
			break;
		}
		if (retries > 1)
			msleep(100);
	} while (--retries);
	if (!retries) {
		dphdcp_err("repeater Bstatus read timeout\n");
		return -ETIMEDOUT;
	}
	/* verify V' thrice to check for link failures */
	do {
		memset(dphdcp->bksv_list, 0, sizeof(dphdcp->bksv_list));
		memset(dphdcp->v_prime, 0, sizeof(dphdcp->v_prime));
		/* read Binfo register */
		e = tegra_dpcd_hdcp_read16(dp, NV_DPCD_HDCP_BINFO_OFFSET,
				&binfo);
		if (e) {
			dphdcp_err("Binfo read failure!\n");
			return e;
		}
		msleep(100);

		if (binfo & BINFO_MAX_DEVS_EXCEEDED) {
			dphdcp_err("repeater:max devices (0x%016llx)\n", binfo);
			return -EINVAL;
		}

		if (binfo & BINFO_MAX_CASCADE_EXCEEDED) {
			dphdcp_err("repeater:max cascade (0x%16llx)\n", binfo);
			return -EINVAL;
		}
		dphdcp->binfo = binfo;
		dphdcp->num_bksv_list = binfo & 0x7f;
		dphdcp_vdbg("Binfo 0x%16llx (devices: %d)\n",
				binfo, dphdcp->num_bksv_list);

		/* do not read KSV FIFO when device count is zero */
		if (dphdcp->num_bksv_list != 0)	 {
			e = get_ksvfifo(dp, dphdcp->num_bksv_list,
						dphdcp->bksv_list);
			if (e) {
				dphdcp_err("KSVFIFO read (err %d)\n", e);
				return e;
			}
		}
		/* verify V' three times to check for link failures */
		e = get_vprime(dp, dphdcp->v_prime);
		if (e)
			dphdcp_err("repeater Vprime read failure!\n");

		err = tsec_hdcp_dp_verify_vprime(dphdcp);
		if (err)
			dphdcp_err("vprime verification failed\n");
	} while (--vcheck_tries && err);
	if (err)
		return -EINVAL;
	return 0;
}

static void dphdcp_downstream_worker(struct work_struct *work)
{
	int e;
	u8 b_caps;
	u32 res;
	u32 tmp;
	u8 bstatus;
	struct tegra_dphdcp *dphdcp =
		container_of(to_delayed_work(work), struct tegra_dphdcp, work);
	struct tegra_dc_dp_data *dp = dphdcp->dp;
	struct tegra_dc_sor_data *sor = dp->sor;
	struct tegra_dc *dc = dp->dc;
	dphdcp_vdbg("%s():started thread %s\n", __func__, dphdcp->name);
	tegra_dc_io_start(dc);
	mutex_lock(&dphdcp->lock);
	if (dphdcp->state == STATE_OFF) {
		dphdcp_err("dphdcp failure - giving up\n");
		goto err;
	}
	dphdcp->state = STATE_UNAUTHENTICATED;

	/* check plug state to terminate early in case flush_workqueue() */
	if (!dphdcp_is_plugged(dphdcp)) {
		dphdcp_err("worker started while unplugged!\n");
		goto lost_dp;
	}
	dphdcp_vdbg("%s():hpd=%d\n", __func__, dphdcp->plugged);

	dphdcp->a_ksv = 0;
	dphdcp->b_ksv = 0;
	dphdcp->a_n = 0;
	mutex_unlock(&dphdcp->lock);

	/* read bcaps from receiver */
	e = get_bcaps(dp, &b_caps);
	mutex_lock(&dphdcp->lock);
	if (e) {
		dphdcp_err("Error reading bcaps\n");
		goto failure;
	}

	dphdcp_vdbg("read Bcaps = 0x%02x\n", b_caps);

	/* check if receiver is hdcp capable */
	if (b_caps & BCAPS_HDCP_CAPABLE)
		dphdcp_vdbg("receiver is hdcp capable\n");
	else {
		dphdcp_err("receiver is not hdcp capable\n");
		goto failure;
	}

	set_bksv(sor, 0, (b_caps & BCAPS_REPEATER));

	e = load_kfuse(dp);
	if (e) {
		dphdcp_err("error loading kfuse\n");
		goto failure;
	}

	usleep_range(20000, 25000);
	hdcp_ctrl_run(sor, 1);

	dphdcp_vdbg("waiting for An_valid\n");

	/* wait for hardware to generate HDCP values */
	e = wait_hdcp_ctrl(sor, AN_VALID | SROM_ERR, &res);
	if (e) {
		dphdcp_err("An key generation timeout\n");
		goto failure;
	}
	if (res & SROM_ERR) {
		dphdcp_err("SROM error\n");
		goto failure;
	}

	msleep(25);

	dphdcp->a_ksv = get_aksv(sor);
	dphdcp->a_n = get_an(sor);

	dphdcp_vdbg("aksv is 0x%016llx\n", dphdcp->a_ksv);
	dphdcp_vdbg("an is 0x%016llx\n", dphdcp->a_n);

	if (verify_ksv(dphdcp->a_ksv)) {
		dphdcp_err("Aksv verify failure! (0x%016llx)\n", dphdcp->a_ksv);
		goto disable;
	}
	mutex_unlock(&dphdcp->lock);

	/* write An to receiver */
	e = tegra_dpcd_hdcp_write64(dp, NV_DPCD_HDCP_AN_OFFSET, dphdcp->a_n);
	if (e) {
		dphdcp_err("An write failure\n");
		mutex_lock(&dphdcp->lock);
		goto failure;
	}
	dphdcp_vdbg("wrote An = 0x%016llx\n", dphdcp->a_n);

	/* write Aksv to receiver */
	e = tegra_dpcd_hdcp_write40(dp, NV_DPCD_HDCP_AKSV_OFFSET,
						dphdcp->a_ksv);
	if (e) {
		dphdcp_err("Aksv write failure\n");
		mutex_lock(&dphdcp->lock);
		goto failure;
	}
	dphdcp_vdbg("wrote Aksv = 0x%010llx\n", dphdcp->a_ksv);

	mutex_lock(&dphdcp->lock);
	/* handle if connection lost in the middle of authentication */
	if (!dphdcp_is_plugged(dphdcp))
		goto lost_dp;

	mutex_unlock(&dphdcp->lock);

	/* get bksv from receiver */
	e = tegra_dpcd_hdcp_read40(dp, NV_DPCD_HDCP_BKSV_OFFSET,
					&dphdcp->b_ksv);
	mutex_lock(&dphdcp->lock);
	if (e) {
		dphdcp_err("Bksv read failure\n");
		goto failure;
	}
	dphdcp_vdbg("read Bksv from device: 0x%016llx\n", dphdcp->b_ksv);

	/* verify the bksv to be if it has a correct combination of
		1's and 0's */
	if (verify_ksv(dphdcp->b_ksv)) {
		dphdcp_err("Bksv verify failure! (0x%016llx)\n", dphdcp->b_ksv);
		goto failure;
	}

	/*if repeater, set the reauth enable irq bit in Ainfo reg */
	if (b_caps & BCAPS_REPEATER) {
		e = tegra_dpcd_hdcp_write8(dp, NV_DPCD_HDCP_AINFO_OFFSET,
						0x01);
		if (e) {
			dphdcp_err("Ainfo write failure\n");
			mutex_lock(&dphdcp->lock);
			goto failure;
		}
	}

	set_bksv(sor, dphdcp->b_ksv, (b_caps & BCAPS_REPEATER));
	dphdcp_vdbg("Loaded Bksv into controller\n");

	/* check if the computations of Km, Ks, M0 and R0 are over */
	e = wait_hdcp_ctrl(sor, R0_VALID, NULL);
	if (e) {
		dphdcp_err("R0 read failure\n");
		goto failure;
	}

	dphdcp_vdbg("R0 valid\n");
	mutex_unlock(&dphdcp->lock);

	msleep(100); /* cannot read R0' within 100 ms of writing AKSV */

	/* after part 1 of authentication protocol, check for
	link integrity
	TODO: add support for both single and multi stream mode */
	e = validate_rx(dphdcp);
	if (e) {
		dphdcp_err("could not validate receiver\n");
		mutex_lock(&dphdcp->lock);
		goto failure;
	}
	tmp = tegra_sor_readl(sor, NV_SOR_DP_HDCP_CTRL);
	tmp |= CRYPT_ENABLED;
	tegra_sor_writel(sor, NV_SOR_DP_HDCP_CTRL, tmp);
	dphdcp_vdbg("CRYPT enabled\n");

	/* part 2 of authentication protocol, if receiver is
	a repeater */
	if (b_caps & BCAPS_REPEATER) {
		e = get_repeater_info(dphdcp);
		if (e) {
			dphdcp_err("get repeater info failed\n");
			mutex_lock(&dphdcp->lock);
			goto failure;
		}
	}

	mutex_lock(&dphdcp->lock);
	dphdcp->state = STATE_LINK_VERIFY;
	dphdcp_info("link verified!\n");

	while (1) {
		if (!dphdcp_is_plugged(dphdcp))
			goto lost_dp;

		if (dphdcp->state != STATE_LINK_VERIFY)
			goto failure;

		mutex_unlock(&dphdcp->lock);
		/* check for link integrity failure */
		e = get_bstatus(dp, &bstatus);
		if (!e && (bstatus & BSTATUS_LINK_INTEG_FAIL)) {
			dphdcp_err("link integrity failure\n");
			goto failure;
		}
		tegra_dc_io_end(dc);
		wait_event_interruptible_timeout(wq_worker,
			!dphdcp_is_plugged(dphdcp), msecs_to_jiffies(200));
		tegra_dc_io_start(dc);
		mutex_lock(&dphdcp->lock);

	}

failure:
	dphdcp->fail_count++;
	if (dphdcp->fail_count > dphdcp->max_retries)
		dphdcp_err("dphdcp failure- too many failures, giving up\n");
	else {
		dphdcp_err("dphdcp failure- renegotiating in 1 second\n");
		if (!dphdcp_is_plugged(dphdcp))
			goto lost_dp;
		queue_delayed_work(dphdcp->downstream_wq, &dphdcp->work,
						msecs_to_jiffies(1000));
	}

lost_dp:
	dphdcp_info("lost dp connection\n");
	dphdcp->state = STATE_UNAUTHENTICATED;
	hdcp_ctrl_run(sor, 0);

err:
	mutex_unlock(&dphdcp->lock);
	tegra_dc_io_end(dc);
	return;

disable:
	dphdcp->state = STATE_OFF;
	dphdcp_set_plugged(dphdcp, false);
	mutex_unlock(&dphdcp->lock);
	tegra_dc_io_end(dc);
	return;
}

static int tegra_dphdcp_on(struct tegra_dphdcp *dphdcp)
{
	u8 hdcp2version = 0;
	int e;
	u32 status;
	struct tegra_dc_dp_data *dp = dphdcp->dp;
	dphdcp->state = STATE_UNAUTHENTICATED;
	if (dphdcp_is_plugged(dphdcp)) {
		dphdcp->fail_count = 0;
		e = tegra_dpcd_hdcp_read(dp, HDCP_HDCP2_VERSION,
				&hdcp2version, 1, &status);
		if (e)
			return -EIO;

		dphdcp_vdbg("read back version:%x\n", hdcp2version);
		if (hdcp2version & HDCP_HDCP2_VERSION_HDCP22_YES)
			dphdcp->hdcp22 = HDCP22_PROTOCOL;
		else {
			INIT_DELAYED_WORK(&dphdcp->work,
					dphdcp_downstream_worker);
			dphdcp->hdcp22 = HDCP1X_PROTOCOL;
		}

		queue_delayed_work(dphdcp->downstream_wq, &dphdcp->work,
						msecs_to_jiffies(100));
	}

	return 0;
}

static int tegra_dphdcp_off(struct tegra_dphdcp *dphdcp)
{
	mutex_lock(&dphdcp->lock);
	dphdcp->state = STATE_OFF;
	dphdcp_set_plugged(dphdcp, false);
	mutex_unlock(&dphdcp->lock);
	wake_up_interruptible(&wq_worker);
	cancel_delayed_work_sync(&dphdcp->work);
	return 0;
}

static int get_dphdcp_state(struct tegra_dphdcp *dphdcp,
			struct tegra_nvhdcp_packet *pkt)
{
	int i;
	mutex_lock(&dphdcp->lock);
	if (dphdcp->state != STATE_LINK_VERIFY) {
		memset(pkt, 0, sizeof(*pkt));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_LINK_FAILED;
	} else {
		pkt->num_bksv_list = dphdcp->num_bksv_list;
		for (i = 0; i < pkt->num_bksv_list; i++)
			pkt->bksv_list[i] = dphdcp->bksv_list[i];
		pkt->binfo = dphdcp->binfo;
		pkt->b_ksv = dphdcp->b_ksv;
		memcpy(pkt->v_prime, dphdcp->v_prime, sizeof(dphdcp->v_prime));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_SUCCESS;
		pkt->hdcp22 = dphdcp->hdcp22;
		pkt->port = TEGRA_NVHDCP_PORT_DP;
	}
	mutex_unlock(&dphdcp->lock);
	return 0;
}

static int tegra_dphdcp_renegotiate(struct tegra_dphdcp *dphdcp)
{
	mutex_lock(&dphdcp->lock);
	dphdcp->state = STATE_RENEGOTIATE;
	mutex_unlock(&dphdcp->lock);
	tegra_dphdcp_on(dphdcp);
	return 0;
}

void tegra_dphdcp_set_plug(struct tegra_dphdcp *dphdcp, bool hpd)
{
	dphdcp_debug("DP hotplug detected (hpd = %d)\n", hpd);

	if (hpd) {
		dphdcp_set_plugged(dphdcp, true);
		tegra_dphdcp_on(dphdcp);
	} else {
		tegra_dphdcp_off(dphdcp);
	}
}

static long dphdcp_dev_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	struct tegra_dphdcp *dphdcp = filp->private_data;
	struct tegra_nvhdcp_packet *pkt;
	int e = -ENOTTY;

	switch (cmd) {
	case TEGRAIO_NVHDCP_ON:
		return tegra_dphdcp_on(dphdcp);

	case TEGRAIO_NVHDCP_OFF:
		return tegra_dphdcp_off(dphdcp);

	case TEGRAIO_NVHDCP_RENEGOTIATE:
		e = tegra_dphdcp_renegotiate(dphdcp);
		break;

	case TEGRAIO_NVHDCP_HDCP_STATE:
		pkt = kmalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt)
			return -ENOMEM;
		e = get_dphdcp_state(dphdcp, pkt);
		if (copy_to_user((void __user *)arg, pkt, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		kfree(pkt);
		return e;

	}
	return e;
kfree_pkt:
	kfree(pkt);
	return e;
}

static int dphdcp_dev_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *miscdev = filp->private_data;
	struct tegra_dphdcp *dphdcp =
		container_of(miscdev, struct tegra_dphdcp, miscdev);
	filp->private_data = dphdcp;
	return 0;
}

static int dphdcp_dev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static const struct file_operations dphdcp_fops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.unlocked_ioctl = dphdcp_dev_ioctl,
	.open			= dphdcp_dev_open,
	.release        = dphdcp_dev_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= dphdcp_dev_ioctl,
#endif
};
/* we only support one AP right now, so should only call this once. */
struct tegra_dphdcp *tegra_dphdcp_create(struct tegra_dc_dp_data *dp,
			int id, int bus)
{
	static struct tegra_dphdcp *dphdcp; /* prevent multiple calls */
	int e;
	if (dphdcp)
		return ERR_PTR(-EMFILE);

	dphdcp = kzalloc(sizeof(*dphdcp), GFP_KERNEL);
	if (!dphdcp)
		return ERR_PTR(-ENOMEM);

	dphdcp->id = id;
	snprintf(dphdcp->name, sizeof(dphdcp->name), "nvhdcp%u", id);
	dphdcp->dp = dp;
	mutex_init(&dphdcp->lock);

	strlcpy(dphdcp->info.type, dphdcp->name, sizeof(dphdcp->info.type));
	dphdcp->bus = bus;
	dphdcp->fail_count = 0;
	dphdcp->max_retries = HDCP_MAX_RETRIES;
	dphdcp->hpd = 0;


	dphdcp->state = STATE_UNAUTHENTICATED;

	dphdcp->downstream_wq = create_singlethread_workqueue(dphdcp->name);

	dphdcp->miscdev.minor = MISC_DYNAMIC_MINOR;
	dphdcp->miscdev.name = dphdcp->name;
	dphdcp->miscdev.fops = &dphdcp_fops;
	/* register miscellaneous device */
	e = misc_register(&dphdcp->miscdev);
	if (e) {
		dphdcp_err("cannot register\n");
		goto free_workqueue;
	}

	dphdcp_vdbg("%s(): created misc device %s\n", __func__, dphdcp->name);

	return dphdcp;

free_workqueue:
	destroy_workqueue(dphdcp->downstream_wq);
	return ERR_PTR(e);
}

#ifdef CONFIG_TEGRA_DEBUG_DP_HDCP
/* show current maximum number of retries for HDCP DP authentication */
static int tegra_dp_max_retries_dbg_show(struct seq_file *m, void *unused)
{
	struct tegra_dphdcp *dphdcp = m->private;

	if (WARN_ON(!dphdcp))
		return -EINVAL;

	seq_printf(m, "hdcp max_retries value: %d\n", dphdcp->max_retries);

	return 0;
}

/* show current hotplug state */
static int tegra_dp_hotplug_dbg_show(struct seq_file *m, void *unused)
{
	struct tegra_dphdcp *dphdcp = m->private;

	if (WARN_ON(!dphdcp))
		return -EINVAL;

	seq_printf(m, "hotplug value set to: %d\n", dphdcp->hpd);

	return 0;
}

/*
 * sw control for hdcp max retries.
 * 5 is the normal number of max retries.
 * 1 is the minimum number of retries.
 * 5 is the maximum number of retries.
 * sw should keep the number of retries to 5 until unless a change is required
 */
static ssize_t tegra_dp_max_retries_dbg_write(struct file *file,
						const char __user *addr,
						size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dphdcp *dphdcp = m->private;
	u8 new_max_retries;
	int ret;

	if (WARN_ON(!dphdcp))
		return -EINVAL;

	ret = kstrtou8_from_user(addr, len, 6, &new_max_retries);
	if (ret < 0)
		return ret;

	if (new_max_retries >= HDCP_MIN_RETRIES &&
		new_max_retries <= HDCP_MAX_RETRIES)
		dphdcp->max_retries = new_max_retries;

	return len;
}

/*
 * sw control for hotplug on and off
 * 1: turn hotplug on
 * 0: turn hotplug off
 */
static ssize_t tegra_dp_hotplug_dbg_write(struct file *file,
					const char __user *addr,
					size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dphdcp *dphdcp = m->private;
	u8 new_hpd;
	int ret;

	if (WARN_ON(!dphdcp))
		return -EINVAL;

	ret = kstrtou8_from_user(addr, len, 6, &new_hpd);
	if (ret < 0)
		return ret;

	if (new_hpd > 0) {
		/* start downstream_worker */
		dphdcp_set_plugged(dphdcp, true);
		tegra_dphdcp_on(dphdcp);
		dphdcp->hpd = new_hpd;
	}
	return len;
}

static int tegra_dp_max_retries_dbg_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, tegra_dp_max_retries_dbg_show,
			inode->i_private);
}

static int tegra_dp_hotplug_dbg_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, tegra_dp_hotplug_dbg_show,
			inode->i_private);
}

static const struct file_operations tegra_dp_max_retries_dbg_ops = {
	.open = tegra_dp_max_retries_dbg_open,
	.read = seq_read,
	.write = tegra_dp_max_retries_dbg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations tegra_dp_hotplug_dbg_ops = {
	.open = tegra_dp_hotplug_dbg_open,
	.read = seq_read,
	.write = tegra_dp_hotplug_dbg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void tegra_dphdcp_debugfs_init(struct tegra_dphdcp *dphdcp)
{
	struct dentry *dir, *ret;

	dir = debugfs_create_dir("tegra_dphdcp",  NULL);

	ret = debugfs_create_file("max_retries", S_IRUGO, dir,
				dphdcp, &tegra_dp_max_retries_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;

	ret = debugfs_create_file("hotplug", S_IRUGO, dir,
				dphdcp, &tegra_dp_hotplug_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;

	return;
fail:
	debugfs_remove_recursive(dir);
	return;
}
#else
void tegra_dphdcp_debugfs_init(struct tegra_dphdcp *dphdcp)
{
	return;
}
#endif
