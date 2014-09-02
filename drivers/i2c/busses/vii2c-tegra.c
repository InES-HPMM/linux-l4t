/*
 * drivers/i2c/busses/vii2c-tegra.c
 *
 * Copyright (C) 2014 NVIDIA Corporation.  All rights reserved.
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/i2c-tegra.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>
#include <linux/module.h>
#include <linux/clk/tegra.h>
#include <linux/spinlock.h>
#include <linux/clk/tegra.h>
#include <linux/tegra-pm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/vii2c.h>

#include <asm/unaligned.h>

#include "../../../arch/arm/mach-tegra/iomap.h"

#define I2C_ERR_NONE				0x00
#define I2C_ERR_NO_ACK				0x01
#define I2C_ERR_ARBITRATION_LOST		0x02
#define I2C_ERR_UNKNOWN_INTERRUPT		0x04
#define I2C_ERR_UNEXPECTED_STATUS		0x08

#define TEGRA_VII2C_TIMEOUT			(msecs_to_jiffies(1000))
#define TEGRA_VII2C_RETRIES			3
#define TEGRA_VII2C_REG_ADDR			0x20

#define VII2C_VI_I2C_INCR_SYNCPT_0				0x000

#define VII2C_I2C_STREAM_DIRECT_I2C_CLK_DIVISOR_REGISTER_0	(0x36C << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_INTERFACE_TIMING_0_0	(0x394 << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_INTERFACE_TIMING_1_0	(0x398 << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_HS_INTERFACE_TIMING_0_0	(0x39C << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_HS_INTERFACE_TIMING_1_0	(0x3A0 << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_BUS_CLEAR_CONFIG_0		(0x384 << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_TLOW_SEXT_0			(0x334 << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_CNFG_0			(0x300 << 2)
#define VII2C_I2C_STREAM_DIRECT_INTERRUPT_MASK_REGISTER_0	(0x364 << 2)
#define VII2C_I2C_STREAM_DIRECT_INTERRUPT_STATUS_REGISTER_0	(0x368 << 2)
#define VII2C_I2C_STREAM_DIRECT_FIFO_CONTROL_0			(0x35C << 2)
#define I2C_FIFO_CONTROL_TX_FLUSH				BIT(1)
#define I2C_FIFO_CONTROL_RX_FLUSH				BIT(0)
#define I2C_FIFO_CONTROL_TX_TRIG_SHIFT				5
#define I2C_FIFO_CONTROL_RX_TRIG_SHIFT				2
#define VII2C_I2C_STREAM_DIRECT_I2C_CONFIG_LOAD_0		(0x38C << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0		(0x350 << 2)
#define VII2C_I2C_STREAM_DIRECT_STREAM_DIRECT_FENCE_0		(0x3AC << 2)

#define VII2C_I2C_STREAM_A_I2C_CLK_DIVISOR_REGISTER_0		(0x16c << 2)
#define VII2C_I2C_STREAM_A_I2C_INTERFACE_TIMING_0_0		(0x194 << 2)
#define VII2C_I2C_STREAM_A_I2C_INTERFACE_TIMING_1_0		(0x198 << 2)
#define VII2C_I2C_STREAM_A_I2C_HS_INTERFACE_TIMING_0_0		(0x19c << 2)
#define VII2C_I2C_STREAM_A_I2C_HS_INTERFACE_TIMING_1_0		(0x1a0 << 2)
#define VII2C_I2C_STREAM_A_I2C_BUS_CLEAR_CONFIG_0		(0x184 << 2)
#define VII2C_I2C_STREAM_A_I2C_TLOW_SEXT_0			(0x134 << 2)
#define VII2C_I2C_STREAM_A_I2C_CNFG_0				(0x100 << 2)
#define VII2C_I2C_STREAM_A_INTERRUPT_MASK_REGISTER_0		(0x164 << 2)
#define VII2C_I2C_STREAM_A_INTERRUPT_STATUS_REGISTER_0		(0x168 << 2)
#define VII2C_I2C_STREAM_A_FIFO_CONTROL_0			(0x15c << 2)
#define VII2C_I2C_STREAM_A_I2C_CONFIG_LOAD_0			(0x18c << 2)
#define VII2C_I2C_STREAM_A_I2C_TX_PACKET_FIFO_0			(0x150 << 2)
#define VII2C_I2C_STREAM_A_STREAM_A_FENCE_0			(0x1ac << 2)
#define VII2C_I2C_STREAM_DIRECT_I2C_RX_FIFO_0			(0x354 << 2)

#define VII2C_INT_MASK_0					0x30
#define VII2C_PROTOCOL_HEADER					0x20
#define VII2C_I2C_RESERVED_0_0					0x5c

#define VII2C_WR(r, v)				i2c_writel(i2c_dev, r, v)
/*#define VII2C_WR(r, v)	*((u32 *)(i2c_dev->nvhost->base + r)) = v*/
#define VII2C_RD(r)				i2c_readl(i2c_dev, r)

struct tegra_vii2c_platform_data {
	int retries;
	int timeout;	/* in jiffies */
	u16 slave_addr;
	int scl_gpio;
	int sda_gpio;
	bool gpio_mode;
};

/*
 * msg_end_type: The bus control which need to be send at end of transfer.
 * @MSG_END_STOP: Send stop pulse at end of transfer.
 * @MSG_END_REPEAT_START: Send repeat start at end of transfer.
 * @MSG_END_CONTINUE: The following on message is coming and so do not send
 *		stop or repeat start.
 */
enum msg_end_type {
	MSG_END_STOP,
	MSG_END_REPEAT_START,
	MSG_END_CONTINUE,
};

/**
 * struct tegra_vii2c_dev - per device i2c context
 * @dev: device reference for power management
 * @adapter: core i2c layer adapter information
 * @div_clk: clock reference for div clock of i2c controller.
 * @fast_clk: clock reference for fast clock of i2c controller.
 * @base: ioremapped registers cookie
 * @cont_id: i2c controller id, used for for packet header
 * @irq: irq number of transfer complete interrupt
 * @is_dvc: identifies the DVC i2c controller, has a different register layout
 * @msg_complete: transfer completion notifier
 * @msg_err: error code for completed message
 * @msg_buf: pointer to current message data
 * @msg_buf_remaining: size of unsent data in the message buffer
 * @msg_read: identifies read transfers
 * @bus_clk_rate: current i2c bus clock rate
 * @is_suspended: prevents i2c controller accesses after suspend is called
 */
struct tegra_vii2c_dev {
	struct i2c_adapter adapter;
	struct platform_device *pdev;
	struct tegra_vii2c_platform_data *pdata;
	struct nvhost_vii2c_struct *nvhost;
	struct device *dev;
	spinlock_t fifo_lock;
	spinlock_t mem_lock;
	int cont_id;
	struct completion msg_complete;
	int msg_err;
	int next_msg_err;
	u8 *msg_buf;
	u8 *next_msg_buf;
	u32 packet_header;
	u32 next_packet_header;
	u32 payload_size;
	u32 next_payload_size;
	u32 io_header;
	u32 next_io_header;
	size_t msg_buf_remaining;
	size_t next_msg_buf_remaining;
	int msg_read;
	int next_msg_read;
	struct i2c_msg *msgs;
	int msg_add;
	int next_msg_add;
	int msgs_num;
	bool is_suspended;
	int scl_gpio;
	int sda_gpio;
	u16 slave_addr;
	bool use_single_xfer_complete;
	struct i2c_algo_bit_data bit_data;
	const struct i2c_algorithm *bit_algo;
	bool is_shutdown;
	struct notifier_block pm_nb;
	bool bit_banging_xfer_only;
};

static inline void tegra_vii2c_gpio_setscl(void *data, int state)
{
	struct tegra_vii2c_dev *i2c_dev = data;

	gpio_set_value(i2c_dev->scl_gpio, state);
}

static inline int tegra_vii2c_gpio_getscl(void *data)
{
	struct tegra_vii2c_dev *i2c_dev = data;

	return gpio_get_value(i2c_dev->scl_gpio);
}

static inline void tegra_vii2c_gpio_setsda(void *data, int state)
{
	struct tegra_vii2c_dev *i2c_dev = data;

	gpio_set_value(i2c_dev->sda_gpio, state);
}

static inline int tegra_vii2c_gpio_getsda(void *data)
{
	struct tegra_vii2c_dev *i2c_dev = data;

	return gpio_get_value(i2c_dev->sda_gpio);
}

static int tegra_vii2c_gpio_request(struct tegra_vii2c_dev *i2c_dev)
{
	int ret;

	ret = gpio_request_one(i2c_dev->scl_gpio,
				GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN,
				"i2c-gpio-scl");
	if (ret < 0) {
		dev_err(i2c_dev->dev, "GPIO request for gpio %d failed %d\n",
				i2c_dev->scl_gpio, ret);
		return ret;
	}

	ret = gpio_request_one(i2c_dev->sda_gpio,
				GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN,
				"i2c-gpio-sda");
	if (ret < 0) {
		dev_err(i2c_dev->dev, "GPIO request for gpio %d failed %d\n",
				i2c_dev->sda_gpio, ret);
		gpio_free(i2c_dev->scl_gpio);
		return ret;
	}
	return ret;
}

static void tegra_vii2c_gpio_free(struct tegra_vii2c_dev *i2c_dev)
{
	gpio_free(i2c_dev->scl_gpio);
	gpio_free(i2c_dev->sda_gpio);
}

static int tegra_vii2c_gpio_xfer(struct i2c_adapter *adap,
	struct i2c_msg msgs[], int num)
{
	struct tegra_vii2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int ret;

	if (i2c_dev->scl_gpio <= 0 ||
		i2c_dev->sda_gpio <= 0 ||
		i2c_dev->scl_gpio == i2c_dev->sda_gpio)
		return -ENODEV;

	ret = tegra_vii2c_gpio_request(i2c_dev);
	if (ret < 0)
		return ret;

	ret = i2c_dev->bit_algo->master_xfer(adap, msgs, num);
	if (ret < 0)
		dev_err(i2c_dev->dev, "i2c-bit-algo xfer failed %d\n", ret);

	tegra_vii2c_gpio_free(i2c_dev);
	return ret;
}

static int tegra_vii2c_gpio_init(struct tegra_vii2c_dev *i2c_dev)
{
	struct i2c_algo_bit_data *bit_data = &i2c_dev->bit_data;

	if (i2c_dev->scl_gpio <= 0 || i2c_dev->sda_gpio <= 0)
		return -ENODEV;

	bit_data->setsda = tegra_vii2c_gpio_setsda;
	bit_data->getsda = tegra_vii2c_gpio_getsda;
	bit_data->setscl = tegra_vii2c_gpio_setscl;
	bit_data->getscl = tegra_vii2c_gpio_getscl;
	bit_data->data = i2c_dev;
	bit_data->udelay = 20; /* 50KHz */
	bit_data->timeout = HZ; /* 10 ms*/
	i2c_dev->bit_algo = &i2c_bit_algo;
	i2c_dev->adapter.algo_data = bit_data;
	return 0;
}

/*
 * i2c_writel and i2c_readl will offset the register if necessary to talk
 * to the I2C block inside the DVC block
 */
static void i2c_writel(struct tegra_vii2c_dev *i2c_dev,
	unsigned long reg, u32 val)
{
#if 0
	writel(val, i2c_dev->nvhost->base + reg);

	/* Read back register to make sure that register writes completed */
	if (reg != VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0)
		readw(i2c_dev->nvhost->base + reg);
#else
	struct nvhost_vii2c_struct *nvhost = i2c_dev->nvhost;
	nvhost->write_reg(reg, val);

	/* Read back register to make sure that register writes completed */
	if ((reg != VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0) &&
		(reg != VII2C_I2C_STREAM_DIRECT_STREAM_DIRECT_FENCE_0))
		nvhost->read_reg(reg);
#endif
}

static u32 i2c_readl(struct tegra_vii2c_dev *i2c_dev, unsigned long reg)
{
#if 0
	return readw(i2c_dev->nvhost->base + reg);
#else
	struct nvhost_vii2c_struct *nvhost = i2c_dev->nvhost;
	return nvhost->read_reg(reg);
#endif
}

static void tegra_vii2c_setup_wr(struct tegra_vii2c_dev *i2c_dev)
{
	dev_dbg(i2c_dev->dev, "%s +++\n", __func__);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_CLK_DIVISOR_REGISTER_0, 0x200001);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_INTERFACE_TIMING_0_0, 0x204);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_INTERFACE_TIMING_1_0, 0x04070404);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_HS_INTERFACE_TIMING_0_0, 0x308);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_HS_INTERFACE_TIMING_1_0, 0x0B0B0B);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_BUS_CLEAR_CONFIG_0, 0x90004);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TLOW_SEXT_0, 0x0);
	usleep_range(1000, 1020);
	dev_dbg(i2c_dev->dev, "%s done.\n", __func__);
}

static int tegra_vii2c_clock_enable(struct tegra_vii2c_dev *i2c_dev)
{
	return 0;
}

static int tegra_vii2c_clock_disable(struct tegra_vii2c_dev *i2c_dev)
{
	return 0;
}

int vii2c_direct_read(
	struct tegra_vii2c_dev *i2c_dev, struct i2c_msg *msg, u8 *val)
{
	struct nvhost_vii2c_struct *nvhost = i2c_dev->nvhost;
	int slv = (msg->addr & 0x7f) << 1;
	int addrn;
	int err = 0;

	addrn = msg->buf[1];
	addrn = (addrn << 8) + msg->buf[0];
	dev_dbg(i2c_dev->dev, "%s: %02x %04x\n", __func__, slv, addrn);

	VII2C_WR(VII2C_VI_I2C_INCR_SYNCPT_0, nvhost->syncpt_inc(nvhost, 8));

	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_CNFG_0, 0x00020D00);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_INTERRUPT_MASK_REGISTER_0, 0x1FF30FFF);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_INTERRUPT_STATUS_REGISTER_0,
		0x1FF30FFF);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_FIFO_CONTROL_0, 0x000000E0);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_CONFIG_LOAD_0, 0x00000007);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, 0x00010010);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, 0x00000001);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, slv);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, addrn);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, 0x00010010);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, 0x00000000);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, BIT(19) + slv);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_STREAM_DIRECT_FENCE_0, 0x0000AC00);

	err = nvhost->syncpt_wait(nvhost);
	if (err)
		return err;

	*val = VII2C_RD(VII2C_I2C_STREAM_DIRECT_I2C_RX_FIFO_0) & 0xFF;

	dev_dbg(i2c_dev->dev, "%s: %02x\n", __func__, *val);
	return 0;
}

static int vii2c_direct_write(
	struct tegra_vii2c_dev *i2c_dev, struct i2c_msg *msg)
{
	struct nvhost_vii2c_struct *nvhost = i2c_dev->nvhost;
	int slv = (msg->addr & 0x7f) << 1;
	u32 payload = msg->buf[2];
	int err = 0;

	payload = (payload << 8) + msg->buf[1];
	payload = (payload << 8) + msg->buf[0];
	dev_dbg(i2c_dev->dev, "%s: %04x %08x\n", __func__, slv, payload);


	VII2C_WR(VII2C_VI_I2C_INCR_SYNCPT_0, nvhost->syncpt_inc(nvhost, 5));

	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_CNFG_0, 0x00020D00);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_INTERRUPT_MASK_REGISTER_0, 0x1FF30FFF);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_INTERRUPT_STATUS_REGISTER_0,
		0x1FF30FFF);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_FIFO_CONTROL_0, 0x000000E0);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_CONFIG_LOAD_0, 0x00000007);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, 0x00010010);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, 0x00000002);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, slv);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_I2C_TX_PACKET_FIFO_0, payload);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_STREAM_DIRECT_FENCE_0, 0x0000AC00);
	/*usleep_range(1000, 1020);*/

	err = nvhost->syncpt_wait(nvhost);
	if (err)
		return err;

	return 0;
}

static int tegra_vii2c_flush_fifos(struct tegra_vii2c_dev *i2c_dev)
{
	unsigned long timeout = jiffies + HZ;

	u32 val = VII2C_RD(VII2C_I2C_STREAM_DIRECT_FIFO_CONTROL_0);
	dev_dbg(i2c_dev->dev, "%s: %08x\n", __func__, val);
	val |= I2C_FIFO_CONTROL_TX_FLUSH | I2C_FIFO_CONTROL_RX_FLUSH;
	dev_dbg(i2c_dev->dev, "%s: %08x\n", __func__, val);
	VII2C_WR(VII2C_I2C_STREAM_DIRECT_FIFO_CONTROL_0, val);

	while (VII2C_RD(VII2C_I2C_STREAM_DIRECT_FIFO_CONTROL_0) &
		(I2C_FIFO_CONTROL_TX_FLUSH | I2C_FIFO_CONTROL_RX_FLUSH)) {
		if (time_after(jiffies, timeout)) {
			dev_warn(i2c_dev->dev, "fifo flush timeout\n");
			return -ETIMEDOUT;
		}
		usleep_range(1000, 1020);
	}
	return 0;
}

static int tegra_vii2c_fill_tx_fifo(struct tegra_vii2c_dev *i2c_dev)
{
	return 0;
}

static void tegra_vii2c_slave_init(struct tegra_vii2c_dev *i2c_dev)
{
}

static int tegra_vii2c_init(struct tegra_vii2c_dev *i2c_dev)
{
	return 0;
}

static int tegra_vii2c_copy_next_to_current(struct tegra_vii2c_dev *i2c_dev)
{
	i2c_dev->msg_buf = i2c_dev->next_msg_buf;
	i2c_dev->msg_buf_remaining = i2c_dev->next_msg_buf_remaining;
	i2c_dev->msg_err = i2c_dev->next_msg_err;
	i2c_dev->msg_read = i2c_dev->next_msg_read;
	i2c_dev->msg_add = i2c_dev->next_msg_add;
	i2c_dev->packet_header = i2c_dev->next_packet_header;
	i2c_dev->io_header = i2c_dev->next_io_header;
	i2c_dev->payload_size = i2c_dev->next_payload_size;

	return 0;
}

static int tegra_vii2c_send_next_read_msg_pkt_header(
	struct tegra_vii2c_dev *i2c_dev,
	struct i2c_msg *next_msg,
	enum msg_end_type end_state)
{
	return 0;
}

#ifdef TEGRA_VII2C_DEBUG_DUMP
static void dump_msg(struct i2c_msg *msg)
{
	int i;
	int size;
	char buf[32];
	char *ptr = buf;

	memset(buf, 0, sizeof(buf));
	size = sizeof(buf) - 1;
	for (i = 0; i < msg->len && size; i++) {
		snprintf(ptr, size, " %02x", msg->buf[i]);
		ptr += strlen(ptr);
		size -= strlen(ptr);
	}
	pr_info("%s\n", buf);
}
#else
static inline void dump_msg(struct i2c_msg *msg) {}
#endif

static int tegra_vii2c_xfer_msg(struct tegra_vii2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_state,
	struct i2c_msg *next_msg, enum msg_end_type next_msg_end_state)
{
	if (msg->len == 0)
		return -EINVAL;

	dev_dbg(i2c_dev->dev, "%s %x %x %d\n",
		__func__, msg->addr, msg->flags, msg->len);
	dump_msg(msg);

	i2c_dev->msg_buf = msg->buf;
	i2c_dev->msg_buf_remaining = msg->len;
	i2c_dev->msg_err = I2C_ERR_NONE;
	i2c_dev->msg_read = (msg->flags & I2C_M_RD);
	INIT_COMPLETION(i2c_dev->msg_complete);

	if (msg->flags & I2C_M_RD)
		vii2c_direct_read(i2c_dev, msg, &msg->buf[2]);
	else if (next_msg && (next_msg->flags & I2C_M_RD))
		vii2c_direct_read(i2c_dev, msg, next_msg->buf);
	else {
		tegra_vii2c_flush_fifos(i2c_dev);
		vii2c_direct_write(i2c_dev, msg);
	}

	return 0;
}

static int tegra_vii2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
	int num)
{
	struct tegra_vii2c_dev *i2c_dev = i2c_get_adapdata(adap);
	struct nvhost_vii2c_struct *nvhost;
	int i;
	int ret = 0;

	dev_dbg(i2c_dev->dev, "\n%s %d messages\n", __func__, num);

	BUG_ON(!rt_mutex_is_locked(&(adap->bus_lock)));
	if (i2c_dev->is_suspended)
		return -EBUSY;

	if (i2c_dev->is_shutdown || adap->atomic_xfer_only ||
		i2c_dev->bit_banging_xfer_only)
		return tegra_vii2c_gpio_xfer(adap, msgs, num);

	if (adap->atomic_xfer_only)
		return -EBUSY;

	i2c_dev->msgs = msgs;
	i2c_dev->msgs_num = num;

	pm_runtime_get_sync(&adap->dev);
	ret = tegra_vii2c_clock_enable(i2c_dev);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "Clock enable failed %d\n", ret);
		pm_runtime_put(&adap->dev);
		return ret;
	}

	if (!i2c_dev->nvhost) {
		nvhost = nvhost_vii2c_open(i2c_dev->pdev);
		if ((nvhost == NULL) || IS_ERR(nvhost)) {
			dev_err(i2c_dev->dev,
				"%s: nvhost not ready yet\n", __func__);
			return -ENOTTY;
		}
		i2c_dev->nvhost = nvhost;
		nvhost->base = IO_ADDRESS(0x546c0000);
	}
	nvhost = i2c_dev->nvhost;
	nvhost->open(nvhost);

	/* keep module powered */
	ret = nvhost->module_start(nvhost);
	if (ret)
		return -EBUSY;

	tegra_vii2c_setup_wr(i2c_dev);

	for (i = 0; i < num; i++) {
		enum msg_end_type end_type = MSG_END_STOP;
		enum msg_end_type n_msg_end_type = MSG_END_STOP;

		if (i < (num - 1)) {
			if (msgs[i + 1].flags & I2C_M_NOSTART)
				end_type = MSG_END_CONTINUE;
			else
				end_type = MSG_END_REPEAT_START;
			if (i < num - 2) {
				if (msgs[i + 2].flags & I2C_M_NOSTART)
					n_msg_end_type = MSG_END_CONTINUE;
				else
					n_msg_end_type = MSG_END_REPEAT_START;
			}
			if ((!(msgs[i].flags & I2C_M_RD)) &&
				(msgs[i].len <= 8) &&
				(msgs[i+1].flags & I2C_M_RD) &&
				(n_msg_end_type != MSG_END_CONTINUE) &&
				(end_type == MSG_END_REPEAT_START)) {
				ret = tegra_vii2c_xfer_msg(i2c_dev, &msgs[i],
					end_type, &msgs[i+1], n_msg_end_type);
				if (ret)
					break;
				i++;
			} else {
				ret = tegra_vii2c_xfer_msg(i2c_dev, &msgs[i],
					end_type, NULL, n_msg_end_type);
				if (ret)
					break;
			}
		} else {
			ret = tegra_vii2c_xfer_msg(i2c_dev, &msgs[i], end_type,
				NULL, n_msg_end_type);
			if (ret)
				break;
		}
	}

	nvhost->module_idle(nvhost);
	nvhost->close(nvhost);

	tegra_vii2c_clock_disable(i2c_dev);
	pm_runtime_put(&adap->dev);

	i2c_dev->msgs = NULL;
	i2c_dev->msgs_num = 0;

	return ret ?: i;
}

static u32 tegra_vii2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm tegra_vii2c_algo = {
	.master_xfer	= tegra_vii2c_xfer,
	.functionality	= tegra_vii2c_func,
};

static int __tegra_vii2c_suspend_noirq(struct tegra_vii2c_dev *i2c_dev);
static int __tegra_vii2c_resume_noirq(struct tegra_vii2c_dev *i2c_dev);

static int tegra_vii2c_pm_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct tegra_vii2c_dev *i2c_dev =
		container_of(nb, struct tegra_vii2c_dev, pm_nb);

	if (event == TEGRA_PM_SUSPEND)
		__tegra_vii2c_suspend_noirq(i2c_dev);
	else if (event == TEGRA_PM_RESUME)
		__tegra_vii2c_resume_noirq(i2c_dev);

	return NOTIFY_OK;
}

static struct tegra_vii2c_platform_data *parse_vii2c_tegra_dt(
	struct platform_device *pdev)
{
	struct tegra_vii2c_platform_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	const char *sname;

	dev_dbg(&pdev->dev, "%s: %s\n", __func__, np->full_name);
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	sname = of_get_property(np, "i2c_use_gpio", NULL);
	if (sname && !strcmp(sname, "yes")) {
		dev_info(&pdev->dev, "%s: gpio mode enabled.\n",
			__func__, np->full_name);
		pdata->gpio_mode = true;
	}
	pdata->scl_gpio = of_get_named_gpio(np, "scl-gpio", 0);
	pdata->sda_gpio = of_get_named_gpio(np, "sda-gpio", 0);
	/* Default configuration for device tree initiated driver */
	pdata->slave_addr = 0xFC;
	return pdata;
}

/* Match table for of_platform binding */
static const struct of_device_id tegra_vii2c_of_match[] = {
	{
		.compatible = "nvidia,tegra210-vii2c",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tegra_vii2c_of_match);

static struct platform_device_id tegra_vii2c_devtype[] = {
	{
		.name = "tegra21-vii2c",
	},
};

static int tegra_vii2c_probe(struct platform_device *pdev)
{
	struct tegra_vii2c_dev *i2c_dev  = NULL;
	struct tegra_vii2c_platform_data *pdata = NULL;
	int ret = 0;
	const struct of_device_id *match = NULL;
	int bus_num = -1;

	dev_info(&pdev->dev, "%s %p\n", __func__, pdev->dev.of_node);
	if (pdev->dev.of_node) {
		match = of_match_device(of_match_ptr(tegra_vii2c_of_match),
			&pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Device Not matching\n");
			return -ENODEV;
		}
		pdata = parse_vii2c_tegra_dt(pdev);
	} else {
		bus_num = pdev->id;
	}

	if (IS_ERR(pdata) || !pdata) {
		dev_err(&pdev->dev, "no platform/chip data?\n");
		return IS_ERR(pdata) ? PTR_ERR(pdata) : -ENODEV;
	}

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev) {
		dev_err(&pdev->dev, "Could not alloc struct tegra_vii2c_dev");
		return -ENOMEM;
	}

	i2c_dev->cont_id = pdev->id;
	i2c_dev->pdev = pdev;
	i2c_dev->dev = &pdev->dev; /* for dev_info */
	i2c_dev->msgs = NULL;
	i2c_dev->msgs_num = 0;
	i2c_dev->scl_gpio = pdata->scl_gpio;
	i2c_dev->sda_gpio = pdata->sda_gpio;
	i2c_dev->bit_banging_xfer_only = pdata->gpio_mode;
	i2c_dev->slave_addr = pdata->slave_addr;
	i2c_dev->pdata = pdata;
	init_completion(&i2c_dev->msg_complete);

	spin_lock_init(&i2c_dev->fifo_lock);

	spin_lock_init(&i2c_dev->mem_lock);

	platform_set_drvdata(pdev, i2c_dev);

	ret = tegra_vii2c_init(i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize i2c controller");
		return ret;
	}

	pm_runtime_enable(&pdev->dev);

	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);
	i2c_dev->adapter.owner = THIS_MODULE;
	i2c_dev->adapter.class = I2C_CLASS_HWMON;
	strlcpy(i2c_dev->adapter.name, "Tegra VII2C adapter",
		sizeof(i2c_dev->adapter.name));
	i2c_dev->adapter.algo = &tegra_vii2c_algo;
	i2c_dev->adapter.dev.parent = &pdev->dev;
	i2c_dev->adapter.nr = bus_num;
	i2c_dev->adapter.dev.of_node = pdev->dev.of_node;

	if (pdata->retries)
		i2c_dev->adapter.retries = pdata->retries;
	else
		i2c_dev->adapter.retries = TEGRA_VII2C_RETRIES;

	if (pdata->timeout)
		i2c_dev->adapter.timeout = pdata->timeout;

	ret = i2c_add_numbered_adapter(&i2c_dev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add I2C adapter\n");
		return ret;
	}

	i2c_dev->pm_nb.notifier_call = tegra_vii2c_pm_notifier;

	tegra_register_pm_notifier(&i2c_dev->pm_nb);

	of_i2c_register_devices(&i2c_dev->adapter);
	pm_runtime_enable(&i2c_dev->adapter.dev);
	tegra_vii2c_gpio_init(i2c_dev);

	dev_info(&pdev->dev, "%s initialized\n", __func__);
	return 0;
}

static int tegra_vii2c_remove(struct platform_device *pdev)
{
	struct tegra_vii2c_dev *i2c_dev = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s\n", __func__);
	tegra_unregister_pm_notifier(&i2c_dev->pm_nb);
	i2c_del_adapter(&i2c_dev->adapter);
	pm_runtime_disable(&i2c_dev->adapter.dev);
	pm_runtime_disable(&pdev->dev);
	nvhost_vii2c_close(i2c_dev->nvhost);
	return 0;
}

static void tegra_vii2c_shutdown(struct platform_device *pdev)
{
	struct tegra_vii2c_dev *i2c_dev = platform_get_drvdata(pdev);

	dev_info(i2c_dev->dev, "Bus is shutdown down..\n");
	i2c_shutdown_adapter(&i2c_dev->adapter);
	nvhost_vii2c_close(i2c_dev->nvhost);
	i2c_dev->is_shutdown = true;
}

#ifdef CONFIG_PM_SLEEP
static int __tegra_vii2c_suspend_noirq(struct tegra_vii2c_dev *i2c_dev)
{
	i2c_dev->is_suspended = true;

	return 0;
}

static int tegra_vii2c_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_vii2c_dev *i2c_dev = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s\n", __func__);
	i2c_lock_adapter(&i2c_dev->adapter);

	__tegra_vii2c_suspend_noirq(i2c_dev);

	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}


static int __tegra_vii2c_resume_noirq(struct tegra_vii2c_dev *i2c_dev)
{
	int ret;

	ret = tegra_vii2c_init(i2c_dev);
	if (ret)
		return ret;

	i2c_dev->is_suspended = false;

	return 0;
}

static int tegra_vii2c_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_vii2c_dev *i2c_dev = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s\n", __func__);
	i2c_lock_adapter(&i2c_dev->adapter);

	__tegra_vii2c_resume_noirq(i2c_dev);

	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static const struct dev_pm_ops tegra_vii2c_pm = {
	.suspend_noirq = tegra_vii2c_suspend_noirq,
	.resume_noirq = tegra_vii2c_resume_noirq,
};
#define TEGRA_VII2C_PM	(&tegra_vii2c_pm)
#else
#define TEGRA_VII2C_PM	NULL
#endif

static struct platform_driver tegra_vii2c_driver = {
	.probe   = tegra_vii2c_probe,
	.remove  = tegra_vii2c_remove,
	.shutdown = tegra_vii2c_shutdown,
	.id_table = tegra_vii2c_devtype,
	.driver  = {
		.name  = "tegra-vii2c",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_vii2c_of_match),
		.pm    = TEGRA_VII2C_PM,
	},
};

static int __init tegra_vii2c_init_driver(void)
{
	return platform_driver_register(&tegra_vii2c_driver);
}

static void __exit tegra_vii2c_exit_driver(void)
{
	platform_driver_unregister(&tegra_vii2c_driver);
}

subsys_initcall(tegra_vii2c_init_driver);
module_exit(tegra_vii2c_exit_driver);

MODULE_DESCRIPTION("nVidia Tegra2 VII2C Bus Controller driver");
MODULE_LICENSE("GPL v2");
