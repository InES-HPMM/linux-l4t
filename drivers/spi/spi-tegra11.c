/*
 * Driver for Nvidia TEGRA spi controller in slave mode.
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

/*#define DEBUG		   1*/
/*#define VERBOSE_DEBUG   1*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/spi/spi.h>
#include <linux/spi-tegra.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/module.h>
#include <mach/iomap.h>
#include <mach/dma.h>
#include <mach/clk.h>
#include <mach/hardware.h>

#define SPI_COMMAND1		0x000
#define   SPI_BIT_LENGTH(x)		(((x) & 0x1f) << 0)
#define   SPI_PACKED			(1 << 5)
#define   SPI_TX_EN			(1 << 11)
#define   SPI_RX_EN			(1 << 12)
#define   SPI_BOTH_EN_BYTE		(1 << 13)
#define   SPI_BOTH_EN_BIT			(1 << 14)
#define   SPI_LSBYTE_FE			(1 << 15)
#define   SPI_LSBIT_FE			(1 << 16)
#define   SPI_BIDIROE				(1 << 17)
#define   SPI_IDLE_SDA_DRIVE_LOW	(0 << 18)
#define   SPI_IDLE_SDA_DRIVE_HIGH	(1 << 18)
#define   SPI_IDLE_SDA_PULL_LOW	(2 << 18)
#define   SPI_IDLE_SDA_PULL_HIGH	(3 << 18)
#define   SPI_IDLE_SDA_MASK		(3 << 18)
#define   SPI_CS_SS_VAL			(1 << 20)
#define   SPI_CS_SW_HW			(1 << 21)
/* SPI_CS_POL_INACTIVE bits are default high */
#define   SPI_CS_POL_INACTIVE_0 (1 << 22)
#define   SPI_CS_POL_INACTIVE_1 (1 << 23)
#define   SPI_CS_POL_INACTIVE_2 (1 << 24)
#define   SPI_CS_POL_INACTIVE_3 (1 << 25)
#define   SPI_CS_POL_INACTIVE_MASK (0xF << 22)
#define   SPI_CS_POL_INACTIVE_SET_LOW(reg, cs)	 \
					(reg &= ~((1 << cs) << 22))
#define   SPI_CS_SEL_0		(0 << 26)
#define   SPI_CS_SEL_1		(1 << 26)
#define   SPI_CS_SEL_2		(2 << 26)
#define   SPI_CS_SEL_3		(3 << 26)
#define   SPI_CS_SEL_MASK		(3 << 26)
#define   SPI_CS_SEL(x)		(((x) & 0x3) << 26)
#define   SPI_CONTROL_MODE_0			(0 << 28)
#define   SPI_CONTROL_MODE_1			(1 << 28)
#define   SPI_CONTROL_MODE_2			(2 << 28)
#define   SPI_CONTROL_MODE_3			(3 << 28)
#define   SPI_CONTROL_MODE_MASK		(3 << 28)
#define   SPI_MODE_SEL(x)		(((x) & 0x3) << 28)
#define   SPI_M_S			(1 << 30)
#define   SPI_PIO			(1 << 31)

#define SPI_COMMAND2		0x004
#define   SPI_TX_TAP_DELAY(x)		(((x) & 0x3F) << 0)
#define   SPI_RX_TAP_DELAY(x)		(((x) & 0x3F) << 6)

#define SPI_CS_TIM1		0x008
#define   SPI_CS_HOLD_TIME_0(x)	(((x) & 0xF) << 0)
#define   SPI_CS_SETUP_TIME_0(x)	(((x) & 0xF) << 4)
#define   SPI_CS_HOLD_TIME_1(x)	(((x) & 0xF) << 8)
#define   SPI_CS_SETUP_TIME_1(x)	(((x) & 0xF) << 12)
#define   SPI_CS_HOLD_TIME_2(x)	(((x) & 0xF) << 16)
#define   SPI_CS_SETUP_TIME_2(x)	(((x) & 0xF) << 20)
#define   SPI_CS_HOLD_TIME_3(x)	(((x) & 0xF) << 24)
#define   SPI_CS_SETUP_TIME_3(x)	(((x) & 0xF) << 28)
#define   SPI_SET_CS_SETUP_TIME(reg, cs, val)		\
			(reg = ((val & 0xF) << (cs*8 + 4)) |	\
			 (reg & ~(0xF << (cs*8 + 4))))
#define   SPI_SET_CS_HOLD_TIME(reg, cs, val)		\
			(reg = ((val & 0xF) << (cs*8)) |	\
			(reg & ~(0xF << (cs*8))))

#define SPI_CS_TIM2		0x00C
#define   CYCLES_BETWEEN_PACKETS_0(x)	(((x) & 0x1F) << 0)
#define   CS_ACTIVE_BETWEEN_PACKETS_0   (1 << 5)
#define   CYCLES_BETWEEN_PACKETS_1(x)	(((x) & 0x1F) << 8)
#define   CS_ACTIVE_BETWEEN_PACKETS_1   (1 << 13)
#define   CYCLES_BETWEEN_PACKETS_2(x)	(((x) & 0x1F) << 16)
#define   CS_ACTIVE_BETWEEN_PACKETS_2   (1 << 21)
#define   CYCLES_BETWEEN_PACKETS_3(x)	(((x) & 0x1F) << 24)
#define   CS_ACTIVE_BETWEEN_PACKETS_3   (1 << 29)
#define   SPI_SET_CS_ACTIVE_BETWEEN_PACKETS(reg, cs, val)		\
		   (reg = ((val & 0x1) << (cs*8 + 5)) |	\
					(reg & ~(1 << (cs*8 + 5))))
#define   SPI_SET_CYCLES_BETWEEN_PACKETS(reg, cs, val)		\
			(reg = ((val & 0xF) << (cs*8)) |	\
			(reg & ~(0xF << (cs*8))))

#define SPI_TRANS_STATUS			0x010
#define   SPI_BLK_CNT(val)			(((val) >> 0) & 0xFFFF)
#define   SPI_SLV_IDLE_COUNT(val)		((val >> 16) & 0xFF)
#define   SPI_RDY						(1 << 30)

#define SPI_FIFO_STATUS			0x014
#define   SPI_RX_FIFO_EMPTY			(1 << 0)
#define   SPI_RX_FIFO_FULL			(1 << 1)
#define   SPI_TX_FIFO_EMPTY			(1 << 2)
#define   SPI_TX_FIFO_FULL			(1 << 3)
#define   SPI_RX_FIFO_UNF			(1 << 4)
#define   SPI_RX_FIFO_OVF			(1 << 5)
#define   SPI_TX_FIFO_UNF			(1 << 6)
#define   SPI_TX_FIFO_OVF			(1 << 7)
#define   SPI_ERR				(1 << 8)
#define   SPI_TX_FIFO_FLUSH			(1 << 14)
#define   SPI_RX_FIFO_FLUSH			(1 << 15)
#define   SPI_TX_FIFO_EMPTY_COUNT(val)	((val >> 16) & 0x7F)
#define   SPI_RX_FIFO_FULL_COUNT(val)		((val >> 23) & 0x7F)
#define   SPI_FRAME_END			(1 << 30)
#define   SPI_CS_INACTIVE			(1 << 31)

#define SPI_TX_DATA			0x018
#define SPI_RX_DATA			0x01C

#define SPI_DMA_CTL			0x020
#define   SPI_TX_TRIG_1		(0 << 15)
#define   SPI_TX_TRIG_4		(1 << 15)
#define   SPI_TX_TRIG_8		(2 << 15)
#define   SPI_TX_TRIG_16		(3 << 15)
#define   SPI_TX_TRIG_MASK		(3 << 15)
#define   SPI_RX_TRIG_1		(0 << 19)
#define   SPI_RX_TRIG_4		(1 << 19)
#define   SPI_RX_TRIG_8		(2 << 19)
#define   SPI_RX_TRIG_16		(3 << 19)
#define   SPI_RX_TRIG_MASK	(3 << 19)
#define   SPI_IE_TX			(1 << 28)
#define   SPI_IE_RX			(1 << 29)
#define   SPI_CONT			(1 << 30)
#define   SPI_DMA			(1 << 31)

#define   SPI_DMA_BLK		0x024
#define   SPI_DMA_BLK_SET(x)	(((x) & 0xFFFF) << 0)

#define SPI_TX_FIFO		0x108
#define SPI_RX_FIFO		0x188
#define MAX_CHIP_SELECT		4
#define SPI_FIFO_DEPTH		32
#define DATA_DIR_TX		(1 << 0)
#define DATA_DIR_RX		(1 << 1)

#define SPI_DMA_TIMEOUT (msecs_to_jiffies(1000))
#define DEFAULT_SPI_DMA_BUF_LEN		(16*1024)
#define TX_FIFO_EMPTY_COUNT_MAX		SPI_TX_FIFO_EMPTY_COUNT(0x20)
#define RX_FIFO_FULL_COUNT_ZERO		SPI_RX_FIFO_FULL_COUNT(0)
#define MAX_HOLD_CYCLES				16

#define SPI_STATUS2_RESET \
	(TX_FIFO_EMPTY_COUNT_MAX | \
	RX_FIFO_FULL_COUNT_ZERO << 16)


static const unsigned long spi_tegra_req_sels[] = {
	TEGRA_DMA_REQ_SEL_SL2B1,
	TEGRA_DMA_REQ_SEL_SL2B2,
	TEGRA_DMA_REQ_SEL_SL2B3,
	TEGRA_DMA_REQ_SEL_SL2B4,
#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
	TEGRA_DMA_REQ_SEL_SL2B5,
	TEGRA_DMA_REQ_SEL_SL2B6,
#endif

};


struct spi_device_setting {
	u32			max_freq;
	u32			bits_per_word;
	u32			mode;
	u32			cs_hold_cycles;
	u32			cs_setup_cycles;
	u32			cycles_between_packets;
	bool		cs_active_between_packets;
	bool		has_setup_device;
};

struct spi_tegra_data {
	struct spi_master		*master;
	struct platform_device	*pdev;
	spinlock_t				lock; /* serialise operations */
	char					port_name[32];
	struct clk				*clk;
	void __iomem			*base;
	unsigned long			phys;
	unsigned				irq;
	u32						cur_speed;
	struct list_head		queue;
	struct spi_transfer		*cur;
	struct spi_device		*cur_spi;
	struct workqueue_struct *spi_workqueue;
	struct work_struct spi_transfer_work;
	unsigned				cur_pos;
	unsigned				cur_len;
	unsigned				words_per_32bit;
	unsigned				bytes_per_word;
	unsigned				curr_dma_words;
	unsigned				cur_direction;
	struct tegra_dma_req	rx_dma_req;
	struct tegra_dma_channel *rx_dma;
	u32						*rx_buf;
	dma_addr_t				rx_buf_phys;
	unsigned				cur_rx_pos;
	struct tegra_dma_req	tx_dma_req;
	struct tegra_dma_channel *tx_dma;
	u32						*tx_buf;
	dma_addr_t				tx_buf_phys;
	unsigned				cur_tx_pos;
	unsigned				dma_buf_size;
	struct completion	spi_complete;
	struct completion	rx_dma_complete;
	struct completion	tx_dma_complete;
	struct spi_device_setting spi_dev_setting[MAX_CHIP_SELECT];
	bool					is_dma_allowed;
	bool					is_curr_dma_xfer;
	bool					is_clkon_always;
	bool					clk_state;
	bool					is_suspended;
	bool					is_packed;
	bool					is_reset;
	bool					msg_in_transfer;
	u32						fifo_status;
	u32						trans_status;
};

static inline unsigned long spi_tegra_readl(struct spi_tegra_data *tspi,
			unsigned long reg)
{
	if (!tspi->clk_state)
		BUG();
	return readl(tspi->base + reg);
}

static inline void spi_tegra_writel(struct spi_tegra_data *tspi,
			unsigned long val, unsigned long reg)
{
	if (!tspi->clk_state)
		BUG();
	writel(val, tspi->base + reg);
}

static void spi_tegra_reg_print(struct spi_tegra_data *tspi)
{
	dev_info(&tspi->pdev->dev,
		"SPI_COMMAND1 = 0x%lx\n", spi_tegra_readl(tspi, SPI_COMMAND1));
	dev_info(&tspi->pdev->dev,
		"SPI_COMMAND2 = 0x%lx\n", spi_tegra_readl(tspi, SPI_COMMAND2));
	dev_info(&tspi->pdev->dev,
		"SPI_CS_TIM1 = 0x%lx\n", spi_tegra_readl(tspi, SPI_CS_TIM1));
	dev_info(&tspi->pdev->dev,
		"SPI_CS_TIM2 = 0x%lx\n", spi_tegra_readl(tspi, SPI_CS_TIM2));
	dev_info(&tspi->pdev->dev,
		"SPI_TRANS_STATUS = 0x%lx\n",
		spi_tegra_readl(tspi, SPI_TRANS_STATUS));
	dev_info(&tspi->pdev->dev,
		"SPI_FIFO_STATUS = 0x%lx\n",
		spi_tegra_readl(tspi, SPI_FIFO_STATUS));
	dev_info(&tspi->pdev->dev,
		"SPI_DMA_CTL = 0x%lx\n", spi_tegra_readl(tspi, SPI_DMA_CTL));
	dev_info(&tspi->pdev->dev,
		"SPI_BLOCK_SIZE = 0x%lx\n", spi_tegra_readl(tspi, SPI_DMA_BLK));
}

static void spi_tegra_clear_status(struct spi_tegra_data *tspi)
{
	unsigned long val;
	unsigned long clear_err = 0;

	val = spi_tegra_readl(tspi, SPI_TRANS_STATUS);
	if (val & SPI_RDY)
		spi_tegra_writel(tspi, val, SPI_TRANS_STATUS);
	/* Save transfer fifo status and clear error bits */
	val = spi_tegra_readl(tspi, SPI_FIFO_STATUS);
	tspi->fifo_status = 0;
	if (val & SPI_ERR) {
		clear_err |= SPI_ERR;
		if (val & SPI_TX_FIFO_UNF)
			clear_err |= SPI_TX_FIFO_UNF;
		if (val & SPI_TX_FIFO_OVF)
			clear_err |= SPI_TX_FIFO_OVF;
		if (val & SPI_RX_FIFO_UNF)
			clear_err |= SPI_RX_FIFO_UNF;
		if (val & SPI_RX_FIFO_OVF)
			clear_err |= SPI_RX_FIFO_OVF;
		spi_tegra_writel(tspi, clear_err, SPI_FIFO_STATUS);
	}
}

static unsigned spi_tegra_calculate_curr_xfer_param(
	struct spi_device *spi, struct spi_tegra_data *tspi,
	struct spi_transfer *t)
{
	unsigned remain_len = t->len - tspi->cur_pos;
	unsigned max_word;
	unsigned bits_per_word ;
	unsigned max_len;
	unsigned total_fifo_words;

	bits_per_word = t->bits_per_word ? t->bits_per_word :
						spi->bits_per_word;
	tspi->bytes_per_word = (bits_per_word - 1) / 8 + 1;

	if (bits_per_word == 8 || bits_per_word == 16) {
		tspi->is_packed = 1;
		tspi->words_per_32bit = 32/bits_per_word;
	} else {
		tspi->is_packed = 0;
		tspi->words_per_32bit = 1;
	}

	if (tspi->is_packed) {
		max_len = min(remain_len, tspi->dma_buf_size);
		tspi->curr_dma_words = max_len/tspi->bytes_per_word;
		total_fifo_words = remain_len/4;
	} else {
		max_word = (remain_len - 1) / tspi->bytes_per_word + 1;
		max_word = min(max_word, tspi->dma_buf_size/4);
		tspi->curr_dma_words = max_word;
		total_fifo_words = remain_len/tspi->bytes_per_word;
	}

	return total_fifo_words;
}

static unsigned spi_tegra_fill_tx_fifo_from_client_txbuf(
	struct spi_tegra_data *tspi, struct spi_transfer *t)
{
	unsigned nbytes;
	unsigned tx_empty_count;
	unsigned long fifo_status;
	u8 *tx_buf = (u8 *)t->tx_buf + tspi->cur_tx_pos;
	unsigned max_n_32bit;
	unsigned i, count;
	unsigned long x;
	unsigned int written_words;

	fifo_status = spi_tegra_readl(tspi, SPI_FIFO_STATUS);
	tx_empty_count = SPI_TX_FIFO_EMPTY_COUNT(fifo_status);
	if (tspi->is_packed) {
		nbytes = tspi->curr_dma_words * tspi->bytes_per_word;
		max_n_32bit = (min(nbytes, tx_empty_count*4) - 1)/4 + 1;
		for (count = 0; count < max_n_32bit; ++count) {
			x = 0;
			for (i = 0; (i < 4) && nbytes; i++, nbytes--)
				x |= (*tx_buf++) << (i*8);
			spi_tegra_writel(tspi, x, SPI_TX_FIFO);
		}
		written_words = min(max_n_32bit * tspi->words_per_32bit,
					tspi->curr_dma_words);
	} else {
		max_n_32bit = min(tspi->curr_dma_words, tx_empty_count);
		nbytes = max_n_32bit * tspi->bytes_per_word;
		for (count = 0; count < max_n_32bit; ++count) {
			x = 0;
			for (i = 0; nbytes && (i < tspi->bytes_per_word);
							++i, nbytes--)
				x |= (*tx_buf++) << (i*8);
			spi_tegra_writel(tspi, x, SPI_TX_FIFO);
		}
		written_words = max_n_32bit;
	}
	tspi->cur_tx_pos += written_words * tspi->bytes_per_word;
	return written_words;
}

static unsigned int spi_tegra_read_rx_fifo_to_client_rxbuf(
		struct spi_tegra_data *tspi, struct spi_transfer *t)
{
	unsigned rx_full_count;
	unsigned long fifo_status;
	u8 *rx_buf = (u8 *)t->rx_buf + tspi->cur_rx_pos;
	unsigned i, count;
	unsigned long x;
	unsigned int read_words;
	unsigned len;

	fifo_status = spi_tegra_readl(tspi, SPI_FIFO_STATUS);
	rx_full_count = SPI_RX_FIFO_FULL_COUNT(fifo_status);

	if (tspi->is_packed) {
		len = tspi->curr_dma_words * tspi->bytes_per_word;
		for (count = 0; count < rx_full_count; ++count) {
			x = spi_tegra_readl(tspi, SPI_RX_FIFO);
			for (i = 0; len && (i < 4); ++i, len--)
				*rx_buf++ = (x >> i*8) & 0xFF;
		}
		tspi->cur_rx_pos +=
			tspi->curr_dma_words * tspi->bytes_per_word;
		read_words += tspi->curr_dma_words;
	} else {
		for (count = 0; count < rx_full_count; ++count) {
			x = spi_tegra_readl(tspi, SPI_RX_FIFO);
			for (i = 0; (i < tspi->bytes_per_word); ++i)
				*rx_buf++ = (x >> (i*8)) & 0xFF;
		}
		tspi->cur_rx_pos += rx_full_count * tspi->bytes_per_word;
		read_words += rx_full_count;
	}
	return read_words;
}

static void spi_tegra_copy_client_txbuf_to_spi_txbuf(
		struct spi_tegra_data *tspi, struct spi_transfer *t)
{
	unsigned len;
	if (tspi->is_packed) {
		len = tspi->curr_dma_words * tspi->bytes_per_word;
		memcpy(tspi->tx_buf, t->tx_buf + tspi->cur_pos, len);
	} else {
		unsigned int i;
		unsigned int count;
		u8 *tx_buf = (u8 *)t->tx_buf + tspi->cur_tx_pos;
		unsigned consume = tspi->curr_dma_words * tspi->bytes_per_word;
		unsigned int x;

		for (count = 0; count < tspi->curr_dma_words; ++count) {
			x = 0;
			for (i = 0; consume && (i < tspi->bytes_per_word);
							++i, consume--)
				x |= ((*tx_buf++) << i*8);
			tspi->tx_buf[count] = x;
		}
	}
	tspi->cur_tx_pos += tspi->curr_dma_words * tspi->bytes_per_word;
}

static void spi_tegra_copy_spi_rxbuf_to_client_rxbuf(
		struct spi_tegra_data *tspi, struct spi_transfer *t)
{
	unsigned len;
	if (tspi->is_packed) {
		len = tspi->curr_dma_words * tspi->bytes_per_word;
		memcpy(t->rx_buf + tspi->cur_rx_pos, tspi->rx_buf, len);
	} else {
		unsigned int i;
		unsigned int count;
		unsigned char *rx_buf = t->rx_buf + tspi->cur_rx_pos;
		unsigned int x;
		for (count = 0; count < tspi->curr_dma_words; ++count) {
			x = tspi->rx_buf[count];
			for (i = 0; (i < tspi->bytes_per_word); ++i)
				*rx_buf++ = (x >> (i*8)) & 0xFF;
		}
	}
	tspi->cur_rx_pos += tspi->curr_dma_words * tspi->bytes_per_word;
}

static int spi_tegra_start_dma_based_transfer(
		struct spi_tegra_data *tspi, struct spi_transfer *t)
{
	unsigned long val;
	unsigned int len;
	int ret = 0;

	INIT_COMPLETION(tspi->rx_dma_complete);
	INIT_COMPLETION(tspi->tx_dma_complete);
	INIT_COMPLETION(tspi->spi_complete);
	val = SPI_DMA_BLK_SET(tspi->curr_dma_words - 1);
	spi_tegra_writel(tspi, val, SPI_DMA_BLK);
	if (tspi->is_packed)
		len = DIV_ROUND_UP(tspi->curr_dma_words * tspi->bytes_per_word,
					4) * 4;
	else
		len = tspi->curr_dma_words * 4;

	if (len & 0xF)
		val = SPI_TX_TRIG_1 | SPI_RX_TRIG_1;
	else if (((len) >> 4) & 0x1)
		val = SPI_TX_TRIG_4 | SPI_RX_TRIG_4;
	else
		val = SPI_TX_TRIG_8 | SPI_RX_TRIG_8;

	val &= ~(SPI_IE_TX | SPI_IE_RX);
	if (tspi->cur_direction & DATA_DIR_TX)
		val |= SPI_IE_TX;

	if (tspi->cur_direction & DATA_DIR_RX)
		val |= SPI_IE_RX;

	spi_tegra_writel(tspi, val, SPI_DMA_CTL);

	if (tspi->cur_direction & DATA_DIR_TX) {
		spi_tegra_copy_client_txbuf_to_spi_txbuf(tspi, t);
		wmb(); /* complete writes */
		tspi->tx_dma_req.size = len;
		ret = tegra_dma_enqueue_req(tspi->tx_dma, &tspi->tx_dma_req);
		if (ret < 0) {
			dev_err(&tspi->pdev->dev, "Error in starting tx dma "
						" error = %d\n", ret);
			return ret;
		}
	}

	if (tspi->cur_direction & DATA_DIR_RX) {
		tspi->rx_dma_req.size = len;
		ret = tegra_dma_enqueue_req(tspi->rx_dma, &tspi->rx_dma_req);
		if (ret < 0) {
			dev_err(&tspi->pdev->dev, "Error in starting rx dma "
						" error = %d\n", ret);
			if (tspi->cur_direction & DATA_DIR_TX)
				tegra_dma_dequeue_req(tspi->tx_dma,
							&tspi->tx_dma_req);
			return ret;
		}
	}
	tspi->is_curr_dma_xfer = true;
	val = spi_tegra_readl(tspi, SPI_DMA_CTL);
	val |= SPI_DMA;
	spi_tegra_writel(tspi, val, SPI_DMA_CTL);
	return ret;
}

static int spi_tegra_start_cpu_based_transfer(
		struct spi_tegra_data *tspi, struct spi_transfer *t)
{
	unsigned long val = 0;
	unsigned curr_words;

	INIT_COMPLETION(tspi->spi_complete);
	val = spi_tegra_readl(tspi, SPI_DMA_CTL);
	val &= ~(SPI_IE_TX | SPI_IE_RX);
	if (tspi->cur_direction & DATA_DIR_TX)
		val |= SPI_IE_TX;
	if (tspi->cur_direction & DATA_DIR_RX)
		val |= SPI_IE_RX;
	spi_tegra_writel(tspi, val, SPI_DMA_CTL);
	if (tspi->cur_direction & DATA_DIR_TX)
		curr_words = spi_tegra_fill_tx_fifo_from_client_txbuf(tspi, t);
	else
		curr_words = tspi->curr_dma_words;

	val = SPI_DMA_BLK_SET(curr_words - 1);
	spi_tegra_writel(tspi, val, SPI_DMA_BLK);

	tspi->is_curr_dma_xfer = false;
	val = spi_tegra_readl(tspi, SPI_COMMAND1);
	val |= SPI_PIO;
	spi_tegra_writel(tspi, val, SPI_COMMAND1);
	return 0;
}
/* Change the CS with SW */
void spi_tegra_cs_change(struct spi_device *spi, bool enable)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	struct tegra_spi_device_controller_data *cdata = spi->controller_data;
	unsigned long val = 0;

	val = spi_tegra_readl(tspi, SPI_COMMAND1);
	if (cdata && !cdata->is_hw_based_cs) {
		if (enable)
			(spi->mode & SPI_CS_HIGH) ?
				(val |= SPI_CS_SS_VAL) :
				(val &= ~SPI_CS_SS_VAL);
		else
			(spi->mode & SPI_CS_HIGH) ?
				(val &= ~SPI_CS_SS_VAL) :
				(val |= SPI_CS_SS_VAL);
	}
	spi_tegra_writel(tspi, val, SPI_COMMAND1);
}

/* Start a new transfer with clean line settings */
static int spi_tegra_line_settings(struct spi_device *spi,
			struct spi_transfer *t)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	struct tegra_spi_device_controller_data *cdata = spi->controller_data;
	unsigned long val = 0;
	u32 err = 0;

	/* Set Mode */
	val = SPI_MODE_SEL(spi->mode & (SPI_CPHA|SPI_CPOL));
	/* Select chip select line */
	val |= SPI_CS_SEL(spi->chip_select);
	/* Select cs line polarity "inactive", default being high*/
	if (spi->mode & SPI_CS_HIGH)
		SPI_CS_POL_INACTIVE_SET_LOW(val, spi->chip_select);
	else
		val &= ~SPI_CS_POL_INACTIVE_MASK;
	val |= SPI_M_S;
	if (cdata && !cdata->is_hw_based_cs)
		val |= SPI_CS_SW_HW;
	else
		val &= (~SPI_CS_SW_HW);
	val |= SPI_IDLE_SDA_DRIVE_LOW;
	spi_tegra_writel(tspi, val, SPI_COMMAND1);

	/* Set the timings */
	val = 0;
	if (cdata) {
		SPI_SET_CS_SETUP_TIME(val,
			spi->chip_select,
			cdata->cs_setup_clk_count);
		SPI_SET_CS_HOLD_TIME(val,
			spi->chip_select, cdata->cs_hold_clk_count);
		spi_tegra_writel(tspi, val, SPI_CS_TIM1);
	}
	val = 0;
	SPI_SET_CS_ACTIVE_BETWEEN_PACKETS(val,
			spi->chip_select, !t->cs_change);
	SPI_SET_CYCLES_BETWEEN_PACKETS(val, spi->chip_select,
	 tspi->spi_dev_setting[spi->chip_select].cycles_between_packets);
	spi_tegra_writel(tspi, val, SPI_CS_TIM2);

	return err;
}

static int spi_tegra_data_settings(struct spi_device *spi,
			struct spi_transfer *t)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	unsigned long val;
	u32 bits_per_word;

	if (t->bits_per_word || spi->bits_per_word)
		bits_per_word = t->bits_per_word ? t->bits_per_word :
					spi->bits_per_word;
	else if (tspi->spi_dev_setting[spi->chip_select].has_setup_device)
		bits_per_word =
			tspi->spi_dev_setting[spi->chip_select].bits_per_word;

	if (!bits_per_word)
		bits_per_word = 8;
	val = spi_tegra_readl(tspi, SPI_COMMAND1);
	val &= ~SPI_BIT_LENGTH(~0);
	val |= SPI_BIT_LENGTH(bits_per_word - 1);
	if (spi->mode & SPI_LSB_FIRST)
		val |= SPI_LSBIT_FE;
	val |= SPI_LSBYTE_FE;
	if (tspi->is_packed)
		val |= SPI_PACKED;
	val &= ~(SPI_RX_EN | SPI_TX_EN);
	if (t->rx_buf)
		val |= SPI_RX_EN;
	if (t->tx_buf)
		val |= SPI_TX_EN;
	spi_tegra_writel(tspi, val, SPI_COMMAND1);
	return 0;
}

static void spi_tegra_start_transfer(struct spi_device *spi,
			struct spi_transfer *t, bool is_first_of_msg,
			bool is_single_xfer)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	unsigned total_fifo_words;
	u32 speed;
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&tspi->lock, flags);
	if (!t->speed_hz ||
		tspi->spi_dev_setting[spi->chip_select].has_setup_device)
		speed = tspi->spi_dev_setting[spi->chip_select].max_freq;
	else
		speed = t->speed_hz;

	if (speed != tspi->cur_speed) {
		clk_set_rate(tspi->clk, speed);
		tspi->cur_speed = speed;
	}

	if (is_first_of_msg || tspi->is_reset) {
		if (!tspi->is_clkon_always) {
			if (!tspi->clk_state) {
				spin_unlock_irqrestore(&tspi->lock, flags);
				clk_enable(tspi->clk);
				udelay(10);
				spin_lock_irqsave(&tspi->lock, flags);
				tspi->clk_state = 1;
			}
		}
		spi_tegra_line_settings(spi, t);
		tspi->is_reset = false;
	}
	spi_tegra_cs_change(spi, true);
	spi_tegra_clear_status(tspi);
	tspi->cur_direction = 0;
	if (t->rx_buf)
		tspi->cur_direction = DATA_DIR_RX;
	if (t->tx_buf)
		tspi->cur_direction |= DATA_DIR_TX;
	tspi->cur = t;
	tspi->cur_spi = spi;
	tspi->cur_pos = 0;
	tspi->cur_rx_pos = 0;
	tspi->cur_tx_pos = 0;
	tspi->fifo_status = 0;
	total_fifo_words = spi_tegra_calculate_curr_xfer_param(spi, tspi, t);
	spi_tegra_data_settings(spi, t);

	if (total_fifo_words > SPI_FIFO_DEPTH && tspi->is_dma_allowed)
		ret = spi_tegra_start_dma_based_transfer(tspi, t);
	else
		ret = spi_tegra_start_cpu_based_transfer(tspi, t);
	spin_unlock_irqrestore(&tspi->lock, flags);
	WARN_ON(ret < 0);
}

static void spi_tegra_start_message(struct spi_device *spi,
					struct spi_message *m)
{
	struct spi_transfer *t;
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	unsigned long flags;
	int single_xfer = 0;

	single_xfer = list_is_singular(&m->transfers);
	m->actual_length = 0;
	m->status = 0;

	spin_lock_irqsave(&tspi->lock, flags);
	t = list_first_entry(&m->transfers,
			struct spi_transfer, transfer_list);
	tspi->msg_in_transfer = 1;
	spin_unlock_irqrestore(&tspi->lock, flags);
	spi_tegra_start_transfer(spi, t, true, single_xfer);
}

static int spi_tegra_setup(struct spi_device *spi)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	unsigned long flags;

	dev_dbg(&spi->dev, "setup %d bpw, %scpol, %scpha, %dHz\n",
		spi->bits_per_word,
		spi->mode & SPI_CPOL ? "" : "~",
		spi->mode & SPI_CPHA ? "" : "~",
		spi->max_speed_hz);
	BUG_ON(spi->chip_select >= MAX_CHIP_SELECT);
	if (spi->bits_per_word > 32 || spi->bits_per_word < 0)
		return -EINVAL;

	spin_lock_irqsave(&tspi->lock, flags);
	tspi->spi_dev_setting[spi->chip_select].bits_per_word =
			spi->bits_per_word;
	tspi->spi_dev_setting[spi->chip_select].mode = spi->mode;
	tspi->spi_dev_setting[spi->chip_select].max_freq = spi->max_speed_hz;
	tspi->spi_dev_setting[spi->chip_select].has_setup_device = true;
	spin_unlock_irqrestore(&tspi->lock, flags);

	return 0;
}


static int spi_tegra_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_tegra_data *tspi = spi_master_get_devdata(spi->master);
	struct spi_transfer *t;
	unsigned long flags;
	int was_empty;
	int bytes_per_word;

	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;

	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (t->bits_per_word < 0 || t->bits_per_word > 32)
			return -EINVAL;

		if (t->len == 0)
			return -EINVAL;

		/* Check that the all words are available */
		if (t->bits_per_word)
			bytes_per_word = (t->bits_per_word + 7)/8;
		else
			bytes_per_word = (spi->bits_per_word + 7)/8;

		if (t->len % bytes_per_word != 0)
			return -EINVAL;

		if (!t->rx_buf && !t->tx_buf)
			return -EINVAL;
	}

	spin_lock_irqsave(&tspi->lock, flags);
	if (WARN_ON(tspi->is_suspended)) {
		spin_unlock_irqrestore(&tspi->lock, flags);
		return -EBUSY;
	}

	m->state = spi;
	was_empty = list_empty(&tspi->queue);
	list_add_tail(&m->queue, &tspi->queue);
	if (was_empty)
		queue_work(tspi->spi_workqueue, &tspi->spi_transfer_work);
	spin_unlock_irqrestore(&tspi->lock, flags);

	return 0;
}

static void spi_tegra_curr_transfer_complete(struct spi_tegra_data *tspi,
	unsigned err, unsigned cur_xfer_size)
{
	struct spi_message *m;
	struct spi_device *spi;
	struct tegra_spi_device_controller_data *cdata;
	unsigned long delay_cycles;
	unsigned long flags;

	spin_lock_irqsave(&tspi->lock, flags);
	/* Check if CS need to be toggle here */
	if (tspi->cur && tspi->cur->cs_change &&
				tspi->cur->delay_usecs) {
		delay_cycles =
			(tspi->cur_speed * tspi->cur->delay_usecs) << 20;
		if (delay_cycles > MAX_HOLD_CYCLES)
			udelay(tspi->cur->delay_usecs);
	}

	m = list_first_entry(&tspi->queue, struct spi_message, queue);
	if (err)
		m->status = -EIO;
	spi = m->state;
	cdata = spi->controller_data;
	spi_tegra_cs_change(spi, false);
	m->actual_length += cur_xfer_size;
	if (!list_is_last(&tspi->cur->transfer_list, &m->transfers)) {
		tspi->cur = list_first_entry(&tspi->cur->transfer_list,
			struct spi_transfer, transfer_list);
		spin_unlock_irqrestore(&tspi->lock, flags);
		spi_tegra_start_transfer(spi, tspi->cur, false, 0);
	} else {
		list_del(&m->queue);
		m->complete(m->context);
		tspi->msg_in_transfer = 0;
		if (!list_empty(&tspi->queue)) {
			queue_work(tspi->spi_workqueue,
				&tspi->spi_transfer_work);
		} else {
			if (!tspi->is_clkon_always) {
				if (tspi->clk_state) {
					/* Provide delay to stablize the signal
					state */
					usleep_range(0, 10);
					clk_disable(tspi->clk);
					tspi->clk_state = 0;
				}
			}
		}
		spin_unlock_irqrestore(&tspi->lock, flags);
	}
}

static void tegra_spi_tx_dma_complete(struct tegra_dma_req *req)
{
	struct spi_tegra_data *tspi = req->dev;
	complete(&tspi->tx_dma_complete);
}

static void tegra_spi_rx_dma_complete(struct tegra_dma_req *req)
{
	struct spi_tegra_data *tspi = req->dev;
	complete(&tspi->rx_dma_complete);
}

static void handle_cpu_based_xfer(void *context_data, int error)
{
	struct spi_tegra_data *tspi = context_data;
	struct spi_transfer *t = tspi->cur;
	unsigned long flags;

	if ((tspi->fifo_status & SPI_ERR) ||
		!(tspi->trans_status & SPI_RDY) ||
		error) {
		dev_err(&tspi->pdev->dev,
			"ERROR bit set fifo_status 0x%x "
			"trans_status 0x%x\n",
			tspi->fifo_status, tspi->trans_status);
		tegra_periph_reset_assert(tspi->clk);
		udelay(2);
		tegra_periph_reset_deassert(tspi->clk);
		WARN_ON(1);
		tspi->is_reset = true;
		spi_tegra_curr_transfer_complete(tspi,
				(SPI_ERR & tspi->fifo_status), t->len);
		goto exit;
	}

	spin_lock_irqsave(&tspi->lock, flags);
	if (tspi->cur_direction & DATA_DIR_RX)
		spi_tegra_read_rx_fifo_to_client_rxbuf(tspi, t);

	if (tspi->cur_direction & DATA_DIR_TX)
		tspi->cur_pos = tspi->cur_tx_pos;
	else if (tspi->cur_direction & DATA_DIR_RX)
		tspi->cur_pos = tspi->cur_rx_pos;
	else
		WARN_ON(1);
	spin_unlock_irqrestore(&tspi->lock, flags);

	if (tspi->cur_pos == t->len) {
		spi_tegra_curr_transfer_complete(tspi,
			(SPI_ERR & tspi->fifo_status), t->len);
		goto exit;
	}

	spin_lock_irqsave(&tspi->lock, flags);
	spi_tegra_calculate_curr_xfer_param(tspi->cur_spi, tspi, t);
	spi_tegra_start_cpu_based_transfer(tspi, t);
	spin_unlock_irqrestore(&tspi->lock, flags);
exit:
	return;
}

static void spi_tegra_work(struct work_struct *work)
{
	struct spi_tegra_data *tspi = container_of(work,
			struct spi_tegra_data, spi_transfer_work);
	unsigned long flags;
	struct spi_message *m;
	struct spi_device *spi;
	int err = 0;
	unsigned total_fifo_words;

	if (list_empty(&tspi->queue))
		return;

	/* Start transfer */
	spin_lock_irqsave(&tspi->lock, flags);
	m = list_first_entry(&tspi->queue, struct spi_message,
		queue);
	spi = m->state;
	spin_unlock_irqrestore(&tspi->lock, flags);
	spi_tegra_start_message(spi, m);
	while (1) {
		struct spi_transfer *t;
		bool msg_in_transfer;
		long wait_status;
		int err_val = 0;

		wait_status = 0;
		spin_lock_irqsave(&tspi->lock, flags);
		t = tspi->cur;
		msg_in_transfer = tspi->msg_in_transfer;
		spin_unlock_irqrestore(&tspi->lock, flags);
		if (!msg_in_transfer)
			break;
		/* Wait for completion interrupt */
		wait_status = wait_for_completion_interruptible_timeout(
			&tspi->spi_complete,
			(msecs_to_jiffies(1000)));
		if (wait_status <= 0) {
			dev_err(&tspi->pdev->dev, "Error in SPI"
			" completion %ld\r\n", wait_status);
			spi_tegra_reg_print(tspi);
			err_val++;
		}

		/* Handle completion */
		if (!tspi->is_curr_dma_xfer) {
			handle_cpu_based_xfer(tspi, err_val);
			continue;
		}

		/* Abort dmas if any error */
		if (tspi->cur_direction & DATA_DIR_TX) {
			if (tspi->fifo_status &
				(SPI_TX_FIFO_UNF | SPI_TX_FIFO_OVF)) {
				tegra_dma_dequeue_req(tspi->tx_dma,
					&tspi->tx_dma_req);
				err += 1;
			} else {
				wait_status =
				wait_for_completion_interruptible_timeout(
						&tspi->tx_dma_complete,
						SPI_DMA_TIMEOUT);
				if (wait_status <= 0) {
					tegra_dma_dequeue_req(tspi->tx_dma,
						&tspi->tx_dma_req);
					dev_err(&tspi->pdev->dev,
					 "Error in Dma Tx ws %ld \r\n",
					 wait_status);
					spi_tegra_reg_print(tspi);
					err += 1;
				}
			}
		}

		if (tspi->cur_direction & DATA_DIR_RX) {
			if (tspi->fifo_status &
				(SPI_RX_FIFO_UNF | SPI_RX_FIFO_OVF)) {
				tegra_dma_dequeue_req(tspi->rx_dma,
					&tspi->rx_dma_req);
				err += 2;
			} else {
				wait_status =
				wait_for_completion_interruptible_timeout(
						&tspi->rx_dma_complete,
						SPI_DMA_TIMEOUT);
				if (wait_status <= 0) {
					tegra_dma_dequeue_req(tspi->rx_dma,
						&tspi->rx_dma_req);
					dev_err(&tspi->pdev->dev,
						"Error in Dma Rx transfer %ld\n",
						wait_status);
					spi_tegra_reg_print(tspi);
					err += 2;
				}
			}
		}
		/* Handle errors */
		spin_lock_irqsave(&tspi->lock, flags);
		if (err) {
			dev_err(&tspi->pdev->dev, "%s ERROR bit set trans_status 0x%x, fifo_status 0x%x\n",
					__func__, tspi->trans_status,
					tspi->fifo_status);
			tegra_periph_reset_assert(tspi->clk);
			udelay(2);
			tegra_periph_reset_deassert(tspi->clk);
			WARN_ON(1);
			spi_tegra_curr_transfer_complete(
				tspi, err, t->len);
			tspi->is_reset = true;
			spin_unlock_irqrestore(&tspi->lock, flags);
			continue;
		}
		/* Finish packet transfer */
		if (tspi->cur_direction & DATA_DIR_RX)
			spi_tegra_copy_spi_rxbuf_to_client_rxbuf(
				tspi, t);

		if (tspi->cur_direction & DATA_DIR_TX)
			tspi->cur_pos = tspi->cur_tx_pos;
		else if (tspi->cur_direction & DATA_DIR_RX)
			tspi->cur_pos = tspi->cur_rx_pos;
		else
			WARN_ON(1);
		if (tspi->cur_pos == t->len) {
			spin_unlock_irqrestore(&tspi->lock, flags);
			spi_tegra_curr_transfer_complete(tspi,
				SPI_ERR & tspi->fifo_status,
				t->len);
			continue;
		}

		/* Continue transfer in current message */
		total_fifo_words = spi_tegra_calculate_curr_xfer_param(
			tspi->cur_spi, tspi, t);
		if (total_fifo_words > SPI_FIFO_DEPTH)
			err =
			 spi_tegra_start_dma_based_transfer(tspi, t);
		else
			err =
			 spi_tegra_start_cpu_based_transfer(tspi, t);

		spin_unlock_irqrestore(&tspi->lock, flags);
		WARN_ON(err < 0);
	}
}

static irqreturn_t spi_tegra_isr(int irq, void *context_data)
{
	struct spi_tegra_data *tspi = context_data;
	unsigned long val = 0;
	unsigned long clear_err = 0;
	unsigned long flags;

	spin_lock_irqsave(&tspi->lock, flags);
	val = spi_tegra_readl(tspi, SPI_TRANS_STATUS);
	tspi->trans_status = val;
	/* Clear interrupt and handle everything in thread */
	if (val & SPI_RDY)
		spi_tegra_writel(tspi, val, SPI_TRANS_STATUS);
	/* Save transfer fifo status and clear error bits */
	val = spi_tegra_readl(tspi, SPI_FIFO_STATUS);
	tspi->fifo_status = val;
	spin_unlock_irqrestore(&tspi->lock, flags);
	if (val & SPI_ERR) {
		clear_err |= SPI_ERR;
		if (val & SPI_TX_FIFO_UNF)
			clear_err |= SPI_TX_FIFO_UNF;
		if (val & SPI_TX_FIFO_OVF)
			clear_err |= SPI_TX_FIFO_OVF;
		if (val & SPI_RX_FIFO_UNF)
			clear_err |= SPI_RX_FIFO_UNF;
		if (val & SPI_RX_FIFO_OVF)
			clear_err |= SPI_RX_FIFO_OVF;
		spi_tegra_writel(tspi, clear_err, SPI_FIFO_STATUS);
	}
	complete(&tspi->spi_complete);
	return IRQ_HANDLED;
}

static int __init spi_tegra_probe(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	struct resource		*r;
	struct tegra_spi_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	char spi_wq_name[20];

	master = spi_alloc_master(&pdev->dev, sizeof *tspi);
	if (master == NULL) {
		dev_err(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;

	if (pdev->id != -1)
		master->bus_num = pdev->id;

	master->setup = spi_tegra_setup;
	master->transfer = spi_tegra_transfer;
	master->num_chipselect = MAX_CHIP_SELECT;

	dev_set_drvdata(&pdev->dev, master);
	tspi = spi_master_get_devdata(master);
	tspi->master = master;
	tspi->pdev = pdev;
	spin_lock_init(&tspi->lock);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		ret = -ENODEV;
		goto fail_no_mem;
	}
	memset(tspi->spi_dev_setting,
		0, MAX_CHIP_SELECT * sizeof(struct spi_device_setting));

	if (!request_mem_region(r->start, (r->end - r->start) + 1,
				dev_name(&pdev->dev))) {
		ret = -EBUSY;
		goto fail_no_mem;
	}

	tspi->phys = r->start;
	tspi->base = ioremap(r->start, r->end - r->start + 1);
	if (!tspi->base) {
		dev_err(&pdev->dev, "can't ioremap iomem\n");
		ret = -ENOMEM;
		goto fail_io_map;
	}

	tspi->irq = platform_get_irq(pdev, 0);
	if (unlikely(tspi->irq < 0)) {
		dev_err(&pdev->dev, "can't find irq resource\n");
		ret = -ENXIO;
		goto fail_irq_req;
	}

	sprintf(tspi->port_name, "tegra_spi_%d", pdev->id);
	ret = request_irq(tspi->irq, spi_tegra_isr,
			IRQF_DISABLED, tspi->port_name, tspi);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register ISR for IRQ %d\n",
					tspi->irq);
		goto fail_irq_req;
	}

	tspi->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(tspi->clk)) {
		dev_err(&pdev->dev, "can not get clock\n");
		ret = PTR_ERR(tspi->clk);
		goto fail_clk_get;
	}
	tspi->cur_speed = 0;

	INIT_LIST_HEAD(&tspi->queue);

	if (pdata) {
		tspi->is_clkon_always = pdata->is_clkon_always;
		tspi->is_dma_allowed = pdata->is_dma_based;
		tspi->dma_buf_size = (pdata->max_dma_buffer) ?
			pdata->max_dma_buffer : DEFAULT_SPI_DMA_BUF_LEN;
	} else {
		tspi->is_clkon_always = false;
		tspi->is_dma_allowed = true;
		tspi->dma_buf_size = DEFAULT_SPI_DMA_BUF_LEN;
	}

	init_completion(&tspi->spi_complete);
	if (!tspi->is_dma_allowed)
		goto skip_dma_alloc;

	init_completion(&tspi->tx_dma_complete);
	init_completion(&tspi->rx_dma_complete);

	tspi->rx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT,
				"spi_rx_%d", pdev->id);
	if (!tspi->rx_dma) {
		dev_err(&pdev->dev, "can not allocate rx dma channel\n");
		ret = -ENODEV;
		goto fail_rx_dma_alloc;
	}

	tspi->rx_buf = dma_alloc_coherent(&pdev->dev, tspi->dma_buf_size,
					 &tspi->rx_buf_phys, GFP_KERNEL);
	if (!tspi->rx_buf) {
		dev_err(&pdev->dev, "can not allocate rx bounce buffer\n");
		ret = -ENOMEM;
		goto fail_rx_buf_alloc;
	}

	memset(&tspi->rx_dma_req, 0, sizeof(struct tegra_dma_req));
	tspi->rx_dma_req.complete = tegra_spi_rx_dma_complete;
	tspi->rx_dma_req.to_memory = 1;
	tspi->rx_dma_req.dest_addr = tspi->rx_buf_phys;
	tspi->rx_dma_req.virt_addr = tspi->rx_buf;
	tspi->rx_dma_req.dest_bus_width = 32;
	tspi->rx_dma_req.source_addr = tspi->phys + SPI_RX_FIFO;
	tspi->rx_dma_req.source_bus_width = 32;
	tspi->rx_dma_req.source_wrap = 4;
	tspi->rx_dma_req.dest_wrap = 0;
	tspi->rx_dma_req.req_sel = spi_tegra_req_sels[pdev->id];
	tspi->rx_dma_req.dev = tspi;
	tspi->tx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT,
				"spi_tx_%d", pdev->id);
	if (!tspi->tx_dma) {
		dev_err(&pdev->dev, "can not allocate tx dma channel\n");
		ret = -ENODEV;
		goto fail_tx_dma_alloc;
	}

	tspi->tx_buf = dma_alloc_coherent(&pdev->dev, tspi->dma_buf_size,
					 &tspi->tx_buf_phys, GFP_KERNEL);
	if (!tspi->tx_buf) {
		dev_err(&pdev->dev, "can not allocate tx bounce buffer\n");
		ret = -ENOMEM;
		goto fail_tx_buf_alloc;
	}

	memset(&tspi->tx_dma_req, 0, sizeof(struct tegra_dma_req));
	tspi->tx_dma_req.complete = tegra_spi_tx_dma_complete;
	tspi->tx_dma_req.to_memory = 0;
	tspi->tx_dma_req.dest_addr = tspi->phys + SPI_TX_FIFO;
	tspi->tx_dma_req.virt_addr = tspi->tx_buf;
	tspi->tx_dma_req.dest_bus_width = 32;
	tspi->tx_dma_req.dest_wrap = 4;
	tspi->tx_dma_req.source_wrap = 0;
	tspi->tx_dma_req.source_addr = tspi->tx_buf_phys;
	tspi->tx_dma_req.source_bus_width = 32;
	tspi->tx_dma_req.req_sel = spi_tegra_req_sels[pdev->id];
	tspi->tx_dma_req.dev = tspi;

skip_dma_alloc:
	clk_enable(tspi->clk);
	tspi->clk_state = 1;
	ret = spi_register_master(master);
	if (!tspi->is_clkon_always) {
		if (tspi->clk_state) {
			clk_disable(tspi->clk);
			tspi->clk_state = 0;
		}
	}

	if (ret < 0) {
		dev_err(&pdev->dev, "register to master failed err %d\n", ret);
		goto fail_master_register;
	}

	/* create the workqueue */
	snprintf(spi_wq_name, sizeof(spi_wq_name), "spi_t3-%d", pdev->id);
	tspi->spi_workqueue = create_singlethread_workqueue(spi_wq_name);
	if (!tspi->spi_workqueue) {
		dev_err(&pdev->dev, "Failed to create work queue\n");
		ret = -ENODEV;
		goto fail_workqueue;
	}
	INIT_WORK(&tspi->spi_transfer_work, spi_tegra_work);
	return ret;

fail_workqueue:
	spi_unregister_master(master);
fail_master_register:
	if (tspi->tx_buf)
		dma_free_coherent(&pdev->dev, tspi->dma_buf_size,
				tspi->tx_buf, tspi->tx_buf_phys);
fail_tx_buf_alloc:
	if (tspi->tx_dma)
		tegra_dma_free_channel(tspi->tx_dma);
fail_tx_dma_alloc:
	if (tspi->rx_buf)
		dma_free_coherent(&pdev->dev, tspi->dma_buf_size,
			tspi->rx_buf, tspi->rx_buf_phys);
fail_rx_buf_alloc:
	if (tspi->rx_dma)
		tegra_dma_free_channel(tspi->rx_dma);
fail_rx_dma_alloc:
	clk_put(tspi->clk);
fail_clk_get:
	free_irq(tspi->irq, tspi);
fail_irq_req:
	iounmap(tspi->base);
fail_io_map:
	release_mem_region(r->start, (r->end - r->start) + 1);
fail_no_mem:
	spi_master_put(master);
	return ret;
}

static int spi_tegra_remove(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	struct resource		*r;

	master = dev_get_drvdata(&pdev->dev);
	tspi = spi_master_get_devdata(master);

	if (tspi->tx_buf)
		dma_free_coherent(&pdev->dev, tspi->dma_buf_size,
				tspi->tx_buf, tspi->tx_buf_phys);
	if (tspi->tx_dma)
		tegra_dma_free_channel(tspi->tx_dma);
	if (tspi->rx_buf)
		dma_free_coherent(&pdev->dev, tspi->dma_buf_size,
			tspi->rx_buf, tspi->rx_buf_phys);
	if (tspi->rx_dma)
		tegra_dma_free_channel(tspi->rx_dma);

	if (tspi->is_clkon_always) {
		clk_disable(tspi->clk);
		tspi->clk_state = 0;
	}

	clk_put(tspi->clk);
	iounmap(tspi->base);

	spi_master_put(master);
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, (r->end - r->start) + 1);

	return 0;
}

#ifdef CONFIG_PM
static int spi_tegra_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	unsigned long		flags;
	unsigned		limit = 50;

	master = dev_get_drvdata(&pdev->dev);
	tspi = spi_master_get_devdata(master);
	spin_lock_irqsave(&tspi->lock, flags);
	tspi->is_suspended = true;

	WARN_ON(!list_empty(&tspi->queue));

	while (!list_empty(&tspi->queue) && limit--) {
		spin_unlock_irqrestore(&tspi->lock, flags);
		mdelay(20);
		spin_lock_irqsave(&tspi->lock, flags);
	}

	spin_unlock_irqrestore(&tspi->lock, flags);
	if (tspi->is_clkon_always) {
		clk_disable(tspi->clk);
		tspi->clk_state = 0;
	}
	return 0;
}

static int spi_tegra_resume(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct spi_tegra_data	*tspi;
	unsigned long		flags;

	master = dev_get_drvdata(&pdev->dev);
	tspi = spi_master_get_devdata(master);

	spin_lock_irqsave(&tspi->lock, flags);
	clk_enable(tspi->clk);
	tspi->clk_state = 1;
	if (!tspi->is_clkon_always) {
		clk_disable(tspi->clk);
		tspi->clk_state = 0;
	}

	tspi->cur_speed = 0;
	tspi->is_suspended = false;
	spin_unlock_irqrestore(&tspi->lock, flags);
	return 0;
}
#endif

MODULE_ALIAS("platform:spi_tegra");

static struct platform_driver spi_tegra_driver = {
	.driver = {
		.name =	"tegra11-spi",
		.owner =	THIS_MODULE,
	},
	.remove =	spi_tegra_remove,
#ifdef CONFIG_PM
	.suspend =	spi_tegra_suspend,
	.resume =	spi_tegra_resume,
#endif
};

static int __init spi_tegra_init(void)
{
	return platform_driver_probe(&spi_tegra_driver, spi_tegra_probe);
}
subsys_initcall(spi_tegra_init);

static void __exit spi_tegra_exit(void)
{
	platform_driver_unregister(&spi_tegra_driver);
}
module_exit(spi_tegra_exit);

MODULE_LICENSE("GPL");
