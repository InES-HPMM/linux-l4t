/*
 * MTD SPI driver for qspi flash chips
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 * Copyright (c) 2005, Intec Automation Inc.
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>

#include <linux/mtd/cfi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/qspi_mtd.h>

#define COMMAND_WIDTH 1
#define ADDRESS_WIDTH 4
#define WE_RETRY_COUNT 20
#define WIP_RETRY_COUNT 2000
#define QUAD_ENABLE_WAIT_TIME 1000
#define WRITE_ENABLE_WAIT_TIME 100
#define WIP_ENABLE_WAIT_TIME 10000
#define BITS8_PER_WORD 8
#define BITS16_PER_WORD 16
#define BITS32_PER_WORD 32

static inline struct qspi *mtd_to_qspi(struct mtd_info *mtd)
{
	return container_of(mtd, struct qspi, mtd);
}

/*
 * Set Mode for transfer request
 * Function sets Bus width, DDR/SDR and opcode
 */

static void set_mode(struct spi_transfer *tfr, uint8_t is_ddr,
		uint8_t bus_width, uint8_t op_code)
{
	tfr->delay_usecs = set_op_mode(op_code) | set_bus_width(bus_width);
	if (is_ddr)
		tfr->delay_usecs |= set_sdr_ddr;
}

/*
 * Copy Paramters from default command table
 * Command table contains command, address and data
 * related information associated with opcode
 */

static void copy_cmd_default(struct qcmdset *qcmd, struct qcmdset *cmd_table)
{
	qcmd->qcmd.op_code = cmd_table->qcmd.op_code;
	qcmd->qcmd.is_ddr = cmd_table->qcmd.is_ddr;
	qcmd->qcmd.bus_width = cmd_table->qcmd.bus_width;
	qcmd->qcmd.post_txn = cmd_table->qcmd.post_txn;
	qcmd->qaddr.address = cmd_table->qaddr.address;
	qcmd->qaddr.is_ddr = cmd_table->qaddr.is_ddr;
	qcmd->qaddr.len = cmd_table->qaddr.len;
	qcmd->qaddr.bus_width = cmd_table->qaddr.bus_width;
	qcmd->qaddr.dummy_cycles = cmd_table->qaddr.dummy_cycles;
	qcmd->qdata.is_ddr = cmd_table->qdata.is_ddr;
	qcmd->qdata.bus_width = cmd_table->qdata.bus_width;
}

/*
 * Copy Paramters from default command table
 * Command table contains command, address and data
 * related information associated with opcode
 */

static int read_reg(struct qspi *flash, uint8_t code, uint8_t *regval)
{
	uint8_t tx_buf[1], rx_buf[1];
	int status = PASS, err;
	struct spi_transfer t[2];
	struct spi_message m;
	spi_message_init(&m);

	memset(t, 0, sizeof(t));
	tx_buf[0] = code;
	t[0].len = COMMAND_WIDTH;
	t[0].tx_buf = tx_buf;
	t[0].bits_per_word = BITS8_PER_WORD;
	set_mode(&t[0], FALSE, X1, STATUS_READ);

	spi_message_add_tail(&t[0], &m);
	t[1].len = COMMAND_WIDTH;
	t[1].rx_buf = rx_buf;
	t[1].bits_per_word = BITS8_PER_WORD;
	set_mode(&t[1], FALSE, X1, STATUS_READ);

	spi_message_add_tail(&t[1], &m);

	err = spi_sync(flash->spi, &m);
	if (err < 0) {
		pr_err("error: write enable failed %d", status);
		status = FAIL;
	}

	*regval = rx_buf[0];
	return status;
}

/*
 * Enable/ Disable QUAD Bit in Configuration Register
 * Set QUAD bit to 1 for QUAD Read/Write operations
 */

static int qspi_quad_flag_set(struct qspi *flash, uint8_t is_set)
{
	uint8_t tx_buf[5], regval;
	int status = PASS, err, tried = 0, comp = QUAD_ENABLE;
	struct spi_transfer t[2];
	struct spi_message m;
	spi_message_init(&m);

	do {
		if (tried++ == WE_RETRY_COUNT) {
			pr_err(KERN_ERR "tried max times not changing quad bit\n");
			return FAIL;
		}

		memset(t, 0, sizeof(t));
		read_reg(flash, RDSR1, &regval);
		tx_buf[0] = OPCODE_WRR;
		tx_buf[1] = regval;
		if (is_set) {
			tx_buf[2] = QUAD_ENABLE;
			comp = QUAD_ENABLE;
		} else {
			tx_buf[2] = QUAD_DISABLE;
			comp = QUAD_DISABLE;
		}

		t[0].len = 3;
		t[0].tx_buf = tx_buf;
		t[0].bits_per_word = BITS8_PER_WORD;

		set_mode(&t[0], FALSE, X1, STATUS_READ);
		spi_message_add_tail(&t[0], &m);
		err = spi_sync(flash->spi, &m);
		if (err < 0) {
			pr_err("error: write enable failed %d", status);
			status = FAIL;
		}
		read_reg(flash, RDCR, &regval);
		udelay(QUAD_ENABLE_WAIT_TIME);
	} while ((regval & QUAD_ENABLE) != comp);

	return status;
}

/*
 * Enable/ Disable Write Enable Bit in Configuration Register
 * Set WEL bit to 1 before Erase and Write Operations
 */

static int qspi_write_en(struct qspi *flash, uint8_t is_enable)
{
	struct spi_transfer t[1];
	uint8_t cmd_buf[5], regval;
	int status = 0, err, tried = 0, comp;
	struct spi_message m;

	spi_message_init(&m);

	do {
		if (tried++ == WE_RETRY_COUNT) {
			pr_err("tired max times not changing WE bit\n");
			return FAIL;
		}
		memset(t, 0, sizeof(t));

		if (is_enable) {
			cmd_buf[0] = OPCODE_WRITE_ENABLE;
			comp = WEL_ENABLE;
		} else {
			cmd_buf[0] = OPCODE_WRITE_DISABLE;
			comp = WEL_DISABLE;
		}

		t[0].len = COMMAND_WIDTH;
		t[0].tx_buf = cmd_buf;
		t[0].bits_per_word = BITS8_PER_WORD;

		set_mode(&t[0], FALSE, X1,
				STATUS_READ);

		spi_message_add_tail(&t[0], &m);
		err = spi_sync(flash->spi, &m);

		if (err < 0) {
			pr_err("error: write enable failed %d", status);
			return 1;
		}

		udelay(WRITE_ENABLE_WAIT_TIME);
		status = read_reg(flash, RDSR1, &regval);

	} while ((regval & WEL_ENABLE) != comp);

	return status;
}

/*
 * Wait till flash is ready for erase/write operations.
 * Returns negative if error occurred.
 */

static int wait_till_ready(struct qspi *flash)
{
	uint8_t regval, status = PASS;
	int tried = 0;

	do {
		if (tried++ == WIP_RETRY_COUNT) {
			pr_err("tired max times not changing WIP bit\n");
			return FAIL;
		}
		if ((tried % 20) == 0)
			pr_info("Waiting in WIP iter: %d\n", tried);

		status = read_reg(flash, RDSR1, &regval);
		udelay(WIP_ENABLE_WAIT_TIME);
	} while ((regval & WIP_ENABLE) == WIP_ENABLE);
	return status;
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */

static int erase_chip(struct qspi *flash)
{
	uint8_t cmd_opcode;
	struct spi_transfer t[1];
	struct spi_message m;

	pr_info("%s: %s %lldKiB\n",
		dev_name(&flash->spi->dev), __func__,
		(long long)(flash->mtd.size >> 10));

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	qspi_write_en(flash, TRUE);

	/* Set up command buffer. */
	cmd_opcode = OPCODE_CHIP_ERASE;

	spi_message_init(&m);
	memset(t, 0, sizeof(t));
	t[0].len = COMMAND_WIDTH;
	t[0].tx_buf = &cmd_opcode;

	set_mode(&t[0], flash->cmd_table.qcmd.is_ddr,
		flash->cmd_table.qcmd.bus_width, cmd_opcode);

	spi_message_add_tail(&t[0], &m);
	spi_sync(flash->spi, &m);

	return 0;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct qspi *flash, u32 offset)
{
	uint8_t cmd_addr_buf[5];
	struct spi_transfer t[1];
	struct spi_message m;

	pr_info("%s: %s %dKiB at 0x%08x\n",
		dev_name(&flash->spi->dev),
		__func__, flash->mtd.erasesize / 1024, offset);

	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		return 1;
	}

	/* Send write enable, then erase commands. */
	qspi_write_en(flash, TRUE);

	/* Set up command buffer. */
	cmd_addr_buf[0] = flash->erase_opcode;
	cmd_addr_buf[1] = (offset >> 24) & 0xFF;
	cmd_addr_buf[2] = (offset >> 16) & 0xFF;
	cmd_addr_buf[3] = (offset >> 8) & 0xFF;
	cmd_addr_buf[4] = offset & 0xFF;

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].len = (COMMAND_WIDTH + ADDRESS_WIDTH);
	t[0].tx_buf = cmd_addr_buf;

	set_mode(&t[0], flash->cmd_table.qcmd.is_ddr,
		flash->cmd_table.qcmd.bus_width, flash->erase_opcode);

	spi_message_add_tail(&t[0], &m);

	spi_sync(flash->spi, &m);

	return 0;
}

/****************************************************************************/

/*
 * MTD Erase, Read and Write implementation
 */

/****************************************************************************/

/*
 * Erase an address range on the flash chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int qspi_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct qspi *flash = mtd_to_qspi(mtd);
	u32 addr, len;
	uint32_t rem;

	pr_info("%s: %s at 0x%llx, len %lld\n",
		dev_name(&flash->spi->dev),
		__func__, (long long)instr->addr,
		(long long)instr->len);

	div_u64_rem(instr->len, mtd->erasesize, &rem);

	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	mutex_lock(&flash->lock);

	/* whole-chip erase? */
	if (len == flash->mtd.size) {
		if (erase_chip(flash)) {
			instr->state = MTD_ERASE_FAILED;
			mutex_unlock(&flash->lock);
			return -EIO;
		}

		/* "sector"-at-a-time erase */
	} else {
		while (len) {
			if (erase_sector(flash, addr)) {
				instr->state = MTD_ERASE_FAILED;
				mutex_unlock(&flash->lock);
				return -EIO;
			}

			addr += mtd->erasesize;
			len -= mtd->erasesize;
		}
	}

	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int qspi_read(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	struct qspi *flash = mtd_to_qspi(mtd);
	struct spi_transfer t[3];
	struct spi_message m;
	uint8_t merge_cmd_addr = FALSE;
	uint8_t cmd_addr_buf[5];
	int err;
	struct tegra_qspi_device_controller_data *cdata
				= flash->spi->controller_data;

	u8 bus_width = X1, num_dummy_cycles = 0;
	bool is_ddr = false;
	u32 speed;

	pr_info("%s: %s from 0x%08x, len %zd\n",
		dev_name(&flash->spi->dev),
		__func__, (u32)from, len);

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	/* Set Controller data Parameters
	 * Set DDR/SDR, X1/X4 and Dummy Cycles from DT
	 */

	if (!cdata) {
		if (len >  cdata->x1_len_limit) {
			is_ddr = cdata->x4_is_ddr;
			bus_width = cdata->x4_bus_speed;
			num_dummy_cycles =  cdata->x4_dymmy_cycle;
			speed = cdata->x4_bus_speed;
			if (is_ddr)
				copy_cmd_default(&flash->cmd_table,
					&cmd_info_table[DDR_QUAD_IO_READ]);
			else
				copy_cmd_default(&flash->cmd_table,
					&cmd_info_table[QUAD_IO_READ]);
		} else {
			is_ddr = false;
			bus_width = cdata->x1_bus_speed;
			num_dummy_cycles =  cdata->x1_dymmy_cycle;
			speed = cdata->x1_bus_speed;
			copy_cmd_default(&flash->cmd_table,
					&cmd_info_table[NORMAL_READ]);
		}
	} else
		copy_cmd_default(&flash->cmd_table,
					&cmd_info_table[DDR_QUAD_IO_READ]);

	/* check if possible to merge cmd and address */
	if ((flash->cmd_table.qcmd.is_ddr ==
		flash->cmd_table.qaddr.is_ddr) &&
		(flash->cmd_table.qcmd.bus_width ==
		flash->cmd_table.qaddr.bus_width) &&
		flash->cmd_table.qcmd.post_txn > 0) {

		merge_cmd_addr = TRUE;
		flash->cmd_table.qcmd.post_txn =
			flash->cmd_table.qcmd.post_txn - 1;
	}

	cmd_addr_buf[0] = flash->cmd_table.qcmd.op_code;
	cmd_addr_buf[1] = (from >> 24) & 0xFF;
	cmd_addr_buf[2] = (from >> 16) & 0xFF;
	cmd_addr_buf[3] = (from >> 8) & 0xFF;
	cmd_addr_buf[4] = from & 0xFF;

	if (merge_cmd_addr) {

		t[0].len = (flash->cmd_table.qaddr.len + 1
			+ (flash->cmd_table.qaddr.dummy_cycles/8));
		t[0].tx_buf = cmd_addr_buf;

		set_mode(&t[0],
			flash->cmd_table.qcmd.is_ddr,
			flash->cmd_table.qcmd.bus_width,
			flash->cmd_table.qcmd.op_code);
		spi_message_add_tail(&t[0], &m);

		t[1].len = len;
		t[1].rx_buf = buf;
		set_mode(&t[1],
			flash->cmd_table.qdata.is_ddr,
			flash->cmd_table.qdata.bus_width,
			flash->cmd_table.qcmd.op_code);

		/* in-activate the cs at the end */
		t[1].cs_change = TRUE;
		spi_message_add_tail(&t[1], &m);

	} else {

		t[0].len = 1;
		t[0].tx_buf = &cmd_addr_buf[0];

		set_mode(&t[0],
			flash->cmd_table.qcmd.is_ddr,
			flash->cmd_table.qcmd.bus_width,
			flash->cmd_table.qcmd.op_code);

		spi_message_add_tail(&t[0], &m);

		t[1].len = ((flash->cmd_table.qaddr.len +
			(flash->cmd_table.qaddr.dummy_cycles/8)));

		t[1].tx_buf = &cmd_addr_buf[1];

		set_mode(&t[1],
			flash->cmd_table.qaddr.is_ddr,
			flash->cmd_table.qaddr.bus_width,
			flash->cmd_table.qcmd.op_code);

		spi_message_add_tail(&t[1], &m);

		t[2].len = len;
		t[2].rx_buf = buf;
		set_mode(&t[2],
			flash->cmd_table.qdata.is_ddr,
			flash->cmd_table.qdata.bus_width,
			flash->cmd_table.qcmd.op_code);

		/* in-activate the cs at the end */
		t[2].cs_change = TRUE;
		spi_message_add_tail(&t[2], &m);
	}

	mutex_lock(&flash->lock);
	if (wait_till_ready(flash)) {
		mutex_unlock(&flash->lock);
		return 1;
	}

	/* Enable QUAD bit before doing QUAD i/o operation */
	if (flash->cmd_table.qdata.bus_width == X4) {
		err = qspi_quad_flag_set(flash, TRUE);
		if (err) {
			mutex_unlock(&flash->lock);
			return 1;

		}
	}

	spi_sync(flash->spi, &m);

	*retlen = m.actual_length -
		(flash->cmd_table.qaddr.len + 1 +
		(flash->cmd_table.qaddr.dummy_cycles/8));

	mutex_unlock(&flash->lock);

	return 0;
}

static int qspi_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct qspi *flash = mtd_to_qspi(mtd);
	u32 page_offset, page_size;
	struct spi_transfer t[2];
	struct spi_message m;
	uint8_t cmd_addr_buf[5];
	uint8_t opcode;
	int err;
	u32 offset = (unsigned long)to;
	struct tegra_qspi_device_controller_data *cdata =
					flash->spi->controller_data;

	u8 bus_width = X1, num_dummy_cycles = 0;
	bool is_ddr = false;
	u32 speed;

	pr_info("%s: %s to 0x%08x, len %zd\n", dev_name(&flash->spi->dev),
			__func__, (u32)to, len);

	/* Set Controller data Parameters
	 * Set DDR/SDR, X1/X4 and Dummy Cycles from DT
	 */

	if (!cdata) {
		if (len >  cdata->x1_len_limit) {
			is_ddr = cdata->x4_is_ddr;
			bus_width = cdata->x4_bus_speed;
			num_dummy_cycles =  cdata->x4_dymmy_cycle;
			speed = cdata->x4_bus_speed;
			copy_cmd_default(&flash->cmd_table,
				&cmd_info_table[QUAD_PAGE_PROGRAM]);
		} else {
			is_ddr = false;
			bus_width = cdata->x1_bus_speed;
			num_dummy_cycles =  cdata->x1_dymmy_cycle;
			speed = cdata->x1_bus_speed;
			copy_cmd_default(&flash->cmd_table,
					&cmd_info_table[PAGE_PROGRAM]);
		}
	} else
		copy_cmd_default(&flash->cmd_table,
				&cmd_info_table[QUAD_PAGE_PROGRAM]);

	cmd_addr_buf[0] = opcode = flash->cmd_table.qcmd.op_code;
	cmd_addr_buf[1] = (offset >> 24) & 0xFF;
	cmd_addr_buf[2] = (offset >> 16) & 0xFF;
	cmd_addr_buf[3] = (offset >> 8) & 0xFF;
	cmd_addr_buf[4] = offset & 0xFF;

	page_offset = offset & (flash->page_size - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= flash->page_size) {

		spi_message_init(&m);
		memset(t, 0, sizeof(t));
		t[0].tx_buf = cmd_addr_buf;
		t[0].len = flash->cmd_table.qaddr.len + 1;
		t[0].bits_per_word = BITS8_PER_WORD;
		set_mode(&t[0], flash->cmd_table.qcmd.is_ddr,
			flash->cmd_table.qcmd.bus_width, opcode);

		spi_message_add_tail(&t[0], &m);

		t[1].tx_buf = buf;

		t[1].len = len;
		set_mode(&t[1], flash->cmd_table.qcmd.is_ddr,
			flash->cmd_table.qdata.bus_width, opcode);

		t[1].cs_change = TRUE;
		spi_message_add_tail(&t[1], &m);
		/* Wait until finished previous write command. */

		mutex_lock(&flash->lock);
		if (wait_till_ready(flash)) {
			mutex_unlock(&flash->lock);
			return 1;
		}

		err = qspi_write_en(flash, TRUE);

		if (flash->cmd_table.qdata.bus_width == X4
			&& !err)
			err = qspi_quad_flag_set(flash, TRUE);

		if (err) {
			mutex_unlock(&flash->lock);
			return 1;
		}

		spi_sync(flash->spi, &m);
		udelay(10000);

		*retlen = m.actual_length - (flash->cmd_table.qaddr.len + 1);

		mutex_unlock(&flash->lock);

	} else {
		u32 i;
		spi_message_init(&m);
		memset(t, 0, sizeof(t));

		/* the size of data remaining on the first page */
		page_size = flash->page_size - page_offset;
		t[0].tx_buf = cmd_addr_buf;
		t[0].len = flash->cmd_table.qaddr.len + 1;

		t[0].bits_per_word = BITS8_PER_WORD;
		set_mode(&t[0], flash->cmd_table.qcmd.is_ddr,
			flash->cmd_table.qcmd.bus_width, opcode);

		spi_message_add_tail(&t[0], &m);

		t[1].tx_buf = buf;

		t[1].len = page_size;
		set_mode(&t[1], flash->cmd_table.qcmd.is_ddr,
			flash->cmd_table.qdata.bus_width, opcode);

		t[1].cs_change = TRUE;
		spi_message_add_tail(&t[1], &m);

		mutex_lock(&flash->lock);
		if (wait_till_ready(flash)) {
			mutex_unlock(&flash->lock);
			return 1;
		}

		err = qspi_write_en(flash, TRUE);

		if (flash->cmd_table.qdata.bus_width == X4
			&& !err)
			err = qspi_quad_flag_set(flash, TRUE);

		if (err) {
			mutex_unlock(&flash->lock);
			return 1;
		}

		spi_sync(flash->spi, &m);

		udelay(10000);
		*retlen = m.actual_length - (flash->cmd_table.qaddr.len + 1);

		mutex_unlock(&flash->lock);

		/* write everything in flash->page_size chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > flash->page_size)
				page_size = flash->page_size;
			/* Need to check for auto address increment */
			/* Check line no 584 to 597 code is required */
			spi_message_init(&m);
			memset(t, 0, sizeof(t));

			offset = to + i;

			/* write the next page to flash */
			cmd_addr_buf[0] = opcode =
				flash->cmd_table.qcmd.op_code;
			cmd_addr_buf[1] = (offset >> 24) & 0xFF;
			cmd_addr_buf[2] = (offset >> 16) & 0xFF;
			cmd_addr_buf[3] = (offset >> 8) & 0xFF;
			cmd_addr_buf[4] = offset & 0xFF;

			t[0].tx_buf = cmd_addr_buf;
			t[0].len = flash->cmd_table.qaddr.len + 1;

			t[0].bits_per_word = BITS8_PER_WORD;
			set_mode(&t[0],
				flash->cmd_table.qcmd.is_ddr,
				flash->cmd_table.qcmd.bus_width,
				opcode);

			spi_message_add_tail(&t[0], &m);

			t[1].tx_buf = (buf + i);
			t[1].len = page_size;
			set_mode(&t[1],
				flash->cmd_table.qcmd.is_ddr,
				flash->cmd_table.qdata.bus_width,
				opcode);

			t[1].cs_change = TRUE;
			spi_message_add_tail(&t[1], &m);

			mutex_lock(&flash->lock);
			if (wait_till_ready(flash)) {
				mutex_unlock(&flash->lock);
				return 1;
			}

			err = qspi_write_en(flash, TRUE);

			if (flash->cmd_table.qdata.bus_width == X4
				&& !err)
				err = qspi_quad_flag_set(flash, TRUE);

			if (err) {
				mutex_unlock(&flash->lock);
				return 1;
			}

			spi_sync(flash->spi, &m);

			udelay(10000);
			*retlen +=
				m.actual_length -
				(flash->cmd_table.qaddr.len + 1);
			mutex_unlock(&flash->lock);
		}
	}
	return 0;
}

static const struct spi_device_id *jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = OPCODE_RDID;
	u8			id[5];
	u32			jedec;
	u16                     ext_jedec;
	struct flash_info	*info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spi_write_then_read(spi, &code, 1, id, 5);
	if (tmp < 0) {
		pr_debug("%s: error %d reading JEDEC ID\n",
				dev_name(&spi->dev), tmp);
		return ERR_PTR(tmp);
	}
	jedec = id[0];
	jedec = jedec << 8;
	jedec |= id[1];
	jedec = jedec << 8;
	jedec |= id[2];

	ext_jedec = id[3] << 8 | id[4];
	for (tmp = 0; tmp < ARRAY_SIZE(qspi_ids) - 1; tmp++) {
		info = (void *)qspi_ids[tmp].driver_data;

		if (info->jedec_id == jedec) {
			if (info->ext_id != 0 && info->ext_id != ext_jedec)
				continue;
			return &qspi_ids[tmp];
		}
	}
	pr_err("unrecognized JEDEC id %06x\n", jedec);
	return ERR_PTR(-ENODEV);
}

static int qspi_probe(struct spi_device *spi)
{
	const struct spi_device_id	*id;
	struct flash_platform_data	*data;
	struct qspi			*flash;
	struct flash_info		*info;
	unsigned			i;
	struct mtd_part_parser_data	ppdata;
	struct device_node __maybe_unused *np;
	struct tegra_qspi_device_controller_data *cdata = spi->controller_data;

	id = spi_get_device_id(spi);
	np = spi->dev.of_node;

#ifdef CONFIG_MTD_OF_PARTS
	if (!of_device_is_available(np))
		return -ENODEV;
#endif

	data = spi->dev.platform_data;
	if (data && data->type) {
		const struct spi_device_id *plat_id;

		for (i = 0; i < ARRAY_SIZE(qspi_ids) - 1; i++) {
			plat_id = &qspi_ids[i];
			if (strcmp(data->type, plat_id->name))
				continue;
			break;
		}

		if (i < ARRAY_SIZE(qspi_ids) - 1)
			id = plat_id;
		else
			dev_warn(&spi->dev, "unrecognized id %s\n", data->type);
	}

	info = (void *)id->driver_data;

	if (info->jedec_id) {
		const struct spi_device_id *jid;

		jid = jedec_probe(spi);
		if (IS_ERR(jid)) {
			return PTR_ERR(jid);
		} else if (jid != id) {
			dev_warn(&spi->dev, "found %s, expected %s\n",
					jid->name, id->name);
			id = jid;
			info = (void *)jid->driver_data;
		}
	}

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	flash->spi = spi;
	mutex_init(&flash->lock);

	dev_set_drvdata(&spi->dev, flash);

	/*
	 * Atmel, SST and Intel/Numonyx serial flash tend to power
	 * up with the software protection bits set
	 */

	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = dev_name(&spi->dev);

	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.writesize = 1;
	flash->mtd.flags = MTD_CAP_NORFLASH;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd._erase = qspi_erase;
	flash->mtd._read = qspi_read;
	flash->mtd._write = qspi_write;
	flash->erase_opcode = OPCODE_SE;
	flash->mtd.erasesize = info->sector_size;

	ppdata.of_node = spi->dev.of_node;
	flash->mtd.dev.parent = &spi->dev;
	flash->page_size = info->page_size;
	flash->mtd.writebufsize = flash->page_size;

	flash->addr_width = ADDRESS_WIDTH;
	cdata = flash->spi->controller_data;

	dev_info(&spi->dev, "%s (%lld Kbytes)\n", id->name,
			(long long)flash->mtd.size >> 10);

	dev_info(&spi->dev,
	"mtd .name = %s, .size = 0x%llx (%lldMiB) .erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->mtd.name,
		(long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
		flash->mtd.erasesize, flash->mtd.erasesize / 1024,
		flash->mtd.numeraseregions);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			dev_info(&spi->dev,
			"mtd.eraseregions[%d] ={.offset = 0x%llx,.erasesize = 0x%.8x (%uKiB),.numblocks = %d}\n",
			i, (long long)flash->mtd.eraseregions[i].offset,
			flash->mtd.eraseregions[i].erasesize,
			flash->mtd.eraseregions[i].erasesize / 1024,
			flash->mtd.eraseregions[i].numblocks);

	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	 */
	return mtd_device_parse_register(&flash->mtd, NULL, &ppdata,
			data ? data->parts : NULL,
			data ? data->nr_parts : 0);
}

static int qspi_remove(struct spi_device *spi)
{
	struct qspi	*flash = dev_get_drvdata(&spi->dev);
	int		status;

	/* Clean up MTD stuff. */
	status = mtd_device_unregister(&flash->mtd);
	if (status == 0)
		kfree(flash);
	return 0;
}

static struct spi_driver qspi_mtd_driver = {
	.driver = {
		.name	= "qspi_mtd",
		.owner	= THIS_MODULE,
	},
	.id_table	= qspi_ids,
	.probe	= qspi_probe,
	.remove	= qspi_remove,

};
module_spi_driver(qspi_mtd_driver);

MODULE_AUTHOR("Amlan Kundu <akundu@nvidia.com>, Ashutosh Patel <ashutoshp@nvidia.com>");
MODULE_DESCRIPTION("MTD SPI driver for Spansion/micron QSPI flash chips");
MODULE_LICENSE("GPL v2");
