/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of NVIDIA CORPORATION nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NVIDIA CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __TEGRA_IVC_H
#define __TEGRA_IVC_H

#include <linux/types.h>

struct device_node;

/* in kernel interfaces */

struct tegra_hv_ivc_ops;

struct tegra_hv_ivc_cookie {
	/* some fields that might be useful */
	int irq;
	int peer_vmid;
	int nframes;
	int frame_size;
};

struct tegra_hv_ivc_ops {
	/* called when data are received */
	void (*rx_rdy)(struct tegra_hv_ivc_cookie *ivck);
	/* called when space is available to write data */
	void (*tx_rdy)(struct tegra_hv_ivc_cookie *ivck);
};

struct ivc;

/**
 * tegra_hv_ivc_reserve - Reserve an IVC queue for use
 * @dn:		Device node pointer to the queue in the DT
 *		If NULL, then operate on first HV device
 * @queue_id	Id number of the queue to use.
 * @ops		Ops structure or NULL
 *
 * Reserves the queue for use
 *
 * Returns a pointer to the ivc_dev to use or an ERR_PTR.
 * Note that returning EPROBE_DEFER means that the ivc driver
 * hasn't loaded yet and you should try again later in the
 * boot sequence.
 */
struct tegra_hv_ivc_cookie *tegra_hv_ivc_reserve(
		struct device_node *dn, int id,
		const struct tegra_hv_ivc_ops *ops);

/**
 * tegra_hv_ivc_unreserve - Unreserve an IVC queue used
 * @ivck	IVC cookie
 *
 * Unreserves the IVC channel
 *
 * Returns 0 on success and an error code otherwise
 */
int tegra_hv_ivc_unreserve(struct tegra_hv_ivc_cookie *ivck);

/**
 * ivc_hv_ivc_write - Writes a frame to the IVC queue
 * @ivck	IVC cookie of the queue
 * @buf		Pointer to the data to write
 * @size	Size of the data to write
 *
 * Write a number of bytes (as a single frame) from the queue.
 *
 * Returns size on success and an error code otherwise
 */
int tegra_hv_ivc_write(struct tegra_hv_ivc_cookie *ivck, const void *buf,
		int size);
int tegra_ivc_write(struct ivc *ivc, const void *buf, size_t size);

/**
 * ivc_hv_ivc_read - Reads a frame from the IVC queue
 * @ivck	IVC cookie of the queue
 * @buf		Pointer to the data to read
 * @size	max size of the data to read
 *
 * Reads a number of bytes (as a single frame) from the queue.
 *
 * Returns size on success and an error code otherwise
 */
int tegra_hv_ivc_read(struct tegra_hv_ivc_cookie *ivck, void *buf, int size);
int tegra_ivc_read(struct ivc *ivc, void *buf, size_t size);

/**
 * ivc_hv_ivc_can_read - Test whether data are available
 * @ivck	IVC cookie of the queue
 *
 * Test wheter data to read are available
 *
 * Returns 1 if data are available in the rx queue, 0 if not
 */
int tegra_hv_ivc_can_read(struct tegra_hv_ivc_cookie *ivck);
int tegra_ivc_can_read(struct ivc *ivc);

/**
 * ivc_hv_ivc_can_write - Test whether data can be written
 * @ivck	IVC cookie of the queue
 *
 * Test wheter data can be written
 *
 * Returns 1 if data are can be written to the tx queue, 0 if not
 */
int tegra_hv_ivc_can_write(struct tegra_hv_ivc_cookie *ivck);
int tegra_ivc_can_write(struct ivc *ivc);

/**
 * ivc_hv_ivc_tx_empty - Test whether the tx queue is empty
 * @ivck	IVC cookie of the queue
 *
 * Test wheter the tx queue is completely empty
 *
 * Returns 1 if the queue is empty, zero otherwise
 */
int tegra_hv_ivc_tx_empty(struct tegra_hv_ivc_cookie *ivck);
int tegra_ivc_tx_empty(struct ivc *ivc);

/**
 * ivc_hv_ivc_loopback - Sets (or clears) loopback mode
 * @ivck	IVC cookie of the queue
 * @mode	Set loopback on/off (1 = on, 0 = off)
 *
 * Sets or clears loopback mode accordingly.
 *
 * When loopback is active any writes are ignored, while
 * reads do not return data.
 * Incoming data are copied immediately to the tx queue.
 *
 * Returns 0 on success, a negative error code otherwise
 */
int tegra_hv_ivc_set_loopback(struct tegra_hv_ivc_cookie *ivck, int mode);

/* debugging aid */
int tegra_hv_ivc_dump(struct tegra_hv_ivc_cookie *ivck);

/**
 * ivc_hv_ivc_read_peek - Peek (copying) data from a received frame
 * @ivck	IVC cookie of the queue
 * @buf		Buffer to receive the data
 * @off		Offset in the frame
 * @count	Count of bytes to copy
 *
 * Peek data from a received frame, copying to buf, without removing
 * the frame from the queue.
 *
 * Returns 0 on success, a negative error code otherwise
 */
int tegra_hv_ivc_read_peek(struct tegra_hv_ivc_cookie *ivck,
		void *buf, int off, int count);
int tegra_ivc_read_peek(struct ivc *ivc, void *buf, size_t off, size_t count);

/**
 * ivc_hv_ivc_read_get_next_frame - Peek at the next frame to receive
 * @ivck	IVC cookie of the queue
 *
 * Peek at the next frame to be received, without removing it from
 * the queue.
 *
 * Returns a pointer to the frame, or an error encoded pointer.
 */
void *tegra_hv_ivc_read_get_next_frame(struct tegra_hv_ivc_cookie *ivck);
void *tegra_ivc_read_get_next_frame(struct ivc *ivc);

/**
 * ivc_hv_ivc_read_advance - Advance the read queue
 * @ivck	IVC cookie of the queue
 *
 * Advance the read queue
 *
 * Returns 0, or a negative error value if failed.
 */
int tegra_hv_ivc_read_advance(struct tegra_hv_ivc_cookie *ivck);
int tegra_ivc_read_advance(struct ivc *ivc);

/**
 * ivc_hv_ivc_write_poke - Poke data to a frame to be transmitted
 * @ivck	IVC cookie of the queue
 * @buf		Buffer to the data
 * @off		Offset in the frame
 * @count	Count of bytes to copy
 *
 * Copy data to a transmit frame, copying from buf, without advancing
 * the the transmit queue.
 *
 * Returns 0 on success, a negative error code otherwise
 */
int tegra_hv_ivc_write_poke(struct tegra_hv_ivc_cookie *ivck,
		const void *buf, int off, int count);
int tegra_ivc_write_poke(struct ivc *ivc, const void *buf, size_t off,
		size_t count);

/**
 * ivc_hv_ivc_write_get_next_frame - Poke at the next frame to transmit
 * @ivck	IVC cookie of the queue
 *
 * Get access to the next frame.
 *
 * Returns a pointer to the frame, or an error encoded pointer.
 */
void *tegra_hv_ivc_write_get_next_frame(struct tegra_hv_ivc_cookie *ivck);
void *tegra_ivc_write_get_next_frame(struct ivc *ivc);

/**
 * ivc_hv_ivc_write_advance - Advance the write queue
 * @ivck	IVC cookie of the queue
 *
 * Advance the write queue
 *
 * Returns 0, or a negative error value if failed.
 */
int tegra_hv_ivc_write_advance(struct tegra_hv_ivc_cookie *ivck);
int tegra_ivc_write_advance(struct ivc *ivc);

#endif
