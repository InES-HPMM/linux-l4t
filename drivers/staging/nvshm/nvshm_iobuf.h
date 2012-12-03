/*
 * Copyright (C) 2012 NVIDIA Corporation.
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

#ifndef _NVSHM_IOBUF_H
#define _NVSHM_IOBUF_H

/* Baseband base address in BB memory space - this is a constant */
#define NVSHM_IPC_BB_BASE (0x8C000000)

/**
 * NVSHM_B2A convert from Baseband address space
 * to AP virtual kernel space (cached)
 *
 * All iobuf conversion are done from/to cached kernel space
 *
 * @param h : struct nvshm_handle pointer
 * @param x : address to convert
 * @return : void * pointer in cached kernel space
 */
#define NVSHM_B2A(h, x) ((void *)(x) + ((int)(h)->ipc_base_virt) \
			 - NVSHM_IPC_BB_BASE)

/**
 * NVSHM_A2B convert from AP kernel space (cached) to Baseband address space
 *
 * All iobuf conversion are done from/to cached kernel space
 *
 * @param h : struct nvshm_handle pointer
 * @param x : address to convert
 * @return : void * pointer in BB memory space
 */
#define NVSHM_A2B(h, x) ((void *)(x) - ((int)(h)->ipc_base_virt) \
			 + NVSHM_IPC_BB_BASE)

/**
 * Alloc a nvshm_iobuf descriptor to be used for write operation
 * Failure of allocation is considered as an Xoff situation and
 * will be followed by a call to (*start_tx)() operation when flow
 * control return to Xon. If excessive size is requested, call to
 * (*error_event)() with NVSHM_IOBUF_ERROR will be raised synchronously
 *
 * @param struct nvshm_channel handle
 * @param size - data size requested in bytes
 * @return iobuf pointer or
 * NULL if no iobuf can be allocated (flow control Xoff)
 */
struct nvshm_iobuf *nvshm_iobuf_alloc(struct nvshm_channel *handle, int size);

/**
 * Free a nvshm_iobuf descriptor given in rx_event
 * pointers are not followed and cleared on free
 *
 * @param struct nvshm_iobuf descriptor to free
 *
 */
void nvshm_iobuf_free(struct nvshm_iobuf *iob);

/**
 * Free a nvshm_iobuf descriptor list given in rx_event
 * both ->next and ->sg_next are followed
 *
 * @param struct nvshm_iobuf list of descriptor to free
 *
 */
void nvshm_iobuf_free_cluster(struct nvshm_iobuf *list);

/**
 * clear/set nvshm_iobuf internal flags (unused/unspecified for now)
 *
 * @param struct nvshm_iobuf descriptor
 * @param unsigned int set value
 * @param unsigned int clear value
 * @return 0 if no error
 */
int nvshm_iobuf_update_bits(struct nvshm_iobuf *iob,
			    unsigned int clear, unsigned int set);

/**
 * Increase reference count of iobuf
 *
 * @param struct nvshm_iobuf descriptor
 * @return previous ref value
 */
int nvshm_iobuf_ref(struct nvshm_iobuf *iob);

/**
 * Decrease reference count of iobuf
 *
 * @param struct nvshm_iobuf descriptor
 * @return previous ref value
 */
int nvshm_iobuf_unref(struct nvshm_iobuf *iob);

/**
 * Increase reference count of iobuf cluster
 *
 * @param struct nvshm_iobuf descriptor
 * @return previous maximum ref value
 */
int nvshm_iobuf_ref_cluster(struct nvshm_iobuf *iob);

/**
 * Decrease reference count of iobuf cluster
 *
 * @param struct nvshm_iobuf descriptor
 * @return previous maximum ref value
 */
int nvshm_iobuf_unref_cluster(struct nvshm_iobuf *iob);

/**
 * Check if iobuf pointers are sane
 *
 * @param handle to nvshm channel
 * @param struct nvshm_iobuf to check
 * @return 0 if sane
 */
int nvshm_iobuf_check(struct nvshm_channel *chan, struct nvshm_iobuf *iob);

/**
 * Init iobuf subsystem
 *
 * @param handle to nvshm channel
 * @return 0 if ok negative otherwise
 */
int nvshm_iobuf_init(struct nvshm_handle *handle);

#endif /* _NVSHM_IOBUF_H */
