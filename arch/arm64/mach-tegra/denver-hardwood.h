/*
 * Copyright (c) 2014 NVIDIA Corporation. All rights reserved.
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

#ifndef _MACH_DENVER_HARDWOOD_H_
#define _MACH_DENVER_HARDWOOD_H_

#define N_CPU 2
#define N_BUFFER 4
#define BUFFER_SIZE (1 << 22)

struct hardwood_cmd {
	__u8 core_id;
	__u8 buffer_id;
	__u64 data;
	__u64 data2;
} __packed;

/* Shouldn't be called by userspace */
#define HARDWOOD_SET_PHYS_ADDR		0x00
#define HARDWOOD_SET_BUFFER_SIZE	0x01

#define HARDWOOD_ENABLE_TRACE_DUMP	0x02
#define HARDWOOD_DISABLE_TRACE_DUMP	0x03
#define HARDWOOD_GET_IP_ADDRESS		0x04
#define HARDWOOD_SET_TRACER_MASK	0x05
#define HARDWOOD_CLR_TRACER_MASK	0x06
#define HARDWOOD_SET_IRQ_TARGET		0x07

#define HARDWOOD_GET_STATUS			0x10
#define HARDWOOD_GET_BYTES_USED		0x11
#define HARDWOOD_MARK_EMPTY			0x12
#define HARDWOOD_RELEASE_BUFFER		0x13
#define HARDWOOD_OVERFLOW_COUNT		0x14

#define HARDWOOD_GET_PHYS_ADDR		0x21
#define HARDWOOD_WAKEUP_READERS		0x22

#endif
