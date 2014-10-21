/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _BPMP_ABI_H
#define _BPMP_ABI_H

#include <mach/irqs.h>

/* CPU to BPMP atomic channels */
#define CPU0_OB_CH0		0
#define CPU1_OB_CH0		1
#define CPU2_OB_CH0		2
#define CPU3_OB_CH0		3

/* CPU to BPMP non-atomic channels */
#define CPU0_OB_CH1		4
#define CPU1_OB_CH1		5
#define CPU2_OB_CH1		6
#define CPU3_OB_CH1		7

/* BPMP to CPU channels */
#define CPU0_IB_CH		8
#define CPU1_IB_CH		9
#define CPU2_IB_CH		10
#define CPU3_IB_CH		11

#define NR_CHANNELS		12
#define MSG_SZ			32
#define MSG_DATA_SZ		24

#define CPU_OB_IRQ		INT_SHR_SEM_OUTBOX_IBF
#define CPU0_IB_IRQ		INT_SHR_SEM_INBOX_IBF
#define CPU1_IB_IRQ		INT_SHR_SEM_INBOX_IBE
#define CPU2_IB_IRQ		INT_SHR_SEM_OUTBOX_IBE
#define CPU3_IB_IRQ		INT_AVP_UCQ

#define DO_ACK			(1 << 0)
#define RING_DOORBELL		(1 << 1)

#define __MRQ_ATTRS		0xff000000
#define __MRQ_PUBLIC		(1 << 31)

#define MRQ_PING		(0 | __MRQ_PUBLIC)
#define MRQ_QUERY_TAG		1
#define MRQ_DO_IDLE		2
#define MRQ_TOLERATE_IDLE	3
#define MRQ_MODULE_LOAD		4
#define MRQ_MODULE_UNLOAD	5
#define MRQ_SWITCH_CLUSTER	6
#define MRQ_TRACE_MODIFY	7
#define MRQ_WRITE_TRACE		8
#define MRQ_THREADED_PING	(9 | __MRQ_PUBLIC)
#define MRQ_CPUIDLE_USAGE	10
#define MRQ_MODULE_MAIL		(11 | __MRQ_PUBLIC)
#define MRQ_SCX_ENABLE		12
#define MRQ_INIT_NR_CPUS	13
#define MRQ_BPMPIDLE_USAGE	(14 | __MRQ_PUBLIC)
#define MRQ_HEAP_USAGE		(15 | __MRQ_PUBLIC)
#define MRQ_SCLK_SKIP_SET_RATE	16

#define NR_MRQS			32

#endif
