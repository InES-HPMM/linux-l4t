/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

/*
 * nvtouch.h
 *
 *  Created on: Sep 13, 2013
 *      Author: kartamonov
 */

#ifndef NVTOUCH_H_
#define NVTOUCH_H_

#define MAJOR_NUM 100
#define DEVICE_FILE_NAME "nvtouch"
#define DEVICE_FILE_NAME_DTA "nvtouch_dta"
#define DEVICE_FILE_NAME_DTA_ADMIN "nvtouch_dta_admin"

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/printk.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>


#define nv_printf(fmt, ...) \
	pr_err(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#else

#include <string.h>
#include <stdio.h>
#include <sys/time.h>

#include <stdint.h>

#define nv_printf

typedef int8_t s8;
typedef uint8_t u8;

typedef int16_t s16;
typedef uint16_t u16;

typedef int32_t s32;
typedef uint32_t u32;

typedef int64_t s64;
typedef uint64_t u64;

#ifndef __cplusplus
#define true 1
#define false 0
#endif

#endif

#define nvtouch_sqr(x) ((x) * (x))

void nvtouch_reset_history(void);

void nvtouch_init(void *unpacked_data);
struct nvtouch_events *nvtouch_process(void *samples, u64 timestamp);

void *nvtouch_malloc(unsigned int size);

void nvtouch_debug_expose_u32 (const char *name, u32 *var);

#define EXPOSE_VALUE_U32(x) nvtouch_debug_expose_u32(#x, (u32 *) &x);


void nv_assert(int condition);

u64 nvtouch_get_time_us(void);

/* Increasing limit increases ioctl memory footprint */
#define NV_TOUCH_COUNT_MAX 16

struct nvtouch_event_data {
	u32 tracking_id;
	int x0;
	int y0;
	int x1;
	int y1;
	u32 position_x;
	u32 position_y;
	u32 position_z;
	u32 pressure;
	u32 touch_major;
	u32 width_major;
#define NVTOUCH_EVENT_TOOL_PEN 1
#define NVTOUCH_EVENT_TOOL_FINGER 2
#define NVTOUCH_EVENT_TOOL_ERASER 3
	u32 tool;
};

#define NVTOUCH_CONTROL_TX_RX_NORMAL		0x00
#define NVTOUCH_CONTROL_TX_RX_REVERSE		0x01
#define NVTOUCH_CONTROL_TX_RX_ALTERNATE		0x02
#define NVTOUCH_CONTROL_STATE_SLEEP		1
#define NVTOUCH_CONTROL_STATE_IDLE		2
#define NVTOUCH_CONTROL_STATE_ACTIVE		3

/* Skip reporting. Used to improve tap/double tap detection,
 * too many events overload android
 * framework */
#define   NVTOUCH_REPORT_FLAG_SKIP      1

struct nvtouch_events {
	u64 timestamp;
	u32 event_count;
	u32 flags;
	struct nvtouch_event_data events[NV_TOUCH_COUNT_MAX];
};

void nvtouch_set_pm_timeouts(u32 pm_active_to_lp_timeout_ms,
	u32 pm_active_to_idle_timeout_ms);

void nvtouch_report_events(struct nvtouch_events *touch_events);
void nvtouch_set_scan_rate(unsigned int scan_rate);
void nvtouch_set_system_state(unsigned int state);

void nvtouch_calibrate(unsigned int calibrate, char factor);

void nvtouch_unpack_data(void *input, short *output);
u32 nvtouch_get_driver_mode(void);

#define NVTOUCH_DRIVER_VERSION 5

#define NVTOUCH_SENSOR_DATA_RESERVED 5000

struct nvtouch_ioctl_config {
	u32 sensor_w; /* out, ito columns */
	u32 sensor_h; /* out, ito rows */
	u32 screen_w; /* out, pixels */
	u32 screen_h; /* out, pixels */
	u32 invert_x; /* out, 0 or 1 */
	u32 invert_y; /* out, 0 or 1 */
	u32 sensor_data_size; /* out */
	u32 driver_version_kernel;  /* out */
	u32 driver_version_userspace;  /* in */
	u32 driver_config;
#define NVTOUCH_DRIVER_CONFIG_MODE_NVTOUCH_ONLY 0
#define NVTOUCH_DRIVER_CONFIG_MODE_VENDOR_ONLY  1
#define NVTOUCH_DRIVER_CONFIG_MODE_DUAL 2
#define NVTOUCH_DRIVER_CONFIG_MODE_VENDOR_DTA 3
};

struct nvtouch_data_frame {
	u64 timestamp;
	u64 frame_counter;
	u64 irq_timestamp;
	u32 data_format;
	u32 reserved2;
	char samples[NVTOUCH_SENSOR_DATA_RESERVED];
};

struct nvtouch_ioctl_data {
	struct nvtouch_events detected_events;
	struct nvtouch_data_frame data_frame;
	u32 trace_mode;
#define NVTOUCH_TRACE_MODE_DISABLED 0
#define NVTOUCH_TRACE_MODE_RECORD   1
#define NVTOUCH_TRACE_MODE_PLAYBACK 2
#define NVTOUCH_TRACE_MODE_PLAYBACK_STOPPED 3
	char unpacked_data[NVTOUCH_SENSOR_DATA_RESERVED];
	u32 driver_config;
#define NVTOUCH_CONFIG_MODE_MASK 7
#define NVTOUCH_CONFIG_REPORT_MODE_SHIFT (3)
#define NVTOUCH_CONFIG_REPORT_MODE_MASK  (7 << 3)
#define NVTOUCH_CONFIG_ENABLE_DTA (1 << 8)
#define NVTOUCH_CONFIG_ENABLE_CALIBRATION (1 << 9)
#define NVTOUCH_CONFIG_ENABLE_CALIBRATION_DEBUG (1 << 10)
#define NVTOUCH_CONFIG_TUNE_DEBUG (1 << 11)
#define NVTOUCH_CONFIG_TUNE_PARAM_SHIFT (12)
#define NVTOUCH_CONFIG_TUNE_PARAM_MASK  (255 << 12)

	struct nvtouch_events vendor_events;
	u32 pm_active_to_lp_timeout_ms;
	u32 pm_active_to_idle_timeout_ms;
};

struct nvtouch_ioctl_dta_config {
	u32 sensor_w; /* out, ito columns */
	u32 sensor_h; /* out, ito rows */
	u32 screen_w; /* out, pixels */
	u32 screen_h; /* out, pixels */
	u32 invert_x; /* out, 0 or 1 */
	u32 invert_y; /* out, 0 or 1 */
	u32 sensor_data_size; /* out */
	u32 driver_version_kernel;  /* out */
	u32 driver_version_userspace;  /* in */
};

struct nvtouch_ioctl_dta {
	u64 timestamp;
	struct nvtouch_events detected_events;
	char data[NVTOUCH_SENSOR_DATA_RESERVED];
};

struct nvtouch_ioctl_dta_old {
	u64 timestamp;
	struct nvtouch_events detected_events;
	short data[NVTOUCH_SENSOR_DATA_RESERVED];
};

struct nvtouch_ioctl_dta_admin_config {
	u32 driver_version_kernel;  /* out */
	u32 driver_version_userspace;  /* in */
};

struct nvtouch_ioctl_dta_admin_data {
#define NVTOUCH_DTA_ADMIN_NULL_PID 0
	u32 pid; /* in */
};

struct nvtouch_ioctl_dta_admin_query {
	u32 connected_clients; /* out */
};

struct nvtouch_ioctl_debug_touch {
	u64 timestamp;
};

#define NVTOUCH_IOCTL_MAGIC 'H'

#define NVTOUCH_IOCTL_CONFIG _IOWR(NVTOUCH_IOCTL_MAGIC, 1,\
	struct nvtouch_ioctl_config)
#define NVTOUCH_IOCTL_UPDATE _IOWR(NVTOUCH_IOCTL_MAGIC, 2,\
	struct nvtouch_ioctl_data)
#define NVTOUCH_IOCTL_LAST (_IOC_NR(NVTOUCH_IOCTL_UPDATE))
#define NVTOUCH_IOCTL_MAX_ARG_SIZE sizeof(struct nvtouch_ioctl_data)

/*
* DTA Devices
*/
#define NVTOUCH_DTA_IOCTL_MAGIC 'I'
#define NVTOUCH_DTA_IOCTL_CONFIG _IOWR(NVTOUCH_DTA_IOCTL_MAGIC, 1,\
	struct nvtouch_ioctl_dta_config)
#define NVTOUCH_DTA_IOCTL_READ _IOWR(NVTOUCH_DTA_IOCTL_MAGIC, 2,\
	struct nvtouch_ioctl_dta)
/* TODO: remove this ioctl after userspace tests are fixed */
#define NVTOUCH_DTA_IOCTL_READ_OLD _IOWR(NVTOUCH_DTA_IOCTL_MAGIC, 2,\
	struct nvtouch_ioctl_dta_old)
#define NVTOUCH_DTA_IOCTL_LAST (_IOC_NR(NVTOUCH_DTA_IOCTL_READ))
#define NVTOUCH_DTA_IOCTL_MAX_ARG_SIZE sizeof(struct nvtouch_ioctl_dta)

#define NVTOUCH_DTA_ADMIN_IOCTL_MAGIC 'J'
#define NVTOUCH_DTA_ADMIN_IOCTL_CONFIG _IOWR(NVTOUCH_DTA_ADMIN_IOCTL_MAGIC, 1,\
	struct nvtouch_ioctl_dta_admin_config)
#define NVTOUCH_DTA_ADMIN_IOCTL_UPDATE _IOWR(NVTOUCH_DTA_ADMIN_IOCTL_MAGIC, 2,\
	struct nvtouch_ioctl_dta_admin_data)
#define NVTOUCH_DTA_ADMIN_IOCTL_QUERY _IOWR(NVTOUCH_DTA_ADMIN_IOCTL_MAGIC, 3,\
	struct nvtouch_ioctl_dta_admin_query)
#define NVTOUCH_DTA_ADMIN_IOCTL_SEND_DEBUG_TOUCH _IOWR(NVTOUCH_DTA_IOCTL_MAGIC,\
	4, struct nvtouch_ioctl_debug_touch)
#define NVTOUCH_DTA_ADMIN_IOCTL_RECV_DEBUG_TOUCH _IOWR(NVTOUCH_DTA_IOCTL_MAGIC,\
	5, struct nvtouch_ioctl_debug_touch)
#define NVTOUCH_DTA_ADMIN_IOCTL_LAST \
	(_IOC_NR(NVTOUCH_DTA_ADMIN_IOCTL_RECV_DEBUG_TOUCH))
#define NVTOUCH_DTA_ADMIN_IOCTL_MAX_ARG_SIZE \
	sizeof(struct nvtouch_ioctl_dta_admin_config)

#endif /* NVTOUCH_H_ */
