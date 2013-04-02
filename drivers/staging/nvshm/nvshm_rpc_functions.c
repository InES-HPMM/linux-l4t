/*
 * Copyright (C) 2013 NVIDIA Corporation.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

/* This is a test module */
#include <linux/module.h>
/* printk, pr_* */
#include <linux/slab.h>
#include <linux/printk.h>
/* msleep */
#include <linux/delay.h>
/* cpu_freq */
#include <linux/cpufreq.h>
/* Messaging tools */
#include "nvshm_rpc_utils.h"
/* To register ourself */
#include "nvshm_rpc_dispatcher.h"

static enum rpc_accept_stat rpc_ping(
	u32 version,
	struct nvshm_rpc_message *request,
	struct nvshm_rpc_message **response)
{
	/* Signature: void rpc_ping(void) */
	int length;

	/* Encode response */
	length = nvshm_rpc_utils_encode_size(true, NULL, 0);
	*response = nvshm_rpc_allocresponse(length, request);
	if (!*response)
		return RPC_SYSTEM_ERR;
	nvshm_rpc_utils_encode_response(RPC_SUCCESS, NULL, 0, *response);
	return RPC_SUCCESS;
}

static enum rpc_accept_stat rpc_test(
	u32 version,
	struct nvshm_rpc_message *request,
	struct nvshm_rpc_message **response)
{
	const char *in_string = "";
	s32 in_value = 0;
	char out_string[] = "Response string";
	s32 out_value = 42;
	/* Signature: (string, int) rpc_test(string, int) */
	struct nvshm_rpc_datum_out req_data[] = {
		NVSHM_RPC_OUT_STRING(&in_string),
		NVSHM_RPC_OUT_SINT(&in_value),
	};
	struct nvshm_rpc_datum_in resp_data[] = {
		NVSHM_RPC_IN_STRING(out_string),
		NVSHM_RPC_IN_SINT(out_value),
	};
	int length;

	/* Decode request */
	nvshm_rpc_utils_decode_args(request, false, req_data,
				    ARRAY_SIZE(req_data));
	/* No call here! */
	/* Encode response */
	length = nvshm_rpc_utils_encode_size(true, resp_data,
					     ARRAY_SIZE(resp_data));
	*response = nvshm_rpc_allocresponse(length, request);
	if (!*response)
		return RPC_SYSTEM_ERR;
	nvshm_rpc_utils_encode_response(RPC_SUCCESS, resp_data,
					ARRAY_SIZE(resp_data), *response);
	return RPC_SUCCESS;
}

static enum rpc_accept_stat rpc_msleep(
	u32 version,
	struct nvshm_rpc_message *request,
	struct nvshm_rpc_message **response)
{
	u32 ms = 0;
	/* Signature: void msleep(uint) */
	struct nvshm_rpc_datum_out req_data[] = {
		NVSHM_RPC_OUT_UINT(&ms),
	};
	int length;

	pr_info("entering\n");
	/* Decode request */
	nvshm_rpc_utils_decode_args(request, false, req_data,
				    ARRAY_SIZE(req_data));
	/* Call */
	pr_info("msleep(%u)\n", ms);
	msleep(ms);
	pr_info("mslept(%u)\n", ms);
	/* Encode response */
	length = nvshm_rpc_utils_encode_size(true, NULL, 0);
	*response = nvshm_rpc_allocresponse(length, request);
	if (!*response)
		return RPC_SYSTEM_ERR;
	nvshm_rpc_utils_encode_response(RPC_SUCCESS, NULL, 0, *response);
	pr_info("exiting\n");
	return RPC_SUCCESS;
}

static enum rpc_accept_stat rpc_printk(
	u32 version,
	struct nvshm_rpc_message *request,
	struct nvshm_rpc_message **response)
{
	const char *str;
	/* Signature: <empty> printk(string) */
	struct nvshm_rpc_datum_out req_data[] = {
		NVSHM_RPC_OUT_STRING(&str),
	};
	int length;

	/* Decode request */
	nvshm_rpc_utils_decode_args(request, false, req_data,
				    ARRAY_SIZE(req_data));
	/* Call */
	printk(KERN_INFO "modem: %s\n", str);
	/* Encode response */
	length = nvshm_rpc_utils_encode_size(true, NULL, 0);
	*response = nvshm_rpc_allocresponse(length, request);
	if (!*response)
		return RPC_SYSTEM_ERR;
	nvshm_rpc_utils_encode_response(RPC_SUCCESS, NULL, 0, *response);
	return RPC_SUCCESS;
}

static nvshm_rpc_function_t procedures[] = {
	rpc_ping,
	rpc_test,
	rpc_msleep,
	rpc_printk,
};

static struct nvshm_rpc_program program = {
	.version_min = 0,
	.version_max = 0,
	.procedures_size = ARRAY_SIZE(procedures),
	.procedures = procedures,
};

static int __init nvshm_rpc_test_functions_init(void)
{
	return nvshm_rpc_program_register(NVSHM_RPC_PROGRAM_TEST, &program);
}

static void __exit nvshm_rpc_test_functions_exit(void)
{
	nvshm_rpc_program_unregister(NVSHM_RPC_PROGRAM_TEST);
}

module_init(nvshm_rpc_test_functions_init);
module_exit(nvshm_rpc_test_functions_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hervé Fache <hfache@nvidia.com>");
MODULE_DESCRIPTION("Test functions for NVSHM RPC framework");
