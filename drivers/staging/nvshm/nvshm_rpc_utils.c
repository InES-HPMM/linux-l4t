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

#define pr_fmt(fmt) "%s:" fmt, __func__

#include <nvshm_rpc_utils.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/export.h>
#include <linux/sunrpc/xdr.h>

/*
 * Call fields (on top of header)
 * - RPC version            (=2)
 * - Program
 * - Program version
 * - Procedure
 * - Credentials (x2)       (=0,0)
 * - Verifier (x2)          (=0,0)
 */
#define SUN_RPC_CALL_HDR_SIZE 8

/*
 * Call fields (on top of header)
 * - Reply status (always accepted here)
 * - Verifier (x2)          (=0,0)
 * - Accept status
 * (Mismatch info to be allocated as opaque)
 */
#define SUN_RPC_ACC_REPLY_HDR_SIZE 4

static int nvshm_rpc_utils_encode_args(const struct nvshm_rpc_datum_in *data,
				       u32 number,
				       u32 *writer)
{
	u32 n;

	for (n = 0; n < number; ++n) {
		const struct nvshm_rpc_datum_in *datum = &data[n];
		switch (datum->type) {
		case TYPE_SINT:
			*writer++ = cpu_to_be32(datum->d.sint_data);
			break;
		case TYPE_UINT:
			*writer++ = cpu_to_be32(datum->d.uint_data);
			break;
		case TYPE_STRING:
			writer = xdr_encode_opaque(writer,
					datum->d.string_data,
					strlen(datum->d.string_data) + 1);
			break;
		case TYPE_BLOB:
			writer = xdr_encode_opaque(writer,
					datum->d.blob_data,
					datum->length);
			break;
		default:
			pr_err("unknown RPC type %d\n", datum->type);
			return -EINVAL;
		}
	}
	return 0;
}

int nvshm_rpc_utils_encode_size(bool is_response,
				const struct nvshm_rpc_datum_in *data,
				u32 number)
{
	int quad_length;
	u32 n;

	if (is_response)
		quad_length = SUN_RPC_ACC_REPLY_HDR_SIZE;
	else
		quad_length = SUN_RPC_CALL_HDR_SIZE;
	for (n = 0; n < number; ++n) {
		const struct nvshm_rpc_datum_in *datum = &data[n];
		switch (datum->type) {
		case TYPE_SINT:
		case TYPE_UINT:
			++quad_length;
			break;
		case TYPE_STRING:
			++quad_length;
			quad_length += XDR_QUADLEN(
				strlen(datum->d.string_data) + 1);
			break;
		case TYPE_BLOB:
			++quad_length;
			quad_length += XDR_QUADLEN(datum->length);
			break;
		default:
			pr_err("unknown RPC type %d\n", datum->type);
			return -EINVAL;
		}
	}
	return quad_length << 2;
}
EXPORT_SYMBOL_GPL(nvshm_rpc_utils_encode_size);

int nvshm_rpc_utils_encode_request(const struct nvshm_rpc_procedure *procedure,
				   const struct nvshm_rpc_datum_in *data,
				   u32 number,
				   struct nvshm_rpc_message *message)
{
	u32 *writer = message->payload;

	/* RPC version */
	*writer++ = cpu_to_be32(2);
	/* Procedure */
	*writer++ = cpu_to_be32(procedure->program);
	*writer++ = cpu_to_be32(procedure->version);
	*writer++ = cpu_to_be32(procedure->procedure);
	/* Authentication (AUTH_NONE, size = 0) */
	*writer++ = cpu_to_be32(0);
	*writer++ = cpu_to_be32(0);
	/* Verifier (AUTH_NONE, size = 0) */
	*writer++ = cpu_to_be32(0);
	*writer++ = cpu_to_be32(0);
	return nvshm_rpc_utils_encode_args(data, number, writer);
}

int nvshm_rpc_utils_encode_response(enum rpc_accept_stat status,
				    const struct nvshm_rpc_datum_in *data,
				    u32 number,
				    struct nvshm_rpc_message *message)
{
	u32 *writer = message->payload;

	/* Reply status (always accepted) */
	*writer++ = cpu_to_be32(0);
	/* Verifier (AUTH_NONE, size = 0) */
	*writer++ = cpu_to_be32(0);
	*writer++ = cpu_to_be32(0);
	/* Accept status */
	*writer++ = cpu_to_be32(status);
	return nvshm_rpc_utils_encode_args(data, number, writer);
}
EXPORT_SYMBOL_GPL(nvshm_rpc_utils_encode_response);

void nvshm_rpc_utils_decode_procedure(const struct nvshm_rpc_message *request,
				      struct nvshm_rpc_procedure *procedure)
{
	const u32 *reader = request->payload;

	/* Skip RPC version */
	reader += 1;
	procedure->program = be32_to_cpup((__be32 *) reader++);
	procedure->version = be32_to_cpup((__be32 *) reader++);
	procedure->procedure = be32_to_cpup((__be32 *) reader);
}

enum rpc_accept_stat
nvshm_rpc_utils_decode_status(const struct nvshm_rpc_message *response)
{
	const u32 *reader = response->payload;

	/* Skip reply status and verifier */
	reader += 3;
	return be32_to_cpup((__be32 *) reader);
}

int nvshm_rpc_utils_decode_versions(
	const struct nvshm_rpc_message *response,
	u32 *version_min,
	u32 *version_max)
{
	struct nvshm_rpc_datum_out versions[] = {
		NVSHM_RPC_OUT_UINT(version_min),
		NVSHM_RPC_OUT_UINT(version_max),
	};
	return nvshm_rpc_utils_decode_args(response, true, versions,
					   ARRAY_SIZE(versions));
}

int nvshm_rpc_utils_decode_args(const struct nvshm_rpc_message *message,
				bool is_response,
				struct nvshm_rpc_datum_out *data,
				u32 number)
{
	const __be32 *reader = message->payload;
	u32 n;

	if (is_response)
		reader += SUN_RPC_ACC_REPLY_HDR_SIZE;
	else
		reader += SUN_RPC_CALL_HDR_SIZE;
	for (n = 0; n < number; ++n) {
		struct nvshm_rpc_datum_out *datum = &data[n];
		u32 uint;

		/* There is always a number, either the data ot its length */
		uint = be32_to_cpup((__be32 *) reader);
		++reader;
		switch (datum->type) {
		case TYPE_SINT:
			*datum->d.sint_data = (s32) uint;
			break;
		case TYPE_UINT:
			*datum->d.uint_data = uint;
			break;
		case TYPE_STRING:
			*datum->d.string_data = (const char *) reader;
			reader += XDR_QUADLEN(uint);
			break;
		case TYPE_BLOB:
			*datum->length = uint;
			*datum->d.blob_data = reader;
			reader += XDR_QUADLEN(uint);
			break;
		default:
			pr_err("unknown RPC type %d\n", datum->type);
			return -EINVAL;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(nvshm_rpc_utils_decode_args);
