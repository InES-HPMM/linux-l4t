/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <osl.h>

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/memory.h>
#include "nvram_params.h"

#include <linux/kernel.h>
#include <linux/crypto.h>

char current_ccode[3];
char current_nvram_code[3];

unsigned char nvram_params[MAX_NVRAMBUF_SIZE];

char *country_to_nvram_code(char *country_code)
{
	char *nvram_code;
	const char EU_countries[] = "AT BE BG HR CZ DK FR DE GR HU IT NL NO PL PT RO SK ES SE CH GB";

	nvram_code = (char *)kmalloc(COUNTRY_CODE_LEN + 1, GFP_KERNEL);
	if (nvram_code == NULL) {
		pr_err("%s: fail to allocate memory\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	if (strncmp(country_code, "US", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "US", 2);
	} else if (strncmp(country_code, "IN", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "US", 2);
	} else if (strncmp(country_code, "CA", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "US", 2);
	} else if (strncmp(country_code, "EU", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "E0", 2);
	} else if (strncmp(country_code, "AU", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "E0", 2);
	} else if (strstr(EU_countries, country_code)) {
		strncpy(nvram_code, "E0", 2);
	} else if (strncmp(country_code, "RU", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "RU", 2);
	} else if (strncmp(country_code, "JP", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "JP", 2);
	} else if (strncmp(country_code, "KR", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "KR", 2);
	} else if (strncmp(country_code, "TW", COUNTRY_CODE_LEN) == 0) {
		strncpy(nvram_code, "TW", 2);
	} else {
		strncpy(nvram_code, "XV", 2);
	}
	nvram_code[2] = '\0';
	return nvram_code;
}

int get_country_code_fs(char *ccode)
{
	struct file *fp;
	int len;
	char country[2];

	fp = filp_open(WIFI_COUNTRY_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("Failed to open %s\n", WIFI_COUNTRY_FILE);
		return -1;
	}
	len = kernel_read(fp, 0, country, 2);
	filp_close(fp, NULL);

	if (len != 2) {
		pr_err("Bad country code: %s, ret: %d\n", country, len);
		return -1;
	} else {
		strncpy(ccode, country, 2);
		return 0;
	}
}

int set_country_code_fs(char *ccode)
{
	struct file *fp;
	int len;

	fp = filp_open(WIFI_COUNTRY_FILE, O_WRONLY | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("Failed to open %s\n", WIFI_COUNTRY_FILE);
		return -1;
	}

	if (ccode == NULL || strlen(ccode) < 2) {
		pr_err("Invalid country code\n");
		return -1;
	} else {
		len = kernel_write(fp, ccode, 2, 0);
		pr_err("country write %d\n", len);
	}
	filp_close(fp, NULL);
	return 0;
}

/* Read encrypted nvram file and return plain text in nvram_plain */
int get_nvram_plain(char *nvram_plain)
{
	struct file *fp;
	struct crypto_cipher *tfm;
	char *buf;
	int len;
	const u8 key[] = "nvram_bcm43241";
	int i;
	int bsize;

	tfm = crypto_alloc_cipher("aes", 0, CRYPTO_ALG_ASYNC);
	crypto_cipher_setkey(tfm, key, sizeof(key));
	bsize = crypto_cipher_blocksize(tfm);

	fp = filp_open(WIFI_NVRAM_FILE_BIN, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s: Failed to open: %s", __func__, WIFI_NVRAM_FILE_BIN);
		return -1;
	}

	buf = (char *)kmalloc(MAX_ALL_NVRAMBUF_SIZE, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("%s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	memset(buf, 0, MAX_ALL_NVRAMBUF_SIZE);
	len = kernel_read(fp, 0, buf, MAX_ALL_NVRAMBUF_SIZE);
	filp_close(fp, NULL);
	if (len < 0) {
		pr_err("%s: file read fail\n", __func__);
		return -1;
	} else {
		pr_err("%s: file read size %d\n", __func__, len);
	}

	for (i = 0; i < len; i += bsize) {
		crypto_cipher_decrypt_one(tfm, buf + i, buf + i);
	}
	memcpy(nvram_plain, buf, len);
	kfree(buf);
	crypto_free_cipher(tfm);
	return 0;
}

int get_nvram_param(char *country)
{
	char *all_nvram;
	char nvram_start[13];
	char nvram_end[11];
	char *nvram_ptr_start = NULL;
	char *nvram_ptr_end = NULL;
	int nvram_param_size;

	sprintf(nvram_start, "==%c%c-START==", country[0], country[1]);
	sprintf(nvram_end, "==%c%c-END==", country[0], country[1]);

	/* open file read 28K nvram data */
	all_nvram = (char *)kmalloc(MAX_ALL_NVRAMBUF_SIZE, GFP_KERNEL);
	if (all_nvram == NULL) {
		pr_err("%s: fail to allocate memory\n", __func__);
		return -ENOMEM;
	}
	memset(all_nvram, 0, MAX_ALL_NVRAMBUF_SIZE);
	if (get_nvram_plain(all_nvram)) {
		pr_err("%s: fail to read nvram plain\n", __func__);
		return -1;
	}
	/* get nvram param for country CC */
	nvram_ptr_start = strstr(all_nvram, nvram_start);
	nvram_ptr_start = nvram_ptr_start + 12;
	if (nvram_ptr_start == NULL) {
		pr_err("nvram_ptr_start null");
		return -1;
	}
	nvram_ptr_end = strstr(all_nvram, nvram_end);
	if (nvram_ptr_end == NULL) {
		pr_err("nvram_ptr_end null");
		return -1;
	}

	nvram_param_size = nvram_ptr_end - nvram_ptr_start;
	if (sizeof(nvram_params) < nvram_param_size) {
		pr_err("%s: no enough memory to save nvram params\n", __func__);
		return -1;
	} else {
		memcpy(nvram_params, nvram_ptr_start, nvram_param_size);
		return 0;
	}
}

