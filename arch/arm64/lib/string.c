/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/types.h>

void *memcpy(void *dest, const void *src, size_t count)
{
	uintptr_t d = (uintptr_t) dest;
	uintptr_t s = (uintptr_t) src;

	while (count && (count < 8 || d & 0x7 || s & 0x7)) {
		*((char *) d) = *((char *) s);
		d += 1;
		s += 1;
		count -= 1;
	}
	while (count >= 8) {
		*((u64 *) d) = *((u64 *) s);
		d += 8;
		s += 8;
		count -= 8;
	}
	while (count) {
		*((char *) d) = *((char *) s);
		d += 1;
		s += 1;
		count -= 1;
	}
	return dest;
}
