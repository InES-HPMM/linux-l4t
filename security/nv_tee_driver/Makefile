#
# Copyright (c) 2013, NVIDIA Corporation.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#

plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_tee_irq.o :=-Wa,-march=armv7-a$(plus_sec)
CFLAGS_tee_comms.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)

nv_tee_driver-objs += tee_device.o
nv_tee_driver-objs += tee_comms.o
nv_tee_driver-objs += tee_irq.o

obj-$(CONFIG_TRUSTED_LITTLE_KERNEL) += nv_tee_driver.o
