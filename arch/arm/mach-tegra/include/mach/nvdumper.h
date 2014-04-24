/*
 * arch/arm64/mach-tegra/include/mach/nvdumper.h
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __MACH_TEGRA_NVDUMPER_H
#define __MACH_TEGRA_NVDUMPER_H

/*
 * This register list is from table Table D1-81 of
 * ARMv8 architecture reference manual
 * The table also includes the mapping of AArch64
 * to AArch32 registers.
 */
struct aar64_system_regs_t {
	u64 ACTLR_EL1;
	u64 AFSR0_EL1;
	u64 AFSR1_EL1;
	u64 AMAIR_EL1;
	u64 CONTEXTIDR_EL1;
	u64 CPACR_EL1;
	u64 CSSELR_EL1;
	u64 FAR_EL1;
	u64 ESR_EL1;
	u64 PAR_EL1;
	u64 MAIR_EL1;
	u64 RMR_EL1;
	u64 SCTLR_EL1;
	u64 TEECR32_EL1;
	u64 TEEHBR32_EL1;
	u64 TPIDR_EL1;
	u64 TCR_EL1;
	u64 TTBR0_EL1;
	u64 TTBR1_EL1;
	u64 VBAR_EL1;

	u64 TPIDRRO_EL0;
	u64 TPIDR_EL0;

	u64 DACR32_EL2;
	u64 HACR_EL2;
	u64 ACTLR_EL2;
	u64 AFSR0_EL2;
	u64 AFSR1_EL2;
	u64 AMAIR_EL2;
	u64 CPTR_EL2;
	u64 HCR_EL2;
	u64 MDCR_EL2;
	u64 FAR_EL2;
	u64 MAIR_EL2;
	u64 HPFAR_EL2;
	u64 SCTLR_EL2;
	u64 ESR_EL2;
	u64 HSTR_EL2;
	u64 TCR_EL2;
	u64 TPIDR_EL2;
	u64 TTBR0_EL2;
	u64 VBAR_EL2;
	u64 IFSR32_EL2;
	u64 RMR_EL2;
	u64 VMPIDR_EL2;
	u64 VPIDR_EL2;
	u64 VTCR_EL2;
	u64 VTTBR_EL2;

	u64 RMR_EL3;
	u64 SDER32_EL3;
};

struct nvdumper_cpu_data_t {
	bool is_online;
	struct pt_regs pt_regs;
	struct aar64_system_regs_t aar64_sys_regs;
	struct task_struct *current_task;
};

int nvdumper_regdump_init(void);
void nvdumper_regdump_exit(void);
void nvdumper_crash_setup_regs(void);
void nvdumper_print_data(void);

#endif
