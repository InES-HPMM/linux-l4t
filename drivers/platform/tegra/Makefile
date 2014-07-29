#
# Makefile for linux/drivers/platform/tegra
# Tegra Platform-Specific Drivers
#
subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/platform/tegra/include

obj-$(CONFIG_USB_SUPPORT)               += tegra_usb_pmc.o
obj-$(CONFIG_TEGRA_USB_SHARED_PAD)      += tegra_usb_pad_ctrl.o
obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o
obj-$(CONFIG_TEGRA_IOMMU_SMMU)          += iommu.o
obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG)    += tegra_bootloader_debug.o

obj-$(CONFIG_TEGRA_CAMERA_MIPI_CAL)     += mipi-cal.o
obj-$(CONFIG_TEGRA_NVADSP)		+= nvadsp/
obj-$(CONFIG_TEGRA_MC) += mc/

ifeq ($(CONFIG_TEGRA_PTM),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)         += tegra_ptm_t124.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)         += tegra_ptm_t210.o
endif

obj-y += nvdumper/
obj-$(CONFIG_PM_SLEEP) += pm-irq.o
obj-$(CONFIG_TEGRA_PMC) += pmc.o
obj-y += pm.o

obj-$(CONFIG_TEGRA_MC_DOMAINS) += pm_domains.o

obj-$(CONFIG_TEGRA_WAKEUP_MONITOR)      += tegra_wakeup_monitor.o

# tempporarily support soctherm platform driver for T12x with include files
# from arch/arm/tegra area.
ifeq ($(CONFIG_TEGRA_SOCTHERM),y)
ccflags-y += -I$(srctree)/arch/arm/mach-tegra
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra_soctherm.o
endif

ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
ccflags-y += -I$(srctree)/arch/arm/mach-tegra
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra_throttle.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += tegra_throttle.o
endif

ifneq ($(CONFIG_ARM64),)

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
asflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra


obj-y += ahb.o

# FIXME: Bug 1211919:
# Remove CFLAGS_common.o flag for common.o when common.c
# becomes stabilized for t210.

#ifdef CONFIG_ARCH_TEGRA_21x_SOC
CFLAGS_common.o = -Werror -Wno-unused-function
#endif

obj-$(CONFIG_TEGRA_COMMON) += common.o
obj-$(CONFIG_TEGRA_DEVICES) += devices.o
obj-y += io.o
obj-$(CONFIG_TEGRA_CLK_FRAMEWORK) += clock.o
obj-$(CONFIG_TEGRA_CLK_FRAMEWORK) += tegra_clocks_pll.o
ifeq ($(CONFIG_TEGRA_CLK_FRAMEWORK),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_clocks.o

# FIXME: Bug 1211919:
# Remove CFLAGS_tegra21_clocks.o flag when tegra21_clocks.c becomes
# stabilized.
CFLAGS_tegra21_clocks.o = -Werror -Wno-unused-function -Wno-unused-variable

obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra21_clocks.o
endif
obj-$(CONFIG_COMMON_CLK) += common_clock.o

obj-$(CONFIG_PM_SLEEP) += tegra-wakeups.o

obj-$(CONFIG_TEGRA3_PM) += pm-t3.o
ifeq ($(CONFIG_TEGRA_CLK_FRAMEWORK),y)
obj-$(CONFIG_DEBUG_FS) += clocks_stats.o
endif
obj-$(CONFIG_TEGRA_CORE_VOLT_CAP) += tegra_core_volt_cap.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += wakeups-t12x.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += wakeups-t21x.o


obj-$(CONFIG_TEGRA_KFUSE) += kfuse.o
obj-y += apbio.o
obj-$(CONFIG_FIQ) += fiq.o

obj-$(CONFIG_TEGRA_POWERGATE) += powergate.o
obj-$(CONFIG_TEGRA_POWERGATE) += powergate-ops-txx.o
obj-$(CONFIG_TEGRA_POWERGATE) += powergate-ops-t1xx.o
ifeq ($(CONFIG_TEGRA_POWERGATE),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += powergate-t12x.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += powergate-t21x.o
endif

obj-$(CONFIG_TEGRA_CLK_FRAMEWORK) += dvfs.o
ifeq ($(CONFIG_TEGRA_CLK_FRAMEWORK),y)
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra13_dvfs.o

# FIXME: Bug 1211919:
# Remove CFLAGS_tegra21_dvfs.o flag when tegra21_dvfs.c becomes
# stabilized.
CFLAGS_tegra21_dvfs.o = -Werror -Wno-unused-function -Wno-unused-variable

obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra21_dvfs.o
endif
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o

obj-$(CONFIG_ARCH_TEGRA_13x_SOC)        += tegra13_speedo.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += tegra21_speedo.o

obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra3_actmon.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra3_actmon.o

ifeq ($(CONFIG_TEGRA_MC),y)
obj-y += tegra_emc.o

obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_emc.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra_emc_dt_parse.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra_emc_dt_parse.o
obj-y += tegra_emc_therm.o
endif

obj-$(CONFIG_TEGRA_MC_DOMAINS) += pm_domains.o
obj-y += bond_out.o
obj-y += flowctrl.o
obj-$(CONFIG_DEBUG_ICEDCC) += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL) += sysfs-cluster.o
obj-$(CONFIG_TEGRA_HMP_CLUSTER_CONTROL) += sysfs-cluster.o
ifeq ($(CONFIG_TEGRA_CLK_FRAMEWORK),y)
obj-$(CONFIG_ARCH_TEGRA_HAS_CL_DVFS) += tegra_cl_dvfs.o
endif

ifeq ($(CONFIG_TEGRA_SOCTHERM),y)
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra_soctherm.o
endif
obj-$(CONFIG_TEGRA_USE_SIMON)    += tegra_simon.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_edp.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra13_edp.o

obj-$(CONFIG_TEGRA_BOARD_COMMON) += board-common.o

obj-$(CONFIG_TEGRA_BOARD_COMMON) += board-touch-raydium_spi.o

obj-$(CONFIG_TEGRA_BOARD_COMMON) += board-panel.o

obj-$(CONFIG_TEGRA_HMP_CLUSTER_CONTROL)      += cluster-control.o

obj-${CONFIG_TEGRA_ISOMGR}              += isomgr.o
obj-$(CONFIG_TEGRA_USE_NCT)             += nct.o nct_sysfs.o

obj-$(CONFIG_TEGRA_EDP_LIMITS)          += edp.o
obj-$(CONFIG_TEGRA_GPU_EDP)             += gpu-edp.o

else # --- CONFIG_ARM64 --- #

obj-y += dummy.o # generate a built-in.o

endif

obj-y					+= asim.o
obj-y					+= hier_ictlr/

obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o
