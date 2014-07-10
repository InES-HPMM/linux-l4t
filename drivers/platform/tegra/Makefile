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

obj-y += mc/
obj-y += nvdumper/
obj-$(CONFIG_PM_SLEEP) += pm-irq.o
obj-y += pmc.o
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

obj-y += common.o
obj-y += devices.o
obj-y += io.o
obj-y += clock.o
obj-y += tegra_clocks_pll.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_clocks.o

# FIXME: Bug 1211919:
# Remove CFLAGS_tegra21_clocks.o flag when tegra21_clocks.c becomes
# stabilized.
CFLAGS_tegra21_clocks.o = -Werror -Wno-unused-function -Wno-unused-variable

obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra21_clocks.o

obj-$(CONFIG_PM_SLEEP) += tegra-wakeups.o

obj-y += pm-t3.o
obj-$(CONFIG_DEBUG_FS) += clocks_stats.o
obj-y += tegra_core_volt_cap.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += wakeups-t12x.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += wakeups-t21x.o


obj-y += kfuse.o
obj-y += apbio.o
obj-$(CONFIG_FIQ) += fiq.o

obj-y += powergate.o
obj-y += powergate-ops-txx.o
obj-y += powergate-ops-t1xx.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += powergate-t12x.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += powergate-t21x.o

obj-y += dvfs.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra13_dvfs.o

# FIXME: Bug 1211919:
# Remove CFLAGS_tegra21_dvfs.o flag when tegra21_dvfs.c becomes
# stabilized.
CFLAGS_tegra21_dvfs.o = -Werror -Wno-unused-function -Wno-unused-variable

obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra21_dvfs.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_AUTO_HOTPLUG),y)
ifeq ($(CONFIG_TEGRA_CPUQUIET),y)
obj-$(CONFIG_ARCH_TEGRA_13x_SOC)         += cpuquiet.o
endif
endif

obj-y += latency_allowance.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12x_la.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += tegra12x_la.o

obj-$(CONFIG_ARCH_TEGRA_13x_SOC)        += tegra13_speedo.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += tegra21_speedo.o

obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra3_actmon.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra3_actmon.o

obj-y += tegra_emc.o

obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_emc.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra_emc_dt_parse.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra_emc_dt_parse.o
obj-y += tegra_emc_therm.o

obj-$(CONFIG_TEGRA_MC_DOMAINS) += pm_domains.o
obj-y += bond_out.o
obj-y += flowctrl.o
obj-$(CONFIG_DEBUG_ICEDCC) += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL) += sysfs-cluster.o
obj-$(CONFIG_ARCH_TEGRA_HAS_CL_DVFS) += tegra_cl_dvfs.o

ifeq ($(CONFIG_TEGRA_SOCTHERM),y)
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra_soctherm.o
endif
obj-$(CONFIG_TEGRA_USE_SIMON)    += tegra_simon.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_edp.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra13_edp.o

obj-y += board-common.o

obj-y += board-touch-raydium_spi.o

obj-y += board-panel.o

obj-$(CONFIG_ARCH_TEGRA_21x_SOC)      += cluster-control.o

obj-${CONFIG_TEGRA_ISOMGR}              += isomgr.o
obj-$(CONFIG_TEGRA_USE_NCT)             += nct.o nct_sysfs.o

obj-$(CONFIG_TEGRA_EDP_LIMITS)          += edp.o

else # --- CONFIG_ARM64 --- #

obj-y += dummy.o # generate a built-in.o

endif

obj-y					+= hier_ictlr/
