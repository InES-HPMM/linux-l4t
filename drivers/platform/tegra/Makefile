#
# Makefile for linux/drivers/platform/tegra
# Tegra Platform-Specific Drivers
#
obj-$(CONFIG_USB_SUPPORT)               += tegra_usb_pmc.o
obj-$(CONFIG_TEGRA_USB_SHARED_PAD)      += tegra_usb_pad_ctrl.o
obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o

ifneq ($(CONFIG_ARCH_TEGRA_2x_SOC),y)
ifneq ($(CONFIG_ARCH_TEGRA_3x_SOC),y)
obj-y                                   += mipi-cal.o
endif
endif

obj-y += mc/

ifneq ($(CONFIG_ARM64),)

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
asflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra


obj-y += ahb.o
obj-y += common.o
obj-y += devices.o
obj-y += io.o
obj-y += clock.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_clocks.o
obj-y += timerinfo.o

obj-$(CONFIG_PM_SLEEP) += tegra-wakeups.o

obj-y += pm-t3.o
obj-$(CONFIG_DEBUG_FS) += clocks_stats.o
obj-y += tegra_core_volt_cap.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += wakeups-t12x.o

obj-$(CONFIG_PM_SLEEP) += pm-irq.o

obj-y += kfuse.o
obj-y += apbio.o
obj-$(CONFIG_FIQ) += fiq.o

obj-y += powergate.o
obj-y += powergate-ops-txx.o
obj-y += powergate-ops-t1xx.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += powergate-t12x.o

obj-$(CONFIG_PM_GENERIC_DOMAINS) += pm_domains.o

obj-y += dvfs.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra13_dvfs.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_AUTO_HOTPLUG),y)
ifeq ($(CONFIG_TEGRA_CPUQUIET),y)
obj-$(CONFIG_ARCH_TEGRA_13x_SOC)         += cpuquiet.o
endif
endif
ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
obj-$(CONFIG_ARCH_TEGRA_13x_SOC)        += tegra3_throttle.o
endif

obj-y += latency_allowance.o
obj-y += tegra3_la.o
obj-y += tegra11x_la.o
obj-y += tegra14x_la.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12x_la.o

obj-$(CONFIG_ARCH_TEGRA_13x_SOC)        += tegra13_speedo.o

obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra3_actmon.o

obj-y += tegra_emc.o

obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_emc.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra_emc_dt_parse.o

obj-y += pmc.o
obj-y += flowctrl.o
obj-y += reset.o
obj-$(CONFIG_DEBUG_ICEDCC) += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL) += sysfs-cluster.o
obj-$(CONFIG_ARCH_TEGRA_HAS_CL_DVFS) += tegra_cl_dvfs.o

obj-$(CONFIG_TEGRA_PCI) += pcie.o

ifeq ($(CONFIG_TEGRA_SOCTHERM),y)
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra11_soctherm.o
endif
obj-$(CONFIG_TEGRA_USE_SIMON)    += tegra_simon.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC) += tegra12_edp.o

obj-y += board-common.o

obj-y += board-touch-raydium_spi.o

obj-y += board-panel.o

obj-${CONFIG_TEGRA_ISOMGR}              += isomgr.o

else # --- CONFIG_ARM64 --- #

obj-y += dummy.o # generate a built-in.o

endif
