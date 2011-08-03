asflags-y				+= -march=armv7-a

obj-y                                   += board-pinmux.o
obj-y                                   += ahb.o
obj-y                                   += common.o
obj-y                                   += devices.o
obj-y                                   += board-info.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += common-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += common-t3.o
obj-y                                   += io.o
obj-y                                   += irq.o
obj-y                                   += clock.o
obj-y                                   += timer.o
obj-y                                   += timerinfo.o
ifeq ($(CONFIG_ARCH_TEGRA_2x_SOC),y)
obj-y                                   += timer-t2.o
else
ifeq ($(CONFIG_ARCH_TEGRA_3x_SOC),y)
obj-y                                   += timer-t3.o
else
obj-y                                   += tegra11_clocks.o
endif
endif
obj-y                                   += pinmux.o
obj-y                                   += delay.o
obj-y                                   += pm.o
obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o
obj-$(CONFIG_PM_SLEEP)                  += pm-irq.o
obj-y                                   += gic.o
obj-y                                   += tegra_fuse.o
obj-y                                   += kfuse.o
obj-y                                   += csi.o
obj-$(CONFIG_TEGRA_SILICON_PLATFORM)    += tegra_odm_fuses.o
obj-y                                   += pmc.o
obj-y                                   += flowctrl.o
obj-y                                   += powergate.o
obj-y                                   += apbio.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-y                                   += cpuidle.o
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += cpuidle-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpuidle-t3.o
endif
endif
ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_throttle.o
endif
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_thermal.o
obj-y                                   += sleep.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += sleep-t2.o
obj-y                                   += mc.o
obj-$(CONFIG_USB_SUPPORT)               += usb_phy.o
obj-$(CONFIG_USB_SUPPORT)               += tegra3_usb_phy.o
obj-$(CONFIG_USB_SUPPORT)               += tegra2_usb_phy.o
obj-$(CONFIG_FIQ)                       += fiq.o
obj-$(CONFIG_TEGRA_ARB_SEMAPHORE)	+= arb_sema.o
ifeq ($(CONFIG_TEGRA_SILICON_PLATFORM),y)
obj-y                                   += dvfs.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += latency_allowance.o
obj-$(CONFIG_TEGRA_EDP_LIMITS)          += edp.o
endif
ifeq ($(CONFIG_TEGRA_SILICON_PLATFORM),y)
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += tegra11_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_actmon.o
endif
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_emc.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_emc.o
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += tegra11_emc.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= sleep-t20.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= wakeups-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= wakeups-t3.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= pm-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += pm-t3.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= pinmux-tegra20-tables.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= pinmux-tegra30-tables.o
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += pinmux-t11-tables.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= tegra20_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= tegra30_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= sleep-t30.o
obj-$(CONFIG_SMP)			+= platsmp.o
obj-y                                   += reset.o
obj-y                                   += headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)               += hotplug.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_AUTO_HOTPLUG),y)
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpu-tegra3.o
endif
obj-$(CONFIG_TEGRA_PCI)			+= pcie.o
obj-$(CONFIG_DEBUG_ICEDCC)              += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL)     += sysfs-cluster.o
obj-$(CONFIG_SENSORS_TEGRA_TSENSOR)     += tegra3_tsensor.o
obj-$(CONFIG_TEGRA_DYNAMIC_PWRDET)      += powerdetect.o
obj-$(CONFIG_TEGRA_USB_MODEM_POWER)     += tegra_usb_modem_power.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-dt-tegra20.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= board-dt-tegra30.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-harmony-pcie.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-paz00.o

obj-m += baseband-xmm-power.o
obj-m += baseband-xmm-power2.o
