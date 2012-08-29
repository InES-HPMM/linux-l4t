GCOV_PROFILE := y

asflags-y				+= -march=armv7-a
subdir-ccflags-y := -Werror

obj-y                                   += board-pinmux.o
obj-y                                   += ahb.o
obj-y                                   += common.o
obj-y                                   += devices.o
obj-y                                   += board-info.o
obj-y                                   += io.o
obj-y                                   += irq.o
obj-y                                   += clock.o
obj-y                                   += timer.o
obj-y                                   += timerinfo.o

ifeq ($(CONFIG_ARCH_TEGRA_2x_SOC),y)
obj-y                                   += common-t2.o
obj-y                                   += pm-t2.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= sleep-t20.o
obj-y                                   += timer-t2.o
obj-y                                   += wakeups-t2.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_PM_SLEEP)                  += cpuidle-t2.o
endif
else
obj-y                                   += common-t3.o
obj-y                                   += pm-t3.o
obj-y                                   += sleep-t30.o
obj-$(CONFIG_DEBUG_FS)                  += clocks_stats.o
obj-y                                   += timer-t3.o
ifeq ($(CONFIG_ARCH_TEGRA_3x_SOC),y)
obj-y                                   += wakeups-t3.o
else
obj-y					+= wakeups-t11x.o
endif
ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_PM_SLEEP)                  += cpuidle-t3.o
endif
endif

obj-y                                   += pinmux.o
obj-y                                   += delay.o
obj-y                                   += pm.o
obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o
obj-$(CONFIG_PM_SLEEP)                  += pm-irq.o
obj-y                                   += gic.o

obj-y                                   += sleep.o

plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_sleep.o :=-Wa,-march=armv7-a$(plus_sec)

obj-y                                   += tegra_fuse.o
obj-y                                   += kfuse.o
obj-y                                   += csi.o
obj-y                                   += tegra_odm_fuses.o
obj-y                                   += powergate.o
obj-y                                   += apbio.o
obj-y                                   += mc.o
obj-$(CONFIG_USB_SUPPORT)               += tegra3_usb_phy.o
obj-$(CONFIG_USB_SUPPORT)               += tegra2_usb_phy.o
obj-$(CONFIG_USB_SUPPORT)               += tegra11x_usb_phy.o
obj-$(CONFIG_FIQ)                       += fiq.o
obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o
obj-$(CONFIG_TEGRA_ARB_SEMAPHORE)       += arb_sema.o
obj-y                                   += dvfs.o
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += tegra11_dvfs.o
ifeq ($(CONFIG_TEGRA_SILICON_PLATFORM),y)
obj-y                                   += latency_allowance.o
obj-$(CONFIG_TEGRA_EDP_LIMITS)          += edp.o
endif
ifeq ($(CONFIG_TEGRA_SILICON_PLATFORM),y)
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += tegra11_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_actmon.o
endif
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += tegra3_actmon.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_emc.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_emc.o
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += tegra11_emc.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += pinmux-tegra20-tables.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += pinmux-tegra30-tables.o
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += pinmux-t11-tables.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= tegra20_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= tegra30_speedo.o
obj-$(CONFIG_HOTPLUG_CPU)               += hotplug.o
obj-y					+= pmc.o
obj-y					+= flowctrl.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_CPU_IDLE)			+= sleep.o
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += tegra11_clocks.o
obj-$(CONFIG_SMP)                       += platsmp.o
obj-y                                   += reset.o
obj-y                                   += headsmp.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_AUTO_HOTPLUG),y)
ifeq ($(CONFIG_TEGRA_CPUQUIET),y)
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpuquiet.o
else
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpu-tegra3.o
endif
endif
obj-$(CONFIG_TEGRA_PCI)                 += pcie.o
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += tegra11_soctherm.o
ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_throttle.o
endif
obj-$(CONFIG_TEGRA_THERMAL)             += tegra3_thermal.o
obj-$(CONFIG_DEBUG_ICEDCC)              += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL)     += sysfs-cluster.o
obj-$(CONFIG_SENSORS_TEGRA_TSENSOR)     += tegra3_tsensor.o
obj-$(CONFIG_TEGRA_DYNAMIC_PWRDET)      += powerdetect.o
obj-$(CONFIG_TEGRA_USB_MODEM_POWER)     += tegra_usb_modem_power.o
obj-$(CONFIG_ARCH_TEGRA_HAS_CL_DVFS)    += tegra_cl_dvfs.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += board-dt-tegra20.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += board-dt-tegra30.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-harmony-pcie.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-paz00.o

obj-${CONFIG_MACH_CURACAO}              += board-curacao.o
obj-${CONFIG_MACH_CURACAO}              += board-curacao-panel.o
obj-${CONFIG_MACH_CURACAO}              += board-curacao-pinmux.o
obj-${CONFIG_MACH_CURACAO}              += board-curacao-power.o
obj-${CONFIG_MACH_CURACAO}              += board-curacao-sdhci.o
obj-${CONFIG_MACH_CURACAO}              += board-curacao-sensors.o
obj-${CONFIG_MACH_CURACAO}              += board-curacao-memory.o

obj-${CONFIG_MACH_KAI}                  += board-touch-kai-synaptics-spi.o
obj-y                                   += board-touch-raydium_spi.o

obj-${CONFIG_MACH_DALMORE}              += board-dalmore.o
obj-${CONFIG_MACH_DALMORE}              += board-dalmore-memory.o
obj-${CONFIG_MACH_DALMORE}              += board-dalmore-pinmux.o
obj-${CONFIG_MACH_DALMORE}              += board-dalmore-power.o
obj-${CONFIG_MACH_DALMORE}              += board-dalmore-sdhci.o
obj-${CONFIG_MACH_DALMORE}              += board-dalmore-panel.o
obj-${CONFIG_MACH_DALMORE}              += board-dalmore-kbc.o

obj-${CONFIG_MACH_TEGRA_PLUTO}          += board-pluto.o
obj-${CONFIG_MACH_TEGRA_PLUTO}          += board-pluto-memory.o
obj-${CONFIG_MACH_TEGRA_PLUTO}          += board-pluto-pinmux.o
obj-${CONFIG_MACH_TEGRA_PLUTO}          += board-pluto-power.o
obj-${CONFIG_MACH_TEGRA_PLUTO}          += board-pluto-sdhci.o
obj-${CONFIG_MACH_TEGRA_PLUTO}          += board-pluto-panel.o
obj-${CONFIG_MACH_TEGRA_PLUTO}          += board-pluto-kbc.o

obj-${CONFIG_TEGRA_BB_XMM_POWER}        += baseband-xmm-power.o
obj-${CONFIG_TEGRA_BB_XMM_POWER2}       += baseband-xmm-power2.o

