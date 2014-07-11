GCOV_PROFILE := y

asflags-y				+= -march=armv7-a
subdir-ccflags-y := -Werror

obj-y                                   += ahb.o
obj-y                                   += common.o
obj-y                                   += devices.o
obj-y                                   += board-info.o
obj-y                                   += io.o
obj-y                                   += clock.o
obj-$(CONFIG_HAVE_ARM_TWD)              += cpu_timer_twd.o

obj-$(CONFIG_PM_SLEEP)                  += tegra-wakeups.o

obj-$(CONFIG_TEGRA_CPU_VOLT_CAP)	+= tegra_volt_cap.o
ifeq ($(CONFIG_ARCH_TEGRA_2x_SOC),y)
obj-y                                   += common-t2.o
obj-y                                   += pm-t2.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= sleep-t20.o
obj-$(CONFIG_PM_SLEEP)                  += wakeups-t2.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_PM_SLEEP)                  += cpuidle-t2.o
endif
else
obj-y                                   += pm-t3.o
obj-y                                   += sleep-t30.o
obj-$(CONFIG_DEBUG_FS)                  += clocks_stats.o
obj-y                                   += tegra_core_volt_cap.o
obj-$(CONFIG_TEGRA_USE_SIMON)           += tegra_simon.o
ifeq ($(CONFIG_ARCH_TEGRA_12x_SOC),y)
obj-$(CONFIG_PM_SLEEP)			+= wakeups-t12x.o
endif

ifeq ($(CONFIG_CPU_IDLE),y)
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += cpuidle-t11x.o
endif
endif
endif

obj-y                                   += delay.o
obj-y                                   += pm.o
obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o

obj-y                                   += sleep.o
obj-$(CONFIG_TEGRA_USE_NCT)             += nct.o nct_sysfs.o

plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_sleep.o :=-Wa,-march=armv7-a$(plus_sec)

obj-y                                   += kfuse.o
obj-$(CONFIG_TEGRA_PTM)                 += tegra_ptm.o

obj-y                                   += powergate.o
obj-y                                   += powergate-ops-txx.o
obj-y                                   += powergate-ops-t1xx.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += powergate-t20.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += powergate-t30.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += powergate-t12x.o

obj-y                                   += apbio.o
obj-$(CONFIG_TEGRA_ARB_SEMAPHORE)       += arb_sema.o
obj-y                                   += dvfs.o
ifeq ($(CONFIG_ARCH_TEGRA_13x_SOC),y)
obj-y                                   += tegra13_dvfs.o
else
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12_dvfs.o
endif
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra_emc_therm.o

obj-y                                   += latency_allowance.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12x_la.o
obj-$(CONFIG_TEGRA_EDP_LIMITS)          += edp.o
obj-$(CONFIG_TEGRA_CORE_EDP_LIMITS)     += edp_core.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12_edp.o
ifeq ($(CONFIG_ARCH_TEGRA_13x_SOC),y)
obj-y                                   += tegra13_speedo.o
else
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12_speedo.o
endif
obj-$(CONFIG_TEGRA_ACTMON)              += tegra3_actmon.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_emc.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_emc.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12_emc.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra_emc_dt_parse.o
obj-y					+= tegra_emc.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= tegra20_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= tegra30_speedo.o
obj-$(CONFIG_HOTPLUG_CPU)               += hotplug.o
obj-y                                   += flowctrl.o
obj-$(CONFIG_CPU_IDLE)                  += cpuidle.o
obj-$(CONFIG_CPU_IDLE)                  += sleep.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12_clocks.o
obj-$(CONFIG_SMP)                       += platsmp.o
obj-y                                   += reset.o
obj-y                                   += headsmp.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_AUTO_HOTPLUG),y)
ifeq ($(CONFIG_TEGRA_CPUQUIET),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += cpuquiet.o
else
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpu-tegra3.o
endif
endif
ifeq ($(CONFIG_TEGRA_SOCTHERM),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra11_soctherm.o
endif
ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra3_throttle.o
endif
obj-$(CONFIG_DEBUG_ICEDCC)              += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL)     += sysfs-cluster.o
obj-$(CONFIG_TEGRA_DYNAMIC_PWRDET)      += powerdetect.o
obj-$(CONFIG_ARCH_TEGRA_HAS_CL_DVFS)    += tegra_cl_dvfs.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += board-dt-tegra20.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += board-dt-tegra30.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-harmony-pcie.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-paz00.o

obj-y					+= board-common.o

obj-y                                   += board-touch-raydium_spi.o
obj-y                                   += board-touch-synaptics-spi.o
obj-y                                   += board-touch-maxim_sti-spi.o

obj-y                                   += board-panel.o
obj-y                                   += panel-p-wuxga-10-1.o
obj-y                                   += panel-lgd-wxga-7-0.o
obj-y                                   += panel-s-wqxga-10-1.o
obj-y                                   += panel-a-1200-800-8-0.o
obj-y                                   += panel-a-1200-1920-8-0.o
obj-y					+= panel-a-edp-1080p-14-0.o
obj-y					+= panel-i-edp-1080p-11-6.o

obj-${CONFIG_MACH_ARDBEG}               += board-ardbeg.o
obj-${CONFIG_MACH_ARDBEG}               += board-ardbeg-sdhci.o
obj-${CONFIG_MACH_ARDBEG}               += board-ardbeg-sensors.o
obj-${CONFIG_MACH_ARDBEG}               += board-ardbeg-panel.o
obj-${CONFIG_MACH_ARDBEG}               += board-ardbeg-memory.o
obj-${CONFIG_MACH_ARDBEG}               += board-ardbeg-power.o
obj-${CONFIG_MACH_ARDBEG}               += board-tn8-power.o
obj-${CONFIG_MACH_ARDBEG}               += board-norrin-power.o
obj-${CONFIG_MACH_ARDBEG}               += board-norrin-memory.o
obj-${CONFIG_MACH_ARDBEG}               += panel-a-1080p-11-6.o
obj-${CONFIG_MACH_ARDBEG}               += panel-s-wqxga-10-1.o
obj-${CONFIG_MACH_ARDBEG}               += panel-s-edp-uhdtv-15-6.o
obj-${CONFIG_MACH_ARDBEG}               += panel-c-lvds-1366-14.o
obj-${CONFIG_MACH_ARDBEG}               += panel-s-edp-wqxgap-15-6.o
ifeq ($(CONFIG_MACH_ARDBEG),y)
obj-${CONFIG_SYSEDP_FRAMEWORK}          += board-tn8-sysedp.o
obj-${CONFIG_SYSEDP_FRAMEWORK}          += board-ardbeg-sysedp.o
endif

obj-${CONFIG_MACH_LOKI}               	+= board-loki.o
obj-${CONFIG_MACH_LOKI}               	+= board-loki-kbc.o
obj-${CONFIG_MACH_LOKI}               	+= board-loki-sensors.o
obj-${CONFIG_MACH_LOKI}               	+= board-loki-panel.o
obj-${CONFIG_MACH_LOKI}               	+= board-loki-memory.o
obj-${CONFIG_MACH_LOKI}               	+= board-loki-power.o
obj-${CONFIG_MACH_LOKI}               	+= board-tn8-power.o
obj-${CONFIG_MACH_LOKI}               	+= board-loki-sdhci.o
obj-${CONFIG_MACH_LOKI}               	+= panel-a-1080p-11-6.o
obj-${CONFIG_MACH_LOKI}               	+= panel-s-wqxga-10-1.o
obj-${CONFIG_MACH_LOKI}               	+= panel-l-720p-5-loki.o
obj-${CONFIG_MACH_LOKI}               	+= panel-j-1440-810-5-8.o
obj-${CONFIG_MACH_LOKI}               	+= panel-j-720p-5.o

obj-${CONFIG_MACH_LAGUNA}               += board-ardbeg.o
obj-${CONFIG_MACH_LAGUNA}               += board-ardbeg-sdhci.o
obj-${CONFIG_MACH_LAGUNA}               += board-ardbeg-sensors.o
obj-${CONFIG_MACH_LAGUNA}               += board-ardbeg-panel.o
obj-${CONFIG_MACH_LAGUNA}               += board-laguna-power.o
obj-${CONFIG_MACH_LAGUNA}               += panel-a-1080p-11-6.o
obj-${CONFIG_MACH_LAGUNA}               += panel-s-wqxga-10-1.o
obj-${CONFIG_MACH_LAGUNA}               += panel-a-1080p-14-0.o

obj-${CONFIG_MACH_VCM30_T124}           += vcm30_t124.o
obj-${CONFIG_MACH_VCM30_T124}           += board-vcm30_t124.o
ifdef CONFIG_TEGRA_WIFI_ENABLE
obj-${CONFIG_MACH_VCM30_T124}           += board-vcm30_t124-sdhci.o
endif
obj-${CONFIG_MACH_VCM30_T124}           += board-vcm30_t124-power.o
obj-${CONFIG_MACH_VCM30_T124}           += board-vcm30_t124-audio.o

obj-${CONFIG_MACH_VCM30_T124}           += therm-monitor.o

obj-${CONFIG_MACH_P2360}                += board-p2360.o
obj-${CONFIG_MACH_P2360}                += board-p2360-panel.o
obj-${CONFIG_MACH_P2360}                += board-p2360-power.o
obj-${CONFIG_MACH_P2360}                += therm-monitor.o

obj-${CONFIG_TEGRA_BB_XMM_POWER}        += baseband-xmm-power.o
obj-${CONFIG_TEGRA_BB_XMM_POWER2}       += baseband-xmm-power2.o

obj-${CONFIG_TEGRA_BASEBAND}            += tegra_bb.o
obj-$(CONFIG_TEGRA_BBC_PROXY)           += tegra_bbc_proxy.o
obj-$(CONFIG_TEGRA_BBC_THERMAL)         += tegra_bbc_thermal.o

obj-${CONFIG_TEGRA_ISOMGR}              += isomgr.o

obj-${CONFIG_TEGRA_NVDUMPER}            += nvdumper.o
