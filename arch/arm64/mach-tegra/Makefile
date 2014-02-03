GCOV_PROFILE := y

# Reference the mach-tegra from arch/arm for now
ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
asflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra

# Assemblies
obj-y += sleep.o
obj-y += reset-handler.o

obj-$(CONFIG_SMP) += platsmp.o

# Temp board files (should be gone when we go DT)
obj-${CONFIG_MACH_EXUMA}              += board-exuma.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-panel.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-memory.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-power.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-powermon.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-sdhci.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-sensors.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-sysedp.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-sata.o
obj-${CONFIG_MACH_EXUMA}              += board-norrin-power.o
obj-${CONFIG_MACH_EXUMA}              += board-norrin-kbc.o
obj-${CONFIG_MACH_EXUMA}              += board-tn8-power.o

obj-${CONFIG_MACH_T132REF}              += board-t132ref.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-laguna-power.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-laguna-powermon.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-memory.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-sensors.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-norrin-kbc.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-norrin-power.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-panel.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-power.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-powermon.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-sata.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-sdhci.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-tn8-power.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-tn8-p1761-powermon.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-sysedp.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-tn8-sysedp.o
obj-${CONFIG_MACH_T132REF}              += panel-a-1080p-11-6.o
obj-${CONFIG_MACH_T132REF}              += panel-a-edp-1080p-14-0.o
obj-${CONFIG_MACH_T132REF}              += panel-a-1080p-14-0.o
obj-${CONFIG_MACH_T132REF}              += panel-a-1200-800-8-0.o
obj-${CONFIG_MACH_T132REF}              += panel-a-1200-1920-7-0.o
obj-${CONFIG_MACH_T132REF}              += panel-c-lvds-1366-14.o
obj-${CONFIG_MACH_T132REF}              += panel-lgd-wxga-7-0.o
obj-${CONFIG_MACH_T132REF}              += panel-p-wuxga-10-1.o
obj-${CONFIG_MACH_T132REF}              += panel-s-edp-uhdtv-15-6.o
obj-${CONFIG_MACH_T132REF}              += panel-s-edp-wqxgap-15-6.o
obj-${CONFIG_MACH_T132REF}              += panel-s-wqxga-10-1.o
obj-${CONFIG_MACH_T132REF}              += panel-i-edp-1080p-11-6.o


obj-y                                   += board-touch-maxim_sti-spi.o


ifeq ($(CONFIG_HOTPLUG_CPU),y)
obj-$(CONFIG_DENVER_CPU)              += hotplug-denver.o
endif

obj-$(CONFIG_DENVER_CPU)              += denver-knobs.o

ifeq ($(CONFIG_HOTPLUG_CPU),y)
obj-$(CONFIG_DENVER_CPU)              += hotplug-denver.o
endif

ifeq ($(CONFIG_PM_SLEEP),y)
obj-y                                 += pm.o
obj-y                                 += sleep.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC)      += pm-tegra132.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC)      += sleep-tegra132.o
endif

# To override the "-march=armv7-xxx" in arm/mach-tegra/Makefile
AFLAGS_sleep.o :=
