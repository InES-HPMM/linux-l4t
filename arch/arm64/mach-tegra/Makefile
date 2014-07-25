GCOV_PROFILE := y

# Reference the mach-tegra from arch/arm for now
ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
asflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
subdir-ccflags-y := -Werror

# Assemblies
obj-y += reset-handler.o trampoline.o

obj-$(CONFIG_SMP) += platsmp.o

# Temp board files (should be gone when we go DT)
obj-${CONFIG_MACH_EXUMA}              += board-exuma.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-panel.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-memory.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-power.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-sdhci.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-sensors.o
obj-${CONFIG_MACH_EXUMA}              += board-ardbeg64-sysedp.o
obj-${CONFIG_MACH_EXUMA}              += board-t132ref-norrin-power.o
obj-${CONFIG_MACH_EXUMA}              += board-t132ref-norrin-memory.o
obj-${CONFIG_MACH_EXUMA}              += board-t132ref-tn8-power.o

obj-y					+= panel-lgd-wxga-7-0.o
obj-y					+= panel-p-wuxga-10-1.o
obj-y					+= panel-s-wqxga-10-1.o
obj-y					+= panel-a-1200-800-8-0.o
obj-y					+= panel-a-1200-1920-8-0.o
obj-y					+= panel-a-edp-1080p-14-0.o
obj-y					+= panel-i-edp-1080p-11-6.o
obj-y					+= panel-c-lvds-1366-14.o
obj-y					+= panel-a-1080p-14-0.o

obj-${CONFIG_MACH_LOKI}               += board-loki-power.o
obj-${CONFIG_MACH_LOKI}               += board-loki-memory.o
obj-${CONFIG_MACH_LOKI}               += board-loki-sdhci.o
obj-${CONFIG_MACH_LOKI}               += board-loki-sensors.o
obj-${CONFIG_MACH_LOKI}               += board-loki-kbc.o
obj-${CONFIG_MACH_LOKI}               += board-loki-panel.o
obj-${CONFIG_MACH_LOKI}               += panel-l-720p-5-loki.o
obj-${CONFIG_MACH_LOKI}               += panel-j-1440-810-5-8.o
obj-${CONFIG_MACH_LOKI}               += panel-j-720p-5.o


obj-${CONFIG_MACH_T132REF}              += board-t132ref.o
obj-${CONFIG_MACH_T132REF}              += board-t210ref.o
obj-${CONFIG_MACH_T132REF}              += board-t210ref-thermal.o
obj-${CONFIG_MACH_T132REF}              += board-t210ref-sdhci.o
obj-${CONFIG_MACH_T132REF}              += board-t210ref-panel.o
obj-${CONFIG_MACH_T132REF}              += board-t210ref-camera.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-laguna-power.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-memory.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-sensors.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-norrin-power.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-norrin-memory.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-panel.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-power.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-sdhci.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-tn8-power.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-sysedp.o
obj-${CONFIG_MACH_T132REF}              += board-t132ref-tn8-sysedp.o
obj-${CONFIG_MACH_T132REF}              += panel-a-1080p-11-6.o
obj-${CONFIG_MACH_T132REF}              += panel-s-edp-uhdtv-15-6.o
obj-${CONFIG_MACH_T132REF}              += panel-s-edp-wqxgap-15-6.o

obj-y                                   += board-touch-maxim_sti-spi.o

ifeq ($(CONFIG_HOTPLUG_CPU),y)
obj-$(CONFIG_DENVER_CPU)              += hotplug-denver.o
endif

obj-$(CONFIG_DENVER_CPU)              += denver-knobs.o
obj-$(CONFIG_DENVER_CPU)              += denver-hardwood.o

obj-${CONFIG_ARCH_TEGRA_21x_SOC}      += board-t210-panel.o
obj-${CONFIG_ARCH_TEGRA_21x_SOC}      += panel-c-edp-1366-768-14-0.o
obj-${CONFIG_ARCH_TEGRA_21x_SOC}      += board-dt-t210.o
obj-${CONFIG_ARCH_TEGRA_21x_SOC}      += tegra21_memory.o
obj-${CONFIG_ARCH_TEGRA_21x_SOC}      += tegra21_init.o

ifeq ($(CONFIG_PM_SLEEP),y)
obj-y                                 += pm.o
obj-y                                 += sleep.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC)      += pm-tegra132.o
endif

obj-$(CONFIG_TEGRA_DYNAMIC_PWRDET)      += powerdetect.o

# To override the "-march=armv7-xxx" in arm/mach-tegra/Makefile
AFLAGS_sleep.o :=
