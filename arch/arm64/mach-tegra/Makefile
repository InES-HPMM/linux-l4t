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
obj-${CONFIG_MACH_EXUMA}              += board-exuma-kbc.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-panel.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-memory.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-pinmux.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-power.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-powermon.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-sdhci.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-sensors.o
obj-${CONFIG_MACH_EXUMA}              += board-tn8-power.o

ifeq ($(CONFIG_HOTPLUG_CPU),y)
obj-$(CONFIG_DENVER_CPU)              += hotplug-denver.o
endif

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
