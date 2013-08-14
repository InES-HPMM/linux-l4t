GCOV_PROFILE := y

# Reference the mach-tegra from arch/arm for now
ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
asflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra

# Assemblies
obj-y += sleep.o
obj-y += sleep-t3.o
obj-y += headsmp.o

obj-$(CONFIG_SMP) += platsmp.o

# Temp board files (should be gone when we go DT)
obj-${CONFIG_MACH_EXUMA}              += board-exuma.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-panel.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-pinmux.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-power.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-sdhci.o
obj-${CONFIG_MACH_EXUMA}              += board-exuma-sensors.o

# To override the "-march=armv7-xxx" in arm/mach-tegra/Makefile
AFLAGS_sleep.o :=
