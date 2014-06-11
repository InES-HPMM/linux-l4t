#
# Memory controller code.
#

ifneq ($(CONFIG_ARM64),)
ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
endif

obj-y                                   += mc.o

# MC error reporting.
obj-y                                   += mcerr.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += mcerr-t12.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += mcerr-t21.o

# EMC clock control
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += tegra21_emc.o

ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += mc-timing-t12x.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += mc-timing-t21x.o
endif
