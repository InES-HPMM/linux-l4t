#
# Memory controller code.
#

GCOV_PROFILE := y

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra

obj-y                                   += mc.o

# MC error reporting.
obj-y                                   += mcerr.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += mcerr-t12.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += mcerr-t21.o

# EMC clock control
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += tegra21_emc.o

# Latency allowance
obj-y                                   += latency_allowance.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += tegra12x_la.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += tegra12x_la.o

ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += mc-timing-t12x.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += mc-timing-t21x.o
endif
