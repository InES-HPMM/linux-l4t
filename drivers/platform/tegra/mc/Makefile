#
# Memory controller code.
#

obj-y                                   += mc.o

# MC error reporting.
obj-y                                   += mcerr.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += mcerr-t3.o
obj-$(CONFIG_ARCH_TEGRA_11x_SOC)        += mcerr-t11.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += mcerr-t12.o
obj-$(CONFIG_ARCH_TEGRA_14x_SOC)        += mcerr-t14.o

ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += mc-timing-t12x.o
endif
