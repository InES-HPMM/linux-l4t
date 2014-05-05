#
# Memory controller code.
#

obj-y                                   += mc.o

# MC error reporting.
obj-y                                   += mcerr.o
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += mcerr-t12.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += mcerr-t21.o

ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_12x_SOC)        += mc-timing-t12x.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += mc-timing-t21x.o
endif
