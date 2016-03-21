GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/media/platform/tegra/camera
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../kernel-t18x/drivers/video/tegra/host/
endif


obj-y += vi_irq.o
obj-y += tegra_vi.o

nvhost-vi-objs += vi.o
obj-$(CONFIG_VIDEO_TEGRA_VI) += nvhost-vi.o
