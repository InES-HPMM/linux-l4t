GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../kernel-t18x/drivers/video/tegra/host/
endif


ifneq (,$(filter $(CONFIG_VIDEO_TEGRA_VI),y m))
obj-y += vi_irq.o
obj-y += tegra_vi.o
endif
obj-y += camera_common.o

nvhost-vi-objs += vi.o vi_common.o core.o channel.o graph.o
nvhost-csi-objs += csi.o
obj-$(CONFIG_VIDEO_TEGRA_VI) += nvhost-vi.o
obj-$(CONFIG_VIDEO_TEGRA_VI) += nvhost-csi.o
