GCOV_PROFILE := y

subdir-ccflags-y := -Werror
subdir-ccflags-y += -I$(srctree)/drivers/platform/tegra/include

EXTRA_CFLAGS += -Idrivers/video/tegra/host

ccflags-$(CONFIG_ADF_TEGRA) += -Idrivers/staging/android

ifeq ($(CONFIG_ARM64),y)
subdir-ccflags-y += -Iarch/arm/mach-tegra/include
ccflags          += -Iarch/arm/mach-tegra/include
endif

obj-$(CONFIG_TEGRA_GRHOST) += host/
obj-$(CONFIG_TEGRA_DC)     += dc/
obj-$(CONFIG_FB_TEGRA)     += fb.o
obj-$(CONFIG_ADF_TEGRA)    += tegra_adf.o
obj-$(CONFIG_TEGRA_NVMAP)  += nvmap/
obj-$(CONFIG_TEGRA_CAMERA_PLATFORM) += camera/
obj-$(CONFIG_TEGRA_GR_VIRTUALIZATION) += virt/
