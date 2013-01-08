GCOV_PROFILE := y
#subdir-ccflags-y := -Werror
EXTRA_CFLAGS += -Idrivers/video/tegra/host
ifeq ($(CONFIG_ARM64),y)
subdir-ccflags-y += -Iarch/arm/mach-tegra/include
ccflags += -Iarch/arm/mach-tegra/include
endif
obj-$(CONFIG_TEGRA_GRHOST) += host/
obj-$(CONFIG_GK20A) += host/gk20a/
obj-$(CONFIG_TEGRA_DC) += dc/
obj-$(CONFIG_FB_TEGRA) += fb.o
obj-$(CONFIG_TEGRA_NVMAP) += nvmap/
obj-$(CONFIG_TEGRA_CAMERA) += camera/
