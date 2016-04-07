GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/media/platform/tegra
ccflags-y += -Werror
ifeq ($(CONFIG_ARCH_TEGRA_21x_SOC), y)
ccflags-y += -Idrivers/media/platform/tegra/mipical
endif
obj-y += camera_common.o mc_common.o core.o channel.o graph.o
