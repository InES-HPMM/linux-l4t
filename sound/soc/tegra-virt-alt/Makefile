GCOV_PROFILE := y

subdir-ccflags-y := -Werror
ccflags-y += -I$(src)/../tegra-alt
ccflags-y += -I$(src)/../codecs

# Tegra platform Support
snd-soc-tegra124-virt-alt-apbif-objs := tegra124_virt_alt_apbif.o
snd-soc-tegra124-virt-alt-ivc-objs := tegra124_virt_alt_ivc.o
snd-soc-tegra124-virt-alt-xbar-objs := tegra124_virt_alt_xbar.o
snd-soc-tegra124-virt-alt-dam-objs := tegra124_virt_alt_dam.o

obj-$(CONFIG_SND_SOC_TEGRA_VIRT_ALT) += snd-soc-tegra124-virt-alt-ivc.o
obj-$(CONFIG_SND_SOC_TEGRA124_VIRT_ALT_APBIF) += snd-soc-tegra124-virt-alt-apbif.o
obj-$(CONFIG_SND_SOC_TEGRA124_VIRT_ALT_XBAR) += snd-soc-tegra124-virt-alt-xbar.o
obj-$(CONFIG_SND_SOC_TEGRA124_VIRT_ALT_DAM) += snd-soc-tegra124-virt-alt-dam.o

# Tegra machine Support for control card
snd-soc-tegra-virt-vcm30t124-control-objs := tegra_virt_vcm30t124_control.o
obj-$(CONFIG_SND_SOC_TEGRA_VIRT_CONTROL) += snd-soc-tegra-virt-vcm30t124-control.o

snd-soc-tegra-virt-vcm30t124-pcm-objs := tegra_virt_vcm30t124_pcm.o
obj-$(CONFIG_SND_SOC_TEGRA_VIRT_PCM) += snd-soc-tegra-virt-vcm30t124-pcm.o
