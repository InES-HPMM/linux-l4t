dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb

targets += dtbs
dtb-$(CONFIG_MACH_EXUMA) += tegra132-exuma.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-norrin-pm374-0001-a00-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-norrin-pm374-0001-a01-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-norrin-tegrasim.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-laguna.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-e1973-1000-a01-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-e1971-1100-a00-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-e1971-1100-a00-00-pcie.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-e1971-1100-a00-00-battery.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-e1971-1100-a00-00-powerconfig.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-tn8-p1761-1270-a03-battery.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-tn8-p1761-1270-a03.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-interposer-e2141-1100-a00-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-interposer-e2141-1100-a00-00-cboot.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-grenada.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-ers-e2190-1100-a00-00.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-ers-e2220-1100-a00-00.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-foster-e-p2530-0032-e00-00.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-foster-e-p2530-0930-e01-00.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-foster-e-hdd-p2530-0932-e00-00.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-foster-e-hdd-p2530-0930-e01-00.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-loki-e-p2530-0032-e00-00.dtb
dtb-$(CONFIG_MACH_GRENADA) += tegra210-vcm30t210.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-sim.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-sim-cl33688874.dtb
targets += $(dtb-y)

DTC_FLAGS := -i $(srctree)/arch/arm/boot/dts
ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
DTC_FLAGS += -i $(srctree)/../kernel-t18x/arch/arm64/boot/dts
endif

DTCCPP_FLAGS := -I$(srctree)/arch/arm/boot/dts
ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
DTCCPP_FLAGS += -I$(srctree)/../kernel-t18x/arch/arm64/boot/dts
endif

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(dtb-y))

clean-files := dts/*.dtb *.dtb
