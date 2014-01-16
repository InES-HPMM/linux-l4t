dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb

targets += dtbs
dtb-$(CONFIG_MACH_EXUMA) += tegra132-exuma.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-norrin.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-laguna.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-ers-s.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-ers.dtb
targets += $(dtb-y)

DTC_FLAGS := -i $(srctree)/arch/arm/boot/dts

DTCCPP_FLAGS := -I$(srctree)/arch/arm/boot/dts

dtbs: $(addprefix $(obj)/, $(dtb-y))

clean-files := *.dtb
