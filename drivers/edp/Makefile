GCOV_PROFILE := y

obj-$(CONFIG_EDP_FRAMEWORK) += edp.o
obj-$(CONFIG_EDP_FRAMEWORK) += edp_bestfit.o
obj-$(CONFIG_EDP_FRAMEWORK) += edp_fair.o
obj-$(CONFIG_EDP_FRAMEWORK) += edp_overage.o
obj-$(CONFIG_EDP_FRAMEWORK) += edp_priority.o
obj-$(CONFIG_EDP_FRAMEWORK) += edp_temporal.o
obj-$(CONFIG_EDP_FRAMEWORK) += edp_sysfs.o
obj-$(CONFIG_EDP_FRAMEWORK) += psy_depletion.o
obj-$(CONFIG_EDP_FRAMEWORK) += tegra_core.o
obj-$(CONFIG_SYSEDP_FRAMEWORK) += sysedp.o
obj-$(CONFIG_SYSEDP_FRAMEWORK) += sysedp_sysfs.o
obj-$(CONFIG_SYSEDP_FRAMEWORK) += sysedp_dynamic_capping.o
obj-$(CONFIG_SYSEDP_FRAMEWORK) += sysedp_batmon_calc.o
ifdef CONFIG_DEBUG_FS
obj-$(CONFIG_EDP_FRAMEWORK) += edp_debug.o
obj-$(CONFIG_SYSEDP_FRAMEWORK) += sysedp_debug.o
endif