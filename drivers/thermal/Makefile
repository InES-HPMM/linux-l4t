#
# Makefile for sensor chip drivers.
#

obj-$(CONFIG_THERMAL)		+= thermal_sys.o
thermal_sys-y			+= thermal_core.o
thermal_sys-$(CONFIG_DEBUG_FS)	+= thermal_debugfs.o

# interface to/from other layers providing sensors
thermal_sys-$(CONFIG_THERMAL_HWMON)		+= thermal_hwmon.o
thermal_sys-$(CONFIG_THERMAL_OF)		+= of-thermal.o

# governors
thermal_sys-$(CONFIG_THERMAL_GOV_FAIR_SHARE)	+= fair_share.o
thermal_sys-$(CONFIG_THERMAL_GOV_STEP_WISE)	+= step_wise.o
thermal_sys-$(CONFIG_THERMAL_GOV_PID)		+= pid_thermal_gov.o
thermal_sys-$(CONFIG_THERMAL_GOV_ADAPTIVE_SKIN) += adaptive_skin.o
thermal_sys-$(CONFIG_THERMAL_GOV_USER_SPACE)	+= user_space.o

# cpufreq cooling
thermal_sys-$(CONFIG_CPU_THERMAL)	+= cpu_cooling.o

# platform thermal drivers
obj-$(CONFIG_SPEAR_THERMAL)	+= spear_thermal.o
obj-$(CONFIG_RCAR_THERMAL)	+= rcar_thermal.o
obj-$(CONFIG_KIRKWOOD_THERMAL)  += kirkwood_thermal.o
obj-$(CONFIG_EXYNOS_THERMAL)	+= exynos_thermal.o
obj-$(CONFIG_DOVE_THERMAL)  	+= dove_thermal.o
obj-$(CONFIG_DB8500_THERMAL)	+= db8500_thermal.o
obj-$(CONFIG_ARMADA_THERMAL)	+= armada_thermal.o
obj-$(CONFIG_DB8500_CPUFREQ_COOLING)	+= db8500_cpufreq_cooling.o
obj-$(CONFIG_INTEL_POWERCLAMP)	+= intel_powerclamp.o
obj-$(CONFIG_PWM_FAN)		+= pwm_fan.o
obj-$(CONFIG_GENERIC_ADC_THERMAL)	+= generic_adc_thermal.o
obj-$(CONFIG_GENERIC_ADC_THERMAL)	+= of_generic_adc_thermal.o
obj-$(CONFIG_PALMAS_THERMAL)	+= palmas_thermal.o
obj-$(CONFIG_MAX77620_THERMAL)    += max77620-thermal.o
obj-$(CONFIG_AS3722_THERMAL)    += as3722_thermal.o
obj-$(CONFIG_SENSORS_TMP006)	+= tmp006.o
obj-$(CONFIG_TEGRA_AOTAG)	+= tegra_aotag.o
obj-$(CONFIG_MODEM_THERMAL)	+= modem_thermal.o
