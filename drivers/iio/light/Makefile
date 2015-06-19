#
# Makefile for IIO Light sensors
#

GCOV_PROFILE := y

ccflags-y += -Werror
obj-$(CONFIG_ADJD_S311)		+= adjd_s311.o
obj-$(CONFIG_SENSORS_LM3533)	+= lm3533-als.o
obj-$(CONFIG_SENSORS_TSL2563)	+= tsl2563.o
obj-$(CONFIG_VCNL4000)		+= vcnl4000.o
obj-$(CONFIG_HID_SENSOR_ALS)	+= hid-sensor-als.o
obj-$(CONFIG_NVS_BH1730FVC)	+= nvs_bh1730fvc.o
obj-$(CONFIG_NVS_CM3217)	+= nvs_cm3217.o
obj-$(CONFIG_NVS_CM3218)	+= nvs_cm3218.o
obj-$(CONFIG_NVS_MAX4400X)	+= nvs_max4400x.o
obj-$(CONFIG_NVS_JSA1127)	+= nvs_jsa1127.o
obj-$(CONFIG_NVS_ISL2902X)	+= nvs_isl2902x.o
obj-$(CONFIG_NVS_LTR659)	+= nvs_ltr659.o

