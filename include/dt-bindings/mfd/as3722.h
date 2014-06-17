/*
 * This header provides constants for binding ams,as3722.
 */

#ifndef _DT_BINDINGS_MFD_AS3722_H
#define _DT_BINDINGS_MFD_AS3722_H

#define AS3722_EXT_CONTROL_ENABLE1 1
#define AS3722_EXT_CONTROL_ENABLE2 2

#define AS3722_BBCCUR_50UA		0
#define AS3722_BBCCUR_200UA		1
#define AS3722_BBCCUR_100UA		2
#define AS3722_BBCCUR_400UA		3
#define AS3722_BBCRESOFF_ENABLE		0
#define AS3722_BBCRESOFF_BYPASS		1
#define AS3722_BBCMODE_OFF		0
#define AS3722_BBCMODE_ACTIVE		1
#define AS3722_BBCMODE_ACT_STBY		2
#define AS3722_BBCMODE_ACT_STBY_OFF	3

/* Power Good OC Mask macro */
#define AS3722_OC_PG_MASK_AC_OK            0x1
#define AS3722_OC_PG_MASK_GPIO3            0x2
#define AS3722_OC_PG_MASK_GPIO4            0x4
#define AS3722_OC_PG_MASK_GPIO5            0x8
#define AS3722_OC_PG_MASK_PWRGOOD_SD0      0x10
#define AS3722_OC_PG_MASK_OVCURR_SD0       0x20
#define AS3722_OC_PG_MASK_POWERGOOD_SD6    0x40
#define AS3722_OC_PG_MASK_OVCURR_SD6       0x80

/* Thermal sensor zones */
#define AS3722_SD0_SENSOR              0
#define AS3722_SD1_SENSOR              1
#define AS3722_SD6_SENSOR              2

/* SDx temperature ADC channel */
#define AS3722_TEMP1_SD0               16
#define AS3722_TEMP2_SD0               17
#define AS3722_TEMP3_SD0               18
#define AS3722_TEMP4_SD0               19
#define AS3722_TEMP_SD1                20
#define AS3722_TEMP1_SD6               21
#define AS3722_TEMP2_SD6               22

#endif
