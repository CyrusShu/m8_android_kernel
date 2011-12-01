/*
 * linux/drivers/power/s3c6410_battery.h
 *
 * Battery measurement code for S3C6410 platform.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DRIVER_NAME	"smdk6410-battery"

/*
 * SMDK6410 board ADC channel
 */
typedef enum s3c_adc_channel {
	S3C_ADC_VOLTAGE = 0,
	S3C_ADC_TEMPERATURE,
	ENDOFADC
} adc_channel_type;

#define GPIO_TA_CONNECTED_N 	S3C64XX_GPN(13)
#define GPIO_TA_CHG_N		S3C64XX_GPN(3)
#define GPIO_TA_EN		S3C64XX_GPL(0) /* ??? */

#define IRQ_TA_CONNECTED_N	IRQ_EINT(13)
#define IRQ_TA_CHG_N		IRQ_EINT(3)

#define convert_adc2voltage(x)		((x - 2170) * 10 / 7 / 100 * 100 + 3200)
