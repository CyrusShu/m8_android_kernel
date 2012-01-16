/* arch/arm/plat-s3c64xx/include/plat/s3c6410.h
 *
 * Copyright 2008 Openmoko,  Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * Header file for s3c6410 cpu support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifdef CONFIG_CPU_S3C6410
extern void s3c64xx_common_init_uarts(struct s3c_uartcfg *cfg, int no);
extern void s3c64xx_setup_clocks(void);

extern  int s3c6410_init(void);
extern void s3c6410_init_irq(void);
extern void s3c6410_map_io(void);
extern void s3c6410_init_clocks(int xtal);
extern void s3c6410_register_clocks(void);

extern int m8_checkse(void);

//#define IRQ_BT_HOST_WAKE      IRQ_EINT(11)
#define IRQ_WLAN_BT_HOST_WAKE      IRQ_EINT(19)
//#define GPIO_WLAN_BT_EN		S3C64XX_GPK(1)
//#define GPIO_BT_nRST			S3C64XX_GPM(3)
//#define GPIO_BT_HOST_WAKE		S3C64XX_GPN(11)
#define GPIO_WLAN_BT_HOST_WAKE	S3C64XX_GPL(11)

extern void m8_bt_power(int on, int sdio);
extern void m8_wifi_power(int on);

enum { NONE = 0x0, WIFI_ON = 0x1, BT_ON = 0x2 };
extern int m8_get_wifi_bt_status(void);

extern void m8_wlan_bt_enable_irq(unsigned int irq);
extern void m8_wlan_bt_disable_irq(unsigned int irq);
extern void m8_wlan_bt_enable_wake_lock(void);
extern void m8_wlan_bt_disable_wake_lock(void);

#define s3c6410_init_uarts s3c64xx_common_init_uarts

#else
#define s3c6410_init_clocks NULL
#define s3c6410_init_uarts NULL
#define s3c6410_map_io NULL
#define s3c6410_init NULL
#endif
