/*
 * Copyright (C) 2010 Samsung Electronics Co., Ltd.
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Modified for Crespo on August, 2010 By Samsung Electronics Co.
 * This is modified operate according to each status.
 *
 * Modified for M8 on Apr., 2011.
 */

/* Control bluetooth power for M8 platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>

#include <plat/s3c6410.h>
#include <plat/devs.h>
#include <plat/sdhci.h>

#ifdef CONFIG_RFKILL

extern void s3c_setup_uart_cfg_gpio(unsigned char port);
extern void s3c_reset_uart_cfg_gpio(unsigned char port);

#define IRQ_BT_HOST_WAKE      IRQ_EINT(11)
//#define IRQ_WLAN_HOST_WAKE????      IRQ_EINT(19)

#define GPIO_WLAN_BT_EN			S3C64XX_GPK(1) // GPL(1)???
#define GPIO_BT_nRST			S3C64XX_GPM(3) // GPL(3)???
#define GPIO_BT_HOST_WAKE		S3C64XX_GPN(11)
//#define GPIO_WLAN_HOST_WAKE????		S3C64XX_GPL(11)

static struct wake_lock rfkill_wake_lock;

#ifndef	GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW		0
#define GPIO_LEVEL_HIGH		1
#endif

static struct rfkill *bt_rfk;
static const char bt_name[] = "btmrvl";

static int loaded = 0;

static int bluetooth_set_power(void *data, enum rfkill_user_states state)
{
	int ret = 0;
	int irq;
	/* BT Host Wake IRQ */
	irq = IRQ_BT_HOST_WAKE;

	switch (state) {

	case RFKILL_USER_STATE_UNBLOCKED:
		printk("[BT] Device Powering ON \n");//pr_debug

		/* Bluetooth over SDIO */
		if (loaded) {
			m8_wifi_power(1);
			msleep(100);
			if (m8_checkse()) 
				sdhci_s3c_force_presence_change(&s3c_device_hsmmc0);
		}

#if 0 // uart not work now
		s3c_setup_uart_cfg_gpio(1);

		if (gpio_is_valid(GPIO_WLAN_BT_EN)) {
			printk("[BT] gpio_is_valid(GPIO_WLAN_BT_EN)\n");
			gpio_direction_output(GPIO_WLAN_BT_EN, GPIO_LEVEL_HIGH);
			}

		if (gpio_is_valid(GPIO_BT_nRST)) {
			gpio_direction_output(GPIO_BT_nRST, GPIO_LEVEL_LOW);
						printk("[BT] gpio_is_valid(GPIO_BT_nRST)\n");
			}

		printk("[BT] GPIO_BT_nRST = %d\n",
				gpio_get_value(GPIO_BT_nRST));//pr_debug

		/* Set GPIO_BT_WLAN_REG_ON high */
		s3c_gpio_setpull(GPIO_WLAN_BT_EN, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_HIGH);

		s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT1);
		s3c_gpio_slp_setpull_updown(GPIO_WLAN_BT_EN,
				S3C_GPIO_PULL_NONE);

		printk("[BT] GPIO_WLAN_BT_EN = %d\n",
				gpio_get_value(GPIO_WLAN_BT_EN));
		/*
		 * FIXME sleep should be enabled disabled since the device is
		 * not booting if its enabled
		 */
		/*
		 * 100msec, delay between reg_on & rst.
		 * (powerup sequence)
		 */
		msleep(500);//100);

		/* Set GPIO_BT_nRST high */
		s3c_gpio_setpull(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_BT_nRST, GPIO_LEVEL_HIGH);

		s3c_gpio_slp_cfgpin(GPIO_BT_nRST, S3C_GPIO_SLP_OUT1);
		s3c_gpio_slp_setpull_updown(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);

		printk("[BT] GPIO_BT_nRST = %d\n",
				gpio_get_value(GPIO_BT_nRST));//pr_debug

		/*
		 * 50msec, delay after bt rst
		 * (libertas powerup sequence)
		 */
		msleep(200);//50);

		//ret = enable_irq_wake(irq);
		//if (ret < 0)
		//	pr_err("[BT] set wakeup src failed\n");

		enable_irq(irq);
#endif
		break;

	case RFKILL_USER_STATE_SOFT_BLOCKED:
		printk("[BT] Device Powering OFF\n");

		/* Bluetooth over SDIO */
		if (loaded) {
			m8_wifi_power(0);
			msleep(100);
			if (m8_checkse()) 
				sdhci_s3c_force_presence_change(&s3c_device_hsmmc0);
		}

#if 0 // UART
		//s3c_reset_uart_cfg_gpio(1);

		//ret = disable_irq_wake(irq);
		//if (ret < 0)
		//	pr_err("[BT] unset wakeup src failed\n");

		disable_irq(irq);
		wake_unlock(&rfkill_wake_lock);

		s3c_gpio_setpull(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_BT_nRST, GPIO_LEVEL_LOW);

		s3c_gpio_slp_cfgpin(GPIO_BT_nRST, S3C_GPIO_SLP_OUT0);
		s3c_gpio_slp_setpull_updown(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);

		printk("[BT] GPIO_BT_nRST = %d\n",
				gpio_get_value(GPIO_BT_nRST));//pr_debug

		if (gpio_get_value(GPIO_BT_nRST) == 0) { //GPIO_WLAN_nRST
			s3c_gpio_setpull(GPIO_WLAN_BT_EN, S3C_GPIO_PULL_NONE);
			gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_LOW);

			s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT0);
			s3c_gpio_slp_setpull_updown(GPIO_WLAN_BT_EN,
					S3C_GPIO_PULL_NONE);

			printk("[BT] GPIO_WLAN_BT_EN = %d\n",
					gpio_get_value(GPIO_WLAN_BT_EN));
		}
#endif
		break;

	default:
		printk(KERN_ERR "[BT] Bad bluetooth rfkill state %d\n", state);//pr_err
	}

	return 0;
}

#if 0
irqreturn_t bt_host_wake_irq_handler(int irq, void *dev_id)
{
	printk("[BT] bt_host_wake_irq_handler start\n");

	if (gpio_get_value(GPIO_BT_HOST_WAKE))
		wake_lock(&rfkill_wake_lock);
	else
		wake_lock_timeout(&rfkill_wake_lock, HZ);

	return IRQ_HANDLED;
}
#endif

static int bt_rfkill_set_block(void *data, bool blocked)
{
	unsigned int ret = 0;

	ret = bluetooth_set_power(data, blocked ?
			RFKILL_USER_STATE_SOFT_BLOCKED :
			RFKILL_USER_STATE_UNBLOCKED);

	return ret;
}

static const struct rfkill_ops bt_rfkill_ops = {
	.set_block = bt_rfkill_set_block,
};

static int __init m8_rfkill_probe(struct platform_device *pdev)
{
	int irq;
	int ret;

#if 0
	/* Initialize wake locks */
	wake_lock_init(&rfkill_wake_lock, WAKE_LOCK_SUSPEND, "bt_host_wake");

	ret = gpio_request(GPIO_WLAN_BT_EN, "GPK");
	if (ret < 0) {
		printk(KERN_ERR"[BT] Failed to request GPIO_WLAN_BT_EN!\n");//pr_error
		goto err_req_gpio_wlan_bt_en;
	}

	ret = gpio_request(GPIO_BT_nRST, "GPM");
	if (ret < 0) {
		printk(KERN_ERR"[BT] Failed to request GPIO_BT_nRST!\n");//pr_error
		goto err_req_gpio_bt_nrst;
	}

	/* BT Host Wake IRQ */
	irq = IRQ_BT_HOST_WAKE;

	ret = request_irq(irq, bt_host_wake_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"bt_host_wake_irq_handler", NULL);

	if (ret < 0) {
		pr_err("[BT] Request_irq failed\n");
		goto err_req_irq;
	}

	disable_irq(irq);
#endif

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			&bt_rfkill_ops, NULL);

	if (!bt_rfk) {
		printk(KERN_ERR"[BT] bt_rfk : rfkill_alloc is failed\n");//pr_error
		ret = -ENOMEM;
		goto err_alloc;
	}

	rfkill_init_sw_state(bt_rfk, 0);

	printk("[BT] rfkill_register(bt_rfk)\n");//pr_debug

	ret = rfkill_register(bt_rfk);
	if (ret) {
		printk("********ERROR IN REGISTERING THE RFKILL********\n");
		goto err_register;
	}

	rfkill_set_sw_state(bt_rfk, 1);
	bluetooth_set_power(NULL, RFKILL_USER_STATE_SOFT_BLOCKED);

	loaded = 1;

	return ret;

 err_register:
	rfkill_destroy(bt_rfk);

 err_alloc:
#if 0
	free_irq(irq, NULL);

 err_req_irq:
	gpio_free(GPIO_BT_nRST);

 err_req_gpio_bt_nrst:
	gpio_free(GPIO_WLAN_BT_EN);

 err_req_gpio_wlan_bt_en:
#endif
	return ret;
}

static struct platform_driver m8_device_rfkill = {
	.probe = m8_rfkill_probe,
	.driver = {
		.name = "bt_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init m8_rfkill_init(void)
{
	int rc = 0;
	rc = platform_driver_register(&m8_device_rfkill);

	return rc;
}

module_init(m8_rfkill_init);
MODULE_DESCRIPTION("M8 rfkill");
MODULE_LICENSE("GPL");

#endif
