/*
 * This file is part of wl12xx
 *
 * Copyright (C) 2010-2011 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/wl12xx.h>
#include <linux/gpio.h>

#define WL12XX_PLATFORM_GPIO_IRQ       63

static struct wl12xx_platform_data platform_data = {
	/* IRQ will be supplied by wl12xx_get_platform_data */
	.irq = 0,
	/* 38.4Mhz TCXO, see http://processors.wiki.ti.com/index.php/WL127x_Modules */
	.board_ref_clock = WL12XX_REFCLOCK_38,
};

int __init wl12xx_set_platform_data(const struct wl12xx_platform_data *data)
{
	printk(KERN_WARNING "wl12xx_set_platform_data called.\n");
	return 0;
}

struct wl12xx_platform_data *wl12xx_get_platform_data(void)
{
	printk(KERN_DEBUG "wl12xx_get_platform_data.\n");
	if (platform_data.irq == 0) {
		if (gpio_request_one(WL12XX_PLATFORM_GPIO_IRQ, GPIOF_IN, "wl12xx_irq"))
			printk(KERN_WARNING "%s failed to aquire wlan_irq gpio %d.\n", __func__, WL12XX_PLATFORM_GPIO_IRQ);
		platform_data.irq = gpio_to_irq(WL12XX_PLATFORM_GPIO_IRQ);
	}
	return &platform_data;
}
EXPORT_SYMBOL(wl12xx_get_platform_data);
