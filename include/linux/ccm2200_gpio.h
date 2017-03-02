/*
 * linux/include/linux/ccm2200_gpio.h
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Userspace access to CCM2200 digital in-/output lines
 *              and indicator LEDs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  @par Modification History:
 *     2006-05-04 gc: initial version (partly derived from ite_gpio.h
 *                    written by Hai-Pao Fan <haipao@mvista.com>)
 */

#ifndef __CCM2200_GPIO_H
#define __CCM2200_GPIO_H

#include <linux/ioctl.h>

struct ccm2200_gpio_ioctl_data {
	__u32 device;
	__u32 mask;
	__u32 data;
};

#define CCM2200_GPIO_IOCTL_BASE	'g'

#define CCM2200_GPIO_IN		_IOWR(CCM2200_GPIO_IOCTL_BASE, 0, struct ccm2200_gpio_ioctl_data)
#define CCM2200_GPIO_OUT	_IOW (CCM2200_GPIO_IOCTL_BASE, 1, struct ccm2200_gpio_ioctl_data)
#define CCM2200_GPIO_SET	_IOW (CCM2200_GPIO_IOCTL_BASE, 2, struct ccm2200_gpio_ioctl_data)
#define CCM2200_GPIO_CLEAR	_IOW (CCM2200_GPIO_IOCTL_BASE, 3, struct ccm2200_gpio_ioctl_data)

#define	CCM2200_GPIO_INPUT	0x01
#define	CCM2200_GPIO_OUTPUT	0x02
#define	CCM2200_GPIO_LED	0x03
#define	CCM2200_GPIO_SCONF	0x04


/* Watchdog LED ioctl */
#define	CCM2200_WATCHDOG_IOCTL_BASE	'W'
#define	CCM2200_WDIOC_SETLED    _IOWR(WATCHDOG_IOCTL_BASE, 100, int)

#endif
