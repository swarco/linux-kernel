/*
 * linux/include/linux/ccm2200_serial.h
 *
 * Copyright (C) 2007 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      CCM2200 board specific ioctl to configure special serial port
 *              operating modes
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
 *     2007-02-01 gc: initial version
 */

#ifndef __CCM2200_SERIAL_H
#define __CCM2200_SERIAL_H

#include <linux/ioctl.h>

struct ccm2200_serial_config {
	enum ccm2200_serial_mode {
                /* normal RS232 configuration, no modification to standard Linux
		 * Kernel behavior (e.g. for all AT-Modems) 
		 */
		CCM2200_SERIAL_MODE_NORMAL,   

                /* RTS Pin driven by UART-hardware (Atmel only, no
		 * Weiss/CC2200+ RS-485) 
		 */
		CCM2200_SERIAL_MODE_RS485HW,  

		/* Kernel-Software driven RTS pin (UART TX-Empty
		 * Interrupt) (not implemented) 
		 */
		CCM2200_SERIAL_MODE_RS485INT, 

		/* Multi-drop modem controlled by host via
		 * modem-control lines ignoring DCD signal (e.g. Loges
		 * 1200MD) 
		 */
		CCM2200_SERIAL_MODE_MODEM_MD, 

		/* Multi-drop modem controlled by host via
		 * modem-control lines waiting for DCD signal
		 * (e.g. Loges 1200MD) 
		 */
		CCM2200_SERIAL_MODE_MODEM_MD_DCD, 

		/* Software driven RS-485 (Weiss Piggyback/CC2200+)
		 * with RTS pin, no evalution of any other control
		 * signal (CTS, DCD, RI)
		 */
		CCM2200_SERIAL_MODE_RS485KERN,

		/* Software driven RS-485 (Weiss Piggyback/CC2200+)
		 * with RTS pin (inverted), no evalution of any other
		 * control signal (CTS, DCD, RI).  Using this mode RTS
		 * is inactive during transmission and active while
		 * receiving
		 */
		CCM2200_SERIAL_MODE_RS485KERN_NEG,
	} mode;
	__u32 turn_on_delay;	    /* RS485 transmitter turn on delay  */
	__u32 turn_off_delay;	    /* RS485 transmitter turn off delay */
};


struct ccm2200_serial_led {
	__u32 mask;
	__u32 delay;
};


#define CCM2200_SERIAL_IOCTL_BASE	'w'

#define CCM2200_SERIAL_GET_CONF		_IOR(CCM2200_SERIAL_IOCTL_BASE, 0, struct ccm2200_serial_config)
#define CCM2200_SERIAL_SET_CONF		_IOW(CCM2200_SERIAL_IOCTL_BASE, 1, struct ccm2200_serial_config)
#define CCM2200_SERIAL_GET_TX_LED	_IOR(CCM2200_SERIAL_IOCTL_BASE, 2, struct ccm2200_serial_led)
#define CCM2200_SERIAL_SET_TX_LED	_IOW(CCM2200_SERIAL_IOCTL_BASE, 3, struct ccm2200_serial_led)
#define CCM2200_SERIAL_GET_RX_LED	_IOR(CCM2200_SERIAL_IOCTL_BASE, 4, struct ccm2200_serial_led)
#define CCM2200_SERIAL_SET_RX_LED	_IOW(CCM2200_SERIAL_IOCTL_BASE, 5, struct ccm2200_serial_led)

#endif
