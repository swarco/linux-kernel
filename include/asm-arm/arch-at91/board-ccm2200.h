/*
 * linux/include/asm-arm/arch-at91rm9200/board-ccm2200.h
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      bord specific defines for the CCM2200 board
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
 *     2006-05-02 gc: initial version
 */

#ifndef __ASM_ARCH_BOARD_CCM2200_H
#define __ASM_ARCH_BOARD_CCM2200_H


/****************************************************************************
 * Constants for CCM2200 memory map
 ****************************************************************************/

/* 
 * PHYS 0: internal memory, CS0: NOR-Flash, CS1: SD-RAM common defined in
 * hardware.h for all AT91RM9200 based boards 
 */


/* CS2: SRAM */
#define CCM2200_SRAM_CS                 2
#define CCM2200_SRAM_PHYS               AT91_CHIPSELECT_2
#define CCM2200_SRAM_SIZE               (2*1024*1024)


/* CS3: NAND-Flash */
#define CCM2200_NAND_FLASH_CS           3
#define CCM2200_NAND_FLASH_PHYS         AT91_CHIPSELECT_3
#define CCM2200_NAND_FLASH_SIZE         0x8000

/* CS4: external quad UART */
#define CCM2200_QUAD_UART_CS            4
#define CCM2200_QUAD_UART_PHYS          AT91_CHIPSELECT_4
#define CCM2200_QUAD_UART_SIZE          0x20

/* CS5: digital output */
#define CCM2200_DIG_OUT_CS              5
#define CCM2200_DIG_OUT_PHYS            AT91_CHIPSELECT_5
#define CCM2200_DIG_OUT_SIZE            0x1

/* CS6: extension board */
#define CCM2200_EXT_A_CS                6
#define CCM2200_EXT_A_PHYS              AT91_CHIPSELECT_6

/* CS7: extension board */
#define CCM2200_EXT_B_CS                7
#define CCM2200_EXT_B_PHYS              AT91_CHIPSELECT_7

/****************************************************************************
 * Definitions for external SC16C754 quad UART
 ****************************************************************************/

#define CCM2200_QUAD_UART_IRQ           AT91RM9200_ID_IRQ0
#define CCM2200_QUAD_UART_CLOCK_RATE    (18432000/2)

#define CCM2200_PIN_PCK1_UART_CLOCK     AT91_PIN_PA24
#define CCM2200_PIN_UART_RESET          AT91_PIN_PB2
#define CCM2200_PIN_UART_IRQ            AT91_PIN_PB29

#define CCM2200_QUAD_UART_PORT_OFFSET   0x8
#define CCM2200_QUAD_UART_NUM_PORTS     4


/* initialize AT91 UART subclass driver and configure ports */
int __init ccm2200_at91_uart_init(void);
#define AT91RM9200_PIN_DBG_UART_TXD     AT91_PIN_PA31
#define AT91RM9200_PIN_DBG_UART_RXD     AT91_PIN_PA30


/****************************************************************************
 * access to CCM2200 frontpanel LEDs and digital outputs
 * (defined in ccm2200_gpio.c)
 ****************************************************************************/
void ccm2200_set_digital_output(register __u32 data, register __u32 mask);
void ccm2200_set_frontpanel_leds(register __u32 data, register __u32 mask);
__u32 ccm2200_get_sconf_input(void);
__u32 ccm2200_get_digital_input(void);

/****************************************************************************
 * CCM2200 board specific serial extended functions (RS485 and LED support)
 ****************************************************************************/

#include <linux/ccm2200_serial.h>
#include <linux/timer.h>

struct ccm2200_led_handler {
        struct ccm2200_serial_led led;
        struct timer_list led_timer;
};

struct uart_port;
#include <linux/serial_core.h>  /* struct uart_ops */

struct ccm2200_board_serial
{
        /* 
         * remember state for kernel driven RS485 modes,
         * in normal RS232 operation we stay always in 
         * CCM2200_BS_RECEIVE
         */
        enum ccm2200_board_serial_state {
                CCM2200_BS_NOT_INITIALIZED = 0,
                CCM2200_BS_RECEIVE,
                CCM2200_BS_DCD_WAIT, /* DCD wait on in multi-drop modem mode */
                CCM2200_BS_TURN_ON_DELAY,
                CCM2200_BS_CTS_WAIT,
                CCM2200_BS_TRANSMIT,
                CCM2200_BS_TURN_OFF_DELAY
        } state;

        struct ccm2200_serial_config conf;
        struct ccm2200_led_handler rxLed;
        struct ccm2200_led_handler txLed;
        struct uart_ops ccm2200_ops;
        struct uart_ops *orig_ops;
        struct timer_list turn_on_off_timer;
};

void ccm2200_board_serial_init(struct uart_port *port,
                               struct ccm2200_board_serial *se);
void ccm2200_board_serial_remove(struct uart_port *port);
void ccm2200_board_serial_trigger_led(struct ccm2200_led_handler *led);
void ccm2200_board_serial_rs485_tx(struct uart_port *port);

#endif /* __ASM_ARCH_BOARD_CCM2200_H */

/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */
