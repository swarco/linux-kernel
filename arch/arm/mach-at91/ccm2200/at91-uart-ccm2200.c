/*
 * linux/arch/arm/mach-at91rm9200/at91-uart-ccm2200.c
 *
 * Copyright (C) 2006 by Weiss-Electronic 
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Subclass driver for AT91RM9200 UARTs on CCM2200
 *              This module drives the additional modem control lines
 *              over AT91 GPIO ports
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
 *     2006-05-04 gc: initial version
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>


#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/irq.h>


#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/board-ccm2200.h>
#include <asm/arch/at91-util.h>
#include <asm/arch/at91_pio.h>
#include <asm/mach/serial_at91.h>

#include <linux/serial_core.h>
//#include <asm/mach/serial_at91rm9200.h>
//#include <asm/arch/AT91RM9200_USART.h>

#include <asm/hardware.h>

#include "../../../../drivers/serial/atmel_serial.h"
//#include <asm/arch/pio.h>

/* only use this module if AT91 serial support is compiled in */
#ifdef CONFIG_SERIAL_ATMEL


#define AT91_USART0     0
#define AT91_USART1     1
#define AT91_USART2     2
#define AT91_USART3     3
#define AT91_USART_DBG  4

/*
 * Serial port configuration.
 * The AT91RM9200 serial ports are used as follows on the CCM2200 board:
 *
 *   USART0: COM2
 *   USART1: COM1
 *   USART2: COM5 Console
 *   USART3: COM6
 *   USART_DBG: optional available on COM5 connector instead of hardware flow
 *              control signals RTS/CTS. These pins will be automatically
 *              switched to the Debug USART if the correponding device is
 *              opended!
 *
 */

/*
 * Serial port configuration.
 *    ttyS0 .. ttyS3 = USART0 .. USART3
 *    ttyS4      = DBGU
 */
/* 2007-05-30 gc: no __initdata used by at91_get_usart */
static struct at91_uart_config ccm2200_uart_config = {
	.console_tty	= 2,				/* ttyS2 */
	.nr_tty		= 5,
        /* ttyS0, ..., ttyS4 */ 
	.tty_map	= { AT91_USART0,          
                            AT91_USART1,
                            AT91_USART2,
                            AT91_USART3,
                            AT91_USART_DBG }
};


static inline int at91_get_usart(struct uart_port *port)
{
        return ccm2200_uart_config.tty_map[port->line];
}

#define UART_PUT_CR(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_CR)
#define UART_GET_MR(port)	__raw_readl((port)->membase + ATMEL_US_MR)
#define UART_PUT_MR(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_MR)
#define UART_PUT_IER(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_IER)
#define UART_PUT_IDR(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_IDR)
#define UART_GET_IMR(port)	__raw_readl((port)->membase + ATMEL_US_IMR)
#define UART_GET_CSR(port)	__raw_readl((port)->membase + ATMEL_US_CSR)
#define UART_GET_CHAR(port)	__raw_readl((port)->membase + ATMEL_US_RHR)
#define UART_PUT_CHAR(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_THR)
#define UART_GET_BRGR(port)	__raw_readl((port)->membase + ATMEL_US_BRGR)
#define UART_PUT_BRGR(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_BRGR)
#define UART_PUT_RTOR(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_RTOR)



enum at91_dbg_uart_pin_cfg {
        AT91_USART2_RTS_CTS,
        AT91_USART_DBG_RX_TX
};

#define AT91C_PA17_TXD0		(1<<17)         //  USART 0 Transmit Data
#define AT91C_PA18_RXD0		(1<<18)         //  USART 0 Receive Data
#define AT91C_PA20_CTS0		(1<<20)         //  USART 0 Clear To Send
#define AT91C_PD21_RTS0		(1<<21)         //  Usart 0 Ready To Send
#define AT91C_PB20_TXD1		(1<<20)         //  USART 1 Transmit Data
#define AT91C_PB21_RXD1		(1<<21)         //  USART 1 Receive Data
#define AT91C_PB23_DCD1		(1<<23)         //  USART 1 Data Carrier Detect
#define AT91C_PB24_CTS1		(1<<24)         //  USART 1 Clear To Send
#define AT91C_PB25_DSR1		(1<<25)         //  USART 1 Data Set ready
#define AT91C_PB26_RTS1		(1<<26)         //  USART 1 Ready To Send
#define AT91C_PD25_DTR1		(1<<25)         //  USART 1 Data Terminal ready
#define AT91C_PA22_RXD2		(1<<22)         //  USART 2 Receive Data
#define AT91C_PA23_TXD2		(1<<23)         //  USART 2 Transmit Data
#define AT91C_PA5_TXD3		(1<<5)          //  USART 3 Transmit Data
#define AT91C_PA6_RXD3          (1<<6)          //  USART 3 Receive Data
#define AT91C_PB0_RTS3          (1<<0)          //  USART 3 Ready To Send
#define AT91C_PB1_CTS3		(1<<1)          //  USART 3 Clear To Send
#define AT91C_PA30_CTS2         (1<<30)
#define AT91C_PA31_RTS2         (1<<31)
#define AT91C_PA30_DRXD         (1<<30)
#define AT91C_PA31_DTXD         (1<<31)

static void at91_config_dbg_uart_pins( enum at91_dbg_uart_pin_cfg cfg)
{
        switch (cfg) {
        case AT91_USART2_RTS_CTS:
/*                 AT91_SYS->PIOA_PDR = AT91C_PA30_CTS2 | AT91C_PA31_RTS2; */
/*                 AT91_SYS->PIOA_BSR = AT91C_PA30_CTS2 | AT91C_PA31_RTS2; */
                at91_sys_write(AT91_PIOA + PIO_PDR, 
                               AT91C_PA30_CTS2 | AT91C_PA31_RTS2);
                at91_sys_write(AT91_PIOA + PIO_BSR, 
                               AT91C_PA30_CTS2 | AT91C_PA31_RTS2);
                break;

        case AT91_USART_DBG_RX_TX:
/*                 AT91_SYS->PIOA_PDR = AT91C_PA30_DRXD | AT91C_PA31_DTXD; */
/*                 AT91_SYS->PIOA_ASR = AT91C_PA30_DRXD | AT91C_PA31_DTXD; */
                at91_sys_write(AT91_PIOA + PIO_PDR, 
                               AT91C_PA30_DRXD | AT91C_PA31_DTXD);
                at91_sys_write(AT91_PIOA + PIO_ASR, 
                               AT91C_PA30_DRXD | AT91C_PA31_DTXD);
                break;
        }
}

static void ccm2200_at91_uart_enable_pins(int uart_num)
{
        switch (uart_num) {
        case 0:
                /* USART 0 */
                /* on CCM2200 errata pin PA21 not used, instead: 
                 * CTS0 on PA20
                 * DTR  on PD20 (gpio)
                 * RTS0 on PD21
                 * DCD  on PD22 (gpio)
                 * DSR  on PC14 (gpio)
                 * RI   on PC15 (gpio)
                 */
                at91_sys_write(AT91_PIOA + PIO_PDR, 
                               AT91C_PA17_TXD0 | AT91C_PA18_RXD0
                               | AT91C_PA20_CTS0);

                at91_sys_write(AT91_PIOD + PIO_PDR,
                               AT91C_PD21_RTS0);
                break;

        case 1:
                /* USART 1 */
                /* on CCM2200: 
                 * CTS1 on PB24
                 * DTR1 on PD25
                 * RTS1 on PB26
                 * DCD1 on PB23 
                 * DSR1 on PB25
                 * RI   on PB27 (gpio)
                 */                             
                at91_sys_write(AT91_PIOB + PIO_PDR, 
                               AT91C_PB20_TXD1 | AT91C_PB21_RXD1
                               | AT91C_PB23_DCD1 | AT91C_PB24_CTS1
                               | AT91C_PB25_DSR1 | AT91C_PB26_RTS1);
                at91_sys_write(AT91_PIOD + PIO_PDR, AT91C_PD25_DTR1);
                break;

        
        case 2:
                /* USART 2 */
                /* on CCM2200: 
                 * CTS2 on PA30 (Periph B)
                 * RTS2 on PA31 (Periph B)
                 * (Console Port has no more modem control lines! )
                 */                             
                at91_sys_write(AT91_PIOA + PIO_PDR, 
                               AT91C_PA22_RXD2 | AT91C_PA23_TXD2
                               | AT91C_PA30_CTS2 | AT91C_PA31_RTS2);
                at91_sys_write(AT91_PIOA + PIO_BSR, 
                               AT91C_PA30_CTS2 | AT91C_PA31_RTS2);
                break;

        case 3:
                /* USART 3 */
                /* on CCM2200: 
                 * CTS3 on PB1 (Periph B)
                 * DTR  on PC3 (gpio)
                 * RTS3 on PB0 (Periph B)
                 * DCD  on PC0 (gpio)
                 * DSR  on PC1 (gpio)
                 * RI   on PC2 (gpio)
                 */                             
                at91_sys_write(AT91_PIOA + PIO_PDR, 
                               AT91C_PA5_TXD3 | AT91C_PA6_RXD3);
                at91_sys_write(AT91_PIOA + PIO_BSR, 
                               AT91C_PA5_TXD3 | AT91C_PA6_RXD3);
                at91_sys_write(AT91_PIOB + PIO_PDR, 
                               AT91C_PB0_RTS3 | AT91C_PB1_CTS3);

                at91_sys_write(AT91_PIOB + PIO_BSR, 
                               AT91C_PB0_RTS3 | AT91C_PB1_CTS3);
                break;
        }
}

struct usart_mctrl_pins {
        /* RxD/TxD and RTS/CTS are controlled by the USART */
        struct at91_pio_pins dtr; /* output */
        struct at91_pio_pins dcd; /* input */
        struct at91_pio_pins dsr; /* input */
        struct at91_pio_pins ri;  /* input */
};

static struct usart_mctrl_pins ccm2200_mctrl_cfg[ATMEL_MAX_UART] =
{
        /* USART0: COM2 */
        {
                .dtr = { AT91_PIO_BASE(AT91_PIOD), 1<<20 },
                .dcd = { AT91_PIO_BASE(AT91_PIOD), 1<<22 },
                .dsr = { AT91_PIO_BASE(AT91_PIOC), 1<<14 },
                .ri  = { AT91_PIO_BASE(AT91_PIOC), 1<<15 }
        },
        /* USART1: COM1 (all modem control lines except RI can be controlled
         *               by the USART) */

        /* 2006-05-08 gc: control modem ctrl lines by software, since
         *                USART seams not correctly work with DCD and DSR
         *                in modem mode!
         */
#if 0
        {
                .dtr = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .dcd = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .dsr = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .ri  = { AT91_PIO_BASE(AT91_PIOB), 1<<27 }
        },
#else
        /* for testing: modem control lines controled manually */
        {
                .dtr = { AT91_PIO_BASE(AT91_PIOD), 1<<25 },
                .dcd = { AT91_PIO_BASE(AT91_PIOB), 1<<23 },
                .dsr = { AT91_PIO_BASE(AT91_PIOB), 1<<25 },
                .ri  = { AT91_PIO_BASE(AT91_PIOB), 1<<27 }
        },
#endif

        /* USART2: COM5 Console: no additional modem control lines */
        {
                .dtr = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .dcd = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .dsr = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .ri  = { AT91_PIO_BASE(AT91_PIOA), 0 }
        },

        /* USART3: COM6 */
        {
                .dtr = { AT91_PIO_BASE(AT91_PIOC), 1<<3 },
                .dcd = { AT91_PIO_BASE(AT91_PIOC), 1<<0 },
                .dsr = { AT91_PIO_BASE(AT91_PIOC), 1<<1 },
                .ri  = { AT91_PIO_BASE(AT91_PIOC), 1<<2 }
        },
        /* USART_DBG */
        {
                .dtr = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .dcd = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .dsr = { AT91_PIO_BASE(AT91_PIOA), 0 },
                .ri  = { AT91_PIO_BASE(AT91_PIOA), 0 }
        }
};



static inline
int at91_have_modem_ctrl_pin(const struct at91_pio_pins *mctrl)
{
        register AT91S_PIO *pio = mctrl->pio;

        if (pio) {
                return 1;
        }
        return 0;
}


static void ccm2200_at91_set_mctrl(struct uart_port *port, u_int mctrl)
{
	unsigned int control = 0;
        register struct usart_mctrl_pins *mctrl_pins =
                &ccm2200_mctrl_cfg[at91_get_usart(port)];


/*         printk("ccm2200_at91_set_mctrl[%d]  RTS: %d, DTR: %d\n", */
/*                port->line, */
/*                !!(mctrl & TIOCM_RTS), */
/*                !!(mctrl & TIOCM_DTR)); */
 
	if (mctrl & TIOCM_RTS)
		control |= ATMEL_US_RTSEN;
	else
		control |= ATMEL_US_RTSDIS;

	if (mctrl & TIOCM_DTR) {
		control |= ATMEL_US_DTREN;
                /* activated pin via GPIO if necessary, PIN is low active! */
                at91_pio_clear_all_pins(&mctrl_pins->dtr);
        } else {
		control |=  ATMEL_US_DTRDIS;
                /* deactivate pin via GPIO if necessary, PIN is low active! */
                at91_pio_set_all_pins(&mctrl_pins->dtr);
        }

	UART_PUT_CR(port, control);
}

/*
 * Get state of the modem control input lines
 */
static u_int ccm2200_at91_get_mctrl(struct uart_port *port)
{
	unsigned int status, ret = 0;
        int usart_num = at91_get_usart(port);
        register struct usart_mctrl_pins *mctrl_pins =
                &ccm2200_mctrl_cfg[usart_num];

        /* 
         * if (usart_num == AT91_USART1 ) {
         *         /\* enable modem mode for USART 1 *\/
         *         UART_PUT_MR(port, (UART_GET_MR(port) & ~AT91C_US_USMODE) 
         *                     | AT91C_US_USMODE_MODEM);
         * }
         */
	status = UART_GET_CSR(port);

        if (at91_have_modem_ctrl_pin(&mctrl_pins->dcd)) {
                /* PIN is low active! */
                if (!at91_pio_get_pins(&mctrl_pins->dcd))
                        ret |= TIOCM_CD;
        } else {
                /* bug in at91_serial.c: all modem control signals are low
                 *                       active!!!!
                 */
                if ((status & ATMEL_US_DCD) == 0)
                        ret |= TIOCM_CD;
        }
        
        /* bug in at91_serial.c: all modem control signals are low
         *                       active on at91rm92000!!!!
         */
	if ((status & ATMEL_US_CTS) == 0)
		ret |= TIOCM_CTS;

        if (at91_have_modem_ctrl_pin(&mctrl_pins->dsr)) {
                /* PIN is low active! */
                if (!at91_pio_get_pins(&mctrl_pins->dsr))
                        ret |= TIOCM_DSR;
        } else {
                /* bug in at91_serial.c: all modem control signals are low
                 *                       active on at91rm92000!!!!
                 */
                if ((status & ATMEL_US_DSR) == 0)
                        ret |= TIOCM_DSR;
        }
        if (at91_have_modem_ctrl_pin(&mctrl_pins->ri)) {
                /* PIN is low active! */
                if (!at91_pio_get_pins(&mctrl_pins->ri))
                        ret |= TIOCM_RI;
        } else {
                /* bug in at91_serial.c: all modem control signals are low
                 *                       active on at91rm92000!!!!
                 */
                if ((status & ATMEL_US_RI) == 0)
                        ret |= TIOCM_RI;
        }

	return ret;
}

/*
 * Enable modem status interrupts
 */
static void ccm2200_at91_enable_ms(struct uart_port *port)
{
        /* @todo: interrupt support for additional modem control lines */
	// printk("ccm2200_at91_enable_ms(%d)\n", port->line);
	UART_PUT_IER(port, ATMEL_US_RIIC | ATMEL_US_DSRIC 
                     | ATMEL_US_DCDIC | ATMEL_US_CTSIC);

        /* 2007-02-07 gc: bugfix, also set port->read_status_mask to enable
         * additional sources in interrupt!!!!
         */
        port->read_status_mask |= ATMEL_US_RIIC | ATMEL_US_DSRIC 
                | ATMEL_US_DCDIC | ATMEL_US_CTSIC;
}

static int ccm2200_at91_uart_open(struct uart_port *port)
{
        int usart_num = at91_get_usart(port);

/*         printk("ccm2200_at91_uart_open: port: %d\n", port->line); */

        ccm2200_at91_uart_enable_pins(usart_num);

        /* route RXD/TXD from debug USART to CTS/RTS from USART0 as long
         * the corresponding device is opened
         */
        if (usart_num ==  AT91_USART_DBG ) {
                at91_config_dbg_uart_pins( AT91_USART_DBG_RX_TX );
        }

        {
                register struct usart_mctrl_pins *mctrl_pins =
                        &ccm2200_mctrl_cfg[usart_num];
                at91_pio_set_all_pins(&mctrl_pins->dtr);
                at91_pio_config_output_pins(&mctrl_pins->dtr);
                at91_pio_config_input_pins(&mctrl_pins->dcd);
                at91_pio_config_input_pins(&mctrl_pins->dsr);
                at91_pio_config_input_pins(&mctrl_pins->ri);
        }
        return 0;
};

static void ccm2200_at91_uart_close(struct uart_port *port)
{
        //printk("ccm2200_at91_uart_close: port: %d\n", port->line);
        /* restore routing of USART0 CTS/RTS */
        if (at91_get_usart(port) ==  AT91_USART_DBG ) {
                at91_config_dbg_uart_pins( AT91_USART2_RTS_CTS );
        }
}

struct atmel_port_fns __initdata ccm2200_at91_port_fns = {
        .set_mctrl = ccm2200_at91_set_mctrl,
        .get_mctrl = ccm2200_at91_get_mctrl,
        .enable_ms = ccm2200_at91_enable_ms,
        .open      = ccm2200_at91_uart_open,
        .close     = ccm2200_at91_uart_close

};


int __init ccm2200_at91_uart_init(void)
{
        /* Setup the serial ports and console */
	at91_init_serial(&ccm2200_uart_config);

	/* register UARTs */
        atmel_register_uart_fns(&ccm2200_at91_port_fns);

        /* confige modem control pins to USART2 the are reconfigured to
         * the DEBUG USART if the coresponding device is opened
         */
        at91_config_dbg_uart_pins(AT91_USART2_RTS_CTS);
        printk("CCM2200 AT91RM9200 internal UARTs successfully initialized\n");
        return 0;
}


static void __exit ccm2200_at91_uart_exit(void)
{
}

/*
 * we musst call the init function in board-ccm2200.c to use the UART
 * as serial console!
 * module_init(ccm2200_at91_uart_init);
 * module_exit(ccm2200_at91_uart_exit);
 */

MODULE_AUTHOR("Guido Classen");
MODULE_DESCRIPTION("CCM2200 AT91RM9200 UART subclass driver");
MODULE_LICENSE("GPL");

#endif /* CONFIG_SERIAL_AT91 */

/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */
