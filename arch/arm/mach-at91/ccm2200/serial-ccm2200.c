/*
 * linux/arch/arm/mach-at91rm9200/serial-ccm2200.c
 *
 * Copyright (C) 2007 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      CCM2200 Board specific serial port functions
 *              (LED and RS485 support)
 * @references:
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

#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/serial_core.h>
#include <linux/ccm2200_serial.h>

#include <asm/arch/hardware.h>
#include <asm/uaccess.h>		/* copy_to_user */
#include "../../../../drivers/serial/atmel_serial.h"
#include <asm/arch/board.h>

#include <asm/arch/board-ccm2200.h>


#define LED_INTERVAL   msecs_to_jiffies(30)
static void ccm2200_board_serial_led_stop(struct ccm2200_led_handler *led);
static int ccm2200_board_serial_ioctl_set_led(struct ccm2200_led_handler *led,
                                              unsigned long arg);
static void ccm2200_board_serial_turn_on_off_timer_handler(unsigned long data);

/******************************************************************************
 * CCM2200 RS485 piggyback support
 *****************************************************************************/

#define UART_PUT_CR(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_CR)
#define UART_GET_MR(port)	__raw_readl((port)->membase + ATMEL_US_MR)
#define UART_PUT_MR(port,v)	__raw_writel(v, (port)->membase + ATMEL_US_MR)

/* turn on piggyback RS485 transmitter */
static inline
void ccm2200_board_serial_turn_on(register struct uart_port *port)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
        struct uart_info *info = port->info;
        if (se->conf.mode != CCM2200_SERIAL_MODE_RS485KERN_NEG) {
          se->orig_ops->set_mctrl(port, TIOCM_RTS | port->mctrl);
        } else {
          se->orig_ops->set_mctrl(port, ~TIOCM_RTS & port->mctrl);
        }
        /* wait for CTS in send mode */
/*         info->flags |= UIF_CTS_FLOW; */
}

/* turn off piggyback RS485 transmitter */
static inline
void ccm2200_board_serial_turn_off(register struct uart_port *port)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
        struct uart_info *info = port->info;

        /* ignore CTS in receive mode */
        info->flags &= ~UIF_CTS_FLOW & ~UIF_CHECK_CD;
        info->tty->hw_stopped = 0;

        if (se->conf.mode != CCM2200_SERIAL_MODE_RS485KERN_NEG) {
          se->orig_ops->set_mctrl(port, ~TIOCM_RTS & port->mctrl);
        } else {
          se->orig_ops->set_mctrl(port, TIOCM_RTS | port->mctrl);
        }
        se->state = CCM2200_BS_RECEIVE;
}


static void ccm2200_board_serial_at91_rs485_mode(struct uart_port *port,
                                                int enable)
{
        unsigned mode = UART_GET_MR(port);

        mode &= ~ATMEL_US_USMODE;
        if (enable) {
                mode |= ATMEL_US_USMODE_RS485;
        } else {
                mode |= ATMEL_US_USMODE_NORMAL;
        }
        UART_PUT_MR(port, mode);
	UART_PUT_CR(port, ATMEL_US_RSTSTA | ATMEL_US_RSTRX);
	UART_PUT_CR(port, ATMEL_US_TXEN | ATMEL_US_RXEN);
}



static
int ccm2200_board_serial_chg_conf(struct uart_port *port,
                                  struct ccm2200_serial_config *new_conf)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;

        if (new_conf->mode != se->conf.mode) {
                /* mode was changed */

                /* switch of HW RS485 mode if previously enabled */
                if (se->conf.mode == CCM2200_SERIAL_MODE_RS485HW)
                        ccm2200_board_serial_at91_rs485_mode(port, 0);

                switch (new_conf->mode) {
                case CCM2200_SERIAL_MODE_NORMAL:
                        if (new_conf->turn_on_delay != 0 ||
                            new_conf->turn_off_delay != 0) {
                                /* turn on/off delay not support in this mode*/
                                return -EINVAL;
                        }
                        break;

                case CCM2200_SERIAL_MODE_RS485HW: {
                        /* this mode is only supported on ports from
                         * ATMEL AT91 UART */
                        struct tty_driver *driver = port->info->tty->driver;
                        if (new_conf->turn_on_delay != 0 ||
                            new_conf->turn_off_delay != 0) {
                                /* turn on/off delay not support in this mode*/
                                return -EINVAL;
                        }
                        if (!strcmp(driver->driver_name, "ttyS")
                            || !strcmp(driver->driver_name, "at91")
                            || !strcmp(driver->driver_name, "atmel_serial")) {

                                ccm2200_board_serial_at91_rs485_mode(port, 1);

                                break;

                        } else {
                                printk(KERN_WARNING
                                       "CCM2200_SERIAL_MODE_RS485HW "
                                       "not supported on "
                                       "this device, using "
                                       "CCM2200_SERIAL_MODE_RS485KERN\n");
                                new_conf->mode =
                                        CCM2200_SERIAL_MODE_RS485KERN;
                        }
                        /* no break */
                }

                case CCM2200_SERIAL_MODE_RS485KERN:
                case CCM2200_SERIAL_MODE_RS485KERN_NEG:
                case CCM2200_SERIAL_MODE_MODEM_MD:
                case CCM2200_SERIAL_MODE_MODEM_MD_DCD:
                        /* clear RTS, it will be set from now on by sending
                         * RS485 data
                         */
                        ccm2200_board_serial_turn_off(port);
                        break;

                case CCM2200_SERIAL_MODE_RS485INT:
                        return -ENOIOCTLCMD;

                default:
                        return -EFAULT;
                }
        }

        new_conf->turn_on_delay = msecs_to_jiffies(new_conf->turn_on_delay);
        new_conf->turn_off_delay = msecs_to_jiffies(new_conf->turn_off_delay);
        memcpy(&se->conf, new_conf, sizeof(se->conf));

        return 0;
}


/* this array contains a list of all port currently sending .
 * We wait for end of transmission and subsequently turn off RTS
 */
/*
 * #define MAX_TX_PORTS    10
 * static struct uart_port *volatile tx_ports[MAX_TX_PORTS];
 * static volatile unsigned tx_ports_size = 0;
 * static spinlock_t tx_ports_lock;
 * static struct timer_list tx_timer;
 */

/*
 * static inline void add_tx_port(struct uart_port *port)
 * {
 *	unsigned long flags;
 *         unsigned old_tx_ports_size;
 *
 * /\*         printk("add_tx_port(%d)\n", tx_ports_size); *\/
 *
 *	spin_lock_irqsave(&tx_ports_lock, flags);
 *         old_tx_ports_size = tx_ports_size;
 *         if (tx_ports_size < MAX_TX_PORTS) {
 *                 tx_ports[tx_ports_size++] = port;
 *         }
 *
 *	spin_unlock_irqrestore(&tx_ports_lock, flags);
 *         if (old_tx_ports_size == 0) {
 *                 //printk("add_tx_port(add_timer)\n");
 *                 tx_timer.expires = jiffies + 1;
 *                 add_timer(&tx_timer);
 *         }
 * }
 */

static inline
void ccm2200_board_serial_on_tx_empty(register struct uart_port *port)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;

/*
  achtung Ausgabe braucht zu lange!!!
  printk("ccm2200_board_serial_on_tx_empty(%d)\n", port->line); */

        if (se->conf.turn_off_delay > 0) {
                /* we have a configured followup delay,
                 * so defer resetting RTS
                 */
                se->state = CCM2200_BS_TURN_OFF_DELAY;
                mod_timer(&se->turn_on_off_timer,
                          jiffies + se->conf.turn_off_delay);

        } else {
                /* no followup delay, immediately reset RTS */
                ccm2200_board_serial_turn_off(port);
        }
}

static
void ccm2200_board_serial_goto_turn_on_state(struct uart_port *port)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;

        ccm2200_board_serial_turn_on(port);

        se->state = CCM2200_BS_TURN_ON_DELAY;
        if (se->conf.turn_on_delay > 0) {
                /* start turn-on delay timer */
                mod_timer(&se->turn_on_off_timer,
                          jiffies + se->conf.turn_on_delay);
        } else {
                ccm2200_board_serial_turn_on_off_timer_handler(
                  (unsigned long) port);
        }
}

static
void ccm2200_board_serial_change_to_transmit_state(struct uart_port *port)
{
  register struct ccm2200_board_serial *se = port->ccm2200_serial;
  register struct uart_info *info = port->info;
  se->state = CCM2200_BS_TRANSMIT;
  /* reread current CTS status
   *
   * uart_handle_cts_change will call start_tx() if
   * we are ready to send
   */
  info->tty->hw_stopped = 0;
  se->orig_ops->start_tx(port);

}

static void ccm2200_board_serial_turn_on_off_timer_handler(unsigned long data)
{
        register struct uart_port *port = (struct uart_port *)data;
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
                switch (se->state) {
                case CCM2200_BS_DCD_WAIT: {
                        /* wait for absent carrier detect (only
                         * Mulit-drop modem mode) */
                        if ((se->orig_ops->get_mctrl(port) & TIOCM_CD) == 0) {
                                ccm2200_board_serial_goto_turn_on_state(port);
                        } else {
                                mod_timer(&se->turn_on_off_timer, jiffies + 1);
                        }
                        return;
                }

                case CCM2200_BS_TURN_ON_DELAY:
/*                         printk("turn on timer expired\n"); */

                  if (se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN
                      || se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN_NEG) {
                    ccm2200_board_serial_change_to_transmit_state(port);
                    return;
                  } else {

                    se->state = CCM2200_BS_CTS_WAIT;
                  }
                    /* no break */

                case CCM2200_BS_CTS_WAIT: {
                        /* wait for CTS control signal */

                        struct uart_info *info = port->info;
                        if (se->orig_ops->get_mctrl(port) & TIOCM_CTS) {
                                //port->ops->enable_ms(port);
                          ccm2200_board_serial_change_to_transmit_state(port);
                          return;
/*                         uart_handle_cts_change(port, */
/*                                                port->ops->get_mctrl(port) */
/*                                                & TIOCM_CTS); */
                        } else {
                                mod_timer(&se->turn_on_off_timer, jiffies + 1);
                        }
                        return;
                }


                case CCM2200_BS_TURN_OFF_DELAY:
                        se->state = CCM2200_BS_TRANSMIT;
                        ccm2200_board_serial_turn_off(port);
                        return;

                case CCM2200_BS_RECEIVE:
                        break;

                case CCM2200_BS_TRANSMIT:
                        if (!port->info->tty->hw_stopped
                            && port->ops->tx_empty(port) != 0) {
                                ccm2200_board_serial_on_tx_empty(port);
                        } else {
                                mod_timer(&se->turn_on_off_timer, jiffies + 1);
                        }
                        break;

                default:
                        ;
        }

}

void ccm2200_board_serial_rs485_tx(struct uart_port *port)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;

        switch (se->conf.mode) {
        case CCM2200_SERIAL_MODE_RS485KERN:
        case CCM2200_SERIAL_MODE_RS485KERN_NEG:
        case CCM2200_SERIAL_MODE_MODEM_MD:
        case CCM2200_SERIAL_MODE_MODEM_MD_DCD:


/*                 printk("rs485_tx\n"); */

          /* add to timer list to check for transmitter empty */
          //add_tx_port(port);

          se->state = CCM2200_BS_TRANSMIT;
          mod_timer(&se->turn_on_off_timer, jiffies + 1);

        default:
          ;
        }
}



/* uart_ops hook / filter functions */
static
void ccm2200_board_serial_start_tx(struct uart_port *port)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
        if (se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN
            || se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN_NEG
            || se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD
            || se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD_DCD) {
/*                 printk("ccm2200_board_serial_start_tx(%d) state=%d\n",  */
/*                        port->line, */
/*                        se->state); */

                switch (se->state) {
                case CCM2200_BS_RECEIVE: {
                        struct uart_info *info = port->info;
                        if (se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD_DCD) {
                                struct uart_info *info = port->info;
                                se->state = CCM2200_BS_DCD_WAIT;

                                /* stop transmitter */
                                info->tty->hw_stopped = 1;
                                se->orig_ops->stop_tx(port);
                                ccm2200_board_serial_turn_on_off_timer_handler(
                                        (unsigned long) port);
                                break;

                        } else {
                                ccm2200_board_serial_goto_turn_on_state(port);
                                return;
                        }
                }

                case CCM2200_BS_TURN_ON_DELAY:
                        return;  /* wait for timer */


                case CCM2200_BS_TURN_OFF_DELAY:
                        del_timer_sync(&se->turn_on_off_timer);
                        se->state = CCM2200_BS_TRANSMIT;
                        break;

                case CCM2200_BS_TRANSMIT:
                        break;

                default:
                        ;
                }
        }
        se->orig_ops->start_tx(port);
}

static
void ccm2200_board_serial_stop_tx(struct uart_port *port)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
/*         printk("ccm2200_board_serial_stop_tx(%d)\n", port->line); */
        se->orig_ops->stop_tx(port);
        if (se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN
            || se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN_NEG
            || se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD
            || se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD_DCD) {
                ccm2200_board_serial_turn_off(port);
        }
}


static
void ccm2200_board_serial_set_mctrl(struct uart_port *port,
                                    unsigned int mctrl)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
        if (se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN
            || se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD
            || se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD_DCD) {
                /* make shure application can't change our turn on signal! */
                if (se->state == CCM2200_BS_RECEIVE)
                        mctrl &= ~TIOCM_RTS;
                else
                        mctrl |= TIOCM_RTS;
        }
        if (se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN_NEG) {
          /* make shure application can't change our turn on signal! */
          if (se->state == CCM2200_BS_RECEIVE)
            mctrl |= TIOCM_RTS;
          else
            mctrl &= ~TIOCM_RTS;
        }
        se->orig_ops->set_mctrl(port, mctrl);
}

static
void ccm2200_board_serial_set_termios(struct uart_port *port,
                                      struct ktermios *new,
                                      struct ktermios *old)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
        se->orig_ops->set_termios(port, new, old);
        if (se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN
            || se->conf.mode == CCM2200_SERIAL_MODE_RS485KERN_NEG
            || se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD
            || se->conf.mode == CCM2200_SERIAL_MODE_MODEM_MD_DCD) {
                ccm2200_board_serial_turn_off(port);
        }
}


int ccm2200_board_serial_startup(struct uart_port *port)
{
/*         printk("serial startup %d\n", port->line); */
        return port->ccm2200_serial->orig_ops->startup(port);
}


/*
 * static void ccm2200_board_serial_tx_timer_handler(unsigned long data)
 * {
 *         struct uart_port *volatile  *iter;
 *	unsigned long flags;
 *
 *	spin_lock_irqsave(&tx_ports_lock, flags);
 *         iter = tx_ports;
 *         while (iter < tx_ports + tx_ports_size) {
 *                 /\* if hw_stopped is set, we wait for CTS... *\/
 *                 if (!(*iter)->info->tty->hw_stopped
 *                       && (*iter)->ops->tx_empty(*iter) != 0) {
 *                         ccm2200_board_serial_on_tx_empty(*iter);
 *                         /\* remove from list *\/
 *                         --tx_ports_size;
 *                         /\* 2007-06-05 gc: bugfix: memcpy -> memmove *\/
 *                         memmove((void*)iter, (void*)(iter+1),
 *                                sizeof(struct uart_port *) *
 *                                (tx_ports_size - (iter - tx_ports)));
 *                 } else {
 *                         ++iter;
 *                 }
 *         }
 *	spin_unlock_irqrestore(&tx_ports_lock, flags);
 *         if (tx_ports_size > 0)
 *                 mod_timer(&tx_timer, jiffies + 1);
 * }
 */


/******************************************************************************
 * CCM2200 serial RX / TX indicator LED support
 *****************************************************************************/

static void ccm2200_board_serial_led_timer_handler(unsigned long data);

static void ccm2200_board_serial_led_init(struct ccm2200_led_handler *led)
{
        led->led.mask = 0;
        led->led.delay = LED_INTERVAL;
        init_timer(&led->led_timer);
	led->led_timer.function = ccm2200_board_serial_led_timer_handler;
	led->led_timer.data = (unsigned long) led;
        led->led_timer.expires = jiffies + led->led.delay;

}


static int ccm2200_board_serial_ioctl_set_led(struct ccm2200_led_handler *led,
                                              unsigned long arg)
{
        ccm2200_board_serial_led_stop(led);
        if (copy_from_user(&led->led,
                           (struct ccm2200_serial_led *)arg,
                           sizeof(led->led)))
                return -EFAULT;
        ;

        if (led->led.delay == 0) {
                led->led.delay = LED_INTERVAL;
        }
        return 0;

}


static void ccm2200_board_serial_led_stop(struct ccm2200_led_handler *led)
{
        if (led->led.mask != 0) {
                del_timer_sync(&led->led_timer);

                /* switch off led */
                ccm2200_set_frontpanel_leds(0, led->led.mask);
        }
}

static void ccm2200_board_serial_led_timer_handler(unsigned long data)
{
        register struct ccm2200_led_handler *led
                = (struct ccm2200_led_handler *)data;
        /* clear led */
        ccm2200_set_frontpanel_leds(0, led->led.mask);
}

void ccm2200_board_serial_trigger_led(struct ccm2200_led_handler *led)
{
        if (led->led.mask != 0 && led->led.delay != 0) {
                /* switch on led */
                ccm2200_set_frontpanel_leds(led->led.mask, led->led.mask);

                mod_timer(&led->led_timer, jiffies + led->led.delay);
        }
}


/******************************************************************************
 * miscellaneous CCM2200 serial driver hook code
 *****************************************************************************/

int ccm2200_board_serial_ioctl(struct uart_port *port,
                               unsigned int cmd,
                               unsigned long arg)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
/*         printk("ccm2200_board_serial_ioctl: cmd: %d\n", cmd); */

        switch (cmd) {
        case CCM2200_SERIAL_GET_CONF: {
                struct ccm2200_serial_config new_conf;
                memcpy(&new_conf, &se->conf, sizeof(se->conf));
                new_conf.turn_on_delay
                        = jiffies_to_msecs(new_conf.turn_on_delay);
                new_conf.turn_off_delay
                        = jiffies_to_msecs(new_conf.turn_off_delay);

                if (copy_to_user((struct ccm2200_serial_config *)arg,
                                 &new_conf, sizeof(new_conf)))
                        return -EFAULT;
                break;
        }

        case CCM2200_SERIAL_SET_CONF: {
                struct ccm2200_serial_config new_conf;
                int result;

                if (copy_from_user(&new_conf,
                                   (struct ccm2200_serial_config *)arg,
                                   sizeof(new_conf)))
                        return -EFAULT;

                result = ccm2200_board_serial_chg_conf(port, &new_conf);
                if (result != 0)
                        return result;

                break;
        }
        case CCM2200_SERIAL_GET_TX_LED:
                if (copy_to_user((struct ccm2200_serial_led *)arg,
                                 &se->txLed.led, sizeof(se->txLed.led)))
                        return -EFAULT;
                break;

        case CCM2200_SERIAL_SET_TX_LED:
                return ccm2200_board_serial_ioctl_set_led(&se->txLed, arg);

        case CCM2200_SERIAL_GET_RX_LED:
                if (copy_to_user((struct ccm2200_serial_led *)arg,
                                 &se->rxLed.led, sizeof(se->rxLed.led)))
                        return -EFAULT;
                break;

        case CCM2200_SERIAL_SET_RX_LED:
                return ccm2200_board_serial_ioctl_set_led(&se->rxLed, arg);

        default:
                return -ENOIOCTLCMD;
        }
        return 0;
}


void ccm2200_board_serial_init(struct uart_port *port,
                               struct ccm2200_board_serial *se)
{
        if (se->state != CCM2200_BS_RECEIVE) {
                memset(se, 0, sizeof(*se));
                se->conf.mode = CCM2200_SERIAL_MODE_NORMAL;
                se->conf.turn_on_delay = 0;
                se->conf.turn_off_delay = 0;
                ccm2200_board_serial_led_init(&se->rxLed);
                ccm2200_board_serial_led_init(&se->txLed);
                port->ccm2200_serial = se;

                /* alloc and hang in subclassed uart_ops structure */
                memcpy(&se->ccm2200_ops, port->ops,
                       sizeof(se->ccm2200_ops));

                se->ccm2200_ops.start_tx = ccm2200_board_serial_start_tx;
                se->ccm2200_ops.stop_tx = ccm2200_board_serial_stop_tx;
                se->ccm2200_ops.set_mctrl = ccm2200_board_serial_set_mctrl;
                se->ccm2200_ops.set_termios = ccm2200_board_serial_set_termios;
                se->ccm2200_ops.startup = ccm2200_board_serial_startup;

                se->orig_ops = port->ops;
/*                 printk("----gc 2 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXx\n"); */
                port->ops = &se->ccm2200_ops;
                init_timer(&se->turn_on_off_timer);
                se->turn_on_off_timer.function = ccm2200_board_serial_turn_on_off_timer_handler;
                se->turn_on_off_timer.data = (unsigned long) port;

                se->state = CCM2200_BS_RECEIVE;
        }
}


void ccm2200_board_serial_remove(struct uart_port *port)
{
        register struct ccm2200_board_serial *se = port->ccm2200_serial;
        if (se->state != CCM2200_BS_NOT_INITIALIZED) {

                del_timer_sync(&se->rxLed.led_timer);
                del_timer_sync(&se->txLed.led_timer);
                se->state = CCM2200_BS_NOT_INITIALIZED;
                port->ops = se->orig_ops;
        }
}


static int __init ccm2200_serial_init(void)
{
	/*
         * spin_lock_init(&tx_ports_lock);
         * tx_ports_size = 0;
         */

        /*
         * init_timer(&tx_timer);
	 * tx_timer.function = ccm2200_board_serial_tx_timer_handler;
	 * tx_timer.data = (unsigned long) 0;
         * tx_timer.expires = jiffies + 1;
         */

        printk("CCM2200 board specific serial handling enabled\n");

        return 0;
}

static void __exit ccm2200_serial_exit(void)
{
}


module_init(ccm2200_serial_init);
module_exit(ccm2200_serial_exit);

MODULE_AUTHOR("Guido Classen");
MODULE_DESCRIPTION("CCM2200 serial LED and piggyback handling ");
MODULE_LICENSE("GPL");

/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */
