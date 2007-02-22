/*
 * Serial driver module for ESS modems.
 *
 *  Copyright (C) 2005 Glauber de Oliveira Costa,
 *                     Gustavo Sverzut Barbieri,
 *                     Murillo Fernandes Bernardes,
 *                     Robert H. Thornburrow
 *  Copyright (C) 2006 Jeffrey E. Trull
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * based on ptserial-2.6.c by Gustavo Barbieri, which was in turn based on work
 * by Robert Thornburrow and others for 2.4 and prior kernels,
 * and finally the 2.6 kernel's serial driver code
 *
 * Valuable knowledge came from studying a "hacked" version of the ESS 2.2 driver
 * for kernel 2.4 by Shachar Raindel
 */

#include <linux/version.h>

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial.h>

#include <asm/io.h>
#include <asm/irq.h>

#include "linmodem-2.6.h"
#include "essserial.h"

#include "include/ess.h"
#include "include/vuart.h"


#define DRIVER_VERSION "v0.3"
#define DRIVER_AUTHOR  "Jeff Trull <linmodemstudent@gmail.com>"
#define DRIVER_DESC    "ESS Tech ES2898 modem driver"
#define DRIVER_LICENSE "GPL"

#define MY_NAME "ess"

#ifdef ESS_DEBUG
extern int debug;

#define dbg(fmt, arg...)						    \
	do {								    \
		if (debug)						    \
			printk (KERN_DEBUG "%s(%d): %s: " fmt "\n",	    \
				MY_NAME , __LINE__, __FUNCTION__ , ## arg); \
	} while (0)
#else  /* ESS_DEBUG */
#define dbg(fmt, arg...) do { } while (0)
#endif /* ESS_DEBUG */

#define err(format, arg...) \
	printk(KERN_ERR "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)
#define info(format, arg...) \
	printk(KERN_INFO "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)


static int country_code = 0;
module_param(country_code, int, 0);
MODULE_PARM_DESC(country_code, "Select a country code for the ESS modem.");


/* get_uart is array [10] of function pointers to functions taking void
   and returning unsigned char */
static unsigned char (*get_uart[10])(void) =
{
	get_uart_rx,
	get_uart_ier,
	get_uart_iir,
	get_uart_lcr,
	get_uart_mcr,
	get_uart_lsr,
	get_uart_msr,
	get_uart_scr,
	get_uart_dll,
	get_uart_dlm
};

/* put_uart is array [10] of function pointers to functions taking unsigned
   char and returning void */
static void (* put_uart[10])(unsigned char) =
{
	put_uart_tx,
	put_uart_ier,
	put_uart_fcr,     /* IIR on read, FCR on write */
	put_uart_lcr,
	put_uart_mcr,
	put_uart_lsr,
	put_uart_msr,
	put_uart_scr,
	put_uart_dll,
	put_uart_dlm
};


static unsigned int ess_serial_in(struct linmodem_port *p, int offset)
{

    if ((get_uart_lcr() & UART_LCR_DLAB) && (offset <= UART_DLM))
		offset += (UART_SCR+1);

	return (*get_uart[offset])();
}

static void ess_serial_out(struct linmodem_port *p, int offset, int value)
{
    if ((get_uart_lcr() & UART_LCR_DLAB) && (offset <= UART_DLM)) {
	offset += (UART_SCR+1);
    }
    (*put_uart[offset])(value);
}

#define ess_serial_inp(port, offset)         ess_serial_in(port, offset)
#define ess_serial_outp(port, offset, value) ess_serial_out(port, offset, value)

/* XXX: BEGIN HACK, remove these later! */
struct irq_info {
	spinlock_t		lock;
	struct list_head	*head;
};
#define is_real_interrupt(irq)	((irq) != 0)
/* XXX: END */

unsigned long last_int_report = 0;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) )
static irqreturn_t  ess_interrupt(int irq, void *dev_id,
				    struct pt_regs *regs )
#else
static irqreturn_t  ess_interrupt(int irq, void *dev_id)
#endif

{
	static union i387_union i387;
	static unsigned long cr0;
	static unsigned long flags;
	struct irq_info *i = dev_id;
	struct linmodem_port *up;

	dbg();

	up = list_entry(i->head, struct linmodem_port, list);

	spin_lock_irqsave(&up->port.lock, flags);

	/* save fpu state */
	__asm__ __volatile__("mov %%cr0,%0 ; clts" : "=r" (cr0));
	__asm__ __volatile__("fnsave %0\n\t"
			     "fwait"
			     : "=m" (i387));

	/* ESS internal interrupt handling */
	esscom_hw_interrupt();

	__asm__ __volatile__("frstor %0": :"m" (i387));
	__asm__ __volatile__("mov %0,%%cr0" : : "r" (cr0));

	spin_unlock_irqrestore(&up->port.lock, flags);

	return IRQ_RETVAL(1);

}

static void ess_autoconfig(struct linmodem_port *p, unsigned int probeflags)
{
	dbg();

	if (p->port.line == ESS_TTY_LINE)
	{
		p->port.type     = PORT_ESS;
		p->port.fifosize = p->config.fifo_size;
		p->tx_loadsz     = 128;

		return;
	}
}

unsigned long last_report = 0;

void esscom_do_timer_tick (unsigned long data) {

    struct linmodem_port *p;
    static union i387_union i387;
    static unsigned long cr0;

    p = (struct linmodem_port *) data;

    /* save fpu state */
    __asm__ __volatile__("mov %%cr0,%0 ; clts" : "=r" (cr0));
    __asm__ __volatile__("fnsave %0\n\t"
			 "fwait"
			 : "=m" (i387));

    esscom_hw_timer_tick();

    __asm__ __volatile__("frstor %0": :"m" (i387));
    __asm__ __volatile__("mov %0,%%cr0" : : "r" (cr0));

    linmodem_handle_port(p, (struct pt_regs *)NULL);

    /* reset timer for 10ms from now */
    mod_timer(&p->timer, jiffies + HZ / 100);

}

static int ess_startup(struct linmodem_port *p)
{
	unsigned long flags;
	int retval;

	dbg();

	p->capabilities = p->config.flags;
	p->mcr = 0;

	info("ESS initialization. Country code is %d.\n", country_code);

	esscom_hw_setup(p->port.iobase, p->port.irq, country_code);

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reeanbled in set_termios())
	 */
	linmodem_clear_fifos(p);

	/*
	 * Clear the interrupt registers.
	 */
	ess_serial_inp(p, UART_LSR);
	ess_serial_inp(p, UART_RX);
	ess_serial_inp(p, UART_IIR);
	ess_serial_inp(p, UART_MSR);

	/*
	 * At this point, there's no way the LSR could still be 0xff;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (!(p->port.flags & UPF_BUGGY_UART) &&
	    (ess_serial_inp(p, UART_LSR) == 0xff)) {
		info("%s%d: LSR safety check engaged!",
		     p->reg.dev_name, p->port.line);
		return -ENODEV;
	}


	/* ESS has both an interrupt handler and a timer */
	/* IRQ first */
	retval = serial_link_irq_chain(p);
	if (retval)
	    return retval;

	/* now the timer */
	init_timer(&p->timer);
	p->timer.function = esscom_do_timer_tick;
	p->timer.data = (unsigned long)p;
	mod_timer(&p->timer, jiffies + 40 * (HZ / 100));

	/*
	 * Now, initialize the UART
	 */
	ess_serial_outp(p, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&p->port.lock, flags);
	/*
	 * Most PC uarts need OUT2 raised to enable interrupts.
	 */
	if (is_real_interrupt(p->port.irq))
		p->port.mctrl |= TIOCM_OUT2;

	linmodem_set_mctrl((struct uart_port *)p, p->port.mctrl);
	spin_unlock_irqrestore(&p->port.lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	p->ier = UART_IER_RLSI | UART_IER_RDI ;
	ess_serial_outp(p, UART_IER, p->ier);

	/*
	 * And clear the interrupt registers again for luck.
	 */
	ess_serial_inp(p, UART_LSR);
	ess_serial_inp(p, UART_RX);
	ess_serial_inp(p, UART_IIR);
	ess_serial_inp(p, UART_MSR);

	return 0;
}

static void ess_shutdown(struct linmodem_port *p)
{
	unsigned long flags;

	dbg();

	/* tell the hardware to stop issuing interrupts and hang up */
	esscom_hw_shutdown();

	/*
	 * Disable interrupts from this port
	 */
	p->ier = 0;
	ess_serial_outp(p, UART_IER, 0);

	spin_lock_irqsave(&p->port.lock, flags);
	if (p->port.flags & UPF_FOURPORT) {
		/* reset interrupts on the AST Fourport board */
		inb((p->port.iobase & 0xfe0) | 0x1f);
		p->port.mctrl |= TIOCM_OUT1;
	} else
		p->port.mctrl &= ~TIOCM_OUT2;

	linmodem_set_mctrl((struct uart_port *)p, p->port.mctrl);
	spin_unlock_irqrestore(&p->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	ess_serial_out(p, UART_LCR,
			 ess_serial_inp(p, UART_LCR) & ~UART_LCR_SBC);
	linmodem_clear_fifos(p);

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	ess_serial_in(p, UART_RX);

	del_timer_sync(&p->timer);
	serial_unlink_irq_chain(p);

}




struct linmodem_ops ess_ops = {
	.serial_in  = ess_serial_in,
	.serial_out = ess_serial_out,
	.interrupt  = ess_interrupt,
	.autoconfig = ess_autoconfig,
	.startup    = ess_startup,
	.shutdown   = ess_shutdown,
};

struct linmodem_port ess_port = {
	.port = {},
	.ops  = &ess_ops,
	.config = {
		.name      = "Ess",
		.fifo_size = 128,
		.flags     = 0,
	},
	.reg = {
		.owner = THIS_MODULE,
		.driver_name = "ess",
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) )
		.devfs_name  = "ttess/",
#endif
		.dev_name    = "ttyS_ESS",
		.major       = ESS_MAJOR,
		.minor       = ESS_MINOR,
		.nr          = 1,
	},
};


int ess_init(void)
{
	dbg();

    info( MY_NAME " " DRIVER_VERSION " loaded" );

	return linmodem_new(&ess_port);
}

void ess_exit(void)
{
	dbg();

    info( MY_NAME " " DRIVER_VERSION " unloaded" );

	linmodem_del(&ess_port);
}



int ess_new(struct uart_port *port)
{
	struct uart_port *p = &(ess_port.port);
	dbg();

	p->iobase   = port->iobase;
	p->membase  = port->membase;
	p->irq      = port->irq;
	p->uartclk  = port->uartclk;
	p->fifosize = port->fifosize;
	p->regshift = port->regshift;
	p->iotype   = port->iotype;
	p->flags    = port->flags | UPF_BOOT_AUTOCONF;
	p->mapbase  = port->mapbase;

	return linmodem_hw_new((struct uart_port*)&ess_port);
}

EXPORT_SYMBOL(ess_new);
