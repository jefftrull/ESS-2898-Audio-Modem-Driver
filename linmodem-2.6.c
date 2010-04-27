/*
 *  Based on linux/drivers/serial/8250.c
 *
 *  Driver for LinModem - Serial Software Modems
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *  Copyright (C) 2005 Glauber de Oliveira Costa,
 *                     Gustavo Sverzut Barbieri,
 *                     Murillo Fernandes Bernardes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 * A note about mapbase / membase
 *
 *  mapbase is the physical address of the IO port.
 *  membase is an 'ioremapped' cookie.
 */
#include <linux/version.h>

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mca.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial.h>

#include "linmodem-2.6.h"

#include <asm/io.h>
#include <asm/irq.h>

#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR  "Gustavo Sverzut Barbieri <barbieri@gmail.com>"
#define DRIVER_DESC    "Generic Linux Software Modem TTY Layer"
#define DRIVER_LICENSE "GPL"

/* Module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);


#define MY_NAME "linmodem"

#ifdef LINMODEM_DEBUG
static int debug = 1;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debugging mode enabled or not");

#define dbg(fmt, arg...)						    \
	do {								    \
		if (debug)						    \
			printk (KERN_DEBUG "%s(%d): %s: " fmt "\n",	    \
				MY_NAME , __LINE__, __FUNCTION__ , ## arg); \
	} while (0)
#else  /* LINMODEM_DEBUG */
#define dbg(fmt, arg...) do { } while (0)
#endif /* LINMODEM_DEBUG */

#define err(format, arg...) \
	printk(KERN_ERR "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)
#define info(format, arg...) \
	printk(KERN_INFO "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)



static DECLARE_MUTEX(linmodem_sem);

struct linmodem_port_list {
	struct list_head     list;
	struct linmodem_port *port;
};
struct linmodem_port_list linmodem_ports;


/*
 * Configuration:
 *   share_irqs - whether we pass SA_SHIRQ to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
unsigned int share_irqs = 0;

spinlock_t driver_lock = SPIN_LOCK_UNLOCKED;


/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)	((irq) != 0)

/*
 * This converts from our new CONFIG_ symbols to the symbols
 * that asm/serial.h expects.  You _NEED_ to comment out the
 * linux/config.h include contained inside asm/serial.h for
 * this to work.
 */
#undef CONFIG_SERIAL_DETECT_IRQ

#ifdef CONFIG_SERIAL_8250_DETECT_IRQ
#define CONFIG_SERIAL_DETECT_IRQ 1
#endif

/*
 * HUB6 is always on.  This will be removed once the header
 * files have been cleaned.
 */
#define CONFIG_HUB6 1

#include <asm/serial.h>

/*
 * SERIAL_PORT_DFNS tells us about built-in ports that have no
 * standard enumeration mechanism.   Platforms that can find all
 * serial ports via mechanisms like ACPI or PCI need not supply it.
 */
#ifndef SERIAL_PORT_DFNS
#define SERIAL_PORT_DFNS
#endif

struct irq_info {
	spinlock_t		lock;
	struct list_head	*head;
};

static struct irq_info irq_lists[NR_IRQS];


static /* inline */ unsigned int serial_in(struct linmodem_port *p, int offset)
{
	int ret;

	ret = p->ops->serial_in(p, offset);
	return ret;
}

static /* inline */ void serial_out(struct linmodem_port *p, int offset,
				  int value)
{
	p->ops->serial_out(p, offset, value);
}

/*
 * We used to support using pause I/O for certain machines.  We
 * haven't supported this for a while, but just in case it's badly
 * needed for certain old 386 machines, I've left these #define's
 * in....
 */
#define serial_inp(up, offset)		serial_in(up, offset)
#define serial_outp(up, offset, value)	serial_out(up, offset, value)


/*
 * For the 16C950
 */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17) )
static void serial_icr_write(struct linmodem_port *p, int offset, int value)
{
	dbg();
	serial_out(p, UART_SCR, offset);
	serial_out(p, UART_ICR, value);
}
#endif

/*
 * FIFO support.
 */
void linmodem_clear_fifos(struct linmodem_port *p)
{
	dbg();
	if (p->capabilities & UART_CAP_FIFO) {
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			    UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_outp(p, UART_FCR, 0);
	}
}
EXPORT_SYMBOL(linmodem_clear_fifos);

/*
 * IER sleep support.  UARTs which have EFRs need the "extended
 * capability" bit enabled.  Note that on XR16C850s, we need to
 * reset LCR to write to IER.
 */
static inline void linmodem_set_sleep(struct linmodem_port *p, int sleep)
{
	dbg();
	if (p->capabilities & UART_CAP_SLEEP) {
		if (p->capabilities & UART_CAP_EFR) {
			serial_outp(p, UART_LCR, 0xBF);
			serial_outp(p, UART_EFR, UART_EFR_ECB);
			serial_outp(p, UART_LCR, 0);
		}
		serial_outp(p, UART_IER, sleep ? UART_IERX_SLEEP : 0);
		if (p->capabilities & UART_CAP_EFR) {
			serial_outp(p, UART_LCR, 0xBF);
			serial_outp(p, UART_EFR, 0);
			serial_outp(p, UART_LCR, 0);
		}
	}
}


/*
 * This routine is called by rs_init() to initialize a specific serial
 * port.  It determines what type of UART chip this serial port is
 * using: 8250, 16450, 16550, 16550A.  The important question is
 * whether or not this UART is a 16550A or not, since this will
 * determine whether or not we can use its FIFO features or not.
 */
static void autoconfig(struct linmodem_port *p, unsigned int probeflags)
{
	dbg();
	if (!p->port.iobase && !p->port.mapbase && !p->port.membase)
		return;

	dbg("%s%d: autoconf (0x%04x, 0x%p): ",
	    p->reg.dev_name, p->port.line, p->port.iobase, p->port.membase);

	p->ops->autoconfig(p, probeflags);
}

static void autoconfig_irq(struct linmodem_port *p)
{
	unsigned char save_mcr, save_ier;
	unsigned char save_ICP = 0;
	unsigned int ICP = 0;
	unsigned long irqs;
	int irq;

	dbg();

	if (p->port.flags & UPF_FOURPORT) {
		ICP = (p->port.iobase & 0xfe0) | 0x1f;
		save_ICP = inb_p(ICP);
		outb_p(0x80, ICP);
		(void) inb_p(ICP);
	}

	/* forget possible initially masked and pending IRQ */
	probe_irq_off(probe_irq_on());
	save_mcr = serial_inp(p, UART_MCR);
	save_ier = serial_inp(p, UART_IER);
	serial_outp(p, UART_MCR, UART_MCR_OUT1 | UART_MCR_OUT2);

	irqs = probe_irq_on();
	serial_outp(p, UART_MCR, 0);
	udelay(10);
	if (p->port.flags & UPF_FOURPORT)  {
		serial_outp(p, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS);
	} else {
		serial_outp(p, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2);
	}
	serial_outp(p, UART_IER, 0x0f);	/* enable all intrs */
	serial_inp(p, UART_LSR);
	serial_inp(p, UART_RX);
	serial_inp(p, UART_IIR);
	serial_inp(p, UART_MSR);
	serial_outp(p, UART_TX, 0xFF);
	udelay(20);
	irq = probe_irq_off(irqs);

	serial_outp(p, UART_MCR, save_mcr);
	serial_outp(p, UART_IER, save_ier);

	if (p->port.flags & UPF_FOURPORT)
		outb_p(save_ICP, ICP);

	p->port.irq = (irq > 0) ? irq : 0;
}

struct linmodem_port *linmodem_find_by_port(struct uart_port *port)
{
	struct linmodem_port_list *lport=NULL;
	struct list_head *ptr, *head;

	dbg();

	down(&linmodem_sem);
	head = (struct list_head *)&(linmodem_ports);
	for (ptr=head->next; ptr != head; ptr = ptr->next){
		lport = list_entry(ptr, struct linmodem_port_list, list);
		if (&(lport->port->port) == port)
			break;
		lport = NULL;
	}
	up(&linmodem_sem);

	if (lport)
		return lport->port;
	else
		return NULL;
}

struct linmodem_port *linmodem_find_by_line(int line)
{
	struct linmodem_port_list *lport=NULL;
	struct list_head *ptr, *head;

	dbg();

	down(&linmodem_sem);
	head = (struct list_head *)&(linmodem_ports);
	for (ptr=head->next; ptr != head; ptr = ptr->next){
		lport = list_entry(ptr, struct linmodem_port_list, list);
		if (lport->port->port.line == line)
			break;
		lport = NULL;
	}
	up(&linmodem_sem);
	if (lport)
		return lport->port;
	else
		return NULL;
}

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17) )
/*static */void linmodem_stop_tx(struct uart_port *port, unsigned int tty_stop)
#else
/*static */void linmodem_stop_tx(struct uart_port *port)
#endif
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
	}

	/*
	 * We only do this from uart_stop - if we run out of
	 * characters to send, we don't want to prevent the
	 * FIFO from emptying.
	 */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17) )
	if (p->port.type == PORT_16C950 && tty_stop) {
		p->acr |= UART_ACR_TXDIS;
		serial_icr_write(p, UART_ACR, p->acr);
	}
#endif
}
EXPORT_SYMBOL(linmodem_stop_tx);

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17) )
static void linmodem_start_tx(struct uart_port *port, unsigned int tty_start)
#else
static void linmodem_start_tx(struct uart_port *port)
#endif
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	if (!(p->ier & UART_IER_THRI)) {
		p->ier |= UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
	}
	/*
	 * We only do this from uart_start
	 */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17) )
	if (tty_start && p->port.type == PORT_16C950) {
		p->acr &= ~UART_ACR_TXDIS;
		serial_icr_write(p, UART_ACR, p->acr);
	}
#endif
}

static void linmodem_stop_rx(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	p->ier &= ~UART_IER_RLSI;
	p->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(p, UART_IER, p->ier);
}

static void linmodem_enable_ms(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	p->ier |= UART_IER_MSI;
	serial_out(p, UART_IER, p->ier);
}

static inline void
receive_chars(struct linmodem_port *p, int *status, struct pt_regs *regs)
{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) )
	struct tty_struct *tty = p->port.info->tty;
#elif ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)) 
	struct tty_struct *tty = p->port.info->port.tty;
#else
	struct tty_struct *tty = p->port.state->port.tty;
#endif
	unsigned char ch, lsr = *status;
	int max_count = 256;
	char flag;

	dbg();

	do {
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16) )
		/* The following is not allowed by the tty layer and
		   unsafe. It should be fixed ASAP */
		if (unlikely(tty->flip.count >= TTY_FLIPBUF_SIZE)) {
			if(tty->low_latency)
				tty_flip_buffer_push(tty);
			/* If this failed then we will throw away the
			   bytes but must do so to clear interrupts */
		}
#endif
		ch = serial_inp(p, UART_RX);

		flag = TTY_NORMAL;
		p->port.icount.rx++;

		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |
				    UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				p->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&p->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				p->port.icount.parity++;
			else if (lsr & UART_LSR_FE)
				p->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				p->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ingored.
			 */
			lsr &= p->port.read_status_mask;

			if (lsr & UART_LSR_BI) {
				dbg("handling break...");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) )
		if (uart_handle_sysrq_char(&p->port, ch, regs))
#else
		if (uart_handle_sysrq_char(&p->port, ch))
#endif
			goto ignore_char;

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13) )
		if ((lsr & p->port.ignore_status_mask) == 0) {
			tty_insert_flip_char(tty, ch, flag);
		}
		if ((lsr & UART_LSR_OE) &&
		    tty->flip.count < TTY_FLIPBUF_SIZE) {
			/*
			 * Overrun is special, since it's reported
			 * immediately, and doesn't affect the current
			 * character.
			 */
			tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		}
#else
		uart_insert_char(&p->port, lsr, UART_LSR_OE, ch, flag);
#endif
	ignore_char:
		lsr = serial_inp(p, UART_LSR);
	} while ((lsr & UART_LSR_DR) && (max_count-- > 0));
	tty_flip_buffer_push(tty);
	*status = lsr;
}

static inline void transmit_chars(struct linmodem_port *p)
{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)) 
	struct circ_buf *xmit = &p->port.info->xmit;
#else
	struct circ_buf *xmit = &p->port.state->xmit;
#endif
	int count;

	dbg();

	if (p->port.x_char) {
		serial_outp(p, UART_TX, p->port.x_char);
		p->port.icount.tx++;
		p->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&p->port)) {
	        STOP_TX(&p->port);
		return;
	}

	count = p->tx_loadsz;
	do {
		serial_out(p, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		p->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&p->port);

	if (uart_circ_empty(xmit))
	        STOP_TX(&p->port);
}

static inline void check_modem_status(struct linmodem_port *p)
{
	int status;

	dbg();

	status = serial_in(p, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		p->port.icount.rng++;
	if (status & UART_MSR_DDSR)
		p->port.icount.dsr++;
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(&p->port, status & UART_MSR_DCD);
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&p->port, status & UART_MSR_CTS);

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)) 
	wake_up_interruptible(&p->port.info->delta_msr_wait);
#else
	wake_up_interruptible(&p->port.state->port.delta_msr_wait);
#endif
}

/*
 * This handles the interrupt from one port.
 */
void
linmodem_handle_port(struct linmodem_port *p, struct pt_regs *regs)
{
	unsigned int status = serial_inp(p, UART_LSR);

	dbg();

	if (status & UART_LSR_DR) {
		receive_chars(p, &status, regs);
	}
	check_modem_status(p);
	if (status & UART_LSR_THRE){
		transmit_chars(p);
	}
}
EXPORT_SYMBOL(linmodem_handle_port);

/*
 * To support ISA shared interrupts, we need to have one interrupt
 * handler that ensures that the IRQ line has been deasserted
 * before returning.  Failing to do this will result in the IRQ
 * line being stuck active, and, since ISA irqs are edge triggered,
 * no more IRQs will be seen.
 */
static void serial_do_unlink(struct irq_info *i, struct linmodem_port *p)
{
	dbg();

	spin_lock_irq(&i->lock);

	if (!list_empty(i->head)) {
		if (i->head == &p->list)
			i->head = i->head->next;
		list_del(&p->list);
	} else {
		BUG_ON(i->head != &p->list);
		i->head = NULL;
	}

	spin_unlock_irq(&i->lock);
}

int serial_link_irq_chain(struct linmodem_port *p)
{
	struct irq_info *i = irq_lists + p->port.irq;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19) )
	int ret, irq_flags = p->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;
#else
	int ret, irq_flags = p->port.flags & UPF_SHARE_IRQ ? SA_SHIRQ : 0;
#endif

	dbg();

	spin_lock_irq(&i->lock);

	if (i->head) {
		list_add(&p->list, i->head);
		spin_unlock_irq(&i->lock);

		ret = 0;
	} else {
		INIT_LIST_HEAD(&p->list);
		i->head = &p->list;
		spin_unlock_irq(&i->lock);

		ret = request_irq(p->port.irq, p->ops->interrupt,
				  irq_flags, "serial", i);
		if (ret < 0)
			serial_do_unlink(i, p);
	}

	return ret;
}
EXPORT_SYMBOL(serial_link_irq_chain);

void serial_unlink_irq_chain(struct linmodem_port *p)
{
	struct irq_info *i = irq_lists + p->port.irq;

	dbg();

	BUG_ON(i->head == NULL);

	if (list_empty(i->head))
		free_irq(p->port.irq, i);

	serial_do_unlink(i, p);
}
EXPORT_SYMBOL(serial_unlink_irq_chain);

static unsigned int linmodem_tx_empty(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;
	unsigned long flags;
	unsigned int ret;

	dbg();

	spin_lock_irqsave(&p->port.lock, flags);
	ret = serial_in(p, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&p->port.lock, flags);

	return ret;
}

static unsigned int linmodem_get_mctrl(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13) )
	unsigned long flags;
#endif
	unsigned char status;
	unsigned int ret;

	dbg();

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13) )
	spin_lock_irqsave(&p->port.lock, flags);
#else	
	/* serial_core takes lock for us */
#endif

	status = serial_in(p, UART_MSR);

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13) )
	spin_unlock_irqrestore(&p->port.lock, flags);
#endif

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

void linmodem_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned char mcr = 0;
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = mcr | p->mcr;

	serial_out(p, UART_MCR, mcr);

}

EXPORT_SYMBOL(linmodem_set_mctrl);

static void linmodem_break_ctl(struct uart_port *port, int break_state)
{
	struct linmodem_port *p = (struct linmodem_port *)port;
	unsigned long flags;

	dbg();

	spin_lock_irqsave(&p->port.lock, flags);
	if (break_state == -1)
		p->lcr |= UART_LCR_SBC;
	else
		p->lcr &= ~UART_LCR_SBC;
	serial_out(p, UART_LCR, p->lcr);
	spin_unlock_irqrestore(&p->port.lock, flags);
}

static int linmodem_startup(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	return p->ops->startup( p );
}

static void linmodem_shutdown(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	p->ops->shutdown( p );
}

static unsigned int linmodem_get_divisor(struct uart_port *port,
					 unsigned int baud)
{
	unsigned int quot;

	dbg();

	/*
	 * Handle magic divisors for baud rates above baud_base on
	 * SMSC SuperIO chips.
	 */
	if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
	    baud == (port->uartclk/4))
		quot = 0x8001;
	else if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
		 baud == (port->uartclk/8))
		quot = 0x8002;
	else
		quot = uart_get_divisor(port, baud);

	return quot;
}

static void
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20) )
linmodem_set_termios(struct uart_port *port, struct termios *termios,
		     struct termios *old)
#else
linmodem_set_termios(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
#endif
{
	struct linmodem_port *p = (struct linmodem_port *)port;
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	dbg();

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = 0x00;
		break;
	case CS6:
		cval = 0x01;
		break;
	case CS7:
		cval = 0x02;
		break;
	default:
	case CS8:
		cval = 0x03;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= 0x04;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = linmodem_get_divisor(port, baud);

	/*
	 * Work around a bug in the Oxford Semiconductor 952 rev B
	 * chip which causes it to seriously miscalculate baud rates
	 * when DLL is 0.
	 */
	if ((quot & 0xff) == 0 && p->port.type == PORT_16C950 &&
	    p->rev == 0x5201)
		quot ++;

	if (p->capabilities & UART_CAP_FIFO && p->port.fifosize > 1) {
		if (baud < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = p->config.fcr;
	}

	/*
	 * MCR-based auto flow control.  When AFE is enabled, RTS will be
	 * deasserted when the receive FIFO contains more characters than
	 * the trigger, or the MCR RTS bit is cleared.  In the case where
	 * the remote UART is not using CTS auto flow control, we must
	 * have sufficient FIFO entries for the latency of the remote
	 * UART to respond.  IOW, at least 32 bytes of FIFO.
	 */
#ifdef UART_MCR_AFE
	if (p->capabilities & UART_CAP_AFE && p->port.fifosize >= 32) {
		p->mcr &= ~UART_MCR_AFE;
		if (termios->c_cflag & CRTSCTS)
			p->mcr |= UART_MCR_AFE;
	}
#endif /* UART_MCR_AFE */

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&p->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	p->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		p->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		p->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	p->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		p->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		p->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			p->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0){
		p->port.ignore_status_mask |= UART_LSR_DR;
	}

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	p->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&p->port, termios->c_cflag))
		p->ier |= UART_IER_MSI;
#ifdef PORT_XSCALE
	if (p->port.type == PORT_XSCALE)
		p->ier |= UART_IER_UUE | UART_IER_RTOIE;
#endif /* PORT_XSCALE */

	serial_out(p, UART_IER, p->ier);

	if (p->capabilities & UART_CAP_EFR) {
		unsigned char efr = 0;
		/*
		 * TI16C752/Startech hardware flow control.  FIXME:
		 * - TI16C752 requires control thresholds to be set.
		 * - UART_MCR_RTS is ineffective if auto-RTS mode is enabled.
		 */
		if (termios->c_cflag & CRTSCTS)
			efr |= UART_EFR_CTS;

		serial_outp(p, UART_LCR, 0xBF);
		serial_outp(p, UART_EFR, efr);
	}

	if (p->capabilities & UART_NATSEMI) {
		/* Switch to bank 2 not bank 1, to avoid resetting EXCR2 */
		serial_outp(p, UART_LCR, 0xe0);
	} else {
		serial_outp(p, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	}

	serial_outp(p, UART_DLL, quot & 0xff);		/* LS of divisor */
	serial_outp(p, UART_DLM, quot >> 8);		/* MS of divisor */

	/*
	 * LCR DLAB must be set to enable 64-byte FIFO mode. If the FCR
	 * is written without DLAB set, this mode will be disabled.
	 */
	if (p->port.type == PORT_16750)
		serial_outp(p, UART_FCR, fcr);

	serial_outp(p, UART_LCR, cval);		        /* reset DLAB */
	p->lcr = cval;					/* Save LCR */
	if (p->port.type != PORT_16750) {
		if (fcr & UART_FCR_ENABLE_FIFO) {
			/* emulated UARTs (Lucent Venus 167x) need two steps */
			serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		}
		serial_outp(p, UART_FCR, fcr);		/* set fcr */
	}
	linmodem_set_mctrl(&p->port, p->port.mctrl);
	spin_unlock_irqrestore(&p->port.lock, flags);
}

static void
linmodem_pm(struct uart_port *port, unsigned int state,
	    unsigned int oldstate)
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	linmodem_set_sleep(p, state != 0);

	if (p->pm)
		p->pm(port, state, oldstate);
}

/*
 * Resource handling.
 */
static int linmodem_request_std_resource(struct linmodem_port *p)
{
	unsigned int size = 8 << p->port.regshift;
	int ret = 0;

	dbg();

	switch (p->port.iotype) {
	case UPIO_MEM:
		if (!p->port.mapbase)
			break;

		if (!request_mem_region(p->port.mapbase, size, "serial")) {
			ret = -EBUSY;
			break;
		}

		if (p->port.flags & UPF_IOREMAP) {
			p->port.membase = ioremap(p->port.mapbase, size);
			if (!p->port.membase) {
				release_mem_region(p->port.mapbase, size);
				ret = -ENOMEM;
			}
		}
		break;

	case UPIO_HUB6:
	case UPIO_PORT:
		if (!request_region(p->port.iobase, size, "serial"))
			ret = -EBUSY;
		break;
	}
	return ret;
}

static void linmodem_release_std_resource(struct linmodem_port *p)
{
	unsigned int size = 8 << p->port.regshift;

	dbg();

	switch (p->port.iotype) {
	case UPIO_MEM:
		if (!p->port.mapbase)
			break;

		if (p->port.flags & UPF_IOREMAP) {
			iounmap(p->port.membase);
			p->port.membase = NULL;
		}

		release_mem_region(p->port.mapbase, size);
		break;

	case UPIO_HUB6:
	case UPIO_PORT:
		release_region(p->port.iobase, size);
		break;
	}
}

static void linmodem_release_port(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	linmodem_release_std_resource(p);
}

static int linmodem_request_port(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;
	int ret = 0;

	dbg();

	ret = linmodem_request_std_resource(p);

	return ret;
}

static void linmodem_config_port(struct uart_port *port, int flags)
{
	struct linmodem_port *p = (struct linmodem_port *)port;
	int probeflags = PROBE_ANY;
	int ret;

	dbg();

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16) )
	/*
	 * Don't probe for MCA ports on non-MCA machines.
	 */
	if (p->port.flags & UPF_BOOT_ONLYMCA && !MCA_bus)
		return;
#endif

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = linmodem_request_std_resource(p);
	if (ret < 0)
		return;

	if (flags & UART_CONFIG_TYPE)
		autoconfig(p, probeflags);
	if (p->port.type != PORT_UNKNOWN && flags & UART_CONFIG_IRQ)
		autoconfig_irq(p);

	if (p->port.type == PORT_UNKNOWN)
		linmodem_release_std_resource(p);
}

static int
linmodem_verify_port(struct uart_port *port, struct serial_struct *ser)
{

	dbg();

	if (ser->irq >= NR_IRQS || ser->irq < 0 ||
	    ser->baud_base < 9600 || ser->type < PORT_UNKNOWN ||
/*	    ser->type >= ARRAY_SIZE(uart_config) */
	    ser->type == PORT_CIRRUS ||
	    ser->type == PORT_STARTECH)
		return -EINVAL;
	return 0;
}

static const char *
linmodem_type(struct uart_port *port)
{
	struct linmodem_port *p = (struct linmodem_port *)port;

	dbg();

	return p->config.name;
}

static struct uart_ops linmodem_pops = {
	.tx_empty	= linmodem_tx_empty,
	.set_mctrl	= linmodem_set_mctrl,
	.get_mctrl	= linmodem_get_mctrl,
	.stop_tx	= linmodem_stop_tx,
	.start_tx	= linmodem_start_tx,
	.stop_rx	= linmodem_stop_rx,
	.enable_ms	= linmodem_enable_ms,
	.break_ctl	= linmodem_break_ctl,
	.startup	= linmodem_startup,
	.shutdown	= linmodem_shutdown,
	.set_termios	= linmodem_set_termios,
	.pm		= linmodem_pm,
	.type		= linmodem_type,
	.release_port	= linmodem_release_port,
	.request_port	= linmodem_request_port,
	.config_port	= linmodem_config_port,
	.verify_port	= linmodem_verify_port,
};

/**
 *	linmodem_suspend_port - suspend one serial port
 *	@line:  serial line number
 *      @level: the level of port suspension, as per uart_suspend_port
 *
 *	Suspend one serial port.
 */
void linmodem_suspend_port(int line)
{
	struct linmodem_port *p = NULL;
	dbg();

	p = linmodem_find_by_line(line);
	uart_suspend_port(&(p->reg), &(p->port));
}
EXPORT_SYMBOL(linmodem_suspend_port);


/**
 *	linmodem_resume_port - resume one serial port
 *	@line:  serial line number
 *      @level: the level of port resumption, as per uart_resume_port
 *
 *	Resume one serial port.
 */
void linmodem_resume_port(int line)
{
	struct linmodem_port *p = NULL;
	dbg();

	p = linmodem_find_by_line(line);

	uart_resume_port(&(p->reg), &(p->port));
}
EXPORT_SYMBOL(linmodem_resume_port);


/*
 * linmodem_register_port and linmodem_unregister_port allows for
 * 16x50 serial ports to be configured at run-time, to support PCMCIA
 * modems and PCI multiport cards.
 */
static DECLARE_MUTEX(serial_sem);



/**
 *	linmodem_register_port - register a serial port
 *	@port: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int linmodem_register_port(struct uart_port *port)
{
	struct linmodem_port *p;
	int ret = -ENOSPC;

	dbg();

	if (port->uartclk == 0)
		return -EINVAL;

	down(&serial_sem);

	p = linmodem_find_by_port(port);

	if (p) {
		if (p->registered) {
			err("Trying to register port again! Exiting");
			goto exit;
		}
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32) )
		p->reg.state->port = NULL;
#else
		p->reg.state->uart_port = NULL;
#endif
		p->port.ops = &linmodem_pops;
		ret = uart_add_one_port(&(p->reg), &(p->port));
		if (ret == 0) {
			ret = p->port.line;
			p->registered = 1;
		}
	} else
		warn("Could not find port, exiting.");

 exit:
	up(&serial_sem);

	return ret;
}
EXPORT_SYMBOL(linmodem_register_port);

void __unregister_port(struct linmodem_port *p)
{
	dbg();

	down(&serial_sem);
	uart_remove_one_port(&(p->reg), &(p->port));
	p->port.dev = NULL;
	up(&serial_sem);
}


/**
 *	linmodem_unregister_port - remove a 16x50 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void linmodem_unregister_port(int line)
{
	struct linmodem_port *p = NULL;

	dbg();

	p = linmodem_find_by_line(line);

	__unregister_port(p);
}
EXPORT_SYMBOL(linmodem_unregister_port);



static int __init linmodem_init(void)
{
	int i;

	dbg();

	info( MY_NAME " " DRIVER_VERSION " loaded" );

	INIT_LIST_HEAD((struct list_head*)&linmodem_ports);

	for (i = 0; i < NR_IRQS; i++)
		spin_lock_init(&irq_lists[i].lock);

/* 	ret = uart_register_driver(&linmodem_reg); */
/* 	if (ret == 0) */
/* 		goto out; */

/* 	uart_unregister_driver(&linmodem_reg); */
/*  out: */
/* 	return ret; */
	return 0;
}

static void __exit linmodem_exit(void)
{
	struct linmodem_port_list *lport=NULL;
	struct list_head *ptr, *head;

	dbg();

	info( MY_NAME " " DRIVER_VERSION " unloaded" );

	down(&linmodem_sem);
	head = (struct list_head*)&linmodem_ports;
	for (ptr=head->next; ptr != head; ptr = ptr->next){
		lport = list_entry(ptr, struct linmodem_port_list, list);
		uart_unregister_driver(&(lport->port->reg));
	}
	up(&linmodem_sem);
}

module_init(linmodem_init);
module_exit(linmodem_exit);


struct linmodem_port **ports = NULL;


int linmodem_new(struct linmodem_port *port)
{
	int ret;
	struct linmodem_port_list *lport;

	dbg();

	/* Add port to list of ports */
	down(&linmodem_sem);

	port->registered = 0;

	lport = kmalloc(sizeof(*lport), GFP_KERNEL);
	INIT_LIST_HEAD((struct list_head*)lport);
	lport->port = port;

	list_add_tail((struct list_head*)lport,
		      (struct list_head*)&linmodem_ports);

	up(&linmodem_sem);

	ret = uart_register_driver(&(port->reg));
	if (ret)
		uart_unregister_driver(&(port->reg));

	return ret;
}
EXPORT_SYMBOL(linmodem_new);

int linmodem_hw_new(struct uart_port *port)
{
	int ret = -ENOSPC;
	struct linmodem_port *p;
	dbg();

	p = linmodem_find_by_port(port);
	if (p) {
		dbg("port found, register it.");
		ret = linmodem_register_port(&(p->port));
	} else
		dbg("given port not found. Maybe unregistered with "
		    "linmodem_new()?");
	return ret;
}
EXPORT_SYMBOL(linmodem_hw_new);


void linmodem_del(struct linmodem_port *port)
{
	struct linmodem_port_list *lport=NULL;
	struct list_head *ptr, *head, *aux;

	dbg("port=%p", port);
	dbg("BEG: Unregister driver------------------------------------------");

	down(&linmodem_sem);
	head = (struct list_head *)&(linmodem_ports);
	for (ptr=head->next; ptr != head; ptr = ptr->next){
		dbg("   %p <- [%p] -> %p", ptr->prev, ptr, ptr->next);
		lport = list_entry(ptr, struct linmodem_port_list, list);
		if (lport->port == port) {
			dbg("   unregister: %p (%p)", lport->port, &lport->port->port);
			if (lport->port->registered &&
			    lport->port->port.dev != NULL) {
				dbg("      __unregister_port(%p)",lport->port);
				__unregister_port(lport->port);
				dbg("      __unregister_port()-> Ok!");
			}
			else
				dbg("      it was not registered.");
			lport->port->registered = 0;
			aux = ptr->next;
			dbg("list_del(%p)", ptr);
			list_del(ptr);
			dbg("kfree(%p)", ptr);
			kfree(ptr);
			ptr = aux;
			break;
		}
	}
	up(&linmodem_sem);

	dbg("END: Unregister driver-----------------------------------------");
	uart_unregister_driver(&(port->reg));
	dbg("END. Driver unregistered");
}
EXPORT_SYMBOL(linmodem_del);
