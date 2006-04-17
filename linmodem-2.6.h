/*
 *  Copyright (C) 2005 Glauber de Oliveira Costa,
 *                     Gustavo Sverzut Barbieri,
 *                     Murillo Fernandes Bernardes,
 *                     Robert H. Thornburrow
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _LINUX_LINMODEM_H
#define _LINUX_LINMODEM_H

#include <linux/version.h>

/* serial_core.h has no include protection, so fix */
#ifndef UPF_FOURPORT
# include <linux/serial_core.h>
#endif
#include <linux/serial.h>

#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */

#define PROBE_ANY	(~0)

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)


struct linmodem_config {
	const char	*name;
	unsigned short	fifo_size;
	unsigned short	tx_loadsz;
	unsigned char	fcr;
	unsigned int	flags;
};

struct linmodem_port {
	struct uart_port        port;
	struct timer_list	timer;		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned int            capabilities;
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned short		rev;
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;

	struct linmodem_ops     *ops;
	struct linmodem_config  config;
	struct uart_driver      reg;
	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);

	unsigned char           registered;
	void *data;
};

struct linmodem_ops {
	unsigned int (*serial_in)(struct linmodem_port *, int offset);
	void         (*serial_out)(struct linmodem_port *, int offset,
				   int value);
	irqreturn_t  (*interrupt)(int irq, void *dev_id,
					  struct pt_regs *regs);
	void         (*set_mctrl)(struct linmodem_port *, unsigned int mctrl);
	void         (*autoconfig)(struct linmodem_port *,
				   unsigned int probeflags);
	int          (*startup)(struct linmodem_port *);
	void         (*shutdown)(struct linmodem_port *);
};

int linmodem_register_port(struct uart_port *port);
void linmodem_unregister_port(int line);
int linmodem_new(struct linmodem_port *port);
void linmodem_del(struct linmodem_port *port);
int linmodem_hw_new(struct uart_port *port);


/* XXX: REMOVE ME!!! */
int serial_link_irq_chain(struct linmodem_port *p);
void serial_unlink_irq_chain(struct linmodem_port *p);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,14)
void linmodem_stop_tx(struct uart_port *port);
#define stop_tx(port) (linmodem_stop_tx(port))
#else
void linmodem_stop_tx(struct uart_port *port, unsigned int tty_stop);
#define stop_tx(port) (linmodem_stop_tx(port, 0))
#endif

#endif /* _LINUX_LINMODEM_H */
