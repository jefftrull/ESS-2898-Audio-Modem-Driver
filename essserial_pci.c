/*
 *  Probe module for ESS modems.
 *
 *  Based on linux/drivers/serial/8250_pci.c
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *  Based on ptserial_pci-2.6.c by Costa/Barbieri/Bernardes (see below)
 *
 *  Copyright (C) 2001 Russell King, All Rights Reserved.
 *  Copyright (C) 2005 Glauber de Oliveira Costa,
 *                     Gustavo Sverzut Barbieri,
 *                     Murillo Fernandes Bernardes,
 *                     Robert H. Thornburrow
 *  Copyright (C) 2006 Jeffrey E. Trull
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/tty.h>
/* serial_core.h has no include protection, so fix */
#ifndef UPF_FOURPORT
# include <linux/serial_core.h>
#endif
#include <linux/8250_pci.h>
#include <linux/bitops.h>

#include <asm/byteorder.h>
#include <asm/io.h>

#include "linmodem-2.6.h"
#include "include/ess.h"
#include "essserial.h"


#define MY_NAME "ess_pci"

#ifdef ESS_DEBUG
static int debug = 1;
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Debugging mode enabled or not");


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


#if defined (HAL_I8XX) || defined (HAL_VIA686A)
static unsigned int iobase       = 0;
static unsigned int iobase1      = 0;
#endif
static int irq                   = 0;

/*
 * Definitions for PCI support.
 */
#define FL_BASE_MASK		0x0007
#define FL_BASE0		0x0000
#define FL_BASE1		0x0001
#define FL_BASE2		0x0002
#define FL_BASE3		0x0003
#define FL_BASE4		0x0004
#define FL_GET_BASE(x)		(x & FL_BASE_MASK)

/* Use successive BARs (PCI base address registers),
   else use offset into some specified BAR */
#define FL_BASE_BARS		0x0008

/* do not assign an irq */
#define FL_NOIRQ		0x0080

/* Use the Base address register size to cap number of ports */
#define FL_REGION_SZ_CAP	0x0100

struct pci_board {
	unsigned int flags;
	unsigned int num_ports;
	unsigned int base_baud;
	unsigned int uart_offset;
	unsigned int reg_shift;
	unsigned int first_offset;
};

/*
 * init function returns:
 *  > 0 - number of ports
 *  = 0 - use board->num_ports
 *  < 0 - error
 */
struct pci_serial_quirk {
	u32	vendor;
	u32	device;
	u32	subvendor;
	u32	subdevice;
	int	(*init)(struct pci_dev *dev);
	int	(*setup)(struct pci_dev *dev, struct pci_board *board,
			 struct uart_port *port, int idx);
	void	(*exit)(struct pci_dev *dev);
};

#define PCI_NUM_BAR_RESOURCES	6

struct serial_private {
	unsigned int		nr;
	void __iomem		*remapped_bar[PCI_NUM_BAR_RESOURCES];
	struct pci_serial_quirk	*quirk;
	int			line[0];
};

static int
setup_port(struct pci_dev *dev, struct uart_port *port,
	   int bar, int offset, int regshift)
{
	struct serial_private *priv = pci_get_drvdata(dev);
	unsigned long base, len;

	dbg();

	if (bar >= PCI_NUM_BAR_RESOURCES)
		return -EINVAL;

	if (pci_resource_flags(dev, bar) & IORESOURCE_MEM) {
		base = pci_resource_start(dev, bar);
		len =  pci_resource_len(dev, bar);

		if (!priv->remapped_bar[bar])
			priv->remapped_bar[bar] = ioremap(base, len);
		if (!priv->remapped_bar[bar])
			return -ENOMEM;

		port->iotype = UPIO_MEM;
		port->mapbase = base + offset;
		port->membase = priv->remapped_bar[bar] + offset;
		port->regshift = regshift;
	} else {
		base = pci_resource_start(dev, bar) + offset;
		port->iotype = UPIO_PORT;
		port->iobase = base;
	}
	return 0;
}

static int
pci_default_setup(struct pci_dev *dev, struct pci_board *board,
		  struct uart_port *port, int idx)
{
	unsigned int bar, offset = board->first_offset, maxnr;

	dbg();

	bar = FL_GET_BASE(board->flags);
	if (board->flags & FL_BASE_BARS)
		bar += idx;
	else
		offset += idx * board->uart_offset;

	maxnr = (pci_resource_len(dev, bar) - board->first_offset) /
		(8 << board->reg_shift);

	if (board->flags & FL_REGION_SZ_CAP && idx >= maxnr)
		return 1;

	return setup_port(dev, port, bar, offset, board->reg_shift);
}

static int
pci_ess_setup(struct pci_dev *dev, struct pci_board *board,
		  struct uart_port *port, int idx)
{
	unsigned char rev;
	int ret;

	dbg();

	port->irq = dev->irq;

	/* Force IRQ line */
	if((irq > 0) && (dev->irq != irq)) {
		u8 new_irq;

		udelay(15);
		pci_write_config_byte(dev, PCI_INTERRUPT_LINE, irq);
		udelay(15);
		pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &new_irq);
		udelay(15);

		if(new_irq == irq)
			info("Ess override IRQ (detected %d) to %d.",
			     dev->irq, irq);
		else
			err("Ess failed to override IRQ %d.", irq);
		port->irq = new_irq;
	}

	if (dev->irq == 0)
		return 0;

	ret = pci_default_setup(dev,board,port,idx);

	pci_read_config_byte(dev, PCI_CLASS_REVISION, &rev);

#ifdef CONFIG_PCI_NAMES
	info("Ess device[%s](0x%x) found \"%s (rev %02x)\", "
	     "iobase=0x%x, irq=%d.", pci_name(dev), dev->devfn,
	     dev->pretty_name, rev, port->iobase, port->irq);
#else
	info("Ess device[%s](0x%x) found %04x:%04x (rev %02x), "
	     "iobase=0x%x, irq=%d.", pci_name(dev), dev->devfn,
	     dev->vendor, dev->device, rev, port->iobase, port->irq);
#endif /* CONFIG_PCI_NAMES */

	return ret;
}

/*
 * Master list of serial port init/setup/exit quirks.
 * This does not describe the general nature of the port.
 * (ie, baud base, number and location of ports, etc)
 *
 * This list is ordered alphabetically by vendor then device.
 * Specific entries must come before more generic entries.
 */
static struct pci_serial_quirk pci_serial_quirks[] = {
	{
		.vendor		= PCI_VENDOR_ID_ESS,
		.device		= PCI_DEVICE_ID_ESS_ES2898,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_ess_setup,
	},

	/*
	 * Default "match everything" terminator entry
	 */
	{
		.vendor		= PCI_ANY_ID,
		.device		= PCI_ANY_ID,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,
	}
};

static inline int quirk_id_matches(u32 quirk_id, u32 dev_id)
{
	dbg();

	return quirk_id == PCI_ANY_ID || quirk_id == dev_id;
}

static struct pci_serial_quirk *find_quirk(struct pci_dev *dev)
{
	struct pci_serial_quirk *quirk;

	dbg();

	for (quirk = pci_serial_quirks; ; quirk++)
		if (quirk_id_matches(quirk->vendor, dev->vendor) &&
		    quirk_id_matches(quirk->device, dev->device) &&
		    quirk_id_matches(quirk->subvendor, dev->subsystem_vendor) &&
		    quirk_id_matches(quirk->subdevice, dev->subsystem_device))
		 	break;
	return quirk;
}

static inline int
get_pci_irq(struct pci_dev *dev, struct pci_board *board, int idx)
{
	dbg();

	if (board->flags & FL_NOIRQ)
		return 0;
	else
		return dev->irq;
}

enum pci_board_num_t {
	pbn_default = 0,

};

/*
 * uart_offset - the space between channels
 * reg_shift   - describes how the UART registers are mapped
 *               to PCI memory by the card.
 * For example IER register on SBS, Inc. PMC-OctPro is located at
 * offset 0x10 from the UART base, while UART_IER is defined as 1
 * in include/linux/serial_reg.h,
 * see first lines of serial_in() and serial_out() in 8250.c
*/

static struct pci_board pci_boards[] __devinitdata = {
	/* Generic serial board, pbn_b0_1_115200, pbn_default */
	{
		.flags		= FL_BASE0,
	  	.num_ports	= 1,
		.base_baud	= 115200
	 },		/* pbn_b0_1_115200,   pbn_default */

};


static inline int
serial_pci_matches(struct pci_board *board, struct pci_board *guessed)
{
	dbg();

	return
	    board->num_ports == guessed->num_ports &&
	    board->base_baud == guessed->base_baud &&
	    board->uart_offset == guessed->uart_offset &&
	    board->reg_shift == guessed->reg_shift &&
	    board->first_offset == guessed->first_offset;
}

/*
 * Probe one serial board.  Unfortunately, there is no rhyme nor reason
 * to the arrangement of serial ports on a PCI card.
 */
static int __devinit
pciserial_init_one(struct pci_dev *dev, const struct pci_device_id *ent)
{
	struct serial_private *priv;
	struct pci_board *board;
	struct pci_serial_quirk *quirk;
	int rc, nr_ports, i;

	dbg();

    /* Sanity check to ensure the correct binary part of the driver */
    if( !esscom_hw_check_modem( dev->vendor ) ) {
        err("pciserial_init_one: invalid ess_hw binary module");
		return -EINVAL;
    }

	if (ent->driver_data >= ARRAY_SIZE(pci_boards)) {
		err("pciserial_init_one: invalid driver_data: %ld",
		    ent->driver_data);
		return -EINVAL;
	}

	board = &pci_boards[ent->driver_data];

	rc = pci_enable_device(dev);
	if (rc)
		return rc;

	board->num_ports = 1;
	board->flags = 0;

	nr_ports = board->num_ports;

	/*
	 * Find an init and setup quirks.
	 */
	quirk = find_quirk(dev);

	/*
	 * Run the new-style initialization function.
	 * The initialization function returns:
	 *  <0  - error
	 *   0  - use board->num_ports
	 *  >0  - number of ports
	 */
	if (quirk->init) {
		rc = quirk->init(dev);
		if (rc < 0)
			goto disable;
		if (rc)
			nr_ports = rc;
	}

	priv = kmalloc(sizeof(struct serial_private) +
		       sizeof(unsigned int) * nr_ports,
		       GFP_KERNEL);
	if (!priv) {
		rc = -ENOMEM;
		goto deinit;
	}

	memset(priv, 0, sizeof(struct serial_private) +
			sizeof(unsigned int) * nr_ports);

	priv->quirk = quirk;
	pci_set_drvdata(dev, priv);

	for (i = 0; i < nr_ports; i++) {
		struct uart_port serial_port;
		memset(&serial_port, 0, sizeof(struct uart_port));

		serial_port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF |
				    UPF_SHARE_IRQ;
		serial_port.uartclk = board->base_baud * 16;
		serial_port.irq = get_pci_irq(dev, board, i);
		serial_port.dev = &dev->dev;
		if (quirk->setup(dev, board, &serial_port, i))
			break;

		info("Setup PCI port: port 0x%x, irq %d, type %d, "
		     "membase %p, ops %p",
		     serial_port.iobase, serial_port.irq, serial_port.iotype,
		     serial_port.membase, serial_port.ops);

		/* priv->line[i] = linmodem_register_port(&serial_port); */
		/* priv->line[i] = linmodem_new_hw(&serial_port); */
		priv->line[i] = ess_new(&serial_port);
		if (priv->line[i] < 0) {
			warn("Couldn't register serial port %s: %d (%s)",
			     pci_name(dev), priv->line[i],
			     (priv->line[i] == -EINVAL) ?
			     "Invalid Argument":
			     (priv->line[i] == -ENOSPC) ?
			     "Unspecified" : "Unknow Error");
			break;
		}
	}

	priv->nr = i;

	return 0;

 deinit:
	if (quirk->exit)
		quirk->exit(dev);
 disable:
	pci_disable_device(dev);
	return rc;
}

static void __devexit pciserial_remove_one(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	dbg();

	pci_set_drvdata(dev, NULL);

	if (priv) {
		struct pci_serial_quirk *quirk;
		int i;

		for (i = 0; i < priv->nr; i++)
			linmodem_unregister_port(priv->line[i]);

		for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
			if (priv->remapped_bar[i])
				iounmap(priv->remapped_bar[i]);
			priv->remapped_bar[i] = NULL;
		}

		/*
		 * Find the exit quirks.
		 */
		quirk = find_quirk(dev);
		if (quirk->exit)
			quirk->exit(dev);

		pci_disable_device(dev);

		kfree(priv);
	}
}

#define MY_PCI_DEVICE(vend,dev) \
    { (vend), (dev), PCI_ANY_ID, PCI_ANY_ID, 0, 0 }

static struct pci_device_id serial_pci_tbl[] __devinitdata = {
    MY_PCI_DEVICE( PCI_VENDOR_ID_ESS, PCI_DEVICE_ID_ESS_ES2898 ),
    { 0, }
};

static struct pci_driver serial_pci_driver = {
	.name		= "ess_hw",
	.probe		= pciserial_init_one,
	.remove		= __devexit_p(pciserial_remove_one),
	.id_table	= serial_pci_tbl,
};

static int __init essserial_pci_init(void)
{
    struct pci_dev *dev = NULL;
    struct device *reldev;
    int ret;
    
    dbg();

    /* Initialise the main body of essserial.c */
    ret = ess_init();
	
    if( ret )
        return ret;

    /* Attempt to release any drivers that may have already claimed our modem */
    for_each_pci_dev(dev)
    {
      /* API changed in 2.6.13 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
        if( pci_match_id( serial_pci_tbl, dev)
#else
        if( pci_match_device( serial_pci_tbl, dev)
#endif
            && pci_dev_driver(dev) && (reldev = get_device(&dev->dev)) )
        {
            info( "device %04x:%04x is grabbed by driver %s: try to release\n",
                  dev->vendor, dev->device,
                  (reldev && reldev->driver) ? reldev->driver->name : "unknown" );
            device_release_driver( reldev );
            put_device( reldev );
        }
	/* release reference to this device */
	pci_dev_put(dev);
    }

    return pci_register_driver(&serial_pci_driver);
}

static void __exit essserial_pci_exit(void)
{
    dbg();

    pci_unregister_driver(&serial_pci_driver);

    /* Exit the main body of essserial.c */
    ess_exit();
}

module_init(essserial_pci_init);
module_exit(essserial_pci_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ESS PCI modem driver: Based on linux 2.6.10 serial 8250_pci driver");
MODULE_DEVICE_TABLE(pci, serial_pci_tbl);

MODULE_PARM(irq,                "i");
MODULE_PARM_DESC(irq,           "Override autodetected IRQ.");

#if defined (HAL_I8XX) || defined (HAL_VIA686A)
MODULE_PARM(iobase,             "i");
MODULE_PARM_DESC(iobase,        "Check for modem at iobase address.");

MODULE_PARM(iobase1,            "i");
MODULE_PARM_DESC(iobase1,       "Using iobase/iobase1 address.");
#endif
