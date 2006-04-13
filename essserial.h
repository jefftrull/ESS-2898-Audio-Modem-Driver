/*
 * follows serial/8250.h
 *
 * based on ptserial-2.6.h by Gustavo Barbieri
 */

#ifndef __ESSSERIAL_2_6_H
#define __ESSSERIAL_2_6_H

#include <linux/config.h>

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9) )
#ifndef __iomem
#define __iomem
#endif /* __iomem */
#endif /* kernel 2.6.9 */

#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */

#undef SERIAL_DEBUG_PCI

#if defined(__i386__) && (defined(CONFIG_M386) || defined(CONFIG_M486))
#define SERIAL_INLINE
#endif

#ifdef SERIAL_INLINE
#define _INLINE_ inline
#else
#define _INLINE_
#endif

#define PROBE_RSA	(1 << 0)
#define PROBE_ANY	(~0)

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)


/* serial_core.h has no include protection, so fix */
#ifndef UPF_FOURPORT
# include <linux/serial_core.h>
#endif
#include <linux/device.h>

struct plat_ptserial_port {
	unsigned long	iobase;		/* io base address */
	void __iomem	*membase;	/* ioremap cookie or NULL */
	unsigned long	mapbase;	/* resource base */
	unsigned int	irq;		/* interrupt number */
	unsigned int	uartclk;	/* UART clock rate */
	unsigned char	regshift;	/* register shift */
	unsigned char	iotype;		/* UPIO_* */
	unsigned int	flags;		/* UPF_* flags */
};

/* uart_port doesn't contain region_0, region_1, so we map to iobase, membase */
#define pctel_region_0(x)    (x).iobase
#define pctel_region_0_p(x)  (x)->iobase
#define pctel_region_1(x)    (x).membase
#define pctel_region_1_p(x)  (x)->membase

#define ESS_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST)

/* override supported serial type definition in serial.h */
#define PORT_ESS                      14

#ifdef  PORT_MAX
#undef  PORT_MAX
#endif
#define PORT_MAX                        14

#define ESS_MAJOR 62
#define ESS_MINOR 64

#define ESS_TTY_LINE			0



/* VENDOR/DEVICEID List */
/*  ESS Tech ES2898 */
/* include/linux/pci_ids.h actually defines PCI_VENDOR_ID_ESS, no need to do it here */
#define PCI_DEVICE_ID_ESS_ES2898	0x2898

/* END VENDOR/DEVICEID List */


extern int ess_new(struct uart_port *port);
extern int ess_init(void);
extern void ess_exit(void);

extern int essserial_hw_check_modem(int vendor);

#endif /* __ESSSERIAL_2_6_H */
