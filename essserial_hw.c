/*
 *  Copyright (C) 2006 Jeffrey Elliot Trull
 *
 *      This file is to be linked with the rest of the libraries in
 *      the distribution package.  It links to proprietary code and
 *      is therefore NOT covered by the GPL.
 *
 *      This file contains mostly stuff that links into or overrides
 *      the proprietary binary esscom.o
 *
 *      This work would have been a lot harder if I hadn't been able
 *      to follow in the footsteps of Shachar Raindel, who managed to
 *      create a working 2.4 version in a similar manner.
 */
 
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <asm/io.h>

#include "include/ess_bin.h"

#include <linux/serial_reg.h>

#define PCI_VENDOR_ID_ESS     0x125D


#define MY_NAME "ess_hw"

#define err(format, arg...) \
	printk(KERN_ERR "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)
#define info(format, arg...) \
	printk(KERN_INFO "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING "%s(%d): " format "\n", MY_NAME, __LINE__, ## arg)


static char* ess_options __initdata =
#if defined (HAL_ESS)
        "ESS"
#else
        "*UNKNOWN*"
#endif
        ;

/**
 * This function is necessary in order to enforce a dependency between ess.ko
 * and ess_hw.ko.  It provides a quick check to see that the correct HAL
 * macro has been defined during compilation giving some idea whether we are
 * linked against the correct binary code.
 */
int esscom_hw_check_modem( int vendor )
{
    int ret = 0;
    ret = (vendor == PCI_VENDOR_ID_ESS);
    return ret;
}

EXPORT_SYMBOL(esscom_hw_check_modem);

unsigned long Get_System_Time(void);

/* take care of scaling from HZ=100 to now and also potential rollover/large values of jiffies */
/* BOZO see if there is a better way to deal with large jiffies */

unsigned long ess_jiffies_start = 0;
#define JIFFIES_OLDSCHOOL ( ((jiffies - ess_jiffies_start) * 100) / HZ)

unsigned long last_tick_jiffies = 0;

/* a substitute for the existing esscom_initialize, in the binary */
/* it used old kernel APIs.  this is a translation of the disassembly with 2.6 updates */

void esscom_initialize(void) {
    unsigned long wait_until;

    /* record the time */
    ess_jiffies_start = jiffies;

    /* do some writes, with 40 old-jiffy pauses, to the port */

    outb(0, gwBaseIO+2);

    wait_until = jiffies + 0x28 * HZ / 100;
    while (time_before(jiffies, wait_until)) {}

    outb(0x10, gwBaseIO+2);

    wait_until = jiffies + 0x28 * HZ / 100;
    while (time_before(jiffies, wait_until)) {}

    outb(0, gwBaseIO+2);

    wait_until = jiffies + 0x28 * HZ / 100;
    while (time_before(jiffies, wait_until)) {}

    AddrMSRShadow = 0x4126 | UART_MSR_CTS;           /* start off with "Clear to Send" set */
    EFlags = 0;
    MiscFlags = 0;

    myPort = kmalloc(sizeof(*myPort), GFP_KERNEL);
    memset(myPort, 0, sizeof(*myPort));

    myPort->dummy1 = NULL;
    myPort->dummy2 = 0x1000;
    myPort->dummy3 = 0;
    myPort->dummy4 = 0;
    myPort->dummy5 = 0;
    myPort->xmit_buf = kmalloc(0x1000, GFP_KERNEL);
    myPort->last_untransmitted_ptr = 0x1000;
    myPort->untransmitted_char_cnt = 0;
    myPort->first_untransmitted_ptr = 0;
    myPort->mystery_ptr = 0;

    ReadID("ESS0425");       /* note: the ReadID function doesn't use any of its parameters! */
                             /* but this is how the binary calls it... */

    info("ESS info:  Card is %x, RealID is %x, CardID is %x, HSP_Flag is %lx, No_LCS is %x, FunctionID is %x",
	 Card, RealID, CardID, HSP_Flag, No_LCS, FunctionID);

    init_at_var();

    if (DownLoadFirmware(gwBaseIO, gwBaseEDSP) == 0) {
	err("DownLoadFirmware failed!\n");
	return;
    }

    init_s_reg();
    action_for_restore_s();
    init_modem();

    intEnaFlag = 0;
    timeout = 0x28;
    modem_status = 0;
    timer_cnt = 0;
    last_tick_jiffies = jiffies;
    BreakFlag = 0;

}

void esscom_hw_shutdown (void) {
    /* I think this is the right thing to do to shut down the hw and prevent interrupts */
    stop_modem(0);

    /* free anything we allocated in esscom_initialize */
    kfree(myPort->xmit_buf);
    kfree(myPort);

}

EXPORT_SYMBOL(esscom_hw_shutdown);

/* a translation of the disassembled modem_task function to permit linking in modern kernels */

void modem_task(void) {

    /* first bit's what I think is a literal translation of the assembly */
    /* do a bunch of internal housekeeping/updating */
    last_modem_status = modem_status;
    modem_status = get_modem_status();

    if (proc_dte_input != proc_v80_input) {
	(*proc_dte_input)();
    }
    else if (!critical_section) {
	proc_v80_input();
    }

    Update_PTT_timer();

    (*at_task)();

    PttSilenceDetect();
    SilenceReport();

    if (online == 1) {
	check_clrdwn();
	check_connect_timer();
	check_carrier_off();
	(*online_task)();
	check_own_busy();
	check_tran_quality();
	check_rate_renegotiation();
    }   
    PTT_Ring_Detector();
    daa_task();

}

/* time-related functions
 * these are superseded versions of those in the binary, intended to remove direct access
 * to the "jiffies" variable.  Since HZ has changed over time, I suspect the timekeeping
 * in the binary is now incorrect.  I will assume the binary was written with HZ=100.
 */

/* Get_SystemTime
 * officially returns 64b quantity in edx:eax, but only lower 32b are used by any callers
 */

unsigned long Get_System_Time (void) {

    /* the result here is basically (10 * JIFFIES_OLDSCHOOL) */
    /* previously fp was used, for some reason (divide by 100.0, multiply by 1000.0) */

    return ((10 * 100) / HZ) * (jiffies - ess_jiffies_start);

}

/* another scaled binary-internal time measure based on jiffies */
unsigned long CurrentBlkTime (void) {
    unsigned long ret;

    /*
     * normally jiffies_oldschool = (100/HZ)*jiffies, but since a multiply is done first,
     * consolidate constants for accuracy and speed
     * old code: ret = 100 * JIFFIES_OLDSCHOOL
     */

    ret = ((100 * 100) / HZ) * (jiffies - ess_jiffies_start);

    return ret;

}

/* yet another function producing a strange scaled version of jiffies */
unsigned long systemTime(void) {
    double tmp;
    unsigned long ret;

    tmp = JIFFIES_OLDSCHOOL / 100.0;
    tmp *= 1000.0;
    tmp /= 40.0;
    ret = tmp;

    return ret;
}

unsigned long  __attribute__((regparm(0))) ChkBlkTime(unsigned long prevtime) {
    unsigned long ret;

    /* BOZO this seems wrong (prevtime is a scaled value!) yet it matches the binary */
    ret = (JIFFIES_OLDSCHOOL - prevtime) / 100;

    return ret;
}    

/*
 * unused function routine
 * this is supplied to override code we *think* is not called anymore.
 * in the event that it is called, an error message will be produced
 */

void ess_unused_function (void) {
    err("this function should never be called! there is a programming error in the driver");
    BUG_ON(1);
}

/*
 * The VUART (Virtual UART)
 * This is the main interface between the binary (handles actual hardware modulation etc.)
 * and the uart/tty subsystem.  If we pretend to be a UART, all of the tricky serial stuff
 * can be handled by the kernel in a single place.
 * I figured out what to do for this stuff mostly by reading the disassembled rs_ioctl
 */

static unsigned char get_uart_msr(void) {
    return 0xff & AddrMSRShadow;
}

EXPORT_SYMBOL(get_uart_msr);

static void put_uart_msr(unsigned char c) {
    /* this should never get called, actually! MSR is read-only */
    err("put_uart_msr called! this should not happen.");
}

EXPORT_SYMBOL(put_uart_msr);

static unsigned char get_uart_lsr(void) {
    /* just reflect the "tx empty" (actually, 1/4 full or less) and "data ready" conditions */
    unsigned char ret;

    if (myPort->untransmitted_char_cnt <= 0xff) {
	ret = 0x60;   /* shift and holding reg empty bits */
    }
    else {
	ret = 0;
    }
    if (((proc_dte_output == proc_output) && (dte_output_buf_put != dte_output_buf_get)) ||
	((proc_dte_output == proc_info_output) && ((dte_output_buf_put != dte_output_buf_get) || (r_info_buf_put != r_info_buf_get))) ||
	((proc_dte_output == proc_v80_output) && (myRxV80[4] != 0))) {
	/* data available */
	ret |= 0x1;
    }
    return ret;
}

EXPORT_SYMBOL(get_uart_lsr);

static void put_uart_lsr(unsigned char c) {
    /* this should never get called, actually! MSR is read-only */
    err("put_uart_lsr called! this should not happen.");
}

EXPORT_SYMBOL(put_uart_lsr);

static unsigned char get_uart_rx(void) {
    unsigned char ret;
    unsigned int rcv_cnt;
    rcv_cnt = (*proc_dte_output)(&ret, 1);
    if (rcv_cnt > 1) {
	err("received %d characters, should be 1!", rcv_cnt);
    }
    return ret;
}

EXPORT_SYMBOL(get_uart_rx);

static void put_uart_tx(unsigned char c) {
    int avail_room;

    avail_room = min(0xfff - myPort->untransmitted_char_cnt,
		     0x1000 - myPort->mystery_ptr);
    if (avail_room > 0) {
	myPort->xmit_buf[myPort->mystery_ptr] = c;
	myPort->mystery_ptr = (1 + myPort->mystery_ptr) & 0xfff;
	myPort->untransmitted_char_cnt++;
    }
    else {
	err("put_uart_tx called without any room for transmit char");
    }
}

EXPORT_SYMBOL(put_uart_tx);

/* I don't believe that either LCR or MCR has any real meaning for a modem */
/* just in case, I will implement them as registers with no effects */
static unsigned char LCR;
static unsigned char get_uart_lcr(void) {
    return LCR;
}
static void put_uart_lcr(unsigned char c) {
    LCR = c;
}

EXPORT_SYMBOL(get_uart_lcr);
EXPORT_SYMBOL(put_uart_lcr);

static unsigned char MCR;
static unsigned char get_uart_mcr(void) {
    return MCR;
}
static void put_uart_mcr(unsigned char c) {
    MCR = c;
}

EXPORT_SYMBOL(get_uart_mcr);
EXPORT_SYMBOL(put_uart_mcr);

/* SCR, the "scratch" register, is pretty much unused as far as I can tell */
/* still, it's easy to implement... */
static unsigned char SCR;
static unsigned char get_uart_scr(void) {
    return SCR;
}
static void put_uart_scr(unsigned char c) {
    SCR = c;
}

EXPORT_SYMBOL(get_uart_scr);
EXPORT_SYMBOL(put_uart_scr);


/* IER doesn't really apply, since interrupts are generated for different reasons */
/* implement anyway, and use for the IIR reporting below */

static unsigned char IER;
static unsigned char get_uart_ier(void) {
    return IER;
}
static void put_uart_ier(unsigned char c) {
    IER = c;
}

EXPORT_SYMBOL(get_uart_ier);
EXPORT_SYMBOL(put_uart_ier);

/* similar to implementation of LSR but I will qualify some bits with IER bits */
/* who knows if this will get used... better safe than sorry */
static unsigned char get_uart_iir(void) {
    unsigned char lsr;

    lsr = get_uart_lsr();
    /* highest priority is "data ready" */
    if (lsr & IER & 0x1) {
	return 0xc4;
    }
    if ((lsr & 0x20) && (IER & 0x2)) {
	return 0xc2;
    }
    return 0x1;
}

static void put_uart_iir(unsigned char c) {
    err("attempt to write IIR with value %x", c);
    BUG_ON(1);
}

EXPORT_SYMBOL(get_uart_iir);
EXPORT_SYMBOL(put_uart_iir);

/* an interesting case.  The code for FCR and IIR are the same but it's IIR on read, FCR on write */
/* the PCTel driver just does nothing when you try to write IIR, but I trapped it, so I knew it was happening */
/* not sure what to do with it yet so just keep the variable around static */
static unsigned char FCR;
static void put_uart_fcr(unsigned char c) {
    FCR = c;
}
EXPORT_SYMBOL(put_uart_fcr);


/* dll/dlm - again, I think these are meaningless for a softmodem, but I will */
/* implement them as registers with no side effects */
static unsigned char DLL;
static unsigned char get_uart_dll(void) {
    return DLL;
}
static void put_uart_dll(unsigned char c) {
    DLL = c;
}

EXPORT_SYMBOL(get_uart_dll);
EXPORT_SYMBOL(put_uart_dll);

static unsigned char DLM;
static unsigned char get_uart_dlm(void) {
    return DLM;
}
static void put_uart_dlm(unsigned char c) {
    DLM = c;
}

EXPORT_SYMBOL(get_uart_dlm);
EXPORT_SYMBOL(put_uart_dlm);

void esscom_hw_setup (u16 iobase, u16 irq, int country_code) {
    IDMA_ADD = gwBaseEDSP = gwBaseIO = iobase;
    IDMA_DATA = iobase + 4;
    ESSIRQ = irq;

    /* set country from the module parameter */
    CountryCode = country_code;

    /* also permit the country to be overridden via at commands */
    PTTFuncOption[0] = 0xff;

    info("setting up iobase=%x, irq=%x\n", iobase, irq);

    esscom_initialize();

    info("initialization sequence complete\n");

}

EXPORT_SYMBOL(esscom_hw_setup);

/* to keep essserial clean of binary interfaces */
void esscom_hw_interrupt (void) {

    rs_interrupt_single();
}

EXPORT_SYMBOL(esscom_hw_interrupt);

/* work to do when the timer goes off */
/* a translation of ess_rs_timer */
void esscom_hw_timer_tick (void) {

    /*
     * old driver added 10 each time timer_tick went off.  Unfortunately we don't get
     * a tick exactly every 10ms if HZ is not a multiple of 100, so instead add the
     * number of ms since the last tick
     */

    timer_cnt += (1000 / HZ) * (jiffies - last_tick_jiffies);
    last_tick_jiffies = jiffies;

    sys_timer_cnt = Get_System_Time();   /* this is one of those twice-scaled (once ess, once HZ change) time values */

    modem_task();

}

EXPORT_SYMBOL(esscom_hw_timer_tick);

/**
 * Module init function.  Doesn't need to do anything, just reports a string
 * indicating the driver version with the given compile options has been
 * loaded.
 */
static int __init essserial_hw_init( void )
{
        info( "Ess hardware driver version %s for %s\n",
              VERSION, ess_options );
        return 0;
}

/**
 * Module exit function.  Doesn't need to do anything, just reports the driver
 * version has been unloaded.
 */
static void __exit essserial_hw_exit( void )
{
        info( "Unloading Ess hardware driver version %s\n", VERSION );
}

module_init(essserial_hw_init);
module_exit(essserial_hw_exit);


MODULE_LICENSE("GPL linked with proprietary libraries");
MODULE_DESCRIPTION("ESS hardware modem driver: binary part of driver");

