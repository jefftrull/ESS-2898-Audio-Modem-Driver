/* ess.h - header file for interfaces into essserial_hw code */

/* from essserial_hw.c */
extern void esscom_hw_setup(u16, u16);
extern void esscom_do_interrupt(void);
extern void esscom_hw_timer_tick(struct linmodem_port *);
