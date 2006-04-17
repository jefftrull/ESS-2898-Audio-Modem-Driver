/* ess.h - header file for interfaces into essserial_hw code */
/*
 *  Copyright (C) 2006 Jeffrey Elliot Trull
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

/* from essserial_hw.c */
extern void esscom_hw_setup(u16, u16, int);
extern void esscom_hw_interrupt(void);
extern void esscom_hw_timer_tick(void);
extern void esscom_hw_shutdown(void);
extern int  esscom_hw_check_modem(int vendor);

