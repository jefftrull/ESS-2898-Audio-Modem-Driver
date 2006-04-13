/*
 * vuart.h - definitions for the virtual uart defined to simplify communication with the hw-independent driver
 * Copyright (C) 2006 Jeffrey Elliot Trull
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

extern unsigned char get_uart_rx(void);
extern unsigned char get_uart_ier(void);
extern unsigned char get_uart_iir(void);
extern unsigned char get_uart_lcr(void);
extern unsigned char get_uart_mcr(void);
extern unsigned char get_uart_lsr(void);
extern unsigned char get_uart_msr(void);
extern unsigned char get_uart_scr(void);
extern unsigned char get_uart_dll(void);
extern unsigned char get_uart_dlm(void);

extern void put_uart_tx(unsigned char c) ;
extern void put_uart_ier(unsigned char c);
extern void put_uart_fcr(unsigned char c);
extern void put_uart_lcr(unsigned char c);
extern void put_uart_mcr(unsigned char c);
extern void put_uart_lsr(unsigned char c);
extern void put_uart_msr(unsigned char c);
extern void put_uart_scr(unsigned char c);
extern void put_uart_dll(unsigned char c);
extern void put_uart_dlm(unsigned char c);


