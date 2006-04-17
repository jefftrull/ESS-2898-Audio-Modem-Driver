/* declarations of stuff from the binary */
/*
 *  Copyright (C) 2006 Jeffrey Elliot Trull
 */

/* It seems clear that the part of the driver that links to the binary cannot be GPL */
/* this is part of that section of the driver, as it describes the binary interfaces */

/* for the interrupt handler */
extern void rs_interrupt_single(void);

/* for esscom_initialize */
extern void ReadID(char *);   /* the binary calls this routine with a parameter, but it is not used! */
extern void init_at_var(void);
extern unsigned short gwBaseEDSP, gwBaseIO, IDMA_ADD, IDMA_DATA;

extern int DownLoadFirmware(unsigned short, unsigned short) __attribute__((regparm(0)));
extern void init_s_reg(void);
extern void action_for_restore_s(void);
extern void init_modem(void);
extern unsigned int intEnaFlag;
extern unsigned int timeout;
extern unsigned int timer_cnt;
extern unsigned long sys_timer_cnt;
extern unsigned int BreakFlag;
extern short ESSIRQ;
extern unsigned int AddrMSRShadow;
extern unsigned int EFlags;
extern unsigned char MiscFlags;
extern unsigned char CountryCode;
extern unsigned char PTTFuncOption[];

/* for esscom_hw_shutdown */
extern void stop_modem(unsigned long) __attribute__((regparm(0)));

/* for modem_task */
extern unsigned char modem_status, last_modem_status;
extern unsigned char get_modem_status(void);
extern void (*proc_dte_input)(void), proc_v80_input(void);
extern unsigned char critical_section;
extern void Update_PTT_timer(void);
extern void (*at_task)(void);
extern void PttSilenceDetect(void);
extern void SilenceReport(void);
extern unsigned short online;
extern void check_clrdwn(void), check_connect_timer(void), check_carrier_off(void);
extern void (*online_task)(void);
extern void check_own_busy(void), check_tran_quality(void), check_rate_renegotiation(void), PTT_Ring_Detector(void), daa_task(void);
extern unsigned int (*proc_dte_output)(unsigned char *, unsigned int) __attribute__((regparm(0)));

/* access to internal structures for the VUART */
extern unsigned int dte_output_buf_put, dte_output_buf_get;
extern unsigned int proc_output(unsigned char *, unsigned int) __attribute__((regparm(0)));
extern unsigned int proc_info_output(unsigned char *, unsigned int) __attribute__((regparm(0)));
extern unsigned int proc_v80_output(unsigned char *, unsigned int) __attribute__((regparm(0)));
extern unsigned short myRxV80[];
extern unsigned short r_info_buf_put, r_info_buf_get;

struct esscom_port_struct {
    unsigned char *dummy1; /*0x0*/
    int dummy2; /*0x4*/
    unsigned char *xmit_buf; /*0x8*/
    unsigned int last_untransmitted_ptr; /*0xc*/
    int dummy3; /*0x10*/
    int dummy4; /*0x14*/
    int dummy5; /*0x18*/
    unsigned int untransmitted_char_cnt; /*0x1c*/
    unsigned int first_untransmitted_ptr; /*0x20*/
    unsigned int mystery_ptr; /*0x24*/
};

extern struct esscom_port_struct *myPort;

extern struct esscom_struct {
    unsigned int dummy1[6];
    struct tty_struct *tty;  /*0x18*/
    unsigned int dummy2[15];
    unsigned char *xmit_buf; /*0x58*/
} *esscom_info;  /* this is 0x14 in data segment in binary */

/* for printing purposes */
extern unsigned int Card;
extern unsigned int RealID;
extern unsigned int CardID;
extern unsigned long HSP_Flag;
extern unsigned int No_LCS;
extern unsigned int FunctionID;
