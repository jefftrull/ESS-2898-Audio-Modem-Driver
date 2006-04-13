/* declarations of stuff from the binary */

extern void rs_interrupt_single(void);
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

extern unsigned  int EFlags;
extern unsigned char MiscFlags;
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
extern unsigned char mytmp_buf[0x400];
extern unsigned int (*proc_dte_output)(unsigned char *, unsigned int) __attribute__((regparm(0)));
extern unsigned int proc_output(unsigned char *, unsigned int) __attribute__((regparm(0)));
extern unsigned int proc_info_output(unsigned char *, unsigned int) __attribute__((regparm(0)));
extern unsigned int proc_v80_output(unsigned char *, unsigned int) __attribute__((regparm(0)));
extern unsigned short myRxV80[];
extern unsigned short r_info_buf_put, r_info_buf_get;

extern void reset_all_black_list(void);
extern void init_PTT_Sreg(void);
extern void SetUpDAADefault(void);


/* from powerup_get_PTT_country */
extern unsigned long PTT_onhook_timer;
extern void AdjustOnhookTime(unsigned long *) __attribute__((regparm(0)));
extern unsigned int *ToneTable, dtmf_od[];
extern unsigned int CardID;
extern unsigned char s[];
extern unsigned int RealID;
extern unsigned char CodecType;
extern unsigned char PTTFuncOption[];
extern unsigned char CountryCode;
extern unsigned char s_parm[];
extern unsigned char GlobalOption;
extern void GetCountryParameter(void), PTT_s_parm_copy(void), DAAStatus(void);
extern void SetSpeakerVolum(unsigned int) __attribute__((regparm(0)));
extern unsigned int startSystemTime;
extern void ReadBlackList(void);

/* access to internal structures for the VUART */
extern unsigned int dte_output_buf_put, dte_output_buf_get;

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

/* BOZO debug stuff only, for printing purposes */
extern unsigned int Card;
extern unsigned int RealID;
extern unsigned int CardID;
extern unsigned long HSP_Flag;
extern unsigned int No_LCS;
extern unsigned int FunctionID;
extern void proc_cmd_input(void);
extern void wait_for_CM(void);
extern void wait_for_connect(void);
extern void idle(void);
extern void check_dte_input_buf(void);
extern void wait_to_answer(void);
extern void pause(void);
extern void flash(void);
extern void forced_wait_for_dial_tone(void);
extern void wait_for_AnsTone_Busy_RB(void);
extern void pre_dtmf_dialing(void);
extern void check_escape(void);
extern void wait_for_online(void);
extern void wait_for_disconnect(void);
extern void process_command(void);
extern void dtmf_dialing(void);
extern void wait_DTMF_complete(void);
extern void data_train(void);
extern void fax_train(void);
extern void wait_real_connect(void);
extern void PTT_Offhook_Surge_Blance(void);
extern void wait_for_dial_tone(void);
extern unsigned short proc_cmd_flag, at, at_cmd_line_len, escape_sequence_found, at_cmd_line_ptr, result_code;
extern unsigned long dte_input_buf_get, dte_input_buf_put;
extern unsigned char dte_input_buf[], dte_output_buf[], last_ch;
extern unsigned long ties_check_at(unsigned char) __attribute__((regparm(0)));
extern void ties_return_online(void) __attribute__((regparm(0)));
extern void connect_printout(void) __attribute__((regparm(0)));
extern void output_result(void) __attribute__((regparm(0)));
extern void pack_cmd_space(void) __attribute__((regparm(0)));
extern unsigned char tolower(unsigned char) __attribute__((regparm(0)));
extern unsigned char at_cmd_line[];
extern unsigned long ties_check_at_cmd(unsigned char *, unsigned short) __attribute__((regparm(0)));
extern unsigned long at_cmd_timer_start(unsigned long *) __attribute__((regparm(0)));
extern unsigned long dte_action_timer;
extern void proc_input(void);

/* for debugging wait_for_connect */
extern unsigned short service_class, spk_flag, DialTone_Cnt, int_trans_size, sm_flag, download_pcm_flag, max_bit_rate;
extern unsigned short data_fax1_auto_answer_mode, data_fax2_auto_answer_mode, bit_rate, op_mode, tbit_rate;
extern unsigned short carrier_off, carrier_off_timer_expired;
extern unsigned long HSP_Flag, myVoiceConnection, received_bit_rate, transmit_bit_rate, dwCtMode, at_cmd_timer;
extern void stop_modem(unsigned long) __attribute__((regparm(0)));
extern unsigned short voice_conf[], current_conf[];
extern void modifyDM(unsigned long, unsigned short *, unsigned long) __attribute__((regparm(0)));
extern unsigned int jumpProgram(unsigned long) __attribute__((regparm(0)));
extern unsigned char PTT_CP_Report;
extern void DownLoadFirmwarePcm(unsigned long, unsigned long) __attribute__((regparm(0)));
extern unsigned long readMem(unsigned short *, unsigned long, unsigned short) __attribute__((regparm(0)));
extern void verify_dsp_memory(unsigned long, unsigned long) __attribute__((regparm(0)));
/* readmem args: mem base addr, base in card mem (?), count */
extern unsigned long CUdBitRate(unsigned short, unsigned long) __attribute__((regparm(0)));
extern unsigned long cModemState[];
extern unsigned char ATUDreport[];
extern void PTT_connect_event(void);
extern void offSpeaker(void);
extern void init_on_line(unsigned long)__attribute__((regparm(0)));
extern void init_v80_online(void);
extern unsigned long at_cmd_timer_expired(unsigned long, unsigned long) __attribute__((regparm(0)));
extern unsigned long dte_action_detected(void);
extern unsigned short r_fax_mode;
extern void AddGreyLine(void);

/* for debugging init_on_line */
extern void init_proc_inf_var(void);
extern unsigned long timer_start(unsigned long *) __attribute__((regparm(0)));
extern unsigned long connect_timer, t_1;
extern unsigned short role, error_control_mode;
extern void init_fax_tx(void);
extern void init_fax_rx(void);
extern void set_dcd(void);
extern void set_HDLC_frame(void);
extern void start_send_CSI_DIS(void), start_recv_CSI_DIS(void);
extern void init_v14_var(void), init_v42_var(void), init_mnp_var(void);
extern void InitRateRenegotiation(void);

extern unsigned long rx_good_frame_total, tx_good_frame_total, rx_good_frame_cntr, tx_good_frame_cntr, rx_retransmit_total, tx_retransmit_total, rx_retransmit_cntr, tx_retransmit_cntr;

