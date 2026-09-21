#ifndef __uart_h__
#define __uart_h__

#include "stdio.h"

void uart1_init();
void uart1_send_data(unsigned char _data);

extern data  unsigned char uart4_rx_idle;
extern data volatile unsigned char uart4_tx_idle;
extern xdata unsigned char uart4_rx_buf[];

extern xdata unsigned char uart3_tx_buf[];

extern xdata unsigned char uart3_rx_buf[];
extern data  unsigned char uart3_rx_idle;
extern xdata unsigned char uart2_rx_buf[];
extern data  unsigned char uart2_rx_idle;
extern struct 
{
    unsigned char w_left;
    unsigned char r_free;
    unsigned char code * wbuf_ptr;
    unsigned char xdata * rbuf_ptr;
} uart4_buf;

extern struct 
{
    unsigned char w_left;
    unsigned char r_free;
    unsigned char xdata * wbuf_ptr;
    unsigned char xdata * rbuf_ptr;
} uart2_buf, uart3_buf;
void uart2_init();
void uart3_init();
void uart4_init();
void uart4_send_data(char _data);
void uart4_send_array(unsigned char *_array, unsigned char len);
void uart3_send_array(unsigned char *_array, unsigned char len);
void uart2_send_array(unsigned char *_array, unsigned char len);
void uart4_restart_rx(void);
void uart3_restart_rx(void);
void uart2_restart_rx(void);

#endif


/*********************************FILE END**********************************/
