//************************************************************************//
// Interrupt driven UART1 functions.                                      //
// for the Keil C51 Compiler                                              //
//                                                                        //
// UART1 uses a receive buffer in internal MOVX SRAM.                     //
// UART1 uses the Timer 1 for baud rate generation. init_uart1 must be    //
// called before using functions. No syntax error handling.               //
// RxD on pin 21, TxD on pin 22, No handshaking.                          //
//************************************************************************//

#include <reg51.h>
#include "stc51.h"

#define FALSE 0
#define TRUE  1
#define FOSC 12000000L                          // 12 MHz system clock frequency
#define RBUFSIZE1 128                           // receive buffer size

#if RBUFSIZE1 < 32
    #error RBUFSIZE1 may not be less than 32.
#elif RBUFSIZE1 > 256
    #error RBUFSIZE1 may not be greater than 256.
#elif ((RBUFSIZE1 & (RBUFSIZE1-1)) != 0)
    #error RBUFSIZE1 must be a power of 2.
#endif

volatile unsigned char rx1_head;                // receive interrupt index for UART1
volatile unsigned char rx1_tail;                // receive read index for UART1
volatile unsigned char xdata rx1_buf[RBUFSIZE1];// receive buffer for UART1 in internal MOVX RAM
volatile bit tx1_ready;

// ---------------------------------------------------------------------------
// UART1 interrupt service routine
// ---------------------------------------------------------------------------
void uart1_isr(void) interrupt 4 using 2 {

   // uart1 transmit interrupt
   if (TI) {                                    // transmit interrupt?
      TI = FALSE;                               // clear transmit interrupt flag
      tx1_ready = TRUE;                         // transmit buffer is ready for a new character
    }

    // uart1 receive interrupt
    if(RI) {                                    // receive character?
        RI = 0;                                 // clear serial receive interrupt flag
        rx1_buf[rx1_head] = SBUF;               // Get character from serial port and put into UART1 fifo.
            if (++rx1_head == RBUFSIZE1) rx1_head = 0;// wrap pointer around to the beginning
    }
}

// ---------------------------------------------------------------------------
//  Initialize UART1 for mode 1 using timer 1 for baud rate generation
// ---------------------------------------------------------------------------
void uart1_init(unsigned long baudrate) {
    rx1_head = 0;                               // initialize UART1 buffer head/tail pointers
    rx1_tail = 0;
    tx1_ready = TRUE;

    TMOD = 0x00;                                // T1 in mode 0 (16-bit auto-relaod timer/counter)
    AUXR = 0x40;                                // T1 in 1T mode
    TL1 = (65536-(FOSC/4/baudrate));            // low byte of preload
    TH1 = (65536-(FOSC/4/baudrate))>>8;         // high byte of preload
    TR1 = 1;                                    // enable Timer 1

    SCON = 0x50;                                // UART1 Mode 1
    REN = TRUE;                                 // set REN to enable reception
    TI = TRUE;                                  // set TI to enable transmition
    RI  = FALSE;                                // clear recieve interrupt flag
    ES = TRUE;                                  // enable UART1 serial interrupt.
    EA = TRUE;                                  // enable global interrupt
}

// ---------------------------------------------------------------------------
// returns 1 if there are character waiting in the UART1 receive buffer
// ---------------------------------------------------------------------------
char char_avail1(void) {
   return (rx1_head != rx1_tail);
}

//-----------------------------------------------------------
// waits until a character is available in the UART1 receive
// buffer. returns the character. does not echo the character.
//-----------------------------------------------------------
char getchar1(void) {
    unsigned char buf;

    while (rx1_head == rx1_tail);               // wait until a character is available
    buf = rx1_buf[rx1_tail];
    if (++rx1_tail == RBUFSIZE1) rx1_tail = 0;
    return(buf);
}

// ---------------------------------------------------------------------------
// output one character from UART1. waits until ready to transmit.
// ---------------------------------------------------------------------------
char putchar1(char c)  {
    while (!tx1_ready);                         // wait until ready to transmit
    SBUF = c;
    tx1_ready = 0;
    return (c);
}

// ---------------------------------------------------------------------------
// output a string from UART1
// ---------------------------------------------------------------------------
void puts1(char *s) {
    while (s && *s)
        putchar1 (*s++);
}