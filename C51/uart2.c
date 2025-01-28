//************************************************************************//
// Interrupt driven UART2 functions with RTS/CTS handshaking.             //
// for the Keil C51 Compiler                                              //
//                                                                        //
// UART2 uses a receive buffer in internal MOVX SRAM.                     //
// UART2 uses the Timer 2 for baud rate generation. init_uart2 must be    //
// called before using functions. No syntax error handling.               //
// RxD2 on pin 9, TxD2 on pin 10, RTS on pin 11, CTS on pin 12            //
//************************************************************************//

#include <reg51.h>
#include "stc51.h"

#define FALSE 0
#define TRUE  1
#define FOSC 12000000L                          // 12 MHz system clock frequency
#define RBUFSIZE2 128                           // must be 256,128,64 or 32 bytes

#if RBUFSIZE2 < 32
    #error RBUFSIZE2 may not be less than 32.
#elif RBUFSIZE2 > 256
    #error RBUFSIZE2 may not be greater than 256.
#elif ((RBUFSIZE2 & (RBUFSIZE2-1)) != 0)
    #error RBUFSIZE2 must be a power of 2.
#endif

#define PAUSELEVEL RBUFSIZE2/4                  // pause communications to avoid overflow (RTS = 1) when buffer space < 64 bytes
#define RESUMELEVEL RBUFSIZE2/2                 // resume communications (RTS = 0) when buffer space > 128 bytes

sbit CTS = P1^3;                                // CTS input on pin 12 (not used)
sbit RTS = P1^2;                                // RTS output on pin 11

volatile unsigned char rx2_head;                // index used to fill receive buffer
volatile unsigned char rx2_tail;                // index used to empty receive buffer
volatile unsigned char rx2_remaining;           // receive buffer space remaining
volatile unsigned char xdata rx2_buf[RBUFSIZE2];// receive buffer  in internal MOVX RAM
volatile bit tx2_ready;                         // set when ready to transmit

// ---------------------------------------------------------------------------
// UART2 interrupt service routine
// ---------------------------------------------------------------------------
void uart2_isr(void) interrupt 8 using 3 {

    // UART2 transmit interrupt
    if (S2TI) {                                 // is this a transmit interrupt?
      CLR_S2TI;                                 // clear transmit interrupt flag
      tx2_ready = TRUE;                         // transmit buffer is ready for a new character
    }

    // UART2 receive interrupt
    if(S2RI) {                                  // is this a receive interrupt?
       CLR_S2RI;                                // clear receive interrupt flag
       rx2_buf[rx2_head++ & (RBUFSIZE2-1)] = S2BUF;// get character from serial port and put into serial fifo.
       --rx2_remaining;                         // space remaining in UART2 buffer decreases
       if (!RTS){                               // if communications is not now paused...
         if (rx2_remaining < PAUSELEVEL) {      // if the remaining buffer space is low...
               RTS = 1;                         // pause communications when space in UART2 buffer decreases to less than 64 bytes
            }
      }
    }
}

// ---------------------------------------------------------------------------
//  Initialize UART2 for mode 1 using timer 2 for baud rate generation
// ---------------------------------------------------------------------------
void uart2_init(unsigned long baudrate) {
    rx2_head = 0;                               // initialize UART2 buffer head/tail pointers.
    rx2_tail = 0;
    rx2_remaining = RBUFSIZE2;

    CLR_T2_CT;                                  // clear T2_C/T to make Timer 2 operate as timer instead of counter
    SET_T2x12;                                  // set T2x12=1 to make Timer 2 operate in 1T mode.
    T2L = (65536-(FOSC/4/baudrate));            // low byte of preload
    T2H = (65536-(FOSC/4/baudrate))>>8;         // high byte of preload
    SET_T2R;                                    // enable Timer 2

    S2CON = 0x50;                               // UART2 for mode 1
    SET_S2REN;                                  // set S2REN to enable reception
    SET_S2TI;                                   // set S2TI to enable transmition
    CLR_S2RI;                                   // clear recieve interrupt flag
    SET_ES2;                                    // enable UART2 serial interrupt
    EA = TRUE;                                  // enable global interrupt

    RTS = 0;                                    // clear RTS to allow transmissions from remote console
}

// ---------------------------------------------------------------------------
// returns 1 if there is a character waiting in the UART2 receive buffer
// ---------------------------------------------------------------------------
char char_avail2(void) {
   return (rx2_head != rx2_tail);
}

//-----------------------------------------------------------
// waits until a character is available in the UART2 receive
// buffer. returns the character. does not echo the character.
//-----------------------------------------------------------
char getchar2(void) {
   unsigned char buf;

   while (rx2_head == rx2_tail);                // wait until a character is available
   buf = rx2_buf[rx2_tail++ &(RBUFSIZE2-1)];
   ++rx2_remaining;                             // space remaining in buffer increases
   if (RTS) {                                   // if communications is now paused...
      if (rx2_remaining > RESUMELEVEL) {
         RTS = 0;                               // clear RTS to resume communications when space remaining in buffer increases above 128 bytes
      }
   }
    return(buf);
}

// ---------------------------------------------------------------------------
// sends one character out UART2. waits until ready to transmit.
// ---------------------------------------------------------------------------
char putchar2(char c)  {
   while (!tx2_ready);                          // wait here for transmit ready
   S2BUF = c;
   tx2_ready = 0;
   return (c);
}

