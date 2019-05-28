//------------------------------------------------------------------------------------------
// Uses an STCmicro STC15W4K32S4 series micro controller to turn an IBM Wheelwriter Electronic
// Typewriter into a teletype-like device.
//
// switch 1    off - linefeed only upon receipt of linefeed character (0x0A)
//             on  - auto linefeed; linefeed is performed with each carriage return (0x0D)
// switch 2    not used
//
// switch 3    not used
//
// switch 4    not used
//
//------------------------------------------------------------------------------------------

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <reg51.h>
#include <stc51.h>
#include <intrins.h>
#include "control.h"
#include "uart2.h"
#include "uart3.h"
#include "uart4.h"
#include "wheelwriter.h"

#define BAUDRATE 9600                   // UART2 serial console at 9600 bps

#define FIFTEENCPI 8                    // number of micro spaces for each character on the 15P printwheel (15 cpi)
#define TWELVECPI 10                    // number of micro spaces for each character on the 12P printwheel (12 cpi)
#define TENCPI 12                       // number of micro spaces for each character on the 10P printwheel (10 cpi)

#define FALSE 0
#define TRUE  1
#define LOW 0
#define HIGH 1
#define ON 0                            // 0 turns the LEDs on
#define OFF 1                           // 1 turns the LEDs off

// 12,000,000 Hz/12 = 1,000,000 Hz = 1.0 microsecond clock period
// 50 milliseconds per interval/1.0 microseconds per clock = 50,000 clocks per interval
#define RELOADHI (65536-50000)/256
#define RELOADLO (65536-50000)&255
#define ONESEC 20                       // 20*50 milliseconds = 1 second

bit switch1 =  OFF;                     // dip switch connected to pin 39 0=on, 1=off (auto LF after CR if on)  
bit switch2 =  OFF;                     // dip switch connected to pin 38 0=on, 1=off (not used)
bit switch3 =  OFF;                     // dip switch connected to pin 37 0=on, 1=off (not used)
bit switch4 =  OFF;                     // dip switch connected to pin 36 0=on, 1=off (not used)  

sbit POR      = P0^4;                   // Power-On-Reset output pin 5
sbit redLED   = P0^5;                   // red   LED connected to pin 6 0=on, 1=off
sbit amberLED = P0^6;                   // amber LED connected to pin 7 0=on, 1=off
sbit greenLED = P0^7;                   // green LED connected to pin 8 0=on, 1=off

bit passThrough = TRUE;
bit errorLED = FALSE;                   // makes the red LED flash when TRUE
unsigned char attribute = 0;            // bit 0=bold, bit 1=continuous underline, bit 2=multiple word underline
unsigned char column = 1;               // current print column (1=left margin)
unsigned char tabStop = 5;              // horizontal tabs every 5 spaces (every 1/2 inch)
volatile unsigned char timeout = 0;     // decremented every 50 milliseconds, used for detecting timeouts
volatile unsigned char hours = 0;       // uptime hours
volatile unsigned char minutes = 0;     // uptime minutes
volatile unsigned char seconds = 0;     // uptime seconds

code char title[]     = "Wheelwriter Teletype Version 1.0.0";
code char mcu[]       = "STCmicro IAP15W4K61S4 MCU";
code char compiled[]  = "Compiled on " __DATE__ " at " __TIME__;
code char copyright[] = "Copyright 2019 Jim Loos";

//------------------------------------------------------------
// Timer 0 ISR: interrupt every 50 milliseconds, 20 times per second
//------------------------------------------------------------
void timer0_isr(void) interrupt 1 using 1{
    static char ticks = 0;

    TL0 = RELOADLO;     			// load timer 0 low byte
    TH0 = RELOADHI;     			// load timer 0 high byte

    if (timeout) {                  // countdown value for detecting timeouts
        --timeout;
    }

    if(++ticks == 20) { 		    // if 20 ticks (one second) have elapsed...
        ticks = 0;

        if (errorLED) {             // if there's an error 
           redLED = !redLED;        // toggle the red LED once each second
        }

        if (++seconds == 60) {		// if 60 seconds (one minute) has elapsed...
            seconds = 0;
            if (++minutes == 60) {	// if 60 minutes (one hour) has elapsed...
                minutes = 0;
                ++hours;
            }
        }
    }
}

//------------------------------------------------------------------------------------------
// The Wheelwriter prints the character and updates column.
// Carriage return cancels bold and underlining.
// Linefeeds automatically printed with carriage return if switch 1 is on.
// The character printed by the Wheelwriter is echoed to the serial port (for monitoring).
//
// Control characters:
//  BEL 0x07    spins the printwheel
//  BS  0x08    non-destructive backspace
//  TAB 0x09    horizontal tab to next tab stop
//  LF  0x0A    moves paper up one line
//  VT  0x0B    moves paper up one line
//  CR  0x0D    returns carriage to left margin, if switch 1 is on, moves paper up one line (linefeed)
//  ESC 0x1B    see Diablo 630 commands below...
//
// Diablo 630 commands emulated:
// <ESC><O>  selects bold printing for one line
// <ESC><&>  cancels bold printing
// <ESC><E>  selects continuous underlining  (spaces between words are underlined) for one line
// <ESC><R>  cancels underlining
// <ESC><X>  cancels both bold and underlining
// <ESC><U>  half line feed (paper up 1/2 line)
// <ESC><D>  reverse half line feed (paper down 1/2 line)
// <ESC><BS> backspace 1/120 inch
// <ESC><LF> reverse line feed (paper down one line)
//
// printer control not part of the Diablo 630 emulation:
// <ESC><u>  selects micro paper up (1/8 line or 1/48")
// <ESC><d>  selects micro paper down (1/8 line or 1/48")
// <ESC><b>  selects broken underlining (spaces between words are not underlined)
// <ESC><p>  selects Pica pitch (10 characters/inch or 12 point)
// <ESC><e>  selects Elite pitch (12 characters/inch or 10 point)
// <ESC><m>  selects Micro Elite pitch (15 characters/inch or 8 point)
//
// diagnostics/debugging:
// <ESC><^Z><c> print (on the serial console) the current column
// <ESC><^Z><k> print (on the serial console) the state of the keyboard pass-through flag
// <ESC><^Z><r> reset the DS89C440 microcontroller
// <ESC><^Z><u> print (on the serial console) the uptime as HH:MM:SS
// <ESC><^Z><w> print (on the serial console) the number of watchdog resets
// <ESC><^Z><e><n> turn flashing red error LED on or off (n=1 is on, n=0 is off)
// <ESC><^Z><p><n> print (on the serial console) the value of Port n (0-3) as 2 digit hex number
//-------------------------------------------------------------------------------------------
void print_character(unsigned char charToPrint) {
    static char escape = 0;                                 // escape sequence state
    char i,t;

    switch (escape) {
        case 0:
            switch (charToPrint) {
                case NUL:
                    break;
                case BEL:
                    ww_spin();
                    putchar(BEL);
                    break;
                case BS:
                    if (column > 1){                        // only if there's at least one character on the line
                        ww_backspace();
                        --column;                           // update column
                        putchar(BS);
                    }
                    break;
                case HT:
                    t = tabStop-(column%tabStop);           // how many spaces to the next tab stop
                    ww_horizontal_tab(t);                   // move carrier to the next tab stop
                    for(i=0; i<t; i++){
                        ++column;                           // update column
                        putchar(SP);
                    }
                    break;
                case LF:
                    ww_linefeed();
                    putchar(LF);
                    break;
                case VT:
                    ww_linefeed();
                    break;
                case CR:
                    ww_carriage_return();                   // return the carrier to the left margin
                    column = 1;                             // back to the left margin
                    attribute = 0;                          // cancel bold and underlining
                    if (!switch1) {                         // if switch 2 is on, automatically print linefeed
                        ww_linefeed();
                    }
                    putchar(CR);
                    break;
                case ESC:
                    escape = 1;
                    break;
                default:
                    ww_print_letter(charToPrint,attribute);
                    putchar(charToPrint);                   // echo the character to the console
                    ++column;                               // update column
            } // switch (charToPrint)

            break;  // case 0:
        case 1:                                             // this is the second character of the escape sequence...
            switch (charToPrint) {
                case 'O':                                   // <ESC><O> selects bold printing
                    attribute |= 0x01;
                    escape = 0;
                    break;
                case '&':                                   // <ESC><&> cancels bold printing
                    attribute &= 0x06;
                    escape = 0;
                    break;
                case 'E':                                   // <ESC><E> selects continuous underline (spaces between words are underlined)
                    attribute |= 0x02;
                    escape = 0;
                    break;
                case 'R':                                   // <ESC><R> cancels underlining
                    attribute &= 0x01;
                    escape = 0;
                    break;
                case 'X':                                   // <ESC><X> cancels both bold and underlining
                    attribute = 0;
                    escape = 0;
                    break;                
                case 'U':                                   // <ESC><U> selects half line feed (paper up one half line)
                    ww_paper_up();
                    escape = 0;
                    break;
                case 'D':                                   // <ESC><D> selects reverse half line feed (paper down one half line)
                    ww_paper_down();
                    escape = 0;
                    break;
                case LF:                                    // <ESC><LF> selects reverse line feed (paper down one line)
                    ww_reverse_linefeed();
                    escape = 0;
                    break;
                case BS:                                    // <ESC><BS> backspace 1/120 inch
                    ww_micro_backspace();
                    escape = 0;
                    break;
                case 'b':                                   // <ESC><b> selects broken underline (spaces between words are not underlined)
                    attribute |= 0x04;
                    escape = 0;
                    break;
                case 'e':                                   // <ESC><e> selects Elite (12 characters/inch)
                    ww_set_printwheel(TWELVECPI);
                    tabStop =6;                             // tab stops every 6 characters (every 1/2 inch)
                    escape = 0;
                    break;
                case 'p':                                   // <ESC><p> selects Pica (10 characters/inch)
                    ww_set_printwheel(TENCPI);
                    tabStop =5;                             // tab stops every 5 characters (every 1/2 inch)
                    escape = 0;
                    break;
                case 'm':                                   // <ESC><m> selects Micro Elite (15 characters/inch)
                    ww_set_printwheel(FIFTEENCPI);
                    tabStop =7;                             // tab stops every 7 characters (every 1/2 inch)
                    escape = 0;
                    break;
                case 'u':                                   // <ESC><u> paper micro up (paper up 1/8 line)
                    ww_micro_up();
                    escape = 0;
                    break;
                case 'd':                                   // <ESC><d> paper micro down (paper down 1/8 line)
                    ww_micro_down();
                    escape = 0;
                    break;
                case '\x1A':                                // <ESC><^Z> for remote diagnostics
                    escape = 2;
                    break;  
            } // switch (charToPrint)
            break;  // case 1:
        case 2:                                             // this is the third character of the escape sequence...
            switch (charToPrint) {
                case 'c':                                   // <ESC><^Z><c> print current column
                    printf("%s %u\n","Column:",(int)column);
                    escape = 0;
                    break;
                case 'k':                                   // <ESC><^Z><k> print pass-through flag
                    if (passThrough)
                        printf("Wheelwriter key strokes go to Wheelwriter.\n");
                    else
                        printf("Wheelwriter key strokes go to serial console.\n");
                    escape = 0;
                    break;
                case 'e':                                   // <ESC><^Z><e> toggle red error LED
                    escape = 4;
                    break;
                case 'p':                                   // <ESC><^Z><p> print port values
                    escape = 3;
                    break;
                case 'r':                                   // <ESC><^Z><r> system reset
                    escape = 0;
                    break; 
                case 'u':                                   // <ESC><^Z><u> print uptime
                    printf("%s %02u%c%02u%c%02u\n","Uptime:",(int)hours,':',(int)minutes,':',(int)seconds);
                    escape = 0;
                    break; 
                case 'w':                                   // <ESC><^Z><w> print watchdog resets
                    escape = 0;
                    break;
            } // switch (charToPrint)
            break;  // case 2:
        case 3:                                             // this is the fourth character of the escape sequence
            switch (charToPrint){
                case '0':
                    printf("%s 0x%02X\n","P0:",(int)P0);    // <ESC><^Z><p><0> print port 0 value
                    escape = 0;
                    break;
                case '1':
                    printf("%s 0x%02X\n","P1:",(int)P1);    // <ESC><^Z><p><1> print port 1 value
                    escape = 0;
                    break;
                case '2':
                    printf("%s 0x%02X\n","P2:",(int)P2);    // <ESC><^Z><p><2> print port 2 value
                    escape = 0;
                    break;
                case '3':
                    printf("%s 0x%02X\n","P3:",(int)P3);    // <ESC><^Z><p><3> print port 3 value
                    escape = 0;
                    break;
            } // switch (charToPrint)
            break;  // case 3:
        case 4:
            if (charToPrint & 0x01) {
                errorLED = TRUE;                            // <ESC><^Z><e><n> odd values turn of n the LED on, even values turn the LED off
            }
            else {
                errorLED = FALSE;
                redLED = OFF;                               // turn off the red LED
            }
            escape = 0;
            break;  // case 4
    } // switch (escape)
}


/*------------------------------------------------------------------------------
Note that the two function below, getchar() and putchar(), replace the library
functions of the same name.  These functions use the interrupt-driven serial
I/O routines in uart2.c
------------------------------------------------------------------------------*/
// for scanf
char _getkey() {
    return getchar2();
}

// for printf
char putchar(char c)  {
   return putchar2(c);
}

//-----------------------------------------------------------
// main(void)
//-----------------------------------------------------------
void main(void){
	unsigned int loopcounter,wwData3,wwData4;
    unsigned char pitch = 0;
    unsigned char state = 0;
    char c;

    P0M1 = 0;                                                   // required to make port 0 work
    P0M0 = 0;

    POR = 1;                                                    // Power-On-Reset on

    amberLED = OFF;                                             // turn off the amber LED
    greenLED = OFF;                                             // turn off the green LED
    redLED = OFF;                                               // turn off the red LED

	TL0 = RELOADLO;                         	                // load timer 0 low byte
	TH0 = RELOADHI;              	                            // load timer 0 high byte
	TMOD = (TMOD & 0xF0) | 0x01; 	                            // configure timer 0 for mode 1 - 16 bit timer
	ET0 = 1;                     	                            // enable timer 0 interrupt
	TR0 = 1;                     	                            // run timer 0

    uart2_init(BAUDRATE);                                       // initialize UART2 for N-8-1 at 9600bps, RTS-CTS handshaking
    uart3_init();                                               // initialize UART3 for connection to the Function Board
    uart4_init();                                               // initialize UART4 for connection to the Printer Board

	EA = TRUE;                     	                            // global interrupt enable

    printf("\n%s\n%s\n%s\n%s\n\n",title,mcu,compiled,copyright);

    POR = 0;                                                    // Power-On-Reset off

    printf("Initializing...\n");
    timeout = 7*ONESEC;                                         // allow up to 7 seconds in case the carrier is at the right margin
    while(timeout) {                                            // loop for 7 seconds waiting for data from the Wheelwriter
        while (uart3_avail()){                                  // if there's a command from the function board...
            wwData3 = uart3_get_data();                         // retrieve the command
            uart4_send(wwData3);                                // relay the command to the printer board
            switch (state) {
                case 0:
                    if (wwData3 == 0x121)                       // was the command from the Function Board 0x121?
                        state = 1;
                    break;
                case 1:
                    if (wwData3 == 0x001)                       // was the 0x121 command followed by 0x001 (the "reset" command)?
                        state = 2;
                    else
                        state = 0;
                    break; 
            } 
        }
   	    while (uart4_avail()) {                                 // if there's a reply from the Printer Board...
            wwData4 = uart4_get_data();                         // retrieve the replay from the printer board
            uart3_send(wwData4);                                // relay replies from the Printer Board to the Function Board
            if (state==2) {                                     // if this is the reply to the reset command sent by the function board
               pitch = wwData4;                                 // the reply from the Printer Board to a reset command is the printwheel pitch
               state = 3;
               timeout = 1;
            }
	    }
    }

    switch (pitch) {
        case 0x000:
            ww_set_printwheel(TWELVECPI);
            tabStop = 6;                                        // tab stops every 6 characters (every 1/2 inch)
            printf("Unable to determine printwheel. Defaulting to 12P.\n");
            break;
        case 0x008:
            ww_set_printwheel(TWELVECPI);
            tabStop = 6;                                        // tab stops every 6 characters (every 1/2 inch)
            printf("PS printwheel\n");
            break;
        case 0x010:
            ww_set_printwheel(FIFTEENCPI);
            tabStop = 7;                                        // tab stops every 7 characters (every 1/2 inch)
            printf("15P printwheel\n");
            break;
        case 0x020:
            ww_set_printwheel(TWELVECPI);
            tabStop = 6;                                        // tab stops every 6 characters (every 1/2 inch)
            printf("12P printwheel\n");
            break;
        case 0x021:
            ww_set_printwheel(TWELVECPI);
            tabStop = 6;                                        // tab stops every 6 characters (every 1/2 inch)
            printf("No printwheel\n");
            break;
        case 0x040:
            ww_set_printwheel(TENCPI);
            tabStop = 5;                                        // tab stops every 5 characters (every 1/2 inch)
            printf("10P printwheel\n");
    }

    printf("Ready\n");

    //----------------- loop here forever -----------------------------------------
    while(TRUE) {

        if (++loopcounter==0) {                                 // every 65536 times through the loop (at about 2Hz)
            greenLED = !greenLED;                               // toggle the green "heart beat" LED
        }

        // check for commands from the function board
	    while (uart3_avail()) {                                // if there's data from the Function Board...
            wwData3 = uart3_get_data();
            if (passThrough) 
                uart4_send(wwData3);                            // relay commands from the Function Board to the Printer Board, do not wait for acknowledge
            else {
                uart3_send_ACK();                               // send Acknowledge to Function Board
                putchar2(ww_decode_keys(wwData3));              // print the ascii character on the serial console
            }
	    }

        // check for replies from the printer board
	    while (uart4_avail()) {                                 // if there's data from the Printer Board...
            wwData4 = uart4_get_data();
            uart3_send(wwData4);                                // relay replies from the Printer Board to the Function Board
	    }

        if (char_avail2()) {                                    // if there is a character in the serial receive buffer...
            c = getchar2();                                     // retrieve the character
            if (c==0x10)                                        // control P  toggles the "Pass through" flag
               passThrough = !passThrough;
            else 
               print_character(c);                              // print it on the Wheelwriter
        }
    }
}