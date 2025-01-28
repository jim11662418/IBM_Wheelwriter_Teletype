//------------------------------------------------------------------------------------------
// Uses an STCmicro IAP15W4K32S4 series micro controller to turn an IBM Wheelwriter Electronic
// Typewriter into a teletype-like device.
//
// For the Keil C51 compiler.
//
// UART1 used for debugging, monitor and in-application-programming - 115200bps N-8-1
// UART2 used for communications with host PC - 9600bps N-8-1
// UART3 used for communications with Wheelwriter Function Board
// UART4 used for communications with Wheelwriter Printer Board
//
// Version 1.0.0 - Initial version
// Version 1.1.0 - Changes necessary to make compatible with Wheelwriter 3, 5 and 6
// Version 1.1.1 - Renamed some functions and variables for added readability
// Version 1.2.0 - Watch Dog Timer added
// Version 1.3.0 - reworked main() and ww_decode_key() functions
// Version 1.3.1 - removed wheelwriter model detection code
// Version 1.3.2 - revised line/local mode, diagnostics display
// Version 1.3.3 - revised wheelwriter.c for ASCII Printwheel (part no. 1353909)
// Version 1.3.4 - use UART1 for debugging and monitor
// Version 1.3.5 - SDCC version
//
// NOTE: When using STCmicro's stc-isp application to download object code to the MCU,
//       make sure the internal clock frequency is set to 12 MHz.
//
//------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------
// Copyright 2019-2025 Jim Loos
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
// (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
// publish, distribute, sub-license, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
// IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//-------------------------------------------------------------------------------------------------------------------------------------

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <reg51.h>
#include "stc51.h"
#include "control.h"
#include "uart1.h"
#include "uart2.h"
#include "ww-uart3.h"
#include "ww-uart4.h"
#include "wheelwriter.h"

#define FALSE 0
#define TRUE  1
#define LOW 0
#define HIGH 1
#define ON 0                                                // 0 turns the LEDs on
#define OFF 1                                               // 1 turns the LEDs off

// 12,000,000 Hz/12 = 1,000,000 Hz = 1.0 microsecond clock period
// 50 milliseconds per interval/1.0 microseconds per clock = 50,000 clocks per interval
#define RELOADHI (65536-50000)/256
#define RELOADLO (65536-50000)&255
#define ONESEC 20                                           // 20*50 milliseconds = 1 second

sbit redLED   = P0^5;                                       // red   LED connected to pin 6 0=on, 1=off
sbit amberLED = P0^6;                                       // amber LED connected to pin 7 0=on, 1=off
sbit greenLED = P0^7;                                       // green LED connected to pin 8 0=on, 1=off

bit autoLineFeed = FALSE;                                   // when true, automatically print a linefeed with each carriage return received from the serial port
bit autoCarriageReturn = FALSE;                             // when true, automatically print a carriage return with each linefeed received from the serial port (for Linux)
bit errorLED = FALSE;                                       // makes the red LED flash when TRUE
bit initializing = TRUE;                                    // makes all three LEDs flash during initialization
bit monitor = FALSE;                                        // monitor communications between function and printer boards
bit localMode = TRUE;                                       // when true wheelwriter keystrokes go to wheelwriter, when false wheelwriter keystrokes go to serial console

unsigned char attribute = 0;                                // bit 0=bold, bit 1=continuous underline, bit 2=multiple word underline
unsigned char column = 1;                                   // current print column (1=left margin)
unsigned char tabStop = 5;                                  // horizontal tabs every 5 spaces (every 1/2 inch)
unsigned char printWheel = 0;                               // 10pt, 12pt, 15pt or PS

extern unsigned char uSpacesPerChar;                        // micro spaces per character; defined in wheelwriter.c
extern unsigned char uLinesPerLine;                         // micro lines per line; defined in wheelwriter.c
extern unsigned int  uSpaceCount;                           // number of micro spaces on the current line; defined in wheelwriter.c

volatile unsigned char timeout = 0;                         // decremented every 50 milliseconds, used for detecting timeouts
volatile unsigned char hours = 0;                           // uptime hours
volatile unsigned char minutes = 0;                         // uptime minutes
volatile unsigned char seconds = 0;                         // uptime seconds

// uninitialized variables in xdata RAM, contents unaffected by reset
unsigned char xdata wdResets         _at_ 0xEF0;            // count of watchdog resets
unsigned char xdata softResetFlag    _at_ 0xEF1;            // flag set on software reset

code char about[] = "Wheelwriter Teletype Version 1.3.5\n"
                    "for STCmicro IAP15W4K61S4 MCU and Keil C51 Compiler\n"
                    "Compiled on " __DATE__ " at " __TIME__"\n"
                    "Copyright 2019-2025 Jim Loos\n";

code char help1[] = "\n\nControl characters:\n"
                    "  BEL 0x07        spins the printwheel\n"
                    "  BS  0x08        non-destructive backspace\n"
                    "  TAB 0x09        horizontal tab\n"
                    "  LF  0x0A        paper up one line\n"
                    "  VT  0x0B        paper up one line\n"
                    "  CR  0x0D        returns carriage to left margin\n"
                    "  ESC 0x1B        see Diablo 630 commands below...\n"
                    "\nDiablo 630 commands emulated:\n"
                    "  <ESC><O>        selects bold printing\n"
                    "  <ESC><&>        cancels bold printing\n"
                    "  <ESC><E>        selects continuous underlining\n"
                    "  <ESC><R>        cancels underlining\n"
                    "  <ESC><X>        cancels both bold and underlining\n"
                    "  <ESC><U>        half line feed\n"
                    "  <ESC><D>        reverse half line feed\n"
                    "  <ESC><BS>       backspace 1/120 inch\n"
                    "  <ESC><LF>       reverse line feed\n"
                    "<Space> for more, <ESC> to exit...";
code char help2[] = "\n\nPrinter control not part of the Diablo 630 emulation:\n"
                    "  <ESC><u>        selects micro paper up\n"
                    "  <ESC><d>        selects micro paper down\n"
                    "  <ESC><b>        selects broken underlining\n"
                    "  <ESC><l><n>     auto linefeed on or off\n"
                    "  <ESC><c><n>     auto carriage return on or off\n"
                    "  <ESC><p>        selects Pica pitch\n"
                    "  <ESC><e>        selects Elite pitch\n"
                    "  <ESC><m>        selects Micro Elite pitch\n"
                    "\nDiagnostics/debugging:\n"
                    "  <ESC><^Z><a>    show version information\n"
                    "  <ESC><^Z><l><n> turn flashing red error LED on or off\n"
                    "  <ESC><^Z><m>    monitor Function Board commands\n"
                    "  <ESC><^Z><p><n> show value of Port n (0-5)\n"
                    "  <ESC><^Z><r>    reset the Wheelwriter\n"
                    "  <ESC><^Z><u>    show uptime\n"
                    "  <ESC><^Z><v>    show variables\n"
                    "  <ESC><^Z><w>    show number of watchdog resets\n"
                    "\nCode+Erase on Wheelwriter toggles line/local mode\n\n";

//---------------------------------------------------------------------------------
// Note that the two function below, _getkey() and putchar(), replace the library
// functions of the same name.  These functions use the interrupt-driven serial
// I/O routines in uart1.c
//---------------------------------------------------------------------------------
// for scanf
char _getkey(void) {
    return getchar1();                                      // return character from uart1
}

// for printf
char putchar(char c)  {
   return putchar1(c);                                      // send character to uart1
}


//------------------------------------------------------------
// Timer 0 ISR: interrupt every 50 milliseconds, 20 times per second
//------------------------------------------------------------
void timer0_isr(void) interrupt 1 using 1{
    static unsigned char ticks = 0;

    if (timeout) {                                          // countdown value for detecting timeouts
        --timeout;
    }

    if (initializing) {                                     // flash all three LEDs at 2Hz while initializing
       amberLED = greenLED = redLED = (ticks < 10);
    }

    if (errorLED) {                                         // flash red LED at 2Hz if error
        redLED = (ticks < 10);
    }

    if(++ticks == 20) {                                     // if 20 ticks (one second) have elapsed...
        ticks = 0;

        if (++seconds == 60) {                              // if 60 seconds (one minute) has elapsed...
            seconds = 0;
            if (++minutes == 60) {                          // if 60 minutes (one hour) has elapsed...
                minutes = 0;
                ++hours;
            }
        }
    }
}

//------------------------------------------------------------------------------------------
// The Wheelwriter prints the character and updates the variable 'column'.
// Carriage return cancels bold and underlining and resets 'column' back to 1.
// Linefeeds automatically printed with carriage return if flag 'autoLineFeed' is TRUE.
// The character printed by the Wheelwriter is echoed to the serial port (for monitoring).
//
// Control characters:
//   BEL 0x07    spins the printwheel
//   BS  0x08    non-destructive backspace
//   TAB 0x09    horizontal tab to next tab stop
//   LF  0x0A    moves paper up one line
//   VT  0x0B    moves paper up one line
//   CR  0x0D    returns carriage to left margin. if autoLineFeed is on, paper moves up one line (linefeed)
//   ESC 0x1B    see Diablo 630 commands below...
//
// Diablo 630 commands emulated:
//   <ESC><O>    selects bold printing
//   <ESC><&>    cancels bold printing
//   <ESC><E>    selects continuous underlining  (spaces between words are underlined) for one line
//   <ESC><R>    cancels underlining
//   <ESC><X>    cancels both bold and underlining
//   <ESC><U>    half line feed (paper up 1/2 line)
//   <ESC><D>    reverse half line feed (paper down 1/2 line)
//   <ESC><BS>   backspace 1/120 inch
//   <ESC><LF>   reverse line feed (paper down one line)
//
// printer control not part of the Diablo 630 emulation:
//   <ESC><u>    selects micro paper up (1/8 line or 1/48")
//   <ESC><d>    selects micro paper down (1/8 line or 1/48")
//   <ESC><b>    selects broken underlining (spaces between words are not underlined)
//   <ESC><l><n> auto linefeed (n=1 is on, n=0 is off)
//   <ESC><c><n> auto carriage return (n=1 is on, n=0 is off)
//   <ESC><p>    selects Pica pitch (10 characters/inch or 12 point)
//   <ESC><e>    selects Elite pitch (12 characters/inch or 10 point)
//   <ESC><m>    selects Micro Elite pitch (15 characters/inch or 8 point)
//-------------------------------------------------------------------------------------------
void print_char_on_WW(unsigned char charToPrint) {
    static unsigned char escape = 0;                        // escape sequence state
    unsigned char i,t;

    switch(escape) {
        case 0:                                             // first character
            switch(charToPrint) {
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
                        //putchar(BS);
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
                    if (autoCarriageReturn)                // if TRUE, automatically print carriage return
                        ww_carriage_return();
                    ww_linefeed();
                    putchar(LF);
                    break;
                case VT:
                    ww_linefeed();
                    putchar(LF);
                    break;
                case CR:
                    ww_carriage_return();                   // return the carrier to the left margin
                    column = 1;                             // back to the left margin
                    attribute = 0;                          // cancel bold and underlining
                    if (autoLineFeed)                       // if TRUE, automatically print linefeed
                        ww_linefeed();
                    putchar(CR);
                    break;
                case ESC:
                    escape = 1;                             // ESCAPE character detected, next state
                    break;
                default:
                    if ((charToPrint>0x1F)&&(charToPrint<0x80)) { // 'printable' characters 0x20-0x7F
                        ww_print_character(charToPrint,attribute);
                        putchar(charToPrint);               // echo the character to the console
                        ++column;                           // update column
                    }
            } // switch(charToPrint)
            break;  // case 0:
        case 1:
            escape = 0;                                     // <ESC> has been detected, this is the second character of the escape sequence...
            switch(charToPrint) {
                case 'O':                                   // <ESC><O> selects bold printing
                    attribute |= 0x01;
                    break;
                case '&':                                   // <ESC><&> cancels bold printing
                    attribute &= 0x06;
                    break;
                case 'E':                                   // <ESC><E> selects continuous underline (spaces between words are underlined)
                    attribute |= 0x02;
                    break;
                case 'R':                                   // <ESC><R> cancels underlining
                    attribute &= 0x01;
                    break;
                case 'X':                                   // <ESC><X> cancels both bold and underlining
                    attribute = 0;
                    break;
                case 'U':                                   // <ESC><U> selects half line feed (paper up one half line)
                    ww_paper_up();
                    break;
                case 'D':                                   // <ESC><D> selects reverse half line feed (paper down one half line)
                    ww_paper_down();
                    break;
                case LF:                                    // <ESC><LF> selects reverse line feed (paper down one line)
                    ww_reverse_linefeed();
                    break;
                case BS:                                    // <ESC><BS> backspace 1/120 inch
                    ww_micro_backspace();
                    break;
                case 'b':                                   // <ESC><b> selects broken underline (spaces between words are not underlined)
                    attribute |= 0x04;
                    break;
                case 'c':
                    escape = 3;                             // <ESC><c> selects auto carriage return, the next character turns it on or off
                    break;
                case 'e':                                   // <ESC><e> selects Elite (12 characters/inch)
                    uSpacesPerChar = 10;                    // 10 micro spaces/character
                    uLinesPerLine = 16;                     // 16 micro lines/full line
                    tabStop = 6;                            // tab stops every 6 characters (every 1/2 inch)
                    break;
                case 'l':                                   // <ESC><l> selects auto linefeed, the next character turns it on or off
                    escape = 2;
                    break;
                case 'p':                                   // <ESC><p> selects Pica (10 characters/inch)
                    uSpacesPerChar = 12;                    // 10 micro spaces/character
                    uLinesPerLine = 16;                     // 16 micro lines/full line
                    tabStop = 5;                            // tab stops every 5 characters (every 1/2 inch)
                    break;
                case 'm':                                   // <ESC><m> selects Micro Elite (15 characters/inch)
                    uSpacesPerChar = 8;                     // 10 micro spaces/character
                    uLinesPerLine = 12;                     // 16 micro lines/full line
                    tabStop = 7;                            // tab stops every 7 characters (every 1/2 inch)
                    break;
                case 'u':                                   // <ESC><u> paper micro up (paper up 1/8 line)
                    ww_micro_up();
                    break;
                case 'd':                                   // <ESC><d> paper micro down (paper down 1/8 line)
                    ww_micro_down();
                    break;
            } // switch(charToPrint)
            break;  // case 1:
        case 2:                                             // <ESC><l><n> has been detected. this is the third character of the escape sequence
            escape = 0;
            if (charToPrint & 0x01)
                autoLineFeed = TRUE;                        // <ESC><l><n> odd values of n turn autoLineFeed on, even values turn autoLineFeed off
            else
                autoLineFeed = FALSE;
            break; // case 2
        case 3:                                             // <ESC><c><n> has been detected. this is the third character of the escape sequence
            escape = 0;
            if (charToPrint & 0x01)
                autoCarriageReturn = TRUE;                  // <ESC><c><n> odd values of n turn autoCarriageReturn on, even values turn autoCarriageReturn off
            else
                autoCarriageReturn = FALSE;
            break; // case 3
    } // switch(escape)
}

#define PATTERN " %c%c%c%c%c%c%c%c\n"
#define TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

//------------------------------------------------------------------------------------------
// Process keystrokes from the UART1 monitor/debug serial connection
//
// for diagnostics/debugging:
//   <ESC><h>        display help
//   <ESC><^Z><a>    show version information
//   <ESC><^Z><l><n> turn flashing red error LED on or off (n=1 is on, n=0 is off)
//   <ESC><^Z><m>    monitor Function Board commands
//   <ESC><^Z><p><n> show the value of Port n (0-5) as 2 digit hex number
//   <ESC><^Z><r>    reset both the MCU and the wheelwriter
//   <ESC><^Z><u>    show uptime as HH:MM:SS
//   <ESC><^Z><v>    show variables
//   <ESC><^Z><w>    show number of watchdog resets
//-------------------------------------------------------------------------------------------
void process_key(unsigned char key) {
    static unsigned char escape = 0;                        // escape sequence state
    unsigned char c;

    switch(escape) {
        case 0:                                             // first character
            switch(key) {
                case ESC:
                    escape = 1;                             // ESCAPE character detected, next state
                    break;
            } // switch(key)
            break;  // case 0:
        case 1:
            escape = 0;                                     // <ESC> has been detected, this is the second character of the escape sequence...
            switch(key) {
                case '\x1A':                                // <ESC><^Z> for remote diagnostics
                    escape = 2;
                    break;
                case 'H':
                case 'h':
                    printf(help1);                          // print the first half of the help
                    escape = 5;                             // wait for a key to be pressed...
                    break;
            } // switch(key)
            break;  // case 1:
        case 2:                                             // <ESC><^Z> detected, this is the third character of the escape sequence...
            escape = 0;
            switch(key) {
               case 'A':
               case 'a':                                    // <ESC><^Z><a> print version info
                    printf("\n%s\n",about);
                    for(c=1; c<column; c++) putchar(SP);    // return cursor to previous position on line
                    break;
               case 'L':
               case 'l':                                    // <ESC><^Z><l> controls the red error LED. the next character turn is on or off
                    escape = 4;
                    break;
               case 'M':
               case 'm':                                    // <ESC><^Z><m> monitor communications
                    monitor = !monitor;                     // toggle monitor flag
                    break;
               case 'P':
               case 'p':                                    // <ESC><^Z><p> print port values. the next character selects the port number
                    escape = 3;
                    break;
               case 'R':
               case 'r':                                    // <ESC><^Z><r> reset the MCU and the Wheelwriter
                    ww_reset(3);                            // reset both Wheelwriter boards
                    softResetFlag = 0x55;                   // set the flag
                    IAP_CONTR = 0x20;                       // reset the MCU
                    break;
               case 'U':
               case 'u':                                    // <ESC><^Z><u> print uptime
                    printf("\n%s %02u%c%02u%c%02u\n","Uptime:",(int)hours,':',(int)minutes,':',(int)seconds);
                    for(c=1; c<column; c++) putchar(SP);    // return cursor to previous position on line
                    break;
               case 'V':
               case 'v':                                    // <ESC><^Z><u> print variables
                    printf("\n");
                    printf("%s %s\n",    "autoLineFeed:      ",autoLineFeed?"true":"false");
                    printf("%s %s\n",    "autoCarriageReturn:",autoCarriageReturn?"true":"false");
                    printf("%s %s\n",    "initializing:      ",initializing?"true":"false");
                    printf("%s %s\n",    "monitor:           ",monitor?"true":"false");
                    printf("%s %s\n",    "localMode:         ",localMode?"true":"false");
                    printf("%s" PATTERN, "attribute:         ",TO_BINARY(attribute));
                    printf("%s %d\n",    "column:            ",(int)column);
                    printf("%s %d\n",    "tabStop:           ",(int)tabStop);
                    printf("%s 0x%02X\n","printWheel:        ",(int)printWheel);
                    printf("%s %d\n",    "uSpacesPerChar:    ",(int)uSpacesPerChar);
                    printf("%s %d\n",    "uLinesPerLine:     ",(int)uLinesPerLine);
                    printf("%s %d\n",    "uSpaceCount:       ",(int)uSpaceCount);
                    for(c=1; c<column; c++) putchar(SP);    // return cursor to previous position on line
                    break;
               case 'W':
               case 'w':                                    // <ESC><^Z><w> print watchdog resets
                    printf("\n%s %d\n","Watch Dog Timer resets:",(int)wdResets);
                    for(c=1; c<column; c++) putchar(SP);    // return cursor to previous position on line
                    break;
            } // switch(key)
            break;  // case 2:
        case 3:                                             // <ESC><^Z><p> has been detected. this is the fourth character of the escape sequence
            escape = 0;
            switch(key){
                case '0':
                    printf("%s 0x%02X\n","P0:",(int)P0);    // <ESC><^Z><p><0> print port 0 value
                    break;
                case '1':
                    printf("%s 0x%02X\n","P1:",(int)P1);    // <ESC><^Z><p><1> print port 1 value
                    break;
                case '2':
                    printf("%s 0x%02X\n","P2:",(int)P2);    // <ESC><^Z><p><2> print port 2 value
                    break;
                case '3':
                    printf("%s 0x%02X\n","P3:",(int)P3);    // <ESC><^Z><p><3> print port 3 value
                    break;
                case '4':
                    printf("%s 0x%02X\n","P4:",(int)P4);    // <ESC><^Z><p><3> print port 4 value
                    break;
                case '5':
                    printf("%s 0x%02X\n","P5:",(int)P5);    // <ESC><^Z><p><3> print port 5 value
                    break;
            } // switch(charToPrint)
            break;  // case 3:
        case 4:                                             // <ESC><^Z><e> has been detected. this is the fourth character of the escape sequence
            escape = 0;
            if (key & 0x01)
                errorLED = TRUE;                            // <ESC><^Z><e><n> odd values of n turn the LED on, even values turn the LED off
            else {
                errorLED = FALSE;
                redLED = OFF;                               // turn off the red LED
            }
            break;  // case 4
        case 5:
            if (key == 0x20) {                              // if it's SPACE...
                printf(help2);                              // print the second half of the help text
                escape = 0;
            }
            else if (key == ESC) {                          // if it's ESCAPE, exit
                putchar(0x0D);
                escape = 0;
            }
            break; // case 5
    } // switch(escape)
}

//-----------------------------------------------------------
// main(void)
//-----------------------------------------------------------
void main(void){
    unsigned int loopcounter,function_board_cmd,printer_board_reply;
    unsigned char state = 0;
    unsigned char wwKey,ch;
    unsigned char lastsec = 0;

    // from the data sheet:
    // "After power-up, all PWM-related I/O ports on the IAP15W4K61S4 are in high impedance state.
    // These ports need to be set to quasi-bidirectional or strong push-pull mode for normal use."
    // Affected ports: P0.6,P0.7,P1.6,P1.7,P2.1,P2.2,P2.3,P2.7,P3.7,P4.2,P4.4,P4.5
    P0M1 = 0;                                                   // set P0 to quasi-bidirectional
    P0M0 = 0;

    TL0 = RELOADLO;                                         // load timer 0 low byte
    TH0 = RELOADHI;                                         // load timer 0 high byte
    TMOD = 0x00;                                            // configure timer 0 for mode 0: 16-bit auto-reload timer
    ET0 = 1;                                                // enable timer 0 interrupt
    TR0 = 1;                                                // run timer 0
    uart1_init(115200);                                     // initialize UART1 for N-8-1 at 115200bps for debug/monitor
    uart2_init(9600);                                       // initialize UART2 for N-8-1 at 9600bps, RTS-CTS handshaking for host PC
    uart3_init();                                           // initialize UART3 for N-9-1 at 187500bps for connection to the Function Board
    uart4_init();                                           // initialize UART4 for N-9-1 at 187500bps for connection to the Printer Board

    EA = TRUE;                                              // global interrupt enable

    printf("\n%s\n",about);
    if (POF) {
        printf ("Power-on reset\n");
        wdResets = 0;
        softResetFlag = 0;
        CLR_POF;
    }

    else if (WDT_FLAG){
         printf("%s %d\n","Watch Dog Timer resets:",(int)++wdResets);
         CLR_WDT_FLAG;
    }

    else if (softResetFlag == 0x55) {
        printf ("Software reset\n");
        softResetFlag = 0;
    }

    printf("Initializing");
    lastsec = seconds;
    ww_reset(3);                                            // reset both boards                                                    // initialize ww vars and reset both boards
    WDT_CONTR |= 0x06;                                      // watch dog timer overflows in 4194.3 mS
    ENABLE_WDT;                                             // run watch dog timer

    //////////// determine the pitch of the printwheel /////////////
    printWheel = 0;
    timeout = ONESEC*6;
    while(!printWheel && timeout) {                         // loop for up to 6 seconds or until the printer board replies to 0x121,0x001...
        if (lastsec != seconds) {                           // once each second...
            lastsec = seconds;
            putchar('.');                                   // dots on screen to monitor progress
            RESET_WDT;                                      // reset watch dog timer every second
        }

        if (function_board_cmd_avail()){                    // if there's a command from the function board...
            function_board_cmd = get_function_board_cmd();  // retrieve the command from UART3
            send_to_printer_board(function_board_cmd);      // relay the command to the printer board
            switch(state) {
                case 0:
                    if (function_board_cmd == 0x121) state = 1;
                    break;
                case 1:
                    if (function_board_cmd == 0x001) state = 2;
                    else state = 0;                         // 0x121,0x001 is the "reset" command  from the function board
            }
        }

        if (printer_board_reply_avail()) {                  // if there's a reply from the Printer Board...
            printer_board_reply = get_printer_board_reply();// retrieve the reply from UART4
            send_to_function_board(printer_board_reply);    // relay replies from the Printer Board to the Function Board
            if (state == 2) {                               // if the reset command has been sent, the reply from the printer board is the printwheel pitch
               printWheel = printer_board_reply;            // we now know the pitch of the printwheel, exit the loop
               switch(printWheel) {
                  case 0x008:
                     printf("\nPS printwheel\n");
                     tabStop = 5;                           // tab stops every 5 characters (every 1/2 inch)
                     uSpacesPerChar = 10;                   // 10 micro spaces/character
                     uLinesPerLine = 16;                    // 16 micro lines/full line
                     break;
                  case 0x010:
                     printf("\n15P printwheel\n");
                     tabStop = 7;                           // tab stops every 7 characters (every 1/2 inch)
                     uSpacesPerChar = 8;
                     uLinesPerLine = 12;
                     break;
                  case 0x020:
                     printf("\n12P printwheel\n");
                     tabStop = 6;                           // tab stops every 6 characters (every 1/2 inch)
                     uSpacesPerChar = 10;                   // 10 micro spaces/character
                     uLinesPerLine = 16;                    // 16 micro lines/full line
                     break;
                  case 0x021:
                     printf("\nNo printwheel\n");
                     tabStop = 6;                           // tab stops every 6 characters (every 1/2 inch)
                     uSpacesPerChar = 10;                   // 10 micro spaces/character
                     uLinesPerLine = 16;                    // 16 micro lines/full line
                     break;
                  case 0x040:
                     printf("\n10P printwheel\n");
                     tabStop = 5;                           // tab stops every 5 characters (every 1/2 inch)
                     uSpacesPerChar = 12;                   // 10 micro spaces/character
                     uLinesPerLine = 16;                    // 16 micro lines/full line
                     break;
                  default:
                     printf("\nUnable to determine printwheel. Defaulting to 12P.\n");
                     printf("0x%02X\n",(int)printWheel);
                     tabStop = 6;                           // tab stops every 6 characters (every 1/2 inch)
                     uSpacesPerChar = 10;                   // 10 micro spaces/character
                     uLinesPerLine = 16;                    // 16 micro lines/full line
                     errorLED=TRUE;
               } // switch(printWheel)
            } // if (state == 2)
        } // if (printer_board_reply_avail())
    } // while(!printWheel)

    if (!timeout)                                           // turn on error light if unable to determine printwheel
       errorLED = TRUE;

    //////////// relay the remaining initialization commands from Function to Printer board /////////////
    timeout = ONESEC;                                       // empirically determined 1 second is adequate
    while(timeout) {                                        // loop for one second
        RESET_WDT;                                          // reset watch dog timer
        if (function_board_cmd_avail()){                    // if there's a command from the function board...
            function_board_cmd = get_function_board_cmd();
            //printf("%03X\n",function_board_cmd);
            send_to_printer_board(function_board_cmd);      // relay the command to the printer board
        }
        if (printer_board_reply_avail()) {                  // if there's a reply from the Printer Board...
            printer_board_reply = get_printer_board_reply();
            //printf("%03X\n",printer_board_reply);
            send_to_function_board(printer_board_reply);    // relay replies from the Printer Board to the Function Board
        }
    }

    printf("<ESC> H for help\n");
    printf("Ready\n");
    initializing = FALSE;
    amberLED = OFF;                                         // turn off the amber LED
    greenLED = OFF;                                         // turn off the green LED
    redLED = OFF;                                           // turn off the red LED
    loopcounter = 0;

    //----------------- loop here forever -----------------------------------------
    while(TRUE) {

        RESET_WDT;                                          // reset the watch dog timer each pass thru the loop

        if (++loopcounter==0) {                             // every 65536 passes through the loop (at about 2Hz)
            greenLED = !greenLED;                           // toggle the green "heart beat" LED
        }

        //////////// check for key press codes coming from the Function Board ////////////
        if (function_board_cmd_avail()) {                   // if there's a command from the Function Board...
            function_board_cmd = get_function_board_cmd();  // retrieve it from UART3
            send_ACK_to_function_board();                   // mimic Printer Board by sending Acknowledge to Function Board
            if (monitor) printf("%03X\n",function_board_cmd);// if the monitor flag is set...

            wwKey = ww_decode_keys(function_board_cmd);     // convert the function board keystroke cmd into ASCII character
            if (wwKey) {                                    // if it's a valid ASCII key...
                if (wwKey == 0xF0) {                        // is it Code+Erase key combo?
                    if (column == 1) {                      // carrier must be at left margin to change modes
                        localMode = !localMode;             // toggle the line/local flag
                        ww_spin();                          // spin the printwheel
                        ww_paper_up();                      // up 1/2 line, then
                        ww_paper_down();                    // down 1/2 line as a visual indication
                    }
                }
                else {
                    if (localMode)
                       print_char_on_WW(wwKey);             // if 'local' mode, print the ASCII character on the Wheelwriter
                   else
                      putchar2(wwKey);                      // else print the ASCII character on the console
                }
            }
        }

        //////////// check for characters to print coming from the serial console (UART2)     ////////////
        if (char_avail2()) {                                // if there is a character in the serial receive buffer...
            ch = getchar2();                                // retrieve the character from UART2
            print_char_on_WW(ch);                           // send it to the Wheelwriter for printing
        }

        //////////// check for characters to coming from the debug serial connection (UART1) ////////////
        if (char_avail1()) {
           ch = getchar1();
           process_key(ch);
        }
    } //----------------- end of loop -----------------------------------------
}