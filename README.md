# IBM Wheelwriter "Teletype"
This project uses an Intel 8052 compatible STC Micro [STC15W4K32S4](https://www.stcmicro.com/datasheet/STC15W4K32S4-en.pdf) series microcontroller (specifically an IAP15W4K61S4) to turn an IBM Wheelwriter Electronic Typewriter into a teletype-like device. This project only works on earlier Wheelwriter models, the ones that internally have two circuit boards: the Function Board and the Printer Board (Wheelwriter models 3, 5 and 6).

The MCU intercepts commands generated by the Function Board when keys are pressed, and converts these commands into ASCII characters that correspond to the keys pressed. In the default "typewriter" or "local" mode, the MCU sends the characters to the Printer Board for printing and so the Wheelwriter acts as a normal typewriter. In the "keyboard" or "line" mode, the MCU sends characters through the console serial port to the host computer. In “keyboard” mode the characters from the Function Board are not sent to the Printer Board, so no characters are printed when keys are pressed. At any time, ASCII characters received from the host computer by the MCU through the console serial port are converted into commands and sent to the Printer Board. Thus, the Wheelwriter acts as a serial printer.

<p align="center"><img src="/images/Wheelwriter Interface.JPEG" width=75% height=75%>
<br><br><br><br>
<p align="center"><img src="/images/Schematic-1.png" width=75% height=75%>
<br><br><br><br>
<p align="center"><img src="/images/IAP15W4K61S4 Connection.png" width=75% height=75%>
<br><p align="left">
The ribbon cable that normally connects the Wheelwriter's Function and Printer boards is disconnected and the STC15W4K32S4 MCU is interposed instead between the two boards. (see diagram above) The MCU uses one of its four UARTs to communicate with the Wheelwriter's Function Board and a second UART to communicate with the Printer Board. See the schematic for details.
<br><br><br><br>
<p align="center"><img src="/images/STC-ISP.png" width=75% height=75%>
 
NOTE: When using [STCmicro's STC-ISP](https://www.stcmicro.com/rjxz.html) application to download object code to the MCU, be sure to specify 12 MHz internal oscillatior frequency. 

If using Grigori Goronzy's [STCGAL](https://github.com/grigorig/stcgal) to download object code, include '-t 12000' on the command line when invoking the application to trim the internal oscillator to 12 MHz. 
