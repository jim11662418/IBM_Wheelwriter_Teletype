# wheelwriter-teletype
This project uses an STCmicro STC15W4K32S4 (an Intel 8052-compatible microcontroller) to turn an IBM Wheelwriter Electronic Typewriter into a teletype-like device. This project only works on earlier Wheelwriter models, the ones that internally have two circuit boards: the Function Board and the Printer Board (Wheelwriter models 3, 5 and 6).

The cable that normally connects the Wheelwriter's Function and Printer boards is disconnected and the MCU is connected instead in series between the two boards. The MCU uses one of its four UARTs to communicate with the Function Board and a second UART to communicate with the Printer Board. See the schematic for details.

In “typewriter" mode, the MCU transparently relays communications back and forth between the Function Board and the Printer Board and so the Wheelwriter acts as a normal typewriter. In “keyboard” mode, the MCU intercepts commands generated by the Function Board when keys are pressed, converts these commands into equivalent ASCII characters, and outputs the characters through the console serial port. In “keyboard” mode the commands from the Function Board are not sent to the Printer Board, so characters are not printed when keys are pressed. In “printer” mode, ASCII characters sent to the MCU through the console serial port are converted into commands and sent to the Printer Board. Thus, in “printer” mode, the Wheelwriter acts as a serial printer or "teletype".
