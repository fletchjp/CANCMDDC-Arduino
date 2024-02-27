<img align="right" src="arduino_cbus_logo.png"  width="150" height="75">

# CANCMDDC-Arduino Version 1.1.16

This project is for an Arduino to control a number of analogue (DC) train controllers 
in response to Merg CBus messages originally intended for use between the CANCAB hand 
throttle and the CANCMD DCC Command Station. The Arduino effectively 'spoofs' a number
of DCC addresses that are then used to control the analogue throttles. It can be used
either 'standalone', or alongside a CANCMD which can then be used to also control DCC
locomotives on a separate track.

Development of this project is documented on the Merg Forum here:
     http://www.merg.org.uk/forum/viewtopic.php?f=45&t=6376

This sketch makes use of the MCP_CAN library, modified by Mark Riddoch, to accept
either an 8MHz clock or a 16MHz clock. It can be found here:
     https://github.com/M1118/CANCMDDC
	 
## Modifications by John Fletcher February 2024

I am modifying these codes to have a version usable by Paul Townsend.

The changes so far are mainly to arrange so that the various files used in the past
by Dave Radcliffe can still be used.

One problem is that he used a library called Liquid Crystal 
https://github.com/jenschr/Arduino-libraries/tree/master/LiquidCrystal

There is a more recent library with the same name and the solution has been to copy
the files needed into this project.

I have found that there are three different versions of the mcp_can library.
The code is using the published version 1.5.1 and not older versions.

## Testing and further modification

I have modified the code and it now works with the two Encoders, keypad and the LCD display.

I had to introduce extra logic to sort out the reversal of direction using the encoder buttons.

The LCD display worked when I corrected the address used for I2C.

## Configuration choices

There are set of defines in the code at about line 142 as follows

DEBUG         1 // set to 0 for no debug messages, 1 for messages to console
OLED_DISPLAY  0 // set to 0 if 128x32 OLED display is not present
LCD_DISPLAY   1 // set to 0 if 4x20 char LCD display is not present
KEYPAD        1 // set to 0 if 4x3 keypad is not present
CANBUS        0 // set to 0 if CAN h/w is not present
ENCODER       1 // set to 0 if encoders not present

These are used to chose what to include when the code is compiled.
I have not tested the OLED DISPLAY option.

At present the CANBUS option takes precedence if it is chosen.

## LCD Display
 
This works by I2C on pins 20 and 21 which is standard for a MEGA.

It is important to set the I2C device address correctly at line 449.
The display I have works on 0x27. David had one on 0x3f which I had to change.
If necessary use an I2C scan program to detect the value to use.

## Encoders 

The encoders use the following pins:

 Digital pin 38             Encoder 1 Switch
 Digital pin 40             Encoder 2 Switch
 Digital / Analog pin 8     Encoder 1 B
 Digital / Analog pin 9     Encoder 1 A
 Digital / Analog pin 10    Encoder 2 B
 Digital / Analog pin 11    Encoder 2 A

These are set like this at lines 294 and 295

 {encoderControllerClass(A9, A8, 38)},
 {encoderControllerClass(A11, A10, 40)}

NOTE: This should give a speed increasing as the knob is turned to the right. If this is not the case, swap the pairs (A9, A8) and (A11, A10).

## Keypad

This plugs into a set of odd numbered pins 37 to 49 on the MEGA. See line no 222.
The keypad and MEGA both have female sockets so male to mail wires are needed.

## CANBUS

The CAN connection to an external MCP2515 board uses the SPI connection.

 Digital pin 19 (RX1)		Int'upt	CAN
 Digital pin 50 (MISO)		SO		CAN
 Digital pin 51 (MOSI)		SI		CAN
 Digital pin 52 (SCK)		Sck		CAN
 Digital pin 53 (SS)		CS		CAN

## Driver for the trains

I have not tested this. The code provides for these pins to be used.

 Digital pin 2 (PWM)		PWM0	H1a
 Digital pin 3 (PWM)		PWM1	H1b
 Digital pin 22				EnableA	H1a
 Digital pin 23				EnableB	H1a
 Digital pin 24				EnableA	H1b
 Digital pin 25				EnableB	H1b

## Operation

On startup the display shows MERG in large letters and there is DEBUG text on the serial monitor.

The two controlers are assigned as 1001 and 1002.

On testing without CBUS, the two encoders can be used to increase and decrease the speed and to reverse direction using the push button.

This can be seen on the display and also in DEBUG messages on the serial monitor.

The Keypad can also be used to set locos and speeds.


John Fletcher <M6777> 27/02/2024

