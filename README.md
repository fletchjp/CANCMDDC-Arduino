<img align="right" src="arduino_cbus_logo.png"  width="150" height="75">

# CANCMDDC-Arduino

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

John Fletcher <M6777> 25/02/2024

