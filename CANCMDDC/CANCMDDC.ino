#define VERSION 1.12
/**
 * CANCMDDC - A DC "command station" for use with MERG CBUS systems
 * Copyright (c) 2015 Mark Riddoch
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 3 of the License
 */

/*
 * This sketch makes use of the MCP_CAN library, modified to accept
 * either an 8MHz clock or a 16MHz clock.
 * Hardware requires are an MCP2515 based CBUS interface and a set
 * of H-Bridges that can be used to drive the DC tracks.
 */

/*
 * Updated and extended by Ian Morgan.
 *
 *  Now uses hardware interrupt to receive incoming CBus messages into a FIFO buffer for later processing.
 *  Also uses a 100Hz timer interrupt to control LED, buzzer and accelleration and deccelleration.
 *  This sketch creates one instance of the FIFO buffer class, and 5 instances for the train controller class.
 */

 /*
 * Updated and extended by David W Radcliffe M3666.
 *
 *  Now creates 6 (and upto 8) instances of the train controller class.
 *  Use an alternate LCD display: this is the Geekcreit� IIC I2C 2004 204 20 x 4 Character LCD Display Module.
 *  Uses an (optional) 4 x 3 keypad for local control of the track outputs and their speed.
 *  Up to 8 CANCMDDCs can be added to one CAN bus, each with its own address and DCC Address range.
 *  DCC Addresses x001 - x00n are used, and the last digit is used by the keypad to select the loco.
 *  This allows loco 0 (and 9) to be used for some other (as yet undefined) function - maybe 'select all' ??
 *  Use (optional) encoders to set speed & direction
 */

/*
 Pin Use map:
 Digital pin 2 (PWM)		PWM0	H1a
 Digital pin 3 (PWM)		PWM1	H1b
 Digital pin 4 (PWM)		Enable	Buzzer
 Digital pin 5 (PWM)		PWM2	H2a
 Digital pin 6 (PWM)		PWM3	H2b
 Digital pin 7 (PWM)		PWM4	H3a
 Digital pin 8 (PWM)		PWM5	H3b
 Digital pin 9 (PWM)		Encoder 7 S unused
 Digital pin 10 (PWM)		Mode	unused - pull low for CANCMD master
 Digital pin 11 (PWM)		PWM6	unused - H-bridge 4a
 Digital pin 12 (PWM)		PWM7	unused - H-bridge 4b
 Digital pin 13 (PWM)				LED (onboard)
 Digital pin 14 (TX3)		EnableA	unused - H-bridge 4a
 Digital pin 15 (RX3)		EnableB	unused - H-bridge 4a
 Digital pin 16 (TX2)		EnableA	unused - H-bridge 4b
 Digital pin 17 (RX2)		EnableB	unused - H-bridge 4b
 Digital pin 18 (TX1)		Shutdown       - pull low when short-circuit detected
 Digital pin 19 (RX1)		Int'upt	CAN
 Digital pin 20 (SDA)		SDA		LCD
 Digital pin 21 (SCL)		SCL		LCD
 Digital pin 22				EnableA	H1a
 Digital pin 23				EnableB	H1a
 Digital pin 24				EnableA	H1b
 Digital pin 25				EnableB	H1b
 Digital pin 26				EnableA	H2a
 Digital pin 27				EnableB	H2a
 Digital pin 28				EnableA	H2b
 Digital pin 29				EnableB	H2b
 Digital pin 30				EnableA	H3a
 Digital pin 31				EnableB	H3a
 Digital pin 32				EnableA	H3b
 Digital pin 33				EnableB	H3b
 Digital pin 34				LED Green  - unused
 Digital pin 35				LED Yellow - unused
 Digital pin 36				Encoder 8 Switch
 Digital pin 37				c0		Keypad - uses odd-numbered pins only so that the 7-way header plugs straight in
 Digital pin 38             Encoder 1 Switch
 Digital pin 39				c1		Keypad
 Digital pin 40             Encoder 2 Switch
 Digital pin 41				c2		Keypad
 Digital pin 42             Encoder 3 Switch
 Digital pin 43				r0		Keypad
 Digital pin 44             Encoder 4 Switch
 Digital pin 45 (PWM)		r1		Keypad
 Digital pin 46             Encoder 5 Switch
 Digital pin 47				r2		Keypad
 Digital pin 48             Encoder 6 Switch
 Digital pin 49				r3		Keypad
 Digital pin 50 (MISO)		SO		CAN
 Digital pin 51 (MOSI)		SI		CAN
 Digital pin 52 (SCK)		Sck		CAN
 Digital pin 53 (SS)		CS		CAN
 Digital / Analog pin 0     Encoder 1 A
 Digital / Analog pin 1     Encoder 2 A
 Digital / Analog pin 2     Encoder 3 A
 Digital / Analog pin 3     Encoder 4 A
 Digital / Analog pin 4     Encoder 5 A
 Digital / Analog pin 5     Encoder 6 A
 Digital / Analog pin 6     Encoder 7 A - unused
 Digital / Analog pin 7     Encoder 8 A - unused
 Digital / Analog pin 8     Encoder 1 B
 Digital / Analog pin 9     Encoder 2 B
 Digital / Analog pin 10    Encoder 3 B
 Digital / Analog pin 11    Encoder 4 B
 Digital / Analog pin 12    Encoder 5 B
 Digital / Analog pin 13    Encoder 6 B
 Digital / Analog pin 14    Encoder 7 B - unused
 Digital / Analog pin 15    Encoder 8 B - unused
*/

#define DEBUG         1 // set to 0 for no debug messages, 1 for messages to console
#define OLED_DISPLAY  0 // set to 0 if 128x32 OLED display is not present
#define LCD_DISPLAY   0 // set to 0 if 4x20 char LCD display is not present
#define KEYPAD        1 // set to 0 if 4x3 keypad is not present
#define CANBUS        1 // set to 0 if CAN h/w is not present
#define ENCODER       1 // set to 0 if encoders not present

//include libraries
#include <PWM.h>     // Library for controlling PWM Frequency
#include "trainController.h"

#if LCD_DISPLAY || OLED_DISPLAY
#include <Wire.h>    // Library for I2C comminications for display
  #if OLED_DISPLAY
  /* libraries for Adafruit display module */
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

  /*
  Adafruit 128x32 size display using I2C to communicate
  3 pins are required to interface (2 I2C (SCL & SDA)and one reset (user definable))
  */
  #define OLED_RESET 22
  #endif

  #if LCD_DISPLAY
  /* libraries for LCD display module */
  #include <LiquidCrystal_I2C.h>
  #include <LiquidCrystal.h>
  #include <LCD.h>
  #include <I2CIO.h>
  #include <FastIO.h>

  /* 	   Geekcreit� IIC I2C 2004 204 20 x 4 Character LCD Display Module using I2C to communicate
		 2 pins are required to interface(I2C(SCL & SDA))

		   see http://arduino-info.wikispaces.com/LCD-Blue-I2C Use Sketch from I2C LCD DISPLAY VERSION 1. Do not Forget to adjust contrast!
			 https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
  */
  #endif
#endif

#if KEYPAD
  #include <Keypad.h>
#endif

#define TRUE    1
#define FALSE   0
#define ON      1
#define OFF     0

#define LED         13     // Pin number for the on-board LED
#define SOUNDER     4      // Pin number for the AWD

/* Address select pins. These are active-low inputs */
/* Link pin to 0V to set address. */
#define Addr0  34
#define Addr1  35
#define Addr2  36

/* pin used for indication of power shutdown. */
/* Link pin to 0V when power off. */
#define SHUTDOWN  18

/*
 * The following are the PWM outputs used to drive the tracks.
 *
 * Pins are fed to H Bridges, along with 2 enable pins.
 *
 * The following pins can handle the high frequency PWM
 */
static int pwmpins[] = {
  2, 3, 5, 6, 7, 8, 11, 12
};

#if KEYPAD
/*
* The following are the inputs/outputs used to drive the keypad.
*/
static byte keypins[] = {
	37, 39, 41, 43, 45, 47, 49 // uses odd-numbered pins only so that the 7-way header plugs straight in
};
#endif

#define NUM_CONTROLLERS  6 // the number of controllers (tuples of pwmpin and 2 enable pins) Max. 8

#define MAXTIMEOUT 30      // Max number of seconds before session is timed out
						   // if no stayalive received for the session

#define TIMER_PRELOAD 64911 //59286 //65536 - ((16000000/256)/100);       // preload timer 65536-(16MHz/256/100Hz)

#define CAN_RTR_MASK 0x40000000

#define CAN_DEFAULT_ID 0x7C
/**
 * The preset tests the jumper in the mode select pin to
 * determine the mode of operation. Connect pin to +5V if CANCMD is present.
 * Leave pin disconnected for standalone operation.
 * It is true if the CANCMDDC is working on a CBUS that has a CANCMD present.
 */

boolean          cancmd_present;
volatile byte    timer_counter  = 0;
volatile byte    flash_counter  = 0;
volatile byte    beep_counter   = 0;
volatile byte    update_counter = 0;
volatile boolean updateNow      = false;
volatile boolean shutdownFlag   = false;
/**
 * Definitions of the flags bits
 */
#define SF_FORWARDS  0x01      // Train is running forwards
#define SF_REVERSE   0x00      // Train is running in reverse
#define SF_LONG      0xC0      // long DCC address. top 2 bits of high byte. both 1 for long, both 0 for short.
#define SF_INACTIVE  -1        // CAB Session is not active
#define SF_UNHANDLED -1        // DCC Address is not associated with this analogue controller (CANCMDDC)
#define SF_LOCAL     -2        // DCC Address is operated only by the Keypad/Encoder, and not part of a CAB Session

#define startAddress 1000     // multiplier for DCC address offset from device address. Device 0 uses 1000, device 1 uses 2000,...
byte deviceAddress = 0;       // assume only unit on CAN bus (for now)

// NOTE: controllers' index (not the DCC address) is used by the keypad handler. Making the last digit of the DCC address = the index aids clarity for user.
struct {
		int				session;      // CAB session id for this loco
		unsigned int	DCCAddress;   // complete address
		byte			longAddress;  // top 2 bits of address = 1 for long
		byte			timeout;
		boolean			shared;       // this loco shared by > 1 CAB (this includes the keypad)
  struct {
		  byte			address;      // DCC short address of consist. 0 = unused.
		  byte			session;      // Session id of consist. 0 = unused.
		  boolean		reverse;
  } consist;
  trainControllerClass trainController;
} controllers[NUM_CONTROLLERS] = {
									{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 1, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(22, 23, pwmpins[0])}
								   ,{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 2, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(24, 25, pwmpins[1])}
								   ,{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 3, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(26, 27, pwmpins[2])}
								   ,{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 4, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(28, 29, pwmpins[3])}
								   ,{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 5, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(30, 31, pwmpins[4])}
								   ,{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 6, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(32, 33, pwmpins[5])}
								 //,{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 7, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(14, 15, pwmpins[6])}
								 //,{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 8, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(16, 17, pwmpins[7])}
};

#if ENCODER
  //#define ENCODER_USE_INTERRUPTS
  #define ENCODER_DO_NOT_USE_INTERRUPTS
  #include "encoderController.h"
  struct {
  	encoderControllerClass encoderController;
  } encoders[NUM_CONTROLLERS] = {
  								{encoderControllerClass(A8,  A0, 38)},
  								{encoderControllerClass(A9,  A1, 40)},
  								{encoderControllerClass(A10, A2, 42)},
  								{encoderControllerClass(A11, A3, 44)},
  								{encoderControllerClass(A12, A4, 46)},
  								{encoderControllerClass(A13, A5, 48)},
  							  //{Encoder(A14, A6, 9), 0, 0, 0},
  							  //{Encoder(A15, A7, 36), 0, 0, 0}
  };
#endif

#if CANBUS
  //include libraries
  #include <SPI.h>     // Library for SPI communications to CAN interface
  #include <mcp_can.h>
  #include "FIFO.h"
  #include "cbusdefs.h"
  /**
  * The following block of #defines configures the pins that are
  * used for various special functions:
  */
  
  /* Pins used by SPI interface to CAN module */
  #define CHIPSELECT  53 // select pin for the CANBUS interface
  #define CBUSINTPIN  19 // "interrupt" pin used by the CANBUS interface to signal that a CANBUS packet is ready to be read
  
  /* pin used for manual selection of use with CANCMD or standalone. */
  /* Link pin to 0V if CANCMD required. */
  #define MODESELECT  10 // jumper used to determine the operating mode, standalone or with CANCMD
  /**
  * The CBUS interface object
  */
  MCP_CAN CAN0(CHIPSELECT);      // MCP CAN library
  
  // CAN parameters
  struct {
  	byte    id;
  	boolean enumerating;
  	/* This could be made more memory-efficient by using one bit per CANID for the 127 possible ids.
  	   Would use only 16 bytes of RAM, but would make the code more complex */
  	byte    ids[110];
  	byte    ptr;
  } canId = {
  	CAN_DEFAULT_ID, false, NULL, 0
  };
  
  // instantiate a FIFO buffer for the CBus messages
  CBusMessageBufferClass fifoMessageBuffer;
  boolean                bufferOverflow;
  boolean                cbusActive = true;
  messageRecordType      nextMessage;

#endif

enum errorStates {
	noError,
	locoStackFull,
	locoTaken,
	noSession,
	consistEmpty,
	locoNotFound,
	CANbusError,
	invalidRequest,
	sessionCancelled
};


#if OLED_DISPLAY
  // construct a display object
  Adafruit_SSD1306 display(OLED_RESET);
#endif

#if (OLED_DISPLAY && SSD1306_LCDHEIGHT != 32)
  #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Display images
#if OLED_DISPLAY
 // 'Merg logo_for_adafruit_display'
const unsigned char mergLogo [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x3c, 0x00, 0xe3, 0xff, 0xcf, 0xfc, 0x0f, 0xfe, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x7c, 0x00, 0xe3, 0xff, 0xcf, 0xfe, 0x1f, 0xff, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x62, 0x00, 0x23, 0x00, 0x0c, 0x02, 0x18, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x61, 0x02, 0x23, 0x00, 0x0c, 0x02, 0x18, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x60, 0x84, 0x23, 0x00, 0x08, 0x02, 0x18, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x60, 0x48, 0x23, 0x00, 0x08, 0x02, 0x18, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x60, 0x70, 0x23, 0x00, 0x08, 0x02, 0x18, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x60, 0x20, 0x23, 0x00, 0x08, 0x02, 0x18, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x60, 0x00, 0x23, 0x00, 0x0c, 0x03, 0x18, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x70, 0x00, 0x23, 0xff, 0xcf, 0xff, 0x1c, 0x1e, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x00, 0x23, 0xff, 0xcf, 0xff, 0x9c, 0x1f, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x00, 0x23, 0xc0, 0x0f, 0x01, 0x9e, 0x19, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0x80, 0x0f, 0x01, 0x9e, 0x19, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0x80, 0x0f, 0x00, 0x9e, 0x19, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0x80, 0x0f, 0x00, 0x9e, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0x80, 0x0f, 0x00, 0x9e, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0x80, 0x0f, 0x00, 0x9e, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0x80, 0x0f, 0x00, 0x9e, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0x80, 0x0f, 0x00, 0x9e, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0xc0, 0x0f, 0x00, 0x9e, 0x01, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0xc0, 0x0f, 0x00, 0x9f, 0x03, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0xff, 0x8f, 0x00, 0x9f, 0xfe, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x78, 0x78, 0x23, 0xff, 0x8f, 0x00, 0x8f, 0xfe, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00
};

 // 'bnhmrslogo'
const unsigned char bnhmrsLogo [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0xf0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x04, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x70, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x1c, 0x03, 0xff, 0x80, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0c, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x38, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x3f, 0x01, 0xbc, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x3f, 0xbb, 0xbc, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x3f, 0xbb, 0xdc, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x60, 0x07, 0xff, 0xf3, 0x3b, 0xfe, 0x3f, 0xc1, 0x9c, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x30, 0x03, 0xff, 0xff, 0xff, 0xdf, 0x3f, 0xc1, 0x9c, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x20, 0x07, 0xfd, 0xe6, 0x20, 0x8f, 0x3f, 0xc1, 0x9c, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x40, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x3f, 0xc1, 0x9c, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x3f, 0xc1, 0x9c, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xbc, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0c, 0x01, 0xff, 0x00, 0x01, 0xff, 0xff, 0xff, 0xc7, 0xfc, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x81, 0x3c, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x83, 0x3c, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x03, 0x38, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x03, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x03, 0x70, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x70, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0xe0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x01, 0xe0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x01, 0xc0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x03, 0xc0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x07, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x1f, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xfe, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfc, 0x00, 0x00, 0x00, 0x00
};
#endif

#if LCD_DISPLAY
// construct a display object
														   // Set the pins on the I2C chip used for LCD connections:
														   //                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C display(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

volatile unsigned long previousTurnon    = 0;
volatile unsigned long alight            = 10000;
volatile boolean       barGraphicsLoaded = false;
volatile boolean       showingSpeeds     = false;
#endif

/****** Function Prototypes *****/

int getSessionIndex (byte session);
int getDCCIndex (unsigned int dcc_address, byte long_address);
void releaseLoco(byte session);
void queryLoco(byte session);
void keepaliveSession(byte session);
void locoRequest(unsigned int address, byte long_address, byte flags);
void locoSession(byte session, unsigned int address, byte long_address, byte direction, byte speed);
void sendPLOC(byte session);
void sendTPower(byte on);
void sendDSPD(byte controllerIndex);
void sendRTR();
void sendError(unsigned int address, byte long_address, byte code);
void sendSessionError(byte session, errorStates code);
void addSessionConsist(byte session, byte consist);
void removeSessionConsist(byte session);
void sendReset(void);
void setSpeedSteps(byte session, byte steps);
void emergencyStopAll(void);
void receiveCBusMessage(void);
void nBeeps (byte numBeeps, int durationMillis);
void setupDisplay(void);
void showSpeeds(void);
void displaySpeed(byte sessionIndex);
void displayImage(const uint8_t *imageBitmap);
void displayVersion(void);
void initialiseDisplay(void);
void customChars(const uint8_t chars[][8]);
byte sid2byte(uint32_t canId);

void(*resetArduino) (void) = 0; //declare reset function at address 0

#if KEYPAD
#pragma region initialise keypad

const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
					 //define the symbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
	{ '1','2','3' },
	{ '4','5','6' },
	{ '7','8','9' },
	{ '*','0','#' }
};
byte rowPins[ROWS] = { keypins[6], keypins[5], keypins[4], keypins[3] }; //connect to the row pinouts of the keypad
byte colPins[COLS] = { keypins[2], keypins[1], keypins[0] };             //connect to the column pinouts of the keypad

																		 //initialize an instance of class NewKeypad
Keypad keyPad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
#pragma endregion

/*
	The keys perform loco/speed selection according to the following FSM:
	Start in idle mode with loco 0 selected.
	Press * - Enter Loco select mode:
			  Press * to stop current loco, then press # to reverse direction or press * to stop all locos. Press 0-n - Enter Speed select mode with that loco selected (if valid).
	Press # - eStop currently selected loco. Press # - eStop all locos.
	Press 0-9 - Enter speed digit. Scroll to left on subsequent digits. Press # - accept entered speed. If invalid (> 127), or Press *, or timeout, cancel current input and enter Speed select mode.
	Timeout after 3 secs to Speed select mode in all cases except eStop all locos.
*/
enum fsmStates {
	idle,
	locoSelect,
	locoStop,
	speedSelect,
	speedDigit,
	locoEmergStop,
	stealOrShare
};

struct {
	fsmStates state;
	byte      currentLoco;
	byte      previousLoco;
	char      digits[3];
} keyFSM = {
	idle, 255, 255, { ' ', ' ', ' ' }
};

volatile unsigned long previousKeypress = 0;
volatile unsigned long interval = 4000;
#endif

/***************************************************************************************************
 * Arduino setup routine
 * Configures the I/O pins, initialises the CANBUS library
 * and sets up the initial session stack
 */
void setup()
{
	#if DEBUG
		// set up the IDE Serial Monitor for debugging output
		Serial.begin(115200);
		Serial.print(F("Initialising"));
		Serial.println();
	#endif

	// set the mode for the Address pins
	pinMode(Addr0, INPUT_PULLUP);
	pinMode(Addr1, INPUT_PULLUP);
	pinMode(Addr2, INPUT_PULLUP);

	// set up the address
	deviceAddress = ((digitalRead(Addr0) + (digitalRead(Addr1) * 2) + (digitalRead(Addr2) * 4)) ^ 0x07);
	if (deviceAddress != 0) // change the default addresses
	{
		for (byte controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
		{
			controllers[controllerIndex].DCCAddress = (startAddress* (deviceAddress + 1)) + controllerIndex + 1;
		}
	}
	#if DEBUG
		Serial.print(F("Address selected: "));
		Serial.println(deviceAddress);
	#endif

	#if CANBUS
		// use a lower ID for non-zero addresses
		canId.id = CAN_DEFAULT_ID - deviceAddress;

		// set the mode for the CBus interrupt pin
		pinMode(CBUSINTPIN, INPUT);

		// set the mode for the operating mode setting input
		pinMode(MODESELECT, INPUT_PULLUP);
		cancmd_present = (digitalRead(MODESELECT) == LOW); // this will be overridden if a message is received from a CANCMD

		// Initialise the CAN interface - do this now so that the h/w can trap any error whilst we display logos
		if (CAN_OK == CAN0.begin(MCP_STDEXT, CAN_125KBPS, MCP_8MHZ)) // init can bus : baudrate = 125k, 8MHz
		{
			#if DEBUG
				Serial.println(F("CAN BUS init ok!"));
			#endif
			if (CAN_OK == CAN0.setMode(MCP_NORMAL))
			{
				#if DEBUG
					Serial.println(F("CAN BUS mode ok!"));
				#endif
				if (cancmd_present == FALSE)
				{
					// Reset any connected CABs
					sendReset();
				}
			}
			else
			{
				#if DEBUG
					Serial.println(F("CAN BUS mode fail"));
				#endif

				#if KEYPAD
					cbusActive = false;
				#else // no CAN or Keypad
					bomb();
				#endif
			}
		}
		else
		{
			//CAN interface initialisation has failed
			#if DEBUG
				  Serial.println(F("CAN BUS init fail"));
			#endif

			#if KEYPAD || ENCODER
				  cbusActive = false;
			#else // no CAN or Keypad or Encoder
				  bomb();
			#endif
		}
	#else // no CAN
		#if !KEYPAD && !ENCODER // no Keypad or Encoders either - can't work!
			bomb();
		#endif
	#endif

	// set the mode for the shutdown status indicator
	pinMode(SHUTDOWN, INPUT_PULLUP);

	// set the mode for the LED status indicator output and sounder
	pinMode(LED, OUTPUT);
	pinMode(SOUNDER, OUTPUT);
	digitalWrite(SOUNDER, HIGH);

	//  //initialize all timers except for 0, to save time keeping functions
	InitTimersSafe();

	// initialize timer5 which will be used to increment session timeout counters.
	// interrupt should fire every 1/10 second

	//  noInterrupts();           // disable all interrupts

	TCCR5A = 0;
	TCCR5B = 0;

	TCNT5 = TIMER_PRELOAD;
	TCCR5B |= (1 << CS12);    // 256 prescaler
	TIMSK5 |= (1 << TOIE5);   // enable timer overflow interrupt

	#if OLED_DISPLAY || LCD_DISPLAY
		initialiseDisplay();

		#if OLED_DISPLAY
			displayImage(mergLogo);
			displayImage(bnhmrsLogo);
		#endif

		#if LCD_DISPLAY
			displayMergLogo();
		#endif

		setupBarGraph();
		displayVersion();

		// start the speed display.
		showSpeeds();
	#else
		// wait for any CANCABs to finish initialising themselves
		delay(2000);
	#endif

	#if KEYPAD
		// wire up keypad events
		keyPad.addEventListener(keypadEvent); // Add an event listener for this keypad
	#endif

	interrupts();             // enable all interrupts

	for (byte controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
	{
	 controllers[controllerIndex].trainController.setPWMFrequency ();
	}

	#if DEBUG
		if (cancmd_present == FALSE)
		Serial.println(F("Standalone mode"));

		Serial.println(F("CANCMDDC started."));
		Serial.println(F("====================================="));
	#endif

	#if CANBUS
		if (cbusActive == false)
		{
			#if !KEYPAD && !ENCODER // no Keypad or Encoders either - can't work!
				bomb();
			#endif
		}
		else
		{
			if (CAN0.errorCountTX() > 0)
			{
				#if DEBUG
					Serial.println(F("CAN Tx error"));
				#endif
				#if KEYPAD || ENCODER
					cbusActive = false;
					nBeeps(2, 100); // two long beeps
				#else // no Keypad or Encoders either - can't work!
					bomb();
				#endif
			}
			else
			{
				attachInterrupt(digitalPinToInterrupt(CBUSINTPIN), receiveCBusMessage, FALLING);
			}
		}
	#endif
}

/******************************************************************************************************
 * Arduino loop routine. Essentially look for CBUS messages
 * and dispatch them to the appropriate handlers
 */
void loop()
{

#if ENCODER
	for (byte index = 0; index < NUM_CONTROLLERS; index++)
	{
		bool push = encoders[index].encoderController.read();

		if (push) // switch pressed
		{
			if (millis() - (encoders[index].encoderController.push) > 500 )
			{
				emergencyStopAll();
				break;
			}
			else
			{
				if (controllers[index].trainController.getSpeed() == 0)
					controllers[index].trainController.setSpeedAndDirection(controllers[index].trainController.getDirection() ^ 1, 0);
				else
					controllers[index].trainController.setSpeed(0);

				displaySpeed(index);
			}
		}

		if (encoders[index].encoderController.newPos != encoders[index].encoderController.lastPos)
		{
#if DEBUG
			Serial.print(index + 1);
			Serial.print(": ");
			Serial.println(encoders[index].encoderController.newPos);
#endif
			if (controllers[index].session == SF_INACTIVE) // not owned by anything
			{
				controllers[index].session = SF_LOCAL;
			}
			controllers[index].trainController.setSpeed(encoders[index].encoderController.newPos);
			encoders[index].encoderController.lastPos = encoders[index].encoderController.newPos;
			displaySpeed(index);
		}
	}
#endif

	long unsigned int dcc_address;
	byte long_address;
	int controllerIndex;

	if (digitalRead(SHUTDOWN) == 0) // power has been shutdown
	{
		if (!shutdownFlag)
		{
#if DEBUG
			Serial.print(F("Shutdown!!"));
#endif
			flash_counter = 20;
			shutdownFlag = true;
			emergencyStopAll();
#if CANBUS
			sendTPower(0);
#endif
			nBeeps(3, 1000);
			delay(500);
		}
	}
	else // power has been restored
	{
		if (shutdownFlag)
		{
#if DEBUG
			Serial.print(F("Power On!!"));
#endif
			shutdownFlag = false;
#if CANBUS
			sendTPower(1);
#endif
			nBeeps(3, 10);
			delay(500);
		}
	}

#if CANBUS
#if LCD_DISPLAY || OLED_DISPLAY
	if (bufferOverflow == true)
	{
#if OLED_DISPLAY
		display.setCursor(64, 24);
#else
		display.setCursor(0, 3);
#endif
		display.print(F("CAN Buffer Overflow!"));

		bufferOverflow = false;
	}
#else
	flash_counter = 25;
#endif

	if (cbusActive == true)
	{
		nextMessage = fifoMessageBuffer.getMessage();
#if DEBUG
		// the CanId bytes are ppppHHHH LLLxxxxx where p = priority bit, x is unused, H & L are high and low bits
		// see: https://www.merg.org.uk/forum/viewtopic.php?f=44&t=3018&p=24032&hilit=re+enumeration#p23986
		Serial.print(F("Fifo CanId: "));
		Serial.println(nextMessage.canId, HEX);
#endif
		if (nextMessage.canId != 0xFFFFFFFF)                             // message rec'd
		{
			if (sid2byte(nextMessage.canId) == canId.id)                // another node is using the same id, so force enum
			{
				flash_counter = 10;
				sendRTR();
			}

			if (nextMessage.len == 0)                                    // check to see whether enum-frame is received.
			{
				if (canId.enumerating)
				{
#if DEBUG
					Serial.print(F("CAN enum: ID "));
					Serial.println(nextMessage.canId, HEX);
#endif
					// add the can id to the list
					canId.ids[canId.ptr] = sid2byte(nextMessage.canId);
					canId.ptr++;
				}
			}
			else                                                         // data is received.
			{
#if DEBUG
				Serial.print(F("CAN msg: ID "));
				Serial.print(nextMessage.canId, HEX);
				Serial.print(F(" OpCode:"));
				for (int i = 0; i < nextMessage.len; i++)                // Print each byte of the data
				{
					if (i == 1)
					{
						Serial.println();
						Serial.print(F(" Data:"));
					}
					Serial.print(F(" "));
					if (nextMessage.rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
					{
						Serial.print(F("0"));
					}
					Serial.print(nextMessage.rxBuf[i], HEX);
				}
				Serial.println();
#endif

				switch (nextMessage.rxBuf[0])                            // Perform the action for the received message
				{

					// -------------------------------------------------------------------
				case 0x07:
#if DEBUG
					Serial.println(F("System Reset (Sent by CANCMD on power up)"));
#endif
					cancmd_present = TRUE; // message came from CANCMD, so must be present
					for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
					{
						// release all active controllers
						if (controllers[controllerIndex].session >= SF_INACTIVE)
						{
							controllers[controllerIndex].trainController.emergencyStop(); // Emergency Stop
							controllers[controllerIndex].session = SF_INACTIVE;
							// update the speed display.
							displaySpeed(controllerIndex);
						}
						// release all consists
						controllers[controllerIndex].consist = { 0, false, false };
					}
					break;

					// -------------------------------------------------------------------
				case 0x08:
#if DEBUG
					Serial.println(F("RTOFF - Request Track Off"));
#endif
					stopAll(true);
					break;

					// -------------------------------------------------------------------
				case 0x09:
#if DEBUG
					Serial.println(F("RTON - Request Track On"));
#endif
					break;

					// -------------------------------------------------------------------
				case 0x21:
#if DEBUG
					Serial.println(F("REL - Release loco"));
#endif
					releaseLoco(nextMessage.rxBuf[1]);
					break;

					// -------------------------------------------------------------------
				case 0x22:
#if DEBUG
					Serial.println(F("QLOC - Query loco"));
#endif
					queryLoco(nextMessage.rxBuf[1]);
					break;

					// -------------------------------------------------------------------
				case 0x23:
#if DEBUG
					Serial.println(F("DKEEP - keep alive"));
#endif
					keepaliveSession(nextMessage.rxBuf[1]);
					break;

					// -------------------------------------------------------------------
				case 0x40:
#if DEBUG
					Serial.println(F("RLOC - Request loco session"));
#endif
					dcc_address = nextMessage.rxBuf[2] + ((nextMessage.rxBuf[1] & 0x3f) << 8);
					long_address = (nextMessage.rxBuf[1] & SF_LONG);
#if DEBUG
					Serial.print(F("Req Sess. "));

					Serial.print(F(" Addr: "));
					Serial.println(dcc_address);
#endif
					if (long_address == 0)
					{
#if DEBUG
						Serial.print(F("Short Consist: "));
						Serial.println(dcc_address);
#endif
						consistRequest(dcc_address);
					}
					else
					{
#if DEBUG
						Serial.print(F("Long Controller: "));
						Serial.println(getDCCIndex(dcc_address, long_address));
#endif
						if (getDCCIndex(dcc_address, long_address) != SF_UNHANDLED)
						{
							locoRequest(dcc_address, long_address, 0);
						}
					}
					break;

					// -------------------------------------------------------------------
				case 0x41:
#if DEBUG
					Serial.println(F("Query Consist......."));
#endif
					break;

					// -------------------------------------------------------------------
				case 0x44:
#if DEBUG
					Serial.println(F("STMOD - Set speed steps"));
#endif
					setSpeedSteps(nextMessage.rxBuf[1], nextMessage.rxBuf[2]);
					break;

					// -------------------------------------------------------------------
				case 0x45:
#if DEBUG
					Serial.println(F("PCON - Put loco in Consist"));
#endif
					addSessionConsist(nextMessage.rxBuf[1], nextMessage.rxBuf[2]);
					break;

					// -------------------------------------------------------------------
				case 0x46:
#if DEBUG
					Serial.println(F("KCON - Remove loco from Consist"));
#endif
					removeSessionConsist(nextMessage.rxBuf[1]);
					break;

					// -------------------------------------------------------------------
				case 0x47:
#if DEBUG
					Serial.println(F("DSPD - Set speed & direction"));
#endif
				{
					byte session = nextMessage.rxBuf[1];
					byte requestedSpeed = nextMessage.rxBuf[2];

					controllerIndex = getSessionIndex(session);
					if (controllerIndex > SF_INACTIVE)
					{
						setSpeedAndDirection(controllerIndex, requestedSpeed, 0);
					}
					else
					{
						// session may be a consist, so apply speed to all controllers in it
						for (int i = 0; i < NUM_CONTROLLERS; i++)
						{
							if (controllers[i].consist.session == session)
								setSpeedAndDirection(i, requestedSpeed, controllers[i].consist.reverse ? 1 : 0);
						}
					}
					// reset the timeout
					keepaliveSession(session);
					break;
				}

					// -------------------------------------------------------------------
				case 0x61:
#if DEBUG
					Serial.println(F("GLOC - Get loco session (Steal/Share)"));
#endif
					dcc_address = nextMessage.rxBuf[2] + ((nextMessage.rxBuf[1] & 0x3f) << 8);
					long_address = (nextMessage.rxBuf[1] & SF_LONG);
					if (long_address == SF_LONG)
					{
						locoRequest(dcc_address, long_address, nextMessage.rxBuf[3]);
					}
					break;

					// -------------------------------------------------------------------
				case 0xE1:
				{
#if DEBUG
					Serial.println(F("PLOC - Allocate session from CANCMD"));
#endif
					dcc_address = nextMessage.rxBuf[3] + ((nextMessage.rxBuf[2] & 0x3f) << 8);
					long_address = (nextMessage.rxBuf[2] & SF_LONG);
#if DEBUG
					Serial.print(F("PLOC from CANCMD.  Addr: "));
					Serial.println(dcc_address);
#endif
					if (long_address == 0)
					{
#if DEBUG
						Serial.print(F("Short"));
#endif
						// address may be of a consist, so apply change to all controllers in it
						for (int i = 0; i < NUM_CONTROLLERS; i++)
						{
							if (controllers[i].consist.address == dcc_address)
							{
								controllers[controllerIndex].consist.session = nextMessage.rxBuf[1];
								setSpeedAndDirection(i, nextMessage.rxBuf[4], controllers[i].consist.reverse ? 1 : 0);
							}
						}
					}
					else
					{
#if DEBUG
						Serial.print(F("Long"));
#endif
						cancmd_present = TRUE; // message came from CANCMD, so must be present
						// only interested in the addresses of our analogue outputs
						for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
						{
							if (controllers[controllerIndex].DCCAddress == dcc_address && controllers[controllerIndex].longAddress == long_address)
							{
								controllers[controllerIndex].session = nextMessage.rxBuf[1];

								setSpeedAndDirection(controllerIndex, nextMessage.rxBuf[4], 0);
							}
						}
					}
					break;
				}
					// -------------------------------------------------------------------
				case 0x0A:
#if DEBUG
					Serial.println(F("RESTP - STOP all"));
#endif
					emergencyStopAll();
					break;

					// -------------------------------------------------------------------
				default:
					// ignore any other CBus messages
					break;
				}
			}
		}
		else // No CBus message received this time round the loop
		{
			if (canId.enumerating)
			{
				// will have had enough time for the responses by now...
				canId.enumerating = false;

				if (canId.ptr > 0) // we have some
				{
					canId.id = 0;

					for (int i = 0; i < canId.ptr; i++) // walk through responses
					{
						if (i < CAN_DEFAULT_ID) // ignore fixed values
						{
							if (canId.ids[i] > canId.id)
								canId.id = canId.ids[i]; // set to highest found
						}
					}
					canId.id++; // use next value
				}
#if DEBUG
				Serial.print(F("Chosen CanId: "));
				Serial.println(canId.id, HEX);
#endif
			}

			// Check the sessions for timeout
			for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
			{
				if ((controllers[controllerIndex].session > SF_INACTIVE) && (controllers[controllerIndex].timeout > MAXTIMEOUT))
				{
#if DEBUG
					Serial.print(F("Session "));
					Serial.print(controllers[controllerIndex].session);
					Serial.print(F(" Address "));
					Serial.print(controllers[controllerIndex].DCCAddress);
					Serial.println(F(" Timed Out."));
#endif
					controllers[controllerIndex].trainController.setSpeedAndDirection(0, 0);
					releaseLoco(controllers[controllerIndex].session);
					sendSessionError(controllers[controllerIndex].session, sessionCancelled); // Send session cancelled message out to CABs
				}

				if (updateNow == true)
				{
					controllers[controllerIndex].trainController.matchToTargets();
					// update the speed display.
					// displaySpeed(controllerIndex);
				}
			}
			updateNow = false;
		}
	}

#endif

#if KEYPAD

	keyPad.getKeys();

	if (previousKeypress > 0)
	{
		if ((unsigned long)(millis() - previousKeypress) >= interval)
			doAfter();
	}
#endif

#if OLED_DISPLAY || LCD_DISPLAY
	if (previousTurnon > 0)
	{
		if ((unsigned long)(millis() - previousTurnon) >= alight)
		{
			display.noBacklight();
			previousTurnon = 0;
		}
	}
#endif
}

byte sid2byte(uint32_t canId)
{
	// canId is 32 bits: eeeeeeee eeeeeeee ppppHHHH LLL00000 where e = extended bit, p = priority, H = high 4 bits, L = low 3 bits
	return (canId & 0x000FE0) >> 5;
}

void bomb()
{
	nBeeps(4, 200); // four long beeps
	delay(200);
	resetArduino();
}

/* *******************************************************************************
 * functions and procedures
 * *******************************************************************************/
int getSessionIndex(byte session)
{
  byte controllerIndex;
  for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
	if (controllers[controllerIndex].session == session)
	{
	  return controllerIndex;
	}
  }
  // session not found, so
  return SF_INACTIVE;
}

int getDCCIndex(unsigned int dcc_address, byte long_address)
{
  byte controllerIndex;
  for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
	if ((controllers[controllerIndex].DCCAddress == dcc_address) && (controllers[controllerIndex].longAddress == long_address))
	{
	  return controllerIndex;
	}
  }
  // controller not found for this DCC address, so
  return SF_UNHANDLED;
}

/*
 * A loco release command for the given session
 */
void releaseLoco(byte session)
{
  int controllerIndex = getSessionIndex(session);
  if (controllerIndex > SF_UNHANDLED)
  {
	controllers[controllerIndex].session = SF_INACTIVE;
	controllers[controllerIndex].timeout = 0;
	controllers[controllerIndex].shared = false;
	#if DEBUG
	  Serial.print(F("Session "));
	  Serial.print(session);
	  Serial.print(F(" Address "));
	  Serial.print(controllers[controllerIndex].DCCAddress);
	  Serial.println(F(" Released."));
	#endif
	// update the speed display.
	displaySpeed(controllerIndex);
  }
  else
  {
	  // session may be a consist
	  for (int i = 0; i < NUM_CONTROLLERS; i++)
	  {
		  if (controllers[i].consist.session == session)
			  controllers[i].consist.session = 0;

		  Serial.print(F("Session "));
		  Serial.print(session);
		  Serial.print(F(" Address "));
		  Serial.print(controllers[i].consist.address);
		  Serial.println(F(" Released."));
	  }
  }
}

#if CANBUS
/*
 * A QLOC command for the given session from a CAB
 */
void queryLoco(byte session)
{
  int controllerIndex = getSessionIndex(session);
  // only respond if working standalone
  if (cancmd_present == FALSE)
  {
#if DEBUG
	Serial.print(F("Query Loco Session "));
	Serial.println(session);
#endif
	if (controllerIndex >= 0)
	{
		// session found, send a PLOC
		if (cbusActive == true)
		{
			sendPLOC(session);
		}
	}
	else
	{
	  // session not active, so send back error
	  sendSessionError(session, noSession);
	}
  }
}
#endif

/*
 * The command station has allocated a session to a locomotive
 */
void locoSession(byte session, unsigned int address, byte long_address, byte direction, byte speed)
{
  byte controllerIndex = getDCCIndex (address, long_address);
   #if DEBUG
	 Serial.print(F("locoSession "));
	 Serial.print(session);
	 if (long_address == 0)
	 {
	   Serial.print(F(" Short"));
	 }
	 else
	 {
	   Serial.print(F(" Long"));
	 }
	 Serial.print(F(" DCC address "));
	 Serial.println(address);
   #endif
  if (controllerIndex >= 0)
  {
	controllers[controllerIndex].session = session;
	controllers[controllerIndex].trainController.setSpeedAndDirection (direction, speed);
	// update the speed display.
	displaySpeed(controllerIndex);
  }
}

/*
 * Keep alive received, so reset the timeout counter
 */
void keepaliveSession(byte session)
{
  int controllerIndex = getSessionIndex(session);
  if (controllerIndex > SF_INACTIVE)
  {
	controllers[controllerIndex].timeout = 0;
  }
}

#if CANBUS
/*
 * A throttle has requested access to a particular loco address
 * This routine is only used if there is no CANCMD on the bus that will
 * allocate sessions.
 */
void locoRequest(unsigned int address, byte long_address, byte flags)
{
#if DEBUG
  Serial.println(F("LocoRequest"));
#endif
  // only respond if working standalone
  if (cancmd_present == FALSE)
  {
#if DEBUG
	Serial.println(F("Standalone"));
#endif
	int controllerIndex = getDCCIndex(address, long_address);
#if DEBUG
	Serial.print(F("Index: "));
	Serial.println(controllerIndex);
#endif
	if (controllerIndex > SF_INACTIVE)
	{
	  // This is a DCC Address associated with our controllers
	  if (controllers[controllerIndex].session > SF_INACTIVE)
	  {
		// Loco is already used in a session
#if DEBUG
		Serial.print(F("Loco already allocated to session: "));
		Serial.println(controllers[controllerIndex].session);

		Serial.print(F("Flag: "));
		Serial.println(flags);
#endif
		if (flags == 0)
		  sendError(address, long_address, locoTaken);
		else if (flags == 1)        // Steal
		{
		  sendError(address, long_address, sessionCancelled);

		  controllers[controllerIndex].session = SF_INACTIVE;
#if KEYPAD
		  controllers[keyFSM.currentLoco].shared = false;
#endif
		}
		else if (flags == 2)        // Share
		{
		  sendPLOC(controllers[controllerIndex].session);
#if KEYPAD
		  controllers[keyFSM.currentLoco].shared = true;
#endif
		}
		else
		  sendError(address, long_address, invalidRequest);
		return;
	  }
	}
	else
	{
	  // This DCC Address is not associated with any of our controllers
	  sendError(address, long_address, invalidRequest);
	  return;
	}
	// If we have got this far then the controller is not in use.
	// Set a new session number for the controller to use.
	locoSession((deviceAddress << 4) + controllerIndex + 1, // Make the session id non-zero and unique across all CANCMDDC instances by using deviceAddress.
															// As deviceAddress is 0-7, the 5 MSB's will always be zero and can be disgarded.
				address,
				long_address,
				SF_FORWARDS,
				0);
 #if DEBUG
		Serial.print(F("Session Allocated: "));
		Serial.println(controllers[controllerIndex].session);
 #endif
	sendPLOC(controllers[controllerIndex].session);
  }
  // Do nothing if there is a CANCMD present - it will assign sessions.
}

/*
* A throttle has requested access to a particular consist address
* This routine is only used if there is no CANCMD on the bus that will
* allocate sessions.
*/
void consistRequest(unsigned int address)
{
#if DEBUG
	Serial.println(F("ConsistRequest"));
#endif
	// only respond if working standalone
	if (cancmd_present == FALSE)
	{
#if DEBUG
		Serial.println(F("Standalone"));
		byte index;
#endif
		if ((address > 0) && address < 128)
		{
			//This is a DCC Address associated with our consists
			boolean found = false;

			for (index = 0; index < NUM_CONTROLLERS; index++)
			{
				if (controllers[index].consist.address == address)
				{
					found = true;
					break;
				}
			}

			if (found == true)
			{
				if (controllers[index].consist.session > 0)
				{
#if DEBUG
					Serial.print(F("Consist in use "));
					Serial.println(address);
#endif
					sendError(address, 0, locoTaken);
					return;
				}
			}
			else
			{
#if DEBUG
				Serial.print(F("Consist not found "));
				Serial.println(address);
#endif
				sendError(address, 0, consistEmpty);
				return;
			}
		}
		else
		{
			// This DCC Address is not associated with any of our consists
#if DEBUG
			Serial.print(F("Invalid consist address: "));
			Serial.println(address);
#endif
			sendError(address, 0, invalidRequest);
			return;
		}
		// If we have got this far then the consist is not in use.
		// Set a new session number for the consist - same as address with MSB to 1.
		// The session id is common across all CANCMDDC instances.
		controllers[index].consist.session = address | 0x80;
#if DEBUG
		Serial.print(F("Consist Session Allocated: "));
		Serial.println(address | 0x80);
#endif
		sendPLOCConsist(address);
	}
	// Do nothing if there is a CANCMD present - it will assign sessions.
}

/**
 * Send a PLOC message in response to a CAB requesting a session for
 * a DCC address
 */
void sendPLOC(byte session)
{
  unsigned char buf[8];
  int controllerIndex = getSessionIndex(session);
  // only send this response if working standalone
  if (cancmd_present == FALSE)
  {
   #if DEBUG
	 Serial.print(F("Send PLOC "));
   #endif
	 if (cbusActive == true)
	 {
		 buf[0] = 0xE1; // OPC_PLOC
		 buf[1] = session;
		 buf[2] = ((controllers[controllerIndex].DCCAddress >> 8) & 0x3f) | (controllers[controllerIndex].longAddress);
		 buf[3] = (controllers[controllerIndex].DCCAddress) & 0xff;
		 buf[4] = controllers[controllerIndex].trainController.getSpeed() | (controllers[controllerIndex].trainController.getDirection() * 0x80);
		 buf[5] = 0;  // Zero function bytes
		 buf[6] = 0;
		 buf[7] = 0;
		 CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 8, buf);
	 }
   #if DEBUG
		  Serial.print(F("CAN msg: "));
		  for(int i = 0; i<8; i++)                // Print each byte of the data
		  {
			Serial.print(F(" "));
			if(buf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
			{
			  Serial.print(F("0"));
			}
			Serial.print(buf[i], HEX);
		  }
		  Serial.println();
   #endif
  }
}

void sendPLOCConsist(byte address)
{
	unsigned char buf[8];
	// only send this response if working standalone
	if (cancmd_present == FALSE)
	{
		// only send this response if 1st device on bus - we don't want up to 8 identical messages sent
		if (deviceAddress == 0)
		{
#if DEBUG
			Serial.print(F("Send PLOC "));
#endif
			if (cbusActive == true)
			{
				buf[0] = 0xE1; // OPC_PLOC
				buf[1] = address | 0x80;
				buf[2] = 0;
				buf[3] = address;
				buf[4] = 0;
				buf[5] = 0;  // Zero function bytes
				buf[6] = 0;
				buf[7] = 0;
				CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 8, buf);
			}
#if DEBUG
			Serial.print(F("CAN msg: "));
			for (int i = 0; i < 8; i++)                // Print each byte of the data
			{
				Serial.print(F(" "));
				if (buf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
				{
					Serial.print(F("0"));
				}
				Serial.print(buf[i], HEX);
			}
			Serial.println();
#endif
		}
	}
}

/**
* Send a TO-N/F message in response to the PSU status changing
*/
void sendTPower(byte on)
{
	unsigned char buf[1];

#if DEBUG
	Serial.print(F("Send TOF/N "));
#endif
	if (cbusActive == true)
	{
		buf[0] = (0x04 + on); // OPC_TOF/TON
		CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 0, 1, buf);
	}
#if DEBUG
	Serial.print(F("CAN msg: "));
	Serial.println(on);
#endif
}

#endif
/**
* Send a DSPD message to CABs showing speed/direction
*/
void sendDSPD(byte controllerIndex)
{
#if CANBUS
	unsigned char buf[3];

#if DEBUG
	Serial.print(F("Send DSPD "));
#endif
	if (cbusActive == true)
	{
		buf[0] = 0x47; // OPC_DSPD
		buf[1] = controllers[controllerIndex].session;
		buf[2] = controllers[controllerIndex].trainController.getSpeed() | (controllers[controllerIndex].trainController.getDirection() * 0x80);
		CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 3, buf);
	}
#if DEBUG
	Serial.print(F("CAN msg: "));
	for (int i = 0; i < 3; i++)                // Print each byte of the data
	{
		Serial.print(F(" "));
		if (buf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
		{
			Serial.print(F("0"));
		}
		Serial.print(buf[i], HEX);
	}
	Serial.println();
#endif
#endif
}

#if CANBUS
/**
* Send an RTR message to all nodes, forcing them to reply with their CAN_ID
*/
void sendRTR()
{
#if DEBUG
	Serial.println(F("Send RTR"));
#endif
	if (cbusActive == true)
	{
		// start self-enum
		canId.enumerating = true;
		canId.ptr = 0;
		CAN0.sendMsgBuf((((unsigned long)canId.id) << 5) | CAN_RTR_MASK, 0, NULL);
		delay(1000);
	}
}

/**
 * Send an error packet labelled with the DCC address
 */
void sendError(unsigned int address, byte long_address, byte code)
{
  unsigned char buf[4];
#if DEBUG
  Serial.print(F("Send Loco "));
  Serial.print(address);
  Serial.print(F(" Error "));
  Serial.println(code);
#endif
  if (cbusActive == true)
  {
	  buf[0] = 0x63; // OPC_ERR
	  buf[1] = ((address >> 8) & 0xff) | long_address;
	  buf[2] = address & 0xff;
	  buf[3] = code;
	  CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 0, 4, buf);
  }
}
#endif

/**
 * Send a session error message to the CABs, labelled with the session number
 */
void sendSessionError(byte session, errorStates code)
{
#if DEBUG
	Serial.print(F("Send Session "));
	Serial.print(session);
	Serial.print(F(" Error "));
	Serial.println(code);
#endif
#if CANBUS
  unsigned char buf[4];
  if (cbusActive == true)
  {
	  buf[0] = 0x63; // OPC_ERR
	  buf[1] = session;
	  buf[2] = 0;
	  buf[3] = code;
	  CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 0, 4, buf);
  }
#endif
}

#if CANBUS
/**
 * Send a reset signal to all connected CABs
 */
void sendReset()
{
	if (cbusActive == true)
	{
		unsigned char buf[1];
		int i;
		buf[0] = 0x07; // OPC_ARST
		for (i = 0; i < 5; i++)
		{
			CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 0, 1, buf);
			delay(100);
		}
		buf[0] = 0x03; // OPC_BON
		CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 0, 1, buf);
		delay(100);
		sendTPower(1);
	}
}

void addSessionConsist(byte session, byte consist)
{
#if DEBUG
	Serial.print(F("Add to consist: "));
	Serial.println(consist);
#endif

	// does the session belong to this controller?
	int index = getSessionIndex(session);

	if (index == SF_UNHANDLED)
		return;

	// is the session already in a consist?
	removeSessionConsist(session);

	// add the consist address to the loco
	controllers[index].consist = { (consist & 0x7f), 0, ((consist & 0x80) == 0x80) };
}

void removeSessionConsist(byte session)
{
#if DEBUG
	Serial.print(F("Remove from consist: "));
	Serial.println(session);
#endif

	for (byte i = 0; i < NUM_CONTROLLERS; i++)
	{
		if (controllers[i].consist.address == (session & 0x7f))
		{
			controllers[i].consist.session = 0;
		}
	}
}

void setSpeedAndDirection(byte index, byte requestedSpeed, byte reverse)
{
	if ((requestedSpeed & 0x7F) == 1)
	{
		// emergency stop
		controllers[index].trainController.emergencyStop();
#if ENCODER
		encoders[index].encoderController.write(0);
#endif
	}
	else
	{
		controllers[index].trainController.setSpeedAndDirection(((requestedSpeed & 0x80) ^ reverse) >> 7, requestedSpeed & 0x7f);
#if ENCODER
		encoders[index].encoderController.write(requestedSpeed & 0x7f);
#endif
	}
	// update the speed display.
	displaySpeed(index);
}
#endif

void setSpeedSteps(byte session, byte steps)
{
  // This is only relevent for DCC, so can be ignored
}

/**
 * Stop all DC tracks immediately
 */
void emergencyStopAll()
{
	stopAll(true);

#if LCD_DISPLAY
	display.backlight();
	displayStopLogo();
	previousTurnon = millis();
#endif

	beep_counter = 200; // sound buzzer 2 seconds

#if CANBUS
	if (cbusActive == true)
	{
		// Tell all the CABs and Throttles
		unsigned char buf[1];
		// Tell all the cabs
		buf[0] = 0x06; // ESTOP
		CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 0, 1, buf);
	}
#endif
}

/**
* Stop all DC tracks
* Loop over every session and if it is not free set
* the speed to 0 (stop) or 1 (eStop)
*/
void stopAll(boolean emergency)
{
	for (byte index = 0; index < NUM_CONTROLLERS; index++)
	{
		// stop all active controllers
		if (controllers[index].session != SF_INACTIVE)
		{
			if (emergency)
				controllers[index].trainController.emergencyStop();
			else
				controllers[index].trainController.setSpeed(0);
			// update the speed display.
			displaySpeed(index);
			sendDSPD(index);
		}
#if ENCODER
		encoders[index].encoderController.write(0);
#endif
	}
}

// interrupt service routine that wraps a user defined function
// supplied by attachInterrupt
ISR(TIMER5_OVF_vect)
{
  // Fires 100 times per second
  noInterrupts();
  TCNT5 = TIMER_PRELOAD;            // preload timer

  if (flash_counter > 0)
  {
	// switch off LED when flash_counter reaches 0
	flash_counter--;
	if (flash_counter == 0)
	{
	  digitalWrite(LED, OFF);
	}
  }

  timer_counter--;
  if (timer_counter <= 0)
  {
	// increment timeout counters every second
	for (byte controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
	{
	  if (controllers[controllerIndex].session != SF_INACTIVE)
	  {
		++controllers[controllerIndex].timeout; // increment timeout counter by 1
	  }
	}
	timer_counter = 100; // timer fire at 100Hz
  }

  update_counter--;
  if (update_counter <= 0)
  {
	// update controller speed 10 times every second
	updateNow = true;
	update_counter = 10; // timer fires at 100Hz
  }

  // should the beeper be sounding?
  if (beep_counter > 0)
  {
	// switch off buzzer when beep_counter reaches 0
	analogWrite(SOUNDER, 50);
	beep_counter--;
	if (beep_counter == 0)
	{
	  digitalWrite(SOUNDER, HIGH);
	}
  }
  interrupts();
}

#if CANBUS
// External interrupt routine to get a CBus message and add it to the FIFO buffer
void receiveCBusMessage()
{
	if (cbusActive == true)
	{
		messageRecordType newMessage;
		noInterrupts();
		digitalWrite(LED, ON);
		flash_counter = 1;
		if (CAN_MSGAVAIL == CAN0.checkReceive())            // Check to see whether data is received.
		{
			// Get the message received from the CBus
			CAN0.readMsgBuf(&newMessage.canId, &newMessage.len, newMessage.rxBuf);  // Read data: canId = Id, len = data length, buf = data byte(s)
#if DEBUG
			// the CanId bytes are ppppHHHH LLLxxxxx where p = priority bit, x is unused, H & L are high and low bits
			// see: https://www.merg.org.uk/forum/viewtopic.php?f=44&t=3018&p=24032&hilit=re+enumeration#p23986
			Serial.print(F("Recd. CanId: "));
			Serial.println(newMessage.canId, HEX);
#endif
			bufferOverflow = (fifoMessageBuffer.addMessage(newMessage) == 1);
			if (bufferOverflow == true)
			{
				// Buffer overflow
				beep_counter = 100; // sound buzzer
			}
		}
		interrupts();
	}
}
#endif

// N beeps
void nBeeps (byte numBeeps,
			 int durationMillis)
{
  for (int i = 0; i < numBeeps; i++)
  {
	analogWrite(SOUNDER, 50);
	delay(durationMillis);
	digitalWrite(SOUNDER, HIGH);
	delay(durationMillis);
  }
}

// set up fixed text on display
void setupDisplay()
{
	// we will display controllers in 2 columns on 3 lines (max 6 controllers)
#if OLED_DISPLAY || LCD_DISPLAY
	int line; // 0-2
	int column; // 0 or 1
	int x_pos;
	int y_pos;

	// Clear the buffer.
#if OLED_DISPLAY
	float barsize;
	display.clearDisplay();
	display.setTextSize(1);
#else
	display.clear();
#endif

	for (byte controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
	{
		line = controllerIndex / 2;
		column = controllerIndex - (line * 2);

#if OLED_DISPLAY
		x_pos = column * 67;
		y_pos = line * 12;
#else
		x_pos = column * 10;
		y_pos = line;
#endif

		display.setCursor(x_pos, y_pos);
		display.print(controllers[controllerIndex].DCCAddress); // display DCC address

//		if (init)
//		{
//#if OLED_DISPLAY
//			display.setCursor((x_pos)+30, y_pos);
//#else
//			display.setCursor((x_pos)+5, y_pos);
//#endif
//
//			display.write("Free");
//		}
	}

	display.display();

	if (!barGraphicsLoaded)
		setupBarGraph();

	showingSpeeds = true;

#endif
}

#if OLED_DISPLAY || LCD_DISPLAY
void showSpeeds()
{
	if (!showingSpeeds)
		setupDisplay();

	for (byte controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
	{
		displaySpeed(controllerIndex);
	}

}
#endif

// Display the current speed settings
void displaySpeed(byte controllerIndex)
{
  // we will display controllers in 2 columns on 3 lines (max 6 controllers)
#if OLED_DISPLAY || LCD_DISPLAY
  int line = controllerIndex / 2; // 0-2
  int column = controllerIndex - (line * 2); // 0 or 1
  int x_pos;
  int y_pos;

  display.backlight();
  previousTurnon = millis();

#if OLED_DISPLAY
	x_pos = column*67;
	y_pos = line*12;
	if (controllers[controllerIndex].session != SF_INACTIVE)
	{
	  float barsize = (controllers[controllerIndex].trainController.getSpeed()*30)/128;
	  display.fillRect((x_pos)+30,y_pos,30,8, BLACK);
	  display.drawRect((x_pos)+30,y_pos,30,8, WHITE);
	  display.fillRect((x_pos)+30,y_pos,barsize,8, WHITE);
	}
	else
	{
	  display.fillRect((x_pos)+30,y_pos,30,8, BLACK);
	  display.setCursor((x_pos)+30, y_pos);
	  display.println("Free");
	}
  display.display();
#else
  if (showingSpeeds)
  {
	  x_pos = column * 10;
	  y_pos = line;
	  if (controllers[controllerIndex].session == SF_INACTIVE)
	  {
		  display.setCursor((x_pos)+4, y_pos);
		  display.write("-Free-");
	  }
	  else
	  {
		  display.setCursor((x_pos)+4, y_pos);

		  int speed = controllers[controllerIndex].trainController.getSpeed();

		  //display.print(speed); // in text

		  // There are 6 sections in the bar-graph, each filled with of 1 of 6 characters (0-4 and <space>) for that part of the speed.
		  // There are 31 unique bar-graphs, meaning that each graph handles 128 / 31 = 4.1 speed steps.
		  // Speed 0 is Stop, and Graph 0 (nothing displayed) is otherwise unused,
		  // so the first 3 graphs and the last one show more speed steps than the others, covering all remaining 127 speed steps.

#pragma region section 1

		  if (controllers[controllerIndex].trainController.eStopped == true)
			display.print("-STOP-");
		  else
		  {
			  switch (speed)
			  {
			  case 0:
				  display.print("-Stop-");
				  break;
			  case 1:
			  case 2:
			  case 3:
			  case 4:
			  case 5:
			  case 6:
				  display.print(char(0));
				  break;
			  case 7:
			  case 8:
			  case 9:
			  case 10:
			  case 11:
				  display.print(char(1));
				  break;
			  case 12:
			  case 13:
			  case 14:
			  case 15:
			  case 16:
				  display.print(char(2));
				  break;
			  case 17:
			  case 18:
			  case 19:
			  case 20:
				  display.print(char(3));
				  break;
			  default:
				  display.print(char(4));
				  break;
			  }
		  }
#pragma endregion

#pragma region section 2
		  switch (speed)
		  {
		  case 0:
			  break;
		  case 1:
		  case 2:
		  case 3:
		  case 4:
		  case 5:
		  case 6:
		  case 7:
		  case 8:
		  case 9:
		  case 10:
		  case 11:
		  case 12:
		  case 13:
		  case 14:
		  case 15:
		  case 16:
		  case 17:
		  case 18:
		  case 19:
		  case 20:
		  case 21:
		  case 22:
		  case 23:
		  case 24:
			  display.print(" ");
			  break;
		  case 25:
		  case 26:
		  case 27:
		  case 28:
			  display.print(char(0));
			  break;
		  case 29:
		  case 30:
		  case 31:
		  case 32:
			  display.print(char(1));
			  break;
		  case 33:
		  case 34:
		  case 35:
		  case 36:
			  display.print(char(2));
			  break;
		  case 37:
		  case 38:
		  case 39:
		  case 40:
			  display.print(char(3));
			  break;
		  default:
			  display.print(char(4));
			  break;
		  }
#pragma endregion

#pragma region section 3
		  switch (speed)
		  {
		  case 0:
			  break;
		  case 1:
		  case 2:
		  case 3:
		  case 4:
		  case 5:
		  case 6:
		  case 7:
		  case 8:
		  case 9:
		  case 10:
		  case 11:
		  case 12:
		  case 13:
		  case 14:
		  case 15:
		  case 16:
		  case 17:
		  case 18:
		  case 19:
		  case 20:
		  case 21:
		  case 22:
		  case 23:
		  case 24:
		  case 25:
		  case 26:
		  case 27:
		  case 28:
		  case 29:
		  case 30:
		  case 31:
		  case 32:
		  case 33:
		  case 34:
		  case 35:
		  case 36:
		  case 37:
		  case 38:
		  case 39:
		  case 40:
		  case 41:
		  case 42:
		  case 43:
		  case 44:
			  display.print(" ");
			  break;
		  case 45:
		  case 46:
		  case 47:
		  case 48:
			  display.print(char(0));
			  break;
		  case 49:
		  case 50:
		  case 51:
		  case 52:
			  display.print(char(1));
			  break;
		  case 53:
		  case 54:
		  case 55:
		  case 56:
			  display.print(char(2));
			  break;
		  case 57:
		  case 58:
		  case 59:
		  case 60:
			  display.print(char(3));
			  break;
		  default:
			  display.print(char(4));
			  break;

		  }
#pragma endregion

#pragma region section 4
		  switch (speed)
		  {
		  case 0:
			  break;
		  case 1:
		  case 2:
		  case 3:
		  case 4:
		  case 5:
		  case 6:
		  case 7:
		  case 8:
		  case 9:
		  case 10:
		  case 11:
		  case 12:
		  case 13:
		  case 14:
		  case 15:
		  case 16:
		  case 17:
		  case 18:
		  case 19:
		  case 20:
		  case 21:
		  case 22:
		  case 23:
		  case 24:
		  case 25:
		  case 26:
		  case 27:
		  case 28:
		  case 29:
		  case 30:
		  case 31:
		  case 32:
		  case 33:
		  case 34:
		  case 35:
		  case 36:
		  case 37:
		  case 38:
		  case 39:
		  case 40:
		  case 41:
		  case 42:
		  case 43:
		  case 44:
		  case 45:
		  case 46:
		  case 47:
		  case 48:
		  case 49:
		  case 50:
		  case 51:
		  case 52:
		  case 53:
		  case 54:
		  case 55:
		  case 56:
		  case 57:
		  case 58:
		  case 59:
		  case 60:
		  case 61:
		  case 62:
		  case 63:
		  case 64:
			  display.print(" ");
			  break;
		  case 65:
		  case 66:
		  case 67:
		  case 68:
			  display.print(char(0));
			  break;
		  case 69:
		  case 70:
		  case 71:
		  case 72:
			  display.print(char(1));
			  break;
		  case 73:
		  case 74:
		  case 75:
		  case 76:
			  display.print(char(2));
			  break;
		  case 77:
		  case 78:
		  case 79:
		  case 80:
			  display.print(char(3));
			  break;
		  default:
			  display.print(char(4));
			  break;
		  }
#pragma endregion

#pragma region section 5
		  switch (speed)
		  {
		  case 0:
			  break;
		  case 1:
		  case 2:
		  case 3:
		  case 4:
		  case 5:
		  case 6:
		  case 7:
		  case 8:
		  case 9:
		  case 10:
		  case 11:
		  case 12:
		  case 13:
		  case 14:
		  case 15:
		  case 16:
		  case 17:
		  case 18:
		  case 19:
		  case 20:
		  case 21:
		  case 22:
		  case 23:
		  case 24:
		  case 25:
		  case 26:
		  case 27:
		  case 28:
		  case 29:
		  case 30:
		  case 31:
		  case 32:
		  case 33:
		  case 34:
		  case 35:
		  case 36:
		  case 37:
		  case 38:
		  case 39:
		  case 40:
		  case 41:
		  case 42:
		  case 43:
		  case 44:
		  case 45:
		  case 46:
		  case 47:
		  case 48:
		  case 49:
		  case 50:
		  case 51:
		  case 52:
		  case 53:
		  case 54:
		  case 55:
		  case 56:
		  case 57:
		  case 58:
		  case 59:
		  case 60:
		  case 61:
		  case 62:
		  case 63:
		  case 64:
		  case 65:
		  case 66:
		  case 67:
		  case 68:
		  case 69:
		  case 70:
		  case 71:
		  case 72:
		  case 73:
		  case 74:
		  case 75:
		  case 76:
		  case 77:
		  case 78:
		  case 79:
		  case 80:
		  case 81:
		  case 82:
		  case 83:
		  case 84:
			  display.print(" ");
			  break;
		  case 85:
		  case 86:
		  case 87:
		  case 88:
			  display.print(char(0));
			  break;
		  case 89:
		  case 90:
		  case 91:
		  case 92:
			  display.print(char(1));
			  break;
		  case 93:
		  case 94:
		  case 95:
		  case 96:
			  display.print(char(2));
			  break;
		  case 97:
		  case 98:
		  case 99:
		  case 100:
			  display.print(char(3));
			  break;
		  default:
			  display.print(char(4));
			  break;
		  }
#pragma endregion

#pragma region section 6
		  switch (speed)
		  {
		  case 0:
			  break;
		  case 1:
		  case 2:
		  case 3:
		  case 4:
		  case 5:
		  case 6:
		  case 7:
		  case 8:
		  case 9:
		  case 10:
		  case 11:
		  case 12:
		  case 13:
		  case 14:
		  case 15:
		  case 16:
		  case 17:
		  case 18:
		  case 19:
		  case 20:
		  case 21:
		  case 22:
		  case 23:
		  case 24:
		  case 25:
		  case 26:
		  case 27:
		  case 28:
		  case 29:
		  case 30:
		  case 31:
		  case 32:
		  case 33:
		  case 34:
		  case 35:
		  case 36:
		  case 37:
		  case 38:
		  case 39:
		  case 40:
		  case 41:
		  case 42:
		  case 43:
		  case 44:
		  case 45:
		  case 46:
		  case 47:
		  case 48:
		  case 49:
		  case 50:
		  case 51:
		  case 52:
		  case 53:
		  case 54:
		  case 55:
		  case 56:
		  case 57:
		  case 58:
		  case 59:
		  case 60:
		  case 61:
		  case 62:
		  case 63:
		  case 64:
		  case 65:
		  case 66:
		  case 67:
		  case 68:
		  case 69:
		  case 70:
		  case 71:
		  case 72:
		  case 73:
		  case 74:
		  case 75:
		  case 76:
		  case 77:
		  case 78:
		  case 79:
		  case 80:
		  case 81:
		  case 82:
		  case 83:
		  case 84:
		  case 85:
		  case 86:
		  case 87:
		  case 88:
		  case 89:
		  case 90:
		  case 91:
		  case 92:
		  case 93:
		  case 94:
		  case 95:
		  case 96:
		  case 97:
		  case 98:
		  case 99:
		  case 100:
		  case 101:
		  case 102:
		  case 103:
		  case 104:
			  display.print(" ");
			  break;
		  case 105:
		  case 106:
		  case 107:
		  case 108:
			  display.print(char(0));
			  break;
		  case 109:
		  case 110:
		  case 111:
		  case 112:
			  display.print(char(1));
			  break;
		  case 113:
		  case 114:
		  case 115:
		  case 116:
			  display.print(char(2));
			  break;
		  case 117:
		  case 118:
		  case 119:
		  case 120:
			  display.print(char(3));
			  break;
		  default:
			  display.print(char(4));
			  break;
		  }
#pragma endregion

#pragma region status line
		  if (NUM_CONTROLLERS < 7)
		  {
			  display.setCursor(0, 3);
			  display.print("Last:");
			  display.print(controllers[controllerIndex].DCCAddress);

			  display.setCursor(10, 3);
			  display.print("Speed:");
			  display.print(to3digits(controllers[controllerIndex].trainController.getSpeed()));

			  display.setCursor(19, 3);
			  display.print(char(5 + controllers[controllerIndex].trainController.getDirection()));
		  }
#pragma endregion
	  }
  }
  else
  {
	  setupDisplay();
	  showSpeeds();
  }
#endif
#endif
}

String to3digits(uint8_t speed)
{
	char s[4];
	dtostrf(speed, 3, 0, s);
	return s;
}

void displayImage(const uint8_t *imageBitmap)
{
#if OLED_DISPLAY
  // Clear the buffer.
  display.clearDisplay();
  // Show Merg logo on the display hardware.
  display.drawBitmap(0, 0, imageBitmap, 128, 32, 1);
  display.display();
  // leave logo on screen for a while
  delay(1500);
#endif
}

void displayVersion()
{
#if OLED_DISPLAY
  // Clear the buffer.
  display.clearDisplay();
  // display module name and version for a short time
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20,8);
  display.println("CANCMDDC");
  display.setTextSize(1);
  display.setCursor(60,24);
  display.println("v1.9");
  display.display();
#endif
#if LCD_DISPLAY
  // Clear the buffer.
  display.clear();

  display.clear();
  display.setCursor(3, 0);
  display.write("CANCMDDC v");
  display.print(VERSION);
  display.setCursor(0, 1);
  display.write(char(7));
  display.setCursor(2, 1);
  display.write("David W Radcliffe");
  display.setCursor(0, 2);
  display.write("Based upon work by:");
  display.setCursor(0, 3);
  display.write("I.Morgan & M.Riddoch");

#endif
  delay(2000);
}

void initialiseDisplay()
{
#if OLED_DISPLAY
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // Clear the buffer.
  display.clearDisplay();
#endif
#if LCD_DISPLAY
  display.begin(20, 4); // initialize the lcd for 20 chars 4 lines, turn on backlight
  display.display();    // turn it on
  display.clear();
#endif
}

#if LCD_DISPLAY
void displayMergLogo()
{
	// Creat a set of new characters
	const uint8_t mergLogo[][8] = {
		{ B00001111, B00011111, B00011111, B00011111, B00011100, B00011100, B00011100, B00011100 }, // 0
		{ B00011111, B00011111, B00011111, B00011111, B00000000, B00000000, B00000000, B00000000 }, // 1
		{ B00011100, B00011100, B00011100, B00011100, B00011100, B00011100, B00011100, B00011100 }, // 2
		{ B00000000, B00000000, B00000000, B00000000, B00011111, B00011111, B00011111, B00011111 }, // 3
		{ B00000111, B00000111, B00000111, B00000111, B00000111, B00000111, B00000111, B00000111 }, // 4
		{ B00000000, B00000000, B00000000, B00000000, B00011111, B00011111, B00011111, B00011111 }, // 5
		{ B00011111, B00011111, B00011111, B00011111, B00001111, B00000111, B00000111, B00000111 }, // 6
		{ B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111 }  // 7
	};

	customChars(mergLogo);

	char chars[4][20] = {
	char(0), char(1), char(6), char(1), char(1), char(2), ' ', char(0), char(1), char(1), ' ', char(0), char(1), char(1), char(2), ' ', char(0), char(1), char(1), char(2),
	char(2), ' ',     char(4), ' ',     ' ',     char(2), ' ', char(2), ' ',     ' ',     ' ', char(2), ' ',     ' ',     char(2), ' ', char(2), ' ',     ' ',     ' ',
	char(7), ' ',     char(4), ' ',     ' ',     char(2), ' ', char(7), char(1), ' ',     ' ', char(7), char(1), char(1), char(6), ' ', char(7), ' ',     char(1), char(2),
	char(7), ' ',     char(4), ' ',     ' ',     char(2), ' ', char(7), char(3), char(3), ' ', char(7), ' ',     ' ',     char(4), ' ', char(7), char(3), char(3), char(2)
	};
	displayLogo(chars);

	delay(2000);
}

void displayStopLogo()
{
	// Creat a set of new characters
	const uint8_t stopLogo[][8] = {
		{ B00000000, B00000001, B00000011, B00000111, B00001111, B00011111, B00011111, B00011111 }, // 0
		{ B00000000, B00010000, B00011000, B00011100, B00011110, B00011111, B00011111, B00011111 }, // 1
		{ B00011111, B00011111, B00011111, B00001111, B00000111, B00000011, B00000001, B00000000 }, // 2
		{ B00011111, B00011111, B00011111, B00011110, B00011100, B00011000, B00010000, B00000000 }, // 3
		{ B00001111, B00000111, B00000011, B00000001, B00000000, B00000000, B00000000, B00000000 }, // 4
		{ B00000000, B00000000, B00000000, B00000000, B00010000, B00011000, B00011100, B00011110 }, // 5
		{ B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111 }, // 6
		{ B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111 }  // 7
	};

	customChars(stopLogo);

	char chars[4][20] = {
		char(0), char(7), char(7), char(1), ' ', char(7), char(7), char(7), char(7), ' ', char(0), char(7), char(7), char(7), char(1), ' ', char(7), char(7), char(7), char(1),
		char(2), char(7), char(5), char(4), ' ', ' ',     char(7), ' ',     ' ',	 ' ', char(7), ' ',	    ' ',     ' ',	  char(7), ' ', char(7), ' ',     ' ',	   char(7),
		char(5), char(4), char(7), char(1), ' ', ' ',     char(7), ' ',     ' ',	 ' ', char(7), ' ',	    ' ',     ' ',	  char(7), ' ', char(7), char(7), char(7), char(3),
		char(2), char(7), char(7), char(3), ' ', ' ',     char(7), ' ',     ' ',	 ' ', char(2), char(7), char(7), char(7), char(3), ' ', char(7), ' ',     ' ',	   ' '
	};
	displayLogo(chars);

	barGraphicsLoaded = false;
	showingSpeeds     = false;
}

void setupBarGraph()
{
	// Creat a set of new characters
	const uint8_t bars[][8] = {
		{ B00010000, B00010000, B00010000, B00010000, B00010000, B00010000, B00010000, B00000000 }, // 0 - 1 segment
		{ B00011000, B00011000, B00011000, B00011000, B00011000, B00011000, B00011000, B00000000 }, // 1 - 2 segments
		{ B00011100, B00011100, B00011100, B00011100, B00011100, B00011100, B00011100, B00000000 }, // 2 - 3 segments
		{ B00011110, B00011110, B00011110, B00011110, B00011110, B00011110, B00011110, B00000000 }, // 3 - 4 segments
		{ B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00000000 }, // 4 - 5 segments
		{ B00000001, B00000011, B00000111, B00001111, B00000111, B00000011, B00000001, B00000000 }, // 5 - < arrow rev
		{ B00010000, B00011000, B00011100, B00011110, B00011100, B00011000, B00010000, B00000000 }, // 6 - > arrow fwd
		{ B00001110, B00011011, B00010101, B00010111, B00010101, B00011011, B00001110, B00000000 }, // 7 - (c) symbol
	};

	customChars(bars);

	barGraphicsLoaded = true;
}

void customChars(const uint8_t chars[][8])
{
	for (int i = 0; i < 8; i++)
	{
		display.createChar(i, (uint8_t *)chars[i]);
	}
}

void displayLogo(const char chars[4][20])
{
#if LCD_DISPLAY
	for (int i = 0; i < 4; i++)
	{
		display.setCursor(0, i);
		displayChars(chars[i], 20);
	}
#endif
}

void displayChars(const char chars[20], int count)
{
#if LCD_DISPLAY
	for (int j = 0; j < count; j++)
	{
		display.write(chars[j]);
	}
#endif
}
#endif

#if KEYPAD
void keypadEvent(KeypadEvent key)
{
#if DEBUG
	const String states[] = {
		F("Idle "),
		F("Loco Select "),
		F("Loco Stop "),
		F("Speed Select "),
		F("Speed Digit "),
		F("Loco Emerg Stop "),
		F("Steal or Share ")
	};
#endif

	// Taking care of keypad events.
	switch (keyPad.getState())
	{
	case PRESSED:
		previousKeypress = 0;
		nBeeps(1, 10);

#if OLED_DISPLAY || LCD_DISPLAY
		display.backlight();
		previousTurnon = millis();
#endif

		#if DEBUG
		Serial.print(states[keyFSM.state]);
		Serial.println(key);
		#endif

		if (keyFSM.state == idle)
			keyFSM.state = locoSelect;
		else
		{
			switch (key)
			{
			case '*':
				switch (keyFSM.state)
				{
				case idle:
				case locoSelect:    // stop current loco
					if (keyFSM.currentLoco != 255)
					{
						controllers[keyFSM.currentLoco].trainController.setSpeed(0);
#if ENCODER
						encoders[keyFSM.currentLoco].encoderController.write(0);
#endif
						if (controllers[keyFSM.currentLoco].shared)
							sendDSPD(keyFSM.currentLoco);

						keyFSM.state = locoStop;
					}
					break;
				case locoStop:      // stop all locos.
					stopAll(false);
					keyFSM.state = speedSelect;
					break;
				case speedSelect:   // Enter Loco select mode. Timeout after 3 secs to Speed select mode.
					keyFSM.state = locoSelect;
					break;
				case speedDigit:    // cancel current input and enter Speed select mode.
				case locoEmergStop: // enter Speed select mode.
					keyFSM.state = speedSelect;
					break;
				case stealOrShare:  // share loco
					keyFSM.state = speedSelect;
					controllers[keyFSM.currentLoco].shared = true;
					break;
				}
				break;
			case '#':
				switch (keyFSM.state)
				{
				case locoSelect:    // enter Speed select mode
					keyFSM.state = speedSelect;
					break;
				case locoStop:      // change direction
					if (keyFSM.currentLoco != 255)
					{
						controllers[keyFSM.currentLoco].trainController.setSpeedAndDirection(controllers[keyFSM.currentLoco].trainController.getDirection() ^ 1, 0);

						if (controllers[keyFSM.currentLoco].shared)
							sendDSPD(keyFSM.currentLoco);

						keyFSM.state = speedSelect;
					}
					break;
				case idle:
				case speedSelect:   // eStop selected loco.
					if (keyFSM.currentLoco != 255)
					{
						controllers[keyFSM.currentLoco].trainController.emergencyStop();
#if ENCODER
						encoders[keyFSM.currentLoco].encoderController.write(0);
#endif
						if (controllers[keyFSM.currentLoco].shared)
							sendDSPD(keyFSM.currentLoco);

						keyFSM.state = locoEmergStop;
					}
					break;
				case speedDigit:    // accept entered speed.  If invalid(> 127) cancel current input and enter Speed select mode.
				{
#if DEBUG
					Serial.print(F("Digits are: "));
					Serial.print(keyFSM.digits[0]);
					Serial.print(keyFSM.digits[1]);
					Serial.println(keyFSM.digits[2]);
#endif


					int speed = atoi(keyFSM.digits);
#if DEBUG
					Serial.print(F("Speed is: "));
					Serial.println(speed);
#endif
					if (speed > 127)
					{
						nBeeps(1, 100);
						keyFSM.digits[2] = ' ';
						keyFSM.digits[1] = ' ';
						keyFSM.digits[0] = ' ';
						keyFSM.state = speedSelect;
					}
					else
					{
#if DEBUG
						Serial.print(F("currentLoco: "));
						Serial.println(keyFSM.currentLoco + 1);
#endif
						if (keyFSM.currentLoco != 255)
						{
							controllers[keyFSM.currentLoco].trainController.setSpeed(speed);
#if ENCODER
							encoders[keyFSM.currentLoco].encoderController.write(speed);
#endif
#if DEBUG
							Serial.print(F("Session: "));
							Serial.println(controllers[keyFSM.currentLoco].session);
#endif
							if (controllers[keyFSM.currentLoco].session > SF_INACTIVE)
							{
#if DEBUG
								Serial.print(F("Shared: "));
								Serial.println(controllers[keyFSM.currentLoco].shared);
#endif
								if (controllers[keyFSM.currentLoco].shared)
									sendDSPD(keyFSM.currentLoco);
								else
									sendSessionError(controllers[keyFSM.currentLoco].session, sessionCancelled); // session has been stolen, so cancel session on owning CAB
							}
						}

						keyFSM.state = idle;
#if OLED_DISPLAY || LCD_DISPLAY
						showingSpeeds = false;
						showSpeeds();
#endif
						return;
					}

					break;
				}
				case locoEmergStop: // eStop all locos.
					emergencyStopAll();
					keyFSM.state = idle;
					return;
					break;
				case stealOrShare: // steal loco from CAB
					sendSessionError(controllers[keyFSM.currentLoco].session, locoTaken); // session has been taken, so send message to CAB
					controllers[keyFSM.currentLoco].shared = false;
					controllers[keyFSM.currentLoco].session = SF_LOCAL;
					keyFSM.state = speedSelect;
					break;
				}
				break;

			default: // digit 0-9
				switch (keyFSM.state)
				{
				case locoSelect:    // Select the loco and handle shared status
				{
					int selectedLoco = String(key).toInt();

					if (selectedLoco != 0
						&&
						selectedLoco <= String(NUM_CONTROLLERS).toInt()
						)
					{
						keyFSM.previousLoco = keyFSM.currentLoco;
						keyFSM.currentLoco = (selectedLoco - 1);

						if (controllers[keyFSM.currentLoco].session <= SF_INACTIVE) // not owned by CAB
						{
							controllers[keyFSM.currentLoco].session = SF_LOCAL;
							keyFSM.state = speedSelect; // Enter Speed select mode with that loco selected.
						}
						else
						{
							keyFSM.state = stealOrShare;
						}
					}
					break;
				}
				case locoStop:      // <nothing>
					break;
				case speedSelect:   // Enter speed digit.
					keyFSM.digits[0] = ' ';
					keyFSM.digits[1] = ' ';
					keyFSM.digits[2] = key;
					keyFSM.state = speedDigit;

					//Serial.println(previousKeypress);

					break;
				case speedDigit:    // Enter speed digit. Scroll to left.

					keyFSM.digits[0] = keyFSM.digits[1];
					keyFSM.digits[1] = keyFSM.digits[2];
					keyFSM.digits[2] = key;

					break;
				case locoEmergStop: // <nothing>
					break;
				}
			}
		}
		previousKeypress = millis();

#if LCD_DISPLAY || OLED_DISPLAY
#if LCD_DISPLAY
		displayOptions();
#else
#endif
#endif
		break;

	case RELEASED:
		break;

	case HOLD:
		break;
	}
}

void doAfter()
{
	previousKeypress = 0;

#if DEBUG
	Serial.println(F("** TimeOut **"));
#endif

	if (keyFSM.state == stealOrShare)
		keyFSM.currentLoco = keyFSM.previousLoco;

	keyFSM.state = idle;
	keyFSM.digits[2] = ' ';
	keyFSM.digits[1] = ' ';
	keyFSM.digits[0] = ' ';

#if LCD_DISPLAY || OLED_DISPLAY
	showingSpeeds = false;
	showSpeeds();
#endif
}

void displayOptions()
{
#if LCD_DISPLAY
	display.clear();
	display.setCursor(0, 0);

	if (keyFSM.state != speedDigit)
	{
		display.print(F("Current Loco is: "));
		display.print((String)((keyFSM.currentLoco + 1) % 256));
	}

	switch (keyFSM.state)
	{
	case locoSelect:
		display.setCursor(0, 1);
		display.print(F(" * to Stop Loco"));
		display.setCursor(0, 2);
		display.print(F(" # to Select Speed"));
		display.setCursor(0, 3);
		display.print(" 1-" + String(String(NUM_CONTROLLERS).toInt()));
		display.print(F(" to Select Loco"));
		break;
	case locoStop:
		display.setCursor(0, 1);
		display.print(F("Current Direction: "));
		if (showingSpeeds == false)
			setupBarGraph();
		display.print(char(5 + controllers[keyFSM.currentLoco].trainController.getDirection()));
		display.setCursor(0, 2);
		display.print(F(" * to Stop all Locos"));
		display.setCursor(0, 3);
		display.print(F(" # to Reverse Loco"));
		break;
	case speedSelect:
		display.setCursor(0, 1);
		display.print(F(" * to Select Loco"));
		display.setCursor(0, 2);
		display.print(F(" # to STOP Loco"));
		display.setCursor(0, 3);
		display.print(F(" 0-9 to enter Speed"));
		break;
	case speedDigit:
		display.print(F(" * to Cancel"));
		display.setCursor(0, 1);
		display.print(F(" # to Accept"));
		display.setCursor(0, 2);
		display.print(F(" 0-9 to Enter Speed"));
		display.setCursor(0, 3);
		display.print(F(" Entered speed: "));
		display.print(keyFSM.digits[0]);
		display.print(keyFSM.digits[1]);
		display.print(keyFSM.digits[2]);
		break;
	case locoEmergStop:
		display.setCursor(0, 2);
		display.print(F(" # to STOP all Locos"));
		break;
	case stealOrShare:
		display.setCursor(0, 1);
		display.print(F(" * to Share Loco"));
		display.setCursor(0, 3);
		display.print(F(" # to Steal Loco"));
		break;
	}
#endif
}
#endif
