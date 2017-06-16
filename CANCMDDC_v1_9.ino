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
#define DEBUG        1 // set to 0 for no debug messages, 1 for messages to console
#define OLED_DISPLAY 1 // set to 0 if 128x32 OLED display is not present

//include libraries 
#include <SPI.h>     // Library for SPI communications to CAN interface
#include <mcp_can.h>
#include <Wire.h>    // Library for I2C comminications for display
#include <PWM.h>     // Library for controlling PWM Frequency
#include "FIFO.h"
#include "trainController.h"

/* libraries for Adafruit display module */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
Adafruit 128x32 size display using I2C to communicate
3 pins are required to interface (2 I2C (SCL & SDA)and one reset (user definable))
*/
#define OLED_RESET 22

#define TRUE    1
#define FALSE   0
#define ON      1
#define OFF     0
/**
 * The following block of #defines configures the pins that are
 * used for various special functions:
 *   CHIPSELECT  is the select pin for the CANBUS interface
 *   CBUSINTPIN  is the "interrupt" pin used by the CANBUS interface
 *               to signal that a CANBUS packet is ready to be read
 *   MODESELECT  is the jumper used to determine the operating mode, standalone or with CANCMD
 */
#define LED         13     // Pin number for the LED
#define SOUNDER     4

/* Pins used by SPI interface to CAN module */
#define CHIPSELECT  53
#define CBUSINTPIN  18 // 49

/* pin used for manual selection of use with CANCMD or standalone. */
/* Link pin to +5V if standalone required. */
#define MODESELECT  48

/*
 * The following are the PWM outputs used to drive the tracks.
 *
 * Pins are used in pairs and fed to H Bridges, the first pair is
 * assigned to the first address and so on.
 * List terminates with a zero
 * 
 * The following pins can handle the high frequency PWM
 * 7, 8, 11, 12, 5, 6, 2, 3
 */
static int pwmpins[] = {
  7, 8, 11, 12, 5, 6, 2, 3, 0
};

#define NUM_CONTROLLERS  5 // the number of controllers (pairs of pwmpins)

#define MAXTIMEOUT 30      // Max number of seconds before session is timed out
                           // if no stayalive received for the session

#define TIMER_PRELOAD 64911 //59286 //65536 - ((16000000/256)/100);       // preload timer 65536-(16MHz/256/100Hz)
/**
 * The preset tests the jumper in the mode select pin to
 * determine the mode of operation. Connect pin to +5V if CANCMD is present.
 * Leave pin disconnected for standalone operation.
 * It is true if the CANCMDDC is working on a CBUS that has a CANCMD present.
 */

int cancmd_present;
volatile int timer_counter = 0;
volatile int flash_counter = 0;
volatile int beep_counter = 0;
volatile int update_counter = 0;
volatile int updateNow = 0;
/**
 * Definitions of the flags bits
 */
#define SF_FORWARDS  0x01      // Train is running forwards
#define SF_REVERSE   0x00      // Train is running in reverse
#define SF_LONG      0xC0      // long DCC address. top 2 bits of high byte. both 1 for long, both 0 for short.
#define SF_INACTIVE  -1        // Session is not active
#define SF_UNHANDLED -1        // DCC Address is not associated with an analogue controller
/**
 * The CBUS interface object
 */
MCP_CAN CAN0(CHIPSELECT);                        // MCP CAN library

struct {
  int                  session;
  long unsigned int    DCCAddress;
  int                  longAddress;
  int                  timeout;
  trainControllerClass trainController;
} controllers[NUM_CONTROLLERS] = {
                                    {SF_INACTIVE, 1000, SF_LONG, 0, trainControllerClass(30, 31, 2)},
                                    {SF_INACTIVE, 1001, SF_LONG, 0, trainControllerClass(32, 33, 3)},
                                    {SF_INACTIVE, 1002, SF_LONG, 0, trainControllerClass(34, 35, 5)},
                                    {SF_INACTIVE, 1003, SF_LONG, 0, trainControllerClass(36, 37, 6)},
                                    {SF_INACTIVE, 1004, SF_LONG, 0, trainControllerClass(38, 39, 7)}
                                  };



// instantiate a FIFO buffer for the CBus messages
CBusMessageBufferClass fifoMessageBuffer;


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


/****** Function Prototypes *****/

int getSessionIndex (int session);
int getDCCIndex (long unsigned int dcc_address, int long_address);
void releaseLoco(int session);
void queryLoco(int session);
void keepaliveSession(int session);
void locoRequest(long unsigned int address, int long_address, int flags);
void locoSession(int session, long unsigned int address, int long_address, int direction, int speed);
void sendPLOC(int session);
void sendError(long unsigned int address, int long_address, int code);
void sendSessionError(int session, int code);
void sendReset(void);
void setSpeedSteps(int session, int steps);
void emergencyStopAll(void);
void receiveCBusMessage(void);
void nBeeps (int numBeeps, int durationMillis);
void setupDisplay();
void displaySpeed(int sessionIndex);
void displayImage(const uint8_t *imageBitmap);
void displayVersion(void);
void initialiseDisplay(void);

void(* resetArduino) (void) = 0;//declare reset function at address 0

/***************************************************************************************************
 * Arduino setup routine
 * Configures the I/O pins, initialises the CANBUS library
 * and sets up the initial session stack
 */
void setup()
{
  int i;

  #if DEBUG
    // set up the IDE Serial Monitor for debugging output
    Serial.begin(115200);
    Serial.print("Initialising");
    Serial.println();
  #endif

  // set the mode for the CBus interrupt pin
  pinMode(CBUSINTPIN, INPUT);
  // set the mode for the operating mode setting input
  pinMode(MODESELECT, INPUT);
  cancmd_present = (digitalRead(MODESELECT) == HIGH); // this will be overridden if a message is received from a CANCMD
  // set the mode for the LED status indicator output and sounder
  pinMode(LED, OUTPUT);
  pinMode(SOUNDER, OUTPUT);
  
  // long beep
  nBeeps(1, 200);

#if OLED_DISPLAY
  initialiseDisplay();
  displayImage(mergLogo);
  displayImage(bnhmrsLogo);
  displayVersion();
#endif
  
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 

  // initialize timer5 which will be used to increment session timeout counters.
  // interrupt should fire every 1/10 second
  
//  noInterrupts();           // disable all interrupts
  
  TCCR5A = 0;
  TCCR5B = 0;

  TCNT5 = TIMER_PRELOAD; 
  TCCR5B |= (1 << CS12);    // 256 prescaler 
  TIMSK5 |= (1 << TOIE5);   // enable timer overflow interrupt

  // Initialise the CAN interface
  if(CAN_OK == CAN0.begin(CAN_125KBPS, 8))                   // init can bus : baudrate = 500k 
    {
      #if DEBUG
        Serial.println("CAN BUS init ok!");
      #endif
    }
    else
    {
      //CAN interface initialisation has failed
      #if DEBUG
        Serial.println("CAN BUS init fail");
      #endif
      // three beeps
      nBeeps(3, 200);
      delay(200);
      resetArduino();
    }

  if (cancmd_present == FALSE)
  {
    // Reset any connected CABs
    sendReset();
  }

  attachInterrupt(digitalPinToInterrupt(CBUSINTPIN), receiveCBusMessage, FALLING);

  // start the speed display.
  setupDisplay();

  interrupts();             // enable all interrupts

  for (int controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
     controllers[controllerIndex].trainController.setPWMFrequency ();
  }

  #if DEBUG
  if (cancmd_present == FALSE)
    Serial.print("Standalone ");
  Serial.println("CANCMDDC started.");
  Serial.println("=====================================");
  #endif
  
  // wait for any CANCABs to finish initialising themselves
  delay(2000);

  // two short beeps
  nBeeps(2, 50);
}

/******************************************************************************************************
 * Arduino loop routine. Essentially look for CBUS messages
 * and dispatch them to the appropriate handlers
 */
void loop()
{
  int id;
  long unsigned int dcc_address;
  int long_address;
  int controllerIndex;
  int lastInBufPtr = 0;
  int lastOutBufPtr = 0;
  messageRecordType nextMessage;
/*
#if OLED_DISPLAY
  if (MessageBuffer.bufferOverflow == 1)
  {
    display.setCursor(64, 24);
    display.println("Overflow!");
    MessageBuffer.bufferOverflow = 0;
  }
#endif
*/


    nextMessage = fifoMessageBuffer.getMessage();

    if(nextMessage.len > 0)            // Check to see whether data is received.
    {
 #if DEBUG
          Serial.print("CAN msg: ID ");
          Serial.print(nextMessage.canId, HEX);
          Serial.print(" OpCode:");
          for(int i = 0; i<nextMessage.len; i++)                // Print each byte of the data
          {
            if (i == 1)
            {
              Serial.print(" Data:");
            }
            Serial.print(" ");
            if(nextMessage.rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
            {
              Serial.print("0");
            }
            Serial.print(nextMessage.rxBuf[i], HEX);
          }
          Serial.println();
#endif

      // Perform the action for the received message
      switch (nextMessage.rxBuf[0])
      {

        case 0x07:                              // System Reset (Sent by CANCMD on power up)
          cancmd_present = TRUE; // message came from CANCMD, so must be present
          for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
          {
            // release all active controllers
            if (controllers[controllerIndex].session != SF_INACTIVE)
            {
              controllers[controllerIndex].trainController.emergencyStop(); // Emergency Stop
              controllers[controllerIndex].session = SF_INACTIVE;
              // update the speed display.
              displaySpeed(controllerIndex);
            }
          }
          break;
          
        // -------------------------------------------------------------------
        case 0x21:                              // Release loco command
          releaseLoco(nextMessage.rxBuf[1]);
          break;

        // -------------------------------------------------------------------
        case 0x22:                              // Query loco command
          queryLoco(nextMessage.rxBuf[1]);
          break;

        // -------------------------------------------------------------------
        case 0x23:                              // CAB Session keep alive command
          keepaliveSession(nextMessage.rxBuf[1]);
          break;
          
       // -------------------------------------------------------------------
       case 0x40:                              // Request loco session (RLOC)
           dcc_address = nextMessage.rxBuf[2] + ((nextMessage.rxBuf[1] & 0x3f) << 8);
           long_address = (nextMessage.rxBuf[1] & SF_LONG);
 #if DEBUG
           Serial.print("Req Sess. ");
           if (long_address == 0)
           {
             Serial.print("Short");
           }
           else
           {
             Serial.print("Long");
           }
           Serial.print(" Addr: ");
           Serial.println(dcc_address);
 #endif
           if (getDCCIndex (dcc_address, long_address) != SF_UNHANDLED)
           {
             locoRequest(dcc_address, long_address, 0);
           }
           break;
           
        // -------------------------------------------------------------------
        case 0x44:                              // Set Speed Step Range
          setSpeedSteps(nextMessage.rxBuf[1],nextMessage.rxBuf[2]);
          break;
          
       // -------------------------------------------------------------------
        case 0x47:                              // Session speed and direction
          controllerIndex = getSessionIndex(nextMessage.rxBuf[1]);
          if (controllerIndex >= 0)
          {
            int requestedSpeed = nextMessage.rxBuf[2] & 0x7f;
           if (requestedSpeed == 1)
            {
              // emergency stop
              controllers[controllerIndex].trainController.emergencyStop();
            }
            else
            {
              controllers[controllerIndex].trainController.setSpeedAndDirection(nextMessage.rxBuf[2] & 0x80, requestedSpeed);
            }
            // update the speed display.
            displaySpeed(controllerIndex);
            // reset the timeout
            keepaliveSession(nextMessage.rxBuf[1]);
          }
          break;
         
       // -------------------------------------------------------------------
        case 0x61:                              // Request Steal or Share loco session (GLOC)
           dcc_address = nextMessage.rxBuf[2] + ((nextMessage.rxBuf[1] & 0x3f) << 8);
           long_address = (nextMessage.rxBuf[1] & SF_LONG);
           locoRequest(dcc_address, long_address, nextMessage.rxBuf[3]);
           break;
          
        // -------------------------------------------------------------------
        case 0xE1:                              // PLOC session Allocate from CANCMD
           cancmd_present = TRUE; // message came from CANCMD, so must be present
           dcc_address = nextMessage.rxBuf[3] + ((nextMessage.rxBuf[2] & 0x3f) << 8);
           long_address = (nextMessage.rxBuf[2] & SF_LONG);
 #if DEBUG
           Serial.print("PLOC from CANCMD. ");
           if (long_address == 0)
           {
             Serial.print("Short");
           }
           else
           {
             Serial.print("Long");
           }
           Serial.print(" Addr: ");
           Serial.println(dcc_address);
 #endif
           // only interested in the addresses of our analogue outputs
           for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
           {
             if (controllers[controllerIndex].DCCAddress == dcc_address  && controllers[controllerIndex].longAddress == long_address)
             {
               int requestedSpeed = nextMessage.rxBuf[4] & 0x7f;
               controllers[controllerIndex].session = nextMessage.rxBuf[1];
               if (requestedSpeed = 1)
               {
                // emergency stop
                controllers[controllerIndex].trainController.emergencyStop();
               }
               else
               {
                 controllers[controllerIndex].trainController.setSpeedAndDirection((nextMessage.rxBuf[4] & 0x80) >> 3, nextMessage.rxBuf[4] & 0x7f);
               }
             }
           }
          break;
          
        // -------------------------------------------------------------------
        case 0x0A:                              // Emergency stop all
          for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
          {
            // stop all active controllers
            if (controllers[controllerIndex].session != SF_INACTIVE)
            {
              controllers[controllerIndex].trainController.emergencyStop ();
              // update the speed display.
              displaySpeed(controllerIndex);
            }
          }
          // Tell all the CABs and Throttles
          emergencyStopAll();
          break;

        // -------------------------------------------------------------------
        default:
          // ignore any other CBus messages
          break;
      }
    }
  else
  {
    // No CBus message received this time round the loop, so check the sessions for timeout
    for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
    {
      if ((controllers[controllerIndex].session != SF_INACTIVE) && (controllers[controllerIndex].timeout > MAXTIMEOUT))
      {
#if DEBUG
  Serial.print("Session ");
  Serial.print(controllers[controllerIndex].session);
  Serial.print(" Address ");
  Serial.print(controllers[controllerIndex].DCCAddress);
  Serial.println(" Timed Out.");
#endif
        controllers[controllerIndex].trainController.setSpeedAndDirection(0, 0);
        releaseLoco(controllers[controllerIndex].session);
        sendSessionError(controllers[controllerIndex].session, 8); // Send session cancelled message out to CABs
      }

      if (updateNow == 1)
      {
        controllers[controllerIndex].trainController.matchToTargets ();
        // update the speed display.
        displaySpeed(controllerIndex);
      }
    }
    updateNow = 0;
  }  
}

/* *******************************************************************************
 * functions and procedures
 * *******************************************************************************/
int getSessionIndex (int session)
{
  int controllerIndex;
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

int getDCCIndex (long unsigned int dcc_address, int long_address)
{
  int controllerIndex;
  for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    if ((controllers[controllerIndex].DCCAddress == dcc_address) && (controllers[controllerIndex].longAddress == long_address))
    {
      return controllerIndex;
    }
  }
  // controller not found, for this DCC address so
  return SF_INACTIVE;
}


/*
 * A loco release command for the given session
 */
void
releaseLoco(int session)
{
  int controllerIndex = getSessionIndex(session);
  if (controllerIndex >= 0)
  {
    controllers[controllerIndex].session = SF_INACTIVE;
    controllers[controllerIndex].timeout = 0;
#if DEBUG
  Serial.print("Session ");
  Serial.print(session);
  Serial.print(" Address ");
  Serial.print(controllers[controllerIndex].DCCAddress);
  Serial.println(" Released.");
#endif
    // update the speed display.
    displaySpeed(controllerIndex);
  }
}

/*
 * A QLOC command for the given session from a CAB
 */
void
queryLoco(int session)
{  
  int controllerIndex = getSessionIndex(session);
  // only respond if working standalone
  if (cancmd_present == FALSE)
  {
#if DEBUG
    Serial.print("Query Loco Session ");
    Serial.println(session);
#endif
    if (controllerIndex >= 0)
    {
      // session found, send a PLOC
      sendPLOC(session);
    }
    else
    {
      // session not active, so send back error
      sendSessionError(session, 3); // 3 - Session not present
    }
  }
}

/*
 * The command station has allocated a session to a locomotive
 */
void
locoSession(int session, long unsigned int address, int long_address, int direction, int speed)
{
  int controllerIndex = getDCCIndex (address, long_address);
   #if DEBUG
     Serial.print("locoSession ");
     Serial.print(session);
     if (long_address == 0)
     {
       Serial.print(" Short");
     }
     else
     {
       Serial.print(" Long");
     }
     Serial.print(" DCC address ");
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
void keepaliveSession(int session)
{
  int controllerIndex = getSessionIndex(session);
  if (controllerIndex >= 0)
  {
    controllers[controllerIndex].timeout = 0;
  }
}


/*
 * A throttle has requested access to a particular loco address
 * This routine is only used if there is no CANCMD on the bus that will
 * allocate sessions.
 */
void
locoRequest(long unsigned int address, int long_address, int flags)
{
  int controllerIndex = getDCCIndex(address, long_address);
#if DEBUG
  Serial.println("locoRequest");
#endif
  // only respond if working standalone
  if (cancmd_present == FALSE)
  {
#if DEBUG
    Serial.println("Standalone");
#endif
    if (controllerIndex >= 0)
    {
      // This is a DCC Address associated with our controllers
      if (controllers[controllerIndex].session != SF_INACTIVE)
      {
        // Loco is already used in a session
#if DEBUG
        Serial.print("Loco already allocated to session ");
        Serial.println(controllers[controllerIndex].session);
#endif
        if (flags == 0)
          sendError(address, long_address, 2);    // Send a Taken error
        else if (flags == 1)        // Steal
        {
          sendError(address, long_address, 8);
          controllers[controllerIndex].session = SF_INACTIVE;
        }
        else if (flags == 2)        // Share
        {
          sendPLOC(controllers[controllerIndex].session);
        }
        else
          sendError(address, long_address, 7);    // Invalid request - unknown flags value
        return;
      }
    }
    else
    {
      // This DCC Address is not associated with any of our controllers
      sendError(address, long_address, 7);    // Invalid request
      return;
    }
    // If we have got this far then the controller is not in use
    // set a new session number for the controller to use
   
    locoSession(controllerIndex, address, long_address, SF_FORWARDS, 0);
 #if DEBUG
        Serial.print("Session Allocated: ");
        Serial.println(controllers[controllerIndex].session);
 #endif
    sendPLOC(controllers[controllerIndex].session);
  }
  // Do nothing if there is a CANCMD present - it will assign sessions.
}


/**
 * Send a PLOC message in response to a CAB requesting a session for
 * a DCC address
 */
void
sendPLOC(int session)
{
  unsigned char buf[8];
  int controllerIndex = getSessionIndex(session);
  // only send this response if working standalone
  if (cancmd_present == FALSE)
  {
   #if DEBUG
     Serial.print("Send PLOC ");
   #endif
    buf[0] = 0xE1; // OPC_PLOC
    buf[1] = session;
    buf[2] = ((controllers[controllerIndex].DCCAddress >> 8) & 0x3f) | (controllers[controllerIndex].longAddress);
    buf[3] = (controllers[controllerIndex].DCCAddress) & 0xff;
    buf[4] = controllers[controllerIndex].trainController.getSpeed() | (controllers[controllerIndex].trainController.getDirection() ? 0x80 : 0);
    buf[5] = 0;  // Zero function bytes
    buf[6] = 0;
    buf[7] = 0;
    CAN0.sendMsgBuf(0, 0, 8, buf);
   #if DEBUG
          Serial.print("CAN msg: ");
          for(int i = 0; i<8; i++)                // Print each byte of the data
          {
            Serial.print(" ");
            if(buf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
            {
              Serial.print("0");
            }
            Serial.print(buf[i], HEX);
          }
          Serial.println();
   #endif
  }
}

/**
 * Send an error packet labelled with the DCC address
 */
void
sendError(long unsigned int address, int long_address, int code)
{
  unsigned char buf[4];
#if DEBUG
  Serial.print("Send Loco ");
  Serial.print(address);
  Serial.print(" Error ");
  Serial.println(code);
#endif
  buf[0] = 0x63; // OPC_ERR
  buf[1] = ((address >> 8) & 0xff) | long_address;
  buf[2] = address & 0xff;
  buf[3] = code;
  CAN0.sendMsgBuf(0, 0, 4, buf);
}

/**
 * Send a session error message to the CABs, labelled with the session number
 */
void
sendSessionError(int session, int code)
{
  unsigned char buf[4];
#if DEBUG
  Serial.print("Send Session ");
  Serial.print(session);
  Serial.print(" Error ");
  Serial.println(code);
#endif
  buf[0] = 0x63; // OPC_ERR
  buf[1] = session;
  buf[2] = 0;
  buf[3] = code;
  CAN0.sendMsgBuf(0, 0, 4, buf);
}

/**
 * Send a reset signal to all connected CABs
 */
void
sendReset()
{
  unsigned char buf[1];
  int i;
  buf[0] = 0x07; // OPC_ARST
  for (i=0; i<5; i++)
  {
   CAN0.sendMsgBuf(0, 0, 1, buf);
  }
  buf[0] = 0x03; // OPC_BON
  CAN0.sendMsgBuf(0, 0, 1, buf);
}

void
setSpeedSteps(int session, int steps)
{
  // This is only relevent for DCC, so can be ignored
}

/**
 * Stop all DC tracks
 * Loop over every session and if it is not free set
 * the speed to 0
 */
void
emergencyStopAll()
{
  unsigned char buf[1];
  // Tell all the cabs
  buf[0] = 0x06; // ESTOP
  CAN0.sendMsgBuf(0, 0, 1, buf);
  beep_counter = 100; // sound buzzer 1 second
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
    for (int controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
    {
      if (controllers[controllerIndex].session >= 0)
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
    updateNow = 1;
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

// External interrupt routine to get a CBus message and add it to the FIFO buffer
void
receiveCBusMessage()
{
  messageRecordType newMessage;
  noInterrupts();
  digitalWrite(LED, ON);
  flash_counter = 1;
  if(CAN_MSGAVAIL == CAN0.checkReceive())            // Check to see whether data is received.
  {
    // Get the message received from the CBus
    CAN0.readMsgBuf(&newMessage.len, newMessage.rxBuf);  // Read data: len = data length, buf = data byte(s)
    newMessage.canId = CAN0.getCanId();                  // and get the canId
    if (fifoMessageBuffer.addMessage(newMessage))
    {
      // Buffer overflow
      beep_counter = 100; // sound buzzer
    }
  }
  interrupts();
}

// N beeps
void nBeeps (int numBeeps,
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
void
setupDisplay()
{
  // we will display controllers in 2 columns on 3 lines (max 6 controllers)
#if OLED_DISPLAY
  int line; // 0-2
  int column; // 0 or 1
  int x_pos;
  int y_pos;
  float barsize;
  
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);

  for (int controllerIndex=0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    line = controllerIndex / 2;
    column = controllerIndex - (line * 2);
    x_pos = column*67;
    y_pos = line*12;
    display.setCursor(x_pos, y_pos);
    display.print(controllers[controllerIndex].DCCAddress); // display DCC address
    display.setCursor((x_pos)+30, y_pos);
    display.println("Free");
  }

  display.display();

#endif
}  

// Display the current speed settings
void
displaySpeed(int controllerIndex)
{
  // we will display controllers in 2 columns on 3 lines (max 6 controllers)
#if OLED_DISPLAY
  int line; // 0-2
  int column; // 0 or 1
  int x_pos;
  int y_pos;
  float barsize;
  
    line = controllerIndex / 2;
    column = controllerIndex - (line * 2);
    x_pos = column*67;
    y_pos = line*12;
    if (controllers[controllerIndex].session != SF_INACTIVE)
    {
      barsize = (controllers[controllerIndex].trainController.getSpeed()*30)/128;
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
#endif
}


void
displayImage(const uint8_t *imageBitmap)
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

void
displayVersion()
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
  delay(1500);
#endif
}

void
initialiseDisplay()
{
#if OLED_DISPLAY  
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // Clear the buffer.
  display.clearDisplay();

#endif
}


