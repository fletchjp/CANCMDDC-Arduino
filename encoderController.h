
/**
 * Copyright (c) 2020 David Radcliffe
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 3 of the License
 */

// Analogue (PWM) Train Controller.
//
// Class: encoderControllerClass
//
// Methods:
//                       encoderControllerClass(int setPinA, int setPinB, int setPinS)            Class Constructor

//#define ENCODER_USE_INTERRUPTS
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include "Encoder.h"

class encoderControllerClass
{
  private:
      Encoder enc;
      bool    ignorePush = false;
  // -------------------------------------------------

  public:
      uint8_t       newPos = 0;
      uint8_t       lastPos = 0;
      unsigned long push = 0;
  // -------------------------------------------------

  // Constructor - initializes the member variables and state
  encoderControllerClass (uint8_t setPinA,
                          uint8_t setPinB,
                          uint8_t setPinS)
  {
      enc = Encoder(setPinA, setPinB, setPinS);
  }

  bool read()
  {
      uint8_t val = enc.read();

      if (val == 255) // button pressed
      {
          if (ignorePush) return false;

          if (push == 0) push = millis();
          enc.write(0);
          newPos = 0;
          lastPos = 0;
          return true;
      }
      else // button released
      {
          ignorePush = false;
          push = 0;
      }

      if (val == 1) // EStop value - not needed
      {
          if (lastPos == 0)
          {
              enc.write(2);
              val = 2;
          }
          else // == 2
          {
              enc.write(0);
              val = 0;
          }
      }

      newPos = val;
      return false;
  }

  void write(uint8_t value)
  {
      enc.write(value);
      newPos = value;
      lastPos = value;
      push = 0;
      ignorePush = true;
  }

  // -------------------------------------------------

  private:

  // -------------------------------------------------
  // -------------------------------------------------
};


