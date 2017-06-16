
/**
 * Copyright (c) 2017 Ian Morgan
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 3 of the License
 */

// Analogue (PWM) Train Controller.
//
// Class: trainControllerClass
//
// Methods:
//                       trainControllerClass(int setPinA, int setPinB, int setPinPWM)            Class Constructor
//      void    matchToTargets ()
//      void    emergencyStop ()
//      void    setSpeedAndDirection (int newLocoDirection, int newLocoSpeed)
//      void    setSpeed (int newLocoSpeed)
//      uint8_t getSpeed ()
//      uint8_t getDirection ()
//      void    setPWMFrequency ()

#define SF_FORWARDS    0x01      // Train is running forwards
#define SF_REVERSE     0x00      // Train is running in reverse
#define PWM_FREQUENCY  30000     // PWM frequency (in Hz)
#define SPEED_STEP     15
#define DEBUG          1         // set to 0 for no debug messages, 1 for messages to console

class trainControllerClass
{
  private: 
  uint8_t  currentLocoSpeed = 0;
  uint8_t  currentLocoDirection = SF_FORWARDS;
  uint8_t  targetLocoSpeed = 0;
  uint8_t  targetLocoDirection = SF_FORWARDS;
  int      pinA;
  int      pinB;
  int      pinPWM;
  
  // -------------------------------------------------
  
  public:

  // -------------------------------------------------
  
  // Constructor - initializes the member variables and state
  trainControllerClass(int setPinA,
                       int setPinB,
                       int setPinPWM)
  {
      pinA = setPinA;
      pinB = setPinB;
      pinPWM = setPinPWM;
      pinMode(pinPWM, OUTPUT);
      analogWrite(pinPWM, 0);
      pinMode(pinA, OUTPUT);
      digitalWrite(pinA, LOW);
      pinMode(pinB, OUTPUT);
      digitalWrite(pinB, LOW);
  }

  // -------------------------------------------------
  
  private:

  // -------------------------------------------------
  
  void setControllerTargets (int newLocoDirection, int newLocoSpeed)
  {
    // Just set the target speed and direction. A timer interrupt will be responsible for changing actuals to match.
#if DEBUG
  Serial.print("setControllerTargets: ");
  if (newLocoDirection)
    Serial.print(" forwards ");
  else
    Serial.print(" reverse ");
  Serial.print(" speed ");
  Serial.println(newLocoSpeed);
#endif

    targetLocoSpeed = newLocoSpeed;
    targetLocoDirection = newLocoDirection;
  }

  
  // -------------------------------------------------
  
  public:

  // -------------------------------------------------
  
  void matchToTargets ()
  {
    // only do anything if speed or direction does not match
    if ((targetLocoSpeed != currentLocoSpeed) || (targetLocoDirection != currentLocoDirection))
    {
      if (targetLocoDirection != currentLocoDirection)
      {
        // need to decellerate to zero first
        if (currentLocoSpeed < SPEED_STEP)
        {
          currentLocoSpeed = 0;
          currentLocoDirection = targetLocoDirection;
          analogWrite(pinPWM, 0);
          if (currentLocoDirection == SF_REVERSE)
          {
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, HIGH);
          }
          else
          {
            digitalWrite(pinB, LOW);
            digitalWrite(pinA, HIGH);
          }
        }
        else
        {
          currentLocoSpeed = currentLocoSpeed - SPEED_STEP;
        }
      }
      else
      {
        // we are going the right direction
        if (currentLocoSpeed > targetLocoSpeed)
        {
          // need to decellerate
          if (targetLocoSpeed + SPEED_STEP > currentLocoSpeed)
          {
            currentLocoSpeed = targetLocoSpeed;
          }
          else
          {
            currentLocoSpeed = currentLocoSpeed - SPEED_STEP;
          }
        }
        else
        {
          // need to accellerate
          if (currentLocoSpeed + SPEED_STEP > targetLocoSpeed)
          {
            currentLocoSpeed = targetLocoSpeed;
          }
          else
          {
            currentLocoSpeed = currentLocoSpeed + SPEED_STEP;
          }
        }
      }
      analogWrite(pinPWM, currentLocoSpeed << 1);
    }
  }

  // -------------------------------------------------
  
  void emergencyStop ()
  {
    noInterrupts();
    analogWrite(pinPWM, 0);
    targetLocoSpeed = 0;
    currentLocoSpeed = 0;
    interrupts();
  }
  
  // -------------------------------------------------
  
  void setSpeedAndDirection (int newLocoDirection, int newLocoSpeed)
  {
    setControllerTargets (newLocoDirection, newLocoSpeed);
  }

  // -------------------------------------------------
  
  void setSpeed (int newLocoSpeed)
  {
    setControllerTargets (currentLocoDirection, newLocoSpeed);
  }

  // -------------------------------------------------
  
  uint8_t getSpeed ()
  {
    return currentLocoSpeed;
  }

  // -------------------------------------------------
  
  uint8_t getDirection ()
  {
    return currentLocoDirection;
  }

  // -------------------------------------------------
  
  void setPWMFrequency ()
  {
      //set the frequency for the specified pin
      // This needs to be in the setup routine after calling InitTimersSafe();
      SetPinFrequencySafe(pinPWM, PWM_FREQUENCY);
  }
};

