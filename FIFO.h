
/**
 * Copyright (c) 2017 Ian Morgan
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 3 of the License
 */

// FIFO buffer for incoming CBus messages.
//
// Class: CBusMessageBufferClass
//
// Methods:
//                       CBusMessageBufferClass()                       Class Constructor
//     int               addMessage (messageRecordType newMessage)
//     messageRecordType getMessage ()
//

#define BUFFERSIZE 128
typedef struct {
    long unsigned int canId; // 32 bits
    unsigned char rxBuf[8];
    byte len; // unsigned 8-bit number
  }messageRecordType;

class CBusMessageBufferClass
{
  // declare local variables as volatile because they will be updated by interrupts
  // so the compiler optimiser should not assume copies of the data in registers will
  // still be valid at a later time
  volatile struct {
                    messageRecordType messageRecord;
                  } messageBuffer[BUFFERSIZE];

  volatile int writePointer;
  volatile int readPointer;

  public:
  // Constructor - initializes the member variables and state
  CBusMessageBufferClass()
  {
    writePointer = 0;
    readPointer = 0;
    messageBuffer[writePointer].messageRecord.canId = 0;
    messageBuffer[writePointer].messageRecord.len   = 0;
  }

  int addMessage (messageRecordType newMessage)
  {
    // this function is uded by the interface interrupt routine to add a new
    // CBus message to the FIFO buffer. If the buffer is full, the oldest
    // entry will be overwritten, and the function returns status '1'.
    int statusReturn = 0;

    // add the message to the buffer
    messageBuffer[writePointer].messageRecord.canId = newMessage.canId;
    messageBuffer[writePointer].messageRecord.len   = newMessage.len;
    for(int i = 0; i<newMessage.len; i++)
    {
      messageBuffer[writePointer].messageRecord.rxBuf[i] = newMessage.rxBuf[i];
    }
    // point to the next buffer location, ready for the next message to come in
    writePointer++;
    if (writePointer == BUFFERSIZE)
    {
      writePointer = 0;
    }
    // Check if the buffer has been over-filled
    if (writePointer == readPointer)
    {
      #if DEBUG
        Serial.println("FIFO Buffer is full");
      #endif
      // oldest message has been overwritten, get ready to return next oldest message
      readPointer++;
      if (readPointer == BUFFERSIZE)
      {
        readPointer = 0;
      }
      statusReturn = 1;
    }
    return statusReturn; // Returns 1 if buffer overflow occurred, otherwise returns 0
  }

// ------------------------------------------------------------------------

  messageRecordType getMessage ()
  {
    // This function is used to return the next unread CBus message from the
    // FIFO buffer, if there is one.
    messageRecordType nextMessage;
	if (writePointer == readPointer)
	{
		// No new message to be removed from buffer
		nextMessage.canId = 0xFFFFFFFF; // invalid;
		nextMessage.len = 0;
		return nextMessage;
	}
    else
    {
      // There is an unread message, so return it to the calling program
      nextMessage.canId = messageBuffer[readPointer].messageRecord.canId;
      nextMessage.len   = messageBuffer[readPointer].messageRecord.len;
      for(int i = 0; i<nextMessage.len; i++)
      {
        nextMessage.rxBuf[i] = messageBuffer[readPointer].messageRecord.rxBuf[i];
      }
      // point to the next message location, ready for the next call
      readPointer++;
      if (readPointer == BUFFERSIZE)
      {
        readPointer = 0;
      }
      return nextMessage;
    }
  }
};


