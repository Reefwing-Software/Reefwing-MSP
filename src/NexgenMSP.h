/******************************************************************
  @file       NexgenMSP.h
  @brief      A light weight Arduino implementation of the MultiWii Serial Protocol.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.3
  Date:        02/06/22

  1.0.0 Original Release.           22/02/22
  1.0.3 IMU ODR & offset bias added 02/06/22

  Credit - Version 2.4 of the MultiWii Protocol class.
           ref: https://github.com/xdu-aero-association/MultiWii_2_4/blob/master/MultiWii/Protocol.cpp
         - Arduino library for MSP by Fabrizio Di Vittorio
           ref: https://github.com/fdivitto/MSP
         - Version 1.44 of the Betaflight MSP Protocol
           ref: https://github.com/betaflight/betaflight/tree/master/src/main/msp

******************************************************************/

#ifndef NexgenMSP_h
#define NexgenMSP_h

#include <Arduino.h>
#include <Stream.h>

#include "Protocol.h"

/******************************************************************
 *
 * NexgenMSP Class - 
 * 
 ******************************************************************/

class NexgenMSP {
  public:
    void begin(Stream & stream, uint32_t timeout = 500);
    void send(uint8_t messageID, void * payload, uint8_t size);
    void error(uint8_t messageID, void * payload, uint8_t size);
    void response(uint8_t messageID, void * payload, uint8_t size);
    bool recv(uint8_t * messageID, void * payload, uint8_t maxSize, uint8_t * recvSize);    
    bool waitFor(uint8_t messageID, void * payload, uint8_t maxSize, uint8_t * recvSize = NULL);
    bool request(uint8_t messageID, void * payload, uint8_t maxSize, uint8_t * recvSize = NULL);
    bool command(uint8_t messageID, void * payload, uint8_t size, bool waitACK = true);
    void reset();
    bool getActiveModes(uint32_t * activeModes);

  private:
    Stream * _stream;
    uint32_t _timeout;
    
};

#endif