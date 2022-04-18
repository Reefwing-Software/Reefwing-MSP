/******************************************************************
  @file       viewMSPResponse.ino
  @brief      Testing the MultiWii Serial Protocol using the Arduino
              IDE Serial Monitor. 
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           22/02/22

  This example, allows you to see MSP response in the Arduino Serial Monitor.
  Choose "newLine" and 115200 baud from pulldown menus in the Serial Monitor.

  There are three types of MSP messages that can be sent:

   - Command: a message sent to the flight controller which has some 
     information to be sent.
   - Request: a message sent to the flight controller asking for some 
     information to be returned.
   - Response: a message sent by the flight controller with information 
     responding to a request.

  This sketch demonstrates MSP responses. These messages would
  usually be sent by the Flight Controller or equivalent. The viewMSPRequest.ino
  sketch demonstrates requests and commands from the Configurator.

  HOW TO USE:

  Upload the sketch to an Arduino board and open the Serial Monitor.

******************************************************************/

#include <NexgenMSP.h>

NexgenMSP msp;
msp_rc_t rc;
msp_api_version_t api;
msp_packet_t packet;

void printInstructions() {
  Serial.println("******************************************************************");
  Serial.println("                 Nexgen MSP - View Response");
  Serial.println("******************************************************************\n");
  Serial.println("In this example, the Serial Monitor is simulating the Configurator");
  Serial.println("and our sketch is emulating the flight controller. Select one of the");
  Serial.println("pre-configured MSP commands to send to the Arduino Board by copying");
  Serial.println("a message below starting with $M, and pasting it into the command box");
  Serial.println("then press <RETURN> or Send.\n");
  Serial.println("1. MSP_IDENT Request: $M<0000000011001001100100");
}

void setup() {
  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  //  Allocate stream and timeout (default timeout = 0, i.e., no timeout)
  msp.begin(Serial);
  printInstructions();
}

void loop() {
  // Poll for valid messageID
  // recvSize can be NULL
  recv(&packet.recvMessageID, packet.payload, packet.maxSize, (packet.recvSize ? packet.recvSize : &packet.recvSizeValue));

  switch(recvMessageID) {
    case MSP_IDENT:
      Serial.println("MSP_IDENT Received.")
      break;
    default:
      break;
  }

}