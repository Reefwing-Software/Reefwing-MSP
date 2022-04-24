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

  This example, allows you to see an MSP response in the Arduino Serial Monitor.
  Select "Newline" and 115200 baud from the pulldown menus in the Serial Monitor.

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
  In this example, the Serial Monitor is simulating the Configurator
  and our sketch is emulating the flight controller. Select one of the
  pre-configured MSP commands to send to the Arduino Board by copying
  a message below starting with $M, and pasting it into the command box
  then press <RETURN> or Send.

  1. MSP_IDENT Request: $M<0000000011001001100100

******************************************************************/

#include <NexgenMSP.h>

NexgenMSP msp;

msp_api_version_t api;
msp_ident_t identReply;
msp_packet_t packet;

void setup() {
  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  //  Allocate stream and timeout (default timeout = 500)
  msp.begin(Serial);

  identReply.multiWiiVersion = 0;
  identReply.multiType = QUADX;
  identReply.mspVersion = MSP_PROTOCOL_VERSION;
  identReply.capability = MSP_FEATURE_VBAT;

  msp.response(MSP_IDENT, &identReply, sizeof(identReply));
}

void loop() {
  // Poll for valid messageID
  // recvSize can be NULL
  if (msp.recv(&packet.recvMessageID, packet.payload, packet.maxSize, &packet.recvSize)) {
    switch(packet.recvMessageID) {
      case MSP_IDENT:
        msp.response(MSP_IDENT, &identReply, sizeof(identReply));
        break;
      default:
        Serial.print("Unhandled MSG ID: ");
        Serial.println(packet.recvMessageID);
        break;
    }
  }
  else {
    //  Serial.println("No MSP MSG Rx");
  }

  delay(500);

}