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

  How to Use:

  Upload the mspResponse.ino sketch to an Arduino board and run the 
  Protocol Tester on your PC. In this example, the Protocol Tester (PT) 
  is simulating the Configurator and our sketch is emulating the flight 
  controller. 

  In the PT, select the serial port that your Arduino is connected to 
  and click on the CONNECT button.

  Select a request from the drop down (e.g., MSP_STATUS) and click on 
  SEND to transmit the message to the Arduino. The message sent will 
  be displayed in the log console as will any response or error. 
  The response will also be displayed in the hex dump at the bottom 
  of the PT screen.

  In order for there to be a response, other than an error, it has 
  to be handled in the mspResponse sketch.

******************************************************************/

#include <NexgenMSP.h>

NexgenMSP msp;

msp_api_version_t api;
msp_ident_t identReply;
msp_packet_t packet;
msp_fc_variant_t variant;

void setup() {
  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  //  Allocate stream and timeout (default timeout = 500)
  msp.begin(Serial);

  //  Original MSP message response
  identReply.multiWiiVersion = 0;
  identReply.multiType = QUADX;
  identReply.mspVersion = MSP_PROTOCOL_VERSION;
  identReply.capability = MSP_FEATURE_VBAT;

  //  Betaflight message variant
  strcpy(variant.flightControlIdentifier, NEXGEN_IDENTIFIER);
}

void loop() {
  // Poll for valid messageID
  // recvSize can be NULL
  if (msp.recv(&packet.recvMessageID, packet.payload, packet.maxSize, &packet.recvSize)) {
    switch(packet.recvMessageID) {
      case MSP_IDENT:
        msp.response(MSP_IDENT, &identReply, sizeof(identReply));
        break;
      case MSP_FC_VARIANT:
        msp.response(MSP_FC_VARIANT, &variant, sizeof(variant));
        break; 
      default:
        msp.error(packet.recvMessageID, NULL, 0);
        break;
    }
  }

  delay(100);

}