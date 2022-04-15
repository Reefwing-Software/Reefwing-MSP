/******************************************************************
  @file       multiWiiConfTest.ino
  @brief      Testing the MultiWii Serial Protocol using the MultiWii
              Configurator.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           22/02/22

  This sketch demonstrates the three types of MSP messages that can be sent:

   - Command: a message sent to the flight controller which has some 
     information to be sent.
   - Request: a message sent to the flight controller asking for some 
     information to be returned.
   - Response: a message sent by the flight controller with information 
     responding to a request.

******************************************************************/

#include <NexgenMSP.h>

NexgenMSP msp;

void setup() {
  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  //  Allocate stream and timeout (default timeout = 0, i.e., no timeout)
  msp.begin(Serial);
}

void loop() {
  // 

}