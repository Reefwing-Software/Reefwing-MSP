/******************************************************************
  @file       terminalTest.ino
  @brief      Testing the MultiWii Serial Protocol using the Arduino
              IDE terminal.
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
msp_api_version_t api;

void printInstructions() {
  Serial.println("******************************************************************");
  Serial.println("                 Nexgen MSP Terminal Test");
  Serial.println("******************************************************************\n");
  Serial.println("In this example, the Arduino Terminal is simulating the Configurator");
  Serial.println("and this sketch is emulating the flight controller. Select one of the");
  Serial.println("pre-configured MSP commands to send to the Arduino Board by typing");
  Serial.println("the number next to the command and then press RETURN.\n")
  Serial.println("1. MSP API VERSION");
}



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