/******************************************************************
  @file       viewMSPRequest.ino
  @brief      View MultiWii Serial Protocol packets using the Arduino
              IDE terminal.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           22/02/22

  This example, allows you to see MSP messages in the Arduino Serial Monitor.
  Choose "newLine" from pulldown menu in the Serial Monitor.

  This sketch demonstrates the three types of MSP messages that can be sent:

   - Command: a message sent to the flight controller which has some 
     information to be sent.
   - Request: a message sent to the flight controller asking for some 
     information to be returned.
   - Response: a message sent by the flight controller with information 
     responding to a request.

******************************************************************/

#include <NexgenMSP.h>

#define MAX_CMD_SIZE    32
#define RETURN    '\r'
#define NEW_LINE  '\n'
#define NULL_CHAR '\0'

NexgenMSP msp;

char cmdString[MAX_CMD_SIZE];

void commandFromSerial(char *cmdString) {
    int index = 0;

    while (Serial.available()) {
        char letter = (char)Serial.read();

        if (letter == NEW_LINE || letter == RETURN) {
            //  NL or RTN character - discard
        }
        else if (index < MAX_CMD_SIZE - 1) {
          cmdString[index] = letter;
          index++;
        }
        else {
            Serial.println("ERROR - command has too many characters");
        }
    }

    // C strings are terminated with the null character 
    cmdString[index] = NULL_CHAR;  
}

void printInstructions() {
  Serial.println("******************************************************************");
  Serial.println("                 Nexgen MSP - View Request");
  Serial.println("******************************************************************\n");
  Serial.println("This example, allows you to see MSP messages in the Serial Monitor.");
  Serial.println("Select 115200 baud & NewLine from the pulldown menus in the Monitor.");
  Serial.println("Enter the message ID name (e.g., MSP_API_VERSION) and press enter");
  Serial.println("or Send to see that message displayed. Valid message ID's may be");
  Serial.println("found in the Protocol.h file.");
  Serial.println("  < - denotes going to the flight controller (command and request).");
  Serial.println("  > - denotes coming from the flight controller (response)."); 
  Serial.println("******************************************************************\n");
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
  // load the cmdString array with command input from IDE Terminal
  commandFromSerial(cmdString);

  //  Parse command and send MSP message to Serial if valid
  if (cmdString[0] != NULL_CHAR) {
    switch(idLookup(cmdString)) {
        case MSP_API_VERSION:
          msp.send(MSP_API_VERSION, NULL, 0);
          Serial.println();
          break;
        case MSP_IDENT:
          Serial.print("MSP_IDENT = ");
          msp.send(MSP_IDENT, NULL, 0);
          Serial.println(" = $M<11001001100100");
          break;
        case BAD_ID:
          Serial.print("Command String not recognized: ");
          Serial.println(cmdString);
          break;
    }
  }

  delay(500);
}