/******************************************************************
  @file       viewMSPRequest.ino
  @brief      View MultiWii Serial Protocol request packets using 
              the Arduino IDE Serial Monitor.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.0.0
  Date:        14/12/22

  1.0.0 Original Release.           22/02/22
  2.0.0 Change Repo and Branding    14/12/22

  This example, allows you to see an MSP request in the Arduino Serial Monitor.
  Select "Newline" and 115200 baud from the pulldown menus in the Serial Monitor.

  There are three types of MSP messages that can be sent:

   - Command: a message sent to the flight controller which has some 
     information to be sent.
   - Request: a message sent to the flight controller asking for some 
     information to be returned.
   - Response: a message sent by the flight controller with information 
     responding to a request.

  This sketch demonstrates requests and commands. These messages would
  usually be sent by the Configurator or equivalent. The viewMSPResponse.ino
  sketch demonstrates expected responses from the Flight Controller.

  HOW TO USE:

  Upload the sketch to an Arduino board and open the Serial Monitor.
  Enter the message ID name (e.g., MSP_IDENT) and press <ENTER>
  or Send to see that message displayed. Valid message ID's may be
  found in the Protocol.h file. Note that message ID's, size and payloads 
  with a byte value less than 33 will be invisible in the Serial Monitor 
  as it displays ASCII and visible characters start at 33 (decimal).

  The third byte of the message after M$ will be:

    < - denotes going to the flight controller (command and request).
    > - denotes coming from the flight controller (response).

******************************************************************/

#include <ReefwingMSP.h>

#define MAX_CMD_SIZE    32
#define RETURN    '\r'
#define NEW_LINE  '\n'
#define NULL_CHAR '\0'

ReefwingMSP msp;

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
  Serial.println("                 Reefwing MSP - View Request");
  Serial.println("******************************************************************\n");
  Serial.println("This sketch allows you to see MSP requests in the Serial Monitor.");
  Serial.println("Select 115200 baud & NewLine from the pulldown menus in Monitor.");
  Serial.println("Enter the message ID name (e.g., MSP_IDENT) and press <enter>");
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
    uint8_t id = idLookup(cmdString);
    
    switch(id) {
        case MSP_IDENT:
          Serial.print("Request MSP_IDENT = ");
          msp.send(MSP_IDENT, NULL, 0);
          Serial.println(" = $M<\\0dd");
          break;
        case BAD_ID:
          Serial.print("BAD ID:: Command String not recognized: ");
          Serial.println(cmdString);
          Serial.println("Note that not all ID's are in the lookup table in Protocol.h");
          break;
        default:
          Serial.print(cmdString);
          Serial.print(" = ");
          msp.send(id, NULL, 0);
          Serial.println();
          break;
    }
  }

  delay(500);
}