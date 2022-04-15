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

NexgenMSP msp;

char cmd[MAX_CMD_SIZE];

void commandFromSerial(char *cmd) {
    int index = 0;

    while (Serial.available()) {
        char letter = (char)Serial.read();

        if (letter == '\n') {
            //  NL character - discard
        }
        else if (index < MAX_CMD_SIZE - 1) {
          cmd[index] = letter;
          index++;
        }
        else {
            Serial.println("ERROR - command has too many characters");
        }
    }

    // C strings are terminated with the null character 
    cmd[index] = '\0';  
}

void printInstructions() {
  Serial.println("******************************************************************");
  Serial.println("                 Nexgen MSP - View Request");
  Serial.println("******************************************************************\n");
  Serial.println("This example, allows you to see MSP messages in the Serial Monitor.");
  Serial.println("Choose newLine from pulldown menu in the Serial Monitor.")
  Serial.println("Enter the message ID (e.g., MSP_API_VERSION) to see that message");
  Serial.println("displayed. Valid message ID's may be found in the Protocol.h file.");
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
  // load cmd array with command input from IDE Terminal
  commandFromSerial(cmd);

  //  Parse command and send MSP message to Serial if valid
  if (!strcmp(cmd, "")) {
    switch(idFromString(cmd)) {
        case MSP_API_VERSION:
          msp.send(MSP_API_VERSION, NULL, 0);
          break;
        case BAD_ID:
          Serial.println("Command String not recognized.");
          break;
    }
  }
}