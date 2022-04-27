/******************************************************************
 MultiWii Serial Protocol Tester
 
 @file    protocolTester.pde
 @brief   For sending and displaying MSP messages
 @author  David Such
 
 Code:        David Such
 Version:     1.0
 Date:        19/04/2022
 
 1.0  Original Release              19/04/22
 
 ******************************************************************
 
 Notes:
 
 1. You will need to select the serial port that your drone/Arduino 
    board is connected to.
 2. Requires installation of the controlP5 library v2.2.6
 
 ******************************************************************/
 
 import processing.serial.*;
 import controlP5.*;
 import java.util.*;  //  Required for List type
 
 /******************************************************************
 CONSTANTS - COLOUR PALETTE
 ******************************************************************/

final color cp5black = color(0, 0, 0);
final color cp5white = color(255, 255, 255);
final color cp5red = color(255, 0, 0);
final color cp5green = color(0, 255, 0);
final color cp5orange = color(255, 165, 0);
final color cp5yellow = color(255, 255, 0);
final color cp5blue = color(0, 44, 91);
final color cp5cyan = color(0, 255, 255);
final color cp5magenta = color(255, 0, 255);
final color cp5grey = color(120);
final color cp5brown = color(165, 42, 42);

final color lightBlue = color(0, 0, 255);
final color darkGrey = color(43, 45, 47);
final color darkGreen = color(21, 71, 52);
final color nexgenGold = color(203, 197, 150);
final color nexgenGreen = color(171, 253, 4);
final color nexgenAqua = color(0, 255, 196);
 
 /******************************************************************
 CONSTANTS - GENERAL
 ******************************************************************/

final float VERSION = 1.0;
final int BAUD_RATE = 115200;
final int CLEAR = 99;
final int BLACK = 0;
final int WHITE = 255;
final int X_AXIS = 1;
final int Y_AXIS = 2;
final int HEX_DUMP_ROWS = 8;
final int HEX_DUMP_COLS = 16;

/******************************************************************
 CONTROL P5 OBJECTS
 ******************************************************************/
 
 ControlP5 cp5mainMenu, cp5console;
 Println console;
 Button discButton;
 
 /******************************************************************
 GLOBAL VARIABLES
 ******************************************************************/

Serial serialPort = null;
String portName = null;
ScrollableList serialPortsList, mspRequestList;
Textarea consoleText;
String serialString = null;
PFont bold, smallBold, regular, fixedWidthBold;
CommandID mspRequest = CommandID.MSP_IDENT;
int hexRow = 0, hexCol = 0, byteCtr = 0, baseBytes = 0;
int[][] hexDump = new int[HEX_DUMP_ROWS][HEX_DUMP_COLS];
char[][] asciiDump = new char[HEX_DUMP_ROWS][HEX_DUMP_COLS];

/******************************************************************
 MSP PAYLOAD VARIABLES
 ******************************************************************/
 
 //  MSP_IDENT:
 
 int version;
 int multiType;  // 1 for tricopter, 2 for quad+, 3 for quadX, ...
 int multiCapability = 0; // Bitflags stating what capabilities are/are not present in the compiled code.
 
 //  MSP_STATUS:
 int cycleTime, i2cError, mode, present = 0, configSetting;
 
 //  MSP_RAW_IMU:
 float gx, gy, gz, ax, ay, az, mx, my, mz;

/******************************************************************
 SETUP
 ******************************************************************/

void setup() 
{
  /******************************************************************
   Application Screens
   ******************************************************************/

  cp5mainMenu     = new ControlP5(this);
  cp5console      = new ControlP5(this);
  
  /******************************************************************
   Font Management
   ******************************************************************/

  bold = createFont("Arial-BoldMT", 24);
  smallBold = createFont("Arial-BoldMT", 14);
  regular = createFont("Lucida Sans Regular", 14);
  ControlFont cf1 = new ControlFont(createFont("Arial", 16));
  fixedWidthBold = createFont("CourierNewPS-BoldMT", 14);

  cp5mainMenu.setFont(cf1);
  cp5console.setFont(cf1);
  
  /******************************************************************
   HEX DUMP
   ******************************************************************/
  
  for (int i = 0; i < HEX_DUMP_ROWS; i++) 
    for (int j = 0; j < HEX_DUMP_COLS; j++) {
      hexDump[i][j] = 0;
      asciiDump[i][j] = '.';
    }
  
  /******************************************************************
   CP5 - Create Console
   ******************************************************************/
   
  cp5console.enableShortcuts();

  cp5console.addButton("clearConsole")
    .setBroadcast(false)
    .setValue(0)
    .setPosition(width - 350, 320)
    .setSize(70, 30)
    .setId(CLEAR)
    .setCaptionLabel("Clear")
    .setBroadcast(true)
    ;

  cp5console.addButton("copyConsole")
    .setBroadcast(false)
    .setValue(0)
    .setPosition(width - 260, 320)
    .setSize(70, 30)
    .setCaptionLabel("Copy")
    .setColorBackground(darkGrey)
    .setBroadcast(true)
    ;

  cp5console.addButton("refreshSerial")
    .setBroadcast(false)
    .setValue(0)
    .setPosition(width - 350, 60)
    .setSize(90, 30)
    .setCaptionLabel("Refresh")
    .setBroadcast(true)
    ;

  discButton = cp5console.addButton("disconnectSerial")
    .setBroadcast(false)
    .setValue(0)
    .setPosition(width - 150, 60)
    .setSize(120, 30)
    .setCaptionLabel("Disconnect")
    .setColorBackground(darkGrey)
    .setBroadcast(true)
    ;
    
  cp5console.addButton("sendRequest")
    .setBroadcast(false)
    .setValue(0)
    .setPosition(330, 145)
    .setSize(90, 30)
    .setCaptionLabel("Send")
    .setBroadcast(true)
    ;

  cp5console.addTextfield("console")
    .setSize(width - 430, 35)
    .setCaptionLabel("Console")
    .setPosition(40, height - 80)
    .setFont(createFont("arial", 20))
    .setAutoClear(true)
    ;

  consoleText = cp5console.addTextarea("txt")
    .setPosition(width - 350, 100)
    .setSize(320, 200)
    .setFont(createFont("arial", 12))
    .setLineHeight(14)
    .setColorBackground(darkGrey)
    ;

  console = cp5console.addConsole(consoleText);

  String[] portNames = Serial.list();

  serialPortsList = cp5console.addScrollableList("serialPorts")
    .setCaptionLabel("Serial Ports")
    .setPosition(width - 350, 20)
    .setColorValue(nexgenAqua)
    .setColorBackground(darkGrey)
    .setBarHeight(30)
    .setWidth(320)
    .setItemHeight(30)
    .setOpen(false)
    ;

  for (int i = 0; i < portNames.length; i++) 
    serialPortsList.addItem(portNames[i], i);
    
  String[] requestNames = { "MSP_FC_VARIANT", "MSP_IDENT", "MSP_STATUS", "MSP_RAW_IMU",  
                            "MSP_SERVO", "MSP_MOTOR", "MSP_RC",
                            "MSP_RAW_GPS", "MSP_COMP_GPS", "MSP_ATTITUDE",
                            "MSP_ALTITUDE", "MSP_ANALOG", "MSP_RC_TUNING",
                            "MSP_PID", "MSP_BOX", "MSP_MISC", "MSP_MOTOR_PINS",
                            "MSP_BOXNAMES", "MSP_PIDNAMES"
                          };

  mspRequestList = cp5console.addScrollableList("mspRequests")
    .setCaptionLabel("MSP Requests")
    .setPosition(20, 250)
    .setColorValue(nexgenGold)
    .setColorBackground(darkGrey)
    .setBarHeight(30)
    .setWidth(320)
    .setItemHeight(30)
    .setOpen(false)
    ;

  for (int i = 0; i < requestNames.length; i++) 
    mspRequestList.addItem(requestNames[i], i);
    
  /******************************************************************
   CP5 - SETUP DONE
   
   Note:
   
   1. Java OpenGL (JOGL) is a wrapper library that allows OpenGL to be used in Java/Processing.
      There is an exception thrown if setup() takes more than 5 seconds to finish after JOGL initialises the canvas.
      Consequently, do this at the end of setup().
   ******************************************************************/

  frameRate(60);
  size(1024, 768, P3D); 
  noSmooth();
  logConsole("Protocol Tester loaded - version: " + VERSION);
}

/******************************************************************
   Logging to Console
 ******************************************************************/

void logConsole(String msg) {
   consoleText.append(msg + "\n");
   consoleText.scroll(5.0);
}

/******************************************************************
 DRAW - Render Methods
 ******************************************************************/
 
 
 void setGradient(int x, int y, float w, float h, color c1, color c2, int axis ) {
  pushMatrix();
  noFill();

  if (axis == Y_AXIS) {  // Top to bottom gradient
    for (int i = y; i <= y+h; i++) {
      float inter = map(i, y, y+h, 0, 1);
      color c = lerpColor(c1, c2, inter);
      stroke(c);
      line(x, i, x+w, i);
    }
  }  
  else if (axis == X_AXIS) {  // Left to right gradient
    for (int i = x; i <= x+w; i++) {
      float inter = map(i, x, x+w, 0, 1);
      color c = lerpColor(c1, c2, inter);
      stroke(c);
      line(i, y, i, y+h);
    }
  }
  
  popMatrix();
}

void drawBackground()
{
  String version = "version " + VERSION;
  
  pushMatrix();
  fill(WHITE);
  textSize(24);
  textFont(bold);
  textAlign(LEFT);
  text("Protocol Tester", 20, 30);
  textSize(16);
  textFont(smallBold);
  text(version, 20, 50);
  setGradient(270, 10, 365, 30, color(0), nexgenGold, X_AXIS);
  popMatrix();
}

void drawFrameRate(int x, int y) {
  stroke(WHITE);
  fill(BLACK);
  rect(x, y, 95, 20);
  rect(x, y+20, 95, 20);
  
  fill(nexgenAqua);
  textFont(bold);
  textSize(14); 
  text("Frame Rate", x+8, y+15);
  fill(WHITE);
  text(int(frameRate) + " fps", x+28, y+35);
}

public void drawCommand(int x, int y) {
  stroke(WHITE);
  fill(cp5blue);
  rect(x, y, 40, 40);
  rect(x+40, y, 40, 40);
  rect(x+80, y, 40, 40);
  
  fill(nexgenGold);
  rect(x+120, y, 40, 40);
  rect(x+160, y, 40, 40);
  
  fill(cp5grey);
  rect(x+200, y, 40, 40);
  
  textFont(fixedWidthBold);
  textSize(14); 
  text("header", x+35, y-10);
  text("size", x+122, y-10);
  text("type", x+163, y-10);
  text("crc", x+207, y-10);
  
  text("0x24", x+2, y+55);
  text("0x4D", x+43, y+55);
  text("0x3C", x+83, y+55);
  text("0x00", x+123, y+55);
  text("0x" + String.format("%1$02X", mspRequest.code), x+163, y+55);
  text("0x" + String.format("%1$02X", mspRequest.code), x+203, y+55);
  
  fill(WHITE);
  textFont(bold);
  textSize(24);
  text("$", x+13, y+28);
  text("M", x+50, y+28);
  text("<", x+92, y+28);
  
  fill(BLACK);
  text("\\0", x+130, y+28);
  text(Character.toString((char)mspRequest.code), x+172, y+28);
  text(Character.toString((char)mspRequest.code), x+212, y+28);
}

void drawHexDump(int x, int y) {
  String rowHex = "", rowAscii = "";
  
  for (int i = 0; i < HEX_DUMP_ROWS; i++) {
    for (int j = 0; j < HEX_DUMP_COLS; j++) {
      String hexValue = String.format("%1$02X", hexDump[i][j]);
      
      rowHex = rowHex + " " + hexValue;
      rowAscii = rowAscii + " " + asciiDump[i][j];
      
      if (j == 7) {
        rowHex = rowHex + "   ";
        rowAscii = rowAscii + "   ";
      }
    }
    
    //  Highlight row being written to
    if (hexRow == i) {
      fill(cp5cyan);
      rect(x-15, y-10+i*20, 10, 10);
    }
    
    fill(cp5grey);
    textFont(fixedWidthBold);
    textSize(14); 
    text(String.format("%1$04X", byteCtr), x,  y+i*20);
    fill(WHITE);
    text(rowHex, x+40, y+i*20);
    textSize(12);
    text(rowAscii, x+470, y+i*20);
    
    rowHex = "";
    rowAscii = "";
    byteCtr += 128;
  }
  byteCtr = baseBytes;
}

void draw() 
{
  background(BLACK);
  lights();

  drawBackground(); 
  drawCommand(20, 140);
  drawHexDump(40, 520);
  drawFrameRate(900, 700);
  
  textFont(bold);
  textSize(12);
  fill(WHITE);
  text(day()+"/"+month()+"/"+year()+" - "+hour()+":"+minute()+":"+second(), 520, 740);
}

/******************************************************************
 Handle Serial Events
 ******************************************************************/
 
 void serialEvent(Serial s) {
  List<Character> payload;
  int c;
  
  try {
    while (s.available() > 0) {
      c = (s.read());
      String hexValue = String.format("%1$02X", c);
      char ascii = (char)c;
      
      switch(c) {
       case 0: 
         ascii = '0'; // NULL
         break;
       default:
         if (c < 32 || c > 126) //  Non-printable char
           ascii = '.';
      }
      
      hexDump[hexRow][hexCol] = c;
      asciiDump[hexRow][hexCol] = ascii;
      hexCol++;
      
      if (hexCol > HEX_DUMP_COLS - 1) {
        hexCol = 0;
        hexRow++;
        if (hexRow > HEX_DUMP_ROWS - 1) {
          hexRow = 0;
          baseBytes += 128;
        }
      }

      if (c_state == IDLE) {
        c_state = (c=='$') ? HEADER_START : IDLE;
      } 
      else if (c_state == HEADER_START) {
        c_state = (c=='M') ? HEADER_M : IDLE;
      } 
      else if (c_state == HEADER_M) {
        if (c == '>') {
          c_state = HEADER_ARROW;
        } 
        else if (c == '!') {
          c_state = HEADER_ERR;
        } 
        else {
          c_state = IDLE;
        }
      } 
      else if (c_state == HEADER_ARROW || c_state == HEADER_ERR) {
        /* is this an error message? */
        err_rcvd = (c_state == HEADER_ERR);        /* now we are expecting the payload size */
        dataSize = (c&0xFF);
        /* reset index variables */
        p = 0;
        offset = 0;
        checksum = 0;
        checksum ^= (c&0xFF);
        /* the command is to follow */
        c_state = HEADER_SIZE;
      } 
      else if (c_state == HEADER_SIZE) {
        cmd = (byte)(c&0xFF);
        checksum ^= (c&0xFF);
        c_state = HEADER_CMD;
      } 
      else if (c_state == HEADER_CMD && offset < dataSize) {
          checksum ^= (c&0xFF);
          inBuf[offset++] = (byte)(c&0xFF);
      } 
      else if (c_state == HEADER_CMD && offset >= dataSize) {
        /* compare calculated and transferred checksum */
        if ((checksum&0xFF) == (c&0xFF)) {
          if (err_rcvd) {
            logConsole("Drone did not understand request type " + c);
            //System.err.println("Copter did not understand request type "+c);
          } 
          else {
            /* we got a valid response packet, evaluate it */
            evaluateCommand(cmd, (int)dataSize);
          }
        } 
        else {
          System.out.println("invalid checksum for command "+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF));
          System.out.print("<"+(cmd&0xFF)+" "+(dataSize&0xFF)+"> {");
          for (int i=0; i<dataSize; i++) {
            if (i!=0) { System.err.print(' '); }
            System.out.print((inBuf[i] & 0xFF));
          }
          System.out.println("} ["+c+"]");
          System.out.println(new String(inBuf, 0, dataSize));
        }
        c_state = IDLE;
      }
    }
  }
  catch(Exception e) {
    logConsole("Error reading serial port " + portName);
    //  e.printStackTrace();
  }
}
 
 public void disconnectSerial(int n) {
  if (serialPort != null) {
    serialPort.clear();
    serialPort.stop();
    serialPort = null;
    logConsole("Protocol Tester: " + portName + " disconnected.");
    logConsole("________________________________________\n");
    serialPortsList.setCaptionLabel("Serial Ports");
    serialPortsList.setColorBackground(darkGrey);
    discButton.setColorBackground(darkGrey);
  }
}

public void refreshSerial(int n) {
  String[] portNames = Serial.list();

  serialPortsList.clear();

  for (int i = 0; i < portNames.length; i++) 
    serialPortsList.addItem(portNames[i], i);

  serialPortsList.close();
  serialPortsList.setCaptionLabel("Serial Ports");
  logConsole("Serial port list refreshed");
}

public void clearConsole(int n) {
  consoleText.clear();
}

public void copyConsole(int n) {
  GClip.copy(consoleText.getText());
}

public void serialPorts(int n) {
  //check if there's a serial port open already, if so, close it
  if (serialPort != null) {
    serialPort.clear();
    serialPort.stop();
    serialPort = null;
  }

  //open the selected port
  portName = cp5console.get(ScrollableList.class, "serialPorts").getItem(n).get("name").toString();

  try {
    serialPort = new Serial(this, portName, BAUD_RATE);
    serialPortsList.setColorBackground(darkGreen);
    logConsole("Connected to Serial Port: " + portName);
    logConsole("========================================\n");
    discButton.setColorBackground(cp5red);
  }
  catch(Exception e) {
    logConsole("Error opening serial port " + portName);
    //e.printStackTrace();
  }
}

/******************************************************************
 Handle CP5 Events
 ******************************************************************/

public void mspRequests(int n) {
  int code = MSP_FC_VARIANT;  //  Betaflight specific
  
  if (n != 0) code = n + 100;
  mspRequest = CommandID.valueOfCode(code);
}

public void sendRequest(int n) {
  String requestType = Character.toString((char)mspRequest.code);
  String crc = requestType;
  
  if (serialPort != null) {
    sendRequestMSP(requestMSP(mspRequest.code));
    logConsole("$M<\\0" + requestType + crc + " MSP Request Sent");
  }
  else {
    logConsole("Unable to send MSP message. No serial connection.");
  }
}
