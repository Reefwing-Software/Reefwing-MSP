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
ScrollableList serialPortsList;
Textarea consoleText;
String serialString = null;
PFont bold, smallBold, regular;

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

  cp5mainMenu.setFont(cf1);
  cp5console.setFont(cf1);
  
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

void draw() 
{
  background(BLACK);
  lights();

  drawBackground(); 
  drawFrameRate(543, 640);
  
  textFont(bold);
  textSize(12);
  fill(WHITE);
  text(day()+"/"+month()+"/"+year()+" - "+hour()+":"+minute()+":"+second(), 520, 740);
}

/******************************************************************
 Handle Serial Events
 ******************************************************************/
 
 
 /******************************************************************
 Handle Control Events - Serial
 ******************************************************************/
 
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
