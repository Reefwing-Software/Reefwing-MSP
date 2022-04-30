# Nexgen MSP

 Nexgen MultiWii Serial Protocol (MSP)

NexgenMSP is a light weight Arduino implementation of the MultiWii Serial Protocol. This is required for easy configuration, simulation, telemetry, black box recording and On Screen Display (OSD) for First Person View (FPV) goggles.

This library was designed for incorporation into flight controller software running on Arduino hardware.

## MultiWii Serial Protocol (MSP)

The MultiWii Serial Protocol was originally created for use with the MultiWii Configurator (a Processing application like the Nexgen Configurator) to control a multirotor RC model. The first draft of the protocol was defined in 2012 and was was initially developed to support Nintendo Wii console gyroscopes and accelerometers (hence the name).

It is now used by a number of open source flight controllers like ArduPilot, the BetaFlight/CleanFlight family, HackFlight and iNav.

ArduPilot supports the MSP protocol for telemetry and sensors via any of its serial ports. This allows ArduPilot to send its telemetry data to MSP compatible devices, such as DJI goggles, for On Screen Display (OSD).

MSP is the main communication protocol used by all Betaflight derived flight stacks. It’s a binary message based protocol used for control, telemetry and sensors. ArduPilot’s MSP protocol module is ported from Betaflight and iNav, and supports both MSPV1 and MSPV2. The ArduPilot implementation (version 4.2) supports:

- MSP Telemetry
- MSP Telemetry OSDs such as DJI FPV Goggles, FatShark’s ByteFrost and SharkByte, MWOSD
- MSP DisplayPort OSDs such as FatShark’s Shark Byte and MWOSD
- MSP Sensors such as lidar, optical flow, gps, barometer, magnetometer and airspeed

The ArduPilot implementation supports only telemetry and sensor messages. So currently ArduPilot can’t be controlled by the Betaflight configurator. Also, there is no proper protocol and message specification that Cleanflight, Betaflight and iNav agree upon, so you may need to tailor you payload decoding to the application.

MSP requires a free serial port, and its speed defaults to 115200 baud.

The original design objectives for the MSP were:

- Light weight.
- Generic (i.e., used transparently by a GUI, OSD, telemetry or home made configuration tool).
- Bit wire efficient -  only sends requested data which is transmitted in a binary format.
- Secure - data is sent with a checksum, preventing corrupted configuration being injected.
- Header sensitive - as it is designed with a specific header, it can be mixed with other frames, like GPS. It should be possible to connect either a GUI or a GPS to the same serial port without changing configurations.
- Backwards compatible. It should be possible to add new commands without breaking previous versions of the protocol.
- Variable data length to future proof the protocol (e.g., to add a new PID controller).

As described above, originally MSP operated in `polling mode`, and would only send a message when requested. With MSP now being used for things like On Screen Displays (OSD) a `telemetry push mode` has been added to some implementations. NexgenMSP currently only supports `polling mode`. 

There are three types of MSP message that can be sent:
- Command - a message sent to the flight controller which has some information to be sent.
- Request - a message sent to the flight controller asking for some information to be returned.
- Response - a message sent by the flight controller with information responding to a request.

MSP messages have a specific structure. They have a header, size, type, data and checksum, in that order.

## Header

The MSP header is 3 bytes in length and has two start bytes `$M` followed by the message direction (`<` or `>`) or the error message indicator (`!`). 

- `<` - From the flight controller (FC →),
- `>` - To the flight controller (→ FC).
- `!` - Error Message.

An error message is a response to receipt of data that cannot be processed (corrupt checksum, unknown function, or message type that cannot be processed). It could be going in either direction.

## Size

The fourth byte is the length (in bytes) of the data payload. For example, if the data section had three `INT 16` variables then the size byte would be 6. This field can be zero as in the case of a data request from the Configurator.

## Type

The type byte is similar to our command/request byte. It defines the command to the drone, request for information or the type of response. A list of the original MSP command types is provided in the reference folder of the NexgenMSP library. 

The incoming message flow is composed of commands and requests while the outgoing flow contains responses. 

The message ID value of each command uses the convention:

- Value `1xx` identify requests; while 
- Value `2xx` identify commands.

By agreement, the message ID range from 50–99 won't be assigned in future versions of MSP and can therefore be used for any custom multiwii fork without fear of MSP ID conflict. Betaflight/Cleanflight have added custom messages below 50.

## Payload

The data payload depends on the type request. An example is the data request MSP_IDENT. This returns three `uint8_t` and one `uint32_t` bits of data. A full list of the returned data types is provided in the reference folder of the NexgenMSP library. Multi-byte data (e.g., `uint16_t`) is transmitted LSB first.

## Checksum

The checksum is computed as the `XOR` of the size, type and payload bytes. The checksum of a request (i.e, a message with no payload) equals the type.

## Betaflight MSP Guidelines

A set of guidelines is provided with the [Betaflight msp_prrotocol class](https://github.com/betaflight/betaflight/blob/master/src/main/msp/msp_protocol.h). We have reproduced these below.

```c++
/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * Each FlightController (FC, Server) MUST change the API version when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible.  Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the returned API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */
```

## MSP v1 JUMBO Messages

In addition to the standard MSP implementation outlined above, some people use JUMBO messages. This was a stop gap measure introduced to increase message size. The MSP v1 protocol is limited to 255 byte message payloads. This combined with limited message ID's and weak check summing is what eventually led to MSPV2.

JUMBO messages have the following characteristics:

- It is a MSP v1 $M ... message
- Set the function code as normal (0–255)
- Set the payload size to 255
- Set the real real payload size as the first two bytes of the payload
- Then the real payload
- Then a MSP V1 XOR checksum as normal

## MSP Protocol Version 2 (MSPV2)

MSPV2 was introduced in iNav 1.73 for legacy commands (September 2017), and is fully implemented (16bit commands) after 1.73 (i.e. 1.74 development branch and successors). An MSP API version of 2 or greater indicates MSPV2 support.

MSPV2 differs from Version 1 in the following ways:

- 16 bit message space. 65535 message IDs. For backwards compatibility, message IDs 0–255 map onto the analogous MSP v1 messages.
- 16 bit payload.
- crc8_dvb_s2 checksum algorithm. This is a single byte CRC algorithm that is much more robust than the XOR checksum in MSP v1.

It is possible to encapsulate V2 messages in a V1 message. This is implemented by setting the V1 function id to 255 and creating a payload of a V2 message without the first three header bytes. Thus a V1 consumer would see a not understood message rather than a communications error. 

A sample MSPV2 request for MSP_IDENT would look like:

```
"$X<\x00d\x00\x00\x00\x8F"
24 58 3c 00 64 00 00 00 8f
```

The equivalent MSPV1 message would be:

```
"$M<\x00dd"
24 4D 3C 00 64 64
```

To calculate the MSPV2 checksum, you can use the code snippet provided by the devs at iNav. The checksum should be initialised to zero.

```c
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}
```

## NexgenMSP

The Nexgen MSP library is a fork of [version 2.4 of the MultiWii Protocol class](https://medium.com/r/?url=https%3A%2F%2Fgithub.com%2Fxdu-aero-association%2FMultiWii_2_4%2Fblob%2Fmaster%2FMultiWii%2FProtocol.cpp), the [Betaflight MSP Protocol](https://github.com/betaflight/betaflight/blob/master/src/main/msp/msp_protocol.h) and a fork of the [Arduino library for MSP](https://medium.com/r/?url=https%3A%2F%2Fgithub.com%2Ffdivitto%2FMSP) (which has been archived) plus some custom additions for the Nexgen Configurator. This first version of our library uses MSP v1, in a future release we will add support for version 2.

Note that the archived [Arduino library for MSP](https://medium.com/r/?url=https%3A%2F%2Fgithub.com%2Ffdivitto%2FMSP) is designed to talk with a flight controller, probably for a black box application. Because of this, the message direction indicators (`<` and `>`) are back to front for our requirements. In addition, we have added two new methods:

```c++
void response(uint8_t messageID, void * payload, uint8_t size);
void error(uint8_t messageID, void * payload, uint8_t size);
```

For a flight controller, you will normally be responding to requests from a Configurator or Ground Station. To do this, use the response() method. To respond to the receipt of data that cannot be processed (e.g., corrupt checksum, unknown function, or a message type that cannot be processed), you can use the MSP error() function. An error message uses the special header $M!, which will be recognised by the Configurator as an error.

The MSP protocol was originally designed to only send messages when requested but with the introduction of FPV goggles with OSD, a polling mode has been added to version 2 of the MSP protocol.

In accordance with the Betaflight MSP guidelines we have created a flight controller `IDENT` for our subset of the protocol implemented. This `#define` may be found in `Protocol.h`.

```c++
#define NEXGEN_IDENTIFIER "NXGN";
```

To use the library, first include it and then create a new instance of the NexgenMSP class.

```c++
#include <NexgenMSP.h>

NexgenMSP msp;
```

Then in `setup()`, open the Serial port and await a connection (required for Arduino boards with native USB). Then allocate the serial stream and assign a timeout value (if left blank the default timeout = 500 ms).

```c++
void setup() {
  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  //  Allocate stream and timeout (default timeout = 500 ms)
  msp.begin(Serial);
}
```

## Examples

For debugging we thought it would be handy to show the raw MSP messages in the Aduino Serial Monitor. Unfortunately the Serial Monitor expects ASCII characters and it isn't possible to send raw hexidecimal or binary bytes (to my knowledge) from the Monitor to the Arduino. The MSP protocol has a lot of message ID's, size and payloads with a byte value less than 33 (decimal) which are invisible in the Serial Monitor. 

For ASCII the visible characters start at 33 decimal. All MSP requests require a size value of zero which corresponds to the ASCII `NUL` character. You can theoretically add unicode characters directly in OS X and Windows but the NUL character is invisible and doesn't work as an input to the Serial Monitor. C strings use the NUL character (`\0`) as a string terminator, so it doesn't help having that in the middle of a message packet.

For these reasons, you will need to use something like [Serial Tools](https://medium.com/r/?url=https%3A%2F%2Fapps.apple.com%2Fau%2Fapp%2Fserialtools%2Fid611021963%3Fmt%3D12) for OS X or [PuTTY](https://medium.com/r/?url=https%3A%2F%2Fwww.putty.org%2F) for Windows, or our Processing Protocol Tester in order to view and send MSP messages.

### 1. View MSP Request

This example allows you to see some MSP requests in the Arduino Serial Monitor but as explained above, you wont see any of the invisible characters (i.e., any bytes with a value less than 33 decimal). Select "Newline" and 115200 baud from the pulldown menus in the Serial Monitor.

There are three types of MSP messages that can be sent:

- Command: a message sent to the flight controller which has some 
 information to be sent.
- Request: a message sent to the flight controller asking for some 
 information to be returned.
- Response: a message sent by the flight controller with information 
 responding to a request.

This sketch demonstrates requests and commands. These messages would usually be sent by the Configurator or equivalent. One application where you may need an Arduino to send requests/commands would be if you had some sort of black box or parallel processing situation.

The `viewMSPResponse.ino` sketch below is the corollary to this sketch and demonstrates expected responses from the Flight Controller. This is the more likely use case.

### How to Use

Upload the sketch to an Arduino board and open the Serial Monitor or Serial Tools. Enter the message ID name (e.g., `MSP_IDENT`) into the command box and press `ENTER` or `Send` to see that message displayed. Valid message ID's may be found in the Protocol.h file. 

The third byte of the message after M$ will be either:

- `<`: denotes going to the flight controller (command and request).
- `>`: denotes coming from the flight controller (response).
- `!`: error (response).

### 2. View MSP Response

This example, allows you to see an MSP response in the Processing Protocol Tester provided with this library. It is difficult to demonstrate a serial connection with only one end attached, so we developed the Protocol Tester to provide a simplified Configurator example. It is written in Processing (Java), so you should be able to run this application on OS X, Windows and Linux.

These messages would usually be sent by the Flight Controller or equivalent. The MSP receive method, recv() has the following header prototype:

```c++
bool recv(uint8_t *messageID, void *payload, uint8_t maxSize, uint8_t *recvSize); 
```

Note that the messageID and recvSize variables should be pointers to uint8_t and the payload is a pointer to void.

### Nexgen Protocol Tester

The Protocol Tester runs on a PC with OS X, Windows or Linux. You will need to open up the processing sketch, protocolTester.pde in the Processing 3 environment and run it. The components of the tester are as follows:

- The `SERIAL PORTS` drop down in the top right allows you to select the port that your Arduino is connected to. If you connected the Arduino after starting this app, click on `REFRESH` to see any newly connected ports. After selecting the correct port, click on `CONNECT`. This same button can be used to `DISCONNECT` the port after connection.
- The logging console is below the Serial Port selection. This will display real time status and error messages. The `CLEAR` button will empty the console, and `COPY` will copy the contents of the console to the clip board for pasting into another application.
- The currently selected message request is shown in the top left. For example, if `MSP_IDENT` is selected, the request will show: `$M<\0dd`. Above the request are the parts of an MSP packet (header, size, type, and crc), and below it are the ASCII encoding for the individual message characters in hexidecimal (e.g., `'$' = 0x24`).
- Clicking the `SEND` button, will transmit the currently selected MSP request to the Arduino.
- Below the displayed request is a drop down from which you can select a new request. The message contents will automatically update, when a new message type is selected.
- At the bottom is a hex dump of the received serial characters and their ASCII equivalents. Invisible characters like `NULL`, line feed and carriage return are represented by a full stop (`.`).

### How to Use

1. Upload the `mspResponse.ino` sketch to an Arduino board and run the Protocol Tester on your PC. In this example, the Protocol Tester (PT) is simulating the Configurator and our sketch is emulating the flight controller. 
2. In the PT, select the serial port that your Arduino is connected to and click on the `CONNECT` button.
3. Select a request from the drop down (e.g., `MSP_STATUS`) and click on `SEND` to transmit the message to the Arduino. The message sent will be displayed in the log console as will any response or error. The response will also be displayed in the hex dump at the bottom of the PT screen.

In order for there to be a response, other than an error, it has to be handled in the `mspResponse` sketch.