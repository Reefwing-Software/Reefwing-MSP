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

The ArduPilot implementation supports only telemetry and sensor messages. So currently ArduPilot can’t be controlled by the Betaflight configurator. 

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

The MSP header is 3 bytes in length and has two start bytes `$M` followed by the message direction (`<` or `>`). 

- `<` - From the flight controller (FC →); and
- `>` - To the flight controller (→ FC).

## Size

The fourth byte is the length (in bytes) of the data payload. For example, if the data section had three `INT 16` variables then the size byte would be 6. This field can be zero as in the case of a data request from the Configurator.

## Type

The type byte is similar to our command/request byte. It defines the command to the drone, request for information or the type of response. A list of the original MSP command types is provided in the reference folder of the NexgenMSP library. 

The incoming message flow is composed of commands and requests while the outgoing flow contains responses. 

The message ID value of each command uses the convention:

- Value `1xx` identify requests; while 
- Value `2xx` identify commands.

By agreement, the message ID range from 50–99 won't be assigned in future versions of MSP and can therefore be used for any custom multiwii fork without fear of MSP ID conflict. 

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

## NexgenMSP

The Nexgen MSP library is a fork of [version 2.4 of the MultiWii Protocol class](https://medium.com/r/?url=https%3A%2F%2Fgithub.com%2Fxdu-aero-association%2FMultiWii_2_4%2Fblob%2Fmaster%2FMultiWii%2FProtocol.cpp), the [Betaflight MSP Protocol](https://github.com/betaflight/betaflight/blob/master/src/main/msp/msp_protocol.h) and the [Arduino library for MSP](https://medium.com/r/?url=https%3A%2F%2Fgithub.com%2Ffdivitto%2FMSP) (which has been archived).

In accordance with the Betaflight MSP guidelines we have created a flight controller `IDENT` for our subset of the protocol implemented. This `#define` may be found in `Protocol.h`.

```c++
#define NEXGEN_IDENTIFIER "NXGN";
```

To use the library, first include it and then create a new instance of the NexgenMSP class.

```c++
#include <NexgenMSP.h>

NexgenMSP msp;
```

Then in `setup()`, open the Serial port and await a connection (required for Arduino boards with native USB). Then allocate the serial stream and assign a timeout value (if left blank the default timeout = 0, i.e., there is no timeout).

```c++
void setup() {
  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  //  Allocate stream and timeout (default timeout = 0, i.e., no timeout)
  msp.begin(Serial);
}
```

## Examples

### 1. Terminal Test

The Betaflight MSP guidelines recommend that you first attempt to get a response from the `MSP_API_VERSION` command. We will use the Arduino IDE serial terminal as our surrogate Configurator.
