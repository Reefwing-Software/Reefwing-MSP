/******************************************************************
  @file       Protocol.cpp
  @brief      Commands, requests and response for the MultiWii Serial Protocol.
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/02/22

  1.0.0 Original Release.           22/02/22

  Credit - Version 2.4 of the MultiWii Protocol class.
           ref: https://github.com/xdu-aero-association/MultiWii_2_4/blob/master/MultiWii/Protocol.cpp
         - Arduino library for MSP by Fabrizio Di Vittorio
           ref: https://github.com/fdivitto/MSP
         - Version 1.44 of the Betaflight MSP Protocol
           ref: https://github.com/betaflight/betaflight/tree/master/src/main/msp

******************************************************************/

#include <Arduino.h>

#include "Protocol.h"

int idLookup(char *key) {
  for (int i = 0; i < NKEYS; i++) {
      msp_commands_t sym = lookupTable[i];

      if (!strcmp(sym.key, key))
          return sym.val;
  }

  return BAD_ID;
}