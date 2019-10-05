/*--------------------------------------------------------------------
  This file is part of the PE1MEW TTN LoRa Beacon.
  
  The PE1MEW TTN LoRa Beacon is free software: 
  you can redistribute it and/or modify it under the terms of a Creative 
  Commons Attribution-NonCommercial 4.0 International License 
  (http://creativecommons.org/licenses/by-nc/4.0/) by 
  PE1MEW (http://pe1mew.nl) E-mail: pe1mew@pe1mew.nl

  The PE1MEW TTN LoRa Beacon is distributed in the hope that 
  it will be useful, but WITHOUT ANY WARRANTY; without even the 
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
  PURPOSE.
  --------------------------------------------------------------------*/

 /// \file PE1MEW_EEPROM.cpp
 /// \brief Memory class to use EEPROM
 /// \date 29-9-2019
 /// \author Remko Welling (PE1MEW)
 /// \version 1 Development
 
#include "PE1MEW_EEPROM.h"
#include "Arduino.h"

PE1MEW_EEPROM_memory::PE1MEW_EEPROM_memory()
{
  if ( EEPROM_LAYOUT_MAGIC != generic4ByteUintRead(EEPROM_LAYOUT_MAGIC_START, EEPROM_LAYOUT_MAGIC_LEN))
  {
    for (int i = EEPROM_LAYOUT_FRAME_COUNTER_UP_START; i < EEPROM_LAYOUT_INTERVAL_START + EEPROM_LAYOUT_INTERVAL_LEN; i++)
    {
      EEPROM.update(i, 0x00);
    }
    generic4ByteUintWrite(EEPROM_LAYOUT_MAGIC, EEPROM_LAYOUT_MAGIC_START, EEPROM_LAYOUT_MAGIC_LEN);
  }
}

uint32_t PE1MEW_EEPROM_memory::readFrameCounterUp(void)
{
  return generic4ByteUintRead(EEPROM_LAYOUT_FRAME_COUNTER_UP_START, EEPROM_LAYOUT_FRAME_COUNTER_UP_LEN);
}

void PE1MEW_EEPROM_memory::writeFrameCounterUp(uint32_t value)
{
  generic4ByteUintWrite(value, EEPROM_LAYOUT_FRAME_COUNTER_UP_START, EEPROM_LAYOUT_FRAME_COUNTER_UP_LEN);
}

uint32_t PE1MEW_EEPROM_memory::readTXInterval(void)
{
  return generic4ByteUintRead(EEPROM_LAYOUT_INTERVAL_START, EEPROM_LAYOUT_INTERVAL_LEN);
}

void PE1MEW_EEPROM_memory::writeTXInterval(uint32_t value)
{
  generic4ByteUintWrite(value, EEPROM_LAYOUT_INTERVAL_START, EEPROM_LAYOUT_INTERVAL_LEN);
}

uint32_t PE1MEW_EEPROM_memory::generic4ByteUintRead(int startAddress, int bytes)
{
  uint32_t returnValue = 0;
  for (int i = startAddress; i < (startAddress + bytes); i++ )
  {
      returnValue += ((uint32_t)EEPROM.read(i) << ((i - startAddress) * 8));
  }
  return returnValue;
}

void PE1MEW_EEPROM_memory::generic4ByteUintWrite(uint32_t val, int startAddress, int bytes)
{
  for (int i = startAddress; i < (startAddress + bytes); i++ )
  {
      EEPROM.update(i, (uint8_t)(val >> ((i - startAddress) * 8)));
  }
}
