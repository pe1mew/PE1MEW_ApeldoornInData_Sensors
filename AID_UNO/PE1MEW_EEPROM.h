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

 /// \file PE1MEW_EEPROM.h
 /// \brief Memory class to use EEPROM
 /// \date 29-9-2019
 /// \author Remko Welling (PE1MEW)
 /// \version 1 Development

 
#ifndef _PE1MEW_EEPROM_H
#define _PE1MEW_EEPROM_H

#include <EEPROM.h>

/// \par comment on first programming
///  

/*!

  EEPROM memory layout
  --------------------
  
\verbatim

 start 
 0x00
  +----+----+----+----+---
  | 1  | 2  | 3  | 4  | Unused EEPROM memory
  +----+----+----+----+--

\endverbatim
  
    1. Magic number. A random number to identify wethever EEPROM is initialized (formatted)
    2. Uplink frame counter: uint32_t.
    3. Interval in milliseconds: uint32_t.
    4. Unallocated space
  
  Layout definition
  -----------------

  Using defines the layout is configured. Each variable has a start address and a length.
  The start address of the following variable is calculated using the previous start address summed with the length of the previous variable.
  
 */

#define EEPROM_LAYOUT_MAGIC 0x0001              ///< Just a random number, stored little-endian
#define EEPROM_LAYOUT_MAGIC_LEN 4
#define EEPROM_LAYOUT_MAGIC_START 0x00
#define EEPROM_LAYOUT_FRAME_COUNTER_UP_LEN 4    ///< 4 bytes
#define EEPROM_LAYOUT_FRAME_COUNTER_UP_START (EEPROM_LAYOUT_MAGIC_START + EEPROM_LAYOUT_MAGIC_LEN)       ///< 4 bytes
#define EEPROM_LAYOUT_INTERVAL_LEN 4            ///< 4 bytes
#define EEPROM_LAYOUT_INTERVAL_START (EEPROM_LAYOUT_FRAME_COUNTER_UP_START + EEPROM_LAYOUT_FRAME_COUNTER_UP_LEN)          ///< 4 bytes

/// \class PE1MEW_EEPROM_memory
/// \brief memory class
class PE1MEW_EEPROM_memory
{
public:
  /// \brief Default constructor
  PE1MEW_EEPROM_memory();

  /// \brief read Uplink frame counter from EEProm
  /// \return Uplink frame counter
  uint32_t readFrameCounterUp(void);
  
  /// \brief write Uplink frame counter to EEProm
  /// \param[in] value Uplink frame counter
  void writeFrameCounterUp(uint32_t value);
  
  /// \brief read TX-Interval from EEProm
  /// \return TX-Interval
  uint32_t readTXInterval(void);
  
  /// \brief write TX-Interval to EEProm
  /// \param[in] value TX-Interval
  void writeTXInterval(uint32_t value);
 
private:
  /// \brief generic function to read uint32_t variables to EEPROM
  /// \param[in] startAddress in EEPROM range
  /// \param[in] bytes Number of bytes to read.
  uint32_t generic4ByteUintRead(int startAddress, int bytes=4);
  
  /// \brief generic function to write uint32_t data to EEPROM
  /// \param[in] val Uint32_t value to be written to EEPROM 
  /// \param[in] startAddress in EEPROM range
  /// \param[in] bytes Number of bytes to write. 
  void generic4ByteUintWrite(uint32_t val, int startAddress, int bytes=4);

};
#endif // _BEACONMEMORY_H
