/*--------------------------------------------------------------------
  This file is part of the RFSee ApeldoornInData node.
  
  The RFSee ApeldoornInData node is free software: 
  you can redistribute it and/or modify it under the terms of a Creative 
  Commons Attribution-NonCommercial 4.0 International License 
  (http://creativecommons.org/licenses/by-nc/4.0/) by 
  PE1MEW (http://pe1mew.nl) E-mail: pe1mew@pe1mew.nl

  The RFSee ApeldoornInData node is distributed in the hope that 
  it will be useful, but WITHOUT ANY WARRANTY; without even the 
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
  PURPOSE.
  --------------------------------------------------------------------*/

/*!

 \file AID_UNO_config.h
 \brief config file for RFSee ApeldoornInData node
 \date 23-9-2019
 \author Remko Welling (RFSee)
 \version 10
  
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

// select sensor type: SENSOR_SCD30 or SENSOR_BMPSHT2
//#define SENSOR_SCD30
#define SENSOR_BMPSHT2

#define PARTICLE_PREFLOW_DURATION 5000  ///< time in ms preflow before particle measurement starts
#define MEASUREMENTS              5     ///< Maximum number of times that a sensor is read.
#define MAXIMUM_TEMPERATURE       50    ///< Maximum temperature that is accepted
#define MINIMUM_TEMPERATURE       -20   ///< Minimum temperature that is accepted
#define MAXIMUM_HUMIDITY          100.1 ///< Maximum humidity that is accepted

// select the type of personalisation
// ----------------------------------
#define OTAA  // See device
//#define ABP


// Channe numbers used in CayenneLPP protocol for al sensors
// ---------------------------------------------------------
#define LPP_CH_VCCVOLTAGE         0     ///< Using AnalogInput in CayenneLPP
#define LPP_CH_TEMP               1
#define LPP_CH_ADC0VOLTAGE        2     ///< Using AnalogInput in CayenneLPP
#define LPP_CH_ADC1VOLTAGE        3     ///< Using AnalogInput in CayenneLPP
#define LPP_CH_HARTBEAT           4     ///< using luminosity in CayenneLPP
#define LPP_CH_BATTOK             5     ///< using Presence in CayenneLPP
#define LPP_CH_SEQUENCE           7     ///< using DigitalInput in CayenneLPP
#define LPP_CH_GPS                8
#define LPP_CH_TEMPERATURE        10
#define LPP_CH_HUMIDITY           20
#define LPP_CH_BAROMETER          30
#define LPP_CH_GAS_CO2            40    ///< using luminosity in CayenneLPP
#define LPP_CH_PARTICLE_25        50    ///< using luminosity in CayenneLPP
#define LPP_CH_PARTICLE_10        51    ///< using luminosity in CayenneLPP

#define LPP_CH_SET_INTERVAL       91    ///< CayenneLPP Channel for setting downlink interval
#define LPP_CH_SW_RELEASE         90    ///< using AnalogOutput in Cayenne LPP

#define APPLICATION_PORT_CAYENNE  99    ///< Application port of Cayenne

// Set up application specific
#define REGULAR_INTERVAL          60000 ///< Regular transmission interval in ms
#define MINIMUM_INTERVAL          30000 ///< minimum allowed interval in ms

// Setup TTN credentials
// ---------------------
#if defined(OTAA)
  const char *appEui  = "0000000000000000";
  const char *appKey  = "00000000000000000000000000000000";
#elif defined(ABP)
  const char *devAddr = "00000000";
  const char *nwkSKey = "00000000000000000000000000000000";
  const char *appSKey = "00000000000000000000000000000000";
#else
  #error "No personalisation configured."
#endif

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868

#endif // __CONFIG_H__
