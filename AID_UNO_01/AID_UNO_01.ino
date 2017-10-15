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

/// \file AID_UNO_01.ino
/// \brief RFSee ApeldoornInData node
/// \date 15-10-2017
/// \author Remko Welling (RFSee)
/// \version 0.1  15-10-2017 Initial version

/// These are the defines for this project:

#include <SDS011.h>             /// Library for SDS011 Particle sensor
#include <Wire.h>               /// Library for I2C
#include <Adafruit_BMP085.h>    /// Library for pressure
#include <Sodaq_SHT2x.h>        /// Library for temperature and humidity
//#include <BH1750.h>           /// Library for lightmeter
#include "CayenneLPP.h"         /// Library for Cayenne library
#include "beaconled.h"          /// PE1MEW_Led class
#include "TheThingsNetwork.h"   /// Library for TTN Network (Uno)

#define loraSerial Serial1      /// Configure serial 1 to RN radio
#define debugSerial Serial      /// Configure serial for debug output 

/// select the type of personalisation
//#define OTAA 
#define ABP

/// Setup all hardware connected.

/// Setup led
beaconLed  led = beaconLed(LED_BUILTIN);  /// Built in LED (Green) for information about operation.
  
/// Setup paricle sensor
SDS011 my_sds;

/// Variables for the particle sensor
/// we will collect 5 measuremenst for both P10 and P2.5.
/// From these measuremenets we calculate the maximum, average and minumum value.
float p10,p25,p10min,p10avg,p10max,p25min,p25avg,p25max;
float p10measurements[5] { 0 };   /// All P10 measurements in a array
float p25measurements[5] { 0 };   /// All P2.5 measurements in a array
int error;                        /// varable to store the serial read result of the SDS011
bool measurementComplete = false; /// housekeeping variable to know when we are finished.

/// Setup i2c sensors

Adafruit_BMP085 bmp;              /// BMP085 barometer and temperature sensor
/// not used in this project.
//BH1750 lightMeter;              /// BH1750 lightMeter

/// Setup Cayenne LPP

#define APPLICATION_PORT_CAYENNE 99 /// Application port of Cayenne

CayenneLPP lpp(51);               /// Cayenne object for composing sensor message

uint8_t lppChannel { 0 };         /// LPP channel iterator

/// Setup TTN

#if defined(OTAA) 
  // Production sensor aid-10
  // Set your AppEUI and AppKey 
  const char *appEui = "xxxxxxxxxxxxxxxx";
  const char *appKey = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
#elif defined(ABP)
  // test sensor aid-2
  // Set your DevAddr, NwkSKey, AppSKey and the frequency plan
  const char *devAddr = "xxxxxxxx";
  const char *nwkSKey = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
  const char *appSKey = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
#else
  #error "No personalisation configured."
#endif

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_EU868

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);  /// object for TTN network

/// \brief setting up the node
void setup()
{
  /// Start Particle sensor
  my_sds.begin(8,9);
  
  /// Start the bmp sensor
  bmp.begin();
  
  /// Start serial ports
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  /// Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;
  
  /// send information about TTN to debug
  ttn.onMessage(message);

  /// Reset LoRaWAN mac and enable ADR
  ttn.reset(true);

  /// Tell the world that we startup TTN connectivity
  debugSerial.println("-- PERSONALIZE");

  /// Indicate to the outside taht we are active on the LoRaWAN radio
  led.setOn();
  
  /// Now personalize the LoRaWAN connection
#if defined(OTAA)
  ttn.join(appEui, appKey);
#elif defined(ABP)
  ttn.personalize(devAddr, nwkSKey, appSKey);
#endif

  /// Indicate to the outside taht we are no longer active on the LoRaWAN radio
  led.setOff();
   
  /// Tell the world the result of our work so far
  debugSerial.println("-- STATUS");
  ttn.showStatus();
}

/// \brief main loop of the program
void loop()
{
  /// Tell the world we are starting our loop
  debugSerial.println("-- LOOP");
  
  /// Indicate we are preparing the particle measurement
  led.blink(1);
  /// wakeup the SDS010
  my_sds.wakeup();
  /// Wait 10 seconds to allow the flow of air troug the SD010
  delay(10000);
  /// Set our housekeeping value to false as we are not ready
  measurementComplete = false;

  /// Indicate we are measuring particles and more,
  led.blink(2);
  /// wait until the measurement of particles is ready
  while(!measurementComplete)
  {
    
    /// Tell the world we are measuring particles
    debugSerial.println("Reading particle sensor... ");

    /// reset all values so we have a "clean sheet"
    int measurement = 0;
    p10min = 0;
    p10avg = 0;
    p10max = 0;
    p25min = 0;
    p25avg = 0;
    p25max = 0;
    memset(p10measurements, 0, sizeof(p10measurements));
    memset(p25measurements, 0, sizeof(p10measurements));
    
    /// We will take 5 measurements
    while(measurement < 5)
    {
      /// read sensor value from soft serial port
      error = my_sds.read(&p25,&p10);
      /// when all data is received process it.
      if (! error) 
      {
        /// Show the world our intermediate results
        debugSerial.println("P2.5: "+String(p25)+", P10: "+String(p10));
        /// save our intermediate results
        p10measurements[measurement] = p10;
        p25measurements[measurement] = p25;
        /// keep track of the number of measurements we have. so: +1!
        measurement++;
      }
      /// wait 100 mS to take a breath
      delay(100);
    }

    /// Now we are ready we will collect our "statistics" over 5 measurements
    for( int i = 0; i < 5; i++)
    {
      /// set values when first statistics cycle is performed.
      /// This will simplify the average calculation later.
      if(i == 0)
      {
        p10min = p10measurements[i];
        p10avg = p10measurements[i];
        p10max = p10measurements[i];
        p25min = p25measurements[i];
        p25avg = p25measurements[i];
        p25max = p25measurements[i];
      }

      /// take maximum and minimum values form the array of measurements
      if(p10measurements[i] < p10min)
      {
        p10min = p10measurements[i];
      }
      if(p10measurements[i] > p10max)
      {
        p10max = p10measurements[i];
      }
      if(p25measurements[i] < p25min)
      {
        p25min = p25measurements[i];
      }
      if(p25measurements[i] > p25max)
      {
        p25max = p25measurements[i];
      }
     
      /// Calculate averages
      p10avg += p10measurements[i];
      p10avg = ( p10avg / 2.0 );
      p25avg += p25measurements[i];
      p25avg = ( p25avg / 2.0 );
    }

    /// Tell the world teh result of our work.
    debugSerial.println("P2.5 min/avg/max: "+String(p25min)+" / "+String(p25avg)+" / "+String(p25max)+", P10 min/avg/max: "+String(p10min)+" / "+String(p10avg)+" / "+String(p10max));
    measurementComplete = true;
  }

  /// Collect all data from other sensors,save the for later and tell the world.
  debugSerial.print("Reading temperature from pressure... ");
  float pressureTemperature = (float)bmp.readTemperature();
  debugSerial.print(pressureTemperature);
  debugSerial.println(" C.");
  
  debugSerial.print("Reading Barometric pressure... ");
  float pressure = (float)bmp.readPressure()/100;
  debugSerial.print(pressure);
  debugSerial.println(" Pascal.");
  
  debugSerial.print("Reading Humidity... ");
  float humidity = SHT2x.GetHumidity();
  debugSerial.print(humidity);
  debugSerial.println(" %RH.");
  
  debugSerial.print("Reading Temperature... ");
  float temperature = SHT2x.GetTemperature();
  debugSerial.print(temperature);
  debugSerial.println(" C.");

//  debugSerial.print("Reading Dewpoint... ");
//  float dewpoint = SHT2x.GetDewPoint();
//  debugSerial.print(dewpoint);
//  debugSerial.println(" C.");

//  debugSerial.print("Reading light intensity... ");
//  float light = lightMeter.readLightLevel();
//  debugSerial.print(light);
//  debugSerial.println(" Lux.");

  /// Compose the Cayenne message
  /// Reset the cayenne object
  lpp.reset();
  /// Reset the channel counter
  lppChannel = 0;
      
  /// add sensor values to cayenne data package
//      lpp.addAnalogInput(lppChannel++, batteryVoltage);
  lpp.addTemperature(lppChannel++, temperature);
  lpp.addRelativeHumidity(lppChannel++, humidity);
//      lpp.addLuminosity(lppChannel++, light);
  lpp.addBarometricPressure(lppChannel++, pressure);
//      lpp.addAnalogInput(lppChannel++, dewpoint);
  lpp.addAnalogInput(lppChannel++, p25avg);
  lpp.addAnalogInput(lppChannel++, p10avg);
  
  /// Indicate to the outside that we are sending data
  led.setOn();
  
  // Send it off
  ttn.sendBytes(lpp.getBuffer(), lpp.getSize(), APPLICATION_PORT_CAYENNE);

  /// Indicate to the outside that we are ready sending data
  led.setOff();

  /// Now lets get to sleep
  /// \todo add som loop with led blinking idle state.
  my_sds.sleep();
  delay(50000);

}

/// \brief function to handle incoming message
/// \todo take downlink message to change interval
void message(const uint8_t *payload, size_t size, port_t port)
{
  debugSerial.println("-- MESSAGE");
  debugSerial.print("Received " + String(size) + " bytes on port " + String(port) + ":");

  for (int i = 0; i < size; i++)
  {
    debugSerial.print(" " + String(payload[i]));
  }

  debugSerial.println();

  //Toggle red LED when a message is received

}
