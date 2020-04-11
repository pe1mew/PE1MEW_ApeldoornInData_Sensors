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

 \file AID_UNO.ino
 \brief RFSee ApeldoornInData node
 \date See revision table
 \author Remko Welling (RFSee)
 \version See revision table
  
 */
#define RELEASE   13    ///< Software version of the sketch
/*!
 Release histroy
 ---------------
 
 Version|Date        |Note
 -------|------------|----
 <10    | 2019       | Previous versions of this sketch that form the fundament of the design 
 10     | 28-9-2019  | New release for using with SCD30
 11     | 30-9-2019  | Added function to store TX interval in EEPROM for reuse at reboot when set remote.
 12     | 30-9-2019  | Added hardware support for different temp/hum/barometer/luminosity sensor.
 13     | 25-3-2020  | Added hardware support for Marvin LoRa board and BMP280.

 Configure for programming the board
 -----------------------------------
 Configure board in IDE as:
 - Board: Arduino Leonardo.
 - Port: select the port that identifies as Arduino Leonardo.

 Work progress
 -------------
 \todo 3 add memory function for TTN credentials (OTAA / ABP)
 \todo move particle measurement to a separate function
 \todo make particle measurement non-blocking when sensor fails.
 
 */

#include "AID_UNO_config.h"       // Configuriation file for node
#include <Wire.h>                 // Library for I2C
#include <avr/wdt.h>              // Watchdog 
#include "CayenneLPP.h"           // Library for Cayenne library
#include "TheThingsNetwork.h"     // Library for TTN Network (Uno)
#include "SDS011.h"               // Library for SDS011 Particle sensor
#include "RunningAverage.h"       // Running average library
#include "PE1MEW_EEPROM.h"        // EEPROM library for PE1MEW LoRaWAN sensors

#if defined(SENSOR_BMP280)
  #include <BMx280I2C.h>          //
#elif defined(SENSOR_SCD30)
  #include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#elif defined(SENSOR_BMP085SHT2X)
  #include <Adafruit_BMP085.h>    // Library for pressure
  #include <Sodaq_SHT2x.h>        // Library for temperature and humidity
  //#include <BH1750.h>           // Library for lightmeter
#else
  #error "No sensor selected."
#endif

// Setup all peripherals and variables

// Serial ports
// ------------
#define loraSerial Serial1                ///< Configure serial 1 to RN radio
#define debugSerial Serial                ///< Configure serial for debug output 

// Particle measurements
// ---------------------
// Using Particle sensor SDS011 we will collect 5 measuremenst for both P10 and P2.5.
// From these measuremenets we calculate the maximum, average and minumum value.
int  error { 0 };                         ///< varable to store the serial read result of the SDS011
bool measurementComplete { false };       ///< housekeeping variable to know when we are finished.
int  measurementCounter { MEASUREMENTS }; ///< counter to keep track of the number of measurements taken.
float p10SensorValue { 0 };               ///< Variable to store intermediate result of SDS011 p10 measurement
float p25SensorValue { 0 };               ///< Variable to store intermediate result of SDS011 p2.5 measurement

SDS011         my_sds;                    ///< Particle sensor object
RunningAverage p25Buffer(MEASUREMENTS);   ///< Running average buffers for p2.5 particles
RunningAverage p10Buffer(MEASUREMENTS);   ///< Running average buffers for p10 particles

// Temperature, Humidity, CO2 sensor
// ---------------------------------
// Because of the coice for using 2 sensor types these can be selected in file: AID_UNO_config.h

float temperature { 0.0 };                ///< Variables to hold temperature value

#if defined(SENSOR_BMP280)
  #define I2C_ADDRESS 0x76
  BMx280I2C bmx280(I2C_ADDRESS);          ///< create a BMx280I2C object using the I2C interface with I2C Address 0x76
  float pressure { 0.0 };                 ///< Variables to hold barometric pressure
#elif defined(SENSOR_SCD30)
  SCD30 airSensor;                        ///< SCD30 airsensor object
  float humidity { 0.0 };                   ///< Variables to hold humidity value
  float gas { 0.0 };                      ///< Variables to hold CO2 value
#elif defined(SENSOR_BMP085SHT2X)
  Adafruit_BMP085 bmp;                    ///< BMP085 barometer and temperature sensor
  float humidity { 0.0 };                   ///< Variables to hold humidity value
  float pressure { 0.0 };                 ///< Variables to hold barometric pressure
#endif

// Setup LoRaWAN
// -------------
CayenneLPP lpp(51);                       ///< Cayenne object for composing sensor message
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);  ///< object for RN2483 LoRaWAN radio. variables are configured in file: AID_UNO_config.h

// Setup node end of application
// -----------------------------
// The TX interval is controlled using a current and a next variable.
// When the next variable is changed a state change will occur that makes the new interval active.
uint32_t currentInterval = REGULAR_INTERVAL;  ///< current interval value milliseconds
uint32_t nextInterval = REGULAR_INTERVAL;     ///< next interval value in milliseconds

PE1MEW_EEPROM_memory epromMemory;


/// \brief setting up the node
void setup()
{
  // Setup watchdog
  MCUSR = 0;
  wdt_disable();
  
  // Start Particle sensor
#if defined(SENSOR_BMP280)  /// \todo modify sensor to board specific
  my_sds.begin(MOSI,MISO);  // TX and RX port that support softserial on a marvin
#elif
  my_sds.begin(9,8);        // TX and RX port that support softserial on a Arduino Leonardo
#endif

  // Start serial ports
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;
  
#if defined(SENSOR_BMP280)
  Wire.begin();
  /// Start the air sensor
  bmx280.begin();
  
  if (bmx280.isBME280()){
    Serial.println(F("BME280"));  
  }else{
    Serial.println(F("BMP280"));
  }
  //reset sensor to default parameters.
  bmx280.resetToDefaults();
  
  //by default sensing is disabled and must be enabled by setting a non-zero
  //oversampling setting.
  //set an oversampling setting for pressure and temperature measurements. 
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

  //if sensor is a BME280, set an oversampling setting for humidity measurements.
  if (bmx280.isBME280()){
    bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);
  }
#elif defined(SENSOR_SCD30)
  /// Start the air sensor
  airSensor.begin(); //This will cause readings to occur every interval seconds 
#elif defined(SENSOR_BMP085SHT2X)
  /// Start the bmp sensor
  bmp.begin();
#endif
  
  // install callback function for downlink messages
  ttn.onMessage(message);

  // Reset LoRaWAN mac and enable ADR
  ttn.reset(true);

  // Tell the world that we startup TTN connectivity
  debugSerial.println(F("-- PERSONALIZE"));

  // Now personalize the LoRaWAN connection
#if defined(OTAA)
  ttn.join(appEui, appKey);
#elif defined(ABP)
  ttn.personalize(devAddr, nwkSKey, appSKey);
#endif
   
  // Tell the world the result of our work so far
  debugSerial.println(F("-- STATUS"));
  ttn.showStatus();

  // clear running averages
  p25Buffer.clear(); // explicitly start clean
  p10Buffer.clear(); // explicitly start clean

  nextInterval = epromMemory.readTXInterval();
  debugSerial.print(F("Interval read from EEPROM: "));
  debugSerial.println(nextInterval);
  
  // When next interval is 0; software is new installed and shall be setup properly
  if(nextInterval < MINIMUM_INTERVAL)
  {
    debugSerial.println(F("EEProm memory initialized and interval set to REGULAR_INTERVAL."));
    nextInterval = REGULAR_INTERVAL;
    epromMemory.writeTXInterval(nextInterval);
  }
  
  debugSerial.print("Interval set to: " + String(currentInterval/1000));
  debugSerial.println(F(" Seconds"));    
}

/// \brief main loop of the program
void loop()
{
  if(currentInterval != nextInterval)
  {
    debugSerial.print("Next interval set to: " + String(nextInterval/1000));
    debugSerial.println(F(" Seconds"));
    epromMemory.writeTXInterval(nextInterval); // Store interval for reuse after restart.

#if defined(SENSOR_SCD30)
    // Update SCD30 to new continous measurement interval 
    airSensor.setMeasurementInterval(nextInterval/10000);  
#endif
  }
  currentInterval = nextInterval;
  
  // Tell the world we are starting our loop
  debugSerial.println(F("-- LOOP"));
  
  // wakeup the SDS010
  my_sds.newwakeup();
       
  // Wait to allow the flow of air trough the SD010
  delay(PARTICLE_PREFLOW_DURATION);
  
  // Set our housekeeping value to false as we are not ready
  measurementComplete = false;
  
  // wait until the measurement of particles is ready
  while(!measurementComplete)
  {
    // Tell the world we are measuring particles
    debugSerial.println(F("Reading particle sensor... "));

    // clear running averages
    p25Buffer.clear(); // explicitly start clean
    p10Buffer.clear(); // explicitly start clean

    // We will take 5 measurements
    while(!p25Buffer.bufferIsFull() && !p10Buffer.bufferIsFull())
    {
      // read sensor value from soft serial port
      error = my_sds.read(&p25SensorValue,&p10SensorValue);
      // when all data is received process it.
      if (!error) 
      {
        // Show the world our intermediate results
        debugSerial.println("P2.5: "+String(p25SensorValue)+", P10: "+String(p10SensorValue));

        // As for unknown reasons negative values appear in the sensor values
        // we only store positive values to the buffers
        if(p25SensorValue > 0)
        {
          p25Buffer.addValue(p25SensorValue);
        }
        if(p10SensorValue > 0)
        {
          p10Buffer.addValue(p10SensorValue);
        }
      }
      // wait 100 mS to take a breath
      delay(100);
    }

    // Tell the world the result of our work.
    debugSerial.println( "P2.5 min/avg/max: "+String(p25Buffer.getMinInBuffer())+" / "+String(p25Buffer.getAverage())+" / "+String(p25Buffer.getMaxInBuffer())+
                        ", P10 min/avg/max: "+String(p10Buffer.getMinInBuffer())+" / "+String(p10Buffer.getAverage())+" / "+String(p10Buffer.getMaxInBuffer()));
    measurementComplete = true;
  }

  // Put SDS011 to sleep
  my_sds.sleep();

#if defined(SENSOR_BMP280)
  bool bmx280ready = true;
  
  //start a measurement
  if (!bmx280.measure()){
    bmx280ready = false;
  }

  //wait for the measurement to finish
  int waitCounter = 5;
  do{
    delay(100);
    if(0 == waitCounter++){
      bmx280ready = false;
      break;
    }
  } while (!bmx280.hasValue());

  if(bmx280ready){
    pressure = (float)bmx280.getPressure()/100;
    temperature = bmx280.getTemperature();
  }

#elif defined(SENSOR_SCD30)
  getSensorValues(temperature, humidity, gas, currentInterval);
#elif defined(SENSOR_BMP085SHT2X)
  pressure = (float)bmp.readPressure()/100;
  humidity = getHumidity(humidity);
  temperature = getTemperature(temperature);
//  dewpoint = SHT2x.GetDewPoint();
//  light = lightMeter.readLightLevel();
#endif

#if defined(SENSOR_BMP280)
  debugSerial.print(F("Barometric pressure: "));
  debugSerial.print(pressure);
  debugSerial.println(F(" Pascal."));
#elif defined(SENSOR_SCD30)
  debugSerial.print(F("CO2: "));
  debugSerial.print(gas);
  debugSerial.println(F(" ppm."));
  debugSerial.print(F("Relative humidity: "));
  debugSerial.print(humidity);
  debugSerial.println(F(" %RH."));
#elif defined(SENSOR_BMP085SHT2X)
  debugSerial.print(F("Barometric pressure: "));
  debugSerial.print(pressure);
  debugSerial.println(F(" Pascal."));
  debugSerial.print(F("Relative humidity: "));
  debugSerial.print(humidity);
  debugSerial.println(F(" %RH."));
#endif
  debugSerial.print(F("Temperature: "));
  debugSerial.print(temperature);
  debugSerial.println(F(" C."));
  
  // Reset the cayenne object
  lpp.reset();

  // add sensor values to cayenne data package
  lpp.addAnalogInput(LPP_CH_VCCVOLTAGE, (float)ttn.getVDD()/1000.0);
  lpp.addTemperature(LPP_CH_TEMPERATURE, temperature);
  lpp.addLuminosity(LPP_CH_PARTICLE_25, p25Buffer.getAverage());
  lpp.addLuminosity(LPP_CH_PARTICLE_10, p10Buffer.getAverage());
  lpp.addAnalogOutput(LPP_CH_SET_INTERVAL, (float)currentInterval/1000);

#if defined(SENSOR_BMP280)
  lpp.addBarometricPressure(LPP_CH_BAROMETER, pressure);
#elif defined(SENSOR_SCD30)
  lpp.addLuminosity(LPP_CH_GAS_CO2, gas);
  lpp.addRelativeHumidity(LPP_CH_HUMIDITY, humidity);
#elif defined(SENSOR_BMP085SHT2X)
  lpp.addBarometricPressure(LPP_CH_BAROMETER, pressure);
  lpp.addRelativeHumidity(LPP_CH_HUMIDITY, humidity);
#endif

  // Send it off
  ttn.sendBytes(lpp.getBuffer(), lpp.getSize(), APPLICATION_PORT_CAYENNE);

  // wait for next activity
  delay(currentInterval);
} // END main()



//bool getParticleValues(float &temp,float &hum, float &co2, uint32_t &interval)
//{
//  uint32_t counter = interval/10000*2;
//  bool returnValue { false };
//  
//  while(counter > 0)
//  {
//    debugSerial.print(F("Access air sensor attempts left: "));
//    debugSerial.println(counter);
//    
//    if (airSensor.dataAvailable())
//    {
//      debugSerial.println(F("Reading air sensor... "));
//      co2 = (float)airSensor.getCO2();
//      temp = airSensor.getTemperature();
//      hum = (float)airSensor.getHumidity();
//      counter = 0;
//      returnValue = true;
//    }
//    else
//    {
//      delay(1000);
//      counter--;
//    }
//  }
//  return returnValue;
//}


/// \brief function to handle incoming message
/// \todo take downlink message to change interval
void message(const uint8_t *payload, size_t size, port_t port)
{
  debugSerial.println(F("-- MESSAGE"));
  debugSerial.print("Received " + String(size) + " bytes on port " + String(port) + ":");

  switch(port)
  {
    case 99:
      if(payload[0] == LPP_CH_SET_INTERVAL)
      {
        uint32_t tempValue = 0;
        tempValue |= payload[1] << 8;
        tempValue |= payload[2];
        nextInterval = tempValue * 10;
        // prevent interval setting below 30 seconds.
        if(nextInterval < MINIMUM_INTERVAL)
        {
          nextInterval = MINIMUM_INTERVAL;
        }
        debugSerial.print("Interval set: " + String(nextInterval/1000));
        debugSerial.println(F(" seconds"));
      }
      else
      {
        debugSerial.println(F("Wrong downlink message."));
      }
      break;
 
    case 2: // Command from console
      if(payload[0] == 0x01){
        debugSerial.println(F("Remote reset initiated"));
        remoteReset();
      }
      else
      {
        debugSerial.println(F("Unknown command"));
      }
      break;

    default:
      {
        for (int i = 0; i < (int)size; i++)
        {
          debugSerial.print(" " + String(payload[i]));
        }
        debugSerial.println();
      }
      break;
  }
}

#if defined(SENSOR_SCD30)
/// \brief Read all sensors from SCD30
/// Based on the interval set the number of attempts to read the sensor is calculated.
/// \param temp Address of variabele where temperature measurement result shall be stored.
/// \param hum Address of variabele where humidity measurement result shall be stored.
/// \param temp Address of variabele where co2 measurement result shall be stored.
/// \param interval Address of variabele where interval time can be retrieved.
/// \return true: successfull acquisition of sensor values, false: failed to acquire sensor values from SCD30
bool getSensorValues(float &temp,float &hum, float &co2, uint32_t &interval)
{
  uint32_t counter = interval/10000*2;
  bool returnValue { false };
  
  while(counter > 0)
  {
    debugSerial.print(F("Access air sensor attempts left: "));
    debugSerial.println(counter);
    
    if (airSensor.dataAvailable())
    {
      debugSerial.println(F("Reading air sensor... "));
      co2 = (float)airSensor.getCO2();
      temp = airSensor.getTemperature();
      hum = (float)airSensor.getHumidity();
      counter = 0;
      returnValue = true;
    }
    else
    {
      delay(1000);
      counter--;
    }
  }
  return returnValue;
}

#elif defined(SENSOR_BMP085SHT2X)

/// \brief read temperature from sensor 
/// This function prevents bogous readings from sensors.
/// \param oldTemp Last read temperature value.
/// \return new temperature from sensor.
float getTemperature(float oldTemp)
{
  float newTemp = oldTemp;
  float tempTemp { 0.0 };
  int i { 0 };

  while( i < MEASUREMENTS)
  {
    tempTemp = SHT2x.GetTemperature();
    
    if((tempTemp < MAXIMUM_TEMPERATURE) && (tempTemp > MINIMUM_TEMPERATURE))
    {
      i = MEASUREMENTS;
      newTemp = tempTemp;
    }
    else
    {
      i++;
    }
  }
  return newTemp;
}

/// \brief read humidity from sensor 
/// This function prevents bogous readings from sensors.
/// \param oldHumid Last read humidity value.
/// \return new humidity from sensor.
float getHumidity(float oldHumid)
{
  float newHumid = oldHumid;
  float tempHumid = 0.0;
  int i = 0;

  while( i < MEASUREMENTS)
  {
    tempHumid = SHT2x.GetHumidity();
    
    if((tempHumid < MAXIMUM_HUMIDITY) && (tempHumid > 0.0))
    {
      i = MEASUREMENTS;
      newHumid = tempHumid;
    }
    else
    {
      i++;
    }
  }
  return newHumid;
}
#endif

// \brief reset within 4 seconds while indication on led that reset is started.
void remoteReset(void){
  wdt_enable(WDTO_4S);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}
