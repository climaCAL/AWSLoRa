// ----------------------------------------------------------------------------
// Utility function to detect I2C sensor lockup 
// ----------------------------------------------------------------------------
bool scanI2CbusFor(uint8_t lookForAddress) {
  bool retCode = false;
  Wire.beginTransmission(lookForAddress);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {   /*if I2C device found*/
    // Serial.print("I2C device found at address 0x");/*print this line if I2C device found*/
    // if (lookForAddress<16) {
    //   Serial.print("0");
    // }
    // Serial.println(lookForAddress,HEX);
    retCode = true;
  }
  return retCode;
}

// ----------------------------------------------------------------------------
// Adafruit BME280 Temperature Humidity Pressure Sensor
// https://www.adafruit.com/product/2652
// ----------------------------------------------------------------------------
void configureBme280(BME_PERIPH_ID devID)
{
  DEBUG_PRINT("Info - Initializing BME280 ("+String(devID)+")...");

  bool retCode = false;

  if (devID < 2) {

    //Set bme280 device's address, default is exterior (0x77)
    uint8_t devAddr = BME280_EXT;
    if (devID == BMEINT) devAddr = BME280_INT;

    if (scanI2CbusFor(devAddr)) {  // -- TBV if this works
      if (bme280.begin(devAddr))
      {
        DEBUG_PRINTLN("success!");
        retCode = true;
      }
      if (!retCode) {
        DEBUG_PRINTLN("failed!");
      }
    } else {
      DEBUG_PRINT("bme280 init: no answer from id 0x");
      DEBUG_PRINTLN_HEX(devAddr);
    }
    online.bme280[devID] = retCode;
  } else {
    DEBUG_PRINTLN("bme280 init: wrong devID!");
  }
    

}

// Read BME280
void readBme280(BME_PERIPH_ID devID)
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  if (devID == BMEEXT || devID == BMEINT) {

    // Initialize sensor
    configureBme280(devID);

    // Check if sensor initialized successfully
    if (online.bme280[devID])  {
      DEBUG_PRINT("Info - Reading BME280 ("+String(devID)+")...");

      myDelay(250);

      // Read sensor data
      if (devID == BMEEXT) {  // AKA as the external one
        temperatureExt  = tempBmeEXT_CF * bme280.readTemperature() + tempBmeEXT_Offset;
        uint16_t humExt = humBmeEXT_CF * bme280.readHumidity() + humBmeEXT_Offset;

        if (humExt >= 100) {
          humidityExt = 100;
        } else {
          humidityExt = humExt;
        }
        
        // Add to statistics object
        temperatureExtStats.add(temperatureExt );
        humidityExtStats.add(humidityExt);
        #if CALIBRATE
          DEBUG_PRINT("\tTemperatureExt: "); DEBUG_PRINT(temperatureExt); DEBUG_PRINTLN(" C");
          DEBUG_PRINT("\tHumidityExt: "); DEBUG_PRINT(humidityExt); DEBUG_PRINTLN("%");
        #endif

      }
      if (devID == BMEINT) {  // AKA as the internal one
        // Read sensor data
        temperatureInt = tempImeINT_CF * bme280.readTemperature() + tempBmeINT_Offset ;
        uint16_t humInt =  humImeINT_CF * bme280.readHumidity() + humBmeINT_Offset; // no need of correction
        pressureInt = bme280.readPressure() / 100.0F;

        if (humInt >= 100) {
          humidityInt = 100;
        } else {
          humidityInt = humInt;
        }

        // Add to statistics object
        temperatureIntStats.add(temperatureInt);
        humidityIntStats.add(humidityInt);
        pressureIntStats.add(pressureInt);

        #if CALIBRATE
          DEBUG_PRINT("\tTemperatureInt: "); DEBUG_PRINT(temperatureInt); DEBUG_PRINTLN(" C");
          DEBUG_PRINT("\tHumidityInt: "); DEBUG_PRINT(humidityInt); DEBUG_PRINTLN("%");
          DEBUG_PRINT("\tPressure(Int): "); DEBUG_PRINT(pressureInt); DEBUG_PRINTLN(" kPa");
        #endif
      }
      DEBUG_PRINTLN("done.");
    }
    else
    {
      DEBUG_PRINTLN("Warning - BME280 offline!");
    }
  } else {
    DEBUG_PRINTLN("ERROR - bme280 read: wrong devID!");
  }
  // Stop the loop timer
  timer.readBme280 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Adafruit VEML7700 Lux Meter
// ----------------------------------------------------------------------------
void configureVEML7700(Adafruit_VEML7700 &veml)
{
  bool retCode = false;

  DEBUG_PRINT("Info - Initializing VEML7700...");
  
  if (scanI2CbusFor(vemlI2cAddr)) {
    if (veml.begin())
    {
      DEBUG_PRINTLN("success!");
      retCode = true;
      /*
      veml.setGain(VEML7700_GAIN_2);
      veml.setIntegrationTime(VEML7700_IT_200MS);
      */
    }
    else
    {
      DEBUG_PRINTLN("failed!");
    }
  } else  {
    DEBUG_PRINT("VEML7700 init: no answer from id 0x");
    DEBUG_PRINTLN_HEX(vemlI2cAddr);
  }

  online.veml7700 = retCode;

}

// Read VEML7700 (solar)
void readVeml7700()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();
   
  // Initialize sensor
  configureVEML7700(veml);
  
  // Check if sensor initialized successfully
  if (online.veml7700)
  {
    DEBUG_PRINT("Info - Reading VEML7700...");

    myDelay(250);

// Add acquisition
  int32_t soleil = veml_CF * veml.readLux() + veml_Offset; // Default = VEML_LUX_NORMAL
  
  if(soleil <= 0) {
    solar = 0;
  } else {
    solar = soleil;
  }

  solarStats.add(solar);

  DEBUG_PRINT("\tSolar: "); DEBUG_PRINT(solar); DEBUG_PRINTLN(" Lux");
    
    DEBUG_PRINTLN("done.");
  }
  else
  {
    DEBUG_PRINTLN("Warning - VEML7700 offline!");
  }

  // Stop the loop timer
  timer.readVeml7700 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Davis Instruments Temperature Humidity Sensor (Sensiron SHT31-LSS)
// ------------------------------
// Colour     Pin     Description
// ------------------------------
// Yellow    3.3V     Power
// Green     GND      Ground
// White     SCK      Clock
// Blue      SDA      Data
// ----------------------------------------------------------------------------
/*void readSht31()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINT("Info - Reading SHT31...");

  // Disable I2C bus
  Wire.end();

  // Add delay
  myDelay(100);

  // Read sensor
  temperatureExt = sht.readTemperatureC();
  humidityExt = sht.readHumidity();

  // Add to statistics object
  temperatureExtStats.add(temperatureExt);
  humidityExtStats.add(humidityExt);

  // Print debug info
  //DEBUG_PRINT("Temperature: "); DEBUG_PRINT(temperatureExt); DEBUG_PRINTLN(" C");
  //DEBUG_PRINT("Humidity: "); DEBUG_PRINT(humidityExt); DEBUG_PRINTLN("%");

  // Re-enable I2C bus
  Wire.begin();

  // Stop the loop timer
  timer.readSht31 = millis() - loopStartTime;
}
*/

// ----------------------------------------------------------------------------
// Adafruit LSM303AGR Accelerometer/Magnetomter
// https://www.adafruit.com/product/4413
// ----------------------------------------------------------------------------
void configureLsm303()
{
  DEBUG_PRINT("Info - Initializing LSM303...");

   // Yh 031823 - May I suggest to "ping" expected/defaut device I2C address first in order to test presence? Then try begin.

  // Initialize LSM303 accelerometer
  if (lsm303.begin())
  {
    online.lsm303 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    online.lsm303 = false;
    DEBUG_PRINTLN("failed!");
  }
}

void readLsm303()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Initialize accelerometer
  configureLsm303();

  // Check if sensor initialized successfully
  if (online.lsm303)
  {
    DEBUG_PRINT("Info - Reading LSM303...");

    myDelay(500);

    float xAvg = 0, yAvg = 0, zAvg = 0;
   
    // Read accelerometer data
    sensors_event_t accel;

    // Average accelerometer values
    int samplesToAverage = 30;
    for (int i = 0; i < samplesToAverage; ++i)
    {
      lsm303.getEvent(&accel);   // Read accelerometer data
      xAvg += accel.acceleration.x;
      yAvg += accel.acceleration.y;
      zAvg += accel.acceleration.z;
      delay(1);
    }
    // Calculate average
    xAvg /= samplesToAverage;
    yAvg /= samplesToAverage;
    zAvg /= samplesToAverage;

    // Calculate pitch and roll
    // Note: X-axis and Z axis swapped due to orientation of sensor when installed
    pitch = atan2(-zAvg, sqrt(yAvg * yAvg + xAvg * xAvg)) * 180 / PI;
    roll = atan2(yAvg, xAvg) * 180 / PI;

    // Write data to union
    LoRaMessage.pitch = pitch * 100;
    LoRaMessage.roll = roll * 100;

    // Add to statistics object
    //pitchStats.add();
    //rollStats.add();

    DEBUG_PRINTLN("done.");

    // Print debug info
    //DEBUG_PRINT(F("pitch: ")); DEBUG_PRINT_DEC(pitch, 2);
    //DEBUG_PRINT(F(" roll: ")); DEBUG_PRINTLN_DEC(roll, 2);

  }
  else
  {
    DEBUG_PRINTLN("Warning - LSM303 offline!");
  }

  // Stop loop timer
  timer.readLsm303 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
//  Vaisala HMP60 Humidity and Temperature Probe
// -----------------------------------------------------
// Colour     Pin     Description
// -----------------------------------------------------
// Brown      12V     Power (5 - 28V)
// White      A3      CH1: Relative humidity (0 - 2.5V)
// Blue       GND     Ground
// Black      A4      CH2: Temperature (0 - 2.5V)
// Shield     GND     Earth ground
// ----------------------------------------------------------------------------
void readHmp60()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINT("Info - Reading HMP60...");

  // Note: A startup delay of 4 s is recommended at 12 V and 2 s at 5 V
  myDelay(4000);

  // Perform analog readings
  (void)analogRead(PIN_TEMP);
  float sensorValue1 = analogRead(PIN_TEMP); // External temperature
  (void)analogRead(PIN_HUMID);
  float sensorValue2 = analogRead(PIN_HUMID); // External humidity

  // Map voltages to sensor ranges
  temperatureExt = mapFloat(sensorValue1, 0, 3103, -60, 40);  // Map temperature from 0-2.5 V to -60 to 40°C
  humidityExt = mapFloat(sensorValue2, 0, 3103, 0, 100);      // Map humidity 0-2.5 V to 0 to 100%

  // Calculate measured voltages
  float voltage1 = sensorValue1 * (3.3 / 4095.0);
  float voltage2 = sensorValue2 * (3.3 / 4095.0);

  DEBUG_PRINTLN("done.");

#if CALIBRATE
  // Print calibration data
  DEBUG_PRINT(F("temperatureExt: ")); DEBUG_PRINT(sensorValue1); DEBUG_PRINT(F(",")); DEBUG_PRINT_DEC(voltage1, 4); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(temperatureExt, 2);
  DEBUG_PRINT(F("humidityExt: ")); DEBUG_PRINT(sensorValue2); DEBUG_PRINT(F(",")); DEBUG_PRINT_DEC(voltage2, 4); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(humidityExt, 2);
#endif

  // Add to statistics object
  temperatureExtStats.add(temperatureExt);
  humidityExtStats.add(humidityExt);

  // Stop loop timer
  timer.readHmp60 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Apogee SP-212 Pyranometer
// -----------------------------------------------------
// Colour    Pin        Description
// -----------------------------------------------------
// White     ?          Positive (signal from sensor)
// Red       5V         Input Power 5-24 V DC
// Black     GND        Ground (from sensor signal and output power)
// Clear     GND        Shield/Ground
// ----------------------------------------------------------------------------
void readSp212()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINT("Info - Reading SP212...");

  // Perform analog readings
  (void)analogRead(PIN_SOLAR);
  float sensorValue = analogRead(PIN_SOLAR); // External temperature

  // Map voltages to sensor ranges
  solar = mapFloat(sensorValue, 0, 3102, 0, 2000); // Map solar irradiance from 0-2.5 V to 0 to 2000 W m^2

  // Calculate measured voltages
  float voltage = sensorValue * (3.3 / 4095.0);

  DEBUG_PRINTLN("done.");

  // Print debug info
  //DEBUG_PRINT(F("solar: ")); DEBUG_PRINT_DEC(voltage, 4); DEBUG_PRINT(F(",")); DEBUG_PRINT(sensorValue); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(solar, 2);

  // Add to statistics object
  solarStats.add(solar);

  // Stop loop timer
  timer.readSp212 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// R.M. Young Wind Monitor 5103L (4-20 mA)
// 150 Ohm 0.1% resistor
// Voltage range: 0.5995 - 2.9675 V
//
// --------------------------------------------------
// Colour     Pin       Description
// --------------------------------------------------
// Black      12V       Wind speed + (WS+)
// Red        A1        Wind speed - (WS-)
// White      12V       Wind direction + (WD+
// Green      A2        Wind direction - (WD-)
// Shield     GND       Earth ground
//
// ----------------------------------------------------------------------------
void read5103L()
{
  unsigned int loopStartTime = millis();

  DEBUG_PRINT("Info - Reading 5103L...");

  // Measure wind speed and direction
  (void)analogRead(PIN_WIND_SPEED);
  float sensorValue1 = analogRead(PIN_WIND_SPEED); // Read analog wind speed value
  (void)analogRead(PIN_WIND_DIR);
  float sensorValue2 = analogRead(PIN_WIND_DIR); // Read analog wind direction value

  // Map wind speed and direction analogue values to
  windSpeed = mapFloat(sensorValue1, 745, 3684, 0, 100); // 0-100 m/s range
  windDirection = mapFloat(sensorValue2, 745, 3684, 0, 360); // 0-360 range

  DEBUG_PRINTLN("done.");

#if CALIBRATE
  // Calculate measured voltages
  float voltage1 = sensorValue1 * (3.3 / 4095.0);
  float voltage2 = sensorValue2 * (3.3 / 4095.0);

  // Print calibration data
  DEBUG_PRINT(F("windSpeed: ")); DEBUG_PRINT_DEC(voltage1, 4); DEBUG_PRINT(F(",")); DEBUG_PRINT(sensorValue1); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(windSpeed, 2);
  DEBUG_PRINT(F("windDirection: ")); DEBUG_PRINT_DEC(voltage2, 4); DEBUG_PRINT(F(",")); DEBUG_PRINT(sensorValue2); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(windDirection, 2);
#endif

  // Check and update wind gust and direction
  if ((windSpeed > 0) && (windSpeed > windGustSpeed))
  {
    windGustSpeed = windSpeed;
    windGustDirection = windDirection;
  }

  // Calculate wind speed and direction vectors
  // For more information see:
  // http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
  float windDirectionRadians = windDirection * DEG_TO_RAD;  // Convert wind direction from degrees to radians
  float u = -1.0 * windSpeed * sin(windDirectionRadians);   // Magnitude of east-west component (u) of vector winds
  float v = -1.0 * windSpeed * cos(windDirectionRadians);   // Magnitude of north-south component (v) of vector winds

  // Write data to union
  LoRaMessage.windGustSpeed = windGustSpeed * 100;
  LoRaMessage.windGustDirection = windGustDirection * 10;

  // Add to wind statistics
  windSpeedStats.add(windSpeed);
  uStats.add(u);
  vStats.add(v);

  // Stop loop timer
  timer.read5103L = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Davis Instruments 7911 Anemometer
// ------------------------------
// Colour   Pin     Description
// ------------------------------
// Black    A1      Wind speed
// Green    A2      Wind direction
// Yellow   5V      Power
// Red      GND     Ground
// ----------------------------------------------------------------------------
void read7911()
{
  uint32_t loopStartTime = millis();

  DEBUG_PRINTLN("Info - Reading 7911...");

  // Enable pull-ups
  pinMode(PIN_WIND_SPEED, INPUT_PULLUP);

  // Attach interrupt to wind speed input pin
  attachInterrupt(PIN_WIND_SPEED, windSpeedIsr, FALLING);
  revolutions = 0;

  // Measure wind speed for 3 seconds
  while (millis() < loopStartTime + 3000);
  {
    // Do nothing
  }

  // Detach interrupt from wind speed input pin
  detachInterrupt(PIN_WIND_SPEED);

  // Disable pull-ups
  pinMode(PIN_WIND_SPEED, INPUT);

  // Calculate wind speed according to Davis Instruments formula: V = P(2.25/T)
  // V = speed in miles per hour
  // P = no. of pulses in sample period
  // T = duration of sample period in seconds
  windSpeed = revolutions * (2.25 / 3);   // Calculate wind speed in miles per hour
  windSpeed *= 0.44704;                   // Convert wind speed to metres per second

  // Enable power
  digitalWrite(PIN_SENSOR_PWR, HIGH);

  // Measure wind direction
  (void)analogRead(PIN_WIND_DIR);
  windDirection = analogRead(PIN_WIND_DIR); // Raw analog wind direction value
  windDirection = map(windDirection, 0, 4095, 0, 359); // Map wind direction to degrees (0-360°)

  // Disable power
  digitalWrite(PIN_SENSOR_PWR, LOW);

  // Correct for negative wind direction values
  if (windDirection > 360)
    windDirection -= 360;
  if (windDirection < 0)
    windDirection += 360;

  if (windSpeed == 0)
  {
    windDirection = 0.0;
  }

  // Check and update wind gust speed and direction
  if ((windSpeed > 0) && (windSpeed > windGustSpeed))
  {
    windGustSpeed = windSpeed;
    windGustDirection = windDirection;
  }

  // Calculate wind speed and direction vectors
  // http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
  float windDirectionRadians = windDirection * DEG_TO_RAD;  // Convert wind direction from degrees to radians
  float u = -1.0 * windSpeed * sin(windDirectionRadians);   // Magnitude of east-west component (u) of vector winds
  float v = -1.0 * windSpeed * cos(windDirectionRadians);   // Magnitude of north-south component (v) of vector winds

  // Write data to union
  LoRaMessage.windGustSpeed = windGustSpeed * 100;
  LoRaMessage.windGustDirection = windGustDirection * 10;

  // Add to wind statistics
  windSpeedStats.add(windSpeed);
  uStats.add(u);
  vStats.add(v);

  // Print debug info
  //DEBUG_PRINT(F("Wind Speed: ")); DEBUG_PRINTLN(windSpeed);
  //DEBUG_PRINT(F("Wind Direction: ")); DEBUG_PRINTLN(windDirection);

  // Stop loop timer
  timer.read7911 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// In-house (CAL) built of combined Wind Sensor
// DFRobot WindSensor comprises the following 2:
//    RS485 Wind Speed Transmitter (SEN0483) : https://wiki.dfrobot.com/RS485_Wind_Speed_Transmitter_SKU_SEN0483
//    RS485 Wind Direction Transmitter (SEN0482) : https://wiki.dfrobot.com/RS485_Wind_Direction_Transmitter_SKU_SEN0482
//Slave ragisters map (read-only):
  /*
  0x00  MSB Angle Vent
  0x01  LSB Angle Vent
  0x02  MSB Direction Vent
  0x03  LSB Direction Vent
  0x04  MSB Vitesse Vent
  0x05  MSB Vitesse Vent
  
  * DFRobot wind direction sensor informations :
  *
  * Direction	      16 Directions Value   Angle(360°)
  * North	                0	              0° - 11.2°
  * North-northeast	      1	              11.3° - 33.7°
  * Northeast	            2	              33.8° - 56.2°
  * East-northeast	      3	              56.3° - 78.7°
  * East	                4	              78.8° - 101.2°
  * East-southeast	      5	              101.3° - 123.7°
  * Southeast	            6	              123.8° - 146.2°
  * South-southeast	      7	              146.3° - 168.7°
  * South	                8	              168.8° - 191.2°
  * South-southwest	      9	              191.3° - 213.7°
  * Southwest	            10	            213.8° - 236.2°
  * West-southwest	      11	            236.3° - 258.7°
  * West	                12	            258.8° - 281.2°
  * West-northwest	      13	            281.3° - 303.7°
  * Northwest	            14	            303.8° - 326.2°
  * North-northwest	      15	            326.3° - 348.7°
  * North	                16	            348.8° - 360°
 */
// ----------------------------------------------------------------------------
void readDFRWindSensor() 
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINTLN("Info - Reading DFRWindSensor... sensor settle time of 2sec");

  // Requires I2C bus
  Wire.begin();
  myDelay(2000);

  vent lectureVent;

  byte len = Wire.requestFrom(WIND_SENSOR_SLAVE_ADDR,0x06);  //Requesting 6 bytes from slave

  if (len != 0) {
    while (Wire.available() > 0) {
      for(int i = 0; i < 6; i++) {
        lectureVent.regMemoryMap[i] = Wire.read();
      }
    }
    lectureVent.angleVentFloat = ((lectureVent.regMemoryMap[0] << 8) + lectureVent.regMemoryMap[1])/10.0;
    lectureVent.directionVentInt = (lectureVent.regMemoryMap[2] << 8) + lectureVent.regMemoryMap[3];
    lectureVent.vitesseVentFloat = ((lectureVent.regMemoryMap[4] << 8) + lectureVent.regMemoryMap[5])/10.0;

    windDirection = lectureVent.angleVentFloat;
    windDirectionSector = lectureVent.directionVentInt;
    windSpeed = lectureVent.vitesseVentFloat;

    char smallMsg[48]={0};  //Temps buffer
    sprintf(smallMsg,"%x %x %x %x %x %x",lectureVent.regMemoryMap[0],lectureVent.regMemoryMap[1],lectureVent.regMemoryMap[2],lectureVent.regMemoryMap[3],lectureVent.regMemoryMap[4],lectureVent.regMemoryMap[5]);
    DEBUG_PRINT(F("\t*RAW* readings: ")); DEBUG_PRINTLN(smallMsg);
    
  } else {
    windDirection = 0.0;
    windDirectionSector = 0;
    windSpeed = 0; 
  }

  if (windSpeed == 0)
  {
    windDirection = 0.0;
    windDirectionSector = 0;
  }

  // Check and update wind gust speed and direction
  if ((windSpeed > 0) && (windSpeed > windGustSpeed))
  {
    windGustSpeed = windSpeed;
    windGustDirection = windDirection;
  }

  // Write data to union (LoRa)
  LoRaMessage.windSpeed = windSpeed * 100;
  LoRaMessage.windDirection = windDirection;

  // Calculate wind speed and direction vectors
  // http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
  float windDirectionRadians = windDirection * DEG_TO_RAD;  // Convert wind direction from degrees to radians
  float u = -1.0 * windSpeed * sin(windDirectionRadians);   // Magnitude of east-west component (u) of vector winds
  float v = -1.0 * windSpeed * cos(windDirectionRadians);   // Magnitude of north-south component (v) of vector winds

  // Write data to union
  LoRaMessage.windGustSpeed = windGustSpeed * 100;
  LoRaMessage.windGustDirection = windGustDirection*10; 
  
  // Add to wind statistics
  windSpeedStats.add(windSpeed);
  uStats.add(u);
  vStats.add(v);

  // Print debug info
  DEBUG_PRINT(F("\tWind Speed: ")); DEBUG_PRINTLN(windSpeed);
  DEBUG_PRINT(F("\tWind Direction: ")); DEBUG_PRINTLN(windDirection);
  DEBUG_PRINT(F("\tWind Dir. Sector: ")); DEBUG_PRINTLN(windDirectionSector);

    // Stop the loop timer
  timer.readDFRWS = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Anemometer model VMS-3000-FSJT-NPNR
// reference: https://www.makerfabs.com/wiki/index.php?title=Anemometer
// ----------------------------------------------------------------------------
void readVMS3000()  
{
  uint32_t loopStartTime = millis();

  DEBUG_PRINTLN("Info - Reading VMS3000...");

  // Configure pin mode
  pinMode(PIN_WIND_SPEED, INPUT);

  // Attach interrupt to wind speed input pin
  attachInterrupt(PIN_WIND_SPEED, windSpeedIsr, FALLING);
  revolutions = 0;

  //Yh: 1er mai 23: Pourquoi ne pas utiliser myDelay(3000); à la place? 
  //  cela éviterais le risque avec le WDT, non?

  // Measure wind speed for 3 seconds
  // while (millis() < loopStartTime + 3000);
  // {
  //   // Do nothing
  // }

  //DEBUG_PRINTLN("capturing..."+String(millis()/1000));
  myDelay(3000);   //Yh 0805 - works!
  //DEBUG_PRINTLN("... done! "+String(millis()/1000));

  // Detach interrupt from wind speed input pin
  detachInterrupt(PIN_WIND_SPEED);

  // Calculate wind speed according to Davis Instruments formula: V = P(1.75/T)
  // V = speed in miles per hour
  // P = no. of pulses in sample period
  // T = duration of sample period in seconds
  windSpeed = revolutions * 1.75/20.0;   // Calculate wind speed in metres per second

  if (windSpeed == 0)
  {
    windDirection = 0.0;
  }

    // Check and update wind gust speed and direction
  if ((windSpeed > 0) && (windSpeed > windGustSpeed))
  {
    windGustSpeed = windSpeed;
    windGustDirection = windDirection;
  }

  // Calculate wind speed and direction vectors
  // http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
  float windDirectionRadians = windDirection * DEG_TO_RAD;  // Convert wind direction from degrees to radians
  float u = -1.0 * windSpeed * sin(windDirectionRadians);   // Magnitude of east-west component (u) of vector winds
  float v = -1.0 * windSpeed * cos(windDirectionRadians);   // Magnitude of north-south component (v) of vector winds

  // Write data to union
  LoRaMessage.windGustSpeed = windGustSpeed * 100;
  LoRaMessage.windGustDirection = windGustDirection * 10;

  // Add to wind statistics
  windSpeedStats.add(windSpeed);
  uStats.add(u);
  vStats.add(v);

  // Write data to union
  LoRaMessage.windSpeed = windSpeed * 100;
  LoRaMessage.windDirection = 1;  //NA with VMS3000 alone, but required? Yh 0805(may)

  // Print debug info
  DEBUG_PRINT(F("Wind Speed: ")); DEBUG_PRINTLN(windSpeed);
  //DEBUG_PRINT(F("Wind Direction: ")); DEBUG_PRINTLN(windDirection);

  // Stop loop timer
  timer.readVMS3K = millis() - loopStartTime;
}

// Interrupt service routine (ISR) for wind speed measurement
// for Davis Instruments 7911 anemometer
void windSpeedIsr()
{
  if ( digitalRead(PIN_WIND_SPEED) == LOW )
    revolutions++;
}

// Calculate mean wind speed and direction from vector components
// For more information see:
// http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
void windVectors()
{
  // Calculate resultant mean wind speed
  float rvWindSpeed = sqrt(sq(uStats.average()) + sq(vStats.average()));

  DEBUG_PRINT("uStats.average(): "); printTab(1); DEBUG_PRINTLN(uStats.average());
  DEBUG_PRINT("vStats.average(): "); printTab(1); DEBUG_PRINTLN(vStats.average());

  // Calculate resultant mean wind direction
  float rvWindDirection = atan2(-1.0 * uStats.average(), -1.0 * vStats.average());
  rvWindDirection *= RAD_TO_DEG;  // Convert from radians to degrees

  DEBUG_PRINT("rvWindSpeed: "); printTab(2); DEBUG_PRINTLN(rvWindSpeed);
  DEBUG_PRINT("rvWindDirection: "); printTab(1); DEBUG_PRINTLN(rvWindDirection);

  // To do: Check if necessary
  if (rvWindDirection < 0)
    rvWindDirection += 360;

  // Zero wind direction if wind speed is zero
  // Note: atan2 can be undefined if u and v vectors are zero
  if (rvWindSpeed == 0)
    rvWindDirection = 0;

  // Write data to union
  LoRaMessage.windSpeed = rvWindSpeed * 100;         // Resultant mean wind speed (m/s)
  LoRaMessage.windDirection = rvWindDirection * 10;  // Resultant mean wind direction (°)
}

// ----------------------------------------------------------------------------
// MaxBotix MB7354 HRXL-MaxSonar-WRS5
// https://www.maxbotix.com/ultrasonic_sensors/mb7354.htm
// --------------------------------------------------
// Colour    Pin    Description             Pin
// --------------------------------------------------
// White    1       Temperature Sensor      Not connected
// Orange   2       Pulse Width Output      Not connected
// Brown    3       Analog Voltage Output   Analog In
// Green    4       Ranging Start/Stop      Not connected
// Blue     5       Serial Output           Not connected
// Red      6       Vcc                     5V
// Black    7       GND                     GND
//
// ----------------------------------------------------------------------------
// Read Maxbotix distance to surface
void readMb7354()
{

}
