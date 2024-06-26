// Calculate statistics
void calculateStats()
{
  // Write data to union
  if (isnan(temperatureIntStats.average()))
    LoRaMessage.temperatureInt = 0;
  else
    LoRaMessage.temperatureInt = (int8_t)(temperatureIntStats.average());          // Mean internal temperature (°C) - integer value

  if (isnan(humidityIntStats.average()))
    LoRaMessage.humidityInt    = 0;
  else
    LoRaMessage.humidityInt    = (uint8_t)(humidityIntStats.average());            // Mean internal humidity (%) - integer value
  
  if (isnan(pressureExtStats.average()))
  LoRaMessage.pressureExt      = 0;
  else
    LoRaMessage.pressureExt    = (pressureExtStats.average()     - 400) * 100;   // Mean internal pressure (hPa)

  if (isnan(temperatureExtStats.average()))
    LoRaMessage.temperatureExt = 0;
  else
    LoRaMessage.temperatureExt = temperatureExtStats.average()   * 100;          // Mean external temperature (°C)

  if (isnan(humidityExtStats.average()))
    LoRaMessage.humidityExt    = 0;
  else
    LoRaMessage.humidityExt    = humidityExtStats.average()      * 100;          // Mean external humidity (%)

  if (isnan(solarStats.average()))
    LoRaMessage.solar          = 0;
  else
    LoRaMessage.solar          = (uint16_t)(log10(solarStats.average())*facteurMultLumino);  // Mean solar irradiance (W m-2), 3800*log(lux)

  if (isnan(batteryStats.average()))
    LoRaMessage.voltage        = 0;
  else
    LoRaMessage.voltage        = batteryStats.average()          * 100;          // Mean battery voltage (V)

  if (isnan(hauteurNeige.average()))
    LoRaMessage.hauteurNeige   = 0;
  else
    LoRaMessage.hauteurNeige   = (uint16_t)(hauteurNeige.average());

  // Calculate mean wind speed and direction vectors
  windVectors();

  // Clear all statistics objects
  clearStats();

  // Clear wind gust speed and direction maximums
  windGustSpeed = 0;
  windGustDirection = 0;
  windDirectionSector = 0;

  // Write location data to union (will reuse previously stored data if readGNSS did not occur during this cycle)
  LoRaMessage.latitude = latitude * 1000000;
  LoRaMessage.longitude = longitude * 1000000;
  LoRaMessage.satellites = satellites;
}

// Clear statistics objects
void clearStats()
{
  batteryStats.clear();
  temperatureIntStats.clear();
  humidityIntStats.clear();
  pressureExtStats.clear();
  temperatureExtStats.clear();
  solarStats.clear();
  humidityExtStats.clear();
  windSpeedStats.clear();
  uStats.clear();
  vStats.clear();
  hauteurNeige.clear();
}

// Print statistics
void printStats()
{
  printLine();
  DEBUG_PRINTLN(F("Statistics"));
  printLine();
  DEBUG_PRINT("Datetime: ");      printTab(1);  printDateTime();
  DEBUG_PRINT(F("Voltage"));      printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(batteryStats.count());            printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(batteryStats.minimum());          printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(batteryStats.maximum());          printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(batteryStats.average());
  DEBUG_PRINT(F("Temp Int"));                                                   printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(temperatureIntStats.count());     printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(temperatureIntStats.minimum());   printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(temperatureIntStats.maximum());   printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(temperatureIntStats.average());
  DEBUG_PRINT(F("Humidity Int"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(humidityIntStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(humidityIntStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(humidityIntStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(humidityIntStats.average());
  DEBUG_PRINT(F("Pressure Ext"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(pressureExtStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(pressureExtStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(pressureExtStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(pressureExtStats.average());
  DEBUG_PRINT(F("Temp Ext"));                                                   printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(temperatureExtStats.count());     printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(temperatureExtStats.minimum());   printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(temperatureExtStats.maximum());   printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(temperatureExtStats.average());
  DEBUG_PRINT(F("Humidity Ext"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(humidityExtStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(humidityExtStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(humidityExtStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(humidityExtStats.average());
  
  DEBUG_PRINT(F("Solar"));                                                      printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(solarStats.count());              printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(solarStats.minimum());            printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(solarStats.maximum());            printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(solarStats.average());

  //Hauteur de neige:
  DEBUG_PRINT(F("HauteurNeige"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(hauteurNeige.count());            printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(hauteurNeige.minimum());          printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(hauteurNeige.maximum());          printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(hauteurNeige.average());
  
  DEBUG_PRINT(F("Wind speed"));   printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(windSpeedStats.count());          printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(windSpeedStats.minimum());        printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(windSpeedStats.maximum());        printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(windSpeedStats.average());
  DEBUG_PRINT(F("vn"));                                                         printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(vStats.count());                  printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(vStats.minimum());                printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(vStats.maximum());                printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(vStats.average());
  DEBUG_PRINT(F("ve"));                                                         printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(uStats.count());                  printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(uStats.minimum());                printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(uStats.maximum());                printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(uStats.average());
  DEBUG_PRINT(F("Wind gust speed: "));      printTab(1);  DEBUG_PRINTLN(windGustSpeed);
  DEBUG_PRINT(F("Wind gust direction: "));  printTab(1);  DEBUG_PRINTLN(windGustDirection);


}
