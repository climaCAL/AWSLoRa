void printLine()
{
  for (byte i = 0; i < 80; i++)
  {
    DEBUG_PRINT("-");
  }
  DEBUG_PRINTLN();
}

void printTab(byte _times)
{
  for (byte i = 0; i < _times; i++)
  {
    DEBUG_PRINT("\t");
  }
}

// Print user-defined beacon settings
void printSettings()
{
  printLine();
  DEBUG_PRINTLN("Current Settings");
  printLine();
  DEBUG_PRINT("sampleInterval: ");    printTab(1);  DEBUG_PRINTLN(sampleInterval);
  DEBUG_PRINT("averageInterval: ");   printTab(1);  DEBUG_PRINTLN(averageInterval);
  DEBUG_PRINT("transmitInterval: ");  printTab(1);  DEBUG_PRINTLN(transmitInterval);
  DEBUG_PRINT("retransmitCounter: "); printTab(1);  DEBUG_PRINTLN(retransmitCounter);
  DEBUG_PRINT("retransmitLimit: ");   printTab(1);  DEBUG_PRINTLN(retransmitLimit);
  DEBUG_PRINT("gnssTimeout: ");       printTab(2);  DEBUG_PRINTLN(gnssTimeout);
  DEBUG_PRINT("loggingMode: ");       printTab(2);  DEBUG_PRINTLN(loggingMode);
  DEBUG_PRINT("batteryCutoff: ");     printTab(2);  DEBUG_PRINTLN(batteryCutoff);
  DEBUG_PRINT("resetFlag: ");         printTab(2);  DEBUG_PRINTLN(resetFlag);
  DEBUG_PRINT("freeRam(): ");         printTab(2);  DEBUG_PRINTLN(freeRam());
  printLine();

}

void printTimers()
{
  printLine();
  DEBUG_PRINTLN("Function Execution Timers");
  printLine();
  DEBUG_PRINT("battery: ");         printTab(1);  DEBUG_PRINTLN(timer.readBattery);
  DEBUG_PRINT("readRtc: ");         printTab(1);  DEBUG_PRINTLN(timer.readRtc);
  DEBUG_PRINT("configMicroSd: ");   printTab(1);  DEBUG_PRINTLN(timer.configMicroSd);
  DEBUG_PRINT("writeMicroSd: ");    printTab(1);  DEBUG_PRINTLN(timer.writeMicroSd);
  DEBUG_PRINT("readBme280: ");      printTab(1);  DEBUG_PRINTLN(timer.readBme280);
  DEBUG_PRINT("readLsm303: ");      printTab(1);  DEBUG_PRINTLN(timer.readLsm303);
  
  DEBUG_PRINT("readDFRWS: ");       printTab(1);  DEBUG_PRINTLN(timer.readDFRWS);
  
  DEBUG_PRINT("readGnss: ");        printTab(1);  DEBUG_PRINTLN(timer.readGnss);
  DEBUG_PRINT("transmitData: ");    printTab(1);  DEBUG_PRINTLN("0");
  DEBUG_PRINT("lora TX: ");         printTab(1);  DEBUG_PRINTLN(timer.lora);
  DEBUG_PRINT("freeRam(): ");       printTab(1);  DEBUG_PRINTLN(freeRam());

  printLine();
}

// Print contents of union/structure storing Mobile Originated (MO) SBD message data
void printMoSbd()
{
  printLine();
  DEBUG_PRINTLN("LoRa Message Data");
  printLine();

  DEBUG_PRINT("unixtime:");         printTab(2);  DEBUG_PRINTLN(LoRaMessage.unixtime);
  DEBUG_PRINT("temperatureInt:");   printTab(2);  DEBUG_PRINTLN(LoRaMessage.temperatureInt);
  DEBUG_PRINT("humidityInt:");      printTab(2);  DEBUG_PRINTLN(LoRaMessage.humidityInt);
  DEBUG_PRINT("pressureExt:");      printTab(2);  DEBUG_PRINTLN(LoRaMessage.pressureExt);
  DEBUG_PRINT("temperatureExt:");   printTab(2);  DEBUG_PRINTLN(LoRaMessage.temperatureExt);
  DEBUG_PRINT("humidityExt:");      printTab(2);  DEBUG_PRINTLN(LoRaMessage.humidityExt);
  DEBUG_PRINT("solar:");            printTab(3);  DEBUG_PRINTLN(LoRaMessage.solar);
  DEBUG_PRINT("pitch:");            printTab(3);  DEBUG_PRINTLN(LoRaMessage.pitch);
  DEBUG_PRINT("roll:");             printTab(3);  DEBUG_PRINTLN(LoRaMessage.roll);
  DEBUG_PRINT("windSpeed:");        printTab(2);  DEBUG_PRINTLN(LoRaMessage.windSpeed);
  DEBUG_PRINT("windDirection:");    printTab(2);  DEBUG_PRINTLN(LoRaMessage.windDirection);
  DEBUG_PRINT("windGustSpeed:");    printTab(2);  DEBUG_PRINTLN(LoRaMessage.windGustSpeed);
  DEBUG_PRINT("windGustDirection:");  printTab(1);  DEBUG_PRINTLN(LoRaMessage.windGustDirection);
  DEBUG_PRINT("latitude:");         printTab(2);  DEBUG_PRINTLN(LoRaMessage.latitude);
  DEBUG_PRINT("longitude:");        printTab(2);  DEBUG_PRINTLN(LoRaMessage.longitude);
  DEBUG_PRINT("satellites:");       printTab(2);  DEBUG_PRINTLN(LoRaMessage.satellites);
  //Yh 18dec2023 (retiré): DEBUG_PRINT("hdop:");             printTab(3);  DEBUG_PRINTLN(LoRaMessage.hdop);
  DEBUG_PRINT("hauteurNeige:");     printTab(1);  DEBUG_PRINTLN(LoRaMessage.hauteurNeige);
  DEBUG_PRINT("voltage:");          printTab(2);  DEBUG_PRINTLN(LoRaMessage.voltage);
  DEBUG_PRINT("transmitDuration:"); printTab(1);  DEBUG_PRINTLN(LoRaMessage.transmitDuration);
  DEBUG_PRINT("transmitStatus:");   printTab(2);  DEBUG_PRINTLN(LoRaMessage.transmitStatus);
  DEBUG_PRINT("iterationCounter:"); printTab(1);  DEBUG_PRINTLN(LoRaMessage.iterationCounter);

  printLine();

}

// Print contents of union/structure
void printMoSbdHex()
{
  DEBUG_PRINTLN("MO-SBD Union/structure ");
  printLine();
  char tempData[16];
  DEBUG_PRINTLN("Byte\tHex");
  for (int i = 0; i < sizeof(LoRaMessage); ++i)
  {
    sprintf(tempData, "%d\t0x%02X", i, LoRaMessage.bytes[i]);
    DEBUG_PRINTLN(tempData);
  }
  printLine();
}

// Print contents of transmit buffer
// void printMoSbdBuffer()
// {
//   printLine();
//   DEBUG_PRINTLN("MO-SBD Transmit buffer");
//   printLine();
//   char tempData[16];
//   DEBUG_PRINTLN("Byte\tHex");
//   for (int i = 0; i < moSbdBufferSize; ++i)
//   {
//     sprintf(tempData, "%d\t0x%02X", i, moSbdBuffer[i]);
//     DEBUG_PRINTLN(tempData);
//   }
// }

// Print contents of transmit buffer
// void printMtSbdBuffer()
// {
//   printLine();
//   DEBUG_PRINTLN("MT-SBD Transmit buffer");
//   printLine();
//   // Print contents of mtSbdBuffer in hexadecimal
//   char tempData[16];
//   DEBUG_PRINTLN("Byte\tHex");
//   for (int i = 0; i < mtSbdBufferSize; ++i)
//   {
//     sprintf(tempData, "%d\t0x%02X", i, mtSbdBuffer[i]);
//     DEBUG_PRINTLN(tempData);
//   }
// }

// Function to print available 32K SRAM memory
extern "C" char *sbrk(int i);
int freeRam()
{
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}
