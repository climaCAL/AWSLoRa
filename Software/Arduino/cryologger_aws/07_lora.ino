void configureLoRa()
{
  DEBUG_PRINTLN("Info - Initializing LoRa module");  //setup LoRa transceiver module
  LoRa.setPins(PIN_RFM95_CS, PIN_RFM95_RST, PIN_RFM95_DIO0);

  bool LoRaStatus = LoRa.begin(RF95_FREQ);

  if (LoRaStatus) {
    DEBUG_PRINTLN("Info - Success Initializing LoRa module, now configuring");
    LoRa.setSpreadingFactor(RF95_SF);
    LoRa.setSignalBandwidth(RF95_BW);
    LoRa.setCodingRate4(RF95_CR);
    LoRa.setTxPower(RF95_PW);
    LoRa.setSyncWord(LoRasyncWord);
    if (RF95_CRC)
      LoRa.enableCrc();
    LoRa.onTxDone(LoRaSendCallback);  //Arduino-LoRa lib warns that onReceive and on onTxDone won't work on SAMD !!! Sh**!!
    online.lora = true;
  } else {
    DEBUG_PRINTLN("Error - Failed intializing LoRa module");
    online.lora = false;  //Houston?!
  }
  //Debug msg 4
}


void writeBuffer()
{
  iterationCounter++; // Increment iteration counter
  transmitCounter++; // Increment data transmission counter
  LoRaMessage.iterationCounter = iterationCounter; // Write message counter data to union

//  Yh - Interesting technique and need TBV if required for LoRa transmission

 // Concatenate current message with existing message(s) stored in transmit buffer
  //memcpy(moSbdBuffer + (sizeof(LoRaMessage) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), LoRaMessage.bytes, sizeof(LoRaMessage));

  // Print MO-SBD union/structure
  //printMoSbd();

  // Clear MO-SBD message union/structure
  memset(&LoRaMessage, 0x00, sizeof(LoRaMessage));
}

void LoRaTransmitData()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Wake up and begin communications
    
  DEBUG_PRINTLN("Info - LoRa transmiting message...");

//Set in-house protocol stuff:
 	LoRaMessage.frameVersion = currentSupportedFrameVersion;
 	LoRaMessage.recipient    = destination;
 	LoRaMessage.sender       = localAddress;
  LoRaMessage.iterationCounter = iterationCounter; // Write message counter data to union

  ToAStart = micros();  //Pour mesurer approx le temps de transmission
  LoRa.beginPacket();
  LoRa.write(LoRaMessage.bytes, sizeof(LoRaMessage));  //Clef pour la transmission "binaire"
  LoRa.endPacket();

  retransmitCounter = 0; // Clear message retransmit counter
  transmitCounter = 0;
  iterationCounter++; // Increment iteration counter

  // Put modem to sleep
  DEBUG_PRINTLN("Info - Putting LoRa module to sleep...");
  //LoRa.sleep();

  // Stop the loop timer
  timer.lora = millis() - loopStartTime;

  transmitCounter++;  //One down, many more to go...
  LoRaTransmitCompleted = false;

}

void LoRaSendCallback()
{
  //DEBUG_PRINTLN("LoRa callBack");  //Attention! Risqué!
  uint32_t ToAStop = micros();
  if (ToAStop > ToAStart)
    LoRaTimeOnAir = ToAStop - ToAStart;
  else LoRaTimeOnAir=0;
  // Write duration of last transmission to union
  LoRaMessage.transmitDuration = LoRaTimeOnAir;
  LoRaTransmitCompleted = true;
}