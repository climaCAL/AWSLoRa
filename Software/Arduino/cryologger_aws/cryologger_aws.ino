/*
    Title:    Cryologger Automatic Weather Station
    Date:     February 24, 2023
    Author:   Adam Garbo
    Version:  0.4

    Description:
    - Code configured for automatic weather stations to be deployed in Igloolik, Nunavut.
    - Code modified and adapted by Yh for LoRa data transmission at CEGEP André-Laurendeau, AWS project

    Components:
    - (Yh removed) Rock7 RockBLOCK 9603
    - (Yh removed) Maxtena M1621HCT-P-SMA antenna (optional)
    - Adafruit Feather M0 Adalogger
    - (Yh new) RFM95 - Lora Module OR Adafruit M0 LoRa
    - Adafruit Ultimate GPS Featherwing
    - Adafruit BME280 Temperature Humidity Pressure Sensor
    - Adafruit LSM303AGR Accelerometer/Magnetomter
    - Pololu 3.3V 600mA Step-Down Voltage Regulator D36V6F3
    - Pololu 5V 600mA Step-Down Voltage Regulator D36V6F5
    - Pololu 12V 600mA Step-Down Voltage Regulator D36V6F5
    - SanDisk Industrial XI 8 GB microSD card

    Sensors:
    - RM Young 05103L Wind Monitor
    - Vaisala HMP60 Humidity and Temperature Probe
    - (Yh added support) VMS3000 Anemometer
    - (CAL added support) RS485 Wind Speed Transmitter (SEN0483) : https://wiki.dfrobot.com/RS485_Wind_Speed_Transmitter_SKU_SEN0483
    - (CAL added support) RS485 Wind Direction Transmitter (SEN0482): https://wiki.dfrobot.com/RS485_Wind_Direction_Transmitter_SKU_SEN0482

    Comments:
    - Sketch uses 98720 bytes (37%) of program storage space. Maximum is 262144 bytes.
    - Power consumption in deep sleep is ~625 uA at 12.5V

*/

// ----------------------------------------------------------------------------
// Libraries
// ----------------------------------------------------------------------------
#include <Adafruit_BME280.h>        // https://github.com/adafruit/Adafruit_BME280 (v2.2.2)
#include <Adafruit_LSM303_Accel.h>  // https://github.com/adafruit/Adafruit_LSM303_Accel (v1.1.4)
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor (v1.1.4)
#include <Arduino.h>                // Required for new Serial instance. Include before <wiring_private.h>
#include <ArduinoLowPower.h>        // https://github.com/arduino-libraries/ArduinoLowPower (v1.2.2)
//Yh-031823-#include <IridiumSBD.h>             // https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library (v3.0.5)
#include <LoRa.h>
#include <RTCZero.h>                // https://github.com/arduino-libraries/RTCZero (v1.6.0)
#include <SdFat.h>                  // https://github.com/greiman/SdFat (v2.1.2)
#include <sensirion.h>              // https://github.com/HydroSense/sensirion
#include <Statistic.h>              // https://github.com/RobTillaart/Statistic (v1.0.0)
#include <TimeLib.h>                // https://github.com/PaulStoffregen/Time (v1.6.1)
#include <TinyGPS++.h>              // https://github.com/mikalhart/TinyGPSPlus (v1.0.3)
#include <Wire.h>                   // https://www.arduino.cc/en/Reference/Wire
#include <wiring_private.h>         // Required for creating new Serial instance

// ----------------------------------------------------------------------------
// Define unique identifier
// ----------------------------------------------------------------------------
#define CRYOLOGGER_ID "CAL"

// ----------------------------------------------------------------------------
// Data logging
// ----------------------------------------------------------------------------
#define LOGGING         true  // Log data to microSD

// ----------------------------------------------------------------------------
// Debugging macros
// ----------------------------------------------------------------------------
#define DEBUG           true   // Output debug messages to Serial Monitor
#define DEBUG_GNSS      false  // Output GNSS debug information
#define DEBUG_IRIDIUM   false  // Output Iridium debug messages to Serial Monitor
#define CALIBRATE       false  // Enable sensor calibration code
#define DEBUG_LORA      false   // Output LoRa messages to SM 

#if DEBUG
#define DEBUG_PRINT(x)            SERIAL_PORT.print(x)
#define DEBUG_PRINTLN(x)          SERIAL_PORT.println(x)
#define DEBUG_PRINT_HEX(x)        SERIAL_PORT.print(x, HEX)
#define DEBUG_PRINTLN_HEX(x)      SERIAL_PORT.println(x, HEX)
#define DEBUG_PRINT_DEC(x, y)     SERIAL_PORT.print(x, y)
#define DEBUG_PRINTLN_DEC(x, y)   SERIAL_PORT.println(x, y)
#define DEBUG_WRITE(x)            SERIAL_PORT.write(x)

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_HEX(x)
#define DEBUG_PRINTLN_HEX(x)
#define DEBUG_PRINT_DEC(x, y)
#define DEBUG_PRINTLN_DEC(x, y)
#define DEBUG_WRITE(x)
#endif

// ----------------------------------------------------------------------------
// Pin definitions
// ----------------------------------------------------------------------------
#define PIN_VBAT            A0
#define PIN_WIND_SPEED      A1
#define PIN_WIND_DIR        A2
#define PIN_HUMID           A3
#define PIN_TEMP            A4
#define PIN_GNSS_EN         A5
#define PIN_MICROSD_CS      9   //Yh confirmedOk - april-may2023 - Was:4 on adaloger
#define PIN_12V_EN          5   // 12 V step-up/down regulator
#define PIN_5V_EN           6   // 5V step-down regulator
#define PIN_LED_GREEN       10 //Was:8   // Green LED
//#define PIN_IRIDIUM_RX      10  // Pin 1 RXD (Yellow)
//#define PIN_IRIDIUM_TX      11  // Pin 6 TXD (Orange)
//#define PIN_IRIDIUM_SLEEP   12  // Pin 7 OnOff (Grey)
#define PIN_LED_RED         13

// Unused
#define PIN_SOLAR           14
#define PIN_SENSOR_PWR      14

//RFM95 - LoRa module
//PMOD-RFM95: CS=10, RST=11, INT/DIO0=12
//AdaRFM95: CS=8, RST=4, INt/DIO0=7
#define PIN_RFM95_CS        8 //marked as D10, PA18
#define PIN_RFM95_RST       4  //marked as D11, PA16
#define PIN_RFM95_INT       7  //marked as D12, PA19
#define PIN_RFM95_DIO0      7  //Same as INT
// Remaining pins are std to M0: MOSI(23), MISO(22), CLK(24)

// ----------------------------------------------------------------------------
// RFM95W radio definitions
// ----------------------------------------------------------------------------
#define RF95_FREQ     902700000UL   // Radio frequency (MHz)
#define RF95_PW       20      // Transmit power (dBm)
#define RF95_SF       10       // Spreading factor
#define RF95_BW       125000UL  // Bandwidth (MHz)
#define RF95_CR       5       // Coding rate
#define RF95_CRC      true    // Cyclic Redundancy Check (CRC)
#define LoRasyncWord  0xAA    // Kinda data "scrambler"


// ------------------------------------------------------------------------------------------------
// Port configuration
// ------------------------------------------------------------------------------------------------
// Create a new UART instance and assign it to pins 10 (RX) and 11 (TX).
// For more information see: https://www.arduino.cc/en/Tutorial/SamdSercom
//Yh-031823-Uart Serial2 (&sercom1, PIN_IRIDIUM_RX, PIN_IRIDIUM_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);

#define SERIAL_PORT   Serial
#define GNSS_PORT     Serial1

//WindSensor module I2C address declaration:
const uint16_t WIND_SENSOR_SLAVE_ADDR = 0x66;

//Yh-031823-#define IRIDIUM_PORT  Serial2

// Attach interrupt handler to SERCOM for new Serial instance
//Yh-031823-void SERCOM1_Handler()
//Yh-031823-{
//Yh-031823-  Serial2.IrqHandler();
//Yh-031823-}

// ----------------------------------------------------------------------------
// Object instantiations
// ----------------------------------------------------------------------------
Adafruit_BME280                 bme280;
Adafruit_LSM303_Accel_Unified   lsm303 = Adafruit_LSM303_Accel_Unified(54321); // I2C address: 0x1E
//Yh-031823-IridiumSBD                      modem(IRIDIUM_PORT, PIN_IRIDIUM_SLEEP);
RTCZero                         rtc;
SdFs                            sd;           // File system object
FsFile                          logFile;      // Log file
TinyGPSPlus                     gnss;
sensirion                       sht(20, 21);  // (data, clock). Pull-up required on data pin

// Custom TinyGPS objects to store fix and validity information
// Note: $GPGGA and $GPRMC sentences produced by GPS receivers (PA6H module)
// $GNGGA and $GNRMC sentences produced by GPS/GLONASS receivers (PA161D module)
TinyGPSCustom gnssFix(gnss, "GPGGA", 6); // Fix quality
TinyGPSCustom gnssValidity(gnss, "GPRMC", 2); // Validity

// ----------------------------------------------------------------------------
// Statistics objects
// ----------------------------------------------------------------------------
Statistic batteryStats;         // Battery voltage
Statistic temperatureIntStats;  // Internal temperature
Statistic humidityIntStats;     // Internal humidity
Statistic pressureIntStats;     // Internal pressure
Statistic temperatureExtStats;  // External temperature
Statistic humidityExtStats;     // External humidity
Statistic solarStats;           // Solar radiation
Statistic windSpeedStats;       // Wind speed
Statistic uStats;               // Wind east-west wind vector component (u)
Statistic vStats;               // Wind north-south wind vector component (v)

// ----------------------------------------------------------------------------
// User defined global variable declarations
// ----------------------------------------------------------------------------
unsigned long sampleInterval    = 1;  //Yh was:5   // Sampling interval (minutes). Default: 5 min (300 seconds)
unsigned int  averageInterval   = 5; //Yh was:12    // Number of samples to be averaged in each message. Default: 12 (hourly)
unsigned int  transmitInterval  = 1;      // Number of messages in each Iridium transmission (340-byte limit)
unsigned int  retransmitLimit   = 1;      // Failed data transmission reattempts (340-byte limit)
unsigned int  gnssTimeout       = 120;    // Timeout for GNSS signal acquisition (seconds)
unsigned int  iridiumTimeout    = 180;    // Timeout for Iridium transmission (seconds)
bool          firstTimeFlag     = true;   // Flag to determine if program is running for the first time
float         batteryCutoff     = 11.0;    // Battery voltage cutoff threshold (V)
byte          loggingMode       = 1;  //Yh was:2    // Flag for new log file creation. 1: daily, 2: monthly, 3: yearly

// ----------------------------------------------------------------------------
// Global variable declarations
// ----------------------------------------------------------------------------
volatile bool alarmFlag         = false;  // Flag for alarm interrupt service routine
volatile bool wdtFlag           = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  wdtCounter        = 0;      // Watchdog Timer interrupt counter
volatile int  revolutions       = 0;      // Wind speed ISR counter
bool          resetFlag         = false;  // Flag to force system reset using Watchdog Timer
uint8_t       moSbdBuffer[340];           // Buffer for Mobile Originated SBD (MO-SBD) message (340 bytes max)
uint8_t       mtSbdBuffer[270];           // Buffer for Mobile Terminated SBD (MT-SBD) message (270 bytes max)
size_t        moSbdBufferSize;
size_t        mtSbdBufferSize;
char          logFileName[30]   = "";     // Log file name
char          dateTime[30]      = "";     // Datetime buffer
byte          retransmitCounter = 0;      // Counter for Iridium 9603 transmission reattempts
byte          transmitCounter   = 0;      // Counter for Iridium 9603 transmission intervals
unsigned long LoRaTimeOnAir     = 0;      // LoRa Time On Air measurement
unsigned long ToAStart          = 0;      // start timer for LoRa ToA measurement
byte          currentLogFile    = 0;      // Variable for tracking when new microSD log files are created
byte          newLogFile        = 0;      // Variable for tracking when new microSD log files are created
byte          currentDate       = 0;      // Variable for tracking when the date changes
byte          newDate           = 0;      // Variable for tracking when the date changes
int           transmitStatus    = 0;      // Iridium transmission status code
bool          LoRaTransmitCompleted = false;
unsigned int  iterationCounter  = 0;      // Counter for program iterations (zero indicates a reset)
unsigned int  failureCounter    = 0;      // Counter of consecutive failed Iridium transmission attempts
unsigned long previousMillis    = 0;      // Global millis() timer
unsigned long alarmTime         = 0;      // Global epoch alarm time variable
unsigned long unixtime          = 0;      // Global epoch time variable
unsigned int  sampleCounter     = 0;      // Sensor measurement counter
unsigned int  cutoffCounter     = 0;      // Battery voltage cutoff sleep cycle counter
unsigned long samplesSaved      = 0;      // Log file sample counter
long          rtcDrift          = 0;      // RTC drift calculated during sync
float         temperatureInt    = 0.0;    // Internal temperature (°C)
float         humidityInt       = 0.0;    // Internal hunidity (%)
float         pressureInt       = 0.0;    // Internal pressure (hPa)
float         temperatureExt    = 0.0;    // External temperature (°C)
float         humidityExt       = 0.0;    // External humidity (%)
float         pitch             = 0.0;    // Pitch (°)
float         roll              = 0.0;    // Roll (°)
float         solar             = 0.0;    // Solar radiation
float         windSpeed         = 0.0;    // Wind speed (m/s)
float         windDirection     = 0.0;    // Wind direction (°)
float         windGustSpeed     = 0.0;    // Wind gust speed  (m/s)
float         windGustDirection = 0.0;    // Wind gust direction (°)
int           windDirectionSector = 0.0;  // Wind direction indicator (ref to DFRWindSpeed() for details)
float         voltage           = 0.0;    // Battery voltage (V)
float         latitude          = 0.0;    // GNSS latitude (DD)
float         longitude         = 0.0;    // GNSS longitude (DD)
byte          satellites        = 0;      // GNSS satellites
float         hdop              = 0.0;    // GNSS HDOP
tmElements_t  tm;                         // Variable for converting time elements to time_t

// ----------------------------------------------------------------------------
// Unions/structures
// ----------------------------------------------------------------------------

// DFRWindSensor (CAL) struc to store/retreive data
typedef struct {
  uint8_t regMemoryMap[6] = {0,0,0,0,0,0};
  float angleVentFloat = 0;
  uint16_t directionVentInt = 0;
  float vitesseVentFloat = 0;
}vent;

// Union to store Iridium Short Burst Data (SBD) Mobile Originated (MO) messages
// Yh 031823 - replaced moSbdMessage for LoRaMessage

const byte localAddress = 0x01;     // address of this device
const byte destination = 0xF1;      // destination to send to (gateway, repeater)
const byte currentSupportedFrameVersion = 0x03;  //AWS cryologger

typedef union
{
  struct
  {
    uint8_t   frameVersion;       // version de la trame            (1 byte)
    uint8_t   recipient;          // recipient address              (1 byte)
    uint8_t   sender;             // local address                  (1 byte)
    uint32_t  unixtime;           // UNIX Epoch time                (4 bytes)
    int16_t   temperatureInt;     // Internal temperature (°C)      (2 bytes)   * 100
    uint16_t  humidityInt;        // Internal humidity (%)          (2 bytes)   * 100
    uint16_t  pressureInt;        // Internal pressure (hPa)        (2 bytes)   - 850 * 100
    int16_t   temperatureExt;     // External temperature (°C)      (2 bytes)   * 100
    uint16_t  humidityExt;        // External humidity (%)          (2 bytes)   * 10
    int16_t   pitch;              // Pitch (°)                      (2 bytes)   * 100
    int16_t   roll;               // Roll (°)                       (2 bytes)   * 100
    //uint16_t  solar;              // Solar irradiance (W m-2)       (2 bytes)   * 100
    uint16_t  windSpeed;          // Mean wind speed (m/s)          (2 bytes)   * 100
    uint16_t  windDirection;      // Mean wind direction (°)        (2 bytes)
    uint16_t  windGustSpeed;      // Wind gust speed (m/s)          (2 bytes)   * 100
    uint16_t  windGustDirection;  // Wind gust direction (°)        (2 bytes)
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)   * 1000000
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)   * 1000000
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint16_t  hdop;               // HDOP                           (2 bytes)
    uint16_t  voltage;            // Battery voltage (V)            (2 bytes)   * 100
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint8_t   transmitStatus;     // Iridium return code            (1 byte)
    uint16_t  iterationCounter;   // Message counter                (2 bytes)
  } __attribute__((packed));                                    // Total: (47 bytes)
  uint8_t bytes[47];
} LORA_MESSAGE;

LORA_MESSAGE LoRaMessage;

/* Yh 031823 - Not required with LoRa (not receiving)
// Union to store received Iridium SBD Mobile Terminated (MT) message
typedef union
{
  struct
  {
    uint8_t   sampleInterval;     // 2 bytes
    uint8_t   averageInterval;    // 1 byte
    uint8_t   transmitInterval;   // 1 byte
    uint8_t   retransmitLimit;    // 1 byte
    uint8_t   batteryCutoff;      // 1 bytes
    uint8_t   resetFlag;          // 1 byte
  };
  uint8_t bytes[7]; // Size of message to be received in bytes
} SBD_MT_MESSAGE;

SBD_MT_MESSAGE mtSbdMessage;
*/

// Structure to store device online/offline states
struct struct_online
{
  bool bme280   = false;
  bool lsm303   = false;
  bool gnss     = false;
  bool microSd  = false;
  bool lora     = false;
} online;

// Structure to store function timers
struct struct_timer
{
  unsigned long readRtc;
  unsigned long readBattery;
  unsigned long configMicroSd;
  unsigned long writeMicroSd;
  unsigned long readGnss;
  unsigned long readBme280;
  unsigned long readLsm303;
  unsigned long readHmp60;
  unsigned long readSht31;  
  unsigned long readDFRWS;  //Yh-0504- New: readDFRWindSensor
  unsigned long read5103L;
  unsigned long read7911;
  unsigned long readSp212;
  unsigned long iridium;  //Yh-031823-not req
  unsigned long lora;  //Yh-031823-new
} timer;

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup()
{
  // Pin assignments
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_SENSOR_PWR, OUTPUT);
  pinMode(PIN_5V_EN, OUTPUT);
  pinMode(PIN_12V_EN, OUTPUT);
  pinMode(PIN_GNSS_EN, OUTPUT);
  pinMode(PIN_VBAT, INPUT);
  digitalWrite(PIN_LED_GREEN, LOW);   // Disable green LED
  digitalWrite(PIN_LED_RED, LOW);     // Disable red LED
  digitalWrite(PIN_SENSOR_PWR, LOW);  // Disable power to 3.3V
  digitalWrite(PIN_5V_EN, LOW);       // Disable power to Iridium 9603
  digitalWrite(PIN_12V_EN, LOW);      // Disable 12V power
  digitalWrite(PIN_GNSS_EN, HIGH);    // Disable power to GNSS

  // Configure analog-to-digital (ADC) converter
  configureAdc();

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz

#if DEBUG
  SERIAL_PORT.begin(115200); // Open serial port at 115200 baud
  blinkLed(PIN_LED_GREEN, 4, 500); // Non-blocking delay to allow user to open Serial Monitor
#endif

  DEBUG_PRINTLN();
  printLine();
  DEBUG_PRINT("Cryologger - Automatic Weather Station #"); DEBUG_PRINTLN(CRYOLOGGER_ID);

  printLine();

#if CALIBRATE
  enable5V();   // Enable 5V power
  enable12V();  // Enable 12V power

  while (true)
  {
    petDog(); // Reset WDT
    //calibrateAdc();
    read5103L();
    readHmp60();
    myDelay(500);
  }
#endif

  // Configure devices
  configureRtc();       // Configure real-time clock (RTC)
  readRtc();            // Read date and time from RTC
  configureWdt();       // Configure Watchdog Timer (WDT)
  readBattery();        // Read battery voltage
  configureSd();        // Configure microSD
  printSettings();      // Print configuration settings
  readGnss();           // Sync RTC with GNSS
//Yh-031823-  configureIridium();   // Configure Iridium 9603 transceiver
  configureLoRa();      // Configure RFM95W radio
  createLogFile();      // Create initial log file

  // Close serial port if immediately entering deep sleep
  if (!firstTimeFlag)
  {
    disableSerial();
  }

  // Blink LED to indicate completion of setup
  for (byte i = 0; i < 10; i++)
  {
    // Blink LEDs
    digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
    digitalWrite(PIN_LED_GREEN, !digitalRead(PIN_LED_GREEN));
    myDelay(250);
  }
}

// ----------------------------------------------------------------------------
// Loop
// ----------------------------------------------------------------------------
void loop()
{
  // Check if RTC alarm triggered or if program is running for first time
  if (alarmFlag || firstTimeFlag)
  {
    // Read the RTC
    readRtc();

    // Print date and time
    DEBUG_PRINT("Info - Alarm trigger: "); printDateTime();

    // Reset WDT
    petDog();

    // Increment the sample counter
    sampleCounter++;

    // Check if program is running for the first time
    if (!firstTimeFlag)
    {
      // Wake from deep sleep
      wakeUp();
    }

    // Read battery voltage
    readBattery();

    // Check if battery voltage is above cutoff threshold
    if (voltage < batteryCutoff)
    {
      cutoffCounter++;

      // In the event that the battery voltage never recovers, force a reset of the
      // system after 1 week
      if (cutoffCounter > 168)
      {
        // Force WDT reset
        while (1);
      }

      DEBUG_PRINTLN("Warning - Battery voltage cutoff exceeded. Entering deep sleep...");

      // Reset sample counter
      sampleCounter = 0;

      // Clear statistics objects
      clearStats();

      // Go to sleep
      setCutoffAlarm();
    }
    else
    {
      DEBUG_PRINT("Info - Battery voltage good: "); DEBUG_PRINTLN(voltage);

      cutoffCounter = 0;

      // Perform measurements
      enable5V();       // Enable 5V power
      enable12V();      // Enable 12V power
      readBme280();     // Read sensor
      readLsm303();     // Read accelerometer
      //readSp212();    // Read solar radiation
      //readSht31();    // Read temperature/relative humidity sensor
      //read7911();     // Read anemometer
      readVMS3000();  // Read Anemometer model VMS-3000-FSJT-NPNR
      //readDFRWindSensor();  // Read Anemometer DFR Wind Sensor (DFRobot - CAL)
      readHmp60();      // Read temperature/relative humidity sensor
      //read5103L();    // Read anemometer
      disable12V();     // Disable 12V power
      disable5V();      // Disable 5V power

      // Print summary of statistics
      printStats();

      // Check if number of samples collected has been reached and calculate statistics (if enabled)
      if ((sampleCounter == averageInterval) || firstTimeFlag)
      {
        calculateStats(); // Calculate statistics of variables to be transmitted
//Yh-031823-        writeBuffer();    // Write data to transmit buffer
        LoRaTransmitData();  //Yh 042923
        // Check if data transmission interval has been reached
        if ((transmitCounter == transmitInterval) || firstTimeFlag)
        {
          // Check for date change
          checkDate();
          if (firstTimeFlag || (currentDate != newDate))
          {
            readGnss(); // Sync RTC with the GNSS
            currentDate = newDate;
            Serial.print("currentDate: "); Serial.println(currentDate);
            Serial.print("newDate: "); Serial.println(newDate);
          }
//Yh-031823-          transmitData(); // Transmit data via Iridium transceiver
        }
        
        //Wait for LoRa transmit to complete (max 1.5 sec):
        //Note: Arduino-LoRa lib warns that onReceive and on onTxDone won't work on SAMD !!! Sh**!!
        //According to https://iftnt.github.io/lora-air-time/index.html estimated air time is 534ms
        uint32_t loRaTimer = millis();
        const uint32_t loRaDelay = 1500;
        while (!LoRaTransmitCompleted && ((millis() - loRaTimer)<loRaDelay))
          myDelay(100);

        if (millis() - loRaTimer>loRaDelay) DEBUG_PRINTLN("Warn - LoRa transmit exceeded"); else DEBUG_PRINTLN("Info - LoRa transmit OK");
        LoRaTransmitCompleted = false;  //Reset flag for next iteration
        
        sampleCounter = 0; // Reset sample counter
      }

      // Log data to microSD card
      logData();

      // Print function execution timers
      printTimers();

      // Set the RTC alarm
      setRtcAlarm();

      DEBUG_PRINTLN("Info - Entering deep sleep...");
      DEBUG_PRINTLN();

      // Prepare for sleep
      prepareForSleep();
    }
  }
  // Check for WDT interrupts
  if (wdtFlag)
  {
    petDog(); // Reset the WDT
  }

  // Blink LED to indicate WDT interrupt and nominal system operation
  blinkLed(PIN_LED_GREEN, 1, 25);

  // Enter deep sleep and wait for WDT or RTC alarm interrupt
  goToSleep();
}


