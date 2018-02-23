/*  Avionics for SSI-65 NETFLIX LAUNCH
 *   
 *   Jason Kurohara, Jonathan Zwiebel, Raul Dagir, ...
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <IridiumSBD.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h> 

#define DEBUG // comment out to turn off debugging
#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (String(x))
#define DEBUG_PRINTLN(x)  Serial.println (String(x))
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// System Parameters
#define SERIAL_TIMEOUT 60
#define ROCKBLOCK_ENABLED true

// Environemental Parameters
#define LAUNCH_SITE_PRESSURE 1014.562

// SPI Ports
#define SCK_PIN 13
#define SD_READER_CS 10

// State
bool firstSend = true;
String dataStringBuffer = "";

// Timing (Internal)
long startTime;
long refTop; //Time difference between 't' command and start of program
long refBottom; // Time difference between 'b' command and start of program
#define SD_CARD_FLUSH_TIME 10000 // 10 Seconds
#define ROCKBLOCK_TRANSMIT_TIME 300000 // 5 Minutes
#define BAROMETER_MEASURMENT_INTERVAL 10000 // 10 Seconds

// SD Card Reader (SPI)
File dataFile;
long lastFlush;

// BMP280 (I2C)
Adafruit_BMP280 bmp;
long lastAscentTime;             // Time of last ascent rate calculation
double lastAlt;                 // Last altitude, for calculation
double ascentRate;              // Last calculated rate, to fill forward in logging

// ROCKBlock (Hardware Serial) Radio
long lastTransmit;
#define IridiumSerial Serial3
IridiumSBD modem(IridiumSerial);

// GPS (Hardware Serial) GPS
TinyGPSPlus gps;
float f_lat = 0, f_long = 0, f_alt = 0;
int sats = -1;

void setup() {
  startTime = millis();
  lastFlush = 0;
  lastTransmit = 0;

  #ifdef DEBUG
  Serial.begin(9600);
  for(int i = 0; i < SERIAL_TIMEOUT && !Serial; i++) {
    continue;
    delay(1000);
  }
  #endif

  // SD Card Reader Setup (SPI)
  SPI.setSCK(SCK_PIN);
  pinMode(SD_READER_CS, OUTPUT);
  DEBUG_PRINTLN("Initializing SD card...");
  if (!SD.begin(SD_READER_CS)) {
    DEBUG_PRINTLN("Card failed, or not present");
  }
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  DEBUG_PRINTLN("Card initialized.");

  // BMP280 Setup (I2C)
  if (!bmp.begin()) {
    DEBUG_PRINTLN("Could not find a valid BMP280 sensor, check wiring!");
  }
  lastAlt = bmp.readAltitude(LAUNCH_SITE_PRESSURE); // initialize ascent rate variables
  ascentRate = 0;
  lastAscentTime = millis();

  // GPS Setup (Serial)
  Serial1.begin(9600);

  // Data column headers. 
  dataFile.print("Time(ms), Pressure(Pa), Alt(m), AscentRate(m/s), TempIn(C), ");
  dataFile.println("GPSLat, GPSLong, GPSAlt, GPSSats");

  // RockBlock Setup (Yikes)
  if(ROCKBLOCK_ENABLED) {
    IridiumSerial.begin(19200);
    DEBUG_PRINTLN("Starting rockblock serial");
    int err = modem.begin();
    int signalQuality = -1;
    DEBUG_PRINTLN(modem.getSignalQuality(signalQuality));
    if (err != ISBD_SUCCESS) {
      DEBUG_PRINT("Begin failed: error ");
      DEBUG_PRINTLN(err);
      if (err == ISBD_NO_MODEM_DETECTED) DEBUG_PRINTLN("No modem detected: check wiring.");
    }
  }
}

void loop() {
  long loopTime = millis();

  if (ROCKBLOCK_ENABLED && (loopTime - lastTransmit > ROCKBLOCK_TRANSMIT_TIME)) {
    DEBUG_PRINTLN("Transmiting to ROCKBlock");
    char buf [200];
    dataStringBuffer.toCharArray(buf, sizeof(buf));
    modem.sendSBDText(buf);
    lastTransmit = loopTime;
  }
  if (!ROCKBLOCK_ENABLED) {
    readAndWrite();
  }
}

// Reads from all of the sensors and outputs the data string
// Data string has the format: "Time(ms), Pressure(Pa), Alt(m), AscentRate(m/s), TempIn(C),
// "GPSLat, GPSLong, GPSAlt. GPSSats"
String readSensors() {
  String dataString = "";

  // Timing
  long loopTime = millis();
  DEBUG_PRINTLN("Loop Time ");
  DEBUG_PRINTLN(loopTime);
  dataString += String(loopTime) + ", ";

  // BMP280 (Barometer + Thermometer) Input
  DEBUG_PRINTLN("BMP280 stuff");
  double tempIn = bmp.readTemperature();
  double pressure = bmp.readPressure();
  double alt = bmp.readAltitude(LAUNCH_SITE_PRESSURE); // avg sea level pressure for hollister for past month
  if (loopTime - lastAscentTime > BAROMETER_MEASURMENT_INTERVAL) { // calculate ascent rate in m/s every 10s
    ascentRate = (alt - lastAlt) * 1000 / (loopTime - lastAscentTime);
    lastAlt = alt;
    lastAscentTime = loopTime;
  }
  dataString += String(pressure) + ", " + String(alt) + ", ";
  dataString += String(ascentRate) + ", " + String(tempIn) + ", ";
  DEBUG_PRINT("Pressure: ");
  DEBUG_PRINTLN(pressure);
  DEBUG_PRINT("Altitude: ");
  DEBUG_PRINTLN(alt);
  DEBUG_PRINT("Ascent Rate: ");
  DEBUG_PRINTLN(ascentRate);
  DEBUG_PRINT("Inside Temperature: ");
  DEBUG_PRINTLN(tempIn);

  // GPS Input
  DEBUG_PRINTLN("GPS Stuff");
  
  while (Serial1.available()) {
    char c = Serial1.read();
    if(gps.encode(c)) {
      DEBUG_PRINTLN("Printing the encoded data!");
    }
  }
  f_lat = gps.location.lat();
  f_long = gps.location.lng();
  f_alt = gps.altitude.meters();
  sats = gps.satellites.value();

  dataString += String(f_lat) + ", " + String(f_long) + ", ";
  dataString += String(f_alt) + ", " + String(sats) + ", ";
  DEBUG_PRINT("GPS Lat: ");
  DEBUG_PRINTLN(f_lat);
  DEBUG_PRINT("GPS Long: ");
  DEBUG_PRINTLN(f_long);
  DEBUG_PRINT("GPS Alt: ");
  DEBUG_PRINTLN(f_alt);
  DEBUG_PRINT("GPS Sats: ");
  DEBUG_PRINTLN(sats);
  DEBUG_PRINTLN("");

  dataStringBuffer = dataString;
  return dataString;     
}

bool readAndWrite() {
  String dataString = readSensors();
  long loopTime = millis();

  if (dataFile) {
    DEBUG_PRINTLN("Writing to datalog.txt");
    dataFile.println(dataString);
  } else {
    DEBUG_PRINTLN("Error opening datalog.txt");
  }

  if (loopTime - lastFlush > SD_CARD_FLUSH_TIME) {
    DEBUG_PRINTLN("Flushing datalog.txt");
    dataFile.flush();
    lastFlush = loopTime;
  }  
  
  delay(50);
  return true;
}

bool ISBDCallback() {
  return readAndWrite();
}
