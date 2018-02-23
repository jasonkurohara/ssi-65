/*
 * Heating Avionics for Netflix Launch
 * 
 */
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

int heat = 0;
bool heaterOn = false;
const int lower_threshold = 25;
const int upper_threshold = 27;
const int heater = 23;

void setup() {

 
 // put your setup code here, to run once:
 Serial.begin(9600);
pinMode(heater, OUTPUT);
digitalWrite(heater, HIGH);
heaterOn = true;

 Serial.println("ok");
 if(!bmp.begin()){
   Serial.println("wrong");
 }

}

void loop() {

 // put your main code here, to run repeatedly:
double tempIn = bmp.readTemperature();
heat = tempIn;
Serial.print("Temperature: ");
 Serial.println(tempIn);
 Serial.print("Heater state: ");
 Serial.println(heaterOn);

//checking if it's below upperthreshold
 if (heat > upper_threshold && heaterOn == 1) {
     digitalWrite(heater, LOW);
     heaterOn = false;
     Serial.println("heater off");

   }
   else if (heat < lower_threshold && heaterOn == 0) {
     digitalWrite(heater, HIGH);
     heaterOn = true;
     Serial.println("heater on");

   }
}
