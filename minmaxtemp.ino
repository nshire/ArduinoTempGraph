/***************************************************************************
  This is for the BMP280 temperature & pressure sensor

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  for the interface.
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bme; // I2C
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

float measuredC = 0, measuredK = 0, maxK = 0, measuredF = 0;
float minK = 1000;

void setup() {
  Serial.begin(9600);
  if (!bme.begin(0x76)) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  measuredC = bme.readTemperature();
}
  
void loop() {

    //get temperature data
    measuredC = bme.readTemperature();

    //temperature conversion
    measuredK = measuredC + 273.15;
    measuredF = ((measuredK - 273.15) * 1.8) + 32;

    //find lowest temperature
    if(measuredK < minK){
      minK = measuredK;
    }

    if(measuredK > maxK){
      maxK = measuredK;
    }
    
    //Output

    Serial.print(measuredF);
    Serial.print(",");
    Serial.print(((minK - 273.15) * 1.8) + 32);
    Serial.print(",");
    Serial.print(((maxK - 273.15) * 1.8) + 32);
    Serial.println();
    delay(50); //1,000ms = 1s
}
