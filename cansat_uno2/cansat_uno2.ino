
//Chips: L3G4200D+ADXL345+HMC5883L+BMP085.

#include <SPI.h>
#include <SD.h>
File dataFile;

String    st1 = "logg001.csv";
int n = 0, c = 0;
char    buf[12] = "logg001.csv";


#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"

ADXL345 accel;
int16_t ax, ay, az;

#include "I2Cdev.h"
#include "HMC5883L.h"
HMC5883L mag;
int16_t mx, my, mz;

#include <L3G4200D.h>
L3G4200D gyroscope;
unsigned long timer = 0;
float timeStep = 0.01;
float pitch = 0;
float roll = 0;
float yaw = 0;

#include "BMP085.h"
BMP085 barometer;

float temperature;
float pressure;
float altitude;
int32_t lastMicros;

//#include <AltSoftSerial.h>
//AltSoftSerial altser;

#include <SoftwareSerial.h>
SoftwareSerial mySerial(8, 9); // RX, TX


#define RF_port mySerial
#define GPS_port Serial




void setup() {
  Serial.begin(115200);
  if (!SD.begin(10))
    Serial.println("Card failed, or not present");
  else
    Serial.println("SD card initialized.");

  st1 = "logg001.csv";
  //    Serial.print("F3:");Serial.print(st1);Serial.println("!!!");
  delay(100);
  while (SD.exists(buf))
  {
    st1 = String(n++, DEC);
    if (st1.length() == 1) st1 = "00" + st1;
    else if (st1.length() == 2) st1 = "0" + st1;
    st1 = "logg" + st1 + ".csv";
    st1.toCharArray(buf, 12) ;
  }
  Serial.print("Filnavn1: ");
  Serial.print(st1); Serial.print(" : ");
  Serial.println(buf);
  dataFile = SD.open(buf, FILE_WRITE);
  dataFile.println(__FILE__);
  dataFile.print("Kompiltert den: ");
  dataFile.println(__TIMESTAMP__);


  Wire.begin();
  accel.initialize();
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  mag.initialize();
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  barometer.initialize();
  //  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
  /*

    while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
    {
      Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
      delay(500);
    }
    Serial.println("Gyroscope initialize and calibrated");
    // gyroscope.calibrate(100);
  */
  dataFile.close();

  for (int g = 0; g < 5; g++)
  {
    dataFile.print(millis()); dataFile.print("\t|\t");
    RF_port.print(millis()); RF_port.print("\t|\t");
    adxl345(); dataFile.print("\t|\t");
     hmc5883l();dataFile.print("\t|\t");
    l3g4200d_pitch_and_roll();dataFile.print("\t|\t");
    BMP085();
    dataFile.println(); RF_port.println();
    dataFile.flush();
  }


  Serial.print(" Lest fra SD (");
  Serial.print(buf);
  Serial.println("):");

  dataFile = SD.open(buf, FILE_READ);
  c = 0;
  while (c != -1)
  { c = dataFile.read();
    Serial.print(char(c));
  }
  dataFile.close();

}

void loop() {

}
