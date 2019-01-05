/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.
Chips: L3G4200D+ADXL345+HMC5883L+BMP085.
 */

#include <SPI.h>
#include <SD.h>
#include "HMC5883L.h"

const int chipSelect = 10;

String st1="logg001.csv";
char buf[12]="logg001.csv";
 File dataFile;
//#define dataFile Serial

/*
    L3G4200D Triple Axis Gyroscope: Pitch, Roll and Yaw.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-l3g4200d.html
    GIT: https://github.com/jarzebski/Arduino-L3G4200D
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <L3G4200D.h>

L3G4200D gyroscope;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"

// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D
ADXL345 accel;

int16_t ax, ay, az;

#include "HMC5883L.h"

// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;

int16_t mx, my, mz;

#include "BMP085.h"

// class default I2C address is 0x77
// specific I2C addresses may be passed as a parameter here
// (though the BMP085 supports only one address)
BMP085 barometer;

float temperature;
float pressure;
float altitude;
int32_t lastMicros;
String GPS_buffer="";
boolean gps_komplett=false;

#include <AltSoftSerial.h>
AltSoftSerial altSerial;

//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(8, 9); // RX, TX


#define RF_port Serial
#define GPS_port Serial

char charBuf[15];
byte c;

void setup() {
//  st1.reserve(80);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  altSerial.begin(19200);
  Serial.println();
  Serial.println();
  Serial.println(buf);
 
   Serial.println("Initializing SD card...");
   if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    }
  else
  {
      Serial.println("SD card initialized.");
     delay(100);
    int n=1;
    st1="logg001.csv";
//    Serial.print("F3:");Serial.print(st1);Serial.println("!!!");
    delay(100);
    while (SD.exists(buf))
  {
     Serial.println("Filen finnes");
    st1=String(n++,DEC);
    Serial.print(st1);Serial.println(" : ");delay(100);
        sprintf(buf, "%d", n);
    Serial.print(buf);Serial.println(" : ");delay(100);
    if (st1.length()==1) st1="00"+st1;
    else if (st1.length()==2) st1="0"+st1;
    Serial.print(st1);Serial.println(" : ");delay(100);
    st1="logg"+st1+".csv";
    Serial.print(st1);Serial.println(" : ");delay(100);
    st1.toCharArray(buf,12) ;
    Serial.print("Filnavn2: ");
    Serial.print(st1);Serial.print(" : ");
    Serial.println(buf);
  }
   Serial.print("Filnavn1: ");
   Serial.print(st1);Serial.print(" : ");
   Serial.println(buf);

  dataFile = SD.open(buf, FILE_WRITE);
  dataFile.println(__FILE__);
  dataFile.print("Kompiltert den: ");
  dataFile.println(__TIMESTAMP__);
 
   }

 Wire.begin();

  // Set scale 2000 dps and 400HZ Output data rate (cut-off 50)
  if(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
  {
    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
    delay(500);
  }
  else
  { 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyroscope.calibrate(100);
  Serial.println("Gyroscope initialized and calibrated");
  }  
  
    mag.initialize();
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    barometer.initialize();
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

    accel.initialize();
    Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
 
 
 delay(500);

//l3g4200d_setup();
}

void loop() { }/*
   dataFile.print(millis());
  l3g4200d(); dataFile.print("\t|\t"); RF_port.print("\t|\t");
          dataFile.flush(); 
// ADSL345_raw();dataFile.print("\t|\t");RF_port.print("\t|\t");
        dataFile.flush();  
  HMC5883L();dataFile.print("\t|\t");RF_port.print("\t|\t");
          dataFile.flush(); 
//   l3g4200d_raw() ;dataFile.print("\t|\t");RF_port.print("\t|\t");
      
      dataFile.println();;RF_port.println();
        dataFile.close(); 
   Serial.println(" Lest fra SD");    
  dataFile = SD.open(buf, FILE_WRITE);
  c=0;
  while (c!=-1)
  {c=dataFile.read();
  Serial.print(c);
  }
        dataFile.close();   
       
}/*
   RF_port.print(millis());  RF_port.print("\t|\t");
   dataFile.print(millis()); dataFile.print("\t|\t");
      BMP085();
      dataFile.println();RF_port.println();
     dataFile.flush(); 
}/*
 //   if ((gps_komplett) && (GPS_buffer.substring(0,6)=="$GPRMC"))
 //   {
  //   RF_port.print(millis()); 
 //    RF_port.println(GPS_buffer);
 //    dataFile.print(GPS_buffer);
 //   }
 //   else
  //  {
  //l3g4200d(); dataFile.print("\t|\t"); RF_port.print("\t|\t");
  
 //  ADSL345_raw();dataFile.print("\t|\t");RF_port.print("\t|\t");
  //  HMC5883L();dataFile.print("\t|\t");RF_port.print("\t|\t");
//   l3g4200d_raw() ;dataFile.print("\t|\t");RF_port.print("\t|\t");
   BMP085();
      dataFile.println();RF_port.println();
     dataFile.flush(); 
   if (gps_komplett)
  //  {
  //    gps_komplett=false;
 //     GPS_buffer="";
   }

 
/*
void serialEvent()  // interupt  drevet lesing av Serial2-port for GPS
{
    while (GPS_port.available())
    {
      
  char    c=GPS_port.read();
  //    Serial.print(c);
      if (int(c)==13)
      {
       gps_komplett=true;
      }
      else if  (int(c)!=10)
      GPS_buffer +=c;
    }
}

*/






