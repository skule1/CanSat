/**********************************************************************
 * 
 * CanSat  med UNO og GOS 4800 baudrate og GY.80 IMU
 *  På grunn av små ressurser i UNO, kan ikke SD brukes sammen med med AltSerialport
 *  Kode for SD er lagt inn, som må aktiviseres ved å sette #define SD_Brikke
 *  Inkludert SD buker programmet 91% minne, uten SD 53% !!!!l 
 *  altsoftserial og SoftwareSerial er 3% av minnet
 *  Men da må andre ressurser kobles ut, for gyro
 *  AltSerialport brukes itl RF (APC220) 19200 baudrate (kan gjerne settes til 9600)
 *  (C) Skule Sørmo 2015
 *  
 **********************/



//#define SD_Brikke
#define gyro



#define BAUD 4800
#define MYUBRR F_CPU/16/BAUD-1



#ifdef SD_Brikke
#include <SPI.h>
#include <SD.h>
File dataFile;
String    st1="logg001.csv";
int n=0,c=0;
char    buf[12]="logg001.csv";
#endif

#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"

ADXL345 accel;
//int16_t ax, ay, az;


#include "I2Cdev.h"
#include "HMC5883L.h"
HMC5883L mag;
int16_t mx, my, mz;
#ifdef gyro

#include <L3G4200D.h>
L3G4200D gyroscope;
unsigned long timer = 0;
float timeStep = 0.01;
float pitch = 0;
float roll = 0;
float yaw = 0;
#endif

#include "BMP085.h"
BMP085 barometer;

float temperature;
float pressure;
float altitude;
int32_t lastMicros;

//#include <AltSoftSerial.h>
//AltSoftSerial altser;

#include <SoftwareSerial.h>
SoftwareSerial mySerial(8,9); // RX, TX


#define RF_port mySerial
#define GPS_port Serial


String gpsbuffer="",buf1="";
boolean gps_ferdig=false;
/*
// SETUP UART
void USART_Init( unsigned int ubrr)
{
   //Set baud rate 
   UBRR0H = (unsigned char)(ubrr>>8);
   UBRR0L = (unsigned char)ubrr;
   
  //Enable receiver and transmitter
//   UCSR0B = (1<<RXEN0)|(1<<TXEN0);
   UCSR0B = (1<<RXCIE0)|(1<<RXEN0);
   //RF.println(UCSR0B,HEX);
   
   // Set frame format: 8data, 2stop bit
   UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}
*/

void setup() {
  buf1.reserve(100);
  gpsbuffer.reserve(100);
//USART_Init(MYUBRR);

  Serial.begin(115200);
#ifdef SD_Brikke 
   if (!SD.begin(10)) 
    Serial.println("Card failed, or not present");
   else
     Serial.println("SD card initialized.");

  st1="logg001.csv";
//    Serial.print("F3:");Serial.print(st1);Serial.println("!!!");
    delay(100);
    while (SD.exists(buf))
  {
    st1=String(n++,DEC);
    if (st1.length()==1) st1="00"+st1;
    else if (st1.length()==2) st1="0"+st1;
    st1="logg"+st1+".csv";
    st1.toCharArray(buf,12) ;
   }
   Serial.print("Filnavn1: ");
   Serial.print(st1);Serial.print(" : ");
   Serial.println(buf);


  dataFile = SD.open(buf, FILE_WRITE);
  dataFile.println(__FILE__);
  dataFile.print("Kompiltert den: ");
  dataFile.println(__TIMESTAMP__);
  dataFile.flush();
#endif
    Wire.begin();
  accel.initialize();
    Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
  mag.initialize();
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

   barometer.initialize();
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

#ifdef gyro
while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
 {
    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
   delay(500);
 }
 Serial.println("Gyroscope initialize and calibrated");
  gyroscope.calibrate(100);
#endif


  //***************************************************
  Serial.println();
 #ifdef SD_Brikke
 
   Serial.print(" Lest fra SD (");    
    Serial.print(buf);
    Serial.println("):");
    
  dataFile = SD.open(buf, FILE_READ);
  c=0;
  while (c!=-1)
  {c=dataFile.read();
  Serial.print(char(c));
  }
        dataFile.close();   
     
#endif
}

void loop() {
  if (gps_ferdig)
  {
//  RF.print("GPS: ");
  RF_port.println(gpsbuffer);
  gpsbuffer="";
  gps_ferdig=false;
  }
 else
 {

#ifdef SD_Brikke
dataFile.print(millis());dataFile.print("\t|\t");
#endif

RF_port.print(millis());RF_port.print("\t|\t");
adxl345();

#ifdef SD_Brikke
dataFile.print("\t|\t");
#endif
RF_port.print("\t|\t");
hmc5883l();
#ifdef SD_Brikke
dataFile.print("\t|\t");
#endif
RF_port.print("\t|\t");
BMP085();
#ifdef SD_Brikke
dataFile.print("\t|\t");
#endif
#ifdef gyro
RF_port.print("\t|\t");
l3g4200d_pitch_and_roll();
#endif
#ifdef SD_Brikke
dataFile.print("\t|\t");
dataFile.println();
  dataFile.flush();
#endif
RF_port.print("\t|\t");
RF_port.println();
 }

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    gpsbuffer += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      gps_ferdig = true;
    }
  }
}


/*
ISR(USART_RX_vect)
{
  // Called when data received from USART

  // Read UDR register to reset flag
   char data = UDR0;
   //         RF.print(data);

     //    RF.println(buf1);
   
   if ((int(data)!=10) && (int(data)!=13))
    buf1 +=data;
    else if (int(data)==13)
    {
      if (buf1.substring(0,6)=="$GPRMC")
      {
      gpsbuffer=buf1;
      gps_ferdig=true;
      }
      buf1="";

    }
   
}

*/
