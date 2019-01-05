/****************************************
 * 
 *  CanSat med Arduino Mega. 
 *  GPS og SD-lagring av csv.fil
 *  Chips GY-80: L3G4200D+ADXL345+HMC5883L+BMP085.
 *  Sensorvariable som leses er akselerasjon i 3 dimensjoner,
 *  magnetisme i 3 dimensjoner og heading (kompasskurs),
 *  pitch, roll og yaw (tilting, rulling og dreiing) fra gyrosensor,
 *  gyro i tre dimensjoner, rå og normalisert,
 *  og temperatur, trykk og høyde
 *  
 *  Definer skilleteng i csv-fil i "define sh" under.
 *  
 *  Library-mappen i zip-fila kopieres til library-mappa i Arduino-mappa i Dokumenter.
 *  
 *  Denne skissen bruker interruptrutine for å lese GPS på Serial2
 *  der et byte leses for hver interrupt og lagres i buffer.
 *  Når GPS-linje er ferdig lest, vil den bli behandlet i loop()-rutinen
 *  
 *  (C) Skule Sørmo, juni 2016.
 *  
 ********************************/

 
#include <SPI.h>
#include <SD.h>
File dataFile;

String    st1 = "logg001.csv";
int n = 0, c = 0;
char    buf[12] = "logg001.csv";
#define sh ','
String shh;

//#include <DateTime.h>
//#include <DateTimeStrings.h>
#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by unix time_t as ten ascii digits
#define TIME_HEADER  255   // Header tag for serial time sync message

#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"

ADXL345 accel;
int16_t ax, ay, az;

//#include "I2Cdev.h"
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

//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(11, 12); // RX, TX


#define RF_port Serial
#define GPS Serial2
String GPS_buffer="",buf1="";
boolean GPS_ferdig=false;


  #define FOSC 160000//// Clock Speed
  #define BAUD 38400
  #define MYUBRR F_CPU/(16UL*BAUD)-1


void USART2_Init( unsigned int ubrr){
  // Set baud rate 
  UBRR2H = (unsigned char)(ubrr>>8);
  UBRR2L = (unsigned char)ubrr;
  // Enable receiver and transmitter 
  UCSR2B =(1<<RXEN2)|(1<<TXEN2)|(3<<6);
  // Set frame format: 8data, 2stop bit 
  UCSR2C = (1<<USBS2)|(3<<UCSZ02);
}

 


ISR(USART2_RX_vect)
{
  char c=char(UDR2);
  if (int(c)==13)
       {
//    Serial.println(buf1.substring(0,6));
  if (buf1.substring(0,6)=="$GPRMC")
   {
      GPS_buffer=buf1;
      GPS_ferdig=true;
   }
       buf1=""; 
    }   
     else if ((int(c)!=10) && (int(c)!=13))
     {
      buf1 +=c;
//Serial.println(buf1);
     }

} 
/*
void serialEvent2()
{
  if (GPS.available())
  {
    char c=GPS.read();
    if ((c!=char(10) && (c!=char(13))))
     GPS_buffer1 +=c;
 //  Serial.println(GPS_buffer1);
    if (c==char(13))
    {
 if (GPS_buffer1.substring(0,6)=="$GPRMC")
  {
    GPS_buffer=GPS_buffer1;
Serial.println(GPS_buffer1);    
    GPS_ferdig=true;
  }
    GPS_buffer1="";
    }
  }
}
*/

void setup() {
  GPS_buffer.reserve(80);
  buf1.reserve(80);
  USART2_Init( MYUBRR);   //GPS
   
  Serial.begin(115200);
//  GPS.begin(38400);
//  RF_port.begin(19200);
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
  dataFile.print("Programfil: ");
  dataFile.println(__FILE__);
  dataFile.print("Kompiltert den: ");
  dataFile.println(__TIMESTAMP__);

   RF_port.print("Programfil: ");
   RF_port.println(__FILE__);
   RF_port.print("Kompiltert den: ");
   RF_port.println(__TIMESTAMP__);
   RF_port.print("Starttid: ");
//  RF_port.println(DateTime.now());

  Wire.begin();
  accel.initialize();
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  mag.initialize();
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  barometer.initialize();
 Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

 
//l3g4200d_setup();

    while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
    {
      Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
      delay(500);
    }
    Serial.println("Gyroscope initialize and calibrated");
    // gyroscope.calibrate(100);

  dataFile.close();

  for (int g = 0; g < 5; g++)
  {
   }

/*
  Serial.print(" Lest fra SD (");
  Serial.print(buf);
  Serial.println("):");

  dataFile = SD.open(buf, FILE_READ);
  c = 0;
  while (c != -1)
  { c = dataFile.read();
    Serial.print(char(c));
  }
 */ 
 dataFile.flush();

shh ="|";
shh +=char(sh);
}

void loop() {
 dataFile.print(millis()); dataFile.print(sh);dataFile.print(shh);
    RF_port.print(millis());RF_port.print(sh); RF_port.print(shh);

  if (GPS_ferdig)
{
RF_port.print(GPS_buffer);RF_port.print(sh);
dataFile.print(GPS_buffer);dataFile.print(sh);
GPS_ferdig=false;

}
else
{
    adxl345(); dataFile.print(shh);RF_port.print(shh);
    hmc5883l();dataFile.print(shh);RF_port.print(shh);
    l3g4200d_pitch_and_roll();dataFile.print(shh);RF_port.print(shh);
    l3g4200D_simple();RF_port.print(shh);
    BMP085a();
}
    dataFile.println(); RF_port.println();
    dataFile.flush();
}
