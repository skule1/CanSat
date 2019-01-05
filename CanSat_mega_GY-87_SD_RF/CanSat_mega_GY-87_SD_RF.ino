#include <SPI.h>
#include <SD.h>
const int chipSelect = 4;
File dataFile;

#define GPS Serial2
#define RF Serial

String GPS_buffer1="",GPS_buffer="";
boolean GPS_ferdig=false;


void setup() {
  GPS_buffer.reserve(100);
  GPS_buffer1.reserve(100);
GPS.begin(38400);
Serial.begin(115200);
//RF.begin(9600);
Serial.println(__FILE__);
Serial.print("Kompilert den ");
Serial.println(__TIMESTAMP__);

 if (!SD.begin(chipSelect))
 Serial.println("Card failed, or not present");
else 
  Serial.println("card initialized.");

  File dataFile = SD.open("datalog.txt", FILE_WRITE);



}
void loop(){}

void loop2() {
if (GPS_ferdig)
{
      RF.print(millis());RF.print('\t');
    dataFile.print(millis());dataFile.print('\t');
    

RF.print(GPS_buffer);RF.print('\t');
dataFile.print(GPS_buffer);dataFile.print('\t');
GPS_ferdig=false;

}
else
{
  
}
RF.println();
dataFile.println();
dataFile.flush();
}




void serialEvent2()
{
  if (GPS.available())
  {
    char c=GPS.read();
    if ((c!=char(10) && (c!=char(13))))
     GPS_buffer1 +=c;
 //   Serial.println(GPS_buffer1);
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

