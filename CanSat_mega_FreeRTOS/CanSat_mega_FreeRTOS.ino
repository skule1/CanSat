/*<textarea style="width:700px;height:600px;background-color:#ccffcc;color:#0000ff">
*  FreerTOS versjon av basic CanSat
*  Leser GPS, temperatur og trykk og sender til RF
*  Logging på SD fungerer foreløpig ikke
* (C) Skule Sørmo 7.februar 2012
*
*
*
*
*
*
*
*
*
*
*
*
*************************/

//#include <basic_io_avr.h>
#include <FreeRTOS_AVR.h>

#include <SPI.h>
//#include <SD.h>
#include <SdFat.h> 
#include <Wire.h>
#include <Adafruit_BMP085.h>
//#include <AltSoftSerial.h>   -- fungere ikke sammen med IMU-bibliteker

Adafruit_BMP085 bmp;

 char buf[12]="logg001.txt";
// AltSoftSerial GPS_port; // Tx: Port 9, Rx: Port 8
const int chipSelect = 4;
String GPS_buffer="",st="";;
 String filename="";
 File datafile; 
 char c;
portBASE_TYPE s1, s2,s3,s4;
xQueueHandle Queue,SDQueue;
#define RF_port Serial 
#define GPS_port Serial2  
 String buf1="";
 String buf2="";
SdFat SD;

void setup() {
 RF_port.begin(115200);
 GPS_port.begin(38400);
 buf1.reserve(100);
 buf2.reserve(100);
 RF_port.println(__FILE__);
 RF_port.print("Kompilert den ");
 RF_port.println(__TIMESTAMP__);
 GPS_buffer.reserve(100); 

  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }

  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(10, OUTPUT);

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  
 
int n=1;

 while ((SD.exists(buf)))
{
 st=String(n++);
 if (st.length()==1) st="00"+st;
 else if (st.length()==2) st="0"+st;
 st="logg"+st+".txt"; 
   	st.toCharArray(buf,12) ;  
 Serial.println(buf);	   
}
filename=st;  
Serial.print("Filnavn: ");
Serial.println(buf);
	//	Serial.print("SD: ");
	//	Serial.println(buf);
	buf2="test";
	datafile =SD.open(buf, FILE_WRITE);
	if (datafile)
	{
		datafile.println(__TIMESTAMP__);
		datafile.println(buf2);
		datafile.close();
	}
	else Serial.println("Feilet SD");


Queue = xQueueCreate( 10, 100 );
SDQueue = xQueueCreate( 10, 100 );
s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE+300, NULL,2, NULL); // GPS
s2 = xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE+200, NULL,2, NULL);  // RF send buffer
s3 = xTaskCreate(Thread3, NULL, configMINIMAL_STACK_SIZE+1000, NULL,3, NULL);  // RF send buffer
//s4 = xTaskCreate(Thread4, NULL, configMINIMAL_STACK_SIZE+600, NULL,3, NULL);  // RF send buffer
vTaskStartScheduler();
}


void loop(){}

 static void Thread1(void*rg) // GPS
{
 	 while(1)
	 {
	   while (GPS_port.available())
         { 
          c=GPS_port.read();
         if (int(c)==13)
          {
            xQueueSendToBack(Queue,&GPS_buffer,50);
	         GPS_buffer="";
          }
         else if  (int(c)!=10)
           GPS_buffer +=c;
    }  
   vTaskDelay(10);
  }
  }


static void Thread2(void*rg) // IMU
{
	while(1)
	{
     buf1="Temperature = "+(String(bmp.readTemperature()));
	 xQueueSendToBack(Queue,&buf1,50);
	  
     buf1="Pressure = "+(String(bmp.readPressure()));
     xQueueSendToBack(Queue,&buf1,50);
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
     buf1="Altitude = "+(String(bmp.readAltitude()));
     xQueueSendToBack(Queue,&buf1,50);

     buf1="Pressure sealevel = "+(String(bmp.readSealevelPressure()));
     xQueueSendToBack(Queue,&buf1,50);

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
      buf1="Real altitude = "+(String(bmp.readAltitude(101500)));
      xQueueSendToBack(Queue,&buf1,50);
  
 	 vTaskDelay(200);
}
}


static void Thread3(void*rg) // Queue
{ 
	while(1)
	{ if ( xQueueReceive( Queue, &buf2, 50) )
		{
		Serial.println(buf2);	
    	Serial.println(buf);
        noInterrupts();		
 		datafile =SD.open(buf, FILE_WRITE);
	//	  	 vTaskDelay(500);	 
 	if (datafile)
 		{
	 		datafile.println(__TIMESTAMP__);
	 		datafile.println(buf2);
	 		datafile.close();
 		}
	else
		Serial.println("Ikke SD");
		 
		 interrupts();
//       xQueueSendToBack(SDQueue,&buf1,50);		
	    }
	  	 vTaskDelay(10);
 	}
}

/*

static void Thread4(void*rg) // SD
{
	
	while(1)
	{ if ( xQueueReceive( Queue, &buf2, 50) )
		{
	//	Serial.print("SD: ");	
	//	Serial.println(buf);	
    
    noInterrupts();
	
		datafile =SD.open(buf, FILE_WRITE);
				vTaskDelay(100);
	if (datafile)
		{
		    datafile.println(__TIMESTAMP__);
			datafile.println(buf2);
			datafile.close();
		}
		else
		 Serial.println("Ikke SD");
 
    interrupts();		 
	}
		vTaskDelay(10);

	}
}
*/