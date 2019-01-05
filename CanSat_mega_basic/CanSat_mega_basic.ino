/*
 *  (C) Skule Sørmo 24.02.2015
 */
 
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
//#include <Adafruit_BMP085.h>
#include <SFE_BMP180.h>
SFE_BMP180 pressure;
  char status;
  double T,P,p0,a;
#define ALTITUDE 1655.0 


 File myFile;
 char buf[12]="logg001.txt";
 const int chipSelect = 4;
 String GPS_buffer="",st="";;
 String filename="";
 File datafile; 
 char c;
 boolean gps_komplett=false;
 
#define OUTPUT_READABLE_YAWPITCHROLL
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" 
MPU6050 mpu(0x68); // <-- use for AD0 high
 #define OUTPUT_READABLE_YAWPITCHROLL
 //#define OUTPUT_READABLE_EULER
 //#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_REALACCEL 
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
	
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}	
 
 
#define RF_port Serial 
#define GPS_port Serial2  

void setup() {
 RF_port.begin(115200);
 GPS_port.begin(38400);
 RF_port.println(__FILE__);
 RF_port.print("Kompilert den ");
 RF_port.println(__TIMESTAMP__);
 GPS_buffer.reserve(255); 

 if (pressure.begin())
   Serial.println("BMP180 init success");
  else
  {
	  // Oops, something went wrong, this is usually a connection problem,
	  // see the comments at the top of this sketch for the proper connections.

	  Serial.println("BMP180 init fail\n\n");
	  while(1); // Pause forever.
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

  
// generer filnavn p� logg-fil. Ny fil lages for hver gang programmet starter

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

}



void serialEvent2()  // interupt  drevet lesing av Serial2-port for GPS
{
	  while (GPS_port.available())
	  {
		  
	char	  c=GPS_port.read();
	//	  Serial.print(c);
		  if (int(c)==13)
		  {
			 gps_komplett=true;
		  }
		  else if  (int(c)!=10)
		  GPS_buffer +=c;
	  }
}

boolean sw=false;
int ii=0;



  void loop()
  {
    if (gps_komplett)
	{
     datafile =SD.open(buf, FILE_WRITE);
	 if (GPS_buffer.substring(0,6)=="$GPRMC")
		{
	 RF_port.print("Timestamp: ");
	 RF_port.println(__TIMESTAMP__);	
     RF_port.println(GPS_buffer);
     datafile.print("Timestamp: ");
     datafile.println(__TIMESTAMP__);	 
     datafile.println(GPS_buffer);
	 // datafile.println();
      if (ii++ % 4 == 0)	//leser vekselvis trykk og temperatur for 
       {                    // hver fjerde GPS-lesing
       if (sw)	  
         temp();
       else  
         trykk();
       sw=!sw;
       }
	  datafile.close();
		}
       GPS_buffer="";
       gps_komplett=false;
      }
    }  
    


void temp()
{
	
	  status = pressure.startTemperature();
	  if (status != 0)
	  {
		  // Wait for the measurement to complete:
		  delay(status);

		  // Retrieve the completed temperature measurement:
		  // Note that the measurement is stored in the variable T.
		  // Function returns 1 if successful, 0 if failure.

		  status = pressure.getTemperature(T);
		  if (status != 0)
		  {
			  // Print out the measurement:
			  Serial.print("temperature: ");
			  Serial.print(T,2);
			  Serial.println(" deg C, ");
	//		  Serial.print((9.0/5.0)*T+32.0,2);
	//		  Serial.println(" deg F");
			  datafile.print("temperature: ");
			  datafile.print(T,2);
			  datafile.println(" deg C, ");
	//		  datafile.print((9.0/5.0)*T+32.0,2);
	//		  datafile.println(" deg F");
			  
		  }
	  }
}

void trykk()
{
 status = pressure.startPressure(3);
 if (status != 0)
 {
	 // Wait for the measurement to complete:
	 delay(status);

	 // Retrieve the completed pressure measurement:
	 // Note that the measurement is stored in the variable P.
	 // Note also that the function requires the previous temperature measurement (T).
	 // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
	 // Function returns 1 if successful, 0 if failure.

 status = pressure.getPressure(P,T);
	 if (status != 0)
	 {
		 // Print out the measurement:
		 Serial.print("absolute pressure: ");
		 Serial.print(P,2);
		 Serial.print(" mb, ");
		 Serial.print(P*0.0295333727,2);
		 Serial.println(" inHg");
		 datafile.print("absolute pressure: ");
		 datafile.print(P,2);
		 datafile.print(" mb, ");
		 datafile.print(P*0.0295333727,2);
		 datafile.println(" inHg");
		 

		 // The pressure sensor returns abolute pressure, which varies with altitude.
		 // To remove the effects of altitude, use the sealevel function and your current altitude.
		 // This number is commonly used in weather reports.
		 // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
		 // Result: p0 = sea-level compensated pressure in mb
		 p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
		 Serial.print("relative (sea-level) pressure: ");
		 Serial.print(p0,2);
		 Serial.print(" mb, ");
		 Serial.print(p0*0.0295333727,2);
		 Serial.println(" inHg");

		 // On the other hand, if you want to determine your altitude from the pressure reading,
		 // use the altitude function along with a baseline pressure (sea-level or other).
		 // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
		 // Result: a = altitude in m.

		 a = pressure.altitude(P,p0);
		 Serial.print("computed altitude: ");
		 Serial.print(a,0);
		 Serial.print(" meters, ");
		 Serial.print(a*3.28084,0);
		 Serial.println(" feet");
		 
	     datafile.print("computed altitude: ");
	     datafile.print(a,0);
	     datafile.print(" meters, ");
	     datafile.print(a*3.28084,0);
	     datafile.println(" feet");
}
	 else Serial.println("error retrieving pressure measurement\n");
 }
}
 
