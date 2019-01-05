#include <SdFat.h>

#include <Wire.h>

//#include <Adafruit_BMP085.h>
#include <delay.h>
#include <SFE_BMP180.h>
#include <AltSoftSerial.h>
/*float altitude;
uint32_t pressure;
uint16_t temperature;*/
SFE_BMP180 pressure;


/*
*
*  Denne program-skissen inneholder de grunnleggende funksjonene i CanSat som kan utf�res p� en UNO eller MINI
* 
*   Lese GPS og sende det tilbakken over RF
*   Lese trykk og temperatur
*   Lagre data p� SD-minne for senere utskrift til kmd-fil til Google Earth og pltt i Excel
*
*
*TSD-card:
* analog sensors on analog ins 0, 1, and 2
* SD card attached to SPI bus as follows:
* MOSI - pin 11
* MISO - pin 12
* CLK - pin 13
* CS - pin 4
*
*  GY-87:  MPU6050+HMC5883L+BMP180,
*  (gyro,akselerasjon,magnetisk, trykk, temperatur)
* HMC5883L: compass;    //SDA: A4, SCL: A5
* BMP180:   trykk,temp,h�yde (BMP085)
* MPU6050:  akselerator, gyro  
*/
#include <SPI.h>
  char status;
  double T,P,p0,a;

AltSoftSerial GPS_port; // Tx: Port 9, Rx: Port 8
const int chipSelect = 4;

//Adafruit_BMP085 bmp;
String GPS_buffer="";
  String filename="";
// File system object.
SdFat sd;
// Log file.
SdFile dataFile;
String st;
//const uint8_t chipSelect = 10;
char c;
#define RF_port Serial

void setup()
{
 RF_port.begin(9600);
 GPS_port.begin(4800);
 RF_port.println(__FILE__);
 RF_port.print("Kompiltert den ");
 RF_port.println(__TIMESTAMP__);
GPS_buffer.reserve(255); 

	   
// Sjekker sensor for trykk og temperatur:
if (!bmp.begin()) {
   Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	   };	   

 // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) 
	Serial.println("Card failed, or not present");
   else    
       Serial.println("card initialized.");

  int n=1;
  char buf[12]="logg001.txt";

  while (!(sd.exists(buf)))
{
 st=String(n);
 if (st.length()==1) st="00"+st;
 else if (st.length()==2) st="0"+st;
 st="logg"+st+".txt"; 
   	st.toCharArray(buf,12) ;  
}
filename=st;  
	   
 //    dataFile = sd.open(buf, O_CREAT | O_WRITE | O_EXCL);//) error("file.open");
  if (!dataFile.open(buf, O_CREAT | O_WRITE | O_EXCL)) Serial.println("file.open");

}

void loop()
{
while (GPS_port.available())
 c=GPS_port.read();
 if (int(c)==13)
 {
 	 RF_port.println(GPS_buffer);
	 dataFile.println(GPS_buffer);
    Temp_pressure();
    Temp_pressure_SD();
   // sd.close;

 }
 else if (int(c)!=10)
  GPS_buffer +=c;
}


void Temp_pressure() {

	pressure.getTemperature(temperature);
    pressure.getPressure(P,T);
	//   hoyde=bmp.readAltitude();
	//   hoyde1=bmp.readAltitude(101500);

//	#ifdef debug
	RF_port.print("Temperature = ");
	RF_port.print(bmp.readTemperature());
	RF_port.println(" *C");
	
	RF_port.print("Pressure = ");
	RF_port.print(bmp.readPressure());
	RF_port.println(" Pa");
	
	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	RF_port.print("Altitude = ");
	RF_port.print(bmp.readAltitude());
	RF_port.println(" meters");
	

	// you can get a more precise measurement of altitude
	// if you know the current sea level pressure which will
	// vary with weather and such. If it is 1015 millibars
	// that is equal to 101500 Pascals.
	RF_port.print("Real altitude = ");
	RF_port.print(bmp.readAltitude(101500));
	RF_port.println(" meters");
	
	RF_port.println();
	//   delay(500);
//	#endif
}


void Temp_pressure_SD() {
	temperature=bmp.readTemperature();
	pressure=bmp.readPressure();
	//   hoyde=bmp.readAltitude();
	//   hoyde1=bmp.readAltitude(101500);

	//	#ifdef debug
	dataFile.print("Temperature = ");
	dataFile.print(bmp.readTemperature());
	dataFile.println(" *C");
	
	dataFile.print("Pressure = ");
	dataFile.print(bmp.readPressure());
	dataFile.println(" Pa");
	
	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	dataFile.print("Altitude = ");
	dataFile.print(bmp.readAltitude());
	dataFile.println(" meters");
	

	// you can get a more precise measurement of altitude
	// if you know the current sea level pressure which will
	// vary with weather and such. If it is 1015 millibars
	// that is equal to 101500 Pascals.
	dataFile.print("Real altitude = ");
	dataFile.print(bmp.readAltitude(101500));
	dataFile.println(" meters");
	
	dataFile.println();
	//   delay(500);
	//	#endif
}



void loop()
{
	char status;
	double T,P,p0,a;

	// Loop here getting pressure readings every 10 seconds.

	// If you want sea-level-compensated pressure, as used in weather reports,
	// you will need to know the altitude at which your measurements are taken.
	// We're using a constant called ALTITUDE in this sketch:
	
	Serial.println();
	Serial.print("provided altitude: ");
	Serial.print(ALTITUDE,0);
	Serial.print(" meters, ");
	Serial.print(ALTITUDE*3.28084,0);
	Serial.println(" feet");
	
	// If you want to measure altitude, and not pressure, you will instead need
	// to provide a known baseline pressure. This is shown at the end of the sketch.

	// You must first get a temperature measurement to perform a pressure reading.
	
	// Start a temperature measurement:
	// If request is successful, the number of ms to wait is returned.
	// If request is unsuccessful, 0 is returned.

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
			Serial.print(" deg C, ");
			Serial.print((9.0/5.0)*T+32.0,2);
			Serial.println(" deg F");
			
			// Start a pressure measurement:
			// The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
			// If request is successful, the number of ms to wait is returned.
			// If request is unsuccessful, 0 is returned.

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
				}
				else Serial.println("error retrieving pressure measurement\n");
			}
			else Serial.println("error starting pressure measurement\n");
		}
		else Serial.println("error retrieving temperature measurement\n");
	}
	else Serial.println("error starting temperature measurement\n");

	delay(5000);  // Pause for 5 seconds.
}