#include <SPI.h>
#include <SD.h>
#include <Wire.h>
//#include <Adafruit_BMP085.h>
#include <SFE_BMP180.h>
SFE_BMP180 pressure;
char status;
double T,P,p0,a;
#define ALTITUDE 1655.0
boolean kgps=true,kaks=true,kgyro=true,ktemp=true,ktrykk=true,kmagn=true;

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
#include "HMC5883L.h"
HMC5883L mag;
int16_t mx, my, mz;
#define LED_PIN 13
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_REALACCEL
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
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

#define RF_port Serial
#define GPS_port Serial2
String kommando="";

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
	
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
   Wire.begin();
   TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
   #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
   Fastwire::setup(400, true);
   #endif
   
       mag.initialize();
    // verify connection
    Serial.println("Testing magnetic device connections...");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);	   
   
	    Serial.println(F("Initializing I2C devices..."));


  mpu.initialize();
  mpu.setI2CMasterModeEnabled(0);
  mpu.setI2CBypassEnabled(1);	
   // verify connection
   Serial.println(F("Testing device connections..."));
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

   // wait for ready
 //  while (Serial.available()); // empty buffer
 //  while (!Serial.available());                 // wait for data
  // while (Serial.available()); // empty buffer again

   // load and configure the DMP
   Serial.println(F("Initializing DMP..."));
   devStatus = mpu.dmpInitialize();

   // supply your own gyro offsets here, scaled for min sensitivity
   mpu.setXGyroOffset(220);
   mpu.setYGyroOffset(76);
   mpu.setZGyroOffset(-85);
   mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

   // make sure it worked (returns 0 if so)
   if (devStatus == 0) {
	   // turn on the DMP, now that it's ready
	   Serial.println(F("Enabling DMP..."));
	   mpu.setDMPEnabled(true);

	   // enable Arduino interrupt detection
	   Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
	   attachInterrupt(0, dmpDataReady, RISING);
	   mpuIntStatus = mpu.getIntStatus();

	   // set our DMP Ready flag so the main loop() function knows it's okay to use it
	   Serial.println(F("DMP ready! Waiting for first interrupt..."));
	   dmpReady = true;

	   // get expected DMP packet size for later comparison
	   packetSize = mpu.dmpGetFIFOPacketSize();
	   } else {
	   // ERROR!
	   // 1 = initial memory load failed
	   // 2 = DMP configuration updates failed
	   // (if it's going to break, usually the code will be 1)
	   Serial.print(F("DMP Initialization failed (code "));
	   Serial.print(devStatus);
	   Serial.println(F(")"));
   }

   // configure LED for output
   // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
   // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
   // while (Serial.available() && Serial.read()); // empty buffer
   // while (!Serial.available());                 // wait for data
   // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
	    mag.initialize();

	    // verify connection
	    Serial.println("Testing device connections...");
	    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

	    // configure Arduino LED for
	    pinMode(LED_PIN, OUTPUT);
	
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

//boolean kgps=true,kaks=true,kgyro=true,ktemp=true,ktrykk=true,kmagn=true;

void loop()
{
	while (RF_port.available())
	{
		c=RF_port.read();
		if (int(c)==13)
		{
			Serial.print("Kommando: ");
			Serial.println(kommando);
 	      if (kommando=="kgps") 	kgps=true;
          else if (kommando=="nkgps") 	kgps=false;		  
	else if (kommando=="kaks") 	kaks=true;
	else if (kommando=="nkaks") 	kaks=false;
	else if (kommando=="kgyro") 	kgyro=true;
	else if (kommando=="nkgyro") 	kgyro=false;
	else if (kommando=="ktemp") 	ktemp=true;
	else if (kommando=="nktemp") 	ktemp=false;
	else if (kommando=="ktrykk") 	ktrykk=true;
	else if (kommando=="nktrykk") 	ktrykk=false;
	else if (kommando=="kmagn") 	kmagn=true;
	else if (kommando=="nkmagn") 	kmagn=false;
//	else if (kommando=="angyro") 	{ kmagn=false;
		
	kommando="";
		}
	else if (int(c)!=10)
	  kommando +=c;	
	}
	if ((gps_komplett) &&(kgps))
	{
		datafile =SD.open(buf, FILE_WRITE);
	//	if (kgps && 
     //	Serial.println(GPS_buffer.substring(0,6));
     if (GPS_buffer.substring(0,6)=="$GPRMC")
		{
			RF_port.print("Timestamp: ");
			RF_port.println(__TIMESTAMP__);
			RF_port.println(GPS_buffer);
			datafile.print("Timestamp: ");
			datafile.println(__TIMESTAMP__);
			datafile.println(GPS_buffer);

     		if ((kgyro)||(kaks)) gyro();
			if (kmagn){delay(100); magn();}

			if ((ktemp) && (ktrykk) && (ii++ % 2 == 0))	//leser vekselvis trykk og temperatur for
			{                    // hver fjerde GPS-lesing
				if (sw)
				temp();
				else
				trykk();
				sw=!sw;
			}
		} // ferdig ned gpstrykk) trkgps);
			GPS_buffer="";
			gps_komplett=false;	
		    datafile.close();
 			}
		if (!kgps)
		{
		datafile =SD.open(buf, FILE_WRITE);
 		RF_port.print("Timestamp: ");
		RF_port.println(__TIMESTAMP__);
			datafile.print("Timestamp: ");
			datafile.println(__TIMESTAMP__);
		if ((kgyro) || (kaks)) gyro();
			if (kmagn) magn();
			if (kgyro) gyro();
			if (kmagn) magn();
  			if (ktemp) temp();		
			if (ktrykk) trykk(); 
			datafile.close();
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
			Serial.print("temperature:   (deg C)  ");
			Serial.println(T,2);
	//		Serial.println(" deg C, ");
			//		  Serial.print((9.0/5.0)*T+32.0,2);
			//		  Serial.println(" deg F");
			datafile.print("temperature:   (deg C)");
			datafile.println(T,2);
//			datafile.println(" deg C, ");
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
			Serial.print("absolute pressure:  (mb) ");
			Serial.println(P,2);
			Serial.print("absolute pressure:  (inHg) ");
			Serial.println(P*0.0295333727,2);
	//		Serial.println(" inHg");
			datafile.print("absolute pressure:   (mb");
			datafile.println(P,2);
	     	datafile.print("absolute pressure:   (inHg");
			datafile.println(P*0.0295333727,2);
		//	datafile.println(" inHg");
			

			// The pressure sensor returns abolute pressure, which varies with altitude.
			// To remove the effects of altitude, use the sealevel function and your current altitude.
			// This number is commonly used in weather reports.
			// Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
			// Result: p0 = sea-level compensated pressure in mb
			p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
			Serial.print("relative (sea-level) pressure:  (mb)  ");
			Serial.println(p0,2);
			Serial.print("relative (sea-level) pressure:  (inHg)  ");
			Serial.println(p0*0.0295333727,2);
//			Serial.println(" inHg");

			// On the other hand, if you want to determine your altitude from the pressure reading,
			// use the altitude function along with a baseline pressure (sea-level or other).
			// Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
			// Result: a = altitude in m.

			a = pressure.altitude(P,p0);
			Serial.print("computed altitude: (meters) ");
	  		Serial.println(a,0);
//	        Serial.print("computed altitude: (feet) ");
	//		Serial.println(a*3.28084,0);
	//		Serial.println(" feet");
			
			datafile.print("computed altitude: (meters) ");
			datafile.println(a,0);
	//		datafile.print("computed altitude: (feet) ");
	//		datafile.println(a*3.28084,0);
	//		datafile.println(" feet");
		}
		else Serial.println("error retrieving pressure measurement\n");
	}
}

void gyro()
{
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	  // reset so we can continue cleanly
	  mpu.resetFIFO();
	  Serial.println(F("FIFO overflow! (gyro)"));

	  // otherwise, check for DMP data ready interrupt (this should happen frequently)
	  } else if (mpuIntStatus & 0x02) {
	  // wait for correct available data length, should be a VERY short wait
	  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	  // read a packet from FIFO
	  mpu.getFIFOBytes(fifoBuffer, packetSize);
	  
	  // track FIFO count here in case there is > 1 packet available
	  // (this lets us immediately read more without waiting for an interrupt)
	  fifoCount -= packetSize;

	  #ifdef OUTPUT_READABLE_QUATERNION
	  // display quaternion values in easy matrix form: w x y z
	  mpu.dmpGetQuaternion(&q, fifoBuffer);
	  Serial.print("quat\t");
	  Serial.print(q.w);
	  Serial.print("\t");
	  Serial.print(q.x);
	  Serial.print("\t");
	  Serial.print(q.y);
	  Serial.print("\t");
	  Serial.println(q.z);
	  #endif

	  #ifdef OUTPUT_READABLE_EULER
	  // display Euler angles in degrees
	  mpu.dmpGetQuaternion(&q, fifoBuffer);
	  mpu.dmpGetEuler(euler, &q);
	  Serial.print("euler\t");
	  Serial.print(euler[0] * 180/M_PI);
	  Serial.print("\t");
	  Serial.print(euler[1] * 180/M_PI);
	  Serial.print("\t");
	  Serial.println(euler[2] * 180/M_PI);
	  #endif
if (kgyro)
{
	  #ifdef OUTPUT_READABLE_YAWPITCHROLL
	  // display Euler angles in degrees
	  mpu.dmpGetQuaternion(&q, fifoBuffer);
	  mpu.dmpGetGravity(&gravity, &q);
	  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	  Serial.print("ypr\t");
	  Serial.print(ypr[0] * 180/M_PI);
	  Serial.print("\t");
	  Serial.print(ypr[1] * 180/M_PI);
	  Serial.print("\t");
	  Serial.println(ypr[2] * 180/M_PI);
	  #endif
}
if (kaks)
{
	  #ifdef OUTPUT_READABLE_REALACCEL
	  // display real acceleration, adjusted to remove gravity
	  mpu.dmpGetQuaternion(&q, fifoBuffer);
	  mpu.dmpGetAccel(&aa, fifoBuffer);
	  mpu.dmpGetGravity(&gravity, &q);
	  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	  Serial.print("areal\t");
	  Serial.print(aaReal.x);
	  Serial.print("\t");
	  Serial.print(aaReal.y);
	  Serial.print("\t");
	  Serial.println(aaReal.z);
	  #endif
}
	  #ifdef OUTPUT_READABLE_WORLDACCEL
	  // display initial world-frame acceleration, adjusted to remove gravity
	  // and rotated based on known orientation from quaternion
	  mpu.dmpGetQuaternion(&q, fifoBuffer);
	  mpu.dmpGetAccel(&aa, fifoBuffer);
	  mpu.dmpGetGravity(&gravity, &q);
	  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	  Serial.print("aworld\t");
	  Serial.print(aaWorld.x);
	  Serial.print("\t");
	  Serial.print(aaWorld.y);
	  Serial.print("\t");
	  Serial.println(aaWorld.z);
	  #endif
	  
	  #ifdef OUTPUT_TEAPOT
	  // display quaternion values in InvenSense Teapot demo format:
	  teapotPacket[2] = fifoBuffer[0];
	  teapotPacket[3] = fifoBuffer[1];
	  teapotPacket[4] = fifoBuffer[4];
	  teapotPacket[5] = fifoBuffer[5];
	  teapotPacket[6] = fifoBuffer[8];
	  teapotPacket[7] = fifoBuffer[9];
	  teapotPacket[8] = fifoBuffer[12];
	  teapotPacket[9] = fifoBuffer[13];
	  Serial.write(teapotPacket, 14);
	  teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
	  #endif

	  // blink LED to indicate activity
	  blinkState = !blinkState;
	  digitalWrite(LED_PIN, blinkState);
  }
  }


  void magn()
  {
  mpu.setI2CMasterModeEnabled(0);
  mpu.setI2CBypassEnabled(1)	;     // read raw heading measurements from device
	     mag.getHeading(&mx, &my, &mz);

	     // display tab-separated gyro x/y/z values
	     Serial.print("mag:\t");
	     Serial.print(mx); Serial.print("\t");
	     Serial.print(my); Serial.print("\t");
	     Serial.print(mz); Serial.println("\t");
	     
	     // To calculate heading in degrees. 0 degree indicates North
	     float heading = atan2(my, mx);
	     if(heading < 0)
	     heading += 2 * M_PI;
	     Serial.print("heading:\t");
	     Serial.println(heading * 180/M_PI);

	     // blink LED to indicate activity
	     blinkState = !blinkState;
	     digitalWrite(LED_PIN, blinkState);
     
  }
