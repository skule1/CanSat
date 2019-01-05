/* UART SERIAL DEFINES */
#define BAUD 4800
#define MYUBRR F_CPU/16/BAUD-1

String gpsbuffer="",buf="",buf1="";
bool gps_ferdig;
#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//#include "ADXL345.h"
//#include <Adafruit_Sensor.h>
//#include <Adafruit_ADXL345_U.h>

#include <SoftwareSerial.h>
SoftwareSerial mySerial(8, 9); // RX, TX
#define RF mySerial
//ADXL345 accel;
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
int16_t ax, ay, az;
int16_t xoff,yoff,zoff;
int16_t Xg, Yg, Zg;
#include <L3G4200D.h>
L3G4200D gyro;

#include "BMP085.h"
BMP085 barometer;

float temperature;
float pressure;
float altitude1;
int32_t lastMicros;
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

int refnr=0;
#include <HMC5883L.h>
HMC5883L compass;
float heading;

//#include <SPI.h>
//#include <SD.h>
  //File dataFile ;
const int chipSelect = 4;

//  #include <Adafruit_Sensor.h>
//#include <Adafruit_ADXL345_U.h>
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
//int xoff,yoff,zoff;

void setup() {
  buf1.reserve(100);
  gpsbuffer.reserve(100);
  RF.begin(19200);
  RF.println("test");
  // put your setup code here, to run once:
USART_Init(MYUBRR);

    Wire.begin();
// accel.initialize();

/*
// Initialise the sensor 
  if(!accel.begin())
  {
    // There was a problem detecting the ADXL345 ... check your connections
    RF.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

// Set the range to whatever is appropriate for your project 
  accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);
  
  // Display some basic information on this sensor 
  displaySensorDetails();
  
  // Display additional settings (outside the scope of sensor_t) 
  displayDataRate();
  displayRange();
 // Serial.println("");
    sensors_event_t event; 
  accel.getEvent(&event);
  xoff=event.acceleration.x;
  yoff=event.acceleration.y;
  zoff=event.acceleration.z;



 // RF.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
 //   accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);
/*  
  // Display some basic information on this sensor
  displaySensorDetails();
  
  // Display additional settings (outside the scope of sensor_t) 
  displayDataRate();
  displayRange();
 // Serial.println("");
    sensors_event_t event; 
  accel.getEvent(&event);
  xoff=event.acceleration.x;
  yoff=event.acceleration.y;
  zoff=event.acceleration.z;
*/
//    accel.getAcceleration(&ax, &ay, &az);
//      accel.getEvent(&event);
//  xoff=ax;
 // yoff=ay;
//  zoff=az;
  gyro.enableDefault();
   RF.println("Initializing barometer...");
    barometer.initialize();
        RF.println("Testing device connections...");
    RF.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

  // Initialize Initialize HMC5883L
  RF.println("Initialize HMC5883L");
  while (!compass.begin())
  {
  RF.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
RF.print("Kompass (heading,kurs,x,y,z) | ");
RF.print("Gyro(x,y,x) | ");
RF.print("Acc  (x,y,z,pitch,roll) | ");
RF.println("Barometer (temo,trykk,hoyde) ");
/*
  RF.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    RF.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  RF.println("card initialized.");
  */
}







void loop() {

 //    dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (gps_ferdig)
  {
//  RF.print("GPS: ");
  RF.println(gpsbuffer);
  gps_ferdig=false;
  }
 else
 {
    compas();
    Gyro() ;
//    Acc();
    Barometer();
  //
 }
  //  dataFile.close();
}



// SETUP UART
void USART_Init( unsigned int ubrr)
{
   //Set baud rate 
   UBRR0H = (unsigned char)(ubrr>>8);
   UBRR0L = (unsigned char)ubrr;
   
  //Enable receiver and transmitter
//   UCSR0B = (1<<RXEN0)|(1<<TXEN0);
   UCSR0B = (1<<RXCIE0)|(1<<RXEN0);
   RF.println(UCSR0B,HEX);
   
   // Set frame format: 8data, 2stop bit
   UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}


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

/*

void Acc() 
{
    double pitch, roll, Xg, Yg, Zg;
    double fXg = 0;
double fYg = 0;
double fZg = 0;
const float alpha = 0.5;
  // Get a new sensor event 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  // Display the results (acceleration is measured in m/s^2)
//  Serial.print("X: "); Serial.print(event.acceleration.x-xoff); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(event.acceleration.y-yoff); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(event.acceleration.z-zoff); Serial.print("  ");Serial.println("m/s^2 ");

 Xg= event.acceleration.x-xoff;
 Yg= event.acceleration.y-xoff;
 Zg= event.acceleration.z-xoff;
    RF.print(Xg); RF.print(", ");
    RF.print(Yg); RF.print(", ");
    RF.print(Zg); RF.print(", ");


  //acc.read(&Xg, &Yg, &Zg);

  //Low Pass Filter
  fXg = Xg * alpha + (fXg * (1.0 - alpha));
  fYg = Yg * alpha + (fYg * (1.0 - alpha));
  fZg = Zg * alpha + (fZg * (1.0 - alpha));

  //Roll & Pitch Equations
  roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
  pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

  RF.print(pitch);
   RF.print(", ");
  RF.print(roll);
    RF.print(",|, "); 
//   dataFile.print(event.acceleration.x-xoff); dataFile.print(", ");
//    dataFile.print(event.acceleration.y-yoff); dataFile.print(", ");
//    dataFile.print(event.acceleration.z-zoff); dataFile.print("/n");
 
}
*/
/*
void acc() {
    double pitch, roll, Xg, Yg, Zg;
    double fXg = 0;
double fYg = 0;
double fZg = 0;  
const float alpha = 0.5;
    // read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);

 Xg= ax-xoff;
 Yg= ay-xoff;
 Zg= az-xoff;
    RF.print(Xg); RF.print(", ");
    RF.print(Yg); RF.print(", ");
    RF.print(Zg); RF.print(", ");


  //acc.read(&Xg, &Yg, &Zg);

  //Low Pass Filter
  fXg = Xg * alpha + (fXg * (1.0 - alpha));
  fYg = Yg * alpha + (fYg * (1.0 - alpha));
  fZg = Zg * alpha + (fZg * (1.0 - alpha));

  //Roll & Pitch Equations
  roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
  pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

  RF.print(pitch);
   RF.print(", ");
  RF.print(roll);
    RF.print(",|, "); 
    /*
    // display tab-separated accel x/y/z values
    RF.print("accel:\t");
    RF.print(ax); RF.print("\t");
    RF.print(ay); RF.print("\t");
    RF.println(az);

    // blink LED to indicate activity
  //  blinkState = !blinkState;
 //   digitalWrite(LED_PIN, blinkState);

}

*/
void Gyro() {
  gyro.read();

 // Serial.print("G ");
 // Serial.print("X: ");
  RF.print((int)gyro.g.x);// dataFile.print((int)gyro.g.x);
   RF.print(", ");//dataFile.print(", ");
// Serial.print(" Y: ");
  RF.print((int)gyro.g.y);// dataFile.print((int)gyro.g.y);
  RF.print(", ");//ataFile.print(", ");
//  Serial.print(" Z: ");
  RF.print((int)gyro.g.z); //dataFile.print((int)gyro.g.z);
  RF.print(",|, ");//dataFile.println(",|, ");

 // delay(100);
}

void Barometer() {
    // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    // wait appropriate time for conversion (4.5ms delay)
    lastMicros = micros();
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();

    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();

    // calculate absolute altitude in meters based on known pressure
    // (may pass a second "sea level pressure" parameter here,
    // otherwise uses the standard value of 101325 Pa)
    altitude1 = barometer.getAltitude(pressure);

    // display measured values if appropriate
 /*   Serial.print("T/P/A\t");
    Serial.print(temperature); Serial.print("\t");
    Serial.print(pressure); Serial.print("\t");
    Serial.print(altitude);
    Serial.println("");
  */  
    RF.print(temperature); RF.print(", ");
    RF.print(pressure); RF.print(", ");
  RF.print(altitude1);RF.println("");
 //   Serial.println("");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
    // delay 100 msec to allow visually parsing blink and any serial output
    delay(100);
}


void compas()
{ 
  Vector norm = compass.readNormalize(); 

  // Calculate heading
heading= atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  // Output
 // Serial.print(" Heading = ");
  RF.print(heading);  //dataFile.print(heading);
//  Serial.print(" Degress = ");
  RF.print(", ");  
  RF.print(headingDegrees);
  RF.print(", ");

   Vector raw = compass.readRaw();
//  Vector norm = compass.readNormalize();


/*  RF.print(raw.XAxis);
  RF.print(", ");
  RF.print(raw.YAxis);
  RF.print(", ");
  RF.print(raw.ZAxis);
  RF.print(", ");
*/
  RF.print(norm.XAxis);
  RF.print(", ");;
  RF.print(norm.YAxis);
  RF.print(", ");
  RF.print(norm.ZAxis);
  RF.print(",|, ");

//  Serial.println();

 // delay(100);
}



/*


void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  RF.println("------------------------------------");
  RF.print  ("Sensor:       "); RF.println(sensor.name);
  RF.print  ("Driver Ver:   "); RF.println(sensor.version);
  RF.print  ("Unique ID:    "); RF.println(sensor.sensor_id);
  RF.print  ("Max Value:    "); RF.print(sensor.max_value); RF.println(" m/s^2");
  RF.print  ("Min Value:    "); RF.print(sensor.min_value); RF.println(" m/s^2");
  RF.print  ("Resolution:   "); RF.print(sensor.resolution); RF.println(" m/s^2");  
  RF.println("------------------------------------");
  RF.println("");
  delay(500);
}

void displayDataRate(void)
{
  RF.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      RF.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      RF.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      RF.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      RF.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      RF.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      RF.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      RF.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      RF.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      RF.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      RF.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      RF.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      RF.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      RF.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      RF.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      RF.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      RF.print  ("0.10 "); 
      break;
    default:
      RF.print  ("???? "); 
      break;
  }  
  RF.println(" Hz");  
}

void displayRange(void)
{
  RF.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      RF.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      RF.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      RF.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      RF.print  ("2 "); 
      break;
    default:
      RF.print  ("?? "); 
      break;
  }  
  RF.println(" g");  
}



void Acc() 
{
    double pitch, roll, Xg, Yg, Zg;
    double fXg = 0;
double fYg = 0;
double fZg = 0;
const float alpha = 0.5;
  // Get a new sensor event 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  // Display the results (acceleration is measured in m/s^2)
//  Serial.print("X: "); Serial.print(event.acceleration.x-xoff); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(event.acceleration.y-yoff); Serial.print("  ");
 // Serial.print("Z: "); Serial.print(event.acceleration.z-zoff); Serial.print("  ");Serial.println("m/s^2 ");

 Xg= event.acceleration.x-xoff;
 Yg= event.acceleration.y-xoff;
 Zg= event.acceleration.z-xoff;
    RF.print(Xg); RF.print(", ");
    RF.print(Yg); RF.print(", ");
    RF.print(Zg); RF.print(", ");


  //acc.read(&Xg, &Yg, &Zg);

  //Low Pass Filter
  fXg = Xg * alpha + (fXg * (1.0 - alpha));
  fYg = Yg * alpha + (fYg * (1.0 - alpha));
  fZg = Zg * alpha + (fZg * (1.0 - alpha));

  //Roll & Pitch Equations
  roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
  pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

  RF.print(pitch);
   RF.print(", ");
  RF.print(roll);
    RF.print(",|, "); 
//   dataFile.print(event.acceleration.x-xoff); dataFile.print(", ");
//    dataFile.print(event.acceleration.y-yoff); dataFile.print(", ");
//    dataFile.print(event.acceleration.z-zoff); dataFile.print("/n");
 
}
*/
