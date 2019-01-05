
#include <HMC5883L.h>
#define RF Serial
#define GPS Serial2
/*
  HMC5883L Triple Axis Digital Compass. Compass Example.
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/
// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
#define DHTPIN 2     // what digital pin we're connected to

#include <Wire.h>

#include "DHT.h"
#include <L3G4200D.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "FloatToString.h"
#include <SPI.h>
#include <SD.h>
#include "I2Cdev.h"
#include "BMP085.h"
BMP085 barometer;

HMC5883L compass;
String gpsbuffer="",buf="",buf1="";
DHT dht(DHTPIN, DHTTYPE);
L3G4200D gyro;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
int xoff,yoff,zoff;
float heading;
const int chipSelect = 4;
File dataFile ;

float temperature;
float pressure;
float altitude1;
int32_t lastMicros;
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
bool gps_ferdig=false;

void setup()
{
  buf1.reserve(100);
  gpsbuffer.reserve(100);
  
//  Serial.begin(115200);
  GPS.begin(4800);
  RF.begin(19200);
  RF.println("hei");
  Wire.begin();
  gyro.enableDefault();
  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
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
    dht.begin();

    Serial.println("Initializing barometer...");
    barometer.initialize();
        Serial.println("Testing device connections...");
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

    // configure LED pin for activity indication
    pinMode(LED_PIN, OUTPUT);

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
 // Serial.println("");
    sensors_event_t event; 
  accel.getEvent(&event);
  xoff=event.acceleration.x;
  yoff=event.acceleration.y;
  zoff=event.acceleration.z;
  
  Serial.print("Initializing dataFile card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

}


void serialEvent2() {
  if (GPS.available())
  {
    char c=GPS.read();
 //      Serial.print(c);
      if (int(c)==13)
       {
//    Serial.println(buf1.substring(0,6));
  if (buf1.substring(0,6)=="$GPRMC")
   {
      gpsbuffer=buf1;
      gps_ferdig=true;
   }
       buf1=""; 
    }   
     else if ((int(c)!=10) && (int(c)!=13))
     {
      buf1 +=c;
Serial.println(buf1);
     }



  }
}

void loop()
{
Serial.println(gps_ferdig);
 //   dataFile = SD.open("datalog.txt", FILE_WRITE  );
 //  serial2Event();
 if (gps_ferdig==true)
   {
   Serial.println(gpsbuffer);
   gps_ferdig=false;
   }
   else
   {
/* compas();
 dht_fuktighet() ;
  Gyro() ;
  Acc() ;
    lys();
    */
  Barometer();
   }
//dataFile.close();

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
  RF.print(heading);  dataFile.print(heading);
//  Serial.print(" Degress = ");
  RF.print(", ");  dataFile.print(", ");
  RF.print(headingDegrees);
  RF.print(",|, ");  dataFile.print(",|, ");
//  Serial.println();

 // delay(100);
}



void dht_fuktighet() {
  // Wait a few seconds between measurements.
 // delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  //Serial.print("Humidity: ");
  RF.print(h);  dataFile.print(h);
   RF.print(", ");dataFile.print(", ");
// Serial.print(" %\t");
//  Serial.print("Temperature: ");
  RF.print(t);  dataFile.print(t);
   RF.print(", ");dataFile.print(", ");
// Serial.print(" *C ");
  RF.print(f);  dataFile.print(f);
  RF.print(", ");dataFile.print(", ");
//  Serial.print(" *F\t");
//  Serial.print("Heat index: ");
  RF.print(hic);  dataFile.print(hic);
  RF.print(", ");dataFile.print(", ");
//  Serial.print(" *C ");
  RF.print(hif);  dataFile.print(hif);
 // Serial.println(" *F");
   RF.print(",|, ");  dataFile.print(",|, ");
}

void Gyro() {
  gyro.read();

 // Serial.print("G ");
 // Serial.print("X: ");
  RF.print((int)gyro.g.x);  dataFile.print((int)gyro.g.x);
   RF.print(", ");dataFile.print(", ");
// Serial.print(" Y: ");
  RF.print((int)gyro.g.y);  dataFile.print((int)gyro.g.y);
  RF.print(", ");dataFile.print(", ");
//  Serial.print(" Z: ");
  RF.print((int)gyro.g.z); dataFile.print((int)gyro.g.z);
  RF.print(",|, ");dataFile.print(",|, ");

 // delay(100);
}



void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}



void Acc() 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
/*  Serial.print("X: "); Serial.print(event.acceleration.x-xoff); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y-yoff); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z-zoff); Serial.print("  ");Serial.println("m/s^2 ");
 */ RF.print(event.acceleration.x-xoff); RF.print(", ");
    RF.print(event.acceleration.y-yoff); RF.print(", ");
    RF.print(event.acceleration.z-zoff); RF.print(",|, ");
//   dataFile.print(event.acceleration.x-xoff); dataFile.print(", ");
//    dataFile.print(event.acceleration.y-yoff); dataFile.print(", ");
//    dataFile.print(event.acceleration.z-zoff); dataFile.print("/n");
 
}
void lys()
{
  RF.print(analogRead(A1));
  RF.print(",|,  ");
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
