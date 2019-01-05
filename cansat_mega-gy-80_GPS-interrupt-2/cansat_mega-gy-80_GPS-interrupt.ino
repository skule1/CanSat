#include <BMP085.h>

#include <HMC5883L.h>
#define RF Serial
#define GPS Serial
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

//#include "DHT.h"




#include <L3G4200D.h>
//#include <L3GD20H.h>
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

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            2         // Pin which is connected to the DHT sensor.

// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11 
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)
  sensor_t sensor;





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
int refnr=0;

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
      gpsbuffer=buf1;
      gps_ferdig=true;
   }
       buf1=""; 
    }   
     else if ((int(c)!=10) && (int(c)!=13))
     {
      buf1 +=c;
//Serial.println(buf1);
     }

} 


void setup()
{
  buf1.reserve(100);
  gpsbuffer.reserve(100);
 USART2_Init( MYUBRR);   //GPS
 
//  Serial.begin(115200);
//  GPS.begin(4800);
  RF.begin(19200);
  RF.println("hei");
  Wire.begin();


  // Set scale 2000 dps and 400HZ Output data rate (cut-off 50)
  while(!gyro.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
  {
    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
    delay(500);
  }
 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyro.calibrate(100)  ;
  // Initialize Initialize HMC5883L
   compass.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
/*
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
*/

  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.

  dht.temperature();
  dht.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity();
  dht.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

}

 /* 
    dht.begin();

    Serial.println("Initializing barometer...");
    barometer.initialize();
        Serial.println("Testing device connections...");
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

    // configure LED pin for activity indication
    pinMode(LED_PIN, OUTPUT);
*/
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
/*
void serialEvent2a() {
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
*/

void loop()
{
 //   dataFile = SD.open("datalog.txt", FILE_WRITE  );
 //  serial2Event();
 RF.print(refnr++);
 RF.print(",|, ");
 if (gps_ferdig==true)
   {
   Serial.println(gpsbuffer);
   gps_ferdig=false;
   }
  else
  { 
  compas();
  dht_fuktighet() ;
  Gyro() ;
  Acc() ;
  lys();
  Barometer();
  }
//dataFile.close();

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

