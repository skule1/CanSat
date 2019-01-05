

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
    double pitch, roll, Xg, Yg, Zg;
    double fXg = 0;
double fYg = 0;
double fZg = 0;
const float alpha = 0.5;
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
/*  Serial.print("X: "); Serial.print(event.acceleration.x-xoff); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y-yoff); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z-zoff); Serial.print("  ");Serial.println("m/s^2 ");
*/
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



void Acc() 
{
    double pitch, roll, Xg, Yg, Zg;
    double fXg = 0;
double fYg = 0;
double fZg = 0;
const float alpha = 0.5;
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
/*  Serial.print("X: "); Serial.print(event.acceleration.x-xoff); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y-yoff); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z-zoff); Serial.print("  ");Serial.println("m/s^2 ");
*/
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
