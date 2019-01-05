void l3g4200d()
{
//  timer = millis();

  // Read normalized values
  Vector norm = gyroscope.readNormalize();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  /*Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);
*/
  dataFile.print(pitch);  dataFile.print("\t");
  dataFile.print(roll); dataFile.print("\t");
  dataFile.print(yaw); dataFile.print("\t");

  RF_port.print(pitch);  RF_port.print("\t");
  RF_port.print(roll); RF_port.print("\t");
  RF_port.print(yaw); RF_port.print("\t");

  // Wait to full timeStep period
  //delay((timeStep*1000) - (millis() - timer));
}

/*
void l3g4200d_setup()
{
   // Initialize L3G4200D(dps, odrbw)

  // dps:
  // L3G4200D_SCALE_250DPS:   200 dps
  // L3G4200D_SCALE_500DPS:   500 dps
  // L3G4200D_SCALE_2000DPS: 2000 dps (default)

  // odrbw:
  // L3G4200D_DATARATE_800HZ_50:   Output Data Rate 800HZ, Cut-off 50
  // L3G4200D_DATARATE_800HZ_35:   Output Data Rate 800HZ, Cut-off 35
  // L3G4200D_DATARATE_800HZ_30:   Output Data Rate 800HZ, Cut-off 30
  // L3G4200D_DATARATE_400HZ_110:  Output Data Rate 400HZ, Cut-off 110
  // L3G4200D_DATARATE_400HZ_50:   Output Data Rate 400HZ, Cut-off 50
  // L3G4200D_DATARATE_400HZ_25:   Output Data Rate 400HZ, Cut-off 25
  // L3G4200D_DATARATE_400HZ_20:   Output Data Rate 400HZ, Cut-off 20
  // L3G4200D_DATARATE_200HZ_70:   Output Data Rate 200HZ, Cut-off 70
  // L3G4200D_DATARATE_200HZ_50:   Output Data Rate 200HZ, Cut-off 50
  // L3G4200D_DATARATE_200HZ_25:   Output Data Rate 200HZ, Cut-off 25
  // L3G4200D_DATARATE_200HZ_12_5: Output Data Rate 200HZ, Cut-off 12.5
  // L3G4200D_DATARATE_100HZ_25:   Output Data Rate 100HZ, Cut-off 25
  // L3G4200D_DATARATE_100HZ_12_5: Output Data Rate 100HZ, Cut-off 12.5 (default)

  Serial.println("Initialize L3G4200D");

  while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
  {
    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
    delay(500);
  }

  // Check selected scale
  Serial.print("Selected scale: ");

  switch(gyroscope.getScale())
  {
    case L3G4200D_SCALE_250DPS:
      Serial.println("250 dps");
      break;
    case L3G4200D_SCALE_500DPS:
      Serial.println("500 dps");
      break;
    case L3G4200D_SCALE_2000DPS:
      Serial.println("2000 dps");
      break;
    default:
      Serial.println("unknown");
      break;
  }

  // Check Output Data Rate and Bandwidth
  Serial.print("Output Data Rate: ");

  switch(gyroscope.getOdrBw())
  {
    case L3G4200D_DATARATE_800HZ_110:
      Serial.println("800HZ, Cut-off 110");
      break;
    case L3G4200D_DATARATE_800HZ_50:
      Serial.println("800HZ, Cut-off 50");
      break;
    case L3G4200D_DATARATE_800HZ_35:
      Serial.println("800HZ, Cut-off 35");
      break;
    case L3G4200D_DATARATE_800HZ_30:
      Serial.println("800HZ, Cut-off 30");
      break;
    case L3G4200D_DATARATE_400HZ_110:
      Serial.println("400HZ, Cut-off 110");
      break;
    case L3G4200D_DATARATE_400HZ_50:
      Serial.println("400HZ, Cut-off 50");
      break;
    case L3G4200D_DATARATE_400HZ_25:
      Serial.println("400HZ, Cut-off 25");
      break;
    case L3G4200D_DATARATE_400HZ_20:
      Serial.println("400HZ, Cut-off 20");
      break;
    case L3G4200D_DATARATE_200HZ_70:
      Serial.println("200HZ, Cut-off 70");
      break;
    case L3G4200D_DATARATE_200HZ_50:
      Serial.println("200HZ, Cut-off 50");
      break;
    case L3G4200D_DATARATE_200HZ_25:
      Serial.println("200HZ, Cut-off 25");
      break;
    case L3G4200D_DATARATE_200HZ_12_5:
      Serial.println("200HZ, Cut-off 12.5");
      break;
    case L3G4200D_DATARATE_100HZ_25:
      Serial.println("100HZ, Cut-off 25");
      break;
    case L3G4200D_DATARATE_100HZ_12_5:
      Serial.println("100HZ, Cut-off 12.5");
      break;
    default:
      Serial.println("unknown");
      break;
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyroscope.calibrate();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  gyroscope.setThreshold(3);

}

void l3g4200d_raw()
{
  // Read normalized values
  Vector raw = gyroscope.readRaw();

  // Read normalized values in deg/sec
  Vector norm = gyroscope.readNormalize();
/*
  // Output raw
  Serial.print(" Xraw = ");
  Serial.print(raw.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(raw.XAxis);
  Serial.print(" Zraw = ");
  Serial.print(raw.YAxis);

  // Output normalized
  Serial.print(" Xnorm = ");
  Serial.print(norm.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(norm.YAxis);
  Serial.print(" ZNorm = ");
  Serial.print(norm.ZAxis);

  Serial.println();


    // Output raw

  dataFile.print(raw.XAxis);  dataFile.print("\t");
  dataFile.print(raw.XAxis);dataFile.print("\t");
  dataFile.print(raw.YAxis);dataFile.print("\t");

  // Output normalized

  dataFile.print(norm.XAxis);dataFile.print("\t");
  dataFile.print(norm.YAxis);dataFile.print("\t");
  dataFile.print(norm.ZAxis);dataFile.print("\t");

  RF_port.print(raw.XAxis);  RF_port.print("\t");
  RF_port.print(raw.XAxis);RF_port.print("\t");
  RF_port.print(raw.YAxis);RF_port.print("\t");

  // Output normalized

  RF_port.print(norm.XAxis);RF_port.print("\t");
  RF_port.print(norm.YAxis);RF_port.print("\t");
  RF_port.print(norm.ZAxis);RF_port.print("\t");


}
*/
