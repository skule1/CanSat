void l3g4200d_pitch_and_roll()
{
  timer = millis();

  // Read normalized values
  Vector norm = gyroscope.readNormalize();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  dataFile.print(pitch); dataFile.print(sh);
  dataFile.print(roll);   dataFile.print(sh);
  dataFile.print(yaw); dataFile.print(sh);

  
  RF_port.print(pitch); RF_port.print(sh);
  RF_port.print(roll);   RF_port.print(sh);
  RF_port.print(yaw); RF_port.print(sh);


}

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


void l3g4200D_simple()

{
  // Read normalized values
  Vector raw = gyroscope.readRaw();

  // Read normalized values in deg/sec
  Vector norm = gyroscope.readNormalize();

  // Output raw
//  Serial.print(" Xraw = ");
  RF_port.print(raw.XAxis);RF_port.print(sh);
//  Serial.print(" Yraw = ");
  RF_port.print(raw.XAxis);RF_port.print(sh);
//  Serial.print(" Zraw = ");
  RF_port.print(raw.YAxis);RF_port.print(sh);

  // Output normalized
//  Serial.print(" Xnorm = ");
  RF_port.print(norm.XAxis);RF_port.print(sh);
 // Serial.print(" Ynorm = ");
  RF_port.print(norm.YAxis);RF_port.print(sh);
//  Serial.print(" ZNorm = ");
  RF_port.print(norm.ZAxis);RF_port.print(sh);


}


