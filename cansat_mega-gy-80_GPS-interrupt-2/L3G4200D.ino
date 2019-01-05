void Gyro()
{
  timer = millis();

  // Read normalized values
  Vector norm = gyroscope.readNormalize();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}


  timer = millis();

  // Read normalized values
  Vector norm = gyroscope.readNormalize();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);

 
 
 
 
 
 /*
 {
    // read raw angular velocity measurements from device
    gyro.getAngularVelocity(&avx, &avy, &avz);
    // int16_t x_single = gyro.getAngularVelocityX();
    // int16_t y_single = gyro.getAngularVelocityY();

    // //read X memory address directly
    // uint8_t xval_l, xval_h;
    // uint8_t devAddress = 0x6B;
    // uint8_t regAddXL = 0x28;
    // uint8_t regAddXH = 0x29;
    // I2Cdev::readByte(devAddress, regAddXL, &xval_l);
    // I2Cdev::readByte(devAddress, regAddXH, &xval_h);
    // //read X memory addresses in single sequential
    // uint8_t data[2];
    // I2Cdev::readBytes(devAddress, regAddXL| 0x80, 2, data);
    

    // Serial.print("Direct Mem Read: Xl: "); 
    // Serial.print(xval_l); Serial.print("\tXh: ");
    // Serial.print(xval_h); Serial.print("\t");
    // Serial.print("Direct readBytes data[0] data[1]: ");
    // Serial.print(data[0]); Serial.print("\t");
    // Serial.print(data[1]); Serial.print("\t");
    // Serial.print("Bitshifted: "); Serial.print((((int16_t)data[1]) << 8) | data[0]);
    Serial.print("angular velocity (dps):\t");
    Serial.print(avx*0.00875F,DEC); Serial.print("\t");
    Serial.print(avy*0.00875F,DEC); Serial.print("\t");
    Serial.print(avz*0.00875F,DEC); Serial.print("\t");
    // Serial.print(" x read only: "); Serial.print(x_single);
    // Serial.print(" y read only: "); Serial.println(y_single);
    Serial.println();
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

*/
