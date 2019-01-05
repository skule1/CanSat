void hmc5883l()
{  float heading;
      mag.getHeading(&mx, &my, &mz);

    // display tab-separated gyro x/y/z values
    Serial.print("mag:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
    
// To calculate heading in degrees. 0 degree indicates North
     heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
    Serial.print("heading:\t");
    Serial.println(heading * 180/M_PI);


  //  Serial.print("mag:\t");
    dataFile.print(mx); dataFile.print("\t");
    dataFile.print(my); dataFile.print("\t");
    dataFile.print(mz); dataFile.print("\t");
    
// To calculate heading in degrees. 0 degree indicates North
     heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
 //   Serial.print("heading:\t");
    dataFile.print(heading * 180/M_PI); dataFile.print("\t");

}

