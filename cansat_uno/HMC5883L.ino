void HMC5883L() {
    // read raw heading measurements from device
   
    mag.getHeading(&mx, &my, &mz);

    // display tab-separated gyro x/y/z values
    dataFile.print(mx); dataFile.print("\t");
    dataFile.print(my); dataFile.print("\t");
    dataFile.print(mz); dataFile.print("\t");

//    Serial.print("mag:\t");
    RF_port.print(mx); RF_port.print("\t");
    RF_port.print(my); RF_port.print("\t");
    RF_port.print(mz); RF_port.print("\t");
    
// To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
 
//    dataFile.print("heading:\t");
    dataFile.print(heading * 180/M_PI);

//    RF_port.print("heading:\t");
    RF_port.print(heading * 180/M_PI);

}
