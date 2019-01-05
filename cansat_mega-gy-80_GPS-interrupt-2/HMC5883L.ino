
void compas()
{ 
 {
    // read raw heading measurements from device
    mag.getHeading(&mx, &my, &mz);

    // display tab-separated gyro x/y/z values
    Serial.print("mag:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
    
// To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
    Serial.print("heading:\t");
    Serial.println(heading * 180/M_PI);

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

