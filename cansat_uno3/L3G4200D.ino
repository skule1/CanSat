void l3g4200d_pitch_and_roll()
{
 timer = millis();

  // Read normalized values
  Vector norm = gyroscope.readNormalize();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  #ifdef SD_Brikke
  dataFile.print(pitch); dataFile.print("\t");
  dataFile.print(roll);   dataFile.print("\t");
  dataFile.print(yaw); dataFile.print("\t");
#endif
  
  RF_port.print(pitch); RF_port.print("\t");
  RF_port.print(roll);   RF_port.print("\t");
  RF_port.print(yaw); RF_port.print("\t");


}
