void l3g4200d_pitch_and_roll()
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

//  dataFile.print(" Pitch = ");
  dataFile.print(pitch); dataFile.print("\t");
//  dataFile.print(" Roll = ");
  dataFile.print(roll);   dataFile.print("\t");
//  dataFile.print(" Yaw = ");
  dataFile.print(yaw); dataFile.print("\t");

  // Wait to full timeStep period
//  delay((timeStep*1000) - (millis() - timer));
}
