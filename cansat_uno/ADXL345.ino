void ADSL345_raw() {
    // read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);

    // display tab-separated accel x/y/z values
/*    Serial.print("accel:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.println(az);
*/
    dataFile.print(ax); dataFile.print("\t");
    dataFile.print(ay); dataFile.print("\t");
    dataFile.print(az); dataFile.print("\t");
    RF_port.print(ax); RF_port.print("\t");
    RF_port.print(ay); RF_port.print("\t");
      dtostrf(az, 6, 2, charBuf);
    RF_port.print(az); RF_port.print("\t");


}
