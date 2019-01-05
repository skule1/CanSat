void adxl345()
{
    accel.getAcceleration(&ax, &ay, &az);

    // display tab-separated accel x/y/z values
    Serial.print("accel:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.println(az);

    dataFile.print(ax); dataFile.print("\t");
    dataFile.print(ay); dataFile.print("\t");
    dataFile.print(az); dataFile.print("\t");
}

