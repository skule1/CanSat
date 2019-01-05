void adxl345()
{
    accel.getAcceleration(&ax, &ay, &az);

    dataFile.print(ax); dataFile.print("\t");
    dataFile.print(ay); dataFile.print("\t");
    dataFile.print(az); dataFile.print("\t");
    
    RF_port.print(ax); RF_port.print("\t");
    RF_port.print(ay); RF_port.print("\t");
    RF_port.print(az); RF_port.print("\t");
}

