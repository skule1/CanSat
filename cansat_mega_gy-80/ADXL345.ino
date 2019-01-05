void adxl345()
{
    accel.getAcceleration(&ax, &ay, &az);

    dataFile.print(ax); dataFile.print(sh);
    dataFile.print(ay); dataFile.print(sh);
    dataFile.print(az); dataFile.print(sh);
    
    RF_port.print(ax); RF_port.print(sh);
    RF_port.print(ay); RF_port.print(sh);
    RF_port.print(az); RF_port.print(sh);
}

