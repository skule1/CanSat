void BMP085a()
{
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    lastMicros = micros();
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());
    temperature = barometer.getTemperatureC();
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());
    pressure = barometer.getPressure();
    altitude = barometer.getAltitude(pressure);

 //    Serial.print("T/P/A\t");
   dataFile.print(temperature); dataFile.print(sh);
   dataFile.print(pressure); dataFile.print(sh);
   dataFile.print(altitude);dataFile.print(sh);

   RF_port.print(temperature); RF_port.print(sh);
   RF_port.print(pressure); RF_port.print(sh);
   RF_port.print(altitude);RF_port.print(sh);   


 
}

