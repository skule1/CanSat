void BMP085() {
    // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    // wait appropriate time for conversion (4.5ms delay)
    lastMicros = micros();
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();

    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();

    // calculate absolute altitude in meters based on known pressure
    // (may pass a second "sea level pressure" parameter here,
    // otherwise uses the standard value of 101325 Pa)
    altitude = barometer.getAltitude(pressure);

    // display measured values if appropriate
/*    Serial.print("T/P/A\t");
    Serial.print(temperature); Serial.print("\t");
    Serial.print(pressure); Serial.print("\t");
    Serial.print(altitude);
    Serial.println("");
  */  // display measured values if appropriate
  //  Serial.print("T/P/A\t");
    dataFile.print(temperature,2); dataFile.print("\t");
    dataFile.print(pressure); dataFile.print("\t");
    dataFile.print(altitude,3); dataFile.print("\t");
  
    RF_port.print(temperature,2); RF_port.print("\t");
    RF_port.print(pressure,0); RF_port.print("\t");
    RF_port.print(altitude,3); RF_port.print("\t");

    }
