

void Barometer() {
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
    altitude1 = barometer.getAltitude(pressure);

    // display measured values if appropriate
 /*   Serial.print("T/P/A\t");
    Serial.print(temperature); Serial.print("\t");
    Serial.print(pressure); Serial.print("\t");
    Serial.print(altitude);
    Serial.println("");
  */  
    RF.print(temperature); RF.print(", ");
    RF.print(pressure); RF.print(", ");
    RF.print(altitude1);RF.println("");
 //   Serial.println("");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
    // delay 100 msec to allow visually parsing blink and any serial output
    delay(100);
}
