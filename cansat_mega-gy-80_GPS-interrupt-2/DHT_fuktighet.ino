/*void DHT() {
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
}
*/


void dht_fuktighet() {
  // Wait a few seconds between measurements.
 // delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
 // float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  //Serial.print("Humidity: ");
  RF.print(h);  dataFile.print(h);
   RF.print(", ");dataFile.print(", ");
// Serial.print(" %\t");
//  Serial.print("Temperature: ");
  RF.print(t);  dataFile.print(t);
   RF.print(", ");dataFile.print(", ");
// Serial.print(" *C ");
//  RF.print(f);  dataFile.print(f);
//  RF.print(", ");dataFile.print(", ");
//  Serial.print(" *F\t");
//  Serial.print("Heat index: ");
  RF.print(hic);  dataFile.print(hic);
//  RF.print(", ");dataFile.print(", ");
//  Serial.print(" *C ");
//  RF.print(hif);  dataFile.print(hif);
 // Serial.println(" *F");
   RF.print(",|, ");  dataFile.print(",|, ");
}


