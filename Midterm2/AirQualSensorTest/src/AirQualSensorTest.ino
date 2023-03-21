 #include "Air_Quality_Sensor.h"

AirQualitySensor sensor(A0);

void setup(void) {
  Serial.begin(9600);
  while (!Serial);

  Serial.printf("Waiting sensor to init...");
  delay(20000);
  
  if (sensor.init()) {
    Serial.printf("Sensor ready.");
  }
  else {
    Serial.printf("Sensor ERROR!");
  }
}

void loop(void) {
  int quality = sensor.slope();
  
  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    Serial.printf("High pollution! Force signal active.");
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    Serial.printf("We have HIGH POLLUTION because our sensor value is: %i OH NO!!! \n", sensor.getValue());
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) {
    Serial.printf("We have LOW POLLUTION because our sensor value is: %i YEAH for LOW POLLUTION!!! \n", sensor.getValue());
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    Serial.printf("We have FRESH AIR because our sensor value is: %i YEAH for FRESH AIR!!! \n", sensor.getValue());
  }
  
  delay(1000);
  
}