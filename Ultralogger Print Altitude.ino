#include <Wire.h>
#include <MS5607.h>

MS5607 barometer;

void setup() {
  // put your setup code here, to run once:

  Serial.swap();
  Serial.begin(115200);
  Serial.setTimeout(100);

  barometer.begin();
  barometer.setOSR(4096);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(millis());
  Serial.print(",");
  barometer.readDigitalValue();
  Serial.print(barometer.getAltitude(), 4);
  Serial.print(",");
  Serial.println(barometer.getTemperature(), 4);
  delay(500);
}
