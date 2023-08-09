#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <EEPROM.h>

/* Assign a unique ID to this sensor at the same time */
/* Uncomment following line for default Wire bus      */
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

unsigned long serialNumber = 0;
int versionNumber = 0;
int flightCount = 0;
long totalFlightTime = 0;
long maxAltitude = 0;
long maxAcceleration = 0;
int minAcceleration = 0;
double maxTemperature = 0;
double minTemperature = 0;
byte units = 1;
byte fastLaunch = 0;
byte liftoffAcc = 0;
byte liftoffAlt = 30;
byte offsetAlt = 1;
byte dataRate = 50;


void wait() {
  //Wait until a new signal comes through on the serial monitor.
  while (Serial.available() < 1)
    ;
  while (Serial.available() > 0) {
    byte dump = Serial.read();
    delay(10);
  }
}

void setup(void)
{
  Serial.swap();
  Serial.begin(115200);

  //EEPROM Default Values
    EEPROM.put(0, serialNumber);
    EEPROM.put(4, versionNumber);
    EEPROM.put(8, flightCount);
    EEPROM.put(12, totalFlightTime);
    EEPROM.put(16, maxAltitude);
    EEPROM.put(20, maxAcceleration);
    EEPROM.put(24, minAcceleration);
    EEPROM.put(28, maxTemperature);
    EEPROM.put(32, minTemperature);
    EEPROM.put(36, units);
    EEPROM.put(37, fastLaunch);
    EEPROM.put(38, liftoffAcc);
    EEPROM.put(39, liftoffAlt);
    EEPROM.put(40, offsetAlt);
    EEPROM.put(41, dataRate);

  /* Initialise the sensor */
  accel.begin(0x1D);

  /* Display some basic information on this sensor */
  accel.printSensorDetails();
  Serial.println("");

  // init offsets to zero
  accel.setTrimOffsets(0, 0, 0);
  
  Serial.println("Mount sensor in fixture. Press anything to continue.");
  wait();
  int16_t x, y, z;
  x = accel.getX();
  y = accel.getY();
  z = accel.getZ();
  Serial.print("Raw X: "); Serial.print(x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(z); Serial.print("  ");Serial.println(" counts");

  // the trim offsets are in 'multiples' of 4, we want to round, so we add 2
  accel.setTrimOffsets(-(x + 20 + 2) / 4,  // X should be '20' at 1g (49mg per bit)
                       -(y + 2) / 4,
                       -(z + 2) / 4);  

  int8_t x_offset, y_offset, z_offset;
  accel.getTrimOffsets(&x_offset, &y_offset, &z_offset);
  Serial.print("Current trim offsets: ");
  Serial.print(x_offset);  Serial.print(", ");
  Serial.print(y_offset);  Serial.print(", ");
  Serial.println(z_offset);

  Serial.println();
  Serial.println("Accelerometer Calibration Complete!");
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x/9.81); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y/9.81); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z/9.81); Serial.print("  ");
  Serial.println("g ");
  
  Serial.print("Raw X: "); Serial.print(accel.getX()); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accel.getY()); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accel.getZ()); Serial.print("  ");
  Serial.println(" counts");
  Serial.println();
  delay(500);
}