#include <Wire.h>
#include <MS5607.h>
#include <SparkFun_External_EEPROM.h>

#define accAddr 0x1D

MS5607 barometer;

ExternalEEPROM EEPROM1;
ExternalEEPROM EEPROM2;
ExternalEEPROM EEPROM3;

long timeMem = 0;
double altMem = 0;
int accMem = 0;
int tempMem = 0;

unsigned long address = 0; 

unsigned long addrByte = 0;

unsigned long time = 0;

unsigned long totalTime = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.swap();
  Serial.begin(115200);
  Serial.setTimeout(100);

  barometer.begin();
  barometer.setOSR(4096);

  Wire.begin();

  //Initialize EEPROM
  EEPROM1.begin(0b1010000);
  EEPROM2.begin(0b1010001);
  EEPROM3.begin(0b1010011);
  
  EEPROM1.setPageSize(128);
  EEPROM2.setPageSize(128);
  EEPROM3.setPageSize(128);

  EEPROM1.setMemorySize(65536);
  EEPROM2.setMemorySize(65536);
  EEPROM3.setMemorySize(65536);

  //Start the accelerometer:
  //Write 00001000 to 0x2D to set it to measurement mode
  Wire.beginTransmission(accAddr);
  Wire.write(0x2D);
  Wire.write(0b00001000);
  Wire.endTransmission();
  //Write 0b00001011 to 0x31, per comments in Adafruit library
  Wire.beginTransmission(accAddr);
  Wire.write(0x31);
  Wire.write(0b00001011);
  Wire.endTransmission();
  //Write 00001010 to 0x2C to set the data rate to 100Hz
  Wire.beginTransmission(accAddr);
  Wire.write(0x2C);
  Wire.write(0b00001010);
  Wire.endTransmission();
  //Write 0 to 0x38 to turn off the FIFO
  Wire.beginTransmission(accAddr);
  Wire.write(0x38);
  Wire.write(0);
  Wire.endTransmission();

}

void wait(){
  //Wait until a new signal comes through on the serial monitor.
  while(Serial.available() < 1);
  while(Serial.available() > 0){
    byte dump = Serial.read();
    delay(10);
  }
}

int accRaw(){
  //Read the acceleration data in the X axis from the accelerometer, report it in the unmodified format.

  byte x0 = 0;
  byte x1 = 0;
  
  //To read the data in X, request 0x32 (LSB) and 0x33 (MSB)
  Wire.beginTransmission(accAddr);
  Wire.write(0x32);
  Wire.endTransmission();

  Wire.requestFrom(accAddr, 2);
  x0 = Wire.read();
  x1 = Wire.read();

  int accGet = (int16_t)(x1 << 8) + x0;
  
  return accGet;

}

void loop() {
  // put your main code here, to run repeatedly:
  wait();
  totalTime = millis();
  time = millis();
  barometer.readDigitalValue();
  time = millis() - time;
  Serial.print("Barometer Get: ");
  Serial.println(time);
  //wait();
  time = millis();
  altMem = barometer.getAltitude();
  time = millis() - time;
  Serial.print("Barometer Alt: ");
  Serial.println(time);
  //wait();
  time = millis();
  int tempMem = round(barometer.getTemperature());
  time = millis() - time;
  Serial.print("Barometer Temp to Int: ");
  Serial.println(time);
  //wait();
  time = millis();
  altMem = accRaw();
  time = millis() - time;
  Serial.print("Accelerometer Read: ");
  Serial.println(time);
  //wait();
  time = millis();
  unsigned long timeMem = millis();
  time = millis() - time;
  Serial.print("Time Get: ");
  Serial.println(time);
  //wait();
  time = millis();
    if(address <= 19662){
      //Log it all to memory:
      //Memory Order: Time, Acceleration, Altitude, Temperature
      unsigned long addrByte = 0;
      //If address is 0 and 5460, use EEPROM1.
      if(address <= 5460){
        addrByte = address * 12;
        EEPROM1.put(addrByte, timeMem);
        EEPROM1.put(addrByte+4, accMem);
        EEPROM1.put(addrByte+6, altMem);
        EEPROM1.put(addrByte+10, tempMem);
      }
      //If address is 5461 to 10921, use EEPROM2.
      if((address > 5461) && (address <= 10921)){
        addrByte = (address - 5462) * 12;
        EEPROM2.put(addrByte, timeMem);
        EEPROM2.put(addrByte+4, accMem);
        EEPROM2.put(addrByte+6, altMem);
        EEPROM2.put(addrByte+10, tempMem);
      }
      //If address is 10922 to 16382, use EEPROM3. 
      if(address >= 10922){
        addrByte = (address - 10922) * 12;
        EEPROM3.put(addrByte, timeMem);
        EEPROM3.put(addrByte+4, accMem);
        EEPROM3.put(addrByte+6, altMem);
        EEPROM3.put(addrByte+10, tempMem);
      }
      address = address + 1;
    }
    time = millis() - time;
    totalTime = millis()-totalTime;
    Serial.print("Record Time:");
    Serial.println(time);
    Serial.println();
    Serial.print("Total Time: ");
    Serial.println(totalTime);      

}
