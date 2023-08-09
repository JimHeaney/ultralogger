#include <Wire.h>
#include <MS5607.h>
#include <EEPROM.h>
#include <SparkFun_External_EEPROM.h>

MS5607 barometer;

ExternalEEPROM EEPROM1;
ExternalEEPROM EEPROM2;
ExternalEEPROM EEPROM3;

#define greenLED PIN_PA7
#define redLED PIN_PC2
#define batState PIN_PC1
#define battery PIN_PA5
#define USB PIN_PC0
#define intAcc PIN_PB3
#define gOffset 0.049

#define accAddr 0x1D

double temperature = 0;
double altitude = 0;
double acceleration = 0;
long time = 0;

unsigned long measStart = 0;

int count = 0;

double groundAlt = 0;
double groundAltBuffer = 0;

bool dataLogged = 0; //set to 1 if there is flight data on the controller
bool units = 0; //0 for meters/celcius, 1 for feet/farenheit
//byte fastLaunch = 20; //how many seconds to be in fast record mode
byte liftoffAcc = 0; //Acceleration change to be considered liftoff
byte liftoffAlt = 50; //Altitude change to be considered liftoff
byte offsetAlt = 0; //Set to 1 to offset altitude to AGL, 0 to ASL.
byte dataRate = 50; //Delay between measurements, in ms.

/*
long flightCount = 0;
double totalFlightTime = 0;
double maxAltitude = 0;
double maxAcceleration = 0;
double minAcceleration = 0;
double maxTemperature = 0;
double minTemperature = 0;
*/

byte unitsModded = 0;
byte fastLaunchModded = 0;
byte liftoffAccModded = 0;
byte liftoffAltModded = 0;
byte offsetAltModded = 0;
byte dataRateModded = 0;

int accMem = 0;
int tempMem = 0;

int dump = 0;
String tempString;

bool liftoff = 0;

unsigned long address = 0; 

unsigned long flightStart = 0;

bool USBIgnore = 0;

unsigned long blinkTime = 0;
bool blinkState;

unsigned long newGroundRead = 0;

bool groundLock = 0;

bool dumping = 0;

unsigned long landCheck = 0;

double oldAltitude = 0;

unsigned long addrByte = 0;

unsigned long liftoffTime = 0;

void setup() {
  // put your setup code here, to run once:

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


  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);

    Serial.swap();
    Serial.begin(115200);
    Serial.setTimeout(100);

}

int accRaw() {
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

  acceleration = accRaw() * gOffset;

  barometer.readDigitalValue();
  altitude = barometer.getAltitude();
  temperature = barometer.getTemperature();

  Serial.print(acceleration);
  Serial.println(F(" g"));
  Serial.print(altitude, 4);
  Serial.println(F(" m ASL"));
  Serial.print(temperature, 4);
  Serial.println(F(" C"));
  Serial.println(); 

  double battVoltage = analogRead(battery) / 155.0;
  Serial.println(battVoltage);
  Serial.println();

  delay(50);
}
