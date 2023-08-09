//Serial Number:
//const int serialNumber = 0001;
//const int versionNumber = 001;

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

#define Conf F("Confirmed")

double temperature = 0;
double altitude = 0;
double acceleration = 0;
double time = 0;

unsigned long measStart = 0;

int count = 0;

int groundAlt = 0;
int groundAltBuffer = 0;

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

unsigned long timeMem = 0;
unsigned int altMem = 0;
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

  /*EEPROM Layout:
    0: Serial Number
    4. Version Number
    8: Flight Count
    12: Total Flight Time
    16: Max Altitude
    20: Max Acceleration
    24: Min Acceleration
    28: Max Temperature
    32: Min Temperature
    36: Units
    37: fastLaunch
    38: liftoffAcc
    39: liftoffAlt
    40: altOffset
    41: dataRate
    42: dataLogged
*/

  void
  setup() {
  // put your setup code here, to run once:


   /*Load Default Values: 
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
  */

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

  if(digitalRead(USB) == 1){

    Serial.swap();
    Serial.begin(115200);
    Serial.setTimeout(100);

    //Load settings into modded variables
    EEPROM.get(36, unitsModded);
    //EEPROM.get(37, fastLaunchModded);
    EEPROM.get(38, liftoffAccModded);
    EEPROM.get(39, liftoffAltModded);
    EEPROM.get(40, offsetAltModded);
    EEPROM.get(41, dataRateModded);

    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
    chargeLED();
    //Attach interrupt for charging status LEDs:
    //attachInterrupt(batState, chargeLED, CHANGE);
  }

    while(digitalRead(USB) == 1){
      //Infinitely loop until USB power is removed

      chargeLED();

      if(Serial.available() > 0){

        //c: report current measurements
        if(Serial.peek() == 99){
          serialClear();
          EEPROM.get(36, units);

          //Accelerometer:
          acceleration = accRaw() * gOffset;

          //Barometer:
          barometer.readDigitalValue();
          altitude = barometer.getAltitude();
          temperature = barometer.getTemperature();

          Serial.print(acceleration);
          Serial.println(F(" g"));
          if(units == 1){
            Serial.print(altitude);
            Serial.println(F(" m ASL"));
          }
          if(units == 0){
            double altitudeft = altitude * 3.28084;
            Serial.print(altitudeft);
            Serial.println(F(" ft ASL"));
          }
          if(units == 1){
            Serial.print(temperature);
            Serial.println(F(" C"));
          }
          if(units == 0){
            double temperaturef = temperature * 1.80;
            temperaturef = temperaturef + 32;
            Serial.print(temperaturef);
            Serial.println(F(" F"));
          }
          Serial.println();
          
          
        }

        //d: download data from memory
        else if(Serial.peek() == 100){
          serialClear();

          EEPROM.get(36, units);
          EEPROM.get(40, offsetAlt);

          dumping = 1;
          address = 0;

          //Clear the serial monitor:
          Serial.write(27);       // ESC command
          Serial.print("[2J");    // clear screen command
          Serial.write(27);
          Serial.print("[H");

          //Write a header
            Serial.print(F("Time (s),"));
            Serial.print(F("Acc (g),"));
            Serial.print(F("Alt")); 
            if(offsetAlt == 0){
              Serial.print(F(" ASL "));
            }
            if(offsetAlt == 1){
              Serial.print(F(" AGL "));
            }
            if(units == 1){
              Serial.println(F("(m),Temp (C)"));
            }
            if(units == 0){
              Serial.println(F("(ft),Temp (F)"));
            }


          //Dump all data from memory
          while(dumping == 1){
            
            //First, read in the values from EEPROM based on address.
            if(address <= 19662){
              //Memory Order: Time, Acceleration, Altitude, Temperature
              //If address is 0 and 6553, use EEPROM1.
              if(address <= 6553){
                addrByte = address * 10;
                EEPROM1.get(addrByte, timeMem);
                EEPROM1.get(addrByte+4, accMem);
                EEPROM1.get(addrByte+6, altMem);
                EEPROM1.get(addrByte+8, tempMem);
              }
              //If address is 6554 to 13108, use EEPROM2.
              if((address > 6553) && (address <= 13108)){
                addrByte = (address - 6554) * 10;
                EEPROM2.get(addrByte, timeMem);
                EEPROM2.get(addrByte+4, accMem);
                EEPROM2.get(addrByte+6, altMem);
                EEPROM2.get(addrByte+8, tempMem);
              }

              //If address is 13109 to 19662, use EEPROM3. 
              if(address >= 13109){
                addrByte = (address - 13109) * 10;
                EEPROM3.get(addrByte, timeMem);
                EEPROM3.get(addrByte+4, accMem);
                EEPROM3.get(addrByte+6, altMem);
                EEPROM3.get(addrByte+8, tempMem);
              }
              Serial.flush();
              address = address + 1;
            } 
            
            //Check if we've reached the end of the memory
            if(address > 19662){
              dumping = 0;
            }

            //Check if we are in empty memory
            if(timeMem == 0){
              dumping = 0;
              goto bypass;
            }

            //Convert the values to the proper units
            double timeOut = timeMem / 1000.0;
            double accOut = accMem * gOffset;
            double altOut = altMem / 2.0;
            double tempOut = tempMem / 100.0;
            if(units == 0){
              altOut = altOut * 3.28084;
              tempOut = tempOut * 1.8;
              tempOut = tempOut + 32;
            }

            //Print the values to the serial monitor
            Serial.print(timeOut);
            Serial.print(F(","));
            Serial.print(accOut);
            Serial.print(F(","));
            Serial.print(altOut);
            Serial.print(F(","));
            Serial.println(tempOut);
          }

          bypass:

          //Once all memory is written, ask if the data should be cleared:
          Serial.println();
          //Serial.println(F("Download Complete."));
          Serial.println(F("Clear? y/n"));
          while (Serial.available() == 0);
          if((Serial.peek() == 121) || (Serial.peek() == 89)){
            Serial.println(F("..."));
            EEPROM1.erase();
            EEPROM2.erase();
            EEPROM3.erase();
            Serial.println(F("Cleared"));
            dataLogged = 0;
            EEPROM.put(42, dataLogged);
          } else if((Serial.peek() == 78) || (Serial.peek() == 110)){
              Serial.println(F("Retained"));
          }
          Serial.println();
        }

        //e: exit settings, force into the loop (for use in debug/testing)
        /*else if(Serial.peek() == 101){
          USBIgnore = 1;
          goto looper; 
        } */

        //f: change units to Feet / Farenheit
        else if(Serial.peek() == 102){
          unitsModded = 0;
          Serial.println(Conf);
          Serial.println();
        }

        //g or G: change offset mode to AGL
        else if(Serial.peek() == 103){
          offsetAltModded = 1;
          Serial.println(Conf);
          Serial.println();
        }

        //h xxx: Set the liftoff threshold to altitude above xxx ft/m (set to 0 to disable)
        else if(Serial.peek() == 104){
          dump = Serial.read();
          dump = Serial.read();
          tempString = Serial.readString();
          EEPROM.get(36, units);
          liftoffAltModded = tempString.toInt();
          if(units == 0){
            liftoffAltModded = round(liftoffAltModded / 3.281);
          }

          Serial.println(Conf);
          Serial.println();
        }

        //i or I: Print unique info about the altimeter
        /*else if((Serial.peek() == 105) || (Serial.peek() == 73)){

          //Read the data from EEPROM:
          EEPROM.get(0, serialNumber);
          EEPROM.get(4, versionNumber);
          EEPROM.get(8, flightCount);
          EEPROM.get(12, totalFlightTime);
          EEPROM.get(16, maxAltitude);
          EEPROM.get(20, maxAcceleration);
          EEPROM.get(24, minAcceleration);
          EEPROM.get(28, maxTemperature);
          EEPROM.get(32, minTemperature);

          Serial.print(F("S/N:"));
          Serial.println(serialNumber);
          Serial.print(F("Software Version: "));
          Serial.println(versionNumber);
          Serial.print(F("Flight Count: "));
          Serial.println(flightCount);
          Serial.print(F("Flight Time (min): "));
          Serial.println(totalFlightTime);
          Serial.print(F("Max Alt (m): "));
          Serial.println(maxAltitude);
          Serial.print(F("Max Acc (g): "));
          Serial.println(maxAcceleration);
          Serial.print(F("Min Acc (g): "));
          Serial.println(minAcceleration);
          Serial.print(F("Max Temp (C): "));
          Serial.println(maxTemperature);
          Serial.print(F("Min Temp (C): "));
          Serial.println(minTemperature);
          Serial.println();
        }*/

        //l yyy: Set the liftoff threshold to acceleration above YYY (set to 0 to disable)
        else if(Serial.peek() == 108){
          dump = Serial.read();
          dump = Serial.read();
          tempString = Serial.readString();
          liftoffAccModded = tempString.toInt();
          Serial.println(Conf);
          Serial.println();
        }

        //m: change units to Meters / Celsius
        else if(Serial.peek() == 109){
          unitsModded = 1;
          Serial.println(Conf);
          Serial.println();
        }

        //o or O: Change altitude offset mode to ASL 
        else if(Serial.peek() == 111){
          offsetAltModded = 0;
          Serial.println(Conf);
          Serial.println();          
        }

        //r xx or R xx: set refresh data rate in normal mode to xx Hz (between 1 and 50)
        /*else if((Serial.peek() == 114) || (Serial.peek() == 82)){
        }*/

        //s: save settings
        else if(Serial.peek() == 115){
          EEPROM.update(36, unitsModded);
          EEPROM.update(37, fastLaunchModded);
          EEPROM.update(38, liftoffAccModded);
          EEPROM.update(39, liftoffAltModded);
          EEPROM.update(40, offsetAltModded);
          EEPROM.update(41, dataRateModded);
          Serial.println(F("Saved!"));
          Serial.println();
        }

        //t xxx or T xxx: set the "fast record time" to xxx seconds
        /*else if((Serial.peek() == 116) || (Serial.peek() == 84)){
          dump = Serial.read();
          dump = Serial.read();
          tempString = Serial.readString();
          fastLaunchModded = tempString.toInt();
          Serial.println(Conf);
          Serial.println();
        }*/

        //v: report the battery voltage
        else if(Serial.peek() == 118){
          double battVoltage = analogRead(battery) / 155.0;
          Serial.print(F("Batt: "));
          Serial.println(battVoltage);
          Serial.println();
        }

        else error();

        serialClear();
      }
    }

  //USB Disconnected or Not Connected. Continue with Initialization:
  looper:
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  //detachInterrupt(batState);

  //Make sure the most updated settings are loaded;
    EEPROM.get(36, units);
    //EEPROM.get(37, fastLaunch);
    EEPROM.get(38, liftoffAcc);
    EEPROM.get(39, liftoffAlt);
    EEPROM.get(40, offsetAlt);
    EEPROM.get(41, dataRate);
    EEPROM.get(42, dataLogged);

  //Take an intial reading of ground level;
    barometer.readDigitalValue();
    groundAlt = round(barometer.getAltitude());
    groundAltBuffer = groundAlt;
    newGroundRead = millis();

    blinkTime = millis();
}

void error() {
  //Print an error message
  Serial.println(F("Err: Unkn!"));
  Serial.println();
}

void chargeLED(){
  //Passes the state of the charging status pin to the LEDs. 
  if(digitalRead(batState) == 0){
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
  }
  if(digitalRead(batState) == 1){
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
  }
}

void serialClear(){
  while(Serial.available() != 0){
    dump = Serial.read();
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

void measure(){
  //Take a measurement, update global variables

  //Time
  timeMem = millis();
  //Acceleration
  accMem = accRaw();
  //Altitude
  barometer.readDigitalValue();
  altMem = round(barometer.getAltitude() * 2);
  //Temperature
  tempMem = round(barometer.getTemperature() * 100);
}

void loop() {
  // put your main code here, to run repeatedly:

  //Wait until liftoff is detected
  while(liftoff == 0){

    //Check if we have exceeded required launch settings
    measure();
    int accCheck = abs(round(accMem * gOffset));
    int altCheck = round(altMem / 2);

    if((accCheck >= liftoffAcc) && (altCheck >= (liftoffAlt + groundAlt))){
      liftoff = 1;
      dataLogged = 1;
      EEPROM.put(42, dataLogged);
      digitalWrite(redLED, LOW);
      //flightStart = millis();
      //EEPROM.get(8, flightCount);
      //flightCount = flightCount + 1;
      //EEPROM.put(8, flightCount);
    }

    //Check if it has been long enough to take a new ground level measurement.
    if(newGroundRead + 30000 <= millis()){
      newGroundRead = millis();
      groundLock = 0;
      measure();
      groundAltBuffer = altMem / 2;
    }

    //If liftoff hasn't been detected in the past 25 seconds, assume the last measurement of ground was actually at the ground and load it.
    if((newGroundRead + 25000 <= millis()) && (groundLock == 0)){
      groundAlt = groundAltBuffer;
      groundLock = 1;
    }

    if(analogRead(battery) <= 596){ //3.84
      //Battery is below 50% charge, reccomend against flying. blink at 500ms.
      if((millis() - blinkTime) >= 500){
        blinkTime = millis();
        if(blinkState == 0){
          blinkState = 1;
          digitalWrite(redLED, HIGH);
        }
        else{
          blinkState = 0;
          digitalWrite(redLED, LOW);
        }
      }
    }
  
    if(dataLogged == 1){
      //There is still uncleared flight data on the controller. Failure to clear that will result in odd performance. Blink at 100ms.
      if((millis() - blinkTime) >= 100){
        blinkTime = millis();
        if(blinkState == 0){
          blinkState = 1;
          digitalWrite(redLED, HIGH);
        }
        else{
          blinkState = 0;
          digitalWrite(redLED, LOW);
        }
      }
    }
  }

  //Wait for it to be time to take another measurement, and log that measurement if possible:
  if(measStart + dataRate <= millis()){

    //Take a measurement:
    //Log the start time of the measurement:
    measStart = millis();
    measure();
    
    //Check if there is still space to log. If so, log the measurement;
    
    if(address <= 19662){
      //Log it all to memory:
      //Memory Order: Time, Acceleration, Altitude, Temperature
      unsigned long addrByte = 0;
      //If address is 0 and 6553, use EEPROM1.
      if(address <= 6553){
        addrByte = address * 10;
        EEPROM1.put(addrByte, timeMem);
        EEPROM1.put(addrByte+4, accMem);
        EEPROM1.put(addrByte+6, altMem);
        EEPROM1.put(addrByte+8, tempMem);
      }
      //If address is 6554 to 13108, use EEPROM2.
      if((address > 6553) && (address <= 13108)){
        addrByte = (address - 6554) * 10;
        EEPROM2.put(addrByte, timeMem);
        EEPROM2.put(addrByte+4, accMem);
        EEPROM2.put(addrByte+6, altMem);
        EEPROM2.put(addrByte+8, tempMem);
      }
      //If address is 13109 to 19662, use EEPROM3. 
      if(address >= 13109){
        addrByte = (address - 13109) * 10;
        EEPROM3.put(addrByte, timeMem);
        EEPROM3.put(addrByte+4, accMem);
        EEPROM3.put(addrByte+6, altMem);
        EEPROM3.put(addrByte+8, tempMem);
      }
      address = address + 1;

      if(address > 19662){
      goto endBlink;
      }
    } 
    digitalWrite(redLED, LOW);
  } 

  //Check if the altitude has not changed in a long time. If so, landing has occured. stop recording.
  if((landCheck + 10000) <= millis()){
    landCheck = millis();
    barometer.readDigitalValue();
    altitude = barometer.getAltitude();
    if(abs(altitude - oldAltitude) <= 1){
      //Less than 2m change over the last 10 seconds, probably landed
      timeMem = 0;
      //Log a value of 0 for time to indicate its the end of the records
      //If address is 0 and 6553, use EEPROM1.
      if(address <= 6553){
        addrByte = address * 10;
        EEPROM1.put(addrByte, timeMem);
        EEPROM1.put(addrByte+4, accMem);
        EEPROM1.put(addrByte+6, altMem);
        EEPROM1.put(addrByte+8, tempMem);
      }
      //If address is 6554 to 13108, use EEPROM2.
      if((address > 6553) && (address <= 13108)){
        addrByte = (address - 6554) * 10;
        EEPROM2.put(addrByte, timeMem);
        EEPROM2.put(addrByte+4, accMem);
        EEPROM2.put(addrByte+6, altMem);
        EEPROM2.put(addrByte+8, tempMem);
      }
      //If address is 13109 to 19662, use EEPROM3. 
      if(address >= 13109){
        addrByte = (address - 13109) * 10;
        EEPROM3.put(addrByte, timeMem);
        EEPROM3.put(addrByte+4, accMem);
        EEPROM3.put(addrByte+6, altMem);
        EEPROM3.put(addrByte+8, tempMem);
      }
      //Do nothing but blink the LED from now on. 
      while(true){
        endBlink:
        if((millis() - blinkTime) >= 1000){
          blinkTime = millis();
          if(blinkState == 0){
            blinkState = 1;
            digitalWrite(redLED, HIGH);
          } else{
            digitalWrite(redLED, LOW);
            blinkState = 0;
          }
        }     
      }    
    }  else{
      oldAltitude = altitude;    
    }
  }
}
