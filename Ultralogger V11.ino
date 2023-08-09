//Serial Number:
//const int serialNumber = 0001;
//const int versionNumber = 001;

//TODO:
//Add the functionality to subtract ground level from measurements if in AGL mode. 
//Optimize code to add more verbose writing to USB outputs
//Optimize to try and add back in odometer 

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
    43: Liftoff Altitude
*/

void setup() {
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

  if (digitalRead(USB) == 1) {

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

          //Accelerometer:
          acceleration = accRaw() * gOffset;

          //Barometer:
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
        }

        //d: download data from memory
        else if(Serial.peek() == 100){
          serialClear();

          EEPROM.get(36, units);
          EEPROM.get(40, offsetAlt);
          EEPROM.get(43, groundAlt);

          dumping = 1;

          //Add a blank line
          Serial.println();

          //Write a header
            Serial.print(F("Time (s),"));
            Serial.print(F("Acc (g),"));
            Serial.print(F("Alt ")); 
            if(offsetAlt == 0){
              Serial.print(F("ASL "));
            }
            if(offsetAlt == 1){
              Serial.print(F("AGL "));
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
              //If address is 0 and 5460, use EEPROM1.
              if(address <= 5460){
                addrByte = address * 12;
                EEPROM1.get(addrByte, time);
                EEPROM1.get(addrByte+4, accMem);
                EEPROM1.get(addrByte+6, altitude);
                EEPROM1.get(addrByte+10, tempMem);
              }
              //If address is 5461 to 10921, use EEPROM2.
              if((address > 5461) && (address <= 10921)){
                addrByte = (address - 5461) * 12;
                EEPROM2.get(addrByte, time);
                EEPROM2.get(addrByte+4, accMem);
                EEPROM2.get(addrByte+6, altitude);
                EEPROM2.get(addrByte+10, tempMem);
              }

              //If address is 10922 to 16382, use EEPROM3. 
              if(address >= 10922){
                addrByte = (address - 10922) * 12;
                EEPROM3.get(addrByte, time);
                EEPROM3.get(addrByte+4, accMem);
                EEPROM3.get(addrByte+6, altitude);
                EEPROM3.get(addrByte+10, tempMem);
              }
              Serial.flush();
              delay(5);
              address = address + 1;
            } 
            
            //Check if we've reached the end of the memory
            if(address > 19662){
              dumping = 0;
            }

            //Check if we are at the end of measurement memory
            if(time == -1){
              dumping = 0;
              goto bypass;
            }

            //Convert the values to the proper units
            double timeOut = time / 1000.0;
            double accOut = accMem * gOffset;
            double tempOut = tempMem / 100.0;
            if(offsetAlt == 1){
              altitude = altitude - groundAlt;
            }
            if(units == 0){
              altitude = altitude * 3.28084;
              tempOut = (tempOut * 1.8) + 32;
            }

            //Print the values to the serial monitor
            Serial.print(timeOut, 4);
            Serial.print(F(","));
            Serial.print(accOut, 4);
            Serial.print(F(","));
            Serial.print(altitude, 4);
            Serial.print(F(","));
            Serial.println(tempOut, 4);
          }

          bypass:

          //Once all memory is written, ask if the data should be cleared:
          Serial.println();
          //Serial.println(F("Download Complete."));
          address = 0;
          Serial.println(F("Clear? y/n"));
          while (Serial.available() == 0);
          if((Serial.peek() == 121) || (Serial.peek() == 89)){
            Serial.println(F("..."));
            EEPROM1.erase();
            EEPROM2.erase();
            EEPROM3.erase();
            dataLogged = 0;
            EEPROM.put(42, dataLogged);
          } 
          Serial.println(F("Done."));
          Serial.println();
        }

        //e: exit settings, force into the loop (for use in debug/testing)
        else if(Serial.peek() == 101){
          USBIgnore = 1;
          goto looper; 
        } 

        //f: change units to Feet / Farenheit
        else if(Serial.peek() == 102){
          unitsModded = 0;
          confirmed();
        }

        //g or G: change offset mode to AGL
        else if(Serial.peek() == 103){
          offsetAltModded = 1;
          confirmed();
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

          confirmed();
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
          confirmed();
        }

        //m: change units to Meters / Celsius
        else if(Serial.peek() == 109){
          unitsModded = 1;
          confirmed();
        }

        //o or O: Change altitude offset mode to ASL 
        /*else if(Serial.peek() == 111){
          offsetAltModded = 0;
          Serial.println(Conf);
          Serial.println();          
        }*/

        //r xx or R xx: set time between measurements in milliseconds. Up to 50 works (20 Hz)
        else if(Serial.peek() == 114){
          dump = Serial.read();
          dump = Serial.read();
          tempString = Serial.readString();
          dataRateModded = tempString.toInt();
          confirmed();
        }

        //s: save settings
        else if(Serial.peek() == 115){
          EEPROM.update(36, unitsModded);
          EEPROM.update(37, fastLaunchModded);
          EEPROM.update(38, liftoffAccModded);
          EEPROM.update(39, liftoffAltModded);
          EEPROM.update(40, offsetAltModded);
          EEPROM.update(41, dataRateModded);
          confirmed();
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
    EEPROM.get(41, dataRate);
    EEPROM.get(42, dataLogged);

  //Take an intial reading of ground level;
    measure();
    groundAlt = altitude;
    groundAltBuffer = groundAlt;
    newGroundRead = millis();

    blinkTime = millis();
}

void error() {
  //Print an error message
  Serial.println(F("Unkn"));
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
  time = millis() - liftoffTime;
  //Acceleration
  accMem = accRaw();
  //Altitude (average 3 measurements)
  barometer.readDigitalValue();
  altitude = barometer.getAltitude();
  //Temperature
  tempMem = round(barometer.getTemperature() * 100);
}

void confirmed(){
  Serial.println(F("Confirmed"));
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:

  //Wait until liftoff is detected
  while(liftoff == 0){

    //Check if we have exceeded required launch settings
    measure();
    int accCheck = abs(round(accMem * gOffset));
    double altCheck = altitude;

    if((accCheck >= liftoffAcc) && (altCheck >= (liftoffAlt + groundAlt))){
      liftoff = 1;
      dataLogged = 1;
      liftoffTime = millis();
      EEPROM.put(42, dataLogged);
      EEPROM.put(43, groundAlt);
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
      groundAltBuffer = altitude;
    }

    //If liftoff hasn't been detected in the past 25 seconds, assume the last measurement of ground was actually at the ground and load it.
    if((newGroundRead + 25000 <= millis()) && (groundLock == 0)){
      groundAlt = groundAltBuffer;
      groundLock = 1;
    }

    if(analogRead(battery) <= 596){ //3.84 volts
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
      //If address is 0 and 5460, use EEPROM1.
      if(address <= 5460){
        addrByte = address * 12;
        EEPROM1.put(addrByte, time);
        EEPROM1.put(addrByte+4, accMem);
        EEPROM1.put(addrByte+6, altitude);
        EEPROM1.put(addrByte+10, tempMem);
      }
      //If address is 5461 to 10921, use EEPROM2.
      if((address > 5461) && (address <= 10921)){
        addrByte = (address - 5462) * 12;
        EEPROM2.put(addrByte, time);
        EEPROM2.put(addrByte+4, accMem);
        EEPROM2.put(addrByte+6, altitude);
        EEPROM2.put(addrByte+10, tempMem);
      }
      //If address is 10922 to 16382, use EEPROM3. 
      if(address >= 10922){
        addrByte = (address - 10922) * 12;
        EEPROM3.put(addrByte, time);
        EEPROM3.put(addrByte+4, accMem);
        EEPROM3.put(addrByte+6, altitude);
        EEPROM3.put(addrByte+10, tempMem);
      }
      address = address + 1;

      if(address > 19662){
      goto endBlink;
      }
    } 
  } 

  //Check if the altitude has not changed in a long time. If so, landing has occured. stop recording.
  if((landCheck + 10000) <= millis()){
    landCheck = millis();
    measure();
    if(abs(altitude - oldAltitude) <= 1.0){
      //Less than 1m change over the last 10 seconds, probably landed
      time = -1;
      //Log a value of -1 for time to indicate its the end of the records
      //If address is 0 and 5460, use EEPROM1.
      if(address <= 5460){
        addrByte = address * 12;
        EEPROM1.put(addrByte, time);
        EEPROM1.put(addrByte+4, accMem);
        EEPROM1.put(addrByte+6, altitude);
        EEPROM1.put(addrByte+10, tempMem);
      }
      //If address is 5461 to 10921, use EEPROM2.
      if((address > 5461) && (address <= 10921)){
        addrByte = (address - 5461) * 12;
        EEPROM2.put(addrByte, time);
        EEPROM2.put(addrByte+4, accMem);
        EEPROM2.put(addrByte+6, altitude);
        EEPROM2.put(addrByte+10, tempMem);
      }
      //If address is 10922 to 16382, use EEPROM3. 
      if(address >= 10922){
        addrByte = (address - 10922) * 12;
        EEPROM3.put(addrByte, time);
        EEPROM3.put(addrByte+4, accMem);
        EEPROM3.put(addrByte+6, altitude);
        EEPROM3.put(addrByte+10, tempMem);
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
