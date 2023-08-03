/*
 * Note-Started using template Jan 2016 for compiler V 1.6.7
 * @TODO-always remember to update version history and programed compiled with below
 *
 * Circuit details below:
 * MPU6050 connected to I2C bus of monteino and int pin of MPU6050 to D3
 *
 *
 */

//*******************************************general libraries******************************************************
#include <Arduino.h>
// #include <CRC32.h> //@TODO-implement CRC for all EEPROM values
//*******************************************END general libraries******************************************************

//*******************************************for IMU******************************************************
//#include <Wire.h> //I2C used by MPU6050
#include <MPU6050.h>
// imu global variable
//  AD0 pin low = use address 0x68 (default for InvenSense evaluation board)
//  AD0 pin high = 0x69
MPU6050 accelgyro(0x68);
// MPU6050 accelgyro(0x69); // <-- use for AD0 high
// anything coming from IMU outside of these range will be considered motion, anything inside, rest. in whatever units the IMU chooses, doesn't matter since it gets calibrated
int maxRawAcc;
int16_t minRawAcc;
// const int interruptPin = 3; @TODO-implement // Pin number on ATMEGA328 to which which the INT pin from IMU is connected to
//*******************************************END IMU**************************************************

//*******************************************for lora radio******************************************************
//#include <SPI.h> //SPI used by lora and flash chip
#include <RH_RF95.h>
#define RFM95_CS 10                      // chip select for SPI
#define RFM95_INT 2                      // interupt pin
unsigned long nodeID = 1;                // up to 2 million for lorawan?
const boolean forceChangeNodeID = false; // if you want to force the IMU to write this hard coded nodeID to EEPROM and use it. Only needed if you need to change it after it has been programmed
float frequency = 904.0;                 // Specify the desired frequency in MHz
// the transmit power in dB. -4 to 20 in 1 dB steps. NOTE-function itself seems to indicate a range of 2-20 dB or 0-15 dB @TODO-research
int8_t TXpower = 20;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
//*******************************************END lora radio**************************************************

//*******************************************for EEPROM**************************************************
#include <EEPROM.h>
//NOTE-EEPROM values are ints that start at 0
const int nodeIdAddr = 0;     // starting EEPROM address of the 4 bit nodeID and its 4 bit CRC32
const int minRawAccAddr = 8;  // starting EEPROM address of the 2 bit minium acceleration (but 4 bytes reserved) to trigger interupt and its 4 bit CRC32
const int maxRawAccAddr = 16; // starting EEPROM address of the 2 bit max acceleration (but 4 bytes reserved) and its 4 bit CRC32
//*******************************************END EEPROM**************************************************

//*******************************************general global variables******************************************************
boolean debug = true;
boolean infoON = true; // print messages to serial and light LED, not needed if running on own. The device ignores this for first part of setup so that important error messages are always available on serial
float firmwareV;
volatile unsigned long secondsSinceBoot = 0;   // seconds since boot or reset.
volatile unsigned long motionLastDetectedTime; // variable that is written to in ISR and then stored to motionEvents array if conditions are met
unsigned long motionEvents[10];                // the array to store all the time stamps when motion was detected. 0 means ignore the value
const unsigned int baudRate = 9600;
//*******************************************END general global variables******************************************************

//*******************************************functions******************************************************

void writeDataWithChecksumToEEPROM(int address, unsigned long data)
{
  // Write the integer data to EEPROM
  EEPROM.put(address, data);

  // Calculate the CRC32 checksum of the data and save it to EEPROM
  //@TODO-get this to work
  // uint32_t checksum = CRC32::calculate(data, sizeof(data));
  /* this works
   *uint8_t byteBuffer[] = "Hello World";
   *uint32_t checksum = CRC32::calculate(byteBuffer, sizeof(data));
   */

  uint32_t checksum = 3545618; // good enough for now
  EEPROM.put(address + sizeof(data), checksum);
}

boolean testEEPROMchecksum(int address)
{
  //NOTE- for now it uses a static 4 byte value for checksum, not CRC32
  //reads 4 bytes starting at address, then looks at the next 4 bytes for CRC32 value (@TODO)
  //returns- true if calculated CRC32 matches the data read, false if checksum does not match
  
  unsigned long data;
  // read the data from EEPROM
  EEPROM.get(address, data); // sets 'data' to value here (also returns a refernece to data)
  uint32_t checksumEEPROM;
  EEPROM.get(address + sizeof(data), checksumEEPROM);

  // Calculate the CRC32 checksum of the data and compare it to data
  //@TODO-get this to work
  // uint32_t checksumCalc = CRC32::calculate(data, sizeof(data));
  /* this works
   *uint8_t byteBuffer[] = "Hello World";
   *uint32_t checksum = CRC32::calculate(byteBuffer, sizeof(data));
   */

  uint32_t checksumCalc = 3545618; // good enough for now
  if (checksumEEPROM == checksumCalc)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void uncorrectableError(unsigned int errorNum, const char *errorMessage)
{
  /*
  call this to halt the program and notify us of some errors we can't recover from
  Error number meanings:
  1-IMU init error
  2-radio init error
  */

  pinMode(LED_BUILTIN, OUTPUT);
  while (true) // just sit here forever
  {
    if (infoON)
    {
      Serial.print(F("ERROR-uncorrectableError() method called. Halting program & blinking error number times. Error number:"));
      Serial.print(errorNum);
      Serial.println(F(". Error message:"));
      Serial.println(errorMessage);
    }
    for (unsigned int i = 0; i < errorNum; i++) // blink
    {
      digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
      delay(500);                      // Wait for half a second (500 milliseconds)
      digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
      delay(500);                      // Wait for half a second (500 milliseconds)
    }
    delay(500);
    // fade in from min to max
    for (byte fadeValue = 0; fadeValue <= 255; fadeValue += 10)
    {
      // sets the value (range from 0 to 255):
      analogWrite(LED_BUILTIN, fadeValue);
      // wait for 30 milliseconds to see the dimming effect
      delay(30);
    }
    // fade out from max to min
    for (byte fadeValue = 255; fadeValue >= 0; fadeValue -= 10)
    {
      // sets the value (range from 0 to 255):
      analogWrite(LED_BUILTIN, fadeValue);
      // wait for 30 milliseconds to see the dimming effect
      delay(30);
    }
    delay(500);
  }
}

long readVcc()
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
// https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ; // measuring

  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;              // Vcc in millivolts
}

boolean setupIMU()
{
  /*
  starts & tests the IMU.
  Returns: 1 if successful, 0 if failed
  */
  boolean successful = 0;
//  join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  accelgyro.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));

  if (accelgyro.testConnection())
  {
    Serial.println(F("MPU6050 connection successful"));
    successful = 1;
  }
  else
  {
    Serial.println(F("MPU6050 connection failed"));
    successful = 0;
  }

  return successful;
}

boolean calibrateIMU()
{
  /*calibrate the IMU. Returns true if sucessful, false otherwise.
  For now, it always returns true
  */

  boolean successful = 1;

  if (infoON)
  {
    Serial.println(F("Getting stationary thresholds. DO NOT MOVE DEVICE..."));
  }
  delay(1000);

  if (debug)
  {
    Serial.println(F("before saving threshold values, I will show you some from the IMU. CSV format below:"));
    Serial.println(F("x,y,z"));
    for (unsigned long i = 0; i < 2000; i++)
    { // max iterations 4294967295
      Serial.print(accelgyro.getAccelerationX());
      Serial.print(F(","));
      Serial.print(accelgyro.getAccelerationY());
      Serial.print(F(","));
      Serial.println(accelgyro.getAccelerationZ());
    }
  }

  // actually start calibration
  if (infoON)
  {
    digitalWrite(LED_BUILTIN, HIGH); // turn on LED to show we are collecting data
  }
  //@TODO-check if we really need long here. each getAcceleration method returns int16_t. may need if doing this: totalAcc = abs(long(accelgyro.getAccelerationX())) + abs(long(accelgyro.getAccelerationY())) + abs(long(accelgyro.getAccelerationZ()));
  int16_t totalAcc;
  unsigned long startT = millis();

  totalAcc = accelgyro.getAccelerationX();
  minRawAcc = totalAcc;
  maxRawAcc = totalAcc;
  while ((millis() - startT) < long(102000)) // collect max and min for 1.7 mins (102000 ms)
  {
    // totalAcc = abs(long(accelgyro.getAccelerationX())) + abs(long(accelgyro.getAccelerationY())) + abs(long(accelgyro.getAccelerationZ()));
    totalAcc = accelgyro.getAccelerationX();
    if (minRawAcc > totalAcc)
    {
      minRawAcc = totalAcc;
      if (debug)
      {
        Serial.print(F("Tim (ms) elapsed:"));
        Serial.print(millis() - startT);
        Serial.print(F(" New min:"));
        Serial.println(totalAcc);
      }
    }
    if (maxRawAcc < totalAcc)
    {
      maxRawAcc = totalAcc;
      if (debug)
      {
        Serial.print(F("Time (ms) elapsed:"));
        Serial.print(millis() - startT);
        Serial.print(F(" New max:"));
        Serial.println(totalAcc);
      }
    }
    delayMicroseconds(3150); // from IMU_Zero example in MPU6050 library
  }
  // add or subtract a set value for a safety margin. A 11 hour test (on x and z directions, 2 different IMUs) showed that the min and max values were mostly determined within 1.7 mis but that adding 264 to each was enouugh to get the max or min discovered within all 11 hours.
  maxRawAcc = maxRawAcc + 550;
  minRawAcc = minRawAcc - 550;
  if (infoON)
  {
    digitalWrite(LED_BUILTIN, LOW); // turn off led
    Serial.print(F("Done getting stationary thresholds. Maxium set to:"));
    Serial.print(maxRawAcc);
    Serial.print(F(". Minimum set to:"));
    Serial.print(minRawAcc);
    Serial.println(F("."));
  }

  //write these values to EEPROM
    writeDataWithChecksumToEEPROM(minRawAccAddr, minRawAcc);
    writeDataWithChecksumToEEPROM(maxRawAccAddr, maxRawAcc);


  //@TODO-lower sample rate
  //@TODO- set motion detect interrupt
  // use the code below to change accel/gyro offset values
  /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

  return successful;
} // End calibrate IMU

boolean setupRadio()
{
  /*
  Starts the LoRa radio, reads from EEPROM for some settings (e.g. nodeID)
  Returns: 1 if successful, 0 if failed
  */

  boolean worked = 1; // start at 1 and set to 0 if errors

  //@TODO-put in function
  // set the nodeID to value in EEPROM if one
  if (!testEEPROMchecksum(nodeIdAddr) || forceChangeNodeID) // if the checksum is invalid or we are being forced to change nodeID, then set node ID to whatever was hard coded when firmware flashed, else read node ID from EEPROM
  {
    if (debug)
    {
      Serial.print(F("no nodeID found in EEPROM (according to checksum or being forced to update nodeID with forceChangeNodeID), setting hard coded one. Its value:"));
      Serial.println(nodeID);
      Serial.print(F("forceChangeNodeID value:"));
      Serial.println(forceChangeNodeID);
    }
    writeDataWithChecksumToEEPROM(nodeIdAddr, nodeID);
  }
  else
  {
    EEPROM.get(nodeIdAddr, nodeID); // sets data to value here (also returns a refernece to data)
    if (debug)
    {
      Serial.print(F("valid nodeID found in EEPROM, set my nodeID to:"));
      Serial.println(nodeID);
    }
  }

  // start RFM95 radio
  if (!rf95.init())
  {
    if (infoON)
    {
      Serial.println(F("RFM95 initialization failed"));
    }
    worked = 0;
  }

  if (worked)
  {
    // Set RFM95 transmission parameters
    // These 5 paramenter settings results in max range. The allowed values also come from this source (source:M. Bor and U. Roedig, “LoRa Transmission Parameter Selection,” in 2017 13th International Conference on Distributed Computing in Sensor Systems (DCOSS), Jun. 2017, pp. 27–34. doi: 10.1109/DCOSS.2017.10.)
    // Note- some set functions do not return anything so don't need to check for sucess.

    //  Set transmit power to the maximum (20 dBm), @TODO-double check docs for 2nd value. false is if PA_BOOST pin is used and true is RFO pin is used on RFM95 module
    // seems like true (and any value) results in weak signal, false (and any value) results in strong signal
    rf95.setTxPower(TXpower, false);
    // rf95.setTxPower(20, true);
    //  Set coding rate to the highest (4/8)
    rf95.setCodingRate4(8); // 5 to 8. offers protection against bursts of interference, A higher CR offers more protection, but increases time on air.
    // Set spreading factor to the highest (SF12)
    rf95.setSpreadingFactor(12); // 6 to 12. A higher spreading factor increases the Signal to Noise Ratio (SNR), and thus sensitivity and range, but also increases the airtime of the packet. The number of chips per symbol is calculated as 2SF . For example, with an SF of 12 (SF12) 4096 chips/symbol are used. Each increase in SF halves the transmission rate and, hence, doubles transmission duration and ultimately energy consumption.
    // Set bandwidth to the lowest (125 kHz). 15600 and 20800 Hz results in fun sounds to be heard on SDR
    rf95.setSignalBandwidth(125000); // in Hz. Allowed range:7.8 to 500 kHz. typical 125,250 or 500 kHz. Higher BW gives a higher data rate (thus shorter time on air), but a lower sensitivity.  Lower BW also requires more accurate crystals (less ppm).
    // Set the desired frequency @TODO-how to set for optimal range?
    if (!rf95.setFrequency(frequency))
    {
      if (infoON)
      {
        Serial.println(F("Failed to set frequency!"));
      }
      worked = 0;
    }
  } // end if(success)

  if (infoON)
  {
    Serial.println(F("RFM95 initialized."));
    if (debug)
    {
      Serial.println(F("All device registers:"));
      rf95.printRegisters();
      Serial.println();
    }
    Serial.print(F("Device version from register 42:"));
    Serial.println(rf95.getDeviceVersion());
    Serial.println(F("nodeID set to:"));
    Serial.println(nodeID);
    //@TODO-consider printing other things like maxMessageLength() (http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#ab273e242758e3cc2ed2679ef795a7196)
  }

  return worked;
}

byte checkJumperMode()
{
  /*
  reads the pins for the jumpers, does stuff approiate for that mode (like turning on or off serial)
  returns mode (0=debug, 1=calibrate, 2=serial, 3=normal)
  Jumper modes:
pins 4-5 bridged = debug (verbose serial output & LED on) & calibrate
pins 5-6 bridged = calibrate
pins 6-7 bridged = enable outputs like serial and LED (wastes power if not needed)
no pins bridged = normal operation
  */

  byte mode = 3;

  // this is for pins 4 to 7. Set one pin to low, sets next to have input pull up resistor on it. Then Checks if input pull up pin is low. If low, it means its being grounded by the pin before it & it set mode to what corresponds to that pin.
  // note- ATMEGA328P has 20K input pullups. This means at 5V, each read is pulling 5÷20000=0.00025 A (0.25 mA) so OK for a little
  byte x = 4;
  for (x; x <= 6; x++)
  {
    pinMode(x, OUTPUT);
    digitalWrite(x, LOW);
    pinMode(x + 1, INPUT_PULLUP);
    delay(1); // give time to change before reading @TODO-needed?
    if (digitalRead(x + 1) == LOW)
    {
      mode = x - 4; // set the mode. If starting at pin 4, we just subtract 4 to get the mode.
      if (debug)
      {
        Serial.print(F("Mode just set to:"));
        Serial.println(mode);
      }
    }
    // Since we don't need pin x anymore, set back to high impedence to save power @TODO-check if working and required?
    if (debug)
    {
      Serial.print(F("setting INPUT on pin :"));
      Serial.println(x);
    }
    pinMode(x, INPUT);
  }
  // set the last pin back to high impedence
  if (debug)
  {
    Serial.print(F("setting INPUT on pin :"));
    Serial.println(x + 1);
  }
  pinMode(x + 1, INPUT);

  boolean oldInfoON = infoON; // for later in this method
  switch (mode)
  {
  case 0:
    // calibration and debug
    infoON = true; // print messages to serial and light LED, not needed if running on own.
    debug = true;
    break;
  case 1:
    // calibration
    infoON = false; // print messages to serial and light LED, not needed if running on own.
    debug = false;
    break;
  case 2:
    // enable non-battery save output (like LED and serial)
    infoON = true; // print messages to serial and light LED, not needed if running on own.
    debug = false;
    break;
  case 3:
    // normal
    infoON = false; // print messages to serial and light LED, not needed if running on own.
    debug = false;
    break;
  default:
    // should never make it to here
    break;
  }

  if (oldInfoON != infoON)
  { // we toggled state and need to do stuff
    if (infoON)
    {
      Serial.begin(baudRate);
      pinMode(LED_BUILTIN, OUTPUT);
    }
    else
    {
      Serial.end();
      pinMode(LED_BUILTIN, INPUT);
    }
  }

  if (debug)
  {
    Serial.print(F("Mode is: "));
    Serial.println(mode);
  }

  return mode;
}

// ISR function to handle a motion interupt from IMU
void IMUinterupt()
{
  // Perform quick and simple tasks only inside an ISR to keep it fast
  // Avoid using Serial communication or complex operations here
  motionLastDetectedTime = secondsSinceBoot;
}

//*******************************************END functions******************************************************

void setup()
{
  Serial.begin(baudRate);
  Serial.println();
  Serial.println();
  Serial.println(F("Compiled with VSCode and platformIO core V5.2.0 ")); //@TODO-update this whenever compiling
  Serial.println(F("and library: electroniccats/MPU6050@^0.4.0"));
  Serial.println(F("MPU6050 test V 1.0"));
  Serial.println(F("This firmware mointors for elevator motion and sends packets when sensed."));
  /*
   * Version history:
   * V1.0-initial
   *
   * @TODO:
   * allow changing of nodeID somehow (serial or from radio?)
   */

  // Pin setup
  // let us know its alive
  // Note according to (http://www.gammon.com.au/power), power consumption is the same if INPUT or OUTPUT. Thing that matters is internal pullups. If must be high, INPUT preferred
  pinMode(LED_BUILTIN, OUTPUT);
  for (byte i = 0; i < 5; i++) // blink LED a little
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }

  // initilize stuff and throw errors if a failure
  if (!setupIMU())
  {
    uncorrectableError(1, "Error starting IMU!");
  }
  if (!setupRadio())
  {
    uncorrectableError(2, "Error starting radio!");
  }

  byte mode = checkJumperMode();
  if (mode == 0 || mode == 1) // try to calibrate the IMU if we are in a mode that requests it
  {
    if (!calibrateIMU())
    {
      uncorrectableError(3, "Error calibrating IMU!");
    }
  }

  // we should now have valid numbers for maxRawAcc and minRawAcc (either from calibration we just did or from a previous power up), attempt to read them from EEPROM
  //@TODO-put in function?
  int tempAddr = minRawAccAddr;
  // need long data type to handle negative numbers properly @TODO-test with negative
  long tempData = (long)minRawAcc;
  if (!testEEPROMchecksum(tempAddr)) // if the checksum is invalid, then throw error
  {
    if (debug)
    {
      Serial.println(F("EEPROM data invalid at address below (according to checksum). (Its value also below)."));
      Serial.print(F("EEPROM Address in question:"));
      Serial.println(tempAddr);
      }
      uncorrectableError(4, "Invalid EEPROM data for motion acceleration threshold (low value).");
//throw error instead of writing hard coded one
    //writeDataWithChecksumToEEPROM(tempAddr, tempData);
  }
  else
  {
    if (debug)
    {
      Serial.println(F("Valid data in EEPROM found at address below (according to checksum), getting value from EEPROM. (Its value also below)."));
      Serial.print(F("EEPROM Address in question:"));
      Serial.println(tempAddr);
      Serial.print(F("Data sizeof() result:"));
      Serial.println(sizeof(tempData));
      Serial.print(F("overwriting this value:"));
      Serial.println(tempData);
    }

    EEPROM.get(tempAddr, tempData); // sets data to the value in EEPROM (function also returns a refernece to data)
    if (debug)
    {
      Serial.print(F("New value:"));
      Serial.println(tempAddr);
    }
  }
  //@TODO-make sure no problems with this conversion
  minRawAcc = (int)tempData;
  if (debug)
  {
    Serial.print(F("minRawAcc value:"));
    Serial.println(minRawAcc);
  }
  //******Do same for max value
  //@TODO-put in function?
  tempAddr = maxRawAccAddr;
  // need long data type to handle negative numbers properly @TODO-test with negative
  tempData = (long)maxRawAcc;
  if (!testEEPROMchecksum(tempAddr)) // if the checksum is invalid, then throw error
  {
    if (debug)
    {
      Serial.println(F("EEPROM data invalid at address below (according to checksum). (Its value also below)."));
      Serial.print(F("EEPROM Address in question:"));
      Serial.println(tempAddr);
      }
      uncorrectableError(5, "Invalid EEPROM data for motion acceleration threshold (high value).");
//throw error instead of writing hard coded one
    //writeDataWithChecksumToEEPROM(tempAddr, tempData);
  }
  else
  {
    if (debug)
    {
      Serial.println(F("Valid data in EEPROM found at address below (according to checksum), getting value from EEPROM. (Its value also below)."));
      Serial.print(F("EEPROM Address in question:"));
      Serial.println(tempAddr);
      Serial.print(F("Data sizeof() result:"));
      Serial.println(sizeof(tempData));
      Serial.print(F("overwriting this value:"));
      Serial.println(tempData);
    }

    EEPROM.get(tempAddr, tempData); // sets data to the value in EEPROM (function also returns a refernece to data)
    if (debug)
    {
      Serial.print(F("New value:"));
      Serial.println(tempAddr);
    }
  }
  //@TODO-make sure no problems with this conversion
  maxRawAcc = (int)tempData;
  if (debug)
  {
    Serial.print(F("maxRawAcc value:"));
    Serial.println(maxRawAcc);
  }

  // set the IMU interupt for the motion thesholds we either read from EEPROM or calibrated (@TODO-will set interupt in next version of program)
  // pinMode(interruptPin, INPUT_PULLUP); // Set pin 3 as an input with internal pull-up
  // attachInterrupt(digitalPinToInterrupt(interruptPin), detectRisingSignal, FALLING); @TODO-check if the IMU is RISING FALLING or what

//send a "node online" message so the other side knows we have a new "node time" for this nodeID

  if (infoON)
  {
    Serial.println(F("Setup complete. Begin running loop()"));
  }
} // end setup

void loop()
{

  /********************************
IMU stuff
*/

  /*
  //if using the functions requiring pointers
int16_t ax, ay, az;
int16_t gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
*/

  int16_t totalAcc = abs(long(accelgyro.getAccelerationX())) + abs(long(accelgyro.getAccelerationY())) + abs(long(accelgyro.getAccelerationZ()));

  //@TODO-replace this with sleep and intterupt code.
  if ((totalAcc > maxRawAcc) || (totalAcc < minRawAcc))
  {
    Serial.print(F("Motion detected! "));
    if (totalAcc < minRawAcc)
    {
      Serial.print(F("Went below minRawAcc (which is "));
      Serial.print(minRawAcc);
      Serial.print(F(") Value was:"));
      Serial.println(totalAcc);
    }
    if (totalAcc > maxRawAcc)
    {
      Serial.print(F("Went above maxRawAcc (which is "));
      Serial.print(maxRawAcc);
      Serial.print(F(") Value was:"));
      Serial.println(totalAcc);
    }
  }

  delayMicroseconds(3150); // from IMU_Zero example in MPU6050 library
} // end loop
