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
#include "moteino_shared_functions.h" // custom functions for this project
//*******************************************END general libraries******************************************************

//*******************************************for IMU******************************************************
// #include <Wire.h> //I2C used by MPU6050
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
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
// #include <SPI.h> //SPI used by lora and flash chip
#define RFM95_CS 10                      // chip select for SPI
#define RFM95_INT 2                      // interupt pin
unsigned long nodeID = 6;                // up to 2 million for lorawan?
const boolean forceChangeNodeID = false; // if you want to force the IMU to write this hard coded nodeID to EEPROM and use it. Only needed if you need to change it after it has been programmed
float frequency = 904.0;                 // Specify the desired frequency in MHz
// the transmit power in dB. -4 to 20 in 1 dB steps. NOTE-function itself seems to indicate a range of 2-20 dB or 0-15 dB @TODO-research
int8_t TXpower = 20;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
const char *ackMessage = "ACK"; // Acknowledgment message, careful changing this as firmware update of all nodes will be needed. All receive nodes set to not send ACKs to this specific message
//*******************************************END lora radio**************************************************

//*******************************************for EEPROM**************************************************
#include <EEPROM.h>
// NOTE-EEPROM values are ints that start at 0
const int nodeIdAddr = 0;     // starting EEPROM address of the 4 bit nodeID and its 4 bit CRC32
const int minRawAccAddr = 8;  // starting EEPROM address of the 2 bit minium acceleration (but 4 bytes reserved) to trigger interupt and its 4 bit CRC32
const int maxRawAccAddr = 16; // starting EEPROM address of the 2 bit max acceleration (but 4 bytes reserved) and its 4 bit CRC32
//*******************************************END EEPROM**************************************************

//*******************************************general global variables******************************************************
bool debug = true;
unsigned int motionInteruptWakupCount = 0; // for debugging, @TODO-delete
byte lastMotion1 = 0;                      // for debugging, @TODO-delete
byte lastMotion2 = 0;                      // for debugging, @TODO-delete
unsigned int motionEventOneCount = 0;      // for debugging, @TODO-delete
unsigned int motionEventTwoCount = 0;      // for debugging, @TODO-delete
boolean infoON = true;                     // print messages to serial and light LED, not needed if running on own. The device ignores this for first part of setup so that important error messages are always available on serial
unsigned int messageType = 1;              // sent with sensor type and firmware version
const unsigned int sensorType = 2;         // show what sensor type we are using (numbers tracked in other document)
unsigned int firmwareVMajor;
unsigned int firmwareVMinor;
unsigned long motionLastDetectedTime;
volatile bool motionInteruptTriggered = false; // variable that is written to in ISR
unsigned long motionEvents[10] = {0};          // DO NOT CHANGE DATA TYPE WITHOUT EDITING CODE the array to store all the time stamps when motion was detected. 0 means ignore the value
const unsigned int baudRate = 9600;
const byte randSeedPin = A0; // pin we read from to get a random number to seed the random number generator. Could be removed if needed since the seed it set multiple times in setup
const byte sleepT = 8;       // in seconds, the time we either simulate the microcontroller going to sleep or actually going to sleep
//*******************************************END general global variables******************************************************

//*******************************************functions******************************************************

unsigned long updateAndGetSecsSinceBoot(byte wakeReason)
{
  // updates the secondsSinceBoot variable, does not work well when waking from interrupts
  // counting seconds with a 32 bit unsigned long, (2^32) we should be able to track 4294967296÷60÷60÷24÷365=136.2 years
  // MUST be called before going to sleep for first time with wakeReason 0. Must be called before sleeping with wakeReason0.
  // MUST be called right after sleep or interupt wake
  // wake reason 0=normal, 1=from sleep 2=from interupt

  static unsigned long lastCallT = millis() / 1000;
  static unsigned long secondsSinceBoot = millis() / 1000; // seconds since boot or reset.

  // unsigned long seconds=millis()/1000; @TODO-is just having this value faster than recomputing it several times?

  if ((lastCallT * 1000) > millis()) // GPT3.5-multiplication takes 2 clock cycles regardless of operand size. but div takes 22-42 clock cycles @TODO-check this
  {                                  // our lastCallT should always be less than or equal to the current time. If not, millis() has overflowed
    //@TODO-handle overflow of millis() better
    lastCallT = 0; // Best we can do??
    if (debug)
    {
      Serial.println(F("WARNING-Detected millis() overflow!"));
    }
  }

  unsigned long old = secondsSinceBoot; //@TODO-delete
  if (debug && false)
  {
    Serial.print(F("updateSecsSinceBoot():old SecsSinceBoot:"));
    Serial.println(secondsSinceBoot);
  }

  switch (wakeReason)
  {
  case 0:                                                                  // normal function call so we can update our time with just the time elapsed since this function was last called
    secondsSinceBoot = secondsSinceBoot + ((millis() / 1000) - lastCallT); //@TODO-which is faster, doing same operation twice or
    break;
  case 1: // woke from sleep so we know sleepT has elapassed
    secondsSinceBoot = secondsSinceBoot + sleepT;
    break;
  case 2: // woke from an interupt so not sure how much time elapased, assume an average @TODO-find better way to update this
    secondsSinceBoot = secondsSinceBoot + (sleepT / 2);
    break;
  default:
    break;
  }
  lastCallT = millis() / 1000; // reset our timer

  if (debug && false)
  {
    Serial.print(F("updateSecsSinceBoot():new SecsSinceBoot:"));
    Serial.println(secondsSinceBoot);
    Serial.print(F("updateSecsSinceBoot():elapsed:"));
    Serial.println(secondsSinceBoot - old);
  }

  return secondsSinceBoot;
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

  if (infoON)
  {
    // initialize device
    Serial.println(F("Initializing IMU..."));
  }
  accelgyro.initialize();

  if (accelgyro.testConnection())
  {
    if (infoON)
    {
      Serial.println(F("IMU connection good!"));
    }
    successful = 1;
  }
  else
  {
    if (infoON)
    {
      Serial.println(F("IMU connection failed!"));
    }
    successful = 0;
  }

  // Set defaults range of IMU. done in initilize routine but set anyway
  // Sets acc/gyro to most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets the clock source to use the X Gyro for reference, which is slightly better than the default internal clock source.
  // accelgyro.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  // accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  //@TODO-consider setting these things:
  // accelgyro.setDLPFMode(); //dighital high pass filter
  // turn off IMU axes not needed @TODO-test
  // setStandbyYAccelEnabled(true);
  // setStandbyZAccelEnabled(true);
  // setStandbyXGyroEnabled(true); //do not put this to sleep as I think it uses this as the clock source @TODO-check
  // setStandbyYGyroEnabled(true);
  // setStandbyZGyroEnabled(true);

  if (debug)
  {
    Serial.print(F("Accelerometer full scale range set to (0 = +/- 2g,  1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g):"));
    Serial.println(accelgyro.getFullScaleAccelRange());
  }

  return successful;
}

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
      Serial.print(F("No nodeID found in EEPROM or forced update with forceChangeNodeID=TRUE. Setting NodeID to:"));
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

    //@TODO- add setPayloadCRC(true); (this is default but needs testing), setPreambleLength(8)
  } // end if(worked)

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
    Serial.print(F("nodeID set to:"));
    Serial.println(nodeID);
    //@TODO-consider printing other things like maxMessageLength() (http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#ab273e242758e3cc2ed2679ef795a7196)
  }

  return worked;
}

boolean calibrateIMU()
{
  /*calibrate the IMU. Returns true if sucessful, false otherwise.
  For now, it always returns true
  */

  boolean successful = 1;
  boolean infoONOld = infoON;
  infoON = true; // force for now

  if (infoON)
  {
    Serial.println(F("Getting stationary thresholds. DO NOT MOVE DEVICE..."));
    fadeLED(10); // fade the LED to warn user not to touch
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
  unsigned long calibrationT = 102000;
  //unsigned long calibrationT = 39600000;//11 hours for debugging
  /*
  if (debug && false)
  {
    Serial.println(F("DEBUG-CalibrateIMU():shortening calibration time to 5s."));
    calibrationT = 1000;
  }
  */
  while ((millis() - startT) < calibrationT) // collect max and min for 1.7 mins (102000 ms)
  {
    totalAcc = accelgyro.getAccelerationX();
    if (minRawAcc > totalAcc)
    {
      minRawAcc = totalAcc;
      if (debug)
      {
        Serial.print(F("Time (ms) elapsed:"));
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
  maxRawAcc = maxRawAcc + 550    ;
  minRawAcc = minRawAcc - 550;
  if (infoON)
  {
    digitalWrite(LED_BUILTIN, LOW); // turn off led
    Serial.print(F("Done w/ stationary thresholds. Max set to:"));
    Serial.print(maxRawAcc);
    Serial.print(F(", Min set to:"));
    Serial.println(minRawAcc);
    fadeLED(7);
  }

  // write these values to EEPROM
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
  infoON = infoONOld; // set back to what it was

  return successful;
} // End calibrate IMU

byte checkJumperMode()
{
  /*
  reads the pins for the jumpers, does stuff approiate for that mode (like turning on or off serial)
  returns mode (0=debug, 1=calibrate, 2=serial, 3=normal)
  Jumper modes:
mode 0 (pins 4-5 bridged) = debug (verbose serial output & LED on) & calibrate
mode 1 (pins 5-6 bridged) = calibrate (output is still enabled)
mode 2 (pins 6-7 bridged) = enable outputs like serial and LED (wastes power if not needed)
mode 3 (no pins bridged) = normal operation
  */

  byte mode = 3;

  // this is for pins 4 to 7. Set one pin to low, sets next to have input pull up resistor on it. Then Checks if input pull up pin is low. If low, it means its being grounded by the pin before it & it set mode to what corresponds to that pin.
  // note- ATMEGA328P has 20K input pullups. This means at 5V, each read is pulling 5÷20000=0.00025 A (0.25 mA) so OK for a little
  byte x;
  for (x = 4; x <= 6; x++)
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
    Serial.print(F("Mode is:"));
    Serial.println(mode);
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
    infoON = true; // print messages to serial and light LED, not needed if running on own.
    debug = false;
    break;
  case 2:
    // enable non-battery save output (like LED and serial)
    infoON = true; // print messages to serial and light LED, not needed if running on own.
    debug = false;
    break;
  case 3:
    // normal
    infoON = false;
    debug = false;
    break;
  default:
    // should never make it to here
    break;
  }

  if (oldInfoON != infoON)
  { // we toggled state and need to enable or disable output devices
    if (infoON)
    {
      Serial.begin(baudRate);
      pinMode(LED_BUILTIN, OUTPUT);
      Serial.print(F("Mode is:"));
      Serial.println(mode);
    }
    else
    {
      Serial.end();
      pinMode(LED_BUILTIN, INPUT);
    }
  }

  return mode;
}

byte countMotionEvents()
{ // count the number of motion events in the SRAM buffer
  byte count = 0;
  for (byte i = 0; i < (sizeof(motionEvents) / sizeof(unsigned long)); i++)
  {
    if (motionEvents[i] != 0)
    {
      count++;
    }
    if (debug)
    {
      Serial.print(F("i:"));
      Serial.print(i);
      Serial.print(F(" motionevents[i]:"));
      Serial.print(motionEvents[i]);
      Serial.print(F(" count:"));
      Serial.println(count);
    }
  }
  return count;
}

byte sendNodeInfoRadioPacket(unsigned int numEvents, unsigned long lastMotionTime)
{
  // sends a radio packet with the number of motion events and the timestamp of the last motion time (in seconds)
  // returns 0 if it worked, 1 if there was a problem with creating the message buffer to send (currently only if debug=true), 2 if problem with send() method

  byte exitCode = 0;

  long supplyV = readVcc(); //@TODO-is this really where we want to be reading supply voltage? perhaps change to parameter we pass to it
  //@TODO-is there a better way than using a garbage variable? CLEAN THIS UP!!
  char garbage[1];
  int lengthNeeded; // number of characters that would have been written if message had been sufficiently large, not counting the terminating null character.
  if (debug && false)
  {
    messageType = 2;
    lengthNeeded = snprintf(garbage, sizeof(garbage), "%u.%u.%u.%u,%lu,%ld,%u,%lu,%hu,%hu,%u,%u,%u", messageType, sensorType, firmwareVMajor, firmwareVMinor, nodeID, supplyV, numEvents, lastMotionTime, lastMotion1, lastMotion2, motionInteruptWakupCount, motionEventOneCount, motionEventTwoCount);
  }
  else
  {
    messageType = 1;
    // format specifiers:%u for unsigned int, %hu for byte (@TODO-test)  %lu for unsigned long, %ld for long, %d for integers (decimal format), %f for floating-point numbers (floating-point format), %c for characters, %s for strings
    lengthNeeded = snprintf(garbage, sizeof(garbage), "%u.%u.%u.%u,%lu,%ld,%u,%lu", messageType, sensorType, firmwareVMajor, firmwareVMinor, nodeID, supplyV, numEvents, lastMotionTime); // A terminating null character is automatically appended after the content written. https://cplusplus.com/reference/cstdio/snprintf/
  }
  char message[lengthNeeded * sizeof(char) + 1];
  if (debug && false) //@TODO-figure this debug message out
  {
    lengthNeeded = snprintf(message, sizeof(message), "%u.%u.%u.%u,%lu,%ld,%u,%lu,%hu,%hu,%u,%u,%u", messageType, sensorType, firmwareVMajor, firmwareVMinor, nodeID, supplyV, numEvents, lastMotionTime, lastMotion1, lastMotion2, motionInteruptWakupCount, motionEventOneCount, motionEventTwoCount);
  }
  else
  {
    lengthNeeded = snprintf(message, sizeof(message), "%u.%u.%u.%u,%lu,%ld,%u,%lu", messageType, sensorType, firmwareVMajor, firmwareVMinor, nodeID, supplyV, numEvents, lastMotionTime); // A terminating null character is automatically appended after the content written. https://cplusplus.com/reference/cstdio/snprintf/
  }

  if ((unsigned long)(lengthNeeded + 1) > (sizeof(message) / sizeof(char)))
  { // truncated message alert!
    if (infoON)
    {
      Serial.println(F("WARNING-char *message too short to contain radio message. Truncated by snprintf."));
    }
    if (debug)
    { // consider this a fatal error only when debuging since we still want it to send the message @TODO-do we want this behavior, we should never reach here?
      exitCode = 1;
    }
  }

  if (infoON)
  {
    Serial.print(F("Sending data:"));
    Serial.println(message);
    fadeLED(4);
  }
  // wait if there is another packet already sending, not really neeed since send() does this anyway
  rf95.waitPacketSent();
  if (!rf95.send((uint8_t *)message, strlen(message)))
  {
    exitCode = 2;
  }
  // rf95.waitPacketSent(); //this just adds useless delay
  if (infoON)
  {
    Serial.println(F("Packet send finishing soon or already sent."));
  }

  return exitCode;
}

byte checkForACK(unsigned int msToWaitAfterSend)
{
  /*
  checks the radio to see if we have an ACK message.
  Returns:
  0 if ACK received
  1 if some other message received
  2 if nothing received
  */
  byte exitCode = 2;
  unsigned long startT = millis(); //@TODO-remove this
  if (debug)
  {
    Serial.print(F("checkForAck():Wait to finish send (if sending)..."));
  }
  rf95.waitPacketSent(); // wait if we are sending something else. Can't get an ACK if we are still sending.

  if (debug)
  {
    Serial.print(F("Done waiting. ms elapsed:"));
    Serial.println(millis() - startT);
    Serial.print(F("checkForAck():now waiting for ACK. using waitCAD and delaying:"));
    Serial.println(msToWaitAfterSend);
  }
  startT = millis();
  delay(msToWaitAfterSend);
  /*This caused program to hang on the 4th call of this function
  rf95.setModeRx();
  delay(msToWaitAfterSend);
  while (rf95.isChannelActive())
  {
    if (debug)
    {
      Serial.println(F("checkForAck(): being held by isChannelActive()"));
      delay(1);
    }
  }
  */
  rf95.waitCAD();
  delay(50); //@TODO- what time is reasonable to give it time to process ACK?
  if (debug)
  {

    Serial.print(F("checkForAck():Done waiting to RX ACK, checking now. ms elapsed:"));
    Serial.println(millis() - startT);
  }
  if (rf95.available())
  { // note- we need to delay because .available() returns true only if there is a new, complete, error free message
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) // recv wants two pointers. You don't need to use &buf because C++ returns the address to the first element &len returns memory address of len
    {
      if (debug)
      {
        Serial.print(F("checkForACK():received message:"));
        Serial.println((char *)buf); // casts this as a chracter array?? @TODO-check
      }
      boolean messageIsACK = strncmp((char *)buf, ackMessage, strlen(ackMessage)) == 0; // compaire the first characters of buffer to see if this is ACK message
      if (!messageIsACK)
      {
        exitCode = 1;
      }
      else
      {
        exitCode = 0;
      }

    } // end if rf95.recv
  }   // end if rf95.available()

  return exitCode;
}

byte sendNodeInfoRadioPacketWithACK(byte triesToAttempt, unsigned int numEvents, unsigned long lastMotionTime)
{

  //@TODO-consider returning the exit code from the ACK plus exit code from send?
  // send a packet with the specified number of attempts to try 0=try forever
  // returns 0 if all good, 1 if an error sending, 2 if error receiving

  //@TODO-this method is not reliable to get ACKs. calculate how long it takes to send an ACK using the given paramaters. datasheet is garbage but it seems to indicate the time is something like this:
  // Ts=(2^SF)/(BW(in kHz??)*1000)
  // Tpreable=(Lpreamble+4.25)*Ts Lpreamble default is 12
  // Tpayload=Ts*(8+ROUNDUP((8*Lpayload-4*SF+[implicit header=24, explicit=44])/(4*SF)))*(CR+4)
  // T on air=Tpreable+Tpayload
  // Assuming 20 km (lora abs max range) and speed of light 300,000 km/s 20÷300000=0.000066667 (66.7 us) one way max
  // for some reason, initial delay of 10000 doesn't work. 110 works but only on second transmit
  unsigned int timeToWaitForAck = 101; // time in ms we will wait for an ACK, after our message has sent. Trip time + rx delay + time on air + trip time

  bool decrement = true;
  if (triesToAttempt == 0)
  { // turn off decrementing and set the tries to 1.
    decrement = false;
    triesToAttempt = 1;
  }

  byte exitCodefromACK;
  byte triesForACK = triesToAttempt;

  // my hackey attempt to try to clear the receive buffer @TODO-fix this issue
  exitCodefromACK = checkForACK(0);
  if (debug)
  {
    Serial.print(F("Try to clear RX buffer. exitCodefromACK:"));
    Serial.println(exitCodefromACK);
  }
  exitCodefromACK = checkForACK(0);
  if (debug)
  {
    Serial.print(F("Try to clear RX buffer. exitCodefromACK:"));
    Serial.println(exitCodefromACK);
  }

  do // while loop for send, wait for ACK
  {
    byte triesToSend = triesToAttempt; // reset the tries to send
    byte exitCodeSend;
    do // while loop for attempt to send. Should not really ever get an exit code != 0 but just in case
    {
      exitCodeSend = sendNodeInfoRadioPacket(numEvents, lastMotionTime);
      if (decrement)
      {
        triesToSend--;
      }
      if (debug)
      {
        Serial.print(F("triesToSend:"));
        Serial.println(triesToSend);
        Serial.print(F("exitCodeSend:"));
        Serial.println(exitCodeSend);
      }
    } while (triesToSend && (exitCodeSend != 0)); // while we still have tries remaining and we did not get an exit code of 0 (success)
    if (exitCodeSend != 0)
    { // if we don't get success back from this after we tried, then we had some problem sending that we can't fix
      return 1;
    }

    // now look for an ACK
    exitCodefromACK = checkForACK(timeToWaitForAck); //@TODO-change behavior here to handle no message received at all vs message received but not ACK
    if (decrement)
    {
      triesForACK--;
    }
    if (exitCodefromACK != 0) // if we didn't get an ACK, increase the time we wait and also delay a random amount of time before trying to send again
    {
      timeToWaitForAck = timeToWaitForAck + 1; //@TODO-determine how much time to wait better
      if (infoON)
      {
        Serial.print(F("No ACK received. Tries remaining:"));
        Serial.print(triesForACK);
        Serial.print(F(". New ms to wait for ACK:"));
        Serial.println(timeToWaitForAck);
        fadeLED(5);
      }
      if (triesForACK)
      {                             // if we still have tries remaining, delay some and send again
        delay(random(1000, 10000)); // wait a random ammount of time within a certain range of ms. @TODO-write code to calculate how long it takes to send ACK, given current TX params and set this as minimum
      }
    }

    if (debug)
    {
      Serial.print(F("exitCodefromACK:"));
      Serial.println(exitCodefromACK);
      Serial.print(F("triesForACK:"));
      Serial.println(triesForACK);
    }
  } while (triesForACK && (exitCodefromACK != 0)); // keep trying if we still have tries remaining & did not get an ACK

  if ((exitCodefromACK != 0))
  { // if we don't get success back from this after we tried & tried, then we had some problem receving the ACK
    if (infoON)
    {
      Serial.println(F("Tries exausted with no ACK. Giving up."));
    }
    return 2;
  }
  else
  {
    if (infoON)
    {
      Serial.println(F("ACK received!"));
      fadeLED(3);
    }
    return 0;
  }
}

byte checkAccelInRangeForT(int32_t upper_limit, int32_t lower_limit, uint32_t time_in_range_ms, uint32_t timeout_ms)
{
  // Function to check if a variable remains above a certain number or below a certain number
  // for a specified period of time.
  // Parameters:
  //   upper_limit: The upper limit to check if the variable is above this number.
  //   lower_limit: The lower limit to check if the variable is below this number.
  //   time_in_range_ms: The time in milliseconds that the variable must remain within the limits.
  //   timeout_ms: The timeout period in milliseconds.

  // Returns:
  //   0 if there was a timeout
  //   1 if the variable remained above 'upper_limit',
  //   2 if the variable remained below 'lower_limit'.

  unsigned int delay_us = 1000; // The delay between IMU reads in microseconds. Note that per data sheet max sample rate is 1000 Hz
  uint32_t startTime = millis();

  bool inRangePreviously = false;
  uint32_t timeInLimit = 0;

  while ((millis() - startTime) < timeout_ms)
  {
    int16_t totalAcc = long(accelgyro.getAccelerationX());

    if ((totalAcc < upper_limit) && (totalAcc > lower_limit)) // if the value is in deadband
    {
      inRangePreviously = false;
      timeInLimit = 0;
    }
    else if (!inRangePreviously) // If the variable just got back within the range, reset the timer.
    {
      inRangePreviously = true;
      timeInLimit = 0;
    } // otherwise, leave the variables along

    if (inRangePreviously) // update our timer
    {
      timeInLimit += (delay_us / 1000); //@TODO-get rid of this division here!!
      if (debug)
      {
        // Serial.print(F("timeInLimit:"));
        // Serial.println(timeInLimit);
        digitalWrite(LED_BUILTIN, HIGH);
        delayMicroseconds(50);
        digitalWrite(LED_BUILTIN, LOW);
      }
      if (timeInLimit >= time_in_range_ms)
      {
        if (totalAcc >= upper_limit)
        {
          return 1; // 'totalAcc' remained above 'upper_limit' for the specified time period.
        }
        else if (totalAcc <= lower_limit)
        {
          return 2; // 'totalAcc' remained below 'lower_limit' for the specified time period.
        }
      }
    }

    static unsigned long previousdelayT = micros();
    while ((micros() - previousdelayT) < delay_us)
    { // while elapsed time is less than the delay, wait
    }
    previousdelayT = micros(); // reset
    // delay(delay_us/1000));
  } // end the while loop for the timeout

  return 0; // Timeout occurred, the variable did not remain above 'upper_limit' or below 'lower_limit' for the specified time period.
}

// ISR function to handle a motion interupt from IMU
void IMUinterupt()
{
  // Perform quick and simple tasks only inside an ISR to keep it fast. Avoid using Serial communication or complex operations here
  motionInteruptTriggered = true;
}

//*******************************************END functions******************************************************

void setup()
{
  /*
   * Version history:
   * V1.0-initial
   * V1.1-major revisions to packet structure
   * V1.2-changes to timings, threshold and some algorithmic changes to get it to WORK (Note-some early tests show V1.1)
   *
   * @TODO:
   * allow changing of nodeID somehow (serial or from radio?)
   * change delimiters from a simple comma to a set of perhaps -_,<>:;~ to get a base 8 system to transmit other data at the end. If needing to transmit 3 variables (plus one encoded in delims), we would have 2 delimiters, resulting in 64 possible combinations (5 bits). OR if using a set of 16 delims (as with this set `~-_=+[{}];:,</?) can send a full byte with 2 delims
   * switch all methods to return exit codes instead of boolean
   */
  firmwareVMajor = 1;
  firmwareVMinor = 2;

  Serial.begin(baudRate);
  Serial.println();
  Serial.println();
  Serial.println(F("Compiled with VSCode and platformIO core")); //@TODO-update this whenever compiling
  Serial.println(F("and libraries: electroniccats/MPU6050, mikem/RadioHead"));
  Serial.print(F("Elevator sensor V "));
  Serial.print(sensorType);
  Serial.print('.');
  Serial.print(firmwareVMajor);
  Serial.print('.');
  Serial.println(firmwareVMinor);
  Serial.println(F("This firmware mointors for elevator motion and sends packets when sensed."));

  // Pin setup & RNG setup
  // Note according to (http://www.gammon.com.au/power), power consumption is the same if INPUT or OUTPUT. Thing that matters is internal pullups. If must be high, INPUT preferred
  pinMode(LED_BUILTIN, OUTPUT);
  fadeLED(5);
  unsigned long randSeed = analogRead(randSeedPin);
  randomSeed(randSeed);
  if (debug)
  {
    Serial.print(F("randSeed="));
    Serial.println(randSeed);
  }

  // initilize hardware and call error method if a failure
  if (!setupIMU())
  {
    uncorrectableError(1);
  }

  if (!setupRadio())
  {
    uncorrectableError(2);
  }

  byte mode = checkJumperMode();
  if (mode == 1 || mode == 0) // try to calibrate the IMU if we are in a mode that requests it
  {
    if (!calibrateIMU())
    {
      uncorrectableError(3);
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
      Serial.print(F("EEPROM data invalid at address:"));
      Serial.println(tempAddr);
    }
    uncorrectableError(4);
    // or we could just go with default if we didn't want this to be an error writeDataWithChecksumToEEPROM(tempAddr, tempData);
  }
  else
  {
    if (debug)
    {
      Serial.print(F("Valid data at EEPROM address:"));
      Serial.println(tempAddr);
      Serial.print(F("sizeof(data):"));
      Serial.println(sizeof(tempData));
      Serial.print(F("Old value in SRAM:"));
      Serial.println(tempData);
    }

    EEPROM.get(tempAddr, tempData); // sets data to the value in EEPROM (function also returns a refernece to data)
    if (debug)
    {
      Serial.print(F("New value from EEPROM, now in SRAM:"));
      Serial.println(tempData);
    }
  }
  //@TODO-make sure no problems with this conversion
  minRawAcc = (int)tempData;
  if (infoON)
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
    uncorrectableError(5);
    // throw error instead of writing hard coded one
    // writeDataWithChecksumToEEPROM(tempAddr, tempData);
  }
  else
  {
    if (debug)
    {
      Serial.print(F("Valid data at EEPROM address:"));
      Serial.println(tempAddr);
      Serial.print(F("sizeof(data):"));
      Serial.println(sizeof(tempData));
      Serial.print(F("Old value in SRAM:"));
      Serial.println(tempData);
    }

    EEPROM.get(tempAddr, tempData); // sets data to the value in EEPROM (function also returns a refernece to data)
    if (debug)
    {
      Serial.print(F("New value from EEPROM, now in SRAM:"));
      Serial.println(tempData);
    }
  }
  //@TODO-make sure no problems with this conversion
  maxRawAcc = (int)tempData;
  if (infoON)
  {
    Serial.print(F("maxRawAcc value:"));
    Serial.println(maxRawAcc);
  }

  //*******send a "new node online" message so the other side is aware of this nodeID's time, try 255 times. When we say we have 2^16-1 motion events, it alerts the other side this is a "new node online" packet
  // if (sendNodeInfoRadioPacketWithACK(255, 65535, updateAndGetSecsSinceBoot(0)) != 0) //@TODO-refresh seconds time each time it sends
  if (sendNodeInfoRadioPacketWithACK(255, 65535, updateAndGetSecsSinceBoot(0)) != 0) //@TODO-refresh seconds time each time it sends
  {
    uncorrectableError(6);
  }

  //@TODO-program this value in as an interupt
  // accelgyro.setMotionDetectionThreshold();
  // setMotionDetectionDuration();
  // setZeroMotionDetectionThreshold();
  // setZeroMotionDetectionDuration();
  // setInterruptDrive(0);//set to pulse interupt @TODO check
  // setInterruptMode() open drain or push pull @TODO-which uses less power?
  // setIntEnabled()//@TODO-see which interupts we should enable
  // setIntMotionEnabled(true);
  // setIntZeroMotionEnabled(true);
  // set the IMU interupt for the motion thesholds we either read from EEPROM or calibrated (@TODO-will set interupt in next version of program)
  // pinMode(interruptPin, INPUT_PULLUP); // Set pin 3 as an input with internal pull-up
  // attachInterrupt(digitalPinToInterrupt(interruptPin), detectRisingSignal, FALLING); @TODO-check if the IMU is RISING FALLING or what


  if (debug)
  {
    Serial.println(F("DEBUG-priting IMU readings..."));
    Serial.println(F("time (ms),x,y,z"));
    for (unsigned int i = 0; i < 20; i++)
    { // max iterations for unsigned long is 4294967295
      char delim = ',';
      Serial.print(millis());
      Serial.print(delim);
      Serial.print(accelgyro.getAccelerationX());
      Serial.print(delim);
      Serial.print(accelgyro.getAccelerationY());
      Serial.print(delim);
      Serial.println(accelgyro.getAccelerationZ());
      delay(1);
    }
  }

  // setup should take such a variable amount of time, this should be good
  randSeed = micros() + random(); // unsigned long
  randomSeed(randSeed);
  if (debug)
  {
    Serial.print(F("randSeed="));
    Serial.println(randSeed);
  }

  if (infoON)
  {
    Serial.println(F("\n\nSetup complete! Begin motion monitoring...\n\n"));
    fadeLED(6);
  }

  updateAndGetSecsSinceBoot(0); // last time we know millis() will be accurate
} // end setup

void loop()
{

  static unsigned int motionEvents = 0; // the number of motion events

  // ************put things to sleep to save power
  if (!rf95.sleep()) // good if sucessful at putting radio to sleep, but doesn't really matter right now if it didn't go to sleep
  {
    if (debug)
    {
      Serial.println(F("WARNING-problem putting radio to sleep"));
    }
    //@TODO-maybe send a packet out with an error?
  }
  //@TODO- consider putting IMU to low power also
  /*
setSleepEnabled(true);
setWakeFrequency(3);//3=sample at 10Hz, 0=sample at k1.25Hz
setSleepEnabled(false);
*/
  // simulate microcontroller sleeping for sleepT
  unsigned long sleepStartT = millis();
  do
  {
    /*
    //if using the functions requiring pointers
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  */
    // int16_t totalAcc = abs(long(accelgyro.getAccelerationX())) + abs(long(accelgyro.getAccelerationY())) + abs(long(accelgyro.getAccelerationZ()));
    int16_t totalAcc = long(accelgyro.getAccelerationX());
    if ((totalAcc > maxRawAcc) || (totalAcc < minRawAcc))
    {
      IMUinterupt(); // call ISR to set motionInteruptTriggered to true
      if (infoON)
      {
        Serial.println(F("------motion threshold triggered------"));
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
    }
    delay(10); // save power? @TODO-add ability to set sample rate. Note that per data sheet max sample rate is 1000 Hz so faster than this is certainly wasting power
    // delayMicroseconds(3150);// from IMU_Zero example in MPU6050 library
  } while ((!motionInteruptTriggered) && ((millis() - sleepStartT) < (sleepT * 1000))); // continue loop if we didn't trigger motion int & time elapsed is less than the sleep time
  // code will start here when wake from sleep
  if (motionInteruptTriggered)
  {
    updateAndGetSecsSinceBoot(2); // call this telling it we woke from an interrupt
    motionInteruptWakupCount++;   //@TODO-for debug,delete
  }
  else
  {
    updateAndGetSecsSinceBoot(1); // call this if we work up from sleep after timeout
  }

  //********check to see if we woke up to the elevator moving or just nonsense
  bool motionEvent = false;
  if (motionInteruptTriggered) // actually check to see if elevator is moving. @TODO-move this to a function
  {
    //@TODO-do we want to warm up radio here? "Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode." (http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#a6f4fef2a1f40e704055bff09799f08cf)
    // rf95.setModeIdle();
    // disable interupt from IMU (but not interupt for radio)

    //@TODO-change some to unsigned long
    unsigned int miniumElevatorAccTime = 550; // in ms, time elevator is outside the deadband of acceleromenter (continiously), min value of all trials
    unsigned int elevatorAccTime = 2000;     // in ms, time the elevator takes to finish its initial acceleration, max value of all trials
    unsigned int minTimeBetweenAcc = 700;     // in ms, time the elevator takes between acceleration and deceleration, min value of all trials
    unsigned int maxDoorToDoorTime = 16000;  // in ms, time it takes elevator to transit from top to bottom or bottom to top (in practce this could be the max time between the end of initial acceleraton and the end of decelleraton), max value of all trials
    byte motion1 = 0;                        // set to 1 (one directio) or 2 (other direction) if out of deadband for miniumElevatorAccTime
    byte motion2 = 0;                        // set to 1 (one directio) or 2 (other direction) if out of deadband for miniumElevatorAccTime
    if (debug)
    {
      Serial.println(F("Begin looking for motion1"));
    }
    //@TODO-get this to do something with the times and record actual max/min acceleraton values
    unsigned long startMot1T = updateAndGetSecsSinceBoot(0);
    motion1 = checkAccelInRangeForT((maxRawAcc-550), (minRawAcc+910), miniumElevatorAccTime, elevatorAccTime);
    lastMotion1 = motion1; //@TODO-for debug,delete
    if (infoON)
    { // give indicator if we sensed motion
      if (motion1)
      {
        motionEventOneCount++; //@TODO-for debug,delete
        // fadeLED(motion1);//dont want to add delay in time sensitive code
        Serial.println(F("1st acceleration or decel sensed."));
        if (debug)
        {
          Serial.print(F("motion1:"));
          Serial.println(motion1);
        }
      }
      else
      {
        Serial.println(F("Timeout. No motion sensed, skip looking for 2nd event"));
      }
    }
    unsigned long startMot2T = updateAndGetSecsSinceBoot(0);

    if (motion1) // if we didn't see motion1, no need to check for motion2
    {
      //@TODO-test minium delay between floors
      if (debug)
      {
        Serial.print(F("Delaying..."));
      }
      delay(minTimeBetweenAcc);
      if (debug)
      {
        Serial.println(F("Delay done, look for 2nd motion event"));
      }

      motion2 = checkAccelInRangeForT((maxRawAcc-910), (minRawAcc+550), miniumElevatorAccTime, maxDoorToDoorTime); // must be above or below threshold for 500 ms, timeout=15 seconds @TODO-check
      lastMotion2 = motion2;                                                                           //@TODO-for debug,delete
      if (infoON)
      {
        if (motion2)
        {
          motionEventTwoCount++; //@TODO-for debug,delete
          // fadeLED(motion2);
          Serial.println(F("2nd acceleration or decel sensed."));
          if (debug)
          {
            Serial.print(F("motion2:"));
            Serial.println(motion2);
          }
        }
        else
        {
          Serial.println(F("Timeout. No elevator motion sensed."));
        }
      }
    }
    unsigned long endMot2T = updateAndGetSecsSinceBoot(0);

    if ((motion1 == 2 && motion2 == 1) || (motion1 == 1 && motion2 == 2)) // check to see if these accelerations were in opposite directions
    {
      motionEvent = true;
      if (infoON)
      {
        Serial.println(F("***Full elevator motion detected***"));
      }
    }

    //if (false && motionInteruptTriggered)
    if (debug && motionInteruptTriggered)
    {
      motionEvent = true; // for debug, treat all triggers as motion
      Serial.println(F("DEBUG-***Full elevator motion forced to be detected***"));
    }
    motionInteruptTriggered = false; // reset interupt flag
    // enable interupt on IMU
  } // end if motionInteruptTriggered

  //**********code to handle when there was a motion event
  if (motionEvent)
  {
    // def need to warm up radio from sleep now. "Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode." (http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#a6f4fef2a1f40e704055bff09799f08cf)
    rf95.setModeIdle();
    motionLastDetectedTime = updateAndGetSecsSinceBoot(0);
    motionEvents++;
    if (motionEvents == 65535)
    {                       // 65535 value is reserved for the when the node first comes online, so set to below that before sending
      motionEvents = 65534; // this will tell base station we are in an overflow condition
    }
    //*********the elvator was moving, send the radio packet
    if (sendNodeInfoRadioPacketWithACK(4, motionEvents, motionLastDetectedTime) == 0) // try to send 4 times @TODO-determine best behavior for this
    {
      motionEvents = 0; // reset these if we get an ACK
      motionLastDetectedTime = 0;

      if (debug)
      {
        motionInteruptWakupCount = 0;
        lastMotion1 = 0;
        lastMotion2 = 0;
        motionEventOneCount = 0;
        motionEventTwoCount = 0;
      }
    }
  }

  //*************code to do stuff at certain time intervals***********************
  //@TODO-consider putting battery read and other sensor read code here
  // accelgyro.SetTempSensorEnabled(bool enabled);
  // accelgyro.getTemperature(); //@TODO-useful to send temperature info, could average with ATMEGA328
  //*****check the jumpers to see if mode changed. consumes power to set pullup resistors so limit this.
  static unsigned long prevCheckModeT = updateAndGetSecsSinceBoot(0);
  if ((updateAndGetSecsSinceBoot(0) - prevCheckModeT) > 60)
  {
    if (debug)
    {
      Serial.println(F("Checking jumper mode."));
    }
    checkJumperMode();
    prevCheckModeT = updateAndGetSecsSinceBoot(0); // Reset the timer
  }
  //******* send a radio status message if its been long enough, and there wasn't just a motion event
  static unsigned long prevSendInfoT = updateAndGetSecsSinceBoot(0);
  static unsigned int sendInfoIntervalT = 3600;                                             // send the first info packet after an hour
  if (((updateAndGetSecsSinceBoot(0) - prevSendInfoT) > sendInfoIntervalT) && !motionEvent) // send a radio packet at a defined interval, with some randomness introduced
  {

    unsigned long timeToSend;
    if (motionEvents == 0)
    { // just send the time the node thinks it is now if we have no motion events to send
      timeToSend = updateAndGetSecsSinceBoot(0);
    }
    else
    {
      timeToSend = motionLastDetectedTime;
    }
    if (sendNodeInfoRadioPacketWithACK(2, motionEvents, timeToSend) == 0) // try to send info packet 2 times @TODO-determine better behavior for this
    {
      motionEvents = 0; // reset these if we get an ACK
      motionLastDetectedTime = 0;

      if (debug)
      {
        motionInteruptWakupCount = 0;
        lastMotion1 = 0;
        lastMotion2 = 0;
        motionEventOneCount = 0;
        motionEventTwoCount = 0;
      }
    }

    prevSendInfoT = updateAndGetSecsSinceBoot(0);    // Reset the timer, not precise
    sendInfoIntervalT = 21600 + random(-3600, 3600); // set a new random time to wait. 21600 is 4 times a day (6 hours)
    if (debug)
    {
      Serial.print(F("Sending a info packet. Would be new semi-random interval to wait (but debug mode forces 3600s):"));
      Serial.println(sendInfoIntervalT);
      sendInfoIntervalT = 3600; // force this to send this interval if debug is on
  }
  }

  //************Code before we go to sleep**********
  motionEvent = false;          // reset elevator flag
  updateAndGetSecsSinceBoot(0); // update before we go to sleep
} // end loop
