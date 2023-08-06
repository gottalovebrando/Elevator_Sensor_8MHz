#include "moteino_shared_functions.h" // custom functions for this project
// other libs needed
#include <Arduino.h>
#include <EEPROM.h>

void fadeLED()
{ // fades LED once
  delay(100);
  // NOTE-fade value needs to be int or it will get stuck, depending if fadeSpeed%255==0 or not
  byte fadeSpeed = 40;
  byte delayTime = 30;
  // fade in from min to max
  for (int fadeValue = 0; fadeValue <= 255; fadeValue += fadeSpeed)
  {
    analogWrite(LED_BUILTIN, fadeValue); // sets the value (range from 0 to 255):
    delay(delayTime);                    // wait to see the dimming effect
  }
  // fade out from max to min
  for (int fadeValue = 255; fadeValue >= 0; fadeValue -= fadeSpeed)
  {
    analogWrite(LED_BUILTIN, fadeValue); // sets the value (range from 0 to 255)
    delay(delayTime);                    // wait to see the dimming effect
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void blinkLED(unsigned int number)
{
  if (debug)
  {
    Serial.print(F("blink LED X"));
    Serial.println(number);
  }

  delay(700);
  for (unsigned int i = 0; i < number; i++) // blink
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }
  delay(700);
}

void fadeLED(unsigned int number)
{
  /*
  call this to fade LED a certian number of times
  */

  // pinMode(LED_BUILTIN, OUTPUT);
  if (debug)
  {
    Serial.print(F("Fading LED X"));
    Serial.println(number);
  }
  delay(700);
  for (unsigned int i = 0; i < number; i++)
  {
    fadeLED();
  }
  delay(700);

  /*old code to fade then blink
  //get attention of user by fading first once
  fadeLED();

    if (infoON)
    {
      Serial.print(F("Blinking LED this number of times:"));
      Serial.println(number);
      }
      //actually blink LED
    for (unsigned int i = 0; i < number; i++) // blink
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(300);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
    //end attention of user by fading first once
    fadeLED();
    */
}

void uncorrectableError(unsigned int errorNum)
{
  /*
  call this to halt the program and notify us of some errors we can't recover from
  Error number meanings:
  1-IMU init error
  2-radio init error
  3-calibration error with IMU
  4-invalid EEPROM data for minRawAcc
  5-invalid EEPROM data for maxRawAcc
  6-error sending the initial radio packet
  7-
  */
  //@TODO-add switch to create error message from error number
  // swtich(errorNum)
  // const char *errorMessage

  // force these on for now @TODO-turn off? Battery use doesn't matter much if we hit an error
  infoON = true;
  debug = true;

  pinMode(LED_BUILTIN, OUTPUT);
  while (true) // just sit here until power loss or reset
  {
    if (infoON)
    {
      Serial.print(F("ERROR-uncorrectableError() called. Error number:"));
      Serial.print(errorNum);
      Serial.println('\n');
      // Serial.println(F(". Error message:"));
      // Serial.println(errorMessage);
      delay(1000);
      blinkLED(errorNum);
      delay(1000);
    }
  }
}

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

bool testEEPROMchecksum(int address)
{
  // NOTE- for now it uses a static 4 byte value for checksum, not CRC32
  // reads 4 bytes starting at address, then looks at the next 4 bytes for CRC32 value (@TODO)
  // returns- true if calculated CRC32 matches the data read, false if checksum does not match

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
