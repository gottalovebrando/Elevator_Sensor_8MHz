//BEGIN header guard
#ifndef moteino_shared_functions_h
#define moteino_shared_functions_h
//END header guard

//variables needed for these functions
extern bool debug;
extern bool infoON;
//NOTE- for non-primitaves, might need to use dereference operatior like:
//extern RH_RF95 *rf95;

//function signatures
void fadeLED();
void fadeLED(unsigned int number);
void blinkLED(unsigned int number);
void writeDataWithChecksumToEEPROM(int address, unsigned long data);
bool testEEPROMchecksum(int address);
void uncorrectableError(unsigned int errorNum);

//BEGIN header guard
#endif
//END header guard