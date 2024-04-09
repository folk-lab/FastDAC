#ifndef FASTDACEEPROM_H
#define FASTDACEEPROM_H

#include <Arduino.h>
#include "FastDACdefs.h"
#include "SparkFun_External_EEPROM.h"

#define EEPROM_I2C_ADDRESS 0x51 //i2c address of 24LC256 EEPROM on Fastdac
#define EEPROM_I2C_CLOCK 400000L //400kHz i2c clock
#define EEPROM_SIZE 256 //Size of EEPROM in kbits 256kbits = 32768 bytes

#define EEPROM_ID_ADDR 0

#define EEPROM_ID_LEN 128

void eepromtest(void);

uint8_t initeeprom(void);

uint8_t writeeepromid(char * idstring);

void readeepromid(char * idstring);

#endif