#ifndef FASTDACEEPROM_H
#define FASTDACEEPROM_H

#include <Arduino.h>
#include "FastDACdefs.h"
#include "SparkFun_External_EEPROM.h"

#define EEPROM_I2C_ADDRESS 0x51 //i2c address of 24LC256 EEPROM on Fastdac
#define EEPROM_I2C_CLOCK 400000L //400kHz i2c clock
#define EEPROM_SIZE 256 //Size of EEPROM in kbits 256kbits = 32768 bytes

#define EEPROM_WP_PIN 13 //Pin 13 has to be jumpered to GND to allow writing ID or factory cals

#define EEPROM_ID_ADDR 0

#define EEPROM_ID_LEN 128

#define EEPROM_DAC_CAL_ADDR EEPROM_ID_ADDR + EEPROM_ID_LEN

//Each DAC channel has 1 byte for offset cal and 1 byte for zero cal
#define EEPROM_DAC_CAL_LEN NUMDACCHANNELS * 2

#define EEPROM_DAC_FACT_CAL_ADDR EEPROM_DAC_CAL_ADDR + EEPROM_DAC_CAL_LEN

#define EEPROM_ADC_CAL_ADDR EEPROM_DAC_FACT_CAL_ADDR + EEPROM_DAC_CAL_LEN

//Each ADC channel has 3 bytes for zero scale, and 3 bytes for full scale, per 'Filter Word', Filter word can be 0 to 127 (128)
#define EEPROM_ADC_NUM_FWS 128

#define EEPROM_ADC_CAL_LEN NUMADCCHANNELS * 6 * EEPROM_ADC_NUM_FWS

#define EEPROM_ADC_FACT_CAL_ADDR EEPROM_ADC_CAL_ADDR + EEPROM_ADC_CAL_LEN



void eepromtest(void);
uint8_t initeeprom(void);
uint8_t writeeepromid(char * idstring);
void readeepromid(char * idstring);
uint8_t readeepromdaccal(uint8_t ch, int8_t * offset, int8_t * gain, bool factory);
uint8_t writeeepromdaccal(uint8_t ch, int8_t offset, int8_t gain, bool factory);
uint8_t readeepromadccal(uint8_t ch, uint8_t fw, uint32_t * zeroscale, uint32_t * fullscale, bool factory);
uint8_t writeeepromadccal(uint8_t ch, uint8_t fw, uint32_t zeroscale, uint32_t fullscale, bool factory);

#endif