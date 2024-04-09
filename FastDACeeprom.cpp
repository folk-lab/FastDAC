#include "FastDACeeprom.h"

#define IDTEST "Unit 2"

ExternalEEPROM extprom;

void eepromtest(void)
{
    SERIALPORT.println("Init external eeprom");
    initeeprom();
    SERIALPORT.println("Write ID");
    writeeepromid(IDTEST);
    char idtest[EEPROM_ID_LEN];
    SERIALPORT.println("Read ID");
    readeepromid(idtest);
    SERIALPORT.println(idtest);
}

uint8_t initeeprom(void)
{
  Wire.begin();//start i2c peripheral
  Wire.setClock(EEPROM_I2C_CLOCK);//setclock

  extprom.setMemoryType(EEPROM_SIZE);

  if (extprom.begin(EEPROM_I2C_ADDRESS) == false)
  {
    //SERIALPORT.println("No EEPROM detected!");
    return -1;
  }
  //SERIALPORT.println("Memory detected!");

  //SERIALPORT.print("Mem size in bytes: ");
  //SERIALPORT.println(extprom.length());
  return 0;
}

//Returns 1 if len too long, 0 if successful, others if there was an i2c problem or nak from EEPROM
uint8_t writeeepromid(char * idstring)
{
    
    uint32_t idlen = strlen(idstring);
    //SERIALPORT.print("ID string size: ");
    //SERIALPORT.println(idlen);
    if(idlen >= EEPROM_ID_LEN)
    {
        return -1;
    }

    uint8_t result;
    result = extprom.write(EEPROM_ID_ADDR, (uint8_t *)idstring, idlen + 1);
    //SERIALPORT.print("Result: ");
    //SERIALPORT.println(result);
    return result;
}

void readeepromid(char * idstring)
{
    extprom.read(EEPROM_ID_ADDR, (uint8_t *)idstring, EEPROM_ID_LEN);
}
