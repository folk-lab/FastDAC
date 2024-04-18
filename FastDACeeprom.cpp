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
  pinMode(EEPROM_WP_PIN, INPUT_PULLUP);
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
        return 1;
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

uint8_t readeepromdaccal(uint8_t ch, int8_t * offset, int8_t * gain, bool factory)
{
  if(ch >= NUMDACCHANNELS)
  {
    return 1;
  }
  if(factory)
  {
    *offset = extprom.read((ch * 2) + EEPROM_DAC_FACT_CAL_ADDR);
    *gain = extprom.read((ch * 2) + EEPROM_DAC_FACT_CAL_ADDR + 1);  
  }
  else
  {
    *offset = extprom.read((ch * 2) + EEPROM_DAC_CAL_ADDR);
    *gain = extprom.read((ch * 2) + EEPROM_DAC_CAL_ADDR + 1);  
  }
  
  return 0;
}

uint8_t writeeepromdaccal(uint8_t ch, int8_t offset, int8_t gain, bool factory)
{
  if(ch >= NUMDACCHANNELS)
  {
    return 1;
  }
  if(factory)
  {
    extprom.write((ch * 2) + EEPROM_DAC_FACT_CAL_ADDR, offset);
    extprom.write((ch * 2) + EEPROM_DAC_FACT_CAL_ADDR + 1, gain);  
  }
  else
  {
    extprom.write((ch * 2) + EEPROM_DAC_CAL_ADDR, offset);
    extprom.write((ch * 2) + EEPROM_DAC_CAL_ADDR + 1, gain);  
  }
  
  return 0;
}

uint8_t readeepromadccal(uint8_t ch, uint8_t fw, uint32_t * zeroscale, uint32_t * fullscale, bool factory)
{

  
  return 0;
}

uint8_t writeeepromadccal(uint8_t ch, uint8_t fw, uint32_t zeroscale, uint32_t fullscale, bool factory)
{

  
  return 0;
}