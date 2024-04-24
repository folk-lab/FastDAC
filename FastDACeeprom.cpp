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
  uint32_t addroffset;
  if(factory)
  {
    addroffset = EEPROM_DAC_FACT_CAL_ADDR;
  }
  else
  {
    addroffset = EEPROM_DAC_CAL_ADDR;
  }
  *offset = extprom.read(addroffset + (ch * 2));
  *gain = extprom.read(addroffset + (ch * 2) + 1);  
  
  return 0;
}

uint8_t writeeepromdaccal(uint8_t ch, int8_t offset, int8_t gain, bool factory)
{
  if(ch >= NUMDACCHANNELS)
  {
    return 1;
  }
  uint32_t addroffset;
  if(factory)
  {
    addroffset = EEPROM_DAC_FACT_CAL_ADDR;
  }
  else
  {
    addroffset = EEPROM_DAC_CAL_ADDR;
  }

  extprom.write(addroffset + (ch * 2), offset);
  extprom.write(addroffset + (ch * 2) + 1, gain);  
  
  return 0;
}

uint8_t readeepromadccal(uint8_t ch, uint8_t fw, uint32_t * zeroscale, uint32_t * fullscale, bool factory)
{
  if(ch >= NUMADCCHANNELS)
  {
    return 1;
  }
  if(fw >= EEPROM_ADC_NUM_FWS)
  {
    SERIALPORT.println("ADDRESS ERROR!");
    return 1;
  }
  uint32_t offset;
  if(factory)
  {
    offset = EEPROM_ADC_FACT_CAL_ADDR;
  }
  else
  {
    offset = EEPROM_ADC_CAL_ADDR;
  }
  uint8_t zerobyte, fullbyte, i;
  uint32_t zerotemp = 0;
  uint32_t fulltemp = 0;
  for(i = 0; i < EEPROM_ADC_CH_VAR_SIZE; i++)
  {
    zerobyte = extprom.read(offset + (ch * EEPROM_ADC_CH_LEN) + (fw * EEPROM_ADC_CH_VAR_SIZE * 2) + i);
    fullbyte = extprom.read(offset + (ch * EEPROM_ADC_CH_LEN) + (fw * EEPROM_ADC_CH_VAR_SIZE * 2) +  EEPROM_ADC_CH_VAR_SIZE + i);
    zerotemp |= zerobyte << (i * 8);
    fulltemp |= fullbyte << (i * 8);
#ifdef DEBUGEEPROM
    SERIALPORT.println(zerobyte);
    SERIALPORT.println(zerotemp);
    SERIALPORT.println(fullbyte);
    SERIALPORT.println(fulltemp);
#endif
  }

  *zeroscale = zerotemp;
  *fullscale = fulltemp;
     
  return 0;
}

uint8_t writeeepromadccal(uint8_t ch, uint8_t fw, uint32_t zeroscale, uint32_t fullscale, bool factory)
{
  if(ch >= NUMADCCHANNELS)
  {
    return 1;
  }
  if(fw >= EEPROM_ADC_NUM_FWS)
  {
    SERIALPORT.println("ADDRESS ERROR!");
    return 1;
  }
  uint32_t offset;
  if(factory)
  {
    offset = EEPROM_ADC_FACT_CAL_ADDR;
  }
  else
  {
    offset = EEPROM_ADC_CAL_ADDR;
  }
  uint8_t zerobyte, fullbyte, i;

  for(i = 0; i < EEPROM_ADC_CH_VAR_SIZE; i++)
  {
    zerobyte = zeroscale & 0xFF;
    fullbyte = fullscale & 0xFF;
    extprom.write((offset + (ch * EEPROM_ADC_CH_LEN) + (fw * EEPROM_ADC_CH_VAR_SIZE * 2) + i), zerobyte);
    extprom.write((offset + (ch * EEPROM_ADC_CH_LEN) + (fw * EEPROM_ADC_CH_VAR_SIZE * 2) +  EEPROM_ADC_CH_VAR_SIZE + i), fullbyte);
#ifdef DEBUGEEPROM
    SERIALPORT.println(zerobyte);
    SERIALPORT.println(zeroscale);
    SERIALPORT.println(fullbyte);
    SERIALPORT.println(fullscale);
#endif
    zeroscale = zeroscale >> 8;
    fullscale = fullscale >> 8;

  }

  return 0;
}