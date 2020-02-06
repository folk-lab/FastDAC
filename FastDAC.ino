//Arduino *DUE*code for controlling EVAL-AD7734 ADC and EVAL-AD5764 DAC
//Andrea Young
//Carlos Kometter
//Modified for 8 DAC channels and consistent high speed ADC sampling by UBC PHAS E-Lab, Nov 2019
#define NUMDACCHANNELS 8
#define NUMADCCHANNELS 4

#include "SPI.h" // necessary library for SPI communication

#include <vector>
#include "FastDACdefs.h"
#include "FastDACcalibration.h" //This cal file should be copied and renamed for each DAQ unit, maybe store in EEPROM in the future

#define USBBUFFSIZE 300//most efficient for USB bandwidth is 1kB(1024) but can cause long delays between sends when using longer sample times

#define DACSETTLETIME  200//milliseconds to wait before starting ramp

//#define DEBUGRAMP //Uncomment this to enable sending of ramp debug info

#define BIT31 0x10000000 //Some scaling constants for fixed-point math
#define BIT47 0x100000000000


int adc=52; //The SPI pin for the ADC
int dac0 = 4; //The SPI pin for the DAC0
int dac1 = 10; //The SPI pin for the DAC1
int ldac0=6; //Load DAC1 pin for DAC1. Make it LOW if not in use.
int ldac1=9; //Load DAC2 pin for DAC1. Make it LOW if not in use.
Pio *ldac_port = digitalPinToPort(ldac0); //ldac0 and ldac1 share the same port, so they can be toggled simultaneously
uint32_t ldac0_mask = digitalPinToBitMask(ldac0);
uint32_t ldac1_mask = digitalPinToBitMask(ldac1);

int reset=44 ; //Reset on ADC
int drdy=48; // Data is ready pin on ADC
int led = 32;
int data=28;//Used for trouble shooting; connect an LED between pin 28 and GND
int err=30;
const int Noperations = 25;
String operations[Noperations] = {"NOP", "INT_RAMP", "SET", "GET_DAC", "GET_ADC", "RAMP_SMART", "RAMP1", "RAMP2", "BUFFER_RAMP", "CAL_ADC_WITH_DAC", "RESET", "TALK", "CONVERT_TIME", "*IDN?", "*RDY?", "GET_DUNIT","SET_DUNIT", "ADC_ZERO_SC_CAL", "ADC_CH_ZERO_SC_CAL", "ADC_CH_FULL_SC_CAL", "DAC_RESET_CAL", "FULL_SCALE", "DAC_OFFSET_ADJ", "DAC_GAIN_ADJ", "WRITE_ADC_CAL", "READ_ADC_CAL"};
int delayUnit=0; // 0=microseconds 1=miliseconds

float DAC_FULL_SCALE = 10.0;

volatile int16_t g_DACsetpoint[NUMDACCHANNELS];//global array for current DAC setpoints, only written to in DACintegersend()

volatile byte g_USBbuff[USBBUFFSIZE + 100]; //add some bytes to the buffer to prevent overflow in interrupt

//Ramp interrupt global variables
volatile uint32_t g_buffindex = 0;
volatile bool g_done = false;
volatile uint32_t g_samplecount = 0;
volatile uint8_t g_numrampADCchannels;
volatile uint8_t g_ADCchanselect[NUMADCCHANNELS];
volatile uint8_t g_numrampDACchannels;
volatile uint8_t g_DACchanselect[NUMDACCHANNELS];
volatile int64_t g_DACramppoint[NUMDACCHANNELS];
volatile int32_t g_DACstartpoint[NUMDACCHANNELS];
volatile int32_t g_DACendpoint[NUMDACCHANNELS];
volatile int64_t g_DACstep[NUMDACCHANNELS];
volatile uint32_t g_numsteps;

// Calibration constants
//float OS[4]={0,0,0,0}; // offset error
//float GE[4]={1,1,1,1}; // gain error

std::vector<String> query_serial()
{
  char received;
  String inByte = "";
  std::vector<String> comm;
  while (received != '\r')
  {
    if(SerialUSB.available())
    {
      received = SerialUSB.read();
      if (received == '\n' || received == ' ')
      {}
      else if (received == ',' || received == '\r')
      {
        comm.push_back(inByte);
        inByte = "";
      }
      else
      {
        inByte += received;
      }
    }
  }
  return comm;
}

namespace std {
  void __throw_bad_alloc()
  {
    SerialUSB.println("Unable to allocate memory");
  }

  void __throw_length_error( char const*e )
  {
    SerialUSB.print("Length Error :");
    SerialUSB.println(e);
  }
}

void setup()
{
  //SerialUSB.begin(115200);

  SerialUSB.begin(2000000);

  pinMode(ldac0,OUTPUT);
  digitalWrite(ldac0,HIGH); //Load DAC pin for DAC0. Make it LOW if not in use.

  pinMode(ldac1,OUTPUT);
  digitalWrite(ldac1,HIGH); //Load DAC pin for DAC1. Make it LOW if not in use.

  //pinMode(spi,OUTPUT);
  pinMode(reset, OUTPUT);
  pinMode(drdy, INPUT);  //Data ready pin for the ADC.
  pinMode(led, OUTPUT);  //Used for blinking indicator LED
  digitalWrite(led, HIGH);
  pinMode(data, OUTPUT);

  digitalWrite(reset,HIGH);  digitalWrite(data,LOW); digitalWrite(reset,LOW);  digitalWrite(data,HIGH); delay(5);  digitalWrite(reset,HIGH);  digitalWrite(data,LOW);//Resets ADC on startup.

  SPI.begin(adc); // wake up the SPI bus for ADC
  SPI.begin(dac0); // wake up the SPI bus for DAC0
  SPI.begin(dac1); // wake up the SPI bus for DAC1

  SPI.setBitOrder(adc,MSBFIRST); //correct order for AD7734.
  SPI.setBitOrder(dac0,MSBFIRST); //correct order for AD5764.
  SPI.setBitOrder(dac1,MSBFIRST); //correct order for AD5764.

  SPI.setClockDivider(adc,7); // Maximum 12 Mhz for AD7734
  SPI.setClockDivider(dac0,4); // Maximum 21 Mhz for AD5764
  SPI.setClockDivider(dac1,4); // Maximum 21 Mhz for AD5764

  SPI.setDataMode(adc,SPI_MODE3); //This should be 3 for the AD7734
  SPI.setDataMode(dac0,SPI_MODE1); //This should be 1 for the AD5764
  SPI.setDataMode(dac1,SPI_MODE1); //This should be 1 for the AD5764

  loaddefaultcals();
  //Initialize saved DAC setpoints to 0
  for(int i = 0; i < NUMDACCHANNELS; i++)
  {
    g_DACsetpoint[i] = 0;
  }
// Disables DAC_SDO to avoid interference with ADC
//  SPI.transfer(dac,1,SPI_CONTINUE);
//  SPI.transfer(dac,0,SPI_CONTINUE);
//  SPI.transfer(dac,1);
}

void blinker(int s){digitalWrite(data,HIGH);delay(s);digitalWrite(data,LOW);delay(s);}
void sos(){blinker(50);blinker(50);blinker(50);blinker(500);blinker(500);blinker(500);blinker(50);blinker(50);blinker(50);}

void error()
{
  digitalWrite(err,HIGH);
  delay(3000);
  digitalWrite(err,LOW);
  delay(500);
}


int indexOfOperation(String operation)
{
  for(int index = 0; index < Noperations; index++)
  {
    if(operations[index] == operation)
    {
      return index;
    }
  }
  return 0;
}

void waitDRDY() {while (digitalRead(drdy)==HIGH){}}

void resetADC() //Resets the ADC, and sets the range to default +-10 V
{
  byte ch = 0;

  digitalWrite(data,HIGH);digitalWrite(reset,HIGH);digitalWrite(reset,LOW);digitalWrite(reset,HIGH);
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    SPI.transfer(adc, ADC_CHSETUP | ch);//Access channel setup register for each channel
    SPI.transfer(adc, ADC_CHSETUP_RNG10BI);//set +/-10V range
  }

}

void talkADC(std::vector<String> DB)
{
  int comm;
  comm=SPI.transfer(adc,DB[1].toInt());
  SerialUSB.println(comm);
  SerialUSB.flush();
}

void writeADCConversionTime(std::vector<String> DB)
{
  if(DB.size() != 3)
  {
    SerialUSB.println("SYNTAX ERROR");
    return;
  }
  int adcChannel;
  switch(DB[1].toInt()){
    case 0:
    adcChannel = 0;
    break;
    case 1:
    adcChannel = 1;
    break;
    case 2:
    adcChannel = 2;
    break;
    case 3:
    adcChannel = 3;
    break;

    default:
    break;
  }
  byte cr;
  float reqtime = DB[2].toFloat();
  if(reqtime > 2686)
  {
    reqtime = 2686;
  }
  byte fw = ((byte)(((reqtime*6.144-249)/128)+0.5));
  if(fw < 2)//cannot have less than 2 or ADC will not sample
  {
    fw = 2;
  }
  fw |= 0x80; //enable chopping

  //fw = 3; //max speed test

  SPI.transfer(adc, ADC_CHCONVTIME | adcChannel); //Write conversion time register
  SPI.transfer(adc, fw); //Write 'filter word' (conversion time)
  delayMicroseconds(100);
  SPI.transfer(adc, ADC_REGREAD | ADC_CHCONVTIME | adcChannel); //Read conversion time register
  cr=SPI.transfer(adc,0); //Read back the CT register
  //SerialUSB.println(fw);
  int convtime = ((int)(((cr&127)*128+249)/6.144)+0.5);
  SerialUSB.println(convtime);
}

float map2(float x, long in_min, long in_max, float out_min, float out_max) //float
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int twoByteToInt(byte DB1,byte DB2) // This gives a 16 bit integer (between +/- 2^16)
{
  return ((int)((DB1<<8)| DB2));
}


void intToTwoByte(int s, byte * DB1, byte * DB2)
{
    *DB1 = ((byte)((s>>8)&0xFF));
    *DB2 = ((byte)(s&0xFF));
}


float int16ToVoltage(int16_t data)
{

  float voltage;

  if (data >= 0)
  {
    voltage = data*DAC_FULL_SCALE/32767;
  }
  else
  {
    voltage = data*DAC_FULL_SCALE/32768;
  }
  return voltage;
}

int16_t voltageToInt16(float voltage)
{
  int decimal;
  if (voltage > DAC_FULL_SCALE || voltage < -1*DAC_FULL_SCALE)
  {
    error();
    return 0;
  }
  else if (voltage >= 0)
  {
    return voltage*32767/DAC_FULL_SCALE;
  }
  else
  {
    return voltage*32768/DAC_FULL_SCALE;
  }
}

int32_t voltageToInt32(float voltage)
{
  int32_t calcint;
  if (voltage > DAC_FULL_SCALE || voltage < -1*DAC_FULL_SCALE)
  {
    calcint = 0;
    error();
  }
  else if(voltage >=0)
  {
    calcint = (int32_t)((voltage/DAC_FULL_SCALE) * 0x7FFFFFFF);
  }
  else
  {
    calcint = (int32_t)((voltage/DAC_FULL_SCALE) * 0x80000000);
  }
  return calcint;
}

float getDAC(int ch)
{
  return int16ToVoltage(g_DACsetpoint[ch]);
}

float getSingleReading(int adcchan)
{
  SerialUSB.flush();
  int statusbyte=0;
  byte o2;
  byte o3;
  int ovr;
  if(adcchan <= 3)
  {
    SPI.transfer(adc, ADC_CHMODE | adcchan);   // Write channel mode register
    SPI.transfer(adc, ADC_MODE_SINGLECONV | ADC_MODE_DUMP); // Single conversion + dump mode
    waitDRDY();                       // Waits until convertion finishes
    SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | adcchan, SPI_CONTINUE);   // Read channel data register
    statusbyte=SPI.transfer(adc, 0, SPI_CONTINUE);   // Reads Channel 'ch' status
    o2=SPI.transfer(adc, 0, SPI_CONTINUE);           // Reads first byte
    o3=SPI.transfer(adc, 0, SPI_CONTINUE);           // Reads second byte
    ovr=statusbyte & ADC_CHSTAT_OVR;
    switch (ovr)
    {
      case 0:
      int decimal;
      decimal = twoByteToInt(o2,o3);
      //SerialUSB.println(decimal);
      float voltage;
      voltage = map2(decimal, 0, 65536, -10.0, 10.0);
      return voltage;
      break;

      case 1:
      return 0.0;
      break;
    }
  }
}

float readADC(byte DB)
{
  int adcChannel=DB;
  switch (adcChannel)
  {
    case 0:
    return getSingleReading(0);
    break;
    case 1:
    return getSingleReading(1);
    break;
    case 2:
    return getSingleReading(2);
    break;
    case 3:
    return getSingleReading(3);
    break;

    default:
    break;
  }
}

float dacDataSend(int ch, float voltage)
{
  digitalWrite(data, HIGH);
  DACintegersend(ch, voltageToInt16(voltage));
  return getDAC(ch);
}

float writeDAC(int dacChannel, float voltage, bool load)
{
  float actualvoltage;
  if(dacChannel >= NUMDACCHANNELS)
  {
    SerialUSB.print("DAC channel address ");
    SerialUSB.print(dacChannel);
    SerialUSB.println(" doesn't exist, write failed");
    return 99.999;
  }
  digitalWrite(data, HIGH);
  actualvoltage = dacDataSend(dacChannel, voltage);
  if(load)//load both DACs if necessary
  {
    digitalWrite(ldac0, LOW);
    digitalWrite(ldac1, LOW);
    digitalWrite(ldac0, HIGH);
    digitalWrite(ldac1, HIGH);
  }
  digitalWrite(data, LOW);
  return actualvoltage;
}

void readingRampAvg(int adcchan, byte b1, byte b2, byte * o1, byte * o2,int count,int nReadings)
{
  SerialUSB.flush();
  int statusbyte=0;
  int ovr;
  byte db1;
  byte db2;
  float sum=0;
  float avg;
  bool toSend = true;
  if(adcchan <= 3)
  {
    for(int i = 1; i<=nReadings; i++)
    {
      SPI.transfer(adc, ADC_CHMODE | adcchan, SPI_CONTINUE);   // Write channel mode register
      SPI.transfer(adc, ADC_MODE_SINGLECONV | ADC_MODE_DUMP); // Single conversion + dump mode
      if (count>0 && toSend)
      {
        SerialUSB.write(b1);                 // Sends previous reading while it is waiting for new reading
        SerialUSB.write(b2);
        toSend = false;
      }
      waitDRDY();                       // Waits until convertion finishes
      SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | adcchan, SPI_CONTINUE);   // Read channel data register
      statusbyte=SPI.transfer(adc,0, SPI_CONTINUE);   // Reads Channel 'ch' status
      db1=SPI.transfer(adc,0, SPI_CONTINUE); // Reads first byte
      db2=SPI.transfer(adc,0);           // Reads second byte
      ovr=statusbyte&1;
      if (ovr){break;}
      int decimal = twoByteToInt(db1,db2);
      float voltage = map2(decimal, 0, 65536, -10.0, 10.0);
      sum += voltage;
    }
    if(ovr)
    {
      *o1=128;
      *o2=0;
    }
    else
    {
      avg = sum/nReadings;
      int decimal = map2(avg, -10.0, 10.0, 0, 65536);
      intToTwoByte(decimal, &db1, &db2);
      *o1=db1;
      *o2=db2;
    }
  }
}

void rampRead(byte DB, byte b1, byte b2, byte * o1, byte * o2, int count, int nReadings )
{
  int adcChannel=DB;
  switch (adcChannel)
  {
    case 0:
    readingRampAvg(0, b1 , b2, o1, o2,count,nReadings);
    break;

    case 1:
    readingRampAvg(1, b1 , b2, o1, o2,count,nReadings);
    break;

    case 2:
    readingRampAvg(2, b1 , b2, o1, o2,count,nReadings);
    break;

    case 3:
    readingRampAvg(3, b1 , b2, o1, o2,count,nReadings);
    break;

    default:
    break;
  }
}

void bufferRamp(std::vector<String> DB)
{

  String channelsDAC = DB[1];
  int NchannelsDAC = channelsDAC.length();
  String channelsADC = DB[2];
  int NchannelsADC = channelsADC.length();
  std::vector<float> vi;
  std::vector<float> vf;
  float v_min = -1*DAC_FULL_SCALE;
  float v_max = DAC_FULL_SCALE;
  for(int i = 3; i < NchannelsDAC+3; i++)
  {
    vi.push_back(DB[i].toFloat());
    vf.push_back(DB[i+NchannelsDAC].toFloat());
  }
  int nSteps=(DB[NchannelsDAC*2+3].toInt());
  byte b1;
  byte b2;
  int count =0;
  for (int j=0; j<nSteps;j++)
  {
    digitalWrite(data,HIGH);
    for(int i = 0; i < NchannelsDAC; i++)
    {
      float v;
      v = vi[i]+(vf[i]-vi[i])*j/(nSteps-1);
      if(v<v_min)
      {
        v=v_min;
      }
      else if(v>v_max)
      {
        v=v_max;
      }
      writeDAC(channelsDAC[i]-'0', v, false);
    }
    digitalWrite(ldac0,LOW);
    digitalWrite(ldac1,LOW);
    digitalWrite(ldac0,HIGH);
    digitalWrite(ldac1,HIGH);
    if (delayUnit)
    {
      delay(DB[NchannelsDAC*2+4].toInt());
    }
    else
    {
      delayMicroseconds(DB[NchannelsDAC*2+4].toInt());
    }
    for(int i = 0; i < NchannelsADC; i++)
    {
      rampRead(channelsADC[i]-'0', b1, b2, &b1, &b2, count,DB[NchannelsDAC*2+5].toInt());
      count+=1;
    }
    if(SerialUSB.available())
    {
      std::vector<String> comm;
      comm = query_serial();
      if(comm[0] == "STOP")
      {
        break;
      }
    }
  }
  SerialUSB.write(b1);
  SerialUSB.write(b2);
  digitalWrite(data,LOW);
}

void autoRamp1(std::vector<String> DB)
{
  if(DB.size() != 6)
  {
    SerialUSB.println("SYNTAX ERROR");
    return;
  }
  float v1 = DB[2].toFloat();
  float v2 = DB[3].toFloat();
  int nSteps = DB[4].toInt();
  int dacChannel=DB[1].toInt();

  for (int j=0; j<nSteps;j++)
  {
    int timer = micros();
    digitalWrite(data,HIGH);
    writeDAC(dacChannel, v1+(v2-v1)*j/(nSteps-1), true);
    digitalWrite(data,LOW);
    while(micros() <= timer + DB[5].toInt());
  }
}

void autoRamp2(std::vector<String> DB)
{
  if(DB.size() != 9)
  {
    SerialUSB.println("SYNTAX ERROR");
    return;
  }
  float vi1 = DB[3].toFloat();
  float vi2 = DB[4].toFloat();
  float vf1 = DB[5].toFloat();
  float vf2 = DB[6].toFloat();
  int nSteps = DB[7].toInt();
  byte b1;
  byte b2;
  int dacChannel1=DB[1].toInt();
  int dacChannel2=DB[2].toInt();

  for (int j=0; j<nSteps;j++)
  {
    int timer = micros();
    digitalWrite(data,HIGH);
    writeDAC(dacChannel1, vi1+(vf1-vi1)*j/(nSteps-1), true);
    writeDAC(dacChannel2, vi2+(vf2-vi2)*j/(nSteps-1), true);
    while(micros() <= timer + DB[8].toInt());
    digitalWrite(data,LOW);
  }
}

void ID()
{
  SerialUSB.println(IDSTRING);
}

void RDY()
{
  SerialUSB.println("READY");
}

void setUnit(int unit)
{
  if (unit == 0)
  {
    delayUnit = 0;
    SerialUSB.println("Delay unit set to microseconds");
  }
  else if (unit == 1)
  {
    delayUnit = 1;
    SerialUSB.println("Delay unit set to miliseconds");
  }
  else
  {
    SerialUSB.println("Unit should be 0 (microseconds) or 1 (miliseconds)");
  }
}

void adc_zero_scale_cal(int ch)
{
  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE);       // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SELFZEROCAL);       // Enter system zero-scale cal mode
  waitDRDY();
}

void debug()
{
  digitalWrite(data,HIGH);
  delay(3000);
  digitalWrite(data,LOW);
  delay(3000);
}

void ramp_smart(std::vector<String> DB)  //(channel,setpoint,ramprate)
{
  if(DB.size() != 4)
  {
    SerialUSB.println("SYNTAX ERROR");
    return;
  }
  float channel = DB[1].toInt();
  float setpoint = DB[2].toFloat();
  float ramprate = DB[3].toFloat();

  float initial = getDAC(channel);
  if (abs(setpoint-initial) < 0.05)  //If already at setpoint
  {
    return;
  }
  // TODO: calc ramprate, make vector string, pass to autoRamp1
  int nSteps = static_cast<int>(abs(setpoint-initial)/ramprate*1000);  //using 1ms as delay

  vector<string> autoRampInput;
  autoRampInput.push_back("RAMP1");
  autoRampInput.push_back(to_string(channel));
  autoRampInput.push_back(to_string(initial));
  autoRampInput.push_back(to_string(setpoint));
  autoRampInput.push_back(to_string(nSteps));
  autoRampInput.push_back(to_string("1000")); //1000us delay between steps
  autoRamp1(autoRampInput)
}

void router(std::vector<String> DB)
{
  float v;
  int operation = indexOfOperation(DB[0]);
  switch ( operation )
  {
    case 0: //NOP
    SerialUSB.println("NOP");
    break;

    /*
    case 1:
    SerialUSB.println("NOP");
    break;
*/
    case 1: //INT_RAMP
    intRamp(DB);
    SerialUSB.println("RAMP_FINISHED");
    break;

    case 2: //SET
    if(DB[2].toFloat() < -1*DAC_FULL_SCALE || DB[2].toFloat() > DAC_FULL_SCALE)
    {
      SerialUSB.println("VOLTAGE_OVERRANGE");
      break;
    }
    v = writeDAC(DB[1].toInt(),DB[2].toFloat(), true);
    SerialUSB.print("DAC ");
    SerialUSB.print(DB[1]);
    SerialUSB.print(" UPDATED TO ");
    SerialUSB.print(v,4);
    SerialUSB.println("V");
    break;

    case 3: //GET_DAC
    SerialUSB.println(getDAC(DB[1].toInt()), 4);
    break;

    case 4: //GET_ADC
    v=readADC(DB[1].toInt());
    SerialUSB.println(v,4);
    break;

    case 5: //RAMP_SMART
    ramp_smart(DB);
    SerialUSB.println("RAMP_FINISHED");
    break;

    case 6: //RAMP1
    autoRamp1(DB);
    SerialUSB.println("RAMP_FINISHED");
    break;

    case 7: //RAMP2
    autoRamp2(DB);
    SerialUSB.println("RAMP_FINISHED");
    break;

    case 8: //BUFFER_RAMP
    bufferRamp(DB);
    SerialUSB.println("BUFFER_RAMP_FINISHED");
    break;

    case 9: //CAL_ADC_WITH_DAC
    calADCwithDAC();
    SerialUSB.println("CALIBRATION_FINISHED");
    break;

    case 10: //RESET
    resetADC();
    break;

    case 11: //TALK
    talkADC(DB);
    break;

    case 12: //CONVERT_TIME
    writeADCConversionTime(DB);
    break;

    case 13: //*IDN?
    ID();
    break;

    case 14: //*RDY?
    RDY();
    break;

    case 15: //GET_DUNIT
    SerialUSB.println(delayUnit);
    break;

    case 16: //SET_DUNIT
    setUnit(DB[1].toInt()); // 0 = microseconds 1 = miliseconds
    break;

    case 17: //ADC_ZERO_SC_CAL
    adc_zero_scale_cal(DB[1].toInt());
    SerialUSB.println("CALIBRATION_FINISHED");
    break;

    case 18: //ADC_CH_ZERO_SC_CAL
    adc_ch_zero_scale_cal(DB[1].toInt());
    SerialUSB.println("CALIBRATION_FINISHED");
    break;

    case 19: //ADC_CH_FULL_SC_CAL
    adc_ch_full_scale_cal(DB[1].toInt());
    SerialUSB.println("CALIBRATION_FINISHED");
    break;

    case 20: //DAC_CH_CAL
    dac_ch_reset_cal(DB[1].toInt());
    SerialUSB.println("CALIBRATION_RESET");
    break;

    case 21: //FULL_SCALE
    DAC_FULL_SCALE = DB[1].toFloat();
    SerialUSB.println("FULL_SCALE_UPDATED");
    break;

    case 22: //DAC_OFFSET_ADJ
    calDACoffset(DB[1].toInt(), DB[2].toFloat());
    SerialUSB.println("CALIBRATION_FINISHED");
    break;

    case 23: //DAC_GAIN_ADJ
    calDACgain(DB[1].toInt(), DB[2].toFloat());
    SerialUSB.println("CALIBRATION_FINISHED");
    break;

    case 24: //WRITE_ADC_CAL
    writeADCcal(DB[1].toInt(), DB[2].toInt(), DB[3].toInt());
    SerialUSB.println("CALIBRATION_CHANGED");
    break;

    case 25: //READ_ADC_CAL
    readADCzerocal(DB[1].toInt());
    readADCfullcal(DB[1].toInt());
    SerialUSB.println("READ_FINISHED");
    break;


    default:
    break;
  }
}

void loop()
{
  SerialUSB.flush();
  std::vector<String> comm;

  if(SerialUSB.available())
  {
    comm = query_serial();
    router(comm);
  }
}


void DACintegersend(byte ch, int16_t value)
{
  if(ch <= 3)
  {
    g_DACsetpoint[ch] = value;
    SPI.transfer(dac0, DAC_DATA | ch, SPI_CONTINUE); // Indicates to DAC to write channel 'ch' in the data register
    SPI.transfer(dac0, (uint8_t)(value >> 8), SPI_CONTINUE);   // writes first byte
    SPI.transfer(dac0, (uint8_t)(value & 0xff));                // writes second byte
  }
  else if(ch < NUMDACCHANNELS)
  {
    g_DACsetpoint[ch] = value;
    ch -= 4;
    SPI.transfer(dac1, DAC_DATA | ch, SPI_CONTINUE); // Indicates to DAC to write channel 'ch' in the data register
    SPI.transfer(dac1, (uint8_t)(value >> 8), SPI_CONTINUE);   // writes first byte
    SPI.transfer(dac1, (uint8_t)(value & 0xff));                // writes second byte
  }
}

void intRamp(std::vector<String> DB)
{
  int i, j;
  bool error = false;
  String channelsDAC = DB[1];
  g_numrampDACchannels = channelsDAC.length();
  String channelsADC = DB[2];
  g_numrampADCchannels = channelsADC.length();
  float v_min = -1 * DAC_FULL_SCALE;
  float v_max = DAC_FULL_SCALE;

  g_done = false;
  g_samplecount = 0;
  //Do some bounds checking
  if((g_numrampDACchannels > NUMDACCHANNELS) || (g_numrampADCchannels > NUMADCCHANNELS) || (DB.size() != g_numrampDACchannels * 2 + 4))
  {
    SerialUSB.println("Syntax error");
    return;
  }
  g_numsteps=(DB[g_numrampDACchannels*2+3].toInt());
  for(i = 0; i < g_numrampDACchannels; i++)
  {
    g_DACchanselect[i] = channelsDAC[i] - '0';
    g_DACstartpoint[i] = voltageToInt32(DB[i+3].toFloat());
    //g_DACramppoint[i] = g_DACstartpoint[i];
    g_DACramppoint[i] = (int64_t)g_DACstartpoint[i] * BIT31;
    g_DACendpoint[i] = voltageToInt32(DB[i+3+g_numrampDACchannels].toFloat());
    g_DACstep[i] = (((int64_t)g_DACendpoint[i] * BIT31) - ((int64_t)g_DACstartpoint[i] * BIT31)) / g_numsteps;
    DACintegersend(g_DACchanselect[i], (g_DACramppoint[i] / BIT47));//Set DACs to initial point

    #ifdef DEBUGRAMP
    SerialUSB.print("DAC ch ");
    SerialUSB.print(g_DACchanselect[i]);
    SerialUSB.print(" Startpoint: ");
    SerialUSB.print(g_DACstartpoint[i]);
    SerialUSB.print(" Ramppoint: ");
    SerialUSB.print((int32_t)(g_DACramppoint[i] / BIT31));
    SerialUSB.print(", Finalpoint: ");
    SerialUSB.print(g_DACendpoint[i]);
    SerialUSB.print(", Stepsize: ");
    SerialUSB.println((int32_t)(g_DACstep[i] / BIT31));
    #endif
  }
  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall
  ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
  ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);

/*
  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  //SPI.transfer(adc, ADC_IO_RDYFN); //Change RDY to only trigger when all channels complete
  SPI.transfer(adc, ADC_IO_RDYFN | ADC_IO_SYNC | ADC_IO_P1DIR); //Change RDY to only trigger when all channels complete and test SYNC
*/
  for(i = 0; i < g_numrampADCchannels; i++)//Configure ADC channels
  {
    g_ADCchanselect[i] = channelsADC[i] - '0';
    SPI.transfer(adc, ADC_CHSETUP | g_ADCchanselect[i]);//Access channel setup register
    SPI.transfer(adc, ADC_CHSETUP_RNG10BI | ADC_CHSETUP_ENABLE);//set +/-10V range and enable for continuous mode
    SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]);   //Access channel mode register
    SPI.transfer(adc, ADC_MODE_CONTCONV | ADC_MODE_CLAMP);  //Continuous conversion with clamping

    #ifdef DEBUGRAMP
    SerialUSB.print("ADC ch: ");
    SerialUSB.print(g_ADCchanselect[i]);
    SerialUSB.println(" selected");
    #endif
  }

  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_RDYFN); //Change RDY to only trigger when all channels complete
  //SPI.transfer(adc, ADC_IO_RDYFN | ADC_IO_SYNC | ADC_IO_P1DIR); //Change RDY to only trigger when all channels complete and test SYNC



  delay(DACSETTLETIME);//Wait for DACs to settle

  digitalWrite(data,HIGH);

  attachInterrupt(digitalPinToInterrupt(drdy), updatead, FALLING);
  while(!g_done)
  {
    if(SerialUSB.available())
    {
      std::vector<String> comm;
      comm = query_serial();
      if(comm[0] == "STOP")
      {
        break;
      }
    }

  }
  detachInterrupt(digitalPinToInterrupt(drdy));
  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_DEFAULT); //Change RDY to trigger when any channel complete
  for(i = 0; i < NUMADCCHANNELS; i++)
  {
  SPI.transfer(adc, ADC_CHSETUP | i);//Access channel setup register
  SPI.transfer(adc, ADC_CHSETUP_RNG10BI);//set +/-10V range and disable for continuous mode
  SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]);   //Access channel mode register
  SPI.transfer(adc, ADC_MODE_IDLE);  //Set ADC to idle
  }
  digitalWrite(data,LOW);
}

void updatead()
{
   int16_t i, temp;
   if(!g_done)
   {
      ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
      ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);

      for(i = 0; i < g_numrampADCchannels; i++)
      {
         SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
         g_USBbuff[g_buffindex] = SPI.transfer(adc, 0, SPI_CONTINUE); // Reads first byte
         g_USBbuff[g_buffindex + 1] = SPI.transfer(adc, 0); // Reads second byte
         g_buffindex += 2;
      }

      if(g_samplecount < 1)//first loop has to be discarded, so just overwrite buffer
      {
        g_buffindex = 0;

      }
      g_samplecount++;

      if (g_buffindex >= USBBUFFSIZE)
      {
        SerialUSB.write((char*)g_USBbuff, g_buffindex);
        g_buffindex = 0;
      }

      if(g_samplecount > g_numsteps)
      {
        if(g_buffindex > 0)
        {
          SerialUSB.write((char*)g_USBbuff, g_buffindex);
          g_buffindex = 0;
        }
        g_done = true;
      }
      else
      {
      //get next DAC step ready if this isn't the last sample
        for(i = 0; i < g_numrampDACchannels; i++)
        {
          g_DACramppoint[i] += g_DACstep[i];
          DACintegersend(g_DACchanselect[i], (g_DACramppoint[i] / BIT47));
        }
      }
   }
}


void adc_ch_zero_scale_cal(int ch)
{

  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE);       // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SYSZEROCAL);       // Enter system zero-scale cal mode
  waitDRDY();
  readADCzerocal(ch);
}


void readADCzerocal(byte ch)
{
  byte b1, b2, b3;
  uint32_t calvalue;
  SPI.transfer(adc, ADC_CHZEROSCALECAL | ADC_REGREAD | ch);   // Access ch zero-scale cal register in read mode
  b1 = SPI.transfer(adc,0x00);   // read byte 1
  b2 = SPI.transfer(adc,0x00);   // read byte 2
  b3 = SPI.transfer(adc,0x00);   // read byte 3

  calvalue += b1 << 16;
  calvalue += b2 << 8;
  calvalue += b3;
  SerialUSB.print("ADC Channel ");
  SerialUSB.print(ch);
  SerialUSB.print(" zero-scale cal register: ");
  SerialUSB.println(calvalue);
}


void adc_ch_full_scale_cal(int ch)
{
  //Put ch in idle mode
  SPI.transfer(adc, ADC_CHMODE | ch); // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE); // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE | ch); // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SYSFULLCAL); // Enter system full-scale cal mode
  waitDRDY();

  readADCfullcal(ch);


}

void readADCfullcal(byte ch)
{
  byte b1, b2, b3;
  uint32_t calvalue;
  SPI.transfer(adc, ADC_CHFULLSCALECAL | ADC_REGREAD | ch);   // Access ch full-scale cal register in read mode
  b1 = SPI.transfer(adc,0x00);   // read byte 1
  b2 = SPI.transfer(adc,0x00);   // read byte 2
  b3 = SPI.transfer(adc,0x00);   // read byte 3

  calvalue += b1 << 16;
  calvalue += b2 << 8;
  calvalue += b3;

  SerialUSB.print("ADC Channel ");
  SerialUSB.print(ch);
  SerialUSB.print(" full-scale cal register: ");
  SerialUSB.println(calvalue);
}

void writeADCcal(byte ch, uint32_t zerocal, uint32_t fullcal)
{
   writeADCchzeroscale(ch, zerocal);
   writeADCchfullscale(ch, fullcal);
}


void calDACoffset(byte ch, float offset)
{
  int8_t numsteps;
  float stepsize = (10.0 * 2.0) / (65535.0 * 8.0); //stepsize is 1/8 of a 16-bit LSB

  if(offset < 0)
  {
    numsteps = (int8_t)((offset / stepsize) - 0.5) * -1;
  }
  else
  {
    numsteps = (int8_t)((offset / stepsize) + 0.5) * -1;
  }


  SerialUSB.print("Offset stepsize is: ");
  SerialUSB.print(stepsize * 1000000);
  SerialUSB.println("uV");
  SerialUSB.print("DAC Channel ");
  SerialUSB.print(ch);
  SerialUSB.print(" offset register: ");
  SerialUSB.println(numsteps);
  writeDACoffset(ch, numsteps);
}

void calDACgain(byte ch, float offset) //Offset is measured relative to ideal negative full scale voltage (usually -10V)
{
  int8_t numsteps;
  float stepsize = (10.0 * 2.0) / (65535.0 * 2.0); //stepsize is 1/2 of a 16-bit LSB
  numsteps = (int8_t)(offset / stepsize);

  if(offset < 0)
  {
    numsteps = (int8_t)((offset / stepsize) - 0.5);
  }
  else
  {
    numsteps = (int8_t)((offset / stepsize) + 0.5);
  }

  SerialUSB.print("Negative full-scale gain stepsize is: ");
  SerialUSB.print(stepsize * 1000000);
  SerialUSB.println("uV");
  SerialUSB.print("DAC Channel ");
  SerialUSB.print(ch);
  SerialUSB.print(" gain register: ");
  SerialUSB.println(numsteps);
  writeDACgain(ch, numsteps);
}

void dac_ch_reset_cal(byte ch)
{
  writeDACoffset(ch, 0);
  writeDACgain(ch, 0);
}

void writeDACoffset(int ch, int8_t steps)
{
  int thisdac;
  int thisldac;

  convertDACch(&ch, &thisdac, &thisldac);

  SPI.transfer(thisdac, DAC_OFFSET | ch,SPI_CONTINUE); // Write DAC channel offset register
  SPI.transfer(thisdac, 0x00,SPI_CONTINUE);   // writes first byte
  SPI.transfer(thisdac, steps);
  digitalWrite(thisldac, LOW);
  digitalWrite(thisldac, HIGH);

}

void writeDACgain(int ch, int8_t steps)
{
  int thisdac;
  int thisldac;

  convertDACch(&ch, &thisdac, &thisldac);

  SPI.transfer(thisdac, DAC_FINEGAIN  | ch,SPI_CONTINUE); // Write DAC channel fine gain register
  SPI.transfer(thisdac, 0x00,SPI_CONTINUE);   // writes first byte
  SPI.transfer(thisdac, steps);
  digitalWrite(thisldac, LOW);
  digitalWrite(thisldac, HIGH);

}
void writeADCchzeroscale(byte ch, int32_t zeroscale)
{
  SPI.transfer(adc, ADC_CHZEROSCALECAL | ch, SPI_CONTINUE); //Write channel zero scale register
  SPI.transfer(adc, (zeroscale & 0xFF0000) >> 16, SPI_CONTINUE); // Write first byte
  SPI.transfer(adc, (zeroscale & 0xFF00) >> 8, SPI_CONTINUE); // Write second byte
  SPI.transfer(adc, zeroscale & 0xFF); // Write third byte
}

void writeADCchfullscale(byte ch, int32_t fullscale)
{
  SPI.transfer(adc, ADC_CHFULLSCALECAL | ch, SPI_CONTINUE); //Write channel zero scale register
  SPI.transfer(adc, (fullscale & 0xFF0000) >> 16, SPI_CONTINUE); // Write first byte
  SPI.transfer(adc, (fullscale & 0xFF00) >> 8, SPI_CONTINUE); // Write second byte
  SPI.transfer(adc, fullscale & 0xFF); // Write third byte
}

void loaddefaultcals()
{
  byte ch = 0;
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    writeADCchzeroscale(ch, defaultADCzeroscale[ch]);
    writeADCchfullscale(ch, defaultADCfullscale[ch]);
  }
  for(ch = 0; ch < NUMDACCHANNELS; ch++)
  {
    writeDACoffset(ch, defaultDACoffset[ch]);
    writeDACgain(ch, defaultDACgain[ch]);
  }
}

void calADCwithDAC()
{
  byte ch = 0;
  SerialUSB.println("Setting DACs to 0VDC...");
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    writeDAC(ch, 0.0, true);
  }
  delay(2000);
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    adc_ch_zero_scale_cal(ch);
  }

  SerialUSB.println("Setting DACs to 10VDC...");
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    writeDAC(ch, 10.0, true);
  }
  delay(2000);
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    adc_ch_full_scale_cal(ch);
  }

}

void convertDACch(int *ch, int *spipin, int *ldacpin)
{
  if(*ch <= 3)
  {
    *spipin = dac0;
    *ldacpin = ldac0;
  }
  else
  {
    *spipin = dac1;
    *ldacpin = ldac1;
    *ch -= 4;
  }
}
