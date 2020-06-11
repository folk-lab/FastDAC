//Arduino *DUE*code for controlling EVAL-AD7734 ADC and EVAL-AD5764 DAC
//Andrea Young (UCSB)
//Carlos Kometter (UCSB)

//Modified for 8 DAC channels and consistent high speed ADC sampling by UBC PHAS E-Lab, Nov 2019

//Modified by Christian Olsen & Tim Child (Quantum Devices Group, UBC), Mar 2020
//Main code units changed to mV & several new functions added (RAMP_SMART,INT_RAMP & SPEC_ANA)
//Most original functions (UCSB) are either removed or replaced.

////////////////
//// SETUP ////
///////////////

#define NUMDACCHANNELS 8
#define NUMADCCHANNELS 4

#include "SPI.h" // necessary library for SPI communication

#include <vector>
#include "FastDACdefs.h"
#include "FastDACcalibration.h" //This cal file should be copied and renamed for each DAQ unit, maybe store in EEPROM in the future

#define OPTICAL //Comment this if still using old USB version

#define USBBUFFSIZE 300// works up to 450, but at some value higher than that the behaviour is to wait and send >2000 byte packets. Don't know why.

#define DACSETTLETIME  1//milliseconds to wait before starting ramp

//#define DEBUGRAMP //Uncomment this to enable sending of ramp debug info

#define BIT31 0x10000000 //Some scaling constants for fixed-point math
#define BIT47 0x100000000000

#ifdef OPTICAL
#define SERIALPORT Serial1
#else
#define SERIALPORT SerialUSB
#endif


#define BAUDRATE 1750000 //Tested with UM232H from regular arduino UART

const int slave_master = 23; //low for master, high for slave
const int clock_lol = 25; //active-high loss-of-lock signal from clock PLL
const int clock_los = 27; //active-high loss-of-signal from clock PLL
const int clock_led = 34; //on board clock ok led output
const int adc_trig_out = 50; //active-low ADC trigger output, starts the sampling
const int adc_trig_in = 49; //active-low ADC trigger input, for diagnostics


const int adc=52; //The SPI pin for the ADC
const int dac0 = 4; //The SPI pin for the DAC0
const int dac1 = 10; //The SPI pin for the DAC1
const int ldac0=6; //Load DAC1 pin for DAC1. Make it LOW if not in use.
const int ldac1=9; //Load DAC2 pin for DAC1. Make it LOW if not in use.
Pio *ldac_port = digitalPinToPort(ldac0); //ldac0 and ldac1 share the same port, so they can be toggled simultaneously
const uint32_t ldac0_mask = digitalPinToBitMask(ldac0);
const uint32_t ldac1_mask = digitalPinToBitMask(ldac1);

const int reset=44 ; //Reset on ADC
const int drdy=48; // Data is ready pin on ADC
const int led = 32;
const int data=28;//Used for trouble shooting; connect an LED between pin 28 and GND
const int err=30;
const int Noperations = 22;
String operations[Noperations] = {"NOP", "*IDN?", "*RDY?", "RESET", "GET_DAC", "GET_ADC", "RAMP_SMART", "INT_RAMP", "SPEC_ANA", "CONVERT_TIME", "READ_CONVERT_TIME", "CAL_ADC_WITH_DAC", "ADC_ZERO_SC_CAL", "ADC_CH_ZERO_SC_CAL", "ADC_CH_FULL_SC_CAL", "READ_ADC_CAL", "WRITE_ADC_CAL", "DAC_OFFSET_ADJ", "DAC_GAIN_ADJ", "DAC_RESET_CAL", "DEFAULT_CAL", "FULL_SCALE"};

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

void setup()
{
  SERIALPORT.begin(BAUDRATE);
  pinMode(slave_master, OUTPUT); //low for master, high for slave
  digitalWrite(slave_master, LOW);
  pinMode(clock_lol, INPUT); //active-high loss-of-lock signal from clock PLL
  pinMode(clock_los, INPUT); //active-high loss-of-signal from clock PLL
  pinMode(clock_led, OUTPUT); //on board clock ok led output
  digitalWrite(clock_led, LOW);
  pinMode(adc_trig_out, OUTPUT); //active-low ADC trigger output, starts the sampling
  digitalWrite(adc_trig_out, LOW);
  pinMode(adc_trig_in, INPUT); //active-low ADC trigger input, for diagnostics
  
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

  digitalWrite(reset,HIGH); // Resets ADC on startup.
  digitalWrite(data,LOW);
  digitalWrite(reset,LOW);
  digitalWrite(data,HIGH);
  delayMicroseconds(5000); // wait 5ms
  digitalWrite(reset,HIGH);
  digitalWrite(data,LOW);

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
}

////////////////
//// ROUTER ////
///////////////

std::vector<String> query_serial()
// read incomming serial commands
{
  char received;
  String inByte = "";
  std::vector<String> comm;
  while (received != '\r')
  {
    if(SERIALPORT.available())
    {
      received = SERIALPORT.read();
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

void loop()
// look for incoming commands, update status
{
  SERIALPORT.flush();
  std::vector<String> comm;
  if(digitalRead(clock_lol) || digitalRead(clock_los))
  {
    digitalWrite(clock_led, LOW);
  }
  else
  {
    digitalWrite(clock_led, HIGH);
  }

  if(SERIALPORT.available())
  {
    comm = query_serial();
    router(comm);
  }
}

void router(std::vector<String> DB)
{
  float v;
  String buffer;
  int operation = indexOfOperation(DB[0]);
  switch ( operation )
  {
    case 0: // NOP
    SERIALPORT.println("NOP");
    break;

    case 1: // *IDN?
    IDN();
    break;

    case 2: // *RDY?
    RDY();
    break;

    case 3: // RESET
    resetADC();
    break;

    case 4: // GET_DAC
    SERIALPORT.println(getDAC(DB[1].toInt()), 4);
    break;

    case 5: // GET_ADC
    v=readADC(DB[1].toInt());
    SERIALPORT.println(v,4);
    break;

    case 6: // RAMP_SMART
    ramp_smart(DB);
    SERIALPORT.println("RAMP_FINISHED");
    break;

    case 7: // INT_RAMP
    intRamp(DB);
    SERIALPORT.println("RAMP_FINISHED");
    break;

    case 8: // SPEC_ANA
    spec_ana(DB);
    SERIALPORT.println("READ_FINISHED");
    break;

    case 9: // CONVERT_TIME
    writeADCConversionTime(DB);
    break;

    case 10: // READ_CONVERT_TIME
    readADCConversionTime(DB);
    break;

    case 11: // CAL_ADC_WITH_DAC
    calADCwithDAC();
    SERIALPORT.println("CALIBRATION_FINISHED");
    break;

    case 12: // ADC_ZERO_SC_CAL
    adc_zero_scale_cal(DB[1].toInt());
    SERIALPORT.println("CALIBRATION_FINISHED");
    break;

    case 13: // ADC_CH_ZERO_SC_CAL
    buffer = adc_ch_zero_scale_cal(DB[1].toInt());
    SERIALPORT.println(buffer);
    SERIALPORT.println("CALIBRATION_FINISHED");
    break;

    case 14: // ADC_CH_FULL_SC_CAL
    buffer = adc_ch_full_scale_cal(DB[1].toInt());
    SERIALPORT.println(buffer);
    SERIALPORT.println("CALIBRATION_FINISHED");
    break;

    case 15: // READ_ADC_CAL
    buffer = readADCzerocal(DB[1].toInt());
    buffer += readADCfullcal(DB[1].toInt());
    SERIALPORT.println(buffer);
    SERIALPORT.println("READ_FINISHED");
    break;

    case 16: // WRITE_ADC_CAL
    writeADCcal(DB[1].toInt(), DB[2].toInt(), DB[3].toInt());
    SERIALPORT.println("CALIBRATION_CHANGED");
    break;

    case 17: // DAC_OFFSET_ADJ
    calDACoffset(DB[1].toInt(), DB[2].toFloat());
    SERIALPORT.println("CALIBRATION_FINISHED");
    break;

    case 18: // DAC_GAIN_ADJ
    calDACgain(DB[1].toInt(), DB[2].toFloat());
    SERIALPORT.println("CALIBRATION_FINISHED");
    break;

    case 19: // DAC_RESET_CAL
    dac_ch_reset_cal(DB[1].toInt());
    SERIALPORT.println("CALIBRATION_RESET");
    break;

    case 20: // DEFAULT_CAL
    loaddefaultcals(); // set default calibration
    SERIALPORT.println("CALIBRATION_CHANGED");
    break;

    case 21: // FULL_SCALE
    DAC_FULL_SCALE = DB[1].toFloat();
    SERIALPORT.println("FULL_SCALE_UPDATED");
    break;

    default:
    break;
  }
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

///////////////////
//// GET/READ ////
//////////////////

//// ADC ////

float readADC(byte DB)
{
  int adcChannel=DB;
  float voltage;
  voltage = getSingleReading(adcChannel); // return mV
  return voltage; // return mV
}

void readADCConversionTime(std::vector<String> DB)
{
  if(DB.size() != 2)
  {
    SERIALPORT.println("SYNTAX ERROR");
    return;
  }
  int adcChannel;
  adcChannel = DB[1].toInt();
  if (adcChannel < 0 || adcChannel > 3)
  {
    SERIALPORT.println("ADC channel must be between 0 - 3");
    return;
  }
  byte cr;
  SPI.transfer(adc, ADC_REGREAD | ADC_CHCONVTIME | adcChannel); //Read conversion time register
  cr=SPI.transfer(adc,0); //Read back the CT register
  //SERIALPORT.println(fw);
  int convtime = ((int)(((cr&127)*128+249)/6.144)+0.5);
  SERIALPORT.println(convtime);
}

//// DAC ////

float getDAC(int ch)
{
  float voltage;
  voltage = int16ToVoltage(g_DACsetpoint[ch]);
  return voltage*1000.0; // return mV
}

///////////////////
//// SET/RAMP ////
//////////////////

//// ADC ////

void writeADCConversionTime(std::vector<String> DB)
{
  int adcChannel = DB[1].toInt();
  if(DB.size() != 3 || adcChannel > NUMADCCHANNELS-1)
  {
    SERIALPORT.println("SYNTAX ERROR");
    return;
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

  SPI.transfer(adc, ADC_CHCONVTIME | adcChannel); //Write conversion time register
  SPI.transfer(adc, fw); //Write 'filter word' (conversion time)
  delayMicroseconds(100);
  SPI.transfer(adc, ADC_REGREAD | ADC_CHCONVTIME | adcChannel); //Read conversion time register
  cr=SPI.transfer(adc,0); //Read back the CT register

  int convtime = ((int)(((cr&127)*128+249)/6.144)+0.5);
  SERIALPORT.println(convtime);
}

//// DAC ////

void ramp_smart(std::vector<String> DB)  //(channel,setpoint,ramprate)
{
  if(DB.size() != 4)
  {
    SERIALPORT.println("SYNTAX ERROR");
    return;
  }
  float channel = DB[1].toInt();
  float setpoint = DB[2].toFloat();
  float ramprate = DB[3].toFloat();
  float initial = getDAC(channel);  //mV

  if (abs(setpoint-initial) < 0.0001)  //If already at setpoint
  {
    return;
  }
  // Calc ramprate, make vector string, pass to autoRamp1
  int nSteps = static_cast<int>(abs(setpoint-initial)/ramprate*1000);  //using 1ms as delay
  if (nSteps < 5)
  {
    nSteps = 5;
  }
  std::vector<String> autoRampInput;
  autoRampInput.push_back("RAMP1");
  autoRampInput.push_back(String(channel));
  autoRampInput.push_back(String(initial));
  autoRampInput.push_back(String(setpoint));
  autoRampInput.push_back(String(nSteps));
  autoRampInput.push_back("1000"); //1000us delay between steps
  autoRamp1(autoRampInput);
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
    SERIALPORT.println("SYNTAX ERROR");
    return;
  }
  g_numsteps=(DB[g_numrampDACchannels*2+3].toInt());
  for(i = 0; i < g_numrampDACchannels; i++)
  {
    g_DACchanselect[i] = channelsDAC[i] - '0';
    g_DACstartpoint[i] = voltageToInt32(DB[i+3].toFloat()/1000.0);
    //g_DACramppoint[i] = g_DACstartpoint[i];
    g_DACramppoint[i] = (int64_t)g_DACstartpoint[i] * BIT31;
    g_DACendpoint[i] = voltageToInt32(DB[i+3+g_numrampDACchannels].toFloat()/1000.0);
    g_DACstep[i] = (((int64_t)g_DACendpoint[i] * BIT31) - ((int64_t)g_DACstartpoint[i] * BIT31)) / g_numsteps;
    DACintegersend(g_DACchanselect[i], (g_DACramppoint[i] / BIT47));//Set DACs to initial point

    #ifdef DEBUGRAMP
    SERIALPORT.print("DAC ch ");
    SERIALPORT.print(g_DACchanselect[i]);
    SERIALPORT.print(" Startpoint: ");
    SERIALPORT.print(g_DACstartpoint[i]);
    SERIALPORT.print(" Ramppoint: ");
    SERIALPORT.print((int32_t)(g_DACramppoint[i] / BIT31));
    SERIALPORT.print(", Finalpoint: ");
    SERIALPORT.print(g_DACendpoint[i]);
    SERIALPORT.print(", Stepsize: ");
    SERIALPORT.println((int32_t)(g_DACstep[i] / BIT31));
    #endif
  }
  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall
  ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
  ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);

  for(i = 0; i < g_numrampADCchannels; i++)//Configure ADC channels
  {
    g_ADCchanselect[i] = channelsADC[i] - '0';
    SPI.transfer(adc, ADC_CHSETUP | g_ADCchanselect[i]);//Access channel setup register
    SPI.transfer(adc, ADC_CHSETUP_RNG10BI | ADC_CHSETUP_ENABLE);//set +/-10V range and enable for continuous mode
    SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]);   //Access channel mode register
    SPI.transfer(adc, ADC_MODE_CONTCONV | ADC_MODE_CLAMP);  //Continuous conversion with clamping

    #ifdef DEBUGRAMP
    SERIALPORT.print("ADC ch: ");
    SERIALPORT.print(g_ADCchanselect[i]);
    SERIALPORT.println(" selected");
    #endif
  }

  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_RDYFN); //Change RDY to only trigger when all channels complete

  delayMicroseconds(1000); // wait for DACs to settle
  digitalWrite(data,HIGH);

  attachInterrupt(digitalPinToInterrupt(drdy), updatead, FALLING);
  while(!g_done)
  {
    if(SERIALPORT.available())
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

////////////////////////////
//// SPECTRUM ANALYZER ////
///////////////////////////

void spec_ana(std::vector<String> DB)
{
  int i, j;
  bool error = false;
  String channelsADC = DB[1];
  g_numrampADCchannels = channelsADC.length();
  g_numsteps=DB[2].toInt();

  g_done = false;
  g_samplecount = 0;
  // Do some bounds checking
  if((g_numrampADCchannels > NUMADCCHANNELS) || (DB.size() != 3))
  {
    SERIALPORT.println("SYNTAX ERROR");
    return;
  }

  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall

  for(i = 0; i < g_numrampADCchannels; i++) // Configure ADC channels
  {
    g_ADCchanselect[i] = channelsADC[i] - '0';
    SPI.transfer(adc, ADC_CHSETUP | g_ADCchanselect[i]);//Access channel setup register
    SPI.transfer(adc, ADC_CHSETUP_RNG10BI | ADC_CHSETUP_ENABLE);//set +/-10V range and enable for continuous mode
    SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]);   //Access channel mode register
    SPI.transfer(adc, ADC_MODE_CONTCONV | ADC_MODE_CLAMP);  //Continuous conversion with clamping
  }

  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_RDYFN); //Change RDY to only trigger when all channels complete

  digitalWrite(data,HIGH);

  attachInterrupt(digitalPinToInterrupt(drdy), writetobuffer, FALLING);
  while(!g_done)
  {
    // Look for user interrupt ("STOP")
    if(SERIALPORT.available())
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
  SPI.transfer(adc, ADC_IO); // Write to ADC IO register
  SPI.transfer(adc, ADC_IO_DEFAULT); // Change RDY to trigger when any channel complete
  for(i = 0; i < NUMADCCHANNELS; i++)
  {
  SPI.transfer(adc, ADC_CHSETUP | i); // Access channel setup register
  SPI.transfer(adc, ADC_CHSETUP_RNG10BI); // set +/-10V range and disable for continuous mode
  SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]); // Access channel mode register
  SPI.transfer(adc, ADC_MODE_IDLE); // Set ADC to idle
  }
  digitalWrite(data,LOW);
}

void writetobuffer()
{
   int16_t i, temp;
   if(!g_done)
   {
      for(i = 0; i < g_numrampADCchannels; i++)
      {
         SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); // Read channel data register
         g_USBbuff[g_buffindex] = SPI.transfer(adc, 0, SPI_CONTINUE); // Reads first byte
         g_USBbuff[g_buffindex + 1] = SPI.transfer(adc, 0); // Reads second byte
         g_buffindex += 2;
      }
      g_samplecount++;

      if (g_buffindex >= USBBUFFSIZE)
      {
        SERIALPORT.write((char*)g_USBbuff, g_buffindex);
        g_buffindex = 0;
      }

      if(g_samplecount >= g_numsteps)
      {
        if(g_buffindex > 0)
        {
          SERIALPORT.write((char*)g_USBbuff, g_buffindex);
          g_buffindex = 0;
        }
        g_done = true;
      }
   }
}

//////////////////////
//// CALIBRATION ////
/////////////////////

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

//// ADC ////

void calADCwithDAC()
{
  byte ch = 0;
  String buffer;

  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    writeDAC(ch, 0.0, true); // mV
  }
  delayMicroseconds(2000); // wait 2 ms
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    buffer += adc_ch_zero_scale_cal(ch);
    buffer += ",";
  }

  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    writeDAC(ch, 10000.0, true); // mV
  }
  delayMicroseconds(2000); // wait 2ms
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    buffer += adc_ch_full_scale_cal(ch);
    buffer += ",";
  }

  SERIALPORT.println(buffer);
}

void adc_zero_scale_cal(int ch)
{
  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE);       // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SELFZEROCAL);       // Enter system zero-scale cal mode
  waitDRDY();
}

String adc_ch_zero_scale_cal(int ch)
{

  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE);       // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SYSZEROCAL);       // Enter system zero-scale cal mode
  waitDRDY();
  return readADCzerocal(ch);
}

String readADCzerocal(byte ch)
{
  byte b1, b2, b3;
  uint32_t calvalue;
  int n;
  String buffer;
  char buffertemp [100];
  SPI.transfer(adc, ADC_CHZEROSCALECAL | ADC_REGREAD | ch);   // Access ch zero-scale cal register in read mode
  b1 = SPI.transfer(adc,0x00);   // read byte 1
  b2 = SPI.transfer(adc,0x00);   // read byte 2
  b3 = SPI.transfer(adc,0x00);   // read byte 3

  calvalue += b1 << 16;
  calvalue += b2 << 8;
  calvalue += b3;

  n = snprintf(buffertemp,100,"ch%d,%d",ch,calvalue);
  buffer = buffertemp;
  return buffer;
}

String adc_ch_full_scale_cal(int ch)
{
  //Put ch in idle mode
  SPI.transfer(adc, ADC_CHMODE | ch); // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE); // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE | ch); // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SYSFULLCAL); // Enter system full-scale cal mode
  waitDRDY();

  return readADCfullcal(ch);
}

String readADCfullcal(byte ch)
{
  byte b1, b2, b3;
  uint32_t calvalue;
  int n;
  String buffer;
  char buffertemp [100];

  SPI.transfer(adc, ADC_CHFULLSCALECAL | ADC_REGREAD | ch);   // Access ch full-scale cal register in read mode
  b1 = SPI.transfer(adc,0x00);   // read byte 1
  b2 = SPI.transfer(adc,0x00);   // read byte 2
  b3 = SPI.transfer(adc,0x00);   // read byte 3

  calvalue += b1 << 16;
  calvalue += b2 << 8;
  calvalue += b3;

  n = snprintf(buffertemp,100,"ch%d,%d",ch,calvalue);
  buffer = buffertemp;

  return buffer;
}

void writeADCcal(byte ch, uint32_t zerocal, uint32_t fullcal)
{
   writeADCchzeroscale(ch, zerocal);
   writeADCchfullscale(ch, fullcal);
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

//// DAC ////

void calDACoffset(byte ch, float offset)
{
  int8_t numsteps;
  int n;
  String buffer;
  char buffertemp [100];
  float stepsize = (10.0 * 2.0) / (65535.0 * 8.0); //stepsize is 1/8 of a 16-bit LSB

  if(offset < 0)
  {
    numsteps = (int8_t)((offset / stepsize) - 0.5) * -1;
  }
  else
  {
    numsteps = (int8_t)((offset / stepsize) + 0.5) * -1;
  }

  n = snprintf(buffertemp,100,"ch%d,%f,%d",ch,stepsize*1000000,numsteps);
  buffer = buffertemp;

  SERIALPORT.println(buffer);
  writeDACoffset(ch, numsteps);
}

void calDACgain(byte ch, float offset)
// offset is measured relative to ideal negative full scale voltage (usually -10V)
{
  int8_t numsteps;
  int n;
  String buffer;
  char buffertemp [100];
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

  n = snprintf(buffertemp,100,"%f,%d",stepsize*1000000,numsteps);
  buffer = buffertemp;

  SERIALPORT.println(buffer);
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

////////////////////
//// UTILITIES ////
///////////////////

//// GENERAL ////

void IDN()
// return IDN string
{
  SERIALPORT.println(IDSTRING);
}

void RDY()
// retrun "READY"
{
  SERIALPORT.println("READY");
}

void waitDRDY()
// wait for DRDY
{
  while (digitalRead(drdy)==HIGH){}
}

void debug()
{
  digitalWrite(data,HIGH);
  delay(3000); // wait 3s
  digitalWrite(data,LOW);
  delay(3000); // wait 3s
}

void error()
// use LED to signal error
{
  digitalWrite(err,HIGH);
  delay(3000); // wait 3s
  digitalWrite(err,LOW);
  delay(500); // wait 0.5s
}

void blinker(int s)
// blink LED on box
{
  digitalWrite(data,HIGH);
  delay(s);
  digitalWrite(data,LOW);
  delay(s);
}

void sos()
{
  // S (. . .)
  blinker(50);
  blinker(50);
  blinker(50);
  // O (- - -)
  blinker(500);
  blinker(500);
  blinker(500);
  // S (. . .)
  blinker(50);
  blinker(50);
  blinker(50);
}

namespace std {
  void __throw_bad_alloc()
  {
    SERIALPORT.println("Unable to allocate memory");
  }

  void __throw_length_error( char const*e )
  {
    SERIALPORT.print("Length Error :");
    SERIALPORT.println(e);
  }
}

//// ADC UTIL ////

float getSingleReading(int adcchan)
{
  SERIALPORT.flush();
  int statusbyte=0;
  byte o2;
  byte o3;
  if(adcchan < NUMADCCHANNELS)
  {
    SPI.transfer(adc, ADC_CHMODE | adcchan);   // Write channel mode register
    SPI.transfer(adc, ADC_MODE_SINGLECONV | ADC_MODE_DUMP | ADC_MODE_CLAMP); // Single conversion + dump mode + clamp
    waitDRDY();                       // Waits until convertion finishes
    SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | adcchan, SPI_CONTINUE);   // Read channel data register
    statusbyte=SPI.transfer(adc, 0, SPI_CONTINUE);   // Reads Channel 'ch' status
    o2=SPI.transfer(adc, 0, SPI_CONTINUE);           // Reads first byte
    o3=SPI.transfer(adc, 0, SPI_CONTINUE);           // Reads second byte

    int decimal;
    decimal = twoByteToInt(o2,o3);
    float voltage;
    voltage = map2(decimal, 0, 65536, -10000.0, 10000.0);
    return voltage;
  }
}

float map2(float x, long in_min, long in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int twoByteToInt(byte DB1,byte DB2)
// map to bytes to 16bit int (between +/- 2^16)
{
  return ((int)((DB1<<8)| DB2));
}

void resetADC()
// resets the ADCs, and sets the range to default +-10 V
{
  byte ch = 0;

  digitalWrite(data,HIGH);digitalWrite(reset,HIGH);digitalWrite(reset,LOW);digitalWrite(reset,HIGH);
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    SPI.transfer(adc, ADC_CHSETUP | ch);// access channel setup register for each channel
    SPI.transfer(adc, ADC_CHSETUP_RNG10BI);// set +/-10V range
  }
}

//// DAC UTIL ////

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
        SERIALPORT.write((char*)g_USBbuff, g_buffindex);
        g_buffindex = 0;
      }

      if(g_samplecount > g_numsteps)
      {
        if(g_buffindex > 0)
        {
          SERIALPORT.write((char*)g_USBbuff, g_buffindex);
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

void autoRamp1(std::vector<String> DB)
// voltage in mV
{
  if(DB.size() != 6)
  {
    SERIALPORT.println("SYNTAX ERROR");
    return;
  }

  float v1 = DB[2].toFloat(); // mV
  float v2 = DB[3].toFloat(); // mV
  int nSteps = DB[4].toInt();
  int dacChannel=DB[1].toInt();

  for (int j=0; j<nSteps;j++)
  {
    int timer = micros();
    digitalWrite(data,HIGH);
    writeDAC(dacChannel, v1+(v2-v1)*j/(nSteps-1), true); // takes mV
    digitalWrite(data,LOW);
    while(micros() <= timer + DB[5].toInt());
  }
}

float writeDAC(int dacChannel, float voltage, bool load)
// voltage in mV
{
  float actualvoltage;
  if(dacChannel >= NUMDACCHANNELS)
  {
    SERIALPORT.println("SYNTAX ERROR");
    return 0;
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

float dacDataSend(int ch, float voltage)
// voltage in mV
{
  digitalWrite(data, HIGH);
  DACintegersend(ch, voltageToInt16(voltage/1000.0));
  float voltreturn;
  voltreturn = getDAC(ch);
  return voltreturn;
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

void intToTwoByte(int s, byte * DB1, byte * DB2)
// map 16bit int to two bytes
{
    *DB1 = ((byte)((s>>8)&0xFF));
    *DB2 = ((byte)(s&0xFF));
}

float int16ToVoltage(int16_t data)
// map 16bit int to float voltage
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
// map float voltage to 16bit int
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
// map float voltage to 32bit int
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
