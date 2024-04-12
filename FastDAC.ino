//Arduino *DUE*code for controlling EVAL-AD7734 ADC and EVAL-AD5764 DAC
//Andrea Young (UCSB)
//Carlos Kometter (UCSB)

//Modified for 8 DAC channels and consistent high speed ADC sampling by UBC PHAS E-Lab, Nov 2019

//Modified by Christian Olsen & Tim Child (Quantum Devices Group, UBC), Mar 2020
//Main code units changed to mV & several new functions added (RAMP_SMART,INT_RAMP & SPEC_ANA)
//Most original functions (UCSB) are either removed or replaced.

//Modified for new PC orchestrator service, extra checks for out-of-bounds array indexing
//Switched use of String class to c strings to have better control over RAM usage, April 2024

////////////////
//// SETUP ////
///////////////


#include "SPI.h" // necessary library for SPI communication

#include "src/PID/PID_v1.h" // Our own 'fork' of https://github.com/br3ttb/Arduino-PID-Library/
#include <vector>
#include "FastDACdefs.h"
#include "FastDACcalibration.h" //This cal file should be copied and renamed for each DAQ unit, maybe store in EEPROM in the future
#include "FastDACeeprom.h"

#define SENDACK //Comment this to stop sending ACKs for every command
#define AWGMAXSETPOINTS 100 //Maximum number of setpoints of waveform generator
#define AWGMAXWAVES 2 //Maximum number of individual waveforms

#define MAXNUMPIDS 1 //Maximum number of simultaneous PID loops, only 1 for now

#define USBBUFFSIZE 300// works up to 450, but at some value higher than that the behaviour is to wait and send >2000 byte packets. Don't know why. Not used with optical comms

#define COMMANDBUFFERSIZE 1024 //Buffer for incoming command
#define MAXPARAMS 50 //maximum number of parameters to be parsed in a single command

#define DACSETTLETIME  1//milliseconds to wait before starting ramp

//#define DEBUGRAMP //Uncomment this to enable sending of ramp debug info

#define BIT31 0x10000000 //Some scaling constants for fixed-point math
#define BIT47 0x100000000000

#define BAUDRATE 1750000 //Tested with UM232H from regular arduino UART, try to stay as integer divisor of 84MHz mclk

typedef enum MS_select {MASTER, SLAVE, INDEP} MS_select;
MS_select g_ms_select = INDEP; //Master/Slave/Independent selection variable
bool g_clock_synced = false;

const int slave_master = 23; //low for master, high for slave
const int clock_lol = 25; //active-high loss-of-lock signal from clock PLL
const int clock_los = 27; //active-high loss-of-signal from clock PLL
const int clock_led = 34; //on board clock ok led output
const int ext_clock_led = 32; //external clock ok led output
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
const int led = 28;
const int data=30;//Used for trouble shooting; connect an LED between pin 28 and GND
const int err=35;

float DAC_FULL_SCALE = 10.0;

volatile int16_t g_DACsetpoint[NUMDACCHANNELS];//global array for current DAC setpoints, only written to in DACintegersend()

volatile byte g_USBbuff[USBBUFFSIZE + 100]; //add some bytes to the buffer to prevent overflow in interrupt

//Ramp interrupt global variables
volatile uint32_t g_buffindex = 0;
volatile bool g_done = false;
volatile bool g_firstsamples = true;
volatile uint8_t g_numrampADCchannels;
volatile uint8_t g_ADCchanselect[NUMADCCHANNELS];
volatile uint8_t g_numrampDACchannels;
volatile uint8_t g_DACchanselect[NUMDACCHANNELS];
volatile int64_t g_DACramppoint[NUMDACCHANNELS];
volatile int32_t g_DACstartpoint[NUMDACCHANNELS];
volatile int32_t g_DACendpoint[NUMDACCHANNELS];
volatile int64_t g_DACstep[NUMDACCHANNELS];
volatile uint32_t g_numsteps;
volatile uint32_t g_stepcount = 0;

typedef struct InCommand
{
  char buffer[COMMANDBUFFERSIZE];
  char *token[MAXPARAMS];
  uint16_t paramcount;
  uint16_t bufflength;
}InCommand;

//Arbitrary waveform specific

typedef struct AWGwave
{
  int16_t setpoint[AWGMAXSETPOINTS];
  uint32_t numsamples[AWGMAXSETPOINTS];
  uint32_t numsetpoints;
  uint8_t numDACchannels;
  uint8_t DACchanselect[NUMDACCHANNELS];
  uint32_t setpointcount;
  uint32_t samplecount;
}AWGwave;

AWGwave g_awgwave[AWGMAXWAVES];

volatile uint8_t g_numwaves;

volatile uint32_t g_numloops;
volatile uint32_t g_loopcount;

volatile bool g_nextloop = false;

//PID Specific
typedef struct PIDparam
{
  bool active = false;
  bool forward_dir = true;
  uint8_t ADCchan = 0;
  uint8_t DACchan = 0;
  uint16_t sampletime = 10;
  uint32_t loopcount = 0;
  double adcin = 0.0;
  double dacout = 0.0;
  double dacoutlim = 0.0;
  double setpoint = 0.0;
  double dacmin = -10000.0;
  double dacmax = 10000.0;
  double kp = 0.1;
  double ki = 1.0;
  double kd = 0.0;
  double slewlimit = 10000000.0; //slew limit in mv/sec, make it big because it intereferes with the pid
  double slewcycle = 4000.0; //slew limit in mv/cycle, default for 400us sampletime
}PIDparam;

PIDparam g_pidparam[MAXNUMPIDS];

PID pid0(&(g_pidparam[0].adcin), &(g_pidparam[0].dacout), &(g_pidparam[0].setpoint), g_pidparam[0].kp, g_pidparam[0].ki, g_pidparam[0].kd, DIRECT);


void setup()
{    
  SERIALPORT.begin(BAUDRATE);

  g_ms_select = INDEP; //Defaults to independent  
  pinMode(slave_master, OUTPUT); //low for master, high for slave
  digitalWrite(slave_master, LOW);
  pinMode(adc_trig_out, OUTPUT); //active-high ADC trigger output, starts the sampling
  digitalWrite(adc_trig_out, HIGH);
  pinMode(adc_trig_in, INPUT); //active-low ADC trigger input, for diagnostics
  
  pinMode(clock_lol, INPUT); //active-high loss-of-lock signal from clock PLL
  pinMode(clock_los, INPUT); //active-high loss-of-signal from clock PLL
  pinMode(clock_led, OUTPUT); //on board clock ok led output
  pinMode(ext_clock_led, OUTPUT); //on board clock ok led output
  digitalWrite(clock_led, LOW);
  digitalWrite(ext_clock_led, LOW);
  
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

  SPI.setClockDivider(adc,8); // Maximum 10.5 Mhz for AD7734
  SPI.setClockDivider(dac0,4); // Maximum 21 Mhz for AD5764
  SPI.setClockDivider(dac1,4); // Maximum 21 Mhz for AD5764

  SPI.setDataMode(adc,SPI_MODE3); //This should be 3 for the AD7734
  SPI.setDataMode(dac0,SPI_MODE1); //This should be 1 for the AD5764
  SPI.setDataMode(dac1,SPI_MODE1); //This should be 1 for the AD5764
  
	if(initeeprom() != 0)
  {
    SERIALPORT.println("ERROR Initializing EEPROM!");
  }
  //loaddefaultcals(); //Only useful if per-unit cals are done, otherwise it's way off
  
	loaddaccals();
	//Initialize saved DAC setpoints to 0
  for(int i = 0; i < NUMDACCHANNELS; i++)
  {
    g_DACsetpoint[i] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(drdy), intset, FALLING);//Interrupt has to be attached once for the priority to stick
  NVIC_SetPriority(PIOC_IRQn, 1); //Make ADC interrupt priority lower than UART
  detachInterrupt(digitalPinToInterrupt(drdy));

}
void intset(void)
{

}

////////////////
//// ROUTER ////
///////////////

//query_serial returns true if command detected
bool query_serial(InCommand *incommand)
{  
  static uint16_t index = 0; 

  bool commreceived = false;  
  char received;  
  if(SERIALPORT.available() > 0)
  {
    received = SERIALPORT.read();
    if ((received == '\r') || (received == '\n'))//Check for end of message 
    {
      if(index >= 1)//check if the termchars were extra from previous command and ignore EOM on its own
      {
        incommand->buffer[index] = '\0';
        commreceived = true;
      }      
    }
    else if(index < COMMANDBUFFERSIZE - 2) //Add new character to string, but leave room for the null
    {
      incommand->buffer[index] = received;
      index++;
    }
    else
    {
      SERIALPORT.println("ERROR_COMMAND_BUFFER_OVERFLOW");
      index = 0;      
    }
  }
  if(commreceived)
  {
    uint16_t count = 0;
    char * token = strtok(incommand->buffer, ",");
    while (token != NULL)
    {
      //SERIALPORT.println(token);
      incommand->token[count] = token;
      count++;
      token = strtok(NULL, ",");
    }    
    incommand->paramcount = count;
    incommand->bufflength = index;
    //SERIALPORT.println(incommand->paramcount);
    //SERIALPORT.println(incommand->bufflength);
    index = 0;
  }
  return commreceived;

}

void loop()
// look for incoming commands, update status
{
  //static bool cmd = false;
  static InCommand incommand = {0};
  if(digitalRead(clock_lol) || digitalRead(clock_los))
  {
    g_clock_synced = false;
    digitalWrite(clock_led, LOW);
    digitalWrite(ext_clock_led, LOW);
  }
  else
  {
    g_clock_synced = true;
    digitalWrite(clock_led, HIGH);
    digitalWrite(ext_clock_led, HIGH);
  }
  
  if(query_serial(&incommand))
  {
    router(&incommand);    
  }
     
}

void router(InCommand *incommand)
{
  char * cmd = incommand->token[0];
  if(strcmp("*IDN?", cmd) == 0)
  {
    idn();
  }
  else if(strcmp("*RDY?", cmd) == 0)
  {
    rdy();
  }
  else if(strcmp("RESET", cmd) == 0)
  {
    resetADC();
  }
  else if(strcmp("GET_DAC", cmd) == 0)
  {
    get_dac(incommand);
  }
  else if(strcmp("GET_ADC", cmd) == 0)
  {
    get_adc(incommand);
  }
  else if(strcmp("RAMP_SMART", cmd) == 0)
  {  
    ramp_smart(incommand);
  }
  else if(strcmp("INT_RAMP", cmd) == 0)  
  {  
    int_ramp(incommand);
  }
  else if(strcmp("SPEC_ANA", cmd) == 0)  
  {  
    spec_ana(incommand);
  }
  else if(strcmp("CONVERT_TIME", cmd) == 0)  
  {  
    convert_time(incommand);
  }
  else if(strcmp("READ_CONVERT_TIME", cmd) == 0)
  {  
    read_convert_time(incommand);
  }
  else if(strcmp("CAL_ADC_WITH_DAC", cmd) == 0)  
  {  
    cal_adc_with_dac(incommand);
  }
  else if(strcmp("ADC_ZERO_SC_CAL", cmd) == 0)
  {  
    adc_zero_sc_cal(incommand);
  }
  else if(strcmp("ADC_CH_ZERO_SC_CAL", cmd) == 0)
  {
    adc_ch_zero_sc_cal(incommand);        
  }
  else if(strcmp("ADC_CH_FULL_SC_CAL", cmd) == 0)
  {  
    adc_ch_full_sc_cal(incommand);
  }
  else if(strcmp("READ_ADC_CAL", cmd) == 0)  
  {  
    read_adc_cal(incommand);
  }
  else if(strcmp("WRITE_ADC_CAL", cmd) == 0)  
  {  
    write_adc_cal(incommand);
  }
  else if(strcmp("DAC_OFFSET_ADJ", cmd) == 0)  
  {  
    dac_offset_adj(incommand);
  }
  else if(strcmp("DAC_GAIN_ADJ", cmd) == 0)  
  {  
    dac_gain_adj(incommand);
  }
  else if(strcmp("DAC_RESET_CAL", cmd) == 0)  
  {  
    dac_reset_cal(incommand);
  }
  else if(strcmp("DEFAULT_CAL", cmd) == 0)  
  {  
    default_cal(incommand);
  }
  else if(strcmp("FULL_SCALE", cmd) == 0)  
  {  
    full_scale(incommand);
  }
  else if(strcmp("SET_MODE", cmd) == 0)  
  {  
    set_mode(incommand);
  }
  else if(strcmp("ARM_SYNC", cmd) == 0)  
  {  
    arm_sync(incommand);
  }
  else if(strcmp("DISARM_SYNC", cmd) == 0)
  {  
    disarm_sync(incommand);
  }
  else if(strcmp("CHECK_SYNC", cmd) == 0)  
  {  
    check_sync(incommand);
  }
  else if(strcmp("ADD_WAVE", cmd) == 0)  
  {  
    add_wave(incommand);
  }
  else if(strcmp("CLR_WAVE", cmd) == 0)  
  {  
    clr_wave(incommand);
  }
  else if(strcmp("CHECK_WAVE", cmd) == 0)  
  { 
    check_wave(incommand);
  }
  else if(strcmp("AWG_RAMP", cmd) == 0)  
  {  
    awg_ramp(incommand);
  }
  else if(strcmp("START_PID", cmd) == 0)
  {  
    start_pid(incommand);    
  }
  else if(strcmp("STOP_PID", cmd) == 0)  
  {  
    stop_pid(incommand);
  }
  else if(strcmp("SET_PID_TUNE", cmd) == 0)
  {  
    set_pid_tune(incommand);
  }
  else if(strcmp("SET_PID_SETP", cmd) == 0)
  {  
    set_pid_setp(incommand);
  }
  else if(strcmp("SET_PID_LIMS", cmd) == 0)
  {  
    set_pid_lims(incommand);
  }
  else if(strcmp("SET_PID_DIR", cmd) == 0)
  {  
    set_pid_dir(incommand);
  }
  else if(strcmp("SET_PID_SLEW", cmd) == 0)
  {  
    set_pid_slew(incommand);   
  }
  else if(strcmp("EEPROM_TEST", cmd) == 0)
  {  
    eeprom_test(incommand);
  }
  else if(strcmp("WRITE_ID_EEPROM", cmd) == 0)
  {  
    write_id_eeprom(incommand);
  }
	else if(strcmp("WRITE_DAC_CAL_EEPROM", cmd) == 0)
  {  
    write_dac_cal_eeprom(incommand);
  }
  else
  {
    SERIALPORT.println("NOP");
  }

	//SERIALPORT.print(F("Free RAM = ")); //F function does the same and is now a built in library, in IDE > 1.0.0
  //SERIALPORT.println(freeMemory(), DEC);  // print how much RAM is available.
}

///////////////////
//// GET/READ ////
//////////////////

//// ADC ////

void get_adc(InCommand *incommand)
{
  if(sync_check(CHECK_CLOCK) != 0)//make sure ADC has a clock
  {
    return;
  }  
  if(incommand->paramcount != 2)//Check correct number of parameters
  {
    syntax_error();
    return;
  }
  uint8_t adcChannel = atoi(incommand->token[1]);
  if(adcChannel >= NUMADCCHANNELS)
  {
    range_error();
    return;
  }
  else
  {
    send_ack();
    SERIALPORT.println(getSingleReading(adcChannel),4);
  }  
  return;
}

void read_convert_time(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  uint8_t adcChannel;
  adcChannel = atoi(incommand->token[1]);
  if(adcChannel >= NUMADCCHANNELS)
  {
    range_error();
    return;
  }
  send_ack();  
  SERIALPORT.println(readADCConversionTime(adcChannel));
}

int readADCConversionTime(int ch)
{
  if (ch < 0 || ch > 3)
  {
    range_error();
    return -1;
  }
  byte cr;
  SPI.transfer(adc, ADC_REGREAD | ADC_CHCONVTIME | ch); //Read conversion time register
  cr=SPI.transfer(adc,0); //Read back the CT register
  //SERIALPORT.println(fw);
  int contime = ((int)(((cr&127)*128+249)/6.144)+0.5);
  return contime;
}

//// DAC ////
void get_dac(InCommand * incommand) //(DAC channel)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  uint8_t dacChannel = atoi(incommand->token[1]);
  if(dacChannel >= NUMDACCHANNELS)
  {
    range_error();
    return;
  }
  else
  {
    send_ack();
    SERIALPORT.println(readDAC(dacChannel), 4);
  }
  
}
float readDAC(int ch)
{
  float voltage;
  voltage = int16ToVoltage(g_DACsetpoint[ch]);
  return voltage*1000.0; // return mV
}

///////////////////
//// SET/RAMP ////
//////////////////

//// ADC ////

void convert_time(InCommand *incommand)
{
  if(incommand->paramcount != 3)//Check correct number of parameters
  {
    syntax_error();
    return;
  }
  uint8_t adcChannel = atoi(incommand->token[1]);
  if(adcChannel >= NUMADCCHANNELS)
  {
    range_error();
    return;
  }

  byte cr;
  float reqtime = atof(incommand->token[2]);
  if(reqtime > 2686)
  {
    reqtime = 2686;
  }
  byte fw = ((byte)(((reqtime*6.144-249)/128)+0.5));
  if(fw < 2)//cannot have less than 2 or ADC will not sample
  {
    fw = 2;
  }

  send_ack();

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

void ramp_smart(InCommand *incommand)  //(channel,setpoint,ramprate)
{
  if(incommand->paramcount != 4)
  {
    syntax_error();
    return;
  }
  uint8_t dacChannel = atoi(incommand->token[1]);
  if(dacChannel >= NUMDACCHANNELS)
  {
    range_error();
    return;
  }
  float setpoint = atof(incommand->token[2]);
  if((abs(setpoint) / 1000.0) > DAC_FULL_SCALE)
  {
    range_error();
    return;
  }
  //float ramprate = DB[3].toFloat();
  float ramprate = atof(incommand->token[3]);
  if(ramprate <= 0.0)
  {
    range_error();
    return;
  }
  float initial = readDAC(dacChannel);  //mV

  send_ack();
  if (abs(setpoint-initial) < 0.0001)  //If already at setpoint
  {
    SERIALPORT.println("RAMP_FINISHED");
    return;
  }
  // Calc ramprate, make vector string, pass to autoRamp1
  int nSteps = static_cast<int>(abs(setpoint-initial)/ramprate*1000);  //using 1ms as delay
  if (nSteps < 5)
  {
    nSteps = 5;
  }
  autoRamp1(initial, setpoint, nSteps, dacChannel, 1000, incommand);
  SERIALPORT.println("RAMP_FINISHED");
  return;
}

void int_ramp(InCommand *incommand)
{
  int i;
    //check for minimum number of parameters
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    return;
  }
  
  if(incommand->paramcount < 6)
  {
    syntax_error();
    return;
  }

  //String channelsDAC = DB[1];
  //g_numrampDACchannels = channelsDAC.length();
  char * channelsDAC = incommand->token[1];
  char * channelsADC = incommand->token[2];
  g_numrampDACchannels = strlen(channelsDAC);  
  g_numrampADCchannels = strlen(channelsADC);

  g_done = false;
  g_stepcount = 0;
  //Do some bounds checking
  if((g_numrampDACchannels > NUMDACCHANNELS) || (g_numrampADCchannels > NUMADCCHANNELS) || (incommand->paramcount != g_numrampDACchannels * 2 + 4))
  {
    syntax_error();
    return;
  }  
  //define DAC channels and check range
  for(i = 0; i < g_numrampDACchannels; i++)
  {
    //g_DACchanselect[i] = channelsDAC[i] - '0';
    g_DACchanselect[i] = channelsDAC[i] - '0';
    if(g_DACchanselect[i] >= NUMDACCHANNELS)
    {
      range_error();
      return;
    }
    float dacstartvoltage = atof(incommand->token[i+3])/1000.0;
    float dacendvoltage = atof(incommand->token[i+3+g_numrampDACchannels])/1000.0;
    if((abs(dacstartvoltage) > DAC_FULL_SCALE) || (abs(dacendvoltage) > DAC_FULL_SCALE))
    {
      range_error();
      return;
    }
    g_DACstartpoint[i] = voltageToInt32(dacstartvoltage);
    g_DACendpoint[i] = voltageToInt32(dacendvoltage);
  }
  //define ADC channels and check range
  for(i = 0; i < g_numrampADCchannels; i++)//Configure ADC channels
  {  
    g_ADCchanselect[i] = channelsADC[i] - '0';
    if(g_ADCchanselect[i] >= NUMADCCHANNELS)
    {
      range_error();
      return;
    }
  }
  //g_numsteps=(DB[g_numrampDACchannels*2+3].toInt());

  g_numsteps = atoi(incommand->token[g_numrampDACchannels*2+3]);
  #ifdef DEBUGRAMP
  SERIALPORT.print("numsteps: ");
  SERIALPORT.println(g_numsteps);
  #endif  
  //configure DAC channels
  for(i = 0; i < g_numrampDACchannels; i++)
  {

    //g_DACramppoint[i] = g_DACstartpoint[i];
    g_DACramppoint[i] = (int64_t)g_DACstartpoint[i] * BIT31;

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
  SPI.transfer(adc, ADC_IO_RDYFN | ADC_IO_SYNC | ADC_IO_P1DIR); //Change RDY to only trigger when all channels complete, and start only when synced, P1 as input

  delayMicroseconds(1000); // wait for DACs to settle
  
  //Check for SYNC again
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    return;
  }
  digitalWrite(data,HIGH);


  send_ack();//send ack and immediately attach interrupt
  
  attachInterrupt(digitalPinToInterrupt(drdy), updatead, FALLING);  

  digitalWrite(adc_trig_out, HIGH); //send sync signal (only matters on master)
  
  while(!g_done)
  {
    if(query_serial(incommand))
    {
      if(strcmp("STOP", incommand->token[0]) == 0)      
      {
        break;
      }     
    }
  }  
  detachInterrupt(digitalPinToInterrupt(drdy));
  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_DEFAULT | ADC_IO_P1DIR); //Change RDY to trigger when any channel complete, set P1 as input
  for(i = 0; i < NUMADCCHANNELS; i++)
  {
  SPI.transfer(adc, ADC_CHSETUP | i);//Access channel setup register
  SPI.transfer(adc, ADC_CHSETUP_RNG10BI);//set +/-10V range and disable for continuous mode
  SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]);   //Access channel mode register
  SPI.transfer(adc, ADC_MODE_IDLE);  //Set ADC to idle
  }
  digitalWrite(data,LOW);
  SERIALPORT.println("RAMP_FINISHED");
}

////////////////////////////
//// SPECTRUM ANALYZER ////
///////////////////////////

void spec_ana(InCommand * incommand)
{
  int i;

  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    return;
  }

  if(incommand->paramcount != 3)
  {
    syntax_error();
    return;
  }
  //String channelsADC = DB[1];
  char * channelsADC = incommand->token[1];
  g_numrampADCchannels = strlen(channelsADC);
  g_numsteps=atoi(incommand->token[2]);

  g_done = false;
  g_stepcount = 0;

  // Do some bounds checking
  if(g_numrampADCchannels > NUMADCCHANNELS)
  {
    syntax_error();
    return;
  }

  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall
  //select ADC channels and check range
  for(i = 0; i < g_numrampADCchannels; i++)
  {
    g_ADCchanselect[i] = channelsADC[i] - '0';
    if(g_ADCchanselect[i] >= NUMADCCHANNELS)
    {
      range_error();
      return;
    }
  }
  for(i = 0; i < g_numrampADCchannels; i++) // Configure ADC channels
  {
    /*
    g_ADCchanselect[i] = incommand->token[1][i] - '0';
    if(g_ADCchanselect[i] >= NUMADCCHANNELS)
    {
      range_error();
      return;
    }
    */
    SPI.transfer(adc, ADC_CHSETUP | g_ADCchanselect[i]);//Access channel setup register
    SPI.transfer(adc, ADC_CHSETUP_RNG10BI | ADC_CHSETUP_ENABLE);//set +/-10V range and enable for continuous mode
    SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]);   //Access channel mode register
    SPI.transfer(adc, ADC_MODE_CONTCONV | ADC_MODE_CLAMP);  //Continuous conversion with clamping
  }

  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_RDYFN | ADC_IO_SYNC | ADC_IO_P1DIR); //Change RDY to only trigger when all channels complete, and wait for sync, P1 as input

  //Check for SYNC again
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    return;
  }
  
  digitalWrite(data,HIGH);
  
  send_ack();//send ack and immediately attach interrupt

  attachInterrupt(digitalPinToInterrupt(drdy), writetobuffer, FALLING);  

  digitalWrite(adc_trig_out, HIGH); //send sync signal (only matters on master)
  
  while(!g_done)
  {
    if(query_serial(incommand))
    {
      if(strcmp("STOP", incommand->token[0]) == 0)
      {
        break;
      }     
    }
  }  

  detachInterrupt(digitalPinToInterrupt(drdy));
  SPI.transfer(adc, ADC_IO); // Write to ADC IO register
  SPI.transfer(adc, ADC_IO_DEFAULT | ADC_IO_P1DIR); // Change RDY to trigger when any channel complete
  for(i = 0; i < NUMADCCHANNELS; i++)
  {
  SPI.transfer(adc, ADC_CHSETUP | i); // Access channel setup register
  SPI.transfer(adc, ADC_CHSETUP_RNG10BI); // set +/-10V range and disable for continuous mode
  SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]); // Access channel mode register
  SPI.transfer(adc, ADC_MODE_IDLE); // Set ADC to idle
  }
  digitalWrite(data,LOW);
  SERIALPORT.println("READ_FINISHED");
}

void writetobuffer()
{
   int16_t i;
   if(!g_done)
   {
#ifdef OPTICAL //no buffer required for regular UART (optical)
     if(g_stepcount > 0) //Discard first sample to stay in sync with ramp commands
     {
       for(i = 0; i < g_numrampADCchannels; i++)
       {
          SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
          SERIALPORT.write(SPI.transfer(adc, 0, SPI_CONTINUE)); // Read/write first byte
          SERIALPORT.write(SPI.transfer(adc, 0)); // Read/write second byte           
       }
     }
     else
     {
       for(i = 0; i < g_numrampADCchannels; i++) //discard first loop
       {
          SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
          SPI.transfer(adc, 0, SPI_CONTINUE); // Read/write first byte
          SPI.transfer(adc, 0); // Read/write second byte
       }
     }

     g_stepcount++;
     if(g_stepcount > g_numsteps)
     {
        g_done = true;
     }
#else
      for(i = 0; i < g_numrampADCchannels; i++)
      {
         SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); // Read channel data register
         g_USBbuff[g_buffindex] = SPI.transfer(adc, 0, SPI_CONTINUE); // Reads first byte
         g_USBbuff[g_buffindex + 1] = SPI.transfer(adc, 0); // Reads second byte
         g_buffindex += 2;
      }

      if(g_stepcount < 1)//first loop has to be discarded, so just overwrite buffer
      {
        g_buffindex = 0;
      }
      g_stepcount++;

      if (g_buffindex >= USBBUFFSIZE)
      {
        SERIALPORT.write((char*)g_USBbuff, g_buffindex);
        g_buffindex = 0;
      }

      if(g_stepcount >= g_numsteps)
      {
        if(g_buffindex > 0)
        {
          SERIALPORT.write((char*)g_USBbuff, g_buffindex);
          g_buffindex = 0;
        }
        g_done = true;
      }
#endif      
   }
}

////////////////////////////
//// PID Correction    ////
///////////////////////////

//Start PID, currently DAC0 and ADC0, no inputs params, just starts
void start_pid(InCommand *incommand)
{
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0)
  {
    return;
  }
  
  g_pidparam[0].dacout = readDAC(g_pidparam[0].DACchan);//Start from current dac setpoint
  g_pidparam[0].dacoutlim = g_pidparam[0].dacout;
  
  g_pidparam[0].sampletime = readADCConversionTime(g_pidparam[0].ADCchan);//Get PID sample time in microsec from adc channel conversion time
  
  pid0.SetSampleTime(g_pidparam[0].sampletime);

  slew_pid_set(g_pidparam[0].slewlimit);//has to occur after setting sample time
  
  if(g_pidparam[0].forward_dir == false)
  {
    pid0.SetControllerDirection(REVERSE);
  }
  else
  {
    pid0.SetControllerDirection(DIRECT);
  }
  send_ack();
  pid0.SetMode(AUTOMATIC);
  pid0.SetOutputLimits(g_pidparam[0].dacmin, g_pidparam[0].dacmax);
  
  SPI.transfer(adc, ADC_CHSETUP | g_pidparam[0].ADCchan);//Access channel setup register
  SPI.transfer(adc, ADC_CHSETUP_RNG10BI | ADC_CHSETUP_ENABLE);//set +/-10V range and enable for continuous mode
  SPI.transfer(adc, ADC_CHMODE | g_pidparam[0].ADCchan);   //Access channel mode register
  SPI.transfer(adc, ADC_MODE_CONTCONV | ADC_MODE_CLAMP);  //Continuous conversion with clamping
  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_RDYFN | ADC_IO_SYNC | ADC_IO_P1DIR); //Change RDY to only trigger when all channels complete, and start only when synced, P1 as input

  attachInterrupt(digitalPinToInterrupt(drdy), pidint, FALLING);

  
}
//PID Interrupts every ADC sample
void pidint(void)
{
  uint8_t b1, b2;
  int decimal;
  int16_t i;

  SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_pidparam[0].ADCchan, SPI_CONTINUE); //Read channel data register
  b1 = (SPI.transfer(adc, 0, SPI_CONTINUE)); // Read first byte
  b2 = SPI.transfer(adc, 0); // Read second byte
  
  decimal = twoByteToInt(b1, b2);
  g_pidparam[0].adcin = map2(decimal, 0, 65536, -10000.0, 10000.0);
    
  if(pid0.Compute())
  {    
    if(g_pidparam[0].dacout > (g_pidparam[0].dacoutlim + g_pidparam[0].slewcycle))
    {
      g_pidparam[0].dacoutlim += g_pidparam[0].slewcycle;
    }
    else if(g_pidparam[0].dacout < (g_pidparam[0].dacoutlim - g_pidparam[0].slewcycle))
    {
      g_pidparam[0].dacoutlim -= g_pidparam[0].slewcycle;
    }
    else 
    {
      g_pidparam[0].dacoutlim = g_pidparam[0].dacout;
    }
    writeDAC(g_pidparam[0].DACchan, g_pidparam[0].dacoutlim, true);
    g_pidparam[0].loopcount++;
    if(g_pidparam[0].loopcount >= 10)
    {
      g_pidparam[0].loopcount = 0;      
      //SERIALPORT.print("IN: ");      
      float seradc = g_pidparam[0].adcin;
      float serdac = g_pidparam[0].dacoutlim;      
      byte * out1 = (byte *) &seradc;
      byte * out2 = (byte *) &serdac;
      //byte * b1 = (byte *) g_pidparam[0].adcin;      
      SERIALPORT.write(out1,4);   
      SERIALPORT.write(out2,4);
      SERIALPORT.write(0xA5);
      SERIALPORT.write(0x5A);

    }
    
  }
}

//Stop PID, no inputs params, just stops 
void stop_pid(InCommand * incommand)
{
  uint8_t i = 0;
  pid0.SetMode(MANUAL);
  detachInterrupt(digitalPinToInterrupt(drdy));
  
  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_DEFAULT | ADC_IO_P1DIR); //Change RDY to trigger when any channel complete, set P1 as input
  for(i = 0; i < NUMADCCHANNELS; i++)
  {
  SPI.transfer(adc, ADC_CHSETUP | i);//Access channel setup register
  SPI.transfer(adc, ADC_CHSETUP_RNG10BI);//set +/-10V range and disable for continuous mode
  SPI.transfer(adc, ADC_CHMODE | i);   //Access channel mode register
  SPI.transfer(adc, ADC_MODE_IDLE);  //Set ADC to idle
  }
  send_ack();
}
//SET_PID_TUNE,<P-coeff>,<I-coeff>,<D-coeff>

void set_pid_tune(InCommand * incommand)
{

  if(incommand->paramcount != 4)
  {
    syntax_error();
    return;
  }
  if(pid0.GetMode() == MANUAL)
  {
    send_ack();
  }
  
  //g_pidparam[0].kp = DB[1].toFloat();
  //g_pidparam[0].ki = DB[2].toFloat();
  //g_pidparam[0].kd = DB[3].toFloat();
  
  g_pidparam[0].kp = atof(incommand->token[1]);
  g_pidparam[0].ki = atof(incommand->token[2]);
  g_pidparam[0].kd = atof(incommand->token[3]);
  pid0.SetTunings(g_pidparam[0].kp, g_pidparam[0].ki, g_pidparam[0].kd);    
}

//SET_PID_SETP,<Setpoint in mV>
void set_pid_setp(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  if(pid0.GetMode() == MANUAL)
  {
    send_ack();
  }
  //g_pidparam[0].setpoint = DB[1].toFloat();  
  g_pidparam[0].setpoint = atof(incommand->token[1]);
}

//SET_PID_LIMS,<Lower limit in mV>,<Upper limit in mV>
void set_pid_lims(InCommand *incommand)
{
  if(incommand->paramcount != 3)
  {
    syntax_error();
    return;
  }
  
  //g_pidparam[0].dacmin = DB[1].toFloat();
  //g_pidparam[0].dacmax = DB[2].toFloat();
  g_pidparam[0].dacmin = atof(incommand->token[1]);
  g_pidparam[0].dacmax = atof(incommand->token[2]);
  if((abs(g_pidparam[0].dacmin) / 1000.0 > DAC_FULL_SCALE) || (abs(g_pidparam[0].dacmax) / 1000.0 > DAC_FULL_SCALE))
  {
    range_error();
    return;
  }
  if(g_pidparam[0].dacmin > g_pidparam[0].dacmax)
  {
    range_error();
    return;
  }
  if(pid0.GetMode() == MANUAL)
  {
    send_ack();
  }
  pid0.SetOutputLimits(g_pidparam[0].dacmin, g_pidparam[0].dacmax);    
}

//SET_PID_DIR,<1 for forward, 0 for reverse>
void set_pid_dir(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }

  if(pid0.GetMode() == MANUAL)
  {
    send_ack();
  }

  if(atoi(incommand->token[1]) == 0)
  {
    g_pidparam[0].forward_dir = false;
    pid0.SetControllerDirection(REVERSE);
  }
  else
  {
    g_pidparam[0].forward_dir = true;
    pid0.SetControllerDirection(DIRECT);
  }
}

//SET_PID_SLEW,<maximum slewrete in mV per second>

void set_pid_slew(InCommand * incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  float slew = atof(incommand->token[1]);
  if(slew < 0.0)
  {
    range_error();
  }
  if(pid0.GetMode() == MANUAL)
  {
    send_ack();
  }
  slew_pid_set(slew);
}

void slew_pid_set(float slewlimit)
{
  if(slewlimit < 0)
  {
    return;
  }    
  g_pidparam[0].slewlimit = slewlimit;
  g_pidparam[0].slewcycle = (g_pidparam[0].slewlimit * g_pidparam[0].sampletime) / 1000000.0; //divide for microseconds
}

//////////////////////
//// CALIBRATION ////
/////////////////////


void default_cal(InCommand *incommand)
{
  if(incommand->paramcount != 1)
  {
    syntax_error();
    return;
  }
  send_ack();
  //loaddefaultcals(); // set default calibration
  loaddaccals(); // set default calibration
  SERIALPORT.println("CALIBRATION_CHANGED");
}

void loaddaccals()
{
  uint8_t ch = 0;
  /*
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    writeADCchzeroscale(ch, adczeroscalecal[ch]);
    writeADCchfullscale(ch, adcfullscalecal[ch]);
  }
  */
  for(ch = 0; ch < NUMDACCHANNELS; ch++)
  {
    int8_t offset, gain;
		readeepromdaccal(ch, &offset, &gain, false);
    writeDACoffset(ch, offset);
    writeDACgain(ch, gain);
  }
}

//// ADC ////

void cal_adc_with_dac(InCommand *incommand)
{
  uint8_t ch = 0;
  if(sync_check(CHECK_CLOCK) != 0)//make sure ADC has a clock
  {
    return;
  }
  if(incommand->paramcount != 1)
  {
    syntax_error();
    return;
  }
  send_ack();
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    writeDAC(ch, 0.0, true); // mV
  }
  delayMicroseconds(2000); // wait 2 ms
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    SERIALPORT.print("ch");
    SERIALPORT.print(ch);
    SERIALPORT.print(",");
    SERIALPORT.print(cal_adc_ch_zero_scale(ch));
    SERIALPORT.print(",");
  }

  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    writeDAC(ch, 10000.0, true); // mV
  }
  delayMicroseconds(2000); // wait 2ms
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    SERIALPORT.print("ch");
    SERIALPORT.print(ch);
    SERIALPORT.print(",");
    SERIALPORT.print(cal_adc_ch_full_scale(ch));
    if(ch < (NUMADCCHANNELS - 1))
    {
      SERIALPORT.print(",");      
    }
    
  }
  SERIALPORT.println();
  //SERIALPORT.println("CALIBRATION_FINISHED");
}

void adc_zero_sc_cal(InCommand *incommand)
{
  if(sync_check(CHECK_CLOCK) != 0)//make sure ADC has a clock
  {
    return;
  }
  if(incommand->paramcount != 1)
  {
    syntax_error();
    return;
  }
  send_ack();
  cal_adc_zero_scale();
  SERIALPORT.println("CALIBRATION_FINISHED");
}

void cal_adc_zero_scale(void)
{
  SPI.transfer(adc, ADC_CHMODE);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE);       // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SELFZEROCAL);       // Enter system zero-scale cal mode
  waitDRDY();
}

void adc_ch_zero_sc_cal(InCommand *incommand)
{
  if(sync_check(CHECK_CLOCK) != 0)//make sure ADC has a clock
  {
    return;
  }
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  uint8_t ch = atoi(incommand->token[1]);
  if(ch >= NUMADCCHANNELS)
  {
    range_error();
    return;
  }
  
  send_ack();
  uint32_t calvalue = cal_adc_ch_zero_scale(ch);
  SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.println(calvalue);
  //SERIALPORT.println("CALIBRATION_FINISHED");
}

uint32_t cal_adc_ch_zero_scale(int ch)
{

  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE);       // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SYSZEROCAL);       // Enter system zero-scale cal mode
  waitDRDY();
  return readADCzerocal(ch);
}

uint32_t readADCzerocal(byte ch)
{
  uint8_t b1, b2, b3;
  uint32_t calvalue;
  //int n;
  //String buffer;
  //char buffertemp [100];
  SPI.transfer(adc, ADC_CHZEROSCALECAL | ADC_REGREAD | ch);   // Access ch zero-scale cal register in read mode
  b1 = SPI.transfer(adc,0x00);   // read byte 1
  b2 = SPI.transfer(adc,0x00);   // read byte 2
  b3 = SPI.transfer(adc,0x00);   // read byte 3

  calvalue = b1 << 16;
  calvalue += b2 << 8;
  calvalue += b3;

  //n = snprintf(buffertemp,100,"ch%d,%d",ch,calvalue);
  //buffer = buffertemp;
  return calvalue;
}

void adc_ch_full_sc_cal(InCommand *incommand)
{
  if(sync_check(CHECK_CLOCK) != 0)//make sure ADC has a clock
  {
    return;
  }
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  uint8_t ch = atoi(incommand->token[1]);
  if(ch >= NUMADCCHANNELS)
  {
    range_error();
    return;
  }
  send_ack();
  uint32_t calvalue = cal_adc_ch_full_scale(ch);
  SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.println(calvalue);
  //SERIALPORT.println("CALIBRATION_FINISHED");
}

uint32_t cal_adc_ch_full_scale(uint8_t ch)
{
  //Put ch in idle mode
  SPI.transfer(adc, ADC_CHMODE | ch); // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_IDLE); // Enter idle mode

  SPI.transfer(adc, ADC_CHMODE | ch); // Access ch mode register in write mode
  SPI.transfer(adc, ADC_MODE_SYSFULLCAL); // Enter system full-scale cal mode
  waitDRDY();

  return readADCfullcal(ch);
}

uint32_t readADCfullcal(uint8_t ch)
{
  uint8_t b1, b2, b3;
  uint32_t calvalue;
  
  SPI.transfer(adc, ADC_CHFULLSCALECAL | ADC_REGREAD | ch);   // Access ch full-scale cal register in read mode
  b1 = SPI.transfer(adc,0x00);   // read byte 1
  b2 = SPI.transfer(adc,0x00);   // read byte 2
  b3 = SPI.transfer(adc,0x00);   // read byte 3

  calvalue = b1 << 16;
  calvalue += b2 << 8;
  calvalue += b3;

  return calvalue;
}

void read_adc_cal(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  uint8_t ch = atoi(incommand->token[1]);
  if(ch >= NUMADCCHANNELS)
  {
    range_error();
    return;
  }
  send_ack();  
  SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.print(readADCzerocal(ch));
  SERIALPORT.print(",");
  SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.println(readADCfullcal(ch));
  //SERIALPORT.println("READ_FINISHED");
}

void write_adc_cal(InCommand *incommand)
{
  if(incommand->paramcount != 4)
  {
    syntax_error();
    return;
  }
  uint8_t ch = atoi(incommand->token[1]);
  if(ch >= NUMADCCHANNELS)
  {
    range_error();
    return;
  }
  uint32_t zerocal = atoi(incommand->token[2]);
  uint32_t fullcal = atoi(incommand->token[3]); 

  if(zerocal > 0xFFFFFF)  
  {
    range_error();
    return;
  }
  if(fullcal > 0xFFFFFF)  
  {
    range_error();
    return;
  }   
  send_ack();
  writeADCcal(ch, zerocal, fullcal);
  SERIALPORT.println("CALIBRATION_CHANGED");
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

void full_scale(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  float dacscale = atof(incommand->token[1]);
  if(dacscale < 0.0)
  {
    range_error();
    return;
  }
  send_ack();
  DAC_FULL_SCALE = dacscale;
  SERIALPORT.println("FULL_SCALE_UPDATED");
}

void dac_offset_adj(InCommand *incommand)
{
  if(incommand->paramcount != 3)
  {
    syntax_error();
    return;
  }
  uint8_t ch = atoi(incommand->token[1]);
  if(ch >= NUMDACCHANNELS)
  {
    range_error();
    return;
  }
  float offset = atof(incommand->token[2]);
  if((offset > (DAC_FULL_SCALE * 2.0 * 16 / 65536.0)) || (offset < (DAC_FULL_SCALE * -2.0 * 15.875 / 65536.0)))
  {
    range_error();
    return;
  }
  send_ack();
  calDACoffset(ch, offset);
  //SERIALPORT.println("CALIBRATION_FINISHED");
}

void calDACoffset(byte ch, float offset)
{
  int8_t numsteps;
  float stepsize = (DAC_FULL_SCALE * 2.0) / (65536.0 * 8.0); //stepsize is 1/8 of a 16-bit LSB

  if(offset < 0)
  {
    numsteps = (int8_t)((offset / stepsize) - 0.5) * -1;
  }
  else
  {
    numsteps = (int8_t)((offset / stepsize) + 0.5) * -1;
  }

  SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.print(stepsize * 1000000);
  SERIALPORT.print(",");
  SERIALPORT.println(numsteps);  
  writeDACoffset(ch, numsteps);
}

void dac_gain_adj(InCommand *incommand)
{
  if(incommand->paramcount != 3)
  {
    syntax_error();
    return;
  }
  uint8_t ch = atoi(incommand->token[1]);
  if(ch >= NUMDACCHANNELS)
  {
    range_error();
    return;
  }
  float offset = atof(incommand->token[2]);
  if((offset > (DAC_FULL_SCALE * 2.0 * 16 / 65536.0)) || (offset < (DAC_FULL_SCALE * -2.0 * 15 / 65536.0)))
  {
    range_error();
    return;
  }
  send_ack();
  calDACgain(ch, offset);
  //SERIALPORT.println("CALIBRATION_FINISHED");
}

void calDACgain(byte ch, float offset)
// offset is measured relative to ideal negative full scale voltage (usually -10V)
{
  int8_t numsteps;
  //int n;
  //String buffer;
  //char buffertemp [100];
  float stepsize = (DAC_FULL_SCALE * 2.0) / (65536.0 * 2.0); //stepsize is 1/2 of a 16-bit LSB
  numsteps = (int8_t)(offset / stepsize);

  if(offset < 0)
  {
    numsteps = (int8_t)((offset / stepsize) - 0.5);
  }
  else
  {
    numsteps = (int8_t)((offset / stepsize) + 0.5);
  }

  //n = snprintf(buffertemp,100,"ch%d,%f,%d",ch,stepsize*1000000,numsteps);
  //buffer = buffertemp;
  SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.print(stepsize * 1000000);
  SERIALPORT.print(",");
  SERIALPORT.println(numsteps);  
  
  writeDACgain(ch, numsteps);
}


void dac_reset_cal(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  uint8_t ch = atoi(incommand->token[1]);
  if(ch >= NUMDACCHANNELS)
  {
    range_error();
    return;
  } 
  send_ack();
  dac_ch_reset_cal(ch);
  SERIALPORT.println("CALIBRATION_RESET");
}

void dac_ch_reset_cal(uint8_t ch)
{
  writeDACoffset(ch, 0);
  writeDACgain(ch, 0);
}

void writeDACoffset(int ch, int8_t steps)
{
  int thisdac;
  int thisldac;

	dacocal[ch] = steps;
  convertDACch(&ch, &thisdac, &thisldac);
	//SERIALPORT.println(steps);
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

	dacgcal[ch] = steps;
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

void send_ack(void)
{
#ifdef SENDACK  
  SERIALPORT.println("ACK");
#endif
}

void syntax_error(void)
{
  SERIALPORT.println("SYNTAX_ERROR");
}

void range_error(void)
{
  SERIALPORT.println("RANGE_ERROR");
}

void idn()
// return IDN string
{
  char idstring[EEPROM_ID_LEN];
  send_ack();
  readeepromid(idstring);
  SERIALPORT.print(IDSTRING);
  SERIALPORT.print("_UNIT-");
  SERIALPORT.print(idstring);
  //SERIALPORT.print(UNITNUM);
  SERIALPORT.print("_");
  SERIALPORT.println(FW_VER);  
}

void rdy()
// retrun "READY"
{
  send_ack();
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

int freeMemory() 
{
  char top;
#ifdef __arm__  
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
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
  else
  {
    return -999999.9;
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
  send_ack();
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
   int16_t i;
   if(!g_done)
   {
      ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
      ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);
      
#ifdef OPTICAL //no buffering with regular UART (optical)
      if(g_stepcount > 0) //first sample comes in on second loop through the interrupt
      {
        for(i = 0; i < g_numrampADCchannels; i++)
        {
           SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
           SERIALPORT.write(SPI.transfer(adc, 0, SPI_CONTINUE)); // Read/write first byte
           SERIALPORT.write(SPI.transfer(adc, 0)); // Read/write second byte           
        }
      }
      else
      {
        for(i = 0; i < g_numrampADCchannels; i++) //discard first loop
        {
           SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
           SPI.transfer(adc, 0, SPI_CONTINUE); // Read/write first byte
           SPI.transfer(adc, 0); // Read/write second byte
        }
      }
#else      
      for(i = 0; i < g_numrampADCchannels; i++)
      {
         SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
         g_USBbuff[g_buffindex] = SPI.transfer(adc, 0, SPI_CONTINUE); // Reads first byte
         g_USBbuff[g_buffindex + 1] = SPI.transfer(adc, 0); // Reads second byte
         g_buffindex += 2;
      }

      if(g_stepcount < 1)//first loop has to be discarded, so just overwrite buffer
      {
        g_buffindex = 0;
      }
#endif
      
      g_stepcount++;

#ifdef OPTICAL
      if(g_stepcount > g_numsteps)
      {        
        g_done = true;
      }
#else      
      if (g_buffindex >= USBBUFFSIZE)
      {
        SERIALPORT.write((char*)g_USBbuff, g_buffindex);
        g_buffindex = 0;
      }

      if(g_stepcount > g_numsteps)
      {
        if(g_buffindex > 0)
        {
          SERIALPORT.write((char*)g_USBbuff, g_buffindex);
          g_buffindex = 0;
        }
        g_done = true;
      }
#endif      
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

void autoRamp1(float v1, float v2, uint32_t nSteps, uint8_t dacChannel, uint32_t period, InCommand *incommand)
// voltage in mV
{
  digitalWrite(data,HIGH);
  uint32_t timer = micros();
  int j = 0;
  while(j < nSteps)
  {
    uint32_t nowmicros = micros();
    digitalWrite(data, HIGH);
    if((nowmicros - timer) > period)//Time to take another step
    {
      timer = nowmicros;
      writeDAC(dacChannel, v1+(v2-v1)*j/(nSteps-1), true); // takes mV
      j++;
    }
    //Check for STOP command   
    if(query_serial(incommand))
    {
      if(strcmp("STOP", incommand->token[0]) == 0)
      {
        break;
      }
    }
  }
  digitalWrite(data,LOW);
}

float writeDAC(uint8_t dacChannel, float voltage, bool load)
// voltage in mV
{
  float actualvoltage;
  if(dacChannel >= NUMDACCHANNELS)
  {
    range_error();
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
  voltreturn = readDAC(ch);
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
  if (voltage > DAC_FULL_SCALE || voltage < -1*DAC_FULL_SCALE)
  {
    error();
    return 0;
  }
  //return (voltage*32768)/DAC_FULL_SCALE;
  
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

//// SYNC UTIL ////

void set_mode(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  if(strcmp("MASTER",incommand->token[1]) == 0)
  {
    send_ack();
    digitalWrite(slave_master, LOW);
    g_ms_select = MASTER;
    SERIALPORT.println("MASTER_SET");  
  }
  else if(strcmp("SLAVE",incommand->token[1]) == 0)
  {
    send_ack();
    digitalWrite(slave_master, HIGH);
    g_ms_select = SLAVE;
    SERIALPORT.println("SLAVE_SET");  
  }
  else if((strcmp("INDEP",incommand->token[1]) == 0))
  {
    send_ack();
    digitalWrite(slave_master , LOW);
    g_ms_select = INDEP;
    SERIALPORT.println("INDEP_SET");
  }
  else
  {
    syntax_error();
    return;
  }    
}

void arm_sync(InCommand *incommand)
{
  if(incommand->paramcount != 1)
  {
    syntax_error();
    return;
  }
  send_ack();
  digitalWrite(adc_trig_out, LOW);
  SERIALPORT.println("SYNC_ARMED");
}

void disarm_sync(InCommand *incommand)
{
  if(incommand->paramcount != 1)
  {
    syntax_error();
    return;
  }
  send_ack();
  digitalWrite(adc_trig_out, HIGH);
  SERIALPORT.println("SYNC_DISARMED");
}

void check_sync(InCommand *incommand)
{
  if(incommand->paramcount != 1)
  {
    syntax_error();
    return;
  }
  send_ack();
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) == 0)
  {
    SERIALPORT.println("CLOCK_SYNC_READY");    
  }
}

uint8_t sync_check(uint8_t mask) //Checks for valid clock and sync signal, returns 0 if ok, 1-3 if not: 1 (clock not ok) or'ed with 2 (sync_ok)
{
  uint8_t syncstatus = 0;
#ifdef OPTICAL //only check clock and sync with optical version
  if(!g_clock_synced && (mask & CHECK_CLOCK))
  {
    syncstatus |= 1;
    SERIALPORT.println("CLOCK_NOT_READY");
  }
  if(g_ms_select != INDEP)
  {
    if((digitalRead(adc_trig_in) == true) && (mask & CHECK_SYNC))
    {
      syncstatus |= 2;
      SERIALPORT.println("SYNC_NOT_READY");
    }
  }
#endif 
  return syncstatus;
}

//// AWG RAMP ////

//ADD_WAVE,<wave number, 0 indexed)>,<Setpoint 0 in mV>,<Number of samples at setpoint 0>,….<Setpoint n in mV>,<Number of samples at Setpoint n>
//Maximum ~20 Setpoints per call, due to possible serial buffer overrun
void add_wave(InCommand *incommand)
{
  if(incommand->paramcount < 4)
  {
    syntax_error();
    return;
  }
  //uint8_t wavenumber = DB[1].toInt();
  uint8_t wavenumber = atoi(incommand->token[1]);
  int32_t numsamples;
  if(wavenumber >= AWGMAXWAVES)
  {
    SERIALPORT.print("ERROR, Max waveforms = ");
    SERIALPORT.println(AWGMAXWAVES);
    return;
  }
  //uint32_t numsetpoints = (DB.size() - 2) / 2;
  uint32_t numsetpoints = (incommand->paramcount - 2) / 2;
  
  if((numsetpoints + g_awgwave[wavenumber].numsetpoints) > AWGMAXSETPOINTS)
  {
    SERIALPORT.print("ERROR, Max setpoints = ");
    SERIALPORT.println(AWGMAXSETPOINTS);
    return;
  }
  //check for voltage and setpoint range errors before adding the wave
  for(uint32_t i = 0; i < numsetpoints; i++)
  {
    float setvolt = atof(incommand->token[(i * 2) + 2]) / 1000.0;
    if(abs(setvolt) > DAC_FULL_SCALE)
    {
      range_error();
      return;
    }
    numsamples = atoi(incommand->token[(i * 2) + 3]);
    if(numsamples < 1)
    {
      range_error();
      return;
    }
  }
  //all good, add the wave
  for(uint32_t i = 0; i < numsetpoints; i++)
  {
    //g_awgwave[wavenumber].setpoint[i + g_awgwave[wavenumber].numsetpoints] = voltageToInt16(DB[(i * 2) + 2].toFloat() / 1000.0);
    g_awgwave[wavenumber].setpoint[i + g_awgwave[wavenumber].numsetpoints] = voltageToInt16(atof(incommand->token[(i * 2) + 2]) / 1000.0);
    //numsamples = DB[(i * 2) + 3].toInt();
    g_awgwave[wavenumber].numsamples[i + g_awgwave[wavenumber].numsetpoints] = atoi(incommand->token[(i * 2) + 3]);
  }
  send_ack();
  g_awgwave[wavenumber].numsetpoints += numsetpoints;
  SERIALPORT.print("WAVE,");
  SERIALPORT.print(wavenumber);
  SERIALPORT.print(",");
  SERIALPORT.println(g_awgwave[wavenumber].numsetpoints);
#ifdef DEBUGRAMP  
  for(uint32_t i = 0; i < g_awgwave[wavenumber].numsetpoints; i++)
  {
    SERIALPORT.print("Setpoint ");
    SERIALPORT.print(i);
    SERIALPORT.print(" = ");
    SERIALPORT.print(g_awgwave[wavenumber].setpoint[i]);
    SERIALPORT.print(" Samplecount = ");
    SERIALPORT.println(g_awgwave[wavenumber].numsamples[i]);
  }
  
  SERIALPORT.print(F("Free RAM = ")); //F function does the same and is now a built in library, in IDE > 1.0.0
  SERIALPORT.println(freeMemory(), DEC);  // print how much RAM is available.
#endif
}


//CLR_WAVE,<wave number>
//Returns WAVE,<wave number>,0
void clr_wave(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  //uint8_t wavenumber = DB[1].toInt();
  uint8_t wavenumber = atoi(incommand->token[1]);
  if(wavenumber >= AWGMAXWAVES)
  {
    //SERIALPORT.print("ERROR, Max waveforms = ");
    //SERIALPORT.println(AWGMAXWAVES);
    range_error();
    return;
  }
  send_ack();
  g_awgwave[wavenumber].numsetpoints = 0;
  
  SERIALPORT.print("WAVE,");
  SERIALPORT.print(wavenumber);
  SERIALPORT.println(",0");   
  
}

//CHECK_WAVE,<wave number>
//Returns WAVE,<wave number>,<total number of setpoints>,<total number of samples>
void check_wave(InCommand *incommand) 
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  //uint8_t wavenumber = DB[1].toInt();
  uint8_t wavenumber = atoi(incommand->token[1]);
  uint32_t totalsamples = 0;  
  if(wavenumber >= AWGMAXWAVES)
  {
    range_error();
    //SERIALPORT.print("ERROR, Max waveforms = ");
    //SERIALPORT.println(AWGMAXWAVES);
    return;
  }
  for(uint32_t i = 0; i < g_awgwave[wavenumber].numsetpoints; i++)
  {
    totalsamples += (g_awgwave[wavenumber].numsamples[i]);
  }
  send_ack();
  SERIALPORT.print("WAVE,");
  SERIALPORT.print(wavenumber);
  SERIALPORT.print(",");
  SERIALPORT.print(g_awgwave[wavenumber].numsetpoints);
  SERIALPORT.print(",");
  SERIALPORT.println(totalsamples);  
}


//AWG_RAMP,<numwaves>,<dacs waveform 0>,<dacs waveform n>,<dacs to ramp>,<adcs>,<initial dac voltage 1>,<…>,<initial dac voltage n>,
//<final dac voltage 1>,<…>,<final dac voltage n>,<# of waveform repetitions at each ramp step>,<# of ramp steps>
void awg_ramp(InCommand *incommand)
{
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0)
  {
    return;
  }
  //Do some initial bounds checking
  if(incommand->paramcount < 9)
  {
    range_error();
    return;
  }
  int i, j;
  //g_numwaves = DB[1].toInt(); //First parameter
  g_numwaves = atoi(incommand->token[1]); //First parameter
  if(g_numwaves > AWGMAXWAVES)
  {
    range_error();
    //SERIALPORT.print("ERROR, Max waveforms = ");
    //SERIALPORT.println(AWGMAXWAVES);
    return;
  }
  for(i = 0; i < g_numwaves; i++)
  {
    //String channelswave = DB[i + 2];
    char * channelswave = incommand->token[i + 2];
    //g_awgwave[i].numDACchannels = channelswave.length();
    g_awgwave[i].numDACchannels = strlen(channelswave);
    g_awgwave[i].samplecount = 0;
    g_awgwave[i].setpointcount = 0;
    for(j = 0; j < g_awgwave[i].numDACchannels; j++)
    {
      //g_awgwave[i].DACchanselect[j] = channelswave[j] - '0';
      uint8_t wavedac = channelswave[j] - '0';
      if(wavedac >= NUMDACCHANNELS)
      {
        range_error();
        return;
      }
      g_awgwave[i].DACchanselect[j] = wavedac;
    }
  }

#ifdef DEBUGRAMP
  SERIALPORT.print("Numwaves = ");
  SERIALPORT.println(g_numwaves);
  for(i = 0; i < g_numwaves; i++)
  {
    SERIALPORT.print("Wave ");
    SERIALPORT.print(i);
    SERIALPORT.print(" DAC Channels: ");
    for(j = 0; j < g_awgwave[i].numDACchannels; j++)
    {
      SERIALPORT.print(g_awgwave[i].DACchanselect[j]);
      SERIALPORT.print(" ");
    }
    SERIALPORT.println(" ");
  }
#endif
       
  //String channelsDAC = DB[g_numwaves + 2];
  //g_numrampDACchannels = channelsDAC.length();
  char * channelsDAC = incommand->token[g_numwaves + 2];
  g_numrampDACchannels = strlen(channelsDAC);

  //String channelsADC = DB[g_numwaves + 3];
  //g_numrampADCchannels = channelsADC.length();
  char * channelsADC = incommand->token[g_numwaves + 3];
  g_numrampADCchannels = strlen(channelsADC);

  g_done = false;
  g_firstsamples = true;
  
  //Do some bounds checking
  //if((g_numrampDACchannels > NUMDACCHANNELS) || (g_numrampADCchannels > NUMADCCHANNELS) || ((uint16_t)DB.size() != g_numrampDACchannels * 2 + 6 + g_numwaves))
  if((g_numrampDACchannels > NUMDACCHANNELS) || (g_numrampADCchannels > NUMADCCHANNELS) || ((uint16_t)incommand->paramcount != g_numrampDACchannels * 2 + 6 + g_numwaves))
  {
    syntax_error();
    //SERIALPORT.println("SYNTAX ERROR");
    return;
  }  

  g_loopcount = 0;
  g_stepcount = 0;
  g_nextloop = false;
   
  //g_numloops=(DB[g_numrampDACchannels*2+4+g_numwaves].toInt());
  g_numloops=atoi(incommand->token[g_numrampDACchannels*2+4+g_numwaves]);
  //g_numsteps=(DB[g_numrampDACchannels*2+5+g_numwaves].toInt());
  g_numsteps=atoi(incommand->token[g_numrampDACchannels*2+5+g_numwaves]);

#ifdef DEBUGRAMP
  SERIALPORT.print("Numloops: ");  
  SERIALPORT.println(g_numloops);
  SERIALPORT.print("Numsteps: ");  
  SERIALPORT.println(g_numsteps);
#endif
  //define DAC channels and check range  
  for(i = 0; i < g_numrampDACchannels; i++)
  {
    g_DACchanselect[i] = channelsDAC[i] - '0';
    if(g_DACchanselect[i] >= NUMDACCHANNELS)
    {
      range_error();
      return;
    }
    float dacstartvoltage = atof(incommand->token[i+4+g_numwaves])/1000.0;
    float dacendvoltage = atof(incommand->token[i+4+g_numwaves+g_numrampDACchannels])/1000.0;
    if((abs(dacstartvoltage) > DAC_FULL_SCALE) || (abs(dacendvoltage) > DAC_FULL_SCALE))
    {
      range_error();
      return;
    }
    g_DACstartpoint[i] = voltageToInt32(dacstartvoltage);
    g_DACendpoint[i] = voltageToInt32(dacendvoltage);   
  }
  
  //define ADC channels and check range
  for(i = 0; i < g_numrampADCchannels; i++)//Configure ADC channels
  {  
    g_ADCchanselect[i] = channelsADC[i] - '0';
    if(g_ADCchanselect[i] >= NUMADCCHANNELS)
    {
      range_error();
      return;
    }
  }

  //configure DAC channels
  for(i = 0; i < g_numrampDACchannels; i++)
  {
    //g_DACchanselect[i] = channelsDAC[i] - '0';
    //g_DACstartpoint[i] = voltageToInt32(DB[i+4+g_numwaves].toFloat()/1000.0);
    //g_DACstartpoint[i] = voltageToInt32(atof(incommand->token[i+4+g_numwaves])/1000.0);

    g_DACramppoint[i] = (int64_t)g_DACstartpoint[i] * BIT31;
    //g_DACendpoint[i] = voltageToInt32(DB[i+4+g_numwaves+g_numrampDACchannels].toFloat()/1000.0);
    //g_DACendpoint[i] = voltageToInt32(atof(incommand->token[i+4+g_numwaves+g_numrampDACchannels])/1000.0);
    if(g_numsteps < 2) //handle numsteps < 2
    {
      g_DACstep[i] = (((int64_t)g_DACendpoint[i] * BIT31) - ((int64_t)g_DACstartpoint[i] * BIT31));
    }
    else
    {
      g_DACstep[i] = (((int64_t)g_DACendpoint[i] * BIT31) - ((int64_t)g_DACstartpoint[i] * BIT31)) / (g_numsteps - 1);
    }
    DACintegersend(g_DACchanselect[i], (g_DACramppoint[i] / BIT47));//Set ramp DACs to initial point

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

  //Set AWG DACs to initial point
  for(i = 0; i < g_numwaves; i++)
  {
    for(j = 0; j < g_awgwave[i].numDACchannels; j++)
    {
      DACintegersend(g_awgwave[i].DACchanselect[j], g_awgwave[i].setpoint[0]);
    }
  }

  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall
  ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
  ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);

  for(i = 0; i < g_numrampADCchannels; i++)//Configure ADC channels
  {
    //g_ADCchanselect[i] = channelsADC[i] - '0';
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
  SPI.transfer(adc, ADC_IO_RDYFN | ADC_IO_SYNC | ADC_IO_P1DIR); //Change RDY to only trigger when all channels complete, and start only when synced, P1 as input
  delayMicroseconds(1000); // wait for DACs to settle

  //Check for SYNC again
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    return;
  }
  digitalWrite(data,HIGH);
  
  send_ack();//send ack and immediately attach interrupt  

  attachInterrupt(digitalPinToInterrupt(drdy), awgint, FALLING);

  digitalWrite(adc_trig_out, HIGH); //send sync signal (only master has control)

  while(!g_done)
  {
    if(query_serial(incommand))
    {
      if(strcmp("STOP", incommand->token[0]) == 0)
      {
        break;
      }     
    }
  }  
  detachInterrupt(digitalPinToInterrupt(drdy));
  SPI.transfer(adc, ADC_IO); //Write to ADC IO register
  SPI.transfer(adc, ADC_IO_DEFAULT | ADC_IO_P1DIR); //Change RDY to trigger when any channel complete, set P1 as input
  for(i = 0; i < NUMADCCHANNELS; i++)
  {
  SPI.transfer(adc, ADC_CHSETUP | i);//Access channel setup register
  SPI.transfer(adc, ADC_CHSETUP_RNG10BI);//set +/-10V range and disable for continuous mode
  SPI.transfer(adc, ADC_CHMODE | g_ADCchanselect[i]);   //Access channel mode register
  SPI.transfer(adc, ADC_MODE_IDLE);  //Set ADC to idle
  }
  digitalWrite(data,LOW);
  SERIALPORT.println("RAMP_FINISHED");
}

void awgint()//interrupt for AWG ramp
{
   uint32_t i, j;
   if(!g_done)
   {
      ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
      ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);
      
#ifdef OPTICAL //no buffering with regular UART (optical)

      if(g_firstsamples)
      {
        for(i = 0; i < g_numrampADCchannels; i++) //discard first loop
        {
           SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
           SPI.transfer(adc, 0, SPI_CONTINUE); // Read/write first byte
           SPI.transfer(adc, 0); // Read/write second byte
           g_firstsamples = false;           
        }        
      }
      else
      {
        for(i = 0; i < g_numrampADCchannels; i++)
        {
           SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
           SERIALPORT.write(SPI.transfer(adc, 0, SPI_CONTINUE)); // Read/write first byte
           SERIALPORT.write(SPI.transfer(adc, 0)); // Read/write second byte
        }
      }
#else      
      for(i = 0; i < g_numrampADCchannels; i++)
      {
         SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
         g_USBbuff[g_buffindex] = SPI.transfer(adc, 0, SPI_CONTINUE); // Reads first byte
         g_USBbuff[g_buffindex + 1] = SPI.transfer(adc, 0); // Reads second byte
         g_buffindex += 2;
      }

      if(g_firstsamples)//first loop has to be discarded, so just overwrite buffer
      {
        g_buffindex = 0;
        g_firstsamples = false;         
      }
#endif

      for(i = 0; i < g_numwaves; i++)
      {
        g_awgwave[i].samplecount++;
        
        if(g_awgwave[i].samplecount >= g_awgwave[i].numsamples[g_awgwave[i].setpointcount]) //got all samples, go to next setpoint
        {
          g_awgwave[i].samplecount = 0;
          g_awgwave[i].setpointcount++;
          if(g_awgwave[i].setpointcount >= g_awgwave[i].numsetpoints) //waveform complete, go to next loop
          {
            g_nextloop = true;
            g_awgwave[i].setpointcount = 0;
          }
          //update wave dacs
          for(j = 0; j < g_awgwave[i].numDACchannels; j++)
          {
            DACintegersend(g_awgwave[i].DACchanselect[j], g_awgwave[i].setpoint[g_awgwave[i].setpointcount]);
          }          
        }
      }
      if(g_nextloop)
      {
        g_nextloop = false;
        g_loopcount++;
        if(g_loopcount >= g_numloops)
        {
          g_loopcount = 0;
          g_stepcount++;
          if(g_stepcount >= g_numsteps)//we're done, just need to get last sample(s) that ADC is currently sampling
          {
            g_done = true;
            waitDRDY();
#ifdef OPTICAL
            for(i = 0; i < g_numrampADCchannels; i++)
            {
              SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
              SERIALPORT.write(SPI.transfer(adc, 0, SPI_CONTINUE)); // Read/write first byte
              SERIALPORT.write(SPI.transfer(adc, 0)); // Read/write second byte
            }
#else                        
            for(i = 0; i < g_numrampADCchannels; i++)
            {
              SPI.transfer(adc, ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i], SPI_CONTINUE); //Read channel data register
              g_USBbuff[g_buffindex] = SPI.transfer(adc, 0, SPI_CONTINUE); // Reads first byte
              g_USBbuff[g_buffindex + 1] = SPI.transfer(adc, 0); // Reads second byte
              g_buffindex += 2;
            }
#endif               
            detachInterrupt(digitalPinToInterrupt(drdy));            
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
#ifndef OPTICAL
      if(g_buffindex >= USBBUFFSIZE || g_done)
      {
        SERIALPORT.write((char*)g_USBbuff, g_buffindex);
        g_buffindex = 0;
      }      
#endif     
   }
}

/////////////////
//   EEPROM    //
/////////////////

void eeprom_test(InCommand * incommand)
{
  eepromtest();
}

void write_id_eeprom(InCommand * incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  if(digitalRead(EEPROM_WP_PIN) == HIGH)
  {
    SERIALPORT.println("WRITE_PROTECTED");
    return;
  }
  if(strlen(incommand->token[1]) >= EEPROM_ID_LEN)
  {
    range_error();
    return;
  }
  if(writeeepromid(incommand->token[1]) == 0)
  {
    send_ack();
    SERIALPORT.println("ID_SAVED");
  }
  else
  {
    SERIALPORT.println("EEPROM_ERROR");
  }
}

void write_dac_cal_eeprom(InCommand * incommand)
{
  if((incommand->paramcount != 2) && (incommand->paramcount != 3))
  {
    syntax_error();
    return;
  }
	uint8_t ch = atoi(incommand->token[1]);
	if(ch >= NUMDACCHANNELS)
	{
		range_error();
		return;
	}

	if(incommand->paramcount == 2)
	{
		writeeepromdaccal(ch, dacocal[ch], dacgcal[ch], false);
	}
	else if(strcmp(incommand->token[2], "FACTORY") == 0)
	{
		if(digitalRead(EEPROM_WP_PIN) == HIGH)
  	{
    	SERIALPORT.println("WRITE_PROTECTED");
    	return;
  	}
		send_ack();
		writeeepromdaccal(ch, dacocal[ch], dacgcal[ch], true);
	}

	send_ack();
	
	SERIALPORT.println("CAL_SAVED");
}