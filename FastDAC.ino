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


#include <SPI.h>
#include "src/PID/PID_v1.h" // Our own 'fork' of https://github.com/br3ttb/Arduino-PID-Library/
//#include <vector>
#include "FastDACdefs.h"
#include "FastDACcalibration.h" //This cal file should be copied and renamed for each DAQ unit, maybe store in EEPROM in the future
#include "FastDACeeprom.h"

#define SENDACK //Comment this to stop sending ACKs for every command

//~2kB of RAM free with ringbuffer increased to 1024
#define AWGMAXSETPOINTS 100 //Maximum number of setpoints of waveform generator
#define AWGMAXWAVES 2 //Maximum number of individual waveforms

#define ARGMAXSETPOINTS 10000 //Maximum number of setpoints in arbitrary ramp
#define ARGMAXRAMPS 4 //Maximum number of arbitrary ramps

#define MAXNUMPIDS 1 //Maximum number of simultaneous PID loops, only 1 for now

#define COMMANDBUFFERSIZE 1025 //Buffer for incoming command
#define MAXPARAMS 300 //maximum number of parameters to be parsed in a single command

#define DACSETTLEMICROS 2000 //microseconds to wait before starting ramp

//#define DEBUGRAMP //Uncomment this to enable sending of ramp debug info (actually debug info in general)

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


SPISettings adcspi(10500000, MSBFIRST, SPI_MODE3);
SPISettings dacspi(21000000, MSBFIRST, SPI_MODE1);
const int adc=52; //The SPI pin for the ADC
const int dac0 = 4; //The SPI pin for the DAC0
const int dac1 = 10; //The SPI pin for the DAC1
const int ldac0=6; //Load DAC1 pin for DAC1. Make it LOW if not in use.
const int ldac1=9; //Load DAC2 pin for DAC1. Make it LOW if not in use.
//Pio *ldac_port = digitalPinToPort(ldac0); //ldac0 and ldac1 share the same port, so they can be toggled simultaneously
//const uint32_t ldac0_mask = digitalPinToBitMask(ldac0);
//const uint32_t ldac1_mask = digitalPinToBitMask(ldac1);

const int reset=44 ; //Reset on ADC
const int drdy=48; // Data is ready pin on ADC
const int led = 28;
const int data=30;//Used for trouble shooting; connect an LED between pin 28 and GND
const int err=35;
const int testpin = 11;

float g_dac_full_scale = 10.0;
float g_dac_bit_res = g_dac_full_scale / 32768.0;

volatile int16_t g_DACsetpoint[NUMDACCHANNELS];//global array for current DAC setpoints, only written to in DACintegersend()

//Ramp interrupt global variables
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
volatile uint8_t g_numargramps;

typedef struct ARGramp
{
  int16_t setpoint[ARGMAXSETPOINTS];
  uint32_t numsetpoints;
  uint8_t numDACchannels;
  uint8_t DACchanselect[NUMDACCHANNELS];
  //uint32_t setpointcount;
}ARGramp;

ARGramp g_argramp[ARGMAXRAMPS];


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
  pinMode(testpin , OUTPUT);
  digitalWrite(reset,HIGH); // Resets ADC on startup.
  digitalWrite(data,LOW);
  digitalWrite(reset,LOW);
  digitalWrite(data,HIGH);
  delayMicroseconds(5000); // wait 5ms
  digitalWrite(reset,HIGH);
  digitalWrite(data,LOW);

  SPI.begin(); // wake up the SPI bus
  
	if(initeeprom() != 0)
  {
    SERIALPORT.println("ERROR Initializing EEPROM!");
  }
  
	loaddaccals();
	//Initialize saved DAC setpoints to 0
  for(int i = 0; i < NUMDACCHANNELS; i++)
  {
    g_DACsetpoint[i] = 0;
  }
  loadadccals();

  attachInterrupt(digitalPinToInterrupt(drdy), intset, FALLING);//Interrupt has to be attached once for the priority to stick
  //NVIC_SetPriority(PIOC_IRQn, 1); //Make ADC interrupt priority lower than UART
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
    if ((received == '\r') || (received == '\n'))// || (received == '\0'))//Check for end of message
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
      if(count >= MAXPARAMS)
      {
        SERIALPORT.println("ERROR_MAX_COMMAND_PARAMS");
        break;
      }
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
  static InCommand incommand{.buffer = {0}, .token = {0}, .paramcount = 0, .bufflength = 0};
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
  digitalWrite(testpin, HIGH);
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
  else if(strcmp("INT_ARG_RAMP", cmd) == 0)  
  {  
    int_arg_ramp(incommand);
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
  else if(strcmp("ADD_RAMP", cmd) == 0)  
  {  
    add_ramp(incommand);
  }
  else if(strcmp("CLR_RAMP", cmd) == 0)  
  {  
    clr_ramp(incommand);
  }
  else if(strcmp("CHECK_RAMP", cmd) == 0)  
  {  
    check_ramp(incommand);
  }  
  else if(strcmp("AWG_RAMP", cmd) == 0)  
  {  
    awg_ramp(incommand);
  }
  else if(strcmp("AWG_ARG_RAMP", cmd) == 0)  
  {  
    awg_arg_ramp(incommand);
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
  else if(strcmp("READ_DAC_CAL_EEPROM", cmd) == 0)
  {  
    read_dac_cal_eeprom(incommand);
  }
  else if(strcmp("WRITE_ADC_CAL_EEPROM", cmd) == 0)
  {  
    write_adc_cal_eeprom(incommand);
  }
  else if(strcmp("READ_ADC_CAL_EEPROM", cmd) == 0)
  {  
    read_adc_cal_eeprom(incommand);
  }
  else if(strcmp("INIT_ALL_EEPROM_VALUES", cmd) == 0)
  {  
    init_all_eeprom_values(incommand);
  }
  else if(strcmp("CAL_ALL_ADC_EEPROM_WITH_DAC", cmd) == 0)
  {  
    cal_all_adc_eeprom_with_dac(incommand);
  }
  
  else
  {
    SERIALPORT.println("NOP");
  }
  digitalWrite(testpin, LOW);
	//SERIALPORT.print(F("Free RAM = "));
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

int32_t readADCConversionTime(uint8_t ch)
{
  if (ch >= NUMADCCHANNELS)
  {
    range_error();
    return -1;
  }
  byte fw;
  fw = readADCfw(ch);
  //SERIALPORT.println(fw);
  int contime = ((int)((fw * 128 + 249) / 6.144 ) + 0.5);
  return contime;
}

uint8_t readADCfw(uint8_t ch)
{
  if (ch >= NUMADCCHANNELS)
  {
    range_error();
    return 0;
  }
  uint8_t fw;

  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_REGREAD | ADC_CHCONVTIME | ch); //Read conversion time register
  fw = SPI.transfer(0); //Read back the CT register
  digitalWrite(adc, HIGH);
  SPI.endTransaction();

  fw &= 0x7F; //get lowest 7 bits
  return fw;
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

  writeADCfw(adcChannel, fw);
  delayMicroseconds(100);
  fw = readADCfw(adcChannel);

  //Automatically load calibration for this fw
  uint32_t zeroscale, fullscale;
  readeepromadccal(adcChannel, fw, &zeroscale, &fullscale, false);
  writeADCcal(adcChannel, zeroscale, fullscale);
  int convtime = ((int)(((fw) * 128 + 249) / 6.144) + 0.5);
  SERIALPORT.println(convtime);
}

void writeADCfw(uint8_t ch, uint8_t fw)
{
  fw |= 0x80; //enable chopping
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_CHCONVTIME | ch); //Write conversion time register
  SPI.transfer(fw); //Write 'filter word' (conversion time)
  digitalWrite(adc, HIGH);
  SPI.endTransaction();
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
  if((abs(setpoint) / 1000.0) > g_dac_full_scale)
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
  uint32_t nSteps = static_cast<int>(abs(setpoint-initial)/ramprate*1000);  //using 1ms as delay
  if (nSteps < 5)
  {
    nSteps = 5;
  }
#ifdef DEBUGRAMP  
  SERIALPORT.print("initial: ");
  SERIALPORT.println(initial);

  SERIALPORT.print("setpoint: ");
  SERIALPORT.println(setpoint);  
  
  SERIALPORT.print("nsteps: ");
  SERIALPORT.println(nSteps);
#endif
  autoRamp1(initial, setpoint, nSteps, dacChannel, 1000, incommand);
  SERIALPORT.println("RAMP_FINISHED");

  return;
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
  
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_CHSETUP | g_pidparam[0].ADCchan);//Access channel setup register
  SPI.transfer(ADC_CHSETUP_RNG10BI | ADC_CHSETUP_ENABLE);//set +/-10V range and enable for continuous mode
  SPI.transfer(ADC_CHMODE | g_pidparam[0].ADCchan);   //Access channel mode register
  SPI.transfer(ADC_MODE_CONTCONV | ADC_MODE_CLAMP);  //Continuous conversion with clamping
  SPI.transfer(ADC_IO); //Write to ADC IO register
  SPI.transfer(ADC_IO_RDYFN | ADC_IO_SYNC | ADC_IO_P1DIR); //Change RDY to only trigger when all channels complete, and start only when synced, P1 as input
  digitalWrite(adc, HIGH);
  SPI.endTransaction();
  
  attachInterrupt(digitalPinToInterrupt(drdy), pidint, FALLING);

  
}
//PID Interrupts every ADC sample
void pidint(void)
{
  uint8_t b1, b2;
  int decimal;
  
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_CHDATA | ADC_REGREAD | g_pidparam[0].ADCchan); //Read channel data register
  b1 = SPI.transfer(0); // Read first byte
  b2 = SPI.transfer(0); // Read second byte
  digitalWrite(adc,HIGH);
  SPI.endTransaction();
  
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
  
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_IO); //Write to ADC IO register
  SPI.transfer(ADC_IO_DEFAULT | ADC_IO_P1DIR); //Change RDY to trigger when any channel complete, set P1 as input
  for(i = 0; i < NUMADCCHANNELS; i++)
  {
  SPI.transfer(ADC_CHSETUP | i);//Access channel setup register
  SPI.transfer(ADC_CHSETUP_RNG10BI);//set +/-10V range and disable for continuous mode
  SPI.transfer(ADC_CHMODE | i);   //Access channel mode register
  SPI.transfer(ADC_MODE_IDLE);  //Set ADC to idle
  }
  digitalWrite(adc,HIGH);
  SPI.endTransaction();

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
  if((abs(g_pidparam[0].dacmin) / 1000.0 > g_dac_full_scale) || (abs(g_pidparam[0].dacmax) / 1000.0 > g_dac_full_scale))
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

void loaddaccals(void)
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

void loadadccals(void)
{  
  //load user ADC cals
  uint8_t ch, fw;
  uint32_t zeroscale, fullscale;
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    fw = readADCfw(ch);
    readeepromadccal(ch, fw, &zeroscale, &fullscale, false);
    writeADCcal(ch, zeroscale, fullscale);
  }
}

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
  delayMicroseconds(CAL_SETTLE_TIME); // wait cal settle time
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
  delayMicroseconds(CAL_SETTLE_TIME); // wait cal settle time
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
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW); 
  SPI.transfer(ADC_CHMODE);   // Access ch mode register in write mode
  SPI.transfer(ADC_MODE_IDLE);       // Enter idle mode

  SPI.transfer(ADC_CHMODE);   // Access ch mode register in write mode
  SPI.transfer(ADC_MODE_SELFZEROCAL);       // Enter system zero-scale cal mode
  digitalWrite(adc,HIGH);
  SPI.endTransaction();  
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

  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(ADC_MODE_IDLE);       // Enter idle mode

  SPI.transfer(ADC_CHMODE | ch);   // Access ch mode register in write mode
  SPI.transfer(ADC_MODE_SYSZEROCAL);       // Enter system zero-scale cal mode
  digitalWrite(adc,HIGH);
  SPI.endTransaction();
  
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
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_CHZEROSCALECAL | ADC_REGREAD | ch);   // Access ch zero-scale cal register in read mode
  b1 = SPI.transfer(0x00);   // read byte 1
  b2 = SPI.transfer(0x00);   // read byte 2
  b3 = SPI.transfer(0x00);   // read byte 3
  digitalWrite(adc,HIGH);
  SPI.endTransaction();  

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
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_CHMODE | ch); // Access ch mode register in write mode
  SPI.transfer(ADC_MODE_IDLE); // Enter idle mode

  SPI.transfer(ADC_CHMODE | ch); // Access ch mode register in write mode
  SPI.transfer(ADC_MODE_SYSFULLCAL); // Enter system full-scale cal mode
  digitalWrite(adc,HIGH);
  SPI.endTransaction();
  waitDRDY();

  return readADCfullcal(ch);
}

uint32_t readADCfullcal(uint8_t ch)
{
  uint8_t b1, b2, b3;
  uint32_t calvalue;
  
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_CHFULLSCALECAL | ADC_REGREAD | ch);   // Access ch full-scale cal register in read mode
  b1 = SPI.transfer(0x00);   // read byte 1
  b2 = SPI.transfer(0x00);   // read byte 2
  b3 = SPI.transfer(0x00);   // read byte 3
  digitalWrite(adc,HIGH);
  SPI.endTransaction();

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
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);  
  SPI.transfer(ADC_CHZEROSCALECAL | ch); //Write channel zero scale register
  SPI.transfer((zeroscale & 0xFF0000) >> 16); // Write first byte
  SPI.transfer((zeroscale & 0xFF00) >> 8); // Write second byte
  SPI.transfer(zeroscale & 0xFF); // Write third byte
  digitalWrite(adc,HIGH);
  SPI.endTransaction();  
}

void writeADCchfullscale(byte ch, int32_t fullscale)
{
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_CHFULLSCALECAL | ch); //Write channel zero scale register
  SPI.transfer((fullscale & 0xFF0000) >> 16); // Write first byte
  SPI.transfer((fullscale & 0xFF00) >> 8); // Write second byte
  SPI.transfer(fullscale & 0xFF); // Write third byte
  digitalWrite(adc,HIGH);
  SPI.endTransaction();  
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
  g_dac_full_scale = dacscale;
  g_dac_bit_res = g_dac_full_scale / 32768.0;
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
  if((offset > (g_dac_full_scale * 2.0 * 16 / 65536.0)) || (offset < (g_dac_full_scale * -2.0 * 15.875 / 65536.0)))
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
  float stepsize = (g_dac_full_scale * 2.0) / (65536.0 * 8.0); //stepsize is 1/8 of a 16-bit LSB

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
  if((offset > (g_dac_full_scale * 2.0 * 16 / 65536.0)) || (offset < (g_dac_full_scale * -2.0 * 15 / 65536.0)))
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
  float stepsize = (g_dac_full_scale * 2.0) / (65536.0 * 2.0); //stepsize is 1/2 of a 16-bit LSB
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
  SPI.beginTransaction(dacspi);
  digitalWrite(thisdac, LOW);
	//SERIALPORT.println(steps);
  SPI.transfer(DAC_OFFSET | ch); // Write DAC channel offset register
  SPI.transfer(0x00);   // writes first byte
  SPI.transfer(steps);
  digitalWrite(thisdac,HIGH);
  SPI.endTransaction(); 

  digitalWrite(thisldac, LOW);
  digitalWrite(thisldac, HIGH);
}

void writeDACgain(int ch, int8_t steps)
{
  int thisdac;
  int thisldac;

	dacgcal[ch] = steps;
  convertDACch(&ch, &thisdac, &thisldac);
  SPI.beginTransaction(dacspi);
  digitalWrite(thisdac, LOW);
  SPI.transfer(DAC_FINEGAIN  | ch); // Write DAC channel fine gain register
  SPI.transfer(0x00);   // writes first byte
  SPI.transfer(steps);
  digitalWrite(thisdac,HIGH);
  SPI.endTransaction(); 

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


/*
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
*/

int freeMemory() 
{
/*
  char top;
//#ifdef __arm__  
  return &top - reinterpret_cast<char*>(sbrk(0));
//#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
//#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
//#endif  // __arm__
*/
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
    SPI.beginTransaction(adcspi);
    digitalWrite(adc, LOW);
    SPI.transfer(ADC_CHMODE | adcchan);   // Write channel mode register
    SPI.transfer(ADC_MODE_SINGLECONV | ADC_MODE_DUMP | ADC_MODE_CLAMP); // Single conversion + dump mode + clamp
    waitDRDY();                       // Waits until convertion finishes
    SPI.transfer(ADC_CHDATA | ADC_REGREAD | adcchan);   // Read channel data register
    statusbyte=SPI.transfer(0);   // Reads Channel 'ch' status
    o2=SPI.transfer(0);           // Reads first byte
    o3=SPI.transfer(0);           // Reads second byte
    digitalWrite(adc,HIGH);
    SPI.endTransaction();
    
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

  digitalWrite(data,HIGH);
  delayMicroseconds(1);
  digitalWrite(reset,HIGH);
  delayMicroseconds(1);
  digitalWrite(reset,LOW);
  delayMicroseconds(1);
  digitalWrite(reset,HIGH);
  delayMicroseconds(1);
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    SPI.beginTransaction(adcspi);
    digitalWrite(adc, LOW);
    SPI.transfer(ADC_CHSETUP | ch);// access channel setup register for each channel
    SPI.transfer(ADC_CHSETUP_RNG10BI);// set +/-10V range
    digitalWrite(adc,HIGH);
    SPI.endTransaction();
  }
}

//// DAC UTIL ////



void autoRamp1(float v1, float v2, uint32_t nSteps, uint8_t dacChannel, uint32_t period, InCommand *incommand)
// voltage in mV
{
  
  #ifdef DEBUGRAMP  
  SERIALPORT.print("v1: ");
  SERIALPORT.println(v1);

  SERIALPORT.print("v2: ");
  SERIALPORT.println(v2);  
  
  SERIALPORT.print("nsteps: ");
  SERIALPORT.println(nSteps);
#endif
  
  digitalWrite(data,HIGH);
  uint32_t timer = micros();
  uint32_t j = 0;
  while(j < nSteps)
  {
    uint32_t nowmicros = micros();
    digitalWrite(data, HIGH);
    if((nowmicros - timer) > period)//Time to take another step
    {
      timer = nowmicros;
      float setpoint = (v1+(v2-v1)*j/(nSteps-1));
      //make sure within range
      if(setpoint > g_dac_full_scale * 1000.0)
      {
        setpoint = g_dac_full_scale * 1000.0;
      }
      else if(setpoint < g_dac_full_scale * -1000.0)
      {
        setpoint = g_dac_full_scale * -1000.0;
      }
      writeDAC(dacChannel, setpoint, true); // takes mV
      //SERIALPORT.println(v1+(v2-v1)*j/(nSteps-1));
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
  //SERIALPORT.println(voltageToInt16(voltage/1000.0));
  float voltreturn;
  voltreturn = readDAC(ch);
  return voltreturn;
}

void DACintegersend(byte ch, int16_t value)
{
  if(ch <= 3)
  {
    g_DACsetpoint[ch] = value;
    SPI.beginTransaction(dacspi);
    digitalWrite(dac0, LOW);
    SPI.transfer(DAC_DATA | ch); // Indicates to DAC to write channel 'ch' in the data register
    SPI.transfer((uint8_t)(value >> 8));   // writes first byte
    SPI.transfer((uint8_t)(value & 0xff));                // writes second byte
    digitalWrite(dac0, HIGH);
    SPI.endTransaction();
  }
  else if(ch < NUMDACCHANNELS)
  {
    g_DACsetpoint[ch] = value;
    ch -= 4;
    SPI.beginTransaction(dacspi);
    digitalWrite(dac1, LOW);    
    SPI.transfer(DAC_DATA | ch); // Indicates to DAC to write channel 'ch' in the data register
    SPI.transfer((uint8_t)(value >> 8)); // writes first byte
    SPI.transfer((uint8_t)(value & 0xff)); // writes second byte
    digitalWrite(dac1, HIGH);
    SPI.endTransaction();
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
  voltage = data * g_dac_full_scale/32768.0;
  /*
  if (data >= 0)
  {
    voltage = data * g_dac_full_scale/32767;
  }
  else
  {
    voltage = data * g_dac_full_scale/32768;
  }
  */
  return voltage;
}

int16_t voltageToInt16(float voltage)
// map float voltage to 16bit int
{
  if (abs(voltage) > g_dac_full_scale)
  {
    error();
    return 0;
  }
  if(voltage > (g_dac_full_scale - g_dac_bit_res))
  {
    voltage = g_dac_full_scale - g_dac_bit_res;
  }
  return  (int16_t)((voltage / g_dac_full_scale) * 0x8000); 
  
  /*
  else if (voltage >= 0)
  {
    return voltage*32767/g_dac_full_scale;
  }
  else
  {
    return voltage*32768/g_dac_full_scale;
  }
  */  
}

int32_t voltageToInt32(float voltage)
// map float voltage to 32bit int
{
  int32_t calcint;
  if (abs(voltage) > g_dac_full_scale)
  {
    calcint = 0;
    error();
  }
  if(voltage > (g_dac_full_scale - g_dac_bit_res))
  {
    voltage = g_dac_full_scale - g_dac_bit_res;
  }
  calcint = (int32_t)((voltage/g_dac_full_scale) * 0x80000000);
  /*
  else if(voltage >=0)
  {
    calcint = (int32_t)((voltage/g_dac_full_scale) * 0x7FFFFFFF);
  }
  else
  {
    calcint = (int32_t)((voltage/g_dac_full_scale) * 0x80000000);
  }
  */
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
    if(abs(setvolt) > g_dac_full_scale)
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

void int_ramp(InCommand *incommand)//<dac channels>,<adc channels>,<initial dac voltage 1>,...<initial dac voltage n>,<final dac voltage 1>,...<final dac voltage n>,<number of steps>
{
  int i;
    //check for minimum number of parameters
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    return;
  }
  
  if(incommand->paramcount < 4)
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
  //check if no DACs selected
  if((g_numrampDACchannels == 1) && (channelsDAC[0] == 'N'))
  {
    //SERIALPORT.println("NO DACS");
    g_numrampDACchannels = 0;
  }

  g_done = false;  
  g_firstsamples = true;
  g_stepcount = 0;
  g_loopcount = 0;
  g_nextloop = false;

  g_numloops = 1; //take only 1 sample at each step
  g_numwaves = 0; //no arbitrary waves for int_ramp
  g_numargramps = 0;//no arbitrary ramps for int_ramp
  //Do some bounds checking
  if((g_numrampDACchannels > NUMDACCHANNELS) || (g_numrampADCchannels > NUMADCCHANNELS) || (incommand->paramcount != g_numrampDACchannels * 2 + 4))
  {
    syntax_error();
    return;
  }  
  //check if no DACs selected
  if(g_numrampDACchannels != 0)
  {
    //define DAC channels and check range
    for(i = 0; i < g_numrampDACchannels; i++)
    {
      g_DACchanselect[i] = channelsDAC[i] - '0';
      if(g_DACchanselect[i] >= NUMDACCHANNELS)
      {
        range_error();
        return;
      }
      float dacstartvoltage = atof(incommand->token[i+3])/1000.0;
      float dacendvoltage = atof(incommand->token[i+3+g_numrampDACchannels])/1000.0;
      if((abs(dacstartvoltage) > g_dac_full_scale) || (abs(dacendvoltage) > g_dac_full_scale))
      {
        range_error();
        return;
      }
      g_DACstartpoint[i] = voltageToInt32(dacstartvoltage);
      g_DACendpoint[i] = voltageToInt32(dacendvoltage);
    }
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
  g_configurerampDACchannels();
  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall
  digitalWrite(ldac0, LOW);
  digitalWrite(ldac1, LOW);
  digitalWrite(ldac0, HIGH);
  digitalWrite(ldac1, HIGH);
  //Need to check if this is still necessary
  //ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
  //ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);

  g_configurerampADCchannels();

  delayMicroseconds(DACSETTLEMICROS); // wait for DACs to settle
  
  //Check for SYNC again
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    g_rampadcidle();
    return;
  }
  digitalWrite(data,HIGH);

  send_ack();//send ack and immediately attach interrupt
  
  attachInterrupt(digitalPinToInterrupt(drdy), awg_ramp_int, FALLING);  
  //attachInterrupt(digitalPinToInterrupt(drdy), int_ramp_int, FALLING);  

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
  
  g_rampadcidle();
  digitalWrite(data,LOW);
  SERIALPORT.println("RAMP_FINISHED");
}

//INT_ARG_RAMP,<number of arg ramps><dac channels assigned to arg 0>,<dac channels assigned to arg n>,<dac channels>,<adc channels>,
//<initial dac voltage 1>,...<initial dac voltage n>,<final dac voltage 1>,...<final dac voltage n>,<number of samples per setpoint>
void int_arg_ramp(InCommand *incommand)
{
  uint32_t i, j;
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

  g_numargramps = atoi(incommand->token[1]); //First parameter
  if(g_numargramps > ARGMAXRAMPS)
  {
    range_error();
    //SERIALPORT.print("ERROR, Max waveforms = ");
    //SERIALPORT.println(AWGMAXWAVES);
    return;
  }
  for(i = 0; i < g_numargramps; i++)
  {
    char * channelsramp = incommand->token[i + 2];
    g_argramp[i].numDACchannels = strlen(channelsramp);
    //g_argramp[i].setpointcount = 0;
    for(j = 0; j < g_argramp[i].numDACchannels; j++)
    {
      uint8_t argdac = channelsramp[j] - '0';
      if(argdac >= NUMDACCHANNELS)
      {
        range_error();
        return;
      }
      g_argramp[i].DACchanselect[j] = argdac;
    }
  }
#ifdef DEBUGRAMP
  SERIALPORT.print("Numargramps = ");
  SERIALPORT.println(g_numargramps);
  for(i = 0; i < g_numargramps; i++)
  {
    SERIALPORT.print("ARG ");
    SERIALPORT.print(i);
    SERIALPORT.print(" DAC Channels: ");
    for(j = 0; j < g_argramp[i].numDACchannels; j++)
    {
      SERIALPORT.print(g_argramp[i].DACchanselect[j]);
      SERIALPORT.print(" ");
    }
    SERIALPORT.println(" ");
  }
#endif  
  char * channelsDAC = incommand->token[g_numargramps + 2];
  char * channelsADC = incommand->token[g_numargramps + 3];
  g_numrampDACchannels = strlen(channelsDAC);  
  g_numrampADCchannels = strlen(channelsADC);
  //check if no DACs selected
  if((g_numrampDACchannels == 1) && (channelsDAC[0] == 'N'))
  {
    //SERIALPORT.println("NO DACS");
    g_numrampDACchannels = 0;
  }

  g_done = false;  
  g_firstsamples = true;
  g_nextloop = false;

  g_stepcount = 0;
  g_loopcount = 0;
 
  g_numwaves = 0; //no arbitrary waves for int_ramp

  //Do some bounds checking
  if((g_numrampDACchannels > NUMDACCHANNELS) || (g_numrampADCchannels > NUMADCCHANNELS) || ((uint16_t)incommand->paramcount != g_numrampDACchannels * 2 + 5 + g_numargramps))
  {
    syntax_error();
    return;
  }  

  g_numloops = atoi(incommand->token[g_numrampDACchannels*2+4+g_numargramps]);
  g_numsteps = 0;
  for(i = 0; i < g_numargramps; i++)
  {
    if(g_argramp[i].numsetpoints > g_numsteps)
    {
      g_numsteps = g_argramp[i].numsetpoints;
    }
  }

#ifdef DEBUGRAMP
  SERIALPORT.print("Numloops: ");  
  SERIALPORT.println(g_numloops);
  SERIALPORT.print("Numsteps: ");  
  SERIALPORT.println(g_numsteps);
#endif

  //check if no DACs selected
  if(g_numrampDACchannels != 0)
  {
    //define DAC channels and check range
    for(i = 0; i < g_numrampDACchannels; i++)
    {
      g_DACchanselect[i] = channelsDAC[i] - '0';
      if(g_DACchanselect[i] >= NUMDACCHANNELS)
      {
        range_error();
        return;
      }
      float dacstartvoltage = atof(incommand->token[i+4+g_numargramps])/1000.0;
      float dacendvoltage = atof(incommand->token[i+4+g_numargramps+g_numrampDACchannels])/1000.0;
      if((abs(dacstartvoltage) > g_dac_full_scale) || (abs(dacendvoltage) > g_dac_full_scale))
      {
        range_error();
        return;
      }
      g_DACstartpoint[i] = voltageToInt32(dacstartvoltage);
      g_DACendpoint[i] = voltageToInt32(dacendvoltage);
    }
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

  //configure DAC channels
  g_configurerampDACchannels();
  //Set ARG DACs to initial point
  for(i = 0; i < g_numargramps; i++)
  {
    for(j = 0; j < g_argramp[i].numDACchannels; j++)
    {
      DACintegersend(g_argramp[i].DACchanselect[j], g_argramp[i].setpoint[0]);
    }
  }
  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall
  digitalWrite(ldac0, LOW);
  digitalWrite(ldac1, LOW);
  digitalWrite(ldac0, HIGH);
  digitalWrite(ldac1, HIGH);
  //ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
  //ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);

  g_configurerampADCchannels();

  delayMicroseconds(DACSETTLEMICROS); // wait for DACs to settle
  
  //Check for SYNC again
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    g_rampadcidle();
    return;
  }
  digitalWrite(data,HIGH);

  send_ack();//send ack and immediately attach interrupt
  
  attachInterrupt(digitalPinToInterrupt(drdy), awg_ramp_int, FALLING);  
  //attachInterrupt(digitalPinToInterrupt(drdy), int_ramp_int, FALLING);  

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
  
  g_rampadcidle();
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
  char * channelsADC = incommand->token[1];
  g_numrampADCchannels = strlen(channelsADC);
  g_numsteps=atoi(incommand->token[2]);

  g_done = false;  
  g_firstsamples = true;
  g_nextloop = false;
  g_stepcount = 0;
  g_loopcount = 0;

  g_numloops = 1; //take only 1 sample at each step
  g_numwaves = 0; //no arbitrary waves for spec_ana
  g_numrampDACchannels = 0; //no linear ramp dacs for spec_ana
  g_numargramps = 0;

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
  g_configurerampADCchannels();

  //Check for SYNC again
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    g_rampadcidle();
    return;
  }
  
  digitalWrite(data,HIGH);
  
  send_ack();//send ack and immediately attach interrupt

  attachInterrupt(digitalPinToInterrupt(drdy), awg_ramp_int, FALLING);  

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
  g_rampadcidle();
  digitalWrite(data,LOW);
  SERIALPORT.println("READ_FINISHED");
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
  if(incommand->paramcount < 7)
  {
    syntax_error();
    return;
  }
  int i, j;
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
    char * channelswave = incommand->token[i + 2];
    g_awgwave[i].numDACchannels = strlen(channelswave);
    g_awgwave[i].samplecount = 0;
    g_awgwave[i].setpointcount = 0;
    for(j = 0; j < g_awgwave[i].numDACchannels; j++)
    {
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
  char * channelsDAC = incommand->token[g_numwaves + 2];
  g_numrampDACchannels = strlen(channelsDAC);

  char * channelsADC = incommand->token[g_numwaves + 3];
  g_numrampADCchannels = strlen(channelsADC);

  //check if no DACs selected
  if((g_numrampDACchannels == 1) && (channelsDAC[0] == 'N'))
  {
    //SERIALPORT.println("NO DACS");
    g_numrampDACchannels = 0;
  }

  g_done = false;
  g_nextloop = false;
  g_firstsamples = true;
  
  //Do some bounds checking
  if((g_numrampDACchannels > NUMDACCHANNELS) || (g_numrampADCchannels > NUMADCCHANNELS) || ((uint16_t)incommand->paramcount != g_numrampDACchannels * 2 + 6 + g_numwaves))
  {
    syntax_error();
    return;
  }  

  g_loopcount = 0;
  g_stepcount = 0;
  g_numargramps = 0;

 
  g_numloops=atoi(incommand->token[g_numrampDACchannels*2+4+g_numwaves]);
  g_numsteps=atoi(incommand->token[g_numrampDACchannels*2+5+g_numwaves]);

#ifdef DEBUGRAMP
  SERIALPORT.print("Numloops: ");  
  SERIALPORT.println(g_numloops);
  SERIALPORT.print("Numsteps: ");  
  SERIALPORT.println(g_numsteps);
#endif
  //check if no DACs selected
  if(g_numrampDACchannels != 0)
  {
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
      if((abs(dacstartvoltage) > g_dac_full_scale) || (abs(dacendvoltage) > g_dac_full_scale))
      {
        range_error();
        return;
      }
      g_DACstartpoint[i] = voltageToInt32(dacstartvoltage);
      g_DACendpoint[i] = voltageToInt32(dacendvoltage);   
    }
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
  g_configurerampDACchannels();
  //Set AWG DACs to initial point
  for(i = 0; i < g_numwaves; i++)
  {
    for(j = 0; j < g_awgwave[i].numDACchannels; j++)
    {
      DACintegersend(g_awgwave[i].DACchanselect[j], g_awgwave[i].setpoint[0]);
    }
  }

  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall
  digitalWrite(ldac0, LOW);
  digitalWrite(ldac1, LOW);
  digitalWrite(ldac0, HIGH);
  digitalWrite(ldac1, HIGH);  
  //ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
  //ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);
  
  g_configurerampADCchannels();
  delayMicroseconds(DACSETTLEMICROS); // wait for DACs to settle

  //Check for SYNC again
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    g_rampadcidle();
    return;
  }
  digitalWrite(data,HIGH);
  
  send_ack();//send ack and immediately attach interrupt  

  attachInterrupt(digitalPinToInterrupt(drdy), awg_ramp_int, FALLING);

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
  
  g_rampadcidle();
  digitalWrite(data,LOW);
  SERIALPORT.println("RAMP_FINISHED");
}

//AWG_ARG_RAMP,<numwaves>,<dacs waveform 0>,<dacs waveform n>,<num arg ramps>,<dacs arg 0>,<dacs arg n>,
//<dacs to ramp>,<adcs>,<initial dac voltage 1>,<…>,<initial dac voltage n>,
//<final dac voltage 1>,<…>,<final dac voltage n>,<# of waveform repetitions at each ramp step>
void awg_arg_ramp(InCommand *incommand)
{
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0)
  {
    return;
  }
  //Do some initial bounds checking
  if(incommand->paramcount < 8)
  {
    syntax_error();
    return;
  }
  int i, j;
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
    char * channelswave = incommand->token[i + 2];
    g_awgwave[i].numDACchannels = strlen(channelswave);
    g_awgwave[i].samplecount = 0;
    g_awgwave[i].setpointcount = 0;
    for(j = 0; j < g_awgwave[i].numDACchannels; j++)
    {
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
  g_numargramps = atoi(incommand->token[g_numwaves + 2]); //Number of arg ramps
  if(g_numargramps > ARGMAXRAMPS)
  {
    range_error();
    //SERIALPORT.print("ERROR, Max waveforms = ");
    //SERIALPORT.println(AWGMAXWAVES);
    return;
  }
  for(i = 0; i < g_numargramps; i++)
  {
    char * channelsramp = incommand->token[i + g_numwaves + 2];
    g_argramp[i].numDACchannels = strlen(channelsramp);
    //g_argramp[i].setpointcount = 0;
    for(j = 0; j < g_argramp[i].numDACchannels; j++)
    {
      uint8_t argdac = channelsramp[j] - '0';
      if(argdac >= NUMDACCHANNELS)
      {
        range_error();
        return;
      }
      g_argramp[i].DACchanselect[j] = argdac;
    }
  }
#ifdef DEBUGRAMP
  SERIALPORT.print("Numargramps = ");
  SERIALPORT.println(g_numargramps);
  for(i = 0; i < g_numargramps; i++)
  {
    SERIALPORT.print("ARG ");
    SERIALPORT.print(i);
    SERIALPORT.print(" DAC Channels: ");
    for(j = 0; j < g_argramp[i].numDACchannels; j++)
    {
      SERIALPORT.print(g_argramp[i].DACchanselect[j]);
      SERIALPORT.print(" ");
    }
    SERIALPORT.println(" ");
  }
#endif  

  char * channelsDAC = incommand->token[g_numwaves + g_numargramps + 3];
  g_numrampDACchannels = strlen(channelsDAC);

  char * channelsADC = incommand->token[g_numwaves + g_numargramps + 4];
  g_numrampADCchannels = strlen(channelsADC);

  //check if no DACs selected
  if((g_numrampDACchannels == 1) && (channelsDAC[0] == 'N'))
  {
    //SERIALPORT.println("NO DACS");
    g_numrampDACchannels = 0;
  }

  g_done = false;
  g_nextloop = false;
  g_firstsamples = true;
  
  //Do some bounds checking
  if((g_numrampDACchannels > NUMDACCHANNELS) || (g_numrampADCchannels > NUMADCCHANNELS) || ((uint16_t)incommand->paramcount != g_numrampDACchannels * 2 + 6 + g_numwaves + g_numargramps))
  {
    syntax_error();
    return;
  }  

  g_loopcount = 0;
  g_stepcount = 0;
 
  g_numloops=atoi(incommand->token[g_numrampDACchannels*2 + 5 + g_numwaves + g_numargramps]);
  g_numsteps = 0;
  for(i = 0; i < g_numargramps; i++)
  {
    if(g_argramp[i].numsetpoints > g_numsteps)
    {
      g_numsteps = g_argramp[i].numsetpoints;
    }
  }

#ifdef DEBUGRAMP
  SERIALPORT.print("Numloops: ");  
  SERIALPORT.println(g_numloops);
  SERIALPORT.print("Numsteps: ");  
  SERIALPORT.println(g_numsteps);
#endif
  //check if no DACs selected
  if(g_numrampDACchannels != 0)
  {
    //define DAC channels and check range  
    for(i = 0; i < g_numrampDACchannels; i++)
    {
      g_DACchanselect[i] = channelsDAC[i] - '0';
      if(g_DACchanselect[i] >= NUMDACCHANNELS)
      {
        range_error();
        return;
      }
      float dacstartvoltage = atof(incommand->token[i+5+g_numwaves+g_numargramps])/1000.0;
      float dacendvoltage = atof(incommand->token[i+5+g_numwaves+g_numargramps+g_numrampDACchannels])/1000.0;
      if((abs(dacstartvoltage) > g_dac_full_scale) || (abs(dacendvoltage) > g_dac_full_scale))
      {
        range_error();
        return;
      }
      g_DACstartpoint[i] = voltageToInt32(dacstartvoltage);
      g_DACendpoint[i] = voltageToInt32(dacendvoltage);   
    }
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
  g_configurerampDACchannels();
  //Set AWG DACs to initial point
  for(i = 0; i < g_numwaves; i++)
  {
    for(j = 0; j < g_awgwave[i].numDACchannels; j++)
    {
      DACintegersend(g_awgwave[i].DACchanselect[j], g_awgwave[i].setpoint[0]);
    }
  }
  //Set ARG DACs to initial point
  for(i = 0; i < g_numargramps; i++)
  {
    for(j = 0; j < g_argramp[i].numDACchannels; j++)
    {
      DACintegersend(g_argramp[i].DACchanselect[j], g_argramp[i].setpoint[0]);
    }
  }  
  
  delayMicroseconds(2); //Need at least 2 microseconds from SYNC rise to LDAC fall
  digitalWrite(ldac0, LOW);
  digitalWrite(ldac1, LOW);
  digitalWrite(ldac0, HIGH);
  digitalWrite(ldac1, HIGH);
  //ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
  //ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);
  
  g_configurerampADCchannels();
  delayMicroseconds(DACSETTLEMICROS); // wait for DACs to settle

  //Check for SYNC again
  if(sync_check(CHECK_CLOCK | CHECK_SYNC) != 0) //make sure ADC has a clock and sync armed if not indep mode
  {
    g_rampadcidle();
    return;
  }
  digitalWrite(data,HIGH);
  
  send_ack();//send ack and immediately attach interrupt  

  attachInterrupt(digitalPinToInterrupt(drdy), awg_ramp_int, FALLING);

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
  
  g_rampadcidle();
  digitalWrite(data,LOW);
  SERIALPORT.println("RAMP_FINISHED");
}


void awg_ramp_int()//interrupt for AWG ramp
{
   uint32_t i, j;
   if(!g_done)
   {
      digitalWrite(ldac0, LOW);
      digitalWrite(ldac1, LOW);
      digitalWrite(ldac0, HIGH);
      digitalWrite(ldac1, HIGH);
      //ldac_port->PIO_CODR |= (ldac0_mask | ldac1_mask);//Toggle ldac pins
      //ldac_port->PIO_SODR |= (ldac0_mask | ldac1_mask);

      if(g_firstsamples)
      {
        for(i = 0; i < g_numrampADCchannels; i++) //discard first loop
        {
          SPI.beginTransaction(adcspi);
          digitalWrite(adc, LOW);
          SPI.transfer(ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i]); //Read channel data register
          SPI.transfer(0); // Read/write first byte
          SPI.transfer(0); // Read/write second byte
          digitalWrite(adc, HIGH);
          SPI.endTransaction();
          g_firstsamples = false;           
        }        
      }
      else
      {
        for(i = 0; i < g_numrampADCchannels; i++)
        {
          SPI.beginTransaction(adcspi);
          digitalWrite(adc, LOW);
          SPI.transfer(ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i]); //Read channel data register
          SERIALPORT.write(SPI.transfer(0)); // Read/write first byte
          SERIALPORT.write(SPI.transfer(0)); // Read/write second byte
          digitalWrite(adc, HIGH);
          SPI.endTransaction();
        }
      }
      if(g_numwaves == 0)
      {
        g_nextloop = true;
      }
      else
      {
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
            for(i = 0; i < g_numrampADCchannels; i++)
            {
              SPI.beginTransaction(adcspi);
              digitalWrite(adc, LOW);
              SPI.transfer(ADC_CHDATA | ADC_REGREAD | g_ADCchanselect[i]); //Read channel data register
              SERIALPORT.write(SPI.transfer(0)); // Read/write first byte
              SERIALPORT.write(SPI.transfer(0)); // Read/write second byte
              digitalWrite(adc, HIGH);
              SPI.endTransaction();
            }
            
            detachInterrupt(digitalPinToInterrupt(drdy));            
          }
          else
          {
            //get next linear ramp DAC step ready if this isn't the last sample
            for(i = 0; i < g_numrampDACchannels; i++)
            {
              g_DACramppoint[i] += g_DACstep[i];
              DACintegersend(g_DACchanselect[i], (g_DACramppoint[i] / BIT47));
            }
            //get next arbitrary ramp DAC step ready
            for(i = 0; i < g_numargramps; i++)
            {
              if(g_stepcount < g_argramp[i].numsetpoints)
              {
                for(j = 0; j < g_argramp[i].numDACchannels; j++)
                {
                  DACintegersend(g_argramp[i].DACchanselect[j], g_argramp[i].setpoint[g_stepcount]);
                }
              }
            }
          }                  
        }
      }
   }
}

void g_configurerampADCchannels(void)
{
  uint8_t i;
  for(i = 1; i <= g_numrampADCchannels; i++) // Configure ADC channels, the first channel to sample gets configured last
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  {
    SPI.transfer(ADC_CHSETUP | g_ADCchanselect[g_numrampADCchannels - i]);//Access channel setup register
    SPI.transfer(ADC_CHSETUP_RNG10BI | ADC_CHSETUP_ENABLE);//set +/-10V range and enable for continuous mode
    SPI.transfer(ADC_CHMODE | g_ADCchanselect[g_numrampADCchannels - i]);   //Access channel mode register
    SPI.transfer(ADC_MODE_CONTCONV | ADC_MODE_CLAMP);  //Continuous conversion with clamping   

    #ifdef DEBUGRAMP
    SERIALPORT.print("ADC ch: ");
    SERIALPORT.print(g_ADCchanselect[g_numrampADCchannels - i]);
    SERIALPORT.println(" selected");
    #endif
  }
  SPI.transfer(ADC_IO); //Write to ADC IO register
  SPI.transfer(ADC_IO_RDYFN | ADC_IO_SYNC | ADC_IO_P1DIR); //Change RDY to only trigger when all channels complete, and wait for sync, P1 as input
  digitalWrite(adc, HIGH);
  SPI.endTransaction();
}


void g_configurerampDACchannels(void)
{
  uint8_t i;
  for(i = 0; i < g_numrampDACchannels; i++)
  {
    g_DACramppoint[i] = (int64_t)g_DACstartpoint[i] * BIT31;
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
}

void g_rampadcidle(void)
{
  uint8_t i;
  SPI.beginTransaction(adcspi);
  digitalWrite(adc, LOW);
  SPI.transfer(ADC_IO); // Write to ADC IO register
  SPI.transfer(ADC_IO_DEFAULT | ADC_IO_P1DIR); // Change RDY to trigger when any channel complete
  for(i = 0; i < NUMADCCHANNELS; i++)
  {
    SPI.transfer(ADC_CHSETUP | i); // Access channel setup register
    SPI.transfer(ADC_CHSETUP_RNG10BI); // set +/-10V range and disable for continuous mode
    SPI.transfer(ADC_CHMODE | g_ADCchanselect[i]); // Access channel mode register
    SPI.transfer(ADC_MODE_IDLE); // Set ADC to idle
  }
  digitalWrite(adc, HIGH);
  SPI.endTransaction();

}


//////////////////////
/// Arbitrary RAMP ///
//////////////////////
//ADD_RAMP,<ramp number, 0 indexed)>,<Setpoint 0 in mV>,….<Setpoint n in mV>
//Maximum ~1024 chars per call, due to possible serial buffer overrun
void add_ramp(InCommand *incommand)
{
  if(incommand->paramcount < 3)
  {
    syntax_error();
    return;
  }
  uint8_t rampnumber = atoi(incommand->token[1]);
  if(rampnumber >= ARGMAXRAMPS)
  {
    SERIALPORT.print("ERROR, Max ramps = ");
    SERIALPORT.println(ARGMAXRAMPS);
    return;
  }
  uint32_t numsetpoints = (incommand->paramcount - 2);
  
  if((numsetpoints + g_argramp[rampnumber].numsetpoints) > ARGMAXSETPOINTS)
  {
    SERIALPORT.print("ERROR, Max setpoints = ");
    SERIALPORT.println(ARGMAXSETPOINTS);
    return;
  }
  //check for voltage and setpoint range errors before adding the ramp
  for(uint32_t i = 0; i < numsetpoints; i++)
  {
    float setvolt = atof(incommand->token[i + 2]) / 1000.0;
    if(abs(setvolt) > g_dac_full_scale)
    {
      range_error();
      return;
    }    
  }
  //all good, add the ramp
  for(uint32_t i = 0; i < numsetpoints; i++)
  {
    g_argramp[rampnumber].setpoint[i + g_argramp[rampnumber].numsetpoints] = voltageToInt16(atof(incommand->token[i + 2]) / 1000.0);
    
  }
  send_ack();
  g_argramp[rampnumber].numsetpoints += numsetpoints;
  SERIALPORT.print("RAMP,");
  SERIALPORT.print(rampnumber);
  SERIALPORT.print(",");
  SERIALPORT.println(g_argramp[rampnumber].numsetpoints);
#ifdef DEBUGRAMP  
  for(uint32_t i = 0; i < g_argramp[rampnumber].numsetpoints; i++)
  {
    SERIALPORT.print("Setpoint ");
    SERIALPORT.print(i);
    SERIALPORT.print(" = ");
    SERIALPORT.println(g_argramp[rampnumber].setpoint[i]);
  }
  
  SERIALPORT.print(F("Free RAM = ")); //F function does the same and is now a built in library, in IDE > 1.0.0
  SERIALPORT.println(freeMemory(), DEC);  // print how much RAM is available.
#endif
}


//CLR_RAMP,<ramp number>
//Returns RAMP,<ramp number>,0
void clr_ramp(InCommand *incommand)
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  //uint8_t wavenumber = DB[1].toInt();
  uint8_t rampnumber = atoi(incommand->token[1]);
  if(rampnumber >= ARGMAXRAMPS)
  {
    //SERIALPORT.print("ERROR, Max waveforms = ");
    //SERIALPORT.println(AWGMAXWAVES);
    range_error();
    return;
  }
  send_ack();
  g_argramp[rampnumber].numsetpoints = 0;
  
  SERIALPORT.print("RAMP,");
  SERIALPORT.print(rampnumber);
  SERIALPORT.println(",0");   
  
}

//CHECK_RAMP,<ramp number>
//Returns RAMP,<ramp number>,<total number of setpoints>
void check_ramp(InCommand *incommand) 
{
  if(incommand->paramcount != 2)
  {
    syntax_error();
    return;
  }
  //uint8_t wavenumber = DB[1].toInt();
  uint8_t rampnumber = atoi(incommand->token[1]);
  if(rampnumber >= ARGMAXRAMPS)
  {
    range_error();
    //SERIALPORT.print("ERROR, Max waveforms = ");
    //SERIALPORT.println(AWGMAXWAVES);
    return;
  }
  send_ack();
  SERIALPORT.print("RAMP,");
  SERIALPORT.print(rampnumber);
  SERIALPORT.print(",");
  SERIALPORT.println(g_argramp[rampnumber].numsetpoints);
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
  bool factory = false;

	if(incommand->paramcount == 3)
	{
		if(strcmp(incommand->token[2], "FACTORY") == 0)
    {
      if(digitalRead(EEPROM_WP_PIN) == HIGH)
  	  {
    	  SERIALPORT.println("WRITE_PROTECTED");
    	  return;
  	  }
      factory = true;
    }
    else
    {
    syntax_error();
    return;
    }
	}

	send_ack();
	writeeepromdaccal(ch, dacocal[ch], dacgcal[ch], factory);
	
	SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.print(dacocal[ch]);
  SERIALPORT.print(",");
  SERIALPORT.print(dacgcal[ch]);
  SERIALPORT.println(",SAVED");
}

void read_dac_cal_eeprom(InCommand * incommand)
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
  bool factory = false;

	if(incommand->paramcount == 3)
	{
		if(strcmp(incommand->token[2], "FACTORY") == 0)
    {
      factory = true;
    }
    else
    {
    syntax_error();
    return;
    }
	}

	send_ack();
	readeepromdaccal(ch, &dacocal[ch], &dacgcal[ch], factory);
  writeDACoffset(ch, dacocal[ch]);
  writeDACgain(ch, dacgcal[ch]);
	SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.print(dacocal[ch]);
  SERIALPORT.print(",");
  SERIALPORT.println(dacgcal[ch]);
}

void write_adc_cal_eeprom(InCommand * incommand)
{
  if((incommand->paramcount != 2) && (incommand->paramcount != 3))
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
  bool factory = false;

	if(incommand->paramcount == 3)
	{
		if(strcmp(incommand->token[2], "FACTORY") == 0)
    {
      if(digitalRead(EEPROM_WP_PIN) == HIGH)
  	  {
    	  SERIALPORT.println("WRITE_PROTECTED");
    	  return;
  	  }
      factory = true;
    }
    else
    {
    syntax_error();
    return;
    }
	}
	send_ack();

  uint32_t zeroscale = readADCzerocal(ch);
  uint32_t fullscale = readADCfullcal(ch);

  uint8_t fw = readADCfw(ch);
	writeeepromadccal(ch, fw, zeroscale, fullscale, factory);
	
	SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.print(zeroscale);
  SERIALPORT.print(",");
  SERIALPORT.print(fullscale);
  SERIALPORT.println(",SAVED");
}

void read_adc_cal_eeprom(InCommand * incommand)
{
  if((incommand->paramcount != 2) && (incommand->paramcount != 3))
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
  bool factory = false;

	if(incommand->paramcount == 3)
	{
		if(strcmp(incommand->token[2], "FACTORY") == 0)
    {
      factory = true;
    }
    else
    {
    syntax_error();
    return;
    }
	}

	send_ack();
  uint32_t zeroscale, fullscale;
  uint8_t fw = readADCfw(ch);
	readeepromadccal(ch, fw, &zeroscale, &fullscale, factory);
  writeADCcal(ch, zeroscale, fullscale);
	SERIALPORT.print("ch");
  SERIALPORT.print(ch);
  SERIALPORT.print(",");
  SERIALPORT.print(zeroscale);
  SERIALPORT.print(",");
  SERIALPORT.println(fullscale);
}

void init_all_eeprom_values(InCommand * incommand)
{
  bool factory = false;
  if((incommand->paramcount != 1) && (incommand->paramcount != 2))
  {
    syntax_error();
    return;
  }
  if(incommand->paramcount == 2)
	{
		if(strcmp(incommand->token[1], "FACTORY") == 0)
    {
      if(digitalRead(EEPROM_WP_PIN) == HIGH)
  	  {
    	  SERIALPORT.println("WRITE_PROTECTED");
    	  return;
  	  }
      factory = true;
    }
    else
    {
    syntax_error();
    return;
    }
	}
  send_ack();
  SERIALPORT.print("INITIALIZING ALL EEPROM VALUES...");
  uint8_t ch, fw;
  for(ch = 0; ch < NUMADCCHANNELS; ch++)
  {
    for(fw = 0; fw < EEPROM_ADC_NUM_FWS; fw++)
    {
      writeeepromadccal(ch, fw, ADC_DEFAULT_ZEROSCALE, ADC_DEFAULT_FULLSCALE, factory);
    }
    SERIALPORT.print("Initialized ADC CH");
    SERIALPORT.print(ch);
    SERIALPORT.print(",");
  }
  for(ch = 0; ch < NUMDACCHANNELS; ch++)
  {
    writeeepromdaccal(ch, DAC_DEFAULT_OFFSET, DAC_DEFAULT_GAIN, factory);
    SERIALPORT.print("Initialized DAC CH");
    SERIALPORT.print(ch);
    SERIALPORT.print(",");
  }
  SERIALPORT.println("WRITE_FINISHED");
}

void cal_all_adc_eeprom_with_dac(InCommand * incommand)
{
  bool factory = false;
  if((incommand->paramcount != 1) && (incommand->paramcount != 2))
  {
    syntax_error();
    return;
  }
  if(incommand->paramcount == 2)
	{
		if(strcmp(incommand->token[1], "FACTORY") == 0)
    {
      if(digitalRead(EEPROM_WP_PIN) == HIGH)
  	  {
    	  SERIALPORT.println("WRITE_PROTECTED");
    	  return;
  	  }
      factory = true;
    }
    else
    {
    syntax_error();
    return;
    }
	}
  send_ack();
  SERIALPORT.print("CAL ALL ADC EEPROM VALUES WITH DAC...");
  uint8_t ch, fw;
  for(fw = 2; fw < EEPROM_ADC_NUM_FWS; fw++)
  {
      uint32_t zeroscale[NUMADCCHANNELS], fullscale[NUMADCCHANNELS];
      //write the filter word (conversion time) and set the DACs to 0
      for(ch = 0; ch < NUMADCCHANNELS; ch++)
      {
        writeADCfw(ch, fw);
        uint8_t readfw = readADCfw(ch);
        if(readfw != fw)        
        {
          SERIALPORT.println("ADC FW ERROR!");
          return;
        }
        writeDAC(ch, 0.0, true); // mV
      }
      delayMicroseconds(CAL_SETTLE_TIME);
      
      //get each ch zero scale value
      for(ch = 0; ch < NUMADCCHANNELS; ch++)
      {
        zeroscale[ch] = cal_adc_ch_zero_scale(ch);        
      }
      //set DACs to fullscale
      for(ch = 0; ch < NUMADCCHANNELS; ch++)
      {
        writeDAC(ch, 10000.0, true); // mV
      }
      delayMicroseconds(CAL_SETTLE_TIME); // wait cal settle time
      //get each ch full scale value 
      for(ch = 0; ch < NUMADCCHANNELS; ch++)
      {
        fullscale[ch] = cal_adc_ch_full_scale(ch);
      }
      //write each channel to eeprom
      for(ch = 0; ch < NUMADCCHANNELS; ch++)
      {
        writeeepromadccal(ch, fw, zeroscale[ch], fullscale[ch], factory);
      }
      SERIALPORT.print("FW ");
      SERIALPORT.print(fw);
      SERIALPORT.print(" SAVED, ");
  }  
  SERIALPORT.println("CALIBRATION_FINISHED");
  
}