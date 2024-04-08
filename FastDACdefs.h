#ifndef FASTDACDEFS_H
#define FASTDACDEFS_H

#define OPTICAL //Comment this if still using old USB version

#ifdef OPTICAL
#define SERIALPORT Serial1
#else
#define SERIALPORT SerialUSB
#endif


//AD7734 Register addresses
#define ADC_IO 0x01
#define ADC_REV 0x02
#define ADC_TEST 0x03
#define ADC_STAT 0x04
#define ADC_CHECKSUM 0x05

#define ADC_CHDATA 0x08

#define ADC_CHZEROSCALECAL 0x10
#define ADC_CHFULLSCALECAL 0x18
#define ADC_CHSTAT 0x20
#define ADC_CHSETUP 0x28
#define ADC_CHCONVTIME 0x30
#define ADC_CHMODE 0x38

#define ADC_REGREAD 0x40

//AD7734 Register contents

#define ADC_IO_P0 0x80
#define ADC_IO_P1 0x40
#define ADC_IO_P0DIR 0x20
#define ADC_IO_P1DIR 0x10
#define ADC_IO_RDYFN 0x08
#define ADC_IO_SYNC 0x01
#define ADC_IO_DEFAULT 0x00

#define ADC_CHSTAT_CHMASK 0x60 >> 5
#define ADC_CHSTAT_0P0 0x10
#define ADC_CHSTAT_RDYP1 0x08
#define ADC_CHSTAT_NOREF 0x04
#define ADC_CHSTAT_SIGN 0x02
#define ADC_CHSTAT_OVR 0x01

#define ADC_CHSETUP_STATOPT 0x10
#define ADC_CHSETUP_ENABLE 0x08
#define ADC_CHSETUP_RNG10BI 0x00
#define ADC_CHSETUP_RNG10 0x01
#define ADC_CHSETUP_RNG5BI 0x02
#define ADC_CHSETUP_RNG5 0x03

#define ADC_MAX_CONVERT_TIME 2686


//AD7734 Main modes
#define ADC_MODE_IDLE 0x00
#define ADC_MODE_CONTCONV 0x20
#define ADC_MODE_SINGLECONV 0x40
#define ADC_MODE_STANDBY 0x60
#define ADC_MODE_SELFZEROCAL 0x80
#define ADC_MODE_SYSZEROCAL 0xC0
#define ADC_MODE_SYSFULLCAL 0xE0

//AD7734 mode option bits
#define ADC_MODE_DUMP 0x08
#define ADC_MODE_CONTRD 0x04
#define ADC_MODE_24BIT 0x02
#define ADC_MODE_CLAMP 0x01

//AD5764 Input register
#define DAC_READ 0x80
#define DAC_FUNC 0x00
#define DAC_DATA 0x10
#define DAC_COARSEGAIN 0x18
#define DAC_FINEGAIN 0x20
#define DAC_OFFSET 0x28

//AD5764 Function register
#define DAC_FUNC_LOCALGNDADJ 0x20
#define DAC_FUNC_D1DIR 0x10
#define DAC_FUNC_D1VAL 0x08
#define DAC_FUNC_D0DIR 0x04
#define DAC_FUNC_D0VAL 0x02
#define DAC_FUNC_SDODISABLE 0x01

#define CHECK_CLOCK 0x1
#define CHECK_SYNC 0x2

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

#endif