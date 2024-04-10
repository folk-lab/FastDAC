
#include "FastDACdefs.h"

//#define MAXNUMCALS 8

// Default conversion time is 395 us. This is set at power up.

int8_t dacocal[NUMDACCHANNELS] = {0};

int8_t dacgcal[NUMDACCHANNELS] = {0};

int32_t adczscal[NUMADCCHANNELS] = {0};

int32_t adcfscal[NUMADCCHANNELS] = {0};
