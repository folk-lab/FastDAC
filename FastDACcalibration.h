#define IDSTRING "DAC-ADC_AD7734-AD5764_UNIT3"

#define MAXNUMCALS 8

// Default conversion time is 395 us. This is set at power up.

int8_t defaultDACoffset[MAXNUMCALS] = {2, 6, 3, -4, 0, 0, -6, 0};

int8_t defaultDACgain[MAXNUMCALS] = {21, 20, 21, 21, 19, 19, 20, 19};

int32_t defaultADCzeroscale[MAXNUMCALS] = {751, 826, 1667, 1074};

int32_t defaultADCfullscale[MAXNUMCALS] = {2100927, 2100757, 2101472, 2100672};
