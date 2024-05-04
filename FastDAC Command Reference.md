# FASTDAC REFERENCE MANUAL

Every command sent to the FastDAC should be a string (ASCII characters). Every string should start with the operation to execute and end with the character that determines the end of the string, in this case `\r` (carriage return), `\n` (newline), or a combination of both.

General Syntax:
`{operation},{data (varies with the operation)}\r(carriage return)`

Where items enclosed in braces `{}` must be substituted for values.

Where a DAC or ADC channel is specified, they are zero-indexed; An 8-channel DAC is addressed as channels 0-7, and a 4-channel ADC is addressed as channels 0-3.

The FastDAC can execute the following operations: `*IDN?`, `*RDY?`, `GET_ADC`, `RAMP_SMART`, `INT_RAMP`, `SPEC_ANA`, `CONVERT_TIME`, `READ_CONVERT_TIME`, `GET_DAC`, `ADC_CH_ZERO_SC_CAL`, `ADC_CH_FULL_SC_CAL`, `CAL_ADC_WITH_DAC`, `WRITE_ADC_CAL`, `READ_ADC_CAL`, `DAC_OFFSET_ADJ`, `DAC_GAIN_ADJ`, `DAC_RESET_CAL`, `FULL_SCALE`, `SET_MODE`, `ARM_SYNC`, `CHECK_SYNC`, `ADD_WAVE`, `CLR_WAVE`, `CHECK_WAVE`, `AWG_RAMP`, `START_PID`, `STOP_PID`, `SET_PID_TUNE`, `SET_PID_SETP`, `SET_PID_LIMS`, `SET_PID_DIR`, `SET_PID_SLEW`

When the FastDAC does not recognize the operation, it return the string `NOP`, which stands for "No Operation"

If the operation is recognized but some parameters are incorrect, it will return `SYNTAX_ERROR` or `RANGE_ERROR`

If the operation is recognized and all parameters are correct, it will return `ACK\r\n` followed by the expected response which is ended by `\r\n`.


## `*IDN?` and `*RDY?`

`*IDN?` returns the string `FASTDAC_UNIT-{unit id}_{firmware version}`

Example:  
`*IDN?`

Returns:  
`FASTDAC_UNIT-FOLK2_SERVICE`

`*RDY?` returns the string `READY` when the DAC-ADC is ready for a new operation.

Example:  
`*RDY?`

Returns:  
`READY`

## GET_ADC

`GET_ADC` returns the voltage in mV read by an input ADC channel. 

Syntax:  
`GET_ADC,{adc channel}`

Example:  
`GET_ADC,0`

Returns:  
`3.9668`

## RAMP_SMART

Ramps one DAC channel to a specified setpoint **in mV** at a given ramprate in 1ms steps. It looks up the current DAC value internally to make sure there are no sudden jumps in voltage. Internally it calls RAMP1 to do the actual ramp.

A ramp can be stopped at any time by sending the command `STOP`.

Syntax:  
`RAMP_SMART,{dac_channel},{setpoint},{ramprate in mV/s}`

Example (to ramp DAC3 to 4000mV at 1000mV/s):  
`RAMP_SMART,3,4000,1000`  

Returns:  
`RAMP_FINISHED`

## INT_RAMP

`INT_RAMP` ramps the specified DAC channels from the initial voltages to the final voltages and reads the specified ADC channels in a synchronized manner in a specified number of steps. It uses the ADC in a continuous sampling mode, and therefore there is no delay between updating the DAC output and acquiring the next ADC samples. While the ADC is acquiring the current samples, the next DAC step output values are being preloaded into the DAC, to be output synchronously as soon as the ADC samples are ready. The sample rate of this function is consistent, and capable of the maximum throughput of the ADC even while updating up to 8 DAC channels.

Oversampling, in order to do additional filtering by the control PC, is achieved by specifying a large number of steps (max 2^31-1). Each DAC channel's output value is treated as a 64-bit integer, and is scaled back to a 16-bit integer before being sent to the DAC. This allows a large number of samples to be taken without actually incrementing the 16-bit DAC output. 

A ramp can be stopped at any time by sending the command `STOP`.

Syntax (ALL mV):  
`INT_RAMP,{DAC channels},{ADC channels},{initial DAC voltage 1},{...},{initial DAC voltage n},{final DAC voltage 1},{...},{final dac voltage n},{# of steps}`

Do not add commas between specified channels. The `{DAC channels}` parameter can be specified as `N` if no DAC channels should ramp

Example (ramping DAC channels 0, 6, and 7, while reading from ADC channels 0, 2, and 3):  
`INT_RAMP,067,023,-1000,-2000,-3000,3000,4000,5000,1000`

Returns:  
`{# of steps x number of selected adc channels x 16-bit integer samples}RAMP_FINISHED`

## SPEC_ANA

`SPEC_ANA` is similar to `INT_RAMP`, but it does not ramp or change any DAC channels. The ADC channels to sample are specified, as well as the number of samples to take from each ADC channel.
An acquisition can be stopped at any time by sending the command `STOP`.

Syntax:  
`SPEC_ANA,{adc channels},{number of samples to take}`

Do not add commas between specified channels.

Example (read from ADC channels 0, 2, and 3 for 3000 samples each):  
`SPEC_ANA,023,3000`

Returns:  
`{# of samples x number of selected adc channels x 16-bit integer samples}READ_FINISHED`

## CONVERT_TIME

`CONVERT_TIME` sets the conversion time, in µs, for each ADC channel, which is the time it takes to digitize the analog signal. The sum of the conversion times of all selected channels will determine overall sample rate.  Shorter conversion times result in more measured noise; Refer to the AD7734 datasheet for typical noise vs conversion times (chopping is always enabled). The correct calibration for the selected channel and conversion time will be loaded from the `USER` area of the EEPROM. Maximum conversion time: 2686µs. Minimum conversion time: 82µs. The function will return the actual closest possible setting.

Syntax:  
`CONVERT_TIME,{adc channel},{conversion time in µs}`

Example:  
`CONVERT_TIME,0,90`

Returns:  
`82`

## READ_CONVERT_TIME

Reads the conversion time of the given ADC Channel in µs. See above `CONVERT_TIME` for more info.

Syntax:  
`READ_CONVERT_TIME,{adc channel}`

Example:  
`READ_CONVERT_TIME,3`

Returns:  
`394`

## GET_DAC

`GET_DAC` returns the current setpoint (in mV) of the specified DAC channel.

Syntax:  
`GET_DAC,{dac channel}`

Example:  
`GET_DAC,1`

Returns:  
`0.6104`

# CALIBRATION FUNCTIONS

## ADC_CH_ZERO_SC_CAL

`ADC_CH_ZERO_SC_CAL` performs a system zero calibration on the specified ADC channel, where the input of the selected ADC channel should be held at 0VDC. This will calibrate out any offsets in the system due to the ADC itself, or any signal conditioning on the input. The function will return the calibration constant that has been stored to the ADC system zero-scale cal register, which can be noted or saved for restoring a calibration later. It is preferable to repeat this calibration any time the conversion time is changed using `CONVERT_TIME`, especially when using conversion times less than 300µs.

Syntax:  
`ADC_CH_ZERO_SC_CAL,{adc channel}`

Example:  
`ADC_CH_ZERO_SC_CAL,1`

Returns:  
`ch{adc channel},{zeroscale}`

## ADC_CH_FULL_SC_CAL

`ADC_CH_FULL_SC_CAL` performs a system full-scale calibration on the specified ADC channel, where the input of the selected ADC channel should be held at positive full-scale (typically 10VDC). This will calibrate out gain errors in the system due to the ADC itself, or any signal conditioning on the input. The function will return the calibration constant that has been stored to the ADC system full-scale cal register, which can be noted or saved for restoring a calibration later. It is preferable to repeat this calibration any time the conversion time is changed using `CONVERT_TIME`, especially when using conversion times less than 300µs.

Syntax:  
`ADC_CH_FULL_SC_CAL,{adc channel}`

Example:  
`ADC_CH_ZERO_SC_CAL,1`

Returns:  
`ch{adc channel},{fullscale}`

## CAL_ADC_WITH_DAC

`CAL_ADC_WITH_DAC` makes use of `ADC_CH_ZERO_SCALE_CAL`, `ADC_CH_FULL_SCALE_CAL`, and an internal DAC set function to calibrate ADC channels 0-3 using DAC channels 0-3. Before running the command, DAC channels 0-3 should be connected to ADC channels 0-3 using BNC cables. The function will automatically set the DACs to 0VDC and then 10VDC during the calibration cycle. It will return the zero-scale and full-scale calibration registers for each ADC channel.

Syntax:  
`CAL_ADC_WITH_DAC`

Example:  
`CAL_ADC_WITH_DAC`

Returns:  
`ch0,{zeroScale ch0},ch1,{zeroScale ch1},ch2,{zeroScale ch2},ch3,{zeroScale ch3},ch0,{fullScale ch0},ch1,{fullScale ch1},ch2,{fullScale ch2},ch3,{fullScale ch3}`

## WRITE_ADC_CAL

`WRITE_ADC_CAL` is used to restore previously noted or saved ADC per-channel calibration register values, for both the zero and full scale calibration points.

Syntax:  
`WRITE_ADC_CAL,{adc channel},{zero scale cal value},{full scale cal value}`

Example:  
`WRITE_ADC_CAL,1,826,2100756`

Returns:  
`CALIBRATION_CHANGED`  

## READ_ADC_CAL

`READ_ADC_CAL` is used to display the currently loaded per-channel ADC calibration register values, for both the zero and full scale calibration points.

Syntax:  
`READ_ADC_CAL,{adc channel}`

Example:  
`READ_ADC_CAL,1`

Returns:  
`ch{adc channel},{zeroscale},ch{adc channel},{fullscale}`  

## DAC_OFFSET_ADJ

`DAC_OFFSET_ADJ` uses the AD5764 DAC's internal registers to remove any zero offset for each channel. The DAC provides a register per channel, which allow zero adjustment in increments of 1/8 of an LSB (38µV for a +/-10V range) in the range of -32 to +31 LSBs. Before performing a calibration, `DAC_RESET_CAL` should be used to zero the calibration registers. To perform the calibration, a reference DMM is connected to the DAC channel to be calibrated. With the DAC channel set at 0V, the DMM reading is noted and input into the function, which will automatically calculate and return the appropriate register values.

Syntax:  
`DAC_OFFSET_ADJ,{dac channel},{dmm zero reading}`

Example (with a DMM reading of -0.000070):  
`DAC_OFFSET_ADJ,0,-0.000070`

Returns:  
`ch{dac channel},{offset stepsize},{offset register}`  

## DAC_GAIN_ADJ

`DAC_GAIN_ADJ` uses the AD5764 DAC's internal registers to remove any gain error for each channel. The DAC provides a register per channel, which allow gain adjustment in increments of 1/2 of an LSB (153µV for a +/-10V range) in the range of -32 to +31 LSBs. Before performing a calibration, `DAC_RESET_CAL` should be used to zero the calibration registers, and the `DAC_OFFSET_ADJ` calibration performed first. To perform the gain calibration, a reference DMM is connected to the DAC channel to be calibrated. With the DAC channel set at negative scale (-10VDC), the DMM reading is noted and the calculated error is input into the function, which will automatically calculate and return the appropriate register values.

Syntax:  
`DAC_GAIN_ADJ,{dac channel},{dmm error from negative full scale}`

Example (with DMM reading of -9.9969V):  
`DAC_GAIN_ADJ,0,0.00310`

Returns:  
`ch{dac channel},{gain stepsize},{gain register}`  

## DAC_RESET_CAL

`DAC_RESET_CAL` is used to zero the calibration registers for a specified channel. This should be used before attempting to recalibrate a DAC channel with a reference DMM.

Syntax:  
`DAC_RESET_CAL,{dac channel}`

Example:  
`DAC_RESET_CAL,0`

Returns:  
`CALIBRATION_RESET`

## FULL_SCALE

`FULL_SCALE` can be used to set the full-scale range of the DAC outputs to something other than the default 10V. This could be useful if a voltage divider or external circuit is used to alter the range of the DACs and you want to have this scale factored in automatically.

Syntax:  
`FULL_SCALE,{DAC positive full-scale range in Volts}`

Example (Setting the full scale range to 5V):  
`FULL_SCALE,5.0`

Returns:  
`FULL_SCALE_UPDATED`

# EEPROM/CALIBRATION FUNCTIONS

The onboard EEPROM of the FastDAC is used to store ADC and DAC calibration data, as well as the ID string for the units. There are 2 areas of the EEPROM, `USER` and `FACTORY`. In order to write to the `FACTORY` area, a write-protect jumper from pin 13 to GND must be installed inside the unit, on the Arduino. It is recommended to only install this when necessary to avoid accidentally overwriting the `FACTORY` area, and instead use the `USER` area for most calibration functions.

The EEPROM stores a calibration for every possible conversion time for each ADC channel, in both the `FACTORY` and `USER` areas. The DAC channels also have a calibration in each area, but only a single calibration as there is no conversion time associated with the DAC. The ID string only exists in the `FACTORY` area. For reading/writing to `FACTORY` areas, most functions require the extra `FACTORY` parameter added to end of the command.

When the FastDAC is first powered up, the calibrations are loaded from the `USER` area for the default conversion time of 394µs. When the ADC `CONVERT_TIME` function is used, the FastDAC automatically loads the associated calibration from the `USER` area.

## WRITE_ID_EEPROM

`WRITE_ID_EEPROM` is used to write a new `{unit id}` string to the FastDAC, which will be returned by the `*IDN?` command, in the format `FASTDAC_UNIT-{unit id}_{firmware version}`. The write protect jumper must be installed.

Syntax:  
`WRITE_ID_EEPROM,{unit id}`

Example:  
`WRITE_ID_EEPROM,FOLK6`

Returns:
`ID_SAVED`

## INIT_ALL_EEPROM_VALUES

`INIT_ALL_EEPROM_VALUES` is used to overwrite all ADC and DAC cals in the EEPROM with default values. This would normally be used when a FastDAC is powered up or programmed for the first time and does not have valid calibration values in the EEPROM. This will result in the default un-calibrated behaviour from the ADCs and DACs

Syntax:  
`INIT_ALL_EEPROM_VALUES,FACTORY (optional)`

Example:  
`INIT_ALL_EEPROM_VALUES`

Returns:
`INITIALIZING ALL EEPROM VALUES...Initialized ADC CH0,{...}Initialized DAC CH7,WRITE_FINISHED`

## WRITE_DAC_CAL_EEPROM

`WRITE_DAC_CAL_EEPROM` is used to save the current calibration of a DAC channel to the `USER` or `FACTORY` area of the eeprom.

Syntax:  
`WRITE_DAC_CAL_EEPROM,{DAC channel cal to save},FACTORY (optional)`

Returns:
`ch{DAC channel},{offset cal},{gain cal},SAVED`

Example:  
`WRITE_DAC_CAL_EEPROM,2`

Returns:  
`ch2,-6,25,SAVED`

## READ_DAC_CAL_EEPROM

`READ_DAC_CAL_EEPROM` is used to load a DAC channel calibration from the `USER` or `FACTORY` area of the eeprom. The DAC channel automatically starts using the newly loaded calibration.

Syntax:  
`READ_DAC_CAL_EEPROM,{DAC channel cal to read},FACTORY (optional)`

Returns:
`ch{DAC channel},{offset cal},{gain cal}`

Example:  
`READ_DAC_CAL_EEPROM,2,FACTORY`

Returns:  
`ch2,-6,25`

## WRITE_ADC_CAL_EEPROM

`WRITE_ADC_CAL_EEPROM` is used to save the current calibration of a ADC channel to the `USER` or `FACTORY` area of the eeprom. The channel's conversion time is automatically checked and the appropriate memory location written to.

Syntax:  
`WRITE_ADC_CAL_EEPROM,{ADC channel cal to save},FACTORY (optional)`

Returns:
`ch{ADC channel},{zero cal},{fullscale cal},SAVED`

Example:  
`WRITE_ADC_CAL_EEPROM,1`

Returns:  
`ch1,8388887,2096806,SAVED`

## READ_ADC_CAL_EEPROM

`READ_ADC_CAL_EEPROM` is used to save load an ADC channel calibration from the `USER` or `FACTORY` area of the eeprom. The channel's conversion time is automatically checked and the appropriate memory location read from. The ADC channel automatically starts using the newly loaded calibration.

Syntax:  
`READ_ADC_CAL_EEPROM,{ADC channel cal to read},FACTORY (optional)`

Returns:
`ch{ADC channel},{zero cal},{fullscale cal}`

Example:  
`READ_ADC_CAL_EEPROM,3,FACTORY`

Returns:  
`ch3,82,2096886`

## CAL_ALL_ADC_EEPROM_WITH_DAC

`CAL_ALL_ADC_EEPROM_WITH_DAC` uses DAC channels 0-3 connected to ADC channels 0-3 to calibrate all 125 possible conversion times of each ADC channels and save the calibrations to EEPROM. The user must first externally connect DAC channels 0-3 to ADC channels 0-3.

Syntax:  
`CAL_ALL_ADC_EEPROM_WITH_DAC,FACTORY (optional)`

Example:  
`CAL_ALL_ADC_EEPROM_WITH_DAC`

Returns:
`CAL ALL ADC EEPROM VALUES WITH DAC...FW 2 SAVED,{....}, FW 127 SAVED, CALIBRATION_FINISHED`

# MASTER/SLAVE FUNCTIONS

The FastDAC has optical clock and sync inputs and outputs to allow synchronization between units. Functions that support sync are `SPEC_ANA`, `INT_RAMP`, and `AWG_RAMP`. Each unit must have the same number of ADC channels being sampled, and the conversion time must be the same. One unit must be set to MASTER, all others to SLAVE. Units can be daisy-chained with the MASTER unit's CLK_OUT and SYNC_OUT connected to the first SLAVE unit's CLK_IN and SYNC_IN, and so on for each additional slave. The general sequence would be:
1. PC uses `SET_MODE` to select one unit as MASTER, and all others as SLAVE. It can take up to 20 seconds for the PLL to relock switching between MASTER and SLAVE if the clock frequency is substantially different, which can be checked with `CHECK_SYNC`
2. PC sends `ARM_SYNC` to MASTER
3. PC sends `INT_RAMP` (for example) to each SLAVE unit, which will `ACK` when ready, and wait for the SYNC signal from MASTER
4. PC sends `INT_RAMP` to MASTER unit.
5. PC waits for `RAMP_FINISHED` from each unit
6. The process can be repeated from step 2 (`ARM_SYNC`)

Note for step 3, the SLAVE unit will send an `ACK` when it is ready. If the ramp is called too quickly on the MASTER after the SLAVE (~2ms DAC settling time) the sync signal may be missed and `SYNC_NOT_READY` will be returned.

## SET_MODE

`SET_MODE` is used to select the mode for each unit, valid modes are `MASTER`, `SLAVE`, and `INDEP`. When in `INDEP` mode, the unit will not check or wait for a valid SYNC signal before starting a sequence but still checks the CLK. The default mode on power-up is `INDEP`.

Syntax:  
`SET_MODE,{mode}`

Example:  
`SET_MODE,INDEP`

Returns:  
`INDEP_SET`

## ARM_SYNC

`ARM_SYNC` is sent from the PC to the MASTER, before sending the sampling command (ie. `INT_RAMP`) to each SLAVE, and lastly to the MASTER. The PC should verify that all units have finished the previous sequence before sending `ARM_SYNC`. If the mode is `INDEP`, or `SLAVE`, this command is not necessary.

Syntax:  
`ARM_SYNC`

Example:  
`ARM_SYNC`

Returns:  
`SYNC_ARMED`

## DISARM_SYNC

`DISARM_SYNC` is sent from the PC to the MASTER, it should rarely have to be used. One possible use is if the PC used `ARM_SYNC` on the MASTER before all SLAVE units had finished their previous sequence, which would cause the ADC to pause sampling. This would allow the SLAVE units to finish their sequence.

Syntax:  
`DISARM_SYNC`

Example:  
`DISARM_SYNC`

Returns:  
`SYNC_DISARMED`

## CHECK_SYNC

`CHECK_SYNC` is used to determine the state of the clock PLL, and the SYNC signal. Possible returned values are:

`CLOCK_NOT_READY` - The clock PLL is still trying to lock to the input clock signal.

`SYNC_NOT_READY` - This message will only appear if in `MASTER` or `SLAVE` mode. For the MASTER this means `ARM_SYNC` still needs to be called, for a SLAVE it could potentially be a bad or incorrect optical SYNC signal, or `ARM_SYNC` still need to be called on the MASTER.

`CLOCK_SYNC_READY` - Both signals are acceptable for the current mode.

Syntax:  
`CHECK_SYNC`

Example:  
`CHECK_SYNC`

Returns:  
`CLOCK_NOT_READY`  
`SYNC_NOT_READY`

# AWG FUNCTIONS

The FastDAC has an arbitrary waveform mode where a sequence of DAC setpoints can be assigned to 2 separate wave arrays. Up to 100 setpoints can be configured for each wave, and the number of ADC samples to take at each setpoint is specified. When an `AWG_RAMP` sequence is started, the AWG DAC channels assigned to each wave loop through the sequence a specified number of times, then the ramp DAC channels will take a step. The AWG waveforms will repeat until the ramp DAC channels finish their ramp.

## ADD_WAVE

`ADD_WAVE` is used to configure the arbitrary DAC setpoints. For each setpoint the number of ADC samples to take is also specified (max 2^32-1). The function can be called multiple times until up to 100 setpoints have been stored, and should be sent in groups of 1024 characters or less to avoid a possible serial buffer overflow.

Syntax:  
`ADD_WAVE,{wave number (0 or 1)},{Setpoint 0 in mV},{Number of ADC samples to take at setpoint 0},{...},{Setpoint n in mV},{Number of samples to take at Setpoint n}`

Example (setting 4 setpoints with various sample lengths to wave 0):  
`ADD_WAVE,0,100.0,50,500.0,25,200.0,100,-5000.0,25`

Returns:  
`WAVE,0,4`

## CHECK_WAVE

`CHECK_WAVE` returns how many setpoints, and the total number of samples, have been configured for the specified wave number (0 or 1).

Syntax:  
`CHECK_WAVE,{wave number}`

Example (for waveform in example above):  
`CHECK_WAVE,0`

Returns:  
`WAVE,0,4,200`

## CLR_WAVE

`CLR_WAVE` resets the number of configured setpoints for a specified waveform back to 0

Syntax:  
`CLR_WAVE,{wave number}`

Example:  
`CLR_WAVE,1`

Returns:  
`WAVE,1,0`

## AWG_RAMP

Similar to `INT_RAMP` you specify which ADC channels to sample and DAC channels to ramp, as well as number of ramp steps. Additionally, you select the number of independent waveforms (currently max 2), The DAC channels assigned to each waveform, and the number of waveform repetitions at each ramp step. If the waveforms are different lengths, the repetition counter will increment when any waveform completes.

A ramp can be stopped at any time by sending the command `STOP`.

The `{DACs to ramp}` parameter can be specified as `N` if no DAC channels should ramp

Syntax:  
`AWG_RAMP,{number of independent waveforms},{DAC channels assigned to waveform 0},{...},{DAC channels assigned to waveform N},{DACs to ramp},{ADCs to sample},{Initial DAC voltage 1},{...},{Initial DAC voltage N},{Final DAC voltage 1},{...},{Final DAC voltage N},{# of waveform repetitions at each ramp step},{# of ramp steps}`

Example (Use 1 waveform, assign DAC 7 to waveform 0, Ramp DACs 1 and 3, Sample ADC 0, Start DAC 1 at -5V, Start DAC 3 at -2.5V, Finish DAC 1 at 5V, Finish DAC 3 at 2.5V, Repeat waveform 10 times at each ramp step, 100 ramp steps):  
`AWG_RAMP,1,7,13,0,-5000,-2500,5000,2500,10,100`

Returns:  
`{# of steps * number of samples in wave * number of repetitions * number of selected adc channels * 16-bit integer samples}RAMP_FINISHED`

# ARG (Arbitrary Ramp Generator) FUNCTIONS

The FastDAC has an arbitrary ramp mode where a sequence of DAC setpoints can be assigned to 4 separate ramp arrays. Up to 10000 setpoints can be configured for each ramp. When an `INT_ARG_RAMP` sequence is started, the ramp with the largest number of samples decides the `NUMBER OF STEPS` compared to a regular `INT_RAMP`. The `NUMBER OF SAMPLES PER STEP` is specified and the `ARG` DAC channels will take a step at the same time as the linear-ramp DAC channels.

## ADD_RAMP

`ADD_RAMP` is used to configure the arbitrary ramp DAC setpoints. The function can be called multiple times until up to 10000 setpoints have been stored, and should be sent in groups of 1024 characters or less to avoid a possible serial buffer overflow.

Syntax:  
`ADD_RAMP,{ramp number (0-3)},{Setpoint 0 in mV},{...},{Setpoint n in mV}`

Example (setting 8 setpoints to ramp 0):  
`ADD_RAMP,0,100.0,50.5,500.0,250.2,-200.0,100.0,-50.0,250.3`

Returns:  
`RAMP,0,8`

## CHECK_RAMP

`CHECK_RAMP` returns how many setpoints have been configured for the specified ramp number (0-3).

Syntax:  
`CHECK_RAMP,{ramp number}`

Example (for ramp in example above):  
`CHECK_RAMP,0`

Returns:  
`RAMP,0,8`

## CLR_RAMP

`CLR_RAMP` resets the number of configured setpoints for a specified ramp back to 0

Syntax:  
`CLR_RAMP,{ramp number}`

Example:  
`CLR_RAMP,1`

Returns:  
`RAMP,1,0`

## INT_ARG_RAMP

Similar to `INT_RAMP` you specify which ADC channels to sample and DAC channels to ramp, but no longer specify the `number of steps` as this is decided by the longest selected ARG ramp. Additionally, you select the number of independent ARG ramps (currently max 4), The DAC channels assigned to each ARG ramp, and the number of samples to take at each ramp step. If the ARG ramps are different lengths, the shorter ARG ramps will stay at their final setpoint until the longest ARG ramp completes.

A ramp can be stopped at any time by sending the command `STOP`.

The `{DACs to ramp}` parameter can be specified as `N` if no DAC channels should linear-ramp

Syntax:  
`INT_ARG_RAMP,{number of independent ARG ramps},{DAC channels assigned to ARG 0},{...},{DAC channels assigned to ARG N},{DACs to linear-ramp},{ADCs to sample},{Initial linear DAC voltage 1},{...},{Initial linear DAC voltage N},{Final linear DAC voltage 1},{...},{Final linear DAC voltage N},{# of samples at each ramp step}`

Example (Use 1 ARG ramp, assign DACs 0,1,2,3 to ARG 0, linear-ramp DAC 4, Sample ADC 0, Start DAC 4 at -5V, Finish DAC 4 at 5V, Take 10 samples at each ramp step):  
`INT_ARG_RAMP,1,0123,4,0,-5000,5000,10`

Returns:  
`{number of setpoints in ARG 0 * number of samples to take * number of selected adc channels * 16-bit integer samples}RAMP_FINISHED`

## AWG_ARG_RAMP

Similar to `AWG_RAMP` you specify which ADC channels, linear-ramp DAC channels, the number of independent AWG waveforms (currently max 2), The DAC channels assigned to each waveform, and the number of waveform repetitions at each ramp step. Additionally, the number of independent ARB ramps (max 4) is also selected, and the DAC channels assigned to each ARB ramp. Unlike `AWG_RAMP`, the `# of ramp steps` parameter is determined by the length of the longest ARB ramp selected.

A ramp can be stopped at any time by sending the command `STOP`.

The `{DACs to ramp}` parameter can be specified as `N` if no linear-DAC channels should ramp

Syntax:  
`AWG_ARG_RAMP,{number of independent waveforms},{DAC channels assigned to waveform 0},{...},{DAC channels assigned to waveform N},{number of independent ARG ramps},{DAC channels assigned to ARG 0},{...},{DAC channels assigned to ARG N},{DACs to ramp},{ADCs to sample},{Initial DAC voltage 1},{...},{Initial DAC voltage N},{Final DAC voltage 1},{...},{Final DAC voltage N},{# of waveform repetitions at each ramp step}`

Example (Use 1 AWG waveform, assign DAC 7 to waveform 0, use one ARG ramp, assign DAC 2 to ARG 0, Ramp DACs 1 and 3, Sample ADC 0, Start DAC 1 at -5V, Start DAC 3 at -2.5V, Finish DAC 1 at 5V, Finish DAC 3 at 2.5V, Repeat waveform 10 times at each ramp step):  
`AWG_ARG_RAMP,1,7,1,2,13,0,-5000,-2500,5000,2500,10`

Returns:  
`{# setpoints in ARG ramp * number of samples in wave * number of repetitions * number of selected adc channels * 16-bit integer samples}RAMP_FINISHED`

# PID FUNCTIONS

Somewhat experimental, the FastDAC can act as a PID controller, currently only ADC 0 acts as the input, and DAC 0 acts as the output. While in PID mode, no other commands which use the ADC should be issued (`INT_RAMP`, `SPEC_ANA`, `AWG_RAMP`, `GET_ADC`), but DAC ramping channels other than DAC 0 via `RAMP_SMART` should be OK. While in PID mode, the loops runs continuously at the selected ADC 0 conversion time, and every 10th sample from ADC 0 and the output setting of DAC 0 will be returned as little-endian 4-byte floats followed by framing sequence `0xA5 0x5A`. This formatting and framing sequence was chosen for easy testing and live tuning with RealTerm 3.0.1.44 and could be changed. The PID routines are based on this library with some modifications:

https://github.com/br3ttb/Arduino-PID-Library

All PID functions can be called live while the PID is running or before it is started.

## START_PID

`START_PID` starts the PID controller, and the stream of samples from ADC 0

Syntax:  
`START_PID`

Example:  
`START_PID`

Returns stream of:  
`{4-byte float ADC 0 in mV}{4 byte float DAC 0 in mV}{0xA5 0x5A}`

## STOP_PID

`STOP_PID` stops the PID controller and the stream of samples from ADC 0.

Syntax:  
`STOP_PID`

Example:  
`STOP_PID`

Returns:  
Nothing

## PID TUNING COMMANDS

The following commands will not send an `ACK` if the PID is running, to facilitate live tuning without corrupting the data stream.

### SET_PID_TUNE

`SET_PID_TUNE` allows setting the proportional, integral, and derivative coefficients of the PID controller. 

Syntax:  
`SET_PID_TUNE,{P-coeff},{I-coeff},{D-coeff}`

Example:  
`SET_PID_TUNE,1.0,0.25,0.0`

Returns:  
Nothing

### SET_PID_SETP

`SET_PID_SETP` sets the value in mV that the controller should try to achieve at the input ADC 0.

Syntax:  
`SET_PID_SETP,{Setpoint in mV}`

Example:  
`SET_PID_SETP,500.0 `

Returns:  
Nothing

### SET_PID_LIMS

`SET_PID_LIMS` sets the minimum and maximum values in mV that the DAC 0 output should not exceed.

Syntax:  
`SET_PID_LIMS,{Minimum output in mV},{Maximum output in mV}`

Example:  
`SET_PID_LIMS,-500,250`

Returns:  
Nothing

### SET_PID_DIR

`SET_PID_DIR` defines the polarity for the output of the PID controller.

Syntax:  
`SET_PID_DIR,{0 for Reverse, 1 for Forward}`

Example:  
`SET_PID_DIR,1`

Returns:  
Nothing

### SET_PID_SLEW

`SET_PID_SLEW` configures the slew rate limit for the output that will be enforced regardless of the calculated output from the controller.

Syntax:  
`SET_PID_SLEW,{Maximum slew rate in mV per second}`

Example:  
`SET_PID_SLEW,50.0`

Returns:  
Nothing


