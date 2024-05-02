# FastDAC
Arduino Code for FastDAC

Developed for arduino SAM cores library 1.6.12 but must have UARTClass.cpp and RingBuffer.h in repo /corepatch folder copied over original core library, which addresses issue at: https://github.com/arduino/ArduinoCore-sam/issues/64, and increases the serial buffer size to 1024, for more efficient transfer of ramp data to the FastDAC

On my Windows install these files are located at:

C:\Users\\{UserName}\Documents\ArduinoData\packages\arduino\hardware\sam\1.6.12\cores\arduino

The other possible location depending on IDE version seems to be:

C:\Users\\{username}\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\cores\arduino

If you have both, best to overwrite them both.
