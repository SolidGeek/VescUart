# VescUart

Arduino library for interfacing with a VESC over UART. This library is based upon the library written by RollingGecko (https://github.com/RollingGecko/VescUartControl). The library is updated for the newest VESC firmware (FW4+) and cleaned up a bit. The library is not backwards compatible, so you have to upload the newest firmware to your VESC.

**Important:** This is not a dropin replacement for RollingGeckos library. You will have to make some changes to your software, as all functions and values is now within a class, see below.

## SwitchSystems/VescUart fork details

Added CAN BUS support to all methods, and additional features. Maintaining backwards compatibility where possible so it should be a drop-in replacement for SolidGeek/VescUart@develop which this was forked from.

Note: You can't use a CAN bus ID of 0 for this library (we use this to refer to the local device), so start numbering at 1.

## Usage
  
Initialize VescUart class and select Serial port for UART communication.  
  
```cpp
#include <VescUart.h>

VescUart UART;

void setup() {
  Serial.begin(115200);

  while (!Serial) {;}

  UART.setSerialPort(&Serial);
}
```

You can now safely use the functions and change the values of the class. 
  
Getting VESC telemetry:
  
```cpp
if ( UART.getVescValues() ) {
  Serial.println(UART.data.rpm);
  Serial.println(UART.data.inpVoltage);
  Serial.println(UART.data.ampHours);
  Serial.println(UART.data.tachometerAbs);
}
```
  
You can find example usage and more information in the examples directory.  
  
