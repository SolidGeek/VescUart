# VescUart

Arduino library for interfacing with a VESC over UART. This library is based upon the library written by RollingGecko (https://github.com/RollingGecko/VescUartControl). The library is updated for the newest VESC firmware (FW3.40) and cleaned up a bit. The library is not backwards compatible, so you have to upload the newest firmware to your VESC.

**Important:** This is not a dropin replacement for RollingGeckos library. You will have to make some changes to your software, as all functions and values is now within a class, see below.

## Implementation

To use the library you will have initiate the VescUart class and set the Serial port for UART communcation.

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

Getting VESC telemetry is easy:

```cpp
if ( UART.getVescValues() ) {
  Serial.println(UART.data.rpm);
  Serial.println(UART.data.inpVoltage);
  Serial.println(UART.data.ampHours);
  Serial.println(UART.data.tachometerAbs);
}
```
