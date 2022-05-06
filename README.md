# VescUart

Arduino library for interfacing with a VESC over UART. This library is based upon the works of many. The library is tested on a Teensy 4, and is updated for VESC firmware FW5+. The library is not nessecary backwards compatible with older versions of VESC firmware, so please update to the newest firmware available to your VESC.

The library supports only a small amount of features available to the VESC. You are welcome to make a pull request if you integrate new functionality and I will do my best to merge. 

## CAN BUS support

CAN BUS is now supported, such that you can communcate with multiple VESCs over a single UART port. All methods can be called with a CAN ID, and the main VESC will forward the command to the desired CAN Id. 

You can't use a CAN bus ID of 0 for this library, as this is used to refer to the local device; start numbering at 1.

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
  
