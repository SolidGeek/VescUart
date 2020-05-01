/*
  Name:    getDieBieMSValues.ino
  Created: 04-08-2019
  Author:  Peemouse
  Description:  This example is made using a Arduino Micro (Atmega32u4) that has a HardwareSerial port (Serial1) seperated from the Serial port.
                A Arduino Nano or Uno that only has one Serial port will not be able to display the data returned.
*/

#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;

char DieBieMS_CANID = 10; //CAN ID of DieBieMS. Default is 10.

void setup() {

  /** Setup Serial port to display data */
  Serial.begin(115200);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);

  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);
}

void loop() {

  /** Call the function getDieBieMSValues() to acquire data from DieBieMS */
  if ( UART.getDieBieMSValues(DieBieMS_CANID) ) {

    Serial.println(UART.DieBieMSdata.soc);

  }
  else
  {
    Serial.println("Failed to get DieBieMS data!");
  }

  delay(50);
}
