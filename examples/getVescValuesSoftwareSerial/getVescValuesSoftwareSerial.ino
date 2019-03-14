/*
  Name:    getVescValuesSoftwareSerial.ino
  Created: 06-03-2019
  Author:  SkewPL
  Description:  This example shows ability to use SoftwareSerial port to communicate with VESC. Confirmed to be working on ESP8266.
                Remember to change the baud rate to a correct value, you can find which baud rate does your VESC use in VESCTool -> UART section.
*/

#include <VescUart.h>
#include <SoftwareSerial.h>

/** Initiate VescUart class */
VescUart vesc;

/** Initiate SoftwareSerial class */
SoftwareSerial vescSerial(13, 15);

void setup() {

  /** Setup Serial port to display data */
  Serial.begin(9600);

  /** Setup SoftwareSerial port */
  vescSerial.begin(19200);

  /** Define which ports to use as UART */
  vesc.setSerialPort(&vescSerial);
}

void loop() {
  
  /** Call the function getVescValues() to acquire data from VESC */
  if ( vesc.getVescValues() ) {

    Serial.println(vesc.data.rpm);
    Serial.println(vesc.data.inpVoltage);
    Serial.println(vesc.data.ampHours);
    Serial.println(vesc.data.tachometerAbs);

  }
  else
  {
    Serial.println("Failed to get data!");
  }

  delay(50);
}