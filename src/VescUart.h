#ifndef _VESCUART_h
#define _VESCUART_h

#include <Arduino.h>
#include "datatypes.h"
#include "buffer.h"
#include "crc.h"

class VescUart
{
	/** Struct to store the telemetry data returned by the VESC */
	struct dataPackage {
		float avgMotorCurrent;
		float avgInputCurrent;
		float dutyCycleNow;
		long rpm;
		float inpVoltage;
		float ampHours;
		float ampHoursCharged;
		long tachometer;
		long tachometerAbs;
	};

	/** Struct to hold the nunchuck values to send over UART */
	struct nunchuckPackage {
		int	valXJoy;
		int	valYJoy;
		bool valUpperButton;
		bool valLowerButton;
	};


	public:
		/**
		 * @brief      Class constructor
		 */
		VescUart(void);

		/** Variabel to hold measurements returned from VESC */
		dataPackage data; 

		/** Variabel to hold nunchuck values */
		nunchuckPackage nunchuck; 

		/**
		 * @brief      Set the serial port for uart communication
		 */
		void setSerialPort(HardwareSerial* _serialPort);

		/**
		 * @brief      Set the serial port for debugging
		 * @param      Serial object 
		 */
		void setDebugPort(Stream* _debugSerialPort);

		/**
		 * @brief      Sends a command to VESC and stores the returned data
		 *
		 * @return     True if successfull otherwise false
		 */
		bool getVescValues(void);

		/**
		 * @brief      Sends values for joystick and buttons to the nunchuck app
		 */
		void setNunchuckValues(void);

		/**
		 * @brief      Help Function to print struct dataPackage over Serial for Debug
		 */
		void printVescValues(void);

	private: 

		/** Variabel to hold the reference to the Serial object to use for UART */
		HardwareSerial* serialPort = NULL;

		/** Variabel to hold the reference to the Serial object to use for debugging. 
		  * Uses the class Stream instead of HarwareSerial */
		Stream* debugPort = NULL;

		/**
		 * @brief      Packs the payload and sends it over Serial
		 *
		 * @param      payload  - The payload as a unit8_t Array with length of int lenPayload
		 * @param      lenPay   - Length of payload
		 * @return     The number of bytes send
		 */
		int packSendPayload(uint8_t * payload, int lenPay);

		/**
		 * @brief      Receives the message over Serial
		 *
		 * @param      payloadReceived  - The received payload as a unit8_t Array
		 * @return     The number of bytes receeived within the payload
		 */
		int receiveUartMessage(uint8_t * payloadReceived);

		/**
		 * @brief      Verifies the message (CRC-16) and extracts the payload
		 *
		 * @param      message  - The received UART message
		 * @param      lenMes   - The lenght of the message
		 * @param      payload  - The final payload ready to extract data from
		 * @return     True if the process was a success
		 */
		bool unpackPayload(uint8_t * message, int lenMes, uint8_t * payload);

		/**
		 * @brief      Extracts the data from the received payload
		 *
		 * @param      message  - The payload to extract data from
		 * @return     True if the process was a success
		 */
		bool processReadPacket(uint8_t * message);

		/**
		 * @brief      Help Function to print uint8_t array over Serial for Debug
		 *
		 * @param      data  - Data array to print
		 * @param      len   - Lenght of the array to print
		 */
		void serialPrint(uint8_t * data, int len);

};

#endif
