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
		float tempFET;
		float tempMotor;
		float avgMotorCurrent;
		float avgInputCurrent;
		float avgIqCurent;
		float avgIdCurent;
		float dutyCycleNow;
		long rpm;
		float inpVoltage;
		float ampHours;
		float ampHoursCharged;
		float watt_hours;
		float watt_hours_charged;
		long tachometer;
		long tachometerAbs;
		uint8_t fault;
		float throttle;
	};

	/** Struct to store the telemetry data returned by the DieBieMS */
	struct DieBieMSdataPackage {
		float packVoltage;
		float packCurrent;
		uint8_t soc;
		float cellVoltageHigh;
		float cellVoltageAverage;
		float cellVoltageLow;
		float cellVoltageMisMatch;
		float loCurrentLoadVoltage;
		float loCurrentLoadCurrent;
		float hiCurrentLoadVoltage;
		float hiCurrentLoadCurrent;
		float auxVoltage;
		float auxCurrent;
		float tempBatteryHigh;
		float tempBatteryAverage;
		float tempBMSHigh;
		float tempBMSAverage;
		uint8_t operationalState;
		uint8_t chargeBalanceActive;
		uint8_t faultState;
	};

	struct DieBieMScellsPackage {
		uint8_t noOfCells;
		float cellsVoltage[11];
	};

	struct FWversionPackage {
		uint8_t major;
	  uint8_t minor;
	};

	/** Struct to hold the nunchuck values to send over UART */
	struct nunchuckPackage {
		int	valueX;
		int	valueY;
		bool upperButton; // valUpperButton
		bool lowerButton; // valLowerButton
	};


	public:
		/**
		 * @brief      Class constructor
		 */
		VescUart(void);

		/** Variabel to hold measurements returned from VESC */
		dataPackage data;

		/** Variable to hold measurements returned from VESC */
		FWversionPackage fw_version;

		/** Variabel to hold nunchuck values */
		nunchuckPackage nunchuck;

		/** Variabel to hold measurements returned from DieBieMS */
		DieBieMSdataPackage DieBieMSdata;

		/** Variabel to hold cells voltages returned from DieBieMS */
		DieBieMScellsPackage DieBieMScells;


		/**
		 * @brief      Set the serial port for uart communication
		 * @param      port  - Reference to Serial port (pointer)
		 */
		void setSerialPort(HardwareSerial* port);

		/**
		 * @brief      Set the serial port for debugging
		 * @param      port  - Reference to Serial port (pointer)
		 */
		void setDebugPort(Stream* port);

		/**
		 * @brief      Sends a command to VESC and stores the returned data
		 * @return     True if successfull otherwise false
		 */
		bool getVescValues(void);

		/**
		 * @brief      Sends a command to VESC and stores the returned data
		 * @param			mask : select which values are sent back by the VESC
		 * @return     True if successfull otherwise false
		 */
		bool getVescValuesSelective(uint32_t mask);

		/**
		 * @brief      Sends a command to VESC and stores the returned data
		 * @param			mask : select which values are sent back by the VESC
		 * @return     True if successfull otherwise false
		 */
		bool getVescValuesSetupSelective(uint32_t mask);

		/**
		 * @brief      Sends values for joystick and buttons to the nunchuck app
		 */
		void setNunchuckValues(void);

		/**
		 * @brief      Set the current to drive the motor
		 * @param      current  - The current to apply
		 */
		void setCurrent(float current);

		/**
		 * @brief      Set the current to brake the motor
		 * @param      brakeCurrent  - The current to apply
		 */
		void setBrakeCurrent(float brakeCurrent);

		/**
		 * @brief      Set the rpm of the motor
		 * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
		 */
		void setRPM(float rpm);

		/**
		 * @brief      Set the duty of the motor
		 * @param      duty  - The desired duty (0.0-1.0)
		 */
		void setDuty(float duty);

		/**
		 * @brief      Help Function to print struct dataPackage over Serial for Debug
		 */
		void printVescValues(void);

		/**
		 * @brief      Request PPM values to local VESC
		 *
		 * @return		 True if successfull otherwise false
		 */
		bool getLocalVescPPM(void);

		/**
		 * @brief      Request PPM values to Master VESC over CANbus
		 * @param      id  - CAN ID of the master VESC
		 * @return		 True if successfull otherwise false
		 */
		bool getMasterVescPPM(uint8_t id);

		/**
		 * @brief      Request PPM values to local VESC
		 *
		 * @return		 True if successfull otherwise false
		 */
		bool getLocalVescNun(void);

		/**
		 * @brief      Request PPM values to Master VESC over CANbus
		 * @param      id  - CAN ID of the master VESC
		 * @return		 True if successfull otherwise false
		 */
		bool getMasterVescNun(uint8_t id);

		/**
		 * @brief      Request version of VESC Firmware
		 *
		 * @return		 True if successfull otherwise false
		 */
		bool getFWversion(void);

		/**
		 * @brief      Sends a command to DieBieMS over CAN and stores the returned data
		 * @param			id - CAN ID of DieBieMS (default is 10)
		 * @return     True if successfull otherwise false
		 */
		bool getDieBieMSValues(uint8_t id);

		/**
		 * @brief      Sends a command to DieBieMS over CAN and stores the returned cells voltage
		 * @param			id - CAN ID of DieBieMS (default is 10)
		 * @return     True if successfull otherwise false
		 */
		bool getDieBieMSCellsVoltage(uint8_t id);

		/**
		 * @brief      Set a profile
		 * @param

		 */
		void setLocalProfile(bool store, bool forward_can, bool ack, bool divide_by_controllers, float current_min_rel, float current_max_rel, float speed_max_reverse, float speed_max, float duty_min, float duty_max, float watt_min, float watt_max);

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
		 * @param			deviceType - 0 if VESC, 1 if DieBieMS
		 * @param      message  - The payload to extract data from
		 * @return     True if the process was a success
		 */
		bool processReadPacket(bool deviceType, uint8_t * message);

		/**
		 * @brief      Help Function to print uint8_t array over Serial for Debug
		 *
		 * @param      data  - Data array to print
		 * @param      len   - Lenght of the array to print
		 */
		void serialPrint(uint8_t * data, int len);



};

#endif
