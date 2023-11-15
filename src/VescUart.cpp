#include <stdint.h>
#include "VescUart.h"

VescUart::VescUart(uint32_t timeout_ms) : _TIMEOUT(timeout_ms)
{
	nunchuck.valueX = 127;
	nunchuck.valueY = 127;
	nunchuck.lowerButton = false;
	nunchuck.upperButton = false;

	appData.dutyCycle=0;
	appData.erpm=0;
	appData.inputVoltage=0;
	appData.loopTime=0;
	appData.motorCurrent=0;
	appData.pidOutput=0;
	appData.pitch=0;
	appData.roll=0;
	appData.state=0;
	appData.switchState=0;
	appData.vescId=0;
}

void VescUart::setSerialPort(Stream *port)
{
	serialPort = port;
}

void VescUart::setDebugPort(Stream *port)
{
	debugPort = port;
}

int VescUart::receiveUartMessage(uint8_t *payloadReceived)
{

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	// Makes no sense to run this function if no serialPort is defined.
	if (serialPort == NULL)
		return -1;

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;

	uint32_t timeout = millis() + _TIMEOUT; // Defining the timestamp for timeout (100ms before timeout)

	while (millis() < timeout && messageRead == false)
	{

		while (serialPort->available())
		{

			messageReceived[counter++] = serialPort->read();

			if (counter == 2)
			{

				switch (messageReceived[0])
				{
				case 2:
					endMessage = messageReceived[1] + 5; // Payload size + 2 for sice + 3 for SRC and End.
					lenPayload = messageReceived[1];
					break;

				case 3:
					// ToDo: Add Message Handling > 255 (starting with 3)
					if (debugPort != NULL)
					{
						debugPort->println("Message is larger than 256 bytes - not supported");
					}
					break;

				default:
					if (debugPort != NULL)
					{
						debugPort->println("Unvalid start bit");
					}
					break;
				}
			}

			if (counter >= sizeof(messageReceived))
			{
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3)
			{
				messageReceived[endMessage] = 0;
				if (debugPort != NULL)
				{
					debugPort->println("End of message reached!");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if (messageRead == false && debugPort != NULL)
	{
		debugPort->println("Timeout");
	}

	bool unpacked = false;

	if (messageRead)
	{
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked)
	{
		// Message was read
		return lenPayload;
	}
	else
	{
		// No Message Read
		return 0;
	}
}

bool VescUart::unpackPayload(uint8_t *message, int lenMes, uint8_t *payload)
{

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if (debugPort != NULL)
	{
		debugPort->print("SRC received: ");
		debugPort->println(crcMessage);
	}

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if (debugPort != NULL)
	{
		debugPort->print("SRC calc: ");
		debugPort->println(crcPayload);
	}

	if (crcPayload == crcMessage)
	{
		if (debugPort != NULL)
		{
			debugPort->print("Received: ");
			serialPrint(message, lenMes);
			debugPort->println();

			debugPort->print("Payload :      ");
			serialPrint(payload, message[1] - 1);
			debugPort->println();
		}

		return true;
	}
	else
	{
		return false;
	}
}

int VescUart::packSendPayload(uint8_t *payload, int lenPay)
{

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(messageSend + count, payload, lenPay);
	count += lenPay;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	// messageSend[count] = NULL;

	if (debugPort != NULL)
	{
		debugPort->print("Package to send: ");
		serialPrint(messageSend, count);
	}

	// Sending package
	if (serialPort != NULL)
		serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}


bool VescUart::processReadPacket(uint8_t *message)
{

	COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId)
	{
	case COMM_FW_VERSION: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

		fw_version.major = message[index++];
		fw_version.minor = message[index++];
		return true;
	case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

		data.tempMosfet = buffer_get_float16(message, 10.0, &index);		  // 2 bytes - mc_interface_temp_fet_filtered()
		data.tempMotor = buffer_get_float16(message, 10.0, &index);			  // 2 bytes - mc_interface_temp_motor_filtered()
		data.avgMotorCurrent = buffer_get_float32(message, 100.0, &index);	  // 4 bytes - mc_interface_read_reset_avg_motor_current()
		data.avgInputCurrent = buffer_get_float32(message, 100.0, &index);	  // 4 bytes - mc_interface_read_reset_avg_input_current()
		index += 4;															  // Skip 4 bytes - mc_interface_read_reset_avg_id()
		index += 4;															  // Skip 4 bytes - mc_interface_read_reset_avg_iq()
		data.dutyCycleNow = buffer_get_float16(message, 1000.0, &index);	  // 2 bytes - mc_interface_get_duty_cycle_now()
		data.rpm = buffer_get_float32(message, 1.0, &index);				  // 4 bytes - mc_interface_get_rpm()
		data.inpVoltage = buffer_get_float16(message, 10.0, &index);		  // 2 bytes - GET_INPUT_VOLTAGE()
		data.ampHours = buffer_get_float32(message, 10000.0, &index);		  // 4 bytes - mc_interface_get_amp_hours(false)
		data.ampHoursCharged = buffer_get_float32(message, 10000.0, &index);  // 4 bytes - mc_interface_get_amp_hours_charged(false)
		data.wattHours = buffer_get_float32(message, 10000.0, &index);		  // 4 bytes - mc_interface_get_watt_hours(false)
		data.wattHoursCharged = buffer_get_float32(message, 10000.0, &index); // 4 bytes - mc_interface_get_watt_hours_charged(false)
		data.tachometer = buffer_get_int32(message, &index);				  // 4 bytes - mc_interface_get_tachometer_value(false)
		data.tachometerAbs = buffer_get_int32(message, &index);				  // 4 bytes - mc_interface_get_tachometer_abs_value(false)
		data.error = (mc_fault_code)message[index++];						  // 1 byte  - mc_interface_get_fault()
		data.pidPos = buffer_get_float32(message, 1000000.0, &index);		  // 4 bytes - mc_interface_get_pid_pos_now()
		data.id = message[index++];											  // 1 byte  - app_get_configuration()->controller_id

		return true;

		break;

	case COMM_GET_CUSTOM_CONFIG:
		// 34 byte received 
		//balance data 
		appData.pidOutput = buffer_get_float32(message, 1e6, &index); // 4 byte  pid output
		appData.pitch = buffer_get_float32(message, 1e6, &index);	   // 4 byte 
		appData.roll = buffer_get_float32(message, 1e6, &index);	//4 byte 
		appData.loopTime = buffer_get_uint32(message, &index); //4 byte 
		appData.motorCurrent = buffer_get_float32(message, 1e6, &index); //4byte 
		appData.state = buffer_get_uint16(message, &index); //2 byte 
		appData.switchState = buffer_get_uint16(message, &index); //2byte 
		//other data 
		appData.vescId = buffer_get_uint16(message, &index); //2byte 
		appData.dutyCycle = buffer_get_float16(message, 1e3, &index);//2byte 
		appData.erpm=buffer_get_float32(message, 1e0, &index); //4byte ;
		appData.inputVoltage=buffer_get_float16(message, 1e1, &index); //2byte ;
		
		return true;
		break;
	default:
		return false;
		break;
	}
}


bool VescUart::getFWversion(void)
{
	return getFWversion(0);
}

bool VescUart::getFWversion(uint8_t canId)
{

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];

	if (canId != 0)
	{
		payload[index++] = {COMM_FORWARD_CAN};
		payload[index++] = canId;
	}
	payload[index++] = {COMM_FW_VERSION};

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);
	if (messageLength > 0)
	{
		return processReadPacket(message);
	}
	return false;
}

bool VescUart::getVescValues(void)
{
	return getVescValues(0);
}

bool VescUart::getVescValues(uint8_t canId)
{

	if (debugPort != NULL)
	{
		debugPort->println("Command: COMM_GET_VALUES " + String(canId));
	}

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0)
	{
		payload[index++] = {COMM_FORWARD_CAN};
		payload[index++] = canId;
	}
	payload[index++] = {COMM_GET_VALUES};

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

	if (messageLength > 55)
	{
		return processReadPacket(message);
	}
	return false;
}
void VescUart::setNunchuckValues()
{
	return setNunchuckValues(0);
}

void VescUart::setNunchuckValues(uint8_t canId)
{

	if (debugPort != NULL)
	{
		debugPort->println("Command: COMM_SET_CHUCK_DATA " + String(canId));
	}
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 11 : 13);
	uint8_t payload[payloadSize];

	if (canId != 0)
	{
		payload[index++] = {COMM_FORWARD_CAN};
		payload[index++] = canId;
	}
	payload[index++] = {COMM_SET_CHUCK_DATA};
	payload[index++] = nunchuck.valueX;
	payload[index++] = nunchuck.valueY;
	buffer_append_bool(payload, nunchuck.lowerButton, &index);
	buffer_append_bool(payload, nunchuck.upperButton, &index);

	// Acceleration Data. Not used, Int16 (2 byte)
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;

	if (debugPort != NULL)
	{
		debugPort->println("Nunchuck Values:");
		debugPort->print("x=");
		debugPort->print(nunchuck.valueX);
		debugPort->print(" y=");
		debugPort->print(nunchuck.valueY);
		debugPort->print(" LBTN=");
		debugPort->print(nunchuck.lowerButton);
		debugPort->print(" UBTN=");
		debugPort->println(nunchuck.upperButton);
	}

	packSendPayload(payload, payloadSize);
}

void VescUart::setCurrent(float current)
{
	return setCurrent(current, 0);
}

void VescUart::setCurrent(float current, uint8_t canId)
{
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0)
	{
		payload[index++] = {COMM_FORWARD_CAN};
		payload[index++] = canId;
	}
	payload[index++] = {COMM_SET_CURRENT};
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setBrakeCurrent(float brakeCurrent)
{
	return setBrakeCurrent(brakeCurrent, 0);
}

void VescUart::setBrakeCurrent(float brakeCurrent, uint8_t canId)
{
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0)
	{
		payload[index++] = {COMM_FORWARD_CAN};
		payload[index++] = canId;
	}

	payload[index++] = {COMM_SET_CURRENT_BRAKE};
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::setRPM(float rpm)
{
	return setRPM(rpm, 0);
}

void VescUart::setRPM(float rpm, uint8_t canId)
{
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0)
	{
		payload[index++] = {COMM_FORWARD_CAN};
		payload[index++] = canId;
	}
	payload[index++] = {COMM_SET_RPM};
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setDuty(float duty)
{
	return setDuty(duty, 0);
}

void VescUart::setDuty(float duty, uint8_t canId)
{
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0)
	{
		payload[index++] = {COMM_FORWARD_CAN};
		payload[index++] = canId;
	}
	payload[index++] = {COMM_SET_DUTY};
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::sendKeepalive(void)
{
	return sendKeepalive(0);
}

void VescUart::sendKeepalive(uint8_t canId)
{
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0)
	{
		payload[index++] = {COMM_FORWARD_CAN};
		payload[index++] = canId;
	}
	payload[index++] = {COMM_ALIVE};
	packSendPayload(payload, payloadSize);
}

void VescUart::serialPrint(uint8_t *data, int len)
{
	if (debugPort != NULL)
	{
		for (int i = 0; i <= len; i++)
		{
			debugPort->print(data[i]);
			debugPort->print(" ");
		}
		debugPort->println("");
	}
}

void VescUart::printVescValues()
{
	if (debugPort != NULL)
	{
		debugPort->print("avgMotorCurrent: ");
		debugPort->println(data.avgMotorCurrent);
		debugPort->print("avgInputCurrent: ");
		debugPort->println(data.avgInputCurrent);
		debugPort->print("dutyCycleNow: ");
		debugPort->println(data.dutyCycleNow);
		debugPort->print("rpm: ");
		debugPort->println(data.rpm);
		debugPort->print("inputVoltage: ");
		debugPort->println(data.inpVoltage);
		debugPort->print("ampHours: ");
		debugPort->println(data.ampHours);
		debugPort->print("ampHoursCharged: ");
		debugPort->println(data.ampHoursCharged);
		debugPort->print("wattHours: ");
		debugPort->println(data.wattHours);
		debugPort->print("wattHoursCharged: ");
		debugPort->println(data.wattHoursCharged);
		debugPort->print("tachometer: ");
		debugPort->println(data.tachometer);
		debugPort->print("tachometerAbs: ");
		debugPort->println(data.tachometerAbs);
		debugPort->print("tempMosfet: ");
		debugPort->println(data.tempMosfet);
		debugPort->print("tempMotor: ");
		debugPort->println(data.tempMotor);
		debugPort->print("error: ");
		debugPort->println(data.error);
	}
}

void VescUart::printCustomValues()
{
	if (debugPort != NULL)
	{
		debugPort->print("pidOutput: ");
		debugPort->println(appData.pidOutput);
		debugPort->print("pitch angle: ");
		debugPort->println(appData.pitch);
		debugPort->print("roll angle: ");
		debugPort->println(appData.roll);
		debugPort->print("motor current: ");
		debugPort->println(appData.motorCurrent);
		debugPort->print("loop time: ");
		debugPort->println(appData.loopTime);
		debugPort->print("state: ");
		debugPort->println(appData.state);
		debugPort->print("switch state: ");
		debugPort->println(appData.switchState);
		debugPort->print("VESC vescId: ");
		debugPort->println(appData.vescId);
		debugPort->print("Input Voltage: ");
		debugPort->println(appData.inputVoltage);
		debugPort->print("ERPM: ");
		debugPort->println(appData.erpm);
		debugPort->print("Duty Cycle: ");
		debugPort->println(appData.dutyCycle);
	}
}
/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool VescUart::updateCustomValues()
{
	return updateCustomValues(0);

}

/**
 * @brief custom data get from vesc 
 * Balance App:	pid_output (float) 
 * 				pitch_angle(float)
 * 				roll_angle(float)
 * 				loop_time (uint32_t)
 * 				State(uint16_t)
 * 				Switch State(uint16_t)
 * Other 	  :	vesc id (uint16_t)
 * 				duty cycle (float)	
 * 				erpm (float)
 * 				input voltage(float)
 * 
 * @param canId 
 * @return true 
 * @return false 
 */
bool VescUart::updateCustomValues(uint8_t canId)
{
	if (debugPort != NULL)
	{
		debugPort->println("COMM_GET_CUSTOM_CONFIG " + String(canId));
	}

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0)
	{
		payload[index++] = {COMM_GET_CUSTOM_CONFIG};
		payload[index++] = canId;
	}
	payload[index++] = {COMM_GET_CUSTOM_CONFIG};
	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);
	
	if (messageLength >34)
	{
		return processReadPacket(message);
	}
	return false;
}


/**
 * @brief Return PID OUTPUT 
 * should call updateCustomValues() first.
 */
float VescUart::getPidOUtput()
{
 return appData.pidOutput;
}
/**
 * @brief return erpm 
 * should call updateCustomValues() first.
 */
float VescUart::getErpm()
{

	return appData.erpm;
}
/**
 * @brief return Switch state 
 * should call updateCustomValues() first.
 */
uint16_t VescUart::getSwitchState()
{
	return appData.switchState;
}
/**
 * @brief return VESC ID to control Audio source  
 * should call updateCustomValues() first.
 */
uint16_t VescUart::getVescId()
{
	return appData.vescId;
}


/**
 * @brief return VESC ID to control Audio source  
 * should call updateCustomValues() first.
 */
float VescUart::getMotorCurrent()
{
	return appData.motorCurrent;
}
