//Compatible with VESC FW3.49

#include "VescUart.h"
#include <HardwareSerial.h>

VescUart::VescUart(void){
	nunchuck.valueX         = 127;
	nunchuck.valueY         = 127;
	nunchuck.lowerButton  	= false;
	nunchuck.upperButton  	= false;
}

void VescUart::setSerialPort(HardwareSerial* port)
{
	serialPort = port;
}

void VescUart::setDebugPort(Stream* port)
{
	debugPort = port;
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;

	uint32_t timeout = millis() + 100; // Defining the timestamp for timeout (100ms before timeout)

	while ( millis() < timeout && messageRead == false) {

		while (serialPort->available()) {

			messageReceived[counter++] = serialPort->read();

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3)
						if( debugPort != NULL ){
							debugPort->println("Message is larger than 256 bytes - not supported");
						}
					break;

					default:
						if( debugPort != NULL ){
							debugPort->println("Unvalid start bit");
						}
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				if (debugPort != NULL) {
					debugPort->println("End of message reached!");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false && debugPort != NULL ) {
		debugPort->println("Timeout");
	}

	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked) {
		// Message was read
		return lenPayload;
	}
	else {
		// No Message Read
		return 0;
	}
}


bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if(debugPort!=NULL){
		debugPort->print("SRC received: "); debugPort->println(crcMessage);
	}

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if( debugPort != NULL ){
		debugPort->print("SRC calc: "); debugPort->println(crcPayload);
	}

	if (crcPayload == crcMessage) {
		if( debugPort != NULL ) {
			debugPort->print("Received: ");
			serialPrint(message, lenMes); debugPort->println();

			debugPort->print("Payload :      ");
			serialPrint(payload, message[1] - 1); debugPort->println();
		}

		return true;
	}
	else {
		return false;
	}
}

int VescUart::packSendPayload(uint8_t * payload, int lenPay) {

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

	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = '\0';

	if(debugPort!=NULL){
		debugPort->print("UART package send: "); serialPrint(messageSend, count);
	}

	// Sending package
	serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}

bool VescUart::processReadPacket(bool deviceType, uint8_t * message) {

	COMM_PACKET_ID packetId;
	COMM_PACKET_ID_DIEBIEMS packetIdDieBieMS;

	int32_t ind = 0;

	if (!deviceType) { //device if VESC type
		packetId = (COMM_PACKET_ID)message[0];
		message++; // Removes the packetId from the actual message (payload)

		switch (packetId){
			case COMM_FW_VERSION: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

				fw_version.major = message[ind++];
				fw_version.minor = message[ind++];
				return true;

			case COMM_GET_VALUES:
			case COMM_GET_VALUES_SELECTIVE: { // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164
				uint32_t mask = 0xFFFFFFFF;

				if (packetId == COMM_GET_VALUES_SELECTIVE){
					mask = buffer_get_uint32(message, &ind);
				}

				if (mask & ((uint32_t)1 << 0)) {data.tempFET 		= buffer_get_float16(message, 10.0, &ind);}
				if (mask & ((uint32_t)1 << 1)) {data.tempMotor 		= buffer_get_float16(message, 10.0, &ind);}
				if (mask & ((uint32_t)1 << 2)) {data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);}
				if (mask & ((uint32_t)1 << 3)) {data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);}
				if (mask & ((uint32_t)1 << 4)) {data.avgIdCurent 	= buffer_get_float32(message, 100.0, &ind);}
				if (mask & ((uint32_t)1 << 5)) {data.avgIqCurent 	= buffer_get_float32(message, 100.0, &ind);}
				if (mask & ((uint32_t)1 << 6)) {data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);}
				if (mask & ((uint32_t)1 << 7)) {data.rpm 				= buffer_get_int32(message, &ind);}
				if (mask & ((uint32_t)1 << 8)) {data.inpVoltage = buffer_get_float16(message, 10.0, &ind);}
				if (mask & ((uint32_t)1 << 9)) {data.ampHours = buffer_get_float32(message, 10000.0, &ind);}
				if (mask & ((uint32_t)1 << 10)) {data.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);}
				if (mask & ((uint32_t)1 << 11)) {data.watt_hours = buffer_get_float32(message, 10000.0, &ind);}
				if (mask & ((uint32_t)1 << 12)) {data.watt_hours_charged = buffer_get_float32(message, 10000.0, &ind);}
				if (mask & ((uint32_t)1 << 13)) {data.tachometer 		= buffer_get_int32(message, &ind);}
				if (mask & ((uint32_t)1 << 14)) {data.tachometerAbs 		= buffer_get_int32(message, &ind);}
				if (mask & ((uint32_t)1 << 15)) {data.fault = message[ind]; }
				//Others values are ignored. You can add them here accordingly to commands.c in VESC Firmware. Please add those variables in "struct dataPackage" in VescUart.h file.

				return true;
			}

			case COMM_GET_VALUES_SETUP_SELECTIVE: { // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164
				uint32_t mask = 0;
				mask += ind++ << 24;
				mask += ind++ << 16;
				mask += ind++ << 8;
				mask += ind++;

				if (mask & ((uint32_t)1 << 0)) {data.tempFET 		= buffer_get_float16(message, 10.0, &ind);}
				if (mask & ((uint32_t)1 << 1)) {data.tempMotor 		= buffer_get_float16(message, 10.0, &ind);}
				if (mask & ((uint32_t)1 << 2)) {data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);}
				if (mask & ((uint32_t)1 << 3)) {data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);}
				if (mask & ((uint32_t)1 << 4)) {data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);}
				if (mask & ((uint32_t)1 << 5)) {data.rpm 				= buffer_get_int32(message, &ind);}
				if (mask & ((uint32_t)1 << 6)) { /* speed */};
				if (mask & ((uint32_t)1 << 7)) {data.inpVoltage = buffer_get_float16(message, 10.0, &ind);}
				if (mask & ((uint32_t)1 << 8)) { /* batt level */}
				if (mask & ((uint32_t)1 << 9)) {data.ampHours = buffer_get_float32(message, 10000.0, &ind);}
				if (mask & ((uint32_t)1 << 10)) {data.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);}
				if (mask & ((uint32_t)1 << 11)) {data.watt_hours = buffer_get_float32(message, 10000.0, &ind);}
				if (mask & ((uint32_t)1 << 12)) {data.watt_hours_charged = buffer_get_float32(message, 10000.0, &ind);}
				if (mask & ((uint32_t)1 << 13)) {/* distance */}
				if (mask & ((uint32_t)1 << 14)) {/* distance absolute */}
				if (mask & ((uint32_t)1 << 15)) {/* PID pos */}
				if (mask & ((uint32_t)1 << 16)) {data.fault = message[ind]; }
				//Others values are ignored. You can add them here accordingly to commands.c in VESC Firmware. Please add those variables in "struct dataPackage" in VescUart.h file.

				return true;
			}

			case COMM_GET_DECODED_PPM:

				data.throttle 	= (float)(buffer_get_int32(message, &ind) / 10000.0);
				//data.rawValuePPM 	= buffer_get_float32(message, 100.0, &ind);
				return true;
			break;

			case COMM_GET_DECODED_CHUK:

				data.throttle 	= (float)(buffer_get_int32(message, &ind) / 10000.0);

				return true;
			break;

			default:
				return false;
			break;
		}
	}
	else { //device is DieBieMS
		packetIdDieBieMS = (COMM_PACKET_ID_DIEBIEMS)message[0];
		message++; // Removes the packetId from the actual message (payload)

		switch (packetIdDieBieMS){

			case DBMS_COMM_GET_VALUES: // Structure defined here: https://github.com/DieBieEngineering/DieBieMS-Firmware/blob/master/Modules/Src/modCommands.c

				ind = 45;
				// DieBieMSdata.packVoltage = buffer_get_float32(message, 1000.0, &ind);
				// DieBieMSdata.packCurrent = buffer_get_float32(message, 1000.0, &ind);
				// DieBieMSdata.cellVoltageHigh = buffer_get_float32(message, 1000.0, &ind);
				// DieBieMSdata.cellVoltageAverage = buffer_get_float32(message, 1000.0, &ind);
				// DieBieMSdata.cellVoltageLow = buffer_get_float32(message, 1000.0, &ind);
				// DieBieMSdata.cellVoltageMisMatch = buffer_get_float32(message, 1000.0, &ind);
				// DieBieMSdata.loCurrentLoadVoltage = buffer_get_float16(message, 100.0, &ind);
				// DieBieMSdata.loCurrentLoadCurrent = buffer_get_float16(message, 100.0, &ind);
				// DieBieMSdata.hiCurrentLoadVoltage = buffer_get_float16(message, 100.0, &ind);
				// DieBieMSdata.hiCurrentLoadCurrent = buffer_get_float16(message, 100.0, &ind);
				// DieBieMSdata.auxVoltage = buffer_get_float16(message, 100.0, &ind);
				// DieBieMSdata.auxCurrent = buffer_get_float16(message, 100.0, &ind);
				// DieBieMSdata.tempBatteryHigh = buffer_get_float16(message, 10.0, &ind);
				// DieBieMSdata.tempBatteryAverage = buffer_get_float16(message, 10.0, &ind);
				// DieBieMSdata.tempBMSHigh = buffer_get_float16(message, 10.0, &ind);
				// DieBieMSdata.tempBMSAverage = buffer_get_float16(message, 10.0, &ind);
				DieBieMSdata.operationalState = message[ind++];
				// DieBieMSdata.chargeBalanceActive = message[ind++];
				// DieBieMSdata.faultState = message[ind++];

				return true;
			break;

			case DBMS_COMM_GET_BMS_CELLS: // Structure defined here: https://github.com/DieBieEngineering/DieBieMS-Firmware/blob/master/Modules/Src/modCommands.c

				DieBieMScells.noOfCells = message[ind++];

				for (uint8_t i=0; i<12;i++){
					DieBieMScells.cellsVoltage[i] = buffer_get_float16(message, 1000.0, &ind);
				}

				return true;
			break;

			default:
				return false;
			break;
		}
	}
}

bool VescUart::getVescValues(void) {
	uint8_t command[1];
	command[0] = { COMM_GET_VALUES };
	uint8_t payload[256];

	packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0 && lenPayload < 55) {
		bool read = processReadPacket(false, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getVescValuesSelective(uint32_t mask) {
	uint8_t command[5];
	command[0] = { COMM_GET_VALUES_SELECTIVE };
	command[1] = { mask >> 24 }; //mask MSB
	command[2] = { mask >> 16 & 0xFF }; //mask
	command[3] = { mask >> 8 & 0xFF }; //mask
	command[4] = { mask & 0xFF }; //mask LSB
	uint8_t payload[256];

	packSendPayload(command, 5);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0 && lenPayload < 55) {
		bool read = processReadPacket(false, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getVescValuesSetupSelective(uint32_t mask) {
	uint8_t command[5];
	command[0] = { COMM_GET_VALUES_SETUP_SELECTIVE };
	command[1] = { mask >> 24 }; //mask MSB
	command[2] = { mask >> 16 & 0xFF }; //mask
	command[3] = { mask >> 8 & 0xFF }; //mask
	command[4] = { mask & 0xFF }; //mask LSB
	uint8_t payload[256];

	packSendPayload(command, 5);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0 && lenPayload < 55) {
		bool read = processReadPacket(false, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getLocalVescPPM(void) {

	uint8_t command[1] = { COMM_GET_DECODED_PPM };
	uint8_t payload[256];

	packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0) { //&& lenPayload < 55
		bool read = processReadPacket(false, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getMasterVescPPM(uint8_t id) {

	uint8_t command[3];
	command[0] = { COMM_FORWARD_CAN };
	command[1] = id;
	command[2] = { COMM_GET_DECODED_PPM };

	uint8_t payload[256];

	packSendPayload(command, 3);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0) { //&& lenPayload < 55
		bool read = processReadPacket(false, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getLocalVescNun(void){
	uint8_t command[1] = { COMM_GET_DECODED_CHUK };
	uint8_t payload[256];

	packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0) { //&& lenPayload < 55
		bool read = processReadPacket(false, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}


bool VescUart::getMasterVescNun(uint8_t id){
	uint8_t command[3];
	command[0] = { COMM_FORWARD_CAN };
	command[1] = id;
	command[2] = { COMM_GET_DECODED_CHUK };

	uint8_t payload[256];

	packSendPayload(command, 3);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0) { //&& lenPayload < 55
		bool read = processReadPacket(false, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getFWversion(void){

	uint8_t command[1] = { COMM_FW_VERSION };
	uint8_t payload[256];

	packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0) { //&& lenPayload < 55
		bool read = processReadPacket(false, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getDieBieMSValues(uint8_t id) {
	uint8_t command[3];
	command[0] = { COMM_FORWARD_CAN }; //VESC command
	command[1] = id;
	command[2] = { DBMS_COMM_GET_VALUES }; //DieBieMS command

	uint8_t payload[256];

	packSendPayload(command, 3);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0) { //&& lenPayload < 55
		bool read = processReadPacket(true, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getDieBieMSCellsVoltage(uint8_t id) {
	uint8_t command[3];
	command[0] = { COMM_FORWARD_CAN }; //VESC command
	command[1] = id;
	command[2] = { DBMS_COMM_GET_BMS_CELLS }; //DieBieMS command

	uint8_t payload[256];

	packSendPayload(command, 3);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 0) { //&& lenPayload < 55
		bool read = processReadPacket(true, payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}


void VescUart::setNunchuckValues() {
	int32_t ind = 0;
	uint8_t payload[11];

	payload[ind++] = COMM_SET_CHUCK_DATA;
	payload[ind++] = nunchuck.valueX;
	payload[ind++] = nunchuck.valueY;
	buffer_append_bool(payload, nunchuck.lowerButton, &ind);
	buffer_append_bool(payload, nunchuck.upperButton, &ind);

	// Acceleration Data. Not used, Int16 (2 byte)
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;

	if(debugPort != NULL){
		debugPort->println("Data reached at setNunchuckValues:");
		debugPort->print("valueX = "); debugPort->print(nunchuck.valueX); debugPort->print(" valueY = "); debugPort->println(nunchuck.valueY);
		debugPort->print("LowerButton = "); debugPort->print(nunchuck.lowerButton); debugPort->print(" UpperButton = "); debugPort->println(nunchuck.upperButton);
	}

	packSendPayload(payload, 11);
}

void VescUart::setCurrent(float current) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setRPM(float rpm) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM ;
	buffer_append_int32(payload, (int32_t)(rpm), &index);

	packSendPayload(payload, 5);
}

void VescUart::setDuty(float duty) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setLocalProfile(bool store, bool forward_can, bool ack, bool divide_by_controllers, float current_min_rel, float current_max_rel, float speed_max_reverse, float speed_max, float duty_min, float duty_max, float watt_min, float watt_max) {
	int32_t index = 0;
	uint8_t payload[38];

	payload[index++] = COMM_SET_MCCONF_TEMP_SETUP;
	payload[index++] ? store : 1, 0;
	payload[index++] ? forward_can : 1, 0;
	payload[index++] ? ack : 1, 0;
	payload[index++] ? divide_by_controllers : 1, 0;
	buffer_append_float32(payload, current_min_rel, 1.0, &index);
	buffer_append_float32(payload, current_max_rel, 1.0, &index);
	buffer_append_float32(payload, speed_max_reverse, 1.0, &index);
	buffer_append_float32(payload, speed_max, 1.0, &index);
	buffer_append_float32(payload, duty_min, 1.0, &index);
	buffer_append_float32(payload, duty_max, 1.0, &index);
	buffer_append_float32(payload, watt_min, 1.0, &index);
	buffer_append_float32(payload, watt_max, 1.0, &index);

	packSendPayload(payload, 38);
}

void VescUart::serialPrint(uint8_t * data, int len) {
	if(debugPort != NULL){
		for (int i = 0; i <= len; i++)
		{
			debugPort->print(data[i]);
			debugPort->print(" ");
		}

		debugPort->println("");
	}
}

void VescUart::printVescValues() {
	if(debugPort != NULL){
		debugPort->print("avgMotorCurrent: "); 	debugPort->println(data.avgMotorCurrent);
		debugPort->print("avgInputCurrent: "); 	debugPort->println(data.avgInputCurrent);
		debugPort->print("dutyCycleNow: "); 	debugPort->println(data.dutyCycleNow);
		debugPort->print("rpm: "); 				debugPort->println(data.rpm);
		debugPort->print("inputVoltage: "); 	debugPort->println(data.inpVoltage);
		debugPort->print("ampHours: "); 		debugPort->println(data.ampHours);
		debugPort->print("ampHoursCharges: "); 	debugPort->println(data.ampHoursCharged);
		debugPort->print("tachometer: "); 		debugPort->println(data.tachometer);
		debugPort->print("tachometerAbs: "); 	debugPort->println(data.tachometerAbs);
	}
}
