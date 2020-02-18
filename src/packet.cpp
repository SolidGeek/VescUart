/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include <Arduino.h>
#include <stdint.h>
#include "packet.h"
#include "crc.h"

void Packet::init( Stream * port ){

	this->serialPort = port;

}

void Packet::reset( void ) {
	this->rx_read_ptr = 0;
	this->rx_write_ptr = 0;
	this->bytes_left = 0;
}

void Packet::send( uint8_t *data, uint16_t len ){

	if (len == 0 || len > PACKET_MAX_LEN) {
		return;
	}

	int16_t index = 0;

	if (len <= 255) {
		this->tx_buffer[index++] = 2;
		this->tx_buffer[index++] = len;
	} else if (len <= 65535) {
		this->tx_buffer[index++] = 3;
		this->tx_buffer[index++] = len >> 8;  	// Bitshift 8 bit to the left (first 8 bit)
		this->tx_buffer[index++] = len & 0xFF; 	// Bitwise AND with 0xFF (last 8 bit)
	} 


	memcpy(this->tx_buffer + index, data, len);
	index += len;

	uint16_t crc = (uint16_t)crc16(data, len);
	this->tx_buffer[index++] = (uint8_t)(crc >> 8);
	this->tx_buffer[index++] = (uint8_t)(crc & 0xFF);
	this->tx_buffer[index++] = 3;


	if (this->serialPort) {
		this->serialPort->write(this->tx_buffer, index);
	}
}

void Packet::process_byte( uint8_t rx_data ) {

	uint16_t data_len = this->rx_write_ptr - this->rx_read_ptr;

	// Out of space (should not happen)
	if (data_len >= BUFFER_LEN) {
		this->rx_write_ptr = 0;
		this->rx_read_ptr = 0;
		this->bytes_left = 0;
		this->rx_buffer[this->rx_write_ptr++] = rx_data;
		return;
	}

	// Everything has to be aligned, so shift buffer if we are out of space (as opposed to using a circular buffer)
	if (this->rx_write_ptr >= BUFFER_LEN) {
		memmove(this->rx_buffer, this->rx_buffer + this->rx_read_ptr, data_len);

		this->rx_read_ptr = 0;
		this->rx_write_ptr = data_len;
	}

	this->rx_buffer[this->rx_write_ptr++] = rx_data;
	data_len++;

	if (this->bytes_left > 1) {
		this->bytes_left--;
		return;
	}

	// Try decoding the packet at various offsets until it succeeds, or until we run out of data.
	while (true) {
		int16_t res = this->decode_packet(this->rx_buffer + this->rx_read_ptr, data_len, &this->bytes_left);

		// More data is needed
		if (res == -2) {
			break;
		}

		if (res > 0) {
			data_len -= res;
			this->rx_read_ptr += res;
		} else if (res == -1) {
			// Something went wrong. Move pointer forward and try again.
			this->rx_read_ptr++;
			data_len--;
		}
	}

	// Nothing left, move pointers to avoid memmove
	if (data_len == 0) {
		this->rx_read_ptr = 0;
		this->rx_write_ptr = 0;
	}
}

/**
 * Try if it is possible to decode a packet from a buffer.
 *
 * @param buffer
 * The buffer to try from
 *
 * @param in_len
 * The length of the buffer
 *
 * @param bytes_left
 * This many additional bytes are required to tell more about the packet.
 *
 * @return
 * >0: Success, number of bytes decoded from buffer (not payload length)
 * -1: Invalid structure
 * -2: OK so far, but not enough data
 */

int8_t Packet::decode_packet( uint8_t *buffer, uint16_t in_len, uint16_t *bytes_left ){
	
	bool is_len_8b = buffer[0] == 2;
	bool is_len_16b = buffer[0] == 3;

	uint16_t data_start = buffer[0];

	*bytes_left = 0;

	if (in_len == 0) {
		*bytes_left = 1;
		return -2;
	}

	// No valid start byte
	if (!is_len_8b && !is_len_16b) {
		return -1;
	}

	// Not enough data to determine length
	if (in_len < data_start) {
		*bytes_left = data_start - in_len;
		return -2;
	}

	// Get length
	uint16_t len = 0;

	if (is_len_8b) {
		len = (uint16_t)buffer[1];

		// No support for zero length packets
		if (len < 1) 
			return -1;
	} else if (is_len_16b) {
		len = (uint16_t)buffer[1] << 8 | (uint16_t)buffer[2];

		// A shorter packet should use less length bytes
		if (len < 255)
			return -1;
	}

	// Too long packet
	if (len > PACKET_MAX_LEN) {
		return -1;
	}

	// Need more data to determine rest of packet
	if (in_len < (len + data_start + 3)) {
		*bytes_left = (len + data_start + 3) - in_len;
		return -2;
	}

	// Invalid stop byte
	if (buffer[data_start + len + 2] != 3) {
		return -1;
	}

	uint16_t crc_calc = (uint16_t)crc16(buffer + data_start, len);
	uint16_t crc_rx = (uint16_t)buffer[data_start + len] << 8 | (uint16_t)buffer[data_start + len + 1];

	if (crc_calc == crc_rx) {
		/* if (process_func) {
			process_func(buffer + data_start, len);
		}*/

		// Return buffered data in some way

		// Return the length 
		return len + data_start + 3;
	} else {
		return -1;
	}
}
