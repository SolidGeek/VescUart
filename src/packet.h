/*
	Copyright 2020 Emil Jacobsen 	solidgeek.dk

	This file is inspired by the VESC software made by Benjamin Vedder (benjamin@vedder.se)

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

#ifndef VESC_PACKET_H_
#define VESC_PACKET_H_

#include <Arduino.h>
#include <stdint.h>

#define PACKET_MAX_LEN 512
#define BUFFER_LEN (PACKET_MAX_LEN + 8)

class Packet
{

public:

	/**
		* @brief      Init the packet with functions for sending and processing data
		* @param      port - Pointer to serial communication object 
	*/
	void init( Stream * port );

	/**
		* @brief      Reset the package object
	*/
	void reset( void );

	/**
		* @brief      Add byte to the rx_buffer and process the data
		* @param      rx_data - Byte to be added
	*/
	void process_byte( uint8_t rx_data );

	void timerfunc( void );

	void send( uint8_t *data, uint16_t len );

	int8_t decode_packet( uint8_t *buffer, uint16_t in_len, uint16_t *bytes_left );


private:
	
	Stream * serialPort = NULL;

	uint16_t rx_read_ptr;
	uint16_t rx_write_ptr;
	uint16_t bytes_left;

	uint8_t rx_buffer[BUFFER_LEN];
	uint8_t tx_buffer[BUFFER_LEN];

};

#endif
