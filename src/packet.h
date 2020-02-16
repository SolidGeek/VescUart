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

#ifndef PACKET_H_
#define PACKET_H_

#include <stdint.h>
#include <stdbool.h>

// Settings
#ifndef PACKET_RX_TIMEOUT
#define PACKET_RX_TIMEOUT		100
#endif

#ifndef PACKET_HANDLERS
#define PACKET_HANDLERS			2
#endif

#ifndef PACKET_MAX_PL_LEN
#define PACKET_MAX_PL_LEN		512
#endif



class Packet
{

public:

	/**
		* @brief      Init the packet with functions for sending and processing data
		* @param      s_func - Pointer to sending function 
		* @param      p_func - Pointer to processing function 
	*/
	void init( void(*s_func)(uint8_t *data, uint16_t len), void(*p_func)(uint8_t *data, uint16_t len) );

	/**
		* @brief      Reset the package object
	*/
	void reset( void );

	/**
		* @brief      Init the packet with functions for sending and processing data
		* @param      s_func - Pointer to sending function 
		* @param      p_func - Pointer to processing function 
	*/
	void process_byte( uint8_t rx_data );
	void timerfunc( void );
	void send_packet( unsigned char *data, unsigned int len );


private:

	// Private types
	typedef struct {
		volatile unsigned short rx_timeout;
		void(*send_func)(unsigned char *data, unsigned int len);
		void(*process_func)(unsigned char *data, unsigned int len);
		unsigned int rx_read_ptr;
		unsigned int rx_write_ptr;
		int bytes_left;
		unsigned char rx_buffer[BUFFER_LEN];
		unsigned char tx_buffer[BUFFER_LEN];
	} PACKET_STATE_t;

}
#endif /* PACKET_H_ */
