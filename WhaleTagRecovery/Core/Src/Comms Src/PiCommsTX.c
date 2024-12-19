/*
 * PiCommsTX.c
 *
 *  Created on: Aug 18, 2023
 *      Author: Kaveet
 */

#include "Comms Inc/PiComms.h"
#include "Recovery Inc/GPS.h"
#include <stdlib.h>
#include <stdint.h>

//Queue to share data between this and the GPS threads
TX_QUEUE gps_tx_queue;

//External variables (uart handler)
extern UART_HandleTypeDef huart2;

#define PI_TX_BUFFER_SIZE (4)

typedef struct __attribute__ ((__packed__, scalar_storage_order ("little-endian"))) {
	PiCommHeader header;
	uint8_t msg[256];
} Packet;

static volatile uint8_t s_write_active = 0;
static volatile size_t s_tx_write_index = 0;
static volatile size_t s_tx_read_index = 0;
static Packet s_tx_buffer[PI_TX_BUFFER_SIZE];
TX_MUTEX s_lock;

void comms_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	// advance buffer
	s_tx_write_index = (s_tx_read_index + 1) % PI_TX_BUFFER_SIZE;
	if (s_tx_write_index != s_tx_read_index) {
		uint16_t packet_size = sizeof(PiCommHeader) + s_tx_buffer[s_tx_write_index].header.length;
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&s_tx_buffer[s_tx_write_index], packet_size); // start DMA transmission
	}
}

void pi_comms_tx_init(void) {
	tx_mutex_create(&s_lock, "PiCommTxLock", TX_NO_INHERIT); // create thread lock
	HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, comms_UART_TxCpltCallback);
	pi_comms_tx_ssid(12);
}

void pi_comms_tx_bufferWriteFromParts(const PiCommHeader *header, void *msg){
	tx_mutex_get(&s_lock, TX_WAIT_FOREVER); // lock from other threads

	// copy buffer to s_tx_buffer
	size_t next_read = (s_tx_read_index + 1) % PI_TX_BUFFER_SIZE;
	uint16_t packet_size = sizeof(PiCommHeader) + header->length;
	memcpy(&s_tx_buffer[next_read].header, header, sizeof(PiCommHeader));
	memcpy(&s_tx_buffer[next_read].msg, msg, header->length);

	HAL_NVIC_DisableIRQ(USART2_IRQn); //lock from ISR
	if (s_tx_read_index == s_tx_write_index) {
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&s_tx_buffer[next_read], packet_size); // start DMA transmission
	}
	s_tx_read_index = next_read;

	// cleanup
	HAL_NVIC_EnableIRQ(USART2_IRQn); // unlock ISR
	tx_mutex_put(&s_lock); //unlock threads
}

void pi_comms_tx_bufferWrite(const Packet *packet){
	tx_mutex_get(&s_lock, TX_WAIT_FOREVER); // lock from other threads

	// copy buffer to s_tx_buffer
	size_t next_read = (s_tx_read_index + 1) % PI_TX_BUFFER_SIZE;
	uint16_t packet_size = sizeof(PiCommHeader) + packet->header.length;
	memcpy(&s_tx_buffer[next_read], packet, packet_size);

	HAL_NVIC_DisableIRQ(USART2_IRQn); //lock from ISR
	if (s_tx_read_index == s_tx_write_index) {
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&s_tx_buffer[next_read], packet_size); // start DMA transmission
	}
	s_tx_read_index = next_read;

	// cleanup
	HAL_NVIC_EnableIRQ(USART2_IRQn); // unlock ISR
	tx_mutex_put(&s_lock); //unlock threads
}

void pi_comms_tx_forward_gps(const uint8_t *buffer, uint8_t len){
	PiCommHeader header = {
		.start_byte = PI_COMMS_START_CHAR,
		.id = PI_COMM_MSG_GPS_PACKET,
		.length = len,
	};
	pi_comms_tx_bufferWriteFromParts(&header, buffer);
}

void pi_comms_tx_pong(void){
	PiCommHeader gps_header = {
		.start_byte = PI_COMMS_START_CHAR,
		.id = PI_COMM_PONG,
		.length = 0,
	};
	pi_comms_tx_bufferWrite((Packet *)&gps_header);
}

void pi_comms_tx_callsign(const char *callsign){
	Packet pkt = {
			.header = {
				.start_byte = PI_COMMS_START_CHAR,
				.id = PI_COMM_MSG_CONFIG_APRS_CALLSIGN,
				.length = strlen(callsign),
			},
	};
	pi_comms_tx_bufferWrite(&pkt);
}

void pi_comms_tx_ssid(uint8_t ssid){
	Packet pkt = {
		.header = {
			.start_byte = PI_COMMS_START_CHAR,
			.id = PI_COMM_MSG_CONFIG_APRS_SSID,
			.length = 1,
		},
		.msg = {ssid},
	};
	pi_comms_tx_bufferWrite(&pkt);
}
