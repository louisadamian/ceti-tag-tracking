/*
 * PiCommsRX.c
 *
 *  Created on: Aug 17, 2023
 *      Author: Kaveet
 */


#include "Comms Inc/PiComms.h"
#include "Lib Inc/state_machine.h"
#include "Lib Inc/threads.h"
#include "Recovery Inc/VHF.h"
#include "config.h"
#include "main.h"
#include "stm32u5xx_hal_uart.h"
#include "stm32u5xx_hal_uart_ex.h"


//External variables
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;

extern Thread_HandleTypeDef threads[NUM_THREADS];
extern TX_EVENT_FLAGS_GROUP state_machine_event_flags_group;

//Private typedef


//Data buffer for receives
static uint8_t piComms_raw_rx_buffer[PI_COMM_RX_BUFFER_COUNT*PI_COMM_RX_BUFFER_SIZE];
uint8_t pi_comm_rx_buffer[PI_COMM_RX_BUFFER_COUNT][PI_COMM_RX_BUFFER_SIZE] = {0}; //max rx size = 3 + 4 = 7
volatile uint_fast8_t pi_comm_rx_buffer_start = 0;
volatile uint_fast8_t pi_comm_rx_buffer_end = 0;
volatile bool pi_comm_rx_buffer_overflow = false;

TX_EVENT_FLAGS_GROUP pi_comms_event_flags_group;

/**
 * @param Size number of bytes read into buffer via dma
 */
void piComm_UARTEx_RxEventCallback(uint16_t Size) {
	static size_t remaining = 0;
	int new = 0;
	int i = 0;
	// rebuffer messages
	Size += remaining;
	while ( i < Size ) {
		if (piComms_raw_rx_buffer[i] != PI_COMMS_START_CHAR) {
			i++;
			continue;
		}
			
		// get packet size
		if (i + sizeof(PiCommHeader) > Size) {
			break;
		}
		size_t pkt_length = sizeof(PiCommHeader) + ((PiCommHeader *)&piComms_raw_rx_buffer[i])->length;
		if (i + pkt_length >= Size) {
			break;
		}

		// copy into packet buffers
		memcpy(pi_comm_rx_buffer[pi_comm_rx_buffer_end], &piComms_raw_rx_buffer[i], pkt_length);

		// advance positions
		new = 1;
		pi_comm_rx_buffer_end = (pi_comm_rx_buffers_end + 1) % PI_COMM_RX_BUFFER_COUNT;
		// ToDo: check overflow
		i += pkt_length;
	}

	if (new) {
		tx_event_flags_set(&state_machine_event_flags_group, STATE_COMMS_MESSAGE_AVAILABLE_FLAG, TX_OR);
	}

	remaining = Size - i;
	if (remaining) {
		//shift memory
		/* ToDo: (MSH 2024_11_16) This can maybe be done more efficiently by
		 * converting the buffer into a circular buffer so this move doesn't 
		 * need to occur? 
		 */ 
		memmove(&piComms_raw_rx_buffer[0], &piComms_raw_rx_buffer[i], remaining);
	}
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &piComms_raw_rx_buffer[remaining], sizeof(piComms_raw_rx_buffer) - remaining); //initiate next transfer
	__HAL_DMA_DISABLE_IT(&handle_GPDMA1_Channel2,DMA_IT_HT);//we don't want the half done transaction interrupt

}

/* ToDo: (MSH 2024_11_16) This no longer needs to be a thread as rx parsing is
 * now purely interrupt driven.
 */ 
void pi_comms_rx_thread_entry(ULONG thread_input){
	tx_event_flags_create(&pi_comms_event_flags_group, "Pi Comms RX Event Flags");

	//initiate UART DMA
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, piComms_raw_rx_buffer, sizeof(piComms_raw_rx_buffer));
	__HAL_DMA_DISABLE_IT(&handle_GPDMA1_Channel2, DMA_IT_HT);//we don't want the half done transaction interrupt

	while (1) {
		// nothing to do
		// messages automatically buffered via the dma
		tx_thread_sleep(tx_s_to_ticks(1));
	}
}

