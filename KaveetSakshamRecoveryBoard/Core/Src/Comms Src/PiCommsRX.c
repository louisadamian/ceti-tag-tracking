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

//External variables
extern UART_HandleTypeDef huart2;
extern Thread_HandleTypeDef threads[NUM_THREADS];
extern TX_EVENT_FLAGS_GROUP state_machine_event_flags_group;

//Private typedef


//Data buffer for receives
uint8_t dataBuffer[256] = {0};

TX_EVENT_FLAGS_GROUP pi_comms_event_flags_group;

void comms_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if (dataBuffer[0] == PI_COMMS_START_CHAR){
		tx_event_flags_set(&pi_comms_event_flags_group, PI_COMMS_VALID_START_FLAG, TX_OR);
	}
	else {
		tx_event_flags_set(&pi_comms_event_flags_group, PI_COMMS_BAD_START_FLAG, TX_OR);
	}

}
void pi_comms_rx_thread_entry(ULONG thread_input){

	tx_event_flags_create(&pi_comms_event_flags_group, "Pi Comms RX Event Flags");

	HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, comms_UART_RxCpltCallback);
	HAL_GPIO_WritePin(APRS_PD_GPIO_Port, APRS_PD_Pin, GPIO_PIN_SET);
	//Start a non-blocking 1 byte UART read. Let the RX complete callback handle the rest.
	HAL_UART_Receive_IT(&huart2, (uint8_t *) dataBuffer, 1);
	while (1) {
		ULONG actual_flags;

		//Wait for the Interrupt callback to fire and set the flag (telling us what to do further)
		tx_event_flags_get(&pi_comms_event_flags_group, PI_COMMS_VALID_START_FLAG | PI_COMMS_BAD_START_FLAG, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

		if (actual_flags & PI_COMMS_VALID_START_FLAG){

			//Read message ID and data length
			HAL_UART_Receive(&huart2, &dataBuffer[1], 2, HAL_MAX_DELAY);

			//Now read the entire payload
			HAL_UART_Receive(&huart2, &dataBuffer[3], dataBuffer[2], HAL_MAX_DELAY);

			HAL_UART_Receive_IT(&huart2, (uint8_t *) dataBuffer, 1);
			//parse the message and act accordingly
			pi_comms_parse_message(dataBuffer[1], &dataBuffer[3], dataBuffer[2]);
		}
		else {
			HAL_UART_Receive_IT(&huart2, (uint8_t *) dataBuffer, 1);
		}

	}

}

void pi_comms_parse_message(PiCommsMessageID message_id, uint8_t * payload_pointer, uint8_t payload_length){

	switch (message_id) {

		case PI_COMM_MSG_START:

			//Publish event flag to state machine to enter recovery
			tx_event_flags_set(&state_machine_event_flags_group, STATE_COMMS_APRS_FLAG, TX_OR);
			break;

		case PI_COMM_MSG_STOP:

			//Publish event flag to state machine to stop (and wait for more commands)
			tx_event_flags_set(&state_machine_event_flags_group, STATE_COMMS_STOP_FLAG, TX_OR);
			break;

		case PI_COMM_MSG_COLLECT_ONLY:

			//Publish event flag to state machine to enter regular GPS collection
			tx_event_flags_set(&state_machine_event_flags_group, STATE_COMMS_COLLECT_GPS_FLAG, TX_OR);
			break;

		case PI_COMM_MSG_CRITICAL:

			//Publish event flag to state machien to enter a critical low power state (nothing runs)
			tx_event_flags_set(&state_machine_event_flags_group, STATE_CRITICAL_LOW_BATTERY_FLAG, TX_OR);
			break;

		case PI_COMM_MSG_CONFIG_CRITICAL_VOLTAGE: {
				PiCommCritVoltagePkt *crit_voltage = (PiCommCritVoltagePkt *)payload_pointer;
				g_config.critical_voltage = crit_voltage->value;
			}
			break;

		case PI_COMM_MSG_CONFIG_VHF_POWER_LEVEL: {
				PiCommTxLevelPkt *tx_pwr_lvl = (PiCommTxLevelPkt *)payload_pointer;
				g_config.vhf_power = tx_pwr_lvl->value;
				vhf_set_power_level(g_config.vhf_power);
			}
			break;

		default:
			//Bad message ID - do nothing
			break;

	}

}
