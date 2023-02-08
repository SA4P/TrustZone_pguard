/*
 * button_task.c
 *
 *  Created on: Jun 27, 2022
 *      Author: SA4P Authors
 */

#include <button_task_ns.h>

static bool_t state_button_pressed = FALSE;

static int bounce_counter = 0;

bool_t button_task_check_pressed(req_t* req_state){
	// (1) Check button register
	bool_t current_button_value = READ_BIT(gpio_button_port->IDR, GPIO_IDR_ID13) != FALSE;
	if(current_button_value == FALSE){
		// If button currently not considered pressed, reset pressed state such that when it is pressed the next time we will actually register it
		state_button_pressed = FALSE;
		return FALSE;
	}

	// If we reach here, the button register currently indicates that the button is pressed

	// (2) Check if the button state was already considered pressed previously ==> Don't want to double-register requests, so just do nothing then
	if(state_button_pressed == TRUE){
		return FALSE;
	}

	// If we reach here, then the button is currently considered pressed but was not considered so in the previous iteration of the function

	// (3) Set button state as pressed
	state_button_pressed = TRUE;

	bounce_counter ++;

	// (4) Set request state
	req_state->access_type = 0;		// Currently, only always use access type 0
	req_state->status      = STATUS_NOT_CHALLENGED;

	// (5) Set transmit ready flag, such that gateway send task can start sending
	gw_flags.tx_rdy = TRUE;

	return TRUE;

}
