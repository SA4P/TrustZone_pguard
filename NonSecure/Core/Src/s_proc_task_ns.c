/*
 * s_proc_task.c
 *
 *  Created on: Jun 27, 2022
 *      Author: SA4P Authors
 */


#include <s_proc_task_ns.h>

uint16_t s_proc_get_access_type(byte_t* payload);
bool_t	 s_proc_set_req_state(uint16_t access_type, req_t *req_state);
void 	 s_proc_signup_resp(msg_t* msg_state);


bool_t s_task_process_msg(msg_t* msg_state, req_t* req_state, uint8_t* payload_type, gw_msg_t* o_gw_msg){
	s_flags.rx_done = FALSE;

	// (0) Set payload type
	*payload_type = msg_state->payload_type;

	if(msg_state->payload_type == SERVER_PAYLOAD_SIGNUP_RESP){
		// Case 1: Received a signup response from server


		// (1.1) Check if we are already signed up
		// TODO: Handle case of duplicated signup more gracefully
		if(dev_state_ns.signed_up){
			while(TRUE){
				__NOP();
			}
		}

		// (1.2) Process signup message
		s_proc_signup_resp(msg_state);

		return TRUE;
	}

	if(msg_state->payload_type == SERVER_PAYLOAD_AUTH_RESP){
		// Case 2: Received authentication request from server

		// (2.1) Check if current request doesn't have expected state (we should be in the state "not authenticated").
		// TODO: Handle case where unexpected authentication response comes more gracefully
		if(req_state->status != STATUS_NOT_AUTHED){
			while(TRUE){
				__NOP();
			}
		}


		// (2.2) Update the request_state's status
		req_state->status = STATUS_AUTHED;

		// (2.3) Set response in request state
		memcpy(req_state->response, msg_state->p_payload, LEN_SERVER_PAYLOAD_AUTH_RESP);

		// (2.3) Build response message to be sent to the gateway. If an error occurs, we fail in an infinite loop
		if(!gw_msg_build(GATEWAY_PAYLOAD_RESPONSE, req_state, o_gw_msg)){
			while(TRUE){
				__NOP();
			}
		}

		// (2.4) Set send flag, such that the gateway send task then later starts sending
		gw_flags.tx_rdy = TRUE;

		return TRUE;
	}

	if(msg_state->payload_type == SERVER_PAYLOAD_CONTROL){
		// Not yet implemented
		while(TRUE){
			__NOP();
		}
	}

	return FALSE;
}

uint16_t s_proc_get_access_type(byte_t* payload){
	return ((uint16_t*)payload)[0];
}


void s_proc_signup_resp(msg_t* msg_state){
	memcpy(&dev_state_ns.dev_id, msg_state->p_payload, DEVICE_ID_LEN);
	dev_state_ns.signed_up = TRUE;
}
