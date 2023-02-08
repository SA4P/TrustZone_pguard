/*
 * send_task.c
 *
 *  Created on: Jun 27, 2022
 *      Author: SA4P Authors
 */


#include <send_task_ns.h>

bool_t s_task_send(s_msg_t* o_msg){

	// Flag already checked in main loop, just double check. If it is for whatever reason not set, we have an error because we should only enter task_send_s if we have data to send to the server, i.e. if s_flags.tx_rdy == TRUE!
	if(!s_flags.tx_rdy){
		while(TRUE){
			__NOP();
		}
	}

	// Set flag to false
	s_flags.tx_rdy = FALSE;

	if(!rb_produce(&lp1_tx_rbuf, o_msg->msg_buf, o_msg->msg_len)){
		return FALSE;
	}

	// Set Server-UART (LPUART1) to send out message
	usart_set_TXEIE(s_uart);

	return TRUE;
}
