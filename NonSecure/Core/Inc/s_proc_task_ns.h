/*
 * proc_task.h
 *
 *  Created on: Apr 3, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_S_PROC_TASK_NS_H_
#define INC_S_PROC_TASK_NS_H_

#include "main.h"

bool_t s_task_process_msg(msg_t* msg_state, req_t* req_state, uint8_t* payload_type, gw_msg_t* o_gw_msg);

#endif /* INC_S_PROC_TASK_NS_H_ */
