/**
  ******************************************************************************
  * @file    Secure/Src/secure_nsc.c
  * @author  MCD Application Team
  * @brief   This file contains the non-secure callable APIs (secure world)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE BEGIN Non_Secure_CallLib */
/* Includes ------------------------------------------------------------------*/
#include <helper_s.h>
#include "main.h"
#include "secure_nsc.h"


/** @addtogroup STM32L5xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Global variables ----------------------------------------------------------*/
void *pSecureFaultCallback = NULL;   /* Pointer to secure fault callback in Non-secure */
void *pSecureErrorCallback = NULL;   /* Pointer to secure error callback in Non-secure */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Secure registration of non-secure callback.
  * @param  CallbackId  callback identifier
  * @param  func        pointer to non-secure function
  * @retval None
  */
CMSE_NS_ENTRY void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func)
{
  if(func != NULL)
  {
    switch(CallbackId)
    {
      case SECURE_FAULT_CB_ID:           /* SecureFault Interrupt occurred */
        pSecureFaultCallback = func;
        break;
      case GTZC_ERROR_CB_ID:             /* GTZC Interrupt occurred */
        pSecureErrorCallback = func;
        break;
      default:
        /* unknown */
        break;
    }
  }
}


 uint16_t CMSE_NS_ENTRY SECURE_request_signup(){
	 return dev_state_s.dev_type;
 }
 uint8_t CMSE_NS_ENTRY SECURE_request_access(uint16_t access_type, void* o_chal_buf){		// chal_buf is a pointer to a buffer which will be populated in the format:
	  #ifdef PROFILING_REQUEST_TRUSTZONE
		 // 1: NSC called
		   PROFILING_SET_COUNTER_E(1);
	  #endif																					// | reboot_counter (4 bytes) | request_counter (4 bytes) | access_type (2 bytes)
	 return handle_access_request(access_type, o_chal_buf);
 }
 uint8_t CMSE_NS_ENTRY SECURE_forward_s_msg(void* i_gw_msg){								// i_msg is a pointer to a gw_msg_t.
	#ifdef PROFILING_RESPONSE_TRUSTZONE
		// 1: NSC called
		PROFILING_SET_COUNTER_E(1);
	#endif
	 return handle_server_msg(i_gw_msg);
 }

/**
  * @}
  */

/**
  * @}
  */
/* USER CODE END Non_Secure_CallLib */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
