/**
  ******************************************************************************
  * @file    Secure_nsclib/secure_nsc.h
  * @author  MCD Application Team
  * @brief   Header for secure non-secure callable APIs list
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
/* USER CODE BEGIN Non_Secure_CallLib_h */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SECURE_NSC_H
#define SECURE_NSC_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

// CUSTOM INCLUDES
#include "string.h"


/* Exported types ------------------------------------------------------------*/
/**
  * @brief  non-secure callback ID enumeration definition
  */
typedef enum
{
  SECURE_FAULT_CB_ID     = 0x00U, /*!< System secure fault callback ID */
  GTZC_ERROR_CB_ID       = 0x01U  /*!< GTZC secure error callback ID */
} SECURE_CallbackIDTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func);

uint16_t  SECURE_request_signup();
uint8_t   SECURE_request_access(uint16_t access_type, void* o_chal_buf);		// chal_buf is a pointer to a buffer which will be populated in the format:
																						// | reboot_counter (4 bytes) | request_counter (4 bytes) | access_type (2 bytes) | mac_tag (32 bytes) |

uint8_t	  SECURE_forward_s_msg(void* resp_msg);											// resp_msg is a pointer to a gw_msg_t with payload_buf field of the format:
																						// |  randomness(RANDOM_LEN)  |  auth_tag(HMAC_OUTPUT_LEN)  |
																						// We don't need an explicit granted message because here the return value is a boolean value either TRUE (access granted) or FALSE

//bool_t SECURE_response(byte_t* rand_buf, byte_t* mac_buf);


#endif /* SECURE_NSC_H */
/* USER CODE END Non_Secure_CallLib_h */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
