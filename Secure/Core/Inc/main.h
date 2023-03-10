/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined ( __ICCARM__ )
#  define CMSE_NS_CALL  __cmse_nonsecure_call
#  define CMSE_NS_ENTRY __cmse_nonsecure_entry
#else
#  define CMSE_NS_CALL  __attribute((cmse_nonsecure_call))
#  define CMSE_NS_ENTRY __attribute((cmse_nonsecure_entry))
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"
#include "secure_nsc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "inttypes.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

// - - - - I M P O R T A N T - - - -
// NOTE: I had deleted that part accidentally previously!
/* Function pointer declaration in non-secure*/
#if defined ( __ICCARM__ )
typedef void (CMSE_NS_CALL *funcptr)(void);
#else
typedef void CMSE_NS_CALL (*funcptr)(void);
#endif

/* typedef for non-secure callback functions */
typedef funcptr funcptr_NS;
// - - - - - - - - - - - - - - - - - -

/* USER CODE BEGIN ET */

#define PCLK1_FREQ			((uint64_t)110000000)														// Run AHB at 110 MHz ==> PCLK1 = PCLK2 := 110MHz
#define RBUF_LEN 			512

// Needed for RNG, taken from stm32l5xx_hal_rng.h
#define RNG_HTCFG_1   0x17590ABCU /*!< Magic number */
#define RNG_HTCFG     0x0000A2B3U /*!< Recommended value for NIST compliancy */


// Profiling macros
#define PROFILING
//#define PROFILING_REQUEST_TRUSTZONE
#define PROFILING_RESPONSE_TRUSTZONE


#define PROFILING_SET_PIN(PIN)			SET_BIT((PIN.port)->BSRR, PIN.pin)
#define PROFILING_RESET_PIN(PIN)		SET_BIT((PIN.port)->BSRR, PIN.pin << 16)					// BSRR is split into 2 16 bit registers, lower 16 bits are to set a pin, upper 16 to reset

#define PROFILING_SET_COUNTER_E(AMNT)		SET_BIT(GPIOE->BSRR, 0xFFFF << 16 | ((AMNT & 0b1) << 2) | ((AMNT & 0b10) << 3) | ((AMNT & 0b1100) << 3))
#define PROFILING_SET_COUNTER_F(AMNT)		SET_BIT(GPIOF->BSRR, 0xFFFF << 16 | ((AMNT & 0b1) << 8) | ((AMNT & 0b10) << 6) | ((AMNT & 0b100) << 7))


// - - - - - - - - - - - CONSTANTS - - - - - - - - - - -
#define TRUE 0x1
#define FALSE 0x0

#define MAX_PRINT_LEN			1024					// Max. length a debug message can have, This is used in: strnlen(..., MAX_PRINT_LEN)

#define SHA256_INPUT_SIZE 		64						// 512 bits
#define SHA256_OUTPUT_SIZE 		32						// 256 bits

#define HMAC_INPUT_SIZE         SHA256_INPUT_SIZE
#define HMAC_KEY_LEN 			32			// Used to be a full input block, but I changed it to be consistent with the non-TZ version which also uses 32 byte keys
#define HMAC_OUTPUT_SIZE		SHA256_OUTPUT_SIZE

#define MAX_PAYLOAD_LEN			HMAC_OUTPUT_SIZE		// Longest (non-signup) message we expect to receive is HMAC_OUTPUT_size == 32 bytes.

#define DEVICE_ID_LEN			4

#define REB_CNT_LEN				4
#define REQ_CNT_LEN				4
#define RANDOM_LEN				16


#define ACCESS_TYPE_LEN			2
#define CHALLENGE_LEN			(REB_CNT_LEN + REQ_CNT_LEN + ACCESS_TYPE_LEN + HMAC_OUTPUT_SIZE)						// NOTE: Challenge here is the compoint bitstring consisting of | req_cnt | req_cnt | access_type | hmac_tac |
#define RESPONSE_LEN			HMAC_OUTPUT_SIZE


#define HEADER_LEN				3
#define HEADER_TYPE_LEN			1
#define HEADER_LEN_LEN			2

#define TIMEOUT_10				10						// 10ms timeout


#define TIMER_SET_UDIS(TIMER_PTR)		SET_BIT(TIMER_PTR->CR1, TIM_CR1_UDIS)
#define TIMER_CLEAR_UDIS(TIMER_PTR)		CLEAR_BIT(TIMER_PTR->CR1, TIM_CR1_UDIS)
#define TIMER_RESET_CNT(TIMER_PTR)		CLEAR_REG(TIMER_PTR->CNT)

// - - - - - - - - - - - - - - - - - - - - - -
// 			Typedefs
// - - - - - - - - - - - - - - - - - - - - - -

typedef uint8_t byte_t;
typedef uint8_t bool_t;
typedef int8_t	err_t;
typedef uint32_t time_tt;

typedef enum{
	CONN_SERVER = 0,
	CONN_GATEWAY = 1
} conns_t;

typedef enum{
	SERVER_PAYLOAD_SIGNUP_REQ   = 0,
	SERVER_PAYLOAD_SIGNUP_RESP  = 1,
	SERVER_PAYLOAD_AUTH_REQ     = 2,
	SERVER_PAYLOAD_AUTH_RESP    = 3,
	SERVER_PAYLOAD_CONTROL      = 4
} s_payload_types_t;

typedef enum{
	GATEWAY_PAYLOAD_SIGNUP    = 0,
	GATEWAY_PAYLOAD_REQUEST   = 1,
	GATEWAY_PAYLOAD_CHALLENGE = 2,
	GATEWAY_PAYLOAD_RESPONSE  = 3,
	GATEWAY_PAYLOAD_GRANTED	  = 4,
	GATEWAY_PAYLOAD_CONTROL   = 5
} gw_payload_types_t;

typedef enum{
	LEN_SERVER_PAYLOAD_SIGNUP_REQ   = 2,             			// Lower bound, this is only the length for the device type
	LEN_SERVER_PAYLOAD_SIGNUP_RESP  = DEVICE_ID_LEN,             			// device_ID (4 bytes)
	LEN_SERVER_PAYLOAD_AUTH_REQ     = DEVICE_ID_LEN + CHALLENGE_LEN, // Authentication request payload is: |  dev_id  |  req_cnt  |  req_cnt  |  access_type  |  hmac_tag  |, where req_cnt | req_cnt | access_type | hmac_tac are the challenge
	LEN_SERVER_PAYLOAD_AUTH_RESP    = RANDOM_LEN + HMAC_OUTPUT_SIZE,           	// 16 bytes randomness followed by server MAC-Tag (32 bytes)
	LEN_SERVER_PAYLOAD_CONTROL      = 4
} s_payload_lens_t;

typedef enum{
	LEN_GATEWAY_PAYLOAD_SIGNUP    = 2,               // The gateway only dictates the device type. This is fixed throughout the device's lifetime
	LEN_GATEWAY_PAYLOAD_REQUEST   = ACCESS_TYPE_LEN,
	LEN_GATEWAY_PAYLOAD_CHALLENGE = CHALLENGE_LEN,
	LEN_GATEWAY_PAYLOAD_RESPONSE  = LEN_SERVER_PAYLOAD_AUTH_RESP,
	LEN_GATEWAY_PAYLOAD_GRANTED	  = ACCESS_TYPE_LEN,
	LEN_GATEWAY_PAYLOAD_CONTROL   = 0
} gw_payload_lens_t;


typedef enum{
	ACCESS_TYPE_SAMPLE_SENSOR_0    = 0,
	ACCESS_TYPE_SAMPLE_SENSOR_1    = 1,
	ACCESS_TYPE_CONTROL_ACTUATOR_0 = 2,
	ACCESS_TYPE_CONTROL_ACTUATOR_1 = 3,
} access_type_t;


typedef enum{
	STATUS_INACTIVE		  = 0,
	STATUS_NOT_CHALLENGED = 1,
	STATUS_NOT_AUTHED	  = 2,
	STATUS_AUTHED		  = 3
} request_status_t;

// - - - - - - - - - - - - - - - - - - - - - -
// 			Structs
// - - - - - - - - - - - - - - - - - - - - - -

typedef struct ringbuf{
	volatile int16_t prod_ind;
	volatile int16_t cons_ind;
	volatile int16_t isr_ctr;																			// Ctr. shared between ISR and normal world, it tells the ISR how many more bytes it has to send before disabling itself (to prevent flood of interrupts)
	volatile bool_t   empty;
			 byte_t buf[RBUF_LEN];
}ringbuf_t;

typedef struct message {
	byte_t payload_type;
	uint32_t payload_len;
	byte_t p_payload[64];																				// 64 byte buffer, UNALIGNED
	time_tt rx_time;
}msg_t;

// Struct holding PAYLOAD (i.e. without header). Hence, we have to explicitly include a PAYLOAD_TYPE field
typedef struct gw_msg{
	uint8_t		payload_type;
//	uint16_t	payload_type;
	byte_t 		payload_buf[64];
} gw_msg_t;

// Holds message length instead of payload type because the msg_buf includes a header which encodes the message type
typedef struct s_msg{
//	uint8_t		payload_type;
	uint16_t	msg_len;
	byte_t 		msg_buf[64];
} s_msg_t;

typedef struct msg_flags{
	bool_t 	 rx_done;					// Flag that we are done reading a message
	bool_t 	 tx_rdy;					// Flag that we are ready to send a message
} msg_flags_t;


typedef struct keys{
	byte_t		key_gw_s[HMAC_KEY_LEN];
	byte_t		key_s_gw[HMAC_KEY_LEN];
}keys_t;

typedef struct cnts{
	uint32_t	reb_cnt;
	uint32_t	req_cnt;
}cntrs_t;

// - - - - - - - - CPU specific structs - - - - - - - -
typedef struct req {
	bool_t    	valid;
	uint16_t  	access_type;
	time_tt   	timer_set_time;
	time_tt   	timeout;
	uint32_t	ref_reb_cnt;							// Reboot counter value at time when the request was generated
	uint32_t	ref_req_cnt;							// Request counter value at time when the request was generated
	byte_t*		tag_ptr;								// Challenge has format: |  reb_cnt  |  req_cnt  |  access_type  |  mac_tag  |. tag_ptr points to the beginning of the mac_tag. This is purely for convenience
	byte_t		challenge[CHALLENGE_LEN];
	byte_t		randomness[RANDOM_LEN];
} req_t;


typedef struct glob_state {
	char*		dev_cap;					// Device capabilities URL
	uint16_t 	dev_type;				// Device type
	bool_t		signed_up;
	uint32_t  	dev_id;						// Device ID
} glob_state_t;

// Profiling Pin
typedef struct profiling_pin{
	GPIO_TypeDef* port;
	uint32_t pin;
}profiling_pin_t;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// - - - - - - - - - - - - - - - - - - - - - -
// 			Global variables
// - - - - - - - - - - - - - - - - - - - - - -

// Declare GPIO port used for the button
GPIO_TypeDef* gpio_button_port;
uint32_t		  gpio_button_pin;

// Declare the two uarts
USART_TypeDef* lpuart;
USART_TypeDef* usart2;

// Declare the aliases which are used when working on a higher abstraction level
USART_TypeDef* s_uart;
USART_TypeDef* gw_usart;

// DEVICE TYPE
glob_state_t dev_state_s;

// FLAGS
msg_flags_t s_flags;
msg_flags_t gw_flags;

//bool_t flag_msg_rdy;
//bool_t flag_send_message;
volatile bool_t flag_currently_granting;
volatile uint16_t flag_type_currently_granting;											// If flag_currently_granting == TRUE, this "flag" (not really a flag) holds the access type that is currently being granted!

int16_t process_bytes_state;
uint32_t rx_msg_buf[64 / 4];
uint16_t rx_msg_payload_len;

//ringbuf_t lp1_tx_rbuf;
//ringbuf_t lp1_rx_rbuf;


// Ringbuffers for low-level send/receive routines
// Server ringbuffers
//ringbuf_t lp1_tx_rbuf;
//ringbuf_t lp1_rx_rbuf;

// Variables for high-level state keeping and message processing
req_t req_state;
keys_t keys_struct;
cntrs_t cntrs_struct;

// Profiling variables
GPIO_TypeDef* gpio_profiling_port_0;
GPIO_TypeDef* gpio_profiling_port_1;

profiling_pin_t	  gpio_profiling_pin_0;
profiling_pin_t	  gpio_profiling_pin_1;
profiling_pin_t	  gpio_profiling_pin_2;
profiling_pin_t	  gpio_profiling_pin_3;
profiling_pin_t	  gpio_profiling_pin_4;
profiling_pin_t	  gpio_profiling_pin_5;
profiling_pin_t	  gpio_profiling_pin_6;
profiling_pin_t	  gpio_profiling_pin_7;


#include <helper_s.h>
//#include "s_recv_task.h"
//#include "s_proc_task.h"
//#include "gw_recv_task.h"
//#include "gw_proc_task.h"
//
//#include "send_task.h"
//#include "button_task.h"
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//#define UART_UE_Enable
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */


/* USER CODE BEGIN EFP */
void handler_func(void* x, void* y);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
