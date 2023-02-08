/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Non-secure Vector table to jump to (internal Flash Bank2 here)             */
/* Caution: address must correspond to non-secure internal Flash where is     */
/*          mapped in the non-secure vector table                             */
#define VTOR_TABLE_NS_START_ADDR  0x08040000UL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void NonSecure_Init(void);
static void MX_GTZC_S_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* SAU/IDAU, FPU and interrupts secure/non-secure allocation setup done */
  /* in SystemInit() based on partition_stm32l552xx.h file's definitions. */
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* GTZC initialisation */
  MX_GTZC_S_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  // (1) Initialize everything for nonsecure world

  // (1.1) UART Initialization
  //	   DIFFERENCE: We do NOT set up USART2 here because we do not need it.
  //	   USART2 was only used for CPU-Gateway communication, which is now handled on-chip

  // (1.1.1) Initialize variable and ALIAS with the predefined memory regions
  lpuart = LPUART1;
  s_uart = lpuart;

  // (1.1.2) Enable clock to LPUART1 and GPIO port G
  rcc_enable_lpuart();
  rcc_enable_gpiog();

  // (1.1.3) GPIO port G needs this extra step.
  rcc_set_vddio2();
//
  // (1.2) Set up GPIO pin for the button
  gpio_button_port = GPIOC;
  gpio_button_pin  = GPIO_IDR_ID13;
  rcc_enable_gpioc();


  // (2) Set access permissions for LPUART1 and the GPIO ports used
//  GTZC_TZSC_TypeDef* tzsc = GTZC_TZSC;

  // (2.1) Do NOT set security bit for LPUART1 because that would mean LPUART1 is only accessible to the secure world!
//  SET_BIT(tzsc->SECCFGR1, 0b1 << GTZC_TZSC_SECCFGR1_LPUART1SEC_Pos);

  // (2.2) Clear security bits for PG7, PG8. This means they are set to being accessible by the nonsecure world
  CLEAR_BIT(GPIOG->SECCFGR, GPIO_SECCFGR_SEC7 | GPIO_SECCFGR_SEC8);

  // (2.3) Clear security bit for PD7, the pin used to sample the button
  CLEAR_BIT(GPIOC->SECCFGR, GPIO_SECCFGR_SEC13);

//  RCC->AHB2S
//  CLEAR_BIT(RCC->SECCFGR, RCC_SECCFGR_)
//  CLEAR_BIT(RCC-)

  // (2.5) PROFILING
  rcc_enable_gpioe();
  gpio_profiling_port_0 = GPIOE;

  rcc_enable_gpiof();
  gpio_profiling_port_1 = GPIOF;

  // Clear bits for profiling pins. IMPORTANT: Can do this only AFTER enabling the GPIO ports (else we'd write to a peripheral that doesn't receive a clock yet)
  CLEAR_BIT(GPIOE->SECCFGR, GPIO_SECCFGR_SEC2 | GPIO_SECCFGR_SEC4 | GPIO_SECCFGR_SEC5 | GPIO_SECCFGR_SEC6 | GPIO_SECCFGR_SEC3);
  CLEAR_BIT(GPIOF->SECCFGR, GPIO_SECCFGR_SEC8 | GPIO_SECCFGR_SEC7 | GPIO_SECCFGR_SEC9);

  uint32_t debug_reg = GPIOE->SECCFGR;


  gpio_init_port_as_output(gpio_profiling_port_0);
  gpio_init_port_as_output(gpio_profiling_port_1);

//  gpio_profiling_port_0->B

#ifdef PROFILING_MACRO_SIGNUP_TIME
	  label_profiling_macro_signup_time:
#endif
  gpio_profiling_pin_0.port = gpio_profiling_port_0;     // PE2 on MCU, D56/SAI_A_MCLK on board
  gpio_profiling_pin_0.pin = GPIO_PIN_2;
  PROFILING_RESET_PIN(gpio_profiling_pin_0);

  gpio_profiling_pin_1.port = gpio_profiling_port_0;     // PE4 on MCU, D57/SAI_A_FS   on board
  gpio_profiling_pin_1.pin = GPIO_PIN_4;
  PROFILING_RESET_PIN(gpio_profiling_pin_1);

  gpio_profiling_pin_2.port = gpio_profiling_port_0;     // PE5 on MCU, D58/SAI_A_SCK  on board
  gpio_profiling_pin_2.pin = GPIO_PIN_5;
  PROFILING_RESET_PIN(gpio_profiling_pin_2);

  gpio_profiling_pin_3.port = gpio_profiling_port_0;     // PE6 on MCU, D59/SAI_A_SD   on board
  gpio_profiling_pin_3.pin = GPIO_PIN_6;
  PROFILING_RESET_PIN(gpio_profiling_pin_3);

  gpio_profiling_pin_4.port = gpio_profiling_port_0;     // PE3 on MCU, D60/SAI_B_SD   on board
  gpio_profiling_pin_4.pin = GPIO_PIN_3;
  PROFILING_RESET_PIN(gpio_profiling_pin_4);

  gpio_profiling_pin_5.port = gpio_profiling_port_1;     // PF8 on MCU, D56/SAI_B_SCK on board
  gpio_profiling_pin_5.pin = GPIO_PIN_8;
  PROFILING_RESET_PIN(gpio_profiling_pin_5);

  gpio_profiling_pin_6.port = gpio_profiling_port_1;     // PF7 on MCU, D56/SAI_B_MCLK on board
  gpio_profiling_pin_6.pin = GPIO_PIN_7;
  PROFILING_RESET_PIN(gpio_profiling_pin_6);

  gpio_profiling_pin_7.port = gpio_profiling_port_1;     // PF9 on MCU, D56/SAI_B_FS on board
  gpio_profiling_pin_7.pin = GPIO_PIN_9;
  PROFILING_RESET_PIN(gpio_profiling_pin_7);

//  PROFILING_SET_COUNTER_E(1);
//  HAL_Delay(10);
//  PROFILING_SET_COUNTER_E(2);
//  HAL_Delay(10);
//  PROFILING_SET_COUNTER_E(3);
//  HAL_Delay(10);
//  PROFILING_SET_COUNTER_E(4);


  // (3) Set secure world state

  // (3.1) Initialize hash peripheral
  hash_startup();

  // (3.2) Initialize timer
  timer_init(TIM7, 5 * (11000 - 1));

  // (3.3) Set device type
  dev_state_s.dev_type = 0x69;

  // (3.4) Set keys
  memset(keys_struct.key_gw_s, 0, HMAC_KEY_LEN);
  keys_struct.key_gw_s[0] = 0x69;

  memset(keys_struct.key_s_gw, 0xff, HMAC_KEY_LEN);
  keys_struct.key_s_gw[0] = 0x69;

  // (3.5) requst state initialized
  req_state.valid = FALSE;
  // NOTE: For debugging, set timeout to 100s
  req_state.timeout = 100000;
  // CHANGE: Added randomness field, will be populated by randomness from server
  memset(req_state.randomness, 0, sizeof(req_state.randomness));				// Initial value of randomness is all-zero



  // (3.6) Counters
  // TODO: Create a better initialization which actually uses flash!
  cntrs_struct.reb_cnt = 0;
  cntrs_struct.req_cnt = 0;


  /* USER CODE END 2 */

  /*************** Setup and jump to non-secure *******************************/

  NonSecure_Init();

  /* Non-secure software does not return, this code is not executed */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief  Non-secure call function
  *         This function is responsible for Non-secure initialization and switch
  *         to non-secure state
  * @retval None
  */
static void NonSecure_Init(void)
{
  funcptr_NS NonSecure_ResetHandler;

  SCB_NS->VTOR = VTOR_TABLE_NS_START_ADDR;

  /* Set non-secure main stack (MSP_NS) */
  __TZ_set_MSP_NS((*(uint32_t *)VTOR_TABLE_NS_START_ADDR));

  /* Get non-secure reset handler */
  NonSecure_ResetHandler = (funcptr_NS)(*((uint32_t *)((VTOR_TABLE_NS_START_ADDR) + 4U)));

  /* Start non-secure state software application */
  NonSecure_ResetHandler();
}

/**
  * @brief GTZC_S Initialization Function
  * @param None
  * @retval None
  */
static void MX_GTZC_S_Init(void)
{

  /* USER CODE BEGIN GTZC_S_Init 0 */

  /* USER CODE END GTZC_S_Init 0 */

  MPCBB_ConfigTypeDef MPCBB_NonSecureArea_Desc = {0};

  /* USER CODE BEGIN GTZC_S_Init 1 */

  /* USER CODE END GTZC_S_Init 1 */
  MPCBB_NonSecureArea_Desc.SecureRWIllegalMode = GTZC_MPCBB_SRWILADIS_ENABLE;
  MPCBB_NonSecureArea_Desc.InvertSecureState = GTZC_MPCBB_INVSECSTATE_NOT_INVERTED;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[0] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[1] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[2] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[3] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[4] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[5] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[6] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[7] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[8] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[9] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[10] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[11] =   0xFFFFFFFF;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[12] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[13] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[14] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[15] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[16] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[17] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[18] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[19] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[20] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[21] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[22] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[23] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_LockConfig_array[0] =   0x00000000;
  if (HAL_GTZC_MPCBB_ConfigMem(SRAM1_BASE, &MPCBB_NonSecureArea_Desc) != HAL_OK)
  {
    Error_Handler();
  }
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[0] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[1] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[2] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[3] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[4] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[5] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[6] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_SecConfig_array[7] =   0x00000000;
  MPCBB_NonSecureArea_Desc.AttributeConfig.MPCBB_LockConfig_array[0] =   0x00000000;
  if (HAL_GTZC_MPCBB_ConfigMem(SRAM2_BASE, &MPCBB_NonSecureArea_Desc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GTZC_S_Init 2 */

  /* USER CODE END GTZC_S_Init 2 */

}

/* USER CODE BEGIN 4 */

//void handler_func(void* x, void* y){
//	huge_struct_t* x_h = x;
//	huge_struct_t* y_h = y;
//	y_h->a = x_h->a + x_h->b + x_h->c + x_h->d;
//}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
