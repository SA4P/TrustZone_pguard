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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // NOTE: We ASSUME here that LPUART1 and the GPIO pins needed have been assigned by
  //	   the secure world to the nonsecure world! Otherwise, accessing them would cause an exception.

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  // (1) General setup

  // (1.1) Ringbuffers initialization
  //	   DIFFERENCE: We here only have one pair of ringbuffers: only for the server communication.
  //			   	   Secure world communication handled in a synchronous, blocking way.
  //	  	  	  	   Server communicates via LPUART1 ==> Set up only that ringbuffer
  rb_init(&lp1_rx_rbuf, 'a');
  rb_init(&lp1_tx_rbuf, 'b');


  // (1.2) Flags initialization

  // (1.2.1) Server flags initialization
  s_flags.rx_done = FALSE;
  s_flags.tx_rdy  = FALSE;

  // (1.2.2) Gateway flags initialization
  gw_flags.rx_done = FALSE;
  gw_flags.tx_rdy  = FALSE;

  // (1.3) Currently granting flag initialization
  flag_currently_granting = FALSE;

  // (1.4) Set up device type and capabilities URL
  dev_state_ns.dev_cap 	= "https://github.com/SA4P";
  dev_state_ns.dev_type	= 0;
  dev_state_ns.dev_id   = 0;

  // (1.3) UART Initialization
  //	   DIFFERENCE: We do NOT set up USART2 here because we do not need it.
  //	   USART2 was only used for CPU-Gateway communication, which is now handled on-chip

  // (1.3.1) Initialize variable and ALIAS with the predefined memory regions
  lpuart = LPUART1;
  s_uart   = lpuart;

  // (1.3.2) Enable clock to LPUART1 and GPIO port G
  rcc_enable_lpuart();
  rcc_enable_gpiog();

  // (1.3.3) Give power to GPIO port G
  rcc_set_vddio2();


  // (1.3.4) Set up GPIO port G such that the LPUART1-specific pins use their alternative function (i.e. LPUART1)  gpio_init_uart(lpuart);

  // (1.3.5) Set up control registers of LPUART1
  usart_init_cr1(lpuart);
  usart_init_cr2(lpuart);
  usart_init_cr3(lpuart);
  usart_init_baudrate(lpuart, 460800);

  // (1.3.6) Enable LPUART1
  usart_enable(lpuart);

  // (1.3.7) Enable transmission
  usart_enable_transmit(lpuart);

  // (1.3.8) Enable receiving
  usart_enable_receive(lpuart);

  // (1.3.9) Enable event generation (RXNE event, i.e. event when RDR register is not empty) inside LPUART1.
  usart_set_RXNEIE(lpuart);

  // (1.3.10) Enable LPUART1 interrupts. I.e. events generated inside LPUART1 now actually interrupt the CPU (if this flag was not set, events would not reach CPU)

  NVIC_EnableIRQ(LPUART1_IRQn);

  // (1.4) Set up GPIO pins for the button
  gpio_button_port = GPIOC;
  gpio_button_pin  = GPIO_IDR_ID13;

  rcc_enable_gpioc();
  gpio_init_button();

  bool_t button_event_flag = FALSE;

  // (1.5) Set up request state (used by the nonsecure world for state keeping)
  //	   and a send_msg_t buffer which is used to transfer messages between nonsecure and secure world
  req_t	   req_state;
  s_msg_t  s_msg_struct;
  gw_msg_t gw_msg_struct;
  // (1.6) General helper variables
  bool_t success;

  msg_t s_msg_state;

  uint8_t payload_type;

  // (1.7) Sign up

  // (1.7.1) Get device type
  dev_state_ns.dev_type = SECURE_request_signup();

  // (1.7.2) Create signup message
  success = s_msg_build(SERVER_PAYLOAD_SIGNUP_REQ, NULL, &s_msg_struct);
  if(!success){
	  // Case: Signup request creation failed
	  while(TRUE){
		  __NOP();
	  }
  }

  // (1.7.3) Send message to server
  s_flags.tx_rdy = TRUE;
  success = s_task_send(&s_msg_struct);
  if(!success){
	  // Case: Signup request could not be sent out
	  while(TRUE){
		  __NOP();
	  }
  }











  // PROFILING
  rcc_enable_gpioe();
  gpio_profiling_port_0 = GPIOE;

  rcc_enable_gpiof();
  gpio_profiling_port_1 = GPIOF;

  gpio_init_port_as_output(gpio_profiling_port_0);
  gpio_init_port_as_output(gpio_profiling_port_1);

//  gpio_profiling_port_0->B

  gpio_profiling_pin_0.port = gpio_profiling_port_0;     // PE2 on MCU, D56/SAI_A_MCLK on board
  gpio_profiling_pin_0.pin = GPIO_PIN_2;
//  PROFILING_RESET_PIN(gpio_profiling_pin_0);

  gpio_profiling_pin_1.port = gpio_profiling_port_0;     // PE4 on MCU, D57/SAI_A_FS   on board
  gpio_profiling_pin_1.pin = GPIO_PIN_4;
//  PROFILING_RESET_PIN(gpio_profiling_pin_1);

  gpio_profiling_pin_2.port = gpio_profiling_port_0;     // PE5 on MCU, D58/SAI_A_SCK  on board
  gpio_profiling_pin_2.pin = GPIO_PIN_5;
//  PROFILING_RESET_PIN(gpio_profiling_pin_2);

  gpio_profiling_pin_3.port = gpio_profiling_port_0;     // PE6 on MCU, D59/SAI_A_SD   on board
  gpio_profiling_pin_3.pin = GPIO_PIN_6;
//  PROFILING_RESET_PIN(gpio_profiling_pin_3);

  gpio_profiling_pin_4.port = gpio_profiling_port_0;     // PE3 on MCU, D60/SAI_B_SD   on board
  gpio_profiling_pin_4.pin = GPIO_PIN_3;
//  PROFILING_RESET_PIN(gpio_profiling_pin_4);

  gpio_profiling_pin_5.port = gpio_profiling_port_1;     // PF8 on MCU, D56/SAI_B_SCK on board
  gpio_profiling_pin_5.pin = GPIO_PIN_8;
//  PROFILING_RESET_PIN(gpio_profiling_pin_5);

  gpio_profiling_pin_6.port = gpio_profiling_port_1;     // PF7 on MCU, D56/SAI_B_MCLK on board
  gpio_profiling_pin_6.pin = GPIO_PIN_7;
//  PROFILING_RESET_PIN(gpio_profiling_pin_6);

  gpio_profiling_pin_7.port = gpio_profiling_port_1;     // PF9 on MCU, D56/SAI_B_FS on board
  gpio_profiling_pin_7.pin = GPIO_PIN_9;
//  PROFILING_RESET_PIN(gpio_profiling_pin_7);


  for(int i = 0; i < 16; ++i){
	  PROFILING_SET_COUNTER_E(i);
	  HAL_Delay(10);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // (2) Main loop of insecure world
  while (1)
  {

	  // (1) Handle button press event

	  // (1.1) Check if button was pressed.
	  //	   DIFFERENCE 1: We here do NOT have a gw_flags.tx_rdy flag anymore!
	  //					 This is mostly a style thing but I actually prefer it without the flag
	  //	   DIFFERENCE 2: gw_msg_struct->msg_buff does NOT hold a header anymore because we pass the whole
	  //					 gw_msg_struct (as a pointer) to the secure world ==> secure world has access to length field
	  button_event_flag = button_task_check_pressed(&req_state);

	  if(button_event_flag){
		  // Case: Button was pressed
		  //	   ==> req_state.challenge will hold the challenge if success == TRUE.

  		  #ifdef PROFILING_REQUEST_TRUSTZONE
			  // 0: Button pressed
			  PROFILING_SET_COUNTER_E(0);
		  #endif

		  success = SECURE_request_access(req_state.access_type, req_state.challenge);

		  #ifdef PROFILING_REQUEST_TRUSTZONE
		      // 8: Received complete challenge message
			  PROFILING_SET_COUNTER_E(8);
		  #endif

		  if(!success){
			  // Case: Request declined from gateway
			  //			   ==> gw_msg_struct holds garbage.
			  //			   For debugging purposes, we lock up in this case as it should
			  //			   under normal circumstances not happenZ
			  while(TRUE){
				  __NOP();
			  }
		  }

		  // (1.2) If we reached here, we have a fresh challenge ==> Set request status accordingly
		  req_state.status = STATUS_NOT_AUTHED;

		  // (1.3) Translate GATEWAY_PAYLOAD_CHALLENGE stored in gw_msg_struct into SERVER_PAYLOAD_AUTH_REQ.
		  s_build_auth_req(&req_state, &s_msg_struct);


		  // (1.4) Set send transmit ready flag and send s_msg_struct (of type SERVER_PAYLOAD_AUTH_REQ) to server
		  s_flags.tx_rdy = TRUE;
		  success = s_task_send(&s_msg_struct);
		  if(!success){
			  while(TRUE){
				  // Case: Commencing to send failed. This should NOT happen, a possible cause is that
				  //	   the transmit ringbuffer is full. This should also never happen though.
				  __NOP();
			  }
		  }
	  }

	  // (2) Handle server message event
	  // (2.1) Try to receive from server
	  s_task_receive(&s_msg_state);

	  // (2.2) If we received something from the server, process it.
	  //	   If processing is successful and if something has to be sent to the secure world (gateway),
	  //	   it is placed in the o_msg struct, which already has type gw_msg_t.
	  //	   DIFFERENCE: Here, because we have a different interface into the secure world than we had over UART,
	  //	   we need to know the payload type we are transferring into the secure world (such that we can call the appropriate function).
	  //	   Right now we only have responses which are forwarded to the secure world, but we might later want to also transfer control/ more complex signup messages
	  if(s_flags.rx_done){
		  success = s_task_process_msg(&s_msg_state, &req_state, &payload_type, &gw_msg_struct);
		  if(!success){
			  // Case: Processing failed, e.g. malformed message.
			  //	   This should not happen in our current testsetup with benign parties
			  while(TRUE){
				  __NOP();
			  }
		  }

		  // (2.3) If processing has created something that should be given to secure world (gateway)
		  //	   call the corresponding function

		  if(gw_flags.tx_rdy){

			  // Unset flag to prevent double-sending
			  gw_flags.tx_rdy = FALSE;
			  switch (payload_type) {
				case GATEWAY_PAYLOAD_RESPONSE:

					// Case: SECURE_forward_response created a message of type GATEWAY_PAYLOAD_RESPONSE

					#ifdef PROFILING_RESPONSE_TRUSTZONE
					    // 0: Response received from Server
					    PROFILING_SET_COUNTER_E(0);
					#endif
					success = SECURE_forward_s_msg(&gw_msg_struct);
					#ifdef PROFILING_RESPONSE_TRUSTZONE
						// 9: Grant received by CPU
						PROFILING_SET_COUNTER_E(9);
					#endif
					if(!success){
						// Case: Secure world was not happy with the response it received
						//		 ==> No access granted
						__NOP();
					}
					else{
						// Case: Secure world has granted access
						__NOP();
						// (2.4) Grant access
						// TODO: Grant access
					}
					break;
				default:
					// Case: s_task_process_msg created a non-response payload for the secure world.
					//       So far, we should only have those payloads ==> error
					while(TRUE){
						__NOP();
					}
					break;
			  }
		  }

		  // The third case, i.e. where secure world (gateway) sends something to nonsecure world (CPU)
		  // does NOT exist in our current protocol (except for signup, which happens outside the main loop)
		  // because the secure world only reacts to calls from the nonsecure world ==> Nonsecure world gets reaction already as return value.
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

//  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//			DEBUGGING: Polling UART Send code
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//  // NOTE: Added this polling send for debugging purposes
//  USART_TypeDef* x_nonsecure = s_uart;
//
//  char debug_buf[16];
//  memcpy(debug_buf, "HELLO WORLD!\r\n", strlen("HELLO WORLD!\r\n") + 1);
//
//  for (int i = 0; i < strlen("HELLO WORLD!\r\n") + 1; ++i){
//
//	  // (D1.1) Wait until transmit data register is empty
//	  while(READ_BIT(s_uart->ISR, USART_ISR_TXE) == FALSE){
//		  __NOP();
//	  }
//
//	  // (D1.2) Populate TDR
//	  s_uart->TDR = debug_buf[i];
//
//  }
//
//
//  while(TRUE){
//	  __NOP();
//  }

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
