/*
 * helper.c
 *
 *  Created on: Jun 27, 2022
 *      Author: SA4P Authors
 */

#include <helper_s.h>
// - - - - - - - - - - - - - - -
// RCC
// - - - - - - - - - - - - - - -


// Enables GPIO port G wrt. RCC (i.e. such that GPIO port G is functional).
// This is necessary of LPUART I think
void rcc_enable_gpiog(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
}

void rcc_enable_gpioc(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
}

void rcc_enable_gpioe(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
}

void rcc_enable_gpiof(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
}

// Enables LPUART1 wrt. RCC (i.e. such that LPUART1 is functional. Otherwise, writing to/ reading from the peripheral would not even reach the peripheral)
void rcc_enable_lpuart(){
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
}

// Enables clock source of USART2, such that it can be programmed and interacted with
void rcc_enable_usart2(){
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART2EN);
}

void rcc_set_vddio2(){
	PWR->CR2 = PWR_CR2_IOSV;
}

// - - - - - - - - - - - - - - -
// GPIO
// - - - - - - - - - - - - - - -
void gpio_init_uart(USART_TypeDef * uart){
	GPIO_TypeDef * gpio;
	if (uart == LPUART1){										// Case: Configure GPIO pins for LPUART1 ==> Pins PG7 and PG8 (i.e. 92, 93) in our case

		/*
		 * We operate on pins Pins PG7 and PG8 (i.e. 92, 93). Hence, we first set gpio to point to the corresponding registers
		 * OSPEEDR: Left at reset, meaning low speed.
		 * PUPDR:   Left at reset because we want the pin to neither be pull-up or pull-down.
		 * AFRL:    Here this is AFR[0]. Controls the alternate function of pins PG0 to PG7 (inclusive). Each pin gets 4 bits of the reg (pin i has bits (i*4+3, i*4+2, i*4+1, i*4)).
		 * AFRH:    Here this is AFR[1]. Controls the alternate function of pins PG8 to PG15 (inclusive).
		 * MODER:   Controls whether a pin is in input mode (0b00), gen-purp. output mode (0b01), alternate function mode (0b10) or analog mode (reset state) (0b11). By default, each pin has MODER value 0b11 (IN PORT G!!! DOES NOT HOLD FOR PORTS A, B, H).
		 */

		gpio = GPIOG;

		gpio->AFR[0] |= GPIO_AF8_LPUART1 << 28;						// GPIO_AF8_LPUART1 has value 0b00001000, which in turn is shifted by 28 bits such that the bit at position 3 (starting at 0) arrives at bit 31, the MSB of AFRL which corresponds to the 3rd bit (starting at 0) of PG7. This is the pin we need to set to 1
		gpio->AFR[1] |= GPIO_AF8_LPUART1;							// PG8's alternative function is set using the 4LSB of AFRH, which is indexed by AFR[1] (I think). Hence, we can just set AFR[1] to GPIO_AF8_LPUART1

		// ASSUMPTION: MODER[14] == MODER[16] == 0
		gpio->MODER ^= (0x1 << 14 | 0x01 << 16);	// MODER is by default set to 0b1111...1 (i.e. all 1 bits). We want the 7th and 8th pin (with bit fields 15:14, 17:16 resp.) to have value 0b10 (Alternative function mode)
																// ==> xor MODER's 14th and 16th bit with 0x01 to get the 14th and 16th bits of MODER to be 0

//		gpio->MODER |=

	}
	else if(uart == USART2){											// Case: Configure GPIO pins for USART2 ==> Pins PD5 and PD6 (i.e. 119, 122) in our case

		gpio = GPIOD;

		gpio->AFR[0] |= GPIO_AF7_USART2 << 20 | GPIO_AF7_USART2 << 24;	// AF7 is the alternative function description of USART2 for the GPIO pins PD5, PD6

		// ASSUMPTION: MODER[10] == MODERL[12] == 0
		gpio->MODER  ^= 0x1 << 10 | 0x1 << 12;							// Change the mode selection bits 0b11, 0b11 of PD5, PD6 to 0b10, 0b10 .

	}
	else{
		// NOTE: We do not plan to need more than 2 (LP)U(S)ARTs!
		while(TRUE){
			__NOP();
		}
	}
}

// Set up GPIO pin PD7 to listen for a button press
// ASSUMPTION: rcc_enable_gpiod() was already called
void gpio_init_button(){
	GPIO_TypeDef * gpio = GPIOD;

	// (1) Set pin mode to input (MODE7 == 0b00)
	CLEAR_BIT(gpio->MODER, GPIO_MODER_MODE7);

	// (2) Activate pull-down resistor (PUPD7 == 0b10)
	SET_BIT(gpio->PUPDR, GPIO_PUPDR_PUPD7_1);
}

void gpio_init_port_as_output(GPIO_TypeDef * gpio){

	// (1) Set pin mode to input (MODE7 == 0b00)
	uint32_t bits_to_clear = GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1 |
			GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1 |
			GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1;

	CLEAR_BIT(gpio->MODER, bits_to_clear);

	// (2) Activate pull-down resistor (PUPD7 == 0b10)
//	SET_BIT(gpio->PUPDR,);
}


// - - - - - - - - - - - - - - -
// DEBUGGING
// - - - - - - - - - - - - - - -

void debug_gpio_set_g7_output(){
	GPIOG->MODER ^= 0x01 << 15;										// Set mode register bits associated with GPIO G7 to 0b01 (rather than the default 0b11)
}


// - - - - - - - - - - - - - - -
// USART
// - - - - - - - - - - - - - - -
void usart_init_cr1(USART_TypeDef * uart){
	uart->CR1 = 0x0;												// Initialize to no interrupts, no nothing
//	uart->CR1 = 0b111 << 5;											// Sets TXEIE, TCIE, RXNEIE to 1 (enables them) in CR1
}

void usart_init_cr2(USART_TypeDef * uart){
	// So far, it doesn't seem as if we need to change CR2 as we don't use any of the features it controls.
}

void usart_init_cr3(USART_TypeDef * uart){
	uart->CR3 = 0b01;												// Sets EIE to 1, which means errors create an interrupt
}

void usart_init_baudrate(USART_TypeDef * uart, uint32_t baudrate){

	if (uart == LPUART1){
		uint64_t usartdiv;
		usartdiv = 256 * PCLK1_FREQ;							// For LPUART, page 1658-1659 of reference manual state how to set usartdiv.
																// Note that lpuart_kec_ck = lpuart_pclk and that we have divider register set to 0 (i.e. no divide),
																// which means lpuart_ckpres = lpuart_pclk = PCLK1_FREQ.
																// ==> To get Baud rate 115200, we need to set BRR (i.e. LPUARTDIV) such that 256 * PCLK1_FREQ / LPUARTDIV = 115200 (or close to it)
																// Equality holds <==> LPUARTDIV = 256 * PCLK1_FREQ / 115200.
																// NOTE: Computation would be OVERFLOW if we didn't cast PCLK1_FREQ in its definition to be uint64_t!
		usartdiv = usartdiv / baudrate;
		uart->BRR = USART_BRR_LPUART & usartdiv;				// LPUART is 20 bits, so just to be super formal = ANDded it with USART_BRR_LPUART, which is 0xFFFFF, i.e. 20 LSB set to 1

	}
	else{
		uint16_t usartdiv;
		usartdiv = PCLK1_FREQ / baudrate;						// 110 * 10^6 / 115200 = 954.8611 = 954 per integer division
		uart->BRR = USART_BRR_BRR & usartdiv;										// usartdiv is already a 16 bit value and the 16 LSB of BRR are meant to hold the usartdiv value. Nevertheless, I add it with USART_BRR_BRR == 0xFFFF just to be super formal

	}

}

void usart_enable(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_UE;							// Set UE bit to 1 by ORing it with 0x01. This sets is to 1 if it wasn't 1 already
}

void usart_enable_receive(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_RE;							// Set UE bit to 1 by ORing it with 0x01. This sets is to 1 if it wasn't 1 already
}

void usart_enable_transmit(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_TE;							// Set UE bit to 1 by ORing it with 0x01. This sets is to 1 if it wasn't 1 already
}

void usart_set_TXEIE(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_TXEIE;						// Set TXEIE bit to 1 ==> get interrupted upon TXE flag being set (TDR being empty)
}

void usart_set_RXNEIE(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_RXNEIE;						// Set RXNEIE bit to 1 ==> get interrupted upon RXNE flag being set (RDR being not empty)
}


// - - - - - - - - - - - - - - -
// RING BUFFER
// - - - - - - - - - - - - - - -

void rb_init(ringbuf_t * rb, char init_val){
	  rb->cons_ind	= 0;
	  rb->prod_ind  = 0;
	  rb->empty		= TRUE;
	  rb->isr_ctr   = 0;
	  memset(rb->buf, init_val, sizeof(rb->buf));
}

bool_t rb_consume_one(ringbuf_t* rb, byte_t* obyte){
	int16_t prod_ind 	 = rb->prod_ind;
	int16_t cons_ind	 = rb->cons_ind;
	int16_t buffsize 	 =	RBUF_LEN;



	if(prod_ind == cons_ind){			// When buffer is empty, i.e.
		return FALSE;
	}

	*obyte = rb->buf[cons_ind];

	int16_t new_cons_ind = mod(cons_ind + 1, buffsize);			// NOTE: cons_ind trails one behind, i.e. when cons_ind == i, then i has already been read!
	rb->cons_ind		 = new_cons_ind;

	return TRUE;

}

int16_t rb_consume_all(ringbuf_t* rb, byte_t* obuf){
	int16_t prod_ind 	 = rb->prod_ind;
	int16_t cons_ind	 = rb->cons_ind;
	int16_t buffsize 	 =	RBUF_LEN;

	int16_t num_readable = mod(prod_ind - cons_ind, buffsize);
	rb_consume(rb, obuf, num_readable);

	return num_readable;
}


bool_t rb_consume(ringbuf_t* rb, byte_t * obuf, int16_t num_items){
	int16_t prod_ind	= rb->prod_ind;
	int16_t cons_ind 	= rb->cons_ind;
	int16_t buffsize 	= RBUF_LEN;
	byte_t*  rbuf      	= rb->buf;

	int16_t num_readable = mod(prod_ind - cons_ind, buffsize);

	if(num_items > num_readable){			// Case: Consume pointer would overtake produce pointer
		return FALSE;
	}

	for(int i = 0; i < num_items; ++i){
		obuf[i] = rbuf[mod(cons_ind + i, buffsize)];
	}
	rb->cons_ind = mod(cons_ind + num_items, buffsize);							// NOTE: We have to do it like this because having "rb->cons_ind = mod(cons_ind + i, buffsize);" in the loop undercounts by one (the i = 0 case adds 0 to rb->cons_ind)
	return TRUE;
}

bool_t rb_produce_one(ringbuf_t* rb, byte_t ibyte){

	return rb_produce(rb, &ibyte, 1);
//	int16_t prod_ind = rb->prod_ind;
//	int16_t cons_ind = rb->cons_ind;
//	int16_t buffsize 	=	RBUF_LEN;
//
//	if(mod(prod_ind + 1 - cons_ind, buffsize) == 0){										// Buffer of n elements is filled with n-1 elements. We don't allow the n-th element to be populated
//																				// as we would otherwise get into the ambiguous case where prod_ind == cons_ind, which we DEFINE to mean empty (and allowing n-th element to be placed would make the full buffer appear as if it was empty)
//		return FALSE;
//	}
//
//	rb->buf[prod_ind] = ibyte;
//	rb->prod_ind = mod(prod_ind + 1, buffsize);
//
//	return TRUE;
}

bool_t rb_produce(ringbuf_t* rb, byte_t* ibuf, int16_t num_items){
	int16_t prod_ind 	= rb->prod_ind;
	int16_t cons_ind 	= rb->cons_ind;
	int16_t buffsize 	= RBUF_LEN;
	byte_t*  rbuf      	= rb->buf;


	if(mod(prod_ind + num_items - cons_ind, buffsize) == 0 || num_items >= buffsize){
		return FALSE;
	}


	for(int i = 0; i < num_items; ++i){
		rbuf[mod(prod_ind + i, buffsize)] = ibuf[i];				// We take local copy of cons_ind for efficiency ==> add + i
	}

	rb->prod_ind = mod(prod_ind + num_items, buffsize);					// Update volatile, global cons_ind AFTER consuming to prevent buffer value from being overwritten

	return TRUE;

}

// - - - - - - - - - - - - - - -
// RNG
// - - - - - - - - - - - - - - -

void rng_wait_ready(){
	while(!READ_BIT(RNG->SR, RNG_SR_DRDY)){
		__NOP();
	}
}

void rng_init(){

	// (1) Clock configuration

	// Switch on HSI48 48MHz oscillator
	RCC->CRRCR |= RCC_CRRCR_HSI48ON;

	// Wait until it is stable by polling on HSI48RDY bit of RCC->CRRCR (Sec. 9.8.31, page 404/22194 of reference manual RM0438)
	while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY_Msk)){
		__NOP();
	}

	// Enable RNG peripheral (wrt. RCC ==> Give it "juice")
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;

	// 48 MHz clock source selector set to HSI48 (the above only enabled HSI48, we still need to set a downstream MUX to have the HSI48 delivered to RNG
	RCC->CCIPR1 = RCC->CCIPR1^ (RCC->CCIPR1 & RCC_CCIPR1_CLK48MSEL);						// Idea: We XOR the bits 27:26 (responsible for setting CLK48MSEL) with itself, guarantees that they are 0 afterwards. All the other bits of (RCC->CCIPR1 & RCC_CCIPR1_CLK48MSEL) are 0 ==> No other bits affected

	// (2) RNG configuration

	// Disable RNG. This is "just in case" as it should be disabled at start
	CLEAR_BIT(RNG->CR, RNG_CR_RNGEN);

	// Build our configuration values:
	uint32_t config_vals = RNG_CR_CED | RNG_CR_CONDRST;

	// Set our config values
	SET_BIT(RNG->CR, config_vals);

	// Set self check values
	WRITE_REG(RNG->HTCR, RNG_HTCFG_1);
	WRITE_REG(RNG->HTCR, RNG_HTCFG);

	// Finishing the setup by clearing the conditional soft reset bit again (like a transaction: Setting it to 1 started the transaction, now we end it)
	CLEAR_BIT(RNG->CR, RNG_CR_CONDRST);

	// Wait until the conditional configuration has completed:
	while(READ_BIT(RNG->CR, RNG_CR_CONDRST)){					// NOTE: Bit set to 1 means reset not complete ==> Wait until bit is 0
		__NOP();
	}

	// Enable RNG peripheral
	SET_BIT(RNG->CR, RNG_CR_RNGEN);

	// Check for seed error. This is done by checking the Seed error INTERRUPT flag, which is set even if interrupts are disabled (if I understood correctly)
	if(READ_BIT(RNG->SR, RNG_SR_SEIS)){
		while(TRUE){
			__NOP();
		}
	}

	// Wait until the first batch of 4 x 32 bytes of randomness are ready
	while(!READ_BIT(RNG->SR, RNG_SR_DRDY)){
		__NOP();
	}
}

// obuf[0], obuf[1], ..., obuf[num_samples - 1] get populated with random bytes
// NOTE: We do not check for any kind of errors in the randomness sampling!
void rng_get_randomness(byte_t* o_buf, uint32_t num_samples){
	uint32_t rand_val;

	int remainder = (int) (num_samples % 4);
	int32_t full_block_indices = num_samples - remainder;

	for(int i = 0; i < full_block_indices; i += 4){
		rng_wait_ready();
		rand_val = READ_REG(RNG->DR);
//		if(rand_val == 0){										// Check whether randomness generation has failed. As stated in the reference manual, one sign of problem is an all-0 output (also incredibly unlikely!)
//			while(TRUE){										// This check is BAD, as it can by chance happen that we get all-0, which would lock up the system!
//				__NOP();
//			}
//		}
		memcpy(o_buf + i, &rand_val, 4);
	}

	// If message buffer is not multiple of 4 bytes in size, we handle the last <= bytes separately
	if(remainder > 0){
		rng_wait_ready();
		rand_val = READ_REG(RNG->DR);
		memcpy(o_buf + full_block_indices, &rand_val, 4);
	}

}

// - - - - - - - - - - - - - - -
// HASH
// - - - - - - - - - - - - - - -


void hash_init(){
	SET_BIT(HASH->CR, HASH_CR_INIT);
}

// Waits until DIN and the FIFO behind it are empty ==> Can load a new block
void hash_wait_input_empty(){
	while(!READ_BIT(HASH->SR, HASH_SR_DINIS)){
		__NOP();
	}
}

// Waits until DIN and the FIFO behind it are empty ==> Can load a new block
void hash_wait_finished(){
	while(!READ_BIT(HASH->SR, HASH_SR_DCIS)){
		__NOP();
	}
}

// Extracted almost one-to-one from the HAL library! The first line seems sketchy but it works in the HAL library...
// NOTE: Casting a pointer to an int certainly doesn't work on all (e.g. 64-bit) platforms!
void hash_extract_digest(byte_t* o_buf){
	uint32_t msgdigest = (uint32_t)o_buf;

	*(uint32_t*)(msgdigest) = __REV(HASH->HR[0]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH->HR[1]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH->HR[2]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH->HR[3]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH->HR[4]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH_DIGEST->HR[5]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH_DIGEST->HR[6]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH_DIGEST->HR[7]);
}

void hash_startup(){
	// (1) Enable clock to it

	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_HASHEN);

	// Added those two lines because HAL also has it, I think it's to delay the execution until we can be sure the bit has been written accordingly
    uint32_t tmpreg = READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_HASHEN);
    UNUSED(tmpreg);

	// (2) Set up HASH peripheral

    // Set all values to reset
    CLEAR_REG(HASH->CR);

    // Set datatype to bytes ==> handles endianness issue
    SET_BIT(HASH->CR, HASH_CR_DATATYPE_1);

    // Set HASH mode to HMAC ==> Knows how to handle key, etc.
    SET_BIT(HASH->CR, HASH_CR_MODE);

    // Set hashing algorithm to have control bits set to 11 ==> SHA256
    SET_BIT(HASH->CR, HASH_CR_ALGO_1 | HASH_CR_ALGO_0);

    // Toggle INIT bit ==> Reset the peripheral's input buffer AND select the hashing that has been set in the command above.
    hash_init();
}

// Updates the hash state with data of length AT MOST ONE BLOCK (512 bits == 64 bytes).
void hash_update(byte_t *block_buf, uint32_t len_bytes){

	// Check whether input is actually at most 64 bytes (1 block) in length
	if(len_bytes > 64){
		while(TRUE){
			__NOP();
		}
	}

	uint32_t interm_buf;															// I THINK this can prevent misaligned memory accesses
	for(int i = 0; i < len_bytes; i += 4){

		// Copy buffer bytewise into a 4-byte aligned buffer
		memcpy(&interm_buf, block_buf + i, 4);

		// Write content of 4-byte aligned buffer into HASH's DIN register
		WRITE_REG(HASH->DIN, interm_buf);

		// XXX: It seems as if we don't need to wait as we are inserting
		//      data of at most one block in size and the hash peripheral can handle that
	}
}

// Digests a WHOLE message, i.e. also some of length > 1 block!
// Does NOT call init.
void hash_prepare_digest(byte_t* msg_buf, uint32_t len_bytes){
	uint8_t suffix_len = 8 * (len_bytes % 4);										// HASH peripheral works on 32 bit words
																					// ==> If we have e.g. 24 bits of data, the upper 8 bits would be 0
	MODIFY_REG(HASH->STR, HASH_STR_NBLW_Msk, suffix_len);

	for(int i = 0; i < len_bytes; i += 64){
		// Load new block into HASH peripheral
		hash_update(msg_buf + (i * 64), min(len_bytes - (i * 64), 64));				// We call the hash_update function with either a full block (64 bytes) or with length suffix_len

		// Wait until the HASH peripheral can take in a new BLOCK of data
//		hash_wait_input_empty();
	}
}

void hash_hmac(byte_t* key_buf, byte_t* msg_buf, uint32_t len_key, uint32_t len_msg, byte_t* o_buf){
	// Step 1: Initialize peripheral
	hash_init();																	// Resets the data FIFO within HASH peripheral, BUT KEEPS THE CONFIGS otherwise

	// Step 2.0: Insert inner key (sets NBLW internally aswell)
	hash_prepare_digest(key_buf, len_key);

	// Step 2.1: Start digest
	SET_BIT(HASH->STR, HASH_STR_DCAL);

	// Step 2.2: Wait until input register and fifo are empty again
	hash_wait_input_empty();

	// Step 3.0: Insert message (sets NBLW internally aswell)
	hash_prepare_digest(msg_buf, len_msg);

	// Step 3.1: Start digest
	SET_BIT(HASH->STR, HASH_STR_DCAL);

	// Step 3.2: Wait until input register and fifo are empty again
	hash_wait_input_empty();

	// Step 4.0: Insert outer key (sets NBLW internally aswell)
	hash_prepare_digest(key_buf, len_key);

	// Step 4.1: Start digest
	SET_BIT(HASH->STR, HASH_STR_DCAL);

	// Step 4.2: Wait until HASH computation is done, s.t. we can get the output
	hash_wait_finished();

	hash_extract_digest(o_buf);

}

// - - - - - - - - - - - - - - -
// Timer
// - - - - - - - - - - - - - - -


// Sets up timer t. NOTE: presc is supposed to be one less than the desired prescaler. I.e. presc = 0 means clock taken directly, presc = 1 means clock halved, presc = 2 means clock 1/3 speed, ...
void timer_init(TIM_TypeDef* t, uint16_t presc){


//	// Clear CR1 (reset it)
//	CLEAR_REG(t->CR1);
//
//	// Clear CR2 (reset it)
//	CLEAR_REG(t->CR2);


	// Enable timer's clock input
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_TIM7EN);

	// Set prescaler
	WRITE_REG(t->PSC, presc);

	// Set repetition counter to 0
	CLEAR_REG(t->RCR);

// Do NOT set one-shot mode! It is not compatible with how we do the timer resetting in case an access needs to be extended in time!
//	SET_BIT(t->CR1, TIM_CR1_OPM);

}

// Starts a timer which goes off after timeout. NOTE that this depends on the prescaler! E.g. we use 10999, i.e. effectively clock divided by 11000. Hence, at 110MHz, this means the timer is incremented every 100us.
// E.g. for timout == 10, the timer overflows and causes update event interrupt after 10 * 100us = 1ms.
void timer_start(TIM_TypeDef* t, uint16_t timeout){

	if(t != TIM7){
		while(TRUE){
			__NOP();
		}
	}

	//	NOTE: If this bit flag is set, timer TIM7 stops when a breakpoint is reached
	SET_BIT(DBGMCU->APB1FZR1, DBGMCU_APB1FZR1_DBG_TIM7_STOP);

	// Check if counter is currenty operating. This is an ERROR because we should NEVER restart a counter before it is finished!
	if(READ_BIT(t->CR1, TIM_CR1_CEN)){
		while(TRUE){
			__NOP();
		}
	}

	// Clear timer count register (16 least significant bits)
	CLEAR_BIT(t->CNT, 0xFFFF);

	// Set auto-reload register (value after which the counter overruns)
	// TODO: Find out whether it should actually be "timeout_100us - 1" because the datasheet says it counts "from 0 to the auto-reload value" in section 33.3.2, page 1080
	WRITE_REG(t->ARR, timeout);

	// NOTE: Prescaler Register is ALWAYS buffered ==> Need to commit the values into the prescaler shadow register by triggering an update event manually!
	SET_BIT(t->EGR, TIM_EGR_UG);
	CLEAR_BIT(t->SR, TIM_SR_UIF);

	//Start timer
	SET_BIT(t->CR1, TIM_CR1_CEN);


	// Set update interrupt to be enabled
	SET_BIT(t->DIER, TIM_DIER_UIE);

	// Enable global interrupt for that timer
	// Currently, only have timer TIM7!
	NVIC_EnableIRQ(TIM7_IRQn);

}

// - - - - - - - - - - - - - - -
// Server Message building
// - - - - - - - - - - - - - - -

bool_t s_msg_build_header(byte_t msg_type, s_msg_t* o_s_msg){
	o_s_msg->msg_buf[0] = msg_type;

	if(msg_type == SERVER_PAYLOAD_SIGNUP_REQ){
			// TODO: Check if this line is correct
			uint8_t dev_cap_len = strlen(dev_state_s.dev_cap);
			o_s_msg->msg_buf[1] = LEN_SERVER_PAYLOAD_SIGNUP_REQ + dev_cap_len;			// NOTE: We do NOT include the 0-byte string-terminator of the device capabilities URI!
			o_s_msg->msg_buf[2] = 0;

			o_s_msg->msg_len = HEADER_LEN + LEN_SERVER_PAYLOAD_SIGNUP_REQ + dev_cap_len;
			return TRUE;
		}
	else if(msg_type == SERVER_PAYLOAD_AUTH_REQ){
		// TODO: Check if this line is correct
		o_s_msg->msg_buf[1] = LEN_SERVER_PAYLOAD_AUTH_REQ;																							// NOTE: uint16_t has 2 bytes, but our messages (so far) have lengths that fit into one byte ==> Just write to that one byte
		o_s_msg->msg_buf[2] = 0;

		o_s_msg->msg_len = HEADER_LEN + LEN_SERVER_PAYLOAD_AUTH_REQ;

		return TRUE;
	}
	else if(msg_type == SERVER_PAYLOAD_CONTROL){
		// Not implemented
		while(TRUE){
			__NOP();
		}
	}

	return FALSE;

}

// Builds a challenge message given the challenge stored in the request state
// NOTE: Difference because we are in the TrustZone implementation. The outbound struct is now a gw_msg_t and no more a send_msg_t
bool_t s_msg_build(byte_t msg_type, req_t* req_state, s_msg_t* o_s_msg){
	/**
	 * Recall message format:
	 * | 3 bytes header | payload |
	 */

	// Check for error in header creation ==> Sign that passed msg_type is incorrect
	if(!s_msg_build_header(msg_type, o_s_msg)){
		while(TRUE){
			__NOP();
		}
	}

	byte_t* o_payload = o_s_msg->msg_buf + 3;

	switch(msg_type){
	case SERVER_PAYLOAD_SIGNUP_REQ:
		memcpy(o_payload, &dev_state_s.dev_type, LEN_SERVER_PAYLOAD_SIGNUP_REQ);		// NOTE: Here, LEN_SERVER_PAYLOAD_SIGNUP_REQ
																						// is actually a lower bound for the payload length,
																						// which is the length of the device type (2 bytes)
		memcpy(o_payload + 2, dev_state_s.dev_cap, strlen(dev_state_s.dev_cap));
		return TRUE;

	case SERVER_PAYLOAD_AUTH_REQ:
		memcpy(o_payload, &dev_state_s.dev_id, DEVICE_ID_LEN);
		memcpy(o_payload + DEVICE_ID_LEN, req_state->challenge, CHALLENGE_LEN);
		return TRUE;

	case SERVER_PAYLOAD_CONTROL:														// NOTE: Case not yet implemented!
		while(TRUE){
			__NOP();
		}
	default:
		return FALSE;
	}

}

// NOTE: Due to TrustZone implementation, gw_msg_t, unlike send_msg_t, does NOT have a header field!
// XXX: WARNING, I noticed that we had a case for GATEWAY_PAYLOAD_RESPONSE, which we should NEVER get! I removed it,
//		consider that if gw_msg_build behaves weirdly!
bool_t gw_msg_build(byte_t msg_type, req_t* req_state, gw_msg_t* o_gw_msg){

	// Set payload type. If that type is illegal, it won't be used anyways because gw_msg_build then returns FALSE.
	o_gw_msg->payload_type = msg_type;

	// Because we don't include the header, o_payload is just the pointer to the beginning of the o_gw_msg payload buffer (instead of that + 3)
	byte_t* o_payload = o_gw_msg->payload_buf;

	// Switch in message types
	switch(msg_type){
	case GATEWAY_PAYLOAD_REQUEST:
		memcpy(o_payload, &req_state->access_type, LEN_GATEWAY_PAYLOAD_REQUEST);
		return TRUE;

	case GATEWAY_PAYLOAD_CONTROL:														// NOTE: Case not yet implemented!
		while(TRUE){
			__NOP();
		}
	default:
		// Case: Unexpected message type
		return FALSE;
	}

}

// - - - - - - - - - - - - - - - - - - - - - - - - - -
// New functions: Specific to TrustZone implementation
// - - - - - - - - - - - - - - - - - - - - - - - - - -

void s_build_auth_req(req_t* req_state, s_msg_t* o_s_msg){
	o_s_msg->msg_len = HEADER_LEN + LEN_SERVER_PAYLOAD_AUTH_REQ;
	memcpy(o_s_msg->msg_buf, &dev_state_s.dev_id, DEVICE_ID_LEN);
	memcpy(o_s_msg->msg_buf + DEVICE_ID_LEN, req_state->challenge, CHALLENGE_LEN);
}

bool_t set_req_state(uint16_t access_type){


	// (1) Check if access type is within range
	if (access_type < 0 || access_type > ACCESS_TYPE_CONTROL_ACTUATOR_1){															// Received malformed access type ==> Ignore request
		__NOP();																											// NOP to break on for debugging purposes
		return FALSE;
	}

	// (2) Set access type
	req_state.access_type = access_type;

	// (3) Build challenge message. Recall, challenge message is: |  reb_cnt  |  req_cnt  |  access_type  |  mac_tag  |
	//     CHANGE: To ensure freshness, we also include the randomness from the last message by the server. This randomness is however not explicitly sent to the server.
	//     We use the randomness from the server's previous authentication response.
	//     This means that the mac_tag is actually computed over: | reb_cnt | req_cnt | access_type | randomness |.


	byte_t hmac_input_buf[REB_CNT_LEN + REQ_CNT_LEN + ACCESS_TYPE_LEN + RANDOM_LEN];

	memcpy(hmac_input_buf, &cntrs_struct.reb_cnt, REB_CNT_LEN);
	memcpy(hmac_input_buf + REB_CNT_LEN, &cntrs_struct.req_cnt, REQ_CNT_LEN);
	memcpy(hmac_input_buf + REB_CNT_LEN + REQ_CNT_LEN, &access_type, ACCESS_TYPE_LEN);
	// CHANGE: Copy randomness from request state
	memcpy(hmac_input_buf + REB_CNT_LEN + REQ_CNT_LEN + ACCESS_TYPE_LEN, req_state.randomness, RANDOM_LEN);

	uint32_t mac_tag_offset = REB_CNT_LEN + REQ_CNT_LEN + ACCESS_TYPE_LEN;
	req_state.tag_ptr = req_state.challenge + mac_tag_offset;

	#ifdef PROFILING_REQUEST_TRUSTZONE
		   // 4: Finished setting up HMAC input/output buffer
		   PROFILING_SET_COUNTER_E(4);
	#endif

	// (3.1) Compute MAC-Tag over |  reb_cnt  |  req_cnt  |  access_type  |
	hash_hmac(keys_struct.key_gw_s, hmac_input_buf, HMAC_KEY_LEN, sizeof(hmac_input_buf), req_state.challenge + mac_tag_offset);

	#ifdef PROFILING_REQUEST_TRUSTZONE
		   // 5: Finished HMAC computation
		   PROFILING_SET_COUNTER_E(5);
	#endif

	// (3.2) CHANGE: Copy the challenge into req_state.challenge at the position before the MAC tag (i.e. before mac_tag_offset)
	//       Before the change, we didn't have an hmac_input_buf, the challenge values were already copied into the req_state.challenge buffer one by one.
	memcpy(req_state.challenge, hmac_input_buf, mac_tag_offset);



	// TODO: Figure out if we should add some randomness to prevent message from being used in potentially another protocol (cryptographic hygiene).
	//		 Gene suggested that the server should add some randomness to its authentication response, so it'd make sense also for the gateway to do it.

	// (4) Increment request counter value
	cntrs_struct.req_cnt++;

	// (5) Set receive time
	req_state.timer_set_time = HAL_GetTick();

	// (6) Set new request state to true (i.e. valid)
	req_state.valid = TRUE;

	return TRUE;
}


bool_t handle_access_request(uint16_t access_type, byte_t* o_chal_buf){
	// TODO:  Handle new request based on global secure world state

	// (1) Check if we are currently granting a request

	#ifdef PROFILING_REQUEST_TRUSTZONE
		 // 2: Started processing request
		   PROFILING_SET_COUNTER_E(2);
	#endif

	if(flag_currently_granting){
		// Case: Currently granting another request
		// (1.1) Check if currently granted access is the same access type
		if(access_type != req_state.access_type){
			// Case: NOT the same access type ==> Reject it
			return FALSE;
		}
	}


	// If we reached here, one of two cases has occurred:
	//       Case 1: Currently granting, new request has same access type ==> Nonsecure world is pipelining requests (allowed)
	//		 Case 2: Currently not granting

	// (2) Create challenge
	// (2.1) Copy counters into challenge

	#ifdef PROFILING_REQUEST_TRUSTZONE
		 // 3: Checked if we are currently granting
		   PROFILING_SET_COUNTER_E(3);
	#endif


	bool_t success = set_req_state(access_type);

	#ifdef PROFILING_REQUEST_TRUSTZONE
		 // 6: Finished setting new request state
		   PROFILING_SET_COUNTER_E(6);
	#endif

	if(success){

		memcpy(o_chal_buf, req_state.challenge, LEN_GATEWAY_PAYLOAD_CHALLENGE);			// Copy challenge request state to output struct's payload buffer
		#ifdef PROFILING_REQUEST_TRUSTZONE
			 // 7: Copy challenge back to buffer
			   PROFILING_SET_COUNTER_E(7);
		#endif
		return TRUE;
	}
	return FALSE;
}


err_t check_response_validity(gw_msg_t* i_gw_msg){

	// (1) Check if the current request is valid, i.e. a valid candidate for being granted
	if(!req_state.valid){
		return -1;
	}

	// (2) Check if request has timed out yet
	if(HAL_GetTick() - req_state.timer_set_time > req_state.timeout){
		req_state.valid = FALSE;
		return -2;
	}


	#ifdef PROFILING_RESPONSE_TRUSTZONE
		// 3: Finished performing preliminary checks
		PROFILING_SET_COUNTER_E(3);
	#endif

	// (3) Verify message
	// (3.1) Generate message buffer with same format the server used, i.e.: |  randomness  |  request_mac_tag  |
	byte_t hmac_input_buf[RANDOM_LEN + HMAC_OUTPUT_SIZE];
	memcpy(hmac_input_buf, i_gw_msg->payload_buf, RANDOM_LEN);
	memcpy(hmac_input_buf + RANDOM_LEN, req_state.tag_ptr, HMAC_OUTPUT_SIZE);

	// (3.2) Compute reference tag over that buffer
	byte_t reference_tag[HMAC_OUTPUT_SIZE];

	#ifdef PROFILING_RESPONSE_TRUSTZONE
		// 4: Finished preparing HMAC in/output buffers
		PROFILING_SET_COUNTER_E(4);
	#endif

	hash_hmac(keys_struct.key_s_gw, hmac_input_buf, HMAC_KEY_LEN, RANDOM_LEN + HMAC_OUTPUT_SIZE, reference_tag);

	#ifdef PROFILING_RESPONSE_TRUSTZONE
		// 5: Finished computing HMAC
		PROFILING_SET_COUNTER_E(5);
	#endif

	// (3.3) Extract tag from server's response message
	byte_t* received_tag = i_gw_msg->payload_buf + RANDOM_LEN;

	// (3.4) Compare locally computed reference HMAC tag with that extracted from the payload. This is NOT constant time! If the match, we return 0 (no error), else -3 (HMAC verifiaction error).
	int difference = memcmp(reference_tag, received_tag, HMAC_OUTPUT_SIZE);
	if(difference == 0){


	#ifdef PROFILING_RESPONSE_TRUSTZONE
		// 6: Finished comparing reference tag with received MAC tag
		PROFILING_SET_COUNTER_E(6);
	#endif
		// (3.5) CHANGE: Copy randomness into the request state, it will be used in HMAC computation of the next request by the GW
		memcpy(req_state.randomness, i_gw_msg->payload_buf, RANDOM_LEN);


		return 0;
	}
	else{
		// Case: Bad tag
		return -3;
	}
}


bool_t handle_server_msg(void* void_i_gw_msg){

	#ifdef PROFILING_RESPONSE_TRUSTZONE
		// 2: Started processing authentication message
		PROFILING_SET_COUNTER_E(2);
	#endif

	gw_msg_t* i_gw_msg = (gw_msg_t*) void_i_gw_msg;
	// Switch based on payload type
	switch (i_gw_msg->payload_type) {
		case GATEWAY_PAYLOAD_RESPONSE:
			// (1) Check response for general validity (fresh, have a current request, ...)
			if(0 == check_response_validity(i_gw_msg)){

				#ifdef PROFILING_RESPONSE_TRUSTZONE
					// 7: Finished copying randomness from message
					PROFILING_SET_COUNTER_E(7);
				#endif
				// If we reach here, we received a valid response to the current request.
				// ==> Start granting
				req_state.valid = FALSE;

				// We have two cases:
				// NOTE: To check in which case we are, we have to disable the the update event generation. Hence, we beforehand have to check if we should REALLY reset/start the counter, as we could otherwise extend an access we should not!
				// (1) Currently already granting. By the checks performed in check_response_validity, we know that this response is to the same request type as is currently being granted AND that the request has a valid MAC-Tag and counter value
				//	   ==> Simply reset the timer to 0.
				// (2) Currently not granting. Then, we just set the timer, set flags "flag_type_currently_granting" and "flag_currently_granting" and then start the timer.

				bool_t equal_access_type = (req_state.access_type == flag_type_currently_granting);

				if(flag_currently_granting && !equal_access_type){
					// We should in principle NEVER reach here, this means that we granted a request despite currently processing another request. Because we do NOT do any pipelining, this is an illegal state.
					__NOP();
					return FALSE;
				}

				// If we reach here we can be 100% sure that we are currently not granting OR we are granting the same request type for which we just got a valid response ==> Can start (resp. reset count register of) timer!

				// Disable update event generation
				TIMER_SET_UDIS(TIM7);

				// Check if we are STILL granting (NOTE: an update event could have occurred between our preliminary check above and now)
				// Now, because we have the UDIS flag set, we can be 100% sure that the flag_currently_granting will not change because the interrupt won't be triggered due to the UDIS flag being set!

				if(flag_currently_granting){
					// If we reach here, we know that the timer is currently running and that (by the preliminary check above) we are currently serving the same access type.
					// Hence, we are in case (1). We have two subcases:
					// (1.1) We reset the UDIS flag before the timer runs out (i.e. before the access should be revoked)
					//       Normal case, the timer count register will just be reset
					// (1.2) A timer update event WOULD have been raised, but it was suppressed (already within the timer device ==> never reached CPU core) because the UDIS flag was set.
					//		 Hence, we actually give the requesting CPU slightly more time than allocated, namely the time between when the update event _would_ have been generated and us resetting the count register.
					//		 This time is likely in the order of single-digit microseconds!
					TIMER_RESET_CNT(TIM7);
				}else{
					// If we reach here, this is because the timer wasn't running in the first place
					flag_currently_granting = TRUE;
					flag_type_currently_granting = req_state.access_type;
					timer_start(TIM7, 5000 * TIMEOUT_10);								// Time is given in milliseconds (e.g. 5000), but one tick of the timer is 100us ==> multiply with 10 to get from 5000 * 100us to 5000 * 1000us = 5000ms

				}

				// Re-Enable update event generation
				TIMER_CLEAR_UDIS(TIM7);

				#ifdef PROFILING_RESPONSE_TRUSTZONE
					// 8: Finished
					PROFILING_SET_COUNTER_E(8);
				#endif

				return TRUE;
			}

			// If we reached here, the response was bad ==> exit switch case and return FALSE
			break;
		default:
			// Case: Not (yet) implemented or generally bad payload type
			break;
	}

	return FALSE;

}

// - - - - - - - - - - - - - - -
// Generic helper functions
// - - - - - - - - - - - - - - -


int min(int a, int b){
	return (a < b) ? a : b;
}

int16_t mod(int16_t x, int16_t m){
	int16_t y = x % m;

	return (y >= 0) ? y : (m + y);									// If y < 0, then we need to compute m - |y| which is equivalent to m + y!
}
