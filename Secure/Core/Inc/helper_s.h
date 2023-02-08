/*
 * helper.h
 *
 *  Created on: Jun 27, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_HELPER_S_H_
#define INC_HELPER_S_H_

#include "main.h"

void rcc_enable_gpiog();
void rcc_enable_gpioc();
void rcc_enable_gpioe();
void rcc_enable_gpiof();

void rcc_enable_lpuart();
void rcc_enable_usart2();
void rcc_set_vddio2();

void gpio_init_uart(USART_TypeDef * uart);
void gpio_init_button();
void gpio_init_port_as_output(GPIO_TypeDef * gpio);

void usart_init_cr1(USART_TypeDef * uart);
void usart_init_cr2(USART_TypeDef * uart);
void usart_init_cr3(USART_TypeDef * uart);
void usart_init_baudrate(USART_TypeDef * uart, uint32_t baudrate);

void usart_enable(USART_TypeDef * uart);
void usart_enable_receive(USART_TypeDef * uart);
void usart_enable_transmit(USART_TypeDef * uart);
void usart_set_TXEIE(USART_TypeDef * uart);
void usart_set_RXNEIE(USART_TypeDef * uart);

void   rb_init(ringbuf_t * rb, char init_val);
bool_t rb_consume_one(ringbuf_t* rb, byte_t* obyte);
bool_t rb_produce_one(ringbuf_t* rb, byte_t ibyte);

int16_t rb_consume_all(ringbuf_t* rb, byte_t* obuf);

bool_t rb_consume(ringbuf_t* rb, byte_t * obuf, int16_t num_items);
bool_t rb_produce(ringbuf_t* rb, byte_t* ibuf, int16_t num_items);

void hash_startup();

void timer_init(TIM_TypeDef* t, uint16_t presc);
void timer_start(TIM_TypeDef* t, uint16_t timeout);

// DIFFERENCE: s_msg_build takes a gateway message and turns it into the corresponding server message
bool_t s_msg_build(byte_t msg_type, req_t* req_state, s_msg_t* o_s_msg);
bool_t gw_msg_build(byte_t msg_type, req_t* req_state, gw_msg_t* o_gw_msg);

// NEW: Handlers
bool_t handle_access_request(uint16_t access_type, byte_t* o_chal_buf);
bool_t handle_server_msg(void* void_i_gw_msg);


bool_t process_bytes();

// Generic helper functions
int min(int a, int b);
int16_t mod(int16_t x, int16_t m);


#endif /* INC_HELPER_S_H_ */
