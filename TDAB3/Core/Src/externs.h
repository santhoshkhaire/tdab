/*
 * externs.h
 *
 *  Created on: 21-Sep-2020
 *      Author: Santhosh.Khaire
 */

#ifndef EXTERNS_H_
#define EXTERNS_H_

#include "typedef.h"

extern int pc_data_in;
extern int rx_data_in;
extern unsigned char rx_data;
extern unsigned char pc_data;
extern unsigned char UART_RxBuff[UART_MSG_SIZE];
extern unsigned char PC_RxBuff[UART_MSG_SIZE];
extern unsigned char UART_Rx_byte_count;
extern unsigned char UART_Pc_byte_count;
extern unsigned char DataAvailable;
extern unsigned char DataLen;
extern unsigned char ac_inputs[MAX_AC_INPUTS];
extern uint16_t timer_count;
extern uint16_t ticks;
extern uint16_t timer_tick;
extern uint16_t uut_timer_tick;
extern unsigned char start_timer;
extern unsigned char uut_timer;
extern uint16_t pulse_counter[MAX_AC_INPUTS];
extern uint32_t adc_buff[NUM_ADC_CHANNELS];
extern uint16_t adc_flag;
extern uint16_t pulse_acc;
extern uint32_t HV_Input1[10000];
extern uint8_t start_capture;
extern uint32_t dma_buff[2];
extern uint16_t HV_input_state, HV_input_prev_state;
extern uint16_t pulse_acc;
extern uint32_t timer7_count;
extern uint16_t pulse_frequency;
extern uint16_t start_time;
extern double timetaken;
extern uint16_t start_falltime_counter;
extern uint16_t falltime_counter;
extern uint16_t fall_time;
extern uint16_t start_risetime_counter;
extern uint16_t risetime_counter;
extern uint16_t rise_time;
#endif /* EXTERNS_H_ */
