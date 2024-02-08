/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "typedef.h"
#include "externs.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Added test line to test github

int count = 0;
// ADC Global Variables
float scale_factor_150V = 2.4936335;		// set the scale factor after testing with 150 V
float scale_factor_24V = 0.4106;		//0.413 For TDAB 2.0
//float scale_factor_24V1 = 0.398;	// 0.36621
float scale_factor_5V = 0.0865;		// 0.07629
float scale_factor_3V3 = 0.057246; 		// 0.0576; TDAB2.0
uint16_t adc_flag = 0;
uint32_t adc_buff[NUM_ADC_CHANNELS];
uint32_t dma_buff[2];
uint32_t raw_adc_buff[NUM_ADC_CHANNELS];
uint32_t temp_adc_buff[NUM_ADC_CHANNELS];
uint32_t adc_val[NUM_ADC_CHANNELS];
uint32_t adc3_buff[NUM_ADC_CHANNELS];
uint16_t calculate_adc_voltage = 0;
float adc_volt[NUM_ADC_CHANNELS];
uint16_t adc_data[NUM_ADC_CHANNELS];
uint16_t adc3_data[NUM_ADC_CHANNELS];
uint16_t uut_ana[UUT_NUM_ADC_CHANNELS];
uint16_t uut_dig_in;
uint16_t uut_eeprom_status = EEPROM_TEST_IN_PROGRESS;
uint16_t test_num = 0;
uint16_t hall_null_offset = 600;
uint16_t Analog_output[3] = {0,0,0};
uint8_t uut_ack = 0;
uint8_t junk_byte = 0;
uint32_t HV_Input1[10000];
uint8_t start_capture = 0;
uint16_t Supply_Current = 0;
uint16_t HV1_channel_voltage = 0;
uint16_t HV1_channel_adc_counts = 0;
uint16_t HV1_channel_voltage_capture[1000];
//Serial Global Variables
int pc_data_in = 0;
int pc_data_out = 0;
int rx_data_in = 0;
int rx_data_out = 0;
unsigned char pc_data = 0;
unsigned char rx_data = 0;
unsigned char DataAvailable = 0;
unsigned char DataLen = UART_MSG_SIZE;
unsigned char UART_Pc_byte_count = 0;
unsigned char UART_Rx_byte_count = 0;
unsigned char PC_RxBuff[UART_MSG_SIZE];
unsigned char PC_TxBuff[UART_MSG_SIZE];
unsigned char UART_TxBuff[UART_MSG_SIZE];
unsigned char UART_RxBuff[UART_MSG_SIZE];

uint8_t spi_data[4];
unsigned char once = 0;
uint16_t ticks = 0;
uint16_t tempticks = 0;
uint16_t timer_tick = 0;
uint16_t uut_timer_tick = 0;
unsigned char start_timer = 0;
unsigned char uut_timer = 0;
unsigned char send_uut_command = 0;
// Digital Inputs
unsigned char dig_inputs[MAX_DIGITAL_INPUTS];
uint16_t Digital_Input;
unsigned char v33_inputs[MAX_ANALOG_INPUTS];
uint16_t V33_Input;
uint16_t V33_Output;
uint16_t V33_Diff_Output;
uint16_t Relay_Output;
uint16_t V24_Source_Output;
uint16_t V24_Sinking_Output;
uint16_t UUT_Power;
unsigned char sw_inputs[MAX_SWITCH_INPUTS];
uint16_t Switch_Inputs;
unsigned char ac_inputs[MAX_AC_INPUTS];
unsigned char acinputs[MAX_AC_INPUTS];
uint16_t AC_Input;
uint8_t dig_input_arr[6];
uint16_t timer_count = 0;
uint16_t pulse_counter[MAX_AC_INPUTS];
uint16_t pulse_acc = 0;	//Added for counting PZD pulses in Timed mode for Pico Nexus EOL Tester
uint16_t pulse_counter_reset = 0;
uint16_t pulse_frequency = 0;
uint32_t timer7_count = 0;
uint16_t HV_input_state = 0, HV_input_prev_state = 0;
uint16_t start_time = 0;
double timetaken = 0;
uint16_t start_falltime_counter = 0;
uint16_t falltime_counter = 0;
uint16_t start_risetime_counter = 0;
uint16_t risetime_counter = 0;
uint16_t fall_time = 0;
uint16_t rise_time = 0;
uint16_t Measure_fall_time = 0;
uint16_t Measure_rise_time = 0;
uint16_t HV1_sample = 0;
uint32_t HV1_readings[2000], HV1_acc = 0;
double temp_HV1_voltage = 0;
float adcbuff_14 = 0;
uint16_t voltage_capture_count = 0;
uint16_t adc_data14_prev = 0, adc_data14_curr = 0;

uint16_t tdab_sw_ver = TDAB_SOFTWARE_VERSION;
uint16_t tdab_hw_ver = TDAB_HARDWARE_VERSION;
uint16_t uut_sw_ver = 0;
uint16_t uut_hw_ver  = 0;
uint16_t clear_data = 0;

uint8_t spiTxData[2] = {0,0};
uint8_t spiRxData[5] = {0,0,0,0,0};
uint8_t spiRx1Data[5] = {0,0,0,0,0};
uint8_t spiUltiTx[5] = {0x55,0x55,0x55,0x55,0x55};
uint8_t spiUltiRx[5] = {0,0,0,0,0};
uint16_t ultimus_temp = 0;
uint8_t strreg[5] = {0,0,0,0,0};
uint16_t pulse_count = 0;
uint16_t PCP_pulse_count_read = 0;

uint16_t read_U13counts = 0;
uint16_t read_U15counts = 0;
uint16_t U13counts = 0;
uint16_t U15counts = 0;

// Pico Valve Board Variables
uint16_t pico_sw_version = 0;
uint16_t pico_hw_version = 0;
uint16_t green_led_flag = 0;
uint16_t red_led_flag = 0;

uint16_t stack_rtd_offset = 0;
uint16_t stack_rtd_counts = 0;
uint16_t stack_rtd_30_deg = 0;
uint16_t stack_rtd_80_deg = 0;
uint16_t stack_rtd_scale_factor = 0;

uint16_t heater_rtd_offset = 0;
uint16_t heater_rtd_counts = 0;
uint16_t heater_rtd_30_deg = 0;
uint16_t heater_rtd_80_deg = 0;
uint16_t heater_rtd_scale_factor = 0;

uint16_t heater_rtd_high = 0;
uint16_t heater_rtd_low = 0;

uint16_t hall_counts = 0;
uint16_t hall_vin_020 = 0;
uint16_t hall_vin_060 = 0;
uint16_t hall_offset = 0;
uint16_t hall_gain = 0;
uint16_t vin_000 = 0;	// 0 Volt
uint16_t vin_020 = 200;	// 0.20 Volt (scale to 1000)
uint16_t vin_060 = 600; // 0.60 Volt (scale t0 1000)

uint16_t heater_flag = 0;
uint16_t pico_board_supply_amps = 0;
uint16_t pico_board_supply_power = 0;

uint16_t calib_coeff1_high = 0;
uint16_t calib_coeff1_low = 0;
uint16_t calib_coeff2_high = 0;
uint16_t calib_coeff2_low = 0;
uint16_t calib_coeff3_high = 0;
uint16_t calib_coeff3_low = 0;
uint16_t calib_coeff_write = 0;
uint16_t calibration_status = 0;



#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  32)   /* Size of array aADCxConvertedData[] */
//ALIGN_32BYTES (static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);
uint32_t  aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];


unsigned char pc_protocol_state = SYNC_STATE;
unsigned char uut_protocol_state = UUT_SYNC_STATE;
RECORD record;
MODBUS_REG_MAP mod_reg_map[] = {
		// Software and Hardware versions for TDAB and UUT
		{100, &tdab_sw_ver, NULL},
		{101, &tdab_hw_ver, NULL},
		{102, &uut_sw_ver, change_uut_state},
		{103, &uut_hw_ver, change_uut_state},
		{104, &clear_data, clear_uut_data},

		// Digital Inputs
		{1000, &AC_Input, NULL},
		{1001, &V33_Input, NULL},
		{1002, &Digital_Input, NULL},

		// Digital Outputs
		{2000, &V33_Output, set_clear_v33_output},
		{2001, &V33_Diff_Output, set_clear_v33diff_output},
		{2002, &Relay_Output, set_clear_relay_output},
		{2003, &V24_Source_Output, set_clear_v24_source_output},
		{2004, &V24_Sinking_Output, set_clear_v24_sink_output},
		{2005, &UUT_Power, set_clear_uut_power},

		// Analog Inputs
		{3000, &adc_data[0], NULL},
		{3001, &adc_data[1], NULL},
		{3002, &adc_data[2], NULL},
		{3003, &adc_data[3], NULL},
		{3004, &adc_data[4], NULL},
		{3005, &adc_data[5], NULL},
		{3006, &adc_data[6], NULL},
		{3007, &adc_data[7], NULL},
		{3008, &adc_data[8], NULL},
		{3009, &adc_data[9], NULL},
		{3010, &adc_data[10], NULL},
		{3011, &adc_data[11], NULL},
		{3012, &Supply_Current, NULL},	//Added calculation of supply current in main() and passing the same in response to MODBUS request for 3012.
		{3013, &adc_data[13], NULL},
		{3014, &HV1_channel_voltage, NULL},	//Changed from adc_data[14] to HV1_channel_voltage (Averaged value after 50 samples)
		{3015, &HV1_channel_adc_counts, NULL}, //Was adc_data[15]

		{4000,&Analog_output[0],update_analog_output},
		{4001,&Analog_output[1],update_analog_output1},

		{5000, &test_num,   change_uut_state},
		{5001, &uut_ana[0], change_uut_state},
		{5002, &uut_ana[1], change_uut_state},
		{5003, &uut_ana[2], change_uut_state},
		{5004, &uut_ana[3], change_uut_state},
		{5005, &uut_ana[4], change_uut_state},
		{5006, &uut_ana[5], change_uut_state},
		{5007, &uut_ana[6], change_uut_state},
		{5008, &uut_ana[7], change_uut_state},
		{5009, &uut_ana[8], change_uut_state},
		{5010, &uut_ana[9], change_uut_state},
		{5011, &uut_ana[10], change_uut_state},
		{5012, &uut_ana[11], change_uut_state},
		{5013, &uut_dig_in, change_uut_state},
		{5014, &uut_eeprom_status, change_uut_state},

		//Added for PCP board
		{5020, &pulse_count, PCP_pulse_counter_test},
		{5021, &PCP_pulse_count_read, NULL},

		//Added for Ultimus I and II board
		{5030, &read_U13counts, Ultimus_Pressure_Sensor1_Test},
		{5031, &U13counts, NULL},
		{5032, &read_U15counts, Ultimus_Pressure_Sensor2_Test},
		{5033, &U15counts, NULL},

		//Added for Pico Valve Board
		{5040, &pico_sw_version, change_uut_state},
		{5041, &pico_hw_version, change_uut_state},
		{5042, &green_led_flag, change_uut_state},
		{5043, &red_led_flag, change_uut_state},

		{5044, &stack_rtd_counts, change_uut_state},
		{5045, &stack_rtd_offset, change_uut_state},
		{5046, &stack_rtd_30_deg, change_uut_state},
		{5047, &stack_rtd_80_deg, change_uut_state},
		{5048, &stack_rtd_scale_factor, change_uut_state},

		{5049, &heater_rtd_counts, change_uut_state},
		//{5050, &heater_rtd_offset, change_uut_state},
		//{5051, &heater_rtd_30_deg, change_uut_state},
		//{5052, &heater_rtd_80_deg, change_uut_state},
		//{5053, &heater_rtd_scale_factor, change_uut_state},

		{5054, &hall_counts, change_uut_state},
		{5055, &hall_offset, change_uut_state},
		{5056, &hall_vin_020, change_uut_state},
		{5057, &hall_vin_060, change_uut_state},
		{5058, &hall_gain, change_uut_state},


		{5059, &heater_flag, change_uut_state},
		{5060, &pico_board_supply_amps, NULL},
		{5061, &pico_board_supply_power, NULL},

		{5080, &calibration_status, change_uut_state},

		{6000, &heater_rtd_high, change_uut_state},
		{6001, &heater_rtd_low, change_uut_state},
		{6002, &calib_coeff1_high, change_uut_state},
		{6003, &calib_coeff1_low, change_uut_state},
		{6004, &calib_coeff2_high, change_uut_state},
		{6005, &calib_coeff2_low, change_uut_state},
		{6006, &calib_coeff3_high, change_uut_state},
		{6007, &calib_coeff3_low, change_uut_state},

		// Added for Pico Nexus EOL tester
		{7000, &pulse_counter_reset, clear_pulse_counter},
		{7001, &pulse_frequency, NULL},
		{7002, &Measure_fall_time, start_fall_time_test},
		{7003, &fall_time, NULL},		//Fall time in uS
		{7004, &Measure_rise_time, start_rise_time_test},
		{7005, &rise_time, NULL},		//Rise time in uS
		// Illegal entry - end of map
		{0,NULL, NULL}
};


const GPIO_INTERFACE relay_map[]  = {
		{GPIOE, GPIO_PIN_2},
		{GPIOE, GPIO_PIN_3},
		{GPIOE, GPIO_PIN_4},
		{GPIOE, GPIO_PIN_5},
		{GPIOE, GPIO_PIN_6},
		{GPIOC, GPIO_PIN_13},
		{GPIOC, GPIO_PIN_14},
		{GPIOC, GPIO_PIN_15},
		{GPIOF, GPIO_PIN_14},
		{GPIOF, GPIO_PIN_15},
		{GPIOG, GPIO_PIN_0},
		{GPIOG, GPIO_PIN_1}
};


const GPIO_INTERFACE v33_dig_out_map[] = {

		// 12 3V3 Digital Output ON
		{GPIOD, GPIO_PIN_14},
		{GPIOD, GPIO_PIN_15},
		{GPIOG, GPIO_PIN_2},
		{GPIOG, GPIO_PIN_3},
		{GPIOG, GPIO_PIN_4},
		{GPIOG, GPIO_PIN_5},
		{GPIOG, GPIO_PIN_6},
		{GPIOG, GPIO_PIN_7},
		{GPIOG, GPIO_PIN_8},
		{GPIOC, GPIO_PIN_6},
		{GPIOC, GPIO_PIN_7},
		{GPIOC, GPIO_PIN_8}
};


const GPIO_INTERFACE v33_diff_out_map[] = {

		// Differential Outputs ON
		{GPIOB, GPIO_PIN_2},
		{GPIOF, GPIO_PIN_11},
		{GPIOF, GPIO_PIN_12},
		{GPIOF, GPIO_PIN_13}
};

const GPIO_INTERFACE v24_source_out_map[] = {

		// 8 sourcing 24V Digital Outputs ON
		{GPIOD, GPIO_PIN_3},
		{GPIOD, GPIO_PIN_4},
		{GPIOG, GPIO_PIN_9},
		{GPIOG, GPIO_PIN_10},
		{GPIOG, GPIO_PIN_11},
		{GPIOG, GPIO_PIN_12},
		{GPIOG, GPIO_PIN_13},
		{GPIOG, GPIO_PIN_14}
};

const GPIO_INTERFACE v24_sink_out_map[] = {
		// Digital Driver Outputs ON
		{GPIOE, GPIO_PIN_7},
		{GPIOE, GPIO_PIN_8},
		{GPIOE, GPIO_PIN_9},
		{GPIOE, GPIO_PIN_10},
		{GPIOE, GPIO_PIN_15},
		{GPIOB, GPIO_PIN_10},
		{GPIOC, GPIO_PIN_9},
		{GPIOA, GPIO_PIN_8}
};




/* Table of CRC values for high–order byte */
const unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;

/* Table of CRC values for low–order byte */
const unsigned char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Initialize(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */
//static void CPU_CACHE_Enable(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1)
{
  /* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
  //SCB_InvalidateDCache_by_Addr((uint32_t *) &adc_buff[0], 14);
  SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCxConvertedData[0], ADC_CONVERTED_DATA_BUFFER_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
   /* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
  //SCB_InvalidateDCache_by_Addr((uint32_t *) &adc_buff[1], 14);
	SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE/2], ADC_CONVERTED_DATA_BUFFER_SIZE);
}


void PCModbusRTU(void)
{
	int i;
	uint16_t temp = 0;
	static uint16_t cal_crc = 0;
	static uint16_t crc_count = 0;
	static uint8_t num_bytes = 0;

	switch(pc_protocol_state) {

		case SYNC_STATE:

			pc_data_in = 0;
			pc_data_out = 0;
			UART_Pc_byte_count = 0;
			memset(PC_RxBuff, 0, UART_MSG_SIZE);
			memset(PC_TxBuff, 0, UART_MSG_SIZE);
			memset(record.data, 0, UART_MSG_SIZE);
			pc_protocol_state = RECEIVE_STATE;
			start_timer = 0;

			break;


		case RECEIVE_STATE:

				if(PC_RxBuff[0] == SLAVE_ADDRESS) {
					switch(PC_RxBuff[1]) {

					case READ_HOLDING_REGISTERS:
						start_timer = 1;
						if(timer_tick >= 500)	// 5 SECS
							pc_protocol_state = SYNC_STATE;

						if(UART_Pc_byte_count >= 8) {
							cal_crc = CRC16(PC_RxBuff,(6));
							temp = PC_RxBuff[7];
							temp = ((temp << 8) | PC_RxBuff[6]);

							if(cal_crc == temp) {
								pc_protocol_state = PROCESS_STATE;
								pc_data_out = 1;
							}
							else
								pc_protocol_state = SYNC_STATE;

							start_timer = 0;
						}

						break;

					case WRITE_MULTIPLE_REGISTERS:
						start_timer = 1;
						if(timer_tick >= 500)	// 5 SECS
							pc_protocol_state = SYNC_STATE;

						num_bytes = PC_RxBuff[6];

						if(num_bytes) {

							if(UART_Pc_byte_count >= (num_bytes + 9)) {
								start_timer = 0;
								timer_tick = 0;

								cal_crc = CRC16(PC_RxBuff,((num_bytes + 9) - 2));
								temp = PC_RxBuff[(num_bytes + 9) - 1];
								temp = ((temp << 8) | PC_RxBuff[(num_bytes + 9) - 2]);

								if(cal_crc == temp) {
									pc_protocol_state = PROCESS_STATE;
									pc_data_out = 1;
								}
								else {
									pc_protocol_state = SYNC_STATE;
								}

							}
						}

						break;

					case READ_WRITE_MULTIPLE_REGISTERS:

						num_bytes = PC_RxBuff[10];

						if(num_bytes) {

							if(UART_Pc_byte_count >= (num_bytes + 12)) {

								cal_crc = CRC16(PC_RxBuff,(UART_Pc_byte_count - 2));
								temp = PC_RxBuff[UART_Pc_byte_count - 1];
								temp = ((temp << 8) | PC_RxBuff[UART_Pc_byte_count - 2]);

								if(cal_crc == temp) {
									pc_protocol_state = PROCESS_STATE;
									pc_data_out = 1;
								}
								else {
									pc_protocol_state = SYNC_STATE;
								}
							}
						}

						break;

					default:
						start_timer = 1;
						if(timer_tick >= 500)	// 5 SECS
							pc_protocol_state = SYNC_STATE;
						break;

					}
				}
				else if(UART_Pc_byte_count) {
					start_timer = 1;
					if(timer_tick >= 500)	// 5 SECS
						pc_protocol_state = SYNC_STATE;
				}

			break;


			case PROCESS_STATE:
				switch(PC_RxBuff[pc_data_out]) {

					case READ_HOLDING_REGISTERS:
						pc_data_out++;
						record.function_code = READ_HOLDING_REGISTERS;
						temp = (uint16_t)(PC_RxBuff[pc_data_out] << 8);
						pc_data_out++;
						record.read_start_address = (uint16_t) (temp | PC_RxBuff[pc_data_out]);
						pc_data_out++;

						temp = (uint16_t)(PC_RxBuff[pc_data_out] << 8);
						pc_data_out++;
						record.read_registers = (uint16_t) (temp | PC_RxBuff[pc_data_out]);
						pc_data_out++;

						record.read_byte_count = (record.read_registers * 2);

						ProcessPCCommands();
						pc_protocol_state = TRANSMIT_STATE;

						break;


					case WRITE_MULTIPLE_REGISTERS:
						pc_data_out++;

						record.function_code = WRITE_MULTIPLE_REGISTERS;
						temp = (uint16_t)PC_RxBuff[pc_data_out] << 8;
						pc_data_out++;
						record.write_start_address = (uint16_t) (temp | PC_RxBuff[pc_data_out]);
						pc_data_out++;

						temp = (uint16_t)PC_RxBuff[pc_data_out] << 8;
						pc_data_out++;
						record.write_registers = (uint16_t) (temp | PC_RxBuff[pc_data_out]);
						pc_data_out++;

						record.write_byte_count = PC_RxBuff[pc_data_out];
						pc_data_out++;
						for(i=0; i<record.write_byte_count; i++)
							record.data[i] = PC_RxBuff[pc_data_out++];

						//uut_protocol_state = UUT_SYNC_STATE;
						ProcessPCCommands();
						pc_protocol_state = TRANSMIT_STATE;

						break;


					case READ_WRITE_MULTIPLE_REGISTERS:

						pc_data_out++;
						record.function_code = READ_WRITE_MULTIPLE_REGISTERS;
						temp = (uint16_t)(PC_RxBuff[pc_data_out] << 8);
						pc_data_out++;
						record.read_start_address = (uint16_t) (temp | PC_RxBuff[pc_data_out]);
						pc_data_out++;

						temp = (uint16_t)(PC_RxBuff[pc_data_out] << 8);
						pc_data_out++;
						record.read_registers = (uint16_t) (temp | PC_RxBuff[pc_data_out]);
						pc_data_out++;

						record.read_byte_count = (record.read_registers * 2);

						temp = (uint16_t)PC_RxBuff[pc_data_out] << 8;
						pc_data_out++;
						record.write_start_address = (uint16_t) (temp | PC_RxBuff[pc_data_out]);
						pc_data_out++;

						temp = (uint16_t)PC_RxBuff[pc_data_out] << 8;
						pc_data_out++;
						record.write_registers = (uint16_t) (temp | PC_RxBuff[pc_data_out]);
						pc_data_out++;

						record.write_byte_count = PC_RxBuff[pc_data_out];
						pc_data_out++;
						for(i=0; i<record.write_byte_count; i++)
							record.data[i] = PC_RxBuff[pc_data_out++];

						pc_protocol_state = TRANSMIT_STATE;


						break;

				}
				break;


			case TRANSMIT_STATE:

				//if(uut_ack) {
					switch(record.function_code) {

						case READ_HOLDING_REGISTERS:

							PC_TxBuff[0] = SLAVE_ADDRESS;
							PC_TxBuff[1] = record.function_code;
							PC_TxBuff[2] = record.read_byte_count;

							FillTransmitBuffer(record.read_registers, 3);
							SerialTransmit();
							pc_protocol_state = SYNC_STATE;

							break;


						case WRITE_MULTIPLE_REGISTERS:

							PC_TxBuff[0] = SLAVE_ADDRESS;
							PC_TxBuff[1] = record.function_code;

							PC_TxBuff[2] = (uint8_t)((record.write_start_address & 0xFF00) >> 8);
							PC_TxBuff[3] = (uint8_t)(record.write_start_address & 0xFF);

							PC_TxBuff[4] = (uint8_t)((record.write_registers & 0xFF00) >> 8);
							PC_TxBuff[5] = (uint8_t)(record.write_registers & 0xFF);

							crc_count = 6;
							cal_crc = CRC16(PC_TxBuff,crc_count);

							// CRC Low Byte first and then High Byte next
							PC_TxBuff[6] = (uint8_t) (cal_crc & 0x00FF);
							PC_TxBuff[7] = (uint8_t) ((cal_crc & 0xFF00) >> 8);
							pc_data_out = crc_count + 2;

							SerialTransmit();
							pc_protocol_state = SYNC_STATE;

							break;


						case READ_WRITE_MULTIPLE_REGISTERS:

							PC_TxBuff[0] = SLAVE_ADDRESS;
							PC_TxBuff[1] = record.function_code;
							PC_TxBuff[2] = record.read_byte_count;
							FillTransmitBuffer(record.read_registers, 3);

							SerialTransmit();
							pc_protocol_state = SYNC_STATE;

							break;

					}
				//}
				break;


			default:
				break;
	}

}


void UUTModbusRTU(void)
{


	int i,j;
	unsigned char index = 0;
	uint16_t cal_crc = 0;
	uint16_t crc_count = 0;
	uint16_t temp = 0;
	uint16_t temp_data = 0;
	float ftemp = 0;
	unsigned char num_bytes = 0;

	switch(uut_protocol_state) {

		case UUT_SYNC_STATE:

			rx_data_in = 0;
			rx_data_out = 0;
			UART_Rx_byte_count = 0;
			memset(UART_RxBuff, 0, UART_MSG_SIZE);
			memset(UART_TxBuff, 0, UART_MSG_SIZE);
			memset(record.uut_data, 0, UART_MSG_SIZE);
			uut_protocol_state = UUT_TRANSMIT_STATE;
			uut_timer = 0;

			break;


		case UUT_TRANSMIT_STATE:


			if(send_uut_command) {

				send_uut_command = 0;
				switch(record.function_code) {

					case READ_HOLDING_REGISTERS:
						UART_TxBuff[0] = SLAVE_ADDRESS;
						UART_TxBuff[1] = record.function_code;

						UART_TxBuff[2] = (uint8_t)((record.read_start_address & 0xFF00) >> 8);
						UART_TxBuff[3] = (uint8_t)(record.read_start_address & 0xFF);

						UART_TxBuff[4] = (uint8_t)((record.read_registers & 0xFF00) >> 8);
						UART_TxBuff[5] = (uint8_t)(record.read_registers & 0xFF);

						crc_count = 6;
						cal_crc = CRC16(UART_TxBuff,crc_count);

						index = crc_count;

						// CRC Low Byte first and then High Byte next
						UART_TxBuff[index++] = (uint8_t) (cal_crc & 0x00FF);
						UART_TxBuff[index++] = (uint8_t) ((cal_crc & 0xFF00) >> 8);
						rx_data_out = index;

						uut_ack = 0;
						UUTSerialTransmit();
						uut_protocol_state = UUT_RECEIVE_STATE;
						break;


					case WRITE_MULTIPLE_REGISTERS:
						UART_TxBuff[0] = SLAVE_ADDRESS;
						UART_TxBuff[1] = record.function_code;

						UART_TxBuff[2] = (uint8_t)((record.write_start_address & 0xFF00) >> 8);
						UART_TxBuff[3] = (uint8_t)(record.write_start_address & 0xFF);

						UART_TxBuff[4] = (uint8_t)((record.write_registers & 0xFF00) >> 8);
						UART_TxBuff[5] = (uint8_t)(record.write_registers & 0xFF);

						UART_TxBuff[6] = (uint8_t)(record.write_byte_count);
						index = 7;

						for(i=0; i<record.write_byte_count; i++) {
							UART_TxBuff[index++] = record.data[i];
						}

						crc_count = index;
						cal_crc = CRC16(UART_TxBuff,crc_count);

						// CRC Low Byte first and then High Byte next
						UART_TxBuff[index++] = (uint8_t) (cal_crc & 0x00FF);
						UART_TxBuff[index++] = (uint8_t) ((cal_crc & 0xFF00) >> 8);
						rx_data_out = index;

						uut_ack = 0;
						UUTSerialTransmit();
						uut_protocol_state = UUT_RECEIVE_STATE;

						// Reset the pulse counter for AC Inputs
						//if((record.data[0] == 0x00) && (record.data[1] == 0x04)) {
						//if(record.data[1] == AC_OUTPUTS) {
						if((record.data[1] == AC_OUTPUTS) && (record.write_start_address == SPECIAL_ADDRESS)) {
							for(i=0; i<5; i++) {
								pulse_counter[i] = 0x00;
								acinputs[i] = 0x00;
							}
							AC_Input = 0x00;
						}

						//if(record.data[1] == EEPROM_TEST)
						//	uut_eeprom_status = EEPROM_TEST_IN_PROGRESS;
						break;


					default:
						break;
				}
			}
			break;

		case UUT_RECEIVE_STATE:

			if(UART_Rx_byte_count) {
				uut_timer = 1;
				if(uut_timer_tick >= 500) {

					uut_protocol_state = UUT_SYNC_STATE;
				}
			}

			if((UART_RxBuff[0] == SLAVE_ADDRESS) && ((UART_RxBuff[1] == READ_HOLDING_REGISTERS) || (UART_RxBuff[1] == WRITE_MULTIPLE_REGISTERS)))
					junk_byte = 0;
			else if((UART_RxBuff[1] == SLAVE_ADDRESS) && ((UART_RxBuff[2] == READ_HOLDING_REGISTERS) || (UART_RxBuff[2] == WRITE_MULTIPLE_REGISTERS)))
					junk_byte = 1;


			if(UART_RxBuff[1 + junk_byte] == READ_HOLDING_REGISTERS)
				num_bytes = (UART_RxBuff[2 + junk_byte] + 5 + junk_byte);
			else
				num_bytes = (8 + junk_byte);

			if(num_bytes) {

				if(UART_Rx_byte_count >= num_bytes) {

					uut_timer = 0;

					if(junk_byte)
						cal_crc = CRC16(&UART_RxBuff[1],(num_bytes - (2 + junk_byte)));
					else
						cal_crc = CRC16(UART_RxBuff,(num_bytes - (2 + junk_byte)));

					temp = UART_RxBuff[num_bytes - 1];
					temp = ((temp << 8) | UART_RxBuff[num_bytes - 2]);

					if(cal_crc == temp)
						uut_protocol_state = UUT_PROCESS_STATE;
					else
						uut_protocol_state = UUT_SYNC_STATE;
				}
			}

			break;


		case UUT_PROCESS_STATE:

			switch(UART_RxBuff[1 + junk_byte]) {

				case READ_HOLDING_REGISTERS:

					for(i=0,j=(3+junk_byte); i<UART_RxBuff[2 + junk_byte]; i++,j++)	{
						record.uut_data[i] = UART_RxBuff[j];
					}


					if((record.read_start_address >= UUT_RTD_START_ADDRESS) && (record.read_start_address <= UUT_RTD_END_ADDRESS)) {

						temp = record.read_start_address - UUT_RTD_START_ADDRESS;

						for(i=0,j=0; i<record.read_registers; i++) {
							temp_data = record.uut_data[j++];
							uut_ana[temp++] = ((temp_data << 8) | record.uut_data[j++]);
						}

					}

					switch(record.read_start_address) {

						case UUT_EEPROM_READ_ADDRESS:
							temp = record.uut_data[0];
							uut_eeprom_status = ((temp << 8) | record.uut_data[1]);
							break;

						case UUT_DIG_INPUT_ADDRESS:
							temp = record.uut_data[0];
							uut_dig_in = ((temp << 8) | record.uut_data[1]);
							break;

						case UUT_SOFTWARE_VER_ADDRESS:
							temp = record.uut_data[0];
							uut_sw_ver = ((temp << 8) | record.uut_data[1]);

							break;

						case UUT_HARDWARE_VER_ADDRESS:
							temp = record.uut_data[0];
							uut_hw_ver = ((temp << 8) | record.uut_data[1]);

							break;


						case PICO_SOFTWARE_VERSION:
							temp = record.uut_data[0];
							pico_sw_version = ((temp << 8) | record.uut_data[1]);

							break;

						case PICO_HARDWARE_VERSION:
							temp = record.uut_data[0];
							pico_hw_version = ((temp << 8) | record.uut_data[1]);

							break;


						case STACK_RTD_COUNTS:
							temp = record.uut_data[0];
							stack_rtd_counts = ((temp << 8) | record.uut_data[1]);

						case STACK_RTD_OFFSET:
							temp = record.uut_data[0];
							stack_rtd_offset = ((temp << 8) | record.uut_data[1]);
							break;


						case STACK_RTD_30_DEG:
							temp = record.uut_data[0];
							stack_rtd_30_deg = ((temp << 8) | record.uut_data[1]);
							break;

						case STACK_RTD_80_DEG:
							temp = record.uut_data[0];
							stack_rtd_80_deg = ((temp << 8) | record.uut_data[1]);
							break;

						case STACK_RTD_SCALE_FACTOR:
							stack_rtd_scale_factor = (uint16_t) (stack_rtd_80_deg - stack_rtd_30_deg);
							break;

						case HEATER_RTD_COUNTS:
							temp = record.uut_data[0];
							heater_rtd_counts = ((temp << 8) | record.uut_data[1]);
							break;

						case HEATER_RTD_CALIB_HIGH:
							temp = record.uut_data[0];
							heater_rtd_high = ((temp << 8) | record.uut_data[1]);
							temp = record.uut_data[2];
							heater_rtd_low = ((temp << 8) | record.uut_data[3]);
							break;

						case CALIB_COEFF1_VAL_HIGH:
							switch(record.read_registers) {
							case 2:
								temp = record.uut_data[0];
								calib_coeff1_high = ((temp << 8) | record.uut_data[1]);
								temp = record.uut_data[2];
								calib_coeff1_low = ((temp << 8) | record.uut_data[3]);
								break;

							case 4:
								temp = record.uut_data[0];
								calib_coeff1_high = ((temp << 8) | record.uut_data[1]);
								temp = record.uut_data[2];
								calib_coeff1_low = ((temp << 8) | record.uut_data[3]);

								temp = record.uut_data[4];
								calib_coeff2_high = ((temp << 8) | record.uut_data[5]);
								temp = record.uut_data[6];
								calib_coeff2_low = ((temp << 8) | record.uut_data[7]);
								break;

							case 6:
								temp = record.uut_data[0];
								calib_coeff1_high = ((temp << 8) | record.uut_data[1]);
								temp = record.uut_data[2];
								calib_coeff1_low = ((temp << 8) | record.uut_data[3]);

								temp = record.uut_data[4];
								calib_coeff2_high = ((temp << 8) | record.uut_data[5]);
								temp = record.uut_data[6];
								calib_coeff2_low = ((temp << 8) | record.uut_data[7]);

								temp = record.uut_data[8];
								calib_coeff3_high = ((temp << 8) | record.uut_data[9]);
								temp = record.uut_data[10];
								calib_coeff3_low = ((temp << 8) | record.uut_data[11]);
								break;
							}

						case CALIB_COEFF2_VAL_HIGH:
							switch(record.read_registers) {

							case 2:
								temp = record.uut_data[0];
								calib_coeff2_high = ((temp << 8) | record.uut_data[1]);
								temp = record.uut_data[2];
								calib_coeff2_low = ((temp << 8) | record.uut_data[3]);
								break;

							case 4:
								temp = record.uut_data[0];
								calib_coeff2_high = ((temp << 8) | record.uut_data[1]);
								temp = record.uut_data[2];
								calib_coeff2_low = ((temp << 8) | record.uut_data[3]);

								temp = record.uut_data[4];
								calib_coeff3_high = ((temp << 8) | record.uut_data[5]);
								temp = record.uut_data[6];
								calib_coeff3_low = ((temp << 8) | record.uut_data[7]);
								break;

							}
							break;

						case CALIB_COEFF3_VAL_HIGH:
							temp = record.uut_data[0];
							calib_coeff3_high = ((temp << 8) | record.uut_data[1]);
							temp = record.uut_data[2];
							calib_coeff3_low = ((temp << 8) | record.uut_data[3]);
							break;


						case CALIBRATION_STATUS:
							temp = record.uut_data[0];
							calibration_status = ((temp << 8) | record.uut_data[1]);
							break;

						case HALL_COUNTS:
							temp = record.uut_data[0];
							hall_counts = ((temp << 8) | record.uut_data[1]);
							break;

						case HALL_OFFSET:
							temp = record.uut_data[0];
							hall_offset = ((temp << 8) | record.uut_data[1]);
							hall_gain = 0;
							break;

						case HALL_VIN_020:
							temp = record.uut_data[0];
							hall_vin_020 = ((temp << 8) | record.uut_data[1]);
							ftemp = (float)((float)hall_vin_020 / (float)vin_020);
							hall_gain = (uint16_t)(ftemp * 1000);
							break;

						case HALL_VIN_060:
							temp = record.uut_data[0];
							hall_vin_060 = ((temp << 8) | record.uut_data[1]);
							ftemp = (float)((float)hall_vin_060 / (float)vin_060);
							hall_gain = (uint16_t) (ftemp * 1000);
							break;


						case BOARD_SUPPLY_AMPS:
							temp = record.uut_data[0];
							pico_board_supply_amps = ((temp << 8) | record.uut_data[1]);
							break;

						case BOARD_SUPPLY_POWER:
							temp = record.uut_data[0];
							pico_board_supply_power = ((temp << 8) | record.uut_data[1]);
							break;


						default:
							break;

					}
					uut_ack = 1;
					uut_protocol_state = UUT_SYNC_STATE;

					break;


				case WRITE_MULTIPLE_REGISTERS:

					switch(record.write_start_address) {

						case SPECIAL_ADDRESS:
							for(i=0; i<5; i++) {
								pulse_counter[i] = 0x00;
								acinputs[i] = 0x00;
							}
							AC_Input = 0x00;

							break;


						default:
							break;

					}
					uut_ack = 1;
					uut_protocol_state = UUT_SYNC_STATE;

					break;


				default:
					uut_protocol_state = UUT_SYNC_STATE;
					break;
			}
			break;

		default:
			break;


	}

}


void SerialTransmit(void)
{

	HAL_UART_Transmit(&huart1, PC_TxBuff, pc_data_out, 5000);
	HAL_Delay(5);

}

void UUTSerialTransmit(void) {

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);		// transmit mode
	HAL_Delay(1);
	HAL_UART_Transmit(&huart2, UART_TxBuff, rx_data_out, 5000);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);	// receive mode
	HAL_Delay(1);
}

void ReadInputs(void)
{

	int i = 0;
	// Digital Inputs
	dig_inputs[i++] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
	dig_inputs[i++] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
	dig_inputs[i++] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
	dig_inputs[i++] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	dig_inputs[i++] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
	dig_inputs[i++] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	dig_inputs[i++] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	dig_inputs[i++] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15);

	Digital_Input = 0;
	Digital_Input = (Digital_Input | dig_inputs[7]);
	for(i=6; i>=0; i--) {

		Digital_Input = ((Digital_Input << 1) | dig_inputs[i]);
	}


	// Analog Inputs
	i = 0;
	v33_inputs[i++] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
	v33_inputs[i++] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);
	v33_inputs[i++] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2);
	v33_inputs[i++] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3);
	v33_inputs[i++] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4);
	v33_inputs[i++] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5);
	v33_inputs[i++] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6);
	v33_inputs[i++] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7);

	V33_Input = 0;
	V33_Input = (V33_Input | v33_inputs[7]);
	for(i=6; i>=0; i--) {

		V33_Input = ((V33_Input << 1) | v33_inputs[i]);
	}

	// for testing
/*	if(v33_inputs[0]) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // GREEN LED
	}
	else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // GREEN LED
	}*/



	// Switch Inputs
	i = 0;
	sw_inputs[i++]	= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9);
	sw_inputs[i++]	= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10);

	// AC Inputs
	AC_Input = 0;
	for(i=0; i<MAX_AC_INPUTS; i++) {
		if(pulse_counter[i])
			acinputs[i] = 1;
		else
			acinputs[i] = 0;
	}

	AC_Input = 0;
	for(i=4; i>=0; i--) {

		AC_Input = ((AC_Input << 1) | acinputs[i]);
	}

/*	dig_input_arr[0] = ((AC_Input & 0xFF00) >> 8);
	dig_input_arr[1] = (AC_Input & 0xFF);
	dig_input_arr[2] = ((V33_Input & 0xFF00) >> 8);
	dig_input_arr[3] = (V33_Input & 0xFF);
	dig_input_arr[4] = ((Digital_Input & 0xFF00) >> 8);
	dig_input_arr[5] = (Digital_Input & 0xFF);

*/


}

/*
void read_analog_data(void)
{

	int i = 0;


	for(i=0; i<6; i++) {

		adc_volt[i] = (float)((adc_buff[i] * 24.0) / MAX_ADC_COUNT_24V);
		adc_data[i] = (uint16_t)(adc_volt[i] * ADC_SCALE);
	}

	for(i=6; i<10; i++) {

			adc_volt[i] = (float)((adc_buff[i] * 5.0) / MAX_ADC_COUNT_5V);
			adc_data[i] = (uint16_t)(adc_volt[i] * ADC_SCALE);
		}

	for(i=10; i<14; i++) {

				adc_volt[i] = (float)((adc_buff[i] * 3.3) / MAX_ADC_COUNT_3V3);
				adc_data[i] = (uint16_t)(adc_volt[i] * ADC_SCALE);

				if(i==11)
				{
					adc_volt[i] = (float)((adc_buff[i] * 3.3) / 4095);		// DJ	//Adjusted for reading lower voltages
					adc_data[i] = (uint16_t)(adc_volt[i] * ADC_SCALE);
				}

				if(i==12)
				{
					adc_volt[i] = (float)((adc_buff[i] * 3.3) / 3400);		// DJ	//Adjusted for reading lower voltages
					adc_data[i] = (uint16_t)(adc_volt[i] * ADC_SCALE);
				}
			}

	for(i=14; i<16; i++) {

					if(i==14)
					{
					adc_volt[i] = (float)((adc_buff[i] * 5.0) / MAX_ADC_COUNT_DIFF);
					adc_data[i] = (uint16_t)(adc_volt[i] * ADC_SCALE);
					}

					if(i==15)
					{
					adc_volt[i] = (float)((adc_buff[i] * 24.0) / 2978);	// DJ	//2978 -> counts for 2.4VDC
					adc_data[i] = (uint16_t)(adc_volt[i] * ADC_SCALE);
					}

				}
}
*/

void read_analog_data1(void)
{

	uint32_t status = 0;
	uint32_t ch_sel = 1;
	uint32_t ch_seq = 0;
	static uint16_t i = 0;
	uint16_t j[14] = {0,1};
	uint16_t k = 0;
	double temp = 0;
//	static uint16_t HV1_sample = 0;
//	static uint32_t HV1_readings[2000], HV1_acc = 0;
//	static double temp_HV1_voltage = 0;
//	static float adcbuff_14 = 0;
//	static uint16_t voltage_capture_count = 0;
//	static uint16_t adc_data14_prev = 0, adc_data14_curr = 0;

	adc_data14_prev = adc_data[14];

	  for(i=0; i<=1; i++)
	  {

		HAL_ADC_Stop(&hadc3);
		status = hadc3.Instance->CR;
		ch_sel = 1;
		k = j[i];

		if((status & 0x4) == 0)
		{
			ch_sel = (ch_sel << k);
			hadc3.Instance->PCSEL = ch_sel;
			ch_seq = (k << 6);
			hadc3.Instance->SQR1  = ch_seq;
			//hadc1.Instance->SQR2  = ch_seq;
			//hadc1.Instance->SQR3  = ch_seq;
			//hadc1.Instance->SQR4  = ch_seq;

			HAL_ADC_Start(&hadc3);
			//HAL_Delay(1);
			HAL_ADC_PollForConversion(&hadc3, 10);
			adc_buff[i + 14] = HAL_ADC_GetValue(&hadc3);
			//temp = (float)((float)adc_buff[k] / 65520.0);
			//adc_volt[k] = (float)(temp * 3.3);
			//adc_data[i] = (uint16_t)adc_buff[i];

			/*
			temp = (double)((float)adc_buff[i + 14] * scale_factor_150V);
			//Since the voltage goes up to 130V, It is not possible to send 130000 millivolts in 16-bit number (Max 65535).
			//Hence converting to 2 decimal precision. 130V -> 130.00V
			temp = temp/10;
			adc_data[i + 14] = (uint16_t)temp;
			*/
			adcbuff_14 = (float)(adc_buff[i + 14]);
			temp = (double)((0.0000007*adcbuff_14*adcbuff_14)+(0.2213*adcbuff_14)-74.627);	//2nd degree polynomial to find voltage (scaled by x100) from counts
			adc_data[i + 14] = (uint16_t)temp;
			adc_data14_curr = adc_data[14];
		}

		//HV_Input1[cnt++] = adc_buff[14];
		//if(cnt >= 10000)
		//	cnt = 0;
	  }
		//calculating Supply Current measured from the Current measurement board
		//Using 2000 samples to find the average current consumption
		if(HV1_sample < 50) //50 samples average
		{
			HV1_sample++;
			HV1_readings[HV1_sample] = adc_data[14];
			HV1_acc = HV1_acc + HV1_readings[HV1_sample];
		}
		else
		{
			HV1_sample = 0;
			temp_HV1_voltage = (HV1_acc/50);	//50 samples average
			HV1_channel_voltage = (uint16_t)(temp_HV1_voltage);		//Current value in mA - milliamps
			HV1_channel_adc_counts = (uint16_t)(((float)(HV1_channel_voltage) * 10.0)/(scale_factor_150V));
			HV1_acc = 0;
		}

		if(voltage_capture_count < 1000)
		{
			HV1_channel_voltage_capture[voltage_capture_count] = adc_data[14];
			voltage_capture_count++;
		}
		else
		{
			voltage_capture_count = 0;
		}

		// Finding the Fall time (Open time) and then the Slew rate in V/microsecond
		if(Measure_fall_time)
		{
			if(adc_data14_prev > 9300 && adc_data14_curr < 9300)
			{
				start_falltime_counter = 1;
			}
		}
		else
		{
			start_falltime_counter = 0;
		}

		if(Measure_fall_time && start_falltime_counter)
		{
			if((adc_data14_prev) > 200 && (adc_data14_curr < 200))
			{
				start_falltime_counter = 0;
				Measure_fall_time = 0;
			}
		}

		// Finding the Rise time (Close time) and then the Slew rate in V/microsecond
		if(Measure_rise_time)
		{
			if(adc_data14_prev < 200 && adc_data14_curr > 200)
			{
				start_risetime_counter = 1;
			}
		}
		else
		{
			start_risetime_counter = 0;
		}

		if(Measure_rise_time && start_risetime_counter)
		{
			if((adc_data14_prev) < 9300 && (adc_data14_curr > 9300))
			{
				start_risetime_counter = 0;
				Measure_rise_time = 0;
			}
		}
}


void update_led(int toggle)
{

	if(toggle) {
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);	// RED LED
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // GREEN LED
		//SetOutputs();
	}
	else {
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // RED LED
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // GREEN LED
		//ClearOutputs();
	}
}

void update_red_led(int toggle1)
{

	if(toggle1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);	// RED LED
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // GREEN LED
	}
	else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // RED LED
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // GREEN LED
	}
}


void SetOutputs(void)
{


	// 12 Relay Outputs ON
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);

	//  12 sinking Digital Driver Outputs ON
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	// UUT Power ON
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);


	// 12 3V3 Digital Output ON
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	// LED ON
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

	// 4 Differential Outputs ON
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);

	// 8 sourcing 24V Digital Outputs ON
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
}


void get_software_hardware_versions(void)
{

	tdab_sw_ver = (uint16_t)TDAB_SOFTWARE_VERSION;
	tdab_hw_ver = (uint16_t)TDAB_HARDWARE_VERSION;
}



void change_uut_state(void *data)
{

	uut_protocol_state = UUT_SYNC_STATE;
	send_uut_command = 1;

}

void clear_uut_data(void *data)
{
	int i = 0;

	if(clear_data) {
		uut_sw_ver = 0;
		uut_hw_ver = 0;
		uut_eeprom_status = EEPROM_TEST_IN_PROGRESS;
		uut_dig_in = 0;

		for(i=0; i<12; i++)
			uut_ana[i] = 0x00;

		//clear Pico 2.1 vale board uut data
		clear_pico_valve_data();
	}

	clear_data = 0;
}

//clear Pico 2.1 vale board uut data
void clear_pico_valve_data(void)
{
	pico_sw_version = 0;
	pico_hw_version = 0;
	green_led_flag = 0;
	red_led_flag = 0;
	stack_rtd_offset = 0;
	stack_rtd_scale_factor = 0;
	stack_rtd_counts = 0;
	heater_rtd_offset = 0;
	heater_rtd_scale_factor = 0;
	heater_rtd_counts = 0;
	hall_offset = 0;
	hall_gain = 0;
	hall_counts = 0;
	heater_flag = 0;
	pico_board_supply_amps = 0;
	pico_board_supply_power = 0;
}

void set_clear_relay_output(void *data)
{
	int i;
	int bit;
	uint16_t *val;

	bit = 1;
	val = (int *)data;

	for (i = 0; i < 12; i++) {
		if (*val & bit)
			HAL_GPIO_WritePin(relay_map[i].port, relay_map[i].pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(relay_map[i].port, relay_map[i].pin, GPIO_PIN_RESET);
		bit <<= 1;
	}

}


void set_clear_v33_output(void *data)
{
	int i;
	int bit;
	int *val;

	bit = 1;
	val = (int *)data;

	for (i = 0; i < 12; i++) {
		if (*val & bit)
			HAL_GPIO_WritePin(v33_dig_out_map[i].port, v33_dig_out_map[i].pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(v33_dig_out_map[i].port, v33_dig_out_map[i].pin, GPIO_PIN_RESET);
		bit <<= 1;
	}

}

void set_clear_v33diff_output(void *data)
{
	int i;
	int bit;
	int *val;

	bit = 1;
	val = (int *)data;

	for (i = 0; i < 4; i++) {
		if (*val & bit)
			HAL_GPIO_WritePin(v33_diff_out_map[i].port, v33_diff_out_map[i].pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(v33_diff_out_map[i].port, v33_diff_out_map[i].pin, GPIO_PIN_RESET);
		bit <<= 1;
	}

}


void set_clear_v24_source_output(void *data)
{
	int i;
	int bit;
	int *val;

	bit = 1;
	val = (int *)data;

	for (i = 0; i < 8; i++) {
		if (*val & bit)
			HAL_GPIO_WritePin(v24_source_out_map[i].port, v24_source_out_map[i].pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(v24_source_out_map[i].port, v24_source_out_map[i].pin, GPIO_PIN_RESET);
		bit <<= 1;
	}

}

/*
void update_analog_output1(void *data)
{


	//if(Analog_output[1] >= 3800)
//		Analog_output[1] = 3800;	// This count gives 0.608 Volts
	//if(Analog_output[0] >= 3800)
	//	Analog_output[0] = 3800;	// This count gives 0.608 Volts


	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = (uint8_t)((Analog_output[0]&0xFF00)>>8);
    spi_data[2] = (uint8_t)(Analog_output[0]& 0x00FF);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
    HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);

	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = (uint8_t)((Analog_output[0] & 0xFF00)>>8);
    spi_data[2] = (uint8_t)(Analog_output[0] & 0x00FF);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
    HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);


	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = (uint8_t)((Analog_output[0] & 0xFF00)>>8);
    spi_data[2] = (uint8_t)(Analog_output[0] & 0x00FF);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
    HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);

	spi_data[0] = 0x19;    //Channel B
	//spi_data[0] = 0x1F;        // Both Channels
	spi_data[1] = (uint8_t)((Analog_output[1] & 0xFF00)>>8);
	spi_data[2] = (uint8_t)(Analog_output[1] & 0x00FF);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
    HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);

	spi_data[0] = 0x19;    //Channel B
	//spi_data[0] = 0x1F;        // Both Channels
	spi_data[1] = (uint8_t)((Analog_output[1] & 0xFF00)>>8);
	spi_data[2] = (uint8_t)(Analog_output[1] & 0x00FF);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
    HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);

}

void update_analog_output(void *data)
{


	//if(Analog_output[0] >= 3800)
	//	Analog_output[0] = 3800;	// This count gives 0.608 Volts
	//if(Analog_output[1] >= 3800)
	//	Analog_output[1] = 3800;	// This count gives 0.608 Volts

	spi_data[0] = 0x19;    //Channel B
	//spi_data[0] = 0x1F;        // Both Channels
	spi_data[1] = (uint8_t)((Analog_output[1] & 0xFF00)>>8);
	spi_data[2] = (uint8_t)(Analog_output[1] & 0x00FF);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
    HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);

	spi_data[0] = 0x19;    //Channel B
	//spi_data[0] = 0x1F;        // Both Channels
	spi_data[1] = (uint8_t)((Analog_output[1] & 0xFF00)>>8);
	spi_data[2] = (uint8_t)(Analog_output[1] & 0x00FF);

   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
   // HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);

	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = 0; //(uint8_t)((Analog_output[0]&0xFF00)>>8);
    spi_data[2] = 0; //(uint8_t)(Analog_output[0]& 0x00FF);

   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
   // HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);

	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = (uint8_t)((Analog_output[0] & 0xFF00)>>8);
    spi_data[2] = (uint8_t)(Analog_output[0] & 0x00FF);

   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
   // HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);


	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = (uint8_t)((Analog_output[0] & 0xFF00)>>8);
    spi_data[2] = (uint8_t)(Analog_output[0] & 0x00FF);

  //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
  //  HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);


}
*/

void update_analog_output1(void *data)
{
		spi_data[0] = 0x71;    //Channel B
		//spi_data[0] = 0x1F;        // Both Channels
		spi_data[1] = 0; //(uint8_t)((Analog_output[1] & 0xFF00)>>8);
		spi_data[2] = 0; //(uint8_t)(Analog_output[1] & 0x00FF);

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
	    HAL_Delay(50);

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(50);
		HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(50);

		spi_data[0] = 0x71;    //Channel B
		//spi_data[0] = 0x1F;        // Both Channels
		spi_data[1] = 0; //(uint8_t)((Analog_output[1] & 0xFF00)>>8);
		spi_data[2] = 0; //(uint8_t)(Analog_output[1] & 0x00FF);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
		HAL_Delay(50);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(50);
		HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(50);

		spi_data[0] = 0x18;    //Channel A
		//spi_data[0] = 0x1F;        // Both Channels
		spi_data[1] = (uint8_t)((Analog_output[0] & 0xFF00)>>8);
		spi_data[2] = (uint8_t)(Analog_output[0] & 0x00FF);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
		HAL_Delay(50);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(50);
		HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(50);

		spi_data[0] = 0x18;    //Channel A
		//spi_data[0] = 0x1F;        // Both Channels
		spi_data[1] = (uint8_t)((Analog_output[0] & 0xFF00)>>8);
		spi_data[2] = (uint8_t)(Analog_output[0] & 0x00FF);

	   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
	   // HAL_Delay(50);

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(50);
		HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(50);

		spi_data[0] = 0x19;    //Channel B
	    //spi_data[0] = 0x19;    //Channel B
	    //spi_data[0] = 0x1F;        // Both Channels
	    spi_data[1] = 0; //(uint8_t)((Analog_output[1]&0xFF00)>>8);
	    spi_data[2] = 0; //(uint8_t)(Analog_output[1]& 0x00FF);

	   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
	   // HAL_Delay(50);

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	    HAL_Delay(50);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	    HAL_Delay(50);
	    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	    HAL_Delay(50);

		spi_data[0] = 0x19;    //Channel B
	    //spi_data[0] = 0x19;    //Channel B
	    //spi_data[0] = 0x1F;        // Both Channels
	    spi_data[1] = (uint8_t)((Analog_output[1] & 0xFF00)>>8);
	    spi_data[2] = (uint8_t)(Analog_output[1] & 0x00FF);

	   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
	   // HAL_Delay(50);

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	    HAL_Delay(50);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	    HAL_Delay(50);
	    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	    HAL_Delay(50);


		spi_data[0] = 0x19;    //Channel B
	    //spi_data[0] = 0x19;    //Channel B
	    //spi_data[0] = 0x1F;        // Both Channels
	    spi_data[1] = (uint8_t)((Analog_output[1] & 0xFF00)>>8);
	    spi_data[2] = (uint8_t)(Analog_output[1] & 0x00FF);

	  //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
	  //  HAL_Delay(50);

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	    HAL_Delay(50);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	    HAL_Delay(50);
	    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	    HAL_Delay(50);
}

void update_analog_output(void *data)
{


	//if(Analog_output[0] >= 3800)
	//	Analog_output[0] = 3800;	// This count gives 0.608 Volts
	//if(Analog_output[1] >= 3800)
	//	Analog_output[1] = 3800;	// This count gives 0.608 Volts

	spi_data[0] = 0x71;    //Channel B
	//spi_data[0] = 0x1F;        // Both Channels
	spi_data[1] = 0; //(uint8_t)((Analog_output[1] & 0xFF00)>>8);
	spi_data[2] = 0; //(uint8_t)(Analog_output[1] & 0x00FF);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
    HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(50);

	spi_data[0] = 0x71;    //Channel B
	//spi_data[0] = 0x1F;        // Both Channels
	spi_data[1] = 0; //(uint8_t)((Analog_output[1] & 0xFF00)>>8);
	spi_data[2] = 0; //(uint8_t)(Analog_output[1] & 0x00FF);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
	HAL_Delay(50);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(50);

	spi_data[0] = 0x19;    //Channel B
	//spi_data[0] = 0x1F;        // Both Channels
	spi_data[1] = (uint8_t)((Analog_output[1] & 0xFF00)>>8);
	spi_data[2] = (uint8_t)(Analog_output[1] & 0x00FF);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
	HAL_Delay(50);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(50);

	spi_data[0] = 0x19;    //Channel B
	//spi_data[0] = 0x1F;        // Both Channels
	spi_data[1] = (uint8_t)((Analog_output[1] & 0xFF00)>>8);
	spi_data[2] = (uint8_t)(Analog_output[1] & 0x00FF);

   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
   // HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(50);

	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = 0; //(uint8_t)((Analog_output[0]&0xFF00)>>8);
    spi_data[2] = 0; //(uint8_t)(Analog_output[0]& 0x00FF);

   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
   // HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(50);

	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = (uint8_t)((Analog_output[0] & 0xFF00)>>8);
    spi_data[2] = (uint8_t)(Analog_output[0] & 0x00FF);

   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
   // HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(50);


	spi_data[0] = 0x18;    //Channel A
    //spi_data[0] = 0x19;    //Channel B
    //spi_data[0] = 0x1F;        // Both Channels
    spi_data[1] = (uint8_t)((Analog_output[0] & 0xFF00)>>8);
    spi_data[2] = (uint8_t)(Analog_output[0] & 0x00FF);

  //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// EEPROM DESELECT
  //  HAL_Delay(50);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_SPI_Transmit(&hspi2, spi_data, 3, 5000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(50);


}


void set_clear_v24_sink_output(void *data)
{
	int i;
	int bit;
	int *val;

	bit = 1;
	val = (int *)data;

	for (i = 0; i < 8; i++) {
		if (*val & bit)
			HAL_GPIO_WritePin(v24_sink_out_map[i].port, v24_sink_out_map[i].pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(v24_sink_out_map[i].port, v24_sink_out_map[i].pin, GPIO_PIN_RESET);
		bit <<= 1;
	}

}

void PCP_pulse_counter_test(void *data)
{
	uint16_t i = 0;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET); // Up Count direction
			HAL_Delay(10);

		  	spiTxData[0] = spiRxData[0] = 0;
		  	spiTxData[1] = spiRxData[1]  = 0;
		  	spiTxData[2] = spiRxData[2]  = 0;
		  	spiTxData[3] = spiRxData[3]  = 0;
		  	spiTxData[4] = spiRxData[4]  = 0;

			  // Generating pulses of Differential output 1 Pin (PB2)
			  for(i=0; i<pulse_count; i++) {
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
				  HAL_Delay(5);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
				  HAL_Delay(5);
			  }

				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	// chip select
				HAL_Delay(10);
				spiTxData[0] = 0x60;	// read counter register
			//	HAL_SPI_Transmit(&hspi4, &spiTxData[0], 5, 5000);
				HAL_SPI_TransmitReceive(&hspi4, &spiTxData[0], &spiRxData[0], 5, 5000);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);	// chip select
				HAL_Delay(10);

				PCP_pulse_count_read = spiRxData[4];

				//Clear the counter register in the IC
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
				HAL_Delay(10);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	// chip select
				HAL_Delay(10);
				spiTxData[0] = 0x98;	// clear counter
				//HAL_SPI_Transmit(&hspi4, &spiTxData[0], 1, 5000);
				HAL_SPI_TransmitReceive(&hspi4, &spiTxData[0], &spiRxData[0], 5, 500);
				HAL_Delay(10);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
				HAL_Delay(10);

}


void Ultimus_Pressure_Sensor1_Test(void *data)
{
	if(read_U13counts != 0)
	{
		U13counts = 0; 	//Clear the earlier value and be ready to read new value
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //This chip select is for SPI3 which was used earlier. Now using SPI4 interface, hence this is commented.
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);	// chip select for SPI4 // J8 Connector
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	// chip select asserted for SPI4
		HAL_Delay(50);
		HAL_SPI_TransmitReceive(&hspi4, &spiUltiTx[0], &spiUltiRx[0], 4, 5000);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);	// Set Chip select line for SPI4
		HAL_Delay(50);
		U13counts = ((U13counts | spiUltiRx[0])<<8)|(spiUltiRx[1]);
	}

}

void Ultimus_Pressure_Sensor2_Test(void *data)
{
	if(read_U15counts != 0)
		{
			U15counts = 0; //Clear the earlier value and be ready to read new value
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	// chip select
			HAL_Delay(50);
			HAL_SPI_TransmitReceive(&hspi4, &spiUltiTx[0], &spiUltiRx[0], 4, 5000);
			HAL_Delay(50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_Delay(50);
			U15counts = ((U15counts | spiUltiRx[0])<<8)|(spiUltiRx[1]);
		}
}


void set_clear_uut_power(void *data)
{

	//int bit;
	int *val;

	//bit = 1;
	val = (int *)data;

		if (*val > 0)												//Any non-Zero value will turn On UUT relay
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);	//UUT Power On
		else
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);		//UUT Power Off
}


void ClearOutputs(void)
{

	// Relay Outputs OFF
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);


	// Clear Digital Driver Outputs
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	// UUT Power OFF
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);


	// 3V3 Digital Output OFF
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);


	// LED OFF
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	// Clear Differential Outputs
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

	// 24V Digital Outputs OFF
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);

}

void calculate_board_power(void)
{
	float temp = 0;
	float board_volt = 0;

	temp = adc_data[5];
	board_volt = (adc_data[0] / 1000);

	// 1.448 V per Amp -> (1 / 1.448 = 0.69)
	// Amp = Vout * 0.69

	pico_board_supply_amps = (uint16_t) (temp * 0.69);
	pico_board_supply_power = (uint16_t) (board_volt * pico_board_supply_amps);

}


void ProcessPCCommands(void)
{

	int i,j;
	uint16_t command = 0;
	uint16_t address = 0;
	uint16_t count = 0;
	MODBUS_REG_MAP *pmap;
	int map_count = 0;

	command = (record.data[0]);
	command = ((command << 8) | record.data[1]);


	map_count = sizeof(mod_reg_map) / sizeof(mod_reg_map[0]);


	switch(record.function_code) {

		case READ_HOLDING_REGISTERS:
			address = record.read_start_address;
			count = record.read_registers;

			// Find logical address in the map
			for(i=0; i<(map_count - 1); i++) {
				pmap = &mod_reg_map[i];
				if(pmap->address == address)
					break;
			}

			// Check if we found a valid address entry
			if (i != map_count - 1) {
				for(i=0,j=0; i<count; i++) {

					record.data[j++] = (uint8_t) ((*pmap->data & 0xFF00) >> 8);
					record.data[j++] = (uint8_t)(*pmap->data & 0xFF);

					if (pmap->callback)
						pmap->callback(pmap->data);

					pmap++;
					if (pmap->address) {
						if(pmap->address != (address + (i+1)))
							break;
					} else
						break;
				}
			}

		break;


		case WRITE_MULTIPLE_REGISTERS:
			address = record.write_start_address;
			count = record.write_registers;

			// Find logical address in the map
			for(i=0; i<map_count - 1; i++) {
				pmap = &mod_reg_map[i];
				if(pmap->address == address)
					break;
			}

			// Check if we found a valid address entry
			if (i != map_count - 1) {
				for(i=0,j=0; i<count; i++) {
					*pmap->data = record.data[j++];
					*pmap->data <<= 8;
					*pmap->data |= record.data[j++];

					if (pmap->callback)
						pmap->callback(pmap->data);

					pmap++;
					if (pmap->address) {
						if(pmap->address != (address + (i+1)))
							break;
					}
					else
						break;
				}
			}

			break;


		default:
			break;

	}
}



void FillTransmitBuffer(uint8_t count, uint8_t index)
{
	int i = 0;
	uint16_t crc_count;
	uint16_t cal_crc;


	for(i=0; i<record.read_byte_count; i++) {

		PC_TxBuff[index++] = record.data[i];
	}

	crc_count = (record.read_byte_count + 3);
	cal_crc = CRC16(PC_TxBuff,crc_count);

	// CRC Low Byte first and then High Byte next
	PC_TxBuff[index++] = (uint8_t) (cal_crc & 0x00FF);
	PC_TxBuff[index] = (uint8_t) ((cal_crc & 0xFF00) >> 8);

	pc_data_out = crc_count + 2;

}


/*
 * 	Calculation of CRC16 software logic
 */

uint16_t CRC16(unsigned char *puchMsg, uint16_t usDataLen)
{
	unsigned char uchCRCHi = 0xFF ; /* high byte of CRC initialized */
	unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized */
	unsigned char uIndex ; /* will index into CRC lookup table */
	uint16_t temp;

	while (usDataLen--) /* pass through message buffer */
	{
			uIndex = (unsigned char)(uchCRCLo ^ *puchMsg++) ; /* calculate the CRC */
			uchCRCLo =(unsigned char)(uchCRCHi ^  auchCRCHi[uIndex]);
			uchCRCHi = auchCRCLo[uIndex] ;
	}

	temp = uchCRCHi;
	return (uint16_t)((temp << 8) | uchCRCLo) ;
}

void average_samples(uint32_t data, uint16_t channel)
{
	static uint16_t avg_counts = 0;
	uint16_t i = 0;

	temp_adc_buff[channel] += data;

	if(channel == 13)
		 avg_counts++;

	if(avg_counts >= ADC_MAX_AVERAGE) {
		avg_counts = 0;
		for(i=0; i<=13; i++) {
			adc_buff[i] = (temp_adc_buff[i] / ADC_MAX_AVERAGE);
			temp_adc_buff[i] = 0;
		}
		calculate_adc_voltage = 1;
	}

}



void init_pcp_board(void)
{


	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET); // Up Count direction
	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET); // UUT Power ON
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Enable Buffers on PCP board
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET); // RED LED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // GREEN LED


 	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	// chip select
	HAL_Delay(10);
	spiTxData[0] = 0x98;	// clear counter
	//HAL_SPI_Transmit(&hspi4, &spiTxData[0], 1, 5000);
	HAL_SPI_TransmitReceive(&hspi4, &spiTxData[0], &spiRxData[0], 5, 500);
	HAL_Delay(10);
  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
  	HAL_Delay(10);

  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);	// chip select
	HAL_Delay(10);
	spiTxData[0] = 0xE0;	// load counter register
	//HAL_SPI_TransmitReceive(&hspi4, &spiTxData[0], &spiRxData[0], 2, 5000);
	HAL_SPI_Transmit(&hspi4, &spiTxData[0], 1, 500);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);	// chip select
	HAL_Delay(10);


}


void clear_pulse_counter(void *data)
{
	int *val;

	val = (int *)data;

	if(*val != 0)
	{
		start_time = 0;
		pulse_frequency = 0;
		timer7_count = 0;
		HV_input_prev_state = 0;
		HV_input_state = 0;
		pulse_acc = 0;
		HAL_Delay(5);
	}

}

void start_fall_time_test(void *data)
{
	int *val;

	val = (int *)data;

	if(*val != 0)
	{
		Measure_fall_time = 1;
		falltime_counter = 0;
		fall_time = 0;
		start_falltime_counter = 0;
	}

}

void start_rise_time_test(void *data)
{
	int *val;

	val = (int *)data;

	if(*val != 0)
	{
		Measure_rise_time = 1;
		risetime_counter = 0;
		rise_time = 0;
		start_risetime_counter = 0;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t toggle = 0xFF;
	uint32_t status = 0;
	uint32_t ch_sel = 1;
	uint32_t ch_seq = 0;
	static uint16_t i = 0;
	uint16_t j[14] = {16,17,14,15,18,19,3,7,9,5,10,11,4,8};
	uint16_t k = 0;
	uint16_t count = 0;
	double	temp = 0.0;
	double temp_current = 0.0;
	uint16_t supplycurrentreadings[2000];
	uint16_t current_sample = 0;
	uint32_t current_acc = 0;

	/* Enable the CPU Cache */
	//CPU_CACHE_Enable();s
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_ADC3_Init();
  MX_TIM6_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  	HAL_UART_Receive_IT(&huart1, &pc_data, 1);
  	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  	HAL_Delay(500);

  	//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, 2);
  	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);


  	//HAL_ADC_Stop(&hadc3);
  	//ADC_Disable(&hadc3);
  	HAL_Delay(500);
  //	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
 // 	HAL_Delay(500);
  //	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, 16);
  	//HAL_Delay(500);



  	HAL_TIM_Base_Start_IT(&htim6);	// 10mSec interrupt for capturing the AC Input
  	HAL_TIM_Base_Start_IT(&htim7);	// Timer for pulse accumulator
  	ClearOutputs();
  	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET); // UUT Power ON
  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);		// RS232 transmit mode always for Quixie board
  	//  init_pcp_board();
  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_UART_Receive_IT(&huart1, &pc_data, 1);
	  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	  //HAL_ADC_Start_DMA(&hadc3, (uint32_t *)dma_buff, 2);

	  		PCModbusRTU();
	  		UUTModbusRTU();
	  		ReadInputs();

	  		// Toggle status led for every 1 seconds
	  		if(ticks >= 100) {
	  			ticks = 0;
	  			update_led(toggle);
	  			toggle = ~toggle;
	  		}


	  		HAL_ADC_Stop(&hadc1);
	  		if(i == 0)
	  		HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	  		status = hadc1.Instance->CR;
	  		ch_sel = 1;
	  		k = j[i];

	  		if((status & 0x4) == 0) {
	  			ch_sel = (ch_sel << k);
	  			hadc1.Instance->PCSEL = ch_sel;
	  			ch_seq = (k << 6);
	  			hadc1.Instance->SQR1  = ch_seq;

	  			HAL_ADC_Start(&hadc1);
	  			HAL_ADC_PollForConversion(&hadc1, 10);
	  			raw_adc_buff[i] = HAL_ADC_GetValue(&hadc1);

	  			average_samples(raw_adc_buff[i], i);

	  			if(i++ >= 13)
	  				i = 0;
	  		}

	  		if(calculate_adc_voltage) {

	  			calculate_adc_voltage = 0;
	  			for(count=0; count<=13; count++) {

	  				if((count>=0) && (count<=3)) {
	  					temp = (double)((float)adc_buff[count] * scale_factor_24V);
	  					adc_data[count] = (uint16_t)temp;
	  				}
	  				else if((count>=4) && (count<=5)) {
	  					temp = (double)((float)adc_buff[count] * scale_factor_5V);
	  					adc_data[count] = (uint16_t)temp;
	  				}
	  				else if((count>=6) && (count<=11)) {
	  					temp = (double)((float)adc_buff[count] * scale_factor_3V3);
	  					adc_data[count] = (uint16_t)temp;
	  				}
	  				else if((count>=12) && (count<=13)) {
	  					temp = (double)((float)adc_buff[count] * scale_factor_5V);
	  					adc_data[count] = (uint16_t)temp;

	  				}
	  			}

	  			calculate_board_power();
	  		}

	  		//calculating Supply Current measured from the Current measurement board
	  		//Using 2000 samples to find the average current consumption
	  		if(current_sample < 2000)
			{
				current_sample++;
				supplycurrentreadings[current_sample] = adc_data[12];
				current_acc = current_acc + supplycurrentreadings[current_sample];

			}
	  		else
	  		{
				current_sample = 0;
				temp_current = ((current_acc/2000) - 2500)/0.4;	//2.5V for Zero current and above that 400mV or 0.4V per Amp.
				Supply_Current = (uint16_t)(temp_current);		//Current value in mA - milliamps
				current_acc = 0;
			}

	  		//Reading HV analog inputs for Pico Nexus (ADC3 input 0 and 1)
	  		read_analog_data1();

			// Counting PZD output pulses
			HV_input_prev_state = HV_input_state;
			HV_input_state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
			if((HV_input_prev_state == 0) && (HV_input_state == 1))
			{
				start_time++;
				pulse_acc++;
			}

	  		//Start capturing ADC data in Timer7 interrupt after reaching main.
	  		start_capture = 1;

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 1;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.PLL3.PLL3M = 8;
  PeriphClkInitStruct.PLL3.PLL3N = 150;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL3;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 14;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DFSDM;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DFSDM;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = ENABLE;
  hadc3.Init.Oversampling.Ratio = 16;
  hadc3.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc3.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC6
                           PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF6 PF7
                           PF9 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF8 PF11 PF12 PF13
                           PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG5 PG6 PG7
                           PG8 PG9 PG10 PG11
                           PG12 PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD14 PD15 PD2
                           PD3 PD4 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 PD11 PD12
                           PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
