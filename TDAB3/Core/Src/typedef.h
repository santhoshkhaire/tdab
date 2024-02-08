/*
 * typedef.h
 *
 *  Created on: 21-Sep-2020
 *      Author: Santhosh.Khaire
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

// Macros Definition
#define NUM_ADC_CHANNELS					32
#define UUT_NUM_ADC_CHANNELS				12
#define MAX_ADC_COUNT 						4095
#define MAX_ADC_VOLTAGE						3.34
#define ADC_SCALE							1000
#define ADC_MAX_AVERAGE						16

#define MAX_ADC_COUNT_24V					3680 //3820
#define	MAX_ADC_COUNT_5V					3683
#define MAX_ADC_COUNT_3V3					3678
#define MAX_ADC_COUNT_DIFF					2455


#define MAX_DIGITAL_INPUTS					8
#define MAX_ANALOG_INPUTS					8
#define MAX_SWITCH_INPUTS					2
#define MAX_AC_INPUTS						5
#define MAX_DIG_INPUTS						6	// Bytes
#define MAX_ANA_INPUTS						16	// 16 Registers

#define UART_MSG_SIZE   					40	//0x20		//Increased to prevent overflowing array // DJ // 19-Jun-2020 Ultimus testing
#define SLAVE_ADDRESS						0x01

#define READ_HOLDING_REGISTERS				0x03
#define WRITE_MULTIPLE_REGISTERS			0x10
#define READ_WRITE_MULTIPLE_REGISTERS		0x17

// Modbus Logical Address
#define V240_AC_DIGITAL_INPUT_ADDRESS		1000
#define V33_DC_DIGITAL_INPUT_ADDRESS		1001
#define V24_DC_DIGITAL_INPUT_ADDRESS		1002

#define V33_DIGITAL_OUTPUT_ADDRESS			2000
#define V33_DIFF_OUTPUT_ADDRESS				2001
#define SLIM_RELAY_OUTPUT_ADDRESS			2002
#define V24_LEVEL_SOURCING_ADDRESS			2003
#define V24_LEVEL_SINKING_ADDRESS			2004
#define UUT_POWER_ON_OFF_ADDRESS			2005

#define V24_ANALOG_INPUT_ADDRESS			3000
#define ANALOG_DIFF_INPUT_ADDRESS			3006
#define V5_ANALOG_INPUT_ADDRESS				3008
#define V33_ANALOG_INPUT_ADDRESS			3012
#define ANALOG_INPUT_ADDRESS				3000
#define ANALOG_OUTPUT_ADDRESS				4000
#define ANALOG_INPUT_END_ADDRESS			3015

#define SPECIAL_ADDRESS						5000
#define UUT_RTD_START_ADDRESS				5001
#define UUT_RTD_END_ADDRESS					5012
#define UUT_DIG_INPUT_ADDRESS				5013
#define UUT_EEPROM_READ_ADDRESS				5014

#define UUT_SOFTWARE_VER_ADDRESS			102	// OneMelt Software Version
#define UUT_HARDWARE_VER_ADDRESS			103	// OneMelt Hardware Version

// Pico Valve Board Logical Address
#define PICO_SOFTWARE_VERSION				5040
#define PICO_HARDWARE_VERSION				5041
#define FLASH_GREEN_LED						5042
#define FLASH_RED_LED						5043

#define STACK_RTD_COUNTS					5044
#define STACK_RTD_OFFSET					5045
#define STACK_RTD_30_DEG					5046
#define STACK_RTD_80_DEG					5047
#define STACK_RTD_SCALE_FACTOR				5048

#define HEATER_RTD_COUNTS					5049
//#define HEATER_RTD_OFFSET					5050
//#define HEATER_RTD_30_DEG					5051
//#define HEATER_RTD_80_DEG					5052
//#define HEATER_RTD_SCALE_FACTOR				5053

#define HALL_COUNTS							5054
#define HALL_OFFSET							5055
#define HALL_VIN_020						5056
#define HALL_VIN_060						5057
#define HALL_GAIN							5058


#define HEATER_ON_OFF						5059
#define BOARD_SUPPLY_AMPS					5060
#define BOARD_SUPPLY_POWER					5061

// Heater RTD Calibration
#define CALIBRATION_STATUS					5080

// Reading and Writing 32 bit data from 6000 address location
#define HEATER_RTD_CALIB_HIGH				6000
#define HEATER_RTD_CALIB_LOW				6001
#define CALIB_COEFF1_VAL_HIGH				6002
#define CALIB_COEFF1_VAL_LOW				6003
#define CALIB_COEFF2_VAL_HIGH				6004
#define CALIB_COEFF2_VAL_LOW				6005
#define CALIB_COEFF3_VAL_HIGH				6006
#define CALIB_COEFF3_VAL_LOW				6007

//#define TDAB_SOFTWARE_VERSION				100		//USED for ONEMELT CONTROLLER TESTER
//#define TDAB_SOFTWARE_VERSION				101		//USED for PCP board tester for NORDSON EFD
//#define TDAB_SOFTWARE_VERSION				103		//USED for Ultimus board tester for NORDSON EFD
#define TDAB_SOFTWARE_VERSION				100		//USED for Pico Nexus NORDSON EFD
#define TDAB_HARDWARE_VERSION				300		//TDAB Rev 3 Board


typedef enum {
	SYNC_STATE,
	RECEIVE_STATE,
	PROCESS_STATE,
	TRANSMIT_STATE
}PROTOCOLSTATES;

typedef enum {
	UUT_SYNC_STATE,
	UUT_TRANSMIT_STATE,
	UUT_RECEIVE_STATE,
	UUT_PROCESS_STATE,
	UUT_WAIT_STATE
}UUT_MODBUS_STATES;


typedef struct {

	uint8_t function_code;
	uint16_t read_start_address;
	uint16_t write_start_address;
	uint16_t read_registers;
	uint16_t write_registers;
	uint8_t read_byte_count;
	uint8_t write_byte_count;
	uint8_t data[UART_MSG_SIZE];
	uint8_t uut_data[UART_MSG_SIZE];

}RECORD;



typedef struct {

	uint16_t address;
	uint16_t *data;
	void (*callback)(void*);
}MODBUS_REG_MAP;

typedef struct {
	GPIO_TypeDef * port;
	uint16_t pin;
}GPIO_INTERFACE;


typedef enum {
	EEPROM_TEST_IN_PROGRESS,
	EEPROM_TEST_PASS,
	EEPROM_TEST_FAIL
}EEPROM_STATUS;

typedef enum {
	TEST_0,
	POWER_ON_TEST,
	EEPROM_TEST,
	DIGITAL_OUTPUTS,
	AC_OUTPUTS,
	KEYPAD_LED_TEST,
	NUM_TESTS
}TESTS;





// Function Prototypes
void PCModbusRTU(void);
void UUTModbusRTU(void);
void ReadInputs(void);
void SetOutputs(void);
void ClearOutputs(void);
void CaptureACInputs(void);
void update_led(int toggle);
void update_red_led(int toggle1);
void SetDigitalOutputs(void);
void SerialTransmit(void);
void UUTSerialTransmit(void);
void ProcessPCCommands(void);
void FillTransmitBuffer(uint8_t, uint8_t);
void set_clear_relay_output(void *data);
void set_clear_v33_output(void *data);
void set_clear_v33diff_output(void *data);
void set_clear_v24_source_output(void *data);
void set_clear_v24_sink_output(void *data);
void set_clear_uut_power(void *data);
void read_analog_data(void);
void change_uut_state(void *data);
void clear_uut_data(void *data);
void clear_pico_valve_data(void);
void get_software_hardware_versions(void);
void init_pcp_board(void);
void PCP_pulse_counter_test(void *data);
void Ultimus_Pressure_Sensor1_Test(void *data);
void Ultimus_Pressure_Sensor2_Test(void *data);

void update_analog_output(void*data);
void update_analog_output1(void *data);
void read_analog_data1(void);
void calculate_board_power(void);
uint16_t CRC16(unsigned char *puchMsg, uint16_t usDataLen);
void average_samples(uint32_t data, uint16_t channel);
void clear_pulse_counter(void *data);
void start_fall_time_test(void *data);
void start_rise_time_test(void *data);

#endif /* TYPEDEF_H_ */
