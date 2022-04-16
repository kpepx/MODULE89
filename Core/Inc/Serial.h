// File name: Serial.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

/////   DATA FRAME  /////
// HEADER1  HEADER2  LEN   INSTRUCTION  ERROR  ADDRESS   DATA 1  ...  DATA N   Check
//  0xFF      0xFD  0-255  instruction  error  address   0-4095       0-4095    sum

#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32h7xx_hal.h"

#define NUM_SERIAL 3

// PACKET DATA
#define HEADER1	               0xFF
#define HEADER2	               0xFD

// INSTRUCTION
#define READ_DATA              0x01
#define WRITE_DATA             0x02
#define FACTORY_RESET          0x03
#define STATUS                 0x55

// ERROR
#define RESULT_FAIL            0x01
#define INSTRUCTION_ERROR      0x02
#define SUM_ERROR              0x03
#define DATA_RANGE_ERROR       0x04
#define DATA_LENGTH_ERROR      0x05
#define DATA_LIMIT_ERROR       0x06
#define ACCESS_ERROR           0x07

// CONTROL
#define NAME                   0x01
#define HOME_OFFSET            0X02
#define MIN_POSITION_LIMIT     0x03
#define MAX_POSITION_LIMIT     0x04
#define MIN_SPEED_LIMIT        0x05
#define MAX_SPEED_LIMIT        0x06

// ADDRESS
#define HOME_CONFIGULATION     0x10
#define START_STOP_MOVE        0x11
#define JOINT_MOVE             0x12
#define XYZ_MOVE               0x13
#define GRIP_CHESS             0x14
#define PRESENT_JOINT          0x15
#define PRESENT_XYZ            0x16
#define FIELD_MOVE             0x17

//Servo Board
#define SERVO_JOINT			   0x20

typedef struct{
	//number
	int number;

	UART_HandleTypeDef *UART_NAME;

	uint8_t rxBuffer[15];
	uint8_t txBuffer[15];

	uint8_t iPacket[15];	//Instruction Packet //not optimal size?
	uint8_t rPacket[15];	//Return Packet //not optimal size?

	uint8_t parameter[15];	//Parameter for Instruction //not optimal size?

	uint8_t length;
	uint8_t address;
	uint8_t instruction;
	uint8_t iPacketLength;

	uint8_t state;

}serial_state;

void Serial_Setup(int num, UART_HandleTypeDef *uart);

uint8_t Cal_sum(serial_state * serial);

uint8_t SHIFT_TO_LSB(uint16_t w);

uint8_t SHIFT_TO_MSB(uint16_t w);

void Servo_gripperChess(int num, uint16_t value);

void Servo_tragetPos(int num, uint16_t pos);

void Servo_StartStop(int num, uint16_t value);

void iRead(serial_state * serial);

void iWrite(serial_state * serial);

void sendIPacket(serial_state * serial);

void data_in(int num);

void selectPacket(int num);


#endif /* __SERIAL_H */

