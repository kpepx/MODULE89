// File name: Serial.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "Serial.h"
#include "STEPPER.h"
#include "TASKSPACE.h"

serial_state Serials[NUM_SERIAL];

void Serial_Setup(int num, UART_HandleTypeDef *uart){
	serial_state * serial = &Serials[num];
	serial->number = num;
	serial->UART_NAME = uart;
	HAL_UART_Receive_DMA(serial->UART_NAME, &serial->rPacket, 14);
}

uint8_t Cal_sum(serial_state * serial){
	uint16_t sum = 0;
	for(int i = 0;i<serial->length+5;i++){
		sum += serial->iPacket[i];
	}
	return ~(sum & 0xFF);
}

uint8_t SHIFT_TO_LSB(uint16_t w){
	return w & 0x00ff;
}

uint8_t SHIFT_TO_MSB(uint16_t w){
	return (w >> 8) & 0x00ff;
}

void Servo_gripperChess(int num, uint8_t value){
	serial_state * serial = &Serials[num];
	serial->length = 2;
	serial->instruction = WRITE_DATA;
	serial->address = GRIP_CHESS;
	serial->parameter[0] = SHIFT_TO_LSB(value);
	serial->parameter[1] = SHIFT_TO_MSB(value);
	iWrite(serial);
	sendIPacket(serial);
}

void Servo_tragetPos(int num, uint16_t pos){
	serial_state * serial = &Serials[num];
	serial->length = 2;
	serial->instruction = WRITE_DATA;
	serial->address = SERVO_JOINT;
	serial->parameter[0] = SHIFT_TO_LSB(pos);
	serial->parameter[1] = SHIFT_TO_MSB(pos);
	iWrite(serial);
	sendIPacket(serial);
}

void Servo_StartStop(int num, uint16_t value){
	serial_state * serial = &Serials[num];
	serial->length = 2;
	serial->instruction = WRITE_DATA;
	serial->address = START_STOP_MOVE;
	serial->parameter[0] = SHIFT_TO_LSB(value);
	serial->parameter[1] = SHIFT_TO_MSB(value);
	iWrite(serial);
	sendIPacket(serial);
}

void iRead(serial_state * serial){
	serial->iPacket[0] = HEADER1;
	serial->iPacket[1] = HEADER2;
	serial->iPacket[2] = 3+serial->length;
	serial->iPacket[3] = serial->instruction;
	serial->iPacket[4] = serial->address;
	serial->iPacket[5] = 0x00;
	serial->iPacket[6] = 0x00;
	serial->iPacket[7] = Cal_sum(serial);

	serial->iPacketLength = 8;
}

void iWrite(serial_state * serial){
	int i;
	serial->iPacket[0] = HEADER1;
	serial->iPacket[1] = HEADER2;
	serial->iPacket[2] = 3+serial->length;
	serial->iPacket[3] = serial->instruction;
	serial->iPacket[4] = serial->address;
	for(i = 5; i < 5+serial->length ; i++)
	{
		serial->iPacket[i] = serial->parameter[i-5]; //Instruction Packet's parameter field 2 and so on (write data n-th byte)
	}
	serial->iPacket[i] = Cal_sum(serial);

	serial->iPacketLength = i+1;
}

void sendIPacket(serial_state * serial){
	HAL_UART_Transmit(serial->UART_NAME, &serial->iPacket, serial->iPacketLength, 10);
}

//void getRPacket(int num){
//	serial_state * serial = &Serials[num];
//	HAL_UART_Transmit(serial->UART_NAME, &serial->rPacket, 14, 5);
//}

void data_in(int num){
	serial_state * serial = &Serials[num];
	serial->state = 1;
}

void selectPacket(int num){
	serial_state * serial = &Serials[num];
	if(serial->state){
		if(serial->rPacket[0] == HEADER1 && serial->rPacket[1] == HEADER2){
			switch (serial->rPacket[3]) {
			case READ_DATA:
				switch (serial->rPacket[4]) {
				case PRESENT_JOINT:

					break;
				case PRESENT_XYZ:

					break;
				default:
					break;
				}
				break;
				case WRITE_DATA:
					switch (serial->rPacket[4]) {
					case HOME_OFFSET:
						break;
					case MIN_POSITION_LIMIT:
						Stepper_SetMinPosition(1, (serial->rPacket[6]<<8) + serial->rPacket[5]);
						Stepper_SetMinPosition(2, (serial->rPacket[8]<<8) + serial->rPacket[7]);
						Stepper_SetMinPosition(3, (serial->rPacket[10]<<8) + serial->rPacket[9]);
						break;
					case MAX_POSITION_LIMIT:
						Stepper_SetMaxPosition(1, (serial->rPacket[6]<<8) + serial->rPacket[5]);
						Stepper_SetMaxPosition(2, (serial->rPacket[8]<<8) + serial->rPacket[7]);
						Stepper_SetMaxPosition(3, (serial->rPacket[10]<<8) + serial->rPacket[9]);
						break;
					case MIN_SPEED_LIMIT:
						Stepper_SetMinSpeed(1, (serial->rPacket[6]<<8) + serial->rPacket[5]);
						Stepper_SetMinSpeed(2, (serial->rPacket[8]<<8) + serial->rPacket[7]);
						Stepper_SetMinSpeed(3, (serial->rPacket[10]<<8) + serial->rPacket[9]);
						break;
					case MAX_SPEED_LIMIT:
						Stepper_SetMaxSpeed(1, (serial->rPacket[6]<<8) + serial->rPacket[5]);
						Stepper_SetMaxSpeed(2, (serial->rPacket[8]<<8) + serial->rPacket[7]);
						Stepper_SetMaxSpeed(3, (serial->rPacket[10]<<8) + serial->rPacket[9]);
						break;
					case HOME_CONFIGULATION:
						Stepper_SetHome(1, 0, (serial->rPacket[6]<<8) + serial->rPacket[5]);
						Stepper_SetHome(2, 0, (serial->rPacket[8]<<8) + serial->rPacket[7]);
						Stepper_SetHome(3, 0, (serial->rPacket[10]<<8) + serial->rPacket[9]);
						//servo home
						break;
					case START_STOP_MOVE:
						Stepper_StartStop(1, (serial->rPacket[6]<<8) + serial->rPacket[5]);
						Stepper_StartStop(2, (serial->rPacket[8]<<8) + serial->rPacket[7]);
						Stepper_StartStop(3, (serial->rPacket[10]<<8) + serial->rPacket[9]);
						Servo_StartStop(2, (serial->rPacket[12]<<8) + serial->rPacket[11]);
						break;
					case JOINT_MOVE:
						Stepper_SetTraget(1, ((float_t)(serial->rPacket[6]<<8) + serial->rPacket[5])/100.00);
						Stepper_SetTraget(2, ((float_t)(serial->rPacket[8]<<8) + serial->rPacket[7])/100.00);
						Stepper_SetTraget(3, ((float_t)(serial->rPacket[10]<<8) + serial->rPacket[9])/100.00);
						Servo_tragetPos(2, (serial->rPacket[12]<<8) + serial->rPacket[11]);

//						updateJoint((serial->rPacket[6]<<8) + serial->rPacket[5], (serial->rPacket[8]<<8) + serial->rPacket[7], (serial->rPacket[10]<<8) + serial->rPacket[9], (serial->rPacket[12]<<8) + serial->rPacket[11]);
						break;
					case XYZ_MOVE:
						updateJoint((serial->rPacket[12]<<8) + serial->rPacket[11], (serial->rPacket[6]<<8) + serial->rPacket[5], (serial->rPacket[8]<<8) + serial->rPacket[7], (serial->rPacket[10]<<8) + serial->rPacket[9]);
//						updateXYZ((serial->rPacket[6]<<8) + serial->rPacket[5], (serial->rPacket[8]<<8) + serial->rPacket[7], (serial->rPacket[10]<<8) + serial->rPacket[9]);
						break;
					case GRIP_CHESS:
						Servo_gripperChess(2, (serial->rPacket[12]<<8) + serial->rPacket[11]);
						break;
					default:
						break;
					}
					break;
					case FACTORY_RESET:

						break;
					default:
						break;
			}
		}
		serial->state = 0;
	}
}
