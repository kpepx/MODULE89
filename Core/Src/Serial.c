// File name: Serial.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "Serial.h"
#include "STEPPER.h"
#include "TASKSPACE.h"
#include "TRAJECTORY.h"
#include "TRAJECTORY_CIRCLE.h"

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

//int16_t check_neg(int16_t w){
//	if(w >> 15 == 1){
//
//	}
//	return
//}

void Servo_gripperChess(int num, uint16_t value){
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

void Servo_StartStop(int num, uint16_t value1, uint16_t value2){
	serial_state * serial = &Serials[num];
	serial->length = 2;
	serial->instruction = WRITE_DATA;
	serial->address = START_STOP_MOVE;
	serial->parameter[0] = value1;
	serial->parameter[1] = value2;
	iWrite(serial);
	sendIPacket(serial);
}

void Feedback_Complete(int num, uint16_t value){
	serial_state * serial = &Serials[num];
	serial->length = 2;
	serial->instruction = STATUS;
	serial->address = FEEDBACK;
	serial->parameter[0] = SHIFT_TO_LSB(value);
	serial->parameter[1] = SHIFT_TO_MSB(value);
	iWrite(serial);
	sendIPacket(serial);
}

void Feedback_JOINT(int num, uint16_t q1, uint16_t q2, uint16_t q3, uint16_t q4){
	serial_state * serial = &Serials[num];
	serial->length = 8;
	serial->instruction = STATUS;
	serial->address = PRESENT_JOINT;
	serial->parameter[0] = SHIFT_TO_LSB(q1);
	serial->parameter[1] = SHIFT_TO_MSB(q1);
	serial->parameter[2] = SHIFT_TO_LSB(q2);
	serial->parameter[3] = SHIFT_TO_MSB(q2);
	serial->parameter[4] = SHIFT_TO_LSB(q3);
	serial->parameter[5] = SHIFT_TO_MSB(q3);
	serial->parameter[6] = SHIFT_TO_LSB(q4);
	serial->parameter[7] = SHIFT_TO_MSB(q4);
	iWrite(serial);
	sendIPacket(serial);
}

void Feedback_XYZ(int num, uint16_t x, uint16_t y, uint16_t z, uint16_t roll){
	serial_state * serial = &Serials[num];
	serial->length = 8;
	serial->instruction = STATUS;
	serial->address = PRESENT_XYZ;
	serial->parameter[0] = SHIFT_TO_LSB(x);
	serial->parameter[1] = SHIFT_TO_MSB(x);
	serial->parameter[2] = SHIFT_TO_LSB(y);
	serial->parameter[3] = SHIFT_TO_MSB(y);
	serial->parameter[4] = SHIFT_TO_LSB(z);
	serial->parameter[5] = SHIFT_TO_MSB(z);
	serial->parameter[6] = SHIFT_TO_LSB(roll);
	serial->parameter[7] = SHIFT_TO_MSB(roll);
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
//	HAL_UART_Transmit_DMA(serial->UART_NAME, &serial->iPacket, serial->iPacketLength);
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
						Servo_tragetPos(2, 0);
						Servo_gripperChess(2, 0);
						reset_trajectory(1);
						reset_trajectory(2);
						reset_trajectory(3);
						//servo home
						break;
					case START_STOP_MOVE:
						Stepper_StartStop(1, (serial->rPacket[6]<<8) + serial->rPacket[5]);
						Stepper_StartStop(2, (serial->rPacket[8]<<8) + serial->rPacket[7]);
						Stepper_StartStop(3, (serial->rPacket[10]<<8) + serial->rPacket[9]);
						Servo_StartStop(2, serial->rPacket[11], serial->rPacket[12]);
						break;
					case JOINT_MOVE:
						Stepper_SetTraget(1, ((float_t)(int16_t)((serial->rPacket[6]<<8) + serial->rPacket[5]))/100.00);
						Stepper_SetTraget(2, ((float_t)(int16_t)((serial->rPacket[8]<<8) + serial->rPacket[7]))/100.00);
						Stepper_SetTraget(3, ((float_t)(int16_t)((serial->rPacket[10]<<8) + serial->rPacket[9]))/100.00);
						Servo_tragetPos(2, (serial->rPacket[12]<<8) + serial->rPacket[11]);

//						updateJoint((serial->rPacket[6]<<8) + serial->rPacket[5], (serial->rPacket[8]<<8) + serial->rPacket[7], (serial->rPacket[10]<<8) + serial->rPacket[9], (serial->rPacket[12]<<8) + serial->rPacket[11]);
						break;
					case XYZ_MOVE:
						updateJoint((int16_t)((serial->rPacket[12]<<8) + serial->rPacket[11]), (int16_t)((serial->rPacket[6]<<8) + serial->rPacket[5]), (int16_t)((serial->rPacket[8]<<8) + serial->rPacket[7]), (int16_t)((serial->rPacket[10]<<8) + serial->rPacket[9]));
//						updateXYZ((serial->rPacket[6]<<8) + serial->rPacket[5], (serial->rPacket[8]<<8) + serial->rPacket[7], (serial->rPacket[10]<<8) + serial->rPacket[9]);
						break;
					case FIELD_CHESS:
						path((int16_t)((serial->rPacket[6]<<8) + serial->rPacket[5]), (int16_t)((serial->rPacket[8]<<8) + serial->rPacket[7]), (int16_t)((serial->rPacket[10]<<8) + serial->rPacket[9]));
//						update_circle((int16_t)((serial->rPacket[12]<<8) + serial->rPacket[11]), (int16_t)((serial->rPacket[6]<<8) + serial->rPacket[5]), (int16_t)((serial->rPacket[8]<<8) + serial->rPacket[7]), (int16_t)((serial->rPacket[10]<<8) + serial->rPacket[9]));
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
