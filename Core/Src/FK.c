// File name: FK.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "FK.h"

static fk_state fks[NUM_FK];

void updateFK(int32_t joint1, int32_t joint2, int32_t joint3, int32_t joint4){
	fk_state * fk = &fks[0];
	fk->X = joint1+joint2+joint3+joint4;
	fk->Y = joint1+joint2+joint3+joint4;
	fk->Z = joint1+joint2+joint3+joint4;
	fk->R = joint4;
}
