// File name: IK.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "IK.h"

static ik_state iks[NUM_IK];

void updateIK(int32_t x, int32_t y, int32_t z, int32_t r){
	fk_state * ik = &iks[0];
	ik->q1 = x+y+z+r;
	ik->q2 = x+y+z+r;
	ik->q3 = x+y+z+r;
	ik->q4 = x+y+z+r;
}
