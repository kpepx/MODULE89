// File name: CARTESIAN.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "CARTESIAN.h"
#include "STEPPER.h"

static cartesian_state cartesians[NUM_CARTESIAN];

void updateXYZ(int32_t joint1, int32_t joint2, int32_t joint3, int32_t joint4){
	cartesian_state * cartesian = &cartesians[0];
	cartesian->X = 0;
	cartesian->Y = 0;
	cartesian->Z = 0;
}

void updateJoint(int32_t x, int32_t y, int32_t z){
	cartesian_state * cartesian = &cartesians[0];
	cartesian->X = x;
	cartesian->Y = y;
	cartesian->Z = z;

}
