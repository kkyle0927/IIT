/*
 * robot_motionmap_vector.h
 *
 *  Created on: Oct 23, 2024
 *      Author: Eunyoung Youn
 */

#ifndef CM_BACKBONE_CONTENTS_FILE_MNGR_INC_ROBOT_MOTIONMAP_VECTOR_H_
#define CM_BACKBONE_CONTENTS_FILE_MNGR_INC_ROBOT_MOTIONMAP_VECTOR_H_


#pragma pack(push, 1)
typedef struct _C_Vector {
	uint8_t FF_gain;
	uint8_t PD_gain;
	uint8_t IC_gain;
	uint8_t DOB_gain;
	uint8_t IRC_gain;
	uint8_t FC_gain;
} C_Vector;

typedef struct _P_Vector {
	int16_t yd;  // desired position  (deg)
	uint16_t L;  // trajectory length (ms)
	uint8_t s0;  // acceleration      (deg/s^2)
	uint8_t sd;  // deceleration      (deg/s^2)
} P_Vector;

typedef struct _F_Vector {
	uint16_t mode_idx;     // (mode) F-vector Tp : 0.1 ~ 10
	int16_t  coefficient;
	uint16_t global_variable_index;
	uint16_t delay;
	uint16_t zero;        // Dummy data for reset
} F_Vector;

typedef struct _MD_F_Vector {
	uint16_t mode_idx;     // (mode) F-vector Tp : 0.1 ~ 10
	int16_t  tau_max;      // (100*Nm) F-vector Torque
	uint16_t delay;        // (ms) Inital F-vector Generation Delay
	uint16_t zero;        // Dummy data for reset
} MD_F_Vector;

typedef struct _I_Vector {
	uint8_t epsilon_target;     // Half width of the Corridor      (x10)
	uint8_t Kp_target;          // Magnitude of Virtual Spring
	uint8_t Kd_target;          // Magnitude of Virtual Damper
	uint8_t lambda_target;      // Impedance Ratio in the Corridor (x100)
	uint16_t duration;          // Duration for translation
} I_Vector;
#pragma pack(pop)


#endif /* CM_BACKBONE_CONTENTS_FILE_MNGR_INC_ROBOT_MOTIONMAP_VECTOR_H_ */
