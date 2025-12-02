/**
 * @file low_level_ctrl_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

#ifndef APPS_LOW_LEVEL_CTRL_INC_LOW_LEVEL_CTRL_H_
#define APPS_LOW_LEVEL_CTRL_INC_LOW_LEVEL_CTRL_H_

#include "module.h"

#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "ioif_adc_common.h"
#include "ioif_gpio_common.h"
#include "ioif_fdcan_common.h"
#include "ioif_wwdg_common.h"
#include "ioif_flash_common.h"
#include "ioif_tmcs1100a2.h"

#include "ioif_rmb20ic.h"
#include "ioif_hall_sensor.h"

#include "signal_generator.h"
#include "motor_controller.h"
#include "inverter.h"
#include "mid_level_ctrl.h"
#include "system_ctrl.h"
#include "gait_ctrl.h"

#include "data_object_common.h"
#include "data_object_interface.h"

#include "ioif_sysctrl.h"

#ifdef DAC_ENABLE
#include "dac.h"
#endif
/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define LOW_LEVEL_CONTROL_FREQUENCY      		25000
#define LOW_LEVEL_CONTROL_PERIOD      			0.00004

#ifdef L30_MD_REV06_ENABLED // TMCS1100A2
#define CURRENT_SENSOR_VOLTAGE      3.3
#define CURRENT_SENSOR_SENSITIVITY  0.100	// V/A
#define CURRENT_SENSOR_RANGE		29		// in Amphere +-14.5A
#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED // TMCS1100A2
#define CURRENT_SENSOR_VOLTAGE      3.3
#define CURRENT_SENSOR_SENSITIVITY  0.100	// V/A
#define CURRENT_SENSOR_RANGE		29		// in Amphere +-14.5A
#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED // TMCS1100A2 or WALKONSUIT Current Sensor
#define CURRENT_SENSOR_VOLTAGE      3.3
#define CURRENT_SENSOR_SENSITIVITY  0.100	// V/A
#define CURRENT_SENSOR_RANGE		29		// in Amphere +-14.5A
#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MD_ENABLED  // TMCS1100A3
#define CURRENT_SENSOR_VOLTAGE      3.3
#define VDC_GAIN_SCALE_ADJ			(1.12)
#define	BAT_VDC_FILTER_WEIGHT	(0.111) // 500Hz
#define	TIM1_RCR_CNT					(1) // 500Hz
#define OTP_DERATE_NORMAL_RATE		(1000.0)	// 100%
#ifdef IOIF_TMCS1100A3_ENABLED
#define CURRENT_SENSOR_SENSITIVITY  0.200 	// V/A
#define CURRENT_SENSOR_RANGE		14.5	// in Amphere +-7.25A
#endif
#ifdef IOIF_TMCS1100A2_ENABLED
#define CURRENT_SENSOR_SENSITIVITY  0.100 	// V/A
#define CURRENT_SENSOR_RANGE		29		// in Amphere +-14.5A
#endif
#endif /* SUIT_MD_ENABLED */

#define ADC1_RESOLUTION	65536
#define CURRENT_CTRL_PERFORMANCE_CHECK_CNT	2000

#define FRICTION_LUT_SIZE	8193

#define SIZE_OF_SECTION_TIME 8

#define PI_2					(6.2832)
#define IF_BW					(1000 * (PI_2))
#define IF_KP					(0.000439 * IF_BW)
#define IF_KI					(0.450 * IF_BW)
#define IF_WR_REF				(8.0)
#define IF_RAMP_CNT				(10)

#define ACTUAL_CURRENT_REF 		(2.0)
#define FORCED_ALIGNED_TIME_SEC (2)

#define SEND_MSG_RETRY_COUNT_MAX 	10

#define STALL_PROTECTION_CURR_0	(7)
#define STALL_PROTECTION_CURR_1	(8)
#define STALL_PROTECTION_CURR_2	(9)

#define STALL_CRNT_DETECT	(STALL_PROTECTION_CURR_0)
#define STALL_SET_TIME		(3 * LOW_LEVEL_CONTROL_FREQUENCY)

#define MAX_POS_LIMIT_ANGLE_SPEC (110.0 / 180.0 * M_PI)
#define MAX_NEG_LIMIT_ANGLE_SPEC (-40.0 / 180.0 * M_PI)

#define MAX_LIMIT_PROTECTION_AGNLE	(6.0 / 180.0 * M_PI)

// 5deg between SET and READY
#define MAX_POS_LIMIT_SET_ANGLE	(MAX_POS_LIMIT_ANGLE_SPEC - MAX_LIMIT_PROTECTION_AGNLE)
#define MAX_POS_LIMIT_READY_ANGLE (MAX_POS_LIMIT_ANGLE_SPEC - (11.0 / 180.0 * M_PI))

#define MAX_NEG_LIMIT_SET_ANGLE	(MAX_NEG_LIMIT_ANGLE_SPEC + MAX_LIMIT_PROTECTION_AGNLE)
#define MAX_NEG_LIMIT_READY_ANGLE (MAX_NEG_LIMIT_ANGLE_SPEC - (-11.0 / 180.0 * M_PI))

#define ANGLE_LIMIT_CTRL_BW		(2 * M_PI * 0.1)
#define POS_LIMIT_MODE		(2)
#define NEG_LIMIT_MODE		(1)
#define ANGLE_LIMIT_CLEAR	(0)

#define POS_SENSOR_VALIDATION_PRD 	(0.004) // 4ms
#define POS_SENSOR_VALIDATION_CNT 	(LOW_LEVEL_CONTROL_FREQUENCY * POS_SENSOR_VALIDATION_PRD)

#define ABS1_SPD_TO_ELEC_SPD 		(10)
#define SPD_TO_MECH_WR				(2 * M_PI / POS_SENSOR_VALIDATION_PRD / 8192)

#define ENC_WR_LOWER_LIMIT				(IF_WR_REF * ACTUATOR_GEAR_RATIO * 0.6)
#define ENC_WR_UPPER_LIMIT				(IF_WR_REF * ACTUATOR_GEAR_RATIO * 1.4)

#define THETA_COMP_COEFF			(8192.0 / 14.0)

//#define STALL_CRNT_DETECT	(0.01)
//#define STALL_SET_TIME		(1 * LOW_LEVEL_CONTROL_FREQUENCY)

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/* Commutation Feedback Sensor */
typedef enum _SensorCommutation {
	e_Commute_Sensor_Inc_Encoder = (uint8_t)0,
	e_Commute_Sensor_Hall,
	e_Commute_Sensor_Inc_Encoder_Hall
} SensorCommutation;

/* Position Feedback Sensor */
typedef enum _SensorPosFeedback {
	e_Pos_Sensor_None = (uint8_t)0,
	e_Pos_Sensor_Inc_Encoder,
	e_Pos_Sensor_Abs_Encoder1,
	e_Pos_Sensor_Abs_Encoder2
} SensorPosFeedback;

/* Electrical Angle Homing Sensor */
typedef enum _SensorElecAngleHoming{
	e_EHoming_Sensor_Forced= (uint8_t)0,
	e_EHoming_Sensor_Hall,
	e_EHoming_Sensor_Abs_Encoder1,
	e_EHoming_Sensor_Abs_Encoder2
} SensorElecAngleHoming;

/* Mechanical Angle Homing Sensor*/
typedef enum _SensorMechAngleHoming{
	e_MHoming_Sensor_Zero= (uint8_t)0,  // default: position(t=0) = 0;
	e_MHoming_Sensor_Abs_Encoder1,      //          position(t=0) = abs_encoder1_position(t=0);
	e_MHoming_Sensor_Abs_Encoder2,      //          position(t=0) = abs_encoder2_position(t=0);
	e_MHoming_Sensor_MD_Calculation,    //          position(t=0) = MD_calculated_value;
	e_MHoming_Sensor_CM_Setting,        //          position(t=0) = CM_setting_value;
} SensorMechAngleHoming;

typedef struct _MotorProperties {
	uint8_t name_length;
	char name[31];

	uint8_t hall_sensor_table[6]; // calibrated hall sensor table (CCW)

	float J; // moment of inertia
	float B; // friction coefficient
	float a, b, c, d;

	float Vbus;     // bus voltage (V)
	float R; 		// resistance (ohm)
	float L; 		// inductance (mH)
	uint8_t pole_pair;
	float Ke; // bEMF constant (V/kRPM)
	float Kt; // torque constant (Nm/A)
	float gear_ratio;
} MotorProperties;

typedef struct _SensorSetting {
	SensorMechAngleHoming m_angle_homing_sensor;
	SensorElecAngleHoming e_angle_homing_sensor;
	SensorPosFeedback pos_feedback_sensor;
	SensorCommutation commutation_sensor;

	uint8_t incremetnal_encoder_usage;
	uint8_t absolute_encoder1_usage;
	uint8_t absolute_encoder2_usage;
	uint8_t hall_sensor_usage;

	uint8_t temperature_sensor_usage;
	uint8_t imu_6axis_usage;
	uint8_t imu_3axis_usage;

} SensorSetting;

typedef struct _MotorSetting {
	DOPI_OpMode_t 		mode_of_operation;
	SensorSetting       sensor_setting;
	CommutationSetting  commutation_set;
	ElecAngleHoming     elec_angle_homing;
	DOPI_InInfo_t 		input_info;

	uint8_t low_level_kalman_on;
	float currCtrl_BW_radPsec;
	float bemf_id_vel;
	float peakCurr_limit;
	float contCurr_limit;
	float max_velocity_rpm;
	float max_velocity_radPsec;		//in rad/s
	float bemf_id_gain;
	uint8_t bemf_id_gain_pctg;
} MotorSetting;

typedef struct _MotorOut {
	uint16_t elec_angle;
	uint16_t actual_elec_angle;
    int32_t I_U, I_V, I_W;                  		// 3-phase & low-side universal current
    int32_t I_U_offset, I_V_offset, I_W_offset;  	// ADC current calibration offset
    int32_t V_U, V_V, V_W;
    int32_t V_U_offset, V_V_offset, V_W_offset;
    uint32_t crnt_offset_done;
    float I_mag;
    float I_U_KF, I_V_KF, I_W_KF;
	float current_act;
	float position, position_f;
	float velocity, velocity_f;
	float velocity_raw;
} MotorOut;

typedef struct _MotorIn {
    float V_U_input, V_V_input, V_W_input;

    /* From msg_hdlr_task */
	float auxiliary_input;

	/* From low_level_ctrl_task */
	float low_id_process_input;
	float friction_input;

	/* From mid_level_ctrl_task */
	float f_vector_input; // no feedback

	float mid_id_process_input;
	float irc_input;
	float analysis_input;
	float mid_ctrl_input;		// (imp_ctrl_input + mid_pid_ctrl_input + f_vector_input + ff_input + dob_input)

	/* Sum of all of inputs */
	float total_current_input;
	float batt_vdc;
	float batt_vdc_old;
	float batt_vdc_filt;

} MotorIn;

typedef struct _MotorElecSystemID {

	uint8_t flag;
	uint32_t cnt;
	uint16_t input;
	uint16_t signal;
	int32_t f_signal;
	int32_t output;
	float level;

//	int32_t input_arr[CHIRP_ARRAY_SIZE];
//	int32_t output_arr[CHIRP_ARRAY_SIZE];

} MotorElecSystemID;

typedef struct _Advanced_Friction_ID {
	float adv_friction_compensator_LUT[FRICTION_LUT_SIZE];

	float friction_cnt;
	uint8_t friction_index;

	float friction_ref;

	float velRefEval;	// for evaluation velocity ref

	float max_vel1;
	float max_vel2;
	float vel_num1;
	float vel_num2;
	float duration;  // id time per single velocity reference (unit: s)
	float section_time[SIZE_OF_SECTION_TIME];
	uint32_t time_per_vel;
	uint32_t cut_time;

	float gap1;
	float gap2;

    float P_gain;
    float I_gain;
	float gainScalingFactor;

    float e;
    float e_sum;

	uint8_t state;

	float scaling_factor; //for compensation

	float lut_mdl;
	float lut_p1;
	float lut_p2;
	float lut_p3;
	float lut_p4;
	float lut_p5;
	float lut_d_mu_v;
	float lut_epsilon;
	float gain;
} Advanced_Friction_ID;

typedef enum _FrictionModel {
	BASIC_MODEL = (uint8_t)1,
	STRIBECK_MODEL,
	KARNO_MODEL
} FrictionModel;

typedef struct IF_SpdCtrl{
	uint16_t clk_timer;
	uint32_t sec_timer;
	int32_t elec_hz_target_step;
	int32_t elec_hz_target_ramp;
	uint16_t elec_angle;
	uint32_t aging_time;
	int32_t StallCnt;
	float scaled_current_ref;
	float current_add;
	uint8_t state;
	int32_t position;
	int32_t position_old;
	float speed;
} IF_SpdCtrl_t;

typedef struct OtpDerate{
	int32_t setTime;
	int16_t clrTime;
	uint16_t state;
	uint16_t rampCnt;
	float DerateTarget;
	float DerateRamp;
	float 	 OtplimitRate;
} OtpDerate_t;
typedef enum OtpDerateState{
	NORMAL_STATE,
	DERATE_LV1_STATE,
	DERATE_LV2_STATE,
	DERATE_LV3_STATE,
	DERATE_LV4_STATE,
	DERATE_LV5_STATE_FOR_STALL
} OtpDerateState_t;

typedef struct SetStall {
	int8_t sector;
	int8_t sector_old;
	int8_t sector_hysH;
	int8_t sector_hysL;
	uint8_t activated;
	int32_t stall_tmr;
	float curr_tgt;
}SetStall_t;

typedef struct MaxAngleCtrl {
	float in;
	float ref;
	float err;
	float Wr;
	float out;
	float comp;
	float outLimit;
	float Kp;
	float Kd;
	float Wc;
	uint8_t enable;
	float curr_ref;
	uint8_t safe_on;
	uint16_t rec_cnt;
	uint8_t temp_off;
	float dec_ref;
}MaxAngleCtrl_t;

typedef struct encCheck{
	uint16_t abs1Old;
	uint16_t abs2Old;
	int32_t	 inc1Old;

	int16_t abs1Delt;
	int16_t abs2Delt;
	int32_t	 inc1Delt;

	float abs1Spd;
	float abs2Spd;
	float inc1Spd;

	uint16_t	abs1_errCnt;
	uint16_t abs2_errCnt;
	uint16_t inc1_errCnt;
	uint8_t faultSet_bit;

	uint16_t faultCnt;
	uint16_t tmr;
}encCheck_t;

typedef struct thetaComp {
	int8_t 		pp_region;
	uint16_t 	absTheta[MOTOR_POLE_PAIR_INT];
	uint16_t 	incTheta[MOTOR_POLE_PAIR_INT];
	float		arc_length[MOTOR_POLE_PAIR_INT];
	int32_t		boundary[MOTOR_POLE_PAIR_INT];
	uint16_t	eTheta;
	uint16_t	theta_input;
	uint16_t	theta_input_old;
	int32_t		eTheta_delta;
	int32_t		eTheta_sum;
	uint8_t     flag;
	uint8_t		flag_old;
	uint8_t		testCnt;
}thetaComp_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskObj_t 			low_level_ctrl_task;
extern MotorProperties 		motor_properties;
extern MotorSetting			motor_setting;
extern MotorIn 				motor_in;
extern MotorOut				motor_out;
extern Advanced_Friction_ID advanced_friction_id;
extern IOIF_IncEnc_t 		inc25KhzObj;
extern IOIF_HallSensor_t 	hallObj;
extern KFObject				kf_current_object;
extern OtpDerate_t 			OtpDerateCtrl;
extern SetStall_t			StallDetect;


extern IF_SpdCtrl_t			AgingCtrl;
extern MESSetting_t			MES;
/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Cal_Elec_System_ID_Batch(void);
void Send_Elec_BEMF_Value(void);
void Send_Elec_Bandwidth_Data(void);
void Send_Advanced_Friction_ID_Data(void);
void Cal_Friction_LUT(void);
void Send_Elec_Angle_Value(void);
void Send_Aging_Test_Result(void);
void GetMES();
void ImportMES_HWFW();
//void ImportMES_Code();

void InitLowLvCtrl();
void RunLowLvCtrl(void* params);


#endif /* APPS_LOW_LEVEL_CTRL_INC_LOW_LEVEL_CTRL_H_ */
