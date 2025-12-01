/**
 * @file mid_level_ctrl_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

#ifndef APPS_MID_LEVEL_CTRL_INC_MID_LEVEL_CTRL_H_
#define APPS_MID_LEVEL_CTRL_INC_MID_LEVEL_CTRL_H_

#include "task_mngr.h"

#include "signal_generator.h"

#include "motor_controller.h"

#include "ioif_rmb20ic.h"
#include "ioif_rmb20sc.h"

#include "low_level_ctrl.h"
#include "msg_hdlr.h"

#include "data_object_common.h"

#include "gait_ctrl.h"
#include "ext_dev_ctrl.h"
/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MID_LEVEL_CONTROL_FREQUENCY     1000
#define MID_LEVEL_CONTROL_PERIOD      	0.001
#define PRBS_ARR_SIZE					8191
#define SBS_FREQUENCY_MAX				50
#define SBS_FREQUENCY_MIN				0.5
#define SBS_FREQUENCY_STEP_NUM			20
#define SBS_REPEAT_NUM					20

#define P_VECTOR_BUFF_SIZE			    20
#define F_VECTOR_BUFF_SIZE		        10
#define I_VECTOR_BUFF_SIZE			    10

#define F_MODE_NUM                 		10

#define CONTROLLER_TRANSITION_DURATION  500 // unit: ms

#define ACTUATOR_MAX_WR_OUT			(30.0)
#define ABS1_MAX_DEGREE			((ACTUATOR_MAX_WR_OUT * 180.0 / MID_LEVEL_CONTROL_FREQUENCY / M_PI) * 1.875)

#define min(x, y) (x) < (y) ? (x) : (y)
#define max(x, y) (x) > (y) ? (x) : (y)

#define MID_LV_ARRAY_MAX			(240)
#define MID_LV_ARRAY_FOR_ENCODER	(640)
//#define TORQUE_SENSOR_ADC2KGFCM_SCALE		(65536.0 / 3.3 * (0.065 * 0.8 / 10 * 3.3)) // 10V -> 3.3V, Scale 0.65 * 80% // 1kgfcm
#define TORQUE_SENSOR_ADC2KGFCM_SCALE		(2924.05) // 10V -> 3.3V, Scale 0.65 * 80% // 1kgfcm
#define LOAD_MOTOR_RESOLUTION				(1.8 / 30.0 / 4.0) // 30 - GearRatio, 4 - pre-scaler
#define LOAD_MOTOR_RES2ELEC_2PI				((360.0 / 18.75 / 14.0)/LOAD_MOTOR_RESOLUTION + 0.5) // if 183, err - 0.28125deg / 360deg
#define TORQUE_STABLE_TIME					(250)
#define TORQUE_MEASUREMENT_NUMBER			(100)
#define UNIFORMITY_MEASUREMENT_NUMBER		(300)
#define UNIFORMITY_SAVE_DATA_NUM			(32)

#define BACKDRIVERBILITY_WARM_UP_CNT		(400)
#define BACKDRIVERBILITY_RECORD_ARC_CNT		(12000)
#define BACKDRIVERBILITY_RECORD_START_CNT	(12400)
#define BACKDRIVERBILITY_SAMP_NUM			(50)
#define BACKDRIVERBILITY_RECORD_NUM			(BACKDRIVERBILITY_RECORD_ARC_CNT / BACKDRIVERBILITY_SAMP_NUM)

//#define TORQUE_STABLE_TIME					(100)
//#define TORQUE_MEASUREMENT_NUMBER			(100)

//#define TORQUE_CHECK_CRNT_MIN	(1.0)
#define TORQUE_CHECK_CRNT_MIN	(0.5)
#define TORQUE_CHECK_CRNT_INTV	(0.25)
#define TORQUE_CHECK_CRNT_MAX	(4.0)
//#define TORQUE_CHECK_TRY_NUM_HALF 		(7)
//#define TORQUE_CHECK_TRY_NUM_FULL 		(14)
#define TORQUE_CHECK_REPEAT_NUM 		(2)

#define TORQUE_CHECK_TRY_NUM_HALF 		(13)
#define TORQUE_CHECK_TRY_NUM_FULL 		(26)

#define LOAD_MOTOR_DUTY					(9600)
#define STEP_MOTOR_RPP					(0.0002618)
#define LOAD_MOTOR_SPD_INIT				(0.2)
#define TIM_ARR_SPD_COEFF				((240E+6) * STEP_MOTOR_RPP)

//#define UNIFORMITY_SAMP_NUM				(40)
#define UNIFORMITY_SAMP_NUM				(80)

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct _PeriodicSignal {
	float amp;
	float freq;
} PeriodicSignal;

typedef struct _VirtualSpringDamper {
	float lower_stiffness, upper_stiffness;
	float lower_damper, upper_damper;

	float lower_limit, upper_limit;
	float lower_damped_range, upper_damped_range;
	float lower_stiff_range, upper_stiff_range;

	float lower_damper_origin, upper_damper_origin;
	float lower_spring_origin, upper_spring_origin;

	float control_input;
	float saturation;
} VirtualSpringDamper;

typedef struct _MidLevelState {
	float initial_pos;
	float position;    // unit: rad
	float position_f;
	float position_ff;

	float position_offset;
	float init_position;

	float velocity_raw;
	float velocity_final;
} MidLevelState;

typedef struct _System_ID_SBS {
	float fmin;
	float fmax;
	float N_iter;
	float N_samples;

	float amp;
	float offset;

	float *f_samples;

	float current_f;
	uint32_t f_cnt;
	uint32_t sys_id_cnt;

	float filtered_input;
	float filtered_output;
	uint8_t done;

	float verify_mag;
} System_ID_SBS;

typedef struct _Backlash_Test{

	float range;
	float amplitude;

} Backlash_Test;

/************************ P Vector ************************/
typedef struct _P_Vector{

	int16_t yd;  // desired position  (deg)
	uint16_t L;  // trajectory length (ms)
	uint8_t s0;  // acceleration      (deg/s^2)
	uint8_t sd;  // deceleration      (deg/s^2)

} P_Vector;

typedef struct _P_Vector_Decoder{

	P_Vector p_buffer[P_VECTOR_BUFF_SIZE];
	uint8_t N;   // (=BufferCount) Number of Trajectory in Buffer
	uint8_t ON;  // (=TrajectoryON)

	float y0;    // initial position

	double a0, a2, a3, a4 ,a5; // coefficient of k'th step trajectory
	double b0, b2, b3, b4 ,b5; // coefficient of k+1'th step trajectory

	double t1;
	double t2;
	double t3;
	double t4;
	double t5;

	double L;
	double L_inv;

	uint16_t count;

	float yd_f; // yd(k-1)

	uint8_t durationCompleted;

} P_Vector_Decoder;



/************************ F Vector ************************/
typedef struct _F_Vector{
	uint8_t  mode_idx;     // mode
	int16_t  tau_max;      // 100*Nm
	uint16_t delay;        // delay (ms)

	float tau, tau_old1, tau_old2, u, u_old1, u_old2;
	uint32_t t_end;
	uint32_t time_stamp;
	uint8_t is_full;       // if full 1, else 0
} F_Vector;


typedef struct _F_Mode_Param{

	double wn, b0, b1, b2, a0, a1, a2;
	float  tp;

} F_Mode_Param;

typedef struct _F_Vector_Decoder {

	F_Vector f_buffer[F_VECTOR_BUFF_SIZE];
	F_Mode_Param mode_param[F_MODE_NUM];

	uint8_t temp_idx;
	float   tp[F_MODE_NUM]; // Tp to Mode Table

	float input;
} F_Vector_Decoder;

/************************ I Vector ************************/
typedef struct _I_Vector{

	uint8_t epsilon_target;     // Half width of the Corridor      (x10)
	uint8_t Kp_target;          // Magnitude of Virtual Spring
	uint8_t Kd_target;          // Magnitude of Virtual Damper
	uint8_t lambda_target;      // Impedance Ratio in the Corridor (x100)
	uint16_t duration;          // Duration for translation

} I_Vector;

typedef struct _ImpedanceCtrl {

	I_Vector i_buffer[I_VECTOR_BUFF_SIZE];
	I_Vector opt1_i_buffer;

	/* Parameters */
	float epsilon, Kp, Kd, lambda;

	float Kp_max; // Kp @ 100%
	float Kd_max; // Kd @ 100%

	float gap_epsilon;
	float gap_Kp;
	float gap_Kd;
	float gap_lambda;

	float e;  // error
	float ef; // output of error function

	float ef_f;
	float ef_diff;

	float control_input;  // control input

	float L;

	uint8_t option;
	uint16_t i; // 1ms counter
	uint8_t ON; // flag
	uint8_t N;  // buffer count

} ImpedanceCtrl;

typedef struct _VelocityComp {
	float velocity_comp_torque;
	float f_velocity_comp_torque;

	float velocity_gain;
	float velocity_threshold;
	float velocity_satu;
	float velocity_alpha;
} VelocityComp;

typedef struct _GravityComp {
	uint8_t move_stop_sw;
	uint8_t keep_move;
	float grav_comp_torque;
	float f_grav_comp_torque;

	float grav_gain;
	float grav_alpha;
} GravityComp;

typedef struct _DegreeLimit {
	float HipRoMUpperLimit;
	float HipRoMLowerLimit;
	float KneeRoMUpperLimit;
	float KneeRoMLowerLimit;

	uint8_t JointRoMLimitON;
	float VirtualSpringCoeff;
	float VirtualDampingCoeff;

	float degree_limit_torque;
} DegreeLimit;

typedef struct _VelocityLimit {
	float HipExtensionVelLimit;
	float HipFlexionVelLimit;
	float KneeVelLimit;

	float VirtualDampingCoeff;
	uint8_t JointVelocityLimitON;

	float alpha;
	float velo_limit_torque;
	float f_velo_limit_torque;

} VelocityLimit;

typedef enum {
    SUIT_IDLE = 0,
	SUIT_SMART_ASSIST = 1,
	SUIT_AQUA = 2,
	SUIT_SPACE = 3,
	SUIT_DEGREE_VELOCITY_LIMit = 4,
} SuitMode;

typedef struct Abs1Checker{
	float		offset;
	uint32_t	raw_offset;
	uint16_t	errCnt;
}Abs1Checker_t;

typedef enum {
	LM_ENABLE,
	LM_DISABLE,
} LoadMotorEn_t;

typedef enum {
	LM_CW,
	LM_CCW,
} LoadMotorDir_t;

typedef struct AcuatorValidation{
	int32_t Adc_offset;
	uint32_t tmr;
	uint32_t tmr2;
	int32_t Adc_value;
	float torque;
	uint8_t step_num;
	uint8_t try_num;
	uint8_t pp_cnt;
	uint8_t repeat_cnt;
	float coeff_R_pos;
	float coeff_R_neg;
	float avg1;
	float avg2;
	float torque_StDev1;
	float torque_StDev2;
	float torque_StDev_rate1;
	float torque_StDev_rate2;
	float min;
	float max;
	uint8_t spdCtrl_start;
	uint16_t lowlevel_skipCnt;
	uint16_t lowlevel_skipNum;
	uint16_t lowlevel_skipNum_old;
	uint8_t lowlevel_flag;
	int16_t encAngle;
	int16_t encAngle_old;
	int32_t encAngle_sum;
	double realAngle_sum;
	float a_p;
	float b_p;
	float a_n;
	float b_n;
	double Wr;
}AcuatorValidation_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskObj_t mid_level_ctrl_task;
extern float mid_ctrl_saturation;
extern IOIF_AbsEnc_t AbsObj1;
extern IOIF_AbsEnc_t AbsObj2;
extern PIDObject velCtrl;
extern VelocityEstObject veObj;
extern PIDObject posCtrl;
extern DOBObject posDOB;
extern FFObject	posFF;
extern VirtualSpringDamper VSD;
extern ImpedanceCtrl impedanceCtrl;
extern IOIF_IncEnc_t inc1KhzObj;
extern ImpedanceReductionCtrl IRC;
extern MidLevelState mid_level_state;
extern float motor_actual_current;
extern P_Vector_Decoder pvectorObj;
extern F_Vector_Decoder fvectorObj;
extern AcuatorValidation_t actuator_checker;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

/* GUI */
void Tune_Gain(void);
void Set_Velo_Estimator(void);

void InitMidLvCtrl(void);
void RunMidLvCtrl(void* params);

void Send_Torque_Accuracy_Result(void);
void Send_Torque_Uniformity_Result(void);
void Send_Actuator_backdriverbility_Result(void);
void Send_Encoder_Linearity_Result(void);

#endif /* APPS_MID_LEVEL_CTRL_INC_MID_LEVEL_CTRL_H_ */
