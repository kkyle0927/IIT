

#ifndef APPS_LOW_LEVEL_CTRL_COMMUTATION_CTRL_INC_INVERTER_H_
#define APPS_LOW_LEVEL_CTRL_COMMUTATION_CTRL_INC_INVERTER_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "module.h"

#include "signal_generator.h"
#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"

#include "common_constants.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define ELECTRICAL_ANGLE_RESOLUTION		65536

#define DUTY_RESOLUTION	4800
#define MAX_DUTY 		4799
#define MIN_DUTY 		0

#define	VECTOR1			0				// 0 degrees
#define	VECTOR2			0x2aaa			// 60 degrees
#define	VECTOR3			0x5555			// 120 degrees
#define	VECTOR4			0x8000			// 180 degrees
#define	VECTOR5			0xaaaa			// 240 degrees
#define	VECTOR6			0xd555			// 300 degrees
#define	SIXTY_DEG		0x2aaa          // 60 degrees
#define VOLTS_LIMIT		28300			//28300 ( = 32768 * sqrt(3)/2)

#ifdef SUIT_MD_ENABLED
/* SVM Code */
#define TIM1_MD_PWM_MAX_CNT	(4800)
#define DPWM_MODE_SVM		(0)
#define DPWM_MODE_MAX		(1)
#define DPWM_MODE_MIN		(2)
#define MD_DPWM_MODE		(DPWM_MODE_SVM)

#define CRNT_HW_SCALE		((ADC1_RESOLUTION) / (CURRENT_SENSOR_RANGE)) // DR/A
#define VDC_HW_SCALE		(VDC_ADC_SCALE)

#define SQRT_3				(56756)		// Q15 - Complete
#define INV_SQRT_3			(0.5774)
#define SIN00				(0)			// Q15
#define SIN60				(28378)		// Q15
#define COS00				(32767)		// Q15
#define COS60				(16384)		// Q15

#define ACTUATOR_GEAR_RATIO 	(18.75)
#define MOTOR_POLE_PAIR 		(14.0)
#define MOTOR_POLE_PAIR_INT 	(14U)
#endif
/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/*************** (Clarke Transform Typedef) ***************/
/* Clarke Transform Input */
typedef struct _ClarkeIn {
	int32_t Iu; // The phase U current
	int32_t Iv; // The phase V current
	int32_t Iw;	// The phase W current
} ClarkeIn;

/* Clarke Transform Output */
typedef struct _ClarkeOut {
	int32_t Ia;
	int32_t Ib;
} ClarkeOut;

/* Inverse Clarke Transform Input */
typedef struct _InvClarkeIn {
	int32_t Va;
	int32_t Vb;
} InvClarkeIn;

/* Inverse Clarke Transform Output */
typedef struct _InvClarkeOut {
	int32_t Vu;
	int32_t Vv;
	int32_t Vw;
} InvClarkeOut;

/*************** (Park Transform Typedef) ***************/
/* Park Transform Input */
typedef struct _ParkIn {
	int32_t Ia;
	int32_t Ib;
	uint16_t theta_e; // electrical angle (radian)
} ParkIn;

/* Park Transform Input */
typedef struct _ParkOut {
	int32_t Id;
	int32_t Iq;
} ParkOut;

/* Inverse Park Transform Input */
typedef struct _InvParkIn {
	int32_t Vd;
	int32_t Vq;
	uint16_t theta_e; // electrical angle (radian)
} InvParkIn;

/* Inverse Park Transform Output */
typedef struct _InvParkOut {
	int32_t Va;
	int32_t Vb;
	float V_mag;
} InvParkOut;

/*************** (Phasor Typedef) ***************/
typedef struct _Phasor{
	int32_t magnitude;
	uint16_t phase;
} Phasor;

typedef struct _TrapezoidalControl {
	uint16_t duty_resolution;
	uint16_t duty;
	uint8_t hall_sensor_step;
	uint8_t direction;
} TrapezoidalControl;

typedef struct _CommutationSetting{

	/*-------------------------------(User Setting Parameters)-------------------------------*/

	/* Parameters Setting on GUI */
	uint16_t max_duty;                     // maximum voltage for forced-commutation
	int8_t  user_desired_dir;              // user set direction on GUI   (+1/-1)
	uint8_t avg_sample_num;
	/* Parameters Setting on FW */
	float duration1;                       // forced commutation 1 step increase period (unit: s)
	float duration2;                       // forced commutation 1 step constant period (unit: s)

	/*----------------------------------(Output Parameters)----------------------------------*/

	/* Direction Parameters for FOC commutation/current control/pos&vel control */
	int8_t cc_dir;                   // current control direction   (+1/-1)
	int8_t ma_dir;                   // current control direction   (+1/-1)
	int8_t ea_dir;                   // electrical angle direction  (+1/-1)

	/* Parameters for 6-step commutation & Hall-based electrical angle homing */
	uint8_t hall_sensor_table[6];    // hall_sensor_table
	int8_t  hall_sensor_dir;         // hall sensor-based direction (+1/-1)

	/* Parameters for absolute encoder-based electrical angle homing */
	int32_t abs_encoder_offset;


	/*-------------------------------(Temporal Variables/Flag)-------------------------------*/
	uint8_t state;                   // state: 0 -> 1 -> 2 -> 3
	int8_t comm_step;               // step:  0 -> 1 -> 2 -> 3 -> 4 -> 5
	uint32_t time_stamp;             // 25usec counter
	int32_t initial_cnt;             // save encoder initial count
	uint8_t done;                    // setting done: 1
	uint8_t start;                   // if commutation setting is enabled: +1, else -1;
	uint32_t abs_encoder_cnt;
	uint16_t duty;
	uint32_t run_cnt;
	float duty_gap;
	uint8_t init_abs_angle_done;
	int32_t raw[MOTOR_POLE_PAIR_INT*2];
	int32_t raw_elecAngle[MOTOR_POLE_PAIR_INT*2];
	float raw_avg[MOTOR_POLE_PAIR_INT*2];
	float std_dev;
	uint16_t raw_mechAngle[MOTOR_POLE_PAIR_INT*2];
	uint16_t raw_abs[MOTOR_POLE_PAIR_INT*2];
	uint8_t compensation_enable;
} CommutationSetting;

typedef struct _ElecAngleHoming{

	uint8_t               done;      // if electrical angle homing is done: 1, else: 0
	uint16_t              offset;    // electrical angle offset

	/* forced homing parameters */
	uint32_t              forced_homing_cnt;
	float                 forced_homing_dy;
	uint16_t              forced_homing_duty;
	uint32_t              forced_homing_duration; // (sec * 25000, ex: 1sec = 25000)

}ElecAngleHoming;



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void GateDriver_ONOFF(uint8_t t_mode);
void Stop_PWM(void);

/* ------------------- PHASE EXCITATION ------------------- */

void excitate_phase_UVnWn(uint16_t t_duty);  // excite UV'W' (= 0  deg)
void excitate_phase_UVn(uint16_t t_duty);    // excite UV'   (= 30 deg)
void excitate_phase_UVWn(uint16_t t_duty);   // excite UVW'  (= 60 deg)
void excitate_phase_VWn(uint16_t t_duty);    // excite VW'   (= 90 deg)
void excitate_phase_UnVWn(uint16_t t_duty);  // excite U'VW' (= 120 deg)
void excitate_phase_UnV(uint16_t t_duty);    // excite U'V   (= 150 deg)
void excitate_phase_UnVW(uint16_t t_duty);   // excite U'VW  (= 180 deg)
void excitate_phase_UnW(uint16_t t_duty);    // excite U'VW  (= 210 deg)
void excitate_phase_UnVnW(uint16_t t_duty);  // excite U'V'W (= 240 deg)
void excitate_phase_VnW(uint16_t t_duty);    // excite V'W   (= 270 deg)
void excitate_phase_UVnW(uint16_t t_duty);   // excite UV'W  (= 300 deg)
void excitate_phase_UWn(uint16_t t_duty);    // excite UW'   (= 330 deg)

void excitate_phase_UnVnWn(void);            // excite U'V'W'

/* ------------------- CLARK & PARK TF ------------------- */
void ClarkeTransform(const ClarkeIn *t_clarke_in, ClarkeOut *t_clarke_out);
void ParkTransform(const ParkIn *t_park_in, ParkOut *t_park_out);
void InvParkTransform(const InvParkIn *t_invpark_in, InvParkOut *t_invpark_out);
void InvParkInputToPhasor(InvParkIn *t_invpark_in, Phasor *t_phasor);

/* ------------------- SPACE VECTOR MODULARIZATION ------------------- */
#ifdef _USE_HW_OVER_REV06
void SVM_FOC(InvParkOut* pInvParkOut, float ulVdc, float* t_v_in_U, float* t_v_in_V, float* t_v_in_W);
#else
void SVM(int t_volts, uint16_t t_angle, float* t_v_in_U, float* t_v_in_V, float* t_v_in_W);
#endif//_USE_HW_OVER_REV06

#endif /* APPS_LOW_LEVEL_CTRL_COMMUTATION_CTRL_INC_INVERTER_H_ */
