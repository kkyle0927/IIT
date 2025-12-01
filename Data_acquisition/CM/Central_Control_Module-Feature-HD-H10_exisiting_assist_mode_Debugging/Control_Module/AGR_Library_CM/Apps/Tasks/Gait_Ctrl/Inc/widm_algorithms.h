/*
 * widm_algorithm.h
 *
 *  Created on: Oct 11, 2023
 *      Author: INVINCIBLENESS
 */

#ifndef GAIT_CTRL_INC_WIDM_ALGORITHMS_H_
#define GAIT_CTRL_INC_WIDM_ALGORITHMS_H_

#include <stdint.h>
#include <math.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define WIDM_CONTROL_PERIOD		0.001
#define WIDM_PI					3.141592653589793


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration of walking state
 */
typedef enum _WIDM_WalkingState_t {
	WIDM_STOP,
	WIDM_WALKING_START,
	WIDM_WALKING_HALF_CYCLE,
	WIDM_WALKING_ONE_CYCLE
} WIDM_WalkingState_t;

/**
 * @brief Enumeration of WIDM attachment cases
 */
typedef enum _WIDM_AttachCase_t {
	WIDM_LEFT_SAGITAL,
	WIDM_LEFT_SAGITAL_DEMO,
	WIDM_CM_TEST,
	WIDM_LEFT_U5,
	WIDM_H10_LEFT,
	WIDM_K10_LEFT,
	WIDM_LEFT_CASE7,
	WIDM_LEFT_CASE8,

	WIDM_RIGHT_SAGITAL,
	WIDM_RIGHT_U5,
	WIDM_H10_RIGHT,
	WIDM_K10_RIGHT,
	WIDM_RIGHT_CASE5,
	WIDM_RIGHT_CASE6,
	WIDM_RIGHT_CASE7,
	WIDM_RIGHT_CASE8,
} WIDM_AttachCase_t;

/**
 * @brief Enumeration of WIDM module (IMU, IMUABS)
 */
typedef enum _WIDM_Module_t {
	WIDM_IMU_U5,
	WIDM_IMU_SAM,
	WIDM_IMUABS_SAM,
	WIDM_IMUINC_SAM,
} WIDM_Module_t;


#pragma pack(push, 1)
/**
 * @brief Structure to save (current & previous) IMU sensor data
 */
typedef struct _WIDM_SensorData_t {
	float accX[2];			// [0] is current value, [1] is previous value (Accelerometer X axis)
	float accY[2];
	float gyrZ[2];			// [0] is current value, [1] is previous value (Gyroscope Z axis)
} WIDM_SensorData_t;

/**
 * @brief Structure to hold (Angle & Angular velocity) data
 */
typedef struct _WIDM_AngleData_t {
	float initAngle; 			// Initial angle(degree) will be set maybe 0

	float initAngleIMU;
	float initAngleABS;

	float degABS;
	float velABS;

	float degINC;
	float degINCPrev;
	float velINC;

	float degAcc;				// Angle using Accelerometer measurements
	float degGyr;				// Angle using Gyroscope measurements

	float degIMUABS;
	float velIMUABS;

	float degIMUINC;
	float velIMUINC;


	float degFinal;
	float velFinal;

	float degAccFiltered;		// Angle through LPF(Acc)
	float degGyrFiltered;		// Angle through HPF(Gyro)
	float degTvcfFiltered;		// Angle through TVCF

	float velRaw[2];			// [0] is current value, [1] is previous value (Angular velocity)
	float degTvcf[2];			// [0] is current value, [1] is previous value (Angel(degree) through TVCF)
	float degLPF1st[2];			// [0] is current value, [1] is previous value (Angle(degree) through 1st order LPF)
	float degLPF2nd[2];			// [0] is current value, [1] is previous value (Angle(degree) through 2nd order LPF)
	float velLPF1st[2];			// [0] is current value, [1] is previous value (Angular velocity through 1st order LPF)
	float velLPF2nd[2];			// [0] is current value, [1] is previous value (Angular velocity through 2nd order LPF)
} WIDM_AngleData_t;

/**
 * @brief Structure to hold Fuzzy Logic parameters
 */
typedef struct _WIDM_FuzzyData_t {
	float fuzzyInput[4];	// Acc, Jerk, Gyro, Wdot(measurement value)
	float wc;				// Cut off Frequency
	float wl;				// Low Frequency
	float wh;				// High Frequency
	float var[4];			// Acc, Jerk, Gyro, Wdot-variance(initially set velue)
} WIDM_FuzzyData_t;

/**
 * @brief Structure to hold Threshold values(Start/Stop) of (angle & angular velocity)
 */
typedef struct _WIDM_ThresData_t {
	float degThStart;			// Angle Threshold (Gait Phase Start)
	float velThStart;			// Angular Velocity Threshold (Gait Phase Stop)
	float degThStop;			// Angle Threshold (Gait Phase Start)
	float velThStop;			// Angular Velocity Threshold (Gait Phase Stop)
} WIDM_ThresData_t;

/**
 * @brief Structure to execute the Normalization of gait phase graph
 */
typedef struct  _WIDM_NormData_t {
	float degOri;			// Center point location of elliptical graph before normalization
	float velOri;			// Center point location of elliptical graph before normalization
	float degOriLPF[2];
	float velOriLPF[2];

	float sumDeg;			// Sum of angle for calculating deg_o
	float sumVel;			// Sum of angular velocity for calculating vel_o

	float degMax;			// Max of angle in elliptical plot
	float degMin;			// Min of angle in elliptical plot
	float velMax;			// Max of angular velocity in elliptical plot
	float velMin;			// Min of angular velocity in elliptical plot

	float degNorm;			// Current angle value on circle after normalization
	float velNorm;			// Current angular velocity value on circle after normalization

	float ampDeg;			// Amplitude of angle of an elliptical graph before normalization
	float ampVel;			// Amplitude of angular velocity of an elliptical graph before normalization
	float ampDegLPF[2];
	float ampVelLPF[2];

	uint16_t sumIter;		// Sum of number(gait phase 50%) for calculating deg_o, vel_o
} WIDM_NormData_t;

/**
 * @brief Structure to save (gait period & gait phase)
 */
typedef struct _WIDM_GaitData_t {
	float gaitPhase;			// Current Gait Phase 0 ~ 100%
	float gaitPhasePre;			// Previous Gait Phase
	uint16_t gaitPeriod;		// Gait Period (ms) < 2000ms
	uint8_t	walkingState;
} WIDM_GaitData_t;
#pragma pack(pop)

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
void WIDM_UpdateBuffer(WIDM_SensorData_t* sensorData, WIDM_AngleData_t* angleData, WIDM_GaitData_t* gaitData, WIDM_NormData_t* normData);
float WIDM_SquareRootSum(float x, float y);
float WIDM_AbsoluteValue(float value);
float WIDM_Derivative(float currentVal, float previousVal);
void WIDM_CalculateMu(float fuzzyVar, float fuzzyInput, float* mu);
void WIDM_CalculateFuzzyInput(WIDM_SensorData_t* sensorData, WIDM_FuzzyData_t* fuzzyData);
float WIDM_CalculateFuzzyWc(WIDM_FuzzyData_t* fuzzyData);
float WIDM_LPF(float currAngle, float filteredAnglePrev, float cutoffFreq, float samplingPeriod);
float WIDM_HPF(float currAngle, float filteredAnglePrev, float cutoffFreq, float samplingPeriod);
void WIDM_RunTVCF(WIDM_SensorData_t* sensorData, WIDM_AngleData_t* angleData, float cutoffFreq, float samplingPeriod, WIDM_AttachCase_t attachCase);
float WIDM_GetMaxValue(float x, float y);
float WIDM_GetMinValue(float x, float y);
void WIDM_1stHalfGaitCycle(WIDM_NormData_t* normData, WIDM_GaitData_t* gaitData);
void WIDM_2ndHalfGaitCycle(WIDM_NormData_t* normData, WIDM_AngleData_t* angleData);
void WIDM_Normalization(WIDM_AngleData_t* angleData, WIDM_NormData_t* normData, WIDM_GaitData_t* gaitData);
float WIDM_GetGaitPhase(WIDM_NormData_t* normData, WIDM_GaitData_t* gaitData);
float WIDM_AttachCaseSetting(WIDM_SensorData_t* sensorData, WIDM_AttachCase_t attachCase);

// Prof //
double WIDM_BPF_walking_Prof(double r);
double WIDM_BPF_Peak_Prof_1(double r, double w);
double WIDM_BPF_Peak_Prof_2(double r, double w);
double WIDM_Abs_double(double value);
float WIDM_Abs_float(float value);
double WIDM_GetMaxValue_double(double x, double y);
double WIDM_LPF_walking_Prof(double r);

/* For SUIT series - MiniCM */
double WIDM_BPF_Peak_Prof_1_RH(double r, double w);
double WIDM_BPF_Peak_Prof_1_LH(double r, double w);
double WIDM_BPF_Peak_Prof_2_RH(double r, double w);
double WIDM_BPF_Peak_Prof_2_LH(double r, double w);
double WIDM_LPF_walking_Prof_RH(double r);
double WIDM_LPF_walking_Prof_LH(double r);

double WIDM_LPF_1NE(double r, double w);


#endif /* GAIT_CTRL_INC_WIDM_ALGORITHMS_H_ */
