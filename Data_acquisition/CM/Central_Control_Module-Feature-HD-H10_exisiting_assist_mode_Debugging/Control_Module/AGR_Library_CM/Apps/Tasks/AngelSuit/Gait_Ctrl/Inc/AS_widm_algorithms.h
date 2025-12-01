#ifndef GAIT_CTRL_INC_WIDM_ALGORITHMS_H_
#define GAIT_CTRL_INC_WIDM_ALGORITHMS_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define WIDM_CONTROL_PERIOD				0.001
#define WIDM_PI							3.141592653589793
#define FC_LPF_ALPHA					0.008				// LPF smoothing factor
#define FUZZY_INPUT_LPF_ALPHA			0.008
#define FUZZY_LOGIC_GRAVITY_CONSTANT 	9.81 				// (m/s^2)
#define FUZZY_LOGIC_VARIANCE_NUM		4					// Fuzzy Logic Variances
#define FUZZY_INPUT_SCALING_FACTOR		8					// experimently


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration of WIDM attachment cases
 */
typedef enum _WIDM_AttachCase_t {
	WIDM_H10_RIGHT_SAG,
	WIDM_H10_LEFT_SAG,

	WIDM_K10_RIGHT_SAG,
	WIDM_K10_LEFT_SAG,

	WIDM_SUIT_CM_FRONTAL,

	WIDM_ATTACH_CASE_NUM
} WIDM_AttachCase_t;

/**
 * @brief Enumeration of WIDM module (IMU, IMUABS)
 */
typedef enum _WIDM_Module_t {
	WIDM_H10_SAM,
	WIDM_K10_SAM,
	WIDM_A10_SAM,
	WIDM_SUIT_CM,
	WIDM_MODULE_NUM
} WIDM_Module_t;

/**
 * @brief Structure to save (current & previous) IMU sensor data
 */
typedef struct _WIDM_SensorData_t {
	/* Auto-Calibration Sensor Parameters */
	// Gyroscope Parameter Scaling coefiicient, Sensor Bias, Mean Value
	float PhiMatrixSum[3][3];
	float invPhiMatrix[3][3];
    float ThetaVectorSum[3];
	float phi[3];
	float degGyrIntegral[2];
	float scaling_gyro;
	float bias_gyro;
	float meanVal_gyro;
	float degAccArctan;
	float matrixDeterminant;
	float GyrCalOutput[3];
    
	// Accelrometer Parameter mean Value(near 9.81)
	float m0Raw;
	float m0RawSum;
	float sqrtAcc;
	float accMeanVal_calibrated;

	float accX[2];	// [0] is current value, [1] is previous value (Accelerometer X axis) (m/s^2)
	float accY[2];	// [0] is current value, [1] is previous value (Accelerometer Y axis) (m/s^2)
	float accZ[2];	// [0] is current value, [1] is previous value (Accelerometer Z axis) (m/s^2)

	float gyrX[2];	// [0] is current value, [1] is previous value (Gyroscope X axis)
	float gyrY[2];	// [0] is current value, [1] is previous value (Gyroscope Y axis)
	float gyrZ[2];	// [0] is current value, [1] is previous value (Gyroscope Z axis)

	float accX_calibrated[2];	// calibrated
	float accY_calibrated[2];	// calibrated
	float accZ_calibrated[2];	// calibrated
	float gyrX_calibrated[2];	// calibrated
	float gyrY_calibrated[2];	// calibrated
	float gyrZ_calibrated[2];	// calibrated

	float accGlobalX;
	float accGlobalY;
	float accGlobalZ;
	float gyrGlobalX;
	float gyrGlobalY;
	float gyrGlobalZ;

	/* Gyration */
	float gyrationLPF[2];
	uint8_t isInitialized_gyrationLPF;

	/* Axis Align */
	float gyroXAligned;
	float accXAligned;

	/* Detection Noise */
	float accXLastValidData;
	float accYLastValidData;
	float accZLastValidData;

	float gyrXLastValidData;
	float gyrYLastValidData;
	float gyrZLastValidData;

	float accXCalibLastValidData;
	float accYCalibLastValidData;
	float accZCalibLastValidData;

	float gyrXCalibLastValidData;
	float gyrYCalibLastValidData;
	float gyrZCalibLastValidData;

	uint32_t accXNoiseCnt;
	uint32_t accYNoiseCnt;
	uint32_t accZNoiseCnt;

	uint32_t gyrXNoiseCnt;
	uint32_t gyrYNoiseCnt;
	uint32_t gyrZNoiseCnt;

	uint32_t accXCalibNoiseCnt;
	uint32_t accYCalibNoiseCnt;
	uint32_t accZCalibNoiseCnt;

	uint32_t gyrXCalibNoiseCnt;
	uint32_t gyrYCalibNoiseCnt;
	uint32_t gyrZCalibNoiseCnt;

	uint8_t accXAlgorithmActivated;
	uint8_t accYAlgorithmActivated;
	uint8_t accZAlgorithmActivated;
	
	uint8_t gyrXAlgorithmActivated;
	uint8_t gyrYAlgorithmActivated;
	uint8_t gyrZAlgorithmActivated;

	uint8_t accXCalibAlgorithmActivated;
	uint8_t accYCalibAlgorithmActivated;
	uint8_t accZCalibAlgorithmActivated;

	uint8_t gyrXCalibAlgorithmActivated;
	uint8_t gyrYCalibAlgorithmActivated;
	uint8_t gyrZCalibAlgorithmActivated;
} WIDM_SensorData_t;

/**
 * @brief Structure to hold (Angle & Angular velocity) data
 */
typedef struct _WIDM_AngleData_t {
	float thighDegBPF[3];
	float thighDegBPFprevInput[3];
	float thighDegBPFprevOutput[3];

	float thighVelBPF[3];
	float thighVelBPFprevInput[3];
	float thighVelBPFprevOutput[3];

	float peakAmpThighDeg;
	float peakAmpThighVel;

	float peakAmpSmoothThighDeg;
	float peakAmpSmoothThighVel;

	float thighDegNorm;		// Normalization
	float thighVelNorm;		// Normalization

	float thighDegUnbiased[2];	// Eliminated Bias(thighDegLPF - NeutralPos)

	float thighDegLPF[2];
	float thighVelLPF[2];		// First Thigh Vel LPF
	float thighVelLPF2[2];		// Second Thigh Vel LPF

	float thighDegBPFAbs;
	float thighVelBPFAbs;

	float thighDegLPFAbs;
	float thighVelLPFAbs;

	uint8_t isInitialized_DegBPF;
	uint8_t isInitialized_VelBPF;

	uint8_t isInitialized_DegLPF;
	uint8_t isInitialized_VelLPF;
	uint8_t isInitialized_VelLPF2;

	float NeutralPosture;
	float ThighRawDeg;
	float NeutralPostureBias;
	float NeutralPostureBiasOffset;
	
	float degTvcf[2];					// [0] is current value, [1] is previous value (Angel(deg) through TVCF)
	float velDegTvcf[2];				// [0] is current value, [1] is previous value (Angular velocity(deg/s))

	float degTvcf_calib[2];
	float velDegTvcf_calib[2];
	
	float thighDegFinal;				// (deg)
	float thighVelDegFinal;				// (deg/s)
	float degAccInitValue;				// For LPF
	float degAcc_1stLPF[2];				// Angle through LPF(Acc)
	float degGyr_Integral_1stHPF[2];	// Angle through HPF(Gyro)
	float degAcc_arctan[2];				// actan(accX, accY) (deg)
	float degAcc_arctan_Raw;
	float degGyr_Integral[2];			// Just Integral no HPF
	float angleTVCF_Deg;				// result of calculation TVCF (deg)

	/* After Auto-Calibration */
	float thighDegFinal_calib[2];
	float thighVelDegFinal_calib[2];
	float thighVelDiffINC;
	float thighVelDiff;
	float degAcc_calib_1stLPF[2];
	float degAcc_calib_1stLPF_LPF[2];
	float degGyr_calb_Integral_1stHPF[2];
	float degGyr_calb_Integral_1stHPF_LPF[2];
	float degGyr_Integral_calibrated[2];
	float angleTVCF_calib_Deg;
	float angleTVCF_calib_Deg_LPF;

	float wcHPF;
	float wcHPF_LPF;

	float gyration[2];
	float forwardVel[2];

	float initOffsetIMUDeg;				// for H10 IMU(TVCF) Thigh Degree Offset(tilted degree)
	float initOffsetAbsDeg;				// For Incremental Encoder Init Offset Degree
	float degAbsOffset;					// Abs Encoder Offset for Incremental Encoder Angle
	float degAbsRaw;					// Applied Gear Ratio, 0 ~ +360
	float degAbsFinal;					// -180 ~ +180

	float degINC;
	float velDegINC;
	float velDiffINC;

	float degIMUINC;
	float velDegIMUINC;
	float velDiffIMUINC;

	float degIMUINC_calib;
	float velDegIMUINC_calib;
} WIDM_AngleData_t;


/**
 * @brief Structure to hold Fuzzy Logic parameters
 */
typedef struct _WIDM_FuzzyData_t {
    float fuzzyInput[FUZZY_LOGIC_VARIANCE_NUM];     // Acc, Jerk, Gyro, Wdot(measurement value)
    float var[FUZZY_LOGIC_VARIANCE_NUM];            // Acc, Jerk, Gyro, Wdot-variance(initially set velue)
    float mu;                                       // Update mu for TVCF cutoff frequency(wc)
    float m0;                                      	// Threshold Value (maybe middle value)
    float sensitivity;								// Sensor Sensitivity (natural logarithm)
    float xbar;                                     // Fuzzy Logic Relational Expressions

	float fc;										// fc Cut off Frequency (Hz)
	float fc_LPF[2];								// fc Cut off Frequency with LPF
    float fc_low;									// Low Frequency
    float fc_high;									// High Frequency

    float wc;                                       // Wc Cut off Frequency (rad/s)

    float mk;										// fuzzyInput[0]+fuzzyInput[2]
    float mk_LPF[2];								// fuzzyInput[0]+fuzzyInput[2] LPF
	float scaling_mk;
	
    uint8_t isInitialized_fc;                       // init prev = current

	float gyroSuppress;
	float SqrtSumData;

	/* After Auto-Calibration */
	float fuzzyInput_calib[FUZZY_LOGIC_VARIANCE_NUM];
	float mk_calib;
	float mk_calib_LPF[2];
	float fc_calib_LPF[2];
	float xbar_calib;
	float mu_calib;
	float fc_calib;
	uint8_t isInitialized_calib_fc;
	
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
	float degOriLPF[2];
	float velOriLPF[2];
	float degOri;			// Center point location of elliptical graph before normalization
	float velOri;			// Center point location of elliptical graph before normalization

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
	uint32_t gaitLoopCnt;
	uint32_t gaitLoopCntPrev;

	float gaitPhase;			// Current Gait Phase 0 ~ 100%
	float gaitPhasePrev;		// Previous Gait Phase

	uint8_t gaitMode[2];		// mode = 1 : Walking, mode = 0 : Stop
	float modeCheck;

	uint16_t gaitPeriod;		// Gait Period (ms) < 2000ms
	uint8_t firstPeriodCheck;

	float gaitCutoffFreq;
	float gaitCutoffFreqLPFSmooth;

	uint8_t firstStepDone;		// 1 : Not Walk Started, 0 : Walking Start
	uint8_t isWalking;			// 1 : Walking, 0: Not Walking
	uint32_t lastStepTime;

	uint32_t gaitCount;			// Gait Count

	uint8_t	walkingState;
} WIDM_GaitData_t;

////////////////////////////////////////////////////////
/* Band-Pass & Low-Pass Filter State Structure Refactoring By HD */
typedef struct _WIDM_BPFState_t {
    bool isInitialized;		// Indicates if the filter is initialized, 초기화 상태를 명확하게 나타내기 위해 변경.
    double prevOutput[3];	// Stores the previous three outputs for feedback calculation, 이전 출력 값을 저장하는 배열.
    double prevInput[3];	// Stores the previous three inputs for feedforward calculation, 이전 입력 값을 저장하는 배열.
} WIDM_BPFState;

typedef struct _WIDM_BPFState_Deg_t {
    bool isInitialized;		// Indicates if the filter is initialized, 초기화 상태를 명확하게 나타내기 위해 변경.
    double prevOutput[3];	// Stores the previous three outputs for feedback calculation, 이전 출력 값을 저장하는 배열.
    double prevInput[3];	// Stores the previous three inputs for feedforward calculation, 이전 입력 값을 저장하는 배열.
} WIDM_BPFState_Deg;

typedef struct _WIDM_BPFState_Vel_t {
    bool isInitialized;		// Indicates if the filter is initialized, 초기화 상태를 명확하게 나타내기 위해 변경.
    double prevOutput[3];	// Stores the previous three outputs for feedback calculation, 이전 출력 값을 저장하는 배열.
    double prevInput[3];	// Stores the previous three inputs for feedforward calculation, 이전 입력 값을 저장하는 배열.
} WIDM_BPFState_Vel;

typedef struct _WIDM_LPFState_t {
    bool isInitialized;  // Indicates if the filter has been initialized
    double prevOutput;   // Stores the previous output for feedback
} WIDM_LPFState;


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

float WIDM_SquareRootSum(float x, float y);
float WIDM_AbsFloat(float value);
float WIDM_Derivative(float currentVal, float previousVal);

float WIDM_GetMaxValue(float x, float y);
float WIDM_GetMinValue(float x, float y);

// Filters //
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

/* For SUIT series by HyundoKim */
/* Band Pass Filters */
float UpdateBPF_ForPeakThighDeg(uint8_t* isInitialized, float* prevOutput, float* prevInput, float currentInput, float peakFreq);
float UpdateBPF_ForPeakThighVel(uint8_t* isInitialized, float* prevOutput, float* prevInput, float currentInput, float peakFreq);
/* Low Pass Filters */
float UpdateLPF(uint8_t* isInitialized, float* prevOutput, float currentInput, float alpha);


#endif /* SUIT_MINICM_ENABLED */

#endif /* GAIT_CTRL_INC_WIDM_ALGORITHMS_H_ */