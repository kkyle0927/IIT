#ifndef APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_AS_GAIT_CTRL_H_
#define APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_AS_GAIT_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include "data_object_common.h"
#include "data_object_dictionaries.h"

#include "AS_dev_mngr.h"
#include "AS_widm_algorithms.h"
#include "AS_imu_ctrl.h"
#include "AS_whole_body_ctrl.h"
#include "AS_ISI.h"

#include "GaitAnalysis.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define READ_FLASH_ARRAY_SIZE 8 // 32 Byte Aligned

#define ISI_B_FLAG_MAX_NUM          500

#define ACC_GRAVITY_CONSTANT 	    9.81    // (m/s^2)

/* For Gait Function */
#define LPF_SMOOTHING_FACTOR    0.02
#define THIGH_DEG_FIRST_LPF_SMOOTHING_FACTOR    0.02
#define THIGH_VEL_FIRST_LPF_SMOOTHING_FACTOR    0.02
#define THIGH_VEL_SECOND_LPF_SMOOTHING_FACTOR   0.0003

#define THIGH_VEL_THRESHOLD         15.0
#define MODE_CHECK_EMA_ALPHA        0.007
#define MODE_CHECK_THRESHOLD_HIGH   10
#define MODE_CHECK_THRESHOLD_LOW    1

#define GAIT_PERIOD_MAX             2000
#define GAIT_PERIOD_MIN             200

#define GAIT_PHASE_MAX              98
#define GAIT_PHASE_MIN              2

#define GAIT_CONTROL_PERIOD         0.001   // 1ms period

#define GAIT_CUTOFF_EMA_ALPHA       0.008
#define PEAK_AMP_DEG_EMA_ALPHA      0.002
#define PEAK_AMP_VEL_EMA_ALPHA      0.002
#define NEUTRAL_POS_EMA_ALPHA       0.01

#define STAND_STOP_DEG_THRESHOLD    20      // (deg) Thigh Degree at Stop & Standing
#define THIGH_ANGLE_WALK_THRESHOLD  10000   // Threshold for thigh angle to determine walking
#define MAX_STEP_DURATION           1100    // Maximum duration between steps to consider continuous walking

/* For Raw Sensor Data Error Check */
#define ACC_X_THRESHOLD_CONSTANT    62  // (8g = 8 * 9.81 m/s^2) 80%
#define ACC_Y_THRESHOLD_CONSTANT    62  // (8g = 8 * 9.81 m/s^2) 80%
#define GYR_Z_THRESHOLD_CONSTANT    800 // -500 ~ + 500, 1000 80%

#define NORM_CUTOFF_FREQ	        3




/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum {
    GAIT_MODE_STOP,
    GAIT_MODE_WALK,

    GAIT_MODE_NUM
} GaitMode;

typedef enum {
    GAIT_PHASE_10PER = 10,
    GAIT_PHASE_20PER = 20,
    GAIT_PHASE_30PER = 30,
    GAIT_PHASE_40PER = 40,
    GAIT_PHASE_50PER = 50,
    GAIT_PHASE_60PER = 60,
    GAIT_PHASE_70PER = 70,
    GAIT_PHASE_80PER = 80,
    GAIT_PHASE_90PER = 90
} GaitPhasePer;

typedef struct {
    // 16bit Scaled Data (For Sending Large PDO Datas)
    int16_t ThighDeg;
    int16_t ThighVelDeg;
    int16_t IncDeg;
    int16_t IncVelDeg;

    int16_t ThighRad;
    int16_t ThighVelRad;
    int16_t IncRad;
    int16_t IncVelRad;

    int16_t GyroZ;
    int16_t AccX;
    int16_t AccY;

    int16_t DegAccFiltered;
    int16_t DegGyroFiltered;
    int16_t RadAccFiltered;
    int16_t RadGyroFiltered;

    uint16_t wc;            // +0 ~ 100
    uint16_t FuzzyInput0;	// (AccX, AccY Square Root Sum - g) Absolute Value
    uint16_t FuzzyInput1;	// Jerk Square Root Sum
    uint16_t FuzzyInput2;	// GyroZ Absolute Value
    uint16_t FuzzyInput3; 	// GyroZ' Absolute Value
} ScaledData;

typedef struct _AngleData_t {
    float degLPF[2]; // 0: current, 1: previous
    float degLPF0ABS;
    float degBPF[3]; // 0: current, 1: previous, 2: previous of previous // center frequency BPF
    float degBPF_LPF[2];
    float degUnbiased[2];
    float degUnbias;
    float degBPF0ABS;
    float angleNorm;

    // For K10
    float incDegRaw;        
    float incDegLPF[3];    // Example for incremental leg position filtering
    float incDegTrig[3];   // Trigger points for incremental degrees
    float filteredIncDeg;

    float degFinal;         // Thigh Angel Degree (deg)
} AngleData_t;

typedef struct _VelData_t {
    float velLPF[2]; // 0: current, 1: previous
    float velLPFAbs;
    float velBPF;
    float velBPF0ABS;
    float velocityNorm;

    float velLPF2[2]; // 0: current, 1: previous

    // For K10
    float incVelRaw;
    float velLPF_K10;
    float incVel;       // Incremental velocity
    float incVelTrig[3];
    float filteredIncVel;

    float velFinal;         // Thigh Angular Velocity (deg/s)
} VelData_t;

typedef struct _GaitFilter_t {
    float cutoffFreq;
    float cutoffFreqSmooth;
    float PeakAmp;
    float PeakWAmp;
    float PeakAmpSmooth;
    float PeakWAmpSmooth;
} GaitFilter_t;

typedef struct _GaitObj_t {
//    GaitMode_t      GaitMode;
    AngleData_t     AngleData;
    VelData_t       VelData;
    GaitFilter_t    GaitFilter;

    float gyroLPF[2];   // 0: current, 1: previous
    float gyrZRaw[2];   // SAM IMU Gyro Z  0: current, 1: previous

    float gaitPhase;
    float gaitPhasePrev;
    float NeutralPosture;

    uint32_t gaitFuncLoopCnt;
    uint32_t gaitFuncLoopCnt_1sec;
    uint32_t timeStampPrev;

    uint32_t walkCurr, walkPrev;

    uint32_t chk1, chk1Prev;

    uint16_t Period;

    uint8_t firstPeriodCheck;

    uint8_t first1NE; // First non-empty check

    uint8_t phaseThreshold1;
    uint8_t phaseThreshold2;
    uint8_t phaseThreshold3;
    uint8_t phaseThreshold4;
    uint8_t phaseThreshold5;
    uint8_t phaseThreshold6;
    uint8_t phaseThreshold7;
    uint8_t phaseThreshold8;
    uint8_t phaseThreshold9;

    uint32_t gaitCount;
} GaitObj_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern float widm_accX_sagittal, widm_accY_sagittal, widm_gyrZ_sagittal;
extern float widm_degTvcf_sagittal, widm_velDegTvcf_sagittal, widm_degAcc_artan_sagittal;
extern float widm_fc_sagittal;

extern float widm_accX_frontal, widm_accY_frontal, widm_gyrZ_frontal;
extern float widm_degTvcf_frontal, widm_velDegTvcf_frontal, widm_degAcc_artan_frontal;
extern float widm_fc_frontal;

extern GaitObj_t gaitObj_RH;
extern GaitObj_t gaitObj_LH;
extern GaitObj_t gaitObj_RK;
extern GaitObj_t gaitObj_LK;

extern float widm_accX, widm_accY, widm_gyrZ;
extern float widm_degTvcf, widm_velDegTvcf, widm_degAcc_artan;
extern float widm_fc;

extern WIDM_SensorData_t widmSensorDataObj_sagittal;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGaitCtrl(void);
void RunGaitCtrl(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_AS_GAIT_CTRL_H_ */
