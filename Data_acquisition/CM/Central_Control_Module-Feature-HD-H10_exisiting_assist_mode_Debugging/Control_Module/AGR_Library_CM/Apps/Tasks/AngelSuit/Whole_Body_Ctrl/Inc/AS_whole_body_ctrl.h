

#ifndef WHOLE_BODY_CTRL_INC_AS_WHOLE_BODY_CTRL_H_
#define WHOLE_BODY_CTRL_INC_AS_WHOLE_BODY_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <inttypes.h> // for uint32_t printf type
#include <stdint.h>
#include <string.h>

#include "robot_setting.h"
#include "robot_FSM.h"
#include "robot_DMS.h"

#include "data_object_common.h"
#include "ring_buffer.h"

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_mcp79510.h"

#include "AS_dev_mngr.h"
#include "AS_system_ctrl.h"
#include "AS_gait_ctrl.h"
#include "AS_imu_ctrl.h"
#include "AS_ble_comm_hdlr.h"
#include "AS_ISI.h"
#include "GaitAnalysis.h"
#include "AS_data_ctrl.h"

/* Test */
#include "risk_mngr.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */
#define FAULT_TASK_BIT		(29U)
#define FAULT_TYPE_BIT		(28U)

#define WHOLEBODY_CONTROL_PERIOD         0.001   // 1ms period

#define MAX_ZERO(value) ((value) > 0 ? (value) : 0)

// Serial Number Configuration
#define SERIAL_NUM_FRONT_LEN            12      // Length of serial number front part from flash
#define H10_ID_STR_LEN                  9       // Valid string length for H10 ID (excluding null terminator)

#define MSG_HDLR_ACTIVATE_CNT			500
#define ALL_DEV_ROUTINE_ACTIVATE_CNT    1000
#define ALL_DEV_TASK_STATE_CNT          1500
#define WHOLE_BODY_LOOP_START_CNT       2000

#define SEND_FDCAN_CNT				990
#define RECEIVE_FDCAN_CNT			10

/* For Data Save */
#define SWAP_SD_BUF_SIZE  			13000
#define BUFFER_SD_TIMEOUT			1000
#define MILLISEC_TO_SEC             1000
#define MIN_TO_SEC                  60
#define MIN_TO_MILLISEC             (MIN_TO_SEC * MILLISEC_TO_SEC)
#define DATA_SAVE_TIME              15

/* For Data Save To Hex */
#define HEX_BUF_SIZE                130

#define CONV_UINT8                  1
#define CONV_UINT16                 2
#define CONV_FLOAT16                2
#define CONV_UINT32                 4
#define CONV_UINT64                 8

/* For Data Save To Hex (for 2Byte Data Processing) */
#define DATA_CONV_CONST_16BIT       65535       // 16Bit Resolution (2Byte)
#define GRF_RANGE_MAX               4000        // ADC 12Bit Resolution
#define VOLT_RANGE_MAX              60          // (Volt) 0 ~ 60V
#define DEG_ENC_RANGE_MAX           720         // (deg) -360 ~ +360
#define VELDEG_ENC_RANGE_MAX        6000        // (deg/s) -3000 ~ +3000
#define RAD_ENC_RANGE_MAX           2261.94624  // (rad) 720 * pi(3.141592)
#define RADVEL_RANGE_MAX            17.4533     // (rad/s) -500 * pi/180 rad/s ~ +500 * pi/180 rad/s
#define CURRENT_RANGE_MAX           60          // (A) -30 ~ +30
#define GYRO_RANGE_MAX              1000        // (deg/s) -500 ~ +500
#define SUM_FUZZY_INPUT_RANGE_MAX   1000        // FuzzyInput0 + FuzzyInput2 0 ~ 1000
#define ACC_RANGE_MAX               78.4532     // (m/s^2) 8 * g(9.80665)
#define ACC_FUZZYINPUT_RANGE_MAX    202.63      // (m/s^2) 8.81(9.81-1) * 23(FUZZY_LOGIC_SCALING_FACTOR)
#define MAG_RANGE_MAX               2400        // (micro Tesla) -1200 ~ +1200
#define WC_RANGE_MAX                20          // (Hz) 0 ~ 20Hz
#define MOTION_PHASE_RANGE_MAX      200         // (%) -100 ~ + 100
#define TEMP_RANGE_MAX      		400         // (℃) -200 ~ + 200
#define I_VECTOR_KPKD_GAIN_RANGE_MAX    100         // (%) 0 ~ +100
#define I_VECTOR_EPSILON_RANGE_MAX      25.5        // (deg) 0 ~ +25.5
#define I_VECTOR_LAMBDA_RANGE_MAX       2           // lambda 0 ~ 2.0

#define GRF_CONSTANT            ((float)DATA_CONV_CONST_16BIT / GRF_RANGE_MAX)              // -1000 ~ +3000
#define VOLT_CONSTANT           ((float)DATA_CONV_CONST_16BIT / VOLT_RANGE_MAX)             // +0V ~ +60V
#define DEG_ENC_CONSTANT        ((float)DATA_CONV_CONST_16BIT / DEG_ENC_RANGE_MAX)          // -360 deg ~ + 360 deg
#define VELDEG_ENC_CONSTANT     ((float)DATA_CONV_CONST_16BIT / VELDEG_ENC_RANGE_MAX)       // -500 deg/s ~ + 500 deg/s
#define RAD_ENC_CONSTANT        ((float)DATA_CONV_CONST_16BIT / RAD_ENC_RANGE_MAX)          // -30pi rad ~ +30pi rad
#define RADVEL_CONSTANT         ((float)DATA_CONV_CONST_16BIT / RADVEL_RANGE_MAX)           // (rad/s) -500 * pi/180 rad/s ~ +500 * pi/180 rad/s
#define CURRENT_CONSTANT        ((float)DATA_CONV_CONST_16BIT / CURRENT_RANGE_MAX)          // -180A ~ + 180A
#define GYRO_CONSTANT           ((float)DATA_CONV_CONST_16BIT / GYRO_RANGE_MAX)             // (deg/s) -500 ~ +500
#define SUM_FUZ_INPUT_CONSTANT  ((float)DATA_CONV_CONST_16BIT / SUM_FUZZY_INPUT_RANGE_MAX)  // FuzzyInput0 + FuzzyInput2 0 ~ 1000
#define ACC_CONSTANT            ((float)DATA_CONV_CONST_16BIT / ACC_RANGE_MAX)              // (m/s^2) 8 * g(9.80665)
#define ACC_FUZZYINPUT_CONSTANT ((float)DATA_CONV_CONST_16BIT / ACC_FUZZYINPUT_RANGE_MAX)   // (m/s^2) 8 * g(9.80665)
#define MAG_CONSTANT            ((float)DATA_CONV_CONST_16BIT / MAG_RANGE_MAX)              // (micro Tesla) -1200 ~ +1200
#define WC_CONSTANT             ((float)DATA_CONV_CONST_16BIT / WC_RANGE_MAX)               // (Hz) 0 ~ 20Hz
#define TEMP_CONSTANT           ((float)DATA_CONV_CONST_16BIT / TEMP_RANGE_MAX)             // (℃) -200 ~ + 200
#define I_VECTOR_EPSILON_CONSTANT   ((float)DATA_CONV_CONST_16BIT / I_VECTOR_EPSILON_RANGE_MAX)     // (deg) 0 ~ +25.5
#define I_VECTOR_KPKD_CONSTANT      ((float)DATA_CONV_CONST_16BIT / I_VECTOR_KPKD_GAIN_RANGE_MAX)   // (%) 0 ~ +100
#define I_VECTOR_LAMBDA_CONSTANT    ((float)DATA_CONV_CONST_16BIT / I_VECTOR_LAMBDA_RANGE_MAX)      // lambda 0 ~ 2.0

/* For Data Save To Hex (for 2Byte Data Processing) */
#define DATA_CONV_CONST_UINT16          65535       // uint16 conversion constant
#define DATA_CONV_CONST_INT16           32768       // int16 conversion constant
#define GRF_SCALING_FACTOR              4000        // ADC 12Bit Resolution
#define VOLT_SCALING_FACTOR             60          // (Volt) 0 ~ +60V
#define DEG_SCALING_FACTOR              720         // (deg) -360 ~ +360
#define VELDEG_SCALING_FACTOR           6000        // (deg/s) -3000 ~ +3000
#define RAD_SCALING_FACTOR              2261.94624  // (rad) 720 * pi(3.141592)
#define RADVEL_SCALING_FACTOR           17.4533     // (rad/s) -500 * pi/180 rad/s ~ +500 * pi/180 rad/s
#define CURRENT_SCALING_FACTOR          60          // (A) -30 ~ +30
#define GYR_SCALING_FACTOR              1000        // (deg/s) -500 ~ +500
#define SUM_FUZZY_INPUT_SCALING_FACTOR  1000        // (deg/s) 0 ~ +500
#define ACC_SCALING_FACTOR              78.4532     // (m/s^2) 8 * g(9.80665) -39.24 ~ +39.24
#define ACC_FUZZYINPUT_SCALING_FACTOR   202.63      // (m/s^2) 8.81(9.81-1) * 23(FUZZY_LOGIC_SCALING_FACTOR) (-39.24 ~ +39.24) * 23
#define MAG_SCALING_FACTOR              2400        // (micro Tesla) -1200 ~ +1200
#define WC_SCALING_FACTOR               20          // (Hz) 0 ~ 20Hz
#define MOTION_PHASE_SCALING_FACTOR     200         // (%) -100 ~ +100
#define I_VECTOR_KPKD_GAIN_SCALING_FACTOR   100         // (%) 0 ~ +100
#define I_VECTOR_EPSILON_SCALING_FACTOR     25.5        // (deg) 0 ~ +25.5
#define I_VECTOR_LAMBDA_SCALING_FACTOR      2           // lambda 0 ~ 2.0
#define TEMP_SCALING_FACTOR					400         // (°C) -200 ~ +200
#define FORWARD_VEL_X_SCALING_FACTOR    100         // (m/s) -50 ~ +50

#define CHECK_IMU_THRESHOLD     0.1f    // 모든 변화 감지 임계값
#define CHECK_IMU_DURATION_MS   5000    // 5초 = 5000ms
#define KalmanFilter_L 0.002743233497235

#define RM_MD_FAULT_BIT (29U)
/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum {
    RH_SAGITAL = 2,
    LH_SAGITAL,
    RK_SAGITAL,
    LK_SAGITAL,
    SUIT_MODULE_NUM,
} SUIT_ModuleIndex;

typedef enum {
    NEUTRAL_POSTURE_NOT_CALIBRATED = 5,
    NEUTRAL_POSTURE_CALIBRATED = 10,
} NeutralizedFlag_t;

typedef struct {
    uint32_t notiSignals;

    // 부위 절대 각도(deg)
    float ThighSagittalAngle;
    float ThighFrontalAngle;
    float ShankSagittalAngle;
    float ShankFrontalAngle;

    // 부위 절대 각속도(deg/s)
    float ThighSagittalVelocity;
    float ThighFrontalVelocity;
    float ShankSagittalVelocity;
    float ShankFrontalVelocity;

    // 부위 중립 각도(deg)
    float ThighNeutralSagittalAngle;
    float ThighNeutralFrontalAngle;
    float ShankNeutralSagittalAngle;
    float ShankNeutralFrontalAngle;

    // 모터 정보
    float MotorRefCurrent;
    float MotorActCurrent;
    float MotorActVoltage;
    
    float MotorTemp;
    float MotorMCUTemp;

    // 인가된 보조력 (Nm) (F Vectors + u_aux + feedback제어)
    float AssistiveTorque;

    // IMU TVCF 각도(deg)
    float IMU_TVCFDeg_sagittal;
    float IMU_TVCFDeg_frontal;
    float IMU_TVCFDeg_yaw;
    
    // 관절 각도 궤적(deg)
    float PvectorRef;

    // 임피던스 제어 파라미터
    float IvectorEpsilon;
    float IvectorKp;
    float IvectorKd;
    float IvectorLambda;

    // 실제 관절 각도(deg) - 엔코더 각도
    float PositionAct;
    float EncoderVelocity;

    /* Auto Calibration Check */
    float accXRaw;
    float accYRaw;
    float accZRaw;
    float gyroXRaw;
    float gyroYRaw;
    float gyroZRaw;

    float accXCalib;
    float accYCalib;
    float accZCalib;
    float gyroXCalib;
    float gyroYCalib;
    float gyroZCalib;

    float accXGlobal;
    float accYGlobal;
    float accZGlobal;
    float gyroXGlobal;
    float gyroYGlobal;
    float gyroZGlobal;
    /* TVCF */
    float wcTVCFCalib;

    float accDegInit;
    float accDegRaw;
    float accDeg;
    float gyrDeg;
} SAM_Motion_Struct;

typedef struct {
    bool ifLHHyperFlexion;
    bool ifLHHyperFlexion2;
    bool ifRHHyperFlexion;
    bool ifRHHyperFlexion2;
    bool ifLHFlexion;
    bool ifRHFlexion;
    bool ifLHNeutral;
    bool ifLHNeutral2;
    bool ifRHNeutral;
    bool ifRHNeutral2;
    bool ifLHExtension;
    bool ifRHExtension;
    bool ifHipAngleGapIsSmall;
    bool ifHipAngleGapIsLarge;
    bool ifHipAngleGapIsLeftLarge;
    bool ifHipAngleGapIsRightLarge;
    bool ifHipAngleGapIsLeftSmall;
    bool ifHipAngleGapIsRightSmall;
    bool ifLHIsStationary;
    bool ifRHIsStationary;
    bool ifLHFlexionIsOn;
    bool ifLHFlexionIsOn2;
    bool ifRHFlexionIsOn;
    bool ifRHFlexionIsOn2;
    bool ifLHFlexionIsFast;
    bool ifRHFlexionIsFast;
    bool ifLHExtensionIsOn;
    bool ifRHExtensionIsOn;
    bool ifLHExtensionIsSlow;
    bool ifRHExtensionIsSlow;
    bool ifLHExtensionIsFast;
    bool ifRHExtensionIsFast;
    bool ifAbsHipAngleGapIsXS;
    bool ifAbsHipAngleGapIsS;
    bool ifLeftHipAngleGapIsXL;
    bool ifLeftHipAngleGapIsXXL;
    bool ifLeftHipAngleGapIsXXXL;
    bool ifLeftHipAngleGapIsL;
    bool ifLeftHipAngleGapIsS;
    bool ifLeftHipAngleGapIsOn;
    bool ifRightHipAngleGapIsXL;
    bool ifRightHipAngleGapIsXXL;
    bool ifRightHipAngleGapIsXXXL;
    bool ifRightHipAngleGapIsL;
    bool ifRightHipAngleGapIsS;
    bool ifRightHipAngleGapIsOn;
    bool ifThighAngleSumIsXXL;
    bool ifThighAngleSumIsXL;
    bool ifThighAngleSumIsL;
    bool ifLKHyperFlexion;
    bool ifLKHyperFlexion2;
    bool ifRKHyperFlexion;
    bool ifRKHyperFlexion2;
    bool ifLKFlexion;
    bool ifRKFlexion;
    bool ifLKNeutral;
    bool ifRKNeutral;
    bool ifLKIsStationary;
    bool ifRKIsStationary;
    bool ifLKFlexionIsFast;
    bool ifRKFlexionIsFast;
    bool ifLKExtensionIsFast;
    bool ifRKExtensionIsFast;
    bool ifLKExtensionIsOn;
    bool ifRKExtensionIsOn;
    bool ifLKShankAngleBack;
    bool ifRKShankAngleBack;
    bool ifRightAccXIsS;
    bool ifRightAccYIsS;
    bool ifRightAccYIsXS;
    bool ifLeftAccXIsS;
    bool ifLeftAccYIsS;
    bool ifLeftAccYIsXS;
    /* 앉기-서기 보조 모드 */
    bool ifProApp_PostureHoldOn;
    bool ifProApp_PostureHoldOff;
    bool ifProApp_SitReady;
    bool ifProApp_SitReadyCancel;
    bool ifProApp_SitStart;
    bool ifProApp_StandReady;
    bool ifProApp_StandReadyCancel;
    bool ifProApp_StandStart;
    bool ifVectorEnd;
    bool ifLHHyperFlexion3;
    bool ifRHHyperFlexion3;
    bool ifRHHyperFlexion4;
    bool ifLHHyperFlexion4;
    bool ifRHHyperFlexion5;
    bool ifLHHyperFlexion5;
    bool ifStandingStart;
    bool ifsumYPlus;
    bool ifsumYMinus;
    bool ifsumsumYPlus;
    bool ifsumsumYMinus;
} MotionAnalysisConditions;

typedef struct _PIDObject {
	float ref;  // yd(k)
	float ref1; // yd(k+1)
	float ref2;	// yd(k+2)

	float act;
	float Ctrl_BW_Hz;

	float Kp;
	float Ki;
	float Kd;

	float R;		// Input penalty in LQ,   q1=1, q2=0

	float control_input;

	float err;
	float err_sum;
	float err_diff;
	float err_diff_f;
	float err_diff_ff;
	float err_diff_raw;
	float err_diff_raw_f;
	float err_diff_raw_ff;
} PIDObject;


typedef enum _AIRWALKING_SEQUENCE {
	AIRWALKING_MODE_OFF = 0,
	AIRWALKING_MODE_STBY,
	AIRWALKING_MODE_RUN_LOW,
	AIRWALKING_MODE_RUN_MID,
	AIRWALKING_MODE_RUN_HIGH,
} AIRWALKING_STEP;

typedef enum _AIRWALKING_POSITION {
	AIRWALKING_OFF = 0,
	AIRWALKING_NEUTRAL,
	AIRWALKING_LEFTUP_RIGHTDOWN,
	AIRWALKING_LEFTDOWN_RIGHTUP,
} AIRWALKING_POS;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern uint8_t pwrBtPushed;

/* For Data Test */
/* extern variable */
extern osSemaphoreId_t sdio_sync_semaphore;
extern osSemaphoreId_t BinSem_PlayBeepHandle;
extern uint8_t *sd_buf_cur;
extern uint8_t *sd_buf_next;

extern uint8_t sd_buf_1[SWAP_SD_BUF_SIZE];
extern uint8_t sd_buf_2[SWAP_SD_BUF_SIZE];

// extern uint8_t sdSendData[HEX_BUF_SIZE];

extern uint8_t dataSaveFinished;

/* For H10 Battery Usage Data Logging */
extern uint8_t oneTimeSave;
extern uint32_t dataSaveCnt;

extern SAM_Motion_Struct RH_Sagittal;
extern SAM_Motion_Struct LH_Sagittal;
extern SAM_Motion_Struct RK_Sagittal;
extern SAM_Motion_Struct LK_Sagittal;

extern uint8_t neutralPosCalCMD;
extern bool isNeutralCalCompleted;

extern uint8_t assistmode;

extern MotionAnalysisConditions MotionFlags;

extern uint8_t canOperate;
extern uint32_t wholeBodyCtrlLoopCnt;
extern uint16_t StandbyCnt;

extern float RH_angle_temp;
extern float LH_angle_temp;

extern uint8_t motion_test_mode;

extern int inter_cnt;
extern uint8_t aging_enable;
extern uint8_t manu_mode;
extern uint8_t prev_manu_mode;
extern uint16_t SUIT_totalDuration;
extern uint16_t SUIT_Duration;
extern uint8_t STSMode;
extern uint8_t PostureHold;

//extern float   SittoStandDuration;
//extern float   StandtoSitDuration;
//extern uint8_t  StandtoSitAssistLevel = 0;
//extern uint8_t  SittoStandAssistLevel = 0;

extern uint8_t SitReadyButton;
extern uint8_t StartSittingButton;
extern uint8_t StandReadyButton;
extern uint8_t StartStandingButton;

extern float RightHipSmartAssistGravityCompensation;
extern float LeftHipSmartAssistGravityCompensation;
extern float RightHipSmartAssistVelocityCompensation;
extern float LeftHipSmartAssistVelocityCompensation;

extern float RightKneeSmartAssistGravityCompensation;
extern float LeftKneeSmartAssistGravityCompensation;
extern float RightKneeSmartAssistVelocityCompensation;
extern float LeftKneeSmartAssistVelocityCompensation;

extern float RightHipAquaGravityCompensation;
extern float LeftHipAquaGravityCompensation;
extern float RightHipAquaVelocityCompensation;
extern float LeftHipAquaVelocityCompensation;

extern float RightKneeAquaGravityCompensation;
extern float LeftKneeAquaGravityCompensation;
extern float RightKneeAquaVelocityCompensation;
extern float LeftKneeAquaVelocityCompensation;

extern float RightHipUnivGravityCompensation;
extern float LeftHipUnivGravityCompensation;
extern float RightHipUnivVelocityCompensation;
extern float LeftHipUnivVelocityCompensation;

extern float RightKneeUnivGravityCompensation;
extern float LeftKneeUnivGravityCompensation;
extern float RightKneeUnivVelocityCompensation;
extern float LeftKneeUnivVelocityCompensation;

extern float RightHipFlexionLimit;
extern float RightHipExtensionLimit;
extern float RightHipFlexionVelocityLimit;
extern float RightHipExtensionVelocityLimit;

extern float LeftHipFlexionLimit;
extern float LeftHipExtensionLimit;
extern float LeftHipFlexionVelocityLimit;
extern float LeftHipExtensionVelocityLimit;

extern float RightKneeFlexionLimit;
extern float RightKneeExtensionLimit;
extern float RightKneeVelocityLimit;

extern float LeftKneeFlexionLimit;
extern float LeftKneeExtensionLimit;
extern float LeftKneeVelocityLimit;

extern float LeftHipMaxAngle;
extern float RightHipMaxAngle;
extern float LeftHipMinAngle;
extern float RightHipMinAngle;
extern float LeftHipMaxVelocity;
extern float RightHipMaxVelocity;
extern float LeftHipMinVelocity;
extern float RightHipMinVelocity;

extern float Asymmetry[2];
extern float AsymmetryFilter;
extern float AsymmetryAvg;
extern float gaitCount[2];
extern uint32_t gaitCountLeft[2];
extern uint32_t gaitCountRight[2];

extern float RHamplitude;
extern float LHamplitude;

extern float R_ThighSagittalAngle;
extern float L_ThighSagittalAngle;
extern float R_ThighSagittalVelocity;
extern float L_ThighSagittalVelocity;
extern float R_ShankSagittalAngle;
extern float L_ShankSagittalAngle;

extern float RH_PositionAct;
extern float LH_PositionAct;
extern float RH_EncoderVelocity;
extern float LH_EncoderVelocity;
extern float RH_IMU_TVCFDeg_sagittal, LH_IMU_TVCFDeg_sagittal;
extern float RH_IMU_TVCFDeg_frontal, LH_IMU_TVCFDeg_frontal;
extern float RK_PositionAct;
extern float LK_PositionAct;
extern float RK_EncoderVelocity;
extern float LK_EncoderVelocity;
extern float RK_IMU_TVCFDeg_sagittal, LK_IMU_TVCFDeg_sagittal;
extern float RK_IMU_TVCFDeg_frontal, LK_IMU_TVCFDeg_frontal;
extern float PelvicSagittalAngle, PelvicFrontalAngle;

extern float RH_ThighNeutralSagittalAngle;
extern float LH_ThighNeutralSagittalAngle;
extern float RK_ThighNeutralSagittalAngle;
extern float LK_ThighNeutralSagittalAngle;

extern float RH_accXCalib, RH_accYCalib, RH_accZCalib;
extern float LH_accXCalib, LH_accYCalib, LH_accZCalib;
extern float RH_gyroXCalib, RH_gyroYCalib, RH_gyroZCalib;
extern float LH_gyroXCalib, LH_gyroYCalib, LH_gyroZCalib;

extern float RK_accXCalib, RK_accYCalib, RK_accZCalib;
extern float LK_accXCalib, LK_accYCalib, LK_accZCalib;
extern float RK_gyroXCalib, RK_gyroYCalib, RK_gyroZCalib;
extern float LK_gyroXCalib, LK_gyroYCalib, LK_gyroZCalib;

extern float RH_accXGlobal[2], RH_accYGlobal[2], RH_accZGlobal[2];
extern float LH_accXGlobal[2], LH_accYGlobal[2], LH_accZGlobal[2];
extern float RH_gyroXGlobal[2], RH_gyroYGlobal[2], RH_gyroZGlobal[2];
extern float LH_gyroXGlobal[2], LH_gyroYGlobal[2], LH_gyroZGlobal[2];

extern float RK_accXGlobal[2], RK_accYGlobal[2], RK_accZGlobal[2];
extern float LK_accXGlobal[2], LK_accYGlobal[2], LK_accZGlobal[2];
extern float RK_gyroXGlobal[2], RK_gyroYGlobal[2], RK_gyroZGlobal[2];
extern float LK_gyroXGlobal[2], LK_gyroYGlobal[2], LK_gyroZGlobal[2];

extern uint8_t STS_P_Vector_StartSittingDuration;
extern uint8_t STS_P_Vector_StartStandingDuration;

extern uint8_t STS_P_Vector_Duration_RH;
extern uint8_t STS_P_Vector_Duration_LH;
extern uint8_t STS_P_Vector_Duration_RK;
extern uint8_t STS_P_Vector_Duration_LK;

extern uint8_t AssistMode;

extern uint16_t sitCount;
extern uint16_t standCount;
extern uint32_t sitTime;
extern uint32_t standTime;
extern uint32_t accumulateSitTime;
extern uint32_t accumulateStandTime;

extern uint8_t DisableStairWalkingDetection;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitWholeBodyCtrl(void);
void RunWholeBodyCtrl(void);

void UpdateNotiSignal(void);
float traj_calcul(float traj_para[], int time, float time_amp, float mag_amp, int time_offset);
void Run_MANU_PID_Control(PIDObject *t_PID_obj, float t_ref, float t_actual, float t_period);

/* Test */
void testFaultCheck(void* param);


#endif /* SUIT_MINICM_ENABLED */

#endif /* WHOLE_BODY_CTRL_INC_AS_WHOLE_BODY_CTRL_H_ */
