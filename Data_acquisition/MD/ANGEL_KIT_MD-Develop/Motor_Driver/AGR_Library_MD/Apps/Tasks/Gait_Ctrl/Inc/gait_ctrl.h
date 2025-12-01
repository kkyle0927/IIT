#ifndef GAIT_CTRL_INC_GAIT_CTRL_H_
#define GAIT_CTRL_INC_GAIT_CTRL_H_

#include "module.h"

#include <math.h>
#include <stdbool.h>

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "error_dictionary.h"

// For IMU //
#include "ioif_icm20608g.h"
#include "ioif_bm1422agmv.h"
#include "widm_algorithms.h"

#include "mid_level_ctrl.h"
#include "low_level_ctrl.h"
#include "msg_hdlr.h"


// For Quaternion //
#include "vqf.h"
#include "spi.h"

#include "data_object_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */


#define DET_EPSILON 1e-10  // 작은 값으로 설정하여 행렬의 특이성 검사 시 사용

#define ISI_B_FLAG_MAX_NUM              500

#define ACC_GRAVITY_CONSTANT 	        9.81    // (m/s^2)

/* For Gait Function */
#define THIGH_DEG_FIRST_LPF_SMOOTHING_FACTOR    0.02
#define THIGH_VEL_FIRST_LPF_SMOOTHING_FACTOR    0.02
#define THIGH_VELLPF2_SMOOTHING_FACTOR          0.0003

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
#define ACC_THRESHOLD_CONSTANT      (float)15.696  // -9.81 m/s^2 ~ +9.81 m/s^2, 19.62 80%
#define GYR_THRESHOLD_CONSTANT      800 // -500 ~ + 500, 1000 80%

/* For Data Save To Hex (for 2Byte Data Processing) */
#define DATA_CONV_CONST_UINT16              65535       // uint16 conversion constant
#define DATA_CONV_CONST_INT16               32768       // int16 conversion constant
#define GRF_SCALING_FACTOR                  4000        // ADC 12Bit Resolution
#define VOLT_SCALING_FACTOR                 60          // (Volt) 0 ~ +60V
#define DEG_SCALING_FACTOR                  720         // (deg) -360 ~ +360
#define VELDEG_SCALING_FACTOR               6000        // (deg/s) -3000 ~ +3000
#define RAD_SCALING_FACTOR                  2261.94624  // (rad) 720 * pi(3.141592)
#define RADVEL_SCALING_FACTOR               17.4533     // (rad/s) -500 * pi/180 rad/s ~ +500 * pi/180 rad/s
#define CURRENT_SCALING_FACTOR              60          // (A) -30 ~ +30
#define GYR_SCALING_FACTOR                  1000        // (deg/s) -500 ~ +500
#define SUM_FUZZY_INPUT_SCALING_FACTOR      1000        // (deg/s) 0 ~ +500
#define ACC_SCALING_FACTOR                  78.4532     // (m/s^2) 8 * g(9.80665) -39.24 ~ +39.24
#define ACC_FUZZYINPUT_SCALING_FACTOR       202.63      // (m/s^2) 8.81(9.81-1) * 23(FUZZY_LOGIC_SCALING_FACTOR) (-39.24 ~ +39.24) * 23
#define MAG_SCALING_FACTOR                  2400        // (micro Tesla) -1200 ~ +1200
#define WC_SCALING_FACTOR                   20          // (Hz) 0 ~ 10Hz
#define MOTION_PHASE_SCALING_FACTOR         200         // (%) -100 ~ +100
#define I_VECTOR_EPSILON_SCALING_FACTOR     25.5        // (rad) 0 ~ 0.45 [0 ~ +25.5(deg)]
#define I_VECTOR_KPKD_GAIN_SCALING_FACTOR   100         // (%) 0 ~ +100
#define I_VECTOR_LAMBDA_SCALING_FACTOR      2           // lambda 0 ~ 2.0
#define TEMP_SCALING_FACTOR					400         // (°C) -200 ~ +200

#define NORM_CUTOFF_FREQ	        	3

#define RAD2DEG							(float)180.0 / M_PI
#define I2C_REVOVERY_TRAILS             3           // 3 Trials
#define I2C_REVOVERY_TIME_INTERVAL      20          // 20ms

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

typedef enum {
    NOT_CALIBRATED,
    CALIBRATED
} AutoCalibratedFlag_t;

typedef enum {
    NEUTRAL_POSTURE_NOT_CALIBRATED = 5,
    NEUTRAL_POSTURE_CALIBRATED = 10,
} NeutralizedFlag_t;

typedef struct {
	struct {
		float x;
		float y;
		float z;
	} accel_local;

	struct {
		float x;
		float y;
		float z;
	} accel_bodyframe;

	struct {
		float x;
		float y;
		float z;
	} gyro_local;

	struct {
		float x;
		float y;
		float z;
	} gyro_global;

	struct {
		float roll;
		float pitch;
		float yaw;
	} euler;

	struct {
		float sagittal;
		float frontal;
		float transverse;
	} angle_bodyframe;

	struct {
		float sagittal;
		float frontal;
		float transverse;
	} sensor_to_body_offset;
} VQF_IMUData_t;

typedef struct {
    // 16bit Scaled Data (For Sending Large PDO Datas)
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyrX;
    int16_t GyrY;
    int16_t GyrZ;

    int16_t AccX_calib;
    int16_t AccY_calib;
    int16_t AccZ_calib;
    int16_t GyrX_calib;
    int16_t GyrY_calib;
    int16_t GyrZ_calib;

    int16_t AccX_global;
    int16_t AccY_global;
    int16_t AccZ_global;
    int16_t GyrX_global;
    int16_t GyrY_global;
    int16_t GyrZ_global;

    int16_t ThighDeg_sagittal;
    int16_t ThighVelDeg_sagittal;
    int16_t ThighVelDiffINC_sagittal;
    int16_t ThighVelDiff_sagittal;
    int16_t IncDeg_sagittal;
    int16_t IncVelDeg_sagittal;
    int16_t IncVelDiff_sagittal;

    int16_t IMU_BodyAngle_Sagittal;
    int16_t IMU_BodyAngle_Frontal;
    int16_t IMU_BodyAngle_Transverse;

    uint16_t fc_sagittal;       // +0 ~ 100
    uint16_t fc_calib_sagittal; // +0 ~ 100

    int16_t MotorRefCurrent;
    int16_t MotorActCurrent;
    uint16_t MotorActVoltage;
    int16_t MotorTemp;

    int16_t P_Vector_Ref;
    int16_t I_Vector_Epsilon;
    int16_t I_Vector_kP;
    int16_t I_Vector_kD;
    int16_t I_Vector_lambda;
} ScaledData;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern IMUAttachCase_t IMUAttachCase;
extern IMUProcessedData_t imuProcessedDataObj;
extern VQF_IMUData_t imuVQFDataObj;
extern FilteredAngleData_t filteredAngleDataObj_Sagittal;

extern uint32_t autoCalibratedFlag;
extern uint32_t neutralizedFlag;

extern Flash_Sensor_t flashsensorObj;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGaitCtrl(void);
void RunGaitCtrl(void* params);

int16_t ScaleFloatToInt16(float value, float scaleFactor);
uint16_t ScaleFloatToUInt16(float value, float scaleFactor);
float ScaleInt16ToFloat(int16_t value, float scaleFactor);


#endif /* GAIT_CTRL_INC_GAIT_CTRL_H_ */
