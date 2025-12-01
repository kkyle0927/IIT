

#ifndef AS_DEV_MNGR_INC_AS_DEV_MNGR_H_
#define AS_DEV_MNGR_INC_AS_DEV_MNGR_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

/*Backbone*/
#include "robot_setting.h"
#include "robot_motionmap_vector.h"
#include "device_id.h"
/*CDI*/
#include "data_object_dictionaries.h"
#include "data_object_interface.h"
/*IOIF*/
#include "ioif_fdcan_common.h"
/*APP*/
#include "AS_RiskMngt_Hdlr.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define AS_DEV_MAX_ROUTINES 8
#define AS_DEV_MAX_PDO_LIST 60

#define AS_DEV_MAX_PDO		60
#define AS_DEV_MAX_SDO		60
#define AS_DEV_IDX_MAX		6

#define RM_MD_ERROR_ARRAY  10 // 갯수 임시 : md 4개 고려

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef uint8_t (*AS_FDCAN_TxFncPtr) (uint16_t, uint8_t*, uint32_t); // ID, Data, Len

typedef enum _AS_IDX_t {
#ifdef SUIT_H10
    AS_MD_IDX_1 = 6,    // RH_SAG
    AS_MD_IDX_2,        // LH_SAG
#endif
#ifdef SUIT_K10
    AS_MD_IDX_1 = 8,    // RK
    AS_MD_IDX_2,        // LK
#endif 
#ifdef SUIT_A10
    AS_MD_IDX_1 = 12,   // RA_LAT
    AS_MD_IDX_2,        // LA_LAT
#endif 
	AS_DEV_MAX_NUM
} AS_IDX_t;

typedef enum _FaultLocationMD_t{
	LOWLEVEL = 1,
	SYSMNGT  = 6,
    WARNING  = 7
} FaultLocationMD_t;

typedef struct _FaultMD_t {
	uint16_t  device_id;
	uint32_t  error_bit;
} FaultMD_t;

typedef struct _AS_TaskData_t {
    uint8_t state;
    uint8_t routines[AS_DEV_MAX_ROUTINES];
    uint8_t n_routines;
} AS_TaskData_t;

typedef struct _Quaternion {
	int16_t q_x;
	int16_t q_y;
	int16_t q_z;
	int16_t q_w;
} Quaternion;

typedef struct _AS_DevData_t {
    AS_TaskData_t msg_hdlr_task;
	AS_TaskData_t low_level_ctrl_task;
	AS_TaskData_t mid_level_ctrl_task;
    AS_TaskData_t imu_ctrl_task;
    AS_TaskData_t gait_ctrl_task;
    AS_TaskData_t ext_dev_ctrl_task;

	uint8_t pdo_list[AS_DEV_MAX_PDO*2];
	uint8_t sdo_list[AS_DEV_MAX_SDO*2];

	C_Vector  c_vector;
	P_Vector  p_vector;
	uint8_t p_vector_reset;
	F_Vector  f_vector;
	MD_F_Vector md_f_vector;
	I_Vector  i_vector;
	float     i_vector_Kp_max;
	float     i_vector_Kd_max;

    float motor_auxiliary_input;	// (A) Aux Current Input
	float gravCompGain;
	float velCompGain;
	float velCompSatu;


	float HipRoMUpperLimit;
	float HipRoMLowerLimit;
	float HipUpperVelLimit;
	float HipLowerVelLimit;

	uint8_t FSM_curr;
	uint8_t yd_f_dummy;

	float SitToStance_Torque;
	float StanceToSit_Torque;

	uint8_t suit_mode;
	uint8_t isNeutralized;
	uint8_t isNeutralInit;
	uint8_t neutralBiasCmd;
	uint8_t autoCalibSensorCmd;

	// Todo : Inc/IMU data must be separated!
	// (rad) H10: Inc(상대 각도) + IMU(TVCF Degree)
	// (rad) K10: IMU(TVCF Degree)
	int16_t thighAngleDeg;
	// (rad/s) H10: Inc Vel(Estimated) + IMU(GyroZ)
	// (rad/s) K10: IMU(GyroZ)
	int16_t thighVelDeg;
	int16_t thighVelDeg_DiffINC;
	int16_t thighVelDeg_Diff;

	int16_t initThighAngleDeg;

    int16_t incActualPosDeg;		// (deg) Inc Actual Position
    int16_t incActualVelDeg;		// (deg/s) Inc Actual Velocity (Estimated)
	int16_t incActualVelDeg_Diff;

	int16_t IMU_TVCFDeg;

	int16_t IMU_TVCFDeg_frontal;
	int16_t IMU_TVCFDeg_sagittal;
	int16_t IMU_TVCFDeg_yaw;

	int16_t accXRaw;			// IMU Acc X Raw Value
	int16_t accYRaw;			// IMU Acc Y Raw Value
	int16_t accZRaw;			// IMU Acc Z Raw Value

	int16_t gyrXRaw;			// IMU Gyro X Raw Value
	int16_t gyrYRaw;			// IMU Gyro Y Raw Value
	int16_t gyrZRaw;			// IMU Gyro Z Raw Value

	int16_t accXCalib;
	int16_t accYCalib;
	int16_t accZCalib;

	int16_t gyrXCalib;
	int16_t gyrYCalib;
	int16_t gyrZCalib;

	int16_t accXGlobal;
	int16_t accYGlobal;
	int16_t accZGlobal;

	int16_t gyrXGlobal;
	int16_t gyrYGlobal;
	int16_t gyrZGlobal;

	uint16_t wcTVCF_calib;

	int16_t accDegInit;
	int16_t accDegRaw;
	int16_t accDeg;
	int16_t gyrDeg;

	int16_t currentRef;			// (A) Current Ref input (Total Current Input)
	int16_t currentAct;			// (A) Actual Current Output
	uint16_t volt_scaling;		// (V) 2480으로 나눈 전압값
	int16_t motor_temp_scaling;

	float actual_volt;
	float actual_curr;
	float motor_temp;
	uint32_t motor_mcutemp;

	uint16_t fsr_middleToe;	// 1:발앞중간
	uint16_t fsr_bigToe;	// 2:발앞안쪽
	uint16_t fsr_littleToe;	// 3:발앞바깥쪽
	uint16_t fsr_heel;		// 4:발뒤꿈치

	uint32_t BFlagsTest;	// to be deleted

	
	int16_t thighRawDeg;
	int16_t thighNeutralPostureDeg;
	int16_t thighNeutralPostureOffset;	// +- 0.5도
	int16_t UserNeutralPostureAngle;	// 태블릿 전송 사용자 중립각도 값

	uint8_t I2CfailedCnt;
	uint16_t I2CfailedStateOffCnt;
	uint32_t I2CfailedStateEnableCnt;

	float scaling_gyro;
	float bias_gyro;
	float accMeanVal_calibrated;

	float accMeanValue;

	float inc_enc;
	float abs1_enc;
	float abs2_enc;

	float inc_enc_velo;
	float abs1_enc_velo;
	float abs2_enc_velo;

	int16_t p_vector_ref;
	int16_t i_vector_epsilon;
	int16_t i_vector_kP;
	int16_t i_vector_lambda;

	uint8_t can_test;

	uint32_t err_code;
	uint8_t joint_limit_sw;
	uint8_t Torque_measure_sw;

} AS_DevData_t;

typedef struct _AS_CtrlObj_t {
    // IO Properties
    uint16_t sdo_tx_id;
    uint16_t pdo_tx_id;
    AS_FDCAN_TxFncPtr tx_fnc;

    // Device Data
    AS_DevData_t data;

    // Device Object
	DOPI_DevObj_t devObj;
} AS_CtrlObj_t;



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern AS_CtrlObj_t userCtrlObj[AS_DEV_IDX_MAX];
extern bool md_error_flag;
extern FaultMD_t mdErrorBit[RM_MD_ERROR_ARRAY];

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

/* Init */
void InitFDCANDevMngr(void);

int TxSDO(int MD_idx);
int TxPDO(int MD_idx);

void SendAuxInput(int DevIdx, float motorAuxIn, float assistPctg);
void SendIsNeutralized(int DevIdx, uint8_t t_isN);
void SendIsNeutralInit(int DevIdx, uint8_t t_isN);
void SendNeutralPostureCalCMD(int DevIdx, uint8_t t_Cmd);
void SendNeutralPostureOffset(int DevIdx, int16_t neutralPostureOffset);
void SendUserNeutralPostureAngle(int DevIdx, float NeutralSagittalAngle);

void Send_C_Vector(int obj_idx, uint8_t K_FF, uint8_t K_PD, uint8_t K_IC, uint8_t K_DOB, uint8_t K_IRC, uint8_t K_FC);
void Send_P_Vector(int obj_idx, int16_t yd, uint16_t L, uint8_t s0, uint8_t sd);
void Send_P_Vector_Reset(int DevIdx);
void Send_F_Vector(int DevIdx, uint8_t modeIdx, int16_t TauMax, uint16_t Delay);
void Send_F_Vector_Reset(int DevIdx);
void Send_I_Vector(int obj_idx, uint8_t t_epsilon, uint8_t t_Kp, uint8_t t_Kd, uint8_t t_lambda, uint16_t duration);
void Set_I_Vector_Kp_Max(int obj_idx, float Kp_max);
void Set_I_Vector_Kd_Max(int obj_idx, float Kd_max);
void Set_I_Vector_KpKd_Max(int objIdx, float kpMax, float kdMax);

void Send_AUX_Torque(int MD_idx, float torque);

void Send_Grav_Vel_Comp_Gain(int obj_idx, float gravCompGain, float velCompGain);
void Send_Comp_parameter(int obj_idx, float gravCompGain, float velCompGain, float velCompSatu, uint8_t suit_mode);
void Send_Limit_parameter(int obj_idx, float HipRoMUpperLimit, float HipRoMLowerLimit, float HipUpperVelLimit, float HipLowerVelLimit, uint8_t suit_mode);
void Send_Degree_Limit(int obj_idx, float HipRoMUpperLimit, float HipRoMLowerLimit);
void Send_Velocity_Limit(int obj_idx, float HipUpperVelLimit, float HipLowerVelLimit);
void Send_PDO_Nothing(int MD_idx);
void AllSyncPDOData(void);

void AllDevOffStates(void);
void AllDevStandbyStates(void);
bool DevStandbyStates(int DevIdx);
void AllDevEnableStates(void);
void AllDevSetRoutines(void);

void AllSetManuPDO(void);
void AllSetDevPDO(void);
void SetManuPDO(int DevIdx);
void SetManu2PDO(int DevIdx);
void SetDevPDO(int DevIdx);
void SetDevRoutine(int DevIdx);
void SetDevTaskState(int DevIdx);

void AllDevSetJointLimitRoutine(void);
void AllDevClearJointLimitRoutine(void);
void SetDevJointLimitRoutine(int DevIdx);
void ClearDevJointLimitRoutine(int DevIdx);
// DOB Routine
void SetDevDOBRoutine(int DevIdx);
void ClearDevDOBRoutine(int DevIdx);
// Degree Limit Routine
void SetDevDegreeLimitRoutine(int DevIdx);
void ClearDevDegreeLimitRoutine(int DevIdx);
// Velocity Limit Routine
void SetDevVelocityLimitRoutine(int DevIdx);
void ClearDevVelocityLimitRoutine(int DevIdx);

void SetSerialSDO(int DevIdx);
void SetVersionSDO(int DevIdx);

void ReqAccMeanValue(int DevIdx);

AS_DevData_t* GetDevDataSet(int DevIdx);

/*RM*/
void FlagMDFault(uint8_t buffer_cnt);

#endif /* SUIT_MINICM_ENABLED */

#endif /* AS_DEV_MNGR_INC_AS_DEV_MNGR_H_ */
