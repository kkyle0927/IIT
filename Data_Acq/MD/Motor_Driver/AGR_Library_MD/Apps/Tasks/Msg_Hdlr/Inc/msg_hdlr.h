/**
 * @file msgHdlrTask.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

#ifndef APPS_MSG_HDLR_INC_MSG_HDLR_H_
#define APPS_MSG_HDLR_INC_MSG_HDLR_H_

#include "task_mngr.h"
#include "error_dictionary.h"
#include "usbd_cdc_if.h"

#include "ioif_fdcan_common.h"
#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_usb_common.h"

#include "module.h"
#include "cvector.h"
#include "data_object_common.h"
#include "data_object_interface.h"
#include "gait_ctrl.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

extern volatile uint32_t MDUpdateFlag __attribute__((section(".MDUpdateSwitch")));
//extern volatile uint32_t MDFWBinarySize __attribute__((section(".MDFWBinSize")));

#define FDCAN_DLC_BYTES_0  ((uint32_t)0x00000000U) /*!< 0 bytes data field  */
#define FDCAN_DLC_BYTES_1  ((uint32_t)0x00010000U) /*!< 1 bytes data field  */
#define FDCAN_DLC_BYTES_2  ((uint32_t)0x00020000U) /*!< 2 bytes data field  */
#define FDCAN_DLC_BYTES_3  ((uint32_t)0x00030000U) /*!< 3 bytes data field  */
#define FDCAN_DLC_BYTES_4  ((uint32_t)0x00040000U) /*!< 4 bytes data field  */
#define FDCAN_DLC_BYTES_5  ((uint32_t)0x00050000U) /*!< 5 bytes data field  */
#define FDCAN_DLC_BYTES_6  ((uint32_t)0x00060000U) /*!< 6 bytes data field  */
#define FDCAN_DLC_BYTES_7  ((uint32_t)0x00070000U) /*!< 7 bytes data field  */
#define FDCAN_DLC_BYTES_8  ((uint32_t)0x00080000U) /*!< 8 bytes data field  */
#define FDCAN_DLC_BYTES_12 ((uint32_t)0x00090000U) /*!< 12 bytes data field */
#define FDCAN_DLC_BYTES_16 ((uint32_t)0x000A0000U) /*!< 16 bytes data field */
#define FDCAN_DLC_BYTES_20 ((uint32_t)0x000B0000U) /*!< 20 bytes data field */
#define FDCAN_DLC_BYTES_24 ((uint32_t)0x000C0000U) /*!< 24 bytes data field */
#define FDCAN_DLC_BYTES_32 ((uint32_t)0x000D0000U) /*!< 32 bytes data field */
#define FDCAN_DLC_BYTES_48 ((uint32_t)0x000E0000U) /*!< 48 bytes data field */
#define FDCAN_DLC_BYTES_64 ((uint32_t)0x000F0000U) /*!< 64 bytes data field */

#define MEMORY_SECOND_HAND_CHECK	1

#define USB_CDC_MSG_TX_TIMEOUT_MS   100

#define SUIT_APP_FW_INFO_SIZE			0x400

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum {
	FIRST_USE = 0x01,
	SECOND_USE,
	MEMORY_UI_MOTOR_PROPERTIES,
	MEMORY_UI_SENSOR_SETTING,
	MEMORY_UI_ELECTRICAL_PROPERTIES,
	MEMORY_UI_FRICTION_ID_PARAMETERS,
	MEMORY_UI_MECHANICAL_PROPERTIES,
	MEMORY_UI_CONTROL_PARAMETERS1,
	MEMORY_UI_CONTROL_PARAMETERS2,
	MEMORY_UI_ADDITIONAL_FUNCTION_PARAMETERS,

	E_SYS_BATCH,
	E_SYS_BEMF,
	BEMF_ID_OVER_CURRENT,
	BW_CHECK,
	FRICTION_ID_RAW_DATA,
	FRICTION_ID_AVERAGED_DATA,
	FRICTION_ID_DONE,
	FRICTION_COMPENSATOR_VERIFICATION,

	MECH_SYS_ID_SBS_RAW_DATA,
	MECH_SYS_ID_SBS_DONE,

	IRC_VERIFICATION,

	GET_IMPEDANCE_SINE_CTRL,
	GET_IMPEDANCE_REC_CTRL,

	GAIN_TUNER,
	GET_VELOCITY_CTRL,
	GET_POSITION_CTRL,

	GET_HALLSENSOR,
	GET_INCENCODER,
	GET_ABSENCODER1,
	GET_ABSENCODER2,

	GET_VSD_UPPER_LIMIT,
	GET_VSD_LOWER_LIMIT,

	VSD_VERIFICATION_DATA,

	GET_BACKLASH_TEST,
	GET_DOB_DATA,

	GET_DIRECTION_SET_DATA,
	GET_DIRECTION_SET_DONE,

	SAVE_DONE,
	GET_VE_TEST_DATA,
	GET_SYSTEM_ID_VERIFY,
	VE_KF_SETTING_ERROR,
	GET_FF_CTRL,
	GET_TOTAL_CTRL,

	ADV_FRICTION_ID_DATA,
	ADV_FRICTION_ID_DONE,

    GET_GAIT_PHASE,
    GET_ACC_GYRO,
    GET_QUATERNION,

    FRICTION_ID_VEL_CTRL_EVAL,
    GET_SBS_ID_VERIFY,
    GET_DOB_VERIFY,
    GET_ELEC_ANGLE_EST_EV,

    GET_HALL_VS_INCENC_DATA,
    GET_P_VECTOR_EVAL,
    GET_F_VECTOR_EVAL,
    GET_I_VECTOR_EVAL,
    GET_LINK_LUT,

	READ_NODE_ID,

	GET_AGING_DATA,
	GET_AGING_DONE,
	GET_ELEC_ANGLE_HOMING_DATA,
	GET_ELEC_ANGLE_HOMING_DONE,

	MEMORY_MES_HWFW,
	MES_CODE_SET,

    MOTOR_NAME_SET,
//    MOTOR_PROPERTIES_SET,
//    SENSOR_SETTINGS_SET,
	POLE_PAIR_SET,
	INC_ENC_RES_SET_LOW,
	INC_ENC_RES_SET_MID,
	GEAR_RATIO_SET,
	TORQUE_CST_SET,
	VOLT_CST_SET,
	PEAK_CURR_SET,
	CONT_CURR_SET,
	MAX_VELO_SET,
	COMM_SENSOR_SET,
	POS_FB_SENSOR_SET,
	ELEC_HOMING_SENSOR_SET,
	MECH_HOMING_SENSOR_SET,
	VELOCITY_ESTIMATOR_SET,
	ABSENC1_SIGN_SET,
	ABSENC2_SIGN_SET,
	FDCAN_SWITCH_SET,
	FDCAN_RESULT,

	TORQUE_ACCURACY_DATA, // 0x54
	TORQUE_ACCURACY_DONE,

	TORQUE_UNIFORMITY_DATA, // 0x56
	TORQUE_UNIFORMITY_DONE,

	ACTUATOR_BACKDRIVABILITY_DATA, // 0x58
	ACTUATOR_BACKDRIVABILITY_DONE, // 0x59

	GET_ELEC_ANGLE_ABSENC_HOMING_DATA, // 0x5A

	GET_ENCODER_ANGLE_DEVIATION_DATA, // 0x5B
	GET_ENCODER_ANGLE_LINEARITY_DONE, // 0x5C

} GUISequence_Enum;

typedef enum {
	IDLE,
	UPLOAD_PROPERTIES,
	SAVE_PROPERTIES,
	DOWNLOAD_PROPERTIES,
	ELECTRICAL_SYSTEM_ID,
	BEMF_ID,
	CURRENT_BANDWIDTH_CHECK,
	AUTO_TUNING,
	ADV_FRICTION_ID,
	CAL_FRICTION_LUT,
	AGING_TEST_DONE,
	INIT_ELEC_ANGLE_ID,
	IMPORT_MES_HWFW,
	IMPORT_MES_CODE,
	FDCAN_TEST_START,
	FDCAN_TEST_RESET,
	SAVE_PROPERTIES_NeutralPosture,
	SEND_TORQUE_ACCURACY_RESULT,
	SEND_TORQUE_UNIFORMITY_RESULT,
	SEND_ACTUATOR_BACKDRIVERBILITY_RESULT,
	SEND_ENCODER_LINEARITY_RESULT,
} MainSequence_Enum;

typedef enum _COMMType {
	COMM_TYPE_FDCAN,
	COMM_TYPE_USB
} COMMType;

typedef enum _BootFlag
{
	FW_UPDATE_FLAG = 1,
	FW_BACKUP_FLAG,
	FW_COPY_FLAG,
} BootFlag;

typedef enum _BootUpdateError
{
	BOOT_UPDATE_OK,
	BOOT_UPDATE_ERROR_FILE_SIZE,
	BOOT_UPDATE_ERROR_FILE_OPEN,
	BOOT_UPDATE_ERROR_FILE_READ,
	BOOT_UPDATE_ERROR_FLASH_WRITE,
	BOOT_UPDATE_ERROR_FLASH_READ,
	BOOT_UPDATE_ERROR_FLASH_ERASE,
	BOOT_UPDATE_ERROR_CRC,
	BOOT_UPDATE_ERROR_FILE_SIGN,
	BOOT_UPDATE_ERROR_FILE_OVERSIZE,
	BOOT_UPDATE_ERROR_VERIFY,
	BOOT_UPDATE_ERROR_INVALID_FW,
} BootUpdateError;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskObj_t			msgHdlrTask;
extern MainSequence_Enum 	MS_enum;
extern uint8_t           	MD_nodeID;
extern uint8_t 				CM_nodeID;
extern uint32_t 			MD_fw_binary_size;
/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Send_EMCY(uint32_t* f_err_code);
int Send_MSG(uint16_t t_COB_ID, uint8_t* t_tx_data, uint32_t t_len);
void Send_NOTI(uint32_t data);
void FDCAN_Test_Reset();
bool Get_FDCAN_Switch();
void FDCAN_Test_Read_Start();
void FDCAN_Test_Write_Start();
void Send_NeutralPostureDeg();
void SendDurationCompletedFlag(void);

void RoboWear_MonitorGPIOPin();
int Send_EOT(uint8_t index);
BootUpdateError Boot_SetFlags(uint32_t t_MD_Update, uint32_t t_MD_Backup);

void InitMsgHdlr(void);
void RunMsgHdlr(void* params);


#endif /* APPS_msgHdlrTask_INC_MSG_HDLR_H_ */
