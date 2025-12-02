/**
 ******************************************************************************
 * @file    ext_link_mngr.h
 * @author  Hyndo Kim
 * @brief   CM에서 확장팩으로의 통신을 전담하는 모듈 API
 * @version 0.1
 * @date    2025-07-11
 *
 * @copyright Copyright (c) 2025
 ******************************************************************************
 */

#ifndef APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_EXT_LINK_MNGR_H_
#define APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_EXT_LINK_MNGR_H_

#include <stdint.h>
#include <stdbool.h>

#include "data_object_interface.h"
#include "robot_motionmap_vector.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define EXT_MAX_PDO		60
#define EXT_MAX_SDO		60


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/* --- 1. CM-SCM 통신 전용 ID 및 타입 정의 --- */
// NMT 상태
typedef enum {
    NMT_STATE_INITIALISING = 0,
    NMT_STATE_PRE_OPERATIONAL,
    NMT_STATE_OPERATIONAL,
    NMT_STATE_STOPPED
} ExtLinkNmtState_t;

typedef uint8_t (*FDCAN_TxFncPtr) (uint16_t, uint8_t*, uint32_t); // ID, Data, Len

typedef struct _ExtLinkData_t {
	uint8_t pdo_list[EXT_MAX_PDO*2];
	uint8_t sdo_list[EXT_MAX_SDO*2];

	// CM
	// NMT
	uint8_t cm_bootup; 		// CM의 Boot up
	uint8_t cm_nmt_state; 	// CM의 NMT 상태 값
	uint8_t cm_heartbeat; 	// CM의 heartbeat 값
	uint8_t cm_sync_states;	// CM의 Sync State 신호

	// 신체 정보
	uint32_t user_weight;
	uint32_t user_height;
	uint32_t user_right_thigh_length;
	uint32_t user_left_thigh_length;
	uint32_t user_right_shin_length;
	uint32_t user_left_shin_length;
	uint32_t user_right_ankle_length;
	uint32_t user_left_ankle_length;

	// EXT
	ExtLinkNmtState_t ext_nmt_state;	// 확장팩의 NMT 상태 값
	// PIF vector
	P_Vector p_vector_RH;
	I_Vector i_vector_RH;
	float kpMaxRH;
	float kdMaxRH;
	float kpMaxLH;
	float kdMaxLH;
	MD_F_Vector f_vector_RH;
	P_Vector p_vector_LH;
	I_Vector i_vector_LH;
	MD_F_Vector f_vector_LH;
	// Aux input
	int16_t motor_auxiliary_input_RH;	// (Nm) Aux Torque Input from Extension board
	int16_t motor_auxiliary_input_LH;	// (Nm) Aux Torque Input from Extension board
	
	uint8_t suit_mode;
	uint8_t suit_assist_level;
	uint8_t P_Vector_Duration_Completed_RH;
	uint8_t P_Vector_Duration_Completed_LH;
	float degree_limit_upper_RH;
	float degree_limit_lower_RH;
	float degree_limit_upper_LH;
	float degree_limit_lower_LH;
	float velocity_limit_upper_RH;
	float velocity_limit_lower_RH;
	float velocity_limit_upper_LH;
	float velocity_limit_lower_LH;
	float velCompGain_RH;
	float velCompGain_LH;

	uint8_t H10_Existing_Assist_Mode_Flag;
} ExtLinkData_t;

// C언어의 struct 패딩을 막아 CAN 페이로드와 1:1로 일치시킴
#pragma pack(push, 1)
/**
 * @brief [신규] EXT -> CM으로 수신되는 실시간 PDO의 고정 페이로드 구조체
 */
typedef struct {
    int16_t   auxTorqueInputRH;  // 2 bytes
    int16_t   auxTorqueInputLH;  // 2 bytes
} ExtLink_PdoRx_ExtToCm_t;
#pragma pack(pop)

typedef struct _ExtCommLinkObj_t {
    uint16_t 		sdo_tx_id;	// 11bit
    uint16_t 		pdo_tx_id;	// 11bit
    FDCAN_TxFncPtr	tx_fnc;
	DOPI_DevObj_t	extObj;
    ExtLinkData_t 	data;	// 데이터 구조체를 멤버로 포함
} ExtCommLinkObj_t;

// 인스턴스 구조체
typedef struct {
    volatile ExtLinkNmtState_t cm_nmt_state;			// CM의 NMT 상태
    volatile bool        is_ext_pack_booted;            // 확장팩 Boot up 상태 플래그
    volatile bool        is_ext_pack_connected;         // 확장팩 연결 상태 플래그
    volatile uint32_t    last_ext_heartbeat_time_ms;    // 확장팩 HB 마지막 수신 시간
} ExtLinkMngrInst_t;

// C언어의 struct 패딩을 막아 CAN 페이로드와 1:1로 일치시킴
#pragma pack(push, 1)
/* For PDO sending */
typedef struct _ScaledData_t {
	int16_t Scaled_Theta_Hip_RH;
	int16_t Scaled_Theta_Hip_LH;
	int16_t Scaled_Theta_Pelvic;		// Theta Trunk RH, LH 평균값
	int16_t Scaled_Theta_Pelvic_Vel_Y;	// global coordinate Gyro Y RH, LH 평균값
	int16_t Scaled_Theta_Knee_RH;
	int16_t Scaled_Theta_Knee_LH;
	int16_t Scaled_Forward_Vel_X;
	int16_t Scaled_Theta_Hip_Motor_RH;	// Hip Encoder
	int16_t Scaled_Theta_Hip_Motor_LH;	// Hip Encoder

	int16_t Scaled_ActualCurrentRH;
	int16_t Scaled_ActualCurrentLH;

	int16_t Scaled_leftHipImuFrontalRoll;
	int16_t Scaled_leftHipImuSagittalPitch;
	int16_t Scaled_rightHipImuFrontalRoll;
	int16_t Scaled_rightHipImuSagittalPitch;
	int16_t Scaled_leftHipImuGlobalAccX;
	int16_t Scaled_leftHipImuGlobalAccY;
	int16_t Scaled_leftHipImuGlobalAccZ;
	int16_t Scaled_leftHipImuGlobalGyrX;
	int16_t Scaled_leftHipImuGlobalGyrY;
	int16_t Scaled_leftHipImuGlobalGyrZ;
	int16_t Scaled_rightHipImuGlobalAccX;
	int16_t Scaled_rightHipImuGlobalAccY;
	int16_t Scaled_rightHipImuGlobalAccZ;
	int16_t Scaled_rightHipImuGlobalGyrX;
	int16_t Scaled_rightHipImuGlobalGyrY;
	int16_t Scaled_rightHipImuGlobalGyrZ;
} ScaledData_t;
#pragma pack(pop)

typedef enum _SuitMode_t{
	SUIT_STANDBY_MODE,
	SUIT_ASSIST_MODE,
	SUIT_MODE_NUM
} SuitMode_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

// 통신 및 상태 관리용 객체 및 변수
extern ExtCommLinkObj_t g_ext_comm_obj; // 통신 채널 객체
extern ExtLinkMngrInst_t g_ext_link_mngr; // NMT 상태 관리 객체


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

/**
 * @brief 확장팩 링크 모듈 초기화.
 * @note  이 함수 내부에서 확장팩의 SDO 요청을 처리할 콜백 함수들을 등록합니다.
 */
void ExtLinkMngr_Init(void);

/**
 * @brief  확장팩으로 SDO 메시지를 전송하는 헬퍼 함수
 */
int TxSDOToExtPack(ExtCommLinkObj_t* ext_link, DOPI_SDOMsg_t* msg);

/**
 * @brief  확장팩으로 PDO 메시지를 전송하는 헬퍼 함수
 */
int TxPDOToExtPack(ExtCommLinkObj_t* ext_link, DOPI_PDOMsg_t* msg);

/**
 * @brief 확장팩 링크 모듈의 주기적 실행 함수.
 * @note  RunWholeBodyCtrl에서 1ms마다 호출되어야 합니다.
 */
void ExtLinkMngr_Execute(void);

/**
 * @brief 확장팩으로부터 수신된 CAN 메시지를 처리합니다.
 * @note  메인 FDCAN 콜백(AS_FDCAN_CB)에서 호출됩니다.
 */
void ExtLinkMngr_ProcessCANMessage(uint16_t can_id, uint8_t* data);

void ScalingForPDOSendToExt(void);

/**
 * @brief 확장팩이 현재 연결되어 있는지 확인하는 함수
 * @param inst 관리자 인스턴스
 * @return true 연결됨, false 끊어짐
 */
bool ExtLinkMngr_IsConnected(const ExtLinkMngrInst_t* inst);

void SendSUITMode(uint8_t suit_mode);
void SendSUITAssistLevel(uint8_t suit_assist_level);
void SendPVectorDurationCompleted_RH(uint8_t P_Vector_Duration_Completed_RH);
void SendPVectorDurationCompleted_LH(uint8_t P_Vector_Duration_Completed_LH);

void _ExtLink_SendPDOs(void);

#endif /* APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_EXT_LINK_MNGR_H_ */
