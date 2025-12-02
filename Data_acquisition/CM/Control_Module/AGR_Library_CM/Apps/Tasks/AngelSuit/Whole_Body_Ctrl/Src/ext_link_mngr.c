/**
 ******************************************************************************
 * @file    ext_link_mngr.c
 * @author  Hyundo Kim
 * @brief   CM에서 확장팩으로의 통신을 전담하는 단일 창구 모듈 API
 * @version 0.1
 * @date    2025-07-11
 *
 * @copyright Copyright (c) 2025 Angel Robotics Inc. All rights reserved.
 ******************************************************************************
 */

#include "ext_link_mngr.h"
#include "cmext_object_dictionary.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "data_object_interface.h"
#include "ioif_fdcan_common.h"
#include "ioif_tim_common.h"
#include "AS_whole_body_ctrl.h"
#include "GaitAnalysis.h"

/**
 *-----------------------------------------------------------
 *             TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

#define EXT_LINK_PDO_LIST_MAX_SIZE 32 // SCM이 요청할 수 있는 PDO의 최대 개수
#define PDO_STAGE_MAX_SIZE 2  // CM->MD으로 전송할 PDO 개수

// 신체 정보 지수
#define DEFAULT_WEIGHT          70.0f   // kg
#define DEFAULT_HEIGHT          1.75f   // m
#define DEFAULT_THIGH_LENGTH    0.45f   // m
#define DEFAULT_SHIN_LENGTH     0.40f   // m
#define DEFAULT_ANKLE_LENGTH    0.10f   // m

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

extern AS_CtrlObj_t userCtrlObj[AS_DEV_IDX_MAX];
extern float Theta_Trunk_Improved;
extern float Theta_Hip_RH;
extern float Theta_Hip_LH;
extern float ThetaLK_Improved[2];
extern float ThetaRK_Improved[2];
extern bool LeftFootContact_Improved[2];
extern bool RightFootContact_Improved[2];
extern float velX_Improved[2];
extern float pelvic_Vel_Y;
extern FSM_Mngr FSMMngrObj;
extern float assistForcePctg;
extern AssistStage assistStage;

// 신체 치수 및 동역학 상수 
extern float TrunkLength_Improved;	//m
extern float ThighLength_Improved;	//m
extern float ShankLength_Improved;	//m
extern float AnkleLength_Improved;	//m
extern float ThighMass_Improved;	//kg
extern float ShankMass_Improved;	//kg
extern float TrunkMass_Improved;	//kg
extern float AnkleMass_Improved;	//kg

extern uint32_t assistModeLoopCnt;
extern SuitMode_t CMMode;

extern uint8_t STS_P_Vector_Duration_Completed_RH;
extern uint8_t STS_P_Vector_Duration_Completed_LH;

// 통신 및 상태 관리용 객체 및 변수
ExtCommLinkObj_t g_ext_comm_obj; // 통신 채널 객체
ExtLinkMngrInst_t g_ext_link_mngr; // NMT 상태 관리 객체

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

// 전용 Object Dictionary 배열
static DOP_Dict_t g_ext_dods[CMEXT_LINK_DICT_NUM] = {0};
// CM-SCM 통신 전용 SDO/PDO 테이블 (필요한 만큼 크기 지정)
static SDOInfo_t g_ext_sdo_table[CMEXT_LINK_DICT_NUM][DOP_SDO_MAX_NUM];
static PDOInfo_t g_ext_pdo_table[CMEXT_LINK_DICT_NUM][DOP_PDO_MAX_NUM];

// PDO 전송 리스트 및 메시지 버퍼
// 변경 전: static cvector_vector_type(DOP_Header_t) g_pdo_send_list = NULL;
// static cvector_vector_type(DOP_Header_t) g_pdo_send_list = NULL;
// 변경 후 ▼
static DOP_Header_t g_pdo_send_list[EXT_LINK_PDO_LIST_MAX_SIZE];
static uint8_t      g_pdo_send_list_size = 0;
static DOPI_SDOMsg_t sdo_msg_to_ext;
static DOPI_PDOMsg_t pdo_msg_to_ext;

// 수신(Rx) PDO 데이터를 저장할 원시 버퍼
// (cm_drv.c의 s_cm.rawPdoData와 동일한 역할)
#pragma pack(push, 1)
static ExtLink_PdoRx_ExtToCm_t s_rawPdoRxBuffer;
#pragma pack(pop)

static ScaledData_t ScaledDataObj;

static bool bootup_sent = false;
uint8_t gaitState_test;
uint8_t gaitCycle_test;

static float auxCurrentInput_LH = 0.0f;
static float auxCurrentInput_RH = 0.0f;
static bool addPDOtoStage_LH = true;
static bool addPDOtoStage_RH = true;

static float _extlink_user_weight;
static float _extlink_user_height;
static float _extlink_user_right_thigh_length;
static float _extlink_user_left_thigh_length;
static float _extlink_user_right_shin_length;
static float _extlink_user_left_shin_length;
static float _extlink_user_right_ankle_length;
static float _extlink_user_left_ankle_length;
static bool isSetUserBodyData = false;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

// Object dictionary
static void _ExtLink_CreateDOD(CmExtLinkDickID_t t_dictID);
static void _ExtLink_CreateSDO(CmExtLinkDickID_t t_dictID, uint8_t t_objID, uint8_t t_type, DOP_SDOCB_t t_callback);
static void _ExtLink_CreatePDO(CmExtLinkDickID_t t_dictID, uint8_t t_objID, uint8_t t_type, uint8_t t_size, void* t_addr);
static void SendBootUp(void);
static void SendHeartBeat(uint8_t cmNmtState);
static void SendSyncStates(void);
static void SetAddrDOD(ExtCommLinkObj_t* ext_link);
// SDO/PDO message Unpack/Pack & Send PDO
static int _ExtLink_UnpackSDO(DOPI_DevObj_t* obj, uint8_t* byte_arr);
static int _ExtLink_UnpackPDO(DOPI_DevObj_t* obj, uint8_t* byte_arr);
static void _ExtLink_ParseStaticPDO(uint8_t* data);
static int _ExtLink_ReadSDO(DOPI_DevObj_t* obj, uint8_t* byte_arr);
static int _ExtLink_CallSDO(DOP_SDO_t* sdo_obj, DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static int _ExtLink_ReadPDO(DOPI_DevObj_t* obj, uint8_t* byte_arr);
static DOP_SDO_t* _ExtLink_FindSDO(uint8_t t_dictID, uint8_t t_objID);
static DOP_PDO_t* _ExtLink_FindPDO(uint8_t t_dictID, uint8_t t_objID);
static int _ExtLink_PackPDOs(uint8_t* byte_arr, uint8_t* byte_len);
static int _ExtLink_ConvertPDOToBytes(DOP_Header_t* header, uint8_t* byte_arr);
// For Tx SDO
static int _ExtLink_AppendSDO(DOPI_SDOUnit_t* sdo_unit, DOPI_SDOMsg_t* sdo_msg);
static SDOInfo_t _ExtLink_GetSDOInfo(CmExtLinkDickID_t dict_id, uint8_t obj_id);
// SDO Response
static void TxSDOResponseToExt(DOP_Header_t* req, DOP_SDOArgs_t* res);
// Safe
static void _GoToStoppedState(void);
// Decoding Rx PDO Data
static void _ExtLink_DecodingPDOs(uint8_t t_dictID, uint8_t t_objID);

/* ------------------- SDO CALLBACK ------------------- */
// List Management For PDO List SDO Message Callback
static void Set_Send_PDO_List(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void _ExtLink_ClearPdoSendList(void);
static int _ExtLink_AddPdoToSendList(uint8_t dict_id, uint8_t obj_id);
// NMT SDO Message Callback
static void Get_Ext_Bootup(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_CM_NMT_State(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Get_Ext_Heartbeat(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Get_Ext_SyncState(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
// User Body Data Callback
static void _ExtLink_Set_User_Body_Data(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void _Calculate_Segment_Parameters(void);
static float _ExtLink_checkValue(float value, float min, float max, float defaultValue);
// PIF Vector
// P-vector
static void Set_P_Vector_Yd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_L_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_S0_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Sd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Reset_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Yd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_L_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_S0_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Sd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Reset_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
// I-vector
static void Set_I_Vector_Epsilon_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kp_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Lambda_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Duration_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kp_Max_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kd_Max_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Epsilon_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kp_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Lambda_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Duration_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kp_Max_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kd_Max_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
// F-vector
static void Set_F_Vector_ModeIDX_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_F_Vector_TauMax_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_F_Vector_Delay_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_F_Vector_ModeIDX_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_F_Vector_TauMax_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_F_Vector_Delay_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
// Degree Limit
static void Set_Degree_Limit_Upper_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Degree_Limit_Lower_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Degree_Limit_Upper_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Degree_Limit_Lower_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Degree_Limit_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Clear_Degree_Limit_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Degree_Limit_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Clear_Degree_Limit_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
// Velocity Limit
static void Set_Velocity_Limit_Upper_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Velocity_Limit_Lower_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Velocity_Limit_Upper_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Velocity_Limit_Lower_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Velocity_Limit_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Clear_Velocity_Limit_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Velocity_Limit_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Clear_Velocity_Limit_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
// DOB
static void Set_DOB_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Clear_DOB_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_DOB_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Clear_DOB_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
// Gravity & Velocity Compensation Gain
static void Set_Normal_Comp_Gain_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Normal_Comp_Gain_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Resistive_Comp_Gain_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Resistive_Comp_Gain_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
// 기존 H10 Assist Mode 사용 여부
static void Set_H10_Existing_Assist_Mode(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

/* ------------------- Create Dictionary Table ------------------- */
static void _ExtLink_CreateSDOTable(void);
static void _ExtLink_CreatePDOTable(void);
static void _ExtLink_AssembleSDO(SDOInfo_t* t_addr, uint8_t t_dataType);
static void _ExtLink_AssemblePDO(PDOInfo_t* t_addr, uint8_t t_dataType, uint8_t t_numOfData);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void ExtLinkMngr_Init(void) {
    // 1.  object init (Tx 함수 포인터 할당(fdcan) 및 sdo&pdo tx id 부여)
    g_ext_comm_obj.tx_fnc = IOIF_TransmitFDCAN1;
    g_ext_comm_obj.sdo_tx_id = SDO | (NODE_ID_CM << 4) | NODE_ID_EXTPACK;
    g_ext_comm_obj.pdo_tx_id = PDO | (NODE_ID_CM << 4) | NODE_ID_EXTPACK;
    g_ext_comm_obj.extObj.nodeID = NODE_ID_EXTPACK;
	g_ext_comm_obj.extObj.errCode = NO_ERROR;
	g_ext_comm_obj.extObj.numOfTask = CMEXT_LINK_DICT_NUM;

    // 2. NMT 관리자 초기화
    g_ext_link_mngr.cm_nmt_state = NMT_STATE_INITIALISING;
    g_ext_link_mngr.is_ext_pack_booted = false;
    g_ext_link_mngr.is_ext_pack_connected = false;
    g_ext_link_mngr.last_ext_heartbeat_time_ms = 0;

    // 3. SDO 콜백 및 PDO 주소 등록
    _ExtLink_CreateDOD(CMEXT_LINK_DICT_ID_CM_to_EXT);
    _ExtLink_CreateDOD(CMEXT_LINK_DICT_ID_EXT_to_CM);
    // 확장팩이 호출할 SDO에 대한 콜백 함수를 연결
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_PDO_LIST,           DOP_UINT8,  Set_Send_PDO_List);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_EXT_NMT_BOOTUP,     DOP_UINT8,  Get_Ext_Bootup);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_CM_NMT_STATE,       DOP_UINT8,  Set_CM_NMT_State);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_EXT_NMT_HEARTBEAT,  DOP_UINT8,  Get_Ext_Heartbeat);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SYNC_STATES,        DOP_UINT8,  Get_Ext_SyncState);
    // 신체 정보 수신시 콜백 함수
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_USER_BODY_DATA,    DOP_UINT32, _ExtLink_Set_User_Body_Data);
    // 확장팩에서 PIF Vector 전송시 콜백 함수
    // P-vector
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_YD_RH, DOP_INT16, Set_P_Vector_Yd_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_L_RH, DOP_UINT16, Set_P_Vector_L_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_S0_RH, DOP_UINT8, Set_P_Vector_S0_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_SD_RH, DOP_UINT8, Set_P_Vector_Sd_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_RESET_RH, DOP_UINT8, Set_P_Vector_Reset_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_YD_LH, DOP_INT16, Set_P_Vector_Yd_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_L_LH, DOP_UINT16, Set_P_Vector_L_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_S0_LH, DOP_UINT8, Set_P_Vector_S0_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_SD_LH, DOP_UINT8, Set_P_Vector_Sd_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_RESET_LH, DOP_UINT8, Set_P_Vector_Reset_LH);
    // I-Vector
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_EPSILON_RH, DOP_UINT8, Set_I_Vector_Epsilon_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KP_RH, DOP_UINT8, Set_I_Vector_Kp_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KD_RH, DOP_UINT8, Set_I_Vector_Kd_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_LAMBDA_RH, DOP_UINT8, Set_I_Vector_Lambda_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_DURATION_RH, DOP_UINT16, Set_I_Vector_Duration_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KP_MAX_RH, DOP_FLOAT32, Set_I_Vector_Kp_Max_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KD_MAX_RH, DOP_FLOAT32, Set_I_Vector_Kd_Max_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_EPSILON_LH, DOP_UINT8, Set_I_Vector_Epsilon_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KP_LH, DOP_UINT8, Set_I_Vector_Kp_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KD_LH, DOP_UINT8, Set_I_Vector_Kd_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_LAMBDA_LH, DOP_UINT8, Set_I_Vector_Lambda_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_DURATION_LH, DOP_UINT16, Set_I_Vector_Duration_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KP_MAX_LH, DOP_FLOAT32, Set_I_Vector_Kp_Max_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KD_MAX_LH, DOP_FLOAT32, Set_I_Vector_Kd_Max_LH);
    // F-Vector
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_MODE_IDX_RH, DOP_UINT16, Set_F_Vector_ModeIDX_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_TMAX_RH, DOP_INT16, Set_F_Vector_TauMax_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_DELAY_RH, DOP_UINT16, Set_F_Vector_Delay_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_MODE_IDX_LH, DOP_UINT16, Set_F_Vector_ModeIDX_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_TMAX_LH, DOP_INT16, Set_F_Vector_TauMax_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_DELAY_LH, DOP_UINT16, Set_F_Vector_Delay_LH);
    // Degree Limit ROM
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SET_DEGREE_LIMIT_ROUTINE_RH,   DOP_UINT8, Set_Degree_Limit_Routine_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_DEGREE_LIMIT_ROUTINE_RH, DOP_UINT8, Clear_Degree_Limit_Routine_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SET_DEGREE_LIMIT_ROUTINE_LH,   DOP_UINT8, Set_Degree_Limit_Routine_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_DEGREE_LIMIT_ROUTINE_LH, DOP_UINT8, Clear_Degree_Limit_Routine_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_DEGREE_LIMIT_UPPER_RH, DOP_FLOAT32, Set_Degree_Limit_Upper_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_DEGREE_LIMIT_LOWER_RH, DOP_FLOAT32, Set_Degree_Limit_Lower_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_DEGREE_LIMIT_UPPER_LH, DOP_FLOAT32, Set_Degree_Limit_Upper_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_DEGREE_LIMIT_LOWER_LH, DOP_FLOAT32, Set_Degree_Limit_Lower_LH);
    // Velocity Limit ROM
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SET_VELOCITY_LIMIT_ROUTINE_RH,   DOP_UINT8, Set_Velocity_Limit_Routine_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_VELOCITY_LIMIT_ROUTINE_RH, DOP_UINT8, Clear_Velocity_Limit_Routine_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SET_VELOCITY_LIMIT_ROUTINE_LH,   DOP_UINT8, Set_Velocity_Limit_Routine_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_VELOCITY_LIMIT_ROUTINE_LH, DOP_UINT8, Clear_Velocity_Limit_Routine_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_VELOCITY_LIMIT_UPPER_RH, DOP_FLOAT32, Set_Velocity_Limit_Upper_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_VELOCITY_LIMIT_LOWER_RH, DOP_FLOAT32, Set_Velocity_Limit_Lower_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_VELOCITY_LIMIT_UPPER_LH, DOP_FLOAT32, Set_Velocity_Limit_Upper_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_VELOCITY_LIMIT_LOWER_LH, DOP_FLOAT32, Set_Velocity_Limit_Lower_LH);
    // DOB
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SET_DOB_ROUTINE_RH,    DOP_UINT8, Set_DOB_Routine_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_DOB_ROUTINE_RH,  DOP_UINT8, Clear_DOB_Routine_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SET_DOB_ROUTINE_LH,    DOP_UINT8, Set_DOB_Routine_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_DOB_ROUTINE_LH,  DOP_UINT8, Clear_DOB_Routine_LH);
    // Gravity & Velocity Compensation Gain
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SEND_NORMAL_COMP_GAIN_RH, DOP_UINT8, Set_Normal_Comp_Gain_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SEND_NORMAL_COMP_GAIN_LH, DOP_UINT8, Set_Normal_Comp_Gain_LH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SEND_RESISITIVE_COMP_GAIN_RH, DOP_FLOAT32, Set_Resistive_Comp_Gain_RH);
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_SEND_RESISITIVE_COMP_GAIN_LH, DOP_FLOAT32, Set_Resistive_Comp_Gain_LH);
    // 기존 H10의 FSM 보조 사용 여부
    _ExtLink_CreateSDO(CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_SDO_ID_EXT_to_CM_H10_ORIGINAL_ASSIST_MODE, DOP_UINT8, Set_H10_Existing_Assist_Mode);
    
    // CM->확장팩 PDO
    /* loop Cnt Data */
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_ASSIST_MODE_LOOP_CNT,      DOP_UINT32, 1, &assistModeLoopCnt);
    /* kinematics data */
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_ANGLE,        DOP_INT16, 1, &ScaledDataObj.Scaled_Theta_Hip_LH);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_ANGLE,       DOP_INT16, 1, &ScaledDataObj.Scaled_Theta_Hip_RH);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_THIGH_ANGLE,      DOP_INT16, 1, &userCtrlObj[LH_SAGITAL].data.thighAngleDeg);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_THIGH_ANGLE,     DOP_INT16, 1, &userCtrlObj[RH_SAGITAL].data.thighAngleDeg);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_PELVIC_ANGLE,          DOP_INT16, 1, &ScaledDataObj.Scaled_Theta_Pelvic);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_PELVIC_VEL_Y,          DOP_INT16, 1, &ScaledDataObj.Scaled_Theta_Pelvic_Vel_Y);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_KNEE_ANGLE,       DOP_INT16, 1, &ScaledDataObj.Scaled_Theta_Knee_LH);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_KNEE_ANGLE,      DOP_INT16, 1, &ScaledDataObj.Scaled_Theta_Knee_RH);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_MOTOR_ANGLE, DOP_INT16, 1, &ScaledDataObj.Scaled_Theta_Hip_Motor_RH);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_MOTOR_ANGLE,  DOP_INT16, 1, &ScaledDataObj.Scaled_Theta_Hip_Motor_LH);
    /* gait data */
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_FOOT_CONTACT,     DOP_UINT8, 1, &LeftFootContact_Improved[0]);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_FOOT_CONTACT,    DOP_UINT8, 1, &RightFootContact_Improved[0]);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_GAIT_STATE,            DOP_UINT8, 1, &gaitState_test);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_GAIT_CYCLE,            DOP_UINT8, 1, &gaitCycle_test);
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_FORWARD_VELOCITY,      DOP_INT16, 1, &ScaledDataObj.Scaled_Forward_Vel_X);
    /* SUIT H10 data */
	_ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_TORQUE,       DOP_INT16, 1, &ScaledDataObj.Scaled_ActualCurrentLH);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_TORQUE,      DOP_INT16, 1, &ScaledDataObj.Scaled_ActualCurrentRH);
    /* imu data */
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_FRONTAL_ROLL, DOP_INT16, 1, &ScaledDataObj.Scaled_leftHipImuFrontalRoll);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_FORNTAL_ROLL, DOP_INT16, 1, &ScaledDataObj.Scaled_leftHipImuSagittalPitch);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_SAGITTAL_PITCH, DOP_INT16, 1, &ScaledDataObj.Scaled_rightHipImuFrontalRoll);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_SAGITTAL_PITCH, DOP_INT16, 1, &ScaledDataObj.Scaled_rightHipImuSagittalPitch);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_ACC_X, DOP_INT16, 1, &ScaledDataObj.Scaled_leftHipImuGlobalAccX);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_ACC_Y, DOP_INT16, 1, &ScaledDataObj.Scaled_leftHipImuGlobalAccY);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_ACC_Z, DOP_INT16, 1, &ScaledDataObj.Scaled_leftHipImuGlobalAccZ);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_GYR_X, DOP_INT16, 1, &ScaledDataObj.Scaled_leftHipImuGlobalGyrX);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_GYR_Y, DOP_INT16, 1, &ScaledDataObj.Scaled_leftHipImuGlobalGyrY);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_GYR_Z, DOP_INT16, 1, &ScaledDataObj.Scaled_leftHipImuGlobalGyrZ);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_ACC_X, DOP_INT16, 1, &ScaledDataObj.Scaled_rightHipImuGlobalAccX);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_ACC_Y, DOP_INT16, 1, &ScaledDataObj.Scaled_rightHipImuGlobalAccY);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_ACC_Z, DOP_INT16, 1, &ScaledDataObj.Scaled_rightHipImuGlobalAccZ);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_GYR_X, DOP_INT16, 1, &ScaledDataObj.Scaled_rightHipImuGlobalGyrX);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_GYR_Y, DOP_INT16, 1, &ScaledDataObj.Scaled_rightHipImuGlobalGyrY);
    _ExtLink_CreatePDO(CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_GYR_Z, DOP_INT16, 1, &ScaledDataObj.Scaled_rightHipImuGlobalGyrZ);
    // SDO & PDO DataType 및 Data Size Assemble
    _ExtLink_CreateSDOTable();
    _ExtLink_CreatePDOTable();
    // 4. CM->확장팩 SDO 및 확장팩->CM PDO 주소 설정
    SetAddrDOD(&g_ext_comm_obj);
}

void ExtLinkMngr_Execute(void) 
{
    uint32_t current_time = IOIF_GetTick();
    // 상태 머신 실행
    switch (g_ext_link_mngr.cm_nmt_state) {
        case NMT_STATE_INITIALISING:
            // TODO : Control Module의 기타 API 모듈 초기화 모두 완료된 후 수행(in wholebodyctrl state enable)
            // 확장팩의 Boot-up 메세지 수신 후 CM NMT State가 callback에 의해 Pre-Operational로 진입
            // 수동적 대기 상태
            // TODO : 토크 입력 0 및 모터 구동 정지
            g_ext_comm_obj.data.motor_auxiliary_input_LH = 0;
            g_ext_comm_obj.data.motor_auxiliary_input_RH = 0;
            break;
        case NMT_STATE_PRE_OPERATIONAL:
            // 첫 진입 시 CM의 Boot-up 메시지 단 한 번 확장팩에 전송
            if (!bootup_sent && g_ext_link_mngr.is_ext_pack_booted) {
                // CM의 부팅 완료를 알리는 Boot-up SDO 전송
                SendBootUp();
                bootup_sent = true;
            }

            // 확장팩의 CM PDO List Set SDO 메시지 수신시 callback을 통해 Response 메시지 송신
            // 확장팩의 NMT 상태 변경 명령을 기다림
            // 확장팩 Heartbeat 타임아웃 감시
            if (current_time - g_ext_link_mngr.last_ext_heartbeat_time_ms > 5000) { /* 5초 */
                _GoToStoppedState();
            }
            break;
        case NMT_STATE_OPERATIONAL:
            // 확장팩 Heartbeat 타임아웃 감시
            if (current_time - g_ext_link_mngr.last_ext_heartbeat_time_ms > 1000) { /* 1초 */
                _GoToStoppedState();
                break;
            }

            // CM의 주기적 Heartbeat SDO 전송
            static uint32_t last_cm_hb_time = 0;
            if (current_time - last_cm_hb_time >= 200) { // 200ms 마다 전송
                SendHeartBeat(g_ext_link_mngr.cm_nmt_state);
                last_cm_hb_time = current_time;
            }
            break;
        case NMT_STATE_STOPPED:
            // 슬레이브는 문제가 생기면 여기서 수동적으로 대기.
            // 마스터(XM)가 Boot-up을 다시 보내주기를 기다린다.
            // 확장팩의 Boot-up 메세지 수신 후 CM NMT State가 callback에 의해 Pre-Operational로 진입
            // TODO : 토크 입력 0 및 모터 구동 정지
            g_ext_comm_obj.data.motor_auxiliary_input_LH = 0;
            g_ext_comm_obj.data.motor_auxiliary_input_RH = 0;
            break;
    }
}

bool ExtLinkMngr_IsConnected(const ExtLinkMngrInst_t* inst) {
    return inst->is_ext_pack_connected;
}

/**
 * @brief 확장팩으로 SDO 메시지를 전송하는 헬퍼 함수
 */
int TxSDOToExtPack(ExtCommLinkObj_t* ext_link, DOPI_SDOMsg_t* msg) {
    return ext_link->tx_fnc(ext_link->sdo_tx_id, msg->txBuf, msg->msgLength);
}

/**
 * @brief 확장팩으로 PDO 메시지를 전송하는 헬퍼 함수
 */
int TxPDOToExtPack(ExtCommLinkObj_t* ext_link, DOPI_PDOMsg_t* msg) {
    return ext_link->tx_fnc(ext_link->pdo_tx_id, msg->txBuf, msg->msgLength);
}

/**
 * @brief 확장보드로부터 수신된 CAN 메시지를 처리합니다. (AS_FDCAN_CB에서 호출됨)
 * @note  DOPI_UnpackSDO/PDO 로직을 참고하여 확장팩 전용으로 재구성했습니다.
 */
void ExtLinkMngr_ProcessCANMessage(uint16_t can_id, uint8_t* data) 
{
    uint16_t fncCode = can_id & 0x700;

    switch (fncCode) {
        case SDO: {
            // 확장팩이 보낸 SDO 요청을 처리하고, 등록된 콜백을 실행
            _ExtLink_UnpackSDO(&g_ext_comm_obj.extObj, data);
            break;
        }
        case PDO: {
            // // 확장팩이 보낸 PDO 데이터를 처리
            // _ExtLink_UnpackPDO(&g_ext_comm_obj.extObj, data);
            // [변경] 동적 UnpackPDO 대신 정적 파서 호출
            _ExtLink_ParseStaticPDO(data);
            break;
        }
        // ... EMCY, NOTI 등 다른 Function Code 처리 ...
    }
}


void ScalingForPDOSendToExt(void) {
	// Scaling for PDO Data(2Byte)
    ScaledDataObj.Scaled_Theta_Hip_RH = ScaleFloatToInt16(Theta_Hip_RH, DEG_SCALING_FACTOR);
    ScaledDataObj.Scaled_Theta_Hip_LH = ScaleFloatToInt16(Theta_Hip_LH, DEG_SCALING_FACTOR);
    ScaledDataObj.Scaled_Theta_Pelvic = ScaleFloatToInt16(Theta_Trunk_Improved, DEG_SCALING_FACTOR);
    ScaledDataObj.Scaled_Theta_Pelvic_Vel_Y = ScaleFloatToInt16(pelvic_Vel_Y, VELDEG_SCALING_FACTOR);
    ScaledDataObj.Scaled_Theta_Knee_RH = ScaleFloatToInt16(ThetaLK_Improved[0], DEG_SCALING_FACTOR);
    ScaledDataObj.Scaled_Theta_Knee_LH = ScaleFloatToInt16(ThetaRK_Improved[0], DEG_SCALING_FACTOR);
    ScaledDataObj.Scaled_Forward_Vel_X = ScaleFloatToInt16(velX_Improved[0], FORWARD_VEL_X_SCALING_FACTOR);
    ScaledDataObj.Scaled_Theta_Hip_Motor_RH = ScaleFloatToInt16(RH_Sagittal.PositionAct, DEG_SCALING_FACTOR);
    ScaledDataObj.Scaled_Theta_Hip_Motor_LH = ScaleFloatToInt16(LH_Sagittal.PositionAct, DEG_SCALING_FACTOR);
    ScaledDataObj.Scaled_ActualCurrentRH = ScaleFloatToInt16(RH_Sagittal.MotorActCurrent, CURRENT_SCALING_FACTOR);
    ScaledDataObj.Scaled_ActualCurrentLH = ScaleFloatToInt16(LH_Sagittal.MotorActCurrent, CURRENT_SCALING_FACTOR);

    ScaledDataObj.Scaled_leftHipImuFrontalRoll = ScaleFloatToInt16(LH_Sagittal.IMU_BodyAngle_Frontal, DEG_SCALING_FACTOR);
	ScaledDataObj.Scaled_leftHipImuSagittalPitch = ScaleFloatToInt16(LH_Sagittal.IMU_BodyAngle_Sagittal, DEG_SCALING_FACTOR);
	ScaledDataObj.Scaled_rightHipImuFrontalRoll = ScaleFloatToInt16(RH_Sagittal.IMU_BodyAngle_Frontal, DEG_SCALING_FACTOR);
	ScaledDataObj.Scaled_rightHipImuSagittalPitch = ScaleFloatToInt16(RH_Sagittal.IMU_BodyAngle_Sagittal, DEG_SCALING_FACTOR);
	ScaledDataObj.Scaled_leftHipImuGlobalAccX = ScaleFloatToInt16(RH_Sagittal.accXGlobal, ACC_SCALING_FACTOR);
	ScaledDataObj.Scaled_leftHipImuGlobalAccY = ScaleFloatToInt16(RH_Sagittal.accYGlobal, ACC_SCALING_FACTOR);
	ScaledDataObj.Scaled_leftHipImuGlobalAccZ = ScaleFloatToInt16(RH_Sagittal.accZGlobal, ACC_SCALING_FACTOR);
	ScaledDataObj.Scaled_leftHipImuGlobalGyrX = ScaleFloatToInt16(RH_Sagittal.gyroXGlobal, GYR_SCALING_FACTOR);
	ScaledDataObj.Scaled_leftHipImuGlobalGyrY = ScaleFloatToInt16(RH_Sagittal.gyroYGlobal, GYR_SCALING_FACTOR);
	ScaledDataObj.Scaled_leftHipImuGlobalGyrZ = ScaleFloatToInt16(RH_Sagittal.gyroZGlobal, GYR_SCALING_FACTOR);
	ScaledDataObj.Scaled_rightHipImuGlobalAccX = ScaleFloatToInt16(RH_Sagittal.accXGlobal, ACC_SCALING_FACTOR);
	ScaledDataObj.Scaled_rightHipImuGlobalAccY = ScaleFloatToInt16(RH_Sagittal.accYGlobal, ACC_SCALING_FACTOR);
	ScaledDataObj.Scaled_rightHipImuGlobalAccZ = ScaleFloatToInt16(RH_Sagittal.accZGlobal, ACC_SCALING_FACTOR);
	ScaledDataObj.Scaled_rightHipImuGlobalGyrX = ScaleFloatToInt16(RH_Sagittal.gyroXGlobal, GYR_SCALING_FACTOR);
	ScaledDataObj.Scaled_rightHipImuGlobalGyrY = ScaleFloatToInt16(RH_Sagittal.gyroYGlobal, GYR_SCALING_FACTOR);
	ScaledDataObj.Scaled_rightHipImuGlobalGyrZ = ScaleFloatToInt16(RH_Sagittal.gyroZGlobal, GYR_SCALING_FACTOR);
}

void SendSUITMode(uint8_t suit_mode) 
{
    DOPI_SDOUnit_t suitmode_sdo;
    DOPI_ClearSDO(&sdo_msg_to_ext);

    g_ext_comm_obj.data.suit_mode = suit_mode;
    suitmode_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_SUIT_MODE, SDO_REQU, 1);
    _ExtLink_AppendSDO(&suitmode_sdo, &sdo_msg_to_ext);

    TxSDOToExtPack(&g_ext_comm_obj, &sdo_msg_to_ext);
}

void SendSUITAssistLevel(uint8_t suit_assist_level) 
{
    DOPI_SDOUnit_t suitassistmode_sdo;
    DOPI_ClearSDO(&sdo_msg_to_ext);

    g_ext_comm_obj.data.suit_assist_level = suit_assist_level;
    suitassistmode_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_ASSIST_LEVEL, SDO_REQU, 1);
    _ExtLink_AppendSDO(&suitassistmode_sdo, &sdo_msg_to_ext);

    TxSDOToExtPack(&g_ext_comm_obj, &sdo_msg_to_ext);
}

void SendPVectorDurationCompleted_RH(uint8_t P_Vector_Duration_Completed_RH) 
{
    DOPI_SDOUnit_t PvectorDuration_sdo;
    DOPI_ClearSDO(&sdo_msg_to_ext);

    g_ext_comm_obj.data.P_Vector_Duration_Completed_RH = P_Vector_Duration_Completed_RH;
    PvectorDuration_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_P_VECTOR_DURATION_COMPLETED_RH, SDO_REQU, 1);
    _ExtLink_AppendSDO(&PvectorDuration_sdo, &sdo_msg_to_ext);

    TxSDOToExtPack(&g_ext_comm_obj, &sdo_msg_to_ext);
}

void SendPVectorDurationCompleted_LH(uint8_t P_Vector_Duration_Completed_LH) 
{
    DOPI_SDOUnit_t PvectorDuration_sdo;
    DOPI_ClearSDO(&sdo_msg_to_ext);

    g_ext_comm_obj.data.P_Vector_Duration_Completed_LH = P_Vector_Duration_Completed_LH;
    PvectorDuration_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_P_VECTOR_DURATION_COMPLETED_LH, SDO_REQU, 1);
    _ExtLink_AppendSDO(&PvectorDuration_sdo, &sdo_msg_to_ext);

    TxSDOToExtPack(&g_ext_comm_obj, &sdo_msg_to_ext);
}

/**
 * @brief CM -> XM으로 PDO를 "정적 매핑" 방식으로 전송합니다. (수정됨)
 * @details PnP에서 약속된 "순서도(g_pdo_send_list)"를 순회하며,
 * [ID] 헤더 없이 [Data]만 순서대로 조립하여 전송합니다.
 */
void _ExtLink_SendPDOs(void) 
{
    // _ExtLink_PackPDOs(&pdo_msg_to_ext.txBuf[0], &pdo_msg_to_ext.msgLength);
    // if (pdo_msg_to_ext.msgLength != 1) {
	// 	if (TxPDOToExtPack(&g_ext_comm_obj, &pdo_msg_to_ext) == 0) {
	// 		//TODO: MSG TX ERROR
	// 	}
    // }
    // 1. 전송 버퍼(임시) 및 커서 준비
    uint8_t txBuffer[64];
    uint8_t cursor = 0;

    // 2. PnP에서 약속된 "순서도(g_pdo_send_list)"를 순회
    for (size_t i = 0; i < g_pdo_send_list_size; ++i) {
        DOP_Header_t header = g_pdo_send_list[i];
        DOP_PDO_t* pdo_obj = _ExtLink_FindPDO(header.dictID, header.objID);

        if (pdo_obj && pdo_obj->addr) {
            // 3. 페이로드 크기 확인
            if (cursor + pdo_obj->objSize > 64) {
                break; // 64바이트 초과
            }
            // 4. [ID 없이] 데이터만 순서대로 복사
            memcpy(&txBuffer[cursor], pdo_obj->addr, pdo_obj->objSize);
            cursor += pdo_obj->objSize;
        }
    }

    // 5. 조립된 순수 데이터 페이로드 전송
    if (cursor > 0) {
        g_ext_comm_obj.tx_fnc(g_ext_comm_obj.pdo_tx_id, txBuffer, (uint32_t)cursor);
    }
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief  전용 DOD 테이블에 Task ID(dictID)를 부여
 * @param  task_id  전용 Task ID (CmExtLinkDickID_t)
 */
static void _ExtLink_CreateDOD(CmExtLinkDickID_t t_dictID) {
    g_ext_dods[t_dictID].dictID = t_dictID;
}

/**
 * @brief  전용 SDO 테이블에 SDO 객체를 생성하고 콜백을 등록합니다.
 * @param  task_id  전용 Task ID (t_dictID)
 * @param  obj_id   전용 SDO ID
 * @param  type     데이터 타입 (DOP_UINT8 등)
 * @param  cb       실행할 콜백 함수 포인터
 */
static void _ExtLink_CreateSDO(CmExtLinkDickID_t t_dictID, uint8_t t_objID, uint8_t t_type, DOP_SDOCB_t t_callback) {
    g_ext_dods[t_dictID].SDOs[t_objID].objID            = t_objID;
    g_ext_dods[t_dictID].SDOs[t_objID].dataType         = t_type;
    g_ext_dods[t_dictID].SDOs[t_objID].callback         = t_callback;
    g_ext_dods[t_dictID].SDOs[t_objID].args.status      = DOP_SDO_IDLE;
	g_ext_dods[t_dictID].SDOs[t_objID].args.dataSize    = 0;
	g_ext_dods[t_dictID].SDOs[t_objID].args.data        = NULL;
	g_ext_dods[t_dictID].SDOs[t_objID].args.typeSize    = DOP_GetDataTypeInfo(t_type).typeSize;
}

/**
 * @brief  전용 PDO 테이블에 PDO 객체를 생성하고 변수 주소를 연결합니다.
 * @param  task_id  전용 Task ID (t_dictID)
 * @param  obj_id   전용 PDO ID
 * @param  type     데이터 타입
 * @param  num      데이터 개수
 * @param  addr     연결할 변수의 주소
 */
static void _ExtLink_CreatePDO(CmExtLinkDickID_t t_dictID, uint8_t t_objID, uint8_t t_type, uint8_t t_size, void* t_addr) {
    g_ext_dods[t_dictID].PDOs[t_objID].objID     = t_objID;
    g_ext_dods[t_dictID].PDOs[t_objID].dataType  = t_type;
    g_ext_dods[t_dictID].PDOs[t_objID].dataSize  = t_size;
    g_ext_dods[t_dictID].PDOs[t_objID].addr      = t_addr;
	g_ext_dods[t_dictID].PDOs[t_objID].objSize   = DOP_GetDataTypeInfo(t_type).typeSize * t_size;
	g_ext_dods[t_dictID].PDOs[t_objID].lastPub   = malloc(g_ext_dods[t_dictID].PDOs[t_objID].objSize);

    memset(g_ext_dods[t_dictID].PDOs[t_objID].lastPub, 0xFF, g_ext_dods[t_dictID].PDOs[t_objID].objSize);
}

static void SendBootUp(void) 
{
    DOPI_SDOUnit_t bootup_sdo;
    DOPI_ClearSDO(&sdo_msg_to_ext);

    bootup_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_CM_NMT_BOOTUP, SDO_REQU, 0);
    _ExtLink_AppendSDO(&bootup_sdo, &sdo_msg_to_ext);

    TxSDOToExtPack(&g_ext_comm_obj, &sdo_msg_to_ext);
}

static void SendHeartBeat(uint8_t cmNmtState) 
{
    DOPI_SDOUnit_t hb_sdo;
    DOPI_ClearSDO(&sdo_msg_to_ext);

    g_ext_comm_obj.data.cm_nmt_state = cmNmtState;
    hb_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_CM_NMT_HEARTBEAT, SDO_REQU, 1);
    _ExtLink_AppendSDO(&hb_sdo, &sdo_msg_to_ext);

    TxSDOToExtPack(&g_ext_comm_obj, &sdo_msg_to_ext);
}

static void SendSyncStates(void)
{
    DOPI_SDOUnit_t sync_sdo;
    DOPI_ClearSDO(&sdo_msg_to_ext);

    sync_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_SYNC_STATES, SDO_REQU, 0);
    _ExtLink_AppendSDO(&sync_sdo, &sdo_msg_to_ext);

    g_ext_comm_obj.data.suit_mode = CMMode;
    sync_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_SUIT_MODE, SDO_REQU, 1);
    _ExtLink_AppendSDO(&sync_sdo, &sdo_msg_to_ext);

    g_ext_comm_obj.data.suit_assist_level = assistStage;
    sync_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_ASSIST_LEVEL, SDO_REQU, 1);
    _ExtLink_AppendSDO(&sync_sdo, &sdo_msg_to_ext);

    g_ext_comm_obj.data.P_Vector_Duration_Completed_RH = STS_P_Vector_Duration_Completed_RH;
    sync_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_P_VECTOR_DURATION_COMPLETED_RH, SDO_REQU, 1);
    _ExtLink_AppendSDO(&sync_sdo, &sdo_msg_to_ext);

    g_ext_comm_obj.data.P_Vector_Duration_Completed_LH = STS_P_Vector_Duration_Completed_LH;
    sync_sdo = DOPI_CreateSDOUnit(&g_ext_comm_obj.extObj, CMEXT_LINK_DICT_ID_CM_to_EXT,
        CMEXT_LINK_SDO_ID_CM_to_EXT_P_VECTOR_DURATION_COMPLETED_LH, SDO_REQU, 1);
    _ExtLink_AppendSDO(&sync_sdo, &sdo_msg_to_ext);

    TxSDOToExtPack(&g_ext_comm_obj, &sdo_msg_to_ext);
}

/**
 * @brief 확장팩 통신 객체의 데이터 주소를 Object Dictionary에 등록
 */
static void SetAddrDOD(ExtCommLinkObj_t* ext_link) 
{
	/* SDO 송신 메시지 */
	// SDO (CM->확장팩으로 전달하는 SDO 메시지)
	// NMT
	DOPI_SetSDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_SDO_ID_CM_to_EXT_CM_NMT_BOOTUP,     &ext_link->data.cm_bootup);
	DOPI_SetSDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_SDO_ID_CM_to_EXT_CM_NMT_STATE, 		&ext_link->data.cm_nmt_state);
	DOPI_SetSDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_SDO_ID_CM_to_EXT_CM_NMT_HEARTBEAT, 	&ext_link->data.cm_nmt_state);
    DOPI_SetSDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_SDO_ID_CM_to_EXT_SYNC_STATES, 	    &ext_link->data.cm_sync_states);
    // SUIT H10
    DOPI_SetSDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_SDO_ID_CM_to_EXT_SUIT_MODE,     &ext_link->data.suit_mode);
	DOPI_SetSDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_SDO_ID_CM_to_EXT_ASSIST_LEVEL, 	&ext_link->data.suit_assist_level);
    DOPI_SetSDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_SDO_ID_CM_to_EXT_P_VECTOR_DURATION_COMPLETED_RH, 	&ext_link->data.P_Vector_Duration_Completed_RH);
    DOPI_SetSDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_CM_to_EXT, CMEXT_LINK_SDO_ID_CM_to_EXT_P_VECTOR_DURATION_COMPLETED_LH, 	&ext_link->data.P_Vector_Duration_Completed_LH);

    /* PDO 수신 메시지 */
	// PDO (확장팩->CM으로 전달되는 PDO 메시지)
	DOPI_SetPDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_PDO_ID_EXT_to_CM_LEFT_AUX_INPUT, &s_rawPdoRxBuffer.auxTorqueInputLH);
	DOPI_SetPDOAddr(&ext_link->extObj, CMEXT_LINK_DICT_ID_EXT_to_CM, CMEXT_LINK_PDO_ID_EXT_to_CM_RIGHT_AUX_INPUT, &s_rawPdoRxBuffer.auxTorqueInputRH);
}

/**
 * @brief 수신된 SDO 메시지 전체를 언패킹합니다.
 */
static int _ExtLink_UnpackSDO(DOPI_DevObj_t* obj, uint8_t* byte_arr) {
    int cursor = 0;
    // Get # of SDOs
    uint8_t num_of_sdo = 0;
    memcpy(&num_of_sdo, &byte_arr[cursor++], 1);

    // Call & Respond SDOs
    for (int i = 0; i < num_of_sdo; ++i) {
        int bytes_read = _ExtLink_ReadSDO(obj, &byte_arr[cursor]);
        if (bytes_read > 0) {
            cursor += bytes_read;
        } else {
            //TODO: Unpack SDO ERROR
            return DOP_STATUS_SDO_RECV_FAIL; // SDO 처리 실패
        }
    }
    return DOP_STATUS_SUCCESS; // 성공
}

/**
 * @brief 수신된 PDO 메시지 전체를 언패킹합니다. (DOPI_UnpackPDO 로직 재활용)
 */
static int _ExtLink_UnpackPDO(DOPI_DevObj_t* obj, uint8_t* byte_arr) {
    int cursor = 0;

    // Get # of PDOs
    uint8_t num_of_pdo = 0;
    memcpy(&num_of_pdo, &byte_arr[cursor++], 1);

    for (int i = 0; i < num_of_pdo; ++i) {
        int bytes_read = _ExtLink_ReadPDO(obj, &byte_arr[cursor]);
        if (bytes_read > 0) {
            cursor += bytes_read;
        } else {
            return DOP_STATUS_PDO_RECV_FAIL; // PDO 처리 실패
        }
    }

    // 보조모드일 때만 MD로 토크 입력 전송
    if (CMMode == SUIT_ASSIST_MODE) {
        // Send to Aux input LH
        if (addPDOtoStage_LH == true && addPDOtoStage_RH == false) {
            Send_AUX_Torque(LH_SAGITAL, auxCurrentInput_LH);
        }
        // Send to Aux input RH
        if (addPDOtoStage_LH == false && addPDOtoStage_RH == true) {
            Send_AUX_Torque(RH_SAGITAL, auxCurrentInput_RH);
        }
        // Send to Aux input LH & RH
        if (addPDOtoStage_LH == true && addPDOtoStage_RH == true) {
            Send_AUX_Torque(LH_SAGITAL, auxCurrentInput_LH);
            Send_AUX_Torque(RH_SAGITAL, auxCurrentInput_RH);
        }
        addPDOtoStage_LH = false;
        addPDOtoStage_RH = false;        
    }

    return DOP_STATUS_SUCCESS; // 성공
}

/**
 * @brief [신규] XM으로부터 수신된 정적 PDO 페이로드를 파싱합니다.
 */
static void _ExtLink_ParseStaticPDO(uint8_t* data)
{
    // // 1. 수신된 페이로드의 길이가 약속된 구조체의 크기와 일치하는지 확인
    // if (len != sizeof(ExtLink_PdoRx_ExtToCm_t)) {
    //     // [ERROR LOG] PnP 약속과 페이로드 크기가 다름
    //     return;
    // }

    // 2. 수신된 데이터를 원시 버퍼(s_rawPdoRxBuffer)로 통째로 복사
    memcpy(&s_rawPdoRxBuffer, data, sizeof(ExtLink_PdoRx_ExtToCm_t));

    // 3. PDO 데이터를 수신했으므로 디코딩 및 처리를 수행
    auxCurrentInput_LH = ((float)s_rawPdoRxBuffer.auxTorqueInputLH) / 100.f;
    auxCurrentInput_LH = auxCurrentInput_LH / H10_SUIT_GEAR_RATIO / TORQUE_CONSTANT_SAM;
    auxCurrentInput_LH = fminf(10.0f, fmaxf(-10.0f, auxCurrentInput_LH)); 

    auxCurrentInput_RH = ((float)s_rawPdoRxBuffer.auxTorqueInputRH) / 100.f;
    auxCurrentInput_RH = auxCurrentInput_RH / H10_SUIT_GEAR_RATIO / TORQUE_CONSTANT_SAM;
    auxCurrentInput_RH = fminf(10.0f, fmaxf(-10.0f, auxCurrentInput_RH));

    // 4. 모터 드라이버로 전송
    if (CMMode == SUIT_ASSIST_MODE) {
        // Send to Aux input LH
        if (addPDOtoStage_LH == true && addPDOtoStage_RH == false) {
            Send_AUX_Torque(LH_SAGITAL, auxCurrentInput_LH);
        }
        // Send to Aux input RH
        if (addPDOtoStage_LH == false && addPDOtoStage_RH == true) {
            Send_AUX_Torque(RH_SAGITAL, auxCurrentInput_RH);
        }
        // Send to Aux input LH & RH
        if (addPDOtoStage_LH == true && addPDOtoStage_RH == true) {
            Send_AUX_Torque(LH_SAGITAL, auxCurrentInput_LH);
            Send_AUX_Torque(RH_SAGITAL, auxCurrentInput_RH);
        }
    }
}

/**
 * @brief SDO 메시지 하나를 읽고 처리합니다. (기존 ReadSDO 로직 재활용)
 * @param byte_arr SDO 하나(헤더+데이터)의 시작 포인터
 * @return 읽은 바이트 수, 에러 시 음수
 */
static int _ExtLink_ReadSDO(DOPI_DevObj_t* obj, uint8_t* byte_arr) {
    int bytes_read = 0;
    
    // 1. 헤더 파싱 (기존 함수 재사용)
    DOP_Header_t header = GetHeader(byte_arr);
    bytes_read += sizeof(DOP_Header_t);
    
    // 2. 전용 테이블에서 SDO 객체 검색 (전용 함수 사용)
    DOP_SDO_t* t_sdo = _ExtLink_FindSDO(header.dictID, header.objID);
    if (t_sdo == NULL) {
        return -1; // 처리할 SDO가 아님
    }

    // 3. 요청 인자 파싱 (기존 함수 재사용)
    uint16_t req_bytes = 0;
    DOP_SDOArgs_t req_args = Bytes2SDOreq(byte_arr + bytes_read, &req_bytes);
    req_args.typeSize = t_sdo->args.typeSize; // Copy SDO info
    bytes_read += req_bytes;
    req_args.nodeID = obj->nodeID;
    
    // 4. 요청(Request) 처리
    if (req_args.status == DOP_SDO_REQU) {
        DOP_SDOArgs_t res_args = {0}; // 응답을 담을 구조체
        
        // 4-1. 콜백 실행 (전용 함수 사용)하고 처리된 바이트 수(n_bytes)를 받음
        int n_bytes = _ExtLink_CallSDO(t_sdo, &req_args, &res_args);
        if (n_bytes < 0) {
        	return -1;
        }
        
        bytes_read += n_bytes; // 커서를 데이터 길이만큼 이동

        // 4-2. 콜백 실행 후 즉시 응답 SDO 전송
        if (res_args.status != DOP_SDO_IDLE) {
            // Set PDO List SDO 메시지에 대해서만 response
            if (header.dictID == CMEXT_LINK_DICT_ID_EXT_to_CM && header.objID == CMEXT_LINK_SDO_ID_EXT_to_CM_PDO_LIST) {
                TxSDOResponseToExt(&header, &res_args);
            }
        }
    } 
    // 5. 응답(Response) 처리 (CM은 확장팩에 대해 Slave이므로 이 로직은 실행될 일 없음)
    else if (req_args.status == DOP_SDO_SUCC || req_args.status == DOP_SDO_FAIL) {
        // 구현한다면 _ExtLink_SetSDOArgs 함수를 만들어 호출
    }

    return bytes_read;
}

/**
 * @brief 전용 SDO 객체에 등록된 콜백 함수를 실행하고, 처리된 데이터 길이를 반환합니다.
 * (DOP_CallSDO 역할)
 */
static int _ExtLink_CallSDO(DOP_SDO_t* sdo_obj, DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
    if (sdo_obj == NULL || sdo_obj->callback == NULL) {
        return -1; // 처리할 콜백이 없음
    }
    // 1. 등록된 콜백 함수 실행
    sdo_obj->callback(req, res);
    
    // 2. 처리된 요청 데이터의 바이트 크기를 계산하여 반환
    uint8_t type_size = DOP_GetDataTypeInfo(sdo_obj->dataType).typeSize;
    return req->dataSize * type_size;
}

/**
 * @brief PDO 메시지 하나를 읽고 처리합니다. (기존 ReadPDO 로직 재활용)
 */
static int _ExtLink_ReadPDO(DOPI_DevObj_t* obj, uint8_t* byte_arr) {
    int cursor = 0;
    DOP_Header_t header;

    // 1. 헤더 파싱
    memcpy(&header, &byte_arr[cursor], sizeof(DOP_Header_t));
    cursor += sizeof(DOP_Header_t);
    
    // 2. 전용 테이블에서 PDO 객체 검색
    DOP_PDO_t* pdo_obj = _ExtLink_FindPDO(header.dictID, header.objID);
    if (pdo_obj == NULL) {
        return -1;
    }
    
    // 3. 등록된 변수에 데이터 복사
    // pdo_obj->objSize는 CreatePDO 시점에 미리 계산된 값
    uint8_t t_size = DOP_ConvertDataSize(g_ext_pdo_table[header.dictID][header.objID][0]) * g_ext_pdo_table[header.dictID][header.objID][1];
    memcpy(obj->tasks[header.dictID].PDOsAddr[header.objID], &byte_arr[cursor], t_size);
    cursor += t_size;

    // TODO : Aux Torque input PDO 메시지 수신때마다 motor 전송
    _ExtLink_DecodingPDOs(header.dictID, header.objID);

    return cursor;
}

/**
 * @brief  전용 SDO 테이블에서 SDO 정보를 찾습니다.
 * @return 성공 시 SDO 정보 포인터, 실패 시 NULL
 */
static DOP_SDO_t* _ExtLink_FindSDO(uint8_t t_dictID, uint8_t t_objID) {
   	return &g_ext_dods[t_dictID].SDOs[t_objID];
}

/**
 * @brief  전용 PDO 테이블에서 PDO 정보를 찾습니다.
 * @return 성공 시 PDO 정보 포인터, 실패 시 NULL
 */
static DOP_PDO_t* _ExtLink_FindPDO(uint8_t t_dictID, uint8_t t_objID) {
	return &g_ext_dods[t_dictID].PDOs[t_objID];
}


/**
 * @brief  전송 목록에 따라 여러 데이터 PDO를 하나의 CAN 메시지로 패킹합니다. (정적 배열 버전)
 * @param  byte_arr 데이터를 채울 전체 CAN 메시지 버퍼
 * @param  byte_len 최종적으로 패킹된 데이터의 총 길이(byte)
 * @return 패킹된 PDO의 총 개수
 */
static int _ExtLink_PackPDOs(uint8_t* byte_arr, uint8_t* byte_len) {
    // 전송할 목록이 비어있으면 아무것도 하지 않음
    // if (g_pdo_send_list == NULL || cvector_empty(g_pdo_send_list)) {
    //     *byte_len = 0;
    //     return 0;
    // }
    if (g_pdo_send_list_size == 0) {
        *byte_len = 0;
        return DOP_PDO_FAULT;
    }

    int cursor = 0;
    uint8_t numOfPDO = 0;
    cursor += 1; // PDO 개수(1 byte) 자리를 비워둠

    // 전송 목록(g_pdo_send_list)을 순회하며 PDO 패킹
    // cvector_size 대신 g_pdo_send_list_size 사용
    // for (size_t i = 0; i < cvector_size(g_pdo_send_list); ++i) {
    for (size_t i = 0; i < g_pdo_send_list_size; ++i) {
        int bytes_written = _ExtLink_ConvertPDOToBytes(&g_pdo_send_list[i], &byte_arr[cursor]);
        if (bytes_written > 0) {
            // CAN-FD 최대 페이로드(64바이트)를 초과하지 않는지 확인
            if (cursor + bytes_written > 64) {
                break;
            }
            cursor += bytes_written;
            ++numOfPDO;
        }
    }
    
    // 최종 PDO 개수를 버퍼의 맨 앞에 기록
   byte_arr[0] = numOfPDO;
	*byte_len = cursor;

    return DOP_SUCCESS;
}

static void _ExtLink_DecodingPDOs(uint8_t t_dictID, uint8_t t_objID) 
{
    if (t_dictID == CMEXT_LINK_DICT_ID_EXT_to_CM &&  t_objID == CMEXT_LINK_PDO_ID_EXT_to_CM_LEFT_AUX_INPUT) {
        // auxCurrentInput_LH = ((float)g_ext_comm_obj.data.motor_auxiliary_input_LH) / 100.f; // int16 -> float
        auxCurrentInput_LH = ((float)s_rawPdoRxBuffer.auxTorqueInputLH) / 100.f; // int16 -> float        
        auxCurrentInput_LH = auxCurrentInput_LH / H10_SUIT_GEAR_RATIO / TORQUE_CONSTANT_SAM;
        auxCurrentInput_LH = fminf(10.0f, fmaxf(-10.0f, auxCurrentInput_LH)); // 14A 입력 제한
        // 전송 대기열에 추가
        addPDOtoStage_LH = true;
    }

    if (t_dictID == CMEXT_LINK_DICT_ID_EXT_to_CM && t_objID == CMEXT_LINK_PDO_ID_EXT_to_CM_RIGHT_AUX_INPUT) {
        // auxCurrentInput_RH = ((float)g_ext_comm_obj.data.motor_auxiliary_input_RH) / 100.f; // int16 -> float
        auxCurrentInput_LH = ((float)s_rawPdoRxBuffer.auxTorqueInputRH) / 100.f; // int16 -> float  
        auxCurrentInput_RH = auxCurrentInput_RH / H10_SUIT_GEAR_RATIO / TORQUE_CONSTANT_SAM;
        auxCurrentInput_RH = fminf(10.0f, fmaxf(-10.0f, auxCurrentInput_RH)); // 14A 입력 제한
        // 전송 대기열에 추가
        addPDOtoStage_RH = true;
    }
}

/**
 * @brief  전송할 PDO 하나를 헤더와 데이터의 바이트 배열로 변환합니다.
 * @param  header   전송할 PDO의 헤더 (TaskID, ObjID)
 * @param  byte_arr 데이터를 채울 버퍼의 시작 주소
 * @return 기록된 총 바이트 수, 에러 시 음수
 */
static int _ExtLink_ConvertPDOToBytes(DOP_Header_t* header, uint8_t* byte_arr) {
    // 1. 전용 PDO 테이블에서 객체 정보 검색
    DOP_PDO_t* pdo_obj = _ExtLink_FindPDO(header->dictID, header->objID);
    if (pdo_obj == NULL || pdo_obj->addr == NULL) {
        // [ERROR LOG] Dictionary에 정의되지 않은 PDO를 보내려 함
        return -1;
    }

    int cursor = 0;

    // 2. 헤더(TaskID, ObjID)를 버퍼에 복사 (2 bytes)
    memcpy(&byte_arr[cursor], header, sizeof(DOP_Header_t));
    cursor += sizeof(DOP_Header_t);
    
    // 3. PDO 객체에 연결된 변수의 실제 데이터를 버퍼에 복사
    memcpy(&byte_arr[cursor], pdo_obj->addr, pdo_obj->objSize);
    cursor += pdo_obj->objSize;

    return cursor; // 총 기록된 바이트 수 (헤더 + 데이터) 반환
}

/**
 * @brief SDO 유닛을 확장보드 전송용 메시지 버퍼에 패킹합니다. (전용 버전)
 */
static int _ExtLink_AppendSDO(DOPI_SDOUnit_t* sdo_unit, DOPI_SDOMsg_t* sdo_msg) {
    uint8_t cursor = 0;
    if (sdo_msg->msgLength == 0) {
        cursor = 1; // 첫 SDO인 경우, 0번 인덱스는 SDO 개수를 위해 비워둠
    } else {
        cursor = sdo_msg->msgLength;
    }

    // 2. SDO 헤더 패킹
    sdo_msg->txBuf[cursor++] = sdo_unit->taskID;    // dict_id
    sdo_msg->txBuf[cursor++] = sdo_unit->SDOID;     // obj_id
    sdo_msg->txBuf[cursor++] = sdo_unit->param.SDOStatus;
    sdo_msg->txBuf[cursor++] = sdo_unit->param.numOfData;

    // 3. SDO 데이터 패킹
    // 데이터 크기를 전용 GetInfo 함수를 통해 가져옴
    SDOInfo_t data_size = _ExtLink_GetSDOInfo(sdo_unit->taskID, sdo_unit->SDOID);
    uint8_t total_size = sdo_unit->param.numOfData * data_size;

    memcpy(&sdo_msg->txBuf[cursor], sdo_unit->param.data, total_size);
    cursor += total_size;

    // 4. 최종 메시지 정보 업데이트
    sdo_msg->numOfSDO++;
    sdo_msg->txBuf[0] = sdo_msg->numOfSDO;
    sdo_msg->msgLength = cursor;

    return DOP_STATUS_SUCCESS;
}

/**
 * @brief 전용 SDO 테이블에서 데이터 타입 크기를 가져옵니다.
 */
static SDOInfo_t _ExtLink_GetSDOInfo(CmExtLinkDickID_t dict_id, uint8_t obj_id) {
    // DOP_ConvertDataSize는 범용 함수이므로 재사용 가능
    return DOP_ConvertDataSize(g_ext_sdo_table[dict_id][obj_id]);
}

/**
 * @brief  처리된 SDO의 결과를 확장팩으로 전송합니다.
 * @param  req_header 확장팩이 보냈던 원본 요청의 헤더
 * @param  res        콜백 함수가 채운 응답 데이터
 */
static void TxSDOResponseToExt(DOP_Header_t* req, DOP_SDOArgs_t* res)
{
    // 전송에 사용할 전역 SDO 버퍼를 초기화합니다.
    DOPI_ClearSDO(&sdo_msg_to_ext);

    // 응답 패킷 구성: [SDO개수=1] [TaskID] [SDOID] [상태] [데이터개수] [데이터...]
    uint8_t cursor = 0;
    
    // 1. SDO 개수 (응답은 항상 1개)
    sdo_msg_to_ext.txBuf[cursor++] = 1;

    // 2. 원본 요청의 Task ID와 SDO ID를 그대로 사용
    sdo_msg_to_ext.txBuf[cursor++] = req->dictID;
    sdo_msg_to_ext.txBuf[cursor++] = req->objID;

    // 3. 콜백 함수가 설정한 처리 결과 상태 (DOP_SDO_SUCC)
    sdo_msg_to_ext.txBuf[cursor++] = res->status;
    
    // 4. 응답 데이터 개수 및 데이터 (현재는 데이터 없이 상태만 보냄)
    sdo_msg_to_ext.txBuf[cursor++] = res->dataSize;
    if (res->dataSize > 0) {
        // 응답으로 보낼 데이터가 있다면 여기서 memcpy
    }

    // 최종 메시지 길이 설정
    sdo_msg_to_ext.msgLength = cursor;

    // CAN으로 전송
    TxSDOToExtPack(&g_ext_comm_obj, &sdo_msg_to_ext);
}

static void _GoToStoppedState(void) {
    // TODO : 확장팩과 연결이 끊어졌습니다. (음성 안내)
    bootup_sent = false;
    g_ext_link_mngr.is_ext_pack_booted = false;
    g_ext_link_mngr.is_ext_pack_connected = false;
    g_ext_link_mngr.last_ext_heartbeat_time_ms = 0;
    g_ext_link_mngr.cm_nmt_state = NMT_STATE_STOPPED;
    // _ExtLink_ClearPdoSendList(); // PDO 전송 리스트 초기화
    g_ext_comm_obj.data.motor_auxiliary_input_LH = 0;
    g_ext_comm_obj.data.motor_auxiliary_input_RH = 0;
}

/* ------------------- SDO CALLBACK ------------------- */
/**
 * @brief 확장팩이 요청한 PDO 리스트를 설정하는 콜백 함수
 */
static void Set_Send_PDO_List(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    // 1. 기존 리스트 초기화
    _ExtLink_ClearPdoSendList();

    // 2. SDO 데이터로부터 (TaskID, ObjID) 쌍을 읽어 리스트에 추가
    uint8_t* ids = (uint8_t*)req->data;
    // 전체 바이트 크기를 2로 나누어 처리할 ID 쌍의 개수를 계산
    int num_of_pairs = req->dataSize / 2; // (DictID(1B) + ObjID(1B) = 2B)

    bool success = true;
    for (int i = 0; i < num_of_pairs; ++i) {
        // i번째 쌍의 TaskID와 ObjID를 읽어옴
        uint8_t dictID = ids[i * 2];
        uint8_t objID  = ids[i * 2 + 1];
        if (_ExtLink_AddPdoToSendList(dictID, objID) != 0) {
            success = false; // 하나라도 추가에 실패하면 (리스트가 꽉 찼거나 등)
        }
    }

    res->dataSize = 0;
    res->status = (success) ? DOP_SDO_SUCC : DOP_SDO_FAIL;
    g_ext_link_mngr.last_ext_heartbeat_time_ms = IOIF_GetTick(); // 타이머 갱신
}

/**
 * @brief PDO 전송 리스트를 비웁니다. (정적 배열 버전)
 */
static void _ExtLink_ClearPdoSendList(void) {
    // cvector_free 대신, 크기 카운터만 0으로 리셋합니다.
    g_pdo_send_list_size = 0;
    // if (g_pdo_send_list != NULL) {
    //     cvector_free(g_pdo_send_list);
    //     g_pdo_send_list = NULL;
    // }
}

/**
 * @brief PDO를 전송 리스트에 추가합니다. (정적 배열 버전)
 * @return 성공 시 0, 실패(리스트 가득 참 등) 시 -1
 */
static int _ExtLink_AddPdoToSendList(uint8_t dict_id, uint8_t obj_id) 
{
    // 1. 유효성 검사: 해당 PDO가 전용 테이블에 존재하는지 확인
    DOP_PDO_t* pdo_obj = _ExtLink_FindPDO(dict_id, obj_id);
    if (pdo_obj == NULL) {
        // [ERROR LOG] SCM requested a non-existent PDO.
        return -1;
    }

    // 2. 리스트 공간 확인 (새로 추가된 안전장치)
    if (g_pdo_send_list_size >= EXT_LINK_PDO_LIST_MAX_SIZE) {
        // [ERROR LOG] PDO send list is full.
        return -1; // 배열이 가득 찼으면 추가하지 않음
    }

    DOP_Header_t new_pdo = {dict_id, obj_id};

    // 3. 중복 검사 (cvector_size 대신 g_pdo_send_list_size 사용)
    // for (int i = 0; i < cvector_size(g_pdo_send_list); ++i) {
    for (int i = 0; i < g_pdo_send_list_size; ++i) {
        if ((g_pdo_send_list[i].dictID == dict_id) && (g_pdo_send_list[i].objID == obj_id)) {
            return 0; // 이미 리스트에 있으면 추가하지 않음
        }
    }

    // 4. 리스트 배열에 추가하고 크기를 1 증가
    g_pdo_send_list[g_pdo_send_list_size] = new_pdo;
    g_pdo_send_list_size++;
    // cvector_push_back(g_pdo_send_list, new_pdo);

    return 0; // 성공
}

// 확장팩이 CM의 상태 변경을 명령하는 콜백
static void Get_Ext_Bootup(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    // OPERATIONAL 중에 Boot-up을 또 받는 것은 마스터가 리셋되었다는 의미
    if (g_ext_link_mngr.cm_nmt_state == NMT_STATE_OPERATIONAL || g_ext_link_mngr.cm_nmt_state == NMT_STATE_PRE_OPERATIONAL) {
        _GoToStoppedState();
    }

    // 핸드셰이크를 다시 시작
    bootup_sent = false;
	g_ext_link_mngr.is_ext_pack_booted = true;
    g_ext_link_mngr.last_ext_heartbeat_time_ms = IOIF_GetTick();
    g_ext_link_mngr.cm_nmt_state = NMT_STATE_PRE_OPERATIONAL;
    res->status = DOP_SDO_SUCC;
    res->dataSize = 0;
}

// 확장팩이 CM의 상태 변경을 명령하는 콜백
static void Set_CM_NMT_State(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    uint8_t new_state;
    memcpy(&new_state, req->data, 1);
    g_ext_link_mngr.last_ext_heartbeat_time_ms = IOIF_GetTick();
    g_ext_link_mngr.cm_nmt_state = (ExtLinkNmtState_t)new_state;
    if (new_state == NMT_STATE_OPERATIONAL) {
        // OPERATIONAL 진입 시, 아직 첫 Heartbeat을 못 받았으므로 플래그를 false로 초기화
        g_ext_link_mngr.is_ext_pack_connected = false;
    }
    res->status = DOP_SDO_SUCC;
    res->dataSize = 0;
}

// 확장팩이 CM의 상태 변경을 명령하는 콜백
static void Get_Ext_Heartbeat(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    g_ext_link_mngr.last_ext_heartbeat_time_ms = IOIF_GetTick();
    if (!g_ext_link_mngr.is_ext_pack_connected) {
        g_ext_link_mngr.is_ext_pack_connected = true; // 첫 HB 수신 시 연결됨으로 간주
    }
    res->status = DOP_SDO_SUCC;
    res->dataSize = 0;
}

// 확장팩이 CM의 상태 변경을 명령하는 콜백
static void Get_Ext_SyncState(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    // 상태 변수 데이터 전송
    SendSyncStates();
    res->status = DOP_SDO_SUCC;
    res->dataSize = 0;
}

// 신체 정보 SDO callback
static void _ExtLink_Set_User_Body_Data(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    // 1. 수신된 데이터 개수가 8개가 맞는지 확인 
    if (req->dataSize != 32) { // 8*4
        res->status = DOP_SDO_FAIL; // 데이터 개수가 맞지 않으면 실패 응답
        res->dataSize = 0;
        return;
    }

    // 2. 수신된 바이트 스트림(req->data)을 uint32_t 배열로 변환
    uint32_t received_data[8];
    memcpy(received_data, req->data, sizeof(uint32_t) * 8);

    // 3. 파싱된 데이터를 통신 객체의 데이터 저장 공간에 저장
    g_ext_comm_obj.data.user_weight             = received_data[0];
    g_ext_comm_obj.data.user_height             = received_data[1];
    g_ext_comm_obj.data.user_right_thigh_length = received_data[2];
    g_ext_comm_obj.data.user_left_thigh_length  = received_data[3];
    g_ext_comm_obj.data.user_right_shin_length  = received_data[4];
    g_ext_comm_obj.data.user_left_shin_length   = received_data[5];
    g_ext_comm_obj.data.user_right_ankle_length = received_data[6];
    g_ext_comm_obj.data.user_left_ankle_length  = received_data[7];

    // 4. 저장된 데이터를 기반으로 신체 분절 파라미터 계산 함수 호출
    _Calculate_Segment_Parameters();

    // 5. Master에게 성공적으로 처리했음을 응답
    isSetUserBodyData = true;
    res->status = DOP_SDO_SUCC;
    res->dataSize = 0;
}

/**
 * @brief 수신된 신체 정보를 바탕으로 실제 사용할 파라미터를 계산합니다.
 */
static void _Calculate_Segment_Parameters(void) {
    // float 변환 및 단위 변환 (g->kg, mm->m)
	_extlink_user_weight = _ExtLink_checkValue((float)g_ext_comm_obj.data.user_weight / 1000, 20.0f, 230.0f, DEFAULT_WEIGHT); 							    // g -> kg
	_extlink_user_height = _ExtLink_checkValue((float)g_ext_comm_obj.data.user_height / 1000, 1.0f, 2.5f, DEFAULT_HEIGHT);    							    // mm -> m
	_extlink_user_right_thigh_length = _ExtLink_checkValue((float)g_ext_comm_obj.data.user_right_thigh_length / 1000, 0.1f, 1.0f, DEFAULT_THIGH_LENGTH);	// mm -> m
	_extlink_user_left_thigh_length = _ExtLink_checkValue((float)g_ext_comm_obj.data.user_left_thigh_length / 1000, 0.1f, 1.0f, DEFAULT_THIGH_LENGTH);	    // mm -> m
	_extlink_user_right_shin_length = _ExtLink_checkValue((float)g_ext_comm_obj.data.user_right_shin_length / 1000, 0.1f, 1.0f, DEFAULT_SHIN_LENGTH);       // mm -> m
	_extlink_user_left_shin_length = _ExtLink_checkValue((float)g_ext_comm_obj.data.user_left_shin_length / 1000, 0.1f, 1.0f, DEFAULT_SHIN_LENGTH);		    // mm -> m
	_extlink_user_right_ankle_length = _ExtLink_checkValue((float)g_ext_comm_obj.data.user_right_ankle_length / 1000, 0.01f, 0.5f, DEFAULT_ANKLE_LENGTH);   // mm -> m
	_extlink_user_left_ankle_length = _ExtLink_checkValue((float)g_ext_comm_obj.data.user_left_ankle_length / 1000, 0.01f, 0.5f, DEFAULT_ANKLE_LENGTH);	    // mm -> m

    // 분절 길이 및 질량 계산
	TrunkLength_Improved = _extlink_user_height - (_extlink_user_right_thigh_length + _extlink_user_left_thigh_length) / 2.0f
			- (_extlink_user_right_shin_length + _extlink_user_left_shin_length) / 2.0f - (_extlink_user_right_ankle_length + _extlink_user_left_ankle_length) / 2.0f;
	ThighLength_Improved = (_extlink_user_right_thigh_length + _extlink_user_left_thigh_length) / 2.0f;
	AnkleLength_Improved = (_extlink_user_right_ankle_length + _extlink_user_left_ankle_length) / 2.0f;
	ShankLength_Improved = (_extlink_user_right_shin_length + _extlink_user_left_shin_length) / 2.0f + AnkleLength_Improved; 	// 종아리 길이에 발목 길이 추가하기로 수정

	TrunkMass_Improved = _extlink_user_weight * TrunkLength_Improved / _extlink_user_height;    // 골반 무게 포함 (GaitAnalysis.c에서 사용하지 않으므로 큰 상관 없음)
	ThighMass_Improved = _extlink_user_weight * ThighLength_Improved / _extlink_user_height / 2.0f;
	ShankMass_Improved = _extlink_user_weight * ShankLength_Improved / _extlink_user_height / 2.0f;
	AnkleMass_Improved = _extlink_user_weight * AnkleLength_Improved / _extlink_user_height / 2.0f;
}

/**
 * @brief 값의 유효 범위를 확인하고 벗어날 경우 기본값을 반환합니다.
 */
static float _ExtLink_checkValue(float value, float min, float max, float defaultValue) 
{
    if (value < min || value > max) {
        return defaultValue;
    }
    return value;
}


/******************** P Vector Setting ********************/
// P vector callback - Send to Motor(RH)
static void Set_P_Vector_Yd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.p_vector_RH.yd, req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_L_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.p_vector_RH.L, req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_S0_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.p_vector_RH.s0, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Sd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.p_vector_RH.sd, req->data, 1);

	Send_P_Vector(RH_SAGITAL, g_ext_comm_obj.data.p_vector_RH.yd,
        g_ext_comm_obj.data.p_vector_RH.L, g_ext_comm_obj.data.p_vector_RH.s0, g_ext_comm_obj.data.p_vector_RH.sd);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Reset_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	Send_P_Vector_Reset(RH_SAGITAL);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

// P vector callback - Send to Motor(LH)
static void Set_P_Vector_Yd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.p_vector_LH.yd , req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_L_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.p_vector_LH.L , req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_S0_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.p_vector_LH.s0 , req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Sd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.p_vector_LH.sd , req->data, 1);

    Send_P_Vector(LH_SAGITAL, g_ext_comm_obj.data.p_vector_LH.yd,
        g_ext_comm_obj.data.p_vector_LH.L, g_ext_comm_obj.data.p_vector_LH.s0, g_ext_comm_obj.data.p_vector_LH.sd);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Reset_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	Send_P_Vector_Reset(LH_SAGITAL);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

/******************** I Vector Setting ********************/
// I vector callback - Send to Motor(RH)
static void Set_I_Vector_Epsilon_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
	memcpy(&g_ext_comm_obj.data.i_vector_RH.epsilon_target, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kp_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
	memcpy(&g_ext_comm_obj.data.i_vector_RH.Kp_target, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
	memcpy(&g_ext_comm_obj.data.i_vector_RH.Kd_target, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Lambda_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
	memcpy(&g_ext_comm_obj.data.i_vector_RH.lambda_target, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Duration_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
	memcpy(&g_ext_comm_obj.data.i_vector_RH.duration, req->data, 2);

    Send_I_Vector(RH_SAGITAL, g_ext_comm_obj.data.i_vector_RH.epsilon_target,
        g_ext_comm_obj.data.i_vector_RH.Kp_target, g_ext_comm_obj.data.i_vector_RH.Kd_target,
        g_ext_comm_obj.data.i_vector_RH.lambda_target, g_ext_comm_obj.data.i_vector_RH.duration);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kp_Max_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&g_ext_comm_obj.data.kpMaxRH, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kd_Max_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&g_ext_comm_obj.data.kdMaxRH, req->data, 4);

    Set_I_Vector_KpKd_Max(RH_SAGITAL, g_ext_comm_obj.data.kpMaxRH, g_ext_comm_obj.data.kdMaxRH);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}


// I vector callback - Send to Motor(LH)
static void Set_I_Vector_Epsilon_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
	memcpy(&g_ext_comm_obj.data.i_vector_LH.epsilon_target, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kp_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
    memcpy(&g_ext_comm_obj.data.i_vector_LH.Kp_target, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
    memcpy(&g_ext_comm_obj.data.i_vector_LH.Kd_target, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Lambda_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
    memcpy(&g_ext_comm_obj.data.i_vector_LH.lambda_target, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Duration_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) {
    memcpy(&g_ext_comm_obj.data.i_vector_LH.duration, req->data, 2);

    Send_I_Vector(LH_SAGITAL, g_ext_comm_obj.data.i_vector_LH.epsilon_target,
        g_ext_comm_obj.data.i_vector_LH.Kp_target, g_ext_comm_obj.data.i_vector_LH.Kd_target,
        g_ext_comm_obj.data.i_vector_LH.lambda_target, g_ext_comm_obj.data.i_vector_LH.duration);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kp_Max_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&g_ext_comm_obj.data.kpMaxLH, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kd_Max_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&g_ext_comm_obj.data.kdMaxLH, req->data, 4);

    Set_I_Vector_KpKd_Max(LH_SAGITAL, g_ext_comm_obj.data.kpMaxLH, g_ext_comm_obj.data.kdMaxLH);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

/******************** F Vector Setting ********************/
// F vector callback - Send to Motor(RH)
static void Set_F_Vector_ModeIDX_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.f_vector_RH.mode_idx, req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_F_Vector_TauMax_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.f_vector_RH.tau_max, req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_F_Vector_Delay_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.f_vector_RH.delay, req->data, 2);

    // TODO : Send to RH F-vector
    Send_F_Vector(RH_SAGITAL, g_ext_comm_obj.data.f_vector_RH.mode_idx,
			g_ext_comm_obj.data.f_vector_RH.tau_max, g_ext_comm_obj.data.f_vector_RH.delay);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

// F vector callback - Send to Motor(LH)
static void Set_F_Vector_ModeIDX_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
	memcpy(&g_ext_comm_obj.data.f_vector_LH.mode_idx , req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_F_Vector_TauMax_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    memcpy(&g_ext_comm_obj.data.f_vector_LH.tau_max , req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_F_Vector_Delay_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    memcpy(&g_ext_comm_obj.data.f_vector_LH.delay , req->data, 2);

    // TODO : Send to LH F-vector
    Send_F_Vector(LH_SAGITAL, g_ext_comm_obj.data.f_vector_LH.mode_idx,
			g_ext_comm_obj.data.f_vector_LH.tau_max, g_ext_comm_obj.data.f_vector_LH.delay);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

// Degree Limit
static void Set_Degree_Limit_Upper_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.degree_limit_upper_RH, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Degree_Limit_Lower_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.degree_limit_lower_RH, req->data, 4);

    // TODO : Send to RH
    Send_Degree_Limit(RH_SAGITAL, g_ext_comm_obj.data.degree_limit_upper_RH, 
                        g_ext_comm_obj.data.degree_limit_lower_RH);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Degree_Limit_Upper_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.degree_limit_upper_LH, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Degree_Limit_Lower_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.degree_limit_lower_LH, req->data, 4);

     // TODO : Send to LH
    Send_Degree_Limit(LH_SAGITAL, g_ext_comm_obj.data.degree_limit_upper_LH, 
                        g_ext_comm_obj.data.degree_limit_lower_LH);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Degree_Limit_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    SetDevDegreeLimitRoutine(RH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Clear_Degree_Limit_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    ClearDevDegreeLimitRoutine(RH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Degree_Limit_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    SetDevDegreeLimitRoutine(LH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Clear_Degree_Limit_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    ClearDevDegreeLimitRoutine(LH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

// Velocity Limit
static void Set_Velocity_Limit_Upper_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.velocity_limit_upper_RH, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Limit_Lower_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.velocity_limit_lower_RH, req->data, 4);

    // TODO : Send to RH
    Send_Velocity_Limit(RH_SAGITAL, g_ext_comm_obj.data.velocity_limit_upper_RH, 
                        g_ext_comm_obj.data.velocity_limit_lower_RH);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Limit_Upper_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.velocity_limit_upper_LH, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Limit_Lower_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.velocity_limit_lower_LH, req->data, 4);

     // TODO : Send to LH
    Send_Velocity_Limit(LH_SAGITAL, g_ext_comm_obj.data.velocity_limit_upper_LH, 
                        g_ext_comm_obj.data.velocity_limit_lower_LH);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Limit_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    SetDevVelocityLimitRoutine(RH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Clear_Velocity_Limit_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    ClearDevVelocityLimitRoutine(RH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Limit_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    SetDevVelocityLimitRoutine(LH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Clear_Velocity_Limit_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    ClearDevVelocityLimitRoutine(LH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

// DOB
static void Set_DOB_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    SetDevDOBRoutine(RH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Clear_DOB_Routine_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    ClearDevDOBRoutine(RH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_DOB_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    SetDevDOBRoutine(LH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Clear_DOB_Routine_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    ClearDevDOBRoutine(LH_SAGITAL);
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

// Gravity & Velocity Compensation Gain
static void Set_Normal_Comp_Gain_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    Send_Comp_parameter(RH_SAGITAL, 0, 0.015, 0.02, 1);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Normal_Comp_Gain_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    Send_Comp_parameter(LH_SAGITAL, 0, 0.015, 0.02, 1);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Resistive_Comp_Gain_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    memcpy(&g_ext_comm_obj.data.velCompGain_RH, req->data, 4);

    Send_Comp_parameter(RH_SAGITAL, 0, g_ext_comm_obj.data.velCompGain_RH, 2, 2);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Resistive_Comp_Gain_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res) 
{
    memcpy(&g_ext_comm_obj.data.velCompGain_LH , req->data, 4);

    Send_Comp_parameter(LH_SAGITAL, 0, g_ext_comm_obj.data.velCompGain_LH, 2, 2);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

// 기존 H10 Assist Mode 사용 여부
static void Set_H10_Existing_Assist_Mode(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&g_ext_comm_obj.data.H10_Existing_Assist_Mode_Flag, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

/* Data Object Table에서 Data Type을 Assemble */
static void _ExtLink_CreateSDOTable(void) {
	//*******************************************************************************************************************//
	//                              |            Task_ID        |			        SDO_ID			        |       DATA_TYPE | //
	//*******************************************************************************************************************//
	/* 확장팩 -> CM */
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_PDO_LIST],   DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SDO_LIST],   DOP_UINT8);

	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_EXT_NMT_BOOTUP],     DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_CM_NMT_STATE],       DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_EXT_NMT_HEARTBEAT],  DOP_UINT8);
    // 신체 정보
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_USER_BODY_DATA],     DOP_UINT32);
   	// P-vector
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_YD_RH], DOP_INT16);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_L_RH],  DOP_UINT16);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_S0_RH], DOP_UINT8);
   	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_SD_RH], DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_RESET_RH], DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_YD_LH], DOP_INT16);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_L_LH],  DOP_UINT16);
   	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_S0_LH], DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_SD_LH], DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_P_VECTOR_RESET_LH], DOP_UINT8);
    // I-vector
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_EPSILON_RH],    DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KP_RH],         DOP_UINT8);
   	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KD_RH],         DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_LAMBDA_RH],     DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_DURATION_RH],   DOP_UINT16);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KP_MAX_RH],     DOP_FLOAT32);
   	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KD_MAX_RH],     DOP_FLOAT32);

	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_EPSILON_LH],    DOP_UINT8);
   	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KP_LH],         DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KD_LH],         DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_LAMBDA_LH],     DOP_UINT8);
   	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_DURATION_LH],   DOP_UINT16);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KP_MAX_LH],     DOP_FLOAT32);
   	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_I_VECTOR_KD_MAX_LH],     DOP_FLOAT32);
    // F-vector
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_MODE_IDX_RH],  DOP_UINT16);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_TMAX_RH],      DOP_INT16);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_DELAY_RH],     DOP_UINT16);
   	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_MODE_IDX_LH],  DOP_UINT16);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_TMAX_LH],      DOP_INT16);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_F_VECTOR_DELAY_LH],     DOP_UINT16);
    // Degree Limit ROM
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SET_DEGREE_LIMIT_ROUTINE_RH],      DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_DEGREE_LIMIT_ROUTINE_RH],    DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SET_DEGREE_LIMIT_ROUTINE_LH],      DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_DEGREE_LIMIT_ROUTINE_LH],    DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_DEGREE_LIMIT_UPPER_RH],     DOP_FLOAT32);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_DEGREE_LIMIT_LOWER_RH],     DOP_FLOAT32);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_DEGREE_LIMIT_UPPER_LH],     DOP_FLOAT32);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_DEGREE_LIMIT_LOWER_LH],     DOP_FLOAT32);
    // Velocity Limit ROM
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SET_VELOCITY_LIMIT_ROUTINE_RH],      DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_VELOCITY_LIMIT_ROUTINE_RH],    DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SET_VELOCITY_LIMIT_ROUTINE_LH],      DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_VELOCITY_LIMIT_ROUTINE_LH],    DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_VELOCITY_LIMIT_UPPER_RH],     DOP_FLOAT32);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_VELOCITY_LIMIT_LOWER_RH],     DOP_FLOAT32);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_VELOCITY_LIMIT_UPPER_LH],     DOP_FLOAT32);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_VELOCITY_LIMIT_LOWER_LH],     DOP_FLOAT32);
    // DOB
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SET_DOB_ROUTINE_RH],   DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_DOB_ROUTINE_RH], DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SET_DOB_ROUTINE_LH],   DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_CLEAR_DOB_ROUTINE_LH], DOP_UINT8);
    // Compensator
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SEND_NORMAL_COMP_GAIN_RH],       DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SEND_NORMAL_COMP_GAIN_LH],       DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SEND_RESISITIVE_COMP_GAIN_RH],   DOP_FLOAT32);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_SEND_RESISITIVE_COMP_GAIN_LH],   DOP_FLOAT32);
    // 기존 H10 Assist Mode 사용 여부
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_SDO_ID_EXT_to_CM_H10_ORIGINAL_ASSIST_MODE],   DOP_UINT8);
    /* CM -> 확장팩 */
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_SDO_ID_CM_to_EXT_CM_NMT_BOOTUP],      DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_SDO_ID_CM_to_EXT_CM_NMT_STATE],       DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_SDO_ID_CM_to_EXT_CM_NMT_HEARTBEAT],   DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_SDO_ID_CM_to_EXT_SYNC_STATES],        DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_SDO_ID_CM_to_EXT_SUIT_MODE],          DOP_UINT8);
	_ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_SDO_ID_CM_to_EXT_ASSIST_LEVEL],       DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_SDO_ID_CM_to_EXT_P_VECTOR_DURATION_COMPLETED_RH],    DOP_UINT8);
    _ExtLink_AssembleSDO( &g_ext_sdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_SDO_ID_CM_to_EXT_P_VECTOR_DURATION_COMPLETED_LH],    DOP_UINT8);
}
static void _ExtLink_CreatePDOTable(void) {
	//*************************************************************************************************************************************************//
	//							                |           Task_ID         |                       PDO_ID                  |  DATA_TYPE   |   #_of_DATA //
	//*************************************************************************************************************************************************//
    /* 확장팩 -> CM */
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_PDO_ID_EXT_to_CM_RIGHT_AUX_INPUT], DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_EXT_to_CM] [CMEXT_LINK_PDO_ID_EXT_to_CM_LEFT_AUX_INPUT],  DOP_INT16,  1);

    /* CM -> 확장팩 */
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_ASSIST_MODE_LOOP_CNT],    DOP_UINT32,  1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_ANGLE],          DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_ANGLE],         DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_THIGH_ANGLE],        DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_THIGH_ANGLE],       DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_PELVIC_ANGLE],            DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_PELVIC_VEL_Y],            DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_KNEE_ANGLE],         DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_KNEE_ANGLE],        DOP_INT16,  1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_MOTOR_ANGLE],    DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_MOTOR_ANGLE],   DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_FOOT_CONTACT],       DOP_UINT8,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_FOOT_CONTACT],      DOP_UINT8,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_GAIT_STATE],              DOP_UINT8,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_GAIT_CYCLE],              DOP_UINT8,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_FORWARD_VELOCITY],        DOP_INT16,  1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_TORQUE],         DOP_INT16,  1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_TORQUE],        DOP_INT16,  1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_FRONTAL_ROLL],   DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_FORNTAL_ROLL],  DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_SAGITTAL_PITCH], DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_SAGITTAL_PITCH],DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_ACC_X],   DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_ACC_Y],   DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_ACC_Z],   DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_GYR_X],   DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_GYR_Y],   DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_LEFT_HIP_IMU_GLOBAL_GYR_Z],   DOP_INT16, 1);
	_ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_ACC_X],  DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_ACC_Y],  DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_ACC_Z],  DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_GYR_X],  DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_GYR_Y],  DOP_INT16, 1);
    _ExtLink_AssemblePDO( (PDOInfo_t*)g_ext_pdo_table [CMEXT_LINK_DICT_ID_CM_to_EXT] [CMEXT_LINK_PDO_ID_CM_to_EXT_RIGHT_HIP_IMU_GLOBAL_GYR_Z],  DOP_INT16, 1);
}

static void _ExtLink_AssembleSDO(SDOInfo_t* t_addr, uint8_t t_dataType) {
	SDOInfo_t t_temp = t_dataType;
	memcpy(t_addr, &t_temp, sizeof(SDOInfo_t));
}

static void _ExtLink_AssemblePDO(PDOInfo_t* t_addr, uint8_t t_dataType, uint8_t t_numOfData) {
	PDOInfo_t t_temp = {t_dataType, t_numOfData};
	memcpy(t_addr, &t_temp, sizeof(PDOInfo_t));
}
