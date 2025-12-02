

#include "AS_ble_comm_hdlr.h"
#include "ioif_mcp79510.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */


#define DEFAULT_WEIGHT 70.0f      // kg
#define DEFAULT_HEIGHT 1.75f      // m
#define DEFAULT_THIGH_LENGTH 0.45f // m
#define DEFAULT_SHIN_LENGTH 0.40f  // m
#define DEFAULT_ANKLE_LENGTH 0.10f // m


/**
 *------------------------------------------------------------
 *                           VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */


TaskObj_t BLECommHdlr;

AS_BLECommState_t isBLEConnect;
AssistStage assistStage_app = 0;

DOPI_SDOMsg_t BLE_sdo_msg;
DOPI_SDOMsg_t prev_BLE_sdo_msg;
DOPI_SDOMsg_t BLE_err_sdo_msg;
AS_BLE_CtrlObj_t BLECtrlObj = {0,};
AS_BLE_TxFncPtr BLE_txfnc = NULL;
rtcTime_t RTC_Time ={0, };
uint8_t BLE_Node_ID = 0;
uint8_t test_packet[244];
uint16_t checksum;
uint8_t check_result;
uint8_t BTN_res[3] = {0,};
uint8_t assistmode;
uint8_t standbymode = 1;
uint32_t trainingsessionid;
uint8_t IsBTReady = 0;
uint8_t aes_res = 0;
uint8_t one_time_sw =0;
uint16_t ble_cnt = 0;

uint8_t SPPB_mode = 0;
uint8_t SPPB_r_sw = 0;
uint8_t SPPB_l_sw = 0;

uint8_t error_sw = 0;
uint8_t errorMSGcode = 0;
uint8_t check_error = 0;
extern uint16_t CM_sw_ver;
uint16_t RH_MD_sw_ver = 100;
uint16_t LH_MD_sw_ver = 100;

uint8_t motion_test_mode = 0;
uint8_t ble_set_mode = 0;

uint8_t Success_id_cnt = 0;

bool IsManuChangeSet = false;
bool IsManuBLEreconnectionReady = false;
bool Is_ManuReady = false;
bool Is_ManuSDCardState = true;

extern SUIT_AudioState_t SuitAudioState;
extern osSemaphoreId_t BinSem_PlaySoundHandle;
extern audioType_t audioOutputMode;
extern uint8_t audioLanguage;


SUITBLEFlashFlag FLASHSaveFlag = IDLE_FLASH_BLE_DATA;
static uint8_t SoundSettingSavedFlag = 0;
static uint8_t FvectorSavedFlag = 0;
uint8_t set_uuid_flag = 0;
uint8_t MobileUuidFront[10] = {0,};
uint8_t MobileUuidBack[10] = {0,};
uint8_t Org_Code[10] = {0,};
uint8_t uuid_reset_result = 0;

static cvector_vector_type(DOP_Header_t) BT_pdo_send_list;

/**
 *------------------------------------------------------------
 *                           VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */
extern SDOInfo_t BT_SDOTable[BT_TASK_NUM][DOP_SDO_MAX_NUM];
extern PDOInfo_t BT_PDOTable[BT_TASK_NUM][DOP_PDO_MAX_NUM];

DOP_Dict_t BT_DODs[BT_TASK_NUM] = {0};

uint32_t BLECommHdlrloopcnt = 0;

uint8_t powerOffCmdByBLE = SUIT_PWR_ON;

uint8_t assistPriority_app = 0;
uint8_t SPI_test_sw = 0;
uint8_t SPI_test_result = 0;
uint8_t ble_check_status = 0;

uint8_t manu_mode_enable = 0;
extern bool audio_end;

uint32_t BLE_startIndex = 0;
int receivedDataSize = 0;


float RH_angle_temp = 0;
float LH_angle_temp = 0;

float SitToStance_Torque = 0;
float StanceToSit_Torque = 0;

uint32_t BLE_test4;

int inter_cnt = 0;

extern uint8_t can_test[16];

extern IOIF_FILE_t	fileObj;
uint32_t ble_fatfs_write_test_byte = 0;
uint32_t ble_fatfs_read_test_byte = 0;
uint8_t aging_enable = 0;

uint8_t prevAssistLvfromApp;
uint8_t prevStandbyModefromApp = 1;

static uint8_t oneTimeSendNeutralPosture = 0;
static bool updateNeutralPostureFlag = false;

static bool oneTimeStateModeFlag = true;
static bool oneTimeAssistModeFlag = true;

/* Initialize variables if not already done */
static float previousDistance = 0.0f;
static float totalCalories = 0.0f;

int16_t RightHipExtensionTorque;
int16_t LeftHipExtensionTorque;
int16_t RightHipFlexionTorque;
int16_t LeftHipFlexionTorque;
int16_t RightKneeExtensionTorque;
int16_t LeftKneeExtensionTorque;
int16_t RightKneeFlexionTorque;
int16_t LeftKneeFlexionTorque;

uint8_t DisableStairWalkingDetection;

uint8_t manu_mode = 0;
uint8_t prev_manu_mode = 0;
uint8_t manu_step = 0;

bool RH_NPOffsetCompleted = false;
bool LH_NPOffsetCompleted = false;
bool RK_NPOffsetCompleted = false;
bool LK_NPOffsetCompleted = false;


float User_Weight;
float User_Height;
float User_Right_Thigh_Length;
float User_Left_Thigh_Length;
float User_Right_Shin_Length;
float User_Left_Shin_Length;
float User_Right_Ankle_Length;
float User_Left_Ankle_Length;

uint16_t GaitCycle[101];
int16_t ThighAngleLeftYMean[101];
int16_t ThighAngleLeftYStd[101];
int16_t ThighAngleRightYMean[101];
int16_t ThighAngleRightYStd[101];
int16_t ThighTorqueLeftYMean[101];
int16_t ThighTorqueLeftYStd[101];
int16_t ThighTorqueRightYMean[101];
int16_t ThighTorqueRightYStd[101];

bool isMotionAnalysisCancel = false;
bool onetime_send_finish_able = false;
bool onetime_finish_set = false;
bool isMotionAnalysisRunning = false;
bool disablePDOSending = true;
uint8_t MotionAnalysisDataState = 255;
uint8_t ReportDataTransferState = 255;
uint8_t motion_analysis_cnt;
uint8_t report_data_cnt;
uint8_t report_data_sw;

uint32_t ReportId;
uint32_t TrackUsage = 2; // 초기값 세팅
uint32_t MotionAnalysisType;
uint8_t DataReceiveState;


extern uint16_t countFor1sec; 			   // 1초 카운트를 위한 변수
extern uint16_t countFor1sec_standbyState; // 대기모드 1초 카운트를 위한 변수

bool IsBLEconnectedtoApp = false;
bool IsOnceBLEconnectedtoApp = false;

AS_TrainingInfo_t trainingInfo[SESSION_COUNT];
uint8_t current_session = 0;

static bool Set_MANU_RH_Offset_calibration_done = false;
static bool Set_MANU_LH_Offset_calibration_done = false;

static uint8_t SittoStandAssistLevel = 0;
static uint8_t StandtoSitAssistLevel = 0;

/* AES encrytions */
int error = 0;
int random_number = 0;
uint32_t bleconnection_trial = 0;
bool bleisconnect =0;
uint8_t bt_ready_sendcount = 0;
uint8_t test_sw = 0;
uint8_t aes_encrypt_sw = 1;
uint8_t aes_send_sw = 0;
uint8_t key[32] = {0x4A, 0x5A, 0x4F, 0x45, 0x66, 0x38, 0x6B, 0x31,
	    0x4A, 0x47, 0x63, 0x51, 0x61, 0x34, 0x78, 0x2F,
	    0x57, 0x43, 0x36, 0x66, 0x46, 0x4D, 0x57, 0x57,
	    0x6D, 0x54, 0x4E, 0x7A, 0x57, 0x6A, 0x52, 0x35
	};
uint8_t in[16]  = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A };
struct AES_ctx ctx;
char base64_res[25];
size_t base64_len;

// Base64 인코딩용 테이블
uint8_t plain_text[16] = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A };
static const char base64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
uint32_t m_val;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Ent(void);
static void StateOff_Run(void);

static void StateStandby_Ent(void);
static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ----------------------- FUNCTION ----------------------- */
static bool BLEInit(uint32_t baudrate);
static void CheckFLASH_ReadWrite(void);
static void SPPB_TEST(void);
static void UpdateVariable(void);
static void CheckFlag(void);
static void TxBLESDO(void);
static void CheckNPOffset(AS_BLE_CtrlObj_t* obj);
static void re_TxBLESDO(void);
static void CheckErrorMSG();
static int BT_Run_Send_PDO();
static int BT_Pack_PDO(uint8_t* t_byte_arr, uint8_t* t_byte_len);
int BT_UnpackSDO(DOPI_DevObj_t* t_obj, uint8_t* t_byteArr);
static int BT_ReadSDO(DOPI_DevObj_t* t_obj, uint8_t* t_byte_arr);
uint16_t BT_CallSDO(DOP_SDO_t* t_SDO, DOP_SDOArgs_t* t_req);
void BT_SetSDOReq(uint8_t t_dictID, uint16_t t_objID, void* t_data, uint8_t t_size);
uint16_t BT_SetSDOArgs(DOP_SDO_t* t_SDO, DOP_SDOArgs_t* t_args);
static int BT_Convert_PDO_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr);
DOP_SDO_t* BT_FindSDO(uint8_t t_dictID, uint16_t t_objID);
DOP_PDO_t* BT_FindPDO(uint8_t t_dictID, uint16_t t_objID);
uint16_t BT_SetPDO(DOP_PDO_t* t_PDO, void* t_data);
static int BT_AS_CB(AS_BLE_CtrlObj_t* obj);
static void BT_CreateDOD(uint8_t t_dictID);
static void BT_CreateSDO(uint8_t t_dictID, uint16_t t_objID, DOP_DataType_t t_type, DOP_SDOCB_t t_callback);
static void BT_CreatePDO(uint8_t t_dictID, uint16_t t_objID, DOP_DataType_t t_type, uint8_t t_size, void* t_addr);
static void BT_Add_PDO_to_Send(uint8_t t_dictID, uint8_t t_objID);
static void BT_Clear_PDO_to_Send();
static void BT_Set_Send_PDO_List(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void BT_ADD_TASK_ID( DOPI_SDOMsg_t* t_SDOMsg, uint8_t fnc_code);
static void BT_AppendSDO(DOPI_SDOUnit_t* t_SDOUnit, DOPI_SDOMsg_t* t_SDOMsg);
static int BT_CheckSDO(uint8_t t_taskID, uint16_t t_SDOID);
static SDOInfo_t BT_GetSDOInfo(uint8_t t_taskID, uint16_t t_SDOID);
static int Check_BLE_COMMU_TX(void);
static int Check_BLE_COMMU_RX(uint8_t* t_byte_arr);
static void SetupBLEDOD(AS_BLE_CtrlObj_t* obj);
static void BLEisConnectedNotifytoApp(void);
static void SendHwSwVersion(void);
static void SendMotionAnalysisFinishAble(void);
static void CheckStandbyStatus(void);
static void AMS_Stepwise_Run(DOPI_SDOUnit_t sdoUnit);
static void SendMotionAnalysisDataState(void);
static void SendMotionAnalysisReportData(void);

/* --------------------- SDO CALLBACK --------------------- */
static void Set_Serial_1(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Serial_2(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Serial_3(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_HW_ver(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_SW_ver_major(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_SW_ver_minor(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_SW_ver_patch(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_SW_ver_debug(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Receive_decrypted_plain_txt(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Mobile_UUID(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_test(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Function(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Receive_Success_MTC(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_User_Data(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_User_Body_Data(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Sound_Setting(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Neutral_Sagittal_Angle(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Left_Hip_Neutral_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Right_Hip_Neutral_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Left_Knee_Neutral_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Right_Knee_Neutral_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Neutral_Posture_Calibration(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Mode_Type(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Smart_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Aqua_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Space_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Limit_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Sit_Stand_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Training_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Training_Mode_Change(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Training_Finish(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Motion_Analysis_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Motion_Analysis_Cancel(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Motion_Analysis_Finish(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Motion_Analysis_Data_Receive_State(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Standby_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Assist_Level(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_STS_Hold(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_STS_Ready(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Sit_to_Stance_Torque(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Stance_to_Sit_Torque(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Check_Error_Msg_State(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void SPPB_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void SPPB_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void SPPB_Assist_OnOff(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Timer_Set(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Timer_Reset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

/* Angel'a TS Callback */
static void Set_MANU_BOOT_CHECK(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Send_MANU_Success(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_name_set(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_ESP32_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_MES_Setting_uint8(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_MES_Setting_string10(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_MES_WIFI_Test(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_SD_Card_check(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_FD_CAN_check(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_HW_Version_check(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_SW_Version_check(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_System_Monitoring(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_ImuCalib_RH_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_ImuCalib_RH_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_ImuCalib_LH_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_ImuCalib_LH_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_EncoderCheck_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_EncoderCheck_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_Encoder_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_RH_Range_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_RH_Value_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_LH_Range_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_LH_Value_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_CurrentSensor_Check(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_Monitoring_Enable(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_ButtonTest_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_RH_PositiveMax_Position(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_RH_NegativeMax_Position(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_LH_PositiveMax_Position(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_LH_NegativeMax_Position(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_DrivingTest_End(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_Led_Test(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_SoundTest_Voice(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_SoundTest_Beep(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_Magnetic_Sensor_Test(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_AgingTest_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_AgingTest_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_Org_code(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_Mobile_UUID_Reset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MANU_RTC(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Start_toruqe_measure(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Stop_torque_measure(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Send_ORG_CODE(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

/* ----------------------- ROUTINE ------------------------ */


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* --------------------- SDO CALLBACK --------------------- */
DOP_COMMON_SDO_CB(BLECommHdlr)

/* ------------------------- MAIN ------------------------- */
void InitBLEComm()
{
	/* Init Task */
	InitTask(&BLECommHdlr);
	DOPC_AssignTaskID(&BLECommHdlr, TASK_IDX_BLE_COMM_HDLR);

	/* Init Device */
	if (BLEInit(115200) == false)
	{
		//Error Handle
	}

	IOIF_ESP32_GPIOCB(); //ESP 동작 시나리오 상 StartupAPP 함수에 선언되면 오동작함

	/* State Definition */
	TASK_CREATE_STATE(&BLECommHdlr, TASK_STATE_OFF,     StateOff_Ent,     StateOff_Run,     NULL, 			 false);
	TASK_CREATE_STATE(&BLECommHdlr, TASK_STATE_STANDBY, StateStandby_Ent, StateStandby_Run,	NULL, 			 true);
	TASK_CREATE_STATE(&BLECommHdlr, TASK_STATE_ENABLE,  StateEnable_Ent,  StateEnable_Run, 	StateEnable_Ext, false);
	TASK_CREATE_STATE(&BLECommHdlr, TASK_STATE_ERROR,   NULL,             StateError_Run,  	NULL, 			 false);

	/* Routine Definition */

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(BT_TASK_ID_ANGELA_CM_to_Mobile); // for BLE Data Communication
	DOP_CreateDOD(BT_TASK_ID_ESP32_CMD);

	// SDO
	// DOP_COMMON_SDO_CREATE(TASK_IDX_BLE_COMM_HDLR)
	DOP_CreateSDO(TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SERIAL_1,         DOP_STRING10,  	Set_Serial_1);
	DOP_CreateSDO(TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SERIAL_2,         DOP_STRING10,  	Set_Serial_2);
	DOP_CreateSDO(TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SERIAL_3,         DOP_STRING10,  	Set_Serial_3);
	DOP_CreateSDO(TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_HW_VER, 			DOP_UINT8,		Set_HW_ver);
	DOP_CreateSDO(TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SW_VER_MAJOR,     DOP_UINT8,  	Set_SW_ver_major);
	DOP_CreateSDO(TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SW_VER_MINOR,     DOP_UINT8,  	Set_SW_ver_minor);
	DOP_CreateSDO(TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SW_VER_PATCH,     DOP_UINT8,  	Set_SW_ver_patch);
	DOP_CreateSDO(TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SW_VER_DEBUG,     DOP_UINT16,  	Set_SW_ver_debug);

	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_FUNCTION,     										DOP_UINT8,  	Set_Function);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SUCCESS,     										DOP_UINT8,  	Receive_Success_MTC);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_PDO_LIST,     									DOP_UINT8,  	BT_Set_Send_PDO_List);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_APP_INTEGRITY,     									DOP_STRING10,  	Receive_decrypted_plain_txt);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_MOBILE_UUID,     									DOP_STRING10,  	Set_Mobile_UUID);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_USER_DATA,     									DOP_UINT32,  	Set_User_Data);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_USER_BODY_DATA,     								DOP_UINT32,  	Set_User_Body_Data);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_SOUND_SETTING,   								DOP_UINT8,  	Set_Sound_Setting);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_NEUTRAL_SAGITTAL_ANGLE, 							DOP_FLOAT32,  	Set_Neutral_Sagittal_Angle);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_LEFT_THIGH_NEUTRAL_SAGITTAL_OFFSET, 			  	DOP_FLOAT32,  	Set_Left_Hip_Neutral_Offset);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_RIGHT_THIGH_NEUTRAL_SAGITTAL_OFFSET, 		   	DOP_FLOAT32,  	Set_Right_Hip_Neutral_Offset);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_LEFT_SHANK_NEUTRAL_SAGITTAL_OFFSET, 			   	DOP_FLOAT32,  	Set_Left_Knee_Neutral_Offset);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_RIGHT_SHANK_NEUTRAL_SAGITTAL_OFFSET, 		   	DOP_FLOAT32,  	Set_Right_Knee_Neutral_Offset);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_NEUTRAL_POSTURE_CALIBRATION, 		   			DOP_UINT8,  	Set_Neutral_Posture_Calibration);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_MODE_TYPE,							 		   	DOP_UINT8,  	Set_Mode_Type);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_MODE_SMART, 							   			DOP_UINT8,  	Set_Smart_Mode);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_MODE_AQUA, 							   			DOP_UINT8,  	Set_Aqua_Mode);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_MODE_SPACE, 							   			DOP_UINT8,  	Set_Space_Mode);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_MODE_LIMIT, 							  		 	DOP_INT8,  	Set_Limit_Mode);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_MODE_SIT_STAND, 							 	  	DOP_FLOAT32,  	Set_Sit_Stand_Mode);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_TRAINING_START, 							  			DOP_UINT32,  	Training_Start);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_TRAINING_MODE_CHANGE, 							  	DOP_UINT32,  	Training_Mode_Change);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_TRAINING_FINISH, 							 	  	DOP_UINT32,  	Training_Finish);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_CONTROL_CM_STANDBY_MODE, 						 	DOP_UINT8,  	Set_Standby_Mode);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_CONTROL_CM_ASSIST_LEVEL, 						  	DOP_UINT8,  	Set_Assist_Level);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_STS_HOLD,						  				DOP_UINT8,  	Set_STS_Hold);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_STS_READY,					 	  				DOP_UINT8,  	Set_STS_Ready);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_STS_SITTOSTAND_ASSIST_LEVEL,					 	DOP_UINT8,  	Set_Sit_to_Stance_Torque);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SET_STS_STANDTOSIT_ASSIST_LEVEL,						DOP_UINT8,  	Set_Stance_to_Sit_Torque);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_MOTION_ANALYSIS_START,						  		DOP_UINT32,  	Motion_Analysis_Start);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_MOTION_ANALYSIS_CANCEL,						  		DOP_UINT32,  	Motion_Analysis_Cancel);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_MOTION_ANALYSIS_FINISH,						  		DOP_UINT32,  	Motion_Analysis_Finish);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_MOTION_ANALYSIS_DATA_RECEIVE_STATE,					DOP_UINT8,  	Motion_Analysis_Data_Receive_State);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_CHECK_ERROR_MSG_RESPONSE,							DOP_UINT8,  	Check_Error_Msg_State);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SPPB_START,									DOP_UINT8,  			SPPB_Start);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SPPB_STOP,									DOP_UINT8,  			SPPB_Stop);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_SPPB_ASSIST_ONOFF,							DOP_UINT8,  			SPPB_Assist_OnOff);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_TIMER_SET,							DOP_INT16,  			Timer_Set);
	BT_CreateSDO(BT_TASK_ID_ANGELA_Mobile_to_CM, SDO_ID_ANGELA_MTC_TIMER_RESET,							DOP_UINT8,  			Timer_Reset);

	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_BOOT_CHECK,										DOP_UINT8,  	Set_MANU_BOOT_CHECK);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_SUCCESS,											DOP_UINT8,  	Send_MANU_Success);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_NAME_SET,										DOP_STRING10,  	Set_MANU_name_set);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_SET_PDO_LIST,									DOP_UINT8,  	BT_Set_Send_PDO_List);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_START,											DOP_UINT8,  	Set_MANU_Start);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_ESP32_COMMAND,									DOP_UINT8,  	Set_ESP32_Command);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_MES_SETTING_UINT8,								DOP_UINT8,  	Set_MANU_MES_Setting_uint8);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_MES_SETTING_STRING10,							DOP_STRING10,  	Set_MANU_MES_Setting_string10);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_WIFI_TEST,										DOP_UINT8,  	Set_MANU_MES_WIFI_Test);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_SD_CARD_CHECK,									DOP_UINT8,  	Set_MANU_SD_Card_check);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_FDCAN_CHECK,										DOP_UINT8,  	Set_MANU_FD_CAN_check);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_HW_VERSION_CEHCK,								DOP_UINT8,  	Set_MANU_HW_Version_check);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_SW_VERSION_CEHCK,								DOP_UINT8,  	Set_MANU_SW_Version_check);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_SYSTEM_MONITORING,								DOP_UINT8,  	Set_MANU_System_Monitoring);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_IMU_CALIB_RH_START,								DOP_UINT8,  	Set_MANU_ImuCalib_RH_Start);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_IMU_CALIB_RH_STOP,								DOP_UINT8,  	Set_MANU_ImuCalib_RH_Stop);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_IMU_CALIB_LH_START,								DOP_UINT8,  	Set_MANU_ImuCalib_LH_Start);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_IMU_CALIB_LH_STOP,								DOP_UINT8,  	Set_MANU_ImuCalib_LH_Stop);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_ENCODER_CHECK_START,								DOP_UINT8,  	Set_MANU_EncoderCheck_Start);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_ENCODER_CHECK_STOP,								DOP_UINT8,  	Set_MANU_EncoderCheck_Stop);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_ENCODER_OFFSET,									DOP_UINT8,  	Set_MANU_Encoder_Offset);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_RH_RANGE_OFFSET,									DOP_UINT8,  	Set_MANU_RH_Range_Offset);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_RH_VALUE_OFFSET,									DOP_UINT8,  	Set_MANU_RH_Value_Offset);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_LH_RANGE_OFFSET,									DOP_UINT8,  	Set_MANU_LH_Range_Offset);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_LH_VALUE_OFFSET,									DOP_UINT8,  	Set_MANU_LH_Value_Offset);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_CURRENT_SENSOR_CHECK,							DOP_UINT8,  	Set_MANU_CurrentSensor_Check);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_MONITORING_ENABLE,								DOP_UINT8,  	Set_MANU_Monitoring_Enable);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_BUTTON_TEST_START,								DOP_UINT8,  	Set_MANU_ButtonTest_Start);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_RH_POSITIVE_MAX_POSITION,						DOP_UINT8,  	Set_MANU_RH_PositiveMax_Position);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_RH_NEGATIVE_MAX_POSITION,						DOP_UINT8,  	Set_MANU_RH_NegativeMax_Position);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_LH_POSITIVE_MAX_POSITION,						DOP_UINT8,  	Set_MANU_LH_PositiveMax_Position);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_LH_NEGATIVE_MAX_POSITION,						DOP_UINT8,  	Set_MANU_LH_NegativeMax_Position);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_DRIVING_TEST_END,								DOP_UINT8,  	Set_MANU_DrivingTest_End);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_LED_TEST,										DOP_UINT8,  	Set_MANU_Led_Test);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_SOUND_TEST_VOICE,								DOP_UINT8,  	Set_MANU_SoundTest_Voice);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_SOUND_TEST_BEEP,									DOP_UINT8,  	Set_MANU_SoundTest_Beep);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_MAGNETIC_SENSOR_TEST,							DOP_UINT8,  	Set_MANU_Magnetic_Sensor_Test);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_AGING_TEST_START,								DOP_UINT8,  	Set_MANU_AgingTest_Start);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_AGING_TEST_STOP,									DOP_UINT8,  	Set_MANU_AgingTest_Stop);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_ORG_CODE_SETTING,								DOP_STRING10,  	Set_MANU_Org_code);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_MOBILE_UUID_RESET,								DOP_UINT8,  	Set_MANU_Mobile_UUID_Reset);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_SET_RTC,											DOP_UINT8,  	Set_MANU_RTC);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_START_MEASURE_TORQUE,							DOP_UINT8,  	Start_toruqe_measure);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_STOP_MEASURE_TORQUE,								DOP_UINT8,  	Stop_torque_measure);
	BT_CreateSDO(BT_TASK_ID_MANU_Mobile_to_CM, SDO_ID_MANU_MTC_READ_ORG_CODE,									DOP_UINT8,  	Send_ORG_CODE);

	// PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_BLE_COMM_HDLR, BLECommHdlr);

	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_TEST1				, DOP_UINT8, 		1,		&BLECtrlObj.data_manu.test1);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_TEST2				, DOP_UINT16,		1, 		&BLECtrlObj.data_manu.test2);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_TEST3				, DOP_UINT32, 		1, 		&BLECtrlObj.data_manu.test3);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_TEST4				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data_manu.test4);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_RH_INC_ENC_VELO		, DOP_FLOAT32, 		1, 		&BLECtrlObj.data_manu.RH_inc_value);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_RH_ABS1_ENC_VELO		, DOP_FLOAT32,		1,		&BLECtrlObj.data_manu.RH_abs_1_value);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_RH_ABS2_ENC_VELO		, DOP_FLOAT32,		1, 		&BLECtrlObj.data_manu.RH_abs_2_value);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_LH_INC_ENC_VELO		, DOP_FLOAT32,		1, 		&BLECtrlObj.data_manu.LH_inc_value);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_LH_ABS1_ENC_VELO		, DOP_FLOAT32, 		1, 		&BLECtrlObj.data_manu.LH_abs_1_value);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_LH_ABS2_ENC_VELO		, DOP_FLOAT32,		1,		&BLECtrlObj.data_manu.LH_abs_2_value);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_RH_ABS1_ENC_DEG		, DOP_FLOAT32,		1, 		&BLECtrlObj.data_manu.RH_abs_value);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_LH_ABS1_ENC_DEG		, DOP_FLOAT32,		1, 		&BLECtrlObj.data_manu.LH_abs_value);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_CM_VOLTAGE			, DOP_FLOAT32, 		1, 		&BLECtrlObj.data_manu.CM_voltage);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_CM_CURRENT			, DOP_FLOAT32,		1,		&BLECtrlObj.data_manu.CM_current);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_CM_TEMP				, DOP_FLOAT32,		1, 		&BLECtrlObj.data_manu.CM_temp);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_RH_VOLTAGE			, DOP_FLOAT32,		1, 		&BLECtrlObj.data_manu.RH_voltage);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_RH_CURRENT			, DOP_FLOAT32, 		1, 		&BLECtrlObj.data_manu.RH_current);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_RH_TEMP				, DOP_FLOAT32,		1,		&BLECtrlObj.data_manu.RH_temp);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_LH_VOLTAGE			, DOP_FLOAT32,		1, 		&BLECtrlObj.data_manu.LH_voltage);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_LH_CURRENT			, DOP_FLOAT32,		1, 		&BLECtrlObj.data_manu.LH_current);
	BT_CreatePDO(BT_TASK_ID_MANU_CM_to_Mobile, PDO_ID_MANU_CTM_LH_TEMP				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data_manu.LH_temp);
	
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_BAT_LEVEL					, DOP_UINT8, 		1,		&BLECtrlObj.data.BatteryLevel);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_FSM_STATE_ID				, DOP_UINT16,		1, 		&BLECtrlObj.data.FSM_StateId);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_CONTROL_CM_ASSIST_MODE					, DOP_UINT8, 		1, 		&BLECtrlObj.data.AssistMode);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_CONTROL_CM_STANDBY_MODE					, DOP_UINT8, 		1, 		&BLECtrlObj.data.StandbyMode);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_CONTROL_CM_ASSIST_LEVEL					, DOP_UINT8, 		1, 		&BLECtrlObj.data.AssistanceLevel);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_CAN_OPERATE								, DOP_UINT8, 		1, 		&BLECtrlObj.data.canOperate);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_SESSION_ID					, DOP_UINT32, 		1, 		&BLECtrlObj.data.TrainingSessionID);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_SESSION_MODE_ID			, DOP_UINT32, 		1, 		&BLECtrlObj.data.TrainingModeID);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LEFT_THIGH_SAGITTAL_ANGLE				, DOP_FLOAT32, 		1,		&BLECtrlObj.data.LeftThighSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_RIGHT_THIGH_SAGITTAL_ANGLE				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.RightThighSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LEFT_HIP_POSITION_ACT					, DOP_INT16, 		1,		&BLECtrlObj.data.LeftHipPositionAct);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_RIGHT_HIP_POSITION_ACT					, DOP_INT16, 		1, 		&BLECtrlObj.data.RightHipPositionAct);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LEFT_SHANK_SAGITTAL_ANGLE				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.LeftShankSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_RIGHT_SHANK_SAGITTAL_ANGLE				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.RightShankSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LEFT_THIGH_NEUTRAL_SAGITTAL_ANGLE		, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.LeftThighNeutralSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_RIGHT_THIGH_NEUTRAL_SAGITTAL_ANGLE		, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.RightThighNeutralSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LEFT_SHANK_NEUTRAL_SAGITTAL_ANGLE		, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.LeftShankNeutralSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_RIGHT_SHANK_NEUTRAL_SAGITTAL_ANGLE		, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.RightShankNeutralSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_LEFT_AMPLITUDE				, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.LeftAmplitude);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_RIGHT_AMPLITUDE			, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.RightAmplitude);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_SESSION_DURATION			, DOP_UINT16,	 	1, 		&BLECtrlObj.data.SessionDuration);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_MODE_DURATION				, DOP_UINT16,	 	1, 		&BLECtrlObj.data.ModeDuration);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_OPERATION_DURATION			, DOP_UINT16,	 	1, 		&BLECtrlObj.data.OperationDuration);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_ASYMMETRIC_LEVEL			, DOP_INT8,	 		1, 		&BLECtrlObj.data.AsymmetricLevel);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_ASYMMETRIC_AVG				, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.AsymmetricLevelAvg);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_WALK_CNT					, DOP_UINT16, 		1, 		&BLECtrlObj.data.WalkCount);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_LEFT_WALK_COUNT			, DOP_UINT16,	 	1, 		&BLECtrlObj.data.LeftWalkCount);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_RIGHT_WALK_COUNT			, DOP_UINT16,	 	1, 		&BLECtrlObj.data.RightWalkCount);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_WALK_DISTANCE				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.WalkDistance);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_CAL_CALORIE				, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.CalculateCalorie);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_RIGHT_HIP_TORQUE			, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.RightHipTorque);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_LEFT_HIP_TORQUE			, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.LeftHipTorque);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_RIGHT_KNEE_TORQUE			, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.RightKneeTorque);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_DATA_LEFT_KNEE_TORQUE			, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.LeftKneeTorque);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LEFT_THIGH_VELOCITY_RAW					, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.LeftThighVelocityRaw);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_RIGHT_THIGH_VELOCITY_RAW					, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.RightThighVelocityRaw);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LEFT_SHANK_VELOCITY_RAW					, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.LeftShankVelocityRaw);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_RIGHT_SHANK_VELOCITY_RAW					, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.RightShankVelocityRaw);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_PELVIC_SAGITTAL_ANGLE					, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.PelvicSagittalAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_PELVIC_FRONTAL_ANGLE						, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.PelvicFrontalAngle);
	
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TRAINING_STEP_LENGTH_AVG					, DOP_UINT8,	 	1, 		&BLECtrlObj.data.StepLengthAvg);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_POSTURE_HOLD							, DOP_UINT8,	 	1, 		&BLECtrlObj.data.STSPostureHold);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_POSTURE_READY						, DOP_UINT8, 		1, 		&BLECtrlObj.data.STSPostureReady);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_SITTOSTAND_ASSIST_LEVEL				, DOP_UINT8, 		1, 		&BLECtrlObj.data.SittoStandAssistLevel);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_STANDTOSIT_ASSIST_LEVEL				, DOP_UINT8,	 	1, 		&BLECtrlObj.data.StandtoSitAssistLevel);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_SITTING_DURATION						, DOP_UINT16,	 	1, 		&BLECtrlObj.data.SittingDuration);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_STANDING_DURATION					, DOP_UINT16,	 	1, 		&BLECtrlObj.data.StandingDuration);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_SITTING_DURATION_TOTAL				, DOP_UINT16, 		1, 		&BLECtrlObj.data.SittingDurationTotal);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_STANDING_DURATION_TOTAL				, DOP_UINT16, 		1, 		&BLECtrlObj.data.StandingDurationTotal);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_SITTING_COUNT						, DOP_UINT16,	 	1, 		&BLECtrlObj.data.SittingCount);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_STS_STANDING_COUNT						, DOP_UINT16,	 	1, 		&BLECtrlObj.data.StandingCount);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LIMIT_LEFT_HIP_MAX_ANGLE					, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.LeftHipMaxAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LIMIT_RIGHT_HIP_MAX_ANGLE				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.RightHipMaxAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LIMIT_LEFT_HIP_MIN_ANGLE					, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.LeftHipMinAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LIMIT_RIGHT_HIP_MIN_ANGLE				, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.RightHipMinAngle);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LIMIT_LEFT_HIP_MAX_VELOCITY				, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.LeftHipMaxVelocity);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LIMIT_RIGHT_HIP_MAX_VELOCITY				, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.RightHipMaxVelocity);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LIMIT_LEFT_HIP_MIN_VELOCITY				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.LeftHipMinVelocity);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_LIMIT_RIGHT_HIP_MIN_VELOCITY				, DOP_FLOAT32, 		1, 		&BLECtrlObj.data.RightHipMinVelocity);

	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DURATION					, DOP_UINT16,	 	1, 		&BLECtrlObj.data.MotionAnalysisDuration);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_STATE					, DOP_UINT32,	 	1, 		&BLECtrlObj.data.ReportId);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_TYPE						, DOP_UINT32, 		1, 		&BLECtrlObj.data.MotionAnalysisType);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_WALK_COUNT				, DOP_UINT16, 		1, 		&BLECtrlObj.data.MotionAnalysisWalkCountPerMin);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_WALK_COUNT_LH			, DOP_UINT16,	 	1, 		&BLECtrlObj.data.MotionAnalysisLeftWalkCount);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_WALK_COUNT_RH			, DOP_UINT16,	 	1, 		&BLECtrlObj.data.MotionAnalysisRightWalkCount);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_STEP_LENGTH_LH			, DOP_UINT8,	 	1, 		&BLECtrlObj.data.MotionAnalysisLeftStepLength);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_STEP_LENGTH_RH			, DOP_UINT8, 		1, 		&BLECtrlObj.data.MotionAnalysisRightStepLength);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_FLEXION_DEG_LH			, DOP_INT16, 		1, 		&BLECtrlObj.data.MotionAnalysisFlexionDeg_LH);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_EXTENSION_LH				, DOP_INT16,	 	1, 		&BLECtrlObj.data.MotionAnalysisExtensionDeg_LH);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_FLEXION_DEG_RH			, DOP_INT16,	 	1, 		&BLECtrlObj.data.MotionAnalysisFlexionDeg_RH);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_EXTENSION_RH				, DOP_INT16,	 	1, 		&BLECtrlObj.data.MotionAnalysisExtensionDeg_RH);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_VELOCITY_MAX_LH			, DOP_INT16, 		1, 		&BLECtrlObj.data.MotionAnalysisVelocityMax_LH);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_VELOCITY_MAX_RH			, DOP_INT16, 		1, 		&BLECtrlObj.data.MotionAnalysisVelocityMax_RH);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_GROUND_CONTACK			, DOP_UINT8,	 	1, 		&BLECtrlObj.data.MotionAnalysisGroundContact);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_WALK_DISTANCE			, DOP_UINT16,	 	1, 		&BLECtrlObj.data.MotionAnalysisWalkDistance);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_MOTION_ANALYSIS_WALK_SPEED				, DOP_FLOAT32,	 	1, 		&BLECtrlObj.data.MotionAnalysisWalkSpeed);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_SPPB_WALK_SPEED							, DOP_UINT16,	 	1, 		&BLECtrlObj.data.SPPBWalkSpeed);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_SPPB_WALK_DISTANCE						, DOP_UINT16,	 	1, 		&BLECtrlObj.data.SPPBWalkDistance);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_SPPB_DURATION							, DOP_UINT16,	 	1, 		&BLECtrlObj.data.SPPBWalkDuration);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_SPPB_ASSIST_ONOFF						, DOP_UINT8,	 	1, 		&BLECtrlObj.data.SPPBAssistOnOff);
	BT_CreatePDO(BT_TASK_ID_ANGELA_CM_to_Mobile, PDO_ID_ANGELA_CTM_TIMER_REMAINING_TIME						, DOP_INT16,	 	1, 		&remaining_time);
	/* default values */
	BLECtrlObj.data.StandbyMode = 1;	// default standby mode
	memset(trainingInfo, 0, sizeof(trainingInfo));
	BLECtrlObj.data.SittoStandAssistLevel = 50;
	BLECtrlObj.data.StandtoSitAssistLevel = 50;

	DownloadBLESetData_FLASH();
}

void RunBLEComm(void)
{
	RunTask(&BLECommHdlr);
}

/* ----------------------- FUNCTION ----------------------- */
// Function to scale a float to a 16-bit integer type(int16)
int16_t ScaleFloatToInt16(float value, float scaleFactor)
{
    // Scale the float value
    int16_t scaledInt16Value = (int16_t)(value * DATA_CONV_CONST_INT16 / scaleFactor);

    return scaledInt16Value;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE MACHINE --------------------- */
static void StateOff_Ent(void)
{
	// No OFF state. Transit to Standby automatically.
}


static void StateOff_Run(void)
{

}


static void StateStandby_Ent(void)
{
	FLASH_ALL_DOWNLOAD();

	isBLEConnect.current_state = IOIF_ESP32_BT_IDLE;
	memset(&BLECtrlObj.data_manu, 0, sizeof(BLECtrlObj.data_manu));
}

static void StateStandby_Run(void)
{
	uint8_t wholebodyCtrlState = DOPC_GetTaskState(TASK_IDX_WHOLE_BODY_CTRL);
	if (wholebodyCtrlState == TASK_STATE_ENABLE)
	{
		IOIF_UART_UpdateHeadTail();											// flush buffer
		StateTransition(&BLECommHdlr.stateMachine, TASK_STATE_ENABLE);
	}
}


static void StateEnable_Ent(void)
{
	SetVersionSDO(2);
	SetVersionSDO(3);
}


static void StateEnable_Run(void)
{
	DOPI_SDOUnit_t sdoUnit;
	static bool Isbleconnectiontimeout = false;

	CheckFLASH_ReadWrite();

	if (IOIF_ESP32_BT_IsReady() && IOIF_ESP32_BT_IsConnected() == true) {
		isBLEConnect.current_state = IOIF_ESP32_BT_CONNECTED;
		bt_ready_sendcount= 0;
		/* CM - APP BLE connection 이후 초기 데이터 송수신 과정 */
		if(IsBLEconnectedtoApp == false)
		{
			BLEisConnectedNotifytoApp();
			/* */
			do{
				memset(BLECtrlObj.data.msgMTC, 0, 10);
				memset(&BLECtrlObj.data_manu.test, 0, 1);
				BT_AS_CB(&BLECtrlObj);

				bleconnection_trial++;
				if(bleconnection_trial > 10 && (BLECtrlObj.data_manu.test != 1)
						&& !(BLECtrlObj.data.msgMTC[0] == 3 && BLECtrlObj.data.msgMTC[1] == 4 && BLECtrlObj.data.msgMTC[2] == 5)
						&& set_uuid_flag == 0)
				{
					bleconnection_trial = 0;
					Isbleconnectiontimeout = true;
					break;
				}
				Isbleconnectiontimeout = false;
				osDelay(100);
				//if(BLECtrlObj.data_manu.test == 1 && IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12) == IOIF_GPIO_PIN_RESET){
				if(Is_ManuReady == true){
					manu_mode = 1;
				}
			}while(!( (BLECtrlObj.data_manu.test == 1)
			//}while(!( (manu_mode == 1)
					|| (BLECtrlObj.data.msgMTC[0] == 3 && BLECtrlObj.data.msgMTC[1] == 4 && BLECtrlObj.data.msgMTC[2] == 5)
					|| set_uuid_flag == 1));

			if(Isbleconnectiontimeout != true)
			{
				if(manu_mode == 1){
					if(IsOnceBLEconnectedtoApp == 0){
						IsOnceBLEconnectedtoApp = 1;
					}
					BLECtrlObj.data.msgCommand = 0;

					set_uuid_flag = 0;
					IsBLEconnectedtoApp = true;
					osDelay(1000);
				}
				else if(aes_res == 1){
					Success_id_cnt=0;
					memset(BLECtrlObj.data.msgCTM, 0, 10);
					DOPI_ClearSDO(&BLE_sdo_msg);
					BT_ADD_TASK_ID(&BLE_sdo_msg, BT_SDO);
					BLECtrlObj.data.LoginData[0]=BLECtrlObj.data.TrainingSessionID;
					BLECtrlObj.data.LoginData[1]=BLECtrlObj.data.TrainingModeID;
					BLECtrlObj.data.LoginData[2]=BLECtrlObj.data.UserID;
					BLECtrlObj.data.LoginData[3]=BLECtrlObj.data.ProUserID;
					sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_CHECK_LOGIN_DATA, SDO_REQU, 4);
					BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
					SendHwSwVersion();
					TxBLESDO();

					if(IsOnceBLEconnectedtoApp == 0){
						IsOnceBLEconnectedtoApp = 1;
					}
					BLECtrlObj.data.msgCommand = 0;

					set_uuid_flag = 0;
					IsBLEconnectedtoApp = true;
					osDelay(1000);
				}
				else{
					DOPI_ClearSDO(&BLE_sdo_msg);
					BT_ADD_TASK_ID(&BLE_sdo_msg, BT_SDO);
					BLECtrlObj.data.ErrorMSGcode=9;//app 무결성 검사 실패
					sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_ERROR_MSG_CODE, SDO_REQU, 1);
					BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
					TxBLESDO();
					osDelay(1000);
				}
			}
		}
		else	/* 초기 데이터 송수신 과정 완료 후 */
		{

			if(manu_mode == 1){

				BLECtrlObj.data_manu.test1++;
				BLECtrlObj.data_manu.test2++;
				BLECtrlObj.data_manu.test3++;
				BLECtrlObj.data_manu.test4++;
				BLECtrlObj.data_manu.RH_inc_value = userCtrlObj[2].data.inc_enc_velo/1.875;
				BLECtrlObj.data_manu.RH_abs_1_value = userCtrlObj[2].data.abs1_enc_velo/1.875;
				BLECtrlObj.data_manu.RH_abs_2_value = userCtrlObj[2].data.abs2_enc_velo/1.875;
				BLECtrlObj.data_manu.LH_inc_value = userCtrlObj[3].data.inc_enc_velo/1.875;
				BLECtrlObj.data_manu.LH_abs_1_value = userCtrlObj[3].data.abs1_enc_velo/1.875;
				BLECtrlObj.data_manu.LH_abs_2_value = userCtrlObj[3].data.abs2_enc_velo/1.875;
				BLECtrlObj.data_manu.RH_abs_value = userCtrlObj[2].data.abs1_enc/1.875;
				BLECtrlObj.data_manu.LH_abs_value = userCtrlObj[3].data.abs1_enc/1.875;
				BLECtrlObj.data_manu.CM_voltage = batData.batVolt;
				BLECtrlObj.data_manu.CM_current = batData.batCurr;
				BLECtrlObj.data_manu.CM_temp = batData.brdTemp;
				BLECtrlObj.data_manu.RH_voltage = userCtrlObj[2].data.actual_volt/2480;
				BLECtrlObj.data_manu.RH_current = userCtrlObj[2].data.actual_curr;
				BLECtrlObj.data_manu.RH_temp = userCtrlObj[2].data.motor_temp;
				BLECtrlObj.data_manu.LH_voltage = userCtrlObj[3].data.actual_volt/2480;
				BLECtrlObj.data_manu.LH_current = userCtrlObj[3].data.actual_curr;
				BLECtrlObj.data_manu.LH_temp = userCtrlObj[3].data.motor_temp;

				UpdateVariable();
				BT_AS_CB(&BLECtrlObj);
				osDelay(20);

				DOPI_ClearSDO(&BLE_sdo_msg);
				BT_Run_Send_PDO();

				osDelay(20);
			} else {
				UpdateVariable();
				BT_AS_CB(&BLECtrlObj);

				osDelay(20);
				DOPI_ClearSDO(&BLE_sdo_msg);
				if (!disablePDOSending){
					BT_Run_Send_PDO();
				}
				osDelay(30);
			}

			DOPI_ClearSDO(&BLE_sdo_msg);
			BT_ADD_TASK_ID(&BLE_sdo_msg, BT_SDO);

			if(manu_mode == 1 ){
				if(Success_id_cnt != 0)
				{
					BLECtrlObj.data_manu.IsConnectSuccess = 1;
					sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SUCCESS, SDO_REQU, Success_id_cnt);
					BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
					Success_id_cnt = 0 ;
					memset(BLECtrlObj.data_manu.msgCTM, 0, 10);
				}
			} else {
				if(Success_id_cnt != 0)
				{
					if (BLECtrlObj.data.msgCTM[0] == 1) {
						disablePDOSending = false;
					}

					if (BLECtrlObj.data.msgCTM[0] == 1 && BLECtrlObj.data.msgCTM[1] == 5 && BLECtrlObj.data.msgCTM[2] == 6
							&& BLECtrlObj.data.msgCTM[3] == 7 && BLECtrlObj.data.msgCTM[4] == 8) {
						BT_Clear_PDO_to_Send();
						memset(&BLECtrlObj.data.TrainingSessionID, 0, sizeof(BLECtrlObj.data.TrainingSessionID));
						memset(&BLECtrlObj.data.TrainingModeID, 0, sizeof(BLECtrlObj.data.TrainingModeID));
						memset(trainingInfo, 0, sizeof(trainingInfo));
						if(standbymode != 1)
						{
							standbymode = 1;
							BLECtrlObj.data.StandbyMode = standbymode;
							BFlag[31] = 1;
						}
						trainingInfo[current_session].isSessionActive = false;
					}


					sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SUCCESS, SDO_REQU, Success_id_cnt);
					BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
					Success_id_cnt = 0 ;
					memset(BLECtrlObj.data.msgCTM, 0, 10);
				}
			}

			// if(aes_send_sw==1){
			// 	memcpy(BLECtrlObj.data.aes_encrypted_base_64_res, (uint8_t *)base64_res, 24);
			// 	sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_APP_INTEGRITY, SDO_REQU, 3);
			// 	BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			// 	aes_send_sw = 0;
			// }

			CheckErrorMSG();
			//function here
			CheckNPOffset(&BLECtrlObj);
			/* 중립자세 flag 체크??? */
			CheckFlag();
			/* Standby mode 상태 업데이트 */
			CheckStandbyStatus();

			/* 동작 분석 리포트 데이터 송신 */
			if (isMotionAnalysisRunning){
				SendMotionAnalysisDataState();

				if (MotionAnalysisFinishAble && !onetime_send_finish_able) {
					SendMotionAnalysisFinishAble();
					onetime_send_finish_able = true;
				}

				if (ReportDataTransferState >= SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA
						&& ReportDataTransferState <= SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_RIGHT_Y_STD){
					SendMotionAnalysisReportData();
				}
			}

			if(manu_mode == 1 ){

				if(Set_MANU_RH_Offset_calibration_done == true && Set_MANU_LH_Offset_calibration_done == true)
				{
					/* Go Standby SAM10 states */
					AllDevStandbyStates();
					osDelay(50);
					/* Restart SAM10 all states*/
					AllDevEnableStates();
					Set_MANU_RH_Offset_calibration_done = false;
					Set_MANU_LH_Offset_calibration_done = false;
				}
				else
					AMS_Stepwise_Run(sdoUnit);
			}

			if(BLE_sdo_msg.msgLength > 6)
			{
				TxBLESDO();
			}
			//end

		}
	} else {

		DOPI_SDOUnit_t sdoUnitCMD;
//		static uint8_t bt_ready_sendcount = 0;

		if(IOIF_ESP32_BT_IsReady() == false)
		{
			//Ready packet 전송
			if(bt_ready_sendcount > 10) {
				DOPI_ClearSDO(&BLE_sdo_msg);
				BT_ADD_TASK_ID(&BLE_sdo_msg, BT_SDO);

				BLECtrlObj.data_cmd.IsReadyCheck = 1;
				sdoUnitCMD = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ESP32_CMD, SDO_ID_ESP32_CMD_READY_CHECK, SDO_REQU, 1);
				BT_AppendSDO(&sdoUnitCMD, &BLE_sdo_msg);

				TxBLESDO();
				bt_ready_sendcount = 0;
			}
			else
				bt_ready_sendcount++;
		}

		/* 동작 분석 관련 모든 값 초기화 */
		if (IsBLEconnectedtoApp) {
			onetime_send_finish_able = false;
			onetime_finish_set = false;
			isMotionAnalysisRunning = false;
			disablePDOSending = true;
			ReportId = 0;
			MotionAnalysisDataState = 255;
			ReportDataTransferState = 255;
			motion_analysis_cnt = 0;
			report_data_cnt = 0;
			report_data_sw = 0;
			isMotionAnalysisCancel = true;
			DOPI_ClearSDO(&BLE_sdo_msg);
		}

		manu_mode = 0;
		bleconnection_trial = 0;
		Is_ManuReady = false;

		/* 재연결 시 msgMTC 초기화 시킴. Success Msg. 초기화함.*/
		memset(BLECtrlObj.data.msgCTM, 0, 10);
		memset(BLECtrlObj.data_manu.msgCTM, 0, 10);

		isBLEConnect.current_state = IOIF_ESP32_BT_ADVERTISING;
		IsBLEconnectedtoApp = false;
		Isbleconnectiontimeout = false;
	}

	BLECommHdlrloopcnt++;
}

static void StateEnable_Ext(void)
{

}

static void StateError_Run(void)
{

}

/* ----------------------- FUNCTION ----------------------- */
static bool BLEInit(uint32_t baudrate)
{
	volatile bool ret = false;

	// UART Start
	if (IOIF_UART_StartReceive(IOIF_ESP32_UART_PORT, baudrate, IOIF_UART_MODE_DMA) != IOIF_UART_STATUS_OK)
		ret = IOIF_ESP32_INIT_ERROR;

	/* SDO & PDO Communication */
	BLE_Node_ID = NODE_ID_ALL;

	// Allocate Tx Function & SDO&PDO Communication Init
	BLE_txfnc = IOIF_ESP32_BT_TransmitData;

	BLECtrlObj.sdoTxID = SDO | (NODE_ID_CM << 4) | BLE_Node_ID;
	BLECtrlObj.txFnc = BLE_txfnc;

	memset(&BLECtrlObj.data, 0, sizeof(BLECtrlObj.data));
	DOPI_InitDevObj(&BLECtrlObj.devObj, BLE_Node_ID);
	SetupBLEDOD(&BLECtrlObj);

	return ret;
}

static void BLEisConnectedNotifytoApp(void)
{


	if(aes_encrypt_sw==1){
		srand(wholeBodyCtrlLoopCnt+RTC_Time.sec);
		random_number=rand();
		plain_text[0]=base64_table[(random_number)%64];
		random_number=rand();
		plain_text[1]=base64_table[(random_number)%64];
		random_number=rand();
		plain_text[2]=base64_table[(random_number)%64];
		random_number=rand();
		plain_text[3]=base64_table[(random_number)%64];
		random_number=rand();
		plain_text[4]=base64_table[(random_number)%64];
		random_number=rand();
		plain_text[5]=base64_table[(random_number)%64];

		memcpy(in, plain_text, 16);
		AES_init_ctx(&ctx, key);
		AES_ECB_encrypt(&ctx, in);
		base64_encode((uint8_t *)in, 16, &base64_len);

//		for(uint8_t i = 0; i < 22 ; i++){ // code for base64 check
//			for(uint8_t j = 0; j < 64 ; j++){
//				if(base64_res[i] == base64_table[j]){
//					break;
//				}
//				if(j == 63){
//					error++;
//				}
//
//			}
//
//		}

		aes_encrypt_sw =0;
	}
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&BLE_sdo_msg);
	BT_ADD_TASK_ID(&BLE_sdo_msg, BT_SDO);

	memcpy(BLECtrlObj.data.aes_encrypted_base_64_res, (uint8_t *)base64_res, 24);
	sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_APP_INTEGRITY, SDO_REQU, 3);
	BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

	memcpy(BLECtrlObj.data.MobileUuidFront, MobileUuidFront, 10);
	memcpy(BLECtrlObj.data.MobileUuidBack, MobileUuidBack, 10);
	sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOBILE_UUID, SDO_REQU, 2);
	BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

	memcpy(BLECtrlObj.data.Org_Code, Org_Code, 10);
	sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_CHECK_USER_ORG_CODE, SDO_REQU, 1);
	BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

	// BLECtrlObj.data.LoginData[0]=BLECtrlObj.data.TrainingSessionID;
	// BLECtrlObj.data.LoginData[1]=BLECtrlObj.data.TrainingModeID;
	// BLECtrlObj.data.LoginData[2]=BLECtrlObj.data.UserID;
	// BLECtrlObj.data.LoginData[3]=BLECtrlObj.data.ProUserID;
	// sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_CHECK_LOGIN_DATA, SDO_REQU, 4);
	// BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

	TxBLESDO();
}

static void CheckFLASH_ReadWrite(void)
{
	if (FLASHSaveFlag == IDLE_FLASH_BLE_DATA) {
		// Do Nothing
	} else if (FLASHSaveFlag == SAVE_FLASH_BLE_DATA) {
		SaveBLESetData_FLASH();
	} else if (FLASHSaveFlag == DOWNLOAD_FLASH_BLE_DATA) {
		DownloadBLESetData_FLASH();
	} else {
		// Do Nothing
	}
}


static void SPPB_TEST(void)
{
	DOPI_SDOUnit_t sdoUnit;
	if(SPPB_r_sw == 1 && BLECtrlObj.data.RightThighSagittalAngle < 50){
		SPPB_r_sw = 0;
	}else if(SPPB_r_sw == 1 && BLECtrlObj.data.LeftThighSagittalAngle < 50){
		SPPB_l_sw = 0;
	}else if(SPPB_r_sw == 0 && SPPB_mode == 6 && BLECtrlObj.data.RightThighSagittalAngle > 50){
		SPPB_r_sw = 1;
		BLECtrlObj.data.SPPB_Start_Flag = 4;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_START, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
		SPPB_mode = BLECtrlObj.data.SPPB_Start_Flag;
	}else if(SPPB_r_sw == 0 && SPPB_mode == 1 && BLECtrlObj.data.RightThighSagittalAngle > 50){
		SPPB_r_sw = 1;
		BLECtrlObj.data.SPPB_Stop_Flag = 1;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_STOP, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
		BLECtrlObj.data.SPPB_Stop_Flag = 0;
		SPPB_mode = 0;
	}else if(SPPB_r_sw == 0 && SPPB_mode == 2 && BLECtrlObj.data.RightThighSagittalAngle > 50){
		SPPB_r_sw = 1;
		BLECtrlObj.data.SPPB_Stop_Flag = 2;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_STOP, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
		BLECtrlObj.data.SPPB_Stop_Flag = 0;
		SPPB_mode = 0;
	}else if(SPPB_r_sw == 0 && SPPB_mode == 3 && BLECtrlObj.data.RightThighSagittalAngle > 50){
		SPPB_r_sw = 1;
		BLECtrlObj.data.SPPB_Stop_Flag = 3;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_STOP, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
		BLECtrlObj.data.SPPB_Stop_Flag = 0;
		SPPB_mode = 0;
	}else if(SPPB_mode == 4 && BLECtrlObj.data.SPPBWalkDistance > 400){
		BLECtrlObj.data.SPPB_Stop_Flag = 4;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_STOP, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
		BLECtrlObj.data.SPPB_Stop_Flag = 0;
		BLECtrlObj.data.SPPBWalkSpeed = 0;
		BLECtrlObj.data.SPPBWalkDistance = 0;
		BLECtrlObj.data.SPPBWalkDuration = 0;
		SPPB_mode = 0;
	}else if(SPPB_mode == 4 && BLECtrlObj.data.RightThighSagittalAngle > 50){
		BLECtrlObj.data.SPPBWalkSpeed += 50;
		BLECtrlObj.data.SPPBWalkDistance += 20;
	}else if(SPPB_mode == 4 && BLECtrlObj.data.LeftThighSagittalAngle > 50){
		BLECtrlObj.data.SPPBWalkSpeed -= 50;
		BLECtrlObj.data.SPPBWalkDuration += 10;
	}else if(SPPB_r_sw == 0 && SPPB_mode == 5 && BLECtrlObj.data.RightThighSagittalAngle > 50){
		SPPB_r_sw = 1;
		BLECtrlObj.data.SPPB_Standing_Count += 1;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_STANDING_COMPLETE, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
	}
}

static void UpdateVariable(void)
{
	BLECtrlObj.data.BatteryLevel = (uint8_t)batPctg;
	BLECtrlObj.data.FSM_StateId = FSMMngrObj.state_curr;
//	BLECtrlObj.data.AssistMode = assistmode;
//	BLECtrlObj.data.StandbyMode = standbymode;
	BLECtrlObj.data.AssistanceLevel = assistStage;
	BLECtrlObj.data.canOperate = canOperate;
//	BLECtrlObj.data.TrainingSessionID = trainingsessionid;
	BLECtrlObj.data.RightThighSagittalAngle = R_ThighSagittalAngle;
	BLECtrlObj.data.LeftThighSagittalAngle = L_ThighSagittalAngle;
	BLECtrlObj.data.RightHipPositionAct = (int16_t)((RH_PositionAct)*10);
	BLECtrlObj.data.LeftHipPositionAct = (int16_t)((LH_PositionAct)*10);
	BLECtrlObj.data.RightShankSagittalAngle = RK_PositionAct;
	BLECtrlObj.data.LeftShankSagittalAngle = LK_PositionAct;
	BLECtrlObj.data.RightThighNeutralSagittalAngle = RH_ThighNeutralSagittalAngle;
	BLECtrlObj.data.LeftThighNeutralSagittalAngle = LH_ThighNeutralSagittalAngle;
	BLECtrlObj.data.RightShankNeutralSagittalAngle = RK_ThighNeutralSagittalAngle;
	BLECtrlObj.data.LeftShankNeutralSagittalAngle = LK_ThighNeutralSagittalAngle;

	BLECtrlObj.data.RightThighVelocityRaw = RH_EncoderVelocity;
	BLECtrlObj.data.LeftThighVelocityRaw = LH_EncoderVelocity;
	BLECtrlObj.data.RightShankVelocityRaw = RK_EncoderVelocity;
	BLECtrlObj.data.LeftShankVelocityRaw = LK_EncoderVelocity;
	BLECtrlObj.data.PelvicSagittalAngle = PelvicSagittalAngle;
	BLECtrlObj.data.PelvicFrontalAngle = PelvicFrontalAngle;
	BLECtrlObj.data.StepLengthAvg = trainingInfo[current_session].step_length_avg[BLECtrlObj.data.AssistMode];

	/* 세션 훈련 시간 */
	BLECtrlObj.data.SessionDuration = trainingInfo[current_session].session_duration;
	/* 모드 사용시간 */
	BLECtrlObj.data.ModeDuration = trainingInfo[current_session].mode_duration[BLECtrlObj.data.AssistMode];
	/* 로봇 운용시간 */
	BLECtrlObj.data.OperationDuration = trainingInfo[current_session].operation_duration;
	/* 보행 수 */
	BLECtrlObj.data.WalkCount = trainingInfo[current_session].walk_count[BLECtrlObj.data.AssistMode];
	/* 보행 거리 */
	BLECtrlObj.data.WalkDistance = trainingInfo[current_session].walk_distance[BLECtrlObj.data.AssistMode];
	/* 비대칭 지수 */
	BLECtrlObj.data.AsymmetricLevel = (int8_t) AsymmetryFilter;
	BLECtrlObj.data.AsymmetricLevelAvg = trainingInfo[current_session].asymmetric_level_avg[BLECtrlObj.data.AssistMode];
	/* 오른쪽 엉덩이 토크 */
	BLECtrlObj.data.RightHipTorque = RH_Sagittal.MotorActCurrent * TORQUE_CONSTANT_SAM * H10_SUIT_GEAR_RATIO;
	/* 왼쪽 엉덩이 토크 */
	BLECtrlObj.data.LeftHipTorque = LH_Sagittal.MotorActCurrent * TORQUE_CONSTANT_SAM * H10_SUIT_GEAR_RATIO;
	/* 오른쪽 무릎 토크 */
	BLECtrlObj.data.RightKneeTorque = RK_Sagittal.MotorActCurrent * TORQUE_CONSTANT_SAM * H10_SUIT_GEAR_RATIO;
	/* 왼쪽 무릎 토크 */
	BLECtrlObj.data.LeftKneeTorque = LK_Sagittal.MotorActCurrent * TORQUE_CONSTANT_SAM * H10_SUIT_GEAR_RATIO;
	/* 오른다리 진폭 */
	BLECtrlObj.data.RightAmplitude = RHamplitude;
	/* 왼다리 진폭 */
	BLECtrlObj.data.LeftAmplitude = LHamplitude;
	/* 보행 수 좌 */
	BLECtrlObj.data.LeftWalkCount = StrideCountLeft_Session;
	/* 보행 수 우 */
	BLECtrlObj.data.RightWalkCount = StrideCountRight_Session;
	/* 칼로리 값 추가 안함 */
	BLECtrlObj.data.CalculateCalorie = 0;

//	BLECtrlObj.data.SittingDuration = (uint16_t)sitTime/100;
//	BLECtrlObj.data.StandingDuration = (uint16_t)standTime/100;
	BLECtrlObj.data.SittingDuration = (uint16_t)sitTime;
	BLECtrlObj.data.StandingDuration = (uint16_t)standTime;
	BLECtrlObj.data.SittingCount = sitCount;
	BLECtrlObj.data.StandingCount = standCount;

//	BLECtrlObj.data.SittingDurationTotal = (uint16_t)(accumulateSitTime/100);
//	BLECtrlObj.data.StandingDurationTotal = (uint16_t)(accumulateStandTime/100);
	BLECtrlObj.data.SittingDurationTotal = (uint16_t)accumulateSitTime;
	BLECtrlObj.data.StandingDurationTotal = (uint16_t)accumulateStandTime;

	BLECtrlObj.data.LeftHipMaxAngle = LeftHipMaxAngle + 15.0;
	BLECtrlObj.data.RightHipMaxAngle = RightHipMaxAngle + 15.0;
	BLECtrlObj.data.LeftHipMinAngle = LeftHipMinAngle + 15.0;
	BLECtrlObj.data.RightHipMinAngle = RightHipMinAngle + 15.0;
	BLECtrlObj.data.LeftHipMaxVelocity = LeftHipMaxVelocity;
	BLECtrlObj.data.RightHipMaxVelocity = RightHipMaxVelocity;
	BLECtrlObj.data.LeftHipMinVelocity = LeftHipMinVelocity;
	BLECtrlObj.data.RightHipMinVelocity = RightHipMinVelocity;

	/*  */
	BLECtrlObj.data.MotionAnalysisDuration = (uint16_t) MotionAnalysisDuration;
	BLECtrlObj.data.ReportId = ReportId;
	BLECtrlObj.data.MotionAnalysisType = MotionAnalysisType;

	BLECtrlObj.data.MotionAnalysisWalkCountPerMin = Cadence_Improved;
	BLECtrlObj.data.MotionAnalysisLeftWalkCount = StrideCountLeft_Improved;
	BLECtrlObj.data.MotionAnalysisRightWalkCount = StrideCountRight_Improved;

	BLECtrlObj.data.MotionAnalysisLeftStepLength = (uint8_t) (StepLengthLeft_Improved * 100);
	BLECtrlObj.data.MotionAnalysisRightStepLength = (uint8_t) (StepLengthRight_Improved * 100);

	BLECtrlObj.data.MotionAnalysisFlexionDeg_LH = (int16_t) LeftHipROMFlex_Improved;
	BLECtrlObj.data.MotionAnalysisExtensionDeg_LH = (int16_t) LeftHipROMExt_Improved;
	BLECtrlObj.data.MotionAnalysisFlexionDeg_RH = (int16_t) RightHipROMFlex_Improved;
	BLECtrlObj.data.MotionAnalysisExtensionDeg_RH = (int16_t) RightHipROMExt_Improved;
	BLECtrlObj.data.MotionAnalysisVelocityMax_LH = (int16_t) LeftHipMaxSpeed_Improved;
	BLECtrlObj.data.MotionAnalysisVelocityMax_RH = (int16_t) RightHipMaxSpeed_Improved;

	BLECtrlObj.data.MotionAnalysisGroundContact = GroundContact;
	BLECtrlObj.data.MotionAnalysisWalkDistance = forwardDistance_Improved;
	BLECtrlObj.data.MotionAnalysisWalkSpeed = velXKinematics_Improved[0];

}

static void CheckStandbyStatus(void)
{
//	// App이 아닌 다른 이유로 Standby State로 천이되었을 경우
//	// App의 Standby Mode = 1로 업데이트
//	if (oneTimeStateModeFlag == true && (FSMMngrObj.state_curr == 1 || (FSMMngrObj.state_curr >= 18 && FSMMngrObj.state_curr <= 26))
//		&& FSMMngrObj.state_curr != FSMMngrObj.state_prev && BLECtrlObj.data.StandbyMode == prevStandbyModefromApp) {
//		BLECtrlObj.data.StandbyMode = 1;
//		prevStandbyModefromApp = 1;
//		oneTimeStateModeFlag = false;
//		oneTimeAssistModeFlag = true;
//	}
//	// App이 아닌 다른 이유로 Assist Mode로 변경되었을 경우
//	// App의 Standby Mode = 0으로 업데이트
//	else if (oneTimeAssistModeFlag == true && FSMMngrObj.state_curr > 1 && FSMMngrObj.state_curr < 18
//		&& FSMMngrObj.state_curr != FSMMngrObj.state_prev && BLECtrlObj.data.StandbyMode == prevStandbyModefromApp) {
//		BLECtrlObj.data.StandbyMode = 0;
//		prevStandbyModefromApp = 0;
//		oneTimeAssistModeFlag = false;
//		oneTimeStateModeFlag = true;
//	}
	// App이 아닌 다른 이유로 Standby State로 천이되었을 경우
	// App의 Standby Mode = 1로 업데이트
	if ((FSMMngrObj.state_curr == 1 || (FSMMngrObj.state_curr >= 18 && FSMMngrObj.state_curr <= 26))){
		standbymode = 1;
		BLECtrlObj.data.StandbyMode = standbymode;
	}
	// App이 아닌 다른 이유로 Assist Mode로 변경되었을 경우
	// App의 Standby Mode = 0으로 업데이트
	else if ((FSMMngrObj.state_curr > 1 && FSMMngrObj.state_curr < 18)
			|| (FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45)
			|| (FSMMngrObj.state_curr >= 65 && FSMMngrObj.state_curr <= 75)) {
		standbymode = 0;
		BLECtrlObj.data.StandbyMode = standbymode;
	}
}

static void AMS_Stepwise_Run(DOPI_SDOUnit_t sdoUnit){
	
	static float vol_aver = 0;
	static float curr_aver = 0;
	static float temp_aver = 0;
	static float per_aver = 0;

	static uint16_t vol_aver_cnt = 0;
	static uint16_t curr_aver_cnt = 0;
	static uint16_t temp_aver_cnt = 0;
	static uint16_t per_aver_cnt = 0;

	if(manu_step == CMD_MANU_START ){
		if(one_time_sw == 1 && ble_check_status==1 && ble_cnt >=10){

			BLECtrlObj.data_manu.start_result = CMD_MANU_START;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_START_CONFIRMED, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw = 2;
		} 

		if(one_time_sw==2 && ble_cnt >=30){

			FLASH_ALL_DOWNLOAD();

			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MES_CONFIRM_UINT8, SDO_REQU, 6);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MES_CONFIRM_STRING10, SDO_REQU, 3);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

			one_time_sw=0;
			ble_check_status=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 

		}

	} else if (manu_step == CMD_MES_SETTING){
			FLASH_ALL_WRITE();
			FLASH_ALL_DOWNLOAD();
			
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MES_CONFIRM_UINT8, SDO_REQU, 6);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MES_CONFIRM_STRING10, SDO_REQU, 3);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
	} else if (manu_step == CMD_WIFI_TEST) {
		if( SPI_test_sw == 1 && SPI_test_result == 1 && ble_cnt > 50){

			BLECtrlObj.data_manu.SPI_test_result = 1;
			BLECtrlObj.data_manu.Wifi_test_result = 0;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_ESP32_COMM_TEST_RESULT, SDO_REQU, 2);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			SPI_test_sw = 0;
			SPI_test_result = 0;

			ble_cnt=0;
			manu_step = CMD_IDLE_;
		}
	} else if (manu_step == CMD_SD_CARD_TEST) {
		if( Is_ManuSDCardState == true && ble_cnt >= 10){
			BLECtrlObj.data_manu.sdcard_test_result = 1;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SD_CARD_CONFIRMED, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}
			
	} else if (manu_step == CMD_FDCAN_TEST) {
//		can_test[NODE_ID_RH_SAG] = 1;
//		can_test[NODE_ID_LH_SAG] = 1;
		uint8_t RCT_result = 1;
		if(can_test[NODE_ID_RH_SAG] == 1 && can_test[NODE_ID_LH_SAG] == 1 && ble_cnt >= 10){
			BLECtrlObj.data_manu.fdcan_test_result = 1;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_FDCAN_CONFIRMED, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			RCT_result = IOIF_MCP79510_GetTime(&RTC_Time);
			if(RCT_result==0){
				memcpy(BLECtrlObj.data_manu.RTC_data, &RTC_Time, 7);
			}
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CONFIRM_RTC, SDO_REQU, 7);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}
		else if((can_test[NODE_ID_RH_SAG] == 0 || can_test[NODE_ID_LH_SAG] ==0) && ble_cnt >= 10){
			BLECtrlObj.data_manu.fdcan_test_result = 0;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_FDCAN_CONFIRMED, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			RCT_result = IOIF_MCP79510_GetTime(&RTC_Time);
			if(RCT_result==0){
				memcpy(BLECtrlObj.data_manu.RTC_data, &RTC_Time, 7);
			}
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CONFIRM_RTC, SDO_REQU, 7);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_;
		}
	} else if (manu_step == CMD_HW_VERSION_CHECK) {
		if(one_time_sw==1)
		{

			SetSerialSDO(2);
			SetSerialSDO(3);
			one_time_sw = 2;
		}
		if(one_time_sw==2)
		{
			uint8_t suit_hw_version = IOIF_SYS_HwRevGPIO();
			BLECtrlObj.data_manu.CM_HW_ver = suit_hw_version;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_HW_VERSION_CONFIRMED, SDO_REQU, 3);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}

	} else if (manu_step == CMD_SW_VERSION_CHECK) {
		if(one_time_sw==1)
		{
			SetSerialSDO(2);
			SetSerialSDO(3);
			one_time_sw = 2;
		}
		if(one_time_sw==2 && ble_cnt >10)
		{

			BLECtrlObj.data_manu.CM_SW_ver_major = FW_VER_MAJOR;
			BLECtrlObj.data_manu.CM_SW_ver_minor = FW_VER_MINOR;
			BLECtrlObj.data_manu.CM_SW_ver_patch = FW_VER_PATCH;

			BLECtrlObj.data_manu.ESP32_SW_ver_major = 0;
			BLECtrlObj.data_manu.ESP32_SW_ver_minor = 0;
			BLECtrlObj.data_manu.ESP32_SW_ver_patch = 0;
			
			BLECtrlObj.data_manu.CONTENTS_FILE_ver_major = FSM_File.file_version_major;
			BLECtrlObj.data_manu.CONTENTS_FILE_ver_minor = FSM_File.file_version_minor;
			BLECtrlObj.data_manu.CONTENTS_FILE_ver_patch = FSM_File.file_version_patch;

			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SW_VERSION_CONFIRMED_UINT8, SDO_REQU, 15);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SW_VERSION_CONFIRMED_STRING10, SDO_REQU, 6);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}
	} else if (manu_step == CMD_SYSTEM_MONITORING) {
		if(ble_cnt <= 20 && ble_cnt >5)
		{
			vol_aver += batData.batVolt;
			curr_aver += batData.batCurr;
			temp_aver += batData.brdTemp;
			per_aver += batData.percentage;

			vol_aver_cnt++;
			curr_aver_cnt++;
			temp_aver_cnt++;
			per_aver_cnt++;
		}

		if( ble_cnt >= 20)
		{
//			BLECtrlObj.data_manu.CM_Voltage_aver 	= vol_aver / (float)vol_aver_cnt;
//			BLECtrlObj.data_manu.CM_Current_aver 	= curr_aver / (float)curr_aver_cnt;
//			BLECtrlObj.data_manu.CM_Temp_aver 	 	= temp_aver / (float)temp_aver_cnt;
			BLECtrlObj.data_manu.CM_Voltage_aver 	= vol_aver / (float)vol_aver_cnt;
			BLECtrlObj.data_manu.CM_Current_aver 	= curr_aver / (float)curr_aver_cnt;
			BLECtrlObj.data_manu.CM_Temp_aver 	 	= temp_aver / (float)temp_aver_cnt;
			BLECtrlObj.data_manu.CM_Percentage_aver = batPctg;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SYSTEM_MONITORING_DONE, SDO_REQU, 4);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}
	} else if (manu_step == CMD_IMU_CALIB_RH_START) {
			DOPI_SDOUnit_t sdo_unit;
			DOPI_ClearSDO(&sdo_msg);

			userCtrlObj[2].data.autoCalibSensorCmd = 1;
			sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_AUTO_CALIB_SENSOR, SDO_REQU, 1);
			DOPI_AppendSDO(&sdo_unit, &sdo_msg);
			TxSDO(2);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 

	} else if (manu_step == CMD_IMU_CALIB_RH_STOP) {
		if(one_time_sw ==1)
		{
			AllDevEnableStates();
			one_time_sw = 2;
		}

		if( ble_cnt >= 5)
		{
			BLECtrlObj.data_manu.RH_acc_mean_value = userCtrlObj[2].data.accMeanVal_calibrated;
			BLECtrlObj.data_manu.RH_gyro_scailing = userCtrlObj[2].data.scaling_gyro;
			BLECtrlObj.data_manu.RH_gyro_bias = userCtrlObj[2].data.bias_gyro;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_IMU_CALIB_RH_STOP_DONE, SDO_REQU, 3);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}
	} else if (manu_step == CMD_IMU_CALIB_LH_START) {
			DOPI_SDOUnit_t sdo_unit;
			DOPI_ClearSDO(&sdo_msg);
			userCtrlObj[3].data.autoCalibSensorCmd = 1;
			sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_AUTO_CALIB_SENSOR, SDO_REQU, 1);
			DOPI_AppendSDO(&sdo_unit, &sdo_msg);
			TxSDO(3);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 

	} else if (manu_step == CMD_IMU_CALIB_LH_STOP) {
		if(one_time_sw ==1)
		{
			AllDevEnableStates();
			one_time_sw = 2;
		}

		if( ble_cnt >= 5)
		{
			BLECtrlObj.data_manu.LH_acc_mean_value = userCtrlObj[3].data.accMeanVal_calibrated;
			BLECtrlObj.data_manu.LH_gyro_scaling = userCtrlObj[3].data.scaling_gyro;
			BLECtrlObj.data_manu.LH_gyro_bias = userCtrlObj[3].data.bias_gyro;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_IMU_CALIB_LH_STOP_DONE, SDO_REQU, 3);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}
	} else if (manu_step == CMD_ENCODER_CHECK_START) {
	} else if (manu_step == CMD_ENCODER_CHECK_STOP) {
	} else if (manu_step == CMD_RH_MD_RANAGE_OFFSET) {
	} else if (manu_step == CMD_RH_MD_VALUE_OFFSET) {
	} else if (manu_step == CMD_LH_MD_RANAGE_OFFSET) {
	} else if (manu_step == CMD_LH_MD_VALUE_OFFSET) {
	} else if (manu_step == CMD_MD_CURRENT_SENSOR_CHECK) {
		BLECtrlObj.data_manu.current_test_result = 1;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CURRENT_SENSOR_CONFIRMED, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
		one_time_sw=0;
		ble_cnt=0;
		manu_step = CMD_IDLE_;
	} else if (manu_step == CMD_MONITORING_ENABLE) {
//		BLECtrlObj.data_manu.current_test_result = 1;
//		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CURRENT_SENSOR_CONFIRMED, SDO_REQU, 1);
//		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
//		one_time_sw=0;
//		ble_cnt=0;
		manu_step = CMD_IDLE_;
	} else if (manu_step == CMD_BUTTON_TEST) {

		if(BTN_test[0] == 1 && BTN_res[0] == 0 ){


			BLECtrlObj.data_manu.button_test_result = 1;

			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BUTTON_TEST_DONE, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			BTN_res[0] = 1;

			ble_cnt=0;
		}

		if(BTN_test[1] == 1 && BTN_res[1] == 0){


			BLECtrlObj.data_manu.button_test_result = 2;

			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BUTTON_TEST_DONE, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			BTN_res[1] = 1;

			ble_cnt=0;
		}

		if(BTN_test[2] == 1 && BTN_res[2] == 0){


			BLECtrlObj.data_manu.button_test_result = 3;

			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BUTTON_TEST_DONE, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			BTN_res[2] = 1;

			ble_cnt=0;
		}

		if(BTN_res[0] == 1 && BTN_res[1] == 1 && BTN_res[2] == 1 && ble_cnt>10){
			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}

	} else if (manu_step == CMD_RH_MD_POSITVE_MAX_POSITION) {
	} else if (manu_step == CMD_RH_MD_NEGATIVE_MAX_POSITION) {
	} else if (manu_step == CMD_LH_MD_POSITVE_MAX_POSITION) {
	} else if (manu_step == CMD_LH_MD_NEGATIVE_MAX_POSITION) {
	} else if (manu_step == CMD_LED_TEST) {
	} else if (manu_step == CMD_SOUND_TEST_VOICE) {
	} else if (manu_step == CMD_SOUND_TEST_BEEP) {
	} else if (manu_step == CMD_MAGNETIC_SENSOR_TEST) {
		if(MAG_test[0] == 1 && MAG_res[0] == 0){


			BLECtrlObj.data_manu.magnetic_test_result = 1;

			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MAGNETIC_SENSOR_TEST_DONE, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			MAG_res[0] = 1;

			ble_cnt=0;
		}

		if(MAG_test[1] == 1 && MAG_res[1] == 0){


			BLECtrlObj.data_manu.magnetic_test_result = 2;

			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MAGNETIC_SENSOR_TEST_DONE, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
			MAG_res[1] = 1;

			ble_cnt=0;
		}

		if(MAG_res[0] == 1 && MAG_res[1] == 1 && ble_cnt>10){
			MAG_res[0] = 0;
			MAG_res[1] = 0;

			one_time_sw=0;
			ble_cnt=0;
			manu_step = CMD_IDLE_; 
		}
	} else if (manu_step == CMD_AGING_TEST_START) {
	} else if (manu_step == CMD_AGING_TEST_STOP) {
	} else if (manu_step == CMD_ORG_SETTING) {
		memcpy(BLECtrlObj.data.Org_Code, Org_Code, 10);
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_ORG_CODE_CONFIRMED, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

		manu_step = CMD_IDLE_;
		ble_cnt=0;
	} else if (manu_step == CMD_MOBILE_UUID_RESET) {
		BLECtrlObj.data_manu.uuid_reset_result = uuid_reset_result;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MOBILE_UUID_CONFIRMED, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

		manu_step = CMD_IDLE_;
		ble_cnt=0;
		uuid_reset_result = 0;
	} else if (manu_step == CMD_ORG_READING) {

		memcpy(BLECtrlObj.data.Org_Code, Org_Code, 10);
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_ORG_CODE_CONFIRMED, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

		manu_step = CMD_IDLE_;
		ble_cnt=0;
	}
	
	ble_cnt++;

}

static void CheckFlag(void)
{
	if ((neutralPosCalCMD == 0 && isNeutralCalCompleted == true && oneTimeSendNeutralPosture == 1) || updateNeutralPostureFlag == true) 
	{ // Neutral Posture Cal Completed

		BLECtrlObj.data.RightThighNeutralSagittalAngle = RH_ThighNeutralSagittalAngle;
		BLECtrlObj.data.LeftThighNeutralSagittalAngle = LH_ThighNeutralSagittalAngle;
		BLECtrlObj.data.RightShankNeutralSagittalAngle = RK_ThighNeutralSagittalAngle;
		BLECtrlObj.data.LeftShankNeutralSagittalAngle = LK_ThighNeutralSagittalAngle;

		DOPI_SDOUnit_t sdoUnit;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_NEUTRAL_POSTURE_CALIBRATION, SDO_REQU, 4);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);


		oneTimeSendNeutralPosture = 0;
		updateNeutralPostureFlag = false;

		FLASHSaveFlag = SAVE_FLASH_BLE_DATA;
	}
}

static void TxBLESDO(void)
{

	if(BLE_sdo_msg.txBuf[0]==2){
		memcpy(prev_BLE_sdo_msg.txBuf,BLE_sdo_msg.txBuf,244);
		prev_BLE_sdo_msg.msgLength = BLE_sdo_msg.msgLength;

	}

	uint16_t t_checksum;
	t_checksum = Checksum_Calcul(BLE_sdo_msg.txBuf, BLE_sdo_msg.msgLength);
	memcpy(BLE_sdo_msg.txBuf + BLE_sdo_msg.msgLength, &t_checksum, 2);
	BLE_sdo_msg.msgLength=BLE_sdo_msg.msgLength + 2;

	memcpy(&uart1dmaTxBuff, &BLE_sdo_msg.txBuf, 244);
	BLECtrlObj.txFnc(uart1dmaTxBuff, BLE_sdo_msg.msgLength);
}

static void CheckErrorMSG()
{
	DOPI_SDOUnit_t sdoUnit;
	if(errorMSGcode!=0){
		BLECtrlObj.data.ErrorMSGcode=errorMSGcode;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_ERROR_MSG_CODE, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
	}

}

static void re_TxBLESDO(void)
{
	uint16_t t_checksum;

	t_checksum = Checksum_Calcul(prev_BLE_sdo_msg.txBuf, prev_BLE_sdo_msg.msgLength);
	memcpy(prev_BLE_sdo_msg.txBuf + prev_BLE_sdo_msg.msgLength, &t_checksum, 2);
	prev_BLE_sdo_msg.msgLength=prev_BLE_sdo_msg.msgLength + 2;

	memcpy(&uart1dmaTxBuff, &prev_BLE_sdo_msg.txBuf, 244);
	BLECtrlObj.txFnc(uart1dmaTxBuff, prev_BLE_sdo_msg.msgLength);
}

static void CheckNPOffset(AS_BLE_CtrlObj_t* obj)
{
	// RightThighNeutralPostureOffset 값이 변했을 때만 업데이트
	if (rightHipNPOffsetFlag == true) {
		// TODO : SDO SUIT -> SAM
		static bool oneTimeRHSendOffset = true;
		if (oneTimeRHSendOffset == true) {
			int16_t scaledOffset = ScaleFloatToInt16(obj->data.RightThighNeutralPostureOffset, DEG_SCALING_FACTOR);
			SendNeutralPostureOffset(RH_SAGITAL, scaledOffset);
			oneTimeRHSendOffset = false;
		}

		UpdateNotiSignal();

        if (RH_Sagittal.notiSignals == 1) {
            RH_Sagittal.notiSignals = 0;
			userCtrlObj[RH_SAGITAL].devObj.noti = 0;
			updateNeutralPostureFlag = true;
			rightHipNPOffsetFlag = false;
			oneTimeRHSendOffset = true;
			RH_NPOffsetCompleted = true;
        }
	}

	// LeftThighNeutralPostureOffset 값이 변했을 때만 업데이트
	if (leftHipNPOffsetFlag == true) {
		// TODO : SDO SUIT -> SAM
		static bool oneTimeLHSendOffset = true;
		if (oneTimeLHSendOffset == true) {
			int16_t scaledOffset = ScaleFloatToInt16(obj->data.LeftThighNeutralPostureOffset, DEG_SCALING_FACTOR);
			SendNeutralPostureOffset(LH_SAGITAL, scaledOffset);
			oneTimeLHSendOffset = false;
		}

		UpdateNotiSignal();

        if (LH_Sagittal.notiSignals == 1) {
            LH_Sagittal.notiSignals = 0;
			userCtrlObj[LH_SAGITAL].devObj.noti = 0;
			updateNeutralPostureFlag = true;
			leftHipNPOffsetFlag = false;
			oneTimeLHSendOffset = true;
			LH_NPOffsetCompleted = true;
        }
	}

	// RightShankNeutralPostureOffset 값이 변했을 때만 업데이트
	if (rightKneeNPOffsetFlag == true) {
		// TODO : SDO SUIT -> SAM
		static bool oneTimeRKSendOffset = true;
		if (oneTimeRKSendOffset == true) {
			int16_t scaledOffset = ScaleFloatToInt16(obj->data.RightShankNeutralPostureOffset, DEG_SCALING_FACTOR);
			SendNeutralPostureOffset(RK_SAGITAL, scaledOffset);
			oneTimeRKSendOffset = false;
		}

		UpdateNotiSignal();

        if (RK_Sagittal.notiSignals == 1) {
            RK_Sagittal.notiSignals = 0;
			userCtrlObj[RK_SAGITAL].devObj.noti = 0;
			updateNeutralPostureFlag = true;
			rightKneeNPOffsetFlag = false;
			oneTimeRKSendOffset = true;
			RK_NPOffsetCompleted = true;
        }
	}

	// LeftShankNeutralPostureOffset 값이 변했을 때만 업데이트
	if (leftKneeNPOffsetFlag == true) {
		// TODO : SDO SUIT -> SAM
		static bool oneTimeLKSendOffset = true;
		if (oneTimeLKSendOffset == true) {
			int16_t scaledOffset = ScaleFloatToInt16(obj->data.LeftShankNeutralPostureOffset, DEG_SCALING_FACTOR);
			SendNeutralPostureOffset(LK_SAGITAL, scaledOffset);
			oneTimeLKSendOffset = false;
		}

		UpdateNotiSignal();

        if (LK_Sagittal.notiSignals == 1) {
            LK_Sagittal.notiSignals = 0;
			userCtrlObj[LK_SAGITAL].devObj.noti = 0;
			updateNeutralPostureFlag = true;
			leftKneeNPOffsetFlag = false;
			oneTimeLKSendOffset = true;
			LK_NPOffsetCompleted = true;
        }
	}
}

static void SendHwSwVersion(void)
{
	DOPI_SDOUnit_t sdoUnit;

	uint8_t suit_hw_version = IOIF_SYS_HwRevGPIO();
	CM_sw_ver = FW_VER_MAJOR * 100 + FW_VER_MINOR * 10 + FW_VER_PATCH * 1;
	BLECtrlObj.data_manu.RH_MD_SW_ver = BLECtrlObj.data_manu.RH_MD_SW_ver_major*100 + BLECtrlObj.data_manu.RH_MD_SW_ver_minor*10 + BLECtrlObj.data_manu.RH_MD_SW_ver_patch*1 ;
	BLECtrlObj.data_manu.LH_MD_SW_ver = BLECtrlObj.data_manu.LH_MD_SW_ver_major*100 + BLECtrlObj.data_manu.LH_MD_SW_ver_minor*10 + BLECtrlObj.data_manu.LH_MD_SW_ver_patch*1 ;

	BLECtrlObj.data.VersionData[0] = (uint16_t) suit_hw_version;
	BLECtrlObj.data.VersionData[1] = (uint16_t) BLECtrlObj.data_manu.RH_MD_HW_ver;
	BLECtrlObj.data.VersionData[2] = (uint16_t) BLECtrlObj.data_manu.LH_MD_HW_ver;

	BLECtrlObj.data.VersionData[3] = FW_VER_MAJOR;
	BLECtrlObj.data.VersionData[4] = FW_VER_MINOR;
	BLECtrlObj.data.VersionData[5] = FW_VER_PATCH;

	BLECtrlObj.data.VersionData[6] = BLECtrlObj.data_manu.RH_MD_SW_ver_major;
	BLECtrlObj.data.VersionData[7] = BLECtrlObj.data_manu.RH_MD_SW_ver_minor;
	BLECtrlObj.data.VersionData[8] = BLECtrlObj.data_manu.RH_MD_SW_ver_patch;

	BLECtrlObj.data.VersionData[9] = BLECtrlObj.data_manu.LH_MD_SW_ver_major;		
	BLECtrlObj.data.VersionData[10] = BLECtrlObj.data_manu.LH_MD_SW_ver_minor;
	BLECtrlObj.data.VersionData[11] = BLECtrlObj.data_manu.LH_MD_SW_ver_patch;

	BLECtrlObj.data.VersionData[12] = 0; // test 목적 추후 대체 예정
	BLECtrlObj.data.VersionData[13] = 0;
	BLECtrlObj.data.VersionData[14] = 0;
	
	BLECtrlObj.data.VersionData[15] = FSM_File.file_version_major;
	BLECtrlObj.data.VersionData[16] = FSM_File.file_version_minor;
	BLECtrlObj.data.VersionData[17] = FSM_File.file_version_patch;
	BLECtrlObj.data.VersionData[18] = FW_VER_DEBUG; // debug test

	sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_HW_VERSION_CONFIRMED, SDO_REQU, 19);

	BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
}

static void SendMotionAnalysisFinishAble(void)
{
	DOPI_SDOUnit_t sdoUnit;

	BLECtrlObj.data.MotionAnalysisFinishAble = (uint8_t) MotionAnalysisFinishAble;
	sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_FINISH_ABLE, SDO_REQU, 1);
	BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
}

static void SendMotionAnalysisDataState(void)
{
	if (MotionAnalysisDataState == 0 && onetime_finish_set){
		BLECtrlObj.data.MotionAnalysisDataState = 0;

		DOPI_SDOUnit_t sdoUnit;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA_STATE, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
		motion_analysis_cnt = 0;
		report_data_cnt = 0;
	}

	else if (MotionAnalysisDataState == 1 && motion_analysis_cnt >= 5){
		BLECtrlObj.data.MotionAnalysisDataState = 1;

		DOPI_SDOUnit_t sdoUnit;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA_STATE, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

		MotionAnalysisDataState = 2;
		motion_analysis_cnt = 0;
		report_data_cnt = 0;
	}

	else if (MotionAnalysisDataState == 2 && motion_analysis_cnt >= 5){
		BLECtrlObj.data.MotionAnalysisDataState = 2;

		DOPI_SDOUnit_t sdoUnit;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA_STATE, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

		MotionAnalysisDataState = 255;
		ReportDataTransferState = SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA;
		motion_analysis_cnt = 0;
		report_data_cnt = 0;
		report_data_sw = 1;
	}

	else if (MotionAnalysisDataState == 3 && motion_analysis_cnt >= 5){
		BLECtrlObj.data.MotionAnalysisDataState = 3;

		DOPI_SDOUnit_t sdoUnit;
		sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA_STATE, SDO_REQU, 1);
		BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

		MotionAnalysisDataState = 255;
		motion_analysis_cnt = 0;
		report_data_cnt = 0;
	}

	motion_analysis_cnt++;
}

static void SendMotionAnalysisReportData(void)
{
	static uint8_t data_size = 0;

	if (report_data_sw == 1){
		BLECtrlObj.data.MotionAnalysisReportId = (uint16_t) ReportId;
		BLECtrlObj.data.MotionAnalysisMeanWalkSpeed = (uint16_t) 100 * meanVelX;
		BLECtrlObj.data.MotionAnalysisStdWalkSpeed = (uint16_t) 100 * stdVelX;
		BLECtrlObj.data.MotionAnalysisMeanWalkCountPerMin = (uint16_t) 10 * meanCadence;
		BLECtrlObj.data.MotionAnalysisStdWalkCountPerMin = (uint16_t) 10 * stdCadence;
//		BLECtrlObj.data.MotionAnalysisMeanStrideLength = (uint16_t) 100 * meanStrideLengthBoth;
		BLECtrlObj.data.MotionAnalysisMeanStrideLength = (uint16_t) 100 * meanStepLengthBoth * 2;
		BLECtrlObj.data.MotionAnalysisStdStrideLength = (uint16_t) 100 * stdStrideLengthBoth;
		BLECtrlObj.data.MotionAnalysisBilateralMeanStepLength = (uint16_t) 100 * meanStepLengthBoth;
		BLECtrlObj.data.MotionAnalysisMeanLeftStepLength = (uint16_t) 100 * meanStepLengthLeft;
		BLECtrlObj.data.MotionAnalysisMeanRightStepLength = (uint16_t) 100 * meanStepLengthRight;
		BLECtrlObj.data.MotionAnalysisAsymmetryStepLength = (uint16_t) 10 * asymmetryStepLength;
		BLECtrlObj.data.MotionAnalysisMeanLeftSwingPhase = (uint16_t) 10 * leftSwingPhase;
		BLECtrlObj.data.MotionAnalysisMeanRightSwingPhase = (uint16_t) 10 * rightSwingPhase;
		BLECtrlObj.data.MotionAnalysisAsymmetrySwingPhase = (uint16_t) 10 * asymmetrySwingPhase;
		BLECtrlObj.data.MotionAnalysisMeanLeftStancePhase = (uint16_t) 10 * leftStancePhase;
		BLECtrlObj.data.MotionAnalysisMeanRightStancePhase = (uint16_t) 10 * rightStancePhase;
		BLECtrlObj.data.MotionAnalysisAsymmetryStancePhase = (uint16_t) 10 * asymmetryStancePhase;
		BLECtrlObj.data.MotionAnalysisMeanLeftHipRoM = (uint16_t) 1 * leftHipROM;
		BLECtrlObj.data.MotionAnalysisMeanLeftHipMax = (uint16_t) 1 * MotionAnalysisMeanLeftHipMax;
		BLECtrlObj.data.MotionAnalysisMeanLeftHipMin = (uint16_t) 1 * MotionAnalysisMeanLeftHipMin;
		BLECtrlObj.data.MotionAnalysisMeanRightHipRoM = (uint16_t) 1 * rightHipROM;
		BLECtrlObj.data.MotionAnalysisMeanRightHipMax = (uint16_t) 1 * MotionAnalysisMeanRightHipMax;
		BLECtrlObj.data.MotionAnalysisMeanRightHipMin = (uint16_t) 1 * MotionAnalysisMeanRightHipMin;
		BLECtrlObj.data.MotionAnalysisAsymmetryRoM = (uint16_t) 10 * asymmetryHipAngle;
		BLECtrlObj.data.MotionAnalysisRMSLeftHipDiff = (uint16_t) 10 * MotionAnalysisRMSLeftHipDiff;
		BLECtrlObj.data.MotionAnalysisRMSRightHipDiff = (uint16_t) 10 * MotionAnalysisRMSRightHipDiff;

		memcpy(BLECtrlObj.data.ThighAngleLeftX,			GaitCycle, 				sizeof(BLECtrlObj.data.ThighAngleLeftX));
		memcpy(BLECtrlObj.data.ThighAngleLeftYMean,		ThighAngleLeftYMean,	sizeof(BLECtrlObj.data.ThighAngleLeftYMean));
		memcpy(BLECtrlObj.data.ThighAngleLeftYStd,		ThighAngleLeftYStd,		sizeof(BLECtrlObj.data.ThighAngleLeftYStd));
		memcpy(BLECtrlObj.data.ThighAngleRightX,		GaitCycle,				sizeof(BLECtrlObj.data.ThighAngleRightX));
		memcpy(BLECtrlObj.data.ThighAngleRightYMean,	ThighAngleRightYMean,	sizeof(BLECtrlObj.data.ThighAngleRightYMean));
		memcpy(BLECtrlObj.data.ThighAngleRightYStd,		ThighAngleRightYStd,	sizeof(BLECtrlObj.data.ThighAngleRightYStd));
		memcpy(BLECtrlObj.data.ThighTorqueLeftX,		GaitCycle,				sizeof(BLECtrlObj.data.ThighTorqueLeftX));
		memcpy(BLECtrlObj.data.ThighTorqueLeftYMean,	ThighTorqueLeftYMean,	sizeof(BLECtrlObj.data.ThighTorqueLeftYMean));
		memcpy(BLECtrlObj.data.ThighTorqueLeftYStd,		ThighTorqueLeftYStd,	sizeof(BLECtrlObj.data.ThighTorqueLeftYStd));
		memcpy(BLECtrlObj.data.ThighTorqueRightX,		GaitCycle, 				sizeof(BLECtrlObj.data.ThighTorqueRightX));
		memcpy(BLECtrlObj.data.ThighTorqueRightYMean,	ThighTorqueRightYMean,	sizeof(BLECtrlObj.data.ThighTorqueRightYMean));
		memcpy(BLECtrlObj.data.ThighTorqueRightYStd,	ThighTorqueRightYStd,	sizeof(BLECtrlObj.data.ThighTorqueRightYStd));

		report_data_sw = 0;
	}

	for (uint8_t i = SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA; i <= SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_RIGHT_Y_STD; i++){
		if (i == ReportDataTransferState && report_data_cnt >= 5){

			if (ReportDataTransferState == SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA){
				data_size = 26;
			} else {
				data_size = 101;
			}

			DOPI_SDOUnit_t sdoUnit;
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, ReportDataTransferState, SDO_REQU, data_size);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

			if (ReportDataTransferState >= SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA && ReportDataTransferState < SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_RIGHT_Y_STD){
				ReportDataTransferState++;
			} else if (ReportDataTransferState == SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_RIGHT_Y_STD){
				ReportDataTransferState = 255;
				MotionAnalysisDataState = 3;
			}

			motion_analysis_cnt = 0;
			report_data_cnt = 0;
		}
	}
	report_data_cnt++;
}

static int BT_Run_Send_PDO()
{
    uint8_t t_byte_len = 0;

    int t_check = BT_Pack_PDO(BLE_sdo_msg.txBuf, &BLE_sdo_msg.msgLength);

    if (BLE_sdo_msg.msgLength != 0){
    	TxBLESDO();

		return t_check;
		//TODO: MSG TX ERROR

    }

	return t_check;
}

static int BT_Pack_PDO(uint8_t* t_byte_arr, uint8_t* t_byte_len)
{
	// check send list whether these are empty or not
    if (BT_pdo_send_list == NULL){
        return 0;
    }
    t_byte_arr[0] = BT_PDO;

    int t_cursor = 1;

    // Pub PDO
    int t_numOfPDO_cursor = t_cursor;
    t_cursor += DOP_OBJ_NUMS_SIZE;

    uint8_t t_numOfPDO = 0;

    if (BT_pdo_send_list != NULL) {
        for(int i = 0; i < cvector_size(BT_pdo_send_list); ++i) {

            int temp_cursor = BT_Convert_PDO_to_Bytes(&BT_pdo_send_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
                ++t_numOfPDO;
            } else if (temp_cursor < 0) {
                //TODO: Pack PDO Error
                return temp_cursor;
            }
        }
    }

    // Set # of PDOs
    memcpy(&t_byte_arr[t_numOfPDO_cursor], &t_numOfPDO, DOP_OBJ_NUMS_SIZE);

    *t_byte_len = t_cursor;

    return DOP_SUCCESS;
}

static int BT_Convert_PDO_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr)
{
//    int t_header_size = sizeof(DOP_Header_t);
	 int t_header_size = 1;
    // Publish PDO
    DOP_PDO_t* t_pdo = BT_FindPDO(t_header->dictID, t_header->objID);
    if (t_pdo == NULL) {
        //TODO: Cannot Find PDO
        return -2;
    }

    uint16_t t_n_bytes = BT_SetPDO(t_pdo, t_byte_arr + t_header_size);
    if (t_n_bytes < 0) {
        //TODO: Copy PDO to Send
        return -1;
    } else if (t_n_bytes == 0) { // Nothing to publish
        return 0;
    }

    memcpy(t_byte_arr, &t_header->objID, t_header_size);
    return t_header_size + t_n_bytes;
}

DOP_SDO_t* BT_FindSDO(uint8_t t_dictID, uint16_t t_objID)
{
   	return &BT_DODs[t_dictID].SDOs[t_objID];
}

DOP_PDO_t* BT_FindPDO(uint8_t t_dictID, uint16_t t_objID)
{
	return &BT_DODs[t_dictID].PDOs[t_objID];
}

uint16_t BT_SetPDO(DOP_PDO_t* t_PDO, void* t_data)
{
    memcpy(t_data, t_PDO->addr, t_PDO->objSize);
    memcpy(t_PDO->lastPub, t_PDO->addr, t_PDO->objSize);
    return t_PDO->objSize;
}

uint16_t BT_GetPDO(DOP_PDO_t* t_PDO, void* t_data)
{
    memcpy(t_PDO->addr, t_data, t_PDO->objSize);
    return t_PDO->objSize;
}

static int BT_AS_CB(AS_BLE_CtrlObj_t* obj)
{
	DOPI_SDOUnit_t sdoUnit;


	/* Revised by TJ */
	uint8_t recv_buff[244] = {0};			// 최대 버퍼 갯수, 변경 가능 (최대 한번에 처리할 데이터 수 설정, Max 244Bytes)

	uint32_t r_byte_len = IOIF_ESP32_BT_ReadCommData(recv_buff, 244);

    uint16_t fncCode = recv_buff[0];

	if(r_byte_len > 0)	// 수진된 데이터가 있을 경우 파싱 수행
	{
		if(recv_buff[0]==255 && recv_buff[1]==1 && recv_buff[2]==255)
		{
			Check_BLE_COMMU_RX(recv_buff);
		}
		else if(recv_buff[0]==2 && recv_buff[2]==1 && recv_buff[3]==0 && recv_buff[6]==255)
		{
			re_TxBLESDO();
			osDelay(20);
		}
		else if(recv_buff[0]==2 && recv_buff[2]==2 && recv_buff[3]==0 && recv_buff[6]==255)
		{
			re_TxBLESDO();
			osDelay(20);
		}
		else
		{
			if(Checksum_Check(recv_buff, r_byte_len-2, (uint16_t*) &recv_buff[r_byte_len-2]))
			{
			    switch(fncCode) {
			        case BT_SDO: BT_UnpackSDO(&obj->devObj, &recv_buff[1]); break;
//			        case BT_PDO: DOPI_UnpackPDO(&obj->devObj, &recv_buff[1]); break;
			        default: break;
			    }
			}
			else
			{
				if(obj->data_manu.test == 1){
					DOPI_ClearSDO(&BLE_sdo_msg);
					DOPI_BLE_ADD_TASK_ID(&BLE_sdo_msg, BT_SDO);

					obj->data_manu.test = CMD_CHECKSUM_ERROR;
					sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_TEST, SDO_REQU, 1);
					BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
					obj->data_manu.test = 1;

					TxBLESDO();
					osDelay(20);
				}else{
					DOPI_ClearSDO(&BLE_sdo_msg);
					DOPI_BLE_ADD_TASK_ID(&BLE_sdo_msg, BT_SDO);

					obj->data.msgCommand = CMD_CHECKSUM_ERROR;
					sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_FUNCTION, SDO_REQU, 1);
					BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);
					obj->data.msgCommand = 0;

					TxBLESDO();
					osDelay(20);
				}

			}
		}
	}


    return 0;
}
int BT_UnpackSDO(DOPI_DevObj_t* t_obj, uint8_t* t_byteArr)
{
    int t_cursor = 0;

    // Get # of SDOs
    uint8_t t_numOfSDO = 0;
    memcpy(&t_numOfSDO, &t_byteArr[t_cursor++], 1);

    // Call & Respond SDOs
    if (t_numOfSDO > 0) {
        for (int i = 0; i < t_numOfSDO; ++i) {
            int t_tempCursor = BT_ReadSDO(t_obj, &t_byteArr[t_cursor]);
            if (t_tempCursor > 0) {
                t_cursor += t_tempCursor;
            } else if (t_tempCursor < 0) {
                //TODO: Unpack SDO ERROR
                return DOP_STATUS_SDO_RECV_FAIL;
            }
        }
    }
    return DOP_STATUS_SUCCESS;
}

static int BT_ReadSDO(DOPI_DevObj_t* t_obj, uint8_t* t_byte_arr)
{
    int t_byte_read = 0;
    DOP_Header_t t_header = GetHeader(t_byte_arr);
    t_byte_read += sizeof(DOP_Header_t);
    DOP_SDO_t* t_sdo = BT_FindSDO(t_header.dictID, t_header.objID);
    if (t_sdo == NULL) {
        //TODO: Cannot Find SDO ERROR
        return -2;
    }

    if(manu_mode == 1){
    	BLECtrlObj.data_manu.msgCTM[Success_id_cnt] = t_header.objID;
    } else {
    	BLECtrlObj.data.msgCTM[Success_id_cnt] = t_header.objID;
    }
    Success_id_cnt++;

    uint16_t t_req_bytes = 0;
    DOP_SDOArgs_t t_req = Bytes2SDOreq(t_byte_arr + t_byte_read, &t_req_bytes);
    t_req.typeSize = t_sdo->args.typeSize; // Copy SDO info
    t_byte_read += t_req_bytes;
    t_req.nodeID = t_obj->nodeID;

    uint16_t t_n_bytes = 0;
    if (t_req.status == DOP_SDO_REQU) {
    	t_n_bytes = BT_CallSDO(t_sdo, &t_req);
//        cvector_push_back(sdo_res_list, t_header); // Assign Response
    } else if(t_req.status == DOP_SDO_SUCC || t_req.status == DOP_SDO_FAIL) {
    	t_n_bytes = BT_SetSDOArgs(t_sdo, &t_req);
        if (t_n_bytes < 0) {
            //TODO: Set SDO Argument ERROR
            return -1;
        }
    } else {
        //TODO: Read SDO Status ERROR
        return -1;
    }

    t_byte_read += t_n_bytes;
    return t_byte_read;
}

uint16_t BT_CallSDO(DOP_SDO_t* t_SDO, DOP_SDOArgs_t* t_req)
{
    if (t_SDO->args.data != NULL){
        free(t_SDO->args.data);
        t_SDO->args.data = NULL;
    }

    t_SDO->args.status = DOP_SDO_IDLE;
    // t_req->typeSize = DOP_GetDataTypeInfo(t_SDO->dataType).typeSize;
    if(t_SDO->callback){
    	t_SDO->callback(t_req, &t_SDO->args);
    }

    return t_req->dataSize * t_SDO->args.typeSize;
}

void BT_SetSDOReq(uint8_t t_dictID, uint16_t t_objID, void* t_data, uint8_t t_size)
{
    DOP_SDOArgs_t t_req;

    DOP_SDO_t* t_SDO = BT_FindSDO(t_dictID, t_objID);
    if (t_SDO == NULL) {
        return;
    }

    t_req.status = DOP_SDO_REQU;
    t_req.data = t_data;
    t_req.dataSize = t_size;
    BT_SetSDOArgs(t_SDO, &t_req);
}

uint16_t BT_SetSDOArgs(DOP_SDO_t* t_SDO, DOP_SDOArgs_t* t_args)
{
    // Copy status
	t_SDO->args.status = t_args->status;

    // Copy size
    int t_totalSize = t_SDO->args.typeSize * t_args->dataSize;
    t_SDO->args.dataSize = t_args->dataSize;
    if (t_totalSize <= 0) {
        return 0;
    }

    // Copy data
    if (t_SDO->args.data != NULL) {
        free(t_SDO->args.data);
        t_SDO->args.data = NULL;
    }
    t_SDO->args.data = malloc(t_totalSize);
    memcpy(t_SDO->args.data, t_args->data, t_totalSize);

    return t_totalSize;
}

static void BT_CreateDOD(uint8_t t_dictID)
{
	BT_DODs[t_dictID].dictID = t_dictID;
}

static void BT_CreateSDO(uint8_t t_dictID, uint16_t t_objID, DOP_DataType_t t_type, DOP_SDOCB_t t_callback)
{
	BT_DODs[t_dictID].SDOs[t_objID].objID          = t_objID;
	BT_DODs[t_dictID].SDOs[t_objID].dataType       = t_type;
	BT_DODs[t_dictID].SDOs[t_objID].callback       = t_callback;
	BT_DODs[t_dictID].SDOs[t_objID].args.status    = DOP_SDO_IDLE;
	BT_DODs[t_dictID].SDOs[t_objID].args.dataSize  = 0;
	BT_DODs[t_dictID].SDOs[t_objID].args.data      = NULL;
	BT_DODs[t_dictID].SDOs[t_objID].args.typeSize  = DOP_GetDataTypeInfo(t_type).typeSize;
}

static void BT_CreatePDO(uint8_t t_dictID, uint16_t t_objID, DOP_DataType_t t_type, uint8_t t_size, void* t_addr)
{
	BT_DODs[t_dictID].PDOs[t_objID].objID          = t_objID;
	BT_DODs[t_dictID].PDOs[t_objID].dataType       = t_type;
	BT_DODs[t_dictID].PDOs[t_objID].dataSize       = t_size;
	BT_DODs[t_dictID].PDOs[t_objID].addr           = t_addr;
	BT_DODs[t_dictID].PDOs[t_objID].objSize       	= DOP_GetDataTypeInfo(t_type).typeSize * t_size;
	BT_DODs[t_dictID].PDOs[t_objID].lastPub        = malloc(BT_DODs[t_dictID].PDOs[t_objID].objSize);

    memset(BT_DODs[t_dictID].PDOs[t_objID].lastPub, 0xFF, BT_DODs[t_dictID].PDOs[t_objID].objSize);
}

DOP_SDO_t* BT_DOP_FindSDO(uint8_t t_dictID, uint16_t t_objID)
{
   	return &BT_DODs[t_dictID].SDOs[t_objID];
}

DOP_PDO_t* BT_DOP_FindPDO(uint8_t t_dictID, uint16_t t_objID)
{
	return &BT_DODs[t_dictID].PDOs[t_objID];
}


static void  BT_Add_PDO_to_Send(uint8_t t_dictID, uint8_t t_objID)
{
	DOP_PDO_t* temp_pdo = DOP_FindPDO(t_dictID, t_objID);
    if (temp_pdo == NULL) {
        //TODO: Cannot Find PDO Error
        return;
    }

    DOP_Header_t t_pdo = {t_dictID, t_objID};

    for (int i = 0; i < cvector_size(BT_pdo_send_list); ++i) {
        if ((BT_pdo_send_list[i].dictID == t_dictID) && (BT_pdo_send_list[i].objID == t_objID)){
            return;
        }
    }
    cvector_push_back(BT_pdo_send_list, t_pdo);
}

static void BT_Clear_PDO_to_Send()
{
    cvector_free(BT_pdo_send_list);
    BT_pdo_send_list = NULL;
}

static void BT_Set_Send_PDO_List(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	BT_Clear_PDO_to_Send();
	uint8_t t_dictID = 0 ;
    int t_cursor = 0;
    uint8_t* t_ids = (uint8_t*)t_req->data;
    while (t_cursor < t_req->dataSize) {
    	if(manu_mode == 1){
        		t_dictID = 2;
        	} else {
        		t_dictID = 0;
        	}

        uint8_t t_objID = t_ids[t_cursor++];
        BT_Add_PDO_to_Send(t_dictID, t_objID);
    }

    t_res->status = DOP_SDO_SUCC;
}

static void BT_ADD_TASK_ID( DOPI_SDOMsg_t* t_SDOMsg, uint8_t fnc_code)
{


	t_SDOMsg->txBuf[0] = fnc_code;
//	t_SDOMsg->txBuf[2] = task_id;
	t_SDOMsg->msgLength = 2;

}

static void BT_AppendSDO(DOPI_SDOUnit_t* t_SDOUnit, DOPI_SDOMsg_t* t_SDOMsg)
{
	uint8_t t_cursor;

	/* Validation Check */
	if(BT_CheckSDO(t_SDOUnit->taskID, t_SDOUnit->SDOID) != 0)	return;

	/* Appending SDO and Packing */
	/* 0th txbuf is for a number of SDO */
	if(t_SDOMsg->msgLength == 0) {t_cursor = 1;}
	else						 {t_cursor = t_SDOMsg->msgLength;}

	t_SDOMsg->txBuf[t_cursor++] = t_SDOUnit->taskID;
	t_SDOMsg->txBuf[t_cursor++] = t_SDOUnit->SDOID;
	t_SDOMsg->txBuf[t_cursor++] = t_SDOUnit->param.SDOStatus;
	t_SDOMsg->txBuf[t_cursor++] = t_SDOUnit->param.numOfData;

	SDOInfo_t dataSize = BT_GetSDOInfo(t_SDOUnit->taskID, t_SDOUnit->SDOID);
	uint8_t t_size = t_SDOUnit->param.numOfData*dataSize;

	memcpy(&t_SDOMsg->txBuf[t_cursor], t_SDOUnit->param.data, t_size);
	t_cursor += t_size;

	t_SDOMsg->numOfSDO++;
	t_SDOMsg->txBuf[1] = t_SDOMsg->numOfSDO;
	t_SDOMsg->msgLength = t_cursor;

}

static int Check_BLE_COMMU_TX(void)
{
    DOPI_ClearSDO(&BLE_sdo_msg);

    BLE_sdo_msg.txBuf[0] = 255;// BT_TASK_ID_MANU_CM_to_Mobile
    BLE_sdo_msg.txBuf[1] = 0; // num of data
    BLE_sdo_msg.txBuf[2] = 255;//

    for(int j = 3; j < 62; j++)
    {
    	BLE_sdo_msg.txBuf[j]=j-2;
    }

    BLE_sdo_msg.msgLength=62;

    TxBLESDO();


    return 0;
}

static int Check_BLE_COMMU_RX(uint8_t* t_byte_arr)
{
    uint8_t t_checker = 0;


    uint8_t t_res;


 	DOPI_ClearSDO(&BLE_sdo_msg);

    for(int i = 3; i<62; i++)
    {
    	if(t_byte_arr[i]==62 - i)
    	{
    		t_checker++;
    	}
    }

    t_res = Checksum_Check(t_byte_arr, 62, (uint16_t*) &t_byte_arr[62]);

    if(t_res==1)
    {
    	ble_check_status = 1;
    }

    return 0;
}

static int BT_CheckSDO(uint8_t t_taskID, uint16_t t_SDOID)
{
	/* Check Error */
	switch(t_taskID){

    case BT_TASK_ID_ANGELA_CM_to_Mobile:
		if(t_SDOID >= SDO_ID_ANGELA_CTM_NUM)	return DOP_STATUS_SDO_SET_FAIL;
		break;
    case BT_TASK_ID_ANGELA_Mobile_to_CM:
		if(t_SDOID >= SDO_ID_SYSMNGT_CM_NUM)	return DOP_STATUS_SDO_SET_FAIL;
		break;
    case BT_TASK_ID_MANU_CM_to_Mobile:
		if(t_SDOID >= SDO_ID_MANU_CTM_NUM)	return DOP_STATUS_SDO_SET_FAIL;
		break;
    case BT_TASK_ID_MANU_Mobile_to_CM:
		if(t_SDOID >= SDO_ID_MANU_MTC_NUM)	return DOP_STATUS_SDO_SET_FAIL;
		break;
    case BT_TASK_ID_ESP32_CMD:
		if(t_SDOID >= SDO_ID_ESP32_CMD_NUM)	return DOP_STATUS_SDO_SET_FAIL;
		break;
	default:
		return DOP_STATUS_SDO_SET_FAIL;
		break;
	}

	return DOP_STATUS_SUCCESS;
}

static SDOInfo_t BT_GetSDOInfo(uint8_t t_taskID, uint16_t t_SDOID)
{
	return DOP_ConvertDataSize(BT_SDOTable[t_taskID][t_SDOID]);
}

void FLASH_ALL_WRITE(void)
{
	uint32_t writeAddr = IOIF_FLASH_SECTOR_6_BANK2_ADDR;
	uint32_t memArr1[8] = {0,};
	uint32_t memArr2[8] = {0,};
	uint32_t memArr3[8] = {0,};
	uint32_t memArr4[8] = {0,};
	uint32_t memArr5[8] = {0,};
	uint32_t memArr6[8] = {0,};
	uint32_t memArr7[8] = {0,};
	uint32_t memArr8[8] = {0,};

	IOIF_EraseFlash(IOIF_FLASH_SECTOR_6_BANK2_ADDR, IOIF_ERASE_ONE_SECTOR);

	memcpy(&memArr1[0], BLECtrlObj.data_manu.Robot_model,			sizeof(BLECtrlObj.data_manu.Robot_model));
	memcpy(&memArr1[3], BLECtrlObj.data_manu.Serial_number_front,	sizeof(BLECtrlObj.data_manu.Serial_number_front));
	IOIF_WriteFlash(writeAddr, memArr1);
	writeAddr += 32;

	memcpy(&memArr2[0], BLECtrlObj.data_manu.Serial_number_middle,	sizeof(BLECtrlObj.data_manu.Serial_number_middle));
	memcpy(&memArr2[3], BLECtrlObj.data_manu.Serial_number_back,	sizeof(BLECtrlObj.data_manu.Serial_number_back));
	memcpy(&memArr2[6], &BLECtrlObj.data_manu.RTC_year,				sizeof(BLECtrlObj.data_manu.RTC_year));
	memcpy(&memArr2[7], &BLECtrlObj.data_manu.RTC_month,			sizeof(BLECtrlObj.data_manu.RTC_month));
	IOIF_WriteFlash(writeAddr, memArr2);
	writeAddr += 32;

	memcpy(&memArr3[0], &BLECtrlObj.data_manu.RTC_day,			sizeof(BLECtrlObj.data_manu.RTC_day));
	memcpy(&memArr3[1], &BLECtrlObj.data_manu.RTC_hour,			sizeof(BLECtrlObj.data_manu.RTC_hour));
	memcpy(&memArr3[2], &BLECtrlObj.data_manu.RTC_minute,		sizeof(BLECtrlObj.data_manu.RTC_minute));
	memcpy(&memArr3[3], &BLECtrlObj.data_manu.Inspector_name,	sizeof(BLECtrlObj.data_manu.Inspector_name));
	memcpy(&memArr3[4], &BLECtrlObj.data_manu.BT_MAC_add_front,	sizeof(BLECtrlObj.data_manu.BT_MAC_add_front));

	IOIF_WriteFlash(writeAddr, memArr3);
	writeAddr += 32;

	memcpy(&memArr4[0], &BLECtrlObj.data_manu.BT_MAC_add_back,	sizeof(BLECtrlObj.data_manu.BT_MAC_add_back));
	memcpy(&memArr4[3], BLECtrlObj.data_manu.BT_name_front,		sizeof(BLECtrlObj.data_manu.BT_name_front));
	IOIF_WriteFlash(writeAddr, memArr4);
	writeAddr += 32;

	memcpy(&memArr5[0], BLECtrlObj.data_manu.BT_name_back,	sizeof(BLECtrlObj.data_manu.BT_name_back));
	memcpy(&memArr5[3], &BLECtrlObj.data_manu.BT_baudrate,	sizeof(BLECtrlObj.data_manu.BT_baudrate));
	memcpy(&memArr5[4], &BLECtrlObj.data_manu.BT_set_first,	sizeof(BLECtrlObj.data_manu.BT_set_first));
	IOIF_WriteFlash(writeAddr, memArr5);
	writeAddr += 32;

	memcpy(&memArr6[0], BLECtrlObj.data_manu.RH_Serial_number_front,	sizeof(BLECtrlObj.data_manu.RH_Serial_number_front));
	memcpy(&memArr6[3], BLECtrlObj.data_manu.RH_Serial_number_middle,	sizeof(BLECtrlObj.data_manu.RH_Serial_number_middle));
	IOIF_WriteFlash(writeAddr, memArr6);
	writeAddr += 32;

	memcpy(&memArr7[0], BLECtrlObj.data_manu.RH_Serial_number_back,		sizeof(BLECtrlObj.data_manu.RH_Serial_number_back));
	memcpy(&memArr7[3], BLECtrlObj.data_manu.LH_Serial_number_front,	sizeof(BLECtrlObj.data_manu.LH_Serial_number_front));
	IOIF_WriteFlash(writeAddr, memArr7);
	writeAddr += 32;

	memcpy(&memArr8[0], BLECtrlObj.data_manu.LH_Serial_number_middle,	sizeof(BLECtrlObj.data_manu.LH_Serial_number_middle));
	memcpy(&memArr8[3], BLECtrlObj.data_manu.LH_Serial_number_back,		sizeof(BLECtrlObj.data_manu.LH_Serial_number_back));
	IOIF_WriteFlash(writeAddr, memArr8);
	writeAddr += 32;
}

void FLASH_ALL_DOWNLOAD(void)
{
	uint32_t readAddr = IOIF_FLASH_SECTOR_6_BANK2_ADDR;
	uint32_t t_add=0;

	t_add = readAddr;

	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.Robot_model,			sizeof(BLECtrlObj.data_manu.Robot_model));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.Serial_number_front,	sizeof(BLECtrlObj.data_manu.Serial_number_front));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

	readAddr = t_add +32;

	for(int i = 0; i<500; ++i) {}

	t_add = readAddr;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.Serial_number_middle,	sizeof(BLECtrlObj.data_manu.Serial_number_middle));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.Serial_number_back,	sizeof(BLECtrlObj.data_manu.Serial_number_back));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data_manu.RTC_year,	sizeof(BLECtrlObj.data_manu.RTC_year));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data_manu.RTC_month,	sizeof(BLECtrlObj.data_manu.RTC_month));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	readAddr = t_add +32;

	for(int i = 0; i<500; ++i) {}

	t_add = readAddr;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data_manu.RTC_day,			sizeof(BLECtrlObj.data_manu.RTC_day));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data_manu.RTC_hour,		sizeof(BLECtrlObj.data_manu.RTC_hour));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data_manu.RTC_minute,		sizeof(BLECtrlObj.data_manu.RTC_minute));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data_manu.Inspector_name,	sizeof(BLECtrlObj.data_manu.Inspector_name));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.BT_MAC_add_front,	sizeof(BLECtrlObj.data_manu.BT_MAC_add_front));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

	readAddr = t_add +32;

	for(int i = 0; i<500; ++i) {}

	t_add = readAddr;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.BT_MAC_add_back,	sizeof(BLECtrlObj.data_manu.BT_MAC_add_back));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.BT_name_front,	sizeof(BLECtrlObj.data_manu.BT_name_front));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

	readAddr = t_add +32;

	for(int i = 0; i<500; ++i) {}

	t_add = readAddr;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.BT_name_back,		sizeof(BLECtrlObj.data_manu.BT_name_back));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data_manu.BT_baudrate,		sizeof(BLECtrlObj.data_manu.BT_baudrate));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data_manu.BT_set_first,	sizeof(BLECtrlObj.data_manu.BT_set_first));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	readAddr = t_add +32;

	for(int i = 0; i<500; ++i) {}

	t_add = readAddr;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.RH_Serial_number_front,	sizeof(BLECtrlObj.data_manu.RH_Serial_number_front));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.RH_Serial_number_middle,	sizeof(BLECtrlObj.data_manu.RH_Serial_number_middle));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

	readAddr = t_add +32;

	for(int i = 0; i<500; ++i) {}

	t_add = readAddr;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.RH_Serial_number_back,	sizeof(BLECtrlObj.data_manu.RH_Serial_number_back));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.LH_Serial_number_front,	sizeof(BLECtrlObj.data_manu.LH_Serial_number_front));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

	readAddr = t_add +32;

	for(int i = 0; i<500; ++i) {}

	t_add = readAddr;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.LH_Serial_number_middle,	sizeof(BLECtrlObj.data_manu.LH_Serial_number_middle));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, BLECtrlObj.data_manu.LH_Serial_number_back,	sizeof(BLECtrlObj.data_manu.LH_Serial_number_back));
	readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

	readAddr = t_add +32;
}

void SaveBLESetData_FLASH(void)
{
	uint32_t writeAddr = IOIF_FLASH_SECTOR_7_BANK2_ADDR;
	uint32_t memArr1[8] = {0,};
	uint32_t memArr2[8] = {0,};
	uint32_t memArr3[8] = {0,};
	uint32_t memArr4[8] = {0,};
	uint32_t memArr5[8] = {0,};

	IOIF_EraseFlash(IOIF_FLASH_SECTOR_7_BANK2_ADDR, IOIF_ERASE_ONE_SECTOR);

	memcpy(&memArr1[0], &SoundSettingSavedFlag,				sizeof(SoundSettingSavedFlag));			// uint8
	memcpy(&memArr1[1], &BLECtrlObj.data.SoundLanguage,		sizeof(BLECtrlObj.data.SoundLanguage));	// uint8
	memcpy(&memArr1[2], &BLECtrlObj.data.SoundLevel,		sizeof(BLECtrlObj.data.SoundLevel));	// uint8
	memcpy(&memArr1[3], &BLECtrlObj.data.SoundGender,		sizeof(BLECtrlObj.data.SoundGender));	// uint8
	memcpy(&memArr1[4], &BLECtrlObj.data.RightThighNeutralSagittalAngle,	sizeof(BLECtrlObj.data.RightThighNeutralSagittalAngle));	// float
	memcpy(&memArr1[5], &BLECtrlObj.data.LeftThighNeutralSagittalAngle,		sizeof(BLECtrlObj.data.LeftThighNeutralSagittalAngle));		// float
	memcpy(&memArr1[6], &BLECtrlObj.data.RightShankNeutralSagittalAngle,	sizeof(BLECtrlObj.data.RightShankNeutralSagittalAngle));	// float
	memcpy(&memArr1[7], &BLECtrlObj.data.LeftShankNeutralSagittalAngle,		sizeof(BLECtrlObj.data.LeftShankNeutralSagittalAngle));		// float
	IOIF_WriteFlash(writeAddr, memArr1);
	writeAddr += 32;

	memcpy(&memArr2[0], &FvectorSavedFlag,				sizeof(FvectorSavedFlag));			// uint8
	memcpy(&memArr2[1], &MobileUuidFront,				sizeof(MobileUuidFront));
	memcpy(&memArr2[4], &MobileUuidBack,				sizeof(MobileUuidBack));
	memcpy(&memArr2[7], &TrunkNeutralPostureBias,		sizeof(TrunkNeutralPostureBias));	// uint8
	IOIF_WriteFlash(writeAddr, memArr2);
	writeAddr += 32;

	memcpy(&memArr3[0], &RightHipFlexionTorque,			sizeof(RightHipFlexionTorque));		// int16
	memcpy(&memArr3[1], &RightHipExtensionTorque,		sizeof(RightHipExtensionTorque));		// int16
	memcpy(&memArr3[2], &LeftHipFlexionTorque,			sizeof(LeftHipFlexionTorque));			// int16
	memcpy(&memArr3[3], &LeftHipExtensionTorque,		sizeof(LeftHipExtensionTorque));		// int16
	memcpy(&memArr3[4], &RightHipSmartAssistGravityCompensation,	sizeof(RightHipSmartAssistGravityCompensation));	// int16
	memcpy(&memArr3[5], &LeftHipSmartAssistGravityCompensation,		sizeof(LeftHipSmartAssistGravityCompensation));	// int16
	IOIF_WriteFlash(writeAddr, memArr3);
	writeAddr += 32;

	memcpy(&memArr4[0], &RightKneeFlexionTorque,		sizeof(RightKneeFlexionTorque));		// int16
	memcpy(&memArr4[1], &RightKneeExtensionTorque,		sizeof(RightKneeExtensionTorque));		// int16
	memcpy(&memArr4[2], &LeftKneeFlexionTorque,			sizeof(LeftKneeFlexionTorque));			// int16
	memcpy(&memArr4[3], &LeftKneeExtensionTorque,		sizeof(LeftKneeExtensionTorque));		// int16
	memcpy(&memArr4[4], &RightKneeSmartAssistGravityCompensation,	sizeof(RightKneeSmartAssistGravityCompensation));	// int16
	memcpy(&memArr4[5], &LeftKneeSmartAssistGravityCompensation,	sizeof(LeftKneeSmartAssistGravityCompensation));	// int16
	IOIF_WriteFlash(writeAddr, memArr4);
	writeAddr += 32;

	memcpy(&memArr5[0], &Org_Code,		sizeof(Org_Code));	// uint8
	IOIF_WriteFlash(writeAddr, memArr5);
	writeAddr += 32;

	FLASHSaveFlag = IDLE_FLASH_BLE_DATA;
}

void DownloadBLESetData_FLASH(void)
{
	uint32_t readAddr = IOIF_FLASH_SECTOR_7_BANK2_ADDR;
	uint32_t t_add = 0;


	t_add = readAddr;

	IOIF_ReadFlash(readAddr, &SoundSettingSavedFlag,			sizeof(SoundSettingSavedFlag));			readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data.SoundLanguage,	sizeof(BLECtrlObj.data.SoundLanguage));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data.SoundLevel,		sizeof(BLECtrlObj.data.SoundLevel));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data.SoundGender,		sizeof(BLECtrlObj.data.SoundGender));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data.RightThighNeutralSagittalAngle,	sizeof(BLECtrlObj.data.RightThighNeutralSagittalAngle));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data.LeftThighNeutralSagittalAngle,	sizeof(BLECtrlObj.data.LeftThighNeutralSagittalAngle));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data.RightShankNeutralSagittalAngle,	sizeof(BLECtrlObj.data.RightShankNeutralSagittalAngle));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &BLECtrlObj.data.LeftShankNeutralSagittalAngle,	sizeof(BLECtrlObj.data.LeftShankNeutralSagittalAngle));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	readAddr = t_add + 32;

	for(int i = 0; i < 500; ++i) {}



	t_add = readAddr;

	IOIF_ReadFlash(readAddr, &FvectorSavedFlag,			sizeof(FvectorSavedFlag));			readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &MobileUuidFront,			sizeof(MobileUuidFront));			readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, &MobileUuidBack,			sizeof(MobileUuidBack));			readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, &TrunkNeutralPostureBias,	sizeof(TrunkNeutralPostureBias));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	readAddr = t_add + 32;

	for(int i = 0; i < 500; ++i) {}



	t_add = readAddr;

	IOIF_ReadFlash(readAddr, &RightHipFlexionTorque,		sizeof(RightHipFlexionTorque));			readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &RightHipExtensionTorque,		sizeof(RightHipExtensionTorque));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &LeftHipFlexionTorque,			sizeof(LeftHipFlexionTorque));			readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &LeftHipExtensionTorque,		sizeof(LeftHipExtensionTorque));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &RightHipSmartAssistGravityCompensation,		sizeof(RightHipSmartAssistGravityCompensation));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &LeftHipSmartAssistGravityCompensation,		sizeof(LeftHipSmartAssistGravityCompensation));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	readAddr = t_add + 32;

	for(int i = 0; i < 500; ++i) {}



	t_add = readAddr;

	IOIF_ReadFlash(readAddr, &RightKneeFlexionTorque,			sizeof(RightKneeFlexionTorque));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &RightKneeExtensionTorque,			sizeof(RightKneeExtensionTorque));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &LeftKneeFlexionTorque,			sizeof(LeftKneeFlexionTorque));			readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &LeftKneeExtensionTorque,			sizeof(LeftKneeExtensionTorque));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &RightKneeSmartAssistGravityCompensation,		sizeof(RightKneeSmartAssistGravityCompensation));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr, &LeftKneeSmartAssistGravityCompensation,		sizeof(LeftKneeSmartAssistGravityCompensation));	readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

	readAddr = t_add + 32;

	for(int i = 0; i < 500; ++i) {}



	t_add = readAddr;

	IOIF_ReadFlash(readAddr, &Org_Code,			sizeof(Org_Code));		readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;

	readAddr = t_add + 32;

	// Sound Setting Not Saved in FLASH
	if (SoundSettingSavedFlag == 0 || SoundSettingSavedFlag == 255) {
		/* default */
		BLECtrlObj.data.SoundLanguage = AUDIO_LANGUAGE_KOREAN;	// 언어 설정 0 : 한국어(default), 1: 영어
		BLECtrlObj.data.SoundLevel = 100; 						// default volume
		BLECtrlObj.data.SoundGender = AUDIO_GENDER_FEMALE;		// 언어 성별 0 : 남성, 1 : 여성(default)
	}

	audioLanguage = BLECtrlObj.data.SoundLanguage;
	audioVolume = BLECtrlObj.data.SoundLevel;

	// F vector Not Saved in FLASH
	if (FvectorSavedFlag == 0 || FvectorSavedFlag == 255) {
		/* default */
		RightHipExtensionTorque		= 1000;
		LeftHipExtensionTorque		= 1000;
		RightHipFlexionTorque		= 1000;
		LeftHipFlexionTorque		= 1000;
		RightKneeExtensionTorque	= 1000;
		LeftKneeExtensionTorque		= 1000;
		RightKneeFlexionTorque		= 1000;
		LeftKneeFlexionTorque		= 1000;
		RightHipSmartAssistGravityCompensation = 3.0;
		LeftHipSmartAssistGravityCompensation = 3.0;
	}

	// ASCII 0~127로 제한
	for (int i = 0; i < 10; i++) MobileUuidFront[i] = (MobileUuidFront[i] > 127) ? 0 : MobileUuidFront[i];
	for (int i = 0; i < 10; i++) MobileUuidBack[i] = (MobileUuidBack[i] > 127) ? 0 : MobileUuidBack[i];
	for (int i = 0; i < 10; i++) Org_Code[i] = (Org_Code[i] > 127) ? 0 : Org_Code[i];


	FLASHSaveFlag = IDLE_FLASH_BLE_DATA;
}


int Check_FD_CAN_COMMU_TX(uint8_t MD_nodeID)
{

    uint8_t t_dest_node = MD_nodeID;
    uint8_t t_tx_data[64];
    uint16_t t_identifier = FDCAN_CHECK|(NODE_ID_CM<<4)|t_dest_node;
    uint16_t t_checksum;

    t_tx_data[0] = 4;// BT_TASK_ID_MANU_CM_to_Mobile
    t_tx_data[1] = 1; // num of data
    t_tx_data[2] = 255;//

    for(int j = 3; j<62; j++)
    {
    	t_tx_data[j]=j - 2;
    }

	t_checksum = Checksum_Calcul(t_tx_data, 62);
	memcpy(t_tx_data + 62, &t_checksum, 2);
	if(MD_nodeID==NODE_ID_RH_SAG){
		IOIF_TransmitFDCAN2(t_identifier, t_tx_data, 64);
	}else{
		IOIF_TransmitFDCAN1(t_identifier, t_tx_data, 64);
	}


    return 0;
}

int Check_FD_CAN_COMMU_RX(uint8_t* t_byte_arr, uint8_t MD_nodeID)
{
    uint8_t t_checker = 0;
    uint8_t t_res;

    for(int i = 3; i<62; i++)
    {
    	if(t_byte_arr[i]==62-i)
    	{
    		t_checker++;
    	}
    }

    t_res = Checksum_Check(t_byte_arr, 62, (uint16_t*) &t_byte_arr[62]);

    if(t_checker == 59 && t_res == 1)
    {
    	can_test[MD_nodeID] = 1;
    }

    return 0;
}


uint16_t Checksum_Calcul(uint8_t* t_test_packet, uint8_t t_length )
{
	uint16_t t_sum = 0;
	uint16_t t_result = 0;

	for(int i = 0; i<t_length; i++)
	{
		t_sum+=t_test_packet[i];
	}

	t_result=~t_sum+1;

	return t_result;
}

uint8_t Checksum_Check(uint8_t* t_test_packet, uint8_t t_length, uint16_t * t_checksum)
{
	uint16_t t_sum = 0;
	uint16_t t_result = 0;

	for(int i = 0; i<t_length; i++)
	{
		t_sum+=t_test_packet[i];
	}

	t_result = *t_checksum + t_sum;

	if(t_result==0){
		return 1;
	}
	else
	{
		return 0;
	}
}


// Base64 인코딩 함수
void base64_encode(const uint8_t *data, size_t input_length, size_t *output_length) {
    *output_length = 4 * ((input_length + 2) / 3);
//    char *encoded = malloc(*output_length + 1);
//    if (!encoded) return NULL;
    char encoded[25] = {0};

    for (size_t i = 0, j = 0; i < input_length;) {
        uint32_t octet_a = i < input_length ? data[i++] : 0;
        uint32_t octet_b = i < input_length ? data[i++] : 0;
        uint32_t octet_c = i < input_length ? data[i++] : 0;

        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        encoded[j++] = base64_table[(triple >> 18) & 0x3F];
        encoded[j++] = base64_table[(triple >> 12) & 0x3F];
        encoded[j++] = base64_table[(triple >> 6) & 0x3F];
        encoded[j++] = base64_table[triple & 0x3F];
    }

    // Add padding '='
    for (size_t i = 0; i < (3 - input_length % 3) % 3; i++)
        encoded[*output_length - 1 - i] = '=';

    encoded[*output_length] = '\0';

    for(uint8_t i =0;i<*output_length;i++){
    	base64_res[i] = encoded[i];
    }
//    free(encoded);
}


/**
 * @brief Set the address of SDO/PDO which you want to use
 * @param obj Object
 */


static void Set_Serial_1(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if(t_req->nodeID==6)
	{
		memcpy(BLECtrlObj.data_manu.RH_Serial_number_front, t_req->data, 10);
	}
	else if(t_req->nodeID==7)
	{
		memcpy(BLECtrlObj.data_manu.LH_Serial_number_front, t_req->data, 10);
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Serial_2(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if(t_req->nodeID==6)
	{
		memcpy(BLECtrlObj.data_manu.RH_Serial_number_middle, t_req->data, 10);
	}
	else if(t_req->nodeID==7)
	{
		memcpy(BLECtrlObj.data_manu.LH_Serial_number_middle, t_req->data, 10);
	}


	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Serial_3(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if(t_req->nodeID==6)
	{
		memcpy(BLECtrlObj.data_manu.RH_Serial_number_back, t_req->data, 10);
	}
	else if(t_req->nodeID==7)
	{
		memcpy(BLECtrlObj.data_manu.LH_Serial_number_back, t_req->data, 10);
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}
static void Set_HW_ver(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if(t_req->nodeID==6)
	{
		memcpy(&BLECtrlObj.data_manu.RH_MD_HW_ver, t_req->data, 1);
	}
	else if(t_req->nodeID==7)
	{
		memcpy(&BLECtrlObj.data_manu.LH_MD_HW_ver, t_req->data, 1);
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_SW_ver_major(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if(t_req->nodeID==6)
	{
		memcpy(&BLECtrlObj.data_manu.RH_MD_SW_ver_major, t_req->data, 1);
	}
	else if(t_req->nodeID==7)
	{
		memcpy(&BLECtrlObj.data_manu.LH_MD_SW_ver_major, t_req->data, 1);
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_SW_ver_minor(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if(t_req->nodeID==6)
	{
		memcpy(&BLECtrlObj.data_manu.RH_MD_SW_ver_minor, t_req->data, 1);
	}
	else if(t_req->nodeID==7)
	{
		memcpy(&BLECtrlObj.data_manu.LH_MD_SW_ver_minor, t_req->data, 1);
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_SW_ver_patch(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if(t_req->nodeID==6)
	{
		memcpy(&BLECtrlObj.data_manu.RH_MD_SW_ver_patch, t_req->data, 1);
	}
	else if(t_req->nodeID==7)
	{
		memcpy(&BLECtrlObj.data_manu.LH_MD_SW_ver_patch, t_req->data, 1);
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_SW_ver_debug(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if(t_req->nodeID==6)
	{
		memcpy(&BLECtrlObj.data_manu.RH_MD_SW_ver_debug, t_req->data, 2);
	}
	else if(t_req->nodeID==7)
	{
		memcpy(&BLECtrlObj.data_manu.LH_MD_SW_ver_debug, t_req->data, 2);
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Receive_decrypted_plain_txt(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.Decrypted_plain_txt, t_req->data, 10);
	
	aes_res = 1;

	for(int i=0; i<6; i++){
		if(BLECtrlObj.data.Decrypted_plain_txt[i]!=plain_text[i]){
			aes_res = 0;
		}
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Mobile_UUID(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.MobileUuidFront, t_req->data, 20);
	memcpy(MobileUuidFront, BLECtrlObj.data.MobileUuidFront, 10);
	memcpy(MobileUuidBack, BLECtrlObj.data.MobileUuidBack, 10);

	set_uuid_flag = 1;
	FLASHSaveFlag = SAVE_FLASH_BLE_DATA;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_test(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLE_test4, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}


static void Set_Function(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(BLECtrlObj.data.msgMTC, t_req->data, t_req->dataSize);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Receive_Success_MTC(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(BLECtrlObj.data.msgMTC, t_req->data, t_req->dataSize);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_User_Data(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.initializeDate, (uint8_t *)t_req->data, 24);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static float checkValue(float value, float min, float max, float defaultValue) {
    if (value < min || value > max) {
        return defaultValue;
    }
    return value;
}

static void Set_User_Body_Data(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.User_Weight, (uint8_t *)t_req->data, 32);

	User_Weight = checkValue((float)BLECtrlObj.data.User_Weight / 1000, 20.0f, 230.0f, DEFAULT_WEIGHT); 							// g -> kg
	User_Height = checkValue((float)BLECtrlObj.data.User_Height / 1000, 1.0f, 2.5f, DEFAULT_HEIGHT);    							// mm -> m
	User_Right_Thigh_Length = checkValue((float)BLECtrlObj.data.User_Right_Thigh_Length / 1000, 0.1f, 1.0f, DEFAULT_THIGH_LENGTH);	// mm -> m
	User_Left_Thigh_Length = checkValue((float)BLECtrlObj.data.User_Left_Thigh_Length / 1000, 0.1f, 1.0f, DEFAULT_THIGH_LENGTH);	// mm -> m
	User_Right_Shin_Length = checkValue((float)BLECtrlObj.data.User_Right_Shin_Length / 1000, 0.1f, 1.0f, DEFAULT_SHIN_LENGTH);		// mm -> m
	User_Left_Shin_Length = checkValue((float)BLECtrlObj.data.User_Left_Shin_Length / 1000, 0.1f, 1.0f, DEFAULT_SHIN_LENGTH);		// mm -> m
	User_Right_Ankle_Length = checkValue((float)BLECtrlObj.data.User_Right_Ankle_Length / 1000, 0.01f, 0.5f, DEFAULT_ANKLE_LENGTH);	// mm -> m
	User_Left_Ankle_Length = checkValue((float)BLECtrlObj.data.User_Left_Ankle_Length / 1000, 0.01f, 0.5f, DEFAULT_ANKLE_LENGTH);	// mm -> m

	TrunkLength_Improved = User_Height - (User_Right_Thigh_Length + User_Left_Thigh_Length)/2
			- (User_Right_Shin_Length + User_Left_Shin_Length)/2 - (User_Right_Ankle_Length + User_Left_Ankle_Length)/2;
	ThighLength_Improved = (User_Right_Thigh_Length + User_Left_Thigh_Length)/2;
	AnkleLength_Improved = (User_Right_Ankle_Length + User_Left_Ankle_Length)/2;
	ShankLength_Improved = (User_Right_Shin_Length + User_Left_Shin_Length)/2 + AnkleLength_Improved; 	// 종아리 길이에 발목 길이 추가하기로 수정

	TrunkMass_Improved = User_Weight * TrunkLength_Improved/User_Height;		// 골반 무게 포함 (GaitAnalysis.c에서 사용하지 않으므로 큰 상관 없음)
	ThighMass_Improved = User_Weight * ThighLength_Improved/User_Height / 2;
	ShankMass_Improved = User_Weight * ShankLength_Improved/User_Height / 2;
	AnkleMass_Improved = User_Weight * AnkleLength_Improved/User_Height / 2;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Sound_Setting(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	uint8_t* t_addr = (uint8_t *)t_req->data;

	memcpy(&BLECtrlObj.data.SoundLanguage, (uint8_t *)t_addr, 1);
	memcpy(&BLECtrlObj.data.SoundLevel, (uint8_t *)t_addr + 1, 1);
	memcpy(&BLECtrlObj.data.SoundGender, (uint8_t *)t_addr + 2, 1);
	memcpy(&BLECtrlObj.data.Soundplay, (uint8_t *)t_addr + 3, 1);

	audioLanguage = BLECtrlObj.data.SoundLanguage;		// audio language
	audioVolume = BLECtrlObj.data.SoundLevel;			// audioVolume update
	if (BLECtrlObj.data.Soundplay == AUDIO_PREVIEW_ON) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_16;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
		BLECtrlObj.data.Soundplay = AUDIO_PREVIEW_OFF;
	}

	SoundSettingSavedFlag = 1;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Neutral_Sagittal_Angle(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.LeftThighNeutralSagittalAngle, (uint8_t *)t_req->data, 16);

	for (int DevIdx = RH_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
		uint8_t tUsage = RS_File.vec[DevIdx].usage;
		static float UserNeutralPostureAngle;

		if (tUsage == 1) {
			if (DevIdx == RH_SAGITAL) {
				UserNeutralPostureAngle = BLECtrlObj.data.RightThighNeutralSagittalAngle;
			} else if (DevIdx == LH_SAGITAL) {
				UserNeutralPostureAngle = BLECtrlObj.data.LeftThighNeutralSagittalAngle;
			} else if (DevIdx == RK_SAGITAL) {
				UserNeutralPostureAngle = BLECtrlObj.data.RightShankNeutralSagittalAngle;
			} else if (DevIdx == LK_SAGITAL) {
				UserNeutralPostureAngle = BLECtrlObj.data.LeftShankNeutralSagittalAngle;
			}
			SendUserNeutralPostureAngle(DevIdx, UserNeutralPostureAngle);
		}
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Left_Hip_Neutral_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

		memcpy(&BLECtrlObj.data.LeftThighNeutralPostureOffset, t_req->data, 4);
        leftHipNPOffsetFlag = true;   // Offset Cal ON
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Right_Hip_Neutral_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

		memcpy(&BLECtrlObj.data.RightThighNeutralPostureOffset, t_req->data, 4);
        rightHipNPOffsetFlag = true;  // Offset Cal ON
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Left_Knee_Neutral_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

		memcpy(&BLECtrlObj.data.LeftShankNeutralPostureOffset, t_req->data, 4);
        leftKneeNPOffsetFlag = true;   // Offset Cal ON
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Right_Knee_Neutral_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

		memcpy(&BLECtrlObj.data.RightShankNeutralPostureOffset, t_req->data, 4);
        rightKneeNPOffsetFlag = true;  // Offset Cal ON
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Neutral_Posture_Calibration(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(&BLECtrlObj.data.NeutralPostureCalibration, t_req->data, 1);

	if(BLECtrlObj.data.NeutralPostureCalibration == 1)
	{
		neutralPosCalCMD = 1;
		oneTimeSendNeutralPosture = 1;
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Mode_Type(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(&BLECtrlObj.data.AssistMode, t_req->data, 1);
	assistmode = BLECtrlObj.data.AssistMode;
	if(assistmode == STS_MODE){
		STSMode = 1;
		DOP_SDOArgs_t req, res;
    
		req.data = &BLECtrlObj.data.SittoStandAssistLevel;
		req.dataSize = 1;
		Set_Sit_to_Stance_Torque(&req, &res);
		
		req.data = &BLECtrlObj.data.StandtoSitAssistLevel;
		req.dataSize = 1;
		Set_Stance_to_Sit_Torque(&req, &res);
		
	} else {
		STSMode = 0;
	}

	if (standbymode == 0){
		BFlag[31] = 1;
	}

	if (BLECtrlObj.data.STSPostureHold == 1 && PostureHold == 1) {
		BLECtrlObj.data.STSPostureHold = 0;
		PostureHold = 0;
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Smart_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	uint8_t* t_addr = (uint8_t *)t_req->data;

	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
		memcpy(&BLECtrlObj.data.LeftHipExtensionTorque,			t_addr, 1);
		memcpy(&BLECtrlObj.data.RightHipExtensionTorque,		t_addr + 1, 1);
		memcpy(&BLECtrlObj.data.LeftHipFlexionTorque,			t_addr + 2, 1);
		memcpy(&BLECtrlObj.data.RightHipFlexionTorque,			t_addr + 3, 1);
		memcpy(&BLECtrlObj.data.LeftHipGravityCompensation,		t_addr + 4, 1);
		memcpy(&BLECtrlObj.data.RightHipGravityCompensation,	t_addr + 5, 1);

		RightHipFlexionTorque = BLECtrlObj.data.RightHipFlexionTorque * 10;
		RightHipExtensionTorque = BLECtrlObj.data.RightHipExtensionTorque * 10;
		LeftHipFlexionTorque = BLECtrlObj.data.LeftHipFlexionTorque * 10;
		LeftHipExtensionTorque = BLECtrlObj.data.LeftHipExtensionTorque * 10;

		RightHipSmartAssistGravityCompensation = 3.0 / 10 * BLECtrlObj.data.RightHipGravityCompensation;
		LeftHipSmartAssistGravityCompensation = 3.0 / 10 * BLECtrlObj.data.LeftHipGravityCompensation;
		RightHipSmartAssistVelocityCompensation = 0.4;
		LeftHipSmartAssistVelocityCompensation = 0.4;
	}

	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 1 && RS_File.vec[LK_SAGITAL].usage == 1) {
		memcpy(&BLECtrlObj.data.LeftKneeExtensionTorque,		t_addr, 1);
		memcpy(&BLECtrlObj.data.RightKneeExtensionTorque,		t_addr + 1, 1);
		memcpy(&BLECtrlObj.data.LeftKneeFlexionTorque,			t_addr + 2, 1);
		memcpy(&BLECtrlObj.data.RightKneeFlexionTorque,			t_addr + 3, 1);
		memcpy(&BLECtrlObj.data.LeftKneeGravityCompensation,	t_addr + 4, 1);
		memcpy(&BLECtrlObj.data.RightKneeGravityCompensation,	t_addr + 5, 1);

		RightKneeFlexionTorque = BLECtrlObj.data.RightKneeFlexionTorque * 10;
		RightKneeExtensionTorque = BLECtrlObj.data.RightKneeExtensionTorque * 10;
		LeftKneeFlexionTorque = BLECtrlObj.data.LeftKneeFlexionTorque * 10;
		LeftKneeExtensionTorque = BLECtrlObj.data.LeftKneeExtensionTorque * 10;

		RightKneeSmartAssistGravityCompensation = 3.0 / 10 * BLECtrlObj.data.RightKneeGravityCompensation;
		LeftKneeSmartAssistGravityCompensation = 3.0 / 10 * BLECtrlObj.data.LeftKneeGravityCompensation;
		RightKneeSmartAssistVelocityCompensation = 0.4;
		LeftKneeSmartAssistVelocityCompensation = 0.4;
	}

	memcpy(&BLECtrlObj.data.DisableStairWalkingDetection,	t_addr + 6, 1);
	DisableStairWalkingDetection = (uint8_t)(BLECtrlObj.data.DisableStairWalkingDetection);

	/* 3. Save Flash */
	FLASHSaveFlag = SAVE_FLASH_BLE_DATA;
	FvectorSavedFlag = 1;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Aqua_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	/* 1. copy data */
	uint8_t* t_addr = (uint8_t *)t_req->data;
	memcpy(&BLECtrlObj.data.LeftHipGravityCompensation, 	t_addr, 1);
	memcpy(&BLECtrlObj.data.RightHipGravityCompensation, 	t_addr + 1, 1);
	memcpy(&BLECtrlObj.data.LeftHipVelocityCompensation, 	t_addr + 2, 1);
	memcpy(&BLECtrlObj.data.RightHipVelocityCompensation, 	t_addr + 3, 1);
	memcpy(&BLECtrlObj.data.LeftKneeGravityCompensation, 	t_addr + 4, 1);
	memcpy(&BLECtrlObj.data.RightKneeGravityCompensation, 	t_addr + 5, 1);
	memcpy(&BLECtrlObj.data.LeftKneeVelocityCompensation, 	t_addr + 6, 1);
	memcpy(&BLECtrlObj.data.RightKneeVelocityCompensation, 	t_addr + 7, 1);

	/* 2. Setting Params */
	/* Gravity, Velocity Compensation for Aqua Mode */
	RightHipAquaGravityCompensation = 1.5 * (BLECtrlObj.data.RightHipGravityCompensation * 0.1);
	LeftHipAquaGravityCompensation = 1.5 * (BLECtrlObj.data.LeftHipGravityCompensation * 0.1);
	RightHipAquaVelocityCompensation = -2 * (BLECtrlObj.data.RightHipVelocityCompensation * 0.1);
	LeftHipAquaVelocityCompensation = -2 * (BLECtrlObj.data.LeftHipVelocityCompensation * 0.1);

	RightKneeAquaGravityCompensation = 1.5 * (BLECtrlObj.data.RightKneeGravityCompensation * 0.1);
	LeftKneeAquaGravityCompensation = 1.5 * (BLECtrlObj.data.LeftKneeGravityCompensation * 0.1);
	RightKneeAquaVelocityCompensation = -2 * (BLECtrlObj.data.RightKneeVelocityCompensation * 0.1);
	LeftKneeAquaVelocityCompensation = -2 * (BLECtrlObj.data.LeftKneeVelocityCompensation * 0.1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Space_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	/* 1. copy data */
	uint8_t* t_addr = (uint8_t *)t_req->data;
	memcpy(&BLECtrlObj.data.LeftHipGravityCompensation, 	t_addr, 1);
	memcpy(&BLECtrlObj.data.RightHipGravityCompensation, 	t_addr + 1, 1);
	memcpy(&BLECtrlObj.data.LeftKneeGravityCompensation, 	t_addr + 2, 1);
	memcpy(&BLECtrlObj.data.RightKneeGravityCompensation, 	t_addr + 3, 1);

	/* 2. Setting Params */
	/* Gravity, Velocity Compensation for Space Mode */

	if(BLECtrlObj.data.RightHipGravityCompensation < 5 ){
		RightHipUnivGravityCompensation = - 6 * ((BLECtrlObj.data.RightHipGravityCompensation - 5) * 0.2);
		LeftHipUnivGravityCompensation = - 6 * ((BLECtrlObj.data.LeftHipGravityCompensation - 5) * 0.2);
		RightHipUnivVelocityCompensation = - 0.6 * ((BLECtrlObj.data.RightHipGravityCompensation - 5) * 0.2); // temp debug for app error need to debug
		LeftHipUnivVelocityCompensation = - 0.6 * ((BLECtrlObj.data.LeftHipGravityCompensation - 5) * 0.2);

		RightKneeUnivGravityCompensation = - 6 * ((BLECtrlObj.data.RightKneeGravityCompensation - 5) * 0.2);
		LeftKneeUnivGravityCompensation = - 6 * ((BLECtrlObj.data.LeftKneeGravityCompensation - 5) * 0.2);
		RightKneeUnivVelocityCompensation = - 0.6 * ((BLECtrlObj.data.RightKneeVelocityCompensation - 5) * 0.2);
		LeftKneeUnivVelocityCompensation = - 0.6 * ((BLECtrlObj.data.LeftKneeVelocityCompensation - 5) * 0.2);
	}else{
		RightHipUnivGravityCompensation = - 4 * ((BLECtrlObj.data.RightHipGravityCompensation - 5) * 0.2);
		LeftHipUnivGravityCompensation = - 4 * ((BLECtrlObj.data.LeftHipGravityCompensation - 5) * 0.2);
		RightHipUnivVelocityCompensation = - 0.4 * ((BLECtrlObj.data.RightHipGravityCompensation - 5) * 0.2); // temp debug for app error need to debug
		LeftHipUnivVelocityCompensation = - 0.4 * ((BLECtrlObj.data.LeftHipGravityCompensation - 5) * 0.2);

		RightKneeUnivGravityCompensation = - 4 * ((BLECtrlObj.data.RightKneeGravityCompensation - 5) * 0.2);
		LeftKneeUnivGravityCompensation = - 4 * ((BLECtrlObj.data.LeftKneeGravityCompensation - 5) * 0.2);
		RightKneeUnivVelocityCompensation = - 0.4 * ((BLECtrlObj.data.RightKneeVelocityCompensation - 5) * 0.2);
		LeftKneeUnivVelocityCompensation = - 0.4 * ((BLECtrlObj.data.LeftKneeVelocityCompensation - 5) * 0.2);
	}


	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Limit_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	/* 1. copy data */
	uint8_t* t_addr = (uint8_t *)t_req->data;
	memcpy(&BLECtrlObj.data.LeftHipFlexionLimit,	t_addr, 1);
	memcpy(&BLECtrlObj.data.RightHipFlexionLimit,	t_addr + 1, 1);
	memcpy(&BLECtrlObj.data.LeftHipExtensionLimit,	t_addr + 2, 1);
	memcpy(&BLECtrlObj.data.RightHipExtensionLimit,	t_addr + 3, 1);
	memcpy(&BLECtrlObj.data.LeftHipFlexionVelocityLimit,	t_addr + 4, 1);
	memcpy(&BLECtrlObj.data.RightHipFlexionVelocityLimit,	t_addr + 5, 1);
	memcpy(&BLECtrlObj.data.LeftHipExtensionVelocityLimit,	t_addr + 6, 1);
	memcpy(&BLECtrlObj.data.RightHipExtensionVelocityLimit,	t_addr + 7, 1);

	// memcpy(&BLECtrlObj.data.LeftKneeFlexionLimit,	t_addr + 6, 1);
	// memcpy(&BLECtrlObj.data.RightKneeFlexionLimit,	t_addr + 7, 1);
	// memcpy(&BLECtrlObj.data.LeftKneeExtensionLimit,	t_addr + 8, 1);
	// memcpy(&BLECtrlObj.data.RightKneeExtensionLimit,t_addr + 9, 1);
	// memcpy(&BLECtrlObj.data.LeftKneeVelocityLimit,	t_addr + 10, 1);
	// memcpy(&BLECtrlObj.data.RightKneeVelocityLimit,	t_addr + 11, 1);

	/* 2. Setting Parameter */
	// TODO : 추후 각도 통일 필요
	RightHipFlexionLimit = BLECtrlObj.data.RightHipFlexionLimit;
	RightHipExtensionLimit = BLECtrlObj.data.RightHipExtensionLimit;

	RightHipFlexionVelocityLimit = BLECtrlObj.data.RightHipFlexionVelocityLimit;
	RightHipExtensionVelocityLimit = BLECtrlObj.data.RightHipExtensionVelocityLimit;

	LeftHipFlexionLimit = BLECtrlObj.data.LeftHipFlexionLimit;
	LeftHipExtensionLimit = BLECtrlObj.data.LeftHipExtensionLimit;

	LeftHipFlexionVelocityLimit = BLECtrlObj.data.LeftHipFlexionVelocityLimit;
	LeftHipExtensionVelocityLimit = BLECtrlObj.data.LeftHipExtensionVelocityLimit;

	// RightKneeFlexionLimit = BLECtrlObj.data.RightKneeFlexionLimit - 15.0;
	// RightKneeExtensionLimit = BLECtrlObj.data.RightKneeExtensionLimit + 15.0;
	// RightKneeVelocityLimit = BLECtrlObj.data.RightKneeVelocityLimit;

	// LeftKneeFlexionLimit = BLECtrlObj.data.LeftKneeFlexionLimit - 15.0;
	// LeftKneeExtensionLimit = BLECtrlObj.data.LeftKneeExtensionLimit + 15.0;
	// LeftKneeVelocityLimit = BLECtrlObj.data.LeftKneeVelocityLimit;


	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Sit_Stand_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

//		memcpy(&BLECtrlObj.data.SoundLanguage, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Training_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(&BLECtrlObj.data.TrainingSessionID, t_req->data, 8);
	memset(trainingInfo, 0, sizeof(trainingInfo));

	/* Standby mode 자동 해제 */
	// BLE로부터 Standby Mode 값을 가져옴 (0 or 1)
//	if (BLECtrlObj.data.StandbyMode != prevStandbyModefromApp) {		// Standby Mode가 앱에서 변경한 값인지 확인
//		prevStandbyModefromApp = BLECtrlObj.data.StandbyMode;			// 이전 앱 Standby Mode에 현재 Standby Mode 적용
//		BFlag[18] = 1;
//	}
	if(standbymode == 1)
	{
		standbymode = 0;
		BLECtrlObj.data.StandbyMode = standbymode;
		BFlag[18] = 1;
	}

	RHamplitude = 0;
	LHamplitude = 0;
	StrideCount_Session = 0;
	StrideCountLeft_Session = 0;
	StrideCountRight_Session = 0;
	posXRaw_Session[1] = 0;
	posX_Session[1] = 0;
	AsymmetryAvgSum = 0;
	AsymmetryAvgCnt = 1;
	totalCalories = 0;

	sitCount = 0;
	standCount = 0;
	sitTime =0;
	standTime=0;
	accumulateSitTime=0;
	accumulateStandTime=0;

	/* 세션 시간 데이터 카운트 시작 */
	trainingInfo[current_session].isSessionActive = true;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Training_Mode_Change(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(&BLECtrlObj.data.TrainingSessionID, t_req->data, 8);

	memset(trainingInfo[current_session].mode_duration, 0, sizeof(trainingInfo[current_session].mode_duration));
	trainingInfo[current_session].operation_duration = 0;
	trainingInfo[current_session].countFor1sec_standbyState = 0;
	memset(trainingInfo[current_session].walk_count, 0, sizeof(trainingInfo[current_session].walk_count));
	memset(trainingInfo[current_session].walk_distance, 0, sizeof(trainingInfo[current_session].walk_distance));
	memset(trainingInfo[current_session].asymmetric_level_avg, 0, sizeof(trainingInfo[current_session].asymmetric_level_avg));
	memset(trainingInfo[current_session].step_length_avg, 0, sizeof(trainingInfo[current_session].step_length_avg));

	RHamplitude = 0;
	LHamplitude = 0;
	StrideCount_Session = 0;
	StrideCountLeft_Session = 0;
	StrideCountRight_Session = 0;
	posXRaw_Session[1] = 0;
	posX_Session[1] = 0;
	AsymmetryAvgSum = 0;
	AsymmetryAvgCnt = 1;
	totalCalories = 0;

	sitCount = 0;
	standCount = 0;
	sitTime =0;
	standTime=0;
	accumulateSitTime=0;
	accumulateStandTime=0;

	LeftHipMaxAngle = 0;
	LeftHipMinAngle = 0;
	LeftHipMaxVelocity = 0;
	LeftHipMinVelocity = 0;
	RightHipMaxAngle = 0;
	RightHipMinAngle = 0;
	RightHipMaxVelocity = 0;
	RightHipMinVelocity = 0;

	/* Mode 변경 후, Mode 관련 시간 정보를 초기화 하는 코드를 추가 필요 */

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Training_Finish(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(&BLECtrlObj.data.TrainingSessionID, t_req->data, 8);
	memset(trainingInfo, 0, sizeof(trainingInfo));
	remaining_time = -1;
	alram_time = -2;
	RHamplitude = 0;
	LHamplitude = 0;
	StrideCount_Session = 0;
	StrideCountLeft_Session = 0;
	StrideCountRight_Session = 0;
	posXRaw_Session[1] = 0;
	posX_Session[1] = 0;
	AsymmetryAvgSum = 0;
	AsymmetryAvgCnt = 1;
	totalCalories = 0;

	sitCount = 0;
	standCount = 0;
	sitTime =0;
	standTime=0;
	accumulateSitTime=0;
	accumulateStandTime=0;

	LeftHipMaxAngle = 0;
	LeftHipMinAngle = 0;
	LeftHipMaxVelocity = 0;
	LeftHipMinVelocity = 0;
	RightHipMaxAngle = 0;
	RightHipMinAngle = 0;
	RightHipMaxVelocity = 0;
	RightHipMinVelocity = 0;

	if(standbymode != 1)
	{
		standbymode = 1;
		BLECtrlObj.data.StandbyMode = standbymode;
		BFlag[31] = 1;
	}

	/* 세션 시간 데이터 카운트 종료 */
	trainingInfo[current_session].isSessionActive = false;

	/* 세선 회차 정보 업데이트 : 이 기능은 넣을 것인지 말지 고민.. */
//	if(current_session > SESSION_COUNT)	current_session = 0;
//	else								current_session++;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Standby_Mode(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if((BLECtrlObj.data.STSPostureHold != 1 && PostureHold != 1 && !(FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 51))
		|| (FSMMngrObj.state_curr == 66) || (FSMMngrObj.state_curr == 71)){
		memcpy(&BLECtrlObj.data.StandbyMode, t_req->data, 1);
		standbymode = BLECtrlObj.data.StandbyMode;

		if (FSMMngrObj.state_curr == 1 && standbymode == 0){
			BFlag[18] = 1;
		}
		else if ((FSMMngrObj.state_curr > 1 && FSMMngrObj.state_curr < 18)
		|| (FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45)
		|| (FSMMngrObj.state_curr >= 65 && FSMMngrObj.state_curr <= 75)) {
			BFlag[31] = 1;
		}

		if(standbymode == 0 && assistmode == STS_MODE){
			STSMode = 1;
		} else {
			STSMode = 0;
			PostureHold = 0;
			BLECtrlObj.data.STSPostureHold = 0;
		}
	}
	else{

		if((FSMMngrObj.state_curr >= 67 && FSMMngrObj.state_curr <= 70) || (FSMMngrObj.state_curr >= 72 && FSMMngrObj.state_curr <= 75))
				errorMSGcode = 2;			// STS state 시 대기모드 진입 불가 Toast Msg.
		else if (FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 51)
				errorMSGcode = 8;			// 대기<->보조 전환 음성 출력 중, 전환 불가
		else
				errorMSGcode = 1;			// 그외 다른 모드에서 대기모드 진입 불가 Toast Msg.
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Assist_Level(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.AssistanceLevel, t_req->data, 1);

	// BLE로부터 Assist Level 값을 가져옴 (0~10)
	if (BLECtrlObj.data.AssistanceLevel != prevAssistLvfromApp) {	// 보조력 단계가 이전 앱에서 변경한 보조력과 다른 지 확인
		prevAssistLvfromApp = BLECtrlObj.data.AssistanceLevel;	// 이전 앱 보조력 단계에 현재 보조력 단계 적용
		currAssistLvfromBt = BLECtrlObj.data.AssistanceLevel;		// 물리 버튼에도 현재 보조력 단계 적용
		prevAssistLvfromBt = BLECtrlObj.data.AssistanceLevel;		// 물리 버튼에도 현재 보조력 단계 적용
		assistStage = BLECtrlObj.data.AssistanceLevel;			// 실제 보조력 단계 적용
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_STS_Hold(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(&BLECtrlObj.data.STSPostureHold, t_req->data, 1);
	PostureHold = BLECtrlObj.data.STSPostureHold;
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}
static void Set_STS_Ready(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(&BLECtrlObj.data.STSPostureReady, t_req->data, 1);
	if(BLECtrlObj.data.STSPostureReady == 0){
		SitReadyButton = 0;
		StandReadyButton = 0;
		StartSittingButton = 0;
		StartStandingButton = 0;
	} else if(BLECtrlObj.data.STSPostureReady == 1){
		SitReadyButton = 0;
		StandReadyButton = 1;
		StartSittingButton = 0;
		StartStandingButton = 0;
	} else if(BLECtrlObj.data.STSPostureReady == 2){
		SitReadyButton = 0;
		StandReadyButton = 0;
		StartSittingButton = 0;
		StartStandingButton = 1;
	} else if(BLECtrlObj.data.STSPostureReady == 3){
		SitReadyButton = 1;
		StandReadyButton = 0;
		StartSittingButton = 0;
		StartStandingButton = 0;
	} else if(BLECtrlObj.data.STSPostureReady == 4){
		SitReadyButton = 0;
		StandReadyButton = 0;
		StartSittingButton = 1;
		StartStandingButton = 0;
	}
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}
static void Set_Sit_to_Stance_Torque(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.SittoStandAssistLevel, t_req->data, 1);

	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	SittoStandAssistLevel = (BLECtrlObj.data.SittoStandAssistLevel < 0) ? 0 : ((BLECtrlObj.data.SittoStandAssistLevel > 100) ? 100 : BLECtrlObj.data.SittoStandAssistLevel);
	SitToStance_Torque = SittoStandAssistLevel * 0.08;		

	userCtrlObj[2].data.SitToStance_Torque = SitToStance_Torque;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_SITTOSTANCE_TOQUE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(2);

	DOPI_ClearSDO(&sdo_msg);
	userCtrlObj[3].data.SitToStance_Torque = SitToStance_Torque;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_SITTOSTANCE_TOQUE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(3);
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}
static void Set_Stance_to_Sit_Torque(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.StandtoSitAssistLevel, t_req->data, 1);

	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	StandtoSitAssistLevel = (BLECtrlObj.data.StandtoSitAssistLevel < 0) ? 0 : ((BLECtrlObj.data.StandtoSitAssistLevel > 100) ? 100 : BLECtrlObj.data.StandtoSitAssistLevel);
	StanceToSit_Torque = StandtoSitAssistLevel * 0.1 * 0.5;		//(Max 5Nm)

	userCtrlObj[2].data.StanceToSit_Torque = StanceToSit_Torque;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STANCETOSIT_TOQUE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(2);

	DOPI_ClearSDO(&sdo_msg);
	userCtrlObj[3].data.StanceToSit_Torque = StanceToSit_Torque;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STANCETOSIT_TOQUE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(3);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Motion_Analysis_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.ReportId, t_req->data, 12);
	ReportId = BLECtrlObj.data.ReportId;
	TrackUsage = BLECtrlObj.data.TrackUsage;
	MotionAnalysisType = BLECtrlObj.data.MotionAnalysisType;
	isMotionAnalysisRunning = true;
	isMotionAnalysisStart = true;
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Motion_Analysis_Cancel(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.ReportId, t_req->data, 4);
	ReportId = 0;
	isMotionAnalysisRunning = false;
	disablePDOSending = false;
	isMotionAnalysisCancel = true;
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Motion_Analysis_Finish(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	if (!onetime_finish_set) {
		memcpy(&BLECtrlObj.data.ReportId, t_req->data, 4);
		ReportId = BLECtrlObj.data.ReportId;
		isMotionAnalysisFinish = true;
		disablePDOSending = true;
		t_res->dataSize = 0;
		t_res->status = DOP_SDO_SUCC;
	}
}

static void Motion_Analysis_Data_Receive_State(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.DataReceiveState, t_req->data, 1);
	DataReceiveState = BLECtrlObj.data.DataReceiveState;

	if (DataReceiveState == 1){
		ReportId = 0;
		isMotionAnalysisRunning = false;
		disablePDOSending = false;
	} else if (DataReceiveState == 0){
		MotionAnalysisDataState = 2;
		disablePDOSending = true;
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Check_Error_Msg_State(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.CheckErrorMsgAck, t_req->data, 1);

	if(BLECtrlObj.data.CheckErrorMsgAck == 1)	errorMSGcode = 0;			// Msg Code Clear

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void SPPB_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.SPPB_Start_Flag, t_req->data, 1);
	SPPB_mode = BLECtrlObj.data.SPPB_Start_Flag;
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void SPPB_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.SPPB_Stop_Flag, t_req->data, 1);
	SPPB_mode = 0;
	BLECtrlObj.data.SPPB_Start_Flag = 0;

	if(SPPB_mode == BLECtrlObj.data.SPPB_Stop_Flag && BLECtrlObj.data.SPPB_Stop_Flag == 4){
		BLECtrlObj.data.SPPBWalkSpeed = 0;
		BLECtrlObj.data.SPPBWalkDistance = 0;
		BLECtrlObj.data.SPPBWalkDuration = 0;
		SPPB_mode = 0;
	}else if(BLECtrlObj.data.SPPB_Stop_Flag == SPPB_mode){
		SPPB_mode = 0;
	}

	BLECtrlObj.data.SPPB_Stop_Flag = 0;
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void SPPB_Assist_OnOff(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data.SPPBAssistOnOff, t_req->data, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Timer_Set(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	int16_t temp = 0;
	memcpy(&temp, t_req->data, 2);
	remaining_time = temp;
	alram_time = -1;
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Timer_Reset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	remaining_time = -1;
	alram_time = -2;
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_MANU_BOOT_CHECK(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	DOPI_SDOUnit_t sdo_unit;

	memcpy(&BLECtrlObj.data_manu.test, t_req->data, 1);

	if(IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12) == IOIF_GPIO_PIN_RESET){
		BLECtrlObj.data_manu.IsConnectSuccess = 1;
		Is_ManuReady = true;
	}
	else
	{
		BLECtrlObj.data_manu.IsConnectSuccess = 2;
		Is_ManuReady = false;
	}

	DOPI_ClearSDO(&BLE_sdo_msg);
	BT_ADD_TASK_ID(&BLE_sdo_msg, BT_SDO);

	sdo_unit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BOOT_CHECK_ACK, SDO_REQU, 1);
	BT_AppendSDO(&sdo_unit, &BLE_sdo_msg);

	TxBLESDO();

	BLECtrlObj.data_manu.IsConnectSuccess = 0;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Send_MANU_Success(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(BLECtrlObj.data_manu.msgMTC, t_req->data, t_req->dataSize);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_MANU_name_set(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

	memcpy(BLECtrlObj.data_manu.deviceLine, t_req->data, 30);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_ESP32_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&BLECtrlObj.data_manu.ESP32_command, t_req->data, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_MANU_Start(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	manu_step = CMD_MANU_START;
	AllSetManuPDO();
	Check_BLE_COMMU_TX();
	one_time_sw = 1;
	ble_cnt = 0;
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[2].data.joint_limit_sw = 1;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(2);

	DOPI_ClearSDO(&sdo_msg);
	userCtrlObj[3].data.joint_limit_sw = 1;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(3);
}

static void Set_MANU_MES_Setting_uint8(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	memcpy(&BLECtrlObj.data_manu.RTC_year, t_req->data, 6);
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
	ble_cnt = 0;
}
static void Set_MANU_MES_Setting_string10(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	memcpy(BLECtrlObj.data_manu.Serial_number_front, t_req->data, 30);
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
	manu_step = CMD_MES_SETTING;
	ble_cnt = 0;
}

static void Set_MANU_MES_WIFI_Test(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	SPI_test_sw = 1;
	SPI_test_result = 0;

	memcpy(BLECtrlObj.data_manu.Wifi_SSID_front, t_req->data, 60);
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
	manu_step = CMD_WIFI_TEST;
	ble_cnt = 0;
}

static void Set_MANU_SD_Card_check(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	uint8_t test_sd_buf[9] = {0,};
	uint8_t test_string[] = "testsdcard";
	/* Open Create File And Write Data */
	if (IOIF_FileWrite(&fileObj, (uint8_t *)"manu_test_file.txt", test_string, sizeof(test_string), &ble_fatfs_write_test_byte) == FR_OK)
	{
		/* Read Data from file */
		if (IOIF_FileRead(&fileObj, (uint8_t *)"manu_test_file.txt", test_sd_buf, sizeof(test_sd_buf), &ble_fatfs_read_test_byte) != FR_OK)
			Is_ManuSDCardState = false;
	}
	else
		Is_ManuSDCardState = false;

	/* Check */
	if (strcmp((const char *)test_string, (const char *)test_sd_buf) == 0) // 두개의 결과가 같지 않으면
		Is_ManuSDCardState = false;

	/* if true, Delete and finish*/
	if (IOIF_FileDelete((uint8_t *)"manu_test_file.txt") != FR_OK)
		Is_ManuSDCardState = false;

	manu_step = CMD_SD_CARD_TEST;
	ble_cnt = 0;
}
static void Set_MANU_FD_CAN_check(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	can_test[NODE_ID_RH_SAG] = 0;
	can_test[NODE_ID_LH_SAG] = 0;

	Check_FD_CAN_COMMU_TX(NODE_ID_RH_SAG);
	Check_FD_CAN_COMMU_TX(NODE_ID_LH_SAG);
	manu_step = CMD_FDCAN_TEST;
	ble_cnt = 0;
}
static void Set_MANU_HW_Version_check(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	SetVersionSDO(2);
	SetVersionSDO(3);
	one_time_sw = 1;
	manu_step = CMD_HW_VERSION_CHECK;
	ble_cnt = 0;
}
static void Set_MANU_SW_Version_check(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	SetVersionSDO(2);
	SetVersionSDO(3);
	one_time_sw = 1;
	manu_step = CMD_SW_VERSION_CHECK;
	ble_cnt = 0;
}
static void Set_MANU_System_Monitoring(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	AllDevStandbyStates();
	AllDevEnableStates();
	manu_step = CMD_SYSTEM_MONITORING;
	ble_cnt = 0;
}
static void Set_MANU_ImuCalib_RH_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	AllDevStandbyStates();
	AllSetManuPDO();
	manu_step = CMD_IMU_CALIB_RH_START;
	ble_cnt = 0;
}
static void Set_MANU_ImuCalib_RH_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	DOPI_SDOUnit_t sdo_unit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[2].data.autoCalibSensorCmd = 2;
	sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_AUTO_CALIB_SENSOR, SDO_REQU, 1);
	DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	TxSDO(2);
	one_time_sw = 1;
	manu_step = CMD_IMU_CALIB_RH_STOP;
	ble_cnt = 0;
}
static void Set_MANU_ImuCalib_LH_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
				AllDevStandbyStates();
				AllSetManuPDO();
				manu_step = CMD_IMU_CALIB_LH_START;
				ble_cnt = 0;
}
static void Set_MANU_ImuCalib_LH_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			DOPI_SDOUnit_t sdo_unit;
			DOPI_ClearSDO(&sdo_msg);

			userCtrlObj[3].data.autoCalibSensorCmd = 2;
			sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_AUTO_CALIB_SENSOR, SDO_REQU, 1);
			DOPI_AppendSDO(&sdo_unit, &sdo_msg);

			TxSDO(3);
			one_time_sw = 1;
			manu_step = CMD_IMU_CALIB_LH_STOP;
			ble_cnt = 0;
}
static void Set_MANU_EncoderCheck_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			AllDevEnableStates();
			SetManu2PDO(2);
			SetManu2PDO(3);
			manu_step = CMD_ENCODER_CHECK_START;
			ble_cnt = 0;
}
static void Set_MANU_EncoderCheck_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
		manu_step = CMD_ENCODER_CHECK_STOP;
		AllSetManuPDO();
		ble_cnt = 0;
}
static void Set_MANU_Encoder_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
		SetManuPDO(2);
		SetManuPDO(3);
		ble_cnt = 0;
}

static void Set_MANU_RH_Range_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			DOPI_SDOUnit_t sdo_unit;
			DOPI_ClearSDO(&sdo_msg);
			sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_RANGE_OFFSET, SDO_REQU, 1);
			DOPI_AppendSDO(&sdo_unit, &sdo_msg);

			TxSDO(2);
			manu_step = CMD_RH_MD_RANAGE_OFFSET;
			ble_cnt = 0;
}

static void Set_MANU_RH_Value_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			DOPI_SDOUnit_t sdo_unit;
			DOPI_ClearSDO(&sdo_msg);
			sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_VALUE_OFFSET, SDO_REQU, 1);
			DOPI_AppendSDO(&sdo_unit, &sdo_msg);

			TxSDO(2);
			manu_step = CMD_RH_MD_VALUE_OFFSET;
			ble_cnt = 0;

			/* Go SAM10 mid-level-task standby */
			Set_MANU_RH_Offset_calibration_done = true;
			//DevStandbyStates(2);

}
static void Set_MANU_LH_Range_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			DOPI_SDOUnit_t sdo_unit;

			DOPI_ClearSDO(&sdo_msg);
			sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_RANGE_OFFSET, SDO_REQU, 1);
			DOPI_AppendSDO(&sdo_unit, &sdo_msg);

			TxSDO(3);
			manu_step = CMD_LH_MD_RANAGE_OFFSET;
			ble_cnt = 0;
}
static void Set_MANU_LH_Value_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			DOPI_SDOUnit_t sdo_unit;
			DOPI_ClearSDO(&sdo_msg);
			sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_VALUE_OFFSET, SDO_REQU, 1);
			DOPI_AppendSDO(&sdo_unit, &sdo_msg);

			TxSDO(3);
			manu_step = CMD_LH_MD_VALUE_OFFSET;
			ble_cnt = 0;

			/* Go SAM10 mid-level-task standby */
			Set_MANU_LH_Offset_calibration_done = true;
			//DevStandbyStates(3);
}
static void Set_MANU_CurrentSensor_Check(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			if(userCtrlObj[2].data.err_code != (uint32_t)1U<<6 && userCtrlObj[3].data.err_code != (uint32_t)1U<<6)
		{
			DOPI_SDOUnit_t sdoUnit;
			DOPI_ClearSDO(&BLE_sdo_msg);
			sdoUnit = DOPI_CreateSDOUnit(&BLECtrlObj.devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CURRENT_SENSOR_CONFIRMED, SDO_REQU, 1);
			BT_AppendSDO(&sdoUnit, &BLE_sdo_msg);

			manu_step = CMD_MD_CURRENT_SENSOR_CHECK;
		}
			ble_cnt = 0;
}
static void Set_MANU_Monitoring_Enable(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			check_error=1;
			manu_step = CMD_MONITORING_ENABLE;
			ble_cnt = 0;
}
static void Set_MANU_ButtonTest_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
			BTN_test[0] = 0;
			BTN_test[1] = 0;
			BTN_test[2] = 0;

			BTN_res[0] = 0;
			BTN_res[1] = 0;
			BTN_res[2] = 0;
			manu_step = CMD_BUTTON_TEST;
			ble_cnt = 0;
}
static void Set_MANU_RH_PositiveMax_Position(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	DOPI_SDOUnit_t sdoUnit;

	DOPI_ClearSDO(&sdo_msg);
	/* Set P-vector Init. Position Reference  */
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);
	/* Send SDO to RH */
	TxSDO(2);

	inter_cnt = 0;
	motion_test_mode = 1;
	ble_cnt = 0;
}
static void Set_MANU_RH_NegativeMax_Position(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	DOPI_SDOUnit_t sdoUnit;

	DOPI_ClearSDO(&sdo_msg);
	/* Set P-vector Init. Position Reference  */
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);
	/* Send SDO to RH */
	TxSDO(2);

	inter_cnt = 0;
	motion_test_mode = 2;
	ble_cnt = 0;
}
static void Set_MANU_LH_PositiveMax_Position(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	DOPI_SDOUnit_t sdoUnit;

	DOPI_ClearSDO(&sdo_msg);
	/* Set P-vector Init. Position Reference  */
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);
	/* Send SDO to RH */
	TxSDO(3);

	inter_cnt = 0;
	motion_test_mode = 3;
	ble_cnt = 0;
}
static void Set_MANU_LH_NegativeMax_Position(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	DOPI_SDOUnit_t sdoUnit;

	DOPI_ClearSDO(&sdo_msg);
	/* Set P-vector Init. Position Reference  */
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);
	/* Send SDO to RH */
	TxSDO(3);

	inter_cnt = 0;
	motion_test_mode = 4;
	ble_cnt = 0;
}

static void Set_MANU_DrivingTest_End(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	inter_cnt = 0;
	motion_test_mode = 0;
	ble_cnt = 0;
}

static void Set_MANU_Led_Test(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	
	memcpy(&LED_test, t_req->data, 1);
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
	manu_step = CMD_LED_TEST;
	ble_cnt = 0;
}
static void Set_MANU_SoundTest_Voice(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	Run_Audio_Action(0);
	ble_cnt = 0;
}
static void Set_MANU_SoundTest_Beep(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	Run_Audio_Action(999);
	ble_cnt = 0;
}

static void Set_MANU_Magnetic_Sensor_Test(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	manu_step = CMD_MAGNETIC_SENSOR_TEST;
	
	MAG_test[0] = 0;
	MAG_test[1] = 0;

	MAG_res[0] = 0;
	MAG_res[1] = 0;
	ble_cnt = 0;
}

static void Set_MANU_AgingTest_Start(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	manu_step = CMD_AGING_TEST_START;

	AllSetManuPDO();

	DOPI_SDOUnit_t sdoUnit;

	DOPI_ClearSDO(&sdo_msg);
	/* 1. Deactivate joint limit */
	userCtrlObj[2].data.joint_limit_sw = 1;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);
	/* 2. Set P-vector Init. Position Reference  */
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);
	/* 3. Send SDO to RH */
	TxSDO(2);

	DOPI_ClearSDO(&sdo_msg);
	/* 1. Deactivate joint limit */
	userCtrlObj[3].data.joint_limit_sw = 1;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);
	/* 2. Set P-vector Init. Position Reference */
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);
	/* 3. Send SDO to LH */
	TxSDO(3);

	motion_test_mode = 5;
	aging_enable = 1;
	inter_cnt = 0;
	ble_cnt = 0;
}

static void Set_MANU_AgingTest_Stop(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	manu_step = CMD_AGING_TEST_STOP;

	motion_test_mode = 0;
	aging_enable = 0;
	inter_cnt = 0;
	ble_cnt = 0;
}

static void Set_MANU_Org_code(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	memcpy(Org_Code, t_req->data, 10);
	FLASHSaveFlag = SAVE_FLASH_BLE_DATA;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
	ble_cnt = 0;
	manu_step = CMD_ORG_SETTING;
}

static void Set_MANU_Mobile_UUID_Reset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	memset(MobileUuidFront, 0, sizeof(MobileUuidFront));
	memset(MobileUuidBack, 0, sizeof(MobileUuidBack));
	FLASHSaveFlag = SAVE_FLASH_BLE_DATA;
	uuid_reset_result = 1;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
	ble_cnt = 0;
	manu_step = CMD_MOBILE_UUID_RESET;
}

static void Set_MANU_RTC(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	memcpy(&RTC_Time, t_req->data, 7);
	IOIF_MCP79510_SetTime(&RTC_Time);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;

}

static void Start_toruqe_measure(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

		DOPI_SDOUnit_t sdoUnit;
		DOPI_ClearSDO(&sdo_msg);

		userCtrlObj[2].data.Torque_measure_sw = *(uint8_t *)t_req->data;
		sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ENABLE_TORQUE_MEASURE, SDO_REQU, 1);
		DOPI_AppendSDO(&sdoUnit, &sdo_msg);

		TxSDO(2);

		DOPI_ClearSDO(&sdo_msg);
		userCtrlObj[3].data.Torque_measure_sw = *(uint8_t *)t_req->data;
		sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ENABLE_TORQUE_MEASURE, SDO_REQU, 1);
		DOPI_AppendSDO(&sdoUnit, &sdo_msg);

		TxSDO(3);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Stop_torque_measure(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

		DOPI_SDOUnit_t sdoUnit;
		DOPI_ClearSDO(&sdo_msg);

		userCtrlObj[2].data.Torque_measure_sw = 0;
		sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[2].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ENABLE_TORQUE_MEASURE, SDO_REQU, 1);
		DOPI_AppendSDO(&sdoUnit, &sdo_msg);

		TxSDO(2);

		DOPI_ClearSDO(&sdo_msg);
		userCtrlObj[3].data.Torque_measure_sw = 0;
		sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[3].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ENABLE_TORQUE_MEASURE, SDO_REQU, 1);
		DOPI_AppendSDO(&sdoUnit, &sdo_msg);

		TxSDO(3);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Send_ORG_CODE(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){

	manu_step = CMD_ORG_READING;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}
static void SetupBLEDOD(AS_BLE_CtrlObj_t* obj)
{
	/* BLE Data Set SDO Address For (PRO)User App */
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_FUNCTION, 			&obj->data.msgCommand);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SUCCESS, 			&obj->data.msgCTM);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_ERROR_MSG_CODE, 		&obj->data.ErrorMSGcode);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_APP_INTEGRITY, 		&obj->data.aes_encrypted_base_64_res);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOBILE_UUID, 			&obj->data.MobileUuidFront);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_CHECK_USER_ORG_CODE, 	&obj->data.Org_Code);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_CHECK_LOGIN_DATA, 		&obj->data.LoginData);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SET_LEFT_THIGH_NEUTRAL_SAGITTAL_OFFSET, 		&obj->data.LeftThighNeutralPostureOffset);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SET_RIGHT_THIGH_NEUTRAL_SAGITTAL_OFFSET, 		&obj->data.RightThighNeutralPostureOffset);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SET_LEFT_SHANK_NEUTRAL_SAGITTAL_OFFSET, 		&obj->data.LeftShankNeutralPostureOffset);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SET_RIGHT_SHANK_NEUTRAL_SAGITTAL_OFFSET, 		&obj->data.RightShankNeutralPostureOffset);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_NEUTRAL_POSTURE_CALIBRATION, 					&obj->data.LeftThighNeutralSagittalAngle);

	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA_STATE, 				&obj->data.MotionAnalysisDataState);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_DATA, 						&obj->data.MotionAnalysisReportId);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_ANGLE_PATTERN_LEFT_X, 				&obj->data.ThighAngleLeftX);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_ANGLE_PATTERN_LEFT_Y_MEAN, 			&obj->data.ThighAngleLeftYMean);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_ANGLE_PATTERN_LEFT_Y_STD, 			&obj->data.ThighAngleLeftYStd);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_ANGLE_PATTERN_RIGHT_X, 				&obj->data.ThighAngleRightX);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_ANGLE_PATTERN_RIGHT_Y_MEAN, 			&obj->data.ThighAngleRightYMean);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_ANGLE_PATTERN_RIGHT_Y_STD, 			&obj->data.ThighAngleRightYStd);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_LEFT_X,				&obj->data.ThighTorqueLeftX);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_LEFT_Y_MEAN,			&obj->data.ThighTorqueLeftYMean);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_LEFT_Y_STD,			&obj->data.ThighTorqueLeftYStd);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_RIGHT_X, 				&obj->data.ThighTorqueRightX);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_RIGHT_Y_MEAN, 			&obj->data.ThighTorqueRightYMean);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_THIGH_TORQUE_PATTERN_RIGHT_Y_STD, 			&obj->data.ThighTorqueRightYStd);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_MOTION_ANALYSIS_FINISH_ABLE, 				&obj->data.MotionAnalysisFinishAble);

	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_HW_VERSION_CONFIRMED, 						&obj->data.VersionData);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_START, 						&obj->data.SPPB_Start_Flag);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_STOP, 						&obj->data.SPPB_Stop_Flag);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_SPPB_STANDING_COMPLETE, 						&obj->data.SPPB_Standing_Count);
	// SDO
	
	// CM -> APP
//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_OPERATION_FSM_VERSION, &obj->data.OperationFsmVersion);
//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_SERVICE_FSM_VERSION, &obj->data.ServiceFsmVersion);
//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_DMS_SETTING_VERSION, &obj->data.DataManagementSettingVersion);
//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_MOTION_MAP_VERSION, &obj->data.MotionMapVersion);
//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_ROBOTSETTING_VERSION, &obj->data.RobotSettingVersion);

//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_LEFT_THIGH_VELOCITY_RAW, &obj->data.LeftThighVelocityRaw);
//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_RIGHT_THIGH_VELOCITY_RAW, &obj->data.RightThighVelocityRaw);
//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_LEFT_SHANK_VELOCITY_RAW, &obj->data.LeftShankVelocityRaw);
//	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_ANGELA, SDO_ID_ANGELA_RIGHT_SHANK_VELOCITY_RAW, &obj->data.LeftShankVelocityRaw);


	/* BLE Data Set SDO Address For MANU App */

	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_TEST, 								&obj->data_manu.test);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SUCCESS, 							&obj->data_manu.msgCTM);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_START_CONFIRMED,					&obj->data_manu.start_result);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_ESP32_COMM_TEST_RESULT, 			&obj->data_manu.SPI_test_result);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MES_CONFIRM_UINT8, 					&obj->data_manu.RTC_year);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MES_CONFIRM_STRING10, 				&obj->data_manu.Serial_number_front);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SD_CARD_CONFIRMED, 					&obj->data_manu.sdcard_test_result);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_FDCAN_CONFIRMED,					&obj->data_manu.fdcan_test_result);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_HW_VERSION_CONFIRMED,				&obj->data_manu.CM_HW_ver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SW_VERSION_CONFIRMED_UINT8,			&obj->data_manu.CM_SW_ver_major);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SW_VERSION_CONFIRMED_STRING10,		&obj->data_manu.RH_Serial_number_front);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SYSTEM_MONITORING_DONE,				&obj->data_manu.CM_Voltage_aver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_IMU_CALIB_RH_STOP_DONE,				&obj->data_manu.RH_acc_mean_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_IMU_CALIB_LH_STOP_DONE,				&obj->data_manu.LH_acc_mean_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CURRENT_SENSOR_CONFIRMED,			&obj->data_manu.current_test_result);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BUTTON_TEST_DONE,					&obj->data_manu.button_test_result);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MAGNETIC_SENSOR_TEST_DONE,			&obj->data_manu.magnetic_test_result);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_ORG_CODE_CONFIRMED,					&obj->data.Org_Code);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MOBILE_UUID_CONFIRMED,				&obj->data_manu.uuid_reset_result);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BOOT_CHECK_ACK,						&obj->data_manu.IsConnectSuccess);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CONFIRM_RTC,						&obj->data_manu.RTC_data);


	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_ROBOT_MODEL, 				&obj->data_manu.Robot_model);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SERIAL_NUMBER_1, 			&obj->data_manu.Serial_number_front);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SERIAL_NUMBER_2, 			&obj->data_manu.Serial_number_middle);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_SERIAL_NUMBER_3, 			&obj->data_manu.Serial_number_back);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_YEAR, 						&obj->data_manu.RTC_year);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MONTH, 						&obj->data_manu.RTC_month);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_DAY, 						&obj->data_manu.RTC_day);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_HOUR, 						&obj->data_manu.RTC_hour);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MINUTE, 					&obj->data_manu.RTC_minute);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_INSPECTOR, 					&obj->data_manu.Inspector_name);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MAC_ADD_1, 					&obj->data_manu.BT_MAC_add_front);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_MAC_ADD_2, 					&obj->data_manu.BT_MAC_add_back);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BT_NAME_1, 					&obj->data_manu.BT_name_front);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BT_NAME_2, 					&obj->data_manu.BT_name_back);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_BAUDRATE, 					&obj->data_manu.BT_baudrate);

	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_HW_VER, 					&obj->data_manu.CM_HW_ver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_MD_HW_VER, 				&obj->data_manu.RH_MD_HW_ver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_MD_HW_VER, 				&obj->data_manu.LH_MD_HW_ver);
//	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_SW_VER, 					&obj->data_manu.CM_SW_ver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_MD_SW_VER, 				&obj->data_manu.RH_MD_SW_ver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_MD_SW_VER, 				&obj->data_manu.LH_MD_SW_ver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_VOLTAGE_AVER, 			&obj->data_manu.CM_Voltage_aver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_CURRENT_AVER, 			&obj->data_manu.CM_Current_aver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_TEMP_AVER, 				&obj->data_manu.CM_Temp_aver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_PERCENTAGE_AVER, 		&obj->data_manu.CM_Percentage_aver);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_ACC_MEAN_VALUE, 			&obj->data_manu.RH_acc_mean_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_GYRO_SCALING, 			&obj->data_manu.RH_gyro_scailing);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_GYRO_BIAS, 				&obj->data_manu.RH_gyro_bias);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_MK_SCALING, 				&obj->data_manu.RH_mk_scaling);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_ACC_MEAN_VALUE, 			&obj->data_manu.LH_acc_mean_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_GYRO_SCALING, 			&obj->data_manu.LH_gyro_scaling);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_GYRO_BIAS, 				&obj->data_manu.LH_gyro_bias);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_MK_SCALING, 				&obj->data_manu.LH_mk_scaling);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_ABS_1_VALUE, 			&obj->data_manu.RH_abs_1_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_ABS_2_VALUE, 			&obj->data_manu.RH_abs_2_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_INC_VALUE, 				&obj->data_manu.RH_inc_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_ABS_1_VALUE, 			&obj->data_manu.LH_abs_1_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_ABS_2_VALUE, 			&obj->data_manu.LH_abs_2_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_INC_VALUE, 				&obj->data_manu.LH_inc_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_ABS_VALUE, 				&obj->data_manu.RH_abs_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_ABS_VALUE, 				&obj->data_manu.LH_abs_value);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_INC_BUTTON_TESTED, 			&obj->data_manu.INC_button_test);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_DEC_BUTTON_TESTED, 			&obj->data_manu.DEC_button_test);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_POWER_BUTTON_TESTED, 		&obj->data_manu.POWER_button_test);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_VOLTAGE, 				&obj->data_manu.CM_voltage);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_CURRENT, 				&obj->data_manu.CM_current);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_CM_TEMP, 			    	&obj->data_manu.CM_temp);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_MD_VOLTAGE, 				&obj->data_manu.RH_voltage);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_MD_CURRENT, 				&obj->data_manu.RH_current);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_MD_TEMP, 				&obj->data_manu.RH_temp);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_MD_VOLTAGE, 				&obj->data_manu.LH_voltage);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_MD_CURRENT, 				&obj->data_manu.LH_current);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_MD_TEMP, 				&obj->data_manu.LH_temp);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_FUNCTION_SW, 				&obj->data_manu.Func_SW);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_ERROR_CODE, 				&obj->data_manu.RH_err_code);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_ERROR_CODE, 				&obj->data_manu.LH_err_code);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_SERIAL_NUMBER_1, 		&obj->data_manu.RH_Serial_number_front);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_SERIAL_NUMBER_2, 		&obj->data_manu.RH_Serial_number_middle);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_RH_SERIAL_NUMBER_3, 		&obj->data_manu.RH_Serial_number_back);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_SERIAL_NUMBER_1, 		&obj->data_manu.LH_Serial_number_front);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_SERIAL_NUMBER_2, 		&obj->data_manu.LH_Serial_number_middle);
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_MANU_CM_to_Mobile, SDO_ID_MANU_CTM_LH_SERIAL_NUMBER_3, 		&obj->data_manu.LH_Serial_number_back);

	// CM -> ESP32 Command & Query
	DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ESP32_CMD, SDO_ID_ESP32_CMD_READY_CHECK, 						&obj->data_cmd.IsReadyCheck);

	// APP -> CM
	// DOPI_SetSDOAddr(&obj->devObj, BT_TASK_ID_ANGELA_CM_to_Mobile, SDO_ID_ANGELA_CTM_CONTENT_FILE_DOWNLOAD, &obj->data.);
}

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- ROUTINE ------------------------ */

#endif /* SUIT_MINICM_ENABLED */
