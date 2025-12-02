

#include "AS_dev_mngr.h"

#include "ext_link_mngr.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

extern int Check_FD_CAN_COMMU_RX(uint8_t* t_byte_arr, uint8_t MD_nodeID);

DOPI_SDOMsg_t sdo_msg;
DOPI_PDOMsg_t pdo_msg;
//AS_CtrlObj_t userCtrlObj[DEV_IDX_MD16];
AS_CtrlObj_t userCtrlObj[AS_DEV_IDX_MAX];
/*RM*/
FaultMD_t mdErrorBit[RM_MD_ERROR_ARRAY];
bool md_error_flag = false;

uint32_t total_error_code;
uint32_t p_total_error_code;

/* [MD->MiniCM] Raw Data version */
// RH //
float widmDegFinal_RH = 0;
float widmVelFinal_RH = 0;

// LH //
float widmDegFinal_LH = 0;
float widmVelFinal_LH = 0;

// RK //
float widmDegFinal_RK = 0;
float widmVelFinal_RK = 0;
float widmGyrZ_RK = 0;
float widmDegINC_RK = 0;
float widmVelINC_RK = 0;

// LK //
float widmDegFinal_LK = 0;
float widmVelFinal_LK = 0;
float widmGyrZ_LK = 0;
float widmDegINC_LK = 0;
float widmVelINC_LK = 0;

// Current Ref, Cur
float CurrentRef_RH = 0;
float CurrentRef_LH = 0;
float CurrentRef_RK = 0;
float CurrentRef_LK = 0;

float CurrentAct_RH = 0;
float CurrentAct_LH = 0;
float CurrentAct_RK = 0;
float CurrentAct_LK = 0;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void SetRoutine(AS_TaskData_t* taskObj, uint8_t routine_id);
static void ClearRoutine(AS_TaskData_t* taskObj, uint8_t routine_id);

static void SetMDRoutines(int DevIdx);
static void StandbyStates(int DevIdx);
static void EnableStates(int DevIdx);
static void OffStates(int DevIdx);

static void SetupDOD(AS_CtrlObj_t* obj);
static int AS_FDCAN_CB(uint16_t wasp_id, uint8_t* rx_data);

/*RM*/
static bool ParsingMDErrorCode(uint8_t device_id, FaultLocationMD_t md_task, uint32_t error_bit);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Initialize Motor Driver Object(CTRLObj) and FDCAN communication
 */
void InitFDCANDevMngr(void)
{
	static uint8_t isDevInit = 0;

	if (isDevInit == 0) {

    	/* Allocate RX Callback*/
		IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, AS_FDCAN_CB);
		IOIF_SetFDCANRxCB(IOIF_FDCAN2, IOIF_FDCAN_RXFIFO0CALLBACK, AS_FDCAN_CB);

		/* Driver Init */
		for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
			uint8_t tUsage = RS_File.vec[DevIdx].usage;
			
			if (tUsage == 1) {
				uint16_t tempNodeID = RS_File.vec[DevIdx].FDCAN_ID;
				uint8_t  tempFDCANCH = RS_File.vec[DevIdx].FDCAN_CH;

				AS_FDCAN_TxFncPtr txfnc = NULL;

				if (tempFDCANCH == 1) {
					txfnc = IOIF_TransmitFDCAN1;
				} else if (tempFDCANCH == 2) {
					txfnc = IOIF_TransmitFDCAN2;
				} else {
					//error handler
				}

				userCtrlObj[DevIdx].sdo_tx_id = SDO | (NODE_ID_CM << 4) | tempNodeID;
				userCtrlObj[DevIdx].pdo_tx_id = PDO | (NODE_ID_CM << 4) | tempNodeID;
				userCtrlObj[DevIdx].tx_fnc = txfnc;

				memset(&userCtrlObj[DevIdx].data, 0, sizeof(userCtrlObj[DevIdx].data));
				DOPI_InitDevObj(&userCtrlObj[DevIdx].devObj, tempNodeID);
				SetupDOD(&userCtrlObj[DevIdx]);
			}
		}

		isDevInit = 1;
	}
}
/* For SUIT Data Collection */
/*void SendAuxInput(int DevIdx, float motorAuxIn, float assistPctg)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.motor_auxiliary_input = motorAuxIn * assistPctg;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_AUX_INPUT, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

void Send_F_Vector(int DevIdx, uint8_t modeIdx, int16_t TauMax, uint16_t delay)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.ModeIdx = modeIdx;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	userCtrlObj[DevIdx].data.TauMax = TauMax * assistForcePctg;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_TMAX, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	userCtrlObj[DevIdx].data.Delay = delay;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_DELAY, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}*/

/*******************************************************************/
void AllDevOffStates(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			OffStates(DevIdx);
		}
	}
}

void AllDevStandbyStates(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			StandbyStates(DevIdx);
		}
	}
}

bool DevStandbyStates(int DevIdx)
{
	bool res = false;

	if (RS_File.vec[DevIdx].usage == 1){
		StandbyStates(DevIdx);
		res = true;
	}
	return res;
}

void AllDevEnableStates(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			SetDevTaskState(DevIdx);
		}
	}
}

void AllDevSetRoutines(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			SetDevRoutine(DevIdx);
		}
	}
}

void AllDevSetJointLimitRoutine(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			SetDevJointLimitRoutine(DevIdx);
		}
	}
}

void AllDevClearJointLimitRoutine(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			ClearDevJointLimitRoutine(DevIdx);
		}
	}
}

void AllSetDevPDO(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			SetDevPDO(DevIdx);
		}
	}
}

void AllSyncPDOData(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			Send_PDO_Nothing(DevIdx);
		}
	}
}

void AllSetManuPDO(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			SetManuPDO(DevIdx);
		}
	}
}

void AllSetDevPDO_MANU(void)
{
	for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		if (RS_File.vec[DevIdx].usage == 1) {
			SetDevPDO(DevIdx);
		}
	}
}


AS_DevData_t* GetDevDataSet(int DevIdx)
{
    return &userCtrlObj[DevIdx].data;
}

/* For SUIT Data Collection */
void SetDevPDO(int DevIdx)
{
 	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

	/*(1) Set MSG PDO */
 	static uint8_t PDOList[30] = {
								  /* Actuator Data */
// 								  TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT,
								  TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT,
								  TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VOLTAGE_SCALING,

								  /* Motion Data */
								  //   TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_POSITION,

								  TASK_ID_GAIT, PDO_ID_GAIT_THIGH_ANGLE_DEG,
								  TASK_ID_GAIT, PDO_ID_GAIT_THIGH_VEL_DEG,

								  TASK_ID_GAIT, PDO_ID_GAIT_INC_POS_DEG,
								  TASK_ID_GAIT, PDO_ID_GAIT_INC_VEL_DEG,

								  /* Acc & Gyro Axis Data */
								  TASK_ID_GAIT, PDO_ID_GAIT_ACC_X_GLOBAL,
								  TASK_ID_GAIT, PDO_ID_GAIT_ACC_Y_GLOBAL,
								  TASK_ID_GAIT, PDO_ID_GAIT_ACC_Z_GLOBAL,

								  TASK_ID_GAIT, PDO_ID_GAIT_GYR_X_GLOBAL,
								  TASK_ID_GAIT, PDO_ID_GAIT_GYR_Y_GLOBAL,
								  TASK_ID_GAIT, PDO_ID_GAIT_GYR_Z_GLOBAL,

								  TASK_ID_GAIT, PDO_ID_GAIT_IMU_VQF_SAGITTAL,
								  TASK_ID_GAIT, PDO_ID_GAIT_IMU_VQF_FRONTAL,
								//   TASK_ID_GAIT, PDO_ID_GAIT_IMU_VQF_TRANSVERSE,

								  TASK_ID_EXTDEV, PDO_ID_EXTDEV_MOTOR_TEMP_SCALING,
 								};

 	memcpy(userCtrlObj[DevIdx].data.pdo_list, PDOList, sizeof(PDOList));
 	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, SDO_REQU, sizeof(PDOList)/2);
 	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

 	TxSDO(DevIdx);
}

/* For SUIT Data Collection */
void SetManuPDO(int DevIdx)
{
 	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

	/*(1) Set MSG PDO */
 	static uint8_t PDOList[18] = {
								  /* Actuator Data */
 			TASK_ID_GAIT, PDO_ID_GAIT_ACC_MEAN_VALUE,
			TASK_ID_GAIT, PDO_ID_GAIT_GYRO_SCALING,
			TASK_ID_GAIT, PDO_ID_GAIT_GYRO_BIAS,
			TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_POSITION,
			TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER1_POSITION,
			TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER2_POSITION,
			TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ACTUAL_VOLTAGE,
			TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ACTUAL_CURRENT,
			TASK_ID_EXTDEV, PDO_ID_EXTDEV_NTC_MOTOR_TEMP
 								};
 	memcpy(userCtrlObj[DevIdx].data.pdo_list, PDOList, sizeof(PDOList));
 	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, SDO_REQU, sizeof(PDOList)/2);
 	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

 	TxSDO(DevIdx);
}

void SetManu2PDO(int DevIdx)
{
 	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

	/*(1) Set MSG PDO */
 	static uint8_t PDOList[6] = {
								  /* Actuator Data */
			TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER1_VELOCITY,
			TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER2_VELOCITY,
			TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW,


 								};
 	memcpy(userCtrlObj[DevIdx].data.pdo_list, PDOList, sizeof(PDOList));
 	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, SDO_REQU, sizeof(PDOList)/2);
 	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

 	TxSDO(DevIdx);
}

void SetSerialSDO(int DevIdx)
{
 	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

	/*(1) Set MSG PDO */
 	static uint8_t SDOList[6] = {
								  /* Actuator Data */
 			TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SERIAL_1,
			TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SERIAL_2,
			TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SERIAL_3,

 								};
 	memcpy(userCtrlObj[DevIdx].data.sdo_list, SDOList, sizeof(SDOList));
 	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SDO_LIST, SDO_REQU, sizeof(SDOList)/2);
 	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

 	TxSDO(DevIdx);
}

void SetVersionSDO(int DevIdx)
{
 	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

	/*(1) Set MSG PDO */
 	static uint8_t SDOList[10] = {
								  /* Actuator Data */
 			TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_HW_VER,
 			TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SW_VER_MAJOR,
 			TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SW_VER_MINOR,
 			TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SW_VER_PATCH,
 			TASK_ID_BLE_COMM_HDLR, SDO_ID_BLE_COMM_HDLR_SW_VER_DEBUG,

 								};
 	memcpy(userCtrlObj[DevIdx].data.sdo_list, SDOList, sizeof(SDOList));
 	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SDO_LIST, SDO_REQU, sizeof(SDOList)/2);
 	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

 	TxSDO(DevIdx);
}

void ReqAccMeanValue(int DevIdx)
{
 	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

	/*(1) Set MSG PDO */
 	static uint8_t SDOList[2] = {
								  /* Actuator Data */
 			TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_ACC_MEAN_VALUE,
 	};

 	memcpy(userCtrlObj[DevIdx].data.sdo_list, SDOList, sizeof(SDOList));
 	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SDO_LIST, SDO_REQU, sizeof(SDOList)/2);
 	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

 	TxSDO(DevIdx);
}

void SetDevJointLimitRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_DEGREE_LIMIT);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_VELOCITY_LIMIT);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

void ClearDevJointLimitRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	ClearRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_DEGREE_LIMIT);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	ClearRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_VELOCITY_LIMIT);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

// DOB
void SetDevDOBRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_DISTURBANCE_OBS);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

void ClearDevDOBRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	ClearRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_DISTURBANCE_OBS);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

// Degree Limit
void SetDevDegreeLimitRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_DEGREE_LIMIT);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

void ClearDevDegreeLimitRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	ClearRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_DEGREE_LIMIT);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

// Velocity Limit
void SetDevVelocityLimitRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_VELOCITY_LIMIT);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

void ClearDevVelocityLimitRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	ClearRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_VELOCITY_LIMIT);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

void SetDevRoutine(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	/* (1) Set MSG Routines */
	SetRoutine(&userCtrlObj[DevIdx].data.msg_hdlr_task, ROUTINE_ID_MSG_PDO_SEND);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.msg_hdlr_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* (2) Set Low Level Ctrl Routines */
    // Activate low&mid Routine (Current Ctrl, Torque Generator)
	SetRoutine(&userCtrlObj[DevIdx].data.low_level_ctrl_task, ROUTINE_ID_LOWLEVEL_CURRENT_CTRL);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.low_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

//	SetRoutine(&userCtrlObj[DevIdx].data.low_level_ctrl_task, ROUTINE_ID_LOWLEVEL_ADV_FRICTION_COMPENSATION);
//	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.low_level_ctrl_task.n_routines);
//	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* (3) Set Mid Level Ctrl Routines */
	//	 SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_IRC);
	//	 sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	//	 DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_GRAVITY_COMPENSATION);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_VELOCITY_COMPENSATION);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_F_VECTOR_DECODER);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_P_VECTOR_DECODER);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_IMPEDANCE_CONTROL);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.mid_level_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* (5) Set Gait Ctrl Routines */
	SetRoutine(&userCtrlObj[DevIdx].data.gait_ctrl_task, ROUTINE_ID_GAIT_TOTAL_FUNCTION);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.gait_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

void SetDevTaskState(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

	/* Set Low Level Ctrl State */
	userCtrlObj[DevIdx].data.low_level_ctrl_task.state = TASK_STATE_ENABLE;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* Set Mid Level Ctrl State */
	userCtrlObj[DevIdx].data.mid_level_ctrl_task.state = TASK_STATE_ENABLE;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* Set Gait Ctrl State */
	userCtrlObj[DevIdx].data.gait_ctrl_task.state = TASK_STATE_ENABLE;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_STATE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* Set Msg Hdlr State */
	userCtrlObj[DevIdx].data.msg_hdlr_task.state = TASK_STATE_ENABLE;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

void SendAuxInput(int DevIdx, float motorAuxIn, float assistPctg)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.motor_auxiliary_input = motorAuxIn * assistPctg;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_AUX_INPUT, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

void SendIsNeutralized(int DevIdx, uint8_t t_isN)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.isNeutralized = t_isN;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_IS_NEUTRALIZED, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

void SendIsNeutralInit(int DevIdx, uint8_t t_isN)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.isNeutralInit = t_isN;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_NEUTRAL_INITIAL_DEG, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

void SendNeutralPostureCalCMD(int DevIdx, uint8_t t_Cmd)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.neutralBiasCmd = t_Cmd;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_NEUTRAL_BIAS_CMD, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

void SendNeutralPostureOffset(int DevIdx, int16_t neutralPostureOffset)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.thighNeutralPostureOffset = neutralPostureOffset;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_NEUTRAL_BIAS_OFFSET, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

void SendUserNeutralPostureAngle(int DevIdx, float UserNeutralPostureAngle)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.UserNeutralPostureAngle = ScaleFloatToInt16(UserNeutralPostureAngle, DEG_SCALING_FACTOR);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_USER_NEUTRAL_POSTURE_ANGLE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

/* PIF Vector */

// function description: send controller gain vector to MD
// param1: MD ID
// param2: desired feed-forward controller gain        [0,255] in CM = [0, 1] in MD
// param3: desired PD position controller gain         [0,255] in CM = [0, 1] in MD
// param4: desired Impedance controller gain           [0,255] in CM = [0, 1] in MD
// param5: desired Disturbance observer gain           [0,255] in CM = [0, 1] in MD
// param5: desired Impedance reduction controller gain [0,255] in CM = [0, 1] in MD
// param6: desired Friction compensator gain           [0,255] in CM = [0, 1] in MD
void Send_C_Vector(int obj_idx, uint8_t K_FF, uint8_t K_PD, uint8_t K_IC, uint8_t K_DOB, uint8_t K_IRC, uint8_t K_FC)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.c_vector.FF_gain  = K_FF;
    userCtrlObj[obj_idx].data.c_vector.PD_gain  = K_PD;
    userCtrlObj[obj_idx].data.c_vector.IC_gain  = K_IC;
    userCtrlObj[obj_idx].data.c_vector.DOB_gain = K_DOB;
    userCtrlObj[obj_idx].data.c_vector.IRC_gain = K_IRC;
    userCtrlObj[obj_idx].data.c_vector.FC_gain  = K_FC;

    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_FF_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_PD_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_IC_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_DOB_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_IRC_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_FC_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(obj_idx);
}

// function description: send P-vector to MD
// param1: MD ID
// param2: desired position (unit: deg)
// param3: duration         (unit: ms)
// param4: normalized acceleration (unit: deg/s^2)
// param5: normalized deceleration (unit: deg/s^2)
void Send_P_Vector(int obj_idx, int16_t yd, uint16_t L, uint8_t s0, uint8_t sd)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.p_vector.yd = 8.726646259971647 * yd;  // = 8.726646259971647 = M_PI*500 /180; (tx: x500, rx: x0.002 x0.1)

    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_YD, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.p_vector.L = L;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_L, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.p_vector.s0 = s0;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_S0, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.p_vector.sd = sd;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_SD, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(obj_idx);
}

// function description: send P-vector to MD
// param1: MD ID
void Send_P_Vector_Reset(int DevIdx)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_RESET, SDO_REQU, 0);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(DevIdx);
}


// function description: send F-vector to MD
// param1: MD ID
// param2: torque mode idx (unit: -)
// param3: maximum torque  (unit: A)
// param4: delay           (unit: ms)
void Send_F_Vector(int DevIdx, uint8_t modeIdx, int16_t TauMax, uint16_t Delay)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.md_f_vector.mode_idx = modeIdx;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	userCtrlObj[DevIdx].data.md_f_vector.tau_max = TauMax;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_TMAX, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	userCtrlObj[DevIdx].data.md_f_vector.delay = Delay;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_DELAY, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);

}

void Send_F_Vector_Reset(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	uint16_t zero =0;

	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.f_vector.zero = zero;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_ZERO, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

// function description: send impedance controller setting to MD
// param1: MD ID
// param2: Half width of the Corridor      (unit: deg)
// param3: Magnitude of the Virtual Spring (unit: %)
// param4: Magnitude of the Virtual Damper (unit: %)
// param5: Impedance Ratio in the Corridor (0~2)
// param6: Duration for Transition         (unit: ms for dT = 1ms)
void Send_I_Vector(int obj_idx, uint8_t t_epsilon, uint8_t t_Kp, uint8_t t_Kd, uint8_t t_lambda, uint16_t duration)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.i_vector.epsilon_target = t_epsilon;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_EPSILON, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.i_vector.Kp_target = t_Kp;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.i_vector.Kd_target = t_Kd;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.i_vector.lambda_target = t_lambda;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_LAMBDA, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.i_vector.duration = duration;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_DURATION, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(obj_idx);
}

void Set_I_Vector_Kp_Max(int obj_idx, float Kp_max)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.i_vector_Kp_max = Kp_max;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP_MAX, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);
    TxSDO(obj_idx);
}

void Set_I_Vector_Kd_Max(int obj_idx, float Kd_max)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.i_vector_Kd_max = Kd_max;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD_MAX, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);
    TxSDO(obj_idx);
}

void Set_I_Vector_KpKd_Max(int objIdx, float kpMax, float kdMax)
{
	DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[objIdx].data.i_vector_Kp_max = kpMax;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[objIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP_MAX, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[objIdx].data.i_vector_Kd_max = kdMax;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[objIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD_MAX, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	TxSDO(objIdx);
}

void Send_AUX_Torque(int MD_idx, float torque)
{
    DOPI_PDOUnit_t pdo_unit;
    DOPI_ClearPDO(&pdo_msg);

    userCtrlObj[MD_idx].data.motor_auxiliary_input = torque;
    pdo_unit = DOPI_CreatePDOUnit(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_AUXILIARY_INPUT, &userCtrlObj[MD_idx].data.motor_auxiliary_input);
    DOPI_AppendPDO(&pdo_unit, &pdo_msg);

    TxPDO(MD_idx);
}

void Send_PDO_Nothing(int MD_idx)
{
    DOPI_ClearPDO(&pdo_msg);

	TxPDO(MD_idx);
}

void Send_Grav_Vel_Comp_Gain(int obj_idx, float gravCompGain, float velCompGain)
{
	DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.gravCompGain = gravCompGain;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_GRAV_COMP_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[obj_idx].data.velCompGain = velCompGain;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VEL_COMP_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(obj_idx);
}

void Send_Comp_parameter(int obj_idx, float gravCompGain, float velCompGain, float velCompSatu, uint8_t suit_mode)
{
	DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.suit_mode = suit_mode;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_SUIT_MODE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.gravCompGain = gravCompGain;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_GRAV_COMP_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[obj_idx].data.velCompGain = velCompGain;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VEL_COMP_GAIN, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[obj_idx].data.velCompSatu = velCompSatu;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VEL_COMP_SATU, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(obj_idx);
}

void Send_Limit_parameter(int obj_idx, float HipRoMUpperLimit, float HipRoMLowerLimit, float HipUpperVelLimit, float HipLowerVelLimit, uint8_t suit_mode)
{
	DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.suit_mode = suit_mode;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_SUIT_MODE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    userCtrlObj[obj_idx].data.HipRoMUpperLimit = HipRoMUpperLimit;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_UPPER_LIMIT, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[obj_idx].data.HipRoMLowerLimit = HipRoMLowerLimit;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_LOWER_LIMIT, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[obj_idx].data.HipUpperVelLimit = HipUpperVelLimit;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_UPPER_VEL_LIMIT, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[obj_idx].data.HipLowerVelLimit = HipLowerVelLimit;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_LOWER_VEL_LIMIT, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(obj_idx);
}

void Send_Degree_Limit(int obj_idx, float HipRoMUpperLimit, float HipRoMLowerLimit)
{
	DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[obj_idx].data.HipRoMUpperLimit = HipRoMUpperLimit;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_UPPER_LIMIT, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[obj_idx].data.HipRoMLowerLimit = HipRoMLowerLimit;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_LOWER_LIMIT, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(obj_idx);
}

void Send_Velocity_Limit(int obj_idx, float HipUpperVelLimit, float HipLowerVelLimit)
{
	DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[obj_idx].data.HipUpperVelLimit = HipUpperVelLimit;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_UPPER_VEL_LIMIT, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	userCtrlObj[obj_idx].data.HipLowerVelLimit = HipLowerVelLimit;
    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[obj_idx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_LOWER_VEL_LIMIT, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(obj_idx);
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* IO */
int TxSDO(int MD_idx)
{
    return userCtrlObj[MD_idx].tx_fnc(userCtrlObj[MD_idx].sdo_tx_id, sdo_msg.txBuf, sdo_msg.msgLength);
}

int TxPDO(int MD_idx)
{
    return userCtrlObj[MD_idx].tx_fnc(userCtrlObj[MD_idx].pdo_tx_id, pdo_msg.txBuf, pdo_msg.msgLength);
}

static void SetRoutine(AS_TaskData_t* taskObj, uint8_t routine_id)
{
    uint8_t idx = taskObj->n_routines;

    if (idx > 0) {
        for (int i = 0; i < idx; ++i) { // Check if the routine is already set
            if (taskObj->routines[i] == routine_id) {
                return;
            }
        }
    }

    if (idx < AS_DEV_MAX_ROUTINES) {
        taskObj->routines[idx] = routine_id;
        ++taskObj->n_routines;
    }
}

static void ClearRoutine(AS_TaskData_t* taskObj, uint8_t routine_id)
{
    for (int i = 0; i < AS_DEV_MAX_ROUTINES; ++i) {
        if (taskObj->routines[i] == routine_id) {
            taskObj->routines[i] = 0;
            --taskObj->n_routines;
            return;
        }
    }
}

//static void ClearRoutine(AS_TaskData_t* taskObj)
//{
//	memset(&taskObj->routines, 0, sizeof(taskObj->routines[0])*AS_DEV_MAX_ROUTINES);
//	taskObj->n_routines = 0;
//}

static void StandbyStates(int DevIdx)
{
    DOPI_SDOUnit_t sdoUnit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[DevIdx].data.low_level_ctrl_task.state = TASK_STATE_STANDBY;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.mid_level_ctrl_task.state = TASK_STATE_STANDBY;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.gait_ctrl_task.state = TASK_STATE_STANDBY;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.msg_hdlr_task.state = TASK_STATE_STANDBY;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

static void EnableStates(int DevIdx)
{
    DOPI_SDOUnit_t sdoUnit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[DevIdx].data.low_level_ctrl_task.state = TASK_STATE_ENABLE;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.mid_level_ctrl_task.state = TASK_STATE_ENABLE;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.gait_ctrl_task.state = TASK_STATE_ENABLE;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.msg_hdlr_task.state = TASK_STATE_ENABLE;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

static void OffStates(int DevIdx)
{
    DOPI_SDOUnit_t sdoUnit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[DevIdx].data.low_level_ctrl_task.state = TASK_STATE_OFF;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.mid_level_ctrl_task.state = TASK_STATE_OFF;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.gait_ctrl_task.state = TASK_STATE_OFF;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.msg_hdlr_task.state = TASK_STATE_OFF;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

/**
 * @brief Set the address of SDO/PDO which you want to use
 * @param MD Object
 */
static void SetupDOD(AS_CtrlObj_t* obj)
{
	// /* LOW LEVEL */
    // SDO
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE,			&obj->data.low_level_ctrl_task.state);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_ROUTINE,		obj->data.low_level_ctrl_task.routines);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_AUX_INPUT,		&obj->data.motor_auxiliary_input);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT,		&obj->data.joint_limit_sw);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ENABLE_TORQUE_MEASURE,		&obj->data.Torque_measure_sw);
	
	// PDO
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT,	&obj->data.currentRef);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_CURRENT_OUTPUT,  		&obj->data.currentAct);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_VOLTAGE_SCALING,		&obj->data.volt_scaling);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_ACTUAL_VOLTAGE,			&obj->data.actual_volt);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_ACTUAL_CURRENT,  		&obj->data.actual_curr);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_POSITION,  				&obj->data.inc_enc);

    // /* MID LEVEL */
    // SDO
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE,			&obj->data.mid_level_ctrl_task.state);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE,		obj->data.mid_level_ctrl_task.routines);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX,	&obj->data.md_f_vector.mode_idx);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_TMAX,		&obj->data.md_f_vector.tau_max);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_DELAY,		&obj->data.md_f_vector.delay);
	//DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_ZERO,		&obj->data.f_vector.zero);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_GRAV_COMP_GAIN,		&obj->data.gravCompGain);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VEL_COMP_GAIN,		&obj->data.velCompGain);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_SUIT_MODE,		&obj->data.suit_mode);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VEL_COMP_SATU,		&obj->data.velCompSatu);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_UPPER_LIMIT,	&obj->data.HipRoMUpperLimit);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_LOWER_LIMIT,	&obj->data.HipRoMLowerLimit);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_UPPER_VEL_LIMIT,		&obj->data.HipUpperVelLimit);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_LOWER_VEL_LIMIT,		&obj->data.HipLowerVelLimit);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FSM_CURR,			&obj->data.FSM_curr);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F,			&obj->data.yd_f_dummy);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_SITTOSTANCE_TOQUE,			&obj->data.SitToStance_Torque);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STANCETOSIT_TOQUE,			&obj->data.StanceToSit_Torque);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_YD,		&obj->data.p_vector.yd);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_L,			&obj->data.p_vector.L);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_S0,		&obj->data.p_vector.s0);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_SD,		&obj->data.p_vector.sd);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_EPSILON,	&obj->data.i_vector.epsilon_target);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP,		&obj->data.i_vector.Kp_target);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD,		&obj->data.i_vector.Kd_target);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_LAMBDA,	&obj->data.i_vector.lambda_target);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_DURATION,	&obj->data.i_vector.duration);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP_MAX,	&obj->data.i_vector_Kp_max);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD_MAX,	&obj->data.i_vector_Kd_max);

    // PDO
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW,  		&obj->data.inc_enc_velo);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_ABSENCODER1_POSITION,  		&obj->data.abs1_enc);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_ABSENCODER2_POSITION,  		&obj->data.abs2_enc);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_ABSENCODER1_VELOCITY,  		&obj->data.abs1_enc_velo);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_ABSENCODER2_VELOCITY,  		&obj->data.abs2_enc_velo);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_AUXILIARY_INPUT,  		&obj->data.motor_auxiliary_input);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_REF_POSITION,  			&obj->data.p_vector_ref);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_IMP_EPSILON,  			&obj->data.i_vector_epsilon);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_IMP_KP,  				&obj->data.i_vector_kP);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL,	PDO_ID_MIDLEVEL_IMP_LAMDA,  			&obj->data.i_vector_lambda);

	// /* Msg Hdlr */
    // SDO
    DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE,	&obj->data.msg_hdlr_task.state);
    DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MSG, SDO_ID_MSG_SET_ROUTINE,	obj->data.msg_hdlr_task.routines);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST,		obj->data.pdo_list);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MSG, SDO_ID_MSG_SDO_LIST,		obj->data.sdo_list);

	// PDO

	// /* Gait Ctrl */
    // SDO
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_STATE,			&obj->data.gait_ctrl_task.state);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_ROUTINE,		obj->data.gait_ctrl_task.routines);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_IS_NEUTRALIZED,		&obj->data.isNeutralized);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_NEUTRAL_BIAS_CMD,	&obj->data.neutralBiasCmd);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_NEUTRAL_BIAS_OFFSET,	&obj->data.thighNeutralPostureOffset);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_USER_NEUTRAL_POSTURE_ANGLE,	&obj->data.UserNeutralPostureAngle);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_AUTO_CALIB_SENSOR,		&obj->data.autoCalibSensorCmd);

	// PDO
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_THIGH_ANGLE_DEG,	&obj->data.thighAngleDeg);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_THIGH_VEL_DEG,		&obj->data.thighVelDeg);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_INC_POS_DEG,		&obj->data.incActualPosDeg);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_INC_VEL_DEG,		&obj->data.incActualVelDeg);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_IMU_VQF_SAGITTAL,		&obj->data.IMU_BodyAngle_Sagittal);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_IMU_VQF_FRONTAL,		&obj->data.IMU_BodyAngle_Frontal);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_IMU_VQF_TRANSVERSE,		&obj->data.IMU_BodyAngle_Transverse);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_X,				&obj->data.accXRaw);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Y,				&obj->data.accYRaw);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Z,				&obj->data.accZRaw);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_X,				&obj->data.gyrXRaw);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Y,				&obj->data.gyrYRaw);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Z,				&obj->data.gyrZRaw);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_X_CALIB,			&obj->data.accXCalib);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Y_CALIB,			&obj->data.accYCalib);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Z_CALIB,			&obj->data.accZCalib);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_X_CALIB,			&obj->data.gyrXCalib);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Y_CALIB,			&obj->data.gyrYCalib);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Z_CALIB,			&obj->data.gyrZCalib);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_X_GLOBAL,			&obj->data.accXGlobal);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Y_GLOBAL,			&obj->data.accYGlobal);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Z_GLOBAL,			&obj->data.accZGlobal);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_X_GLOBAL,			&obj->data.gyrXGlobal);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Y_GLOBAL,			&obj->data.gyrYGlobal);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Z_GLOBAL,			&obj->data.gyrZGlobal);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_WC_TVCF_CALIB,			&obj->data.wcTVCF_calib);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_ACC_MEAN_VALUE,				&obj->data.accMeanVal_calibrated);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYRO_SCALING,				&obj->data.scaling_gyro);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYRO_BIAS,					&obj->data.bias_gyro);

    // /* Imu Ctrl */
    // SDO
    // PDO

    /* System Mngt */
	// SDO
	// PDO

    /* Ext Dev Ctrl */
	// SDO
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE,		&obj->data.ext_dev_ctrl_task.state);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_ROUTINE,	obj->data.ext_dev_ctrl_task.routines);

	// PDO
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_EXTDEV,	PDO_ID_EXTDEV_FSR_MIDDLE_TOE,	&obj->data.fsr_middleToe);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_EXTDEV,	PDO_ID_EXTDEV_FSR_BIG_TOE,		&obj->data.fsr_bigToe);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_EXTDEV,	PDO_ID_EXTDEV_FSR_LITTLE_TOE,	&obj->data.fsr_littleToe);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_EXTDEV,	PDO_ID_EXTDEV_FSR_HEEL,			&obj->data.fsr_heel);

	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_EXTDEV,	PDO_ID_EXTDEV_NTC_MOTOR_TEMP,	&obj->data.motor_temp);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_EXTDEV,	PDO_ID_EXTDEV_MCU_TEMP,			&obj->data.motor_mcutemp);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_EXTDEV,	PDO_ID_EXTDEV_MOTOR_TEMP_SCALING,	&obj->data.motor_temp_scaling);
}

// uint8_t testRxData[128];
/**
 * @brief FDCAN RxCallback function which executes when the ori_node is matched
 * @param wasp_id = 11bit identifier of FDCAN, rx_data = data packet
 */
static int AS_FDCAN_CB(uint16_t wasp_id, uint8_t* rx_data)
{
    uint16_t fncCode = wasp_id & 0x700;
    uint16_t origin_node_id = (wasp_id & 0x0F0)>>4;
    static uint8_t buffer_cnt = 0;
    // memcpy(testRxData, rx_data, 128);

    uint16_t tempDevIdx = 0;
    for (int DevIdx = DEV_IDX_MD1; DevIdx < DEV_IDX_MD16; DevIdx++) {
		uint8_t tUsage = RS_File.vec[DevIdx].usage;

		if (tUsage == 1) {
			if (RS_File.vec[DevIdx].FDCAN_ID == origin_node_id) {
				tempDevIdx = DevIdx;
				break;
			}
		}
    }
	
	/*
     * 확장보드에서 보낸 메시지인지 확인 (가장 먼저 처리)
     * - 확장보드의 Node ID는 NODE_ID_EXTPACK(2)로 약속합니다.
     */
	if (origin_node_id == NODE_ID_EXTPACK) { // Extension Board node ID
		// 메시지를 확장보드 전용 처리 모듈로 전달
        ExtLinkMngr_ProcessCANMessage(wasp_id, rx_data); // len은 실제 값으로 수정 필요  
	} else {
		switch(fncCode) {
			case EMCY:
				DOPI_UnpackEMCY(&userCtrlObj[tempDevIdx].data.err_code, rx_data);
				if (buffer_cnt == RM_MD_ERROR_ARRAY) buffer_cnt = 0;
				mdErrorBit[buffer_cnt].device_id = tempDevIdx;
				mdErrorBit[buffer_cnt].error_bit = userCtrlObj[tempDevIdx].data.err_code;
				md_error_flag = true;
				buffer_cnt ++;
				break;
			case SDO: DOPI_UnpackSDO(&userCtrlObj[tempDevIdx].devObj, rx_data); break;
			case FDCAN_CHECK: Check_FD_CAN_COMMU_RX(rx_data, origin_node_id); break;
			case PDO: DOPI_UnpackPDO(&userCtrlObj[tempDevIdx].devObj, rx_data); break;
			case NOTI: DOPI_UnpackNOTI(&userCtrlObj[tempDevIdx].devObj, rx_data); break;
			default: break;
		}
	}
    return 0;
}

/*Risk Mngr*/
static bool ParsingMDErrorCode(uint8_t buffer_cnt, FaultLocationMD_t md_task, uint32_t error_bit)
{
	return ((((0xE0000000) & mdErrorBit[buffer_cnt].error_bit) == (md_task << RM_MD_FAULT_BIT)) &&  (((0x0FFFFFFF) & mdErrorBit[buffer_cnt].error_bit) == ((uint32_t)1U << error_bit)));
}

void FlagMDFault(uint8_t buffer_cnt)
{

	if(ParsingMDErrorCode(buffer_cnt, LOWLEVEL, CURRENT_OFFSET_ERROR_PHASE_A))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,0};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_HIGH, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

	if(ParsingMDErrorCode(buffer_cnt, LOWLEVEL, CURRENT_OFFSET_ERROR_PHASE_B))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,1};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_HIGH, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

	if(ParsingMDErrorCode(buffer_cnt, LOWLEVEL, CURRENT_OFFSET_ERROR_PHASE_C))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,2};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_HIGH, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

	if(ParsingMDErrorCode(buffer_cnt, LOWLEVEL, OVER_CURRENT_HW_FAULT))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,3};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

	if(ParsingMDErrorCode(buffer_cnt, LOWLEVEL, OVER_CURRENT_SW_FAULT))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,4};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

	if(ParsingMDErrorCode(buffer_cnt, LOWLEVEL, OVER_VOLTAGE_VDC))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,5};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

	if(ParsingMDErrorCode(buffer_cnt, LOWLEVEL, UNDER_VOLTAGE_VDC))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,6};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_MID, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

	if(ParsingMDErrorCode(buffer_cnt, SYSMNGT, OVER_TEMPERATURE_MOTOR))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,7};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

	if(ParsingMDErrorCode(buffer_cnt, WARNING, OVER_TEMPERATURE_MOTOR))
	{
		FaultArrange_t ID = {mdErrorBit[buffer_cnt].device_id,8};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_MID, FaultIDRegister, &ID, sizeof(ID));
		//flag 기록
	}

}

#endif /* SUIT_MINICM_ENABLED */
