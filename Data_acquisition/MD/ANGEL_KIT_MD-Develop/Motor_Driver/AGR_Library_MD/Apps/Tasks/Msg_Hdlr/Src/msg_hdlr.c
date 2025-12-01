/**
 * @file msgHdlrTask.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

// TODO : refactoring!!
#include "msg_hdlr.h"

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

MainSequence_Enum MS_enum;
TaskObj_t msgHdlrTask;
uint8_t MD_nodeID;
uint8_t NOTI_sw;
uint32_t NOTI_data;
uint8_t CM_nodeID=1;
uint8_t test_sw;
uint8_t test_BL=0;

uint8_t dh_test[10]= "1234567890";

IOIF_GPIOPinState_t stategpio;

uint32_t MD_fw_binary_size = 0;

uint32_t FW_Update, FW_Backup;
uint32_t MD_EOT_ACK_Flag = 0;



/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

volatile uint32_t MDUpdateFlag __attribute__((section(".MDUpdateSwitch")));
//volatile uint32_t MDFWBinarySize __attribute__((section(".MDFWBinSize")));

static cvector_vector_type(DOP_Header_t) pdo_send_list;
static cvector_vector_type(DOP_Header_t) sdo_req_list;
static cvector_vector_type(DOP_Header_t) sdo_res_list;

static DOPI_TrajBuff_t trajectory_buffer;
static COMMType comm_type;

static uint8_t GUI_onoff;
static uint8_t GUI_command;

static uint8_t ori_node;
static uint8_t dest_node;
static uint32_t fnc_code;
static uint32_t err_code;
static uint8_t fdcanTxData[128];
static uint8_t fdcanRxData[64];
static uint8_t usbRxData[64];
static int comm_loop_cnt;

static uint8_t fdcan_test_run_start =0;

static int32_t test_dummy[10];

static float msgTimeElap;

extern uint8_t safety_sw;
extern uint8_t safety_timer_flag;
extern float safety_timer;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateStandby_Ent();
static void StateStandby_Run();
static void StateStandby_Ext();

static void StateEnable_Ent();
static void StateEnable_Run();
static void StateEnable_Ext();

static void StateError_Run();

/* ------------------- READ NODE ID ------------------- */
static uint8_t Read_Node_ID();

/* ------------------- CONVERT BYTE TO LENGTH ------------------- */
static DOP_Header_t Get_Header(uint8_t* t_byte_arr);

/* ------------------- EMCY ------------------- */
void Send_EMCY(uint32_t* t_err_code);
static void Recv_EMCY(uint8_t* t_byte_arr, uint32_t* t_err_code);

/* ------------------- SDO RX ------------------- */
static DOP_SDOArgs_t Convert_Bytes_to_SDO_req(uint8_t* t_byte_arr, uint16_t *t_byte_len);
static int Read_SDO(uint8_t* t_byte_arr);
static int Unpack_SDO(uint8_t* t_byte_arr);

/* ------------------- SDO TX ------------------- */
static int Convert_SDOres_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr);
static int Pack_SDO(uint8_t* t_byte_arr, uint8_t* t_byte_len);
static int Send_SDO(uint8_t t_dest_node);

/* ------------------- PDO RX ------------------- */
static int Convert_Bytes_to_PDO(uint8_t* t_byte_arr);
static int Unpack_PDO(uint8_t* t_byte_arr);

/* ------------------- PDO TX ------------------- */
static int Convert_PDO_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr);
static int Pack_PDO(uint8_t* t_byte_arr, uint8_t* t_byte_len);

/* ------------------- PDO TX Routine ------------------- */
static int Run_Send_PDO();
static int Ext_Send_PDO();
static int Set_PDO_Dummy();

/* ------------------- MANUFACTURE ------------------- */
int Check_FD_CAN_COMMU(uint8_t* t_byte_arr);
uint16_t Checksum_Calcul(uint8_t* t_test_packet, uint8_t t_length );
uint8_t Checksum_Check(uint8_t* t_test_packet, uint8_t t_length, uint16_t * t_checksum);


/* ------------------- TRAJECTORY ------------------- */
static int Check_Trajectory_Error(uint16_t t_frameIDX);
static int Assemble_Trajectory(uint8_t* t_byte_arr);

/* ------------------- LIST MANAGEMENT ------------------- */
static void Add_PDO_to_Send(uint8_t t_dictID, uint8_t t_objID);
static void Send_USB_Trick(uint8_t* t_in_buf, uint32_t t_in_len, uint8_t* t_out_buf);
static void Add_SDO_to_Send(uint8_t t_dictID, uint8_t t_objID);
static void Clear_SDO_to_Send();

/* ------------------- MSG HANDLER ------------------- */
static int USB_Rx_Hdlr(uint8_t* t_Buf, uint32_t* t_Len);
static int Fdcan_Rx_Hdlr(uint16_t t_wasp_id, uint8_t* t_rx_data);

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Send_PDO_List(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Send_SDO_List(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MS_Enum(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

/* ------------------- SDO CALLBACK(GUI) ------------------- */
static void Set_GUI_COMM_OnOff(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_GUI_COMM_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_GUINodeID(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_FDCAN_Test_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Send_FLASH_STATE(void);
static void Send_SAFETY_SW(void);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void SoftwareSystemReset(void);

DOP_COMMON_SDO_CB(msgHdlrTask)

void InitMsgHdlr()
{
    /*Task Init*/
    InitTask(&msgHdlrTask);

    MD_nodeID = Read_Node_ID();
    ori_node = 0x00;

//    MDFWBinarySize = MD_fw_binary_size;

    // TODO : Error Handle!
#ifndef _USE_DEBUG_CLI
    IOIF_InitFDCAN1(MD_nodeID);
    IOIF_InitUSB(IOIF_USBD_CDC, IOIF_TIM6);
#endif /* !_USE_DEBUG_CLI */

	/* State Definition */
	TASK_CREATE_STATE(&msgHdlrTask, TASK_STATE_OFF,      NULL,                  NULL,    			NULL,               false);
	TASK_CREATE_STATE(&msgHdlrTask, TASK_STATE_STANDBY,  StateStandby_Ent,      StateStandby_Run,	StateStandby_Ext,   true);
	TASK_CREATE_STATE(&msgHdlrTask, TASK_STATE_ENABLE,   StateEnable_Ent,       StateEnable_Run, 	StateEnable_Ext,   	false);
	TASK_CREATE_STATE(&msgHdlrTask, TASK_STATE_ERROR,    NULL,                  StateError_Run,     NULL,   	        false);

    TASK_CREATE_ROUTINE(&msgHdlrTask,  ROUTINE_ID_MSG_PDO_SEND,           NULL,   Run_Send_PDO,   Ext_Send_PDO);
    TASK_CREATE_ROUTINE(&msgHdlrTask,  ROUTINE_ID_MSG_PDO_DUMMY_TEST,     NULL,   Set_PDO_Dummy,  NULL);

	/* Data Object Definition */
    DOP_CreateDOD(TASK_ID_MSG);

    DOP_COMMON_SDO_CREATE(TASK_ID_MSG)
    DOP_CreateSDO(TASK_ID_MSG, SDO_ID_MSG_PDO_LIST,             DOP_UINT16, Set_Send_PDO_List);
    DOP_CreateSDO(TASK_ID_MSG, SDO_ID_MSG_MS_ENUM,              DOP_UINT8,  Set_MS_Enum);
    DOP_CreateSDO(TASK_ID_MSG, SDO_ID_MSG_GUI_COMM_ONOFF,  		DOP_UINT8,  Set_GUI_COMM_OnOff);
    DOP_CreateSDO(TASK_ID_MSG, SDO_ID_MSG_GUI_COMM_COMMAND,     DOP_UINT8,  Set_GUI_COMM_Command);

    DOP_CreateSDO(TASK_ID_MSG, SDO_ID_MSG_READ_NODE_ID,         DOP_UINT8,  Set_GUINodeID);
    DOP_CreateSDO(TASK_ID_MSG, SDO_ID_MSG_SDO_LIST,             DOP_UINT16, Set_Send_SDO_List);

    DOP_CreateSDO(TASK_ID_MSG, SDO_ID_MSG_SET_FDCAN_TEST_SWITCH,     DOP_UINT32,  Set_FDCAN_Test_Command);

	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST1,    DOP_INT32, 1, &test_dummy[0]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST2,    DOP_INT32, 1, &test_dummy[1]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST3,    DOP_INT32, 1, &test_dummy[2]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST4,    DOP_INT32, 1, &test_dummy[3]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST5,    DOP_INT32, 1, &test_dummy[4]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST6,    DOP_INT32, 1, &test_dummy[5]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST7,    DOP_INT32, 1, &test_dummy[6]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST8,    DOP_INT32, 1, &test_dummy[7]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST9,    DOP_INT32, 1, &test_dummy[8]);
	DOP_CreatePDO(TASK_ID_MSG, PDO_ID_MSG_TEST10,   DOP_INT32, 1, &test_dummy[9]);

	DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_RH_NeutralPosture,	&filteredAngleDataObj_Sagittal.NeutralPostureBias,		DOP_FLOAT32);
	DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_LH_NeutralPosture,	&filteredAngleDataObj_Sagittal.NeutralPostureBias,		DOP_FLOAT32);
	DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_RK_NeutralPosture,	&filteredAngleDataObj_Sagittal.NeutralPostureBias_KneeEncoder,	DOP_FLOAT32);
	DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_LK_NeutralPosture,	&filteredAngleDataObj_Sagittal.NeutralPostureBias_KneeEncoder,	DOP_FLOAT32);
    
	DOP_SetSDOReq(TASK_ID_SYSMNGT_CM,	SDO_ID_SYSMNGT_CM_FLASH_WRITE_NOTI,	&MS_enum,	DOP_UINT8);
	DOP_SetSDOReq(TASK_ID_MIDLEVEL,		SDO_ID_MIDLEVEL_SAFETY_SW,			&safety_sw,	DOP_UINT8);

    DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_RH_P_VECTOR_DURATION_COMPLETED, &pvectorObj.durationCompleted, DOP_UINT8);
    DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_LH_P_VECTOR_DURATION_COMPLETED, &pvectorObj.durationCompleted, DOP_UINT8);
    DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_RK_P_VECTOR_DURATION_COMPLETED, &pvectorObj.durationCompleted, DOP_UINT8);
    DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_LK_P_VECTOR_DURATION_COMPLETED, &pvectorObj.durationCompleted, DOP_UINT8);

    DOP_SetSDOReq(TASK_ID_EXTDEV,  SDO_ID_EXTDEV_ROBOWEAR_MONITOR, &stategpio, DOP_UINT8);

    DOP_SetSDOReq(TASK_ID_WHOLEBODY,  SDO_ID_WHOLEBODY_ACC_MEAN_VALUE, &flashsensorObj.accMeanVal_calibrated, DOP_FLOAT32);

    /* Communication Init */
    if (IOIF_IsUSBD_connected() == true) {
        comm_type = COMM_TYPE_USB;
    } else {
        comm_type = COMM_TYPE_FDCAN; // Must be Change if use USB
    }

	/* Callback Allocation */
	if (comm_type == COMM_TYPE_FDCAN) {
		#ifndef _USE_DEBUG_CLI
	    IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, Fdcan_Rx_Hdlr);
		#endif /* !_USE_DEBUG_CLI */
	} else if (comm_type == COMM_TYPE_USB) {
	    ioif_usbRxCBPtr = USB_Rx_Hdlr;
	}

	/* Timer 6 Callback Allocation */
	if (IOIF_StartTimIT(IOIF_TIM6) > 0) {
		//TODO: ERROR PROCESS
	}

	IOIF_SetTimCB(IOIF_TIM6, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunMsgHdlr, NULL);
}

void RunMsgHdlr(void* params)
{	
    /* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Run Device */
	RunTask(&msgHdlrTask);

    /* Check FDCAN or USB Connect */
    if (IOIF_IsUSBD_connected() == true) {
        comm_type = COMM_TYPE_USB;
        ioif_usbRxCBPtr = USB_Rx_Hdlr;
#ifndef _USE_DEBUG_CLI
        IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, NULL);
#endif /* !_USE_DEBUG_CLI */
    } else {
        comm_type = COMM_TYPE_FDCAN;
#ifndef _USE_DEBUG_CLI
        IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, Fdcan_Rx_Hdlr);
#endif /* !_USE_DEBUG_CLI */
        ioif_usbRxCBPtr = NULL;
    }

	/* Elapsed Time Check */
	msgTimeElap = DWT->CYCCNT/480;	// in microsecond
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateStandby_Ent()
{

}

static void StateStandby_Run()
{
	Send_FLASH_STATE();
	Send_SAFETY_SW();
	RoboWear_MonitorGPIOPin();
	  if(test_BL==1){
	        IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13);  // Toggle LED
	     	MDUpdateFlag = 1;
	     	SoftwareSystemReset();
	    }
}

static void StateStandby_Ext()
{

}

static void StateEnable_Ent()
{
	comm_loop_cnt = 0;
	EntRoutines(&msgHdlrTask.routine);
}

static void StateEnable_Run()
{
	RoboWear_MonitorGPIOPin();
	if(test_sw==1)
	{
//		dh_test = (uint8_t)(comm_loop_cnt%64);
		Clear_SDO_to_Send();
		Add_SDO_to_Send(TASK_ID_SYSMNGT_CM, SDO_ID_SYSMNGT_CM_FLASH_WRITE_NOTI);
		Send_SDO(NODE_ID_CM);
		test_sw=0;
	}
	Send_FLASH_STATE();
	Send_SAFETY_SW();
	RunRoutines(&msgHdlrTask.routine);
    comm_loop_cnt++;
}

static void StateEnable_Ext()
{
	ExtRoutines(&msgHdlrTask.routine);

//	GUI_onoff = 0;
//	GUI_command = 0;
}

static void StateError_Run()
{
	Send_FLASH_STATE();
	Send_SAFETY_SW();
}

void SoftwareSystemReset(void) {
    __disable_irq();  // Disable all interrupts
    NVIC_SystemReset();  // Perform system reset
}

/* ------------------- READ NODE ID ------------------- */
static uint8_t Read_Node_ID()
{
    uint8_t temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;
    
#if defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
    temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_8);
    temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_9);
    temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_10);
    temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_11);
#endif

#if defined(SUIT_MD_ENABLED)
    temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_2);
    temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_3);
    temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_4);
    temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_5);
#endif

    return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
}

/* ------------------- CONVERT BYTE TO LENGTH ------------------- */

static DOP_Header_t Get_Header(uint8_t* t_byte_arr)
{
	DOP_Header_t t_header = {0};
    memcpy(&t_header, t_byte_arr, sizeof(DOP_Header_t));
    return t_header;
}

/* ------------------- EMCY ------------------- */
void Send_EMCY(uint32_t* t_err_code)
{
    uint8_t* t_tx_data = malloc(sizeof(uint8_t)*4);
    uint16_t t_identifier = EMCY|(MD_nodeID<<4);

    memcpy(t_tx_data, t_err_code, DOP_ERR_CODE_SIZE);

    if(Send_MSG(t_identifier, t_tx_data, 4) != 0){
        //TODO: MSG TX ERROR
    }

    free(t_tx_data);
    t_tx_data = NULL;
}

static void Recv_EMCY(uint8_t* t_byte_arr, uint32_t* t_err_code)
{
    memcpy(t_err_code, t_byte_arr, DOP_ERR_CODE_SIZE);
}

/* ------------------- NOTIFICATION ------------------- */
void Send_NOTI(uint32_t data)
{
	uint8_t t_dest_node = NODE_ID_CM;
    uint8_t t_tx_data[4];
    uint16_t t_identifier = NOTI|(MD_nodeID<<4)|t_dest_node;

    memcpy(t_tx_data, &data, 4);

    if(Send_MSG(t_identifier, t_tx_data, 4) != 0){
        //TODO: MSG err
    }
}

static void Send_FLASH_STATE(void)
{
	static uint8_t prev_MS_enum = 0;

	if(prev_MS_enum != MS_enum)
	{
		Clear_SDO_to_Send();
		Add_SDO_to_Send(TASK_ID_SYSMNGT_CM, SDO_ID_SYSMNGT_CM_FLASH_WRITE_NOTI);
		Send_SDO(NODE_ID_CM);
	}

	prev_MS_enum = MS_enum;
}

static void Send_SAFETY_SW(void)
{
	if(safety_sw == 1)
	{
		Clear_SDO_to_Send();
		Add_SDO_to_Send(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SAFETY_SW);
		Send_SDO(NODE_ID_CM);
		safety_sw = 0;
		safety_timer_flag = 1;
	}
}

/* ------------------- SDO RX ------------------- */
static DOP_SDOArgs_t Convert_Bytes_to_SDO_req(uint8_t* t_byte_arr, uint16_t *t_byte_len)
{
	DOP_SDOArgs_t t_req = {0};
    *t_byte_len = 0;

    int t_idx = sizeof(t_req.status);
    int t_len = sizeof(t_req.dataSize);

    memcpy(&t_req.dataSize, &t_byte_arr[t_idx], t_len);
    *t_byte_len += t_len;

    t_req.data = &t_byte_arr[t_idx + t_len];

    t_req.status = t_byte_arr[0];
    *t_byte_len += 1;

    return t_req;
}

static int Read_SDO(uint8_t* t_byte_arr)
{
    int t_byte_read = 0;
    DOP_Header_t t_header = Get_Header(t_byte_arr);
    t_byte_read += sizeof(DOP_Header_t);

    DOP_SDO_t* t_sdo = DOP_FindSDO(t_header.dictID, t_header.objID);
    if (t_sdo == NULL) {
        //TODO: Cannot Find SDO ERROR
        return -2;
    }

    uint16_t t_req_bytes = 0;
    DOP_SDOArgs_t t_req = Convert_Bytes_to_SDO_req(t_byte_arr + t_byte_read, &t_req_bytes);
    t_req.typeSize = t_sdo->args.typeSize; // Copy SDO info
    t_byte_read += t_req_bytes;

    uint16_t t_n_bytes = 0;
    if (t_req.status == DOP_SDO_REQU) {
    	t_n_bytes = DOP_CallSDO(t_sdo, &t_req);
        cvector_push_back(sdo_res_list, t_header); // Assign Response
    } else if(t_req.status == DOP_SDO_SUCC || t_req.status == DOP_SDO_FAIL) {
    	t_n_bytes = DOP_SetSDOArgs(t_sdo, &t_req);
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

static int Unpack_SDO(uint8_t* t_byte_arr)
{
    int t_cursor = 0;

    // Get # of SDOs
    uint16_t t_numOfSDO = 0;
    memcpy(&t_numOfSDO, &t_byte_arr[t_cursor], DOP_OBJ_NUMS_SIZE);
    t_cursor += DOP_OBJ_NUMS_SIZE;

    // Call & Respond SDOs
    if (t_numOfSDO > 0) {
        for (int i = 0; i < t_numOfSDO; ++i) {
            int temp_cursor = Read_SDO(&t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
            } else if (temp_cursor < 0) {
                //TODO: Unpack SDO ERROR
                return DOP_SDO_FAULT;
            }
        }
    }

    return DOP_SUCCESS;
}

/* ------------------- SDO TX ------------------- */
static int Convert_SDOres_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr)
{
    int t_byte_written = 0;
    // Set SDO Header
    memcpy(t_byte_arr, t_header, sizeof(DOP_Header_t));
    t_byte_written += sizeof(DOP_Header_t);

    // Return Response
    DOP_SDO_t* t_sdo = DOP_FindSDO(t_header->dictID, t_header->objID);
    if (t_sdo == NULL) {
        //TODO: Cannot Find SDO ERROR
        return -2;
    }

    memcpy(t_byte_arr + t_byte_written, &t_sdo->args.status, sizeof(t_sdo->args.status));
    t_byte_written += sizeof(t_sdo->args.status);
    memcpy(t_byte_arr + t_byte_written, &t_sdo->args.dataSize,   sizeof(t_sdo->args.dataSize));
    t_byte_written += sizeof(t_sdo->args.dataSize);

    int t_data_len = t_sdo->args.dataSize * t_sdo->args.typeSize;
    memcpy(t_byte_arr + t_byte_written, t_sdo->args.data, t_data_len);

    t_byte_written += t_data_len;

    return t_byte_written;
}

static int Pack_SDO(uint8_t* t_byte_arr, uint8_t* t_byte_len)
{
	// check send list whether these are empty or not 
	if ((sdo_res_list == NULL) && (sdo_req_list == NULL)){
		return DOP_SDO_NOTHING;
	}

	// Message Packaging
    int t_cursor = 0;

    // Res SDOs
    int t_numOfSDO_cursor = t_cursor;
    t_cursor += DOP_OBJ_NUMS_SIZE;

    uint8_t t_numOfSDO = 0;
    
    if (sdo_res_list != NULL) {
        for(int i = 0; i < cvector_size(sdo_res_list); ++i) {
            int temp_cursor = Convert_SDOres_to_Bytes(&sdo_res_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
                ++t_numOfSDO;
            } else if (temp_cursor < 0) {
                //TODO: Pack Response SDO Error
                return DOP_SDO_FAULT;
            }
        }
        cvector_free(sdo_res_list);
        sdo_res_list = NULL;
    }

    // Req SDOs
    if (sdo_req_list != NULL) {
        for(int i = 0; i < cvector_size(sdo_req_list); ++i) {
            int temp_cursor = Convert_SDOres_to_Bytes(&sdo_req_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
                ++t_numOfSDO;
            } else if (temp_cursor < 0) {
                //TODO: Pack Request SDO Error
                return DOP_SDO_FAULT;
            }
        }
        cvector_free(sdo_req_list);
        sdo_req_list = NULL;
    }

    // Set # of SDOs
    memcpy(&t_byte_arr[t_numOfSDO_cursor], &t_numOfSDO, DOP_OBJ_NUMS_SIZE);

    *t_byte_len = t_cursor;

    return DOP_SUCCESS;
}

static int Send_SDO(uint8_t t_dest_node)
{
    uint8_t t_byte_len = 0;
    uint16_t t_identifier = SDO|(MD_nodeID<<4)|t_dest_node;

    int t_check = Pack_SDO(fdcanRxData, &t_byte_len);

    if(t_check < 0){
        //TODO: Send SDO Error
    	return t_check;
    } else if(t_check){
    	return t_check;
    }

    if (t_byte_len > 64) {
        //TODO: TX MESSAGE TOO LONG ERROR 
    }

    if(Send_MSG(t_identifier, fdcanRxData, t_byte_len) != 0){
        return t_check;
        //TODO: MSG TX ERROR
    }

    return t_check;
}

/* ------------------- PDO RX ------------------- */
static int Convert_Bytes_to_PDO(uint8_t* t_byte_arr)
{
    int t_byte_read = 0;

    DOP_Header_t t_header = Get_Header(t_byte_arr);
    t_byte_read += sizeof(DOP_Header_t);

    DOP_PDO_t* t_pdo = DOP_FindPDO(t_header.dictID, t_header.objID);
    if (t_pdo == NULL) {
        //TODO: Cannot Find PDO Error
        return -2;
    }

    uint16_t t_n_bytes = DOP_GetPDO(t_pdo, (void*)(t_byte_arr + t_byte_read));
    if (t_n_bytes < 0) {
        //TODO: Copy PDO to Receive Error
        return -1;
    }
    t_byte_read += t_n_bytes;

    return t_byte_read;
}

static int Unpack_PDO(uint8_t* t_byte_arr)
{
    int t_cursor = 0;

    // Get # of PDOs
    uint8_t t_numOfPDO = 0;
    memcpy(&t_numOfPDO, &t_byte_arr[t_cursor], DOP_OBJ_NUMS_SIZE);
    t_cursor += DOP_OBJ_NUMS_SIZE;

    if (t_numOfPDO > 0) {
        for (int i = 0; i < t_numOfPDO; ++i) {
            int temp_cursor = Convert_Bytes_to_PDO(&t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
            } else if (temp_cursor < 0) {
                //TODO: Unpack PDO Error
                return DOP_PDO_FAULT;
            }
        }
    }

    return DOP_SUCCESS;
}

/* ------------------- PDO TX ------------------- */
static int Convert_PDO_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr)
{
    int t_header_size = sizeof(DOP_Header_t);
    // Publish PDO
    DOP_PDO_t* t_pdo = DOP_FindPDO(t_header->dictID, t_header->objID);
    if (t_pdo == NULL) {
        //TODO: Cannot Find PDO
        return -2;
    }

    uint16_t t_n_bytes = DOP_SetPDO(t_pdo, t_byte_arr + t_header_size);
    if (t_n_bytes < 0) {
        //TODO: Copy PDO to Send 
        return -1;
    } else if (t_n_bytes == 0) { // Nothing to publish
        return 0;
    }

    memcpy(t_byte_arr, t_header, t_header_size);
    return t_header_size + t_n_bytes;
}

static int Pack_PDO(uint8_t* t_byte_arr, uint8_t* t_byte_len)
{
	// check send list whether these are empty or not 
    if (pdo_send_list == NULL){
        return 0;
    }

    int t_cursor = 0;

    // Pub PDO
    int t_numOfPDO_cursor = t_cursor;
    t_cursor += DOP_OBJ_NUMS_SIZE;

    uint8_t t_numOfPDO = 0;

    if (pdo_send_list != NULL) {
        for(int i = 0; i < cvector_size(pdo_send_list); ++i) {

            int temp_cursor = Convert_PDO_to_Bytes(&pdo_send_list[i], &t_byte_arr[t_cursor]);
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

/* ------------------- PDO TX Routine ------------------- */
static int Run_Send_PDO()
{
    uint8_t t_byte_len = 0;
    uint8_t t_dest_node = NODE_ID_CM;
    uint16_t t_identifier = 0;

    if (GUI_onoff) {
    	t_identifier = GUI_SYNC|GUI_command;
    } else {
    	t_identifier = PDO|(MD_nodeID<<4)|t_dest_node;
    }

    int t_check = Pack_PDO(fdcanTxData, &t_byte_len);

    if(t_check != 0){
        //TODO: Send PDO Error
    	return t_check;
    } else if(t_check){
    	return t_check;
    }

    if (t_byte_len != 1){
		if(Send_MSG(t_identifier, fdcanTxData, t_byte_len) == 0){
			return t_check;
			//TODO: MSG TX ERROR
		}
    }

	return t_check;
}

static int Ext_Send_PDO()
{
	if (GUI_command == GET_DIRECTION_SET_DATA) {
		Send_MSG((uint16_t)(GUI_SYNC|GET_DIRECTION_SET_DONE), (uint8_t*)0, 1);
	}
	else if (GUI_command == GET_AGING_DATA) {
		Send_MSG((uint16_t)(GUI_SYNC|GET_AGING_DONE), (uint8_t*)0, 1);
	}

	return 0;
}

static int Set_PDO_Dummy()
{
	static int t_count = 0;

	test_dummy[0] = comm_loop_cnt;
	test_dummy[1] = comm_loop_cnt;
	test_dummy[2] = comm_loop_cnt;
	test_dummy[3] = comm_loop_cnt;
	test_dummy[4] = comm_loop_cnt;
	test_dummy[5] = comm_loop_cnt;
	test_dummy[6] = comm_loop_cnt;
	test_dummy[7] = comm_loop_cnt;
	test_dummy[8] = comm_loop_cnt;
	test_dummy[9] = comm_loop_cnt;

	t_count++;

	return 0;
}

/* ------------------- MANUFACTURE ------------------- */

int Check_FD_CAN_COMMU(uint8_t* t_byte_arr)
{

    uint8_t t_checker = 0;
    uint8_t t_dest_node = NODE_ID_CM;
    uint8_t t_tx_data[64];
    uint16_t t_identifier = TRAJECTORY|(MD_nodeID<<4)|t_dest_node;
    uint8_t t_res;
	uint16_t t_checksum;


    for(int i = 3; i<62; i++)
    {
    	if(t_byte_arr[i]==i - 2)
    	{
    		t_checker++;
    	}
    }

    t_res = Checksum_Check(t_byte_arr, 62, (uint16_t*) &t_byte_arr[62]);

    if(t_checker == 59 && t_res == 1)
    {
    	t_tx_data[0] = 4;// TASK_ID_MANU
    	t_tx_data[1] = 1; // num of data
    	t_tx_data[2] = 255;//

    	for(int j = 3; j<62; j++)
    	{
    		t_tx_data[j]=62-j;
    	}

    	t_checksum = Checksum_Calcul(t_tx_data, 62);
    	memcpy(t_tx_data + 62, &t_checksum, 2);

        if(Send_MSG(t_identifier, t_tx_data, 64) != 0){
            //TODO: MSG err
        }
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


/* ------------------- TRAJECTORY ------------------- */
static int Check_Trajectory_Error(uint16_t t_frameIDX)
{
	if((t_frameIDX % D10_TRAJECTORY_ELEMENT_NUMBER) != 0)		{return -1;}
	if( (t_frameIDX - trajectory_buffer.frameIDX) != 0)		{return -2;}

	trajectory_buffer.frameIDX += D10_TRAJECTORY_ELEMENT_NUMBER;

	return 0;
}

static int Assemble_Trajectory(uint8_t* t_byte_arr)
{
	uint8_t t_cursor = 0, t_check = 0, t_buf = 0;
	uint16_t t_index = 0;
	int8_t t_ack = 0;
    uint16_t t_identifier = TRAJECTORY|(MD_nodeID<<4)|NODE_ID_CM;

	/* Get index */
	t_cursor = 0;
	memcpy(&t_index, &t_byte_arr[t_cursor], 2);
	t_cursor += 2;

	/* Check Error*/
	t_check = Check_Trajectory_Error(t_index);

	if(t_check != 0) {
		trajectory_buffer.frameIDX = 0;
		t_ack = -2;
	} else {

		/* Save Buffer */
		for(int i = 0; i < D10_TRAJECTORY_ELEMENT_NUMBER; ++i){
			memcpy(&trajectory_buffer.buff[t_index++], &t_byte_arr[t_cursor], 4);
			t_cursor += 4;
		}

		/* Check End of Trajectory */
		if(t_index >= D10_TRAJECTORY_TOTAL_LENGTH){
			t_ack = -1;
			trajectory_buffer.frameIDX = 0;
		} else {
			t_ack = 0;
		}
	}

	/* Send Acknowledgement */
	memcpy(&t_buf, &t_ack, 1);
	Send_MSG(t_identifier, &t_buf, 1);

	return t_check;
}

/* ------------------- LIST MANAGEMENT ------------------- */
static void Add_PDO_to_Send(uint8_t t_dictID, uint8_t t_objID)
{
	DOP_PDO_t* temp_pdo = DOP_FindPDO(t_dictID, t_objID);
    if (temp_pdo == NULL) {
        //TODO: Cannot Find PDO Error
        return;
    }

    DOP_Header_t t_pdo = {t_dictID, t_objID};

    for (int i = 0; i < cvector_size(pdo_send_list); ++i) {
        if ((pdo_send_list[i].dictID == t_dictID) && (pdo_send_list[i].objID == t_objID)){
            return;
        }
    }
    cvector_push_back(pdo_send_list, t_pdo);
}

static void Clear_PDO_to_Send()
{
    cvector_free(pdo_send_list);
    pdo_send_list = NULL;
}

static void Add_SDO_to_Send(uint8_t t_dictID, uint8_t t_objID)
{
	DOP_SDO_t* temp_sdo = DOP_FindSDO(t_dictID, t_objID);
    if (temp_sdo == NULL) {
        //TODO: Cannot Find PDO Error
        return;
    }

    DOP_Header_t t_sdo = {t_dictID, t_objID};

    for (int i = 0; i < cvector_size(sdo_res_list); ++i) {
        if ((sdo_res_list[i].dictID == t_dictID) && (sdo_res_list[i].objID == t_objID)){
            return;
        }
    }
    cvector_push_back(sdo_res_list, t_sdo);
}

static void Clear_SDO_to_Send()
{
    cvector_free(sdo_res_list);
    sdo_res_list = NULL;
}


/* ------------------- MSG HANDLER ------------------- */
static void Send_USB_Trick(uint8_t* t_in_buf, uint32_t t_in_len, uint8_t* t_out_buf)
{
	/*
	 * This function is designed to prevent
	 * the continuous array of 'CR/LF' used
	 * as a terminal signal in matlab gui.
	 *
	 * Sometimes, when sending a float signal,
	 * a continuous array of 'CR/LF' is created by coincidence,
	 * which interrupts the USB communication between the GUI
	 * and MD and breaks the sequence of the GUI.
	 *
	 * Therefore, 0x00 is inserted between every byte and sent.
	 * */

	for(int i = 0; i < t_in_len; ++i){
		*(t_out_buf + (2*i)) = *(t_in_buf + i);
		*(t_out_buf + (2*i+1)) = 0;
	}
}

int Send_MSG(uint16_t t_COB_ID, uint8_t* t_tx_data, uint32_t t_len)
{
	static uint8_t t_fnc_code = 0, t_nodeID = 0;
	int t_check = 0;
	uint8_t t_txBuf[67] = {0};
	uint8_t t_usb_txBuf[137] = {0};

	if (comm_type == COMM_TYPE_FDCAN) {
		if (IOIF_TransmitFDCAN1(t_COB_ID, t_tx_data, t_len) != 0) {
			return t_check = 1;
			//TODO: MSG TX ERROR
		}
	} else if (comm_type == COMM_TYPE_USB) {

		t_fnc_code = (t_COB_ID & 0xF00) >> 8;
		t_nodeID = (t_COB_ID & 0xFF);

		memcpy(&t_txBuf[2], t_tx_data, t_len);
		memcpy(t_txBuf, &t_fnc_code, 1);			t_len++;
		memcpy(&t_txBuf[1], &t_nodeID, 1);			t_len++;

		Send_USB_Trick(t_txBuf, t_len, t_usb_txBuf);
		t_len *= 2;

		t_usb_txBuf[t_len++] = '\r';
		t_usb_txBuf[t_len++] = '\n';

		// if(CDC_Transmit_FS(t_usb_txBuf, t_len) != 0){
        if (IOIF_USBD_Write(t_usb_txBuf, t_len, USB_CDC_MSG_TX_TIMEOUT_MS) != 0) {
			return t_check = 1;
			//TODO: MSG TX ERROR
		}
	}

	return -1;
}

static int USB_Rx_Hdlr(uint8_t* t_Buf, uint32_t* t_Len)
{
	uint32_t t_cursor = 0;

	fnc_code = ((uint16_t)*t_Buf) << 8;
	t_cursor++;

	ori_node = ((*(t_Buf+t_cursor)) & 0xF0)>>4;
	t_cursor++;

	memcpy(usbRxData, &t_Buf[t_cursor], *t_Len);

	switch(fnc_code) {

		case EMCY:
			Recv_EMCY(usbRxData, &err_code);
			// TODO: ERROR Process
			break;

		case SDO:
			if (Unpack_SDO(usbRxData) < 0) {
				return SDO_RX_ERR;
			} else{
				Send_SDO(ori_node);
			}
			break;

		case PDO:
			if (Unpack_PDO(usbRxData) < 0) {
				return PDO_RX_ERR;
			} else{
				Run_Send_PDO(ori_node);
			}
			break;

		default: break;
	}

	return 0;
}
uint32_t FW_Update, FW_Backup;

static int Fdcan_Rx_Hdlr(uint16_t t_wasp_id, uint8_t* t_rx_data)
{
	uint8_t temp[64] ={0,};

	fnc_code = t_wasp_id & 0x700;
    ori_node = (t_wasp_id & 0x0F0)>>4;
    dest_node = t_wasp_id & 0xF;

    memcpy(temp, t_rx_data, 64);

    switch(fnc_code) {

    case EMCY:
    	Recv_EMCY(t_rx_data, &err_code);
    	// TODO: ERROR Process
    	break;

    case SDO:
    	if (Unpack_SDO(t_rx_data) < 0) {
    		return SDO_RX_ERR;
    	} else{
    		Send_SDO(ori_node);
    	}
    	break;

    case PDO:
    	if (Unpack_PDO(t_rx_data) < 0) {
    		return PDO_RX_ERR;
    	} else{
    		//    		Run_Send_PDO(ori_node);
    	}
    	break;

    case TRAJECTORY:
    	Check_FD_CAN_COMMU(t_rx_data);
    	break;

//    case TRAJECTORY:
//    	Assemble_Trajectory(t_rx_data);
//    	break;


    case FW_UPDATE:
    	if(t_rx_data[0]==0){
    		IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13);  // Toggle LED
    		//MDUpdateFlag = 1;
    		Boot_SetFlags(0x1,0x0);
    		SoftwareSystemReset();
    	}

    	else if (t_rx_data[0]==1){
    		Send_EOT(1);
    	}

    	else if (t_rx_data[0]==2){
    		__disable_irq();
    		//RESET flags
    		uint32_t writeAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;
    		//erase
    		IOIF_EraseFlash(writeAddr, IOIF_ERASE_ONE_SECTOR);

//    		uint32_t readAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;

//    		IOIF_ReadFlash(readAddr, &FW_Update,       	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_SIZE_32B;
//    		IOIF_ReadFlash(readAddr, &FW_Backup,         IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_SIZE_32B;
    		__enable_irq();

    		Send_EOT(2);


    	}
    	break;

    default: break;
    }

    return 0;
}


BootUpdateError Boot_SetFlags(uint32_t t_MD_Update, uint32_t t_MD_Backup){

	__disable_irq();
	BootUpdateError ret = BOOT_UPDATE_OK;
	uint32_t writeAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;

	//erase
	if(IOIF_EraseFlash(writeAddr, IOIF_ERASE_ONE_SECTOR)!=IOIF_FLASH_STATUS_OK){
		return BOOT_UPDATE_ERROR_FLASH_ERASE;
	}
	for(int i=0; i<20000;i++){}

	//write
	if(IOIF_WriteFlash(writeAddr, &t_MD_Update)!=IOIF_FLASH_STATUS_OK){
		return BOOT_UPDATE_ERROR_FLASH_WRITE;
	}
	for(int i=0; i<20000;i++){}
	writeAddr+=32;

	if(IOIF_WriteFlash(writeAddr, &t_MD_Backup)!=IOIF_FLASH_STATUS_OK){
		return BOOT_UPDATE_ERROR_FLASH_WRITE;
	}
	for(int i=0; i<20000;i++){}
	writeAddr+=32;

	if(IOIF_WriteFlash(writeAddr, &MD_fw_binary_size)!=IOIF_FLASH_STATUS_OK){
		return BOOT_UPDATE_ERROR_FLASH_WRITE;
	}
	for(int i=0; i<20000;i++){}

	__enable_irq();

	return 	ret;
}

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Send_PDO_List(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	Clear_PDO_to_Send();

    int t_cursor = 0;
    uint8_t* t_ids = (uint8_t*)t_req->data;
    while (t_cursor < 2*t_req->dataSize) {
        uint8_t t_dictID = t_ids[t_cursor++];
        uint8_t t_objID = t_ids[t_cursor++];
        Add_PDO_to_Send(t_dictID, t_objID);
    }

    t_res->status = DOP_SDO_SUCC;
}
static void Set_Send_SDO_List(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	Clear_SDO_to_Send();

    int t_cursor = 0;
    uint8_t* t_ids = (uint8_t*)t_req->data;
    while (t_cursor < 2*t_req->dataSize) {
        uint8_t t_dictID = t_ids[t_cursor++];
        uint8_t t_objID = t_ids[t_cursor++];
        Add_SDO_to_Send(t_dictID, t_objID);
    }

    Send_SDO(NODE_ID_CM);

    t_res->status = DOP_SDO_SUCC;
}

static void Set_MS_Enum(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&MS_enum, t_req->data, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_GUI_COMM_OnOff(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&GUI_onoff, t_req->data, 1);

	t_res->dataSize = 1;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_GUI_COMM_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&GUI_command, t_req->data, 1);

	t_res->dataSize = 1;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_GUINodeID(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	static uint16_t t_identifier = 0;

	t_identifier = GUI_SYNC|READ_NODE_ID;
	Send_MSG(t_identifier, &MD_nodeID, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_FDCAN_Test_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	static uint16_t t_identifier = 0;
	fdcan_test_run_start = 0;
	memcpy(&fdcan_test_run_start, t_req->data, sizeof(fdcan_test_run_start));

	t_identifier = GUI_SYNC|FDCAN_SWITCH_SET;
	Send_MSG(t_identifier, &fdcan_test_run_start, 1);

	t_res->dataSize = 1;
	t_res->status = DOP_SDO_SUCC;

	if(fdcan_test_run_start == 1){//1616961754
		MS_enum = FDCAN_TEST_START; //Start/stop both need SW reset afterwards
	}
//	else if(fdcan_test_run_start == 0){
//		MS_enum= FDCAN_TEST_RESET;
//	}
	else{
		MS_enum = IDLE;
		//send msg that weird index
	}
}

void FDCAN_Test_Reset(){
	NVIC_SystemReset();
}

void Send_NeutralPostureDeg()
{
	Clear_SDO_to_Send();
	if (MD_nodeID == NODE_ID_RH_SAG) {
		Add_SDO_to_Send(TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_RH_NeutralPosture);
	} else if (MD_nodeID == NODE_ID_LH_SAG) {
		Add_SDO_to_Send(TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_LH_NeutralPosture);
	} else if (MD_nodeID == NODE_ID_RK) {
		Add_SDO_to_Send(TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_RK_NeutralPosture);
	} else if (MD_nodeID == NODE_ID_LK) {
		Add_SDO_to_Send(TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_LK_NeutralPosture);
	}
	Send_SDO(NODE_ID_CM);
}

void SendDurationCompletedFlag(void)
{
	Clear_SDO_to_Send();
	if (MD_nodeID == NODE_ID_RH_SAG) {
		Add_SDO_to_Send(TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_RH_P_VECTOR_DURATION_COMPLETED);
	} else if (MD_nodeID == NODE_ID_LH_SAG) {
		Add_SDO_to_Send(TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_LH_P_VECTOR_DURATION_COMPLETED);
	} else if (MD_nodeID == NODE_ID_RK) {
		Add_SDO_to_Send(TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_RK_P_VECTOR_DURATION_COMPLETED);
	} else if (MD_nodeID == NODE_ID_LK) {
		Add_SDO_to_Send(TASK_ID_WHOLEBODY, SDO_ID_WHOLEBODY_LK_P_VECTOR_DURATION_COMPLETED);
	}
	Send_SDO(NODE_ID_CM);
}

void RoboWear_MonitorGPIOPin() {
    static int previous_state = -1;
    int current_state;

    current_state = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_C, MAG_SENSOR_IN_Pin);
    stategpio=current_state;

if (comm_loop_cnt == 300 && current_state != previous_state) { // 첫 initial 시 sdo 못받을 수 있어서, 300ms 후에 재전송 로직 => sdo 수신 안정화가 되면 제거
    	Clear_SDO_to_Send();
    	Add_SDO_to_Send(TASK_ID_EXTDEV, SDO_ID_EXTDEV_ROBOWEAR_MONITOR);
    	Send_SDO(NODE_ID_CM);
    }

    previous_state = current_state;
}

int Send_EOT(uint8_t index){
	int ret=0;
	//send Start transmission
	uint16_t t_id = ((uint16_t)0x500U) | (MD_nodeID << 4) | (CM_nodeID) ;
	uint8_t array[8]={0,};
	array[0]=index;

	GetMES();
	array[1]=MES.FWVer.major;
	array[2]=MES.FWVer.minor;
	array[3]=MES.FWVer.patch;

	if(IOIF_TransmitFDCAN1(t_id, array , 8) != 0){
		ret = 100;			// tx error
	}

	MD_EOT_ACK_Flag ++;

	return ret;
}
