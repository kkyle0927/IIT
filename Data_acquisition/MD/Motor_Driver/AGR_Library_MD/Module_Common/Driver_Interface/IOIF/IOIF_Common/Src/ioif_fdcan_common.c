/**
 *-----------------------------------------------------------
 *                 FDCAN Communication Driver
 *-----------------------------------------------------------
 * @date Created on: Aug 23, 2023
 * @author AngelRobotics HW Team
 * @brief Driver code for the FDCAN communication.
 *
 * This source file provides functionality to interface
 *
 * @ref FDCAN reference
 */

#include "ioif_fdcan_common.h"

/** @defgroup FDCAN FDCAN
  * @brief FDCAN BSP module driver
  * @{
  */
#ifdef BSP_FDCAN_MODULE_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

#ifdef _USE_DEBUG_CLI
volatile bool fdcan_rxflag = false;
volatile uint32_t fdcan_rxcnt = 0;
static IOIF_FDCANState_t moduleinit_res=0;
//static uint32_t cli_rx_cnt = 0;
//static uint8_t false_cnt = 0; //receive Ÿ�̹� �ȸ��� �� print��
//uint16_t checksum = 0;
uint16_t errorsum = 0;
uint32_t error_cnt = 0;
//uint8_t error_buff[64] = {};
#endif /* _USE_DEBUG_CLI */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

static IOIF_FDCANUserCBPtr_t userRxCBPtr1;
static IOIF_FDCANUserCBPtr_t userRxCBPtr2;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */
static IOIF_FDCANObj_t* fdcanPtr = NULL;

static IOIF_FDCANObj_t fdcan1Obj = {0};
static IOIF_FDCANObj_t fdcan2Obj = {0};

static BSP_FDCANFilterTypeDef_t IOIF_FDCANsFilter = {
	.IdType = BSP_FDCAN_STANDARD_ID,
	.FilterIndex = 0,
	.FilterType = BSP_FDCAN_FILTER_RANGE,
	.FilterConfig = BSP_FDCAN_FILTER_TO_RXFIFO0,
	.FilterID1 = 0x000,
	.FilterID2 = 0x7FF,
	.RxBufferIndex = 0,			// ignored when (FilterConfig != FDCAN_FILTER_TO_RXBUFFER
	.IsCalibrationMsg = 0		// ignored when (FilterConfig != FDCAN_FILTER_TO_RXBUFFER
};

static BSP_FDCANTxHeaderTypeDef_t IOIF_FDCANTxHeader = {
	.Identifier = 0,
	.IdType = BSP_FDCAN_STANDARD_ID,
	.TxFrameType = BSP_FDCAN_DATA_FRAME,
	.DataLength = 0,
	.ErrorStateIndicator = BSP_FDCAN_ESI_ACTIVE,
	.BitRateSwitch = BSP_FDCAN_BRS_ON,
	.FDFormat = BSP_FDCAN_FD_CAN,
	.TxEventFifoControl = BSP_FDCAN_NO_TX_EVENTS,
	.MessageMarker = 0
};

// Define the static global constant lookup table
static const IOIF_DLCLookupTable_t lookupTable[] = {
    {4,  BSP_FDCAN_DLC_BYTES_4},
    {8,  BSP_FDCAN_DLC_BYTES_8},
    {12, BSP_FDCAN_DLC_BYTES_12},
    {16, BSP_FDCAN_DLC_BYTES_16},
    {20, BSP_FDCAN_DLC_BYTES_20},
    {24, BSP_FDCAN_DLC_BYTES_24},
    {32, BSP_FDCAN_DLC_BYTES_32},
    {48, BSP_FDCAN_DLC_BYTES_48},
    {64, BSP_FDCAN_DLC_BYTES_64}
};


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void IOIF_AllocateFDCANRxCB(IOIF_FDCAN_t fdcan, IOIF_FDCANUserCBPtr_t funcPtr);
static void RunFDCAN1RxCB(void* params);
static void RunFDCAN2RxCB(void* params);
static uint32_t Len2DLC(uint32_t len);

#ifdef _USE_DEBUG_CLI
static uint16_t IOIF_FdcanCalchecksum(uint8_t data[], size_t length);
#endif /* _USE_DEBUG_CLI */

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Initialize FDCAN.
 */
IOIF_FDCANState_t IOIF_InitFDCAN1(uint32_t nodeId)
{
	IOIF_FDCANState_t status;

	fdcanPtr = &fdcan1Obj;

	fdcanPtr->maskWindow 	 	= 0x00F;
	fdcanPtr->userFilter1 	 	= 0;
	fdcanPtr->userFilter2 	 	= nodeId;
	fdcanPtr->sFilterConfig  	= IOIF_FDCANsFilter;
	fdcanPtr->TxHeader			= IOIF_FDCANTxHeader;

	if (BSP_ConfigFDCANGlobalFilter(BSP_FDCAN1, BSP_FDCAN_REJECT, BSP_FDCAN_REJECT, BSP_FDCAN_FILTER_REMOTE, BSP_FDCAN_FILTER_REMOTE) != BSP_OK) {
		status=IOIF_FDCAN_STATUS_ERROR;
#ifdef _USE_DEBUG_CLI
		moduleinit_res = status;
#endif /* _USE_DEBUG_CLI */
		return status;
	}

	if (BSP_ConfigFDCANFilter(BSP_FDCAN1, &fdcanPtr->sFilterConfig) != BSP_OK) {
		status=IOIF_FDCAN_STATUS_ERROR;
#ifdef _USE_DEBUG_CLI
		moduleinit_res = status;
#endif /* _USE_DEBUG_CLI */
		return status;
	}

	if (BSP_ActivateFDCANNotification(BSP_FDCAN1, BSP_FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != BSP_OK) {
		status=IOIF_FDCAN_STATUS_ERROR;
#ifdef _USE_DEBUG_CLI
		moduleinit_res = status;
#endif /* _USE_DEBUG_CLI */
		return status;
	}

	if (BSP_ConfigFDCANTxDelay(BSP_FDCAN1, IOIF_TDC_FILTER) != BSP_OK) {
		status=IOIF_FDCAN_STATUS_ERROR;
#ifdef _USE_DEBUG_CLI
		moduleinit_res = status;
#endif /* _USE_DEBUG_CLI */
		return status;
	}

	if (BSP_EnableFDCANTxDelay(BSP_FDCAN1) != BSP_OK) {
		status=IOIF_FDCAN_STATUS_ERROR;
#ifdef _USE_DEBUG_CLI
		moduleinit_res = status;
#endif /* _USE_DEBUG_CLI */
		return status;
	}

	if (BSP_StartFDCAN(BSP_FDCAN1) != BSP_OK) {
		status=IOIF_FDCAN_STATUS_ERROR;
#ifdef _USE_DEBUG_CLI
		moduleinit_res = status;
#endif /* _USE_DEBUG_CLI */
		return status;
	}

	status=IOIF_FDCAN_STATUS_OK;
#ifdef _USE_DEBUG_CLI
	moduleinit_res = status;
#endif /* _USE_DEBUG_CLI */
	return status;
}

IOIF_FDCANState_t IOIF_InitFDCAN2(uint32_t nodeId)
{
	fdcanPtr = &fdcan2Obj;

	fdcanPtr->maskWindow 	 	= 0x00F;
	fdcanPtr->userFilter1 	 	= 0;
	fdcanPtr->userFilter2 	 	= nodeId;
	fdcanPtr->sFilterConfig  	= IOIF_FDCANsFilter;
	fdcanPtr->TxHeader			= IOIF_FDCANTxHeader;

	if (BSP_ConfigFDCANGlobalFilter(BSP_FDCAN2, BSP_FDCAN_REJECT, BSP_FDCAN_REJECT, BSP_FDCAN_FILTER_REMOTE, BSP_FDCAN_FILTER_REMOTE) != BSP_OK) {
		return IOIF_FDCAN_STATUS_ERROR;
	}

	if (BSP_ConfigFDCANFilter(BSP_FDCAN2, &fdcanPtr->sFilterConfig) != BSP_OK) {
		return IOIF_FDCAN_STATUS_ERROR;
	}

	if (BSP_ActivateFDCANNotification(BSP_FDCAN2, BSP_FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != BSP_OK) {
		return IOIF_FDCAN_STATUS_ERROR;
	}

	if (BSP_ConfigFDCANTxDelay(BSP_FDCAN2, IOIF_TDC_FILTER) != BSP_OK) {
		return IOIF_FDCAN_STATUS_ERROR;
	}

	if (BSP_EnableFDCANTxDelay(BSP_FDCAN2) != BSP_OK) {
		return IOIF_FDCAN_STATUS_ERROR;
	}

	if (BSP_StartFDCAN(BSP_FDCAN2) != BSP_OK) {
		return IOIF_FDCAN_STATUS_ERROR;
	}

	return IOIF_FDCAN_STATUS_OK;
}


/**
 * @brief Transmit Data with FDCAN
 * @param msgId is identifier, len is message's length, txData is a pointer of data which you want to transmit
 * @return Status of the data transmission
 */
uint8_t IOIF_TransmitFDCAN1(uint16_t msgId, uint8_t* txData, uint32_t len)
{
	fdcanPtr = &fdcan1Obj;

	fdcanPtr->TxHeader.Identifier = msgId;	// msgId = (0x"Msg Type"/"ori_node"/"dest_node")
	fdcanPtr->TxHeader.DataLength = Len2DLC(len);

	return BSP_AddFDCANMsgToTxQ(BSP_FDCAN1, &fdcanPtr->TxHeader, txData);
}

uint8_t IOIF_TransmitFDCAN2(uint16_t msgId, uint8_t* txData, uint32_t len)
{
	fdcanPtr = &fdcan2Obj;

	fdcanPtr->TxHeader.Identifier = msgId;	// msgId = (0x"Msg Type"/"ori_node"/"dest_node")
	fdcanPtr->TxHeader.DataLength = Len2DLC(len);

	return BSP_AddFDCANMsgToTxQ(BSP_FDCAN2, &fdcanPtr->TxHeader, txData);
}


/**
 * @brief Allocating RxFifo0Callback function
 */
static void IOIF_AllocateFDCANRxCB(IOIF_FDCAN_t fdcan, IOIF_FDCANUserCBPtr_t funcPtr)
{
	if (fdcan == IOIF_FDCAN1) {
		userRxCBPtr1 = funcPtr;
	} else if (fdcan == IOIF_FDCAN2) {
		userRxCBPtr2 = funcPtr;
	}
}

void IOIF_SetFDCANRxCB(IOIF_FDCAN_t fdcan, IOIF_FDCANCBType_t callbackType, IOIF_FDCANUserCBPtr_t funcPtr)
{
	if (fdcan == IOIF_FDCAN1) {
		BSP_SetFDCANCB(fdcan, callbackType, RunFDCAN1RxCB, NULL);
	} else if (fdcan == IOIF_FDCAN2) {
		BSP_SetFDCANCB(fdcan, callbackType, RunFDCAN2RxCB, NULL);
	}

	IOIF_AllocateFDCANRxCB(fdcan, funcPtr);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void RunFDCAN1RxCB(void* params)
{
#ifdef _USE_DEBUG_CLI
    fdcan_rxflag = true;
    fdcan_rxcnt++;
#endif /* _USE_DEBUG_CLI */

	fdcanPtr = &fdcan1Obj;
	if (BSP_GetFDCANRxMsg(IOIF_FDCAN1, BSP_FDCAN_RX_FIFO0, &fdcanPtr->RxHeader, fdcanPtr->RxData) != BSP_OK) {
		// TODO : Handle Error !
		return;
	}

#ifdef _USE_DEBUG_CLI
//	static uint16_t checksum = 0;
//	checksum = IOIF_FdcanCalchecksum(fdcanPtr->RxData, 64);
//
//	if (checksum != 2160) error_cnt ++;//PCAN
////	if(checksum!=576) error_cnt++;//CM

#ifdef CM_MD_TEST
	//Transmit received data
	IOIF_TransmitFDCAN1(256, &fdcanPtr->RxData[0], 64);//CM-MD test
	memset(&fdcanPtr->RxData[0],0, 64);
#endif

#else /* _USE_DEBUG_CLI */


	// For Checking Origin Node
	if (((fdcanPtr->RxHeader.Identifier & fdcanPtr->maskWindow) != fdcanPtr->userFilter1) && \
		((fdcanPtr->RxHeader.Identifier & fdcanPtr->maskWindow) != fdcanPtr->userFilter2)) {
		// TODO : Handle Error !
		return;
	}


	if (userRxCBPtr1 != NULL) {
		userRxCBPtr1(fdcanPtr->RxHeader.Identifier, fdcanPtr->RxData);
	}
#endif

}

static void RunFDCAN2RxCB(void* params)
{
	fdcanPtr = &fdcan2Obj;

	if (BSP_GetFDCANRxMsg(IOIF_FDCAN2, BSP_FDCAN_RX_FIFO0, &fdcanPtr->RxHeader, fdcanPtr->RxData) != BSP_OK) {
		// TODO : Handle Error !
		return;
	}

	// For Checking Origin Node
	if (((fdcanPtr->RxHeader.Identifier & fdcanPtr->maskWindow) != fdcanPtr->userFilter1) && \
		((fdcanPtr->RxHeader.Identifier & fdcanPtr->maskWindow) != fdcanPtr->userFilter2)) {
		// TODO : Handle Error !
		return;
	}

	if (userRxCBPtr2 != NULL) {
		userRxCBPtr2(fdcanPtr->RxHeader.Identifier, fdcanPtr->RxData);
	}
}

/**
 * @brief Allocate FDCAN Object according to the FDCAN Port Number.
 */
// Your function using the global lookup table
static uint32_t Len2DLC(uint32_t len)
{
    for(size_t i = 0; i < sizeof(lookupTable) / sizeof(lookupTable[0]); i++) {
        if (len <= lookupTable[i].maxLen) {
            return lookupTable[i].dlcValue;
        }
    }

    return BSP_FDCAN_DLC_BYTES_64;
}

#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */
uint16_t checksum = 0;
static uint16_t IOIF_FdcanCalchecksum(uint8_t data[], size_t length) {
	checksum=0;
	for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}


//uint8_t buff[64];
uint32_t cli_rx_cnt=0;

void CLI_RunFdcan(cli_args_t *args){
	fdcan_rxcnt = 0;
	errorsum = 0;
	error_cnt = 0;

	bool ret = false;
    if (args->cmpStr(0, "isinit") == true)
    {
        IOIF_FDCANState_t state = moduleinit_res;
        const char* fdcanStateStrings[] = {
                "IOIF_FDCAN_STATUS_OK",
                "IOIF_FDCAN_STATUS_ERROR",
                "IOIF_FDCANSTATUS_BUSY",
                "IOIF_FDCANSTATUS_TIMEOUT"
        };

        CLI_Printf(" * FDCAN init state : %s * \r\n",fdcanStateStrings[state]);

    } else if(args->argc == 2 && args->cmpStr(0, "write") == true)
    {
        uint8_t id = 256;//100;
        uint8_t buff[64];
        uint32_t duration = 10; //ms
        uint32_t cnt = 0;

        duration = (uint32_t) args->getData(1);

        memset(buff, 9, sizeof(buff));
        if(duration < 0){
            ret = false;
            CLI_Printf(" * Duration too short. Minimum is 0ms(equivalent to 1ms in reality). * \r\n");
        }
        else{
            ret = true;
            while(ret && CLI_KeepLoop())
            {
            	buff[0] = (uint8_t)(cnt & 0xFF);
            	buff[63] = buff[0];

                IOIF_TransmitFDCAN1(id, buff, sizeof(buff));

                //CLI_Printf(" * Packet Transmit %d times *\r\n", cnt);

                cnt++;

                if (cnt >= 4000000000) //4,000,000,000
                {
                    CLI_Printf(" * End of writing *\r\n", cnt);
                    CLI_Printf(" * Please reset MCU for new fdcan write test*\r\n", cnt);
                    break;
                }

                /* Duration */
                CLI_Delay(duration);
            }
    		CLI_Printf(" * End of writing *\r\n");
    		CLI_Printf(" * Packet Transmitted %d times *\r\n", cnt);

        }
    } else if (args->argc == 2 && args->cmpStr(0, "read") == true)
    {
        CLI_Printf(" * FDCAN Receive is ready! * \r\n");
        CLI_Printf(" * Send 'FDCAN CLI.xmt' from PCAN * \r\n");
        CLI_Printf(" * Do not use other files * \r\n");
        CLI_Printf(" \r\n");
        CLI_Printf(" * Receiving data... * \r\n");

        uint32_t duration = 10; //ms
//        uint8_t size = 64;
//        uint8_t buff[64] = {0};
		static uint32_t pre_error = 0;

        duration = (uint32_t) args->getData(1);

        if(duration < 1){
            ret = false;
            CLI_Printf(" * Duration too short. Minimum is 1ms. * \r\n");
        }
        else{
            ret = true;
            while(ret && CLI_KeepLoop())
            {
//                memcpy(&buff[0], &fdcanPtr->RxData[0], size);
//                //          CLI_Printf("Received: %d", fdcanPtr->RxData[0]);
//
//                checksum = IOIF_FdcanCalchecksum(buff, size);
//                if (checksum != 2160) {
//
//                    if (checksum != 0) {
//
//                        error_cnt ++;
//                        errorsum = checksum;
//                        memcpy(&error_buff[0], &fdcanPtr->RxData[0], 64);
//                        //                  CLI_Printf(" * FDCAN error times : %d * \r\n", error_cnt);
//
            	//                    } else if (checksum == 0) {
            	//                        //                  CLI_Printf(" * No data transmitted * \r\n");
            	//                    }
            	//                }
//            	                cli_rx_cnt++;


//                IOIF_TransmitFDCAN1(256, &fdcanPtr->RxData[0], 64);//CM-MD test

            	if (error_cnt > pre_error)
            	{
            		CLI_Printf(" Receive error : %d times \r\n", error_cnt);
            		pre_error = error_cnt;
            	}

            	if (fdcan_rxcnt >= 4000000000 )
            	{
            		CLI_Printf(" * End of receiving *\r\n");
            		CLI_Printf(" * Please reset MCU for new fdcan receive test*\r\n");
            		break;
            	}


            	/* Duration */
            	CLI_Delay(duration);
            }
    		CLI_Printf(" * End of receiving *\r\n");
    		CLI_Printf(" * FDCAN error times : %d out of %d * \r\n", error_cnt, fdcan_rxcnt);


        }
    }else if (ret == false) //help
    {
    	CLI_Printf(" * FDCAN Comm.* \r\n");
    	CLI_Printf("   Data exchange via FDCAN. \r\n");
    	CLI_Printf("   !! Written considering the use of PCAN in validation !! \r\n");
    	CLI_Printf(" * Input data specification  * \r\n");
    	CLI_Printf("   duration(ms) * \r\n");
    	CLI_Printf(" * Usage * \r\n");
    	CLI_Printf("   fdcan isinit \r\n");
		CLI_Printf("   fdcan write [duration (substract 1 from desired period)] \r\n");
        CLI_Printf("   fdcan read [duration] \r\n");
        CLI_Printf(" * Command Example * \r\n");
        CLI_Printf("   fdcan write 0 (for 1ms freq.)\r\n");

    }
}

#endif /* _USE_DEBUG_CLI */


#endif /* BSP_FDCAN_MODULE_ENABLED */

