/**
 *-----------------------------------------------------------
 *               IOIF INCREMENTAL ENCODER INTERFACE
 *-----------------------------------------------------------
 * @file ioif_rmb20ic.c
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface functions for the Incremental Encoder.
 * 
 * This source file contains the implementation of the interface functions 
 * for reading incremental encoders using timer encoder mode. It provides 
 * functionalities such as initialization, setting offset, reading encoder 
 * counts, and handling rollovers.
 * 
 * @ref Timer Encoder Mode Documentation or Datasheet
 */

#include "ioif_rmb20ic.h"

/** @defgroup TIM TIM
  * @brief TIM EMB20IC Incremental Encoder module driver
  * @{
  */
#ifdef IOIF_RMB20IC_ENABLED

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

#ifdef _USE_DEBUG_CLI
IOIF_IncEnc_t cliIncData;
#endif /* _USE_DEBUG_CLI */


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

#ifdef _USE_DEBUG_CLI
static IOIF_IncState_t moduleinit_res=0;
#endif /* _USE_DEBUG_CLI */

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void HandleRollover(IOIF_IncEnc_t* incEnc);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
  * @brief  Initializes Incremental Encoder Interface.
  * @param  timer IOIF timer enumeration for the encoder.
  * @param  channel Channel number for the encoder.
  * @param  incEnc Pointer to the IOIF_IncEnc_t structure that will store encoder details.
  * @retval IOIF_IncState_t status indicating success or error.
  */
IOIF_IncState_t IOIF_InitIncEnc(IOIF_Tim_t timer, uint32_t channel, IOIF_IncEnc_t* incEnc)
{
    IOIF_IncState_t status = 0;
    // Enable the GPIO for the Incremental Encoder
    BSP_WriteGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_3, BSP_GPIO_PIN_SET);

    if(BSP_RunTimEnc((BSP_Tim_t)timer, channel, BSP_TIM_ENCODER_START) > 0)
    {
        // Error in starting the encoder
        status = IOIF_INCENC_STATUS_ERROR;
    #ifdef _USE_DEBUG_CLI
        moduleinit_res = status;
    #endif /* _USE_DEBUG_CLI */
        return status;
    }

    // Initialize the Incremental Encoder structure
    incEnc->offset = BSP_GetCnt(timer);
    incEnc->currCnt = 0;
    incEnc->prevCnt = 0;
    incEnc->userCnt = 0;

    status = IOIF_INCENC_STATUS_OK;

    #ifdef _USE_DEBUG_CLI
        moduleinit_res = status;
    #endif /* _USE_DEBUG_CLI */
    return status;
}

/**
  * @brief  Set the offset for the Incremental Encoder.
  * @param  timer IOIF timer enumeration for the encoder.
  * @param  incEnc Pointer to the IOIF_IncEnc_t structure that will store encoder details.
  * @retval IOIF_IncState_t status indicating success.
  */
IOIF_IncState_t IOIF_SetIncOffset(IOIF_Tim_t timer, IOIF_IncEnc_t* incEnc)
{
    if (timer < 0 || timer >= IOIF_TIM_COUNT) {
        return IOIF_INCENC_TIM_ERROR;
    }
    
    incEnc->offset = BSP_GetCnt(timer);
    return IOIF_INCENC_STATUS_OK;
}

/**
  * @brief  Read the current count of the Incremental Encoder and handle rollovers.
  * @param  timer IOIF timer enumeration for the encoder.
  * @param  incEnc Pointer to the IOIF_IncEnc_t structure that will store encoder details.
  * @retval IOIF_IncState_t status indicating success.
  */
IOIF_IncState_t IOIF_ReadIncCnt(IOIF_Tim_t timer, IOIF_IncEnc_t* incEnc)
{
    if (timer < 0 || timer >= IOIF_TIM_COUNT) {
        return IOIF_INCENC_TIM_ERROR;
    }

    if (!incEnc) {
        return IOIF_INCENC_NULL_POINTER;
    }

    incEnc->currCnt = BSP_GetCnt(timer) - incEnc->offset;  // Assuming count is relative to offset
	HandleRollover(incEnc);
    return IOIF_INCENC_STATUS_OK;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
  * @brief  Handle rollovers for the incremental encoder counter.
  * @param  incEnc Pointer to the IOIF_IncEnc_t structure that will store encoder details.
  * @param  currCnt Current counter value after subtracting the offset.
  * @retval None.
  */
static void HandleRollover(IOIF_IncEnc_t* incEnc)
{
    int32_t diff = incEnc->currCnt - incEnc->prevCnt;

    // Handle positive rollover
    if (incEnc->prevCnt > 0 && incEnc->currCnt < 0 && (incEnc->prevCnt - incEnc->currCnt) > IOIF_INT32_THRESHOLD) {
        // Calculate the increment considering the overflow
        int32_t positiveIncrement = (IOIF_INT32_MAX - incEnc->prevCnt) + (incEnc->currCnt - IOIF_INT32_MIN);
        // Apply saturation
        if (INT32_MAX - positiveIncrement <= incEnc->userCnt) {
            incEnc->userCnt = INT32_MAX;
        } else {
            incEnc->userCnt += positiveIncrement;
        }
    }
    // Handle negative rollover
    else if (incEnc->prevCnt < 0 && incEnc->currCnt > 0 && (incEnc->currCnt - incEnc->prevCnt) > IOIF_INT32_THRESHOLD) {
        // Calculate the decrement considering the underflow
        int32_t negativeDecrement = (incEnc->prevCnt - IOIF_INT32_MIN) + (IOIF_INT32_MAX - incEnc->currCnt);
        // Apply saturation
        if (INT32_MIN + negativeDecrement >= incEnc->userCnt) {
            incEnc->userCnt = INT32_MIN;
        } else {
            incEnc->userCnt -= negativeDecrement;
        }
    }
    // Handle normal case without rollover
    else {
        // Apply saturation for addition
        if (diff > 0 && INT32_MAX - diff <= incEnc->userCnt) {
            incEnc->userCnt = INT32_MAX;
        }
        // Apply saturation for subtraction
        else if (diff < 0 && INT32_MIN - diff >= incEnc->userCnt) {
            incEnc->userCnt = INT32_MIN;
        }
        // Normal addition
        else {
            incEnc->userCnt += diff;
        }
    }

    incEnc->prevCnt = incEnc->currCnt;
}

#ifdef _USE_DEBUG_CLI
/**
*------------------------------------------------------------
*                      CLI FUNCTIONS
*------------------------------------------------------------
* @brief Functions are supported Commmand Line Interface (CLI).
*/
void CLI_RunRMB20IC(cli_args_t *args)
{
    bool ret = false;
    uint32_t duration = 10;                             //default,

    if (args->cmpStr(0, "isinit") == true)
    {
        IOIF_IncState_t state = moduleinit_res;

        const char* incEncStateStrings[] = {
                "IOIF_INCENC_STATUS_OK",
                "IOIF_INCENC_STATUS_ERROR",
                "IOIF_INCENC_TIM_ERROR",
                "IOIF_INCENC_NULL_POINTER"
        };

        CLI_Printf(" * RMB20IC init state : %s * \r\n", incEncStateStrings[state]);

    } else if(args->argc == 2 && args->cmpStr(0, "getdata") == true) {

        CLI_Printf(" * RMB20IC data gathering started! * \r\n");
        duration = (uint32_t) args->getData(1);         // param. 1 �� ���� string �����͸� ������ ��ȯ

        if(duration < 10){
            ret = false;
            CLI_Printf(" * Duration too short. Minimum is 10ms. * \r\n");
        }
        else{
            ret = true;
        }

        while(ret && CLI_KeepLoop())
        {
            IOIF_ReadIncCnt(IOIF_TIM5, &cliIncData);
            /* Output to String */
            CLI_Printf("Inc enc count: %d. \r\n", cliIncData.currCnt);
            /* Duration */
            CLI_Delay(duration);
        }
    } else if(ret == false)     //help
    {
        CLI_Printf(" * Inc encoder Monitoring * \r\n");
        CLI_Printf("   Get rmb20ic count sensor data at a specified period of time[ms] \r\n");
        CLI_Printf(" * Usage * \r\n");
        CLI_Printf("   rmb20ic isinit \r\n");
        CLI_Printf("   rmb20ic getdata [duration(ms) (min. 10ms)]\r\n");
        CLI_Printf(" * Command Example * \r\n");
        CLI_Printf("   rmb20ic getdata 100 \r\n");
    }
}
#endif /* _USE_DEBUG_CLI */

#endif /* IOIF_RMB20IC_ENABLED */
