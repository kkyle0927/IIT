/**
 *-----------------------------------------------------------
 *            		MOTOR TEMPERATURE USING NTC
 *-----------------------------------------------------------
 * @file ioif_503ntc.c
 * @date Created on: Aug 20, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface functions for the Motor Temperature Measurement using NTC.
 * 
 * This source file contains the implementation of the functions and routines 
 * for measuring the temperature of the motor using a Negative Temperature Coefficient (NTC) resistor.
 * It provides functionalities such as reading the ADC values, converting them 
 * to voltages, and further processing these voltages to derive temperature values.
 * Calibration and filtering mechanisms are also integrated to improve the accuracy 
 * and stability of the temperature readings.
 * 
 * For more detailed information on NTC temperature measurement and calibration, 
 * refer to the associated documentation and references.
 * 
 * @ref NTC Datasheet
 */

#include "ioif_503ntc.h"

/** @defgroup ADC ADc
  * @brief ADC 504 NTC Motor Temp Sensor module driver
  * @{
  */
#ifdef IOIF_503NTC_ENABLED

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
IOIF_NTC_t cliNTCData= {0,};
uint16_t rawNTC=0;
#endif /* _USE_DEBUG_CLI */


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

//#ifdef _USE_DEBUG_CLI
//static IOIF_NTCState_t moduleinit_res=0;
//#endif /* _USE_DEBUG_CLI */

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static float FirstLPF(float currVal, float prevVal, float alpha);
static IOIF_NTCState_t GetNTCVolt(IOIF_NTC_t* ntc, uint16_t adcBuff);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Retrieves the motor temperature using the ADC buffer value.
 * 
 * @param ntc Pointer to the NTC structure.
 * @param adcBuff ADC buffer value.
 * @return IOIF_NTCState_t status indicating the result of the operation.
 */
IOIF_NTCState_t IOIF_GetNTCTemp(IOIF_NTC_t* ntc, uint16_t adcBuff)
{
    // Get the NTC voltage and apply LPF
    uint8_t status = GetNTCVolt(ntc, adcBuff);

    if (status != IOIF_NTC_STATUS_OK) {
//#ifdef _USE_DEBUG_CLI
//      moduleinit_res=status;
//#endif /* _USE_DEBUG_CLI */
        return status;  // Return early if there was an error getting the voltage
    }

    // Calculate the NTC resistance based on the voltage across it and the known series resistance
    ntc->motorR = IOIF_NTC_R * (IOIF_ADC3_VREF / ntc->filtVolt - 1);
    ntc->logMotorR = log(ntc->motorR / IOIF_NTC_R0);

    // Now, calculate the temperature based on the derived NTC resistance
    ntc->motorTemp = 1 / (1 / IOIF_NTC_ROOM_TEMP + log(ntc->motorR / IOIF_NTC_R0) / IOIF_NTC_BETA_COEFF) + IOIF_NTC_KEL_TO_CEL;
//#ifdef _USE_DEBUG_CLI
//      moduleinit_res=status;
//#endif /* _USE_DEBUG_CLI */
    return IOIF_NTC_STATUS_OK;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Retrieves the NTC voltage and applies a LPF to it.
 * 
 * @param ntc Pointer to the NTC structure.
 * @param adcBuff ADC buffer value.
 * @return IOIF_NTCState_t status indicating the result.
 */
static IOIF_NTCState_t GetNTCVolt(IOIF_NTC_t* ntc, uint16_t adcBuff)
{
    // Validity checks
    if (!ntc) {
//#ifdef _USE_DEBUG_CLI
//      moduleinit_res=IOIF_NTC_STATUS_ERROR;
//#endif /* _USE_DEBUG_CLI */
        return IOIF_NTC_STATUS_ERROR;  // Assuming you have an error status
    }
    
    // Convert ADC buffer value to voltage
    ntc->currVolt = (float)adcBuff * IOIF_ADC3_VREF / IOIF_ADC3_NTC_16BIT_RESOLUTION - IOIF_NTC_MEASURED_OFFSET;

    // Apply the LPF
    ntc->filtVolt = FirstLPF(ntc->currVolt, ntc->prevVolt, IOIF_NTC_LPF_WEIGHT);
    
    // Update the previous voltage value for the next iteration
    ntc->prevVolt = ntc->filtVolt;
//#ifdef _USE_DEBUG_CLI
//      moduleinit_res=IOIF_NTC_STATUS_OK;
//#endif /* _USE_DEBUG_CLI */
    return IOIF_NTC_STATUS_OK;  // Assuming you have a status indicating success
}


/**
 * @brief Applies a first-order low-pass filter to the input.
 * 
 * @param currValue Current input value.
 * @param prevValue Previous filtered value.
 * @param alpha Weight for the current value. (0 < alpha < 1)
 * @return Filtered value.
 */
static bool isInitialized_NTC = true;

#ifdef _USE_DEBUG_CLI
static bool isInitialized_NTC_CLI = true;
#endif /* _USE_DEBUG_CLI */

static float FirstLPF(float currVal, float prevVal, float alpha)
{
#ifndef _USE_DEBUG_CLI
    if (isInitialized_NTC == true) {
#endif /* !_USE_DEBUG_CLI */

#ifdef _USE_DEBUG_CLI
    if (isInitialized_NTC_CLI == true) {
#endif /* _USE_DEBUG_CLI */
        prevVal = currVal;
        isInitialized_NTC = false;
    }
    return alpha * currVal + (1.0 - alpha) * prevVal;
}

#ifdef _USE_DEBUG_CLI
/**
*------------------------------------------------------------
*                      CLI FUNCTIONS
*------------------------------------------------------------
* @brief Functions are supported Commmand Line Interface (CLI).
*/
void CLI_Run503NTC(cli_args_t *args)
{
    bool ret = false;
    uint32_t duration = 10;                             //default,

//  if (args->cmpStr(0, "isinit") == true)
//  {
//      IOIF_NTCState_t state = moduleinit_res;
//
//      const char* NTCStateStrings[] = {
//              "IOIF_NTC_STATUS_OK",
//              "IOIF_NTC_STATUS_ERROR"
//      };
//
//      CLI_Printf(" * 503NTC init state : %s * \r\n",NTCStateStrings[state]);
//
//  } else
    if(args->argc == 2 && args->cmpStr(0, "getdata") == true) {

        CLI_Printf(" * 503NTC data gathering started! * \r\n");
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
            rawNTC=IOIF_GetNTCValue();
            IOIF_GetNTCTemp(&cliNTCData, rawNTC);
            /* Output to String */
            CLI_Printf("Motor temp: %f deg Celsius.\r\n", cliNTCData.motorTemp);
            /* Duration */
            CLI_Delay(duration);
        }
    } else if(ret == false)     //help
    {
        CLI_Printf(" * Motor Temperature Monitoring * \r\n");
        CLI_Printf("   Get 503NTC temperature sensor data at a specified period of time[ms] \r\n");
        CLI_Printf(" * Usage * \r\n");
//      CLI_Printf("   503ntc isinit \r\n"); //
        CLI_Printf("   503ntc getdata [duration(ms) (min. 10ms)]\r\n");
        CLI_Printf(" * Command Example * \r\n");
        CLI_Printf("   503ntc getdata 100 \r\n");
    }
}
#endif /*_USE_DEBUG_CLI*/

#endif /* IOIF_503NTC_ENABLED */
