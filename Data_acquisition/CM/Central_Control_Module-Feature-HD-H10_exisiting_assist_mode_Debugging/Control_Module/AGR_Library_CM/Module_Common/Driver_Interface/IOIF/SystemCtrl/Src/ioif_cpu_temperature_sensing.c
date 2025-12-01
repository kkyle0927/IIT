
/**
 *-----------------------------------------------------------
 *             CPU Temperature Sense & Calculate
 *-----------------------------------------------------------
 * @file ioif_cpu_temperature_sensing.c
 * @date Created on: Nov 2, 2023
 * @author AngelRobotics HW Team
 * @brief
 *
 * @ref
 */

#include "ioif_cpu_temperature_sensing.h"

/** @defgroup ADC ADC
  * @brief ADC BSP module driver
  * @{
  */
#ifdef IOIF_CPUTEMP_ENABLE

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




/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */
BSP_ADC_t adc_port = BSP_ADC3;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static uint32_t IOIF_GetTempSensorADC(void);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

uint32_t IOIF_ReadCPUTemp(void)
{
	static uint32_t raw_temp=0;
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	raw_temp = IOIF_GetTempSensorADC();
	return __HAL_ADC_CALC_TEMPERATURE(3300, raw_temp, LL_ADC_RESOLUTION_16B);
	//	static uint32_t raw_temp=0;
//	BSP_StartADCExCalib(adc_port, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
//	raw_temp = IOIF_GetTempSensorADC();
//	return BSP_ADC_CALC_TEMPERATURE(3300, raw_temp, LL_ADC_RESOLUTION_12B);
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
static uint32_t IOIF_GetTempSensorADC(void)
{
	uint32_t temp;

	BSP_RunADCBlock(adc_port, BSP_ADC_START);
	BSP_RunADCConvBlock(adc_port, 100);
	temp = BSP_GetADCValue(adc_port);
	BSP_RunADCBlock(adc_port, BSP_ADC_STOP);

	return temp;
}

#endif /* IOIF_CPUTEMP_ENABLE */

