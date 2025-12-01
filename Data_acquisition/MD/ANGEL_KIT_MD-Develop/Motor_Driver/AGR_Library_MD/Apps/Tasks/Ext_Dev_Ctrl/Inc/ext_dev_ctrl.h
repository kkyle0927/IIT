

#ifndef APPS_EXT_DEV_CTRL_INC_EXT_DEV_CTRL_H_
#define APPS_EXT_DEV_CTRL_INC_EXT_DEV_CTRL_H_

#include "task_mngr.h"

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_adc_common.h"

#include "ioif_szh_hws004.h"
#include "ioif_10kohm_lp.h"
#include "ioif_503ntc.h"
#include "ioif_tb67h450fngel.h"
#include "ioif_tp_lm_sensor.h"
#include "ioif_nzr_led.h"

#include "msg_hdlr.h"
#include "low_level_ctrl.h"

#include "ioif_cpu_temperature_sensing.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define POLL_CONV_TIMEOUT 					(100)
#define EXT_DEV_CONTROL_FREQUENCY      		(100)
#define EXT_DEV_CONTROL_PERIOD      		(0.01)


/* for derating control to avoid Pwm off by OTP */
#define	OVER_TEMP_DERATING_START_LV			(110.0)
#define OVER_TEMP_DERATING_END_LV			(100.0)
#define OVER_TEMP_DERATING_SET_TIME_LV1		(200) 		// 2s
#define OVER_TEMP_DERATING_SET_TIME_LV2		(1200) 		// 12s
#define OVER_TEMP_DERATING_SET_TIME_LV3		(2200) 		// 22s
#define OVER_TEMP_DERATING_SET_TIME_LV4		(3200) 		// 32s
#define OVER_TEMP_DERATING_SET_TIME_HYS		(100) 		// 32s
#define OVER_TEMP_DERATING_CLEAR_TIME		(12000) 	// 120s
#define STALL_DERATING_CLEAR_TIME			(800) 		// 20s

#define OTP_DE_RATE_NONE					(OTP_DERATE_NORMAL_RATE)	// 100%
#define OTP_DE_RATE_LV1						(900.0)		// 90%
#define OTP_DE_RATE_LV2						(850.0)		// 85%
#define OTP_DE_RATE_LV3						(800.0)		// 80%
#define OTP_DE_RATE_LV4						(750.0)		// 75%
#define OTP_DE_RATE_LV5						(700.0)		// 70%
#define RAMP_TIME							(2.0)
#define RAMP_TIME_DIV						(1.0/RAMP_TIME)


#define STALL_DE_RATE			(0.7)
#define STALL_HYS_TIME			(5 * EXT_DEV_CONTROL_FREQUENCY)

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct RunStall{
	uint8_t step;
	uint8_t enable;
	uint32_t est_cnt;
	uint32_t half_cnt;
	float max_time_0;
	float max_time_1;
	float max_time_2;
	float est_time;
	float curr_limit;
	float curr_target;
	int32_t tmr;
}RunStall_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskObj_t ext_dev_ctrl_task;
extern uint16_t* rawAdc3;
extern uint16_t EMCY_sw;

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	extern IOIF_UPRIGHT_t uprightObj;
	extern IOIF_LS_t lsObj;
	extern IOIF_FSR_t fsrObj;
	extern IOIF_LP_t lpObj;
#endif

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitExtDevCtrl(void);
void RunExtDevCtrl(void* params);


#endif /* APPS_EXT_DEV_CTRL_INC_EXT_DEV_CTRL_H_ */
