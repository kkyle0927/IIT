/**
 * @file system_ctrl_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#ifndef SYSTEM_CTRL_INC_AS_SYSTEM_CTRL_H_
#define SYSTEM_CTRL_INC_AS_SYSTEM_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdint.h>
#include <stdlib.h> // for qsort
#include "main.h"
#include "version.h"
/*CDI*/
#include "data_object_common.h"
#include "data_object_dictionaries.h"
#include "error_dictionary.h"
/*IOIF*/
#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_ltc2944.h"
#include "ioif_pca9957hnmp.h"
#include "ioif_usb_common.h"
#include "ioif_sysctrl.h"
/*App*/
#include "AS_RiskMngt_Hdlr.h"
#include "AS_whole_body_ctrl.h"
#include "AS_ISI.h"
#include "AS_ble_comm_hdlr.h"
#include "AS_data_ctrl.h"


/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define BATTERY_VOLT_MAX        24.6    // 배터리 최대 전압
#define BATTERY_VOLT_MIN        17.0    // 18V 완전 방전 19.1V 이하 Dead zone

#define DEBOUNCE_TIME           75      // 75ms의 디바운스 시간
#define DOUBLE_CLICK_TIME_MIN   100     // 더블 클릭 최소 시간 100ms
#define DOUBLE_CLICK_TIME_MAX   300     // 더블 클릭 최대 시간 300ms

#define BAT_VOLT_SAMPLE_SIZE        100 // Samples 100 times in a minute
#define BAT_VOLT_SAMPLING_INTERVAL (30 / BAT_VOLT_SAMPLE_SIZE) // Sampling 100 times in a minute, 60seconds * 1000 ms / 100 Samples.

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef enum {
	IDLE,
	UPLOAD_PROPERTIES,
	SAVE_PROPERTIES,
	DOWNLOAD_PROPERTIES,
	ELECTRICAL_SYSTEM_ID,
	BEMF_ID,
	CURRENT_BANDWIDTH_CHECK,
	AUTO_TUNING,
	ADV_FRICTION_ID,
	CAL_FRICTION_LUT,
	AGING_TEST_DONE,
	INIT_ELEC_ANGLE_ID,
	IMPORT_MES_HWFW,
	IMPORT_MES_CODE,
	FDCAN_TEST_START,
	FDCAN_TEST_RESET,
	SAVE_PROPERTIES_NeutralPosture,
} MainSequence_Enum;

typedef enum _SUIT_PWR_CTRL {
    SUIT_PWR_OFF,
    SUIT_PWR_ON
} SUITPwrCtrl;

typedef enum _GaitModeByBt_t {
    GAIT_MODE_STANDBY,
    GAIT_MODE_ASSIST,
} GaitModeByBt_t;

typedef enum _IncDecState {
    UNKNOWN_ASSIST,
    INCREASE_ASSIST,
    DECREASE_ASSIST,
    INCDEC_NUM,
} IncDecState;

typedef enum _AssistStage {
    STAGE_0 = 0,
    STAGE_1,
    STAGE_2,
    STAGE_3,
    STAGE_4,
    STAGE_5,
    STAGE_6,
    STAGE_7,
    STAGE_8,
    STAGE_9,
    STAGE_10,
    STAGE_NUM
} AssistStage;

typedef enum _SystemState {
    SYSTEM_UNKNOWN,
    SYSTEM_IDLE,
    SYSTEM_ASSIST,
    SYSTEM_FITNESS,
    SYSTEM_ERROR,
    SYSTEM_NUM,
} SystemState;

typedef enum _InitState {
    INIT_OK,
    INIT_ERROR,
    INIT_BUSY,
    INIT_TIMEOUT,
    INIT_NUM,
} InitState;

// 강남 세브란스
typedef struct _Protection{
	uint8_t CM_temp;
	uint8_t CM_curr;
	uint8_t CM_volt;
	uint8_t p_CM_temp;
	uint8_t p_CM_curr;
	uint8_t p_CM_volt;
	uint8_t MD_temp;
	uint8_t MD_curr;
	uint8_t MD_volt;
} Protection;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern uint32_t SystemCtrlLoopCnt;
extern TaskObj_t sysCtrlTask;

extern IOIF_BatMonitor_Data_t batData;
extern float medianVolt;

extern uint8_t systemStateFlag;
extern uint8_t wholeTaskFlag;
extern float batPctg;
extern AssistStage assistStage;
extern AssistStage pre_assistStage;
extern uint8_t testFlag;
extern uint8_t assistFlag;
extern float assistForcePctg;

extern uint8_t prevAssistLevelFromApp;
extern uint8_t prevAssistLevelFromButton;
extern uint8_t currentAssistLevel;

extern GaitModeByBt_t gaitModeState;

extern int voltSampleIndex;

extern uint8_t prevAssistLvfromBt;
extern uint8_t currAssistLvfromBt;

extern MainSequence_Enum MD_state;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitSysCtrl(void);
void RunSysCtrl(void);

void UpdateLED (void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* SYSTEM_CTRL_INC_AS_SYSTEM_CTRL_H_ */
