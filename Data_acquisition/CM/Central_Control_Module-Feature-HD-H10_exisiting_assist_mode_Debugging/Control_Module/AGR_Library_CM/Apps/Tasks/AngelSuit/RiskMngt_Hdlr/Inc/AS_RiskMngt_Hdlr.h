/*
 * AS_RiskMngt_Hdlr.h
 *
 *  Created on: Aug 30, 2024
 *      Author: Angelrobotics
 */

#ifndef APPS_TASKS_ANGELSUIT_RISKMNGT_HDLR_INC_AS_RISKMNGT_HDLR_H_
#define APPS_TASKS_ANGELSUIT_RISKMNGT_HDLR_INC_AS_RISKMNGT_HDLR_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

/*CDI*/
#include "data_object_common.h"
#include "risk_mngr.h"
/*IOIF*/
#include "ioif_pca9957hnmp.h"
#include "ioif_mcp79510.h"
#include "IOIF_QSPI_Flash.h"
/*App*/
#include "AS_ISI.h"
#include "AS_whole_body_ctrl.h"
#include "AS_system_ctrl.h"
#include "AS_Action.h"
#include "AS_system_log.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */
#define RM_RECOVERY_TRY 3
#define RM_CM_VOLT_TRESHOLD 25.5
#define RM_CM_CURR_TRESHOLD 13
#define RM_CM_TEMP_TRESHOLD 110
#define RM_CM_TEMP_WARNING  100
#define RM_CM_MCU_TEMP_TRESHOLD 120
#define RM_CM_MCU_TEMP_WARNING  110


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef enum {
	/* Fault Flag */
	INIT_FAULT_OCCUR = 0,
	/* Fatal Errors: Critical faults that robot should not operate */
	INIT_FAULT_LED,
    INIT_FAULT_BAT,
    INIT_FAULT_MAGNETO,
    INIT_FAULT_SDCARD,
    INIT_FAULT_SAI,
    INIT_FAULT_ESP32,
    INIT_FAULT_FDCAN_CH1,
    INIT_FAULT_FDCAN_CH2,
	INIT_FAULT_USB,
	/* Warning: Non-critical faults but should be repaired */
	INIT_FAULT_RTC,

    INIT_FAULT_COUNT
} InitFault;

typedef struct _FaultArrange_t { //Service FSM을 고려하여 작성
	uint8_t  category;
	uint8_t  number;

} FaultArrange_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */
extern FaultArrange_t fault_ID[FAULT_QUEUE_SIZE_MAX];
extern uint8_t RM_error_led_ctrl;
extern bool rm_power_off;
extern bool init_fault[INIT_FAULT_COUNT];
/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */
void InitRiskMngtHdlr(void);
void RunRiskMngtHdlr(void);

void FaultIDRegister(void* param);

#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_RISKMNGT_HDLR_INC_AS_RISKMNGT_HDLR_H_ */
