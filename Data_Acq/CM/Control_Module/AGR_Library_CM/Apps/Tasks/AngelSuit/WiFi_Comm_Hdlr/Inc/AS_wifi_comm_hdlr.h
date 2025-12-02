/*
 * AS_wifi_comm_hdlr.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Angelrobotics
 */

#ifndef APPS_TASKS_ANGELSUIT_WIFI_COMM_HDLR_INC_AS_WIFI_COMM_HDLR_H_
#define APPS_TASKS_ANGELSUIT_WIFI_COMM_HDLR_INC_AS_WIFI_COMM_HDLR_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include "data_object_common.h"
#include "AS_ble_comm_hdlr.h"

#include "ioif_esp32.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitWiFiComm(void);
void RunWiFiComm(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_WIFI_COMM_HDLR_INC_AS_WIFI_COMM_HDLR_H_ */
