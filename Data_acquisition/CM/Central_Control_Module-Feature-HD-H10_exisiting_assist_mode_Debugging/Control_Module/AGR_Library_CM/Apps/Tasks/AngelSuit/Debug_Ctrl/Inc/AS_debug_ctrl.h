

#ifndef DEBUG_CTRL_INC_AS_DEBUG_CTRL_H_
#define DEBUG_CTRL_INC_AS_DEBUG_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdint.h>
/*CDI*/
#include "data_object_common.h"
#include "cli.h"
/*IOIF*/
#include "ioif_sysctrl.h"  		//unit test - system ctrl
#include "ioif_cpu_temperature_sensing.h"
#include "ioif_ltc2944.h"       //unit test - power monitoring
#include "ioif_pca9957hnmp.h"   //unit test - led display
#include "ioif_fatfs.h"			//unit test - file system
#include "ioif_sai_common.h" 	//unit test - audio basic play
#include "ioif_sai_wavplay.h"	//unit test - audio save file play
#include "ioif_usb_common.h" 	//unit test - USB Comm.
#include "ioif_mdbt42q-at.h"    //unit test - BLE Comm.
#include "ioif_fdcan_common.h"  //unit test - FDCAN Comm.
#include "ioif_esp32.h" 		//unit test - Wifi-BT Comm.
#include "ioif_mcp79510.h"		//uint test - RTC

#include "ioif_icm20608g.h"
#include "ioif_bm1422agmv.h"
#include "ioif_tim_common.h"
/*App*/




/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IMU_6AXIS_BUFF_SIZE 32


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

//extern TaskObj_t debug_task_dvs;

extern uint32_t debug_task_loop_time;

//extern IOIF_IMU6AxisData_t imu6AxisData;
//extern IOIF_MagData_t magData;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitDebugTask(void);
void RunDebugTask(void);

#endif /* SUIT_MINICM_ENABLED */

#endif /* DEBUG_CTRL_INC_AS_DEBUG_CTRL_H_ */
