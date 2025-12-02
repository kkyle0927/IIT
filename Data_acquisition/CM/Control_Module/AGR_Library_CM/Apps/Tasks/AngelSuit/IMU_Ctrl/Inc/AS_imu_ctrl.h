

#ifndef APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_AS_IMU_CTRL_H_
#define APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_AS_IMU_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdbool.h>

#include "data_object_common.h"
#include "data_object_dictionaries.h"

#include "ioif_gpio_common.h"
#include "ioif_icm20608g.h"
#include "ioif_bm1422agmv.h"

#include "AS_vqf.h"

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

extern TaskObj_t imuCtrlTask;

extern IOIF_AccGyro_Data_t CM_imu6AxisDataObj;
extern IOIF_Magneto_Data_t   CM_magDataObj;

extern VQF_MagCalib_t CM_vqfMagCalibObj;

extern float roll_q, pitch_q, yaw_q;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitIMUCtrl(void);
void RunIMUCtrl(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_AS_IMU_CTRL_H_ */
