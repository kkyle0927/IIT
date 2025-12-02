/*
 * gait_ctrl_task.h
 *
 *  Created on: Oct 11, 2023
 *      Author: INVINCIBLENESS
 */

#ifndef GAIT_CTRL_INC_GAIT_CTRL_H_
#define GAIT_CTRL_INC_GAIT_CTRL_H_

#include "module.h"

/* Select WIDM ANGLE CHECK MODE */
#ifdef CM_MODULE
#define IMU_MODE
//#define IMUABS_MODE
//#define IMUINC_MODE
#endif

#ifdef MD_MODULE
//#define IMU_MODE
//#define IMUABS_MODE
#define IMUINC_MODE
#endif

#ifdef WIDM_MODULE
#define IMU_MODE
//#define IMUABS_MODE
//#define IMUINC_MODE
#endif
//////////////////////

/* Select when you want to do Quaternion test */
#ifdef CM_MODULE
//#define QUATERNION
#endif

#ifdef MD_MODULE
//#define QUATERNION
#endif

#ifdef WIDM_MODULE
#define QUATERNION
#endif
//////////////////////

#include <stdbool.h>

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "error_dictionary.h"

// For IMU //
#include "ioif_icm20608g.h"
#include "ioif_bm1422agmv.h"
#include "widm_algorithms.h"

#ifdef CM_MODULE
#include "data_object_common.h"
#ifdef WALKON5_CM_ENABLED
#include "ws_dev_mngr.h"
#endif
#ifdef L30_CM_ENABLED
#include "L30_dev_mngr.h"
#endif
#ifdef SUIT_MINICM_ENABLED
#include "as_dev_mngr.h"
#endif
#endif

#ifdef MD_MODULE
#include "mid_level_ctrl.h"
#include "low_level_ctrl.h"
#include "msg_hdlr.h"
#include "msg_common.h"
#endif

#ifdef WIDM_MODULE
#include "msg_hdlr.h"
#include "msg_common.h"
#endif

#ifdef IMUABS_MODE
#include "mid_level_ctrl.h"
#endif

#ifdef IMUINC_MODE
#include "low_level_ctrl.h"
#endif

// For Quaternion //
#ifdef QUATERNION
#include "vqf.h"
#include "spi.h"
#endif

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define NORM_CUTOFF_FREQ	3

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

extern WIDM_GaitData_t widmGaitDataObj;
extern WIDM_AngleData_t widmAngleDataObj;
extern WIDM_AttachCase_t widmAttachCaseObj;

#ifdef SUIT_MD_ENABLED
extern uint8_t B7Flag;
extern uint8_t B8Flag;
extern uint8_t B9Flag;
extern uint8_t B10Flag;
extern uint8_t B11Flag;
extern uint8_t B12Flag;
extern uint8_t B13Flag;
extern uint8_t B14Flag;
extern uint8_t B16Flag;
extern uint32_t gaitCount;

extern float incDeg;
extern float incVel;
#endif


#ifdef SUIT_MINICM_ENABLED
// RH //
extern uint8_t B1Flag_RH;  // walking
extern uint8_t B7Flag_RH;  // ~~ 0%  FB Transition
extern uint8_t B8Flag_RH;  // ~~ 50% BF Transition
extern uint8_t B9Flag_RH;  // ~~ 75% BF Moving
extern uint8_t B10Flag_RH; // ~~ 25% FB Moving

// LH //
extern uint8_t B1Flag_LH;  // walking
extern uint8_t B7Flag_LH;  // ~~ 0%  FB Transition
extern uint8_t B8Flag_LH;  // ~~ 50% BF Transition
extern uint8_t B9Flag_LH;  // ~~ 75% BF Moving
extern uint8_t B10Flag_LH; // ~~ 25% FB Moving
#endif


#ifdef QUATERNION
extern VQF_MagCalib_t vqfMagCalibObj;
#endif

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

#ifdef CM_MODULE
void InitGaitCtrl(void);
void RunGaitCtrl(void);
#endif /* CM_MODULE */

#if defined(MD_MODULE) || defined(WIDM_MODULE)
void InitGaitCtrl(void);
void RunGaitCtrl(void* params);
#endif /* MD_MODULE & WIDM_MODULE */

#endif /* GAIT_CTRL_INC_GAIT_CTRL_H_ */
