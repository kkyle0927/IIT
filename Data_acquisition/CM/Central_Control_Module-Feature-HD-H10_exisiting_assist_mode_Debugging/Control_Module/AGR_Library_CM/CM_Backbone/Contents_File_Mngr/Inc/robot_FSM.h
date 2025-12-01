#ifndef CM_BACKBONE_CCONTENTS_FILE_FSM_H_
#define CM_BACKBONE_CCONTENTS_FILE_FSM_H_

#include "module.h"

#include <stdint.h>

#include "robot_motion_map.h"

#include "ioif_flash_common.h"

#ifdef WALKON5_CM_ENABLED
#include "WS_ISI.h"
#include "WS_Action.h"
#endif /* WALKON5_CM_ENABLED */

#ifdef L30_CM_ENABLED
#include "L30_ISI.h"
#include "L30_Action.h"
#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
#include "AS_ISI.h"
#include "AS_Action.h"
#endif /* SUIT_MINICM_ENABLED */

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define NUMEL_STATE_VECTOR    8
#define MAX_N_STATE           76
#define MAX_N_TRANSITION      20


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/*typedef enum _StateID{

	STATE_INIT,
	STATE_DANCE,
	STATE_STAND,
	STATE_L_HALF_SWING,
	STATE_L_FULL_SWING,
	STATE_R_FULL_SWING,
	STATE_SQUAT,
	STATE_AIR_SIT,
	STATE_AIR_SIT2,
	STATE_SIT,
	STATE_STAND2SIT,
	STATE_SIT2STAND,
	STATE_KNEE_TEST,
	STATE_L_FOOT_UP,
	STATE_R_FOOT_UP,
	STATE_L_TILT,
	STATE_R_TILT,

	STATE_ID_NUM
}StateID;*/

typedef enum _FSMStateID_SUIT_Test {

	STATE_INIT,
	STATE_STANDBY,
	STATE_IDLE,
	STATE_ASSIST,
	STATE_STAND,
	STATE_RHS,
	STATE_LHS,
	STATE_RTS,
	STATE_LTS,

	STATE_ID_NUM
} StateID;

typedef struct _FSM_Mngr {
	uint32_t time_stamp;

	uint8_t state_curr;
	uint8_t state_prev;
} FSM_Mngr;

typedef struct _StateVector {
	uint8_t  StateID;
	uint8_t  MotionSetID;
	uint32_t ActionID;
	uint16_t TimeOut;
	uint8_t  TabletModeID;
	uint8_t  DefaultTargetStateID;
	uint8_t  ExitConditionID[MAX_N_TRANSITION];
	uint8_t  TargetStateID[MAX_N_TRANSITION];

} __attribute__((packed))StateVector;

typedef struct _FSMFileInfo {
	uint8_t     robot_id;
	uint8_t     file_version_major;
	uint8_t		file_version_minor;
	uint8_t		file_version_patch;

	StateVector vec[MAX_N_STATE];

} FSMFileInfo;

typedef uint32_t FSMFile[MAX_N_STATE][NUMEL_STATE_VECTOR][MAX_N_TRANSITION];


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern FSMFileInfo FSM_File;

extern FSM_Mngr    FSMMngrObj;

extern FSMFile FSM1_File;
extern FSMFile FSM2_File;
extern FSMFile FSM3_File;
extern FSMFile FSM4_File;
extern FSMFile FSM5_File;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Make_FSM1_Examples(void);
void Save_FSM1(void);
void Download_FSM1(void);
int Check_FSM_Save(void);

void Download_Test_FSM(FSMFile t_FSM_file);
void Init_FSM(void);
void Run_FSM(void);


#endif /* CM_BACKBONE_CCONTENTS_FILE_FSM_H_ */
