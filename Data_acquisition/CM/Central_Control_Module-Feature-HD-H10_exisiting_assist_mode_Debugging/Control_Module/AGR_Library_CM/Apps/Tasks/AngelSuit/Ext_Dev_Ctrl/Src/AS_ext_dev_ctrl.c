

#include "AS_ext_dev_ctrl.h"

#ifdef SUIT_MINICM_ENABLED

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

TaskObj_t extDevCtrl;

uint8_t RoboWear_GPIOPin_Value_RH = 1;
uint8_t RoboWear_GPIOPin_Value_LH = 1;

uint8_t MAG_test[2] = {0, };
uint8_t MAG_res[2] = {0, };
/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

// for debug
static uint8_t testTimRes = IOIF_TIM_OK;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */
/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */

/* ----------------------- ROUTINE ------------------------ */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

static void Read_RoboWear_GPIOPin(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* --------------------- SDO CALLBACK --------------------- */
 DOP_COMMON_SDO_CB(extDevCtrl)

/* ------------------------- MAIN ------------------------- */
void InitExtDevCtrl(void)
{
	/* Init Task */
	InitTask(&extDevCtrl);
	DOPC_AssignTaskID(&extDevCtrl, TASK_IDX_EXT_DEV_CTRL);

	/* Init Device */

	/* State Definition */
	TASK_CREATE_STATE(&extDevCtrl, TASK_STATE_OFF,      NULL,              StateOff_Run,		NULL,         		 false);
	TASK_CREATE_STATE(&extDevCtrl, TASK_STATE_STANDBY,  NULL,              StateStandby_Run,	NULL,         		 true);
	TASK_CREATE_STATE(&extDevCtrl, TASK_STATE_ENABLE,   StateEnable_Ent,   StateEnable_Run, 	StateEnable_Ext,     false);
	TASK_CREATE_STATE(&extDevCtrl, TASK_STATE_ERROR,    NULL,              StateError_Run,		NULL,         		 false);

	/* Routine Definition */

	/* DOD Definition */
	//DOD
	 DOP_CreateDOD(TASK_IDX_EXT_DEV_CTRL);
	//PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_EXT_DEV_CTRL, extDevCtrl);
	//SDO
	 DOP_COMMON_SDO_CREATE(TASK_IDX_EXT_DEV_CTRL)
     DOP_CreateSDO(TASK_ID_EXTDEV, SDO_ID_EXTDEV_ROBOWEAR_MONITOR,		DOP_UINT8, 	Read_RoboWear_GPIOPin);

}

void RunExtDevCtrl(void)
{
	RunTask(&extDevCtrl);
}

/* ----------------------- FUNCTION ----------------------- */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run()
{

}

static void StateStandby_Run()
{
    StateTransition(&extDevCtrl.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent()
{
    EntRoutines(&extDevCtrl.routine);
}

static void StateEnable_Run()
{
    RunRoutines(&extDevCtrl.routine);
}

static void StateEnable_Ext()
{
    ClearRoutines(&extDevCtrl.routine);
}

static void StateError_Run(void)
{

}
/* --------------------- SDO CALLBACK --------------------- */
static void Read_RoboWear_GPIOPin(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	
	if(t_req->nodeID==6)
	{
		memcpy(&RoboWear_GPIOPin_Value_RH, t_req->data, sizeof(RoboWear_GPIOPin_Value_RH));
		if(RoboWear_GPIOPin_Value_RH == 0){
			MAG_test[0] = 1;
		}
	}
	else if(t_req->nodeID==7)
	{

		memcpy(&RoboWear_GPIOPin_Value_LH, t_req->data, sizeof(RoboWear_GPIOPin_Value_LH));

		if(RoboWear_GPIOPin_Value_LH == 0){
			MAG_test[1] = 1;
		}
	}

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

/* ----------------------- FUNCTION ----------------------- */

/* ----------------------- ROUTINE ------------------------ */

#endif /* SUIT_MINICM_ENABLED */
