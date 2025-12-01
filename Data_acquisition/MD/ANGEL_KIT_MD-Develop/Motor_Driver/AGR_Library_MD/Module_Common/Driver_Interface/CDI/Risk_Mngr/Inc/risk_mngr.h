/**
 * @file risk_mngr.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief 
 * @ref
 */

#ifndef RISK_MNGR_INC_RISK_MNGR_H_
#define RISK_MNGR_INC_RISK_MNGR_H_

#include <stdint.h>
#include "task_state_machine.h"
#include "data_object_dictionaries.h"
#include "error_dictionary.h"
/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */
/* PARAM FOR ERROR FUNCTION */
#define TOTAL_NUMBER_OF_FALUT	(28U)

#define FAULT_TASK_BIT	(29U)
#define FAULT_TYPE_BIT		(28U)

#define	LOWLEVEL_TASK_FAULT_NUM			(1U)
#define	MIDLEVEL_TASK_FAULT_NUM			(2U)
#define	MSG_TASK_FAULT_NUM				(3U)
#define	GAIT_TASK_FAULT_NUM				(4U)
#define	SYSMNGT_TASK_FAULT_NUM			(5U)
#define	EXTDEV_TASK_FAULT_NUM			(6U)
#define	WARNING_NUM						(7U)

#define VDC_ADC_SCALE			(2220U) // Actual calculated scale : (2206) / Previous SVM Scale : (2365)

#ifdef SUIT_MD_ENABLED
/* FAULT DETECTION CONDITION SET VAL - LOW LEV */
// OCP //
#define FAULT_CURRENT_OCP_DEFAULT_LV	(14U)
#define FAULT_CURRENT_OCP_CNT			(5U)

// OVP //
#define FAULT_VDC_OVP_SET		(29U)
#define FAULT_VDC_OVP_CLEAR		(28U)
#define FAULT_VDC_OVP_CNT		(250U)
#define FAULT_VDC_OVP_SET_LV	(FAULT_VDC_OVP_SET*VDC_ADC_SCALE*VDC_GAIN_SCALE_ADJ) // batt_vdc ���� Current Scale�� �����ϰ� �����
#define FAULT_VDC_OVP_CLEAR_LV	(FAULT_VDC_OVP_CLEAR*VDC_ADC_SCALE*VDC_GAIN_SCALE_ADJ) // batt_vdc ���� Current Scale�� �����ϰ� �����

// UVP //
#define FAULT_VDC_UVP_SET		(14U)
#define FAULT_VDC_UVP_CLEAR		(16U)
#define FAULT_VDC_UVP_CNT		(250U)
#define FAULT_VDC_UVP_SET_LV	(FAULT_VDC_UVP_SET*VDC_ADC_SCALE*VDC_GAIN_SCALE_ADJ) // batt_vdc ���� Current Scale�� �����ϰ� �����
#define FAULT_VDC_UVP_CLEAR_LV	(FAULT_VDC_UVP_CLEAR*VDC_ADC_SCALE*VDC_GAIN_SCALE_ADJ) // batt_vdc ���� Current Scale�� �����ϰ� �����

/* FAULT DETECTION CONDITION SET VAL - EXT DEV */
// OTP //
#define FAULT_MOTOR_OTP_SET			(125U)
#define FAULT_MOTOR_OTP_CLEAR		(100U)
#define FAULT_MOTOR_OTP_CNT			(200U)

/* FAULT DETECTION CONDITION SET VAL - MID LEV */
#define FAULT_CURRENT_OFFSET_LV		(3912U)
#define FAULT_CURRENT_OFFSET_CNT	(3U)
#endif

#define CREATE_FAULT_RECOVERY(faultObj, FaultID, RecvFuncPtr)	SetRecvFaultFunc((faultObj), (FaultID), (RecvFuncPtr))
#define ABS(a)					((a) >= 0 ? (a) : -(a))
/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */


typedef enum {
	TEMPORARY,
	CONFIRMED,
}FaultType_t;

typedef struct _Faultbit_t{
	FaultType_t	type; 			// Confirm : 1, Temp : 0
	uint32_t faultBit; 		// OR ó��
	uint32_t faultBitOld;
	uint8_t faultNumCnt;
	StateEnum_t restart_state; // ���� ����
	uint8_t taskRange;
	uint32_t packet;		//
	void (*RecvFunc[TOTAL_NUMBER_OF_FALUT]) (void);
} FaultInfo_t;

typedef struct WarningState{
	uint32_t warningBit;
	uint32_t packet;
	uint8_t warningNumCnt;
}WarningState_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */


extern WarningState_t		WarningStateObj;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */
int SetFaultInfo(FaultInfo_t* pFault, FaultType_t type, uint8_t faultBit, StateEnum_t restart_state, uint8_t taskRange);
void FaultPacketDef(DOP_TaskID_t task, FaultInfo_t* pFault);
void SetRecvFaultFunc(FaultInfo_t* pFault, uint8_t ID, void* RecvFunc);
int ClearFaultInfo(FaultInfo_t* pFault, uint8_t faultID);
int FaultNumCntConsistencyCheck(FaultInfo_t* pFault);
void InitFaultInfo(DOP_TaskID_t task, FaultInfo_t* pFault);
int FaultRecoveryFuncRun(FaultInfo_t* pFault);
void SetWarningInfo(WarnigList_t WaringID, WarningState_t* pWarning);
void ClearWarningInfo(WarnigList_t WaringID, WarningState_t* pWarning);

#endif /* RISK_MNGR_INC_RISK_MNGR_H_ */
