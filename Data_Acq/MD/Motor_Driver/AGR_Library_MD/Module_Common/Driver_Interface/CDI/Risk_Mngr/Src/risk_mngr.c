/*
 * risk_mngr.c
 *
 *  Created on: Aug 21, 2023
 *      Author: HyundoKim
 */


#include "risk_mngr.h"


WarningState_t		WarningStateObj;

/* Register Fault Recovery Function */
void SetRecvFaultFunc(FaultInfo_t* pFault, uint8_t ID, void (*RecvFunc))
{
	pFault->RecvFunc[ID] = RecvFunc;
}

/* Generate Fault Packet */
void FaultPacketDef(DOP_TaskID_t task, FaultInfo_t* pFault) // Ent
{
/* 1. Fault Msg Frame */
	uint32_t ulBit = 0, ulPos0 = 0, ulPos1 = 0, ulPos2 = 0;

	ulBit = pFault->faultBit;
	ulPos0 = (0x0FFFFFFF) & ulBit;
	pFault->faultBit = ulPos0;

	ulBit = (pFault->type) << FAULT_TYPE_BIT;
	ulPos1 = (0x10000000) & ulBit;

	switch(task){
		case (TASK_ID_LOWLEVEL): // 0
			ulBit =  LOWLEVEL_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_MIDLEVEL): // 1
			ulBit = MIDLEVEL_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_MSG):	// 2
			ulBit = MSG_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_GAIT): // 4
			ulBit = GAIT_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_SYSMNGT_MD): // 5
			ulBit = SYSMNGT_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_EXTDEV): // 6
			ulBit = EXTDEV_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;

		default:
			break;
	}
	ulPos2 = (0xE0000000) & ulBit;

	pFault->packet = (ulPos2|ulPos1|ulPos0);
}

/* Register Fault Information */
int SetFaultInfo(FaultInfo_t* pFault, FaultType_t type, uint8_t faultID, StateEnum_t restart_state, uint8_t taskRange)
{
	if(pFault->faultNumCnt > 0){	// ���� Cycle���� �ٸ� ���� �߻� ���� Ȯ��
		if(pFault->faultBit > 0){
			if (pFault->restart_state == TASK_STATE_OFF){
				pFault->restart_state = TASK_STATE_OFF;	// �ٸ� ���忡�� Reaction state�� OFF �����̸�, OFF ���¸� ����.
			}
			else{
				pFault->restart_state = restart_state;
			}
		}
		else{
			pFault->faultBit = 0;
			pFault->faultNumCnt = 0;
			pFault->restart_state = restart_state;
		}
	}
	else{
		pFault->faultNumCnt = 0;
		pFault->faultBit = 0;
		pFault->restart_state = restart_state;	// �ٸ� ������ ���ٸ� ���� �߻� ������ Restart state�� ������.
	}

	pFault->type &= (0x01);	// Temp, Confirm ���� Ȯ��
	pFault->type |= (0x01) & type;	// Temp, Confirm ���� Ȯ��

	pFault->faultBit &= (0x0FFFFFFF);
	pFault->faultBit |= (0x0FFFFFFF) & ((uint32_t)1U<<faultID); // FaultBit ����

	pFault->faultNumCnt += 1;

	return 0;
}

/* Erase fault Information */
int ClearFaultInfo(FaultInfo_t* pFault, uint8_t faultID)
{
	uint32_t ulTemp0 = 0, ulTemp1 = 0;

	ulTemp0 = (0x0FFFFFFF) & ((1U) << faultID);
	ulTemp1 = pFault->faultBit;

	if(pFault->type == TEMPORARY){
		if(ulTemp0 & ulTemp1){
			pFault->faultNumCnt -= 1;
			ulTemp1 &= ~ulTemp0; // FaultBit ����
			pFault->faultBit = (0x0FFFFFFF) & ulTemp1;
		}
	}
	return 0;
}

int FaultRecoveryFuncRun(FaultInfo_t* pFault)
{
	uint32_t ulTemp0 = 0;

    for (uint8_t ucTemp = 0; ucTemp < TOTAL_NUMBER_OF_FALUT; ucTemp++) {
    	ulTemp0 = (1U) << ucTemp;
    	ulTemp0 &= (0x0FFFFFFF);
    	if(pFault->faultBit & ulTemp0){
    		pFault->RecvFunc[ucTemp]();
    	}
    }
    return 0;
}

void InitFaultInfo(DOP_TaskID_t task, FaultInfo_t* pFault)
{
	uint32_t ulBit = 0;

	pFault->type = TEMPORARY; 			// Confirm : 1, Temp : 0
	pFault->faultBit = 0; 		// OR ó��
	pFault->faultNumCnt = 0;
	pFault->restart_state = TASK_STATE_STANDBY; // ���� ����
	pFault->taskRange = 0;

	switch(task){
		case (TASK_ID_LOWLEVEL): // 0
			ulBit =  LOWLEVEL_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_MIDLEVEL): // 1
			ulBit = MIDLEVEL_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_MSG):	// 2
			ulBit = MSG_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_GAIT): // 4
			ulBit = GAIT_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_SYSMNGT_MD): // 5
			ulBit = SYSMNGT_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;
		case (TASK_ID_EXTDEV): // 6
			ulBit = EXTDEV_TASK_FAULT_NUM << FAULT_TASK_BIT;
			break;

		default:
			break;
	}
	pFault->packet = (0xE0000000) & ulBit;
}

void InitFaultRecvFunc(FaultInfo_t* pFault)
{
	for (uint8_t ucTemp = 0; ucTemp < TOTAL_NUMBER_OF_FALUT; ucTemp++) {
		pFault->RecvFunc[ucTemp] = NULL;
	}
}

/* Verify Fault Information Consistency */
int FaultNumCntConsistencyCheck(FaultInfo_t* pFault)
{
	uint32_t ulTemp0 = 0;
	uint8_t ucTemp0 = 0;

    for (uint8_t ucTemp1 = 0; ucTemp1 < TOTAL_NUMBER_OF_FALUT; ucTemp1++) {
    	ulTemp0 = (1U) << ucTemp1;
    	ulTemp0 &= (0x0FFFFFFF);
    	if(pFault->faultBit & ulTemp0) ucTemp0++;
    }

    if(pFault->faultNumCnt == ucTemp0){
    	if(ucTemp0 > 0){
    		pFault->faultBitOld = pFault->faultBit;
    		return 0;
    	}
    	else{
    		pFault->faultNumCnt = 0;
    		return -1;
    	}
    }
    else{
    	if(ucTemp0 > 0){
        	pFault->faultNumCnt = ucTemp0;
    	}
    	else{
    		pFault->faultNumCnt = 0;
    		pFault->faultBit = 0;
    	}
    	return -1;
    }
}

void SetWarningInfo(WarnigList_t WaringID, WarningState_t* pWarning)
{
	uint32_t ulBit = 0, ulPos0 = 0, ulPos1 = 0, ulPos2 = 0;

	ulPos0 = (0x0FFFFFFF) & pWarning->warningBit;
	ulBit = (0x0FFFFFFF) & ((uint32_t)1U << WaringID);
	ulPos0 |= ulBit;
	pWarning->warningBit = ulPos0;

	ulPos1 = 0 << FAULT_TYPE_BIT;
	ulPos1 = (0x10000000) & ulPos1;

	ulPos2 = WARNING_NUM << FAULT_TASK_BIT;
	ulPos2 = (0xE0000000) & ulPos2;

	pWarning->packet = (ulPos2|ulPos1|ulPos0);

}

void ClearWarningInfo(WarnigList_t WaringID, WarningState_t* pWarning)
{
	uint32_t ulBit = 0, ulPos0 = 0, ulPos1 = 0, ulPos2 = 0;

	ulPos0 = (0x0FFFFFFF) & pWarning->warningBit;
	ulBit = (0x0FFFFFFF) & ((uint32_t)1U << WaringID);
	ulPos0 &= ~ulBit;
	pWarning->warningBit = ulPos0;

	ulPos1 = 0 << FAULT_TYPE_BIT;
	ulPos1 = (0x10000000) & ulPos1;

	ulPos2 = WARNING_NUM << FAULT_TASK_BIT;
	ulPos2 = (0xE0000000) & ulPos2;

	pWarning->packet = (ulPos2|ulPos1|ulPos0);
}

