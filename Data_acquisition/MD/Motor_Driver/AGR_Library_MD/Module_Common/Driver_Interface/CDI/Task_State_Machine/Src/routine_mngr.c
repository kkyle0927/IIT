

#include "routine_mngr.h"
#include "task_mngr.h"

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




/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

// Routine Entity
RoutineEntityFuncPtr_t CreateRoutineEntity(RoutineFncPtr ent, RoutineFncPtr run, RoutineFncPtr ext)
{
    RoutineEntityFuncPtr_t tEntityFuncPtr;
    tEntityFuncPtr.onEnter = ent;
    tEntityFuncPtr.onRun = run;
    tEntityFuncPtr.onExit = ext;
    return tEntityFuncPtr;
}


// DriveRoutine Interface
void InitRoutine(RoutineObj_t* routine)
{
    for (int i = 0; i < ROUTINE_MAX_ENTITIES; i++) {
    	routine->id[i] = 0;
    	routine->entities[i].onEnter = NULL;
    	routine->entities[i].onRun = NULL;
    	routine->entities[i].onExit = NULL;
    }
    routine->numOfRoutineID = 0;
}

int EntRoutines(RoutineObj_t* routine)
{
	int tRes = 0;
	int tempID = 0;

    for (int i = 0; i < routine->numOfRoutineID; i++){
        //TODO: routine func exception handling

    	tempID = routine->id[i];
		if (routine->entities[tempID].onEnter) {
			tRes = routine->entities[tempID].onEnter();
		}
        if (tRes < 0) {
            return tRes;
        }
    }
    return 0;
}

int RunRoutines(RoutineObj_t* routine)
{
	int tRes = 0;
	int tempID = 0;

    for (int i = 0; i < routine->numOfRoutineID; i++) {
        //TODO: routine func exception handling

    	tempID = routine->id[i];
		if (routine->entities[tempID].onRun) {
			tRes = routine->entities[tempID].onRun();
		}
        if (tRes < 0) {
            return tRes;
        }
    }
    return 0;
}

int ExtRoutines(RoutineObj_t* routine)
{
	int tRes = 0;
	int tempID = 0;

    for (int i = 0; i < routine->numOfRoutineID; i++) {
        //TODO: routine func exception handling

    	tempID = routine->id[i];
		if (routine->entities[tempID].onExit) {
			tRes = routine->entities[tempID].onExit();
		}
        if (tRes < 0) {
            return tRes;
        }
    }
    return 0;
}

// void ClearRoutines(RoutineObj_t* routine)
// {
//     for (int i = 0; i < ROUTINE_MAX_ENTITIES; i++) {
//     	routine->id[i] = ROUTINE_DEFAULT_ID;
//     }
//     routine->numOfRoutineID = 0;
// }

/**
 * @brief routine의 모든 루틴 목록을 초기화합니다.
 * @param routine: RoutineObj_t 객체의 포인터
 */
void ClearRoutines(RoutineObj_t* routine)
{
    // (권장) 목록을 비우기 전에 모든 루틴의 onExit을 먼저 호출
    ExtRoutines(routine);

    // 모든 ID를 기본값으로 리셋
    for (int i = 0; i < ROUTINE_MAX_ENTITIES; i++) {
        routine->id[i] = ROUTINE_DEFAULT_ID; // -1 과 같은 값
    }
    // 루틴 개수를 0으로 리셋
    routine->numOfRoutineID = 0;
}

/**
 * @brief  활성화된 루틴 목록에서 특정 routineID를 찾아 제거합니다.
 * @param  routine: RoutineObj_t 객체의 포인터
 * @param  routineID: 제거할 루틴의 ID
 * @return 0이면 성공, -1이면 해당 ID를 찾지 못함
 */
int PopRoutine(RoutineObj_t* routine, uint8_t routineID)
{
    int found_idx = -1;

    // 1. 제거할 루틴 ID가 목록의 몇 번째에 있는지 검색합니다.
    for (int i = 0; i < routine->numOfRoutineID; i++) {
        if (routine->id[i] == routineID) {
            found_idx = i;
            break;
        }
    }

    // 2. 목록에서 해당 루틴 ID를 찾았을 경우
    if (found_idx != -1) {
        // 해당 루틴의 onExit 함수를 호출하여 안전하게 종료시킵니다.
        if (routine->entities[routineID].onExit) {
            routine->entities[routineID].onExit();
        }

        // 3. 배열에서 빈 공간이 생기지 않도록, 제거된 위치 뒤의 모든 요소들을 한 칸씩 앞으로 당깁니다.
        for (int i = found_idx; i < routine->numOfRoutineID - 1; i++) {
            routine->id[i] = routine->id[i + 1];
        }
        
        // 4. 전체 루틴 개수를 하나 줄입니다.
        routine->numOfRoutineID--;
        return 0; // 성공
    }
    
    return -1; // 해당 루틴이 목록에 없어 제거하지 못함
}

int PushRoutine(RoutineObj_t* routine, uint8_t routineID)
{
    if (routine->numOfRoutineID >= ROUTINE_MAX_ENTITIES) {
        return -1; // 버퍼가 가득 참
    }

    // 이미 등록된 루틴인지 확인
    for (int i = 0; i < routine->numOfRoutineID; i++) {
        if(routine->id[i] == routineID){
            return 0; // 이미 있으므로 아무것도 안 함
        }
    }

    // 루틴 ID를 목록에 추가
    routine->id[routine->numOfRoutineID++] = routineID;

    // 새로 추가된 루틴의 onEnter 함수를 즉시 호출하여 초기화합니다.
    if (routine->entities[routineID].onEnter) {
        routine->entities[routineID].onEnter();
    }
    
    return 0;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */



