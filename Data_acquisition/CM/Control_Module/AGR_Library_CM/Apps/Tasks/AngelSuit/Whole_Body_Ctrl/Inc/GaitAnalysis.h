#ifndef APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_GAITANALYSIS_H_
#define APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_GAITANALYSIS_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "AS_whole_body_ctrl.h"
#include "AS_ble_comm_hdlr.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define TOTAL_SAMPLE_COUNT  30000   // 1ms, 30초, 30000샘플
#define DOWN_SAMPLE_COUNT   3       // Post Processing을 위한 DownSample 수 30초->10초 다운샘플
#define PP_MAX_INDEX        10000   // Post Processing 전체 샘플 수
// 보행 주기별 리샘플링 관련 상수
#define FIXED_GAIT_SAMPLES  101   // 각 보행 주기마다 리샘플할 샘플 수 (0~100%를 표현하기 위함)
#define MAX_NUM_GAIT_EVENTS 20    // 최대 보행 이벤트트 수 (5m 동작 분석 시 Heel Strike가 최대 MAX_NUM_GAIT_CYCLES번 나올 것이다 가정)
#define MAX_SEGMENT_LENGTH  5000  // 한 보행 구간의 최대 길이 (샘플 수)

#define RESAMPLE_SHIFT_SAMPLES  20  // Resample된 데이터 shift percentage (실측 데이터 기반)


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum {
    RIGHT_GAIT_EVENT,
    LEFT_GAIT_EVENT
} HEEL_STRIKE;

// 2D 벡터 구조체
typedef struct {
    float x;
    float y;
} vec2D;

// heel strike 이벤트 발생 시, 해당 이벤트 시점(샘플 번호)와 그때의 기준 위치(posX + 발 x좌표)를 저장
typedef struct {
    uint32_t prevSample;    // 바로 직전 heel strike의 샘플 번호
    float currRefPos;
    float prevRefPos;       // 그 시점의 기준 위치: posX + 발의 x 좌표
} GaitEventTracker;

// 온라인 평균 및 표준편차 계산을 위한 구조체
typedef struct {
    uint32_t n;    // 샘플 개수
    float mean;    // 현재까지 평균
    float M2;      // 분산 누적값 (variance * (n-1))
} OnlineStats;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern bool isMotionAnalysisDuring;   // 보행 분석 시작 신호: 가감속 구간 통과 후 true, 종료 시 false -> 현재는 준비 신호 true 조건에서 바로 true로 설정
extern bool isMotionAnalysisStart;   // 준비(리셋) 신호: 세션 시작 시 한 번 true로 설정됨
extern bool isMotionAnalysisFinish;  // 전체 보행 분석 종료 신호: App에서 종료 버튼을 누르거나 16걸음 걷기 시 true로 설정됨

extern uint16_t StrideCountLeft_Improved;
extern uint16_t StrideCountRight_Improved;

extern float StepLengthLeft_Improved;
extern float StepLengthRight_Improved;
extern float StrideLengthLeft_Improved;
extern float StrideLengthRight_Improved;

extern float Cadence_Improved;

extern float velXLPF_Improved[2];

extern float LeftHipROMFlex_Improved;
extern float LeftHipROMExt_Improved;
extern float RightHipROMFlex_Improved;
extern float RightHipROMExt_Improved;
extern float LeftHipMaxSpeed_Improved;
extern float RightHipMaxSpeed_Improved;
extern float leftHipROM;
extern float rightHipROM;

extern bool LeftFootContact_Improved[2];
extern bool RightFootContact_Improved[2];

extern float Theta_Trunk_Improved;
extern uint8_t calTrunkNeutralPostureCmd;
extern float TrunkNeutralPostureBias;

extern float velXKinematics_Improved[2];
extern float velX_Improved[2];
extern float posX_Improved[2];
extern float forwardDistance_Improved;

extern float ThetaLShank_Improved[3];
extern float ThetaRShank_Improved[3];

extern float ThetaLK_Improved[2];
extern float ThetaRK_Improved[2];

extern float tauLeftKnee_Improved;
extern float tauRightKnee_Improved;
extern float ThetaLShank_term_Improved;
extern float ThetaRShank_term_Improved;

extern bool isPostProcessed;
extern bool isMotionAnalysisFinishedBefore;
extern bool isMotionAnalysisFinishedForDataSave;

extern float velX4AnalysisFiltered[PP_MAX_INDEX];
extern float ThetaLH4AnalysisFiltered[PP_MAX_INDEX];
extern float ThetaRH4AnalysisFiltered[PP_MAX_INDEX];
extern float TorqueLH4AnalysisFiltered[PP_MAX_INDEX];
extern float TorqueRH4AnalysisFiltered[PP_MAX_INDEX];

extern uint32_t postProcessingCnt;

extern float CalculatedDistance;
extern float meanVelX;
extern float stdVelX;
extern float meanCadence;
extern float stdCadence;
extern float meanStepLengthLeft;
extern float meanStepLengthRight;
extern float meanStepLengthBoth;
extern float meanStrideLengthBoth;
extern float stdStrideLengthBoth;

extern uint32_t IndexLeftHeelStrikeCalibrated[MAX_NUM_GAIT_EVENTS];
extern uint32_t IndexRightHeelStrikeCalibrated[MAX_NUM_GAIT_EVENTS];
extern uint8_t numRightHeelStrike;
extern uint8_t numLeftHeelStrike;
extern uint8_t numLeftToeOff;
extern uint8_t numRightToeOff;

extern float meanResampledLeftHipAngles[FIXED_GAIT_SAMPLES];
extern float stdResampledLeftHipAngles[FIXED_GAIT_SAMPLES];
extern float meanResampledRightHipAngles[FIXED_GAIT_SAMPLES];
extern float stdResampledRightHipAngles[FIXED_GAIT_SAMPLES];
extern float meanResampledLeftHipTorques[FIXED_GAIT_SAMPLES];
extern float stdResampledLeftHipTorques[FIXED_GAIT_SAMPLES];
extern float meanResampledRightHipTorques[FIXED_GAIT_SAMPLES];
extern float stdResampledRightHipTorques[FIXED_GAIT_SAMPLES];

extern float leftStancePhase;
extern float leftSwingPhase;
extern float rightStancePhase;
extern float rightSwingPhase;
extern float asymmetryHipAngle;
extern float asymmetryStepLength;
extern float asymmetryStancePhase;
extern float asymmetrySwingPhase;

extern float finishPos;

extern uint16_t divisionByZeroCnt;

extern float ResampledLeftHipROMFlex;
extern float ResampledLeftHipROMExt;
extern float ResampledRightHipROMFlex;
extern float ResampledRightHipROMExt;

extern float leftHeelStrikeDeltaTime;

// 세션 관련 변수들
extern bool MotionAnalysisSelfStop;
extern bool MotionAnalysisFinishAble;
extern bool isOperationDuring;
extern float forwardDistance_Session;
extern uint32_t StrideCountLeft_Session;
extern uint32_t StrideCountRight_Session;
extern float StrideCount_Session;
extern uint8_t StepLengthAvg_Session;
extern float ThetaLK_Session;
extern float ThetaRK_Session;
extern float posXRaw_Session[2];
extern float posX_Session[2];

// 태블릿 연동
extern uint32_t MotionAnalysisDuration;
extern uint8_t GroundContact;
extern float MotionAnalysisMeanLeftHipMax;
extern float MotionAnalysisMeanLeftHipMin;
extern float MotionAnalysisMeanRightHipMax;
extern float MotionAnalysisMeanRightHipMin;
extern float MotionAnalysisRMSLeftHipDiff;
extern float MotionAnalysisRMSRightHipDiff;

extern float TrunkLength_Improved;
extern float ThighLength_Improved;
extern float ShankLength_Improved;
extern float AnkleLength_Improved;
extern float ThighMass_Improved;
extern float ShankMass_Improved;
extern float TrunkMass_Improved;
extern float AnkleMass_Improved;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGaitAnalysis(void);
void GaitAnalysis_Improved(void);
void GaitAnalysis_PostProcessing(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_GAITANALYSIS_H_ */
