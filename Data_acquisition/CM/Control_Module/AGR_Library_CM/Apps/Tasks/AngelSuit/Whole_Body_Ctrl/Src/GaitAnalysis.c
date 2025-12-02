/*======================================================================
 * GaitAnalysis함수에서 개선된 보행 분석 알고리즘 (GaitAnalysis_Improved)
 *
 * 이 코드는 MATLAB 최신 버전의 보행 분석 알고리즘과 동일한 동작을 목표로 함.
 * 주요 개선 사항:
 *  1. 회전행렬 계산을 함수화하여 MATLAB의 R()와 동일하게 처리 (R_func_Improved)
 *  2. 발 접촉 판정에 MATLAB과 동일한 히스테리시스 조건(0.045, 0.05 threshold)을 적용
 *  3. GRF, 무릎 스프링–댐퍼 모델, 및 hyper-extension 보정을 MATLAB 코드의 계수 (100000, 1000, 0.07, 15, 12 등)와 동일하게 구현
 *  4. 중앙 차분법을 통한 동역학 통합에 MATLAB 보정 계수를 반영 (WHOLEBODY_CONTROL_PERIOD^2, 180/M_PI 등)
 *  5. 전방 속도 및 위치 산출에 대해 가속도 적분과 기구학적 속도 융합 (계수 1.85, 0.255, LPF 계수 등) 적용
 *  6. Heel Strike, Toe Off 등의 보행 이벤트 검출 및 세션 관리(Ready 신호, 변수 reset) 로직을 MATLAB 코드와 동일하게 구현
 *
 * 기존 버전과의 차이를 명확히 하기 위해 모든 변수와 함수 이름에 "_Improved" 접미사를 사용함.
 *======================================================================*/

#include "GaitAnalysis.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *     TYPE DEFINITIONS AND ENUMERATIONS AND VARIABLES
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

// Tablet App에서 전달받은 보행 분석 신호 (Tablet Rx Data)
bool isMotionAnalysisDuring = false;  // 보행 분석 중 신호
bool isMotionAnalysisStart = false;   // 보행 분석 시작 신호
bool isMotionAnalysisFinish = false;  // 전체 보행 분석 종료 신호: App에서 종료 버튼을 누르거나 16걸음 걷기 시 true로 설정됨

// Tablet App에서 전달받은 신체 치수 및 동역학 상수 (Tablet Rx Data)
float TrunkLength_Improved = 0.90f;	//m
float ThighLength_Improved = 0.49f;	//m
float ShankLength_Improved = 0.45f;	//m
float AnkleLength_Improved = 0.1f;	//m
float ThighMass_Improved = 10.8f;	//kg
float ShankMass_Improved = 4.4f;	//kg
float TrunkMass_Improved = 51.0f;	//kg
float AnkleMass_Improved = 2.0f;	//kg
float ShankMoment_Improved = 0.0f;  // = 0.5 * ShankMass_Improved * (ShankLength_Improved)^2

// 상체 각도
float Theta_Trunk_Improved = 0.0f;
// 고관절 각도, For Extenision Board
float Theta_Hip_RH = 0.0f;
float Theta_Hip_LH = 0.0f;

// 샘플 인덱스
uint32_t gaitDetectLoopCnt = 0;

// P0_Improved: 골반 위치 (기준점) – Tablet App에서 업데이트; 여기서는 원점으로 가정
vec2D P0_Improved = {0.0f, 0.0f};

// PT_Improved: 상체 끝 위치, 계산결과 (current만 필요)
vec2D PT_Improved = {0.0f, 0.0f};

// P1L_Improved, P1R_Improved: 좌우 허벅지 말단 위치 (무릎 근처), current만 필요
vec2D P1L_Improved = {0.0f, 0.0f};
vec2D P1R_Improved = {0.0f, 0.0f};

// P20L_Improved, P20R_Improved: 종아리 무게중심, 배열 [0]=current, [1]=prev
vec2D P20L_Improved[3] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
vec2D P20R_Improved[3] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};

// P2L_Improved, P2R_Improved: 발(ankle) 위치, current만 계산
vec2D P2L_Improved[2] = {{0.0f, 0.0f}};
vec2D P2R_Improved[2] = {{0.0f, 0.0f}};

// P2CL_Improved, P2CR_Improved: 무릎 위치 (종아리 무게중심에서 dP를 더한 값), current만 계산
vec2D P2CL_Improved[2] = {{0.0f, 0.0f}};
vec2D P2CR_Improved[2] = {{0.0f, 0.0f}};

// dPLeftShank_Improved, dPRightShank_Improved: 종아리 무게중심에서 무릎 방향 벡터, current만 계산
vec2D dPLeftShank_Improved = {0.0f, 0.0f};
vec2D dPRightShank_Improved = {0.0f, 0.0f};

// 발 접촉 상태, 배열 [0]=current, [1]=prev
bool LeftFootContact_Improved[2] = {false, false};
bool RightFootContact_Improved[2] = {false, false};

// GRF 계산
float fLGRF_x = 0.0f, fRGRF_x = 0.0f;
float fLGRF_y = 0.0f, fRGRF_y = 0.0f;

// Knee joint error and force, 배열 [0]=current, [1]=prev
vec2D errorLeft_Improved[2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
vec2D errorRight_Improved[2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
vec2D fLeftKnee_Improved = {0.0f, 0.0f};
vec2D fRightKnee_Improved = {0.0f, 0.0f};

// Knee relative angles (ThetaLK, ThetaRK) [deg], 배열 [0]=current, [1]=prev
float ThetaLK_Improved[2] = {0.0f, 0.0f};
float ThetaRK_Improved[2] = {0.0f, 0.0f};

// Shank angles (ThetaLShank, ThetaRShank) [deg], 배열 [0]=current, [1]=prev
float ThetaLShank_Improved[3] = {0.0f, 0.0f, 0.0f};
float ThetaRShank_Improved[3] = {0.0f, 0.0f, 0.0f};
// 보정용 cross term (current만)
float ThetaLShank_term_Improved = 0.0f;
float ThetaRShank_term_Improved = 0.0f;

// Knee torques
float tauLeftKnee_Improved = 0.0f, tauRightKnee_Improved = 0.0f;

// Forward speed and position (x-axis), 배열 [0]=current, [1]=prev
float velXKinematics_Improved[2] = {0.0f, 0.0f};
float velX_Improved[2] = {0.0f, 0.0f};
float velXLPF_Improved[2] = {0.0f, 0.0f};
float posXRaw_Improved[2] = {0.0f, 0.0f};
float posX_Improved[2] = {0.0f, 0.0f};

// Cumulative forward distance
float forwardDistance_Improved = 0.0f;

// Hip ROM, Hip 최대 각속도 계산
float LeftHipROMFlex_Improved = 0.0f;
float LeftHipROMExt_Improved = 0.0f;
float RightHipROMFlex_Improved = 0.0f;
float RightHipROMExt_Improved = 0.0f;
float LeftHipMaxSpeed_Improved = 0.0f;
float RightHipMaxSpeed_Improved = 0.0f;

// ====================================================================
// Gait Event Variables (보행 이벤트 검출을 위한 변수들)
// ====================================================================
// (이 변수들은 보행 이벤트(Heel Strike, Toe Off, 보폭, 보폭 측정 모드 등)를 계산하는 데 사용됩니다.)

// 보행 수
uint16_t StrideCountLeft_Improved = 0;   // 왼발 보행 수
uint16_t StrideCountRight_Improved = 0;  // 오른발 보행 수

// 좌측과 우측 heel strike 이벤트 추적 변수
static GaitEventTracker leftHeelTracker = { 0, 0.0f, 0.0f };
static GaitEventTracker rightHeelTracker = { 0, 0.0f, 0.0f };

// 보폭 및 반걸음 보폭 관련 변수
float StrideLengthLeft_Improved = 0.0f;
float StrideLengthRight_Improved = 0.0f;
float StepLengthLeft_Improved = 0.0f;
float StepLengthRight_Improved = 0.0f;
uint16_t StepLengthCountLeft_Improved = 0;
uint16_t StepLengthCountRight_Improved = 0;
float leftHeelStrikeDeltaTime = 0;
uint16_t divisionByZeroCnt = 0;
float Cadence_Improved = 0.0f;
bool StepLengthLeftInMeasure_Improved = false;
bool StepLengthRightInMeasure_Improved = false;
float ReferencePositionLeftFoot_Improved = 0.0f;
float ReferencePositionRightFoot_Improved = 0.0f;

float stride_time = 0.0f;
float LeftStepTime = 0.0f;  // L → R 시간 (초)
float RightStepTime = 0.0f; // R → L 시간 (초)
float Cadence_both_display = 0.0f;  // 양발 기준 Cadence 저장

/* ------------------- Neutral Posture Calculation ------------------- */
uint32_t calTrunkNeutralPostureCnt = 0;
uint8_t calTrunkNeutralPostureCmd = 0;
uint32_t trunkNeutralizedFlag = 0;
float tAccumulateTrunkAngle = 0.0;
float TrunkNeutralPostureBias = 0.0f;

/* Recovery Logic */
bool recoverFlag = false;
float prev_RH_IMU_angle = 0.0f;
float prev_LH_IMU_angle = 0.0f;

/* ------------------- Post Processing ------------------- */
bool PostProcessingStart = false;
bool isPostProcessed = false;
bool isMotionAnalysisFinishedBefore = true;
bool isMotionAnalysisFinishedForDataSave = false;

float ZPF_TempArray[PP_MAX_INDEX] __attribute__((section(".ZPF_TempArray"))) = {0};
float velX4AnalysisFiltered[PP_MAX_INDEX] __attribute__((section(".velX4AnalysisFiltered"))) = {0};
float ThetaRH4AnalysisFiltered[PP_MAX_INDEX] __attribute__((section(".ThetaRH4AnalysisFiltered"))) = {0};
float ThetaLH4AnalysisFiltered[PP_MAX_INDEX] __attribute__((section(".ThetaLH4AnalysisFiltered"))) = {0};
float TorqueRH4AnalysisFiltered[PP_MAX_INDEX] __attribute__((section(".TorqueRH4AnalysisFiltered"))) = {0};
float TorqueLH4AnalysisFiltered[PP_MAX_INDEX] __attribute__((section(".TorqueLH4AnalysisFiltered"))) = {0};
float postProcessingCntFiltered[PP_MAX_INDEX] __attribute__((section(".postProcessingCntFiltered"))) = {0};
float segmentBuffer[MAX_SEGMENT_LENGTH] __attribute__((section(".segmentBuffer"))) = {0};

float tempVelX = 0.0f;
float tempThetaLH = 0.0f;
float tempThetaRH = 0.0f;
float tempTorqueLH = 0.0f;
float tempTorqueRH = 0.0f;

uint8_t sampleCnt_VelX = 0;
uint8_t sampleCnt_ThetaLH = 0;
uint8_t sampleCnt_ThetaRH = 0;
uint8_t sampleCnt_TorqueLH = 0;
uint8_t sampleCnt_TorqueRH = 0;

uint16_t sampleIndex_PostProcessingCnt = 0;
uint16_t sampleIndex_VelX = 0;
uint16_t sampleIndex_ThetaLH = 0;
uint16_t sampleIndex_ThetaRH = 0;
uint16_t sampleIndex_TorqueLH = 0;
uint16_t sampleIndex_TorqueRH = 0;

uint32_t postProcessingCnt = 0;

// Online
OnlineStats cadenceStats = {0, 0.0f, 0.0f};
float meanCadence = 0.0f;
float stdCadence = 0.0f;

OnlineStats stepLengthLeftStats = {0, 0.0f, 0.0f};
float meanStepLengthLeft = 0.0f;
OnlineStats stepLengthRightStats = {0, 0.0f, 0.0f};
float meanStepLengthRight = 0.0f;
float meanStepLengthBoth = 0.0f;

OnlineStats strideLengthLeftStats = {0, 0.0f, 0.0f};
float meanStrideLengthLeft = 0.0f;
float stdStrideLengthLeft = 0.0f;
OnlineStats strideLengthRightStats = {0, 0.0f, 0.0f};
float meanStrideLengthRight = 0.0f;
float stdStrideLengthRight = 0.0f;
float meanStrideLengthBoth = 0.0f;
float stdStrideLengthBoth = 0.0f;

// Offline
float meanVelX = 0.0f;
float stdVelX = 0.0f;
float meanThetaRH = 0.0f;
float stdThetaRH = 0.0f;
float meanThetaLH = 0.0f;
float stdThetaLH = 0.0f;
float meanTorqueRH = 0.0f;
float stdTorqueRH = 0.0f;
float meanTorqueLH = 0.0f;
float stdTorqueLH = 0.0f;

// Real Distance 보정
float RealDistance = 5.0f; // 5m
float finishPos = 0.0f;
float CalculatedDistance = 0.0f;
float Distance_SF = 0.0f;

// 보행 이벤트 인덱스 (다운샘플 전 인덱스; 동작분석 시작 후 저장된 절대 인덱스)
// uint32_t IndexLeftHeelStrike[MAX_NUM_GAIT_EVENTS] __attribute__((section(".IndexLeftHeelStrike"))) = {0};    // 왼발 Heel Strike 수
// uint32_t IndexRightHeelStrike[MAX_NUM_GAIT_EVENTS] __attribute__((section(".IndexRightHeelStrike"))) = {0};   // 오른발 Heel Strike 수
uint32_t IndexLeftHeelStrike = 0;   // 왼발 Heel Strike 시점 기록(Realtime용)
uint32_t IndexRightHeelStrike = 0;  // 오른발 Heel Strike 시점 기록(Realtime용)
uint32_t IndexLeftToeOff = 0;       // 왼발 Toe Off 시점 기록(Realtime용)
uint32_t IndexRightToeOff = 0;      // 오른발 Toe Off 시점 기록(Realtime용)

uint32_t IndexLeftHeelStrikeCalibrated[MAX_NUM_GAIT_EVENTS] __attribute__((section(".IndexLeftHeelStrikeCalibrated"))) = {0};    // 왼발 Heel Strike 수
uint32_t IndexLeftToeOffCalibrated[MAX_NUM_GAIT_EVENTS] __attribute__((section(".IndexLeftToeOffCalibrated"))) = {0};        // 왼발 Toe Off 수
uint8_t numLeftHeelStrike = 0;  // 왼발 Heel Strike 이벤트 수
uint8_t numLeftToeOff = 0;      // 왼발 Toe Off 이벤트 수
uint32_t IndexRightHeelStrikeCalibrated[MAX_NUM_GAIT_EVENTS] __attribute__((section(".IndexRightHeelStrikeCalibrated"))) = {0};   // 오른발 Heel Strike 수
uint32_t IndexRightToeOffCalibrated[MAX_NUM_GAIT_EVENTS] __attribute__((section(".IndexRightToeOffCalibrated"))) = {0};       // 오른발 Toe Off 수
uint8_t numRightHeelStrike = 0; // 오른발 Heel Strike 이벤트 수
uint8_t numRightToeOff = 0;     // 오른발 Toe Off 이벤트 수

// 결과를 저장할 2차원 배열 (보행 주기별 리샘플링 결과)
// 각 보행 주기별 리샘플된 데이터가 FIXED_GAIT_SAMPLES 크기로 저장됨
float ResampledLeftHipAngles[MAX_NUM_GAIT_EVENTS][FIXED_GAIT_SAMPLES] __attribute__((section(".ResampledLeftHipAngles"))) = {0};
float ResampledRightHipAngles[MAX_NUM_GAIT_EVENTS][FIXED_GAIT_SAMPLES] __attribute__((section(".ResampledRightHipAngles"))) = {0};
float ResampledLeftHipTorques[MAX_NUM_GAIT_EVENTS][FIXED_GAIT_SAMPLES] __attribute__((section(".ResampledLeftHipTorques"))) = {0};
float ResampledRightHipTorques[MAX_NUM_GAIT_EVENTS][FIXED_GAIT_SAMPLES] __attribute__((section(".ResampledRightHipTorques"))) = {0};

float meanResampledLeftHipAngles[FIXED_GAIT_SAMPLES] __attribute__((section(".meanResampledLeftHipAngles"))) = {0};
float stdResampledLeftHipAngles[FIXED_GAIT_SAMPLES] __attribute__((section(".stdResampledLeftHipAngles"))) = {0};
float meanResampledRightHipAngles[FIXED_GAIT_SAMPLES] __attribute__((section(".meanResampledRightHipAngles"))) = {0};
float stdResampledRightHipAngles[FIXED_GAIT_SAMPLES] __attribute__((section(".stdResampledRightHipAngles"))) = {0};
float meanResampledLeftHipTorques[FIXED_GAIT_SAMPLES] __attribute__((section(".meanResampledLeftHipTorques"))) = {0};
float stdResampledLeftHipTorques[FIXED_GAIT_SAMPLES] __attribute__((section(".stdResampledLeftHipTorques"))) = {0};
float meanResampledRightHipTorques[FIXED_GAIT_SAMPLES] __attribute__((section(".meanResampledRightHipTorques"))) = {0};
float stdResampledRightHipTorques[FIXED_GAIT_SAMPLES] __attribute__((section(".stdResampledRightHipTorques"))) = {0};

float cutOffFreq_ForResampledData = 0.0f;

float ResampledLeftHipROMFlex = 0.0f;
float ResampledLeftHipROMExt = 0.0f;
float ResampledRightHipROMFlex = 0.0f;
float ResampledRightHipROMExt = 0.0f;

// 보행 주기별 결과를 저장할 변수들 (최종 평균값)
uint8_t numOfLeftCycles = 0;    // 왼쪽 보행 주기
uint8_t numOfRightCycles = 0;   // 오른쪽 보행 주기기
uint8_t numOfCycles = 0;        // 전체 보행 주기
float rightStancePhase = 0.0f;     // 오른쪽 입각기 평균 비율 (%)
float rightSwingPhase = 0.0f;      // 오른쪽 유유각기 평균 비율 (%)
float leftStancePhase = 0.0f;      // 왼쪽 입각기 평균 비율 (%)
float leftSwingPhase = 0.0f;       // 왼쪽 유각기 평균 비율 (%)
float leftSwingStartPhase = 0.0f;       // 왼쪽 유각기 시작 평균 비율 (%)
float rightStanceStartPhase = 0.0f;     // 오른쪽 입각기 시작 평균 비율 (%)
float rightSwingStartPhase = 0.0f;      // 오른쪽 유각기 시작 평균 비율 (%)
float leftHipROM = 0.0f;            // 왼쪽 엉덩관절 가동 범위
float rightHipROM = 0.0f;           // 오른쪽 엉덩관절 가동 범위
float maxHipROM = 0.0f;             // 양쪽 중 큰 엉덩관절 가동 범위
float meanHipROM = 0.0f;			// 양쪽 엉덩관절 가동 범위 평균

// 비대칭 지수 계산
float asymmetryStepLength = 0.0f;
float asymmetryHipAngle = 0.0f;
float asymmetryStancePhase = 0.0f;
float asymmetrySwingPhase = 0.0f;

// Step Time, Step Frequency, meanStepFreq 계산을 함수로 호출
float stepTime[MAX_NUM_GAIT_EVENTS * 2 - 1];
float stepFrequency[MAX_NUM_GAIT_EVENTS * 2 - 1];
float meanStepFreq = 0.0f;

// 세션 관련 변수들
bool MotionAnalysisSelfStop = false;
bool MotionAnalysisFinishAble = false;
bool isOperationDuring = false;
float forwardDistance_Session = 0.0f;
uint32_t StrideCountLeft_Session = 0;   // 왼발 보행 수
uint32_t StrideCountRight_Session = 0;  // 오른발 보행 수
float StrideCount_Session = 0;
uint8_t StepLengthAvg_Session = 0;	// cm 단위
float ThetaLK_Session = 0.0f;
float ThetaRK_Session = 0.0f;
float posXRaw_Session[2] = {0.0f, 0.0f};
float posX_Session[2] = {0.0f, 0.0f};

// 태블릿 연동
uint32_t MotionAnalysisDuration = 0;
uint8_t GroundContact = 0;
float MotionAnalysisMeanLeftHipMax = 0.0f;
float MotionAnalysisMeanLeftHipMin = 0.0f;
float MotionAnalysisMeanRightHipMax = 0.0f;
float MotionAnalysisMeanRightHipMin = 0.0f;

float SumLeftHipAnglesDiff = 0.0f;
float SumRightHipAnglesDiff = 0.0f;
float normalSubjectHipAngles[FIXED_GAIT_SAMPLES] = {
	    38.84458, 38.3064, 37.70357, 37.05658, 36.38629, 35.71223, 35.04904, 34.40113, 33.75981, 33.11087,
	    32.44376, 31.75622, 31.04884, 30.3167, 29.54536, 28.71689, 27.81911, 26.8505, 25.81988, 24.62949,
	    24.62949, 23.40195, 22.25776, 21.08322, 19.86974, 18.611, 17.30721, 15.96327, 14.5892, 13.19673,
	    11.79772, 10.40472, 9.029638, 7.683899, 6.378007, 5.120625, 3.917369, 2.770071, 1.677437, 0.637037,
	    -0.35313, -1.29407, -2.18593, -3.02778, -3.81786, -4.55518, -5.23942, -5.87344, -6.46281, -7.01629,
	    -7.54317, -8.05057, -8.5421, -9.016, -9.46662, -9.88425, -10.2563, -10.5661, -10.7914, -10.9069,
	    -10.8874, -10.7126, -10.3691, -9.84665, -9.13525, -8.2288, -7.12645, -5.83397, -4.36289, -2.72923,
	    -0.95124, 0.956616, 2.982006, 5.11284, 7.332737, 9.622552, 11.95786, 14.30923, 16.64821, 18.95114,
	    21.19799, 23.37033, 25.45079, 27.422, 29.26758, 30.97417, 32.53294, 33.93909, 35.19183, 36.29151,
	    37.23985, 38.03997, 38.69668, 39.21561, 39.60246, 39.86201, 39.99678, 40.00672, 39.89105, 39.65233,
	    39.29913
};
float MotionAnalysisRMSLeftHipDiff = 0.0f;
float MotionAnalysisRMSRightHipDiff = 0.0f;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void DetectGaitEvents(vec2D* P2L, vec2D* P2R, uint32_t loopCnt);

static void CalTrunkNeutralPosture(void);

static void RecoverTotalAlgorithm(void);
static void DetectAngleSingularity(void);

/* Post Processing */
static void UpdateForPostProcessingData(void);

static inline void AccumulateDownsample_ForCnt(const float current_value, uint16_t *sampleIndex, float *filtered, uint16_t maxIndex);
static inline void AccumulateDownsample(const float current_value, float *temp,
                                        uint8_t *sampleCnt, uint16_t *sampleIndex,
                                        float *filtered, uint16_t maxIndex);
static inline void FinalizeDownsample(float *temp, uint8_t *sampleCnt, uint16_t *sampleIndex,
                                        float *filtered, uint16_t maxIndex);

static void ZeroPhaseFilter(float *data, int N, float CutOffFrequency);
static void ZeroPhaseFilter2D(float data[][FIXED_GAIT_SAMPLES], int numGaitEvents, int numSamples, float CutOffFrequency);

static void OnlineStats_Init(OnlineStats *stats);
static void UpdateOnlineStats(OnlineStats *stats, float new_val);
static float OnlineStatsGetStd(const OnlineStats *stats);
static void OfflineStatsGetMeanNStd(const float *data, int n, float *mean, float *std, float scale_factor);
static float CombineMean(float m1, float m2);
static float CombineStd(float m1, float std1, float m2, float std2);

static void LinearResample_Segment(const float *input, uint32_t input_length, float *output, uint32_t output_length);
static void ResampleGaitCycleSegment(const float *input, const uint32_t *heelIndices, 
                                    uint8_t numHeelStrike, float output[][FIXED_GAIT_SAMPLES],
                                    float *segmentBuffer);
static void ShiftGaitCycle(float input[][FIXED_GAIT_SAMPLES], uint8_t numCycles, uint8_t shift_samples, float output[][FIXED_GAIT_SAMPLES]);

static void UpdateHeelStrikeIndices(uint8_t isLeft, uint32_t *IndexLeftHeelStrike, uint32_t *IndexRightHeelStrike, 
                                    uint8_t *numLeftHeelStrike, uint8_t *numRightHeelStrike, 
									const uint32_t *currentDownSampledIndex);
static void UpdateToeOffIndices(uint8_t isLeft, uint32_t *IndexLeftToeOff, uint32_t *IndexRightToeOff, 
                                uint8_t *numLeftToeOff, uint8_t *numRightToeOff, 
								const uint32_t *currentDownSampledIndex);
static inline void AlignEventIndices(float temp, uint8_t sampleCnt, uint16_t sampleIndex, 
                                        float *filtered, uint16_t maxIndex);

static void ComputeResampledStats(const float data[][FIXED_GAIT_SAMPLES],
                                    int numRows,
									float mean_out[FIXED_GAIT_SAMPLES],
									float std_out[FIXED_GAIT_SAMPLES]);

static void ComputeNumCycles(uint8_t numLeftHeel, uint8_t numLeftToe,
                            uint8_t numRightHeel, uint8_t numRightToe,
                            uint8_t *numOfLeftCycles, uint8_t *numOfRightCycles, uint8_t *numOfCycles);
static void ComputeGaitPhase(const uint32_t *IndexLeftHeel, const uint32_t *IndexLeftToe,
                                const uint32_t *IndexRightHeel, const uint32_t *IndexRightToe,
                                const uint8_t *numCycles,
                                float *meanLeftSwing, float *meanRightStance, float *meanRightSwing);

static void ComputeGaitPhaseRatios(
    const uint32_t* IndexLeftHeelStrikeCalibrated, const uint32_t* IndexLeftToeOffCalibrated,
    const uint32_t* IndexRightHeelStrikeCalibrated, const uint32_t* IndexRightToeOffCalibrated,
    uint8_t numLeftHeelStrike, uint8_t numLeftToeOff,
    uint8_t numRightHeelStrike, uint8_t numRightToeOff,
    float* leftStancePhase, float* leftSwingPhase,
    float* rightStancePhase, float* rightSwingPhase
);

static float ComputeAsymmetryIndex(float leftValue, float rightValue);

static float GetMinValue(float arr[FIXED_GAIT_SAMPLES]);
static float GetMaxValue(float arr[FIXED_GAIT_SAMPLES]);

static void ComputeStepFrequency(
    uint32_t* leftHS, uint8_t numLeftHS,
    uint32_t* rightHS, uint8_t numRightHS,
    float* stepTime, float* stepFrequency, float* meanStepFreq
);

static void SessionInfoUpdate(void);
static void ResetAllDisplayedValuesToZero(void);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void InitGaitAnalysis(void)
{
    memset(ZPF_TempArray, 0, sizeof(ZPF_TempArray));
    memset(velX4AnalysisFiltered, 0, sizeof(velX4AnalysisFiltered));
    memset(ThetaRH4AnalysisFiltered, 0, sizeof(ThetaRH4AnalysisFiltered));
    memset(ThetaLH4AnalysisFiltered, 0, sizeof(ThetaLH4AnalysisFiltered));
    memset(TorqueRH4AnalysisFiltered, 0, sizeof(TorqueRH4AnalysisFiltered));
    memset(TorqueLH4AnalysisFiltered, 0, sizeof(TorqueLH4AnalysisFiltered));
    memset(postProcessingCntFiltered, 0, sizeof(postProcessingCntFiltered));
    memset(segmentBuffer, 0, sizeof(segmentBuffer));
    
    memset(IndexLeftHeelStrikeCalibrated, 0, sizeof(IndexLeftHeelStrikeCalibrated));
    memset(IndexLeftToeOffCalibrated, 0, sizeof(IndexLeftToeOffCalibrated));
    memset(IndexRightHeelStrikeCalibrated, 0, sizeof(IndexRightHeelStrikeCalibrated));
    memset(IndexRightToeOffCalibrated, 0, sizeof(IndexRightToeOffCalibrated));
    
    memset(meanResampledLeftHipAngles, 0, sizeof(meanResampledLeftHipAngles));
    memset(stdResampledLeftHipAngles, 0, sizeof(stdResampledLeftHipAngles));
    memset(meanResampledRightHipAngles, 0, sizeof(meanResampledRightHipAngles));
    memset(stdResampledRightHipAngles, 0, sizeof(stdResampledRightHipAngles));
    memset(meanResampledLeftHipTorques, 0, sizeof(meanResampledLeftHipTorques));
    memset(stdResampledLeftHipTorques, 0, sizeof(stdResampledLeftHipTorques));
    memset(meanResampledRightHipTorques, 0, sizeof(meanResampledRightHipTorques));
    memset(stdResampledRightHipTorques, 0, sizeof(stdResampledRightHipTorques));

    memset(ResampledLeftHipAngles, 0, sizeof(ResampledLeftHipAngles));
    memset(ResampledRightHipAngles, 0, sizeof(ResampledRightHipAngles));
    memset(ResampledLeftHipTorques, 0, sizeof(ResampledLeftHipTorques));
    memset(ResampledRightHipTorques, 0, sizeof(ResampledRightHipTorques));
}

void GaitAnalysis_Improved(void)
{
    // IMU Singularity에 의한 오류 발생시 Recovery Logic
	RecoverTotalAlgorithm();

    // 초기값 설정
    static bool isFirstTime = true;
    if (isFirstTime) {
        ThetaLShank_Improved[0] = LH_Sagittal.ThighSagittalAngle;
        ThetaLShank_Improved[1] = LH_Sagittal.ThighSagittalAngle;
        ThetaLShank_Improved[2] = LH_Sagittal.ThighSagittalAngle;
        ThetaRShank_Improved[0] = RH_Sagittal.ThighSagittalAngle;
        ThetaRShank_Improved[1] = RH_Sagittal.ThighSagittalAngle;
        ThetaRShank_Improved[2] = RH_Sagittal.ThighSagittalAngle;

        isFirstTime = false;
    }

    // 동작 분석 알고리즘 수행
    // (B) Shank Moment, Theta Trunk 계산
    CalTrunkNeutralPosture();
    ShankMoment_Improved = 0.5 * ShankMass_Improved * ShankLength_Improved * ShankLength_Improved;
    Theta_Trunk_Improved = PelvicSagittalAngle;
    // For Extension Board
    Theta_Hip_RH = Theta_Trunk_Improved + RH_Sagittal.ThighSagittalAngle;
    Theta_Hip_LH = Theta_Trunk_Improved + LH_Sagittal.ThighSagittalAngle;

    // (C) Kinematic Relationship
    // (C-1) 상체 중심: PT_Improved = P0_Improved + R(Theta_Trunk_Improved)*[0; TrunkLength_Improved]
    PT_Improved.x = -sinf(Theta_Trunk_Improved * M_PI/180.0f) * TrunkLength_Improved;
    PT_Improved.y = cosf(Theta_Trunk_Improved * M_PI/180.0f) * TrunkLength_Improved;
    
    // (C-2) 허벅지 말단: 좌측, 우측
    P1L_Improved.x = -sinf(LH_Sagittal.ThighSagittalAngle * M_PI/180.0f) * -ThighLength_Improved;
    P1L_Improved.y = cosf(LH_Sagittal.ThighSagittalAngle * M_PI/180.0f) * -ThighLength_Improved;
    P1R_Improved.x = -sinf(RH_Sagittal.ThighSagittalAngle * M_PI/180.0f) * -ThighLength_Improved;
    P1R_Improved.y = cosf(RH_Sagittal.ThighSagittalAngle * M_PI/180.0f) * -ThighLength_Improved;
    // (C-3) 종아리 무게중심에서 무릎 관절을 가리키는 벡터: dP = R(ThetaShank)*[0; ShankLength_Improved*0.5f]
    dPLeftShank_Improved.x = -sinf(ThetaLShank_Improved[0] * M_PI/180.0f) * ShankLength_Improved/2.0f;
    dPLeftShank_Improved.y = cosf(ThetaLShank_Improved[0] * M_PI/180.0f) * ShankLength_Improved/2.0f;
    dPRightShank_Improved.x = -sinf(ThetaRShank_Improved[0] * M_PI/180.0f) * ShankLength_Improved/2.0f;
    dPRightShank_Improved.y = cosf(ThetaRShank_Improved[0] * M_PI/180.0f) * ShankLength_Improved/2.0f;
    
    // (C-4) 발 및 무릎 위치 계산
    // 좌측:
    P2L_Improved[0].x = P20L_Improved[0].x - dPLeftShank_Improved.x;
    P2L_Improved[0].y = P20L_Improved[0].y - dPLeftShank_Improved.y;
    P2CL_Improved[0].x = P20L_Improved[0].x + dPLeftShank_Improved.x;
    P2CL_Improved[0].y = P20L_Improved[0].y + dPLeftShank_Improved.y;
    // 우측:
    P2R_Improved[0].x = P20R_Improved[0].x - dPRightShank_Improved.x;
    P2R_Improved[0].y = P20R_Improved[0].y - dPRightShank_Improved.y;
    P2CR_Improved[0].x = P20R_Improved[0].x + dPRightShank_Improved.x;
    P2CR_Improved[0].y = P20R_Improved[0].y + dPRightShank_Improved.y;

    // (D) Foot Contact Detection (Hysteresis Filter)
    if ((!LeftFootContact_Improved[1] || !RightFootContact_Improved[1]) &&
        // (fabs(P2L_Improved[0].y - P2R_Improved[0].y) < 0.045f)) {
        // (fabs(P2L_Improved[0].y - P2R_Improved[0].y) < 0.029f)) {
        (fabs(P2L_Improved[0].y - P2R_Improved[0].y) < 0.0172f)) {
        LeftFootContact_Improved[0] = true;
        RightFootContact_Improved[0] = true;
    }
    // if (fabs(P2L_Improved[0].y - P2R_Improved[0].y) > 0.05f) {
    // if (fabs(P2L_Improved[0].y - P2R_Improved[0].y) > 0.04f) {
    if (fabs(P2L_Improved[0].y - P2R_Improved[0].y) > 0.03f) {
        if (P2L_Improved[0].y > P2R_Improved[0].y) {
            RightFootContact_Improved[0] = true;
            LeftFootContact_Improved[0] = false;
        } else {
            LeftFootContact_Improved[0] = true;
            RightFootContact_Improved[0] = false;
        }
    }

    // (E) GRF 계산 (x 방향만, factor -200)
    if (LeftFootContact_Improved[0] == true) {
        fLGRF_x = -200.0f * (P2L_Improved[0].x - P2L_Improved[1].x) / WHOLEBODY_CONTROL_PERIOD;
        fLGRF_y = 0.0f;
    } else {
        fLGRF_x = 0.0;
        fLGRF_y = 0.0f;
    }
    if (RightFootContact_Improved[0] == true) {
        fRGRF_x = -200.0f * (P2R_Improved[0].x - P2R_Improved[1].x) / WHOLEBODY_CONTROL_PERIOD;
        fRGRF_y = 0.0f;
    } else {
        fRGRF_x = 0.0f;
        fRGRF_y = 0.0f;
    }

    // (F) Knee Joint Model: error = P1 - P2C, then force = 100000*error + 1000*(error-current - error-prev)/WHOLEBODY_CONTROL_PERIOD
    //   e(k) = P1L(k) - P2CL(k) ∈ R^2, 즉 허벅지 말단 위치와 종아리 무게중심 기준 무릎 위치의 차이
    //   f_knee(k) = 100000 * e(k) + 1000 * (e(k) - e(k-1)) / dt
    // 여기서 fLeftKnee와 fRightKnee는 각각 2차원 벡터로, x, y 성분을 별도로 계산한다.

    // (F-1) 현재(인덱스 [0])의 error 계산:
    static bool isFirstError = true;
    if (isFirstError) {
        errorLeft_Improved[0].x = 0.00001f;
        errorLeft_Improved[0].y = 0.00001f;
        errorRight_Improved[0].x = 0.00001f;
        errorRight_Improved[0].y = 0.00001f;

        errorLeft_Improved[1].x = 0.00001f;
        errorLeft_Improved[1].y = 0.00001f;
        errorRight_Improved[1].x = 0.00001f;
        errorRight_Improved[1].y = 0.00001f;
        isFirstError = false;
    } else {
        errorLeft_Improved[0].x = P1L_Improved.x - P2CL_Improved[0].x;
        errorLeft_Improved[0].y = P1L_Improved.y - P2CL_Improved[0].y;
        errorRight_Improved[0].x = P1R_Improved.x - P2CR_Improved[0].x;
        errorRight_Improved[0].y = P1R_Improved.y - P2CR_Improved[0].y;
    }

    // (F-2) 관절 상호작용력 (스프링-댐핑 힘) 계산:
    // 100,000은 스프링 강성 계수, 1,000은 댐핑 계수, WHOLEBODY_CONTROL_PERIOD는 dt (여기서는 WHOLEBODY_CONTROL_PERIOD)
    // f_knee = 100000 * error + 1000 * (error - error_prev) / WHOLEBODY_CONTROL_PERIOD;
    fLeftKnee_Improved.x = 100000.0f * errorLeft_Improved[0].x +
                            1000.0f * (errorLeft_Improved[0].x - errorLeft_Improved[1].x) / WHOLEBODY_CONTROL_PERIOD;
    fLeftKnee_Improved.y = 100000.0f * errorLeft_Improved[0].y +
                            1000.0f * (errorLeft_Improved[0].y - errorLeft_Improved[1].y) / WHOLEBODY_CONTROL_PERIOD;
    fRightKnee_Improved.x = 100000.0f * errorRight_Improved[0].x +
                                1000.0f * (errorRight_Improved[0].x - errorRight_Improved[1].x) / WHOLEBODY_CONTROL_PERIOD;
    fRightKnee_Improved.y = 100000.0f * errorRight_Improved[0].y +
                                1000.0f * (errorRight_Improved[0].y - errorRight_Improved[1].y) / WHOLEBODY_CONTROL_PERIOD;

    // (G) Knee Relative Angles and Torque Calculation:
    // 기본 토크는 각속도에 비례하는 선형 마찰항으로 계산:
    //   tauLeftKnee = -0.070*(ThetaLK(k) - ThetaLK(k-1))/dt;
    //   tauRightKnee = -0.070*(ThetaRK(k) - ThetaRK(k-1))/dt;
    // 추가적으로, 무릎이 0도 이하 (hyper-extension) 이고 각속도가 음수이면, 
    // knee locking 효과를 모사하기 위해 스프링 및 댐핑 항 (계수 15, 12)을 추가

    // (G-1) 무릎 상대 각도 계산 (현재 값: 인덱스 [0])
    ThetaLK_Improved[0] = LH_Sagittal.ThighSagittalAngle - ThetaLShank_Improved[0];
    ThetaRK_Improved[0] = RH_Sagittal.ThighSagittalAngle - ThetaRShank_Improved[0];

    // (G-2) 기본 선형 댐핑 토크 계산
    static bool isFirstKneeCal = true;
    if (isFirstKneeCal) {
        ThetaLK_Improved[1] = ThetaLK_Improved[0];
        ThetaRK_Improved[1] = ThetaRK_Improved[0];
        isFirstKneeCal = false;
    }
    tauLeftKnee_Improved = -0.070f * ((ThetaLK_Improved[0] - ThetaLK_Improved[1]) / WHOLEBODY_CONTROL_PERIOD);
    tauRightKnee_Improved = -0.070f * ((ThetaRK_Improved[0] - ThetaRK_Improved[1]) / WHOLEBODY_CONTROL_PERIOD);

    // (G-3) knee locking 보정: 무릎 상대 각도가 0° 이하(과신전)이고, 각속도가 음수(즉, 과신전이 진행 중)인 경우,
    // 추가 스프링 및 댐핑 계수를 적용하여 무릎 관절이 0° 이상이 되도록 반발력을 발생시킵니다.
    if ((ThetaLK_Improved[0] < 0.0f) && ((ThetaLK_Improved[0] - ThetaLK_Improved[1]) < 0.0f)) {
        tauLeftKnee_Improved = tauLeftKnee_Improved - 15.0f * ThetaLK_Improved[0] - 12.0f * (ThetaLK_Improved[0] - ThetaLK_Improved[1]) / WHOLEBODY_CONTROL_PERIOD;
    }
    if ((ThetaRK_Improved[0] < 0.0f) && ((ThetaRK_Improved[0] - ThetaRK_Improved[1]) < 0.0f)) {
        tauRightKnee_Improved = tauRightKnee_Improved - 15.0f * ThetaRK_Improved[0] - 12.0f * (ThetaRK_Improved[0] - ThetaRK_Improved[1]) / WHOLEBODY_CONTROL_PERIOD;
    }
    
    // (H) Shank Dynamics Integration (Central Difference):
    // 중앙 차분법(Central Difference Method)은 현재 상태와 바로 이전 상태를 이용하여 2차 미분(가속도, 즉 x'')를 근사하는 방법
    // 근사식: x'' ≈ (x(k+1) - 2*x(k) + x(k-1)) / dt^2
    // 이를 재배열하면, 새로운 상태:
    //   x(k+1) ≈ 2*x(k) - x(k-1) + x''(k)*dt^2
    // 종아리에 작용하는 총 가속도는 중력 가속도 [0; -9.81]과 무릎 관절 힘 및 지면 반력(fKnee + fGRF)을 종아리 질량(ShankMass_Improved)으로 나눈 값
    // 즉, 가속도 = ([0; -9.81] + (fKnee + fGRF)/ShankMass_Improved)
    // 따라서, 종아리 무게중심의 다음 상태는 다음과 같이 계산:
    // P20_next = 2*P20_current - P20_prev + ( [0; -9.81] + (fKnee + fGRF)/ShankMass_Improved ) * dt^2
    //
    // 종아리 각도(ThetaShank)도 동일한 방식으로 업데이트

    // (H-1) 계산: 종아리에 작용하는 가속도
    vec2D accLeft, accRight;
    accLeft.x = 0.0f + (fLeftKnee_Improved.x + fLGRF_x) / ShankMass_Improved;
    accLeft.y = -9.81f + (fLeftKnee_Improved.y + fLGRF_y) / ShankMass_Improved;
    accRight.x = 0.0f + (fRightKnee_Improved.x + fRGRF_x) / ShankMass_Improved;
    accRight.y = -9.81f + (fRightKnee_Improved.y + fRGRF_y) / ShankMass_Improved;
    
    // (H-2) 중앙 차분법을 이용하여 새로운 종아리 무게중심 위치 newP20 계산:
    P20L_Improved[0].x = 2 * P20L_Improved[1].x - P20L_Improved[2].x + accLeft.x * WHOLEBODY_CONTROL_PERIOD * WHOLEBODY_CONTROL_PERIOD;
    P20L_Improved[0].y = 2 * P20L_Improved[1].y - P20L_Improved[2].y + accLeft.y * WHOLEBODY_CONTROL_PERIOD * WHOLEBODY_CONTROL_PERIOD;
    P20R_Improved[0].x = 2 * P20R_Improved[1].x - P20R_Improved[2].x + accRight.x * WHOLEBODY_CONTROL_PERIOD * WHOLEBODY_CONTROL_PERIOD;
    P20R_Improved[0].y = 2 * P20R_Improved[1].y - P20R_Improved[2].y + accRight.y * WHOLEBODY_CONTROL_PERIOD * WHOLEBODY_CONTROL_PERIOD;
    
    // (H-3) 계산: cross term
    // 중앙 차분법을 사용하여 종아리 각도(ThetaLShank, ThetaRShank)를 업데이트
    // 수식: newTheta = 2*Theta_current - Theta_prev + (180/PI)*(-tauKnee + cross_term)/ShankMoment * dt^2
    // 여기서 cross_term = dPLeftShank' * S * (fLeftKnee - fLGRF), t = r X F, fLeftKnee - fLGRF는 무릎에 작용하는 순수 힘
    // S는 [0,1; -1,0]이므로, cross_term = dPLeftShank.x * fLeftKnee.y - dPLeftShank.y * fLeftKnee.x (fLGRF의 y성분은 0으로 가정)
    // 이는 종아리 무게중심에서 무릎까지의 벡터와 무릎에 작용하는 힘 사이의 외적 값, 즉 종라이 무게 중심에서 무릎의 회전력, 토크
    // 이 값을 ShankMoment인 관성모멘트로 나누면 각가속도가 나옴.
    
    float crossLeft = dPLeftShank_Improved.x * (fLeftKnee_Improved.y - fLGRF_y) -
                   dPLeftShank_Improved.y * (fLeftKnee_Improved.x - fLGRF_x);

    float crossRight = dPRightShank_Improved.x * (fRightKnee_Improved.y - fRGRF_y) -
                    dPRightShank_Improved.y * (fRightKnee_Improved.x - fRGRF_x);
    
    // (H-4) 계산: 종아리 각도
    ThetaLShank_Improved[0] = 2 * ThetaLShank_Improved[1] - ThetaLShank_Improved[2] +
                            (180.0f/M_PI) * (-tauLeftKnee_Improved + crossLeft) / ShankMoment_Improved * WHOLEBODY_CONTROL_PERIOD * WHOLEBODY_CONTROL_PERIOD;
    ThetaRShank_Improved[0] = 2 * ThetaRShank_Improved[1] - ThetaRShank_Improved[2] +
                            (180.0f/M_PI) * (-tauRightKnee_Improved + crossRight) / ShankMoment_Improved * WHOLEBODY_CONTROL_PERIOD * WHOLEBODY_CONTROL_PERIOD;
    
    // (H-5) 저장: 보정용 cross term을 별도로 저장 (추후 분석 등에서 활용 가능)
    ThetaLShank_term_Improved = crossLeft;
    ThetaRShank_term_Improved = crossRight;

    // (I) Forward Velocity & Distance Estimation
    // ----------------------------------------------------------------------
    // (I-1) 기구학적 속도 추정:
    //    - 발의 x좌표 변화량을 이용하여 전방 속도를 계산합니다.
    //    - 만약 한 쪽 발만 접촉 중이면 해당 발의 x좌표 변화량을 사용하고,
    //      양 발 모두 접촉 중이면 두 발의 변화량 중 작은 값을 선택합니다.
    //    - 음수 값은 0으로 처리하며(후진이 아니므로), 실험적으로 1.85의 보정 계수를 곱합니다.
    float velXKinematics_temp = 0.0f;
    if (LeftFootContact_Improved[0] == true && RightFootContact_Improved[0] == false) // 왼발만 지면 접촉: 좌측 발의 x좌표 변화량 사용
        velXKinematics_temp = fmaxf(0.0f, -(P2L_Improved[0].x - P2L_Improved[1].x) / WHOLEBODY_CONTROL_PERIOD);
    else if (LeftFootContact_Improved[0] == false && RightFootContact_Improved[0] == true) // 오른발만 지면 접촉: 우측 발의 x좌표 변화량 사용
        velXKinematics_temp = fmaxf(0.0f, -(P2R_Improved[0].x - P2R_Improved[1].x) / WHOLEBODY_CONTROL_PERIOD);
    else if (LeftFootContact_Improved[0] == true && RightFootContact_Improved[0] == true) {
        // 양발 모두 접촉: 두 발의 x좌표 변화량 중 작은 값(즉, 더 안정적인 측정치)을 선택
        float diffL = fabs(P2L_Improved[0].x - P2L_Improved[1].x);
        float diffR = fabs(P2R_Improved[0].x - P2R_Improved[1].x);
        if (diffL > diffR)
            velXKinematics_temp = -(P2R_Improved[0].x - P2R_Improved[1].x) / WHOLEBODY_CONTROL_PERIOD;
        else
            velXKinematics_temp = -(P2L_Improved[0].x - P2L_Improved[1].x) / WHOLEBODY_CONTROL_PERIOD;
    } else {
        velXKinematics_temp = 0.0f;
    }
    velXKinematics_Improved[0] = 1.85f * velXKinematics_temp;
    
    // (I-2) 가속도 센서 적분 및 융합:
    //    - 가속도 센서 데이터를 적분하여 전방 속도(velX)를 업데이트합니다.
    //    - 만약 발이 지면에 닿아있다면, 기구학적 속도와의 차이를 보정 항으로 더합니다.
    velX_Improved[0] = velX_Improved[1] + 5.0f * (LH_Sagittal.accXGlobal + RH_Sagittal.accXGlobal) * 0.5f * WHOLEBODY_CONTROL_PERIOD;
    if (LeftFootContact_Improved[0] || RightFootContact_Improved[0]) // 좌측 또는 우측 발이 지면에 접촉 중이면
        velX_Improved[0] = velX_Improved[0] + 0.255f * (velXKinematics_Improved[0] - velX_Improved[0]);
    else
        velX_Improved[0] = 0.0f;
    velXLPF_Improved[0] = 0.998f * velXLPF_Improved[1] + 0.002f * velX_Improved[0];
    
    // (I-3) 위치 적분:
    //    - 속도를 적분하여 전방 이동 거리를 업데이트합니다.
    //    - posXRaw는 적분된 원시 위치, posX는 추가 필터링을 적용한 위치입니다.
    posXRaw_Improved[0] = posXRaw_Improved[1] + velX_Improved[0] * WHOLEBODY_CONTROL_PERIOD;
    posX_Improved[0] = 0.9752f * posX_Improved[1] + 0.0248f * posXRaw_Improved[0];
    // (I-4) 누적 전방 이동 거리:
    //    - 최종적으로 posX_Improved 값이 전방 이동 거리로 사용됩니다.
    forwardDistance_Improved = posX_Improved[0];

    // (J) Gait Event Detection: Tablet App 실시간 디스플레이용 데이터
    // StepCountLeft 왼다리 보행 수
    // StepCountRight 오른다리 보행 수
    // StepLengthLeft 왼다리 보폭
    // StepLengthRight 오른다리 보폭
    // Cadence 분당 걸음 수
    if (isMotionAnalysisDuring) {
        DetectGaitEvents(&P2L_Improved[0], &P2R_Improved[0], postProcessingCnt);
    } else {
        DetectGaitEvents(&P2L_Improved[0], &P2R_Improved[0], gaitDetectLoopCnt);
        gaitDetectLoopCnt++;
    }

    // (K) Hip ROM, 최대 각속도 계산
    if (isMotionAnalysisDuring && !isMotionAnalysisStart) { // 동작 분석 데이터를 reset하지 않을때만 계산
        LeftHipROMFlex_Improved = fmax(Theta_Trunk_Improved + LH_Sagittal.ThighSagittalAngle, LeftHipROMFlex_Improved);
        LeftHipROMExt_Improved = fmin(Theta_Trunk_Improved + LH_Sagittal.ThighSagittalAngle, LeftHipROMExt_Improved);
        RightHipROMFlex_Improved = fmax(Theta_Trunk_Improved + RH_Sagittal.ThighSagittalAngle, RightHipROMFlex_Improved);
        RightHipROMExt_Improved = fmin(Theta_Trunk_Improved + RH_Sagittal.ThighSagittalAngle, RightHipROMExt_Improved);
        LeftHipMaxSpeed_Improved = fmax(fabs(LH_Sagittal.ThighSagittalVelocity), LeftHipMaxSpeed_Improved);
        RightHipMaxSpeed_Improved = fmax(fabs(RH_Sagittal.ThighSagittalVelocity), RightHipMaxSpeed_Improved);
    }

    // 태블릿 실시간 전송값 계산
    if (ThetaLK_Improved[0] < 0) {ThetaLK_Session = 0;}
    else {ThetaLK_Session = ThetaLK_Improved[0];}
    if (ThetaRK_Improved[0] < 0) {ThetaRK_Session = 0;}
    else {ThetaRK_Session = ThetaRK_Improved[0];}

    if (isOperationDuring) {
		SessionInfoUpdate();
    }

    // 동작분석 실시간 값 계산
    if (!LeftFootContact_Improved[0] && !RightFootContact_Improved[0]) {
    	GroundContact = 0;
    } else if (!LeftFootContact_Improved[0] && RightFootContact_Improved[0]) {
    	GroundContact = 1;
    } else if (LeftFootContact_Improved[0] && !RightFootContact_Improved[0]) {
    	GroundContact = 2;
    } else {
    	GroundContact = 3;
    }
    MotionAnalysisDuration = postProcessingCnt / 1000;

    // (L) Update Previous States: copy current ([0]) to previous ([1])
    P2L_Improved[1] = P2L_Improved[0];
    P2R_Improved[1] = P2R_Improved[0];
    P2CL_Improved[1] = P2CL_Improved[0];
    P2CR_Improved[1] = P2CR_Improved[0];

    P20L_Improved[2] = P20L_Improved[1];
    P20L_Improved[1] = P20L_Improved[0];
    P20R_Improved[2] = P20R_Improved[1];
    P20R_Improved[1] = P20R_Improved[0];

    ThetaLShank_Improved[2] = ThetaLShank_Improved[1];
    ThetaLShank_Improved[1] = ThetaLShank_Improved[0];
    ThetaRShank_Improved[2] = ThetaRShank_Improved[1];
    ThetaRShank_Improved[1] = ThetaRShank_Improved[0];

    ThetaLK_Improved[1] = ThetaLK_Improved[0];
    ThetaRK_Improved[1] = ThetaRK_Improved[0];

    velX_Improved[1] = velX_Improved[0];
    velXLPF_Improved[1] = velXLPF_Improved[0];
    posXRaw_Improved[1] = posXRaw_Improved[0];
    posX_Improved[1] = posX_Improved[0];
    errorLeft_Improved[1] = errorLeft_Improved[0];
    errorRight_Improved[1] = errorRight_Improved[0];
    LeftFootContact_Improved[1] = LeftFootContact_Improved[0];
    RightFootContact_Improved[1] = RightFootContact_Improved[0];

    /* Post Processing */
    UpdateForPostProcessingData();

    // (A) Session Flags Set: Tablet App 신호에 따른 동작 분석 Command Flags Set
    // (A-1) 동작 분석 시작 신호시 데이터 초기 Reset(세션 시작때마다 한 번만)
    if (isMotionAnalysisStart) { // Tablet의 동작 분석 Data 또한 Reset 필요
        gaitDetectLoopCnt = 0;

        // 전방 위치, 속도, 누적 이동거리 초기화
        memset(&posXRaw_Improved, 0, sizeof(posXRaw_Improved));
        memset(&posX_Improved, 0, sizeof(posX_Improved));
//        memset(&velXLPF_Improved, 0, sizeof(velXLPF_Improved));
        finishPos = 0.0f;

        LeftHipROMFlex_Improved = 0.0f;
        LeftHipROMExt_Improved = 0.0f;
        RightHipROMFlex_Improved = 0.0f;
        RightHipROMExt_Improved = 0.0f;
        LeftHipMaxSpeed_Improved = 0.0f;
        RightHipMaxSpeed_Improved = 0.0f;

        StrideCountLeft_Improved = 0;
        StrideCountRight_Improved = 0;
        StrideLengthLeft_Improved = 0.0f;
        StrideLengthRight_Improved = 0.0f;
        StepLengthCountLeft_Improved = 0;
        StepLengthCountRight_Improved = 0;
        StepLengthLeft_Improved = 0.0f;
        StepLengthRight_Improved = 0.0f;
        Cadence_Improved = 0.0f;
        stride_time = 0.0f;
        LeftStepTime = 0.0f;
        RightStepTime = 0.0f;
        IndexLeftHeelStrike = 0;
        IndexRightHeelStrike = 0;
        IndexLeftToeOff = 0;
        IndexRightToeOff = 0;
        StepLengthLeftInMeasure_Improved = false;
        StepLengthRightInMeasure_Improved = false;
        ReferencePositionLeftFoot_Improved = 0.0f;
        ReferencePositionRightFoot_Improved = 0.0f;
        memset(&leftHeelTracker, 0, sizeof(leftHeelTracker));
        memset(&rightHeelTracker, 0, sizeof(rightHeelTracker));

        OnlineStats_Init(&cadenceStats);
        OnlineStats_Init(&stepLengthLeftStats);
        OnlineStats_Init(&stepLengthRightStats);
        OnlineStats_Init(&strideLengthLeftStats);
        OnlineStats_Init(&strideLengthRightStats);

        isMotionAnalysisStart = false;
        isMotionAnalysisDuring = true;
        onetime_send_finish_able = false;
        onetime_finish_set = false;
        MotionAnalysisFinishAble = false;
        MotionAnalysisSelfStop = false;
        isPostProcessed = false;
    }

    if (isMotionAnalysisDuring && (StrideCountLeft_Improved >= 2 && StrideCountRight_Improved >= 2)
    		&& (StrideCountLeft_Improved + StrideCountRight_Improved >= 5)
    		&& !MotionAnalysisFinishAble) {
    	MotionAnalysisFinishAble = true;
    }

    if (isMotionAnalysisDuring &&
    		((!TrackUsage && StrideCountLeft_Improved >= 16 && StrideCountRight_Improved >= 16)
    		|| (StrideCountLeft_Improved >= MAX_NUM_GAIT_EVENTS || StrideCountRight_Improved >= MAX_NUM_GAIT_EVENTS)
    		|| postProcessingCnt == TOTAL_SAMPLE_COUNT)) {
    	MotionAnalysisSelfStop = true;
    }

    // (A-2) 동작 분석 종료 신호시 or 자체 종료 판단시
    if ((isMotionAnalysisFinish || MotionAnalysisSelfStop) && !onetime_finish_set) {
    	onetime_finish_set = true;
    	finishPos = posX_Improved[0];
        isMotionAnalysisDuring = false;
        isMotionAnalysisFinish = false;
        PostProcessingStart = true;
        MotionAnalysisDataState = 0;
    }

    // 동작 분석 취소시 초기화
    if (isMotionAnalysisCancel) {
    	isMotionAnalysisCancel = false;
        ResetAllDisplayedValuesToZero();
    	if (isMotionAnalysisDuring || PostProcessingStart) {
    		isMotionAnalysisDuring = false;
    		PostProcessingStart = false;
    		MotionAnalysisFinishAble = false;
    		MotionAnalysisSelfStop = false;
    		postProcessingCnt = 0;
            isMotionAnalysisFinishedBefore = true;
            isMotionAnalysisFinishedForDataSave = false;
    	}
    }
}

void GaitAnalysis_PostProcessing(void)
{
    if (PostProcessingStart) {
        // 실제 이동거리를 고려한 보정 인자 계산
        if (finishPos < 0.1){
        	Distance_SF = 0;
        } else {
        	Distance_SF = RealDistance / finishPos;
        }

        if (!TrackUsage) {
        	Distance_SF = 1;
        }

        // 1개 또는 2개 남는 샘플을 평균내어 마무리
        FinalizeDownsample(&tempVelX, &sampleCnt_VelX, &sampleIndex_VelX, velX4AnalysisFiltered, PP_MAX_INDEX);
        FinalizeDownsample(&tempThetaRH, &sampleCnt_ThetaRH, &sampleIndex_ThetaRH, ThetaRH4AnalysisFiltered, PP_MAX_INDEX);
        FinalizeDownsample(&tempThetaLH, &sampleCnt_ThetaLH, &sampleIndex_ThetaLH, ThetaLH4AnalysisFiltered, PP_MAX_INDEX);
        FinalizeDownsample(&tempTorqueRH, &sampleCnt_TorqueRH, &sampleIndex_TorqueRH, TorqueRH4AnalysisFiltered, PP_MAX_INDEX);
        FinalizeDownsample(&tempTorqueLH, &sampleCnt_TorqueLH, &sampleIndex_TorqueLH, TorqueLH4AnalysisFiltered, PP_MAX_INDEX);

        // Step Time, Step Frequency, meanStepFreq 계산을 함수로 호출 (For Velx cutoff Frequency)
        ComputeStepFrequency(
            IndexLeftHeelStrikeCalibrated, numLeftHeelStrike,
            IndexRightHeelStrikeCalibrated, numRightHeelStrike,
            stepTime, stepFrequency, &meanStepFreq
        );

        // 각 배열에 대해 실제 저장된 샘플 수(sampleIndex_XXX)를 이용하여 필터 적용
        ZeroPhaseFilter(velX4AnalysisFiltered, sampleIndex_VelX, meanStepFreq / 3.0f);
        // ZeroPhaseFilter(ThetaRH4AnalysisFiltered, sampleIndex_ThetaRH, 20.0f/3.0f);
        // ZeroPhaseFilter(ThetaLH4AnalysisFiltered, sampleIndex_ThetaLH, 20.0f/3.0f);
        // ZeroPhaseFilter(TorqueRH4AnalysisFiltered, sampleIndex_TorqueRH, 20.0f/3.0f);
        // ZeroPhaseFilter(TorqueLH4AnalysisFiltered, sampleIndex_TorqueLH, 20.0f/3.0f);

        // Distance Scale Factor 적용
        for (int i = 0; i < sampleIndex_VelX; i++) {
            velX4AnalysisFiltered[i] = velX4AnalysisFiltered[i] * Distance_SF;
        }

        // === 실측 Data 기반 Scale Factors(Mean값에 적용) ===
        // SwingPhase: 0.9824
        // Cadence: 0.9974
        // Speed: 1.0565
        // Step Length: 1.0723
        // Stride Length: 1.0719
        
        /* 평균과 표준편차 계산(Offline) */
        OfflineStatsGetMeanNStd(velX4AnalysisFiltered, sampleIndex_VelX, &meanVelX, &stdVelX, 1.0565);

        /* 평균과 표준편차 계산(Online) */
        // Cadence 평균 및 표준편차
        if (cadenceStats.mean == 0.0f) {
            meanCadence = 0.0f;
        } else {
            meanCadence = (60.0f / cadenceStats.mean) * 2.0f * 0.9974;
        }
        stdCadence = OnlineStatsGetStd(&cadenceStats);

        // Step Length 양쪽 평균, 오른쪽 평균, 왼쪽 평균
        meanStepLengthLeft = stepLengthLeftStats.mean * Distance_SF * 1.0723;
        meanStepLengthRight = stepLengthRightStats.mean * Distance_SF * 1.0723;
        meanStepLengthBoth = CombineMean(meanStepLengthLeft, meanStepLengthRight);
        // 비대칭 지수 계산
        asymmetryStepLength = ComputeAsymmetryIndex(meanStepLengthLeft, meanStepLengthRight);

        // Stride Length 양쪽 평균, 양쪽 표준편차
        meanStrideLengthLeft = strideLengthLeftStats.mean * Distance_SF * 1.0719;
        meanStrideLengthRight = strideLengthRightStats.mean * Distance_SF * 1.0719;
        meanStrideLengthBoth = CombineMean(meanStrideLengthLeft, meanStrideLengthRight);
        stdStrideLengthLeft = OnlineStatsGetStd(&strideLengthLeftStats) * Distance_SF;
        stdStrideLengthRight = OnlineStatsGetStd(&strideLengthRightStats) * Distance_SF;
        stdStrideLengthBoth = CombineStd(meanStrideLengthLeft, stdStrideLengthLeft, meanStrideLengthRight, stdStrideLengthRight);

        // 왼쪽 엉덩관절 각도
        ResampleGaitCycleSegment(ThetaLH4AnalysisFiltered, IndexLeftHeelStrikeCalibrated, numLeftHeelStrike, ResampledLeftHipAngles, segmentBuffer);
        memset(segmentBuffer, 0, sizeof(segmentBuffer));
        // 오른쪽 엉덩관절 각도
        ResampleGaitCycleSegment(ThetaRH4AnalysisFiltered, IndexRightHeelStrikeCalibrated, numRightHeelStrike, ResampledRightHipAngles, segmentBuffer);
        memset(segmentBuffer, 0, sizeof(segmentBuffer));
        // 왼쪽 엉덩관절 토크
        ResampleGaitCycleSegment(TorqueLH4AnalysisFiltered, IndexLeftHeelStrikeCalibrated, numLeftHeelStrike, ResampledLeftHipTorques, segmentBuffer);
        memset(segmentBuffer, 0, sizeof(segmentBuffer));
        // 오른쪽 엉덩관절절 토크
        ResampleGaitCycleSegment(TorqueRH4AnalysisFiltered, IndexRightHeelStrikeCalibrated, numRightHeelStrike, ResampledRightHipTorques, segmentBuffer);

        // 실측 Data 기반 Scale Factors Shift Resampled Data(20% 앞당기기)
        ShiftGaitCycle(ResampledLeftHipAngles, numLeftHeelStrike, RESAMPLE_SHIFT_SAMPLES, ResampledLeftHipAngles);
        ShiftGaitCycle(ResampledRightHipAngles, numRightHeelStrike, RESAMPLE_SHIFT_SAMPLES, ResampledRightHipAngles);
        ShiftGaitCycle(ResampledLeftHipTorques, numLeftHeelStrike, RESAMPLE_SHIFT_SAMPLES, ResampledLeftHipTorques);
        ShiftGaitCycle(ResampledRightHipTorques, numRightHeelStrike, RESAMPLE_SHIFT_SAMPLES, ResampledRightHipTorques);

        // Hip Joint & Torque ZPF
        cutOffFreq_ForResampledData = 10.0f * (meanStepFreq / 2.0f);
        cutOffFreq_ForResampledData = fmaxf(2.0f, fminf(10.0f, cutOffFreq_ForResampledData));
        ZeroPhaseFilter2D(ResampledLeftHipAngles, numLeftHeelStrike, FIXED_GAIT_SAMPLES, cutOffFreq_ForResampledData);
        ZeroPhaseFilter2D(ResampledRightHipAngles, numRightHeelStrike, FIXED_GAIT_SAMPLES, cutOffFreq_ForResampledData);
        ZeroPhaseFilter2D(ResampledLeftHipTorques, numLeftHeelStrike, FIXED_GAIT_SAMPLES, cutOffFreq_ForResampledData);
        ZeroPhaseFilter2D(ResampledRightHipTorques, numRightHeelStrike, FIXED_GAIT_SAMPLES, cutOffFreq_ForResampledData);

        // 엉덩관절 각도, 토크의 평균 및 표준편차 계산
        ComputeResampledStats(ResampledLeftHipAngles, numLeftHeelStrike - 1, meanResampledLeftHipAngles, stdResampledLeftHipAngles);
        ComputeResampledStats(ResampledRightHipAngles, numRightHeelStrike - 1, meanResampledRightHipAngles, stdResampledRightHipAngles);
        ComputeResampledStats(ResampledLeftHipTorques, numLeftHeelStrike - 1, meanResampledLeftHipTorques, stdResampledLeftHipTorques);
        ComputeResampledStats(ResampledRightHipTorques, numRightHeelStrike - 1, meanResampledRightHipTorques, stdResampledRightHipTorques);

        // 입각기 및 유각기 비율 계산
        // ComputeNumCycles(numLeftHeelStrike, numLeftToeOff, numRightHeelStrike, numRightToeOff,
        //                     &numOfLeftCycles, &numOfRightCycles, &numOfCycles);
        // ComputeGaitPhase(IndexLeftHeelStrikeCalibrated, IndexLeftToeOffCalibrated, IndexRightHeelStrikeCalibrated, IndexRightToeOffCalibrated,
        //                     &numOfCycles, &leftSwingStartPhase, &rightStanceStartPhase, &rightSwingStartPhase);
        // rightStancePhase = rightStanceStartPhase + rightSwingStartPhase;
        // rightSwingPhase = 100 - rightStancePhase;
        // leftStancePhase = leftSwingStartPhase;
        // leftSwingPhase = 100 - leftStancePhase;
        ComputeGaitPhaseRatios(
            IndexLeftHeelStrikeCalibrated, IndexLeftToeOffCalibrated,
            IndexRightHeelStrikeCalibrated, IndexRightToeOffCalibrated,
            numLeftHeelStrike, numLeftToeOff,
            numRightHeelStrike, numRightToeOff,
            &leftStancePhase, &leftSwingPhase,
            &rightStancePhase, &rightSwingPhase
        );

        // 엉덩관절 최대 굽힘/폄 각도 및 가동범위 계산
        ResampledLeftHipROMFlex = GetMaxValue(meanResampledLeftHipAngles);
        ResampledLeftHipROMExt = GetMinValue(meanResampledLeftHipAngles);
        ResampledRightHipROMFlex = GetMaxValue(meanResampledRightHipAngles);
        ResampledRightHipROMExt = GetMinValue(meanResampledRightHipAngles);

        // 동작분석 데이터 바(bar) 표시를 위해 변환
        MotionAnalysisMeanLeftHipMax = fmax(0.0, fmin(150.0, ResampledLeftHipROMFlex + 30));
		MotionAnalysisMeanLeftHipMin = fmax(0.0, fmin(150.0, ResampledLeftHipROMExt + 30));
		MotionAnalysisMeanRightHipMax = fmax(0.0, fmin(150.0, ResampledRightHipROMFlex + 30));
		MotionAnalysisMeanRightHipMin = fmax(0.0, fmin(150.0, ResampledRightHipROMExt + 30));

        leftHipROM = ResampledLeftHipROMFlex - ResampledLeftHipROMExt;
        rightHipROM = ResampledRightHipROMFlex - ResampledRightHipROMExt;
        maxHipROM = leftHipROM > rightHipROM ? leftHipROM : rightHipROM;
        meanHipROM = (leftHipROM + rightHipROM) / 2.0f;

        // 비대칭 지수 계산
        if (meanHipROM == 0) {
            asymmetryHipAngle = 25.0f;
        } else {
            asymmetryHipAngle = (rightHipROM - leftHipROM) / meanHipROM * 100.0f;
            asymmetryHipAngle += 25.0f;
            asymmetryHipAngle = fmax(0.0, fmin(50.0, asymmetryHipAngle));
        }

        asymmetryStancePhase = ComputeAsymmetryIndex(leftStancePhase, rightStancePhase);
        asymmetrySwingPhase = ComputeAsymmetryIndex(leftSwingPhase, rightSwingPhase);

        SumLeftHipAnglesDiff = 0.0f;
        SumRightHipAnglesDiff = 0.0f;

        // 보행주기(0~100%) 그래프
        for (size_t i = 0; i < FIXED_GAIT_SAMPLES; i++) {
        	GaitCycle[i] = i;  // 0~100까지 값을 저장
        	ThighAngleLeftYMean[i] = (int16_t) (meanResampledLeftHipAngles[i] * 10);
        	ThighAngleLeftYStd[i] = (int16_t) (stdResampledLeftHipAngles[i] * 10);
        	ThighAngleRightYMean[i] = (int16_t) (meanResampledRightHipAngles[i] * 10);
        	ThighAngleRightYStd[i] = (int16_t) (stdResampledRightHipAngles[i] * 10);
        	ThighTorqueLeftYMean[i] = (int16_t) (meanResampledLeftHipTorques[i] * 100);
        	ThighTorqueLeftYStd[i] = (int16_t) (stdResampledLeftHipTorques[i] * 100);
        	ThighTorqueRightYMean[i] = (int16_t) (meanResampledRightHipTorques[i] * 100);
        	ThighTorqueRightYStd[i] = (int16_t) (stdResampledRightHipTorques[i] * 100);

        	SumLeftHipAnglesDiff += powf(meanResampledLeftHipAngles[i] - normalSubjectHipAngles[i], 2.0);
        	SumRightHipAnglesDiff += powf(meanResampledRightHipAngles[i] - normalSubjectHipAngles[i], 2.0);
        }
        MotionAnalysisRMSLeftHipDiff = sqrtf(SumLeftHipAnglesDiff / FIXED_GAIT_SAMPLES);
        MotionAnalysisRMSRightHipDiff = sqrtf(SumRightHipAnglesDiff / FIXED_GAIT_SAMPLES);

        // 초기화
//        ResetAllDisplayedValuesToZero();
        MotionAnalysisFinishAble = false;
        MotionAnalysisSelfStop = false;
        PostProcessingStart = false;
        postProcessingCnt = 0;
        isPostProcessed = true;
        isMotionAnalysisFinishedBefore = true;
        isMotionAnalysisFinishedForDataSave = true;
        dataSaveCnt = 0;

        // 태블릿 연동 스테이트
        MotionAnalysisDataState = 1;
    }
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void DetectGaitEvents(vec2D* P2L, vec2D* P2R, uint32_t loopCnt)
{
    static bool isFirstUpdateCount1 = true;
    // static bool isFirstUpdateCount2 = true;
    static bool isFirstUpdateCount3 = true;
    // static bool isFirstUpdateCount4 = true;
    // --- 좌측 Heel Strike 이벤트 (Swing->Stance 전이) ---
    if (!LeftFootContact_Improved[1] && LeftFootContact_Improved[0]) {
        // 왼발 걸음 수 업데이트
        if (isFirstUpdateCount1) {isFirstUpdateCount1 = false;}
        else {
            StrideCountLeft_Improved++;
            // IndexLeftHeelStrike[StrideCountLeft_Improved - 1] = loopCnt;
            IndexLeftHeelStrike = loopCnt;
        }

        // 첫 heel strike 이벤트인 경우
        if (StrideCountLeft_Improved <= 1) {
            // 현재 기준 위치를 저장
            leftHeelTracker.prevSample = loopCnt;
            leftHeelTracker.prevRefPos = posX_Improved[0] + P2L->x;
        }
        else { // 두 번째 이후 이벤트: 이전 이벤트 정보가 유효하므로 보폭 및 cadence 계산
            // 현재 기준 위치: 현재 전방 위치 posX_Improved[0]와 좌측 발의 x 위치 P2L_x_Improved를 더한 값
            leftHeelTracker.currRefPos = posX_Improved[0] + P2L->x;  // 현재 기준 위치 계산
            // 보폭 (stride length): 현재 기준 위치에서 바로 직전 기준 위치의 차이
            StrideLengthLeft_Improved = leftHeelTracker.currRefPos - leftHeelTracker.prevRefPos;  // 보폭 계산
            // // 두 이벤트 간 시간 차 (초)
            // leftHeelStrikeDeltaTime = ((float)loopCnt - (float)leftHeelTracker.prevSample) * WHOLEBODY_CONTROL_PERIOD;
            // if (leftHeelStrikeDeltaTime > 0) {
            //     Cadence_Improved = 120.0f / leftHeelStrikeDeltaTime;  // 분당 걸음수 계산: 60 / deltaTime(sec)
            // }
            // 평균과 표준편차 계산(Online) For Post Processing
            if (isMotionAnalysisDuring) {
                UpdateOnlineStats(&strideLengthLeftStats, StrideLengthLeft_Improved);
                // UpdateOnlineStats(&cadenceStats, leftHeelStrikeDeltaTime);
            }

            // 업데이트: 현재 heel strike 이벤트 정보를 tracker에 저장
            leftHeelTracker.prevSample = loopCnt;
            leftHeelTracker.prevRefPos = leftHeelTracker.currRefPos;
        }

        // Right Step Time (R → L)
        // if (StrideCountRight_Improved > 0 && IndexRightHeelStrike[StrideCountRight_Improved - 1] < loopCnt) {
            // RightStepTime = ((float)loopCnt - (float)IndexRightHeelStrike[StrideCountRight_Improved - 1]) * WHOLEBODY_CONTROL_PERIOD;
        if (StrideCountRight_Improved > 0 && IndexRightHeelStrike < loopCnt) {
            RightStepTime = ((float)loopCnt - (float)IndexRightHeelStrike) * WHOLEBODY_CONTROL_PERIOD;
            if (StrideCountLeft_Improved > 1 && LeftStepTime > 0) {
                stride_time = LeftStepTime + RightStepTime;
                if (stride_time > 0) {
                    Cadence_Improved = (60.0f / stride_time) * 2.0f;
                }
                if (isMotionAnalysisDuring) {
                    UpdateOnlineStats(&cadenceStats, stride_time);
                }
            }
        }

        // 반걸음 보폭 측정 모드 전환: 양 발 모두 반걸음 보폭 측정 중이 아니면,
        // 왼발이 접촉한 상태이므로 오른발의 반걸음 측정을 시작하도록 설정.
        if (!StepLengthLeftInMeasure_Improved && !StepLengthRightInMeasure_Improved) {
            StepLengthLeftInMeasure_Improved = false;
            StepLengthRightInMeasure_Improved = true;
        }
        // 만약 이미 좌측 반걸음 측정 모드였다면,
        // 현재 heel strike 시점의 기준 위치와 이전 반걸음 기준 위치의 차이를 계산하여 반걸음 보폭을 산출
        if (StepLengthLeftInMeasure_Improved && !StepLengthRightInMeasure_Improved) {
            // if (isFirstUpdateCount2) {isFirstUpdateCount2 = false;}
            // else {StepLengthCountLeft_Improved++;}

            // if (StepLengthCountLeft_Improved > 1) {
                // 보폭 계산 (이전 기준점과 현재 발 위치 차이를 사용)
                StepLengthLeft_Improved = (posX_Improved[0] + P2L->x) - ReferencePositionRightFoot_Improved;
                // 평균과 표준편차 계산(Online) For Post Processing
                if (isMotionAnalysisDuring) {
                    UpdateOnlineStats(&stepLengthLeftStats, StepLengthLeft_Improved);
                }
            // }
            // 기준점을 현재 heel strike의 기준 위치로 업데이트
            ReferencePositionLeftFoot_Improved = posX_Improved[0] + P2L->x;
            // 전환: 이제 오른발 반걸음 측정 모드로 전환
            StepLengthLeftInMeasure_Improved = false;
            StepLengthRightInMeasure_Improved = true;
        }
        
        // 오류 상황: 왼발이 다시 접촉하면, 카운트는 증가하지 않고 기준점만 업데이트
        if (!StepLengthLeftInMeasure_Improved && StepLengthRightInMeasure_Improved) {
            ReferencePositionLeftFoot_Improved = posX_Improved[0] + P2L->x;
        }

        if (isMotionAnalysisDuring) {
            if (sampleIndex_PostProcessingCnt >= 1 && (sampleIndex_PostProcessingCnt < PP_MAX_INDEX)) {
                uint32_t indexToUse = (uint32_t)postProcessingCntFiltered[sampleIndex_PostProcessingCnt - 1];
                UpdateHeelStrikeIndices(LEFT_GAIT_EVENT, IndexLeftHeelStrikeCalibrated, IndexRightHeelStrikeCalibrated,
                                        &numLeftHeelStrike, &numRightHeelStrike, &indexToUse);
            }
        }
    }

    // 좌측 스윙 시작 시점 저장 (Left Toe Off)
    if (LeftFootContact_Improved[1] && !LeftFootContact_Improved[0]) {
        if (StrideCountLeft_Improved > 0) {
            IndexLeftToeOff = loopCnt;
            if (isMotionAnalysisDuring) {
                if (sampleIndex_PostProcessingCnt >= 1 && (sampleIndex_PostProcessingCnt < PP_MAX_INDEX)) {
                    uint32_t indexToUse = (uint32_t)postProcessingCntFiltered[sampleIndex_PostProcessingCnt - 1];
                    UpdateToeOffIndices(LEFT_GAIT_EVENT, IndexLeftToeOffCalibrated, IndexRightToeOffCalibrated,
                                        &numLeftToeOff, &numRightToeOff, &indexToUse);
                }
            }
        }
    }

    // 우측 Heel Strike 이벤트 처리 (Swing -> Stance 전이)
    if (!RightFootContact_Improved[1] && RightFootContact_Improved[0]) {
        // 오른발 걸음 수 업데이트
        if (isFirstUpdateCount3) {isFirstUpdateCount3 = false;}
        else {
            StrideCountRight_Improved++;
            // IndexRightHeelStrike[StrideCountRight_Improved - 1] = loopCnt;
            IndexRightHeelStrike = loopCnt;
        }

        // 첫 heel strike 이벤트인 경우
        if (StrideCountRight_Improved <= 1) {
            // 현재 기준 위치를 저장
            rightHeelTracker.prevRefPos = posX_Improved[0] + P2R->x;
        }
        else {
            // 두 번째 이후 이벤트: 이전 이벤트 정보가 유효하므로 보폭 및 cadence 계산
            rightHeelTracker.currRefPos = posX_Improved[0] + P2R->x;  // 현재 기준 위치 계산
            StrideLengthRight_Improved = rightHeelTracker.currRefPos - rightHeelTracker.prevRefPos;  // 보폭 계산
            // 평균과 표준편차 계산(Online) For Post Processing
            if (isMotionAnalysisDuring) {
                UpdateOnlineStats(&strideLengthRightStats, StrideLengthRight_Improved);
            }

            // 업데이트: 현재 heel strike 이벤트 정보를 tracker에 저장
            rightHeelTracker.prevRefPos = rightHeelTracker.currRefPos;
        }

        // Left Step Time (L → R)
        // if (StrideCountLeft_Improved > 0 && IndexLeftHeelStrike[StrideCountLeft_Improved - 1] < loopCnt) {
            // LeftStepTime = ((float)loopCnt - (float)IndexLeftHeelStrike[StrideCountLeft_Improved - 1]) * WHOLEBODY_CONTROL_PERIOD;
        if (StrideCountLeft_Improved > 0 && IndexLeftHeelStrike < loopCnt) {
            LeftStepTime = ((float)loopCnt - (float)IndexLeftHeelStrike) * WHOLEBODY_CONTROL_PERIOD;
            if (StrideCountRight_Improved > 1 && RightStepTime > 0) {
                stride_time = LeftStepTime + RightStepTime;
                if (stride_time > 0) {
                    Cadence_Improved = (60.0f / stride_time) * 2.0f;
                }
                if (isMotionAnalysisDuring) {
                    UpdateOnlineStats(&cadenceStats, stride_time);
                }
            }
        }

        // 반걸음 보폭 측정 모드 전환:
        // 우측 heel strike의 경우, 우측 발이 접촉했으므로, 만약 양 발 모두 반걸음 측정 모드가 꺼져있다면,
        // 좌측 반걸음 측정을 시작하도록 설정합니다.
        if (!StepLengthLeftInMeasure_Improved && !StepLengthRightInMeasure_Improved) {
            // 초기 상태: 아무 반걸음 측정도 진행 중이 아니므로, 좌측 측정을 시작하도록 설정
            StepLengthLeftInMeasure_Improved = true;
            StepLengthRightInMeasure_Improved = false;
        }
        // 만약 이미 우측 반걸음 측정 모드였다면,
        // 현재 heel strike 시점의 기준 위치와 이전 반걸음 기준 위치의 차이를 계산하여 우측 반걸음 보폭을 산출하고,
        // 기준점을 업데이트한 후, 모드를 좌측 측정으로 전환합니다.
        if (!StepLengthLeftInMeasure_Improved && StepLengthRightInMeasure_Improved) {
            // if (isFirstUpdateCount4) {isFirstUpdateCount4 = false;}
            // else {StepLengthCountRight_Improved++;}

            // if (StepLengthCountRight_Improved > 1) {
                StepLengthRight_Improved = (posX_Improved[0] + P2R->x) - ReferencePositionLeftFoot_Improved;
                // 평균과 표준편차 계산(Online) For Post Processing
                if (isMotionAnalysisDuring) {
                    UpdateOnlineStats(&stepLengthRightStats, StepLengthRight_Improved);
                }
            // }
            // 기준점을 현재 heel strike 시점의 우측 발 위치로 업데이트
            ReferencePositionRightFoot_Improved = posX_Improved[0] + P2R->x;
            // 전환: 이제 좌측 반걸음 측정을 진행
            StepLengthLeftInMeasure_Improved = true;
            StepLengthRightInMeasure_Improved = false;
        }
        
        // 오류 상황: 만약 우측 heel strike 이벤트 발생 후, 예상치 못한 방식으로 (예: 연속 heel strike)
        // 모드가 그대로 유지된다면, 기준점만 업데이트합니다.
        if (StepLengthLeftInMeasure_Improved && !StepLengthRightInMeasure_Improved) {
            ReferencePositionRightFoot_Improved = posX_Improved[0] + P2R->x;
        }

        if (isMotionAnalysisDuring) {
            if (sampleIndex_PostProcessingCnt >= 1 && (sampleIndex_PostProcessingCnt < PP_MAX_INDEX)) {
                uint32_t indexToUse = (uint32_t)postProcessingCntFiltered[sampleIndex_PostProcessingCnt - 1];
                UpdateHeelStrikeIndices(RIGHT_GAIT_EVENT, IndexLeftHeelStrikeCalibrated, IndexRightHeelStrikeCalibrated,
                                        &numLeftHeelStrike, &numRightHeelStrike, &indexToUse);
            }
        }
    }

    // 우측 스윙 시작 시점 저장 (Right Toe Off)
    if (RightFootContact_Improved[1] && !RightFootContact_Improved[0]) {
        if (StrideCountRight_Improved > 0) {
            IndexRightToeOff = loopCnt;
            if (isMotionAnalysisDuring) {
                if (sampleIndex_PostProcessingCnt >= 1 && (sampleIndex_PostProcessingCnt < PP_MAX_INDEX)) {
                    uint32_t indexToUse = (uint32_t)postProcessingCntFiltered[sampleIndex_PostProcessingCnt - 1];
                    UpdateToeOffIndices(RIGHT_GAIT_EVENT, IndexLeftToeOffCalibrated, IndexRightToeOffCalibrated,
                                        &numLeftToeOff, &numRightToeOff, &indexToUse);
                }
            }
        }
    }
}

static void SessionInfoUpdate(void)
{
    static bool isFirstUpdateCount1 = true;
    static bool isFirstUpdateCount2 = true;
    static bool isStrideLeftCount = false;
    static bool isStrideRightCount = false;

    // --- 좌측 Heel Strike 이벤트 (Swing->Stance 전이) ---
    if (!LeftFootContact_Improved[1] && LeftFootContact_Improved[0]) {
        // 왼발 걸음 수 업데이트
        if (isFirstUpdateCount1) {isFirstUpdateCount1 = false;}
        else {
			StrideCountLeft_Session++;
			if (isStrideRightCount) {
				StrideCount_Session += 0.5;
				isStrideRightCount = false;
			}
			if (!isStrideLeftCount) {isStrideLeftCount = true;}
        }
    }

    // 우측 Heel Strike 이벤트 처리 (Swing -> Stance 전이)
    if (!RightFootContact_Improved[1] && RightFootContact_Improved[0]) {
        // 오른발 걸음 수 업데이트
        if (isFirstUpdateCount2) {isFirstUpdateCount2 = false;}
        else {
			StrideCountRight_Session++;
			if (isStrideLeftCount) {
				StrideCount_Session += 0.5;
				isStrideLeftCount = false;
			}
			if (!isStrideRightCount) {isStrideRightCount = true;}
        }
    }

	posXRaw_Session[0] = posXRaw_Session[1] + velX_Improved[0] * WHOLEBODY_CONTROL_PERIOD;
	if (posXRaw_Session[0] < 0) {posXRaw_Session[0] = 0;}
	posX_Session[0] = 0.9752f * posX_Session[1] + 0.0248f * posXRaw_Session[0];
	forwardDistance_Session = posX_Session[0];

	posXRaw_Session[1] = posXRaw_Session[0];
	posX_Session[1] = posX_Session[0];

	if (StrideCount_Session != 0) {
		StepLengthAvg_Session = forwardDistance_Session / StrideCount_Session * 100;
	} else {
		StepLengthAvg_Session = 0;
	}

	trainingInfo[current_session].walk_count[BLECtrlObj.data.AssistMode] = (uint16_t) StrideCount_Session;
	trainingInfo[current_session].walk_distance[BLECtrlObj.data.AssistMode] = forwardDistance_Session;
	trainingInfo[current_session].step_length_avg[BLECtrlObj.data.AssistMode] = StepLengthAvg_Session;
}

static void ResetAllDisplayedValuesToZero(void)
{
	Cadence_Improved = 0.0f;
	StrideCountLeft_Improved = 0;
	StrideCountRight_Improved = 0;

	StepLengthLeft_Improved = 0.0f;
	StepLengthRight_Improved = 0.0f;

	LeftHipROMFlex_Improved = 0.0f;
	LeftHipROMExt_Improved = 0.0f;
	RightHipROMFlex_Improved = 0.0f;
	RightHipROMExt_Improved = 0.0f;
	LeftHipMaxSpeed_Improved = 0.0f;
	RightHipMaxSpeed_Improved = 0.0f;

	GroundContact = 3;
	forwardDistance_Improved = 0.0f;
	velXKinematics_Improved[0] = 0.0f;
}

static void CalTrunkNeutralPosture(void)
{
	if (calTrunkNeutralPostureCmd == 1) {
		if (calTrunkNeutralPostureCnt < 1000) {
			tAccumulateTrunkAngle += Theta_Trunk_Improved;
			calTrunkNeutralPostureCnt++;
		} else if (calTrunkNeutralPostureCnt == 1000) {
			TrunkNeutralPostureBias = tAccumulateTrunkAngle / (float)(calTrunkNeutralPostureCnt);
			tAccumulateTrunkAngle = 0;
			calTrunkNeutralPostureCnt = 0;
			calTrunkNeutralPostureCmd = 0;
			trunkNeutralizedFlag = NEUTRAL_POSTURE_CALIBRATED;
            FLASHSaveFlag = SAVE_FLASH_BLE_DATA;
		}
	}
}

static void RecoverTotalAlgorithm(void)
{
    DetectAngleSingularity();
    if (recoverFlag == true) {
        // 초기화
        // 상체 위치 초기화: 골반 위치(P0_Improved)를 기준으로 설정 (필요시 센서 초기값 적용)
        memset(&PT_Improved, 0, sizeof(PT_Improved));
        // 허벅지 말단 위치 초기화: 골반 위치(P0_Improved)를 기준으로 설정 (필요시 센서 초기값 적용)
        memset(&P1L_Improved, 0, sizeof(P1L_Improved));
        memset(&P1R_Improved, 0, sizeof(P1R_Improved));
        // 종아리 무게중심 위치 초기화: 골반 위치(P0_Improved)를 기준으로 설정 (필요시 센서 초기값 적용)
        memset(&P20L_Improved, 0, sizeof(P20L_Improved));
        memset(&P20R_Improved, 0, sizeof(P20R_Improved));
        // 발 위치 초기화: 골반 위치(P0_Improved)를 기준으로 설정 (필요시 센서 초기값 적용)
        memset(&P2L_Improved, 0, sizeof(P2L_Improved));
        memset(&P2R_Improved, 0, sizeof(P2R_Improved));
        // 무릎 위치 초기화: 골반 위치(P0_Improved)를 기준으로 설정 (필요시 센서 초기값 적용)
        memset(&P2CL_Improved, 0, sizeof(P2CL_Improved));
        memset(&P2CR_Improved, 0, sizeof(P2CR_Improved));
        // 종아리 무게중심에서 무릎 방향 벡터 초기화: 골반 위치(P0_Improved)를 기준으로 설정 (필요시 센서 초기값 적용)
        memset(&dPLeftShank_Improved, 0, sizeof(dPLeftShank_Improved));
        memset(&dPRightShank_Improved, 0, sizeof(dPRightShank_Improved));

        // GRF 변수 0으로 초기화
        fLGRF_x = 0.0f;
        fRGRF_x = 0.0f;
        fLGRF_y = 0.0f;
        fRGRF_y = 0.0f;

        // knee 관절 에러 및 힘 변수 0으로 초기화
        memset(&errorLeft_Improved, 0, sizeof(errorLeft_Improved));
        memset(&errorRight_Improved, 0, sizeof(errorRight_Improved));
        memset(&fLeftKnee_Improved, 0, sizeof(fLeftKnee_Improved));
        memset(&fRightKnee_Improved, 0, sizeof(fRightKnee_Improved));

        // Knee relative angles 초기화 (0으로 설정)
        memset(&ThetaLK_Improved, 0, sizeof(ThetaLK_Improved));
        memset(&ThetaRK_Improved, 0, sizeof(ThetaRK_Improved));

        // Shank angle 초기화 
        memset(&ThetaLShank_Improved, 0, sizeof(ThetaLShank_Improved));
        memset(&ThetaRShank_Improved, 0, sizeof(ThetaRShank_Improved));
        
        // Knee torques 0으로 초기화
        tauLeftKnee_Improved = 0.0f;
        tauRightKnee_Improved = 0.0f;

        // 전방 위치, 속도, 누적 이동거리 초기화
        memset(&posXRaw_Improved, 0, sizeof(posXRaw_Improved));
        memset(&posX_Improved, 0, sizeof(posX_Improved));
        memset(&velX_Improved, 0, sizeof(velX_Improved));
        memset(&velXLPF_Improved, 0, sizeof(velXLPF_Improved));
        forwardDistance_Improved = 0.0f;

        // Hip ROM, 최대 각속도 초기화
        LeftHipROMFlex_Improved = 0.0f;
        LeftHipROMExt_Improved = 0.0f;
        RightHipROMFlex_Improved = 0.0f;
        RightHipROMExt_Improved = 0.0f;
        LeftHipMaxSpeed_Improved = 0.0f;
        RightHipMaxSpeed_Improved = 0.0f;

        // Gait Event 관련 변수 초기화
//        StrideCountLeft_Improved = 0;
//        StrideCountRight_Improved = 0;
        StrideLengthLeft_Improved = 0.0f;
        StrideLengthRight_Improved = 0.0f;
        StepLengthCountLeft_Improved = 0;
        StepLengthCountRight_Improved = 0;
        StepLengthLeft_Improved = 0.0f;
        StepLengthRight_Improved = 0.0f;
        Cadence_Improved = 0.0f;
        stride_time = 0.0f;
        LeftStepTime = 0.0f;
        RightStepTime = 0.0f;
        IndexLeftHeelStrike = 0;
        IndexRightHeelStrike = 0;
        IndexLeftToeOff = 0;
        IndexRightToeOff = 0;
        StepLengthLeftInMeasure_Improved = false;
        StepLengthRightInMeasure_Improved = false;
        ReferencePositionLeftFoot_Improved = 0.0f;
        ReferencePositionRightFoot_Improved = 0.0f;
        memset(&leftHeelTracker, 0, sizeof(leftHeelTracker));
        memset(&rightHeelTracker, 0, sizeof(rightHeelTracker));

        OnlineStats_Init(&cadenceStats);
        OnlineStats_Init(&stepLengthLeftStats);
        OnlineStats_Init(&stepLengthRightStats);
        OnlineStats_Init(&strideLengthLeftStats);
        OnlineStats_Init(&strideLengthRightStats);

        // Shank angle 초기값 설정
        ThetaLShank_Improved[0] = LH_Sagittal.ThighSagittalAngle;
        ThetaLShank_Improved[1] = LH_Sagittal.ThighSagittalAngle;
        ThetaLShank_Improved[2] = LH_Sagittal.ThighSagittalAngle;
        ThetaRShank_Improved[0] = RH_Sagittal.ThighSagittalAngle;
        ThetaRShank_Improved[1] = RH_Sagittal.ThighSagittalAngle;
        ThetaRShank_Improved[2] = RH_Sagittal.ThighSagittalAngle;

        // knee 관절 에러 초기값 설정
        errorLeft_Improved[0].x = 0.00001f;
        errorLeft_Improved[0].y = 0.00001f;
        errorRight_Improved[0].x = 0.00001f;
        errorRight_Improved[0].y = 0.00001f;

        errorLeft_Improved[1].x = 0.00001f;
        errorLeft_Improved[1].y = 0.00001f;
        errorRight_Improved[1].x = 0.00001f;
        errorRight_Improved[1].y = 0.00001f;

        gaitDetectLoopCnt = 0;
        recoverFlag = false;
    }
}

static void DetectAngleSingularity(void)
{
    if (ThetaLK_Improved[0] > 180.0f || ThetaLK_Improved[0] < -180.0f) {
        recoverFlag = true;
    }

    if (ThetaRK_Improved[0] > 180.0f || ThetaRK_Improved[0] < -180.0f) {
        recoverFlag = true;
    }
}

/* Post Processing */
static void UpdateForPostProcessingData(void)
{
    if (isMotionAnalysisDuring) { // 최대 배열 index 10000, down sample size가 3, DT = 0.001이므로, 총 30초 저장
        // initialize post processing data
        if (isMotionAnalysisFinishedBefore) {
            memset(stepTime, 0, sizeof(stepTime));
            memset(stepFrequency, 0, sizeof(stepFrequency));
            meanStepFreq = 0.0f;

            memset(ZPF_TempArray, 0, sizeof(ZPF_TempArray));
            memset(velX4AnalysisFiltered, 0, sizeof(velX4AnalysisFiltered));
            memset(ThetaRH4AnalysisFiltered, 0, sizeof(ThetaRH4AnalysisFiltered));
            memset(ThetaLH4AnalysisFiltered, 0, sizeof(ThetaLH4AnalysisFiltered));
            memset(TorqueRH4AnalysisFiltered, 0, sizeof(TorqueRH4AnalysisFiltered));
            memset(TorqueLH4AnalysisFiltered, 0, sizeof(TorqueLH4AnalysisFiltered));
            memset(postProcessingCntFiltered, 0, sizeof(postProcessingCntFiltered));

            tempVelX = 0.0f;
            tempThetaLH = 0.0f;
            tempThetaRH = 0.0f;
            tempTorqueLH = 0.0f;
            tempTorqueRH = 0.0f;

            sampleCnt_VelX = 0;
            sampleCnt_ThetaLH = 0;
            sampleCnt_ThetaRH = 0;
            sampleCnt_TorqueLH = 0;
            sampleCnt_TorqueRH = 0;

            sampleIndex_PostProcessingCnt = 0;
            sampleIndex_VelX = 0;
            sampleIndex_ThetaLH = 0;
            sampleIndex_ThetaRH = 0;
            sampleIndex_TorqueLH = 0;
            sampleIndex_TorqueRH = 0;

            memset(segmentBuffer, 0, sizeof(segmentBuffer));

            memset(ResampledLeftHipAngles, 0, sizeof(ResampledLeftHipAngles));
            memset(ResampledRightHipAngles, 0, sizeof(ResampledRightHipAngles));
            memset(ResampledLeftHipTorques, 0, sizeof(ResampledLeftHipTorques));
            memset(ResampledRightHipTorques, 0, sizeof(ResampledRightHipTorques));

            memset(meanResampledLeftHipAngles, 0, sizeof(meanResampledLeftHipAngles));
            memset(stdResampledLeftHipAngles, 0, sizeof(stdResampledLeftHipAngles));

            memset(meanResampledRightHipAngles, 0, sizeof(meanResampledRightHipAngles));
            memset(stdResampledRightHipAngles, 0, sizeof(stdResampledRightHipAngles));

            memset(meanResampledLeftHipTorques, 0, sizeof(meanResampledLeftHipTorques));
            memset(stdResampledLeftHipTorques, 0, sizeof(stdResampledLeftHipTorques));

            memset(meanResampledRightHipTorques, 0, sizeof(meanResampledRightHipTorques));
            memset(stdResampledRightHipTorques, 0, sizeof(stdResampledRightHipTorques));

            memset(IndexLeftHeelStrikeCalibrated, 0, sizeof(IndexLeftHeelStrikeCalibrated));
            memset(IndexLeftToeOffCalibrated, 0, sizeof(IndexLeftToeOffCalibrated));
            numLeftHeelStrike = 0;
            numLeftToeOff = 0;
            memset(IndexRightHeelStrikeCalibrated, 0, sizeof(IndexRightHeelStrikeCalibrated));
            memset(IndexRightToeOffCalibrated, 0, sizeof(IndexRightToeOffCalibrated));
            numRightHeelStrike = 0;
            numRightToeOff = 0;

            numOfLeftCycles = 0;
            numOfRightCycles = 0;
            numOfCycles = 0;

            rightStancePhase = 0.0f;
            rightSwingPhase = 0.0f;
            leftStancePhase = 0.0f;
            leftSwingPhase = 0.0f;

            leftSwingStartPhase = 0.0f;
            rightStanceStartPhase = 0.0f;
            rightSwingStartPhase = 0.0f;

            asymmetryStancePhase = 0.0f;
            asymmetrySwingPhase = 0.0f;

            isMotionAnalysisFinishedBefore = false;
        }

        if (postProcessingCnt < TOTAL_SAMPLE_COUNT) {
            // Down Sampling By 3
            float ThetaRH = Theta_Trunk_Improved + RH_Sagittal.ThighSagittalAngle;
            float ThetaLH = Theta_Trunk_Improved + LH_Sagittal.ThighSagittalAngle;
            float TorqueRH = RH_Sagittal.MotorActCurrent * TORQUE_CONSTANT_SAM * H10_SUIT_GEAR_RATIO;
            float TorqueLH = LH_Sagittal.MotorActCurrent * TORQUE_CONSTANT_SAM * H10_SUIT_GEAR_RATIO;
            AccumulateDownsample(velX_Improved[0], &tempVelX, &sampleCnt_VelX, &sampleIndex_VelX, velX4AnalysisFiltered, PP_MAX_INDEX);
            AccumulateDownsample(ThetaRH, &tempThetaRH, &sampleCnt_ThetaRH, &sampleIndex_ThetaRH, ThetaRH4AnalysisFiltered, PP_MAX_INDEX);
            AccumulateDownsample(ThetaLH, &tempThetaLH, &sampleCnt_ThetaLH, &sampleIndex_ThetaLH, ThetaLH4AnalysisFiltered, PP_MAX_INDEX);
            AccumulateDownsample(TorqueRH, &tempTorqueRH, &sampleCnt_TorqueRH, &sampleIndex_TorqueRH, TorqueRH4AnalysisFiltered, PP_MAX_INDEX);
            AccumulateDownsample(TorqueLH, &tempTorqueLH, &sampleCnt_TorqueLH, &sampleIndex_TorqueLH, TorqueLH4AnalysisFiltered, PP_MAX_INDEX);
            AccumulateDownsample_ForCnt((float)postProcessingCnt, &sampleIndex_PostProcessingCnt, postProcessingCntFiltered, PP_MAX_INDEX);
        
            postProcessingCnt++;
        } else {
            // 5m 동작 분석 종료, Cnt 30000개 일때 or 종료 신호 or untrack(직선 트랙 아닐때)일때, 양쪽(?) 보행수 16걸음 이상일 때
        }
    }
}

static inline void AccumulateDownsample_ForCnt(const float current_value, uint16_t *sampleIndex, float *filtered, uint16_t maxIndex)
{
    if (*sampleIndex >= maxIndex) return;

    // 단순히 current_value를 1/3로 나누어 저장
    filtered[*sampleIndex] = current_value / 3.0f;
    (*sampleIndex)++;
}

// down sampling을 위한 누적 및 카운트 변수를 업데이트하고, 
// DOWN_SAMPLE_COUNT에 도달하면 평균값을 filtered 배열의 해당 인덱스에 저장하는 함수입니다.
// current_value: 1ms마다 업데이트되는 현재 값
// temp: 누적합 (누적 변수의 포인터)
// sampleCnt: 누적 샘플 카운트 (포인터)
// sampleIndex: filtered 배열에 저장할 인덱스 (포인터)
// filtered: 다운샘플링 결과가 저장될 배열
// maxIndex: 배열의 최대 인덱스 (예: 10000)
static inline void AccumulateDownsample(const float current_value, float *temp,
                                        uint8_t *sampleCnt, uint16_t *sampleIndex,
                                        float *filtered, uint16_t maxIndex)
{
    // 만약 이미 최대 인덱스에 도달했다면 업데이트하지 않음
    if (*sampleIndex >= maxIndex) return;

    *temp += current_value;
    (*sampleCnt)++;
    if (*sampleCnt == DOWN_SAMPLE_COUNT) {
        filtered[*sampleIndex] = *temp / (float)DOWN_SAMPLE_COUNT;
        *sampleCnt = 0;
        *temp = 0.0f;
        (*sampleIndex)++;
    }
}

// FinalizeDownsample() 함수: 남은 샘플들이 있을 경우 평균내어 마지막 출력값으로 저장
static inline void FinalizeDownsample(float *temp, uint8_t *sampleCnt, uint16_t *sampleIndex,
                                        float *filtered, uint16_t maxIndex)
{
    if (*sampleCnt > 0 && *sampleIndex < maxIndex) {
        // 남은 샘플들의 평균 계산
        filtered[*sampleIndex] = *temp / (float)(*sampleCnt);
        (*sampleIndex)++;
        *sampleCnt = 0;
        *temp = 0.0f;
    }
}

// // Zero Phase Filter (in-place, 동적 할당 버전)
// // data: 필터링할 배열 (길이 N)
// // N: 배열 길이
// // CutOffFrequency: 차단 주파수 (Hz)
// static void ZeroPhaseFilter(float *data, int N, float CutOffFrequency) 
// {
//     float a = expf(-WHOLEBODY_CONTROL_PERIOD * 2 * M_PI * CutOffFrequency);
//     // 임시 버퍼 할당 (메모리 제한이 있다면 정적 버퍼 사용 고려)
//     float *temp = (float *)malloc(sizeof(float) * N);
//     if (temp == NULL) {
//         // 메모리 할당 실패 시 에러 처리: 여기서는 그냥 리턴
//         // TODO: Error 상황이 있는 지 확인 요망
//         return;
//     }

//     // Forward pass (전방 필터링)
//     temp[0] = data[0];
//     for (int k = 1; k < N; k++) {
//         // raw[k-1] 대신 이전 forward 값을 사용
//         temp[k] = a * temp[k - 1] + (1.0f - a) * data[k];
//     }
//     // Backward pass (역방향 필터링, zero-phase 효과)
//     data[N - 1] = temp[N - 1];
//     for (int k = N - 2; k >= 0; k--) {
//         data[k] = a * data[k + 1] + (1.0f - a) * temp[k];
//     }
//     free(temp);
// }

// Zero Phase Filter (in-place, 정적 배열 버전)
// data: 필터링할 배열 (길이 N)
// N: 배열 길이 (PP_MAX_INDEX 이하이어야 함)
// CutOffFrequency: 차단 주파수 (Hz)
static void ZeroPhaseFilter(float *data, int N, float CutOffFrequency) 
{
    float a = expf(-WHOLEBODY_CONTROL_PERIOD * DOWN_SAMPLE_COUNT * 2 * M_PI * CutOffFrequency); // 0.001 * 3은 downsampling
    
    memset(ZPF_TempArray, 0, sizeof(ZPF_TempArray));
    // 전방 패스 (표준 EMA 적용)
    ZPF_TempArray[0] = data[0];
    for (int k = 1; k < N; k++) {
        ZPF_TempArray[k] = a * ZPF_TempArray[k - 1] + (1.0f - a) * data[k];
    }

    // 후방 패스 (표준 제로 페이즈 적용)
    data[N - 1] = ZPF_TempArray[N - 1];
    for (int k = N - 2; k >= 0; k--) {
        data[k] = a * data[k + 1] + (1.0f - a) * ZPF_TempArray[k];
    }
}

// 2차원 배열용 Zero Phase Filter (제자리 처리, 정적 배열 버전)
// data: 필터링할 2차원 배열 (numGaitEvents x numSamples 크기)
// numGaitEvents: 보행 주기 수 (행 수)
// numSamples: 주기별 샘플 수 (열 수, PP_MAX_INDEX 이하)
// CutOffFrequency: 차단 주파수 (Hz)
static void ZeroPhaseFilter2D(float data[][FIXED_GAIT_SAMPLES], int numGaitEvents, int numSamples, float CutOffFrequency)
{
    // // 입력 검증
    // if (numSamples > PP_MAX_INDEX || numGaitEvents <= 0 || numSamples <= 0) {
    //     return; // 유효하지 않은 매개변수 처리
    // }

    float a = expf(-0.01 * 2 * M_PI * CutOffFrequency);

    // 각 보행 주기(행)를 독립적으로 처리
    for (int i = 0; i < numGaitEvents; i++) {
        // 해당 행의 임시 배열 초기화
        memset(ZPF_TempArray, 0, sizeof(ZPF_TempArray));

        // 전방 패스 (표준 EMA)
        ZPF_TempArray[0] = data[i][0];
        for (int k = 1; k < numSamples; k++) {
            ZPF_TempArray[k] = a * ZPF_TempArray[k - 1] + (1.0f - a) * data[i][k];
        }

        // 후방 패스 (제로 페이즈)
        data[i][numSamples - 1] = ZPF_TempArray[numSamples - 1];
        for (int k = numSamples - 2; k >= 0; k--) {
            data[i][k] = a * data[i][k + 1] + (1.0f - a) * ZPF_TempArray[k];
        }
    }
}

// 구조체 초기화 함수
static void OnlineStats_Init(OnlineStats *stats)
{
    stats->n = 0;
    stats->mean = 0.0f;
    stats->M2 = 0.0f;
}

// 새로운 값 new_val를 반영하여 온라인 통계를 업데이트
static void UpdateOnlineStats(OnlineStats *stats, float new_val)
{
    stats->n++;
    float delta = new_val - stats->mean;
    stats->mean += delta / stats->n;
    float delta2 = new_val - stats->mean;
    stats->M2 += delta * delta2;
}

// 현재까지의 표준편차를 계산 (n>=2일 때)
static float OnlineStatsGetStd(const OnlineStats *stats)
{
    if (stats->n < 2) {
        return 0.0f;
    }
    float variance = stats->M2 / (stats->n - 1);
    return sqrtf(variance);
}

// 배열 data의 길이 n에 대한 평균과 표준편차를 계산하는 함수
static void OfflineStatsGetMeanNStd(const float *data, int n, float *mean, float *std, float scale_factor)
{
    // 유효성 검사
    if (n == 0) {
        *mean = 0;
        *std = 0;
        return;
    }

    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += data[i];
    }
    *mean = sum / n  * scale_factor; //  scale_factor 1.0565 Only For VelX;

    float sq_sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float diff = data[i] - (*mean);
        sq_sum += diff * diff;
    }
    *std = sqrtf(sq_sum / n);
}

// 두 그룹의 평균을 결합하는 함수 (샘플 수가 동일한 경우)
static float CombineMean(float m1, float m2)
{
    return (m1 + m2) / 2.0f;
}

// 두 그룹의 표준편차를 결합하는 함수 (샘플 수가 동일한 경우)
// m1, std1: 그룹1의 평균과 표준편차
// m2, std2: 그룹2의 평균과 표준편차
static float CombineStd(float m1, float std1, float m2, float std2)
{
    float variance = (std1 * std1 + std2 * std2) / 2.0f + ((m1 - m2) * (m1 - m2)) / 4.0f;
    return sqrtf(variance);
}

// 선형 보간을 이용하여 input 배열을 output_length 길이로 리샘플링하는 함수
static void LinearResample_Segment(const float *input, uint32_t input_length, float *output, uint32_t output_length)
{
    // 출력 길이가 1보다 큰 경우에 대해 처리 (1이면 그냥 첫 번째 값 복사)
    if (output_length <= 1) {
        output[0] = input[0];
        return;
    }
    for (uint32_t i = 0; i < output_length; i++) {
        // 출력 인덱스 i에 대응하는 입력 배열 내의 위치 t 계산
        // t는 0 ~ (input_length-1) 사이의 실수 값을 가짐
        float t = ((float)i) / (output_length - 1) * (input_length - 1);
        uint32_t idx = (uint32_t)t;
        float frac = t - idx;
        // 만약 idx가 마지막 인덱스에 가까우면 마지막 샘플을 그대로 사용
        if (idx >= input_length - 1) {
            output[i] = input[input_length - 1];
        } else {
            // 선형 보간: 인덱스 idx와 idx+1 사이의 가중 평균 계산
            output[i] = input[idx] * (1.0f - frac) + input[idx + 1] * frac;
        }
    }
}

// -----------------------------------------------------------------------------
// 보행 주기별로 리샘플된 데이터를 계산하는 함수 (모듈화)
// input: 원본 신호 배열 (예: 왼쪽 엉덩 각도, 오른쪽 엉덩 각도, 토크 등)
// heelIndices: 해당 신호의 Heel Strike 시점(보정된 인덱스) 배열, 길이 numHeelStrike (예: IndexLeftHeelStrikeCalibrated)
// numHeelStrike: Heel Strike 시점의 총 개수
// output: 리샘플된 결과를 저장할 2차원 배열 (정적 배열 [numHeelStrike-1][FIXED_GAIT_SAMPLES])
// segmentBuffer: 호출자가 제공하는 임시 버퍼 (크기는 MAX_SEGMENT_LENGTH 이상)
// -----------------------------------------------------------------------------  
static void ResampleGaitCycleSegment(const float *input, const uint32_t *heelIndices, 
                                    uint8_t numHeelStrike, float output[][FIXED_GAIT_SAMPLES],
                                    float *segmentBuffer)
{
    // 유효성 검사 적어도 두 걸음 이상 걸어야 함
    if (numHeelStrike <= 1) {
        return;
    }

    // 각 보행 주기에 대해 리샘플링 수행 (보행 주기는 numHeelStrike - 1 개)
    for (uint8_t j = 0; j < numHeelStrike - 1; j++) {
        uint32_t startIdx = heelIndices[j];
        uint32_t endIdx = heelIndices[j + 1];
        uint32_t n = endIdx - startIdx;
        // n이 유효한 범위 내에 있는지 확인
        if (n <= 0 || n > MAX_SEGMENT_LENGTH) {continue;}   // 만약 구간 길이가 너무 짧거나 너무 길면, 해당 주기는 건너뜁니다.

        // 입력 신호 구간을 segmentBuffer 배열에 복사
        for (int i = 0; i < n; i++) {segmentBuffer[i] = input[startIdx + i];}

        // 주기성 강제 조정: 시작 값과 끝 값을 평균으로 설정을 통한 주기성 보장
        float mean_value = (segmentBuffer[0] + segmentBuffer[n - 1]) / 2.0f;
        segmentBuffer[0] = mean_value;       // 시작 값 조정 (0%)
        segmentBuffer[n - 1] = mean_value;   // 끝 값 조정 (100%)

        // bias 보정: 구간의 마지막 값(bias)을 사용하여, 모든 샘플에서 bias를 뺍니다.
        float bias = segmentBuffer[n - 1];
        for (int i = 0; i < n; i++) {segmentBuffer[i] -= bias;}

        // 선형 보간을 이용한 리샘플링: segmentBuffer 배열을 FIXED_GAIT_SAMPLES 길이로 리샘플링
        LinearResample_Segment(segmentBuffer, n, output[j], FIXED_GAIT_SAMPLES);

        // 리샘플된 결과에 bias를 다시 더합니다.
        for (int i = 0; i < FIXED_GAIT_SAMPLES; i++) {output[j][i] += bias;}
    }
}

// 배열 데이터를 주어진 퍼센트만큼 오른쪽으로 이동하는 함수
static void ShiftGaitCycle(float input[][FIXED_GAIT_SAMPLES], uint8_t numCycles, uint8_t shift_samples, float output[][FIXED_GAIT_SAMPLES])
{
    for (uint8_t j = 0; j < numCycles - 1; j++) {
        float temp[FIXED_GAIT_SAMPLES];
        // 원본 데이터를 임시 배열에 복사
        for (int i = 0; i < FIXED_GAIT_SAMPLES; i++) {
            temp[i] = input[j][i];
        }

        // 이동 후 순환 적용 (순서 유지)
        for (int i = 0; i < FIXED_GAIT_SAMPLES; i++) {
            int src_idx = (i - shift_samples + FIXED_GAIT_SAMPLES) % FIXED_GAIT_SAMPLES; // 원본 인덱스 계산
            output[j][i] = temp[src_idx];
        }

        for (int i = shift_samples - 1; i >= 0; i--) {
            output[j][i+1] = output[j][i];
        }
        output[j][0] = output[j][FIXED_GAIT_SAMPLES-1];
    }
}

//----------------------------------------------------------------------------
// Heel Strike 이벤트가 발생할 때 index를 기록(다운샘플링된)
// 이 함수는 DetectGaitEvents 내에서 호출되어야 하며, 동작분석 시작 후에만 기록
//----------------------------------------------------------------------------
static void UpdateHeelStrikeIndices(uint8_t isLeft, uint32_t *IndexLeftHeelStrike, uint32_t *IndexRightHeelStrike, 
                                    uint8_t *numLeftHeelStrike, uint8_t *numRightHeelStrike, 
                                    const uint32_t *currentDownSampledIndex)
{
    if (isLeft) {
        if (*numLeftHeelStrike < MAX_NUM_GAIT_EVENTS) {
            IndexLeftHeelStrike[*numLeftHeelStrike] = *currentDownSampledIndex;
            (*numLeftHeelStrike)++;
        }
    } else {
        if (*numRightHeelStrike < MAX_NUM_GAIT_EVENTS) {
            IndexRightHeelStrike[*numRightHeelStrike] = *currentDownSampledIndex;
            (*numRightHeelStrike)++;
        }
    }
}

static void UpdateToeOffIndices(uint8_t isLeft, uint32_t *IndexLeftToeOff, uint32_t *IndexRightToeOff, 
                                uint8_t *numLeftToeOff, uint8_t *numRightToeOff, 
                                const uint32_t *currentDownSampledIndex)
{
    if (isLeft) {
        if (*numLeftToeOff < MAX_NUM_GAIT_EVENTS) {
            IndexLeftToeOff[*numLeftToeOff] = *currentDownSampledIndex;
            (*numLeftToeOff)++;
        }
    } else {
        if (*numRightToeOff < MAX_NUM_GAIT_EVENTS) {
            IndexRightToeOff[*numRightToeOff] = *currentDownSampledIndex;
            (*numRightToeOff)++;
        }
    }
}

// Heel Strike 시점 저장 보정
static inline void AlignEventIndices(float temp, uint8_t sampleCnt, uint16_t sampleIndex, 
                                            float *filtered, uint16_t maxIndex)
{
    if (sampleCnt > 0 && sampleIndex < maxIndex) {
        // 남은 샘플들의 평균 계산
        filtered[sampleIndex] = temp / (float)(sampleCnt) / (float)DOWN_SAMPLE_COUNT;
    }
}

/**
 * @brief 2차원 배열의 각 열에 대해 평균과 (샘플) 표준편차를 계산합니다.
 * @param data          입력 2차원 배열 (크기: numRows x FIXED_GAIT_SAMPLES)
 * @param numRows       배열의 행 개수
 * @param mean_out      각 열의 평균 결과를 저장할 배열 (크기: FIXED_GAIT_SAMPLES)
 * @param std_out       각 열의 표준편차 결과를 저장할 배열 (크기: FIXED_GAIT_SAMPLES)
 */
static void ComputeResampledStats(const float data[][FIXED_GAIT_SAMPLES],
                          int numRows,
						  float mean_out[FIXED_GAIT_SAMPLES],
						  float std_out[FIXED_GAIT_SAMPLES])
{
    for (int col = 0; col < FIXED_GAIT_SAMPLES; col++) {
        float sum = 0.0f;
        // 각 열에 대해 합계 계산
        for (int row = 0; row < numRows; row++) {
            sum += data[row][col];
        }
        float mean = sum / numRows;
        mean_out[col] = mean;
        
        // 분산 계산 (자유도: numRows-1)
        float var = 0.0f;
        if (numRows > 1) {
            for (int row = 0; row < numRows; row++) {
                float diff = data[row][col] - mean;
                var += diff * diff;
            }
            var /= (numRows - 1);
        } else {
            var = 0.0f;
        }
        std_out[col] = sqrtf(var);
    }
}

// ComputeNumCycles: 보행 주기 수를 계산하는 함수
// numLeftHeel: 왼쪽 Heel Strike 이벤트 수 (예: IndexLeftHeelStrike 배열의 길이)
// numLeftToe:  왼쪽 Toe Off 이벤트 수 (예: IndexLeftToeOff 배열의 길이)
// numRightHeel: 오른쪽 Heel Strike 이벤트 수 (예: IndexRightHeelStrike 배열의 길이)
// numRightToe:  오른쪽 Toe Off 이벤트 수 (예: IndexRightToeOff 배열의 길이)
// numOfLeftCycles: 좌측 보행 주기 수 (min( numLeftHeel, numLeftToe ))
// numOfRightCycles: 우측 보행 주기 수 (min( numRightHeel, numRightToe ))
// numOfCycles: 전체 보행 주기 수 (min(numOfLeftCycles, numOfRightCycles))
static void ComputeNumCycles(uint8_t numLeftHeel, uint8_t numLeftToe,
                             uint8_t numRightHeel, uint8_t numRightToe,
                             uint8_t *numOfLeftCycles, uint8_t *numOfRightCycles, uint8_t *numOfCycles)
{
    // 각 측면의 보행 주기는 해당 Toe Off 이벤트 수와 Heel Strike 이벤트 수 중 작은 값으로 정의
    *numOfLeftCycles = (numLeftToe < numLeftHeel) ? numLeftToe : numLeftHeel;
    *numOfRightCycles = (numRightToe < numRightHeel) ? numRightToe : numRightHeel;
    // 전체 보행 주기 수는 양쪽 주기 수의 최소값
    *numOfCycles = ((*numOfLeftCycles) < (*numOfRightCycles)) ? (*numOfLeftCycles) : (*numOfRightCycles);
}

static void ComputeGaitPhase(const uint32_t *IndexLeftHeel, const uint32_t *IndexLeftToe,
                            const uint32_t *IndexRightHeel, const uint32_t *IndexRightToe,
                            const uint8_t *numCycles,
                            float *meanLeftSwing, float *meanRightStance, float *meanRightSwing)
{
    float cumulativeLeftSwing = 0.0f;
    float cumulativeRightStance = 0.0f;
    float cumulativeRightSwing = 0.0f;
    uint8_t validCycles = 0;

    // 반복문: 보행 주기는 0부터 numCycles-2 (총 numCycles-1 주기)
    for (uint8_t k = 0; k < *numCycles - 1; k++) {
        // 한 보행 주기의 전체 길이 (예: 왼쪽 Heel Strike 간격)
        float wholeCycle = (float)((int32_t)IndexLeftHeel[k+1] - (int32_t)IndexLeftHeel[k]);
        if (wholeCycle <= 0) continue; // 0으로 나누는 것을 방지하고 이 주기는 건너뜀뜀

        // Left Swing Start (%) 계산: (Left Toe Off - Left Heel Strike) / Whole Cycle * 100
        float leftSwingStartValue = 100.0f * (float)((int32_t)IndexLeftToe[k] - (int32_t)IndexLeftHeel[k]) / wholeCycle;

        // Right Stance Start (%) 계산: (Right Heel Strike - Left Heel Strike) / Whole Cycle * 100, 모듈로 100 적용
        float rightStanceStartValue = fmodf(100.0f * (((int32_t)IndexRightHeel[k] - (int32_t)IndexLeftHeel[k]) / wholeCycle), 100.0f);
        if (rightStanceStartValue < 0) rightStanceStartValue += 100.0f;

        // Right Swing Start (%) 계산: (Right Toe Off - Left Heel Strike) / Whole Cycle * 100, 모듈로 100 적용
        float rightSwingStartValue = fmodf(100.0f * ((float)((int32_t)IndexRightToe[k] - (int32_t)IndexLeftHeel[k]) / wholeCycle), 100.0f);
        if (rightSwingStartValue < 0) rightSwingStartValue += 100.0f;

        cumulativeLeftSwing += leftSwingStartValue;
        cumulativeRightStance += rightStanceStartValue;
        cumulativeRightSwing += rightSwingStartValue;
        validCycles++;
    }

    if (validCycles > 0) {
        *meanLeftSwing = cumulativeLeftSwing / validCycles;
        *meanRightStance = cumulativeRightStance / validCycles;
        *meanRightSwing = cumulativeRightSwing / validCycles;
    } else {
        *meanLeftSwing = 0;
        *meanRightStance = 0;
        *meanRightSwing = 0;
    }
}

// 입각기와 유각기 비율 계산의 정의
// 입각기(Stance Phase): 발이 지면에 닿아 있는 기간으로, 일반적으로 Heel Strike(발뒤꿈치 접지)에서 Toe Off(발끝 이탈)까지의 시간 비율.
// 유각기(Swing Phase): 발이 지면에서 떨어져 있는 기간으로, Toe Off에서 다음 Heel Strike까지의 시간 비율.
// 보행 주기(Gait Cycle): 한쪽 발의 연속된 두 Heel Strike 사이의 전체 시간(Stride Time)으로 정의되며, 입각기와 유각기의 합은 100%가 되어야 함.
// 정상 보행 기준: 건강한 성인의 경우 입각기 약 60%, 유각기 약 40% (문헌: Perry, J., Gait Analysis: Normal and Pathological Function).
// numLeftHeelStrike: 왼쪽 Heel Strike 이벤트 수
// numLeftToeOff:  왼쪽 Toe Off 이벤트 수
// numRightHeelStrike: 오른쪽 Heel Strike 이벤트 수
// numRightToeOff:  오른쪽 Toe Off 이벤트 수
// numLeftCycles: 좌측 보행 주기 수
// numRightCycles: 우측 보행 주기 수
// numCycles: 전체 보행 주기 수
static void ComputeGaitPhaseRatios(
    const uint32_t* IndexLeftHeelStrikeCalibrated, const uint32_t* IndexLeftToeOffCalibrated,
    const uint32_t* IndexRightHeelStrikeCalibrated, const uint32_t* IndexRightToeOffCalibrated,
    uint8_t numLeftHeelStrike, uint8_t numLeftToeOff,
    uint8_t numRightHeelStrike, uint8_t numRightToeOff,
    float* leftStancePhase, float* leftSwingPhase,
    float* rightStancePhase, float* rightSwingPhase
)
{
    // (1) 주기 수 계산
    uint8_t numLeftCycles = (numLeftToeOff < numLeftHeelStrike) ? numLeftToeOff : numLeftHeelStrike;
    uint8_t numRightCycles = (numRightToeOff < numRightHeelStrike) ? numRightToeOff : numRightHeelStrike;
    uint8_t numCycles = (numLeftCycles < numRightCycles) ? numLeftCycles : numRightCycles;

    // 최소 2개 주기 필요
    if (numCycles < 2) {
        *leftStancePhase = 0.0f;
        *leftSwingPhase = 0.0f;
        *rightStancePhase = 0.0f;
        *rightSwingPhase = 0.0f;
        // TODO: 모수가 작아서 더 걸어야 함!! Risk Management 처리 필요(Tablet Display로 띄운다든지)
        return;
    }

    // (2) 변수 초기화
    float leftStanceAvg = 0.0f, leftSwingAvg = 0.0f;
    float rightStanceAvg = 0.0f, rightSwingAvg = 0.0f;
    uint8_t validCycles = 0;

    // (3) 각 주기의 입각기와 유각기 비율 계산
    for (uint8_t k = 0; k < numCycles - 1; k++) {
        // 주기 길이 계산 (int32_t로 캐스팅하여 오버플로우 방지)
        float leftCycleDuration = (float)((int32_t)IndexLeftHeelStrikeCalibrated[k + 1] - (int32_t)IndexLeftHeelStrikeCalibrated[k]);
        float rightCycleDuration = (float)((int32_t)IndexRightHeelStrikeCalibrated[k + 1] - (int32_t)IndexRightHeelStrikeCalibrated[k]);

        // 데이터 유효성 체크
        if (leftCycleDuration <= 0.0f || 
            (int32_t)IndexLeftToeOffCalibrated[k] <= (int32_t)IndexLeftHeelStrikeCalibrated[k] || 
            (int32_t)IndexLeftToeOffCalibrated[k] >= (int32_t)IndexLeftHeelStrikeCalibrated[k + 1]) {
            continue;  // 비정상 주기 스킵
            // TODO: Risk Management 처리 필요(Tablet Display로 띄운다든지)
        }
        if (rightCycleDuration <= 0.0f || 
            (int32_t)IndexRightToeOffCalibrated[k] <= (int32_t)IndexRightHeelStrikeCalibrated[k] || 
            (int32_t)IndexRightToeOffCalibrated[k] >= (int32_t)IndexRightHeelStrikeCalibrated[k + 1]) {
            continue;  // 비정상 주기 스킵
            // TODO: Risk Management 처리 필요(Tablet Display로 띄운다든지)
        }

        // 왼쪽 입각기 비율
        float leftStanceRatio = 100.0f * (float)((int32_t)IndexLeftToeOffCalibrated[k] - (int32_t)IndexLeftHeelStrikeCalibrated[k]) / leftCycleDuration;
        leftStanceRatio = fminf(100.0f, fmaxf(0.0f, leftStanceRatio));  // 0~100% 제한
        leftStanceAvg = (leftStanceAvg * validCycles + leftStanceRatio) / (validCycles + 1);
        // leftSwingAvg = 100.0f - leftStanceAvg;

        // 오른쪽 입각기 비율
        float rightStanceRatio = 100.0f * (float)((int32_t)IndexRightToeOffCalibrated[k] - (int32_t)IndexRightHeelStrikeCalibrated[k]) / rightCycleDuration;
        rightStanceRatio = fminf(100.0f, fmaxf(0.0f, rightStanceRatio));  // 0~100% 제한
        rightStanceAvg = (rightStanceAvg * validCycles + rightStanceRatio) / (validCycles + 1);
        // rightSwingAvg = 100.0f - rightStanceAvg;

        validCycles++;
    }

    // (4) 결과 할당
    if (validCycles > 0) {
        *leftStancePhase = leftStanceAvg * 1.030887068;
        *leftSwingPhase = 100 - *leftStancePhase;
        *rightStancePhase = rightStanceAvg * 1.030887068;
        *rightSwingPhase = 100 - *rightStancePhase;

        // 실측 Data 기반 Scale Factors 적용
        *leftSwingPhase = *leftSwingPhase * 0.9824;
        *leftStancePhase = 100 - *leftSwingPhase;
        *rightSwingPhase = *rightSwingPhase * 0.9824;
        *rightStancePhase = 100 - *rightSwingPhase;
    } else {
        *leftStancePhase = 0.0f;
        *leftSwingPhase = 0.0f;
        *rightStancePhase = 0.0f;
        *rightSwingPhase = 0.0f;
        // TODO: 모수가 작아서 더 걸어야 함!! Risk Management 처리 필요(Tablet Display로 띄운다든지)
    }
}

// 비대칭 지수 계산 함수
// leftValue: 왼쪽 평균 값 (예: 입각기 또는 유각기 %)
// rightValue: 오른쪽 평균 값
// Returns: 비대칭 지수 (백분율)
// side: 결과 문자열에 "Left-dominant", "Right-dominant" 또는 "Symmetric"을 저장할 버퍼(충분한 크기 필요)
static float ComputeAsymmetryIndex(float leftValue, float rightValue)
{
    float meanVal = (leftValue + rightValue) / 2.0f;
    float asymmetry = 0.0f;
    
    if (meanVal == 0.0f) {
        asymmetry = 25.0f;
    } else {
        asymmetry = (rightValue - leftValue) / meanVal * 100.0f;
        asymmetry += 25.0f;
        asymmetry = fmax(0.0, fmin(50.0, asymmetry));
    }

    return asymmetry;
}

static float GetMinValue(float arr[FIXED_GAIT_SAMPLES])
{
    float minVal = arr[0];
    for (uint8_t i = 1; i < FIXED_GAIT_SAMPLES; i++) {
        if (arr[i] < minVal) { minVal = arr[i]; }
    }
    return minVal;
}

static float GetMaxValue(float arr[FIXED_GAIT_SAMPLES])
{
    float maxVal = arr[0];
    for (uint8_t i = 1; i < FIXED_GAIT_SAMPLES; i++) {
        if (arr[i] > maxVal) { maxVal = arr[i]; }
    }
    return maxVal;
}

static void ComputeStepFrequency(
    uint32_t* leftHS, uint8_t numLeftHS,
    uint32_t* rightHS, uint8_t numRightHS,
    float* stepTime, float* stepFrequency, float* meanStepFreq
)
{
    // (1) Heel Strike 이벤트 수집 및 정렬
    uint32_t allHeelStrikes[MAX_NUM_GAIT_EVENTS * 2];
    uint8_t totalHS = numLeftHS + numRightHS;
    if (totalHS <= 1) {
        *meanStepFreq = 2.0f; // 2.0 Hz로 제한
        return;
    }
    if (totalHS > MAX_NUM_GAIT_EVENTS * 2) totalHS = MAX_NUM_GAIT_EVENTS * 2;

    for (uint8_t i = 0; i < numLeftHS; i++) {
        allHeelStrikes[i] = leftHS[i];
    }
    for (uint8_t i = 0; i < numRightHS; i++) {
        allHeelStrikes[numLeftHS + i] = rightHS[i];
    }

    // 버블 정렬
    for (uint8_t i = 0; i < totalHS - 1; i++) {
        for (uint8_t j = 0; j < totalHS - i - 1; j++) {
            if (allHeelStrikes[j] > allHeelStrikes[j + 1]) {
                uint32_t temp = allHeelStrikes[j];
                allHeelStrikes[j] = allHeelStrikes[j + 1];
                allHeelStrikes[j + 1] = temp;
            }
        }
    }

    // (2) Step Time 계산
    float dt = WHOLEBODY_CONTROL_PERIOD * DOWN_SAMPLE_COUNT;
    for (uint8_t i = 0; i < totalHS - 1; i++) {
        stepTime[i] = (float)(allHeelStrikes[i + 1] - allHeelStrikes[i]) * dt;
    }

    // (3) Step Frequency 계산
    for (uint8_t i = 0; i < totalHS - 1; i++) {
        if (stepTime[i] > 0.0f) {
            stepFrequency[i] = 1.0f / stepTime[i];
            stepFrequency[i] = fmaxf(1.0f, fminf(3.0f, stepFrequency[i])); // 1.0~3.0 Hz 제한
        } else {
            stepFrequency[i] = 2.0f; // 0 이하일 경우 2.0으로 고정
        }
    }

    // (4) 평균 Step Frequency 계산
    if (totalHS > 1) {
        float sum = 0.0f;
        for (uint8_t i = 0; i < totalHS - 1; i++) {
            sum += stepFrequency[i];
        }
        *meanStepFreq = sum / (totalHS - 1);
    }
}

#endif /* SUIT_MINICM_ENABLED */

