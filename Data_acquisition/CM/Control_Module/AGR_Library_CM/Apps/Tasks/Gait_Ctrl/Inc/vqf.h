/*
 * vqf.h
 *
 *  Created on: Nov 14, 2023
 *      Author: EXOLAB_KTY
 *      Revisor: INVINCIBLENESS
 */

#ifndef GAIT_CTRL_INC_VQF_H_
#define GAIT_CTRL_INC_VQF_H_

#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <assert.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define VQF_NO_MOTION_BIAS_ESTIMATION

#ifndef VQF_SINGLE_PRECISION
typedef float VQF_Real_t;
#else
typedef double VQF_Real_t;
#endif


#define EPS		1e-7
#define NaN		NAN

/**
*------------------------------------------------------------
*                     TYPE DECLARATIONS
*------------------------------------------------------------
* @brief Custom data types and structures for the module.
*/

typedef struct _VQF_Params_t {
	VQF_Real_t tauAcc;
	VQF_Real_t tauMag;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
	int motionBiasEstEnabled;
    VQF_Real_t biasSigmaMotion;
    VQF_Real_t biasVerticalForgettingFactor;
#endif
	int restBiasEstEnabled;
	int magDistRejectionEnabled;
	VQF_Real_t biasSigmaInit;
	VQF_Real_t biasForgettingTime;
	VQF_Real_t biasClip;
    VQF_Real_t biasSigmaRest;
    VQF_Real_t restMinT;
    VQF_Real_t restFilterTau;
    VQF_Real_t restThGyr;
    VQF_Real_t restThAcc;
    VQF_Real_t magCurrentTau;
    VQF_Real_t magRefTau;
    VQF_Real_t magNormTh;
    VQF_Real_t magDipTh;
    VQF_Real_t magNewTime;
    VQF_Real_t magNewFirstTime;
    VQF_Real_t magNewMinGyr;
    VQF_Real_t magMinUndisturbedTime;
    VQF_Real_t magMaxRejectionTime;
    VQF_Real_t magRejectionFactor;
} VQF_Params_t;


typedef struct _VQF_State_t {
    VQF_Real_t gyrQuat[4];
    VQF_Real_t accQuat[4];
    VQF_Real_t delta;
    int restDetected;
    int magDistDetected;
    VQF_Real_t lastAccLp[3];
    double accLpState[3 * 2];
    VQF_Real_t lastAccCorrAngularRate;
    VQF_Real_t kMagInit;
    VQF_Real_t lastMagDisAngle;
    VQF_Real_t lastMagCorrAngularRate;
    VQF_Real_t bias[3];
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    VQF_Real_t biasP[9];
    double motionBiasEstRLpState[9 * 2];
    double motionBiasEstBiasLpState[2 * 2];
#else
    VQF_Real_t biasP;
#endif
    VQF_Real_t restLastSquaredDeviations[2];
    VQF_Real_t restT;
    VQF_Real_t restLastGyrLp[3];
    double restGyrLpState[3 * 2];
    VQF_Real_t restLastAccLp[3];
    double restAccLpState[3 * 2];
    VQF_Real_t magRefNorm;
    VQF_Real_t magRefDip;
    VQF_Real_t magUndisturbedT;
    VQF_Real_t magRejectT;
    VQF_Real_t magCandidateNorm;
    VQF_Real_t magCandidateDip;
    VQF_Real_t magCandidateT;
    VQF_Real_t magNormDip[2];
    double magNormDipLpState[2 * 2];
} VQF_State_t;


typedef struct _VQF_Coeffs_t {
    VQF_Real_t gyrTs;
    VQF_Real_t accTs;
    VQF_Real_t magTs;
    double accLpB[3];
    double accLpA[2];
    VQF_Real_t kMag;
    VQF_Real_t biasP0;
    VQF_Real_t biasV;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    VQF_Real_t biasMotionW;
    VQF_Real_t biasVerticalW;
#endif
    VQF_Real_t biasRestW;
    double restGyrLpB[3];
    double restGyrLpA[2];
    double restAccLpB[3];
    double restAccLpA[2];
    VQF_Real_t kMagRef;
    double magNormDipLpB[3];
    double magNormDipLpA[2];
} VQF_Coeffs_t;


typedef struct _VQF_t {
    VQF_Params_t params;
    VQF_State_t state;
    VQF_Coeffs_t coeffs;
} VQF_t;


typedef struct _VQF_MagCalib_t {
	float a11;
	float a12;
	float a13;
	float a21;
	float a22;
	float a23;
	float a31;
	float a32;
	float a33;

	float b1;
	float b2;
	float b3;

	float A_inv[3][3];
	float ironErr[3];
} VQF_MagCalib_t;



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

double VQF_Square(double x);
void VQF_ParamsInit(VQF_Params_t* vqfParams);
void VQF_FilterCoeffs(VQF_Real_t tau, VQF_Real_t Ts, double outB[], double outA[]);
VQF_Real_t VQF_GainFromTau(VQF_Real_t tau, VQF_Real_t Ts);

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF_Matrix3SetToScaledIdentity(VQF_Real_t scale, VQF_Real_t out[9]);
void VQF_SetMotionBiasEstEnabled(VQF_t* vqf, int enabled);
void VQF_Matrix3Multiply(const VQF_Real_t in1[9], const VQF_Real_t in2[9], VQF_Real_t out[9]);
void VQF_Matrix3MultiplyTpsFirst(const VQF_Real_t in1[9], const VQF_Real_t in2[9], VQF_Real_t out[9]);
void VQF_Matrix3MultiplyTpsSecond(const VQF_Real_t in1[9], const VQF_Real_t in2[9], VQF_Real_t out[9]);
int VQF_Matrix3Inv(const VQF_Real_t in[9], VQF_Real_t out[9]);
#endif
void VQF_ResetState(VQF_t* vqf);
void VQF_Setup(VQF_t* vqf);
void VQF_Init(VQF_t* vqf, VQF_Real_t gyrTs, VQF_Real_t accTs, VQF_Real_t magTs);
void VQF_InitWithParams(VQF_t* vqf, const VQF_Params_t* params, VQF_Real_t gyrTs, VQF_Real_t accTs, VQF_Real_t magTs);

void VQF_FilterInitialState(VQF_Real_t x0, const double b[3], const double a[2], double out[]);
VQF_Real_t VQF_FilterStep(VQF_Real_t x, const double b[3], const double a[2], double state[2]);
void VQF_FilterVec(const VQF_Real_t x[], size_t N, VQF_Real_t tau, VQF_Real_t Ts, const double b[3], const double a[2], double state[], VQF_Real_t out[]);
void VQF_QuatMultiply(const VQF_Real_t q1[4], const VQF_Real_t q2[4], VQF_Real_t out[4]);
VQF_Real_t VQF_Norm(const VQF_Real_t vec[], size_t N);
void VQF_Normalize(VQF_Real_t vec[], size_t N);
void VQF_UpdateGyr(VQF_t* vqf, const VQF_Real_t gyr[3]);

void VQF_QuatRotate(const VQF_Real_t q[4], const VQF_Real_t v[3], VQF_Real_t out[3]);
void VQF_Clip(VQF_Real_t vec[], size_t N, VQF_Real_t min, VQF_Real_t max);
void VQF_UpdateAcc(VQF_t* vqf, const VQF_Real_t acc[3]);

void VQF_GetQuat6D(const VQF_t* vqf, VQF_Real_t out[4]);
void VQF_UpdateMag(VQF_t* vqf, const VQF_Real_t mag[3]);

void VQF_Update(VQF_t* vqf, const VQF_Real_t gyr[3], const VQF_Real_t acc[3]);
void VQF_UpdateWithMag(VQF_t* vqf, const VQF_Real_t gyr[3], const VQF_Real_t acc[3], const VQF_Real_t mag[3]);
void VQF_GetQuat3D(const VQF_t* vqf, VQF_Real_t out[4]);
void VQF_QuatApplyDelta(const VQF_Real_t q[], VQF_Real_t delta, VQF_Real_t out[]);
void VQF_GetQuat9D(const VQF_t* vqf, VQF_Real_t out[4]);
VQF_Real_t VQF_GetDelta(const VQF_t* vqf);
VQF_Real_t VQF_GetBiasEstimate(const VQF_t* vqf, VQF_Real_t out[3]);
void VQF_SetBiasEstimate(VQF_t* vqf, VQF_Real_t bias[3], VQF_Real_t sigma);
int VQF_GetRestDetected(const VQF_t* vqf);
int VQF_GetMagDistDetected(const VQF_t* vqf);
void VQF_GetRelativeRestDeviations(const VQF_t* vqf, VQF_Real_t out[2]);
void VQF_GetRelativeRestDeviations(const VQF_t* vqf, VQF_Real_t out[2]);
VQF_Real_t VQF_GetMagRefNorm(const VQF_t* vqf);
VQF_Real_t VQF_GetMagRefDip(const VQF_t* vqf);
void VQF_SetMagRef(VQF_t* vqf, VQF_Real_t norm, VQF_Real_t dip);

void VQF_SetRestBiasEstEnabled(VQF_t* vqf, int enabled);
void VQF_SetMagDistRejectionEnabled(VQF_t* vqf, int enabled);
void VQF_FilterAdaptStateForCoeffChange(VQF_Real_t last_y[], size_t N, const double b_old[], const double a_old[], const double b_new[], const double a_new[], double state[]);
void VQF_SetTauAcc(VQF_t* vqf, VQF_Real_t tauAcc);
void VQF_setTauMag(VQF_t* vqf, VQF_Real_t tauMag);

void VQF_SetRestDetectionThresholds(VQF_t* vqf, VQF_Real_t thGyr, VQF_Real_t thAcc);
VQF_Params_t VQF_GetParams(const VQF_t* vqf);
VQF_Coeffs_t VQF_GetCoeffs(const VQF_t* vqf);
VQF_State_t VQF_GetState(const VQF_t* vqf);
void VQF_SetState(VQF_t* vqf, const VQF_State_t* state);
void VQF_QuatConj(const VQF_Real_t q[4], VQF_Real_t out[4]);


#endif /* GAIT_CTRL_INC_VQF_H_ */
