/*
 * vqf.c
 *
 *  Created on: Nov 14, 2023
 *      Author: EXOLAB_KTY
 *      Revisor: INVINCIBLENESS
 */

#include "vqf.h"

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

/*
*Utility function to square a value
*/
double VQF_Square(double x)
{
    return x * x;
}

/*
*VQFParams structure initialization
*/
void VQF_ParamsInit(VQF_Params_t* vqfParams)
{
    vqfParams->tauAcc = 3.0;
    vqfParams->tauMag = 9.0;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqfParams->motionBiasEstEnabled = true;
    vqfParams->biasSigmaMotion = 0.1;
    vqfParams->biasVerticalForgettingFactor = 0.0001;
#endif
    vqfParams->restBiasEstEnabled = true;
    vqfParams->magDistRejectionEnabled = true;
    vqfParams->biasSigmaInit = 0.5;
    vqfParams->biasForgettingTime = 100.0;
    vqfParams->biasClip = 2.0;
    vqfParams->biasSigmaRest = 0.03;
    vqfParams->restMinT = 1.5;
    vqfParams->restFilterTau = 0.5;
    vqfParams->restThGyr = 2.0;
    vqfParams->restThAcc = 0.5;
    vqfParams->magCurrentTau = 0.05;
    vqfParams->magRefTau = 20.0;
    vqfParams->magNormTh = 0.1;
    vqfParams->magDipTh = 10.0;
    vqfParams->magNewTime = 20.0;
    vqfParams->magNewFirstTime = 5.0;
    vqfParams->magNewMinGyr = 20.0;
    vqfParams->magMinUndisturbedTime = 0.5;
    vqfParams->magMaxRejectionTime = 60.0;
    vqfParams->magRejectionFactor = 2.0;
}

void VQF_FilterCoeffs(VQF_Real_t tau, VQF_Real_t Ts, double outB[], double outA[])
{
    assert(tau > 0);
    assert(Ts > 0);

    // second order Butterworth filter based on https://stackoverflow.com/a/52764064
    double fc = (M_SQRT2 / (2.0 * M_PI)) / tau; // time constant of dampened, non-oscillating part of step response
    double C = tan(M_PI * fc * Ts);
    double D = C * C + sqrt(2) * C + 1;
    double b0 = C * C / D;
    outB[0] = b0;
    outB[1] = 2 * b0;
    outB[2] = b0;
    // a0 = 1.0
    outA[0] = 2 * (C * C - 1) / D; // a1
    outA[1] = (1 - sqrt(2) * C + C * C) / D; // a2
}

VQF_Real_t VQF_GainFromTau(VQF_Real_t tau, VQF_Real_t Ts)
{
    assert(Ts > 0);
    if (tau < (VQF_Real_t)(0.0)) {
        return 0; // k=0 for negative tau (disable update)
    }
    else if (tau == (VQF_Real_t)(0.0)) {
        return 1; // k=1 for tau=0
    }
    else {
        return 1 - exp(-Ts / tau);  // fc = 1/(2*pi*tau)
    }
}


void VQF_QuatSetToIdentity(VQF_Real_t out[4])
{
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}


#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF_Matrix3SetToScaledIdentity(VQF_Real_t scale, VQF_Real_t out[9])
{
    out[0] = scale;
    out[1] = 0.0;
    out[2] = 0.0;
    out[3] = 0.0;
    out[4] = scale;
    out[5] = 0.0;
    out[6] = 0.0;
    out[7] = 0.0;
    out[8] = scale;
}

void VQF_Matrix3Multiply(const VQF_Real_t in1[9], const VQF_Real_t in2[9], VQF_Real_t out[9])
{
    VQF_Real_t tmp[9];
    tmp[0] = in1[0] * in2[0] + in1[1] * in2[3] + in1[2] * in2[6];
    tmp[1] = in1[0] * in2[1] + in1[1] * in2[4] + in1[2] * in2[7];
    tmp[2] = in1[0] * in2[2] + in1[1] * in2[5] + in1[2] * in2[8];
    tmp[3] = in1[3] * in2[0] + in1[4] * in2[3] + in1[5] * in2[6];
    tmp[4] = in1[3] * in2[1] + in1[4] * in2[4] + in1[5] * in2[7];
    tmp[5] = in1[3] * in2[2] + in1[4] * in2[5] + in1[5] * in2[8];
    tmp[6] = in1[6] * in2[0] + in1[7] * in2[3] + in1[8] * in2[6];
    tmp[7] = in1[6] * in2[1] + in1[7] * in2[4] + in1[8] * in2[7];
    tmp[8] = in1[6] * in2[2] + in1[7] * in2[5] + in1[8] * in2[8];
    memcpy(out, tmp, 9 * sizeof(*tmp));
}

void VQF_Matrix3MultiplyTpsFirst(const VQF_Real_t in1[9], const VQF_Real_t in2[9], VQF_Real_t out[9])
{
    VQF_Real_t tmp[9];
    tmp[0] = in1[0] * in2[0] + in1[3] * in2[3] + in1[6] * in2[6];
    tmp[1] = in1[0] * in2[1] + in1[3] * in2[4] + in1[6] * in2[7];
    tmp[2] = in1[0] * in2[2] + in1[3] * in2[5] + in1[6] * in2[8];
    tmp[3] = in1[1] * in2[0] + in1[4] * in2[3] + in1[7] * in2[6];
    tmp[4] = in1[1] * in2[1] + in1[4] * in2[4] + in1[7] * in2[7];
    tmp[5] = in1[1] * in2[2] + in1[4] * in2[5] + in1[7] * in2[8];
    tmp[6] = in1[2] * in2[0] + in1[5] * in2[3] + in1[8] * in2[6];
    tmp[7] = in1[2] * in2[1] + in1[5] * in2[4] + in1[8] * in2[7];
    tmp[8] = in1[2] * in2[2] + in1[5] * in2[5] + in1[8] * in2[8];
    memcpy(out, tmp, 9 * sizeof(*tmp));
}

void VQF_Matrix3MultiplyTpsSecond(const VQF_Real_t in1[9], const VQF_Real_t in2[9], VQF_Real_t out[9])
{
    VQF_Real_t tmp[9];
    tmp[0] = in1[0] * in2[0] + in1[1] * in2[1] + in1[2] * in2[2];
    tmp[1] = in1[0] * in2[3] + in1[1] * in2[4] + in1[2] * in2[5];
    tmp[2] = in1[0] * in2[6] + in1[1] * in2[7] + in1[2] * in2[8];
    tmp[3] = in1[3] * in2[0] + in1[4] * in2[1] + in1[5] * in2[2];
    tmp[4] = in1[3] * in2[3] + in1[4] * in2[4] + in1[5] * in2[5];
    tmp[5] = in1[3] * in2[6] + in1[4] * in2[7] + in1[5] * in2[8];
    tmp[6] = in1[6] * in2[0] + in1[7] * in2[1] + in1[8] * in2[2];
    tmp[7] = in1[6] * in2[3] + in1[7] * in2[4] + in1[8] * in2[5];
    tmp[8] = in1[6] * in2[6] + in1[7] * in2[7] + in1[8] * in2[8];
    memcpy(out, tmp, 9 * sizeof(*tmp));
}

int VQF_Matrix3Inv(const VQF_Real_t in[9], VQF_Real_t out[9]) {
    // in = [a b c; d e f; g h i]
    double A = in[4] * in[8] - in[5] * in[7]; // (e*i - f*h)
    double D = in[2] * in[7] - in[1] * in[8]; // -(b*i - c*h)
    double G = in[1] * in[5] - in[2] * in[4]; // (b*f - c*e)
    double B = in[5] * in[6] - in[3] * in[8]; // -(d*i - f*g)
    double E = in[0] * in[8] - in[2] * in[6]; // (a*i - c*g)
    double H = in[2] * in[3] - in[0] * in[5]; // -(a*f - c*d)
    double C = in[3] * in[7] - in[4] * in[6]; // (d*h - e*g)
    double F = in[1] * in[6] - in[0] * in[7]; // -(a*h - b*g)
    double I = in[0] * in[4] - in[1] * in[3]; // (a*e - b*d)

    double det = in[0] * A + in[1] * B + in[2] * C; // a*A + b*B + c*C;

    if (det >= -EPS && det <= EPS) {
        memset(out, 0, sizeof(double) * 9);
        return 0; // false in C
    }

    // out = [A D G; B E H; C F I]/det
    out[0] = A / det;
    out[1] = D / det;
    out[2] = G / det;
    out[3] = B / det;
    out[4] = E / det;
    out[5] = H / det;
    out[6] = C / det;
    out[7] = F / det;
    out[8] = I / det;

    return 1;
}
#endif


void VQF_ResetState(VQF_t* vqf)
{
    VQF_QuatSetToIdentity(vqf->state.gyrQuat);
    VQF_QuatSetToIdentity(vqf->state.accQuat);
    vqf->state.delta = 0.0;

    vqf->state.restDetected = 0; // false in C
    vqf->state.magDistDetected = 1; // true in C

    memset(vqf->state.lastAccLp, 0, sizeof(double) * 3);
    for (int i = 0; i < 3 * 2; i++) {
        vqf->state.accLpState[i] = NAN;
    }
    vqf->state.lastAccCorrAngularRate = 0.0;

    vqf->state.kMagInit = 1.0;
    vqf->state.lastMagDisAngle = 0.0;
    vqf->state.lastMagCorrAngularRate = 0.0;

    memset(vqf->state.bias, 0, sizeof(double) * 3);

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    VQF_Matrix3SetToScaledIdentity(vqf->coeffs.biasP0, &(vqf->state.biasP));
#else
    vqf->state.biasP = vqf->coeffs.biasP0;
#endif

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    for (int i = 0; i < 9 * 2; i++) {
        vqf->state.motionBiasEstRLpState[i] = NAN;
    }
    for (int i = 0; i < 2 * 2; i++) {
        vqf->state.motionBiasEstBiasLpState[i] = NAN;
    }
#endif

    memset(vqf->state.restLastSquaredDeviations, 0, sizeof(double) * 3);
    vqf->state.restT = 0.0;
    memset(vqf->state.restLastGyrLp, 0, sizeof(double) * 3);
    for (int i = 0; i < 3 * 2; i++) {
        vqf->state.restGyrLpState[i] = NAN;
    }
    memset(vqf->state.restLastAccLp, 0, sizeof(double) * 3);
    for (int i = 0; i < 3 * 2; i++) {
        vqf->state.restAccLpState[i] = NAN;
    }

    vqf->state.magRefNorm = 0.0;
    vqf->state.magRefDip = 0.0;
    vqf->state.magUndisturbedT = 0.0;
    vqf->state.magRejectT = vqf->params.magMaxRejectionTime;
    vqf->state.magCandidateNorm = -1.0;
    vqf->state.magCandidateDip = 0.0;
    vqf->state.magCandidateT = 0.0;
    memset(vqf->state.magNormDip, 0, sizeof(double) * 2);
    for (int i = 0; i < 2 * 2; i++) {
        vqf->state.magNormDipLpState[i] = NAN;
    }
}


void VQF_Setup(VQF_t* vqf)
{
    assert(vqf->coeffs.gyrTs > 0);
    assert(vqf->coeffs.accTs > 0);
    assert(vqf->coeffs.magTs > 0);

    VQF_FilterCoeffs(vqf->params.tauAcc, vqf->coeffs.accTs, vqf->coeffs.accLpB, vqf->coeffs.accLpA);

    vqf->coeffs.kMag = VQF_GainFromTau(vqf->params.tauMag, vqf->coeffs.magTs);

    vqf->coeffs.biasP0 = VQF_Square(vqf->params.biasSigmaInit * 100.0);
    vqf->coeffs.biasV = VQF_Square(0.1 * 100.0) * vqf->coeffs.accTs / vqf->params.biasForgettingTime;

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    VQF_Real_t pMotion = VQF_Square(vqf->params.biasSigmaMotion * 100.0);
    vqf->coeffs.biasMotionW = VQF_Square(pMotion) / vqf->coeffs.biasV + pMotion;
    vqf->coeffs.biasVerticalW = vqf->coeffs.biasMotionW / ((vqf->params.biasVerticalForgettingFactor > 1e-10) ? vqf->params.biasVerticalForgettingFactor : 1e-10);
#endif

    VQF_Real_t pRest = VQF_Square(vqf->params.biasSigmaRest * 100.0);
    vqf->coeffs.biasRestW = VQF_Square(pRest) / vqf->coeffs.biasV + pRest;

    VQF_FilterCoeffs(vqf->params.restFilterTau, vqf->coeffs.gyrTs, vqf->coeffs.restGyrLpB, vqf->coeffs.restGyrLpA);
    VQF_FilterCoeffs(vqf->params.restFilterTau, vqf->coeffs.accTs, vqf->coeffs.restAccLpB, vqf->coeffs.restAccLpA);

    vqf->coeffs.kMagRef = VQF_GainFromTau(vqf->params.magRefTau, vqf->coeffs.magTs);
    if (vqf->params.magCurrentTau > 0) {
        VQF_FilterCoeffs(vqf->params.magCurrentTau, vqf->coeffs.magTs, vqf->coeffs.magNormDipLpB, vqf->coeffs.magNormDipLpA);
    }
    else {
        for (int i = 0; i < 3; i++) {
            vqf->coeffs.magNormDipLpB[i] = NaN;
        }
        for (int i = 0; i < 2; i++) {
            vqf->coeffs.magNormDipLpA[i] = NaN;
        }
    }

    VQF_ResetState(vqf);
}


/*
*VQF structure initialization with three parameters
*/
void VQF_Init(VQF_t* vqf, VQF_Real_t gyrTs, VQF_Real_t accTs, VQF_Real_t magTs)
{
    VQF_ParamsInit(&vqf->params);
    vqf->coeffs.gyrTs = gyrTs;
    vqf->coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    vqf->coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    VQF_Setup(vqf);
}

/*
*VQF structure initialization with VQFParams and three parameters
*/
void VQF_InitWithParams(VQF_t* vqf, const VQF_Params_t* params, VQF_Real_t gyrTs, VQF_Real_t accTs, VQF_Real_t magTs)
{
    vqf->params = *params;
    vqf->coeffs.gyrTs = gyrTs;
    vqf->coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    vqf->coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    VQF_Setup(vqf);
}


void VQF_FilterInitialState(VQF_Real_t x0, const double b[3], const double a[2], double out[])
{
    // initial state for steady state (equivalent to scipy.signal.lfilter_zi, obtained by setting y=x=x0 in the filter
    // update equation)
    out[0] = x0 * (1 - b[0]);
    out[1] = x0 * (b[2] - a[1]);
}


VQF_Real_t VQF_FilterStep(VQF_Real_t x, const double b[3], const double a[2], double state[2])
{
    // difference equations based on scipy.signal.lfilter documentation
    // assumes that a0 == 1.0
    double y = b[0] * x + state[0];
    state[0] = b[1] * x - a[0] * y + state[1];
    state[1] = b[2] * x - a[1] * y;
    return y;
}


void VQF_FilterVec(const VQF_Real_t x[], size_t N, VQF_Real_t tau, VQF_Real_t Ts, const double b[3], const double a[2], double state[], VQF_Real_t out[])
{
    assert(N >= 2);

    // to avoid depending on a single sample, average the first samples (for duration tau)
    // and then use this average to calculate the filter initial state
    if (isnan(state[0])) { // initialization phase
        if (isnan(state[1])) { // first sample
            state[1] = 0; // state[1] is used to store the sample count
            for (size_t i = 0; i < N; i++) {
                state[2 + i] = 0; // state[2+i] is used to store the sum
            }
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) {
            state[2 + i] += x[i];
            out[i] = state[2 + i] / state[1];
        }
        if (state[1] * Ts >= tau) {
            for (size_t i = 0; i < N; i++) {
                VQF_FilterInitialState(out[i], b, a, state + 2 * i);
            }
        }
        return;
    }

    for (size_t i = 0; i < N; i++) {
        out[i] = VQF_FilterStep(x[i], b, a, state + 2 * i);
    }
}


void VQF_QuatMultiply(const VQF_Real_t q1[4], const VQF_Real_t q2[4], VQF_Real_t out[4])
{
    VQF_Real_t w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    VQF_Real_t x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    VQF_Real_t y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    VQF_Real_t z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

VQF_Real_t VQF_Norm(const VQF_Real_t vec[], size_t N)
{
    VQF_Real_t s = 0;
    for (size_t i = 0; i < N; i++) {
        s += vec[i] * vec[i];
    }
    return sqrt(s);
}

void VQF_Normalize(VQF_Real_t vec[], size_t N)
{
    VQF_Real_t n = VQF_Norm(vec, N);
    if (n < EPS) {
        return;
    }
    for (size_t i = 0; i < N; i++) {
        vec[i] /= n;
    }
}


void VQF_UpdateGyr(VQF_t* vqf, const VQF_Real_t gyr[3])
{
    // rest detection
    if (vqf->params.restBiasEstEnabled || vqf->params.magDistRejectionEnabled) {
        VQF_FilterVec(gyr, 3, vqf->params.restFilterTau, vqf->coeffs.gyrTs, vqf->coeffs.restGyrLpB, vqf->coeffs.restGyrLpA, vqf->state.restGyrLpState, vqf->state.restLastGyrLp);

        vqf->state.restLastSquaredDeviations[0] = VQF_Square(gyr[0] - vqf->state.restLastGyrLp[0]) + VQF_Square(gyr[1] - vqf->state.restLastGyrLp[1]) + VQF_Square(gyr[2] - vqf->state.restLastGyrLp[2]);

        VQF_Real_t biasClip = vqf->params.biasClip * (VQF_Real_t)(M_PI / 180.0);
        if (vqf->state.restLastSquaredDeviations[0] >= VQF_Square(vqf->params.restThGyr * (M_PI / 180.0))
            || fabs(vqf->state.restLastGyrLp[0]) > biasClip || fabs(vqf->state.restLastGyrLp[1]) > biasClip
            || fabs(vqf->state.restLastGyrLp[2]) > biasClip) {
            vqf->state.restT = 0.0;
            vqf->state.restDetected = false;
        }
    }

    // remove estimated gyro bias
    VQF_Real_t gyrNoBias[3] = { gyr[0] - vqf->state.bias[0], gyr[1] - vqf->state.bias[1], gyr[2] - vqf->state.bias[2] };

    // gyroscope prediction step
    VQF_Real_t gyrNorm = VQF_Norm(gyrNoBias, 3);
    VQF_Real_t angle = gyrNorm * vqf->coeffs.gyrTs;
    if (gyrNorm > EPS) {
        VQF_Real_t c = cos(angle / 2);
        VQF_Real_t s = sin(angle / 2) / gyrNorm;
        VQF_Real_t gyrStepQuat[4] = { c, s * gyrNoBias[0], s * gyrNoBias[1], s * gyrNoBias[2] };
        VQF_QuatMultiply(vqf->state.gyrQuat, gyrStepQuat, vqf->state.gyrQuat);
        VQF_Normalize(vqf->state.gyrQuat, 4);
    }
}


void VQF_QuatRotate(const VQF_Real_t q[4], const VQF_Real_t v[3], VQF_Real_t out[3])
{
    VQF_Real_t x = (1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]) * v[0] + 2 * v[1] * (q[2] * q[1] - q[0] * q[3]) + 2 * v[2] * (q[0] * q[2] + q[3] * q[1]);
    VQF_Real_t y = 2 * v[0] * (q[0] * q[3] + q[2] * q[1]) + v[1] * (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3]) + 2 * v[2] * (q[2] * q[3] - q[1] * q[0]);
    VQF_Real_t z = 2 * v[0] * (q[3] * q[1] - q[0] * q[2]) + 2 * v[1] * (q[0] * q[1] + q[3] * q[2]) + v[2] * (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}


void VQF_Clip(VQF_Real_t vec[], size_t N, VQF_Real_t min, VQF_Real_t max)
{
    for (size_t i = 0; i < N; i++) {
        if (vec[i] < min) {
            vec[i] = min;
        }
        else if (vec[i] > max) {
            vec[i] = max;
        }
    }
}


void VQF_UpdateAcc(VQF_t* vqf, const VQF_Real_t acc[3])
{
    // ignore [0 0 0] samples
    if (acc[0] == 0.0 && acc[1] == 0.0 && acc[2] == 0.0){
        return;
    }

    // rest detection
    if (vqf->params.restBiasEstEnabled){
        VQF_FilterVec(acc, 3, vqf->params.restFilterTau, vqf->coeffs.accTs, vqf->coeffs.restAccLpB, vqf->coeffs.restAccLpA, vqf->state.restAccLpState, vqf->state.restLastAccLp);

        vqf->state.restLastSquaredDeviations[1] = VQF_Square(acc[0] - vqf->state.restLastAccLp[0]) + VQF_Square(acc[1] - vqf->state.restLastAccLp[1]) + VQF_Square(acc[2] - vqf->state.restLastAccLp[2]);

        if (vqf->state.restLastSquaredDeviations[1] >= VQF_Square(vqf->params.restThAcc)){
            vqf->state.restT = 0.0;
            vqf->state.restDetected = false;
        }
        else{
            vqf->state.restT += vqf->coeffs.accTs;
            if (vqf->state.restT >= vqf->params.restMinT) {
                vqf->state.restDetected = true;
            }
        }
    }

    VQF_Real_t accEarth[3];

    // filter acc in inertial frame
    VQF_QuatRotate(vqf->state.gyrQuat, acc, accEarth);
    VQF_FilterVec(accEarth, 3, vqf->params.tauAcc, vqf->coeffs.accTs, vqf->coeffs.accLpB, vqf->coeffs.accLpA, vqf->state.accLpState, vqf->state.lastAccLp);

    // transform to 6D earth frame and normalize
    VQF_QuatRotate(vqf->state.accQuat, vqf->state.lastAccLp, accEarth);
    VQF_Normalize(accEarth, 3);

    // inclination correction
    VQF_Real_t accCorrQuat[4];
    VQF_Real_t q_w = sqrt((accEarth[2] + 1) / 2);
    if (q_w > 1e-6) {
        accCorrQuat[0] = q_w;
        accCorrQuat[1] = 0.5 * accEarth[1] / q_w;
        accCorrQuat[2] = -0.5 * accEarth[0] / q_w;
        accCorrQuat[3] = 0;
    }
    else {
        // to avoid numeric issues when acc is close to [0 0 -1], i.e. the correction step is close (<= 0.00011°) to 180°:
        accCorrQuat[0] = 0;
        accCorrQuat[1] = 1;
        accCorrQuat[2] = 0;
        accCorrQuat[3] = 0;
    }
    VQF_QuatMultiply(accCorrQuat, vqf->state.accQuat, vqf->state.accQuat);
    VQF_Normalize(vqf->state.accQuat, 4);

    // calculate correction angular rate to facilitate debugging
    vqf->state.lastAccCorrAngularRate = acos(accEarth[2]) / vqf->coeffs.accTs;

    // bias estimation
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    if (vqf->params.motionBiasEstEnabled || vqf->params.restBiasEstEnabled) {
        VQF_Real_t biasClip = vqf->params.biasClip * VQF_Real_t(M_PI / 180.0);

        VQF_Real_t accGyrQuat[4];
        VQF_Real_t R[9];
        VQF_Real_t biasLp[2];

        // get rotation matrix corresponding to accGyrQuat
        getQuat6D(accGyrQuat);
        R[0] = 1 - 2 * VQF_Square(accGyrQuat[2]) - 2 * VQF_Square(accGyrQuat[3]); // r11
        R[1] = 2 * (accGyrQuat[2] * accGyrQuat[1] - accGyrQuat[0] * accGyrQuat[3]); // r12
        R[2] = 2 * (accGyrQuat[0] * accGyrQuat[2] + accGyrQuat[3] * accGyrQuat[1]); // r13
        R[3] = 2 * (accGyrQuat[0] * accGyrQuat[3] + accGyrQuat[2] * accGyrQuat[1]); // r21
        R[4] = 1 - 2 * VQF_Square(accGyrQuat[1]) - 2 * VQF_Square(accGyrQuat[3]); // r22
        R[5] = 2 * (accGyrQuat[2] * accGyrQuat[3] - accGyrQuat[1] * accGyrQuat[0]); // r23
        R[6] = 2 * (accGyrQuat[3] * accGyrQuat[1] - accGyrQuat[0] * accGyrQuat[2]); // r31
        R[7] = 2 * (accGyrQuat[0] * accGyrQuat[1] + accGyrQuat[3] * accGyrQuat[2]); // r32
        R[8] = 1 - 2 * VQF_Square(accGyrQuat[1]) - 2 * VQF_Square(accGyrQuat[2]); // r33

        // calculate R*b_hat (only the x and y component, as z is not needed)
        biasLp[0] = R[0] * vqf->state.bias[0] + R[1] * vqf->state.bias[1] + R[2] * vqf->state.bias[2];
        biasLp[1] = R[3] * vqf->state.bias[0] + R[4] * vqf->state.bias[1] + R[5] * vqf->state.bias[2];

        // low-pass filter R and R*b_hat
        VQF_FilterVec(R, 9, vqf->params.tauAcc, vqf->coeffs.accTs, vqf->coeffs.accLpB, vqf->coeffs.accLpA, vqf->state.motionBiasEstRLpState, R);
        VQF_FilterVec(biasLp, 2, vqf->params.tauAcc, vqf->coeffs.accTs, vqf->coeffs.accLpB, vqf->coeffs.accLpA, vqf->state.motionBiasEstBiasLpState, biasLp);

        // set measurement error and covariance for the respective Kalman filter update
        VQF_Real_t w[3];
        VQF_Real_t e[3];
        if (vqf->state.restDetected && vqf->params.restBiasEstEnabled){
            e[0] = vqf->state.restLastGyrLp[0] - vqf->state.bias[0];
            e[1] = vqf->state.restLastGyrLp[1] - vqf->state.bias[1];
            e[2] = vqf->state.restLastGyrLp[2] - vqf->state.bias[2];
            VQF_Matrix3SetToScaledIdentity(1.0, R);
            //std::fill(w, w + 3, vqf->coeffs.biasRestW);
            memset(w, vqf->coeffs.biasRestW, sizeof(VQF_Real_t) * 3);
        }
        else if (vqf->params.motionBiasEstEnabled){
            e[0] = -accEarth[1] / vqf->coeffs.accTs + biasLp[0] - R[0] * vqf->state.bias[0] - R[1] * vqf->state.bias[1] - R[2] * vqf->state.bias[2];
            e[1] = accEarth[0] / vqf->coeffs.accTs + biasLp[1] - R[3] * vqf->state.bias[0] - R[4] * vqf->state.bias[1] - R[5] * vqf->state.bias[2];
            e[2] = -R[6] * vqf->state.bias[0] - R[7] * vqf->state.bias[1] - R[8] * vqf->state.bias[2];
            w[0] = vqf->coeffs.biasMotionW;
            w[1] = vqf->coeffs.biasMotionW;
            w[2] = vqf->coeffs.biasVerticalW;
        }
        else {
            std::fill(w, w + 3, -1); // disable update
        }

        // Kalman filter update
        // step 1: P = P + V (also increase covariance if there is no measurement update!)
        if (vqf->state.biasP[0] < vqf->coeffs.biasP0){
            vqf->state.biasP[0] += vqf->coeffs.biasV;
        }
        if (vqf->state.biasP[4] < vqf->coeffs.biasP0){
            vqf->state.biasP[4] += vqf->coeffs.biasV;
        }
        if (vqf->state.biasP[8] < vqf->coeffs.biasP0){
            vqf->state.biasP[8] += vqf->coeffs.biasV;
        }
        if (w[0] >= 0){
            // clip disagreement to -2..2 °/s
            // (this also effectively limits the harm done by the first inclination correction step)
            VQF_Clip(e, 3, -biasClip, biasClip);

            // step 2: K = P R^T inv(W + R P R^T)
            VQF_Real_t K[9];
            VQF_Matrix3MultiplyTpsSecond(vqf->state.biasP, R, K); // K = P R^T
            VQF_Matrix3Multiply(R, K, K); // K = R P R^T
            K[0] += w[0];
            K[4] += w[1];
            K[8] += w[2]; // K = W + R P R^T
            VQF_Matrix3Inv(K, K); // K = inv(W + R P R^T)
            VQF_Matrix3MultiplyTpsFirst(R, K, K); // K = R^T inv(W + R P R^T)
            VQF_Matrix3Multiply(vqf->state.biasP, K, K); // K = P R^T inv(W + R P R^T)

            // step 3: bias = bias + K (y - R bias) = bias + K e
            vqf->state.bias[0] += K[0] * e[0] + K[1] * e[1] + K[2] * e[2];
            vqf->state.bias[1] += K[3] * e[0] + K[4] * e[1] + K[5] * e[2];
            vqf->state.bias[2] += K[6] * e[0] + K[7] * e[1] + K[8] * e[2];

            // step 4: P = P - K R P
            VQF_Matrix3Multiply(K, R, K); // K = K R
            VQF_Matrix3Multiply(K, vqf->state.biasP, K); // K = K R P
            for (size_t i = 0; i < 9; i++) {
                vqf->state.biasP[i] -= K[i];
            }

            // clip bias estimate to -2..2 °/s
            VQF_Clip(vqf->state.bias, 3, -biasClip, biasClip);
        }
    }
#else
    // simplified implementation of bias estimation for the special case in which only rest bias estimation is enabled
    if (vqf->params.restBiasEstEnabled){
        VQF_Real_t biasClip = vqf->params.biasClip * (VQF_Real_t)(M_PI / 180.0);
        if (vqf->state.biasP < vqf->coeffs.biasP0){
            vqf->state.biasP += vqf->coeffs.biasV;
        }
        if (vqf->state.restDetected){
            VQF_Real_t e[3];
            e[0] = vqf->state.restLastGyrLp[0] - vqf->state.bias[0];
            e[1] = vqf->state.restLastGyrLp[1] - vqf->state.bias[1];
            e[2] = vqf->state.restLastGyrLp[2] - vqf->state.bias[2];
            VQF_Clip(e, 3, -biasClip, biasClip);

            // Kalman filter update, simplified scalar version for rest update
            // (this version only uses the first entry of P as P is diagonal and all diagonal elements are the same)
            // step 1: P = P + V (done above!)
            // step 2: K = P R^T inv(W + R P R^T)
            VQF_Real_t k = vqf->state.biasP / (vqf->coeffs.biasRestW + vqf->state.biasP);
            // step 3: bias = bias + K (y - R bias) = bias + K e
            vqf->state.bias[0] += k * e[0];
            vqf->state.bias[1] += k * e[1];
            vqf->state.bias[2] += k * e[2];
            // step 4: P = P - K R P
            vqf->state.biasP -= k * vqf->state.biasP;
            VQF_Clip(vqf->state.bias, 3, -biasClip, biasClip);
        }
    }
#endif
}


void VQF_GetQuat6D(const VQF_t* vqf, VQF_Real_t out[4])
{
    VQF_QuatMultiply(vqf->state.accQuat, vqf->state.gyrQuat, out);
}


void VQF_UpdateMag(VQF_t* vqf, const VQF_Real_t mag[3])
{
    // ignore [0 0 0] samples
    if (mag[0] == (VQF_Real_t)0.0 && mag[1] == (VQF_Real_t)0.0 && mag[2] == (VQF_Real_t)0.0) {
        return;
    }

    VQF_Real_t magEarth[3];

    // bring magnetometer measurement into 6D earth frame
    VQF_Real_t accGyrQuat[4];
    VQF_GetQuat6D(vqf, accGyrQuat); // Assuming vqf has the required data for this function.
    VQF_QuatRotate(accGyrQuat, mag, magEarth);

    if (vqf->params.magDistRejectionEnabled) {
        vqf->state.magNormDip[0] = VQF_Norm(magEarth, 3);
        vqf->state.magNormDip[1] = -asin(magEarth[2] / vqf->state.magNormDip[0]);

        if (vqf->params.magCurrentTau > 0) {
            VQF_FilterVec(vqf->state.magNormDip, 2, vqf->params.magCurrentTau, vqf->coeffs.magTs, vqf->coeffs.magNormDipLpB,
                vqf->coeffs.magNormDipLpA, vqf->state.magNormDipLpState, vqf->state.magNormDip);
        }

        // magnetic disturbance detection
        if (fabs(vqf->state.magNormDip[0] - vqf->state.magRefNorm) < vqf->params.magNormTh * vqf->state.magRefNorm
            && fabs(vqf->state.magNormDip[1] - vqf->state.magRefDip) < vqf->params.magDipTh * (VQF_Real_t)(M_PI / 180.0)) {
            vqf->state.magUndisturbedT += vqf->coeffs.magTs;
            if (vqf->state.magUndisturbedT >= vqf->params.magMinUndisturbedTime) {
                vqf->state.magDistDetected = 0;
                vqf->state.magRefNorm += vqf->coeffs.kMagRef * (vqf->state.magNormDip[0] - vqf->state.magRefNorm);
                vqf->state.magRefDip += vqf->coeffs.kMagRef * (vqf->state.magNormDip[1] - vqf->state.magRefDip);
            }
        }
        else {
            vqf->state.magUndisturbedT = 0.0;
            vqf->state.magDistDetected = 1;
        }

        // new magnetic field acceptance
        if (fabs(vqf->state.magNormDip[0] - vqf->state.magCandidateNorm) < vqf->params.magNormTh * vqf->state.magCandidateNorm
            && fabs(vqf->state.magNormDip[1] - vqf->state.magCandidateDip) < vqf->params.magDipTh * (VQF_Real_t)(M_PI / 180.0)) {
            if (VQF_Norm(vqf->state.restLastGyrLp, 3) >= vqf->params.magNewMinGyr * M_PI / 180.0) {
                vqf->state.magCandidateT += vqf->coeffs.magTs;
            }
            vqf->state.magCandidateNorm += vqf->coeffs.kMagRef * (vqf->state.magNormDip[0] - vqf->state.magCandidateNorm);
            vqf->state.magCandidateDip += vqf->coeffs.kMagRef * (vqf->state.magNormDip[1] - vqf->state.magCandidateDip);

            if (vqf->state.magDistDetected && (vqf->state.magCandidateT >= vqf->params.magNewTime || (
                vqf->state.magRefNorm == 0.0 && vqf->state.magCandidateT >= vqf->params.magNewFirstTime))) {
                vqf->state.magRefNorm = vqf->state.magCandidateNorm;
                vqf->state.magRefDip = vqf->state.magCandidateDip;
                vqf->state.magDistDetected = 0;
                vqf->state.magUndisturbedT = vqf->params.magMinUndisturbedTime;
            }
        }
        else {
            vqf->state.magCandidateT = 0.0;
            vqf->state.magCandidateNorm = vqf->state.magNormDip[0];
            vqf->state.magCandidateDip = vqf->state.magNormDip[1];
        }
    }

    // calculate disagreement angle based on current magnetometer measurement
    vqf->state.lastMagDisAngle = atan2(magEarth[0], magEarth[1]) - vqf->state.delta;

    // make sure the disagreement angle is in the range [-pi, pi]
    if (vqf->state.lastMagDisAngle > (VQF_Real_t)M_PI) {
        vqf->state.lastMagDisAngle -= (VQF_Real_t)(2 * M_PI);
    }
    else if (vqf->state.lastMagDisAngle < (VQF_Real_t)(-M_PI)) {
        vqf->state.lastMagDisAngle += (VQF_Real_t)(2 * M_PI);
    }

    VQF_Real_t k = vqf->coeffs.kMag;

    if (vqf->params.magDistRejectionEnabled) {
        // magnetic disturbance rejection
        if (vqf->state.magDistDetected) {
            if (vqf->state.magRejectT <= vqf->params.magMaxRejectionTime) {
                vqf->state.magRejectT += vqf->coeffs.magTs;
                k = 0;
            }
            else {
                k /= vqf->params.magRejectionFactor;
            }
        }
        else {
            vqf->state.magRejectT = fmax(vqf->state.magRejectT - vqf->params.magRejectionFactor * vqf->coeffs.magTs, (VQF_Real_t)0.0);
        }
    }

    // ensure fast initial convergence
    if (vqf->state.kMagInit != (VQF_Real_t)0.0) {
        // make sure that the gain k is at least 1/N, N=1,2,3,... in the first few samples
        if (k < vqf->state.kMagInit) {
            k = vqf->state.kMagInit;
        }

        // iterative expression to calculate 1/N
        vqf->state.kMagInit = vqf->state.kMagInit / (vqf->state.kMagInit + 1);

        // disable if t > tauMag
        if (vqf->state.kMagInit * vqf->params.tauMag < vqf->coeffs.magTs) {
            vqf->state.kMagInit = 0.0;
        }
    }

    // first-order filter step
    vqf->state.delta += k * vqf->state.lastMagDisAngle;
    // calculate correction angular rate to facilitate debugging
    vqf->state.lastMagCorrAngularRate = k * vqf->state.lastMagDisAngle / vqf->coeffs.magTs;

    // make sure delta is in the range [-pi, pi]
    if (vqf->state.delta > (VQF_Real_t)M_PI) {
        vqf->state.delta -= (VQF_Real_t)(2 * M_PI);
    }
    else if (vqf->state.delta < (VQF_Real_t)(-M_PI)) {
        vqf->state.delta += (VQF_Real_t)(2 * M_PI);
    }
}


void VQF_Update(VQF_t* vqf, const VQF_Real_t gyr[3], const VQF_Real_t acc[3])
{
    VQF_UpdateGyr(vqf, gyr);
    VQF_UpdateAcc(vqf, acc);
}

void VQF_UpdateWithMag(VQF_t* vqf, const VQF_Real_t gyr[3], const VQF_Real_t acc[3], const VQF_Real_t mag[3])
{
    VQF_UpdateGyr(vqf, gyr);
    VQF_UpdateAcc(vqf, acc);
    VQF_UpdateMag(vqf, mag);
}


void VQF_GetQuat3D(const VQF_t* vqf, VQF_Real_t out[4])
{
    memcpy(out, vqf->state.gyrQuat, 4 * sizeof(VQF_Real_t));
}


void VQF_QuatApplyDelta(const VQF_Real_t q[], VQF_Real_t delta, VQF_Real_t out[])
{
    // out = quatMultiply([cos(delta/2), 0, 0, sin(delta/2)], q)
	VQF_Real_t c = cos(delta / 2);
	VQF_Real_t s = sin(delta / 2);
	VQF_Real_t w = c * q[0] - s * q[3];
	VQF_Real_t x = c * q[1] - s * q[2];
	VQF_Real_t y = c * q[2] + s * q[1];
	VQF_Real_t z = c * q[3] + s * q[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}


void VQF_GetQuat9D(const VQF_t* vqf, VQF_Real_t out[4])
{
    VQF_QuatMultiply(vqf->state.accQuat, vqf->state.gyrQuat, out);
    VQF_QuatApplyDelta(out, vqf->state.delta, out);
}


VQF_Real_t VQF_GetDelta(const VQF_t* vqf)
{
    return vqf->state.delta;
}


VQF_Real_t VQF_GetBiasEstimate(const VQF_t* vqf, VQF_Real_t out[3])
{
    if (out) {
        memcpy(out, vqf->state.bias, 3 * sizeof(VQF_Real_t));
    }
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    VQF_Real_t sum1 = fabs(vqf->state.biasP[0]) + fabs(vqf->state.biasP[1]) + fabs(vqf->state.biasP[2]);
    VQF_Real_t sum2 = fabs(vqf->state.biasP[3]) + fabs(vqf->state.biasP[4]) + fabs(vqf->state.biasP[5]);
    VQF_Real_t sum3 = fabs(vqf->state.biasP[6]) + fabs(vqf->state.biasP[7]) + fabs(vqf->state.biasP[8]);
    VQF_Real_t P = fmin(fmax(fmax(sum1, sum2), sum3), vqf->coeffs.biasP0);
#else
    VQF_Real_t P = vqf->state.biasP;
#endif
    return sqrt(P) * (VQF_Real_t)(M_PI / 100.0 / 180.0);
}


void VQF_SetBiasEstimate(VQF_t* vqf, VQF_Real_t bias[3], VQF_Real_t sigma)
{
    memcpy(vqf->state.bias, bias, 3 * sizeof(VQF_Real_t));
    if (sigma > 0) {
    	VQF_Real_t P = VQF_Square(sigma * (VQF_Real_t)(180.0 * 100.0 / M_PI));
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        VQF_Matrix3SetToScaledIdentity(P, vqf->state.biasP);
#else
        vqf->state.biasP = P;
#endif
    }
}


int VQF_GetRestDetected(const VQF_t* vqf) // int used as a replacement for bool
{
    return vqf->state.restDetected;
}


int VQF_GetMagDistDetected(const VQF_t* vqf) // int used as a replacement for bool
{
    return vqf->state.magDistDetected;
}


void VQF_GetRelativeRestDeviations(const VQF_t* vqf, VQF_Real_t out[2])
{
    out[0] = sqrt(vqf->state.restLastSquaredDeviations[0]) / (vqf->params.restThGyr * (VQF_Real_t)(M_PI / 180.0));
    out[1] = sqrt(vqf->state.restLastSquaredDeviations[1]) / vqf->params.restThAcc;
}

VQF_Real_t VQF_GetMagRefNorm(const VQF_t* vqf)
{
    return vqf->state.magRefNorm;
}

VQF_Real_t VQF_GetMagRefDip(const VQF_t* vqf)
{
    return vqf->state.magRefDip;
}

void VQF_SetMagRef(VQF_t* vqf, VQF_Real_t norm, VQF_Real_t dip)
{
    vqf->state.magRefNorm = norm;
    vqf->state.magRefDip = dip;
}


#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void VQF_SetMotionBiasEstEnabled(VQF_t* vqf, int enabled)
{
    if (vqf->params.motionBiasEstEnabled == enabled) {
        return;
    }
    vqf->params.motionBiasEstEnabled = enabled;
    for (int i = 0; i < 9 * 2; i++) {
        vqf->state.motionBiasEstRLpState[i] = NAN;
    }
    for (int i = 0; i < 2 * 2; i++) {
        vqf->state.motionBiasEstBiasLpState[i] = NAN;
    }
}
#endif


void VQF_SetRestBiasEstEnabled(VQF_t* vqf, int enabled)
{
    if (vqf->params.restBiasEstEnabled == enabled) {
        return;
    }
    vqf->params.restBiasEstEnabled = enabled;
    vqf->state.restDetected = 0;
    memset(vqf->state.restLastSquaredDeviations, 0, sizeof(double) * 3);
    vqf->state.restT = 0.0;
    memset(vqf->state.restLastGyrLp, 0, sizeof(double) * 3);
    for (int i = 0; i < 3 * 2; i++) {
        vqf->state.restGyrLpState[i] = NAN;
    }
    memset(vqf->state.restLastAccLp, 0, sizeof(double) * 3);
    for (int i = 0; i < 3 * 2; i++) {
        vqf->state.restAccLpState[i] = NAN;
    }
}


void VQF_SetMagDistRejectionEnabled(VQF_t* vqf, int enabled)
{
    if (vqf->params.magDistRejectionEnabled == enabled) {
        return;
    }
    vqf->params.magDistRejectionEnabled = enabled;
    vqf->state.magDistDetected = 1;
    vqf->state.magRefNorm = 0.0;
    vqf->state.magRefDip = 0.0;
    vqf->state.magUndisturbedT = 0.0;
    vqf->state.magRejectT = vqf->params.magMaxRejectionTime;
    vqf->state.magCandidateNorm = -1.0;
    vqf->state.magCandidateDip = 0.0;
    vqf->state.magCandidateT = 0.0;
    for (int i = 0; i < 2 * 2; i++) {
        vqf->state.magNormDipLpState[i] = NAN;
    }
}


void VQF_FilterAdaptStateForCoeffChange(VQF_Real_t last_y[], size_t N, const double b_old[], const double a_old[], const double b_new[], const double a_new[], double state[])
{
    if (isnan(state[0])) {
        return;
    }
    for (size_t i = 0; i < N; i++) {
        state[0 + 2 * i] = state[0 + 2 * i] + (b_old[0] - b_new[0]) * last_y[i];
        state[1 + 2 * i] = state[1 + 2 * i] + (b_old[1] - b_new[1] - a_old[0] + a_new[0]) * last_y[i];
    }
}


void VQF_SetTauAcc(VQF_t* vqf, VQF_Real_t tauAcc)
{
    if (vqf->params.tauAcc == tauAcc) {
        return;
    }
    vqf->params.tauAcc = tauAcc;
    double newB[3];
    double newA[3];

    VQF_FilterCoeffs(vqf->params.tauAcc, vqf->coeffs.accTs, newB, newA);
    VQF_FilterAdaptStateForCoeffChange(vqf->state.lastAccLp, 3, vqf->coeffs.accLpB, vqf->coeffs.accLpA, newB, newA, vqf->state.accLpState);

}


void VQF_setTauMag(VQF_t* vqf, VQF_Real_t tauMag)
{
    vqf->params.tauMag = tauMag;
    vqf->coeffs.kMag = VQF_GainFromTau(vqf->params.tauMag, vqf->coeffs.magTs);
}


void VQF_SetRestDetectionThresholds(VQF_t* vqf, VQF_Real_t thGyr, VQF_Real_t thAcc)
{
  vqf->params.restThGyr = thGyr;
  vqf->params.restThAcc = thAcc;
}


VQF_Params_t VQF_GetParams(const VQF_t* vqf)
{
    return vqf->params;
}


VQF_Coeffs_t VQF_GetCoeffs(const VQF_t* vqf)
{
    return vqf->coeffs;
}

VQF_State_t VQF_GetState(const VQF_t* vqf)
{
    return vqf->state;
}

void VQF_SetState(VQF_t* vqf, const VQF_State_t* state)
{
    vqf->state = *state;
}


void VQF_QuatConj(const VQF_Real_t q[4], VQF_Real_t out[4])
{
	VQF_Real_t w = q[0];
	VQF_Real_t x = -q[1];
	VQF_Real_t y = -q[2];
	VQF_Real_t z = -q[3];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}





