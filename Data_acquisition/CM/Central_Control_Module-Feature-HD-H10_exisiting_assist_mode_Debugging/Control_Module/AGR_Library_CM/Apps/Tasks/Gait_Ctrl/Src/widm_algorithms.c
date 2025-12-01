/*
 * widm_algorithm.c
 *
 *  Created on: Oct 11, 2023
 *      Author: INVINCIBLENESS
 */

#include "widm_algorithms.h"

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

/* ------------------- SAVE PREVIOUS VALUE ------------------- */
void WIDM_UpdateBuffer(WIDM_SensorData_t* sensorData, WIDM_AngleData_t* angleData, WIDM_GaitData_t* gaitData, WIDM_NormData_t* normData)
{
	sensorData->accX[1] = sensorData->accX[0];
	sensorData->accY[1] = sensorData->accY[0];
	sensorData->gyrZ[1] = sensorData->gyrZ[0];

	angleData->velRaw[1] = angleData->velRaw[0];
	angleData->degTvcf[1] = angleData->degTvcf[0];

	angleData->degLPF1st[1] = angleData->degLPF1st[0];
	angleData->degLPF2nd[1] = angleData->degLPF2nd[0];
	angleData->velLPF1st[1] = angleData->velLPF1st[0];
	angleData->velLPF2nd[1] = angleData->velLPF2nd[0];

	normData->degOriLPF[1] = normData->degOriLPF[0];
	normData->velOriLPF[1] = normData->velOriLPF[0];
	normData->ampDegLPF[1] = normData->ampDegLPF[0];
	normData->ampVelLPF[1] = normData->ampVelLPF[0];

	gaitData->gaitPhasePre = gaitData->gaitPhase;
}

/* ------------------- FUZZY LOGIC ------------------- */
float WIDM_SquareRootSum(float x, float y)
{
	return sqrt(pow(x, 2) + pow(y, 2));
}

float WIDM_AbsoluteValue(float value)
{
	return fabs(value);
}

float WIDM_Derivative(float currentVal, float previousVal)
{
	return (currentVal - previousVal) / WIDM_CONTROL_PERIOD;
}

void WIDM_CalculateMu(float fuzzyVar, float fuzzyInput, float* mu)
{
	/* fuzzyVar is measurement value (initially set value in Set_Init_Parameters) */
	float xoi = 3 * fuzzyVar;									// Threshold Value (maybe middle value)
	float si = log(3) / fuzzyVar;								// Sensor Sensitivity (natural logarithm)
	float xbar = 0.5 * (1 + tanh(si * (fuzzyInput - xoi)));		// Fuzzy Logic Relational Expressions
	*mu *= (1 - xbar);											// Update mu for TVCF cutoff frequency(wc)
}

/*
*Generate Fuzzy Logic Input (Acc, Jerk, Angular Velocity, Angular Accerleration)
*/
void WIDM_CalculateFuzzyInput(WIDM_SensorData_t* sensorData, WIDM_FuzzyData_t* fuzzyData)
{
	float jerkX = WIDM_Derivative(sensorData->accX[0], sensorData->accX[1]);
	float jerkY = WIDM_Derivative(sensorData->accY[0], sensorData->accY[1]);
	float wdotZ = WIDM_Derivative(sensorData->gyrZ[0], sensorData->gyrZ[1]);

	// absolute ACC
	fuzzyData->fuzzyInput[0] = WIDM_SquareRootSum(sensorData->accX[0], sensorData->accY[0]);
	// absolute Jerk
	fuzzyData->fuzzyInput[1] = WIDM_SquareRootSum(jerkX, jerkY);
	// absolute Gyr(Angular Velocity)
	fuzzyData->fuzzyInput[2] = WIDM_AbsoluteValue(sensorData->gyrZ[0]);
	// absolute Wdot(Angular Acceleration)
	fuzzyData->fuzzyInput[3] = WIDM_AbsoluteValue(wdotZ);
}

/*
*Calculate Wc(CutOff Frequency)
*/
float WIDM_CalculateFuzzyWc(WIDM_FuzzyData_t* fuzzyData)
{
	fuzzyData->wc = 0;
	float mu = 1;

	/* Perform calculations for each fuzzy input (Acc, Jerk, Angular Velocity, Angular Accerleration) */
	for (int i = 0; i < 4; i++) {
		WIDM_CalculateMu(fuzzyData->var[i], fuzzyData->fuzzyInput[i], &mu);
	}

	fuzzyData->wc = mu * (fuzzyData->wh) + (1 - mu) * (fuzzyData->wl);

	return fuzzyData->wc;
}


/* ------------------- FILTERS ------------------- */
/*
*Low pass filtering
*/
float WIDM_LPF(float currAngle, float filteredAnglePrev, float cutoffFreq, float samplingPeriod)
{
	float filteredAngle = (cutoffFreq * samplingPeriod * currAngle + filteredAnglePrev) / (cutoffFreq * samplingPeriod + 1);
	return filteredAngle;
}

/*
*High pass filtering
*/
float WIDM_HPF(float currAngle, float filteredAnglePrev, float cutoffFreq, float samplingPeriod)
{
	float filteredAngle = (currAngle * samplingPeriod + filteredAnglePrev) / (cutoffFreq * samplingPeriod + 1);
	return filteredAngle;
}

/*
*Bandpass filtering : 0.3Hz ~ 1Hz
*/
double WIDM_BPF_walking_Prof(double r)
{
	double y_1;
	static uint8_t firstRun_1 = 0;
	static double y1_1, y2_1, y3_1, r1_1, r2_1, r3_1;
	if (firstRun_1 == 0){
		y3_1 = r;
		y2_1 = r;
		y1_1 = r;
		r3_1 = 0;
		r2_1 = 0;
		r1_1 = 0;
		firstRun_1 = 1;
	}

	// First code //
	y_1 = (2.985589845070495*y1_1 - 2.971242511965642*y2_1 + 0.985652593015590*y3_1 + 0.0001 * (0.195971613578490*r1_1 - 0.195971613578490*r3_1));

	y3_1 = y2_1;
    y2_1 = y1_1;
    y1_1 = y_1;
    r3_1 = r2_1;
    r2_1 = r1_1;
    r1_1 = r;

    return y_1;
}


/*
*Bandpass filtering : Peaking cutoff Frequency
*/
double WIDM_BPF_Peak_Prof_1(double r, double w)
{
	double y_2;
	static uint8_t firstRun_2 = 0;
	static double y1_2, y2_2, y3_2, r1_2, r2_2, r3_2;

	double T_2 = 0.001;
	double a_2 = 1.02;
	double b_2 = 8;

	if (firstRun_2 == 0){
		y3_2 = r;
		y2_2 = r;
		y1_2 = r;
		r3_2 = 0;
		r2_2 = 0;
		r1_2 = 0;
		firstRun_2 = 1;
	}


	y_2 = ( - ( 2*w*w - 24/(T_2*T_2) - (4*a_2*w)/T_2 - (4*b_2*w)/T_2 + 3*T_2*b_2*w*w*w + 2*a_2*b_2*w*w )*y1_2 - ( 24/(T_2*T_2) - 2*w*w - (4*a_2*w)/T_2 - (4*b_2*w)/T_2 + 3*T_2*b_2*w*w*w - 2*a_2*b_2*w*w )*y2_2 - ( T_2*b_2*w*w*w - 2*a_2*b_2*w*w + (4*a_2*w)/T_2 + (4*b_2*w)/T_2 - 8/(T_2*T_2) - 2*w*w )*y3_2 + 2*a_2*b_2*w*w * ( r + r1_2 - r2_2 - r3_2 ) ) / ( 2*w*w + 8/(T_2*T_2) + (4*a_2*w)/T_2 + (4*b_2*w)/T_2 + T_2*b_2*w*w*w + 2*a_2*b_2*w*w );


	y3_2 = y2_2;
    y2_2 = y1_2;
    y1_2 = y_2;
    r3_2 = r2_2;
    r2_2 = r1_2;
    r1_2 = r;

    return y_2;
}

double WIDM_BPF_Peak_Prof_1_RH(double r, double w)
{
	double y_2_RH;
	static uint8_t firstRun_2_RH = 0;
	static double y1_2_RH, y2_2_RH, y3_2_RH, r1_2_RH, r2_2_RH, r3_2_RH;

	double T_2_RH = 0.001;
	double a_2_RH = 1.02;
	double b_2_RH = 8;

	if (firstRun_2_RH == 0){
		y3_2_RH = r;
		y2_2_RH = r;
		y1_2_RH = r;
		r3_2_RH = 0;
		r2_2_RH = 0;
		r1_2_RH = 0;
		firstRun_2_RH = 1;
	}


	y_2_RH = ( - ( 2*w*w - 24/(T_2_RH*T_2_RH) - (4*a_2_RH*w)/T_2_RH - (4*b_2_RH*w)/T_2_RH + 3*T_2_RH*b_2_RH*w*w*w + 2*a_2_RH*b_2_RH*w*w )*y1_2_RH - ( 24/(T_2_RH*T_2_RH) - 2*w*w - (4*a_2_RH*w)/T_2_RH - (4*b_2_RH*w)/T_2_RH + 3*T_2_RH*b_2_RH*w*w*w - 2*a_2_RH*b_2_RH*w*w )*y2_2_RH - ( T_2_RH*b_2_RH*w*w*w - 2*a_2_RH*b_2_RH*w*w + (4*a_2_RH*w)/T_2_RH + (4*b_2_RH*w)/T_2_RH - 8/(T_2_RH*T_2_RH) - 2*w*w )*y3_2_RH + 2*a_2_RH*b_2_RH*w*w * ( r + r1_2_RH - r2_2_RH - r3_2_RH ) ) / ( 2*w*w + 8/(T_2_RH*T_2_RH) + (4*a_2_RH*w)/T_2_RH + (4*b_2_RH*w)/T_2_RH + T_2_RH*b_2_RH*w*w*w + 2*a_2_RH*b_2_RH*w*w );


	y3_2_RH = y2_2_RH;
    y2_2_RH = y1_2_RH;
    y1_2_RH = y_2_RH;
    r3_2_RH = r2_2_RH;
    r2_2_RH = r1_2_RH;
    r1_2_RH = r;

    return y_2_RH;
}

double WIDM_BPF_Peak_Prof_1_LH(double r, double w)
{
	double y_2_LH;
	static uint8_t firstRun_2_LH = 0;
	static double y1_2_LH, y2_2_LH, y3_2_LH, r1_2_LH, r2_2_LH, r3_2_LH;

	double T_2_LH = 0.001;
	double a_2_LH = 1.02;
	double b_2_LH = 8;

	if (firstRun_2_LH == 0){
		y3_2_LH = r;
		y2_2_LH = r;
		y1_2_LH = r;
		r3_2_LH = 0;
		r2_2_LH = 0;
		r1_2_LH = 0;
		firstRun_2_LH = 1;
	}


	y_2_LH = ( - ( 2*w*w - 24/(T_2_LH*T_2_LH) - (4*a_2_LH*w)/T_2_LH - (4*b_2_LH*w)/T_2_LH + 3*T_2_LH*b_2_LH*w*w*w + 2*a_2_LH*b_2_LH*w*w )*y1_2_LH - ( 24/(T_2_LH*T_2_LH) - 2*w*w - (4*a_2_LH*w)/T_2_LH - (4*b_2_LH*w)/T_2_LH + 3*T_2_LH*b_2_LH*w*w*w - 2*a_2_LH*b_2_LH*w*w )*y2_2_LH - ( T_2_LH*b_2_LH*w*w*w - 2*a_2_LH*b_2_LH*w*w + (4*a_2_LH*w)/T_2_LH + (4*b_2_LH*w)/T_2_LH - 8/(T_2_LH*T_2_LH) - 2*w*w )*y3_2_LH + 2*a_2_LH*b_2_LH*w*w * ( r + r1_2_LH - r2_2_LH - r3_2_LH ) ) / ( 2*w*w + 8/(T_2_LH*T_2_LH) + (4*a_2_LH*w)/T_2_LH + (4*b_2_LH*w)/T_2_LH + T_2_LH*b_2_LH*w*w*w + 2*a_2_LH*b_2_LH*w*w );


	y3_2_LH = y2_2_LH;
    y2_2_LH = y1_2_LH;
    y1_2_LH = y_2_LH;
    r3_2_LH = r2_2_LH;
    r2_2_LH = r1_2_LH;
    r1_2_LH = r;

    return y_2_LH;
}

/*
*Bandpass filtering : Peaking cutoff Frequency
*/
double WIDM_BPF_Peak_Prof_2(double r, double w)
{
	double y_3;
	static uint8_t firstRun_3 = 0;
	static double y1_3, y2_3, y3_3, r1_3, r2_3, r3_3;

	double T_3 = 0.001;
	double a_3 = 0.62;
	double b_3 = 6;

	if (firstRun_3 == 0){
		y3_3 = r;
		y2_3 = r;
		y1_3 = r;
		r3_3 = 0;
		r2_3 = 0;
		r1_3 = 0;
		firstRun_3 = 1;
	}

	y_3 = ( - ( 2*w*w - 24/(T_3*T_3) - (4*a_3*w)/T_3 - (4*b_3*w)/T_3 + 3*T_3*b_3*w*w*w + 2*a_3*b_3*w*w )*y1_3 - ( 24/(T_3*T_3) - 2*w*w - (4*a_3*w)/T_3 - (4*b_3*w)/T_3 + 3*T_3*b_3*w*w*w - 2*a_3*b_3*w*w )*y2_3 - ( T_3*b_3*w*w*w - 2*a_3*b_3*w*w + (4*a_3*w)/T_3 + (4*b_3*w)/T_3 - 8/(T_3*T_3) - 2*w*w )*y3_3 + 2*a_3*b_3*w*w * ( r + r1_3 - r2_3 - r3_3 ) ) / ( 2*w*w + 8/(T_3*T_3) + (4*a_3*w)/T_3 + (4*b_3*w)/T_3 + T_3*b_3*w*w*w + 2*a_3*b_3*w*w );

	y3_3 = y2_3;
    y2_3 = y1_3;
    y1_3 = y_3;
    r3_3 = r2_3;
    r2_3 = r1_3;
    r1_3 = r;

    return y_3;
}

double WIDM_BPF_Peak_Prof_2_RH(double r, double w)
{
	double y_3_RH;
	static uint8_t firstRun_3_RH = 0;
	static double y1_3_RH, y2_3_RH, y3_3_RH, r1_3_RH, r2_3_RH, r3_3_RH;

	double T_3_RH = 0.001;
	double a_3_RH = 0.62;
	double b_3_RH = 6;

	if (firstRun_3_RH == 0){
		y3_3_RH = r;
		y2_3_RH = r;
		y1_3_RH = r;
		r3_3_RH = 0;
		r2_3_RH = 0;
		r1_3_RH = 0;
		firstRun_3_RH = 1;
	}

	y_3_RH = ( - ( 2*w*w - 24/(T_3_RH*T_3_RH) - (4*a_3_RH*w)/T_3_RH - (4*b_3_RH*w)/T_3_RH + 3*T_3_RH*b_3_RH*w*w*w + 2*a_3_RH*b_3_RH*w*w )*y1_3_RH - ( 24/(T_3_RH*T_3_RH) - 2*w*w - (4*a_3_RH*w)/T_3_RH - (4*b_3_RH*w)/T_3_RH + 3*T_3_RH*b_3_RH*w*w*w - 2*a_3_RH*b_3_RH*w*w )*y2_3_RH - ( T_3_RH*b_3_RH*w*w*w - 2*a_3_RH*b_3_RH*w*w + (4*a_3_RH*w)/T_3_RH + (4*b_3_RH*w)/T_3_RH - 8/(T_3_RH*T_3_RH) - 2*w*w )*y3_3_RH + 2*a_3_RH*b_3_RH*w*w * ( r + r1_3_RH - r2_3_RH - r3_3_RH ) ) / ( 2*w*w + 8/(T_3_RH*T_3_RH) + (4*a_3_RH*w)/T_3_RH + (4*b_3_RH*w)/T_3_RH + T_3_RH*b_3_RH*w*w*w + 2*a_3_RH*b_3_RH*w*w );

	y3_3_RH = y2_3_RH;
    y2_3_RH = y1_3_RH;
    y1_3_RH = y_3_RH;
    r3_3_RH = r2_3_RH;
    r2_3_RH = r1_3_RH;
    r1_3_RH = r;

    return y_3_RH;
}

double WIDM_BPF_Peak_Prof_2_LH(double r, double w)
{
	double y_3_LH;
	static uint8_t firstRun_3_LH = 0;
	static double y1_3_LH, y2_3_LH, y3_3_LH, r1_3_LH, r2_3_LH, r3_3_LH;

	double T_3_LH = 0.001;
//	double a_3_LH = 0.62;
//	double b_3_LH = 6;
	double a_3_LH = 1.02;
	double b_3_LH = 6;

	if (firstRun_3_LH == 0){
		y3_3_LH = r;
		y2_3_LH = r;
		y1_3_LH = r;
		r3_3_LH = 0;
		r2_3_LH = 0;
		r1_3_LH = 0;
		firstRun_3_LH = 1;
	}

	y_3_LH = ( - ( 2*w*w - 24/(T_3_LH*T_3_LH) - (4*a_3_LH*w)/T_3_LH - (4*b_3_LH*w)/T_3_LH + 3*T_3_LH*b_3_LH*w*w*w + 2*a_3_LH*b_3_LH*w*w )*y1_3_LH - ( 24/(T_3_LH*T_3_LH) - 2*w*w - (4*a_3_LH*w)/T_3_LH - (4*b_3_LH*w)/T_3_LH + 3*T_3_LH*b_3_LH*w*w*w - 2*a_3_LH*b_3_LH*w*w )*y2_3_LH - ( T_3_LH*b_3_LH*w*w*w - 2*a_3_LH*b_3_LH*w*w + (4*a_3_LH*w)/T_3_LH + (4*b_3_LH*w)/T_3_LH - 8/(T_3_LH*T_3_LH) - 2*w*w )*y3_3_LH + 2*a_3_LH*b_3_LH*w*w * ( r + r1_3_LH - r2_3_LH - r3_3_LH ) ) / ( 2*w*w + 8/(T_3_LH*T_3_LH) + (4*a_3_LH*w)/T_3_LH + (4*b_3_LH*w)/T_3_LH + T_3_LH*b_3_LH*w*w*w + 2*a_3_LH*b_3_LH*w*w );

	y3_3_LH = y2_3_LH;
    y2_3_LH = y1_3_LH;
    y1_3_LH = y_3_LH;
    r3_3_LH = r2_3_LH;
    r2_3_LH = r1_3_LH;
    r1_3_LH = r;

    return y_3_LH;
}

/*
*Low pass filtering
*/
double WIDM_LPF_walking_Prof(double r)
{
	double y_4;
	static uint8_t firstRun_4 = 0;
	static double y1_4;

	if (firstRun_4 == 0){
		y1_4 = r;
		firstRun_4 = 1;
	}

	// WIDM3 code //
	y_4 = 0.98*y1_4 + 0.02*r;

    y1_4 = y_4;

    return y_4;
}

double WIDM_LPF_walking_Prof_RH(double r)
{
	double y_4_RH;
	static uint8_t firstRun_4_RH = 0;
	static double y1_4_RH;

	if (firstRun_4_RH == 0){
		y1_4_RH = r;
		firstRun_4_RH = 1;
	}

	// WIDM3 code //
	y_4_RH = 0.98*y1_4_RH + 0.02*r;

    y1_4_RH = y_4_RH;

    return y_4_RH;
}

double WIDM_LPF_walking_Prof_LH(double r)
{
	double y_4_LH;
	static uint8_t firstRun_4_LH = 0;
	static double y1_4_LH;

	if (firstRun_4_LH == 0){
		y1_4_LH = r;
		firstRun_4_LH = 1;
	}

	// WIDM3 code //
	y_4_LH = 0.98*y1_4_LH + 0.02*r;

    y1_4_LH = y_4_LH;

    return y_4_LH;
}

double WIDM_LPF_1NE(double r, double w)
{
	double y_LPF_1ne;
	static uint8_t firstRun_LPF_1ne = 0;
	static double y1_LPF_1ne;
	static double r1_LPF_1ne;
	double T_1ne = 0.001;
	double K_1ne = 5;


	if (firstRun_LPF_1ne == 0){
		y1_LPF_1ne = r;
		r1_LPF_1ne = 0;
		firstRun_LPF_1ne = 1;
	}

	y_LPF_1ne = ( (2 - w*T_1ne)*y1_LPF_1ne + K_1ne*w*T_1ne*r + K_1ne*w*T_1ne*r1_LPF_1ne) / (2 + w*T_1ne);

    y1_LPF_1ne = y_LPF_1ne;
    r1_LPF_1ne = r;

    return y_LPF_1ne;
}

double WIDM_Abs_double(double value)
{
	if (value >= (double)0.0){
		return (double)value;
	}
	else{
		return (double)((-1.0)*(value));
	}
}

float WIDM_Abs_float(float value)
{
	if (value > 0){
		return value;
	}
	else{
		return (-1.0)*(value);
	}
}

double WIDM_GetMaxValue_double(double x, double y)
{
	return (x > y) ? x : y;
}

/*
*Function to apply a Time Variant Complementary Filter (TVCF) to an angle
*/
void WIDM_RunTVCF(WIDM_SensorData_t* sensorData, WIDM_AngleData_t* angleData, float cutoffFreq, float samplingPeriod, WIDM_AttachCase_t attachCase)
{
	/* Calculate the angle using accelerometer measurements and convert it to degrees */
    /* Thigh Angle Degree */
	float degAcc = 0.0;
	float degAccFilteredUpdate = 0.0;
	float degGyrFilteredUpdate = 0.0;
	float degTvcf = 0.0;

	degAcc = WIDM_AttachCaseSetting(sensorData, attachCase);

	/* Apply Low Pass Filter (LPF) on accelerometer angle */
	degAccFilteredUpdate = WIDM_LPF(degAcc, angleData->degAccFiltered, cutoffFreq, samplingPeriod);

	/* Apply High Pass Filter (HPF) on gyroscope measurements */
	degGyrFilteredUpdate = WIDM_HPF(sensorData->gyrZ[0], angleData->degGyrFiltered, cutoffFreq, samplingPeriod);

	/* Combine filtered accelerometer and gyroscope measurements */
	degTvcf = degAccFilteredUpdate + degGyrFilteredUpdate;

	angleData->degAccFiltered 	= degAccFilteredUpdate;
	angleData->degGyrFiltered 	= degGyrFilteredUpdate;
	angleData->degTvcfFiltered 	= degTvcf;
}


/* ------------------- GAIT FUNCTION ------------------- */
/*
*Get Max or Min Value between two variables for Normalization
*/
float WIDM_GetMaxValue(float x, float y)
{
	return (x > y) ? x : y;
}

float WIDM_GetMinValue(float x, float y)
{
	return (x < y) ? x : y;
}

void WIDM_1stHalfGaitCycle(WIDM_NormData_t* normData, WIDM_GaitData_t* gaitData)
{
	normData->degOri = normData->sumDeg / normData->sumIter;
	normData->velOri = normData->sumVel / normData->sumIter;

	gaitData->gaitPeriod = normData->sumIter;

	normData->ampDeg = (normData->degMax - normData->degMin) / 2;
	normData->ampVel = (normData->velMax - normData->velMin) / 2;
	normData->sumIter = 0;
	normData->sumDeg = 0;
	normData->sumVel = 0;
	normData->degMax = 0;
	normData->velMax = 0;
	normData->degMin = 0;
	normData->velMin = 0;
}

void WIDM_2ndHalfGaitCycle(WIDM_NormData_t* normData, WIDM_AngleData_t* angleData)
{
	normData->sumIter++;
	normData->sumDeg += angleData->degLPF2nd[0];
	normData->sumVel += angleData->velLPF2nd[0];
	normData->degMax = WIDM_GetMaxValue(angleData->degLPF2nd[0], normData->degMax);
	normData->degMin = WIDM_GetMinValue(angleData->degLPF2nd[0], normData->degMin);
	normData->velMax = WIDM_GetMaxValue(angleData->velLPF2nd[0], normData->velMax);
	normData->velMin = WIDM_GetMinValue(angleData->velLPF2nd[0], normData->velMin);
}

/*
*Function to Prepare for Circular Normalization
*/
void WIDM_Normalization(WIDM_AngleData_t* angleData, WIDM_NormData_t* normData, WIDM_GaitData_t* gaitData)
{
	if (angleData->velLPF2nd[0] < 0 && angleData->velLPF2nd[1] > 0
		&& normData->sumIter > (gaitData->gaitPeriod)*0.5) {
		WIDM_1stHalfGaitCycle(normData, gaitData);
	}
	else{
		WIDM_2ndHalfGaitCycle(normData, angleData);
	}
}

/*
*Function to calculate the current phase of the gait (0~100%)
*/
float WIDM_GetGaitPhase(WIDM_NormData_t* normData, WIDM_GaitData_t* gaitData)
{
	/* Calculate initial phase using atan function */
	float gaitPhase = atan((-1) * (normData->velNorm) / (normData->degNorm));

	/* Adjust phase based on the value of normalized degree */
    if (normData->degNorm < 0){
        gaitPhase += WIDM_PI;
    }
    else if (normData->degNorm > 0 && normData->velNorm > 0){
        gaitPhase += 2 * WIDM_PI;
    }

	/* Convert phase from radians to custom scale */
    gaitPhase = gaitPhase * 50.0f / WIDM_PI;
//    gaitPhase -= 12.11;
    gaitPhase -= 5.6;

    /* Adjust phase if it falls outside the range 0-100 */
    if (gaitPhase < 0 && gaitPhase != -100){
        gaitPhase += 100;
    }

    /* Compare phase with the stored gait phase in gaitInfo */
    if (gaitPhase > 5 && gaitPhase < 95){
        gaitPhase = WIDM_GetMaxValue(gaitPhase, gaitData->gaitPhase);
	}

    /* If the walking state is 0 or 1, set the gait phase to -100 */	// Added for fluctuation of gaitPhase
    if (gaitData->walkingState == WIDM_STOP || gaitData->walkingState == WIDM_WALKING_START){
    	gaitPhase = -100;
    }

	return gaitPhase;	// 0 ~ 100%
}

/*
*Function to find degAcc according to the "WIDM Attach Case"
*/
float WIDM_AttachCaseSetting(WIDM_SensorData_t* sensorData, WIDM_AttachCase_t attachCase)
{
	float estimatedAngle = 0.0;

	switch (attachCase)
	{
		case (WIDM_LEFT_SAGITAL):
			estimatedAngle = atan2((sensorData->accY[0])*(-1), (sensorData->accX[0])*(-1)) * (180 / WIDM_PI);		// arctan(-y/-x) : Left Sagital case
			return estimatedAngle;

		case (WIDM_RIGHT_SAGITAL):
			estimatedAngle = atan2(sensorData->accY[0], (sensorData->accX[0])*(-1)) * (180 / WIDM_PI);				// arctan(y/-x) : Right Sagital case
			return estimatedAngle;

		case (WIDM_LEFT_SAGITAL_DEMO):
			estimatedAngle = atan2(sensorData->accX[0], (sensorData->accY[0])*(-1)) * (180 / WIDM_PI);				// arctan(x/-y) : Left Sagital DEMO ver case
			return estimatedAngle;

		case (WIDM_CM_TEST):
			estimatedAngle = atan2(sensorData->accX[0], (sensorData->accY[0])*(-1)) * (180 / WIDM_PI);				// arctan(x/-y) : FOR CM TEST
			return estimatedAngle;

		case (WIDM_LEFT_U5):
			estimatedAngle = atan2((sensorData->accY[0])*(-1), (sensorData->accX[0])*(-1)) * (180 / WIDM_PI);		// arctan(-y/-x) : FOR U5_LEFT TEST
			return estimatedAngle;

		case (WIDM_RIGHT_U5):
			estimatedAngle = atan2(sensorData->accY[0], (sensorData->accX[0])*(-1)) * (180 / WIDM_PI);				// arctan(y/-x) : FOR U5_RIGHT TEST
			return estimatedAngle;

		case (WIDM_H10_RIGHT):
			estimatedAngle = atan2((sensorData->accX[0]) * (-1), (sensorData->accY[0])*(-1)) * (180 / WIDM_PI);		// arctan(-x/-y) : FOR H10_RIGHT TEST
			return estimatedAngle;

		case (WIDM_H10_LEFT):
			estimatedAngle = atan2(sensorData->accX[0], (sensorData->accY[0])*(-1)) * (180 / WIDM_PI);				// arctan(x/-y) : FOR H10_LEFT TEST
			return estimatedAngle;

		case (WIDM_K10_LEFT):
			estimatedAngle = atan2(sensorData->accX[0], (sensorData->accY[0])*(-1)) * (180 / WIDM_PI);				// arctan(x/-y) : FOR K10_LEFT TEST
			return estimatedAngle;

		case (WIDM_K10_RIGHT):
			estimatedAngle = atan2((sensorData->accX[0]) * (-1), (sensorData->accY[0])*(-1)) * (180 / WIDM_PI);		// arctan(-x/-y) : FOR K10_RIGHT TEST
			return estimatedAngle;

		default:
			return estimatedAngle;
	}
}
