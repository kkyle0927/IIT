#include "widm_algorithms.h"

/**
 *-----------------------------------------------------------
 *      TYPE DEFINITIONS AND ENUMERATIONS AND VARIABLES
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

// static void InitBPF(WIDM_BPFState* state, double currentInput);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- Math Operations ------------------- */
float GetMaxValue(float x, float y)
{
	return (x > y) ? x : y;
}

float GetMinValue(float x, float y)
{
	return (x < y) ? x : y;
}

float SquareRootSum(float x, float y)
{
	return sqrt(pow(x, 2) + pow(y, 2));
}

float GetAbsoluteValue(float value)
{
	return fabs(value);
}

/* ------------------- Filters ------------------- */
/* Low Pass Filters */
float ApplyLPF(uint8_t* isInitialized, float* prevOutput, float currentInput, float alpha)
{
    // Initialize the filter state on the first call
    if (!(*isInitialized)) {
        *prevOutput = currentInput; // Set initial output to current input to avoid startup transients
        *isInitialized = 1;      	// Mark the filter as initialized
    }

    // Apply the low-pass filter equation with custom alpha
    float currentOutput = alpha * currentInput + (1 - alpha) * (*prevOutput);

    // Update the state for the next iteration
    *prevOutput = currentOutput;

    return currentOutput;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 예전 SBS D Forum에 사용되었던 Gait Phase 관련 필터 함수 */
/* ------------------- Gait Data Estimation Filters ------------------- */
/* BPF & LPF Code Refactoring By HD */
/* Band Pass Filters */
// /**
//  * @brief Updates the band-pass filter with a new input and calculates the new output.
//  * 
//  * This function processes a new input signal through the band-pass filter designed for
//  * the frequency range of 0.3Hz to 1Hz. It updates the filter state and calculates
//  * the output based on the filter's coefficients and the new input.
//  * 
//  * @param state Pointer to the filter state structure.
//  * @param currentInput The new input signal to be processed.
//  * @return The output signal after filtering.
//  */
// double UpdateBPF_0_3Hz_to_1Hz(WIDM_BPFState* state, double currentInput)
// {
//     // Ensure the filter is initialized before processing the input
//     if (!state->isInitialized) {
//         InitBPF(state, currentInput);
//     }

//     // Coefficients defined for a 0.3Hz ~ 1Hz band-pass filter
//     const double a[] = {2.985589845070495, -2.971242511965642, 0.985652593015590}; // Feedback coefficients for previous outputs
//     const double b[] = {0.0001 * 0.195971613578490, 0, -0.0001 * 0.195971613578490}; // Feedforward coefficients for current and previous inputs

//     // Calculate the new output using the filter's coefficients
//     double currentOutput = a[0] * state->prevOutput[0] + a[1] * state->prevOutput[1] + a[2] * state->prevOutput[2]
//                             + b[0] * currentInput - b[2] * state->prevInput[2];

//     // Update the filter state for the next iteration
//     for (int i = 2; i > 0; i--) {
//         state->prevOutput[i] = state->prevOutput[i-1]; // Shift previous outputs to the right
//         state->prevInput[i] = state->prevInput[i-1];   // Shift previous inputs to the right
//     }
//     state->prevOutput[0] = currentOutput; // Store the current output
//     state->prevInput[0] = currentInput;   // Store the current input as the most recent input

//     return currentOutput; // Return the calculated output
// }

// /**
//  * @brief Updates the band-pass filter state with a new input for peaking cutoff frequency.
//  * 
//  * This function applies the band-pass filter with a peaking cutoff frequency to the input signal.
//  * The filter coefficients are optimized for emphasizing a specific frequency peak by adjusting
//  * the filter's response curve around the peak frequency. This is achieved through specific filter
//  * coefficients calculated based on the provided parameters.
//  * 
//  * @param state Pointer to the filter state structure, maintaining previous inputs and outputs.
//  * @param currentInput The new input signal to be processed by the filter.
//  * @param peakFreq The peaking cutoff frequency around which the filter's response is emphasized.
//  * @return The filtered signal output that emphasizes the signal around the peakFreq.
//  */
// double UpdateBPF_PeakCutoff_ForDeg(WIDM_BPFState* state, double currentInput, double peakFreq)
// {
//     if (!state->isInitialized) {
//         InitBPF(state, currentInput);
//     }

//     // Time constant (T), and coefficients (a, b) specific to the peaking cutoff frequency filter
//     const double T = 0.001; // Time constant defining the filter's response time and stability
//     const double a = 1.02;  // Coefficient 'a' influencing the filter's gain and stability at peak frequency
//     const double b = 8;     // Coefficient 'b' affecting the bandwidth and shape of the peak response

//     // Calculate the filter's current output based on the peaking frequency characteristics
//     double currentOutput = (
//         - (2 * peakFreq * peakFreq - 24 / (T * T) - (4 * a * peakFreq) / T - (4 * b * peakFreq) / T + 3 * T * b * peakFreq * peakFreq * peakFreq + 2 * a * b * peakFreq * peakFreq) * state->prevOutput[0]
//         - (24 / (T * T) - 2 * peakFreq * peakFreq - (4 * a * peakFreq) / T - (4 * b * peakFreq) / T + 3 * T * b * peakFreq * peakFreq * peakFreq - 2 * a * b * peakFreq * peakFreq) * state->prevOutput[1]
//         - (T * b * peakFreq * peakFreq * peakFreq - 2 * a * b * peakFreq * peakFreq + (4 * a * peakFreq) / T + (4 * b * peakFreq) / T - 8 / (T * T) - 2 * peakFreq * peakFreq) * state->prevOutput[2]
//         + 2 * a * b * peakFreq * peakFreq * (currentInput + state->prevInput[0] - state->prevInput[1] - state->prevInput[2])
//     ) / (2 * peakFreq * peakFreq + 8 / (T * T) + (4 * a * peakFreq) / T + (4 * b * peakFreq) / T + T * b * peakFreq * peakFreq * peakFreq + 2 * a * b * peakFreq * peakFreq);


//     // Update the state for the next iteration
//     for (int i = 2; i > 0; i--) {
//         state->prevOutput[i] = state->prevOutput[i-1];
//         state->prevInput[i] = state->prevInput[i-1];
//     }
//     state->prevOutput[0] = currentOutput;
//     state->prevInput[0] = currentInput;

//     return currentOutput;
// }

// double UpdateBPF_PeakCutoff_ForVel(WIDM_BPFState* state, double currentInput, double peakFreq)
// {
// 	if (!state->isInitialized) {
//         InitBPF(state, currentInput);
//     }

//     // Coefficients specific to the second version of the peaking cutoff frequency filter
//     const double T = 0.001;   // Time constant for the filter
//     const double a = 0.62;    // Coefficient 'a' affecting the filter's response
//     const double b = 6;       // Coefficient 'b' affecting the bandwidth and shape

//     double currentOutput = (
//         - (2 * peakFreq * peakFreq - 24 / (T * T) - (4 * a * peakFreq) / T - (4 * b * peakFreq) / T + 3 * T * b * peakFreq * peakFreq * peakFreq + 2 * a * b * peakFreq * peakFreq) * state->prevOutput[0]
//         - (24 / (T * T) - 2 * peakFreq * peakFreq - (4 * a * peakFreq) / T - (4 * b * peakFreq) / T + 3 * T * b * peakFreq * peakFreq * peakFreq - 2 * a * b * peakFreq * peakFreq) * state->prevOutput[1]
//         - (T * b * peakFreq * peakFreq * peakFreq - 2 * a * b * peakFreq * peakFreq + (4 * a * peakFreq) / T + (4 * b * peakFreq) / T - 8 / (T * T) - 2 * peakFreq * peakFreq) * state->prevOutput[2]
//         + 2 * a * b * peakFreq * peakFreq * (currentInput + state->prevInput[0] - state->prevInput[1] - state->prevInput[2])
//     ) / (2 * peakFreq * peakFreq + 8 / (T * T) + (4 * a * peakFreq) / T + (4 * b * peakFreq) / T + T * b * peakFreq * peakFreq * peakFreq + 2 * a * b * peakFreq * peakFreq);

//     // Update the state for the next iteration
//     for (int i = 2; i > 0; i--) {
//         state->prevOutput[i] = state->prevOutput[i-1];
//         state->prevInput[i] = state->prevInput[i-1];
//     }
//     state->prevOutput[0] = currentOutput;
//     state->prevInput[0] = currentInput;

//     return currentOutput;
// }

// float UpdateBPF_ForPeakThighDeg(uint8_t* isInitialized, float* prevOutput, float* prevInput, float currentInput, float peakFreq)
// {
//     // Initialize the filter state on the first call
//     if (!(*isInitialized)) {
//         for (int i = 0; i < 3; i++) {
//             prevOutput[i] = currentInput; // Set initial output to current input to avoid startup transients
//             prevInput[i] = 0.0;           // Reset previous inputs to zero
//         }
//         *isInitialized = 1;               // Mark the filter as initialized
//     }

//     // Time constant (T), and coefficients (a, b) specific to the peaking cutoff frequency filter
//     const float T = 0.001; // Time constant defining the filter's response time and stability
//     const float a = 1.02;  // Coefficient 'a' influencing the filter's gain and stability at peak frequency
//     const float b = 8;     // Coefficient 'b' affecting the bandwidth and shape of the peak response

//     // Calculate the filter's current output based on the peaking frequency characteristics
//     float currentOutput = (
//         - (2 * peakFreq * peakFreq - 24 / (T * T) - (4 * a * peakFreq) / T - (4 * b * peakFreq) / T + 3 * T * b * peakFreq * peakFreq * peakFreq + 2 * a * b * peakFreq * peakFreq) * prevOutput[0]
//         - (24 / (T * T) - 2 * peakFreq * peakFreq - (4 * a * peakFreq) / T - (4 * b * peakFreq) / T + 3 * T * b * peakFreq * peakFreq * peakFreq - 2 * a * b * peakFreq * peakFreq) * prevOutput[1]
//         - (T * b * peakFreq * peakFreq * peakFreq - 2 * a * b * peakFreq * peakFreq + (4 * a * peakFreq) / T + (4 * b * peakFreq) / T - 8 / (T * T) - 2 * peakFreq * peakFreq) * prevOutput[2]
//         + 2 * a * b * peakFreq * peakFreq * (currentInput + prevInput[0] - prevInput[1] - prevInput[2])
//     ) / (2 * peakFreq * peakFreq + 8 / (T * T) + (4 * a * peakFreq) / T + (4 * b * peakFreq) / T + T * b * peakFreq * peakFreq * peakFreq + 2 * a * b * peakFreq * peakFreq);

//     // Update the state for the next iteration
//     for (int i = 2; i > 0; i--) {
//         prevOutput[i] = prevOutput[i-1];
//         prevInput[i] = prevInput[i-1];
//     }
//     prevOutput[0] = currentOutput;
//     prevInput[0] = currentInput;

//     return currentOutput;
// }

// float UpdateBPF_ForPeakThighVel(uint8_t* isInitialized, float* prevOutput, float* prevInput, float currentInput, float peakFreq)
// {
//     // Initialize the filter state on the first call
//     if (!(*isInitialized)) {
//         for (int i = 0; i < 3; i++) {
//             prevOutput[i] = currentInput; // Set initial output to current input to avoid startup transients
//             prevInput[i] = 0.0;           // Reset previous inputs to zero
//         }
//         *isInitialized = 1;               // Mark the filter as initialized
//     }

//     // Coefficients specific to the second version of the peaking cutoff frequency filter
//     const float T = 0.001;   // Time constant for the filter
//     const float a = 0.62;    // Coefficient 'a' affecting the filter's response
//     const float b = 6;       // Coefficient 'b' affecting the bandwidth and shape

//     float currentOutput = (
//         - (2 * peakFreq * peakFreq - 24 / (T * T) - (4 * a * peakFreq) / T - (4 * b * peakFreq) / T + 3 * T * b * peakFreq * peakFreq * peakFreq + 2 * a * b * peakFreq * peakFreq) * prevOutput[0]
//         - (24 / (T * T) - 2 * peakFreq * peakFreq - (4 * a * peakFreq) / T - (4 * b * peakFreq) / T + 3 * T * b * peakFreq * peakFreq * peakFreq - 2 * a * b * peakFreq * peakFreq) * prevOutput[1]
//         - (T * b * peakFreq * peakFreq * peakFreq - 2 * a * b * peakFreq * peakFreq + (4 * a * peakFreq) / T + (4 * b * peakFreq) / T - 8 / (T * T) - 2 * peakFreq * peakFreq) * prevOutput[2]
//         + 2 * a * b * peakFreq * peakFreq * (currentInput + prevInput[0] - prevInput[1] - prevInput[2])
//     ) / (2 * peakFreq * peakFreq + 8 / (T * T) + (4 * a * peakFreq) / T + (4 * b * peakFreq) / T + T * b * peakFreq * peakFreq * peakFreq + 2 * a * b * peakFreq * peakFreq);

//     // Update the state for the next iteration
//     for (int i = 2; i > 0; i--) {
//         prevOutput[i] = prevOutput[i-1];
//         prevInput[i] = prevInput[i-1];
//     }
//     prevOutput[0] = currentOutput;
//     prevInput[0] = currentInput;

//     return currentOutput;
// }

// /*
// *Bandpass filtering : 0.3Hz ~ 1Hz
// */
// double WIDM_BPF_walking_Prof(double r)
// {
// 	double y_1 = 0.0;
// 	static uint8_t firstRun_1 = 0;
// 	static double y1_1 = 0.0, y2_1 = 0.0, y3_1 = 0.0, r1_1 = 0.0, r2_1 = 0.0, r3_1 = 0.0;
// 	if (firstRun_1 == 0){
// 		y3_1 = r;
// 		y2_1 = r;
// 		y1_1 = r;
// 		r3_1 = 0;
// 		r2_1 = 0;
// 		r1_1 = 0;
// 		firstRun_1 = 1;
// 	}

// 	// First code //
// 	y_1 = (2.985589845070495*y1_1 - 2.971242511965642*y2_1 + 0.985652593015590*y3_1 + 0.0001 * (0.195971613578490*r1_1 - 0.195971613578490*r3_1));

// 	y3_1 = y2_1;
//     y2_1 = y1_1;
//     y1_1 = y_1;
//     r3_1 = r2_1;
//     r2_1 = r1_1;
//     r1_1 = r;

//     return y_1;
// }

// /*
// *Bandpass filtering : Peaking cutoff Frequency
// */
// double WIDM_BPF_Peak_Prof_1(double r, double w)
// {
// 	double y_2 = 0.0;
// 	static uint8_t firstRun_2 = 0;
// 	static double y1_2 = 0.0, y2_2 = 0.0, y3_2 = 0.0, r1_2 = 0.0, r2_2 = 0.0, r3_2 = 0.0;

// 	double T_2 = 0.001;
// 	double a_2 = 1.02;
// 	double b_2 = 8;

// 	if (firstRun_2 == 0){
// 		y3_2 = r;
// 		y2_2 = r;
// 		y1_2 = r;
// 		r3_2 = 0;
// 		r2_2 = 0;
// 		r1_2 = 0;
// 		firstRun_2 = 1;
// 	}

// 	y_2 = ( - ( 2*w*w - 24/(T_2*T_2) - (4*a_2*w)/T_2 - (4*b_2*w)/T_2 + 3*T_2*b_2*w*w*w + 2*a_2*b_2*w*w )*y1_2 - ( 24/(T_2*T_2) - 2*w*w - (4*a_2*w)/T_2 - (4*b_2*w)/T_2 + 3*T_2*b_2*w*w*w - 2*a_2*b_2*w*w )*y2_2 - ( T_2*b_2*w*w*w - 2*a_2*b_2*w*w + (4*a_2*w)/T_2 + (4*b_2*w)/T_2 - 8/(T_2*T_2) - 2*w*w )*y3_2 + 2*a_2*b_2*w*w * ( r + r1_2 - r2_2 - r3_2 ) ) / ( 2*w*w + 8/(T_2*T_2) + (4*a_2*w)/T_2 + (4*b_2*w)/T_2 + T_2*b_2*w*w*w + 2*a_2*b_2*w*w );

// 	y3_2 = y2_2;
//     y2_2 = y1_2;
//     y1_2 = y_2;
//     r3_2 = r2_2;
//     r2_2 = r1_2;
//     r1_2 = r;

//     return y_2;
// }

// double WIDM_BPF_Peak_Prof_1_RH(double r, double w)
// {
// 	double y_2_RH = 0.0;
// 	static uint8_t firstRun_2_RH = 0;
// 	static double y1_2_RH = 0.0, y2_2_RH = 0.0, y3_2_RH = 0.0, r1_2_RH = 0.0, r2_2_RH = 0.0, r3_2_RH = 0.0;

// 	double T_2_RH = 0.001;
// 	double a_2_RH = 1.02;
// 	double b_2_RH = 8;

// 	if (firstRun_2_RH == 0) {
// 		y3_2_RH = r;
// 		y2_2_RH = r;
// 		y1_2_RH = r;
// 		r3_2_RH = 0;
// 		r2_2_RH = 0;
// 		r1_2_RH = 0;
// 		firstRun_2_RH = 1;
// 	}

// 	y_2_RH = ( - ( 2*w*w - 24/(T_2_RH*T_2_RH) - (4*a_2_RH*w)/T_2_RH - (4*b_2_RH*w)/T_2_RH + 3*T_2_RH*b_2_RH*w*w*w + 2*a_2_RH*b_2_RH*w*w )*y1_2_RH - ( 24/(T_2_RH*T_2_RH) - 2*w*w - (4*a_2_RH*w)/T_2_RH - (4*b_2_RH*w)/T_2_RH + 3*T_2_RH*b_2_RH*w*w*w - 2*a_2_RH*b_2_RH*w*w )*y2_2_RH - ( T_2_RH*b_2_RH*w*w*w - 2*a_2_RH*b_2_RH*w*w + (4*a_2_RH*w)/T_2_RH + (4*b_2_RH*w)/T_2_RH - 8/(T_2_RH*T_2_RH) - 2*w*w )*y3_2_RH + 2*a_2_RH*b_2_RH*w*w * ( r + r1_2_RH - r2_2_RH - r3_2_RH ) ) / ( 2*w*w + 8/(T_2_RH*T_2_RH) + (4*a_2_RH*w)/T_2_RH + (4*b_2_RH*w)/T_2_RH + T_2_RH*b_2_RH*w*w*w + 2*a_2_RH*b_2_RH*w*w );


// 	y3_2_RH = y2_2_RH;
//     y2_2_RH = y1_2_RH;
//     y1_2_RH = y_2_RH;
//     r3_2_RH = r2_2_RH;
//     r2_2_RH = r1_2_RH;
//     r1_2_RH = r;

//     return y_2_RH;
// }

// double WIDM_BPF_Peak_Prof_1_LH(double r, double w)
// {
// 	double y_2_LH = 0.0;
// 	static uint8_t firstRun_2_LH = 0;
// 	static double y1_2_LH = 0.0, y2_2_LH = 0.0, y3_2_LH = 0.0, r1_2_LH = 0.0, r2_2_LH = 0.0, r3_2_LH = 0.0;

// 	double T_2_LH = 0.001;
// 	double a_2_LH = 1.02;
// 	double b_2_LH = 8;

// 	if (firstRun_2_LH == 0){
// 		y3_2_LH = r;
// 		y2_2_LH = r;
// 		y1_2_LH = r;
// 		r3_2_LH = 0;
// 		r2_2_LH = 0;
// 		r1_2_LH = 0;
// 		firstRun_2_LH = 1;
// 	}

// 	y_2_LH = ( - ( 2*w*w - 24/(T_2_LH*T_2_LH) - (4*a_2_LH*w)/T_2_LH - (4*b_2_LH*w)/T_2_LH + 3*T_2_LH*b_2_LH*w*w*w + 2*a_2_LH*b_2_LH*w*w )*y1_2_LH - ( 24/(T_2_LH*T_2_LH) - 2*w*w - (4*a_2_LH*w)/T_2_LH - (4*b_2_LH*w)/T_2_LH + 3*T_2_LH*b_2_LH*w*w*w - 2*a_2_LH*b_2_LH*w*w )*y2_2_LH - ( T_2_LH*b_2_LH*w*w*w - 2*a_2_LH*b_2_LH*w*w + (4*a_2_LH*w)/T_2_LH + (4*b_2_LH*w)/T_2_LH - 8/(T_2_LH*T_2_LH) - 2*w*w )*y3_2_LH + 2*a_2_LH*b_2_LH*w*w * ( r + r1_2_LH - r2_2_LH - r3_2_LH ) ) / ( 2*w*w + 8/(T_2_LH*T_2_LH) + (4*a_2_LH*w)/T_2_LH + (4*b_2_LH*w)/T_2_LH + T_2_LH*b_2_LH*w*w*w + 2*a_2_LH*b_2_LH*w*w );

// 	y3_2_LH = y2_2_LH;
//     y2_2_LH = y1_2_LH;
//     y1_2_LH = y_2_LH;
//     r3_2_LH = r2_2_LH;
//     r2_2_LH = r1_2_LH;
//     r1_2_LH = r;

//     return y_2_LH;
// }

// /*
// *Bandpass filtering : Peaking cutoff Frequency
// */
// double WIDM_BPF_Peak_Prof_2(double r, double w)
// {
// 	double y_3 = 0.0;
// 	static uint8_t firstRun_3 = 0;
// 	static double y1_3 = 0.0, y2_3 = 0.0, y3_3 = 0.0, r1_3 = 0.0, r2_3 = 0.0, r3_3 = 0.0;

// 	double T_3 = 0.001;
// 	double a_3 = 0.62;
// 	double b_3 = 6;

// 	if (firstRun_3 == 0) {
// 		y3_3 = r;
// 		y2_3 = r;
// 		y1_3 = r;
// 		r3_3 = 0;
// 		r2_3 = 0;
// 		r1_3 = 0;
// 		firstRun_3 = 1;
// 	}

// 	y_3 = ( - ( 2*w*w - 24/(T_3*T_3) - (4*a_3*w)/T_3 - (4*b_3*w)/T_3 + 3*T_3*b_3*w*w*w + 2*a_3*b_3*w*w )*y1_3 - ( 24/(T_3*T_3) - 2*w*w - (4*a_3*w)/T_3 - (4*b_3*w)/T_3 + 3*T_3*b_3*w*w*w - 2*a_3*b_3*w*w )*y2_3 - ( T_3*b_3*w*w*w - 2*a_3*b_3*w*w + (4*a_3*w)/T_3 + (4*b_3*w)/T_3 - 8/(T_3*T_3) - 2*w*w )*y3_3 + 2*a_3*b_3*w*w * ( r + r1_3 - r2_3 - r3_3 ) ) / ( 2*w*w + 8/(T_3*T_3) + (4*a_3*w)/T_3 + (4*b_3*w)/T_3 + T_3*b_3*w*w*w + 2*a_3*b_3*w*w );

// 	y3_3 = y2_3;
//     y2_3 = y1_3;
//     y1_3 = y_3;
//     r3_3 = r2_3;
//     r2_3 = r1_3;
//     r1_3 = r;

//     return y_3;
// }

// double WIDM_BPF_Peak_Prof_2_RH(double r, double w)
// {
// 	double y_3_RH = 0.0;
// 	static uint8_t firstRun_3_RH = 0;
// 	static double y1_3_RH = 0.0, y2_3_RH = 0.0, y3_3_RH = 0.0, r1_3_RH = 0.0, r2_3_RH = 0.0, r3_3_RH = 0.0;

// 	double T_3_RH = 0.001;
// 	double a_3_RH = 0.62;
// 	double b_3_RH = 6;

// 	if (firstRun_3_RH == 0) {
// 		y3_3_RH = r;
// 		y2_3_RH = r;
// 		y1_3_RH = r;
// 		r3_3_RH = 0;
// 		r2_3_RH = 0;
// 		r1_3_RH = 0;
// 		firstRun_3_RH = 1;
// 	}

// 	y_3_RH = ( - ( 2*w*w - 24/(T_3_RH*T_3_RH) - (4*a_3_RH*w)/T_3_RH - (4*b_3_RH*w)/T_3_RH + 3*T_3_RH*b_3_RH*w*w*w + 2*a_3_RH*b_3_RH*w*w )*y1_3_RH - ( 24/(T_3_RH*T_3_RH) - 2*w*w - (4*a_3_RH*w)/T_3_RH - (4*b_3_RH*w)/T_3_RH + 3*T_3_RH*b_3_RH*w*w*w - 2*a_3_RH*b_3_RH*w*w )*y2_3_RH - ( T_3_RH*b_3_RH*w*w*w - 2*a_3_RH*b_3_RH*w*w + (4*a_3_RH*w)/T_3_RH + (4*b_3_RH*w)/T_3_RH - 8/(T_3_RH*T_3_RH) - 2*w*w )*y3_3_RH + 2*a_3_RH*b_3_RH*w*w * ( r + r1_3_RH - r2_3_RH - r3_3_RH ) ) / ( 2*w*w + 8/(T_3_RH*T_3_RH) + (4*a_3_RH*w)/T_3_RH + (4*b_3_RH*w)/T_3_RH + T_3_RH*b_3_RH*w*w*w + 2*a_3_RH*b_3_RH*w*w );

// 	y3_3_RH = y2_3_RH;
//     y2_3_RH = y1_3_RH;
//     y1_3_RH = y_3_RH;
//     r3_3_RH = r2_3_RH;
//     r2_3_RH = r1_3_RH;
//     r1_3_RH = r;

//     return y_3_RH;
// }

// double WIDM_BPF_Peak_Prof_2_LH(double r, double w)
// {
// 	double y_3_LH = 0.0;
// 	static uint8_t firstRun_3_LH = 0;
// 	static double y1_3_LH = 0.0, y2_3_LH = 0.0, y3_3_LH = 0.0, r1_3_LH = 0.0, r2_3_LH = 0.0, r3_3_LH = 0.0;

// 	double T_3_LH = 0.001;
// //	double a_3_LH = 0.62;
// //	double b_3_LH = 6;
// 	double a_3_LH = 1.02;
// 	double b_3_LH = 6;

// 	if (firstRun_3_LH == 0) {
// 		y3_3_LH = r;
// 		y2_3_LH = r;
// 		y1_3_LH = r;
// 		r3_3_LH = 0;
// 		r2_3_LH = 0;
// 		r1_3_LH = 0;
// 		firstRun_3_LH = 1;
// 	}

// 	y_3_LH = ( - ( 2*w*w - 24/(T_3_LH*T_3_LH) - (4*a_3_LH*w)/T_3_LH - (4*b_3_LH*w)/T_3_LH + 3*T_3_LH*b_3_LH*w*w*w + 2*a_3_LH*b_3_LH*w*w )*y1_3_LH - ( 24/(T_3_LH*T_3_LH) - 2*w*w - (4*a_3_LH*w)/T_3_LH - (4*b_3_LH*w)/T_3_LH + 3*T_3_LH*b_3_LH*w*w*w - 2*a_3_LH*b_3_LH*w*w )*y2_3_LH - ( T_3_LH*b_3_LH*w*w*w - 2*a_3_LH*b_3_LH*w*w + (4*a_3_LH*w)/T_3_LH + (4*b_3_LH*w)/T_3_LH - 8/(T_3_LH*T_3_LH) - 2*w*w )*y3_3_LH + 2*a_3_LH*b_3_LH*w*w * ( r + r1_3_LH - r2_3_LH - r3_3_LH ) ) / ( 2*w*w + 8/(T_3_LH*T_3_LH) + (4*a_3_LH*w)/T_3_LH + (4*b_3_LH*w)/T_3_LH + T_3_LH*b_3_LH*w*w*w + 2*a_3_LH*b_3_LH*w*w );

// 	y3_3_LH = y2_3_LH;
//     y2_3_LH = y1_3_LH;
//     y1_3_LH = y_3_LH;
//     r3_3_LH = r2_3_LH;
//     r2_3_LH = r1_3_LH;
//     r1_3_LH = r;

//     return y_3_LH;
// }

// /*
// *Low pass filtering
// */
// double WIDM_LPF_walking_Prof(double r)
// {
// 	double y_4 = 0.0;
// 	static uint8_t firstRun_4 = 0;
// 	static double y1_4 = 0.0;

// 	if (firstRun_4 == 0){
// 		y1_4 = r;
// 		firstRun_4 = 1;
// 	}

// 	// WIDM3 code //
// 	y_4 = 0.98 * y1_4 + 0.02 * r;

//     y1_4 = y_4;

//     return y_4;
// }

// double WIDM_LPF_walking_Prof_RH(double r)
// {
// 	double y_4_RH = 0.0;
// 	static uint8_t firstRun_4_RH = 0;
// 	static double y1_4_RH = 0.0;

// 	if (firstRun_4_RH == 0) {
// 		y1_4_RH = r;
// 		firstRun_4_RH = 1;
// 	}

// 	// WIDM3 code //
// 	y_4_RH = 0.98 * y1_4_RH + 0.02 * r;

//     y1_4_RH = y_4_RH;

//     return y_4_RH;
// }

// double WIDM_LPF_walking_Prof_LH(double r)
// {
// 	double y_4_LH;
// 	static uint8_t firstRun_4_LH = 0;
// 	static double y1_4_LH;

// 	if (firstRun_4_LH == 0) {
// 		y1_4_LH = r;
// 		firstRun_4_LH = 1;
// 	}

// 	// WIDM3 code //
// 	y_4_LH = 0.98 * y1_4_LH + 0.02 * r;

//     y1_4_LH = y_4_LH;

//     return y_4_LH;
// }

// double WIDM_LPF_1NE(double r, double w)
// {
// 	double y_LPF_1ne = 0.0;
// 	static uint8_t firstRun_LPF_1ne = 0;
// 	static double y1_LPF_1ne = 0.0;
// 	static double r1_LPF_1ne = 0.0;
// 	double T_1ne = 0.001;
// 	double K_1ne = 5;

// 	if (firstRun_LPF_1ne == 0) {
// 		y1_LPF_1ne = r;
// 		r1_LPF_1ne = 0;
// 		firstRun_LPF_1ne = 1;
// 	}

// 	y_LPF_1ne = ((2 - w*T_1ne)*y1_LPF_1ne + K_1ne*w*T_1ne*r + K_1ne*w*T_1ne*r1_LPF_1ne) / (2 + w*T_1ne);

//     y1_LPF_1ne = y_LPF_1ne;
//     r1_LPF_1ne = r;

//     return y_LPF_1ne;
// }


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- Gait Data Estimation Filters ------------------- */
/* BPF & LPF Code Refactoring By HD */
/* Band Pass Filters */
// /**
//  * @brief Initializes the band-pass filter state with the first input.
//  * 
//  * This function sets the initial state for the band-pass filter using the current input.
//  * It also resets previous inputs and outputs to ensure the filter starts smoothly.
//  * 
//  * @param state Pointer to the filter state structure.
//  * @param currentInput The current input signal to initialize the filter with.
//  */
// static void InitBPF(WIDM_BPFState* state, double currentInput)
// {
//     for (int i = 0; i < 3; i++) {
//         state->prevOutput[i] = currentInput; // Set initial output to current input to avoid startup transients
//         state->prevInput[i] = 0.0;           // Reset previous inputs to zero
//     }
//     state->isInitialized = true;             // Mark the filter as initialized
// }