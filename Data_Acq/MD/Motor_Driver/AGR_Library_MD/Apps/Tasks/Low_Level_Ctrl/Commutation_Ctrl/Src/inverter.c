

#include "inverter.h"
#include "low_level_ctrl.h"

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

// This sinewave lookup table has 171 entries.  (1024 points per electrical cycle -- 1024*(60/360) = 171)
// The table covers 60 degrees of the sine function. (sin(60) * 32768 = 28377)
const int sinetable[172] =
{0,201,401,602,803,1003,1204,1404,1605,1805,
2005,2206,2406,2606,2806,3006,3205,3405,3605,3804,4003,4202,4401,4600,
4799,4997,5195,5393,5591,5789,5986,6183,6380,6577,6773,6970,7166,7361,
7557,7752,7947,8141,8335,8529,8723,8916,9109,9302,9494,9686,9877,10068,
10259,10449,10639,10829,11018,11207,11395,11583,11771,11958,12144,
12331,12516,12701,12886,13070,13254,13437,13620,13802,13984,14165,
14346,14526,14706,14885,15063,15241,15419,15595,15772,15947,16122,
16297,16470,16643,16816,16988,17159,17330,17500,17669,17838,18006,
18173,18340,18506,18671,18835,18999,19162,19325,19487,19647,19808,
19967,20126,20284,20441,20598,20753,20908,21062,21216,21368,21520,
21671,21821,21970,22119,22266,22413,22559,22704,22848,22992,23134,
23276,23417,23557,23696,23834,23971,24107,24243,24377,24511,24644,
24776,24906,25036,25165,25293,25420,25547,25672,25796,25919,26042,
26163,26283,26403,26521,26638,26755,26870,26984,27098,27210,27321,
27431,27541,27649,27756,27862,27967,28071,28174,28276,28377};


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

void GateDriver_ONOFF(uint8_t t_mode)
{
	if(t_mode) {
        IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, IOIF_GPIO_PIN_14, IOIF_GPIO_PIN_SET);   // DRV8350_EN_Pin
	} else 	 {
        IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, IOIF_GPIO_PIN_14, IOIF_GPIO_PIN_RESET); // DRV8350_EN_Pin
	}
}

void Stop_PWM(void)
{
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}


/* ------------------- FORCE COMMUTATION ------------------- */
void excitate_phase_UVnWn(uint16_t t_duty)     // UV'W'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, t_duty);                 		// U
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, DUTY_RESOLUTION);         		// V'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, DUTY_RESOLUTION);          	// W'
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UVWn(uint16_t t_duty)      // UVW'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, t_duty);                 		// U
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, t_duty);                 		// V
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, DUTY_RESOLUTION);          	// W'
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UnVWn(uint16_t t_duty)     // U'VW'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, DUTY_RESOLUTION);      		// U'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, t_duty);      					// V
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, DUTY_RESOLUTION);      		// W'
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UnVW(uint16_t t_duty)      // U'VW
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, DUTY_RESOLUTION);      		// U'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, t_duty);      					// V
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, t_duty);      					// W
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UnVnW(uint16_t t_duty)     // U'V'W
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, DUTY_RESOLUTION);      		// U'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, DUTY_RESOLUTION);     			// V'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, t_duty);      					// W
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UVnW(uint16_t t_duty)      // UV'W
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, t_duty);                 		// U
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, DUTY_RESOLUTION);          	// V'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, t_duty);                		// W
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UnVnWn()      // U'V'W'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, DUTY_RESOLUTION);      // U'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, DUTY_RESOLUTION);      // V'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, DUTY_RESOLUTION);      // W'
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UVn(uint16_t t_duty)     // UV'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, t_duty);              			// U
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, DUTY_RESOLUTION);          	// V'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, 0);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_VnW(uint16_t t_duty)     // WV'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, 0);
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, DUTY_RESOLUTION);   			// V'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, t_duty);   					// W
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UnW(uint16_t t_duty)     // WU'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, DUTY_RESOLUTION);   			// U'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, 0);
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, t_duty);              			// W
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UnV(uint16_t t_duty)     // VU'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, DUTY_RESOLUTION);   			// U'
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, t_duty);   					// V
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, 0);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_VWn(uint16_t t_duty)     // VW'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, 0);
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, t_duty);              			// V
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, DUTY_RESOLUTION);          	// W'
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

void excitate_phase_UWn(uint16_t t_duty)     // UW'
{
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, t_duty);              			// U
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, 0);
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, DUTY_RESOLUTION);   			// W'
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);	IOIF_StopPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StopPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);	IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);
}

/* ------------------- CLARK & PARK TF ------------------- */
void ClarkeTransform(const ClarkeIn *t_clarke_in, ClarkeOut *t_clarke_out)
{
	static int32_t t_Ia = 0, t_Ib = 0;

	int32_t t_Iu = t_clarke_in->Iu;
	int32_t t_Iv = t_clarke_in->Iv;
	int32_t t_Iw = t_clarke_in->Iw;

	t_Ia = (21845 * t_Iu - 10923 * (t_Iv + t_Iw)) >> 15;		// 21845/32768 = (2/3), 10923/32768 = (1/3)
	t_Ib = (18919 * (t_Iv - t_Iw)) >> 15;						// 18919/32768 = (sqrt(3)/3)

	t_clarke_out->Ia = t_Ia;
	t_clarke_out->Ib = t_Ib;
}

void ParkTransform(const ParkIn *t_park_in, ParkOut *t_park_out)
{
	static int32_t t_Id = 0, t_Iq = 0;
	static uint16_t t_park_angle = 0;

	t_park_angle = t_park_in->theta_e;

	t_Id = t_park_in->Ia * cosine_signal[t_park_angle] + t_park_in->Ib * sine_signal[t_park_angle];
	t_Iq = t_park_in->Ib * cosine_signal[t_park_angle] - t_park_in->Ia * sine_signal[t_park_angle];

	t_park_out->Id =(t_Id); // + park_out->Id)>>1;
	t_park_out->Iq =(t_Iq); // + park_out->Iq)>>1;
}

void InvParkTransform(const InvParkIn *t_invpark_in, InvParkOut *t_invpark_out)
{
	static int32_t t_Va = 0, t_Vb = 0;
	static uint16_t t_park_angle = 0;

	t_park_angle = t_invpark_in->theta_e;

	t_Va = t_invpark_in->Vd * cosine_signal[t_park_angle] - t_invpark_in->Vq * sine_signal[t_park_angle];
	t_Vb = t_invpark_in->Vq * cosine_signal[t_park_angle] + t_invpark_in->Vd * sine_signal[t_park_angle];

	t_invpark_out->Va = t_Va;
	t_invpark_out->Vb = t_Vb;
}

void InvParkInputToPhasor(InvParkIn *t_invpark_in, Phasor *t_phasor)
{
	static double t_Vd = 0.0, t_Vq = 0.0;
	static double t_VdVq_ratio = 0.0;

	t_Vd = (double)t_invpark_in->Vd;
	t_Vq = (double)t_invpark_in->Vq;

	if (t_Vd != 0) {
		t_VdVq_ratio = t_Vq / t_Vd;
	} else {
		// TODO: Handle division by zero
		t_VdVq_ratio = 0.0;
	}
	
	t_phasor->magnitude = sqrt(t_Vd * t_Vd + t_Vq * t_Vq);

	if ((t_Vd > 0) && (t_Vq > 0)) {
		t_phasor->phase = (uint16_t)(10430 * atan(t_VdVq_ratio));
	} else if ((t_Vd < 0) && (t_Vq > 0)) {
		t_phasor->phase = (uint16_t)((10430 * atan(t_VdVq_ratio)) + (0xFFFF >> 1));
	} else if ((t_Vd < 0) && (t_Vq < 0)) {
		t_phasor->phase = (uint16_t)(10430 * atan(t_VdVq_ratio)) + (0xFFFF >> 1);
	} else if ((t_Vd > 0) && (t_Vq < 0)) {
		t_phasor->phase = (uint16_t)((10430 * atan(t_VdVq_ratio)) + (0xFFFF));
	}
}

/* ------------------- SPACE VECTOR MODULARIZATION ------------------- */
#ifdef _USE_HW_OVER_REV06
void SVM_FOC(InvParkOut* pInvParkOut, float ulVdc, float* t_v_in_U, float* t_v_in_V, float* t_v_in_W)
{

	static double Vds = 0.0, Vqs = 0.0;
	static float ulVector_limit = 0;
	uint8_t ucSector = 0;
	int32_t lTemp = 0, lTT1 = 0, lTT2 = 0;
	uint32_t ulT1 = 0, ulT2 = 0, ulT0 = 0, ulT0_h = 0, ulTemp = 0;
	uint16_t usTa = 0, usTb = 0, usTc = 0;

	Vds = (double)pInvParkOut->Va;
	Vqs = (double)pInvParkOut->Vb;



	ulVector_limit = (ulVdc*INV_SQRT_3);
	pInvParkOut->V_mag = sqrt((Vds * Vds) + (Vqs * Vqs));

	if(ulVector_limit < pInvParkOut->V_mag){
		pInvParkOut->Va = (pInvParkOut->Va * ulVector_limit) / pInvParkOut->V_mag;
		pInvParkOut->Vb = (pInvParkOut->Vb * ulVector_limit) / pInvParkOut->V_mag;
	}

/* SVM Sector determination */
	lTemp = (pInvParkOut->Va * SQRT_3)>>15;

	if (pInvParkOut->Vb >= 0)
	{
		if (pInvParkOut->Vb < lTemp)		ucSector = 0;
		else if (pInvParkOut->Vb < -lTemp)	ucSector = 2;
		else								ucSector = 1;
	}
	else
	{
		if (pInvParkOut->Vb >= lTemp)		ucSector = 3;
		else if (pInvParkOut->Vb >= -lTemp)	ucSector = 5;
		else								ucSector = 4;
	}

/* TT1/2 Calculation */
	switch(ucSector)
	{
		case 0:
			lTT1 = (pInvParkOut->Va*SIN60 - pInvParkOut->Vb*COS60)>>15;
			lTT2 = (pInvParkOut->Vb*COS00 - pInvParkOut->Va*SIN00)>>15;
			break;
		case 1:
			lTT2 = (pInvParkOut->Va*SIN60 + pInvParkOut->Vb*COS60)>>15;
			lTT1 = (pInvParkOut->Vb*COS60 - pInvParkOut->Va*SIN60)>>15;
			break;
		case 2:
			lTT1 = (pInvParkOut->Va*SIN00 + pInvParkOut->Vb*COS00)>>15;
			lTT2 = (-pInvParkOut->Vb*COS60 - pInvParkOut->Va*SIN60)>>15;
			break;
		case 3:
			lTT2 = (-pInvParkOut->Va*SIN60 + pInvParkOut->Vb*COS60)>>15;
			lTT1 = (-pInvParkOut->Vb*COS00 - pInvParkOut->Va*SIN00)>>15;
			break;
		case 4:
			lTT1 = (-pInvParkOut->Va*SIN60 - pInvParkOut->Vb*COS60)>>15;
			lTT2 = (-pInvParkOut->Vb*COS60 + pInvParkOut->Va*SIN60)>>15;
			break;
		case 5:
			lTT2 = (pInvParkOut->Va*SIN00 - pInvParkOut->Vb*COS00)>>15;
			lTT1 = (pInvParkOut->Vb*COS60 + pInvParkOut->Va*SIN60)>>15;
			break;
		default:
			break;
	}

	if(lTT1 <= 0) lTT1 = 0;
	if(lTT2 <= 0) lTT2 = 0;

/* T1/2/0 Calculation - Need To debug : sqrt(3)/2   --> Complete */
	ulTemp = ulVdc;

	lTemp = ((lTT1*SQRT_3)>>15)*TIM1_MD_PWM_MAX_CNT;
	ulT1 = (lTemp/ulTemp);

	lTemp = ((lTT2*SQRT_3)>>15)*TIM1_MD_PWM_MAX_CNT;
	ulT2 = (lTemp/ulTemp);

	ulTemp = ulT1 + ulT2;
	ulT0 = TIM1_MD_PWM_MAX_CNT - ulTemp;

/* @Over-modulation */
	if(ulTemp > TIM1_MD_PWM_MAX_CNT)
	{

		ulT1 = (TIM1_MD_PWM_MAX_CNT * ulT1)/ulTemp;
		ulT2 = (TIM1_MD_PWM_MAX_CNT * ulT2)/ulTemp;

		ulTemp = ulT1 + ulT2;

		if(ulTemp <= TIM1_MD_PWM_MAX_CNT)
		{
			ulT0 = TIM1_MD_PWM_MAX_CNT - ulTemp;
		}
		else
		{
			ulT0 = 0;
		}
	}

	if (MD_DPWM_MODE == DPWM_MODE_SVM)
	{
		ulT0_h = ulT0>>1;
	}
	else if (MD_DPWM_MODE == DPWM_MODE_MAX)
	{
		ulT0_h = ulT0;
	}
	else if (MD_DPWM_MODE == DPWM_MODE_MIN)
	{
		ulT0_h = 0;
	}
	else
	{
		ulT0_h = ulT0>>1;
	}

/* Get Ta/b/c */
	switch(ucSector)
	{
		case 0:
			usTa = ulT0_h + ulT1 + ulT2;
			usTb = usTa - ulT1;
			usTc = usTb - ulT2;
			break;
		case 1:
			usTb = ulT0_h + ulT1 + ulT2;
			usTa = usTb - ulT1;
			usTc = usTa - ulT2;
			break;
		case 2:
			usTb = ulT0_h + ulT1 + ulT2;
			usTc = usTb - ulT1;
			usTa = usTc - ulT2;
			break;
		case 3:
			usTc = ulT0_h + ulT1 + ulT2;
			usTb = usTc - ulT1;
			usTa = usTb - ulT2;
			break;
		case 4:
			usTc = ulT0_h + ulT1 + ulT2;
			usTa = usTc - ulT1;
			usTb = usTa - ulT2;
			break;
		case 5:
			usTa = ulT0_h + ulT1 + ulT2;
			usTc = usTa - ulT1;
			usTb = usTc - ulT2;
			break;
		default:
			break;
	}

	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, usTa);
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, usTb);
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, usTc);

	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);  IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);  IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);  IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);

	*t_v_in_U = (float)(usTa);
	*t_v_in_V = (float)(usTb);
	*t_v_in_W = (float)(usTc);
}

#else
void SVM(int t_volts, uint16_t t_angle, float* t_v_in_U, float* t_v_in_V, float* t_v_in_W) // volts = duty (max: 32768 / ex: conventional duty 2400(4000) -> 16384)
{		
	// These variables hold the normalized sector angles used to find t1, t2.
	unsigned int t_angle1 = 0, t_angle2 = 0;

	// These variables hold the space vector times.
	unsigned int t_half_t0 = 0, t_t1 = 0, t_t2 = 0, t_tpwm = 0;
	unsigned int t_CCR1 = 0, t_CCR2 = 0, t_CCR3 = 0;

	// Calculate the total PWM count period
	//tpwm = (htim->Instance->ARR + 1) << 1;
	t_tpwm = htim1.Instance->ARR;			// duty_resolutio, 4800 -> 8000

	// Limit volts input to avoid over-modulation.
	if (t_volts > VOLTS_LIMIT) {
		t_volts = VOLTS_LIMIT;
	}

	if (t_angle < VECTOR2) {		// SVM하고, scale down 0~4799 ERROR 처리 필요
		t_angle2 = t_angle - VECTOR1;		// Reference SVM angle to the current sector
		t_angle1 = SIXTY_DEG - t_angle2;	// Calculate second angle referenced to sector

		t_t1 = sinetable[(unsigned char)(t_angle1 >> 6)];	// Look up values from table.
		t_t2 = sinetable[(unsigned char)(t_angle2 >> 6)];

		// Scale t1 to by the volts variable.
		t_t1 = ((long)t_t1*(long)t_volts) >> 15;
		t_t1 = ((long)t_t1*(long)t_tpwm) >> 15;
		// Scale t2 time
		t_t2 = ((long)t_t2*(long)t_volts) >> 15;
		t_t2 = ((long)t_t2*(long)t_tpwm) >> 15;

		t_half_t0 = (t_tpwm - t_t1 - t_t2) >> 1;		// Calculate half_t0 null time from period and t1,t2

		// Calculate duty cycles for Sector 1  (0 - 59 degrees)

		t_CCR1 = t_t1 + t_t2 + t_half_t0;
		t_CCR2 = t_t2 + t_half_t0;
		t_CCR3 = t_half_t0;
	} else if (t_angle < VECTOR3) {

		t_angle2 = t_angle - VECTOR2;		// Reference SVM angle to the current sector
		t_angle1 = SIXTY_DEG - t_angle2;	// Calculate second angle referenced to sector

		t_t1 = sinetable[(unsigned char)(t_angle1 >> 6)];	// Look up values from table.
		t_t2 = sinetable[(unsigned char)(t_angle2 >> 6)];

		// Scale t1 to by the volts variable.
		t_t1 = ((long)t_t1*(long)t_volts) >> 15;
		// Scale t1 for the duty cycle range.
		t_t1 = ((long)t_t1*(long)t_tpwm) >> 15;
		// Scale t2 time
		t_t2 = ((long)t_t2*(long)t_volts) >> 15;
		t_t2 = ((long)t_t2*(long)t_tpwm) >> 15;

		t_half_t0 = (t_tpwm - t_t1 - t_t2) >> 1;		// Calculate half_t0 null time from period and t1,t2

		// Calculate duty cycles for Sector 2  (60 - 119 degrees)

		t_CCR1 = t_t1 + t_half_t0;
		t_CCR2 = t_t1 + t_t2 + t_half_t0;
		t_CCR3 = t_half_t0;
	} else if (t_angle < VECTOR4) {
		t_angle2 = t_angle - VECTOR3;		// Reference SVM angle to the current sector
		t_angle1 = SIXTY_DEG - t_angle2;	// Calculate second angle referenced to sector

		t_t1 = sinetable[(unsigned char)(t_angle1 >> 6)];	// Look up values from table.
		t_t2 = sinetable[(unsigned char)(t_angle2 >> 6)];

		// Scale t1 to by the volts variable.
		t_t1 = ((long)t_t1 * (long)t_volts) >> 15;
		// Scale t1 for the duty cycle range.
		t_t1 = ((long)t_t1 * (long)t_tpwm) >> 15;
		// Scale t2 time
		t_t2 = ((long)t_t2 * (long)t_volts) >> 15;
		t_t2 = ((long)t_t2 * (long)t_tpwm) >> 15;

		t_half_t0 = (t_tpwm - t_t1 - t_t2) >> 1;		// Calculate half_t0 null time from period and t1,t2

		// Calculate duty cycles for Sector 3  (120 - 179 degrees)

		t_CCR1 = t_half_t0;
		t_CCR2 = t_t1 + t_t2 + t_half_t0;
		t_CCR3 = t_t2 + t_half_t0;
	} else if(t_angle < VECTOR5) {
		t_angle2 = t_angle - VECTOR4;		// Reference SVM angle to the current sector
		t_angle1 = SIXTY_DEG - t_angle2;	// Calculate second angle referenced to sector

		t_t1 = sinetable[(unsigned char)(t_angle1 >> 6)];	// Look up values from table.
		t_t2 = sinetable[(unsigned char)(t_angle2 >> 6)];

		// Scale t1 to by the volts variable.
		t_t1 = ((long)t_t1 * (long)t_volts) >> 15;
		// Scale t1 for the duty cycle range.
		t_t1 = ((long)t_t1 * (long)t_tpwm) >> 15;
		// Scale t2 time
		t_t2 = ((long)t_t2 * (long)t_volts) >> 15;
		t_t2 = ((long)t_t2 * (long)t_tpwm) >> 15;

		t_half_t0 = (t_tpwm - t_t1 - t_t2) >> 1;		// Calculate half_t0 null time from period and t1,t2

		// Calculate duty cycles for Sector 4  (180 - 239 degrees)

		t_CCR1 = t_half_t0;
		t_CCR2 = t_t1 + t_half_t0;
		t_CCR3 = t_t1 + t_t2 + t_half_t0;

	} else if(t_angle < VECTOR6) {
		t_angle2 = t_angle - VECTOR5;		// Reference SVM angle to the current sector
		t_angle1 = SIXTY_DEG - t_angle2;	// Calculate second angle referenced to sector

		t_t1 = sinetable[(unsigned char)(t_angle1 >> 6)];	// Look up values from table.
		t_t2 = sinetable[(unsigned char)(t_angle2 >> 6)];
		//t1 = sin((2*PI*angle1) >> 16) << 15;
		//t2 = sin((2*PI*angle2) >> 16) << 15;

		// Scale t1 to by the volts variable.
		t_t1 = ((long)t_t1 * (long)t_volts) >> 15;
		// Scale t1 for the duty cycle range.
		t_t1 = ((long)t_t1 * (long)t_tpwm) >> 15;
		// Scale t2 time
		t_t2 = ((long)t_t2 * (long)t_volts) >> 15;
		t_t2 = ((long)t_t2 * (long)t_tpwm) >> 15;

		t_half_t0 = (t_tpwm - t_t1 - t_t2) >> 1;		// Calculate half_t0 null time from period and t1,t2

		// Calculate duty cycles for Sector 5  (240 - 299 degrees)

		t_CCR1 = t_t2 + t_half_t0;
		t_CCR2 = t_half_t0;
		t_CCR3 = t_t1 + t_t2 + t_half_t0;

	} else {
		t_angle2 = t_angle - VECTOR6;		// Reference SVM angle to the current sector
		t_angle1 = SIXTY_DEG - t_angle2;	// Calculate second angle referenced to sector

		t_t1 = sinetable[(unsigned char)(t_angle1 >> 6)];	// Look up values from table.
		t_t2 = sinetable[(unsigned char)(t_angle2 >> 6)];

		//t1 = sin((2*PI*angle1) >> 16) << 15;
		//t2 = sin((2*PI*angle2) >> 16) << 15;

		// Scale t1 to by the volts variable.
		t_t1 = ((long)t_t1 * (long)t_volts) >> 15;
		// Scale t1 for the duty cycle range.
		t_t1 = ((long)t_t1 * (long)t_tpwm) >> 15;
		// Scale t2 time
		t_t2 = ((long)t_t2 * (long)t_volts) >> 15;
		t_t2 = ((long)t_t2 * (long)t_tpwm) >> 15;

		t_half_t0 = (t_tpwm - t_t1 - t_t2) >> 1;		// Calculate half_t0 null time from period and t1,t2

		// Calculate duty cycles for Sector 6  ( 300 - 359 degrees )

		t_CCR1 = t_t1 + t_t2 + t_half_t0;
		t_CCR2 = t_half_t0;
		t_CCR3 = t_t1 + t_half_t0;
	}

	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_1, t_CCR1);
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_2, t_CCR2);
	IOIF_SetTimCompVal(IOIF_TIM1, IOIF_TIM_CHANNEL_3, t_CCR3);

	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_1);  IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_1);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_2);  IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_2);
	IOIF_StartPWM(IOIF_TIM1, IOIF_TIM_CHANNEL_3);  IOIF_StartPWMN(IOIF_TIM1, IOIF_TIM_CHANNEL_3);

	*t_v_in_U = (float)(t_CCR1 - t_half_t0);
	*t_v_in_V = (float)(t_CCR2 - t_half_t0);
	*t_v_in_W = (float)(t_CCR3 - t_half_t0);
}			// end SVM()
#endif //_USE_HW_OVER_REV06

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
