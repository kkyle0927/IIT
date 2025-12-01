
#include "AS_ISI.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                           VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

bool isi_output_vectors[EXT_COND_ID_NUM];
bool rm_flag[RM_ID_NUM];

uint8_t assistIncBt_Test;

extern AS_BLECommState_t isBLEConnect;
extern uint8_t ble_debug;

ISI_ButtonObj assistIncBt = {0};
ISI_ButtonObj assistDecBt = {0};

bool BFlag[EXT_COND_ID_NUM];


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void Init_Bt(ISI_ButtonObj* button, IOIF_GPIOPort_t port, IOIF_GPIOPin_t pin, uint8_t state);
static void CheckMotionFlags_H10(MotionAnalysisConditions M_Flags);
static void CheckMotionFlags_K10(MotionAnalysisConditions M_Flags);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */
void Init_Button_ISI(void)
{
	Init_Bt(&assistIncBt, IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_10, ACTIVE_LOW);
	Init_Bt(&assistDecBt, IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12, ACTIVE_LOW);
}

void Read_Button(ISI_ButtonObj* t_button)
{
	uint8_t t_state;
	t_state = IOIF_ReadGPIOPin(t_button->GPIO_port, t_button->GPIO_pin);

	if (t_button->active_type == ACTIVE_HIGH) {
		if (t_state) t_button->state_curr = 1;
		else         t_button->state_curr = 0;
	} else if (t_button->active_type == ACTIVE_LOW) {
		if (t_state) t_button->state_curr = 0;
		else         t_button->state_curr = 1;
	}

	if (t_button->state_curr == 0)
		t_button->on_time = 0;
	else
		t_button->on_time++;
}

void Flush_ISI(void)
{
 	for (int i = 0; i < EXT_COND_ID_NUM; i++) {
 		isi_output_vectors[i] = false;
 	}

 	assistIncBt.on_time = 0;
 	assistDecBt.on_time = 0;
}



void Check_ISI(void)
{
	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
		CheckMotionFlags_H10(MotionFlags);
	}
	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 1 && RS_File.vec[LK_SAGITAL].usage == 1) {
		CheckMotionFlags_K10(MotionFlags);
	}

	/* Exit 1 : 양 허벅지가 모두 hyper flexion 상태이고, 모두 동작이 작을 때 */
	// to Sit
	if (BFlag[1] == true) {
		isi_output_vectors[EXT1] = true;
		BFlag[1] = false;
	}

	/* Exit 2 : 양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 양 허벅지가 비슷한 각도일 때 */
	// to stand (still)
	if (BFlag[2] == true) {
		isi_output_vectors[EXT2] = true;
		BFlag[2] = false;
	}

	/* Exit 3 : 양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 왼쪽 허벅지가 앞쪽에 있을 때 */
	// to LDS (still)
	if (BFlag[3] == true) {
		isi_output_vectors[EXT3] = true;
		BFlag[3] = false;
	}

	/* Exit 4 :양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 오른쪽 허벅지가 앞쪽에 있을 때 */
	// to RDS (still)
	if (BFlag[4] == true) {
		isi_output_vectors[EXT4] = true;
		BFlag[4] = false;
	}

	/* Exit 5 : 양 허벅지가 모두 flexion 상태이고, 모두 flexion 방향으로 동작 중이며, 양 허벅지가 비슷한 각도일 때 */
	// to sitting
	if (BFlag[5] == true) {
		isi_output_vectors[EXT5] = true;
		BFlag[5] = false;
	}

	/* Exit 6 : 양 허벅지가 모두 flexion 상태이고, 모두 extension 방향으로 동작 중이며, 양 허벅지가 비슷한 각도일 때 */
	// to standing
	if (BFlag[6] == true) {
		isi_output_vectors[EXT6] = true;
		BFlag[6] = false;
	}

	/* Exit 7 : 오른쪽 허벅지가 왼쪽보다 더 flexion 되어 있고, 오른쪽 허벅지는 flextion 방향으로 빠르게 움직일 때 */
	// stand to Left Thigh Lifting
	if (BFlag[7] == true) {
		isi_output_vectors[EXT7] = true;
		BFlag[7] = false;
	}

	/* Exit 8 : 왼쪽 허벅지가 오른쪽보다 더 flexion 되어 있고, 왼쪽 허벅지는 flextion 방향으로 빠르게 움직일 때 */
	// stand to Right Thigh Lifting
	if (BFlag[8] == true) {
		isi_output_vectors[EXT8] = true;
		BFlag[8] = false;
	}

	/* Exit 9 : 오른쪽 허벅지는 neutral 상태이고, 동작이 작으며, 왼쪽 허벅지가 오른쪽보다 더 flexion 되어 있고, extension 방향으로 느리게 동작 중일 때 */
	// to Left Thigh Down (slow)
	if (BFlag[9] == true) {
		isi_output_vectors[EXT9] = true;
		BFlag[9] = false;
	}

	/* Exit 10 : 왼쪽 허벅지는 neutral 상태이고, 동작이 작으며, 오른쪽 허벅지가 왼쪽보다 더 flexion 되어 있고, extension 방향으로 느리게 동작 중일 때 */
	// to Right Thigh Down (slow)
	if (BFlag[10] == true) {
		isi_output_vectors[EXT10] = true;
		BFlag[10] = false;
	}

	/* Exit 11 : 오른쪽 허벅지가 왼쪽 보다 앞에 있고, 왼쪽 허벅지가 flexion 방향으로 빠르게 동작 중일 때 */
	// RDS to LSW
	if (BFlag[11] == true) {
		isi_output_vectors[EXT11] = true;
		BFlag[11] = false;
	}

	/* Exit 12 : 왼쪽 허벅지가 오른쪽 보다 앞에 있고, 오른쪽 허벅지가 flexion 방향으로 빠르게 동작 중일 때 */
	// LDS to RSW
	if (BFlag[12] == true) {
		isi_output_vectors[EXT12] = true;
		BFlag[12] = false;
	}

	/* Exit 13 : 오른쪽 허벅지는 움직이고 있고, 왼쪽 허벅지가 앞쪽에 있는 상태이고, 왼쪽 허벅지 각도가 감소할 때, */
	// (LSW, L lifting) to LSW final
	if (BFlag[13] == true) {
		isi_output_vectors[EXT13] = true;
		BFlag[13] = false;
	}

	/* Exit 14 : 왼쪽 허벅지는 움직이고 있고, 오른쪽 허벅지가 앞쪽에 있는 상태이고, 오른쪽 허벅지 각도가 감소할 때, */
	// (RSW, R lifting) to RSW final
	if (BFlag[14] == true) {
		isi_output_vectors[EXT14] = true;
		BFlag[14] = false;
	}

	if (BFlag[15] == true) {
		isi_output_vectors[EXT15] = true;
		BFlag[15] = false;
	}

	/* Exit 18 : Standby Mode Off -> Gait Mode ON (물리 버튼 or 앱 버튼) */
	if (BFlag[18] == true) {
		isi_output_vectors[EXT18] = true;
		BFlag[18] = false;
	}

	/* Exit 19 : Assist Start Voice End */
	if (BFlag[19] == true) {
		isi_output_vectors[EXT19] = true;
		BFlag[19] = false;
	}

	/* Exit 20 : 왼쪽 허벅지 중립 자세 각도 설정 안된 경우 */
	if (BFlag[20] == true) {
		isi_output_vectors[EXT20] = true;
		BFlag[20] = false;
	}

	/* Exit 21 : 오른쪽 허벅지 중립 자세 각도 설정 안된 경우 */
	if (BFlag[21] == true) {
		isi_output_vectors[EXT21] = true;
		BFlag[21] = false;
	}

	/* Exit 22 : 양쪽 허벅지 중립 자세 각도 설정 안된 경우 */
	if (BFlag[22] == true) {
		isi_output_vectors[EXT22] = true;
		BFlag[22] = false;
	}

	/* Exit 23 : 중립 자세 각도 설정 안된 상태에서 양쪽 모두 중립 자세 각도 설정 완료한 경우 */
	if (BFlag[23] == true) {
		isi_output_vectors[EXT23] = true;
		BFlag[23] = false;
	}

	/* Exit 24 : 5초 이상 왼쪽 모터드라이버에서 계산 완료 신호가 오지 않았을 경우 */
	if (BFlag[24] == true) {
		isi_output_vectors[EXT24] = true;
		BFlag[24] = false;
	}

	/* Exit 25 : 5초 이상 오른쪽 모터드라이버에서 계산 완료 신호가 오지 않았을 경우 */
	if (BFlag[25] == true) {
		isi_output_vectors[EXT25] = true;
		BFlag[25] = false;
	}

	/* Exit 26 : 5초 이상 양쪽 모터드라이버에서 계산 완료 신호가 오지 않았을 경우 */
	if (BFlag[26] == true) {
		isi_output_vectors[EXT26] = true;
		BFlag[26] = false;
	}

	/* Exit 27 : 왼쪽 모터드라이버에서 I2C 오류로 IMU 값이 업데이트 되지 않는 경우 */
	if (BFlag[27] == true) {
		isi_output_vectors[EXT27] = true;
		BFlag[27] = false;
	}

	/* Exit 28 : 오른쪽 모터드라이버에서 I2C 오류로 IMU 값이 업데이트 되지 않는 경우 */
	if (BFlag[28] == true) {
		isi_output_vectors[EXT28] = true;
		BFlag[28] = false;
	}

	/* Exit 29 : 양쪽 모터드라이버에서 I2C 오류로 IMU 값이 업데이트 되지 않는 경우 */
	if (BFlag[29] == true) {
		isi_output_vectors[EXT29] = true;
		BFlag[29] = false;
	}

	/* Exit 30 : 중립 각도 계산이 모두 완료되고 음성이 종료되었을 때 */
	if (BFlag[30] == true) {
		isi_output_vectors[EXT30] = true;
		BFlag[30] = false;
	}

	/* Exit 31 : BLE App msgCommand이 1이상인 값들일 때(기기 세팅 및 설정) 안전을 위해 Standby State로 고정할 때 */
	if (BFlag[31] == true) {
		isi_output_vectors[EXT31] = true;
		BFlag[31] = false;
	}

	/* Exit 32 : BAT Voltage 5% 미만인 경우 */
	if (BFlag[32] == true) {
		isi_output_vectors[EXT32] = true;
		BFlag[32] = false;
	}

	/* Exit 33 : BAT Voltage 경고 띠링 소리 입구 */
	if (BFlag[33] == true) {
		isi_output_vectors[EXT33] = true;
		BFlag[33] = false;
	}

	/* Exit 34 : BAT Voltage 경고 띠링 소리 출구 */
	if (BFlag[34] == true) {
		isi_output_vectors[EXT34] = true;
		BFlag[34] = false;
	}

	/* Exit 35 : System Off */
	if (BFlag[35] == true) {
		isi_output_vectors[EXT35] = true;
		BFlag[35] = false;
	}

	if (BFlag[36] == true) {
		isi_output_vectors[EXT36] = true;
		BFlag[36] = false;
	}

	if (BFlag[37] == true) {
		isi_output_vectors[EXT37] = true;
		BFlag[37] = false;
	}

	if (BFlag[38] == true) {
		isi_output_vectors[EXT38] = true;
		BFlag[38] = false;
	}

	if (BFlag[39] == true) {
		isi_output_vectors[EXT39] = true;
		BFlag[39] = false;
	}

	if (BFlag[40] == true) {
		isi_output_vectors[EXT40] = true;
		BFlag[40] = false;
	}

	if (BFlag[41] == true) {
		isi_output_vectors[EXT41] = true;
		BFlag[41] = false;
	}

	if (BFlag[42] == true) {
		isi_output_vectors[EXT42] = true;
		BFlag[42] = false;
	}

	if (BFlag[43] == true) {
		isi_output_vectors[EXT43] = true;
		BFlag[43] = false;
	}

	if (BFlag[44] == true) {
		isi_output_vectors[EXT44] = true;
		BFlag[44] = false;
	}

	if (BFlag[45] == true) {
		isi_output_vectors[EXT45] = true;
		BFlag[45] = false;
	}

	if (BFlag[46] == true) {
		isi_output_vectors[EXT46] = true;
		BFlag[46] = false;
	}

	if (BFlag[47] == true) {
		isi_output_vectors[EXT47] = true;
		BFlag[47] = false;
	}

	if (BFlag[48] == true) {
		isi_output_vectors[EXT48] = true;
		BFlag[48] = false;
	}

	if (BFlag[49] == true) {
		isi_output_vectors[EXT49] = true;
		BFlag[49] = false;
	}

	if (BFlag[50] == true) {
		isi_output_vectors[EXT50] = true;
		BFlag[50] = false;
	}

	if (BFlag[51] == true) {
		isi_output_vectors[EXT51] = true;
		BFlag[51] = false;
	}

	if (BFlag[52] == true) {
		isi_output_vectors[EXT52] = true;
		BFlag[52] = false;
	}

	if (BFlag[53] == true) {
		isi_output_vectors[EXT53] = true;
		BFlag[53] = false;
	}

	if (BFlag[54] == true) {
		isi_output_vectors[EXT54] = true;
		BFlag[54] = false;
	}

	if (BFlag[55] == true) {
		isi_output_vectors[EXT55] = true;
		BFlag[55] = false;
	}

	if (BFlag[56] == true) {
		isi_output_vectors[EXT56] = true;
		BFlag[56] = false;
	}

	if (BFlag[57] == true) {
		isi_output_vectors[EXT57] = true;
		BFlag[57] = false;
	}

	if (BFlag[58] == true) {
		isi_output_vectors[EXT58] = true;
		BFlag[58] = false;
	}

	if (BFlag[59] == true) {
		isi_output_vectors[EXT59] = true;
		BFlag[59] = false;
	}

	if (BFlag[60] == true) {
		isi_output_vectors[EXT60] = true;
		BFlag[60] = false;
	}

	if (BFlag[61] == true) {
		isi_output_vectors[EXT61] = true;
		BFlag[61] = false;
	}

	if (BFlag[62] == true) {
		isi_output_vectors[EXT62] = true;
		BFlag[62] = false;
	}

	if (BFlag[63] == true) {
		isi_output_vectors[EXT63] = true;
		BFlag[63] = false;
	}

	// if (BFlag[64] == true) {
	// 	isi_output_vectors[EXT64] = true;
	// 	BFlag[64] = false;
	// }

	// if (BFlag[65] == true) {
	// 	isi_output_vectors[EXT65] = true;
	// 	BFlag[65] = false;
	// }

	// if (BFlag[66] == true) {
	// 	isi_output_vectors[EXT66] = true;
	// 	BFlag[66] = false;
	// }	
	
	// if (BFlag[67] == true) {
	// 	isi_output_vectors[EXT67] = true;
	// 	BFlag[67] = false;
	// }

	// if (BFlag[68] == true) {
	// 	isi_output_vectors[EXT68] = true;
	// 	BFlag[68] = false;
	// }	

	// if (BFlag[69] == true) {
	// 	isi_output_vectors[EXT69] = true;
	// 	BFlag[69] = false;
	// }

	if (BFlag[70] == true) {
		isi_output_vectors[EXT70] = true;
		BFlag[70] = false;
	}

	if (BFlag[71] == true) {
		isi_output_vectors[EXT71] = true;
		BFlag[71] = false;
	}

	if (BFlag[72] == true) {
		isi_output_vectors[EXT72] = true;
		BFlag[72] = false;
	}

	if (BFlag[73] == true) {
		isi_output_vectors[EXT73] = true;
		BFlag[73] = false;
	}

	if (BFlag[74] == true) {
		isi_output_vectors[EXT74] = true;
		BFlag[74] = false;
	}	
	
	if (BFlag[75] == true) {
		isi_output_vectors[EXT75] = true;
		BFlag[75] = false;
	}

	if (BFlag[76] == true) {
		isi_output_vectors[EXT76] = true;
		BFlag[76] = false;
	}	

	if (BFlag[77] == true) {
		isi_output_vectors[EXT77] = true;
		BFlag[77] = false;
	}

	if (BFlag[78] == true) {
		isi_output_vectors[EXT78] = true;
		BFlag[78] = false;
	}

	if (BFlag[79] == true) {
		isi_output_vectors[EXT79] = true;
		BFlag[79] = false;
	}

	if (BFlag[80] == true) {
		isi_output_vectors[EXT80] = true;
		BFlag[80] = false;
	}

	if (BFlag[81] == true) {
		isi_output_vectors[EXT81] = true;
		BFlag[81] = false;
	}

	if (BFlag[82] == true) {
		isi_output_vectors[EXT82] = true;
		BFlag[82] = false;
	}

	if (BFlag[83] == true) {
		isi_output_vectors[EXT83] = true;
		BFlag[83] = false;
	}

	if (BFlag[84] == true) {
		isi_output_vectors[EXT84] = true;
		BFlag[84] = false;
	}

	if (BFlag[85] == true) {
		isi_output_vectors[EXT85] = true;
		BFlag[85] = false;
	}

	if (BFlag[86] == true) {
		isi_output_vectors[EXT86] = true;
		BFlag[86] = false;
	}

	if (BFlag[87] == true) {
		isi_output_vectors[EXT87] = true;
		BFlag[87] = false;
	}

	if (BFlag[88] == true) {
		isi_output_vectors[EXT88] = true;
		BFlag[88] = false;
	}

	if (BFlag[89] == true) {
		isi_output_vectors[EXT89] = true;
		BFlag[89] = false;
	}

	if (BFlag[90] == true) {
		isi_output_vectors[EXT90] = true;
		BFlag[90] = false;
	}

	if (BFlag[91] == true) {
		isi_output_vectors[EXT91] = true;
		BFlag[91] = false;
	}

	if (BFlag[92] == true) {
		isi_output_vectors[EXT92] = true;
		BFlag[92] = false;
	}
	if (BFlag[93] == true) {
		isi_output_vectors[EXT92] = true;
		BFlag[93] = false;
	}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void Init_Bt(ISI_ButtonObj* button, IOIF_GPIOPort_t port, IOIF_GPIOPin_t pin, uint8_t active_type)
{
	button->GPIO_port  = port;
	button->GPIO_pin   = pin;
	button->active_type = active_type;
}

static void CheckMotionFlags_H10(MotionAnalysisConditions M_Flags)
{
	// 양 허벅지가 모두 hyper flexion 상태이고, 모두 동작이 작을 때
	// to Sit
	BFlag[1] = M_Flags.ifLHHyperFlexion2 && M_Flags.ifRHHyperFlexion2 && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary;

	// 양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 양 허벅지가 비슷한 각도일 때
	// to stand (still)
	BFlag[2] = M_Flags.ifRightAccXIsS && M_Flags.ifLeftAccXIsS && M_Flags.ifRightAccYIsS && M_Flags.ifLeftAccYIsS && (M_Flags.ifLHNeutral || M_Flags.ifLHExtension) && (M_Flags.ifRHNeutral || M_Flags.ifRHExtension) && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && (M_Flags.ifAbsHipAngleGapIsXS || M_Flags.ifAbsHipAngleGapIsS);

	// 양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 왼쪽 허벅지가 앞쪽에 있을 때
	// to LDS (still)
	BFlag[3] = M_Flags.ifLHNeutral && M_Flags.ifRHNeutral && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && (M_Flags.ifLeftHipAngleGapIsL || M_Flags.ifLeftHipAngleGapIsXL) && (!M_Flags.ifThighAngleSumIsXL && !M_Flags.ifThighAngleSumIsXXL);

	// 양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 오른쪽 허벅지가 앞쪽에 있을 때
	// to RDS (still)
	BFlag[4] = M_Flags.ifLHNeutral && M_Flags.ifRHNeutral && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && (M_Flags.ifRightHipAngleGapIsL || M_Flags.ifRightHipAngleGapIsXL) && (!M_Flags.ifThighAngleSumIsXL && !M_Flags.ifThighAngleSumIsXXL);

	// 양 허벅지가 모두 flexion 상태이고, 모두 flexion 방향으로 동작 중이며, 양 허벅지가 비슷한 각도일 때
	// to sitting
//	BFlag[5] = M_Flags.ifLHFlexion && M_Flags.ifRHFlexion && M_Flags.ifLHFlexionIsOn && M_Flags.ifRHFlexionIsOn && (M_Flags.ifAbsHipAngleGapIsS || M_Flags.ifAbsHipAngleGapIsXS);

	// 양 허벅지가 모두 flexion 상태이고, 모두 extension 방향으로 동작 중이며, 양 허벅지가 비슷한 각도일 때
	// to standing
//	BFlag[6] = M_Flags.ifLHFlexion && M_Flags.ifRHFlexion && M_Flags.ifLHExtensionIsOn && M_Flags.ifRHExtensionIsOn && (M_Flags.ifAbsHipAngleGapIsS || M_Flags.ifAbsHipAngleGapIsXS);

	// stand to Left Thigh Lifting
	// BFlag[7] = (M_Flags.ifLHNeutral || M_Flags.ifLHExtension) && (M_Flags.ifRHNeutral || M_Flags.ifRHExtension) && M_Flags.ifLHFlexionIsFast && (!M_Flags.ifRHFlexionIsFast) && (M_Flags.ifRHIsStationary || M_Flags.ifRHExtensionIsFast) && (!M_Flags.ifRightHipAngleGapIsXL && !M_Flags.ifRightHipAngleGapIsXXL);
	BFlag[7] = (M_Flags.ifLHNeutral || M_Flags.ifLHExtension) && (!M_Flags.ifRHHyperFlexion2) && M_Flags.ifLHFlexionIsFast && (!M_Flags.ifRightHipAngleGapIsS &&!M_Flags.ifRightHipAngleGapIsL &&!M_Flags.ifRightHipAngleGapIsXL && !M_Flags.ifRightHipAngleGapIsXXL && !M_Flags.ifRightHipAngleGapIsXXXL) && (!M_Flags.ifsumsumYMinus || DisableStairWalkingDetection);

	 // stand to Right Thigh Lifting
	 // BFlag[8] = (M_Flags.ifLHNeutral || M_Flags.ifLHExtension) && (M_Flags.ifRHNeutral || M_Flags.ifRHExtension) && M_Flags.ifRHFlexionIsFast && (!M_Flags.ifLHFlexionIsFast) && (M_Flags.ifLHIsStationary || M_Flags.ifLHExtensionIsFast) && (!M_Flags.ifLeftHipAngleGapIsXL && !M_Flags.ifLeftHipAngleGapIsXXL);
	BFlag[8] = (M_Flags.ifRHNeutral || M_Flags.ifRHExtension) && (!M_Flags.ifLHHyperFlexion2) && M_Flags.ifRHFlexionIsFast && (!M_Flags.ifLeftHipAngleGapIsS && !M_Flags.ifLeftHipAngleGapIsL &&!M_Flags.ifLeftHipAngleGapIsXL && !M_Flags.ifLeftHipAngleGapIsXXL && !M_Flags.ifLeftHipAngleGapIsXXXL) && (!M_Flags.ifsumsumYMinus || DisableStairWalkingDetection);

	 // to Left Thigh Down (slow)
	BFlag[9] = M_Flags.ifLHExtensionIsFast;

	 // to Right Thigh Down (slow)
	BFlag[10] = M_Flags.ifRHExtensionIsFast;

	 // 오른쪽 허벅지가 왼쪽 보다 앞에 있고, 왼쪽 허벅지가 flexion 방향으로 빠르게 동작 중일 때
	 // RDS to LSW
	 // BFlag[11] = (!M_Flags.ifRHHyperFlexion && !M_Flags.ifRHHyperFlexion2) && M_Flags.ifLHFlexionIsFast && (M_Flags.ifRightHipAngleGapIsXL || M_Flags.ifRightHipAngleGapIsXXL || M_Flags.ifRightHipAngleGapIsXXXL);
	BFlag[11] = (M_Flags.ifLHNeutral2 || M_Flags.ifLHExtension) && (!M_Flags.ifRHHyperFlexion2) && M_Flags.ifLHFlexionIsFast && (M_Flags.ifRightHipAngleGapIsS || M_Flags.ifRightHipAngleGapIsL || M_Flags.ifRightHipAngleGapIsXL || M_Flags.ifRightHipAngleGapIsXXL || M_Flags.ifRightHipAngleGapIsXXXL) && (!M_Flags.ifsumsumYMinus || DisableStairWalkingDetection) && (!M_Flags.ifsumsumYPlus || DisableStairWalkingDetection);

	 // 왼쪽 허벅지가 오른쪽 보다 앞에 있고, 오른쪽 허벅지가 flexion 방향으로 빠르게 동작 중일 때
	 // LDS to RSW
	 // BFlag[12] = (!M_Flags.ifLHHyperFlexion && !M_Flags.ifLHHyperFlexion2) && M_Flags.ifRHFlexionIsFast && (M_Flags.ifLeftHipAngleGapIsXL || M_Flags.ifLeftHipAngleGapIsXXL || M_Flags.ifLeftHipAngleGapIsXXXL);
	BFlag[12] = (M_Flags.ifRHNeutral2 || M_Flags.ifRHExtension) && (!M_Flags.ifLHHyperFlexion2) && M_Flags.ifRHFlexionIsFast && (M_Flags.ifLeftHipAngleGapIsS || M_Flags.ifLeftHipAngleGapIsL || M_Flags.ifLeftHipAngleGapIsXL || M_Flags.ifLeftHipAngleGapIsXXL || M_Flags.ifLeftHipAngleGapIsXXXL) && (!M_Flags.ifsumsumYMinus || DisableStairWalkingDetection) && (!M_Flags.ifsumsumYPlus || DisableStairWalkingDetection);

	 // (LSW, L lifting) to LSW final
	BFlag[13] = M_Flags.ifLHExtensionIsFast;

	 // (RSW, R lifting) to RSW final
	BFlag[14] = M_Flags.ifRHExtensionIsFast;

	 // to ASTR_LDS (계단오르기)
	// BFlag[36] = M_Flags.ifLHNeutral && (M_Flags.ifLHHyperFlexion || M_Flags.ifLHHyperFlexion2) && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && (M_Flags.ifLeftHipAngleGapIsL || M_Flags.ifLeftHipAngleGapIsXL) && (M_Flags.ifThighAngleSumIsXL || M_Flags.ifThighAngleSumIsXXL) && !DisableStairWalkingDetection;
	BFlag[36] = M_Flags.ifLHExtensionIsFast && !DisableStairWalkingDetection;

	 // to ASTR_RDS (계단오르기)
	// BFlag[37] = M_Flags.ifRHNeutral && (M_Flags.ifRHHyperFlexion || M_Flags.ifRHHyperFlexion2) && M_Flags.ifRHIsStationary && M_Flags.ifLHIsStationary && (M_Flags.ifRightHipAngleGapIsL || M_Flags.ifRightHipAngleGapIsXL) && (M_Flags.ifThighAngleSumIsXL || M_Flags.ifThighAngleSumIsXXL) && !DisableStairWalkingDetection;
	BFlag[37] = M_Flags.ifRHExtensionIsFast && !DisableStairWalkingDetection;

	 // to ASTR_LSW (계단오르기)
	 // BFlag[38] = (M_Flags.ifRHHyperFlexion || M_Flags.ifRHHyperFlexion2) && M_Flags.ifLHFlexionIsFast && (M_Flags.ifRightHipAngleGapIsXXL || M_Flags.ifRightHipAngleGapIsXXXL);
	BFlag[38] = (M_Flags.ifLHNeutral || M_Flags.ifLHExtension) && (M_Flags.ifRHHyperFlexion || M_Flags.ifRHHyperFlexion2) && M_Flags.ifLHFlexionIsFast && (M_Flags.ifRightHipAngleGapIsS || M_Flags.ifRightHipAngleGapIsL || M_Flags.ifRightHipAngleGapIsXL || M_Flags.ifRightHipAngleGapIsXXL || M_Flags.ifRightHipAngleGapIsXXXL) && M_Flags.ifsumsumYPlus && !DisableStairWalkingDetection;

	 // to ASTR_RSW (계단오르기)
	 // BFlag[39] = (M_Flags.ifLHHyperFlexion || M_Flags.ifLHHyperFlexion2) && M_Flags.ifRHFlexionIsFast && (M_Flags.ifLeftHipAngleGapIsXXL || M_Flags.ifLeftHipAngleGapIsXXXL);
	BFlag[39] = (M_Flags.ifRHNeutral || M_Flags.ifRHExtension) && (M_Flags.ifLHHyperFlexion || M_Flags.ifLHHyperFlexion2) && M_Flags.ifRHFlexionIsFast && (M_Flags.ifLeftHipAngleGapIsS || M_Flags.ifLeftHipAngleGapIsL || M_Flags.ifLeftHipAngleGapIsXL || M_Flags.ifLeftHipAngleGapIsXXL || M_Flags.ifLeftHipAngleGapIsXXXL) && M_Flags.ifsumsumYPlus && !DisableStairWalkingDetection;

	 // to DSTR_L-Lowering (계단내려가기)
	 // BFlag[40] = (M_Flags.ifLHHyperFlexion || M_Flags.ifLHHyperFlexion2) && (M_Flags.ifRHFlexion || M_Flags.ifRHHyperFlexion) && M_Flags.ifLHFlexionIsOn2 && M_Flags.ifRHFlexionIsOn2 && (M_Flags.ifLeftHipAngleGapIsS || M_Flags.ifLeftHipAngleGapIsL || M_Flags.ifLeftHipAngleGapIsXL);
	BFlag[40] = M_Flags.ifLHExtensionIsFast && !DisableStairWalkingDetection;

	 // to DSTR_R-Lowering (계단내려가기)
	 // BFlag[41] = (M_Flags.ifRHHyperFlexion || M_Flags.ifRHHyperFlexion2) && (M_Flags.ifLHFlexion || M_Flags.ifLHHyperFlexion) && M_Flags.ifRHFlexionIsOn2 && M_Flags.ifLHFlexionIsOn2 && (M_Flags.ifRightHipAngleGapIsS || M_Flags.ifRightHipAngleGapIsL || M_Flags.ifRightHipAngleGapIsXL);
	BFlag[41] = M_Flags.ifRHExtensionIsFast && !DisableStairWalkingDetection;

	 // to DSTR_LSW (계단내려가기)
	 // BFlag[42] = M_Flags.ifLeftHipAngleGapIsOn;
	BFlag[42] = M_Flags.ifsumsumYMinus && M_Flags.ifLHFlexionIsFast && !DisableStairWalkingDetection;

	 // to DSTR_RSW (계단내려가기)
	 // BFlag[43] = M_Flags.ifRightHipAngleGapIsOn;
	BFlag[43] = M_Flags.ifsumsumYMinus && M_Flags.ifRHFlexionIsFast && !DisableStairWalkingDetection;

	 // 첫발 Left Lifting
	 // BFlag[44] = M_Flags.ifLHFlexion && M_Flags.ifRHNeutral && M_Flags.ifLHFlexionIsFast && (!M_Flags.ifRHFlexionIsFast);
	BFlag[44] = (M_Flags.ifLHFlexion || M_Flags.ifLHHyperFlexion) && M_Flags.ifRHNeutral && M_Flags.ifLHFlexionIsFast && (!M_Flags.ifRHFlexionIsFast) && (!M_Flags.ifsumsumYMinus || DisableStairWalkingDetection);

	 // 첫발 Right Lifting
	 // BFlag[45] = M_Flags.ifRHFlexion && M_Flags.ifLHNeutral && M_Flags.ifRHFlexionIsFast && (!M_Flags.ifLHFlexionIsFast);
	BFlag[45] = (M_Flags.ifRHFlexion || M_Flags.ifRHHyperFlexion) && M_Flags.ifLHNeutral && M_Flags.ifRHFlexionIsFast && (!M_Flags.ifLHFlexionIsFast) && (!M_Flags.ifsumsumYMinus || DisableStairWalkingDetection);

	 // Left Lifting Large
	// BFlag[46] = M_Flags.ifLeftHipAngleGapIsXXXL;

	 // Right Lifting Large
	// BFlag[47] = M_Flags.ifRightHipAngleGapIsXXXL;

	 // to sitting 2
	// BFlag[48] = M_Flags.ifLHHyperFlexion && M_Flags.ifRHHyperFlexion && M_Flags.ifLHFlexionIsOn && M_Flags.ifRHFlexionIsOn && (M_Flags.ifAbsHipAngleGapIsXS || M_Flags.ifAbsHipAngleGapIsS);

	 // to crouch Left Lifting
	 // BFlag[49] = (M_Flags.ifLHFlexion) && (M_Flags.ifRHNeutral || M_Flags.ifRHExtension || M_Flags.ifRHFlexion || M_Flags.ifRHHyperFlexion) && M_Flags.ifLHFlexionIsFast && (!M_Flags.ifRHFlexionIsFast) && (M_Flags.ifRHIsStationary || M_Flags.ifRHExtensionIsFast) && (!M_Flags.ifRightHipAngleGapIsXL && !M_Flags.ifRightHipAngleGapIsXXL);
	BFlag[49] = (M_Flags.ifLHFlexion) && (M_Flags.ifRHNeutral || M_Flags.ifRHExtension || M_Flags.ifRHFlexion || M_Flags.ifRHHyperFlexion) && M_Flags.ifLHFlexionIsFast && (!M_Flags.ifRightHipAngleGapIsS &&!M_Flags.ifRightHipAngleGapIsL &&!M_Flags.ifRightHipAngleGapIsXL && !M_Flags.ifRightHipAngleGapIsXXL && !M_Flags.ifRightHipAngleGapIsXXXL) && (!M_Flags.ifsumsumYMinus || DisableStairWalkingDetection);

	 // to crouch Right Lifting
	 // BFlag[50] = (M_Flags.ifLHNeutral || M_Flags.ifLHExtension || M_Flags.ifLHFlexion || M_Flags.ifLHHyperFlexion) && (M_Flags.ifRHFlexion) && M_Flags.ifRHFlexionIsFast && (!M_Flags.ifLHFlexionIsFast) && (M_Flags.ifLHIsStationary || M_Flags.ifLHExtensionIsFast) && (!M_Flags.ifLeftHipAngleGapIsXL && !M_Flags.ifLeftHipAngleGapIsXXL);
	BFlag[50] = (M_Flags.ifRHFlexion) && (M_Flags.ifLHNeutral || M_Flags.ifLHExtension || M_Flags.ifLHFlexion || M_Flags.ifLHHyperFlexion) && M_Flags.ifRHFlexionIsFast && (!M_Flags.ifLeftHipAngleGapIsS && !M_Flags.ifLeftHipAngleGapIsL &&!M_Flags.ifLeftHipAngleGapIsXL && !M_Flags.ifLeftHipAngleGapIsXXL && !M_Flags.ifLeftHipAngleGapIsXXXL) && (!M_Flags.ifsumsumYMinus || DisableStairWalkingDetection);

	/* 앉기 서기 보조 모드 */
	// BFlag[70]은 STS Mode 전환

	// 프로앱 자세유지버튼 on 상태
	BFlag[71] = M_Flags.ifProApp_PostureHoldOn;
	// 프로앱 자세유지버튼 off 상태(default)
	BFlag[72] = M_Flags.ifProApp_PostureHoldOff;
	// 프로앱 앉기 준비 버튼 On
	BFlag[73] = M_Flags.ifProApp_SitReady;
	// 프로앱 앉기 준비 취소 버튼 On
	BFlag[74] = M_Flags.ifProApp_SitReadyCancel;
	// 프로앱 앉기 시작 버튼 on
	BFlag[75] = M_Flags.ifProApp_SitStart;
	// 프로앱 서기 준비 버튼 on
	BFlag[76] = M_Flags.ifProApp_StandReady;
	// 프로앱 서기 준비 취소 버튼 on
	BFlag[77] = M_Flags.ifProApp_StandReadyCancel;
	// 프로앱 서기 시작 버튼 on
	BFlag[78] = M_Flags.ifProApp_StandStart;

	// [pSTS] p_Sitting_Start to p_Sitting
	BFlag[79] = M_Flags.ifLHHyperFlexion3 && M_Flags.ifRHHyperFlexion3;

	// [pSTS] p_Sitting to p_Sit
	// BFlag[80] = (IIOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_8) == IOIF_GPIO_PIN_RESET) || (M_Flags.ifLHHyperFlexion4 && M_Flags.ifRHHyperFlexion4 && M_Flags.ifLHExtensionIsOn && M_Flags.ifRHExtensionIsOn) || (M_Flags.ifLHHyperFlexion5 && M_Flags.ifRHHyperFlexion5 && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary);	
	BFlag[80] = (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_8) == IOIF_GPIO_PIN_RESET) || (M_Flags.ifLHHyperFlexion4 && M_Flags.ifRHHyperFlexion4); //TODO: 전원 버튼으로 앉기 완료되는 것은 태블릿 업데이트때 태블릿에 앉기 완료 버튼으로 대체

	// [pSTS] 보조없이 자세변경 to p_Sit
	BFlag[81] = M_Flags.ifLHHyperFlexion4 && M_Flags.ifRHHyperFlexion4 && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary;

	// [pSTS] to p_Standing
	BFlag[82] = ((M_Flags.ifLHNeutral && M_Flags.ifRHNeutral) || M_Flags.ifStandingStart) && M_Flags.ifProApp_PostureHoldOff;

	// [pSTS] to p_Standing Hold
	BFlag[83] = ((M_Flags.ifLHNeutral && M_Flags.ifRHNeutral) || M_Flags.ifStandingStart) && M_Flags.ifProApp_PostureHoldOn;

	// [pSTS] assist to p_Stand
	BFlag[84] = (!M_Flags.ifLHHyperFlexion3) && (!M_Flags.ifRHHyperFlexion3)  && M_Flags.ifProApp_PostureHoldOff;

	// [pSTS] assist to p_Stand Hold
	BFlag[85] = (M_Flags.ifLHNeutral) && (M_Flags.ifRHNeutral)  && M_Flags.ifProApp_PostureHoldOn;

	// [pSTS] 강제 자세변경 to p_Stand
	BFlag[86] = BFlag[2] && M_Flags.ifProApp_PostureHoldOff;   // 나중에 바뀔지도 몰라서 자리만..

	// [pSTS] assist 후 앉지 못해 to p_Sitting_off
	BFlag[87] = (!M_Flags.ifLHHyperFlexion5) && (!M_Flags.ifRHHyperFlexion5) ;

	// [pSTS] to p_Standing_off
	BFlag[88] = (!M_Flags.ifLHHyperFlexion5) && (!M_Flags.ifRHHyperFlexion5);

	// [pSTS] to p_LDS
	BFlag[89] = BFlag[3];

	// [pSTS] to p_RDS
	BFlag[90] = BFlag[4];

	// [pSTS] to p_Astairs LDS
	BFlag[91] = BFlag[36];
	
	// [pSTS] to p_Astairs RDS
	BFlag[92] = BFlag[37];

	// [pSTS] 강제 자세변경 to p_Stand_hold
	BFlag[93] = BFlag[2] && M_Flags.ifProApp_PostureHoldOn;   // 나중에 바뀔지도 몰라서 자리만..
}




static void CheckMotionFlags_K10(MotionAnalysisConditions M_Flags)
{
	// 양 허벅지가 모두 hyper flexion 상태이고, 모두 동작이 작을 때
	// to Sit
	BFlag[1] = M_Flags.ifLHHyperFlexion2 && M_Flags.ifRHHyperFlexion2 && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && (M_Flags.ifAbsHipAngleGapIsS || M_Flags.ifAbsHipAngleGapIsXS);

	// 양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 양 허벅지가 비슷한 각도일 때
	// to stand (still)
	BFlag[2] = (M_Flags.ifLKNeutral || M_Flags.ifLKFlexion) && (M_Flags.ifRKNeutral || M_Flags.ifRKFlexion) && (M_Flags.ifLHNeutral || M_Flags.ifLHExtension) && (M_Flags.ifRHNeutral || M_Flags.ifRHExtension) && M_Flags.ifLKIsStationary && M_Flags.ifRKIsStationary && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && (M_Flags.ifAbsHipAngleGapIsXS || M_Flags.ifAbsHipAngleGapIsS);

	// 양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 왼쪽 허벅지가 앞쪽에 있을 때
	// to LDS (still)
	BFlag[3] = (M_Flags.ifLKNeutral || M_Flags.ifLKFlexion) && (M_Flags.ifRKNeutral || M_Flags.ifRKFlexion) && (M_Flags.ifLHNeutral || M_Flags.ifLHExtension) && (M_Flags.ifRHNeutral || M_Flags.ifRHExtension) && M_Flags.ifLKIsStationary && M_Flags.ifRKIsStationary && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && M_Flags.ifLeftHipAngleGapIsL;

	// 양 허벅지가 모두 neutral 상태이고, 모두 동작이 작으며, 오른쪽 허벅지가 앞쪽에 있을 때
	// to RDS (still)
	BFlag[4] = (M_Flags.ifLKNeutral || M_Flags.ifLKFlexion) && (M_Flags.ifRKNeutral || M_Flags.ifRKFlexion) && (M_Flags.ifLHNeutral || M_Flags.ifLHExtension) && (M_Flags.ifRHNeutral || M_Flags.ifRHExtension) && M_Flags.ifLKIsStationary && M_Flags.ifRKIsStationary && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && M_Flags.ifRightHipAngleGapIsL;

	// 양 허벅지가 모두 flexion 상태이고, 모두 flexion 방향으로 동작 중이며, 양 허벅지가 비슷한 각도일 때
	// to sitting
	BFlag[5] = (M_Flags.ifLKHyperFlexion || M_Flags.ifLKHyperFlexion2) && (M_Flags.ifRKHyperFlexion || M_Flags.ifRKHyperFlexion2) && (M_Flags.ifLHHyperFlexion || M_Flags.ifLHHyperFlexion2) && (M_Flags.ifRHHyperFlexion || M_Flags.ifRHHyperFlexion2) && M_Flags.ifLKFlexionIsFast && M_Flags.ifRKFlexionIsFast;

	// 양 허벅지가 모두 flexion 상태이고, 모두 extension 방향으로 동작 중이며, 양 허벅지가 비슷한 각도일 때
	// to standing
	BFlag[6] = (M_Flags.ifLKHyperFlexion || M_Flags.ifLKHyperFlexion2) && (M_Flags.ifRKHyperFlexion || M_Flags.ifRKHyperFlexion2) && (M_Flags.ifLHHyperFlexion || M_Flags.ifLHHyperFlexion2) && (M_Flags.ifRHHyperFlexion || M_Flags.ifRHHyperFlexion2) && M_Flags.ifLKExtensionIsFast && M_Flags.ifRKExtensionIsFast;

	// to Left Leg Lifting
	BFlag[7] = (M_Flags.ifRKNeutral || M_Flags.ifRKFlexion || M_Flags.ifRKHyperFlexion) && (!M_Flags.ifRKFlexionIsFast) && M_Flags.ifLKFlexionIsFast && M_Flags.ifLHFlexionIsFast && (M_Flags.ifLeftHipAngleGapIsS || M_Flags.ifLeftHipAngleGapIsL) && (!M_Flags.ifThighAngleSumIsL);

	// to Right Leg Lifting
	BFlag[8] = (M_Flags.ifLKNeutral || M_Flags.ifLKFlexion || M_Flags.ifLKHyperFlexion) && (!M_Flags.ifLKFlexionIsFast) && M_Flags.ifRKFlexionIsFast && M_Flags.ifRHFlexionIsFast && (M_Flags.ifRightHipAngleGapIsS || M_Flags.ifRightHipAngleGapIsL) && (!M_Flags.ifThighAngleSumIsL);

	// to Left Leg Lowering
	BFlag[9] = M_Flags.ifLKExtensionIsOn && M_Flags.ifLeftHipAngleGapIsOn;

	// to Right Leg Down
	BFlag[10] = M_Flags.ifRKExtensionIsOn && M_Flags.ifRightHipAngleGapIsOn;

	// to Left Swing (LSW) Mid
	BFlag[11] = M_Flags.ifLHFlexionIsFast && M_Flags.ifLeftHipAngleGapIsOn;

	// to Right Swing (RSW) Mid
	BFlag[12] = M_Flags.ifRHFlexionIsFast && M_Flags.ifRightHipAngleGapIsOn;

	// to Left Swing (LSW) Terminal
	BFlag[13] = M_Flags.ifLHExtensionIsFast && M_Flags.ifLeftHipAngleGapIsOn;

	// to Right Swing (RSW) Terminal
	BFlag[14] = M_Flags.ifRHExtensionIsFast && M_Flags.ifRightHipAngleGapIsOn;

	// to ASTR Left Swing Terminal (계단오르기)
	BFlag[36] = M_Flags.ifLHExtensionIsFast;

	// to ASTR Right Swing Terminal (계단오르기)
	BFlag[37] = M_Flags.ifRHExtensionIsFast;

	// to ASTR Left Swing (LSW) (계단오르기)
	BFlag[38] = M_Flags.ifRKHyperFlexion2  && M_Flags.ifLKFlexionIsFast && M_Flags.ifLHFlexionIsFast && M_Flags.ifRightHipAngleGapIsOn;

	// to ASTR Right Swing (RSW) (계단오르기)
	BFlag[39] = M_Flags.ifLKHyperFlexion2 && M_Flags.ifRKFlexionIsFast && M_Flags.ifRHFlexionIsFast && M_Flags.ifLeftHipAngleGapIsOn;

	// to DSTR Left Lowering (계단내려가기)
	BFlag[40] = M_Flags.ifRKFlexionIsFast && (M_Flags.ifRHFlexion || M_Flags.ifRHHyperFlexion) && (M_Flags.ifLHFlexion || M_Flags.ifLHHyperFlexion) && M_Flags.ifRightHipAngleGapIsOn && M_Flags.ifThighAngleSumIsL;

	// to DSTR Right Lowering (계단내려가기)
	BFlag[41] = M_Flags.ifLKFlexionIsFast && (M_Flags.ifRHFlexion || M_Flags.ifRHHyperFlexion) && (M_Flags.ifLHFlexion || M_Flags.ifLHHyperFlexion) && M_Flags.ifLeftHipAngleGapIsOn && M_Flags.ifThighAngleSumIsL;

	// to DSTR Left Swing (LSW) (계단내려가기)
	BFlag[42] = M_Flags.ifLKExtensionIsOn;

	// to DSTR Right Swing (RSW) (계단내려가기)
	BFlag[43] = M_Flags.ifRKExtensionIsOn;

	// 첫발 Left Lifting
	BFlag[44] = false;

	// 첫발 Right Lifting
	BFlag[45] = false;

	// Left Lifting Large
	BFlag[46] = false;

	// Right Lifting Large
	BFlag[47] = false;

	// to sitting 2
	BFlag[48] = false;

	// to Left Swing Initial
	BFlag[49] = (!M_Flags.ifRKHyperFlexion2) && M_Flags.ifLKFlexionIsFast && M_Flags.ifLHExtension && M_Flags.ifLHFlexionIsFast && (M_Flags.ifRightHipAngleGapIsS || M_Flags.ifRightHipAngleGapIsL);

	// to Right Swing Initial
	BFlag[50] = (!M_Flags.ifLKHyperFlexion2) && M_Flags.ifRKFlexionIsFast && M_Flags.ifRHExtension && M_Flags.ifRHFlexionIsFast && (M_Flags.ifLeftHipAngleGapIsS || M_Flags.ifLeftHipAngleGapIsL);

	// to ASTAIRS LDS (still)
	BFlag[51] = M_Flags.ifLKHyperFlexion2 && M_Flags.ifLHHyperFlexion2 && M_Flags.ifLKIsStationary && M_Flags.ifRKIsStationary && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && M_Flags.ifLeftHipAngleGapIsL && M_Flags.ifLeftAccXIsS && M_Flags.ifRightAccXIsS;

	// to ASTAIRS RDS (still)
	BFlag[52] = M_Flags.ifRKHyperFlexion2 && M_Flags.ifRHHyperFlexion2 && M_Flags.ifLKIsStationary && M_Flags.ifRKIsStationary && M_Flags.ifLHIsStationary && M_Flags.ifRHIsStationary && M_Flags.ifRightHipAngleGapIsL && M_Flags.ifLeftAccXIsS && M_Flags.ifRightAccXIsS;

	// ASTR_LDS/RDS to Stand
	BFlag[53] = M_Flags.ifAbsHipAngleGapIsS || M_Flags.ifAbsHipAngleGapIsXS;
}

#endif /* SUIT_MINICM_ENABLED */
