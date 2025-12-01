
#include "robot_FSM.h"

#include "ext_link_mngr.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

FSM_Mngr    FSMMngrObj;
FSMFileInfo FSM_File;
FSMFileInfo FSM_File_Read;
/*
FSMFile FSM1_File =
{
		{{STATE_INIT},        {MS_NONE},      {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_STAND}                   },
		{{STATE_STAND},       {MS_STAND},     {BEEP3}, {0}, {0},     {},            {EXT1},       {STATE_SQUAT}                   },
		{{STATE_SQUAT},       {MS_SQUAT},     {BEEP4}, {0}, {0},     {},            {EXT1},       {STATE_AIR_SIT}                 },
		{{STATE_AIR_SIT},     {MS_AIR_SIT},   {BEEP4}, {0}, {0},     {},            {EXT1},       {STATE_SIT}                     },
		{{STATE_SIT},         {MS_SIT},       {BEEP4}, {0}, {0},     {},            {EXT1},       {STATE_AIR_SIT2}                },
		{{STATE_AIR_SIT2},    {MS_AIR_SIT},   {BEEP5}, {0}, {0},     {},            {EXT1},       {STATE_SIT2STAND}               },
		{{STATE_SIT2STAND},   {MS_SIT2STAND}, {BEEP5}, {0}, {0},     {},            {EXT1},       {STATE_STAND2SIT}               },
		{{STATE_STAND2SIT},   {MS_STAND2SIT}, {BEEP5}, {0}, {0},     {},            {EXT1},       {STATE_STAND}                   },
};

FSMFile FSM2_File =
{
		{{STATE_INIT},         {MS_NONE},         {BEEP1}, {0}, {0},     {},            {EXT4},       {STATE_DANCE},                     },
		{{STATE_DANCE},        {MS_DANCE},        {BEEP1}, {0}, {15000}, {STATE_STAND}, {},           {},                                },
		{{STATE_STAND},        {MS_STAND},        {BEEP3}, {0}, {0},     {},            {EXT4, EXT5}, {STATE_SQUAT, STATE_L_HALF_SWING}, },
		{{STATE_L_HALF_SWING}, {MS_L_HALF_SWING}, {BEEP2}, {0}, {0},     {},            {EXT5},       {STATE_R_FULL_SWING},              },
		{{STATE_R_FULL_SWING}, {MS_R_FULL_SWING}, {BEEP2}, {0}, {0},     {},            {EXT4, EXT5}, {STATE_STAND, STATE_L_FULL_SWING}, },
		{{STATE_L_FULL_SWING}, {MS_L_FULL_SWING}, {BEEP2}, {0}, {0},     {},            {EXT4, EXT5}, {STATE_STAND, STATE_R_FULL_SWING}, },
		{{STATE_SQUAT},        {MS_SQUAT},        {BEEP4}, {0}, {0},     {},            {EXT4, EXT5}, {STATE_AIR_SIT, STATE_STAND},      },
		{{STATE_AIR_SIT},      {MS_AIR_SIT},      {BEEP4}, {0}, {0},     {},            {EXT4, EXT5}, {STATE_SIT, STATE_SQUAT},          },
		{{STATE_SIT},          {MS_SIT},          {BEEP5}, {0}, {0},     {},            {EXT5},       {STATE_AIR_SIT},                   },
};


FSMFile FSM3_File =
{
		{{STATE_INIT},         {MS_NONE},         {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_KNEE_TEST}},
		{{STATE_KNEE_TEST},    {MS_KNEE_TEST},    {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_INIT}},

};

FSMFile FSM4_File =
{
		{{STATE_INIT},          {MS_NONE},          {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_L_FOOT_UP}},
		{{STATE_L_FOOT_UP},     {MS_L_FOOT_UP},     {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_STAND}},
		{{STATE_STAND},         {MS_STAND},         {BEEP1}, {0}, {0},     {},            {EXT1, EXT2}, {STATE_L_FOOT_UP, STATE_R_FOOT_UP}},
		{{STATE_R_FOOT_UP},     {MS_R_FOOT_UP},     {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_STAND}},
};

FSMFile FSM5_File =
{
		{{STATE_INIT},          {MS_NONE},          {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_L_TILT}},
		{{STATE_L_TILT},        {MS_L_TILT},        {BEEP1}, {0}, {0},     {},            {EXT1, EXT2}, {STATE_R_FOOT_UP, STATE_STAND}},
		{{STATE_R_FOOT_UP},     {MS_R_FOOT_UP},     {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_L_TILT}},
		{{STATE_STAND},         {MS_STAND},         {BEEP1}, {0}, {0},     {},            {EXT1, EXT2}, {STATE_R_TILT, STATE_L_TILT}},
		{{STATE_R_TILT},        {MS_R_TILT},        {BEEP1}, {0}, {0},     {},            {EXT1, EXT2}, {STATE_L_FOOT_UP, STATE_STAND}},
		{{STATE_L_FOOT_UP},     {MS_L_FOOT_UP},     {BEEP1}, {0}, {0},     {},            {EXT1},       {STATE_R_TILT}},
};*/

FSMFile FSM_File_AS_Test =
{
		{{STATE_INIT},		{MS_NONE},      {1}, {1}, {0},	{3000},	{},								{STATE_STANDBY}},
		{{STATE_STANDBY},	{MS_NONE},		{2}, {2}, {0},	{2000},	{},								{STATE_ASSIST}},
		{{STATE_IDLE},		{MS_NONE},		{2}, {3}, {0},	{},		{EXT1},							{STATE_ASSIST}},
		{{STATE_ASSIST},	{MS_NONE},		{3}, {4}, {0},	{},		{EXT1, EXT2},					{STATE_IDLE, STATE_STAND}},
		{{STATE_STAND},		{MS_NONE},		{3}, {0}, {0},	{},		{EXT1, EXT3, EXT4, EXT5, EXT6},	{STATE_IDLE, STATE_RHS, STATE_LHS, STATE_RTS, STATE_LTS}},
		{{STATE_RHS},		{MS_RHS_LTS},   {3}, {0}, {0},	{},		{EXT1, EXT2, EXT4, EXT5, EXT6},	{STATE_IDLE, STATE_STAND, STATE_LHS, STATE_RTS, STATE_LTS}},
		{{STATE_LHS},		{MS_LHS_RTS},	{3}, {0}, {0},	{},		{EXT1, EXT2, EXT3, EXT5, EXT6},	{STATE_IDLE, STATE_STAND, STATE_RHS, STATE_RTS, STATE_LTS}},
		{{STATE_RTS},		{MS_RL_SWING},	{3}, {0}, {0},	{},		{EXT1, EXT2, EXT3, EXT4, EXT6},	{STATE_IDLE, STATE_STAND, STATE_RHS, STATE_LHS, STATE_LTS}},
		{{STATE_LTS},		{MS_LR_SWING},	{3}, {0}, {0},	{},		{EXT1, EXT2, EXT3, EXT4, EXT5},	{STATE_IDLE, STATE_STAND, STATE_RHS, STATE_LHS, STATE_RTS}},
};


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

extern ExtLinkMngrInst_t g_ext_link_mngr;


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

/************************* (2023-12-01) *************************/
void Make_FSM1_Examples(void)
{
	FSM_File.robot_id =5;
	FSM_File.file_version_major = 10;

	for (int i = 0 ; i < MAX_N_STATE ; i++) {
		uint8_t actID[4];
		for (int j = 0 ; j < 4 ; j++) {
			actID[j] = i+j;
		}

		FSM_File.vec[i].StateID = i+5;
		FSM_File.vec[i].MotionSetID = i+6;
		FSM_File.vec[i].ActionID = (actID[0]<<24) + (actID[1]<<16) + (actID[2]<<8) + (actID[3]);
		FSM_File.vec[i].TabletModeID = i+7;
		FSM_File.vec[i].TimeOut = i+8;
		FSM_File.vec[i].DefaultTargetStateID = i+9;
		for (int k = 0 ; k < MAX_N_TRANSITION ; k++) {
			FSM_File.vec[i].ExitConditionID[k] = k;
			FSM_File.vec[i].TargetStateID[k]   = k;
		}
	}
}

void Save_FSM1(void)
{
	uint32_t writeAddr = 0;

	IOIF_EraseFlash(IOIF_FLASH_START_FSM_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_START_FSM_ADDR;

	/* (1) Save ID & Version */
	uint32_t memArr[4] = {0};
	memcpy(&memArr[0], &FSM_File.robot_id,     sizeof(FSM_File.robot_id));
	memcpy(&memArr[1], &FSM_File.file_version_major, sizeof(FSM_File.file_version_major));
	memcpy(&memArr[2], &FSM_File.file_version_minor, sizeof(FSM_File.file_version_minor));
	memcpy(&memArr[3], &FSM_File.file_version_patch, sizeof(FSM_File.file_version_patch));

	IOIF_WriteFlash(writeAddr, memArr);
	writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;

	for (int i = 0; i < MAX_N_STATE; ++i) {
		uint32_t memArr1[6] = {0};
		
		memcpy(&memArr1[0], &FSM_File.vec[i].StateID,              sizeof(FSM_File.vec[i].StateID));
		memcpy(&memArr1[1], &FSM_File.vec[i].MotionSetID,          sizeof(FSM_File.vec[i].MotionSetID));
		memcpy(&memArr1[2], &FSM_File.vec[i].ActionID,             sizeof(FSM_File.vec[i].ActionID));
		memcpy(&memArr1[3], &FSM_File.vec[i].TimeOut,              sizeof(FSM_File.vec[i].TimeOut));
		memcpy(&memArr1[4], &FSM_File.vec[i].TabletModeID,         sizeof(FSM_File.vec[i].TabletModeID));
		memcpy(&memArr1[5], &FSM_File.vec[i].DefaultTargetStateID, sizeof(FSM_File.vec[i].DefaultTargetStateID));

		IOIF_WriteFlash(writeAddr, memArr1);
		writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;

		uint8_t memArr2[MAX_N_TRANSITION] = {0};
		for (int j = 0; j < MAX_N_TRANSITION; ++j)
		{
			memcpy(&memArr2[j], &FSM_File.vec[i].ExitConditionID[j], sizeof(FSM_File.vec[i].ExitConditionID[j]));
		}
		IOIF_WriteFlash(writeAddr, memArr2);
		writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;

		uint8_t memArr3[MAX_N_TRANSITION] = {0};
		for (int j = 0; j < MAX_N_TRANSITION; ++j)
		{
			memcpy(&memArr3[j], &FSM_File.vec[i].TargetStateID[j], sizeof(FSM_File.vec[i].TargetStateID[j]));
		}
		IOIF_WriteFlash(writeAddr, memArr3);
		writeAddr += IOIF_FLASH_READ_ADDR_SIZE_32B;
	}
}

void Download_FSM1(void) {

	uint32_t readAddr = IOIF_FLASH_START_FSM_ADDR;

	IOIF_ReadFlash(readAddr,&FSM_File.robot_id,     sizeof(FSM_File.robot_id));     			readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr,&FSM_File.file_version_major, sizeof(FSM_File.file_version_major)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr,&FSM_File.file_version_minor, sizeof(FSM_File.file_version_minor)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
	IOIF_ReadFlash(readAddr,&FSM_File.file_version_patch, sizeof(FSM_File.file_version_patch)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_20B;


	for (int i = 0; i < MAX_N_STATE; ++i) {
		IOIF_ReadFlash(readAddr, &FSM_File.vec[i].StateID,              sizeof(FSM_File.vec[i].StateID));              readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
		IOIF_ReadFlash(readAddr, &FSM_File.vec[i].MotionSetID,          sizeof(FSM_File.vec[i].MotionSetID));          readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
		IOIF_ReadFlash(readAddr, &FSM_File.vec[i].ActionID,             sizeof(FSM_File.vec[i].ActionID));			 readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
		IOIF_ReadFlash(readAddr, &FSM_File.vec[i].TimeOut,              sizeof(FSM_File.vec[i].TimeOut));				 readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
		IOIF_ReadFlash(readAddr, &FSM_File.vec[i].TabletModeID,         sizeof(FSM_File.vec[i].TabletModeID));		 readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
		IOIF_ReadFlash(readAddr, &FSM_File.vec[i].DefaultTargetStateID, sizeof(FSM_File.vec[i].DefaultTargetStateID)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_12B;
		for (int j = 0; j < MAX_N_TRANSITION; ++j) {
			IOIF_ReadFlash(readAddr, &FSM_File.vec[i].ExitConditionID[j], sizeof(FSM_File.vec[i].ExitConditionID[j])); readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
		}
		readAddr += (IOIF_FLASH_READ_ADDR_SIZE_32B - MAX_N_TRANSITION);
		for (int j = 0; j < MAX_N_TRANSITION; ++j) {
			IOIF_ReadFlash(readAddr, &FSM_File.vec[i].TargetStateID[j],   sizeof(FSM_File.vec[i].TargetStateID[j]));   readAddr += IOIF_FLASH_READ_ADDR_SIZE_1B;
		}
		readAddr += (IOIF_FLASH_READ_ADDR_SIZE_32B - MAX_N_TRANSITION);
	}
}

int Check_FSM_Save(void)
{
	int err = 0;

	if (FSM_File.file_version_major != FSM_File_Read.file_version_major) err++;
	if (FSM_File.file_version_minor != FSM_File_Read.file_version_minor) err++;
	if (FSM_File.file_version_patch != FSM_File_Read.file_version_patch) err++;

	if (FSM_File.robot_id     != FSM_File_Read.robot_id)     err++;

	for (int i = 0; i < MAX_N_STATE; i++) {
		StateVector in  = FSM_File.vec[i];
		StateVector out = FSM_File_Read.vec[i];

		if (in.ActionID             != out.ActionID)             err++;
		if (in.DefaultTargetStateID != out.DefaultTargetStateID) err++;
		if (in.MotionSetID          != out.MotionSetID)          err++;
		if (in.StateID              != out.StateID)              err++;
		if (in.TabletModeID         != out.TabletModeID)         err++;
		if (in.TimeOut              != out.TimeOut)              err++;

		for (int j = 0; j <MAX_N_TRANSITION; j++) {
			uint8_t in1  = in.ExitConditionID[j];
			uint8_t out1 = out.ExitConditionID[j];
			uint8_t in2  = in.TargetStateID[j];
			uint8_t out2 = out.TargetStateID[j];

			if (in1 != out1)             err++;
			if (in2 != out2)             err++;
		}
	}
	return err;
}

/************************* (2023-12-01) *************************/

// Download FSM.csv file to FSM ObjectINI
void Download_Test_FSM(FSMFile t_FSM_file)
{
	memset(&FSM_File, 0, sizeof(FSMFileInfo));

	for (int i = 0; i < MAX_N_STATE; i++) {
		FSM_File.vec[i].StateID              = (uint8_t)t_FSM_file[i][0][0];
		FSM_File.vec[i].MotionSetID          = (uint8_t)t_FSM_file[i][1][0];
		FSM_File.vec[i].ActionID             = (uint32_t)t_FSM_file[i][2][0];
		FSM_File.vec[i].TabletModeID         = (uint8_t)t_FSM_file[i][3][0];
		FSM_File.vec[i].TimeOut              = (uint16_t)t_FSM_file[i][4][0];
		FSM_File.vec[i].DefaultTargetStateID = (uint8_t)t_FSM_file[i][5][0];

		for (int j = 0; j < MAX_N_TRANSITION; j++) {
			FSM_File.vec[i].ExitConditionID[j] = (uint8_t)t_FSM_file[i][6][j];
			FSM_File.vec[i].TargetStateID[j]   = (uint8_t)t_FSM_file[i][7][j];
		}
	}
}

void Init_FSM(void)
{
	Init_Button_ISI();

    FSMMngrObj.state_curr = 0;
    FSMMngrObj.state_prev = 0;
}

static uint8_t t_LED_ID;
static uint8_t t_Audio_ID;
static uint8_t t_Tablet_ID;
static uint8_t t_Empty_ID;


void Run_FSM(void)
{
	uint8_t state_curr = 0;
	uint8_t idx = 0;

	// Step1. Find index
	state_curr = FSMMngrObj.state_curr;
	for (int i = 0; i < MAX_N_STATE; i++) {
		if (FSM_File.vec[i].StateID == state_curr) {
			idx = i;
			break;
		}
	}
	// Step2. Send Audio Message
	if (FSMMngrObj.time_stamp == 1) { // Enter
		uint32_t t_ActionID = FSM_File.vec[idx].ActionID;

//		uint8_t t_LED_ID    = (t_ActionID & (0xFF000000)) >> 24;
//		uint8_t t_Audio_ID  = (t_ActionID & (0x00FF0000)) >> 16;
//		uint8_t t_Tablet_ID = (t_ActionID & (0x0000FF00)) >> 8;
//		uint8_t t_Empty_ID  = (t_ActionID & (0x000000FF));

		t_LED_ID    = (t_ActionID & (0xFF000000)) >> 24;
	    t_Audio_ID  = (t_ActionID & (0x00FF0000)) >> 16;
		t_Tablet_ID = (t_ActionID & (0x0000FF00)) >> 8;
		t_Empty_ID  = (t_ActionID & (0x000000FF));

		Run_Audio_Action(t_Audio_ID);
	}

	// Step3. Send Proper Motion Set
	// Send_Motion_Set(FSM_File.vec[idx].MotionSetID);
	if (!g_ext_link_mngr.is_ext_pack_connected) { // extension module과 연결되지 않을 시 기존 보조 알고리즘 수행
		Send_Motion_Set(FSM_File.vec[idx].MotionSetID);
	} else { // extension module과 연결시 extension module에서 설계한 보조 알고리즘 수행.
		if (g_ext_comm_obj.data.H10_Existing_Assist_Mode_Flag) { // XM10 연결시에도 H10 기존 보조 알고리즘 사용시
			Send_Motion_Set(FSM_File.vec[idx].MotionSetID);
		}
	}

	// Step4. Check Transition Condition
	// Step4-1. If Time Tick-based Transition
	if (FSM_File.vec[idx].TimeOut != 0) {
		if(FSMMngrObj.time_stamp >=  FSM_File.vec[idx].TimeOut) {
			MotionMap_File.send_state = 0;
			FSMMngrObj.time_stamp = 0;
			FSMMngrObj.state_prev = FSMMngrObj.state_curr;
			FSMMngrObj.state_curr = FSM_File.vec[idx].DefaultTargetStateID;
			Flush_ISI();
		}
	} else { 	// Step4-2. If ISI-based Transition

		Check_ISI();

		for (int j = 0; j < MAX_N_TRANSITION; j++) {
			uint8_t t_ext_idx = FSM_File.vec[idx].ExitConditionID[j];
			if (isi_output_vectors[t_ext_idx] == 1) {
				MotionMap_File.send_state = 0;
				FSMMngrObj.time_stamp = 0;
				FSMMngrObj.state_prev = FSMMngrObj.state_curr;
				FSMMngrObj.state_curr = FSM_File.vec[idx].TargetStateID[j];
				Flush_ISI();
				break;
			}
		}

	}
	FSMMngrObj.time_stamp++;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
