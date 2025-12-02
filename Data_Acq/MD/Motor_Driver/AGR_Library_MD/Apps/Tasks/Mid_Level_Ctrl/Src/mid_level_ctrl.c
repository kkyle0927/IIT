/**
 * @file mid_level_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

// TODO : refactoring!!
#include "mid_level_ctrl.h"

/**
 *-----------------------------------------------------------
 *      TYPE DEFINITIONS AND ENUMERATIONS AND VARIABLES
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

TaskObj_t 		        mid_level_ctrl_task;

PIDObject 		        posCtrl;
PIDObject 		        velCtrl;

ImpedanceReductionCtrl 	IRC;
ImpedanceCtrl           impedanceCtrl;
VelocityComp 			velocitycomp;
GravityComp				gravcomp;
DegreeLimit             degreelimit;
VelocityLimit			velolimit;


DOBObject	            posDOB;
FFObject	            posFF;

VirtualSpringDamper     VSD;

VelocityEstObject       veObj;

IOIF_IncEnc_t       inc1KhzObj;
IOIF_AbsEnc_t       AbsObj1;
IOIF_AbsEnc_t       AbsObj2;

MidLevelState 		mid_level_state;

ScaledData 			FloatTo2ByteData_mid;
Abs1Checker_t		abs1Checker;
AcuatorValidation_t actuator_checker;

float torque_inter_gain = 0;
float torque_inter_threshold = 0;
uint8_t torque_inter_sw = 0;
uint8_t torque_inter_after_0 = 1;
float SitToStance_Torque = 0.0f;
float StanceToSit_Torque = 0.0f;

float comp_inter_gain = 0;
float comp_inter_threshold = 0;
uint8_t comp_inter_sw = 0;
uint8_t comp_inter_after_0 = 1;
uint8_t FSM_prev = 0 ;
uint8_t FSM_curr = 0 ;
uint8_t safety_sw = 0;
float STS_torque = 0;
float f_STS_torque = 0;
extern uint8_t safety_timer_flag;
uint8_t suit_mode = 0;
// float initOffsetAbsDeg;

P_Vector_Decoder pvectorObj;
F_Vector_Decoder fvectorObj;

float mid_ctrl_saturation;
float degree_limit_ctrl_saturation;

static PeriodicSignal 	cur_periodic_sig;
static PeriodicSignal 	vel_periodic_sig;
static PeriodicSignal 	pos_periodic_sig;

static Backlash_Test    backlash_test;
static System_ID_SBS    sys_id_sbs;

static uint32_t mid_level_loop_cnt;
static float midTimeElap;

static volatile bool ts_encoder_offset_done = false;

static uint32_t p_standing_start_cnt = 0;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run();

static void StateStandby_Ent();
static void StateStandby_Run();
static void StateStandby_Ext();

static void StateEnable_Ent();
static void StateEnable_Run();
static void StateEnable_Ext();

static void StateError_Run();

/* ------------------- SACLING DATA ------------------- */
static void UpdateScaledData_Mid(ScaledData* Scaled2ByteData);

/* ------------------- KALMAN FILTER ------------------- */
static void Init_Kalman_Filter(KFObject * t_KF_obj, float t_kf_A, float t_kf_B, float t_kf_Q, float t_kf_R);
static int Run_Kalman_Filter(KFObject * t_KF_obj, float t_u, float t_y);

/* ------------------- GET POS & VEL ------------------- */
static void Init_Position_Velocity();
static void Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc, MidLevelState* t_mid_level_state);
static void Init_Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc, MidLevelState* t_mid_level_state);

/* ------------------- INIT TORQUE MODE ------------------- */
static void Init_F_Vector_Modes();

/* ------------------- SDO CALLBACK ------------------- */
static void Set_IRC_Numerator_Length(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_IRC_Denominator_Length(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_IRC_Numerator(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_IRC_Denominator(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_IRC_Saturation(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_Vel_Ctrl_BW(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Vel_Ctrl_PGain(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Vel_Ctrl_IGain(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_Pos_Ctrl_Input_Penalty(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Pos_Ctrl_PGain(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Pos_Ctrl_DGain(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_Mid_Ctrl_Saturation(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_IOIF_IncEnc_t_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_IOIF_AbsEnc1_t_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_IOIF_AbsEnc1_t_Sign(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_IOIF_AbsEnc2_t_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_IOIF_AbsEnc2_t_Sign(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_DOB_Q_BW(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_DOB_GQ_Num(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_DOB_GQ_Den(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_DOB_Q_Num(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_DOB_Q_Den(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_DOB_Saturation(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_Current_Periodic_Sig_Info(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Velocity_Periodic_Sig_Info(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Position_Periodic_Sig_Info(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_VSD_Stiffness(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_VSD_Damper(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_VSD_Damped_Range(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_VSD_Stiff_Range(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_VSD_Upper_Limit(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_VSD_Lower_Limit(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_VSD_Saturation(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_Feedforward_Num(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Feedforward_Den(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Velocity_Estimator(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Velocity_Estimator_LeadLag(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_1khz_Enc_Resolution(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_System_ID_SBS_Info(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_System_ID_Verification_Mag(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_P_Vector_Yd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_L(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_S0(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Sd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Reset(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_F_Vector_ModeIDX(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_F_Vector_TauMax(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_F_Vector_Delay(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_F_Vector_ZERO(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_I_Vector_Epsilon(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kp(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Lambda(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Duration(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kp_Max(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Kd_Max(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_I_Vector_Option(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_Desired_Mech_Angle(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_IOIF_AbsEnc1_t_Location(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_IOIF_AbsEnc2_t_Location(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

// static void ToggleLED(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_IOIF_AbsEnc1_Resolution(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_IOIF_AbsEnc2_Resolution(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_Grav_Comp_Gain(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Vel_Comp_Gain(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_Hip_Upper_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Hip_Lower_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Hip_Upper_Vel_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Hip_Lower_Vel_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_FSM_State(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_yd_f(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_SitToStance_Torque(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_StanceToSit_Torque(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_IOIF_AbsEnc1_range_offset(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_IOIF_AbsEnc1_value_offset(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_Suit_mode(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Vel_Comp_Satu(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Init_Check_Abs1_rationality(IOIF_AbsEnc_t* abs1, Abs1Checker_t* checker);
static void Check_Abs1_rationality(IOIF_AbsEnc_t* abs1, MidLevelState* t_mid_level_state, Abs1Checker_t* checker);

#ifdef SUIT_MD_ENABLED
static int Read_NodeID(void);
static void Set_Properties_SAM10(void);
#endif
/* ------------------- ROUTINE ------------------- */
static int Ent_Position_Ctrl();
static int Run_Position_Ctrl();

static int Ent_Velocity_Ctrl();
static int Run_Velocity_Ctrl();

static int Ent_Impedance_Reduction();
static int Run_Impedance_Reduction();

static int Ent_Compressional_SpringDamper();
static int Run_Compressional_SpringDamper();
static int Ext_Compressional_SpringDamper();

static int Run_Backlash_Test();

static int Ent_Disturbance_Obs();
static int Run_Disturbance_Obs();

static int Ent_Mech_SystemID_SBS();
static int Run_Mech_SystemID_SBS();
static int Ext_Mech_SystemID_SBS();

static int Ent_P_Vector_Decoder();
static int Run_P_Vector_Decoder();
static int Ext_P_Vector_Decoder();
static void Reset_P_Vector();

static int Run_F_Vector_Decoder();
static int Ext_F_Vector_Decoder();

static int Ent_Feedforward_Filter();
static int Run_Feedforward_Filter();
static int Ext_Feedforward_Filter();

static int Run_System_ID_Verify();

static int Ent_Gravity_Compensation();
static int Run_Gravity_Compensation();

static int Ent_Velocity_Compensation();
static int Run_Velocity_Compensation();

static int Ent_Degree_Limit();
static int Run_Degree_Limit();

static int Ent_Velocity_Limit();
static int Run_Velocity_Limit();

static void error_filter2(ImpedanceCtrl *t_impedanceCtrl);
static int Ent_Corridor_Impedance_Control();
static int Run_Corridor_Impedance_Control();

static int Ent_Generate_Current_Sine();
static int Run_Generate_Current_Sine();

static int Ent_Generate_Velocity_Sine();
static int Run_Generate_Velocity_Sine();

static int Ent_Generate_Position_Sine();
static int Run_Generate_Position_Sine();

static int Ent_Generate_Current_Tanh();
static int Run_Generate_Current_Tanh();

static int Ent_Generate_Current_Rec();
static int Run_Generate_Current_Rec();

static int Ent_Generate_Velocity_Rectangle();
static int Run_Generate_Velocity_Rectangle();

static int Ent_Generate_Position_Rectangle();
static int Run_Generate_Position_Rectangle();

static int Run_Send_IOIF_Hall_t_Values_to_GUI();
static int Run_Send_IOIF_IncEnc_t_Values_to_GUI();
static int Run_Send_IOIF_AbsEnc1_t_Values_to_GUI();
static int Run_Send_IOIF_AbsEnc2_t_Values_to_GUI();

static int Ent_Check_Torque_Accuracy(void);
static int Run_Check_Torque_Accuracy(void);
static int Ext_Check_Torque_Accuracy(void);

static int Ent_Check_Torque_Uniformity(void);
static int Run_Check_Torque_Uniformity(void);
static int Ext_Check_Torque_Uniformity(void);

static int Ent_Check_backdriverbility(void);
static int Run_Check_backdriverbility(void);
static int Ext_Check_backdriverbility(void);

static int Ent_Check_EncoderLinearity(void);
static int Run_Check_EncoderLinearity(void);
static int Ext_Check_EncoderLinearity(void);

static float mid_level_process_array0[MID_LV_ARRAY_MAX];
static float mid_level_process_array1[MID_LV_ARRAY_MAX];
static float mid_level_process_array2[MID_LV_ARRAY_MAX];
static float mid_level_process_array3[MID_LV_ARRAY_FOR_ENCODER]; // 360deg / 0.45 Interval
static float mid_level_process_array4[MID_LV_ARRAY_FOR_ENCODER]; // 360deg / 0.45 Interval


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

DOP_COMMON_SDO_CB(mid_level_ctrl_task)

float test_min;
float test_max;
void InitMidLvCtrl(void)
{
   	InitTask(&mid_level_ctrl_task);
	Init_F_Vector_Modes();
#ifdef SUIT_MD_ENABLED
	Set_Properties_SAM10();
#endif
   	/**/
	IOIF_InitIncEnc(IOIF_TIM5, IOIF_TIM_CHANNEL_ALL , &inc25KhzObj);
	//#ifdef _USE_SYSTEM_ID
	IOIF_InitAbsEnc(IOIF_SPI1, 1, &AbsObj1, IOIF_ABS_ENC_JOINT1, 			IOIF_ABS_RESOLUTION, MID_LEVEL_CONTROL_FREQUENCY);
	IOIF_InitAbsEnc(IOIF_SPI3, 2, &AbsObj2, IOIF_ABS_ENC_ACTUATOR_INPUT, 	IOIF_ABS_RESOLUTION, MID_LEVEL_CONTROL_FREQUENCY);
	//#else
	////#ifndef _USE_DEBUG_CLI
	//   	IOIF_InitAbsEnc(IOIF_SPI1, 1, &AbsObj1, AbsObj1.location, IOIF_ABS_RESOLUTION, MID_LEVEL_CONTROL_FREQUENCY);//AbsObj1.resolution, MID_LEVEL_CONTROL_FREQUENCY);
	//	IOIF_InitAbsEnc(IOIF_SPI3, 2, &AbsObj2, AbsObj2.location, IOIF_ABS_RESOLUTION, MID_LEVEL_CONTROL_FREQUENCY);//AbsObj2.resolution, MID_LEVEL_CONTROL_FREQUENCY);
	//#endif /* !_USE_DEBUG_CLI */
	Init_Kalman_Filter(&veObj.kf_obj, -motor_properties.a, motor_properties.b, 1, 5);

	/* State Definition*/
	TASK_CREATE_STATE(&mid_level_ctrl_task, TASK_STATE_OFF,     NULL,     			StateOff_Run,           NULL, 				true);
	TASK_CREATE_STATE(&mid_level_ctrl_task, TASK_STATE_STANDBY, StateStandby_Ent, 	StateStandby_Run,		StateStandby_Ext, 	false);
	TASK_CREATE_STATE(&mid_level_ctrl_task, TASK_STATE_ENABLE,  StateEnable_Ent, 	StateEnable_Run, 		StateEnable_Ext, 	false);
	TASK_CREATE_STATE(&mid_level_ctrl_task, TASK_STATE_ERROR,   NULL,             	StateError_Run,  		NULL, 				false);

	/* Routine Definition*/
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_SYS_ID_SBS,  		            Ent_Mech_SystemID_SBS,      	 Run_Mech_SystemID_SBS,                 Ext_Mech_SystemID_SBS);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_IRC, 				            Ent_Impedance_Reduction,		 Run_Impedance_Reduction,		        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_IMPEDANCE_CONTROL,  	            Ent_Corridor_Impedance_Control,  Run_Corridor_Impedance_Control,        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_REC_REF,	            Ent_Generate_Position_Rectangle, Run_Generate_Position_Rectangle,       NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_SINE_REF,  	            Ent_Generate_Position_Sine, 	 Run_Generate_Position_Sine, 	        NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_CTRL, 		            Ent_Position_Ctrl, 				 Run_Position_Ctrl, 			        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_VELOCITY_CTRL, 		            Ent_Velocity_Ctrl, 				 Run_Velocity_Ctrl, 				    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_VELOCITY_SINE_REF,  	            Ent_Generate_Velocity_Sine, 	 Run_Generate_Velocity_Sine, 	        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_VELOCITY_REC_REF,	            Ent_Generate_Velocity_Rectangle, Run_Generate_Velocity_Rectangle,       NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CURRENT_SINE_REF,  	            Ent_Generate_Current_Sine, 	     Run_Generate_Current_Sine, 	        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CURRENT_REC_REF,	                Ent_Generate_Current_Rec,        Run_Generate_Current_Rec,              NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CURRENT_TANH_REF,	            Ent_Generate_Current_Tanh,       Run_Generate_Current_Tanh,             NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GET_HALL_SENSOR_VALUE,           NULL,				 			 Run_Send_IOIF_Hall_t_Values_to_GUI,    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GET_INCENCODER_VALUE,            NULL,				 			 Run_Send_IOIF_IncEnc_t_Values_to_GUI,  NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GET_ABSENCODER1_VALUE, 			NULL,				 			 Run_Send_IOIF_AbsEnc1_t_Values_to_GUI, NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GET_ABSENCODER2_VALUE, 			NULL,				 			 Run_Send_IOIF_AbsEnc2_t_Values_to_GUI, NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_COMPRESSIONAL_VSD,				Ent_Compressional_SpringDamper,	 Run_Compressional_SpringDamper,	Ext_Compressional_SpringDamper);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_BACKLASH_TEST,        			NULL,   	                     Run_Backlash_Test, 			    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_DISTURBANCE_OBS,      			Ent_Disturbance_Obs,             Run_Disturbance_Obs,          	    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_P_VECTOR_DECODER,  		        Ent_P_Vector_Decoder,            Run_P_Vector_Decoder,  	        Ext_P_Vector_Decoder);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_F_VECTOR_DECODER,   			    NULL,                            Run_F_Vector_Decoder,              Ext_F_Vector_Decoder);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_FEEDFORWARD_FILTER,  	        Ent_Feedforward_Filter,          Run_Feedforward_Filter,       	    Ext_Feedforward_Filter);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_SYS_ID_SBS_VERIFY,  	            NULL,         					 Run_System_ID_Verify,       	    NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GRAVITY_COMPENSATION,  	        Ent_Gravity_Compensation,        Run_Gravity_Compensation,       	    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_VELOCITY_COMPENSATION,  	        Ent_Velocity_Compensation,       Run_Velocity_Compensation,      	    NULL);

//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_DEGREE_LIMIT,    				Ent_Degree_Limit,  				Run_Degree_Limit,			NULL);
//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_VELOCITY_LIMIT,    				Ent_Velocity_Limit,  			Run_Velocity_Limit,			NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CHECK_TORQUE_ACCURACY,    		Ent_Check_Torque_Accuracy,  	Run_Check_Torque_Accuracy,  	Ext_Check_Torque_Accuracy);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CHECK_TORQUE_UNIFORMITY,    		Ent_Check_Torque_Uniformity,  	Run_Check_Torque_Uniformity,  	Ext_Check_Torque_Uniformity);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CHECK_ACTUATOR_BACKDRIVABILITY,  Ent_Check_backdriverbility,  	Run_Check_backdriverbility,  	Ext_Check_backdriverbility);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CHECK_ENCODER_LINEARITY,  		Ent_Check_EncoderLinearity,  	Run_Check_EncoderLinearity,  	Ext_Check_EncoderLinearity);

	/* DOD Definition*/
	// DOD
	DOP_CreateDOD(TASK_ID_MIDLEVEL);

	// PDO
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LOOP_CNT,						DOP_UINT32,   1, &mid_level_loop_cnt);
	// DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_POSITION, 					DOP_FLOAT32,  1, &posCtrl.ref);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_POSITION, 					DOP_INT16,	  1, &FloatTo2ByteData_mid.P_Vector_Ref);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_VELOCITY, 					DOP_FLOAT32,  1, &velCtrl.ref);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_POSITION, 				DOP_FLOAT32,  1, &mid_level_state.position);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW, 			DOP_FLOAT32,  1, &mid_level_state.velocity_raw);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_SYSTEM_ID_SBS_FREQ,				DOP_FLOAT32,  1, &sys_id_sbs.current_f);

	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_CTRL_INPUT,		 			DOP_FLOAT32,  1, &impedanceCtrl.control_input);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_POS_PID_CTRL_INPUT,				DOP_FLOAT32,  1, &posCtrl.control_input);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VEL_PID_CTRL_INPUT, 			DOP_FLOAT32,  1, &velCtrl.control_input);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VSD_INPUT,					 	DOP_FLOAT32,  1, &VSD.control_input);

	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_F_VECTOR_INPUT,	         		DOP_FLOAT32,  1, &fvectorObj.input);

	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER1_POSITION,  			DOP_FLOAT32,  1, &AbsObj1.posDeg);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER2_POSITION,  			DOP_FLOAT32,  1, &AbsObj2.posDeg);

	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_DOB_DISTURABNCE, 				DOP_FLOAT32,  1, &posDOB.disturbance);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_DOB_INPUT, 						DOP_FLOAT32,  1, &posDOB.control_input);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_FF_INPUT, 						DOP_FLOAT32,  1, &posFF.control_input);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED, 			DOP_FLOAT32,  1, &mid_level_state.velocity_final);

	// DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_EPSILON, 			        DOP_FLOAT32,  1, &impedanceCtrl.epsilon);
	// DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_KP, 	 			        DOP_FLOAT32,  1, &impedanceCtrl.Kp);
	// DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_KD,    			        	DOP_FLOAT32,  1, &impedanceCtrl.Kd );
	// DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_LAMDA,   		         	DOP_FLOAT32,  1, &impedanceCtrl.lambda);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_EPSILON, 			        DOP_INT16,  1, &FloatTo2ByteData_mid.I_Vector_Epsilon);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_KP, 	 			        DOP_INT16,  1, &FloatTo2ByteData_mid.I_Vector_kP);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_KD,    			        	DOP_INT16,  1, &FloatTo2ByteData_mid.I_Vector_kD);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_LAMDA,   		         	DOP_INT16,  1, &FloatTo2ByteData_mid.I_Vector_lambda);

	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER1_VELOCITY,  			DOP_FLOAT32,  1, &AbsObj1.velDeg);
	DOP_CreatePDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER2_VELOCITY,  			DOP_FLOAT32,  1, &AbsObj2.velDeg);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_MIDLEVEL)

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_NUMERATOR_LENGTH,     		DOP_UINT8,		Set_IRC_Numerator_Length);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_DENOMINATOR_LENGTH,	    	DOP_UINT8,		Set_IRC_Denominator_Length);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_NUMERATOR,	       			DOP_FLOAT32,  	Set_IRC_Numerator);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_DENOMINATOR,	       		DOP_FLOAT32,  	Set_IRC_Denominator);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_SATURATION,	       			DOP_FLOAT32,  	Set_IRC_Saturation);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_CURRENT_PERIODIC_SIG_INFO,	    DOP_FLOAT32, 	Set_Current_Periodic_Sig_Info);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_PERIODIC_SIG_INFO,		DOP_FLOAT32, 	Set_Velocity_Periodic_Sig_Info);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_PERIODIC_SIG_INFO,		DOP_FLOAT32, 	Set_Position_Periodic_Sig_Info);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_Q_BW,	       				DOP_FLOAT32,  	Set_DOB_Q_BW);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_GQ_NUM,	       				DOP_FLOAT32,  	Set_DOB_GQ_Num);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_GQ_DEN,	       				DOP_FLOAT32,  	Set_DOB_GQ_Den);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_Q_NUM,	       				DOP_FLOAT32,  	Set_DOB_Q_Num);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_Q_DEN,	       				DOP_FLOAT32,  	Set_DOB_Q_Den);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_SATURATION,	       			DOP_FLOAT32,  	Set_DOB_Saturation);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_CTRL_BW, 				DOP_FLOAT32, 	Set_Vel_Ctrl_BW);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_CTRL_P_GAIN, 			DOP_FLOAT32, 	Set_Vel_Ctrl_PGain);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_CTRL_I_GAIN, 			DOP_FLOAT32, 	Set_Vel_Ctrl_IGain);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_CTRL_INPUT_PENALTY,	DOP_FLOAT32, 	Set_Pos_Ctrl_Input_Penalty);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_CTRL_P_GAIN, 			DOP_FLOAT32, 	Set_Pos_Ctrl_PGain);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_CTRL_D_GAIN, 			DOP_FLOAT32, 	Set_Pos_Ctrl_DGain);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_MID_CTRL_SATURATION, 			DOP_FLOAT32, 	Set_Mid_Ctrl_Saturation);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_INCENCODER_SET_OFFSET, 			DOP_UINT8, 		Set_IOIF_IncEnc_t_Offset);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_SET_OFFSET,        	DOP_UINT8, 		Set_IOIF_AbsEnc1_t_Offset);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_CHANGE_DIRECTION,	DOP_UINT8, 		Set_IOIF_AbsEnc1_t_Sign);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER2_SET_OFFSET,        	DOP_UINT8, 		Set_IOIF_AbsEnc2_t_Offset);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER2_CHANGE_DIRECTION,	DOP_UINT8, 		Set_IOIF_AbsEnc2_t_Sign);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_STIFFNESS,					DOP_FLOAT32, 	Set_VSD_Stiffness);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_DAMPER,						DOP_FLOAT32, 	Set_VSD_Damper);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_DAMPED_RANGE,				DOP_FLOAT32, 	Set_VSD_Damped_Range);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_STIFF_RANGE,				DOP_FLOAT32, 	Set_VSD_Stiff_Range);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_VSD_UPPER_LIMIT,			DOP_FLOAT32, 	Set_VSD_Upper_Limit);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_VSD_LOWER_LIMIT,			DOP_FLOAT32, 	Set_VSD_Lower_Limit);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_SATURATION,					DOP_FLOAT32, 	Set_VSD_Saturation);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FEEDFORWARD_NUM,				DOP_FLOAT32,  	Set_Feedforward_Num);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FEEDFORWARD_DEN,				DOP_FLOAT32,  	Set_Feedforward_Den);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_ESTIMATOR,				DOP_UINT8,  	Set_Velocity_Estimator);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_ESTIMATOR_LEAD_LAG,	DOP_FLOAT32,  	Set_Velocity_Estimator_LeadLag);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ENCODER_RESOLUTION,				DOP_UINT16,  	Set_1khz_Enc_Resolution);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SYSTEM_ID_SBS_INFO,				DOP_FLOAT32,  	Set_System_ID_SBS_Info);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SYSTEM_ID_VERIFICATION_MAG,		DOP_FLOAT32,  	Set_System_ID_Verification_Mag);

	/* P vector */
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_YD,	                DOP_INT16,    	Set_P_Vector_Yd);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_L, 	                DOP_UINT16,   	Set_P_Vector_L);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_S0,	                DOP_UINT8,    	Set_P_Vector_S0);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_SD,	                DOP_UINT8,    	Set_P_Vector_Sd);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_RESET,	                DOP_UINT8,    	Set_P_Vector_Reset);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX,             	DOP_UINT8,    	Set_F_Vector_ModeIDX);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_TMAX,	                DOP_INT16,    	Set_F_Vector_TauMax);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_DELAY,	            	DOP_UINT16,   	Set_F_Vector_Delay);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_ZERO,	            	DOP_UINT16,   	Set_F_Vector_ZERO);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_EPSILON,	            DOP_UINT8,    	Set_I_Vector_Epsilon);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP, 	                DOP_UINT8,    	Set_I_Vector_Kp);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD,	                DOP_UINT8,    	Set_I_Vector_Kd);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_LAMBDA,	            DOP_UINT8,    	Set_I_Vector_Lambda);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_DURATION,	            DOP_UINT16,   	Set_I_Vector_Duration);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP_MAX,	            DOP_FLOAT32,  	Set_I_Vector_Kp_Max); // Actual Kp @ 100%
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD_MAX,	            DOP_FLOAT32,  	Set_I_Vector_Kd_Max); // Actual Kd @ 100%
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_OPTION,	            DOP_UINT8,    	Set_I_Vector_Option); // Actual Kd @ 100%
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DESIRED_MECH_ANGLE,            	DOP_FLOAT32,  	Set_Desired_Mech_Angle);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_LOCATION,          	DOP_UINT8,    	Set_IOIF_AbsEnc1_t_Location);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER2_LOCATION,          	DOP_UINT8,    	Set_IOIF_AbsEnc2_t_Location);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_RESOLUTION,         DOP_UINT16,    	Set_IOIF_AbsEnc1_Resolution);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER2_RESOLUTION,         DOP_UINT16,    	Set_IOIF_AbsEnc2_Resolution);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_GRAV_COMP_GAIN,         		DOP_FLOAT32,	Set_Grav_Comp_Gain);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VEL_COMP_GAIN,         			DOP_FLOAT32,	Set_Vel_Comp_Gain);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_RANGE_OFFSET,       DOP_UINT8,    	Set_IOIF_AbsEnc1_range_offset);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_VALUE_OFFSET,       DOP_UINT8,    	Set_IOIF_AbsEnc1_value_offset);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_SUIT_MODE,       	 		DOP_UINT8,    	Set_Suit_mode);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VEL_COMP_SATU,       			DOP_UINT8,    	Set_Vel_Comp_Satu);

	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_UPPER_LIMIT,       			DOP_FLOAT32,    Set_Hip_Upper_Limit);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_LOWER_LIMIT,       	 		DOP_FLOAT32,    Set_Hip_Lower_Limit);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_UPPER_VEL_LIMIT,       			DOP_FLOAT32,    Set_Hip_Upper_Vel_Limit);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_HIP_LOWER_VEL_LIMIT,       			DOP_FLOAT32,    Set_Hip_Lower_Vel_Limit);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FSM_CURR,       				DOP_UINT8,    Set_FSM_State);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F,       				DOP_UINT8,    Set_yd_f);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_SITTOSTANCE_TOQUE,       				DOP_FLOAT32,    Set_SitToStance_Torque);
	DOP_CreateSDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STANCETOSIT_TOQUE,       				DOP_FLOAT32,    Set_StanceToSit_Torque);
	/* Timer 7 Callback Allocation */
	if(IOIF_StartTimIT(IOIF_TIM7) > 0) {
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM7, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunMidLvCtrl, NULL);

	/* PWM Input Setting*/
	if(motor_setting.input_info.inputMethod == DOPI_PWM) {
		// Set_GPIOE_State(PWM_INPUT_Pin, GPIO_PIN_RESET);
		// HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);

		// ioif_tim15_callback_ptr = Get_PWM_Control_Reference;
	}

	velolimit.HipExtensionVelLimit = 100;
	velolimit.HipFlexionVelLimit = 100;

#ifndef USE_FSR_SENSOR
    if(IOIF_StartTimIT(IOIF_TIM17) > 0){					//4kHz
        //TODO: ERROR PROCESS
    }

	IOIF_StartPWM(IOIF_TIM17, IOIF_TIM_CHANNEL_1);
#endif
	veObj.type = e_VE_KALMAN;
}

void RunMidLvCtrl(void* params)
{
	//TODO: REMOVE
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, TP5_Pin, IOIF_GPIO_PIN_SET);

	/* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/*Run Device */
	RunTask(&mid_level_ctrl_task);

	/* Elapsed Time Check */
	midTimeElap = DWT->CYCCNT/480;	// in microsecond

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, TP5_Pin, IOIF_GPIO_PIN_RESET);
}

void Tune_Gain()
{
	static float t_wvc = 0.0, t_num = 0.0, t_den = 0.0, t_R = 0.0;

	t_wvc = velCtrl.Ctrl_BW_Hz * 2 * M_PI;
	t_R = posCtrl.R;

	memset(&velCtrl, 0, sizeof(PIDObject));
	memset(&posCtrl, 0, sizeof(PIDObject));

	velCtrl.Kp = motor_properties.J * t_wvc;
	velCtrl.Ki = motor_properties.B * t_wvc;

	velCtrl.Ctrl_BW_Hz = t_wvc / 2 / M_PI;
	posCtrl.R = t_R;

	t_num = 2*sqrtf(posCtrl.R*motor_properties.J*motor_properties.J);
	t_den = posCtrl.R;

	posCtrl.Kp = sqrtf(1/posCtrl.R);

	if (t_den != 0) {
		posCtrl.Kd = -motor_properties.B + sqrtf(motor_properties.B*motor_properties.B + ( t_num/t_den ));
	} else {
		// TODO: Handle division by zero
		posCtrl.Kd = 0.0;
	}

	/* Send to GUI */
	uint16_t t_identifier = 0;
	float t_buf[4] = {0};

	memcpy(&t_buf[0], &velCtrl.Kp, 4);
	memcpy(&t_buf[1], &velCtrl.Ki, 4);
	memcpy(&t_buf[2], &posCtrl.Kp, 4);
	memcpy(&t_buf[3], &posCtrl.Kd, 4);

	for(int j = 0; j<100000; ++j){}

	t_identifier = GUI_SYNC|GAIN_TUNER;
	Send_MSG(t_identifier, (uint8_t*)t_buf, 16);

	MS_enum = IDLE;
}

void Set_Velo_Estimator(){
	//Velocity estimator
	memset(&veObj.lpf_a, 0, sizeof(float));
	memset(&veObj.lpf_b, 0, sizeof(float));
	memset(veObj.vel_save, 0, sizeof(veObj.vel_save));
	memset(veObj.pos_save, 0, sizeof(veObj.pos_save));
	memset(&veObj.kf_obj, 0, sizeof(veObj.kf_obj));
	veObj.leadlag_a = 0;
	veObj.leadlag_b = 0;
	veObj.masking_size = 0;

	veObj.type= e_VE_KALMAN;

	mid_level_state.position = 0;
	mid_level_state.position_f = 0;
	mid_level_state.velocity_raw = 0;
	mid_level_state.velocity_final = 0;
	motor_in.mid_ctrl_input  = 0;
	motor_in.irc_input = 0;
	motor_in.auxiliary_input = 0;
	motor_in.f_vector_input = 0;

	//		if(isnan(motor_properties.a) || isnan(motor_properties.b)) {
	//			Send_MSG((uint16_t)(GUI_SYNC|VE_KF_SETTING_ERROR), (uint8_t*)0, 1);
	//		} else if(motor_properties.a == 0 || motor_properties.b == 0) {
	//			Send_MSG((uint16_t)(GUI_SYNC|VE_KF_SETTING_ERROR), (uint8_t*)0, 1);
	//		} else {
	Init_Kalman_Filter(&veObj.kf_obj, -motor_properties.a, motor_properties.b, 1, 5);
	//		}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run()
{

	Init_Position_Velocity();
	StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
	degree_limit_ctrl_saturation = 6;

	//#ifndef _USE_SYSTEM_ID
	//	Set_Velo_Estimator();
	//#endif
}

static void StateStandby_Ent( )
{
	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);
//	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
//	Get_Position_Velocity(&inc1KhzObj, &mid_level_state);
//	if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Zero) {
//		mid_level_state.position_offset = 0;
//	} else if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Abs_Encoder1) {
//		mid_level_state.init_position = AbsObj1.posDegMultiTurn[0] * 0.017453292519943;
//		mid_level_state.position_offset += mid_level_state.init_position - mid_level_state.position;
//	} else if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Abs_Encoder2) {
//		mid_level_state.init_position = AbsObj2.posDegMultiTurn[0] * 0.017453292519943;
//		mid_level_state.position_offset += mid_level_state.init_position - mid_level_state.position;
//	}

	if (isnan(mid_level_state.position_offset)) {
		// TODO: Handle division by zero
		mid_level_state.position_offset = 0;
	} else {

	}
}

static void StateStandby_Run( )
{
	static uint8_t onetime_init_sw = 1;
	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);
	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
	if(onetime_init_sw == 1)
	{
		// initOffsetAbsDeg = AbsObj1.posDeg[0] / 1.875;
		mid_level_state.position_offset = (AbsObj1.posDeg[0] / 1.875) * M_PI / 180.0f;
		Get_Position_Velocity(&inc1KhzObj, &mid_level_state);
		onetime_init_sw = 0;
	}

	if(ts_encoder_offset_done == true)
	{
		mid_level_state.position_offset = (AbsObj1.posDeg[0] / 1.875) * M_PI / 180.0f;
		inc1KhzObj.userCnt = 0;			// inc.encoder count 초기화
		Get_Position_Velocity(&inc1KhzObj, &mid_level_state);
		ts_encoder_offset_done = false;
	}

	// PushRoutine(&mid_level_ctrl_task.routine, ROUTINE_ID_MIDLEVEL_IRC);
	// PushRoutine(&mid_level_ctrl_task.routine, ROUTINE_ID_MIDLEVEL_GRAVITY_COMPENSATION);
	// PushRoutine(&mid_level_ctrl_task.routine, ROUTINE_ID_MIDLEVEL_VELOCITY_COMPENSATION);

	//	 PushRoutine(&mid_level_ctrl_task.routine, ROUTINE_ID_MIDLEVEL_DEGREE_LIMIT);
//		 PushRoutine(&mid_level_ctrl_task.routine, ROUTINE_ID_MIDLEVEL_VELOCITY_LIMIT);
	// PushRoutine(&mid_level_ctrl_task.routine, ROUTINE_ID_MIDLEVEL_F_VECTOR_DECODER);
//		 StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_ENABLE);
}

static void StateStandby_Ext( )
{

}

static void StateEnable_Ent( )
{
	mid_level_loop_cnt = 0;

	impedanceCtrl.control_input = 0;        // impedance controller
	posCtrl.control_input = 0;
	velCtrl.control_input = 0;
	posFF.control_input = 0;
	posDOB.control_input = 0;

	Init_Kalman_Filter(&veObj.kf_obj, -motor_properties.a, motor_properties.b, 1, 5); // for dev mech sys verification
	Init_Check_Abs1_rationality(&AbsObj1, &abs1Checker);
	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);
	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
	Init_Get_Position_Velocity(&inc1KhzObj, &mid_level_state);
	posCtrl.ref = mid_level_state.position;
	mid_ctrl_saturation = motor_setting.contCurr_limit;

	EntRoutines(&mid_level_ctrl_task.routine);
}

static void StateEnable_Run( )
{
	static float t_input = 0;
	static float f_torque_grav_comp = 0;
	static float f_negative_damping = 0;
	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);               // Get Absolute Encoder CH1 Angle
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);               					// Get Absolute Encoder CH2 Angle
	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);              // Get Incremental Encoder Angle
	Get_Position_Velocity(&inc1KhzObj, &mid_level_state); // Calculate Angle and Velocity
	Check_Abs1_rationality(&AbsObj1, &mid_level_state, &abs1Checker);

	RunRoutines(&mid_level_ctrl_task.routine);
	if(MD_nodeID == 6 || MD_nodeID == 7)
	{
//		if((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias) > torque_inter_threshold)
//		{
//			torque_inter_sw = 0;
//			torque_inter_after_0 = 0;
//			if(torque_inter_gain<0.002)
//			{
//				torque_inter_gain =1;
//			}
//		}
//
//		if((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias) < torque_inter_threshold && torque_inter_after_0 != 1)
//		{
//			torque_inter_sw = 1;
//		}
//
//
//		if((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias) > comp_inter_threshold)
//		{
//			comp_inter_after_0 = 0;
//			comp_inter_sw = 0;
//			comp_inter_gain = 1;
//		}
//
//
//		if((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias) < comp_inter_threshold && comp_inter_after_0 != 1)
//		{
//			comp_inter_sw = 1;
//		}
//
		if (FSM_curr !=73){
			p_standing_start_cnt = 0;
		}

		if(fabsf(motor_in.total_current_input - motor_actual_current) > 0.2
			&& fabsf(mid_level_state.velocity_raw / M_PI * 180.0) > 300
			&& ((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias) > 85
					|| (AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal- filteredAngleDataObj_Sagittal.NeutralPostureBias) < -55)
			&& safety_timer_flag != 1)
		{
			safety_sw = 1;
		}

		if(FSM_curr == 69|| FSM_curr == 70){
			STS_torque = StanceToSit_Torque*sin((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias) * 2.0f * M_PI / 180.0f);
		}
		else if(FSM_curr == 74 || FSM_curr == 75 ){
//			STS_torque = SitToStance_Torque*sin((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal)*2*M_PI/180) + 10 * mid_level_state.velocity_raw;
			STS_torque = SitToStance_Torque*sin((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias) * 2.0f * M_PI / 180.0f);
				//				mid_level_state.velocity_raw
		}
		
		else if(FSM_curr == 73){
			p_standing_start_cnt++;
			if (p_standing_start_cnt < 2000) STS_torque = -3.0f;
			else STS_torque = 0.0f;
		}
		else if(FSM_curr >= 65 && FSM_curr <= 75){
			STS_torque = 0;
		}

		if(FSM_curr >= 65 && FSM_curr <= 75){
			float rate_limit = 0.02f;
			if(FSM_curr == 73){
				rate_limit = 0.002f; 						// 일어설때 팍 치지 않게
			}
			else if (FSM_prev == 75 || FSM_prev == 74){ 	// 일어서고 토크가 서서히 줄어들게
				rate_limit = 0.002f;
			}
			else if (FSM_prev == 70){ // 앉고나서 토크가 서서히 줄어들게
				rate_limit = 0.004f;
			}

			if (f_STS_torque - STS_torque > rate_limit){
				f_STS_torque -= rate_limit;
			}
			else if (f_STS_torque - STS_torque < -rate_limit){
				f_STS_torque += rate_limit;
			}
			else f_STS_torque = STS_torque;
		}

		t_input = motor_in.mid_id_process_input + 					/*control input for model identification */            \
				impedanceCtrl.control_input   					+ /*control input of impedance controller */             \
				posCtrl.control_input         					+ /*control input of PID position controller */          \
				velCtrl.control_input         					+ /*control input of PID velocity controller */          \
				posFF.control_input           					+ /*control input of position feed-forward controller */ \
				VSD.control_input									+ /*control input of virtual-spring-damper */            \
				-posDOB.control_input         					+ /*control input of disturbance observer */             \
				velocitycomp.f_velocity_comp_torque              	+ /*control input of negative damping */				 \
				gravcomp.f_grav_comp_torque						 +/*control input of gravity compensation */\
				-f_STS_torque;


		if(suit_mode == SUIT_SMART_ASSIST) // algorithm for smart assist only
		{
			if(comp_inter_sw == 1)
			{
				t_input = t_input * comp_inter_gain;

				if(comp_inter_gain > 0)
				{
					comp_inter_gain -= 0.002;
				}
			}
		}

		Run_Degree_Limit();
		Run_Velocity_Limit();

		if (suit_mode != SUIT_DEGREE_VELOCITY_LIMit) {
			degreelimit.JointRoMLimitON = 0;
			degreelimit.degree_limit_torque = 0;
			velolimit.f_velo_limit_torque = 0;
		}

		/* Joint Degree Limit Saturation */
		if     (degreelimit.degree_limit_torque > +degree_limit_ctrl_saturation)		{degreelimit.degree_limit_torque = +degree_limit_ctrl_saturation;}
		else if(degreelimit.degree_limit_torque < -degree_limit_ctrl_saturation)		{degreelimit.degree_limit_torque = -degree_limit_ctrl_saturation;}


		t_input += degreelimit.degree_limit_torque					+ /*control input of degree limit */\
				velolimit.f_velo_limit_torque;						/*control input of velocity limit */

		/* Input Saturation */
		if     (t_input > +mid_ctrl_saturation)		{motor_in.mid_ctrl_input = +mid_ctrl_saturation;}
		else if(t_input < -mid_ctrl_saturation)		{motor_in.mid_ctrl_input = -mid_ctrl_saturation;}
		else										{motor_in.mid_ctrl_input = t_input;}
	}

	if(MD_nodeID == 8 || MD_nodeID == 9)
	{
		//			if((AbsObj1.posDeg[0]/1.87 - imuVQFDataObj.angle_bodyframe.sagittal) > torque_inter_threshold)
		//			{
		//				torque_inter_sw = 0;
		//				torque_inter_after_0 = 0;
		//				if(torque_inter_gain<0.002)
		//				{
		//					torque_inter_gain =1;
		//				}
		//			}
		//
		//			if((AbsObj1.posDeg[0]/1.875 - imuVQFDataObj.angle_bodyframe.sagittal) < torque_inter_threshold && torque_inter_after_0 != 1)
		//			{
		//				torque_inter_sw = 1;
		//			}
		//
		//
		//			if((AbsObj1.posDeg[0]/1.875 - imuVQFDataObj.angle_bodyframe.sagittal) > comp_inter_threshold)
		//			{
		//				comp_inter_after_0 = 0;
		//			}
		//
		//
		//			if((AbsObj1.posDeg[0]/1.875 - imuVQFDataObj.angle_bodyframe.sagittal) < comp_inter_threshold && comp_inter_after_0 != 1)
		//			{
		//				comp_inter_sw = 1;
		//			}
		//
		//
		//			if(motor_in.total_current_input - motor_out.current_act > 0.2 && mid_level_state.velocity_raw / M_PI * 180.0 > 300 && (AbsObj1.posDeg[0]/1.875 - imuVQFDataObj.angle_bodyframe.sagittal) > 70)
		//			{
		//				safety_sw = 1;
		//			}


		t_input = motor_in.mid_id_process_input + 					/*control input for model identification */            \
				impedanceCtrl.control_input   					+ /*control input of impedance controller */             \
				posCtrl.control_input         					+ /*control input of PID position controller */          \
				velCtrl.control_input         					+ /*control input of PID velocity controller */          \
				posFF.control_input           					+ /*control input of position feed-forward controller */ \
				VSD.control_input									+ /*control input of virtual-spring-damper */            \
				-posDOB.control_input         					+ /*control input of disturbance observer */             \
				velocitycomp.f_velocity_comp_torque;              	 /*control input of negative damping */				 \
				//					gravcomp.f_grav_comp_torque;						 /*control input of gravity compensation */

				//			if(suit_mode == SUIT_SMART_ASSIST) // algorithm for smart assist only
				//			{
				//				if(comp_inter_sw == 1)
				//				{
				//					t_input = t_input * comp_inter_gain;
				//
				//					if(comp_inter_gain > 0)
				//					{
				//						comp_inter_gain -= 0.002;
				//					}
				//				}
				//			}
				t_input += degreelimit.degree_limit_torque					+ /*control input of degree limit */\
				velolimit.f_velo_limit_torque;						/*control input of velocity limit */

				/* Input Saturation */
				if     (t_input > +mid_ctrl_saturation)		{motor_in.mid_ctrl_input = +mid_ctrl_saturation;}
				else if(t_input < -mid_ctrl_saturation)		{motor_in.mid_ctrl_input = -mid_ctrl_saturation;}
				else										{motor_in.mid_ctrl_input = t_input;}
	}

	UpdateScaledData_Mid(&FloatTo2ByteData_mid);
	mid_level_loop_cnt++;
}

static void StateEnable_Ext( )
{
	ExtRoutines(&mid_level_ctrl_task.routine);
}

static void StateError_Run( )
{
}

/* ------------------- SACLING DATA ------------------- */
static void UpdateScaledData_Mid(ScaledData* Scaled2ByteData)
{
	// Scaling for PDO Data
	Scaled2ByteData->P_Vector_Ref = ScaleFloatToInt16(posCtrl.ref, RAD_SCALING_FACTOR);
	Scaled2ByteData->I_Vector_Epsilon = ScaleFloatToInt16(impedanceCtrl.epsilon * (180 / M_PI), I_VECTOR_EPSILON_SCALING_FACTOR);
	Scaled2ByteData->I_Vector_kP = ScaleFloatToInt16(impedanceCtrl.Kp, I_VECTOR_KPKD_GAIN_SCALING_FACTOR);
	Scaled2ByteData->I_Vector_kD = ScaleFloatToInt16(impedanceCtrl.Kd, I_VECTOR_KPKD_GAIN_SCALING_FACTOR);
	Scaled2ByteData->I_Vector_lambda = ScaleFloatToInt16(impedanceCtrl.lambda, I_VECTOR_LAMBDA_SCALING_FACTOR);
}

/* ------------------- KALMAN FILTER ------------------- */
static void Init_Kalman_Filter(KFObject * t_KF_obj, float t_kf_A, float t_kf_B, float t_kf_Q, float t_kf_R)
{
	memset(t_KF_obj, 0, sizeof(KFObject));
	t_KF_obj->kf_A = t_kf_A;
	t_KF_obj->kf_B = t_kf_B;
	t_KF_obj->kf_Q = t_kf_Q;
	t_KF_obj->kf_R = t_kf_R;
	t_KF_obj->kf_P = 1;
	t_KF_obj->kf_y = 0;
}

static int Run_Kalman_Filter(KFObject * t_KF_obj, float t_u, float t_y)
{
	if(isnan(t_KF_obj->kf_y)) t_KF_obj->kf_y = t_y;

	// (STEP1) Model-based Prediction
	t_KF_obj->kf_y = t_KF_obj->kf_A * t_KF_obj->kf_y + t_KF_obj->kf_B * t_u;

	// (STEP2) Project the error covariance ahead
	t_KF_obj->kf_P = t_KF_obj->kf_A * t_KF_obj->kf_P + t_KF_obj->kf_A + t_KF_obj->kf_Q;

	// (STEP3) Update the Kalman Gain
	if ((t_KF_obj->kf_P + t_KF_obj->kf_R)!= 0) {
		t_KF_obj->kf_K = t_KF_obj->kf_P / (t_KF_obj->kf_P + t_KF_obj->kf_R);
	} else {
		// TODO: Handle division by zero
		t_KF_obj->kf_K = 0.0; // 또는 적절한 오류 처리 코드
	}

	// (STEP4) Update the estimate via 'y'
	t_KF_obj->kf_y = t_KF_obj->kf_y + t_KF_obj->kf_K * (t_y - t_KF_obj->kf_y);

	// (STEP5) Update the Error Covariance
	t_KF_obj->kf_P = (1 - t_KF_obj->kf_K) * t_KF_obj->kf_P;

	/* (end) implement here */

	return 0;
}

/* ------------------- GET POS & VEL ------------------- */
static void Init_Position_Velocity()
{
	memset(&mid_level_state, 0, sizeof(mid_level_state));
}

static void Init_Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc, MidLevelState* t_mid_level_state)
{
	float t_position = 0.0;

	if (t_inc_enc->resolution * motor_properties.gear_ratio != 0) {
		t_position = motor_setting.commutation_set.ma_dir * (t_inc_enc->userCnt / (t_inc_enc->resolution * motor_properties.gear_ratio) * 2 * M_PI); // in rad
	} else {
		t_position = 0.0;
	}
	t_mid_level_state->position = t_position + t_mid_level_state->position_offset;
	t_mid_level_state->position_f = t_mid_level_state->position;
	t_mid_level_state->velocity_raw = 0;
	t_mid_level_state->velocity_final = 0;
}

static void Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc, MidLevelState* t_mid_level_state)
{
	float t_position = 0.0;

	// 분모가 0인지 확인합니다.
	if (t_inc_enc->resolution * motor_properties.gear_ratio != 0) {
		t_position = motor_setting.commutation_set.ma_dir * (t_inc_enc->userCnt / (t_inc_enc->resolution * motor_properties.gear_ratio) * 2 * M_PI); // in rad
	} else {
		// TODO: Handle division by zero
		t_position = 0.0; // 또는 적절한 오류 처리 코드
	}

	t_mid_level_state->position_f = t_mid_level_state->position;
	t_mid_level_state->position = t_position + t_mid_level_state->position_offset;

	/*Low Pass Filter*/
	t_mid_level_state->velocity_raw = (t_mid_level_state->position - t_mid_level_state->position_f) * MID_LEVEL_CONTROL_FREQUENCY;

	/* Velocity Estimator */
	if (advanced_friction_id.state == 1) {
		// Friction ID -> 30Hz LPF
		t_mid_level_state->velocity_final = 0.8282 * t_mid_level_state->velocity_final + 0.1718 * t_mid_level_state->velocity_raw;
	} else if (veObj.type == e_VE_RAW) {

		veObj.velocity = t_mid_level_state->velocity_raw;
		t_mid_level_state->velocity_final = veObj.velocity;

	} else if ((veObj.type >= e_VE_LPF_300) && (veObj.type <= e_VE_LPF_500)) {

		veObj.vel_save[0] = t_mid_level_state->velocity_raw;
		veObj.velocity = veObj.lpf_a * veObj.vel_save[1] + veObj.lpf_b * veObj.vel_save[0];
		veObj.vel_save[1] = veObj.velocity;
		t_mid_level_state->velocity_final = veObj.velocity;

	} else if (veObj.type == e_VE_MOVING_AVR) {

		veObj.pos_save[0] = t_mid_level_state->position;

		// 분모가 0인지 확인합니다.
		if (veObj.masking_size != 0) {
			veObj.velocity = (veObj.pos_save[0] - veObj.pos_save[veObj.masking_size]) / (veObj.masking_size * MID_LEVEL_CONTROL_PERIOD);
		} else {
			// TODO: Handle division by zero
			veObj.velocity = 0.0; // 또는 적절한 오류 처리 코드
		}

		for (int i = veObj.masking_size; i > 0; --i) {
			veObj.pos_save[i] = veObj.pos_save[i-1];
		}
		t_mid_level_state->velocity_final = veObj.velocity;

	} else if (veObj.type == e_VE_KALMAN) {
		Run_Kalman_Filter(&veObj.kf_obj, (motor_in.mid_ctrl_input + motor_in.irc_input + motor_in.auxiliary_input + motor_in.f_vector_input), t_mid_level_state->velocity_raw);
		veObj.velocity = veObj.kf_obj.kf_y;
		t_mid_level_state->velocity_final = veObj.kf_obj.kf_y;
	} else {
		t_mid_level_state->velocity_final = t_mid_level_state->velocity_raw;
	}
}

/* ------------------- INIT F VECTOR MODES ------------------- */
static void Init_F_Vector_Modes()
{
	static double t_temp1 = 0.0, t_temp2 = 0.0, t_temp3 = 0.0;

	float t_tp[10] = {0.1, 0.2, 0.3, 0.5, 0.75, 1, 2, 3, 4, 5};
	memcpy(fvectorObj.tp, t_tp, sizeof(t_tp));

	for(int i = 0; i < F_MODE_NUM; ++i) {
		fvectorObj.mode_param[i].tp = t_tp[i];
		fvectorObj.mode_param[i].wn = 1 / fvectorObj.mode_param[i].tp;

		t_temp1 = (fvectorObj.mode_param[i].wn * MID_LEVEL_CONTROL_PERIOD + 2);
		t_temp2 = (fvectorObj.mode_param[i].wn * MID_LEVEL_CONTROL_PERIOD - 2);
		t_temp3 = MID_LEVEL_CONTROL_PERIOD * fvectorObj.mode_param[i].wn * exp(1);

		fvectorObj.mode_param[i].b0 = t_temp1*t_temp1;
		fvectorObj.mode_param[i].b1 = -2*t_temp2/t_temp1;
		fvectorObj.mode_param[i].b2 = -(t_temp2*t_temp2) / fvectorObj.mode_param[i].b0;
		fvectorObj.mode_param[i].a0 = t_temp3 / fvectorObj.mode_param[i].b0;
		fvectorObj.mode_param[i].a1 = 2 * t_temp3 / fvectorObj.mode_param[i].b0;
		fvectorObj.mode_param[i].a2 = fvectorObj.mode_param[i].a0;
	}
}

/* ------------------- SDO CALLBACK ------------------- */
static void Set_IRC_Numerator_Length(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&IRC.numerator_length, req->data, 1);

	memset(&IRC.irc_num, 0, sizeof(IRC.irc_num));

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_IRC_Denominator_Length(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&IRC.denominator_length, req->data, 1);

	memset(&IRC.irc_den, 0, sizeof(IRC.irc_den));

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_IRC_Numerator(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&IRC.irc_num, req->data, 4*IRC.numerator_length);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_IRC_Denominator(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&IRC.irc_den, req->data, 4*IRC.denominator_length);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_IRC_Saturation(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&IRC.saturation, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Vel_Ctrl_BW(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&velCtrl.Ctrl_BW_Hz, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Vel_Ctrl_PGain(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&velCtrl.Kp, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Vel_Ctrl_IGain(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&velCtrl.Ki, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Pos_Ctrl_Input_Penalty(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&posCtrl.R, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Pos_Ctrl_PGain(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&posCtrl.Kp, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Pos_Ctrl_DGain(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&posCtrl.Kd, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Mid_Ctrl_Saturation(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&mid_ctrl_saturation, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_IOIF_IncEnc_t_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
	IOIF_SetIncOffset(IOIF_TIM5, &inc1KhzObj);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_IOIF_AbsEnc1_t_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	float t_des_angle;
	memcpy(&t_des_angle, t_req->data, 4);

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);

	AbsObj1.offset = AbsObj1.offset + AbsObj1.sign*(AbsObj1.posDeg[0] - t_des_angle);
	if (AbsObj1.offset > 360)  AbsObj1.offset = AbsObj1.offset - 360;
	if (AbsObj1.offset < -360) AbsObj1.offset = AbsObj1.offset + 360;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}


static void Set_IOIF_AbsEnc1_t_Sign(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	static uint16_t t_identifier = 0;
	static uint8_t t_sign = 0;

	memcpy(&t_sign, t_req->data, 1);

	if(t_sign == 2){
		AbsObj1.sign = -1;
	}
	else{
		IOIF_SetAbsSign(&AbsObj1);
	}

	t_identifier = GUI_SYNC|ABSENC1_SIGN_SET;
	Send_MSG(t_identifier, &t_sign, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_IOIF_AbsEnc2_t_Offset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	float t_des_angle;
	memcpy(&t_des_angle, t_req->data, 4);

	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);

	AbsObj2.offset = AbsObj2.offset + AbsObj2.sign*(AbsObj2.posDeg[0] - t_des_angle);

	if (AbsObj2.offset > 360)  AbsObj2.offset = AbsObj2.offset - 360;
	if (AbsObj2.offset < -360) AbsObj2.offset = AbsObj2.offset + 360;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}


static void Set_IOIF_AbsEnc2_t_Sign(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	static uint16_t t_identifier = 0;
	static uint8_t t_sign = 0;

	memcpy(&t_sign, t_req->data, 1);

	if(t_sign == 2){
		AbsObj2.sign = -1;
	}
	else{
	IOIF_SetAbsSign(&AbsObj2);
	}

	t_identifier = GUI_SYNC|ABSENC2_SIGN_SET;
	Send_MSG(t_identifier, &t_sign, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_DOB_Q_BW(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&posDOB.wc_Q, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_DOB_GQ_Num(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memset(posDOB.gq_num, 0, sizeof(posDOB.gq_num));

	memcpy(&posDOB.gq_num_length, t_req->data, 4);
	memcpy(posDOB.gq_num, ((float*)t_req->data)+1, posDOB.gq_num_length*4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_DOB_GQ_Den(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memset(posDOB.gq_den, 0, sizeof(posDOB.gq_den));

	memcpy(&posDOB.gq_den_length, t_req->data, 4);
	memcpy(posDOB.gq_den, ((float*)t_req->data)+1, posDOB.gq_den_length*4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_DOB_Q_Num(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memset(posDOB.q_num, 0, sizeof(posDOB.q_num));

	memcpy(&posDOB.q_num_length, t_req->data, 4);
	memcpy(posDOB.q_num, ((float*)t_req->data)+1, posDOB.q_num_length*4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_DOB_Q_Den(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memset(posDOB.q_den, 0, sizeof(posDOB.q_den));

	memcpy(&posDOB.q_den_length, t_req->data, 4);
	memcpy(posDOB.q_den, ((float*)t_req->data)+1, posDOB.q_den_length*4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_DOB_Saturation(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&posDOB.saturation, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Current_Periodic_Sig_Info(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&cur_periodic_sig.amp, 		t_req->data, 			 4);
	memcpy(&cur_periodic_sig.freq, 		((float*)t_req->data)+1, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Periodic_Sig_Info(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&vel_periodic_sig.amp, 		t_req->data, 			 4);
	memcpy(&vel_periodic_sig.freq, 		((float*)t_req->data)+1, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Position_Periodic_Sig_Info(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&pos_periodic_sig.amp, 		t_req->data, 4);
	memcpy(&pos_periodic_sig.freq, 		((float*)t_req->data)+1, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_VSD_Stiffness(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&VSD.lower_stiffness, t_req->data, 8);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_VSD_Damper(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&VSD.lower_damper, t_req->data, 8);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_VSD_Damped_Range(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&VSD.lower_damped_range, t_req->data, 8);

	VSD.lower_damper_origin = VSD.lower_limit + VSD.lower_damped_range;
	VSD.upper_damper_origin = VSD.upper_limit - VSD.upper_damped_range;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_VSD_Stiff_Range(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&VSD.lower_stiff_range, t_req->data, 8);

	VSD.lower_spring_origin = VSD.lower_limit + VSD.lower_stiff_range;
	VSD.upper_spring_origin = VSD.upper_limit - VSD.upper_stiff_range;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_VSD_Upper_Limit(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	static uint16_t t_identifier = 0;

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);

	if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT)
		VSD.upper_limit = AbsObj1.posDeg[0];
	else if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT)
		VSD.upper_limit = AbsObj2.posDeg[0];


	VSD.upper_damper_origin = VSD.upper_limit - VSD.upper_damped_range;
	VSD.upper_spring_origin = VSD.upper_limit - VSD.upper_stiff_range;

	t_identifier = GUI_SYNC|GET_VSD_UPPER_LIMIT;
	Send_MSG(t_identifier, (uint8_t*)(&VSD.upper_limit), 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_VSD_Lower_Limit(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	static uint16_t t_identifier = 0;

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);

	if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT)
		VSD.lower_limit = AbsObj1.posDeg[0];
	else if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT)
		VSD.lower_limit = AbsObj2.posDeg[0];


	VSD.lower_damper_origin = VSD.lower_limit + VSD.lower_damped_range;
	VSD.lower_spring_origin = VSD.lower_limit + VSD.lower_stiff_range;

	t_identifier = GUI_SYNC|GET_VSD_LOWER_LIMIT;
	Send_MSG(t_identifier, (uint8_t*)(&VSD.lower_limit), 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_VSD_Saturation(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&VSD.saturation, t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

/******************** P Vector Setting ********************/
static void Set_P_Vector_Yd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&pvectorObj.p_buffer[pvectorObj.N].yd , req->data, 2);
	if(FSM_curr >= 65 && FSM_curr <= 75){
		pvectorObj.yd_f = mid_level_state.position;
	}
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_L(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&pvectorObj.p_buffer[pvectorObj.N].L , req->data, 2);

	pvectorObj.durationCompleted = 0;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_S0(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&pvectorObj.p_buffer[pvectorObj.N].s0 , req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Sd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&pvectorObj.p_buffer[pvectorObj.N].sd , req->data, 1);

	pvectorObj.N++;
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Reset(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	Reset_P_Vector();

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

/******************** F Vector Setting ********************/
static void Set_F_Vector_ModeIDX(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	for (int8_t t_idx = 0; t_idx < F_VECTOR_BUFF_SIZE; t_idx++) {
		if (fvectorObj.f_buffer[t_idx].is_full == 0) {
			fvectorObj.temp_idx = t_idx;
			break;
		}
	}

	memcpy(&fvectorObj.f_buffer[fvectorObj.temp_idx].mode_idx , req->data, 1);

	float t_tp = fvectorObj.mode_param[fvectorObj.f_buffer[fvectorObj.temp_idx].mode_idx].tp;
	fvectorObj.f_buffer[fvectorObj.temp_idx].t_end = (uint32_t)(15 * t_tp * MID_LEVEL_CONTROL_FREQUENCY);

	torque_inter_sw = 0;
	torque_inter_gain = 1;
	torque_inter_after_0 = 1;

	comp_inter_sw = 0;
	comp_inter_gain = 1;
	comp_inter_after_0 = 1;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_F_Vector_TauMax(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&fvectorObj.f_buffer[fvectorObj.temp_idx].tau_max , req->data, 2);

	fvectorObj.f_buffer[fvectorObj.temp_idx].tau_max = fvectorObj.f_buffer[fvectorObj.temp_idx].tau_max / motor_properties.gear_ratio / motor_properties.Kt;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_F_Vector_Delay(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&fvectorObj.f_buffer[fvectorObj.temp_idx].delay , req->data, 2);
	fvectorObj.f_buffer[fvectorObj.temp_idx].time_stamp = 0;
	fvectorObj.f_buffer[fvectorObj.temp_idx].is_full = 1;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_F_Vector_ZERO(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	motor_in.f_vector_input = 0;
	for (int i = 0; i < F_VECTOR_BUFF_SIZE; ++i) {
		fvectorObj.f_buffer[i].mode_idx = 0;
		fvectorObj.f_buffer[i].tau_max = 0;
		fvectorObj.f_buffer[i].delay = 0;
		fvectorObj.f_buffer[i].u = 0;
		fvectorObj.f_buffer[i].u_old1 = 0;
		fvectorObj.f_buffer[i].u_old2 = 0;
		fvectorObj.f_buffer[i].tau = 0;
		fvectorObj.f_buffer[i].tau_old1 = 0;
		fvectorObj.f_buffer[i].tau_old2 = 0;
		fvectorObj.f_buffer[i].t_end = 0;
		fvectorObj.f_buffer[i].time_stamp = 0;
		fvectorObj.f_buffer[i].is_full = 0;
	}

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

/******************** I Vector Setting ********************/

static void Set_I_Vector_Epsilon(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].epsilon_target , req->data, 1);
	} else if (impedanceCtrl.option == 1) {
		memcpy(&impedanceCtrl.opt1_i_buffer.epsilon_target , req->data, 1);
	}

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kp(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].Kp_target , req->data, 1);
	} else if (impedanceCtrl.option == 1) {
		memcpy(&impedanceCtrl.opt1_i_buffer.Kp_target , req->data, 1);
	}

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].Kd_target , req->data, 1);
	} else if (impedanceCtrl.option == 1) {
		memcpy(&impedanceCtrl.opt1_i_buffer.Kd_target , req->data, 1);
	}

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Lambda(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].lambda_target, req->data, 1);
	} else if (impedanceCtrl.option == 1) {
		memcpy(&impedanceCtrl.opt1_i_buffer.lambda_target , req->data, 1);
	}

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Duration(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].duration , req->data, 2);
	} else if (impedanceCtrl.option == 1) {
		memcpy(&impedanceCtrl.opt1_i_buffer.duration , req->data, 2);
	}

	impedanceCtrl.N++;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kp_Max(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&impedanceCtrl.Kp_max, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Kd_Max(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&impedanceCtrl.Kd_max, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_I_Vector_Option(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&impedanceCtrl.option, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

// input: yd -> update offset to set y = yd
static void Set_Desired_Mech_Angle(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&mid_level_state.init_position, req->data, 4);

	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
	Get_Position_Velocity(&inc1KhzObj, &mid_level_state);
	mid_level_state.position_offset += mid_level_state.init_position - mid_level_state.position;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_IOIF_AbsEnc1_t_Location(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&AbsObj1.location, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_IOIF_AbsEnc2_t_Location(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&AbsObj2.location, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Feedforward_Num(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memset(posFF.num, 0, sizeof(posFF.num));

	memcpy(&posFF.num_length, t_req->data, 4);
	memcpy(posFF.num, ((float*)t_req->data) + 1, 4 * (uint8_t)(posFF.num_length));

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Feedforward_Den(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memset(posFF.den, 0, sizeof(posFF.den));

	memcpy(&posFF.den_length, t_req->data, 4);
	memcpy(posFF.den, ((float*)t_req->data) + 1, 4 * (uint8_t)(posFF.den_length));

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Estimator(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memset(&veObj.lpf_a, 0, sizeof(float));
	memset(&veObj.lpf_b, 0, sizeof(float));
	memset(veObj.vel_save, 0, sizeof(veObj.vel_save));
	memset(veObj.pos_save, 0, sizeof(veObj.pos_save));
	memset(&veObj.kf_obj, 0, sizeof(veObj.kf_obj));
	veObj.leadlag_a = 0;
	veObj.leadlag_b = 0;
	veObj.masking_size = 0;

	memcpy(&veObj.type, t_req->data, 1);
//	memcpy(&veObj.masking_size, ((uint8_t*)t_req->data)+1, 1);

//	veObj.type = e_VE_KALMAN;

	mid_level_state.position = 0;
	mid_level_state.position_f = 0;
	mid_level_state.velocity_raw = 0;
	mid_level_state.velocity_final = 0;
	motor_in.mid_ctrl_input  = 0;
	motor_in.irc_input = 0;
	motor_in.auxiliary_input = 0;
	motor_in.f_vector_input = 0;

	if(veObj.type == e_VE_LPF_300) {
		veObj.lpf_a = exp(-300*2*M_PI*MID_LEVEL_CONTROL_PERIOD);
		veObj.lpf_b = 1-veObj.lpf_a;
	} else if(veObj.type == e_VE_LPF_400) {
		veObj.lpf_a = exp(-400*2*M_PI*MID_LEVEL_CONTROL_PERIOD);
		veObj.lpf_b = 1-veObj.lpf_a;
	} else if(veObj.type == e_VE_LPF_500) {
		veObj.lpf_a = exp(-500*2*M_PI*MID_LEVEL_CONTROL_PERIOD);
		veObj.lpf_b = 1-veObj.lpf_a;
	} else if(veObj.type == e_VE_KALMAN) {
		if(isnan(motor_properties.a) || isnan(motor_properties.b)) {
			Send_MSG((uint16_t)(GUI_SYNC|VE_KF_SETTING_ERROR), (uint8_t*)0, 1);
		} else if(motor_properties.a == 0 || motor_properties.b == 0) {
			Send_MSG((uint16_t)(GUI_SYNC|VE_KF_SETTING_ERROR), (uint8_t*)0, 1);
		} else {
			Init_Kalman_Filter(&veObj.kf_obj, -motor_properties.a, motor_properties.b, 1, 5);
		}
	}
		Send_MSG((uint16_t)(GUI_SYNC|VELOCITY_ESTIMATOR_SET), (uint8_t*)0, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Estimator_LeadLag(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&veObj.leadlag_a, t_req->data, 4);
	memcpy(&veObj.leadlag_b, ((float*)t_req->data)+1, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_1khz_Enc_Resolution(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&inc1KhzObj.resolution , t_req->data, 2);
	Send_MSG((uint16_t)(GUI_SYNC|INC_ENC_RES_SET_MID), (uint8_t*)0, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_System_ID_SBS_Info(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&sys_id_sbs.fmin, 		t_req->data, 4);
	memcpy(&sys_id_sbs.fmax, 		((float*)t_req->data)+1, 4);
	memcpy(&sys_id_sbs.N_samples, 	((float*)t_req->data)+2, 4);
	memcpy(&sys_id_sbs.N_iter, 		((float*)t_req->data)+3, 4);
	memcpy(&sys_id_sbs.amp, 		((float*)t_req->data)+4, 4);
	memcpy(&sys_id_sbs.offset, 		((float*)t_req->data)+5, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_System_ID_Verification_Mag(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&sys_id_sbs.verify_mag , t_req->data, 4);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

// static void ToggleLED(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
// {
// 	IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13);//LED_BOOT_RED_Pin

// 	t_res->dataSize = 0;
// 	t_res->status = DOP_SDO_SUCC;
// }

static void Set_IOIF_AbsEnc1_Resolution(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&AbsObj1.resolution, req->data, 2);//2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_IOIF_AbsEnc2_Resolution(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&AbsObj2.resolution, req->data, 2);//2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Grav_Comp_Gain(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&gravcomp.grav_gain, req->data, 4);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Vel_Comp_Gain(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&velocitycomp.velocity_gain, req->data, 4);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_IOIF_AbsEnc1_range_offset(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	AbsObj1.rawBitsTo13Bit += AbsObj1.raw_offset;

	if(AbsObj1.rawBitsTo13Bit > 4096)
	{
		AbsObj1.raw_offset = AbsObj1.rawBitsTo13Bit - 4096;
	}
	else if(AbsObj1.rawBitsTo13Bit <= 4096)
	{
		AbsObj1.raw_offset = AbsObj1.rawBitsTo13Bit + 4096;
	}

	MS_enum = SAVE_PROPERTIES;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}


static void Set_IOIF_AbsEnc1_value_offset(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	AbsObj1.offset = AbsObj1.posDegRaw + AbsObj1.sign*40*1.875;

	MS_enum = 2;

	/* position update */
	//mid_level_state.position_offset = (AbsObj1.posDeg[0] / 1.875) * M_PI / 180.0f;
	//Get_Position_Velocity(&inc1KhzObj, &mid_level_state);

	ts_encoder_offset_done = true;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Suit_mode(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&suit_mode, req->data, 1);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Vel_Comp_Satu(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&velocitycomp.velocity_satu, req->data, 4);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Hip_Upper_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&degreelimit.HipRoMUpperLimit, req->data, 4);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Hip_Lower_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&degreelimit.HipRoMLowerLimit, req->data, 4);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Hip_Upper_Vel_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&velolimit.HipFlexionVelLimit, req->data, 4);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_Hip_Lower_Vel_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&velolimit.HipExtensionVelLimit, req->data, 4);
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_FSM_State(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	FSM_prev = FSM_curr;
	memcpy(&FSM_curr, req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}
static void Set_yd_f(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	pvectorObj.yd_f = mid_level_state.position;
	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}
static void Set_SitToStance_Torque(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&SitToStance_Torque, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}
static void Set_StanceToSit_Torque(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&StanceToSit_Torque, req->data, 4);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}
/* ------------------- ROUTINE ------------------- */
/*ROUTINE_ID_MIDLEVEL_POSITION_CTRL*/
static int Ent_Position_Ctrl()
{
	posCtrl.err = 0;
	posCtrl.err_sum = 0;
	posCtrl.err_diff = 0;
	mid_level_state.initial_pos = mid_level_state.position;

	return 0;
}
static int Run_Position_Ctrl()
{
	Run_PID_Control(&posCtrl, posCtrl.ref, mid_level_state.position, MID_LEVEL_CONTROL_PERIOD);
//	Run_PID_Control(&posCtrl, posCtrl.ref*180/M_PI, AbsObj1.posDeg[0]/1.875, MID_LEVEL_CONTROL_PERIOD);
	return 0;
}

/*ROUTINE_ID_MIDLEVEL_VELOCITY_CTRL*/
static int Ent_Velocity_Ctrl()
{
	velCtrl.err = 0;
	velCtrl.err_sum = 0;
	velCtrl.err_diff = 0;

	return 0;
}
static int Run_Velocity_Ctrl()
{
	Run_PID_Control(&velCtrl, velCtrl.ref, mid_level_state.velocity_final, MID_LEVEL_CONTROL_PERIOD);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_IRC*/
static int Ent_Impedance_Reduction()
{
	memset(IRC.irc_input, 0, sizeof(IRC.irc_input));
	memset(IRC.irc_output, 0, sizeof(IRC.irc_output));
	motor_in.irc_input = 0;
	return 0;
}
static int Run_Impedance_Reduction()
{
	double t_vel_term = 0.0, t_cur_term = 0.0, t_input = 0.0;

	/* k-th input (velocity) */
	IRC.irc_input[0] = mid_level_state.velocity_final;

	/* Calculation */
	for (int i = 0; i < IRC.numerator_length; ++i) {
		t_vel_term += IRC.irc_num[i] * IRC.irc_input[i];
	}
	for (int i = 1; i < IRC.denominator_length; ++i) {
		t_cur_term += IRC.irc_den[i] * IRC.irc_output[i];
	}

	t_input = -t_cur_term + t_vel_term;

	/* Saturation */
	if      (t_input >  IRC.saturation)	{t_input =  IRC.saturation;}
	else if (t_input < -IRC.saturation)	{t_input = -IRC.saturation;}

	/* Transfer to Low-level Task*/
	motor_in.irc_input = t_input;
	IRC.irc_output[0] = motor_in.irc_input;

	/* Array Shifting */
	for (int i = IRC.numerator_length-1; i > 0; --i) {
		IRC.irc_input[i] = IRC.irc_input[i-1];
	}
	for (int i = IRC.denominator_length-1; i > 0; --i) {
		IRC.irc_output[i] = IRC.irc_output[i-1];
	}

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_COMPRESSIONAL_VSD*/
static int Ent_Compressional_SpringDamper()
{
	VSD.lower_damper_origin = VSD.lower_limit + VSD.lower_damped_range;
	VSD.lower_spring_origin = VSD.lower_limit + VSD.lower_stiff_range;
	VSD.upper_damper_origin = VSD.upper_limit - VSD.upper_damped_range;
	VSD.upper_spring_origin = VSD.upper_limit - VSD.upper_stiff_range;

	VSD.control_input = 0;
	return 0;
}

static int Run_Compressional_SpringDamper()
{
	VSD.control_input = 0.0;

	float t_posDeg = 0.0;
	float t_velDeg = 0.0;


	if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT) {
		t_posDeg = AbsObj1.posDeg[0];
		t_velDeg = AbsObj1.velDeg;
	} else if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT) {
		t_posDeg = AbsObj2.posDeg[0];
		t_velDeg = AbsObj2.velDeg;
	}

	/* Lower Spring */
	if (t_posDeg < VSD.lower_spring_origin) {
		VSD.control_input += VSD.lower_stiffness * (VSD.lower_spring_origin - t_posDeg);
	}

	/* Lower Damper */
	if (t_posDeg < VSD.lower_damper_origin) {
		if (t_velDeg < 0) {
			VSD.control_input  += -VSD.lower_damper*t_velDeg;
		}
	}

	/* Upper Spring */
	if (t_posDeg > VSD.upper_spring_origin) {
		VSD.control_input += VSD.upper_stiffness * (VSD.upper_spring_origin - t_posDeg);
	}

	/* Upper Damper */
	if (t_posDeg > VSD.upper_damper_origin) {
		if (t_velDeg > 0) {
			VSD.control_input  += -VSD.upper_damper*t_velDeg;
		}
	}

	/* Saturation */
	if      (VSD.control_input > +VSD.saturation)	{VSD.control_input =  VSD.saturation;}
	else if (VSD.control_input < -VSD.saturation)	{VSD.control_input = -VSD.saturation;}

	return 0;
}

static int Ext_Compressional_SpringDamper()
{
	VSD.control_input = 0;
	return 0;
}

/*ROUTINE_ID_MIDLEVEL_COMPRESSIONAL_VSD*/
static int Run_Backlash_Test()
{
	static int flag = 0;

	if (mid_level_state.position > abs(backlash_test.range))
		flag = 1;
	else if (mid_level_state.position < -abs(backlash_test.range))
		flag = 0;

	if      (flag == 0) motor_in.analysis_input = +backlash_test.amplitude;
	else if (flag == 1) motor_in.analysis_input = -backlash_test.amplitude;

	return 0;
}

// Disturbance Observer
static int Ent_Disturbance_Obs()
{
	memset(posDOB.q_in, 0, sizeof(posDOB.q_in));
	memset(posDOB.gq_in, 0, sizeof(posDOB.gq_in));
	memset(posDOB.q_out, 0, sizeof(posDOB.q_out));
	memset(posDOB.gq_out, 0, sizeof(posDOB.gq_out));

	posDOB.initial_pos = mid_level_state.position;
	posDOB.time_stamp = 0;
	posDOB.transition_time = CONTROLLER_TRANSITION_DURATION;
	return 0;
}

static int Run_Disturbance_Obs()
{
	float gq_out_term = 0.0;
	float gq_in_term = 0.0;
	float q_out_term = 0.0;
	float q_in_term = 0.0;

	/* k-th input */
	posDOB.q_in[0] = motor_in.mid_ctrl_input;
	posDOB.gq_in[0] = mid_level_state.position - posDOB.initial_pos;

	/* G^-1*Q  Calculation */
	for (int i = 1; i < posDOB.gq_den_length; ++i) {
		gq_out_term += posDOB.gq_den[i]*posDOB.gq_out[i];
	}
	for (int i = 0; i < posDOB.gq_num_length; ++i) {
		gq_in_term += posDOB.gq_num[i]*posDOB.gq_in[i];
	}

	posDOB.gq_out[0] = -gq_out_term + gq_in_term;

	/* Q Calculation */
	for (int i = 1; i < posDOB.q_den_length; ++i) {
		q_out_term += posDOB.q_den[i]*posDOB.q_out[i];
	}
	for (int i = 0; i < posDOB.q_num_length; ++i) {
		q_in_term += posDOB.q_num[i]*posDOB.q_in[i];
	}

	posDOB.q_out[0] = -q_out_term + q_in_term;

	/* Get Disturbance */
	posDOB.disturbance = posDOB.gq_out[0] - posDOB.q_out[0];

	/* Saturation */
	if		(posDOB.disturbance >  posDOB.saturation)	{posDOB.control_input = +posDOB.saturation;}
	else if (posDOB.disturbance < -posDOB.saturation)	{posDOB.control_input = -posDOB.saturation;}
	else 												{posDOB.control_input = +posDOB.disturbance;}

	/* Array Shifting */
	for (int i = posDOB.gq_den_length-1; i > 0; --i) {
		posDOB.gq_out[i] = posDOB.gq_out[i-1];
	}
	for (int i = posDOB.gq_num_length-1; i > 0; --i) {
		posDOB.gq_in[i] = posDOB.gq_in[i-1];
	}
	for (int i = posDOB.q_den_length-1; i > 0; --i) {
		posDOB.q_out[i] = posDOB.q_out[i-1];
	}
	for (int i = posDOB.q_num_length-1; i > 0; --i) {
		posDOB.q_in[i] = posDOB.q_in[i-1];
	}

	return 0;
}

// Advanced System ID SBS
static int Ent_Mech_SystemID_SBS()
{
	// make frequency samples
	float f_sample = 0.0;
	float log_fmin = log10(sys_id_sbs.fmin);
	float log_fmax = log10(sys_id_sbs.fmax);
	float gap = 0.0;

	if ((sys_id_sbs.N_samples - 1) != 0) {
		gap = (log_fmax - log_fmin) / (sys_id_sbs.N_samples - 1);
	} else {
		// TODO: Handle division by zero
		gap = 0.0;
		// return -1;
	}

	// frequency initialization
	sys_id_sbs.current_f = sys_id_sbs.fmin;

	// make frequency samples that evenly distributed in log-scale
	sys_id_sbs.f_samples = (float*)malloc(sizeof(float) * sys_id_sbs.N_samples);

	if (sys_id_sbs.f_samples != NULL) {
		for (int i = 0; i < sys_id_sbs.N_samples; i++) {
			f_sample = log_fmin + gap * i;
			sys_id_sbs.f_samples[i] = powf(10, f_sample);
		}
	}
	sys_id_sbs.sys_id_cnt = 0;
	sys_id_sbs.f_cnt = 0;
	sys_id_sbs.done = 0;
	return 0;
}

static int Run_Mech_SystemID_SBS()
{
	/* End of Frequency Index */
	sys_id_sbs.current_f = sys_id_sbs.f_samples[sys_id_sbs.f_cnt];

	motor_in.mid_id_process_input = sys_id_sbs.amp * sin(2 * M_PI * sys_id_sbs.f_samples[sys_id_sbs.f_cnt] * sys_id_sbs.sys_id_cnt * MID_LEVEL_CONTROL_PERIOD) + sys_id_sbs.offset;

	if (sys_id_sbs.sys_id_cnt >= MID_LEVEL_CONTROL_FREQUENCY * sys_id_sbs.N_iter * (1/sys_id_sbs.current_f)) {
		sys_id_sbs.sys_id_cnt = 0;
		sys_id_sbs.f_cnt++;
	} else {
		sys_id_sbs.sys_id_cnt++;
	}

	if (sys_id_sbs.f_cnt == sys_id_sbs.N_samples) {
		motor_in.mid_id_process_input = 0;
		sys_id_sbs.done++;

		if (sys_id_sbs.done >= 1) {
			StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
			StateTransition(&msgHdlrTask.stateMachine,       TASK_STATE_STANDBY);
		}
	}

	return 0;
}

static int Ext_Mech_SystemID_SBS()
{
	sys_id_sbs.sys_id_cnt = 0;
	sys_id_sbs.f_cnt = 0;

	free(sys_id_sbs.f_samples);

	/* Done Signal */
	Send_MSG((uint16_t)(GUI_SYNC|MECH_SYS_ID_SBS_DONE), (uint8_t*)0, 1);
	StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
	StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
	StateTransition(&msgHdlrTask.stateMachine,       TASK_STATE_STANDBY);

	return 0;
}

/***********************************************************************************************************/
static int Ent_P_Vector_Decoder()
{
	memset(&pvectorObj, 0, sizeof(P_Vector_Decoder));
	pvectorObj.yd_f = mid_level_state.position;

	return 0;
}

static int Run_P_Vector_Decoder()
{
	if ((pvectorObj.N > 0) && (pvectorObj.ON == 0)) {
		// FIFO process
		// double yd = (double)pvectorObj.p_buffer[0].yd*0.002;
		// double y0 = pvectorObj.yd_f;
		// double s0 =  (double)pvectorObj.p_buffer[0].s0;
		// double sd = -(double)pvectorObj.p_buffer[0].sd;
		double yd = (double)pvectorObj.p_buffer[0].yd*0.002*0.1;
		double y0 = (double)mid_level_state.position;
		double s0 = (double)pvectorObj.p_buffer[0].s0*0.1;
		double sd = -(double)pvectorObj.p_buffer[0].sd*0.1;
		pvectorObj.yd_f = yd;

		double e  = yd - y0;

		pvectorObj.a0 = y0;
		pvectorObj.a2 =                 0.5*s0 * e;
		pvectorObj.a3 = (10 - 1.5*s0 + 0.5*sd) * e;
		pvectorObj.a4 =    (-15 + 1.5*s0 - sd) * e;
		pvectorObj.a5 =  (6 - 0.5*s0 + 0.5*sd) * e;

		pvectorObj.L     = (float)pvectorObj.p_buffer[0].L;
		if (pvectorObj.L != 0) {
			pvectorObj.L_inv = 1/pvectorObj.L;
		} else {
			// TODO: Handle division by zero
			pvectorObj.L_inv = 0.0;
		}

		pvectorObj.count = 0;
		pvectorObj.ON    = 1;

		if (pvectorObj.N == 1) {
			y0 = (double)pvectorObj.p_buffer[0].yd * 0.002;
			pvectorObj.b0 = y0;
			pvectorObj.b2 = 0;
			pvectorObj.b3 = 0;
			pvectorObj.b4 = 0;
			pvectorObj.b5 = 0;

		} else if (pvectorObj.N > 1) {
			yd = (double)pvectorObj.p_buffer[1].yd * 0.002;
			y0 = (double)pvectorObj.p_buffer[0].yd * 0.002;
			s0 =  (double)pvectorObj.p_buffer[1].s0;
			sd = -(double)pvectorObj.p_buffer[1].sd;
			e  = yd - y0;

			pvectorObj.b0 = y0;
			pvectorObj.b2 =                 0.5*s0 * e;
			pvectorObj.b3 = (10 - 1.5*s0 + 0.5*sd) * e;
			pvectorObj.b4 =    (-15 + 1.5*s0 - sd) * e;
			pvectorObj.b5 =  (6 - 0.5*s0 + 0.5*sd) * e;
		}
	}

	if (pvectorObj.ON == 1) {
		pvectorObj.t1 = (float)pvectorObj.count * pvectorObj.L_inv;	// tau^1
		if (pvectorObj.t1 > 1) pvectorObj.t1 = 1;					// saturation for safety
		pvectorObj.t2 = pvectorObj.t1 * pvectorObj.t1;               // tau^2
		pvectorObj.t3 = pvectorObj.t2 * pvectorObj.t1;               // tau^3
		pvectorObj.t4 = pvectorObj.t3 * pvectorObj.t1;               // tau^4
		pvectorObj.t5 = pvectorObj.t4 * pvectorObj.t1;               // tau^5

		posCtrl.ref = pvectorObj.a0 + //
				pvectorObj.a2*pvectorObj.t2 + //
				pvectorObj.a3*pvectorObj.t3 + //
				pvectorObj.a4*pvectorObj.t4 + //
				pvectorObj.a5*pvectorObj.t5;  //

		if (pvectorObj.count < (pvectorObj.L - 2)) {
			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			t_t1 = ((double)pvectorObj.count + 1) * pvectorObj.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl.ref1 = pvectorObj.a0 + //
					pvectorObj.a2 * t_t2 + //
					pvectorObj.a3 * t_t3 + //
					pvectorObj.a4 * t_t4 + //
					pvectorObj.a5 * t_t5;  //

			t_t1 = ((double)pvectorObj.count + 2) * pvectorObj.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl.ref2 = pvectorObj.a0 + //
					pvectorObj.a2 * t_t2 + //
					pvectorObj.a3 * t_t3 + //
					pvectorObj.a4 * t_t4 + //
					pvectorObj.a5 * t_t5;  //
		} else if (pvectorObj.count == (pvectorObj.L - 2)) {
			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			t_t1 = ((double)pvectorObj.count + 1) * pvectorObj.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl.ref1 = pvectorObj.a0 + //
					pvectorObj.a2 * t_t2 + //
					pvectorObj.a3 * t_t3 + //
					pvectorObj.a4 * t_t4 + //
					pvectorObj.a5 * t_t5;  //

			posCtrl.ref2 = (double)pvectorObj.p_buffer[0].yd * 0.002;

		} else if (pvectorObj.count == (pvectorObj.L - 1)) {
			posCtrl.ref1 = (double)pvectorObj.p_buffer[0].yd * 0.002;

			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			if (pvectorObj.p_buffer[1].L != 0) {
				t_t1 = (double)1 / pvectorObj.p_buffer[1].L;
			} else {
				// TODO: Handle division by zero
				t_t1 = 0.0;
			}

			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl.ref2 = pvectorObj.b0 + //
					pvectorObj.b2 * t_t2 + //
					pvectorObj.b3 * t_t3 + //
					pvectorObj.b4 * t_t4 + //
					pvectorObj.b5 * t_t5;  //
		}

		pvectorObj.count++;

		if (pvectorObj.count >= pvectorObj.L) {
			pvectorObj.ON = 0;
			pvectorObj.count  = 0;

			// FIFO shifting
			for (int i = 0; i < pvectorObj.N; i++) {
				memcpy(&pvectorObj.p_buffer[i], &pvectorObj.p_buffer[i+1], sizeof(P_Vector));
			}
			pvectorObj.N--;
			
			// TODO : Send Msg Duration Completed!!
			pvectorObj.durationCompleted = 1;
			SendDurationCompletedFlag();
		}
	}

	return 0;
}

static int Ext_P_Vector_Decoder()
{
	for (int i = 0; i < P_VECTOR_BUFF_SIZE; ++i) {
		pvectorObj.p_buffer[i].yd = 0;
		pvectorObj.p_buffer[i].L = 0;
		pvectorObj.p_buffer[i].s0 = 0;
		pvectorObj.p_buffer[i].sd = 0;
	}
	return 0;
}

static void Reset_P_Vector(void)
{
    // pvectorObj 변수 0으로 초기화
    pvectorObj.N = 0;
	memset(pvectorObj.p_buffer, 0, sizeof(pvectorObj.p_buffer));
    // 리셋 후의 시작 위치는 현재 모터의 실제 위치로 설정
    pvectorObj.yd_f = mid_level_state.position; 
}

/* F Vector-based Torque Generation */
static int Run_F_Vector_Decoder()
{
	fvectorObj.input = 0;

	for (int i = 0; i < F_VECTOR_BUFF_SIZE; i++) {
		// Step 1. Check time delay
		if (fvectorObj.f_buffer[i].is_full)	{
			if (fvectorObj.f_buffer[i].time_stamp == fvectorObj.f_buffer[i].delay) {
				fvectorObj.f_buffer[i].u = fvectorObj.f_buffer[i].tau_max * 0.01;
				//fvectorObj.f_buffer[i].time_stamp = 0; //reset time stamp, and restart to measure duration
			}
		}

		// (Step2) Calculate torque and sum
		uint8_t t_idx = fvectorObj.f_buffer[i].mode_idx;

		fvectorObj.f_buffer[i].tau = fvectorObj.mode_param[t_idx].b1 * fvectorObj.f_buffer[i].tau_old1 + \
				fvectorObj.mode_param[t_idx].b2 * fvectorObj.f_buffer[i].tau_old2 + \
				fvectorObj.mode_param[t_idx].a0 * fvectorObj.f_buffer[i].u + \
				fvectorObj.mode_param[t_idx].a1 * fvectorObj.f_buffer[i].u_old1 + \
				fvectorObj.mode_param[t_idx].a2 * fvectorObj.f_buffer[i].u_old2;

		fvectorObj.f_buffer[i].tau_old2 = fvectorObj.f_buffer[i].tau_old1;
		fvectorObj.f_buffer[i].tau_old1 = fvectorObj.f_buffer[i].tau;
		fvectorObj.f_buffer[i].u_old2   = fvectorObj.f_buffer[i].u_old1;
		fvectorObj.f_buffer[i].u_old1 = fvectorObj.f_buffer[i].u;
		fvectorObj.f_buffer[i].u = 0;

		fvectorObj.input += fvectorObj.f_buffer[i].tau;

		// Step 3. Update times
		if (fvectorObj.f_buffer[i].is_full)	{
			fvectorObj.f_buffer[i].time_stamp++;
		}

		// Step 4. Check F-vector erase condition
		if (fvectorObj.f_buffer[i].time_stamp >= (fvectorObj.f_buffer[i].t_end + fvectorObj.f_buffer[i].delay))	{
			memset(&fvectorObj.f_buffer[i], 0, sizeof(F_Vector));
			fvectorObj.f_buffer[i].is_full = 0;
		}
	}

	if(torque_inter_sw == 1)
	{
		motor_in.f_vector_input = fvectorObj.input * torque_inter_gain;

		if(torque_inter_gain > 0)
		{
			torque_inter_gain -= 0.002;
		}
	}
	else
	{
		motor_in.f_vector_input = fvectorObj.input;
	}


	return 0;
}

static int Ext_F_Vector_Decoder()
{
	motor_in.f_vector_input = 0;

	for (int i = 0; i < F_VECTOR_BUFF_SIZE; ++i) {
		fvectorObj.f_buffer[i].u = 0;
		fvectorObj.f_buffer[i].u_old1 = 0;
		fvectorObj.f_buffer[i].u_old2 = 0;
		fvectorObj.f_buffer[i].tau = 0;
		fvectorObj.f_buffer[i].tau_old1 = 0;
		fvectorObj.f_buffer[i].tau_old2 = 0;
	}
	return 0;
}

static int Ent_Feedforward_Filter()
{
	memset(posFF.in, 0, sizeof(posFF.in));
	memset(posFF.out, 0, sizeof(posFF.out));
	posFF.diff = posFF.num_length - posFF.den_length;

	posFF.control_input = 0;

	return 0;
}

static int Run_Feedforward_Filter()
{
	double t_in_term = 0.0, t_out_term = 0.0;

	for (int i = 0; i < posFF.num_length; ++i) {
		t_in_term += (double)posFF.num[i]*(double)posFF.in[i];
	}
	for (int i = 1; i < posFF.den_length; ++i) {
		t_out_term += (double)posFF.den[i]*(double)posFF.out[i];
	}
	posFF.control_input = -t_out_term + t_in_term;
	posFF.out[0] = posFF.control_input;

	/* Array Shifting */
	for (int i = posFF.num_length-1; i > 0; --i) {
		posFF.in[i] = posFF.in[i-1];
	}
	for (int i = posFF.den_length-1; i > 0; --i) {
		posFF.out[i] = posFF.out[i-1];
	}

	return 0;
}

static int Ext_Feedforward_Filter()
{
	posFF.diff = 0;
	return 0;
}



static int Run_System_ID_Verify()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[3] = {0};

	if (mid_level_loop_cnt <= 2000) {
		motor_in.mid_id_process_input = sys_id_sbs.verify_mag*rgs_input[mid_level_loop_cnt];

		memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
		memcpy(&t_temp_arr[1], &motor_in.mid_id_process_input, 4);
		memcpy(&t_temp_arr[2], &mid_level_state.velocity_raw, 4);

		t_identifier = GUI_SYNC|GET_SYSTEM_ID_VERIFY;
		Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 12);
	} else {
		StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		motor_in.mid_id_process_input = 0;
	}

	return 0;
}

static int Ent_Gravity_Compensation()
{
	gravcomp.grav_comp_torque = 0;
	gravcomp.f_grav_comp_torque = 0;

	gravcomp.grav_gain = 0;
	gravcomp.grav_alpha = 0.99;

	return 0;
}

static int Run_Gravity_Compensation()
{

	if((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal) < 30)
	{
		gravcomp.keep_move = 0;
		gravcomp.move_stop_sw = 0;

	}
	else if((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal) > 30 && mid_level_state.velocity_raw < - 0.1){
		if(gravcomp.keep_move == 1)
		{
			gravcomp.move_stop_sw = 1;
		}
	}

	if(suit_mode != SUIT_AQUA)//gravity compensation algorithm change as mode changing
	{
		if ((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal) > 0 && mid_level_state.velocity_raw > 0 && gravcomp.move_stop_sw == 0) {
			//		torque_grav_comp = grav_gain * (sin((AbsObj1.posDegMultiTurn[0]/1.875 - imuVQFDataObj.angle_bodyframe.sagittal)*M_PI/180)); //k10

			gravcomp.grav_comp_torque =  gravcomp.grav_gain * (sin((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias)*M_PI/180)); //h10
			gravcomp.f_grav_comp_torque = gravcomp.grav_alpha*gravcomp.f_grav_comp_torque + (1-gravcomp.grav_alpha)*gravcomp.grav_comp_torque;

			gravcomp.keep_move = 1;	}
		else
		{
			gravcomp.grav_comp_torque = 0;
			gravcomp.f_grav_comp_torque = gravcomp.grav_alpha*gravcomp.f_grav_comp_torque + (1-gravcomp.grav_alpha)*gravcomp.grav_comp_torque;
		}
	}
	else if(suit_mode == SUIT_AQUA )
	{
		gravcomp.grav_comp_torque =  gravcomp.grav_gain * (sin((AbsObj1.posDeg[0]/1.875 + imuVQFDataObj.angle_bodyframe.sagittal - filteredAngleDataObj_Sagittal.NeutralPostureBias)*M_PI/180)); //h10
		gravcomp.f_grav_comp_torque = gravcomp.grav_alpha*gravcomp.f_grav_comp_torque + (1-gravcomp.grav_alpha)*gravcomp.grav_comp_torque;
	}

	return 0;
}

static int Ent_Velocity_Compensation()
{

	velocitycomp.velocity_comp_torque = 0;
	velocitycomp.f_velocity_comp_torque = 0;

	velocitycomp.velocity_gain = 0;
	velocitycomp.velocity_threshold = 0.2;
	velocitycomp.velocity_satu = 1;
	velocitycomp.velocity_alpha = 0.99;

	return 0;
}

static int Run_Velocity_Compensation()
{
	if(mid_level_state.velocity_raw >= velocitycomp.velocity_threshold) {
		velocitycomp.velocity_comp_torque = velocitycomp.velocity_gain * mid_level_state.velocity_raw - velocitycomp.velocity_threshold * velocitycomp.velocity_gain;
	}
	else if(mid_level_state.velocity_raw <= - velocitycomp.velocity_threshold) {
		velocitycomp.velocity_comp_torque = velocitycomp.velocity_gain * mid_level_state.velocity_raw + velocitycomp.velocity_threshold * velocitycomp.velocity_gain;
		// For SUIT_SPACE mode, invert compensation sign when velocity is negative (use absolute velocity)
		if(suit_mode == SUIT_SPACE) {
			velocitycomp.velocity_comp_torque = -0.5*velocitycomp.velocity_comp_torque;
		}
	}
	else {
		velocitycomp.velocity_comp_torque = 0;
	}

	if(suit_mode == SUIT_IDLE || suit_mode == SUIT_SMART_ASSIST) //saturation difference as mode chage
	{
		if(velocitycomp.velocity_comp_torque > velocitycomp.velocity_satu)
		{
			velocitycomp.velocity_comp_torque = velocitycomp.velocity_satu;
		}
		else if(velocitycomp.velocity_comp_torque< - 4 * velocitycomp.velocity_satu)
		{
			velocitycomp.velocity_comp_torque = - 4 * velocitycomp.velocity_satu;
		}
	}
	else if(suit_mode == SUIT_AQUA || suit_mode == SUIT_SPACE)
	{
		if(velocitycomp.velocity_comp_torque > velocitycomp.velocity_satu)
		{
			velocitycomp.velocity_comp_torque = velocitycomp.velocity_satu;
		}
		else if(velocitycomp.velocity_comp_torque< -  velocitycomp.velocity_satu)
		{
			velocitycomp.velocity_comp_torque = - velocitycomp.velocity_satu;
		}
	}

	velocitycomp.f_velocity_comp_torque = velocitycomp.velocity_alpha * velocitycomp.f_velocity_comp_torque + (1 - velocitycomp.velocity_alpha) * velocitycomp.velocity_comp_torque;

	return 0;
}

static int Ent_Degree_Limit()
{

//	degreelimit.JointRoMLimitON = 0;
//	degreelimit.degree_limit_torque = 0;
	degreelimit.VirtualSpringCoeff = 1*180/M_PI;
	degreelimit.VirtualDampingCoeff = 0.001*180/M_PI;
	//	degreelimit.VirtualDampingCoeff = 0;
//	degreelimit.HipRoMUpperLimit = 180;
//	degreelimit.HipRoMLowerLimit = -180;
	return 0;
}

static int Run_Degree_Limit()
{
	float PosUpperError = 0;
	float PosLowerError = 0;

	degreelimit.VirtualSpringCoeff = 1*180/M_PI;
	degreelimit.VirtualDampingCoeff = 0.001*180/M_PI;

	PosUpperError = min(degreelimit.HipRoMUpperLimit - AbsObj1.posDeg[0]/1.875, 0);
	PosLowerError = max(degreelimit.HipRoMLowerLimit - AbsObj1.posDeg[0]/1.875, 0);

	if(degreelimit.HipRoMUpperLimit >= AbsObj1.posDeg[0]/1.875 && degreelimit.HipRoMLowerLimit <= AbsObj1.posDeg[0]/1.875)
	{
		degreelimit.JointRoMLimitON = 1;
	}

	if (PosUpperError < 0 )
	{
		degreelimit.degree_limit_torque = degreelimit.JointRoMLimitON * (degreelimit.VirtualSpringCoeff * PosUpperError/180*M_PI + degreelimit.VirtualDampingCoeff * mid_level_state.velocity_raw);
	}
	else if (PosLowerError > 0 )
	{
		degreelimit.degree_limit_torque = degreelimit.JointRoMLimitON * (degreelimit.VirtualSpringCoeff * PosLowerError/180*M_PI + degreelimit.VirtualDampingCoeff * mid_level_state.velocity_raw);
	}
	else
	{
		degreelimit.degree_limit_torque = 0;
	}

	return 0;
}


static int Ent_Velocity_Limit()
{
	velolimit.VirtualDampingCoeff = 0.1*180/M_PI;
	//	velolimit.VirtualDampingCoeff = 0;
	velolimit.alpha = 0.9;
	return 0;
}

float f_velo = 0;
float velo_sum = 0;
float velo_gain = 0.15;

static int Run_Velocity_Limit()
{
	float HVelError = 0;
	float velo_alpha = 0.0;

	velolimit.VirtualDampingCoeff = 0.04*180/M_PI;
	velolimit.alpha = 0.0;

	f_velo= velo_alpha * f_velo + (1-velo_alpha) * mid_level_state.velocity_raw;

	if(((velolimit.HipFlexionVelLimit/180*M_PI - f_velo  <0 / 180*M_PI) && f_velo >=0) ){

		HVelError = velolimit.HipFlexionVelLimit/180*M_PI - f_velo;
	}
	else if(((- velolimit.HipExtensionVelLimit/180*M_PI - f_velo > 0  / 180*M_PI)&& ( f_velo <=0))){
		HVelError = -velolimit.HipExtensionVelLimit/180*M_PI - f_velo;
	}

	velo_sum += HVelError;

	if(((velolimit.HipFlexionVelLimit/180*M_PI - f_velo  > 10 / 180*M_PI) && f_velo >=0) ){
				velo_sum = 0.99* velo_sum;
			}
	else if(((velolimit.HipFlexionVelLimit/180*M_PI - f_velo  > 0 / 180*M_PI) && f_velo >=0) ){
		velo_sum = 0.995* velo_sum;
	}
	else if(((- velolimit.HipExtensionVelLimit/180*M_PI - f_velo < -10  / 180*M_PI)&& ( f_velo <=0))){

		velo_sum = 0.99* velo_sum;
	}
	else if(((- velolimit.HipExtensionVelLimit/180*M_PI - f_velo < -0  / 180*M_PI)&& ( f_velo <=0))){

		velo_sum = 0.995* velo_sum;
	}

	velolimit.velo_limit_torque = velolimit.VirtualDampingCoeff * HVelError + velo_gain * velo_sum;

	if(velolimit.velo_limit_torque > 10){
		velolimit.velo_limit_torque = 10;
	}else if(velolimit.velo_limit_torque<-10){
		velolimit.velo_limit_torque = -10;
	}

	velolimit.f_velo_limit_torque = velolimit.alpha*velolimit.f_velo_limit_torque + (1 - velolimit.alpha)*velolimit.velo_limit_torque; //filter for prevent oscillation

	return 0;
}

/*ROUTINE_ID_CORRIDOR_IMPEDANCE_CONTROL */
static void error_filter2(ImpedanceCtrl *t_impedanceCtrl)
{
	// f2(e,t) = lambda * e + (1 - lambda)*sign(e)*max(|e| - epsilon, 0)
	float t_abs_e = 0.0;
	float t_sign_e = 0.0;
	float t_max = 0.0;
	float t_diff = 0.0;
	float y_ef = 0.0;

	/* Calculate 'sign(e) & |e|' */
	if (t_impedanceCtrl->e > 0) {t_abs_e = +t_impedanceCtrl->e; t_sign_e = +1; }
	else                        {t_abs_e = -t_impedanceCtrl->e; t_sign_e = -1; }

	/* Calculate 'max(|e| - epsilon, 0)' */
	t_diff = t_abs_e - t_impedanceCtrl->epsilon;
	if (t_diff > 0) {t_max = t_diff;}
	else            {t_max = 0;}

	y_ef = (t_impedanceCtrl->lambda * t_impedanceCtrl->e) + (1 - t_impedanceCtrl->lambda) * t_sign_e * t_max;

	t_impedanceCtrl->ef_diff = (y_ef - t_impedanceCtrl->ef) * MID_LEVEL_CONTROL_FREQUENCY;

	t_impedanceCtrl->ef   = y_ef;
}

static int Ent_Corridor_Impedance_Control()
{
	memset(&impedanceCtrl, 0, sizeof(ImpedanceCtrl));

	if (impedanceCtrl.Kp_max == 0)
		impedanceCtrl.Kp_max = 10;

	if (impedanceCtrl.Kd_max == 0)
		impedanceCtrl.Kd_max = 10;

	if (isnan(impedanceCtrl.Kp_max))
		impedanceCtrl.Kp_max = 10;

	if (isnan(impedanceCtrl.Kd_max))
		impedanceCtrl.Kd_max = 10;

	if (isnanf(impedanceCtrl.option))
		impedanceCtrl.option = 0;

	if ((impedanceCtrl.option != 0) && (impedanceCtrl.option != 1))
		impedanceCtrl.option = 0;

	if (impedanceCtrl.option == 1) {
		impedanceCtrl.epsilon = impedanceCtrl.opt1_i_buffer.epsilon_target * 0.001745329252;
		// impedanceCtrl.Kp = impedanceCtrl.opt1_i_buffer.Kp_target * impedanceCtrl.Kp_max * 0.392156862745098;
		// impedanceCtrl.Kd = impedanceCtrl.opt1_i_buffer.Kd_target * impedanceCtrl.Kd_max * 0.392156862745098;
		// impedanceCtrl.lambda = impedanceCtrl.opt1_i_buffer.lambda_target * 0.01;
		impedanceCtrl.Kp = impedanceCtrl.opt1_i_buffer.Kp_target * impedanceCtrl.Kp_max * 0.01; // 0.392156862745098 = 100/255
		impedanceCtrl.Kd = impedanceCtrl.opt1_i_buffer.Kd_target * impedanceCtrl.Kd_max * 0.01;
		impedanceCtrl.lambda = impedanceCtrl.opt1_i_buffer.lambda_target * 0.1;
	}

	return 0;
}

static int Run_Corridor_Impedance_Control()
{
	float t_epsilon = 0.0;
	float t_Kp = 0.0;
	float t_Kd = 0.0;
	float t_lambda = 0.0;

	if (impedanceCtrl.option == 0) { // Variable I-Vector
		if ((impedanceCtrl.N > 0) && (impedanceCtrl.ON == 0)) {
			// FIFO process
			t_epsilon = (float)impedanceCtrl.i_buffer[0].epsilon_target * 0.001745329252; // unit: rad   (0.001745329252 = 0.1 * pi/180)
			// t_Kp      = (float)impedanceCtrl.i_buffer[0].Kp_target * impedanceCtrl.Kp_max * 0.392156862745098; //0.57295779513;      // unit: A/rad (0.57295779513 = 0.01 * 180/pi)
			// t_Kd      = (float)impedanceCtrl.i_buffer[0].Kd_target * impedanceCtrl.Kd_max * 0.392156862745098; //     // unit: A/rad (0.57295779513 = 0.01 * 180/pi)
			// t_lambda  = (float)impedanceCtrl.i_buffer[0].lambda_target * 0.01;
			t_Kp      = (float)impedanceCtrl.i_buffer[0].Kp_target * impedanceCtrl.Kp_max * 0.01;
			t_Kd      = (float)impedanceCtrl.i_buffer[0].Kd_target * impedanceCtrl.Kd_max * 0.01;
			t_lambda  = (float)impedanceCtrl.i_buffer[0].lambda_target * 0.1;
			impedanceCtrl.L = (float)impedanceCtrl.i_buffer[0].duration;

			if (impedanceCtrl.L > 0) {
				float invT      = 1/impedanceCtrl.L;

				impedanceCtrl.gap_epsilon = (t_epsilon - impedanceCtrl.epsilon) * invT;
				impedanceCtrl.gap_Kp      = (t_Kp      - impedanceCtrl.Kp)      * invT;
				impedanceCtrl.gap_Kd      = (t_Kd      - impedanceCtrl.Kd)      * invT;
				impedanceCtrl.gap_lambda  = (t_lambda  - impedanceCtrl.lambda)  * invT;
			}

			impedanceCtrl.i  = 0; // initialize 1ms counter
			impedanceCtrl.ON = 1;
		}

		if (impedanceCtrl.ON == 1) {
			if (impedanceCtrl.L == 0) {
				impedanceCtrl.epsilon = t_epsilon;
				impedanceCtrl.Kp      = t_Kp;
				impedanceCtrl.Kd      = t_Kd;
				impedanceCtrl.lambda  = t_lambda;
			} else {
				impedanceCtrl.epsilon = impedanceCtrl.epsilon + impedanceCtrl.gap_epsilon;
				impedanceCtrl.Kp      = impedanceCtrl.Kp      + impedanceCtrl.gap_Kp;
				impedanceCtrl.Kd      = impedanceCtrl.Kd      + impedanceCtrl.gap_Kd;
				impedanceCtrl.lambda  = impedanceCtrl.lambda  + impedanceCtrl.gap_lambda;
				impedanceCtrl.i++;
			}

			if (impedanceCtrl.i >= impedanceCtrl.L)	{
				impedanceCtrl.ON = 0;
				impedanceCtrl.i = 0;

				// FIFO shifting
				for (int i = 0; i < impedanceCtrl.N; i++) {
					memcpy(&impedanceCtrl.i_buffer[i], &impedanceCtrl.i_buffer[i+1], sizeof(I_Vector));
				}
				impedanceCtrl.N--;
			}
		}
	}

	/* Impedance Controller */
	impedanceCtrl.e = posCtrl.ref - mid_level_state.position;

	error_filter2(&impedanceCtrl);

	float t_ef_diff = 0.0;

	if (((impedanceCtrl.ef > 0) & (impedanceCtrl.ef_diff > 0)) | ((impedanceCtrl.ef <= 0) & (impedanceCtrl.ef_diff <= 0))) {
		t_ef_diff = +impedanceCtrl.ef_diff;
	} else {
		t_ef_diff = -impedanceCtrl.ef_diff;
	}

	impedanceCtrl.control_input = impedanceCtrl.Kp * impedanceCtrl.ef + impedanceCtrl.Kd * t_ef_diff;
	
	// Saturate control input to ±10 using fminf and fmaxf
	impedanceCtrl.control_input = fminf(fmaxf(impedanceCtrl.control_input, -10.0f), 10.0f);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_CURRENT_SINE_REF*/
static int Ent_Generate_Current_Sine() {return 0;}
static int Run_Generate_Current_Sine()
{
	motor_in.mid_id_process_input = Generate_Sine(cur_periodic_sig.amp, 0, cur_periodic_sig.freq, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_VELOCITY_SINE_REF*/
static int Ent_Generate_Velocity_Sine() {return 0;}
static int Run_Generate_Velocity_Sine()
{
	velCtrl.ref = Generate_Sine(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_POSITION_SINE_REF*/
static int Ent_Generate_Position_Sine()
{
	mid_level_state.initial_pos = mid_level_state.position;
	return 0;
}
static int Run_Generate_Position_Sine()
{
	/* For PD controller */
	posCtrl.ref = Generate_Sine(pos_periodic_sig.amp, mid_level_state.initial_pos, pos_periodic_sig.freq, \
			mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	/* For Feedforward */
	if (posFF.diff != 0) {
		for (int i = 0; i < posFF.diff; ++i) {
			posFF.in[i] = Generate_Sine(pos_periodic_sig.amp, mid_level_state.initial_pos, pos_periodic_sig.freq, \
					mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, posFF.diff-i)	\
					- mid_level_state.initial_pos;
		}
		posFF.in[posFF.diff] = posCtrl.ref - mid_level_state.initial_pos;
	}

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_CURRENT_TANH_REF*/
static int Ent_Generate_Current_Tanh() {return 0;}
static int Run_Generate_Current_Tanh()
{
	//velCtrl.ref = Generate_Rectangle(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, 0.5, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1);
	motor_in.mid_id_process_input = Generate_Rectangle_tanh(cur_periodic_sig.amp, 0, cur_periodic_sig.freq, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_CURRENT_REC_REF*/
static int Ent_Generate_Current_Rec() {return 0;}
static int Run_Generate_Current_Rec()
{
	motor_in.mid_id_process_input = Generate_Rectangle(cur_periodic_sig.amp, 0, cur_periodic_sig.freq, 0.5, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_VELOCITY_REC_REF*/
static int Ent_Generate_Velocity_Rectangle() {return 0;}
static int Run_Generate_Velocity_Rectangle()
{
	//velCtrl.ref = Generate_Rectangle(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, 0.5, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1);
	velCtrl.ref = Generate_Rectangle_tanh(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_POSITION_REC_REF*/
static int Ent_Generate_Position_Rectangle()
{
	mid_level_state.initial_pos = mid_level_state.position;
	return 0;
}

int32_t t_lead = 0;

static int Run_Generate_Position_Rectangle()
{
	/* For PD controller */
	posCtrl.ref = Generate_Rectangle_tanh(pos_periodic_sig.amp, mid_level_state.initial_pos, pos_periodic_sig.freq, \
			mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	/* For Feedforward */
	if(posFF.diff != 0){
		for(int i = 0; i < posFF.diff; ++i){
			posFF.in[i] = Generate_Rectangle_tanh(pos_periodic_sig.amp, mid_level_state.initial_pos, pos_periodic_sig.freq,\
					mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, posFF.diff-i)	\
					- mid_level_state.initial_pos;
		}
		posFF.in[posFF.diff] = posCtrl.ref - mid_level_state.initial_pos;
	}
	return 0;
}

/*ROUTINE_ID_MIDLEVEL_GET_HALL_SENSOR_VALUE*/
static int Run_Send_IOIF_Hall_t_Values_to_GUI()
{
	Read_Hall_Sensors(&hallObj);

	static uint16_t t_identifier = 0;
	static uint32_t t_temp_arr[5] = {0};

	memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
	memcpy(&t_temp_arr[1], &hallObj.H1, 1);
	memcpy(&t_temp_arr[2], &hallObj.H2, 1);
	memcpy(&t_temp_arr[3], &hallObj.H3, 1);
	memcpy(&t_temp_arr[4], &hallObj.hall_logic, 1);

	t_identifier = GUI_SYNC|GET_HALLSENSOR;
	Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 20);

	return 0;
}


/*ROUTINE_ID_MIDLEVEL_GET_ENCODER_VALUE*/
static int Run_Send_IOIF_IncEnc_t_Values_to_GUI()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[3] = {0};

	memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
	memcpy(&t_temp_arr[1], &inc1KhzObj.userCnt, 4);
	memcpy(&t_temp_arr[2], &mid_level_state.position, 4);

	t_identifier = GUI_SYNC|GET_INCENCODER;
	Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 12);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_GET_ABSENCODER1_VALUE*/
static int Run_Send_IOIF_AbsEnc1_t_Values_to_GUI()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[3] = {0};

	memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
	memcpy(&t_temp_arr[1], &AbsObj1.posDegMultiTurn, 4);
	memcpy(&t_temp_arr[2], &AbsObj1.offset, 4);

	t_identifier = GUI_SYNC|GET_ABSENCODER1;
	Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 12);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_GET_ABSENCODER2_VALUE*/
static int Run_Send_IOIF_AbsEnc2_t_Values_to_GUI()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[3] = {0};

	memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
	memcpy(&t_temp_arr[1], &AbsObj2.posDegMultiTurn, 4);
	memcpy(&t_temp_arr[2], &AbsObj2.offset, 4);

	t_identifier = GUI_SYNC|GET_ABSENCODER2;
	Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 12);

	return 0;
}

static int Read_NodeID(void)
{
#if defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
	int temp1, temp2, temp3, temp4;
	temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_8);
	temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_9);
	temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_10);
	temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_11);
	return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
#endif /* L30_MD_REV06_ENABLED & L30_MD_REV07_ENABLED & L30_MD_REV08_ENABLED */

#if defined(SUIT_MD_ENABLED)
	int temp1, temp2, temp3, temp4;
	temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_2);
	temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_3);
	temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_4);
	temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_5);
	return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
#endif /* SUIT_MD_ENABLED */
}

#ifdef SUIT_MD_ENABLED
static void Set_Properties_SAM10(void)
{

	motor_properties.pole_pair 	= 14;
	inc1KhzObj.resolution 		= 8192;
	inc25KhzObj.resolution 		= 8192;
	motor_properties.gear_ratio = 18.75;
	motor_properties.Kt 		= 0.085;
	motor_properties.Ke 		= 0.085;
	motor_setting.peakCurr_limit = 14.0;
	motor_setting.contCurr_limit = mid_ctrl_saturation = 10.0;
	motor_setting.max_velocity_rpm = 1610.0;

	motor_setting.sensor_setting.e_angle_homing_sensor = e_EHoming_Sensor_Abs_Encoder2;
	motor_setting.sensor_setting.m_angle_homing_sensor = e_MHoming_Sensor_Abs_Encoder1;
	motor_setting.sensor_setting.pos_feedback_sensor = e_Pos_Sensor_Inc_Encoder;
	motor_setting.sensor_setting.temperature_sensor_usage = 0;
	motor_setting.sensor_setting.imu_6axis_usage = 0;
	motor_setting.sensor_setting.imu_3axis_usage = 0;

	motor_setting.sensor_setting.commutation_sensor = e_Commute_Sensor_Inc_Encoder;
	AbsObj1.location = IOIF_ABS_ENC_JOINT1;
	AbsObj2.location = IOIF_ABS_ENC_ACTUATOR_INPUT;

	uint8_t id = Read_NodeID();

	switch(id)
	{
		case 6: // RH
			AbsObj1.sign = -1;
			AbsObj2.sign = -1;
			motor_setting.commutation_set.ea_dir = -1;
			motor_setting.commutation_set.ma_dir = 1;
			motor_setting.commutation_set.cc_dir = -1;

			break;
		case 7: // LH
			AbsObj1.sign = 1;
			AbsObj2.sign = 1;
			motor_setting.commutation_set.ma_dir = -1;
			motor_setting.commutation_set.ea_dir = -1;
			motor_setting.commutation_set.cc_dir = 1;

			break;
		case 8:

			break;
		case 9:

			break;

		default:
			break;
	}

}

static void Init_Check_Abs1_rationality(IOIF_AbsEnc_t* abs1, Abs1Checker_t* checker)
{
	checker->offset = abs1->offset;
	checker->raw_offset = abs1->raw_offset;
	checker->errCnt = 0;
}

static volatile uint8_t testSet_flag = 0;
static volatile float testSet_value = 0;

static void Check_Abs1_rationality(IOIF_AbsEnc_t* abs1, MidLevelState* t_mid_level_state, Abs1Checker_t* checker)
{
	float temp0 = 0.0;

	if((checker->offset != abs1->offset) || (checker->raw_offset != abs1->raw_offset)) {
		checker->offset = abs1->offset;
		checker->raw_offset = abs1->raw_offset;
		return;
	}

	temp0 = fabs(abs1->posDeg[0] - abs1->posDeg[1]);

	if(temp0 > (360.0 - ABS1_MAX_DEGREE)) temp0 = 360 - temp0;

	if(temp0 > ABS1_MAX_DEGREE){
		temp0 = (t_mid_level_state->position - t_mid_level_state->position_f) * 180 / M_PI * 1.875;

		if(temp0 > ABS1_MAX_DEGREE) temp0 = ABS1_MAX_DEGREE;
		else if(temp0 < -ABS1_MAX_DEGREE) temp0 = -ABS1_MAX_DEGREE;

		abs1->posDeg[0] = abs1->posDeg[1] + temp0;

		checker->errCnt++;
	}

	checker->offset = abs1->offset;
	checker->raw_offset = abs1->raw_offset;
}

static int Ent_Check_Torque_Accuracy(void)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, LOAD_MT_DRV_EN, LM_ENABLE);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CCW);
	IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);

	motor_in.mid_id_process_input = 0.0;

	actuator_checker.step_num = 0;
	actuator_checker.tmr = 0;
	actuator_checker.Adc_offset = 0;
	actuator_checker.Adc_value = 0;
	actuator_checker.try_num = 0;
	actuator_checker.repeat_cnt = 0;

	for(int i = 0; i<MID_LV_ARRAY_MAX; i++){
		mid_level_process_array0[i] = 0;
		mid_level_process_array1[i] = 0;
		mid_level_process_array2[i] = 0;
	}

	return 0;
}

static int Run_Check_Torque_Accuracy(void)
{
	float tempX = 0.0, tempY = 0.0, tempXY = 0.0, tempXX = 0.0;
	float a = 0.0, b = 0.0, Den = 0.0;

	if(actuator_checker.step_num == 0){
		actuator_checker.tmr++;
		if((actuator_checker.tmr > 500) && (actuator_checker.tmr <= 1000)){
			motor_in.mid_id_process_input = -1.0;
		}
		else if((actuator_checker.tmr > 1000) && (actuator_checker.tmr <= 1500)){
			motor_in.mid_id_process_input = 0.0;
		}
		else if((actuator_checker.tmr > 1500) && (actuator_checker.tmr <= 2000)){
			motor_in.mid_id_process_input = 1.0;
		}
		else if((actuator_checker.tmr > 2000) && (actuator_checker.tmr <= 2500)){
			motor_in.mid_id_process_input = 0.0;
		}
		else if(actuator_checker.tmr > 2500){
			actuator_checker.tmr = 0;
			actuator_checker.step_num = 1;
		}
	}
	else if(actuator_checker.step_num == 1){
		actuator_checker.tmr++;

		if(actuator_checker.tmr > TORQUE_STABLE_TIME && actuator_checker.tmr <= TORQUE_STABLE_TIME + TORQUE_MEASUREMENT_NUMBER){
			actuator_checker.Adc_offset += rawAdc3[1];

			if(actuator_checker.tmr == TORQUE_STABLE_TIME + TORQUE_MEASUREMENT_NUMBER){
				actuator_checker.Adc_offset = actuator_checker.Adc_offset/TORQUE_MEASUREMENT_NUMBER;
				actuator_checker.Adc_value = 0;
				actuator_checker.tmr = 0;
				actuator_checker.step_num = 2;
				actuator_checker.try_num = 0;
			}
		}
	}
	else if(actuator_checker.step_num == 2){
		actuator_checker.tmr++;

		motor_in.mid_id_process_input = (TORQUE_CHECK_CRNT_MIN + (TORQUE_CHECK_CRNT_INTV * actuator_checker.try_num)) \
										* motor_setting.commutation_set.cc_dir;

		if(actuator_checker.tmr >= 50){
			if(actuator_checker.try_num >= TORQUE_CHECK_TRY_NUM_HALF){
				if(actuator_checker.tmr >= 1500){
					actuator_checker.try_num = 0;
					motor_in.mid_id_process_input = 0;
					actuator_checker.tmr = 0;
					actuator_checker.step_num = 3;
				}
			}
			else{
				actuator_checker.tmr = 0;
				actuator_checker.try_num++;
			}
		}
	}
	else if(actuator_checker.step_num == 3){
		actuator_checker.tmr++;
		motor_in.mid_id_process_input = 0.0;

		if(actuator_checker.tmr > 1000){
			actuator_checker.step_num = 4;
			actuator_checker.tmr = 0;
		}
	}
	else if(actuator_checker.step_num == 4){
		actuator_checker.tmr++;
		motor_in.mid_id_process_input = (TORQUE_CHECK_CRNT_MIN + (TORQUE_CHECK_CRNT_INTV * actuator_checker.try_num)) \
										* motor_setting.commutation_set.cc_dir;

		if(actuator_checker.tmr > TORQUE_STABLE_TIME){
			actuator_checker.Adc_value += rawAdc3[1];

			if(actuator_checker.tmr == TORQUE_STABLE_TIME + TORQUE_MEASUREMENT_NUMBER){
				actuator_checker.Adc_value = actuator_checker.Adc_value/TORQUE_MEASUREMENT_NUMBER;
				actuator_checker.torque = ((float)actuator_checker.Adc_value - actuator_checker.Adc_offset) / TORQUE_SENSOR_ADC2KGFCM_SCALE;
				actuator_checker.Adc_value = 0;
				actuator_checker.tmr = 0;
				mid_level_process_array0[actuator_checker.try_num] = motor_in.mid_id_process_input * motor_setting.commutation_set.cc_dir;
				mid_level_process_array1[actuator_checker.try_num] = \
						(mid_level_process_array1[actuator_checker.try_num] * actuator_checker.repeat_cnt / TORQUE_CHECK_REPEAT_NUM) \
						+ (actuator_checker.torque * (TORQUE_CHECK_REPEAT_NUM - actuator_checker.repeat_cnt) / TORQUE_CHECK_REPEAT_NUM);
				actuator_checker.try_num++;
			}
		}

		if(actuator_checker.try_num >= TORQUE_CHECK_TRY_NUM_HALF){
			actuator_checker.tmr = 0;
			actuator_checker.step_num = 5;
		}
	}
	else if(actuator_checker.step_num == 5){
		actuator_checker.tmr++;

		motor_in.mid_id_process_input = (-1 * (TORQUE_CHECK_CRNT_MIN + (TORQUE_CHECK_CRNT_INTV * (actuator_checker.try_num - TORQUE_CHECK_TRY_NUM_HALF)))) \
										* motor_setting.commutation_set.cc_dir;

		if(actuator_checker.tmr >= 50){
			if(actuator_checker.try_num >= TORQUE_CHECK_TRY_NUM_FULL){
				if(actuator_checker.tmr >= 1500){
					actuator_checker.try_num = TORQUE_CHECK_TRY_NUM_HALF;
					motor_in.mid_id_process_input = 0;
					actuator_checker.tmr = 0;
					actuator_checker.step_num = 6;
				}
			}
			else{
				actuator_checker.tmr = 0;
				actuator_checker.try_num++;
			}
		}
	}
	else if(actuator_checker.step_num == 6){
		actuator_checker.tmr++;
		motor_in.mid_id_process_input = 0.0;

		if(actuator_checker.tmr > 1000){
			actuator_checker.step_num = 7;
			actuator_checker.tmr = 0;
		}
	}
	else if(actuator_checker.step_num == 7){
		actuator_checker.tmr++;

		motor_in.mid_id_process_input = (-1 * (TORQUE_CHECK_CRNT_MIN + (TORQUE_CHECK_CRNT_INTV * (actuator_checker.try_num - TORQUE_CHECK_TRY_NUM_HALF)))) \
										* motor_setting.commutation_set.cc_dir;

		if(actuator_checker.tmr > TORQUE_STABLE_TIME){
			actuator_checker.Adc_value += rawAdc3[1];

			if(actuator_checker.tmr == TORQUE_STABLE_TIME + TORQUE_MEASUREMENT_NUMBER){
				actuator_checker.Adc_value = actuator_checker.Adc_value/TORQUE_MEASUREMENT_NUMBER;
				actuator_checker.torque = ((float)actuator_checker.Adc_value - actuator_checker.Adc_offset) / TORQUE_SENSOR_ADC2KGFCM_SCALE;
				actuator_checker.Adc_value = 0;
				actuator_checker.tmr = 0;
				mid_level_process_array0[actuator_checker.try_num] = motor_in.mid_id_process_input * motor_setting.commutation_set.cc_dir;
				mid_level_process_array1[actuator_checker.try_num] = \
										(mid_level_process_array1[actuator_checker.try_num] * actuator_checker.repeat_cnt / TORQUE_CHECK_REPEAT_NUM) \
										+ (actuator_checker.torque * (TORQUE_CHECK_REPEAT_NUM - actuator_checker.repeat_cnt) / TORQUE_CHECK_REPEAT_NUM);
				actuator_checker.try_num++;
			}
		}

		if(actuator_checker.try_num >= TORQUE_CHECK_TRY_NUM_FULL){
			actuator_checker.repeat_cnt++;
			motor_in.mid_id_process_input = 0.0;

			if(actuator_checker.repeat_cnt == TORQUE_CHECK_REPEAT_NUM){
				actuator_checker.step_num = 8;
			}
			else{
				actuator_checker.step_num = 2;
				actuator_checker.try_num = 0;
			}
		}
	}
	else if(actuator_checker.step_num == 8){
		/************************* POS *************************/
		for(int i = 0; i<TORQUE_CHECK_TRY_NUM_HALF; i++){
			tempX += mid_level_process_array0[i]; // x
			tempY += mid_level_process_array1[i]; // y
			tempXY += mid_level_process_array0[i] * mid_level_process_array1[i]; // xy
			tempXX += mid_level_process_array0[i] * mid_level_process_array0[i]; // x^2
		}

		Den = (TORQUE_CHECK_TRY_NUM_HALF * tempXX) - (tempX * tempX);

		if(fabs(Den) < 1e-6) {
			a = 0;
			b = 0;
			tempXX = 0.0;
			tempXY = 0.0;
		}
		else{
		    a = ((TORQUE_CHECK_TRY_NUM_HALF * tempXY) - (tempX * tempY))/Den;
		    b = (tempY - (a * tempX)) / TORQUE_CHECK_TRY_NUM_HALF;
		    actuator_checker.a_p = a;
		    actuator_checker.b_p = b;
			tempXX = 0.0;
			tempXY = 0.0;
		}

		for(int i = 0; i<TORQUE_CHECK_TRY_NUM_HALF; i++){
			mid_level_process_array2[i] = a * mid_level_process_array0[i] + b;
			tempXY += (mid_level_process_array1[i] - (tempY/TORQUE_CHECK_TRY_NUM_HALF)) * (mid_level_process_array1[i] - (tempY/TORQUE_CHECK_TRY_NUM_HALF));
			tempXX += (mid_level_process_array2[i] - mid_level_process_array1[i]) * (mid_level_process_array2[i] - mid_level_process_array1[i]);
		}

		actuator_checker.coeff_R_pos = 1 - tempXX/tempXY;
		tempX = tempY = tempXY = tempXX = 0.0;

		/************************* NEG *************************/
		for(int i = TORQUE_CHECK_TRY_NUM_HALF; i<TORQUE_CHECK_TRY_NUM_FULL; i++){
			tempX += mid_level_process_array0[i]; // x
			tempY += mid_level_process_array1[i]; // y
			tempXY += mid_level_process_array0[i] * mid_level_process_array1[i]; // xy
			tempXX += mid_level_process_array0[i] * mid_level_process_array0[i]; // x^2
		}

		Den = (TORQUE_CHECK_TRY_NUM_HALF * tempXX) - (tempX * tempX);

		if(fabs(Den) < 1e-6) {
			a = 0;
			b = 0;
			tempXX = 0.0;
			tempXY = 0.0;
		}
		else{
		    a = ((TORQUE_CHECK_TRY_NUM_HALF * tempXY) - (tempX * tempY))/Den;
		    b = (tempY - (a * tempX)) / TORQUE_CHECK_TRY_NUM_HALF;
		    actuator_checker.a_n = a;
		    actuator_checker.b_n = b;
			tempXX = 0.0;
			tempXY = 0.0;
		}

		for(int i = TORQUE_CHECK_TRY_NUM_HALF; i<TORQUE_CHECK_TRY_NUM_FULL; i++){
			mid_level_process_array2[i] = a * mid_level_process_array0[i] + b;
			tempXY += (mid_level_process_array1[i] - (tempY/TORQUE_CHECK_TRY_NUM_HALF)) * (mid_level_process_array1[i] - (tempY/TORQUE_CHECK_TRY_NUM_HALF));
			tempXX += (mid_level_process_array2[i] - mid_level_process_array1[i]) * (mid_level_process_array2[i] - mid_level_process_array1[i]);
		}

		actuator_checker.coeff_R_neg = 1 - tempXX/tempXY;

		tempXX = (actuator_checker.a_p + actuator_checker.a_n) / 2.0;
		tempXY = fabs(tempXX - actuator_checker.a_p);

		actuator_checker.min = (tempXY / tempXX) * 100;
		actuator_checker.max = actuator_checker.a_p;

		motor_in.mid_id_process_input = 0.0;
		StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		MS_enum = SEND_TORQUE_ACCURACY_RESULT;
	}

	return 0;
}

static int Ext_Check_Torque_Accuracy(void)
{
	motor_in.mid_id_process_input = 0.0;
	actuator_checker.tmr = 0;

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, LOAD_MT_DRV_EN, LM_DISABLE);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
	IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);

	return 0;
}

void Send_Torque_Accuracy_Result(void)
{
	uint16_t t_identifier = 0;
	float t_buf[4] = {0};

	for (int i = 0; i < TORQUE_CHECK_TRY_NUM_FULL; i++) {
		memcpy(&t_buf[0], &mid_level_process_array0[i], 4);
		memcpy(&t_buf[1], &mid_level_process_array1[i], 4);
		memcpy(&t_buf[2], &mid_level_process_array2[i], 4);

		for (int j = 0; j < 1000000; j++) {}

		t_identifier = GUI_SYNC|TORQUE_ACCURACY_DATA;
		Send_MSG(t_identifier, (uint8_t*)t_buf, 12);
	}

	for (int k = 0; k < 1000000; k++) {}

	memcpy(&t_buf[0], &actuator_checker.coeff_R_pos, 4);
	memcpy(&t_buf[1], &actuator_checker.coeff_R_neg, 4);
	memcpy(&t_buf[2], &actuator_checker.min, 4);
	memcpy(&t_buf[3], &actuator_checker.max, 4);


	t_identifier = GUI_SYNC|TORQUE_ACCURACY_DONE;
	Send_MSG(t_identifier, (uint8_t*)t_buf, 16);

	MS_enum = IDLE;
}

static int Ent_Check_Torque_Uniformity(void)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, LOAD_MT_DRV_EN, LM_ENABLE);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CCW);
	IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);

	motor_in.mid_id_process_input = 0.0;
	actuator_checker.tmr = 0;
	actuator_checker.pp_cnt = 0;
	actuator_checker.step_num = 0;

	actuator_checker.avg1 = 0.0;
	actuator_checker.avg2 = 0.0;

	actuator_checker.torque_StDev1 = 0.0;
	actuator_checker.torque_StDev2 = 0.0;

	actuator_checker.torque_StDev_rate1 = 0.0;
	actuator_checker.torque_StDev_rate2 = 0.0;

	actuator_checker.Adc_value = 0;
	actuator_checker.Adc_offset = 0;
	actuator_checker.tmr2 = 0;

	actuator_checker.repeat_cnt = 0;

	for(int i = 0; i<MID_LV_ARRAY_MAX; i++){
		mid_level_process_array0[i] = 0;
		mid_level_process_array1[i] = 0;
		mid_level_process_array2[i] = 0;
		mid_level_process_array3[i] = 0;
	}

	return 0;
}

static int Run_Check_Torque_Uniformity(void)
{
//	uint16_t temp0 = LOAD_MOTOR_RES2ELEC_2PI; // 91
	float temp0 = 0.0, temp1 = 0.0;

	if(actuator_checker.step_num == 0){
		actuator_checker.tmr++;
		if((actuator_checker.tmr > 500) && (actuator_checker.tmr <= 1000)){
			motor_in.mid_id_process_input = -1.0;
		}
		else if((actuator_checker.tmr > 1000) && (actuator_checker.tmr <= 1500)){
			motor_in.mid_id_process_input = 0.0;
		}
		else if((actuator_checker.tmr > 1500) && (actuator_checker.tmr <= 2000)){
			motor_in.mid_id_process_input = 1.0;
		}
		else if((actuator_checker.tmr > 2000) && (actuator_checker.tmr <= 2500)){
			motor_in.mid_id_process_input = 0.0;
		}
		else if(actuator_checker.tmr > 2500){
			actuator_checker.tmr = 0;
			actuator_checker.step_num = 1;
		}
	}
	else if(actuator_checker.step_num == 1){
		actuator_checker.tmr++;

		if(actuator_checker.tmr <= 1000){
			actuator_checker.Adc_offset += rawAdc3[1];

			if(actuator_checker.tmr == 1000){
				actuator_checker.Adc_offset = actuator_checker.Adc_offset/1000;
				actuator_checker.Adc_value = 0;
				actuator_checker.tmr = 0;
				actuator_checker.step_num = 2;
			}
		}
	}
	else if(actuator_checker.step_num == 2){
		if(actuator_checker.repeat_cnt == 0){
			motor_in.mid_id_process_input = 0.0;
			actuator_checker.step_num = 3;
			actuator_checker.tmr = 0;
		}
		else if(actuator_checker.repeat_cnt == 1){
			if(fabs(motor_in.mid_id_process_input) < 1.25){
				motor_in.mid_id_process_input += (0.25 * motor_setting.commutation_set.cc_dir);
			}
			else{
				motor_in.mid_id_process_input = 1.25 * motor_setting.commutation_set.cc_dir;
				actuator_checker.tmr++;
				if(actuator_checker.tmr >= 2500){
					actuator_checker.step_num = 3;
					actuator_checker.tmr = 0;
				}
			}
		}
	}
	else if(actuator_checker.step_num == 3){
		actuator_checker.tmr++;

		if(actuator_checker.tmr <= 1240){
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr <= 5120 + 1240){
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);

			actuator_checker.Adc_value += (rawAdc3[1] - actuator_checker.Adc_offset);
			actuator_checker.tmr2++;
			if(actuator_checker.tmr2 == UNIFORMITY_SAMP_NUM){
				mid_level_process_array0[actuator_checker.pp_cnt] = (360.0 / (2560.0 / 40.0))  * (actuator_checker.pp_cnt + 1);

				if(actuator_checker.repeat_cnt == 0){
					mid_level_process_array2[actuator_checker.pp_cnt] = actuator_checker.Adc_value/UNIFORMITY_SAMP_NUM/TORQUE_SENSOR_ADC2KGFCM_SCALE;
					actuator_checker.avg2 += mid_level_process_array2[actuator_checker.pp_cnt];
				}
				else if(actuator_checker.repeat_cnt == 1){
					mid_level_process_array3[actuator_checker.pp_cnt] = actuator_checker.Adc_value/UNIFORMITY_SAMP_NUM/TORQUE_SENSOR_ADC2KGFCM_SCALE;
					actuator_checker.avg1 += mid_level_process_array3[actuator_checker.pp_cnt];
				}
				actuator_checker.pp_cnt++;
				actuator_checker.Adc_value = 0;
				actuator_checker.tmr2 = 0;
			}
		}
		else if(actuator_checker.tmr <= 5120 + 1240 + 80){
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
			motor_in.mid_id_process_input = 0.0;
		}
		else if(actuator_checker.tmr <= 5120 + 1240 + 80 + 500){
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
		}
		else if(actuator_checker.tmr <= 5120 + 1240 + 80 + 500 + 5120 + 80 + 1240){
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
			actuator_checker.Adc_value = 0;
			actuator_checker.tmr2 = 0;
			actuator_checker.pp_cnt = 64;
		}
		else if(actuator_checker.tmr <= 5120 + 1240 + 80 + 500 + 5120 + 80 + 1240 + 500){
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CCW);
		}
		else if(actuator_checker.tmr > 5120 + 1240 + 80 + 500 + 5120 + 80 + 1240 + 500){
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			motor_in.mid_id_process_input = 0.0;
			actuator_checker.tmr = 0;
			actuator_checker.Adc_value = 0;
			actuator_checker.tmr2 = 0;
			actuator_checker.repeat_cnt++;
			actuator_checker.pp_cnt = 0;

			if(actuator_checker.repeat_cnt == 1){
				actuator_checker.step_num = 2;
			}
			else if(actuator_checker.repeat_cnt == 2){
				actuator_checker.step_num = 4;
			}
		}
	}
	else if(actuator_checker.step_num == 4){
		actuator_checker.avg1 = actuator_checker.avg1 / 64.0;
		actuator_checker.avg2 = actuator_checker.avg2 / 64.0;
		temp0 = actuator_checker.avg1 - actuator_checker.avg2;
		actuator_checker.avg1 = temp0;

		for(int i = 0; i<64; i++){
			mid_level_process_array1[i] = mid_level_process_array3[i] - mid_level_process_array2[i];
			temp1 += (temp0 - mid_level_process_array1[i]) * (temp0 - mid_level_process_array1[i]);
		}

		actuator_checker.torque_StDev1 = sqrt(temp1 / 64.0);
		actuator_checker.torque_StDev_rate1 = (actuator_checker.torque_StDev1 * 100) / actuator_checker.avg1;
		StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		MS_enum = SEND_TORQUE_UNIFORMITY_RESULT;
	}

	return 0;
}

static int Ext_Check_Torque_Uniformity(void)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, LOAD_MT_DRV_EN, LM_DISABLE);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
	IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);

	actuator_checker.step_num = 0;
	actuator_checker.pp_cnt = 0;
	actuator_checker.Adc_value = 0;
	actuator_checker.Adc_offset = 0;

	return 0;
}

void Send_Torque_Uniformity_Result(void)
{
	uint16_t t_identifier = 0;
	float t_buf[3] = {0};

	for (int i = 0; i < 64; i++) {
		memcpy(&t_buf[0], &mid_level_process_array0[i], 4);
		memcpy(&t_buf[1], &mid_level_process_array1[i], 4);
		memcpy(&t_buf[2], &mid_level_process_array2[i], 4);

		for (int j = 0; j < 1000000; j++) {}

		t_identifier = GUI_SYNC|TORQUE_UNIFORMITY_DATA;
		Send_MSG(t_identifier, (uint8_t*)t_buf, 12);
	}

	for (int k = 0; k < 1000000; k++) {}

	memcpy(&t_buf[0], &actuator_checker.avg1, 4);
	memcpy(&t_buf[1], &actuator_checker.torque_StDev1, 4);
	memcpy(&t_buf[2], &actuator_checker.torque_StDev_rate1, 4);

	t_identifier = GUI_SYNC|TORQUE_UNIFORMITY_DONE;
	Send_MSG(t_identifier, (uint8_t*)t_buf, 12);

	MS_enum = IDLE;
}

static int Ent_Check_backdriverbility(void)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, LOAD_MT_DRV_EN, LM_ENABLE);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
	IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);

	actuator_checker.step_num = 0;
	actuator_checker.pp_cnt = 0;
	actuator_checker.torque_StDev1 = 0.0;
	actuator_checker.torque_StDev_rate1 = 0.0;
	actuator_checker.torque_StDev2 = 0.0;
	actuator_checker.torque_StDev_rate2 = 0.0;
	actuator_checker.avg1 = 0.0;
	actuator_checker.avg2 = 0.0;
	actuator_checker.min = 1000.0;
	actuator_checker.max = -1000.0;

	for(int i = 0; i<MID_LV_ARRAY_MAX; i++){
		mid_level_process_array0[i] = 0;
		mid_level_process_array1[i] = 0;
		mid_level_process_array2[i] = 0;
	}

	return 0;
}

static int Run_Check_backdriverbility(void)
{
	float temp0 = 0.0, temp1 = 0.0;

	if(actuator_checker.step_num == 0){ // Position Align
		actuator_checker.tmr++;
		if(actuator_checker.tmr <= 30){ // 0.45도 CW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 30 && actuator_checker.tmr <= 1030){ // 정지
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CCW);
		}
		else if(actuator_checker.tmr > 1030 && actuator_checker.tmr <= 1035){ // 0.075도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 1035 && actuator_checker.tmr <= 1500){ // 정지
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
		}
		else if(actuator_checker.tmr > 1500 && actuator_checker.tmr <= 1505){ // 0.075도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 1505 && actuator_checker.tmr <= 2000){ // 정지
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
		}
		else if(actuator_checker.tmr > 2000 && actuator_checker.tmr <= 2005){ // 0.075도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 2005 && actuator_checker.tmr <= 2500){ // 정지
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
		}
		else if(actuator_checker.tmr > 2500 && actuator_checker.tmr <= 2505){ // 0.75도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 2505 && actuator_checker.tmr <= 3000){ // 정지
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
		}
		else if(actuator_checker.tmr > 3000 && actuator_checker.tmr <= 3005){ // 0.75도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 3005 && actuator_checker.tmr <= 3500){ // 정지
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
		}
		else if(actuator_checker.tmr > 3500 && actuator_checker.tmr <= 3505){ // 0.75도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 3505 && actuator_checker.tmr <= 4000){ // 정지 & 방향 전환
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
		}
		else if(actuator_checker.tmr > 4000 && actuator_checker.tmr <= 4005){ // 0.75도 CW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 4005 && actuator_checker.tmr <= 4500){ // 정지 & 방향 전환
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CCW);
		}
		else if(actuator_checker.tmr > 4500 && actuator_checker.tmr <= 4505){ // 0.75도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 4505 && actuator_checker.tmr <= 5000){ // 정지
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CCW);
		}
		else if(actuator_checker.tmr > 5000){
			actuator_checker.tmr = 0;
			actuator_checker.step_num = 1;
		}
	}
	else if(actuator_checker.step_num == 1){
		actuator_checker.tmr++;

		if(actuator_checker.tmr <= 1000){
			actuator_checker.Adc_offset += rawAdc3[1];

			if(actuator_checker.tmr == 1000){
				actuator_checker.Adc_offset = actuator_checker.Adc_offset/1000;
				actuator_checker.Adc_value = 0;
				actuator_checker.tmr = 0;
				actuator_checker.step_num = 2;
			}
		}
	}
	else if(actuator_checker.step_num == 2){
		actuator_checker.tmr++;
		if(actuator_checker.tmr <= 12000){ // 90도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr <= 12500){ // 정지 & 회전 방향 변경
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
		}
		else if(actuator_checker.tmr <= 24500){ // 6도 CW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr <= 25000){ // 6도 CW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CCW);
		}
		else if(actuator_checker.tmr > 25000){ // 6도 CW 회전
			actuator_checker.tmr = 0;
			actuator_checker.step_num = 3;
		}
	}
	else if(actuator_checker.step_num == 3){
		actuator_checker.tmr++;
		if(actuator_checker.tmr <= BACKDRIVERBILITY_WARM_UP_CNT){ // 6도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr <= BACKDRIVERBILITY_RECORD_START_CNT){ // 180도 CCW 회전
			actuator_checker.tmr2++;
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);

			actuator_checker.Adc_value += (rawAdc3[1] - actuator_checker.Adc_offset);
			if(actuator_checker.tmr2 == BACKDRIVERBILITY_SAMP_NUM){
				actuator_checker.tmr2 = 0;
				mid_level_process_array1[actuator_checker.pp_cnt] = actuator_checker.Adc_value/BACKDRIVERBILITY_SAMP_NUM/TORQUE_SENSOR_ADC2KGFCM_SCALE;
				mid_level_process_array0[actuator_checker.pp_cnt] = (LOAD_MOTOR_RESOLUTION * BACKDRIVERBILITY_SAMP_NUM) * (actuator_checker.pp_cnt + 1);
				actuator_checker.avg1 += mid_level_process_array1[actuator_checker.pp_cnt];
				actuator_checker.pp_cnt++;
				actuator_checker.Adc_value = 0;
			}
		}
		else if(actuator_checker.tmr <= 12800){ // 6도 CCW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr <= 12900){ // 정지 & 회전 방향 변경
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
		}
		else if(actuator_checker.tmr <= 13300){ // 6도 CW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
			actuator_checker.tmr2 = 0;
			actuator_checker.pp_cnt = BACKDRIVERBILITY_RECORD_NUM;
		}
		else if(actuator_checker.tmr <= 25300){ // 180도 CW 회전
			actuator_checker.tmr2++;
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);

			actuator_checker.Adc_value += (rawAdc3[1] - actuator_checker.Adc_offset);
			if(actuator_checker.tmr2 == BACKDRIVERBILITY_SAMP_NUM){
				actuator_checker.tmr2 = 0;
				actuator_checker.pp_cnt--;
				mid_level_process_array2[actuator_checker.pp_cnt] = fabs(actuator_checker.Adc_value/BACKDRIVERBILITY_SAMP_NUM/TORQUE_SENSOR_ADC2KGFCM_SCALE);
				actuator_checker.avg2 += mid_level_process_array2[actuator_checker.pp_cnt];
				actuator_checker.Adc_value = 0;
			}
		}
		else if(actuator_checker.tmr <= 25700){ // 6도 CW 회전
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		}
		else if(actuator_checker.tmr > 25700){
			actuator_checker.avg1 = actuator_checker.avg1 / BACKDRIVERBILITY_RECORD_NUM;
			actuator_checker.avg2 = actuator_checker.avg2 / BACKDRIVERBILITY_RECORD_NUM;

			for(int i = 0; i<240; i++){
				temp0 += (actuator_checker.avg1 - mid_level_process_array1[i]) * (actuator_checker.avg1 - mid_level_process_array1[i]);
				temp1 += (actuator_checker.avg2 - mid_level_process_array2[i]) * (actuator_checker.avg2 - mid_level_process_array2[i]);

				if(mid_level_process_array1[i] > actuator_checker.max){
					actuator_checker.max = mid_level_process_array1[i];
				}

				if(mid_level_process_array1[i] < actuator_checker.min){
					actuator_checker.min = mid_level_process_array1[i];
				}
			}

			actuator_checker.torque_StDev1 = sqrt(temp0 / BACKDRIVERBILITY_RECORD_NUM);
			actuator_checker.torque_StDev2 = sqrt(temp1 / BACKDRIVERBILITY_RECORD_NUM);

			actuator_checker.torque_StDev_rate1 = (actuator_checker.torque_StDev1 * 100) / actuator_checker.avg1;
			actuator_checker.torque_StDev_rate2 = (actuator_checker.torque_StDev2 * 100) / actuator_checker.avg2;

			actuator_checker.step_num = 4;
			actuator_checker.pp_cnt = 0;
			actuator_checker.tmr = 0;
			actuator_checker.tmr2 = 0;
			IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
		}
	}
	else if(actuator_checker.step_num == 4){
		StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		MS_enum = SEND_ACTUATOR_BACKDRIVERBILITY_RESULT;
	}

	return 0;
}


static int Ext_Check_backdriverbility(void)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, LOAD_MT_DRV_EN, LM_DISABLE);

	actuator_checker.step_num = 0;
	actuator_checker.tmr = 0;
	actuator_checker.tmr2 = 0;

	return 0;
}
static volatile uint8_t uctest0509= 0;

void Send_Actuator_backdriverbility_Result(void)
{
	uint16_t t_identifier = 0;
	float t_buf[4] = {0};

	uctest0509++;
	for (int i = 0; i < BACKDRIVERBILITY_RECORD_NUM; i++) {
		memcpy(&t_buf[0], &mid_level_process_array0[i], 4);
		memcpy(&t_buf[1], &mid_level_process_array1[i], 4);

		for (int j = 0; j < 1000000; j++) {}

		t_identifier = GUI_SYNC|ACTUATOR_BACKDRIVABILITY_DATA;
		Send_MSG(t_identifier, (uint8_t*)t_buf, 8);
	}

	for (int k = 0; k < 1000000; k++) {}

	memcpy(&t_buf[0], &actuator_checker.avg1, 4);
	memcpy(&t_buf[1], &actuator_checker.torque_StDev1, 4);
	memcpy(&t_buf[2], &actuator_checker.max, 4);
	memcpy(&t_buf[3], &actuator_checker.min, 4);

	t_identifier = GUI_SYNC|ACTUATOR_BACKDRIVABILITY_DONE;
	Send_MSG(t_identifier, (uint8_t*)t_buf, 16);

	MS_enum = IDLE;
}

static int Ent_Check_EncoderLinearity(void)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, LOAD_MT_DRV_EN, LM_ENABLE);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CCW);
	IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);

	motor_in.mid_id_process_input = 0.0;
	actuator_checker.lowlevel_skipCnt = 0;
	actuator_checker.tmr = 0;
	actuator_checker.tmr2 = 0;
	actuator_checker.encAngle = 0;
	actuator_checker.encAngle_old = 0;
	actuator_checker.encAngle_sum = 0;
	actuator_checker.realAngle_sum = 0;
	actuator_checker.avg1 = 0;
	actuator_checker.torque_StDev1 = 0;

	for(int i = 0; i < MID_LV_ARRAY_FOR_ENCODER ; i++){
		mid_level_process_array3[i] = 0;
		mid_level_process_array4[i] = 0;
	}

	return 0;
}

static int Run_Check_EncoderLinearity(void)
{
	int16_t temp0 = 0;
	float temp1 = 0.0, temp2 = 0.0;

	actuator_checker.tmr++;

	if(actuator_checker.tmr <= 200){ // 3도 CCW 회전
		IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
		actuator_checker.encAngle = AbsObj2.rawBitsTo13Bit;
		actuator_checker.encAngle_old = actuator_checker.encAngle;
	}
	else if(actuator_checker.tmr <= 200 + 1280){ // 720도 CCW 회전
		actuator_checker.tmr2++;
		actuator_checker.encAngle = AbsObj2.rawBitsTo13Bit;
		if(actuator_checker.tmr2 == 2){ // 0.45도
			temp0 = actuator_checker.encAngle - actuator_checker.encAngle_old;
			if(temp0 > 4095){
				temp0 -= 8192;
			}
			else if(temp0 < -4095){
				temp0 += 8192;
			}
			actuator_checker.encAngle_sum += temp0;
			actuator_checker.realAngle_sum += 0.5625; // = 360 / (1280 / 2)

			mid_level_process_array3[actuator_checker.lowlevel_skipCnt] = actuator_checker.realAngle_sum;
			mid_level_process_array4[actuator_checker.lowlevel_skipCnt] = actuator_checker.realAngle_sum - (((float)actuator_checker.encAngle_sum) / 8192.0 * 360.0);

			actuator_checker.tmr2 = 0;
			actuator_checker.lowlevel_skipCnt++;
			actuator_checker.encAngle_old = actuator_checker.encAngle;
		}
		IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
	}
	else if(actuator_checker.tmr <= 200 + 1280 + 1280 + 200){
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
		IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, LOAD_MOTOR_DUTY);
	}
	else if(actuator_checker.tmr > 200 + 1280 + 1280 + 200){
		actuator_checker.avg1 = actuator_checker.avg1 / 640;

		for (int i = 0; i < MID_LV_ARRAY_FOR_ENCODER; i++) {
			temp1 = actuator_checker.avg1 - mid_level_process_array4[i];
			temp2 += temp1 * temp1;
		}
		actuator_checker.torque_StDev1 = sqrt(temp2 / MID_LV_ARRAY_FOR_ENCODER);
		IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
		StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		MS_enum = SEND_ENCODER_LINEARITY_RESULT;
	}

	return 0;
}

static int Ext_Check_EncoderLinearity(void)
{
	actuator_checker.tmr = 0;
	actuator_checker.tmr2 = 0;

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, LOAD_MT_DRV_EN, LM_DISABLE);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_F, LOAD_MT_DRV_DIR, LM_CW);
	IOIF_SetTimCompVal(IOIF_TIM17, IOIF_TIM_CHANNEL_1, 0);
	return 0;
}

void Send_Encoder_Linearity_Result(void)
{
	uint16_t t_identifier = 0;
	float t_buf[4] = {0};

	uctest0509++;
	for (int i = 0; i < MID_LV_ARRAY_FOR_ENCODER; i++) {

		memcpy(&t_buf[0], &mid_level_process_array3[i], 4);
		memcpy(&t_buf[0], &mid_level_process_array4[i], 4);

		for (int j = 0; j < 1000000; j++) {}

		t_identifier = GUI_SYNC|GET_ENCODER_ANGLE_DEVIATION_DATA;
		Send_MSG(t_identifier, (uint8_t*)t_buf, 8);
	}

	for (int k = 0; k < 1000000; k++) {}

	memcpy(&t_buf[0], &actuator_checker.torque_StDev1, 4);
//	memcpy(&t_buf[0], &actuator_checker.avg1, 4);

	t_identifier = GUI_SYNC|GET_ENCODER_ANGLE_LINEARITY_DONE;
	Send_MSG(t_identifier, (uint8_t*)t_buf, 4);

	MS_enum = IDLE;
}

#endif
