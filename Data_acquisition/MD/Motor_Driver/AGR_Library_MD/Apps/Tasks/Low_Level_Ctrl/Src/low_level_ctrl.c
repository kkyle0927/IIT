/**
 * @file low_level_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

// TODO : refactoring!!
#include "low_level_ctrl.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

TaskObj_t 				low_level_ctrl_task;
MotorProperties 		motor_properties;
MotorSetting			motor_setting;
MotorIn 				motor_in;
MotorOut				motor_out;
KFObject        		kf_current_object;
IOIF_IncEnc_t			inc25KhzObj;
IOIF_HallSensor_t       hallObj;
Advanced_Friction_ID 	advanced_friction_id;
OtpDerate_t 			OtpDerateCtrl;
SetStall_t				StallDetect;

ScaledData 				FloatTo2ByteData_low;

IF_SpdCtrl_t			AgingCtrl;
MESSetting_t			MES	__attribute__((section(".mesdata")));

MaxAngleCtrl_t 		MaxAngleLimit;
encCheck_t			encCheck;

uint8_t joint_limit_sw = 0;

static MotorElecSystemID 	elec_system_id;
// static DOPI_CurrVerif_t 	current_ctrl_verify;

static PIDObject 			pid_d_axis_ctrl;       // d-axis current controller for FOC
static PIDObject 			pid_q_axis_ctrl;       // q-axis current controller for FOC
static PIDObject			pid_trape_ctrl;

static ClarkeIn 			clarke_in;
static ClarkeOut 			clarke_out;
// static InvClarkeIn		inv_clarke_in;
// static InvClarkeOut 		inv_clarke_out;
static ParkIn 	    		park_in;
static ParkOut 	    		park_out;
static InvParkIn 	    	inv_park_in;
static InvParkOut			inv_park_out;

static Phasor 				svm_phasor;

static TrapezoidalControl trape_ctrl;

static DOPI_SharedBuff_t shared_array_buffer;


/**
 *------------------------------------------------------------
 *                            VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

static uint32_t low_level_ctrl_loop_cnt;

static uint16_t RT_Broken;
static float lowTimeElap;

static float batch_result_1, batch_result_2;
static float rms_err, mean_vel;
static int N;

int8_t current_sign;
static uint16_t* rawCurr = {0};	// 0,1,2 UVW Phase Current & 3,4,5 UVW BEMF Voltage
static int32_t commutation_raw[50];

uint8_t torque_measure_enable = false;

float motor_actual_current;
uint8_t safety_timer_flag = 0;
float capture_safety_current;
float safety_timer = 1;
static thetaComp_t thetaComp;

static volatile uint32_t crnt_ramp_tmr = 0;
static volatile uint8_t torque_measurement_mode_curr_on = 0;
static volatile uint8_t torque_measurement_mode_prev_on = 0;
static volatile uint8_t torque_measurement_warm_up_on = 1;
static volatile uint8_t torque_measurement_warm_up_cnt = 0;

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

static void StateError_Ent();
static void StateError_Run();
static void StateError_Ext();

/* ------------------- SACLING DATA ------------------- */
static void UpdateScaledData_Low(ScaledData* Scaled2ByteData);

/* ------------------- INITIALIZATION ------------------- */
static void Init_Kalman_Filter(KFObject * t_KF_obj, float t_A_KF, float t_B_KF, float t_C_KF, float t_Q_KF, float t_R_KF, float t_P_KF);
static void Init_Commutation();
static void Init_SensorUsage();
static void Init_Current_Ctrl();
static void Init_Position_Velocity();

/* ------------------- FOC ------------------- */
static void Get_3Phase_Current();
static void Get_3Phase_Voltage();
static void Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc, MotorOut* t_motor_out);
static void Cal_Trape_Total_Current();
static void Cal_Elec_Angle();
static void Cal_Elec_Angle_with_Hall();
static void Run_Kalman_Filter();
static void Run_FOC_Ctrl(float t_current_in);

/* ------------------- TRAPEZOIDAL ------------------- */
static void Run_Trape_Ctrl(float t_currentRef);

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Name(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Pole_Pair(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Enc_Resolution(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Gear_Ratio(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Torque_Constant(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Velocity_Constant(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Peak_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Continu_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Max_Velocity(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Commutation_Duty(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_User_Direction(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_Elec_Sys_ID_Mag(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Resistance(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Inductance(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_BEMF_ID_velocity(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_BEMF_ID_Gain_PCTG(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Curr_Ctrl_BW(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_Inertia(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Damping_Coef(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Mech_Model_a(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Mech_Model_b(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Friction_ID_Info(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Friction_LUT_Info(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

static void Set_Commutation_Sensor(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Position_Feedback_Sensor(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Elec_Angle_Homing_Sensor(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Mech_Angle_Homing_Sensor(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Sensor_Usage(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Aux_Input(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Aging_time(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Aging_Test_Done(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_MES(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_Joint_limit_Sw(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Enable_Torque_Measure(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

/* ----------- HALL SENSOR INTERRUPT ROUTINE ----------- */
void RunHallITCallback(uint16_t gpioPins);

/* ------------------- ROUTINE ------------------- */
static int Ent_Elec_Angle_Homing();
static int Run_Elec_Angle_Homing();

static int Ent_Set_Commutation();
static int Run_Set_Commutation();

static int Ent_Electrical_System_ID();
static int Run_Electrical_System_ID();

static int Ent_BEMF_ID();
static int Run_BEMF_ID();
static int Ext_BEMF_ID();

static int Ent_Current_Control();
static int Run_Current_Control();

static int Ent_Check_Current_Controller_Bandwidth();
static int Run_Check_Current_Controller_Bandwidth();
static int Ext_Check_Current_Controller_Bandwidth();

static int Ent_Advanced_Friction_ID();
static int Run_Advanced_Friction_ID();
static int Ext_Advanced_Friction_ID();

static int Ent_Evaluation_Friction_ID_Velocity_Ctrl();
static int Run_Evaluation_Friction_ID_Velocity_Ctrl();

static int Ent_Adv_Friction_Compensation();
static int Run_Adv_Friction_Compensation();

static int Ent_Adv_Friction_Compensation_FF();
static int Run_Adv_Friction_Compensation_FF();

static float FirstLPF(float currVal, float prevVal, float alpha);


static int Ent_AbsEnc_Position_Detection();
static int Run_AbsEnc_Position_Detection();
static int Ext_AbsEnc_Position_Detection(); // 250214 - JS,

static int Run_torque_measurement_jig3(void);

#ifdef _USE_HW_OVER_REV06
static void Get_Batt_VDC_Voltage();
static void Fault_SW_OCP();
static void Clear_Fault_SW_OCP();
static void Fault_VDC_OVP();
static void Clear_Fault_VDC_OVP();
static void Fault_VDC_UVP();
static void Clear_Fault_VDC_UVP();
static void Fault_HW_OCP();
static void Clear_Fault_HW_OCP();
static int Fault_Check_Current_Offset();
static void Clear_Fault_Check_Current_Offset();
static int Ent_IF_Spd_Control();
static int Run_IF_Spd_Control();
static int Ext_IF_Spd_Control();
#else
static void Check_OverCurrent();
#endif
static void Set_Stall_State(SetStall_t* pSetStall);

static void MaxAngleLimitCtrl(MaxAngleCtrl_t* pAngleLimit, float Wr);
static void Init_MaxAngleLimitCtrl(MaxAngleCtrl_t* pAngleLimit);
static void Validation_PosSensor(encCheck_t* ec);
static void Init_Validation_PosSensor(encCheck_t* ec);

static void Init_elec_angle_compensation(thetaComp_t* pAngleComp);
static void Get_rotor_pole_number(IOIF_AbsEnc_t* abs2, thetaComp_t* pAngleComp);
static void IncEnc_compensation(thetaComp_t* pAngleComp);
/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

DOP_COMMON_SDO_CB(low_level_ctrl_task)


void InitLowLvCtrl(void)
{
	/* init*/
	Init_SensorUsage();
   	Init_Commutation();
   	Init_Position_Velocity();
	InitFaultInfo(TASK_ID_LOWLEVEL, &low_level_ctrl_task.fault);

	/* Start DMA ADC1 for Current Sensor */
	if(IOIF_StartADCDMA(IOIF_ADC1, &rawCurr, IOIF_ADC1_BUFFER_LENGTH)) {
		//TODO: Error Process
	}

	Init_Kalman_Filter(&kf_current_object, kf_current_object.kf_A, kf_current_object.kf_B, kf_current_object.kf_C, 1, 10, 1);	// A, B, C, Q, R, P (Q = 1, R = 10)
	Init_PID(&pid_d_axis_ctrl, motor_properties.L * 0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), 0);
	Init_PID(&pid_q_axis_ctrl, motor_properties.L * 0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), 0);
	Init_PID(&pid_trape_ctrl, motor_properties.L * 0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R *  0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), 0);

	/* State Definition*/
	TASK_CREATE_STATE(&low_level_ctrl_task, TASK_STATE_OFF,     NULL,     		 	StateOff_Run,   	NULL, 	            true);
	TASK_CREATE_STATE(&low_level_ctrl_task, TASK_STATE_STANDBY, StateStandby_Ent, 	StateStandby_Run,   StateStandby_Ext, 	false);
	TASK_CREATE_STATE(&low_level_ctrl_task, TASK_STATE_ENABLE,  StateEnable_Ent,  	StateEnable_Run, 	StateEnable_Ext, 	false);
	TASK_CREATE_STATE(&low_level_ctrl_task, TASK_STATE_ERROR,   StateError_Ent,   StateError_Run,  	StateError_Ext, 				false);

	/* Routine Definition*/
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_COMMUTATION_SET,  				Ent_Set_Commutation, 			     		Run_Set_Commutation, 						NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ELECTRICAL_SYS_ID, 				Ent_Electrical_System_ID, 					Run_Electrical_System_ID, 					NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ELECTRICAL_BEMF_ID, 				Ent_BEMF_ID,								Run_BEMF_ID, 								Ext_BEMF_ID);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_CURRENT_CTRL, 					Ent_Current_Control, 						Run_Current_Control, 						NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_CURRENT_CTRL_BANDWIDTH_CHECK,	Ent_Check_Current_Controller_Bandwidth,		Run_Check_Current_Controller_Bandwidth,		Ext_Check_Current_Controller_Bandwidth);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADVANCED_FRICTION_ID,        	Ent_Advanced_Friction_ID,					Run_Advanced_Friction_ID, 					Ext_Advanced_Friction_ID);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADV_FRICTION_COMPENSATION,       Ent_Adv_Friction_Compensation,   			Run_Adv_Friction_Compensation,   			NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADV_FRICTION_ID_VEL_CTRL_EVAL,   Ent_Evaluation_Friction_ID_Velocity_Ctrl,   Run_Evaluation_Friction_ID_Velocity_Ctrl,   NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADV_FRICTION_COMPENSATION_FF,    Ent_Adv_Friction_Compensation_FF,  			Run_Adv_Friction_Compensation_FF,  			NULL);

    #ifdef _USE_HW_OVER_REV06
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_IF_SPD_CONTROL,    				Ent_IF_Spd_Control,  						Run_IF_Spd_Control,  						Ext_IF_Spd_Control);
#endif
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ABS_ENC_INITIAL_POSITION_DETECTION,    Ent_AbsEnc_Position_Detection,  		Run_AbsEnc_Position_Detection,  			Ext_AbsEnc_Position_Detection);

#ifdef _USE_HW_OVER_REV06
    /* Fault Recovery function Definition */
    CREATE_FAULT_RECOVERY(&low_level_ctrl_task.fault, OVER_CURRENT_SW_FAULT, Clear_Fault_SW_OCP);
    CREATE_FAULT_RECOVERY(&low_level_ctrl_task.fault, OVER_CURRENT_HW_FAULT, Clear_Fault_HW_OCP);
    CREATE_FAULT_RECOVERY(&low_level_ctrl_task.fault, OVER_VOLTAGE_VDC, Clear_Fault_VDC_OVP);
    CREATE_FAULT_RECOVERY(&low_level_ctrl_task.fault, UNDER_VOLTAGE_VDC, Clear_Fault_VDC_UVP);
    CREATE_FAULT_RECOVERY(&low_level_ctrl_task.fault, CURRENT_OFFSET_ERROR_PHASE_A, Clear_Fault_Check_Current_Offset);
    CREATE_FAULT_RECOVERY(&low_level_ctrl_task.fault, CURRENT_OFFSET_ERROR_PHASE_B, Clear_Fault_Check_Current_Offset);
    CREATE_FAULT_RECOVERY(&low_level_ctrl_task.fault, CURRENT_OFFSET_ERROR_PHASE_C, Clear_Fault_Check_Current_Offset);
#endif

	/* DOD Definition*/
	// DOD
	DOP_CreateDOD(TASK_ID_LOWLEVEL);

	// PDO
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_3PHASES_CURRENT_RAW,		DOP_INT32,  	3, &motor_out.I_U);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_3PHASES_CURRENT_KF,			DOP_FLOAT32, 	3, &motor_out.I_U_KF);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_RAW,		DOP_INT32,  	3, &motor_out.V_U);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_KF,			DOP_FLOAT32,  	3, &motor_out.V_U);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_POSITION,					DOP_FLOAT32, 	1, &motor_out.position);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VELOCITY,					DOP_FLOAT32, 	1, &motor_out.velocity);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CLARKE_OUT,					DOP_INT32, 		2, &clarke_out.Ia);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_PARK_OUT,					DOP_INT32, 		2, &park_out.Id);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VOLTAGE_IN,					DOP_FLOAT32, 	3, &motor_in.V_U_input);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ELEC_ANGLE,					DOP_UINT16,  	1, &park_in.theta_e);//BLDC_Motor.elec_angle);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_PRBS_DATA,					DOP_FLOAT32, 	2, &elec_system_id.f_signal);

	// DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT,		DOP_FLOAT32, 	1, &motor_in.total_current_input);
	// DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT,			DOP_FLOAT32, 	1, &motor_out.current_act);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT,		DOP_INT16, 		1, &FloatTo2ByteData_low.MotorRefCurrent);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT,				DOP_INT16, 		1, &FloatTo2ByteData_low.MotorActCurrent);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VOLTAGE_SCALING,			DOP_UINT16, 	1, &FloatTo2ByteData_low.MotorActVoltage);

	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_AUXILIARY_INPUT,			DOP_FLOAT32, 	1, &motor_in.auxiliary_input);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_F_VECTOR_INPUT,				DOP_FLOAT32, 	1, &motor_in.f_vector_input);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_LOW_ID_PROCESS_INPUT,		DOP_FLOAT32, 	1, &motor_in.low_id_process_input);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_FRICTION_COMPENSATOR_INPUT,	DOP_FLOAT32, 	1, &motor_in.friction_input);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_MID_ID_PROCESS_INPUT,		DOP_FLOAT32, 	1, &motor_in.mid_id_process_input);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_IRC_INPUT,					DOP_FLOAT32, 	1, &motor_in.irc_input);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_MID_CTRL_INPUT,			 	DOP_FLOAT32, 	1, &motor_in.mid_ctrl_input);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ANALYZER_INPUT,				DOP_FLOAT32, 	1, &motor_in.analysis_input);

	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_COMMUTATION_STEP,			DOP_UINT8, 		1, &motor_setting.commutation_set.comm_step);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_FRICTION_ID_REF,			DOP_FLOAT32, 	1, &advanced_friction_id.friction_ref);

	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_HALL_SENSOR_SIG,			DOP_UINT8, 		3, &hallObj.H1);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC, 			DOP_UINT8, 		1, &hallObj.hall_logic);

	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ACTUAL_VOLTAGE, 			DOP_FLOAT32, 	1, &motor_in.batt_vdc);
	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ACTUAL_CURRENT, 			DOP_FLOAT32, 	1, &motor_actual_current);

	DOP_CreatePDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ABS_ENC_OFFSET, 			DOP_UINT16, 	1, &motor_setting.commutation_set.abs_encoder_offset);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_LOWLEVEL)

	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_NAME,						DOP_STRING10, 	Set_Name);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_POLE_PAIR,					DOP_UINT8, 		Set_Pole_Pair);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ENCODER_RESOLUTION, 		DOP_UINT16, 	Set_Enc_Resolution);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_GEAR_RATIO, 				DOP_FLOAT32, 	Set_Gear_Ratio);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_TORQUE_CONSTANT,			DOP_FLOAT32, 	Set_Torque_Constant);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_VELOCITY_CONSTANT,			DOP_FLOAT32, 	Set_Velocity_Constant);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_PEAK_CURRENT_LIMIT, 		DOP_FLOAT32, 	Set_Peak_Limit);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CONTINUOUS_CURRENT_LIMIT, 	DOP_FLOAT32, 	Set_Continu_Limit);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_MAX_VELOCITY,				DOP_FLOAT32, 	Set_Max_Velocity);

	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_COMMUTATION_DUTY, 			DOP_UINT16, 	Set_Commutation_Duty);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_USER_DIRECTION, 			DOP_INT8, 		Set_User_Direction);

	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ELEC_SYSTEM_ID_MAG,			DOP_FLOAT32, 	Set_Elec_Sys_ID_Mag);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_TERMINAL_RESISTANCE, 		DOP_FLOAT32, 	Set_Resistance);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_TERMINAL_INDUCTANCE, 		DOP_FLOAT32, 	Set_Inductance);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_BEMF_ID_VELOCITY, 			DOP_FLOAT32, 	Set_BEMF_ID_velocity);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_BEMF_ID_GAIN_PCTG, 			DOP_UINT8, 		Set_BEMF_ID_Gain_PCTG);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_CTRL_BW_RAD, 		DOP_FLOAT32, 	Set_Curr_Ctrl_BW);

	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_INERTIA, 					DOP_FLOAT32, 	Set_Inertia);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DAMPING_COEF, 				DOP_FLOAT32, 	Set_Damping_Coef);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_MECH_MODEL_A, 				DOP_FLOAT32, 	Set_Mech_Model_a);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_MECH_MODEL_B, 				DOP_FLOAT32, 	Set_Mech_Model_b);

	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_FRICTION_ID_INFO,			DOP_FLOAT32, 	Set_Friction_ID_Info);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_FRICTION_LUT_INFO,			DOP_FLOAT32, 	Set_Friction_LUT_Info);

	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_COMMUTATION_SENSOR,		DOP_UINT8, 		Set_Commutation_Sensor);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_POS_FEEDBACK_SENSOR,	DOP_UINT8, 		Set_Position_Feedback_Sensor);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_E_ANGLE_HOMING_SENSOR,	DOP_UINT8, 		Set_Elec_Angle_Homing_Sensor);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_M_ANGLE_HOMING_SENSOR,	DOP_UINT8, 		Set_Mech_Angle_Homing_Sensor);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_SENSOR_USAGE,			DOP_UINT8, 		Set_Sensor_Usage);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_AUX_INPUT,				DOP_FLOAT32, 	Set_Aux_Input);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_AGING_TIME,				DOP_UINT8, 		Set_Aging_time);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_AGING_TEST_STOP,			DOP_UINT8, 		Set_Aging_Test_Done);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_MES,					DOP_STRING10, 	Set_MES);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT,		DOP_UINT8, 	Set_Joint_limit_Sw);
	DOP_CreateSDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ENABLE_TORQUE_MEASURE,		DOP_UINT8, 	Enable_Torque_Measure);

	DOP_SetSDOReq(TASK_ID_BLE_COMM_HDLR,  SDO_ID_BLE_COMM_HDLR_SERIAL_1, &MES.code[0], DOP_STRING10);
	DOP_SetSDOReq(TASK_ID_BLE_COMM_HDLR,  SDO_ID_BLE_COMM_HDLR_SERIAL_2, &MES.code[10], DOP_STRING10);
	DOP_SetSDOReq(TASK_ID_BLE_COMM_HDLR,  SDO_ID_BLE_COMM_HDLR_SERIAL_3, &MES.code[20], DOP_STRING10);
	DOP_SetSDOReq(TASK_ID_BLE_COMM_HDLR,  SDO_ID_BLE_COMM_HDLR_HW_VER, &MES.HWVer, 		DOP_UINT8);
	DOP_SetSDOReq(TASK_ID_BLE_COMM_HDLR,  SDO_ID_BLE_COMM_HDLR_SW_VER_MAJOR, &MES.FWVer.major, DOP_UINT8);
	DOP_SetSDOReq(TASK_ID_BLE_COMM_HDLR,  SDO_ID_BLE_COMM_HDLR_SW_VER_MINOR, &MES.FWVer.minor, DOP_UINT8);
	DOP_SetSDOReq(TASK_ID_BLE_COMM_HDLR,  SDO_ID_BLE_COMM_HDLR_SW_VER_PATCH, &MES.FWVer.patch, DOP_UINT8);
	DOP_SetSDOReq(TASK_ID_BLE_COMM_HDLR,  SDO_ID_BLE_COMM_HDLR_SW_VER_DEBUG, &MES.FWVer.debug, DOP_UINT16);

	/* Timer 1 Callback Allocation */
    if(IOIF_StartTimIT(IOIF_TIM1) > 0){					//25kHz
        //TODO: ERROR PROCESS
    }

#ifdef _USE_HW_OVER_REV06
    IOIF_SetTimRCR(IOIF_TIM1, TIM1_RCR_CNT);
#endif

    IOIF_StartTimOCIT(IOIF_TIM1, IOIF_TIM_CHANNEL_4);

	IOIF_SetTimCB(IOIF_TIM1, IOIF_TIM_OC_DELAY_ELAPSED_CALLBACK, RunLowLvCtrl, NULL);

	// Hall Sensor Interrupt callback
	IOIF_SetGPIOCB(IOIF_GPIO_PIN_6, IOIF_GPIO_EXTI_CALLBACK, RunHallITCallback);
	IOIF_SetGPIOCB(IOIF_GPIO_PIN_7, IOIF_GPIO_EXTI_CALLBACK, RunHallITCallback);
	IOIF_SetGPIOCB(IOIF_GPIO_PIN_8, IOIF_GPIO_EXTI_CALLBACK, RunHallITCallback);

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_RESET);
#endif
}

void RunLowLvCtrl(void* params)
{
	 // Loop Start Time Check
	 CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	 DWT->CYCCNT = 0;
	 DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	// IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, IOIF_GPIO_PIN_1, IOIF_GPIO_PIN_SET); // TP6_Pin

	// Run Device
	RunTask(&low_level_ctrl_task);

	// IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, IOIF_GPIO_PIN_1, IOIF_GPIO_PIN_SET); // TP6_Pin

	// Elapsed Time Check
	lowTimeElap = DWT->CYCCNT/480;	// in microsecond

	if(lowTimeElap > 40){
		RT_Broken++;
		#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_SET); // LED_BOOT_RED_Pin
		#endif
		#ifdef SUIT_MD_ENABLED
  			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_RESET); // STATUS_LED_B_Pin
		#endif
	}
}

void RunHallITCallback(uint16_t gpioPins)
{
	if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall)	Run_Elec_Angle_Homing();
}

/* ------------------- MAIN SEQUENCE FUNCTION ------------------- */
void Send_Aging_Test_Result()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[2] = {0};

	/* Send Data for Synchronization With UI */
	memcpy(&t_temp_arr[0], &AgingCtrl.state, 1);

	t_identifier = GUI_SYNC|GET_AGING_DONE;

	for (int i = 0; i<100000; ++i) {}

	Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 4);

	MS_enum = IDLE;
}

void Send_Elec_Angle_Value()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[4] = {0};
	float temp = 0;

	/* Send Data for Synchronization With UI */

	for (int i = 0; i < MOTOR_POLE_PAIR_INT; i++) {
		temp = i+1;
		memcpy(&t_temp_arr[0], &temp, 4);
		memcpy(&t_temp_arr[1], &motor_setting.commutation_set.raw_avg[i], 4);
//		memcpy(&t_temp_arr[2], &mid_level_process_array2[i], 4);

		for (int j = 0; j < 1000000; j++) {}

		t_identifier = GUI_SYNC|TORQUE_ACCURACY_DATA;
		Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 8);
	}

	for (int k = 0; k < 1000000; k++) {}

	memcpy(&t_temp_arr[0], &motor_setting.commutation_set.abs_encoder_offset, 4);
	memcpy(&t_temp_arr[1], &motor_setting.commutation_set.std_dev, 4);

	t_identifier = GUI_SYNC|GET_ELEC_ANGLE_HOMING_DONE;//GET_ELEC_ANGLE_HOMING_DATA;
	Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 8);

	MS_enum = IDLE;
}

void Cal_Elec_System_ID_Batch()
{
	double t_i = 0.0, t_i_prev = 0.0;
	double t_v = 0.0, t_v_prev = 0.0;
	double phi_11 = 0.0, phi_12 = 0.0, phi_22 = 0.0;
	double phi_sum_11 = 0.0, phi_sum_12 = 0.0, phi_sum_22 = 0;
	double out_sum_1 = 0.0, out_sum_2 = 0.0;
	double det_phi_sum = 0.0, inv_phi_sum_11 = 0.0, inv_phi_sum_12 = 0.0, inv_phi_sum_22 = 0.0;


	for (int i = 0; i < CHIRP_ARRAY_SIZE; ++i) {
		t_v = (double)(shared_array_buffer.buff1[i]); // voltage, unit: duty(0~4799)
		t_i = (double)(shared_array_buffer.buff2[i]); // current, unit: -32767 ~ 32768

		phi_11 = (t_i_prev) * (t_i_prev);
		phi_12 = (t_i_prev) * (t_v_prev);
		phi_22 = (t_v_prev) * (t_v_prev);

		phi_sum_11 += phi_11;
		phi_sum_12 += phi_12;
		phi_sum_22 += phi_22;

		out_sum_1 += (t_i_prev) * (t_i);
		out_sum_2 += (t_v_prev) * (t_i);

		t_i_prev = t_i;
		t_v_prev = t_v;
	}

	det_phi_sum = (phi_sum_11*phi_sum_22) - (phi_sum_12*phi_sum_12);
	inv_phi_sum_11 = phi_sum_22/det_phi_sum;
	inv_phi_sum_12 = -phi_sum_12/det_phi_sum;
	inv_phi_sum_22 = phi_sum_11/det_phi_sum;

	batch_result_1 = (inv_phi_sum_11*out_sum_1) + (inv_phi_sum_12*out_sum_2);
	batch_result_2 = (inv_phi_sum_12*out_sum_1) + (inv_phi_sum_22*out_sum_2);

	kf_current_object.kf_A = batch_result_1;		// a
	kf_current_object.kf_B = batch_result_2;		// b

	// i(k) = A*i(k-1) + B*u(k-1)

	/* Send Data for Synchronization With UI */
	uint16_t t_identifier;
	float t_temp_arr[2] = {0};

	memcpy(&t_temp_arr[0], &kf_current_object.kf_A, 4);
	memcpy(&t_temp_arr[1], &kf_current_object.kf_B, 4);

	for(int i = 0; i<25000; ++i){}

	t_identifier = GUI_SYNC|E_SYS_BATCH;
    Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 8);

	MS_enum = IDLE;
	return;
}

void Send_Elec_BEMF_Value()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[2] = {0};

	/* Send Data for Synchronization With UI */
	memcpy(&t_temp_arr[0], &kf_current_object.kf_C, 4);

	t_identifier = GUI_SYNC|E_SYS_BEMF;
	Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 4);

	MS_enum = IDLE;
}

void Send_Elec_Bandwidth_Data()
{
	uint16_t t_identifier = 0;
	float t_buf[15] = {0};
	int cnt = BW_CHECK_ARRAY_SIZE / 7;

	for (int i = 0; i < cnt; ++i) {
		t_buf[0] = (float)i;

		memcpy(&t_buf[1], &shared_array_buffer.buff1[i*7], 4);
		memcpy(&t_buf[2], &shared_array_buffer.buff2[i*7], 4);

		memcpy(&t_buf[3], &shared_array_buffer.buff1[i*7+1], 4);
		memcpy(&t_buf[4], &shared_array_buffer.buff2[i*7+1], 4);

		memcpy(&t_buf[5], &shared_array_buffer.buff1[i*7+2], 4);
		memcpy(&t_buf[6], &shared_array_buffer.buff2[i*7+2], 4);

		memcpy(&t_buf[7], &shared_array_buffer.buff1[i*7+3], 4);
		memcpy(&t_buf[8], &shared_array_buffer.buff2[i*7+3], 4);

		memcpy(&t_buf[9], &shared_array_buffer.buff1[i*7+4], 4);
		memcpy(&t_buf[10], &shared_array_buffer.buff2[i*7+4], 4);

		memcpy(&t_buf[11], &shared_array_buffer.buff1[i*7+5], 4);
		memcpy(&t_buf[12], &shared_array_buffer.buff2[i*7+5], 4);

		memcpy(&t_buf[13], &shared_array_buffer.buff1[i*7+6], 4);
		memcpy(&t_buf[14], &shared_array_buffer.buff2[i*7+6], 4);

		for (int j = 0; j<1000000; ++j) {}

		t_identifier = GUI_SYNC|BW_CHECK;
		Send_MSG(t_identifier, (uint8_t*)t_buf, 60);
	}


/*	uint16_t t_identifier = 0;
	float t_buf[3] = {0};

	for (int i = 0; i < BW_CHECK_ARRAY_SIZE; i++)
	{
		t_buf[0] = (float)i;

		memcpy(&t_buf[1], &shared_array_buffer.buff1[i], 4);
		memcpy(&t_buf[2], &shared_array_buffer.buff2[i], 4);

		for(int j = 0; j<200000; ++j){}

		t_identifier = GUI_SYNC|BW_CHECK;
		Send_MSG(t_identifier, (uint8_t*)t_buf, 12);
	}*/

	MS_enum = IDLE;
}


void Send_Advanced_Friction_ID_Data()
{
	uint16_t t_identifier = 0;

	float t_buf[3] = {0};

	for(int i = 0; i < 4 * (advanced_friction_id.vel_num1+advanced_friction_id.vel_num2); ++i) {
		t_buf[0] = (float)i;
		memcpy(&t_buf[1], &shared_array_buffer.buff1[i], 4);
		memcpy(&t_buf[2], &shared_array_buffer.buff2[i], 4);

		for (int j = 0; j<100000; ++j) {}

		t_identifier = GUI_SYNC|ADV_FRICTION_ID_DATA;
	    Send_MSG(t_identifier, (uint8_t*)t_buf, 12);
	}

	for (int i = 0; i<100000; ++i) {}

	Send_MSG((uint16_t)(GUI_SYNC|ADV_FRICTION_ID_DONE), (uint8_t*)0, 1);

	MS_enum = IDLE;
}

void Cal_Friction_LUT()
{
	double t_temp_act     = 0.0;
	double t_temp_desired = 0.0;
	double t_max_vel = 0.0, t_vel = 0.0;

	double Fs = 0.0;
	double Fc = 0.0;
	double Fv = 0.0;
	double vs = 0.0;
	double b = 0.0;
	double c = 0.0;

	double t_sign = 0.0;

	if ((motor_properties.gear_ratio * motor_properties.Ke) != 0) {
		t_max_vel = ((48 - motor_setting.peakCurr_limit * motor_properties.R) / (motor_properties.Ke) / (motor_properties.gear_ratio)) * 0.9;
	} else {
		t_max_vel = 0.0;
	}
	
	advanced_friction_id.scaling_factor = (2 * t_max_vel) / (FRICTION_LUT_SIZE - 1);

    for (int i = 0; i < FRICTION_LUT_SIZE; ++i) {
    	t_vel = (i - ((FRICTION_LUT_SIZE - 1) / 2)) * advanced_friction_id.scaling_factor;

		if (advanced_friction_id.lut_mdl == 1) { // Coulomb Viscous Model
			Fc = advanced_friction_id.lut_p1;
			Fv = advanced_friction_id.lut_p2;

			if ((advanced_friction_id.lut_epsilon) != 0) {
				t_temp_act = Fc * tanh((M_PI / 2) * (t_vel) / advanced_friction_id.lut_epsilon) + Fv * t_vel;
			} else {
				// TODO: Handle division by zero
				t_temp_act = 0;
			}
		} else if (advanced_friction_id.lut_mdl == 2) { // Stribeck 1
			Fs = advanced_friction_id.lut_p1;
			Fc = advanced_friction_id.lut_p2;
			vs = advanced_friction_id.lut_p3;
			b  = advanced_friction_id.lut_p4;

			if ((vs * advanced_friction_id.lut_epsilon) != 0) {
				t_temp_act = (Fs - (Fs - Fc) * exp(-(t_vel / vs) * (t_vel / vs))) * tanh((M_PI / 2) * (t_vel) / advanced_friction_id.lut_epsilon) + b * t_vel;
			} else {
				// TODO: Handle division by zero
				t_temp_act = 0;
			}
		} else if (advanced_friction_id.lut_mdl == 3) { // Stribeck 2
			Fs = advanced_friction_id.lut_p1;
			Fc = advanced_friction_id.lut_p2;
			vs = advanced_friction_id.lut_p3;
			b  = advanced_friction_id.lut_p4;
			c  = advanced_friction_id.lut_p5;

			t_sign = 0;
			if (t_vel >= 0) t_sign = +1;
			else            t_sign = -1;

			if ((vs * advanced_friction_id.lut_epsilon) != 0) {
				t_temp_act = (Fs - (Fs - Fc) * exp(-(t_vel / vs) * (t_vel / vs))) * tanh((M_PI / 2) * (t_vel) / advanced_friction_id.lut_epsilon) + b * t_vel + c * t_vel * t_vel * t_sign;
			} else {
				// TODO: Handle division by zero
				t_temp_act = 0;
			}
		}

		t_temp_desired = advanced_friction_id.lut_d_mu_v * t_vel;
		advanced_friction_id.adv_friction_compensator_LUT[i] = advanced_friction_id.gain * (t_temp_act - t_temp_desired);
	}

	MS_enum = IDLE;
}

void GetMES(){
	MES.HWVer=IOIF_HW_RevGPIO();
	MES.FWVer=IOIF_FW_SetRev();

}

void ImportMES_HWFW(){

	GetMES();
	static uint16_t t_identifier = 0;
	static uint8_t t_temp_arr[40] = {0};
	static uint8_t t_index=0;
	static uint8_t t_size=0;
	/* Send Data for Synchronization With UI */
	memcpy(&t_temp_arr[0], &MES.HWVer, 				1); 		t_index++;
	memcpy(&t_temp_arr[1], &MES.FWVer.major,       	1); 		t_index++;
	memcpy(&t_temp_arr[2], &MES.FWVer.minor,       	1); 		t_index++;
	memcpy(&t_temp_arr[3], &MES.FWVer.patch,      	1); 		t_index++;
	memcpy(&t_temp_arr[4], &MES.FWVer.debug,      	2); 		t_index+=2;//only for developers
	memcpy(&t_temp_arr[6], &MES.code_length,      	1); 		t_index++; 		t_size = MES.code_length;
	memcpy(&t_temp_arr[7], &MES.code,      			t_size); 	t_index+= t_size;

    IOIF_msDelay(15);


    t_identifier = GUI_SYNC|MEMORY_MES_HWFW;
    Send_MSG(t_identifier, t_temp_arr, t_index);//40);

    MS_enum = IDLE;
//    MS_enum=IMPORT_MES_CODE;
}

//void ImportMES_Code(){
//	static uint16_t t_identifier = 0;
//	static uint8_t t_temp_arr[40] = {0};
//
//	/* Send Data for Synchronization With UI */
//	memcpy(&t_temp_arr[0], &MES.code_length,      	1);
//	memcpy(&t_temp_arr[1], &MES.code,      			30);
//
//    IOIF_msDelay(15);
//
//
//    t_identifier = GUI_SYNC|MES_CODE_SET;
//    Send_MSG(t_identifier, t_temp_arr, 40);
//
//    MS_enum = IDLE;
//}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run()
{
	static uint32_t calib_cnt = 0;

	if (calib_cnt < 1000) {
		GateDriver_ONOFF(DISABLE);

		motor_in.total_current_input = 0.0;

		motor_out.I_U_offset += rawCurr[0];
		motor_out.I_V_offset += rawCurr[1];
		motor_out.I_W_offset += rawCurr[2];

		motor_out.V_U_offset += rawCurr[3];
		motor_out.V_V_offset += rawCurr[4];
		motor_out.V_W_offset += rawCurr[5];
	} else if(calib_cnt == 1000) {
		GateDriver_ONOFF(DISABLE);
		motor_in.total_current_input = 0.0;
		calib_cnt = 0;

		motor_out.I_U_offset = motor_out.I_U_offset / 1000;
		motor_out.I_V_offset = motor_out.I_V_offset / 1000;
		motor_out.I_W_offset = motor_out.I_W_offset / 1000;

		motor_out.V_U_offset = motor_out.V_U_offset / 1000;
		motor_out.V_V_offset = motor_out.V_V_offset / 1000;
		motor_out.V_W_offset = motor_out.V_W_offset / 1000;

		motor_out.I_U_offset = motor_out.I_U_offset - (ADC1_RESOLUTION>>1);
		motor_out.I_V_offset = motor_out.I_V_offset - (ADC1_RESOLUTION>>1);
		motor_out.I_W_offset = motor_out.I_W_offset - (ADC1_RESOLUTION>>1);

#ifdef _USE_HW_OVER_REV06
		if(Fault_Check_Current_Offset()){
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		}
		else{
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_ERROR);
		}
	#else
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
#endif//_USE_HW_OVER_REV06
	}

	calib_cnt++;
}

static void StateStandby_Ent()
{
	GateDriver_ONOFF(DISABLE);
	Init_Current_Ctrl();
}

// For Debug //
 uint8_t oneTimeTransition = 0;
static void StateStandby_Run()
{
	Get_3Phase_Current();
	Get_3Phase_Voltage();

#ifdef _USE_HW_OVER_REV06
	Get_Batt_VDC_Voltage();
	Fault_VDC_OVP();
	Fault_VDC_UVP();
#endif//_USE_HW_OVER_REV06

	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Get_Position_Velocity(&inc25KhzObj, &motor_out);
	Cal_Elec_Angle();
#ifndef _USE_SYSTEM_ID
	if (oneTimeTransition == 1) {
		PushRoutine(&low_level_ctrl_task.routine, ROUTINE_ID_LOWLEVEL_CURRENT_CTRL);
		//		PushRoutine(&low_level_ctrl_task.routine, ROUTINE_ID_LOWLEVEL_IF_SPD_CONTROL);
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_ENABLE);
		oneTimeTransition = 0;
	}
#endif
}

static void StateStandby_Ext()
{
	GateDriver_ONOFF(DISABLE);
}

static void StateEnable_Ent()
{
	Init_elec_angle_compensation(&thetaComp);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2); // Don't remove. This code was implemented to improve the accuracy of the Homing Angle //
	Init_MaxAngleLimitCtrl(&MaxAngleLimit);
	Ent_Elec_Angle_Homing();
	EntRoutines(&low_level_ctrl_task.routine);
	low_level_ctrl_loop_cnt = 0;
	GateDriver_ONOFF(ENABLE);
	Stop_PWM();
}

// For Debug //
uint8_t testCmd = 0;

float position = 0;
uint16_t x = 0;
uint16_t abs_encoder_e_angle_offset = 0;
float    abs_encoder_m_angle_deg = 0;
uint16_t abs_encoder_m_angle_cnt = 0;
uint16_t m_angle_offset;
uint16_t abs_e_angle;
int8_t sign = 0;
uint32_t temp = 0;
uint16_t elec_angle = 0;
uint8_t init_sw = 0;
uint8_t homing_sw = 0;
uint32_t userCnt = 0;
static uint8_t tTestCnt=0;
static void StateEnable_Run()
{

//	if(init_sw==1){
//
//		if (position < 0)
//		{
//			position = position * (-1);
//			x = (position * 182.0444444444445) ;
//			x = x *(-1) ;
//		}
//		else if (position >= 0)
//		{
//			x = (position * 182.0444444444445);
//		}
//
//
//		if (motor_properties.pole_pair != 0) {
//			abs_encoder_e_angle_offset = (x*motor_properties.pole_pair);
//		} else {
//			abs_encoder_e_angle_offset = 0;
//		}
//		init_sw=0;
//	}
//
//	if(homing_sw==1){
//
//		if (abs_encoder_m_angle_deg < 0)
//		{
//			abs_encoder_m_angle_deg = abs_encoder_m_angle_deg * (-1);
//			abs_encoder_m_angle_cnt = (abs_encoder_m_angle_deg * 182.0444444444445) ;
//			abs_encoder_m_angle_cnt = abs_encoder_m_angle_cnt *(-1) ;
//		}
//		else if (abs_encoder_m_angle_deg >= 0)
//		{
//			abs_encoder_m_angle_cnt = (abs_encoder_m_angle_deg * 182.0444444444445);
//		}
//
//		sign = motor_setting.commutation_set.ea_dir * motor_setting.commutation_set.ma_dir;
//		m_angle_offset = sign * (abs_encoder_m_angle_cnt);
//
//		if (motor_properties.pole_pair != 0) {
//			abs_e_angle = m_angle_offset*motor_properties.pole_pair - abs_encoder_e_angle_offset;
//		} else {;
//			abs_e_angle =0;
//		}
//
//		temp = ((uint32_t)userCnt) * ((uint32_t)(65536 / (double)8192));
//
//		elec_angle         = motor_setting.commutation_set.ea_dir * temp*motor_properties.pole_pair;
//		motor_out.elec_angle = elec_angle + motor_setting.elec_angle_homing.offset;
//
//		motor_setting.elec_angle_homing.offset = abs_e_angle;
//		homing_sw = 0;
//	}
//
//	temp = ((uint32_t)userCnt) * ((uint32_t)(65536 / (double)8192));
//
//	elec_angle         = motor_setting.commutation_set.ea_dir * temp*motor_properties.pole_pair;
//	motor_out.elec_angle = elec_angle + motor_setting.elec_angle_homing.offset;


	if (motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Hall)	Run_Elec_Angle_Homing();

	Cal_Elec_Angle();
	IncEnc_compensation(&thetaComp);

	// Safety Condition for Enable Run
	if ((motor_setting.commutation_set.start == 1) |  \
	    (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall) | \
		((motor_setting.commutation_set.done == 1) & (motor_setting.elec_angle_homing.done == 1)) | (AgingCtrl.aging_time) )
	{
		Get_3Phase_Current();
		Get_3Phase_Voltage();

#ifdef _USE_HW_OVER_REV06
		Get_Batt_VDC_Voltage();
		Fault_VDC_OVP();
		Fault_VDC_UVP();
		Fault_HW_OCP();
#endif//_USE_HW_OVER_REV06

		IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
		Get_Position_Velocity(&inc25KhzObj, &motor_out);
		UpdateScaledData_Low(&FloatTo2ByteData_low);

#ifndef _USE_SYSTEM_ID
//		 For Debug //
		if (testCmd == 1) {
			Run_Set_Commutation();
		}
#endif
		Run_torque_measurement_jig3();
		RunRoutines(&low_level_ctrl_task.routine);

		// IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_RESET); // STATUS_LED_B_Pin

		low_level_ctrl_loop_cnt++;
	}
}

static void StateEnable_Ext()
{
	motor_setting.elec_angle_homing.offset = 0;
	motor_setting.elec_angle_homing.done = 0;
	ExtRoutines(&low_level_ctrl_task.routine);
	GateDriver_ONOFF(DISABLE);
	Stop_PWM();
}

static void StateError_Ent()
{
#ifdef _USE_HW_OVER_REV06
	FaultNumCntConsistencyCheck(&low_level_ctrl_task.fault);
	FaultPacketDef(TASK_ID_LOWLEVEL, &low_level_ctrl_task.fault);
	Send_EMCY(&low_level_ctrl_task.fault.packet);
#endif
}

static void StateError_Run()
{
	Get_3Phase_Current();
	Get_3Phase_Voltage();

#ifdef _USE_HW_OVER_REV06
	Get_Batt_VDC_Voltage();
	FaultRecoveryFuncRun(&low_level_ctrl_task.fault);
	FaultPacketDef(TASK_ID_LOWLEVEL, &low_level_ctrl_task.fault);
#endif

	if(low_level_ctrl_task.fault.faultNumCnt == 0){
		low_level_ctrl_task.fault.faultBit = 0;
		StateTransition(&low_level_ctrl_task.stateMachine, low_level_ctrl_task.fault.restart_state);
	}
}


static void StateError_Ext()
{
#ifdef _USE_HW_OVER_REV06
	InitFaultInfo(TASK_ID_LOWLEVEL, &low_level_ctrl_task.fault);
	Send_EMCY(&low_level_ctrl_task.fault.packet);
#endif
}

/* ------------------- SACLING DATA ------------------- */
static void UpdateScaledData_Low(ScaledData* Scaled2ByteData)
{
	// Scaling for PDO Data
	Scaled2ByteData->MotorRefCurrent = ScaleFloatToInt16(motor_in.total_current_input, CURRENT_SCALING_FACTOR);
	Scaled2ByteData->MotorActCurrent = ScaleFloatToInt16(motor_actual_current, CURRENT_SCALING_FACTOR);
	Scaled2ByteData->MotorActVoltage = ScaleFloatToUInt16((motor_in.batt_vdc)/2480, VOLT_SCALING_FACTOR);
}

/* ------------------- INITIALIZATION ------------------- */
static void Init_Kalman_Filter(KFObject * t_KF_obj, float t_A_KF, float t_B_KF, float t_C_KF, float t_Q_KF, float t_R_KF, float t_P_KF)
{
   memset(t_KF_obj, 0, sizeof(KFObject));

   t_KF_obj->kf_A = t_A_KF;
   t_KF_obj->kf_B = t_B_KF;
   t_KF_obj->kf_C = t_C_KF;
   t_KF_obj->kf_Q = t_Q_KF;
   t_KF_obj->kf_R = t_R_KF;
   t_KF_obj->kf_P = t_P_KF;
}

static void Init_SensorUsage()
{
	if       (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder) {
		motor_setting.sensor_setting.incremetnal_encoder_usage = 1;
	} else if (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Hall) {
		motor_setting.sensor_setting.hall_sensor_usage = 1;
	} else if (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder_Hall) {
		motor_setting.sensor_setting.incremetnal_encoder_usage = 1;
		motor_setting.sensor_setting.hall_sensor_usage = 1;
	}

	if       (motor_setting.sensor_setting.pos_feedback_sensor == e_Pos_Sensor_Inc_Encoder) {
		motor_setting.sensor_setting.incremetnal_encoder_usage = 1;
	} else if (motor_setting.sensor_setting.pos_feedback_sensor == e_Pos_Sensor_Abs_Encoder1) {
		motor_setting.sensor_setting.absolute_encoder1_usage = 1;
	} else if (motor_setting.sensor_setting.pos_feedback_sensor == e_Pos_Sensor_Abs_Encoder2) {
		motor_setting.sensor_setting.absolute_encoder2_usage = 1;
	}

	if       (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall) {
		motor_setting.sensor_setting.hall_sensor_usage = 1;
	} else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder1) {
		motor_setting.sensor_setting.absolute_encoder1_usage = 1;
	} else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder2) {
		motor_setting.sensor_setting.absolute_encoder2_usage = 1;
	}

	if       (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Abs_Encoder1) {
		motor_setting.sensor_setting.absolute_encoder1_usage = 1;
	} else if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Abs_Encoder2) {
		motor_setting.sensor_setting.absolute_encoder2_usage = 1;
	} else if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_MD_Calculation) {
		motor_setting.sensor_setting.incremetnal_encoder_usage = 1;
	}

	if (AbsObj1.location != IOIF_ABS_ENC_NONE)
		motor_setting.sensor_setting.absolute_encoder1_usage = 1;

	if (AbsObj2.location != IOIF_ABS_ENC_NONE)
		motor_setting.sensor_setting.absolute_encoder2_usage = 1;

	if ((motor_setting.sensor_setting.incremetnal_encoder_usage != 1) &&
		(motor_setting.sensor_setting.incremetnal_encoder_usage != 0))
	{
		motor_setting.sensor_setting.incremetnal_encoder_usage = 0;
	}

	if ((motor_setting.sensor_setting.absolute_encoder1_usage != 1) &&
		(motor_setting.sensor_setting.absolute_encoder1_usage != 0))
	{
		motor_setting.sensor_setting.absolute_encoder1_usage = 0;
	}

	if ((motor_setting.sensor_setting.absolute_encoder2_usage != 1) &&
		(motor_setting.sensor_setting.absolute_encoder2_usage != 0))
	{
		motor_setting.sensor_setting.absolute_encoder2_usage = 0;
	}

	if ((motor_setting.sensor_setting.hall_sensor_usage != 1) &&
		(motor_setting.sensor_setting.hall_sensor_usage != 0))
	{
		motor_setting.sensor_setting.hall_sensor_usage = 0;
	}

	if ((motor_setting.sensor_setting.temperature_sensor_usage != 1) &&
		(motor_setting.sensor_setting.temperature_sensor_usage != 0))
	{
		motor_setting.sensor_setting.temperature_sensor_usage = 0;
	}

	if ((motor_setting.sensor_setting.imu_6axis_usage != 1) &&
		(motor_setting.sensor_setting.imu_6axis_usage != 0))
	{
		motor_setting.sensor_setting.imu_6axis_usage = 0;
	}

	if ((motor_setting.sensor_setting.imu_3axis_usage != 1) &&
		(motor_setting.sensor_setting.imu_3axis_usage != 0))
	{
		motor_setting.sensor_setting.imu_3axis_usage = 0;
	}
}

static void Init_Commutation()
{
//	if (motor_setting.commutation_set.done != 1) {
//		motor_setting.commutation_set.done   = 0;
//		motor_setting.commutation_set.ea_dir = +1;
//		motor_setting.commutation_set.ma_dir = +1;
//		motor_setting.commutation_set.cc_dir = +1;
//	}
//
//	motor_setting.commutation_set.max_duty           = 300;
//	motor_setting.commutation_set.state              = 0;
//	motor_setting.commutation_set.time_stamp         = 0;
//	motor_setting.commutation_set.comm_step          = 0;
//	motor_setting.commutation_set.user_desired_dir   = 0;

//	if((motor_setting.sensor_setting.commutation_sensor != e_Commute_Sensor_Inc_Encoder) &&
//	   (motor_setting.sensor_setting.commutation_sensor != e_Commute_Sensor_Hall) &&
//	   (motor_setting.sensor_setting.commutation_sensor != e_Commute_Sensor_Inc_Encoder_Hall))
//	{
//		motor_setting.sensor_setting.commutation_sensor = e_Commute_Sensor_Inc_Encoder;
//	}
//
//	if((motor_setting.sensor_setting.pos_feedback_sensor != e_Pos_Sensor_None) &&
//	   (motor_setting.sensor_setting.pos_feedback_sensor != e_Pos_Sensor_Inc_Encoder) &&
//	   (motor_setting.sensor_setting.pos_feedback_sensor != e_Pos_Sensor_Abs_Encoder1) &&
//	   (motor_setting.sensor_setting.pos_feedback_sensor != e_Pos_Sensor_Abs_Encoder2))
//	{
//		motor_setting.sensor_setting.pos_feedback_sensor = e_Pos_Sensor_None;
//	}
//
//	if((motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Forced) &&
//	   (motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Hall) &&
//	   (motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Abs_Encoder1) &&
//	   (motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Abs_Encoder2))
//	{
//		motor_setting.sensor_setting.e_angle_homing_sensor = e_EHoming_Sensor_Forced;
//	}
//
//	if((motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_Zero) &&
//	   (motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_Abs_Encoder1) &&
//	   (motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_Abs_Encoder2) &&
//	   (motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_MD_Calculation) &&
//	   (motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_CM_Setting))
//	{
//		motor_setting.sensor_setting.m_angle_homing_sensor = e_MHoming_Sensor_Zero;
//	}

//	motor_setting.elec_angle_homing.offset = 0;
//	motor_setting.elec_angle_homing.forced_homing_cnt = 0;

	if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);
}

static void Init_Current_Ctrl()
{
	pid_d_axis_ctrl.err = 0;
	pid_d_axis_ctrl.err_sum = 0;
	pid_q_axis_ctrl.err = 0;
	pid_q_axis_ctrl.err_sum = 0;
	pid_trape_ctrl.err = 0;
	pid_trape_ctrl.err_sum = 0;
	motor_out.I_mag = 0;
	motor_actual_current = 0;
}

static void Init_Position_Velocity()
{
	motor_out.position = 0;
	motor_out.position_f = 0;
	motor_out.velocity = 0;
	motor_out.velocity_f = 0;
}

#ifdef _USE_HW_OVER_REV06
static void Fault_SW_OCP()
{
	static int8_t OcpCnt = 0;

	if(motor_setting.peakCurr_limit <= 0){
		motor_setting.peakCurr_limit = FAULT_CURRENT_OCP_DEFAULT_LV;
	}

	if (motor_out.I_mag >= motor_setting.peakCurr_limit){
		OcpCnt++;
		if (OcpCnt >= FAULT_CURRENT_OCP_CNT) {
			OcpCnt = 0;

			GateDriver_ONOFF(DISABLE);
			Stop_PWM();
			SetFaultInfo(&low_level_ctrl_task.fault, TEMPORARY, OVER_CURRENT_SW_FAULT, TASK_STATE_STANDBY, 0);
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_ERROR);
		}
	}
	else {
		OcpCnt--;
		if (OcpCnt < 0) OcpCnt = 0;
	}
}

static void Clear_Fault_SW_OCP()
{
	ClearFaultInfo(&low_level_ctrl_task.fault, OVER_CURRENT_SW_FAULT);
}

static void Fault_HW_OCP()
{
	// HW OCP is non-reaction fault //
	if((IOIF_ReadGPIOPin(IOIF_GPIO_PORT_C, BSP_GPIO_PIN_6)) == 0){
		GateDriver_ONOFF(DISABLE);
		Stop_PWM();
		SetFaultInfo(&low_level_ctrl_task.fault, TEMPORARY, OVER_CURRENT_HW_FAULT, TASK_STATE_STANDBY, 0);
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_ERROR); 	// HW OCP is non-reaction fault
	}
}

static void Clear_Fault_HW_OCP()
{
	ClearFaultInfo(&low_level_ctrl_task.fault, OVER_CURRENT_HW_FAULT);
}

static void Fault_VDC_OVP()
{
	static int16_t OvpCnt = 0;

	if (motor_in.batt_vdc >= FAULT_VDC_OVP_SET_LV){
		OvpCnt++;
		if (OvpCnt >= FAULT_VDC_OVP_CNT) {
			OvpCnt = 0;
			SetFaultInfo(&low_level_ctrl_task.fault, TEMPORARY, OVER_VOLTAGE_VDC, TASK_STATE_STANDBY, 0);
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_ERROR);
		}
	}
	else {
		OvpCnt--;
		if (OvpCnt < 0) OvpCnt = 0;
	}
}

static void Clear_Fault_VDC_OVP()
{
	static int16_t ClrOvpCnt = 0;

	if(motor_in.batt_vdc < FAULT_VDC_OVP_CLEAR_LV){
		ClrOvpCnt++;
		if(ClrOvpCnt >= FAULT_VDC_OVP_CNT){
			ClrOvpCnt = 0;
			ClearFaultInfo(&low_level_ctrl_task.fault, OVER_VOLTAGE_VDC);
		}
	}
	else {
		ClrOvpCnt--;
		if (ClrOvpCnt < 0) ClrOvpCnt = 0;
	}
}

static void Fault_VDC_UVP()
{
	static int16_t UvpCnt = 0;

	if (motor_in.batt_vdc <= FAULT_VDC_UVP_SET_LV){
		UvpCnt++;
		if (UvpCnt >= FAULT_VDC_UVP_CNT) {
			UvpCnt = 0;
			SetFaultInfo(&low_level_ctrl_task.fault, TEMPORARY, UNDER_VOLTAGE_VDC, TASK_STATE_STANDBY, 0);
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_ERROR);
		}
	}
	else {
		UvpCnt--;
		if (UvpCnt < 0) UvpCnt = 0;
	}
}

static void Clear_Fault_VDC_UVP()
{
	static int16_t ClrUvpCnt = 0;

	if(motor_in.batt_vdc > FAULT_VDC_UVP_CLEAR_LV){
		ClrUvpCnt++;
		if(ClrUvpCnt >= FAULT_VDC_OVP_CNT){
			ClrUvpCnt = 0;
			ClearFaultInfo(&low_level_ctrl_task.fault, UNDER_VOLTAGE_VDC);
		}
	}
	else {
		ClrUvpCnt--;
		if (ClrUvpCnt < 0) ClrUvpCnt = 0;
	}
}

static int Fault_Check_Current_Offset()
{
	int16_t offsetA = 0, offsetB = 0, offsetC = 0;
	static int8_t ErrorCntA = 0, ErrorCntB = 0, ErrorCntC = 0;

	if(motor_out.I_U_offset < 0)	offsetA = -motor_out.I_U_offset;
	else	offsetA = motor_out.I_U_offset;

	if(motor_out.I_V_offset < 0)	offsetB = -motor_out.I_V_offset;
	else	offsetB = motor_out.I_V_offset;

	if(motor_out.I_W_offset < 0)	offsetC = -motor_out.I_W_offset;
	else	offsetC = motor_out.I_W_offset;

	if(offsetA > FAULT_CURRENT_OFFSET_LV){
		ErrorCntA++;
		if(ErrorCntA >= FAULT_CURRENT_OFFSET_CNT){
			SetFaultInfo(&low_level_ctrl_task.fault, CONFIRMED, CURRENT_OFFSET_ERROR_PHASE_A, TASK_STATE_OFF, 0);
		}
		else{
			SetFaultInfo(&low_level_ctrl_task.fault, TEMPORARY, CURRENT_OFFSET_ERROR_PHASE_A, TASK_STATE_OFF, 0);
		}
	}
	else{
		ErrorCntA = 0;
	}

	if(offsetB > FAULT_CURRENT_OFFSET_LV){
		ErrorCntB++;
		if(ErrorCntB >= FAULT_CURRENT_OFFSET_CNT){
			SetFaultInfo(&low_level_ctrl_task.fault, CONFIRMED, CURRENT_OFFSET_ERROR_PHASE_B, TASK_STATE_OFF, 0);
		}
		else{
			SetFaultInfo(&low_level_ctrl_task.fault, TEMPORARY, CURRENT_OFFSET_ERROR_PHASE_B, TASK_STATE_OFF, 0);
		}
	}
	else{
		ErrorCntB = 0;
	}

	if(offsetC > FAULT_CURRENT_OFFSET_LV){
		ErrorCntC++;
		if(ErrorCntC >= FAULT_CURRENT_OFFSET_CNT){
			SetFaultInfo(&low_level_ctrl_task.fault, CONFIRMED, CURRENT_OFFSET_ERROR_PHASE_C, TASK_STATE_OFF, 0);
		}
		else{
			SetFaultInfo(&low_level_ctrl_task.fault, TEMPORARY, CURRENT_OFFSET_ERROR_PHASE_C, TASK_STATE_OFF, 0);
		}
	}
	else{
		ErrorCntC = 0;
	}

	if((ErrorCntA == 0) && (ErrorCntB == 0) && (ErrorCntC == 0)){
		motor_out.crnt_offset_done = 1;
	}
	else{
		motor_out.crnt_offset_done = 0;
	}

	return motor_out.crnt_offset_done;
}


static void Clear_Fault_Check_Current_Offset()
{
	ClearFaultInfo(&low_level_ctrl_task.fault, CURRENT_OFFSET_ERROR_PHASE_A);
	ClearFaultInfo(&low_level_ctrl_task.fault, CURRENT_OFFSET_ERROR_PHASE_B);
	ClearFaultInfo(&low_level_ctrl_task.fault, CURRENT_OFFSET_ERROR_PHASE_C);
}

static void Set_Stall_State(SetStall_t* pSetStall)
{
	pSetStall->curr_tgt = fabsf(pSetStall->curr_tgt);

	pSetStall->sector_old = pSetStall->sector;

	if		(motor_out.elec_angle > 54613)	{pSetStall->sector = 5;} // 300
	else if (motor_out.elec_angle > 43690)	{pSetStall->sector = 4;} // 240
	else if (motor_out.elec_angle > 32768)	{pSetStall->sector = 3;} // 180
	else if (motor_out.elec_angle > 21845)	{pSetStall->sector = 2;} // 120
	else if (motor_out.elec_angle > 10922)	{pSetStall->sector = 1;} // 60
	else									{pSetStall->sector = 0;}

	if(pSetStall->curr_tgt >= STALL_CRNT_DETECT){

		if(pSetStall->activated == 0){
			if(pSetStall->sector == pSetStall->sector_old)
			{
				pSetStall->stall_tmr++;
				if(pSetStall->stall_tmr >= STALL_SET_TIME){ // state 1

					pSetStall->sector_hysH = pSetStall->sector + 2;
					if(pSetStall->sector_hysH >= 6) {pSetStall->sector_hysH -= 6;}

					pSetStall->sector_hysL = pSetStall->sector - 2;
					if(pSetStall->sector_hysL < 0) {pSetStall->sector_hysL += 6;}

					pSetStall->activated = 1;
				}
			}
			else{
				pSetStall->stall_tmr = 0;
			}
		}
		else{
			pSetStall->stall_tmr = 0;
			if((pSetStall->sector == pSetStall->sector_hysH) || (pSetStall->sector == pSetStall->sector_hysL)){
				pSetStall->activated = 0;
				pSetStall->sector_hysH = 0;
				pSetStall->sector_hysL = 0;
			}
		}
	}
	else{
		pSetStall->activated = 0;
		pSetStall->sector_hysH = 0;
		pSetStall->sector_hysL = 0;
	}
}

#else
static void Check_OverCurrent()
{
	static uint32_t overcurrent_cnt = 0;

	if((motor_out.current_act > motor_setting.contCurr_limit) || (motor_out.current_act < -motor_setting.contCurr_limit)) {
		overcurrent_cnt++;
	} else {
		overcurrent_cnt = 0;
	}
	// TODO: Error Process
	if (overcurrent_cnt > 75000) {
		overcurrent_cnt = 0;
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		low_level_ctrl_task.errCode = ERROR_PHASE_OVER_CURRENT;
	}
}
#endif //_USE_HW_OVER_REV06

/* ------------------- FOC ------------------- */
static void Get_3Phase_Current()
{
	motor_out.I_U = rawCurr[0] - 32768 - motor_out.I_U_offset;
	motor_out.I_V = rawCurr[1] - 32768 - motor_out.I_V_offset;
	motor_out.I_W = rawCurr[2] - 32768 - motor_out.I_W_offset;
}

static void Get_3Phase_Voltage()
{
	motor_out.V_U = rawCurr[3] - motor_out.V_U_offset;
	motor_out.V_V = rawCurr[4] - motor_out.V_V_offset;
	motor_out.V_W = rawCurr[5] - motor_out.V_W_offset;
}
#ifdef _USE_HW_OVER_REV06
static void Get_Batt_VDC_Voltage()
{
	motor_in.batt_vdc = rawCurr[6];
	motor_in.batt_vdc_filt = FirstLPF(motor_in.batt_vdc, motor_in.batt_vdc_old, BAT_VDC_FILTER_WEIGHT);
	motor_in.batt_vdc_old = motor_in.batt_vdc_filt;
	motor_in.batt_vdc = (motor_in.batt_vdc_filt*VDC_GAIN_SCALE_ADJ); // Gain Scale
}
#endif //_USE_HW_OVER_REV06

static void Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc,  MotorOut* t_motor_out)
{
//#ifdef _USE_SYSTEM_ID
	tTestCnt++;
//#endif
	// 분모가 0인지 확인합니다.
    if (t_inc_enc->resolution * motor_properties.gear_ratio != 0) {
        t_motor_out->position = motor_setting.commutation_set.ma_dir * (t_inc_enc->userCnt / (t_inc_enc->resolution * motor_properties.gear_ratio) * 2* M_PI); 		// in rad
    } else {
        // TODO: Handle division by zero
        t_motor_out->position = 0.0; // 또는 적절한 오류 처리 코드
    }

	/*Low Pass Filter*/
	float temp = (t_motor_out->position - t_motor_out->position_f) * LOW_LEVEL_CONTROL_FREQUENCY;
	t_motor_out->velocity_raw = (t_motor_out->position - t_motor_out->position_f) * LOW_LEVEL_CONTROL_FREQUENCY;

	if (isnan(t_motor_out->velocity)) {
		t_motor_out->velocity = temp;
	} else {
		//t_motor_out->velocity = 0.817862*t_motor_out->velocity + 0.182138*temp;		// 800Hz low-pass filter

		if (advanced_friction_id.state == 0)
			t_motor_out->velocity = 0.963*t_motor_out->velocity + 0.037*temp;		        // 300Hz low-pass filter
		else
			t_motor_out->velocity = 0.9962*t_motor_out->velocity + 0.0038*temp;		        // 30Hz low-pass filter
	}

	t_motor_out->position_f = t_motor_out->position;
	t_motor_out->velocity_f = t_motor_out->velocity;
}

static void Cal_Trape_Total_Current()
{
	float total_current = 0.0;
	float temp_current = 0.0;

	if      (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[0]) {total_current = (motor_out.I_V - motor_out.I_W)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[1]) {total_current = (motor_out.I_V - motor_out.I_U)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[2]) {total_current = (motor_out.I_W - motor_out.I_U)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[3]) {total_current = (motor_out.I_W - motor_out.I_V)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[4]) {total_current = (motor_out.I_U - motor_out.I_V)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[5]) {total_current = (motor_out.I_U - motor_out.I_W)*0.5; }

	temp_current = total_current * 0.0004425049; // 29/65535
	motor_out.current_act = motor_out.current_act * 0.99 + temp_current * 0.01;
}

static void Cal_Elec_Angle()
{
	uint32_t temp = 0;
	uint16_t t_elec_angle = 0;

	if (inc25KhzObj.resolution != 0) {
		temp = ((uint32_t)inc25KhzObj.userCnt) * ((uint32_t)(65536 / (double)inc25KhzObj.resolution));
	} else {
		// TODO: Handle division by zero
		temp = 0;
	}
//	t_elec_angle         = motor_setting.commutation_set.ea_dir * ((temp * motor_properties.pole_pair) - (((temp * motor_properties.pole_pair) >> 16) << 16));
	t_elec_angle         = motor_setting.commutation_set.ea_dir * temp * motor_properties.pole_pair;
	motor_out.elec_angle = t_elec_angle + motor_setting.elec_angle_homing.offset;
}

static void Cal_Elec_Angle_with_Hall()
{
	if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);

	if (motor_setting.commutation_set.cc_dir * motor_in.total_current_input >= 0) {
		if      (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[0]) motor_out.elec_angle = +5461;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[1]) motor_out.elec_angle = +16384;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[2]) motor_out.elec_angle = +27307;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[3]) motor_out.elec_angle = +38229;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[4]) motor_out.elec_angle = +49152;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[5]) motor_out.elec_angle = +60075;
		else { /* Error Handler */ }
	} else {
		if      (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[0]) motor_out.elec_angle = +60075;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[1]) motor_out.elec_angle = +5461;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[2]) motor_out.elec_angle = +16384;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[3]) motor_out.elec_angle = +27307;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[4]) motor_out.elec_angle = +38229;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[5]) motor_out.elec_angle = +49152;
		else { /* Error Handler */ }
	}
}

static void Run_Kalman_Filter()
{
	static float u_A = 0.0, u_B = 0.0, u_C = 0.0, u_N = 0.0;
	uint16_t e_angle_u = 0, e_angle_v = 0, e_angle_w = 0;
	static float t_vel = 0.0;

	u_N = (motor_in.V_U_input + motor_in.V_V_input + motor_in.V_W_input) / 3; // Neutral-point voltage
	u_A = motor_in.V_U_input - u_N;           // phase-U voltage (range: 0~4799) (=0~48V)
	u_B = motor_in.V_V_input - u_N;           // phase-V voltage (range: 0~4799) (=0~48V)
	u_C = motor_in.V_W_input - u_N;           // phase-W voltage (range: 0~4799) (=0~48V)

	e_angle_u = park_in.theta_e + ((uint16_t)((svm_phasor.phase - (0xFFFF >> 2))) >> 4);
	if (e_angle_u > 4095) e_angle_u = e_angle_u - 4095;
	if (e_angle_u < 0)    e_angle_u = e_angle_u + 4095;

	e_angle_v = e_angle_u + 2730;
	e_angle_w = e_angle_u + 1365;

	if (e_angle_v > 4095)	{ e_angle_v = e_angle_v - 4095;}
	if (e_angle_v > 4095)	{ e_angle_v = e_angle_v - 4095;}
	if (e_angle_w > 4095)	{ e_angle_w = e_angle_w - 4095;}


	if(motor_out.velocity > 0) t_vel = motor_out.velocity;
	else t_vel = -motor_out.velocity;

	// (STEP2) model-based estimation
	motor_out.I_U_KF = (kf_current_object.kf_A * motor_out.I_U_KF) + (kf_current_object.kf_B * u_A) + (kf_current_object.kf_C * t_vel * sine_signal[e_angle_u]);		// (duty_scaler) * ke * w * sin() --> duty
	motor_out.I_V_KF = (kf_current_object.kf_A * motor_out.I_V_KF) + (kf_current_object.kf_B * u_B) + (kf_current_object.kf_C * t_vel * sine_signal[e_angle_v]);
	motor_out.I_W_KF = (kf_current_object.kf_A * motor_out.I_W_KF) + (kf_current_object.kf_B * u_C) + (kf_current_object.kf_C * t_vel * sine_signal[e_angle_w]);

	// (STEP3) P matrix calculation
	kf_current_object.kf_P = kf_current_object.kf_A * kf_current_object.kf_P * kf_current_object.kf_A + kf_current_object.kf_Q;

	if ((kf_current_object.kf_P + kf_current_object.kf_R) != 0) {
		kf_current_object.kf_K = kf_current_object.kf_P / (kf_current_object.kf_P + kf_current_object.kf_R);
	} else {
		// TODO: Handle division by zero
		kf_current_object.kf_K = 0;
	}

	// (STEP5) Kalman filtering
	motor_out.I_U_KF = motor_out.I_U_KF + kf_current_object.kf_K * (motor_out.I_U - motor_out.I_U_KF);
	motor_out.I_V_KF = motor_out.I_V_KF + kf_current_object.kf_K * (motor_out.I_V - motor_out.I_V_KF);
	motor_out.I_W_KF = motor_out.I_W_KF + kf_current_object.kf_K * (motor_out.I_W - motor_out.I_W_KF);

	// (STEP6) P matrix update
	kf_current_object.kf_P = (1 - kf_current_object.kf_K) * kf_current_object.kf_P;
}

#define CURR_LIMIT (15)
static void Run_FOC_Ctrl(float t_current_in)
{
	int32_t t_iu = 0, t_iv = 0, t_iw = 0;

	float D2A = ADC1_RESOLUTION / CURRENT_SENSOR_RANGE;

	float t_currentRef = motor_setting.commutation_set.cc_dir * t_current_in * D2A;

	if		(t_currentRef >  D2A * motor_setting.contCurr_limit) { t_currentRef =  D2A * motor_setting.contCurr_limit; }
	else if	(t_currentRef < -D2A * motor_setting.contCurr_limit) { t_currentRef = -D2A * motor_setting.contCurr_limit; }

	Run_Kalman_Filter();

	if (motor_setting.low_level_kalman_on == 1) {
		t_iu =  motor_out.I_U_KF;
		t_iv =  motor_out.I_V_KF;
		t_iw =  motor_out.I_W_KF;
	} else {
		t_iu =  motor_out.I_U;
		t_iv =  motor_out.I_V;
		t_iw =  motor_out.I_W;
	}

	clarke_in.Iu = t_iu;
	clarke_in.Iv = t_iv;
	clarke_in.Iw = t_iw;
	ClarkeTransform(&clarke_in, &clarke_out);

	park_in.Ia = clarke_out.Ia;
	park_in.Ib = clarke_out.Ib;

	if(motor_setting.commutation_set.compensation_enable){
		park_in.theta_e = (thetaComp.eTheta>>4);		// sine & cosine table range: 0~4096
	}
	else{
		park_in.theta_e = (motor_out.elec_angle>>4);		// sine & cosine table range: 0~4096
	}

	ParkTransform(&park_in, &park_out);

	Run_PID_Control(&pid_d_axis_ctrl,			   0, park_out.Id, LOW_LEVEL_CONTROL_PERIOD);
	Run_PID_Control(&pid_q_axis_ctrl,	t_currentRef,  park_out.Iq, LOW_LEVEL_CONTROL_PERIOD); // ParkOut.Iq : Total Current

	if(pid_q_axis_ctrl.err_sum > CURR_LIMIT) {pid_q_axis_ctrl.err_sum = CURR_LIMIT;}
	else if(pid_q_axis_ctrl.err_sum < -CURR_LIMIT) {pid_q_axis_ctrl.err_sum = -CURR_LIMIT;}

	inv_park_in.Vd = pid_d_axis_ctrl.control_input;
	inv_park_in.Vq = pid_q_axis_ctrl.control_input;
	inv_park_in.theta_e = park_in.theta_e;
	InvParkInputToPhasor(&inv_park_in, &svm_phasor);

	motor_out.current_act = motor_setting.commutation_set.cc_dir * \
							((float)park_out.Iq * CURRENT_SENSOR_RANGE) / ADC1_RESOLUTION; // - BLDC_Sense.friction_input;


	if(motor_setting.commutation_set.compensation_enable){
		motor_out.actual_elec_angle = thetaComp.eTheta + svm_phasor.phase;
	}
	else{
		motor_out.actual_elec_angle = motor_out.elec_angle + svm_phasor.phase;
	}

	motor_out.I_mag = sqrt(((park_out.Iq * park_out.Iq)>>4) + ((park_out.Id * park_out.Id)>>4));
	motor_out.I_mag = (motor_out.I_mag / (ADC1_RESOLUTION>>2)) * CURRENT_SENSOR_RANGE;

	if (park_out.Iq > 0) {
		current_sign = 1;
	} else {
		current_sign = -1;
	}
	if (IMUAttachCase == IMU_H10_RIGHT_SAG) {
		current_sign = -1 * current_sign;
	}
	motor_actual_current = motor_out.I_mag * current_sign;

	#ifdef _USE_HW_OVER_REV06
	Fault_SW_OCP();
	InvParkTransform(&inv_park_in, &inv_park_out);

	if (safety_timer_flag == 1){
//		inv_park_out.Va = 0;
//		inv_park_out.Vb = 0;

		safety_timer -= 0.05;
		if (safety_timer <= 0){
			safety_timer_flag = 0;
			safety_timer = 1;
		}
	}

	SVM_FOC(&inv_park_out, motor_in.batt_vdc, &motor_in.V_U_input, &motor_in.V_V_input, &motor_in.V_W_input);
	#else
	Check_OverCurrent();
	SVM(svm_phasor.magnitude, motor_out.actual_elec_angle, &motor_in.V_U_input, &motor_in.V_V_input, &motor_in.V_W_input);
	#endif//_USE_HW_OVER_REV06
}


/* ------------------- TRAPEZOIDAL ------------------- */
static void Run_Trape_Ctrl(float t_currentRef)
{
	float D2A = ADC1_RESOLUTION / CURRENT_SENSOR_RANGE;

	if		(t_currentRef >  D2A * motor_setting.contCurr_limit) { t_currentRef =  D2A * motor_setting.contCurr_limit; }
	else if	(t_currentRef < -D2A * motor_setting.contCurr_limit) { t_currentRef = -D2A * motor_setting.contCurr_limit; }

	Run_PID_Control(&pid_trape_ctrl, t_currentRef, motor_out.current_act, LOW_LEVEL_CONTROL_PERIOD);

	/*Float Duty to Uint Duty*/
	if (pid_trape_ctrl.control_input >= 0) {
		trape_ctrl.duty = (uint16_t)pid_trape_ctrl.control_input;
		trape_ctrl.direction = CCW;
	} else if (pid_trape_ctrl.control_input < 0) {
		trape_ctrl.duty = (uint16_t)(-pid_trape_ctrl.control_input);
		trape_ctrl.direction = CW;
	}

	/* Duty Limit */
	if (abs(trape_ctrl.duty) > MAX_DUTY) {
		trape_ctrl.duty = MAX_DUTY;
	} else {
		trape_ctrl.duty = abs(trape_ctrl.duty);
	} 

	if (trape_ctrl.duty < MIN_DUTY) {
		trape_ctrl.duty = 0;
	} else {
		trape_ctrl.duty = trape_ctrl.duty;
	} 
}

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Name(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	motor_properties.name_length = (uint8_t)(((uint8_t*)req->data)[0]);

	memset(&motor_properties.name, 0, sizeof(motor_properties.name));
	memcpy(&motor_properties.name, &(((char*)req->data)[1]), motor_properties.name_length);

	Send_MSG((uint16_t)(GUI_SYNC|MOTOR_NAME_SET), (uint8_t*)0, 1);


    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Pole_Pair(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.pole_pair, req->data, 1);
	Send_MSG((uint16_t)(GUI_SYNC|POLE_PAIR_SET), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Enc_Resolution(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&inc25KhzObj.resolution, req->data, 2);
	memcpy(&inc1KhzObj.resolution, req->data, 2);

    Send_MSG((uint16_t)(GUI_SYNC|INC_ENC_RES_SET_LOW), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Gear_Ratio(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.gear_ratio, req->data, 4);

	Send_MSG((uint16_t)(GUI_SYNC|GEAR_RATIO_SET), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Torque_Constant(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.Kt, req->data, 4);

	Send_MSG((uint16_t)(GUI_SYNC|TORQUE_CST_SET), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Velocity_Constant(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.Ke, req->data, 4);

	Send_MSG((uint16_t)(GUI_SYNC|VOLT_CST_SET), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Peak_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.peakCurr_limit, req->data, 4);

	Send_MSG((uint16_t)(GUI_SYNC|PEAK_CURR_SET), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Continu_Limit(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.contCurr_limit, req->data, 4);

	Send_MSG((uint16_t)(GUI_SYNC|CONT_CURR_SET), (uint8_t*)0, 1);

	res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Max_Velocity(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.max_velocity_rpm, req->data, 4);

	motor_setting.max_velocity_radPsec = motor_setting.max_velocity_rpm*M_PI/30;

	Send_MSG((uint16_t)(GUI_SYNC|MAX_VELO_SET), (uint8_t*)0, 1);

	res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Commutation_Duty(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.commutation_set.max_duty, req->data, 2);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_User_Direction(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.commutation_set.user_desired_dir, (int8_t*)req->data, 1);
	res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Elec_Sys_ID_Mag(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&elec_system_id.level, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Resistance(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.R, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Inductance(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.L, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_BEMF_ID_velocity(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.bemf_id_vel, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_BEMF_ID_Gain_PCTG(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.bemf_id_gain_pctg, req->data, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Curr_Ctrl_BW(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.currCtrl_BW_radPsec, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Inertia(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.J, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Damping_Coef(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.B, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Mech_Model_a(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.a, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Mech_Model_b(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_properties.b, req->data, 4);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Friction_ID_Info(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	float t_tps = 0.0, t_ct = 0.0, t_P_pctg = 0.0, t_I_pctg = 0.0;
	
	memcpy(&advanced_friction_id.velRefEval,		req->data, 4);
	memcpy(&advanced_friction_id.max_vel1, 			((float*)req->data)+1, 4);
	memcpy(&advanced_friction_id.max_vel2, 			((float*)req->data)+2, 4);
	memcpy(&advanced_friction_id.vel_num1, 			((float*)req->data)+3, 4);
	memcpy(&advanced_friction_id.vel_num2, 			((float*)req->data)+4, 4);
	memcpy(&t_tps, 									((float*)req->data)+5, 4);
	memcpy(&t_ct, 									((float*)req->data)+6, 4);
	memcpy(&t_P_pctg,					    		((float*)req->data)+7, 4);
	memcpy(&t_I_pctg,								((float*)req->data)+8, 4);
	memcpy(&advanced_friction_id.gainScalingFactor,	((float*)req->data)+9, 4);

	advanced_friction_id.time_per_vel = LOW_LEVEL_CONTROL_FREQUENCY*t_tps;
	advanced_friction_id.cut_time = LOW_LEVEL_CONTROL_FREQUENCY*t_ct;

	// TODO : Reassign!!
	advanced_friction_id.P_gain = (motor_setting.contCurr_limit)*t_P_pctg/advanced_friction_id.gainScalingFactor;
	advanced_friction_id.I_gain = (motor_setting.contCurr_limit)*t_I_pctg/advanced_friction_id.gainScalingFactor;


//	Set_Velo_Estimator();

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Friction_LUT_Info(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&advanced_friction_id.lut_mdl, 					req->data, 4);
	memcpy(&advanced_friction_id.lut_p1, 		((float*)req->data)+1, 4);
	memcpy(&advanced_friction_id.lut_p2, 		((float*)req->data)+2, 4);
	memcpy(&advanced_friction_id.lut_p3, 		((float*)req->data)+3, 4);
	memcpy(&advanced_friction_id.lut_p4, 		((float*)req->data)+4, 4);
	memcpy(&advanced_friction_id.lut_p5, 		((float*)req->data)+5, 4);
	memcpy(&advanced_friction_id.gain,	        ((float*)req->data)+6, 4);
	memcpy(&advanced_friction_id.lut_d_mu_v, 	((float*)req->data)+7, 4);
	memcpy(&advanced_friction_id.lut_epsilon,	((float*)req->data)+8, 4);

    MS_enum = CAL_FRICTION_LUT;

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Commutation_Sensor(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.sensor_setting.commutation_sensor, req->data, 1);

	Send_MSG((uint16_t)(GUI_SYNC|COMM_SENSOR_SET), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Position_Feedback_Sensor(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.sensor_setting.pos_feedback_sensor, req->data, 1);

	Send_MSG((uint16_t)(GUI_SYNC|POS_FB_SENSOR_SET), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Elec_Angle_Homing_Sensor(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.sensor_setting.e_angle_homing_sensor, req->data, 1);

	Send_MSG((uint16_t)(GUI_SYNC|ELEC_HOMING_SENSOR_SET), (uint8_t*)0, 1);

	res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Mech_Angle_Homing_Sensor(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.sensor_setting.m_angle_homing_sensor, req->data, 1);

    Send_MSG((uint16_t)(GUI_SYNC|MECH_HOMING_SENSOR_SET), (uint8_t*)0, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Sensor_Usage(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_setting.sensor_setting.temperature_sensor_usage,			req->data, 		1);
	memcpy(&motor_setting.sensor_setting.imu_6axis_usage, 		  	((char*)req->data)+1, 	1);
	memcpy(&motor_setting.sensor_setting.imu_3axis_usage, 		  	((char*)req->data)+2, 	1);
	memcpy(&motor_setting.sensor_setting.hall_sensor_usage,			((char*)req->data)+3, 	1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Aux_Input(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&motor_in.auxiliary_input, req->data, 4);
	res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Aging_time(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&AgingCtrl.aging_time, req->data, 2);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Aging_Test_Done(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&AgingCtrl.state, req->data, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

/* ------------------- ROUTINE ------------------- */
/*ROUTINE_ID_BLDC_DIRECTION_SET*/
static int Ent_AbsEnc_Position_Detection()
{
	motor_setting.commutation_set.duration1 = 0.2; // unit (s)
	motor_setting.commutation_set.duration2 = 0.1; // unit (s)
	motor_setting.commutation_set.time_stamp = 0;
	motor_setting.commutation_set.comm_step = 1; // setting the state to 1 intentionally. Do not change it.
	motor_setting.commutation_set.abs_encoder_cnt = 0;
	motor_setting.commutation_set.init_abs_angle_done = 0;
	motor_setting.commutation_set.start              = 1;
	motor_setting.commutation_set.abs_encoder_offset = 0;
	motor_setting.elec_angle_homing.done = 0;
	motor_setting.elec_angle_homing.offset = 0;
	motor_setting.commutation_set.done = 0;

	if (isnanf(motor_setting.commutation_set.max_duty)){
		motor_setting.commutation_set.max_duty = 300;
	}
	else if (motor_setting.commutation_set.max_duty <= 0){
		motor_setting.commutation_set.max_duty = 300;
	}

	if(motor_properties.pole_pair > 0){
		motor_setting.commutation_set.avg_sample_num = motor_properties.pole_pair<<1;
	}
	else{
		motor_setting.commutation_set.avg_sample_num = 1<<1;
	}
	return 0;
}

static int Run_AbsEnc_Position_Detection()
{

	static float var = 0.0, err = 0.0;

	motor_setting.commutation_set.time_stamp++;
	motor_setting.commutation_set.duty = motor_setting.commutation_set.max_duty;

	if      (motor_setting.commutation_set.comm_step == 0)		excitate_phase_UVnWn(motor_setting.commutation_set.duty);
	else if (motor_setting.commutation_set.comm_step == 1)		excitate_phase_UVWn(motor_setting.commutation_set.duty);
	else if (motor_setting.commutation_set.comm_step == 2)		excitate_phase_UnVWn(motor_setting.commutation_set.duty);
	else if (motor_setting.commutation_set.comm_step == 3)		excitate_phase_UnVW(motor_setting.commutation_set.duty);
	else if (motor_setting.commutation_set.comm_step == 4)		excitate_phase_UnVnW(motor_setting.commutation_set.duty);
	else if (motor_setting.commutation_set.comm_step == 5)		excitate_phase_UVnW(motor_setting.commutation_set.duty);

	if(motor_setting.commutation_set.comm_step == 0){
		motor_setting.commutation_set.duration1 = 0.2;
		motor_setting.commutation_set.duration2 = 0.1;
	}
	else{
		motor_setting.commutation_set.duration1 = 0;
		motor_setting.commutation_set.duration2 = 0.01;
	}

	if(motor_setting.commutation_set.time_stamp >= (motor_setting.commutation_set.duration1 + motor_setting.commutation_set.duration2) * LOW_LEVEL_CONTROL_FREQUENCY){
		float t_position = 0;
		uint16_t t_x = 0;
		uint16_t t_abs_encoder_e_angle_offset = 0;
		int32_t	t_abs_encoder_offset_temp = 0;
		int32_t tmp = 0;
		uint8_t num = 0;

		motor_setting.commutation_set.time_stamp = 0;

		if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder1) {
			IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
			if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
				t_position = AbsObj1.posDeg[0];
			}
		}
		else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder2) {
			IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);
			if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
				t_position = AbsObj2.posDeg[0];
			}
		}

		if (t_position < 0){
			t_position = t_position * (-1);
			t_x = (t_position * 182.0444444444445) ;
			t_x = t_x *(-1) ;
		}
		else if (t_position > 0){
			t_x = (t_position * 182.0444444444445);
		}

		if (motor_properties.pole_pair != 0) {
			t_abs_encoder_e_angle_offset = (t_x * motor_properties.pole_pair);
		}
		else {
			// TODO: Handle division by zero
			t_abs_encoder_e_angle_offset = 0;
		}

		if(motor_setting.commutation_set.comm_step == 0){
			// recursive average
			if(motor_setting.commutation_set.abs_encoder_cnt == 0){
				motor_setting.commutation_set.raw[motor_setting.commutation_set.abs_encoder_cnt] = t_abs_encoder_e_angle_offset;
				motor_setting.commutation_set.raw_abs[motor_setting.commutation_set.abs_encoder_cnt] = AbsObj2.rawBit;
			}
			else{
				tmp = t_abs_encoder_e_angle_offset - motor_setting.commutation_set.raw[0];
				if(tmp > 32767){
					motor_setting.commutation_set.raw[motor_setting.commutation_set.abs_encoder_cnt] = t_abs_encoder_e_angle_offset - 65536;
				}
				else if(tmp < -32768){
					motor_setting.commutation_set.raw[motor_setting.commutation_set.abs_encoder_cnt] = t_abs_encoder_e_angle_offset + 65536;
				}
				else{
					motor_setting.commutation_set.raw[motor_setting.commutation_set.abs_encoder_cnt] = t_abs_encoder_e_angle_offset;
				}
				motor_setting.commutation_set.raw_abs[motor_setting.commutation_set.abs_encoder_cnt] = AbsObj2.rawBit;
			}

			motor_setting.commutation_set.abs_encoder_offset += (motor_setting.commutation_set.raw[motor_setting.commutation_set.abs_encoder_cnt]);
			motor_setting.commutation_set.abs_encoder_cnt++;
		}

		if(motor_setting.commutation_set.abs_encoder_cnt < (motor_setting.commutation_set.avg_sample_num>>1)){
			motor_setting.commutation_set.comm_step++;
			if(motor_setting.commutation_set.comm_step > 5) motor_setting.commutation_set.comm_step = 0;
		}
		else if(motor_setting.commutation_set.abs_encoder_cnt >= (motor_setting.commutation_set.avg_sample_num>>1)){
			motor_setting.commutation_set.comm_step--;
			if(motor_setting.commutation_set.comm_step < 0) motor_setting.commutation_set.comm_step = 5;
		}

		if(motor_setting.commutation_set.abs_encoder_cnt == motor_setting.commutation_set.avg_sample_num){

			motor_setting.commutation_set.abs_encoder_offset = motor_setting.commutation_set.abs_encoder_offset / (MOTOR_POLE_PAIR_INT<<1);

			var = 0.0;
			for(int i = 0; i < MOTOR_POLE_PAIR_INT; i++){
				if(i < (MOTOR_POLE_PAIR_INT - 1)){
					motor_setting.commutation_set.raw_elecAngle[i] = (motor_setting.commutation_set.raw[((MOTOR_POLE_PAIR_INT*2)-2)-i] + motor_setting.commutation_set.raw[i])>>1;

					if(motor_setting.commutation_set.raw_elecAngle[i] > 65535){
						motor_setting.commutation_set.raw_elecAngle[i] -= 65536;
					}
					else if(motor_setting.commutation_set.raw_elecAngle[i] < 0){
						motor_setting.commutation_set.raw_elecAngle[i] += 65536;
					}

					motor_setting.commutation_set.raw_mechAngle[i] = (motor_setting.commutation_set.raw_abs[((MOTOR_POLE_PAIR_INT*2)-2)-i] + motor_setting.commutation_set.raw_abs[i])>>1;
					motor_setting.commutation_set.raw_avg[i] = (((((double)motor_setting.commutation_set.raw[((MOTOR_POLE_PAIR_INT*2)-2)-i] + (double)motor_setting.commutation_set.raw[i])/2)) \
							- ((double)motor_setting.commutation_set.abs_encoder_offset))/32768.0*180.0;
					err = motor_setting.commutation_set.raw_avg[i];
					var += err * err;
				}
				else{
					motor_setting.commutation_set.raw_elecAngle[i] = (motor_setting.commutation_set.raw[((MOTOR_POLE_PAIR_INT*2)-1)] + motor_setting.commutation_set.raw[i])>>1;

					if(motor_setting.commutation_set.raw_elecAngle[i] > 65535){
						motor_setting.commutation_set.raw_elecAngle[i] -= 65536;
					}
					else if(motor_setting.commutation_set.raw_elecAngle[i] < 0){
						motor_setting.commutation_set.raw_elecAngle[i] += 65536;
					}

					motor_setting.commutation_set.raw_mechAngle[i] = (motor_setting.commutation_set.raw_abs[((MOTOR_POLE_PAIR_INT*2)-1)] + motor_setting.commutation_set.raw_abs[i])>>1;
					motor_setting.commutation_set.raw_avg[i] = (((((double)motor_setting.commutation_set.raw[((MOTOR_POLE_PAIR_INT*2)-1)] + (double)motor_setting.commutation_set.raw[i])/2)) \
							- ((double)motor_setting.commutation_set.abs_encoder_offset))/32768.0*180.0;
					err = motor_setting.commutation_set.raw_avg[i];
					var += err * err;

					// when err sinusoidal, the avg val is 2/pi of the peak value. if peak val 10, the var must be met within 6.3
				}
			}
			motor_setting.commutation_set.std_dev = (sqrt(var/MOTOR_POLE_PAIR));

			if(motor_setting.commutation_set.abs_encoder_offset > 65535){
				motor_setting.commutation_set.abs_encoder_offset -= 65536;
			}
			else if(motor_setting.commutation_set.abs_encoder_offset < 0){
				motor_setting.commutation_set.abs_encoder_offset += 65536;
			}

			if(motor_setting.commutation_set.std_dev > 4.0){
				motor_setting.commutation_set.compensation_enable = 1;
			}
			else{
				motor_setting.commutation_set.compensation_enable = 0;
			}

			motor_setting.commutation_set.time_stamp = 0;
			motor_setting.commutation_set.comm_step = 0;
			motor_setting.commutation_set.abs_encoder_cnt = 0;
			motor_setting.commutation_set.init_abs_angle_done = 1;
			motor_setting.commutation_set.done = 1;
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
			StateTransition(&msgHdlrTask.stateMachine,       TASK_STATE_STANDBY);
			excitate_phase_UnVnWn();
			motor_setting.commutation_set.start              = 0;
			MS_enum = INIT_ELEC_ANGLE_ID;
		}
	}
	return 0;
}

static int Ext_AbsEnc_Position_Detection()
{
	motor_setting.elec_angle_homing.done = 0;
	motor_setting.elec_angle_homing.offset = 0;

	return 0;
}

static int Ent_Set_Commutation()
{
	motor_setting.commutation_set.duration1 = 0.2; // unit (s)
	motor_setting.commutation_set.duration2 = 0.1; // unit (s)

	if (isnanf(motor_setting.commutation_set.max_duty))
		motor_setting.commutation_set.max_duty = 300;
	else if (motor_setting.commutation_set.max_duty <= 0)
		motor_setting.commutation_set.max_duty = 300;

	motor_setting.commutation_set.duty_gap = motor_setting.commutation_set.max_duty / (motor_setting.commutation_set.duration1 * LOW_LEVEL_CONTROL_FREQUENCY);

	motor_setting.commutation_set.start              = 1;
	motor_setting.commutation_set.done               = 0;
	motor_setting.commutation_set.state              = 0;
//	motor_setting.commutation_set.abs_encoder_offset = 0;
	//motor_setting.commutation_set.abs_encoder_cnt    = 0;
	motor_setting.commutation_set.duty               = 0;
	motor_setting.commutation_set.user_desired_dir   = 0;
	motor_setting.commutation_set.run_cnt            = 0;
	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	motor_setting.commutation_set.initial_cnt        = inc25KhzObj.userCnt;
	return 0;
}

static int Run_Set_Commutation()
{
	int32_t inc_after_commutation = 0;

	/*(STEP1) Start Forced-Commutation */
	if (motor_setting.commutation_set.state == 0) {
		if (motor_setting.commutation_set.run_cnt == 0)	{
			motor_setting.commutation_set.duty = (uint16_t)(motor_setting.commutation_set.duty_gap * (float)motor_setting.commutation_set.time_stamp);
			if (motor_setting.commutation_set.duty > motor_setting.commutation_set.max_duty)
				motor_setting.commutation_set.duty = motor_setting.commutation_set.max_duty;
		} else {
			motor_setting.commutation_set.duty = motor_setting.commutation_set.max_duty;
		}

		if (motor_setting.commutation_set.user_desired_dir == 0) {
			if      (motor_setting.commutation_set.comm_step == 0)		excitate_phase_UVnWn(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 1)		excitate_phase_UVWn(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 2)		excitate_phase_UnVWn(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 3)		excitate_phase_UnVW(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 4)		excitate_phase_UnVnW(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 5)		excitate_phase_UVnW(motor_setting.commutation_set.duty);

			motor_setting.commutation_set.time_stamp++;

			if (motor_setting.commutation_set.time_stamp >= (motor_setting.commutation_set.duration1 + motor_setting.commutation_set.duration2) * LOW_LEVEL_CONTROL_FREQUENCY) {
				if (motor_setting.sensor_setting.hall_sensor_usage == 1) {
					Read_Hall_Sensors(&hallObj);
					motor_setting.commutation_set.hall_sensor_table[motor_setting.commutation_set.comm_step] = hallObj.hall_logic;
				}

/*				if (motor_setting.commutation_set.comm_step == 0) {
					float t_position = 0;
					uint16_t t_x = 0;
					uint16_t t_abs_encoder_e_angle_offset = 0;

					if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder1) {
						IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
						if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
							t_position = AbsObj1.posDeg[0];
						}
					} else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder2) {
						IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);
						if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
							t_position = AbsObj2.posDeg[0];
						}
					}

					if (t_position < 0)
					{
						t_position = t_position * (-1);
						t_x = (t_position * 182.0444444444445) ;
						t_x = t_x *(-1) ;
					}
					else if (t_position > 0)
					{
						t_x = (t_position * 182.0444444444445);
					}

//					if (t_position < 0) t_position = t_position + 360; // 0~360
//					t_x = t_position * 182.0444444444445;              // 360 degree to 65535


					if (motor_properties.pole_pair != 0) {
//						t_abs_encoder_e_angle_offset = t_x - ((motor_properties.pole_pair * t_x)>>16) * (65536 / motor_properties.pole_pair);
						t_abs_encoder_e_angle_offset = (t_x*motor_properties.pole_pair);
					} else {
						// TODO: Handle division by zero
						t_abs_encoder_e_angle_offset = 0;
					}

					// recursive average
					motor_setting.commutation_set.abs_encoder_offset = (t_abs_encoder_e_angle_offset + motor_setting.commutation_set.abs_encoder_offset * motor_setting.commutation_set.abs_encoder_cnt) / (motor_setting.commutation_set.abs_encoder_cnt + 1);
					motor_setting.commutation_set.abs_encoder_cnt++;
				}

*/				motor_setting.commutation_set.time_stamp   = 0;
				motor_setting.commutation_set.duty         = 0;
				motor_setting.commutation_set.comm_step    = (motor_setting.commutation_set.comm_step + 1) % 6;
				motor_setting.commutation_set.run_cnt++;
			}
		} else {
			motor_setting.commutation_set.time_stamp = 0;
			motor_setting.commutation_set.comm_step = 0;
			motor_setting.commutation_set.run_cnt   = 0;
//			motor_setting.commutation_set.abs_encoder_cnt = 0;
			motor_setting.commutation_set.state = 1;
			excitate_phase_UnVnWn(); // 0-voltage
		}
	} else if (motor_setting.commutation_set.state == 1) { 	/* (STEP2) Decision Rotation Direction */
		if (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder) {
			IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
			inc_after_commutation = inc25KhzObj.userCnt - motor_setting.commutation_set.initial_cnt;
		} else {
			return -1; // (error): please set proper commutation sensor
		}

		/* Set commutation direction */
		if(inc_after_commutation > 0) {motor_setting.commutation_set.ea_dir = +1; }
		else                          {motor_setting.commutation_set.ea_dir = -1; }

		/* Set current control direction */
		if      (motor_setting.commutation_set.user_desired_dir == 1) {motor_setting.commutation_set.cc_dir = +1; motor_setting.commutation_set.hall_sensor_dir = +1; hallObj.direction_set = +1; }
		else if (motor_setting.commutation_set.user_desired_dir == 2) {motor_setting.commutation_set.cc_dir = -1; motor_setting.commutation_set.hall_sensor_dir = -1; hallObj.direction_set = -1;}

		/* Set mechanical angle direction */
		if     ((inc_after_commutation > 0) && (motor_setting.commutation_set.user_desired_dir == 1))
			{motor_setting.commutation_set.ma_dir = +1;}
		else if((inc_after_commutation > 0) && (motor_setting.commutation_set.user_desired_dir == 2))
			{motor_setting.commutation_set.ma_dir = -1;}
		else if((inc_after_commutation < 0) && (motor_setting.commutation_set.user_desired_dir == 1))
			{motor_setting.commutation_set.ma_dir = -1;}
		else if((inc_after_commutation < 0) && (motor_setting.commutation_set.user_desired_dir == 2))
			{motor_setting.commutation_set.ma_dir = +1;}

		motor_setting.commutation_set.state = 2;
	}

	/* (STEP3) Check Hall sensor table */
	if (motor_setting.commutation_set.state == 2) {
		if (motor_setting.sensor_setting.hall_sensor_usage == 1) {
			float t_sum = 0.0;
			float t_mul = 1.0;
			for (int i = 0; i < 6; i++) {
				t_sum = t_sum + motor_setting.commutation_set.hall_sensor_table[i];
				t_mul = t_mul * motor_setting.commutation_set.hall_sensor_table[i];
			}

			if ((t_sum != 21) || (t_mul != 720)) {
				motor_setting.commutation_set.state = 0;
				motor_setting.commutation_set.done = 0;
				motor_setting.commutation_set.user_desired_dir = 0;
				StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
				StateTransition(&msgHdlrTask.stateMachine,       TASK_STATE_STANDBY);
				low_level_ctrl_task.errCode = ERROR_HALL_TABLE_GENERATION_FAILED;
				return -1; // (error): please set proper commutation sensor
			}
			else
				motor_setting.commutation_set.state = 3;
		} else
			motor_setting.commutation_set.state = 3;
	}

	/* (STEP4) Finish Driver Commutation Setting */
	if (motor_setting.commutation_set.state == 3) {
		motor_setting.commutation_set.state = 0;
		motor_setting.commutation_set.start = 0;
		motor_setting.commutation_set.done = 1;
		motor_setting.commutation_set.user_desired_dir = 0;
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		StateTransition(&msgHdlrTask.stateMachine,       TASK_STATE_STANDBY);
		return 0;
	}

	return 1;

}

/*ROUTINE_ID_BLDC_ELECTRICAL_SYS_ID*/
static int Ent_Electrical_System_ID()
{
	static int init_index = 0;

	if (init_index == 0) {
		elec_system_id.flag = 0;
		elec_system_id.cnt = 0;
	    init_index++;
	}

	elec_system_id.input = 0;
	elec_system_id.signal = 0;
	elec_system_id.f_signal = 0;
	elec_system_id.output = 0.0;	//in amphere
    elec_system_id.flag = 0;
    elec_system_id.cnt = 0;

    memset(shared_array_buffer.buff1, 0, sizeof(shared_array_buffer.buff1));
    memset(shared_array_buffer.buff2, 0, sizeof(shared_array_buffer.buff2));

    motor_setting.low_level_kalman_on = 0;

	return 0;
}

static int Run_Electrical_System_ID()
{
	if (elec_system_id.flag == 0) {

		/* Phase Align */
		if (elec_system_id.cnt < 4000) {

			excitate_phase_UVn(500);

		} else if ((4000 <= elec_system_id.cnt) && (elec_system_id.cnt <= 8000)) {

			excitate_phase_UnVnWn();

			/* System ID by Chirp signal */
		} else if (8000 < elec_system_id.cnt) {

			/* chirp */
			if (elec_system_id.cnt >= (8000+CHIRP_ARRAY_SIZE)) {
				elec_system_id.flag = 1;
				excitate_phase_UnVnWn();
			} else {

				elec_system_id.f_signal = elec_system_id.level*chirp_freq[elec_system_id.cnt-8001] * VBUS2DUTY_RATIO;	// 100~500hz		mag: 0~4800 (= 0~48V)

				if (elec_system_id.f_signal >= 0) {
					excitate_phase_UVn((uint16_t)elec_system_id.f_signal);
				} else {
					excitate_phase_UnV((uint16_t)(-elec_system_id.f_signal));
				}

				elec_system_id.output = (motor_out.I_U - motor_out.I_V);					// mag: -32768 ~ 32767

				shared_array_buffer.buff1[elec_system_id.cnt - 8001] = elec_system_id.f_signal / 2;
				shared_array_buffer.buff2[elec_system_id.cnt - 8001] = elec_system_id.output / 2;
			}
		}

		elec_system_id.cnt++;
	}

	if (elec_system_id.flag) {
	    StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
	    MS_enum = ELECTRICAL_SYSTEM_ID;
		return 0;
	}

	return 0;

}

/*ROUTINE_ID_BLDC_ELECTRICAL_BEMF*/
static int Ent_BEMF_ID()
{
	motor_setting.currCtrl_BW_radPsec = 1000 * 2 * M_PI;
	motor_setting.bemf_id_gain = (motor_setting.contCurr_limit / 10) * motor_setting.bemf_id_gain_pctg / VBUS2DUTY_RATIO;

	return 0;
}
static int Run_BEMF_ID()
{
	static int t_flag = 0;
	static float t_error = 0.0;

	t_error = motor_setting.bemf_id_vel - motor_out.velocity;

	if (low_level_ctrl_loop_cnt == 0) {

		if (motor_setting.low_level_kalman_on == 1) {
			motor_setting.low_level_kalman_on = 0;
			t_flag = 1;
		} else {
			t_flag = 0;
		}
	}

	if (low_level_ctrl_loop_cnt <= 25000) {	// 1 sec

		motor_in.low_id_process_input = motor_setting.bemf_id_gain * t_error;
		rms_err = 0;
		mean_vel = 0;

		kf_current_object.kf_C = 0;
		kf_current_object.kf_Q = 0;

	} else if((low_level_ctrl_loop_cnt > 25000) && (low_level_ctrl_loop_cnt < 100000)) {
		N = low_level_ctrl_loop_cnt - 25000;

		motor_in.low_id_process_input = motor_setting.bemf_id_gain * t_error;

        rms_err = sqrt(((motor_out.I_U_KF - motor_out.I_U) * (motor_out.I_U_KF - motor_out.I_U) + (rms_err * rms_err) * (N-1)) / N);
        mean_vel = (motor_out.velocity + mean_vel * (N-1)) / N;

	} else {
		if (mean_vel != 0) {
			kf_current_object.kf_C = rms_err * 1.4142 * (1 - kf_current_object.kf_A) / mean_vel;
			motor_in.low_id_process_input = 0;
			kf_current_object.kf_Q = 1;

			if (t_flag)		motor_setting.low_level_kalman_on = 1;

			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
			MS_enum = BEMF_ID;
			return 0;
		} else {
			// TODO: Handle division by zero
			return -1;
		}
	}

	return 0;
}
static int Ext_BEMF_ID()
{
	// Added
	if (low_level_ctrl_task.errCode == ERROR_PHASE_OVER_CURRENT) {
		static uint8_t temp_arr = 0;
		static uint16_t identifier = 0;

	    identifier = GUI_SYNC|BEMF_ID_OVER_CURRENT;
		Send_MSG(identifier, &temp_arr, 0);
	}

	RT_Broken = 0;
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_BOOT_RED_Pin, IOIF_GPIO_PIN_RESET);
#endif

	// TODO : Angel Suit SAM Indicating!!

    motor_setting.low_level_kalman_on = 1;
	return 0;
}

/*ROUTINE_ID_BLDC_CURRENT_CTRL*/
static int Ent_Current_Control()
{
	motor_out.I_U_KF = 0;
	motor_out.I_V_KF = 0;
	motor_out.I_W_KF = 0;
	motor_out.velocity = 0;

	Init_PID(&pid_d_axis_ctrl, motor_properties.L * 0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 1.75 * 0.26 * (motor_setting.currCtrl_BW_radPsec), 0);
	Init_PID(&pid_q_axis_ctrl, motor_properties.L * 0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 1.75 * 0.26 * (motor_setting.currCtrl_BW_radPsec), 0);
	Init_PID(&pid_trape_ctrl,  motor_properties.L * 0.26 * 1.75 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 1.75 * 0.26 * (motor_setting.currCtrl_BW_radPsec), 0);

	motor_in.auxiliary_input = 0;
	motor_in.f_vector_input = 0;
	motor_in.low_id_process_input = 0;
	motor_in.friction_input = 0;
	motor_in.mid_id_process_input = 0;
	motor_in.irc_input = 0;
	motor_in.mid_ctrl_input = 0;
	motor_in.analysis_input = 0;

	OtpDerateCtrl.OtplimitRate = (float)OtpDerateCtrl.DerateRamp/OTP_DERATE_NORMAL_RATE;
	OtpDerateCtrl.setTime = 0;
	OtpDerateCtrl.clrTime = 0;
	OtpDerateCtrl.state = NORMAL_STATE;
	OtpDerateCtrl.rampCnt = 0;
	OtpDerateCtrl.DerateTarget = OTP_DERATE_NORMAL_RATE;
	OtpDerateCtrl.DerateRamp = OTP_DERATE_NORMAL_RATE;

	Init_Current_Ctrl();

	return 0;
}


static int Run_Current_Control()
{
	motor_in.total_current_input = motor_in.auxiliary_input 		+ \
			                       motor_in.f_vector_input          + \
								   motor_in.low_id_process_input 	+ \
								   motor_in.friction_input 			+ \
								   motor_in.irc_input 				+ \
								   motor_in.mid_ctrl_input 			+ \
								   motor_in.analysis_input;

	// Limit function must be located //
	if((OtpDerateCtrl.OtplimitRate <= 0) || (OtpDerateCtrl.OtplimitRate > 1.0)) OtpDerateCtrl.OtplimitRate = 1.0;
	StallDetect.curr_tgt = motor_in.total_current_input;
	motor_in.total_current_input = motor_in.total_current_input * OtpDerateCtrl.OtplimitRate;

	MaxAngleLimit.curr_ref = motor_in.total_current_input;
	if((low_level_ctrl_task.routine.id[0] == ROUTINE_ID_LOWLEVEL_ELECTRICAL_BEMF_ID) ||	\
			(low_level_ctrl_task.routine.id[0] == ROUTINE_ID_LOWLEVEL_ADV_FRICTION_ID_VEL_CTRL_EVAL) || \
			(low_level_ctrl_task.routine.id[0] == ROUTINE_ID_LOWLEVEL_ADVANCED_FRICTION_ID) || \
			(mid_level_ctrl_task.routine.id[0] == ROUTINE_ID_MIDLEVEL_SYS_ID_SBS) || \
			(mid_level_ctrl_task.routine.id[0] == ROUTINE_ID_MIDLEVEL_CURRENT_SINE_REF)||\
			(mid_level_ctrl_task.routine.id[0] == ROUTINE_ID_MIDLEVEL_CHECK_TORQUE_ACCURACY)||\
			(mid_level_ctrl_task.routine.id[0] == ROUTINE_ID_MIDLEVEL_CHECK_TORQUE_UNIFORMITY)||\
			(mid_level_ctrl_task.routine.id[0] == ROUTINE_ID_MIDLEVEL_CHECK_ACTUATOR_BACKDRIVABILITY)||\
			(mid_level_ctrl_task.routine.id[0] == ROUTINE_ID_MIDLEVEL_CHECK_ENCODER_LINEARITY)||\
			(AbsObj1.offset == 0 && AbsObj1.raw_offset == 0)||\
			(joint_limit_sw))
	{
		MaxAngleLimit.comp = 0;
	}
	else
	{
		MaxAngleLimitCtrl(&MaxAngleLimit, mid_level_state.velocity_raw);
		motor_in.total_current_input += MaxAngleLimit.comp;
	}

	//TODO
	if       (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder){


		if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall)
		{
			if (motor_setting.elec_angle_homing.done == 1)			Cal_Elec_Angle();
			else                                    				Cal_Elec_Angle_with_Hall();
		}
		else														Cal_Elec_Angle();

		if (safety_timer_flag == 1){
			motor_in.total_current_input = 0;
		}

		Run_FOC_Ctrl(motor_in.total_current_input);
		Set_Stall_State(&StallDetect);

	} else if(motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Hall) {

		Get_3Phase_Current();
		if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);
		Cal_Trape_Total_Current();
		Run_Trape_Ctrl(motor_in.total_current_input);
	}

	return 0;
}


/*ROUTINE_ID_BLDC_CURRENT_CTRL_BANDWIDTH_CHECK*/
static int Ent_Check_Current_Controller_Bandwidth()
{
	memset(shared_array_buffer.buff1, 0, sizeof(shared_array_buffer.buff1));
	memset(shared_array_buffer.buff2, 0, sizeof(shared_array_buffer.buff2));
	return 0;
}
static int Run_Check_Current_Controller_Bandwidth()
{
	if (low_level_ctrl_loop_cnt <= BW_CHECK_ARRAY_SIZE) {

		motor_in.low_id_process_input = current_bandwidth[low_level_ctrl_loop_cnt];
		shared_array_buffer.buff1[low_level_ctrl_loop_cnt] = motor_out.current_act;
		shared_array_buffer.buff2[low_level_ctrl_loop_cnt] = motor_in.low_id_process_input;
	} else {
		motor_in.low_id_process_input = 0;
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
	    MS_enum = CURRENT_BANDWIDTH_CHECK;
	}

	return 0;
}
static int Ext_Check_Current_Controller_Bandwidth()
{
	RT_Broken = 0;
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_BOOT_RED_Pin, IOIF_GPIO_PIN_RESET);
#endif
	// TODO : Angel Suit Indicating!!
	return 0;
}

static int Ent_Advanced_Friction_ID()
{
	if((advanced_friction_id.time_per_vel == 0) ||
	   (advanced_friction_id.max_vel1 == 0) ||
	   (advanced_friction_id.max_vel2 == 0) ||
	   (advanced_friction_id.vel_num1 == 0) ||
	   (advanced_friction_id.vel_num2 == 0) ||
	   (advanced_friction_id.P_gain == 0)) {

		// TODO: Handle division by zero
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);

		return -1;
	} else {
		/* Entry */
		advanced_friction_id.gap1 = advanced_friction_id.max_vel1 / advanced_friction_id.vel_num1;
		advanced_friction_id.gap2 = (advanced_friction_id.max_vel2 - advanced_friction_id.max_vel1) / advanced_friction_id.vel_num2;

		memset(shared_array_buffer.buff1, 0, sizeof(shared_array_buffer.buff1));
		memset(shared_array_buffer.buff2, 0, sizeof(shared_array_buffer.buff2));

		advanced_friction_id.state = 0;
		advanced_friction_id.friction_cnt = 1;
		advanced_friction_id.friction_index = 0;

		advanced_friction_id.section_time[0] = advanced_friction_id.time_per_vel * advanced_friction_id.vel_num1;
		advanced_friction_id.section_time[1] = advanced_friction_id.time_per_vel * (advanced_friction_id.vel_num1 + advanced_friction_id.vel_num2);
		advanced_friction_id.section_time[2] = advanced_friction_id.time_per_vel * (advanced_friction_id.vel_num1 + 2 * advanced_friction_id.vel_num2);
		advanced_friction_id.section_time[3] = advanced_friction_id.time_per_vel * (2 * advanced_friction_id.vel_num1 + 2 * advanced_friction_id.vel_num2);
		advanced_friction_id.section_time[4] = advanced_friction_id.time_per_vel * (3 * advanced_friction_id.vel_num1 + 2 * advanced_friction_id.vel_num2);
		advanced_friction_id.section_time[5] = advanced_friction_id.time_per_vel * (3 * advanced_friction_id.vel_num1 + 3 * advanced_friction_id.vel_num2);
		advanced_friction_id.section_time[6] = advanced_friction_id.time_per_vel * (3 * advanced_friction_id.vel_num1 + 4 * advanced_friction_id.vel_num2);
		advanced_friction_id.section_time[7] = advanced_friction_id.time_per_vel * (4 * advanced_friction_id.vel_num1 + 4 * advanced_friction_id.vel_num2);
		advanced_friction_id.e_sum           = 0;
	}
	
	return 0;
}

static int Run_Advanced_Friction_ID()
{
	static uint8_t save = 0;
	advanced_friction_id.state = 1;
	int i = 0;

	if ((low_level_ctrl_loop_cnt % advanced_friction_id.time_per_vel) == 0) { // @ every 3 sec
		advanced_friction_id.friction_cnt = 1;
		if (low_level_ctrl_loop_cnt == 0) {   // Init
			advanced_friction_id.friction_index = 0;
		} else {
			if (save) advanced_friction_id.friction_index++;
		}
	}

	if (advanced_friction_id.time_per_vel != 0) {
 		i = (low_level_ctrl_loop_cnt / advanced_friction_id.time_per_vel) + 1;
	} else {
		// TODO: Handle division by zero
		i = 0;
		return -1;
	}

	if (low_level_ctrl_loop_cnt < advanced_friction_id.section_time[0]) {
		advanced_friction_id.friction_ref = i*advanced_friction_id.gap1;
		save = 1;

	} else if (low_level_ctrl_loop_cnt < advanced_friction_id.section_time[1]) {
		advanced_friction_id.friction_ref = advanced_friction_id.max_vel1 + (i - advanced_friction_id.vel_num1) * advanced_friction_id.gap2;
		save = 1;

	} else if (low_level_ctrl_loop_cnt < advanced_friction_id.section_time[2]) {
		advanced_friction_id.friction_ref = advanced_friction_id.max_vel2 - (i - advanced_friction_id.vel_num1 - advanced_friction_id.vel_num2) * advanced_friction_id.gap2;
		save = 1;

	} else if (low_level_ctrl_loop_cnt < advanced_friction_id.section_time[3]) {
		advanced_friction_id.friction_ref = advanced_friction_id.max_vel1 - (i - advanced_friction_id.vel_num1 - 2 * advanced_friction_id.vel_num2) * advanced_friction_id.gap1;
		save = 1;

	} else if (low_level_ctrl_loop_cnt < advanced_friction_id.section_time[4]) {
		advanced_friction_id.friction_ref = -(i - 2*advanced_friction_id.vel_num1 - 2 * advanced_friction_id.vel_num2) * advanced_friction_id.gap1;
		save = 1;

	} else if (low_level_ctrl_loop_cnt < advanced_friction_id.section_time[5]) {
		advanced_friction_id.friction_ref = -advanced_friction_id.max_vel1 - (i - 3 * advanced_friction_id.vel_num1 - 2 * advanced_friction_id.vel_num2) * advanced_friction_id.gap2;
		save = 1;

	} else if (low_level_ctrl_loop_cnt < advanced_friction_id.section_time[6]) {
		advanced_friction_id.friction_ref = -advanced_friction_id.max_vel2 + (i - 3 * advanced_friction_id.vel_num1 - 3 * advanced_friction_id.vel_num2) * advanced_friction_id.gap2;
		save = 1;

	} else if (low_level_ctrl_loop_cnt < advanced_friction_id.section_time[7]) {
		advanced_friction_id.friction_ref = -advanced_friction_id.max_vel1 + (i - 3 * advanced_friction_id.vel_num1 - 4 * advanced_friction_id.vel_num2) * advanced_friction_id.gap1;
		save = 1;
	} else {
		advanced_friction_id.friction_ref = 0;
		motor_in.low_id_process_input  = 0;
		MS_enum = ADV_FRICTION_ID;
		StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
	}

	if (save == 1) {
		advanced_friction_id.e        = advanced_friction_id.friction_ref - motor_out.velocity;
		advanced_friction_id.e_sum    = advanced_friction_id.e_sum + advanced_friction_id.e * LOW_LEVEL_CONTROL_PERIOD;
		motor_in.low_id_process_input = advanced_friction_id.P_gain*advanced_friction_id.e + advanced_friction_id.I_gain * advanced_friction_id.e_sum;
	}

	/* Moving Averaging */
	if ((advanced_friction_id.friction_cnt >= advanced_friction_id.cut_time) && (save == 1)) {
		if ((advanced_friction_id.friction_cnt - (advanced_friction_id.cut_time-1)) != 0) {
			shared_array_buffer.buff1[advanced_friction_id.friction_index] = ((advanced_friction_id.friction_cnt - advanced_friction_id.cut_time) * \
																				shared_array_buffer.buff1[advanced_friction_id.friction_index] + \
																				motor_out.velocity) / \
																				(advanced_friction_id.friction_cnt - (advanced_friction_id.cut_time - 1));
			shared_array_buffer.buff2[advanced_friction_id.friction_index] = ((advanced_friction_id.friction_cnt - advanced_friction_id.cut_time) * \
																					shared_array_buffer.buff2[advanced_friction_id.friction_index] + \
																					motor_in.low_id_process_input) / \
																					(advanced_friction_id.friction_cnt - (advanced_friction_id.cut_time - 1));
		} else {
			// TODO: Handle division by zero
			shared_array_buffer.buff1[advanced_friction_id.friction_index] = 0;
			shared_array_buffer.buff2[advanced_friction_id.friction_index] = 0;
			return -1;
		}
	}
	advanced_friction_id.friction_cnt++;

	return 0;
}

static int Ext_Advanced_Friction_ID()
{
	RT_Broken = 0;
	advanced_friction_id.state = 0;
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_RESET);

	return 0;
}

static int Ent_Evaluation_Friction_ID_Velocity_Ctrl()
{
	advanced_friction_id.e        = 0;
	advanced_friction_id.e_sum    = 0;
	return 0;
}

static int Run_Evaluation_Friction_ID_Velocity_Ctrl()
{
	advanced_friction_id.e        = advanced_friction_id.velRefEval - motor_out.velocity;
	advanced_friction_id.e_sum    = advanced_friction_id.e_sum + advanced_friction_id.e * LOW_LEVEL_CONTROL_PERIOD;
	motor_in.low_id_process_input = advanced_friction_id.P_gain*advanced_friction_id.e + advanced_friction_id.I_gain * advanced_friction_id.e_sum;
	advanced_friction_id.state = 1;
	return 0;
}

static int Ent_Adv_Friction_Compensation()
{
	return 0;
}

static int Run_Adv_Friction_Compensation()
{
	int16_t index = 0;

	if (advanced_friction_id.scaling_factor != 0) {
		index = (int16_t)(4096 + motor_out.velocity / advanced_friction_id.scaling_factor);
	} else {
		// TODO: Handle division by zero
		motor_in.friction_input = 0;
		return -1;
	}

	if (index < 0)         index = 0;
	else if (index > 8192) index = 8192;

	motor_in.friction_input = advanced_friction_id.adv_friction_compensator_LUT[index];

	return 0;
}

static int Ent_Adv_Friction_Compensation_FF()
{
	return 0;
}

static int Run_Adv_Friction_Compensation_FF()
{
	int16_t index = 0;

	if (advanced_friction_id.scaling_factor != 0) {
		index = (int16_t)(4096 + advanced_friction_id.friction_ref / advanced_friction_id.scaling_factor);
	} else {
		// TODO: Handle division by zero
		motor_in.friction_input = 0;
		return -1;
	}

	if (index < 0)         index = 0;
	else if (index > 8192) index = 8192;

	motor_in.friction_input = advanced_friction_id.adv_friction_compensator_LUT[index];

	return 0;
}



/* Electrical Angle Homing Algorithm*/
static void Run_Forced_Homing()
{
	if (motor_setting.elec_angle_homing.forced_homing_cnt == 0) {
		if (motor_setting.elec_angle_homing.forced_homing_duration != 0) {
			motor_setting.elec_angle_homing.forced_homing_dy = (float)motor_setting.elec_angle_homing.forced_homing_duty / (float)motor_setting.elec_angle_homing.forced_homing_duration;
			GateDriver_ONOFF(ENABLE);
			htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
		} else {
			// TODO: Handle division by zero
			return;
		}
	} else if (motor_setting.elec_angle_homing.forced_homing_cnt <= motor_setting.elec_angle_homing.forced_homing_duration) {
		motor_setting.elec_angle_homing.forced_homing_duty = motor_setting.elec_angle_homing.forced_homing_duty + motor_setting.elec_angle_homing.forced_homing_dy;

		if (motor_setting.elec_angle_homing.forced_homing_duty > 1000) motor_setting.elec_angle_homing.forced_homing_duty = 1000;

		excitate_phase_UVnWn(motor_setting.elec_angle_homing.forced_homing_duty);
	} else if (motor_setting.elec_angle_homing.forced_homing_cnt == motor_setting.elec_angle_homing.forced_homing_duration + 5000) {
		IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
		Cal_Elec_Angle();
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle;
		excitate_phase_UnVnWn();
	} else if (motor_setting.elec_angle_homing.forced_homing_cnt == motor_setting.elec_angle_homing.forced_homing_duration + 15000) {
		motor_setting.elec_angle_homing.forced_homing_cnt = 0;
		motor_setting.elec_angle_homing.done = 1;
	}
	motor_setting.elec_angle_homing.forced_homing_cnt++;
}

static void Run_Hall_Homing()
{
	if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);

	uint8_t t_hall_logic = 0, t_hall_logic_f = 0;
	uint8_t *t_hall_sensor_table = NULL;
	t_hall_logic        = hallObj.hall_logic;
	t_hall_logic_f      = hallObj.hall_logic_f;
	t_hall_sensor_table = motor_setting.commutation_set.hall_sensor_table;

	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();

	if ((t_hall_logic_f == t_hall_sensor_table[0]) && (t_hall_logic == t_hall_sensor_table[1]))	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 5461;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[1]) && (t_hall_logic == t_hall_sensor_table[0])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 5461;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[1]) && (t_hall_logic == t_hall_sensor_table[2])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 16384;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[2]) && (t_hall_logic == t_hall_sensor_table[1])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 16384;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[2]) && (t_hall_logic == t_hall_sensor_table[3])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 27307;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[3]) && (t_hall_logic == t_hall_sensor_table[2])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 27307;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[3]) && (t_hall_logic == t_hall_sensor_table[4])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 38229;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[4]) && (t_hall_logic == t_hall_sensor_table[3])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 38229;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[4]) && (t_hall_logic == t_hall_sensor_table[5])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 49152;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[5]) && (t_hall_logic == t_hall_sensor_table[4])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 49152;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[5]) && (t_hall_logic == t_hall_sensor_table[0])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 60075;
		motor_setting.elec_angle_homing.done = 1;
	} else if ((t_hall_logic_f == t_hall_sensor_table[0]) && (t_hall_logic == t_hall_sensor_table[5])) {
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 60075;
		motor_setting.elec_angle_homing.done = 1;
	} else {
		low_level_ctrl_task.errCode = ERROR_ELEC_ANGLE_HOMING_FAILED;
	}
}

static void Run_Abs_Encoder1_Homing()
{
	float    t_abs_encoder_m_angle_deg = 0;
	uint16_t t_abs_encoder_m_angle_cnt = 0;
	uint16_t t_m_angle_offset = 0;
	uint16_t t_abs_e_angle = 0;

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);

	if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
		t_abs_encoder_m_angle_deg = AbsObj1.posDeg[0];
	} else {

		low_level_ctrl_task.errCode = ERROR_ELEC_ANGLE_HOMING_NO_ABS_ENCODER;
	}

	if (t_abs_encoder_m_angle_deg < 0) t_abs_encoder_m_angle_deg = t_abs_encoder_m_angle_deg + 360;
	t_abs_encoder_m_angle_cnt = t_abs_encoder_m_angle_deg * 182.0444444444445;

	int8_t t_sign = 0;
	t_sign = motor_setting.commutation_set.ea_dir * motor_setting.commutation_set.ma_dir;
	t_m_angle_offset = t_sign * (t_abs_encoder_m_angle_cnt - motor_setting.commutation_set.abs_encoder_offset);
	// get (remainder of (t_m_angle_offset) devide to (65536/P)) * P
	if (motor_properties.pole_pair != 0) {
		t_abs_e_angle = (t_m_angle_offset - ((motor_properties.pole_pair*t_m_angle_offset)>>16) * (65536 / motor_properties.pole_pair)) * motor_properties.pole_pair;
	} else {
		// TODO: Handle division by zero
		t_abs_e_angle = 0;
	}

	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();

	motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + t_abs_e_angle;
	motor_setting.elec_angle_homing.done = 1;
}

static void Run_Abs_Encoder2_Homing()
{
	float    t_abs_encoder_m_angle_deg = 0;
	uint16_t t_abs_encoder_m_angle_cnt = 0;
	uint16_t t_m_angle_offset;
	uint16_t t_abs_e_angle;

	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);
	Get_rotor_pole_number(&AbsObj2, &thetaComp);

	if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
		t_abs_encoder_m_angle_deg = AbsObj2.posDeg[0];
	} else {
		low_level_ctrl_task.errCode = ERROR_ELEC_ANGLE_HOMING_NO_ABS_ENCODER;
	}


	if (t_abs_encoder_m_angle_deg < 0)
	{
		t_abs_encoder_m_angle_deg = t_abs_encoder_m_angle_deg * (-1);
		t_abs_encoder_m_angle_cnt = (t_abs_encoder_m_angle_deg * 182.0444444444445) ;
		t_abs_encoder_m_angle_cnt = t_abs_encoder_m_angle_cnt *(-1) ;
	}
	else if (t_abs_encoder_m_angle_deg > 0)
	{
		t_abs_encoder_m_angle_cnt = (t_abs_encoder_m_angle_deg * 182.0444444444445);
	}


	int8_t t_sign = 0;
	t_sign = motor_setting.commutation_set.ea_dir * motor_setting.commutation_set.ma_dir;
//	t_sign = motor_setting.commutation_set.ea_dir;
//	t_m_angle_offset = t_sign * (t_abs_encoder_m_angle_cnt - motor_setting.commutation_set.abs_encoder_offset);
	t_m_angle_offset = t_sign * (t_abs_encoder_m_angle_cnt);
	// get (remainder of (t_m_angle_offset) devide to (65536/P)) * P

	if (motor_properties.pole_pair != 0) {

		if(motor_setting.commutation_set.compensation_enable){
			t_abs_e_angle = t_m_angle_offset * motor_properties.pole_pair - t_sign * thetaComp.incTheta[thetaComp.pp_region];
		}
		else{
			t_abs_e_angle = t_m_angle_offset * motor_properties.pole_pair - t_sign * motor_setting.commutation_set.abs_encoder_offset;
		}
	} else {
		t_abs_e_angle =0;
	}


	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();

	motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + t_abs_e_angle;
	motor_setting.elec_angle_homing.done = 1;

	/*
	if (t_abs_encoder_m_angle < 0) t_abs_encoder_m_angle = t_abs_encoder_m_angle + 360;
	t_abs_encoder_e_angle = t_abs_encoder_m_angle * (65536/360) * motor_properties.pole_pair;

	t_offset = motor_setting.commutation_set.ea_dir * (t_abs_encoder_e_angle - motor_setting.commutation_set.abs_encoder_offset);

	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();

	motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + t_offset;
	motor_setting.elec_angle_homing.done = 1;
	*/
}

static int Ent_Elec_Angle_Homing()
{
	motor_setting.elec_angle_homing.forced_homing_duration = 25000;

	float t_peak_voltage = 0.0;
	if (!isnan(motor_properties.R) & !isnan(motor_setting.peakCurr_limit)) // if R, peakCurr is not nan
		t_peak_voltage = motor_setting.peakCurr_limit * motor_properties.R;
	else                                                                   // if R or peakCurr is nan, default duty = 5V
		t_peak_voltage = 5;

	motor_setting.elec_angle_homing.forced_homing_duty     = t_peak_voltage * VBUS2DUTY_RATIO;
	motor_setting.elec_angle_homing.forced_homing_cnt      = 0;

	if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);

	return 0;
}

static int Run_Elec_Angle_Homing()
{
	// if electrical angle homing is not done
	if (motor_setting.elec_angle_homing.done == 0) {
		if (motor_setting.commutation_set.done == 0) {
			// error_handler: You must do commutation_setting on GUI
		} else {
			if      (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Forced)          Run_Forced_Homing();
			else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall)            Run_Hall_Homing();
			else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder1)    Run_Abs_Encoder1_Homing();
			else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder2)    Run_Abs_Encoder2_Homing();

			if (motor_setting.elec_angle_homing.done == 1)		BSP_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, BSP_GPIO_PIN_RESET);
		}
	}

	return 0;
}

#ifdef _USE_HW_OVER_REV06
/* IF Control for aging actuator */
static int Ent_IF_Spd_Control()
{
	float mech_wr_target_step = 0.0;
	float elec_wr_target_step = 0.0;

	AgingCtrl.clk_timer = 0;
	AgingCtrl.sec_timer = 0;
	AgingCtrl.elec_angle = 0;
	AgingCtrl.state = 0;
	//AgingCtrl.current_add = 0.0;

	AgingCtrl.position_old = 0;
	AgingCtrl.position = 0;
	AgingCtrl.speed = 0.0;
	if(motor_setting.currCtrl_BW_radPsec == 0){
		pid_d_axis_ctrl.Kp = IF_KP;
		pid_d_axis_ctrl.Ki = IF_KI;
		pid_q_axis_ctrl.Kp = IF_KP;
		pid_q_axis_ctrl.Ki = IF_KI;
	}

	if(AgingCtrl.aging_time == 0){
		AgingCtrl.aging_time = 1;
	}

	mech_wr_target_step = IF_WR_REF;
	elec_wr_target_step = mech_wr_target_step * MOTOR_POLE_PAIR * ACTUATOR_GEAR_RATIO;
	AgingCtrl.elec_hz_target_step = (elec_wr_target_step/PI_2)*1024; // Q10

	Init_Validation_PosSensor(&encCheck);
	return 0;
}

static int Run_IF_Spd_Control()
{
	uint8_t IF_state = 0;
	int32_t diff = 0;
	float D2A = ADC1_RESOLUTION / CURRENT_SENSOR_RANGE;
	float current_limit = (ACTUAL_CURRENT_REF * 2);
	static int8_t IF_OcpCnt = 0;

	AgingCtrl.clk_timer++;
	if(AgingCtrl.clk_timer >= LOW_LEVEL_CONTROL_FREQUENCY){
		AgingCtrl.clk_timer = 0;
		AgingCtrl.sec_timer++;

		if(AgingCtrl.sec_timer >= 259200) {
			AgingCtrl.sec_timer = 259200; // 72h
 		}
	}

//	AgingCtrl.position = inc25KhzObj.userCnt;
//	diff = AgingCtrl.position - AgingCtrl.position_old;
//	AgingCtrl.speed = (diff / ACTUATOR_GEAR_RATIO * (LOW_LEVEL_CONTROL_FREQUENCY / 8192)) * 2 * M_PI;
//	AgingCtrl.position_old = inc25KhzObj.userCnt;

	if(AgingCtrl.sec_timer < FORCED_ALIGNED_TIME_SEC){
		IF_state = 0;
	}
	else if (AgingCtrl.sec_timer >= FORCED_ALIGNED_TIME_SEC \
			&& AgingCtrl.sec_timer < (FORCED_ALIGNED_TIME_SEC+((AgingCtrl.aging_time*60)>>1))){
		IF_state = 1;
	}
	else if (AgingCtrl.sec_timer >= (FORCED_ALIGNED_TIME_SEC+((AgingCtrl.aging_time*60)>>1)) \
			&& AgingCtrl.sec_timer < (FORCED_ALIGNED_TIME_SEC + (AgingCtrl.aging_time*60))){
		IF_state = 2;
	}
	else if (AgingCtrl.sec_timer >= (FORCED_ALIGNED_TIME_SEC + (AgingCtrl.aging_time*60))){
		IF_state = 3;
	}

//	if(IF_state > 0 && (AgingCtrl.speed < 0.1) && (AgingCtrl.speed > -0.1)){
//		AgingCtrl.StallCnt++;
//		if(AgingCtrl.StallCnt >= 100000){
//			AgingCtrl.state = 3;
//			AgingCtrl.StallCnt = 0;
//			GateDriver_ONOFF(DISABLE);
//			Stop_PWM();
//			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
//			MS_enum = AGING_TEST_DONE;
//			return 0;
//		}
//	}

	if(AgingCtrl.scaled_current_ref > ((ACTUAL_CURRENT_REF + AgingCtrl.current_add) * D2A)) {
		AgingCtrl.scaled_current_ref -= 2;
	}
	else if(AgingCtrl.scaled_current_ref < ((ACTUAL_CURRENT_REF + AgingCtrl.current_add) * D2A)){
		AgingCtrl.scaled_current_ref += 2;
	}

	switch(IF_state)
	{
		case 0:
				AgingCtrl.elec_hz_target_ramp = 0;

			break;
		case 1:
			AgingCtrl.elec_hz_target_ramp += IF_RAMP_CNT;
			if(AgingCtrl.elec_hz_target_ramp >= AgingCtrl.elec_hz_target_step){
				AgingCtrl.elec_hz_target_ramp = AgingCtrl.elec_hz_target_step;
			}

			break;
		case 2:
			AgingCtrl.elec_hz_target_ramp -= IF_RAMP_CNT;
			if(AgingCtrl.elec_hz_target_ramp <= -AgingCtrl.elec_hz_target_step){
				AgingCtrl.elec_hz_target_ramp = -AgingCtrl.elec_hz_target_step;
			}

			break;
		case 3:
			if(AgingCtrl.elec_hz_target_ramp > 0){
				AgingCtrl.elec_hz_target_ramp--;
				if(AgingCtrl.elec_hz_target_ramp < 0) {
					AgingCtrl.elec_hz_target_ramp = 0;
					AgingCtrl.scaled_current_ref = 0;
				}
			}
			else if(AgingCtrl.elec_hz_target_ramp < 0){
				AgingCtrl.elec_hz_target_ramp++;
				if(AgingCtrl.elec_hz_target_ramp > 0) {
					AgingCtrl.elec_hz_target_ramp = 0;
					AgingCtrl.scaled_current_ref = 0;
				}
			}
			else{
				AgingCtrl.state = 1;
				AgingCtrl.state |= (encCheck.faultSet_bit<<4);
				StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
				MS_enum = AGING_TEST_DONE;
				return 0;
			}

			break;

		default:
			break;
	}

	AgingCtrl.elec_angle += (AgingCtrl.elec_hz_target_ramp<<6)/LOW_LEVEL_CONTROL_FREQUENCY; // Q10+6



	if		(AgingCtrl.scaled_current_ref >  D2A * current_limit) { AgingCtrl.scaled_current_ref =  D2A * current_limit; }
	else if	(AgingCtrl.scaled_current_ref < -D2A * current_limit) { AgingCtrl.scaled_current_ref = -D2A * current_limit; }

	clarke_in.Iu =  motor_out.I_U;
	clarke_in.Iv =  motor_out.I_V;
	clarke_in.Iw =  motor_out.I_W;

	ClarkeTransform(&clarke_in, &clarke_out);

	park_in.Ia = clarke_out.Ia;
	park_in.Ib = clarke_out.Ib;
	park_in.theta_e = (AgingCtrl.elec_angle>>4);		// sine & cosine table range: 0~4096
	ParkTransform(&park_in, &park_out);

	Run_PID_Control(&pid_d_axis_ctrl,	AgingCtrl.scaled_current_ref,	park_out.Id, LOW_LEVEL_CONTROL_PERIOD);
	Run_PID_Control(&pid_q_axis_ctrl,	0,	park_out.Iq, LOW_LEVEL_CONTROL_PERIOD); // ParkOut.Iq : Total Current

	inv_park_in.Vd = pid_d_axis_ctrl.control_input;
	inv_park_in.Vq = pid_q_axis_ctrl.control_input;
	inv_park_in.theta_e = park_in.theta_e;
	InvParkInputToPhasor(&inv_park_in, &svm_phasor);

	motor_out.I_mag = sqrt(((park_out.Iq * park_out.Iq)>>4) + ((park_out.Id * park_out.Id)>>4));
	motor_out.I_mag = (motor_out.I_mag / (ADC1_RESOLUTION>>2)) * CURRENT_SENSOR_RANGE;

	if(motor_setting.peakCurr_limit <= 0){
		motor_setting.peakCurr_limit = FAULT_CURRENT_OCP_DEFAULT_LV;
	}

	if (motor_out.I_mag >= motor_setting.peakCurr_limit){
		IF_OcpCnt++;
		if (IF_OcpCnt >= FAULT_CURRENT_OCP_CNT) {
			IF_OcpCnt = 0;
			AgingCtrl.StallCnt = 0;
			AgingCtrl.state = 2;
			GateDriver_ONOFF(DISABLE);
			Stop_PWM();
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
			MS_enum = AGING_TEST_DONE;
			return 0;
		}
	}
	else {
		IF_OcpCnt--;
		if (IF_OcpCnt < 0) IF_OcpCnt = 0;
	}

	InvParkTransform(&inv_park_in, &inv_park_out);
	SVM_FOC(&inv_park_out, motor_in.batt_vdc, &motor_in.V_U_input, &motor_in.V_V_input, &motor_in.V_W_input);

	Validation_PosSensor(&encCheck);

	return 0;

}
static int Ext_IF_Spd_Control(){
	if(motor_setting.currCtrl_BW_radPsec == 0){
		pid_d_axis_ctrl.Kp = 0;
		pid_d_axis_ctrl.Ki = 0;
		pid_q_axis_ctrl.Kp = 0;
		pid_q_axis_ctrl.Ki = 0;
	}

	if(AgingCtrl.state == 3){
		AgingCtrl.current_add = ACTUAL_CURRENT_REF/2;
	}
	else {
		AgingCtrl.current_add = 0.0;
	}
	pid_d_axis_ctrl.err = 0;
	pid_d_axis_ctrl.err_sum = 0;
	pid_q_axis_ctrl.err = 0;
	pid_q_axis_ctrl.err_sum = 0;
	AgingCtrl.elec_angle = 0;
	AgingCtrl.aging_time = 0;
	motor_setting.peakCurr_limit = 0;
	AgingCtrl.StallCnt = 0;

	return 0;
}

static void Init_Validation_PosSensor(encCheck_t* ec)
{
	ec->faultSet_bit = 0;
	ec->abs1_errCnt = ec->abs2_errCnt = ec->inc1_errCnt = 0;
	ec->faultCnt = ((IF_WR_REF * MOTOR_POLE_PAIR * ACTUATOR_GEAR_RATIO / PI_2 / IF_RAMP_CNT) * 1024 * 4 * 10) / (POS_SENSOR_VALIDATION_PRD / LOW_LEVEL_CONTROL_PERIOD); // Q10
	// 1024 means Q8, 4 is the number of ramp direction, 10 is margin //
}

static void Validation_PosSensor(encCheck_t* ec)
{

	ec->tmr++;

	if(ec->tmr >= POS_SENSOR_VALIDATION_CNT){
		ec->tmr = 0;

		ec->abs1Delt = AbsObj1.rawBitsTo13Bit - ec->abs1Old;
		if(ec->abs1Delt > 4095) 		ec->abs1Delt -= 8192;
		else if(ec->abs1Delt < -4095)	ec->abs1Delt += 8192;
		ec->abs1Spd = ec->abs1Delt * SPD_TO_MECH_WR * ABS1_SPD_TO_ELEC_SPD;
		ec->abs1Spd = fabs(ec->abs1Spd);

		ec->abs2Delt = AbsObj2.rawBitsTo13Bit - ec->abs2Old;
		if(ec->abs2Delt > 4095)			ec->abs2Delt -= 8192;
		else if(ec->abs2Delt < -4095)	ec->abs2Delt += 8192;
		ec->abs2Spd = ec->abs2Delt * SPD_TO_MECH_WR;
		ec->abs2Spd = fabs(ec->abs2Spd);

		ec->inc1Delt = inc25KhzObj.currCnt - ec->inc1Old;
		ec->inc1Spd = ec->inc1Delt * SPD_TO_MECH_WR;
		if(ec->inc1Spd < 0) ec->inc1Spd = -ec->inc1Spd;
		ec->inc1Spd = fabs(ec->inc1Spd);

		ec->abs1Old = AbsObj1.rawBitsTo13Bit;
		ec->abs2Old = AbsObj2.rawBitsTo13Bit;
		ec->inc1Old = inc25KhzObj.currCnt;

		if((ec->abs1Spd < ENC_WR_LOWER_LIMIT) || (ec->abs1Spd > ENC_WR_UPPER_LIMIT)){
			ec->abs1_errCnt++;
			if(ec->abs1_errCnt > ec->faultCnt){
				ec->abs1_errCnt = ec->faultCnt;
				ec->faultSet_bit |= 0x01;
			}
		}

		if((ec->abs2Spd < ENC_WR_LOWER_LIMIT) || (ec->abs2Spd > ENC_WR_UPPER_LIMIT)){
			ec->abs2_errCnt++;
			if(ec->abs2_errCnt > ec->faultCnt){
				ec->abs2_errCnt = ec->faultCnt;
				ec->faultSet_bit |= 0x02;
			}
		}

		if((ec->inc1Spd < ENC_WR_LOWER_LIMIT) || (ec->inc1Spd > ENC_WR_UPPER_LIMIT)){
			ec->inc1_errCnt++;
			if(ec->inc1_errCnt > ec->faultCnt){
				ec->inc1_errCnt = ec->faultCnt;
				ec->faultSet_bit |= 0x04;
			}
		}
	}
}

#endif

static void Init_MaxAngleLimitCtrl(MaxAngleCtrl_t* pAngleLimit)
{
	pAngleLimit->outLimit = mid_ctrl_saturation; // 10A
	pAngleLimit->Wc = ANGLE_LIMIT_CTRL_BW;
	pAngleLimit->enable = ANGLE_LIMIT_CLEAR;
	pAngleLimit->temp_off = 1; // to prevent sudden movements when the power is turned on
//	pAngleLimit->Kp = pAngleLimit->Wc * (motor_properties.B * 65536 / 29) / 2;
//	pAngleLimit->Kd = pAngleLimit->Wc * (motor_properties.J * 65536 / 29) / 2;
	pAngleLimit->Kp = 40;
	pAngleLimit->Kd = 0.04;
}

static void MaxAngleLimitCtrl(MaxAngleCtrl_t* pAngleLimit, float Wr)
{
	float temp0 = 0.0;

	pAngleLimit->in = AbsObj1.posDeg[0]/1.875/180*M_PI;
	pAngleLimit->Wr = Wr;

/* joint limit angle detect */
	if(pAngleLimit->in > MAX_POS_LIMIT_READY_ANGLE) {
		pAngleLimit->enable = POS_LIMIT_MODE;

		temp0 = (MAX_POS_LIMIT_SET_ANGLE - pAngleLimit->in) \
						/ (MAX_POS_LIMIT_SET_ANGLE - MAX_POS_LIMIT_READY_ANGLE);

		if(temp0 < 0) temp0 = 0;
		else if(temp0 > 1) temp0 = 1;

		pAngleLimit->dec_ref = pAngleLimit->curr_ref * (1 - temp0);
	}
	else if(pAngleLimit->in < MAX_NEG_LIMIT_READY_ANGLE) {
		pAngleLimit->enable = NEG_LIMIT_MODE;

		temp0 = (MAX_NEG_LIMIT_SET_ANGLE - pAngleLimit->in) \
						/ (MAX_NEG_LIMIT_SET_ANGLE - MAX_NEG_LIMIT_READY_ANGLE);

		if(temp0 < 0) temp0 = 0;
		else if(temp0 > 1) temp0 = 1;

		pAngleLimit->dec_ref = pAngleLimit->curr_ref * (1 - temp0);

	}
	else{
		pAngleLimit->dec_ref = 0;
		pAngleLimit->temp_off = 0;
		pAngleLimit->enable = ANGLE_LIMIT_CLEAR;
	}


/* limit control start */
	if(pAngleLimit->enable == POS_LIMIT_MODE){ // Forward Direction
		// When the force acts in the direction of the +maximum joint angle
		if(pAngleLimit->in > MAX_POS_LIMIT_SET_ANGLE){
			pAngleLimit->err = MAX_POS_LIMIT_SET_ANGLE - pAngleLimit->in;

			if(pAngleLimit->err > 0) {
				pAngleLimit->err = 0;
				pAngleLimit->Wr = 0;
			}
			else{
				if(pAngleLimit->err < -MAX_LIMIT_PROTECTION_AGNLE){
					pAngleLimit->err = -MAX_LIMIT_PROTECTION_AGNLE;
				}
			}

			pAngleLimit->out = ((pAngleLimit->err * pAngleLimit->Kp) - ((pAngleLimit->Wr) * pAngleLimit->Kd));
			pAngleLimit->comp = pAngleLimit->out - pAngleLimit->dec_ref;
			if(pAngleLimit->comp > pAngleLimit->outLimit) pAngleLimit->comp = pAngleLimit->outLimit;
			else if(pAngleLimit->comp < -pAngleLimit->outLimit) pAngleLimit->comp = -pAngleLimit->outLimit;
		}
		else{							// abnormal operation -
			pAngleLimit->out = FirstLPF(0, pAngleLimit->out, 0.001);
			pAngleLimit->comp = -pAngleLimit->dec_ref;
		}
	}
	else if(pAngleLimit->enable == NEG_LIMIT_MODE){	 // Backward Direction
		if(pAngleLimit->in < MAX_NEG_LIMIT_SET_ANGLE){
			pAngleLimit->err = MAX_NEG_LIMIT_SET_ANGLE - pAngleLimit->in;

			if(pAngleLimit->err < 0) {
				pAngleLimit->err = 0;
				pAngleLimit->Wr = 0;
			}
			else{
				if(pAngleLimit->err > MAX_LIMIT_PROTECTION_AGNLE){
					pAngleLimit->err = MAX_LIMIT_PROTECTION_AGNLE;
				}
			}

			pAngleLimit->out = ((pAngleLimit->err * pAngleLimit->Kp) - ((pAngleLimit->Wr) * pAngleLimit->Kd));
			pAngleLimit->comp = pAngleLimit->out - pAngleLimit->dec_ref;
			if(pAngleLimit->comp > pAngleLimit->outLimit) pAngleLimit->comp = pAngleLimit->outLimit;
			else if(pAngleLimit->comp < -pAngleLimit->outLimit) pAngleLimit->comp = -pAngleLimit->outLimit;
		}
		else{
			pAngleLimit->out = FirstLPF(0, pAngleLimit->out, 0.001);
			pAngleLimit->comp = -pAngleLimit->dec_ref;
		}
	}
	else{
		pAngleLimit->out = 0;
		pAngleLimit->dec_ref = 0;
		pAngleLimit->comp = 0;
	}


/* limit control disable to avoid unnecessary torque */
	if(pAngleLimit->enable){
		if(fabs(pAngleLimit->curr_ref) <= 0.001){ // to avoid float error, not 0.0 but 0.0001
			pAngleLimit->rec_cnt++;
			if(pAngleLimit->rec_cnt >= 10000){
				pAngleLimit->rec_cnt = 0;
				pAngleLimit->out = 0;
				pAngleLimit->dec_ref = 0;
				pAngleLimit->comp = 0;
				pAngleLimit->enable = ANGLE_LIMIT_CLEAR;
				pAngleLimit->temp_off = 1;
			}
		}
		else{
			pAngleLimit->rec_cnt = 0;
		}

	}

	if(pAngleLimit->temp_off){
		pAngleLimit->out = 0;
		pAngleLimit->dec_ref = 0;
		pAngleLimit->comp = 0;
	}
}

static int Run_torque_measurement_jig3(void)
{
	static float ftemp0 = 0.0;

	if(torque_measurement_mode_curr_on){
		if(torque_measurement_warm_up_on){
			crnt_ramp_tmr++;
			if(crnt_ramp_tmr > 1250){
				crnt_ramp_tmr = 0;
				ftemp0 += 0.1;

				if(ftemp0 > 0.8) {
					ftemp0 = 0.8;
					torque_measurement_warm_up_cnt++;
				}

				if(torque_measurement_warm_up_cnt >= 20){
					ftemp0 = 0;
					crnt_ramp_tmr = 0;
					torque_measurement_warm_up_cnt = 0;
					torque_measurement_warm_up_on = 0;
				}
			}
		}
		else {
			crnt_ramp_tmr++;

			if(crnt_ramp_tmr > 1250){
				crnt_ramp_tmr = 0;
				torque_measurement_warm_up_cnt++;
				if(torque_measurement_warm_up_cnt >= 20){
					torque_measurement_warm_up_cnt = 20;
					ftemp0 -= 0.1;
				}
			}

			if(torque_measurement_mode_curr_on == 1){
				if(ftemp0 < -1.5) ftemp0 = -1.5;
			}
			else if(torque_measurement_mode_curr_on == 2){
				if(ftemp0 < -3.0) ftemp0 = -3.0;
			}
			Run_Current_Control();
		}
		motor_in.low_id_process_input = ftemp0;
	}

	if(torque_measurement_mode_curr_on == 0){
		if(torque_measurement_mode_prev_on == 1 || torque_measurement_mode_prev_on == 2){
			torque_measurement_warm_up_on = 1;
			torque_measurement_warm_up_cnt = 0;
			ftemp0 = 0.0;
			motor_in.low_id_process_input = 0;
			crnt_ramp_tmr = 0;
			Init_Current_Ctrl();
			Stop_PWM();
		}
	}

	torque_measurement_mode_prev_on = torque_measurement_mode_curr_on;

	return 0;
}

static void Set_MES(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	MES.code_length=(uint8_t)(((uint8_t*)req->data)[0]);
	memset(&MES.code, 0, sizeof(MES.code));
	memcpy(&MES.code, &(((char*)req->data)[1]), MES.code_length);

	Send_MSG((uint16_t)(GUI_SYNC|MES_CODE_SET), (uint8_t*)0, 1);

	res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_Joint_limit_Sw(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&joint_limit_sw, req->data, 1);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Enable_Torque_Measure(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&torque_measure_enable, req->data, 1);
	if(*(uint8_t *)req->data == 1){
		torque_measurement_mode_curr_on = 1;
	}
	else if(*(uint8_t *)req->data == 2){
		torque_measurement_mode_curr_on = 2;
	}
	else if(*(uint8_t *)req->data == 0){
		torque_measurement_mode_curr_on = 0;
	}

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static float FirstLPF(float currVal, float prevVal, float alpha)
{
    return alpha * currVal + (1.0 - alpha) * prevVal;
}

static void Init_elec_angle_compensation(thetaComp_t* pAngleComp)
{
	float temp0;

	for(int i = 0; i < MOTOR_POLE_PAIR_INT; i++){
		pAngleComp->absTheta[i] = motor_setting.commutation_set.raw_mechAngle[i];
		pAngleComp->incTheta[i] = motor_setting.commutation_set.raw_elecAngle[i];
	}

	pAngleComp->flag = 0;
	pAngleComp->flag_old = 0;

	for(int i = 0; i < MOTOR_POLE_PAIR_INT; i++){
		if(i+1 < MOTOR_POLE_PAIR_INT){
			temp0 = pAngleComp->absTheta[i+1] - pAngleComp->absTheta[i];
		}
		else{
			temp0 = pAngleComp->absTheta[0] - pAngleComp->absTheta[i];
		}
		if(temp0 < 0) temp0 += 8192;
		pAngleComp->arc_length[i] = ((temp0 - THETA_COMP_COEFF) / THETA_COMP_COEFF * 65536);
		pAngleComp->boundary[i] = 65536 + pAngleComp->arc_length[i];
	}

	Get_rotor_pole_number(&AbsObj2, &thetaComp);
}

/* boundary - number */
/* absTheta - number */
/* rate - region */

static void IncEnc_compensation(thetaComp_t* pAngleComp)
{
	pAngleComp->flag = motor_setting.elec_angle_homing.done;

	pAngleComp->theta_input = motor_out.elec_angle;

	if(pAngleComp->flag == 1 && pAngleComp->flag_old == 0){
		pAngleComp->eTheta_sum = 0;
		pAngleComp->eTheta_sum = motor_out.elec_angle;
		pAngleComp->theta_input_old = pAngleComp->theta_input;
	}

	pAngleComp->eTheta_delta = pAngleComp->theta_input - pAngleComp->theta_input_old;

	if(pAngleComp->eTheta_delta > (1<<15)) pAngleComp->eTheta_delta -= (1<<16);
	else if(pAngleComp->eTheta_delta < -(1<<15)) pAngleComp->eTheta_delta += (1<<16);

	pAngleComp->eTheta_sum += pAngleComp->eTheta_delta;

	if(pAngleComp->eTheta_sum >= pAngleComp->boundary[pAngleComp->pp_region]){
		pAngleComp->eTheta_sum -= pAngleComp->boundary[pAngleComp->pp_region];

		pAngleComp->pp_region++;
		if(pAngleComp->pp_region >= MOTOR_POLE_PAIR_INT){
			pAngleComp->pp_region -= MOTOR_POLE_PAIR_INT;
		}
	}
	else if(pAngleComp->eTheta_sum < 0){
		pAngleComp->pp_region--;
		if(pAngleComp->pp_region < 0){
			pAngleComp->pp_region += MOTOR_POLE_PAIR_INT;
		}

		pAngleComp->eTheta_sum += pAngleComp->boundary[pAngleComp->pp_region];
	}

	pAngleComp->eTheta = ((double)pAngleComp->eTheta_sum/(double)pAngleComp->boundary[pAngleComp->pp_region])*65536;
	if(pAngleComp->eTheta > 65535){
		pAngleComp->testCnt++;
	}
	pAngleComp->flag_old = pAngleComp->flag;
	pAngleComp->theta_input_old = pAngleComp->theta_input;
}

static void Get_rotor_pole_number(IOIF_AbsEnc_t* abs2, thetaComp_t* pAngleComp)
{
	int16_t isTemp0 = 0;
	int16_t isTemp1[MOTOR_POLE_PAIR_INT] = {0};

	for(int i = 0; i < MOTOR_POLE_PAIR_INT; i++){
		isTemp0 = pAngleComp->absTheta[i] - pAngleComp->absTheta[0];
		if(isTemp0 < 0){
			isTemp0 += inc25KhzObj.resolution;
		}
		isTemp1[i] = isTemp0;
	}

	isTemp0 = (int16_t)abs2->rawBit - (int16_t)pAngleComp->absTheta[0];
	if(isTemp0 < 0){
		isTemp0 += inc25KhzObj.resolution;
	}

	if(isTemp0 >= isTemp1[0] && isTemp0 < isTemp1[1]){
		pAngleComp->pp_region = 0;
	}
	else if(isTemp0 >= isTemp1[1] && isTemp0 < isTemp1[2]){
		pAngleComp->pp_region = 1;
	}
	else if(isTemp0 >= isTemp1[2] && isTemp0 < isTemp1[3]){
		pAngleComp->pp_region = 2;
	}
	else if(isTemp0 >= isTemp1[3] && isTemp0 < isTemp1[4]){
		pAngleComp->pp_region = 3;
	}
	else if(isTemp0 >= isTemp1[4] && isTemp0 < isTemp1[5]){
		pAngleComp->pp_region = 4;
	}
	else if(isTemp0 >= isTemp1[5] && isTemp0 < isTemp1[6]){
		pAngleComp->pp_region = 5;
	}
	else if(isTemp0 >= isTemp1[6] && isTemp0 < isTemp1[7]){
		pAngleComp->pp_region = 6;
	}
	else if(isTemp0 >= isTemp1[7] && isTemp0 < isTemp1[8]){
		pAngleComp->pp_region = 7;
	}
	else if(isTemp0 >= isTemp1[8] && isTemp0 < isTemp1[9]){
		pAngleComp->pp_region = 8;
	}
	else if(isTemp0 >= isTemp1[9] && isTemp0 < isTemp1[10]){
		pAngleComp->pp_region = 9;
	}
	else if(isTemp0 >= isTemp1[10] && isTemp0 < isTemp1[11]){
		pAngleComp->pp_region = 10;
	}
	else if(isTemp0 >= isTemp1[11] && isTemp0 < isTemp1[12]){
		pAngleComp->pp_region = 11;
	}
	else if(isTemp0 >= isTemp1[12] && isTemp0 < isTemp1[13]){
		pAngleComp->pp_region = 12;
	}
	else if(isTemp0 >= isTemp1[13]){
		pAngleComp->pp_region = 13;
	}
	else {
		pAngleComp->pp_region = 0;
	}
}
