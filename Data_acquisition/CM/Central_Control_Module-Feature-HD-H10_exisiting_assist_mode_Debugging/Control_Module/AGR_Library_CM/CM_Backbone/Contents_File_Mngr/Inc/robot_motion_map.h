#ifndef CM_BACKBONE_CCONTENTS_FILE_MOTION_MAP_H_
#define CM_BACKBONE_CCONTENTS_FILE_MOTION_MAP_H_

#include "module.h"

#include <stdint.h>
#include <string.h>

#include "device_id.h"
#include "robot_setting.h"

#ifdef SUIT_MINICM_ENABLED

#include "AS_system_ctrl.h"
#include "AS_ble_comm_hdlr.h"
#endif 

#include "robot_motionmap_vector.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MAX_N_MOTION_SET  40   // Define Maximum Available Number of Motion Set for a Single Motion Map
#define MAX_N_C_VECTORS   1    // Define Maximum Number of Controller Gain Vectors for a Single Motion Set
#define MAX_N_P_VECTORS   10   // Define Maximum Number of P Vectors for a Single Motion Set
#define MAX_N_F_VECTORS   10   // Define Maximum Number of F Vectors for a Single Motion Set
#define MAX_N_I_VECTORS   10   // Define Maximum Number of I Vectors for a Single Motion Set
#define NUMEL_C_VECTOR    6    // Define Number of Element of a Single P Vector
#define NUMEL_P_VECTOR    4    // Define Number of Element of a Single P Vector
#define NUMEL_F_VECTOR    3    // Define Number of Element of a Single F Vector
#define NUMEL_I_VECTOR    5    // Define Number of Element of a Single I Vector

#define HALF_N_P_VECTORS  5    // for receiving and saving p vector data
#define HALF_N_F_VECTORS  5    // for receiving and saving f vector data
#define HALF_N_I_VECTORS  5    // for receiving and saving i vector data


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _State {
  e_DISABLE = 0,
  e_ENABLE = !e_DISABLE
} State;

typedef enum _MotionSetID_SUIT_Test_t {
	MS_NONE,
	MS_RHS_LTS,
	MS_LHS_RTS,
	MS_RL_SWING,
	MS_LR_SWING,

	MS_ID_NUM
} MotionSetID;

/********************************************************/
// Erase Later

typedef float   MotionSetFile_Cvector[MAX_N_MD][MAX_N_C_VECTORS][NUMEL_C_VECTOR];
typedef int16_t MotionSetFile_Pvector[MAX_N_MD][MAX_N_P_VECTORS][NUMEL_P_VECTOR];
typedef int16_t MotionSetFile_Fvector[MAX_N_MD][MAX_N_F_VECTORS][NUMEL_F_VECTOR];
typedef int16_t MotionSetFile_Ivector[MAX_N_MD][MAX_N_I_VECTORS][NUMEL_I_VECTOR];

/********************************************************/

typedef struct _SingleJointMotionSet {
	C_Vector     c_vector[MAX_N_C_VECTORS];
	P_Vector     p_vector[MAX_N_P_VECTORS];
	F_Vector     f_vector[MAX_N_F_VECTORS];
	I_Vector     i_vector[MAX_N_I_VECTORS];
} __attribute__((packed))SingleJointMotionSet;

typedef struct _RobotMotionSet {
	SingleJointMotionSet MD[MAX_N_MD];
	uint8_t MS_ID;
	uint8_t  max_p_vectors_len;
	uint8_t  max_f_vectors_len;
	uint8_t  max_i_vectors_len;
} __attribute__((packed))RobotMotionSet;

typedef struct _MotionMapFileInfo {
	uint8_t     robot_id;
	uint8_t     file_version_major;
	uint8_t     file_version_minor;
	uint8_t     file_version_patch;

	RobotMotionSet MS[MAX_N_MOTION_SET];

	uint16_t  num_ms;
	uint16_t  cnt;
	uint8_t   send_state;
} __attribute__((packed))MotionMapFileInfo;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern MotionMapFileInfo MotionMap_File_Read;
extern MotionMapFileInfo MotionMap_File;

extern MotionSetFile_Cvector c_vectors_FF_PD_DOB;

extern MotionSetFile_Pvector p_vectors_stand;
extern MotionSetFile_Pvector p_vectors_squat;
extern MotionSetFile_Pvector p_vectors_air_sit;
extern MotionSetFile_Pvector p_vectors_sit;
extern MotionSetFile_Pvector p_vectors_sit2stand;
extern MotionSetFile_Pvector p_vectors_stand2sit;
extern MotionSetFile_Pvector p_vectors_walking;
extern MotionSetFile_Pvector p_vectors_half_walking;
extern MotionSetFile_Pvector p_vectors_abdduction;
extern MotionSetFile_Pvector p_vectors_sin_30deg;
extern MotionSetFile_Pvector p_vectors_dance;
extern MotionSetFile_Pvector p_vectors_left_half_swing;
extern MotionSetFile_Pvector p_vectors_left_full_swing;
extern MotionSetFile_Pvector p_vectors_right_full_swing;
extern MotionSetFile_Pvector p_vectors_knee_test;
extern MotionSetFile_Pvector p_vector_left_foot_up;
extern MotionSetFile_Pvector p_vector_right_foot_up;
extern MotionSetFile_Pvector p_vector_left_tilt;
extern MotionSetFile_Pvector p_vector_right_tilt;

extern MotionSetFile_Ivector i_vectors_test;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Make_MotionMap_Examples(void);
void Save_MotionMap(void);
void Download_MotionMap(void);
int Check_MM_Save(void);

void Reset_MotionMap(int8_t val);
void Download_MotionSet(uint8_t t_MS_idx,
						MotionSetFile_Cvector MotionSet_c,
						MotionSetFile_Pvector MotionSet_p,
						MotionSetFile_Fvector MotionSet_f,
						MotionSetFile_Ivector MotionSet_i);

void Get_Max_PFI_Vectors_Length(void);
void Send_Motion_Set(uint8_t MS_idx);
void Set_Robot_Information(void);


#endif /* CM_BACKBONE_CCONTENTS_FILE_MOTION_MAP_H_ */
