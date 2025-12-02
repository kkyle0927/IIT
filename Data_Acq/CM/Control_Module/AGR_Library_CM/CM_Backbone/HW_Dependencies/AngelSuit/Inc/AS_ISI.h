

#ifndef CM_BACKBONE_HW_DEPENDENCIES_ANGELSUIT_AS_ISI_H_
#define CM_BACKBONE_HW_DEPENDENCIES_ANGELSUIT_AS_ISI_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdint.h>
#include "data_object_common.h"
#include "risk_mngr.h"

#include "ioif_gpio_common.h"
#include "ioif_mdbt42q-at.h"

#include "AS_dev_mngr.h"
#include "AS_whole_body_ctrl.h"

#include "ioif_sysctrl.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define ACTIVE_LOW 			0
#define ACTIVE_HIGH 		1
#define ISI_INPUT_N_MAX 	34
#define ISI_OUTPUT_N_MAX 	34

#define B_FLAG_N_MAX        60

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _ExtCondID {
	EXT0,
	EXT1,
	EXT2,
	EXT3,
	EXT4,
	EXT5,
	EXT6,
	EXT7,
	EXT8,
	EXT9,
	EXT10,
	EXT11,
	EXT12,
	EXT13,
	EXT14,
	EXT15,
	EXT16,
	EXT17,
	EXT18,
	EXT19,
	EXT20,
	EXT21,
	EXT22,
	EXT23,
	EXT24,
	EXT25,
	EXT26,
	EXT27,
	EXT28,
	EXT29,
	EXT30,
	EXT31,
	EXT32,
	EXT33,
	EXT34,
	EXT35,
	EXT36,
	EXT37,
	EXT38,
	EXT39,
	EXT40,
	EXT41,
	EXT42,
	EXT43,
	EXT44,
	EXT45,
	EXT46,
	EXT47,
	EXT48,
	EXT49,
	EXT50,
	EXT51,
	EXT52,
	EXT53,
	EXT54,
	EXT55,
	EXT56,
	EXT57,
	EXT58,
	EXT59,

	/* RM */
	EXT60 = 60,  //fault clear
	EXT61,
	EXT62,
	EXT63,

	/* STS Mode */
	EXT70 = 70,
	EXT71,
	EXT72,
	EXT73,
	EXT74,
	EXT75,
	EXT76,
	EXT77,
	EXT78,
	EXT79,
	EXT80,
	EXT81,
	EXT82,
	EXT83,
	EXT84,
	EXT85,
	EXT86,
	EXT87,
	EXT88,
	EXT89,
	EXT90,
	EXT91,
	EXT92,
	EXT93,

	EXT_COND_ID_NUM
} ExtCondID;

typedef enum _RiskManagementID{
	RM0, //fault clear
	RM1,
	RM2,
	RM3,
	RM4,
	RM5,
	RM6,
	RM7,

	RM_ID_NUM
} RiskManagementID;

typedef struct _ISI_ButtonObj {
	IOIF_GPIOPort_t GPIO_port;
	IOIF_GPIOPin_t  GPIO_pin;

	uint8_t active_type; // 0: Active LOW, 1: Active HIGH

	uint8_t state_curr;
	uint8_t state_prev;
	uint32_t on_time;

} ISI_ButtonObj;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern bool isi_input_vectors[EXT_COND_ID_NUM];
extern bool isi_output_vectors[EXT_COND_ID_NUM];
extern bool rm_flag[RM_ID_NUM];

extern bool BFlag[EXT_COND_ID_NUM];


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */
void Check_ISI(void);
void Flush_ISI(void);

void Init_Button_ISI(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* CM_BACKBONE_HW_DEPENDENCIES_ANGELSUIT_AS_ISI_H_ */
