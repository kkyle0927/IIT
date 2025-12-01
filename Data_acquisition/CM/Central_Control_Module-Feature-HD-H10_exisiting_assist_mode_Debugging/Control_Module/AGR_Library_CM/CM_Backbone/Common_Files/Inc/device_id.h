#ifndef CM_BACKBONE_COMMON_FILES_DEVICE_CONFIG_H_
#define CM_BACKBONE_COMMON_FILES_DEVICE_CONFIG_H_

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MAX_N_DEV       50 // Max number of devices that can be registered in the device list
#define MAX_N_CANFD_DEV 16 // Max number of fdcan devices that can be handled
#define MAX_N_MD        16 // Max number of MD that can be handled
#define MAX_L_NAME      20 // Max length of device name


#define MAX_CANFD_ID    15 // Maximum fdcan ID that device can get = 0x1111


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _DeviceID {

	DEV_IDX_AM,
	DEV_IDX_CM,
	DEV_IDX_MD1,
	DEV_IDX_MD2,
	DEV_IDX_MD3,
	DEV_IDX_MD4,
	DEV_IDX_MD5,
	DEV_IDX_MD6,
	DEV_IDX_MD7,
	DEV_IDX_MD8,
	DEV_IDX_MD9,
	DEV_IDX_MD10,
	DEV_IDX_MD11,
	DEV_IDX_MD12,
	DEV_IDX_MD13,
	DEV_IDX_MD14,
	DEV_IDX_MD15,
	DEV_IDX_MD16,
	DEV_IDX_WIDM1,
	DEV_IDX_WIDM2,
	DEV_IDX_WIDM3,
	DEV_IDX_WIDM4,
	DEV_IDX_WIDM5,
	DEV_IDX_WIDM6,
	DEV_IDX_WIDM7,
	DEV_IDX_WIDM8,
	DEV_IDX_WIDM9,
	DEV_IDX_WIDM10,
	DEV_IDX_L_CRUTCH,
	DEV_IDX_R_CRUTCH,
	DEV_IDX_L_GRF_MODULE,
	DEV_IDX_R_GRF_MODULE,

	DEVICE_ID_NUM
} DeviceID;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */




#endif /* CM_BACKBONE_COMMON_FILES_DEVICE_CONFIG_H_ */
