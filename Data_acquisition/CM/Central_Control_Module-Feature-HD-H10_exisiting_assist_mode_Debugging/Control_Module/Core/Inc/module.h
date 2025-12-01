#ifndef USER_DEFINED_MODULE_H_
#define USER_DEFINED_MODULE_H_

/* ------------------- Module Enable/Disable ------------------- */
//******* IF CM *******//
#define SUIT_MINICM_ENABLED

//******* IF MD *******//
//#define SUIT_MD_ENABLED


/* ------------------- Select CM or MD or WIDM For WIDM(Gait Ctrl) ------------------- */
#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED) || defined(SUIT_MINICM_ENABLED)
#define CM_MODULE
#endif
#if defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED) || defined(SUIT_MD_ENABLED)
#define MD_MODULE
#endif
#if defined(WIDM_ENABLED)
#define WIDM_MODULE
#endif

/* ------------------- SUIT_MINICM_ENABLED ------------------- */
#ifdef SUIT_MINICM_ENABLED

#define _USE_OS_RTOS_BSP
#define _USE_OS_RTOS
#define _USE_CMSISV2
#define _USE_SEMAPHORE
//#define _USE_DEBUG_CLI
#define IOIF_CPUTEMP_ENABLE
#define IOIF_BATMONITOR_ENABLED
#define IOIF_MAGNETO_ENABLED
#define IOIF_ACCGYRO_ENABLED
#define IOIF_PCA9957HNMP_ENABLED
//#define IOIF_MDBT42Q_AT_ENABLED
#define IOIF_ESP32_ENABLED
#define IOIF_FATFS_ENABLED
#define IOIF_AUDIO_WAV_SAI_ENABLED
#define BUS_VOLTAGE 24
#define VBUS2DUTY_RATIO 200
#define FATFS_SD_ENABLE
//#define FATFS_USB_ENABLE
#define IOIF_MCP79510_ENABLED
#define IOIF_QSPI_FLASH_ENABLED

/* Optional Functions */
// #define AIR_WALKING_MODE_ENABLED
//#define USB_LOG_ENABLED
// #define SENSOR_DATA_LOGGING_ENABLED

#endif /* SUIT_MINICM_ENABLED */
/* ------------------- SUIT_MD_ENABLED ------------------- */
#ifdef SUIT_MD_ENABLED

#define _USE_BAREMETAL
#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED
#define IOIF_TMCS1100A3_ENABLED
#define IOIF_503NTC_ENABLED
#define IOIF_GRFSENSOR_ENABLED
#define IOIF_RMB20SC_ENABLED
#define IOIF_RMB20IC_ENABLED
#define BUS_VOLTAGE 24
#define VBUS2DUTY_RATIO 200

#endif /* SUIT_MD_ENABLED */


#endif /* USER_DEFINED_MODULE_H_ */
