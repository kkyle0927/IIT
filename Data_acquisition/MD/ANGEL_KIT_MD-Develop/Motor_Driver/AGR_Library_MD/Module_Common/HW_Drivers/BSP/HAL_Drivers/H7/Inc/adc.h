/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H7_INC__ADC_H__
#define H7_INC__ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ########################## WALKON5_CM_ENABLED ############################## */
#ifdef WALKON5_CM_ENABLED
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

//extern ADC_HandleTypeDef hadc1;
//
//extern ADC_HandleTypeDef hadc2;
//
//extern ADC_HandleTypeDef hadc3;
//
///* USER CODE BEGIN Private defines */
//
///* USER CODE END Private defines */
//
//void MX_ADC1_Init(void);
//void MX_ADC2_Init(void);
//void MX_ADC3_Init(void);
//
///* USER CODE BEGIN Prototypes */
//
///* USER CODE END Prototypes */
//
//#ifdef __cplusplus
//}
//#endif
#endif /* WALKON5_CM_ENABLED */

/* ########################## L30_CM_ENABLED ############################## */
#if defined (L30_CM_ENABLED) || defined (WALKON5_CM_ENABLED)
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /* L30_CM_ENABLED */

/* ########################## SUIT_MINICM_ENABLED ############################## */
#ifdef SUIT_MINICM_ENABLED

#endif /* SUIT_MINICM_ENABLED */

/* ########################## WALKON5_MD_ENABLED ############################## */
#ifdef WALKON5_MD_ENABLED

#endif /* WALKON5_MD_ENABLED */

/* ########################## SUIT_MD_ENABLED ############################## */
#ifdef SUIT_MD_ENABLED

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* SUIT_MD_ENABLED */

/* ########################## L30_MD_REV06_ENABLED ############################## */
#ifdef L30_MD_REV06_ENABLED

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* L30_MD_REV06_ENABLED */

/* ########################## L30_MD_REV07_ENABLED ############################## */
#ifdef L30_MD_REV07_ENABLED

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* L30_MD_REV07_ENABLED */

/* ########################## L30_MD_REV08_ENABLED ############################## */
#ifdef L30_MD_REV08_ENABLED

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* L30_MD_REV08_ENABLED */

#endif /* H7_INC__ADC_H__ */

