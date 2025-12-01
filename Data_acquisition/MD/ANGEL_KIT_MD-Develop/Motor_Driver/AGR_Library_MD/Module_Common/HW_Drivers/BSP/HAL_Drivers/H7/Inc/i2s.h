/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2s.h
  * @brief   This file contains all the function prototypes for
  *          the i2s.c file
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
#ifndef H7_INC__I2S_H__
#define H7_INC__I2S_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ########################## WALKON5_CM_ENABLED ############################## */
#ifdef WALKON5_CM_ENABLED

#endif /* WALKON5_CM_ENABLED */

/* ########################## L30_CM_ENABLED ############################## */
#ifdef L30_CM_ENABLED

#endif /* L30_CM_ENABLED */

/* ########################## SUIT_MINICM_ENABLED ############################## */
#ifdef SUIT_MINICM_ENABLED
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

// extern I2S_HandleTypeDef hi2s2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2S2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */
#endif /* SUIT_MINICM_ENABLED */

/* ########################## WALKON5_MD_ENABLED ############################## */
#ifdef WALKON5_MD_ENABLED

#endif /* WALKON5_MD_ENABLED */

/* ########################## SUIT_MD_ENABLED ############################## */
#ifdef SUIT_MD_ENABLED

#endif /* SUIT_MD_ENABLED */

/* ########################## L30_MD_REV06_ENABLED ############################## */
#ifdef L30_MD_REV06_ENABLED

#endif /* L30_MD_REV06_ENABLED */

/* ########################## L30_MD_REV07_ENABLED ############################## */
#ifdef L30_MD_REV07_ENABLED

#endif /* L30_MD_REV07_ENABLED */

/* ########################## L30_MD_REV08_ENABLED ############################## */
#ifdef L30_MD_REV08_ENABLED

#endif /* L30_MD_REV08_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* H7_INC__I2S_H__ */

