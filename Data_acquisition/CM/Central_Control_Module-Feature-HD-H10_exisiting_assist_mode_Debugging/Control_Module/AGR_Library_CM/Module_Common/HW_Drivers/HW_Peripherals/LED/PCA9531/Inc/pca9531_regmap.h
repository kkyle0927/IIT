
/**
 *-----------------------------------------------------------
 *     PCA9531 8-BIT I2C LED DIMMER DRIVER REGISTER MAP          
 *-----------------------------------------------------------
 *  @file PCA9531_REGMAP.h
 * @date Created on: Aug 8, 2023
 * @author AngelRobotics HW Team
 * @brief Register map definitions for PCA9531 LED driver.
 * 
 * This header file contains definitions for device addresses, register addresses,
 * data sizes, and configurations related to the PCA9531 LED driver.
 * 
 * Refer to the PCA9531 datasheet and related documents for more information.
 *
 * @ref PCA9531 Datasheet
 */

#ifndef PCA9531_INC_PCA9531_REGMAP_H_
#define PCA9531_INC_PCA9531_REGMAP_H_

/* Device Specification */

/* Device Address */
#define PCA9531_DEV_ADDR_PREFIX     0b1100000
#define PCA9531_DEV_ADDR_MAX_HW_ID  0b11

/* Register Address */
#define PCA9531_CONTROL_REG 0b00000000U
#define PCA9531_INPUT_REG   0b00000000U
#define PCA9531_PSC0_REG    0b00000001U
#define PCA9531_PWM0_REG    0b00000010U
#define PCA9531_PSC1_REG    0b00000011U
#define PCA9531_PWM1_REG    0b00000100U
#define PCA9531_LS0_REG     0b00000101U //LED0~3
#define PCA9531_LS1_REG     0b00000110U //LED4~7

/* Data Size */
#define PCA9531_CONTROL_SIZE 1U
#define PCA9531_PWM0_SIZE    1U
#define PCA9531_PWM1_SIZE    1U
#define PCA9531_LS0_SIZE     1U
#define PCA9531_LS1_SIZE     1U

#endif /* PCA9531_INC_PCA9531_REGMAP_H_ */
