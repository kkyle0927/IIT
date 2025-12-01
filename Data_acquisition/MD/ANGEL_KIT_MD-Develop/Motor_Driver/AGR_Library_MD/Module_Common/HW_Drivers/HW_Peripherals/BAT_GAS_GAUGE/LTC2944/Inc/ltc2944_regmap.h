/**
 *-----------------------------------------------------------
 *       LTC2944 BATTERY GAS GAUGE DRIVER REGISTER MAP
 *-----------------------------------------------------------
 * @file ltc2944_regmap.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Hardware Peripherals Register Map for LTC2944.
 * 
 * This header file contains definitions for the LTC2944 Battery Gas Gauge IC.
 * It includes the device's register addresses, control bits, and other
 * constants required for interfacing with the IC.
 * 
 * The LTC2944 is used to monitor battery status, including charge, voltage,
 * current, and temperature. It communicates over I2C and has an adjustable
 * sense resistor value.
 * 
 * Refer to the LTC2944 datasheet for more detailed information on register
 * functions, configurations, and usage.
 *
 * @ref LTC2944 Datasheet
 */

#ifndef LTC2944_INC_LTC2944_REGMAP_H_
#define LTC2944_INC_LTC2944_REGMAP_H_

/* Device Specification */
#define LTC2944_RSENS 1 // mOhm

/* Device Address */
#define LTC2944_DEV_ADDR                   0b1100100

/* Register Address */
#define LTC2944_STATUS_REG                     0x00U
#define LTC2944_CONTROL_REG                    0x01U

#define LTC2944_ACCUMULATED_CAHRGE_REG         0x02U
#define LTC2944_CAHRGE_THRESHOLD_HIGH_REG      0x04U
#define LTC2944_CAHRGE_THRESHOLD_LOW_REG       0x06U

#define LTC2944_VOLTAGE_REG                    0x08U
#define LTC2944_VOLTAGE_THRESHOLD_HIGH_REG     0x0AU
#define LTC2944_VOLTAGE_THRESHOLD_LOW_REG      0x0CU

#define LTC2944_CURRENT_REG                    0x0EU
#define LTC2944_CURRENT_THRESHOLD_HIGH_REG     0x10U
#define LTC2944_CURRENT_THRESHOLD_LOW_REG      0x12U

#define LTC2944_TEMPERATURE_REG                0x14U
#define LTC2944_TEMPERATURE_THRESHOLD_HIGH_REG 0x16U
#define LTC2944_TEMPERATURE_THRESHOLD_LOW_REG  0x17U

/* Data Size */
#define LTC2944_STATUS_SIZE                     1U
#define LTC2944_CONTROL_SIZE                    1U

#define LTC2944_ACCUMULATED_CAHRGE_SIZE         2U
#define LTC2944_CAHRGE_THRESHOLD_HIGH_SIZE      2U
#define LTC2944_CAHRGE_THRESHOLD_LOW_SIZE       2U

#define LTC2944_VOLTAGE_SIZE                    2U
#define LTC2944_VOLTAGE_THRESHOLD_HIGH_SIZE     2U
#define LTC2944_VOLTAGE_THRESHOLD_LOW_SIZE      2U

#define LTC2944_CURRENT_SIZE                    2U
#define LTC2944_CURRENT_THRESHOLD_HIGH_SIZE     2U
#define LTC2944_CURRENT_THRESHOLD_LOW_SIZE      2U

#define LTC2944_TEMPERATURE_SIZE                2U
#define LTC2944_TEMPERATURE_THRESHOLD_HIGH_SIZE 1U
#define LTC2944_TEMPERATURE_THRESHOLD_LOW_SIZE  1U

/* Register Data */
// Status Flag
#define LTC2944_ALERT_CURRENT       0b1000000
#define LTC2944_ALERT_ACCUM_CHARGE  0b0100000
#define LTC2944_ALERT_TEMPERATURE   0b0010000
#define LTC2944_ALERT_CHARGE_HIGH   0b0001000
#define LTC2944_ALERT_CHARGE_LOW    0b0000100
#define LTC2944_ALERT_VOLTAGE       0b0000010
#define LTC2944_ALERT_UNDER_VOLTAGE 0b0000001

// Control Configuration
#define LTC2944_ADC_AUTO       0b11000000
#define LTC2944_ADC_SCAN       0b10000000
#define LTC2944_ADC_MANUAL     0b01000000
#define LTC2944_ADC_SLEEP      0b00000000

#define LTC2944_PRESCALE_1BIT    0b000000
#define LTC2944_PRESCALE_2BIT    0b001000
#define LTC2944_PRESCALE_4BIT    0b010000
#define LTC2944_PRESCALE_6BIT    0b011000
#define LTC2944_PRESCALE_8BIT    0b100000
#define LTC2944_PRESCALE_10BIT   0b101000
#define LTC2944_PRESCALE_12BIT   0b110000

#define LTC2944_ALCC_ALERT          0b100
#define LTC2944_ALCC_CHARGE_CPLT    0b010
#define LTC2944_ALCC_DISALBED       0b000

#define LTC2944_SHUTDOWN              0b1

// Operation Mode
#define LTC2944_OPR_MODE_CONFIGMODE 0b0000U
#define LTC2944_OPR_MODE_ACCGYR     0b0101U
#define LTC2944_OPR_MODE_ACCMAGGYR  0b0111U

#endif /* LTC2944_INC_LTC2944_REGMAP_H_ */
