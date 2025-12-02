/**
 *-----------------------------------------------------------
 *          INCREMENTAL ENCODER (HALL) INTERFACE
 *-----------------------------------------------------------
 * @file ioif_inc_enc.h
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief Unified IO Interface header for managing Incremental Encoders.
 *
 * This header file consolidates the IO interfaces of various
 * incremental encoders. It provides a unified point of access
 * for developers wishing to integrate or utilize incremental encoder
 * functionalities within the AngelRobotics framework.
 *
 * By including this header, users can easily interface with different
 * incremental encoders without needing to delve into each specific
 * encoder's individual interface. This approach simplifies sensor
 * integration and ensures a consistent interface across different
 * incremental encoders.
 *
 * Currently, this header includes the interface for the RMB20IC incremental encoder
 * (ioif_rmb20ic.h). As new incremental encoders are integrated into the system,
 * their specific IO interfaces will be included in this unified header.
 *
 * @ref RMB20IC_DATA_SHEET_XXXXXXX.pdf
 *
 * Note: Future encoder datasheets will be referenced as they are integrated.
 */

#ifndef HALL_SENSOR_IOIF_HALL_SENSOR_H_
#define HALL_SENSOR_IOIF_HALL_SENSOR_H_

#include <stdint.h>
#include "bsp_gpio.h"
/**
 *-----------------------------------------------------------
 *              			HEADERS
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */



#pragma pack(push,1)
typedef struct _IOIF_HallSensor_t{

	uint8_t H1;
	uint8_t H2;
	uint8_t H3;
	uint8_t hall_logic;
	uint8_t hall_logic_f;
	int8_t  direction;     // rotational direction
	int8_t  direction_set;

	int32_t cnt;

	uint32_t read_cnt;     // ++ when read
	uint32_t error_cnt;    // if error generate, +1
} IOIF_HallSensor_t;
#pragma pack(pop)

void Read_Hall_Sensors(IOIF_HallSensor_t* t_hall_sensor);

#endif /* HALL_SENSOR_IOIF_HALL_SENSOR_H_ */
