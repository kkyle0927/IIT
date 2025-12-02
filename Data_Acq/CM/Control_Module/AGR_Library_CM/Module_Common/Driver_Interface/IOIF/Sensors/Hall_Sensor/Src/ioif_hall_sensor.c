/*
 * ioif_hall_sensor.c
 *
 *  Created on: 2023. 9. 1.
 *      Author: User
 */
#include "ioif_hall_sensor.h"

void Read_Hall_Sensors(IOIF_HallSensor_t* t_hall_sensor)
{
	uint8_t            t_hall_logic;
	BSP_GPIOPinState_t t_H1, t_H2, t_H3;

	t_H1 = BSP_ReadGPIOPin(BSP_GPIO_PORT_C, BSP_GPIO_PIN_6);
	t_H2 = BSP_ReadGPIOPin(BSP_GPIO_PORT_C, BSP_GPIO_PIN_7);
	t_H3 = BSP_ReadGPIOPin(BSP_GPIO_PORT_C, BSP_GPIO_PIN_8);

	t_hall_sensor->H1 = t_H1;
	t_hall_sensor->H2 = t_H2;
	t_hall_sensor->H3 = t_H3;

	t_hall_logic = (t_H1 << 2)|(t_H2 << 1)|t_H3;

	// (Error Detection 1) -----------------------------------------------------------------------------------------
	if ((t_hall_logic == 0b000) || (t_hall_logic == 0b111)) { t_hall_sensor->error_cnt++; }// Error Handler
	//--------------------------------------------------------------------------------------------------------------

	else if (t_hall_logic == 0b001)
	{
		if      (t_hall_sensor->hall_logic == 0b001) {t_hall_sensor->direction = 0; }
		else if (t_hall_sensor->hall_logic == 0b011) {t_hall_sensor->direction = +t_hall_sensor->direction_set; }
		else if (t_hall_sensor->hall_logic == 0b101) {t_hall_sensor->direction = -t_hall_sensor->direction_set; }
		else    { if (t_hall_sensor->read_cnt != 0)   t_hall_sensor->error_cnt++; } // (Error Detection 2)
    }

	else if (t_hall_logic == 0b101)
	{
		if      (t_hall_sensor->hall_logic == 0b101) {t_hall_sensor->direction = 0; }
		else if (t_hall_sensor->hall_logic == 0b001) {t_hall_sensor->direction = +t_hall_sensor->direction_set; }
		else if (t_hall_sensor->hall_logic == 0b100) {t_hall_sensor->direction = -t_hall_sensor->direction_set; }
		else    { if (t_hall_sensor->read_cnt != 0)   t_hall_sensor->error_cnt++; } // (Error Detection 2)
    }

	else if (t_hall_logic == 0b100)
	{
		if      (t_hall_sensor->hall_logic == 0b100) {t_hall_sensor->direction = 0; }
		else if (t_hall_sensor->hall_logic == 0b101) {t_hall_sensor->direction = +t_hall_sensor->direction_set; }
		else if (t_hall_sensor->hall_logic == 0b110) {t_hall_sensor->direction = -t_hall_sensor->direction_set; }
		else    { if (t_hall_sensor->read_cnt != 0)   t_hall_sensor->error_cnt++; } // (Error Detection 2)
    }

	else if (t_hall_logic == 0b110)
	{
		if      (t_hall_sensor->hall_logic == 0b110) {t_hall_sensor->direction = 0; }
		else if (t_hall_sensor->hall_logic == 0b100) {t_hall_sensor->direction = +t_hall_sensor->direction_set; }
		else if (t_hall_sensor->hall_logic == 0b010) {t_hall_sensor->direction = -t_hall_sensor->direction_set; }
		else    { if (t_hall_sensor->read_cnt != 0)   t_hall_sensor->error_cnt++; } // (Error Detection 2)
    }

	else if (t_hall_logic == 0b010)
	{
		if      (t_hall_sensor->hall_logic == 0b010) {t_hall_sensor->direction = 0; }
		else if (t_hall_sensor->hall_logic == 0b110) {t_hall_sensor->direction = +t_hall_sensor->direction_set; }
		else if (t_hall_sensor->hall_logic == 0b011) {t_hall_sensor->direction = -t_hall_sensor->direction_set; }
		else    { if (t_hall_sensor->read_cnt != 0)   t_hall_sensor->error_cnt++; } // (Error Detection 2)
    }

	else if (t_hall_logic == 0b011)
	{
		if      (t_hall_sensor->hall_logic == 0b011) {t_hall_sensor->direction = 0; }
		else if (t_hall_sensor->hall_logic == 0b010) {t_hall_sensor->direction = +t_hall_sensor->direction_set; }
		else if (t_hall_sensor->hall_logic == 0b001) {t_hall_sensor->direction = -t_hall_sensor->direction_set; }
		else    { if (t_hall_sensor->read_cnt != 0)   t_hall_sensor->error_cnt++; } // (Error Detection 2)
    }

	t_hall_sensor->read_cnt++;
	t_hall_sensor->cnt          = t_hall_sensor->cnt + t_hall_sensor->direction;
	t_hall_sensor->hall_logic_f = t_hall_sensor->hall_logic;
	t_hall_sensor->hall_logic   = t_hall_logic;
}
