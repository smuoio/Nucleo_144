/**
 * \file LedManager.cpp
 *
 *  Created on: 09 mar 2023
 *      Author: salvatore muoio
 */
#include "main.h"
#include "Util.h"
#include "LedManager.h"


static t_dataled dataled;

static void activateled(void);


t_dataled *getdataled(void)
{
	return(&dataled);
}

void ledmanager(void)
{
	if (ispressed(Blue_Button_GPIO_Port,Blue_Button_Pin) == TRUE)
		getdataled()->pressed++;;
	activateled();
}


static void activateled(void)
{
	switch(getdataled()->pressed)
	{
	case 0:
		break;
	case 1:
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED2_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED3_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		getdataled()->pressed = 0x0;
		HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
		break;
	default:
		getdataled()->pressed = 0x0;
		HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
	}
}

void leddebug(void)
{
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}
