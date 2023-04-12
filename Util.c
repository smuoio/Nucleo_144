/**
 * \file Util.c
 *
 *  Created on: 09 mar 2023
 *      Author: salvatore muoio
 */
#include "Util.h"


e_bool ispressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	e_bool lret =FALSE;
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET)
		lret = TRUE;
	return(lret);
}


