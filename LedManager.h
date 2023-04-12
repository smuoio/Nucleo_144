/**
 * \file LedManager.h
 *
 *  Created on: 09 mar 2023
 *      Author: salvatore muoio
 */

#ifndef LEDMANAGER_H_
#define LEDMANAGER_H_
#include "Type.h"

typedef struct
{
	unsigned char pressed;
}t_dataled;

/**
 * \fn getdataled
 * \brief interface function to access to t_dataled structure
 */

t_dataled *getdataled(void);

/**
 * \fn ledmanager
 * \brief main function that manages leds
 */
void ledmanager(void);
void leddebug(void);



#endif /* LEDMANAGER_H_ */
