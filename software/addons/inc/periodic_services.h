/*
 * periodic_services.h
 *
 *  Created on: Apr 15, 2022
 *      Author: Matej
 */

#ifndef INC_PERIODIC_SERVICES_H_
#define INC_PERIODIC_SERVICES_H_

#include "stm32g4xx_ll_tim.h"


typedef struct
{
	TIM_TypeDef *	timerN;

} periodic_services_handle_t;



void PSERV_init(void);
void PSERV_enable(void);
void PSERV_disable(void);
void PSERV_run_services_Callback(void);



#endif /* INC_PERIODIC_SERVICES_H_ */
