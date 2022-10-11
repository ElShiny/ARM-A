/*
 * periodic_services.c
 *
 *  Created on: Apr 15, 2022
 *      Author: Matej
 */

#include "periodic_services.h"
#include "kbd.h"

periodic_services_handle_t periodic_services;


void PSERV_init(void){

	periodic_services.timerN = TIM6;

	LL_TIM_EnableCounter (periodic_services.timerN);

}


void PSERV_enable(void){
	LL_TIM_EnableIT_UPDATE (periodic_services.timerN);
}


void PSERV_disable(void){
	LL_TIM_DisableIT_UPDATE (periodic_services.timerN);
}


void PSERV_run_services_Callback(void){
	KBD_scan();
	//KBD_demo_toggle_LEDs_if_buttons_pressed();
}
