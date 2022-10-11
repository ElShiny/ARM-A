/*
 * mcp3464.h
 *
 *  Created on: 17 Sep 2022
 *      Author: Matej
 */

#ifndef INC_MCP3464_H_
#define INC_MCP3464_H_


#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_gpio.h"



#define NUM_OF_JOINTS 16

#define MCP3464_SPI_PORT 		hspi1

#define MCP3464_CS1_GPIO_Port  	GPIOA
#define MCP3464_CS1_Pin        	LL_GPIO_PIN_4

#define MCP3464_CS2_GPIO_Port  	GPIOA
#define MCP3464_CS2_Pin        	LL_GPIO_PIN_6



typedef struct {

	uint16_t position_raw[NUM_OF_JOINTS];
	uint16_t position_offset_streched[NUM_OF_JOINTS];
	float    position_offset_squished[NUM_OF_JOINTS];
	int16_t  position_call[NUM_OF_JOINTS];
	float 	 position_rad[NUM_OF_JOINTS];
	uint8_t  pointer;

}MCP3464;

MCP3464 hand;





void MCP_init();
void writeReg(uint8_t reg, uint8_t* data, uint8_t len, uint8_t cs);
void readReg(uint8_t reg, uint8_t* data, uint8_t len, uint8_t cs);
void fastCMD(uint8_t cmd, uint8_t cs);
void analogRead();
void MCP3464_Calibrate(void);
void MCP3464_Radians(void);
void EXTIValue_callback();

#endif /* INC_MCP3464_H_ */
