/*
 * pca9685.h
 *
 *  Created on: Jul 30, 2022
 *      Author: Matej
 */

#ifndef INCLUDE_PCA9685_H_
#define INCLUDE_PCA9685_H_
#endif

// ----------- Include other modules (for public) -------------
#include "stm32g4xx_hal.h"

// Include the stdint.h library to get integer number definitions.
#include "stdint.h"


// -------------------- Public defines ------------------------

#define PCA_DEFAULT_ADDRESS 0x80
#define PCA_REG_MODE1 		0x00
#define PCA_REG_MODE2 		0x01
#define PCA_LEDx_REG_START 	0x06
#define PCA_LED_REG_SIZE 	0X04
#define PCA_PRESCALE		0xFE

#define PCA_I2C_TIMEOUT 	1



// -------------------- Public definitions --------------------

typedef struct {

	I2C_HandleTypeDef *i2c_handle;
	uint16_t device_address;
	uint8_t inverted;
	uint32_t osc_freq;

} pca9685_handle_t;


HAL_StatusTypeDef pca9685_write_u8(pca9685_handle_t *handle, uint8_t address, uint8_t value);

HAL_StatusTypeDef pca9685_read_u8(pca9685_handle_t *handle, uint8_t address, uint8_t *data);

void pca9685_init(pca9685_handle_t *handle);
void pca9685_reset(pca9685_handle_t *handle);
void pca9685_sleep(pca9685_handle_t *handle);
void pca9685_wakeup(pca9685_handle_t *handle);
void pca9685_setPrescaleFreq(pca9685_handle_t *handle, float freq);
void pca9685_setOscFreq(pca9685_handle_t *handle, uint32_t freq);
void pca9685_setPWM(pca9685_handle_t *handle, uint8_t num, uint16_t on, uint16_t off);
