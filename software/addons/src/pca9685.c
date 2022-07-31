/*
 * pca9685.c
 *
 *  Created on: Jul 30, 2022
 *      Author: Matej
 */

/* **************** MODULE DESCRIPTION *************************

Support drivers for pca9685. Original drivers for Arduino:
https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library

************************************************************* */

// ----------- Include other modules (for private) -------------

#include "pca9685.h"


// ---------------------- Private definitions ------------------





// -------------- Public function implementations --------------

HAL_StatusTypeDef pca9685_write_u8(pca9685_handle_t *handle, uint8_t address, uint8_t value)
{
	uint8_t data[] = {address, value};
	return HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, data, 2, PCA_I2C_TIMEOUT);
}

HAL_StatusTypeDef pca9685_read_u8(pca9685_handle_t *handle, uint8_t address, uint8_t *data)
{
	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, &address, 1, PCA_I2C_TIMEOUT);
	return HAL_I2C_Master_Receive(handle->i2c_handle, handle->device_address, data, 1, PCA_I2C_TIMEOUT);
}

void pca9685_init(pca9685_handle_t *handle){

	//assert(handle->i2c_handle != NULL);
	pca9685_reset(handle);
	pca9685_setPrescaleFreq(handle, 1000);
	pca9685_setOscFreq(handle, 25000000);
}

void pca9685_reset(pca9685_handle_t *handle) {
	pca9685_write_u8(handle, PCA_REG_MODE1, 0x80);
	HAL_Delay(10);
}

void pca9685_sleep(pca9685_handle_t *handle) {
  uint8_t awake;
  pca9685_read_u8(handle, PCA_REG_MODE1, &awake);
  uint8_t sleep = awake | 0x10; // set sleep bit high
  pca9685_write_u8(handle, PCA_REG_MODE1, sleep);
  HAL_Delay(10); // wait until cycle ends for sleep to be active
}

void pca9685_wakeup(pca9685_handle_t *handle) {
  uint8_t sleep;
  pca9685_read_u8(handle, PCA_REG_MODE1, &sleep);
  uint8_t wakeup = sleep & ~(0x10); // set sleep bit low
  pca9685_write_u8(handle, PCA_REG_MODE1, wakeup);
}

void pca9685_setPrescaleFreq(pca9685_handle_t *handle, float freq) {

  // Range output modulation frequency is dependant on oscillator
  if (freq < 1)
    freq = 1;
  if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescalevel = ((handle->osc_freq / (freq * 4096.0)) + 0.5) - 1;
  if (prescalevel < 3)
    prescalevel = 3;
  if (prescalevel > 255)
    prescalevel = 255;
  uint8_t prescale = (uint8_t)prescalevel;


  uint8_t oldmode;
  pca9685_read_u8(handle, PCA_REG_MODE1, &oldmode);
  uint8_t newmode = (oldmode & ~0x80) | 0x10; // sleep
  pca9685_write_u8(handle, PCA_REG_MODE1, newmode);                             // go to sleep
  pca9685_write_u8(handle, PCA_PRESCALE, prescale); // set the prescaler
  pca9685_write_u8(handle, PCA_REG_MODE1, oldmode);
  HAL_Delay(10);
  // This sets the MODE1 register to turn on auto increment.
  pca9685_write_u8(handle, PCA_REG_MODE1, oldmode | 0x80 | 0x20);

}

void pca9685_setOscFreq(pca9685_handle_t *handle, uint32_t freq) {
	handle->osc_freq = freq;
}

void pca9685_setPWM(pca9685_handle_t *handle, uint8_t num, uint16_t on, uint16_t off) {

  pca9685_write_u8(handle, PCA_LEDx_REG_START+ 4*num, on);
  pca9685_write_u8(handle, PCA_LEDx_REG_START+1+ 4*num, on >> 8);
  pca9685_write_u8(handle, PCA_LEDx_REG_START+2+ 4*num, off);
  pca9685_write_u8(handle, PCA_LEDx_REG_START+3+ 4*num, off >> 8);
}

