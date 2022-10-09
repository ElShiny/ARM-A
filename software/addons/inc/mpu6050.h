#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

/*
 * mpu6050.h
 *
 *  Created on: Oct 7, 2022
 *      Author: Matej
 */



#include "stm32g4xx_hal.h"
#include "main.h"



#define MPU6050_ADDR (0x68 << 1)

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define CALIB_OFFSET_REPEAT 500

#define MPU6050_I2C_PORT hi2c2


//#define RAW_IS_RESCALED


typedef enum
{

	MPU_X, MPU_Y, MPU_Z, MPU_NUM_OF_AXES

} mpu_axes_enum_t;


typedef struct{

	uint8_t mpu_addr;
	int16_t accel_raw[MPU_NUM_OF_AXES];
	int16_t gyro_raw[MPU_NUM_OF_AXES];

	int16_t accel_offset[MPU_NUM_OF_AXES];
	int16_t gyro_offset[MPU_NUM_OF_AXES];

	int16_t accel_call[MPU_NUM_OF_AXES];
	int16_t gyro_call[MPU_NUM_OF_AXES];

	float accel_angle[MPU_NUM_OF_AXES];
	float gyro_angle[MPU_NUM_OF_AXES];

	float yaw, pitch, roll;

}mpu_handle_t;

mpu_handle_t hand1_mpu;



void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Calc_Offsets(uint8_t is_calc_acc, uint8_t is_calc_gyro);
void MPU6050_Read_All(void);
void EXTIValue_MPU_callback(void);


#endif /* INC_MPU6050_H_ */
