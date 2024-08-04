/*
 * mpu6050.c
 *
 *  Created on: Oct 7, 2022
 *      Author: Matej
 */

#include "mpu6050.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>



I2C_HandleTypeDef MPU6050_I2C_PORT;




void MPU6050_Init(void)
{
	uint8_t check = 0;
	uint8_t Data;

	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);

	hand1_mpu.mpu_addr = MPU6050_ADDR;


	HAL_I2C_Mem_Read (&MPU6050_I2C_PORT, hand1_mpu.mpu_addr ,WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);

	printf("check: %x\r\n", check);

	if (check == 0x98)
	{
		printf("mpu6050 exists\r\n");
		// power management
		Data = 0;
		HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, PWR_MGMT_1_REG, 1,&Data, 1, HAL_MAX_DELAY);

		//CONFIG
		Data = 0x03;
		HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, 0x1A, 1, &Data, 1, HAL_MAX_DELAY);

		//SMPLRT_DIV
		Data = 19;
		HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, SMPLRT_DIV_REG, 1, &Data, 1, HAL_MAX_DELAY);

		//ACCEL_CONFIG
		Data = 1<<3;
		HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, ACCEL_CONFIG_REG, 1, &Data, 1, HAL_MAX_DELAY);

		//GYRO_CONFIG
		Data = 1<<3;
		HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, GYRO_CONFIG_REG, 1, &Data, 1, HAL_MAX_DELAY);

		//INT_CONFIG
		Data = 0b10110000;
		HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, 0x37, 1, &Data, 1, HAL_MAX_DELAY);

		//INT_ENABLE
		Data = 0x01;
		HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, 0x38, 1, &Data, 1, HAL_MAX_DELAY);


		printf("MPU init finished\r\n");
	}


	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
	MPU6050_Read_All();

}


void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, HAL_MAX_DELAY);

	hand1_mpu.accel_raw[MPU_X] = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	hand1_mpu.accel_raw[MPU_Y] = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	hand1_mpu.accel_raw[MPU_Z] = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	hand1_mpu.accel_call[MPU_X] = hand1_mpu.accel_raw[MPU_X] - hand1_mpu.accel_offset[MPU_X];
	hand1_mpu.accel_call[MPU_Y] = hand1_mpu.accel_raw[MPU_Y] - hand1_mpu.accel_offset[MPU_Y];
	hand1_mpu.accel_call[MPU_Z] = hand1_mpu.accel_raw[MPU_Z] - hand1_mpu.accel_offset[MPU_Z];

#ifdef RAW_IS_RESCALED
	hand1_mpu.accel_raw[MPU_X] = hand1_mpu.accel_raw[MPU_X]/16384.0;
	hand1_mpu.accel_raw[MPU_Y] = hand1_mpu.accel_raw[MPU_Y]/16384.0;
	hand1_mpu.accel_raw[MPU_Z] = hand1_mpu.accel_raw[MPU_Z]/16384.0;
#endif

}


void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, GYRO_XOUT_H_REG, 1, Rec_Data, 6, HAL_MAX_DELAY);

	hand1_mpu.gyro_raw[MPU_X] = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	hand1_mpu.gyro_raw[MPU_Y] = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	hand1_mpu.gyro_raw[MPU_Z] = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	hand1_mpu.gyro_call[MPU_X] = hand1_mpu.gyro_raw[MPU_X] - hand1_mpu.gyro_offset[MPU_X];
	hand1_mpu.gyro_call[MPU_Y] = hand1_mpu.gyro_raw[MPU_Y] - hand1_mpu.gyro_offset[MPU_Y];
	hand1_mpu.gyro_call[MPU_Z] = hand1_mpu.gyro_raw[MPU_Z] - hand1_mpu.gyro_offset[MPU_Z];

#ifdef RAW_IS_RESCALED
	hand1_mpu.gyro_raw[MPU_X] = hand1_mpu.gyro_raw[MPU_X]/131.0;
	hand1_mpu.gyro_raw[MPU_Y] = hand1_mpu.gyro_raw[MPU_Y]/131.0;
	hand1_mpu.gyro_raw[MPU_Z] = hand1_mpu.gyro_raw[MPU_Z]/131.0;
#endif


}

void MPU6050_Calc_Offsets(uint8_t is_calc_acc, uint8_t is_calc_gyro)
 {
	int32_t ag[6] = { 0, 0, 0, 0, 0, 0 };

	//MPU6050_Read_All();

	for (int i = 0; i < CALIB_OFFSET_REPEAT; i++) {
		//MPU6050_Read_All();
		ag[0] += hand1_mpu.accel_raw[MPU_X];
		ag[1] += hand1_mpu.accel_raw[MPU_Y];
		ag[2] += (hand1_mpu.accel_raw[MPU_Z] - 16384);
		ag[3] += hand1_mpu.gyro_raw[MPU_X];
		ag[4] += hand1_mpu.gyro_raw[MPU_Y];
		ag[5] += hand1_mpu.gyro_raw[MPU_Z];
		HAL_Delay(1); // wait a little bit between 2 measurements
		//printf("c\r\n");
	}

	if (is_calc_acc) {
		hand1_mpu.accel_offset[MPU_X] = ag[0] / CALIB_OFFSET_REPEAT;
		hand1_mpu.accel_offset[MPU_Y] = ag[1] / CALIB_OFFSET_REPEAT;
		hand1_mpu.accel_offset[MPU_Z] = ag[2] / CALIB_OFFSET_REPEAT;
	}

	if (is_calc_gyro) {
		hand1_mpu.gyro_offset[MPU_X] = ag[3] / CALIB_OFFSET_REPEAT;
		hand1_mpu.gyro_offset[MPU_Y] = ag[4] / CALIB_OFFSET_REPEAT;
		hand1_mpu.gyro_offset[MPU_Z] = ag[5] / CALIB_OFFSET_REPEAT;
	}

	MPU6050_Read_All();

	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);
	printf("################################################################################\r\n");
	printf("gyro x OFF: %d, gyro y OFF: %d, gyro z OFF: %d,\r\n",
			hand1_mpu.gyro_offset[MPU_X], hand1_mpu.gyro_offset[MPU_Y],
			hand1_mpu.gyro_offset[MPU_Z]);

	printf("accel x OFF: %d, accel y OFF: %d, accel z OFF: %d,\r\n",
			hand1_mpu.accel_offset[MPU_X], hand1_mpu.accel_offset[MPU_Y],
			hand1_mpu.accel_offset[MPU_Z]);


	printf("gyro x call: %d, gyro y call: %d, gyro z call: %d,\r\n",
			hand1_mpu.gyro_call[MPU_X], hand1_mpu.gyro_call[MPU_Y],
			hand1_mpu.gyro_call[MPU_Z]);

	printf("accel x call: %d, accel y call: %d, accel z call: %d,\r\n",
			hand1_mpu.accel_call[MPU_X], hand1_mpu.accel_call[MPU_Y],
			hand1_mpu.accel_call[MPU_Z]);

	printf("################################################################################\r\n");
	printf("\r\n");

	hand1_mpu.yaw = 0;
	hand1_mpu.roll = 0;
	hand1_mpu.pitch = 0;
	hand1_mpu.accel_angle[MPU_X] = 0;
	hand1_mpu.accel_angle[MPU_Y] = 0;
	hand1_mpu.gyro_angle[MPU_X] = 0;
	hand1_mpu.gyro_angle[MPU_Y] = 0;

	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
	MPU6050_Read_All();
}

void MPU6050_Read_All(void)
{
//	uint8_t Rec_Data = 0;

	MPU6050_Read_Gyro();
	MPU6050_Read_Accel();
//
//	printf("gyro x: %d, gyro y: %d, gyro z: %d,\r\n", hand1_mpu.gyro_call[MPU_X], hand1_mpu.gyro_call[MPU_Y], hand1_mpu.gyro_call[MPU_Z]);
//
//	printf("accel x: %d, accel z: %d, accel z: %d,\r\n", hand1_mpu.accel_call[MPU_X], hand1_mpu.accel_call[MPU_Y], hand1_mpu.accel_call[MPU_Z]);
//
//	HAL_I2C_Mem_Read (&MPU6050_I2C_PORT, hand1_mpu.mpu_addr, 0x3A, 1, &Rec_Data, 1, HAL_MAX_DELAY);
//	printf("int: %x\r\n", Rec_Data);
//
//	printf("A\r\n");

}

void EXTIValue_MPU_callback(void) {

	MPU6050_Read_All();

	//haram calculations


	float sgZ = hand1_mpu.accel_call[MPU_Z]<0 ? -1 : 1;

	hand1_mpu.accel_angle[MPU_X] = (atan2f(hand1_mpu.accel_call[MPU_Y], sgZ * sqrtf(powf(hand1_mpu.accel_call[MPU_X], 2) + powf(hand1_mpu.accel_call[MPU_Z], 2))) * 57.29578);
	hand1_mpu.accel_angle[MPU_Y] = (atan2f(hand1_mpu.accel_call[MPU_X], sqrtf(powf(hand1_mpu.accel_call[MPU_Y], 2) + powf(hand1_mpu.accel_call[MPU_Z], 2))) * 57.29578);

	hand1_mpu.gyro_angle[MPU_X] = hand1_mpu.gyro_angle[MPU_X] + ((hand1_mpu.gyro_call[MPU_X] * 0.01)/131.0);
	hand1_mpu.gyro_angle[MPU_Y] = hand1_mpu.gyro_angle[MPU_Y] + ((hand1_mpu.gyro_call[MPU_Y] * 0.01)/131.0);

	hand1_mpu.yaw = hand1_mpu.yaw + ((hand1_mpu.gyro_call[MPU_Z] * 0.01)/131.0);
	hand1_mpu.roll = (0.96 * hand1_mpu.gyro_angle[MPU_X] + 0.04 * hand1_mpu.accel_angle[MPU_X]);
	hand1_mpu.pitch = 0.96 * hand1_mpu.gyro_angle[MPU_Y] + 0.04 * hand1_mpu.accel_angle[MPU_Y];


}
