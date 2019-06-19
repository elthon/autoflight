/*
 * MPU6050.h
 *
 *  Created on: Jun 18, 2019
 *      Author: elthon
 */

#ifndef MPU6050_H_
#define MPU6050_H_


#include "i2c.h"


/*******************************************************************************************************/
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIGL			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_ADDRESS	0x3B
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define GYRO_ADDRESS  	0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)

//-------------------------------6050系列---------------------------------
#define MPU6050_PRODUCT_ID 0x68
#define MPU6050A_PRODUCT_ID 0x98
#define MPU6052C_PRODUCT_ID 0x72

#define MPU6050_ADDRESS 0xD0//0x68

typedef struct{

	int16_t x;
	int16_t y;
	int16_t z;

} RAW_DATA;

extern RAW_DATA raw_acc;
extern RAW_DATA raw_gyro;

extern RAW_DATA filter_acc;
extern RAW_DATA filter_gyro;

extern float angle_pitch;
extern float angle_roll;
extern float angle_yaw;
extern float angle_pitch_acc;
extern float angle_roll_acc;

void mpu_fetchdata();
void init_mpu();
int16_t getAccelValue(char axis);
int16_t getGyroValue(char axis);



#endif /* MPU6050_H_ */
