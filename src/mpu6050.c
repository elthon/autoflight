#include <mpu6050.h>
#include "diag/Trace.h"


//各坐标轴上静止偏差（重力，角速度）
int16_t offsetAccelX = -195;
int16_t offsetAccelY = 560;
int16_t offsetAccelZ = -169;

int16_t offsetGyroX = 12;
int16_t offsetGyroY = 33;
int16_t offsetGyroZ = 4;


void init_mpu() {

	MX_I2C1_Init();

	WriteI2C_OneByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);    //解除休眠状态

	WriteI2C_OneByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);		//// sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz

	WriteI2C_OneByte(MPU6050_ADDRESS, CONFIGL, 0x06);		//内部低通滤波频率，98hz

	WriteI2C_OneByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);		//1000deg/s

	WriteI2C_OneByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x01);	// Accel scale 8g (4096 LSB/g)

	HAL_Delay(500);

	uint8_t whoami = ReadI2C_OneByte(MPU6050_ADDRESS, 0x75);

	if(whoami == MPU6050_PRODUCT_ID){
		trace_puts("Init MPU OK");
	}

	uint16_t sum_x = 0;
	uint16_t sum_y = 0;
	uint16_t sum_z = 0;

	//获取加速度的校准偏差
	for(int i=0; i<100; i++){
		uint16_t x = getAccelValue('X');
		uint16_t y = getAccelValue('Y');
		uint16_t z = getAccelValue('Z');

		sum_x += x;
		sum_y += y;
		sum_z += z;
		HAL_Delay(10);
	}

	offsetAccelX = sum_x / 1000;
	offsetAccelY = sum_y / 1000;
	offsetAccelZ = sum_z / 1000;

	trace_printf("Acc offset: (%d, %d, %d)\n", offsetAccelX, offsetAccelY, offsetAccelZ);
}

//**************************************
//合成数据
//**************************************
static int16_t getMPUOutValue(uint8_t REG_Address)
{
    int16_t result;
    uint8_t H,L;
    H=ReadI2C_OneByte(MPU6050_ADDRESS,REG_Address);
    L=ReadI2C_OneByte(MPU6050_ADDRESS,REG_Address+1);
    result = (H<<8)+L;
    return result;   //合成数据
}

//**************************************
//取某一轴上的加速度数据
//**************************************
int16_t getAccelValue(char axis)
{
    int16_t result = 0;
    switch(axis)
    {
        case 'x':
        case 'X':
        {
            result = getMPUOutValue(ACCEL_XOUT_H) - offsetAccelX;
        }
        break;
        case 'y':
        case 'Y':
        {
            result = getMPUOutValue(ACCEL_YOUT_H) - offsetAccelY;
        }
        break;
        case 'z':
        case 'Z':
        {
            result = getMPUOutValue(ACCEL_ZOUT_H) - offsetAccelZ;
        }
        break;
    }
    return result;
}

//**************************************
//取某一轴上的角速度数据
//**************************************
int16_t getGyroValue(char axis)
{
    int16_t result = 0;
    switch(axis)
    {
        case 'x':
        case 'X':
        {
            result = getMPUOutValue(GYRO_XOUT_H) - offsetGyroX;
        }
        break;
        case 'y':
        case 'Y':
        {
            result = getMPUOutValue(GYRO_YOUT_H) - offsetGyroY;
        }
        break;
        case 'z':
        case 'Z':
        {
            result = getMPUOutValue(GYRO_ZOUT_H) - offsetGyroZ;
        }
        break;
    }
    return result;
}
