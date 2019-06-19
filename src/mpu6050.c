#include <mpu6050.h>
#include "diag/Trace.h"

//各坐标轴上静止偏差（重力，角速度）
int16_t offsetAccelX = -195;
int16_t offsetAccelY = 560;
int16_t offsetAccelZ = -169;

int16_t offsetGyroX = 12;
int16_t offsetGyroY = 33;
int16_t offsetGyroZ = 4;

RAW_DATA raw_acc;
RAW_DATA raw_gyro;

RAW_DATA filter_acc;	//窗口滑动后对滤波数据
RAW_DATA filter_gyro;	//窗口滑动后对滤波数据

float angle_pitch, angle_roll, angle_yaw;
float angle_pitch_acc, angle_roll_acc;

#define N 200	//保存最近对128个数值

int16_t xAccWindow[N]; //N相当于选定一个窗口大小，对窗口数据做平均！
int16_t yAccWindow[N]; //N相当于选定一个窗口大小，对窗口数据做平均！
int16_t zAccWindow[N]; //N相当于选定一个窗口大小，对窗口数据做平均！

int16_t xGyroWindow[N]; //N相当于选定一个窗口大小，对窗口数据做平均！
int16_t yGyroWindow[N]; //N相当于选定一个窗口大小，对窗口数据做平均！
int16_t zGyroWindow[N]; //N相当于选定一个窗口大小，对窗口数据做平均！

uint8_t xAcci = 0, yAcci = 0, zAcci = 0;
uint8_t xGyroi = 0, yGyroi = 0, zGyroi = 0;

int16_t filter(int16_t *window, uint8_t *i, int16_t input) {
	uint8_t count;
	int64_t sum = 0;
	window[(*i)++] = input;
	if ((*i) == N)
		(*i) = 0; //当数据大于数组长度，替换数据组的一个数据  相当于环形队列更新，先进先出！
	for (count = 0; count < N; count++)
		sum = window[count];
	return (int16_t) (sum / N);
}



void mpu_fetchdata() {

	int16_t xAcc = getAccelValue('X');
	int16_t yAcc = getAccelValue('Y');
	int16_t zAcc = getAccelValue('Z');

	int16_t xGyro = getGyroValue('X');
	int16_t yGyro = getGyroValue('Y');
	int16_t zGyro = getGyroValue('Z');

	//update struct data
	raw_acc.x = xAcc;
	raw_acc.y = yAcc;
	raw_acc.z = zAcc;
	raw_gyro.x = xGyro;
	raw_gyro.y = yGyro;
	raw_gyro.z = zGyro;

	//使用滑动窗口滤波
	filter_acc.x = filter(xAccWindow, &xAcci, xAcc);
	filter_acc.y = filter(yAccWindow, &yAcci, yAcc);
	filter_acc.z = filter(zAccWindow, &zAcci, zAcc);

	filter_gyro.x = filter(xGyroWindow, &xGyroi, xGyro);
	filter_gyro.y = filter(yGyroWindow, &yGyroi, yGyro);
	filter_gyro.z = filter(zGyroWindow, &zGyroi, zGyro);
}

void init_mpu() {

	MX_I2C1_Init();

	WriteI2C_OneByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);    //解除休眠状态

	WriteI2C_OneByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);		//// sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz

	WriteI2C_OneByte(MPU6050_ADDRESS, CONFIGL, 0x00);		//不使用内部低通滤波

	/**
	 *  0x18	【不做三轴自检，陀螺仪满量程范围 ±2000 dps】	16.384 = 2^16 / 4000
	 *  0x10	【不做三轴自检，陀螺仪满量程范围 ±1000 dps】	32.768 = 2^16 / 2000
	 *  0x08	【不做三轴自检，陀螺仪满量程范围 ±500 dps】	65.536 = 2^16 / 1000
	 *  0x00 	【不做三轴自检，陀螺仪满量程范围 ±250 dps】	131.072 = 2^16 / 500
	 */
	WriteI2C_OneByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x08);	// Gryo scale 250 dps

	/**
	 *  0x18	【不做三轴自检，加速度满量程范围 ±16g】	2048LSB/g    = 2^16 / 32	 量程范围 16， 有效位数16位， 表示精度， 量程越大，精度越低
	 *  0x10	【不做三轴自检，加速度满量程范围 ±8g】	4096LSB/g    = 2^16 / 16
	 *  0x08	【不做三轴自检，加速度满量程范围 ±4g】	8192LSB/g	 = 2^16 / 8
	 *  0x00 	【不做三轴自检，加速度满量程范围 ±2g】	16384LSB/g	 = 2^16 / 4
	 */
	WriteI2C_OneByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00);	// Accel scale 8g (4096 LSB/g)

	HAL_Delay(500);

	uint8_t whoami = ReadI2C_OneByte(MPU6050_ADDRESS, 0x75);

	if (whoami == MPU6050_PRODUCT_ID) {
		trace_puts("Init MPU OK");
	}

	int16_t sum_x = 0;
	int16_t sum_y = 0;
	int16_t sum_z = 0;

	int count = 2000;

	//获取陀螺仪的校准偏差
	trace_puts("start Gyro calibration\n");
	for (int i = 0; i < count; i++) {
		int16_t x = getGyroValue('X');
		int16_t y = getGyroValue('Y');
		int16_t z = getGyroValue('Z');

		sum_x += x;
		sum_y += y;
		sum_z += z;
		HAL_Delay(3);
		//trace_puts(". ");
	}
	trace_puts("Gyro calibration OK\n");

	offsetGyroX = sum_x / count;
	offsetGyroY = sum_y / count;
	offsetGyroZ = sum_z / count;

	sum_z = sum_x = sum_y = 0;

	//获取加速度的校准偏差
	trace_puts("start Acc calibration\n");
	for (int i = 0; i < count; i++) {
		int16_t x = getAccelValue('X');
		int16_t y = getAccelValue('Y');
		int16_t z = getAccelValue('Z');

		sum_x += x;
		sum_y += y;
		sum_z += z;
		HAL_Delay(3);
		//trace_puts(". ");
	}
	trace_puts("Acc calibration OK\n");

	offsetAccelX = sum_x / count;
	offsetAccelY = sum_y / count;
	offsetAccelZ = sum_z / count;

	trace_printf("Acc offset: (%d, %d, %d), Gyro offset:(%d, %d, %d)\n", offsetAccelX, offsetAccelY, offsetAccelZ, offsetGyroX, offsetGyroY, offsetGyroZ);
}

//**************************************
//合成数据
//**************************************
static int16_t getMPUOutValue(uint8_t REG_Address) {
	int16_t result;
	uint8_t H, L;
	H = ReadI2C_OneByte(MPU6050_ADDRESS, REG_Address);
	L = ReadI2C_OneByte(MPU6050_ADDRESS, REG_Address + 1);
	result = (H << 8) + L;
	return result;   //合成数据
}

//**************************************
//取某一轴上的加速度数据
//**************************************
int16_t getAccelValue(char axis) {
	int16_t result = 0;
	switch (axis) {
	case 'x':
	case 'X': {
		result = getMPUOutValue(ACCEL_XOUT_H) - offsetAccelX;
	}
		break;
	case 'y':
	case 'Y': {
		result = getMPUOutValue(ACCEL_YOUT_H) - offsetAccelY;
	}
		break;
	case 'z':
	case 'Z': {
		result = getMPUOutValue(ACCEL_ZOUT_H) - offsetAccelZ;
	}
		break;
	}
	return result;
}

//**************************************
//取某一轴上的角速度数据
//**************************************
int16_t getGyroValue(char axis) {
	int16_t result = 0;
	switch (axis) {
	case 'x':
	case 'X': {
		result = getMPUOutValue(GYRO_XOUT_H) - offsetGyroX;
	}
		break;
	case 'y':
	case 'Y': {
		result = getMPUOutValue(GYRO_YOUT_H) - offsetGyroY;
	}
		break;
	case 'z':
	case 'Z': {
		result = getMPUOutValue(GYRO_ZOUT_H) - offsetGyroZ;
	}
		break;
	}
	return result;
}
