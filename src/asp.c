/*
 * asp.c
 *
 *	Autoflight Serial Protocol 串口协议
 *  Created on: Jun 19, 2019
 *      Author: elthon
 */
#include "asp.h"
#include "mpu6050.h"
#include "usbd_cdc_if.h"
#include "diag/Trace.h"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

static uint8_t head_pack_u1(uint8_t *buff) {

	uint8_t cnt = 0;
	buff[cnt++] = 0xAA;		//帧头
	buff[cnt++] = 0x01;		//源设备地址
	buff[cnt++] = 0xAF;		//目标设备地址（上位机）

	buff[cnt++] = 0xF1;	//用户数据1

	return cnt;
}

static void send_angle(const float angle_pitch, const float angle_roll) {
	static uint8_t buff[256] = { 0 };

	uint8_t cnt = head_pack_u1(buff);

	buff[cnt++] = 4;		//数据长度 : 4个字节

	int16_t pitch = (int16_t)(angle_pitch * 1000);
	buff[cnt++] = BYTE1(pitch);
	buff[cnt++] = BYTE0(pitch);

	int16_t roll = (int16_t)(angle_roll * 1000);
	buff[cnt++] = BYTE1(roll);
	buff[cnt++] = BYTE0(roll);

	uint8_t sum = 0;
	for (int i = 0; i < cnt; i++) {
		sum += buff[i];
	}

	buff[cnt++] = sum;

	CDC_Transmit_FS(buff, cnt);
}

static void send_raw_acc_gyro(const RAW_DATA *acc, const RAW_DATA *gyro) {

	static uint8_t buff[256] = { 0 };

	uint8_t cnt = head_pack_u1(buff);

	buff[cnt++] = 26;		//数据长度 : 12个字节

	buff[cnt++] = BYTE1(acc->x);
	buff[cnt++] = BYTE0(acc->x);

	buff[cnt++] = BYTE1(acc->y);
	buff[cnt++] = BYTE0(acc->y);

	buff[cnt++] = BYTE1(acc->z);
	buff[cnt++] = BYTE0(acc->z);

	buff[cnt++] = BYTE1(gyro->x);
	buff[cnt++] = BYTE0(gyro->x);

	buff[cnt++] = BYTE1(gyro->y);
	buff[cnt++] = BYTE0(gyro->y);

	buff[cnt++] = BYTE1(gyro->z);
	buff[cnt++] = BYTE0(gyro->z);

	int32_t pitch = (int32_t)(angle_pitch * 1000);
	buff[cnt++] = BYTE3(pitch);
	buff[cnt++] = BYTE2(pitch);
	buff[cnt++] = BYTE1(pitch);
	buff[cnt++] = BYTE0(pitch);

	int32_t roll = (int32_t)(angle_roll * 1000);
	buff[cnt++] = BYTE3(roll);
	buff[cnt++] = BYTE2(roll);
	buff[cnt++] = BYTE1(roll);
	buff[cnt++] = BYTE0(roll);

	buff[cnt++] = BYTE1(filter_acc.x);
	buff[cnt++] = BYTE0(filter_acc.x);

	buff[cnt++] = BYTE1(filter_acc.y);
	buff[cnt++] = BYTE0(filter_acc.y);

	buff[cnt++] = BYTE1(filter_acc.z);
	buff[cnt++] = BYTE0(filter_acc.z);
	uint8_t sum = 0;
	for (int i = 0; i < cnt; i++) {
		sum += buff[i];
	}

	buff[cnt++] = sum;

	CDC_Transmit_FS(buff, cnt);
}

void data_exchange(SEND_TYPE type) {

	static uint32_t acc_count = 0;
	static uint32_t angle_count = 0;

	acc_count++;
	angle_count++;

	if (type == SENSOR) {
		if (acc_count > 50) {	//100ms
			send_raw_acc_gyro(&raw_acc, &raw_gyro);
			acc_count = 0;
		}
	}

	if (type == ANGLE) {
		if (acc_count > 40) {	//1000ms
			send_angle(angle_pitch, angle_roll);
			acc_count = 0;
		}
	}

}

