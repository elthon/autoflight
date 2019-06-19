/*
 * asp.h
 *
 *  Created on: Jun 19, 2019
 *      Author: elthon
 */

#ifndef ASP_H_
#define ASP_H_


typedef enum{
	SENSOR,		//发送原始的sensor数据
	ANGLE,		//发送计算后的角度数据
	ALL,		//发送所有数据
}SEND_TYPE;

void data_exchange(SEND_TYPE type);


#endif /* ASP_H_ */
