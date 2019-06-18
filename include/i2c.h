/*
 * I2C.h
 *
 *  Created on: Jun 18, 2019
 *      Author: elthon
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void MX_I2C1_Init(void);

void WriteI2C_OneByte(uint16_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);
uint8_t ReadI2C_OneByte(uint16_t SlaveAddress, uint8_t REG_Address);

#endif /* I2C_H_ */
