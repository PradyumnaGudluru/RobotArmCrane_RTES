/*
 * mpu6050.h
 *
 *  Created on: Apr 14, 2022
 *      Author: vishn
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

#define SLAVE_ADDRESS       (0x68)

void vMPU6050_Init(void);
void vMPU6050_Write(uint8_t ui8Reg, uint8_t* ui8WrBuff, uint8_t ui8WrBuffLen);
void vMPU6050_Read(uint8_t ui8Reg, uint8_t* ui8RdBuff, uint8_t ui8RdBuffLen);



#endif /* MPU6050_H_ */
