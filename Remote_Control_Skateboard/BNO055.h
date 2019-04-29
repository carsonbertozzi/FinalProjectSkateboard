/*
 * BNO055.h
 *
 *  Created on: Apr 24, 2019
 *      Author: carsonbertozzi
 */

#ifndef BNO055_H_
#define BNO055_H_

#include "msp.h"
#include "delay.h"


#define BNO055_ADDRESS 0x28

void InitBNO055(uint8_t DeviceAddress);
void WriteBNO055(uint8_t MemAddress, uint8_t MemByte);
uint8_t ReadBNO055(uint8_t MemAddress);

#endif /* BNO055_H_ */
