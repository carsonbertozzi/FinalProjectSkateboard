/*
 * ADC.h
 *
 *  Created on: May 9, 2018
 *      Author: Vishnu
 */

#ifndef ADC_H_
#define ADC_H_

#include "msp.h"

#define ADC_V_MAX 3.3
#define RMS_SAMPLE_36 36
#define ADC_RESOLUTION 16384
#define ECCFF 495 //Extremely Carefully Calculated Fudge Factor

static uint8_t ADCFlag = 0;
static uint16_t ADCSample = 0;
static uint16_t ADCSample_1 = 0;

void ADCInit(void);
void clearADCFlag(void);
uint8_t getADCFlag(void);
uint16_t getADCSample(void);
uint16_t getADCSample_1(void);


#endif /* ADC_H_ */
