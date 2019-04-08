/*
 * ADC.c
 *
 *  Created on: May 9, 2018
 *      Author: Vishnu
 */

#include "ADC.h"


void ADCInit(void){
    P5->SEL0 |= BIT4 | BIT5;
    P5->SEL1 |= BIT4 | BIT5;

    ADC14->CTL0 &= ~ADC14_CTL0_ENC;
    ADC14->CTL0 = ADC14_CTL0_SHT0__16 | ADC14_CTL0_ON | ADC14_CTL0_SHP | ADC14_CTL0_CONSEQ_1; //sample for 16 clock cycles, turn on, sample manually, default clock, sequence of channels
    ADC14->CTL1 = ADC14_CTL1_RES_3; //set resolution to 14 bits
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_1; //hook up A1, save in MEM0
    ADC14->MCTL[1] = ADC14_MCTLN_INCH_0 | ADC14_MCTLN_EOS; //hook up A0, save in MEM1

    ADC14->IER0 |= ADC14_IER0_IE0 | ADC14_IER0_IE1; //enables interrupt for MEM[0] and MEM[1]
    ADC14->CTL0 |= ADC14_CTL0_ENC;

    __enable_irq();
    NVIC->ISER[0] = (1 << ADC14_IRQn);
}

void clearADCFlag(void){
    ADCFlag = 0;
}

uint16_t getADCSample(void){
    return ADCSample;
}

uint16_t getADCSample_1(void){
    return ADCSample_1;
}

uint8_t getADCFlag(void){
    return ADCFlag;
}

void ADC14_IRQHandler(){
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG0){
        ADCSample = ADC14->MEM[0];
        ADCFlag = 1;
    }
    else if (ADC14->IFGR0 & ADC14_IFGR0_IFG1){
        ADCSample_1 = ADC14->MEM[1];
        ADCFlag = 1;
    }
}
