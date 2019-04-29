/*
 * delay.c
 *
 *  Created on: Apr 9, 2018
 *      Author: Vishnu
 */

#include "msp.h"
#include "delay.h"
#include <stdint.h>


void delay_ms(int msDelay, int freq){

    uint32_t i = freq / 10000 * msDelay;

    for ( ; i > 0; i--);

}

void delay_us(int usDelay, int freq){

    uint32_t i = 0;
    if (usDelay < 20){
        switch(freq){
        case FREQ_1500KHz:
        case FREQ_3MHz:
            i = freq / 10000 * usDelay / 1000 / 50;
            break;
        case FREQ_6MHz:
            i = freq / 10000 * usDelay / 1000 / 5;
            break;
        case FREQ_12MHz:
            i = freq / 10000 * usDelay / 1000 - 6 ;
            break;
        case FREQ_24MHz:
        case FREQ_48MHz:
            i = freq / 10000 * usDelay / 1000 - 4;
            break;
        }
    }
    else if (usDelay < 40 && usDelay >= 20){
        i = freq / 10000 * usDelay / 1000 - 6;
    }
    else if (usDelay <= 500 && usDelay >= 40){
        i = freq / 10000 * usDelay / 1000 - 5;
    }
    else{
        i = freq / 10000 * usDelay / 1000 + 1;
    }
    for ( ; i > 0; i--);

}

int setDCO(int freq){
    uint32_t hold;
    CS->KEY = CS_KEY_VAL; // unlock CS registers
    hold = CS->CTL0;
    CS->CTL0 = 0; // clear register CTL0

    switch(freq){

    case FREQ_1500KHz:
        CS->CTL0 |= CS_CTL0_DCORSEL_0;
        break;
    case FREQ_3MHz:
        CS->CTL0 |= CS_CTL0_DCORSEL_1;
        break;
    case FREQ_6MHz:
        CS->CTL0 |= CS_CTL0_DCORSEL_2;
        break;
    case FREQ_12MHz:
        CS->CTL0 |= CS_CTL0_DCORSEL_3;
        break;
    case FREQ_24MHz:
        CS->CTL0 |= CS_CTL0_DCORSEL_4;
        break;
    case FREQ_48MHz:
        /* Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */
        while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));

        PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;

        while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));

        /* Configure Flash wait-state to 1 for both banks 0 & 1 */
        FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL &
         ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) | FLCTL_BANK0_RDCTL_WAIT_1;

        FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL &
         ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) | FLCTL_BANK1_RDCTL_WAIT_1;

        CS->CTL0 |= CS_CTL0_DCORSEL_5;

        break;
    default:
        CS->CTL0 = hold; //If value is not valid, put back original value
        CS->KEY = 0; //Lock the CS register
        return -1;
    }

    CS->CTL1 = CS_CTL1_SELM__DCOCLK | CS_CTL1_SELS__DCOCLK | CS_CTL1_SELA__REFOCLK; //Set the SMCLK signal to the DCO clock
    CS->KEY = 0;

    return 0;
}
