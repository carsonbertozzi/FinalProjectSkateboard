#include "msp.h"
#include "delay.h"
#include "ADC.h"
#include "USB_UART.h"
#include <stdio.h>
#define PWM_MIN 12000
#define PWM_MAX 22200
#define WEIGHT_SPLIT_MIN -.7
#define WEIGHT_SPLIT_MAX .7

/**
 * main.c
 */


uint16_t throttle_diff;
uint16_t sampled_flag;



void main(void)
{
    uint16_t back_input, front_input, pwm_output;
    float weight_split, slope = 1.0 * (PWM_MAX - PWM_MIN) / (WEIGHT_SPLIT_MAX - WEIGHT_SPLIT_MIN);

//    uint16_t mod_throttle_diff;
//    int weight_diff;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    setDCO(FREQ_12MHz);

    ADCInit();

    USB_UARTInit();


    P2->SEL0 |= (BIT6 | BIT4); //setting throttle and turning inputs
    P2->SEL1 &= ~(BIT6);
    P2->DIR &= ~(BIT6);
    P2->DIR |= BIT4;


    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__CONTINUOUS
                   | TIMER_A_CTL_CLR;

    TIMER_A0->CCTL[1]  |= TIMER_A_CCTLN_OUTMOD_7;

    TIMER_A0->CCTL[3] |= TIMER_A_CCTLN_CM__BOTH | TIMER_A_CCTLN_CCIS__CCIA
                       | TIMER_A_CCTLN_CAP | TIMER_A_CCTLN_CCIE
                       | TIMER_A_CCTLN_SCS;

    __enable_irq();
    NVIC->ISER[0] = (1 << TA0_N_IRQn);

    while(1){

        ADC14->CTL0 |= ADC14_CTL0_SC; //gets one back_input
        while (!getADCFlag());
        clearADCFlag();
        back_input = getADCSample();
        front_input = getADCSample_1();
//        giveUARTInt(sample);
//        giveUARTString(" ");
//        giveUARTInt(front_input);
//        giveUARTString("\r");


        if (sampled_flag){
            //the flags ensure that the data is only edited all in one go,
            //instead of potentially sporadically changing based whenever

            weight_split = (float)(front_input - back_input) / (front_input + back_input);

            if (weight_split > WEIGHT_SPLIT_MAX){
                weight_split = WEIGHT_SPLIT_MAX;
            }

            else if (weight_split < WEIGHT_SPLIT_MIN){
                weight_split = WEIGHT_SPLIT_MIN;
            }

            pwm_output = (int)PWM_MIN + slope * (weight_split - WEIGHT_SPLIT_MIN);

            if (front_input + back_input < 1000){ //sets a minimum weight threshold
                TIMER_A0->CCR[1] = 18000;
            }

            else if (throttle_diff > pwm_output){
                TIMER_A0->CCR[1] = pwm_output;
            }

            else{
                TIMER_A0->CCR[1] = throttle_diff;
            }



//            mod_throttle_diff = throttle_diff - 11000;//makes throttle input a range from 0-10000
//            weight_diff = (((((front_input - back_input) * 100) / (front_input + back_input)) + 17) * 150); //ask vishnu to see the picture on his phone
//            //above is a transformation that accepts all weight and converts the ADC difference values to between 0-10000
//            if (front_input + back_input < 1000){
//                TIMER_A0->CCR[1] = 18000;
//            }
//            else if (weight_diff < 0){
//                TIMER_A0->CCR[1] = 12500;
//            }
//            else if (mod_throttle_diff > weight_diff){
//               TIMER_A0->CCR[1] = weight_diff + 12000;
//            }
//
//            else {
//                TIMER_A0->CCR[1] = throttle_diff;
//            }


            //giveUARTInt(throttle_diff);
            //giveUARTString(" ");
            //giveUARTInt(weight_diff);
            //giveUARTString("\r");

            sampled_flag = 0;

        }
    }
}

void TA0_N_IRQHandler(void){
    static uint16_t turnRising, turnFalling;
    static uint16_t turnCaps = 0;

    if (TIMER_A0->CCTL[3] & TIMER_A_CCTLN_CCIFG){
        if (turnCaps == 0){
            turnRising = TIMER_A0->CCR[3];
            turnCaps++;
        }
        else if (turnCaps == 1){
            turnFalling = TIMER_A0->CCR[3];
            throttle_diff = turnFalling - turnRising;
            sampled_flag = 1;
            turnCaps = 0;
        }
    }
    TIMER_A0->CCTL[3] &= ~TIMER_A_CCTLN_CCIFG;
}
