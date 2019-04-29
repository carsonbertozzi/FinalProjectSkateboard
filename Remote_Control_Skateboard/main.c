#include "msp.h"
#include "delay.h"
#include "ADC.h"
#include "USB_UART.h"
#include "BNO055.h"
#include <stdio.h>
#define PWM_MIN 12000
#define PWM_MAX 22200
#define MAX_DIFF_ACCEL 65
#define MAX_DIFF_DECEL 90

/**
 * main.c
 */


uint16_t throttle_diff;
uint16_t sampled_flag;



void main(void)
{

    float WEIGHT_SPLIT_MIN = -.55;
    float WEIGHT_SPLIT_MAX = .35;

    int16_t signed_int_gyro_value;
    uint16_t back_input, front_input, calc_pwm_output, prev_pwm_output = PWM_MIN, concat_gyro_value;
    uint8_t low_gyro_value, high_gyro_value;
    float weight_split, acc_weight_shift, slope = 1.0 * (PWM_MAX - PWM_MIN) / (WEIGHT_SPLIT_MAX - WEIGHT_SPLIT_MIN);

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    delay_ms(1000,FREQ_3MHz);


    setDCO(FREQ_12MHz);

    InitBNO055(BNO055_ADDRESS);

    WriteBNO055(0x3E, 0x00);
    WriteBNO055(0x3B, 0x00);
    WriteBNO055(0x3D, 0x0C);
    delay_ms(7,FREQ_12MHz);

    ADCInit();

    USB_UARTInit();

    /* Timer input capture pins
     * P2.6 is capture, P2.4 is PWM out for V-Controllers
     */
    P2->SEL0 |= (BIT6 | BIT4); //setting PWM input and output
    P2->SEL1 &= ~(BIT6);
    P2->DIR &= ~(BIT6);
    P2->DIR |= BIT4;


    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__CONTINUOUS
                   | TIMER_A_CTL_CLR;

    TIMER_A0->CCTL[1]  |= TIMER_A_CCTLN_OUTMOD_7; //set pwm output

    TIMER_A0->CCTL[3]  |= TIMER_A_CCTLN_CM__BOTH | TIMER_A_CCTLN_CCIS__CCIA
                       | TIMER_A_CCTLN_CAP | TIMER_A_CCTLN_CCIE
                       | TIMER_A_CCTLN_SCS; //set interrupts and timer capture for pwm input

    __enable_irq();
    NVIC->ISER[0] = (1 << TA0_N_IRQn);



    while(1){


        WEIGHT_SPLIT_MIN = -.55;
        WEIGHT_SPLIT_MAX = .35;

        ADC14->CTL0 |= ADC14_CTL0_SC; //gets one front and back input
        while (!getADCFlag());
        clearADCFlag();
        back_input = getADCSample();
        front_input = getADCSample_1();


        low_gyro_value = ReadBNO055(0x1C);     // Read value from BNO055
        delay_ms(1,FREQ_12MHz);
        high_gyro_value = ReadBNO055(0x1D);
        concat_gyro_value = high_gyro_value << 8 | low_gyro_value;
        signed_int_gyro_value = (int16_t)concat_gyro_value;

        //1300 - -1300 is the complete range
        //guesstimate is +-300
        //divide to make the range shift the range appropiately by .45 so the maximum never exceeds +-1
        acc_weight_shift = (float) signed_int_gyro_value / 667;
        if (acc_weight_shift > .45)
            acc_weight_shift = .45;
        else if (acc_weight_shift < -.45)
            acc_weight_shift = -.45;
        WEIGHT_SPLIT_MAX += acc_weight_shift;
        WEIGHT_SPLIT_MIN += acc_weight_shift;

//        giveUARTPosNegInt((int16_t) WEIGHT_SPLIT_MAX * 100);
//        giveUARTString(" : ");
//        giveUARTPosNegInt((int16_t) WEIGHT_SPLIT_MIN * 100);
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

            calc_pwm_output = (int)PWM_MIN + slope * (weight_split - WEIGHT_SPLIT_MIN);
            if (calc_pwm_output > prev_pwm_output + MAX_DIFF_ACCEL){
                calc_pwm_output = prev_pwm_output + MAX_DIFF_ACCEL;
            }


            else if (calc_pwm_output < prev_pwm_output - MAX_DIFF_DECEL){
                calc_pwm_output = prev_pwm_output - MAX_DIFF_DECEL;
            }

            if (front_input + back_input < 1000){ //sets a minimum weight threshold
                TIMER_A0->CCR[1] = 18000;
                calc_pwm_output = 18000;
            }

            else if (throttle_diff > calc_pwm_output){
                TIMER_A0->CCR[1] = calc_pwm_output;
            }

            else{
                TIMER_A0->CCR[1] = throttle_diff;
            }

            prev_pwm_output = calc_pwm_output;
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
