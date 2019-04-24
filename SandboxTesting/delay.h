/*
 * delay.h
 *
 *  Created on: Apr 5, 2018
 *      Author: carsonbertozzi
 */

#ifndef DELAY_H_
#define DELAY_H_

#define FREQ_32KHz 32768.0
#define FREQ_1500KHz 1500000
#define FREQ_3MHz 3000000
#define FREQ_6MHz 6000000
#define FREQ_12MHz 12000000 //min delay 5us, gives ~7us
#define FREQ_24MHz 24000000 //min delay 3us, gives ~5us
#define FREQ_48MHz 48000000 //min delay 1us, gives ~3us

void delay_ms(int msDelay, int freq);
void delay_us(int usDelay, int freq);
int setDCO(int freq);

#endif /* DELAY_H_ */
