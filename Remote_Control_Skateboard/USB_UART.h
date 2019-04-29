/*
* USB_UART.h
*
* Created on: May 6, 2018
* Author: carsonbertozzi
*/

#ifndef USB_UART_H_
#define USB_UART_H_

#include "msp.h"

static uint8_t flag = 0;
static uint16_t buf = 0;

void USB_UARTInit(void);
void USB_UARTWrite(uint8_t num);
uint8_t getUARTFlag(void);
uint16_t getUARTNum(void);
void clearUARTFlag(void);
void clearUARTBuf(void);
void giveUARTAnalog(uint16_t analog);
void giveUARTInt(uint16_t);
void giveUARTPosNegInt(int16_t);
void giveUARTString(char *string);
void resetTerminal(void);
void drawBorder(void);
void writeDC(uint16_t dc);
void writeVPP(uint16_t vpp);
void writeRMS(uint16_t RMSFloatValue);
void writeFrequency(uint16_t freq);
void drawDCBars(uint16_t dc);
void drawRMSBars(uint16_t RMSFloatValue);
void drawBarScale(void);

#endif /* USB_UART_H_ */
