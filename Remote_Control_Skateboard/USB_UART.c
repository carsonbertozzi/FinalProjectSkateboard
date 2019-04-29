//USB_UART.C

#include "USB_UART.h"

void USB_UARTInit(void){
    // Configure UART pins
    P1->SEL0 |= BIT2 | BIT3;
    // set 2-UART pin as secondary function
    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
    EUSCI_A_CTLW0_SSEL__SMCLK; // Configure eUSCI clock source for SMCLK
    // Baud Rate calculation
    // 12000000/(115200) = 104.1666667
    // Fractional portion = 0.1666667
    // User's Guide Table 21-4: UCBRSx = 0x00
    // UCBRx = int (104.16667 / 16) = 6
    // UCBRFx = int (((104.1667/16)-6)*16) = 8
    EUSCI_A0->BRW = 6; // Using baud rate calculator
    EUSCI_A0->MCTLW = (8 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16;
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG; // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE; // Enable USCI_A0 RX interrupt
    // Enable global interrupt
    __enable_irq();
    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
}

// UART interrupt service routine
void EUSCIA0_IRQHandler(void) {
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        // Echo the received character back
        EUSCI_A0->TXBUF = EUSCI_A0->RXBUF; //also clears the flags
        if (EUSCI_A0->RXBUF <= 57 && EUSCI_A0->RXBUF >= 48) //check if between ascii 0 and 9
            buf = (buf * 10) + (EUSCI_A0->RXBUF - 48); //write the char as an int to the ones space of buf and shift the others up by 10
        else if (EUSCI_A0->RXBUF == 0xD) // if the key is "Enter" set the flag
            flag = 1;
        // if key is anything else reset the buffer, obviously the user is stupid and their inputs should be cleared
        else
            buf = 0;
    }
}

void USB_UARTWrite(uint8_t num){
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); //check if TX buffer is empty

    EUSCI_A0->TXBUF = num; //writes to UART
}

uint8_t getUARTFlag(void){
    return flag;
}
uint16_t getUARTNum(void){
    if (buf > 0 && buf < 4096)
        return buf;
    else
        return 0;
}

void clearUARTFlag(void){
    flag = 0;
}
void clearUARTBuf(void){
    buf = 0;
}

void giveUARTAnalog(uint16_t analog){
    if (analog < 100){
        USB_UARTWrite('0');
    }
    else{
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
    }
    USB_UARTWrite(46); //decimal point
    USB_UARTWrite(analog / 10 + 48);
    analog %= 10;
    USB_UARTWrite(analog + 48);
}

void giveUARTInt(uint16_t analog){
    if (analog > 100000){
        USB_UARTWrite(analog / 100000 + 48);
        analog %= 100000;
        USB_UARTWrite(analog / 10000 + 48);
        analog %= 10000;
        USB_UARTWrite(analog / 1000 + 48);
        analog %= 1000;
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else if (analog > 10000){
        USB_UARTWrite(' ');
        USB_UARTWrite(analog / 10000 + 48);
        analog %= 10000;
        USB_UARTWrite(analog / 1000 + 48);
        analog %= 1000;
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else if (analog > 1000){
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(analog / 1000 + 48);
        analog %= 1000;
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else if (analog > 100){
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else if (analog > 10){
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else{
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(analog + 48);
    }
}

void giveUARTPosNegInt(int16_t analog){

    if (analog < 0){
        USB_UARTWrite('-');
    }

    analog = abs(analog);

    if (analog > 100000){
        USB_UARTWrite(analog / 100000 + 48);
        analog %= 100000;
        USB_UARTWrite(analog / 10000 + 48);
        analog %= 10000;
        USB_UARTWrite(analog / 1000 + 48);
        analog %= 1000;
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else if (analog > 10000){
        USB_UARTWrite(' ');
        USB_UARTWrite(analog / 10000 + 48);
        analog %= 10000;
        USB_UARTWrite(analog / 1000 + 48);
        analog %= 1000;
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else if (analog > 1000){
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(analog / 1000 + 48);
        analog %= 1000;
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else if (analog > 100){
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(analog / 100 + 48);
        analog %= 100;
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else if (analog > 10){
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(analog / 10 + 48);
        analog %= 10;
        USB_UARTWrite(analog + 48);
    }
    else{
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(' ');
        USB_UARTWrite(analog + 48);
    }


}


void giveUARTString(char *string){
    uint8_t s = 0;
    while (string[s] != '\0'){
        USB_UARTWrite(string[s]);
        s++;
    }
}

void resetTerminal(void){
    giveUARTString("\x1B[2K");
    giveUARTString("\x1B[H");
    USB_UARTWrite('\r');
}

void drawBorder(void){
    uint16_t displayCount;
    for (displayCount = 0; displayCount < 50; displayCount++){
        USB_UARTWrite('*');
    }
    giveUARTString("\n\r");
}

void writeDC(uint16_t dc){
    giveUARTString("DC Value: ");
    giveUARTAnalog(dc);
    giveUARTString("V\n\r");
}

void writeVPP(uint16_t vpp){
    giveUARTString("Peak-to-peak : ");
    giveUARTAnalog(vpp);
    giveUARTString("V\n\r");
}

void writeRMS(uint16_t RMSFloatValue){
    giveUARTString("RMS Value: ");
    giveUARTAnalog(RMSFloatValue);
    giveUARTString("V\n\r");
}

void writeFrequency(uint16_t freq){
    giveUARTString("Frequency: ");
    giveUARTInt(freq);
    giveUARTString("Hz\n\r");
}

void drawDCBars(uint16_t dc){
    uint16_t displayCount = dc / 10;
    uint16_t graphCount;
    giveUARTString("DC Bar Graph:  ");
    for (graphCount = 0; graphCount < 34; graphCount++){
        if (graphCount < displayCount){
            giveUARTString(" | ");
        }
        else{
            giveUARTString(" - ");
        }
    }
    giveUARTString("\n\r");
}

void drawRMSBars(uint16_t RMSFloatValue){
    uint16_t displayCount = RMSFloatValue / 10;
    uint16_t graphCount;
    giveUARTString("RMS Bar Graph: ");
    for (graphCount = 0; graphCount < 34; graphCount++){
        if (graphCount < displayCount){
            giveUARTString(" | ");
        }
        else{
            giveUARTString(" - ");
        }
    }
    giveUARTString("\n\r");
}

void drawBarScale(void){
    giveUARTString("Volts          ");
    giveUARTString("0.0            ");
    giveUARTString("0.5            ");
    giveUARTString("1.0            ");
    giveUARTString("1.5            ");
    giveUARTString("2.0            ");
    giveUARTString("2.5            ");
    giveUARTString("3.0      ");
    giveUARTString("3.3\n\r");
}
