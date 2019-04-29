#include "BNO055.h"

volatile uint16_t TransmitFlag = 0;

void InitBNO055(uint8_t DeviceAddress)
{

  P6->SEL0 |= BIT5 | BIT4;                // Set I2C pins of EUSCI_B1

  // Enable EUSCIB1 interrupt in NVIC module
  NVIC->ISER[0] = 1 << ((EUSCIB1_IRQn) & 31);

  // Configure USCI_B0 for I2C mode
  EUSCI_B1->CTLW0 |= EUSCI_A_CTLW0_SWRST;   // Software reset enabled
  EUSCI_B1->CTLW0 = EUSCI_A_CTLW0_SWRST |   // Remain eUSCI in reset mode
          EUSCI_B_CTLW0_MODE_3 |            // I2C mode
          EUSCI_B_CTLW0_MST |               // Master mode
          EUSCI_B_CTLW0_SYNC |              // Sync mode
          EUSCI_B_CTLW0_SSEL__SMCLK;        // SMCLK

  EUSCI_B1->BRW = 30;                       // baudrate = SMCLK / 30 = 100kHz
  EUSCI_B1->I2CSA = DeviceAddress;          // Slave address
  EUSCI_B1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;  // Release eUSCI from reset

  EUSCI_B1->IE |= EUSCI_A_IE_RXIE |         // Enable receive interrupt
                  EUSCI_A_IE_TXIE;

  WriteBNO055(0x3D, 0x00);
  delay_ms(19,FREQ_12MHz);

}

void WriteBNO055(uint8_t MemAddress, uint8_t MemByte)
{

  EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR;          // Set transmit mode (write)
  EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;       // I2C start condition

  while (!TransmitFlag);                // Wait for EEPROM address to transmit
  TransmitFlag = 0;

  EUSCI_B1 -> TXBUF = MemAddress;    // Send the high byte of the memory address

  while (!TransmitFlag);            // Wait for the transmit to complete
  TransmitFlag = 0;

  EUSCI_B1 -> TXBUF = MemByte;      // Send the byte to store in EEPROM

  while (!TransmitFlag);            // Wait for the transmit to complete
  TransmitFlag = 0;

  EUSCI_B1 -> CTLW0 |= EUSCI_B_CTLW0_TXSTP;   // I2C stop condition
}


uint8_t ReadBNO055(uint8_t MemAddress)
{
  uint8_t ReceiveByte;

  EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR;      // Set transmit mode (write)
  EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;   // I2C start condition

  while (!TransmitFlag);                // Wait for EEPROM address to transmit
  TransmitFlag = 0;

  EUSCI_B1 -> TXBUF = MemAddress;    // Send the low byte of the memory address

  while (!TransmitFlag);            // Wait for the transmit to complete
  TransmitFlag = 0;

  EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;   // Set receive mode (read)
  EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT; // I2C start condition (restart)

  // Wait for start to be transmitted
  while ((EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTT));

  // set stop bit to trigger after first byte
  EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

  while (!TransmitFlag);            // Wait to receive a byte
  TransmitFlag = 0;

  ReceiveByte = EUSCI_B1->RXBUF;    // Read byte from the buffer

  return ReceiveByte;
}

void EUSCIB1_IRQHandler(void)
{
    if (EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0)     // Check if transmit complete
    {
        EUSCI_B1->IFG &= ~ EUSCI_B_IFG_TXIFG0;  // Clear interrupt flag
        TransmitFlag = 1;                       // Set global flag
    }

    if (EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0)     // Check if receive complete
    {
        EUSCI_B1->IFG &= ~ EUSCI_B_IFG_RXIFG0;  // Clear interrupt flag
        TransmitFlag = 1;                       // Set global flag
    }
}
