//******************************************************************************
//  CPE 329 - Assignment 9
//
//  Description: This demo connects an MSP432 to a Microchip 24LC256 EEPROM via
//  the I2C bus. The MSP432 acts as the master and the EEPROM is a slave.
//  The EEPROM uses 3 external connections A2 A1 A0 to set the lower 3 bits of
//  its bus address. This creates a bus address of "1 0 1 0 A2 A1 A0". The code
//  below assumes those three connections are all connected to VSS (Ground) and
//  are logic 0. This gives the EEPROM a bus address of 0x50.
//
//
//                                /|\  /|\
//               MSP432P401      10k  10k     24LC256 EEPROM
//                 master          |    |          Slave
//             -----------------   |    |   -----------------
//            |     P1.6/UCB0SDA|<-|----|->|SDA (5)          |
//            |                 |  |       |                 |
//            |                 |  |       |                 |
//            |     P1.7/UCB0SCL|<-|------>|SCL (6)          |
//            |                 |          |                 |
//
//   Paul Hummel
//   Cal Poly
//   May 2017 (created)
//   Built with CCSv7.1
//******************************************************************************
#include "msp.h"
#include <stdint.h>
#include "delay.h"

#define BNO055_ADDRESS 0x28

void InitBNO055(uint8_t DeviceAddress);
void WriteBNO055(uint8_t MemAddress, uint8_t MemByte);
uint8_t ReadBNO055(uint8_t MemAddress);

uint16_t TransmitFlag = 0;

int main(void)
{
    uint32_t i;
    uint8_t low_value, high_value, concat_value;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;       // Stop watchdog timer

    __enable_irq();                           // Enable global interrupt

    setDCO(FREQ_12MHz);

    InitBNO055(BNO055_ADDRESS);
    WriteBNO055(0x3D, 0x00);
    delay_ms(19,FREQ_12MHz);


    WriteBNO055(0x3E, 0x00);
    WriteBNO055(0x3B, 0x00);
    WriteBNO055(0x3D, 0x0C);
    delay_ms(7,FREQ_12MHz);


    while(1){

        low_value = ReadBNO055(0x1A);     // Read value from EEPROM
        high_value = ReadBNO055(0x1B);
        concat_value = high_value << 16 | low_value;

    }
}

////////////////////////////////////////////////////////////////////////////////
//
// Initialize I2C bus for communicating with EEPROM.
//
////////////////////////////////////////////////////////////////////////////////
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
}

////////////////////////////////////////////////////////////////////////////////
//
//  Function that writes a single byte to the EEPROM.
//
//  MemAddress  - 2 byte address specifies the address in the EEPROM memory
//  MemByte     - 1 byte value that is stored in the EEPROM
//
//  Procedure :
//      start
//      transmit address+W (control+0)     -> ACK (from EEPROM)
//      transmit data      (high address)  -> ACK (from EEPROM)
//      transmit data      (low address)   -> ACK (from EEPROM)
//      transmit data      (data)          -> ACK (from EEPROM)
//      stop
//
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
//
//  Function that reads a single byte from the EEPROM.
//
//  MemAddress  - 2 byte address specifies the address in the EEPROM memory
//  ReceiveByte - 1 byte value that is received from the EEPROM
//
//  Procedure :
//      start
//      transmit address+W (control+0)    -> ACK (from EEPROM)
//      transmit data      (high address) -> ACK (from EEPROM)
//      transmit data      (low address)  -> ACK (from EEPROM)
//      start
//      transmit address+R (control+1)    -> ACK (from EEPROM)
//      transmit data      (data)         -> NACK (from MSP432)
//      stop
//
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
//
// I2C Interrupt Service Routine
//
////////////////////////////////////////////////////////////////////////////////
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
