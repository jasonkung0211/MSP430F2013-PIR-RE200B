//******************************************************************************
//  MSP430F2013
//
//  Description: Continuously sends a counter value over the USI in SPI which
//				 acting as if it were a UART. The sample circuit connects an
//				 FTDI breakout board to the transmit line and reads. The baud
//				 rate is determined by the USI speed, which in this case is
//				 1100000 baud (DCO = 1.1MHz).
//
//  Nicholas J. Conn
//  08-04-2010
//  Built with CCS Version: 4.1.3
//******************************************************************************

#include  "msp430F2013.h"

// Function Definitions
void transmit(unsigned int);

int main( void )
{

  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
  
  //****************************
  //   Set up USI - LSB first
  //****************************
  USICTL0 |= USIPE6 + USIMST + USILSB + USIOE;	// Enable Output, SPI master, LSB first
  USICTL1 |= USIIE;              				// Counter interrupt, flag remains set
  USICKCTL = USISSEL_3;                			// Use SMCLK
  
  USICTL0 &= ~USISWRST;                 		// USI released for operation
  
  //*******************************************************
  //   Set initial program varibles and start peripherals
  //*******************************************************
  unsigned int i = 0;
  while(1)
  {
  	transmit(i);
  	i++;
  }
} 

// Transmits one byte (TXData) using the USI as a UART
void transmit( unsigned int TXData )
{    
  //**********************************************
  //   Send a byte on the USI as if it were UART
  //**********************************************
  TXData = TXData << 1;             // Add start bit
  TXData |= 0xFE00;                 // Add stop bit
  
  while (!(USIIFG & USICTL1)); 		// Wait if USI is busy
  USISR = TXData;    				// Sets word to be sent
  USICNT = USI16B + 10;           	// 16 bit mode, sends 10 bits
}
