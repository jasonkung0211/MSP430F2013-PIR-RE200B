//*****************************************************************************
//  "Ultra-low Power Motion Detection using the MSP430F2013"
//
//  Version 0-00
//
//  Version Summary:
//  0-00 released 8-10-2020- Initial release
//
//  K. Jason
//  Yadong Security Co Inc.
//  August 2020
//  Built with IAR Embedded Workbench Version: 7.20.1
//*****************************************************************************

#include  <msp430x20x3.h>

#define   LED_OUT         BIT0              // Bit location for LED
#define   SENSOR_PWR      BIT7              // Bit location for power to sensor
#define   THRESHOLD       50                // Threshold for motion

static unsigned int result_old = 0;         // Storage for last conversion

void main(void)
{
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL; // ACLK/32768, int timer: ~10s
  
  // Set DCO to 1MHz
  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;
  
  BCSCTL1 |= DIVA_2;                        // ACLK = VLO/4
  BCSCTL3 |= LFXT1S_2;
  
  P1OUT = 0x10;                             // P1OUTs
  P1SEL = 0x08;                             // Select VREF function
  P1DIR = 0xEF;                             // Unused pins as outputs
  
  P2OUT = 0x00 + SENSOR_PWR;                // P2OUTs
  P2SEL &= ~SENSOR_PWR;                     // P2.7 = GPIO
  P2DIR = 0xff;                             // Unused pins as outputs

  /******************************************************************************
   * SD16_A configuration
   ******************************************************************************/  
  SD16CTL = SD16VMIDON | SD16REFON | SD16SSEL_1;        // 1.2V ref, SMCLK
  SD16INCTL0 = SD16GAIN_32 | SD16INCH_4 | SD16INTDLY_0;                // PGA = 32x, Diff inputs A4- & A4+
  SD16CCTL0 =  SD16SNGL | SD16IE | SD16OSR_1024; // Single conversion, 1024OSR, Int enable

  SD16CTL &= ~SD16VMIDON;                   // VMID off: used to settle ref cap
  SD16AE = SD16AE1 + SD16AE2;               // P1.1 & P1.2: A4+/- SD16_A inputs
  /******************************************************************************
   * SD16_A configuration
   ******************************************************************************/
  

  // Wait for PIR sensor to settle: 1st WDT+ interval
  P1SEL |= LED_OUT;                         // Turn LED on with ACLK (for low Icc)
  while(!(IFG1 & WDTIFG));                  // ~5.4s delay: PIR sensor settling
  P1SEL &= ~LED_OUT;                        // Turn LED off with ACLK (for low Icc)

  // Reconfig WDT+ for normal operation: interval of ~341msec
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1;// ACLK/512, int timer: 341msec
  BCSCTL1 |= DIVA_3;                        // ACLK = VLO/8
  IE1 |= WDTIE;                             // Enable WDT interrupt

  _BIS_SR(LPM3_bits + GIE);                 // Enter LPM3 with interrupts
}

/******************************************************
// SD16_A interrupt service routine
******************************************************/
#pragma vector = SD16_VECTOR
__interrupt void SD16ISR(void)
{ unsigned int result_new;

  SD16CTL &= ~SD16REFON;                    // Turn off SD16_A ref
  result_new = SD16MEM0;                    // Save result (clears IFG)

  if (result_new > result_old)              // Get difference between samples
    result_old = result_new - result_old;
  else
    result_old = result_old - result_new;

  if (result_old > THRESHOLD)               // If motion detected...
  {
     P1OUT |= LED_OUT;                      // Turn LED on
  }

  result_old = SD16MEM0;                    // Save last conversion

  __bis_SR_register_on_exit(SCG1+SCG0);     // Return to LPM3 after reti
}

/******************************************************
// Watchdog Timer interrupt service routine
******************************************************/
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
  if (!(P1OUT & LED_OUT))                   // Has motion already been detected?
  {
    SD16CTL |= SD16REFON;                   // If no, turn on SD16_A ref
    SD16CCTL0 |= SD16SC;                    // Set bit to start new conversion
    __bic_SR_register_on_exit(SCG1+SCG0);   // Keep DCO & SMCLK on after reti
  }
  else
    P1OUT &= ~LED_OUT;                      // If yes, turn off LED, measure on next loop
}
