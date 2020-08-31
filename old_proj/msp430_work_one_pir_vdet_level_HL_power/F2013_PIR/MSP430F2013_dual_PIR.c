//*****************************************************************************
//  Code for - "Dual Ultra-low Power Motion Detection
//    using the MSP430F2013"
//
//  Version 2-00
//
//  Version Summary:
//   1-00 released 12-22-2014- Initial release
//   2-00 released 9-24-2015- digital low pass filter
//
//   6-00 released 8-28-2019- 8/13 experimental results to find the dividend 2048
//                            8/27 Propose how to find the eigenvalues of the waveform
//                            8/28 Using DAC to generate signals, testing 5000 times, can correctly capture the eigenvalues
//   7-00 released 9-6-2019-  9/6  For the additional processing of the eigenvalue 0X02, it will be triggered 3 to 5 times in 20 minutes
//   8-00 released 9-23-2019- 9/23 for dual PIR
//   8-10 released 11-26-2019- 11/26 for dual PIR voltage detector
//   9-00 released 12-11-2019- dual PIR voltage detector; low power mode
//  10-00 released 01-15-2020- one PIR voltage detector; low power mode
//  10-10 released 03-30-2020- one PIR voltage detector; low power mode; chage NORMAL_ADCANY_HLIMIT = 3 (嵌揚PIR感度較差) lEDfLICKER 75 => 100
//  10-11 released 04-24-2020- one PIR voltage detector; 參數化(Information memory D)
//  11-00 released 05-25-2020- one PIR voltage detector; Join algorithm 邊緣偵測做level/freq分類
//  Built with IAR Embedded Workbench Version: 5.20
//*****************************************************************************

#include  <msp430x20x3.h>
const unsigned int NORMAL_ADCANY_HLIMIT_F @ 0x1000  = 0x03; //Information memory D
const unsigned int lEDfLICKER_F           @ 0x1002  = 0x64; //Information memory D
const unsigned int tRIGtIMEoUT_F          @ 0x1004  = 0x64; //Information memory D
const unsigned int MODESTATE_F            @ 0x1006  = 0xFF; //Information memory D
#define   WDT_PIN          BIT7                // P2.7 Watch Dog Timer 
#define   TEST_PIN         BIT7                // P1.7 test pin 
#define   LED_OUT          BIT0                // P1.0 =>Bit location for LED
#define   RELAY_ON         BIT6                // P2.6
#define   SPICS_PIN        BIT4                // P1.4 SPI CS for debug
#define   fWvERSION          11                // firm ware VERSION
#define   iLEDtIME        42000                // V3:31250 iNITION LED tIME 250ms flash one time
#define   iLEDcYCLE          10                //140 for 30sec, 40 for 10sec, Wait sensor stable
#define   wDTtIMEoUT         12                //10x24ms=0.24S
//#define   tRIGtIMEoUT     100                //100x24ms=2.4S
#define   HIGH_               1
#define   LOW_                0
#define   vOLTAGE_dET_H   45300                //9.0V + 0.5V
#define   vOLTAGE_dET_L   43300                //9.0V - 0.5V

#define   mODE12V             0                //12V
#define   mODE09V             1                //9V

static unsigned char  LED_LOOP=0;
static unsigned char  WDT_Count = 0;
static unsigned char  TRIGGER_Count = 0;

static unsigned int Result4_old = 35840;                    // Storage for last conversion
static unsigned int ADCANY4= 70;//70;                            // adc/2^9
static unsigned char selectADC = 4;                         //P1.1_A4+
static unsigned char timeOutMode09V= 0; 
static unsigned char timeOutMode12V= 0; 

static unsigned char lEDfLICKER = 0;                        // Information memory B @0x1002
static unsigned char tRIGtIMEoUT = 0;                       // Information memory B @0x1004
static unsigned char mODEoNoFF = 255;                       // Information memory B @0x1006
static unsigned char modeState = 0; 

static int  aDCtEMP[5] = {64,64,64,64,64};
static int  eDGEdET[3] = {0,0,0};
static int  eDGErESULT = 0;
static int  eDGEcOUNT = 0;
static int  eDGEfLAG = 0;
static int  eDGEfREQ = 0;
static int  bASEfREQ = 1*1000*10/2/24;                      //1*1000=1000mSec,23mSec=scale,2=2*(1/2)duty
static int  eDGEfREQtEMP[5] = {0,0,0,0,0};
static int  eDGEcOUNTtEMP[5] = {0,0,0,0,0};
static int  aDCrESULT[5] = {0,0,0,0,0};
static int  aDCaVR = 64;

static int  eDGEpOS = 2;
static int  eDGEnEG = -2;

static int  aDClIMIT = 5*2;

void main(void)

{
  
  unsigned int init_led_onoff_time = 0;
  unsigned int init_led_onoff_cycle = 0;
  
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL;                 // ACLK/32768, int timer: ~10s   
  BCSCTL1 = CALBC1_1MHZ;                                    // Set DCO to 1MHz   
  DCOCTL = CALDCO_1MHZ;   
  BCSCTL1 |= DIVA_2;                                        // ACLK = VLO/4   
  BCSCTL3 |= LFXT1S_2;   
  
  P1SEL = 0x08;

  P1DIR |= LED_OUT;  P1SEL &= ~LED_OUT;  P1OUT &= ~LED_OUT;  P1REN &= ~LED_OUT;
  P1DIR |= TEST_PIN; P1SEL &= ~TEST_PIN; P1OUT &= ~TEST_PIN; P1REN &= ~TEST_PIN; 
  P2DIR |= RELAY_ON; P2SEL &= ~RELAY_ON; P2OUT |= RELAY_ON;  P2REN &= ~RELAY_ON;
  P2DIR |= WDT_PIN;  P2SEL &= ~WDT_PIN;  P2OUT |= WDT_PIN;   P2REN &= ~WDT_PIN;
  P1DIR |= SPICS_PIN; P1SEL &= ~SPICS_PIN; P1OUT &= ~SPICS_PIN; P1REN &= ~SPICS_PIN; // for debug
   
  SD16CTL = SD16LP + SD16VMIDON + SD16REFON + SD16SSEL_1;   // 1.2V ref, SMCLK   
  SD16INCTL0 = SD16GAIN_1 + SD16INCH_4;                     // PGA = 4x, Diff inputs A4- & A4+   
  SD16CCTL0 =  SD16SNGL + SD16IE + SD16OSR_32;              // Single conversion, 256OSR, Int enable   
  SD16CTL &= ~SD16VMIDON;                                   // VMID off: used to settle ref cap   
  SD16AE = SD16AE1 + SD16AE6 ;                              // P1.1_A4+, P1.6_A3+ SD16_A inputs  
  

  if ((lEDfLICKER_F>=75) && (lEDfLICKER_F<=200))
  {
    lEDfLICKER=lEDfLICKER_F;
  }
  else
  {
    lEDfLICKER=100;
  }
  
  if ((tRIGtIMEoUT_F>=70) && (tRIGtIMEoUT_F<=160))
  {
    tRIGtIMEoUT=tRIGtIMEoUT_F;
  }
  else
  {
    tRIGtIMEoUT=125;
  }
  
  if (MODESTATE_F==0)
  {
    mODEoNoFF=0;        //turn on 09V/12V Judgment
  }
  else
  {
    mODEoNoFF=255;      //turn off 09V/12V Judgment
  }
  
  for(init_led_onoff_cycle=0; init_led_onoff_cycle <=iLEDcYCLE; init_led_onoff_cycle++)
  {
    P2OUT ^= WDT_PIN;
    for(init_led_onoff_time=0; init_led_onoff_time <=iLEDtIME; init_led_onoff_time++)
    { P1OUT &= ~LED_OUT; }
    if(((fWvERSION*2-2)<= init_led_onoff_cycle) && (init_led_onoff_cycle <= 16))
    {
     //do nothing
    }
    else
    {
     for(LED_LOOP=0; LED_LOOP<=lEDfLICKER; LED_LOOP++) { P1OUT ^= LED_OUT; }
     P1OUT &= ~LED_OUT;
    }
  }
  
  
  P1OUT &= ~LED_OUT; 
  P1OUT |= TEST_PIN;                          //for test
  
  // Reconfig WDT+ for normal operation: interval of ~341msec   
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0;// ACLK/512, int timer: 341msec   
  //BCSCTL1 |= DIVA_1;                        // ACLK = VLO/8   
  IE1 |= WDTIE;                               // Enable WDT interrupt   
  
  modeState=mODE12V;  
  _BIS_SR(LPM3_bits + GIE);                   // Enter LPM3 with interrupts 
  
}

/******************************************************
// SD16_A interrupt service routine
******************************************************/
#pragma vector = SD16_VECTOR
__interrupt void SD16ISR(void)
{     
  SD16CTL &= ~SD16REFON;                      // Turn off SD16_A ref
  
  switch(selectADC)
  {
    //A4+ for PIR1  
    case 4:
      Result4_old = SD16MEM0;                 // Save result (clears IFG)  
      ADCANY4 = Result4_old>>9;               // adc shift 9 bits;
      aDCtEMP[4] = aDCtEMP[3];
      aDCtEMP[3] = aDCtEMP[2];
      aDCtEMP[2] = aDCtEMP[1];
      aDCtEMP[1] = aDCtEMP[0];
      aDCtEMP[0] = ADCANY4;
      
      eDGEdET[0] = ((aDCtEMP[0]+aDCtEMP[2])-(2*aDCtEMP[1]));
      eDGEdET[1] = ((aDCtEMP[1]+aDCtEMP[3])-(2*aDCtEMP[2]));
      eDGEdET[2] = ((aDCtEMP[2]+aDCtEMP[4])-(2*aDCtEMP[3]));      
      eDGErESULT = eDGEdET[0] + eDGEdET[1] + eDGEdET[2];
      
      //if ((eDGErESULT<0)&&(eDGErESULT<=eDGEnEG))
      if (eDGErESULT<=eDGEnEG)
      {
        if (eDGEcOUNT >=5)
        {
          eDGEfREQ = 217/(eDGEcOUNT);
          eDGEcOUNTtEMP[4] = eDGEcOUNTtEMP[3];
          eDGEcOUNTtEMP[3] = eDGEcOUNTtEMP[2];
          eDGEcOUNTtEMP[2] = eDGEcOUNTtEMP[1];
          eDGEcOUNTtEMP[1] = eDGEcOUNTtEMP[0];
          eDGEcOUNTtEMP[0] = eDGEcOUNT;  
          eDGEfREQtEMP[4]  = eDGEfREQtEMP[3];
          eDGEfREQtEMP[3]  = eDGEfREQtEMP[2];
          eDGEfREQtEMP[2]  = eDGEfREQtEMP[1];
          eDGEfREQtEMP[1]  = eDGEfREQtEMP[0];
          eDGEfREQtEMP[0]  = eDGEfREQ;
          aDCrESULT[4]     = aDCrESULT[3];
          aDCrESULT[3]     = aDCrESULT[2];
          aDCrESULT[2]     = aDCrESULT[1];
          aDCrESULT[1]     = aDCrESULT[0];
          aDCrESULT[0]     = abs(aDCaVR-(aDCtEMP[0]+aDCtEMP[1]+aDCtEMP[2]+aDCtEMP[3]+aDCtEMP[4]));
        }
        eDGEcOUNT = 0;
        eDGEfLAG=1;
        P1OUT &= ~TEST_PIN;
      }
      //else if ((eDGErESULT>0)&&(eDGErESULT>=eDGEpOS))
      else if (eDGErESULT>=eDGEpOS)
      {
        if (eDGEcOUNT >=5)
        {
          eDGEfREQ = bASEfREQ/(eDGEcOUNT); 
          eDGEcOUNTtEMP[4] = eDGEcOUNTtEMP[3];
          eDGEcOUNTtEMP[3] = eDGEcOUNTtEMP[2];
          eDGEcOUNTtEMP[2] = eDGEcOUNTtEMP[1];
          eDGEcOUNTtEMP[1] = eDGEcOUNTtEMP[0];
          eDGEcOUNTtEMP[0] = eDGEcOUNT;
          eDGEfREQtEMP[4]  = eDGEfREQtEMP[3];
          eDGEfREQtEMP[3]  = eDGEfREQtEMP[2];
          eDGEfREQtEMP[2]  = eDGEfREQtEMP[1];
          eDGEfREQtEMP[1]  = eDGEfREQtEMP[0];
          eDGEfREQtEMP[0]  = eDGEfREQ;
          aDCrESULT[4]     = aDCrESULT[3];
          aDCrESULT[3]     = aDCrESULT[2];
          aDCrESULT[2]     = aDCrESULT[1];
          aDCrESULT[1]     = aDCrESULT[0];
          aDCrESULT[0]     = abs((aDCtEMP[0]+aDCtEMP[1]+aDCtEMP[2]+aDCtEMP[3]+aDCtEMP[4])-aDCaVR);
        }
        eDGEcOUNT = 0;
        eDGEfLAG=1;
        P1OUT &= ~TEST_PIN;
      }
      else 
      {
        aDCaVR = (aDCtEMP[0]+aDCtEMP[1]+aDCtEMP[2]+aDCtEMP[3]+aDCtEMP[4]);
      }
      
      if ((eDGEfREQ>=3)&&(eDGEfREQ<=50)&&(aDCrESULT[0]>=aDClIMIT))      //0.3Hz~5Hz
      {          
          eDGEcOUNT=0;
          eDGEfLAG=0;
          eDGEfREQ=0;
          eDGErESULT=0;
          TRIGGER_Count=0;
      }
      else if (eDGEfREQ>50)
      {
          eDGEcOUNT=0;
          eDGEfLAG=0;
          eDGEfREQ=0;
          eDGErESULT=0;
      }
      
      __bis_SR_register_on_exit(SCG1+SCG0);   // Return to LPM3 after reti
    break;
    
    default :
      Result4_old = SD16MEM0;                 // Save result (clears IFG)  
      ADCANY4 = Result4_old>>9;               // adc shift 9 bits;
      __bis_SR_register_on_exit(SCG1+SCG0);   // Return to LPM3 after reti
    break;
  }
  
}

/******************************************************
// Watchdog Timer interrupt service routine
******************************************************/
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)

{  
  
  P2OUT &= ~WDT_PIN;  
  P1OUT ^= SPICS_PIN; //for debug
  WDT_Count ++; 
  P1OUT |= TEST_PIN;
  
  if (eDGEfLAG==1)
  {
    if (eDGEcOUNT < bASEfREQ) eDGEcOUNT++;
    else eDGEcOUNT = bASEfREQ;
  }
  
  if ((WDT_Count%3)==0)
  {
  selectADC=4;
  SD16INCTL0 |= SD16INCH_4;                 // Enable channel A4+
  SD16CTL |= SD16REFON;                     // If no, turn on SD16_A ref   
  SD16CCTL0 |= SD16SC;                      // Set bit to start new conversion   
  __bic_SR_register_on_exit(SCG1+SCG0);     // Keep DCO & SMCLK on after reti
  }
  
  if(TRIGGER_Count>=tRIGtIMEoUT) { P2OUT |= RELAY_ON; TRIGGER_Count=tRIGtIMEoUT; }
  else
  {
    P2OUT &= ~RELAY_ON;
    TRIGGER_Count++;
    for(LED_LOOP=0; LED_LOOP<=lEDfLICKER; LED_LOOP++) { P1OUT ^= LED_OUT; }
    P1OUT &= ~LED_OUT;
  } 
  

  if(WDT_Count>=wDTtIMEoUT)
  {
    SD16CTL = SD16LP + SD16SSEL_1;                            // 1.2V ref, SMCLK   
    SD16INCTL0 = SD16GAIN_1 + SD16INCH_4;                     // PGA = 4x, Diff inputs A4- & A4+   
    SD16CCTL0 =  SD16SNGL + SD16IE + SD16OSR_32;              // Single conversion, 256OSR, Int enable   
    SD16AE = SD16AE1 + SD16AE6 ;                              // P1.1_A4+, P1.6_A3+ SD16_A inputs   
    
    P2OUT |= WDT_PIN;  WDT_Count=0; 
  }
}
