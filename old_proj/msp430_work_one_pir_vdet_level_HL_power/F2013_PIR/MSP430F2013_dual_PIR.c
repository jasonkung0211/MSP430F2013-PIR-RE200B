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

#define EAS_DH14        0
#define EAS_DH18        1
#define EAS_DH18_DEBUG  0

#if (EAS_DH18)
  #define   WDT_PIN          BIT7                // P2.7 Watch Dog Timer 
#endif
#if (EAS_DH18_DEBUG)
  #define   WDT_PIN          BIT7                // P2.7 Watch Dog Timer 
#endif
#if(EAS_DH14)
  #define   WDT_PIN          BIT0                // P1.0 Watch Dog Timer 
#endif

#define   TEST3_PIN        BIT7                // P1.7 test3 pin 

#if (EAS_DH18)
  #define   LED_OUT          BIT0                // P1.0 =>Bit location for LED
#endif
#if (EAS_DH18_DEBUG)
  #define   LED_OUT          BIT0                // P1.0 =>Bit location for LED
#endif
#if(EAS_DH14)
  #define   LED_OUT          BIT6                // P1.6 =>Bit location for LED
#endif



#define   RELAY_ON         BIT6                // P2.6
#define   TEST2_PIN        BIT5                // P1.5 test2 pin
#define   TEST1_PIN        BIT4                // P1.4 test1 pin

#define   fWvERSION          11                // firm ware VERSION
#define   iLEDtIME        42000                // V3:31250 iNITION LED tIME 250ms flash one time
#define   iLEDcYCLE          10                //140 for 30sec, 40 for 10sec, Wait sensor stable
#define   wDTtIMEoUT         86                //86x5.8ms=0.5S


static unsigned char  LED_LOOP=0;
static unsigned char  WDT_Count = 0;
static unsigned char  TRIGGER_Count = 0;

static unsigned int Result4_old = 35840;                    // Storage for last conversion
static unsigned int ADCANY4= 128;                           // adc/2^8
static unsigned char selectADC = 4;                         //P1.1_A4+

static unsigned char lEDfLICKER = 0;                        // Information memory B @0x1002
static unsigned char tRIGtIMEoUT = 0;                       // Information memory B @0x1004

static int  aDCtEMP[7] = {128,128,128,128,128,128,128};
static int  sMOOTH1ST[5] = {0,0,0,0,0};
static int  sMOOTH2RD[3] = {0,0,0};
static int  eDGErESULT1ST = 0;
static int  eDGErESULT2RD = 0;
static int  eDGEsLOPE = 0;
static int  eDGEcOUNT = 0;

static unsigned int  eDGE1STcNTn = 0;
static unsigned int  eDGE1STcNTu = 0;
static unsigned int  eDGEcOUNTlIMIT = 1000/6/1;//1000/3/1
static unsigned int  eDGE1STcNTtRIG = 2;

static int  eDGEaDCmAX=130;
static int  eDGEaDCmIN=130;
static int  eDGEaDCrEF=0;
static int  eDGEaDCdIF=0;
static int  eDGEaDCdIFsUM=0;
static unsigned int  eDGEaDCcNT = 0;

void main(void)

{
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL;                   
  BCSCTL1 = CALBC1_1MHZ;                                    // Set DCO to 1MHz   
  DCOCTL = CALDCO_1MHZ;   
  BCSCTL1 |= DIVA_0;                                        // ACLK = VLO/1
  BCSCTL3 |= LFXT1S_2;
  
  P1SEL = 0x08;

  P1DIR |= LED_OUT;  P1SEL &= ~LED_OUT;  P1OUT &= ~LED_OUT;  P1REN &= ~LED_OUT;  
  P2DIR |= RELAY_ON; P2SEL &= ~RELAY_ON; P2OUT |= RELAY_ON;  P2REN &= ~RELAY_ON;
  
  #if (EAS_DH18)
    P2DIR |= WDT_PIN;  P2SEL &= ~WDT_PIN;  P2OUT |= WDT_PIN;   P2REN &= ~WDT_PIN;
  #endif
  #if (EAS_DH18_DEBUG)
    P2DIR |= WDT_PIN;  P2SEL &= ~WDT_PIN;  P2OUT |= WDT_PIN;   P2REN &= ~WDT_PIN;
  #endif
  #if(EAS_DH14)
    P1DIR |= WDT_PIN;  P1SEL &= ~WDT_PIN;  P1OUT |= WDT_PIN;   P1REN &= ~WDT_PIN;
  #endif  

  P1DIR |= TEST1_PIN; P1SEL &= ~TEST1_PIN; P1OUT &= ~TEST1_PIN; P1REN &= ~TEST1_PIN; 
  P1DIR |= TEST2_PIN; P1SEL &= ~TEST2_PIN; P1OUT &= ~TEST2_PIN; P1REN &= ~TEST2_PIN;
  P1DIR |= TEST3_PIN; P1SEL &= ~TEST3_PIN; P1OUT &= ~TEST3_PIN; P1REN &= ~TEST3_PIN;
   
  SD16CTL = SD16LP + SD16VMIDON + SD16REFON + SD16SSEL_1;   // 1.2V ref, SMCLK  
  
  #if (EAS_DH18)
    SD16INCTL0 = SD16GAIN_1 + SD16INCH_4;
    SD16AE = SD16AE1 + SD16AE2 ;
  #endif
  #if (EAS_DH18_DEBUG)
    SD16INCTL0 = SD16GAIN_1 + SD16INCH_2;
    SD16AE = SD16AE4 + SD16AE5 ;
  #endif
  #if(EAS_DH14)
    SD16INCTL0 = SD16GAIN_1 + SD16INCH_4;
    SD16AE = SD16AE1 + SD16AE2 ;
  #endif  
  //SD16INCTL0 = SD16GAIN_1 + SD16INCH_4;                     // PGA = 1, Diff inputs A4- & A4+ 
  
  SD16CCTL0 =  SD16SNGL + SD16IE + SD16OSR_64;              // Single conversion, 256OSR, Int enable   
  SD16CTL &= ~SD16VMIDON;                                   // VMID off: used to settle ref cap   
  
  if ((lEDfLICKER_F>=75) && (lEDfLICKER_F<=200))
  {
    lEDfLICKER=lEDfLICKER_F;
  }
  else
  {
    lEDfLICKER=100;
  }
  
  if ((tRIGtIMEoUT_F>=166) && (tRIGtIMEoUT_F<=666))
  {
    tRIGtIMEoUT=tRIGtIMEoUT_F;
  }
  else
  {
    tRIGtIMEoUT=500;
  }
  
  P1OUT &= ~LED_OUT; 
  P1OUT &= ~TEST1_PIN;                        
  P1OUT &= ~TEST2_PIN;
  P1OUT &= ~TEST3_PIN;
  
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0; //10K~12K(VLO)/64(WDTIS1+WDTIS0)=156.25(6.4ms)~187.5(5.3ms)
  IE1 |= WDTIE;                               // Enable WDT interrupt   

  _BIS_SR(LPM3_bits + GIE);                   // Enter LPM3 with interrupts 
  
}

/******************************************************
// SD16_A interrupt service routine
******************************************************/
#pragma vector = SD16_VECTOR
__interrupt void SD16ISR(void)
{     
  //int i=0;
  SD16CTL &= ~SD16REFON;                      // Turn off SD16_A ref
  switch(selectADC)
  {
    //A4+ for PIR1  
    case 4:
      Result4_old = SD16MEM0;                 // Save result (clears IFG)  
      ADCANY4 = Result4_old>>7;               // adc shift 8 bits;
      if (ADCANY4==aDCtEMP[0]) {}
      else
      {
        aDCtEMP[6] = aDCtEMP[5];
        aDCtEMP[5] = aDCtEMP[4];
        aDCtEMP[4] = aDCtEMP[3];
        aDCtEMP[3] = aDCtEMP[2];
        aDCtEMP[2] = aDCtEMP[1];
        aDCtEMP[1] = aDCtEMP[0];
        aDCtEMP[0] = ADCANY4;
        sMOOTH1ST[0] = ((aDCtEMP[0]*14+aDCtEMP[2]*36+aDCtEMP[1]*25)/75);
        sMOOTH1ST[1] = ((aDCtEMP[1]*15+aDCtEMP[3]*35+aDCtEMP[2]*25)/75);
        sMOOTH1ST[2] = ((aDCtEMP[2]*16+aDCtEMP[4]*34+aDCtEMP[3]*25)/75);
        sMOOTH1ST[3] = ((aDCtEMP[3]*17+aDCtEMP[5]*33+aDCtEMP[4]*25)/75);
        sMOOTH1ST[4] = ((aDCtEMP[4]*18+aDCtEMP[6]*32+aDCtEMP[5]*25)/75);
        sMOOTH2RD[0] = ((sMOOTH1ST[0]*19+sMOOTH1ST[2]*31+sMOOTH1ST[1]*25)/75);
        sMOOTH2RD[1] = ((sMOOTH1ST[1]*20+sMOOTH1ST[3]*30+sMOOTH1ST[2]*25)/75);
        sMOOTH2RD[2] = ((sMOOTH1ST[2]*21+sMOOTH1ST[4]*29+sMOOTH1ST[3]*25)/75);
        eDGErESULT1ST = (aDCtEMP[0]+aDCtEMP[2])-(2*aDCtEMP[1]);
        //eDGErESULT1ST = (sMOOTH1ST[0]+sMOOTH1ST[2])-(2*sMOOTH1ST[1]);
        //eDGErESULT1ST = (sMOOTH1ST[1]+sMOOTH1ST[3])-(2*sMOOTH1ST[2])+eDGErESULT1ST;
        //eDGErESULT1ST = (sMOOTH1ST[2]+sMOOTH1ST[4])-(2*sMOOTH1ST[3])+eDGErESULT1ST;
        eDGErESULT2RD = (sMOOTH2RD[0]+sMOOTH2RD[2])-(2*sMOOTH2RD[1]);
        //eDGEsLOPE  = (sMOOTH2RD[2]-sMOOTH2RD[0])/2;
        //eDGEsLOPE  = (sMOOTH1ST[2]-sMOOTH1ST[0])/2;
        eDGEsLOPE  = (aDCtEMP[2]-aDCtEMP[0])/2;

        if ((eDGEsLOPE==0))
        {
          if (eDGErESULT1ST<0)
          {
            eDGEaDCmAX=aDCtEMP[1];
          }
          if (eDGErESULT1ST>0)
          {
            eDGEaDCmIN=aDCtEMP[1];
          }
          if (eDGEaDCmAX>eDGEaDCmIN) eDGEaDCdIF=eDGEaDCmAX-eDGEaDCmIN;
          else eDGEaDCdIF=eDGEaDCmIN-eDGEaDCmAX;
          
          eDGEaDCrEF=3;
          //if (eDGEaDCrEF<=3) eDGEaDCrEF=3;//2

          if ((eDGEaDCdIF>eDGEaDCrEF))
          {
            if (eDGEaDCcNT>=65535) eDGEaDCcNT=65535;
            else eDGEaDCcNT++;
            
            eDGEcOUNT=0;
            
            eDGEaDCdIFsUM=eDGEaDCdIFsUM+eDGEaDCdIF;
            
            P1OUT |= TEST3_PIN;
            
            for(LED_LOOP=0; LED_LOOP<=75; LED_LOOP++) { P1OUT ^= LED_OUT; }
            P1OUT &= ~LED_OUT;
            
            if (eDGEaDCcNT>=4)
            {
              TRIGGER_Count=0;            
              eDGEaDCcNT=0;
              eDGEaDCdIFsUM=0;
              eDGEaDCmAX=eDGEaDCmIN;
              eDGEaDCdIFsUM=0;
            }            
            eDGEaDCdIF=0;
          }
        }
        /*
        //^
        //if ((eDGEsLOPE>0)&&(eDGErESULT<0))
        if ((eDGEsLOPE>0))
        {
          if (eDGE1STcNTn==65535) eDGE1STcNTn=65535;
          else eDGE1STcNTn++;
          
          //P1OUT |= TEST3_PIN;
          eDGEcOUNT = 0;
          
          for(LED_LOOP=0; LED_LOOP<=75; LED_LOOP++) { P1OUT ^= LED_OUT; }
          P1OUT &= ~LED_OUT;
          
          //if ((eDGE1STcNTn>=eDGE1STcNTtRIG)&&(eDGE1STcNTu>=eDGE1STcNTtRIG)&&(eDGEaDCdIF>=1)) 
          if ((eDGE1STcNTn>=eDGE1STcNTtRIG)&&(eDGE1STcNTu>=eDGE1STcNTtRIG))
            TRIGGER_Count=0;
        }
        //v
        //if ((eDGEsLOPE<0)&&(eDGErESULT>0))
        if ((eDGEsLOPE<0))
        {
          if (eDGE1STcNTu==65535) eDGE1STcNTu=65535;
          else eDGE1STcNTu++;
          
          //P1OUT |= TEST1_PIN;
          eDGEcOUNT = 0;
          
          for(LED_LOOP=0; LED_LOOP<=75; LED_LOOP++) { P1OUT ^= LED_OUT; }
          P1OUT &= ~LED_OUT;
          
          //if ((eDGE1STcNTn>=eDGE1STcNTtRIG)&&(eDGE1STcNTu>=eDGE1STcNTtRIG)&&(eDGEaDCdIF>=1)) 
          if ((eDGE1STcNTn>=eDGE1STcNTtRIG)&&(eDGE1STcNTu>=eDGE1STcNTtRIG))
            TRIGGER_Count=0;
        }
        */
      }
      selectADC=0;
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
  WDT_Count ++;
  if (WDT_Count>=wDTtIMEoUT) 
  {
    WDT_Count=0; 

    #if (EAS_DH18)
      P2OUT ^= WDT_PIN;
    #endif
    #if (EAS_DH18_DEBUG)
      P2OUT ^= WDT_PIN;
    #endif
    #if(EAS_DH14)
      P1OUT ^= WDT_PIN;
    #endif
  }
 
  P1OUT &= ~TEST3_PIN;
  
  if (eDGEcOUNT < eDGEcOUNTlIMIT) eDGEcOUNT++;
  else 
  {
    eDGEcOUNT=eDGEcOUNTlIMIT;
    eDGE1STcNTu=0;
    eDGE1STcNTn=0;
    aDCtEMP[6] = aDCtEMP[5];
    aDCtEMP[5] = aDCtEMP[4];
    aDCtEMP[4] = aDCtEMP[3];
    aDCtEMP[3] = aDCtEMP[2];
    aDCtEMP[2] = aDCtEMP[1];
    aDCtEMP[1] = aDCtEMP[0];
    sMOOTH1ST[4] = sMOOTH1ST[3];
    sMOOTH1ST[3] = sMOOTH1ST[2];
    sMOOTH1ST[2] = sMOOTH1ST[1];
    sMOOTH1ST[1] = sMOOTH1ST[0];
    sMOOTH2RD[2] = sMOOTH2RD[1];
    sMOOTH2RD[1] = sMOOTH2RD[0];
    
    eDGEaDCcNT=0;
    
    eDGEaDCdIF=0;
    eDGEaDCmAX=eDGEaDCmIN;
    eDGEaDCdIF=0;
    /*
    if (eDGEaDCdIFsUM>=16)
    {
      TRIGGER_Count=0;
      eDGEaDCdIFsUM=0;
    }
    */
  }
  
  if(TRIGGER_Count>=tRIGtIMEoUT) 
  { 
    #if (EAS_DH18)
      P2OUT |= RELAY_ON;
    #endif
    #if (EAS_DH18_DEBUG)
      P2OUT |= RELAY_ON;
    #endif
    #if(EAS_DH14)
      P2OUT &= ~RELAY_ON;
    #endif
    
    TRIGGER_Count=tRIGtIMEoUT; 
    P1OUT &= ~LED_OUT;
  }
  else
  {
    #if (EAS_DH18)
      P2OUT &= ~RELAY_ON;
    #endif
    #if (EAS_DH18_DEBUG)
      P2OUT &= ~RELAY_ON;
    #endif
    #if(EAS_DH14)
      P2OUT |= RELAY_ON;
    #endif
    
    TRIGGER_Count++;
    LED_LOOP=1;
    for(LED_LOOP=0; LED_LOOP<=lEDfLICKER; LED_LOOP++) { P1OUT ^= LED_OUT; }
    P1OUT &= ~LED_OUT;    
  } 
  
  selectADC=4;
  
  #if (EAS_DH18)
    SD16INCTL0 |= SD16INCH_4;                 // Enable channel A4+
    SD16AE = SD16AE1 + SD16AE2 ;
  #endif
  #if (EAS_DH18_DEBUG)
    SD16INCTL0 |= SD16INCH_2;                 // Enable channel A4+
    SD16AE = SD16AE4 + SD16AE5 ;
  #endif
  #if(EAS_DH14)
    SD16INCTL0 |= SD16INCH_4;                 // Enable channel A4+
    SD16AE = SD16AE1 + SD16AE2 ;
  #endif
  
  
  SD16CTL |= SD16REFON;                     // If no, turn on SD16_A ref   
  SD16CCTL0 |= SD16SC;                      // Set bit to start new conversion
  
  __bic_SR_register_on_exit(SCG1+SCG0);     // Keep DCO & SMCLK on after reti
  
}
