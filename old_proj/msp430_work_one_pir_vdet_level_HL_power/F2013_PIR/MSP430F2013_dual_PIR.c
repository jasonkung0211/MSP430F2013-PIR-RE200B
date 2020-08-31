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
#define   wDTtIMEoUT         86                //86x5.8ms=0.5S


static unsigned char  LED_LOOP=0;
static unsigned char  WDT_Count = 0;
static unsigned char  TRIGGER_Count = 0;

static unsigned int Result4_old = 35840;                    // Storage for last conversion
static unsigned int ADCANY4= 128;                           // adc/2^8
static unsigned char selectADC = 4;                         //P1.1_A4+

static unsigned char lEDfLICKER = 0;                        // Information memory B @0x1002
static unsigned char tRIGtIMEoUT = 0;                       // Information memory B @0x1004

static int  aDCtEMP[5] = {64,64,64,64,64};
static int  eDGEdET[3] = {0,0,0};
static int  eDGErESULT = 0;
static int  eDGErESULToLD = 0;
static int  eDGEcOUNT = 0;
static int  eDGEfREQ = 0;
static int  bASEfREQ = 1*1000*10/2/6;                       //1*1000=1000mSec,6mSec=scale(10K~12K(VLO)/64(WDTIS1+WDTIS0)),2=2*(1/2)duty
static int  eDGEfREQtEMP[5] = {0,0,0,0,0};
static int  aDCrESULT[5] = {0,0,0,0,0};
static int  aDCaMP=0;
static int  aDCaVR = 128;

static int  eDGEpOS = 1;
static int  eDGEnEG = -1;

static int  aDClIMIT = 6;
static int  aDClIMITtEMP = 0;
static int  aDClIMITcNT = 0;


int tABLaMPfREQ(int frequency, int amplitude)
{
  int temp;
  if (frequency<10)
  {
    switch (frequency)
    {
      case  3: temp=(amplitude*10)/3;   break;
      case  4: temp=(amplitude*10)/4;   break;
      case  5: temp=(amplitude*10)/5;   break;
      case  6: temp=(amplitude*10)/6;   break;
      case  7: temp=(amplitude*10)/7;   break;
      case  8: temp=(amplitude*10)/8;   break;
      case  9: temp=(amplitude*10)/9;   break;
      default: temp=amplitude;          break;
    }
  }
  else
  {
    frequency=frequency/10;
    switch (frequency)
    {
      case  1: temp=amplitude;          break;
      case  2: temp=(amplitude*2);      break;
      case  3: temp=(amplitude*3);      break;
      case  4: temp=(amplitude*4);      break;
      case  5: temp=(amplitude*5);      break;
      default: temp=amplitude;          break;
    }
  }
  return temp;
}


/*
int tABLaMPfREQ(int frequency, int amplitude)
{
  int temp;
  if (frequency<10)
  {
    switch (frequency)
    {
      case  3: temp=(amplitude*45)/10;  break; //case  3: temp=(amplitude*89)/10;  break;
      case  4: temp=(amplitude*22)/10;  break; //case  4: temp=(amplitude*44)/10;  break;
      case  5: temp=(amplitude*12)/10;  break; //case  5: temp=(amplitude*25)/10;  break;
      case  6: temp=(amplitude*11)/10;  break;
      case  7: temp=(amplitude*11)/10;  break;
      case  8: temp=(amplitude*10)/10;  break;
      case  9: temp=amplitude;          break;
      default: temp=amplitude;          break;
    }
  }
  else
  {
    frequency=frequency/10;
    switch (frequency)
    {
      case  1: temp=amplitude;          break;
      case  2: temp=(amplitude*14)/10;  break;
      case  3: temp=(amplitude*20)/10;  break; //case  3: temp=(amplitude*28)/10;  break;
      case  4: temp=(amplitude*40)/10;  break; //case  4: temp=(amplitude*56)/10;  break;
      case  5: temp=(amplitude*80)/10; break; //case  5: temp=(amplitude*100)/10; break;
      default: temp=amplitude;          break;
    }
  }
  return temp;
}
*/

void main(void)

{
  
  unsigned int init_led_onoff_time = 0;
  unsigned int init_led_onoff_cycle = 0;
  
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL;                   
  BCSCTL1 = CALBC1_1MHZ;                                    // Set DCO to 1MHz   
  DCOCTL = CALDCO_1MHZ;   
  BCSCTL1 |= DIVA_0;                                        // ACLK = VLO/1
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
  P1OUT |= TEST_PIN;                          //for test
  
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
  int i=0;
  SD16CTL &= ~SD16REFON;                      // Turn off SD16_A ref
  switch(selectADC)
  {
    //A4+ for PIR1  
    case 4:
      Result4_old = SD16MEM0;                 // Save result (clears IFG)  
      ADCANY4 = Result4_old>>8;               // adc shift 8 bits;
      if (ADCANY4==aDCtEMP[0]) {}
      else{
      aDCtEMP[4] = aDCtEMP[3];
      aDCtEMP[3] = aDCtEMP[2];
      aDCtEMP[2] = aDCtEMP[1];
      aDCtEMP[1] = aDCtEMP[0];
      aDCtEMP[0] = ADCANY4;      
      eDGEdET[0] = ((aDCtEMP[0]+aDCtEMP[2])-(2*aDCtEMP[1]));
      eDGEdET[1] = ((aDCtEMP[1]+aDCtEMP[3])-(2*aDCtEMP[2]));
      eDGEdET[2] = ((aDCtEMP[2]+aDCtEMP[4])-(2*aDCtEMP[3])); 
      }
      //^
      //if ((eDGEdET[0]>=0)&&(eDGEdET[2]<=eDGEnEG)&&(eDGEdET[3]>=0)&&(eDGEdET[1]<=eDGEnEG))
      if ((eDGEdET[0]>=-1)&&(eDGEdET[2]>=-1)&&(eDGEdET[1]<=-2)&&(aDCtEMP[2]>aDCtEMP[0]))
      //if ((eDGEdET[0]>=-1)&&(eDGEdET[2]>=-1)&&(eDGEdET[1]<=-2))
      //if (eDGErESULT<=eDGEnEG)
      //if (eDGEdET[1]<=eDGEnEG)
      //if ((aDCtEMP[3]>=aDCtEMP[4])&&(aDCtEMP[3]>aDCtEMP[5])&&(aDCtEMP[2]>=aDCtEMP[1])&&(aDCtEMP[2]>aDCtEMP[0]))
      {
        if (eDGErESULToLD>=0)
        {  
          //if ((aDCaMP>=aDClIMIT)&&(aDCtEMP[2]>aDCtEMP[1])&&(aDCtEMP[2]>aDCtEMP[0]))
          {  
          //eDGEfREQ = bASEfREQ/(eDGEcOUNT);
          if (aDCaVR>=aDCtEMP[2]) aDCaMP = aDCaVR-aDCtEMP[2];
          else     aDCaMP = aDCtEMP[2]-aDCaVR;
          //aDCaMP = tABLaMPfREQ( eDGEfREQ, aDCaMP );
          //eDGEcOUNT = 0;
          }
          if (aDCaMP>=aDClIMIT) 
          {
            eDGEfREQ = bASEfREQ/(eDGEcOUNT);
            //if (aDCaVR>=aDCtEMP[2]) aDCaMP = aDCaVR-aDCtEMP[2];
            //else     aDCaMP = aDCtEMP[2]-aDCaVR;
            //aDCaMP = tABLaMPfREQ( eDGEfREQ, aDCaMP );            
            P1OUT &= ~TEST_PIN;
            P1OUT &= ~SPICS_PIN;
            eDGEcOUNT = 0;
          }
        }
        //eDGErESULToLD = eDGErESULT;
        eDGErESULToLD = -1;        
      }
      //v
      //else if ((eDGEdET[0]<=0)&&(eDGEdET[2]>=eDGEpOS)&&(eDGEdET[3]<=0)&&(eDGEdET[1]>=eDGEpOS))
      else if ((eDGEdET[0]<=1)&&(eDGEdET[2]<=1)&&(eDGEdET[1]>=2)&&(aDCtEMP[2]<aDCtEMP[0]))
      //else if ((eDGEdET[0]<=1)&&(eDGEdET[2]<=1)&&(eDGEdET[1]>=2))
      //else if (eDGErESULT>=eDGEpOS)
      //else if (eDGEdET[1]>=eDGEpOS)
      //else if ((aDCtEMP[3]<=aDCtEMP[4])&&(aDCtEMP[3]<aDCtEMP[5])&&(aDCtEMP[2]<=aDCtEMP[1])&&(aDCtEMP[2]<aDCtEMP[0]))
      {
        if (eDGErESULToLD<0)
        {   
          
          //if ((aDCaMP>=aDClIMIT)&&(aDCtEMP[2]<aDCtEMP[1])&&(aDCtEMP[2]<aDCtEMP[0]))
          {
          //eDGEfREQ = bASEfREQ/(eDGEcOUNT);
          if (aDCaVR>=aDCtEMP[2]) aDCaMP = aDCaVR-aDCtEMP[2];
          else     aDCaMP = aDCtEMP[2]-aDCaVR;
          //aDCaMP = tABLaMPfREQ( eDGEfREQ, aDCaMP );
          //eDGEcOUNT = 0;  
          }
          if (aDCaMP>=aDClIMIT) 
          {
            eDGEfREQ = bASEfREQ/(eDGEcOUNT);
            //if (aDCaVR>=aDCtEMP[2]) aDCaMP = aDCaVR-aDCtEMP[2];
            //else     aDCaMP = aDCtEMP[2]-aDCaVR;
            //aDCaMP = tABLaMPfREQ( eDGEfREQ, aDCaMP );
            P1OUT &= ~TEST_PIN;
            P1OUT &= ~SPICS_PIN;
            eDGEcOUNT = 0;
          }
        }
        //eDGErESULToLD = eDGErESULT;
        eDGErESULToLD = 1;
      }
      
      else 
      {
        aDCaVR = (aDCtEMP[0]+aDCtEMP[1]+aDCtEMP[2]+aDCtEMP[3]+aDCtEMP[4])/5;
        //aDCaVR=128;
      }
      
      if ((eDGEfREQ>=3)&&(eDGEfREQ<=55)&&(aDCaMP>=aDClIMIT))      //0.3Hz~5Hz
      {          
         
          eDGEfREQtEMP[4]  = eDGEfREQtEMP[3];
          eDGEfREQtEMP[3]  = eDGEfREQtEMP[2];
          eDGEfREQtEMP[2]  = eDGEfREQtEMP[1];
          eDGEfREQtEMP[1]  = eDGEfREQtEMP[0];
          eDGEfREQtEMP[0]  = eDGEfREQ;   
          aDCrESULT[4]     = aDCrESULT[3];
          aDCrESULT[3]     = aDCrESULT[2];
          aDCrESULT[2]     = aDCrESULT[1];
          aDCrESULT[1]     = aDCrESULT[0];
          aDCrESULT[0]     = aDCaMP;
        
          if (aDClIMITcNT>=5) aDClIMITcNT=5;
          else                aDClIMITcNT++;
          for (i=0; i<aDClIMITcNT; i++)
          {
            aDClIMITtEMP=aDClIMITtEMP+aDCrESULT[i];
          }
          aDClIMITtEMP=aDClIMITtEMP/aDClIMITcNT;
          aDClIMIT=(aDClIMITtEMP/5);
         
          if (aDClIMIT<=6) aDClIMIT = 6;
          eDGEcOUNT=0;
          eDGEfREQ=0;
          eDGErESULT=0;
          TRIGGER_Count=0;
          aDClIMITtEMP=0;
      }
      else
      //else if ((eDGEfREQ>55)||(eDGEfREQ<3))
      {
          //eDGEcOUNT=0;
          eDGEfREQ=0;
          eDGErESULT=0;
          aDClIMITtEMP=0;
      }
      
      __bis_SR_register_on_exit(SCG1+SCG0);   // Return to LPM3 after reti
    break;
    
    default :
      Result4_old = SD16MEM0;                 // Save result (clears IFG)  
      ADCANY4 = Result4_old>>8;               // adc shift 9 bits;
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
  WDT_Count ++; 
  P1OUT |= TEST_PIN;
  P1OUT |= SPICS_PIN; //for debug
  
  if (eDGEcOUNT < bASEfREQ) eDGEcOUNT++;
  else eDGEcOUNT = bASEfREQ;
 
  
  if(TRIGGER_Count>=tRIGtIMEoUT) 
  { 
    P2OUT |= RELAY_ON; TRIGGER_Count=tRIGtIMEoUT; 
  }
  else
  {
    P2OUT &= ~RELAY_ON;
    TRIGGER_Count++;
    for(LED_LOOP=0; LED_LOOP<=lEDfLICKER; LED_LOOP++) { P1OUT ^= LED_OUT; }
    P1OUT &= ~LED_OUT;
  } 
  

  if(WDT_Count>=wDTtIMEoUT)
  {
    P2OUT |= WDT_PIN;  WDT_Count=0;     
  }
  
  //if ((WDT_Count%1)==0)
  {
    //P1OUT |= TEST_PIN;
    selectADC=4;
    SD16INCTL0 |= SD16INCH_4;                 // Enable channel A4+
    SD16CTL |= SD16REFON;                     // If no, turn on SD16_A ref   
    SD16CCTL0 |= SD16SC;                      // Set bit to start new conversion   
    __bic_SR_register_on_exit(SCG1+SCG0);     // Keep DCO & SMCLK on after reti
  }
}
