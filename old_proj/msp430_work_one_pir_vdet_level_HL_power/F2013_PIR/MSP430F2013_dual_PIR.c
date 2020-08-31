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
#define   TEST3_PIN        BIT7                // P1.7 test3 pin 
#define   LED_OUT          BIT0                // P1.0 =>Bit location for LED
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

static int  aDCtEMP[5] = {128,128,128,128,128};
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
static int  eDGEfREQtATOL = 0;

//static int  eDGEpOS = 1;
//static int  eDGEnEG = -1;

static int  aDClIMIT = 1;
//static int  aDClIMITtEMP = 0;
static unsigned char  aDClIMITcNT = 0;
static int  aDCiNTEGRAL = 0;
static int  aDCiNTEGRALlIMIT = 4;//8
static int  aDCiNTEGRALlIMITaDJ = 1;//12
//static int  aDCiNTEGRALbASE = 4;
static int  temp1=0;
static int  temp2=0;
static int  temp3=0;

int limitFB(int limit, int amplitude, int base)
{
  if (amplitude > limit) limit=limit+base;
  if (amplitude < limit) limit=limit-base;
  if (amplitude = limit) limit=limit;
  return limit;
}

int tABLaMPfREQ(int frequency, int amplitude)
{
  int temp;
  if (frequency<10)
  {
    switch (frequency)
    {
    /*  
      case  3: temp=(amplitude*10)/3;   break;
      case  4: temp=(amplitude*10)/4;   break;
      case  5: temp=(amplitude*10)/5;   break;
      case  6: temp=(amplitude*10)/6;   break;
      case  7: temp=(amplitude*10)/7;   break;
      case  8: temp=(amplitude*10)/8;   break;
      case  9: temp=(amplitude*10)/9;   break;
      */
      //default: temp=(amplitude*15)/20;          break;
      default: temp=amplitude;          break;
    }
  }
  
  else
  {
    temp=amplitude;
  }
  
  /*
  else
  {
    frequency=frequency/10;
    switch (frequency)
    {
      case  1: temp=amplitude;          break;
      case  2: temp=(amplitude);      break;
      case  3: temp=(amplitude);      break;
      case  4: temp=(amplitude*15)/20;      break;
      case  5: temp=(amplitude*15)/20;      break;
      default: temp=amplitude;          break;
    }
  }
  */
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
  
  //unsigned int init_led_onoff_time = 0;
  //unsigned int init_led_onoff_cycle = 0;
  
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL;                   
  BCSCTL1 = CALBC1_1MHZ;                                    // Set DCO to 1MHz   
  DCOCTL = CALDCO_1MHZ;   
  BCSCTL1 |= DIVA_0;                                        // ACLK = VLO/1
  BCSCTL3 |= LFXT1S_2;
  
  P1SEL = 0x08;

  P1DIR |= LED_OUT;  P1SEL &= ~LED_OUT;  P1OUT &= ~LED_OUT;  P1REN &= ~LED_OUT;  
  P2DIR |= RELAY_ON; P2SEL &= ~RELAY_ON; P2OUT |= RELAY_ON;  P2REN &= ~RELAY_ON;
  P2DIR |= WDT_PIN;  P2SEL &= ~WDT_PIN;  P2OUT |= WDT_PIN;   P2REN &= ~WDT_PIN;

  P1DIR |= TEST1_PIN; P1SEL &= ~TEST1_PIN; P1OUT &= ~TEST1_PIN; P1REN &= ~TEST1_PIN; 
  P1DIR |= TEST2_PIN; P1SEL &= ~TEST2_PIN; P1OUT &= ~TEST2_PIN; P1REN &= ~TEST2_PIN;
  P1DIR |= TEST3_PIN; P1SEL &= ~TEST3_PIN; P1OUT &= ~TEST3_PIN; P1REN &= ~TEST3_PIN;
   
  SD16CTL = SD16LP + SD16VMIDON + SD16REFON + SD16SSEL_1;   // 1.2V ref, SMCLK   
  SD16INCTL0 = SD16GAIN_1 + SD16INCH_4;                     // PGA = 4x, Diff inputs A4- & A4+   
  SD16CCTL0 =  SD16SNGL + SD16IE + SD16OSR_32;              // Single conversion, 256OSR, Int enable   
  SD16CTL &= ~SD16VMIDON;                                   // VMID off: used to settle ref cap   
  //SD16AE = SD16AE1 + SD16AE6 ;                              // P1.1_A4+, P1.6_A3+ SD16_A inputs  
  SD16AE = SD16AE1 ;                              
  

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
  
  /*
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
  */
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
      ADCANY4 = Result4_old>>8;               // adc shift 8 bits;
      if (ADCANY4==aDCtEMP[0]) {}
      else{
      aDCtEMP[4] = aDCtEMP[3];
      aDCtEMP[3] = aDCtEMP[2];
      aDCtEMP[2] = aDCtEMP[1];
      aDCtEMP[1] = aDCtEMP[0];
      aDCtEMP[0] = ADCANY4;
      /*
      eDGEdET[0] = ((aDCtEMP[0]+aDCtEMP[2])-(2*aDCtEMP[1]));
      eDGEdET[1] = ((aDCtEMP[1]+aDCtEMP[3])-(2*aDCtEMP[2]));
      eDGEdET[2] = ((aDCtEMP[2]+aDCtEMP[4])-(2*aDCtEMP[3])); 
      eDGErESULT = eDGEdET[0]+eDGEdET[1]+eDGEdET[2];
      */
      
      eDGEdET[0] = ((aDCtEMP[0]*2+aDCtEMP[2]*2+aDCtEMP[1]*4)/8);
      eDGEdET[1] = ((aDCtEMP[1]*2+aDCtEMP[3]*2+aDCtEMP[2]*4)/8);
      eDGEdET[2] = ((aDCtEMP[2]*2+aDCtEMP[4]*2+aDCtEMP[3]*4)/8); 
      eDGErESULT = (eDGEdET[0]+eDGEdET[2])-(2*eDGEdET[1]);
      eDGErESULToLD=(eDGEdET[2]-eDGEdET[0])/2;
      }
      /*
      //^
      //if ((eDGErESULT<=-1)&&(aDCtEMP[2]>aDCtEMP[0])&&(aDCtEMP[2]>aDCtEMP[4]))
      if ((eDGErESULT<=-1)&&(aDCtEMP[2]>aDCtEMP[0]))
      {
        if (eDGErESULToLD>0)
        {
          if (aDCaVR>=aDCtEMP[2]) aDCaMP = aDCaVR-aDCtEMP[2];
          else                    aDCaMP = aDCtEMP[2]-aDCaVR;
          //if(aDCaMP>=1)
          {
            eDGEfREQ = bASEfREQ/(eDGEcOUNT);
            P1OUT |= TEST3_PIN;
            eDGEcOUNT = 0;
          }          
        }
        eDGErESULToLD = eDGErESULT;
        eDGEdET[1] = 0;
      }
      else if ((eDGErESULT>=1)&&(aDCtEMP[2]<aDCtEMP[0]))
      {
        if (eDGErESULToLD<=0)
        {
          if (aDCaVR>=aDCtEMP[2]) aDCaMP = aDCaVR-aDCtEMP[2];
          else                    aDCaMP = aDCtEMP[2]-aDCaVR;
          //if(aDCaMP>=1)
          {
            eDGEfREQ = bASEfREQ/(eDGEcOUNT);
            P1OUT |= TEST3_PIN;
            eDGEcOUNT = 0;
          }          
        }
        eDGErESULToLD = eDGErESULT;
        eDGEdET[1] = 0;
      }
      */
      //if ((eDGErESULT<=-1)&&(aDCtEMP[2]>aDCtEMP[0])&&(aDCtEMP[2]>aDCtEMP[4]))
      //if ((eDGErESULT<=-1)&&(aDCtEMP[2]>aDCtEMP[0]))
      //if ((eDGErESULT<=-2)&&(eDGEdET[1]>eDGEdET[0]))
      //if ((eDGErESULT<=-2)&&(eDGErESULToLD>0))
      if ((eDGErESULToLD>0))
      {
        //if (eDGErESULToLD>=0)
        {  
          if (aDCaVR>=aDCtEMP[2]) aDCaMP = aDCaVR-aDCtEMP[2];
          else                    aDCaMP = aDCtEMP[2]-aDCaVR;
          eDGEfREQ = bASEfREQ/(eDGEcOUNT);
          //aDCaMP = tABLaMPfREQ( eDGEfREQ, aDCaMP );
          P1OUT |= TEST3_PIN;
          eDGEcOUNT = 0;
          //for(LED_LOOP=0; LED_LOOP<=75; LED_LOOP++) { P1OUT ^= LED_OUT; }
          //P1OUT &= ~LED_OUT;
        }
        //eDGErESULToLD = eDGErESULT;
        //eDGEdET[1] = 0;
      }
      //v
      //else if ((eDGErESULT>=1)&&(aDCtEMP[2]<aDCtEMP[0]))
      //else if ((eDGErESULT>=1)&&(aDCtEMP[2]<aDCtEMP[0])&&(aDCtEMP[2]<aDCtEMP[4]))
      //else if ((eDGErESULT>=2)&&(eDGEdET[1]<eDGEdET[0]))
      //else if ((eDGErESULT>=2)&&(eDGErESULToLD<0))
      else if ((eDGErESULToLD<0))
      {
        //if (eDGErESULToLD<0)
        {   
          if (aDCaVR>=aDCtEMP[2]) aDCaMP = aDCaVR-aDCtEMP[2];
          else     aDCaMP = aDCtEMP[2]-aDCaVR;
          eDGEfREQ = bASEfREQ/(eDGEcOUNT);
          //aDCaMP = tABLaMPfREQ( eDGEfREQ, aDCaMP );
          P1OUT |= TEST3_PIN;
          eDGEcOUNT = 0;
          //for(LED_LOOP=0; LED_LOOP<=75; LED_LOOP++) { P1OUT ^= LED_OUT; }
          //P1OUT &= ~LED_OUT;
        }
        //eDGErESULToLD = eDGErESULT;
        //eDGEdET[1] = 0;
      }
      
      else 
      {
        //aDCaVR=(aDCaVR+ADCANY4)/2;
        aDCaVR=128;
      }
      
      
      
      if ((eDGEfREQ>=3)&&(eDGEfREQ<=55)&&(aDCaMP>=(aDClIMIT)))      //0.3Hz~5Hz
      {          
        if(aDCrESULT[0]==aDCaMP)
        {}
        else
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
          
          aDCiNTEGRAL   = (aDCaMP+aDCiNTEGRAL);
          eDGEfREQtATOL = (eDGEfREQ+eDGEfREQtATOL);
          temp3 = aDCiNTEGRAL + eDGEfREQtATOL;
          temp1 = aDCiNTEGRAL;
          
          //if (aDClIMITcNT>=5) aDClIMITcNT=5;
          //else                aDClIMITcNT++;    
          if (aDClIMITcNT>=10) aDClIMITcNT=10;
          else                aDClIMITcNT++;
          //temp2=limitFB(aDCiNTEGRALlIMIT,aDCiNTEGRAL,2);
          //aDCiNTEGRALlIMIT=(aDCiNTEGRALlIMIT+temp2)/2;
          //temp2=0;
          
          //aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+4;        
          //if (aDCiNTEGRALlIMITaDJ<=1) aDCiNTEGRALlIMITaDJ=1;//12,5
          //if (aDCiNTEGRALlIMITaDJ>=18) aDCiNTEGRALlIMITaDJ=18;//25
        
          aDCaMP=0;
          eDGEfREQ=0;
        } 
        
        //for(LED_LOOP=0; LED_LOOP<=75; LED_LOOP++) { P1OUT ^= LED_OUT; }
        //P1OUT &= ~LED_OUT;
        
        aDCaMP=0;
        eDGEfREQ=0;
        temp2=limitFB(aDCiNTEGRALlIMIT,temp1,2);
        aDCiNTEGRALlIMIT=(aDCiNTEGRALlIMIT+temp2)/2;
        temp2=0;         
        //temp2=limitFB(aDClIMIT,aDCaMP,2);
        //aDClIMIT=(aDClIMIT+temp2)/2;
        //temp2=0;
        //if (aDClIMIT<=1) aDClIMIT=1;
        aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*15/10;
        
        
        //aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+4;        
        if (aDCiNTEGRALlIMITaDJ<=16) aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+4;//12,5
        //if (aDCiNTEGRALlIMITaDJ>=18) aDCiNTEGRALlIMITaDJ=18;//25
        
        //if((aDClIMITcNT>=1)&&(aDClIMITcNT<=4))
        {                        
          
          //if((aDCiNTEGRAL>aDCiNTEGRALlIMITaDJ)&&(temp1>20))
          //if((aDCiNTEGRAL>aDCiNTEGRALlIMITaDJ))
          //if((aDCiNTEGRAL>aDCiNTEGRALlIMITaDJ) || (temp3>56))
          
          {
            //if ((eDGEfREQtATOL/aDCiNTEGRAL)<3)
            {
              //TRIGGER_Count=0;
              //temp3 = 0;
            }
          }
          
          /*
          switch(aDClIMITcNT)
          {
            case 1:              
              if (eDGEfREQtATOL<=20)
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+1;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) 
                {
                  TRIGGER_Count=0;
                  P1OUT &= ~TEST1_PIN;                        
                  P1OUT &= ~TEST2_PIN;
                  //P1OUT &= ~TEST3_PIN;
                }
              }
              if (eDGEfREQtATOL>20)
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+2;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) 
                {
                  TRIGGER_Count=0;
                  P1OUT &= ~TEST1_PIN;                        
                  P1OUT &= ~TEST2_PIN;
                  //P1OUT &= ~TEST3_PIN;
                }
              }
              break;
              
            case 2:
 
              if (eDGEfREQtATOL<=40)
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+2;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ)
                {
                  TRIGGER_Count=0;
                  P1OUT |= TEST1_PIN;                        
                  P1OUT &= ~TEST2_PIN;
                  //P1OUT &= ~TEST3_PIN;
                }
              }
              if (eDGEfREQtATOL>40)
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+3;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) 
                {
                  TRIGGER_Count=0;
                  P1OUT |= TEST1_PIN;                        
                  P1OUT &= ~TEST2_PIN;
                  //P1OUT &= ~TEST3_PIN;
                }
              }
              break;
              
            case 3:
              if (eDGEfREQtATOL<=60)
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+3;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) 
                {
                  TRIGGER_Count=0;
                  P1OUT &= ~TEST1_PIN;                        
                  P1OUT |= TEST2_PIN;
                  //P1OUT &= ~TEST3_PIN;
                }
              }
              if (eDGEfREQtATOL>60)
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+4;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) 
                {
                  TRIGGER_Count=0;
                  P1OUT &= ~TEST1_PIN;                        
                  P1OUT |= TEST2_PIN;
                  //P1OUT &= ~TEST3_PIN;
                }
              }
              break;
              
            case 4:
              if ((eDGEfREQtATOL<=100)&&(eDGEfREQtATOL>=40))
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+4;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) 
                {
                  TRIGGER_Count=0;
                  P1OUT &= ~TEST1_PIN;                        
                  P1OUT &= ~TEST2_PIN;
                  //P1OUT |= TEST3_PIN;
                }
              }
              if (eDGEfREQtATOL<40)
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+3;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) 
                {
                  TRIGGER_Count=0;
                  P1OUT &= ~TEST1_PIN;                        
                  P1OUT &= ~TEST2_PIN;
                  //P1OUT |= TEST3_PIN;
                }
              }            
              if (eDGEfREQtATOL>100)
              {
                aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT+5;
                if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) 
                {
                  TRIGGER_Count=0;
                  P1OUT &= ~TEST1_PIN;                        
                  P1OUT &= ~TEST2_PIN;
                  //P1OUT |= TEST3_PIN;
                }
              }
              break;
              
            case 5:
              eDGEfREQtATOL=0;
              aDCiNTEGRAL=0;
              aDClIMITcNT=0;
              //aDCrESULT[0]=0; aDCrESULT[1]=0; aDCrESULT[2]=0; aDCrESULT[3]=0; aDCrESULT[4]=0;
              break;
    
            default :
              eDGEfREQtATOL=0;
              aDCiNTEGRAL=0;
              aDClIMITcNT=0;
              //aDCrESULT[0]=0; aDCrESULT[1]=0; aDCrESULT[2]=0; aDCrESULT[3]=0; aDCrESULT[4]=0;
              break;
          }
          */
          /*
          if (aDClIMITcNT==1) 
          {
            if (eDGEfREQtATOL<=30)
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*15/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }
            if (eDGEfREQtATOL>30)
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*20/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }
          }    
          
          if (aDClIMITcNT==2) 
          {
            if (eDGEfREQtATOL<=60)
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*15/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }
            if (eDGEfREQtATOL>60)
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*20/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }
          }
          
          if (aDClIMITcNT==3) 
          {
            
            if (eDGEfREQtATOL<=90)
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*15/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }
            if (eDGEfREQtATOL>90)
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*20/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }
          }
          if (aDClIMITcNT==4) 
          {
            if ((eDGEfREQtATOL<=150)&&(eDGEfREQtATOL>=60))
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*15/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }
            if (eDGEfREQtATOL<60)
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*12/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }            
            if (eDGEfREQtATOL>150)
            {
              aDCiNTEGRALlIMITaDJ=aDCiNTEGRALlIMIT*20/10;
              if(aDCiNTEGRAL>=aDCiNTEGRALlIMITaDJ) TRIGGER_Count=0;
            }
          }
          */
          //eDGEfREQtATOL=0;
          //aDCiNTEGRAL=0;
          //aDClIMITcNT=0;
        }
        if(aDClIMITcNT>=10)
        //if(aDClIMITcNT>=5)
        {
          aDClIMITcNT=0;
          eDGEfREQtATOL=0;
          aDCiNTEGRAL=0;
          aDClIMITcNT=0;
          //aDCrESULT[0]=0; aDCrESULT[1]=0; aDCrESULT[2]=0; aDCrESULT[3]=0; aDCrESULT[4]=0;
        }
      }
      else
      {
        eDGEfREQ=0;
        eDGErESULT=0;
        aDCaMP=0;
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
    P2OUT ^= WDT_PIN;    
  }
 
  P1OUT &= ~TEST3_PIN;
  //P1OUT |= SPICS_PIN; //for debug
  
  if(TRIGGER_Count>=tRIGtIMEoUT) 
  { 
    P2OUT |= RELAY_ON; 
    TRIGGER_Count=tRIGtIMEoUT; 
    P1OUT &= ~LED_OUT;
    
    if (eDGEcOUNT < bASEfREQ) eDGEcOUNT++;
    else 
    {
      eDGEcOUNT = 0;
      eDGEfREQtATOL=0;
      aDCiNTEGRAL=0;
      aDClIMITcNT=0;
      eDGEfREQ=3;
      eDGErESULT=0;
      aDCaMP=1;
 
      //for(LED_LOOP=0; LED_LOOP<=75; LED_LOOP++) { P1OUT ^= LED_OUT; }
      //P1OUT &= ~LED_OUT;
    }
    selectADC=4;
    SD16INCTL0 |= SD16INCH_4;                 // Enable channel A4+
    SD16CTL |= SD16REFON;                     // If no, turn on SD16_A ref   
    SD16CCTL0 |= SD16SC;                      // Set bit to start new conversion   
  }
  else
  {
    P2OUT &= ~RELAY_ON;
    TRIGGER_Count++;
    LED_LOOP=1;
    for(LED_LOOP=0; LED_LOOP<=lEDfLICKER; LED_LOOP++) { P1OUT ^= LED_OUT; }
    P1OUT &= ~LED_OUT;
    eDGEfREQtATOL=0;
    aDCiNTEGRAL=0;
    aDClIMITcNT=0;
  } 
  
  __bic_SR_register_on_exit(SCG1+SCG0);     // Keep DCO & SMCLK on after reti
  
}
