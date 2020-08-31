//*****************************************************************************
//  Code for - "Dual Ultra-low Power Motion Detection
//    using the MSP430F2013"
//
//  Version 2-00
//
//  Version Summary:
//  1-00 released 12-22-2014- Initial release
//  2-00 released 9-24-2015- digital low pass filter
//
//  Built with IAR Embedded Workbench Version: 5.20
//*****************************************************************************

#include  <msp430x20x3.h>

#define   TEST_PIN        BIT0              // P1.0
#define   LED_OUT         BIT6              // P1.6 =>Bit location for LED
#define   LED2_OUT        BIT7              // P1.7 =>Bit location for LED2

#define   RELAY_ON        BIT6              // P2.6
//#define   SENSOR_PWR      BIT7              // Bit location for power to sensor
#define   fWvERSION       3                 // firm ware VERSION
#define   iNITtHREShOLD   28
#define   tHREShOLDsTEP   50                // after first threshold,for step count
#define   tHREShOLDsTEP2  50
#define   tHREShOLD       80                // Threshold for motion, default is 50
#define   tHREShOLD2      80
#define   lEDoNtIME       38                // 
#define   lED1cHECKtIME   120
#define   lED2oNtIME      68
#define   iLEDtIME        31250           // iNITION LED tIME 250ms flash one time
#define   iLEDcYCLE       120             //120 for 30sec, 40 for 10sec, Wait sensor stable
#define   nOISEnUMBER     200             //200 for MP
#define   pIR1tOPlEVEL    20000
#define   pIR2tOPlEVEL    20000
#define   pIR1lEVEL       5500            //High wave
#define   pIR2lEVEL       5500
#define   pIR1lOWlEVEL    1000
#define   pIR2lOWlEVEL    1000
#define   pIR1fREQ        7               //Long wave
#define   pIR2fREQ        7
#define   lONGpIR1fREQ    12              //extra Long wave
#define   lONGpIR2fREQ    12
#define   nOISEpIR1fREQ   3
#define   nOISEpIR2fREQ   3
#define   fINALeVEN       9               //triged via even 1 to 8

//#define   AD_size         10            //2-1for test, dump A/D noise used
//void AD_RECORD(unsigned int);           //2-2
void WAVE1_MODE(void);
void WAVE2_MODE(void);

//static unsigned int  AD_point=0;              //2-1for test, dump A/D noise used
//extern unsigned int  AD_area[AD_size+1]={0};  //2-2

extern unsigned int  pir_1_Hightest = 0;
extern unsigned int  pir_2_Hightest = 0;
extern unsigned char  Wave_1_Mode_No =0;
extern unsigned char  Wave_2_Mode_No =0;
extern unsigned char  Even_1_No =0;
extern unsigned char  Even_2_No =0;
extern unsigned char  pir_1_Trig_Counter = 0;
extern unsigned char  pir_2_Trig_Counter = 0;
extern unsigned char  pir_1_TC_Save = 0;          //Trig Counter Save
extern unsigned char  pir_2_TC_Save = 0;
extern unsigned char  High_Long_1=0;
extern unsigned char  High_Long_2=0;
extern unsigned char  Low_Long_1 =0;
extern unsigned char  Low_Long_2 =0;


static unsigned int Result_old = 50000;         // Storage for last conversion
static unsigned int Result2_old = 50000;    // Storage2 for last conversion(for first)
static unsigned char Ch_counter=0;
//static unsigned int  pir_1_Counter = 0;
//static unsigned int  pir_2_Counter = 0;
static unsigned char  en_1_First_Level = 1;
static unsigned char  en_2_First_Level = 1;
static unsigned int   pir_1_Noise_Sum=0;
static unsigned int   pir_2_Noise_Sum=0;
static unsigned char  pir_1_Noise_Point=0;
static unsigned char  pir_2_Noise_Point=0;
static unsigned int  Threshold_Dynamic = tHREShOLD;
static unsigned int  Threshold2_Dynamic = tHREShOLD2;







void main(void)

{
  unsigned int init_led_onoff_time = 0;
  unsigned int init_led_onoff_cycle = 0;
  unsigned int i_long =0;
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL; // ACLK/32768, int timer: ~10s
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
  DCOCTL = CALDCO_1MHZ;
  BCSCTL1 |= DIVA_2;                        // ACLK = VLO/4(default)
  BCSCTL3 |= LFXT1S_2;

  //P1OUT = 0x10;                             // P1OUTs
  P1OUT = 0x10;
  P1SEL = 0x08;                             // Select VREF function
  //P1DIR = 0xEF;                             // Unused pins as outputs
  P1DIR = 0xFF;
  P2SEL &= ~RELAY_ON;                        // P2.7(RELAY_ON) = GPIO
  P2DIR = 0xff;                             // Unused pins as outputs

  SD16CTL = SD16VMIDON + SD16REFON + SD16SSEL_1;// 1.2V ref, SMCLK
  SD16INCTL0 = SD16GAIN_1 + SD16INCH_4;     // PGA = 8x, Diff inputs A4- & A4+
  SD16CCTL0 =  SD16SNGL + SD16IE + SD16OSR_1024;           // Single conversion, 256OSR, Int enable
  SD16CTL &= ~SD16VMIDON;                   // VMID off: used to settle ref cap
  //SD16AE = SD16AE1 + SD16AE2;               // P1.1 & P1.2: A4+/- SD16_A inputs
  SD16AE = SD16AE1 + SD16AE2 +SD16AE4 + SD16AE5;

  // Wait for PIR sensor to settle: 1st WDT+ interval
  //P1SEL |= LED_OUT;                         // Turn LED on with ACLK (for low Icc)
  P1OUT |= LED_OUT;                         // Turn LED on
  P1OUT &=~LED2_OUT;
  P2OUT &=~RELAY_ON;
//  while(!(IFG1 & WDTIFG));                  // ~5.4s delay: PIR sensor settling
  for(init_led_onoff_cycle=0;init_led_onoff_cycle <=iLEDcYCLE;init_led_onoff_cycle++)
  {
    for(init_led_onoff_time=0;init_led_onoff_time <=iLEDtIME;init_led_onoff_time++)
    {
      if (init_led_onoff_cycle==(4))
        for(i_long=0;i_long <=6 ;i_long++)
        {}
    }
     P1OUT ^= LED_OUT;                        // Turn LED off with ACLK (for low Icc)
     P1OUT ^= LED2_OUT;
  }
  P1SEL &= ~LED_OUT;                        // Turn LED off with ACLK (for low Icc)
  P1OUT &= ~LED2_OUT;
  // Reconfig WDT+ for normal operation: interval of ~341msec
 //  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1;// ACLK/512, int timer: 341msec
   BCSCTL1 |= DIVA_1;                        // DIVA_3=>ACLK = VLO/8
  
  WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0;//11:ACLK/64 10:ACLK/512 TimeBase=47.2mS
  IE1 |= WDTIE;                             // Enable WDT interrupt

  _BIS_SR(LPM3_bits + GIE);                 // Enter LPM3 with interrupts
}

/******************************************************
// SD16_A interrupt service routine
******************************************************/
#pragma vector = SD16_VECTOR
__interrupt void SD16ISR(void)
{ unsigned int result_new;
  unsigned int result2_new;
 //// P2OUT ^= TEST_PIN;
  SD16CTL &= ~SD16REFON;                    // Turn off SD16_A ref
  
   switch(Ch_counter)
    {  
    case 0:
      result_new = SD16MEM0;                    // Save result (clears IFG)
      if (result_new > Result_old)              // Get difference between samples
      {
      Result_old = result_new - Result_old;

          if (Result_old > tHREShOLDsTEP)               // If motion detected...
          {
            if(en_1_First_Level)
            {
              if(Result_old > (Threshold_Dynamic + tHREShOLDsTEP))
              {
                en_1_First_Level = 0;
              }
//              AD_RECORD(Result_old);   //for test, dump A/D noise used
            }
            else
            {
              P1OUT |= LED_OUT;                      // Turn LED on
              pir_1_Noise_Point = 0;
              pir_1_Noise_Sum = 0;
              pir_2_Noise_Point = 0;      //2-1 avoid pir1 to effect pir2
              pir_2_Noise_Sum = 0;        //2-2     
              pir_1_Hightest = pir_1_Hightest + Result_old;
              pir_1_Trig_Counter++;
 //             AD_RECORD(Result2_old);   //for test, dump A/D trig level              
            }
          }
          
          if (pir_1_Noise_Point < nOISEnUMBER)
          {  
            pir_1_Noise_Sum = pir_1_Noise_Sum + Result_old;
            pir_1_Noise_Point++;
          }
          else
          {
            pir_1_Noise_Sum = pir_1_Noise_Sum / nOISEnUMBER;
            Threshold_Dynamic = pir_1_Noise_Sum * 2;
              if(Threshold_Dynamic < iNITtHREShOLD)
                  Threshold_Dynamic = iNITtHREShOLD;
            pir_1_Noise_Point = 0;
          }
          
 //         AD_RECORD(Result_old);   //for test, dump A/D noise used 
      }
      else
       {
        en_1_First_Level = 1;  //re-count first level when new<old(wave fall from top)
//        if(pir_1_Counter > 1)
        if(pir_1_Trig_Counter > 1)    
         {
          if(pir_1_Hightest > pIR1lOWlEVEL)
          WAVE1_MODE();        // got one wave to judge it
//          for(AD_point =0;AD_point < AD_size+1 ;AD_point++)
//          AD_area[AD_point]=0;    //for test : clear record buffer
//          AD_point =0;            //for test
          pir_1_Hightest = 0;
          pir_1_Trig_Counter =0;          
         }
                  
       }
     
      Result_old = SD16MEM0;                    // Save last conversion    
      SD16INCTL0 &= ~SD16INCH_4;          // Disable channel A4+/-
      Ch_counter++;
                        
      SD16INCTL0 |= SD16INCH_2;           // Enable channel A2+/- 
   
      
      break;

    case 1:
       result2_new = SD16MEM0;                    // Save result (clears IFG)
      if (result2_new > Result2_old)              // Get difference between samples
      {   
      Result2_old = result2_new - Result2_old;

          if (Result2_old > tHREShOLDsTEP2)               // If motion detected...
          {
          
            if(en_2_First_Level)
            {
              if(Result2_old > (Threshold2_Dynamic + tHREShOLDsTEP2))
              {
                en_2_First_Level = 0;
              }
//              AD_RECORD(Result2_old);   //for test, dump A/D noise used
            }
            else
            {
              P1OUT |= LED2_OUT;                      // Turn LED on
              pir_2_Noise_Point = 0;
              pir_2_Noise_Sum = 0;
              pir_1_Noise_Point = 0;      //2-1 avoid pir2 to effect pir1
              pir_1_Noise_Sum = 0;        //2-2               
              pir_2_Hightest = pir_2_Hightest + Result2_old;
              pir_2_Trig_Counter++;
//              AD_RECORD(Result2_old);   //for test, dump A/D trig level
            }
          }
          
          if (pir_2_Noise_Point < nOISEnUMBER)
          {  
            pir_2_Noise_Sum = pir_2_Noise_Sum + Result2_old;
            pir_2_Noise_Point++;
          }
          else
          {
            pir_2_Noise_Sum = pir_2_Noise_Sum / nOISEnUMBER;
            Threshold2_Dynamic = pir_2_Noise_Sum * 2;
              if(Threshold2_Dynamic < iNITtHREShOLD)
                  Threshold2_Dynamic = iNITtHREShOLD;            
            pir_2_Noise_Point = 0;
          }
          
//          AD_RECORD(Result2_old);   //for test, dump A/D noise used 
      }
      
      
      else
       {
        en_2_First_Level = 1;  //re-count first level when new<old
 //       if(pir_2_Counter > 1)
        if(pir_2_Trig_Counter > 1)
         {
          if(pir_2_Hightest > pIR2lOWlEVEL)
          WAVE2_MODE();              // got one wave to judge it(wave fall from top)
//          for(AD_point =0;AD_point < AD_size+1 ;AD_point++)
//          AD_area[AD_point]=0;    //for test : clear record buffer
//          AD_point =0;            //for test
          pir_2_Hightest = 0;
          pir_2_Trig_Counter =0;          
         }
                  
       }

      
      Result2_old = SD16MEM0;                    // Save last conversion     
      SD16INCTL0 &= ~SD16INCH_2;
      Ch_counter = 0;                     // Reset channel count (end of seq)
      SD16INCTL0 |= SD16INCH_4;
      break;      
    }
   
  __bis_SR_register_on_exit(SCG1+SCG0);     // Return to LPM3 after reti
}

/******************************************************
// Watchdog Timer interrupt service routine
******************************************************/
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)

{
  static unsigned int  pir_1_Timer = lEDoNtIME;           //around 2 second.
  static unsigned int  led_1_LoopCheck = lED1cHECKtIME;           //around 5 second.
  static unsigned char  en_pir_1_Counter=1;
//  static unsigned char  en_pir_1_Timer=0;
  static unsigned char  multi_timer = 0;
  static unsigned int  pir_2_Timer = lED2oNtIME;           //around 2 second.
  static unsigned char  en_pir_2_Counter=1;
//  static unsigned char  en_pir_2_Timer=0;
  static unsigned char   stop_pir_1_Detec=0;         //set PGA =1 for 4x50ms after flash LED,pir2 needn't due to same PGA
  
 
 //          if (pir_1_Counter >= pIR1tRIGcOUNT)       
        if ((Even_1_No >0) && (Even_1_No < fINALeVEN))  //Trig Even here for fast doing trig process
            {
              P1OUT |= LED_OUT;              // Triged by PIR_1
              P2OUT |= RELAY_ON;             
              Even_1_No =0;
              Wave_1_Mode_No =0;
              pir_1_TC_Save =0;
              High_Long_1 = 0;
              Low_Long_1 = 0;
              pir_1_Timer = lEDoNtIME;
              multi_timer = 1;
              en_pir_1_Counter = 0;
            } 
  
 // P1OUT ^= TEST_PIN;
      if(stop_pir_1_Detec <= 0)
      {  
 //       SD16INCTL0 &= ~SD16GAIN_1;
 //       SD16INCTL0 |= SD16GAIN_32;
        if (!(P1OUT & LED_OUT))                   // Has motion already been detected?
          {
          SD16CTL |= SD16REFON;                   // If no(no got motion signal), turn on SD16_A ref
          SD16CCTL0 |= SD16SC;                    // Set bit to start new conversion
         __bic_SR_register_on_exit(SCG1+SCG0);   // Keep DCO & SMCLK on after reti
          }
        else
          {  
          if (en_pir_1_Counter)                     // got triged, keep LED1 on
          P1OUT &= ~LED_OUT;                       // If yes(got motion signal), turn off LED, measure on next loop 
                             //Trig Even here will slow down trig process,it will interferenced by 6sec flash LED
          }
      }
      else
        stop_pir_1_Detec--;
////////////////////////////////////////////////////////////////        
    
        if ((Even_2_No >0) && (Even_2_No < fINALeVEN))
            {
              P1OUT |= LED2_OUT;              // Triged by PIR_2
              P2OUT |= RELAY_ON; 
 //             SD16INCTL0 &= ~SD16GAIN_32;
 //             SD16INCTL0 |= SD16GAIN_1;              
              Even_2_No =0;
              Wave_2_Mode_No =0;
              pir_2_TC_Save =0;
              High_Long_2 = 0;
              Low_Long_2 = 0;              
              pir_2_Timer = lED2oNtIME;
              multi_timer = 2;
              en_pir_2_Counter = 0;
            }    
    
      if (!(P1OUT & LED2_OUT))                   // Has motion already been detected?
        {
        SD16CTL |= SD16REFON;                   // If no, turn on SD16_A ref
        SD16CCTL0 |= SD16SC;                    // Set bit to start new conversion
        __bic_SR_register_on_exit(SCG1+SCG0);   // Keep DCO & SMCLK on after reti
        }
      else
        {  
        if (en_pir_2_Counter)        
          P1OUT &= ~LED2_OUT;               // If yes, turn off LED, measure on next loop 
                             //Trig Even here will slow down trig process,it will interferenced by 6sec flash LED     
        }
   
/*    
 if (!(P1OUT & LED2_OUT))                   // Has motion already been detected?
  {
    SD16CTL |= SD16REFON;                   // If no, turn on SD16_A ref
    SD16CCTL0 |= SD16SC;                    // Set bit to start new conversion
    __bic_SR_register_on_exit(SCG1+SCG0);   // Keep DCO & SMCLK on after reti
  }
  else
    P1OUT &= ~LED2_OUT;                      // If yes, turn off LED, measure on next loop
*/    
    
//******************************************    
  switch(multi_timer)
  {    
    case 0:                           // LED and LED2 flash one time period 6sec
      if (LED_OUT&&LED2_OUT)
      {P1OUT &= ~LED_OUT;
      P1OUT &= ~LED2_OUT;}
      led_1_LoopCheck--;
      if (led_1_LoopCheck <= 0)
      {
//        SD16INCTL0 &= ~SD16GAIN_32;
//        SD16INCTL0 |= SD16GAIN_1;
        stop_pir_1_Detec = 4;
        P1OUT |=  LED_OUT;
        P1OUT |=  LED2_OUT;  
        multi_timer = 0;
        led_1_LoopCheck = lED1cHECKtIME; 
        
        if(pir_1_TC_Save>0)
          pir_1_TC_Save--;             //reduce Triger 1 if no continue trig
        else
        {
          Even_1_No = 0;
          Wave_1_Mode_No =0;
        }
        
        if(pir_2_TC_Save>0)
          pir_2_TC_Save--;             //reduce Triger 2 if no continue trig
        else
        {
          Even_2_No = 0;
          Wave_2_Mode_No =0;
        }
        
        if(High_Long_1>0)
          High_Long_1--;
        if(Low_Long_1>0)
          Low_Long_1--;
        if(High_Long_2>0)
          High_Long_2--;
        if(Low_Long_2>0)
          Low_Long_2--;               
      }      
      
      break;
      
    case 1:
      //P1OUT |= LED_OUT;              // Triged by PIR1,turn LED off after 2sec
      pir_1_Timer--;
      if (pir_1_Timer <= 0)
      {
        P1OUT &= ~LED_OUT;
        P2OUT &= ~RELAY_ON;
        multi_timer = 0;
        led_1_LoopCheck = lED1cHECKtIME;
        en_pir_1_Counter = 1;
        pir_1_Timer = lEDoNtIME;
        en_pir_2_Counter = 1;           //avoide PIR2 to interrupt PIR1
        pir_2_Timer = lED2oNtIME;    //            
      }
      break;
      
    case 2:
      //P1OUT |= LED2_OUT;              // Triged by PIR2,turn LED2 off after 2sec       
      pir_2_Timer--;
      if (pir_2_Timer <= 0)
      {
        P1OUT &= ~LED2_OUT;
        P2OUT &= ~RELAY_ON;
//        SD16INCTL0 &= ~SD16GAIN_1;
//        SD16INCTL0 |= SD16GAIN_32;        
        multi_timer = 0;
        led_1_LoopCheck = lED1cHECKtIME;
        en_pir_2_Counter = 1;
        pir_2_Timer = lED2oNtIME;
        en_pir_1_Counter = 1;           //avoide PIR1 to interrupt PIR2
        pir_1_Timer = lEDoNtIME;    //   
        
      }
      break;  
      
  }  
} 

void WAVE1_MODE(void)
{
  if(pir_1_Hightest < pIR1tOPlEVEL)
  {
  if(pir_1_Hightest > pIR1lEVEL)
  {
    if(pir_1_Trig_Counter >= pIR1fREQ) //High_Long wave
    {
      pir_1_TC_Save = pir_1_Trig_Counter;
      Wave_1_Mode_No = 11;
      High_Long_1 = High_Long_1+pir_1_Trig_Counter;
      if(High_Long_1>lONGpIR1fREQ)
      {
        Wave_1_Mode_No = 7; //let Even(0)+Mode(7)=7 for doing even trig
        Even_1_No = 0;
        High_Long_1 = 0;
      }
    }
    else
      if(pir_1_Trig_Counter >= nOISEpIR1fREQ)    //High_Short wave and not noise
    {
      pir_1_TC_Save = pir_1_Trig_Counter;
      Wave_1_Mode_No =12;
    }
  }
  else
  {
    if(pir_1_Trig_Counter >= pIR1fREQ)   //Low_Long wave
    {
      pir_1_TC_Save = pir_1_Trig_Counter;
      Wave_1_Mode_No = 15;
      Low_Long_1 = Low_Long_1+pir_1_Trig_Counter;
      if(Low_Long_1>lONGpIR1fREQ)
      {
        Wave_1_Mode_No = 8; //let Even(0)+Mode(8)=8 for doing even trig
        Even_1_No = 0;
        Low_Long_1 = 0;
      }      
    }
    else
      if(pir_1_Trig_Counter >= nOISEpIR1fREQ)       //Low_Short wave,not noise
    {
        pir_1_TC_Save = pir_1_Trig_Counter;
        Wave_1_Mode_No =17;
    }
  }
  }
  if(!Even_1_No)
    Even_1_No = Wave_1_Mode_No;
  if(Even_1_No != Wave_1_Mode_No)
    Even_1_No = Even_1_No + Wave_1_Mode_No;
  switch(Even_1_No)
  {
    case 0:
      break;
      
    case 23:
      Even_1_No = 1;
      break;
    case 26:
      Even_1_No = 2;
      break;
    case 28:
      Even_1_No = 3;
      break;
    case 27:
      Even_1_No = 4;
      break;
    case 29:
      Even_1_No = 5;
      break;
    case 32:    
      Even_1_No = 6;
      break;  
      
  default:
    break;
    
  }
  
//  pir_1_Hightest = 0;
//  pir_1_Trig_Counter =0;

}


void WAVE2_MODE(void)
{
  if(pir_2_Hightest < pIR2tOPlEVEL)
  {
  if(pir_2_Hightest > pIR2lEVEL)
  {
    if(pir_2_Trig_Counter >= pIR2fREQ) //High_Long wave
    {
      pir_2_TC_Save = pir_2_Trig_Counter;
      Wave_2_Mode_No = 11;
      High_Long_2 = High_Long_2+pir_2_Trig_Counter;
      if(High_Long_2>lONGpIR2fREQ)
      {
        Wave_2_Mode_No = 7; //let Even(0)+Mode(7)=7 for doing even trig
        Even_2_No = 0;
        High_Long_2 = 0;
      }      
    }
    else
      if(pir_2_Trig_Counter >= nOISEpIR2fREQ)    //High_Short wave and not noise
    {
      pir_2_TC_Save = pir_2_Trig_Counter;
      Wave_2_Mode_No =12;
    }
  }
  else
  {
    if(pir_2_Trig_Counter >= pIR2fREQ)   //Low_Long wave
    {
      pir_2_TC_Save = pir_2_Trig_Counter;
      Wave_2_Mode_No = 15;
      Low_Long_2 = Low_Long_2+pir_2_Trig_Counter;
      if(Low_Long_2>lONGpIR2fREQ)
      {
        Wave_2_Mode_No = 8; //let Even(0)+Mode(8)=8 for doing even trig
        Even_2_No = 0;
        Low_Long_2 = 0;
      }      
    }
    else
      if(pir_2_Trig_Counter >= nOISEpIR2fREQ)       //Low_Short wave but not noise
    {
        pir_2_TC_Save = pir_2_Trig_Counter;
        Wave_2_Mode_No =17;
    }
  }
  }
  if(!Even_2_No)
    Even_2_No = Wave_2_Mode_No;
  if(Even_2_No != Wave_2_Mode_No)
    Even_2_No = Even_2_No + Wave_2_Mode_No;
  switch(Even_2_No)
  {
    case 0:
      break;
      
    case 23:
      Even_2_No = 1;
      break;
    case 26:
      Even_2_No = 2;
      break;
    case 28:
      Even_2_No = 3;
      break;
    case 27:
      Even_2_No = 4;
      break;
    case 29:
      Even_2_No = 5;
      break;
    case 32:    
      Even_2_No = 6;
      break;    
      
  default:
    break;
    
  }
  
//  pir_2_Hightest = 0;
//  pir_2_Trig_Counter =0;

}

/*
void AD_RECORD(unsigned int ad_input)
{
        if (AD_point < AD_size)
      {
        AD_area[AD_point] = ad_input;
        AD_area[AD_size]=AD_area[AD_size]+AD_area[AD_point];
        AD_point++;
        if(AD_point >=AD_size)
          AD_area[AD_size]=AD_area[AD_size]/AD_size;
      }
      else
          AD_point =0;
}
*/