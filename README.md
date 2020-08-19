# MSP430F2013
![image](image/pinout.png)
# IAR embedded workbench
 1. download: https://www.iar.com/iar-embedded-workbench/#!?architecture=MSP430
 2. tutorial: https://youtu.be/9QzkK1CaEmM
 3. visual c++ redistributable: https://support.microsoft.com/zh-tw/help/2977003/the-latest-supported-visual-c-downloads
 4. User Guides: IAR Embedded Workbench for TI MSP430 https://www.iar.com/support/user-guides/user-guidesiar-embedded-workbench-for-ti-msp430/
 
# JTAG USB debug interface
 1. MSP-FET
 2. MSP-FET430UIF
 
 ![image](image/MSP430_JTAG_2_wire_pinout.jpg)
# MSP430 USB Stick Development Tool
  EZ430-F2013
# Flash tools 
  UniFlash
  FlashPro-430 and GangPro-430
  
## 低雜音長距離 PIR sensor 電路設計與分析
![image](image/pir_low_pass.png.jpg)
 1. PIR 之信號應取交流之部分(C2 與 C6)，故必須提供1/2Vcc電壓(R2 and R3)。
 2. OPA開環路增益約為10^5，反饋電路降低整體電路的增益會使系統比較穩定。OPA的開環增益隨頻率快速的下降，限制頻寬(C4,C8)。
 3. OPA輸出加上RC Filter可以降低OPA輸出之雜音。
 4. Gain值可能會受到截止頻率之影響。
 5. Low pass 轉折頻率點設計在10HZ，反推回1/2PI* RC，約1.5K歐姆(R1,R6,R11)
 6. High pass 轉折頻率點設計在0.7HZ，約1.5K歐姆(R4,R9)
 7. 直流準位R2=R3=R7=R8=2R4=13.6K歐姆
 8. Gain 90dB=Gain/2=45dB=177.8*(Vout/Vin)，R5 =(Gain-1)* R4 ， R5= 1.2M歐姆
 9. C4,C8 = 1/(2PI * R5 * 10HZ) = 13nF，choose 15nF 
  
