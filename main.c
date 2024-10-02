/* 
 * File:   main.c
 * Author: tmiller
 *
 * Created on January 5, 2016, 10:19 PM
 */

/* The device at the Antenna */



// PIC16F1455 Configuration Bit Settings

// 'C' source line config statements



// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
//  NOW IN system.c
// CONFIG1
//#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
//#pragma config WDTE = OFF        // Watchdog Timer Enable (WDT enabled)
//#pragma config PWRTE = OFF       // Power-up Timer Enable (PWRT enabled)
//#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
//#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
//#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
//#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
//#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
//#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)
//
//// CONFIG2
//#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
//#pragma config CPUDIV = NOCLKDIV // CPU System Clock Selection Bit (CPU system clock divided by 6)
//#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
//#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
//#pragma config PLLEN = DISABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
//#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
//#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
//#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
//#pragma config LVP = OFF        // Low-Voltage Programming Enable (Low-voltage programming enabled)

/*
 *     // The Motor power pins are wired Red and Black on the module.
    // The Green wire goes into the resistor. V+
    // THe blue wire connects to Ground
    // The white wire is signal out.
 */


#include <xc.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>



#include "user.h"
#include "system.h"


//#define _XTAL_FREQ 2000000UL
#define ClockWise_Delay 10 //2               // 3 milliseconds seems to be the minimum
#define CounterCLockWise_Delay 5 // 2        // in debug.

#ifdef _USE_IOC_
volatile unsigned int intOnChange;
volatile unsigned int intLimit;
volatile extern unsigned int T0Count;
volatile extern unsigned int T1Count;
#endif

volatile unsigned int pulseCount;
volatile unsigned int Timer0Int;

    char lastStep;

   // Pin 1 Power/Vdd
#ifdef _USE_IOC_
#define IOC_PORT LATAbits.LATA5    // Pin 2 RA5 4A/B2 Coil on Stepper motor
#endif
    // Pin 3 RA4 a connector pin output - no connection
    // Pin 4 RA3 (Reset)
#define MotorEnable LATCbits.LATC5    // Pin 5 RC5 3A/B1 Coil on Stepper motor SN754410 Ports 1 & 2 Enable PWM1
#define Motor1A LATCbits.LATC4        // Pin 6 RC4 Motor Control 1A
#define Tuning LATCbits.LATC3   // Pin 7 RC3 A/D tuning dial AN7
#define Motor2A LATCbits.LATC2     // Pin 8 RC2 Motor Control 2A
#define Motor3A LATCbits.LATC5
#define Motor4A LATAbits.LATA5
    // Pin 9 ISCPCLK/RC1
    // Pin 10 ISCPDAT/RC0
    // Pin 11 RA2 Vusb
    // Pin 12 D-
    // Pin 13 D+
    // Pin 14 Ground/Vss

#define FULL_RANGE_ADCMAX 1008
#define FULL_RANGE_ADCMIN 4
#define Clockwise 1
#define CounterClockwise 0
// #define _USE_IOC_ to implement that code - Not needed for stepper motor
//    unsigned int pulseValues[40];


int sendPulse(long int i) {
    int j;
    LATCbits.LATC0 = 1;      // Turn on Transmit Data & Receive Off
    for(j = 0; j < i; j++) {
        LATAbits.LATA4 = 1;
        NOP();              // Clock is at 48MHz, FOSC = 48MHz/4 = 12MHz = system clock -> 83.3333 ns per instruction cycle
        NOP();              // NOP is 1 instruction cycle
        NOP();              // 16 NOP() = 1.333 microseconds
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        LATAbits.LATA4 = 0;
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();          // 2.6 microseconds in nop() + port sets and loop
    }
    LATCbits.LATC0 = 0;     // Turn off transmit Data & Receive On
    return (EXIT_SUCCESS);

}

    unsigned int ADCValue ;
    unsigned int oldADCValue;
    
    extern unsigned int SPIRead, ReadSDI;

int main(int argc, char** argv) {


    static unsigned char Answer1, Answer2;

    unsigned long int i;
    int avgIntPerRev;           // Calibrated value of interrupts in Capacitance range.
    int totalintLimit;
    double tuning;
    double oldtuning;
    int numberOfSteps;
    int oldNumberOfSteps;
    unsigned char direction;
    unsigned int oldPulseCount;
    unsigned int stepCount;
    unsigned int savPulseCount;
    unsigned int firstPulseCount;
    char buf[10], sbuf[5];


    PMD0bits.NVMMD = 1;  // Disable NVM Module
    PMD1bits.NCOMD = 1; // Disable NCO
    PMD1bits.TMR6MD = 1;  // Disable TMR6
    PMD1bits.TMR5MD = 1;  // Disable TMR5
    PMD1bits.TMR4MD = 1;  // Disable TMR4
    PMD1bits.TMR3MD = 1;   // Disable TMR3
    PMD2bits.DACMD = 1;     // Disable DAC
    PMD3bits.CWG1MD = 1;    // Disable CWG
    PMD3bits.CWG2MD = 1;    // Diaable CWG
    PMD3bits.PWM6MD = 1;    // Disable PWM
    PMD3bits.CCP4MD = 1;    // Disable Capture and Compare
    PMD3bits.CCP3MD = 1;
    PMD3bits.CCP2MD = 1;
    PMD3bits.CCP1MD = 1;
    PMD4bits.UART1MD = 1;   // Disable UART1
    PMD4bits.MSSP2MD = 1;   // Disable MSSP
    PMD5bits.CLC1MD = 1;    // Disable Configure Logic Cell
    PMD5bits.CLC2MD = 1;    // Disable CLC2
    PMD5bits.CLC3MD = 1;    // Disable CLC3
    PMD5bits.CLC4MD = 1;    // Disable CLC4
    PMD5bits.DSMMD = 1;     // Disable DSM
    

    
    
//    SYSTEM_Tasks();
//    TRISCbits.TRISC3 = 0;
//    APFCONbits.CLKRSEL = 1; /Motor1A/ Pin 7 clock out.
//    CLKRCONbits.CLKRDIV = 1;    // Clock divider
//    CLKRCONbits.CLKRSLR = 1;    // Slew limit
//    CLKRCONbits.CLKRCD = 2;     // Duty Cycle
//    CLKRCONbits.CLKROE = 1;     // Enable pin ouput
//    CLKRCONbits.CLKREN = 1;     // Enable clock
    
    // Port Control
    

    PORTA = 0;
//    PORTA &= 0b000110000;    // No output on pins 2 RA5, 3 RA4; 4A, Connector on Pot.
//    PORTC &= 0b000110100;    // No output on pins RC55  3A pin 5,RC4 1A pin 6, RC2 pin 8 2A
    PORTC = 0;        // RC1/AN5/Pin 9 input for pot motor
    PORTCbits.RC0 = TRISCbits.TRISC0 = 0;      // Enable Receiver on RC3
    LATAbits.LATA4 = LATAbits.LATA5 = 0;    // No latched outputs
//    LATBbits.LATB4 = LATBbits.LATB5 = LATBbits.LATB6 = LATCbits.LATC6 = 0;   // SPI Ports
    LATCbits.LATC2 = LATCbits.LATC3 = LATCbits.LATC4 = LATCbits.LATC5 = 0;
    TRISAbits.TRISA5 = 0;    // 1455 Motor Control Output RA5, 18345 LED
    TRISAbits.TRISA4 = 0;    // SN75176 Pin DI/ Data into RS-485 receiver
#ifdef __PIC16F1455__
    // Motor control lines
    TRISCbits.TRISC2 = TRISCbits.TRISC4 = TRISCbits.TRISC5 = TRISAbits.TRISA5 = 0;
#endif

    TRISCbits.TRISC1 = 1;       // Motor POT, PIN 9, AN5
    INTCONbits.GIE = 1;     // Enable all interrupts that are enabled.
    INTCONbits.PEIE = 1;
    CLKRCONbits.CLKREN = 0;     // Reference clock disabled.

#ifdef _USE_IOC_

    PIE1 = 1;    // Enable timer 2 interrupt
    INTCONbits.GIE = 1;     // Enable all interrupts that are enabled.
//    INTCONbits.PEIE = 1;
//    LATC = 0;           // Output no signals at startup
    // Cannot turn this interrupt off in debugging
    INTCONbits.INTE = 0;



//    setupTimer0();
    setupTimer1();


    // The Motor Enable pin (Q1 RC3 Pin 7) is driven by the PWM generator Ouput
    initPWM();       // Period

//    startPWM(0xF8);     // Duty Cycle
#endif

    // Position sensor - A Potentiometer geared to the position
    // Analog to digital conversion port RC3
    ANSELCbits.ANSC3 = 1;       // RC3 / Pin 7 is C1IN3- for the Read SN175 trans/recv.
    ANSELCbits.ANSC1 = 1;       // Analog pin 9 as AN5 for motor pot.
    // AN7 - the Potentiometer input. All others Digital
    // Configure Fixed Voltage reference for use with AtoD conversion.
    FVRCONbits.ADFVR = 3; // 4.096 Volts.
    FVRCONbits.CDAFVR = 3; // Comparator FVR 4.096V
    FVRCONbits.FVREN = 1;  // Enable the FVR
    while (!FVRCONbits.FVRRDY) {};      // Wait until the FVR is ready.

    
#ifdef __PIC16F1455__  
    TRISCbits.TRISC3 = 1;       // C1IN3- is input to Pin 7 - AN7
    TRISCbits.TRISC1 = 1;       // AN5 input pin 7 motor pot
#endif
    // AN7 will vary from 0 to 4 Volts.
    // R2 is a 10k potentiometer, R2 is the tuner knob for the capacitor.
    // Prepare the AtoD converter to sample knob position voltage
//    setupADC();
 initIOC();
#ifdef _USE_IOC_
    // Motor Control
//    TRISCbits.TRISC2 /2= 0;       //  RC2 (pin 8) 1A
//    TRISCbits.TRISC4 = 0;       // RC4  (pin 6) 2A

//  Testin1005.0g Motor1A & Motor2A - won't work without enable pin high
//    LATCbits.LATC2 = 1;
//    LATCbits.LATC4 = 1;


    // Pin 3 / RA4  is the IOC Digital Motor Rotation Sensor.
//    TRISAbits.TRISA4 = 1;       // Pin 3 RA4 Digital Motor Rotation Sensor IOC
    // Prepare the IOC pin to notify when the motor turns - IOC increments
    intLimit = 0xFFFF;
   
    initIOC();

////  Calculated: 32 interrupts per rotation, 7.5 rotations, x010E = 270
//    avgIntPerRev = calibrateCapacitor(5,0);        // UP - decrease capacitance.
//    avgIntPerRev = calibrateCapacitor(5,1);     // DOWN - increase capacitance.
    avgIntPerRev = 270;

   // Prepare the PWM to move the motor at the desired speed
   rMotor(0,240);// zero capacitor C4 UP
   // Find where the potentiometer is set:
//   oldADCValue = getADCValue(); // returns the integer from 0 - 1024 (10 bit conversion)
//   if(oldADCValue > 1008)
//       oldADCValue = 1024;
//   if(oldADCValue < 8)
//       oldADCValue = 0;
//   // This value is the follwing part of the the total range.
//   oldtuning = (double) oldADCValue/(1024);
//   // That total part is this many interrupts from the rMotor() reset
//   totalintLimit = (int) ((double) oldtuning * avgIntPerRev);
//   // prepare the internal counter
//   intOnChange = 0;
   // Set to move as little as possible per motor power up
//   intLimit = intOnChange + 1;
//   while(totalintLimit > 0) {
//       while(intOnChange < intLimit) {
//            startTMR1();
//            LATCbits.LATC2 = 1;     // Raise Capacitance DOWN if totalintLimit > 0
//            while (LATCbits.LATC2 ) {};
//            stopTMR1();
//       }
//       intLimit = intOnChange;
//       totalintLimit -= intLimit;
//       if(totalintLimit > 0) {
//            intLimit =  1;
//            intOnChange = 0;
//       } else {
//           break;
//       }
//   }


   // A positioning test code section
//   totalintLimit = 135;
//            intOnChange = 0;
//            intLimit = intOnChange + 1;
//            while(totalintLimit > 0) {
//                while(intOnChange < intLimit) {
//                    startTMR1();
//                    LATCbits.LATC2 = 1;     // Raise Capacitance DOWN
//                    while (LATCbits.LATC2 ) {};
//                    stopTMR1();
//               __delay_ms(40);
                }
//                totalintLimit -= intOnChange;
//                intLimit = intOnChange + 1;
//                intOnChange = 0;
//            }

   oldADCValue = 1024;
   oldtuning = 1.0;

    // In Perpetual Tuning
    while(1)
    {

        ADCValue = getADCValue(); // returns the integer from 0 - 1024 (10 bit conversion)
        if(ADCValue > 1008)
            ADCValue = 1024;
        if(ADCValue < 8)
            ADCValue = 0;

        if(abs(ADCValue - oldADCValue) > 0x10) {
            tuning = (dohttps://www.google.com/search?q=differential+voltage+driver+receiver&ie=utf-8&oe=utf-8uble) ADCValue/1024.0 - oldtuning;
            totalintLimit = (int) ( fabs(tuning) * (double) avgIntPerRev);
            oldtuning = (double) ADCValue/1024.0;
            intOnChange = 0;
            intLimit = intOnChange + 2;
            if(ADCValue < oldADCValue) {
                while(totalintLimit > 0) {

                    LATCbits.LATC2 = 1;     // Raise Capacitance DOWN
                    startTMR1();
      SYPIC16F1STEM_Tasks();              while (LATCbits.LATC2 ) {};
                    stopTMR1();
 
                    intLimit = intOnChange;
                    totalintLimit -= intLimit;
                    if(totalintLimit < 0)
                        goto done;
                    intLimit =  2;
                    intOnChange = 0;
                }
            } else  {
 
                while (totalintLimit > 0 ) {
                    startTMR1();
                    LATCbits.LATC4 = 1;     // Lower Capacitance  UP
                    while(LATCbits.LATC4) {};
    //                for (i = 0; i< 47; i++)
    //                    NOP();
                    LATCbits.LATC4 = 0;     // Turn the motor off
                    stopTMR1();
//                    if(T1Count)
//            2            goto done;

//                    startTMR1();
//                    LATCbits.LATC4 = 1;     // Lower Capacitance UP
//                    while (LATCbits.LATC4) {};
//                    stopTMR1();
//
//                    intLimit = intOnChange;
                    if(!T1Count)
                        totalintLimit -= intOnChange;
//                    if(totalintLimit < 0)
//                        goto done;
              2      intLimit =  1;
                    intOnChange = 0;
                }
            }
done:1
            oldADCValue = ADCValue;
        }
    }




    // With this value I can set the capacitor position
    // There are 7 * 64 + 37 = 485 possible positions, each 64 being one full turn.

    // Capacitor has been aligned with Tuning knob setting.





    
    // Now I want the DAC to be set to the same voltage and wait for the 
    // pot to be turned so that when it is an interrupt will tell me to
    // move the capacitor, but there are only 32 possible voltage outputs and
    // 512 possible inputs, so I will not use it.

//    while (1) {
//        int newADCValue = getADCValue();
//        double NewKnobVoltage = (double) ADCValue/512.0 * 4.003;
//
//        int differenceKnobVoltage = newADCValue - ADCValue;
//
//
//        if(differenceKnobVoltage > 2 | differenceKnobVoltage < 2 ) {  // The knob has moved
//            if(differenceKnobVoltage > 2) {
//                // I will assume the direction temporarily
//                while (differenceKnobVoltage > 0 ) {
//                    intOnChange = 1;
//                    LATCbits.LATC4 = 1;
//                    LATCbits.LATC5 = 1;         // Turn the motor to the position
//                    while(intOnChange) {};
//                    differenceKnobVoltage--;
//                }
//            } else {
//                while (differenceKnobVoltage < 0 ) {
//                    intOnChange = 1;
//                    LATCbits.LATC2 = 1;
//                    LATCbits.LATC3 = 1;         // Turn the motor to the position
//                    while (intOnChange) {};
//                    differenceKnobVoltage++;
//                }
//            }
//
//        }
//    }
#endif

    //
    // Get Capacitor to a known position
    //
    // Full swing Clockwise
     __delay_ms(4000);      // 4 seconds
#if defined ( _USE_USB_)
    __delay_ms(10000);  /** 4 sec**/
#endif
   
    // Stepper Motor Setup
    
//    PMD4bits.MSSP1MD = 1;   // Disable MSSP module to set ports   
    RB6PPS = 0b11000;   // RC6 port at pin 11 is assigned as SCK1
    SSP1CLKPPS = 0b01110;   // SCLK1 is at RB6
    RB5PPS = 0b11001;    // RB5 port at pin 12 is assigned as SDO1
    SSP1DATPPS = 0b1100;   // Peripheral SSP1SDI is at RB4
    SSP1SSPPS = 0b10110;    // SS1 is at RC6 pin 8
   
    LATBbits.LATB4 = LATBbits.LATB5 = LATBbits.LATB6 = LATCbits.LATC6 = 0;
    ANSELBbits.ANSB4 = ANSELBbits.ANSB5 = ANSELBbits.ANSB6 = ANSELCbits.ANSC6 = 0;  // Digital pins
//    WPUBbits.WPUB4 = 
                        WPUBbits.WPUB5 = WPUBbits.WPUB6 = WPUCbits.WPUC6 = 1;     // Weak Pull ups
    ODCONBbits.ODCB4 = 1;   // Open drain (sink current only)
    SLRCONBbits.SLRB4 = 1;  // Slew Rate enabled (slows it down)
    INLVLBbits.INLVLB4 = 1; // ST input (lower voltage for high)
    TRISBbits.TRISB4 = 1;   // SDI (TRIS=1) RB4/13
    TRISBbits.TRISB5 = 0;   // SDO (TRIS=0) RB5/12
    TRISBbits.TRISB6 = 0;   // SCLK (TRIS=0) RB6/11
    TRISCbits.TRISC6 = 0;   // ~SS (TRIS=1) RC6/8
 //   SP1BRG = 0x13;      // 400kHz
    // SPI Interrupts
    PIR1bits.SSP1IF = 0;    // Clear Read/Write Status interrupt Flag
    PIR1bits.BCL1IF = 0;    // Clear Bit Collision on SPI1 interrupt Flag    
    PIE1bits.SSP1IE = 1;    // Read or write complete on SPI1
    PIE1bits.BCL1IE = 1;    // Bit Collision on SPI1   
 
 //   PMD4bits.MSSP1MD = 0;   // Enable MSSP module to set ports       
    
    SSP1ADD = 0x13;     // 400kHz    
    SSP1CON1bits.SSPM = 0b1010;      // SPI Master Mode, clock = FOSC/4 = 32MHz/64 = 500kHz
    SSP1CON1bits.CKP = 1;  // Clock idle (Clock Polarity) state is high so MP6620 Data shifts on the rising edge of SCLK
    SSP1STATbits.CKE = 0;  // Transmit occurs on transition from active to idle clock state.
    SSP1STATbits.SMP = 1;   // Input data sampled at end of data output time.
//    SSP1CON3bits.BOEN = 1;
//    Data input sample phase (middle or end of data output time)
//    Clock Edge (output data on rising/falling edge of SCK)
 //   LATCbits.LATC6 = 1;  // Slave not selected


    // Wake up MP6602 from sleep drive pin 6 low
    TRISBbits.TRISB7 = 0;
    WPUBbits.WPUB7 = 0;     // MP6602 has a pull down resistor
    LATBbits.LATB7 = 0;
    // Reset MP6602 - drive pin 7 low
    WPUCbits.WPUC7 = 1;     // Enable weak pull up
    LATCbits.LATC7 = 0;
    // Enable MP6602 - drive pin 9 
    TRISCbits.TRISC1 = 0;
    WPUCbits.WPUC1 = 0;
    LATCbits.LATC1 = 1;
    

          
    
    
     SSP1CON1bits.SSPEN = 1;  // ENable SPI
     
//    static unsigned char x1;
//    while (1) {
//        x1 = PORTB;
//    }     
    __delay_ms(1);   
    // MP6602 Register definitions
#define CtrlReg  0
#define Ctrl2Reg 2
#define IsetReg 4
#define StallReg 6
#define BemfReg 8
#define TspReg 10
#define OcpReg 12
#define FaultReg 14
    

    SPIRead = 0;  // Do not read on interrupt
    LATCbits.LATC6 = 1;  // Slave not selected      
    while(1) {
//        for(int pp = 2; pp < 0x0f; pp = pp +2) {
    LATCbits.LATC6 = 0;     // Slave Select   
    NOP();
    NOP();
    NOP();
    // Write to reg 0: 0xF22
    SSP1BUF = 0x10; ;
    while(!SSP1STATbits.BF);    // while there is no answer, wait
    Answer1 = SSP1BUF;          // The register contains SDI
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);   
    Answer2 = SSP1BUF;
    LATCbits.LATC6 = 1;  // Slave not selected 
    __delay_ms(1);
    LATCbits.LATC6 = 0;     // Slave Select    
    // Write to reg 0: 0xF22
    SSP1BUF = 0x12; ;
    while(!SSP1STATbits.BF);
    Answer1 = SSP1BUF;
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);   
    Answer2 = SSP1BUF;
    LATCbits.LATC6 = 1;  // Slave not selected 
    __delay_ms(1);  
    LATCbits.LATC6 = 0;     // Slave Select    
    // Write to reg 0: 0xF22
    SSP1BUF = 0x14; ;
     while(!SSP1STATbits.BF);
    Answer1 = SSP1BUF;     
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);   
    Answer2 = SSP1BUF;
    LATCbits.LATC6 = 1;  // Slave not selected 
    __delay_ms(1); 
    LATCbits.LATC6 = 0;     // Slave Select    
    // Write to reg 0: 0xF22
    SSP1BUF = 0x16; ;
     while(!SSP1STATbits.BF);
    Answer1 = SSP1BUF;    
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);   
    Answer2 = SSP1BUF;
    LATCbits.LATC6 = 1;  // Slave not selected 
    __delay_ms(1);
     LATCbits.LATC6 = 0;     // Slave Select    
    // Write to reg 0: 0xF22
    SSP1BUF = 0x18; ;
     while(!SSP1STATbits.BF);
    Answer1 = SSP1BUF;     
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);   
    Answer2 = SSP1BUF;
    LATCbits.LATC6 = 1;  // Slave not selected 
    __delay_ms(1);
     LATCbits.LATC6 = 0;     // Slave Select    
    // Write to reg 0: 0xF22
    SSP1BUF = 0x1A; ;
     while(!SSP1STATbits.BF);
     Answer1 = SSP1BUF;    
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);   
    Answer2 = SSP1BUF;
    LATCbits.LATC6 = 1;  // Slave not selected 
    __delay_ms(1);
     LATCbits.LATC6 = 0;     // Slave Select    
    // Write to reg 0: 0xF22
    SSP1BUF = 0x1C; ;
    while(!SSP1STATbits.BF);
    Answer1 = SSP1BUF;     
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);   
    Answer2 = SSP1BUF;
    LATCbits.LATC6 = 1;  // Slave not selected 
    __delay_ms(1); 
     LATCbits.LATC6 = 0;     // Slave Select    
    // Write to reg 0: 0xF22
    SSP1BUF = 0x1E; ;
    while(!SSP1STATbits.BF);
    Answer1 = SSP1BUF;    
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);   
    Answer2 = SSP1BUF;
    LATCbits.LATC6 = 1;  // Slave not selected 
    __delay_ms(1); 
    
//    // Clock is C6
//    IOCCNbits.IOCCN6 = 1;       // Enable clock negative edge on C6
//    ReadSDI = 0;

    
    
    while(1) {
    
    LATCbits.LATC6 = 0;     // Slave Select   
//    SPIRead = 1;
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);    
    Answer1 = SSP1BUF;
    SSP1BUF = 0x00;
    while(!SSP1STATbits.BF);
    Answer2 = SSP1BUF;
//    SPIRead = 0;
    LATCbits.LATC6 = 1;     // Slave not Selected.
    };
    
    };

    
    LATCbits.LATC6 = 0;     // Slave Select    
    // Write to reg 0: 0xF22
    SPIRead = 1;
    SSP1BUF = 0x00; ;
    __delay_ms(1); 
//    Answer1 = SSP1BUF;     
    SSP1BUF = 0x00;
    __delay_ms(1)  ;
//    Answer2 = SSP1BUF;
    
////    LATCbits.LATC6 = 0;     // Slave Select    
//    SSP1BUF = Ctrl2Reg | Read | 0;
//    SSP1BUF = 0;
//    while(!SSP1STATbits.BF);
////    LATCbits.LATC6 = 1;  // Slave not selected  
//    Answer = SSP1BUF;
//    Answer = SSP1BUF;
//    __delay_ms(100);

////    LATCbits.LATC6 = 0;     // Slave Select
//    SSP1BUF = IsetReg | Read | 0;
//    while(!SSP1STATbits.BF);
////    LATCbits.LATC6 = 1;  // Slave not selected      
//    Answer = SSP1BUF;
////    LATCbits.LATC6 = 0;     // Slave Select
//    SSP1BUF = StallReg | Read | 0;
//    while(!SSP1STATbits.BF);
////    LATCbits.LATC6 = 1;  // Slave not selected      
//    Answer = SSP1BUF;
////    LATCbits.LATC6 = 0;     // Slave Select    
//    SSP1BUF = BemfReg | Read | 0;
//    while(!SSP1STATbits.BF);
////    LATCbits.LATC6 = 1;  // Slave not selected      
//    Answer = SSP1BUF;
////    LATCbits.LATC6 = 0;     // Slave Select
//    SSP1BUF = TspReg | Read | 0;
//    while(!SSP1STATbits.BF);
// //   LATCbits.LATC6 = 1;  // Slave not selected 
//    Answer = SSP1BUF;
////    LATCbits.LATC6 = 0;     // Slave Select
//     SSP1BUF = OcpReg | Read | 0;
//    while(!SSP1STATbits.BF);
// //   LATCbits.LATC6 = 1;  // Slave not selected 
//    Answer = SSP1BUF;
// //   LATCbits.LATC6 = 0;     // Slave Select
//    SSP1BUF = FaultReg | Read | 0;
//    while(!SSP1STATbits.BF);
// //   LATCbits.LATC6 = 1;  // Slave not selected     
//    Answer = SSP1BUF;
//    };

#define USE_ADC  1

#if USE_ADC
    setupADC();             // Read motor pot.
#endif 
    
#ifdef __PIC16F1488__
    TRISCbits.TRISC0 = 0;   // Voltage reference is internal only

    resetMotor(Clockwise,0);        // 0 capacitance
    unsigned int ClockWiseResistanceLimit = ADCValue;
    resetMotor(CounterClockwise,0);
    unsigned int CounterClockWiseResitanceLimit = ADCValue;
#endif
    setupTimer0();      // Used to delay for confirmation pulse after valid pulse
//    startTMR0();

    setupTimer1();      // Used to measure pulse widths, gated with comparator

    setupComparator();

    resetTimer1();

    oldNumberOfSteps = 1;
     
    // Continue to respond to tuning
    i = 0;
//    total = 67.0;
    oldPulseCount  = 0;

//    while(1) {
    __delay_ms(100);
    sendPulse(1);       // Initialization complete- get the remote ready
//    };

//#define PULSE_DEBUG
#ifdef PULSE_DEBUG

//    int l = 1;             // This is a test routine to send pulses
//    while(1) {             // They are measured at the other end
//        sendPulse(l++);    // to verify the transmission from this end.
//        __delay_ms(200);  //
//        if(l > 400)        //
//            l = 1;         //
//    }

    while (1) {
        if(pulseCount) {
            sendPulse(pulseCount);
            resetTimer1();
        }
    }

#endif
    CLRWDT();               // Clear the Watchdog Reset Timer
    WDTCONbits.WDTPS = 0xB; // Set the Watchdog Reset Timer Period to 2 seconds
    WDTCONbits.SWDTEN = 1;  // Turn on the Watchdog Reset Timer
    while (1) {
     CLRWDT();               // Clear the Watchdog Reset Timer       
//        SLEEP();            // Only a signal from the transmitter should
                            // wake this up. Timer0 cannot operate in sleep mode.
        //
        // Read the tuning knob position
        //
//        rMotor(CounterClockwise, 376 ) ;

        // The AtoD position of the motor POT.
//        ADCValue = getADCValue(); // returns the integer from 0 - 1023 (10 bit conversion)
#if defined ( _USE_USB_)
//            itoa(buf,ADCValue,10);
//            strcat(buf," ");
//            putUSBUSART(buf,strlen(buf));
//            CDCTxService();
#endif
        if (pulseCount) {
//            firstPulseCount = (float) (pulseCount - 21.5)/47.88;
            firstPulseCount = (unsigned int) ((float) (pulseCount * 0.5)); // 250ns/500ns
            resetTimer1();      // also sets pulseCount = 0
            __delay_ms(40);
            while(!pulseCount) {};
//            startTMR0();
//            while(Timer0Int) {};    // A new pulse count comes in
//            stopTMR0();
            pulseCount = (unsigned int) ((float) (pulseCount * 0.5)); //  - 21.5)/47.88;
#if defined ( _USE_USB_)
            strcpy(&buf[0],"P ");
            new_itoa (sbuf,firstPulseCount,10);
            strcat(buf,sbuf);
            strcat(buf," ");
            new_itoa (sbuf,pulseCount,10);
            strcat(buf,sbuf);
            strcat(buf," ");
            putUSBUSART(buf,strlen(buf));
            CDCTxService();
            APP_DeviceCDCBasicDemoTasks();
#endif
            if ((pulseCount >= firstPulseCount - 2)  && (pulseCount <= firstPulseCount + 2)) {

//            Calculate the step value from the timed signal width
//                pulseCount =  (float) ((pulseCount - 7.5)/9.5)/0.625834343;
//                pulseCount = (float) (pulseCount - 7.5)/5.9454262585;

//                pulseCount /= 2;
//                while(1) {
//                    __delay_ms(5);
                sendPulse(pulseCount);      // send back a valid pulse
//                }



                if(pulseCount > oldPulseCount+1) {
                    stepCount = pulseCount - oldPulseCount;
                    rMotor( CounterClockwise, stepCount ) ;
                    oldPulseCount = pulseCount;
//                    ADCValue = getADCValue(); // returns the integer from 0 - 1023 (10 bit conversion)
                } else if (pulseCount < oldPulseCount-1) {
                    stepCount = oldPulseCount - pulseCount;
                    rMotor( Clockwise, stepCount ) ;
                    oldPulseCount = pulseCount;
//                    ADCValue = getADCValue(); // returns the integer from 0 - 1023 (10 bit conversion)
                }


            } else if (pulseCount == 0 || firstPulseCount == 0 ) {
                 sendPulse(pulseCount);      // send back a valid pulse
                 if(pulseCount < oldPulseCount ) {      // if 0 < 1
                     stepCount = oldPulseCount - pulseCount;
                     rMotor(Clockwise, stepCount);  // move from 1 to 0
                     oldPulseCount = pulseCount;
                 } else if (pulseCount > oldPulseCount ) { // if 1 > 0
                     stepCount = pulseCount - oldPulseCount;
                     rMotor( CounterClockwise, stepCount);
                     oldPulseCount = pulseCount;       // move from 0 to 1
                 }
                 
            } else { // the two pulses did not match closely enough
                sendPulse(400);     // Could not match any sent pulse
            }

            resetTimer1();

 
        }
 #if defined (_USE_USB_)
         #if defined(USB_POLLING)
            // Interrupt or polling method.  If using polling, must call
            // this function periodically.  This function will take care
            // of processing and responding to SETUP transactions
            // (such as during the enumeration process when you first
            // plug in).  USB hosts require that USB devices should accept
            // and process SETUP packets in a timely fashion.  Therefore,
            // when using polling, this function should be called
            // regularly (such as once every 1.8ms or faster** [see
            // inline code comments in usb_device.c for explanation when
            // "or faster" applies])  In most cases, the USBDeviceTasks()
            // function does not take very long to execute (ex: <100
            // instruction cycles) before it returns.
            USBDeviceTasks();
        #endif


        /* If the USB device isn't configured yet, we can't really do anything
         * else since we don't have a host to talk to.  So jump back to the
         * top of the while loop. */
//        if( USBGetDeviceState() < CONFIGURED_STATE )
//        {
//            /* Jump back to the top of the while loop. */
//            continue;
//        }
//
//        /* If we are currently suspended, then we need to see if we need to
//         * issue a remote wakeup.  In either case, we shouldn't process any
//         * keyboard commands since we aren't currently communicating to the host
//         * thus just continue back to the start of the while loop. */
//        if( USBIsDeviceSuspended()== true )
//        {
//            /* Jump back to the top of the while loop. */
//            continue;
//        }
//
//        //Application specific tasks
        APP_DeviceCDCBasicDemoTasks();
#endif

    }

// theAVE = 0.0;


//    for (i =0; i < 40; i++)
//        total += (float) pulseValues[i];
//    theAVE = total/40.0;
//    i = 0;

//        // Scale the voltage to the number of motor movements
//        tuning = (double) ADCValue * 0.3515625;
//
//        // Save the number of motor movements
//        numberOfSteps = (int) round(tuning);
//
//        // Compare direction of new movement to current position to establish direction
//        // or do not move.
//        if(numberOfSteps != oldNumberOfSteps) {
//
//            if(numberOfSteps > oldNumberOfSteps)
//                direction = CounterClockwise;
//            else
//                direction = Clockwise;
//
//            // Move the motor to new position
//            rMotor(direction,(unsigned char) abs(numberOfSteps));
//
//            // Save the new position
//            oldNumberOfSteps = numberOfSteps;
//        }
//
//    }


    return (EXIT_SUCCESS);
}

#ifdef _USE_IOC_
int calibrateCapacitor(int trials, unsigned char direction)
{
    int i = 0;
    int avgIntPerRev = 0;
    static int lowest, highest;
    lowest = 0xEFF;
    highest = 0;

    // Duty Cycle = 1, 100% (240)
    while(i < trials) {
        rMotor(direction,240);      //  Maximum Capacitance  Turn On 1A
        intOnChange = 0;
        rMotor((~direction)&1,240);
        avgIntPerRev += intOnChange;
        if(intOnChange < lowest)
            lowest = intOnChange;
        if(intOnChange > highest)
            highest = intOnChange;
        i++;
    }
    avgIntPerRev /= trials;
    return avgIntPerRev;
}
#endif


#ifdef _USE_IOC_
int increaseCapacitanceMotor(unsigned short value)
{
    unsigned char dutyCycle;
    unsigned char oldIntOnChange, savIntOnChange;
    dutyCycle = 255;
    intOnChange = 0;
 //   unsigned char count;

//    count = 255;

//    TRISCbits.TRISC5 = 1;           // Not putting out a motor drive signal - block port output
//    LATCbits.LATC5 = 0;             // Motor will drive when I/O of port is set.
//    while(intOnChange < value) {    // While I have not counted the correct number of interrupts
//   intOnChange = intOnChange - savIntOnChange;   // Subtract any movement between port enables.
//        oldIntOnChange = intOnChange;
            startPWM(dutyCycle);
//            PWM1DCH = dutyCycle >> 2;       // Set the PWM while the
//            PWM1DCL = dutyCycle | 0x3;      // port is not in use
//            TRISCbits8.TRISC5 = 1;   // turn the port on for a moment (? how long ?)
//            TRISCbits.TRISC5 = 0;   // turn the port back off
            LATCbits.LATC2 = 1;
            while(intOnChange < value)        // There are 32 slots in the wheel.
                    NOP();
            LATCbits.LATC2 = 0;

 

//            for (unsigned int j = 0; j < count; j++) {      // Time for the motor to settle.

//            }


//            LATCbits.LATC5 = 0;

            // 506 edges were consistantly counted with both edges being detected

//            int status = PORTAbits.RA4;     // The pin will either be +5 (at a slot)
// or 0 (on a divider).
            // I am detecting falling edges (dividers)
            // if status = 0 then
//            if(intOnChange > (319 - status)  ) // && intOnChange < 490)
//                chatter();

            // The full power move should give the number of interrupts in a full
            // rotation of the capacitor.

            
            // After the motor moves once it can move back, so I save the position
            // I moved too, let it generate more interrupts while it falls back and I will
            // subtract them before I try again. ie if it moves 5 I save five, it moves to 10


            // Determine if this duty cycle is suitable to move the motor
 


    return (EXIT_SUCCESS);
}

#endif

#ifdef _USE_IOC_
int resetMotor(unsigned char direction, unsigned char dutyCycle)
{
    // direction = 0 means get as close to zero capacitance as possible.
    // direction = 1 means get as much capacitance as possible.

    // Pulse Period is (PR2 + 1) * 4 * Tosc * TMR2

    int i,j;
    // Start modulating the EN pin of the SN75 H-bridge at 90% duty cycle
    startPWM(dutyCycle);               // 50% Duty Cycle 80, 90% 144

 

        if(direction == 0) {
            // Position to 0F capacitance by turning on C4 for 7800 NOP()'s
            LATCbits.LATC4 = 1;     // C4 is forward - 0F

            for (i = 0; i< 1000; i++)
                NOP();
            LATCbits.LATC4 = 0;     // Turn the motor off
        } else if (direction == 1) {
        // Position to 225pF capacitance by turning on C5 for 7800 NOP()'s
            LATCbits.LATC2 = 1;     // C4 is forward - 0F

            for (i = 0; i< 1000; i++)
                NOP();
            LATCbits.LATC2 = 0;     // Turn the motor off
        }

    return (EXIT_SUCCESS);
}

int rMotor(unsigned char direction, unsigned char dutyCycle)
{
    // direction = 0 means get as close to zero capacitance as possible.
    // direction = 1 means get as much capacitance as possible.

    // Pulse Period is (PR2 + 1) * 4 * Tosc * TMR2

    int i,j;
    // Start modulating the EN pin of the SN75 H-bridge at 90% duty cycle
    startPWM(dutyCycle);               // 50% Duty Cycle 80, 90% 144
    intOnChange = 0;


        if(direction == 0) {                // Just one direction each time
            for(j= 0; j < 120; j++) {       // This small movement will be tried j times
                // Position to 0 capacitance on C4 Up
                intLimit = intOnChange + 1; // set the IOC to turn off the port after 1 slot passes.
                startTMR1();
                LATCbits.LATC4 = 1;     // C4 is forward - 0F - UP
                while(LATCbits.LATC4) {};
//                for (i = 0; i< 35; i++)  // This much power is generally required to move the motor
//                    NOP();              // If it does move then IOC will turn off the port.
                LATCbits.LATC4 = 0;     // If it did not move turn the motor off
                stopTMR1();
                if(T1Count)
                    break;
                 for (unsigned int i = 0; i < 50; i++) {
                    NOP();
                    NOP();
                    NOP();
                    NOP();
                }
            }

        } else if (direction == 1) {
        // Position to 225pF capacitance on C2 Down
            for(j= 0; j < 100; j++) {
                intLimit = intOnChange + 1;
                startTMR1();
                LATCbits.LATC2 = 1;     // C4 is forward - DOWN
                while(LATCbits.LATC2) {};
//                for (i = 0; i< 47; i++)
//                    NOP();
                LATCbits.LATC2 = 0;     // Turn the motor off
                stopTMR1();
                if(T1Count)
                    break;
                 for (unsigned int i = 0; i < 50; i++) {
                    NOP();
                    NOP();
                    NOP();
                    NOP();
                }
            }
        }

    return (EXIT_SUCCESS);
}


int chatter()
{

    unsigned ixnt i, j;
    startPWM(255);
    j = 0;
    while(j< 5) {
    LATCbits.LATC2 = 0;
    for (i = 0; i < 1024; i++)
        LATCbits.LATC4 = 1;
    LATCbits.LATC4 = 0;
    for (i = 0; i < 1024; i++)
        LATCbits.LATC2 = 1;
    j++;
    }
    LATCbits.LATC2 = 0;
    return(EXIT_SUCCESS);
}

#endif

int resetMotor (unsigned char direction, unsigned char dutyCycle)
{
    int i;
    char buf[50];
    char sbuf[5];
    ADCValue = getADCValue();

    if(direction == CounterClockwise)
        oldADCValue = 2000;
    else
        oldADCValue = 2000;

#ifdef _DEBUG_POT
    strcpy(&buf[0],"Reset ");
    itoa (sbuf,ADCValue,10);
    strcat(buf,sbuf);
    strcat(buf," ");
    itoa (sbuf,oldADCValue,10);
    strcat(buf,sbuf);
    strcat(buf," ");
    putUSBUSART(buf,strlen(buf));
    CDCTxService();
    APP_DeviceCDCBasicDemoTasks();
#endif

    //  Move motor to one side
    for (i = 0; i < 100; i++) {      // TOTAL TURNS STILL NOT KNOWN!!!
        if (direction == CounterClockwise) {  // counter-clockwise
#ifndef __FullStep_
            stepA1();
            stepB1();
            stepA2();
            stepB2();
#else
            step4();
            stepB2();
            step3();
            stepB1();
            step2();
            stepA2();
            step1();
            stepA1();          
#endif
            lastStep = 4;
#if USE_ADC

            ADCValue = getADCValue(); // returns the integer from 0 - 1023 (10 bit conversion)
            if (ADCValue <= oldADCValue-3) {
                oldADCValue = ADCValue;
//                i = 94;
            }
            oldADCValue = ADCValue;

#endif
         } else {            // clockwise
//#ifndef __FullStep_
//            stepB2();
//            for (int j = 0; j < 10; j++) { // counter clockwise view from gear side
//            stepA1();       // 3A
//            __delay_ms(1000);
//            stepB1();
//            __delay_ms(1000);
//            stepA2();
//            __delay_ms(1000);
//            stepB2();
//            __delay_ms(1000);
//            }
//// __delay_ms(4000);           
////            for (int j = 0; j < 10; j++) {  // X
////            stepA1();       // 3A
////            __delay_ms(1000);
////            stepA2();
////            __delay_ms(1000);
////            stepB1();
////            __delay_ms(1000);
////            stepB2();
////            __delay_ms(1000);
////            }
////__delay_ms(4000);            
////             for (int j = 0; j < 10; j++) { //
////            stepA1();       // 3A
////            __delay_ms(1000);
////            stepA2();
////            __delay_ms(1000);
////            stepB2();
////            __delay_ms(1000);
////            stepB1();
////            __delay_ms(1000);
////            }  
//// __delay_ms(4000);           
////            for (int j = 0; j < 10; j++) {  //x
////            stepA1();       // 3A
////            __delay_ms(1000);
////            stepB1();
////            __delay_ms(1000);
////            stepB2();
////            __delay_ms(1000);
////            stepA2();
////            __delay_ms(1000);
////            }    
// __delay_ms(4000);           
//             for (int j = 0; j < 10; j++) {  //clockwise viewd from gear face
//            stepB1();       // 3A
//            __delay_ms(1000);
//            stepA1();
//            __delay_ms(1000);
//            stepB2();
//            __delay_ms(1000);
//            stepA2();
//            __delay_ms(1000);
//            }  
////__delay_ms(4000);            
////            for (int j = 0; j < 10; j++) {  //x
////            stepB1();       // 3A
////            __delay_ms(1000);
////            stepA1();
////            __delay_ms(1000);
////            stepA2();
////            __delay_ms(1000);
////            stepB2();
////            __delay_ms(1000);
////            }            
////__delay_ms(4000);            
////            for (int j = 0; j < 10; j++) {  //x
////            stepA2();       // 3A
////            __delay_ms(1000);
////            stepA1();
////            __delay_ms(1000);
////            stepB1();
////            __delay_ms(1000);
////            stepB2();
////            __delay_ms(1000);
////            }     
////__delay_ms(4000);            
////             for (int j = 0; j < 10; j++) { //x
////            stepA2();       // 3A
////            __delay_ms(1000);
////            stepA1();
////            __delay_ms(1000);
////            stepB2();
////            __delay_ms(1000);
////            stepB1();
////            __delay_ms(1000);
////            }     
////__delay_ms(4000);            
////             for (int j = 0; j < 10; j++) { // starts at a different point - correct sequence
////            stepB2();       // 3A
////            __delay_ms(1000);
////            stepA1();
////            __delay_ms(1000);
////            stepB1();
////            __delay_ms(1000);
////            stepA2();
////            __delay_ms(1000);
////            }           
//// __delay_ms(4000);      // 10         
////            for (int j = 0; j < 10; j++) {
////                __delay_ms(1000);
////            stepB2();       // 3A
////            __delay_ms(1000);
////            stepA1();
////            __delay_ms(1000);
////            stepA2();
////            __delay_ms(1000);
////            stepB1();
////            }            
// __delay_ms(4000);
//            for (int j = 0; j < 10; j++) {
//            stepB2();       // 3A
//            stepB1();
//            stepA1();
//            stepA2();
//            }
// __delay_ms(4000);           
//            for (int j = 0; j < 10; j++) {
//            stepB2();       // 3A
//            stepB1();
//            stepA2();
//            stepA1();
//            } 
// __delay_ms(4000);           
//             for (int j = 0; j < 10; j++) {
//            stepA2();       // 3A
//            stepB1();
//            stepB2();
//            stepA1();
//            }     
//            
// __delay_ms(4000);           
//            for (int j = 0; j < 10; j++) {
//            stepA2();       // 3A
//            stepB1();
//            stepA1();
//            stepB2();
//            }            
// 
// __delay_ms(4000);           
//             for (int j = 0; j < 10; j++) {
//            stepA2();       // 3A
//            stepB2();
//            stepA1();
//            stepB1();
//            }  
//            
// __delay_ms(4000);           
//             for (int j = 0; j < 10; j++) {
//            stepA2();       // 3A
//            stepB2();
//            stepB1();
//            stepA1();
//            }  
//__delay_ms(4000);            
//             for (int j = 0; j < 10; j++) {
//            stepB1();       // 3A
//            stepB2();
//            stepA1();
//            stepA2();
//            }   
//__delay_ms(4000);            
//             for (int j = 0; j < 10; j++) {
//            stepB1();       // 3A
//            stepB2();
//            stepA2();
//            stepA1();
//            }           
//__delay_ms(4000); 
//             for (int j = 0; j < 10; j++) {
//            stepB1();       // 3A
//            stepA2();
//            stepB2();
//            stepA1();
//            }           
//__delay_ms(4000);       //20     
//            for (int j = 0; j < 10; j++) {
//            stepB1();       // 3A
//            __delay_ms(1000);            
//            stepA2();
//            __delay_ms(1000);
//            stepA1();
//            __delay_ms(1000);
//            stepB2();
//            __delay_ms(1000);
//            }            
////__delay_ms(4000);            
////            for (int j = 0; j < 10; j++) {
////            stepA1();       // 3A
////            stepB2();
////            stepA2();
////            stepB1();
////            } 
////__delay_ms(4000);            
////            for (int j = 0; j < 10; j++) {
////            stepA1();       // 3A
////            stepB2();
////            stepB1();
////            stepA2();
////            }     
////
////__delay_ms(4000);            
////            for (int j = 0; j < 10; j++) {
////            stepB2();       // 3A
////            stepA2();
////            stepA1();
////            stepB1();
////            }
////__delay_ms(4000);            
////            for (int j = 0; j < 10; j++) {
////            stepB2();       // 3A
////            stepA2();
////            stepB1();
////            stepA1();
////            }            
////            stpA1();       // 4A
//#else
// __delay_ms(4000);            
 //           for (int j = 0; j < 100; j++) {
            stepA1();
            step1();
//            __delay_ms(20);
            stepA2();
            step2();
//            __delay_ms(20);
            stepB1();
            step3();
//            __delay_ms(20);
            stepB2();
            step4();
//            __delay_ms(20);
 
//#endif
//            lastStep = 4;
#if USE_ADC
            ADCValue = getADCValue(); // returns the integer from 0 - 1023 (10 bit conversion)
#if defined ( _USE_USB_)
#ifdef _DEBUG_POT
            strcpy(&buf[0],"ADC ");
            itoa (sbuf,ADCValue,10);
            strcat(buf,sbuf);
            strcat(buf," ");
            itoa (sbuf,oldADCValue,10);
            strcat(buf,sbuf);
            strcat(buf," ");
            putUSBUSART(buf,strlen(buf));
            CDCTxService();
            APP_DeviceCDCBasicDemoTasks();
#endif
#endif
//            if (ADCValue >= oldADCValue+1)  {         // 1.397/4.72 = 285
//                oldADCValue = ADCValue;
//                i = 94;
//            }
            oldADCValue = ADCValue;
#endif
        }

    }
//#if USE_ADC
//        oldADCValue = ADCValue;
//#endif
#if defined ( _USE_POT)
            strcpy(&buf[0],"END ");
            itoa (sbuf,ADCValue,10);
            strcat(buf,sbuf);
            strcat(buf," ");
            itoa (sbuf,oldADCValue,10);
            strcat(buf,sbuf);
            strcat(buf," ");
            putUSBUSART(buf,strlen(buf));
            CDCTxService();
         APP_DeviceCDCBasicDemoTasks();
#endif
         return 0;
}

int rMotor (unsigned char direction, unsigned int NumberOfSteps)
{
    while(NumberOfSteps > 0) {
        switch(lastStep) {
            case 1:
                if(direction == Clockwise) 
#ifndef __FullStep_
                    stepB1();
#else
                    step4();
#endif
                else
#ifndef __FullStep_                   
                    stepB1();
#else
                    step3();
#endif
                break;
            case 2:
                if (direction == Clockwise)
#ifndef __FullStep_                    
                    stepA1();
#else
                    step3();
#endif
                else
 #ifndef __FullStep_                   
                    stepA2();
#else
                    step2();
#endif
                break;
            case 3:
                if (direction == Clockwise)
#ifndef __FullStep_                   
                    stepB1();
#else
                    step4();
#endif
                else
#ifndef __FullStep_                   
                    stepB2();
#else
                    step1();
#endif
                break;
            case 4:
                if (direction == Clockwise)
#ifndef __FullStep_
                    stepA2();
#else
                    step1();
#endif
                else
#ifndef __FullStep_
                    stepA1();
#else
                    step4();
#endif
                break;
        }
        NumberOfSteps--;
    }
    return(EXIT_SUCCESS);
}

int stepA1(void)
{
    Motor4A = 1;
    __delay_ms(CounterCLockWise_Delay);
    Motor4A = 0;
    lastStep = 1;
    return(EXIT_SUCCESS);

}

int step1(void)
{
    Motor3A = Motor4A = 1;
    __delay_ms(CounterCLockWise_Delay);
    Motor3A = Motor4A = 0;
    lastStep = 1;
    return(EXIT_SUCCESS);
}

int stepA2(void)
{
    Motor3A = 1;
    __delay_ms(CounterCLockWise_Delay);
    Motor3A = 0;
    lastStep = 2;
    return(EXIT_SUCCESS);
}

int step2(void)
{
    Motor2A = Motor3A = 1;
    __delay_ms(CounterCLockWise_Delay);
    Motor2A = Motor3A = 0;
    lastStep = 2;
    return(EXIT_SUCCESS);
}


int stepB1(void)
{
    Motor2A = 1;
    __delay_ms(CounterCLockWise_Delay);
    Motor2A = 0;
    lastStep = 3;
    return(EXIT_SUCCESS);
}

int step3(void)
{
    Motor2A = Motor1A = 1;
    __delay_ms(CounterCLockWise_Delay);
    Motor2A = Motor1A = 0;
    lastStep = 3;
    return(EXIT_SUCCESS);
}

int stepB2(void)
{
    Motor1A = 1;
    __delay_ms(CounterCLockWise_Delay);
    Motor1A = 0;
    lastStep = 4;
    return(EXIT_SUCCESS);
}

int step4(void)
{
    Motor4A = Motor1A = 1;
    __delay_ms(CounterCLockWise_Delay);
    Motor4A = Motor1A = 0;
    lastStep = 4;
    return(EXIT_SUCCESS);
}