/* 
 * File:   ioc.c
 * Author: tmiller
 *
 * Created on January 6, 2016, 7:20 AM
 */

#include <stdio.h>
#include <stdlib.h>
// PIC16F1455 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

#include "user.h"
#include "system.h"


#ifdef _USE_IOC_
volatile extern unsigned int intOnChange;
volatile extern unsigned int intLimit;
volatile extern unsigned int T0Count;
volatile extern unsigned int T1Count;
#endif
volatile extern unsigned int pulseCount;
volatile extern unsigned int Timer0Int;



/*
 * 
 */

int initIOC(void)       // Old code for old motor.
{
    // On pic16f1455:  Prepare for Interrupt on Change signals on RA4 PIN 3 from SN75176 pin 4 Data IN (Read)
//    IOCANbits.IOCAN5 = 1;   // Detecta falling edge on RA4
////    IOCAPbits.IOCAP4 = 1;   // Detect a rising edge on RA4   
    // On pic16F18345:  Prepare for Interrupt on Change signals on RA4 PIN 3 from SN75176 pin 4 Data IN (Read)
    IOCANbits.IOCAN4 = 1;       // Detect a falling edge on RA4 (pic16f8345)
    // Cascading two transistors will produce an IOC signal.
    // Enable Interrupt on Change pic16f1455
//    INTCONbits.IOCIE = 1;  // Enable IOC
    // Enable Interrupt on Change pic16f18345
//    PIE0bits.IOCIE = 1;
//    INTCONbits.GIE = 1;     // Enable all interrupts that are enabled.
    
    // Set the port bits pic16F1455 - this does not belong here, it controls
    // the SN75176
//    TRISAbits.TRISA5 = 1;
    //
//    LATAbits.LATA4 = 0;
    IOCBPbits.IOCBP4 = 1;    
    
    // FAULT CONDITION ON MP6022 pin 8
    IOCANbits.IOCAN5 = 1;       // Detect falling edge on RC5 (Fault condition)
    PIE0bits.IOCIE = 1;         // Enable interrupt on Change
    



//    intOnChange = 0;
    return EXIT_SUCCESS;
}

unsigned char AnswerSPI[3] = {0,0,0};
unsigned char AnswerSDI[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int SPIRead, ReadSDI;
void __interrupt()  myIsr (void) {

#ifdef _USE_IOC_
    if(INTCONbits.IOCIF) {  // If the Interrupt on Change is enabled
//        if(IOCAFbits.IOCAF5) {        // If Falling edge or Rising edge Interrupt on Change
                intOnChange++;        // nearly 100% duty cycle
                IOCAF &= 0xDF;     // Clear falling edge flag
                if (intOnChange >= intLimit)
                    LATC &= 0x2B;  // C2 & C4 are bits 2 and 4
//        }
        INTCON &= 0xF7;
        return;
    }


    if(INTCONbits.T0IF) {
        INTCON &= 0xFB;
//        LATC = LATC & 0x2B;  // C2 & C4 are bits 2 and 4
//        T0Count = 1;
        return;
    }

     if(PIR1bits.TMR2IF) {
        PIR1 &= 0xFD;
        return;
    }

    if(PIR1bits.TMR1IF) {
        PIR1 &= 0xFE;
        LATC = LATC & 0x2B;
        T1Count = 1;
        return;
    }

#endif
    
#ifdef __PIC16F1455__
    if (INTCONbits.T0IF) {
        INTCONbits.TMR0IF = 0;
        Timer0Int = 0;
    }
#else
    
    if(PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        Timer0Int = 0;
    }
#endif

#if PIC16F1455
    if (INTCONbits.INTF) {
        INTCONbits.INTF = 0; //  &= 0xFD;
        return;
    }
#else
    if (IOCAFbits.IOCAF4) {
        IOCAFbits.IOCAF4 = 0;

    }
    if (IOCAFbits.IOCAF5) {     // Clear Fault condition of MP6602
        IOCAFbits.IOCAF5 = 0;   // Conditions: over current, over voltage
                                // under voltage , thermal warning,
                                // thermal shutdown, open load, stall
    }
    
    if (IOCCFbits.IOCCF6) {
        if(IOCCN6)
            AnswerSDI[ReadSDI++] = PORTBbits.RB4;
        IOCCFbits.IOCCF6 = 0;
    }
                           
    
    if(PIR1bits.SSP1IF) {   // SPI1 Read/Write Status interrupt Flag
        if(SPIRead) 
            AnswerSPI[SPIRead++] = SSP1BUF;
        PIR1bits.SSP1IF = 0;
    }
    
     if (IOCBFbits.IOCBF4) {     // This pin does not seem to be able to get input.
        IOCBFbits.IOCBF4 = 0;
    }   
    
    if(PIR1bits.BCL1IF) {   // Clear Bit Collision on SPI1 interrupt Flag   
        PIR1bits.BCL1IF = 0;

    }
#endif

    if(PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;

    }

    if(PIR1bits.TMR1GIF) {      // Gated timer single event duration.  This timer counts in 4MHz^-1 = 250ns increments.
                                // A 400 pulse number * 500ns = 200 microseconds.  This timer can count to 65536*250ns = 16.384 milliseconds
                                // so it is big enough.  
        pulseCount = TMR1H;
        pulseCount = (pulseCount << 8) | TMR1L;
        PIR1bits.TMR1GIF = 0;

    }
 
    if(PIR1bits.ADIF) {     // A/D conversion complete
        PIR1bits.ADIF = 0;

    }

//    // For testing Comparator
//    if(PIR2bits.C1IF) {
//        PIR2bits.C1IF = 0;
//        return;
//    }



#if defined(_USE_USB_)
#if defined(USB_INTERRUPT)
        USBDeviceTasks();
#endif
#endif
        return;
        
}


//int moveMotor(int numberOfSlots)
//{
//    intOnChange = 0;
//    numberOfSlots *= 2;    // Rising and Falling edges are detected
//    while (intOnChange < numberOfSlots > 0 ) {
//        if(!intOnChange) {                  // One time, turn on the motor
//            if(numberOfSlots > 0) {
//                LATCbits.LATC4 = 1; // Forward
//            } else {
//                numberOfSlots *= -1;
//                LATCbits.LATC3 = 1; // Reverse
//            }
//
//        }
//    }
//    return EXIT_SUCCESS;
//}