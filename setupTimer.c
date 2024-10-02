/* 
 * File:   setupTimer1.c
 * Author: tmiller
 *
 * Created on January 6, 2016, 12:58 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

/*
 * 
 */

#ifdef _USE_IOC_
volatile unsigned int T0Count;
volatile unsigned int T1Count;
#endif

volatile extern unsigned int pulseCount;
volatile extern unsigned int Timer0Int;

int setupTimer0(void)
{

    // Set clock to FOSC/4 pic16F1455
    //    OPTION_REGbits.TMR0CS = 0;    // timer FOSC/4 timer, not counter
    // Set clock to FOSC/4 pic16F18345
    T0CON1bits.T0CS = 0b010;
 
////    ADCON2bits.TRIGSEL = 0x3;   // Timer 0 overflow


////    T1CONbits.T1CKPS = 2;   // Divide by 4 giving 250kHz
////    T1CONbits.T1OSCEN = 1;  // Enable timer
////    PR2 = 256;              // Timer period
////    T1CONbits.TMR1ON = 1;   // Turn timer on

    return (EXIT_SUCCESS);
}

int setupTimer1(void)
{
    T1GCONbits.TMR1GE = 1;      // Configure as gated timer
    T1CONbits.TMR1CS =  1;      // System Clock Source (FOSC/4 - 0)

    T1GCONbits.T1GSS = 2;       // Gate Source is Comparator 1 (Pin 7)
    T1CONbits.T1CKPS = 0;       // Prescaler /8=3
    T1GCONbits.T1GTM =  0;      // Gate toggle off
    T1GCONbits.T1GPOL = 1;      // Timer active when gate is high.

    T1GCONbits.T1GSPM = 1;      // Single pulse mode
#ifdef __PIC16F1455__
    T1CONbits.nT1SYNC = 1;      // Synchronized
#else
    T1CONbits.T1SYNC = 1;       // Synchronized
#endif
    TMR1H = 0;
    TMR1L = 0;
    PIE1bits.TMR1GIE = 1;       // Enable single pulse measured interupt
    T1CONbits.TMR1ON = 1;       // Timer On
//    PIE1bits.TMR1IE = 1;


    

    return (EXIT_SUCCESS);
}

int resetTimer1(void)
{

    T1CONbits.TMR1ON = 0;
    TMR1H = 0;
    TMR1L = 0;
    pulseCount = 0;
    T1GCONbits.T1GGO_nDONE = 1; // Ready for single pulse
    T1CONbits.TMR1ON = 1;

    return (EXIT_SUCCESS);
}

#ifdef _USE_IOC_
int startTMR0 ()
{
    T1CONbits.TMR1ON = 1;       // Turn on timer 1
    T0Count = 0;
    INTCONbits.T0IE = 1;      // Enable timer interrupt
    OPTION_REGbits.T0CS=0;    // Fosc/4  timer is the instruction clock
    TMR0 = 255;
    return (EXIT_SUCCESS);
}
#endif
int startTMR1(void)
{

    PIE1bits.TMR1IE = 1;        // Timer interrupt enable
    INTCONbits.PEIE = 1;        // Peripheral enable
    PIR1bits.TMR2IF = 0;        // Timer to flag clear
    TMR1H = 0;           // value
    TMR1L = 0;           //  value
//    T1Count = 0;
    T1CONbits.TMR1ON = 1;       // Turn on timer 1
    return (EXIT_SUCCESS);

}

int startTMR0(void)
{
#ifdef _PIC16F1455_
    INTCONbits.T0IF  = 0;
    INTCONbits.T0IE = 1;        // Enable timer 1 interrupt.
    TMR0 = 0;
//    OPTION_REGbits.PSA = 1;     // prescaler not assigned to TMR1
//      OPTION_REGbits.PSA = 1;     // prescaler not assigned to TMR1
     OPTION_REGbits.PS = 7;      // 256 Prescaler    
     Timer0Int = 1;  
#else
    // Pic16F18345
    PIR0bits.TMR0IF = 0;    // Clear Timer0 Interrupt flag
    PIE0bits.TMR0IE = 1;    // Enable timer 1 external interrupt
    TMR0 = 0;
    T0CON1bits.T0CKPS = 0b1000; // 256 Prescale (input and not output assumed 9/29/24)
#endif    

    return (EXIT_SUCCESS);
}

int stopTMR0 (void)
{
//    OPTION_REGbits.T0CS= 1; // timer is on edge of CLKT0
#ifdef _PIC16F1455_
    INTCONbits.T0IE = 0;
#else
    PIE0bits.TMR0IE = 0;
#endif
    return (EXIT_SUCCESS);
}

int stopTMR1(void)
{
    T1CONbits.TMR1ON = 0;
    PIE1bits.TMR1IE = 0;
    return (EXIT_SUCCESS);
}