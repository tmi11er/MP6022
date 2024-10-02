/*
 * File:   PWM.c
 * Author: tmiller
 *
 * Created on February 10, 2016, 8:40 PM
 */

#include <stdio.h>
#include <stdlib.h>

// 'C' source line config statements

#include <xc.h>


// PWM on RC3, Pin 7
int initPWM(void)
{

    // The timer period = (PR2 + 1) * 4 * Tosc * PR2  ( Tosc = 1/Fosc)
    // The largest period is 16.32ms, the smallest 500ns
    // The timer frequency is the inverse of the timer period
    // The largest frequency is 2MHz, the smallest 61.27Hz

    TRISCbits.TRISC5 = 1;   // Disable the PWM1 pin output driver - this drove the motor.
    
#ifdef __PIC16F1455__
    PWM1CON = 0;            // Clear the PWMxCON register.
#else
    PWM5CON = 0;
#endif
    PR2 = 59;            // Period value = (0x244 + 1) * 4 * 1/16MHz * 64 = 4.096us - 244Hz
#ifdef __PIC16F1455__   
    PWM1DCH = 0;            // Clear to start
    PWM1DCL = 0;            // Clear to start
#else
    PWM5DCH = 0;
    PWM5DCL = 0;
#endif
    TMR2 = 255;             // Timer period register
    T2CONbits.TMR2ON = 0;       // Turn Timer off
    T2CONbits.T2CKPS = 0;       // Post Scaler T2 not used.
    PIE1bits.TMR2IE = 1;        // T2 interrupt enable.
    PIR1bits.TMR2IF = 0;        // Clear timer 2 interrupt flag
    T2CONbits.T2CKPS = 00;       // Prescaler value. 0 = 0, 1 = 4, 2 = 16, 3 = 64
    PIR1bits.TMR2IF = 0;    // Disable timer interrupt
    T2CONbits.TMR2ON = 1;   // Turn the timer on.
 #ifdef __PIC16F1455__   
    PWM1CONbits.PWM1OE = 1; // Enable PWM Output pin.
#else
   // PWM5 output will be assigned with PPS or something else is going on.
#endif
    while(!PIR1bits.TMR2IF) {};     // Wait until timer times out
    PIR1bits.TMR2IF = 0;        // Clear the interrupt
    TRISCbits.TRISC5 = 0;       // Enable pin output
//    PWM2CONbits.PWM2EN = 1;     // Enable PWM
//    PWM2DCH = 0xF9>>2;           // Duty Cycle Ratio = 122/(4(PR2+1)) = 0.119 / 12%
//    PWM2DCL = 0xF9|0x3;          // Pulse Width = DCJ:DCL * 1/16MHz * 64 = 0.488us
    return EXIT_SUCCESS;
}

int startPWM(int DutyCycle)
{
#ifdef __PIC16F1455__
    PWM1CONbits.PWM1EN = 1;     // Enable PWM
    PWM1DCH = DutyCycle >> 2;           // Duty Cycle Ratio = 122/(4(PR2+1)) = 0.119 / 12%
    PWM1DCL = DutyCycle | 0x3;          // Pulse Width = DCJ:DCL * 1/16MHz * 64 = 0.488us
#else
    PWM5CONbits.PWM5EN = 1;     // Enable PWM
    PWM5DCH = DutyCycle >> 2;           // Duty Cycle Ratio = 122/(4(PR2+1)) = 0.119 / 12%
    PWM5DCL = DutyCycle | 0x3;          // Pulse Width = DCJ:DCL * 1/16MHz * 64 = 0.488us    
    
    
#endif
    return EXIT_SUCCESS;
}