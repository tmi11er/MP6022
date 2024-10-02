/*
 * File:   comparator.c
 * Author: tmiller
 *
 * Created on March 6, 2016, 15:45 PM
 */

// 'C' source line config statements


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

// For output to In house Turner from pin 7 RC3

int setupComparator(void)
{

    CM1CON0bits.C1SYNC = 1;     // Synchronize output with timer 1.
    CM1CON0bits.C1POL = 1;      // Do not Invert the output.
#ifdef __PIC16F1455__
    CM1CON0bits.C1OE = 0;       // Ouput not enabled (internal).
#else
    // Output is not enabled unless assigned through PSS
#endif
    CM1CON0bits.C1SP = 1;       // High speed mode.
    CM1CON0bits.C1HYS = 1;      // 75mV hysteresis
    CM1CON1bits.C1PCH = 2;      // Positive input to Fixed Voltage Reference.
    CM1CON1bits.C1NCH = 3;      // Negative input to C1IN3- pin. ( Pin 7).
    CM1CON0bits.C1ON = 1;       // Turn On Comparator.

//    // For testing comparator:
//    CM1CON1bits.C1INTN = 1;
//    CM1CON1bits.C1INTP = 1;
//    PIR2bits.C1IF = 0;
//    PIE2bits.C1IE = 1;      // interrupt
    return (EXIT_SUCCESS);
}