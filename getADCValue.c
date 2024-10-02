/*
 * File:   getADCValue.c
 * Author: tmiller
 *
 * Created on January 6, 2016, 14:10 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

/*
 * The ADC is assumed to be not operating upon entry
 * Turn it on, get the result, and turn it off
 */
int setupADC(void) {

//    ANSELCbits.  .ANSC4 = 1;  // Select this port - RC3, Pin 7 - for Analogue input. (2nd time)

    ADCON1bits.ADCS = 6;   // Conversion clock FOSC/x, sample time 8.0uS of 8.5uS (worst case scenario) +/- 3uS


    FVRCONbits.FVREN = 1;   // Turn on FVR
    FVRCONbits.ADFVR = 3;   // 4x = 4.096V
    ADCON1bits.ADPREF = 3;  // Use internal FVR at 4.096, max input is calculated at 4.003V

    ADCON0bits.CHS = 5;     // RC1 is AN5 pin 9.
    ADCON0bits.ADON = 1;       // Turn on the A/D
    ADCON1bits.ADFM = 1;        // Right formatted output

    return EXIT_SUCCESS;

}

unsigned int getADCValue(void) {

    unsigned int ADCValue;

    ADCON0bits.GO = 1;      // Start the conversion

    while(ADCON0bits.GO_nDONE) {};  // Wait for conversion to complete

    ADCValue = ADRESH;
    ADCValue =  ADCValue << 8;
    ADCValue |= ADRESL;  // The Voltage

    return (ADCValue);
}