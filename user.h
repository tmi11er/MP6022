/* 
 * File:   user.h
 * Author: tmiller
 *
 * Created on January 6, 2016, 1:26 PM
 */

#ifndef USER_H
#define	USER_H

#ifdef	__cplusplus
extern "C" {
#endif
int setupADC(void);
int setupTimer0(void);
int setupTimer1(void);
int startTMR0 (void);
int startTMR1 (void);
int stopTMR0 (void);
int stopTMR1 (void);
unsigned int getADCValue(void);
int initIOC(void);
//int moveMotor(int numberOfSlots);
int initPWM (void);
int startPWM (int DutyCycle);
int increaseCapacitanceMotor(unsigned short value);
int resetMotor(unsigned char direction, unsigned char dutyCycle);
int rMotor(unsigned char direction, unsigned int dutyCycle);
int resetTimer1(void);
#ifdef _USE_IOC_
int chatter();
int calibrateCapacitor(int trials, unsigned char direction);
#endif
int stepA1(void), stepA2(void), stepB1(void), stepB2(void);
int setupComparator(void);
char* new_itoa(int num, char* str, int base);
int stepA1(void);
int step1(void);
int stepA2(void);
int step2(void);
int stepB1(void);
int step3(void);
int stepB2(void);
int step4(void);

#ifdef	__cplusplus
}
#endif

#endif	/* USER_H */

