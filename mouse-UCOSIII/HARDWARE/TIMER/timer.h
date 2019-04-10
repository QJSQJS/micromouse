#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

#define MAXPWM	1500
#define LPWML TIM4->CCR2	//PB7
#define LPWMH TIM4->CCR1	//PB6
#define RPWMH TIM4->CCR4	//PB9
#define RPWML TIM4->CCR3	//PB8

#define ACCV0 10

#define SPEED_HAHA 350

#define RIGHTPWM TIM4->CCR3 
#define LEFTPWM TIM4->CCR1
#define RIGHTDIR PBout(9)
#define LEFTDIR PBout(7)

//tasks
#define NULL 0
#define GOSTRAIGHT 1
#define TURN 2
#define TURNAROUND 4
#define STOP 5
#define SUPERSTRAIGHT 12
#define INFRSTRAIGHT 13
#define TURNLEFTFAST 14
#define TURNRIGHTFAST 15
#define EMERGENCY 16
#define STRAIGHT45 17
#define FASTSTRAIGHT 18
#define BEFORE90TURN 27
#define BEFORE180TURN 28
#define AFTER180TURN 29
#define SMARTSTRAIGHT 30
#define CHANGESPEED 31
#define SMARTSTRAIGHTFAST 32  
#define STRAIGHTEYECLOSECOORADJ 33
#define STRAIGHTPOLE 34
#define ACC 35
#define SLOWDOWN 36
#define SPEEDUP 37
#define KEEPWALLDISTANCE 38
#define POINTTURN90 39
#define EDGESTRAIGHT 40
#define FORWARDIRTEST 41
#define SPURT90BLIND 42
#define SPURT90IR 43
#define SMARTSTRAIGHTSPURT 44
#define SMARTSTRAIGHTSPURTL 45

extern long leftb,rightb,leftbh,rightbh,loopahead,absoluteangle,baseangle,angle,anglehis[],newindex,anglehismax,anglehismin;

extern u8 gangleSee,isSpeedTest;
extern int speedListL[1],speedListR[1];//51

void 	Left_Code_Init(void);
void PWM_motor_Init(void);
void Timer2_Init(void);
void Timer3_Init(void);
void Timer1_Init(void);
int speedlimit(int s);
int errorlimit(int e);

int abs(int t);
void send(long s);

#endif






















