#ifndef __UTILITY_H
#define __UTILITY_H
#include "sys.h"

#define Stright 0
#define TurnLeft 1
#define TurnRight 2

#define LT90SEARCH 101	
#define	RT90SEARCH 102
#define POINTR90 103
#define POINTL90 104
#define STRAIGHT_spurt_90 105
#define LT90SPURT 106
#define	RT90SPURT 107

#define LT90L 1	
#define	RT90L 2
#define LV90L 3
#define RV90L 4
#define LT45L 5
#define RT45L 6
#define LV45L 7
#define RV45L 8
#define LT135L 9
#define RT135L 10
#define LV135L 11
#define RV135L 12
#define L180L  13
#define R180L  14
#define STRAIGHT90L 15
#define STRAIGHT45L 16

#define LT90M 21	
#define	RT90M 22
#define LV90M 23
#define RV90M 24
#define LT45M 25
#define RT45M 26
#define LV45M 27
#define RV45M 28
#define LT135M 29
#define RT135M 30
#define LV135M 31
#define RV135M 32
#define L180M  33
#define R180M  34
#define STRAIGHT90M 35
#define STRAIGHT45M 36

#define LT90H 41	
#define	RT90H 42
#define LV90H 43
#define RV90H 44
#define LT45H 45
#define RT45H 46
#define LV45H 47
#define RV45H 48
#define LT135H 49
#define RT135H 50
#define LV135H 51
#define RV135H 52
#define L180H  53
#define R180H  54
#define STRAIGHT90H 55
#define STRAIGHT45H 56

#define LOW_90 60
#define LOW 61
#define MEDIUM 62
#define HIGH 63

//冲刺直线校正方法
#define BOTHSIDE 80
#define LEFTSIDE 81
#define RIGHTSIDE 82
#define NOSIDE		83

void taskgostraight(u16 dis);
void taskturn(u8 pattern);
void taskturnaround(void);
void tasknull(void);
void taskstop(void);
void taskdelay(u16 ms);
void taskinfrstraight(u16 dis);
void taskturnrightfast(void);
void taskturnleftfast(void);
void tasktest(void);
void taskstraight45(u16 dis);
void taskacc(void);
void taskslowdown(u16 dis);
void taskspeedup(u16 dis);
void taskpointturnright(void);
void taskpointturnleft(void);


//==以下是实际使用的各种搜索拐弯函数====
void mazeSearch(void);
void mouseTurnright(void);
void mouseTurnright_spurt(void);
void mouseTurnright45(void);
void mouseTurnright135(void);
void mouseTurnrightfast(void);
void mouseTurnback(void);
void mouseTurnleft(void);
void mouseTurnleft_spurt(void);
void mouseTurnleft45(void);
void mouseTurnleft135(void);
void mouseTurnleftfast(void);
void mouseGoahead(char cNBlock); 
void mazesearchhaswall(void);
void mazesearchnowall(void);
void specialmouseTurnback(void);

void before180turn(void);
void after180turn(void);
void before90turn(u8 Direction);
void before90turn_l_spurt(void);
void before90turn_r_spurt(void);

int abovezero(int t);
void settrack(int t);
void setangle(int a);

void beforemouseturn(u8 pattern);
void mouseturn(u8 pattern);
void actiongenerate(u8 mode);
void action(u8 mode);
void taskstraight90(u16 dis);
void spurt(u8 mode);
int getfinaldir(void);

int getfinaldir_90(void);
void actiongenerate_90(u8 mode);
void action_90(u8 mode);
void spurt_90(u8 mode);

u8 keyCheck(void);
void __mouseCoorUpdate (void);
void __mouseCoorUpdate_Fast (u8 num);
void __wallCheck (void);

void spurt_back_mouseTurnback(void);
#endif






















