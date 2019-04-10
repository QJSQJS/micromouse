#include "timer.h"
#include "sys.h"
#include "stmflash.h"
#include "usart.h"		
#include "delay.h"	
#include "adc.h"
#include "utility.h"
#include "stratagy.h"
#include "led.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <math.h>

u16 led_test=0;//调试参数，能走就删除
int 	TIM2CNT=0; 
 
extern  uchar    GucMouseTask;
u8 save_data_temp=0;
u16 save_data_num=0;
u8 save_data_time=1;
extern const u16 save_num;
extern const u8 per_timer;

//常量定义
#define COORADJUST				//是否开启纵向矫正
u8 EMERGENCYON=1;				//是否开启撞墙刹车功能
#define SHOWCPU				//是否开启显示时钟利用率
//#define SHOWVOLTAGE
#define FLASHSAVE				//共要修改三个地方

//调试标志cjm
u8 gangleSee = 0;
u8 isSpeedTest = 0;
int speedListL[1] = {0};//51
int speedListR[1] = {0};//51
int speedListi = 0;

const u16 angleratio=306;		//编码器角度系数 ADC/degree 值越大，转的角度越大		261
const u16 gangleratio=522;	//陀螺仪角度系数 周期1ms时是175。值越大，转的角度越大			1034/2
const float TrackRatioL=243.59 * 0.9,TrackRatioR=246.70*0.9;	//编码器距离系数 pulse/mm 值越大，走的距离越大  222.67   221.67
const double SpeedRatioL=2.4359*0.9,SpeedRatioR=2.4670*0.9;	//cm/s -> pulse/ms 系数越大走得越快		2.2267  2.216
u8 sidewall_threshold=160;		//160两侧有墙的阈值，该数值设置较大是考虑到单独柱子测距会偏大。	
const u8 dis2wall=44;			//车在中央时传感器测得到墙的距离
const u8 trackcoor=137+8+1;		//纵向矫正，测量越灵敏，此值越小，车拐得越晚	 149mm即有墙无墙变化点出现在距离柱子中间149-22=127mm处（70cm/s时）  

u16 speed_ss=70;				//搜索直线速度
u16 speed_turn=70;				//搜索转弯和加减速速度
u16 speed_fastss=70;			//冲刺直线最大速度？？？	
double GOSTRAIGHT_kp=0.4;
double SMARTSTRAIGHTFAST_bigerror_kp=0.5;		//0.3 2014速度降低后新改的！！！！！！！！！！！！！！
double SMARTSTRAIGHTFAST_smallerror_kp=0.35;	//0.15
double STRAIGHT45_kp=0.12,STRAIGHT45_kd=0.1;
double turn_kp=10,turn_kd=6;
float SPURT90IR_kp1,SPURT90IR_kp2,SPURT90IR_kd,SPURT90GYRO_kp,SPURT90GYRO_kd;
double kiturn;

//变量定义
//编码器测速和PID相关
int lw,rw;						//左右轮转过的角度和格数。
int lwh=0,rwh=0;				//上一时刻左右轮转过的角度，用于计算速度
int gdspeedl,gdspeedr;			//左右轮编码器测量速度
int speedl,speedr,speed; 		//以cm/s为单位的真实速度	
long setvl=0,setvr=0;			//左右轮设定速度
double gdvl=0,gdvr=0;
int errl=0,errlh=0,errr=0,errrh=0,errls=0,errrs=0;
double pwml,pwmr;
double kpl=6,kil=2,kdl=0,kpr=6,kir=2,kdr=0;
//编码器角度相关
long leftb,rightb,leftbh,rightbh,loopahead=0,absoluteangle=0,baseangle=0,angle,angleskip=0,anglehis[50]={0},newindex=0,anglehismax=-100,anglehismin=100;
long gangleh,ganglehh,peakgangle,peakgangleh;
//陀螺仪角度相关
int gyroinit=0,gyroswitch=0;
int gyro=0,gyro0;
int gdiff=0,gdiffh=0,gangle=0,ganglet=0,ggangleh=0,setgangle=0;
u16 angle_counter=0;
u8 angleterm=0,gyro_adj=0;						   //Q=0.000912,P=0.368		  Q=1.0,P=4000.0
double g=1888.0,covg=1.0,gtmp,covgtmp,kg,Q=0.000912,P=0.368;//Q是系统噪声的cov，P是测量噪声的cov.Q不变时，P越大，输出越稳定且反应迟钝。
//两轮前进距离相关
long leftd,rightd,leftdh,rightdh,leftcycle=0,rightcycle=0,absolutetrack,absolutetrackl,absolutetrackr,basetrack=0,basetrackl=0,basetrackr=0,trackl,trackr,track;
//红外测距和矫正相关
long disf1,disf2,disf,disl,disr,disl45,disr45;
long dislh,disrh,dislhh,disrhh,disl45h,disr45h,disl45hh,disr45hh;
u8 lefthaswall=1,righthaswall=1,forwardhaswall,lefthaswallh=1,righthaswallh=1;
u8 lefthaswall45=0,righthaswall45=0,lefthaswallh45=0,righthaswallh45=0;
u8 leftwallcritical=0,rightwallcritical=0,leftwallcriticalh=0,rightwallcriticalh=0,leftwallcriticalhh=0,rightwallcriticalhh=0,leftwallcriticalhhh=0,rightwallcriticalhhh=0;;
u16 dislpeak=100,disrpeak=100,leftpeaktrack=0,rightpeaktrack=0;
u8 lastside=LEFT;
u8 irswitch=1;
u8 infrskipround=0;
//通用PID误差变量
long e,eh=0,esum=0;
u16 tick=0;  //通用延时变量
u8 step;
//当前任务标识
u8 task=0;
u16 turnaround_tick=0;
u8 blocknum=1,blocknum45=0;	//判断连续走了几个格子用
double battary,battaryh=0;
u8 battarypercent=0;
u16 isemergency=0,emergency_count=0,emergency_delay=0;
int cpu0,cpu;
u8 freemode=0;
u8 sleepmode=0;
extern u8 lpeakfindenable,rpeakfindenable;
extern u16 flashaddr,flashaddrmax;
extern u8 flashwrite,flashenable;
const short *turn_gyro,*turn_vl,*turn_vr;
double decratio,accratio;
long queue[50]={0};
u8 queuenum=0,queuei,anglei;
int wtf;
int forwardirdis;
int stoptick;
extern int showbattery;

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void Timer2_Init()			//PA15 PB3 两相编码器输入，左轮
{
	RCC->APB2ENR|=3<<2;		//GPIOA B Clock Enable
	GPIOA->CRH&=0x0FFFFFFF;
	GPIOA->CRH|=0x80000000;
	GPIOB->CRL&=0xFFFF0FFF;
	GPIOB->CRL|=0x00008000;	//PA15 PB3弱下拉输入
	
	RCC->APB2ENR|=1<<0;		//Alternate Function I/O Clock Enable
	RCC->APB1ENR|=1<<0;		//TIM2 Clock Enable
	AFIO->MAPR&=0xF8FFFCFF;
	AFIO->MAPR|=0x04000000;	//JTAG-DP SW-DP Clock Disable
	AFIO->MAPR&=0xFFFFFCFF;
	AFIO->MAPR|=1<<8;		//TIM2 Partial Remap1
		
	TIM2->ARR=10000-1;		//top
	TIM2->PSC=0;			//1-Divided
	TIM2->CCER|=0<<1;		//CC1反相
	TIM2->CCMR1|=1<<9;		//IC2->TI2
	TIM2->CCMR1|=1<<1;		//IC1->TI1
	TIM2->SMCR|=0x0003;		//编码器模式3
	TIM2->SR=0x0000;		//清中断
	TIM2->CNT=0x0000;
	TIM2->CR1|=1<<0;		//使能定时器2
	
  
}

void Timer3_Init()			//PB4 PB5两相编码器输入，右轮
{
	RCC->APB2ENR|=1<<3;		//GPIOB Clock Enable
	GPIOB->CRL&=0xFF00FFFF;
	GPIOB->CRL|=0x00880000;	//PB4 PB5弱下拉输入
	
	RCC->APB1ENR|=1<<1;		//TIM3 Clock Enable
	AFIO->MAPR&=0XFFFFF3FF;
	AFIO->MAPR|=1<<11;		//TIM3 Partial Remap1
	
	TIM3->ARR=10000-1;		//top   
	TIM3->PSC=0;			//1-Divided
	TIM3->CCER|=1<<1;		//CC1反相
	TIM3->CCMR1|=1<<8;
	TIM3->CCMR1|=1<<0;
	TIM3->SMCR|=0x0003;		//编码器模式3
	TIM3->SR=0x0000;		//清中断 
	TIM3->CNT=0x0000;
	TIM3->CR1|=1<<0;		//使能定时器3		   
}

void Timer1_Init()			//延时函数
{
	RCC->APB2ENR|=1<<11;	//TIM1时钟使能    
 	TIM1->ARR=10000;		//设定计数器自动重装值
	TIM1->PSC=71;			//72分频 1MHz
	TIM1->CR1|=1<<0;		//使能定时器1									 
}




void PWM_motor_Init()		//电机PWM控制
{		 					 
	//主电机
	RCC->APB1ENR|=1<<2;		//TIM4 Clock Enable
	RCC->APB2ENR|=1<<3;		//GPIOB Clock Enable
	  	
	GPIOB->CRL&=0X00FFFFFF;
	GPIOB->CRL|=0XBB000000;	//PB6 7复用推挽输出
	GPIOB->CRH&=0XFFFFFF00;
	GPIOB->CRH|=0X000000BB;	//PB8 9复用推挽输出
	
	TIM4->ARR=MAXPWM;		//Period
	TIM4->PSC=1;			//2-Divided 36MHz
	
	TIM4->CCMR1&=0x0C0C;	//CH1 CH2 Output Compare
	TIM4->CCMR1|=6<<4;		//CH1 PWM1 MODEL
	TIM4->CCMR1|=6<<12;		//CH2 PWM1 MODEL
	
	TIM4->CCMR2&=0x0C0C;	//CH3 CH4 Output Compare
	TIM4->CCMR2|=6<<4;		//CH3 PWM1 MODEL
	TIM4->CCMR2|=6<<12;		//CH4 PWM1 MODEL
	
	TIM4->CCMR1|=1<<3;		//CH1 Preload Enable
	TIM4->CCMR1|=1<<11;		//CH2 Preload Enable
	TIM4->CCMR2|=1<<3;		//CH3 Preload Enable
	TIM4->CCMR2|=1<<11;		//CH4 Preload Enable 

	TIM4->CCER|=1<<0;		//OC1 Output Enable
	TIM4->CCER|=1<<4;		//OC2 Output Enable
	TIM4->CCER|=1<<8;		//OC3 Output Enable
	TIM4->CCER|=1<<12;		//OC4 Output Enable   

	TIM4->CR1|=1<<7;		//Auto-reload Preload Enable
	TIM4->CR1|=1<<0;		//TIM4 Enable
}


int alimit(int e) //
{
 	int maxa=10000;
	if(e>maxa)return maxa;
	else if(e<-maxa)return -maxa;
	else return e;
}


int elimit(int e) //
{
 	u8 maxe=130;
	if(e>maxe)return maxe;
	else if(e<-maxe)return -maxe;
	else return e;
}

int abs(int t)
{
	if(t>=0)return t;
	else return -1*t;
}

void send(long s)
{
	queue[queuenum]=s;
	queuenum++;
}




