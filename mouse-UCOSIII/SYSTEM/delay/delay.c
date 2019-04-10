#include <stm32f10x.h>
#include "delay.h"
//#include "stmflash.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//使用SysTick的普通计数模式对延迟进行管理（适合STM32F10x系列）
//包括delay_us,delay_ms
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2010/1/1
//版本：V1.8
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.2修改说明
//修正了中断中调用出现死循环的错误
//防止延时不准确,采用do while结构!
//V1.3修改说明
//增加了对UCOSII延时的支持.
//如果使用ucosII,delay_init会自动设置SYSTICK的值,使之与ucos的TICKS_PER_SEC对应.
//delay_ms和delay_us也进行了针对ucos的改造.
//delay_us可以在ucos下使用,而且准确度很高,更重要的是没有占用额外的定时器.
//delay_ms在ucos下,可以当成OSTimeDly来用,在未启动ucos时,它采用delay_us实现,从而准确延时
//可以用来初始化外设,在启动了ucos之后delay_ms根据延时的长短,选择OSTimeDly实现或者delay_us实现.
//V1.4修改说明 20110929
//修改了使用ucos,但是ucos未启动的时候,delay_ms中中断无法响应的bug.
//V1.5修改说明 20120902
//在delay_us加入ucos上锁，防止由于ucos打断delay_us的执行，可能导致的延时不准。
//V1.6修改说明 20150109
//在delay_ms加入OSLockNesting判断。
//V1.7修改说明 20150319
//修改OS支持方式,以支持任意OS(不限于UCOSII和UCOSIII,理论上任意OS都可以支持)
//添加:delay_osrunning/delay_ostickspersec/delay_osintnesting三个宏定义
//添加:delay_osschedlock/delay_osschedunlock/delay_ostimedly三个函数
//V1.8修改说明 20150519
//修正UCOSIII支持时的2个bug：
//delay_tickspersec改为：delay_ostickspersec
//delay_intnesting改为：delay_osintnesting
//////////////////////////////////////////////////////////////////////////////////  
extern u8 task;
static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数
	
	
#if SYSTEM_SUPPORT_OS							//如果SYSTEM_SUPPORT_OS定义了,说明要支持OS了(不限于UCOS).
//当delay_us/delay_ms需要支持OS的时候需要三个与OS相关的宏定义和函数来支持
//首先是3个宏定义:
//    delay_osrunning:用于表示OS当前是否正在运行,以决定是否可以使用相关函数
//delay_ostickspersec:用于表示OS设定的时钟节拍,delay_init将根据这个参数来初始哈systick
// delay_osintnesting:用于表示OS中断嵌套级别,因为中断里面不可以调度,delay_ms使用该参数来决定如何运行
//然后是3个函数:
//  delay_osschedlock:用于锁定OS任务调度,禁止调度
//delay_osschedunlock:用于解锁OS任务调度,重新开启调度
//    delay_ostimedly:用于OS延时,可以引起任务调度.

//本例程仅作UCOSII和UCOSIII的支持,其他OS,请自行参考着移植
//支持UCOSII
#ifdef 	OS_CRITICAL_METHOD						//OS_CRITICAL_METHOD定义了,说明要支持UCOSII				
#define delay_osrunning		OSRunning			//OS是否运行标记,0,不运行;1,在运行
#define delay_ostickspersec	OS_TICKS_PER_SEC	//OS时钟节拍,即每秒调度次数
#define delay_osintnesting 	OSIntNesting		//中断嵌套级别,即中断嵌套次数
#endif

//支持UCOSIII
#ifdef 	CPU_CFG_CRITICAL_METHOD					//CPU_CFG_CRITICAL_METHOD定义了,说明要支持UCOSIII	
#define delay_osrunning		OSRunning			//OS是否运行标记,0,不运行;1,在运行
#define delay_ostickspersec	OSCfg_TickRate_Hz	//OS时钟节拍,即每秒调度次数
#define delay_osintnesting 	OSIntNestingCtr		//中断嵌套级别,即中断嵌套次数
#endif


//us级延时时,关闭任务调度(防止打断us级延迟)
void delay_osschedlock(void)
{
#ifdef CPU_CFG_CRITICAL_METHOD   				//使用UCOSIII
	OS_ERR err; 
	OSSchedLock(&err);							//UCOSIII的方式,禁止调度，防止打断us延时
#else											//否则UCOSII
	OSSchedLock();								//UCOSII的方式,禁止调度，防止打断us延时
#endif
}

//us级延时时,恢复任务调度
void delay_osschedunlock(void)
{	
#ifdef CPU_CFG_CRITICAL_METHOD   				//使用UCOSIII
	OS_ERR err; 
	OSSchedUnlock(&err);						//UCOSIII的方式,恢复调度
#else											//否则UCOSII
	OSSchedUnlock();							//UCOSII的方式,恢复调度
#endif
}

//调用OS自带的延时函数延时
//ticks:延时的节拍数
void delay_ostimedly(u32 ticks)
{
#ifdef CPU_CFG_CRITICAL_METHOD
	OS_ERR err; 
	OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);	//UCOSIII延时采用周期模式
#else
	OSTimeDly(ticks);							//UCOSII延时
#endif 
}
 
//systick中断服务函数,使用ucos时用到
void SysTick_Handler(void)
{	
	if(delay_osrunning==1)						//OS开始跑了,才执行正常的调度处理
	{
		OSIntEnter();							//进入中断
		OSTimeTick();       					//调用ucos的时钟服务程序 
		
///////////////////////////////////////////////////////////////		
    cpu0=TIM1->CNT;
	if(led_test==1000) {led_test=0;}
	led_test++;
	
	if(sleepmode==1)			//Sleep Mode. Close all sensors and set PWM to 0
	{
		LPWMH=0; LPWML=0;
		RPWMH=0; RPWML=0;
	}
	else
	{
		cpu0=TIM1->CNT;
//=========================================================================================
//		系统信息采集核心中断函数START
//=========================================================================================
//			车体角度采集部分START
//			左转为负，右转为正。
//=========================================================================================
//				陀螺仪gangle角度采集部分START
//=========================================================================================
		gyro=Get_Adc(ADC_GYRO);
		//Kalman filter	 滞后10ms，不可接受，所以不用卡尔曼 注释掉了 
//		gtmp=g;
//		covgtmp=covg+Q;
//		kg=covgtmp/(covgtmp+P);
//		g=gtmp+kg*((double)gyro-gtmp);//input gyro, output g
//		covg=(1-kg)*covgtmp;	
		g=gyro;
	
		gdiffh=gdiff;
		gdiff=(int)(g-gyro0)/2;
		if(gdiff<0) gdiff--;
		ganglet+=(gdiff+gdiffh)/2;//梯形积分。用加法保证顺时针转角度为正。清零角度只需令ganglet=0;设定角度需要令ganglet=设定角度*gangleratio!!!
		ggangleh=gangle;
		gangle=ganglet/gangleratio;
	
		if(gangle>ggangleh) angleterm=1;
		else if(gangle<ggangleh) angleterm=2;

		//陀螺仪零点矫正用
		if(gyro_adj==1)
		{
			angle_counter++;
			if(gangle>ggangleh)
			{
				gyro0++;
				angle_counter=0;
			}
			else if(gangle<ggangleh)
				{
					gyro0--;
					angle_counter=0;
				}
		}
//=========================================================================================
//				陀螺仪gangle角度采集部分END
//=========================================================================================
//				编码器angle角度采集部分START
//=========================================================================================
		leftbh=leftb;
		rightbh=rightb;
		leftb=TIM2->CNT;
		rightb=TIM3->CNT;
		if((leftb-leftbh)<-9000)loopahead++;		//左轮前进跨界
		if((rightb-rightbh)<-9000)loopahead--;		//右轮前进跨界
		if((leftb-leftbh)>9000)loopahead--;			//左轮倒退跨界
		if((rightb-rightbh)>9000)loopahead++;		//右轮倒退跨界
		absoluteangle=(leftb-rightb+loopahead*10000)/angleratio;	//绝对角度，不停累加
		angle=absoluteangle-baseangle;				//相对角度
	
		//用编码器校正陀螺仪
		
		if(gangleSee == 0)
		{
			
					angleskip++;
		if(angleskip==5)
		{
			angleskip=0;
			anglehis[newindex]=angle;				//数组循环保存最近的50个编码器角度
			newindex++;
			if(newindex==40) newindex=0;
			
			anglehismin=100;anglehismax=-100;
			for(anglei=0;anglei<40;anglei++)
			{
				if(anglehis[anglei]<anglehismin) anglehismin=anglehis[anglei];
				if(anglehis[anglei]>anglehismax) anglehismax=anglehis[anglei];
			}
			if(gyro_adj!=1)							//标定陀螺仪的时候不能清零
			{										//anglehis[i]>20是转弯状态，此时不应当清零陀螺仪
				if(abs(anglehismax-anglehismin)<2 && abs(angle)<20) ganglet=0;	//只要编码器角度超过50个周期基本保持不变，就置零陀螺仪
			}
		}
		
		}
		

//=========================================================================================
//				编码器angle角度采集部分END
//=========================================================================================
//			车体角度采集部分END
//=========================================================================================
//			车体角度峰值处理START
//=========================================================================================
		if((ganglehh-gangleh)>0 && (gangleh-gangle)<0) {peakgangleh=peakgangle; peakgangle=gangle;}
		if((ganglehh-gangleh)<0 && (gangleh-gangle)>0) {peakgangleh=peakgangle; peakgangle=gangle;}
		ganglehh=gangleh;
		gangleh=gangle;
//=========================================================================================
//			车体角度峰值处理END
//=========================================================================================
//			车体前进距离采集部分START
//=========================================================================================	
//		前进距离变量是track，10000约等于11cm。long型，最大值2147483647，约等于2000m
		leftdh=leftd;
		rightdh=rightd;
		leftd=TIM2->CNT;
		rightd=TIM3->CNT;
		if((leftd-leftdh)<-9000) leftcycle++;		//Left Coder Spillover Positively
		if((rightd-rightdh)<-9000) rightcycle++;	//Right Coder Spillover Positively
		if((leftd-leftdh)>9000) leftcycle--;		//Left Coder Spillover Negatively
		if((rightd-rightdh)>9000) rightcycle--;		//Right Coder Spillover Negatively
		absolutetrackl=(float)(leftd+leftcycle*10000)/TrackRatioL;
		absolutetrackr=(float)(rightd+rightcycle*10000)/TrackRatioR;
		absolutetrack=(absolutetrackl+absolutetrackr)/2;
		trackl=absolutetrackl-basetrackl;
		trackr=absolutetrackr-basetrackr;				   
		track=absolutetrack-basetrack;
//=========================================================================================
//			车体前进距离采集部分END
//=========================================================================================
//			传感器距离采集部分START
//=========================================================================================
		switch(irswitch)//传感器开关。转弯时候关闭传感器以提高电机效率
		{
			case 1:
				dislhh=dislh;
				disrhh=disrh;
				dislh=disl;
				disrh=disr;
				disl45hh=disl45h;
				disr45hh=disr45h;			
				disl45h=disl45;
				disr45h=disr45;			
				getdistance();
				disf=(disf1+disf2)/2;
				break;
			case 0:
				infrskipround++;
				break;
		}
		//90度直线有无墙的判别
		lefthaswallh=lefthaswall;
		righthaswallh=righthaswall;
		if(disl<sidewall_threshold) lefthaswall=1;
		else lefthaswall=0;
		if(disr<sidewall_threshold) righthaswall=1;
		else righthaswall=0;
		if((disf1<230&&disf2<245)||(disf2<240&&disf1<245)||(disf1<80)||(disf2<80)) forwardhaswall=1;
		else forwardhaswall=0;

		//45度直线矫正时判定有无墙使用
		lefthaswallh45=lefthaswall45;
		righthaswallh45=righthaswall45;
		if(disl<150) lefthaswall45=1;
		else lefthaswall45=0;
		if(disr<150) righthaswall45=1;
		else righthaswall45=0;

		//45度冲刺时判定转弯点使用
		leftwallcriticalhhh=leftwallcriticalhh;
		leftwallcriticalhh=leftwallcriticalh;
		leftwallcriticalh=leftwallcritical;
		rightwallcriticalhhh=rightwallcriticalhh;
		rightwallcriticalhh=rightwallcriticalh;
		rightwallcriticalh=rightwallcritical;
		if(disl<60) leftwallcritical=1;//原本是disl45<70 已固定，与斜线转弯起始点有关。不要动
		else leftwallcritical=0;
		if(disr<60) rightwallcritical=1;
		else rightwallcritical=0;
//=========================================================================================
//			传感器距离采集部分END
//=========================================================================================
//		系统信息采集核心中断函数END
//=========================================================================================
		
//=========================================================================================
//		系统控制核心中断函数START
//=========================================================================================
	switch(task)
	{
//=========================================================================================
	case NULL:
//=========================================================================================
		break;
	
//=========================================================================================
	case ACC:				//加速
//=========================================================================================
		eh=e;
		e=gangle;
		//位置要求
		setvl=accratio*track+ACCV0;
		setvr=setvl;
		//角度要求
		setvl-=e*2+(e-eh)*1;
		setvr+=e*2+(e-eh)*1;
		break;
	
//=========================================================================================
	case STOP:				//刹车
//=========================================================================================
		//PID累计，每个模块必有
		eh=e;
		e=gangle;	   
		//位置要求
		setvl=-decratio*track;
		setvr=setvl;
		//角度要求
		setvl-=e*3+(e-eh)*1;
		setvr+=e*3+(e-eh)*1;
		if(track<2&&track>-2&&speedl<2&&speedr<2&&speedl>-2&&speedr>-2) tick++;
		else tick=0;
		stoptick++;
		break;
	
//=========================================================================================
	case SLOWDOWN:			//非零速度之间的减速
//=========================================================================================
		//PID累计，每个模块必有
		eh=e;
		e=gangle;
		tick++;
		//if(tick==3)
		{
			setvl-=3;//减速度比加速度稍微大一些，目前基本可以保证在30mm距离内速度由150降到70
			setvr=setvl;
			//tick=0;
			//setvl-=e*1;
			//setvr+=e*1;
		}
		break;
		
//=========================================================================================
	case SPEEDUP:			//非零速度之间的加速
//=========================================================================================
		//PID累计，每个模块必有
		eh=e;
		e=gangle;
		tick++;
		//if(tick==3)
		{
			setvl+=1;
			setvr=setvl;
			//tick=0;
			//setvl-=e*3+(e-eh)*1;
			//setvr+=e*3+(e-eh)*1;
		}
		break;
		
//=========================================================================================
	case TURN:
//=========================================================================================
		setgangle=*(turn_gyro+tick);	 	 
		eh=e;
		e=gangle-setgangle;
		tick++;
		setvl=*(turn_vl+tick)-e*turn_kp-(e-eh)*turn_kd;//8,5	12,15  -20
		setvr=*(turn_vr+tick)+e*turn_kp+(e-eh)*turn_kd;            //-20
		break;
	
//=========================================================================================
	case TURNAROUND:		//原地转180度
//=========================================================================================
		if(step==0)
		{
			tick++;	 		
			setgangle=*(turn_gyro+tick);	 	 
			eh=e;
			e=gangle-setgangle;
//       e=0;/////////////////////////////////////////////////////////////////////			
			esum+=e;		
			setvl=*(turn_vl+tick)-e*8-esum*0.1-(e-eh)*5;
			setvr=*(turn_vr+tick)+e*8+esum*0.1+(e-eh)*5;
			if(tick>265){step=1;tick=0;}               //tick判断数值180度回转数组长度有关
			turnaround_tick = 0;
		}
		else if(step==1)
		{
			eh=e;
			e=gangle;
			setvl=-e*4-(e-eh)*1;	 
			setvr=e*4+(e-eh)*1;
			if(speedl<2&&speedr<2&&speedl>-2&&speedr>-2)tick++;//2
			else tick=0;
			turnaround_tick++;
			if(turnaround_tick>100)tick=100;//防止抖动卡死
		}
		break;
		
//=========================================================================================
	case POINTTURN90:		//原地转90度
//=========================================================================================
		if(step==0)
		{
			tick++;	 		
			setgangle=*(turn_gyro+tick);	 	 
			eh=e;
			e=gangle-setgangle;	   
			esum+=e;		
			setvl=*(turn_vl+tick)-e*8-esum*0.1-(e-eh)*5;
			setvr=*(turn_vr+tick)+e*8+esum*0.1+(e-eh)*5;
			if(tick>230){step=1;tick=0;}
			turnaround_tick = 0;
		}
		else if(step==1)
		{	 	 
			e=gangle;
			setvl=e*(-1);	 
			setvr=e*1;
			if(speedl<15&&speedr<15&&speedl>-15&&speedr>-15)tick++;//2
			else tick=0;
			turnaround_tick++;
			if(turnaround_tick>100)tick=100;//防止抖动卡死
		}
		break;
		
//=========================================================================================
	case BEFORE90TURN:
//=========================================================================================
		if(forwardhaswall==1)//转弯前若前方有墙优先使用前方。
		{			 
			e=disf2-disf1;
			setvl=speed_turn-e*0.1;
			setvr=speed_turn+e*0.1;
		}
		else 
		{
			e=gangle;
			setvl=speed_turn-e*1;
			setvr=speed_turn+e*1;
		}
		//若已经开始转弯又出现了有墙无墙变化，说明上一个格子错过了矫正，那么在此重新矫正
		{
//			if(lefthaswallh==1&&lefthaswall==0){settrack(trackcoor-180);}
//			if(righthaswallh==1&&righthaswall==0){settrack(trackcoor-180);}
		}
		break;
		
//=========================================================================================
	case BEFORE180TURN:
//=========================================================================================
		if(righthaswall==1 && lefthaswall==0 && disf>60)//80
		{
			setvl=speed_turn+(disr-dis2wall);
			setvr=speed_turn-(disr-dis2wall);	
		}
		else if(righthaswall==0 && lefthaswall==1 && disf>60)
		{
			setvl=speed_turn-(disl-dis2wall);
			setvr=speed_turn+(disl-dis2wall);	
		}
		else if(righthaswall==1 && lefthaswall==1 && disf>60)
		{
			setvl=speed_turn+(disr-disl);
			setvr=speed_turn-(disr-disl);	
		}
		else
		{
			setvl=(disf-30)*0.5+(disf1-30)*1;			//调整到距离30mm
			setvr=(disf-30)*0.5+(disf2-30)*1;
			if(setvl>speed_turn) setvl=speed_turn;
			if(setvl<-speed_turn) setvl=-speed_turn;
			if(setvr>speed_turn) setvr=speed_turn;
			if(setvr<-speed_turn) setvr=-speed_turn;
			if( (disf-30)<-20 && abs(disf1-disf2)<100 )	//太远且角度偏差不大，直走
			{
				setvl=-speed_turn;
				setvr=-speed_turn;
			}
			if( (disf-30)>20 && abs(disf1-disf2)<100 )
			{
				setvl=speed_turn;
				setvr=speed_turn;
			}
			if(speedl<2&&speedr<2&&speedl>-2&&speedr>-2)tick++;
			else tick=0;
			turnaround_tick++;
			if(turnaround_tick>100)tick=100;//防止抖动卡死
		}
		break;
		
//=========================================================================================
	case FORWARDIRTEST:		//红外测试 向前对正
//=========================================================================================
		setvl=(disf-forwardirdis)*0.5+(disf1-forwardirdis)*1;	//调整到forwardirdis mm
		setvr=(disf-forwardirdis)*0.5+(disf2-forwardirdis)*1;
		if(setvl>50) setvl=50;
		if(setvl<-50) setvl=-50;
		if(setvr>50) setvr=50;
		if(setvr<-50) setvr=-50;
		if(setvl<2 && setvl>-2) setvl=0;
		if(setvr<2 && setvr>-2) setvr=0;
		if( (disf-forwardirdis)<-20 && abs(disf1-disf2)<100 )	//太远且角度偏差不大，直走
		{
			setvl=-50;
			setvr=-50;
		}
		if( (disf-forwardirdis)>20 && abs(disf1-disf2)<100 )
		{
			setvl=50;
			setvr=50;
		}
		break;
		
//=========================================================================================
	case KEEPWALLDISTANCE:
//=========================================================================================
		setvl=(disf-17)*0.5+(disf1-17)*1;				//调整到17 mm
		setvr=(disf-17)*0.5+(disf2-17)*1;
		if(setvl>50) setvl=50;
		if(setvl<-50) setvl=-50;
		if(setvr>50) setvr=50;
		if(setvr<-50) setvr=-50;
		if( (disf-17)<-20 && abs(disf1-disf2)<100 )
		{
			setvl=-50;
			setvr=-50;
		}
		if( (disf-17)>20 && abs(disf1-disf2)<100 )
		{
			setvl=50;
			setvr=50;
		}
		if(speedl<2&&speedr<2&&speedl>-2&&speedr>-2)tick++;
		else tick=0;
		turnaround_tick++;
		if(turnaround_tick>500)tick=100;				//防止抖动卡死
		break;
		
//===========================================================================================================================================
//===========================================================================================================================================
//=============================================================搜索直线======================================================================
//===========================================================================================================================================
//===========================================================================================================================================
		
		
//=========================================================================================
	case GOSTRAIGHT:			//闭眼走直线	  
//=========================================================================================
		eh=e;
		e=gangle;
		setvl=speed_ss-e*GOSTRAIGHT_kp;//0.4;		
		setvr=speed_ss+e*GOSTRAIGHT_kp;//0.4;
		break;
	
//=========================================================================================
	case SMARTSTRAIGHT:			//转弯后补全，180度回转之前补全
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//闭眼走直线
		{
			eh=e;
			e=gangle;
			setvl=speed_turn-e*1;		
			setvr=speed_turn+e*1;
		}
		else if(lefthaswall==1)
		{
			eh=e;
			e=disl-dis2wall;
			setvl=speed_turn-e*0.1;		
			setvr=speed_turn+e*0.1;
		}
		else if(righthaswall==1)
		{
			eh=e;
			e=dis2wall-disr;
			setvl=speed_turn-e*0.1;		
			setvr=speed_turn+e*0.1;
		}
		break;
		
//=========================================================================================
	case SMARTSTRAIGHTFAST:		//搜索直线每格前半段，根据左右是否有墙采用睁眼或闭眼跑
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//闭眼走直线
		{
			eh=e;
			e=gangle;
			setvl=speed_ss-e*1;		
			setvr=speed_ss+e*1;
		}
		else
		{
			if((lastside==LEFT && lefthaswall==1) || (lastside==RIGHT && righthaswall==0))//懒惰模式：上一次用的哪边墙，这次只要那边有墙就还用那边
			{
				lastside=LEFT;
				eh=e;				  
				e=disl-dis2wall;
				if(abs(e)>10)
				{
					setvl=speed_ss-e*SMARTSTRAIGHTFAST_bigerror_kp-(e-eh)*1;//0.2;		
					setvr=speed_ss+e*SMARTSTRAIGHTFAST_bigerror_kp+(e-eh)*1;//0.2;
				}
				else
				{
					setvl=speed_ss-e*SMARTSTRAIGHTFAST_smallerror_kp-(e-eh)*1;//0.15;		
					setvr=speed_ss+e*SMARTSTRAIGHTFAST_smallerror_kp+(e-eh)*1;//0.15;
				}
			}
			else
			{
				lastside=RIGHT;			  
				eh=e;
				e=dis2wall-disr;
				if(abs(e)>10)
				{
					setvl=speed_ss-e*SMARTSTRAIGHTFAST_bigerror_kp-(e-eh)*1;//0.2;		
					setvr=speed_ss+e*SMARTSTRAIGHTFAST_bigerror_kp+(e-eh)*1;//0.2;
				}
				else
				{
					setvl=speed_ss-e*SMARTSTRAIGHTFAST_smallerror_kp-(e-eh)*1;//0.15;		
					setvr=speed_ss+e*SMARTSTRAIGHTFAST_smallerror_kp+(e-eh)*1;//0.15;
				}
			}		
		}
		//setangle(0);
		break;
		
//=========================================================================================
	case STRAIGHTEYECLOSECOORADJ:	//搜索，有墙格子后半段，若遇到有墙无墙则纵向矫正
//=========================================================================================
		eh=e;
		e=gangle;
		setvl=speed_ss-e*0.3;		
		setvr=speed_ss+e*0.3;
		#ifdef COORADJUST
		//if(blocknum>=3)//只有连续走3格及以上直线时才开启矫正，以免车摇晃导致矫正更糟
		//纵向矫正有一个改进方法，若左右两侧都有有墙无墙变化，则取二者平均值来矫正
		{												
			if(lefthaswallh==1&&lefthaswall==0){settrack(trackcoor);}
			if(righthaswallh==1&&righthaswall==0){settrack(trackcoor);}
		}
		#endif
		break;
		
//=========================================================================================
	case STRAIGHTPOLE:			//搜索，无墙格子后半段。用柱子校正姿态，纵向矫正
//两侧传感器到达峰值的时间差代表车的角度偏差；到达峰值时距离值代表横向误差。
//因为该任务必定在两侧无墙-有墙期间调用，所以:
//当测距小于某数值时激活峰值检测；
//当前后两次测量距离差首次小于某数值时认为到达峰值点，此时记录下tick和峰值数值。同时关闭本侧传感器的峰值检测。
//当两侧的峰值都已经得到时，利用tick之差和峰值之差进行一次矫正。
//=================================
//后续可以考虑识别排骨情况用两侧横线挡板进行较短距离的矫正！！！
//=========================================================================================
// 		eh=e;
		e=gangle;
		setvl=speed_ss-e*0.5;		
		setvr=speed_ss+e*0.5;

		#ifdef COORADJUST
		//if(blocknum>=3)//只有连续走3格及以上直线时才开启矫正，以免车摇晃导致矫正更糟
		{
			if(lefthaswallh==1 && lefthaswall==0) settrack(trackcoor);
			if(righthaswallh==1 && righthaswall==0) settrack(trackcoor);
		}
		#endif
		break;	

//=======================================================================================================================================================================================
//=======================================================================================================================================================================================
//=======================================================================================冲刺相关========================================================================================
//=======================================================================================================================================================================================
//=======================================================================================================================================================================================
//用到了speed_ss, speed_fastss, 
//=========================================================================================
	case SMARTSTRAIGHTSPURT://转弯前补全
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//闭眼走直线
		{
			eh=e;
			e=gangle;
			setvl=speed_ss-e*1;		
			setvr=speed_ss+e*1;
		}
		else if(lefthaswall==1)
		{
			eh=e;
			e=disl-dis2wall;
			setvl=speed_ss-e*0.03;		
			setvr=speed_ss+e*0.03;
		}
		else if(righthaswall==1)
		{
			eh=e;
			e=dis2wall-disr;
			setvl=speed_ss-e*0.03;		
			setvr=speed_ss+e*0.03;
		}
		break;
//=========================================================================================
	case SMARTSTRAIGHTSPURTL://转弯前补全
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//闭眼走直线
		{
			eh=e;
			e=gangle;
			setvl=speed_ss-e*1;		
			setvr=speed_ss+e*1;
		}
		else if(lefthaswall==1)
		{
			eh=e;
			e=disl-dis2wall;
			setvl=speed_ss-e*0.01;		
			setvr=speed_ss+e*0.01;
		}
		else if(righthaswall==1)
		{
			eh=e;
			e=dis2wall-disr;
			setvl=speed_ss-e*0.01;		
			setvr=speed_ss+e*0.01;
		}
		break;
		
//=========================================================================================
	case SPURT90IR:	//冲刺时睁眼走90度直线
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//闭眼走直线
		{
			eh=e;
			e=gangle;
			setvl=speed_ss-e*1;		
			setvr=speed_ss+e*1;
		}
		else
		{
			if((lastside==LEFT && lefthaswall==1) || (lastside==RIGHT && righthaswall==0))//懒惰模式：上一次用的哪边墙，这次只要那边有墙就还用那边
			{
				lastside=LEFT;
				eh=e;				  
				e=disl-dis2wall;
				if(abs(e)>10)
				{
					setvl=speed_ss-e*SPURT90IR_kp1;//0.3;	
					setvr=speed_ss+e*SPURT90IR_kp1;
				}
				else
				{
					setvl=speed_ss-e*SPURT90IR_kp2-(e-eh)*SPURT90IR_kd;
					setvr=speed_ss+e*SPURT90IR_kp2+(e-eh)*SPURT90IR_kd;
				}
			}
			else
			{			lastside=RIGHT;			  
				eh=e;
				e=dis2wall-disr;
				if(abs(e)>10)
				{
					setvl=speed_ss-e*SPURT90IR_kp1;
					setvr=speed_ss+e*SPURT90IR_kp1;
				}
				else
				{
					setvl=speed_ss-e*SPURT90IR_kp2-(e-eh)*SPURT90IR_kd;
					setvr=speed_ss+e*SPURT90IR_kp2+(e-eh)*SPURT90IR_kd;
				}

			}		
		}
		break;
		
//=========================================================================================
	case SPURT90BLIND://冲刺时闭眼走90度直线
//=========================================================================================
		eh=e;
		e=gangle;
		setvl=speed_ss-e*SPURT90GYRO_kp-(e-eh)*SPURT90GYRO_kd;
		setvr=speed_ss+e*SPURT90GYRO_kp+(e-eh)*SPURT90GYRO_kd;
		
	//	#ifdef COORADJUST								
	//		if(lefthaswallh==1&&lefthaswall==0){settrack(trackcoor-90);}
	//		if(righthaswallh==1&&righthaswall==0){settrack(trackcoor-90);}
	//	#endif
		break;
	
//=========================================================================================
	case STRAIGHT45://冲刺45度直线
//睁眼走指定距离的斜线以及纵向矫正和角度正方向标定
//未来可以在两项误差上乘以系数
//=========================================================================================
		if(disl<150)//左侧有墙
		{
			e=abovezero(disl+50-disf1)+abovezero(180-disf1);//e越大，说明车越左偏。两项和，第一项代表角度偏差，第二项代表横向坐标偏差
			setvl=speed_fastss+e*STRAIGHT45_kp+(e-eh)*STRAIGHT45_kd;
			setvr=speed_fastss-e*STRAIGHT45_kp-(e-eh)*STRAIGHT45_kd;
		}
		else if(disr<150)//右侧有墙 
		{
			e=abovezero(disr+50-disf2)+abovezero(180-disf2);//e越大，说明车越右偏
			setvl=speed_fastss-e*STRAIGHT45_kp-(e-eh)*STRAIGHT45_kd;
			setvr=speed_fastss+e*STRAIGHT45_kp+(e-eh)*STRAIGHT45_kd;
		}
		if(disr>150 && disl>150)//两侧都没有墙
		{
			e=/*angle+*/abovezero(220-disf2)-abovezero(220-disf1);
			setvl=speed_fastss-e*STRAIGHT45_kp;
			setvr=speed_fastss+e*STRAIGHT45_kp;
		}
		break;

	default:
		ledon(0);
	break;
	}//系统控制核心中断函数END
//=======================================================================================================================================================================================
//=======================================================================================================================================================================================
//=======================================================================================================================================================================================
//=======================================================================================================================================================================================
//=======================================================================================================================================================================================
//=======================================================================================================================================================================================

	if(isemergency==1)
	{
		freemode=1;
	//	flashwrite=0;
		ledon(0);
		task=NULL;
		setvl=0;
		setvr=0;
		emergency_delay++;
		if(emergency_delay==200) sleepmode=1;
	//	Led_Time=300;
	}
//=========================================================================================
//电机PID控制核心中断函数START
	//Left Motor
	gdvl=setvl*SpeedRatioL;						//cm/s -> Pulse/ms 系数越大走得越快
	lw=TIM2->CNT;		  
	gdspeedl=lw-lwh;									  
	if(gdspeedl<-5000) gdspeedl+=10000;			//Positive Spillover
	if(gdspeedl>5000) gdspeedl-=10000;			//Negative Spillover
	lwh=lw;
	errlh=errl;
	errl=(int)gdvl-gdspeedl;
	errls+=errl;
	if(errls>1200) errls=1200;
	if(errls<-1200) errls=-1200;				//Anti-windup
	
	//Right Motor
	gdvr=setvr*SpeedRatioR;						//cm/s -> Pulse/ms
	rw=TIM3->CNT;		  
	gdspeedr=rw-rwh;
	if(gdspeedr<-5000) gdspeedr+=10000;
	if(gdspeedr>5000) gdspeedr-=10000;
	rwh=rw;
	errrh=errr;
	errr=(int)gdvr-gdspeedr;
	errrs+=errr;
	if(errrs>1200) errrs=1200;
	if(errrs<-1200) errrs=-1200;				//Anti-windup

	speedl=gdspeedl/SpeedRatioL;
	speedr=gdspeedr/SpeedRatioR;
	speed=(speedl+speedr)/2;
	
//	if(errl > speedErrMaxL || errl < -1*speedErrMaxL) speedErrMaxL = abs(errl);
//	if(errr > speedErrMaxR || errr < -1*speedErrMaxR) speedErrMaxR = abs(errr);

	
	if(isSpeedTest == 1)
	{
		if(speedListi > 5000)
		{
			speedListi = 5000;
		}
		
		if(speedListi % 100 == 0)
		{
			speedListL[speedListi/100] = gdspeedl;
			speedListR[speedListi/100] = gdspeedr;
		}	
		speedListi++;
	}

	

	if(freemode==0)
	{//30-1-2
		kpl=9.25;kil=0.6;kdl=0.44;
		pwml=errl*kpl+errls*kil+(errl-errlh)*kdl;
		kpr=9.25;kir=0.5;kdr=0.36;
		pwmr=errr*kpr+errrs*kir+(errr-errrh)*kdr;
	}
	else
	{
		pwml=0;pwmr=0;
	}									   

	if(pwml>SPEED_HAHA) pwml=SPEED_HAHA;
	if(pwml<-SPEED_HAHA) pwml=-SPEED_HAHA;
	if(pwml>=35)//前进
	{
		LPWML=0;LPWMH=pwml;
	}
	else if(pwml<=-35)//后退
	{
		LPWML=-pwml;LPWMH=0;
	}
	else
	{
		LPWML = 0;
		LPWMH = 0;
	}
	
	
	if(pwmr>SPEED_HAHA) pwmr=SPEED_HAHA;
	if(pwmr<-SPEED_HAHA) pwmr=-SPEED_HAHA;
	if(pwmr>=35)
	{
		RPWML=0;RPWMH=pwmr;
	}
	else if(pwmr<=-35)
	{
		RPWML=-pwmr;RPWMH=0;
	}
	else 
	{
		RPWML = 0;
		RPWMH = 0;
	}
//电机PID控制核心中断函数END
//=========================================================================================
//=========================================================================================
//故障判断																		
	if(EMERGENCYON==1)
	{
		//if(pwml>800 || pwmr>800) emergency_count++;
		if(pwml>SPEED_HAHA || pwmr>SPEED_HAHA || pwml<-SPEED_HAHA || pwmr<-SPEED_HAHA) emergency_count++;
		else emergency_count=0;
		if(emergency_count>200 || setvl>2000 || setvr>2000 || setvl<-2000 || setvr<-2000) isemergency=1;//200
	}
//=========================================================================================
//=========================================================================================
//电池电压检测START	
	if(showbattery==1)
	{
		battaryh=battary;
		battary=Get_Adc(ADC_BATTARY)*3.3*3/4096*100;//最后乘100转换为整数。设置8V-7V为电源范围
		battary=(battary*1+battaryh*9)/10;
		battarypercent=(battary-700)/12;
		if(battarypercent==0) battarypercent=1;
		if(battarypercent>8) battarypercent=8;
		#ifdef SHOWVOLTAGE
		ledoff(0);
		ledon(battarypercent);
		#endif
	}
//电池电压检测END
//=========================================================================================
//=========================================================================================
//FLASH缓冲存储，每个周期只存一个long型数
	#ifdef FLASHSAVE
// 	if(queuenum!=0)
// 	{
// 		STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_UART+flashaddr,queue[0]);flashaddr+=2;
// 		queuenum--;
// 		for(queuei=0;queuei<queuenum;queuei++)
// 		{
// 			queue[queuei]=queue[queuei+1];
// 		}
// 	}
//FLASH连续存储
	if(flashenable==1)						
	{
		if(flashwrite==1)//存一个55us
		{
			save_data_temp++;
			if(save_data_temp>(per_timer-1))
			{write_data(save_data_num);save_data_num++;save_data_temp=0;
			if(save_data_num>=save_num-1) {save_data_num=0;save_data_time++;}
			}
		}
	}	
	#endif
//=========================================================================================
		//printf("%d ",setvl);		 
//=========================================================================================
	#ifdef SHOWCPU
	cpu=TIM1->CNT-cpu0;
	if(cpu<0) cpu=cpu+10000;
	cpu=cpu/10;			//转换成百分制
	if(flashenable==1)						
	{
		if(cpu>95)		//时钟周期占用率高于95%
		{
			ledon(1);ledon(7);
		}
		else ledoff(0);
	}
	#endif
//=========================================================================================
	if(isemergency==1) ledon(0);
}

/////////////////////////////////////////////////////////////////	

		OSIntExit();       	 					//触发任务切换软中断
	}
}
#endif

			   
//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()
{
#if SYSTEM_SUPPORT_OS  							//如果需要支持OS.
	u32 reload;
#endif
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/8000000;				//为系统时钟的1/8  
#if SYSTEM_SUPPORT_OS  							//如果需要支持OS.
	reload=SystemCoreClock/8000000;				//每秒钟的计数次数 单位为K	   
	reload*=1000000/delay_ostickspersec;		//根据delay_ostickspersec设定溢出时间
												//reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右	
	fac_ms=1000/delay_ostickspersec;			//代表OS可以延时的最少单位	   

	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
	SysTick->LOAD=reload; 						//每1/delay_ostickspersec秒中断一次	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK    

#else
	fac_ms=(u16)fac_us*1000;					//非OS下,代表每个ms需要的systick时钟数   
#endif
}								    

#if SYSTEM_SUPPORT_OS  							//如果需要支持OS.
//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;					//LOAD的值	    	 
	ticks=nus*fac_us; 							//需要的节拍数	  		 
	tcnt=0;
	delay_osschedlock();						//阻止OS调度，防止打断us延时
	told=SysTick->VAL;        					//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;		//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;				//时间超过/等于要延迟的时间,则退出.
		}  
	};
	delay_osschedunlock();						//恢复OS调度									    
}
//延时nms
//nms:要延时的ms数
void delay_ms(u16 nms)
{	
	if(delay_osrunning&&delay_osintnesting==0)	//如果OS已经在跑了,并且不是在中断里面(中断里面不能任务调度)	    
	{		 
		if(nms>=fac_ms)							//延时的时间大于OS的最少时间周期 
		{ 
   			delay_ostimedly(nms/fac_ms);		//OS延时
		}
		nms%=fac_ms;							//OS已经无法提供这么小的延时了,采用普通方式延时    
	}
	delay_us((u32)(nms*1000));					//普通方式延时  
}
#else //不用OS时
//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//时间加载	  		 
	SysTick->VAL=0x00;        					//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					 //清空计数器	 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       					//清空计数器	  	    
} 
#endif 





































