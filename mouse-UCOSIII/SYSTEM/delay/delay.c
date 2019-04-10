#include <stm32f10x.h>
#include "delay.h"
//#include "stmflash.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//ʹ��SysTick����ͨ����ģʽ���ӳٽ��й����ʺ�STM32F10xϵ�У�
//����delay_us,delay_ms
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2010/1/1
//�汾��V1.8
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.2�޸�˵��
//�������ж��е��ó�����ѭ���Ĵ���
//��ֹ��ʱ��׼ȷ,����do while�ṹ!
//V1.3�޸�˵��
//�����˶�UCOSII��ʱ��֧��.
//���ʹ��ucosII,delay_init���Զ�����SYSTICK��ֵ,ʹ֮��ucos��TICKS_PER_SEC��Ӧ.
//delay_ms��delay_usҲ���������ucos�ĸ���.
//delay_us������ucos��ʹ��,����׼ȷ�Ⱥܸ�,����Ҫ����û��ռ�ö���Ķ�ʱ��.
//delay_ms��ucos��,���Ե���OSTimeDly����,��δ����ucosʱ,������delay_usʵ��,�Ӷ�׼ȷ��ʱ
//����������ʼ������,��������ucos֮��delay_ms������ʱ�ĳ���,ѡ��OSTimeDlyʵ�ֻ���delay_usʵ��.
//V1.4�޸�˵�� 20110929
//�޸���ʹ��ucos,����ucosδ������ʱ��,delay_ms���ж��޷���Ӧ��bug.
//V1.5�޸�˵�� 20120902
//��delay_us����ucos��������ֹ����ucos���delay_us��ִ�У����ܵ��µ���ʱ��׼��
//V1.6�޸�˵�� 20150109
//��delay_ms����OSLockNesting�жϡ�
//V1.7�޸�˵�� 20150319
//�޸�OS֧�ַ�ʽ,��֧������OS(������UCOSII��UCOSIII,����������OS������֧��)
//���:delay_osrunning/delay_ostickspersec/delay_osintnesting�����궨��
//���:delay_osschedlock/delay_osschedunlock/delay_ostimedly��������
//V1.8�޸�˵�� 20150519
//����UCOSIII֧��ʱ��2��bug��
//delay_tickspersec��Ϊ��delay_ostickspersec
//delay_intnesting��Ϊ��delay_osintnesting
//////////////////////////////////////////////////////////////////////////////////  
extern u8 task;
static u8  fac_us=0;							//us��ʱ������			   
static u16 fac_ms=0;							//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��
	
	
#if SYSTEM_SUPPORT_OS							//���SYSTEM_SUPPORT_OS������,˵��Ҫ֧��OS��(������UCOS).
//��delay_us/delay_ms��Ҫ֧��OS��ʱ����Ҫ������OS��صĺ궨��ͺ�����֧��
//������3���궨��:
//    delay_osrunning:���ڱ�ʾOS��ǰ�Ƿ���������,�Ծ����Ƿ����ʹ����غ���
//delay_ostickspersec:���ڱ�ʾOS�趨��ʱ�ӽ���,delay_init�����������������ʼ��systick
// delay_osintnesting:���ڱ�ʾOS�ж�Ƕ�׼���,��Ϊ�ж����治���Ե���,delay_msʹ�øò����������������
//Ȼ����3������:
//  delay_osschedlock:��������OS�������,��ֹ����
//delay_osschedunlock:���ڽ���OS�������,���¿�������
//    delay_ostimedly:����OS��ʱ,���������������.

//�����̽���UCOSII��UCOSIII��֧��,����OS,�����вο�����ֲ
//֧��UCOSII
#ifdef 	OS_CRITICAL_METHOD						//OS_CRITICAL_METHOD������,˵��Ҫ֧��UCOSII				
#define delay_osrunning		OSRunning			//OS�Ƿ����б��,0,������;1,������
#define delay_ostickspersec	OS_TICKS_PER_SEC	//OSʱ�ӽ���,��ÿ����ȴ���
#define delay_osintnesting 	OSIntNesting		//�ж�Ƕ�׼���,���ж�Ƕ�״���
#endif

//֧��UCOSIII
#ifdef 	CPU_CFG_CRITICAL_METHOD					//CPU_CFG_CRITICAL_METHOD������,˵��Ҫ֧��UCOSIII	
#define delay_osrunning		OSRunning			//OS�Ƿ����б��,0,������;1,������
#define delay_ostickspersec	OSCfg_TickRate_Hz	//OSʱ�ӽ���,��ÿ����ȴ���
#define delay_osintnesting 	OSIntNestingCtr		//�ж�Ƕ�׼���,���ж�Ƕ�״���
#endif


//us����ʱʱ,�ر��������(��ֹ���us���ӳ�)
void delay_osschedlock(void)
{
#ifdef CPU_CFG_CRITICAL_METHOD   				//ʹ��UCOSIII
	OS_ERR err; 
	OSSchedLock(&err);							//UCOSIII�ķ�ʽ,��ֹ���ȣ���ֹ���us��ʱ
#else											//����UCOSII
	OSSchedLock();								//UCOSII�ķ�ʽ,��ֹ���ȣ���ֹ���us��ʱ
#endif
}

//us����ʱʱ,�ָ��������
void delay_osschedunlock(void)
{	
#ifdef CPU_CFG_CRITICAL_METHOD   				//ʹ��UCOSIII
	OS_ERR err; 
	OSSchedUnlock(&err);						//UCOSIII�ķ�ʽ,�ָ�����
#else											//����UCOSII
	OSSchedUnlock();							//UCOSII�ķ�ʽ,�ָ�����
#endif
}

//����OS�Դ�����ʱ������ʱ
//ticks:��ʱ�Ľ�����
void delay_ostimedly(u32 ticks)
{
#ifdef CPU_CFG_CRITICAL_METHOD
	OS_ERR err; 
	OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);	//UCOSIII��ʱ��������ģʽ
#else
	OSTimeDly(ticks);							//UCOSII��ʱ
#endif 
}
 
//systick�жϷ�����,ʹ��ucosʱ�õ�
void SysTick_Handler(void)
{	
	if(delay_osrunning==1)						//OS��ʼ����,��ִ�������ĵ��ȴ���
	{
		OSIntEnter();							//�����ж�
		OSTimeTick();       					//����ucos��ʱ�ӷ������ 
		
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
//		ϵͳ��Ϣ�ɼ������жϺ���START
//=========================================================================================
//			����ǶȲɼ�����START
//			��תΪ������תΪ����
//=========================================================================================
//				������gangle�ǶȲɼ�����START
//=========================================================================================
		gyro=Get_Adc(ADC_GYRO);
		//Kalman filter	 �ͺ�10ms�����ɽ��ܣ����Բ��ÿ����� ע�͵��� 
//		gtmp=g;
//		covgtmp=covg+Q;
//		kg=covgtmp/(covgtmp+P);
//		g=gtmp+kg*((double)gyro-gtmp);//input gyro, output g
//		covg=(1-kg)*covgtmp;	
		g=gyro;
	
		gdiffh=gdiff;
		gdiff=(int)(g-gyro0)/2;
		if(gdiff<0) gdiff--;
		ganglet+=(gdiff+gdiffh)/2;//���λ��֡��üӷ���֤˳ʱ��ת�Ƕ�Ϊ��������Ƕ�ֻ����ganglet=0;�趨�Ƕ���Ҫ��ganglet=�趨�Ƕ�*gangleratio!!!
		ggangleh=gangle;
		gangle=ganglet/gangleratio;
	
		if(gangle>ggangleh) angleterm=1;
		else if(gangle<ggangleh) angleterm=2;

		//��������������
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
//				������gangle�ǶȲɼ�����END
//=========================================================================================
//				������angle�ǶȲɼ�����START
//=========================================================================================
		leftbh=leftb;
		rightbh=rightb;
		leftb=TIM2->CNT;
		rightb=TIM3->CNT;
		if((leftb-leftbh)<-9000)loopahead++;		//����ǰ�����
		if((rightb-rightbh)<-9000)loopahead--;		//����ǰ�����
		if((leftb-leftbh)>9000)loopahead--;			//���ֵ��˿��
		if((rightb-rightbh)>9000)loopahead++;		//���ֵ��˿��
		absoluteangle=(leftb-rightb+loopahead*10000)/angleratio;	//���ԽǶȣ���ͣ�ۼ�
		angle=absoluteangle-baseangle;				//��ԽǶ�
	
		//�ñ�����У��������
		
		if(gangleSee == 0)
		{
			
					angleskip++;
		if(angleskip==5)
		{
			angleskip=0;
			anglehis[newindex]=angle;				//����ѭ�����������50���������Ƕ�
			newindex++;
			if(newindex==40) newindex=0;
			
			anglehismin=100;anglehismax=-100;
			for(anglei=0;anglei<40;anglei++)
			{
				if(anglehis[anglei]<anglehismin) anglehismin=anglehis[anglei];
				if(anglehis[anglei]>anglehismax) anglehismax=anglehis[anglei];
			}
			if(gyro_adj!=1)							//�궨�����ǵ�ʱ��������
			{										//anglehis[i]>20��ת��״̬����ʱ��Ӧ������������
				if(abs(anglehismax-anglehismin)<2 && abs(angle)<20) ganglet=0;	//ֻҪ�������Ƕȳ���50�����ڻ������ֲ��䣬������������
			}
		}
		
		}
		

//=========================================================================================
//				������angle�ǶȲɼ�����END
//=========================================================================================
//			����ǶȲɼ�����END
//=========================================================================================
//			����Ƕȷ�ֵ����START
//=========================================================================================
		if((ganglehh-gangleh)>0 && (gangleh-gangle)<0) {peakgangleh=peakgangle; peakgangle=gangle;}
		if((ganglehh-gangleh)<0 && (gangleh-gangle)>0) {peakgangleh=peakgangle; peakgangle=gangle;}
		ganglehh=gangleh;
		gangleh=gangle;
//=========================================================================================
//			����Ƕȷ�ֵ����END
//=========================================================================================
//			����ǰ������ɼ�����START
//=========================================================================================	
//		ǰ�����������track��10000Լ����11cm��long�ͣ����ֵ2147483647��Լ����2000m
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
//			����ǰ������ɼ�����END
//=========================================================================================
//			����������ɼ�����START
//=========================================================================================
		switch(irswitch)//���������ء�ת��ʱ��رմ���������ߵ��Ч��
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
		//90��ֱ������ǽ���б�
		lefthaswallh=lefthaswall;
		righthaswallh=righthaswall;
		if(disl<sidewall_threshold) lefthaswall=1;
		else lefthaswall=0;
		if(disr<sidewall_threshold) righthaswall=1;
		else righthaswall=0;
		if((disf1<230&&disf2<245)||(disf2<240&&disf1<245)||(disf1<80)||(disf2<80)) forwardhaswall=1;
		else forwardhaswall=0;

		//45��ֱ�߽���ʱ�ж�����ǽʹ��
		lefthaswallh45=lefthaswall45;
		righthaswallh45=righthaswall45;
		if(disl<150) lefthaswall45=1;
		else lefthaswall45=0;
		if(disr<150) righthaswall45=1;
		else righthaswall45=0;

		//45�ȳ��ʱ�ж�ת���ʹ��
		leftwallcriticalhhh=leftwallcriticalhh;
		leftwallcriticalhh=leftwallcriticalh;
		leftwallcriticalh=leftwallcritical;
		rightwallcriticalhhh=rightwallcriticalhh;
		rightwallcriticalhh=rightwallcriticalh;
		rightwallcriticalh=rightwallcritical;
		if(disl<60) leftwallcritical=1;//ԭ����disl45<70 �ѹ̶�����б��ת����ʼ���йء���Ҫ��
		else leftwallcritical=0;
		if(disr<60) rightwallcritical=1;
		else rightwallcritical=0;
//=========================================================================================
//			����������ɼ�����END
//=========================================================================================
//		ϵͳ��Ϣ�ɼ������жϺ���END
//=========================================================================================
		
//=========================================================================================
//		ϵͳ���ƺ����жϺ���START
//=========================================================================================
	switch(task)
	{
//=========================================================================================
	case NULL:
//=========================================================================================
		break;
	
//=========================================================================================
	case ACC:				//����
//=========================================================================================
		eh=e;
		e=gangle;
		//λ��Ҫ��
		setvl=accratio*track+ACCV0;
		setvr=setvl;
		//�Ƕ�Ҫ��
		setvl-=e*2+(e-eh)*1;
		setvr+=e*2+(e-eh)*1;
		break;
	
//=========================================================================================
	case STOP:				//ɲ��
//=========================================================================================
		//PID�ۼƣ�ÿ��ģ�����
		eh=e;
		e=gangle;	   
		//λ��Ҫ��
		setvl=-decratio*track;
		setvr=setvl;
		//�Ƕ�Ҫ��
		setvl-=e*3+(e-eh)*1;
		setvr+=e*3+(e-eh)*1;
		if(track<2&&track>-2&&speedl<2&&speedr<2&&speedl>-2&&speedr>-2) tick++;
		else tick=0;
		stoptick++;
		break;
	
//=========================================================================================
	case SLOWDOWN:			//�����ٶ�֮��ļ���
//=========================================================================================
		//PID�ۼƣ�ÿ��ģ�����
		eh=e;
		e=gangle;
		tick++;
		//if(tick==3)
		{
			setvl-=3;//���ٶȱȼ��ٶ���΢��һЩ��Ŀǰ�������Ա�֤��30mm�������ٶ���150����70
			setvr=setvl;
			//tick=0;
			//setvl-=e*1;
			//setvr+=e*1;
		}
		break;
		
//=========================================================================================
	case SPEEDUP:			//�����ٶ�֮��ļ���
//=========================================================================================
		//PID�ۼƣ�ÿ��ģ�����
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
	case TURNAROUND:		//ԭ��ת180��
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
			if(tick>265){step=1;tick=0;}               //tick�ж���ֵ180�Ȼ�ת���鳤���й�
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
			if(turnaround_tick>100)tick=100;//��ֹ��������
		}
		break;
		
//=========================================================================================
	case POINTTURN90:		//ԭ��ת90��
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
			if(turnaround_tick>100)tick=100;//��ֹ��������
		}
		break;
		
//=========================================================================================
	case BEFORE90TURN:
//=========================================================================================
		if(forwardhaswall==1)//ת��ǰ��ǰ����ǽ����ʹ��ǰ����
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
		//���Ѿ���ʼת���ֳ�������ǽ��ǽ�仯��˵����һ�����Ӵ���˽�������ô�ڴ����½���
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
			setvl=(disf-30)*0.5+(disf1-30)*1;			//����������30mm
			setvr=(disf-30)*0.5+(disf2-30)*1;
			if(setvl>speed_turn) setvl=speed_turn;
			if(setvl<-speed_turn) setvl=-speed_turn;
			if(setvr>speed_turn) setvr=speed_turn;
			if(setvr<-speed_turn) setvr=-speed_turn;
			if( (disf-30)<-20 && abs(disf1-disf2)<100 )	//̫Զ�ҽǶ�ƫ���ֱ��
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
			if(turnaround_tick>100)tick=100;//��ֹ��������
		}
		break;
		
//=========================================================================================
	case FORWARDIRTEST:		//������� ��ǰ����
//=========================================================================================
		setvl=(disf-forwardirdis)*0.5+(disf1-forwardirdis)*1;	//������forwardirdis mm
		setvr=(disf-forwardirdis)*0.5+(disf2-forwardirdis)*1;
		if(setvl>50) setvl=50;
		if(setvl<-50) setvl=-50;
		if(setvr>50) setvr=50;
		if(setvr<-50) setvr=-50;
		if(setvl<2 && setvl>-2) setvl=0;
		if(setvr<2 && setvr>-2) setvr=0;
		if( (disf-forwardirdis)<-20 && abs(disf1-disf2)<100 )	//̫Զ�ҽǶ�ƫ���ֱ��
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
		setvl=(disf-17)*0.5+(disf1-17)*1;				//������17 mm
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
		if(turnaround_tick>500)tick=100;				//��ֹ��������
		break;
		
//===========================================================================================================================================
//===========================================================================================================================================
//=============================================================����ֱ��======================================================================
//===========================================================================================================================================
//===========================================================================================================================================
		
		
//=========================================================================================
	case GOSTRAIGHT:			//������ֱ��	  
//=========================================================================================
		eh=e;
		e=gangle;
		setvl=speed_ss-e*GOSTRAIGHT_kp;//0.4;		
		setvr=speed_ss+e*GOSTRAIGHT_kp;//0.4;
		break;
	
//=========================================================================================
	case SMARTSTRAIGHT:			//ת���ȫ��180�Ȼ�ת֮ǰ��ȫ
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//������ֱ��
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
	case SMARTSTRAIGHTFAST:		//����ֱ��ÿ��ǰ��Σ����������Ƿ���ǽ�������ۻ������
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//������ֱ��
		{
			eh=e;
			e=gangle;
			setvl=speed_ss-e*1;		
			setvr=speed_ss+e*1;
		}
		else
		{
			if((lastside==LEFT && lefthaswall==1) || (lastside==RIGHT && righthaswall==0))//����ģʽ����һ���õ��ı�ǽ�����ֻҪ�Ǳ���ǽ�ͻ����Ǳ�
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
	case STRAIGHTEYECLOSECOORADJ:	//��������ǽ���Ӻ��Σ���������ǽ��ǽ���������
//=========================================================================================
		eh=e;
		e=gangle;
		setvl=speed_ss-e*0.3;		
		setvr=speed_ss+e*0.3;
		#ifdef COORADJUST
		//if(blocknum>=3)//ֻ��������3������ֱ��ʱ�ſ������������⳵ҡ�ε��½�������
		//���������һ���Ľ����������������඼����ǽ��ǽ�仯����ȡ����ƽ��ֵ������
		{												
			if(lefthaswallh==1&&lefthaswall==0){settrack(trackcoor);}
			if(righthaswallh==1&&righthaswall==0){settrack(trackcoor);}
		}
		#endif
		break;
		
//=========================================================================================
	case STRAIGHTPOLE:			//��������ǽ���Ӻ��Ρ�������У����̬���������
//���ഫ���������ֵ��ʱ�������ĽǶ�ƫ������ֵʱ����ֵ���������
//��Ϊ������ض���������ǽ-��ǽ�ڼ���ã�����:
//�����С��ĳ��ֵʱ�����ֵ��⣻
//��ǰ�����β���������״�С��ĳ��ֵʱ��Ϊ�����ֵ�㣬��ʱ��¼��tick�ͷ�ֵ��ֵ��ͬʱ�رձ��ഫ�����ķ�ֵ��⡣
//������ķ�ֵ���Ѿ��õ�ʱ������tick֮��ͷ�ֵ֮�����һ�ν�����
//=================================
//�������Կ���ʶ���Ź������������ߵ�����н϶̾���Ľ���������
//=========================================================================================
// 		eh=e;
		e=gangle;
		setvl=speed_ss-e*0.5;		
		setvr=speed_ss+e*0.5;

		#ifdef COORADJUST
		//if(blocknum>=3)//ֻ��������3������ֱ��ʱ�ſ������������⳵ҡ�ε��½�������
		{
			if(lefthaswallh==1 && lefthaswall==0) settrack(trackcoor);
			if(righthaswallh==1 && righthaswall==0) settrack(trackcoor);
		}
		#endif
		break;	

//=======================================================================================================================================================================================
//=======================================================================================================================================================================================
//=======================================================================================������========================================================================================
//=======================================================================================================================================================================================
//=======================================================================================================================================================================================
//�õ���speed_ss, speed_fastss, 
//=========================================================================================
	case SMARTSTRAIGHTSPURT://ת��ǰ��ȫ
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//������ֱ��
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
	case SMARTSTRAIGHTSPURTL://ת��ǰ��ȫ
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//������ֱ��
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
	case SPURT90IR:	//���ʱ������90��ֱ��
//=========================================================================================
		if(lefthaswall==0 && righthaswall==0)//������ֱ��
		{
			eh=e;
			e=gangle;
			setvl=speed_ss-e*1;		
			setvr=speed_ss+e*1;
		}
		else
		{
			if((lastside==LEFT && lefthaswall==1) || (lastside==RIGHT && righthaswall==0))//����ģʽ����һ���õ��ı�ǽ�����ֻҪ�Ǳ���ǽ�ͻ����Ǳ�
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
	case SPURT90BLIND://���ʱ������90��ֱ��
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
	case STRAIGHT45://���45��ֱ��
//������ָ�������б���Լ���������ͽǶ�������궨
//δ����������������ϳ���ϵ��
//=========================================================================================
		if(disl<150)//�����ǽ
		{
			e=abovezero(disl+50-disf1)+abovezero(180-disf1);//eԽ��˵����Խ��ƫ������ͣ���һ�����Ƕ�ƫ��ڶ�������������ƫ��
			setvl=speed_fastss+e*STRAIGHT45_kp+(e-eh)*STRAIGHT45_kd;
			setvr=speed_fastss-e*STRAIGHT45_kp-(e-eh)*STRAIGHT45_kd;
		}
		else if(disr<150)//�Ҳ���ǽ 
		{
			e=abovezero(disr+50-disf2)+abovezero(180-disf2);//eԽ��˵����Խ��ƫ
			setvl=speed_fastss-e*STRAIGHT45_kp-(e-eh)*STRAIGHT45_kd;
			setvr=speed_fastss+e*STRAIGHT45_kp+(e-eh)*STRAIGHT45_kd;
		}
		if(disr>150 && disl>150)//���඼û��ǽ
		{
			e=/*angle+*/abovezero(220-disf2)-abovezero(220-disf1);
			setvl=speed_fastss-e*STRAIGHT45_kp;
			setvr=speed_fastss+e*STRAIGHT45_kp;
		}
		break;

	default:
		ledon(0);
	break;
	}//ϵͳ���ƺ����жϺ���END
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
//���PID���ƺ����жϺ���START
	//Left Motor
	gdvl=setvl*SpeedRatioL;						//cm/s -> Pulse/ms ϵ��Խ���ߵ�Խ��
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
	if(pwml>=35)//ǰ��
	{
		LPWML=0;LPWMH=pwml;
	}
	else if(pwml<=-35)//����
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
//���PID���ƺ����жϺ���END
//=========================================================================================
//=========================================================================================
//�����ж�																		
	if(EMERGENCYON==1)
	{
		//if(pwml>800 || pwmr>800) emergency_count++;
		if(pwml>SPEED_HAHA || pwmr>SPEED_HAHA || pwml<-SPEED_HAHA || pwmr<-SPEED_HAHA) emergency_count++;
		else emergency_count=0;
		if(emergency_count>200 || setvl>2000 || setvr>2000 || setvl<-2000 || setvr<-2000) isemergency=1;//200
	}
//=========================================================================================
//=========================================================================================
//��ص�ѹ���START	
	if(showbattery==1)
	{
		battaryh=battary;
		battary=Get_Adc(ADC_BATTARY)*3.3*3/4096*100;//����100ת��Ϊ����������8V-7VΪ��Դ��Χ
		battary=(battary*1+battaryh*9)/10;
		battarypercent=(battary-700)/12;
		if(battarypercent==0) battarypercent=1;
		if(battarypercent>8) battarypercent=8;
		#ifdef SHOWVOLTAGE
		ledoff(0);
		ledon(battarypercent);
		#endif
	}
//��ص�ѹ���END
//=========================================================================================
//=========================================================================================
//FLASH����洢��ÿ������ֻ��һ��long����
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
//FLASH�����洢
	if(flashenable==1)						
	{
		if(flashwrite==1)//��һ��55us
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
	cpu=cpu/10;			//ת���ɰٷ���
	if(flashenable==1)						
	{
		if(cpu>95)		//ʱ������ռ���ʸ���95%
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

		OSIntExit();       	 					//���������л����ж�
	}
}
#endif

			   
//��ʼ���ӳٺ���
//��ʹ��OS��ʱ��,�˺������ʼ��OS��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init()
{
#if SYSTEM_SUPPORT_OS  							//�����Ҫ֧��OS.
	u32 reload;
#endif
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/8000000;				//Ϊϵͳʱ�ӵ�1/8  
#if SYSTEM_SUPPORT_OS  							//�����Ҫ֧��OS.
	reload=SystemCoreClock/8000000;				//ÿ���ӵļ������� ��λΪK	   
	reload*=1000000/delay_ostickspersec;		//����delay_ostickspersec�趨���ʱ��
												//reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��1.86s����	
	fac_ms=1000/delay_ostickspersec;			//����OS������ʱ�����ٵ�λ	   

	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//����SYSTICK�ж�
	SysTick->LOAD=reload; 						//ÿ1/delay_ostickspersec���ж�һ��	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//����SYSTICK    

#else
	fac_ms=(u16)fac_us*1000;					//��OS��,����ÿ��ms��Ҫ��systickʱ����   
#endif
}								    

#if SYSTEM_SUPPORT_OS  							//�����Ҫ֧��OS.
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;					//LOAD��ֵ	    	 
	ticks=nus*fac_us; 							//��Ҫ�Ľ�����	  		 
	tcnt=0;
	delay_osschedlock();						//��ֹOS���ȣ���ֹ���us��ʱ
	told=SysTick->VAL;        					//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;		//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;				//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};
	delay_osschedunlock();						//�ָ�OS����									    
}
//��ʱnms
//nms:Ҫ��ʱ��ms��
void delay_ms(u16 nms)
{	
	if(delay_osrunning&&delay_osintnesting==0)	//���OS�Ѿ�������,���Ҳ������ж�����(�ж����治���������)	    
	{		 
		if(nms>=fac_ms)							//��ʱ��ʱ�����OS������ʱ������ 
		{ 
   			delay_ostimedly(nms/fac_ms);		//OS��ʱ
		}
		nms%=fac_ms;							//OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
	}
	delay_us((u32)(nms*1000));					//��ͨ��ʽ��ʱ  
}
#else //����OSʱ
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
	SysTick->VAL=0x00;        					//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      					 //��ռ�����	 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;       					//��ռ�����	  	    
} 
#endif 





































