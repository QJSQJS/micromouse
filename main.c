#include <stm32f10x.h>
#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "timer.h"
#include "adc.h"
#include "utility.h"
#include "stratagy.h"
#include "led.h"
#include "stmflash.h"
#include "includes.h"

/////////////////////////////////////
//�����ñ���

//cjm
int testi = 0;
int goout = 1;
extern uchar spurtTestAction[200];//200!һ��Ҫ�Ļ�����
extern u8 sidewall_threshold;
#define FLASHSAVE
extern u16 save_data_num,save_num;
extern u8 save_data_time,per_timer;
u16 Test_Time_Flag=0;extern int cpu;
extern u8 biaodingmode;
extern uchar 	SpurtTest_FastwayAction[MAZETYPE * MAZETYPE];
extern u16 accdistance;
extern u8 speedmode;
extern int TIM2CNT;
extern u8	spurtTestFlag;
extern u8 back90_flag;
extern u16 totaltrack;
extern u16 decdistance;
extern u8 spurt_45_end_flag;
int LTopSpeedh,L45TopSpeedh,BackTopSpeedh;
u16 ledcontrol=0;
u8 turndir=0;
u8 goaheadnum=0;
u8 BackSearchFlag=3;//������û�г�������ڵ�һ�γ�����������á�
extern long angle;
//==================================================================================
//========================��Ҫ���Գ��ʱ�޸�����=1��������block����==================================
extern u8 spurtTestFlag;
//==================================================================================
//==================================================================================
////////////����Ϊ������////////////  
u8 blockx,blocky;//FLASH��дblock������
u8 SearchOrSpurt_Flag=0;

u8 GoOrBackFlag=0;
uchar n          = 0;                                               /*  GmcCrossway[]�±�           */
extern u8 goal_keep_flag;//1Ϊ��  ��2Ϊ��

extern uchar	Mapfastway[MAZETYPE][MAZETYPE]      ;
extern FASTWAYACTION	FastwayAction[MAZETYPE * MAZETYPE];              /*  ��mapStepEdit()������ջʹ�� */
extern uchar	FastwayCount;

extern uchar	Mapfastway_90[MAZETYPE][MAZETYPE]      ;
extern FASTWAYACTION	FastwayAction_90[MAZETYPE * MAZETYPE];              /*  ��mapStepEdit()������ջʹ�� */
extern uchar	FastwayCount_90;

extern uchar	actionsequence[];

extern uchar    GucXStart;                /*  ��������                  */
extern uchar    GucYStart;                /*  ���������                  */

extern uchar    GucXGoal0;            /*  �յ�X���꣬������ֵ         */
extern uchar    GucXGoal1;
extern uchar    GucYGoal0;            /*  �յ�Y���꣬������ֵ         */
extern uchar    GucYGoal1;

extern uchar    GucMouseTask;             /*  ״̬������ʼ״̬Ϊ�ȴ�      */

extern char GoalX;
extern char GoalY;

extern uchar    GucMapStep[MAZETYPE][MAZETYPE];           /*  ���������ĵȸ�ֵ          */
extern uchar    GucMapStepS[MAZETYPE][MAZETYPE];           /*  ���������ĵȸ�ֵ          */
extern uchar    GucMapStepC[MAZETYPE][MAZETYPE];           /*  ���������ĵȸ�ֵ  QJS:����Ϊ�Ƕ�GucMapStep�Ŀ��� */  
extern uchar    GucMapStepSC[MAZETYPE][MAZETYPE];           /*  ���������ĵȸ�ֵ          */

extern MAZECOOR GmcStack[MAZETYPE * MAZETYPE];              /*  ��mapStepEdit()������ջʹ�� */
extern MAZECOOR GmcStackS1[MAZETYPE * MAZETYPE]; 
extern MAZECOOR GmcStackS2[MAZETYPE * MAZETYPE]; 
extern MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE];              /*  Main()���ݴ�δ�߹�֧·����  */

extern MAZECOOR GmcMouse;                                               /*  GmcMouse.x :�����������    */
                                                                        /*  GmcMouse.y :������������    */
                                                                        
extern uchar    GucMouseDir;                                            /*  �������ǰ������            */
extern uchar    GucMapBlock[MAZETYPE][MAZETYPE];                        /*  GucMapBlock[x][y]           */
                                                                        /*  x,������;y,������;          */
                                                                       /*  bit3~bit0�ֱ������������   */
                                                                       /*  0:�÷�����·��1:�÷�����·  */
extern uchar	GucMapGet[MAZETYPE][MAZETYPE];

extern u16 disf1,disf2,disf,disl,disr,disl45,disr45,disl45h,disr45h,disl45hh,disr45hh;
extern u8 leftwallcritical,rightwallcritical,leftwallcriticalh,rightwallcriticalh;
extern u8 righthaswall;
extern int setvl,setvr;
extern int gyroinit,ganglet;
extern double g;
extern u8 task;
extern int gangle,gangleh,ggangleh;
extern int gyro,gyro0;
extern int pwml,pwmr;
extern u8 battarypercent;
extern u16 gangleratio;
extern u8 angleterm,gyro_adj;
extern u16 angle_counter;
extern u8 freemode;
extern u8 sleepmode;
extern long track;
extern u8 irswitch;
extern u8 lefthaswall,righthaswall,forwardhaswall,lefthaswallh,righthaswallh;
extern u16 tick,turnaround_tick;
extern u8 dis2wall;
extern u16 speed_ss,speed_turn,speed_fastss;
extern u8 kalmanswitch;      //�������˲�  QJS

extern u16 dataf1[240],dataf2[240],datal[220],datar[220];
extern int rawf1,rawf2,rawl,rawr,rawf1h,rawf2h,rawlh,rawrh;
int rawf1avrgnear,rawf2avrgnear,rawlavrgnear,rawravrgnear,rawf1avrgfar,rawf2avrgfar,rawlavrgfar,rawravrgfar;
int iii;
extern int forwardirdis;
int showbattery=0;
extern int LTopSpeed,L45TopSpeed,L90TopSpeed,BackTopSpeed;
//=========================================================================================
u16 flashaddr=0,flashaddrmax,flashreadtemp;//flash�洢 ��ǰ��Ե�ַ��������ż�����������Ե�ַ
u8 flashwrite=1,flashenable=0;
int irselftestdiff;
//=========================================================================================
//=========================================================================================
u8 c=0;    //key
u16 tmptt;
//=========================================================================================
u8 spurtmode;                /*  Ԥ����  ���ģʽ ��LOW  LOW_90 ����ѡ��  QJS*/
//=========================================================================================

uchar ucRoadStat = 0;                                               /*  ͳ��ĳһ�����ǰ����֧·��  */
	uchar ucTemp     = 0;
	uchar m=0;
	//uchar l=0;
 //	uchar k=0;
	
//	u16 SpurtTest_Test=0;   //���ñ��� QJS
	
	uchar shortwayX=0;
	uchar shortwayY=0;
//	uchar shortwayN=0;      //���ñ��� QJS
	
	uchar GoalGet=0;
	uchar stepMax=12;
	uchar stepMaxLimit=12;
	
//	uchar dstep=2;
//	uchar dstepS=2;
	
//	uchar tempBlock;

	
	//��ʱ����
	char tempX=0;
	char tempY=0;
	
	char tempCX1,tempCX2,tempCY1,tempCY2,gox,goy;

	//�ײ�ģ����ϲ��޹�
	
	u32 ft=0;
	
	long flashtemp;

//�������ȼ�
#define START_TASK_PRIO		3
//�����ջ��С	
#define START_STK_SIZE 		512
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

#define KEY_TASK_PRIO		4
//�����ջ��С	
#define KEY_STK_SIZE 		128
//������ƿ�
OS_TCB KeyTaskTCB;
//�����ջ	
CPU_STK KEY_TASK_STK[KEY_STK_SIZE];
void key_task(void *p_arg);



//�������ȼ�
#define PREDO_TASK_PRIO		6
//�����ջ��С
#define PREDO_STK_SIZE		128
//������ƿ�
OS_TCB	PredoTaskTCB;
//�����ջ
CPU_STK	PREDO_TASK_STK[PREDO_STK_SIZE];
//������
void predo_task(void *p_arg);	
	

//�������ȼ�
#define MAZE_TASK_PRIO		7
//�����ջ��С
#define MAZE_STK_SIZE		128
//������ƿ�
OS_TCB	MazeTaskTCB;
//�����ջ
CPU_STK	MAZE_TASK_STK[MAZE_STK_SIZE];
//������
void maze_task(void *p_arg);
	
void hardware_init();
void mouseInit();

//�����ź���
OS_SEM BACKGROUND_SEM;
OS_SEM BEEP_SEM;
OS_SEM LED_SEM;

int main()                                          //������
 { //int i=0,j=0;
	OS_ERR err;
	CPU_SR_ALLOC();
	//delay_init(168);  	//ʱ�ӳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ�������	 
	hardware_init();            /*  Ӳ����ʼ��                         */
	mouseInit();               /*  ǽ�������ڴ���ʼ��                */
	
	//freemode=1;
	
	//����ǰ�������Ѵ��к��������ݣ����ô洢�����ݸ���ԭʼ����
	flashreadtemp=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA);
	if(flashreadtemp!=65535)
	{
		flashaddr=0;
		rawf1avrgnear=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
		rawf1avrgfar=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
		rawf2avrgnear=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
		rawf2avrgfar=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
		rawlavrgnear=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
		rawlavrgfar=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
		rawravrgnear=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
		rawravrgfar=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
		
		InfCorrect(rawlavrgnear,33,rawlavrgfar,94,250,datal,L);
		InfCorrect(rawravrgnear,33,rawravrgfar,95,250,datar,R);
		InfCorrect(rawf1avrgnear,32,rawf1avrgfar,213,250,dataf1,F1);//31-214
		InfCorrect(rawf2avrgnear,32,rawf2avrgfar,213,250,dataf2,F2);
	
		ledon(8);delay_ms(120);ledoff(0);delay_ms(120);ledon(8);delay_ms(120);ledoff(0);delay_ms(120);
	}
	else
	{
		InfCorrect(1719,33,441,94,250,datal,L);
		InfCorrect(1383,33,351,95,250,datar,R);
		InfCorrect(1676,32,120,213,250,dataf1,F1);//31-214
		InfCorrect(1474,32,111,213,250,dataf2,F2);
	}

//=========================================================================================
	//press button to choose between functions or modes
//=========================================================================================


	//ԭkey�����λ��QJS
	//ԭԤ��������λ��QJS
   //ԭmaze����λ��QJS
//=========================================================================================
//======================================������ʼ=========================================
//=========================================================================================	
	
	OSInit(&err);		//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();//�����ٽ���
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);  //����UCOSIII

        
}
		
//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	//���������л��ź���
	OSSemCreate((OS_SEM* ) &BACKGROUND_SEM,
				(CPU_CHAR* )"background_sem",
				(OS_SEM_CTR)0,
				(OS_ERR*   )&err);
	//���������������ź���
	OSSemCreate((OS_SEM* ) &BEEP_SEM,
				(CPU_CHAR* )"beep_sem",
				(OS_SEM_CTR)0,
				(OS_ERR*   )&err);
	//������ˮ�ƿ����ź���
	OSSemCreate((OS_SEM* ) &LED_SEM,
				(CPU_CHAR* )"led_sem",
				(OS_SEM_CTR)0,
				(OS_ERR*   )&err);
	//��������ɨ������
	OSTaskCreate((OS_TCB 	* )&KeyTaskTCB,		
				 (CPU_CHAR	* )"KEY task", 		
                 (OS_TASK_PTR )key_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )KEY_TASK_PRIO,     
                 (CPU_STK   * )&KEY_TASK_STK[0],	
                 (CPU_STK_SIZE)KEY_STK_SIZE/10,	
                 (CPU_STK_SIZE)KEY_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	
	//������������������
	OSTaskCreate((OS_TCB 	* )&PredoTaskTCB,		
				 (CPU_CHAR	* )"PREDO task", 		
                 (OS_TASK_PTR )predo_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )PREDO_TASK_PRIO,     
                 (CPU_STK   * )&PREDO_TASK_STK[0],	
                 (CPU_STK_SIZE)PREDO_STK_SIZE/10,	
                 (CPU_STK_SIZE)PREDO_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	 
	//����LCD��ʾ����
	OSTaskCreate((OS_TCB 	* )&MazeTaskTCB,		
				 (CPU_CHAR	* )"MAZE task", 		
                 (OS_TASK_PTR )maze_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )MAZE_TASK_PRIO,     
                 (CPU_STK   * )&MAZE_TASK_STK[0],	
                 (CPU_STK_SIZE)MAZE_STK_SIZE/10,	
                 (CPU_STK_SIZE)MAZE_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);	

	
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
	OS_CRITICAL_EXIT();	//�˳��ٽ���
}


void key_task(void *p_arg)
{
	u16 keydelay=0,ispressed=0;//key_task
    u8 keyfunc=0,keytmp=0;
	fuction_Choose:
	ledoff(0);ledon(6);
	while(1)				//��ʱ�ȴ��Լ���������ѡ��
	{
		if(BUTTON1==0 && ispressed==0)					//δ������ ��ʱ�ȴ� ����ʱ������������ѡ��	
		{
			delay_ms(5);
			keydelay++;
			if(keydelay==50){ledoff(0);ledon(5);}
			if(keydelay==100){ledoff(0);ledon(4);}
			if(keydelay==150){ledoff(0);ledon(3);}
			if(keydelay==200){ledoff(0);break;}
		}
		if(BUTTON1)		//�������£�ֹͣ����ʱ��
		{
			freemode=1;		//ֻҪ������ȥ������ջ�����Ϊ�п���Ҫ���������Ǳ궨�����Ի�������ȫ����sleepmode
			if(ispressed==0){ledoff(0);ispressed=1;}	//�������ʱ��ʱ�ȴ����ŵĵơ�
			delay_ms(5);	//����
			if(BUTTON1)
			{
				while(BUTTON1)
				{
					keytmp++;
					delay_ms(10);
					if(keytmp>60) break;				//Long Click
				}

				if(keytmp<60)							//Short Click
				{
					keytmp=0;
					ledoff(keyfunc);
					keyfunc++;
					if(keyfunc==9)keyfunc=1;
					ledon(keyfunc);
				}
				else									//Long Click
				{	
					keytmp=0;
					ledoff(keyfunc);
					delay_ms(80);
					ledon(keyfunc);
					delay_ms(80);
					ledoff(keyfunc);
					delay_ms(80);
					ledon(keyfunc);
					delay_ms(500);
					ledoff(keyfunc);
					break;
				}
			}
			else;
		}
	}
	while(BUTTON1);		//ȷ������ѡ��ʱ�򳤰������Ѿ����֣���ֹ������Ϊ�ڹ����а�����
	
	freemode=0;
	
	switch(keyfunc)
	{
//=========================================================================================
		case 0:				//No Pressing
//=========================================================================================
			break;
//=========================================================================================
		case 1:				//Show Battary Life
//=========================================================================================
//			showbattery=1;
//			while(1)
//			{
//				//sleepmode=1;//��ȡ����ʱ���رչ��ܣ���Ӱ����ʵ����
//				ledoff(0);
//				delay_ms(200);
//				ledon(battarypercent);
//				delay_ms(200);
//			}
//================================���FLASH����ĵ�������������============================
			freemode=1;
			#ifdef FLASHSAVE
			print_data ();
			ledoff(0);ledon(1);ledon(2);
			delay_ms(2000);
//			erase_run_data();
			#endif
//			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_DATA_NUM);
			ledon(0);
			while(1)
			{
				sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(500);c++;
			}
//=========================================================================================
		case 2:				//��������У׼����
//=========================================================================================
		freemode = 1;	
		flashaddr=0;
			for(tmptt=0;tmptt<8;tmptt++)
			{
				STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
			}
			while(1)
			{
				sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(1000);c++;
			}
//=========================================================================================
		case 3:				//�Զ�У׼���������
//=========================================================================================
			angle_counter=0;
		freemode = 1;
			gyro_adj=1;
			while(1)
			{	
				if(gangle>=0)
				{ledoff(0);ledon(gangle%8+1);}
				else if(gangle<0)
				{ledoff(0);ledon(gangle%8+8);}

				if(angle_counter>1500)break;//����1���ڽǶ� ������Ϊ�Ѿ��������
			}
			gyro_adj=0;
			STMFLASH_Unlock();
			STMFLASH_ErasePage(FLASH_ADDR_GYRO0);
			STMFLASH_Unlock();
			STMFLASH_WriteHalfWord(FLASH_ADDR_GYRO0,gyro0);//д�������������������ѹ
			sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
			ledon(0);delay_ms(100);ledoff(0);
			c=1;
				
			while(1)
			{
				printf("GYRO0:%d,",gyro0);
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(500);c++;
			}
			
			ledon(0);
			delay_ms(500);
			ledoff(0);
			sleepmode=0;
			break;
			break;
//=========================================================================================
		case 4:				//�������Լ�
//=========================================================================================
			freemode=1;
		
		while(1)
			{
				irselftestdiff=disl-dis2wall+4;
				if(irselftestdiff>8) irselftestdiff=8;
				if(irselftestdiff<1) irselftestdiff=1;
				ledoff(0);
				ledon(irselftestdiff);
				if(BUTTON1==1)//�������£�ֹͣ����ʱ��
				{
					delay_ms(5);//����
					if(BUTTON1==1)break;
				}
				printf("disl:%d,disr:%d \r\n",disl,disr);
				delay_ms(100);
			}
			ledon(0);
			delay_ms(2000);
			ledoff(0);
			while(1)
			{
				irselftestdiff=disr-dis2wall+4;
				if(irselftestdiff>8) irselftestdiff=8;
				if(irselftestdiff<1) irselftestdiff=1;
				ledoff(0);
				ledon(irselftestdiff);
				if(BUTTON1==1)//�������£�ֹͣ����ʱ��
				{
					delay_ms(5);//����
					if(BUTTON1==1)break;
				}
				printf("disl:%d,disr:%d \r\n",disl,disr);
				delay_ms(100);
			}
			
			ledoff(0);ledon(4);delay_ms(1000);ledoff(0);ledon(3);delay_ms(1000);ledoff(0);ledon(2);delay_ms(1000);ledoff(0);ledon(1);delay_ms(1000);
			
			freemode=0;
			task=FORWARDIRTEST;
			forwardirdis=100;
			delay_ms(5000);
			forwardirdis=50;
			delay_ms(5000);
			forwardirdis=17;
			delay_ms(5000);
			task=NULL;
			while(1)
			{
				freemode=1;
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(1000);c++;
			}
//=========================================================================================
		case 5:				//��ʾ����������
//=========================================================================================
			irswitch=1;
		freemode=1;
		gangleSee = 1;
			
			while(1)
			{
				printf("f1:%d  f2:%d  l:%d  r:%d  gangle&Adc:%d,%d  angle:%ld CPU:%dpercent   \r\n",disf1,disf2,disl,disr,gangle,gyro,angle,cpu);
				if(BUTTON1==1)
				{
					delay_ms(5);
					if(BUTTON1) break;
				}
				delay_ms(300);
			}
			ledon(0);
			delay_ms(1000);
			ledoff(0);
			while(1)
			{
				printf("adcf1:%d adcf2:%d adcl:%d adcr:%d gangle_Rate:%d  gangle:%d  angle_code:%ld \r\n",rawf1,rawf2,rawl,rawr,gyro,gangle,angle);
				if(BUTTON1==1)
				{
					delay_ms(5);
					if(BUTTON1) break;
				}
				delay_ms(300);
			}
			
			while(1)
			{
				sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
				
				if(c==9||c==0)c=1;
				
				ledoff(0);ledon(c);delay_ms(1000);c++;
				if(BUTTON1==1)
				   {  
					   delay_ms(5);
					   if(BUTTON1==1)
					      {
				             ispressed=0;
				             goto fuction_Choose;
				          }
				    }
			}
//=========================================================================================
		case 6:				//��������ߴ���������
//=========================================================================================
		//��������ǽ�ľ��룺����31mm��103mm��ǰ��30mm��210mm
		//��һ�ζ�ȡ����߽����ұ�Զ���ڶ��ζ�ȡ���ұ߽������Զ�������ζ�ȡ��ǰ���������Ĵζ�ȡ��ǰ��Զ��
		//����˳���ȿ����ڿ��ң�Ȼ��ǰ������Ǻ��
			
		freemode = 1;
		flashreadtemp=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA);
			if(flashreadtemp!=65535)
			{
				flashaddr=0;
				rawf1avrgnear=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
				rawf1avrgfar=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
				rawf2avrgnear=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
				rawf2avrgfar=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
				rawlavrgnear=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
				rawlavrgfar=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
				rawravrgnear=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
				rawravrgfar=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
				
				printf("rawLFavrgnear:%d,rawLFavrgfar:%d \r\n",rawf1avrgnear,rawf1avrgfar);
				printf("rawLavrgnear:%d,rawLavrgfar:%d \r\n",rawlavrgnear,rawlavrgfar);
				printf("rawRavrgnear:%d,rawRavrgfar:%d \r\n",rawravrgnear,rawravrgfar);
				printf("rawRFavrgnear:%d,rawRFavrgnear:%d \r\n",rawf2avrgnear,rawf2avrgfar);
				
				while(1)
				{
					sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
					if(c==9||c==0)c=1;
					ledoff(0);ledon(c);delay_ms(1000);c++;
				}
			}
	
			biaodingmode=1;
			ledoff(0);
			while(1)
			{
				ledon(1);ledon(2);
				if(BUTTON1==1)
				{
					delay_ms(5);//����
					if(BUTTON1==1)break;
				}					
			}
			ledoff(0);ledon(4);delay_ms(1000);ledoff(0);ledon(3);delay_ms(1000);ledoff(0);ledon(2);delay_ms(1000);ledoff(0);ledon(1);delay_ms(1000);
			rawlavrgnear=rawl;//��ȡ�󴫸�����ǽ��ѹ
			rawravrgfar=rawr;//��ȡ�Ҵ�����Զǽ��ѹ
			//===================
			ledoff(0);
			delay_ms(500);
			while(1)
			{
				ledon(7);ledon(8);
				if(BUTTON1==1)
				{
					delay_ms(5);//����
					if(BUTTON1==1)break;
				}					
			}
			ledoff(0);ledon(4);delay_ms(1000);ledoff(0);ledon(3);delay_ms(1000);ledoff(0);ledon(2);delay_ms(1000);ledoff(0);ledon(1);delay_ms(1000);
			rawlavrgfar=rawl;
			rawravrgnear=rawr;
			//====================
			
			ledoff(0);
			delay_ms(500);
			while(1)
			{
				ledon(4);ledon(5);
				if(BUTTON1==1)
				{
					delay_ms(5);//����
					if(BUTTON1==1)break;
				}					
			}
			ledoff(0);ledon(4);delay_ms(1000);ledoff(0);ledon(3);delay_ms(1000);ledoff(0);ledon(2);delay_ms(1000);ledoff(0);ledon(1);delay_ms(1000);
			rawf1avrgnear=rawf1;
			rawf2avrgnear=rawf2;
			//====================
			
			ledoff(0);
			delay_ms(500);
			while(1)
			{
				ledon(3);ledon(6);
				if(BUTTON1==1)
				{
					delay_ms(5);//����
					if(BUTTON1==1)break;
				}					
			}
			ledoff(0);ledon(4);delay_ms(1000);ledoff(0);ledon(3);delay_ms(1000);ledoff(0);ledon(2);delay_ms(1000);ledoff(0);ledon(1);delay_ms(1000);
			rawf1avrgfar=rawf1;
			rawf2avrgfar=rawf2;
			//====================
			
			flashaddr=0;
			for(tmptt=0;tmptt<8;tmptt++)
			{
				STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
			}
			flashaddr=0;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_IRDATA+flashaddr,rawf1avrgnear);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_IRDATA+flashaddr,rawf1avrgfar);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_IRDATA+flashaddr,rawf2avrgnear);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_IRDATA+flashaddr,rawf2avrgfar);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_IRDATA+flashaddr,rawlavrgnear);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_IRDATA+flashaddr,rawlavrgfar);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_IRDATA+flashaddr,rawravrgnear);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_IRDATA+flashaddr,rawravrgfar);flashaddr+=2;
			
			ledon(0);delay_ms(300);ledoff(0);delay_ms(300);
			biaodingmode=0;
			while(1)
			{
				sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(1000);c++;
			}
			break;
//=========================================================================================
		case 7:				//��ʾ���䶯���ͺ���У׼����
//=========================================================================================
		freemode=1;	
		flashaddr=0;
			for(blockx=0;blockx<16;blockx++)
			{for(blocky=0;blocky<16;blocky++)
			{GucMapBlock[blockx][blocky]=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK+flashaddr);flashaddr+=2;}}
			flashaddr=0;
			GucXStart=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr);flashaddr+=2;
			GucYStart=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr);flashaddr+=2;
			GoalX=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr);flashaddr+=2;
			GoalY=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr);flashaddr+=2;

			printf("block picture:\r\n");
			Show_Block();
			
			printf("block data:\r\n");
			for(blockx=0;blockx<16;blockx++)
			{
				printf("{\r\n");
				for(blocky=0;blocky<16;blocky++) printf("%d,",GucMapBlock[blockx][blocky]);
				printf("},\r\n");
			}
		
			printf("\r\n");
			printf("45 spurt go path: ");
			flashaddr=0;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				FastwayAction[tmptt].action=STMFLASH_ReadHalfWord(FLASH_ADDR_GOPATH+flashaddr); flashaddr+=2;
				printf("%d,",FastwayAction[tmptt].action);
			}
			printf("        \r\nback path: ");
			flashaddr=0;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				FastwayAction[tmptt].action=STMFLASH_ReadHalfWord(FLASH_ADDR_BACKPATH+flashaddr); flashaddr+=2;
				printf("%d,",FastwayAction[tmptt].action);
			}
			printf("\r\n"); printf("90 spurt go path: ");
			flashaddr=0;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				FastwayAction[tmptt].action=STMFLASH_ReadHalfWord(FLASH_ADDR_GOPATH_90+flashaddr); flashaddr+=2;
				printf("%d ",FastwayAction[tmptt].action);
			}
			printf("        \r\nback path: ");
			flashaddr=0;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				FastwayAction[tmptt].action=STMFLASH_ReadHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr); flashaddr+=2;
				printf("%d ",FastwayAction[tmptt].action);
			}
			
			printf("       \r\nIR DATA: ");
			flashaddr=0;
			for(tmptt=0;tmptt<8;tmptt++)
			{			
				flashreadtemp=STMFLASH_ReadHalfWord(FLASH_ADDR_IRDATA+flashaddr); flashaddr+=2;
				printf("%d,",flashreadtemp);
			}
			printf("\r\nu16 dataf1[250]={");
			for(tmptt=0;tmptt<249;tmptt++) printf("%d,",dataf1[tmptt]);
			printf("0};");
			printf("\r\nu16 dataf2[250]={");
			for(tmptt=0;tmptt<249;tmptt++) printf("%d,",dataf2[tmptt]);
			printf("0};");
			printf("\r\nu16 datal[250]={");
			for(tmptt=0;tmptt<249;tmptt++) printf("%d,",datal[tmptt]);
			printf("0};");
			printf("\r\nu16 datar[250]={");
			for(tmptt=0;tmptt<249;tmptt++) printf("%d,",datar[tmptt]);	
			printf("0};");		
			while(1)
			{
				sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(1000);c++;
			}
//=========================================================================================
		case 8:				//�����̶���
//=========================================================================================
		freemode=1;	
		#ifdef FLASHSAVE
			erase_run_data();
			#endif
			STMFLASH_Unlock(); STMFLASH_ErasePage(FLASH_ADDR_BLOCK);
			STMFLASH_Unlock(); STMFLASH_ErasePage(FLASH_ADDR_BLOCK_POINT);
			STMFLASH_Unlock(); STMFLASH_ErasePage(FLASH_ADDR_GOPATH);
			STMFLASH_Unlock(); STMFLASH_ErasePage(FLASH_ADDR_BACKPATH);
			STMFLASH_Unlock(); STMFLASH_ErasePage(FLASH_ADDR_GOPATH_90);
			STMFLASH_Unlock(); STMFLASH_ErasePage(FLASH_ADDR_BACKPATH_90);
			while(1)
			{
				sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
				ledoff(0);ledon(c);delay_ms(1000);c++;
				if(c==9) c=1;
			}		   
		default:
			while(1)
			{
				sleepmode=1;//�ر�ʱ�����ڵ�ȫ����������Լ����
				ledoff(0);ledon(c);delay_ms(1000);c++;
				if(c==9) c=1;
			}
	}
}



//=========================================================================================
void mouseInit()                                   //�ȸ�ֵ��ʼ�� QJS
{
	int i=0,j=0;

	for(i=0;i<MAZETYPE;i++)
	{
		for(j=0;j<MAZETYPE;j++)
		{
			GucMapBlock[i][j] = 0xf0; 
		}
	}

	GucMapBlock[0][0] = 0x11;
	GucMapBlock[1][0] = 0x70;
		
//=========================================================================================		
// 		flashaddr=0;
// 		for(blockx=0;blockx<16;blockx++)
// 		{for(blocky=0;blocky<16;blocky++)
// 			{
// 				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK+flashaddr,GucMapBlock[blockx][blocky]);flashaddr+=2;
// 			}}
// 			
// 			flashaddr=0;
// 		for(blockx=0;blockx<16;blockx++)
// 		{for(blocky=0;blocky<16;blocky++)
// 			{
// 				GucMapBlock[blockx][blocky]=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK+flashaddr);flashaddr+=2;
// 			}}	
//=========================================================================================		
}
//=========================================================================================
//=========================================================================================
void hardware_init()                               //Ӳ����ʼ�� QJS
{
	Stm32_Clock_Init(9);	//ϵͳʱ������, 9��Ƶ, ϵͳ������8M*9=72MHz
	delay_init();			//��ʱ��ʼ��
	uart_init(72,115200);	//���ڳ�ʼ�� 
	Adc_Init();				//ADC��ʼ��Ҫ��delay֮ǰ��Ϊ�ӳ��ṩʱ��
	PWM_motor_Init();
	Led_Init();
	Timer1_Init();
	Timer2_Init();
	Timer3_Init();
	gyro0=STMFLASH_ReadHalfWord(FLASH_ADDR_GYRO0);		//��ȡ����flash�е�����������ѹֵ
	if(gyro0==-1)
	{
		u8 ledswitch=0;
		gyro0=2044;freemode=1;
		angle_counter=0;
		gyro_adj=1;
		while(1)
		{	
			if(gangle>=0)
			{ledoff(0);ledon(gangle%8+1);}
			else if(gangle<0)
			{ledoff(0);ledon(gangle%8+8);}

			if(angle_counter>1000)break;				//����1���ڽǶȲ�����Ϊ�Ѿ��������
		}
		gyro_adj=0;
		STMFLASH_Unlock();
		STMFLASH_ErasePage(FLASH_ADDR_GYRO0);
		STMFLASH_Unlock();
		STMFLASH_WriteHalfWord(FLASH_ADDR_GYRO0,gyro0);	//д�������������������ѹ
		sleepmode=1;									//�ر�ʱ�����ڵ�ȫ����������Լ����
		ledon(0);delay_ms(100);ledoff(0);
		ledswitch=1;
		while(1)
		{
			ledoff(0); ledon(ledswitch);
			delay_ms(200);
			ledswitch++;
			if(ledswitch==9) break;
		}
		ledon(0);
		delay_ms(500);
		ledoff(0);
		sleepmode=0;
	}
}

void predo_task(void *p_arg)	
{
	u16 spurtTestNum1=0, spurtTestNum2=0, spurtTestNum3=0;
	flashaddr=0;
	flashaddrmax=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK);
	if(flashaddrmax!=65535)
	{
		flashaddr=0;
		for(blockx=0;blockx<16;blockx++)
		{
			for(blocky=0;blocky<16;blocky++)
			{
				GucMapBlock[blockx][blocky]=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK+flashaddr); flashaddr+=2;
			}
		}
		
		flashaddr=0;
		GucXStart=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr); flashaddr+=2;
		GucYStart=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr); flashaddr+=2;
		GoalX=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr); flashaddr+=2;
		GoalY=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr); flashaddr+=2;
		goal_keep_flag=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr); flashaddr+=2;
		SearchOrSpurt_Flag=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr); flashaddr+=2;
		
		GmcMouse.cX=GucXStart;
		GmcMouse.cY=GucYStart;
		 
		delay_ms(100);ledon(1);delay_ms(120);ledoff(0);delay_ms(120);ledon(1);delay_ms(120);ledoff(0);delay_ms(120);
		while(1)
		{
			if(disf1<20)									//б�߳��
			{
				GucMouseTask=SPURT_45;
				spurtmode=LOW;
				ledon(1);ledon(2); delay_ms(1000);
				ledoff(0); delay_ms(500);
				while(1)
				{
					if(disf1<20)							//����
					{
						LTopSpeed=200;
						L45TopSpeed=160;
						BackTopSpeed=200;
						ledon(1);ledon(2);ledon(3);ledon(4); delay_ms(1000);
						ledoff(0);
						break;
					}
					if(disf2<20)							//����
					{
						LTopSpeed=240;
						L45TopSpeed=180;
						BackTopSpeed=240;
						ledon(5);ledon(6);ledon(7);ledon(8); delay_ms(1000);
						ledoff(0);
						break;
					}
				}
				break;
			 }
			
		 	if(disf2<20)
			{
				GucMouseTask=SPURT_90;
				spurtmode=LOW_90;
				ledon(7);ledon(8); delay_ms(1000);
				ledoff(0); delay_ms(500);
				while(1)
				{
					if(disf1<20)							//����
					{
						L90TopSpeed=70;
						BackTopSpeed=70;
						ledon(1);ledon(2);ledon(3);ledon(4); delay_ms(1000);
						ledoff(0);
						break;
					}
					if(disf2<20)							//����
					{
						L90TopSpeed=240;
						BackTopSpeed=220;
						ledon(5);ledon(6);ledon(7);ledon(8); delay_ms(1000);
						ledoff(0);
						break;
					}
				}
				goaheadnum=0;
				break;
			}
		}
	}
	
/////////////                     QJS
	if(GucMouseTask!=SPURT_45 && GucMouseTask!=SPURT_90)
	{
		while(1)
		{
			if(disf2<20){goaheadnum=2; break;}//��һ��ֱ�߳��
			if(disf1<20){goaheadnum=1; break;}//��һ��б�߳��
		}
		ledon(6);delay_ms(1000);ledoff(0);
	}
	flashenable=0;			//flashwriteĬ��Ϊ1�����Դӳ�����ʽ���п�ʼ�ͻ�д�롣ֻ��Ҫ��ֹͣд��ĵط���һ��flashwrite=0;����
	irswitch=1;
	biaodingmode=0;
	setangle(0);			//����֮ǰ������һ��������
	BackSearchFlag=0;
//=========================================================================================	
//====================================��̵���ģʽ=========================================
//=========================================================================================
	if(spurtTestFlag==1)
	{
		flashenable=1;
		sidewall_threshold=140;
		ledoff(0);ledon(4);ledon(5);
		speed_turn=110;accdistance=205;
		taskacc();settrack(180);
		speedmode=LOW;
		taskstraight90(180*1);
		for(spurtTestNum3=0;spurtTestNum3<1;spurtTestNum3++)
			for(spurtTestNum1=0;spurtTestNum1<5;)
			{
				if(spurtTestAction[spurtTestNum1]==STRAIGHT45L)
				{	
					spurtTestNum2=0;
					while(spurtTestAction[spurtTestNum1]==STRAIGHT45L)
					{
						spurtTestNum2++;
						spurtTestNum1++;
					}
					taskstraight45(127*spurtTestNum2);
				 }
				else if (spurtTestAction[spurtTestNum1]==STRAIGHT90L)
					{
						spurtTestNum2=0;
						while(spurtTestAction[spurtTestNum1]==STRAIGHT90L)
						{
							spurtTestNum2++;
							spurtTestNum1++;
						}
						taskstraight90(180*spurtTestNum2);
					}
					else
					{
						spurt_45_end_flag=0; 			
						mouseturn(spurtTestAction[spurtTestNum1]);
						spurtTestNum1++;
					}
			}
		decdistance=110;
		taskstop();
		freemode=1;
		sleepmode=1;
		flashenable=0;
		ledon(1);ledon(3);ledon(5);ledon(7);
		erase_run_data();	
		//------------------------------------------
		ledoff(0);
		ledon(2);ledon(4);ledon(6);ledon(8);
		flashaddr=0;
		STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA_NUM+flashaddr,save_data_time);flashaddr+=2;
		STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA_NUM+flashaddr,save_data_num);flashaddr+=2;
		flashaddr=0;
		for(save_data_num=0;save_data_num<save_num;save_data_num++)
		{
			save_data(save_data_num);
			flashaddr+=2;
		}
		ledoff(0);
		delay_ms(1000);
		ledon(0);
		while(1) sleepmode=1;
	}
//////////////////////////////////////////Ԥ�������  finish	 QJS
}

void maze_task(void *p_arg)
{
	while (1)
  {
	uchar l=0;
 	uchar k=0;
	
   switch (GucMouseTask) {                                         /*  ״̬������                  */
            
        case WAIT:
            //delay_ms(1000);                        
			taskacc();settrack(100);
            GucMouseTask = START;
            break;
            
        case START:                                                     /*  �жϵ��������ĺ�����      */
	        mazeSearch();                                               /*  ��ǰ����                    */
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) {         /*  �жϵ���������Ƿ���ڳ���  */
                GucXStart   = MAZETYPE - 1;                             /*  �޸ĵ��������ĺ�����      */
                GmcMouse.cX = MAZETYPE - 1;                             /*  �޸ĵ�����ǰλ�õĺ�����  */    
                /*
                 *  ����Ĭ�ϵ����Ϊ(0,0)��������Ҫ���Ѽ�¼��ǽ������ת������
                 */
				 
				if(GmcMouse.cY<MAZETYPE - 1)
				{
					GucMapBlock[MAZETYPE - 1][GmcMouse.cY+1] = GucMapBlock[0][GmcMouse.cY+1];
					GucMapBlock[0][GmcMouse.cY+1] = 0xf0;
				}
                ucTemp = GmcMouse.cY;
                do {
                    GucMapBlock[MAZETYPE - 1][ucTemp] = GucMapBlock[0][ucTemp];
					GucMapBlock[MAZETYPE - 2][ucTemp] = 0xD0;
                    GucMapBlock[0 ][ucTemp] = 0xf0;
					GucMapBlock[1 ][ucTemp] = 0xf0;
                }while (ucTemp--);
				GucMapBlock[MAZETYPE - 2][GmcMouse.cY] = 0xf0;
				GoalX=8;
                /*
                 *  ��OFFSHOOT[0]�б����������
                 */
                GmcCrossway[n].cX = MAZETYPE - 1;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  ״̬ת��Ϊ��Ѱ״̬          */
            }
            else if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) {         /*  �жϵ������ұ��Ƿ���ڳ���  */
                /*
                 *  ��OFFSHOOT[0]�б����������
                 */
                GmcCrossway[n].cX = 0;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  ״̬ת��Ϊ��Ѱ״̬          */
            }
            break;
			
//=========================================================================================
//=======================================������ʼ==========================================
//=========================================================================================	   
        case MAZESEARCH:
//---------------------------------�жϵ������Ƿ����յ�------------------------------------		 
			if(((GmcMouse.cX==GucXGoal0)||(GmcMouse.cX==GucXGoal1))&&((GmcMouse.cY==GucYGoal0)||(GmcMouse.cY==GucYGoal1)))
			{
				if((GmcMouse.cX+GmcMouse.cY+GucMouseDir)%2==0) goal_keep_flag=1;  //�����
				else goal_keep_flag=2; 
				GoalGet=1;
				GoalX=GmcMouse.cX;
				GoalY=GmcMouse.cY;
				goalWallchange(GoalX,GoalY);
				
				mapStepEditS(GucXStart,GucYStart);
				mapStepEditSC();
				mapStepEdit(GucXStart,GucYStart);
				mapStepEditC();
				mapStepEditS(GoalX,GoalY);
				mapStepEdit(GoalX,GoalY);
				
				Block_Write();             /*  ��λ BackSearchFlag  */      
//===========================================================================
//=======================2016��3��18���޸Ķ���===============================
//=======================���յ�ͷ��������================================
			if(BackSearchFlag==0)
			{
				//objectGoTo(GucXStart,GucYStart);
				//������㿪ʼ
				mouseGoahead(1);
						tick=0;
						turnaround_tick=0;
						task=KEEPWALLDISTANCE;
						while(tick<50) ;                   
										//������ʼ
														if(goal_keep_flag!=0)
										{//GucMouseDir = (GucMouseDir + 2) % 4;
											if(goal_keep_flag==1)
											{
												taskturn(POINTL90);
												tick=0;
												turnaround_tick=0;
												task=KEEPWALLDISTANCE;
												while(tick<50);
												taskpointturnleft();
											}
											else if(goal_keep_flag==2)
											{
												taskturn(POINTR90);
												tick=0;
												turnaround_tick=0;
												task=KEEPWALLDISTANCE;
												while(tick<50);
												taskpointturnright();
											}	
										}
										else
										{specialmouseTurnback();}
										//��������
				LTopSpeedh=LTopSpeed;L45TopSpeedh=L45TopSpeed;BackTopSpeedh=BackTopSpeed;
				LTopSpeed=70;L45TopSpeed=140;BackTopSpeed=70;
				fastway_90(GucXStart,GucYStart);
				spurt_90(LOW_90);
				LTopSpeed=LTopSpeedh;L45TopSpeed=L45TopSpeedh;BackTopSpeed=BackTopSpeedh;
				//����������
				specialmouseTurnback();
					GmcMouse.cX=GucXStart;
					GmcMouse.cY=GucYStart;
					GucMouseDir=0;
										
										
				//======================cjm================
//				LTopSpeed=70;L45TopSpeed=140;BackTopSpeed=70;
//				fastway_90(GoalX,GoalY);
//				spurt_90(LOW_90);
//				while(1)
//				{
//					freemode = 1;
//					ledoff(0);
//					ledon(1);
//					ledon(4);
//				}
				//=======================================
										
										
				GucMouseTask = SPURT;                               /*  �����󽫿�ʼ���״̬        */
				break;
			}
//===========================================================================

				if((GucMapStep[GucXStart][GucYStart]<=stepMax)||(GucMapStep[GucXStart][GucYStart]-GucMapStepS[GucXStart][GucYStart]==0))
				{
					objectGoTo(GucXStart,GucYStart);
					specialmouseTurnback();
					GucMouseTask = SPURT; 
					break;
				}
				else
				{
					while (--n)    /*  n��GmcCrossway[]�±�    GmcCrossway[]��Main()���ݴ�δ�߹�֧·����  QJS */
					{
						ucRoadStat = crosswayCheck(GmcCrossway[n].cX,GmcCrossway[n].cY);
						tempX=GmcCrossway[n].cX;
						tempY=GmcCrossway[n].cY;
 
						/// ucRoadStat����ǰ��֧·�� 	���³������˵����п���ǰ����֧·����ͨ���ȸ�ͼ�ж���ô�� QJS
						
						if (ucRoadStat)                    
						{
							if(GucMapStepC[tempX][tempY]+GucMapStepS[tempX][tempY]<GucMapStep[GucXStart][GucYStart])
							{
								objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);                      //���������ص���Ľ�
								shortwayX=GmcCrossway[n].cX;
								shortwayY=GmcCrossway[n].cY;
								k=n;
								if (ucRoadStat > 1) 
								{
									k++;
								}
								mapStepEdit(GoalX,GoalY);
								backpointwaychoice(tempX,tempY);
								mazeSearch();
								break;
							}
						}
					}
					if (n == 0)
					{
						objectGoTo(GmcCrossway[0].cX, GmcCrossway[0].cY);
						specialmouseTurnback();
						GucMouseTask = SPURT;                               /*  �����󽫿�ʼ���״̬        */
						break;
					}
				}
			}
///-----------------------------------------���������յ�-----------------------------------------------------------	
			
			else
			{
//-----------------------------------------������δ������յ�----------------------------------------------------------	
				if(GoalGet==0)
				{
					ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  ͳ�ƿ�ǰ����֧·��          */
					gox=GmcMouse.cX;
					goy=GmcMouse.cY;
					if (ucRoadStat) {                                           /*  �п�ǰ������                */
						m=deadwaycheck(n);                     //�����������ص�
						if(m==n)
						{
							if (ucRoadStat > 1) {                                   /*  �ж�����ǰ�����򣬱�������  */
								GmcCrossway[n].cX = GmcMouse.cX;
								GmcCrossway[n].cY = GmcMouse.cY;
								n++;
							}
						}
						else
						{
							gox=GmcCrossway[m].cX;
							goy=GmcCrossway[m].cY;
							n=m+1;
							objectGoTo(gox,goy);
						}
						crosswayChoice();
						mazeSearch();                                           /*  ǰ��һ��                    */
					} 
					else 
					{                                                    /*  û�п�ǰ�����򣬻ص����֧·*/
						m=deadwaycheck(n);
						if(m<n) n=m+1;
						while (--n) {
							ucRoadStat = crosswayCheck(GmcCrossway[n].cX,
													   GmcCrossway[n].cY);
																				/*  ͳ�����֧��δ�߹��ķ�����  */
																				
							/*
							 *  ������δ�߹���·����ǰ����֧�㣬������ѭ��
							 *  ����������һ���δ�߹���֧·��
							 */
							if (ucRoadStat) {
								m=n;
								mapStepEditS(GoalX,GoalY);
								if(m>0)
								{
									tempCX1=GmcCrossway[m].cX;
									tempCY1=GmcCrossway[m].cY;
									l=m;                             //��ôдȷ��û������?   ����ĸL -lû�����
									tempCX2=GmcCrossway[m].cX;
									tempCY2=GmcCrossway[m].cY;
									gox=tempCX1;
									goy=tempCY1;
									while(l)                                   //shizimu  L   QJS   ������ѭ��
									{
										l--;
										ucRoadStat = crosswayCheck(GmcCrossway[l].cX,GmcCrossway[l].cY);
										if (ucRoadStat)
										{
											tempCX2=GmcCrossway[l].cX;
											tempCY2=GmcCrossway[l].cY;
											if(GucMapStepS[tempCX1][tempCY1]>GucMapStepS[tempCX2][tempCY2])
											{
												m=l;
												tempCX1=GmcCrossway[m].cX;
												tempCY1=GmcCrossway[m].cY;
												gox=GmcCrossway[m].cX;
												goy=GmcCrossway[m].cY;
												continue;
											}
											else
											{
												break;
											}
										}
									}									
								}

								objectGoTo(gox,goy); 
								ucRoadStat = crosswayCheck(gox,goy);
								n++;
								if (ucRoadStat > 1) {
									GmcCrossway[n].cX=GmcCrossway[m].cX;
									GmcCrossway[n].cY=GmcCrossway[m].cY;
									n++;
								}
								crosswayChoice();
								mazeSearch(); 
								break;
							}
						}
					    if (n == 0) {                                           /*  ���������е�֧·���ص����  */
								objectGoTo(GmcCrossway[0].cX, GmcCrossway[0].cY);
								specialmouseTurnback();
								GucMouseTask = SPURT;                               /*  �����󽫿�ʼ���״̬        */
								break;
						}
					}
				}
//-----------------------------------------�������Ѿ�������յ�ķ���;��----------------------------------------------------------	
				else
				{
SearchJump:			mapStepEditS(GucXStart,GucYStart);
					mapStepEditSC();
					mapStepEdit(GucXStart,GucYStart);
					mapStepEditC();
					mapStepEditS(GoalX,GoalY);
					mapStepEdit(GoalX,GoalY);                    //GmcMouse.X  ��  GmcMouse.cX�Ĳ��  2018.12.27 QJS.
					
					ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  ͳ�ƿ�ǰ����֧·��          */
					
					if (ucRoadStat                                           /*  �п�ǰ������                */
						&&(GucMapStepC[shortwayX][shortwayY]+GucMapStepS[shortwayX][shortwayY]<GucMapStep[GucXStart][GucYStart])) 
					{
						if (ucRoadStat > 1) {                                   /*  �ж�����ǰ�����򣬱�������  */
							GmcCrossway[k].cX = GmcMouse.cX;
							GmcCrossway[k].cY = GmcMouse.cY;
							k++;	
						}
						if((GucMapStep[GucXStart][GucYStart]<=stepMax)||(GucMapStep[GucXStart][GucYStart]-GucMapStepS[GucXStart][GucYStart]==0))
						{
							Block_Write();
							mouseTurnback();
							objectGoTo(GucXStart,GucYStart);
							specialmouseTurnback();
							GucMouseTask = SPURT; 
							break;
						}
						crosswayChoice();                                       /*  �����ַ�������ѡ��ǰ������  */
						mazeSearch();                                           /*  ǰ��һ��                    */
					} 
					
					
					
					else
					{	
						if((GucMapStep[GucXStart][GucYStart]<=stepMax)||(GucMapStep[GucXStart][GucYStart]-GucMapStepS[GucXStart][GucYStart]==0))
						{
							Block_Write();
							mouseTurnback();
							objectGoTo(GucXStart,GucYStart);
							specialmouseTurnback();
							GucMouseTask = SPURT; 
							break;
						}
						else
						{
							if((GucMapStep[shortwayX][shortwayY]-GucMapStepS[shortwayX][shortwayY]>0)
								&&(GucMapStepC[shortwayX][shortwayY]+GucMapStepS[shortwayX][shortwayY]<GucMapStep[GucXStart][GucYStart]))
							{
								while (k>n)
								{
									k--;
									ucRoadStat = crosswayCheck(GmcCrossway[k].cX,GmcCrossway[k].cY);
									tempX=GmcCrossway[k].cX;
									tempY=GmcCrossway[k].cY;
									if (ucRoadStat)
									{
										if((GucMapStep[tempX][tempY]-GucMapStepS[tempX][tempY]>0)
											&&(GucMapStepC[tempX][tempY]+GucMapStepS[tempX][tempY]<GucMapStep[GucXStart][GucYStart]))
										{
											
											objectGoTo(GmcCrossway[k].cX,GmcCrossway[k].cY);
											if (ucRoadStat > 1) 
											{
												k++;
											}
											mapStepEdit(GoalX,GoalY);
											backpointwaychoice(tempX,tempY);
											mazeSearch();
  	goto next;                       //goout = 0;
											
										}										
									}
								}
								if (k == 0) {                                           /*  ���������е�֧·���ص����  */
									objectGoTo(GmcCrossway[0].cX, GmcCrossway[0].cY);
									specialmouseTurnback();
									GucMouseTask = SPURT;                               /*  �����󽫿�ʼ���״̬        */
									break;
								}
							}
							while (--n)
							{
								ucRoadStat = crosswayCheck(GmcCrossway[n].cX,GmcCrossway[n].cY);
								tempX=GmcCrossway[n].cX;
								tempY=GmcCrossway[n].cY;
								if (ucRoadStat)
								{
									if(GucMapStepC[tempX][tempY]+GucMapStepS[tempX][tempY]<GucMapStep[GucXStart][GucYStart])
									{
										if((GucMapStep[tempX][tempY]-GucMapStepS[tempX][tempY]>0))
										{
											objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);                      //���������ص���Ľ�
											shortwayX=GmcCrossway[n].cX;
											shortwayY=GmcCrossway[n].cY;
											k=n;
											if (ucRoadStat > 1) 
											{
												k++;
											}
											mapStepEdit(GoalX,GoalY);
											backpointwaychoice(tempX,tempY);
											mazeSearch();
											break;
										}
									}
								}
							}
							if (n == 0)
							{
								objectGoTo(GmcCrossway[0].cX, GmcCrossway[0].cY);
								specialmouseTurnback();
								GucMouseTask = SPURT;                               /*  �����󽫿�ʼ���״̬        */
								break;
							}
						}
						if(stepMax<stepMaxLimit)
						{
							stepMax++;
						}
					}
				}
			}
next:		break;

//=========================================================================================
//=======================================׼�����==========================================
//=========================================================================================	
		case SPURT:
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_BLOCK_POINT);
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_BLOCK);
			flashaddr=0;
			for(blockx=0;blockx<16;blockx++)
			{
				for(blocky=0;blocky<16;blocky++)
				{
					STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK+flashaddr,GucMapBlock[blockx][blocky]);flashaddr+=2;
				}
			}
			flashaddr=0;
			for(blockx=0;blockx<16;blockx++)
			{
				for(blocky=0;blocky<16;blocky++)
				{
					GucMapBlock[blockx][blocky]=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK+flashaddr);flashaddr+=2;
				}
			}	
			SearchOrSpurt_Flag=1;
			flashaddr=0;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,GucXStart);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,GucYStart);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,GoalX);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,GoalY);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,goal_keep_flag);flashaddr+=2;
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,SearchOrSpurt_Flag);flashaddr+=2;
//===========================================================================
//=======================2016��3��18���޸Ķ���===============================
//=======================���յ�ͷ��������================================
			BackSearchFlag+=1;
//===========================================================================			
				
			delay_ms(200);
			if(goaheadnum==1||goaheadnum==3)
			{
				GucMouseTask=SPURT_45;
				spurtmode=LOW;
				break;
			}
			if(goaheadnum==2)
			{
				GucMouseTask=SPURT_90;
				spurtmode=LOW_90;
				break;
			}

//=========================================================================================
//=======================================б�߳��==========================================
//=========================================================================================	
		case SPURT_45:
			sleepmode=1;
//===================================б�߳��·������======================================
			fastway(GoalX,GoalY);
			flashaddr=0;
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_GOPATH);
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH+flashaddr,FastwayCount);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)        //��ʱ����
			{
				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH+flashaddr,FastwayAction[tmptt].action);flashaddr+=2;
			}
			
			GmcMouse.cX=GoalX;
			GmcMouse.cY=GoalY;
			GucMouseDir=(getfinaldir()/2+2)%4;
			
// 			fastway(GucXStart,GucYStart);
// 			flashaddr=0;
// 			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH+flashaddr,FastwayCount);flashaddr+=2;
// 			for(tmptt=0;tmptt<200;tmptt++)
// 			{
// 				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH+flashaddr,FastwayAction[tmptt]);flashaddr+=2;
// 			}		

//===================================��ֱ����·������======================================	
			fastway_90(GucXStart,GucYStart);
			flashaddr=0;
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_BACKPATH_90);
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr,FastwayCount_90);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr,FastwayAction_90[tmptt].action);flashaddr+=2;
			}
//=====================================б�߳�̿�ʼ========================================
			sleepmode=0;		
		 	taskturn(POINTL90);
			tick=0;
			turnaround_tick=0;
			task=KEEPWALLDISTANCE;
		 	while(tick<50);
			taskpointturnright();
			flashaddr=0;
			FastwayCount=STMFLASH_ReadHalfWord(FLASH_ADDR_GOPATH+flashaddr);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				FastwayAction[tmptt].action=STMFLASH_ReadHalfWord(FLASH_ADDR_GOPATH+flashaddr);flashaddr+=2;
			}
			spurt_45_end_flag=1;
			spurt(spurtmode);
			spurt_45_end_flag=0;
			ledon(5);ledon(6);ledon(7);ledon(8);
			if(goal_keep_flag!=0)
			{
				if(goal_keep_flag==1)
				{
					taskturn(POINTL90);
					tick=0;
					turnaround_tick=0;
					task=KEEPWALLDISTANCE;
					while(tick<50);
					taskpointturnleft();
				}
				else if(goal_keep_flag==2)
				{
					taskturn(POINTR90);
					tick=0;
					turnaround_tick=0;
					task=KEEPWALLDISTANCE;
					while(tick<50);
					taskpointturnright();
				}	
			}
//===========================================================================
//=======================2016��3��18���޸Ķ���===============================
//=======================���յ�ͷ��������================================
			if(BackSearchFlag==1)
			{
			ledoff(0);
			accdistance=232;
			taskacc();
			accdistance=40;
			GmcMouse.cX=GoalX;
			GmcMouse.cY=GoalY;
			settrack(0);
			__mouseCoorUpdate();
			BackSearchFlag+=1;
			GucMouseTask = MAZESEARCH;                               /*  �����󽫿�ʼ���״̬        */
			goto SearchJump;
			}
///===========================================================================	
				
// 			flashaddr=0;
// 			FastwayCount=STMFLASH_ReadHalfWord(FLASH_ADDR_BACKPATH+flashaddr);flashaddr+=2;
// 			for(tmptt=0;tmptt<200;tmptt++)
// 			{
// 				FastwayAction[tmptt]=STMFLASH_ReadHalfWord(FLASH_ADDR_BACKPATH+flashaddr);flashaddr+=2;
// 			}
// 			spurt(LOW);
// 			taskturnaround();
			
//===================================��ֱ���ؿ�ʼ======================================
			flashaddr=0;
			FastwayCount_90=STMFLASH_ReadHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				FastwayAction_90[tmptt].action=STMFLASH_ReadHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr);flashaddr+=2;
			}
			GoOrBackFlag=1;
			back90_flag=1;
			spurt_90(LOW_90);
			back90_flag=0;
			GoOrBackFlag=0;
			spurt_back_mouseTurnback();
			
//			fastway(GoalX,GoalY);
//			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH+flashaddr,FastwayCount);flashaddr+=2;
//			for(tmptt=0;tmptt<200;tmptt++)
//			{
//				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH+flashaddr,FastwayAction[tmptt]);flashaddr+=2;
//			}

//		 	taskturn(POINTL90);
//			tick=0;
//			turnaround_tick=0;
//			task=KEEPWALLDISTANCE;
//		 	while(tick<50);
//			taskpointturnright();

//			spurt(LOW);

//			if(righthaswall==1)
//			{
//			 	taskturn(POINTR90);
//				tick=0;
//				turnaround_tick=0;
//				task=KEEPWALLDISTANCE;
//			 	while(tick<50);
//				taskpointturnright();
//			}
//			else if(lefthaswall==1)
//			{
//			 	taskturn(POINTL90);
//				tick=0;
//				turnaround_tick=0;
//				task=KEEPWALLDISTANCE;
//			 	while(tick<50);
//				taskpointturnleft();
//			}
//			else
//			{
//			 	taskturnaround();
//			}
//			
//			GmcMouse.cX=GoalX;
//			GmcMouse.cY=GoalY;
//			GucMouseDir=(getfinaldir()/2+2)%4;//?????????(4?)
//			fastway(GucXStart,GucYStart);
//			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH+flashaddr,FastwayCount);flashaddr+=2;
//			for(tmptt=0;tmptt<200;tmptt++)
//			{
//				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH+flashaddr,FastwayAction[tmptt]);flashaddr+=2;
//			}
//			spurt(LOW);
//			taskturnaround();

// 			GucMouseTask=SPURT_90;spurtmode=LOW_90;break;

// 				if(spurt_time==1&&spurt_time_1==0) {
// 				GmcMouse.cX=GucXStart;
// 				GmcMouse.cY=GucYStart;
//				GucMouseDir=0;
// 				GucMouseTask=SPURT;spurt_time_1=1;break;	}   goto
				
			if(goaheadnum==1||goaheadnum==2)
			{
				goaheadnum=3;//����б�߳�
				LTopSpeed=240;
				L45TopSpeed=180;
				BackTopSpeed=240;
				ledon(5);ledon(6);ledon(7);ledon(8); delay_ms(500);
				ledoff(0);
				GmcMouse.cX=GucXStart;
				GmcMouse.cY=GucYStart;
				GucMouseDir=0;
				GucMouseTask=SPURT_45;
			}
			if(goaheadnum==3)
			{
				goaheadnum=4;//������б�߳�
				LTopSpeed=240;
				L45TopSpeed=180;
				BackTopSpeed=240;
				ledon(5);ledon(6);ledon(7);ledon(8); delay_ms(500);
				ledoff(0);
				GmcMouse.cX=GucXStart;
				GmcMouse.cY=GucYStart;
				GucMouseDir=0;
				GucMouseTask=SPURT_45;
			}
			else if(goaheadnum==4||goaheadnum==0) 
			while(1)
			{{
				sleepmode=1;
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(1000);c++;
			}}
			break;
				
//=========================================================================================
//=======================================��ֱ���==========================================
//=========================================================================================	
		case SPURT_90:
//===================================��ֱ���·������======================================			
			fastway_90(GoalX,GoalY);
			flashaddr=0;
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_GOPATH_90);
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH_90+flashaddr,FastwayCount_90);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH_90+flashaddr,FastwayAction_90[tmptt].action);flashaddr+=2;
			}

			GmcMouse.cX=GoalX;
			GmcMouse.cY=GoalY;
			GucMouseDir=(getfinaldir_90()+2)%4;

//===================================��ֱ����·������======================================	
			fastway_90(GucXStart,GucYStart);

			flashaddr=0;
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_BACKPATH_90);
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr,FastwayCount_90);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr,FastwayAction_90[tmptt].action);flashaddr+=2;
			}			

		 	taskturn(POINTL90);
			tick=0;
			turnaround_tick=0;
			task=KEEPWALLDISTANCE;
		 	while(tick<50);
			taskpointturnright();

			flashaddr=0;
			FastwayCount_90=STMFLASH_ReadHalfWord(FLASH_ADDR_GOPATH_90+flashaddr);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				FastwayAction_90[tmptt].action=STMFLASH_ReadHalfWord(FLASH_ADDR_GOPATH_90+flashaddr);flashaddr+=2;
			}
//--------------------��ֱ��̷���·����������-------------------------------
////=============================2015����===========================
//sleepmode=0;
////================================================================		
			//GoOrBackFlag=1;
			spurt_45_end_flag=1;
			spurt_90(LOW_90);
			spurt_45_end_flag=0;

			if(goal_keep_flag!=0)
			{
				if(goal_keep_flag==1)
				{
					taskturn(POINTL90);
					tick=0;
					turnaround_tick=0;
					task=KEEPWALLDISTANCE;
					while(tick<50);
					taskpointturnleft();
				}
				else if(goal_keep_flag==2)
					{
						taskturn(POINTR90);
						tick=0;
						turnaround_tick=0;
						task=KEEPWALLDISTANCE;
						while(tick<50);
						taskpointturnright();
					}
			}
//===========================================================================
//=======================2016��3��18���޸Ķ���===============================
//=======================���յ�ͷ��������================================
			if(BackSearchFlag==1)
			{
				ledoff(0);
			accdistance=232;
			taskacc();
			accdistance=40;
			GmcMouse.cX=GoalX;
			GmcMouse.cY=GoalY;
			settrack(0);
			__mouseCoorUpdate();
			BackSearchFlag+=1;
			GucMouseTask = MAZESEARCH;                               /*  �����󽫿�ʼ���״̬        */
			goto SearchJump;
			}
///===========================================================================	
				
			flashaddr=0;
			FastwayCount_90=STMFLASH_ReadHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				FastwayAction_90[tmptt].action=STMFLASH_ReadHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr);flashaddr+=2;
			}
			GoOrBackFlag=1;
			back90_flag=1;
			spurt_90(LOW_90);
			back90_flag=0;
			GoOrBackFlag=0;
 			spurt_back_mouseTurnback();

//			fastway(GoalX,GoalY);
//			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH+flashaddr,FastwayCount);flashaddr+=2;
//			for(tmptt=0;tmptt<200;tmptt++)
//			{
//				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH+flashaddr,FastwayAction[tmptt]);flashaddr+=2;
//			}

//		 	taskturn(POINTL90);
//			tick=0;
//			turnaround_tick=0;
//			task=KEEPWALLDISTANCE;
//		 	while(tick<50);
//			taskpointturnright();

//			spurt(LOW);

//			if(righthaswall==1)
//			{
//			 	taskturn(POINTR90);
//				tick=0;
//				turnaround_tick=0;
//				task=KEEPWALLDISTANCE;
//			 	while(tick<50);
//				taskpointturnright();
//			}
//			else if(lefthaswall==1)
//			{
//			 	taskturn(POINTL90);
//				tick=0;
//				turnaround_tick=0;
//				task=KEEPWALLDISTANCE;
//			 	while(tick<50);
//				taskpointturnleft();
//			}
//			else
//			{
//			 	taskturnaround();
//			}
//			
//			GmcMouse.cX=GoalX;
//			GmcMouse.cY=GoalY;
//			GucMouseDir=(getfinaldir()/2+2)%4;//?????????(4?)
//			fastway(GucXStart,GucYStart);
//			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH+flashaddr,FastwayCount);flashaddr+=2;
//			for(tmptt=0;tmptt<200;tmptt++)
//			{
//				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH+flashaddr,FastwayAction[tmptt]);flashaddr+=2;
//			}
//			spurt(LOW);
//			taskturnaround();
			
// 			GucMouseTask=SPURT_45;spurtmode=LOW;break;

// 				if(spurt_time==1&&spurt_time_1==0) {
// 				GmcMouse.cX=GucXStart;
// 				GmcMouse.cY=GucYStart;
// 				GucMouseDir=0;
// 				GucMouseTask=SPURT;spurt_time_1=1;break;	}
			if(goaheadnum==2)
			{
				goaheadnum=1;//����б�߳�
				LTopSpeed=160;
				L45TopSpeed=120;
				BackTopSpeed=180;
				ledon(5);ledon(6);ledon(7);ledon(8); delay_ms(500);
				ledoff(0);
				GmcMouse.cX=GucXStart;
				GmcMouse.cY=GucYStart;
				GucMouseDir = 0;
				spurtmode=LOW;//!!!!!!!!!!!!!!!!!!!!!!
				GucMouseTask=SPURT_45;
			}
			else if(goaheadnum==0) 	
			{while(1)
				{
					sleepmode=1;//???????????,????
					if(c==9||c==0)c=1;
					ledoff(0);ledon(c);delay_ms(1000);c++;
				}}
			break;

        default:
            break;
        }  //switch ѭ��
 
   }
}  //while(1) ������ѭ��
