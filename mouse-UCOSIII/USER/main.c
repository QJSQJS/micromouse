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
//测试用变量

//cjm
int testi = 0;
int goout = 1;
extern uchar spurtTestAction[200];//200!一定要改回来！
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
u8 BackSearchFlag=3;//区分有没有冲过，用于第一次冲完继续搜索用。
extern long angle;
//==================================================================================
//========================需要测试冲刺时修改以下=1，并导入block数据==================================
extern u8 spurtTestFlag;
//==================================================================================
//==================================================================================
////////////以上为测试用////////////  
u8 blockx,blocky;//FLASH读写block操作用
u8 SearchOrSpurt_Flag=0;

u8 GoOrBackFlag=0;
uchar n          = 0;                                               /*  GmcCrossway[]下标           */
extern u8 goal_keep_flag;//1为左  ，2为右

extern uchar	Mapfastway[MAZETYPE][MAZETYPE]      ;
extern FASTWAYACTION	FastwayAction[MAZETYPE * MAZETYPE];              /*  在mapStepEdit()中作堆栈使用 */
extern uchar	FastwayCount;

extern uchar	Mapfastway_90[MAZETYPE][MAZETYPE]      ;
extern FASTWAYACTION	FastwayAction_90[MAZETYPE * MAZETYPE];              /*  在mapStepEdit()中作堆栈使用 */
extern uchar	FastwayCount_90;

extern uchar	actionsequence[];

extern uchar    GucXStart;                /*  起点横坐标                  */
extern uchar    GucYStart;                /*  起点纵坐标                  */

extern uchar    GucXGoal0;            /*  终点X坐标，有两个值         */
extern uchar    GucXGoal1;
extern uchar    GucYGoal0;            /*  终点Y坐标，有两个值         */
extern uchar    GucYGoal1;

extern uchar    GucMouseTask;             /*  状态机，初始状态为等待      */

extern char GoalX;
extern char GoalY;

extern uchar    GucMapStep[MAZETYPE][MAZETYPE];           /*  保存各坐标的等高值          */
extern uchar    GucMapStepS[MAZETYPE][MAZETYPE];           /*  保存各坐标的等高值          */
extern uchar    GucMapStepC[MAZETYPE][MAZETYPE];           /*  保存各坐标的等高值  QJS:我认为是对GucMapStep的拷贝 */  
extern uchar    GucMapStepSC[MAZETYPE][MAZETYPE];           /*  保存各坐标的等高值          */

extern MAZECOOR GmcStack[MAZETYPE * MAZETYPE];              /*  在mapStepEdit()中作堆栈使用 */
extern MAZECOOR GmcStackS1[MAZETYPE * MAZETYPE]; 
extern MAZECOOR GmcStackS2[MAZETYPE * MAZETYPE]; 
extern MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE];              /*  Main()中暂存未走过支路坐标  */

extern MAZECOOR GmcMouse;                                               /*  GmcMouse.x :电脑鼠横坐标    */
                                                                        /*  GmcMouse.y :电脑鼠纵坐标    */
                                                                        
extern uchar    GucMouseDir;                                            /*  电脑鼠的前进方向            */
extern uchar    GucMapBlock[MAZETYPE][MAZETYPE];                        /*  GucMapBlock[x][y]           */
                                                                        /*  x,横坐标;y,纵坐标;          */
                                                                       /*  bit3~bit0分别代表左下右上   */
                                                                       /*  0:该方向无路，1:该方向有路  */
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
extern u8 kalmanswitch;      //卡尔曼滤波  QJS

extern u16 dataf1[240],dataf2[240],datal[220],datar[220];
extern int rawf1,rawf2,rawl,rawr,rawf1h,rawf2h,rawlh,rawrh;
int rawf1avrgnear,rawf2avrgnear,rawlavrgnear,rawravrgnear,rawf1avrgfar,rawf2avrgfar,rawlavrgfar,rawravrgfar;
int iii;
extern int forwardirdis;
int showbattery=0;
extern int LTopSpeed,L45TopSpeed,L90TopSpeed,BackTopSpeed;
//=========================================================================================
u16 flashaddr=0,flashaddrmax,flashreadtemp;//flash存储 当前相对地址（必须是偶数）；最大相对地址
u8 flashwrite=1,flashenable=0;
int irselftestdiff;
//=========================================================================================
//=========================================================================================
u8 c=0;    //key
u16 tmptt;
//=========================================================================================
u8 spurtmode;                /*  预处理  冲刺模式 有LOW  LOW_90 两种选择  QJS*/
//=========================================================================================

uchar ucRoadStat = 0;                                               /*  统计某一坐标可前进的支路数  */
	uchar ucTemp     = 0;
	uchar m=0;
	//uchar l=0;
 //	uchar k=0;
	
//	u16 SpurtTest_Test=0;   //无用变量 QJS
	
	uchar shortwayX=0;
	uchar shortwayY=0;
//	uchar shortwayN=0;      //无用变量 QJS
	
	uchar GoalGet=0;
	uchar stepMax=12;
	uchar stepMaxLimit=12;
	
//	uchar dstep=2;
//	uchar dstepS=2;
	
//	uchar tempBlock;

	
	//临时数据
	char tempX=0;
	char tempY=0;
	
	char tempCX1,tempCX2,tempCY1,tempCY2,gox,goy;

	//底层的，和上层无关
	
	u32 ft=0;
	
	long flashtemp;

//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		512
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

#define KEY_TASK_PRIO		4
//任务堆栈大小	
#define KEY_STK_SIZE 		128
//任务控制块
OS_TCB KeyTaskTCB;
//任务堆栈	
CPU_STK KEY_TASK_STK[KEY_STK_SIZE];
void key_task(void *p_arg);



//任务优先级
#define PREDO_TASK_PRIO		6
//任务堆栈大小
#define PREDO_STK_SIZE		128
//任务控制块
OS_TCB	PredoTaskTCB;
//任务堆栈
CPU_STK	PREDO_TASK_STK[PREDO_STK_SIZE];
//任务函数
void predo_task(void *p_arg);	
	

//任务优先级
#define MAZE_TASK_PRIO		7
//任务堆栈大小
#define MAZE_STK_SIZE		128
//任务控制块
OS_TCB	MazeTaskTCB;
//任务堆栈
CPU_STK	MAZE_TASK_STK[MAZE_STK_SIZE];
//任务函数
void maze_task(void *p_arg);
	
void hardware_init();
void mouseInit();

//按键信号量
OS_SEM BACKGROUND_SEM;
OS_SEM BEEP_SEM;
OS_SEM LED_SEM;

int main()                                          //主程序
 { //int i=0,j=0;
	OS_ERR err;
	CPU_SR_ALLOC();
	//delay_init(168);  	//时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组配置	 
	hardware_init();            /*  硬件初始化                         */
	mouseInit();               /*  墙壁资料内存块初始化                */
	
	//freemode=1;
	
	//启动前若发现已存有红外测距数据，则用存储的数据覆盖原始数组
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


	//原key代码段位置QJS
	//原预处理代码段位置QJS
   //原maze代码位置QJS
//=========================================================================================
//======================================主程序开始=========================================
//=========================================================================================	
	
	OSInit(&err);		//初始化UCOSIII
	OS_CRITICAL_ENTER();//进入临界区
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);  //开启UCOSIII

        
}
		
//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//进入临界区
	//创建背景切换信号量
	OSSemCreate((OS_SEM* ) &BACKGROUND_SEM,
				(CPU_CHAR* )"background_sem",
				(OS_SEM_CTR)0,
				(OS_ERR*   )&err);
	//创建蜂鸣器开关信号量
	OSSemCreate((OS_SEM* ) &BEEP_SEM,
				(CPU_CHAR* )"beep_sem",
				(OS_SEM_CTR)0,
				(OS_ERR*   )&err);
	//创建流水灯开关信号量
	OSSemCreate((OS_SEM* ) &LED_SEM,
				(CPU_CHAR* )"led_sem",
				(OS_SEM_CTR)0,
				(OS_ERR*   )&err);
	//创建按键扫描任务
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
	//创建蜂鸣器开关任务
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
	//创建LCD显示任务
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

	
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
	OS_CRITICAL_EXIT();	//退出临界区
}


void key_task(void *p_arg)
{
	u16 keydelay=0,ispressed=0;//key_task
    u8 keyfunc=0,keytmp=0;
	fuction_Choose:
	ledoff(0);ledon(6);
	while(1)				//延时等待以及按键功能选择
	{
		if(BUTTON1==0 && ispressed==0)					//未被按下 延时等待 倒计时过后跳过功能选择	
		{
			delay_ms(5);
			keydelay++;
			if(keydelay==50){ledoff(0);ledon(5);}
			if(keydelay==100){ledoff(0);ledon(4);}
			if(keydelay==150){ledoff(0);ledon(3);}
			if(keydelay==200){ledoff(0);break;}
		}
		if(BUTTON1)		//键被按下，停止倒计时。
		{
			freemode=1;		//只要按键就去掉电机闭环，因为有可能要进行陀螺仪标定，所以还不能完全进入sleepmode
			if(ispressed==0){ledoff(0);ispressed=1;}	//灭掉按键时延时等待亮着的灯。
			delay_ms(5);	//消抖
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
	while(BUTTON1);		//确保功能选择时候长按按键已经松手，防止被误认为在功能中按键。
	
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
//				//sleepmode=1;//读取电量时若关闭功能，则影响真实电量
//				ledoff(0);
//				delay_ms(200);
//				ledon(battarypercent);
//				delay_ms(200);
//			}
//================================输出FLASH保存的电脑鼠运行数据============================
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
				sleepmode=1;//关闭时钟周期的全部函数，节约电量
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(500);c++;
			}
//=========================================================================================
		case 2:				//擦除红外校准数据
//=========================================================================================
		freemode = 1;	
		flashaddr=0;
			for(tmptt=0;tmptt<8;tmptt++)
			{
				STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_IRDATA+flashaddr);flashaddr+=2;
			}
			while(1)
			{
				sleepmode=1;//关闭时钟周期的全部函数，节约电量
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(1000);c++;
			}
//=========================================================================================
		case 3:				//自动校准陀螺仪零点
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

				if(angle_counter>1500)break;//持续1秒内角度 不变认为已经调零完毕
			}
			gyro_adj=0;
			STMFLASH_Unlock();
			STMFLASH_ErasePage(FLASH_ADDR_GYRO0);
			STMFLASH_Unlock();
			STMFLASH_WriteHalfWord(FLASH_ADDR_GYRO0,gyro0);//写入矫正过的陀螺仪零点电压
			sleepmode=1;//关闭时钟周期的全部函数，节约电量
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
		case 4:				//红外线自检
//=========================================================================================
			freemode=1;
		
		while(1)
			{
				irselftestdiff=disl-dis2wall+4;
				if(irselftestdiff>8) irselftestdiff=8;
				if(irselftestdiff<1) irselftestdiff=1;
				ledoff(0);
				ledon(irselftestdiff);
				if(BUTTON1==1)//键被按下，停止倒计时。
				{
					delay_ms(5);//消抖
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
				if(BUTTON1==1)//键被按下，停止倒计时。
				{
					delay_ms(5);//消抖
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
		case 5:				//显示传感器数据
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
				sleepmode=1;//关闭时钟周期的全部函数，节约电量
				
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
		case 6:				//计算红外线传感器数据
//=========================================================================================
		//传感器到墙的距离：左右31mm和103mm，前面30mm和210mm
		//第一次读取：左边近、右边远；第二次读取：右边近、左边远；第三次读取：前方近；第四次读取：前方远。
		//放置顺序：先靠左，在靠右，然后前，最后是后边
			
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
					sleepmode=1;//关闭时钟周期的全部函数，节约电量
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
					delay_ms(5);//消抖
					if(BUTTON1==1)break;
				}					
			}
			ledoff(0);ledon(4);delay_ms(1000);ledoff(0);ledon(3);delay_ms(1000);ledoff(0);ledon(2);delay_ms(1000);ledoff(0);ledon(1);delay_ms(1000);
			rawlavrgnear=rawl;//获取左传感器近墙电压
			rawravrgfar=rawr;//获取右传感器远墙电压
			//===================
			ledoff(0);
			delay_ms(500);
			while(1)
			{
				ledon(7);ledon(8);
				if(BUTTON1==1)
				{
					delay_ms(5);//消抖
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
					delay_ms(5);//消抖
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
					delay_ms(5);//消抖
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
				sleepmode=1;//关闭时钟周期的全部函数，节约电量
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(1000);c++;
			}
			break;
//=========================================================================================
		case 7:				//显示喷射动作和红外校准数据
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
				sleepmode=1;//关闭时钟周期的全部函数，节约电量
				if(c==9||c==0)c=1;
				ledoff(0);ledon(c);delay_ms(1000);c++;
			}
//=========================================================================================
		case 8:				//清除冲刺动作
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
				sleepmode=1;//关闭时钟周期的全部函数，节约电量
				ledoff(0);ledon(c);delay_ms(1000);c++;
				if(c==9) c=1;
			}		   
		default:
			while(1)
			{
				sleepmode=1;//关闭时钟周期的全部函数，节约电量
				ledoff(0);ledon(c);delay_ms(1000);c++;
				if(c==9) c=1;
			}
	}
}



//=========================================================================================
void mouseInit()                                   //等高值初始化 QJS
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
void hardware_init()                               //硬件初始化 QJS
{
	Stm32_Clock_Init(9);	//系统时钟设置, 9倍频, 系统工作在8M*9=72MHz
	delay_init();			//延时初始化
	uart_init(72,115200);	//串口初始化 
	Adc_Init();				//ADC初始化要在delay之前，为延迟提供时间
	PWM_motor_Init();
	Led_Init();
	Timer1_Init();
	Timer2_Init();
	Timer3_Init();
	gyro0=STMFLASH_ReadHalfWord(FLASH_ADDR_GYRO0);		//读取存在flash中的陀螺仪零点电压值
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

			if(angle_counter>1000)break;				//持续1秒内角度不变认为已经调零完毕
		}
		gyro_adj=0;
		STMFLASH_Unlock();
		STMFLASH_ErasePage(FLASH_ADDR_GYRO0);
		STMFLASH_Unlock();
		STMFLASH_WriteHalfWord(FLASH_ADDR_GYRO0,gyro0);	//写入矫正过的陀螺仪零点电压
		sleepmode=1;									//关闭时钟周期的全部函数，节约电量
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
			if(disf1<20)									//斜线冲刺
			{
				GucMouseTask=SPURT_45;
				spurtmode=LOW;
				ledon(1);ledon(2); delay_ms(1000);
				ledoff(0); delay_ms(500);
				while(1)
				{
					if(disf1<20)							//低速
					{
						LTopSpeed=200;
						L45TopSpeed=160;
						BackTopSpeed=200;
						ledon(1);ledon(2);ledon(3);ledon(4); delay_ms(1000);
						ledoff(0);
						break;
					}
					if(disf2<20)							//高速
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
					if(disf1<20)							//低速
					{
						L90TopSpeed=70;
						BackTopSpeed=70;
						ledon(1);ledon(2);ledon(3);ledon(4); delay_ms(1000);
						ledoff(0);
						break;
					}
					if(disf2<20)							//高速
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
			if(disf2<20){goaheadnum=2; break;}//第一次直线冲刺
			if(disf1<20){goaheadnum=1; break;}//第一次斜线冲刺
		}
		ledon(6);delay_ms(1000);ledoff(0);
	}
	flashenable=0;			//flashwrite默认为1，所以从程序正式运行开始就会写入。只需要在停止写入的地方加一句flashwrite=0;即可
	irswitch=1;
	biaodingmode=0;
	setangle(0);			//启动之前再清零一次陀螺仪
	BackSearchFlag=0;
//=========================================================================================	
//====================================冲刺调试模式=========================================
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
//////////////////////////////////////////预处理结束  finish	 QJS
}

void maze_task(void *p_arg)
{
	while (1)
  {
	uchar l=0;
 	uchar k=0;
	
   switch (GucMouseTask) {                                         /*  状态机处理                  */
            
        case WAIT:
            //delay_ms(1000);                        
			taskacc();settrack(100);
            GucMouseTask = START;
            break;
            
        case START:                                                     /*  判断电脑鼠起点的横坐标      */
	        mazeSearch();                                               /*  向前搜索                    */
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) {         /*  判断电老鼠左边是否存在出口  */
                GucXStart   = MAZETYPE - 1;                             /*  修改电脑鼠起点的横坐标      */
                GmcMouse.cX = MAZETYPE - 1;                             /*  修改电脑鼠当前位置的横坐标  */    
                /*
                 *  由于默认的起点为(0,0)，现在需要把已记录的墙壁资料转换过来
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
                 *  在OFFSHOOT[0]中保存起点坐标
                 */
                GmcCrossway[n].cX = MAZETYPE - 1;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  状态转换为搜寻状态          */
            }
            else if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) {         /*  判断电老鼠右边是否存在出口  */
                /*
                 *  在OFFSHOOT[0]中保存起点坐标
                 */
                GmcCrossway[n].cX = 0;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  状态转换为搜寻状态          */
            }
            break;
			
//=========================================================================================
//=======================================搜索开始==========================================
//=========================================================================================	   
        case MAZESEARCH:
//---------------------------------判断电脑鼠是否在终点------------------------------------		 
			if(((GmcMouse.cX==GucXGoal0)||(GmcMouse.cX==GucXGoal1))&&((GmcMouse.cY==GucYGoal0)||(GmcMouse.cY==GucYGoal1)))
			{
				if((GmcMouse.cX+GmcMouse.cY+GucMouseDir)%2==0) goal_keep_flag=1;  //左矫正
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
				
				Block_Write();             /*  置位 BackSearchFlag  */      
//===========================================================================
//=======================2016年3月18日修改顶层===============================
//=======================到终点就返回起点冲刺================================
			if(BackSearchFlag==0)
			{
				//objectGoTo(GucXStart,GucYStart);
				//返回起点开始
				mouseGoahead(1);
						tick=0;
						turnaround_tick=0;
						task=KEEPWALLDISTANCE;
						while(tick<50) ;                   
										//矫正开始
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
										//矫正结束
				LTopSpeedh=LTopSpeed;L45TopSpeedh=L45TopSpeed;BackTopSpeedh=BackTopSpeed;
				LTopSpeed=70;L45TopSpeed=140;BackTopSpeed=70;
				fastway_90(GucXStart,GucYStart);
				spurt_90(LOW_90);
				LTopSpeed=LTopSpeedh;L45TopSpeed=L45TopSpeedh;BackTopSpeed=BackTopSpeedh;
				//返回起点结束
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
										
										
				GucMouseTask = SPURT;                               /*  电脑鼠将开始冲刺状态        */
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
					while (--n)    /*  n：GmcCrossway[]下标    GmcCrossway[]：Main()中暂存未走过支路坐标  QJS */
					{
						ucRoadStat = crosswayCheck(GmcCrossway[n].cX,GmcCrossway[n].cY);
						tempX=GmcCrossway[n].cX;
						tempY=GmcCrossway[n].cY;
 
						/// ucRoadStat：可前进支路数 	以下程序就是说如果有可以前进的支路咱再通过等高图判断怎么走 QJS
						
						if (ucRoadStat)                    
						{
							if(GucMapStepC[tempX][tempY]+GucMapStepS[tempX][tempY]<GucMapStep[GucXStart][GucYStart])
							{
								objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);                      //！！！返回点待改进
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
						GucMouseTask = SPURT;                               /*  电脑鼠将开始冲刺状态        */
						break;
					}
				}
			}
///-----------------------------------------电脑鼠不在终点-----------------------------------------------------------	
			
			else
			{
//-----------------------------------------电脑鼠未到达过终点----------------------------------------------------------	
				if(GoalGet==0)
				{
					ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  统计可前进的支路数          */
					gox=GmcMouse.cX;
					goy=GmcMouse.cY;
					if (ucRoadStat) {                                           /*  有可前进方向                */
						m=deadwaycheck(n);                     //死区函数返回点
						if(m==n)
						{
							if (ucRoadStat > 1) {                                   /*  有多条可前进方向，保存坐标  */
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
						mazeSearch();                                           /*  前进一格                    */
					} 
					else 
					{                                                    /*  没有可前进方向，回到最近支路*/
						m=deadwaycheck(n);
						if(m<n) n=m+1;
						while (--n) {
							ucRoadStat = crosswayCheck(GmcCrossway[n].cX,
													   GmcCrossway[n].cY);
																				/*  统计最近支点未走过的方向数  */
																				
							/*
							 *  若存在未走过的路，则前往该支点，并跳出循环
							 *  否则继续查找还有未走过的支路。
							 */
							if (ucRoadStat) {
								m=n;
								mapStepEditS(GoalX,GoalY);
								if(m>0)
								{
									tempCX1=GmcCrossway[m].cX;
									tempCY1=GmcCrossway[m].cY;
									l=m;                             //这么写确定没问题吗?   是字母L -l没问题的
									tempCX2=GmcCrossway[m].cX;
									tempCY2=GmcCrossway[m].cY;
									gox=tempCX1;
									goy=tempCY1;
									while(l)                                   //shizimu  L   QJS   不是死循环
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
					    if (n == 0) {                                           /*  走完了所有的支路，回到起点  */
								objectGoTo(GmcCrossway[0].cX, GmcCrossway[0].cY);
								specialmouseTurnback();
								GucMouseTask = SPURT;                               /*  电脑鼠将开始冲刺状态        */
								break;
						}
					}
				}
//-----------------------------------------电脑鼠已经到达过终点的返回途中----------------------------------------------------------	
				else
				{
SearchJump:			mapStepEditS(GucXStart,GucYStart);
					mapStepEditSC();
					mapStepEdit(GucXStart,GucYStart);
					mapStepEditC();
					mapStepEditS(GoalX,GoalY);
					mapStepEdit(GoalX,GoalY);                    //GmcMouse.X  与  GmcMouse.cX的差别  2018.12.27 QJS.
					
					ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  统计可前进的支路数          */
					
					if (ucRoadStat                                           /*  有可前进方向                */
						&&(GucMapStepC[shortwayX][shortwayY]+GucMapStepS[shortwayX][shortwayY]<GucMapStep[GucXStart][GucYStart])) 
					{
						if (ucRoadStat > 1) {                                   /*  有多条可前进方向，保存坐标  */
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
						crosswayChoice();                                       /*  用右手法则搜索选择前进方向  */
						mazeSearch();                                           /*  前进一格                    */
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
								if (k == 0) {                                           /*  走完了所有的支路，回到起点  */
									objectGoTo(GmcCrossway[0].cX, GmcCrossway[0].cY);
									specialmouseTurnback();
									GucMouseTask = SPURT;                               /*  电脑鼠将开始冲刺状态        */
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
											objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);                      //！！！返回点待改进
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
								GucMouseTask = SPURT;                               /*  电脑鼠将开始冲刺状态        */
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
//=======================================准备冲刺==========================================
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
//=======================2016年3月18日修改顶层===============================
//=======================到终点就返回起点冲刺================================
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
//=======================================斜线冲刺==========================================
//=========================================================================================	
		case SPURT_45:
			sleepmode=1;
//===================================斜线冲刺路径制作======================================
			fastway(GoalX,GoalY);
			flashaddr=0;
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_GOPATH);
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_GOPATH+flashaddr,FastwayCount);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)        //暂时变量
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

//===================================垂直返回路径制作======================================	
			fastway_90(GucXStart,GucYStart);
			flashaddr=0;
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_BACKPATH_90);
			STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr,FastwayCount_90);flashaddr+=2;
			for(tmptt=0;tmptt<200;tmptt++)
			{
				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BACKPATH_90+flashaddr,FastwayAction_90[tmptt].action);flashaddr+=2;
			}
//=====================================斜线冲刺开始========================================
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
//=======================2016年3月18日修改顶层===============================
//=======================到终点就返回起点冲刺================================
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
			GucMouseTask = MAZESEARCH;                               /*  电脑鼠将开始冲刺状态        */
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
			
//===================================垂直返回开始======================================
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
				goaheadnum=3;//高速斜线冲
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
				goaheadnum=4;//超高速斜线冲
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
//=======================================垂直冲刺==========================================
//=========================================================================================	
		case SPURT_90:
//===================================垂直冲刺路径制作======================================			
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

//===================================垂直返回路径制作======================================	
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
//--------------------垂直冲刺返回路径制作结束-------------------------------
////=============================2015新增===========================
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
//=======================2016年3月18日修改顶层===============================
//=======================到终点就返回起点冲刺================================
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
			GucMouseTask = MAZESEARCH;                               /*  电脑鼠将开始冲刺状态        */
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
				goaheadnum=1;//低速斜线冲
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
        }  //switch 循环
 
   }
}  //while(1) 主程序循环
