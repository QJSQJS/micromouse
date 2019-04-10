#include "timer.h"
#include "sys.h"
#include "usart.h"	
#include "utility.h"
#include "stratagy.h"
#include "led.h"
#include "stmflash.h"
#include "delay.h"

#define FLASHSAVE
u8 block_write_flag=0;
extern u8 task;
extern u8 goal_keep_flag;
extern u8 blockx,blocky;//FLASH读写block操作用
extern uchar    GucMapBlock[MAZETYPE][MAZETYPE]; 
extern uchar    GucXStart;                /*  起点横坐标                  */
extern uchar    GucYStart;                /*  起点纵坐标                  */
extern  char GoalX;
extern char GoalY;
extern  uchar n;
extern uchar k;
extern MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE];

#ifndef FLASHSAVE
const u16 save_num=1;
const u8 per_timer=2;
#endif
//--------存储运行数据参数------------------------------------------------------------
#ifdef FLASHSAVE
const u16 save_num=1200;
const u8 per_timer=2;
#endif
//--------存储运行数据个数------------------------------------------------------------
extern int flashaddr;
extern MAZECOOR GmcMouse;    
extern long disf1,disf2,disf,disl,disr;
extern long track;
extern int gangle,setgangle;
extern int speedl,speedr; //以cm/s为单位的真实速度	
extern long setvl,setvr;//左右轮设定速度

//解锁STM32的FLASH
void STMFLASH_Unlock(void)
{ FLASH->KEYR=FLASH_KEY1;//写入解锁序列.
  FLASH->KEYR=FLASH_KEY2;
}
//flash上锁
void STMFLASH_Lock(void)
{
  FLASH->CR|=1<<7;//上锁
}
//得到FLASH状态
u8 STMFLASH_GetStatus(void)
{	
	u32 res;		
	res=FLASH->SR; 
	if(res&(1<<0))return 1;		    //忙
	else if(res&(1<<2))return 2;	//编程错误
	else if(res&(1<<4))return 3;	//写保护错误
	return 0;						//操作完成
}
//等待操作完成
//time:要延时的长短
//返回值:状态.
u8 STMFLASH_WaitDone(u16 time)
{
	u8 res;
	do
	{
		res=STMFLASH_GetStatus();
		if(res!=1)break;//非忙,无需等待了,直接退出.
		delay_us(1);
		time--;
	 }while(time);
	 if(time==0)res=0xff;//TIMEOUT
	 return res;
}
//擦除页
//paddr:页地址
//返回值:执行情况
u8 STMFLASH_ErasePage(u32 paddr)
{
	u8 res=0;
	res=STMFLASH_WaitDone(0X5FFF);//等待上次操作结束,>20ms    
	if(res==0)
	{ 
		FLASH->CR|=1<<1;//页擦除
		FLASH->AR=paddr;//设置页地址 
		FLASH->CR|=1<<6;//开始擦除		  
		res=STMFLASH_WaitDone(0X5FFF);//等待操作结束,>20ms  
		if(res!=1)//非忙
		{
			FLASH->CR&=~(1<<1);//清除页擦除标志.
		}
	}
	return res;
}
//在FLASH指定地址写入半字
//faddr:指定地址(此地址必须为2的倍数!!)
//dat:要写入的数据
//返回值:写入的情况
u8 STMFLASH_WriteHalfWord(u32 faddr, int dat)
{
	u8 res;	   	    
	res=STMFLASH_WaitDone(0XFF);
	if(res==0)//OK
	{
		FLASH->CR|=1<<0;//编程使能
		*(vs16*)faddr=dat;//写入数据
		res=STMFLASH_WaitDone(0XFF);//等待操作完成
		if(res!=1)//操作成功
		{
			FLASH->CR&=~(1<<0);//清除PG位
		}
	} 
	return res;
} 
//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
int STMFLASH_ReadHalfWord(u32 faddr)
{	
	return *(vs16*)faddr; 
}
	 			  	
//设置FLASH 保存地址
//必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000
//最大0X0801FFFF，即FLASH存储空间是0X08020000-FLASH_SAVE_ADDR这么多个单元
//必须每次写入16字节（所以下面才都加了强制类型转换），也就是说256个迷宫格子加上算好的路径数组等内容加起来有几项就用几个flash单元，不要浪费
//#define FLASH_SAVE_ADDR  0X08010000 	
				
/*
STM32F103中容量FLASH共128K，分为128页，每页1024字节。FLASH存储要求每个元素占16位即两个字节。所以每页能存512个元素。
把整个FLASH用户存储区分成UART部分和迷宫部分。
代码占用按40K，迷宫占1K，留7K机动，还剩80K用来存储UART数据，能存80*1024/2 = 40960个元素。
如果每个时钟周期（1毫秒）存储4个元素，那么要想持续10秒，则会存储40000个元素。
0-39页留给程序；
40-120页留给UART
126-127页留给迷宫相关

注意，每次写或擦除页之前都要unlock。
*/






// RUN_DATA run_data[1];
RUN_DATA run_data[save_num];

void write_data(u16 num)
{
//	run_data[num].cx=GmcMouse.cX;
//	run_data[num].cy=GmcMouse.cY;
	run_data[num].gangle=gangle;
	run_data[num].task=task;
//	run_data[num].setgangle=setgangle;
//	run_data[num].disf1=disf1;
//	run_data[num].disf2=disf2;
//	run_data[num].disl=disl;
//	run_data[num].disr=disr;
//	run_data[num].setvl=setvl;
//	run_data[num].setvr=setvr;
	run_data[num].speedl=speedl;
	run_data[num].speedr=speedr;
//	run_data[num].track=track;
}

void save_data(u16 num)
{
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].cx);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].cy);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].track);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].disf1);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].disl);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].disr);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].disf2);flashaddr+=2;
	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].task);flashaddr+=2;
	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].gangle);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].setgangle);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].setvl);flashaddr+=2;
//	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].setvr);flashaddr+=2;
	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].speedl);flashaddr+=2;
	STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_DATA+flashaddr,run_data[num].speedr);	
}

u16 printf_line=1;
void print_data ()
{   int data_temp=0,data_temp1=0,repeat_time=0;
		flashaddr=0;
		data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA_NUM+flashaddr);flashaddr+=2;printf("The Time Has Saved:%d\r\n",data_temp);
		data_temp1=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA_NUM+flashaddr);flashaddr+=2;printf("The Number of The Current Time:%d\r\n",data_temp1);
		if(data_temp==1){ repeat_time=data_temp1+1;} else repeat_time=save_num+1;
//		printf("cx,cy,track,disf1,disl,disr,disf2,gangle,setgangle,setvl,setvr,speedl,speedr,\r\n");
		printf("task,track,speedl,speedr,\r\n");
	
	  flashaddr=0;
		printf("task:\r\n");
		for(printf_line=1;printf_line<repeat_time;printf_line++)
		{
			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2*4;printf("%d,",data_temp);
		}
		printf("\r\n");
		
			  flashaddr=2;
		printf("gangle:\r\n");
		for(printf_line=1;printf_line<repeat_time;printf_line++)
		{
			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2*4;printf("%d,",data_temp);
		}
		printf("\r\n");
		
			  flashaddr=4;
		printf("speedl:\r\n");
		for(printf_line=1;printf_line<repeat_time;printf_line++)
		{
			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2*4;printf("%d,",data_temp);
		}
		printf("\r\n");
		
			  flashaddr=6;
		printf("speedr:\r\n");
		for(printf_line=1;printf_line<repeat_time;printf_line++)
		{
			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2*4;printf("%d,",data_temp);
		}
		printf("\r\n");
		
//		{ printf("%d:",printf_line);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("[%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d],",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("[%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d],",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			data_temp=STMFLASH_ReadHalfWord(FLASH_ADDR_DATA+flashaddr);flashaddr+=2;printf("%d,",data_temp);
//			printf("\r\n");	 }
}

void erase_run_data()
{
		for(flashaddr=0;flashaddr<(save_num/100)+1;flashaddr++)
		{STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_DATA+flashaddr*0x400);}
 		STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_DATA_NUM);
}

extern u8 SearchOrSpurt_Flag;
void SaveBlockData()
{
	STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_BLOCK_POINT);
    STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_BLOCK);
			flashaddr=0;
		for(blockx=0;blockx<16;blockx++)
		{for(blocky=0;blocky<16;blocky++)
			{
				STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK+flashaddr,GucMapBlock[blockx][blocky]);flashaddr+=2;
			}}
			
			flashaddr=0;
		for(blockx=0;blockx<16;blockx++)
		{for(blocky=0;blocky<16;blocky++)
			{
				GucMapBlock[blockx][blocky]=STMFLASH_ReadHalfWord(FLASH_ADDR_BLOCK+flashaddr);flashaddr+=2;
			}}	
		flashaddr=0;
    STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,GucXStart);flashaddr+=2;
		STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,GucYStart);flashaddr+=2;
		STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,GoalX);flashaddr+=2;
		STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,GoalY);flashaddr+=2;
		STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,goal_keep_flag);flashaddr+=2;
		STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_POINT+flashaddr,SearchOrSpurt_Flag);flashaddr+=2;

}
		
void SaveSearch_Stack()
		{ u8 temp_num;
			STMFLASH_Unlock();STMFLASH_ErasePage(FLASH_ADDR_BLOCK_STACK);
			flashaddr=0;
		 for(temp_num=0;temp_num<n+1;temp_num++)
			{STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_STACK+flashaddr,GmcCrossway[temp_num].cX);flashaddr+=2;
			 STMFLASH_Unlock();STMFLASH_WriteHalfWord(FLASH_ADDR_BLOCK_STACK+flashaddr,GmcCrossway[temp_num].cY);flashaddr+=2;}
    }
		
void Block_Write()
{
	block_write_flag=1;
}












