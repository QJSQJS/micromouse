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
extern u8 blockx,blocky;//FLASH��дblock������
extern uchar    GucMapBlock[MAZETYPE][MAZETYPE]; 
extern uchar    GucXStart;                /*  ��������                  */
extern uchar    GucYStart;                /*  ���������                  */
extern  char GoalX;
extern char GoalY;
extern  uchar n;
extern uchar k;
extern MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE];

#ifndef FLASHSAVE
const u16 save_num=1;
const u8 per_timer=2;
#endif
//--------�洢�������ݲ���------------------------------------------------------------
#ifdef FLASHSAVE
const u16 save_num=1200;
const u8 per_timer=2;
#endif
//--------�洢�������ݸ���------------------------------------------------------------
extern int flashaddr;
extern MAZECOOR GmcMouse;    
extern long disf1,disf2,disf,disl,disr;
extern long track;
extern int gangle,setgangle;
extern int speedl,speedr; //��cm/sΪ��λ����ʵ�ٶ�	
extern long setvl,setvr;//�������趨�ٶ�

//����STM32��FLASH
void STMFLASH_Unlock(void)
{ FLASH->KEYR=FLASH_KEY1;//д���������.
  FLASH->KEYR=FLASH_KEY2;
}
//flash����
void STMFLASH_Lock(void)
{
  FLASH->CR|=1<<7;//����
}
//�õ�FLASH״̬
u8 STMFLASH_GetStatus(void)
{	
	u32 res;		
	res=FLASH->SR; 
	if(res&(1<<0))return 1;		    //æ
	else if(res&(1<<2))return 2;	//��̴���
	else if(res&(1<<4))return 3;	//д��������
	return 0;						//�������
}
//�ȴ��������
//time:Ҫ��ʱ�ĳ���
//����ֵ:״̬.
u8 STMFLASH_WaitDone(u16 time)
{
	u8 res;
	do
	{
		res=STMFLASH_GetStatus();
		if(res!=1)break;//��æ,����ȴ���,ֱ���˳�.
		delay_us(1);
		time--;
	 }while(time);
	 if(time==0)res=0xff;//TIMEOUT
	 return res;
}
//����ҳ
//paddr:ҳ��ַ
//����ֵ:ִ�����
u8 STMFLASH_ErasePage(u32 paddr)
{
	u8 res=0;
	res=STMFLASH_WaitDone(0X5FFF);//�ȴ��ϴβ�������,>20ms    
	if(res==0)
	{ 
		FLASH->CR|=1<<1;//ҳ����
		FLASH->AR=paddr;//����ҳ��ַ 
		FLASH->CR|=1<<6;//��ʼ����		  
		res=STMFLASH_WaitDone(0X5FFF);//�ȴ���������,>20ms  
		if(res!=1)//��æ
		{
			FLASH->CR&=~(1<<1);//���ҳ������־.
		}
	}
	return res;
}
//��FLASHָ����ַд�����
//faddr:ָ����ַ(�˵�ַ����Ϊ2�ı���!!)
//dat:Ҫд�������
//����ֵ:д������
u8 STMFLASH_WriteHalfWord(u32 faddr, int dat)
{
	u8 res;	   	    
	res=STMFLASH_WaitDone(0XFF);
	if(res==0)//OK
	{
		FLASH->CR|=1<<0;//���ʹ��
		*(vs16*)faddr=dat;//д������
		res=STMFLASH_WaitDone(0XFF);//�ȴ��������
		if(res!=1)//�����ɹ�
		{
			FLASH->CR&=~(1<<0);//���PGλ
		}
	} 
	return res;
} 
//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
int STMFLASH_ReadHalfWord(u32 faddr)
{	
	return *(vs16*)faddr; 
}
	 			  	
//����FLASH �����ַ
//����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000
//���0X0801FFFF����FLASH�洢�ռ���0X08020000-FLASH_SAVE_ADDR��ô�����Ԫ
//����ÿ��д��16�ֽڣ���������Ŷ�����ǿ������ת������Ҳ����˵256���Թ����Ӽ�����õ�·����������ݼ������м�����ü���flash��Ԫ����Ҫ�˷�
//#define FLASH_SAVE_ADDR  0X08010000 	
				
/*
STM32F103������FLASH��128K����Ϊ128ҳ��ÿҳ1024�ֽڡ�FLASH�洢Ҫ��ÿ��Ԫ��ռ16λ�������ֽڡ�����ÿҳ�ܴ�512��Ԫ�ء�
������FLASH�û��洢���ֳ�UART���ֺ��Թ����֡�
����ռ�ð�40K���Թ�ռ1K����7K��������ʣ80K�����洢UART���ݣ��ܴ�80*1024/2 = 40960��Ԫ�ء�
���ÿ��ʱ�����ڣ�1���룩�洢4��Ԫ�أ���ôҪ�����10�룬���洢40000��Ԫ�ء�
0-39ҳ��������
40-120ҳ����UART
126-127ҳ�����Թ����

ע�⣬ÿ��д�����ҳ֮ǰ��Ҫunlock��
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












