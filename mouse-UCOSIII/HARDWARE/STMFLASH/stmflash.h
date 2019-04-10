#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  


//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//STM32 FLASH 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
//存储数据结构体类型
/*********************************************************************************************************/
struct run_data
{
//	u8 cx,cy;
//	u16 track;
//	u8 disf1,disl,disr,disf2;
	u8 task;
	s16 gangle,speedl,speedr;	
//	s16 setgangle,setvl,setvr,
};
typedef struct run_data RUN_DATA;

void erase_run_data(void);
void write_data(u16 num);
void save_data(u16 num);
void print_data(void);

//////////////////////////////////////////////////////////////////////////////////////////////////////
//用户根据自己的需要设置
#define STM32_FLASH_SIZE 	128 	 		//所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 	1              	//使能FLASH写入(0，不是能;1，使能)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
//FLASH解锁键值
#define FLASH_KEY1               0X45670123
#define FLASH_KEY2               0XCDEF89AB
void STMFLASH_Unlock(void);					  //FLASH解锁
void STMFLASH_Lock(void);					  //FLASH上锁
u8 STMFLASH_GetStatus(void);				  //获得状态
u8 STMFLASH_WaitDone(u16 time);				  //等待操作结束
u8 STMFLASH_ErasePage(u32 paddr);			  //擦除页
u8 STMFLASH_WriteHalfWord(u32 faddr, int dat);//写入半字
int STMFLASH_ReadHalfWord(u32 faddr);		  //读出半字  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//指定地址开始写入指定长度的数据
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//指定地址开始读取指定长度数据
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//从指定地址开始读出指定长度的数据

//测试写入
void Test_Write(u32 WriteAddr,u16 WriteData);
void SaveBlockData(void);
void SaveSearch_Stack(void);
void Block_Write(void);

//我的地址列表
// #define FLASH_ADDR_DATA_NUM  	0X0801B800 //page 110
// #define FLASH_ADDR_DATA   		FLASH_ADDR_UARTMAX+4

#define FLASH_ADDR_DATA_NUM  	0X08016000 //page 88
#define FLASH_ADDR_DATA   		0X08016400 //page 89

#define FLASH_ADDR_BLOCK_STACK   	0X0801D800 //page 118
#define FLASH_ADDR_BLOCK_POINT   	0X0801DC00 //page 119
#define FLASH_ADDR_BLOCK   	0X0801E000 //page 120
#define FLASH_ADDR_GOPATH_90   	0X0801E400 //page 121  
#define FLASH_ADDR_BACKPATH_90   	0X0801E800 //page 122 

#define FLASH_ADDR_IRDATA		0X0801EC00 //page 123
#define FLASH_ADDR_GOPATH   	0X0801F000 //page 124  
#define FLASH_ADDR_BACKPATH   	0X0801F800 //page 126  
#define FLASH_ADDR_GYRO0  		0X0801FC00 //page 127 
								   
#endif

/*
最大0X0801FFFF
STM32F103中容量FLASH共128K，分为128页，每页1024字节。FLASH存储要求每个元素占16位即两个字节。所以每页能存512个元素。
把整个FLASH用户存储区分成UART部分和迷宫部分。
代码占用按40K，迷宫占1K，留7K机动，还剩80K用来存储UART数据，能存80*1024/2 = 40960个元素。
如果每个时钟周期（1毫秒）存储4个元素，那么要想持续10秒，则会存储40000个元素。
0-39页留给程序
40-120页留给UART
126-127页留给迷宫相关
*/















