#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  


//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//STM32 FLASH ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
//�洢���ݽṹ������
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
//�û������Լ�����Ҫ����
#define STM32_FLASH_SIZE 	128 	 		//��ѡSTM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN 	1              	//ʹ��FLASHд��(0��������;1��ʹ��)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ
//FLASH������ֵ
#define FLASH_KEY1               0X45670123
#define FLASH_KEY2               0XCDEF89AB
void STMFLASH_Unlock(void);					  //FLASH����
void STMFLASH_Lock(void);					  //FLASH����
u8 STMFLASH_GetStatus(void);				  //���״̬
u8 STMFLASH_WaitDone(u16 time);				  //�ȴ���������
u8 STMFLASH_ErasePage(u32 paddr);			  //����ҳ
u8 STMFLASH_WriteHalfWord(u32 faddr, int dat);//д�����
int STMFLASH_ReadHalfWord(u32 faddr);		  //��������  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//ָ����ַ��ʼ��ȡָ����������
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(u32 WriteAddr,u16 WriteData);
void SaveBlockData(void);
void SaveSearch_Stack(void);
void Block_Write(void);

//�ҵĵ�ַ�б�
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
���0X0801FFFF
STM32F103������FLASH��128K����Ϊ128ҳ��ÿҳ1024�ֽڡ�FLASH�洢Ҫ��ÿ��Ԫ��ռ16λ�������ֽڡ�����ÿҳ�ܴ�512��Ԫ�ء�
������FLASH�û��洢���ֳ�UART���ֺ��Թ����֡�
����ռ�ð�40K���Թ�ռ1K����7K��������ʣ80K�����洢UART���ݣ��ܴ�80*1024/2 = 40960��Ԫ�ء�
���ÿ��ʱ�����ڣ�1���룩�洢4��Ԫ�أ���ôҪ�����10�룬���洢40000��Ԫ�ء�
0-39ҳ��������
40-120ҳ����UART
126-127ҳ�����Թ����
*/















