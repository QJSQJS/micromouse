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
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
//入口参数 ：GmcMouse.x，GmcMouse.y
//作用：通过8个led灯显示电脑鼠坐标
//1、2、3、4灯显示X坐标，5、6、7、8灯显示Y坐标
extern MAZECOOR GmcMouse;
extern uchar    GucMapBlock[MAZETYPE][MAZETYPE];
extern u8 blockx,blocky;//FLASH读写block操作用
extern  uchar    GucXStart;                /*  起点横坐标                  */
extern char GoalX;
extern char GoalY;


void ShowMouseXY()
{
	ledoff(0);
	switch(GmcMouse.cX)
	{
		 case 0:
			 break;
		 case 1:
			 ledon(4);break;
		 case 2:
			 ledon(3);break;
		 case 3:
			 ledon(3);ledon(4);break;
		 case 4:
			 ledon(2);break;
		 case 5:
			 ledon(2);ledon(4);break;
		 case 6:
			 ledon(2);ledon(3);break;
		 case 7:
			 ledon(2);ledon(3);ledon(4);break;
		 case 8:
			 ledon(1);break;
		 case 9:
			 ledon(1);ledon(4);break;
		 case 10:
			 ledon(1);ledon(3);break;
		 case 11:
			 ledon(1);ledon(3);ledon(4);break;
		 case 12:
			 ledon(1);ledon(2);break;
		 case 13:
			 ledon(1);ledon(2);ledon(4);break;
		 case 14:
			 ledon(1);ledon(2);ledon(3);break;
		 case 15:
			 ledon(1);ledon(2);ledon(3);ledon(4);break;
		 default:break;		 
  }
	switch(GmcMouse.cY)
	{
		 case 0:
			 break;
		 case 1:
			 ledon(8);break;
		 case 2:
			 ledon(7);break;
		 case 3:
			 ledon(7);ledon(8);break;
		 case 4:
			 ledon(6);break;
		 case 5:
			 ledon(6);ledon(8);break;
		 case 6:
			 ledon(6);ledon(7);break;
		 case 7:
			 ledon(6);ledon(7);ledon(8);break;
		 case 8:
			 ledon(5);break;
		 case 9:
			 ledon(5);ledon(8);break;
		 case 10:
			 ledon(5);ledon(7);break;
		 case 11:
			 ledon(5);ledon(7);ledon(8);break;
		 case 12:
			 ledon(5);ledon(6);break;
		 case 13:
			 ledon(5);ledon(6);ledon(8);break;
		 case 14:
			 ledon(5);ledon(6);ledon(7);break;
		 case 15:
			 ledon(5);ledon(6);ledon(7);ledon(8);break;
		 default:break;		 
  }
	
	
}


void Led_Init()
{
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟 
	GPIOA->CRH&=0XFFFFFFF0; 
	GPIOA->CRH|=0X00000003;//PA8推挽输出 

	//RCC->APB2ENR|=1<<3;	   //PORTB时钟使能
	//GPIOB->CRL&=0XFFFFFFF0;
	//GPIOB->CRL|=0X00000003;//PB0推挽输出
	//GPIOB->CRH&=0XFFFF00FF;
	//GPIOB->CRH|=0X00003300;//PB10,11推挽输出

 	RCC->APB2ENR|=1<<4;    //使能PORTC口时钟 
	GPIOC->CRL&=0XFFFFFFFF;
	GPIOC->CRL|=0X30000000;//PC7推挽输出 
	GPIOC->CRH&=0XFFFFFF00;
	GPIOC->CRH|=0X00000033;//PC8,9推挽输出 
	
	GPIOC->CRH&=0XFFFFF0FF; 
	GPIOC->CRH|=0X00000800;//PC10输入 

	ledoff(0);
}

void ledon(u8 n)
{
	switch(n)
	{
	 	case 1:LED1=0;break;
	 	case 2:LED2=0;break;
	 	case 3:LED3=0;break;
	 	case 4:LED4=0;break;
	 	case 5:LED5=0;break;
	 	case 6:LED6=0;break;
	 	case 7:LED7=0;break;
	 	case 8:LED8=0;break;
	 	case 0:LED1=0;LED2=0;LED3=0;LED4=0;LED5=0;LED6=0;LED7=0;LED8=0;break;
		default:break;
	}
}

void ledoff(u8 n)
{
	switch(n)
	{
	 	case 1:LED1=1;break;
	 	case 2:LED2=1;break;
	 	case 3:LED3=1;break;
	 	case 4:LED4=1;break;
	 	case 5:LED5=1;break;
	 	case 6:LED6=1;break;
	 	case 7:LED7=1;break;
	 	case 8:LED8=1;break;
	 	case 0:LED1=1;LED2=1;LED3=1;LED4=1;LED5=1;LED6=1;LED7=1;LED8=1;break;
		default:break;
	}
}


// +---+---+
// |   | + |
// +---+---+
// GucMapBlock,blockx,blocky,
void Show_Block()
{
	blockx=0;blocky=0;
	blocky=15;printf("+");
	for(blockx=0;blockx<16;blockx++) {if(GucMapBlock[blockx][blocky] & 0x01) printf("   +"); else printf("---+");} printf("\r\n");
	

	for(blocky=16;blocky>0;)
	{blocky--;
	 if(GucMapBlock[0][blocky] & 0x08) printf(" "); else printf("|");
	 for(blockx=0;blockx<16;blockx++) 
			{if(((blockx==GucXStart)&&(blocky==0))||((blockx==7||blockx==8)&&(blocky==7||blocky==8)))
			{if((blockx==GoalX)&&(blocky==GoalY)){if(GucMapBlock[blockx][blocky] & 0x02) printf("%d,%d ",blockx,blocky); else  printf("%d,%d|",blockx,blocky); }
       else if(GucMapBlock[blockx][blocky] & 0x02) printf(" $  "); else printf(" $ |");}  
			else if(GucMapBlock[blockx][blocky] & 0x02)  printf("    "); else printf("   |");} printf("\r\n");
		printf("+");
		for(blockx=0;blockx<16;blockx++) {if(GucMapBlock[blockx][blocky] & 0x04)  printf("   +"); else printf("---+");} printf("\r\n");}
   printf("(%d,0) Is The Start_Point",GucXStart);printf("\r\n");
}




