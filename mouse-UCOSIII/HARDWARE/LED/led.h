#ifndef __LED_H
#define __LED_H	
//////////////////////////////////////////////////////////////////////////////////	 
#define LED1		PAout(8)
#define LED2		PCout(9)
#define LED3		PCout(8)
#define LED4		PCout(7)
#define LED5		PAout(8)
#define LED6		PCout(9)
#define LED7		PCout(8)
#define LED8		PCout(7)

#define BUTTON1		PCin(10)

void Led_Init(void);
void ledon(u8 n);
void ledoff(u8 n);
void ShowMouseXY(void);
void Show_Block(void);
#endif 















