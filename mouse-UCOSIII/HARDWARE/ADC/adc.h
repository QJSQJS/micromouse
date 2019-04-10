#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//ADC ��������			   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/7 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved					   
//////////////////////////////////////////////////////////////////////////////////	 
#define F2 	1
#define R  	2
#define L  	3
#define F1  4
#define NONE 5

#define INFR_F2		INFR_RF
#define INFR_F1		INFR_LF
#define ADC_F2  	ADC_RF
#define ADC_F1  	ADC_LF

#define INFR_RF		PCout(6)
#define INFR_R		PBout(15)
#define INFR_L		PBout(13)
#define INFR_LF		PBout(14)

//ADCͨ����
#define ADC_RF 		13 //PC3 13
#define ADC_R  		12 //PC2 12
#define ADC_L  		11 //PC1 11
#define ADC_LF  	10//PC0 10
#define ADC_GYRO  	0  //PB1
#define ADC_BATTARY 1  //PA1
 
void Adc_Init(void);
u16  Get_Adc(u8 ch);  
u16 adc2dis(u16 adc);
void getdistance(void);

u16 FindDis(u16 ADC,u16 *data,u16 Length );

void ifRight(double logV1,double logV2,double logD1,double logD2);
void CurveFit(double Y1,double X1,double Y2,double X2);
void CheckValue(u16 num,u16 infr[],u8 Name);
void InfCorrect(double vol1,double dist1,double vol2,double dist2,u16 num,u16 infr[],u8 Name);//?????vol1 ??dist1,?????vol2,??dist2,????num,????infr

#endif 















