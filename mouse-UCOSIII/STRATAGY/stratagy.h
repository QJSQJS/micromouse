#ifndef __STRATAGY_H
#define __STRATAGY_H
#include "sys.h"

/*********************************************************************************************************
  �����궨�� -- ��������
*********************************************************************************************************/
#define RIGHTMETHOD         0
#define LEFTMETHOD          1
#define CENTRALMETHOD       2
#define FRONTRIGHTMETHOD    3
#define FRONTLEFTMETHOD     4


/*********************************************************************************************************
  �����궨�� -- ���������о��Է���
*********************************************************************************************************/
#define UP                  0
#define RIGHT               1
#define DOWN                2
#define LEFT                3


/*********************************************************************************************************
  �����궨�� -- ������������Է���
*********************************************************************************************************/
#define MOUSELEFT           0
#define MOUSEFRONT          1
#define MOUSERIGHT          2


/*********************************************************************************************************
  �����궨�� -- �þ��Է�������ʾ��Է����ϵ����ϱ�λ��
*********************************************************************************************************/
#define MOUSEWAY_F          (1 <<   GucMouseDir)
#define MOUSEWAY_R          (1 << ((GucMouseDir + 1) % 4))
#define MOUSEWAY_B          (1 << ((GucMouseDir + 2) % 4))
#define MOUSEWAY_L          (1 << ((GucMouseDir + 3) % 4))


/*********************************************************************************************************
  ����ṹ������
*********************************************************************************************************/
struct mazecoor
{
	char cX;
	char cY;
};
typedef struct mazecoor MAZECOOR;

struct fastwayaction
{
	char cx;
	char cy;
	char action;
};
typedef struct fastwayaction FASTWAYACTION;

/********************************************************************************************************
*                       Date types(Compiler specific)  ��������                                         *                 
********************************************************************************************************/
typedef unsigned char  uchar;        // Unsigned  8 bit quantity  �޷��� 8λ���ͱ���     
typedef unsigned short ushort;       // Unsigned 16 bit quantity  �޷���16λ���ͱ���  
typedef unsigned int   uint;         // Unsigned 32 bit quantity  �޷���32λ���ͱ���   

#define TRUE  1
#define FALSE 0
#define NULL  0


/*********************************************************************************************************
  �����궨�� -- �Թ�����
*********************************************************************************************************/
#define MAZETYPE        16                                               /*  8: �ķ�֮һ�Թ���16: ȫ�Թ� */


/*********************************************************************************************************
  �����궨�� -- �趨�ķ�֮һ�Թ����յ�����
*********************************************************************************************************/
#if MAZETYPE == 8
#define XDST0           6
#define XDST1           7
#define YDST0           6
#define YDST1           7
#endif


/*********************************************************************************************************
  �����궨�� -- �趨ȫ�Թ����յ�����
*********************************************************************************************************/
#if MAZETYPE == 16
#define XDST0           7
#define XDST1           8
#define YDST0           7
#define YDST1           8
#endif


/*********************************************************************************************************
  �����궨�� -- �����Թ���������
  ����ʹ�õķ���:
  RIGHTMETHOD           -- ���ַ���
  LEFTMETHOD            -- ���ַ���
  CENTRALMETHOD         -- ���ķ���
  FRONTRIGHTMETHOD      -- ���ҷ���
  FRONTLEFTMETHOD       -- ������
*********************************************************************************************************/
//#define SEARCHMETHOD    CENTRALMETHOD                                     /*  �����Թ���������Ϊ���ַ���  */


/*********************************************************************************************************
  �����궨��--��������ٶ�
*********************************************************************************************************/
//#define MAXSPEED        150                                             /*  �����������ʱ������ٶ�    */
//#define SEARCHSPEED     68                                              /*  ��������Թ�ʱ������ٶ�    */


/*********************************************************************************************************
  �����궨��--ǰ��һ���Թ��񲽽������Ҫ�ߵĲ���
*********************************************************************************************************/
//#define ONEBLOCK        125     û�õ�

#define  WAIT           0                                               /*  �ȴ�״̬                    */
#define  START          1                                               /*  ����״̬                    */
#define  MAZESEARCH     2                                               /*  ��Ѱ״̬                    */
#define  SPURT          3                                               /*  ���״̬                    */
#define  SPURT_45       4                                               /*  б�߳��״̬                */
#define  SPURT_90       5                                               /*  ��ֱ���״̬                */



void mapStepEdit (char  cX, char  cY);
void mapStepEditC (void);
void mapStepEditS (char  X, char  Y);
void mapStepEditSC (void);
void mouseSpurt (void);
void objectGoTo (char  cXdst, char  cYdst);
uchar mazeBlockDataGet (uchar  ucDirTemp);
uchar mazeGetDataGet (uchar  ucDirTemp);
void rightMethod (void);
void leftMethod (void);
void frontRightMethod (void);
void frontLeftMethod (void);
void centralMethod (void);
uchar crosswayCheck (char  cX, char  cY);
void crosswayChoice (void);
void goalWallchange(char GoalX,char GoalY);
void backpointwaychoice(char cX,char cY);
uchar sStepGet (uchar  ucDirTemp);
void fastway(uchar GX,uchar GY);

void fastway_90(uchar GX,uchar GY);

uchar deadwaycheck(uchar cross);

void backPath(FASTWAYACTION *FastwayAction, uchar FastwayCount);

#endif
