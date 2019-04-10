#ifndef __STRATAGY_H
#define __STRATAGY_H
#include "sys.h"

/*********************************************************************************************************
  常量宏定义 -- 搜索法则
*********************************************************************************************************/
#define RIGHTMETHOD         0
#define LEFTMETHOD          1
#define CENTRALMETHOD       2
#define FRONTRIGHTMETHOD    3
#define FRONTLEFTMETHOD     4


/*********************************************************************************************************
  常量宏定义 -- 电脑鼠运行绝对方向
*********************************************************************************************************/
#define UP                  0
#define RIGHT               1
#define DOWN                2
#define LEFT                3


/*********************************************************************************************************
  常量宏定义 -- 电脑鼠运行相对方向
*********************************************************************************************************/
#define MOUSELEFT           0
#define MOUSEFRONT          1
#define MOUSERIGHT          2


/*********************************************************************************************************
  常量宏定义 -- 用绝对方向来表示相对方向上的资料表位置
*********************************************************************************************************/
#define MOUSEWAY_F          (1 <<   GucMouseDir)
#define MOUSEWAY_R          (1 << ((GucMouseDir + 1) % 4))
#define MOUSEWAY_B          (1 << ((GucMouseDir + 2) % 4))
#define MOUSEWAY_L          (1 << ((GucMouseDir + 3) % 4))


/*********************************************************************************************************
  定义结构体类型
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
*                       Date types(Compiler specific)  数据类型                                         *                 
********************************************************************************************************/
typedef unsigned char  uchar;        // Unsigned  8 bit quantity  无符号 8位整型变量     
typedef unsigned short ushort;       // Unsigned 16 bit quantity  无符号16位整型变量  
typedef unsigned int   uint;         // Unsigned 32 bit quantity  无符号32位整型变量   

#define TRUE  1
#define FALSE 0
#define NULL  0


/*********************************************************************************************************
  常量宏定义 -- 迷宫类型
*********************************************************************************************************/
#define MAZETYPE        16                                               /*  8: 四分之一迷宫；16: 全迷宫 */


/*********************************************************************************************************
  常量宏定义 -- 设定四分之一迷宫的终点坐标
*********************************************************************************************************/
#if MAZETYPE == 8
#define XDST0           6
#define XDST1           7
#define YDST0           6
#define YDST1           7
#endif


/*********************************************************************************************************
  常量宏定义 -- 设定全迷宫的终点坐标
*********************************************************************************************************/
#if MAZETYPE == 16
#define XDST0           7
#define XDST1           8
#define YDST0           7
#define YDST1           8
#endif


/*********************************************************************************************************
  常量宏定义 -- 设置迷宫搜索法则
  可以使用的法则:
  RIGHTMETHOD           -- 右手法则
  LEFTMETHOD            -- 左手法则
  CENTRALMETHOD         -- 中心法则
  FRONTRIGHTMETHOD      -- 中右法则
  FRONTLEFTMETHOD       -- 中左法则
*********************************************************************************************************/
//#define SEARCHMETHOD    CENTRALMETHOD                                     /*  设置迷宫搜索法则为右手法则  */


/*********************************************************************************************************
  常量宏定义--电机运行速度
*********************************************************************************************************/
//#define MAXSPEED        150                                             /*  电机加速运行时的最大速度    */
//#define SEARCHSPEED     68                                              /*  电机搜索迷宫时的最大速度    */


/*********************************************************************************************************
  常量宏定义--前进一个迷宫格步进电机需要走的步数
*********************************************************************************************************/
//#define ONEBLOCK        125     没用到

#define  WAIT           0                                               /*  等待状态                    */
#define  START          1                                               /*  启动状态                    */
#define  MAZESEARCH     2                                               /*  搜寻状态                    */
#define  SPURT          3                                               /*  冲刺状态                    */
#define  SPURT_45       4                                               /*  斜线冲刺状态                */
#define  SPURT_90       5                                               /*  垂直冲刺状态                */



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
