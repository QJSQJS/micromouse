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

//==================================================================================
//========================需要测试冲刺时修改以下三处================================
u8 spurtTestFlag=0;
uchar spurtTestAction[200]={STRAIGHT90L,STRAIGHT90L,STRAIGHT90L,R180L};//200！一定要改回来！
uchar SpurtTest_FastwayAction[MAZETYPE * MAZETYPE]={0};
uchar FastwayCount;//=7;
//==================================================================================

/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
//extern u8 startcompute;
uchar    GucXStart       = 0;                /*  起点横坐标                  */
uchar    GucYStart       = 0;                /*  起点纵坐标                  */

uchar    GucXGoal0       = XDST0;            /*  终点X坐标，有两个值         */
uchar    GucXGoal1       = XDST1;
uchar    GucYGoal0       = YDST0;            /*  终点Y坐标，有两个值         */
uchar    GucYGoal1       = YDST1;

uchar    GucMouseTask    = WAIT;             /*  状态机，初始状态为等待      */

char GoalX=7;
char GoalY=7;

uchar    GucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  保存各坐标的等高值          */
uchar    GucMapStepC[MAZETYPE][MAZETYPE]      = {0xff};           /*  保存各坐标的等高值          */
uchar    GucMapStepS[MAZETYPE][MAZETYPE]      = {0xff};           /*  保存各坐标的等高值          */
uchar    GucMapStepSC[MAZETYPE][MAZETYPE]      = {0xff};           /*  保存各坐标的等高值          */

MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  在mapStepEdit()中作堆栈使用 */
MAZECOOR GmcStackS1[MAZETYPE * MAZETYPE]       = {0}; 
MAZECOOR GmcStackS2[MAZETYPE * MAZETYPE]       = {0}; 
MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()中暂存未走过支路坐标  */

MAZECOOR GmcMouse= {0,0};                                               /*  GmcMouse.x :电脑鼠横坐标    */
                                                                        /*  GmcMouse.y :电脑鼠纵坐标    */
                                                                        
uchar    GucMouseDir=UP;                                            /*  电脑鼠的前进方向            */
uchar    GucMapBlock[MAZETYPE][MAZETYPE]={0};                      /*  GucMapBlock[x][y]    墙壁资料 QJS	 */
                                                                        /*  x,横坐标;y,纵坐标;          */
                                                                       /*  bit3~bit0分别代表左下右上   */
                                                                       /*  0:该方向无路，1:该方向有路  */
uchar    GucMapGet[MAZETYPE][MAZETYPE]={0}; 
extern int setvl,setvr;
extern u8 task;
extern uchar    GucMouseTask;

uchar	Mapfastway[MAZETYPE][MAZETYPE]= {0xff};

//==================================================================================	
FASTWAYACTION	FastwayAction[MAZETYPE * MAZETYPE]={0};//={0,0,1,1,1,0,7};   /*  在mapStepEdit()中作堆栈使用 */
//==================================================================================	


uchar	Mapfastway_90[MAZETYPE][MAZETYPE]= {0xff};
FASTWAYACTION	FastwayAction_90[MAZETYPE * MAZETYPE]={0};//={0,0,1,1,1,0,7};  /*  在mapStepEdit()中作堆栈使用 */
uchar	FastwayCount_90;//=7;

uchar	FastwayActionTemp[MAZETYPE * MAZETYPE];
u8 ObjectGoTo_Flag=0;

void stop()
{
	task=NULL;GucMouseTask=99;setvl=0;setvr=0;while(1);
}

																	   
/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        制作以目标点为起点的等高图
** input parameters:    uiX:    目的地横坐标
**                      uiY:    目的地纵坐标
** output parameters:   GucMapStep[][]:  各坐标上的等高值
** Returned value:      无

** mapStepEdit 有墙      mapStepEditS  无墙   2018.12.29 QJS

*********************************************************************************************************/
void mapStepEdit (char  cX, char  cY)
{
    uchar n         = 0;                                                /*  GmcStack[]下标              */
    uchar ucStep    = 0;                                                /*  等高值                      */
    uchar ucStat    = 0;                                                /*  统计可前进的方向数          */
    uchar i,j;
    
    GmcStack[n].cX  = cX;                                               /*  起点X值入栈                 */
    GmcStack[n].cY  = cY;                                               /*  起点Y值入栈                 */
    n++;
    /*
     *  初始化各坐标等高值
     */
    for (i = 0; i < MAZETYPE; i++)
	{
        for (j = 0; j < MAZETYPE; j++)
		{
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  制作等高图，直到堆栈中所有数据处理完毕
     */
    while (n)
	{
        GucMapStep[cX][cY] = ucStep++;                                  /*  填入等高值                  */

        /*
         *  对当前坐标格里可前进的方向统计
         */
        ucStat = 0;
		
		    // /* GucMapBlock低八位  每一位0无路  1有路 bit3~bit0分别代表左下右上  *///
		
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  前方有路   */    
            (GucMapStep[cX][cY + 1] > (ucStep))) {                      /*  前方等高值大于计划设定值    */
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] > (ucStep))) {                      /*  右方等高值大于计划设定值    */
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&
            (GucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&
            (GucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        /*
         *  没有可前进的方向，则跳转到最近保存的分支点
         *  否则任选一可前进方向前进
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  有多个可前进方向，保存坐标  */
                GmcStack[n].cX = cX;                                    /*  横坐标X值入栈               */
                GmcStack[n].cY = cY;                                    /*  纵坐标Y值入栈               */
                n++;
            }
            /*
             *  任意选择一条可前进的方向前进
             */
            if ((GucMapBlock[cX][cY] & 0x01) &&                         /*  上方有路                    */
                (GucMapStep[cX][cY + 1] > (ucStep))) {                  /*  上方等高值大于计划设定值    */
                cY++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x02) &&                         /*  右方有路                    */
                (GucMapStep[cX + 1][cY] > (ucStep))) {                  /*  右方等高值大于计划设定值    */
                cX++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x04) &&                         /*  下方有路                    */
                (GucMapStep[cX][cY - 1] > (ucStep))) {                  /*  下方等高值大于计划设定值    */
                cY--;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x08) &&                         /*  左方有路                    */
                (GucMapStep[cX - 1][cY] > (ucStep))) {                  /*  左方等高值大于计划设定值    */
                cX--;                                                   /*  修改坐标                    */
                continue;
            }
        }
    }
}

void mapStepEditS (char  X, char  Y)
{
	uchar n         = 0;
	uchar m         = 0;
	uchar ucStep    = 0;
	uchar i,j;
	char cX=X;
	char cY=Y;
		
	for (i = 0; i < MAZETYPE; i++)
	{
        for (j = 0; j < MAZETYPE; j++) 
		{
            GucMapStepS[i][j] = 0xff;
        }
    }
	
	GucMapStepS[cX][cY]=ucStep;
	
	GmcStackS1[n].cX  = cX;                                               /*  起点X值入栈                 */
    GmcStackS1[n].cY  = cY;
	n++;
	
	while(n)
	{
		ucStep++;
		while(n)
		{
			cX=GmcStackS1[n-1].cX;
			cY=GmcStackS1[n-1].cY;
			if( cY < MAZETYPE-1 ) 
			{
				if ((GucMapBlock[cX][cY] & 0x10) &&                             /*  前方有路                    */
					(GucMapStepS[cX][cY + 1] > (ucStep))) {              /*  前方等高值大于计划设定值    */
					GucMapStepS[cX][cY + 1]=ucStep;
					GmcStackS2[m].cX  = cX;
					GmcStackS2[m].cY  = cY+1;
					m++;
				}
			}
			if( cX < MAZETYPE-1 )
			{
				if ((GucMapBlock[cX][cY] & 0x20) &&                             /*  右方有路                    */
					(GucMapStepS[cX + 1][cY] > (ucStep))) {                      /*  右方等高值大于计划设定值    */
					GucMapStepS[cX + 1][cY]=ucStep;
					GmcStackS2[m].cX  = cX+1;
					GmcStackS2[m].cY  = cY;
					m++;
				}
			}
			if( cY >0 )
			{
				if ((GucMapBlock[cX][cY] & 0x40) &&
					(GucMapStepS[cX][cY - 1] > (ucStep))) {
					GucMapStepS[cX][cY - 1]=ucStep;
					GmcStackS2[m].cX  = cX;
					GmcStackS2[m].cY  = cY-1;
					m++;
				}
			}
			if( cX >0 )
			{
				if ((GucMapBlock[cX][cY] & 0x80) &&
					(GucMapStepS[cX - 1][cY] > (ucStep))) {
					GucMapStepS[cX - 1][cY]=ucStep;
					GmcStackS2[m].cX  = cX-1;
					GmcStackS2[m].cY  = cY;
					m++;
				}
			}
			n--;
		}
		for(i=0;i<m;i++)
		{
			GmcStackS1[i].cX  = GmcStackS2[i].cX;
			GmcStackS1[i].cY  = GmcStackS2[i].cY;
		}
		n=m;
		m=0;
	}
}

void mapStepEditC (void)
{
	uchar i,j;
		
	for (i = 0; i < MAZETYPE; i++)
	{
        for (j = 0; j < MAZETYPE; j++) 
		{
            GucMapStepC[i][j] = GucMapStep[i][j];
        }
    }	
}

//复制保存无墙迷宫的数据到GucMapStepSC[i][j]
void mapStepEditSC (void)
{
	uchar i,j;
		
	for (i = 0; i < MAZETYPE; i++)
	{
        for (j = 0; j < MAZETYPE; j++) 
		{
            GucMapStepSC[i][j] = GucMapStepS[i][j];
        }
    }	
}
/*********************************************************************************************************
** Function name:       mouseSpurt
** Descriptions:        电脑鼠从起点以最短路径跑向终点
** input parameters:    无
 ** output parameters:  无
** Returned value:      无
*********************************************************************************************************/
void mouseSpurt (void)
{
    uchar ucTemp = 0xff;
    char cXdst = 0,cYdst = 0;
    /*
     *  对终点的四个坐标分别制作等高图
     *  取离起点最近的一个点作为目标点
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    objectGoTo(cXdst,cYdst);                                            /*  运行到指定目标点            */
}
/*********************************************************************************************************
** Function name:       objectGoTo
** Descriptions:        使电脑鼠运动到指定坐标
** input parameters:    cXdst: 目的地的横坐标
**                      cYdst: 目的地的纵坐标
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void objectGoTo (char  cXdst, char  cYdst)
{		
    uchar ucStep = 1;
    char cNBlock = 0, cDirTemp;
    char cX,cY;
	ledon(4);ledon(5);
	ObjectGoTo_Flag=1;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst,cYdst);                                           /*  制作等高图                  */
    /*
     *  根据等高值向目标点运动，直到达到目的地
     */
    while ((cX != cXdst) || (cY != cYdst))
	{
        ucStep = GucMapStep[cX][cY];
        /*
         *  任选一个等高值比当前自身等高值小的方向前进
         */
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  上方有路                    */
            (GucMapStep[cX][cY + 1] < ucStep)) {                        /*  上方等高值较小              */
            cDirTemp = UP;                                              /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] < ucStep)) {                        /*  右方等高值较小              */
            cDirTemp = RIGHT;                                           /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  下方有路                    */
            (GucMapStep[cX][cY - 1] < ucStep)) {                        /*  下方等高值较小              */
            cDirTemp = DOWN;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY--;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  左方有路                    */
            (GucMapStep[cX - 1][cY] < ucStep)) {                        /*  左方等高值较小  有墙迷宫qjs */
            cDirTemp = LEFT;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX--;
                continue;                                               /*  跳过本次循环                */
            }
        }
        cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;                      /*  计算方向偏移量              */
        
        if (cNBlock)
		{
			mouseGoahead(cNBlock);
        }        
        cNBlock = 0;                                                    /*  任务清零                    */
        
        /*
         *  控制电脑鼠转弯
         */
        switch (cDirTemp) {

        case 1:
            mouseTurnright();
            break;

        case 2:
            mouseTurnback();
            break;

        case 3:
            mouseTurnleft();
            break;

        default:
            break;
        }

    	cX = GmcMouse.cX;
    	cY = GmcMouse.cY;
    }
    /*
     *  判断任务是否完成，否则继续前进
     */
	
    if (cNBlock)
	{	
        mouseGoahead(cNBlock);
    }
	ObjectGoTo_Flag=0;
}
/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        根据电脑鼠的相对方向，取出该方向上迷宫格的墙壁资料
** input parameters:    ucDir: 电脑鼠的相对方向
** output parameters:   无
** Returned value:      GucMapBlock[cX][cY] : 墙壁资料
*********************************************************************************************************/
uchar mazeBlockDataGet (uchar  ucDirTemp)
{
    char cX = 0,cY = 0;
    
    /*
     *  把电脑鼠的相对方向转换为绝对方向
     */
    switch (ucDirTemp) {

    case MOUSEFRONT:        //1
        ucDirTemp = GucMouseDir;
        break;

    case MOUSELEFT:         //0
        ucDirTemp = (GucMouseDir + 3) % 4;
        break;

    case MOUSERIGHT:        //2
        ucDirTemp = (GucMouseDir + 1) % 4;
        break;

    default:
        break;
    }
    
    /*
     *  根据绝对方向计算该方向上相邻格的坐标
     */
    switch (ucDirTemp) {

    case 0:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY + 1;
        break;
        
    case 1:
        cX = GmcMouse.cX + 1;
        cY = GmcMouse.cY;
        break;
        
    case 2:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY - 1;
        break;
        
    case 3:
        cX = GmcMouse.cX - 1;
        cY = GmcMouse.cY;
        break;
        
    default:
        break;
    }
    
    return(GucMapBlock[cX][cY]);                                        /*  返回迷宫格上的资料          */
}

uchar mazeGetDataGet (uchar  ucDirTemp)           //判断有没有走过 QJS 2018.12.29
{
    char cX = 0,cY = 0;
    
    /*
     *  把电脑鼠的相对方向转换为绝对方向
     */
    switch (ucDirTemp) {

    case MOUSEFRONT:
        ucDirTemp = GucMouseDir;
        break;

    case MOUSELEFT:
        ucDirTemp = (GucMouseDir + 3) % 4;
        break;

    case MOUSERIGHT:
        ucDirTemp = (GucMouseDir + 1) % 4;
        break;

    default:
        break;
    }
    
    /*
     *  根据绝对方向计算该方向上相邻格的坐标
     */
    switch (ucDirTemp) {

    case 0:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY + 1;
        break;
        
    case 1:
        cX = GmcMouse.cX + 1;
        cY = GmcMouse.cY;
        break;
        
    case 2:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY - 1;
        break;
        
    case 3:
        cX = GmcMouse.cX - 1;
        cY = GmcMouse.cY;
        break;
        
    default:
        break;
    }
    
    return(GucMapGet[cX][cY]);                                        /*  返回迷宫格上的资料          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        右手法则，优先向右前进
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void rightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
}

void rightleftMethod (void)
{
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
		(mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  电脑鼠的右边没有走过        */
		mouseTurnright();                                               /*  电脑鼠右转                  */
		return;
	}
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        左手法则，优先向左运动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }    
}

void leftrightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }   
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        中右法则，优先向前运行，其次向右
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontRightMethod (void)
{    
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }   
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        中左法则，优先向前运行，其次向左
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }    
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }   
}
/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        中心法则，根据电脑鼠目前在迷宫中所处的位置觉定使用何种搜索法则
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的右上角
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
				if(GmcMouse.cX<=GmcMouse.cY)
                frontRightMethod();                                     /*  中右法则                    */
				else
				rightMethod();
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
				if(GmcMouse.cX>=GmcMouse.cY)
                frontLeftMethod();                                      /*  中左法则                    */
				else
				leftMethod();
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的右下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
				if(GmcMouse.cX-7<=8-GmcMouse.cY)
                frontLeftMethod();                                      /*  中左法则                    */
				else
				leftMethod();
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
				if(GmcMouse.cX-7>=8-GmcMouse.cY)
                frontRightMethod();                                     /*  中右法则                    */
				else
				rightMethod();
                break;

            default:
                break;
            }
        }
    } else {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的左上角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
				if(8-GmcMouse.cX>=GmcMouse.cY-7)
                frontRightMethod();                                     /*  中右法则                    */
				else
				rightMethod();
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
				if(8-GmcMouse.cX<=GmcMouse.cY-7)
                frontLeftMethod();                                      /*  中左法则                    */
				else
				leftMethod();
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                leftMethod();                                           /*  左手法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的左下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
				if(8-GmcMouse.cX<=8-GmcMouse.cY)
                frontRightMethod();                                     /*  中右法则                    */
				else
				rightMethod();
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
				if(8-GmcMouse.cX>=8-GmcMouse.cY)
                frontLeftMethod();                                      /*  中左法则                    */
				else
				leftMethod();
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                rightMethod();                                          /*  右手法则                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        统计某坐标存在还未走过的支路数
** input parameters:    ucX，需要检测点的横坐标
**                      ucY，需要检测点的纵坐标
** output parameters:   无
** Returned value:      ucCt，未走过的支路数
*********************************************************************************************************/
uchar crosswayCheck (char  cX, char  cY)
{
    uchar ucCt = 0;
    if ((GucMapBlock[cX][cY] & 0x01) &&                                 /*  绝对方向，迷宫上方有路      */
        (GucMapGet[cX][cY + 1] == 0)) {                            /*  绝对方向，迷宫上方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                                 /*  绝对方向，迷宫右方有路      */
        (GucMapGet[cX + 1][cY] == 0)) {                            /*  绝对方向，迷宫右方没有走过  */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                                 /*  绝对方向，迷宫下方有路      */
        (GucMapGet[cX][cY - 1] == 0)) {                            /*  绝对方向，迷宫下方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                                 /*  绝对方向，迷宫左方有路      */
        (GucMapGet[cX - 1][cY] == 0)) {                            /*  绝对方向，迷宫左方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        选择一条支路作为前进方向 QJS用无墙迷宫的等高值来定此时的策略  大体上往哪走 QJS
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void crosswayChoice (void)
{
    uchar sF,sL,sR;
	mapStepEditS(GoalX,GoalY);
	sF=sStepGet(MOUSEFRONT);
	sL=sStepGet(MOUSELEFT);
	sR=sStepGet(MOUSERIGHT);

	//printf("cC ");
    if((sR<sF)&&(sF<=sL))
    {
        rightMethod();
    }
	else if((sR<sL)&&(sL<=sF))
	{
		rightleftMethod();
	}
    else if((sL<sF)&&(sF<=sR))
	{
        leftMethod();
    }
    else if((sL<sR)&&(sR<=sL))
	{
        leftrightMethod();
    }    
    else if((sF<sR)&&(sR<=sL))
	{
        frontRightMethod();
	}
    else if((sF<sL)&&(sL<=sR))
	{
        frontLeftMethod();
	}
	else
	{
		centralMethod();
	}
}
/*********************************************************************************************************
** Function name:       main
** Descriptions:        主函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void goalWallchange(char GoalX,char GoalY)
{
	uchar temp,temp1;
	GucMapBlock[GucXGoal0][GucYGoal0]=((GucMapBlock[GucXGoal0][GucYGoal0])&0x0F);
	GucMapBlock[GucXGoal0][GucYGoal1]=((GucMapBlock[GucXGoal0][GucYGoal1])&0x0F);
	GucMapBlock[GucXGoal1][GucYGoal0]=((GucMapBlock[GucXGoal1][GucYGoal0])&0x0F);
	GucMapBlock[GucXGoal1][GucYGoal1]=((GucMapBlock[GucXGoal1][GucYGoal1])&0x0F);
	temp1=GucMapBlock[GoalX][GoalY];
	temp=(temp1<<4);
	GucMapBlock[GoalX][GoalY]=((GucMapBlock[GoalX][GoalY])|temp);
}

void backpointwaychoice(char cX,char cY)
{
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeGetDataGet(MOUSEFRONT)== 0)&&
		(sStepGet(MOUSEFRONT)<GucMapStepS[cX][cY])) {
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeGetDataGet(MOUSELEFT )== 0)&&
		(sStepGet(MOUSELEFT)<GucMapStepS[cX][cY])) {
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeGetDataGet(MOUSERIGHT)== 0)&&
		(sStepGet(MOUSERIGHT)<GucMapStepS[cX][cY])) {
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}
  
/*********************************************************************************************************
** Function name:       sStepGet
** Descriptions:        
** input parameters:    ucDirTemp
** output parameters:   
** Returned value:      GucMapStepS[cX][cY]
**********************************************************************************************************/

uchar sStepGet (uchar  ucDirTemp)
{
    char cX = 0,cY = 0;
    switch (ucDirTemp) {
    case MOUSEFRONT:
        ucDirTemp = GucMouseDir;
        break;
    case MOUSELEFT:
        ucDirTemp = (GucMouseDir + 3) % 4;
        break;
    case MOUSERIGHT:
        ucDirTemp = (GucMouseDir + 1) % 4;
        break;
    default:
        break;
    }
    switch (ucDirTemp) 
	{
    case 0:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY + 1;
        break;
    case 1:
        cX = GmcMouse.cX + 1;
        cY = GmcMouse.cY;
        break;
    case 2:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY - 1;
        break;
    case 3:
        cX = GmcMouse.cX - 1;
        cY = GmcMouse.cY;
        break;
        
    default:
        break;
    }
	if((cX<=15)&&(cY<=15))
	{
		return(GucMapStepS[cX][cY]);
	}
	else
	{
		return 255;
	}
}

//死区的判断
uchar deadwaycheck(uchar cross)
{						
	uchar backcross=cross;     //死区返回点
	uchar n=cross;
	uchar m;
	uchar bx=GmcMouse.cX;
	uchar by=GmcMouse.cY;
	uchar countmap=0;
	
	uchar tempRoadState;
	uchar tempx,tempy;
	uchar tempBlock;
	
	mapStepEditS(GoalX,GoalY);
	mapStepEditSC();
	
	while(--n)
	{
		tempRoadState=crosswayCheck(GmcCrossway[n].cX,GmcCrossway[n].cY);      //可前进支路数
		if(tempRoadState)
		{
			tempx=GmcCrossway[n].cX;
			tempy=GmcCrossway[n].cY;
			if(GucMapStepSC[tempx][tempy]<GucMapStepSC[bx][by])
			{
				tempBlock=GucMapBlock[tempx][tempy];
				GucMapBlock[tempx][tempy]=0x00;
				mapStepEditS(GmcMouse.cX,GmcMouse.cY);
				countmap++;
				GucMapBlock[tempx][tempy]=tempBlock;
				if(GucMapStepS[GoalX][GoalY]==0xff)
				{
					m=cross;
					while(--m>n)  //！！！考虑一下如果有点在外面怎么办，目前会直接导致函数无效
					{
						if(GucMapStepS[GmcCrossway[m].cX][GmcCrossway[m].cY]!=0xff)
						{
							continue;
						}
						else
						{
							goto enddead;
						}
					}
					bx=tempx;
					by=tempy;
					backcross=n;
				}
				if(countmap>=20)
				{
					break;
				}
			}
			else
			{
				continue;
			}
		}
		else
		{
			//GmcCrossway[n].cX=GmcMouse.cX;     //可以对乱序的外部点进行处理，对其他处的安全性影响有待考证
			//GmcCrossway[n].cY=GmcMouse.cY;
			continue;
		}
	}
enddead:	
	return backcross;
}

//冲刺路线计算
void fastway(uchar GX,uchar GY)
{
	char i,j,cX,cY;
	uchar ucStep;
	uchar cDirTemp;
	uchar wayDir;     //????????
	uchar goDir;
	uchar mouseDir=GucMouseDir*2;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
		 	
	for(i=0;i<16;i++)
	{
		for(j=0;j<16;j++)
		{
			Mapfastway[i][j]=0xff;
		}
	}
	FastwayCount=0;

	mapStepEdit(GX,GY);
	ucStep=GucMapStep[cX][cY];

	if ((GucMapBlock[cX][cY] & 0x01) && (GucMapStep[cX][cY + 1] < ucStep))
	{	 
		cDirTemp = 0;
		if(mouseDir==cDirTemp)
		{
			Mapfastway[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction[FastwayCount].action=0;
			FastwayAction[FastwayCount].cx=cX;
			FastwayAction[FastwayCount].cy=cY;
			FastwayCount++;
			cY++;
			goto sec;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x02) && (GucMapStep[cX + 1][cY] < ucStep))
	{	  
		cDirTemp = 2;
		if(mouseDir==cDirTemp)
		{
			Mapfastway[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction[FastwayCount].action=0;
			FastwayAction[FastwayCount].cx=cX;
			FastwayAction[FastwayCount].cy=cY;
			FastwayCount++;
			cX++;
			goto sec;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x04) && (GucMapStep[cX][cY - 1] < ucStep))
	{	
		cDirTemp = 4;
		if(mouseDir==cDirTemp)
		{
			Mapfastway[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction[FastwayCount].action=0;
			FastwayAction[FastwayCount].cx=cX;
			FastwayAction[FastwayCount].cy=cY;
			FastwayCount++;
			cY--;
			goto sec;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x08) && (GucMapStep[cX - 1][cY] < ucStep))
	{	
		cDirTemp = 6;
		if(mouseDir==cDirTemp)
		{
			Mapfastway[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction[FastwayCount].action=0;
			FastwayAction[FastwayCount].cx=cX;
			FastwayAction[FastwayCount].cy=cY;
			FastwayCount++;
			cX--;
			goto sec;
		}
	}
	
	FastwayAction[FastwayCount].action=(8+cDirTemp-mouseDir)%8;
	FastwayAction[FastwayCount].cx=cX;
	FastwayAction[FastwayCount].cy=cY;
	FastwayCount++;
	
	Mapfastway[cX][cY]=cDirTemp;
	wayDir=cDirTemp;
	mouseDir=cDirTemp;
	switch(cDirTemp)	//????????,?????????????
	{
		case 0:cY++;
			   break;
		case 2:cX++;
			   break;
		case 4:cY--;
			   break;
		case 6:cX--;
			   break;
		default:break;
	}
		
sec:	
	while ((cX != GX) || (cY != GY))
	{
		ucStep = GucMapStep[cX][cY];
		mouseDir=cDirTemp;
		if ((GucMapBlock[cX][cY] & 0x01) && (GucMapStep[cX][cY + 1] < ucStep))	
		{
			goDir=0;
			if(wayDir==0)
			{
				cDirTemp = 0;
			}
			if(wayDir==2)
			{
				cDirTemp = 1;
			}
			if(wayDir==6)
			{
				cDirTemp = 7;
			}
			if(mouseDir==cDirTemp)
			{
				Mapfastway[cX][cY]=cDirTemp;
				wayDir=goDir;
				FastwayAction[FastwayCount].action=0;
				FastwayAction[FastwayCount].cx=cX;
				FastwayAction[FastwayCount].cy=cY;
				FastwayCount++;
				cY++;
				continue; 
			}
		}
		if ((GucMapBlock[cX][cY] & 0x02) && (GucMapStep[cX + 1][cY] < ucStep))
		{
			goDir=2;
			if(wayDir==0)
			{
				cDirTemp = 1;
			}
			if(wayDir==2)
			{
				cDirTemp = 2;
			}
			if(wayDir==4)
			{
				cDirTemp = 3;
			}
			if(mouseDir==cDirTemp)
			{
				Mapfastway[cX][cY]=cDirTemp;
				wayDir=goDir;
				FastwayAction[FastwayCount].action=0;
				FastwayAction[FastwayCount].cx=cX;
				FastwayAction[FastwayCount].cy=cY;
				FastwayCount++;
				cX++;
				continue; 
			}
		}
		if ((GucMapBlock[cX][cY] & 0x04) && (GucMapStep[cX][cY - 1] < ucStep))
		{
			goDir=4;
			if(wayDir==2)
			{
				cDirTemp = 3;
			}
			if(wayDir==4)
			{
				cDirTemp = 4;
			}
			if(wayDir==6)
			{
				cDirTemp = 5;
			}
			if(mouseDir==cDirTemp)
			{
				Mapfastway[cX][cY]=cDirTemp;
				wayDir=goDir;
				FastwayAction[FastwayCount].action=0;
				FastwayAction[FastwayCount].cx=cX;
				FastwayAction[FastwayCount].cy=cY;
				FastwayCount++;
				cY--;
				continue; 
			}
		}
		if ((GucMapBlock[cX][cY] & 0x08) && (GucMapStep[cX - 1][cY] < ucStep))
		{
			goDir=6;
			if(wayDir==0)
			{
				cDirTemp = 7;
			}
			if(wayDir==4)
			{
				cDirTemp = 5;
			}
			if(wayDir==6)
			{
				cDirTemp = 6;
			}
			if(mouseDir==cDirTemp)
			{
				Mapfastway[cX][cY]=cDirTemp;
				wayDir=goDir;
				FastwayAction[FastwayCount].action=0;
				FastwayAction[FastwayCount].cx=cX;
				FastwayAction[FastwayCount].cy=cY;
				FastwayCount++;
				cX--;
				continue; 
			}
		}
		
		Mapfastway[cX][cY]=cDirTemp;
		
		FastwayAction[FastwayCount].action=(8+Mapfastway[cX][cY]-mouseDir)%8;
		FastwayAction[FastwayCount].cx=cX;
		FastwayAction[FastwayCount].cy=cY;
		FastwayCount++;
		
		mouseDir=cDirTemp;
		wayDir=goDir;
		
		switch(goDir)
		{
			case 0:cY++;
				   break;
			case 2:cX++;
				   break;
			case 4:cY--;
				   break;
			case 6:cX--;
				   break;
			default:break;
		}
	}
	
	FastwayAction[FastwayCount].action=(8+wayDir-mouseDir)%8;
	FastwayAction[FastwayCount].cx=cX;
	FastwayAction[FastwayCount].cy=cY;
	FastwayCount++;
}


//--------------------------------垂直冲刺 2014年10月10日 卖萌日--------------------------------------------
void fastway_90(uchar GX,uchar GY)
{
	char i,j,cX,cY;
	uchar ucStep;
	uchar cDirTemp;
	uchar wayDir;     //????????
//	uchar goDir;
	uchar mouseDir=GucMouseDir;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
		 	
	for(i=0;i<16;i++)
	{
		for(j=0;j<16;j++)
		{
			Mapfastway_90[i][j]=0xff;
		}
	}
	FastwayCount_90=0;

	mapStepEdit(GX,GY);
	ucStep=GucMapStep[cX][cY];
	if ((GucMapBlock[cX][cY] & 0x01) && (GucMapStep[cX][cY + 1] < ucStep))
	{	 
		cDirTemp = 0;
		if(mouseDir==cDirTemp)
		{
			Mapfastway_90[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction_90[FastwayCount_90].action=0;
			FastwayAction_90[FastwayCount_90].cx=cX;
			FastwayAction_90[FastwayCount_90].cy=cY;
			FastwayCount_90++;
			cY++;
			goto sec;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x02) && (GucMapStep[cX + 1][cY] < ucStep))
	{	  
		cDirTemp = 1;
		if(mouseDir==cDirTemp)
		{
			Mapfastway_90[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction_90[FastwayCount_90].action=0;
			FastwayAction_90[FastwayCount_90].cx=cX;
			FastwayAction_90[FastwayCount_90].cy=cY;
			FastwayCount_90++;
			cX++;
			goto sec;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x04) && (GucMapStep[cX][cY - 1] < ucStep))
	{	
		cDirTemp = 2;
		if(mouseDir==cDirTemp)
		{
			Mapfastway_90[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction_90[FastwayCount_90].action=0;
			FastwayAction_90[FastwayCount_90].cx=cX;
			FastwayAction_90[FastwayCount_90].cy=cY;
			FastwayCount_90++;
			cY--;
			goto sec;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x08) && (GucMapStep[cX - 1][cY] < ucStep))
	{	
		cDirTemp = 3;
		if(mouseDir==cDirTemp)
		{
			Mapfastway_90[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction_90[FastwayCount_90].action=0;
			FastwayAction_90[FastwayCount_90].cx=cX;
			FastwayAction_90[FastwayCount_90].cy=cY;			
			FastwayCount_90++;
			cX--;
			goto sec;
		}
	}
	
	FastwayAction_90[FastwayCount_90].action=(4+cDirTemp-mouseDir)%4;
	FastwayAction_90[FastwayCount_90].cx=cX;
	FastwayAction_90[FastwayCount_90].cy=cY;
	FastwayCount_90++;
	
	Mapfastway_90[cX][cY]=cDirTemp;
	wayDir=cDirTemp;
	mouseDir=cDirTemp;
	switch(cDirTemp)	//????????,?????????????
	{
		case 0:cY++;
			   break;
		case 1:cX++;
			   break;
		case 2:cY--;
			   break;
		case 3:cX--;
			   break;
		default:break;
	}
		
sec:
	while ((cX != GX) || (cY != GY))
	{		
		ucStep = GucMapStep[cX][cY];
		mouseDir=cDirTemp;	
	if ((GucMapBlock[cX][cY] & 0x01) && (GucMapStep[cX][cY + 1] < ucStep))
	{	 
		cDirTemp = 0;
		if(mouseDir==cDirTemp)
		{
			Mapfastway_90[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction_90[FastwayCount_90].action=0;
			FastwayAction_90[FastwayCount_90].cx=cX;
			FastwayAction_90[FastwayCount_90].cy=cY;
			FastwayCount_90++;
			cY++;
			continue;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x02) && (GucMapStep[cX + 1][cY] < ucStep))
	{	  
		cDirTemp = 1;
		if(mouseDir==cDirTemp)
		{
			Mapfastway_90[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction_90[FastwayCount_90].action=0;
			FastwayAction_90[FastwayCount_90].cx=cX;
			FastwayAction_90[FastwayCount_90].cy=cY;
			FastwayCount_90++;
			cX++;
			continue;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x04) && (GucMapStep[cX][cY - 1] < ucStep))
	{	
		cDirTemp = 2;
		if(mouseDir==cDirTemp)
		{
			Mapfastway_90[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction_90[FastwayCount_90].action=0;
			FastwayAction_90[FastwayCount_90].cx=cX;
			FastwayAction_90[FastwayCount_90].cy=cY;
			FastwayCount_90++;
			cY--;
			continue;
		}
	}
	if ((GucMapBlock[cX][cY] & 0x08) && (GucMapStep[cX - 1][cY] < ucStep))
	{	
		cDirTemp = 3;
		if(mouseDir==cDirTemp)
		{
			Mapfastway_90[cX][cY]=cDirTemp;
			wayDir=cDirTemp;
			FastwayAction_90[FastwayCount_90].action=0;
			FastwayAction_90[FastwayCount_90].cx=cX;
			FastwayAction_90[FastwayCount_90].cy=cY;
			FastwayCount_90++;
			cX--;
			continue;
		}
	}			
		FastwayAction_90[FastwayCount_90].action=(4+cDirTemp-mouseDir)%4;
		FastwayAction_90[FastwayCount_90].cx=cX;
		FastwayAction_90[FastwayCount_90].cy=cY;
		FastwayCount_90++;
		Mapfastway_90[cX][cY]=cDirTemp;
		wayDir=cDirTemp;
		mouseDir=cDirTemp;
		
		switch(cDirTemp)
		{
			case 0:cY++;
				   break;
			case 1:cX++;
				   break;
			case 2:cY--;
				   break;
			case 3:cX--;
				   break;
			default:break;
		}
			}
	FastwayAction_90[FastwayCount_90].action=(4+wayDir-mouseDir)%4;
	FastwayAction_90[FastwayCount_90].cx=cX;
	FastwayAction_90[FastwayCount_90].cy=cY;
	FastwayCount_90++;
	}
	
	////我写的计算返回路线的函数
void backPath(FASTWAYACTION *FastwayAction_1, uchar FastwayCount_2)
{
	int i;
	for(i=0;i<FastwayCount_2;i++)
	{
		FastwayActionTemp[i]=FastwayAction_1[FastwayCount_2-i-1].action;
	}
	for(i=0;i<FastwayCount_2;i++)
	{
		switch(FastwayActionTemp[i])
		{
			case 1:FastwayActionTemp[i]=7;break;
			case 2:FastwayActionTemp[i]=6;break;
			case 6:FastwayActionTemp[i]=2;break;
			case 7:FastwayActionTemp[i]=1;break;
		}
	}	
	FastwayAction_1[0].action=0;
	for(i=1;i<FastwayCount_2;i++)
	{
		FastwayAction_1[i].action=FastwayActionTemp[i-1];
	}
}




