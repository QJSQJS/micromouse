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
//========================��Ҫ���Գ��ʱ�޸���������================================
u8 spurtTestFlag=0;
uchar spurtTestAction[200]={STRAIGHT90L,STRAIGHT90L,STRAIGHT90L,R180L};//200��һ��Ҫ�Ļ�����
uchar SpurtTest_FastwayAction[MAZETYPE * MAZETYPE]={0};
uchar FastwayCount;//=7;
//==================================================================================

/*********************************************************************************************************
  ȫ�ֱ�������
*********************************************************************************************************/
//extern u8 startcompute;
uchar    GucXStart       = 0;                /*  ��������                  */
uchar    GucYStart       = 0;                /*  ���������                  */

uchar    GucXGoal0       = XDST0;            /*  �յ�X���꣬������ֵ         */
uchar    GucXGoal1       = XDST1;
uchar    GucYGoal0       = YDST0;            /*  �յ�Y���꣬������ֵ         */
uchar    GucYGoal1       = YDST1;

uchar    GucMouseTask    = WAIT;             /*  ״̬������ʼ״̬Ϊ�ȴ�      */

char GoalX=7;
char GoalY=7;

uchar    GucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  ���������ĵȸ�ֵ          */
uchar    GucMapStepC[MAZETYPE][MAZETYPE]      = {0xff};           /*  ���������ĵȸ�ֵ          */
uchar    GucMapStepS[MAZETYPE][MAZETYPE]      = {0xff};           /*  ���������ĵȸ�ֵ          */
uchar    GucMapStepSC[MAZETYPE][MAZETYPE]      = {0xff};           /*  ���������ĵȸ�ֵ          */

MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  ��mapStepEdit()������ջʹ�� */
MAZECOOR GmcStackS1[MAZETYPE * MAZETYPE]       = {0}; 
MAZECOOR GmcStackS2[MAZETYPE * MAZETYPE]       = {0}; 
MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()���ݴ�δ�߹�֧·����  */

MAZECOOR GmcMouse= {0,0};                                               /*  GmcMouse.x :�����������    */
                                                                        /*  GmcMouse.y :������������    */
                                                                        
uchar    GucMouseDir=UP;                                            /*  �������ǰ������            */
uchar    GucMapBlock[MAZETYPE][MAZETYPE]={0};                      /*  GucMapBlock[x][y]    ǽ������ QJS	 */
                                                                        /*  x,������;y,������;          */
                                                                       /*  bit3~bit0�ֱ������������   */
                                                                       /*  0:�÷�����·��1:�÷�����·  */
uchar    GucMapGet[MAZETYPE][MAZETYPE]={0}; 
extern int setvl,setvr;
extern u8 task;
extern uchar    GucMouseTask;

uchar	Mapfastway[MAZETYPE][MAZETYPE]= {0xff};

//==================================================================================	
FASTWAYACTION	FastwayAction[MAZETYPE * MAZETYPE]={0};//={0,0,1,1,1,0,7};   /*  ��mapStepEdit()������ջʹ�� */
//==================================================================================	


uchar	Mapfastway_90[MAZETYPE][MAZETYPE]= {0xff};
FASTWAYACTION	FastwayAction_90[MAZETYPE * MAZETYPE]={0};//={0,0,1,1,1,0,7};  /*  ��mapStepEdit()������ջʹ�� */
uchar	FastwayCount_90;//=7;

uchar	FastwayActionTemp[MAZETYPE * MAZETYPE];
u8 ObjectGoTo_Flag=0;

void stop()
{
	task=NULL;GucMouseTask=99;setvl=0;setvr=0;while(1);
}

																	   
/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        ������Ŀ���Ϊ���ĵȸ�ͼ
** input parameters:    uiX:    Ŀ�ĵغ�����
**                      uiY:    Ŀ�ĵ�������
** output parameters:   GucMapStep[][]:  �������ϵĵȸ�ֵ
** Returned value:      ��

** mapStepEdit ��ǽ      mapStepEditS  ��ǽ   2018.12.29 QJS

*********************************************************************************************************/
void mapStepEdit (char  cX, char  cY)
{
    uchar n         = 0;                                                /*  GmcStack[]�±�              */
    uchar ucStep    = 0;                                                /*  �ȸ�ֵ                      */
    uchar ucStat    = 0;                                                /*  ͳ�ƿ�ǰ���ķ�����          */
    uchar i,j;
    
    GmcStack[n].cX  = cX;                                               /*  ���Xֵ��ջ                 */
    GmcStack[n].cY  = cY;                                               /*  ���Yֵ��ջ                 */
    n++;
    /*
     *  ��ʼ��������ȸ�ֵ
     */
    for (i = 0; i < MAZETYPE; i++)
	{
        for (j = 0; j < MAZETYPE; j++)
		{
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  �����ȸ�ͼ��ֱ����ջ���������ݴ������
     */
    while (n)
	{
        GucMapStep[cX][cY] = ucStep++;                                  /*  ����ȸ�ֵ                  */

        /*
         *  �Ե�ǰ��������ǰ���ķ���ͳ��
         */
        ucStat = 0;
		
		    // /* GucMapBlock�Ͱ�λ  ÿһλ0��·  1��· bit3~bit0�ֱ������������  *///
		
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  ǰ����·   */    
            (GucMapStep[cX][cY + 1] > (ucStep))) {                      /*  ǰ���ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] > (ucStep))) {                      /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&
            (GucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&
            (GucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        /*
         *  û�п�ǰ���ķ�������ת���������ķ�֧��
         *  ������ѡһ��ǰ������ǰ��
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  �ж����ǰ�����򣬱�������  */
                GmcStack[n].cX = cX;                                    /*  ������Xֵ��ջ               */
                GmcStack[n].cY = cY;                                    /*  ������Yֵ��ջ               */
                n++;
            }
            /*
             *  ����ѡ��һ����ǰ���ķ���ǰ��
             */
            if ((GucMapBlock[cX][cY] & 0x01) &&                         /*  �Ϸ���·                    */
                (GucMapStep[cX][cY + 1] > (ucStep))) {                  /*  �Ϸ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x02) &&                         /*  �ҷ���·                    */
                (GucMapStep[cX + 1][cY] > (ucStep))) {                  /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cX++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x04) &&                         /*  �·���·                    */
                (GucMapStep[cX][cY - 1] > (ucStep))) {                  /*  �·��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY--;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x08) &&                         /*  ����·                    */
                (GucMapStep[cX - 1][cY] > (ucStep))) {                  /*  �󷽵ȸ�ֵ���ڼƻ��趨ֵ    */
                cX--;                                                   /*  �޸�����                    */
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
	
	GmcStackS1[n].cX  = cX;                                               /*  ���Xֵ��ջ                 */
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
				if ((GucMapBlock[cX][cY] & 0x10) &&                             /*  ǰ����·                    */
					(GucMapStepS[cX][cY + 1] > (ucStep))) {              /*  ǰ���ȸ�ֵ���ڼƻ��趨ֵ    */
					GucMapStepS[cX][cY + 1]=ucStep;
					GmcStackS2[m].cX  = cX;
					GmcStackS2[m].cY  = cY+1;
					m++;
				}
			}
			if( cX < MAZETYPE-1 )
			{
				if ((GucMapBlock[cX][cY] & 0x20) &&                             /*  �ҷ���·                    */
					(GucMapStepS[cX + 1][cY] > (ucStep))) {                      /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
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

//���Ʊ�����ǽ�Թ������ݵ�GucMapStepSC[i][j]
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
** Descriptions:        ���������������·�������յ�
** input parameters:    ��
 ** output parameters:  ��
** Returned value:      ��
*********************************************************************************************************/
void mouseSpurt (void)
{
    uchar ucTemp = 0xff;
    char cXdst = 0,cYdst = 0;
    /*
     *  ���յ���ĸ�����ֱ������ȸ�ͼ
     *  ȡ����������һ������ΪĿ���
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    objectGoTo(cXdst,cYdst);                                            /*  ���е�ָ��Ŀ���            */
}
/*********************************************************************************************************
** Function name:       objectGoTo
** Descriptions:        ʹ�������˶���ָ������
** input parameters:    cXdst: Ŀ�ĵصĺ�����
**                      cYdst: Ŀ�ĵص�������
** output parameters:   ��
** Returned value:      ��
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
    mapStepEdit(cXdst,cYdst);                                           /*  �����ȸ�ͼ                  */
    /*
     *  ���ݵȸ�ֵ��Ŀ����˶���ֱ���ﵽĿ�ĵ�
     */
    while ((cX != cXdst) || (cY != cYdst))
	{
        ucStep = GucMapStep[cX][cY];
        /*
         *  ��ѡһ���ȸ�ֵ�ȵ�ǰ����ȸ�ֵС�ķ���ǰ��
         */
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  �Ϸ���·                    */
            (GucMapStep[cX][cY + 1] < ucStep)) {                        /*  �Ϸ��ȸ�ֵ��С              */
            cDirTemp = UP;                                              /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] < ucStep)) {                        /*  �ҷ��ȸ�ֵ��С              */
            cDirTemp = RIGHT;                                           /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  �·���·                    */
            (GucMapStep[cX][cY - 1] < ucStep)) {                        /*  �·��ȸ�ֵ��С              */
            cDirTemp = DOWN;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY--;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  ����·                    */
            (GucMapStep[cX - 1][cY] < ucStep)) {                        /*  �󷽵ȸ�ֵ��С  ��ǽ�Թ�qjs */
            cDirTemp = LEFT;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX--;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;                      /*  ���㷽��ƫ����              */
        
        if (cNBlock)
		{
			mouseGoahead(cNBlock);
        }        
        cNBlock = 0;                                                    /*  ��������                    */
        
        /*
         *  ���Ƶ�����ת��
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
     *  �ж������Ƿ���ɣ��������ǰ��
     */
	
    if (cNBlock)
	{	
        mouseGoahead(cNBlock);
    }
	ObjectGoTo_Flag=0;
}
/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        ���ݵ��������Է���ȡ���÷������Թ����ǽ������
** input parameters:    ucDir: ���������Է���
** output parameters:   ��
** Returned value:      GucMapBlock[cX][cY] : ǽ������
*********************************************************************************************************/
uchar mazeBlockDataGet (uchar  ucDirTemp)
{
    char cX = 0,cY = 0;
    
    /*
     *  �ѵ��������Է���ת��Ϊ���Է���
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
     *  ���ݾ��Է������÷��������ڸ������
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
    
    return(GucMapBlock[cX][cY]);                                        /*  �����Թ����ϵ�����          */
}

uchar mazeGetDataGet (uchar  ucDirTemp)           //�ж���û���߹� QJS 2018.12.29
{
    char cX = 0,cY = 0;
    
    /*
     *  �ѵ��������Է���ת��Ϊ���Է���
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
     *  ���ݾ��Է������÷��������ڸ������
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
    
    return(GucMapGet[cX][cY]);                                        /*  �����Թ����ϵ�����          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        ���ַ�����������ǰ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void rightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}

void rightleftMethod (void)
{
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
		(mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  ��������ұ�û���߹�        */
		mouseTurnright();                                               /*  ��������ת                  */
		return;
	}
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        ���ַ������������˶�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }    
}

void leftrightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }   
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        ���ҷ���������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontRightMethod (void)
{    
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }   
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        ������������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }    
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }   
}
/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        ���ķ��򣬸��ݵ�����Ŀǰ���Թ���������λ�þ���ʹ�ú�����������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
				if(GmcMouse.cX<=GmcMouse.cY)
                frontRightMethod();                                     /*  ���ҷ���                    */
				else
				rightMethod();
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
				if(GmcMouse.cX>=GmcMouse.cY)
                frontLeftMethod();                                      /*  ������                    */
				else
				leftMethod();
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
				if(GmcMouse.cX-7<=8-GmcMouse.cY)
                frontLeftMethod();                                      /*  ������                    */
				else
				leftMethod();
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
				if(GmcMouse.cX-7>=8-GmcMouse.cY)
                frontRightMethod();                                     /*  ���ҷ���                    */
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
             *  ��ʱ���������Թ������Ͻ�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
				if(8-GmcMouse.cX>=GmcMouse.cY-7)
                frontRightMethod();                                     /*  ���ҷ���                    */
				else
				rightMethod();
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
				if(8-GmcMouse.cX<=GmcMouse.cY-7)
                frontLeftMethod();                                      /*  ������                    */
				else
				leftMethod();
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
				if(8-GmcMouse.cX<=8-GmcMouse.cY)
                frontRightMethod();                                     /*  ���ҷ���                    */
				else
				rightMethod();
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
				if(8-GmcMouse.cX>=8-GmcMouse.cY)
                frontLeftMethod();                                      /*  ������                    */
				else
				leftMethod();
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        ͳ��ĳ������ڻ�δ�߹���֧·��
** input parameters:    ucX����Ҫ����ĺ�����
**                      ucY����Ҫ�����������
** output parameters:   ��
** Returned value:      ucCt��δ�߹���֧·��
*********************************************************************************************************/
uchar crosswayCheck (char  cX, char  cY)
{
    uchar ucCt = 0;
    if ((GucMapBlock[cX][cY] & 0x01) &&                                 /*  ���Է����Թ��Ϸ���·      */
        (GucMapGet[cX][cY + 1] == 0)) {                            /*  ���Է����Թ��Ϸ�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                                 /*  ���Է����Թ��ҷ���·      */
        (GucMapGet[cX + 1][cY] == 0)) {                            /*  ���Է����Թ��ҷ�û���߹�  */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                                 /*  ���Է����Թ��·���·      */
        (GucMapGet[cX][cY - 1] == 0)) {                            /*  ���Է����Թ��·�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                                 /*  ���Է����Թ�����·      */
        (GucMapGet[cX - 1][cY] == 0)) {                            /*  ���Է����Թ���δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        ѡ��һ��֧·��Ϊǰ������ QJS����ǽ�Թ��ĵȸ�ֵ������ʱ�Ĳ���  ������������ QJS
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeGetDataGet(MOUSEFRONT)== 0)&&
		(sStepGet(MOUSEFRONT)<GucMapStepS[cX][cY])) {
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeGetDataGet(MOUSELEFT )== 0)&&
		(sStepGet(MOUSELEFT)<GucMapStepS[cX][cY])) {
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeGetDataGet(MOUSERIGHT)== 0)&&
		(sStepGet(MOUSERIGHT)<GucMapStepS[cX][cY])) {
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeGetDataGet(MOUSEFRONT)== 0)) {
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeGetDataGet(MOUSELEFT )== 0)) {
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeGetDataGet(MOUSERIGHT)== 0)) {
        mouseTurnright();                                               /*  ��������ת                  */
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

//�������ж�
uchar deadwaycheck(uchar cross)
{						
	uchar backcross=cross;     //�������ص�
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
		tempRoadState=crosswayCheck(GmcCrossway[n].cX,GmcCrossway[n].cY);      //��ǰ��֧·��
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
					while(--m>n)  //����������һ������е���������ô�죬Ŀǰ��ֱ�ӵ��º�����Ч
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
			//GmcCrossway[n].cX=GmcMouse.cX;     //���Զ�������ⲿ����д������������İ�ȫ��Ӱ���д���֤
			//GmcCrossway[n].cY=GmcMouse.cY;
			continue;
		}
	}
enddead:	
	return backcross;
}

//���·�߼���
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


//--------------------------------��ֱ��� 2014��10��10�� ������--------------------------------------------
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
	
	////��д�ļ��㷵��·�ߵĺ���
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




