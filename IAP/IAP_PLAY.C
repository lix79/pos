#include "IAP_PLAY.H"
#include "IAP_DATA.H"
#include "IAP_EXE.H"
#include "ff.h"
#include "sys.h"
#include "delay.h"
#include "sys.h"
#include "Led.h"
#include "Oled.h"
#include "Malloc.h"
#include "sdio_sdcard.h"
#include <string.h>
#include "OLED_GUI.h"
#include "STM32_CRC.h"
#include "stmflash.h"
#include "IAP_PLAY.H"

//��ʾ��ǰ������½���
//x,y:����
//size:�����С
//fsize:�����ļ���С
//pos:��ǰ�ļ�ָ��λ��
void IAP_EXE_PALY_prog(u8 x,u8 y,u32 fsize,u32 pos)
{
	float prog;
	u8 t=0XFF;
	prog=(float)pos/fsize;
	prog*=100;
	if(t!=prog)
	{
		OLED_ShowString(x+24,y,"%",16,1,1);		
		t=prog;
		if(t>100)
		{
			t=100;
		}
		OLED_ShowNum(x,y,t,3,16,1);//��ʾ��ֵ
 //��ʾ������----------------------------------------------------------
		OLED_Fills(14,y-4-20 ,14+t,y-4,1,0);    //���     �߶�20������
		OLED_DrawRectangle(14,y-4-20 ,114,y-4); //�����ľ���  ˢ�»���
	}					    
} 









