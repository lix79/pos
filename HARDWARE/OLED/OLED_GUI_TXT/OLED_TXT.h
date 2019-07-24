#ifndef _OLED_TXT_H_
#include "sys.h"
/*********************V 2.1 �汾˵��*****************************
 * ���ߣ� �ܳ���     ���ʱ�䣺 15 - 4 -22
 * ʵ�ֹ��ܣ� �Ѵ�ԭ�ӿ�����ʵ���ֿ������
 * �ѹ��ɹ��ܺ��� ��ʾ�ַ����ͺ���  ���Զ����� 
 * �������⣺ ��ʱ��δ�ҵ�C51 �ֽڶ������Ч�ؼ��� 
 * �޷���ȡ����Ч�ĳ�Ա����ֵ   ��ֲ�Լ���չ�Խϲ�
 * �޷���Ч���ж��Ƿ�����ֿ⼰�ֿ�����
*****************************************************************/

/*********************V 2.2 �汾˵��*******************************
 * ����ʱ�䣺 15 - 4 - 23
 * ����˵���� �޸��޷���ȡ����Ч��Ա����ֵ   
 * C51��Ƭ��Ϊ8λ��Ƭ�� ���ֽڶ������� 
 * �ֿ���д��STM32ΪС��ģʽ   C51Ϊ���ģʽ  ���ݶ�ȡʱ����
 * ���ڱ��ļ��м����С��ת������  ����ֿ�������Ϊ���ģʽ �������
 * ���޸���С������  ��ֲ�Լ���չ�Դ������
******************************************************************/

#define Mode_Swapping_EN     0      //����ֿ���д�ĵ�Ƭ����Ϊ���ģʽ �˺�ֵ��Ϊ 1
#define PACK_EN              1      //��Ҫ�ṹ���ֽڶ��� �˺�ֵΪ1   ����Ƭ����Ϊ8λ��Ҫ��Ϊ1


#define FONTINFOADDR      ((u32)0)   //�ֿ�ͷ��ʼ��ַ
#define FONTSECSIZE       ((u32)((766080+174344)/4096)+1)         //�ֿ���������


#if PACK_EN 
__packed typedef struct 
{
	u8  fontok;				//�ֿ���ڱ�־��0XAA���ֿ��������������ֿⲻ����
	u32 ugbkaddr; 		//unigbk�ĵ�ַ
	u32 ugbksize;			//unigbk�Ĵ�С	 
	u32 f12addr;			//gbk12��ַ	
	u32 gbk12size;		//gbk12�Ĵ�С	 
	u32 f16addr;			//gbk16��ַ
	u32 gbk16size;		//gbk16�Ĵ�С		 
	u32 f24addr;			//gbk24��ַ
	u32 gkb24size;		//gbk24�Ĵ�С 
}_fontinfo; 		

#else
typedef struct               //���ֽڶ���
{
	u8  fontok;				//�ֿ���ڱ�־��0XAA���ֿ��������������ֿⲻ����
	u32 ugbkaddr; 		//unigbk�ĵ�ַ
	u32 ugbksize;			//unigbk�Ĵ�С	 
	u32 f12addr;			//gbk12��ַ	
	u32 gbk12size;		//gbk12�Ĵ�С	 
	u32 f16addr;			//gbk16��ַ
	u32 gbk16size;		//gbk16�Ĵ�С		 
	u32 f24addr;			//gbk24��ַ
	u32 gkb24size;		//gbk24�Ĵ�С 
}_fontinfo;	

#endif

extern _fontinfo ftinfo;	//�ֿ���Ϣ�ṹ��

void convertToLittleEndian(u32 *date);   //��С��ת������    ���ת����С��   С��ת���ɴ��

u8 OLED_TXT_Init(void);     //�ֿ��ʼ��   0�ɹ�  1ʧ��

void Get_HzMat(u8 *code_date,u8 *mat);  //code_date �ַ����Ŀ�ʼ��ַ,GBK��  ���ֿ��в��ҳ���ģ mat  ���ݴ�ŵ�ַ

void OLED_GUI_Str(u8 x,u8 y,u8 *p,u8 mode,u8 play) ; //��ʾ���ֺ��ַ���





#endif
