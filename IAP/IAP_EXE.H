#ifndef   __IAP_EXE_H__
#define   __IAP_EXE_H__

#include "SYS.H"
#include "ff.h"
#include "IAP_DATA.H"

extern IAP_CRC_DATA_TYP   *APP_File_DATA    ;      //APP�ļ�   У������
extern IAP_CRC_DATA_TYP  * FLASH_DATA       ;      //FLASH �����У������ 



void SYS_IAP_Init(void) ;         //IAP Ӳ����ʼ��
void RTK01_IAP_EXE(void);         //ִ��IAP
void APP_STAR_FUN(u32 AppAddr) ;  //����APP	

//�õ��ļ���Ϣ  �� У��ֵ
u8 Read_FILE_DATA_CRC(IAP_CRC_DATA_TYP * IAP_CRC_DAT ) ;   //����0 �ɹ�    1ʧ��




#endif






