#ifndef __TXT_H_
#define __TXT_H_	 
#include "sys.h"

void Moving_Station_New_File(u8 *pname);  //�ƶ�վ�ļ���
void Base_Station_New_File(u8 *pname) ;  //GPS �ļ�������
void Aerial_Point_New_File(u8 *pname);    //POS�����ļ���
void Gps_Point_New_File(u8 *pname);    //POS�����ļ���
void Camera_Count_New_File(u8 *pname); //������ո�����Event�¼��������ļ���
void Pos_Point_New_File(u8 *pname);    //POS�����ļ���
void New_Folder(u8 *pname);             //�ļ�����


void Get_System_Time(char *pname); //��ȡϵͳʱ��
extern u8 *Log_File_Name;	 //��־��¼�ļ�

 



#endif
