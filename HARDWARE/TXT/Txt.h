#ifndef __TXT_H_
#define __TXT_H_	 
#include "sys.h"

void Moving_Station_New_File(u8 *pname);  //移动站文件名
void Base_Station_New_File(u8 *pname) ;  //GPS 文件数据名
void Aerial_Point_New_File(u8 *pname);    //POS数据文件名
void Gps_Point_New_File(u8 *pname);    //POS数据文件名
void Camera_Count_New_File(u8 *pname); //相机拍照个数（Event事件个数）文件名
void Pos_Point_New_File(u8 *pname);    //POS数据文件名
void New_Folder(u8 *pname);             //文件夹名


void Get_System_Time(char *pname); //获取系统时间
extern u8 *Log_File_Name;	 //日志记录文件

 



#endif
