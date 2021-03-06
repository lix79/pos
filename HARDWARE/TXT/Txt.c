#include "Txt.h"
#include "Rtc.h"
#include "stdio.h"
#include "Malloc.h"  
#include "string.h"
#include "SRAM_DATA.h"

//通过时间获取文件名
//仅限在sd卡保存,不支持flash disk保存
//组合成:形如"0:20120321210633.txt"的文件名
u8 *Log_File_Name = "0:/log.txt";
void Moving_Station_New_File(u8 *pname)  //GPS 文件数据名
{	 
	RTC_Get();//更新时间
 	sprintf((char*)pname,"0:/%s/Moving_Station_%04d%02d%02d%02d%02d%02d.HCN",Folder_Name,calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
} 

void Base_Station_New_File(u8 *pname)  //GPS 文件数据名
{	 
	RTC_Get();//更新时间
 	sprintf((char*)pname,"0:/%s/Base_Station_%04d%02d%02d%02d%02d%02d.txt",Folder_Name,calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
} 

void Aerial_Point_New_File(u8 *pname)
{
	RTC_Get();//更新时间
 	sprintf((char*)pname,"0:/%s/Aerial_Point_%04d%02d%02d%02d%02d%02d.txt",Folder_Name,calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
}
void Gps_Point_New_File(u8 *pname)
{
	RTC_Get();//更新时间
 	sprintf((char*)pname,"0:/%s/Gps_Point_%04d%02d%02d%02d%02d%02d.UBT",Folder_Name,calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
}
void Camera_Count_New_File(u8 *pname)
{
	RTC_Get();
 	sprintf((char*)pname,"0:/%s/Camera_Count_%04d%02d%02d%02d%02d%02d.UBT",Folder_Name,calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
}
void Pos_Point_New_File(u8 *pname)
{
	RTC_Get();//更新时间
 	sprintf((char*)pname,"0:/%s/Pos_Point_%04d%02d%02d%02d%02d%02d.txt",Folder_Name,calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
}
void New_Folder(u8 *pname)
{
	RTC_Get();//更新时间
	sprintf((char*)pname,"%04d%02d%02d%02d%02d%02d",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
}

void Get_System_Time(char *pname)
{
	RTC_Get();
	sprintf((char*)pname,"systime is: %04d-%02d-%02d %02d:%02d:%02d\r\n",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
}
















