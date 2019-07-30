#include "delay.h"
#include "sys.h"
#include "USART1_DMA.h"
#include "Led.h"
#include "Tim4.h"
#include "Oled.h"
#include "Malloc.h"
#include "Rtc.h"
#include "exfuns.h"
#include "Exit.h"
#include "Txt.h"
#include "Gps.h"
#include "PLAY_DATA.H"
#include "sdio_sdcard.h"
#include "SRAM_DATA.h"
#include "GPS_STA.h"
#include "OLED_GUI.h"
#include "GPS_DATE.h"
#include <string.h>
#include "file_ini.h"
#include "Cam_FIFO.h"
#include "OLED_BMP.h"
#include "OLED_TXT.h"
#include "fontupd.h"
#include "IAP_DATA.H"
#include "COPYRIGHT.h"
#include "stdlib.h"	 
extern unsigned char gps_write_data_flag;
extern  char *gps_timea_buff;
//extern  char *pos_timea_buff;
extern char pos_jilu_flag;

extern double plongitude;
extern double platitude;
extern float phigh;
extern float ppitch;
extern float prow;
extern float pdirct;
unsigned char pos_first_flag=1;
 //char *pos_item="ID          time            latitude            longtitude            height          row         pitch        yaw        cog     GroundSpeed   PosNumber";
char *pos_item="ID          latitude            longtitude            height          row         pitch        yaw";
unsigned long pos_number=0;
unsigned long pos_id=0;

const u8 *FILE_CREATE_Error		=  "er00*00"   ;//文件创建失败
const u8 *SRAM_Error					=  "er01*01"   ;//变量申请失败
const u8 *NO_CARD_DETECT 			=  "er02*02"   ;  //没有检测到sd卡

//#define rtk_error_test

void ADD_HCN_HEAD(void);
int main(void)
{
	MY_NVIC_SetVectorTable(FLASH_BASE,APP_START_ADDR-FLASH_BASE);    //中断向量表偏移
	//MY_NVIC_SetVectorTable(FLASH_BASE,0);
	Scan_SYS_Bootloader_Lock();   //检测用户代码是否为Bootloader烧录  如果不是设备自毁
	delay_init();	    	 				//延时函数初始化	 
	NVIC_Configuration(); 	 			//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	mallco_dev.init();	   	      //初始化内存池
	OLED_Init();                  //OLED初始化
	LED_Init();                   //LED初始化
  GPS_PV_Init();                //GPS PV引脚检测  
	SD_SACN_Init();               //SD卡插入检测
	exfuns_init();		            //为fatfs相关变量申请内存
	

	
	#ifdef rtk_error_test
	Uart3_Init(115200);	//串口初始化，抢占优先级0，子优先级2
	USART_Cmd(USART3, ENABLE);
	
	
	while(1)
	{
		delay_ms(1000);
	Uart3_send_str((u8*)FILE_CREATE_Error,strlen((const char *)FILE_CREATE_Error));
	//Uart3_send_str((u8*)SRAM_Error,strlen((const char *)SRAM_Error));
	//Uart3_send_str((u8*)NO_CARD_DETECT,strlen((const char *)NO_CARD_DETECT));//error_02检测不到SD卡  给飞控发消息
	}
	#endif
	
  //指针缓存申请-----------------------------------------------------------------
	if(SRAM_DATA_Init()==1)   //只要有一个申请失败 就错误
	{
		OLED_BUF_Clear();
		OLED_ShowString(0,32,"SRAM Error!",16,1,2);
	  while(1)
		{
		 	LED1 =! LED1; 	//初始化失败，LED闪烁提示
			delay_ms(1000);
			Uart3_send_str((u8*)SRAM_Error,strlen((const char *)SRAM_Error));//error_01
		}
	}
//----缓存申请结束---------------------------------------------------------------
		
	Movie_Show_Img(LOGO_YW);//显示LOGO
	delay_ms(3000);
		//OLED_ShowString(80,16," WANGXINWANG",16,1,1);
	//delay_ms(3000);
	//显示版本及版权----------------------------
	Play_RTK_Copyright();
	delay_ms(3000) ;
	//OLED_ShowString(80,16," WANGXINWANG",16,1,1);
	//检测SD卡与初始化，SD卡未插入，死循环，LED闪烁提示
//	if(SD_Init()||(SD_SACN!=0)||(KMM!=1))   //第一次扫描SD卡是否存在   检测不到SD卡  
	if(SD_Init()||(KMM!=1))   //第一次扫描SD卡是否存在   检测不到SD卡  
	{
	  Movie_Show_Img(SD_ERR);   //SD 卡故障图片显示   				
//	  while(SD_Init()||(SD_SACN!=0)||(KMM!=1))
		while(SD_Init()||(KMM!=1))
	  {
		  OLED_Gram_Inverse();   //显存反色 显示		
	    LED0 =! LED0;//DS0闪烁
	    delay_ms(1000);
			
			Uart3_send_str((u8*)NO_CARD_DETECT,strlen((const char *)NO_CARD_DETECT));//error_02检测不到SD卡  给飞控发消息
	  }
	}
	f_mount(fs[0],"0:",1)	;				//挂载SD卡
	SD_Init() ;

 //SD卡初始化成功提示，显示SD卡容量
	Movie_Show_Img(SD_INIT) ;   //SD卡 汉字初始化界面
	OLED_ShowNum(40,16,SDCardInfo.CardCapacity>>20,5,16,1);//显示SD卡容量
	OLED_ShowString(80,16," MB",16,1,1);
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:
			OLED_ShowString(0,0,"Type:SDSC V1.1",16,1,1);
		break;
		
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:
			OLED_ShowString(0,0,"Type:SDSC V2.0",16,1,1);
		break;
		
		case SDIO_HIGH_CAPACITY_SD_CARD:
			OLED_ShowString(0,0,"Type:SDHC V2.0",16,1,1);
		break;
		
		case SDIO_MULTIMEDIA_CARD:
			OLED_ShowString(0,0,"Type:MMC Card",16,1,1);
		break;
	}	
	delay_ms(5000);

//初始化字库及FLASH图片检查-------------------------------------------
//	if(font_init()>0)		  //初始化字库  失败返回 1
//	{
//	  if(update_font()>0)   
//		{
//		   while(1) ;    //SD卡内没有字库文件 或者内存申请失败  或者 SD卡拔出
//		}
//	}
//FLASH内部图片检查，也可不检查----------------------------------------

//--------------------------------------------------------------------	
//配置内部RTC---------------------------------------------------	
	Movie_Show_Img(RC_S);  ////RTC配置初始化
	while(RTC_Init())  //RTC初始化，抢占优先级2，子优先级1  
	{
		LED1 =! LED1; 	//初始化失败，LED闪烁提示
		OLED_BUF_Clear();  //清除显存
  		OLED_ShowString(0,16,"32.768KHZ Error",16,1,2);   //错误原因 只有晶振不起振
		delay_ms(300);
   };
	delay_ms(500);
	Movie_Show_Img(RC_E);  //RTC配置完成
	delay_ms(500);
	
//SD卡 配置文件 配置任务   检测GPS板卡是否正常----------------------------------------------
    NO_CONFIG_FILE_EXE();	 
//上电第一次RTC时间GPS时间数据校准结束-------------------------------------------------------
	//Uart2_Init(115200);	//串口初始化，抢占优先级0，子优先级2
	USART_Cmd(USART2, DISABLE);                    //关闭串口
	Uart3_Init(115200);	//串口初始化，抢占优先级0，子优先级2
	USART_Cmd(USART3, DISABLE);                    //关闭串口
	USART_Cmd(USART1, DISABLE);                    //关闭串口,串口1初始化在NO_CONFIG_FILE_EXE();  Uart1_Init(baud);
	LED1=0;
	Movie_Show_Img(GPS_OK);  //GPS时间校准数据成功   定位成功界面
	delay_ms(1000);
	
	Movie_Show_Img(File_S) ;    //文件创建开始界面
	delay_ms(500);
	
//GPS数据文件----------------------------------------------------	
//创建保存GPS轨迹的文件和航拍点时间文件的文件夹，以时间命名---------------------------------
	New_Folder(Folder_Name);  //赋予文件名     保存快门时间数据
	SD_Res_STA=f_mkdir((const TCHAR*)Folder_Name);  //创建文件夹
	
	//创建记录GPS轨迹的文件,文件名Moving_Station_+时间
	Moving_Station_New_File(Moving_Station_File_Name); //赋予文件名
	SD_Res_STA = f_open(file, (const TCHAR*)Moving_Station_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件  如果没有文件 则创建文件  
	SD_Res_STA = f_lseek(file,file ->fsize); //移动指针到结尾 
	SD_Res_STA = f_close(file);   //关闭打开的文件
	
	//创建记录航拍点时间的文件,文件名Aerial_Point_+时间
	Aerial_Point_New_File(Aerial_Point_File_Name);  //赋予文件名 航拍点时间数据文件
	SD_Res_STA = f_open(file, (const TCHAR*)Aerial_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件
	f_lseek(file,file ->fsize); //移动指针到结尾
	SD_Res_STA = f_close(file);   //关闭打开的文件
	
	Gps_Point_New_File(Gps_Point_File_Name);  //赋予文件名 航拍点时间数据文件
	SD_Res_STA = f_open(file, (const TCHAR*)Gps_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件
	f_lseek(file,file ->fsize); //移动指针到结尾
	SD_Res_STA = f_close(file);   //关闭打开的文件
	
	Pos_Point_New_File(Pos_Point_File_Name);  //赋予文件名 航拍点时间数据文件
	SD_Res_STA = f_open(file, (const TCHAR*)Pos_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件
	f_lseek(file,file ->fsize); //移动指针到结尾
	SD_Res_STA = f_close(file);   //关闭打开的文件
//文件操作结果判定------------------------------------------------------
	if(SD_Res_STA!=FR_OK)
	{
	  Movie_Show_Img(FILE_ERR) ;    //文件创建失败界面
		LED0 = 1;   //熄灭
	  while(1)
		{
//			if(SD_SACN==0)   //SD卡插入
//			{
//			  delay_ms(1000);
//				if(SD_SACN==0)
//				{
				  Sys_Soft_Reset();  //系统软件复位		
//				}	
//			}
		  OLED_Gram_Inverse();   //显存反色 显示	
			LED1 =! LED1; 	//初始化失败，LED闪烁提示
			delay_ms(1000);
			Uart3_send_str((u8*)FILE_CREATE_Error,strlen((const char *)FILE_CREATE_Error));//error_00检测不到SD卡  给飞控发消息
		}
	}
//---------------------------------------------------------------------
	Movie_Show_Img(File_E) ;    //文件创建完成界面
	delay_ms(500);

//初始化变量-------------------------------------------------------
	LED0_OFF;                               //关闭快门指示灯
	Write_Counter  = 0;                     //SD卡写GPS数据次数清零
	Write_Counter_CAMM = 0 ;                //拍照次数清零
	
//系统运行指针缓存申请-----------------------------------------------------------------
	if(SYS_START_Init()==1)   //只要有一个申请失败 就错误
	{
		OLED_BUF_Clear();
		OLED_ShowString(0,16,"SYSTEM START",16,1,2);
		OLED_ShowString(0,32,"SRAM Error!",16,1,2);
	  while(1)
		{
		 	LED1 =! LED1; 	//初始化失败，LED闪烁提示
			delay_ms(300);
		}
	}
	
	PLAY_DATE_Init();                       //初始化显示界面	
	Cam_FIFO_CLR(TIME_FIFO);                //清空快门时间记录FIFO
	
//微秒级定时器使能---------------------------------------------------
	Tim4_Init(1000-1);  //1MHz定时器 1000 中断周期 1mS   快门引脚状态延时检测 用于消抖及干扰 及用于控制拍照指示灯
  // Tim4_Init(3000-1);  //1MHz定时器 1000 中断周期 1mS   快门引脚状态延时检测 用于消抖及干扰 及用于控制拍照指示灯
//航拍快门中断---------------------------------
	Exit_Init();
	
	ADD_HCN_HEAD();//添加HCN文件头
	
  	USART1_RXD_DMA_Init();                        //串口1 DMA接收初始化
	USART_Cmd(USART2, ENABLE);                    //开启串口
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
	
	USART_Cmd(USART1, ENABLE);                    //开启串口
	USART_Cmd(USART3, ENABLE);                    //开启串口
	//ADD_HCN_HEAD();//添加HCN文件头
 while(1)
 {
	//GPS数据写入SD卡--------------------------------
	 if(RX_DMA_FLAT > 0 ) 	//如果DMA缓存有一个接收完成
	 {
   //GPS数据写入文件--------------------------------------------------------------------------------------------		 
		 SD_Res_STA = f_open(file, (const TCHAR*)Moving_Station_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件
		 SD_Res_STA = f_lseek(file,file ->fsize); //移动指针到结尾
		 if(RX_DMA_Nuber == 0 ) //如果现在开始写缓存0  说明缓存1接收完成
		 {
			 SD_Res_STA = f_write(file,RX_DMA_BUFF1,DMA_BUFF_SIZE,&br);			 //写入文件   缓存 1 数据写入SD卡
		 }
		 else  //说明缓存 0 接收完成了
		 {
			 SD_Res_STA = f_write(file,RX_DMA_BUFF0,DMA_BUFF_SIZE,&br);			 //写入文件	  写缓存 0  数据写入SD卡
		 }
		 	SD_Res_STA = f_close(file);   //关闭打开的文件
		  CLR_RX_DMA_UP ;    //清除写标记
		 	LED1 =! LED1; 
			Write_Counter++;       //写SD卡次数 
	 }
	 if(gps_write_data_flag==1)    //Event 事件
	 {
		 gps_write_data_flag=0;
		 sprintf((char *)Write_Buff_gps,"%s",gps_timea_buff);
	  SD_Res_STA=f_open(ftemp_gps, (const TCHAR*)Gps_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件  		
	  SD_Res_STA=f_lseek(ftemp_gps,ftemp_gps ->fsize); //移动指针到结尾
		SD_Res_STA = f_write(ftemp_gps,Write_Buff_gps,strlen((char *)Write_Buff_gps),&br);			 //写入文件
	  SD_Res_STA=f_close(ftemp_gps);   
	 }
	  if(pos_first_flag==1)
	 {
		  pos_first_flag=0;
		 	 sprintf((char *)Write_Buff_pos,"%s\n",pos_item);
	  SD_Res_STA=f_open(ftemp_pos, (const TCHAR*)Pos_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件  		
	  SD_Res_STA=f_lseek(ftemp_pos,ftemp_pos ->fsize); //移动指针到结尾
		SD_Res_STA = f_write(ftemp_pos,Write_Buff_pos,strlen((char *)Write_Buff_pos),&br);			 //写入文件
	  SD_Res_STA=f_close(ftemp_pos);   
	     
	 }
//	 if(pos_jilu_flag==1)
//	 {
//		  pos_jilu_flag=0;
//		   pos_number++;
//		 pos_id++;
//		 RTC_Get();//更新时间
//		 sprintf((char *)Write_Buff_pos,"%ld\t%04d-%02d-%02dT%02d:%02d:%02d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\tDSC_%04ld\r\n",pos_id,calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,platitude,plongitude,phigh,ppitch,prow,pdirct,pcog,pground_speed,pos_number);     
//	  SD_Res_STA=f_open(ftemp_pos, (const TCHAR*)Pos_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件  		
//	  SD_Res_STA=f_lseek(ftemp_pos,ftemp_pos ->fsize); //移动指针到结尾
//		SD_Res_STA = f_write(ftemp_pos,Write_Buff_pos,strlen((char *)Write_Buff_pos),&br);			 //写入文件
//	  SD_Res_STA=f_close(ftemp_pos);   
//	     
//	 }

	 if(pos_jilu_flag==1)
	 {
		  pos_jilu_flag=0;
		   pos_number++;
		 	 sprintf((char *)Write_Buff_pos,"DSC%05ld\t%f\t%f\t%f\t%f\t%f\t%f\r\n",pos_number,plongitude,platitude,phigh,ppitch,prow,pdirct);
	  SD_Res_STA=f_open(ftemp_pos, (const TCHAR*)Pos_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件  		
	  SD_Res_STA=f_lseek(ftemp_pos,ftemp_pos ->fsize); //移动指针到结尾
		SD_Res_STA = f_write(ftemp_pos,Write_Buff_pos,strlen((char *)Write_Buff_pos),&br);			 //写入文件
	  SD_Res_STA=f_close(ftemp_pos);   
	    
	 }

	 
   PLAY_DATE_TASK();   //显示任务

//文件操作失败-------------------------------------------------------------------------------------
   SYS_Scan_TF_Err();
//--------------------------------------------------------------------------------------------------------------
 }
}

//float ant_height = 0.0;
//void ADD_HCN_HEAD(void)
//{
//	sprintf((char *)Hcn_Head,
//		"HUACENAV COLLECTED DATA FILE\r\n\
//		ver 01\r\n\
//		ReceiverID:123456\r\n\
//		Date:%4d/%d/%d\r\n\
//		Model:Unknown\r\n\
//		AntHigh:%0.4f\r\n\
//		AntType:3S-02-TSADM     NONE\r\n\
//		MeasuredTo:2\r\n\
//		X:\r\n\
//		Y:\r\n\
//		Z:\r\n\
//		MARKER NAME:Mr.T\r\n\
//		MARKER NUMBER:20190326\r\n\
//		REC #:\r\n\
//		REC TYPE:Unknown\r\n\
//		REC VERS:\r\n\
//		ANT #:\r\n\
//		ANT TYPE:\r\n\
//		INTERVAL:\r\n\
//		TIME OF FIRST OBS:\r\n\
//		LEAP SECONDS:\r\n",
//		calendar.w_year,calendar.w_month,calendar.w_date
//		,ant_height);
//		
//	SD_Res_STA = f_open(file, (const TCHAR*)Moving_Station_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件
//	SD_Res_STA = f_lseek(file,file ->fsize); //移动指针到结尾
//	SD_Res_STA = f_write(file,Hcn_Head,strlen((char *)Hcn_Head),&br);
//	SD_Res_STA = f_close(file);
//}

float ant_height = 0.0;
void ADD_HCN_HEAD(void)
{
	sprintf((char *)Hcn_Head,
		"HUACENAV COLLECTED DATA FILE\r\n"
		"ver 01\r\n"
		"ReceiverID:123456\r\n"
		"Date:%4d/%d/%d\r\n"//文件记录日期
		"Model:Unknown\r\n"
		"AntHigh:%0.4f\r\n"//天线高
		"AntType:3S-02-TSADM     NONE\r\n"
		"MeasuredTo:2\r\n"
		"X:\r\n"
		"Y:\r\n"
		"Z:\r\n"
		"MARKER NAME:Mr.T\r\n"
		"MARKER NUMBER:20190326\r\n"
		"REC #:\r\n"
		"REC TYPE:Unknown\r\n"
		"REC VERS:\r\n"
		"ANT #:\r\n"
		"ANT TYPE:\r\n"
		"INTERVAL:\r\n"
		"TIME OF FIRST OBS:\r\n"
		"LEAP SECONDS:\r\n",
		calendar.w_year,calendar.w_month,calendar.w_date
		,ant_height);
		
	SD_Res_STA = f_open(file, (const TCHAR*)Moving_Station_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//打开文件
	SD_Res_STA = f_lseek(file,file ->fsize); //移动指针到结尾
	SD_Res_STA = f_write(file,Hcn_Head,strlen((char *)Hcn_Head),&br);
	SD_Res_STA = f_close(file);
}

 


