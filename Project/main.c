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

const u8 *FILE_CREATE_Error		=  "er00*00"   ;//�ļ�����ʧ��
const u8 *SRAM_Error					=  "er01*01"   ;//��������ʧ��
const u8 *NO_CARD_DETECT 			=  "er02*02"   ;  //û�м�⵽sd��

//#define rtk_error_test

void ADD_HCN_HEAD(void);
int main(void)
{
	MY_NVIC_SetVectorTable(FLASH_BASE,APP_START_ADDR-FLASH_BASE);    //�ж�������ƫ��
	//MY_NVIC_SetVectorTable(FLASH_BASE,0);
	Scan_SYS_Bootloader_Lock();   //����û������Ƿ�ΪBootloader��¼  ��������豸�Ի�
	delay_init();	    	 				//��ʱ������ʼ��	 
	NVIC_Configuration(); 	 			//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	mallco_dev.init();	   	      //��ʼ���ڴ��
	OLED_Init();                  //OLED��ʼ��
	LED_Init();                   //LED��ʼ��
  GPS_PV_Init();                //GPS PV���ż��  
	SD_SACN_Init();               //SD��������
	exfuns_init();		            //Ϊfatfs��ر��������ڴ�
	

	
	#ifdef rtk_error_test
	Uart3_Init(115200);	//���ڳ�ʼ������ռ���ȼ�0�������ȼ�2
	USART_Cmd(USART3, ENABLE);
	
	
	while(1)
	{
		delay_ms(1000);
	Uart3_send_str((u8*)FILE_CREATE_Error,strlen((const char *)FILE_CREATE_Error));
	//Uart3_send_str((u8*)SRAM_Error,strlen((const char *)SRAM_Error));
	//Uart3_send_str((u8*)NO_CARD_DETECT,strlen((const char *)NO_CARD_DETECT));//error_02��ⲻ��SD��  ���ɿط���Ϣ
	}
	#endif
	
  //ָ�뻺������-----------------------------------------------------------------
	if(SRAM_DATA_Init()==1)   //ֻҪ��һ������ʧ�� �ʹ���
	{
		OLED_BUF_Clear();
		OLED_ShowString(0,32,"SRAM Error!",16,1,2);
	  while(1)
		{
		 	LED1 =! LED1; 	//��ʼ��ʧ�ܣ�LED��˸��ʾ
			delay_ms(1000);
			Uart3_send_str((u8*)SRAM_Error,strlen((const char *)SRAM_Error));//error_01
		}
	}
//----�����������---------------------------------------------------------------
		
	Movie_Show_Img(LOGO_YW);//��ʾLOGO
	delay_ms(3000);
		//OLED_ShowString(80,16," WANGXINWANG",16,1,1);
	//delay_ms(3000);
	//��ʾ�汾����Ȩ----------------------------
	Play_RTK_Copyright();
	delay_ms(3000) ;
	//OLED_ShowString(80,16," WANGXINWANG",16,1,1);
	//���SD�����ʼ����SD��δ���룬��ѭ����LED��˸��ʾ
//	if(SD_Init()||(SD_SACN!=0)||(KMM!=1))   //��һ��ɨ��SD���Ƿ����   ��ⲻ��SD��  
	if(SD_Init()||(KMM!=1))   //��һ��ɨ��SD���Ƿ����   ��ⲻ��SD��  
	{
	  Movie_Show_Img(SD_ERR);   //SD ������ͼƬ��ʾ   				
//	  while(SD_Init()||(SD_SACN!=0)||(KMM!=1))
		while(SD_Init()||(KMM!=1))
	  {
		  OLED_Gram_Inverse();   //�Դ淴ɫ ��ʾ		
	    LED0 =! LED0;//DS0��˸
	    delay_ms(1000);
			
			Uart3_send_str((u8*)NO_CARD_DETECT,strlen((const char *)NO_CARD_DETECT));//error_02��ⲻ��SD��  ���ɿط���Ϣ
	  }
	}
	f_mount(fs[0],"0:",1)	;				//����SD��
	SD_Init() ;

 //SD����ʼ���ɹ���ʾ����ʾSD������
	Movie_Show_Img(SD_INIT) ;   //SD�� ���ֳ�ʼ������
	OLED_ShowNum(40,16,SDCardInfo.CardCapacity>>20,5,16,1);//��ʾSD������
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

//��ʼ���ֿ⼰FLASHͼƬ���-------------------------------------------
//	if(font_init()>0)		  //��ʼ���ֿ�  ʧ�ܷ��� 1
//	{
//	  if(update_font()>0)   
//		{
//		   while(1) ;    //SD����û���ֿ��ļ� �����ڴ�����ʧ��  ���� SD���γ�
//		}
//	}
//FLASH�ڲ�ͼƬ��飬Ҳ�ɲ����----------------------------------------

//--------------------------------------------------------------------	
//�����ڲ�RTC---------------------------------------------------	
	Movie_Show_Img(RC_S);  ////RTC���ó�ʼ��
	while(RTC_Init())  //RTC��ʼ������ռ���ȼ�2�������ȼ�1  
	{
		LED1 =! LED1; 	//��ʼ��ʧ�ܣ�LED��˸��ʾ
		OLED_BUF_Clear();  //����Դ�
  		OLED_ShowString(0,16,"32.768KHZ Error",16,1,2);   //����ԭ�� ֻ�о�������
		delay_ms(300);
   };
	delay_ms(500);
	Movie_Show_Img(RC_E);  //RTC�������
	delay_ms(500);
	
//SD�� �����ļ� ��������   ���GPS�忨�Ƿ�����----------------------------------------------
    NO_CONFIG_FILE_EXE();	 
//�ϵ��һ��RTCʱ��GPSʱ������У׼����-------------------------------------------------------
	//Uart2_Init(115200);	//���ڳ�ʼ������ռ���ȼ�0�������ȼ�2
	USART_Cmd(USART2, DISABLE);                    //�رմ���
	Uart3_Init(115200);	//���ڳ�ʼ������ռ���ȼ�0�������ȼ�2
	USART_Cmd(USART3, DISABLE);                    //�رմ���
	USART_Cmd(USART1, DISABLE);                    //�رմ���,����1��ʼ����NO_CONFIG_FILE_EXE();  Uart1_Init(baud);
	LED1=0;
	Movie_Show_Img(GPS_OK);  //GPSʱ��У׼���ݳɹ�   ��λ�ɹ�����
	delay_ms(1000);
	
	Movie_Show_Img(File_S) ;    //�ļ�������ʼ����
	delay_ms(500);
	
//GPS�����ļ�----------------------------------------------------	
//��������GPS�켣���ļ��ͺ��ĵ�ʱ���ļ����ļ��У���ʱ������---------------------------------
	New_Folder(Folder_Name);  //�����ļ���     �������ʱ������
	SD_Res_STA=f_mkdir((const TCHAR*)Folder_Name);  //�����ļ���
	
	//������¼GPS�켣���ļ�,�ļ���Moving_Station_+ʱ��
	Moving_Station_New_File(Moving_Station_File_Name); //�����ļ���
	SD_Res_STA = f_open(file, (const TCHAR*)Moving_Station_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�  ���û���ļ� �򴴽��ļ�  
	SD_Res_STA = f_lseek(file,file ->fsize); //�ƶ�ָ�뵽��β 
	SD_Res_STA = f_close(file);   //�رմ򿪵��ļ�
	
	//������¼���ĵ�ʱ����ļ�,�ļ���Aerial_Point_+ʱ��
	Aerial_Point_New_File(Aerial_Point_File_Name);  //�����ļ��� ���ĵ�ʱ�������ļ�
	SD_Res_STA = f_open(file, (const TCHAR*)Aerial_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�
	f_lseek(file,file ->fsize); //�ƶ�ָ�뵽��β
	SD_Res_STA = f_close(file);   //�رմ򿪵��ļ�
	
	Gps_Point_New_File(Gps_Point_File_Name);  //�����ļ��� ���ĵ�ʱ�������ļ�
	SD_Res_STA = f_open(file, (const TCHAR*)Gps_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�
	f_lseek(file,file ->fsize); //�ƶ�ָ�뵽��β
	SD_Res_STA = f_close(file);   //�رմ򿪵��ļ�
	
	Pos_Point_New_File(Pos_Point_File_Name);  //�����ļ��� ���ĵ�ʱ�������ļ�
	SD_Res_STA = f_open(file, (const TCHAR*)Pos_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�
	f_lseek(file,file ->fsize); //�ƶ�ָ�뵽��β
	SD_Res_STA = f_close(file);   //�رմ򿪵��ļ�
//�ļ���������ж�------------------------------------------------------
	if(SD_Res_STA!=FR_OK)
	{
	  Movie_Show_Img(FILE_ERR) ;    //�ļ�����ʧ�ܽ���
		LED0 = 1;   //Ϩ��
	  while(1)
		{
//			if(SD_SACN==0)   //SD������
//			{
//			  delay_ms(1000);
//				if(SD_SACN==0)
//				{
				  Sys_Soft_Reset();  //ϵͳ�����λ		
//				}	
//			}
		  OLED_Gram_Inverse();   //�Դ淴ɫ ��ʾ	
			LED1 =! LED1; 	//��ʼ��ʧ�ܣ�LED��˸��ʾ
			delay_ms(1000);
			Uart3_send_str((u8*)FILE_CREATE_Error,strlen((const char *)FILE_CREATE_Error));//error_00��ⲻ��SD��  ���ɿط���Ϣ
		}
	}
//---------------------------------------------------------------------
	Movie_Show_Img(File_E) ;    //�ļ�������ɽ���
	delay_ms(500);

//��ʼ������-------------------------------------------------------
	LED0_OFF;                               //�رտ���ָʾ��
	Write_Counter  = 0;                     //SD��дGPS���ݴ�������
	Write_Counter_CAMM = 0 ;                //���մ�������
	
//ϵͳ����ָ�뻺������-----------------------------------------------------------------
	if(SYS_START_Init()==1)   //ֻҪ��һ������ʧ�� �ʹ���
	{
		OLED_BUF_Clear();
		OLED_ShowString(0,16,"SYSTEM START",16,1,2);
		OLED_ShowString(0,32,"SRAM Error!",16,1,2);
	  while(1)
		{
		 	LED1 =! LED1; 	//��ʼ��ʧ�ܣ�LED��˸��ʾ
			delay_ms(300);
		}
	}
	
	PLAY_DATE_Init();                       //��ʼ����ʾ����	
	Cam_FIFO_CLR(TIME_FIFO);                //��տ���ʱ���¼FIFO
	
//΢�뼶��ʱ��ʹ��---------------------------------------------------
	Tim4_Init(1000-1);  //1MHz��ʱ�� 1000 �ж����� 1mS   ��������״̬��ʱ��� �������������� �����ڿ�������ָʾ��
  // Tim4_Init(3000-1);  //1MHz��ʱ�� 1000 �ж����� 1mS   ��������״̬��ʱ��� �������������� �����ڿ�������ָʾ��
//���Ŀ����ж�---------------------------------
	Exit_Init();
	
	ADD_HCN_HEAD();//���HCN�ļ�ͷ
	
  	USART1_RXD_DMA_Init();                        //����1 DMA���ճ�ʼ��
	USART_Cmd(USART2, ENABLE);                    //��������
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
	
	USART_Cmd(USART1, ENABLE);                    //��������
	USART_Cmd(USART3, ENABLE);                    //��������
	//ADD_HCN_HEAD();//���HCN�ļ�ͷ
 while(1)
 {
	//GPS����д��SD��--------------------------------
	 if(RX_DMA_FLAT > 0 ) 	//���DMA������һ���������
	 {
   //GPS����д���ļ�--------------------------------------------------------------------------------------------		 
		 SD_Res_STA = f_open(file, (const TCHAR*)Moving_Station_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�
		 SD_Res_STA = f_lseek(file,file ->fsize); //�ƶ�ָ�뵽��β
		 if(RX_DMA_Nuber == 0 ) //������ڿ�ʼд����0  ˵������1�������
		 {
			 SD_Res_STA = f_write(file,RX_DMA_BUFF1,DMA_BUFF_SIZE,&br);			 //д���ļ�   ���� 1 ����д��SD��
		 }
		 else  //˵������ 0 ���������
		 {
			 SD_Res_STA = f_write(file,RX_DMA_BUFF0,DMA_BUFF_SIZE,&br);			 //д���ļ�	  д���� 0  ����д��SD��
		 }
		 	SD_Res_STA = f_close(file);   //�رմ򿪵��ļ�
		  CLR_RX_DMA_UP ;    //���д���
		 	LED1 =! LED1; 
			Write_Counter++;       //дSD������ 
	 }
	 if(gps_write_data_flag==1)    //Event �¼�
	 {
		 gps_write_data_flag=0;
		 sprintf((char *)Write_Buff_gps,"%s",gps_timea_buff);
	  SD_Res_STA=f_open(ftemp_gps, (const TCHAR*)Gps_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�  		
	  SD_Res_STA=f_lseek(ftemp_gps,ftemp_gps ->fsize); //�ƶ�ָ�뵽��β
		SD_Res_STA = f_write(ftemp_gps,Write_Buff_gps,strlen((char *)Write_Buff_gps),&br);			 //д���ļ�
	  SD_Res_STA=f_close(ftemp_gps);   
	 }
	  if(pos_first_flag==1)
	 {
		  pos_first_flag=0;
		 	 sprintf((char *)Write_Buff_pos,"%s\n",pos_item);
	  SD_Res_STA=f_open(ftemp_pos, (const TCHAR*)Pos_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�  		
	  SD_Res_STA=f_lseek(ftemp_pos,ftemp_pos ->fsize); //�ƶ�ָ�뵽��β
		SD_Res_STA = f_write(ftemp_pos,Write_Buff_pos,strlen((char *)Write_Buff_pos),&br);			 //д���ļ�
	  SD_Res_STA=f_close(ftemp_pos);   
	     
	 }
//	 if(pos_jilu_flag==1)
//	 {
//		  pos_jilu_flag=0;
//		   pos_number++;
//		 pos_id++;
//		 RTC_Get();//����ʱ��
//		 sprintf((char *)Write_Buff_pos,"%ld\t%04d-%02d-%02dT%02d:%02d:%02d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\tDSC_%04ld\r\n",pos_id,calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,platitude,plongitude,phigh,ppitch,prow,pdirct,pcog,pground_speed,pos_number);     
//	  SD_Res_STA=f_open(ftemp_pos, (const TCHAR*)Pos_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�  		
//	  SD_Res_STA=f_lseek(ftemp_pos,ftemp_pos ->fsize); //�ƶ�ָ�뵽��β
//		SD_Res_STA = f_write(ftemp_pos,Write_Buff_pos,strlen((char *)Write_Buff_pos),&br);			 //д���ļ�
//	  SD_Res_STA=f_close(ftemp_pos);   
//	     
//	 }

	 if(pos_jilu_flag==1)
	 {
		  pos_jilu_flag=0;
		   pos_number++;
		 	 sprintf((char *)Write_Buff_pos,"DSC%05ld\t%f\t%f\t%f\t%f\t%f\t%f\r\n",pos_number,plongitude,platitude,phigh,ppitch,prow,pdirct);
	  SD_Res_STA=f_open(ftemp_pos, (const TCHAR*)Pos_Point_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�  		
	  SD_Res_STA=f_lseek(ftemp_pos,ftemp_pos ->fsize); //�ƶ�ָ�뵽��β
		SD_Res_STA = f_write(ftemp_pos,Write_Buff_pos,strlen((char *)Write_Buff_pos),&br);			 //д���ļ�
	  SD_Res_STA=f_close(ftemp_pos);   
	    
	 }

	 
   PLAY_DATE_TASK();   //��ʾ����

//�ļ�����ʧ��-------------------------------------------------------------------------------------
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
//	SD_Res_STA = f_open(file, (const TCHAR*)Moving_Station_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�
//	SD_Res_STA = f_lseek(file,file ->fsize); //�ƶ�ָ�뵽��β
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
		"Date:%4d/%d/%d\r\n"//�ļ���¼����
		"Model:Unknown\r\n"
		"AntHigh:%0.4f\r\n"//���߸�
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
		
	SD_Res_STA = f_open(file, (const TCHAR*)Moving_Station_File_Name, FA_OPEN_ALWAYS | FA_WRITE); 	//���ļ�
	SD_Res_STA = f_lseek(file,file ->fsize); //�ƶ�ָ�뵽��β
	SD_Res_STA = f_write(file,Hcn_Head,strlen((char *)Hcn_Head),&br);
	SD_Res_STA = f_close(file);
}

 


