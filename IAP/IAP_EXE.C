#include "IAP_EXE.H"
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
#include "exfuns.h"

///////////////////////////////�����ļ���,ʹ��malloc��ʱ��////////////////////////////////////////////
typedef void (*AppFun)(void);                 //ָ������ָ��


AppFun    AppStart ;                          //APP MAIN()
	
IAP_CRC_DATA_TYP   *APP_File_DATA   ;      //APP�ļ�   У������
IAP_CRC_DATA_TYP  * FLASH_DATA      ;      //FLASH �����У������ 
 


void SYS_IAP_Init(void)  //IAP Ӳ����ʼ��
{
	SystemInit(); 			 					//ϵͳʱ�ӳ�ʼ��Ϊ72M	  SYSCLK_FREQ_72MHz
	delay_init();	    	 				  //��ʱ������ʼ��	 
	NVIC_Configuration(); 	 			//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	OLED_Init();                  //OLED��ʼ��
	mallco_dev.init();            //�ڴ������ʼ��
	exfuns_init();                //�ļ����������ڴ�����
	STM32_CRC_Init();             //CRC ��ʼ��
}



void APP_STAR_FUN(u32 AppAddr)  //����APP
{
	if(((*(vu32*)AppAddr)&0x2ffe0000) == 0x20000000)
	{
		AppStart = (AppFun)(*(vu32*)(AppAddr+4));
		MSR_MSP(*(vu32*)AppAddr);
		AppStart();
	}
}


void SYS_FLASH_Update(void)   //��¼FLASH   �ɹ� 0    ���� 1
{
	u32  addrx = 0 ,endaddr = 0 ;               //��ʼ��ַ   ��ֹ��ַ
	u8  *iap_buffer = 0 ;                       //���ݻ���  
	br = f_open(file,(const TCHAR *)Updata_Path,FA_OPEN_EXISTING|FA_READ);   //���ļ�
  if(br != FR_OK )
	{
		return ;
	}
	OLED_Init();                      //OLED ��ʼ��
	OLED_ShowString(16,48,(const u8 *)"Updating ...",16,1,2);		
	OLED_ShowString(80,32,(const u8 *)"YDZ",16,1,2);		
  ErasePage_FLASH_APP_CRC_DATA();                        //����IAP ���� 
	ErasePage_FLASH_APP_FLASH(APP_File_DATA->File_SIZE);   //�����û�FLASH ����
  addrx = APP_File_DATA->APP_STAR_ADDR;                  //��ʼ��ַ
  endaddr = APP_File_DATA->APP_END_ADDR;                 //�����ĵ�ַΪ��ʼ��ַ���ļ���С
	iap_buffer = (u8 * ) mymalloc(SECTION_PAGE_SIZE ) ;    //�����ڴ�   2048 �ֽ�
	while(1)
	{
		br = f_read(file,iap_buffer,SECTION_PAGE_SIZE,&bw);//��ȡ2048���ֽ�
		if((br != FR_OK )|| (bw == 0 ))//�����ȡʧ�ܻ��߶�ȡ���
		{
		  break;  //�˳�  д�����
		}
		
		FLASH_Unlock();						//����
		STMFLASH_Write_NoCheck(addrx,(u16*)iap_buffer,bw/2);   //д��FLASH      ������Ƿ����  ��Ϊ֮ǰ�Ѿ�������
		FLASH_Lock();//����
		
		addrx += bw ;                                          //��ַƫ��
		
		IAP_EXE_PALY_prog(32,32,endaddr-APP_START_ADDR,addrx-APP_START_ADDR );		//���½���
		if(addrx>=endaddr)                                                        //д����
		{
			break;
		}
	 }
	 myfree(iap_buffer);  //�ͷ��ڴ�
	 f_close(file);  //�ر��ļ�
}



void RTK01_IAP_EXE(void)  //ִ��IAP
{
 APP_File_DATA  = (IAP_CRC_DATA_TYP*)mymalloc(sizeof(IAP_CRC_DATA_TYP));		//�����ڴ�
 FLASH_DATA     = (IAP_CRC_DATA_TYP*)mymalloc(sizeof(IAP_CRC_DATA_TYP));		//�����ڴ�
 if(SD_Init() ==0) //SD����ʼ���ɹ�
 {
 	if(f_mount(fs[0],"0:",1) == FR_OK)  //�����ļ�ϵͳ�ɹ�
	{  
	 br = f_open(file,(const TCHAR *)Updata_Path,FA_READ);   //�Ѷ���ʽ���ļ�
   
	 if(br != FR_OK ) //�����ʧ��
	 {
	   f_mount(fs[0],"0:",0) ;  //����ļ�ϵͳ
		 return ;                 //�˳�
	 } 
	 if( (file->fsize <(20*1024))||(file->fsize >(SECTION_PAGE_MAX*SECTION_PAGE_SIZE)) ||(file->fsize%4 > 0)) //����ļ���СС��20K ���߳���FLASH��С ���� ��С����4�ı��� ���ļ�������
	 {
	  f_close(file);   //�ر��ļ�
	 	f_mount(fs[0],"0:",0) ;    //����ļ�ϵͳ
		return ;                 //�˳�
	 }
	 f_close(file);   //�ر��ļ�
	 
//��BIN �ļ�  ������Ҫ����-------------------------------------------
  if(Read_FILE_DATA_CRC(APP_File_DATA) == 0 )         //�õ��ļ���Ϣ  �� У��ֵ
	{
	  Read_FLASH_APP_CRC_DATA(FLASH_DATA);              //�õ�FLASH �� ��У������
	 //�ȶ�����   ���Ƿ���Ҫ����-------------------------------------
	 if( Bijiao_CRC_DATA(APP_File_DATA,FLASH_DATA) >0 ) //�������ͬ  ����Ҫ����
	 {
	   SYS_FLASH_Update() ;      //��¼FLASH
		 f_mount(fs[0],"0:",0) ;   //����ļ�ϵͳ
		 if( br == FR_OK )         //�����������
		 {
	     Write_FLASH_APP_CRC_DATA(APP_File_DATA);   //�ļ���Ϣ У������ д��FLASH
			 delay_ms(500);
		 }
	 }
	 else
	 {
	 	f_mount(fs[0],"0:",0) ;  //����ļ�ϵͳ
	  return ;                 //�˳�
	 }
	}
 	else
	{
	 f_mount(fs[0],"0:",0) ;    //����ļ�ϵͳ
	 return ;                   //�˳�	
	 }
  }
 }
}

//�õ��ļ���Ϣ  �� У��ֵ
u8 Read_FILE_DATA_CRC(IAP_CRC_DATA_TYP * IAP_CRC_DAT )    //����0 �ɹ�    1ʧ��
{
	u16  i = 0 ;                  //����
	u16  CRC_Number  = 0 ;        //��������У����    �������ļ� ��2048�ֽ�Ϊһ��    һ�ζ�У��
	u8   *iap_buffer = 0 ;        //���ݻ��� 
  iap_buffer = (u8 * ) mymalloc( CRC_SECTION_SIZE ) ;  //�����ڴ�      ����һ��CRCУ��δ�С
  br = f_open(file,(const TCHAR *)Updata_Path,FA_OPEN_EXISTING|FA_READ);   //�Ѷ���ʽ���ļ�
	if(br == FR_OK)    //����򿪳ɹ�
	{
		IAP_CRC_DAT->IAP_CRC_DATA_HEAD   = IAP_CRC_HEAD ;                                          //���ͷ
	  IAP_CRC_DAT->File_SIZE           =  file->fsize ;                                          //�õ��ļ���С
	  IAP_CRC_DAT->APP_STAR_ADDR       =  APP_START_ADDR ;                                       //APP��ʼ��ַ
	  IAP_CRC_DAT->APP_END_ADDR        =  IAP_CRC_DAT->APP_STAR_ADDR + IAP_CRC_DAT->File_SIZE ;  //APP������ַ
		
		//��ȡ�ļ�У��ֵ-------------------------------------------------
		CRC_Number = file->fsize/CRC_SECTION_SIZE ;             //�õ��ֶ���
    for( i = 0 ; i <CRC_Number ; i ++)                       //�õ��ֶ�CRCֵ
		{
      br = f_lseek(file,i * CRC_SECTION_SIZE );             //�ƶ�ָ���дָ��
	    br = f_read(file,iap_buffer,CRC_SECTION_SIZE,&bw);    //��ȡ2048���ֽ�
		  if((br == FR_OK)&&(bw == CRC_SECTION_SIZE ))          //����򿪳ɹ�
		  {
			  IAP_CRC_DAT->DATA_CRC[i]	 = ST32_CRC_DATA_Str((u32 *)iap_buffer,CRC_SECTION_SIZE/4);   //����CRCֵ
	 	  }
		  else
		  {
			  myfree(iap_buffer);  //�ͷ��ڴ�
	      f_close(file);       //�ر��ļ�
	      return 1;  //���ش���
	    }		
		}
		CRC_Number = file->fsize%CRC_SECTION_SIZE ;                //�õ�ʣ����
		if( CRC_Number > 0)
		{
      br = f_lseek(file,i * CRC_SECTION_SIZE );                //�ƶ�ָ���дָ��
	    br = f_read(file,iap_buffer,CRC_Number,&bw);             //��ȡʣ���ֽ�		
	    if((br == FR_OK)&&(bw == CRC_Number ))                   //����򿪳ɹ�
		  {
			  IAP_CRC_DAT->DATA_CRC[i] = ST32_CRC_DATA_Str((u32 *)iap_buffer,CRC_Number/4);   //����CRCֵ
	 	  }
			else
			{
			  myfree(iap_buffer);  //�ͷ��ڴ�
	      f_close(file);       //�ر��ļ�
	      return 1;  //���ش���	
			}
	  }
	}
	myfree(iap_buffer);  //�ͷ��ڴ�
	f_close(file);       //�ر��ļ�
	if(br == FR_OK)
	{
	  return 0 ;   //���سɹ�
	}
	return 1 ;    //����ʧ��
}






