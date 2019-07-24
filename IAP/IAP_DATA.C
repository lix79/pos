#include "IAP_DATA.H"
#include "Malloc.h"
#include "sdio_sdcard.h"
#include "OLED_GUI.h"
#include "STM32_CRC.h"
#include "stmflash.h"

const u8  *Updata_Path  ="0:/Updata/RTK01_YDZ.bin";     //�����ļ�·��

void Write_FLASH_APP_CRC_DATA(IAP_CRC_DATA_TYP * IAP_CRC_DAT )  //дFLASH����
{
	FLASH_Unlock();						//����
  STMFLASH_Write_NoCheck(APP_DATA_START_Adress,(u16 * )IAP_CRC_DAT,sizeof(IAP_CRC_DATA_TYP)/2); //д��FLASH
 	FLASH_Lock();//����
}

void ErasePage_FLASH_APP_CRC_DATA(void)    //����IAP CRC����
{
	FLASH_Unlock();						//����
  FLASH_ErasePage(APP_DATA_START_Adress);//�����������
  FLASH_Lock();//����
}

void ErasePage_FLASH_APP_FLASH(u32 APP_Size)    //�����û����� FLASH ���� ����
{
	u16  i =  APP_START_PAGE ;    //�õ��û���ʼ����
	u16  PAGE = 0 ;               //��������
	PAGE = APP_Size/SECTION_PAGE_SIZE ;   //�õ�����
	if((APP_Size%SECTION_PAGE_SIZE) >0)   //����г�����  ���ǻ�û�е���һ��������
	{
	  PAGE ++ ;   //��������һ
	}
	
	FLASH_Unlock();						//����
	for( ; i<( APP_START_PAGE+PAGE ) ; i ++)
	{
    FLASH_ErasePage(Section_Start_Adress(i));//�����������
	}
	FLASH_Lock();//����
}

void Read_FLASH_APP_CRC_DATA(IAP_CRC_DATA_TYP * IAP_CRC_DAT )   //��ȡ������FLASH �е�У��ֵ
{
  STMFLASH_Read(APP_DATA_START_Adress,(u16 * )IAP_CRC_DAT,sizeof(IAP_CRC_DATA_TYP)/2);
}



//�õ��û����� У��ֵ
u8 Read_USER_FLASH_DATA_CRC(IAP_CRC_DATA_TYP * IAP_CRC_DAT )    //����0 �ɹ�    1ʧ��
{
	u16  i = 0 ;                            //����
	u16  CRC_Number      = 0 ;              //��������У����    �������ļ� ��2048�ֽ�Ϊһ��    һ�ζ�У��
	u8   *iap_buffer     = 0 ;              //���ݻ��� 
	Read_FLASH_APP_CRC_DATA(IAP_CRC_DAT);   //�õ�������FLASH ��У������
	if(IAP_CRC_DAT->IAP_CRC_DATA_HEAD != IAP_CRC_HEAD )   //���ͷ����
	{
	  return 1 ;  //����
	}
  iap_buffer = (u8 * ) mymalloc( CRC_SECTION_SIZE ) ;  //�����ڴ�      ����һ��CRC�δ�С
//��ȡ�û�����У��ֵ-------------------------------------------------
	CRC_Number = IAP_CRC_DAT->File_SIZE/CRC_SECTION_SIZE ;              //�õ��ֶ���
  for( i = 0 ; i <CRC_Number ; i ++)                                  //�õ��ֶ�CRCֵ
	{
	 STMFLASH_Read(IAP_CRC_DAT->APP_STAR_ADDR+i*CRC_SECTION_SIZE ,(u16 *)iap_buffer,CRC_SECTION_SIZE/2) ; //��ȡ2048���ֽ�
	 IAP_CRC_DAT->DATA_CRC[i]	 = ST32_CRC_DATA_Str((u32 *)iap_buffer,CRC_SECTION_SIZE/4);             //����CRCֵ                                                        //��ַƫ��
	}
	CRC_Number = IAP_CRC_DAT->File_SIZE%CRC_SECTION_SIZE ;                                        //�õ�ʣ����
  STMFLASH_Read(IAP_CRC_DAT->APP_STAR_ADDR+i*CRC_SECTION_SIZE,(u16 *)iap_buffer,CRC_Number/2) ; //��ȡʣ���ֽ�		        
	IAP_CRC_DAT->DATA_CRC[i] = ST32_CRC_DATA_Str((u32 *)iap_buffer,CRC_Number/4);                 //����CRCֵ
	myfree(iap_buffer);                                                                           //�ͷ��ڴ�
	return 0 ;                                                                                    //���سɹ�
}

u8  Bijiao_CRC_DATA(IAP_CRC_DATA_TYP * IAP_CRC_DAT0,IAP_CRC_DATA_TYP * IAP_CRC_DAT1)  //�Ƚ�   ��ͬΪ 0   ��ͬΪ 1
{
	u16  i = 0 ; 
	u8 * data0 = (u8*)IAP_CRC_DAT0 ;
	u8 * data1 = (u8*)IAP_CRC_DAT1 ;
	for(i = 0 ; i < sizeof(IAP_CRC_DATA_TYP); i++ )
  {
	  if( (*data0) !=(*data1) )
		{
		 return 1 ;
		}
		data0++;
		data1++;
	}
  return 0 ;
}


void SYS_SCAN_Destruct_Exe(void)  //����豸�Ƿ���Ҫ�Ի�
{
	u16    Destruct_FLAT    =   SYS_Destruct_DR ;
  if( Destruct_FLAT == IAP_CRC_HEAD )  //������Իٱ��
	{
	  ErasePage_FLASH_APP_CRC_DATA() ;        //����IAP CRC����
    ErasePage_FLASH_APP_FLASH(30*1024)  ;   //����FLASH ���� ����    ����30K ����
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);	//ʹ��PWR��BKP����ʱ��   
    PWR_BackupAccessCmd(ENABLE);	                                            //ʹ�ܺ󱸼Ĵ������� 
		SYS_Destruct_DR = 0XFFFF;                                                 //����Իٱ��
	}
}

void SET_SYS_Destruct(void)    //�����豸�Ի�
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);	//ʹ��PWR��BKP����ʱ��   
  PWR_BackupAccessCmd(ENABLE);	                                            //ʹ�ܺ󱸼Ĵ������� 
  SYS_Destruct_DR = IAP_CRC_HEAD ;                                          //д���Իٱ��
	Sys_Soft_Reset();                                                         //�豸��λ
}


void Scan_SYS_Bootloader_Lock(void)  //����û������Ƿ�ΪBootloader��¼  ��������豸�Ի�
{
  IAP_CRC_DATA_TYP * FLASH_CRC_DATA ;   //FLASH �ڱ����CRC����
	IAP_CRC_DATA_TYP * USER_CRC_DATA  ;   //�û�����У������
	FLASH_CRC_DATA  =(IAP_CRC_DATA_TYP*)mymalloc(sizeof(IAP_CRC_DATA_TYP));		//�����ڴ�	
	USER_CRC_DATA  =(IAP_CRC_DATA_TYP*)mymalloc(sizeof(IAP_CRC_DATA_TYP));		//�����ڴ�	
  if( Read_USER_FLASH_DATA_CRC(USER_CRC_DATA) == 0 )   //��ȡ�û����� CRC
	{
	  Read_FLASH_APP_CRC_DATA(FLASH_CRC_DATA);  //��ȡFLASH�� �����У������
		if( Bijiao_CRC_DATA(USER_CRC_DATA,FLASH_CRC_DATA) > 0)   //���У�����
		{
		 	myfree(USER_CRC_DATA);    //�ͷ��ڴ�
	 	  myfree(FLASH_CRC_DATA);  
		  SET_SYS_Destruct();       //�����豸�Ի�		
		}
	}
	else     //��ȡʧ��
	{
	 	myfree(USER_CRC_DATA);    //�ͷ��ڴ�
	 	myfree(FLASH_CRC_DATA);  
		SET_SYS_Destruct();       //�����豸�Ի�	
	}
  myfree(USER_CRC_DATA);    //�ͷ��ڴ�
	myfree(FLASH_CRC_DATA);  
}


