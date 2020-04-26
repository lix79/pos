#include "GPS_STA.h"
#include <string.h>
#include "delay.h"
#include "USART1_DMA.h"
#include "MY_FIFO.H"
#include "SRAM_DATA.h"
#include "OLED_GUI.h"
#include "Oled.h"
#include "Led.h"
#include "M_DATA.H"


#ifdef MB2
const u8 *GPS_CMM_5HZ[GPS_COMM_CONFIG_Nuber] ={            //5HZ ����ָ�  
  "$PASHS,NME,ALL,A,OFF\r\n"  ,
	 "$PASHS,SBA,OFF\r\n"  ,
	 "$PASHS,ATL,ON\r\n"  ,
	"$PASHS,ATM,RNX,A,ON,0.2,&SCN,0\r\n"     ,          //���5Hz��ԭʼ�۲�����               
 "$PASHS,ATM,NAV,A,ON\r\n"       ,          //���gps�����ļ�
// "$PASHS,ATM,PVT,A,ON,1\r\n"   ,          //������GLONASS�����ļ�
	 "$PASHS,ATM,ATR,A,ON\r\n"       ,          //���gps�����ļ�
 "$PASHS,ATM,EVT,A,ON,1\r\n"       ,          //������������ļ�
  "$PASHS,NME,TTT,A,ON\r\n"  ,								//EVENT �¼�   A��Ĭ�ϳ������� 
	"$PASHS,NME,TTT,D,ON\r\n"  ,								//EVENT �¼�   D��Ĭ�ϳ�ASCII��           
	"$PASHS,NME,GGA,A,ON,60\r\n"    ,						 //��������ָ��ZDA��GGA�Ǹ��ų��������������ʹ�õ�
 "$PASHS,PHE,R\r\n"  
} ;


  

const u8 *GPS_REST_CMM           =  "$PASHS,RST\r\n" ;                //GPS�ָ��ɳ�����������
const u8 *GPS_TIME_CMM           =  "$PASHS,NME,ZDA,A,ON,1\r\n"   ;  //GPS�������GPSʱ������
const u8 *GPS_UNLOGALL_CMM       =  "$PASHS,ATM,ALL,A,OFF\r\n"   ;  //GPS���ùر����д��ڴ�ӡ���
const u8 *GPS_SAVE_CMM           =  "saveconfig\r\n"           ;  //����GPS����

const u8 *GPS_MARK_INPUT         = "markcontrol mark1 enable negative 50 2\r\n";//mark��������
const u8 *GPS_MARK_ON_TIME       = "log com1 marktime onnew\r\n";//mark ���λʱ��
const u8 *GPS_MARK_ON_POS        = "log com1 markpos  onnew\r\n" ;//mark ���λ��
const u8 *GPS_MARK_ON_POSB       = "log com1 markposb  onnew\r\n";//mark pos ����������ļ�
#endif

#ifdef UB482
const u8 *GPS_CMM_5HZ[GPS_COMM_CONFIG_Nuber] ={            //5HZ ����ָ�  
  	"UNLOG COM1\r\n"  ,
	 	//"$PASHS,SBA,OFF\r\n"  ,//UB482Ҫ��ô���ã�
	 	//"$PASHS,ATL,ON\r\n"  ,//UB482Ҫ��ô���ã�
		//"LOG BINEX7F05 ONTIME 0.2\r\n"     ,          //���5Hz��ԭʼ�۲�����               
 		//"LOG BINEX0101 ONCHANGED\r\n"       ,          //���GPS�����ļ�
		//"LOG BINEX0102 ONCHANGED\r\n"   ,          //������GLONASS�����ļ�
	 	//"LOG BINEX0105 ONCHANGED\r\n"       ,          //���BDS�����ļ�
 		"log rangeb ontime 0.2\r\n" 
	"log gpsephemb ontime 60\r\n" 
	"log bd2ephemb ontime 60\r\n" 
	"log gloephemerisb ontime 60\r\n" 
	//"$PASHS,ATM,EVT,A,ON,1\r\n"       ,          //������������ļ�
  	"LOG EVENTMARKB ONCHANGED\r\n"  ,								//EVENT �¼�   A��Ĭ�ϳ������� 
		//"$PASHS,NME,TTT,D,ON\r\n"  ,			//����3EVENT�¼���Ҫ�������� 				//EVENT �¼�   D��Ĭ�ϳ�ASCII��           
		"GPGGA COM1 0.2\r\n"    ,	// �ڴ���1��0.5s�����������ϵͳ���϶�λ���					 //��������ָ��ZDA��GGA�Ǹ��ų��������������ʹ�õ�
		"GPZDA COM1 0.2\r\n"  //�ڴ���1��0.5s���������GPZDA ���ں�ʱ��
		//"CONFIG EVENT ENABLE POSITIVE 10\r\n"
	//"UNLOG COM1\r\n"  ,
		"CONFIG EVENT ENABLE POSITIVE 10\r\n"
		"LOG EVENTMARKA ONCHANGED\r\n"
	
} ;


  

const u8 *GPS_REST_CMM           =  "FRESET\r\n" ;                //GPS�ָ��ɳ�����������
const u8 *GPS_TIME_CMM           =  "GPZDA COM1 1\r\n"   ;  //GPS�������GPSʱ������
const u8 *GPS_UNLOGALL_CMM       =  "UNLOG COM1\r\n"   ;  //GPS���ùر����д��ڴ�ӡ���
const u8 *GPS_SAVE_CMM           =  "saveconfig\r\n"   ;  //����GPS����
const u8 *GPS_EVENT_CONFIG           =  "CONFIG EVENT ENABLE NEGATIVE 10\r\n";
const u8 *GPS_EVENT_ASCII_RECORD           =  "LOG EVENTMARKA ONCHANGED\r\n" ;  //
  //


const u8 *GPS_MARK_INPUT         = "markcontrol mark1 enable negative 50 2\r\n";//mark��������
const u8 *GPS_MARK_ON_TIME       = "log com1 marktime onnew\r\n";//mark ���λʱ��
const u8 *GPS_MARK_ON_POS        = "log com1 markpos  onnew\r\n" ;//mark ���λ��
const u8 *GPS_MARK_ON_POSB       = "log com1 markposb  onnew\r\n";//mark pos ����������ļ�

#endif
	
void GPS_PV_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(GPS_PV_RCC,ENABLE);//ʹ��ʱ��

	GPIO_InitStructure.GPIO_Pin  = GPS_PV_PIN;//PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //���ø�������
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
 	GPIO_Init(GPS_PV_, &GPIO_InitStructure);	
}

u8 GPS_CONFIG_RESET(void)  //GPS��λ   ���� 0�ɹ�   1ʧ��
{
	u8 i = 0 ;
	u8 TESTBUF[9] ={0};     //�������������
  Uart1_send_str((u8*)GPS_REST_CMM,strlen((const char *)GPS_REST_CMM));   //���͸�λ
	FIFO_CLR_BUF(RX_BUF_FIFO);        //���FIFO
	delay_ms(500);     //�ȴ�����ش����
	
	if(FIFO_BUF_Valid_NUM(RX_BUF_FIFO)<64)   //��λָ��ش��ֽ���  ��Լ����
	{
	  return 1 ;       //ʧ��
	}
	FIFO_CLR_BUF(RX_BUF_FIFO);        //���FIFO
	
	while(i<40)    //�ȴ���λ���   ���� [COM1] �ַ���  �ȴ� 20 S
	{
	 if(FIFO_BUF_Valid_NUM(RX_BUF_FIFO)>=8)  //�ַ�����С����
	 {
	   FIFO_READ_BUF(RX_BUF_FIFO,TESTBUF,8);  //��ȡ����
	   TESTBUF[8] = '\0';   //��ӽ�����
		 if(strcmp((const char*)TESTBUF,(const char*)"[COM1]\r\n")==0)   //�Ƚ�
		 {
       return 0 ;         //��λ�ɹ���
		 }
		 else
		 {
		   return 1 ;         //�������� ��ʧ��
		 }	
	 }
	 delay_ms(500);
	 i++;    //�˴��ɼ���ʾ ������Ч��
	}
	return 1 ;         //��ʱ ����ʧ��
}

#define GPS_REST_MAX   60   //��λʧ�ܴ���   ��Լ 12 S

u8 GPS_CONFIG_5HZ(void)   //���ó� 5HZ   ����0�ɹ�   1ʧ��
{
	u8 i = 0 ;
	while(GPS_UNLOGALL_CONFIG()&&(i<GPS_REST_MAX)) 
	{  
	   i ++;
		delay_ms(200) ;
	}
	if(i>=GPS_REST_MAX)
	{
	  return 1;  //ʧ��
	}
	

	Uart2_send_str((u8*)GPS_EVENT_CONFIG,strlen((const char *)GPS_EVENT_CONFIG));
	
	Uart2_send_str((u8*)GPS_EVENT_ASCII_RECORD,strlen((const char *)GPS_EVENT_ASCII_RECORD));
	
  for(i=0;i<GPS_COMM_CONFIG_Nuber;i++)
	{
	 Uart1_send_str((u8*)GPS_CMM_5HZ[i],strlen((const char *)GPS_CMM_5HZ[i]));
	}
	
//	Uart1_send_str((u8*)GPS_MARK_INPUT,strlen((const char *)GPS_MARK_INPUT));
	
//	Uart1_send_str((u8*)GPS_MARK_ON_TIME,strlen((const char *)GPS_MARK_ON_TIME));
	
//	Uart1_send_str((u8*)GPS_MARK_ON_POSB,strlen((const char *)GPS_MARK_ON_POSB));
//	
//	Uart1_send_str((u8*)GPS_MARK_ON_POS,strlen((const char *)GPS_MARK_ON_POS));
	
	//Uart2_send_str((u8*)GPS_EVENT_CONFIG,strlen((const char *)GPS_EVENT_CONFIG));
	
	//Uart2_send_str((u8*)GPS_EVENT_ASCII_RECORD,strlen((const char *)GPS_EVENT_ASCII_RECORD));
	
	return 0 ;    //�ɹ�
}

u8 GPS_UNLOGALL_CONFIG(void)  //GPS�ر��������   0�ɹ�    1ʧ��
{
	FIFO_CLR_BUF(RX_BUF_FIFO);        //���FIFO
  Uart1_send_str((u8*)GPS_UNLOGALL_CMM,strlen((const char *)GPS_UNLOGALL_CMM)); //���͹ر��������
	delay_ms(200) ;  //�ȴ��������
	
	if(FIFO_BUF_Valid_NUM(RX_BUF_FIFO)>=10)  //ʵ����ӦΪ 10�ֽ�
	{
	  if(RX_BUF_FIFO->FIFO_BUFF[FIFO_BUF_Valid_NUM(RX_BUF_FIFO)-6]==0X4B)  //������3�͵�4��  Ϊ OK �ַ�
	  {
		  return 0;  //���سɹ�
		}
	  return 1;
	}

	return 1 ;
}

u8 GPS_CONFIG_TIMEA(void)  //�������ʱ������
{
  u8 i = 0;
	/*�رհ忨log�����ȷ�ϰ忨Ӧ���Ƿ���ȷ*/
	while(GPS_UNLOGALL_CONFIG()&&(i<GPS_REST_MAX))
	{
	   i ++;
		delay_ms(200);
	}
	
	if(i>=GPS_REST_MAX)
	{
	  return 1;  //ʧ��
	}
//	GPS_SAVE_CONFIG();  //��������
	Uart1_send_str((u8*)GPS_TIME_CMM,strlen((const char *)GPS_TIME_CMM));//���ð忨���ZDA��Ϣ
	return 0;  //�ɹ�
}

u8 GPS_SAVE_CONFIG(void)  //��������
{
	FIFO_CLR_BUF(RX_BUF_FIFO);         //���FIFO
  Uart1_send_str((u8*)GPS_SAVE_CMM,strlen((const char *)GPS_SAVE_CMM)); //���ͱ�������
	delay_ms(20) ;  //�ȴ��������
	
	if(FIFO_BUF_Valid_NUM(RX_BUF_FIFO)>=10)  
	{
	  if((RX_BUF_FIFO->FIFO_BUFF[FIFO_BUF_Valid_NUM(RX_BUF_FIFO)-4]==0X4F)&&(RX_BUF_FIFO->FIFO_BUFF[FIFO_BUF_Valid_NUM(RX_BUF_FIFO)-3]==0X4B))  //������3�͵�4��  Ϊ OK �ַ�
	  {
		  return 0;  //���سɹ�
		}
	  return 1;
	}
	return 1 ;
}
