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
const u8 *GPS_CMM_5HZ[GPS_COMM_CONFIG_Nuber] ={            //5HZ 配置指令集  
  "$PASHS,NME,ALL,A,OFF\r\n"  ,
	 "$PASHS,SBA,OFF\r\n"  ,
	 "$PASHS,ATL,ON\r\n"  ,
	"$PASHS,ATM,RNX,A,ON,0.2,&SCN,0\r\n"     ,          //输出5Hz的原始观测数据               
 "$PASHS,ATM,NAV,A,ON\r\n"       ,          //输出gps星历文件
// "$PASHS,ATM,PVT,A,ON,1\r\n"   ,          //输出输出GLONASS星历文件
	 "$PASHS,ATM,ATR,A,ON\r\n"       ,          //输出gps星历文件
 "$PASHS,ATM,EVT,A,ON,1\r\n"       ,          //输出北斗星历文件
  "$PASHS,NME,TTT,A,ON\r\n"  ,								//EVENT 事件   A口默认出二进制 
	"$PASHS,NME,TTT,D,ON\r\n"  ,								//EVENT 事件   D口默认出ASCII码           
	"$PASHS,NME,GGA,A,ON,60\r\n"    ,						 //后面两条指令ZDA和GGA是给张辰辰后差分软件解算使用的
 "$PASHS,PHE,R\r\n"  
} ;


  

const u8 *GPS_REST_CMM           =  "$PASHS,RST\r\n" ;                //GPS恢复成出厂设置命令
const u8 *GPS_TIME_CMM           =  "$PASHS,NME,ZDA,A,ON,1\r\n"   ;  //GPS配置输出GPS时间数据
const u8 *GPS_UNLOGALL_CMM       =  "$PASHS,ATM,ALL,A,OFF\r\n"   ;  //GPS配置关闭所有串口打印输出
const u8 *GPS_SAVE_CMM           =  "saveconfig\r\n"           ;  //保存GPS配置

const u8 *GPS_MARK_INPUT         = "markcontrol mark1 enable negative 50 2\r\n";//mark控制输入
const u8 *GPS_MARK_ON_TIME       = "log com1 marktime onnew\r\n";//mark 输出位时间
const u8 *GPS_MARK_ON_POS        = "log com1 markpos  onnew\r\n" ;//mark 输出位置
const u8 *GPS_MARK_ON_POSB       = "log com1 markposb  onnew\r\n";//mark pos 输出二进制文件
#endif

#ifdef UB482
const u8 *GPS_CMM_5HZ[GPS_COMM_CONFIG_Nuber] ={            //5HZ 配置指令集  
  	"UNLOG COM1\r\n"  ,
	 	//"$PASHS,SBA,OFF\r\n"  ,//UB482要怎么配置？
	 	//"$PASHS,ATL,ON\r\n"  ,//UB482要怎么配置？
		//"LOG BINEX7F05 ONTIME 0.2\r\n"     ,          //输出5Hz的原始观测数据               
 		//"LOG BINEX0101 ONCHANGED\r\n"       ,          //输出GPS星历文件
		//"LOG BINEX0102 ONCHANGED\r\n"   ,          //输出输出GLONASS星历文件
	 	//"LOG BINEX0105 ONCHANGED\r\n"       ,          //输出BDS星历文件
 		"log rangeb ontime 0.2\r\n" 
	"log gpsephemb ontime 60\r\n" 
	"log bd2ephemb ontime 60\r\n" 
	"log gloephemerisb ontime 60\r\n" 
	//"$PASHS,ATM,EVT,A,ON,1\r\n"       ,          //输出北斗星历文件
  	"LOG EVENTMARKB ONCHANGED\r\n"  ,								//EVENT 事件   A口默认出二进制 
		//"$PASHS,NME,TTT,D,ON\r\n"  ,			//串口3EVENT事件需要单独配置 				//EVENT 事件   D口默认出ASCII码           
		"GPGGA COM1 0.2\r\n"    ,	// 在串口1以0.5s的周期输出多系统联合定位结果					 //后面两条指令ZDA和GGA是给张辰辰后差分软件解算使用的
		"GPZDA COM1 0.2\r\n"  //在串口1以0.5s的周期输出GPZDA 日期和时间
		//"CONFIG EVENT ENABLE POSITIVE 10\r\n"
	//"UNLOG COM1\r\n"  ,
		"CONFIG EVENT ENABLE POSITIVE 10\r\n"
		"LOG EVENTMARKA ONCHANGED\r\n"
	
} ;


  

const u8 *GPS_REST_CMM           =  "FRESET\r\n" ;                //GPS恢复成出厂设置命令
const u8 *GPS_TIME_CMM           =  "GPZDA COM1 1\r\n"   ;  //GPS配置输出GPS时间数据
const u8 *GPS_UNLOGALL_CMM       =  "UNLOG COM1\r\n"   ;  //GPS配置关闭所有串口打印输出
const u8 *GPS_SAVE_CMM           =  "saveconfig\r\n"   ;  //保存GPS配置
const u8 *GPS_EVENT_CONFIG           =  "CONFIG EVENT ENABLE NEGATIVE 10\r\n";
const u8 *GPS_EVENT_ASCII_RECORD           =  "LOG EVENTMARKA ONCHANGED\r\n" ;  //
  //


const u8 *GPS_MARK_INPUT         = "markcontrol mark1 enable negative 50 2\r\n";//mark控制输入
const u8 *GPS_MARK_ON_TIME       = "log com1 marktime onnew\r\n";//mark 输出位时间
const u8 *GPS_MARK_ON_POS        = "log com1 markpos  onnew\r\n" ;//mark 输出位置
const u8 *GPS_MARK_ON_POSB       = "log com1 markposb  onnew\r\n";//mark pos 输出二进制文件

#endif
	
void GPS_PV_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(GPS_PV_RCC,ENABLE);//使能时钟

	GPIO_InitStructure.GPIO_Pin  = GPS_PV_PIN;//PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //设置浮空输入
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
 	GPIO_Init(GPS_PV_, &GPIO_InitStructure);	
}

u8 GPS_CONFIG_RESET(void)  //GPS复位   返回 0成功   1失败
{
	u8 i = 0 ;
	u8 TESTBUF[9] ={0};     //串口命令反馈缓存
  Uart1_send_str((u8*)GPS_REST_CMM,strlen((const char *)GPS_REST_CMM));   //发送复位
	FIFO_CLR_BUF(RX_BUF_FIFO);        //清空FIFO
	delay_ms(500);     //等待命令回传完成
	
	if(FIFO_BUF_Valid_NUM(RX_BUF_FIFO)<64)   //复位指令回传字节数  大约估计
	{
	  return 1 ;       //失败
	}
	FIFO_CLR_BUF(RX_BUF_FIFO);        //清空FIFO
	
	while(i<40)    //等待复位完成   接收 [COM1] 字符串  等待 20 S
	{
	 if(FIFO_BUF_Valid_NUM(RX_BUF_FIFO)>=8)  //字符串最小长度
	 {
	   FIFO_READ_BUF(RX_BUF_FIFO,TESTBUF,8);  //读取缓存
	   TESTBUF[8] = '\0';   //添加结束符
		 if(strcmp((const char*)TESTBUF,(const char*)"[COM1]\r\n")==0)   //比较
		 {
       return 0 ;         //复位成功！
		 }
		 else
		 {
		   return 1 ;         //如果不相等 则失败
		 }	
	 }
	 delay_ms(500);
	 i++;    //此处可加显示 及动画效果
	}
	return 1 ;         //超时 返回失败
}

#define GPS_REST_MAX   60   //复位失败次数   大约 12 S

u8 GPS_CONFIG_5HZ(void)   //配置成 5HZ   返回0成功   1失败
{
	u8 i = 0 ;
	while(GPS_UNLOGALL_CONFIG()&&(i<GPS_REST_MAX)) 
	{  
	   i ++;
		delay_ms(200) ;
	}
	if(i>=GPS_REST_MAX)
	{
	  return 1;  //失败
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
	
	return 0 ;    //成功
}

u8 GPS_UNLOGALL_CONFIG(void)  //GPS关闭所有输出   0成功    1失败
{
	FIFO_CLR_BUF(RX_BUF_FIFO);        //清空FIFO
  Uart1_send_str((u8*)GPS_UNLOGALL_CMM,strlen((const char *)GPS_UNLOGALL_CMM)); //发送关闭所有输出
	delay_ms(200) ;  //等待接收完成
	
	if(FIFO_BUF_Valid_NUM(RX_BUF_FIFO)>=10)  //实际响应为 10字节
	{
	  if(RX_BUF_FIFO->FIFO_BUFF[FIFO_BUF_Valid_NUM(RX_BUF_FIFO)-6]==0X4B)  //倒数第3和第4个  为 OK 字符
	  {
		  return 0;  //返回成功
		}
	  return 1;
	}

	return 1 ;
}

u8 GPS_CONFIG_TIMEA(void)  //串口输出时间数据
{
  u8 i = 0;
	/*关闭板卡log输出并确认板卡应答是否正确*/
	while(GPS_UNLOGALL_CONFIG()&&(i<GPS_REST_MAX))
	{
	   i ++;
		delay_ms(200);
	}
	
	if(i>=GPS_REST_MAX)
	{
	  return 1;  //失败
	}
//	GPS_SAVE_CONFIG();  //保存配置
	Uart1_send_str((u8*)GPS_TIME_CMM,strlen((const char *)GPS_TIME_CMM));//配置板卡输出ZDA消息
	return 0;  //成功
}

u8 GPS_SAVE_CONFIG(void)  //保存配置
{
	FIFO_CLR_BUF(RX_BUF_FIFO);         //清空FIFO
  Uart1_send_str((u8*)GPS_SAVE_CMM,strlen((const char *)GPS_SAVE_CMM)); //发送保存配置
	delay_ms(20) ;  //等待接收完成
	
	if(FIFO_BUF_Valid_NUM(RX_BUF_FIFO)>=10)  
	{
	  if((RX_BUF_FIFO->FIFO_BUFF[FIFO_BUF_Valid_NUM(RX_BUF_FIFO)-4]==0X4F)&&(RX_BUF_FIFO->FIFO_BUFF[FIFO_BUF_Valid_NUM(RX_BUF_FIFO)-3]==0X4B))  //倒数第3和第4个  为 OK 字符
	  {
		  return 0;  //返回成功
		}
	  return 1;
	}
	return 1 ;
}
