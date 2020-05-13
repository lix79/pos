#include "USART1_DMA.h"
#include "ff.h"
#include "exfuns.h"
#include "stm32f10x_it.h"
#include "Led.h"
#include "SRAM_DATA.h"
#include "PLAY_DATA.H"
#include "GPS_DATE.h"
#include "Gps.h"
#include "SRAM_DATA.h"
#include "PLAY_DATA.H"





void USART1_RXD_DMA_CONFIG(void)
{
	DMA_InitTypeDef   DMA_InitStructure ;   //DMA���ýṹ��  ȫ�ֱ���
	NVIC_InitTypeDef  NVIC_InitStruct ;
  //������DMA����    
  //����DMAʱ��  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
  //DMA1ͨ��5����  
  DMA_DeInit(DMA1_Channel5);  
 //�����ַ  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);  
 //�ڴ��ַ  
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;  
 //dma���䷽����  
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
 //����DMA�ڴ���ʱ�������ĳ���  
  DMA_InitStructure.DMA_BufferSize = 0;  
 //����DMA�������ַ������
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
 //����DMA���ڴ����ģʽ  
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
 //���������ֳ�  
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
 //�ڴ������ֳ�  
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
 //����DMA�Ĵ���ģʽ  
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;    //��Ϊ�ػ�ģʽ
 //����DMA�����ȼ���  
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
 //����DMA��2��memory�еı����������  
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //�Ǵ�������������ģʽ
  DMA_Init(DMA1_Channel5,&DMA_InitStructure);  
	
	//����DMA��ʽ����  
  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);            //��������DMA����ģʽ
  DMA_ClearFlag(DMA1_FLAG_TC5);                           //���ͨ��5������ɱ�־
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC|DMA_IT_TE,ENABLE); //����������� �ж�   ��������ж�
	NVIC_InitStruct.NVIC_IRQChannel  = DMA1_Channel5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd =   ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //������ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority  =  0 ;
 	NVIC_Init(&NVIC_InitStruct);//DMA�ж�����
	
//�ر�ͨ��5  
  DMA_Cmd(DMA1_Channel5,DISABLE);  
}
void DMA_FIFO_CONFIG(u32 MemoryBaseAddr,u16 LEN)  //���ô洢����ַ ��DMA ����  ����DMA
{
	DMA_Cmd(DMA1_Channel5,DISABLE);      //�ر�DMA

  DMA1_Channel5->CMAR   =  MemoryBaseAddr ;   //����װ�ش洢����ַ
  DMA1_Channel5->CNDTR  =  LEN ;  	          //װ�ػ��泤�� 
	
 	DMA_Cmd(DMA1_Channel5,ENABLE);      //����DMA
}


void USART1_RXD_DMA_Init(void)
{
	RX_DMA_STA = 0 ;  //����
	SET_RX_DMA_BUF0 ;  //���û��� 0
	//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);                        //�رս����ж�
	FIFO_CLR_BUF(RX_BUF_FIFO);                                             //���FIFO
	
  USART1_RXD_DMA_CONFIG() ;                                              //��ʼ�� ����1 DMA ��������   ���δ���  ��������ж�
  DMA_FIFO_CONFIG((u32)RX_DMA_BUFF0 ,DMA_BUFF_SIZE) ;                    //���ô洢����ַ ��DMA ����  ����DMA
}


void USART1_IRQHandler(void)   
{
	u8 com_data =0;
	//�����ж� (���ռĴ����ǿ�)----------------------------------------
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
	 com_data= USART1->DR;
	
//UTCʱ��У׼����------------------------------------------
  if(RTC_CONFIG_FLAT)	 //��ҪУ׼		     
	{
    GPS_BUF_TIME_Adjust_Task(GPS_BUF,com_data);   //����GPSʱ��У׼����		
		if(GPS_BUF_TIME_Adjust(GPS_BUF)==GPS_FREE)  //ʱ�����	 
		{
			RTC_Set(GPS_TIME->syear,GPS_TIME->smon,GPS_TIME->sday,GPS_TIME->hour,GPS_TIME->min,GPS_TIME->sec);  //����RTCʱ��
	    RTC_CONFIG_Nuber_ADD;        //У׼������һ		
		}
	}
	else
	{
	//д��FIFO------------------------------------------------
		FIFO_WRITE_Byte(RX_BUF_FIFO,com_data);  //д��FIFO
	  //Uart1_send_str(RX_DMA_BUFF0 ,com_data);
	}
 }
}

typedef enum 
{
	initial_state,
	got_hash,
	got_asterisk
}Prase_status;

 char gps_shuzu[EVENT_BUF_LEN];
 char *gps_timea_buff=gps_shuzu;
unsigned int data_gps_cnt=0;
unsigned char gps_write_data_flag=0;
unsigned char zhengti_cnt=0;
unsigned char zhengti_flag=0;
Prase_status receive_flag = initial_state;

void USART2_IRQHandler(void)   //Event�¼�
{
	//unsigned char com_data =0;
	//�����ж� (���ռĴ����ǿ�)----------------------------------------
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
	{
			//com_data= USART2->DR;

            return ;
/*		
//		switch(receive_flag)
//		{
//			case initial_state:
//				
//		
//	
//		
//		}
	(*(gps_timea_buff+data_gps_cnt++))= com_data;
		//gps_shuzu[data_gps_cnt++]=com_data;
	
		if(zhengti_flag==1)
		{
		  zhengti_cnt++;
			 if(zhengti_cnt==10)
				 {
					 //���յ�\r\n�����Ļ��������㣬��ֹ�ϴν��յ����ݱ������¼*
					 for(;data_gps_cnt < EVENT_BUF_LEN;data_gps_cnt++)
					 {
						 (*(gps_timea_buff+data_gps_cnt))= '\0';
					 }
			    
					 gps_write_data_flag=1;
					 zhengti_cnt=0;
					 data_gps_cnt=0;
					 zhengti_flag=0;
						// *(gps_timea_buff+data_gps_cnt)= '\0';
				 }
		}	
		else if( (*(gps_timea_buff+data_gps_cnt-1))=='*')
		//else if(gps_shuzu[data_gps_cnt-1]=='*')
		{
		  if(data_gps_cnt>15)
			{  
				zhengti_flag=1;	
			}  
			else
			data_gps_cnt=0;
		  
		}
	
//UTCʱ��У׼����------------------------------------------
 */
 }
}

//char  start_shuzu[3]=0;
   char pos_shuzu[100]={0};
//char pos_sz[100]=0;
// char *pos_timea_buff=pos_sz;

unsigned char data_pos_cnt=0;
char pos_jilu_flag=0;

const unsigned char START_BYTE=0x55;
const unsigned char END_BYTE=0xF0;
#define RX_START  0
#define RX_DATA  1	
#define RX_MAX   100
unsigned char start_cnt=0;
unsigned char end_cnt=0;
unsigned char RX_STATUS=RX_START;

//char longitude[8];
double plongitude=0;
//char latitude[8];
double platitude=0;
char high[4];
float phigh=0;
char pitch[4];
float ppitch=0;
char row[4];
float prow=0;
char dirct[4];
float pdirct=0;
void USART3_IRQHandler(void)   
{
	unsigned char com_data =0;
	unsigned char i=0;
	//�����ж� (���ռĴ����ǿ�)----------------------------------------
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
	{    
        com_data= USART3->DR;
	     
        switch (RX_STATUS)
        {
        case RX_START:
            if (com_data == START_BYTE)//0x55 ��д�ַ�U
            {
                start_cnt++;
                if (start_cnt == 3)
                {
                    RX_STATUS = RX_DATA;
                    start_cnt=0;
                    //RX_head_check_cnt_0 = 0;
                    //RX_rear_check_cnt_0 = 0;
                }
            }
            else
            {
              start_cnt = 0;
            }
            break;

        case RX_DATA:
            pos_shuzu[data_pos_cnt] = com_data;
            data_pos_cnt ++;
            if(data_pos_cnt==8)
            {
                for(i=0;i<8;i++)
                {
                    ((char *)&platitude)[i] = pos_shuzu[i];
                }
            }
            else if(data_pos_cnt==16)
            {
                for(i=0;i<8;i++)
                {
                    ((char *)&plongitude)[i] = pos_shuzu[i+8];
                }
            }
            else if(data_pos_cnt==20)
            {
                for(i=0;i<4;i++)
                {
                    high[i]=pos_shuzu[i+16];
                }
                phigh=*(float*)&high;
            }
            else if(data_pos_cnt==24)
            {
                for(i=0;i<4;i++)
                {
                    pitch[i]=pos_shuzu[i+20];
                }
                ppitch=*(float*)&pitch;
            }
            else if(data_pos_cnt==28)
            {
                for(i=0;i<4;i++)
                {
                    row[i]=pos_shuzu[i+24];
                }
                prow=*(float*)&row;
            }
            else if(data_pos_cnt==32)
            {
                for(i=0;i<4;i++)
                {
                    dirct[i]=pos_shuzu[i+28];
                }
                pdirct=*(float*)&dirct;
            }
            
            if(data_pos_cnt>RX_MAX)
            {
                data_pos_cnt=0;
                RX_STATUS=RX_START;
            }

            if (com_data == END_BYTE)//0xF0
            {
                end_cnt++;
                if (end_cnt == 3)
                {
                    end_cnt=0;

                    RX_STATUS=RX_START;
                    data_pos_cnt=0;
                    pos_jilu_flag=1;	
                }
            }
            else
            {
                end_cnt = 0;
            }

            break;
        }
   }
}

void DMA1_Channel5_IRQHandler(void)    //����DMA��������ж�
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC5)!=RESET)	//�ж�ͨ��5�������
	{
		DMA_ClearFlag(DMA1_FLAG_TC5|DMA1_FLAG_HT5);                               //���ͨ��5������ɱ�־ ������������
		if( RX_DMA_FLAT == 0 )   //���֮ǰ������ٶȴ�������
		{
     SET_RX_DMA_UP ;          //���û��������ɱ��
		 if( RX_DMA_Nuber == 0 )  //���֮ǰ������ǵ�0������
		 { 
			SET_RX_DMA_BUF1 ;   //ָʾ��һ�����濪ʼ����
		  DMA_FIFO_CONFIG((u32)RX_DMA_BUFF1 ,DMA_BUFF_SIZE) ;   //���ô洢����ַ ��DMA ����  ����DMA
			// Uart1_send_str(RX_DMA_BUFF1 ,DMA_BUFF_SIZE);
		 }
		 else
		 {
			SET_RX_DMA_BUF0 ;   //ָʾ��0�����濪ʼ����
		  DMA_FIFO_CONFIG((u32)RX_DMA_BUFF0 ,DMA_BUFF_SIZE) ;   //���ô洢����ַ ��DMA ����  ����DMA		 
		 //  Uart1_send_str(RX_DMA_BUFF0 ,DMA_BUFF_SIZE);
		 }
	  }
	  else   //DMA�Ѿ�������� �����ϴλ������ݻ�û������
	  {
	    Config_Counter ++;        //����SD��д�ٶȹ������   ���ݶ�ʧ
		 if( RX_DMA_Nuber == 0 )    //���֮ǰ������ǵ�0������
		 { 
			SET_RX_DMA_BUF1 ;         //ָʾ��һ�����濪ʼ����
		  DMA_FIFO_CONFIG((u32)RX_DMA_BUFF1 ,DMA_BUFF_SIZE) ;   //���ô洢����ַ ��DMA ����  ����DMA
		 }
		 else
		 {
			SET_RX_DMA_BUF0 ;         //ָʾ��0�����濪ʼ����
		  DMA_FIFO_CONFIG((u32)RX_DMA_BUFF0 ,DMA_BUFF_SIZE) ;   //���ô洢����ַ ��DMA ����  ����DMA		 
		 }
	  }
	}

	if(DMA_GetFlagStatus(DMA1_FLAG_TE5)!=RESET)   //��������ж�
	{
	  DMA_ClearFlag(DMA1_FLAG_TE5);           
	  Config_Counter +=20000 ;
	}
}


