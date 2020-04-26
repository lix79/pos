#include "Tim4.h"


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   	void Tim4_Init(u16 period_num)
*��������:	   	��ʱ��4��ʼ��������Ƶ��10KHz����ռ���ȼ�2�������ȼ�0���ж��д�������ģ����ж�
*���������  		period_num��Ԥװ��ֵ
*���������  		��
*******************************************************************************/
void Tim4_Init(u16 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//ʱ������
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	//��λ��ʱ��4����
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period=period_num;//װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//��Ƶϵ��
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//count up
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	//�ж�����
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//clear the TIM4 overflow interrupt flag
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	//TIM4 overflow interrupt enable
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	//enable TIM4
	TIM_Cmd(TIM4,ENABLE);
}

