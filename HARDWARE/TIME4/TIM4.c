#include "Tim4.h"


/**************************实现函数********************************************
*函数原型:	   	void Tim4_Init(u16 period_num)
*功　　能:	   	定时器4初始化，记数频率10KHz，抢占优先级2，子优先级0，中断中处理无线模块的中断
*输入参数：  		period_num，预装载值
*输出参数：  		无
*******************************************************************************/
void Tim4_Init(u16 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//时钟配置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	//复位定时器4配置
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period=period_num;//装载值
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//count up
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	//中断配置
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

