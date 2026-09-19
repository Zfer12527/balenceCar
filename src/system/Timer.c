#include "stm32f10x.h"                  // Device header
void Timer_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // Enable clock for TIM2
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 50000-1; // Set the period (ARR value)
    TIM_TimeBaseStructure.TIM_Prescaler = 7199; // Set the prescaler (PSC value)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // No clock division
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // Count up
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//重复计数器
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); // Initialize TIM1 with the configuration

    // TIM_ClearFlag(TIM1,TIM_FLAG_Update);//不清除上电后直接执行一次
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; // Set the IRQ channel for TIM2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // Set the preemption priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // Set the subpriority
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable the IRQ channel
    NVIC_Init(&NVIC_InitStructure); // Initialize the NVIC with the configuration

    TIM_Cmd(TIM1, ENABLE); // Enable TIM1
}
// void TIM1_UP_IRQHandler(void)
// {
//     if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // Check if the update interrupt flag is set
//     {
//         TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // Clear the update interrupt flag
//         // Your code here to handle the timer interrupt
//     }
// }