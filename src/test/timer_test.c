// volatile uint16_t i=1;		//定义用于计数的变量
// void TIM1_UP_IRQHandler(void)
// {
//     if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // Check if the update interrupt flag is set
//     {
// 		i++;
// 		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // Clear the update interrupt flag
// 		__NOP(); // No operation, can be used for debugging
//     }
// }
// void main(void)
// {
// 	OLED_Init();		//OLED初始化
// 	Timer_Init();
// 	while (1)
// 	{
// 		OLED_ShowNum(1, 1, i, 5);		//OLED显示数字
// 	}
// }