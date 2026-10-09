#include "stm32f10x.h" // Device header
#define JDBL 9         // 精度倍率
/**
 * 函    数：PWM初始化
 * 参    数：无
 * 返 回 值：无
 */
void PWM_Init(void)
{
    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // 开启TIM1的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 开启GPIOA的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 开启GPIOB的时钟

    /*GPIO重映射*/
    //	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			//开启AFIO的时钟，重映射必须先开启AFIO的时钟
    //	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM1, ENABLE);			//将TIM1的引脚部分重映射，具体的映射方案需查看参考手册
    //	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);		//将JTAG引脚失能，作为普通GPIO引脚使用

    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // 受外设控制的引脚，均需要配置为复用模式

    /*配置时钟源*/
    TIM_InternalClockConfig(TIM1); // 选择TIM1为内部时钟，若不调用此函数，TIM默认也为内部时钟

    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;                    // 定义结构体变量
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;       // 时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up; // 计数器模式，选择向上计数
    TIM_TimeBaseInitStructure.TIM_Period            = 100 * JDBL - 1;     // 计数周期，即ARR的值
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 36 / JDBL - 1;      // 预分频器，即PSC的值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;                  // 重复计数器，高级定时器才会用到
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);                   // 将结构体变量交给TIM_TimeBaseInit，配置TIM1的时基单元

    /*输出比较初始化*/
    TIM_OCInitTypeDef TIM_OCInitStructure;                        // 定义结构体变量
    TIM_OCStructInit(&TIM_OCInitStructure);                       // 结构体初始化，若结构体没有完整赋值
                                                                  // 则最好执行此函数，给结构体所有成员都赋一个默认值
                                                                  // 避免结构体初值不确定的问题
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;        // 输出比较模式，选择PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;    // 输出极性，选择为高，若选择极性为低，则输出高低电平取反
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse       = 0;                      // 初始的CCR值
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);                      // 将结构体变量交给TIM_OC1Init，配置TIM1的输出比较通道1
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    /*TIM使能*/
    TIM_Cmd(TIM1, ENABLE); // 使能TIM1，定时器开始运行
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
 * 函    数：PWM设置CCR
 * 参    数：Compare 要写入的CCR的值，范围：0~100
 * 返 回 值：无
 * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
 *           占空比Duty = CCR / (ARR + 1)
 */
void PWM_SetMotor(uint8_t MotorNum, float Compare)
{
    if (Compare > 100 || Compare < -100 || (MotorNum != 1 && MotorNum != 2)) // 判断输入的值是否在范围内
    {
        return; // 若不在范围内，则直接返回
    }
    if (MotorNum == 1) {
        if (Compare > 0) // 若输入的值小于0，则将其转换为正数
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
            TIM_SetCompare1(TIM1, Compare * JDBL); // 设置CCR1的值pa8
        } else {
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            TIM_SetCompare1(TIM1, -Compare * JDBL); // 设置CCR4的值pa8
        }
    } else if (MotorNum == 2) {
        if (Compare > 0) {
            GPIO_SetBits(GPIOB, GPIO_Pin_12);
            GPIO_ResetBits(GPIOB, GPIO_Pin_13);
            TIM_SetCompare4(TIM1, Compare * JDBL);
        } else {
            GPIO_SetBits(GPIOB, GPIO_Pin_13);
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
            TIM_SetCompare4(TIM1, -Compare * JDBL); // 设置CCR4的值pa11
        }
    }
}