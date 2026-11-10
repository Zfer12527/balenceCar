#include "stm32f10x.h" // 引入引脚、外部中断和内核计时
#include "HCSR04.h" // 引入测距接口
enum { IDLE, WAIT_RISE, WAIT_FALL }; // 定义非阻塞测量状态
static volatile uint8_t state; // 与边沿中断共享测量状态
static volatile uint32_t riseCycles; // 保存回波上升沿时间
static volatile uint16_t distanceCm; // 保存最新完整测量结果
static volatile uint32_t resultCycles; // 保存结果更新时间
static uint32_t cyclesPerUs; // 内核周期到微秒的换算系数
static uint32_t triggerCycles; // 保存最近一次触发时间
static uint8_t clockReady; // 标记计时器可用
static void WaitUs(uint32_t microseconds) // 仅用于产生短触发脉冲
{ // 开始短延时
    uint32_t start = DWT->CYCCNT; // 保存周期起点
    while ((uint32_t)(DWT->CYCCNT - start) < microseconds * cyclesPerUs) { __NOP(); } // 兼容计数回绕
} // 结束短延时
void HCSR04_Init(void) // 初始化超声波硬件
{ // 开始硬件初始化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); // 开启引脚与边沿映射时钟
    GPIO_InitTypeDef gpio; // 定义引脚配置
    GPIO_StructInit(&gpio); // 初始化全部配置字段
    gpio.GPIO_Pin = GPIO_Pin_3; // TRIG 使用 PA3
    gpio.GPIO_Mode = GPIO_Mode_Out_PP; // 触发使用推挽输出
    gpio.GPIO_Speed = GPIO_Speed_2MHz; // 设置触发引脚速度
    GPIO_Init(GPIOA, &gpio); // 应用触发引脚配置
    GPIO_ResetBits(GPIOA, GPIO_Pin_3); // 保证初始触发电平为低
    gpio.GPIO_Pin = GPIO_Pin_1; // ECHO 使用 PA1
    gpio.GPIO_Mode = GPIO_Mode_IPD; // 使用下拉输入避免空闲悬空
    GPIO_Init(GPIOA, &gpio); // 应用回波引脚配置
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 开启内核调试计时资源
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 启动周期计时但不清除共享计数
    __DSB(); // 确保计数器配置完成
    __ISB(); // 同步后续指令
    cyclesPerUs = SystemCoreClock / 1000000U; // 按实际内核频率换算
    uint32_t probe = DWT->CYCCNT; // 保存计数探测起点
    for (unsigned i = 0; i < 16; ++i) __NOP(); // 留出足够计数推进时间
    clockReady = cyclesPerUs != 0U && DWT->CYCCNT != probe; // 检查周期计数确实工作
    state = IDLE; // 初始化测量状态
    distanceCm = 0; // 上电没有有效测距
    triggerCycles = DWT->CYCCNT - 60000U * cyclesPerUs; // 允许第一次调度立即触发
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1); // 将 PA1 连接到 EXTI1
    EXTI_InitTypeDef exti; // 定义外部中断配置
    EXTI_StructInit(&exti); // 初始化全部配置字段
    exti.EXTI_Line = EXTI_Line1; // 选择回波对应的中断线
    exti.EXTI_Mode = EXTI_Mode_Interrupt; // 使用中断而非轮询等待
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // 捕获回波上升沿和下降沿
    exti.EXTI_LineCmd = ENABLE; // 开启边沿捕获
    EXTI_ClearITPendingBit(EXTI_Line1); // 清除初始化残留事件
    EXTI_Init(&exti); // 应用边沿配置
    NVIC_InitTypeDef irq = {EXTI1_IRQn, 0, 0, ENABLE}; // 让时间戳采样高于控制与串口中断
    NVIC_Init(&irq); // 开启回波中断入口
} // 结束硬件初始化
void HCSR04_Tick(void) // 推进测量而不等待回波
{ // 开始测量调度
    if (!clockReady) return; // 计时无效时保持无效距离
    uint32_t now = DWT->CYCCNT; // 读取实际经过的时间
    uint32_t mask = __get_PRIMASK(); // 保存原中断状态
    __disable_irq(); // 原子检查超时，避免覆盖刚完成的边沿结果
    if (state != IDLE && (uint32_t)(now - triggerCycles) >= 30000U * cyclesPerUs) { // 最多等待三十毫秒回波
        state = IDLE; // 超时后结束当前测量
        distanceCm = 0; // 不继续沿用旧的远距离结果
    } // 结束超时处理
    uint8_t ready = state == IDLE && (uint32_t)(now - triggerCycles) >= 60000U * cyclesPerUs; // 两次触发至少间隔六十毫秒
    if (ready) { // 准备发起新测量
        triggerCycles = now; // 保存触发时刻
        if (GPIOA->IDR & GPIO_Pin_1) distanceCm = 0; // 回波持续高电平时判定当前不可测量
        else state = WAIT_RISE; // 在触发前准备好上升沿接收
    } // 结束测量准备
    uint8_t trigger = ready && state == WAIT_RISE; // 保存是否需要触发
    __set_PRIMASK(mask); // 在发送脉冲前恢复边沿中断
    if (!trigger) return; // 当前没有新的触发任务
    GPIO_ResetBits(GPIOA, GPIO_Pin_3); // 确保触发前为低电平
    WaitUs(2); // 保持触发前的低电平
    GPIO_SetBits(GPIOA, GPIO_Pin_3); // 开始触发高电平
    WaitUs(12); // 满足模块要求的触发脉宽
    GPIO_ResetBits(GPIOA, GPIO_Pin_3); // 结束触发并交由边沿中断接收
} // 结束测量调度
void EXTI1_IRQHandler(void) // 捕获精确的回波边沿
{ // 开始边沿处理
    if (EXTI_GetITStatus(EXTI_Line1) == RESET) return; // 忽略非回波事件
    EXTI_ClearITPendingBit(EXTI_Line1); // 先清事件以捕获下一边沿
    uint32_t now = DWT->CYCCNT; // 立即读取边沿时间戳
    if (state == WAIT_RISE && (GPIOA->IDR & GPIO_Pin_1)) { // 收到期待的上升沿
        riseCycles = now; // 保存高电平起点
        state = WAIT_FALL; // 等待高电平结束
    } else if (state == WAIT_FALL && !(GPIOA->IDR & GPIO_Pin_1)) { // 收到期待的下降沿
        uint32_t microseconds = (now - riseCycles) / cyclesPerUs; // 换算回波高电平宽度
        distanceCm = microseconds >= 116U && microseconds <= 23200U ? (uint16_t)(microseconds / 58U) : 0; // 校验量程并向下取整厘米
        resultCycles = now; // 更新结果时效
        state = IDLE; // 完成一次测量
    } // 结束边沿分派
} // 结束边沿处理
uint16_t HCSR04_GetDistanceCm(void) // 读取一致且尚未过期的测量结果
{ // 开始距离快照
    uint32_t mask = __get_PRIMASK(); // 保存原中断状态
    __disable_irq(); // 防止结果与时间戳来自不同测量
    uint16_t value = distanceCm; // 复制距离
    uint32_t timestamp = resultCycles; // 复制结果时间
    __set_PRIMASK(mask); // 恢复原中断状态
    if (!clockReady || (uint32_t)(DWT->CYCCNT - timestamp) > 120000U * cyclesPerUs) return 0; // 超过一百二十毫秒的旧结果视为无效
    return value; // 返回有效厘米数
} // 结束距离快照
uint16_t HCSR04_MeasureCm(void) { return HCSR04_GetDistanceCm(); } // 保留旧名称但不阻塞控制中断
