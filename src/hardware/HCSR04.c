#include "stm32f10x.h"
#include <stdbool.h>
#define HCSR04_TRIG_PORT      GPIOA
#define HCSR04_TRIG_PIN       GPIO_Pin_3
#define HCSR04_ECHO_PORT      GPIOA
#define HCSR04_ECHO_PIN       GPIO_Pin_1
// 时序参数（单位：微秒）
#define HCSR04_TRIG_WIDTH_US  12U     // 触发脉冲宽度（原厂要求≥10us，留2us余量）
#define HCSR04_INTERVAL_US    60000U  // 两次测量最小间隔，避免余波干扰
#define HCSR04_ECHO_TIMEOUT_US 30000U // 回波总超时，对应最大约5m量程

// 量程对应回波时间（单位：微秒）
#define HCSR04_MIN_ECHO_US    116U    // 最小量程2cm对应回波时间
#define HCSR04_MAX_ECHO_US    23200U  // 最大量程4m对应回波时间

// 距离计算整数系数（声速340m/s，往返路程，近似公式：mm = (us*10 +29)/58）
#define HCSR04_DIST_NUM       10U
#define HCSR04_DIST_DEN       58U
#define HCSR04_DIST_ROUND     29U
/************************  类型定义  ************************/
typedef enum {
    HCSR04_OK = 0,          // 测量成功
    HCSR04_ERR_TIMER,       // DWT定时器未就绪
    HCSR04_ERR_ECHO_BUSY,   // 测量前ECHO已为高（引脚故障/模块忙）
    HCSR04_ERR_NO_ECHO,     // 未检测到回波上升沿
    HCSR04_ERR_ECHO_TIMEOUT,// 回波高电平持续超时
    HCSR04_ERR_OUT_OF_RANGE // 结果超出有效量程
} HCSR04_Status_t;

// 传感器实例结构体：聚合所有相关变量，减少零散全局变量
typedef struct {
    HCSR04_Status_t status; // 最新测量状态
    uint32_t echo_us;       // 回波高电平时长（微秒）
    uint32_t distance_mm;   // 测量距离（毫米，整数运算无精度损失）
    uint32_t cycles_per_us; // 每微秒对应的内核时钟周期数
    uint32_t last_trig_cyc; // 上一次触发的CYCCNT值
    bool timer_ready;       // DWT定时器就绪标志
    bool has_triggered;     // 是否已执行过触发（用于间隔控制）
} HCSR04_HandleTypeDef;

/************************  内部静态变量与辅助函数  ************************/
static HCSR04_HandleTypeDef hcsr04;
// 计算从start到当前的周期数（32位无符号减法自动处理溢出）
static uint32_t elapsed_cycles(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start);
}

// 微秒级阻塞延时
static void wait_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t total_cyc = us * hcsr04.cycles_per_us;
    while (elapsed_cycles(start) < total_cyc);
}

// 等待ECHO引脚达到指定电平，超时返回false
static bool wait_echo_level(bool target_level, uint32_t timeout_cyc)
{
    uint32_t start = DWT->CYCCNT;
    GPIO_TypeDef *port = HCSR04_ECHO_PORT;
    uint16_t pin = HCSR04_ECHO_PIN;

    while (1) {
        bool current = (port->IDR & pin) != 0U;
        if (current == target_level) return true;
        if (elapsed_cycles(start) >= timeout_cyc) return false;
    }
}

// 发送一次触发脉冲
static void send_trigger(void)
{
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    wait_us(2);
    GPIO_SetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    wait_us(HCSR04_TRIG_WIDTH_US);
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    hcsr04.last_trig_cyc = DWT->CYCCNT;
    hcsr04.has_triggered = true;
}
void HCSR04_Init(void)
{
    GPIO_InitTypeDef gpio_cfg;
    uint32_t test_start;

    // 初始化句柄默认值
    hcsr04.status        = HCSR04_ERR_TIMER;
    hcsr04.echo_us       = 0;
    hcsr04.distance_mm   = 0;
    hcsr04.timer_ready   = false;
    hcsr04.has_triggered = false;

    // 计算时钟基准
    SystemCoreClockUpdate();
    hcsr04.cycles_per_us = SystemCoreClock / 1000000U;

    // 开启GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // TRIG引脚：推挽输出，初始低电平
    GPIO_StructInit(&gpio_cfg);
    gpio_cfg.GPIO_Pin   = HCSR04_TRIG_PIN;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio_cfg.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(HCSR04_TRIG_PORT, &gpio_cfg);
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    // ECHO引脚：下拉输入，确保空闲电平稳定
    gpio_cfg.GPIO_Pin  = HCSR04_ECHO_PIN;
    gpio_cfg.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(HCSR04_ECHO_PORT, &gpio_cfg);

    // 使能DWT周期计数器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();

    // 校验DWT是否正常运行
    test_start = DWT->CYCCNT;
    for (uint8_t i = 0; i < 16; i++) __NOP();

    if (hcsr04.cycles_per_us != 0U && DWT->CYCCNT != test_start) {
        hcsr04.timer_ready = true;
        hcsr04.status      = HCSR04_OK;
    }
}
// 执行一次测量，返回状态码；结果存在hcsr04结构体中
HCSR04_Status_t HCSR04_Measure(void)
{
    uint32_t rise_cyc;
    uint32_t timeout_cyc = HCSR04_ECHO_TIMEOUT_US * hcsr04.cycles_per_us;

    hcsr04.echo_us = 0;
    hcsr04.distance_mm = 0;

    // 1. 定时器就绪检查
    if (!hcsr04.timer_ready) {
        hcsr04.status = HCSR04_ERR_TIMER;
        return hcsr04.status;
    }

    // 2. 强制保证两次触发最小间隔
    if (hcsr04.has_triggered) {
        uint32_t interval_cyc = HCSR04_INTERVAL_US * hcsr04.cycles_per_us;
        while (elapsed_cycles(hcsr04.last_trig_cyc) < interval_cyc);
    }

    // 3. 检查ECHO是否空闲
    if (!wait_echo_level(false, timeout_cyc)) {
        hcsr04.status = HCSR04_ERR_ECHO_BUSY;
        return hcsr04.status;
    }

    // 4. 发送触发脉冲
    send_trigger();

    // 5. 等待回波上升沿
    if (!wait_echo_level(true, timeout_cyc)) {
        hcsr04.status = HCSR04_ERR_NO_ECHO;
        return hcsr04.status;
    }
    rise_cyc = DWT->CYCCNT;

    // 6. 等待回波下降沿
    if (!wait_echo_level(false, timeout_cyc)) {
        hcsr04.status = HCSR04_ERR_ECHO_TIMEOUT;
        return hcsr04.status;
    }

    // 7. 计算回波时长（四舍五入到微秒）
    uint32_t pulse_cyc = elapsed_cycles(rise_cyc);
    hcsr04.echo_us = (pulse_cyc + hcsr04.cycles_per_us / 2U) / hcsr04.cycles_per_us;

    // 8. 量程校验
    if (hcsr04.echo_us < HCSR04_MIN_ECHO_US || hcsr04.echo_us > HCSR04_MAX_ECHO_US) {
        hcsr04.status = HCSR04_ERR_OUT_OF_RANGE;
        return hcsr04.status;
    }

    // 9. 整数运算计算距离（毫米），完全避免浮点开销
    hcsr04.distance_mm = (hcsr04.echo_us * HCSR04_DIST_NUM + HCSR04_DIST_ROUND) / HCSR04_DIST_DEN;
    hcsr04.status = HCSR04_OK;

    return hcsr04.status;
}
// 兼容原有接口：返回厘米浮点值，失败返回-1
uint16_t HCSR04_MeasureCm(void)
{
    if (HCSR04_Measure() != HCSR04_OK) {
        return 0;
    }
    return hcsr04.distance_mm / 10;
}