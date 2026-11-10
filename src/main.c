#include "stm32f10x.h" // 引入芯片接口
#include "OLED.h" // 引入显示接口
#include "PWM.h" // 引入电机输出接口
#include "MPU6050.h" // 引入姿态采样接口
#include "Serial.h" // 引入现有串口协议接口
#include "Timer.h" // 引入一毫秒定时调度
#include "PID.h" // 保留当前工程的 PID 实现与增益单位
#include "Encoder.h" // 引入轮速采样
#include "HCSR04.h" // 引入 PA3/PA1 非阻塞测距
#include "Avoidance.h" // 引入二十厘米避障策略
#include <math.h> // 引入角度和有限数值计算
#include <string.h> // 引入字段解析与接收快照
#include <stdlib.h> // 引入数值转换
#include <stdio.h> // 引入有界显示格式化
static volatile uint32_t milliseconds; // 保存前台刷新使用的毫秒计数
static volatile uint16_t distance; // 保存当前有效厘米数，零表示无效
static uint16_t obstacleStopCm = OBSTACLE_STOP_DEFAULT_CM; // 保存手机配置的避障阈值
static volatile uint8_t runFlag; // 保存当前电机运行状态
static volatile float speedRequest; // 保存尚未经过避障约束的速度命令
static volatile float turnRequest; // 保存方向命令且不受前方障碍屏蔽
static float angleOffset = 1.0f; // 保留当前代码的机械偏角初值
static float angleTargetRaw; // 保留原有 AngleTarget 参数接收兼容
static float angle; // 保存融合角度
static float differentialPwm; // 保存差速环输出
static PID_t anglePid = {.Kp=120.0f, .Ki=4.0f, .Kd=120.0f, .OutMax=3000.0f, .OutMin=-3000.0f, .ErrorIntAbsMax=1000.0f, .EnableIntErrorValue=10.0f}; // 保留恢复后代码的直立环配置
static PID_t speedPid = {.Kp=2.0f, .Ki=0.05f, .Kd=0.0f, .OutMax=20.0f, .OutMin=-20.0f, .ErrorIntAbsMax=10.0f, .EnableIntErrorValue=10.0f}; // 保留速度环对称输出限幅
static PID_t turnPid = {.Kp=50.0f, .Ki=90.0f, .Kd=0.0f, .OutMax=1200.0f, .OutMin=-1200.0f, .ErrorIntAbsMax=1200.0f, .EnableIntErrorValue=1200.0f}; // 保留方向环配置
static void ResetPid(PID_t *pid) // 清除一个控制环的旧状态
{ // 开始状态清理
    pid->Target = pid->Out = pid->Error0 = pid->Error1 = pid->ErrorInt = 0.0f; // 保留参数和反馈并清除旧控制量
} // 结束状态清理
static void StopControl(void) // 在停机或倾倒时清除旧命令
{ // 开始停机
    runFlag = 0; // 禁止继续运行
    speedRequest = turnRequest = 0.0f; // 丢弃停机前的运动命令
    ResetPid(&anglePid); // 清除直立环旧状态
    ResetPid(&speedPid); // 清除速度环旧状态
    ResetPid(&turnPid); // 清除方向环旧状态
    differentialPwm = 0.0f; // 清除旧差速输出
    PWM_SetMotor(1, 0); // 清除左轮占空比
    PWM_SetMotor(2, 0); // 清除右轮占空比
    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); // 关闭桥臂输入
} // 结束停机
static void ApplyMotionTargets(void) // 统一应用摇杆请求和距离约束
{ // 开始目标更新
    float target = Avoidance_Target(speedRequest, distance, obstacleStopCm); // 使用当前阈值限制前进并保留后退
    if (speedRequest > 0.0f && target == 0.0f) speedRequest = 0.0f; // 取消被拦截的旧前进请求以免障碍消失后自动恢复
    if (speedPid.Target != 0.0f && target == 0.0f) speedPid.ErrorInt = 0.0f; // 进入制动时去掉旧运动积分，保留速度反馈制动
    speedPid.Target = target; // 直接设置期望速度而不破坏速度环输出上下限
    turnPid.Target = turnRequest; // 方向归零始终生效
} // 结束目标更新
static void UpdateAttitude(void) // 每十毫秒采样与融合姿态
{ // 开始姿态更新
    int16_t ax, ay, az, gx, gy, gz; // 保存传感器原始数据
    MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz); // 保留当前传感器驱动
    float acc = atan2f((float)ay, (float)az) * 57.2957795f + angleOffset; // 计算加速度角度并叠加校准
    float gyro = angle + ((float)gx + 25.0f) / 32768.0f * 2000.0f * 0.01f; // 先转换浮点再补偿以避免十六位数值溢出
    angle = 0.99f * gyro + 0.01f * acc; // 保留当前互补滤波比例
    if (!isfinite(angle) || fabsf(angle) > 55.0f) StopControl(); // 保留倾倒停机并阻止异常输出
    anglePid.Actual = angle; // 更新直立反馈
} // 结束姿态更新
static void UpdateSpeedAndTurn(void) // 每五十毫秒执行速度与转向环
{ // 开始外环更新
    float left = Encoder_Get(1) / 44.0f / 0.05f / 9.27666f; // 保留左轮速度单位，停机时也清空编码器增量
    float right = Encoder_Get(2) / 44.0f / 0.05f / 9.27666f; // 保留右轮速度单位
    speedPid.Actual = (left + right) * 0.5f; // 计算平均速度
    turnPid.Actual = left - right; // 计算左右轮差速
    if (!runFlag) return; // 停机期间不积累控制输出
    PID_Update(&speedPid); // 目标为零时利用实测速度产生制动反馈
    anglePid.Target = -speedPid.Out; // 保留原有速度到倾角的映射并去掉随后被覆盖的无效平滑代码
    PID_Update(&turnPid); // 独立更新转向反馈
    differentialPwm = turnPid.Out; // 保存差速输出
} // 结束外环更新
static void UpdateBalance(void) // 每十毫秒执行直立控制与电机混合
{ // 开始内环更新
    if (!runFlag) return; // 停机后保持已清除的输出
    PID_Update(&anglePid); // 保留平衡控制，避障时不断开电机
    PWM_SetMotor(1, anglePid.Out + differentialPwm * 0.5f); // 输出左轮平衡与差速叠加量
    PWM_SetMotor(2, anglePid.Out - differentialPwm * 0.5f); // 输出右轮平衡与差速叠加量
} // 结束内环更新
static uint8_t ParseNumber(const char *text, float *value) // 完整校验串口数值
{ // 开始数值校验
    if (!text || !*text) return 0; // 拒绝缺失字段
    char *end; // 保存转换结束位置
    *value = strtof(text, &end); // 转换十进制数值
    return end != text && *end == '\0' && isfinite(*value); // 拒绝非法尾部和非有限数值
} // 结束数值校验
static void HandleParameter(const char *name, const char *text) // 根据名称更新一个合法参数
{ // 开始参数处理
    float value; // 保存校验后的数值
    if (!name || !ParseNumber(text, &value)) return; // 无效参数不更新任何状态
    if (!strcmp(name, "ObstacleStopCm")) { // 单独校验手机发送的避障阈值
        if (value < OBSTACLE_STOP_MIN_CM || value > OBSTACLE_STOP_MAX_CM || truncf(value) != value) return; // 仅接受五到一百厘米的整数
        uint32_t mask = __get_PRIMASK(); // 保存原中断状态
        __disable_irq(); // 原子更新阈值与运动目标
        obstacleStopCm = (uint16_t)value; // 保存有效避障距离
        ApplyMotionTargets(); // 调大阈值时立即取消已进入限制区的前进请求
        __set_PRIMASK(mask); // 恢复原中断状态
        return; // 不再进入普通 PID 参数处理
    } // 结束避障阈值更新
    static const char *names[] = {"AngleKp","AngleKi","AngleKd","SpeedKp","SpeedKi","SpeedKd","TurnKp","TurnKi","TurnKd","AngleOffset","AngleTarget"}; // 保留现有参数并兼容手机矫正角度键名
    float *parameters[] = {&anglePid.Kp,&anglePid.Ki,&anglePid.Kd,&speedPid.Kp,&speedPid.Ki,&speedPid.Kd,&turnPid.Kp,&turnPid.Ki,&turnPid.Kd,&angleOffset,&angleTargetRaw}; // 映射具体参数地址
    for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); ++i) { // 查找参数名称
        if (strcmp(name, names[i])) continue; // 跳过不匹配的参数
        if ((i < 9 && (value < 0 || value > 1000)) || (i >= 9 && fabsf(value) > 100)) return; // 限制异常输入范围
        uint32_t mask = __get_PRIMASK(); // 保存中断状态
        __disable_irq(); // 避免控制中断看到写入一半的参数
        *parameters[i] = value; // 写入一个合法数值
        __set_PRIMASK(mask); // 恢复原中断状态
        return; // 完成一个参数更新
    } // 结束名称查找
} // 结束参数处理
static void PollCommands(void) // 在前台解析当前固件支持的方括号帧内容
{ // 开始命令处理
    char packet[100]; // 使用局部快照避免接收缓存被下一帧改写
    uint32_t mask = __get_PRIMASK(); // 保存原中断状态
    __disable_irq(); // 原子获取完成帧与清除标志
    if (!*(volatile uint8_t *)&Serial_RxFlag) { __set_PRIMASK(mask); return; } // 没有完整帧时不清除未来到达的标志
    memcpy(packet, Serial_RxPacket, sizeof(packet)); // 拷贝现有驱动的百字节接收缓冲
    packet[sizeof(packet) - 1U] = '\0'; // 保证局部字符串有界
    Serial_RxFlag = 0; // 只有取得完整快照后才允许接收下一帧
    __set_PRIMASK(mask); // 在字符串解析前恢复接收中断
    char *field[6]; // 保存最多六个字段以检测多余字段
    unsigned count = 0; // 初始化字段计数
    char *next = packet; // 从首字段开始
    while (next && count < 6) { // 保留空字段用于合法性检查
        field[count++] = next; // 保存当前字段
        next = strchr(next, ','); // 查找下一字段
        if (next) *next++ = '\0'; // 结束当前字段
    } // 结束字段拆分
    if (next) return; // 拒绝多余字段
    if (count == 3 && !strcmp(field[0], "slider")) HandleParameter(field[1], field[2]); // 处理合法滑杆参数
    else if (count == 2 && !strcmp(field[0], "key")) { // 处理现有开关帧
        mask = __get_PRIMASK(); // 保存原中断状态
        __disable_irq(); // 保护多字段运行状态更新
        if (!strcmp(field[1], "pwmoff")) StopControl(); // 关闭时清除所有旧目标
        else if (!strcmp(field[1], "pwmon") && !runFlag) { StopControl(); runFlag = 1; } // 新会话从零运动目标启动
        __set_PRIMASK(mask); // 恢复原中断状态
    } else if (count == 5 && !strcmp(field[0], "joystick")) { // 解析全部四轴后才提交运动目标
        float axes[4]; // 保存完整校验后的轴值
        for (unsigned i = 0; i < 4; ++i) if (!ParseNumber(field[i + 1], &axes[i]) || fabsf(axes[i]) > 100 || truncf(axes[i]) != axes[i]) return; // 拒绝缺轴、越界和小数轴值
        mask = __get_PRIMASK(); // 保存原中断状态
        __disable_irq(); // 原子提交速度与方向请求
        if (runFlag) { speedRequest = axes[1] / 30.0f; turnRequest = axes[2] / 30.0f; ApplyMotionTargets(); } // 保留当前映射且归零不受障碍屏蔽
        __set_PRIMASK(mask); // 恢复原中断状态
    } // 结束命令分派
} // 结束前台命令处理
static void SendSpeedIntegralPlot(void) // 在前台发送速度环积分的两个实时通道
{ // 开始积分快照
    uint32_t mask = __get_PRIMASK(); // 保存原中断状态
    __disable_irq(); // 保证积分值和积分增益来自同一时刻
    float integral = speedPid.ErrorInt; // 获取速度环原始积分累积值
    float integralOutput = speedPid.Ki * integral; // 获取积分项实际贡献
    __set_PRIMASK(mask); // 在格式化和串口发送前恢复平衡控制中断
    Serial_Printf("[plot,%f]", speedPid.Target); // 按原始积分、积分输出的顺序发送且不追加换行
} // 结束积分发送
static void RefreshDisplay(void) // 每一百毫秒刷新状态而不在控制中断中发送调试文本
{ // 开始状态显示
    uint32_t mask = __get_PRIMASK(); // 保存原中断状态
    __disable_irq(); // 获取一致的显示快照
    uint16_t centimeters = distance; // 快照当前距离
    uint16_t stopCentimeters = obstacleStopCm; // 快照当前避障阈值
    int32_t measuredAngle = (int32_t)(angle * 10.0f); // 快照角度十分之一度单位
    int32_t targetSpeed = (int32_t)(speedPid.Target * 100.0f); // 快照速度目标百分之一单位
    uint8_t running = runFlag; // 快照运行状态
    __set_PRIMASK(mask); // 在慢速显示传输之前恢复中断
    OLED_ShowString(1, 1, running ? "RUN ANG:" : "OFF ANG:"); // 显示运行状态与角度标签
    OLED_ShowSignedNum(1, 9, measuredAngle, 4); // 显示当前倾角
    OLED_ShowString(2, 1, "D:"); // 显示当前距离标签
    OLED_ShowNum(2, 3, centimeters, 3); // 显示厘米距离，零代表无效
    OLED_ShowString(2, 7, "STOP:"); // 显示可配置阈值标签
    OLED_ShowNum(2, 12, stopCentimeters, 3); // 显示实际收到的避障距离
    OLED_ShowString(3, 1, !centimeters ? "RANGE INVALID   " : (centimeters < stopCentimeters ? "OBSTACLE BRAKE  " : "PATH CLEAR      ")); // 使用同一配置判断避障状态
    OLED_ShowString(4, 1, "SPD:"); // 显示目标速度标签
    OLED_ShowSignedNum(4, 5, targetSpeed, 4); // 显示最终避障约束后的速度
} // 结束状态显示
int main(void) // 初始化硬件并执行前台任务
{ // 开始主入口
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 在初始化各中断前统一分组
    PWM_Init(); // 先初始化电机输出
    StopControl(); // 上电明确关闭电机与旧控制状态
    OLED_Init(); // 初始化显示
    MPU6050_Init(); // 初始化现有姿态传感器
    Encoder_Init(); // 初始化轮速反馈
    HCSR04_Init(); // 初始化 PA3 触发与 PA1 回波捕获
    Serial_Init(); // 初始化现有蓝牙串口
    Timer_Init(); // 最后启动平衡控制调度
    uint32_t lastDisplay = 0; // 保存上次显示刷新时间
    for (;;) { // 持续运行前台任务
        PollCommands(); // 优先处理运动和归零指令
        uint32_t now = milliseconds; // 读取一毫秒时基
        if ((uint32_t)(now - lastDisplay) >= 100U) { // 将显示和积分发送限制为每秒十次
            lastDisplay = now; // 保存本轮发送起点以避免积压补发
            RefreshDisplay(); // 刷新当前状态
            SendSpeedIntegralPlot(); // 在前台发送当前速度环积分，控制中断仍可抢占
        } // 结束前台周期任务
    } // 结束前台循环
} // 结束主入口
void TIM2_IRQHandler(void) // 按当前一毫秒定时器驱动各控制任务
{ // 开始控制调度
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == RESET) return; // 忽略非更新中断
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 先清除中断事件
    ++milliseconds; // 推进显示时基
    static uint8_t balanceTicks, speedTicks; // 分别记录内外环采样分频
    HCSR04_Tick(); // 只推进测距状态，不等待回波或超时
    distance = HCSR04_GetDistanceCm(); // 每周期读取最新测距结果
    ApplyMotionTargets(); // 在最晚一个调度周期内将近距前进目标归零
    uint8_t balanceDue = ++balanceTicks >= 10U; // 检查十毫秒姿态与平衡周期
    if (balanceDue) { balanceTicks = 0; UpdateAttitude(); } // 先更新新姿态
    if (++speedTicks >= 50U) { speedTicks = 0; UpdateSpeedAndTurn(); } // 用零目标反馈制动并更新方向控制
    if (balanceDue) UpdateBalance(); // 使用最新姿态和速度目标输出平衡控制
} // 结束控制调度
