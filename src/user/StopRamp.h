#ifndef CAR_STOP_RAMP_H // 防止减速曲线接口重复包含
#define CAR_STOP_RAMP_H // 标记接口已包含
#include <stdint.h> // 提供回绕安全的毫秒计数类型
#define STOP_RAMP_DURATION_MS 500U // 普通松手后用0.5 秒将速度目标降到零
typedef struct { // 保存一次松手减速的起点且不受后续零值帧重置
    float initialSpeed; // 保存松手时的速度目标及方向
    uint32_t startedAt; // 保存本次减速开始时间
    uint8_t active; // 标记当前减速曲线是否正在执行
} StopRamp; // 结束减速状态定义
static inline void StopRamp_Cancel(StopRamp *ramp) // 停机、避障或新非零指令立即取消旧减速过程
{ // 开始状态清理
    ramp->active = 0; // 不再使用旧的起点
    ramp->initialSpeed = 0.0f; // 清除旧方向与幅度
} // 结束状态清理
static inline float StopRamp_Target(StopRamp *ramp, float currentTarget, uint32_t now) // 连续收到零命令时保持同一条减速曲线
{ // 开始减速计算
    if (!ramp->active) { // 只在第一次进入松手状态时记录起点
        if (currentTarget == 0.0f) return 0.0f; // 已经停止时不启动新的计时
        ramp->initialSpeed = currentTarget; // 固定本次减速的初始目标
        ramp->startedAt = now; // 固定本次松手时刻
        ramp->active = 1; // 开始一次减速过程
    } // 结束起点记录
    uint32_t elapsed = now - ramp->startedAt; // 无符号差值兼容毫秒时基回绕
    if (elapsed >= STOP_RAMP_DURATION_MS) { StopRamp_Cancel(ramp); return 0.0f; } // 0.5 秒后精确归零而不跨过零点
    float progress = (float)elapsed / (float)STOP_RAMP_DURATION_MS; // 将实际经过时间映射到零到一
    float remaining = 1.0f - progress; // 计算尚未完成的时间比例
    float scale = remaining * remaining * (1.0f + 2.0f * progress); // 三次平滑曲线保证开始和结束时目标变化率为零
    if (scale > 1.0f) scale = 1.0f; // 消除浮点舍入导致的微小幅度增长
    return ramp->initialSpeed * scale; // 只缩小幅度，始终保持松手前的方向直到零
} // 结束减速计算
#endif // 结束接口保护
