#ifndef CAR_AVOIDANCE_H // 防止重复包含避障策略
#define CAR_AVOIDANCE_H // 标记接口已包含
#include <stdint.h> // 提供距离类型
#define OBSTACLE_STOP_DEFAULT_CM 20U // 上电默认阈值为二十厘米
#define OBSTACLE_STOP_MIN_CM 5U // 手机滑杆允许的最小阈值
#define OBSTACLE_STOP_MAX_CM 100U // 手机滑杆允许的最大阈值
static inline float Avoidance_Target(float requested, uint16_t centimeters, uint16_t stopCentimeters) // 使用当前配置约束前进目标
{ // 开始避障策略
    return requested > 0.0f && (centimeters == 0U || centimeters < stopCentimeters) ? 0.0f : requested; // 严格小于配置值时暂停前进并保留后退
} // 结束避障策略
#endif // 结束接口保护
