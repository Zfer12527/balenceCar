#ifndef CAR_HCSR04_H // 防止接口重复包含
#define CAR_HCSR04_H // 标记接口已包含
#include <stdint.h> // 提供距离数据类型
void HCSR04_Init(void); // 初始化 TRIG=PA3、ECHO=PA1 和边沿中断
void HCSR04_Tick(void); // 每毫秒推进测距，只有十四微秒触发脉冲需要短等待
uint16_t HCSR04_GetDistanceCm(void); // 获取最新结果，零表示无效或已经过期
uint16_t HCSR04_MeasureCm(void); // 兼容旧调用名称，现为非阻塞读取
#endif // 结束接口保护
