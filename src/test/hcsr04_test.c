// /* HC-SR04 阻塞式驱动优化版
//  * 硬件连接：TRIG -> PA3，ECHO -> PA1（5V模块必须分压，PA1非5V容忍引脚）
//  * 分压参考：ECHO -- 1kΩ -- PA1 -- 2kΩ -- GND，模块与MCU必须共地
//  * 调试串口：USART3 TX=PB10，9600 8N1
//  * 计时基准：内核DWT周期计数器，不占用TIM/SysTick，不修改共享计数器
//  */
// #include "stm32f10x.h"
// #include "Serial.h"
// #include "HCSR04.h"
// #include "OLED.h"
// #include "Delay.h"

// int main(void)
// {
//     Serial_Init();
//     OLED_Init();
//     HCSR04_Init();


//     for (;;) {
//         uint16_t res = HCSR04_MeasureCm();
//         OLED_ShowNum(1, 1, res, 8);
//         Delay_ms(100); // 额外循环间隔
//     }
// }
