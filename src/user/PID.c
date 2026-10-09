#include "stm32f10x.h"
#include "PID.h"
void PID_Update(PID_t *p)
{
    p->Error1 = p->Error0;             // 保存上一次误差
    p->Error0 = p->Target - p->Actual; // 计算本次误差
    if (p->Actual < 30 && p->Actual > -30) {
        p->ErrorInt += p->Error0; // 误差积分
    }

    p->ErrorInt = p->ErrorInt > 500 ? 500 : p->ErrorInt;
    p->ErrorInt = p->ErrorInt < -500 ? -500 : p->ErrorInt;
    p->out      = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1); // 计算输出
    if (p->out > p->OutMax)                                                                  // 输出限幅
    {
        p->out = p->OutMax;
    } else if (p->out < p->OutMin) {
        p->out = p->OutMin;
    }
}