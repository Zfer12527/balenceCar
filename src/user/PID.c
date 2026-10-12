#include "stm32f10x.h" // Device header
#include "PID.h"
void PID_Update(PID_t *p)
{
    /*获取本次误差和上次误差*/
    p->Error1 = p->Error0;             // 获取上次误差
    p->Error0 = p->Target - p->Actual; // 获取本次误差，目标值减实际值，即为误差值

    /*外环误差积分（累加）*/
    /*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
    /*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
    /*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
    if (p->Ki == 0) {
        p->ErrorInt = 0;
    } else if (p->Error0 < p->EnableIntErrorValue && p->Error0 > -p->EnableIntErrorValue) // 如果Ki不为0
    {
        p->ErrorInt += p->Error0; // 进行误差积分
    }

    if ((p->Ki * p->ErrorInt) > p->ErrorIntAbsMax) { p->ErrorInt = p->ErrorIntAbsMax / p->Ki; }   // 限制误差积分最大值
    if ((p->Ki * p->ErrorInt) < -p->ErrorIntAbsMax) { p->ErrorInt = -p->ErrorIntAbsMax / p->Ki; } // 限制误差积分最小值
    /*PID计算*/
    /*使用位置式PID公式，计算得到输出值*/
    float temp = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1);

    /*输出限幅*/
    if (temp > p->OutMax) { temp = p->OutMax; } // 限制输出值最大为结构体指定的OutMax
    if (temp < p->OutMin) { temp = p->OutMin; } // 限制输出值最小为结构体指定的OutMin
    
    p->Out = temp;
}
