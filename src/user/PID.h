#ifndef __PID_H
#define __PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float Target;
    float Actual;
    float Out;

    float Error0;
    float Error1;
    float ErrorInt;

    float OutMax;
    float OutMin;

    float ErrorIntAbsMax; // 积分项绝对值最大值
    float EnableIntErrorValue; // 允许积分的误差值范围，超过这个范围就不进行积分
} PID_t;
void PID_Update(PID_t *p);
#endif
