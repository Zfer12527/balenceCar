#ifndef __PID_H
#define __PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float Target;
    float Actual;
    float out;

    float Error0;
    float Error1;
    float ErrorInt;

    float OutMax;
    float OutMin;
} PID_t;
void PID_Update(PID_t *p);
#endif
