#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "math.h"
#include "MPU6050.h"
#include "Serial.h"
#include "Timer.h"
#include "PID.h"
#include "string.h"
#include "stdlib.h"
#include "Encoder.h"
#define M_PI 3.14159
char strArray[20];
uint16_t execTime;
int16_t AX, AY, AZ, GX, GY, GZ;
float Angle;
float AngleAcc;
float AngleGyro;
uint8_t RunFlag = 1;
int16_t LeftPWM;
int16_t RightPWM;
int16_t AveragePWM;
int16_t DiffPWM;
float AngleOffset  = 1.0;
static float Alpha = 0.99;
// uint8_t PwmEnable = 1;
float LeftSpeed,
    RightSpeed,
    AverageSpeed,
    DiffSpeed;
float AngleTargetRaw = 0; // 摇杆给出的原始速度目标，经斜坡后才写入SpeedPID.Target
float SpeedTargetRaw = 0;
PID_t AnglePID       = {
    .Kp                  = 120.0,
    .Ki                  = 4.0, // 直立环只用PD，禁止加积分（停车摆动主因）
    .Kd                  = 120.0,
    .Target              = 0.0,
    .OutMax              = 3000,
    .OutMin              = -3000,
    .ErrorIntAbsMax      = 1000,
    .EnableIntErrorValue = 10};
PID_t SpeedPID = {
    .Kp                  = 2.0,
    .Ki                  = 0.05, // 速度环用PI，积分从小往大调
    .Kd                  = 0.0,
    .OutMax              = 30, // 输出到角度环
    .OutMin              = -30,
    .ErrorIntAbsMax      = 10,
    .EnableIntErrorValue = 10};

PID_t TurnPID = {
    .Kp                  = 50.0,
    .Ki                  = 90.00, // 速度环用PI，积分从小往大调
    .Kd                  = 0.0,
    .OutMax              = 1200, // 输出到角度环
    .OutMin              = -1200,
    .ErrorIntAbsMax      = 1200,
    .EnableIntErrorValue = 1200};
// 判断 str 是否以 prefix 开头
int StringStartWith(char *str, char *prefix)
{
    // 前缀长度
    size_t pre_len = strlen(prefix);
    // 源字符串长度必须 >= 前缀长度，并且前pre_len字符相等
    if (strlen(str) >= pre_len && strncmp(str, prefix, pre_len) == 0) {
        return 1; // 是，以前缀开头
    }
    return 0; // 不是
}
void showSomething(void)
{
    OLED_ShowSignedNum(1, 1, execTime, 6);
    sprintf(strArray, "kp%.2f", TurnPID.Kp);
    OLED_ShowString(2, 1, strArray);
    sprintf(strArray, "ki%.2f", TurnPID.Ki);
    OLED_ShowString(3, 1, strArray);
    sprintf(strArray, "kd%.2f", TurnPID.Kd);
    OLED_ShowString(4, 1, strArray);
    // Serial_Printf("[plot,%f,%f,%f]", AnglePID.Target, AnglePID.Actual, AnglePID.Error0, AnglePID.Ki * AnglePID.ErrorInt);
    // Serial_Printf("[plot,%f,%f,%f,%f]", SpeedPID.Target, SpeedPID.Actual, SpeedPID.Error0, SpeedPID.Ki * SpeedPID.ErrorInt);
    Serial_Printf("[plot,%f,%f]", SpeedPID.Target, SpeedPID.Out);
    // Serial_Printf("[plot,%f,%f,%f]", LeftSpeed,DiffSpeed, RightSpeed);
}
void dealReceivedData(void)
{
    if (Serial_RxFlag == 1) // 如果接收到数据包
    {
        char *Tag = strtok(Serial_RxPacket, ","); // 提取数据1，定义为标签Tag
        if (strcmp(Tag, "key") == 0)              // Tag为key，收到按键数据包
        {
            char *Name   = strtok(NULL, ","); // 提取数据2，定义为按键名称
            char *Action = strtok(NULL, ","); // 提取数据3，定义为按键动作
            if (strcmp(Name, "pwmon") == 0)   // 如果滑杆名称是AngleKp
            {
                // PwmEnable = 1;
                RunFlag = 1;
            } else if (strcmp(Name, "pwmoff") == 0) // 如果滑杆名称是AngleKp
            {
                // PwmEnable = 0;
                RunFlag = 0;
                GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
            }
            /*此处可执行按键操作，目前程序暂时没用到按键*/
        } else if (strcmp(Tag, "slider") == 0) // Tag为slider，收到滑杆数据包
        {
            char *Name  = strtok(NULL, ","); // 提取数据2，定义为滑杆名称
            char *Value = strtok(NULL, ","); // 提取数据3，定义为滑杆值

            /*执行滑杆操作*/
            if (strcmp(Name, "AngleKp") == 0) // 如果滑杆名称是AngleKp
            {
                AnglePID.Kp = atof(Value);           // 则把滑杆值赋值给角度环Kp
            } else if (strcmp(Name, "AngleKi") == 0) // 如果滑杆名称是AngleKi
            {
                AnglePID.Ki = atof(Value);           // 则把滑杆值赋值给角度环Ki
            } else if (strcmp(Name, "AngleKd") == 0) // 如果滑杆名称是AngleKd
            {
                AnglePID.Kd = atof(Value);           // 则把滑杆值赋值给角度环Kd
            } else if (strcmp(Name, "SpeedKp") == 0) // 如果滑杆名称是SpeedKp
            {
                SpeedPID.Kp = atof(Value);           // 则把滑杆值赋值给速度环Kp
            } else if (strcmp(Name, "SpeedKi") == 0) // 如果滑杆名称是SpeedKi
            {
                SpeedPID.Ki = atof(Value);           // 则把滑杆值赋值给速度环Ki
            } else if (strcmp(Name, "SpeedKd") == 0) // 如果滑杆名称是SpeedKd
            {
                SpeedPID.Kd = atof(Value);          // 则把滑杆值赋值给速度环Kd
            } else if (strcmp(Name, "TurnKp") == 0) // 如果滑杆名称是TurnKp
            {
                TurnPID.Kp = atof(Value);           // 则把滑杆值赋值给转向环Kp
            } else if (strcmp(Name, "TurnKi") == 0) // 如果滑杆名称是TurnKi
            {
                TurnPID.Ki = atof(Value);           // 则把滑杆值赋值给转向环Ki
            } else if (strcmp(Name, "TurnKd") == 0) // 如果滑杆名称是TurnKd
            {
                TurnPID.Kd = atof(Value);                // 则把滑杆值赋值给转向环Kd
            } else if (strcmp(Name, "AngleTarget") == 0) // 如果滑杆名称是offset
            {
                AngleTargetRaw = atof(Value); // 则把滑杆值赋值给角度偏移
                // AnglePID.Target = atof(Value); // 则把滑杆值赋值给角度偏移
                // OLED_ShowString(1, 1, Value);
            }
        } else if (strcmp(Tag, "joystick") == 0) // Tag为joystick，收到摇杆数据包
        {
            int8_t LH       = atoi(strtok(NULL, ",")); // 提取数据2，定义为摇杆值LH
            int8_t LV       = atoi(strtok(NULL, ",")); // 提取数据3，定义为摇杆值LV
            int8_t RH       = atoi(strtok(NULL, ",")); // 提取数据4，定义为摇杆值RH
            int8_t RV       = atoi(strtok(NULL, ",")); // 提取数据5，定义为摇杆值RV
            SpeedPID.Target = LV / 30.0;               // 左摇杆垂直方向只改原始目标，斜坡在中断里做
            // SpeedTargetRaw = LV / 50.0;               // 左摇杆垂直方向只改原始目标，斜坡在中断里做
            // SpeedTargetRaw  = LV / 50.0;               // 左摇杆垂直方向只改原始目标，斜坡在中断里做
            TurnPID.Target = RH / 30.0;
        }
    }
}
int main(void)
{
    OLED_Init();
    MPU6050_Init();
    Encoder_Init();
    PWM_Init();
    Serial_Init();
    Timer_Init();
    while (1) {
        showSomething();
        dealReceivedData();
        Serial_RxFlag = 0; // 处理完成后，标志位置0，允许接收下一个数据包
    }
}
void TIM2_IRQHandler(void)
{
    static uint16_t startTime;

    static uint8_t pid_count;
    static uint8_t pid_count2;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) // Check if the update interrupt flag is set
    {
        startTime = TIM_GetCounter(TIM2);
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // Clear the update interrupt flag
        if (++pid_count >= 10) {
            pid_count = 0;
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            GX += 25;
            AngleAcc = atan2(AY, AZ) / M_PI * 180;
            AngleAcc += AngleOffset;                                // 加速度计角度
            AngleGyro = Angle + GX / 32768.0 * 2000 * 0.01;         // 陀螺仪往总角度积分
            Angle     = Alpha * AngleGyro + (1 - Alpha) * AngleAcc; // 互补滤波
            if (Angle > 55 || Angle < -55) {
                RunFlag = 0;
            }
            if (RunFlag) {
                AnglePID.Actual = Angle;
                PID_Update(&AnglePID);
                AveragePWM = AnglePID.Out;
                LeftPWM  = AveragePWM + DiffPWM / 2;
                RightPWM = AveragePWM - DiffPWM / 2;
                PWM_SetMotor(1, LeftPWM);
                PWM_SetMotor(2, RightPWM);
                // Serial_Printf("%f,%f,%f\r\n", AnglePID.Kp, AnglePID.Ki, AnglePID.Kd);
                //  Serial_Printf("%f,%f,%f\r\n", AnglePID.Out, AnglePID.Error0, Angle);
            } else {
                AnglePID.ErrorInt = 0;
                GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
            }
        }

        if (++pid_count2 >= 50) {
            pid_count2 = 0;
            if (RunFlag) {
                LeftSpeed    = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
                RightSpeed   = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;
                AverageSpeed = (LeftSpeed + RightSpeed) / 2.0;
                DiffSpeed    = LeftSpeed - RightSpeed;

                SpeedPID.Actual = AverageSpeed;
                PID_Update(&SpeedPID);
                AnglePID.Target += (-SpeedPID.Out - AnglePID.Target) * 0.5f;
                AnglePID.Target = -SpeedPID.Out;

                TurnPID.Actual = DiffSpeed;
                PID_Update(&TurnPID);
                DiffPWM = TurnPID.Out;

                execTime = TIM_GetCounter(TIM2) - startTime;
            }
        }
    }
}