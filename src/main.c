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
#define M_PI 3.14159265358979323846
int16_t AX, AY, AZ, GX, GY, GZ;
float Angle;
float AngleAcc;
float AngleGyro;
uint8_t RunFlag = 0;
float LeftPWM;
float RightPWM;
float AveragePWM;
float DiffPWM;
float AngleOffset = 0;
PID_t AnglePID = {
    .Kp     = 3.0,
    .Ki     = 0.1,
    .Kd     = 3.0,
    .OutMax = 100,
    .OutMin = -100};
    // 判断 str 是否以 prefix 开头
int StringStartWith(char *str, char *prefix)
{
    // 前缀长度
    size_t pre_len = strlen(prefix);
    // 源字符串长度必须 >= 前缀长度，并且前pre_len字符相等
    if(strlen(str) >= pre_len && strncmp(str, prefix, pre_len) == 0)
    {
        return 1; // 是，以前缀开头
    }
    return 0; // 不是
}
int main(void)
{
    
    Delay_ms(300);
    /*模块初始化*/
    		//OLED初始化
    // PWM_Init();			//PWM初始化
    // PWM_SetCompare1(10);	//设置PWM的占空比为30%
    
   
    PWM_Init();

    Timer_Init();
    MPU6050_Init();
    OLED_Init();
    Serial_Init();
    Serial_Printf("%f\r\n", 1.23);
    while (1) {
        if (Serial_RxFlag == 1)		//如果接收到数据包
		{
            Serial_RxFlag = 0;			//处理完成后，需要将接收数据包标志位清零，否则将无法接收后续数据包
			OLED_ShowString(4, 1, "                ");
			OLED_ShowString(4, 1, Serial_RxPacket);				//OLED清除指定位置，并显示接收到的数据包
            if(strncmp(Serial_RxPacket, "pwmon", 5) == 0)			//如果收到LED_ON指令
            {
                TIM_CtrlPWMOutputs(TIM1, ENABLE);
            }
            if(strncmp(Serial_RxPacket, "pwmoff", 6) == 0)			//如果收到LED_ON指令
            {
                TIM_CtrlPWMOutputs(TIM1, DISABLE);
            }
            if (StringStartWith(Serial_RxPacket, "kp"))			//如果收到LED_ON指令
			{
                OLED_ShowNum(1, 1, 1234567890, 10);
                AnglePID.Kp = atof(Serial_RxPacket + 2);
			}
            if (StringStartWith(Serial_RxPacket, "ki"))			//如果收到LED_ON指令
			{
                OLED_ShowNum(1, 1, 1234567890, 10);
                AnglePID.Ki = atof(Serial_RxPacket + 2);
			}
            if (StringStartWith(Serial_RxPacket, "kd"))			//如果收到LED_ON指令
			{
                OLED_ShowNum(1, 1, 1234567890, 10);
                AnglePID.Kd = atof(Serial_RxPacket + 2);
			}
            OLED_ShowString(3, 1, Serial_RxPacket+2);
            AngleOffset = atof(Serial_RxPacket);
        }
        
        OLED_ShowNum(1, 1, 1234567890, 10);
        
        // Delay_ms(1);
        // if (Angle > -100 && Angle < 100) {
        //     PWM_SetMotor(1, Angle);
        //     PWM_SetMotor(2, Angle);
        //     // Delay_ms(1000);
        // }
    }
}
void TIM2_IRQHandler(void)
{
    static uint8_t pid_count;
    static float Alpha = 0.995;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) // Check if the update interrupt flag is set
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // Clear the update interrupt flag
        if (pid_count++ >= 5) {
            pid_count = 0;
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            GX += 24;
            AngleAcc = atan2(AY, AZ) / M_PI * 180;
            AngleAcc += AngleOffset;
            AngleGyro = Angle + GX / 32768.0 * 2000 * 0.005;
            Angle     = Alpha * AngleGyro + (1 - Alpha) * AngleAcc;
            if (Angle > 55 || Angle < -55) {
                RunFlag = 0;
            } else if (Angle <= 45 && Angle >= -45) {
                RunFlag = 1;
            }
            if (RunFlag == 1) {
                AnglePID.Actual = Angle;
                PID_Update(&AnglePID);
                AveragePWM = AnglePID.out;
                LeftPWM    = AveragePWM + DiffPWM / 2.0;
                RightPWM   = AveragePWM - DiffPWM / 2.0;

                //PWM_SetMotor(1, -LeftPWM);
                //PWM_SetMotor(2, RightPWM);
                //Serial_Printf("%f,%f,%f\r\n", AnglePID.Kp, AnglePID.Ki, AnglePID.Kd);
                // Serial_Printf("%f,%f,%f\r\n", AnglePID.out, AnglePID.Error0, Angle);
            } else {
                AnglePID.ErrorInt = 0;
                GPIO_ResetBits(GPIOB,  GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
            }
        }

        // Your code here to handle the timer interrupt
    }
}