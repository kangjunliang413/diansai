/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "uart.h"
#include "key.h"
#include "motor.h"
#include "huidu.h"
#include "jy901s.h"

int status = 0;
int run_mode = 2;  // 运行模式：0=停止, 1=直行, 2=循迹PID测试, 3=直行+右转, 4=保留

static void FormatAngleX10(char *str, const char *label, int32_t angle_x10)
{
    char sign = ' ';

    if (angle_x10 < 0) {
        sign = '-';
        angle_x10 = -angle_x10;
    }

    sprintf(str, "%s:%c%3ld.%1ld deg", label, sign,
            (long)(angle_x10 / 10L), (long)(angle_x10 % 10L));
}

int main(void)
{
    SYSCFG_DL_init();
    JY901S_Init();

    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
    OLED_Clear();
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);
    OLED_ShowString(0, 0, (u8 *)"JY901S calibrate", 16);
    OLED_ShowString(0, 16, (u8 *)"Keep car still ", 16);
    OLED_Refresh();
    if (JY901S_CalibrateGyroZ(40U, 3000U) == 0U) {
        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"GZ cal timeout ", 16);
        OLED_ShowString(0, 16, (u8 *)"Check UART data ", 16);
        OLED_Refresh();
        delay_ms(1000);
    }
    JY901S_ResetGyroZTurnAngle();

    // 使能GPIOA中断（按键KEY1和左电机编码器AA共享此中断）
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);  // 等价于 GPIOA_INT_IRQn
    NVIC_EnableIRQ(KEY_INT_IRQN);
    DL_Timer_setCaptureCompareValue(SERVO_INST, 50, GPIO_SERVO_C1_IDX);
    DL_Timer_startCounter(SERVO_INST);

    // ===== 双轮电机初始化 =====
    motor_init(1);  // 初始化左轮
    motor_init(2);  // 初始化右轮

    // 启动PID定时器（50ms周期，自动执行PI闭环控制）
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    Motor_Left.integral = 0.0f;
    Motor_Left.error = 0.0f;
    Motor_Left.pwm_output = 0;
    Motor_Right.integral = 0.0f;
    Motor_Right.error = 0.0f;
    Motor_Right.pwm_output = 0;
    while (1) {
        // ========== 主循环：根据模式执行不同的控制逻辑 ==========
        switch(run_mode) {
            case 1:  // 模式1：直行测试
                motor_set_direction(1, 1);
                motor_set_direction(2, 1);
                delay_ms(3000);  // 直行 3 秒
                // 停止
                motor_set_direction(1, 0);
                motor_set_direction(2, 0);
                delay_ms(1000);
                break;

            case 2:  // 模式2：循迹PID测试
            {
                char oled_str[24];
                JY901S_Angle_t angle;
                int32_t gz_turn_angle_x10;
                uint8_t i;

                motor_set_direction(1, 0);
                motor_set_direction(2, 0);
                __disable_irq();
                Motor_Left.target_speed = 0.0f;
                Motor_Right.target_speed = 0.0f;
                Motor_Left.integral = 0.0f;
                Motor_Right.integral = 0.0f;
                __enable_irq();

                OLED_Clear();
                for (i = 0; i < 10; i++) {
                    JY901S_GetAngle(&angle);
                    gz_turn_angle_x10 = JY901S_GetGyroZTurnAngleX10();

                    OLED_ShowString(0, 0, (u8 *)"JY901S Z yaw    ", 16);

                    FormatAngleX10(oled_str, "ZA", angle.yaw_x10);
                    OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                    FormatAngleX10(oled_str, "ZR", gz_turn_angle_x10);
                    OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                    sprintf(oled_str, "F:%02X N%04lu A%04lu",
                            JY901S_GetLastFrameType(),
                            (unsigned long)(JY901S_GetGyroFrameCount() % 10000UL),
                            (unsigned long)(JY901S_GetAngleFrameCount() % 10000UL));
                    OLED_ShowString(0, 48, (u8 *)oled_str, 16);

                    OLED_Refresh();
                    delay_ms(50);
                }
                break;
            }

            case 3:  // 模式3：直行 + 右转
                // 直行
                motor_set_direction(1, 1);
                motor_set_direction(2, 1);
                delay_ms(2000);
                // 右转（右轮停，左轮前进）
                motor_set_direction(1, 1);
                motor_set_direction(2, 0);
                delay_ms(1000);
                // 停止
                motor_set_direction(1, 0);
                motor_set_direction(2, 0);
                delay_ms(1000);
                break;

            case 4:  // 模式4：暂待使用，保持静止
                motor_set_direction(1, 0);
                motor_set_direction(2, 0);
                delay_ms(1000);
                break;

            default:
                // 异常情况，停止
                motor_set_direction(1, 0);
                motor_set_direction(2, 0);
                delay_ms(1000);
                break;
        }
    }
        
        
        
        // // 通知ADC开始采样
        // DL_ADC12_startConversion(xuanniu_INST);

        // //等Adc采样完
        // delay_ms(10);

        // // 获取ADC采样结果
        // uint16_t adc_result = DL_ADC12_getMemResult(xuanniu_INST, xuanniu_ADCMEM_0);
        // float_t adc_value = adc_result * xuanniu_ADCMEM_0_REF_VOLTAGE_V / 4096.0; // Assuming 12-bit ADC resolution
        
        // char oled_str[50];
        // sprintf(oled_str, "ADC: %.2f V", adc_value);
        // OLED_ShowString(0, 32, (u8 *)oled_str, 16);
        // OLED_Refresh();
        

        // if(status == 0){
        //     OLED_Clear();
        //     OLED_ShowString(0, 0, (u8 *)"status: 0", 16);
        //     OLED_Refresh();
        // } 
        // else if(status == 1){
        //     OLED_Clear();
        //     OLED_ShowString(0, 0, (u8 *)"status: 1", 16);
        //     OLED_Refresh();
        // }
        // else if(status == 2){
        //     OLED_Clear();
        //     OLED_ShowString(0, 0, (u8 *)"status: 2", 16);
        //     OLED_Refresh();
        // }
        

        // Toggle the LED every 500 ms
        // char oled_str[50];
        // int int_a = 20;
        // sprintf(oled_str, "Integer: %d", int_a);
        // OLED_ShowString(0, 46, (u8 *)oled_str, 16);
        // OLED_Refresh();
        

        // OLED_ShowString(0, 0, (u8 *)"Hello, TI!", 16);
        // OLED_Refresh();
        // delay_ms(500);
        // DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
        // DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN);
        // delay_ms(500);
        // DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
        // DL_GPIO_setPins(LED_PORT, LED_LED1_PIN);
        // UART_send_string(PRINT_INST, "hello, ti!\n");
    }
