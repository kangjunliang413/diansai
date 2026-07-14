#include "motor.h"
#include "delay.h"
#include "huidu.h"
#include "jy901s.h"
#include <math.h>

#define MOTOR_TURN_SPEED_MM_S        40.0f
#define MOTOR_TURN_SLOW_SPEED_MM_S   15.0f
#define MOTOR_TURN_SLOW_WINDOW_X10   100L /* Distance to target at which to slow down: 10.0 deg */
#define MOTOR_TURN_STOP_MARGIN_X10   15L  /* Low-speed stop lead: 1.0 deg */
#define MOTOR_TURN_TIMEOUT_MS        3000U
#define MOTOR_TURN_LOOP_MS           10U
#define MOTOR_STRAIGHT_CHECK_MS       10U
#define MOTOR_STRAIGHT_LINE_ARM_MS  1000U
#define MOTOR_RAMP_STEP_MS             50U
#define MOTOR_RAMP_START_SPEED_MM_S    30.0f

// ===================== 全局变量定义 =====================

// 左右轮PI控制器实例化
Motor_PI_TypeDef Motor_Left = {
    .target_speed = 0.0f,
    .current_speed = 0.0f,
    .kp = 5.0f,              // 比例系数（需根据实际调试）
    .ki = 1.80f,              // 积分系数（需根据实际调试）
    .error = 0.0f,
    .integral = 0.0f,
    .integral_max = 2000.0f, // 积分上限（防止积分饱和）
    .integral_min = -2000.0f,// 积分下限
    .pwm_output = 0
};

Motor_PI_TypeDef Motor_Right = {
    .target_speed = 0.0f,
    .current_speed = 0.0f,
    .kp = 5.0f,              // 比例系数（需根据实际调试）
    .ki = 1.81f,              // 积分系数（需根据实际调试）
    .error = 0.0f,
    .integral = 0.0f,
    .integral_max = 2000.0f, // 积分上限（防止积分饱和）
    .integral_min = -2000.0f,// 积分下限
    .pwm_output = 0
};

// 编码器计数器
volatile int32_t encoder_counter_left = 0;
volatile int32_t encoder_counter_right = 0;



// ===================== 函数实现 =====================

static void motor_stop_all(void)
{
    __disable_irq();
    Motor_Left.target_speed = 0.0f;
    Motor_Right.target_speed = 0.0f;
    Motor_Left.error = 0.0f;
    Motor_Right.error = 0.0f;
    Motor_Left.integral = 0.0f;
    Motor_Right.integral = 0.0f;
    Motor_Left.pwm_output = 0;
    Motor_Right.pwm_output = 0;
    __enable_irq();

    motor_set_direction(1, 0);
    motor_set_direction(2, 0);
    motor_set_duty(1, 0);
    motor_set_duty(2, 0);
}

/**
 * @brief 电机初始化
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void motor_init(uint8_t motor_id)
{
    if(motor_id == 1) {
        // 左轮初始化
        // 使能STBY引脚（高电平使能TB6612）
        DL_GPIO_setPins(DC_MOTOR_LEFT_STBY_PORT, DC_MOTOR_LEFT_STBY_PIN);

        // 初始化方向控制引脚（默认停止）
        DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);

        // 启动PWMA定时器
        DL_Timer_startCounter(PWMA_INST);
        DL_Timer_setCaptureCompareValue(PWMA_INST, 0, GPIO_PWMA_C0_IDX);

    } else if(motor_id == 2) {
        // 右轮初始化
        // 注意：右轮与左轮共用STBY引脚
        DL_GPIO_setPins(DC_MOTOR_LEFT_STBY_PORT, DC_MOTOR_LEFT_STBY_PIN);

        // 初始化方向控制引脚（默认停止）
        DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN3_PIN);
        DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN4_PIN);

        // 启动PWMB定时器
        DL_Timer_startCounter(PWMB_INST);
        DL_Timer_setCaptureCompareValue(PWMB_INST, 0, GPIO_PWMB_C0_IDX);
    }
}

/**
 * @brief 设置电机PWM占空比
 * @param motor_id 电机ID：1=左轮，2=右轮
 * @param duty PWM占空比（0-4000）
 */
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    // PWM限幅
    if(duty > PWM_MAX_DUTY) {
        duty = PWM_MAX_DUTY;
    }

    if(motor_id == 1) {
        DL_Timer_setCaptureCompareValue(PWMA_INST, duty, GPIO_PWMA_C0_IDX);
    } else if(motor_id == 2) {
        DL_Timer_setCaptureCompareValue(PWMB_INST, duty, GPIO_PWMB_C0_IDX);
    }
}

/**
 * @brief 设置电机方向
 * @param motor_id 电机ID：1=左轮，2=右轮
 * @param direction 方向：0=停止，1=正转，2=反转
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1) {
        // 左轮方向控制
        if(direction == 0) {
            // 停止：AIN1=0, AIN2=0（或都为1也可以刹车）
            DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);
        } else if(direction == 1) {
            // 正转：AIN1=1, AIN2=0
            DL_GPIO_setPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);
        } else if(direction == 2) {
            // 反转：AIN1=0, AIN2=1
            DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);
        }
    } else if(motor_id == 2) {
        // 右轮方向控制
        if(direction == 0) {
            // 停止：AIN3=0, AIN4=0
            DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN3_PIN);
            DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN4_PIN);
        } else if(direction == 1) {
            // 正转：AIN3=1, AIN4=0
            DL_GPIO_setPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN3_PIN);
            DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN4_PIN);
        } else if(direction == 2) {
            // 反转：AIN3=0, AIN4=1
            DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN3_PIN);
            DL_GPIO_setPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN4_PIN);
        }
    }
}

/**
 * @brief 计算电机速度（从编码器脉冲转换为物理速度）
 *
 * 测速换算公式推导：
 * 1. 采样周期：50ms = 0.05s
 * 2. 编码器每转脉冲数：13线 × 28减速比 = 364（单边沿计数）
 * 3. 车轮周长：π × 65mm ≈ 204.2mm
 * 4. 转速(转/s) = 脉冲增量 / 364 / 0.05s = 脉冲增量 / 18.2
 * 5. 线速度(mm/s) = 转速 × 周长 = (脉冲增量 / 18.2) × 204.2 = 脉冲增量 × 11.2
 *
 * 简化公式：speed_mm_s = pulse_count * (WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR / PID_SAMPLE_TIME_S)
 *
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        // 左轮速度计算（中断方式）
        Motor_Left.current_speed = -(float)encoder_counter_left * WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR / PID_SAMPLE_TIME_S;
        
        // 清零编码器计数器
        encoder_counter_left = 0;

    } else if (motor_id == 2) {
        // 右轮速度计算（彻底改为纯中断方式，删除原来的轮询代码）
        
        // 直接使用 GROUP1_IRQHandler 中断里累加好的脉冲数来计算速度
        Motor_Right.current_speed = (float)encoder_counter_right * WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR / PID_SAMPLE_TIME_S;

        // 清零编码器计数器（为下一次 50ms 采样做准备）
        encoder_counter_right = 0;
    }
}

/**
 * @brief PI控制算法（位置式）
 *
 * 位置式PI公式：
 * output(k) = Kp × error(k) + Ki × Σerror(k)
 *
 * 其中：
 * - error(k) = target - current
 * - Σerror(k) = integral（积分累计）
 *
 * 关键特性：
 * 1. 积分限幅：防止积分饱和导致系统失控
 * 2. PWM输出限幅：确保输出在有效范围内
 *
 * @param motor 电机PI结构体指针
 * @return 计算出的PWM占空比（带符号，正数=正转，负数=反转）
 */
int32_t motor_pi_control(Motor_PI_TypeDef *motor)
{
    // 计算误差
    motor->error = motor->target_speed - motor->current_speed;

    // 累加积分
    motor->integral += motor->error;

    // ★★★ 积分限幅（防止积分饱和）★★★
    if (motor->integral > motor->integral_max) {
        motor->integral = motor->integral_max;
    } else if (motor->integral < motor->integral_min) {
        motor->integral = motor->integral_min;
    }

    // 位置式PI计算
    float pi_output = motor->kp * motor->error + motor->ki * motor->integral;

    // 转换为整型PWM值
    int32_t pwm_value = (int32_t)pi_output;

    // ★★★ PWM输出限幅（防止超出定时器重载值）★★★
    if (pwm_value > (int32_t)PWM_MAX_DUTY) {
        pwm_value = (int32_t)PWM_MAX_DUTY;
    } else if (pwm_value < -(int32_t)PWM_MAX_DUTY) {
        pwm_value = -(int32_t)PWM_MAX_DUTY;
    }

    motor->pwm_output = pwm_value;

    return pwm_value;
}

/**
 * @brief 执行电机PI闭环控制
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void motor_pi_loop(uint8_t motor_id)
{
    int32_t pwm_output = 0;
    uint8_t direction = 0;
    uint32_t duty = 0;

    if (motor_id == 1) {
        // 左轮PI控制
        if (Motor_Left.target_speed == 0.0f) {
            Motor_Left.error = 0.0f;
            Motor_Left.integral = 0.0f;
            Motor_Left.pwm_output = 0;
            motor_set_direction(1, 0);
            motor_set_duty(1, 0);
            return;
        }

        pwm_output = motor_pi_control(&Motor_Left);

        // 根据PWM输出的正负确定方向和占空比
        if (pwm_output > 0) {
            direction = 1;  // 正转
            duty = (uint32_t)pwm_output;
        } else if (pwm_output < 0) {
            direction = 2;  // 反转
            duty = (uint32_t)(-pwm_output);
        } else {
            direction = 0;  // 停止
            duty = 0;
        }

        motor_set_direction(1, direction);
        motor_set_duty(1, duty);

    } else if (motor_id == 2) {
        // 右轮PI控制
        if (Motor_Right.target_speed == 0.0f) {
            Motor_Right.error = 0.0f;
            Motor_Right.integral = 0.0f;
            Motor_Right.pwm_output = 0;
            motor_set_direction(2, 0);
            motor_set_duty(2, 0);
            return;
        }

        pwm_output = motor_pi_control(&Motor_Right);

        // 根据PWM输出的正负确定方向和占空比
        if (pwm_output > 0) {
            direction = 1;  // 正转
            duty = (uint32_t)pwm_output;
        } else if (pwm_output < 0) {
            direction = 2;  // 反转
            duty = (uint32_t)(-pwm_output);
        } else {
            direction = 0;  // 停止
            duty = 0;
        }

        motor_set_direction(2, direction);
        motor_set_duty(2, duty);
    }
}

void motor_accelerate_straight(float target_speed_mm_s, uint16_t ramp_time_ms)
{
    float start_speed_mm_s;
    float speed_step_mm_s;
    uint16_t step_count;
    uint16_t step_index;

    /* 目标为零时直接停车，避免带着上次的积分状态重新起步。 */
    if (target_speed_mm_s == 0.0f) {
        motor_stop_all();
        return;
    }

    /*
     * 斜坡按 50 ms（与 PID 采样周期一致）更新。
     * 500 ms 对应 10 级，300 mm/s 时从 30 mm/s 起步、每级约增加 27 mm/s。
     */
    step_count = ramp_time_ms / MOTOR_RAMP_STEP_MS;
    if (step_count == 0U) {
        step_count = 1U;
    }

    start_speed_mm_s = (target_speed_mm_s > 0.0f) ?
                           MOTOR_RAMP_START_SPEED_MM_S :
                           -MOTOR_RAMP_START_SPEED_MM_S;

    /* 低于起步速度时无需做反向斜坡，直接给目标速度即可。 */
    if (((target_speed_mm_s > 0.0f) &&
         (target_speed_mm_s <= start_speed_mm_s)) ||
        ((target_speed_mm_s < 0.0f) &&
         (target_speed_mm_s >= start_speed_mm_s))) {
        __disable_irq();
        Motor_Left.error = 0.0f;
        Motor_Right.error = 0.0f;
        Motor_Left.integral = 0.0f;
        Motor_Right.integral = 0.0f;
        Motor_Left.pwm_output = 0;
        Motor_Right.pwm_output = 0;
        Motor_Left.target_speed = target_speed_mm_s;
        Motor_Right.target_speed = target_speed_mm_s;
        __enable_irq();
        return;
    }

    speed_step_mm_s = (target_speed_mm_s - start_speed_mm_s) / (float)step_count;

    /* 从静止起步时清除积分，防止上一次运行的积分项造成 PWM 突跳。 */
    __disable_irq();
    Motor_Left.error = 0.0f;
    Motor_Right.error = 0.0f;
    Motor_Left.integral = 0.0f;
    Motor_Right.integral = 0.0f;
    Motor_Left.pwm_output = 0;
    Motor_Right.pwm_output = 0;
    Motor_Left.target_speed = start_speed_mm_s;
    Motor_Right.target_speed = start_speed_mm_s;
    __enable_irq();

    for (step_index = 1U; step_index <= step_count; step_index++) {
        delay_ms(MOTOR_RAMP_STEP_MS);

        __disable_irq();
        Motor_Left.target_speed = start_speed_mm_s + speed_step_mm_s * (float)step_index;
        Motor_Right.target_speed = start_speed_mm_s + speed_step_mm_s * (float)step_index;
        __enable_irq();
    }

    /* 避免浮点计算误差，最终目标值精确落在调用方指定的速度。 */
    __disable_irq();
    Motor_Left.target_speed = target_speed_mm_s;
    Motor_Right.target_speed = target_speed_mm_s;
    __enable_irq();
}

void motor_drive_straight(uint16_t duration_ms, float speed_mm_s)
{
    uint16_t elapsed_ms = 0U;

    if ((duration_ms == 0U) || (speed_mm_s <= 0.0f)) {
        motor_stop_all();
        return;
    }

    __disable_irq();
    Motor_Left.error = 0.0f;
    Motor_Right.error = 0.0f;
    Motor_Left.integral = 0.0f;
    Motor_Right.integral = 0.0f;
    Motor_Left.pwm_output = 0;
    Motor_Right.pwm_output = 0;
    Motor_Left.target_speed = speed_mm_s;
    Motor_Right.target_speed = speed_mm_s;
    __enable_irq();

    while (elapsed_ms < duration_ms) {
        uint16_t wait_ms = MOTOR_STRAIGHT_CHECK_MS;

        /* 起步满 1 秒后，任一灰度通道检测到黑线即结束直线段。 */
        if ((elapsed_ms >= MOTOR_STRAIGHT_LINE_ARM_MS) &&
            (Huidu_Read_Raw() != 0x00U)) {
            break;
        }

        if ((duration_ms - elapsed_ms) < wait_ms) {
            wait_ms = duration_ms - elapsed_ms;
        }

        delay_ms(wait_ms);
        elapsed_ms += wait_ms;
    }

    motor_stop_all();
}

/**
 * @brief 原地按角度转向
 * @param angle_deg 目标角度，正数=左转，负数=右转，单位：度
 *
 * 使用JY901S的Z轴陀螺仪积分角度作为相对转向角。
 * 已实测：小车左转时Z轴相对角度增加。
 */
void motor_turn_angle(int16_t angle_deg)
{
    uint16_t elapsed_ms = 0U;
    uint8_t slow_speed_applied = 0U;
    int32_t target_x10;
    int32_t stop_x10;

    if (angle_deg == 0) {
        motor_stop_all();
        return;
    }

    if (angle_deg > 0) {
        target_x10 = (int32_t)angle_deg * 10L;
    } else {
        target_x10 = -(int32_t)angle_deg * 10L;
    }

    if (target_x10 > MOTOR_TURN_STOP_MARGIN_X10) {
        stop_x10 = target_x10 - MOTOR_TURN_STOP_MARGIN_X10;
    } else {
        stop_x10 = target_x10;
    }

    motor_stop_all();
    delay_ms(50);

    __disable_irq();
    Motor_Left.error = 0.0f;
    Motor_Right.error = 0.0f;
    Motor_Left.integral = 0.0f;
    Motor_Right.integral = 0.0f;
    Motor_Left.pwm_output = 0;
    Motor_Right.pwm_output = 0;

    if (angle_deg > 0) {
        Motor_Left.target_speed = -MOTOR_TURN_SPEED_MM_S;
        Motor_Right.target_speed = MOTOR_TURN_SPEED_MM_S;
    } else {
        Motor_Left.target_speed = MOTOR_TURN_SPEED_MM_S;
        Motor_Right.target_speed = -MOTOR_TURN_SPEED_MM_S;
    }
    __enable_irq();

    while (elapsed_ms < MOTOR_TURN_TIMEOUT_MS) {
        int32_t angle_x10 = JY901S_GetGyroZTurnAngleX10();
        int32_t turned_x10 = (angle_deg > 0) ? angle_x10 : -angle_x10;

        /*
         * Slow down before the target so that gyro sample latency and motor
         * coast do not produce a target-angle-dependent overshoot.
         */
        if ((slow_speed_applied == 0U) &&
            (turned_x10 >= (target_x10 - MOTOR_TURN_SLOW_WINDOW_X10))) {
            __disable_irq();
            if (angle_deg > 0) {
                Motor_Left.target_speed = -MOTOR_TURN_SLOW_SPEED_MM_S;
                Motor_Right.target_speed = MOTOR_TURN_SLOW_SPEED_MM_S;
            } else {
                Motor_Left.target_speed = MOTOR_TURN_SLOW_SPEED_MM_S;
                Motor_Right.target_speed = -MOTOR_TURN_SLOW_SPEED_MM_S;
            }
            __enable_irq();
            slow_speed_applied = 1U;
        }

        if (turned_x10 >= stop_x10) {
            break;
        }

        delay_ms(MOTOR_TURN_LOOP_MS);
        elapsed_ms += MOTOR_TURN_LOOP_MS;
    }

    motor_stop_all();
    delay_ms(50);
    // JY901S_ResetGyroZTurnAngle();
}

// ===================== 定时器中断服务函数（PID控制周期：50ms）=====================

/**
 * @brief PID定时器中断服务函数（TIMA0）
 *
 * 硬件映射：
 * - MOTOR_PID_INST → PID_INST → TIMA0
 * - 中断函数名必须为：TIMA0_IRQHandler（与启动文件向量表匹配）
 *
 * 触发周期：50ms（20Hz）
 *
 * 完整闭环流程：
 * 1. 读取编码器脉冲增量
 * 2. 计算实际物理速度
 * 3. 执行PI算法计算PWM
 * 4. 更新电机PWM输出
 *
 * 中断标志位清除：
 * DL_Timer_getPendingInterrupt() 会自动读取并清除中断标志位（IIDX寄存器）
 */
void TIMA0_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        // 左轮闭环控制
        calculate_speed(1);      // 计算左轮速度
        motor_pi_loop(1);        // 执行左轮PI控制

        // 右轮闭环控制
        calculate_speed(2);      // 计算右轮速度
        motor_pi_loop(2);        // 执行右轮PI控制

        break;

    default:
        break;
    }
}

// ===================== GPIO中断服务函数（编码器脉冲计数 + 按键处理）=====================

/**
 * @brief GROUP1中断服务函数（处理GPIOA和GPIOB的编码器+按键中断）
 *
 * 共享中断源：
 * - KEY_KEY1_IIDX：按键中断
 * - DC_MOTOR_LEFT_AA_IIDX：左轮编码器A相中断（GPIOA.9）
 * - DC_MOTOR_RIGHT_BA_IIDX：右轮编码器A相中断（GPIOB.7）
 *
 * 编码器计数策略：
 * 使用单边沿计数（只计A相上升沿）
 * 通过B相状态判断旋转方向：
 * - A上升沿时，B=0 → 正转，计数+1
 * - A上升沿时，B=1 → 反转，计数-1
 *
 * 中断标志位清除：
 * DL_GPIO_getPendingInterrupt() 会自动读取并清除中断标志位
 */
void GROUP1_IRQHandler(void)
{
    // 1. 读取GPIOA的中断标志位
    uint32_t gpioa_iidx = DL_GPIO_getPendingInterrupt(DC_MOTOR_LEFT_AA_PORT);

    // 2. 读取GPIOB的中断标志位
    uint32_t gpiob_iidx = DL_GPIO_getPendingInterrupt(DC_MOTOR_RIGHT_PORT);

    // 3. 处理GPIOA中断（左轮编码器 + 按键）
    switch (gpioa_iidx)
    {
    case DC_MOTOR_LEFT_AA_IIDX:
        // 左轮编码器A相中断（上升沿）
        {
            // 读取B相状态判断方向
            uint32_t pin_b = DL_GPIO_readPins(DC_MOTOR_LEFT_AB_PORT, DC_MOTOR_LEFT_AB_PIN);

            if (pin_b == 0) {
                encoder_counter_left++;  // 正转
            } else {
                encoder_counter_left--;  // 反转
            }
        }
        break;

    case KEY_KEY1_IIDX:
        // 按键中断：循环切换运行模式
        {
            extern volatile int run_mode;
            run_mode = 1;  // 循环切换 0~4 五种模式 (run_mode + 1) % 5
        }
        break;

    default:
        break;
    }

    // 4. 处理GPIOB中断（右轮编码器）
    switch (gpiob_iidx)
    {
    case DC_MOTOR_RIGHT_BA_IIDX:
        // 右轮编码器A相中断（上升沿）
        {
            // 读取B相状态判断方向
            uint32_t pin_bb = DL_GPIO_readPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_BB_PIN);

            if (pin_bb == 0) {
                encoder_counter_right++;  // 正转
            } else {
                encoder_counter_right--;  // 反转
            }
        }
        break;

    default:
        break;
    }
}
