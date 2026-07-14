#include "huidu.h"
#include "motor.h"  // 引入电机控制接口
#include "delay.h"

// ===================== 巡线控制参数 =====================

// 参考工程的循迹位置 PID：place_Kp=60, place_Ki=1.8, place_Kd=45
#define LINE_PLACE_KP           18.0f
#define LINE_PLACE_KI           0.05f
#define LINE_PLACE_KD           4.5f
#define LINE_PLACE_I_LIMIT      500.0f

// 8 路传感器位置编号为 1~8，中心点为 4.5；参考工程 12 路时中心点为 6.5
#define LINE_POSITION_CENTER    4.5f

// 默认基础速度和目标速度限幅，单位 mm/s
#define LINE_FOLLOW_DEFAULT_BASE_SPEED  200.0f
#define SPEED_MAX               500.0f
#define SPEED_MIN               0.0f

// ===================== 全局变量 =====================

static float last_line_position = LINE_POSITION_CENTER;
static float last_line_error = 0.0f;
static float previous_line_error = 0.0f;
static float line_error_sum = 0.0f;
static float last_control_output = 0.0f;

static float Huidu_Limit_Float(float value, float min_value, float max_value)
{
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

// ===================== 函数实现 =====================

/**
 * @brief 读取8路灰度传感器的原始状态
 *
 * 传感器物理布局（从左到右）：
 * L4, L3, L2, L1, R1, R2, R3, R4
 *
 * 读取步骤：
 * 1. 使用 DL_GPIO_readPins() 读取每个传感器的引脚状态
 * 2. 将读到的电平按位取反（黑线=0 → 1，白底=1 → 0）
 * 3. 拼接成8位数据返回
 *
 * @return uint8_t 8位二进制状态（1=黑线，0=白底）
 */
uint8_t Huidu_Read_Raw(void)
{
    uint8_t sensor_data = 0;

    // 读取传感器 L4（最左边）- Bit 0
    if (DL_GPIO_readPins(HUI_DU_L4_PORT, HUI_DU_L4_PIN) == 0) {
        sensor_data |= (1 << 0);  // 黑线（低电平） → 置1
    }

    // 读取传感器 L3 - Bit 1
    if (DL_GPIO_readPins(HUI_DU_L3_PORT, HUI_DU_L3_PIN) == 0) {
        sensor_data |= (1 << 1);
    }

    // 读取传感器 L2 - Bit 2
    if (DL_GPIO_readPins(HUI_DU_L2_PORT, HUI_DU_L2_PIN) == 0) {
        sensor_data |= (1 << 2);
    }

    // 读取传感器 L1 - Bit 3
    if (DL_GPIO_readPins(HUI_DU_L1_PORT, HUI_DU_L1_PIN) == 0) {
        sensor_data |= (1 << 3);
    }

    // 读取传感器 R1 - Bit 4
    if (DL_GPIO_readPins(HUI_DU_R1_PORT, HUI_DU_R1_PIN) == 0) {
        sensor_data |= (1 << 4);
    }

    // 读取传感器 R2 - Bit 5
    if (DL_GPIO_readPins(HUI_DU_R2_PORT, HUI_DU_R2_PIN) == 0) {
        sensor_data |= (1 << 5);
    }

    // 读取传感器 R3 - Bit 6
    if (DL_GPIO_readPins(HUI_DU_R3_PORT, HUI_DU_R3_PIN) == 0) {
        sensor_data |= (1 << 6);
    }

    // 读取传感器 R4（最右边）- Bit 7
    if (DL_GPIO_readPins(HUI_DU_R4_PORT, HUI_DU_R4_PIN) == 0) {
        sensor_data |= (1 << 7);
    }

    return sensor_data;
}

/**
 * @brief 计算当前小车相对于黑线的位置误差（参考 xunji_shua_xin 加权平均法）
 */
float Huidu_Get_Error(void)
{
    uint8_t sensor_data = Huidu_Read_Raw();
    float weighted_sum = 0.0f;
    float active_count = 0.0f;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if ((sensor_data & (1U << i)) != 0U) {
            weighted_sum += (float)(i + 1U);
            active_count += 1.0f;
        }
    }

    if (active_count > 0.0f) {
        last_line_position = weighted_sum / active_count;
    }

    last_line_error = LINE_POSITION_CENTER - last_line_position;
    return last_line_error;
}

/**
 * @brief 获取上一次有效的误差值（用于调试）
 *
 * @return float 上一次计算出的有效误差值
 */
float Huidu_Get_Last_Error(void)
{
    return last_line_error;
}

float Huidu_Get_Position(void)
{
    return last_line_position;
}

float Huidu_Get_Control_Output(void)
{
    return last_control_output;
}

/**
 * @brief 判断循迹线路是否结束
 *
 * @return uint8_t 1=所有传感器均为白色，循迹结束；0=仍检测到黑线
 */
uint8_t Huidu_Is_LineEnd(void)
{
    uint8_t sensor_data = Huidu_Read_Raw();
    return (sensor_data == 0x00) ? 1 : 0;
}

/**
 * @brief 判断左侧终止标记
 *
 * L4 + L3（Bit 0、1）或 L3 + L2（Bit 1、2）同时检测到黑线时，
 * 认为已经到达循迹终点。
 */
static uint8_t Huidu_Is_LeftEndMarker(uint8_t sensor_data)
{
    const uint8_t left_outer_pair_mask = 0x03U;  // L4 + L3
    const uint8_t left_inner_pair_mask = 0x06U;  // L3 + L2

    return (((sensor_data & left_outer_pair_mask) == left_outer_pair_mask) ||
            ((sensor_data & left_inner_pair_mask) == left_inner_pair_mask)) ? 1U : 0U;
}

// ===================== 自动巡线控制函数 =====================

/**
 * @brief 自动巡线控制任务（参考 place_PID_value 位置式 PID）
 *
 * 控制流程：
 * 1. 读取灰度传感器，计算位置误差
 * 2. 使用位置式PID算法计算转向控制量
 * 3. 将转向控制量叠加到基础速度上
 * 4. 更新左右轮目标速度，底层PI控制器会自动追踪
 *
 * 参考工程公式：
 * place_err = center - xun
 * control_output = Kp × place_err + Ki × Σplace_err + Kd × Δplace_err
 *
 * 差速控制策略：
 * - 误差为正（黑线在左）：左轮减速，右轮加速 → 左转
 * - 误差为负（黑线在右）：左轮加速，右轮减速 → 右转
 * - 误差为0（居中）：左右轮同速 → 直行
 *
 * 使用方法：
 * 在主循环中以一定周期（如50ms）调用此函数
 */
static void Huidu_LineFollow_Task_WithSpeed(float base_speed)
{
    uint8_t sensor_data = Huidu_Read_Raw();
    float error;
    float derivative;
    float control_output;
    float left_speed;
    float right_speed;

    if (sensor_data == 0x00U) {
        Huidu_LineFollow_Stop();
        return;
    }

    error = Huidu_Get_Error();
    derivative = error - previous_line_error;

    line_error_sum += error;
    line_error_sum = Huidu_Limit_Float(line_error_sum, -LINE_PLACE_I_LIMIT, LINE_PLACE_I_LIMIT);

    control_output = LINE_PLACE_KP * error + LINE_PLACE_KI * line_error_sum + LINE_PLACE_KD * derivative;
    previous_line_error = error;
    last_control_output = control_output;

    // 参考工程：左轮=基础-修正，右轮=基础+修正
    left_speed  = base_speed - control_output;
    right_speed = base_speed + control_output;

    left_speed = Huidu_Limit_Float(left_speed, SPEED_MIN, SPEED_MAX);
    right_speed = Huidu_Limit_Float(right_speed, SPEED_MIN, SPEED_MAX);

    Motor_Left.target_speed = left_speed;
    Motor_Right.target_speed = right_speed;
}

void Huidu_LineFollow_Task(void)
{
    Huidu_LineFollow_Task_WithSpeed(LINE_FOLLOW_DEFAULT_BASE_SPEED);
}

void Huidu_LineFollow(uint16_t duration_ms, float speed_mm_s)
{
    uint16_t elapsed_ms = 0U;
    const uint16_t control_period_ms = 20U;
    float base_speed;

    if (speed_mm_s <= 0.0f) {
        Huidu_LineFollow_Stop();
        return;
    }

    base_speed = Huidu_Limit_Float(speed_mm_s, SPEED_MIN, SPEED_MAX);

    while ((duration_ms == 0U) || (elapsed_ms < duration_ms)) {
        uint8_t sensor_data = Huidu_Read_Raw();

        /*
         * 左侧终止标记优先于普通丢线判断，且不会进入本周期的循迹控制。
         * 这是连续动作的交接点：保留当前速度并直接返回，由调用方接管。
         */
        if (Huidu_Is_LeftEndMarker(sensor_data) != 0U) {
            return;
        }

        if (sensor_data == 0x00U) {
            break;
        }

        Huidu_LineFollow_Task_WithSpeed(base_speed);
        delay_ms(control_period_ms);
        elapsed_ms += control_period_ms;
    }

    Huidu_LineFollow_Stop();
}

/**
 * @brief 停止巡线（停止电机）
 */
void Huidu_LineFollow_Stop(void)
{
    Motor_Left.target_speed = 0.0f;
    Motor_Right.target_speed = 0.0f;
    Motor_Left.integral = 0.0f;
    Motor_Right.integral = 0.0f;
    Motor_Left.pwm_output = 0;
    Motor_Right.pwm_output = 0;

    motor_set_duty(1, 0);
    motor_set_duty(2, 0);
    motor_set_direction(1, 0);
    motor_set_direction(2, 0);

    line_error_sum = 0.0f;
    last_line_error = 0.0f;
    previous_line_error = 0.0f;
    last_line_position = LINE_POSITION_CENTER;
    last_control_output = 0.0f;
}
