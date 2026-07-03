#include "huidu.h"

// ===================== 全局变量 =====================

// 记录上一次有效的误差值（用于丢线时保持记忆）
static float last_valid_error = 0.0f;

// ===================== 函数实现 =====================

/**
 * @brief 读取8路灰度传感器的原始状态
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

    // ⚠️ 注意：以下引脚宏需要根据 SysConfig 实际生成的名称替换
    // 示例：如果 SysConfig 中命名为 GRAY_SENSOR_PORT 和 GRAY_SENSOR_PIN_0，
    // 则将 HUIDU_PORT 替换为 GRAY_SENSOR_PORT，HUIDU_PIN_0 替换为 GRAY_SENSOR_PIN_0

    // 读取传感器0（最左边）- Bit 0
    if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_0) == 0) {
        sensor_data |= (1 << 0);  // 黑线（低电平） → 置1
    }

    // 读取传感器1 - Bit 1
    if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_1) == 0) {
        sensor_data |= (1 << 1);
    }

    // 读取传感器2 - Bit 2
    if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_2) == 0) {
        sensor_data |= (1 << 2);
    }

    // 读取传感器3 - Bit 3
    if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_3) == 0) {
        sensor_data |= (1 << 3);
    }

    // 读取传感器4 - Bit 4
    if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_4) == 0) {
        sensor_data |= (1 << 4);
    }

    // 读取传感器5 - Bit 5
    if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_5) == 0) {
        sensor_data |= (1 << 5);
    }

    // 读取传感器6 - Bit 6
    if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_6) == 0) {
        sensor_data |= (1 << 6);
    }

    // 读取传感器7（最右边）- Bit 7
    if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_7) == 0) {
        sensor_data |= (1 << 7);
    }

    return sensor_data;
}

/**
 * @brief 计算当前小车相对于黑线的位置误差
 *
 * 加权平均法原理：
 * 1. 为每个传感器分配位置权重（从左到右：-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5）
 * 2. 误差 = Σ(传感器状态 × 权重) / Σ(传感器状态)
 * 3. 如果所有传感器都是0（全白，丢线），则返回上一次的误差值
 *
 * 权重设计说明：
 * - 权重范围 -3.5 ~ +3.5，均匀分布
 * - 负权重在左边：传感器检测到黑线 → 黑线在左侧 → 小车偏右 → 误差为负
 * - 正权重在右边：传感器检测到黑线 → 黑线在右侧 → 小车偏左 → 误差为正
 *
 * 示例计算：
 * - 传感器状态：0b00011000（传感器3和4检测到黑线，居中）
 *   加权和 = 1×(-0.5) + 1×(0.5) = 0
 *   传感器和 = 1 + 1 = 2
 *   误差 = 0 / 2 = 0.0（完美居中）
 *
 * - 传感器状态：0b01100000（传感器5和6检测到黑线，偏右）
 *   加权和 = 1×(1.5) + 1×(2.5) = 4.0
 *   传感器和 = 1 + 1 = 2
 *   误差 = 4.0 / 2 = 2.0（小车偏左，需要右转）
 *
 * @return float 位置误差值
 */
float Huidu_Get_Error(void)
{
    // 读取传感器原始状态
    uint8_t sensor_data = Huidu_Read_Raw();

    // 传感器位置权重数组（从左到右）
    // 权重间隔为1.0，范围从 -3.5 到 +3.5
    const float weights[8] = {
        -3.5f,  // 传感器0（最左边）
        -2.5f,  // 传感器1
        -1.5f,  // 传感器2
        -0.5f,  // 传感器3
         0.5f,  // 传感器4
         1.5f,  // 传感器5
         2.5f,  // 传感器6
         3.5f   // 传感器7（最右边）
    };

    // 加权和与传感器状态和
    float weighted_sum = 0.0f;
    uint8_t sensor_sum = 0;

    // 遍历8个传感器，计算加权平均
    for (uint8_t i = 0; i < 8; i++) {
        if (sensor_data & (1 << i)) {  // 如果第i个传感器检测到黑线
            weighted_sum += weights[i];
            sensor_sum++;
        }
    }

    // 判断是否丢线
    if (sensor_sum == 0) {
        // 所有传感器都是白色，完全丢线
        // 保持上一次的误差值（记忆功能）
        return last_valid_error;
    }

    // 计算误差
    float error = weighted_sum / (float)sensor_sum;

    // 更新上一次有效误差
    last_valid_error = error;

    return error;
}

/**
 * @brief 获取上一次有效的误差值（用于调试）
 *
 * @return float 上一次计算出的有效误差值
 */
float Huidu_Get_Last_Error(void)
{
    return last_valid_error;
}

/**
 * @brief 判断是否完全丢线
 *
 * @return uint8_t 1=丢线（所有传感器都是白色），0=在线上
 */
uint8_t Huidu_Is_Lost(void)
{
    uint8_t sensor_data = Huidu_Read_Raw();
    return (sensor_data == 0x00) ? 1 : 0;
}
