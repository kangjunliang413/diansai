#ifndef HUIDU_H
#define HUIDU_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

// ===================== 灰度传感器配置说明 =====================
/*
 * 硬件连接：
 * - 8路数字灰度传感器，从左到右依次为：L4, L3, L2, L1, R1, R2, R3, R4
 * - 传感器输出特性：黑线=0（低电平），白底=1（高电平）
 * - 代码中已做按位取反处理，使得黑线=1，白底=0
 *
 * 引脚定义（已在 ti_msp_dl_config.h 中生成）：
 * - HUI_DU_L4_PORT / HUI_DU_L4_PIN：最左边传感器（Bit 0）
 * - HUI_DU_L3_PORT / HUI_DU_L3_PIN：传感器L3（Bit 1）
 * - HUI_DU_L2_PORT / HUI_DU_L2_PIN：传感器L2（Bit 2）
 * - HUI_DU_L1_PORT / HUI_DU_L1_PIN：传感器L1（Bit 3）
 * - HUI_DU_R1_PORT / HUI_DU_R1_PIN：传感器R1（Bit 4）
 * - HUI_DU_R2_PORT / HUI_DU_R2_PIN：传感器R2（Bit 5）
 * - HUI_DU_R3_PORT / HUI_DU_R3_PIN：传感器R3（Bit 6）
 * - HUI_DU_R4_PORT / HUI_DU_R4_PIN：最右边传感器（Bit 7）
 *
 * 位序定义：
 * - Bit 0（最低位）：最左边的传感器（L4）
 * - Bit 7（最高位）：最右边的传感器（R4）
 */

// ===================== 函数声明 =====================

/**
 * @brief 读取8路灰度传感器的原始状态
 *
 * @return uint8_t 8位数据，每一位代表一个传感器：
 *         - 0 = 检测到白色（背景）
 *         - 1 = 检测到黑色（目标线）
 *         - Bit 0（最低位）= 最左边传感器
 *         - Bit 7（最高位）= 最右边传感器
 *
 * 示例：
 * - 0b00011000 (0x18)：中间两个传感器检测到黑线，小车在线上
 * - 0b11111111 (0xFF)：全黑（可能在起跑线或全黑区域）
 * - 0b00000000 (0x00)：全白（完全丢线）
 */
uint8_t Huidu_Read_Raw(void);

/**
 * @brief 计算当前小车相对于黑线的位置误差
 *
 * 参考 xunji_shua_xin() 的加权平均思路：
 * - L4~R4 位置编号为 1~8
 * - 黑线位置 = Σ(传感器状态 × 位置编号) / Σ(传感器状态)
 * - 误差 = 中心位置 4.5 - 黑线位置
 *
 * @return float 位置误差值：
 *         - 0.0：小车正对黑线（居中）
 *         - 正值：黑线在左侧，需要向左修正
 *         - 负值：黑线在右侧，需要向右修正
 *         - 丢线时：返回上一次的误差值（记忆功能）
 */
float Huidu_Get_Error(void);

/**
 * @brief 获取上一次有效的误差值（用于调试）
 *
 * @return float 上一次计算出的有效误差值
 */
float Huidu_Get_Last_Error(void);

/**
 * @brief 获取当前加权平均位置，范围约为 1.0~8.0，中心为 4.5
 */
float Huidu_Get_Position(void);

/**
 * @brief 获取最近一次循迹 PID 输出的差速修正量
 */
float Huidu_Get_Control_Output(void);

/**
 * @brief 判断循迹线路是否结束
 *
 * @return uint8_t 1=所有传感器均为白色，循迹结束；0=仍检测到黑线
 */
uint8_t Huidu_Is_LineEnd(void);

/**
 * @brief 自动巡线控制任务（基于参考工程 place_PID_value 的位置式 PID）
 *
 * 功能：
 * - 读取灰度传感器误差
 * - 使用位置式PID算法计算转向控制量
 * - 更新左右轮目标速度实现差速转向
 *
 * 使用方法：
 * 在主循环中以一定周期（如10ms~50ms）调用此函数
 *
 * 注意：
 * - 调用前需确保电机PI控制已启动（TIMA0定时器中断）
 * - 调用前需确保左右轮电机已初始化
 */
void Huidu_LineFollow_Task(void);

/**
 * @brief 连续执行循迹控制
 * @param duration_ms 循迹最长时间，单位：ms；传入 0 时一直循迹到线路结束
 * @param speed_mm_s 循迹基础速度，单位：mm/s；必须大于 0，最大为 500 mm/s
 *
 * 所有灰度传感器均为白色或到达最长时间时，循迹结束并停车。L4+L3、L3+L2
 * 同时检测到黑线时，函数立即返回但保留当前电机速度，由调用方接管后续动作。
 * 左右轮速度会在基础速度上叠加 PID 修正量，并限幅至 0~500 mm/s。
 */
void Huidu_LineFollow(uint16_t duration_ms, float speed_mm_s);

/**
 * @brief 停止巡线（停止电机）
 *
 * 功能：
 * - 将左右轮目标速度设为0
 * - 重置PD控制器状态
 */
void Huidu_LineFollow_Stop(void);

#endif // HUIDU_H
