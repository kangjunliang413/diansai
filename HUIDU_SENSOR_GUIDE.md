# 8路灰度传感器驱动使用说明

**文件创建时间：** 2026-07-03  
**适用平台：** TI MSPM0G3507 + TI Driverlib  
**传感器类型：** 8路数字灰度传感器（黑线=0，白底=1）

---

## 📁 文件结构

```
user_driver/
├── huidu.h          # 灰度传感器驱动头文件
├── huidu.c          # 灰度传感器驱动实现
huidu_test_main.c    # 测试代码示例（参考用）
```

---

## 🔧 SysConfig 配置要求

### 引脚配置

在 `.syscfg` 文件中，需要配置 8 个 GPIO 引脚作为输入：

**配置参数：**
- **Mode:** `Input`
- **Internal Resistor:** `Pull Up`（建议）或 `None`
- **Interrupt:** `Disable`（灰度传感器使用轮询方式读取）

**命名规范（重要）：**

代码中使用的占位符宏需要根据 SysConfig 实际生成的名称替换：

```c
// 代码占位符           →  SysConfig 实际命名（示例）
HUIDU_PORT            →  GRAY_SENSOR_PORT
HUIDU_PIN_0           →  GRAY_SENSOR_0_PIN
HUIDU_PIN_1           →  GRAY_SENSOR_1_PIN
...
HUIDU_PIN_7           →  GRAY_SENSOR_7_PIN
```

**查找方法：**
配置完成后，打开 `Debug/ti_msp_dl_config.h`，搜索你配置的引脚名称，复制生成的宏定义。

---

## 📋 快速集成步骤

### 步骤1：配置 SysConfig

1. 打开 `.syscfg` 文件
2. 添加 8 个 GPIO 引脚（建议使用同一个 PORT，方便管理）
3. 配置为输入模式，上拉电阻
4. 保存并生成代码

### 步骤2：替换引脚宏定义

打开 `user_driver/huidu.c`，找到以下代码段：

```c
// ⚠️ 注意：以下引脚宏需要根据 SysConfig 实际生成的名称替换

// 读取传感器0（最左边）- Bit 0
if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_0) == 0) {
    sensor_data |= (1 << 0);
}
```

将 `HUIDU_PORT` 和 `HUIDU_PIN_0 ~ HUIDU_PIN_7` 替换为你的实际宏定义。

**示例：**

```c
// 替换前
if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_0) == 0) {

// 替换后（假设你的配置命名为 GRAY_SENSOR）
if (DL_GPIO_readPins(GRAY_SENSOR_PORT, GRAY_SENSOR_0_PIN) == 0) {
```

### 步骤3：添加到工程

1. 将 `huidu.c` 和 `huidu.h` 添加到你的 CCS 工程
2. 在 `main.c` 中包含头文件：
   ```c
   #include "huidu.h"
   ```

### 步骤4：编写测试代码

参考 `huidu_test_main.c` 中的示例代码，在 `main()` 函数中添加：

```c
#include "huidu.h"
#include "oled.h"
#include "delay.h"
#include <stdio.h>

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_Clear();

    while (1) {
        // 读取原始状态
        uint8_t sensor_raw = Huidu_Read_Raw();
        
        // 计算误差
        float error = Huidu_Get_Error();
        
        // 判断丢线
        uint8_t is_lost = Huidu_Is_Lost();

        // 显示在 OLED 上
        char oled_str[64];
        
        // 二进制显示
        sprintf(oled_str, "Bin:");
        for (int8_t i = 7; i >= 0; i--) {
            strcat(oled_str, (sensor_raw & (1 << i)) ? "1" : "0");
        }
        OLED_ShowString(0, 0, (u8 *)oled_str, 16);

        // 十六进制显示
        sprintf(oled_str, "Hex: 0x%02X", sensor_raw);
        OLED_ShowString(0, 16, (u8 *)oled_str, 16);

        // 误差值显示
        sprintf(oled_str, "Err:%+.1f", error);
        OLED_ShowString(0, 32, (u8 *)oled_str, 16);

        // 状态显示
        OLED_ShowString(0, 48, (u8 *)(is_lost ? "LOST" : "OK  "), 16);
        
        OLED_Refresh();
        delay_ms(100);
    }
}
```

---

## 📊 API 函数说明

### 1. Huidu_Read_Raw()

**功能：** 读取8路灰度传感器的原始状态

**返回值：** `uint8_t` - 8位数据
- Bit 0（最低位）= 最左边传感器
- Bit 7（最高位）= 最右边传感器
- 1 = 黑线，0 = 白底

**示例：**
```c
uint8_t sensor = Huidu_Read_Raw();
// 0b00011000 (0x18) = 中间两个传感器检测到黑线
```

### 2. Huidu_Get_Error()

**功能：** 计算小车相对于黑线的位置误差

**返回值：** `float` - 位置误差
- 0.0 = 小车在线中央
- 负值（-3.5 ~ 0）= 小车偏右，需要左转
- 正值（0 ~ +3.5）= 小车偏左，需要右转
- 丢线时保持上一次的误差值

**示例：**
```c
float error = Huidu_Get_Error();

if (error > 0) {
    // 小车偏左，需要右转
    motor_turn_right();
} else if (error < 0) {
    // 小车偏右，需要左转
    motor_turn_left();
} else {
    // 完美居中，直行
    motor_go_straight();
}
```

### 3. Huidu_Is_Lost()

**功能：** 判断是否完全丢线

**返回值：** `uint8_t`
- 1 = 丢线（所有传感器都是白色）
- 0 = 在线上

**示例：**
```c
if (Huidu_Is_Lost()) {
    // 丢线处理：停车或执行丢线策略
    motor_stop();
}
```

### 4. Huidu_Get_Last_Error()

**功能：** 获取上一次有效的误差值（调试用）

**返回值：** `float` - 上一次计算的误差值

---

## 🧮 误差计算原理

### 加权平均法

**权重分配：**

```
传感器位置：  S0    S1    S2    S3    S4    S5    S6    S7
           ┌────┬────┬────┬────┬────┬────┬────┬────┐
           │ -3.5│-2.5│-1.5│-0.5│ 0.5│ 1.5│ 2.5│ 3.5│
           └────┴────┴────┴────┴────┴────┴────┴────┘
            左边                                右边
```

**计算公式：**

```
误差 = Σ(传感器状态 × 权重) / Σ(传感器状态)
```

**示例1：小车居中**

```
传感器状态： 0b00011000
位置分布：   ....##....
             S3  S4

加权和 = 1×(-0.5) + 1×(0.5) = 0
传感器和 = 1 + 1 = 2
误差 = 0 / 2 = 0.0  ✅ 完美居中
```

**示例2：小车偏左（黑线在右）**

```
传感器状态： 0b01100000
位置分布：   .....##...
             S5  S6

加权和 = 1×(1.5) + 1×(2.5) = 4.0
传感器和 = 1 + 1 = 2
误差 = 4.0 / 2 = +2.0  ✅ 偏左，需要右转
```

**示例3：小车偏右（黑线在左）**

```
传感器状态： 0b00000011
位置分布：   ##........
             S0  S1

加权和 = 1×(-3.5) + 1×(-2.5) = -6.0
传感器和 = 1 + 1 = 2
误差 = -6.0 / 2 = -3.0  ✅ 偏右，需要左转
```

---

## 📋 校准与验证

### 校准步骤

1. **中心校准**
   - 将小车放在黑线正中央
   - 观察误差值是否接近 0.0（±0.5 范围内）

2. **左侧校准**
   - 将小车向左移动（黑线相对偏右）
   - 观察误差值是否变为正数（0 ~ +3.5）

3. **右侧校准**
   - 将小车向右移动（黑线相对偏左）
   - 观察误差值是否变为负数（-3.5 ~ 0）

4. **丢线测试**
   - 将小车完全移出黑线
   - 观察是否显示 "LOST"
   - 误差值保持上一次的值（不变）

### 常见情况对照表

| 传感器状态（二进制） | 十六进制 | 误差值 | 小车位置 |
|---------------------|---------|-------|---------|
| 0b00011000          | 0x18    | 0.0   | 完美居中 |
| 0b00001100          | 0x0C    | -1.0  | 稍偏右（黑线偏左） |
| 0b00110000          | 0x30    | +1.0  | 稍偏左（黑线偏右） |
| 0b00000011          | 0x03    | -3.0  | 严重偏右 |
| 0b11000000          | 0xC0    | +3.0  | 严重偏左 |
| 0b00000001          | 0x01    | -3.5  | 极限偏右 |
| 0b10000000          | 0x80    | +3.5  | 极限偏左 |
| 0b11111111          | 0xFF    | 0.0   | 全黑（起跑线？） |
| 0b00000000          | 0x00    | 保持  | 丢线 |

---

## 🔍 常见问题

### Q1：误差方向反了怎么办？

**问题：** 小车向左移动时，误差变为负数（预期应该是正数）

**解决方法：**

**方法1：检查物理安装**
- 确认传感器0在最左边，传感器7在最右边
- 如果装反了，重新安装传感器

**方法2：修改权重数组**

打开 `huidu.c`，找到权重数组，将所有权重取反：

```c
// 原权重
const float weights[8] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f
};

// 修改为（全部取反）
const float weights[8] = {
    3.5f, 2.5f, 1.5f, 0.5f, -0.5f, -1.5f, -2.5f, -3.5f
};
```

### Q2：传感器读数全是0或全是1？

**可能原因：**
1. 传感器供电不正常
2. GPIO 引脚配置错误
3. 传感器距离地面太远或太近
4. 环境光干扰（部分模拟传感器）

**排查步骤：**
1. 用万用表测量传感器供电电压（通常是3.3V或5V）
2. 用示波器或万用表测量GPIO引脚电平
3. 调整传感器高度（通常距离地面2-5mm）
4. 检查 SysConfig 中的引脚配置

### Q3：丢线后无法恢复？

**原因：** `Huidu_Get_Error()` 在丢线时保持上一次的误差值，如果控制算法不正确，可能导致小车一直跑偏。

**解决方法：**

在巡线控制代码中增加丢线检测：

```c
if (Huidu_Is_Lost()) {
    // 丢线策略1：停车
    motor_stop();
    
    // 丢线策略2：根据上一次误差方向继续转弯寻线
    // float last_error = Huidu_Get_Last_Error();
    // if (last_error > 0) {
    //     motor_turn_right();  // 上次偏左，继续右转寻线
    // } else {
    //     motor_turn_left();   // 上次偏右，继续左转寻线
    // }
}
```

### Q4：如何提高精度？

**方法1：增加传感器数量**
- 从8路改为12路或16路
- 修改权重数组和读取函数

**方法2：使用模拟灰度传感器**
- 数字传感器只能输出0/1
- 模拟传感器可以输出0-255的灰度值，精度更高

**方法3：使用更密集的权重分布**
- 当前权重间隔是1.0（-3.5 ~ +3.5）
- 可以改为0.5间隔，提高分辨率

---

## 🚀 与电机控制集成

### 简单巡线控制示例

```c
#include "huidu.h"
#include "motor.h"

// PD控制参数
#define KP  50.0f   // 比例系数
#define KD  10.0f   // 微分系数

float last_error = 0.0f;

void line_follow_control(void)
{
    // 1. 读取位置误差
    float error = Huidu_Get_Error();
    
    // 2. 计算微分（误差变化率）
    float derivative = error - last_error;
    last_error = error;
    
    // 3. PD控制算法
    float control_output = KP * error + KD * derivative;
    
    // 4. 转换为左右轮速度差
    float base_speed = 300.0f;  // 基础速度（mm/s）
    float left_speed  = base_speed - control_output;
    float right_speed = base_speed + control_output;
    
    // 5. 速度限幅
    if (left_speed < 0) left_speed = 0;
    if (right_speed < 0) right_speed = 0;
    if (left_speed > 500) left_speed = 500;
    if (right_speed > 500) right_speed = 500;
    
    // 6. 设置电机目标速度
    Motor_Left.target_speed = left_speed;
    Motor_Right.target_speed = right_speed;
}

// 在定时器中断中调用（如10ms周期）
void PID_TIMER_IRQHandler(void)
{
    line_follow_control();  // 巡线控制
    motor_pi_loop(1);       // 左轮PI控制
    motor_pi_loop(2);       // 右轮PI控制
}
```

---

## 📝 总结

### 文件清单

- ✅ `user_driver/huidu.h` - 灰度传感器驱动头文件
- ✅ `user_driver/huidu.c` - 灰度传感器驱动实现
- ✅ `huidu_test_main.c` - 测试代码示例

### 集成检查清单

- [ ] SysConfig 中配置了8个GPIO输入引脚
- [ ] 替换了 `huidu.c` 中的引脚宏定义
- [ ] 将 `huidu.c` 和 `huidu.h` 添加到工程
- [ ] 编译通过
- [ ] 在OLED上看到传感器状态显示
- [ ] 校准误差值方向和范围
- [ ] 集成到巡线控制算法

### 下一步

1. **完成校准** - 确保误差值方向正确
2. **调试PD参数** - 找到合适的Kp和Kd值
3. **测试不同线宽** - 验证在不同宽度黑线上的表现
4. **优化速度** - 根据弯道半径动态调整速度

---

**文档版本：** v1.0  
**最后更新：** 2026-07-03  
**状态：** ✅ 完成，待硬件测试
