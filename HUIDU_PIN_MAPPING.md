# 灰度传感器引脚映射说明

**更新时间：** 2026-07-03  
**硬件配置：** 8路数字灰度传感器

---

## 📌 引脚映射关系

### 物理布局（从左到右）

```
┌────┬────┬────┬────┬────┬────┬────┬────┐
│ L4 │ L3 │ L2 │ L1 │ R1 │ R2 │ R3 │ R4 │
└────┴────┴────┴────┴────┴────┴────┴────┘
  ↑                                      ↑
 最左边                               最右边
```

### 代码位序映射

| 物理位置 | 传感器名称 | GPIO引脚 | 代码位序 | 权重 |
|---------|-----------|---------|---------|------|
| 最左边 | L4 | GPIOA.16 | Bit 0 | -3.5 |
| 左3 | L3 | GPIOA.12 | Bit 1 | -2.5 |
| 左2 | L2 | GPIOA.13 | Bit 2 | -1.5 |
| 左1 | L1 | GPIOA.15 | Bit 3 | -0.5 |
| 右1 | R1 | GPIOA.17 | Bit 4 | +0.5 |
| 右2 | R2 | GPIOA.24 | Bit 5 | +1.5 |
| 右3 | R3 | GPIOB.8  | Bit 6 | +2.5 |
| 最右边 | R4 | GPIOA.14 | Bit 7 | +3.5 |

### SysConfig 生成的宏定义

```c
// 从 Debug/ti_msp_dl_config.h

// L4 - 最左边
#define HUI_DU_L4_PORT    (GPIOA)
#define HUI_DU_L4_PIN     (DL_GPIO_PIN_16)

// L3
#define HUI_DU_L3_PORT    (GPIOA)
#define HUI_DU_L3_PIN     (DL_GPIO_PIN_12)

// L2
#define HUI_DU_L2_PORT    (GPIOA)
#define HUI_DU_L2_PIN     (DL_GPIO_PIN_13)

// L1
#define HUI_DU_L1_PORT    (GPIOA)
#define HUI_DU_L1_PIN     (DL_GPIO_PIN_15)

// R1
#define HUI_DU_R1_PORT    (GPIOA)
#define HUI_DU_R1_PIN     (DL_GPIO_PIN_17)

// R2
#define HUI_DU_R2_PORT    (GPIOA)
#define HUI_DU_R2_PIN     (DL_GPIO_PIN_24)

// R3
#define HUI_DU_R3_PORT    (GPIOB)
#define HUI_DU_R3_PIN     (DL_GPIO_PIN_8)

// R4 - 最右边
#define HUI_DU_R4_PORT    (GPIOA)
#define HUI_DU_R4_PIN     (DL_GPIO_PIN_14)
```

---

## 🔢 数据格式说明

### 原始数据（uint8_t）

每一位代表一个传感器的状态：
- **1** = 检测到黑线
- **0** = 检测到白底

**位序排列（从低到高）：**
```
Bit:  7   6   5   4   3   2   1   0
传感器: R4  R3  R2  R1  L1  L2  L3  L4
```

### 示例数据解读

**示例1：小车居中**
```
二进制：0b00011000
十六进制：0x18
含义：L1 和 R1 检测到黑线（中间两个传感器）
误差：0.0（完美居中）
```

**示例2：小车偏右**
```
二进制：0b00000110
十六进制：0x06
含义：L2 和 L3 检测到黑线（黑线在左侧）
误差：-2.0（小车偏右，需要左转）
```

**示例3：小车偏左**
```
二进制：0b01100000
十六进制：0x60
含义：R2 和 R3 检测到黑线（黑线在右侧）
误差：+2.0（小车偏左，需要右转）
```

---

## 🎯 权重分配说明

### 位置权重表

```
传感器：  L4    L3    L2    L1    R1    R2    R3    R4
权重：   -3.5  -2.5  -1.5  -0.5   0.5   1.5   2.5   3.5
```

### 误差计算公式

```
误差 = Σ(传感器状态 × 权重) / Σ(传感器状态)
```

**说明：**
- **负权重（左侧传感器）**：检测到黑线 → 黑线在左 → 小车偏右 → 误差为负 → 需要左转
- **正权重（右侧传感器）**：检测到黑线 → 黑线在右 → 小车偏左 → 误差为正 → 需要右转
- **权重为0的中间位置**：不存在，L1=-0.5，R1=+0.5，保持对称性

---

## 🔧 huidu.c 中的实现

### Huidu_Read_Raw() 函数

```c
uint8_t Huidu_Read_Raw(void)
{
    uint8_t sensor_data = 0;

    // Bit 0: L4（最左边）
    if (DL_GPIO_readPins(HUI_DU_L4_PORT, HUI_DU_L4_PIN) == 0) {
        sensor_data |= (1 << 0);
    }

    // Bit 1: L3
    if (DL_GPIO_readPins(HUI_DU_L3_PORT, HUI_DU_L3_PIN) == 0) {
        sensor_data |= (1 << 1);
    }

    // Bit 2: L2
    if (DL_GPIO_readPins(HUI_DU_L2_PORT, HUI_DU_L2_PIN) == 0) {
        sensor_data |= (1 << 2);
    }

    // Bit 3: L1
    if (DL_GPIO_readPins(HUI_DU_L1_PORT, HUI_DU_L1_PIN) == 0) {
        sensor_data |= (1 << 3);
    }

    // Bit 4: R1
    if (DL_GPIO_readPins(HUI_DU_R1_PORT, HUI_DU_R1_PIN) == 0) {
        sensor_data |= (1 << 4);
    }

    // Bit 5: R2
    if (DL_GPIO_readPins(HUI_DU_R2_PORT, HUI_DU_R2_PIN) == 0) {
        sensor_data |= (1 << 5);
    }

    // Bit 6: R3
    if (DL_GPIO_readPins(HUI_DU_R3_PORT, HUI_DU_R3_PIN) == 0) {
        sensor_data |= (1 << 6);
    }

    // Bit 7: R4（最右边）
    if (DL_GPIO_readPins(HUI_DU_R4_PORT, HUI_DU_R4_PIN) == 0) {
        sensor_data |= (1 << 7);
    }

    return sensor_data;
}
```

### 权重数组

```c
const float weights[8] = {
    -3.5f,  // Bit 0: L4（最左边）
    -2.5f,  // Bit 1: L3
    -1.5f,  // Bit 2: L2
    -0.5f,  // Bit 3: L1
     0.5f,  // Bit 4: R1
     1.5f,  // Bit 5: R2
     2.5f,  // Bit 6: R3
     3.5f   // Bit 7: R4（最右边）
};
```

---

## ✅ 已修复的问题

### 修复前（错误的占位符）

```c
// ❌ 使用占位符，需要手动替换
if (DL_GPIO_readPins(HUIDU_PORT, HUIDU_PIN_0) == 0) {
    sensor_data |= (1 << 0);
}
```

### 修复后（正确的引脚名称）

```c
// ✅ 使用实际的引脚宏定义
if (DL_GPIO_readPins(HUI_DU_L4_PORT, HUI_DU_L4_PIN) == 0) {
    sensor_data |= (1 << 0);
}
```

---

## 📋 校准验证清单

- [ ] 编译通过，无引脚宏定义错误
- [ ] OLED 显示传感器状态（二进制+十六进制）
- [ ] 小车在线中央时，误差接近 0.0
- [ ] 小车向左移动时，误差变为正数（0 ~ +3.5）
- [ ] 小车向右移动时，误差变为负数（-3.5 ~ 0）
- [ ] 完全丢线时，显示 "LOST"，误差保持上次值
- [ ] 权重方向正确（负数左转，正数右转）

---

**更新说明：** 已将占位符宏 `HUIDU_PORT/HUIDU_PIN_X` 替换为实际的 `HUI_DU_LX/RX_PORT/PIN`  
**状态：** ✅ 引脚映射正确，可以直接编译使用
