# TF03 内部 MCU 与 DroneCAN 可行性说明

**日期:** 2026-09-22  
**状态:** 结论记录（不直接刷原厂测距固件）  
**相关:** [TF02PRO UART→DroneCAN 设计](./2026-09-22-tf02pro-dronecan-periph-design.md)

## 背景

北醒 TF03 拆机可见主控为 **GD32F103CBT6**（LQFP48，与 STM32F103CBT6 引脚兼容）。目标曾评估：能否把该芯片改造成标准 **DroneCAN** 测距节点。

## 芯片 CAN / SWD 引脚（对照用）

片上仅一路 CAN，两组复用：

| 配置 | CAN_RX | CAN_TX | 封装脚号 |
|------|--------|--------|----------|
| 默认 | PA11 | PA12 | 32 / 33 |
| 重映射 | PB8 | PB9 | 45 / 46 |

SWD：PA13 = SWDIO（34），PA14 = SWCLK（37）。

矢量顶视图：[`docs/GD32F103CBT6-LQFP48-CAN-pinout.svg`](../../GD32F103CBT6-LQFP48-CAN-pinout.svg)

## 实测结论：不宜直接刷板载 GD32

板载 GD32 **不是** 单纯 UART/CAN 桥，还接了 **激光发射等测距前端控制脚**。原厂固件负责测距时序与光学前端；无公开内部原理图，重写测距固件再发 DroneCAN **不可行**。

公开资料仅有外部接口（线序、UART/CAN 协议、外形），无 PCB 原理图。

## 推荐路径

| 方案 | 做法 | 飞控侧 | 建议 |
|------|------|--------|------|
| A. 外挂 AP_Periph | 保留 TF03 原厂固件；另板读 UART，发 DroneCAN（同 TF02PRO） | `RNGFND_TYPE=24` | **要标准 DroneCAN 时首选** |
| B. 北醒私有 CAN | TF03 切 CAN 模式，飞控直连 | `RNGFND_TYPE=34`（Benewake_CAN） | 能测距即可时最简单 |
| C. 串口直连 | TF03 UART → 飞控串口 | `RNGFND_TYPE=27`（BenewakeTF03） | 无 CAN 总线时可用 |
| D. 刷 TF03 板载 GD32 | 自研测距 + DroneCAN | — | **不做**（缺原理图、激光控制耦合） |

数据路径（方案 A）：

```
TF03（原厂固件，UART）
    ↔  外挂 GD32 / F103（AP_Periph）
    ↔  CAN（DroneCAN range_sensor.Measurement）
飞控
```

## 与 TF02PRO 工作的关系

TF02PRO 适配板 MCU 主要做 UART↔CAN，可刷 AP_Periph。  
TF03 板载 MCU 兼管激光前端，角色不同，**不能照搬“刷传感器内部 MCU”**；若要 DroneCAN，应另做/复用 **外挂 Periph**，而不是改写 TF03 内部程序。

## 待定（产品决策）

- 飞控是否必须标准 DroneCAN（24），还是 Benewake_CAN（34）即可。
- 若走方案 A：复用 TF02PRO-Periph 思路另建 `TF03-Periph` hwdef，或共用适配板硬件改默认 `RNGFND1_TYPE=27`。

## 参考

- ArduPilot：`AP_RangeFinder` 类型 24 / 27 / 34；`Tools/AP_Periph/rangefinder.cpp` 发布 `uavcan.equipment.range_sensor.Measurement`
- 北醒 TF03 产品手册：外部 UART / CAN 线序与协议（非 DroneCAN）
