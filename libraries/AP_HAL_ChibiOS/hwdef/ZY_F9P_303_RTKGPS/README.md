# ZY_F9P_303_RTKGPS 使用说明书

双 **u-blox NEO-F9P** + **STM32F303CBT6** DroneCAN GPS 节点，用于 **Moving Baseline 航向（GPS-for-Yaw）**。

## 1. 系统架构

```
┌─────────────┐  UART2 RTCM@460800  ┌─────────────┐
│ F9P Base    │ ──────────────────► │ F9P Rover   │
│ (天线 A)    │   模块间直连        │ (天线 B)    │
└──────┬──────┘                     └──────┬──────┘
       │ UART1 UBX@230400                  │ UART1 UBX@230400
       ▼                                   ▼
┌─────────────┐                     ┌─────────────┐
│ F303 Base   │                     │ F303 Rover  │
│ GPS_TYPE=17 │                     │ GPS_TYPE=18 │
└──────┬──────┘                     └──────┬──────┘
       │ DroneCAN                          │ DroneCAN
       └──────────────┬────────────────────┘
                      ▼
                 飞控 (类型 22 / 23)
```

- **基线 RTCM**：F9P↔F9P **UART2 直连**（不走 CAN、不经 F303）
- **位置 / 航向**：F303 解析后经 **DroneCAN** 上报（Fix2 + RelPosHeading）
- **无罗盘 / 无气压**：纯 GPS 节点

## 2. 硬件要点

| 项目 | 要求 |
|------|------|
| MCU | STM32F303CBT6（128KB Flash） |
| GNSS | NEO-F9P-0-02B（两边型号一致） |
| F9P 固件 | 建议 **HPG ≥ 1.32**，两边版本相同 |
| UART2 | Base TX→Rover RX，共地，线短，远离图传/ESC |
| 天线 | 同型号、同高、同朝向；基线建议 **≥ 50 cm** |
| CAN | 两端 120Ω；建议 **500k 或 1M**（全机一致） |

## 3. 编译与刷写

```bash
./waf configure --board ZY_F9P_303_RTKGPS
./waf AP_Periph
# 产物：build/ZY_F9P_303_RTKGPS/bin/AP_Periph.bin (.apj)
```

- Board ID：`AP_HW_ZY_F9P_303_RTKGPS` = **6901**
- CAN 节点名：`org.ardupilot.ZY_F9P_303_RTKGPS`
- 两颗节点刷 **同一固件**，用参数区分 Base / Rover

## 4. 节点参数（Periph）

上电默认（代码写入，无需 defaults.parm）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `GPS_TYPE` | **18**（Rover） | Base 天线节点改为 **17** |
| `GPS_DRV_OPTIONS` | **1** | bit0 = UART2 传 RTCM |
| `GPS1_RATE_MS` | **200** | 5 Hz |
| `GPS1_GNSS_MODE` | **77** | GPS+Galileo+Beidou+QZSS |
| `GPS_AUTO_CONFIG` | **1** | 自动 VALSET |

**Base 节点**：DroneCAN → 该节点 `GPS_TYPE = 17` → 重启。  
**Rover 节点**：保持 `18`。

## 5. 飞控参数

| 参数 | 值 |
|------|-----|
| `GPS1_TYPE` | `22`（DroneCAN MB Base） |
| `GPS2_TYPE` | `23`（DroneCAN MB Rover） |
| `GPS_AUTO_CONFIG` | `2` |
| `GPS_DRV_OPTIONS` | bit0 = 1（与节点一致） |
| `GPS1_POS_*` / `GPS2_POS_*` | 实测天线相位中心（误差 &lt; 约 20%） |
| `EK3_SRC1_YAW` | `2`（GPS） |

## 6. F303 自动配置的 F9P 内容

开启 `GPS_TYPE` 17/18 + `GPS_DRV_OPTIONS` bit0 后，自动写：

**UART2（模块间）**

- 波特率 460800  
- Base：仅出 RTCM（关 UBX/NMEA）  
- Rover：仅收 RTCM  

**UART1（接 F303）**

- 波特率 230400  
- 仅 UBX（关 NMEA）  
- Rover：输出 `NAV-RELPOSNED`  

**其它**

- 关闭 Survey-In（`TMODE=0`）  
- 导航 5 Hz  
- 动模 Airborne &lt; 4g  

## 7. 调试（CAN Messages）

固件打开 `F9PDBG` / CFG / MB 调试，Mission Planner → Messages 可过滤：

| 关键字 | 含义 |
|--------|------|
| `F9PDBG gps.init` | 初始化 |
| `F9PDBG status` | Fix 状态变化 |
| `F9PDBG yaw ON/OFF` | 航向有效性 |
| `F9PDBG RelPosH` | 航向/基线周期输出 |
| `MB Base/Rover cfg UART2` | 正在下发 MB 配置 |

调通后若 CAN 忙，可在 `hwdef.dat` 关闭：

- `UBLOX_CFG_DEBUGGING` / `UBLOX_MB_DEBUGGING` / `AP_PERIPH_GPS_DEBUG_ENABLED`

## 8. CAN 与资源

- `HAL_CAN_POOL_SIZE` = **6000**（减轻发送阻塞）  
- CBT6 Flash 紧张：未嵌入 ROMFS/`defaults.parm`  
- RTCM **不要**改走 CAN（易堵，Fixed/航向一起差）

## 9. 验收清单

1. 两颗 F9P `MON-VER` 固件 ≥ 1.32 且一致  
2. Rover 开阔地长期 **RTK Fixed**  
3. Messages 有 `RelPosH`，`dist` 接近实测基线  
4. 转动机头，航向跟随且静止抖动可接受  
5. 冷启动无需 u-center 手工改 UART  

## 10. 常见问题

| 现象 | 排查 |
|------|------|
| 长期 Float / 无航向 | F9P 固件、UART2 接线、星座两边是否同为 77 |
| 有 Fixed 无航向 | Periph `GPS_TYPE` 是否 17/18；飞控是否 22/23；`POS` 是否准 |
| 航向时有时无 | CAN 拥堵 / 调试刷屏；查 `Tx error`；可关 DEBUG |
| 节点不识别 | Board ID 6901、bootloader 布局 20KB |

---

目录：`libraries/AP_HAL_ChibiOS/hwdef/ZY_F9P_303_RTKGPS/`  
相关改动：`AP_GPS_UBLOX` UART1/2 MB 配置增强、`Tools/AP_Periph` 调试与默认参数。
