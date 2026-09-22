# ZY_M323_F303 使用说明书

**ZY M323 GPS**（STM32F303CBT6）DroneCAN 外设节点：u-blox GNSS + TE MS5611 气压 + QMC/HMC 5883 罗盘。

同系列还有 F103 版：见 [`../ZY_M323_F103/README.md`](../ZY_M323_F103/README.md)。

## 1. 功能概览

| 项目 | 内容 |
|------|------|
| MCU | STM32F303CBT6（**128KB** Flash） |
| GNSS | u-blox（实测常见为 M9 / HW `00190000`） |
| 气压 | TE MS5611（I2C `0x77` / `0x76`） |
| 罗盘 | QMC5883L `@0x0D`（常标 HA5883）或 HMC5843 `@0x1E` |
| CAN | 默认 **500 kbit/s**，节点 ID **30** |
| 节点名 | `org.ardupilot.ZY_M323_F303` |

## 2. 引脚（与常见 F303 CAN-GPS 小板一致）

| 功能 | 引脚 |
|------|------|
| CAN RX / TX | PA11 / PA12 |
| GPS UART | USART1（PA9 TX / PA10 RX） |
| I2C1（气压+罗盘） | PB6 SCL / PB7 SDA |
| 状态 LED | PA4（高电平亮） |

## 3. 编译

```bash
./waf configure --board ZY_M323_F303
./waf AP_Periph
# 产物：build/ZY_M323_F303/bin/AP_Periph.bin (.apj / .hex)
```

Bootloader（首次 SWD 烧录）：

```bash
./waf configure --board ZY_M323_F303 --bootloader
./waf bootloader
# 或使用仓库内 Tools/bootloaders/ZY_M323_F303_bl.{bin,hex}
```

## 4. 刷写（ST-Link / STM32CubeProgrammer）

1. 接 SWD：SWDIO / SWCLK / GND / 3V3（勿与飞控同时供电冲突）。
2. 先刷 **bootloader** 到 `0x08000000`。
3. 再刷 **AP_Periph**（或用 Mission Planner / DroneCAN GUI 经 CAN 升级）。
4. 接 CAN 到飞控（两端 120Ω，全机波特率一致）。

## 5. 飞控侧要点

| 项 | 建议 |
|----|------|
| CAN 波特率 | **500000**（与节点默认一致） |
| 总线 | 本机调试常用 **CAN2**（`bus_number=2`） |
| GPS | DroneCAN GPS（按飞控版本选对应 TYPE） |
| 气压 / 罗盘 | 开启 DroneCAN 传感器；节点侧已固定探测，无需 BARO2/3 |

节点默认：

| 参数/宏 | 值 | 说明 |
|---------|-----|------|
| `HAL_CAN_BAUDRATE_DEFAULT` | 500000 | CAN 速率 |
| `HAL_CAN_DEFAULT_NODE_ID` | 30 | 默认节点号 |
| `BARO_MAX_INSTANCES` | 1 | 不暴露 BARO2/3 |
| `AP_GPS_ADVANCED_CONFIG_ENABLED` | 0 | 隐藏少用 GPS 高级参数 |
| `HAL_PERIPH_PRINTF_TO_CAN` | 1 | `printf` → DroneCAN LogMessage |

## 6. 调试（Mission Planner → Messages）

上电可见类似：

- `I2C probe...` — I2C 扫描
- `BARO n=1 healthy=1` — 气压已就绪（`BARO1_DEVID` 非 0，如 `751361`）
- MS5611 失败时：`MS5611 fail a=0x77 rst=…`（短日志，省 CAN 池）

## 7. 已知说明

- CBT6 仅 128KB：无 ROMFS / 无板载 BL 二次烧录镜像。
- 气压、罗盘走 **同一 I2C1**；GPS 走 USART1。
- 参数精简不影响其它板：默认 `AP_GPS_ADVANCED_CONFIG_ENABLED=1`，仅本 hwdef 关掉。

## 8. 验收清单

- [ ] CAN 上出现节点 `org.ardupilot.ZY_M323_F303`（ID 约 30）
- [ ] GPS 有 Fix / 卫星数
- [ ] `BARO1_DEVID ≠ 0` 且高度合理
- [ ] 罗盘有数据、方向正确
- [ ] LED 有心跳/活动指示
