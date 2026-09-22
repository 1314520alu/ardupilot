# ZY_M323_F103 使用说明书

**ZY M323 GPS**（STM32F103CBT6）DroneCAN 外设节点：u-blox GNSS + TE MS5611 气压 + QMC/HMC 5883 罗盘。

引脚与 F303 版对齐（LED 在 **PA4**）。优先推荐 Flash/性能更好的 F303 版：见 [`../ZY_M323_F303/README.md`](../ZY_M323_F303/README.md)。

## 1. 功能概览

| 项目 | 内容 |
|------|------|
| MCU | STM32F103CBT6（128KB Flash） |
| GNSS | u-blox（USART1） |
| 气压 | TE MS5611（I2C `0x77` / `0x76`） |
| 罗盘 | QMC5883L `@0x0D` 或 HMC5843 `@0x1E` |
| CAN | 默认 **500 kbit/s**，节点 ID **30** |
| 节点名 | `org.ardupilot.ZY_M323_F103` |

## 2. 引脚

| 功能 | 引脚 |
|------|------|
| CAN RX / TX | PA11 / PA12 |
| GPS UART | USART1（PA9 TX / PA10 RX） |
| I2C1（气压+罗盘） | PB6 SCL / PB7 SDA |
| 状态 LED | **PA4**（高电平亮） |

## 3. 编译

```bash
./waf configure --board ZY_M323_F103
./waf AP_Periph
# 产物：build/ZY_M323_F103/bin/AP_Periph.bin (.apj / .hex)
```

Bootloader：

```bash
./waf configure --board ZY_M323_F103 --bootloader
./waf bootloader
# 或使用 Tools/bootloaders/ZY_M323_F103_bl.{bin,hex}
```

## 4. 刷写

与 F303 相同：ST-Link 先刷 `ZY_M323_F103_bl`，再刷 `AP_Periph`；CAN 500k、两端终端电阻。

## 5. 飞控 / 节点默认

与 F303 相同：CAN **500k**、节点 **30**、单气压实例、隐藏 GPS 高级参数、`printf` 走 CAN LogMessage。

## 6. 气压（重要）

F303 同设计上 MS5611 可正常探测（`BARO1_DEVID` 非 0）。

**部分 F103 板**上 I2C 扫描能看到罗盘（`0x0D`），但 **MS5611 无 ACK**，CAN 日志类似：

```text
MS5611 fail a=0x77 rst=0 z=1 p1=0
BARO n=0 healthy=0
```

此时优先查硬件，而非固件：

- MS5611 **CSB** 是否拉到正确地址（`0x77` / `0x76`）
- **PS** 是否为 I2C 模式（非 SPI）
- I2C 上拉、焊接、与 F303 板对比同一模块

固件侧已对 F103 I2C 做：`HAL_I2C_CLEAR_ON_TIMEOUT=0`、MS5611 `split_transfers`、复位后延时加长。

## 7. 验收清单

- [ ] 节点名 `org.ardupilot.ZY_M323_F103`
- [ ] GPS Fix
- [ ] 罗盘有数据
- [ ] 气压：若 `BARO1_DEVID=0`，按上一节查硬件
- [ ] LED（PA4）工作
