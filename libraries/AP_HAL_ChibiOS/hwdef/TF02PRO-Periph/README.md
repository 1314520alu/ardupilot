# TF02PRO-Periph（ZY_TF02PRO-40M）

Benewake **TF02-Pro** UART → **DroneCAN** 适配固件（AP_Periph）。  
量产节点名：`ZY_TF02PRO-40M`；写入 `BRD_SERIAL_NUM` 后为 `ZY_TF02PRO-40M(序列号)`。

## 硬件

| 项目 | 说明 |
|------|------|
| MCU | GD32F103TBU6（按 STM32F103xB / 128KB Flash 编译） |
| 晶振 | 12 MHz HSE → SYSCLK 72 MHz（PLL×6） |
| 雷达 UART | USART2：**PA2 TX / PA3 RX**，115200 |
| CAN | **PA11 RX / PA12 TX**，收发器 TJA1050，默认 **500 kbit/s** |
| 烧录 | SWD（PA13/PA14），Board ID `AP_HW_TF02PRO_PERIPH` = **6900** |
| 其它 | 无 LED；TJA1050 `VCC=5V`，**S 脚接地**（勿接 Silent） |

## 编译

```bash
./waf configure --board TF02PRO-Periph
./waf bootloader
./waf AP_Periph
```

产物：

- Bootloader：`build/TF02PRO-Periph/bin/AP_Bootloader.bin`（副本 `Tools/bootloaders/TF02PRO-Periph_bl.bin`）
- 应用：`build/TF02PRO-Periph/bin/AP_Periph.bin` / `.apj`

## 烧录（SWD）

| 镜像 | 地址 |
|------|------|
| Bootloader | `0x08000000` |
| AP_Periph | `0x08006400`（`FLASH_RESERVE_START_KB 25`） |

若芯片开了读保护（RDP），先解除 RDP 再整片擦除后烧录。  
调试时建议 **拔掉 ST-Link** 再测 CAN，否则节点可能间歇消失。

## 客户参数（板端）

| 参数 | 默认 | 说明 |
|------|------|------|
| `CAN_NODE` | `0` | `0`=DNA 动态分配；多雷达建议每板固定互异号，改后重启 |
| `CAN_BAUDRATE` | `500000` | 须与飞控一致，改后重启 |
| `BRD_SERIAL_NUM` | `0` | 产线序列号；非 0 出现在 App Name |
| `RNGFND1_ADDR` | `0` | DroneCAN `sensor_id`，与飞控 `RNGFNDx_ADDR` **相同** |
| `RNGFND_MAX_RATE` | `20` | 避障/测距 DroneCAN 输出频率 (Hz)；`0`=不限 |
| `RNGFND1_MIN` | `0.1` | 可靠最小距离 (m) |
| `RNGFND1_MAX` | `40` | 可靠最大距离 (m) |
| `RNGFND1_GNDCLR` | `0.1` | 落地期望测距 (m) |

多雷达：**每块板**不同 `CAN_NODE`（或全 DNA）+ 不同 `RNGFND1_ADDR`。

出厂默认见同目录 `defaults.parm`。

## 飞控侧

| 参数 | 建议值 |
|------|--------|
| `CAN_P1_BITRATE`（或对应口） | `500000` |
| `CAN_D1_PROTOCOL` | `1`（DroneCAN） |
| `RNGFNDx_TYPE` | `24`（DroneCAN） |
| `RNGFNDx_ADDR` | 与板端 `RNGFND1_ADDR` 一致 |
| `RNGFNDx_ORIENT` / `MIN` / `MAX` / `GNDCLR` | 按安装（方向只在飞控设） |

## 已隐藏参数

硬件固定，客户界面不可见：

- `FORMAT_VERSION`、`DEBUG`、`OPTIONS`
- `RNGFND_BAUDRATE`（115200）、`RNGFND_PORT`（3）
- `RNGFND1_TYPE`（19 BenewakeTF02）
- `RNGFND1_ORIENT`（用飞控 `RNGFNDx_ORIENT`）
- `RNGFND1_PIN` / `SCALING` / `OFFSET` / `FUNCTION` / `STOP_PIN` / `RMETRIC` / `PWRRNG` / `POS_*`

## 开发注意（GD32）

1. **CAN_RX(PA11)** 必须为浮空输入；F1 `get_CR_F1` 若给 INPUT 加 SPEED 会变成开漏输出，导致 Form/ACK 错误、节点不可见。
2. F103 mcuconf 需支持 **12 MHz HSE**（PLL×6）。
3. 自环调试可用 `HAL_CAN_LOOPBACK_DEBUG`（量产固件关闭）。
