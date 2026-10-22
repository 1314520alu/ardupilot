# AP_eRobCANOpen

零差云控 eRob（CANopen / CiA402）的最小主站。目标机是零一 X6 Ultra，仓库里的板型是 `libraries/AP_HAL_ChibiOS/hwdef/ZeroOneX6`。

飞控侧由 `CAN_Dx_PROTOCOL = 15`（`eRobCAN`，参数里显示 `eRob CAN`）构造 `AP_eRobCANOpen`。`AP_EROB_CANOPEN_ENABLED` 默认为 0，源码在 `src/`，由 `AP_eRobCANOpen.cpp` 在该宏为 1 时包含。ZeroOneX6 的 hwdef 打开这个宏。没选协议 15 时不会构造驱动。CAN1 上的 DroneCAN 默认参数不动。

## 与上游 ArduPilot 的关系

fork：[1314520alu/ardupilot](https://github.com/1314520alu/ardupilot)，父仓库是 [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)。

功能分支 `feature/erob-canopen-can2` 从已经合并上游的 `master` 拉出。对齐的上游提交是：

- 提交 `3d24375e579203b8af27605df841dc1022f0e9e1`
- 时间 2026-10-22
- 说明 `autotest: add Sub LOITER_TURNS circle direction tests`
- fork 上的合并提交 `2fa0bb6a170e1b3fef9d25e653bbd5e43fca5c87`

fork 自己的板级改动（ZeroOneX9、RPLidar S 系列等）仍留在 `master` 上，本驱动提交不碰它们。

以后保持一致：

```bash
git remote add upstream https://github.com/ArduPilot/ardupilot.git
git fetch upstream master
git checkout master
git merge upstream/master
git push origin master
git checkout feature/erob-canopen-can2
git rebase master
```

不要把本驱动直接堆在过期的 fork 提交上。上游往前走之后，先合并 `upstream/master`，再变基本分支。

## CAN1 / CAN2

`ZeroOneX6/hwdef.dat` 已经有两路 CAN：

| 丝印 | MCU | 本项目 |
| --- | --- | --- |
| CAN1 | PD0 / PD1 | DroneCAN。默认用途不动 |
| CAN2 | PB12 / PB13 | 协议 15（eRob CAN）时整口归该驱动 |

`ZeroOneX6/defaults.parm` 里 `CAN_P1_DRIVER 1` 且 `CAN_P2_DRIVER 1`：两个物理口都交给虚拟驱动 1，协议仍是 DroneCAN。数值没有改，只加了注释。现有固件仍是两路 DroneCAN，直到用户改参数并重启。

要选 eRob CAN，按 ArduPilot 的三层参数打开 CAN2，CAN1 保持不动：

```text
CAN_P1_DRIVER    1
CAN_D1_PROTOCOL  1
CAN_P2_DRIVER    2
CAN_D2_PROTOCOL  15
CAN_P2_BITRATE   1000000
CAN_D2_EROB_NODE 1
CAN_D2_EROB_BM   1008
CAN_D2_EROB_MIN  0
CAN_D2_EROB_MAX  524288
CAN_D2_EROB_GEAR 0
CAN_D2_EROB_RES  524288
```

上面这组是旧的连续编号。`SV1`..`SV8` 全是 0 时才走它：`BM` 从低位到高位，最低位用 `NODE`，下一位用下一个 Node-ID。`1008` 是 bit 4..9，也就是 SERVO5..SERVO10，节点 1..6。

任意 Node-ID 用槽，不要靠连续编号。全名 `CAN_D2_EROB_SV1`、`CAN_D2_EROB_ID1` 都是 15 个字符（上限 16）。`SVn` 是 SERVO 输出号 1..32，0 表示这个槽不用。`IDn` 是该槽的 CANopen Node-ID，1..127。飞控不写 `0x100B`，ID 在舵机上写死。改 `SV` / `ID` / `NODE` / `BM` 都要重启。`MIN`、`MAX`、`GEAR`、`RES`、`CTR` 仍是这一路所有轴共用，`CTR` 不用重启。

只要有一个 `SVn` 不是 0，槽生效，`NODE` 和 `BM` 忽略。`SVn` 全是 0 才回到上面的 `NODE` + `BM`。初始化时跳过并在地面站告警：重复的 ID、重复的 SERVO 号、ID 超出 1..127、或者 `SVn` 有值但 `IDn` 是 0。先出现的那个槽保留。

`GEAR` 默认就是 0。要电机转速时改成电机转数 / 输出转，不用为了它重启。eRob142 铭牌减速比要加 1，120 写成 121。

`RES` 是输出端一圈的编码器计数，默认 524288（手册 §5.2.1 的 19 bit）。全名 `CAN_D2_EROB_RES` 是 15 个字符。角度和输出转速都除以它。改它不用重启。0 或负数按 524288 算。`MIN` / `MAX` 仍是 plus，驱动不会按 `RES` 改写它们。HS 高精度单圈可能是 20 bit，也就是 1048576。手册 V1.9 的对象目录没有 `0x608F`（`0x6087` 之后是 `0x6093`，而 `0x6093` 位置因子标明暂不支持），所以驱动不读分辨率，也不覆盖这个参数。确认方法：输出轴转一整圈，看 `0x6064` 的增量。

`15` 是 `AP_CAN::Protocol::eRobCAN`，参数列表显示为 `eRob CAN`。一个 `CAN_Dx_PROTOCOL` 独占那个虚拟驱动：1 是 DroneCAN，15 是这组舵机。驱动只接受 HAL 接口 1（CAN2）。这不是通用 CANopen 工具，默认只做 CiA402 CSP。

eRob 按舵机用。`CAN_D2_EROB_BM` 的位和 `CAN_Dx_UC_SRV_BM` 一样：bit 0 是 SERVO1。全名放不下 `SRV_BM`（`CAN_D2_EROB_SRV_BM` 有 18 个字符，上限 16），所以参数名是 `BM`。最多 8 轴。`SRV_Channels::push()` 把 PWM 线性映到 `MIN`..`MAX`（相对 `CTR` 的偏移），再加上共用的 `CTR` 写入 `0x607A`。`CTR` 默认 0，目标仍是映射值本身。一圈输出端是 `RES` 个计数。掩码为 0 且槽也全空时不跟舵机输出。第一轴负责发 SYNC，其余轴只收这个 SYNC。同一接口上的其它帧被丢掉。CAN1 保持 `CAN_D1_PROTOCOL 1`。使能后 500 ms 内主站仍保持实际位置，之后才跟踪舵机目标。每轴单独的行程还没做。输出角度不乘减速比；电机转速才用 `GEAR`。

遥测名字仍是 `ER` + SERVO 号 + 字母。日志 `ERB1` / `ERB3` 的 `Sv` 是这个 SERVO 号。槽 `SV1=5`、`ID1=11` 时，节点 11 的数据出现在 `ER5*`，`Sv` 为 5。

启动时在 CAN 线程上做一次在线检查，不写对象字典。先广播 NMT reset communication，在最多 2 s 内等这些节点的 boot-up（`0x700+id`，数据 `0x00`），提前收齐就结束。还没出现的节点再读一次 `0x1000`（SDO 超时 100 ms）。地面站原文是：

```text
eRob: SERVO5 id 12 OK
eRob: SERVO6 id 13 no response
```

`OK` 是 INFO，`no response` 是 WARNING。之后仍按原来的流程逐轴配置 CSP。

## DY1000 四个舵机，ID 不连续

四个舵机对象相同，ID 在舵机上已经写好。例如 SERVO5..8 对节点 11..14：

```text
CAN_P1_DRIVER    1
CAN_D1_PROTOCOL  1
CAN_P2_DRIVER    2
CAN_D2_PROTOCOL  15
CAN_P2_BITRATE   1000000
CAN_D2_EROB_SV1  5
CAN_D2_EROB_ID1  11
CAN_D2_EROB_SV2  6
CAN_D2_EROB_ID2  12
CAN_D2_EROB_SV3  7
CAN_D2_EROB_ID3  13
CAN_D2_EROB_SV4  8
CAN_D2_EROB_ID4  14
CAN_D2_EROB_MIN  0
CAN_D2_EROB_MAX  524288
CAN_D2_EROB_GEAR 0
CAN_D2_EROB_RES  524288
```

`SV5`..`SV8` 和 `ID5`..`ID8` 保持 0。此时 `CAN_D2_EROB_NODE` 和 `CAN_D2_EROB_BM` 不参与映射。SERVO5 驱动节点 11，遥测是 `ER5*`。

## 传统直升机斜盘

传统直升机（ArduCopter Heli）是三舵斜盘加尾舵。斜盘类型在 `H_SW_TYPE`（H3 或默认的 H3_120）。本驱动不改直升机混控，只把已经算好的 SERVO PWM 送到对应节点。

| 输出 | 功能 | 功能号 | 槽 |
| --- | --- | --- | --- |
| SERVO1 | Motor1，斜盘 | 33 | `SV1=1` |
| SERVO2 | Motor2，斜盘 | 34 | `SV2=2` |
| SERVO3 | Motor3，斜盘 | 35 | `SV3=3` |
| SERVO4 | Motor4，尾舵 | 36 | `SV4=4` |

60° 行程、中位两侧各 30°。`MIN` / `MAX` 是相对中心的偏移，不是绝对编码器计数。PWM 的最小端映到 `CTR + MIN`，最大端映到 `CTR + MAX`。PWM 中点（对称安装时的 `SERVOx` 中位）映到 `CTR + (MIN+MAX)/2`。`CTR` 默认 0，这时行为和以前一样，目标就是映射值本身。各轴共用这一个 `CTR`。型号参数见下一节。下面的 ID 只是例子。

```text
SERVO1_FUNCTION  33
SERVO2_FUNCTION  34
SERVO3_FUNCTION  35
SERVO4_FUNCTION  36
CAN_D2_EROB_SV1  1
CAN_D2_EROB_ID1  11
CAN_D2_EROB_SV2  2
CAN_D2_EROB_ID2  12
CAN_D2_EROB_SV3  3
CAN_D2_EROB_ID3  13
CAN_D2_EROB_SV4  4
CAN_D2_EROB_ID4  14
CAN_D2_EROB_MIN  -43691
CAN_D2_EROB_MAX  43691
CAN_D2_EROB_RES  524288
CAN_D2_EROB_CTR  0
```

飞行前用 `H_SV_MAN` 核对每个舵的方向和中位。`H_SV_MAN` 飞行时必须是 0（Disabled）。方向用舵机通道自己的反向，驱动不再翻一次。

状态字 bit3（`0x6041 & 0x0008`）在任一已映射舵机上置位时，包括斜盘 SERVO1..3，地面站发 CRITICAL，约每 5 s 重复一次：

```text
eRob: SERVO1 id 11 statusword fault
```

这条文字先入队，然后该槽只发一次故障复位控制字，之后不再写 CSP 目标，直到重启。不要在没有这条 CRITICAL 的情况下继续让该槽动。尾舵同样告警。这不是直升机故障保护逻辑，混控代码没有改。

## eRob142H120I-FHS-18CT

型号 `eRob142H120I-FHS-18CT[V4]`，用在大型无人传统直升机，4 轴（3 个斜盘 + 尾舵）。上一节的 `SV1`..`SV4` = 1..4 和功能号 33..36 直接套这台。

| 项 | 值 |
| --- | --- |
| 铭牌减速比 | 120。电机转数是铭牌加 1，所以 `GEAR` 填 121 |
| `RES` | 524288。客户确认单圈中点是 262144，等于 `RES/2`，所以是 19 bit |
| `CTR` | 262144。`MIN` / `MAX` 是相对它的偏移 |
| ±30° | `MIN` −43691，`MAX` 43691。绝对目标是 218453..305835。PWM 中点是 262144，角度 0° |
| 额定扭矩 | 178 N·m。驱动仍从 `0x6076` 读 mN·m，不把 178 写进参数 |
| 峰值输出转速 | 16.7 rpm，大约 100 °/s。60° 全行程至少 0.6 s |
| 刹车 | 无。掉电后斜盘不会被抱闸保住 |
| 供电 | 48 V |

```text
CAN_D2_EROB_GEAR 121
CAN_D2_EROB_RES  524288
CAN_D2_EROB_CTR  262144
CAN_D2_EROB_MIN  -43691
CAN_D2_EROB_MAX  43691
```

写入 `0x607A` 的绝对计数是 `CTR + 映射值`。PWM 最小端是 218453，最大端是 305835，中点是 262144。`ER5G` 和 `ERB3.Deg` 是 `(0x6064 - CTR) * 360 / RES`，`CTR` 处为 0°。各轴共用这一个 `CTR`。

装好后在机械中位读 `0x6064`。单圈中点应落在 262144 附近，多圈部分应为 0。若读数是 `262144 + N * 524288`，说明多圈计数不是 0，`CTR` 要填实际读数，不能只填 `RES/2`。`ER5P` 仍是原始 plus，不减中心。

## 协议范围

依据《eRob CANopen and EtherCAT 用户手册 V1.9》。只做 eRob 这一条链路：

| 做 | 不做 |
| --- | --- |
| NMT Stop / Reset communication / Start | 节点守护、LSS、完整对象字典 |
| 快速 SDO 读写 | 分段 SDO、EDS 解析 |
| 主站发 SYNC `0x080` | 多个 SYNC 源 |
| 固定 RxPDO1 与 TxPDO1..4 | 通用 PDO 映射引擎 |
| CiA402：清故障 → 6 → 7 → `0x000F` | PP/CSV/CST 的完整工具链 |

默认：Node-ID `1`（占位）、CSP（`0x6060 = 8`）、SYNC 周期 4000 µs。手册 §5.5.1 把 `0x1006 = 1000` 写成 1000 µs，所以这里按微秒写入 4000。从站的 `0x1005` 写成 `0x00000080`，禁止从站自己发 SYNC。

周期顺序与手册示例一致：先 RxPDO1（`0x200+Node-ID`，控制字 + 目标位置），再 SYNC。从站按传输类型 1 回四个 TxPDO。TxPDO1 仍是状态字 + 实际位置。

## 反馈

每个已映射的舵机缓存最新值。CSP 的 `push_servos()` 不读这些值。故障槽不再更新目标。

Pre-op 里把 TxPDO 写成下面这张表。每条都不超过 8 字节，传输类型都是 1（每个 SYNC）：

| PDO | COB-ID | 字节 | 对象 |
| --- | --- | --- | --- |
| TxPDO1 | `0x180+id` | 6 | `0x6041` u16 状态字，`0x6064` i32 实际位置 plus |
| TxPDO2 | `0x280+id` | 8 | `0x2240` i32 电机端编码器 plus，`0x20A0` i32 负载端编码器 plus |
| TxPDO3 | `0x380+id` | 8 | `0x606C` i32 速度 plus/s，`0x6077` i16 扭矩千分比，`0x6078` i16 电流千分比 |
| TxPDO4 | `0x480+id` | 8 | `0x6079` u32 母线 mV，`0x60F4` i32 跟随误差 plus |

`0x6061` 是 8 bit。EtherCAT 映射它时还要再补一个 8 bit 才能对齐，CANopen 这条链路不把它放进 PDO。下面几项走慢速 SDO，大约每轴 250 ms 读一个，不跟每个 SYNC：

| 对象 | 含义 |
| --- | --- |
| `0x603F` u16 | 错误码 |
| `0x3B68` u32 | 警告码 |
| `0x22A2` i16 | 驱动温度 °C。手册标了 PDO mapping NO |
| `0x6061` i8 | 模式显示。8 是 CSP |
| `0x2241` i32 | 双编码器差值 plus。可选，用来看回差，不是刚度计算 |
| `0x6075` u32 | 额定电流 mA。配置时读，失败就在慢速 SDO 里重试。读到 0 仍算无效 |
| `0x6076` u32 | 额定扭矩 mN·m。同样。各轴分开缓存 |

使能状态不是单独的对象：`(0x6041 & 0x006F) == 0x0027`。

物理量用负载端编码器计数。一圈是 `CAN_Dx_EROB_RES`（默认 524288）。`CAN_Dx_EROB_GEAR` 是这一个驱动上各轴共用的电机转 / 输出转。默认 0，表示减速比未知，电机转速不算。`GEAR` 和 `RES` 都不用重启。全名 `CAN_D2_EROB_GEAR` 正好 16 个字符，`CAN_D2_EROB_RES` 是 15 个。

| 量 | 公式 |
| --- | --- |
| 输出角度 ° | `(0x6064 - CTR) * 360 / RES`。各轴共用 `CTR` |
| 输出转速 rpm | `0x606C * 60 / RES` |
| 电机转速 rpm | 输出转速 × `GEAR`，仅当 `GEAR > 0`。eRob142 的 120 减速比填 121 |
| 电流 A | `0x6078 * 0x6075 / 1e6`。6078 是额定电流的千分比，6075 是 mA |
| 扭矩 N·m | `0x6077 * 0x6076 / 1e6`。6076 是 mN·m |
| 母线 V | `0x6079 / 1000` |

绕组温度仍然没有单独对象。电机端编码器 `0x2240` 不拿来算转速。

日志（10 Hz，正在记录时）。DataFlash 一条消息最多 16 列，`ERB1` 已经满了，所以换算结果在同一次记录里的 `ERB3`，原始值仍留在 `ERB1`。额定值还没读到、或 `GEAR` 为 0 时，对应的列是 NaN，不是 0。

| 消息 | 内容 |
| --- | --- |
| `ERB1` | 原始对象：状态字、模式、使能、位置 plus、两个编码器、速度 plus/s、扭矩千分比、电流千分比、母线 mV、跟随误差 |
| `ERB2` | 错误码、警告码、驱动温度、双编码器差值 |
| `ERB3` | 输出角度、输出 rpm、电机 rpm、电流 A、扭矩 N·m、母线 V |

Mission Planner 回放时画 `ERB1`（原始）和 `ERB3`（物理单位）。`Sv` 是 SERVO 通道号。`ERB1` 的母线列是 mV，倍率显示成 V；`ERB3.V` 直接是伏特。

同一组数也会以大约 1 Hz 发给地面站。名字是 `ER` + SERVO 号 + 一个字母，例如掩码包含 SERVO5 时：

| 名字 | 对象 | 类型 |
| --- | --- | --- |
| `ER5P` | `0x6064` 实际位置 plus | NAMED_VALUE_INT |
| `ER5G` | 相对 `CTR` 的输出角度 °。0 是 `CTR`。`P` 仍是原始 plus | NAMED_VALUE_FLOAT |
| `ER5M` | `0x2240` 电机编码器 plus | NAMED_VALUE_INT |
| `ER5L` | `0x20A0` 负载编码器 plus | NAMED_VALUE_INT |
| `ER5S` | `0x606C` 速度 plus/s | NAMED_VALUE_INT |
| `ER5R` | 输出转速 rpm | NAMED_VALUE_FLOAT |
| `ER5N` | 电机转速 rpm。`GEAR` 为 0 时不发 | NAMED_VALUE_FLOAT |
| `ER5I` | `0x6078` 电流千分比 | NAMED_VALUE_INT |
| `ER5C` | 电流 A。额定电流无效时不发 | NAMED_VALUE_FLOAT |
| `ER5Q` | 扭矩 N·m。额定扭矩无效时不发。千分比只留在 `ERB1.Tq` | NAMED_VALUE_FLOAT |
| `ER5U` | `0x6079` 母线电压，单位 V | NAMED_VALUE_FLOAT |
| `ER5F` | `0x60F4` 跟随误差 plus | NAMED_VALUE_INT |
| `ER5T` | `0x22A2` 驱动温度 °C | NAMED_VALUE_FLOAT |
| `ER5E` | `0x603F` 错误码 | NAMED_VALUE_INT |
| `ER5W` | `0x3B68` 警告码 | NAMED_VALUE_INT |
| `ER5O` | `0x6061` 模式 | NAMED_VALUE_INT |
| `ER5D` | `0x2241` 双编码器差值 | NAMED_VALUE_INT |
| `ER5A` | 使能，1 或 0 | NAMED_VALUE_INT |

Mission Planner 的 MAVLink Inspector 里看 `NAMED_VALUE_INT` 和 `NAMED_VALUE_FLOAT`。这些帧同时记成 `NVI` / `NVF`。还没收到的对象不会发。四个舵机就是四组名字，对象表相同。

四轴、每轴四个 TxPDO、4 ms 一拍，大约是 4000 个 TxPDO/s，再加上 RxPDO 和 SYNC。1 Mbps 上还放得下。八个槽都接上会更紧。

使能后 500 ms 内仍下发对齐过的实际位置，不发新目标（手册 §5.1.1）。下使能与使能间隔小于 300 ms 时 `erob_disable()` 直接返回失败（手册 §5.1.2）。位置命令单位是 plus。输出角度是 `(0x6064 - CTR) * 360 / RES`，输出转速是 `0x606C * 60 / RES`，都不乘减速比。减速比只用于电机转速。

每个 Node-ID 有自己的 `canopen_master_t`。多轴共用这一路时，只有第一轴 `sync_enabled`，其余关掉，否则总线上会出现多个 SYNC。

## 目录

```text
libraries/AP_eRobCANOpen/
  README.md  DESIGN.md  CUSTOMER_ASK.md  Makefile
  AP_eRobCANOpen.h  AP_eRobCANOpen.cpp  AP_eRobCANOpen_config.h
  erob_config.h  can_frame.h  canopen_master.h  erob_servo.h
  src/     canopen_master.c  erob_servo.c
  host/    can_bus_stub.c  main.c
```

## 主机验证

不需要配置整棵 ArduPilot 树：

```bash
make -C libraries/AP_eRobCANOpen check
```

通过时打印一行 `OK CAN2 iface 1 @ 1000000 bps, SYNC 4000 us, ...`，进程退出码为 0。桩会应答手册 §5.5.1 的 SDO，并检查 NMT、`0x1006 = 4000`、PDO 映射、CiA402 使能和 500 ms 位置保持。

`src/*.c` 不在 waf 的顶层扫描里。`AP_eRobCANOpen.cpp` 在宏打开时把它们包含进去，避免和 host 的 Makefile 各编一份再链到同一个程序。宏为 0 时这个 cpp 是空的翻译单元。

## 联调（有舵机之后）

1. 先补齐 `CUSTOMER_ASK.md`：型号与轴数、每轴 Node-ID、CSP 还是 CSV、EDS、4 ms 是否允许先放宽、位置单位和限位、样机。
2. 参数按上面的例子设置后重启。eRob 只接 CAN2，1 Mbps，两端终端电阻。CAN1 维持 DroneCAN。
3. 驱动线程会重试 `erob_configure_pdo_csp()`，直到伺服就绪（手册：较新固件大约 5 s）。失败时约每 5 s 再试一次。
4. 使能后至少 500 ms 再 `set_target_pos()`。
5. 用 CAN 分析仪看 CAN2：配置阶段没有 SYNC；运行后每 4 ms 有 `0x080`，以及本节点的 `0x200+id` 和 `0x180+id`、`0x280+id`、`0x380+id`、`0x480+id`。慢速 SDO 在 `0x600+id` / `0x580+id`，不是每个 SYNC 都有。CAN1 上不应出现这些帧。CAN2 上不要再挂别的协议。

出厂 Node-ID、出厂 PDO 和减速比仍未在手册里写死。驱动会按 §5.5.1 强制重映射，但联调前仍要以客户 EDS 或 SDO 读回为准。
