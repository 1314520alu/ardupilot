# eRob CANopen / CiA402 设计摘录

来源：零差云控《eRob CANopen and EtherCAT 用户手册 V1.9》。手册写明功能按 CiA DS 301 与 DSP 402。标成「需客户确认」的项，摘录里没有写死，不能靠猜测填进飞控。

实现对应 `src/canopen_master.c` 与 `src/erob_servo.c`。默认物理口是 CAN2，见 `erob_config.h`。

## 1. 角色

| 项 | 取值 |
| --- | --- |
| 主站 | `AP_eRobCANOpen`，只绑定 HAL CAN iface 1 |
| 从站 | eRob 关节 |
| 比特率 | 1 Mbps（`CAN_P2_BITRATE`） |
| 周期 | 4 ms（4000 µs） |
| 协议号 | `CAN_Dx_PROTOCOL = 15`（`eRobCAN`，显示 `eRob CAN`） |
| 用途 | 多轴舵机，CiA402 CSP。不是通用 CANopen 工具 |
| CAN1 | DroneCAN，本模块不占用 |
| CAN2 | 协议 15（eRob CAN）时整口归该主站 |

手册 CSP 示例把通信周期设成 1 ms（`0x1006 = 1000`）。4 ms 仍是该对象的整数微秒值。不稳定时可以先把 `EROB_SYNC_PERIOD_US` 改成 10000，不必去动 CAN1。

## 2. NMT 与上电

上电初始化结束后从站在 Pre-Operational：可以 SDO，不能 PDO。NMT Start 之后进入 Operational。

NMT COB-ID `0x000`，byte0 是命令，byte1 是 Node-ID（0 表示所有节点）。

| 命令 | 值 | 本驱动 |
| --- | --- | --- |
| Start | `0x01` | 映射完成后发送 |
| Stop | `0x02` | 配置开始时发送 |
| Enter pre-op | `0x80` | 保留常量，配置流程未用 |
| Reset node | `0x81` | 保留常量 |
| Reset communication | `0x82` | 配置时发送，并等待 Boot-up |

Boot-up：COB-ID `0x700 + Node-ID`，数据 `0x00`。心跳走同一 COB-ID，但数据不是 0，本驱动不当成 Boot-up。

`erob_configure_pdo_csp()` 的顺序：

1. 调用方保证伺服已经上电就绪（手册：较新固件约 5 s）。这一步不在驱动里空转。
2. NMT Stop，NMT Reset communication，等待 Boot-up（默认 2 s）。
3. Pre-op 下写模式、周期和 PDO。
4. NMT Start。
5. 读 `0x6064`，写入 `0x607A`。
6. 才允许本地 SYNC。

## 3. SYNC

同一网络只允许一个 SYNC 生产者。本主站发 `0x080`，DLC 0。

| 对象 | 作用 | 本驱动 |
| --- | --- | --- |
| `0x1005` | SYNC COB-ID；bit30 = 1 表示本机产生 SYNC | 写 `0x00000080`，从站不产生 |
| `0x1006` | 通信周期 | 写 `sync_period_us`，默认 4000 |
| `0x1007` | 同步窗口 | 不写 |

手册 §5.5.1 的报文 `23 06 10 00 E8 03 00 00` 等于 1000，正文写明 1000 µs，因此按微秒处理。若某版固件的 EDS 单位不同，以 EDS 为准改 `EROB_SYNC_PERIOD_US` 的换算，不要改 CAN 口。

本地节拍目前用毫秒：4000 µs 正好是 4 ms。短于 1 ms 会被抬到 1 ms，注释写在 `co_master_poll_sync()` 里。配置阶段 `sync_enabled` 为假，避免 Pre-op 期间发 SYNC。

## 4. 固定 PDO

默认 COB-ID（手册表 3-10）：

| PDO | 通信参数 | 映射参数 | COB-ID | 映射 |
| --- | --- | --- | --- | --- |
| RxPDO1 | `0x1400` | `0x1600` | `0x200 + Node-ID` | `0x6040` u16 + `0x607A` i32 |
| TxPDO1 | `0x1800` | `0x1A00` | `0x180 + Node-ID` | `0x6041` u16 + `0x6064` i32 |
| TxPDO2 | `0x1801` | `0x1A01` | `0x280 + Node-ID` | `0x2240` i32 + `0x20A0` i32 |
| TxPDO3 | `0x1802` | `0x1A02` | `0x380 + Node-ID` | `0x606C` i32 + `0x6077` i16 + `0x6078` i16 |
| TxPDO4 | `0x1803` | `0x1A03` | `0x480 + Node-ID` | `0x6079` u32 + `0x60F4` i32 |

TxPDO1 与手册 §5.5.1 相同。TxPDO2..4 是反馈，每条正好不超过 8 字节。四条的写法一样：先把 COB-ID 的 bit31 置 1 关掉，传输类型写 1，清空映射，写入子索引，再打开。驱动对当前 Node-ID 各写一遍。

TxPDO1：

- 关闭：`0x1800:01 = 0x80000000 | (0x180+id)`
- 传输类型：`0x1800:02 = 1`（每个 SYNC）
- `0x1A00:00 = 0`
- `0x1A00:01 = 0x60410010`（状态字，16 bit）
- `0x1A00:02 = 0x60640020`（实际位置，32 bit）
- `0x1A00:00 = 2`
- 打开：`0x1800:01 = 0x180+id`

RxPDO1：

- 关闭：`0x1400:01 = 0x80000000 | (0x200+id)`
- 传输类型：`0x1400:02 = 1`
- `0x1600:00 = 0`
- `0x1600:01 = 0x60400010`（控制字，16 bit）
- `0x1600:02 = 0x607A0020`（目标位置，32 bit）
- `0x1600:00 = 2`
- 打开：`0x1400:01 = 0x200+id`

出厂上电是否已经是这张表，手册没有承诺。驱动每次配置都重写。CSV 若要改成 `0x60FF`，需要另一张映射；在 EDS 确认前，速度只存在 `target_vel`，不会塞进这条位置 PDO。

## 5. SDO

主站请求 `0x600 + Node-ID`，从站应答 `0x580 + Node-ID`。只做快速传输：

| 操作 | 命令字 |
| --- | --- |
| 写 1 字节 | `0x2F` |
| 写 2 字节 | `0x2B` |
| 写 4 字节 | `0x23` |
| 读 | `0x40` |
| 写成功 | `0x60` |
| 读 1 / 2 / 4 字节 | `0x4F` / `0x4B` / `0x43` |
| 中止 | `0x80` |

超时默认 100 ms。等待应答时若收到 TPDO、EMCY 或 Boot-up，会先交给回调，避免被 SDO 事务丢掉。载荷按小端写入；STM32 与 host 桩都是小端，其它字节序会在编译期报错。

使能之后的诊断不占用这条阻塞读。`erob_poll_diag()` 大约每 250 ms 发一个上传请求，应答由 `co_master_handle_frame()` 收下。顺序是 `0x603F`、`0x3B68`、`0x22A2`、`0x6061`、`0x2241`，外加还没读到的 `0x6075` / `0x6076`。中止或超时就跳到下一个，不把该轴打成故障。`0x22A2` 手册写明不能做 PDO。`0x6061` 留在 SDO 里，避免为了 8 bit 对齐再塞一个填充对象。

`0x6075`（额定电流，mA）和 `0x6076`（额定扭矩，mN·m）在 Pre-op、NMT Start 之前各读一次。失败或读到 0 不打断 CSP 配置，有效位保持清除，之后用上面的慢速 SDO 重试。读成功且非 0 就不再读。输出角度和输出转速除以 `CAN_Dx_EROB_RES`（默认 524288）。手册 V1.9 对象目录没有 `0x608F`，`0x6093` 标明暂不支持，所以不从驱动读分辨率，也不改写 `RES`。换算见 README。

## 6. CiA402 使能

控制字 `0x6040`，状态字 `0x6041`。bit2（Quick stop）在正常运行时为 1。bit7 上升沿清故障。

| 控制字 | 含义 |
| --- | --- |
| `0x0080` | Fault reset（手册周期示例里的 128） |
| `0x0006` | Shutdown |
| `0x0007` | Switch on |
| `0x000F` | Enable operation |

状态字用掩码 `0x006F`：`0x0040` / `0x0021` / `0x0023` / `0x0027` 分别对应 Switch on disabled、Ready、Switched on、Operation enabled。bit3 为 1 视为故障。

`erob_enable()` 先读 `0x6064` 对齐目标，再按 50 ms 节拍发送 `0x0080 → 0x0006 → 0x0007 → 0x000F`，直到状态字进入 Operation enabled。使能后 500 ms 内周期帧仍带对齐位置。手册示例最后一帧目标位置用过控制字 `0x001F`；CSP 表里 bit4–6 是保留位，周期保持用 `0x000F`。

运行模式 `0x6060 = 8` 为 CSP。切换到位置模式前把实际位置抄到目标位置（手册 §5.3.2）。

## 7. 舵机输出

`SRV_Channels::push()` 在协议为 eRob CAN 时调用 `push_servos()`。Node-ID 在舵机上写死。飞控不写 `0x100B`。

1. 主映射是 `CAN_Dx_EROB_SV1`..`SV8` 和 `CAN_Dx_EROB_ID1`..`ID8`。全名例如 `CAN_D2_EROB_SV1`，15 个字符。`SVn` 为 0 的槽不用。`SERVO<SVn>` 驱动节点 `IDn`。遥测名字是 `ER<SVn>*`，日志 `Sv` 是这个 SERVO 号。最多 8 路。
2. 只要有一个 `SVn` 非 0，槽生效，`NODE` 和 `BM` 忽略。`SVn` 全是 0 时才用位掩码：位定义与 `CAN_Dx_UC_SRV_BM` 相同，bit 0 是 SERVO1，从低到高对应从 `NODE` 起的连续 Node-ID。参数全名最长 16 个字符，所以掩码不叫 `SRV_BM`。
3. 初始化跳过并告警：重复 ID、重复 SERVO 号、ID 不在 1..127、`SVn` 有值但 ID 为 0、SERVO 号超出 1..32。先出现的槽保留。这些参数和 `NODE` / `BM` 一样，重启后生效。
4. PWM 用该通道自己的 `SERVOx_MIN` / `SERVOx_MAX`，线性映到共用的 `MIN` / `MAX`。这两个值是相对 `CTR` 的偏移。写入 `0x607A` 的是 `CTR + 映射值`。`CAN_Dx_EROB_CTR` 是所有轴共用的一个中心（默认 0，目标和以前一样）。PWM 中点映到 `CTR + (MIN+MAX)/2`。角度是 `(0x6064 - CTR) * 360 / RES`。`GEAR` 和 `RES` 仍共用。PWM 仍为 0，或该通道在舵机禁用掩码里时，保持上一次目标。
5. 状态字 bit3 置位后，地面站发 CRITICAL（`eRob: SERVO<n> id <id> fault 603F=0x....`，5 s 限速，故障还在就一直重复）。控制字先 0 再 `0x0080` 形成故障复位上升沿，然后 `0x0006` / `0x0007` / `0x000F`。仍故障则每 500 ms 再试，不锁死到重启。回到 Operation Enabled 时先把 `0x607A` 写成当前 `0x6064`，再按约 100 °/s（`RES` 换算）在最多 0.6 s 内斜坡回到指令，并报 `eRob: SERVO<n> id <id> recovered`。次数在 `ERB3.Flt`，恢复不清零。心跳或 PDO 超过 200 ms 没有则告警 `silent`；重新出现后发 NMT Start，再使能并做同样的无扰恢复。`push_servos()` 仍更新指令，故障期间不把该指令直接写入 `0x607A`。
6. 输出反转沿用 ArduPilot 舵机通道自己的反向，驱动不再翻一次。直升机斜盘和尾舵只是这些 SERVO 通道的上游，本库不改混控。
7. 解锁检查走 `AP_Arming::can_checks()`。`pre_arm_check()` 只读缓存。已配置槽里只要有一个 TxPDO1 超过 200 ms、状态字 fault、未 Operation Enabled，或 `0x6064` 超出 `CTR+MIN`..`CTR+MAX` 超过约 5°，就拒绝解锁。文案是 `eRob: SERVOn id X not enabled` / `no response` / `fault 603F=0x....` / `pos out of range`。

## 8. 总线

1. 一个 `CAN_Dx_PROTOCOL` 独占该驱动。15 是 eRob CAN，1 是 DroneCAN。不要在同一驱动上再挂第二种协议。
2. 只发 11-bit 帧。扩展帧、CAN FD 帧，以及不属于本节点 CANopen 的 11-bit 帧，读到后丢掉。
3. 主站使用 NMT `0x000`、SYNC `0x080`，以及本 Node-ID 的 EMCY、四个 PDO、SDO 和心跳。
4. `add_interface()` 拒绝 iface 1 以外的口，所以 CAN1 上不会调用 `can_bus_send`。
5. 多轴时全总线只有一个 SYNC 生产者。第一轴发出 SYNC，其余轴不发。

## 9. 仍要客户确认

- 舵机上写死的 Node-ID 是否已经是 1 Mbps，以及每个 ID 对应哪个 SERVO
- 是否接受驱动按 §5.5.1 强制重映射（需要 EDS 或现场读回）
- 轴数。飞控用 SV/ID 槽填写，不从飞控写 0x100B
- 模式是 CSP 还是 CSV；CSV 的 PDO 还没映射
- 4 ms 是否允许先用 10 ms
- plus 到角度的减速比、软限位由谁做
- 样机、电源和联调方式
