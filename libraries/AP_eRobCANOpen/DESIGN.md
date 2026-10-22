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
| 协议号 | `CAN_Dx_PROTOCOL = 15` |
| CAN1 | DroneCAN，本模块不占用 |
| CAN2 | CANopen 主站，其余 11-bit ID 为标准 CAN |

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

| PDO | 通信参数 | 映射参数 | COB-ID |
| --- | --- | --- | --- |
| RxPDO1 | `0x1400` | `0x1600` | `0x200 + Node-ID` |
| TxPDO1 | `0x1800` | `0x1A00` | `0x180 + Node-ID` |

§5.5.1（示例 Node-ID = 1）的映射，驱动对当前 Node-ID 各写一遍：

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

## 7. 总线

1. 只发 11-bit 帧。扩展帧和 CAN FD 帧直接丢掉。
2. CANopen 占用 NMT `0x000`、SYNC `0x080`，以及本 Node-ID 的 EMCY、四个 PDO、SDO 和心跳。`erob_id_is_reserved()` 是同一份判断。
3. 其余 11-bit ID 进 classic 队列，不交给主站。`CAN_Dx_PROTOCOL2` 不用，原因见 README。
4. `add_interface()` 拒绝 iface 1 以外的口，所以 CAN1 上不会调用 `can_bus_send`。
5. 多轴时全总线只有一个 SYNC 生产者。一个驱动实例只服务一个 Node-ID。

## 8. 仍要客户确认

- 出厂 Node-ID，以及是不是已经是 1 Mbps
- 是否接受驱动按 §5.5.1 强制重映射（需要 EDS 或现场读回）
- 轴数与每个 Node-ID
- 模式是 CSP 还是 CSV；CSV 的 PDO 还没映射
- 4 ms 是否允许先用 10 ms
- plus 到角度的减速比、软限位由谁做
- 样机、电源和联调方式
