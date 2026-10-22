# AP_eRobCANOpen

零差云控 eRob（CANopen / CiA402）的最小主站。目标机是零一 X6 Ultra，仓库里的板型是 `libraries/AP_HAL_ChibiOS/hwdef/ZeroOneX6`。

本目录是可选模块：源码放在 `src/` 和 `host/`，不在库顶层，因此 waf 不会把它链进 Copter / Plane / Rover。飞控默认行为不变，CAN1 上的 DroneCAN 也不改。

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
| CAN2 | PB12 / PB13 | 本主站，只接 eRob |

`ZeroOneX6/defaults.parm` 里 `CAN_P1_DRIVER 1` 且 `CAN_P2_DRIVER 1`：两个物理口都交给虚拟驱动 1，而驱动 1 的默认协议是 DroneCAN。这次提交不改这份默认参数，所以现有固件仍是两路 DroneCAN。

以后若要把 CAN2 交给本驱动，只改集成配置，不改 CAN1：

1. `CAN_P1_DRIVER` 保持 1（DroneCAN 留在 CAN1）。
2. `CAN_P2_DRIVER` 设为 0，避免 DroneCAN 再占用 CAN2。
3. 胶水层打开 HAL 接口 `erob_hal_can_iface()`（CAN2 对应 1，不是 0），比特率 `EROB_CAN_BITRATE`（1 Mbps），只发 11-bit 标准帧。

`can_bus_send` / `can_bus_recv` / `can_bus_millis` 是预留接缝。host 桩实现了它们。飞控侧的 HAL 胶水还没写，也还没接到主循环。

## 协议范围

依据《eRob CANopen and EtherCAT 用户手册 V1.9》。只做 eRob 这一条链路：

| 做 | 不做 |
| --- | --- |
| NMT Stop / Reset communication / Start | 节点守护、LSS、完整对象字典 |
| 快速 SDO 读写 | 分段 SDO、EDS 解析 |
| 主站发 SYNC `0x080` | 多个 SYNC 源 |
| 按手册 §5.5.1 写死 RxPDO1 / TxPDO1 | 通用 PDO 映射引擎 |
| CiA402：清故障 → 6 → 7 → `0x000F` | PP/CSV/CST 的完整工具链 |

默认：Node-ID `1`（占位）、CSP（`0x6060 = 8`）、SYNC 周期 4000 µs。手册 §5.5.1 把 `0x1006 = 1000` 写成 1000 µs，所以这里按微秒写入 4000。从站的 `0x1005` 写成 `0x00000080`，禁止从站自己发 SYNC。

周期顺序与手册示例一致：先 RxPDO1（`0x200+Node-ID`，控制字 + 目标位置），再 SYNC，从站回 TxPDO1（`0x180+Node-ID`，状态字 + 实际位置）。

使能后 500 ms 内仍下发对齐过的实际位置，不发新目标（手册 §5.1.1）。下使能与使能间隔小于 300 ms 时 `erob_disable()` 直接返回失败（手册 §5.1.2）。位置单位是 plus；负载端单圈 524288（手册 §5.2.1）。换成角度还要减速比，见 `CUSTOMER_ASK.md`。

一个 `canopen_master_t` 对应一个 Node-ID，并且会发 SYNC。多轴共用 CAN2 时，只让其中一个实例 `sync_enabled`，其余关掉，否则总线上会出现多个 SYNC。

## 目录

```text
libraries/AP_eRobCANOpen/
  README.md  DESIGN.md  CUSTOMER_ASK.md  Makefile
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

确认 waf 目前扫不到这些源文件（顶层没有 `.c` / `.cpp`，和现有的纯头文件库一样）：

```bash
find libraries/AP_eRobCANOpen -maxdepth 1 \( -name '*.c' -o -name '*.cpp' \)
```

没有输出就是预期结果。整树 `./waf copter` 很重，而且本模块还没进固件，那次编译证明不了本驱动。

## 联调（有舵机之后）

1. 先补齐 `CUSTOMER_ASK.md`：型号与轴数、每轴 Node-ID、CSP 还是 CSV、EDS、4 ms 是否允许先放宽、位置单位和限位、样机。
2. 只把 eRob 接到 CAN2，1 Mbps，两端终端电阻。CAN1 维持原来的 DroneCAN。
3. 上电后等伺服就绪（手册：较新固件大约 5 s）再调用 `erob_configure_pdo_csp()`。
4. `erob_enable()` 后至少 500 ms 再 `erob_set_target_pos()`。
5. 用 CAN 分析仪看 CAN2：配置阶段没有 SYNC；运行后每 4 ms 有 `0x080`，以及本节点的 `0x200+id` / `0x180+id`。CAN1 上不应出现这些 11-bit 帧。

出厂 Node-ID、出厂 PDO 和减速比仍未在手册里写死。驱动会按 §5.5.1 强制重映射，但联调前仍要以客户 EDS 或 SDO 读回为准。
