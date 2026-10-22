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
```

`15` 是 `AP_CAN::Protocol::eRobCAN`，参数列表显示为 `eRob CAN`。`CAN_D2_EROB_NODE` 刚好 16 个字符。一个 `CAN_Dx_PROTOCOL` 独占那个虚拟驱动：1 是 DroneCAN，15 是本主站。驱动只接受 HAL 接口 1（CAN2）。节点号超出 1..127 时按 1 处理。

选了 15 之后，这一路只跑 eRob CAN。主站使用 NMT `0x000`、SYNC `0x080`，以及该节点的 EMCY、PDO、SDO 和心跳。同一接口上的其它帧被丢掉，不再交给第二个协议。CAN1 保持 `CAN_D1_PROTOCOL 1`。位置目标用 `set_target_pos()`，单位是 plus；使能后 500 ms 内主站仍保持实际位置。

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
5. 用 CAN 分析仪看 CAN2：配置阶段没有 SYNC；运行后每 4 ms 有 `0x080`，以及本节点的 `0x200+id` / `0x180+id`。CAN1 上不应出现这些帧。CAN2 上不要再挂别的协议。

出厂 Node-ID、出厂 PDO 和减速比仍未在手册里写死。驱动会按 §5.5.1 强制重映射，但联调前仍要以客户 EDS 或 SDO 读回为准。
