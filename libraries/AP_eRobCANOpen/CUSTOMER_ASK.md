# 联调前还要向客户要的资料

已经定下来、不必再问的部分：eRob 当舵机用，不是单独的 CANopen 设备。飞控 CAN2 的协议名是 eRob CAN，CiA402 CSP，1 Mbps，周期目标 4 ms。Node-ID 由客户在舵机上写死，飞控不写 `0x100B`。哪个 SERVO 驱动哪个 ID 用 `CAN_Dx_EROB_SV1`..`SV8` 和 `ID1`..`ID8`。`SVn` 全是 0 时才退回 `NODE` + `BM` 的连续编号。CAN1 继续 DroneCAN，默认参数这次不动。

还缺下面这些，映射和限位才能定死：

1. eRob 型号和轴数，以及减速比铭牌。
2. 每个轴在舵机上写死的 Node-ID，以及它要跟哪个 SERVO 输出。飞控只填 `SVn` / `IDn`，不负责改号。
3. 运行模式用 CSP（`0x6060 = 8`）还是 CSV（`0x6060 = 9`）。现在按 CSP。RxPDO1 仍是控制字 + 目标位置。输出角度和输出 rpm 按 524288 plus/圈换算。电机 rpm 要 `CAN_Dx_EROB_GEAR`。绕组温度没有单独对象。
4. EDS，或者书面确认可以按手册 §5.5.1 在每次上电时强制重写 PDO。
5. 4 ms 是否必须。若总线不稳，实现上可以先把 `EROB_SYNC_PERIOD_US` 改成 10000。
6. 位置单位和限幅：plus 还是角度，软限位做在飞控还是舵机。
7. 联调条件：有没有实机和电源，现场还是寄样。
8. CAN2 在 `CAN_D2_PROTOCOL = 15`（eRob CAN）时整口只给这组舵机。不要把 DroneCAN 或其它 CAN 设备留在这一路。默认参数仍不打开它。
9. 传统直升机：斜盘 SERVO1..3（功能 33/34/35）和尾舵 SERVO4（功能 36）各自的 Node-ID。60° 行程、中位 ±30° 时共用 `MIN=-43691`、`MAX=43691`。飞行前用 `H_SV_MAN` 对方向。`SVn` 全 0 时仍可用 `BM`。

可以直接转发：

> eRob 当舵机接在飞控 CAN2（协议 eRob CAN，CSP，1 Mbps，目标 4 ms），CAN1 的 DroneCAN 不动。Node-ID 请在舵机上写死，飞控不改 0x100B。请补充型号和轴数、每个已写死的 Node-ID 对应哪个 SERVO、CSP 还是 CSV、EDS、4 ms 能否先放宽、每轴 plus 行程和限位，以及联调样机。
