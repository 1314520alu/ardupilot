# 联调前还要向客户要的资料

已经定下来、不必再问的部分：eRob 当舵机用，不是单独的 CANopen 设备。飞控 CAN2 的协议名是 eRob CAN，CiA402 CSP，1 Mbps，周期目标 4 ms。连续 Node-ID 映射到连续 SERVO 通道。CAN1 继续 DroneCAN，默认参数这次不动。

还缺下面这些，映射和限位才能定死：

1. eRob 型号和轴数，以及减速比铭牌。
2. 每个轴的 Node-ID。驱动里的默认值 1 只是占位。
3. 运行模式用 CSP（`0x6060 = 8`）还是 CSV（`0x6060 = 9`）。现在按 CSP，PDO 是手册 §5.5.1 的控制字 + 目标位置。
4. EDS，或者书面确认可以按手册 §5.5.1 在每次上电时强制重写 PDO。
5. 4 ms 是否必须。若总线不稳，实现上可以先把 `EROB_SYNC_PERIOD_US` 改成 10000。
6. 位置单位和限幅：plus 还是角度，软限位做在飞控还是舵机。
7. 联调条件：有没有实机和电源，现场还是寄样。
8. CAN2 在 `CAN_D2_PROTOCOL = 15`（eRob CAN）时整口只给这组舵机。不要把 DroneCAN 或其它 CAN 设备留在这一路。默认参数仍不打开它。
9. 每轴的 SERVO 通道号、Node-ID，以及 `SERVOx_MIN` 到 `SERVOx_MAX` 对应的 plus 行程。驱动里各轴暂时共用 `CAN_D2_EROB_MIN` / `MAX`。

可以直接转发：

> eRob 当舵机接在飞控 CAN2（协议 eRob CAN，CSP，1 Mbps，目标 4 ms），CAN1 的 DroneCAN 不动。请补充型号和轴数、每轴 Node-ID 和对应的 SERVO 通道、CSP 还是 CSV、EDS、4 ms 能否先放宽、每轴 plus 行程和限位，以及联调样机。
