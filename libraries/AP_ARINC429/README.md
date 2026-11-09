# CS35 ARINC429 in ArduPilot

Flight-controller side of appendix B from the CS35 engine electrical ICD.
The codec builds and inspects 32-bit logical words. It does not command an
engine, a servo, or a motor. A TCP client can carry bytes to an EBYTE
NT1-M. Those bytes are not ARINC words until framing is opted in.

This branch is cut from `0ac57cc5` (`AP_eRobCANOpen: restore CSP slew and
back off fault re-enable`, pull request 3). eRob CANopen on CAN2 is
unchanged. `AP_EROB_CANOPEN_ENABLED` stays set on ZeroOneX6.

## What runs where

| Path | Status |
| --- | --- |
| Word layout, odd parity, BNR scaling, ranges, SM, SDI, reserved bits | Host-tested. Compiled into the ZeroOneX6 helicopter image when `AP_ARINC429_ENABLED` is 1. |
| 12.5 ms FC transmit schedule | Host-tested. `AP_ARINC429::update()` polls it with `AP_HAL::micros()`. |
| Per-label freshness timeout | Host-tested. Default is off. 24 ms is the ECU's nominal refresh, not the timeout. |
| Command-word levels and rising-edge detector | Host-tested. Levels are not edges. |
| Simulation inject / simulation transmit snapshot | In the library. Nothing in the vehicle fills a snapshot from a flight mode. |
| NT1-M TCP client byte pipe | Implemented on the existing `AP_Networking` lwIP stack. Host-tested against a scripted pipe and a local mock TCP server. Not a physical NT1-M. |
| UART429-S `C0` framing | Provisional and off by default (`A429_FRAME=1`). Not confirmed for the supplied converter. |
| Supplied converter RS422 format | Blocked. No RS422 document for that converter has been applied. |
| Flight-controller IP, PROFINET sharing, physical 429 bit timing | Not changed and not proven by these tests. |
| Hardware loopback, flashing, engine actuation | Not done. |

`A429_ENABLE` defaults to 0, so a ZeroOneX6 image that contains the codec
still does not schedule words until that parameter is set. With the
parameter at 0 the rest of the `A429_` group is hidden, which is the
normal ArduPilot enable-parameter behaviour. Set `A429_ENABLE` to 1 and
reboot to see the other parameters.

## Board

The helicopter target is the waf board `ZeroOneX6`.

* `APJ_BOARD_ID` is `AP_HW_ZeroOne_X6`.
* `Tools/AP_Bootloader/board_types.txt` assigns that name the value **5600**
  (IDs 5600-5699 are reserved for ZeroOne).
* The image to build is `arducopter-heli` (`./waf heli`), not the
  multicopter `arducopter` image. Both images share the ArduCopter
  libraries; only the heli image is the CS35 target.
* ZeroOneX6-ODID includes this hwdef, so it inherits the same compile flag.
  It is not a separate protocol target.

The X6 has one Ethernet port. A future PROFINET gateway needs that same
port. This library does not take a network interface, a socket, or a
ChibiOS MAC driver. A later carrier has to share the port. It must be
added as another `a429_transport_t`, not by editing the word codec.

## Word layout

Appendix B numbers bits D1..D32. In the `uint32` word, D1 is the least
significant bit.

| Field | Bits | Rule |
| --- | --- | --- |
| Label | D1..D8 | Octal label stored as that integer. 0144 is `0x64`. |
| SDI | D9..D10 | Default 00. `A429_SDI` selects the value that counts as fresh. |
| Reserved | D11..D13 | Must be 0. |
| Data | D14..D29 | Signed int16, two's complement. D29 is the sign. |
| SM | D30..D31 | 00 failure, 01 non-computed, 10 functional test, 11 normal. |
| Parity | D32 | Odd parity over all 32 bits. |

The octal label is not bit-reversed here. The ICD numbers the label field
as D1..D8 and does not describe a second reversal. A converter PHY that
presents labels reversed on the wire has to undo that in its own carrier.
That carrier does not exist yet.

Only SM 11 produces a fresh engineering value or a fresh discrete word.
The other three SM values are preserved (raw word, SM, time of receipt,
counters) and do not refresh `valid_us`. A parity error, a reserved bit,
an unexpected SDI, or a BNR value outside the appendix range does the same.
Invalid words still increment real `uint32` counters. Those counters are
not per-word flag bits.

BNR scaling uses an integer denominator (`0.01` is 100, `0.1` is 10) and
rejects NaN, infinity, and values that do not fit in int16. Range limits
are applied to the rounded raw count, from the appendix table.

## Phase one and the rest of appendix B

Phase one is the set this library tracks. It is not the whole contract.

Receive, tracked:

| Octal | Name | Kind | Resolution | Range |
| --- | --- | --- | --- | --- |
| 144 | Ng | BNR | 0.01 % | 0 .. 120.5 |
| 145 | Np | BNR | 0.01 % | 0 .. 150 |
| 147 | T45 | BNR | 0.1 °C | -65 .. 1060 |
| 151 | Mkp | BNR | 0.1 Nm | -30 .. 800 |
| 175 | Warn | discrete | raw int16 | bit names not in the supplied images |
| 176 | State | discrete | raw int16 | bit names not in the supplied images |
| 177 | Warn2 | discrete | raw int16 | bit names not in the supplied images |

Transmit, built only from a simulation snapshot:

| Octal | Name | Kind | Resolution | Range in the appendix |
| --- | --- | --- | --- | --- |
| 310 | CLP | BNR | 0.01 V | none stated |
| 311 | Nr | BNR | 0.01 % | none stated |
| 340 | PCMD1 | discrete | table B.17 | command levels |
| 342 | PCMD2 | discrete | table B.17 | same layout as 340 |
| 344 | PCMD3 | discrete | table B.17 | same layout as 340 |

Labels that are in the appendix and are not phase-one channels:

| Octal | Name | Direction | Notes |
| --- | --- | --- | --- |
| 146 | Ngc | ECU to FC | BNR 0.01 %, 0 .. 150. Encoded by the codec, not a freshness channel. |
| 152 | P3 | ECU to FC | BNR 0.0001 MPa, 0.02 .. 1.6 |
| 153 | CLP | ECU to FC | Range -6 .. 105 marked as airframe-dependent. Resolution is "—" so it is not scaled. |
| 154 | LGt | ECU to FC | BNR 0.01 °, -28 .. 88 |
| 155 | T1 | ECU to FC | BNR 0.1 °C, -65 .. 105 |
| 156 | P0 | ECU to FC | BNR 0.0001 MPa, 0 .. 0.14 |
| 160 | NgAc | ECU to FC | BNR 0.01 %, 0 .. 120.5 |
| 164 | CmdDiffFlt | ECU to FC | discrete, bit table not in the supplied images |
| 165 | ASigFlt | ECU to FC | discrete |
| 166 | BSigFlt | ECU to FC | discrete |
| 167 | SigDiffFlt | ECU to FC | discrete |
| 170 | Swi | ECU to FC | discrete |
| 171 | Swo | ECU to FC | discrete |
| 172 | SigFlt | ECU to FC | discrete |
| 173 | ECUFlt1 | ECU to FC | discrete |
| 174 | ECUFlt2 | ECU to FC | discrete, includes 429/1553 fault bits in the appendix note |
| 200 .. 220 | LIFE | ECU to FC | life / timer words. The span is recorded. Individual meanings are not. |
| 312 | P0 | FC to ECU | BNR 0.01 kPa, no range stated |
| 313 | T0 | FC to ECU | BNR 0.1 °C |
| 314 | H | FC to ECU | BNR 1 m |
| 316 | TAS | FC to ECU | BNR 0.1 m/s |
| 317 | Pm | FC to ECU | BNR 0.0001 MPa |
| 320 | Tm | FC to ECU | BNR 0.1 °C |
| 321 | Hour | FC to ECU | resolution 1 |
| 322 | Minute | FC to ECU | resolution 1 |
| 323 | Second | FC to ECU | resolution 1 |
| 324 | Year | FC to ECU | resolution 1 |
| 325 | Month | FC to ECU | resolution 1 |
| 326 | Day | FC to ECU | resolution 1. The appendix name is "Data" (date). |
| 331 | State331 | FC to ECU | discrete. See table B.16 below. Not decoded. |

Ingesting a non-phase-one label increments `rx_untracked` and stores that
raw word. It does not refresh Ng, Np, T45, torque, or the warn words.

### Table B.17 (labels 340, 342, 344)

D16..D14 is one 3-bit code, with D16 as the most significant bit:

| Code | Meaning |
| --- | --- |
| 001 | stop |
| 011 | idle |
| 100 | flight |
| anything else | unknown. The word is not encoded. |

| Bit | Snapshot field | How it is used |
| --- | --- | --- |
| D17 | `cold_run` | level. The ECU treats 0 to 1 as the command. |
| D18 | `false_start` | level. 0 to 1 at the ECU. |
| D19 | `rotor_brake` | level, 1 active |
| D20 | `on_ground` | level, 1 ground, 0 air |
| D21 | `power_assurance` | level, 1 active |
| D22 | `channel_switch` | level. 0 to 1 at the ECU. |
| D23 | `overspeed_a` | level, 1 active |
| D24 | `overspeed_b` | level, 1 active |

The snapshot uses those level names. Rising edges are a separate result
from `a429_edge_update()`. Each of 340, 342 and 344 has its own detector.
The first valid sample only arms the detector, so a high bit at startup
is not a command. An invalid sample (bad parity, bad SM, bad SDI, reserved
bits) does not move that detector and does not report an edge.

Transmit copies the supplied levels into the word. It does not turn a
snapshot into a one-frame pulse, and the vehicle never supplies a level
from a flight mode. Calling `sim_set_pcmd()` is a simulation input.

### Table B.16 (label 331, not implemented)

Recorded from the appendix image so the bits are not dropped. The codec
does not name these fields.

| Bit | Description | Note |
| --- | --- | --- |
| D14 | fuel pressure low | 1 active |
| D15 | oil pressure low | 1 active |
| D16 | metal chip | 1 active |
| D17 | fuel-filter impending blockage | 1 active |
| D18 | fire warning | 1 active |
| D19 | starter motor on | 1 active |
| D20 | ignition on | 1 active |

Warn, state and warn2 (175, 176, 177) are the same situation in the other
direction: the supplied images do not include tables B.3 through B.15, so
the library keeps the raw int16 and the validity bits and does not invent
bit names.

## Schedule and timeouts

FC to ECU uses a single period of 12500 microseconds from a monotonic
`uint32` microsecond clock. The first poll only arms the next deadline.
Each later poll emits at most one slot.

* On time, or late by less than one period: the next deadline stays on
  the 12500 microsecond grid.
* Late by one period or more: one slot is emitted, the missed slots are
  counted in `tx_slots_skipped`, and the next deadline is one period
  after now. There is no catch-up loop and no 12 ms / 13 ms alternation.

`AP_ARINC429::update()` is scheduled at 400 Hz so the slot is not tied to
the 50 Hz EFI task. A vehicle loop slower than 80 Hz will skip slots
instead of bursting.

Each phase-one receive label has its own timeout parameter, in
milliseconds, default 0. Zero means "do not time out". A non-zero value
is the operator's policy. The appendix's 24 ms ECU refresh is
`A429_ECU_NOMINAL_REFRESH_US` and is not copied into those parameters.
Only a valid word for that label moves `valid_us`. A timeout fires once
when the age reaches the policy, then waits for a new valid word.

## NT1-M TCP client

The NT1-M ("超级网口") is a TTL-UART to Ethernet adapter. It is not the
ARINC429 converter. The converter's TTL UART is what the module carries.
The manual (V1.2) factory example is TCP server `192.168.3.7:8887`, serial
115200 8N1, a 1024-byte serial buffer and a 512-byte packing length. It
does not state a packing timeout. Those values are the printed defaults,
not the settings of a delivered module, and this firmware does not write
them. Heartbeat, registration, and "send the serial cache on connect" are
module options. If cache-forward is left enabled, the module can still
emit bytes it held before the TCP session. This client cannot disable that.

The flight controller is a TCP client. `A429_IP0`..`A429_IP3` and
`A429_REM_PORT` are the peer. Any zero address or a zero port leaves the
socket closed. `A429_LOC_PORT` is not bound. The client uses an ephemeral
local port. The manual example `192.168.3.7:8887` is not the parameter
default. The flight-controller address is whatever `NET_*` already has.
This code does not write `NET_IP`, `NET_NETMASK`, `NET_GW`, or `NET_DHCP`,
and it does not restart the MAC. A belief that the bench network is
`192.168.144.x` is not verified here.

ZeroOneX6 has `ETH1`, so the hwdef turns on `AP_NETWORKING_ENABLED` and
`HAL_USE_MAC`. `AP_Networking` already owns that interface and lwIP.
`SocketAPM` is the socket API on that stack. `NET_ENABLE` still defaults
to 0. The ARINC worker calls `AP::network().get_ip_active()` and does not
open a socket while that address is 0. `startup_wait()` is not used,
because on ChibiOS it blocks until an address exists. The worker waits
the same way, off the vehicle thread.

`connect_timeout(..., 200)` can block the worker for 200 ms. It is not
called from `AP_ARINC429::update()`. After connect, send and receive are
non-blocking. Retries wait 200 ms, then double, and stop growing at 5 s.
One attempt is started per poll. The TX hold is 32 bytes, one slot, and
a new slot is dropped while that hold is unfinished. RX is 64 bytes. A
full RX buffer stops the socket read and counts backpressure. Disconnect
and peer change discard the TX hold. Nothing is replayed on the next
connect. The 12500 microsecond scheduler is unchanged and still emits one
slot per poll.

What this does not prove: electrical 429 bit timing, NT1-M packing delay,
PROFINET sharing the same RJ45, or the delivered module's mode. UDP is
not implemented. The manual's UDP note (512 bit packets) is not a TCP rule.

## Framing

The TCP pipe and the byte interpretation are separate.

`A429_FRAME` defaults to 0. Socket bytes are counted as
`rx_unframed_bytes` and are not decoded into labels. A due transmit slot
increments `tx_unframed` and is not written to the socket.

`A429_FRAME=1` is an explicit opt-in named for the UART429-S manual, not
for the supplied converter. The vendor said the network product uses their
RS422 format. That sentence does not say the format is UART429-S. No
RS422 document for the supplied converter is in this tree, so this profile
stays off and is not a claim about that wire.

The profile, when opted in, is `0xC0` plus the codec `uint32` in
little-endian order. The first payload byte is the logical label. Octal
label 036 is `0x1E` in this codec. Bit-reversing that byte is `0x78`,
which is the UART429-S manual's example of label 036 on that product's
wire. This profile does not apply that reversal. A `0xCD` byte is skipped
as unknown. UART429-S speed commands are not generated.

Until the converter's RS422 frame, label bit order, and packing are
confirmed, do not treat `A429_FRAME=1` as hardware-ready. Engine, motor,
servo, and eRob outputs are still not fed by these words.

## Vehicle hook

`AP_Vehicle` owns one `AP_ARINC429` when `AP_ARINC429_ENABLED` is 1.

* Parameter group `A429_`, index 34. Existing indices are untouched.
* `setup()` calls `init()`.
* The scheduler calls `update()` at 400 Hz.
* `update()` does not write motors, servos, EFI, or eRob targets.

The compile flag is in `libraries/AP_HAL_ChibiOS/hwdef/ZeroOneX6/hwdef.dat`.
Other boards keep `AP_ARINC429_ENABLED` at 0, so their images do not gain
the codec. `Tools/scripts/build_options.py` lists the flag for the custom
build server, default off.

## Tests

Host, no waf:

```sh
make -C libraries/AP_ARINC429 check
```

That suite hard-codes words that were calculated from the bit table
(Ng 50% is `0xE2710064`), and it checks negative values, both ends of
each phase-one range, all four SM values, parity and reserved-bit faults,
NaN and infinity, command codes, per-label edges, startup and invalid
samples, a missing-data timeout, a clock wrap, and a late tick. The NT1-M
tests add a scripted byte pipe and a local TCP server: invalid peer,
fragmented and coalesced reads, partial writes, reconnect without replay,
peer silence, a full RX buffer, one late scheduler slot, and the
difference between logical label `0x1E` and wire example `0x78`.

SITL repeats the same suite as `libraries/AP_ARINC429/tests/test_arinc429.cpp`
when the board is built with gtest. SITL leaves the feature off, so the
test compiles the codec itself and does not duplicate the firmware objects.

Helicopter image, not flashed:

```sh
./waf configure --board ZeroOneX6
./waf heli
```

The apj `board_id` field is the check for 5600. eRob remains
`AP_EROB_CANOPEN_ENABLED 1` in the same hwdef.
