# CS35 ARINC429 in ArduPilot

Flight-controller side of appendix B from the CS35 engine electrical ICD.
The codec builds and inspects 32-bit logical words. It does not move a
servo, a motor, or an eRob swash. When enabled, label 310 carries the
helicopter collective output as a voltage, and label 311 carries
measured rotor speed from `AP_RPM`. Labels 312, 313, 314 and 316 carry
ambient pressure, barometer temperature, absolute altitude and sensor
true airspeed when those samples are valid. Labels 321 through 326
carry UTC from GPS or the RTC, all six together or not at all. Labels
317, 320 and 331 stay unsent: this flight controller has no oil
pressure, oil temperature, or accessory discrete inputs. When
`A429_ENG_CH` selects an RC channel it fills labels 340, 342 and 344
with the engine mode. A TCP
client can carry bytes to an EBYTE NT1-M. `A429_FRAME` defaults to 2,
so those bytes use the RS422-429-1C frame: `C0` plus four little-endian
bytes, with only the label byte bit-reversed. `A429_FRAME=0` leaves the
bytes uninterpreted.

This branch is cut from `0ac57cc5` (`AP_eRobCANOpen: restore CSP slew and
back off fault re-enable`, pull request 3). eRob CANopen on CAN2 is
unchanged. `AP_EROB_CANOPEN_ENABLED` stays set on ZeroOneX6.

## What runs where

| Path | Status |
| --- | --- |
| Word layout, odd parity, BNR scaling, ranges, SM, SDI, reserved bits | Host-tested. Compiled into the ZeroOneX6 helicopter image when `AP_ARINC429_ENABLED` is 1. |
| 12.5 ms FC transmit schedule | Host-tested. `AP_ARINC429::update()` polls it with `AP_HAL::micros()`. |
| Per-label freshness timeout | Host-tested. Default is off. A non-normal SM or a parity error clears the getter immediately. 24 ms is the ECU's nominal refresh, not the timeout. |
| Command-word levels and rising-edge detector | Host-tested. Levels are not edges. |
| Simulation inject / simulation transmit snapshot | In the library. `A429_ENG_CH` 0 leaves 340, 342 and 344 alone. A non-zero channel overwrites only 340, 342 and 344. Enabled `update()` writes or clears 310 from the heli collective, 311 from measured Nr, and 312, 313, 314, 316 and 321..326 from the air-data and clock samples below. |
| NT1-M TCP client byte pipe | Implemented on the existing `AP_Networking` lwIP stack. Host-tested against a scripted pipe and a local mock TCP server. Not a physical NT1-M. |
| UART429-S `C0` framing | Provisional (`A429_FRAME=1`), not the default. Sends the codec logical word. Does not bit-reverse the label and does not send a `CD` rate command. |
| RS422-429-1C wire profile | Default (`A429_FRAME=2`). Page 3 `C0` plus four little-endian bytes. Page 4 reverses only the label byte. Host-tested. Not a test of the NT1-M or of the internal TTL board. |
| Flight-controller IP, PROFINET sharing, physical 429 bit timing | Not changed and not proven by these tests. |
| Hardware loopback, flashing, engine actuation | Not done. |

`A429_ENABLE` defaults to 1, so a fresh image that contains the codec
schedules words without a parameter change. `A429_ENG_CH`, `A429_NR_REF`,
and the peer address and port stay at 0, so labels 340, 342 and 344 stay
on the simulation snapshot and the socket stays closed. Setting
`A429_ENABLE` to 0 hides the rest of the `A429_` group after reboot,
which is the normal ArduPilot enable-parameter behaviour.

## Board

`ZeroOneX9` is the helicopter image that carries both this codec and the
NT1-M TCP client. `Tools/AP_Bootloader/board_types.txt` assigns
`AP_HW_ZeroOne_X9` the value **5601**. The hwdef keeps the RMII `ETH1`
pins and sets `AP_ARINC429_ENABLED` and `AP_EROB_CANOPEN_ENABLED`. eRob is
compiled in and is not selected until `CAN_P2_DRIVER` moves off the shared
DroneCAN driver, matching ZeroOne X6. `A429_ENABLE` defaults to 1 and
`A429_FRAME` defaults to 2.

The earlier helicopter target is the waf board `ZeroOneX6`.

* `APJ_BOARD_ID` is `AP_HW_ZeroOne_X6`.
* `Tools/AP_Bootloader/board_types.txt` assigns that name the value **5600**
  (IDs 5600-5699 are reserved for ZeroOne).
* The image to build is `arducopter-heli` (`./waf heli`), not the
  multicopter `arducopter` image. Both images share the ArduCopter
  libraries; only the heli image is the CS35 target.
* ZeroOneX6-ODID includes this hwdef, so it inherits the same compile flag.
  It is not a separate protocol target.

The X6 and the X9 each have one Ethernet port. A future PROFINET gateway
needs that same port. The NT1-M client uses the existing `AP_Networking`
stack and does not take a second MAC. The X9 `ETH1` pins stay in the hwdef
so that stack can run.

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
The ICD status matrix describes the word that just arrived: 00 failure,
01 non-computed, 10 functional test, 11 normal. A non-normal word, a
parity error, a reserved bit, an unexpected SDI, or a BNR value outside
range replaces the current view and clears `fresh` immediately. It does
not move `valid_us` or `last_valid_eu`. The previous normal sample stays
in `last_valid` as history and is not what a fresh getter returns. A
later normal word sets `fresh` again and updates `valid_us`. Invalid
words still increment real `uint32` counters. Those counters are not
per-word flag bits.

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

Transmit. CLP (310) is the helicopter collective output in volts when
`update()` has a finite sample. Otherwise that update clears 310. Host
tests can still place a 310 snapshot with `a429_sim_set_clp` until that
publish runs. Nr (311) is the measured rotor percent when `update()` has
a finite `AP_RPM` sample and a positive `A429_NR_REF`. Otherwise that
update clears 311. Host tests can still place a 311 snapshot with
`a429_sim_set_nr` until that publish runs. The three command words are
the snapshot, or the engine-mode owner when `A429_ENG_CH` is 1..16:

| Octal | Name | Kind | Resolution | Range in the appendix |
| --- | --- | --- | --- | --- |
| 310 | CLP | BNR | 0.01 V | none stated |
| 311 | Nr | BNR | 0.01 % | none stated |
| 312 | P0 | BNR | 0.01 kPa | none stated. Sent only from a healthy, finite, positive baro pressure |
| 313 | T0 | BNR | 0.1 °C | none stated. Sent only from a healthy, finite baro temperature |
| 314 | H | BNR | 1 m | none stated. Absolute AMSL, not AGL |
| 316 | TAS | BNR | 0.1 m/s | none stated. Sensor true airspeed only |
| 321 | Hour | BNR | 1 | 0 .. 23. Sent only with 322..326 |
| 322 | Minute | BNR | 1 | 0 .. 59 |
| 323 | Second | BNR | 1 | 0 .. 59 |
| 324 | Year | BNR | 1 | Gregorian year, for example 2026 |
| 325 | Month | BNR | 1 | 1 .. 12 |
| 326 | Day | BNR | 1 | 1 .. 31. The appendix name is "Data" (date) |
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
| 317 | Pm | FC to ECU | BNR 0.0001 MPa. Not sent. No oil-pressure sensor on this flight controller. |
| 320 | Tm | FC to ECU | BNR 0.1 °C. Not sent. No oil-temperature sensor. |
| 331 | State331 | FC to ECU | discrete. See table B.16 below. Not sent and not packed. |

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
snapshot into a one-frame pulse. Calling `sim_set_pcmd()` is a simulation
input. With `A429_ENG_CH` at 0 the vehicle does not replace that input.
With `A429_ENG_CH` at 1..16, `AP_ARINC429::update()` writes the same level
into 340, 342 and 344 on every enabled pass. Cold-crank, false-start and
channel-switch stay 0, because those bits are rising-edge commands and
must not be held at 1. Rotor brake stays 0. The on-ground bit is 1 when
`AP_Notify::flags.flying` is false.

### Table B.16 (label 331, not sent)

Recorded from the appendix image so the bits are not dropped. The codec
does not pack this word. Labels 317 and 320 are the same kind of
engine-accessory measurement and are not encoded either. A zero or a
parameter is not substituted for a sensor this flight controller does
not have.

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
Each later poll emits at most one slot. That slot contains every FC
word that currently has a sample (310, 311, 312, 313, 314, 316,
321..326, and 340/342/344 when those are published). It does not rotate
one label per tick, so label 310 stays in every period while its
collective sample is valid. `A429_TX_MAX` is 16 words. Fifteen is the
largest set that can be valid together.

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
when a sample is still fresh and its age reaches the policy, then waits
for a new valid word. An already-invalid sample is not counted again.
Silence does not erase `last_valid`. A TCP session change does the
same to every phase-one label even when its timeout is 0: `fresh` becomes
0, while `last_valid`, `last_valid_eu`, and `valid_us` stay. Reconnect
does not copy that history back into a fresh sample. `A429_ENABLE` 0
clears freshness in that same update and does not schedule a slot.
`get_fresh_eu` and `get_fresh_disc` are for the vehicle thread that calls
`update()`. They return false when that thread can see a newer TCP epoch
than the one `update()` last applied. The raw `fresh` flag is cleared
when `update()` runs, which is later than the worker's socket close.
These getters are not a lock and are not safe to call from arbitrary
threads. The TCP worker is the only thread that opens, sends, receives,
or closes the socket, and it does those outside the state lock.

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
`A429_REM_PORT` are the peer. A zero octet inside a non-zero address is
legal, so `10.0.0.1` and `192.168.0.7` are configured peers. The socket
stays closed only when the whole address is `0.0.0.0` or the port is 0.
`A429_LOC_PORT` is not bound. The client uses an ephemeral local port.
The manual example `192.168.3.7:8887` is not the parameter default. The
flight-controller address is whatever `NET_*` already has. This code does
not write `NET_IP`, `NET_NETMASK`, `NET_GW`, or `NET_DHCP`, and it does
not restart the MAC. A belief that the bench network is `192.168.144.x`
is not verified here.

ZeroOneX6 has `ETH1`, so the hwdef turns on `AP_NETWORKING_ENABLED` and
`HAL_USE_MAC`. `AP_Networking` already owns that interface and lwIP.
`SocketAPM` is the socket API on that stack. `NET_ENABLE` still defaults
to 0. The ARINC worker calls `AP::network().get_ip_active()` and does not
open a socket while that address is 0. `startup_wait()` is not used,
because on ChibiOS it blocks until an address exists. The worker waits
the same way, off the vehicle thread.

`connect_timeout(..., 200)` can block the worker for 200 ms. It is not
called from `AP_ARINC429::update()`. Socket open, send, receive, and close
run after the state lock is released. After connect, send and receive are
non-blocking. The first refusal waits 200 ms. Each later refusal doubles
that wait, and the wait stops growing at 5 s. Opening the socket does not
put the wait back to 200 ms. The wait returns to 200 ms only when the
session that just died had one submitted slot fully accepted by `send()`.
Time blocked inside `connect_timeout`, and time spent up while `send()`
makes no progress, do not count. The stamp stored for the open is read
from the clock after `open` returns, and that read is not done while the
state lock is held. A connect that drops without a completed slot keeps
the grown wait. One attempt is started per poll.

The TX hold is 80 bytes, one slot (16 words at 5 bytes each). A new slot is dropped while that hold
is unfinished. If `send()` has not finished the slot within 50 ms, the
slot is abandoned. When `send()` has accepted no bytes, the slot is
dropped and the connection stays up. When `send()` has already accepted a
prefix, the connection is reset and the frame decoder is cleared, so the
next slot is a whole frame and is not concatenated onto that prefix.
Closing the socket does not purge bytes the kernel or the NT1-M has
already accepted. The NT1-M manual can still forward its serial cache on
the next connect when that module option is left enabled.

RX is 64 bytes. A full RX buffer stops the socket read and counts
backpressure. A failed `send()` in a poll that also received bytes drops
the connection, discards the RX staging for that connection, and clears
the frame decoder before those bytes can be decoded. Disconnect, peer
change, and a new connection do the same for leftover RX and for a
half-parsed `C0` frame, and they clear `fresh` on every phase-one label.
Clearing the half-parsed frame does not by itself retire a sample that
was already decoded. Nothing queued in the application hold is
replayed on the next connect, and reconnect does not mark the previous
sample fresh again. The 12500 microsecond scheduler is
unchanged and still emits one slot per poll.

What this does not prove: electrical 429 bit timing, NT1-M packing delay,
PROFINET sharing the same RJ45, or the delivered module's mode. It also
does not prove that a `close()` removed bytes already copied into the
socket buffer or the module. UDP is not implemented. The manual's UDP
note (512 bit packets) is not a TCP rule.

## Framing

The TCP pipe and the byte interpretation are separate.

`A429_FRAME` defaults to 2 (RS422-429-1C). Value 0 counts socket bytes as
`rx_unframed_bytes` and does not decode them into labels. A due transmit
slot with value 0 increments `tx_unframed` and is not written to the socket.

`A429_FRAME=1` is the provisional UART429-S profile. It
sends `0xC0` plus the codec `uint32` in little-endian order. The first
payload byte is the logical label integer. Octal label 036 is `0x1E` in
this codec. Octal label 0144 is `0x64`. This profile does not reverse
either byte. A `0xCD` byte is skipped as unknown. No speed command is
generated. Setting `A429_FRAME=1` does not by itself open the TCP
client. `A429_ENABLE` defaults to 1. The peer address and port still
default to 0, so the socket stays closed.

## RS422-429-1C wire profile, `A429_FRAME=2`

Source checked against the file itself: 天津展翼航空科技有限责任公司,
title page "RS422转ARINC429控制器 产品说明书", model RS422-429-1C on
page 2, five A4 pages, PDF author choi, Microsoft Word LTSC, created
2026-01-18 21:43 +08. The manufacturer share used to obtain it is
`https://share.fnnas.net/s/4a81bf74f8584075a3`. This is the RS422 DB9
product. It is not a measurement of the NT1-M or of the internal TTL
board on the bench.

Page 3, section 3, states both directions use `0xC0` plus four bytes.
`0xC0` is the packet header and does not appear on the 429 bus. The four
bytes are the 32-bit word in little-endian order. The worked example is
word `0x12345678` as `C0 78 56 34 12` toward the module and back from it.
The same page gives the rate commands, each `0xCD` plus four bytes:
100 kbps `CD 00 00 55 55`, 50 kbps `CD 03 00 55 55`, 12.5 kbps
`CD 07 00 55 55`. The page does not state a power-up rate, a command
reply, or that the receiver detects the rate.

Page 3 continues into section 4, and page 4 states the label rule. The
low byte is the label byte. Bit 1 of that byte is the MSB and bit 8 is
the LSB, so decoding the label reverses the byte. The worked word is
`0x629218C1`: `C1` is the label byte, and reversing it yields octal 203
(`C1` is `11000001`, reversed `10000011` is `0x83`). The page 3 send
example does not add a second reverse step on top of the 32-bit word. The
low byte of the word they put on the serial line is already that label
byte.

The codec in this tree does the other thing. It stores the octal label as
an integer in the low byte and applies odd parity to that logical word.
`A429_FRAME=1` still writes that logical word after `0xC0`. For octal
0144 the serial label byte is `0x64`. `A429_FRAME=2` is the separate
RS422-429-1C selector. It reverses only those eight label bits, so octal
0144 goes out as `0x26` and the frame is `C0 26 00 71 E2`. The other 24
bits stay put. Reversing the byte does not change its popcount, so a
valid odd parity bit stays valid and an invalid one stays invalid. Receive
applies the same swap and does not recompute parity. The manual example
`0x629218C1` has label byte `C1`, which reverses to `0x83`, octal 203.

Changing `A429_FRAME`, `A429_SDI`, the peer, or `A429_ENABLE` drops a
half-parsed frame and an unsent slot, and clears `fresh` in that same
call. The command-edge detectors are re-armed, so the next valid sample
only latches. The socket is closed by its owner thread, not by the
caller, when this connection has accepted any transmit byte or a
`send` is still in flight. A new profile is not written onto that
socket. `send` that finishes after the epoch has moved does not update
the new session and does not close a socket opened later.
`A429_ENABLE` defaults to 1. No frame value sends
`CD 00 00 55 55`, `CD 03 00 55 55`, or `CD 07 00 55 55`. Those three
commands have a standalone encoder for tests. The scheduler does not call it.

Page 2 fixes the RS422 baud at 115200 and does not state the data bits,
parity, or stop bits. Pages 2 and 3 describe the DB9, the 6–28 V supply,
and a common ground with no isolation. None of that applies to the NT1-M
Ethernet side. The vendor's statement that the network product uses the
RS422 message format is still not a test of the board behind this NT1-M.
Do not treat `A429_FRAME=1` as hardware-ready. Label 310 is the
helicopter collective in volts. Label 311 is measured Nr. Labels 340,
342 and 344 are the engine-mode words. They are not copied into motor,
servo, or eRob outputs, and the scheduler does not send a `CD` rate
command. `A429_FRAME` defaults to 2, so a due slot is encoded as `C0`
plus four little-endian bytes with only the label byte reversed. Value 0
still sends no bytes. The peer address and port still default to 0, so
nothing is written until a peer is configured.

## Engine mode

`A429_ENG_CH` defaults to 0. Zero does not touch labels 340, 342 or 344,
so a simulation snapshot survives every update. Values 1..16 select an RC
channel. PWM above 1800 requests engine run. Low, middle, an invalid
sample, or RC failsafe requests stop (`001`). There is no separate start
label: idle is what makes the ECU sequence the starter and ignition.

Motor interlock is aux function `MOTOR_INTERLOCK` (option 32), read with
`find_channel_for_option`. It is not the engine switch. Arming is
`AP::arming().is_armed()` and does not choose stop versus idle.

| Engine run | Armed | Interlock high | Words 340, 342, 344 |
| --- | --- | --- | --- |
| no | either | either | stop `001` |
| yes | no | either | idle `011`, kept sending |
| yes | yes | no | idle `011`, kept sending |
| yes | yes | yes | flight `100` on that update |

Flight does not wait for Nr. Idle power cannot accelerate the rotor.
Releasing interlock or disarming returns to idle and clears a runup latch.

The runup abort is off unless `A429_NR_REF`, `A429_NR_CRIT` and
`A429_RUNUP` are all non-zero and `AP_RPM` returns a finite sample.
Nr percent is `rpm * 100 / A429_NR_REF`. `A429_NR_INST` selects the sensor;
0 is RPM1. If Nr stays below `A429_NR_CRIT` for `A429_RUNUP` seconds after
flight is commanded, the three words latch back to idle until interlock is
released or the aircraft is disarmed. No sample does not abort. A dip
after the threshold was reached does not abort. The timer is unsigned
`now - start`.

Label 311 uses that same percent. `A429_NR_INST` is the `AP_RPM` instance
whose measured rate is published (0 is RPM1). That rate is the Hall
sample when the selected driver is the pin/Hall backend. It is not an
RSC setpoint and not a commanded target. The word is the existing BNR
encode: resolution 0.01 percent, SM normal, odd parity, SDI from
`A429_SDI`.

`update()` publishes 311 only while the sample is finite and
`A429_NR_REF` is greater than 0. A missing sample, a non-finite rpm, or a
reference that is 0, negative, or not finite clears 311. The previous
percent is not held, and no rpm is invented so the label would read 100
percent. A percent that does not fit the signed 0.01 BNR field is cleared
too. `AP_RPM::get_rpm` already withholds an unhealthy sensor, so there is
no second Nr timeout. A stopped rotor (0 rpm) is a real sample and is
sent as 0.00 percent. Clearing 311 does not change label 310 or the
stop, idle, and flight words.

`A429_ENABLE` 0 or `A429_ENG_CH` 0 stops the engine-mode owner from
overwriting the command snapshot. The three command words are cleared
only if this logic had taken them over. That release does not clear CLP
or Nr. Labels 310 and 311 are not part of that snapshot rule: while
`A429_ENABLE` is 1, `update()` writes or clears both from the current
sample on every pass, including when `A429_ENG_CH` is 0. The same pass
writes or clears 312, 313, 314, 316 and 321..326. A missing
collective clears 310 and does not change 311, the air-data words, or
the command words.

## Collective voltage, label 310

The source is the helicopter collective output: the filtered value
`AP_Motors` passes into the swash (`get_heli_collective`). It is the
same filter as `get_throttle()` on a heli, which is logged as `ThO`.
It is not an RC PWM, and it is not the RSC throttle curve. 0 is minimum
collective and 1 is maximum collective. A multicopter throttle is not
used. A missing motors object, a frame that is not a helicopter, or a
non-finite sample omits 310 for that frame. The previous voltage is not
held.

`A429_CLP_V0` defaults to 1 V and `A429_CLP_V100` defaults to 9 V.
Collective outside 0 to 100 percent is clamped, then

```text
V = CLP_V0 + (CLP_V100 - CLP_V0) * collective_percent / 100
```

which is `V = 1 + 8 * (collective_percent / 100)` with the defaults.
The word is that voltage at 0.01 V (BNR denominator 100), SM normal,
odd parity, SDI from `A429_SDI`. It is not a percent and not blade
degrees. Label 153 is the ECU echo and is a different word; its
airframe percent range is not applied here. A voltage that does not fit
the signed 16-bit field is omitted. The word rides the existing 12.5 ms
FC-to-ECU slot, in the same burst as the other valid FC words.

## Air data and UTC, labels 312, 313, 314, 316 and 321..326

`update()` publishes these from sensors the vehicle already has.
A sample that is missing, not finite, or outside the BNR field clears
that label on the same pass. The previous value is not held.

| Label | Source | When it is sent |
| --- | --- | --- |
| 312 P0 | `AP_Baro::get_pressure()`, kPa = Pa / 1000 | Barometer healthy, value finite and positive. Resolution 0.01 kPa, denominator 100. |
| 313 T0 | `AP_Baro::get_temperature()` in °C | Barometer healthy and the temperature finite. This is the primary barometer temperature, not the clamped external-temperature fallback. Resolution 0.1 °C, denominator 10. |
| 314 H | Absolute altitude AMSL, metres | AHRS healthy and `get_location()` yields a finite absolute altitude. Otherwise baro `get_altitude_AMSL()` when the barometer is healthy and a field elevation makes that value differ from the relative calibration height. Otherwise GPS altitude when the fix is 3D or better and the location altitude is absolute and finite. Relative-to-home height, HAGL, and a baro height with no field elevation are not sent as AMSL. Resolution 1 m, denominator 1. |
| 316 TAS | `AP_AHRS::airspeed_TAS()` in m/s | Only while `using_airspeed_sensor()` is true and the value is finite and not negative. A bench with no airspeed sensor sends nothing. A synthetic airspeed built from groundspeed is not used. Resolution 0.1 m/s, denominator 10. |
| 321..326 | UTC hour, minute, second, year, month, day | GPS date when `AP_GPS::time_epoch_usec()` is non-zero, otherwise `AP_RTC::get_date_and_time_utc()` when the RTC has been set. Both are UTC. No timezone conversion is applied. RTC and `gmtime_r` month 0..11 is stored as 1..12. All six labels are sent together. If either source is missing, or any field is outside 0..23 / 0..59 / 0..59 / 1..12 / 1..31, none of the six is sent. A leap second of 60 clears the set instead of being rewritten to 59. Year is the full Gregorian year. |

Labels 317 (oil pressure, MPa, denominator 10000), 320 (oil
temperature, °C, denominator 10) and 331 (table B.16 discrete) are
not transmitted.

## Vehicle hook

`AP_Vehicle` owns one `AP_ARINC429` when `AP_ARINC429_ENABLED` is 1.

* Parameter group `A429_`, index 34. Indices through 20 are `ENG_CH` 16,
  `NR_INST` 17, `NR_REF` 18, `NR_CRIT` 19, and `RUNUP` 20. Label 310 adds
  21 `CLP_V0` and 22 `CLP_V100`. Earlier indices are unchanged.
* `setup()` calls `init()`.
* The scheduler calls `update()` at 400 Hz.
* `update()` fills 310 from the heli collective and 311 from the selected
  `AP_RPM` instance on every enabled pass, including when `A429_ENG_CH`
  is 0. On that same pass it fills or clears 312, 313, 314, 316 and
  321..326 from the air-data and clock sources above. It fills 340, 342
  and 344 only while `A429_ENABLE` is 1 and `A429_ENG_CH` is 1..16.
  It does not write motors, servos, EFI, or eRob, and it does not send
  317, 320 or 331.

The compile flag is set on the ZeroOne X6 and ZeroOne X9 hwdefs.
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
samples, a missing-data timeout, a clock wrap, and a late tick. Measured
Nr checks label 311 at 0.01 percent from `rpm * 100 / A429_NR_REF`, odd
parity, and clears the word when the sample is missing or `A429_NR_REF`
is 0. Collective checks label 310 at 0.01 V: 0 percent is 1.00 V, 100
percent is 9.00 V, 50 percent is 5.00 V, values outside 0 to 100 percent
clamp to those endpoints, and a missing or non-finite collective clears
the word without holding the previous voltage. Air-data checks encode
and clear 312, 313, 314, 316 and 321..326, refuse 317, 320 and 331, and
build one slot that still starts with 310 when every other valid FC
word is present. Two on-time 12.5 ms polls each include 310, and the
same tick does not send a second slot. Invalid SM values and a
parity error clear freshness while leaving the last normal sample in
`last_valid`. The NT1-M tests add a scripted byte pipe
and a local TCP server: invalid peer, a zero octet inside a real address,
fragmented and coalesced reads, partial writes, reconnect without
cross-connection assembly, a 50 ms would-block, a partial frame that
resets instead of being spliced, a read that succeeds in the same poll as
a failed write, unstable-connection backoff, peer silence, a full RX
buffer, one late scheduler slot, and the difference between logical label
`0x1E` and wire example `0x78`.

SITL repeats the same suite as `libraries/AP_ARINC429/tests/test_arinc429.cpp`
when the board is built with gtest. SITL leaves the feature off, so the
test compiles the codec itself and does not duplicate the firmware objects.

Helicopter image for the 01 Aero X9, not flashed:

```sh
./waf configure --board ZeroOneX9
./waf heli
```

The apj `board_id` field is the check for 5601. The generated hwdef keeps
`AP_ARINC429_ENABLED` and `AP_EROB_CANOPEN_ENABLED` at 1, and the `ETH1`
pins stay. The X6 image is still:

```sh
./waf configure --board ZeroOneX6
./waf heli
```

Its apj `board_id` field is 5600. eRob remains
`AP_EROB_CANOPEN_ENABLED 1` in that hwdef too.
