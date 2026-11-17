#!/usr/bin/env python3
'''
AP_FLAKE8_CLEAN

Simulated CS35 ECU and a SITL check of A429 named values.

The ECU is a TCP server in the RS422-429-1C frame: 0xC0 plus four
little-endian bytes, label byte reversed. Fourteen labels are sent
every 24 ms. This is not a physical NT1-M.
'''

import argparse
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

PERIOD_S = 0.024
MAVLINK2_NAMED = 30

# octal label, kind, value. bnr is engineering units. raw and disc are the data field.
BENCH = (
    (0o144, "bnr", 50.0, 100),
    (0o145, "bnr", 100.0, 100),
    (0o146, "bnr", 48.0, 100),
    (0o147, "bnr", 450.0, 10),
    (0o151, "bnr", 80.0, 10),
    (0o152, "bnr", 0.8, 10000),
    (0o153, "raw", 2500, 0),
    (0o154, "bnr", 10.0, 100),
    (0o155, "bnr", 25.0, 10),
    (0o156, "bnr", 0.1013, 10000),
    (0o160, "bnr", 1.5, 100),
    (0o175, "disc", 0x11, 0),
    (0o176, "disc", 0x22, 0),
    (0o177, "disc", 0x33, 0),
)

PACK_GROUPS = (
    (144, 145, 146, 147),
    (151, 152, 153, 154),
    (155, 156, 160, 175),
    (176, 177, None, None),
)


def apply_odd(word):
    word &= 0x7FFFFFFF
    x = word
    x ^= x >> 16
    x ^= x >> 8
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    if (x & 1) == 0:
        word |= 1 << 31
    return word


def bitrev(value):
    out = 0
    for _ in range(8):
        out = ((out << 1) | (value & 1)) & 0xFF
        value >>= 1
    return out


def pack_word(label, data):
    word = label & 0xFF
    word |= (int(data) & 0xFFFF) << 13
    word |= 3 << 29
    return apply_odd(word)


def data_field(kind, value, denom):
    if kind == "bnr":
        scaled = value * denom
        if scaled >= 0:
            return int(scaled + 0.5)
        return int(scaled - 0.5)
    return int(value)


def frame2(word):
    wire = (word & 0xFFFFFF00) | bitrev(word & 0xFF)
    return bytes((0xC0,)) + struct.pack("<I", wire)


def logical_words():
    words = []
    for label, kind, value, denom in BENCH:
        words.append((label, pack_word(label, data_field(kind, value, denom))))
    return words


def check_known_frame():
    ng = pack_word(0o144, 5000)
    frame = frame2(ng)
    if ng != 0xE2710064 or frame != bytes.fromhex("c0260071e2"):
        raise RuntimeError("Ng 50 percent FRAME2 vector mismatch: %s" % frame.hex())


class EcuSim(threading.Thread):
    def __init__(self, host, port):
        super().__init__(name="cs35-ecu", daemon=True)
        self.host = host
        self.port = port
        self.stop_labels = set()
        self._labels_lock = threading.Lock()
        self._halt = threading.Event()
        self.ready = threading.Event()
        self.clients = 0
        self.frames = 0
        self.rx_bytes = 0
        self.error = None

    def halt(self):
        self._halt.set()

    def label_stopped(self, label):
        with self._labels_lock:
            return label in self.stop_labels

    def set_label_stopped(self, label, stopped):
        with self._labels_lock:
            if stopped:
                self.stop_labels.add(label)
            else:
                self.stop_labels.discard(label)

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.listen(1)
            sock.settimeout(0.5)
            self.ready.set()
            while not self._halt.is_set():
                try:
                    conn, _addr = sock.accept()
                except socket.timeout:
                    continue
                self.clients += 1
                try:
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self._stream(conn)
                except OSError as exc:
                    self.error = exc
                finally:
                    try:
                        conn.close()
                    except OSError:
                        pass
        except OSError as exc:
            self.error = exc
            self.ready.set()
        finally:
            try:
                sock.close()
            except OSError:
                pass

    def _drain_tx(self, conn):
        # The flight controller sends a full FC slot every 12.5 ms. Read
        # those bytes so the TCP window stays open for the ECU frames.
        conn.setblocking(False)
        try:
            while True:
                try:
                    data = conn.recv(4096)
                except BlockingIOError:
                    return True
                except InterruptedError:
                    continue
                if not data:
                    return False
                self.rx_bytes += len(data)
        finally:
            conn.setblocking(True)

    def _stream(self, conn):
        nxt = time.monotonic()
        while not self._halt.is_set():
            if not self._drain_tx(conn):
                return
            blob = bytearray()
            for label, word in logical_words():
                if self.label_stopped(label):
                    continue
                blob += frame2(word)
                self.frames += 1
            try:
                conn.sendall(blob)
            except OSError:
                return
            nxt += PERIOD_S
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()


def field_name(msg):
    raw = getattr(msg, "name", "")
    if isinstance(raw, bytes):
        raw = raw.split(b"\x00", 1)[0].decode("ascii", "replace")
    else:
        raw = str(raw).split("\x00", 1)[0]
    return raw


def field_text(msg):
    raw = getattr(msg, "text", "")
    if isinstance(raw, bytes):
        raw = raw.split(b"\x00", 1)[0].decode("ascii", "replace")
    else:
        raw = str(raw).split("\x00", 1)[0]
    return raw


def unpack_counts(values):
    counts = {}
    for group, labels in enumerate(PACK_GROUPS):
        key = "A4C%d" % group
        if key not in values:
            continue
        packed = int(values[key]) & 0xFFFFFFFF
        for shift, label in enumerate(labels):
            if label is None:
                continue
            counts[label] = (packed >> (8 * shift)) & 0xFF
    return counts


def set_params(mav, params):
    # MAV_PARAM_TYPE_REAL32. ArduPilot accepts it for every scalar.
    for name, value in params.items():
        mav.mav.param_set_send(
            mav.target_system,
            mav.target_component,
            name.encode("utf-8"),
            float(value),
            9,
        )
        time.sleep(0.05)


def run_check(args):
    from pymavlink import mavutil

    check_known_frame()
    ecu = EcuSim(args.ecu_host, args.ecu_port)
    ecu.start()
    if not ecu.ready.wait(2.0):
        raise RuntimeError("ECU socket did not open")
    if ecu.error is not None:
        raise RuntimeError("ECU socket failed: %s" % ecu.error)

    tmp = tempfile.mkdtemp(prefix="a429-sitl-")
    log_path = os.path.join(tmp, "sitl.log")
    sitl_log = open(log_path, "w", encoding="utf-8")
    cmd = [
        args.bin,
        "--model", "heli",
        "--speedup", "1",
        "--defaults", args.defaults,
        "-w",
        "--serial0", "tcp:%d:wait" % args.mav_port,
        "--home", "-35.363261,149.165230,584,353",
    ]
    sitl = subprocess.Popen(
        ["stdbuf", "-oL", "-eL"] + cmd,
        cwd=tmp,
        stdout=sitl_log,
        stderr=subprocess.STDOUT,
    )
    report = {
        "tmp": tmp,
        "bytes": 0,
        "messages": 0,
        "seconds": 0.0,
        "rates": {},
        "samples": {},
        "counts": {},
        "stale": "",
        "recovered": "",
    }
    try:
        time.sleep(1.0)
        mav = mavutil.mavlink_connection(
            "tcp:%s:%d" % (args.ecu_host, args.mav_port),
            source_system=255,
        )
        hb = mav.wait_heartbeat(timeout=60)
        if hb is None:
            raise RuntimeError("no heartbeat from heli SITL")
        set_params(mav, {
            "A429_ENABLE": 1,
            "A429_FRAME": 2,
            "A429_IP0": 127,
            "A429_IP1": 0,
            "A429_IP2": 0,
            "A429_IP3": 1,
            "A429_REM_PORT": args.ecu_port,
            "A429_LOC_PORT": 0,
            "A429_TLM_HZ": 10,
            "A429_NG_TO": 400,
            "A429_SDI": 0,
        })
        deadline = time.monotonic() + 40
        seen_ng = False
        while time.monotonic() < deadline:
            msg = mav.recv_match(type=["NAMED_VALUE_FLOAT", "STATUSTEXT"], blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "NAMED_VALUE_FLOAT" and field_name(msg) == "A4NG":
                seen_ng = True
                break
        if not seen_ng:
            raise RuntimeError("A4NG did not arrive; ecu clients=%d error=%s" % (ecu.clients, ecu.error))

        kinds = ["NAMED_VALUE_FLOAT", "NAMED_VALUE_INT", "STATUSTEXT"]
        counts_msg = {}
        samples = {}
        t0 = time.monotonic()
        window = 5.0
        while time.monotonic() - t0 < window:
            msg = mav.recv_match(type=kinds, blocking=True, timeout=1)
            if msg is None:
                continue
            name = field_name(msg) if msg.get_type() != "STATUSTEXT" else ""
            if not name.startswith("A4"):
                continue
            buf = msg.get_msgbuf()
            report["bytes"] += len(buf) if buf is not None else MAVLINK2_NAMED
            report["messages"] += 1
            counts_msg[name] = counts_msg.get(name, 0) + 1
            samples[name] = msg.value
        elapsed = time.monotonic() - t0
        report["seconds"] = elapsed
        report["rates"] = {key: counts_msg[key] / elapsed for key in sorted(counts_msg)}
        report["samples"] = samples
        report["counts"] = unpack_counts(samples)

        ecu.set_label_stopped(0o144, True)
        stale_deadline = time.monotonic() + 8
        while time.monotonic() < stale_deadline:
            msg = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if msg is None:
                continue
            text = field_text(msg)
            if "144" in text and "stale" in text:
                report["stale"] = text
                break
        if not report["stale"]:
            raise RuntimeError("STATUSTEXT for Ng stale did not arrive")

        time.sleep(1.2)
        ecu.set_label_stopped(0o144, False)
        ok_deadline = time.monotonic() + 8
        while time.monotonic() < ok_deadline:
            msg = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if msg is None:
                continue
            text = field_text(msg)
            if "144" in text and "ok" in text:
                report["recovered"] = text
                break
        if not report["recovered"]:
            raise RuntimeError("STATUSTEXT for Ng recovery did not arrive")

        ng_hz = report["rates"].get("A4NG", 0.0)
        if not 7.0 <= ng_hz <= 13.0:
            raise RuntimeError("A4NG rate %.2f Hz is outside 7..13" % ng_hz)
        ng_count = report["counts"].get(144)
        if ng_count is None or not 30 <= ng_count <= 55:
            raise RuntimeError("label 144 count/s is %s" % ng_count)
        for label in (145, 146, 147, 151, 152, 153, 154, 155, 156, 160, 175, 176, 177):
            count = report["counts"].get(label)
            if count is None or not 30 <= count <= 55:
                raise RuntimeError("label %d count/s is %s" % (label, count))
        if abs(samples.get("A4NG", 0.0) - 50.0) > 0.05:
            raise RuntimeError("A4NG value %s" % samples.get("A4NG"))
        if abs(samples.get("A4NP", 0.0) - 100.0) > 0.05:
            raise RuntimeError("A4NP value %s" % samples.get("A4NP"))
        if abs(samples.get("A4T45", 0.0) - 450.0) > 0.2:
            raise RuntimeError("A4T45 value %s" % samples.get("A4T45"))
        if abs(samples.get("A4MKP", 0.0) - 80.0) > 0.2:
            raise RuntimeError("A4MKP value %s" % samples.get("A4MKP"))
        if abs(samples.get("A4P3", 0.0) - 0.8) > 0.001:
            raise RuntimeError("A4P3 value %s" % samples.get("A4P3"))
        if abs(samples.get("A4CLP", 0.0) - 2500.0) > 0.5:
            raise RuntimeError("A4CLP value %s" % samples.get("A4CLP"))
        if int(samples.get("A4WN1", -1)) != 0x11:
            raise RuntimeError("A4WN1 value %s" % samples.get("A4WN1"))
        ngc_hz = report["rates"].get("A4NGC", 0.0)
        if not 0.5 <= ngc_hz <= 2.5:
            raise RuntimeError("A4NGC rate %.2f Hz is outside 0.5..2.5" % ngc_hz)
        count_hz = report["rates"].get("A4C0", 0.0)
        if not 0.5 <= count_hz <= 2.5:
            raise RuntimeError("A4C0 rate %.2f Hz is outside 0.5..2.5" % count_hz)
        if ecu.rx_bytes == 0:
            raise RuntimeError("ECU received no FC transmit bytes")
        report["ecu_rx_bytes"] = ecu.rx_bytes
        return report
    except Exception:
        sitl_log.flush()
        try:
            with open(log_path, "r", encoding="utf-8", errors="replace") as logf:
                tail = logf.readlines()[-50:]
        except OSError:
            tail = []
        sys.stderr.write("".join(tail))
        raise
    finally:
        ecu.halt()
        ecu.join(timeout=2)
        if sitl.poll() is None:
            sitl.terminate()
            try:
                sitl.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sitl.kill()
        sitl_log.close()


def main():
    parser = argparse.ArgumentParser(description="SITL check for CS35 ARINC429 telemetry")
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    parser.add_argument("--bin", default=os.path.join(root, "build", "sitl", "bin", "arducopter-heli"))
    parser.add_argument("--defaults", default=os.path.join(root, "Tools", "autotest", "default_params", "copter-heli.parm"))
    parser.add_argument("--ecu-host", default="127.0.0.1")
    parser.add_argument("--ecu-port", type=int, default=8891)
    parser.add_argument("--mav-port", type=int, default=5760)
    args = parser.parse_args()
    if not os.path.isfile(args.bin):
        print("missing SITL binary %s" % args.bin, file=sys.stderr)
        return 2
    try:
        report = run_check(args)
    except Exception as exc:
        print("FAIL: %s" % exc, file=sys.stderr)
        return 1
    bps = report["bytes"] / report["seconds"] if report["seconds"] else 0.0
    print("SITL telemetry window %.2f s, %d A4 messages, %.0f bytes/s" % (
        report["seconds"], report["messages"], bps))
    for name in ("A4NG", "A4NP", "A4T45", "A4MKP", "A4NGC", "A4P3", "A4CLP", "A4WN1", "A4C0", "A4PAR"):
        if name in report["rates"]:
            print("  %s %.2f Hz sample %s" % (name, report["rates"][name], report["samples"].get(name)))
    print("  per-label counts/s %s" % report["counts"])
    print("  stale: %s" % report["stale"])
    print("  recovered: %s" % report["recovered"])
    print("  ecu rx bytes %d" % report.get("ecu_rx_bytes", 0))
    print("  workdir %s" % report["tmp"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
