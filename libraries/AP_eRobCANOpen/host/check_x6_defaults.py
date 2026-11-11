#!/usr/bin/env python3

"""
Lock the ZeroOne X6 first-boot eRob and safety defaults.

AP_FLAKE8_CLEAN

"""

import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

EROB_GROUP = {
    "NODE": 1,
    "BM": 0,
    "MIN": -43691,
    "MAX": 43691,
    "GEAR": 0,
    "RES": 524288,
    "CTR": 262144,
    "SV1": 1,
    "SV2": 2,
    "SV3": 3,
    "SV4": 4,
    "SV5": 0,
    "SV6": 0,
    "SV7": 0,
    "SV8": 0,
    "ID1": 1,
    "ID2": 2,
    "ID3": 3,
    "ID4": 4,
    "ID5": 0,
    "ID6": 0,
    "ID7": 0,
    "ID8": 0,
    "RATE": 100,
    "ACC": 500,
    "DEC": 500,
}

X6_PARM = {
    "CAN_P1_DRIVER": 1,
    "CAN_D1_PROTOCOL": 1,
    "CAN_P2_DRIVER": 2,
    "CAN_D2_PROTOCOL": 15,
    "CAN_D2_PROTOCOL2": 0,
    "CAN_P2_BITRATE": 1000000,
    "BRD_SAFETY_DEFLT": 0,
}
for _name, _value in EROB_GROUP.items():
    X6_PARM["CAN_D2_EROB_" + _name] = _value


def fail(msg):
    """Record one mismatch."""
    print("FAIL: " + msg, file=sys.stderr)
    return 1


def parse_parm(path):
    """Read active name/value lines. Comments and blanks are ignored."""
    values = {}
    with open(path, "r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = re.split(r"[\s,=]+", line)
            if len(parts) < 2:
                raise ValueError("bad parm line in %s: %s" % (path, raw.rstrip()))
            values[parts[0]] = int(float(parts[1]))
    return values


def parse_groupinfo(path):
    """Read AP_GROUPINFO default values for AP_eRobCANOpen."""
    text = open(path, "r", encoding="utf-8").read()
    found = {}
    pattern = re.compile(
        r'AP_GROUPINFO\(\s*"([A-Z0-9]+)"\s*,\s*\d+\s*,\s*AP_eRobCANOpen\s*,'
        r'\s*[A-Za-z0-9_\[\]]+\s*,\s*(-?\d+)\s*\)'
    )
    for name, value in pattern.findall(text):
        found[name] = int(value)
    return found


def main():
    """Return 0 when the X6 defaults match the agreed set."""
    errors = 0
    cpp = os.path.join(ROOT, "libraries", "AP_eRobCANOpen", "AP_eRobCANOpen.cpp")
    group = parse_groupinfo(cpp)
    for name, value in EROB_GROUP.items():
        if group.get(name) != value:
            errors += fail("AP_GROUPINFO %s is %s, want %s" % (name, group.get(name), value))

    x6 = os.path.join(
        ROOT, "libraries", "AP_HAL_ChibiOS", "hwdef", "ZeroOneX6", "defaults.parm")
    parm = parse_parm(x6)
    for name, value in X6_PARM.items():
        if parm.get(name) != value:
            errors += fail("ZeroOneX6 %s is %s, want %s" % (name, parm.get(name), value))

    cuav = os.path.join(
        ROOT, "libraries", "AP_HAL_ChibiOS", "hwdef", "CUAV-X7", "defaults.parm")
    cuav_parm = parse_parm(cuav)
    if cuav_parm.get("CAN_P2_DRIVER") == 2 or cuav_parm.get("CAN_D2_PROTOCOL") == 15:
        errors += fail("CUAV-X7 defaults enable CAN2 eRob")
    if "BRD_SAFETY_DEFLT" in cuav_parm:
        errors += fail("CUAV-X7 defaults override BRD_SAFETY_DEFLT")

    board = os.path.join(ROOT, "libraries", "AP_BoardConfig", "AP_BoardConfig.cpp")
    board_text = open(board, "r", encoding="utf-8").read()
    needle = "SAFETY_DEFLT\",   3, AP_BoardConfig, state.safety_enable, BOARD_SAFETY_ENABLE_DEFAULT"
    if needle not in board_text:
        errors += fail("shared BRD_SAFETY_DEFLT no longer uses BOARD_SAFETY_ENABLE_DEFAULT")

    can = os.path.join(
        ROOT, "libraries", "AP_CANManager", "AP_CANManager_CANDriver_Params.cpp")
    can_text = open(can, "r", encoding="utf-8").read()
    if "float(AP_CAN::Protocol::DroneCAN)" not in can_text:
        errors += fail("global CAN protocol default is no longer DroneCAN")

    x9 = os.path.join(
        ROOT, "libraries", "AP_HAL_ChibiOS", "hwdef", "ZeroOneX9", "defaults.parm")
    if os.path.exists(x9):
        x9_parm = parse_parm(x9)
        if x9_parm.get("CAN_D2_PROTOCOL") == 15 or x9_parm.get("CAN_P2_DRIVER") == 2:
            errors += fail("ZeroOneX9 CAN defaults were changed")

    if errors:
        return 1
    print("OK ZeroOne X6 defaults")
    return 0


if __name__ == "__main__":
    sys.exit(main())
