#!/usr/bin/env python3
"""
Offline reference for BO_ 837 LANE_KEEP (Jaecoo J7 / cherry_general_pt.dbc).

Bus topology (Jaecoo J7 via comma 3 / panda):
  - bus 0 = vehicle / ECU (EPS, transmission, etc.). openpilot TXes LANE_KEEP HERE.
  - bus 2 = stock camera leg. Stock ADAS camera transmits LANE_KEEP HERE.
  - cherry safety blocks bus 2 -> bus 0 forwarding of 0x345 when the relay is open,
    so the ECU only ever sees one source of LANE_KEEP at a time.

On-vehicle: capture stock frames (lateral handed back to stock LKA), then:
  cd /data/openpilot
  python selfdrive/debug/can_table.py 0x345 2   # observe stock frame on bus 2
  python selfdrive/debug/can_table.py 0x345 0   # with relay open, this is openpilot

Compare printed 8-byte payload to this script's output: padding bytes 2..5
(ff fc f4 63), upper nibble of byte 6 (0xF), and cherry_checksum on byte 7 must
all match. Counter must increment by exactly 1 with no jumps at 50 Hz.

Requires: PYTHONPATH including opendbc_repo (same as openpilot dev shell).
"""

from __future__ import annotations

import argparse

from opendbc.can.packer import CANPacker
from opendbc.car.cherry.cherrycan import cherry_checksum, create_lane_keep_command
from opendbc.car.cherry.values import CANBUS


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
  parser.add_argument("--steer-deg", type=float, default=0.0, help="STEER_CMD_ANGLE (CAN physical deg, before OP sign)")
  parser.add_argument("--lkas", type=int, default=1, choices=(0, 1), help="LKAS_ENABLE")
  parser.add_argument("--counter", type=int, default=None, help="COUNTER 0..15 (default: packer auto)")
  parser.add_argument("--sign", type=float, default=-1.0, help="steering_deg_sign (Jaecoo J7 uses -1)")
  args = parser.parse_args()

  dbc = "cherry_general_pt"
  packer = CANPacker(dbc)
  vals = {
    "STEER_CMD_ANGLE": args.steer_deg,
    "LKAS_ENABLE": args.lkas,
    "SET_ME_XFF": 255,
    "SET_ME_XFC": 252,
    "SET_ME_XF4": 244,
    "SET_ME_X63": 99,
    "SET_ME_XF": 15,
  }
  if args.counter is not None:
    vals["COUNTER"] = args.counter

  addr, dat, bus = packer.make_can_msg("LANE_KEEP", CANBUS.main_bus, vals)
  b = bytearray(dat)
  chk = cherry_checksum(addr, None, b)
  print(f"addr=0x{addr:03X} bus={bus} (TX bus = vehicle/ECU side) hex={dat.hex()}")
  print(f"bytes[2:6] (padding) = {b[2]:02x} {b[3]:02x} {b[4]:02x} {b[5]:02x}")
  print(f"byte6=0x{b[6]:02x}  CHECKSUM=0x{b[7]:02x}  formula==byte7: {chk == b[7]}")

  msg = create_lane_keep_command(packer, steer_angle_deg=5.0, steer_req=True, meas_angle_deg=3.0, steering_deg_sign=args.sign)
  print(f"\ncreate_lane_keep_command(5°, req=True, meas=3°, sign={args.sign}): {msg[1].hex()} bus={msg[2]}")


if __name__ == "__main__":
  main()
