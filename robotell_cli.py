#!/usr/bin/env python3
"""
robotell-cli — talk to a TQ HPR50 e-bike over CAN using a generic Robotell
USB-CAN adapter (via python-can), mirroring the CAN features of the C# `can-cli`.

It speaks the TQ "PER" protocol directly on the wire (standard 11-bit frames,
500 kbit/s), so it does NOT need any TQ assemblies. It loads the same
parameter table (`params.json`) that `can-cli` uses for name<->ID/node/range
resolution and value scaling.

What it can do (same as can-cli):  scan, info, list-params, param-info,
read, write, raw-read, raw-write, live, reset.

What it CANNOT do (TQ-dongle-only, intentionally omitted): power-on/wake,
dongle ADC diagnostics (diag), DongleValue reads, bootloader start-app,
and the web-service commands. See ROBOTELL_CLI.md for why.

Requires:  pip install python-can pyserial
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass

# python-can is imported lazily (only when a command actually opens the bus) so
# offline commands — list-params, param-info, list-nodes — work without it.

# --------------------------------------------------------------------------
# PER protocol constants  (see CAN.md in repo root for the full description)
# --------------------------------------------------------------------------

CAN_BITRATE = 500_000          # HPR50 bus is fixed at 500 kbit/s

# python-can interfaces whose Bus() accepts our ttyBaudrate/rtscts kwargs.
# (robotell and slcan both use `ttyBaudrate`.)  Other backends — socketcan,
# pcan, kvaser, ixxat, vector, seeedstudio, … — take different kwargs, so for
# those use --interface/--channel/--bitrate plus any --bus-arg you need.
SERIAL_TTY_INTERFACES = {"robotell", "slcan"}
SERVICE_TOOL_NODE = 61         # who we pretend to be on the bus
SERVICE_DONGLE_NODE = 60       # the TQ service dongle's node id

# NodeHWType (TargetLib) — the HW-type byte we put in our SLAVECHANGED announce.
# The MCB grants param access based on the registered identity, so the dongle
# type may unlock more (e.g. factory-protected writes) than the tool type.
HW_TYPE_SERVICE_TOOL = 0xF0
HW_TYPE_SERVICE_DONGLE = 0xF1
IDENTITIES = {
    # name:     (src_node,            hw_type_byte)
    "tool":     (SERVICE_TOOL_NODE,   HW_TYPE_SERVICE_TOOL),
    "dongle":   (SERVICE_DONGLE_NODE, HW_TYPE_SERVICE_DONGLE),
}

# P3 point-to-point message ids:  canid = 0x600 | (node << 3) | msgId
P3_SDR = 2                     # service-data read  (read a parameter)
P3_SDW = 3                     # service-data write (write a parameter)

# P1 broadcast / ack message ids:  canid = (msgId << 6) | node
P1_ACK_SDR = 8
P1_ACK_SDW = 10
P1_BROADCAST = 14
P1_SLAVECHANGED = 15

# ACK reply status byte (E_CanStatus from CanLib's CheckReceiveMsg). The status
# is the FIRST payload byte; address follows at [1:3]; value (SDR) at [3:7].
# NOTE: STAT_OK == 1, *not* 0 — a zero here is not success.
STAT_OK = 1
CAN_STATUS_NAMES = {
    1: "OK", 2: "ERRCAN", 3: "ERRADDRESS", 4: "ERRRANGE", 5: "ERRACCESS",
    6: "ERRCMDVAR", 7: "ERRUNSPEC", 8: "ERRDATA", 9: "BUSY", 10: "PENDING",
    11: "OK_DATA_START", 12: "OK_DATA_CONTINUE", 13: "OK_DATA_END",
}

NODE_NAMES = {
    1: "MCB / motor master",
    16: "BMS (legacy)",
    17: "BMS HPR50",
    18: "Charger port",
    20: "Range extender",
    21: "Display (HMI)",
    32: "Light",
    44: "Charger",
    46: "Smartbox",
    60: "Service Dongle",
    61: "Service Tool (this PC)",
}

# Nodes worth actively probing during `scan`, paired with a parameter id known
# to exist on them (used as a harmless read "ping").
SCAN_PROBES = {
    1: 7168,    # MCB_RT_LOG_OPERATING_TIME
    17: 12611,  # BATT_SOH
    20: 12563,  # RAEXT_SOC
    21: 24578,  # DISP_WALK_ASSIST
    46: 12321,  # SB_ESHIFT
}


def p3_id(node: int, msg_id: int) -> int:
    return 0x600 | ((node & 0x3F) << 3) | (msg_id & 0x7)


def p1_id(node: int, msg_id: int) -> int:
    return ((msg_id & 0xF) << 6) | (node & 0x3F)


# --------------------------------------------------------------------------
# Parameter table
# --------------------------------------------------------------------------

@dataclass
class Param:
    name: str
    id: int
    type: str
    node: int
    access: str
    unit: str = ""
    scale: float = 1.0
    width: int = 4
    signed: bool = False
    vmin: int | None = None
    vmax: int | None = None

    @property
    def readable(self) -> bool:
        return "Read" in self.access

    @property
    def writable(self) -> bool:
        return "Write" in self.access


class ParamTable:
    def __init__(self, entries: list[Param]):
        self._by_name = {p.name.upper(): p for p in entries}
        self._by_id = {}
        for p in entries:
            self._by_id.setdefault(p.id, p)
        self.all = entries

    @classmethod
    def load(cls, path: str) -> "ParamTable":
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
        out = []
        for e in raw:
            vr = e.get("ValidRange") or {}
            scale = e.get("UnitScalingFactor", 1)
            try:
                scale = float(str(scale))
            except (TypeError, ValueError):
                scale = 1.0
            # ValueWidth is an enum string: absent -> 32-bit, "SixteenBix" -> 16-bit
            width = 2 if str(e.get("ValueWidth")).startswith("Sixteen") else 4
            out.append(Param(
                name=e["Name"],
                id=int(e["ID"]),
                type=e.get("Type", "CanValue"),
                node=int(e.get("CanTargetNodeId", -1)),
                access=e.get("Access", "Read"),
                unit=e.get("UnitString", "") or "",
                scale=scale,
                width=width,
                signed=bool(e.get("IsSigned", False)),
                vmin=vr.get("Min"),
                vmax=vr.get("Max"),
            ))
        return cls(out)

    def resolve(self, token: str) -> Param | None:
        t = token.strip()
        if t.upper() in self._by_name:
            return self._by_name[t.upper()]
        try:
            pid = int(t, 16) if t.lower().startswith("0x") else int(t)
        except ValueError:
            return None
        return self._by_id.get(pid)


def default_params_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "BikeParameters", "params.json"))


# --------------------------------------------------------------------------
# Bus wrapper — SDR / SDW transactions over python-can
# --------------------------------------------------------------------------

class PerBus:
    def __init__(self, interface: str, channel: str, bitrate: int,
                 tty_baud: int, rtscts: bool, extra: dict, verbose: bool):
        try:
            import can
        except ImportError:
            sys.exit("error: python-can is not installed.  Run:  pip install python-can pyserial")
        self._can = can
        self.verbose = verbose
        # identity we announce as; overridable via --identity (see _open)
        self.src_node = SERVICE_TOOL_NODE
        self.hw_type = HW_TYPE_SERVICE_TOOL

        # Build kwargs common to every python-can backend, then add the
        # serial-tty knobs only for interfaces that accept them, then let
        # --bus-arg override/extend anything for less-common adapters.
        kwargs = {"interface": interface, "channel": channel}
        if bitrate:
            kwargs["bitrate"] = bitrate
        if interface in SERIAL_TTY_INTERFACES:
            kwargs["ttyBaudrate"] = tty_baud
            kwargs["rtscts"] = rtscts
        kwargs.update(extra)
        if verbose:
            shown = {k: v for k, v in kwargs.items() if k != "interface"}
            print(f"opening can.Bus(interface={interface!r}, {shown})", file=sys.stderr)
        try:
            self.bus = can.Bus(**kwargs)
        except Exception as e:
            sys.exit(f"error: could not open CAN interface '{interface}' on "
                     f"channel '{channel}': {e}")

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _send(self, can_id: int, data: bytes):
        msg = self._can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        if self.verbose:
            print(f"  TX {can_id:03X}  {data.hex(' ')}", file=sys.stderr)
        self.bus.send(msg)

    def _wait_for(self, can_id: int, addr: int | None, timeout: float):
        """Wait for a frame with arbitration_id == can_id (and matching addr bytes
        if addr is given). Returns the can.Message or None on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            m = self.bus.recv(timeout=deadline - time.monotonic())
            if m is None:
                break
            if self.verbose:
                print(f"  RX {m.arbitration_id:03X}  {bytes(m.data).hex(' ')}", file=sys.stderr)
            if m.arbitration_id != can_id:
                continue
            # ACK payload is [status, addr_lo, addr_hi, ...] — address at [1:3].
            if addr is not None and len(m.data) >= 3:
                got = m.data[1] | (m.data[2] << 8)
                if got != addr:
                    continue
            return m
        return None

    # ---- service-data read ------------------------------------------------
    def sdr(self, node: int, addr: int, timeout: float):
        """Returns (ok, raw_int, reply_bytes)."""
        payload = bytes([addr & 0xFF, (addr >> 8) & 0xFF])
        self._send(p3_id(node, P3_SDR), payload)
        reply = self._wait_for(p1_id(node, P1_ACK_SDR), addr, timeout)
        if reply is None:
            return False, None, None
        data = bytes(reply.data)
        status = data[0] if len(data) >= 1 else 0xFF
        if status != STAT_OK:
            return False, None, data
        val_bytes = (data[3:7] + b"\x00\x00\x00\x00")[:4]
        raw = int.from_bytes(val_bytes, "little", signed=False)
        return True, raw, data

    # ---- service-data write ----------------------------------------------
    def sdw(self, node: int, addr: int, value: int, timeout: float, wait_ack: bool = True):
        """Returns (ok, status, reply_bytes).  status is None if no ack seen."""
        v = value & 0xFFFFFFFF
        payload = bytes([addr & 0xFF, (addr >> 8) & 0xFF]) + v.to_bytes(4, "little")
        self._send(p3_id(node, P3_SDW), payload)
        if not wait_ack:
            return True, None, None
        reply = self._wait_for(p1_id(node, P1_ACK_SDW), addr, timeout)
        if reply is None:
            return False, None, None
        data = bytes(reply.data)
        status = data[0] if len(data) >= 1 else 0xFF
        return (status == STAT_OK), status, data

    def announce(self, repeat: int = 5):
        """Register the service-tool node (61) on the bus, like the DST does on
        connect.  Some ECUs — notably the MCB — only grant access to protected
        parameters once we've announced ourselves; without this they reject
        SDR/SDW with STAT_ERRACCESS (status 5).  Mirrors CanLib PERBus:
        SLAVECHANGED x N, then NODEINFO x N.
        """
        # SLAVECHANGED payload: [hw_type, sw_type, ota_ver, cpu_type, FF×4]
        slavechanged = bytes([self.hw_type, 0x02, 0x03, 0x00, 0xFF, 0xFF, 0xFF, 0xFF])
        for _ in range(repeat):
            self._send(p1_id(self.src_node, P1_SLAVECHANGED), slavechanged)
            time.sleep(0.02)
        for _ in range(repeat):
            # NODEINFO broadcast, payload 03 00 (vs 04 00 = reset)
            self._send(p1_id(self.src_node, P1_BROADCAST), bytes([0x03, 0x00]))
            time.sleep(0.02)

    def broadcast_reset(self):
        # P1 BROADCAST from our node, fixed payload 04 00 (BC_RESET)
        self._send(p1_id(self.src_node, P1_BROADCAST), bytes([0x04, 0x00]))


# --------------------------------------------------------------------------
# Value scaling helpers
# --------------------------------------------------------------------------

def interpret(raw: int, p: Param) -> int:
    """Apply width + sign to a raw 32-bit little-endian read."""
    if p.signed:
        bits = (p.width or 4) * 8
        mask = (1 << bits) - 1
        v = raw & mask
        if v & (1 << (bits - 1)):
            v -= (1 << bits)
        return v
    return raw


def scaled_str(value: int, p: Param) -> str:
    s = value * p.scale
    if p.scale == 1.0:
        return f"{value} {p.unit}".strip()
    return f"{s} {p.unit}".strip()


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------

def cmd_list_ports(args, table):
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("no serial ports found")
        return 0
    print("Serial ports (candidate Robotell adapters):")
    for pinfo in ports:
        desc = pinfo.description or ""
        hwid = pinfo.hwid or ""
        hint = "  <-- likely Robotell (CH340)" if "CH340" in (desc + hwid).upper() else ""
        print(f"  {pinfo.device:10}  {desc}  [{hwid}]{hint}")
    return 0


def cmd_list_nodes(args, table):
    print("Known node ids:")
    for nid, name in NODE_NAMES.items():
        print(f"  {nid:3}  {name}")
    return 0


def cmd_list_params(args, table):
    flt = (args.filter or "").upper()
    rows = [p for p in table.all if p.type == "CanValue"]
    if flt:
        rows = [p for p in rows if flt in p.name.upper()]
    print(f"{'Name':40} {'ID':>6}  {'Node':>4}  {'Access':10} {'Unit'}")
    for p in rows:
        print(f"{p.name:40} 0x{p.id:04X}  {p.node:>4}  {p.access:10} {p.unit}")
    print(f"\n{len(rows)} CAN parameter(s)")
    return 0


def cmd_param_info(args, table):
    p = table.resolve(args.param)
    if p is None:
        print(f"parameter not found: {args.param}", file=sys.stderr)
        return 2
    info = {
        "Name": p.name, "ID": p.id, "hexID": f"0x{p.id:04X}", "Type": p.type,
        "Node": p.node, "Access": p.access, "Unit": p.unit, "Scale": p.scale,
        "Width": p.width, "Signed": p.signed, "Min": p.vmin, "Max": p.vmax,
    }
    print(json.dumps(info, indent=2))
    return 0


def _parse_bus_args(items: list[str]) -> dict:
    """Turn ['key=value', ...] into a kwargs dict for can.Bus, coercing obvious
    int/float/bool literals so e.g. --bus-arg receive_own_messages=true works."""
    out = {}
    for item in items or []:
        if "=" not in item:
            sys.exit(f"error: --bus-arg must be KEY=VALUE, got {item!r}")
        key, val = item.split("=", 1)
        low = val.strip().lower()
        if low in ("true", "false"):
            coerced = (low == "true")
        else:
            try:
                coerced = int(val, 0)
            except ValueError:
                try:
                    coerced = float(val)
                except ValueError:
                    coerced = val
        out[key.strip()] = coerced
    return out


def _open(args) -> PerBus:
    extra = _parse_bus_args(getattr(args, "bus_arg", None))
    bus = PerBus(args.interface, args.channel, args.bitrate,
                 args.tty_baud, args.rtscts, extra, args.verbose)
    bus.src_node, bus.hw_type = IDENTITIES[getattr(args, "identity", "tool")]
    # Announce ourselves so access-protected nodes (e.g. the MCB) accept our
    # transactions instead of replying STAT_ERRACCESS.  Skippable with
    # --no-announce for a pure passive listen.
    if not getattr(args, "no_announce", False):
        bus.announce()
    return bus


def cmd_scan(args, table):
    with _open(args) as bus:
        print(f"connected: robotell @ {args.channel}", file=sys.stderr)
        found = {}

        # 1) passive listen — decode source node of any frame already on the bus
        deadline = time.monotonic() + (args.wait / 1000.0 if args.wait else 0.4)
        while time.monotonic() < deadline:
            m = bus.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if m is None:
                continue
            cid = m.arbitration_id
            # P1/P3 split per CanLib PERCanReceiveMsg.filter: bit 0x400 selects
            # the layer. IDs in 0x400..0x5FF are not PER frames — ignore them
            # (otherwise their low bits masquerade as bogus node ids, e.g. 31).
            if cid & 0x400 == 0:                   # P1 broadcast/ack
                node = cid & 0x3F
            elif cid & 0x200:                      # P3 point-to-point
                node = (cid >> 3) & 0x3F
            else:
                continue
            if node and node != bus.src_node:
                found.setdefault(node, "passive")

        # 2) active probe — read a known param on each candidate node
        for node, pid in SCAN_PROBES.items():
            ok, raw, _ = bus.sdr(node, pid, args.timeout / 1000.0)
            if ok:
                found[node] = f"replied (probe 0x{pid:04X}={raw})"

        print(f"Targets ({len(found)}):")
        for node in sorted(found):
            print(f"  node={node:<3} {NODE_NAMES.get(node, '?'):20} {found[node]}")
        if not found:
            print("  (none — bus silent / no node ACKed; check power, wiring, termination)")
    return 0


def cmd_info(args, table):
    node = int(args.node) if args.node is not None else None
    with _open(args) as bus:
        print(f"connected: robotell @ {args.channel}", file=sys.stderr)
        nodes = [node] if node is not None else sorted(SCAN_PROBES)
        for n in nodes:
            params = [p for p in table.all if p.type == "CanValue" and p.node == n and p.readable]
            print(f"node={n} ({NODE_NAMES.get(n, '?')}) — {len(params)} readable param(s)")
            for p in params:
                ok, raw, _ = bus.sdr(n, p.id, args.timeout / 1000.0)
                if ok:
                    val = interpret(raw, p)
                    print(f"  {p.name:38} = {scaled_str(val, p)}")
                else:
                    print(f"  {p.name:38} = <no reply>")
            print()
    return 0


def cmd_read(args, table):
    p = table.resolve(args.param)
    if p is None:
        print(f"parameter not found: {args.param}", file=sys.stderr)
        return 2
    if p.type != "CanValue":
        print(f"robotell-cli can only read CAN parameters, got {p.type} "
              f"(DongleValue params need the TQ dongle)", file=sys.stderr)
        return 2
    node = p.node if p.node >= 0 else int(args.node or 1)
    with _open(args) as bus:
        ok, raw, data = bus.sdr(node, p.id, args.timeout / 1000.0)
        if not ok:
            if data:
                st = data[0]
                print(f"read failed — node {node} returned status {st} "
                      f"(STAT_{CAN_STATUS_NAMES.get(st, '?')})", file=sys.stderr)
            else:
                print("read failed — no ACK_SDR; bus may be silent or HEAVY",
                      file=sys.stderr)
            return 1
        val = interpret(raw, p)
        print(f"{p.name} ID=0x{p.id:04X} node={node}")
        print(f"  raw     : {raw}  (0x{raw:08X})")
        print(f"  value   : {scaled_str(val, p)}")
        print(f"  reply   : {data.hex(' ')}")
    return 0


def cmd_write(args, table):
    p = table.resolve(args.param)
    if p is None:
        print(f"parameter not found: {args.param}", file=sys.stderr)
        return 2
    if p.type != "CanValue":
        print(f"robotell-cli can only write CAN parameters, got {p.type}", file=sys.stderr)
        return 2
    if not p.writable:
        print(f"parameter {p.name} is read-only", file=sys.stderr)
        return 2
    value = int(args.value, 16) if args.value.lower().startswith("0x") else int(args.value)
    if p.vmin is not None and (value < p.vmin or value > p.vmax):
        print(f"value {value} out of range [{p.vmin}..{p.vmax}]", file=sys.stderr)
        return 2
    node = p.node if p.node >= 0 else int(args.node or 1)
    with _open(args) as bus:
        ok, status, data = bus.sdw(node, p.id, value, args.timeout / 1000.0,
                                   wait_ack=not args.no_ack)
        if args.no_ack:
            print(f"OK (fire-and-forget) — sent {value} to {p.name}; no ACK requested")
            return 0
        if ok:
            print(f"OK — wrote {value} to {p.name} (STAT_OK)")
            return 0
        if status is None:
            print(f"FAILED — no ACK_SDW from node {node} (bus silent/HEAVY?)", file=sys.stderr)
        else:
            print(f"FAILED — node {node} returned status {status} "
                  f"(STAT_{CAN_STATUS_NAMES.get(status, '?')})", file=sys.stderr)
        return 1


def cmd_raw_read(args, table):
    node = int(args.node, 0)
    addr = int(args.addr, 16) if args.addr.lower().startswith("0x") else int(args.addr)
    with _open(args) as bus:
        ok, raw, data = bus.sdr(node, addr, args.timeout / 1000.0)
        if not ok:
            if data:
                st = data[0]
                print(f"read failed — node {node} returned status {st} "
                      f"(STAT_{CAN_STATUS_NAMES.get(st, '?')})", file=sys.stderr)
            else:
                print("read failed — no ACK_SDR (bus silent/HEAVY?)", file=sys.stderr)
            return 1
        val_bytes = (bytes(data[3:7]) + bytes(4))[:4]
        i32 = int.from_bytes(val_bytes, "little", signed=True)
        print(f"node={node} addr=0x{addr:04X}")
        print(f"  raw bytes : {data.hex(' ')}")
        print(f"  int32 LE  : {i32}")
        print(f"  uint32 LE : {raw}")
    return 0


def cmd_raw_write(args, table):
    node = int(args.node, 0)
    addr = int(args.addr, 16) if args.addr.lower().startswith("0x") else int(args.addr)
    value = int(args.value, 16) if args.value.lower().startswith("0x") else int(args.value)
    with _open(args) as bus:
        ok, status, data = bus.sdw(node, addr, value, args.timeout / 1000.0,
                                   wait_ack=not args.no_ack)
        if args.no_ack:
            print(f"OK (fire-and-forget) — sent 0x{value:X} to node {node} addr 0x{addr:04X}")
            return 0
        if ok:
            print(f"OK — wrote 0x{value:X} to node {node} addr 0x{addr:04X} (STAT_OK)")
            return 0
        if status is None:
            print(f"FAILED — no ACK_SDW from node {node}", file=sys.stderr)
        else:
            print(f"FAILED — node {node} returned status {status} "
                  f"(STAT_{CAN_STATUS_NAMES.get(status, '?')})", file=sys.stderr)
        return 1


def cmd_live(args, table):
    params = []
    for tok in args.params:
        p = table.resolve(tok)
        if p is None:
            print(f"parameter not found: {tok}", file=sys.stderr)
            return 2
        if p.type != "CanValue":
            print(f"skip {tok}: not a CAN parameter", file=sys.stderr)
            continue
        params.append(p)
    if not params:
        return 2
    period = 1.0 / args.rate if args.rate > 0 else 1.0
    with _open(args) as bus:
        try:
            while True:
                cells = []
                for p in params:
                    node = p.node if p.node >= 0 else int(args.node or 1)
                    ok, raw, _ = bus.sdr(node, p.id, args.timeout / 1000.0)
                    cells.append(f"{p.name}={scaled_str(interpret(raw, p), p)}" if ok
                                 else f"{p.name}=<n/a>")
                print(" | ".join(cells))
                time.sleep(period)
        except KeyboardInterrupt:
            print("\nstopped", file=sys.stderr)
    return 0


def cmd_reset(args, table):
    with _open(args) as bus:
        bus.broadcast_reset()
        print("sent broadcast reset (P1 BROADCAST id 0x{:03X}, payload 04 00)".format(
            p1_id(bus.src_node, P1_BROADCAST)))
    return 0


# --------------------------------------------------------------------------
# CLI wiring
# --------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="robotell-cli",
        description="Talk to a TQ HPR50 e-bike over CAN via any python-can adapter "
                    "(Robotell by default).")
    ap.add_argument("--interface", default=os.environ.get("ROBOTELL_INTERFACE", "robotell"),
                    help="python-can interface: robotell (default), socketcan, slcan, "
                         "pcan, kvaser, ixxat, vector, … (or $ROBOTELL_INTERFACE)")
    ap.add_argument("--channel", default=os.environ.get("ROBOTELL_PORT", "COM4"),
                    help="adapter channel: serial port (COM4, /dev/ttyUSB0) for "
                         "serial dongles, or e.g. can0 / PCAN_USBBUS1 for others")
    ap.add_argument("--bitrate", type=int, default=CAN_BITRATE,
                    help=f"CAN bitrate (default {CAN_BITRATE}); 0 to leave it to the "
                         "interface (e.g. a pre-configured socketcan link)")
    ap.add_argument("--bus-arg", action="append", metavar="KEY=VALUE",
                    help="extra keyword passed straight to can.Bus(), repeatable "
                         "(e.g. --bus-arg receive_own_messages=true); for backend-specific knobs")
    ap.add_argument("--tty-baud", type=int, default=115200,
                    help="USB-serial baud for serial dongles (robotell/slcan; default 115200)")
    ap.add_argument("--rtscts", action="store_true",
                    help="enable hardware flow control (serial dongles)")
    ap.add_argument("--node", default=None, help="default node id for params with no fixed node")
    ap.add_argument("--timeout", type=int, default=2000, help="per-transaction timeout in ms")
    ap.add_argument("--params", default=default_params_path(), help="path to params.json")
    ap.add_argument("-v", "--verbose", action="store_true", help="print TX/RX frames to stderr")
    ap.add_argument("--no-announce", action="store_true",
                    help="don't announce the service-tool node on connect "
                         "(announcing is needed for MCB access-protected params)")
    ap.add_argument("--identity", choices=sorted(IDENTITIES), default="tool",
                    help="identity to announce as: 'tool' (node 61, default) or "
                         "'dongle' (node 60) — dongle may unlock factory-protected params")

    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list-ports", help="list serial ports (candidate adapters)").set_defaults(fn=cmd_list_ports)
    sub.add_parser("list-nodes", help="show known node ids").set_defaults(fn=cmd_list_nodes)

    sp = sub.add_parser("list-params", help="list CAN parameters")
    sp.add_argument("filter", nargs="?", help="substring filter")
    sp.set_defaults(fn=cmd_list_params)

    sp = sub.add_parser("param-info", help="show a parameter definition")
    sp.add_argument("param")
    sp.set_defaults(fn=cmd_param_info)

    sp = sub.add_parser("scan", help="discover nodes on the bus")
    sp.add_argument("--wait", type=int, default=0, help="passive listen window in ms")
    sp.set_defaults(fn=cmd_scan)

    sp = sub.add_parser("info", help="dump readable params for a node (or all known)")
    sp.add_argument("node", nargs="?", help="node id (default: all known nodes)")
    sp.set_defaults(fn=cmd_info)

    sp = sub.add_parser("read", help="read a parameter by name or 0xID")
    sp.add_argument("param")
    sp.set_defaults(fn=cmd_read)

    sp = sub.add_parser("write", help="write a parameter by name or 0xID")
    sp.add_argument("param")
    sp.add_argument("value")
    sp.add_argument("--no-ack", action="store_true", help="fire-and-forget, don't wait for ACK_SDW")
    sp.set_defaults(fn=cmd_write)

    sp = sub.add_parser("raw-read", help="direct SDR: <node> <0xADDR>")
    sp.add_argument("node")
    sp.add_argument("addr")
    sp.set_defaults(fn=cmd_raw_read)

    sp = sub.add_parser("raw-write", help="direct SDW: <node> <0xADDR> <value>")
    sp.add_argument("node")
    sp.add_argument("addr")
    sp.add_argument("value")
    sp.add_argument("--no-ack", action="store_true")
    sp.set_defaults(fn=cmd_raw_write)

    sp = sub.add_parser("live", help="poll one or more params continuously")
    sp.add_argument("params", nargs="+")
    sp.add_argument("--rate", type=float, default=2.0, help="polls per second (default 2)")
    sp.set_defaults(fn=cmd_live)

    sub.add_parser("reset", help="broadcast soft reset to all nodes").set_defaults(fn=cmd_reset)

    return ap


def main(argv=None) -> int:
    # Some parameter units contain non-ASCII chars; avoid cp1252 crashes on Windows.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
    args = build_parser().parse_args(argv)
    try:
        table = ParamTable.load(args.params)
    except FileNotFoundError:
        print(f"error: params.json not found at {args.params} (use --params)", file=sys.stderr)
        return 2
    try:
        return args.fn(args, table)
    except Exception as ex:  # serial/CAN open failures, etc. — report cleanly
        print(f"error: {ex}", file=sys.stderr)
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
