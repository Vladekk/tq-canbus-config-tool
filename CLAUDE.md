# tq-canbus-config-tool — manual

A small Python tool that talks to a **TQ HPR50 e-bike** over CAN using a generic
**Robotell USB‑CAN adapter** (via [`python-can`](https://python-can.readthedocs.io/en/stable/interfaces/robotell.html)).
It speaks the TQ **PER protocol directly on the wire** — so it needs **no TQ
software stack and no TQ dongle**.

It reads from / writes to the bike's CAN nodes and parameters by loading
`params.json` (in this repo, next to the script) for name ↔ ID / node / range /
scaling resolution.

> See [`CAN.md`](CAN.md) for the full PER‑protocol description these frames are built from.

---

## 1. Requirements & install

```bash
pip install -r requirements.txt        # python-can + pyserial
# or:  pip install python-can pyserial
```

- Python 3.10+.
- A Robotell USB‑CAN stick (STM32 + CH340, binary protocol). Shows up as a
  serial port (`COMx` on Windows, `/dev/ttyUSBx` on Linux).
- The bike’s CAN bus must be wired to the adapter’s CAN_H / CAN_L, properly
  terminated (~120 Ω), and at least one **powered, awake** node must be on the
  bus to ACK frames (see *Limitations*).

The adapter stores the CAN bitrate persistently; this tool always configures it
for the HPR50 bus: **500 kbit/s, standard 11‑bit frames**.

---

## 2. Quick start

```bash
# run with no command to auto-detect adapters and get a ready-to-run command line
python tq_canbus_config.py

# find your adapter's serial port
python tq_canbus_config.py list-ports

# point every command at it with --channel (or set TQ_CANBUS_PORT)
python tq_canbus_config.py --channel COM4 scan
python tq_canbus_config.py --channel COM4 read BATT_SOC
python tq_canbus_config.py --channel COM4 write SB_OUT1 1
```

On Linux: `--channel /dev/ttyUSB0`. You can also embed the serial baud:
`--channel COM4@115200`.

---

## 3. Global options

| Option | Default | Meaning |
|---|---|---|
| `--interface <name>` | `robotell` (or `$TQ_CANBUS_INTERFACE`) | python‑can backend: `robotell`, `socketcan`, `slcan`, `pcan`, `kvaser`, `ixxat`, `vector`, … |
| `--channel <chan>` | `COM4` (or `$TQ_CANBUS_PORT`) | Adapter channel: serial port (`COM4`, `/dev/ttyUSB0`) for serial dongles, or e.g. `can0` / `PCAN_USBBUS1` for others |
| `--bitrate <n>` | `500000` | CAN bitrate; pass `0` to leave it to the interface (e.g. a pre‑configured socketcan link) |
| `--bus-arg <K=V>` | — | Extra keyword passed straight to `can.Bus()`, repeatable — for backend‑specific knobs (e.g. `--bus-arg receive_own_messages=true`) |
| `--tty-baud <n>` | `115200` | USB‑serial baud for serial dongles (`robotell`/`slcan`); not the CAN bitrate |
| `--rtscts` | off | Enable serial hardware flow control (serial dongles) |
| `--node <id>` | — | Default node id for params that have no fixed node |
| `--timeout <ms>` | `2000` | Per‑transaction wait for an ACK |
| `--params <path>` | `params.json` (next to the script) | Parameter table to load |
| `--identity <who>` | `tool` | Announce as `tool` (node 61), `dongle` (node 60), or a raw node to impersonate — `<node>` or `<node>:<hw_type>` (decimal/`0x`-hex, e.g. `17` or `21:0xF0`); see §6 |
| `--no-announce` | off | Skip the connect‑time SLAVECHANGED/NODEINFO announce (pure passive listen) |
| `-v, --verbose` | off | Print the bus open kwargs + every TX/RX CAN frame to stderr |

> **Any python‑can adapter works** — the HPR50 PER protocol is the same on the wire
> regardless of dongle. Only `--interface robotell`/`slcan` use `--tty-baud`/`--rtscts`;
> for other backends pass their own options via `--bus-arg`. Examples:
> ```bash
> # SocketCAN (Linux), interface already up at 500k via `ip link`
> python tq_canbus_config.py --interface socketcan --channel can0 --bitrate 0 scan
> # PEAK PCAN-USB
> python tq_canbus_config.py --interface pcan --channel PCAN_USBBUS1 read BATT_SOC
> # SLCAN (Lawicel/CANUSB)
> python tq_canbus_config.py --interface slcan --channel /dev/ttyACM0 scan
> ```

---

## 4. Commands

### Discovery / reference (work offline, no adapter needed)
| Command | Description |
|---|---|
| *(no command)* | Auto-detect serial ports and print a ready-to-run command line for each candidate adapter (multiple ports → one suggestion per port, so you can pick) |
| `list-ports` | List serial ports; flags likely Robotell (CH340) adapters |
| `list-nodes` | Print the known node‑id table |
| `list-params [filter]` | List CAN parameters (optional name substring filter) |
| `param-info <name\|0xID>` | Show one parameter’s full definition (node, range, width, scale) |

### Bus operations (need the adapter + bike)
| Command | Description |
|---|---|
| `scan [--wait <ms>]` | Discover nodes: passively listen, then actively probe known nodes |
| `selftest [--wait <ms>]` | **Is the adapter working?** Probes the adapter's own MCU over USB (via the Robotell config channel — independent of the CAN bus), then passively listens, and prints a verdict that tells **adapter dead** apart from **bus silent**. Use this first when you see no traffic at all |
| `monitor [--duration <ms>] [--raw] [--unknown-only] [--log <file>]` | Passively sniff the bus and print/decode **every** frame other nodes send (PER layer, PDR/PDW/SDR/SDW + ACKs, source/target node, param name, value/status). Never transmits. On exit prints a per‑id summary with a **changed‑byte map** (`X`=byte varied, `.`=constant) — the lever for reversing unknown frames. `--node N` filters to one node; `--unknown-only` shows just the non‑PER (`0x400`–`0x5FF`) foreign frames; `--log <file>` dumps `<t> <id> <hex>` for offline analysis; `--raw` skips decoding; Ctrl‑C to stop |
| `info [<node>]` | Read & print all readable params of a node (or all known nodes) |
| `read <name\|0xID> [--node N]` | Read one parameter via **SDR**, scaled to its unit |
| `write <name\|0xID> <value> [--no-ack]` | Write one parameter via **SDW**, range‑checked |
| `raw-read <node> <0xADDR>` | Direct SDR to any node/address; prints raw bytes + int views |
| `raw-write <node> <0xADDR> <value> [--no-ack]` | Direct SDW to any node/address |
| `live <name…> [--rate Hz]` | Poll one or more params continuously (Ctrl‑C to stop) |
| `reset` | Broadcast soft reset to all nodes (PER `BC_RESET`) |

`value` accepts decimal or `0x` hex. `--no-ack` sends fire‑and‑forget (no wait
for the write acknowledgement) — handy when the bus is degraded.

---

## 5. Examples

```bash
# Is the adapter even working, or is the bus just silent?
python tq_canbus_config.py --channel COM4 selftest

# Discover what's on the bus
python tq_canbus_config.py --channel COM4 scan --wait 1500

# Passively watch everything other nodes put on the bus (decoded), Ctrl-C to stop
python tq_canbus_config.py --channel COM4 monitor
# Only the BMS (node 17), capture 5 s, raw frames (no decode)
python tq_canbus_config.py --channel COM4 --node 17 monitor --duration 5000 --raw
# Hunt a foreign/unknown protocol (e.g. a non-TQ BMS): show only non-PER frames,
# log them, and read the changed-byte map in the exit summary to reverse them
python tq_canbus_config.py --channel COM4 monitor --unknown-only --log bms.log

# Read battery state of charge (BMS, node 17)
python tq_canbus_config.py --channel COM4 read BATT_SOC

# Enable Smartbox aux output 1 (node 46) permanently  (1=ON, 0=OFF, 3=switchable)
python tq_canbus_config.py --channel COM4 write SB_OUT1 1

# Same write, addressed manually instead of by name
python tq_canbus_config.py --channel COM4 raw-write 46 0x3024 1

# Read it back raw
python tq_canbus_config.py --channel COM4 raw-read 46 0x3024

# Live‑watch a couple of values at 5 Hz
python tq_canbus_config.py --channel COM4 live BATT_SOC MCB_RT_LOG_MILEAGE --rate 5

# See the actual frames going out/in
python tq_canbus_config.py --channel COM4 -v read BATT_SOC
```

---

## 6. How it maps to the PER protocol

All multi‑byte fields are little‑endian. (`canid` formulas from [`CAN.md`](CAN.md).)

| Operation | TX frame | Expected reply |
|---|---|---|
| **read** (SDR) | id `0x600 \| node<<3 \| 2`, payload `addr_lo addr_hi` | `ACK_SDR` id `8<<6 \| node`, payload `status addr_lo addr_hi v0 v1 v2 v3` |
| **write** (SDW) | id `0x600 \| node<<3 \| 3`, payload `addr_lo addr_hi v0 v1 v2 v3` | `ACK_SDW` id `10<<6 \| node`, payload `status addr_lo addr_hi` |
| **reset** | id `15<<6 \| 61` = `0x3BD`, payload `04 00` | (none) |

- In the reply, **`status` is the first byte**, the 16‑bit address follows at
  `reply[1:3]`, and a read’s value is `int32(reply[3:7])` — then sign‑adjusted by
  the parameter’s width (`SixteenBix` → 16‑bit signed) and multiplied by its
  `UnitScalingFactor`.
- **`status == 1` (`STAT_OK`) means OK** — not 0. Other values are `E_CanStatus`
  error codes (2 ERRCAN, 3 ERRADDRESS, 4 ERRRANGE, 5 ERRACCESS, …).

**Identity / node impersonation.** Only the connect-time announce frames
(`SLAVECHANGED`/`NODEINFO`, P1 ids) carry a *source* node — `SDR`/`SDW`
read/write frames address a target and carry no source, so there is nothing to
spoof there. `--identity` sets which node those announce frames claim to be:
`tool` (61) and `dongle` (60) are the two TQ service identities, and any raw
node id (`0`–`63`, optionally `:<hw_type>`) lets the tool register on the bus as
that node — e.g. `--identity 17` announces as the BMS. Whether a target ECU
honours a non-service identity is firmware-dependent.

Worked example — `write SB_OUT1 1` (Smartbox node 46, param `0x3024`):
sends `0x773  24 30 01 00 00 00`, expects `0x2AE  01 24 30` (status `01` =
`STAT_OK`). These are byte‑for‑byte the same frames the TQ tool emits.

---

## 7. Limitations (what it intentionally can't do)

These features need the TQ dongle's own hardware/firmware and are
**intentionally not in this tool** — a Robotell is a plain CAN transceiver and
cannot do them:

| Missing feature | Why |
|---|---|
| `power-on` / `wake` | Pulses the dongle’s `RT_FET_ENABLE_BAT_WAKEUP_PD_GND_680` FET — TQ hardware only. A Robotell cannot wake a sleeping bike; wake it another way (HMI power button), or keep the TQ dongle for the wake pulse. |
| `diag` + DongleValue reads | `RT_ADC_VOLTAGE_12V/48V/CAN_HIGH/CAN_LOW`, etc. are the TQ dongle’s onboard ADC sensors, reported over its USB protocol — not CAN. No equivalent on a Robotell. |
| `start-app` / bootloader flows | Rely on TQ’s bootloader transaction layer. |
| `web-status` / `web-client-info` | TQ cloud web services — unrelated to CAN; out of scope for this CAN tool. |
| Rich `info` (firmware/bootloader versions, serials) | The TQ tooling gets these via TQ’s `TargetLib` enumeration. Here `info` instead dumps the readable parameters of a node. |

**Shared bus realities (same for any adapter):**
- The bus must **ACK** your frames. A lone adapter on a silent/half‑broken bus
  goes error‑passive/bus‑off and nothing is delivered — exactly like the TQ
  dongle reporting `HEAVY`. You need a healthy CAN pair + a powered, awake node.
- A broken/swapped CAN_H/CAN_L wire fails identically on every adapter; swapping
  to a Robotell does **not** work around a harness fault.

---

## 8. Node id reference

| ID | Node | | ID | Node |
|----|------|-|----|------|
| 1  | MCB / motor master | | 32 | Light |
| 16 | BMS (legacy) | | 44 | Charger |
| 17 | BMS HPR50 | | 46 | Smartbox |
| 18 | Charger port | | 60 | Service Dongle |
| 20 | Range extender | | 61 | Service Tool (this PC) |
| 21 | Display (HMI) | | | |

CAN parameters in `params.json` are defined for nodes **1, 17, 20, 21, 46**.

---

## 9. Parameter reference (what you can read / set)

`params.json` defines **78 CAN parameters**. The tables below list the useful ones
with their hex address, declared **Access**, valid range, and meaning.

**Read this first — declared access ≠ what the firmware allows:**

- **`Access` is the param table's declaration, not a runtime guarantee.** Several
  `ReadWrite` parameters are **firmware/access-protected**: the node answers an
  SDR/SDW with `STAT_ERRACCESS` (status 5) regardless of `--identity`. Confirmed
  for `MCB_TIRE_CIRC*`; the `BATT_*` writes are similarly BMS/factory-gated. Treat
  "settable?" below as *best-known*, and trust what the node actually returns.
- **Reads of access-protected params can also fail** with `STAT_ERRACCESS` — it's
  not write-only protection.
- Values are scaled per the param's `UnitScalingFactor`/width; `read`/`write`
  handle that, `raw-read`/`raw-write` don't.

Legend: **R** = read-only · **R/W** = read+write per table · 🔒 = write (and often
read) rejected by firmware in the field.

### Node 1 — MCB / motor master

Config:

| Name | Addr | Access | Range | Meaning |
|---|---|---|---|---|
| `MCB_TIRE_CIRC` | `0x2601` | R/W 🔒 | 1800–2500 mm | Rear wheel size (rolling circumference). Feeds speed → affects odometer **and** the 25 km/h assist cutoff. **Factory-locked** (`STAT_ERRACCESS`). |
| `MCB_TIRE_CIRC_ALT` | `0x260A` | R/W 🔒 | 1800–2500 mm | Alternate wheel size (0 if unused). Factory-locked. |
| `MCB_TIRE_CIRC_SEL` | `0x260B` | R/W 🔒 | 1800–2500 mm | Selects active size = one of the two stored values. The only tyre-circ knob the DST exposes; still access-gated on locked bikes. |

Telemetry / counters (all **R**): `MCB_RT_WHEELSPEED` (`0x1602`),
`MCB_RT_LOG_MILEAGE` (`0x1C01`, Mileage), `MCB_RT_LOG_OPERATING_TIME` (`0x1C00`),
`MCB_RT_LOG_ACTIVATIONS` (`0x1C02`), `MCB_RT_LOG_WHEELSPEED_MAX` (`0x1C03`, km/h),
`MCB_RT_LOG_PEDALSPEED_MAX/MEAN` (`0x1C04/05`, rpm), `MCB_RT_LOG_HUMAN_PWR_MEAN`
(`0x1C0B`, avg rider power), `MCB_RT_LOG_BAT_ENERGY_KM` (`0x1C0C`, Wh/km),
`MCB_RT_LOG_TEMP_{CPU,PCB,FET,MOT}_{MIN,MAX}` (`0x1C10`–`0x1C17`, °C),
`MCB_RT_PEDALDISTANCE` (`0x1606`), `MCB_RT_LOG_MOTORROTATIONS` (`0x1C0A`),
encoder diagnostics `RT_ENC_*` / `MCB_RT_ENC_HALL_DEVIATION` (`0x16B0`–`0x16BA`),
and `MCB_USR_DM_1..5_MOTOR_TUNE` (`0x6011`–`0x6015`, read-only tune slots).

### Node 17 — BMS HPR50

| Name | Addr | Access | Range | Meaning |
|---|---|---|---|---|
| `BATT_SOC` | `0x3113` | R | % | State of charge |
| `BATT_SOH` | `0x3143` | R/W 🔒 | 0–100 % | State of health (BMS-gated) |
| `BATT_CHARGE_CYC` | `0x31FF` | R | 0–65535 | Charge cycles |
| `BATT_FULL_CAP` | `0x3134` | R/W 🔒 | 0–10000 | Full capacity (BMS/factory) |
| `BATT_DESIGN_CAP` | `0x397A` | R/W 🔒 | 0–10000 | Design capacity (BMS/factory) |
| `BATT_SOHREADY` | `0x3208` | R/W 🔒 | 0/1 | SOH-ready flag |
| `BATT_LIGHT_BACKUP` | `0x3220` | R/W | 0/1 | Light backup buffer from main battery |
| `BATT_KEEP_ALIVE` | `0x410F` | R/W | 0/1 | Keep-alive |

### Node 20 — Range extender

| Name | Addr | Access | Meaning |
|---|---|---|---|
| `RAEXT_SOC` | `0x3113` | R | Range-extender state of charge (%) |

### Node 21 — Display (HMI) — the genuinely user-settable group

| Name | Addr | Access | Range | Meaning |
|---|---|---|---|---|
| `DISP_WALK_ASSIST` | `0x6002` | R/W | 0/1 | Walk-assist (push-assist ~6 km/h) on/off |
| `DISP_UNITS` | `0x6003` | R/W | 0/1 | Units: metric ↔ imperial toggle (polarity unverified — read it set both ways to confirm) |
| `DISP_SOUNDS` | `0x6004` | R/W | 0/1 | UI sounds on/off |
| `DISP_CENTERBUTTON_MODE` | `0x6010` | R/W | 0/1 | Center button w/ remote |
| `DISP_BUTTON_CONFIG` | `0x3400` | R/W | 0–7 | Button configuration |
| `DISP_BE_OUT1` | `0x6050` | R/W | 0–5 | Bar-end display output 1 mode |
| `DISP_BE_STEALTH` | `0x6052` | R/W | 0/1 | Stealth mode (bar-end) |
| `DISP_ANT_ID` | `0x6053` | R/W | 0–65535 | ANT+ id |
| `DISP_ALTINFO_1..10` | `0x6020`+ | R/W | 0–8 | Data fields shown on the home screen pages (incl. `DISP_FC_ALTINFO_*_P/S` full-colour primary/secondary variants at the same addresses) |

### Node 46 — Smartbox

| Name | Addr | Access | Range | Meaning |
|---|---|---|---|---|
| `SB_OUT1` | `0x3024` | R/W | 0–3 | Aux output 1: **0**=off, **1**=on, **3**=switchable |
| `SB_OUT2` | `0x3022` | R/W | 0–3 | Aux output 2 (same encoding) |
| `SB_OUT3` | `0x3023` | R/W | 0–3 | Aux output 3 (same encoding) |
| `SB_ESHIFT` | `0x3021` | R/W | 0/1 | Electronic shifting (E-Shift) enable |

> Tip: `python tq_canbus_config.py list-params [filter]` prints the live table from
> `params.json`, and `param-info <name>` shows one param's full definition
> (node, range, width, scale). Those are the source of truth if `params.json`
> changes; this section is a curated, annotated subset.

---

## 10. License

MIT — see [`LICENSE`](LICENSE). © 2026 Vladislavs Kugelevics.
