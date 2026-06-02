# Sending Raw CAN Data to an HPR50 Bike (PER Protocol)

This document describes the low-level CAN protocol used by the TQ HPR50 e-bike so
you can talk to it from **any** CAN interface — not just the TQ Dongle. The dongle
is only a USB→CAN bridge; everything below is what actually appears on the wire.

Sources: decompiled `CanLib` (`PERCanOutMsg`, `PERMessagesP1/P3`, `PERCanSDW`,
`CanMeasurementProvider`, `GenericCanOutMsg`).

---

## 1. Physical layer

| Property        | Value                                             |
|-----------------|---------------------------------------------------|
| Frame format    | Standard 11-bit identifiers (no extended IDs)     |
| Bit rate        | **500 kbit/s** (fixed — see `PERBus.Connect` → `Open(500000)`) |
| CAN-FD          | No                                                |
| Termination     | 120 Ω at both ends (HPR50 harness usually terminates internally) |
| Byte order      | **Little-endian** for all multi-byte fields       |

Any CAN adapter works: PEAK PCAN-USB, Kvaser, Lawicel/CANUSB (SLCAN),
ESP32 + MCP2515, Raspberry Pi + MCP2515 (SocketCAN), Arduino + MCP2515, etc.

---

## 2. CAN ID encoding

The protocol has two layers, **P1** (broadcast / acknowledgements) and **P3**
(point-to-point service/process data). They encode the 11-bit ID differently.

### P3 — request to an ECU (the layer you use for read/write)

```
canid = 0x600 | (nodeId << 3) | msgId
```

| msgId | Name | Meaning                          |
|-------|------|----------------------------------|
| 0     | PDR  | Process-data read                |
| 1     | PDW  | Process-data write               |
| 2     | SDR  | **Service-data read** (read param)  |
| 3     | SDW  | **Service-data write** (write param) |
| 5     | DATA | Continuation chunk (large transfers) |

### P1 — broadcast / acknowledgement from an ECU

```
canid = (msgId << 6) | nodeId
```

| msgId | Name         | Meaning                       |
|-------|--------------|-------------------------------|
| 0     | EMCY_OFF     | Emergency cleared             |
| 1     | EMCY         | Emergency / error report      |
| 7     | ACK_PDR      | Ack of process-data read      |
| 8     | ACK_SDR      | Ack of service-data read      |
| 9     | ACK_PDW      | Ack of process-data write     |
| 10    | ACK_SDW      | Ack of service-data write     |
| 14    | BROADCAST    | Bus broadcast                 |
| 15    | SLAVECHANGED | Node announces itself         |

---

## 3. Node IDs

| ID  | Node                       |
|-----|----------------------------|
| 1   | MCB / motor master         |
| 16  | BMS (legacy)               |
| 17  | BMS HPR50                  |
| 18  | Charger port               |
| 20  | Range extender             |
| 21  | Display (HMI)              |
| 32  | Light                      |
| 44  | Charger                    |
| 46  | Smartbox                   |
| 60  | Service Dongle             |
| 61  | Service Tool (the PC/you)  |

---

## 4. Payload layout

All payloads are little-endian. SDR/SDW start with the 16-bit **parameter ID**
(a.k.a. address/`Adr`), low byte first.

| Direction              | DLC | Bytes                                          |
|------------------------|-----|------------------------------------------------|
| **SDW request**        | 6   | `Adr_lo, Adr_hi, val0, val1, val2, val3`        |
| SDW reply (ACK_SDW)    | 3   | `status, Adr_lo, Adr_hi`                        |
| **SDR request**        | 2   | `Adr_lo, Adr_hi`                                |
| SDR reply (ACK_SDR)    | 7   | `status, Adr_lo, Adr_hi, val0, val1, val2, val3`|

- **In the reply, `status` is the FIRST byte**, the 16-bit address follows at
  bytes `[1:3]`, and the value (SDR) at bytes `[3:7]`. (Verified against
  `CanLib.SimpleTransaction.CheckReceiveMsg`: it matches `readUInt16(1) == addr`,
  reads status from `readUInt8(0)`, then `seek(3)` for the value.)
- **`status == 1` (`STAT_OK`) means success — not 0.** Other values are errors
  from `E_CanStatus`: 2 ERRCAN, 3 ERRADDRESS, 4 ERRRANGE, 5 ERRACCESS,
  6 ERRCMDVAR, 7 ERRUNSPEC, 8 ERRDATA, 9 BUSY, 10 PENDING, 11–13 OK_DATA_*.
- The 32-bit value is little-endian (`val0` is least-significant). The official
  `GetValue` reads it as `ToInt32(data, 3)` (value starts at offset 3, after the
  status byte + 2 address bytes).

> Note: the official tool always sends 4 value bytes for an SDW even for 16-bit
> parameters (the unused high bytes are 0). Replicating that is safest.

---

## 5. Worked example — enable all three Smartbox aux channels

Smartbox = node **46**. SDW CAN ID = `0x600 | (46 << 3) | 3` = **0x773**.

Parameters (`SB_OUTx`, values: `0`=OFF, `1`=ON permanent, `3`=SWITCHABLE):

| Param   | Param ID | Adr bytes (LE) |
|---------|----------|----------------|
| SB_OUT1 | 0x3024   | `24 30`        |
| SB_OUT2 | 0x3022   | `22 30`        |
| SB_OUT3 | 0x3023   | `23 30`        |

### Frames to write value `1` (permanently ON)

| What        | CAN ID | DLC | Payload (hex)        |
|-------------|--------|-----|----------------------|
| SB_OUT1 = 1 | 0x773  | 6   | `24 30 01 00 00 00`  |
| SB_OUT2 = 1 | 0x773  | 6   | `22 30 01 00 00 00`  |
| SB_OUT3 = 1 | 0x773  | 6   | `23 30 01 00 00 00`  |

For SWITCHABLE (display-controlled) use value `3`: e.g. `24 30 03 00 00 00`.

### Expected reply from the Smartbox

ACK_SDW CAN ID = `(10 << 6) | 46` = **0x2AE**, DLC 3:

```
0x2AE  01 24 30      # status 01 = STAT_OK, then addr 0x3024
```

### Reading a channel back

SDR CAN ID = `0x600 | (46<<3) | 2` = **0x772**:

| What         | CAN ID | DLC | Payload  |
|--------------|--------|-----|----------|
| Read SB_OUT1 | 0x772  | 2   | `24 30`  |

Reply on ACK_SDR `(8<<6)|46` = **0x22E**, DLC 7:

```
0x22E  01 24 30 01 00 00 00     # status 01 (STAT_OK), addr 0x3024, value = 0x00000001 (ON)
```

---

## 6. Announcing yourself (required for access-protected params)

The official DST announces itself on connect: it sends `SLAVECHANGED` from node 61
~10×, then `NODEINFO` ~5× (`CanLib.PERBus._sendSlaveChangedMessage` /
`PERCanBCNodeInfoMsg`).

For **unprotected** parameters this is optional — the BMS, Smartbox and Display
answer SDR/SDW from anyone. But the **MCB gates access-protected parameters**
(e.g. `MCB_TIRE_CIRC*`) on it: until node 61 has announced itself, the MCB
rejects even a *read* of those with `STAT_ERRACCESS` (status 5). So announce
before talking to the MCB.

```
SLAVECHANGED:  CAN ID = (15 << 6) | 61 = 0x3FD,  DLC 8,  Payload: F0 02 03 00 FF FF FF FF
NODEINFO:      CAN ID = (14 << 6) | 61 = 0x3BD,  DLC 2,  Payload: 03 00
```

(`0x3BD` is the broadcast id; payload byte 0 distinguishes it: `03` = NODEINFO,
`04` = `BC_RESET`.)

---

## 7. Practical gotchas

1. **The bus must ACK your frames.** A lone transmitter on an otherwise-silent or
   unterminated bus immediately goes error-passive (the CLI reports `Bus status:
   HEAVY`). Make sure at least one other powered node (HMI / Smartbox / MCB) is on
   the bus, or enable "ack/normal mode" on a second interface.
2. **Power.** HMI and Smartbox are normally fed 12 V from the bike harness (battery
   side). With the motor unplugged you still need the battery or a bench 12 V supply
   feeding the harness, or those nodes never come up.
3. **Termination.** Add a 120 Ω terminator if you splice your own adapter into a
   harness that isn't self-terminated on your branch.
4. **Endianness.** `0x3024` on the wire is `24 30`. Both the address and the value
   are little-endian.
5. **No application-layer checksum.** Only the standard CAN frame CRC protects the
   data; a frame arrives intact or not at all.
6. **Permanent vs switchable.** `SB_OUTx = 1` forces the output on whenever the bike
   is powered; `= 3` routes it through the display's light toggle.

---

## 8. SocketCAN quick recipe (Linux)

```bash
# bring up the interface at 500k
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# enable all three Smartbox channels
cansend can0 773#243001000000     # SB_OUT1 = 1
cansend can0 773#223001000000     # SB_OUT2 = 1
cansend can0 773#233001000000     # SB_OUT3 = 1

# watch for ACKs / replies from the Smartbox
candump can0,2AE:7FF   &           # ACK_SDW
candump can0,22E:7FF   &           # ACK_SDR

# read SB_OUT1 back
cansend can0 772#2430
```

`python-can` equivalent:

```python
import can
bus = can.Bus(channel="can0", interface="socketcan", bitrate=500000)

def sdw(node, param_id, value):
    canid = 0x600 | (node << 3) | 3
    data = bytes([param_id & 0xFF, (param_id >> 8) & 0xFF]) + value.to_bytes(4, "little")
    bus.send(can.Message(arbitration_id=canid, data=data, is_extended_id=False))

for pid in (0x3024, 0x3022, 0x3023):   # SB_OUT1..3
    sdw(46, pid, 1)                     # 1 = permanently ON
```

---

## 9. Mapping to the CLI

The `tq_canbus_config.py` tool builds exactly these frames:

| CLI command                                    | Frame produced                         |
|------------------------------------------------|----------------------------------------|
| `tq_canbus_config.py write SB_OUT1 1`          | SDW 0x773 `24 30 01 00 00 00`, waits for ACK_SDW |
| `tq_canbus_config.py write SB_OUT1 1 --no-ack` | same frame, fire-and-forget (no ACK wait) |
| `tq_canbus_config.py raw-write 46 0x3024 1`    | identical bytes, addressed manually     |
| `tq_canbus_config.py read SB_OUT1`             | SDR 0x772 `24 30`, parses ACK_SDR reply |

Use `--no-ack` only when you understand the bus may not ACK (no confirmation that the
write landed). See `--help` for details.
