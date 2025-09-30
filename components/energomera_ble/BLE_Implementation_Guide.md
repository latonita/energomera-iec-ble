# BLE Implementation Guide for Energomera CE901v3

## Overview
This guide summarizes how the original Android application talks to the CE901 series meters over BLE. It is distilled from the decompiled sources (`MainActivity.java`, `helpFuncClass.java`, `meckSend.java`) and mirrors the behaviour you must reproduce when porting the logic to another platform (for example, an ESPHome custom component).

## Service Topology
- **Primary service** `b91b0100-8bef-45e2-97c3-1cd862d914df` (`helpFuncClass.MY_NEED_SERVICE`).
- **Version/readiness characteristic** `b91b0101-8bef-45e2-97c3-1cd862d914df`; read immediately after bonding to confirm the device (`MainActivity.java:904`).
- **TX write characteristic** `b91b0105-8bef-45e2-97c3-1cd862d914df`; all commands are written here with `WRITE_TYPE_NO_RESPONSE` (`MainActivity.java:549`).
- **Response bank**: 16 read slots exposed through the array `helpFuncClass.CHARACTERISTIC_READ`. The UUIDs span `b91b0106` through `b91b0114`. Slot indices 0-15 are valid; indices 7 and 8 point to the same UUID in the original array, but the state machine still iterates across all sixteen positions (`MainActivity.java:936`, `helpFuncClass.java:13`).
- **CCCD** `00002902-0000-1000-8000-00805f9b34fb` on the TX characteristic; enables notifications used as a "data ready" flag (`MainActivity.java:918`).

## Connection and Pairing Flow
1. **Scan** - A 60 s BLE scan runs while filtering names that match Energomera models (`MainActivity.java:855`, `MainActivity.java:1755`).
2. **Device selection** - Saved login/PIN (4-6 digits) is loaded from preferences and matched to the chosen device (`MainActivity.java:874`, `MainActivity.java:1672`).
3. **Bonding** - If the GATT status reports `GATT_INSUFFICIENT_ENCRYPTION` or `GATT_INSUFFICIENT_AUTHENTICATION`, the app pushes the PIN with `setPin()` and re-issues `createBond()` (`MainActivity.java:273`). Bond change broadcasts drive retries and advance the timer state machine (`MainActivity.java:448`).
4. **Connect** - The app calls `connectGatt(autoConnect=false, transport=TRANSPORT_LE)`, then immediately raises the connection priority and negotiates an MTU of 240 (`MainActivity.java:656`, `MainActivity.java:284`).
5. **Service discovery** - Triggered inside `onMtuChanged`. When `getServices()` returns empty, the status text reports "no services" and the timer jumps back to state 3 (bonding) via `goTimer(3, PathInterpolatorCompat.MAX_NUM_POINTS)` (`MainActivity.java:330`, `MainActivity.java:333`). When the desired service is located, a short delay either re-enters bonding (if not yet bonded) or proceeds to the version read (`MainActivity.java:336`, `MainActivity.java:343`).
6. **Version read** - Characteristic `0101` is read to confirm firmware and meter identity (`MainActivity.java:904`).
7. **Enable notifications** - The TX characteristic is set to notify and the CCCD is written (`MainActivity.java:918`). The code does **not** wait for `onDescriptorWrite`; instead it schedules the next timer step 500 ms later and moves straight into the command loop.
8. **Command state machine** - Timer step 6 invokes `send_Message`, which serializes Energomera protocol frames, writes them to the TX characteristic, and waits for the notification-driven read cycle to finish before queuing the next command (`MainActivity.java:539`, `MainActivity.java:1028`).

## Command Serialization
- Frames originate from `meckSend.makeReq()` and cover 75 different requests (watch, LOE, tariff blocks, daily/monthly logs, etc.).
- Every payload byte is converted to 7-bit data with odd parity applied in `helpFuncClass.HELP_MakeParity()`.
- A sequencing header `first_last` is prepended. Bit 7 marks the final fragment; lower bits act as a rolling packet counter. With MTU > 23 the first packet is flagged `0x00`, intermediate fragments increment by one, and the final fragment sets bit 7 (`MainActivity.java:565`, `MainActivity.java:997`).
- MTU framing: the Android code sends 19 payload bytes per fragment (20 total with header) and rebuilds the remainder array until all chunks are written.
- Before each write, `go_rTimer(1)` starts a 10 s watchdog. Missing responses trigger up to five retransmissions; repeated failures reset the workflow back to scanning (`MainActivity.java:608`, `MainActivity.java:1947`).

## Notification and Readback Loop
1. After a command write completes, the meter responds by toggling the TX notification. The first byte of that notification (`need_read`) states how many response characteristics must be pulled (`MainActivity.java:355`).
2. `goTimer(step + 1, 50)` schedules the sequential reads. Steps 7-22 map to read handles #0-#15. The loop accepts indices up to 15; higher indices raise "Error 9" (`MainActivity.java:936`).
3. Each `readCharacteristic` masks the parity bit and appends the 7-bit data into `in_buf` (`MainActivity.java:380`).
4. Once `step > need_read + 6`, the app validates that the frame ends with ETX (`0x03`), trims the STX/ETX/checksum envelope, and dispatches to `message_parsing()` (`MainActivity.java:1842`, `MainActivity.java:1854`).
5. Parsers inside `meckGetVars` populate internal structures for watch data, load profiles, tariffs, and diagnostic counters. Multi-part responses such as `LST01`/`LST02` are concatenated until all chunks arrive (`MainActivity.java:2068`).

## Error Handling and Recovery
- **Bond failures** - `bond_none_cnt` permits five failed attempts before preferences are cleared and the workflow drops back to the PIN-entry screen (`MainActivity.java:467`, `MainActivity.java:523`).
- **GATT disconnects** - Status codes 133 and 22 reset timers and bounce to scanning or reconnection as appropriate (`MainActivity.java:296`).
- **No services** - As noted above, service discovery failure routes through `goTimer(3, PathInterpolatorCompat.MAX_NUM_POINTS)` rather than calling `createBond()` directly.
- **Watchdog** - `go_rTimer(0)` fires after 15 s without progress, forcing a reconnection attempt or full reset (`MainActivity.java:1926`).

## Request Catalogue (selected entries)
Refer to `meckSend.java` for the exhaustive list; highlights include:

ID | Constant | Energomera query (ASCII before parity)
-- | -------- | -------------------------------------
0  | `ASK_EMD_WATCH` | `/?!` SOH `R1` STX `GRPNM(WATCH()ID_FW()PROFI()EMD01(0.0,FF))` ETX NUL
1  | `ASK_PARAM_LOE` | `/?!` SOH `R1` STX `GRPNM(LOE28(1)LOE29(1)LOE30(1)CORIU()FREQU()SNUMB())` ETX NUL
2  | `ASK_PARAM_POWERS` | `/?!` SOH `R1` STX `GRPNM(POWES()POWEQ()CORUU())` ETX NUL
3  | `ASK_PARAMS` | `/?!` SOH `R1` STX `GRPNM(CRCPR()VOLTA()CURRE()POWEP()STAT_())` ETX NUL
4  | `ASK_PREDELU` | `/?!` SOH `R1` STX `GRPNM(LSU01()LSU02()MODEL())` ETX NUL
5-10 | `ASK_LST02_x` | Logbook segments `LST02(start,count)`
11-26 | `ASK_LST01_x` | Load profile pages `LST01(start,count)`
27 | `ASK_PARTS0` | Group `GRPNM(TSUWI()V2501(1,1)EMD01(0.1,0)EMM01(0.1,0))`
28-42 | `ASK_PARTS` variants | Daily and monthly energy group slices
43+ | `ASK_TAR` | Tariff tables `GRF01(n)` where `n = askId - 42`

## Implementation Checklist for ESPHome
1. **BLE transport** - Connect, request MTU 240, and model the state machine with equivalent timers.
2. **Bonding** - Provide the stored PIN during pairing retries. Abort after five failures to mimic Android behaviour.
3. **Notifications** - Enable the TX characteristic's CCCD and advance the state machine after a short delay; do not rely on `onDescriptorWrite`.
4. **Command framing** - Add odd parity, fragment into 19-byte segments, and honour the `first_last` semantics.
5. **Response reading** - Use the TX notification byte as the count, read each characteristic in order, and verify the ETX terminator before parsing.
6. **Retries** - Re-send commands after 10 s if no response, and escalate to a full reconnect once the retry counter expires.
7. **Service loss** - If services disappear, close the GATT link, re-enter the bonding step, and restart the workflow.

## Key Sources
- `MainActivity.java` - BLE state machine, timers, command loop.
- `helpFuncClass.java` - UUID constants, parity helper, vendor filters.
- `meckSend.java` - Energomera command catalogue and checksum generation.
- `meckGetVars.java` - Response parsers for watch, parameters, profiles, and tariffs.

