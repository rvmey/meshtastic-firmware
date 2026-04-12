# GPS Store-and-Forward Module — Design & Implementation Plan

## Problem Statement

The nRF52840-based T1000-R tracker has no WiFi. GPS positions are broadcast over LoRa
every 5 seconds and forwarded to an MQTT broker only when a gateway node is within LoRa
range. Positions produced while no gateway is in range are lost permanently.

This document describes a `GpsStoreForwardModule` that logs every GPS fix to LittleFS
flash and replays the full log to a trusted gateway node when one comes into range.

---

## Architecture

```
  GPS (every 5s)
       │  position packet (local loopback)
       ▼
 GpsStoreForwardModule
       │  appendFix()
       ▼
  /static/gpslog.bin   ← ring-buffer of 28-byte GpsRecord structs
  /static/gpslog.idx   ← 8-byte {writeHead, readHead}
       │
       │  runOnce() — detects paired gateway in NodeDB
       ▼
  sendNextFix() → MeshPacket (portnum 256, subtype FIX_DATA)
       │          to=gatewayNode, want_ack=true, hop_limit=1
       ▼
  Gateway node (ESP32 with WiFi)
       │  receives FIX_DATA packets, uploads to server
       │  mesh router automatically sends ROUTING_APP ACK
       ▼
  handleReceived() sees ROUTING_APP with request_id==pendingPacketId
       │  onFixAcked() — advance readHead, save index
       ▼
  send next fix... until readHead==writeHead (log drained)
```

---

## Custom Protocol

Port number: `meshtastic_PortNum_PRIVATE_APP` (256)

Payload layout (raw bytes, not protobuf):

| Offset | Size | Field       | Description                       |
| ------ | ---- | ----------- | --------------------------------- |
| 0      | 1    | subtype     | 0x01=PAIR_REQUEST, 0x02=PAIR_ACK, |
|        |      |             | 0x03=FIX_DATA, 0x04=FIX_ACK       |
| 1      | 4    | timestamp   | Unix epoch (seconds), big-endian  |
| 5      | 4    | latitude_i  | degrees × 1e7, signed, big-endian |
| 9      | 4    | longitude_i | degrees × 1e7, signed, big-endian |
| 13     | 4    | altitude    | metres, signed, big-endian        |
| 17     | 2    | hdop        | hdop × 100, unsigned, big-endian  |
| 19     | 1    | sats        | satellites in view                |
| 20     | 4    | seq         | record sequence number (readHead) |

Total payload: 24 bytes for FIX_DATA.  
PAIR_REQUEST and PAIR_ACK use only the subtype byte.

---

## Flash Storage

### Files

- `/static/gpslog.bin` — raw records, each 28 bytes
- `/static/gpslog.idx` — `uint32_t writeHead, readHead` (8 bytes)

### GpsRecord struct (28 bytes, stored as-is)

```cpp
struct GpsRecord {
    uint32_t timestamp;    // Unix epoch
    int32_t  latitude_i;   // degrees × 1e7
    int32_t  longitude_i;  // degrees × 1e7
    int32_t  altitude;     // metres
    uint16_t hdop;         // × 100
    uint8_t  sats;
    uint8_t  flags;        // reserved
    uint32_t seq;          // writeHead at time of write
    // 4 bytes padding
    uint8_t  _pad[4];
};
```

### Capacity

- 24 hours × 720 records/hr (every 5s) = **17,280 records**
- 17,280 × 28 bytes = **~470 KB**
- nRF52840: 1 MB internal flash; app uses ~477 KB (58.5% of 815 KB window from 0x27000);
  LittleFS occupies the region from end-of-app to end-of-flash (~300+ KB available with
  current app size, growing toward ~500 KB as app binary stays under 600 KB)
- **Ring buffer**: writeHead wraps at MAX_RECORDS=17280; oldest data overwritten when full

### Write strategy

- Index is written **after** the data record → worst-case on power loss: one duplicate
  record on replay (harmless, server should deduplicate on `seq`)
- Records are NOT written to a SafeFile (too slow for 5s cadence); plain FSCom open/write/close

---

## State Machine

```
UNPAIRED
  │  PAIR_REQUEST received from node X
  ▼
PAIRED  (gatewayNode = X, paired channel stored)
  │  runOnce(): sinceLastSeen(gateway) < 600s AND recordCount() > 0
  ▼
DELIVERING
  │  sendNextFix() — sends FIX_DATA, sets pendingPacketId, pendingSentAt
  │
  │  handleReceived(): ROUTING_APP ACK with request_id==pendingPacketId
  │    → onFixAcked(): advance readHead, saveIndex(), clear pending
  │
  │  runOnce(): pendingPacketId != 0 AND millis()-pendingSentAt > 10000
  │    → timeout: re-send same fix
  │
  │  runOnce(): sinceLastSeen(gateway) >= 600s
  │    → pause; stay PAIRED, resume when gateway reappears
  │
  │  recordCount() == 0
  ▼
PAIRED (idle, waiting for new records)
```

---

## Position Capture

The module uses:

- `isPromiscuous = true` — receives all decoded packets passing through this node
- `loopbackOk = true` — receives locally generated packets (our own position broadcasts)
- `wantPacket()` returns true for `POSITION_APP` **and** `ROUTING_APP` and `PRIVATE_APP`

When a `POSITION_APP` packet arrives from ourselves (`isFromUs(&mp)`), the module calls
`appendFix()` using `localPosition` (always the authoritative local position).

---

## Gateway Requirements

The gateway node must:

1. Send a `PAIR_REQUEST` packet on the shared private channel to the tracker's NodeNum
2. Receive `FIX_DATA` packets and forward to server (via its existing MQTT uplink)

A minimal companion Python script for the gateway is outside the scope of this firmware
change. Any Meshtastic node with MQTT uplink enabled will forward the raw position data
automatically; the gateway only needs to initiate pairing once.

---

## Files Changed

| File                                    | Change                                         |
| --------------------------------------- | ---------------------------------------------- |
| `src/modules/GpsStoreForwardModule.h`   | New file                                       |
| `src/modules/GpsStoreForwardModule.cpp` | New file                                       |
| `src/modules/Modules.cpp`               | Register module under `#ifdef TRACKER_T1000_R` |

---

## Risks & Mitigations

| Risk                           | Mitigation                                                                                                                                              |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Flash wear at 5s write cadence | nRF52840 flash: 10k erase cycles/4KB block. 28-byte records spread across many blocks via LittleFS wear-levelling; expected lifetime >> device lifespan |
| Log full before gateway seen   | Ring-buffer silently overwrites oldest; newest data preserved                                                                                           |
| ACK lost in flight             | 10-second timeout + up to 3 retransmit attempts, then advance (drop oldest)                                                                             |
| Corrupt index on power loss    | Re-scan log on mount failure; conservative: reset readHead=writeHead (log treated as empty)                                                             |
| Large replay floods mesh       | hop_limit=1 (direct only); delivery pauses when gateway not heard in 10 min                                                                             |
