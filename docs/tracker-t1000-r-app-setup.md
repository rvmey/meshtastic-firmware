# Seeed T1000-R — Meshtastic App Configuration Guide

This guide covers every setting you need to configure in the Meshtastic app (iOS, Android, or Web) after flashing the custom T1000-R firmware. Default values baked into the firmware are noted where relevant so you know what is already handled.

---

## 1. Prerequisites

- The T1000-R firmware has been flashed via the `.uf2` or nRfUtil method.
- The Meshtastic app is connected to the device over Bluetooth.
- You have a second Meshtastic node (the **gateway node**) that has Wi-Fi and MQTT configured and is within LoRa range when you want data delivered.

---

## 2. Device Role

**Settings → Device → Role**

| Setting | Value     |
| ------- | --------- |
| Role    | `TRACKER` |

The `TRACKER` role reduces unnecessary re-broadcasting and tunes the device for one-way position reporting. Using any other role wastes power and mesh bandwidth.

---

## 3. Private Channel (Required for GPS Transmission)

The firmware sets **channel 0 (the default public channel) position precision to 0**, which permanently suppresses GPS broadcasts on that channel. GPS coordinates are only transmitted on channels that have a positive precision setting. You must create at least one private channel.

**Settings → Channels → Add / Edit Channel 1** (or any slot other than 0):

| Setting                 | Value                                                         |
| ----------------------- | ------------------------------------------------------------- |
| Channel name            | Any name, e.g. `tracker`                                      |
| PSK                     | Generate a new random key (or paste a shared key)             |
| Position precision      | `32` (highest; sends exact coordinates) or your desired level |
| Allow position requests | Off (the device pushes fixes proactively)                     |
| Uplink enabled          | **On** — required for MQTT relay via the gateway node         |
| Downlink enabled        | Off (tracker does not need to receive mesh traffic)           |

> **Important:** The gateway node must have this same channel configured with identical name and PSK, and must have **MQTT uplink enabled** on the channel as well.

---

## 4. GPS / Position Settings

These values are already set in firmware and do not normally need changing. They are listed here for reference.

| Setting                     | Firmware default | Description                                   |
| --------------------------- | ---------------- | --------------------------------------------- |
| GPS update interval         | **5 seconds**    | How often t he GPS chip produces a new fix    |
| Position broadcast interval | **5 seconds**    | How often a position packet is sent over LoRa |
| Smart position (adaptive)   | **Disabled**     | Fixed interval, not distance-triggered        |

If you need to change the broadcast interval (e.g. to reduce mesh traffic on a shared network), go to **Settings → Position → Position Broadcast Interval** and set a larger value. Do not set it below 5 seconds.

---

## 5. MQTT Module

MQTT is enabled by default in the firmware. The MQTT module on the T1000-R itself does nothing because the nRF52840 chip has no Wi-Fi. Actual MQTT delivery is handled by the **gateway node** (an ESP32-based device) that receives LoRa packets and forwards them to the broker.

Configuration needed **on the gateway node** (not the T1000-R):

**Settings → MQTT:**

| Setting             | Value                                   |
| ------------------- | --------------------------------------- |
| MQTT enabled        | On                                      |
| Server address      | Your broker, e.g. `mqtt.meshtastic.org` |
| Username / Password | As required by your broker              |
| Encryption enabled  | On (recommended)                        |
| Root topic          | `msh` (default) or your custom prefix   |

**On the gateway node's channel matching the tracker's private channel:**

| Setting          | Value    |
| ---------------- | -------- |
| Uplink enabled   | On       |
| Downlink enabled | Optional |

---

## 6. GPS Store-and-Forward (Custom Module)

The firmware contains a custom `GpsStoreForwardModule` that writes every GPS fix to the device's internal flash (LittleFS) and replays stored fixes to a designated gateway node when it comes within LoRa range. No configuration is needed in the standard Meshtastic app — the module operates automatically. The following explains how the pairing and delivery flow works so you can verify correct operation.

### 6.1 How Pairing Works

**Simple method (recommended):** In the Meshtastic app on the gateway node, open the tracker channel and send the message `pair` (case-insensitive). The T1000-R will respond with a confirmation text message, e.g. `T1000-R paired. Stored fixes: 1234.`, and emit a short beep.

**Programmatic method:** Send a 1-byte packet on **port 256** with payload byte `0x01` (`PAIR_REQUEST`). The T1000-R responds with a `PAIR_ACK` (subtype `0x02`) on the same port, plus the confirmation text message above.

In both cases:

- The T1000-R remembers the sending node as the gateway and begins replaying stored fixes.
- Until pairing occurs, fixes are stored in flash but not transmitted.

> **Tip:** If no confirmation text appears after sending `pair`, the message did not reach the T1000-R. Check that both devices share the same channel PSK and are within LoRa range.

### 6.2 Storage Capacity

| Parameter      | Value                          |
| -------------- | ------------------------------ |
| Fix interval   | 5 seconds                      |
| Fixes stored   | 17 280 (ring buffer)           |
| Storage window | ~24 hours                      |
| Flash file     | `/static/gpslog.bin` (~470 KB) |
| Index file     | `/static/gpslog.idx` (8 bytes) |

When the buffer is full, the oldest fixes are overwritten. If the device is offline for more than 24 hours, fixes older than 24 hours are lost.

### 6.3 Delivery Behaviour

- Fixes are sent one at a time with `want_ack = true` and `hop_limit = 1` (direct link only).
- The module waits up to **10 seconds** for a mesh-layer ACK before retrying.
- Retries continue indefinitely — **no fix is ever discarded** due to failed delivery. If the gateway goes out of range mid-delivery, the module pauses and resumes from the same position when the gateway is heard again.
- A gateway is considered "in range" if a packet from it has been heard within the last **10 minutes** (600 seconds).
- When the last stored fix is delivered the T1000-R sends `T1000-R: all GPS fixes delivered.` to the gateway and emits a short beep.

### 6.4 Querying the Fix Count

Any node that shares a channel with the T1000-R can request a status report at any time — no pairing required.

**Simple method (recommended):** In the Meshtastic app, open the tracker channel and send the message `status` (case-insensitive).

**Programmatic method:** Send a 1-byte packet on **port 256** with payload byte `0x05` (subtype `STATUS_REQUEST`).

**Response:** The T1000-R replies with a text message on the same channel, e.g.:

```
T1000-R: 720 fixes stored. Paired: yes. Delivering: no.
```

| Field          | Meaning                                             |
| -------------- | --------------------------------------------------- |
| `fixes stored` | Number of GPS records in flash not yet delivered    |
| `Paired`       | Whether the device has an active gateway pairing    |
| `Delivering`   | Whether a delivery session is currently in progress |

---

## 7. Power Settings

**Settings → Power:**

| Setting             | Recommended value | Reason                                                                                                |
| ------------------- | ----------------- | ----------------------------------------------------------------------------------------------------- |
| Power saving mode   | On                | The device has no screen and no interactive role; this enables aggressive sleep between transmissions |
| Battery INA enabled | Off               | No external INA sensor on this hardware                                                               |

---

## 8. Bluetooth

The device pairs over BLE for initial configuration. Once configured there is no requirement to keep a phone connected.

**Settings → Bluetooth:**

| Setting           | Value                                     |
| ----------------- | ----------------------------------------- |
| Bluetooth enabled | On (required for app access)              |
| Fixed PIN         | Set a PIN to prevent unauthorised pairing |

---

## 9. Verification Checklist

After completing the above steps, confirm the following:

- [ ] Device role is `TRACKER`
- [ ] Channel 0 has no position precision (already enforced by firmware — verify it shows `0` in the app)
- [ ] A private channel (slot 1 or higher) exists with a shared PSK
- [ ] The private channel has **Uplink enabled**
- [ ] The same private channel is configured identically on the gateway node
- [ ] The gateway node has MQTT enabled and connected to a broker
- [ ] The gateway node has received and sent a `PAIR_REQUEST` to the T1000-R (if using store-and-forward delivery)
- [ ] GPS fixes appear in the MQTT broker's topic stream within ~30 seconds of the devices being in range

---

## 10. Troubleshooting

| Symptom                                                         | Likely cause                                            | Fix                                                                           |
| --------------------------------------------------------------- | ------------------------------------------------------- | ----------------------------------------------------------------------------- |
| No position packets visible anywhere                            | Channel 0 precision is 0; no private channel configured | Add private channel with precision > 0                                        |
| Position visible on mesh but not in MQTT                        | Uplink not enabled on the private channel               | Enable uplink on the private channel on the gateway node                      |
| MQTT receives packets but GPS store-and-forward does not replay | Gateway has not sent a `PAIR_REQUEST`                   | Ensure gateway firmware sends the pairing packet on port 256                  |
| No status reply after sending `STATUS_REQUEST`                  | Packet not received by T1000-R                          | Confirm both nodes share the same channel PSK and are within LoRa range       |
| No GPS fix acquired                                             | Cold start can take several minutes outdoors            | Leave the device in open sky; the GPS LED (if present) indicates fix acquired |
| Battery drains faster than expected                             | Power saving mode not enabled                           | Enable power saving in Settings → Power                                       |
