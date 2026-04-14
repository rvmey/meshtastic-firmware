#ifdef TRACKER_T1000_R

#include "GpsStoreForwardModule.h"
#include "GPS.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "buzz/buzz.h"
#include "configuration.h"
#include "main.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "mesh/generated/meshtastic/portnums.pb.h"
#include "meshUtils.h"

GpsStoreForwardModule *gpsStoreForwardModule;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

GpsStoreForwardModule::GpsStoreForwardModule() : MeshModule("GpsStoreForward"), concurrency::OSThread("GpsStoreForward")
{
    isPromiscuous = true; // see all decoded packets passing through this node
    loopbackOk = true;    // also see packets we generated locally (position broadcasts)

    // Ensure /static directory exists
    if (!FSCom.exists("/static")) {
        FSCom.mkdir("/static");
    }

    loadIndex();

    // Start the periodic thread after a short boot delay
    setIntervalFromNow(15 * 1000);
}

// ---------------------------------------------------------------------------
// MeshModule — wantPacket
// ---------------------------------------------------------------------------

bool GpsStoreForwardModule::wantPacket(const meshtastic_MeshPacket *p)
{
    if (p->which_payload_variant != meshtastic_MeshPacket_decoded_tag)
        return false;

    auto portnum = p->decoded.portnum;

    // Pairing and optional FIX_ACK from gateway
    if (portnum == GSF_PORTNUM)
        return true;

    // ROUTING_APP — we need to detect the mesh ACK for our pending FIX_DATA packet
    if (portnum == meshtastic_PortNum_ROUTING_APP && pendingPacketId != 0) {
        // Only intercept if destined to us
        if (isToUs(p))
            return true;
    }

    // TEXT_MESSAGE_APP — accept all incoming text messages (keyword check in handleReceived gates replies)
    if (portnum == meshtastic_PortNum_TEXT_MESSAGE_APP && !isFromUs(p))
        return true;

    return false;
}

// ---------------------------------------------------------------------------
// MeshModule — handleReceived
// ---------------------------------------------------------------------------

ProcessMessage GpsStoreForwardModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag)
        return ProcessMessage::CONTINUE;

    const auto &decoded = mp.decoded;

    // -----------------------------------------------------------------------
    // 1. Private app — pairing or optional gateway FIX_ACK
    // -----------------------------------------------------------------------
    if (decoded.portnum == GSF_PORTNUM) {
        if (decoded.payload.size == 0)
            return ProcessMessage::CONTINUE;

        uint8_t subtype = decoded.payload.bytes[0];

        if (subtype == GSF_SUBTYPE_PAIR_REQUEST) {
            // Accept pairing from any node on any channel
            gatewayNode = mp.from;
            gatewayChannel = mp.channel;
            LOG_INFO("GsfStoreForward: paired with gateway 0x%08x on ch%u", gatewayNode, gatewayChannel);
            sendPairAck(mp.from, mp.channel);
            sendPairingConfirmationText(mp.from, mp.channel);
        } else if (subtype == GSF_SUBTYPE_FIX_ACK) {
            // Gateway can optionally send an explicit FIX_ACK with seq in bytes 1-4
            if (pendingPacketId != 0) {
                onFixAcked();
            }
        } else if (subtype == GSF_SUBTYPE_STATUS_REQUEST) {
            sendStatusText(mp.channel);
        }
        return ProcessMessage::CONTINUE;
    }

    // -----------------------------------------------------------------------
    // 3. Text message commands — "status" or "pair" (case-insensitive)
    // -----------------------------------------------------------------------
    if (decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP) {
        char text[meshtastic_Constants_DATA_PAYLOAD_LEN + 1];
        uint32_t sz = decoded.payload.size < meshtastic_Constants_DATA_PAYLOAD_LEN
                          ? decoded.payload.size
                          : (uint32_t)meshtastic_Constants_DATA_PAYLOAD_LEN;
        memcpy(text, decoded.payload.bytes, sz);
        text[sz] = '\0';
        // Strip trailing whitespace
        for (int i = (int)sz - 1; i >= 0 && (text[i] == ' ' || text[i] == '\r' || text[i] == '\n'); i--)
            text[i] = '\0';
        if (strcasecmp(text, "status") == 0) {
            sendStatusText(mp.channel);
        } else if (strcasecmp(text, "pair") == 0) {
            gatewayNode = mp.from;
            gatewayChannel = mp.channel;
            LOG_INFO("GsfStoreForward: paired via text with gateway 0x%08x on ch%u", gatewayNode, gatewayChannel);
            sendPairAck(mp.from, mp.channel);
            sendPairingConfirmationText(mp.from, mp.channel);
            playBeep();
        }
        return ProcessMessage::CONTINUE;
    }

    // -----------------------------------------------------------------------
    // 4. ROUTING_APP — check for mesh ACK of our pending FIX_DATA packet
    // -----------------------------------------------------------------------
    if (decoded.portnum == meshtastic_PortNum_ROUTING_APP && pendingPacketId != 0) {
        // The mesh router places the original packet ID in request_id
        if (decoded.request_id == pendingPacketId) {
            meshtastic_Routing routing = meshtastic_Routing_init_default;
            if (pb_decode_from_bytes(decoded.payload.bytes, decoded.payload.size, &meshtastic_Routing_msg, &routing)) {
                if (routing.error_reason == meshtastic_Routing_Error_NONE) {
                    onFixAcked();
                } else {
                    // NAK: gateway couldn't deliver — back off and retry
                    LOG_WARN("GsfStoreForward: NAK for fix seq=%u, will retry", readHead);
                    pendingPacketId = 0;
                    pendingSentAt = 0;
                    // runOnce will re-send after GSF_ACK_TIMEOUT_MS
                }
            }
        }
        return ProcessMessage::CONTINUE;
    }

    return ProcessMessage::CONTINUE;
}

// ---------------------------------------------------------------------------
// OSThread — runOnce
// ---------------------------------------------------------------------------

int32_t GpsStoreForwardModule::runOnce()
{
    // -----------------------------------------------------------------------
    // Always: sample localPosition and store a new fix when the GPS timestamp
    // changes (or every 5 s as a fallback when GPS has no RTC sync yet).
    // This runs regardless of pairing / delivery state so no fixes are missed.
    // localPosition is updated by PositionModule whenever the GPS chip reports
    // a new solution, independent of channel broadcasts.
    // -----------------------------------------------------------------------
    if (localPosition.latitude_i != 0 || localPosition.longitude_i != 0) {
        uint32_t t = localPosition.time;
        bool newByTimestamp = (t != 0 && t != lastStoredTimestamp);
        bool newByMillis = (t == 0 && millis() - lastStoredMs >= 5000UL);
        if (newByTimestamp || newByMillis) {
            lastStoredTimestamp = t;
            lastStoredMs = millis();
            appendFix();
            LOG_DEBUG("GsfStoreForward: stored fix, writeHead=%u recordCount=%u", writeHead, recordCount());
        }
    }

    if (gatewayNode == 0)
        return 1000; // Not paired; check again in 1s

    // Check if gateway is still "in range" (heard recently)
    const meshtastic_NodeInfoLite *gwNode = nodeDB->getMeshNode(gatewayNode);
    bool gatewayInRange = (gwNode != nullptr) && (sinceLastSeen(gwNode) < GSF_GATEWAY_TIMEOUT_SECS);

    if (!gatewayInRange) {
        if (delivering && pendingPacketId != 0) {
            // Gateway went away mid-delivery — pause until it returns
            LOG_INFO("GsfStoreForward: gateway out of range, pausing delivery");
            delivering = false;
            pendingPacketId = 0;
            pendingSentAt = 0;
            retryCount = 0;
            pendingBatchSize = 0;
        }
        return 5000;
    }

    // Gateway is in range
    if (recordCount() == 0) {
        // Nothing to deliver
        delivering = false;
        return 1000;
    }

    // Handle ACK timeout / retry
    if (pendingPacketId != 0) {
        uint32_t elapsed = millis() - pendingSentAt;
        if (elapsed < GSF_ACK_TIMEOUT_MS)
            return 500; // Still waiting for ACK

        // Timeout — re-send the same record. We never advance past a record
        // without an ACK; no GPS fix is ever discarded due to retry exhaustion.
        // Delivery is only paused when the gateway goes out of range (see above).
        retryCount++;
        LOG_INFO("GsfStoreForward: retry %u for batch starting at slot %u", retryCount, readHead);
        pendingPacketId = 0;
        pendingSentAt = 0;
        pendingBatchSize = 0; // recalculated in sendNextFix
    }

    if (pendingPacketId == 0) {
        if (!delivering) {
            // Snapshot writeHead so we know when the backlog is fully cleared
            deliveryTargetHead = writeHead;
            LOG_INFO("GsfStoreForward: starting delivery, target=%u", deliveryTargetHead);
        }
        delivering = true;
        sendNextFix();
    }

    return 500; // Poll every 500 ms during active delivery
}

// ---------------------------------------------------------------------------
// Storage — appendFix
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::appendFix()
{
    if (!indexLoaded)
        loadIndex();

    GpsRecord rec;
    memset(&rec, 0, sizeof(rec));

    const meshtastic_Position &pos = localPosition;
    rec.timestamp = pos.time > 0 ? pos.time : (uint32_t)getTime();
    rec.latitude_i = pos.latitude_i;
    rec.longitude_i = pos.longitude_i;
    rec.altitude = pos.altitude;
    rec.hdop = (uint16_t)(pos.HDOP * 100.0f);
    rec.sats = (uint8_t)pos.sats_in_view;
    rec.flags = 0;
    rec.seq = writeHead;

    // Compute byte offset for this slot
    uint32_t offset = writeHead * sizeof(GpsRecord);

    // Open log file in write mode; position to `offset`.
    // Adafruit LittleFS does not support O_RDWR seek-and-write, so we use a
    // full-file read-modify-write only for the index; for the data file we rely
    // on the fact that LittleFS files can be opened for writing at any offset by
    // seeking after open.
    File f = FSCom.open(GSF_LOG_FILE, FILE_O_WRITE);
    if (!f) {
        LOG_WARN("GsfStoreForward: cannot open log file for writing");
        return;
    }
    if (!f.seek(offset)) {
        LOG_WARN("GsfStoreForward: seek failed at offset %u", offset);
        f.close();
        return;
    }
    f.write(reinterpret_cast<const uint8_t *>(&rec), sizeof(GpsRecord));
    f.close();

    // Advance write head (ring buffer)
    uint32_t nextWrite = (writeHead + 1) % GSF_MAX_RECORDS;

    // If we lapped the read head, advance readHead too (discard oldest)
    if (nextWrite == readHead && recordCount() > 0) {
        readHead = (readHead + 1) % GSF_MAX_RECORDS;
        LOG_DEBUG("GsfStoreForward: log full, discarding oldest record");
    }

    writeHead = nextWrite;
    saveIndex();
}

// ---------------------------------------------------------------------------
// Storage — readRecord
// ---------------------------------------------------------------------------

bool GpsStoreForwardModule::readRecord(uint32_t slot, GpsRecord &out)
{
    if (slot >= GSF_MAX_RECORDS)
        return false;

    uint32_t offset = slot * sizeof(GpsRecord);
    File f = FSCom.open(GSF_LOG_FILE, FILE_O_READ);
    if (!f) {
        LOG_WARN("GsfStoreForward: cannot open log file for reading");
        return false;
    }
    if (!f.seek(offset)) {
        f.close();
        return false;
    }
    size_t n = f.read(reinterpret_cast<uint8_t *>(&out), sizeof(GpsRecord));
    f.close();
    return n == sizeof(GpsRecord);
}

// ---------------------------------------------------------------------------
// Storage — loadIndex
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::loadIndex()
{
    GpsLogIndex idx;
    File f = FSCom.open(GSF_IDX_FILE, FILE_O_READ);
    if (f) {
        size_t n = f.read(reinterpret_cast<uint8_t *>(&idx), sizeof(GpsLogIndex));
        f.close();
        if (n == sizeof(GpsLogIndex) && idx.writeHead < GSF_MAX_RECORDS && idx.readHead < GSF_MAX_RECORDS) {
            writeHead = idx.writeHead;
            readHead = idx.readHead;
            indexLoaded = true;
            LOG_INFO("GsfStoreForward: loaded index write=%u read=%u (%u records buffered)", writeHead, readHead, recordCount());
            return;
        }
        LOG_WARN("GsfStoreForward: index corrupt, resetting");
    }
    writeHead = 0;
    readHead = 0;
    indexLoaded = true;
    saveIndex();
}

// ---------------------------------------------------------------------------
// Storage — saveIndex
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::saveIndex()
{
    GpsLogIndex idx = {writeHead, readHead};

    // Overwrite the small index file atomically by removing and rewriting
    FSCom.remove(GSF_IDX_FILE);
    File f = FSCom.open(GSF_IDX_FILE, FILE_O_WRITE);
    if (!f) {
        LOG_WARN("GsfStoreForward: cannot write index file");
        return;
    }
    f.write(reinterpret_cast<const uint8_t *>(&idx), sizeof(GpsLogIndex));
    f.close();
}

// ---------------------------------------------------------------------------
// Storage — recordCount
// ---------------------------------------------------------------------------

uint32_t GpsStoreForwardModule::recordCount() const
{
    if (writeHead >= readHead)
        return writeHead - readHead;
    return GSF_MAX_RECORDS - readHead + writeHead;
}

// ---------------------------------------------------------------------------
// Delivery — sendNextFix
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::sendNextFix()
{
    if (recordCount() == 0 || gatewayNode == 0)
        return;

    // Number of fixes remaining in the current delivery batch
    uint32_t remaining = (deliveryTargetHead - readHead + GSF_MAX_RECORDS) % GSF_MAX_RECORDS;
    if (remaining == 0) {
        // Shouldn't reach here, but guard against infinite loop
        LOG_WARN("GsfStoreForward: sendNextFix called with remaining=0");
        return;
    }
    uint32_t batchSz = remaining < GSF_BATCH_SIZE ? remaining : GSF_BATCH_SIZE;
    pendingBatchSize = 0;

    for (uint32_t i = 0; i < batchSz; i++) {
        uint32_t slot = (readHead + i) % GSF_MAX_RECORDS;
        GpsRecord rec;
        if (!readRecord(slot, rec)) {
            LOG_WARN("GsfStoreForward: failed to read slot %u, stopping batch", slot);
            break;
        }

        // Encode as a POSITION_APP protobuf so the standard gateway MQTT module
        // forwards it to the broker without any custom gateway software.
        meshtastic_Position pos = meshtastic_Position_init_zero;
        pos.has_latitude_i = true;
        pos.has_longitude_i = true;
        pos.has_altitude = true;
        pos.latitude_i = rec.latitude_i;
        pos.longitude_i = rec.longitude_i;
        pos.altitude = rec.altitude;
        pos.time = rec.timestamp;
        pos.timestamp = rec.timestamp;
        pos.HDOP = rec.hdop;
        pos.sats_in_view = rec.sats;
        pos.seq_number = rec.seq;

        meshtastic_MeshPacket *p = router->allocForSending();
        if (!p) {
            LOG_WARN("GsfStoreForward: no packet pool space at batch index %u", i);
            break;
        }
        p->decoded.portnum = meshtastic_PortNum_POSITION_APP;
        p->to = gatewayNode;
        p->channel = gatewayChannel;
        p->hop_limit = 1; // direct only — no mesh flooding
        p->decoded.payload.size =
            pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Position_msg, &pos);

        bool isLast = (i == batchSz - 1);
        if (isLast) {
            // Only the last fix in the batch requests an ACK; its arrival gates
            // readHead advancement for the whole batch.
            p->want_ack = true;
            pendingPacketId = p->id;
            pendingSentAt = millis();
        } else {
            p->want_ack = false;
        }

        service->sendToMesh(p, RX_SRC_LOCAL, true);
        pendingBatchSize++;
        LOG_DEBUG("GsfStoreForward: sent fix slot=%u seq=%u want_ack=%d", slot, rec.seq, (int)isLast);
    }
    LOG_INFO("GsfStoreForward: batch of %u sent, pending id=0x%08x", pendingBatchSize, pendingPacketId);
}

// ---------------------------------------------------------------------------
// Delivery — onFixAcked
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::onFixAcked()
{
    LOG_INFO("GsfStoreForward: ACK for batch of %u starting at slot %u", pendingBatchSize, readHead);
    for (uint8_t i = 0; i < pendingBatchSize; i++)
        readHead = (readHead + 1) % GSF_MAX_RECORDS;
    saveIndex();
    pendingPacketId = 0;
    pendingSentAt = 0;
    retryCount = 0;
    pendingBatchSize = 0;

    // All backlogged fixes have been delivered — notify the gateway and beep
    if (readHead == deliveryTargetHead) {
        LOG_INFO("GsfStoreForward: delivery complete, all fixes sent");
        delivering = false;
        sendDeliveryCompleteText();
        playBeep();
        return; // runOnce() will start a new delivery batch if more fixes exist
    }

    // More fixes remain in this batch — send the next one immediately so we
    // don't wait up to 500 ms for the next runOnce() poll.
    sendNextFix();
}

// ---------------------------------------------------------------------------
// Pairing — sendPairAck
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::sendPairAck(NodeNum dest, uint8_t channel)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return;

    uint8_t ack[1] = {GSF_SUBTYPE_PAIR_ACK};
    p->decoded.portnum = GSF_PORTNUM;
    p->to = dest;
    p->channel = channel;
    p->want_ack = false;
    p->hop_limit = 1;
    p->decoded.payload.size = sizeof(ack);
    memcpy(p->decoded.payload.bytes, ack, sizeof(ack));

    service->sendToMesh(p, RX_SRC_LOCAL, true);
    LOG_INFO("GsfStoreForward: sent PAIR_ACK to 0x%08x", dest);
}

// ---------------------------------------------------------------------------
// Pairing — sendPairingConfirmationText
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::sendPairingConfirmationText(NodeNum dest, uint8_t channel)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return;

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "T1000-R paired. Stored fixes: %u.", (unsigned)recordCount());
    if (len <= 0 || len > (int)sizeof(buf))
        len = sizeof(buf) - 1;

    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->to = dest;
    p->channel = channel;
    p->want_ack = false;
    p->hop_limit = 1;
    p->decoded.payload.size = (uint32_t)len;
    memcpy(p->decoded.payload.bytes, buf, (size_t)len);

    service->sendToMesh(p, RX_SRC_LOCAL, true);
    LOG_INFO("GsfStoreForward: sent pairing confirmation text to 0x%08x", dest);
}

// ---------------------------------------------------------------------------
// Delivery — sendDeliveryCompleteText
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::sendDeliveryCompleteText()
{
    if (gatewayNode == 0)
        return;

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return;

    const char *msg = "T1000-R: all GPS fixes delivered.";
    size_t len = strlen(msg);

    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->to = NODENUM_BROADCAST; // broadcast so it appears in the channel feed
    p->channel = gatewayChannel;
    p->want_ack = false;
    p->hop_limit = 1;
    p->decoded.payload.size = (uint32_t)len;
    memcpy(p->decoded.payload.bytes, msg, len);

    service->sendToMesh(p, RX_SRC_LOCAL, true);
    LOG_INFO("GsfStoreForward: sent delivery complete text on ch%u", gatewayChannel);
}

// ---------------------------------------------------------------------------
// Status — sendStatusText
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::sendStatusText(uint8_t channel)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return;

    // Collect GPS status
    bool gpsHasLock = gps && gps->hasLock();
    uint32_t gpsSats = (gps && gpsHasLock) ? gps->p.sats_in_view : 0;
    uint32_t gpsHdop = (gps && gpsHasLock) ? gps->p.HDOP : 0; // HDOP × 100

    char gpsBuf[24];
    if (!gps || !gps->isConnected()) {
        snprintf(gpsBuf, sizeof(gpsBuf), "no GPS");
    } else if (!gpsHasLock) {
        snprintf(gpsBuf, sizeof(gpsBuf), "searching");
    } else {
        snprintf(gpsBuf, sizeof(gpsBuf), "lock sats=%u hdop=%.1f", (unsigned)gpsSats, gpsHdop * 0.01f);
    }

    char buf[96];
    int len = snprintf(buf, sizeof(buf), "T1000-R: %u fixes. GPS: %s. Paired: %s. Delivering: %s.", (unsigned)recordCount(),
                       gpsBuf, gatewayNode != 0 ? "yes" : "no", delivering ? "yes" : "no");
    if (len <= 0 || len > (int)sizeof(buf))
        len = (int)sizeof(buf) - 1;

    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->to = NODENUM_BROADCAST;
    p->channel = channel;
    p->want_ack = false;
    p->hop_limit = 1;
    p->decoded.payload.size = (uint32_t)len;
    memcpy(p->decoded.payload.bytes, buf, (size_t)len);

    service->sendToMesh(p, RX_SRC_LOCAL, true);
    LOG_INFO("GsfStoreForward: sent status on ch%u: %s", channel, buf);
}

#endif // TRACKER_T1000_R
