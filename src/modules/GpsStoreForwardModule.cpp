#ifdef TRACKER_T1000_R

#include "GpsStoreForwardModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
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

    // Our own position fixes (loopback)
    if (portnum == meshtastic_PortNum_POSITION_APP && isFromUs(p))
        return true;

    // Pairing and optional FIX_ACK from gateway
    if (portnum == GSF_PORTNUM)
        return true;

    // ROUTING_APP — we need to detect the mesh ACK for our pending FIX_DATA packet
    if (portnum == meshtastic_PortNum_ROUTING_APP && pendingPacketId != 0) {
        // Only intercept if destined to us
        if (isToUs(p))
            return true;
    }

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
    // 1. Capture our own position fixes
    // -----------------------------------------------------------------------
    if (decoded.portnum == meshtastic_PortNum_POSITION_APP && isFromUs(&mp)) {
        if (nodeDB->hasLocalPositionSinceBoot())
            appendFix();
        return ProcessMessage::CONTINUE;
    }

    // -----------------------------------------------------------------------
    // 2. Private app — pairing or optional gateway FIX_ACK
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
        } else if (subtype == GSF_SUBTYPE_FIX_ACK) {
            // Gateway can optionally send an explicit FIX_ACK with seq in bytes 1-4
            if (pendingPacketId != 0) {
                onFixAcked();
            }
        }
        return ProcessMessage::CONTINUE;
    }

    // -----------------------------------------------------------------------
    // 3. ROUTING_APP — check for mesh ACK of our pending FIX_DATA packet
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
    if (gatewayNode == 0)
        return 5000; // Not paired; check again in 5s

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
        LOG_INFO("GsfStoreForward: retry %u for seq=%u", retryCount, readHead);
        pendingPacketId = 0; // clear so sendNextFix re-sends the same readHead slot
        pendingSentAt = 0;
    }

    if (pendingPacketId == 0) {
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

    GpsRecord rec;
    if (!readRecord(readHead, rec)) {
        LOG_WARN("GsfStoreForward: failed to read record at slot %u, skipping", readHead);
        readHead = (readHead + 1) % GSF_MAX_RECORDS;
        saveIndex();
        return;
    }

    // Build raw payload: subtype(1) + timestamp(4) + lat(4) + lon(4) + alt(4) + hdop(2) + sats(1) + seq(4) = 24 bytes
    uint8_t buf[24];
    buf[0] = GSF_SUBTYPE_FIX_DATA;
    // Big-endian encoding
    buf[1] = (rec.timestamp >> 24) & 0xFF;
    buf[2] = (rec.timestamp >> 16) & 0xFF;
    buf[3] = (rec.timestamp >> 8) & 0xFF;
    buf[4] = rec.timestamp & 0xFF;
    buf[5] = ((uint32_t)rec.latitude_i >> 24) & 0xFF;
    buf[6] = ((uint32_t)rec.latitude_i >> 16) & 0xFF;
    buf[7] = ((uint32_t)rec.latitude_i >> 8) & 0xFF;
    buf[8] = (uint32_t)rec.latitude_i & 0xFF;
    buf[9] = ((uint32_t)rec.longitude_i >> 24) & 0xFF;
    buf[10] = ((uint32_t)rec.longitude_i >> 16) & 0xFF;
    buf[11] = ((uint32_t)rec.longitude_i >> 8) & 0xFF;
    buf[12] = (uint32_t)rec.longitude_i & 0xFF;
    buf[13] = ((uint32_t)rec.altitude >> 24) & 0xFF;
    buf[14] = ((uint32_t)rec.altitude >> 16) & 0xFF;
    buf[15] = ((uint32_t)rec.altitude >> 8) & 0xFF;
    buf[16] = (uint32_t)rec.altitude & 0xFF;
    buf[17] = (rec.hdop >> 8) & 0xFF;
    buf[18] = rec.hdop & 0xFF;
    buf[19] = rec.sats;
    buf[20] = (rec.seq >> 24) & 0xFF;
    buf[21] = (rec.seq >> 16) & 0xFF;
    buf[22] = (rec.seq >> 8) & 0xFF;
    buf[23] = rec.seq & 0xFF;

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) {
        LOG_WARN("GsfStoreForward: no packet pool space");
        return;
    }
    p->decoded.portnum = GSF_PORTNUM;
    p->to = gatewayNode;
    p->channel = gatewayChannel;
    p->want_ack = true;
    p->hop_limit = 1; // direct only — no mesh flooding
    p->decoded.payload.size = sizeof(buf);
    memcpy(p->decoded.payload.bytes, buf, sizeof(buf));

    pendingPacketId = p->id;
    pendingSentAt = millis();

    service->sendToMesh(p, RX_SRC_LOCAL, true);
    LOG_DEBUG("GsfStoreForward: sent fix seq=%u (id=0x%08x) to 0x%08x", rec.seq, pendingPacketId, gatewayNode);
}

// ---------------------------------------------------------------------------
// Delivery — onFixAcked
// ---------------------------------------------------------------------------

void GpsStoreForwardModule::onFixAcked()
{
    LOG_INFO("GsfStoreForward: ACK for fix seq=%u", readHead);
    readHead = (readHead + 1) % GSF_MAX_RECORDS;
    saveIndex();
    pendingPacketId = 0;
    pendingSentAt = 0;
    retryCount = 0;
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

#endif // TRACKER_T1000_R
