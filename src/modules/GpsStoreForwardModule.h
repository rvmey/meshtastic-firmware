#pragma once

#ifdef TRACKER_T1000_R

#include "FSCommon.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "concurrency/OSThread.h"
#include "mesh/MeshModule.h"
#include "mesh/MeshTypes.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

// Custom port number for the GPS store-and-forward protocol.
// meshtastic_PortNum_PRIVATE_APP = 256
#define GSF_PORTNUM meshtastic_PortNum_PRIVATE_APP

// Packet subtypes (first byte of payload)
#define GSF_SUBTYPE_PAIR_REQUEST 0x01   // Gateway → Tracker: "pair with me"
#define GSF_SUBTYPE_PAIR_ACK 0x02       // Tracker → Gateway: "paired"
#define GSF_SUBTYPE_FIX_DATA 0x03       // Tracker → Gateway: one stored GPS fix
#define GSF_SUBTYPE_FIX_ACK 0x04        // Gateway → Tracker: "fix received" (optional)
#define GSF_SUBTYPE_STATUS_REQUEST 0x05 // Any node → Tracker: request a status report

// Ring-buffer capacity: 24 h × 720 fixes/h (one every 5 s) = 17 280 records
#define GSF_MAX_RECORDS 17280U

// A gateway is considered "in range" if heard within this many seconds
#define GSF_GATEWAY_TIMEOUT_SECS 600U

// ACK wait before retrying a FIX_DATA packet (ms)
#define GSF_ACK_TIMEOUT_MS 3000U

// Maximum fixes to send per batch (all but the last are fire-and-forget;
// the last requests an ACK and gates readHead advancement)
#define GSF_BATCH_SIZE 4U

// Path constants
#define GSF_LOG_FILE "/static/gpslog.bin"
#define GSF_IDX_FILE "/static/gpslog.idx"

/**
 * 28-byte record stored in /static/gpslog.bin.
 * Written as raw binary, one struct per GPS fix appended to the ring buffer.
 */
struct __attribute__((packed)) GpsRecord {
    uint32_t timestamp;  // Unix epoch (seconds)
    int32_t latitude_i;  // degrees × 1e7
    int32_t longitude_i; // degrees × 1e7
    int32_t altitude;    // metres
    uint16_t hdop;       // horizontal dilution × 100
    uint8_t sats;        // satellites in view
    uint8_t flags;       // reserved
    uint32_t seq;        // writeHead value at the time of write
    uint8_t _pad[4];     // pad to 28 bytes
};
static_assert(sizeof(GpsRecord) == 28, "GpsRecord must be 28 bytes");

/**
 * 8-byte index persisted in /static/gpslog.idx.
 */
struct __attribute__((packed)) GpsLogIndex {
    uint32_t writeHead; // next slot to write (wraps at GSF_MAX_RECORDS)
    uint32_t readHead;  // oldest undelivered slot
};
static_assert(sizeof(GpsLogIndex) == 8, "GpsLogIndex must be 8 bytes");

/**
 * GpsStoreForwardModule
 *
 * - Sniffs every locally generated POSITION_APP packet and appends it to flash.
 * - Listens for a PAIR_REQUEST on portnum 256 from any node.
 * - Once paired, replays stored fixes to the gateway one at a time, waiting for a
 *   ROUTING_APP ACK (want_ack=true delivery) before advancing the read pointer.
 * - hop_limit=1 ensures delivery is direct (no mesh flooding).
 */
class GpsStoreForwardModule : public MeshModule, private concurrency::OSThread
{
  public:
    GpsStoreForwardModule();

  protected:
    // -------------------------------------------------------------------
    // MeshModule interface
    // -------------------------------------------------------------------

    /** Accept PRIVATE_APP (pairing), ROUTING_APP (ACKs), TEXT_MESSAGE_APP (commands) */
    bool wantPacket(const meshtastic_MeshPacket *p) override;

    /** Handle pairing, ACKs, and text commands */
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

    // -------------------------------------------------------------------
    // OSThread interface
    // -------------------------------------------------------------------

    /** Periodic: detect gateway in range and drive delivery */
    int32_t runOnce() override;

  private:
    // -------------------------------------------------------------------
    // Storage
    // -------------------------------------------------------------------

    /** Append a fix from localPosition to the ring buffer */
    void appendFix();

    /** Read the GpsRecord at slot `slot` into `out`. Returns false on error. */
    bool readRecord(uint32_t slot, GpsRecord &out);

    /** Load writeHead/readHead from /static/gpslog.idx; reset both to 0 on failure */
    void loadIndex();

    /** Persist writeHead/readHead to /static/gpslog.idx */
    void saveIndex();

    /** Number of records currently stored (undelivered + in-flight) */
    uint32_t recordCount() const;

    // -------------------------------------------------------------------
    // Delivery
    // -------------------------------------------------------------------

    /** Send the record at readHead to gatewayNode */
    void sendNextFix();

    /** Called when mesh ACK arrives for pendingPacketId — advance readHead */
    void onFixAcked();

    // -------------------------------------------------------------------
    // Pairing
    // -------------------------------------------------------------------

    /** Send a PAIR_ACK back to `dest` on `channel` */
    void sendPairAck(NodeNum dest, uint8_t channel);

    /** Send a TEXT_MESSAGE_APP to `dest` on `channel` confirming pairing and pending fix count */
    void sendPairingConfirmationText(NodeNum dest, uint8_t channel);

    /** Send a TEXT_MESSAGE_APP to the gateway confirming all stored fixes have been delivered */
    void sendDeliveryCompleteText();

    /** Send a TEXT_MESSAGE_APP status report to `dest` on `channel` */
    void sendStatusText(uint8_t channel);

    // -------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------

    NodeNum gatewayNode = 0;    // 0 = unpaired
    uint8_t gatewayChannel = 0; // channel index we received PAIR_REQUEST on

    bool delivering = false;
    PacketId pendingPacketId = 0;
    uint32_t pendingSentAt = 0;
    uint8_t retryCount = 0;
    uint8_t pendingBatchSize = 0;    // fixes sent in the current in-flight batch
    uint32_t deliveryTargetHead = 0; // writeHead snapshot taken when delivery begins

    uint32_t writeHead = 0;
    uint32_t readHead = 0;

    bool indexLoaded = false;
    uint32_t lastStoredTimestamp = 0; // localPosition.time of last stored GPS fix
    uint32_t lastStoredMs = 0;        // millis() of last stored fix (fallback when GPS time==0)
};

extern GpsStoreForwardModule *gpsStoreForwardModule;

#endif // TRACKER_T1000_R
