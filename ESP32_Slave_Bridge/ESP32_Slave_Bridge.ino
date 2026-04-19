#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>
#include <stdarg.h>
#include "DiskDensityTypes.h"
#include "EspNowQueueTypes.h"

#ifndef ESP_IDF_VERSION_MAJOR
#define ESP_IDF_VERSION_MAJOR 4
#endif

// ======== SIO hacia XF551 ========
#define SIO_RX      16
#define SIO_TX      17
#define SIO_COMMAND 18

HardwareSerial SerialSIO(2);

// Códigos SIO
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

// ======== Protocolo ESP-NOW ========
#define TYPE_HELLO         0x20
#define TYPE_CMD_FRAME     0x01
#define TYPE_SECTOR_CHUNK  0x10
#define TYPE_ACK           0x11
#define TYPE_NAK           0x12
#define TYPE_TIMING_UPDATE 0x30

// NUEVO CFG
#define TYPE_CFG_UPDATE    0x40
#define TYPE_CFG_ACK       0x41

#define CHUNK_PAYLOAD     244
#define MAX_SECTOR_BYTES  256
#define SECTOR_128        128
#define SECTOR_256        256
#define PERCOM_BLOCK_LEN  12
#define PERCOM_SEC_MAGIC  0xFFFF
#define MAX_PREFETCH_SECTORS 26

#define ESPNOW_CHANNEL 1

static inline uint32_t getLE32(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}
static inline void putLE32(uint8_t* p, uint32_t v) {
  p[0]=(uint8_t)(v); p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}

// ===== DEBUG/ROBUSTEZ (runtime desde WEB) =====
static uint8_t  g_verifyFlags = 0b1110;
static bool g_verifyAllWrites = false;
static bool g_verifyBootSectors = true;
static bool g_verifyVtocDir = true;
static bool g_verifyWriteWithVerify = true;

static void applyVerifyFlags(uint8_t f) {
  g_verifyFlags = f;
  g_verifyAllWrites       = (f & 0x01) != 0;
  g_verifyBootSectors     = (f & 0x02) != 0;
  g_verifyVtocDir         = (f & 0x04) != 0;
  g_verifyWriteWithVerify = (f & 0x08) != 0;
}

static uint32_t g_netDelayUs = 0;

static uint32_t g_xfSioBaud = 19200;
static uint32_t g_pendingXfSioBaud = 0;

static void applyXfSioBaudNow(uint32_t baud) {
  if (baud < 9600) baud = 9600;
  if (baud > 115200) baud = 115200;
  if (baud == g_xfSioBaud) return;

  SerialSIO.end();
  delay(10);
  SerialSIO.begin((uint32_t)baud, SERIAL_8N1, SIO_RX, SIO_TX);
  g_xfSioBaud = baud;
}

// ===== actividad/idle =====
static volatile uint32_t g_lastActivityMs = 0;
static inline void markActivity() { g_lastActivityMs = millis(); }
static inline bool isIdleFor(uint32_t ms) { return (millis() - g_lastActivityMs) > ms; }

static const uint32_t INTERBYTE_GAP_MS = 200;   // Restaurado
static const uint32_t DRAIN_MS         = 2;     // Camino rápido: drenado ultra corto en discos sanos
static const uint8_t  AUTO_READAHEAD_EXTRA = 4; // Base: sector actual + 4 adelantados
static const uint8_t  AUTO_READAHEAD_SEQ_EXTRA = 25; // En secuencia, preferir hasta fin de pista/cache
static const uint32_t SERIAL_DEBUG_BAUD = 460800;
static const bool LOG_READ_OK = false;
static const bool LOG_CACHE_HIT = false;
static const bool LOG_PREFETCH = false;
static const bool LOG_RECOVER  = false;

// IDs
const uint8_t DEV_MIN = 0x31; // D1
const uint8_t DEV_MAX = 0x34; // D4

// El SLAVE ya no usa un ID lógico fijo. El MASTER decide D1..D4 por MAC.
// El SLAVE detecta automáticamente la unidad física real del XF551 (jumpers).
uint8_t g_physicalDev = DEV_MIN;
volatile uint8_t g_lastLogicDev = DEV_MIN;

const bool supports256 = true;
static bool g_currentDD = false;

// AUTO-DETECCIÓN DE DENSIDAD
static bool g_diskDensityKnown = false;
static DiskDensity g_diskDensity = DENS_UNKNOWN;
static uint16_t g_diskBytesPerSector = 128;
static uint16_t g_diskSectorsPerTrack = 18;
static bool g_ddForceActive = false;
static uint32_t g_lastDdForceMs = 0;
static DiskDensity g_last128Density = DENS_SD;

// STATUS lógico temporal para no arrastrar errores crudos de la XF551
static bool g_statusOverrideActive = false;
static uint8_t g_statusOverride[4] = {0x00, 0xFF, 0xFE, 0x00};
static uint32_t g_statusOverrideUntilMs = 0;
static uint32_t g_lastIoActivityMs = 0;
static bool g_forceMediaProbe = true;
static const uint32_t kMediaChangeIdleWindowMs = 5000;

static inline const char* densityName(DiskDensity d) {
  switch (d) {
    case DENS_SD: return "SD";
    case DENS_ED: return "ED";
    case DENS_DD: return "DD";
    default:      return "UNKNOWN";
  }
}

static inline bool densityUses256(DiskDensity d) {
  return (d == DENS_DD);
}

static inline void remember128Density(DiskDensity d) {
  if (d == DENS_SD || d == DENS_ED) g_last128Density = d;
}

static inline DiskDensity hostVisibleDensity() {
  if (g_diskDensityKnown && g_diskDensity != DENS_UNKNOWN) return g_diskDensity;
  return g_last128Density;
}

static inline void clearStatusOverride() {
  g_statusOverrideActive = false;
  g_statusOverrideUntilMs = 0;
}

static inline void markIoActivity() {
  g_lastIoActivityMs = millis();
}

static inline void resetReadCacheAndSequence();
static inline void resetPendingTransferState();
static inline void mediaSoftReset(const char* why);
static inline void maybeResetOnDiskChangeWindow(uint8_t nextCmd);

static inline bool statusOverrideIsActive() {
  if (!g_statusOverrideActive) return false;
  if ((int32_t)(g_statusOverrideUntilMs - millis()) <= 0) {
    clearStatusOverride();
    return false;
  }
  return true;
}

static inline void buildCleanStatusForDensity(DiskDensity d, uint8_t st[4]) {
  st[0] = 0x00;
  if (d == DENS_ED) st[0] = 0x80;
  else if (d == DENS_DD) st[0] = 0x20;
  st[1] = 0xFF;
  st[2] = (d == DENS_ED) ? 0xE0 : 0xFE;
  st[3] = 0x00;
}

static inline void armStatusOverrideForDensity(DiskDensity d, uint32_t durationMs, const char* reason) {
  buildCleanStatusForDensity(d, g_statusOverride);
  g_statusOverrideActive = true;
  g_statusOverrideUntilMs = millis() + durationMs;
  if (reason && *reason) {
    logf("[SLAVE] STATUS lógico limpio armado (%s) dens=%s bytes=%02X %02X %02X %02X por %lums",
         reason, densityName(d),
         g_statusOverride[0], g_statusOverride[1], g_statusOverride[2], g_statusOverride[3],
         (unsigned long)durationMs);
  }
}

static inline uint16_t expectedSectorSizeForMode(DiskDensity d, uint16_t sec) {
  if (densityUses256(d) && !(sec >= 1 && sec <= 3)) return SECTOR_256;
  return SECTOR_128;
}

static inline uint16_t expectedSectorSizeForDensity(bool dd, uint16_t sec) {
  return expectedSectorSizeForMode(dd ? DENS_DD : DENS_SD, sec);
}

static inline uint16_t sectorsPerTrackForDensity(DiskDensity d) {
  if (d == DENS_ED) return 26;
  if (d == DENS_SD || d == DENS_DD) return 18;
  return g_diskSectorsPerTrack ? g_diskSectorsPerTrack : 18;
}

static inline uint8_t sectorsRemainingInTrack(uint16_t sec, DiskDensity d) {
  if (sec <= 3) return 1;
  uint16_t spt = sectorsPerTrackForDensity(d);
  if (spt == 0) return 1;
  uint16_t pos = (uint16_t)((sec - 4) % spt);
  uint16_t rem = (uint16_t)(spt - pos);
  if (rem == 0) rem = 1;
  if (rem > 255) rem = 255;
  return (uint8_t)rem;
}

const uint8_t BCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Forward declarations for Arduino auto-prototype safety
static bool xfReadStatusRaw(uint8_t out[4], uint8_t aux1, uint8_t aux2);
static bool xfStatusIndicatesDD(const uint8_t st[4]);
static const char* xfStatusDensityName(const uint8_t st[4]);
static bool xfForceDDByPercom(uint8_t devLogical);
bool readPercomFromXF(uint8_t devLogical, uint8_t aux1, uint8_t aux2, uint8_t* outBuf);
bool writePercomToXF(uint8_t devLogical, uint8_t aux1, uint8_t aux2, const uint8_t* buf, int len);
static void runXf551DensityDiagnostics(uint8_t devLogical);
static void xfDiagProbeSector(uint8_t devLogical, uint16_t sec, DiskDensity mode);


uint8_t g_lastMaster[6] = {0};
bool    g_haveMasterMac = false;

// PREFETCH CACHE
static uint8_t  cacheBuf[MAX_PREFETCH_SECTORS][MAX_SECTOR_BYTES];
static uint16_t cacheFirstSec = 0;
static uint8_t  cacheCount    = 0;
static uint8_t  cacheDev      = 0;
static DiskDensity cacheDensity = DENS_UNKNOWN;

// SEQUENTIAL READ TRACKING
static uint16_t    lastReadSec     = 0;
static uint8_t     lastReadDev     = 0;
static DiskDensity lastReadDensity = DENS_UNKNOWN;
static uint8_t     seqReadStreak   = 0;

// WRITE buffers
static uint8_t  writeBuf[MAX_SECTOR_BYTES];
static int      writeExpected  = 0;
static int      writeMaxEnd    = 0;
static uint8_t  writeChunkCount = 0;
static uint32_t writeChunkMask  = 0;

static bool     writePending   = false;
static uint8_t  writePendDev   = 0;
static uint8_t  writePendCmd   = 0;
static bool     writePendDD    = false;
static uint16_t writePendSec   = 0;

// PERCOM WRITE buffer
static uint8_t  percomWriteBuf[PERCOM_BLOCK_LEN];
static int      percomLen      = 0;
static bool     percomWritePending = false;
static uint8_t  percomWriteDev     = 0;

static uint8_t g_percomAux1 = 0;
static uint8_t g_percomAux2 = 0;

// Último PERCOM escrito por el master y aún relevante para FORMAT/READ PERCOM
static bool     g_percomPendingValid = false;
static uint8_t  g_percomPendingBlock[PERCOM_BLOCK_LEN];
static uint8_t  g_percomPendingCmd   = 0;
static bool     g_percomPendingNeedsReadback = false;
static uint32_t g_lastPercomWriteMs = 0;

// HELLO control
static uint32_t g_lastNetActivityMs = 0;
static uint32_t g_lastHelloMs       = 0;

// FIX: Flag para suprimir HELLO durante FORMAT
static volatile bool g_formatInProgress = false;

static inline void resetReadCacheAndSequence() {
  cacheCount = 0;
  cacheDensity = DENS_UNKNOWN;
  seqReadStreak = 0;
  lastReadSec = 0;
  lastReadDev = 0;
  lastReadDensity = DENS_UNKNOWN;
}

static inline void resetPendingTransferState() {
  writePending = false;
  writeExpected = 0;
  writeMaxEnd = 0;
  writeChunkCount = 0;
  writeChunkMask = 0;
  writePendDev = 0;
  writePendCmd = 0;
  writePendDD = false;
  writePendSec = 0;

  percomWritePending = false;
  percomLen = 0;
  percomWriteDev = 0;
}

static inline void resetTransientState(const char* why, bool clearOverride) {
  resetReadCacheAndSequence();
  resetPendingTransferState();
  g_formatInProgress = false;
  g_currentDD = false;
  if (clearOverride) clearStatusOverride();
  (void)why;
}

static inline void mediaSoftReset(const char* why) {
  logf("[SLAVE] Media soft reset: %s", (why && *why) ? why : "idle/new media");
  clearStatusOverride();
  resetReadCacheAndSequence();
  resetPendingTransferState();
  g_formatInProgress = false;
  g_currentDD = false;
  g_forceMediaProbe = true;
  g_percomPendingValid = false;
  g_percomPendingNeedsReadback = false;
  g_lastPercomWriteMs = 0;
  g_diskDensityKnown = false;
  g_diskDensity = DENS_UNKNOWN;
  g_diskBytesPerSector = 128;
  g_diskSectorsPerTrack = 18;
}

static inline void maybeResetOnDiskChangeWindow(uint8_t nextCmd) {
  uint32_t now = millis();
  if (g_lastIoActivityMs == 0) {
    g_lastIoActivityMs = now;
    return;
  }
  bool mediaMgmtCmd = (nextCmd == 0x4E || nextCmd == 0x4F || nextCmd == 0x21 || nextCmd == 0x22 || nextCmd == 0xA1);
  if (mediaMgmtCmd && (uint32_t)(now - g_lastIoActivityMs) > kMediaChangeIdleWindowMs) {
    mediaSoftReset("idle/new media window");
  }
}

// ======== DEBUG COUNTERS ========
static uint32_t g_totalReads = 0;
static uint32_t g_totalWrites = 0;
static uint32_t g_checksumFailures = 0;
static uint32_t g_verifyFailures = 0;

// ======== Cola RX ESP-NOW (procesar fuera del callback Wi-Fi) ========
#define RX_QUEUE_LEN 8
#define RX_PKT_MAX   256

static PendingEspNowPacket g_rxQueue[RX_QUEUE_LEN];
static volatile uint8_t g_rxQHead = 0;
static volatile uint8_t g_rxQTail = 0;
static volatile uint32_t g_rxDrops = 0;
static portMUX_TYPE g_rxQMutex = portMUX_INITIALIZER_UNLOCKED;

static bool enqueueRxPacket(const uint8_t* src, const uint8_t* data, int len) {
  if (!data || len <= 0) return false;
  if (len > RX_PKT_MAX) len = RX_PKT_MAX;

  bool ok = false;
  portENTER_CRITICAL(&g_rxQMutex);
  uint8_t next = (uint8_t)((g_rxQHead + 1) % RX_QUEUE_LEN);
  if (next != g_rxQTail) {
    PendingEspNowPacket &pkt = g_rxQueue[g_rxQHead];
    pkt.valid = true;
    pkt.hasSrc = (src != nullptr);
    if (src) memcpy(pkt.src, src, 6);
    pkt.len = len;
    memcpy(pkt.data, data, len);
    g_rxQHead = next;
    ok = true;
  } else {
    g_rxDrops++;
  }
  portEXIT_CRITICAL(&g_rxQMutex);
  return ok;
}

static bool dequeueRxPacket(PendingEspNowPacket &out) {
  bool ok = false;
  portENTER_CRITICAL(&g_rxQMutex);
  if (g_rxQTail != g_rxQHead) {
    out = g_rxQueue[g_rxQTail];
    g_rxQueue[g_rxQTail].valid = false;
    g_rxQTail = (uint8_t)((g_rxQTail + 1) % RX_QUEUE_LEN);
    ok = true;
  }
  portEXIT_CRITICAL(&g_rxQMutex);
  return ok;
}

// ======== Utilidades ========

void logf(const char* fmt, ...) {
  char b[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  Serial.println(b);
}

uint8_t sioChecksum(const uint8_t* d, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++){
    s += d[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;
  }
  return (uint8_t)(s & 0xFF);
}

static bool isBroadcastMac(const uint8_t* mac) {
  if (!mac) return false;
  for (int i = 0; i < 6; i++) if (mac[i] != 0xFF) return false;
  return true;
}

void ensurePeer(const uint8_t* mac) {
  if (!mac) return;
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = ESPNOW_CHANNEL;
  p.encrypt = false;
  p.ifidx = WIFI_IF_STA;

  esp_err_t e = esp_now_add_peer(&p);
  if (e != ESP_OK && e != ESP_ERR_ESPNOW_EXIST) {
    logf("[ESPNOW] esp_now_add_peer fallo err=%d", (int)e);
  }
}

const uint8_t* replyMac() {
  return g_haveMasterMac ? g_lastMaster : BCAST_MAC;
}

static inline void throttleNet() {
  if (g_netDelayUs > 0) {
    uint32_t d = g_netDelayUs;
    if (d > 20000) d = 20000;
    delayMicroseconds(d);
  }
}

bool send_now_to(const uint8_t* mac, const uint8_t* data, int len) {
  if (!mac || !data || len <= 0) return false;
  ensurePeer(mac);

  esp_err_t e = esp_now_send(mac, data, len);
  if (e != ESP_OK) {
    logf("[ESPNOW] esp_now_send FAIL err=%d len=%d type=0x%02X", (int)e, len, data[0]);
    return false;
  }
  markActivity();
  throttleNet();
  return true;
}

void sendHello() {
  if (g_formatInProgress) return;  // FIX: No enviar durante FORMAT
  uint8_t p[4];
  p[0] = TYPE_HELLO;
  p[1] = g_physicalDev;
  p[2] = supports256  ? 1 : 0;
  p[3] = g_physicalDev;
  (void)send_now_to(BCAST_MAC, p, sizeof(p));
}

void sendACK() {
  uint8_t p[2] = {TYPE_ACK, g_lastLogicDev};
  (void)send_now_to(replyMac(), p, sizeof(p));
}
void sendNAK() {
  uint8_t p[2] = {TYPE_NAK, g_lastLogicDev};
  (void)send_now_to(replyMac(), p, sizeof(p));
}

// ======== SIO hacia XF551 ========

static void drainSio(uint32_t ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (SerialSIO.available()) (void)SerialSIO.read();
    delay(1);
  }
}

static void drainSioAdaptive(uint32_t maxMs = 6, uint32_t idleUs = 1500) {
  uint32_t t0 = millis();
  uint32_t last = micros();

  while ((millis() - t0) < maxMs) {
    bool readAny = false;
    while (SerialSIO.available()) {
      (void)SerialSIO.read();
      readAny = true;
      last = micros();
    }
    if (!readAny && (micros() - last) > idleUs) break;
    yield();
  }
}

static uint16_t xfRecoveryDelayMs(uint16_t sec, int attempt, bool hadAck, int got, int expected) {
  (void)sec;
  (void)expected;

  // Tras un fallo con datos parciales/válidos conviene esperar cerca de una vuelta completa
  // del disco antes de reintentar. En XF551 suele rendir mejor que reintentar "demasiado pronto".
  if (!hadAck) {
    return (uint16_t)(120 + ((attempt > 1) ? 40 * (attempt - 1) : 0));
  }

  if (got > 0) {
    return (uint16_t)(220 + ((attempt > 1) ? 40 * (attempt - 1) : 0));
  }

  return (uint16_t)(160 + ((attempt > 1) ? 40 * (attempt - 1) : 0));
}

static void xfRecoveryAfterFailure(uint16_t sec, DiskDensity mode, int attempt, bool hadAck, int got, int expected) {
  uint16_t recoverMs = xfRecoveryDelayMs(sec, attempt, hadAck, got, expected);

  if (LOG_RECOVER) logf("[XF] Recover sec=%u (%s) try=%d ack=%s got=%d/%d wait=%ums",
       sec, densityName(mode), attempt, hadAck ? "Y" : "N", got, expected, recoverMs);

  drainSioAdaptive(12, 2000);
  delay(recoverMs);
}

void pulseCommandAndSendFrame(const uint8_t* frame5) {
  drainSioAdaptive(DRAIN_MS);

  digitalWrite(SIO_COMMAND, LOW);
  delayMicroseconds(1500);

  SerialSIO.write(frame5, 5);
  SerialSIO.flush();

  delayMicroseconds(2500);
  digitalWrite(SIO_COMMAND, HIGH);
}

static constexpr uint8_t XF_TERM_COMPLETE = SIO_COMPLETE;
static constexpr uint8_t XF_TERM_ERROR    = SIO_ERROR;
static constexpr uint8_t XF_TERM_NAK      = SIO_NAK;
static constexpr uint8_t XF_TERM_TIMEOUT  = 0xFF;

bool waitByte(uint8_t want, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (SerialSIO.available()) {
      uint8_t b = SerialSIO.read();
      if (b == want) return true;

      if (b == SIO_NAK) {
        logf("[XF] Recibido NAK (0x4E) mientras esperaba 0x%02X", want);
        if (want != SIO_NAK) return false;
      }

      if (b == SIO_ERROR) {
        logf("[XF] Recibido ERROR (0x45) mientras esperaba 0x%02X", want);
        if (want != SIO_ERROR) return false;
      }
    }
    delay(1);
  }
  logf("[XF] Timeout esperando 0x%02X", want);
  return false;
}

bool xfReadByteTimeout(uint8_t &b, uint32_t timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (SerialSIO.available()) {
      b = SerialSIO.read();
      return true;
    }
    delay(1);
  }
  return false;
}

static uint8_t xfWaitTerminator(uint32_t timeoutMs) {
  unsigned long start = millis();

  while ((millis() - start) < timeoutMs) {
    if (!SerialSIO.available()) {
      delayMicroseconds(80);
      continue;
    }

    uint8_t b = (uint8_t)SerialSIO.read();

    if (b == SIO_ACK)      continue;
    if (b == SIO_COMPLETE) return XF_TERM_COMPLETE;
    if (b == SIO_ERROR)    return XF_TERM_ERROR;
    if (b == SIO_NAK)      return XF_TERM_NAK;
  }

  return XF_TERM_TIMEOUT;
}

bool xfWaitComplete(uint32_t timeoutMs) {
  uint8_t t = xfWaitTerminator(timeoutMs);

  if (t == XF_TERM_COMPLETE) return true;

  if (t == XF_TERM_ERROR || t == XF_TERM_NAK) {
    logf("[XF] ERROR/NAK 0x%02X mientras esperábamos COMPLETE", (uint8_t)t);
    return false;
  }

  logf("[XF] Timeout esperando COMPLETE (0x43)");
  return false;
}

static bool xfReadExact(uint8_t* out, int n, uint32_t timeoutMs, int &got) {
  got = 0;
  unsigned long start = millis();

  while (got < n && (millis() - start) < timeoutMs) {
    while (SerialSIO.available() && got < n) {
      out[got++] = (uint8_t)SerialSIO.read();
    }
    if (got < n) delayMicroseconds(80);
  }

  return got == n;
}

static bool xfInterpretEmbeddedChecksum(uint8_t* buf, int got, int requested, int &payloadLen) {
  payloadLen = 0;
  if (got < 2) return false;
  if (got >= requested) return false;

  uint8_t chk = buf[got - 1];
  uint8_t calc = sioChecksum(buf, got - 1);
  if (chk != calc) return false;

  payloadLen = got - 1;
  return true;
}

static int xfReadUntilGap(uint8_t* out, int maxLen,
                          uint32_t firstByteTimeoutMs = 30,
                          uint32_t interByteGapMs = 3) {
  int got = 0;
  uint8_t b;

  if (!xfReadByteTimeout(b, firstByteTimeoutMs)) return 0;
  out[got++] = b;

  unsigned long last = millis();

  while (got < maxLen) {
    if (SerialSIO.available()) {
      out[got++] = (uint8_t)SerialSIO.read();
      last = millis();
    } else if ((millis() - last) >= interByteGapMs) {
      break;
    } else {
      delayMicroseconds(80);
    }
  }

  return got;
}

bool readFromDrive(uint8_t* out, int &dataLen, unsigned long timeoutMs,
                   bool allowErrorData = false,
                   bool allowShortRead = false) {
  int requested = dataLen;
  dataLen = 0;

  if (requested <= 0 || requested > MAX_SECTOR_BYTES) {
    logf("[XF] readFromDrive: requested invalido %d", requested);
    return false;
  }

  uint8_t term = xfWaitTerminator(timeoutMs);

  if (term == XF_TERM_TIMEOUT) {
    logf("[XF] readFromDrive: timeout esperando C/E");
    return false;
  }

  if (term == XF_TERM_NAK) {
    logf("[XF] readFromDrive: NAK");
    return false;
  }

  if (term == XF_TERM_COMPLETE) {
    int got = 0;

    if (!xfReadExact(out, requested, timeoutMs, got)) {
      int shortLen = 0;
      if (xfInterpretEmbeddedChecksum(out, got, requested, shortLen)) {
        dataLen = shortLen;
        logf("[XF] readFromDrive: COMPLETE con frame corto válido de %d bytes (se pidió %d)", shortLen, requested);
        return allowShortRead;
      }

      dataLen = got;
      logf("[XF] readFromDrive: incompleto got=%d esperado=%d", got, requested);
      return false;
    }

    uint8_t chk;
    if (!xfReadByteTimeout(chk, 1000)) {
      dataLen = got;
      logf("[XF] readFromDrive: sin checksum (%d)", requested);
      return false;
    }

    uint8_t calc = sioChecksum(out, requested);
    if (chk != calc) {
      dataLen = got;
      logf("[XF] CHECKSUM FAIL %d chk=%02X calc=%02X", requested, chk, calc);
      g_checksumFailures++;
      return false;
    }

    dataLen = requested;
    return true;
  }

  int got = xfReadUntilGap(out, requested, 30, 3);
  dataLen = got;

  if (got <= 0) {
    logf("[XF] readFromDrive: ERROR sin datos");
    return false;
  }

  int shortLen = 0;
  if (xfInterpretEmbeddedChecksum(out, got, requested, shortLen)) {
    dataLen = shortLen;
    logf("[XF] readFromDrive: ERROR con frame corto válido de %d bytes (se pidió %d)", shortLen, requested);
    return allowErrorData && allowShortRead;
  }

  uint8_t chk;
  if (!xfReadByteTimeout(chk, 80)) {
    logf("[XF] readFromDrive: ERROR con %d bytes pero sin checksum", got);
    return false;
  }

  uint8_t calc = sioChecksum(out, got);
  if (chk != calc) {
    logf("[XF] CHECKSUM FAIL(ERROR) got=%d chk=%02X calc=%02X", got, chk, calc);
    g_checksumFailures++;
    return false;
  }

  if (got == requested) {
    logf("[XF] readFromDrive: ERROR con %d bytes válidos", got);
    return allowErrorData;
  }

  logf("[XF] readFromDrive: ERROR con %d/%d bytes válidos", got, requested);
  return allowErrorData && allowShortRead;
}

static bool decodePercomBlock(const uint8_t percom[PERCOM_BLOCK_LEN], DiskGeometry& g) {
  g.tracks = percom[0];
  g.sectorsPerTrack = ((uint16_t)percom[2] << 8) | (uint16_t)percom[3];
  uint8_t sideCode = percom[4];
  g.sides = (sideCode & 0x01) ? 2 : 1;
  g.bytesPerSector = ((uint16_t)percom[6] << 8) | (uint16_t)percom[7];

  if (g.bytesPerSector == 0) g.bytesPerSector = SECTOR_128;
  if (g.sectorsPerTrack == 0) g.sectorsPerTrack = 18;

  g.density = DENS_UNKNOWN;
  if (g.bytesPerSector == 256) {
    g.density = DENS_DD;
  } else if (g.bytesPerSector == 128) {
    if (g.sectorsPerTrack == 26) g.density = DENS_ED;
    else g.density = DENS_SD;
  }

  g.valid = (g.tracks != 0 && g.sectorsPerTrack != 0 &&
             (g.bytesPerSector == 128 || g.bytesPerSector == 256));
  return g.valid;
}

// READ SECTOR con logging mejorado y soporte SD/ED/DD
int readSectorFromXFMode(uint8_t devLogical, uint16_t sec, DiskDensity mode, uint8_t* outBuf) {
  (void)devLogical;

  uint8_t frame[5];
  frame[0] = g_physicalDev;
  frame[1] = 0x52;
  frame[2] = (uint8_t)(sec & 0xFF);
  frame[3] = (uint8_t)(sec >> 8);
  frame[4] = sioChecksum(frame, 4);

  const int MAX_TRIES = 3;
  int sz = (int)expectedSectorSizeForMode(mode, sec);

  for (int attempt = 1; attempt <= MAX_TRIES; attempt++) {
    yield();
    pulseCommandAndSendFrame(frame);

    if (!waitByte(SIO_ACK, 5000)) {
      logf("[SLAVE D%u] READ sin ACK sec=%u (%s) (try %d/%d)",
           (unsigned)(g_lastLogicDev - 0x30), sec, densityName(mode), attempt, MAX_TRIES);
      xfRecoveryAfterFailure(sec, mode, attempt, false, 0, sz);
      continue;
    }

    int readLen = sz;
    bool ok = readFromDrive(outBuf, readLen, 15000);

    if (ok && readLen == sz) {
      if (mode == DENS_DD && sz == 256) {
        g_ddForceActive = true;
        g_lastDdForceMs = millis();
        g_diskDensity = DENS_DD;
        g_diskDensityKnown = true;
        g_diskBytesPerSector = 256;
      }
      g_totalReads++;
      if (LOG_READ_OK) {
        logf("[SLAVE D%u] ✅ READ OK sec=%u (%s) len=%d (try %d/%d) primeros: %02X %02X %02X %02X",
             (unsigned)(g_lastLogicDev - 0x30), sec, densityName(mode), readLen, attempt, MAX_TRIES,
             outBuf[0], outBuf[1], outBuf[2], outBuf[3]);
      }
      return readLen;
    }

    if (mode == DENS_DD && sec > 3 && sz == 256 && readLen == 128) {
      g_ddForceActive = false;
      logf("[XF] DD solicitado sec=%u pero llegaron 128 bytes: DD no confirmado por datos", sec);
      resetTransientState("DD no confirmado", false);
      if (attempt < MAX_TRIES) {
        primeXf551DD(devLogical);
      }
    }

    logf("[SLAVE D%u] READ fallo sec=%u (%s) (try %d/%d)",
         (unsigned)(g_lastLogicDev - 0x30), sec, densityName(mode), attempt, MAX_TRIES);
    xfRecoveryAfterFailure(sec, mode, attempt, true, readLen, sz);
  }

  return 0;
}

static void primeXf551DD(uint8_t devLogical) {
  (void)devLogical;
  if (!supports256) return;

  bool ddSelected = xfForceDDByPercom(devLogical);
  if (ddSelected) {
    logf("[SLAVE] DD seleccionado por STATUS/PERCOM; falta confirmarlo con READ=256");
  }

  uint8_t frame[5];
  frame[0] = g_physicalDev;
  frame[1] = 0x52;
  frame[2] = 0x04;
  frame[3] = 0x00;
  frame[4] = sioChecksum(frame, 4);

  logf("[SLAVE] Prime DD con lectura dummy sector 4...");

  for (int attempt = 1; attempt <= 2; attempt++) {
    drainSioAdaptive(DRAIN_MS, 1500);
    pulseCommandAndSendFrame(frame);

    if (!waitByte(SIO_ACK, 3000)) {
      logf("[SLAVE] Prime DD sin ACK (try %d/2)", attempt);
      continue;
    }

    uint8_t dummy[MAX_SECTOR_BYTES];
    int len = SECTOR_256;
    bool ok = readFromDrive(dummy, len, 4000, true, true);

    if (len == 256) {
      g_ddForceActive = true;
      g_lastDdForceMs = millis();
      g_diskDensity = DENS_DD;
      g_diskDensityKnown = true;
      g_diskBytesPerSector = 256;
      logf("[SLAVE] Prime DD OK: llegaron 256 bytes (%s)", ok ? "válidos" : "con ERROR");
      drainSioAdaptive(4, 1200);
      return;
    }

    if (len == 128) {
      g_ddForceActive = false;
      uint8_t st[4];
      if (xfReadStatusRaw(st, 0x00, 0x00)) {
        logf("[SLAVE] Prime DD: READ real sigue en 128 bytes (%s) status=%02X %02X %02X %02X",
             xfStatusDensityName(st), st[0], st[1], st[2], st[3]);
      } else {
        logf("[SLAVE] Prime DD: la XF551 sigue en 128 bytes (ED/SD) (try %d/2)", attempt);
      }
    } else {
      logf("[SLAVE] Prime DD: respuesta no útil len=%d (try %d/2)", len, attempt);
    }

    if (attempt < 2) {
      xfForceDDByPercom(devLogical);
    }

    drainSioAdaptive(6, 1200);
    delay(10);
  }
}

int readSectorFromXF(uint8_t devLogical, uint16_t sec, bool dd, uint8_t* outBuf) {
  return readSectorFromXFMode(devLogical, sec, dd ? DENS_DD : DENS_SD, outBuf);
}

// ✅✅✅ WRITE con verificación runtime ✅✅✅
bool writeSectorToXF(uint8_t devLogical, uint8_t cmd, uint16_t sec, bool dd,
                     const uint8_t* buf, int len) {
  (void)devLogical;

  int expected = (sec >= 1 && sec <= 3) ? SECTOR_128 : ((supports256 && dd) ? SECTOR_256 : SECTOR_128);

  uint8_t frame[5];
  frame[0] = g_physicalDev;
  uint8_t physCmd = (cmd == 0x57) ? 0x50 : cmd;
  frame[1] = physCmd;
  frame[2] = (uint8_t)(sec & 0xFF);
  frame[3] = (uint8_t)(sec >> 8);
  frame[4] = sioChecksum(frame, 4);

  uint8_t tmp[MAX_SECTOR_BYTES];
  memset(tmp, 0, sizeof(tmp));
  int copyLen = len;
  if (copyLen > expected) copyLen = expected;
  if (copyLen > 0) memcpy(tmp, buf, copyLen);
  uint8_t chk = sioChecksum(tmp, expected);

  const int MAX_TRIES = 3;
  for (int attempt = 1; attempt <= MAX_TRIES; attempt++) {
    drainSio(DRAIN_MS);
    pulseCommandAndSendFrame(frame);

    if (!waitByte(SIO_ACK, 5000)) {
      logf("[SLAVE D%u] WRITE sin ACK sec=%u (try %d/%d)",
           (unsigned)(g_lastLogicDev - 0x30), sec, attempt, MAX_TRIES);
      delay(120);
      continue;
    }

    SerialSIO.write(tmp, expected);
    SerialSIO.flush();
    SerialSIO.write(chk);
    SerialSIO.flush();

    if (!xfWaitComplete(25000)) {
      logf("[SLAVE D%u] WRITE sin COMPLETE sec=%u (try %d/%d)",
           (unsigned)(g_lastLogicDev - 0x30), sec, attempt, MAX_TRIES);
      delay(150);
      continue;
    }

    // ✅ DECIDE SI VERIFICAR (runtime)
    bool doVerify = false;
    if (g_verifyAllWrites) {
      doVerify = true;
    } else {
      if (g_verifyWriteWithVerify && cmd == 0x57) doVerify = true;
      if (g_verifyBootSectors && (sec >= 1 && sec <= 3)) doVerify = true;
      if (g_verifyVtocDir && (sec >= 360 && sec <= 368)) doVerify = true;
    }

    if (doVerify) {
      delay(20);

      uint8_t rb[MAX_SECTOR_BYTES];
      DiskDensity verifyMode = dd ? DENS_DD : (g_diskDensityKnown ? g_diskDensity : DENS_SD);
      int r = readSectorFromXFMode(devLogical, sec, verifyMode, rb);

      if (r != expected) {
        logf("[SLAVE D%u] ❌ VERIFY FAIL sec=%u: read=%d esperado=%d (try %d/%d)",
             (unsigned)(g_lastLogicDev - 0x30), sec, r, expected, attempt, MAX_TRIES);
        g_verifyFailures++;

        if (attempt < MAX_TRIES) {
          delay(200);
          continue;
        }
        return false;
      }

      if (memcmp(rb, tmp, expected) != 0) {
        logf("[SLAVE D%u] ❌ VERIFY FAIL sec=%u: datos no coinciden (try %d/%d)",
             (unsigned)(g_lastLogicDev - 0x30), sec, attempt, MAX_TRIES);
        g_verifyFailures++;

        for (int i = 0; i < expected; i++) {
          if (rb[i] != tmp[i]) {
            logf("  Diff @ offset %d: escribí 0x%02X, leí 0x%02X", i, tmp[i], rb[i]);
            if (i >= 10) {
              logf("  (más diferencias omitidas)");
              break;
            }
          }
        }

        if (attempt < MAX_TRIES) {
          delay(200);
          continue;
        }
        return false;
      }

      DiskDensity verifyLogMode = dd ? DENS_DD : hostVisibleDensity();
      logf("[SLAVE D%u] ✅ VERIFY OK sec=%u (%s)",
           (unsigned)(g_lastLogicDev - 0x30), sec, densityName(verifyLogMode));
    }

    g_totalWrites++;
    DiskDensity logMode = dd ? DENS_DD : hostVisibleDensity();
    logf("[SLAVE D%u] ✅ WRITE OK sec=%u (%s) primeros: %02X %02X %02X %02X",
         (unsigned)(g_lastLogicDev - 0x30), sec, densityName(logMode),
         tmp[0], tmp[1], tmp[2], tmp[3]);

    armStatusOverrideForDensity(logMode, 4000, "WRITE OK");
    return true;
  }

  logf("[SLAVE D%u] ❌ WRITE FALLÓ sec=%u después de %d intentos",
       (unsigned)(g_lastLogicDev - 0x30), sec, MAX_TRIES);
  return false;
}

// PERCOM READ/WRITE (sin cambios)
bool readPercomFromXF(uint8_t devLogical, uint8_t aux1, uint8_t aux2, uint8_t* outBuf) {
  (void)devLogical;
  uint8_t frame[5] = { g_physicalDev, 0x4E, aux1, aux2, 0x00 };
  frame[4] = sioChecksum(frame, 4);

  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 4000)) {
    logf("[SLAVE D%u] READ PERCOM sin ACK", (unsigned)(g_lastLogicDev - 0x30));
    return false;
  }

  int len = PERCOM_BLOCK_LEN;
  if (!readFromDrive(outBuf, len, 10000)) {
    logf("[SLAVE D%u] READ PERCOM fallo lectura", (unsigned)(g_lastLogicDev - 0x30));
    return false;
  }
  return true;
}

static inline uint16_t percomBytesPerSector(const uint8_t* p) {
  uint16_t bps = ((uint16_t)p[6] << 8) | (uint16_t)p[7];
  if (bps == 0) bps = 128;
  return bps;
}

static inline uint16_t percomSectorsPerTrack(const uint8_t* p) {
  uint16_t spt = ((uint16_t)p[2] << 8) | (uint16_t)p[3];
  if (spt == 0) spt = 18;
  return spt;
}

static inline uint8_t percomSides(const uint8_t* p) {
  return (uint8_t)((p[4] & 0x01) ? 2 : 1);
}

static inline const char* percomModeName(const uint8_t* p) {
  uint16_t spt = percomSectorsPerTrack(p);
  uint16_t bps = percomBytesPerSector(p);
  uint8_t sides = percomSides(p);

  if (spt == 26 && bps == 128) return "ED";
  if (bps == 256 && sides >= 2) return "DSDD";
  if (bps == 256) return "SSDD";
  return "SD";
}

static inline bool percomUses256(const uint8_t* p) {
  return percomBytesPerSector(p) >= 256;
}

static void buildPercomSSDD(uint8_t out[PERCOM_BLOCK_LEN]) {
  out[0]  = 40;
  out[1]  = 0x00;
  out[2]  = 0x00; out[3]  = 18;
  out[4]  = 0x00;
  out[5]  = 0x04;
  out[6]  = 0x01; out[7]  = 0x00;
  out[8]  = 0x01;
  out[9]  = 0x41;
  out[10] = 0x00;
  out[11] = 0x00;
}

static bool xfReadStatusRaw(uint8_t out[4], uint8_t aux1, uint8_t aux2) {
  uint8_t frame[5] = { g_physicalDev, 0x53, aux1, aux2, 0x00 };
  frame[4] = sioChecksum(frame, 4);

  drainSio(DRAIN_MS);
  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 3000)) return false;

  int len = 4;
  if (!readFromDrive(out, len, 8000, true, true)) return false;
  return len >= 4;
}

static bool xfStatusIndicatesDD(const uint8_t st[4]) {
  return ((st[0] & 0x20) != 0) && ((st[0] & 0x80) == 0);
}

static const char* xfStatusDensityName(const uint8_t st[4]) {
  if (st[0] & 0x80) return "ED";
  if (st[0] & 0x20) return "DD";
  return "SD";
}

static bool xfForceDDByPercom(uint8_t devLogical) {
  if (!supports256) return false;

  uint8_t percom[PERCOM_BLOCK_LEN];
  buildPercomSSDD(percom);

  logf("[XF] WRITE PERCOM -> forzando SSDD (18 spt, 256 bps, 1 side)");
  if (!writePercomToXF(devLogical, 0x00, 0x00, percom, PERCOM_BLOCK_LEN)) {
    logf("[XF] WRITE PERCOM DD fallo");
    return false;
  }

  delay(8);

  uint8_t rb[PERCOM_BLOCK_LEN];
  bool percomOk = readPercomFromXF(devLogical, 0x00, 0x00, rb);
  if (percomOk) {
    logf("[XF] READ PERCOM tras WRITE: mode=%s bps=%u spt=%u sides=%u",
         percomModeName(rb), (unsigned)percomBytesPerSector(rb),
         (unsigned)percomSectorsPerTrack(rb), (unsigned)percomSides(rb));
  } else {
    logf("[XF] READ PERCOM tras WRITE falló");
  }

  uint8_t st[4];
  bool statusOk = xfReadStatusRaw(st, 0x00, 0x00);
  if (statusOk) {
    logf("[XF] STATUS tras WRITE PERCOM: %02X %02X %02X %02X => %s",
         st[0], st[1], st[2], st[3], xfStatusDensityName(st));
  } else {
    logf("[XF] STATUS tras WRITE PERCOM falló");
  }

  // OJO: PERCOM/STATUS solo indican el modo seleccionado/reportado.
  // DD queda confirmado únicamente cuando un READ sec>3 devuelve 256 bytes reales.
  return statusOk && xfStatusIndicatesDD(st);
}

// AUTO-DETECCIÓN: Consulta PERCOM del XF551 para saber densidad del disco
bool autoDetectDiskDensity() {
  logf("[SLAVE] Auto-detectando densidad del disco...");

  uint8_t percom[PERCOM_BLOCK_LEN];
  if (!readPercomFromXF(g_lastLogicDev, 0x00, 0x00, percom)) {
    logf("[SLAVE] ❌ No se pudo leer PERCOM del disco");
    return false;
  }

  DiskGeometry geom = {};
  if (!decodePercomBlock(percom, geom)) {
    logf("[SLAVE] ❌ PERCOM inválido");
    return false;
  }

  g_diskBytesPerSector  = geom.bytesPerSector;
  g_diskSectorsPerTrack = geom.sectorsPerTrack;
  g_diskDensity         = geom.density;
  g_diskDensityKnown    = (geom.density != DENS_UNKNOWN);
  g_ddForceActive       = false;

  if (geom.density == DENS_DD) {
    logf("[SLAVE] ✅ Disco reportado por PERCOM: %s (%u bytes/sector, spt=%u, sides=%u) -- DD aún no confirmado por READ",
         percomModeName(percom), (unsigned)geom.bytesPerSector,
         (unsigned)geom.sectorsPerTrack, (unsigned)geom.sides);
  } else {
    logf("[SLAVE] ✅ Disco detectado: %s (%u bytes/sector, spt=%u, sides=%u)",
         percomModeName(percom), (unsigned)geom.bytesPerSector,
         (unsigned)geom.sectorsPerTrack, (unsigned)geom.sides);
  }

  return g_diskDensityKnown;
}

bool getDiskDD() {
  if (!g_diskDensityKnown) {
    autoDetectDiskDensity();
  }
  return (g_diskDensity == DENS_DD);
}

static void xfDiagProbeSector(uint8_t devLogical, uint16_t sec, DiskDensity mode) {
  (void)devLogical;

  uint8_t stBefore[4] = {0}, stAfter[4] = {0};
  bool haveBefore = xfReadStatusRaw(stBefore, 0x00, 0x00);

  uint8_t frame[5];
  frame[0] = g_physicalDev;
  frame[1] = 0x52;
  frame[2] = (uint8_t)(sec & 0xFF);
  frame[3] = (uint8_t)(sec >> 8);
  frame[4] = sioChecksum(frame, 4);

  uint8_t buf[MAX_SECTOR_BYTES];
  int req = (int)expectedSectorSizeForMode(mode, sec);
  int got = req;
  bool ack = false;
  bool ok = false;

  drainSio(DRAIN_MS);
  pulseCommandAndSendFrame(frame);
  ack = waitByte(SIO_ACK, 4000);
  if (ack) {
    ok = readFromDrive(buf, got, 12000, true, true);
  } else {
    got = 0;
  }

  bool haveAfter = xfReadStatusRaw(stAfter, 0x00, 0x00);

  logf("[DIAG] sec=%u mode=%s req=%d ack=%s ok=%s got=%d | status-before=%s %02X %02X %02X %02X | status-after=%s %02X %02X %02X %02X",
       sec, densityName(mode), req,
       ack ? "SI" : "NO",
       ok ? "SI" : "NO",
       got,
       haveBefore ? xfStatusDensityName(stBefore) : "N/A",
       stBefore[0], stBefore[1], stBefore[2], stBefore[3],
       haveAfter ? xfStatusDensityName(stAfter) : "N/A",
       stAfter[0], stAfter[1], stAfter[2], stAfter[3]);

  if (got > 0) {
    logf("[DIAG] sec=%u mode=%s primeros: %02X %02X %02X %02X",
         sec, densityName(mode), buf[0], buf[1], buf[2], buf[3]);
  }
}

static void runXf551DensityDiagnostics(uint8_t devLogical) {
  if (!supports256) {
    logf("[DIAG] Saltado: esta unidad/compilación no usa sectores de 256 bytes");
    return;
  }

  if (!isIdleFor(500) || writePending || percomWritePending || g_formatInProgress) {
    logf("[DIAG] Busy: espera a que el bridge esté idle para correr diagnóstico");
    return;
  }

  logf("[DIAG] ===== XF551 density diagnostic begin =====");
  logf("[DIAG] Unidad física=0x%02X lógica=0x%02X", g_physicalDev, devLogical);

  resetDiskDetection();
  autoDetectDiskDensity();

  const uint16_t sectors[] = {4, 18, 19, 361, 721};
  const int n = (int)(sizeof(sectors) / sizeof(sectors[0]));

  logf("[DIAG] --- Pasada ED (lecturas de 128) ---");
  for (int i = 0; i < n; i++) {
    xfDiagProbeSector(devLogical, sectors[i], DENS_ED);
    delay(20);
  }

  logf("[DIAG] --- Selección DD por PERCOM ---");
  bool sel = xfForceDDByPercom(devLogical);
  logf("[DIAG] WRITE/STATUS DD seleccionado=%s (sin confirmación aún)", sel ? "SI" : "NO");
  delay(20);

  logf("[DIAG] --- Pasada DD (lecturas de 256 donde corresponda) ---");
  for (int i = 0; i < n; i++) {
    xfDiagProbeSector(devLogical, sectors[i], DENS_DD);
    delay(20);
  }

  logf("[DIAG] ===== XF551 density diagnostic end =====");
}

// Resetear detección (para cuando se cambia el disco)
void resetDiskDetection() {
  g_diskDensityKnown = false;
  g_diskDensity = DENS_UNKNOWN;
  g_diskBytesPerSector = 128;
  g_diskSectorsPerTrack = 18;
  g_ddForceActive = false;
  g_lastDdForceMs = 0;
  logf("[SLAVE] Detección de densidad reseteada");
}

bool writePercomToXF(uint8_t devLogical, uint8_t aux1, uint8_t aux2, const uint8_t* buf, int len) {
  (void)devLogical;

  uint8_t tmp[PERCOM_BLOCK_LEN];
  memset(tmp, 0, sizeof(tmp));
  if (len > PERCOM_BLOCK_LEN) len = PERCOM_BLOCK_LEN;
  memcpy(tmp, buf, len);

  uint8_t frame[5] = { g_physicalDev, 0x4F, aux1, aux2, 0x00 };
  frame[4] = sioChecksum(frame, 4);

  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 4000)) return false;

  SerialSIO.write(tmp, PERCOM_BLOCK_LEN);
  SerialSIO.flush();

  uint8_t chk = sioChecksum(tmp, PERCOM_BLOCK_LEN);
  SerialSIO.write(chk);
  SerialSIO.flush();

  if (!xfWaitComplete(20000)) return false;
  return true;
}

// ======== ENVÍO DE SECTOR AL MASTER ========

void sendSectorChunk(uint16_t sec, const uint8_t* buf, int len) {
  uint8_t pkt[6 + CHUNK_PAYLOAD];
  int cc = (len + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;
  if (cc <= 0) cc = 1;
  const uint8_t* mac = replyMac();

  for (int i = 0; i < cc; i++) {
    int off = i * CHUNK_PAYLOAD;
    int n   = min((int)CHUNK_PAYLOAD, len - off);

    pkt[0] = TYPE_SECTOR_CHUNK;
    pkt[1] = g_lastLogicDev;
    pkt[2] = (uint8_t)(sec & 0xFF);
    pkt[3] = (uint8_t)(sec >> 8);
    pkt[4] = (uint8_t)i;
    pkt[5] = (uint8_t)cc;
    memcpy(pkt + 6, buf + off, n);

    (void)send_now_to(mac, pkt, 6 + n);
    throttleNet();
    yield();
  }
}

// ======== HANDLERS LÓGICOS ========

void handleReadFromMaster(uint8_t devLogic, uint16_t sec, bool dd, uint8_t pfCount) {
  g_lastLogicDev = devLogic;
    resetPendingTransferState();

  DiskDensity startMode = dd ? DENS_DD : DENS_SD;
  if (g_diskDensityKnown && g_diskDensity != DENS_UNKNOWN) {
    startMode = g_diskDensity;
  }

  if (!dd && sec > 3 && (!g_diskDensityKnown || g_diskDensity == DENS_SD)) {
    autoDetectDiskDensity();
    if (g_diskDensityKnown && g_diskDensity != DENS_UNKNOWN) {
      startMode = g_diskDensity;
    }
  }

  uint8_t buf[MAX_SECTOR_BYTES];
  int sz = (int)expectedSectorSizeForMode(startMode, sec);
  bool servedFromCache = false;

  if (cacheCount > 0 &&
      cacheDev == devLogic &&
      cacheDensity == startMode &&
      sec >= cacheFirstSec &&
      sec < (uint16_t)(cacheFirstSec + cacheCount)) {
    uint8_t idx = (uint8_t)(sec - cacheFirstSec);
    memcpy(buf, cacheBuf[idx], sz);
    servedFromCache = true;
    if (LOG_CACHE_HIT) logf("[CACHE] ✅ Hit sec=%u (%s)", sec, densityName(startMode));
  }

  if (!servedFromCache) {
    DiskDensity candidates[3] = { DENS_UNKNOWN, DENS_UNKNOWN, DENS_UNKNOWN };

    if (g_ddForceActive && sec > 3) {
      candidates[0] = DENS_DD;
    }
    else if (sec > 3 && g_diskDensityKnown && g_diskDensity != DENS_UNKNOWN) {
      // Geometría ya conocida: NO cruzar entre SD/ED/DD
      candidates[0] = g_diskDensity;
    }
    else {
      // Solo cuando todavía no sabemos la geometría real
      candidates[0] = startMode;

      if (startMode == DENS_SD) {
        candidates[1] = DENS_ED;
        candidates[2] = DENS_DD;
      } else if (startMode == DENS_ED) {
        candidates[1] = DENS_SD;
        candidates[2] = DENS_DD;
      } else {
        candidates[1] = DENS_ED;
        candidates[2] = DENS_SD;
      }
    }

    int r = 0;
    DiskDensity usedMode = DENS_UNKNOWN;

    for (int i = 0; i < 3; i++) {
      DiskDensity mode = candidates[i];
      if (mode == DENS_UNKNOWN) continue;

      bool dup = false;
      for (int j = 0; j < i; j++) {
        if (candidates[j] == mode) { dup = true; break; }
      }
      if (dup) continue;

      if (mode == DENS_DD && sec > 3) {
        primeXf551DD(devLogic);
      }

      r = readSectorFromXFMode(devLogic, sec, mode, buf);
      if (r > 0) {
        usedMode = mode;
        break;
      }

      if (!g_diskDensityKnown && mode == DENS_SD && sec > 3) {
        autoDetectDiskDensity();
      }
    }

    if (r <= 0) {
      sendNAK();
      cacheCount = 0;
      cacheDensity = DENS_UNKNOWN;
      return;
    }

    sz = r;
    if (usedMode == DENS_UNKNOWN) {
      sendNAK();
      cacheCount = 0;
      cacheDensity = DENS_UNKNOWN;
      return;
    }

    g_diskDensityKnown = true;
    g_diskDensity = usedMode;
    g_diskBytesPerSector = expectedSectorSizeForMode(usedMode, sec);
    if (usedMode == DENS_DD && sz == 256) {
      g_ddForceActive = true;
      g_lastDdForceMs = millis();
    } else if (usedMode != DENS_DD) {
      g_ddForceActive = false;
    }
  }

  sendACK();
  sendSectorChunk(sec, buf, sz);

  bool isSequentialRead = (lastReadDev == devLogic &&
                           lastReadDensity == g_diskDensity &&
                           sec == (uint16_t)(lastReadSec + 1));

  if (isSequentialRead) {
    if (seqReadStreak < 255) seqReadStreak++;
  } else {
    seqReadStreak = 0;
  }

  uint8_t effectivePf = pfCount;
  if (effectivePf == 0 && g_diskDensityKnown) {
    uint8_t trackRemain = sectorsRemainingInTrack(sec, g_diskDensity);
    if (seqReadStreak >= 1) {
      effectivePf = trackRemain;
    } else {
      uint8_t autoExtra = AUTO_READAHEAD_EXTRA;
      effectivePf = (autoExtra > 0) ? (uint8_t)(1 + autoExtra) : 1;
      if (trackRemain > 0 && effectivePf > trackRemain) effectivePf = trackRemain;
    }
  }
  if (effectivePf == 0) {
    cacheCount = 0;
    cacheDensity = DENS_UNKNOWN;
    return;
  }
  if (effectivePf > MAX_PREFETCH_SECTORS) effectivePf = MAX_PREFETCH_SECTORS;

  uint8_t reused = 0;
  if (servedFromCache && cacheCount > 0 &&
      cacheDev == devLogic && cacheDensity == g_diskDensity &&
      sec >= cacheFirstSec && sec < (uint16_t)(cacheFirstSec + cacheCount)) {
    uint8_t idx = (uint8_t)(sec - cacheFirstSec);
    reused = (uint8_t)(cacheCount - idx);
    if (reused > MAX_PREFETCH_SECTORS) reused = MAX_PREFETCH_SECTORS;

    memcpy(cacheBuf[0], buf, sz);
    for (uint8_t k = 1; k < reused; k++) {
      memcpy(cacheBuf[k], cacheBuf[idx + k], sz);
    }

    cacheFirstSec = sec;
    cacheDev      = devLogic;
    cacheDensity  = g_diskDensity;
    cacheCount    = reused;
  } else {
    memcpy(cacheBuf[0], buf, sz);
    cacheFirstSec = sec;
    cacheDev      = devLogic;
    cacheDensity  = g_diskDensity;
    cacheCount    = 1;
  }

  for (uint8_t i = cacheCount; i < effectivePf; i++) {
    uint16_t nextSec = sec + i;
    int r = readSectorFromXFMode(devLogic, nextSec, g_diskDensity, cacheBuf[i]);
    if (r != sz) break;
    cacheCount++;
  }

  if (cacheCount > 1 && LOG_PREFETCH) {
    logf("[CACHE] Prefetch %u sectores desde %u (%s)%s%s%s%s",
         cacheCount, sec, densityName(g_diskDensity),
         (pfCount == 0 ? " [auto]" : ""),
         (reused > 1 ? " [reuse]" : ""),
         (seqReadStreak >= 1 ? " [seq]" : ""),
         (cacheCount >= sectorsRemainingInTrack(sec, g_diskDensity) ? " [track]" : ""));
  }

  lastReadSec = sec;
  lastReadDev = devLogic;
  lastReadDensity = g_diskDensity;
}

void handleStatusFromMaster(uint8_t devLogic, uint8_t aux1, uint8_t aux2, bool dd) {
  g_lastLogicDev = devLogic;
  (void)dd;
  // Acepta cualquier dev lógico: el MASTER decide por MAC
  resetPendingTransferState();

  if (statusOverrideIsActive()) {
    sendACK();
    sendSectorChunk(0, g_statusOverride, 4);
    logf("[SLAVE] STATUS lógico limpio enviado: %02X %02X %02X %02X",
         g_statusOverride[0], g_statusOverride[1], g_statusOverride[2], g_statusOverride[3]);
    return;
  }

  uint8_t frame[5] = { g_physicalDev, 0x53, aux1, aux2, 0 };
  frame[4] = sioChecksum(frame, 4);

  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 3000)) { sendNAK(); return; }

  uint8_t st[4];
  int len = 4;
  if (!readFromDrive(st, len, 8000)) { sendNAK(); return; }

  DiskDensity visible = hostVisibleDensity();
  if (visible == DENS_ED) {
    st[0] = 0x80;
    st[1] = 0xFF;
    st[2] = 0xE0;
    st[3] = 0x00;
    logf("[1050] STATUS normalizado a ED: %02X %02X %02X %02X", st[0], st[1], st[2], st[3]);
  } else if (visible == DENS_DD) {
    st[0] = 0x20;
    st[1] = 0xFF;
    st[2] = 0xFE;
    st[3] = 0x00;
    logf("[XF] STATUS normalizado a DD: %02X %02X %02X %02X", st[0], st[1], st[2], st[3]);
  }

  sendACK();
  sendSectorChunk(0, st, 4);
}

// FIX: formatXF simplificado - ignora eco, confía en COMPLETE
static void formatXF(uint8_t cmd, uint8_t aux1, uint8_t aux2, bool isDD) {
  g_formatInProgress = true;

  uint8_t base = cmd & 0x7F;

  int formatReplyLen = SECTOR_128;

  // Solo el PERCOM pendiente manda realmente el tamaño de cierre.
  if (g_percomPendingValid && percomUses256(g_percomPendingBlock)) {
    formatReplyLen = SECTOR_256;
  }

  // Para 0x21 sin PERCOM pendiente, no usar isDD arrastrado.
  if (base == 0x21 && !g_percomPendingValid) {
    formatReplyLen = SECTOR_128;
  }

  unsigned long timeout = (formatReplyLen >= 256) ? 360000UL : 150000UL;

  logf("[XF] FORMAT iniciando cmd=0x%02X base=0x%02X isDD=%d pending=%d replyLen=%d",
       cmd, base, isDD ? 1 : 0, g_percomPendingValid ? 1 : 0, formatReplyLen);

  drainSio(100);

  uint8_t frame[5];
  frame[0] = g_physicalDev;
  frame[1] = cmd;
  frame[2] = aux1;
  frame[3] = aux2;
  frame[4] = sioChecksum(frame, 4);

  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 5000)) {
    logf("[XF] FORMAT: No ACK de XF551");
    g_formatInProgress = false;
    resetTransientState("FORMAT sin ACK", true);
    sendNAK();
    return;
  }

  logf("[XF] FORMAT: ACK recibido, esperando COMPLETE...");

  unsigned long t0 = millis();
  bool gotComplete = false;

  while (millis() - t0 < timeout) {
    yield();

    while (SerialSIO.available()) {
      uint8_t b = SerialSIO.read();

      if (b == SIO_COMPLETE) {
        gotComplete = true;
        logf("[XF] FORMAT: COMPLETE tras %lus", (millis() - t0) / 1000);
        break;
      } else if (b == SIO_ERROR || b == SIO_NAK) {
        logf("[XF] FORMAT: ERROR/NAK 0x%02X", b);
        g_formatInProgress = false;
        resetTransientState("FORMAT ERROR/NAK", true);
        sendNAK();
        return;
      }
    }

    if (gotComplete) break;
    delay(50);
    yield();
  }

  if (!gotComplete) {
    logf("[XF] FORMAT: Timeout");
    g_formatInProgress = false;
    resetTransientState("FORMAT timeout", true);
    sendNAK();
    return;
  }

  delay(100);
  while (SerialSIO.available()) {
    SerialSIO.read();
  }

  uint8_t buf[MAX_SECTOR_BYTES];
  memset(buf, 0xFF, sizeof(buf));

  g_formatInProgress = false;
  resetTransientState("FORMAT OK", false);
  resetDiskDetection();

  if (g_percomPendingValid) {
    uint16_t bps = percomBytesPerSector(g_percomPendingBlock);
    uint16_t spt = percomSectorsPerTrack(g_percomPendingBlock);
    g_diskBytesPerSector  = bps;
    g_diskSectorsPerTrack = spt;
    g_diskDensity         = (bps >= 256) ? DENS_DD : ((spt == 26) ? DENS_ED : DENS_SD);
    g_diskDensityKnown    = true;
    remember128Density(g_diskDensity);
    g_percomPendingNeedsReadback = true;

    logf("[XF] FORMAT OK - manteniendo PERCOM pendiente: mode=%s",
         percomModeName(g_percomPendingBlock));
  } else {
    g_diskBytesPerSector  = (formatReplyLen >= 256) ? 256 : 128;
    g_diskSectorsPerTrack = (formatReplyLen >= 256) ? 18 : 18;
    g_diskDensity         = (formatReplyLen >= 256) ? DENS_DD : DENS_SD;
    g_diskDensityKnown    = true;
    remember128Density(g_diskDensity);
  }

  sendACK();
  sendSectorChunk(0, buf, formatReplyLen);
  armStatusOverrideForDensity(hostVisibleDensity(), 6000, "FORMAT OK");
  logf("[XF] FORMAT completado len=%d", formatReplyLen);
}

void handleFormatSD(uint8_t devLogic, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  g_lastLogicDev = devLogic;
  // Acepta cualquier dev lógico: el MASTER decide por MAC
  resetPendingTransferState();
  formatXF(cmd, aux1, aux2, false);  // SD
}
void handleFormatDD(uint8_t devLogic, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  g_lastLogicDev = devLogic;
  // Acepta cualquier dev lógico: el MASTER decide por MAC
  resetPendingTransferState();
  formatXF(cmd, aux1, aux2, true);   // DD
}

void handleWriteFromMaster(uint8_t devLogic, uint8_t cmd, uint16_t sec, bool dd) {
  g_lastLogicDev = devLogic;
  // Acepta cualquier dev lógico: el MASTER decide por MAC
// AUTO-DETECCIÓN: Si el Master dice SD pero detectamos DD, usar DD
  if (!dd && g_diskDensityKnown && g_diskDensity == DENS_DD) {
    dd = true;
    logf("[SLAVE] WRITE: Corrigiendo SD->DD (disco detectado como DD)");
  }

  writePending   = true;
  writePendDev   = devLogic;
  writePendCmd   = cmd;
  writePendSec   = sec;
  writePendDD    = dd;

  memset(writeBuf, 0, sizeof(writeBuf));
  writeMaxEnd     = 0;
  writeChunkCount = 0;
  writeChunkMask  = 0;

  writeExpected = (int)expectedSectorSizeForDensity(dd, sec);

  resetReadCacheAndSequence();
}

void handleReadPercomFromMaster(uint8_t devLogic, uint8_t aux1, uint8_t aux2) {
  g_lastLogicDev = devLogic;
  uint8_t buf[PERCOM_BLOCK_LEN];
  memset(buf, 0, sizeof(buf));

  if (g_percomPendingValid) {
    memcpy(buf, g_percomPendingBlock, PERCOM_BLOCK_LEN);

    uint16_t bps = percomBytesPerSector(buf);
    uint16_t spt = percomSectorsPerTrack(buf);
    uint8_t sides = percomSides(buf);

    g_diskBytesPerSector  = bps;
    g_diskSectorsPerTrack = spt;
    g_diskDensity         = (bps >= 256) ? DENS_DD : ((spt == 26) ? DENS_ED : DENS_SD);
    g_diskDensityKnown    = true;

    logf("[SLAVE] READ PERCOM -> devolviendo PENDIENTE: mode=%s bps=%u spt=%u sides=%u",
         percomModeName(buf), (unsigned)bps, (unsigned)spt, (unsigned)sides);

    sendACK();
    sendSectorChunk(PERCOM_SEC_MAGIC, buf, PERCOM_BLOCK_LEN);

    if (g_percomPendingNeedsReadback) {
      g_percomPendingNeedsReadback = false;
      g_percomPendingValid = false;
    }
    return;
  }

  if (!readPercomFromXF(devLogic, aux1, aux2, buf)) { sendNAK(); return; }

  uint16_t bps = percomBytesPerSector(buf);
  uint16_t spt = percomSectorsPerTrack(buf);
  uint8_t sides = percomSides(buf);

  g_diskBytesPerSector  = bps;
  g_diskSectorsPerTrack = spt;
  g_diskDensity         = (bps >= 256) ? DENS_DD : ((spt == 26) ? DENS_ED : DENS_SD);
  g_diskDensityKnown    = true;

  logf("[SLAVE] READ PERCOM → XF551 real: mode=%s bps=%u spt=%u sides=%u",
       percomModeName(buf), (unsigned)bps, (unsigned)spt, (unsigned)sides);

  sendACK();
  sendSectorChunk(PERCOM_SEC_MAGIC, buf, PERCOM_BLOCK_LEN);
}

void handleWritePercomFromMaster(uint8_t devLogic, uint8_t aux1, uint8_t aux2) {
  g_lastLogicDev = devLogic;
  // Acepta cualquier dev lógico: el MASTER decide por MAC
  percomWritePending = true;
  percomWriteDev     = devLogic;
  percomLen          = 0;

  g_percomAux1 = aux1;
  g_percomAux2 = aux2;

  memset(percomWriteBuf, 0, sizeof(percomWriteBuf));
  cacheCount = 0;
  cacheDensity = DENS_UNKNOWN;
  seqReadStreak = 0;
  lastReadSec = 0;
  lastReadDev = 0;
  lastReadDensity = DENS_UNKNOWN;

  logf("[SLAVE] WRITE PERCOM pendiente para D%u", (unsigned)(devLogic - 0x30));
}

static inline uint32_t fullMask32(uint8_t count) {
  if (count == 0) return 0;
  if (count >= 32) return 0xFFFFFFFFUL;
  return (1UL << count) - 1UL;
}

void handleSectorChunkFromMaster(const uint8_t* in, int len) {
  if (len < 6) return;

  uint8_t devLogic = in[1];
  g_lastLogicDev = devLogic;
  uint16_t sec     = (uint16_t)in[2] | ((uint16_t)in[3] << 8);
  uint8_t idx      = in[4];
  uint8_t count    = in[5];
  const uint8_t* payload = &in[6];
  int dlen = len - 6;
  // Acepta cualquier dev lógico: el MASTER decide por MAC
// WRITE SECTOR
  if (writePending && devLogic == writePendDev && sec == writePendSec) {
    if (dlen <= 0 || count == 0) {
      sendNAK();
      writePending = false;
      return;
    }

    if (writeChunkCount == 0) {
      writeChunkCount = count;
      writeChunkMask  = 0;
      writeMaxEnd     = 0;
    } else if (count != writeChunkCount) {
      sendNAK();
      writePending = false;
      return;
    }

    int off = (int)idx * (int)CHUNK_PAYLOAD;
    if (off < 0 || off >= MAX_SECTOR_BYTES) {
      sendNAK();
      writePending = false;
      return;
    }

    int room = MAX_SECTOR_BYTES - off;
    if (dlen > room) dlen = room;
    if (dlen <= 0) {
      sendNAK();
      writePending = false;
      return;
    }

    memcpy(writeBuf + off, payload, dlen);
    int endPos = off + dlen;
    if (endPos > writeMaxEnd) writeMaxEnd = endPos;

    if (idx < 32) writeChunkMask |= (1UL << idx);

    uint32_t need = fullMask32(writeChunkCount);
    if (need != 0 && ((writeChunkMask & need) == need)) {
      if (writeMaxEnd < writeExpected) {
        logf("[SLAVE] ❌ WRITE incompleto: maxEnd=%d esperado=%d", writeMaxEnd, writeExpected);
        sendNAK();
      } else {
        bool ok = writeSectorToXF(writePendDev, writePendCmd, writePendSec, writePendDD, writeBuf, writeExpected);
        if (ok) sendACK(); else sendNAK();
      }

      writePending = false;
    }
    return;
  }

  // WRITE PERCOM
  if (percomWritePending && devLogic == percomWriteDev && sec == PERCOM_SEC_MAGIC) {
    if (dlen <= 0 || count == 0) {
      sendNAK();
      percomWritePending = false;
      return;
    }

    int off = (int)idx * (int)CHUNK_PAYLOAD;
    if (off < 0 || off >= PERCOM_BLOCK_LEN) {
      sendNAK();
      percomWritePending = false;
      return;
    }

    int room = PERCOM_BLOCK_LEN - off;
    if (dlen > room) dlen = room;
    if (dlen <= 0) {
      sendNAK();
      percomWritePending = false;
      return;
    }

    memcpy(percomWriteBuf + off, payload, dlen);
    int endPos = off + dlen;
    if (endPos > percomLen) percomLen = endPos;

    bool last = (idx + 1 >= count);
    if (!last) return;

    bool ok = writePercomToXF(percomWriteDev, g_percomAux1, g_percomAux2, percomWriteBuf, percomLen);

    if (!ok) {
      resetTransientState("WRITE PERCOM FAIL", true);
      sendNAK();
      return;
    }

    memset(g_percomPendingBlock, 0, sizeof(g_percomPendingBlock));
    memcpy(g_percomPendingBlock, percomWriteBuf, min(percomLen, PERCOM_BLOCK_LEN));
    g_percomPendingValid = true;
    g_percomPendingCmd = 0x4F;
    g_lastPercomWriteMs = millis();
    g_percomPendingNeedsReadback = true;

    uint16_t bps = percomBytesPerSector(g_percomPendingBlock);
    uint16_t spt = percomSectorsPerTrack(g_percomPendingBlock);
    uint8_t sides = percomSides(g_percomPendingBlock);

    logf("[SLAVE] WRITE PERCOM aplicado: mode=%s bps=%u spt=%u sides=%u",
         percomModeName(g_percomPendingBlock),
         (unsigned)bps, (unsigned)spt, (unsigned)sides);

    g_diskBytesPerSector  = bps;
    g_diskSectorsPerTrack = spt;
    g_diskDensity         = (bps >= 256) ? DENS_DD : ((spt == 26) ? DENS_ED : DENS_SD);
    g_diskDensityKnown    = true;

    resetTransientState("WRITE PERCOM OK", true);

    sendACK();
    sendSectorChunk(PERCOM_SEC_MAGIC, g_percomPendingBlock, PERCOM_BLOCK_LEN);
    return;
  }
}

// ===== CFG UPDATE handler =====
void handleCfgUpdate(const uint8_t* in, int len) {
  if (len < 11) return;
  uint8_t target = in[1];
  if (target != 0x00 && target != g_physicalDev) return;

  uint32_t newSio = getLE32(&in[2]);
  uint32_t newNet = getLE32(&in[6]);
  uint8_t  vflg   = in[10];

  g_netDelayUs = (newNet > 20000) ? 20000 : newNet;
  applyVerifyFlags(vflg);

  // aplica baud solo cuando esté idle (no cortar operaciones)
  g_pendingXfSioBaud = newSio;

  // ACK
  uint8_t ack[12];
  ack[0] = TYPE_CFG_ACK;
  ack[1] = g_physicalDev;
  ack[2] = 1;              // ok
  ack[3] = g_verifyFlags;
  putLE32(&ack[4], g_pendingXfSioBaud ? g_pendingXfSioBaud : g_xfSioBaud);
  putLE32(&ack[8], g_netDelayUs);

  send_now_to(replyMac(), ack, sizeof(ack));

  logf("[CFG] UPDATE vflg=0x%02X netDelayUs=%lu xfSio(pend)=%lu",
       (unsigned)g_verifyFlags,
       (unsigned long)g_netDelayUs,
       (unsigned long)g_pendingXfSioBaud);
}

static void processIncomingPacket(const uint8_t* src, const uint8_t* in, int len) {
  markActivity();
  if (len <= 0 || !in) return;
  g_lastNetActivityMs = millis();

  if (src && !isBroadcastMac(src)) {
    memcpy(g_lastMaster, src, 6);
    g_haveMasterMac = true;
    ensurePeer(g_lastMaster);
  }

  uint8_t type = in[0];

  if (type == TYPE_CFG_UPDATE) {
    markIoActivity();
    resetTransientState("CFG UPDATE", true);
    handleCfgUpdate(in, len);
    return;
  }

  if (type == TYPE_SECTOR_CHUNK && len >= 6) {
    markIoActivity();
    handleSectorChunkFromMaster(in, len);
    return;
  }

  if (type == TYPE_CMD_FRAME && len >= 6) {
    uint8_t cmd  = in[1];
    maybeResetOnDiskChangeWindow(cmd);
    markIoActivity();
    uint8_t dev  = in[2];
    uint8_t aux1 = in[3];
    uint8_t aux2 = in[4];
    bool    dd   = supports256 && (in[5] != 0);
    uint8_t pf   = (len >= 7) ? in[6] : 1;

    if (dev < DEV_MIN || dev > DEV_MAX) { sendNAK(); return; }
    g_lastLogicDev = dev;

    uint8_t base = cmd & 0x7F;
    uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

    if (base == 0x21 || base == 0x22) {
      g_currentDD = dd;
      logf("[SLAVE] FORMAT cmd=0x%02X dd=%d", cmd, dd);
    } else if (base == 0x52 || base == 0x50 || base == 0x57) {
      g_currentDD = dd;
    }

    if (base == 0x52) { handleReadFromMaster(dev, sec, dd, pf); return; }
    if (base == 0x53) { handleStatusFromMaster(dev, aux1, aux2, dd); return; }
    if (base == 0x21 || base == 0x22) { formatXF(cmd, aux1, aux2, dd); return; }
    if (base == 0x50 || base == 0x57) { handleWriteFromMaster(dev, cmd, sec, dd); return; }
    if (base == 0x4E) { handleReadPercomFromMaster(dev, aux1, aux2); return; }
    if (base == 0x4F) { handleWritePercomFromMaster(dev, aux1, aux2); return; }

    sendNAK();
  }
}

// ================== CALLBACK ESP-NOW ==================
#if ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t* in, int len) {
  const uint8_t *src = info ? info->src_addr : nullptr;
  (void)enqueueRxPacket(src, in, len);
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t s) {
  (void)info;
  (void)s;
}
#else
void onDataRecv(const uint8_t* src, const uint8_t* in, int len) {
  (void)enqueueRxPacket(src, in, len);
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t s) {
  (void)mac;
  (void)s;
}
#endif

// ✅ Comando para mostrar estadísticas
void printStats() {
  logf("\n=== ESTADÍSTICAS XF551 SLAVE ===");
  logf("Total READs:      %lu", (unsigned long)g_totalReads);
  logf("Total WRITEs:     %lu", (unsigned long)g_totalWrites);
  logf("Checksum FAILs:   %lu", (unsigned long)g_checksumFailures);
  logf("Verify FAILs:     %lu", (unsigned long)g_verifyFailures);
  logf("RX queue drops:    %lu", (unsigned long)g_rxDrops);
  logf("XF551 SIO baud:   %lu", (unsigned long)g_xfSioBaud);
  logf("Net delay (us):   %lu", (unsigned long)g_netDelayUs);
  logf("Verify flags:     0x%02X", (unsigned)g_verifyFlags);
  logf("================================\n");
}



// ===== Auto-detect unidad física XF551 (D1..D4) =====
// Nota: el ESP32 usa g_physicalDev para hablar con la XF551, pero responde al MASTER usando
// el dev lógico (g_lastLogicDev) que venga en los frames del MASTER.
uint8_t detectPhysicalDev() {
  // Intenta STATUS (0x53) en D1..D4 y elige el primero que responda.
  uint8_t st[4];

  for (uint8_t dev = DEV_MIN; dev <= DEV_MAX; dev++) {
    uint8_t frame[5];
    frame[0] = dev;
    frame[1] = 0x53; // STATUS
    frame[2] = 0x00;
    frame[3] = 0x00;
    frame[4] = sioChecksum(frame, 4);

    drainSio(DRAIN_MS);
    pulseCommandAndSendFrame(frame);

    // esperar ACK (0x41)
    if (!waitByte(SIO_ACK, 300)) continue;

    int len = 4;
    if (!readFromDrive(st, len, 800)) continue;

    return dev;
  }

  return DEV_MIN; // fallback
}

// ======== SETUP / LOOP ========
void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.printf("\n[SLAVE D%u] XF551 WiFi Bridge v2.1 - WEB CFG + ERROR 163 FIXES\n",
                (unsigned)(g_lastLogicDev - 0x30));
  Serial.printf("[SLAVE] Física D%u | DD=%s\n",
                (unsigned)(g_physicalDev - 0x30),
                supports256 ? "SÍ" : "NO");

  g_xfSioBaud = 19200;
  SerialSIO.begin((uint32_t)g_xfSioBaud, SERIAL_8N1, SIO_RX, SIO_TX);

  // Detectar automáticamente la unidad física (jumpers XF551)
  g_physicalDev = detectPhysicalDev();
  g_lastLogicDev = g_physicalDev;
  Serial.printf("[SLAVE] Unidad física detectada: 0x%02X (D%d)\n", g_physicalDev, (int)(g_physicalDev - 0x30));


  applyVerifyFlags(g_verifyFlags);

  pinMode(SIO_COMMAND, OUTPUT);
  digitalWrite(SIO_COMMAND, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] esp_now_init falló");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);

  sendHello();
  g_lastHelloMs = millis();
  g_lastNetActivityMs = millis();

  cacheCount = 0;
  cacheDensity = DENS_UNKNOWN;
  seqReadStreak = 0;
  lastReadSec = 0;
  lastReadDev = 0;
  lastReadDensity = DENS_UNKNOWN;
    resetPendingTransferState();

  markActivity();
  logf("[SLAVE] ✅ Listo. Esperando comandos... (Serial: X=diag densidad, S=stats)");
}

void loop() {
  static unsigned long lastHello = 0;
  static unsigned long lastStats = 0;

  // aplicar baud pendiente cuando esté idle (no interrumpir transacción)
  if (g_pendingXfSioBaud != 0 && !g_formatInProgress) {
    if (isIdleFor(500) && !writePending && !percomWritePending) {
      applyXfSioBaudNow(g_pendingXfSioBaud);
      logf("[CFG] ✅ XF551 SIO baud=%lu", (unsigned long)g_xfSioBaud);
      g_pendingXfSioBaud = 0;
    }
  }

  // HELLO periódico si está idle (y no hay FORMAT en progreso)
  if (millis() - lastHello > 10000) {
    if (isIdleFor(1200)) {
      sendHello();
    }
    lastHello = millis();
  }

  // Estadísticas cada 60s
  if (millis() - lastStats > 60000) {
    //printStats();
    lastStats = millis();
  }

  if (g_percomPendingValid && !g_percomPendingNeedsReadback) {
    if (millis() - g_lastPercomWriteMs > 8000) {
      g_percomPendingValid = false;
    }
  }

  // Procesar paquetes recibidos fuera del callback Wi-Fi
  for (int i = 0; i < 4; i++) {
    PendingEspNowPacket pkt;
    if (!dequeueRxPacket(pkt)) break;
    if (!pkt.valid) continue;
    processIncomingPacket(pkt.hasSrc ? pkt.src : nullptr, pkt.data, pkt.len);
  }

  delay(1);
}
