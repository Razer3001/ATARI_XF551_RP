// ================================================================
// VERSION: F24_CODE_CLEAN_STEP1_2026-05-21_1325
// ARCHIVO: RP2040_Master_SIO.ino
// NOTA: version/build visible al abrir el .ino. Limpieza binaria paso 1: ring CAS antiguo no llamado eliminado.
// ================================================================

// ================================================================
// VERSION: F24_CODE_CLEAN_STEP1_2026-05-21_1325
// ARCHIVO: RP2040_Master_SIO.ino
// ROL: RP2040
// NOTA: Versión definida dentro del .ino y visible por Serial.
// ================================================================

// ⚠️ ESTE ARCHIVO REEMPLAZA COMPLETAMENTE TU rp2040_sio_bridge.ino
// Versión DUAL-CORE:
// - Core0: SIO Atari (CMD/frames) + ejecutar operaciones
// - Core1: RX UART desde MASTER (ESP32) + onMasterFrame()
// Mantiene funcionalidades existentes + CFG_UPDATE + validación READ
// V4: mantiene resync V3 y agrega diagnóstico de DATA/ACK de disco hacia el MASTER.
// Fix importante: FORMAT vuelve a TX por SerialSIO (evita que el Atari “quede esperando” en boards donde Serial1 != uart0)

#include <Arduino.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <pico/critical_section.h>

static const char RP_BUILD[] = "F24_CODE_CLEAN_STEP1_2026-05-21_1325";

// ================== PINES ==================
const int PIN_CMD    = 2;
const int PIN_SIO_TX = 0;
const int PIN_SIO_RX = 1;

const int PIN_ESP_TX = 4;
const int PIN_ESP_RX = 5;

const int LED_STATUS = LED_BUILTIN;

#define SerialSIO Serial1

#define UART_ESP uart1
#define UART_ESP_BAUD 460800  // BOOT default (Master también arranca a este)

// ================== Protocolo UART RP <-> MASTER ==================
#define TYPE_CMD_FRAME     0x01
#define TYPE_SECTOR_CHUNK  0x10
#define TYPE_ACK           0x11
#define TYPE_NAK           0x12
#define TYPE_HELLO         0x20
#define TYPE_TIMING_UPDATE 0x30

// NUEVO CFG
#define TYPE_CFG_UPDATE    0x40
#define TYPE_CFG_ACK       0x41
#define TYPE_SUPERDOS_HINT 0x42

// Printer 820 / Atari P:
#define TYPE_PRINTER_CFG_UPDATE 0x70
#define TYPE_PRINTER_LINE       0x71
#define TYPE_PRINTER_CFG_ACK    0x72
#define TYPE_PRINTER_DIAG       0x73
#define TYPE_DISK_DIAG          0x74  // RP2040 -> MASTER disk READ/DATA diagnostic

// Cassette manual C: (F49)
#define TYPE_CAS_CONTROL       0x80
#define TYPE_CAS_DATA          0x81
#define TYPE_CAS_STATUS        0x82
#define TYPE_CAS_ACK           0x83

#define SIO_PRINTER_DEFAULT_DEV 0x40
#define PRN_CMD_STATUS          0x53
#define PRN_CMD_WRITE           0x57
#define PRN_AUX_NORMAL          0x4E
#define PRN_AUX_SIDEWAYS        0x53

// v20: tiempos propios de P: / Atari 820.
// No usamos T_ACK_TO_COMPLETE global porque en esta versión puede estar muy agresivo
// para discos/cache. Para la impresora preferimos margen estable.
#define PRN_CMD_ACK_TO_DATA_US       1200
#define PRN_DATA_ACK_TO_COMPLETE_US  2200
#define PRN_STATUS_ACK_TO_COMPLETE_US 1200

// v25:
// Después de que Atari envía DATA + checksum, el RP2040 no debe responder ACK
// demasiado rápido. Dejamos margen para que el Atari cambie de TX a RX.
// Esto solo aplica a P:, no toca discos.
#define PRN_CHK_TO_DATA_ACK_US       900
#define PRN_STATUS_COMPLETE_TO_DATA_US 1000
#define PRN_STATUS_DATA_TO_CHK_US      80

// v26:
// En impresora P: si falla checksum, NO imprimimos esa línea.
// Pero tampoco detenemos BASIC: respondemos ACK + COMPLETE para que RUN continúe.
// Los discos NO usan esta tolerancia.
#define PRN_TOLERANT_CHECKSUM 1
#define PRN_DROP_BAD_CHECKSUM_LINE 1

// Códigos SIO
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

#define SIO_DEV_MIN 0x31
#define SIO_DEV_MAX 0x37

#define PERCOM_BLOCK_LEN 12
#define PERCOM_SEC_MAGIC 0xFFFF

#define BRIDGE_PERCOM_TRANSPARENTE  0
#define BRIDGE_FIXED_SD_GEOMETRY    0
#define BRIDGE_READ_MISMATCH_ZERO_FILL 0  // 0 = no corromper loaders: si espera 256 y llegan 128, reintenta DD y si falla devuelve ERROR

static const uint16_t CHUNK_PAYLOAD = 240;

// Timings SIO - OPTIMIZADOS
uint16_t T_ACK_TO_COMPLETE   = 300;
uint16_t T_COMPLETE_TO_DATA  = 250;
uint16_t T_DATA_TO_CHK       = 50;
uint16_t T_CHUNK_DELAY       = 100;

bool lastCmdState = HIGH;
uint8_t cmdBuf[5];

// ✅ Contadores de debug
static uint32_t g_totalReads = 0;
static uint32_t g_totalWrites = 0;
static uint32_t g_readErrors = 0;
static uint32_t g_writeErrors = 0;
static uint16_t g_lastErrorSector = 0;

// ====== NUEVO: contadores UART (útiles para debug) ======
static volatile uint32_t g_uart_bad_chk = 0;
static volatile uint32_t g_uart_bad_len = 0;

// ================== FSM UART desde MASTER (core1) ==================
static uint8_t  uartState = 0;
static uint8_t  uartLen   = 0;
static uint8_t  uartIdx   = 0;
static uint8_t  uartBuf[260];

// ================== SYNC CORES ==================
static volatile bool g_core0Ready = false;

// barrera simple (orden de memoria entre cores)
static inline void mb() { __asm volatile("" ::: "memory"); }

enum CurrentOp : uint8_t {
  OP_NONE = 0,
  OP_STATUS,
  OP_READ,
  OP_FORMAT,
  OP_PERCOM,
  OP_WRITE
};

static volatile CurrentOp g_currentOp = OP_NONE;

// STATUS remoto
static volatile bool     g_statusDone    = false;
static volatile bool     g_statusSuccess = false;
static uint8_t           g_statusData[4];

// READ remoto (FIX robusto)
static volatile bool     g_readDone        = false;
static volatile bool     g_readSuccess     = false;
static uint8_t           g_readBuf[256];
static volatile int      g_readLen         = 0;
static volatile uint16_t g_readSec         = 0;

static volatile bool     g_readStarted     = false;
static volatile uint8_t  g_readChunkCount  = 0;
static volatile uint32_t g_readChunkMask32 = 0;
static volatile uint32_t g_readLastChunkMs = 0;
static volatile int      g_readExpectedLen = 0;

// FORMAT remoto
static volatile bool     g_formatDone      = false;
static volatile bool     g_formatSuccess   = false;
static uint8_t           g_formatBuf[128];
static volatile int      g_formatLen       = 0;

// PERCOM remoto
static volatile bool     g_percomDone      = false;
static volatile bool     g_percomSuccess   = false;
static uint8_t           g_percomBuf[PERCOM_BLOCK_LEN];

// WRITE remoto
static volatile bool     g_writeDone       = false;
static volatile bool     g_writeSuccess    = false;
static volatile uint16_t g_writeSec        = 0;

// Geometría
static volatile uint16_t g_bytesPerSector = 128;
static volatile bool     g_isDDActive     = false;

// READ_LOADER_V4: geometría local por unidad lógica D1..D7.
// Evita que un PERCOM/WRITE PERCOM de D2 contamine lecturas de D1,
// y permite reintentos limpios para loaders/juegos en DD.
struct LocalDriveGeometry {
  uint16_t bytesPerSector;
  bool isDDActive;
  bool valid;
  uint32_t lastPercomMs;
};
static LocalDriveGeometry g_driveGeom[7];

// ===== NUEVO: CFG (baud UART / baud SIO) =====
static inline uint32_t getLE32(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}
static inline void putLE32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline void putLE16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static volatile uint32_t g_uartBaud = UART_ESP_BAUD;
static volatile uint32_t g_sioBaud  = 19200;

// ===== Cassette manual C: F49 =====
// Sin cable MOTOR pin 8: el MASTER/web inicia y detiene la reproducción manual.
#define CAS_RING_SIZE 4096
#define CAS_RING_MASK (CAS_RING_SIZE - 1)
#define CAS_RECORD_BUFFER_SIZE 2048
static uint8_t g_casRing[CAS_RING_SIZE];
static volatile uint16_t g_casHead = 0;
static volatile uint16_t g_casTail = 0;
static volatile bool g_casPlaying = false;
static volatile bool g_casBootMode = false;
static volatile bool g_casEofFromMaster = false;
static volatile bool g_casUartActive = false;
static volatile bool g_casOverflow = false;
static volatile uint16_t g_casBaud = 600;
static volatile uint32_t g_casBytesPlayed = 0;
static volatile uint32_t g_casBytesRx = 0;
static volatile uint32_t g_casPauseUntilMs = 0;
static volatile uint32_t g_casRecordEndTarget = 0;
static volatile bool g_casRecordFlushPending = false;
static uint32_t g_casLastStatusMs = 0;
static volatile uint16_t g_casLastSeq = 0;
static volatile bool     g_casHaveLastSeq = false;
static volatile uint32_t g_casAckSent = 0;
static volatile uint32_t g_casAckDuplicate = 0;
static volatile uint32_t g_casAckNak = 0;


// F49Z5: buffer de registro cassette completo estilo SDrive-MAX/FujiNet.
// El ESP32 puede transportar el registro en fragmentos con ACK, pero el RP2040
// no lo reproduce hasta recibir el fragmento marcado como fin de registro.
static uint8_t g_casRecordBuf[CAS_RECORD_BUFFER_SIZE];
static volatile uint16_t g_casRecordLen = 0;
static volatile uint16_t g_casRecordPos = 0;
static volatile uint16_t g_casRecordGapMs = 0;
static volatile uint16_t g_casRecordBaud = 600;
static volatile bool g_casRecordAccum = false;
static volatile bool g_casRecordReady = false;
static volatile bool g_casRecordSending = false;
static volatile bool g_casRecordGapStarted = false;
static volatile uint32_t g_casRecordsRx = 0;
static volatile uint32_t g_casRecordsDone = 0;
static volatile uint32_t g_casRecordOverflow = 0;

static inline uint16_t casRingUsed() {
  return (uint16_t)((g_casHead - g_casTail) & CAS_RING_MASK);
}
// [F24] Eliminado helper no usado: casRingFree
static inline void casRingClear() {
  g_casHead = 0;
  g_casTail = 0;
}
// [F24] Eliminado helper no usado: casRingPush
// [F24] Eliminado helper no usado: casRingPop
static inline uint16_t casRecordUsed() {
  if (g_casRecordReady || g_casRecordSending) {
    return (g_casRecordLen >= g_casRecordPos) ? (uint16_t)(g_casRecordLen - g_casRecordPos) : 0;
  }
  if (g_casRecordAccum) return g_casRecordLen;
  return 0;
}
static inline uint16_t casRecordFree() {
  if (g_casRecordReady || g_casRecordSending) return 0;
  if (g_casRecordLen >= CAS_RECORD_BUFFER_SIZE) return 0;
  return (uint16_t)(CAS_RECORD_BUFFER_SIZE - g_casRecordLen);
}
static inline bool casRecordBusy() {
  return g_casRecordAccum || g_casRecordReady || g_casRecordSending || g_casRecordFlushPending;
}
static inline void casRecordClear() {
  g_casRecordLen = 0;
  g_casRecordPos = 0;
  g_casRecordGapMs = 0;
  g_casRecordBaud = g_casBaud ? g_casBaud : 600;
  g_casRecordAccum = false;
  g_casRecordReady = false;
  g_casRecordSending = false;
  g_casRecordGapStarted = false;
  g_casRecordFlushPending = false;
  g_casRecordEndTarget = 0;
  g_casPauseUntilMs = 0;
}

// ===== Printer 820 / Atari P: =====
static volatile bool    g_printerEnabled = true;  // P: listo por defecto para no fallar OPEN #1,"P:"
static volatile uint8_t g_printerSioDev  = SIO_PRINTER_DEFAULT_DEV;

// v19: cola asíncrona solo para impresora P:.
// No se toca la lógica de discos. Core0 responde SIO rápido;
// Core1 envía las líneas al MASTER por UART fuera del tiempo crítico.
#define PRN_QUEUE_SIZE 16
struct PrinterQueueItem {
  uint8_t dev;
  uint8_t cmd;
  uint8_t aux1;
  uint8_t aux2;
  uint8_t len;
  uint8_t data[40];
};
static volatile uint8_t g_prnQHead = 0;
static volatile uint8_t g_prnQTail = 0;
static volatile uint32_t g_prnQDrop = 0;
static volatile uint32_t g_prnQSent = 0;

// v29: pushback SIO de 1 byte.
// Caso observado: en P: a veces el byte leído como checksum es 0x40,
// que en realidad es el DEV del siguiente comando P:. Lo devolvemos para
// que readSioCommandFrame() no pierda el comienzo del próximo frame.
static volatile bool g_sioPushbackValid = false;
static volatile uint8_t g_sioPushbackByte = 0;

// V4: diagnóstico de DATA de disco enviado al MASTER para ver si el Atari
// ACK/NAKea cada sector y detectar si el cuelgue ocurre después de datos OK.
static volatile uint8_t g_lastAtariPostDataResp = 0;
static volatile uint8_t g_lastAtariPostDataFlags = 0;
#define DISK_DIAG_READ_SENT   1
#define DISK_DIAG_READ_NAK    2
#define DISK_DIAG_READ_FAIL   3
#define DISK_DIAG_FLAG_NONE   0x00
#define DISK_DIAG_FLAG_NO_RESP 0x01
#define DISK_DIAG_FLAG_PUSHBACK 0x02
#define DISK_DIAG_FLAG_IGNORED 0x04

// V5: no leer el bus SIO después de enviar DATA de disco.
// Algunos loaders mandan el siguiente comando inmediatamente; dejar los bytes
// en RX permite que readSioCommandFrame() los resincronice sin comerse DEV.
#define DISK_POST_DATA_POLL 0
#define DISK_DIAG_ENABLE 0       // V6: diagnóstico por defecto apagado para no meter tráfico UART durante loaders/juegos
#define SIO_CMD_FALL_DEBOUNCE_US 160


// v24: separar frames de impresora por UART para evitar pérdida en MASTER.
// No afecta SIO ni discos; solo regula TYPE_PRINTER_LINE de P:.
static uint32_t g_prnLastUartLineMs = 0;
#define PRN_UART_LINE_GAP_MS 35

// F49Z9: bloqueo TX para UART RP2040 <-> ESP32 Master.
// Core0 y Core1 pueden emitir frames al Master; sin lock, los bytes de dos
// frames se podían entremezclar y el ESP32 reportaba checksum inválido.
//
// IMPORTANTE: no usar __atomic_test_and_set aquí. En algunas instalaciones
// Arduino-Pico no enlaza libatomic y falla con undefined reference.
// Usamos critical_section de Pico SDK, compatible con RP2040 dual-core.
static critical_section_t g_uartEspTxCs;
static volatile bool g_uartEspTxCsReady = false;

static inline void uartEspTxLockInit() {
  critical_section_init(&g_uartEspTxCs);
  g_uartEspTxCsReady = true;
}

static inline void uartEspTxLock() {
  if (g_uartEspTxCsReady) {
    critical_section_enter_blocking(&g_uartEspTxCs);
  } else {
    noInterrupts();
  }
}

static inline void uartEspTxUnlock() {
  if (g_uartEspTxCsReady) {
    critical_section_exit(&g_uartEspTxCs);
  } else {
    interrupts();
  }
}

// v22 diagnóstico P:
static volatile uint32_t g_prnStatusCount = 0;
static volatile uint32_t g_prnWriteCount = 0;
static volatile uint32_t g_prnWriteOk = 0;
static volatile uint32_t g_prnTimeoutData = 0;
static volatile uint32_t g_prnTimeoutChk = 0;
static volatile uint32_t g_prnChecksumErr = 0;
static volatile uint32_t g_prnUnsupportedCmd = 0;
static uint32_t g_prnLastDiagMs = 0;

static PrinterQueueItem g_prnQ[PRN_QUEUE_SIZE];

static volatile bool     g_cfgPending = false;
static volatile uint32_t g_newUartBaud = 0;
static volatile uint32_t g_newSioBaud  = 0;
static volatile uint32_t g_applyCfgAtMs = 0;

// ================== PROTOTIPOS ==================
uint8_t calcChecksumSIO(const uint8_t *buf, int len);
bool readByteWithTimeoutSIO(uint8_t &outByte, uint16_t timeoutMs);
void dumpCommandFrame(const uint8_t *buf);

void uartSendByte(uint8_t b);
void uartSendFrame(uint8_t type, const uint8_t *payload, uint8_t payloadLen);
void sendCasAckToMaster(uint16_t seq, bool ok);
void sendCasStatusToMaster(bool force);
void casApplyPlaybackBaud(uint16_t baud);
void casRestoreDiskBaud();
void sendDiskDiagToMaster(uint8_t eventCode, uint8_t dev, uint8_t cmd, uint16_t sec, uint16_t len, uint8_t dataChk, uint8_t postResp, uint8_t flags);
void sendSectorChunkToMaster(uint8_t dev, uint16_t sec, const uint8_t *data, uint16_t dataLen);
void onMasterFrame(uint8_t type, const uint8_t *data, uint8_t len);
void serviceUartFromMaster(); // core1

bool sendAtariDataFrame(const uint8_t *buf, int len, bool isFormat);

bool doRemoteStatus(uint8_t dev, uint8_t aux1, uint8_t aux2);
bool doRemoteRead(uint8_t dev, uint16_t sec, uint8_t cmd);
bool doRemoteFormatSD(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2);
bool doRemoteFormatDD(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2);
bool doRemotePercomRead(uint8_t dev);
bool doRemotePercomWrite(uint8_t dev, const uint8_t *data);
bool doRemoteWrite(uint8_t dev, uint16_t sec, uint8_t cmd, bool dd, const uint8_t *data, int len);

bool readSioCommandFrame(uint8_t buf[5]);
bool handlePrinter820Command(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2);
void sendPrinterDiagToMaster() {
  uint32_t vals[9];
  vals[0] = (uint32_t)g_prnStatusCount;
  vals[1] = (uint32_t)g_prnWriteCount;
  vals[2] = (uint32_t)g_prnWriteOk;
  vals[3] = (uint32_t)g_prnQSent;
  vals[4] = (uint32_t)g_prnQDrop;
  vals[5] = (uint32_t)g_prnTimeoutData;
  vals[6] = (uint32_t)g_prnTimeoutChk;
  vals[7] = (uint32_t)g_prnChecksumErr;
  vals[8] = (uint32_t)g_prnUnsupportedCmd;

  uint8_t payload[9 * 4];
  for (uint8_t i = 0; i < 9; i++) {
    payload[i * 4 + 0] = (uint8_t)(vals[i] & 0xFF);
    payload[i * 4 + 1] = (uint8_t)((vals[i] >> 8) & 0xFF);
    payload[i * 4 + 2] = (uint8_t)((vals[i] >> 16) & 0xFF);
    payload[i * 4 + 3] = (uint8_t)((vals[i] >> 24) & 0xFF);
  }

  uartSendFrame(TYPE_PRINTER_DIAG, payload, sizeof(payload));

  Serial.print(F("[PRN-DIAG] status="));
  Serial.print((uint32_t)g_prnStatusCount);
  Serial.print(F(" write="));
  Serial.print((uint32_t)g_prnWriteCount);
  Serial.print(F(" ok="));
  Serial.print((uint32_t)g_prnWriteOk);
  Serial.print(F(" qSent="));
  Serial.print((uint32_t)g_prnQSent);
  Serial.print(F(" qDrop="));
  Serial.print((uint32_t)g_prnQDrop);
  Serial.print(F(" toData="));
  Serial.print((uint32_t)g_prnTimeoutData);
  Serial.print(F(" toChk="));
  Serial.print((uint32_t)g_prnTimeoutChk);
  Serial.print(F(" chkErr="));
  Serial.print((uint32_t)g_prnChecksumErr);
  Serial.print(F(" unsupported="));
  Serial.println((uint32_t)g_prnUnsupportedCmd);
}

void sendDiskDiagToMaster(uint8_t eventCode, uint8_t dev, uint8_t cmd, uint16_t sec, uint16_t len, uint8_t dataChk, uint8_t postResp, uint8_t flags) {
#if !DISK_DIAG_ENABLE
  (void)eventCode; (void)dev; (void)cmd; (void)sec; (void)len; (void)dataChk; (void)postResp; (void)flags;
  return;
#else
  uint8_t payload[10];
  payload[0] = eventCode;
  payload[1] = dev;
  payload[2] = cmd;
  payload[3] = (uint8_t)(sec & 0xFF);
  payload[4] = (uint8_t)((sec >> 8) & 0xFF);
  payload[5] = (uint8_t)(len & 0xFF);
  payload[6] = (uint8_t)((len >> 8) & 0xFF);
  payload[7] = dataChk;
  payload[8] = postResp;
  payload[9] = flags;
  uartSendFrame(TYPE_DISK_DIAG, payload, sizeof(payload));

  Serial.print(F("[DISK-DIAG] ev=")); Serial.print(eventCode);
  Serial.print(F(" dev=0x")); if (dev < 0x10) Serial.print('0'); Serial.print(dev, HEX);
  Serial.print(F(" cmd=0x")); if (cmd < 0x10) Serial.print('0'); Serial.print(cmd, HEX);
  Serial.print(F(" sec=")); Serial.print(sec);
  Serial.print(F(" len=")); Serial.print(len);
  Serial.print(F(" chk=0x")); if (dataChk < 0x10) Serial.print('0'); Serial.print(dataChk, HEX);
  Serial.print(F(" post=0x")); if (postResp < 0x10) Serial.print('0'); Serial.print(postResp, HEX);
  Serial.print(F(" flags=0x")); if (flags < 0x10) Serial.print('0'); Serial.println(flags, HEX);
#endif
}

void handleCasControlFromMaster(const uint8_t* data, uint8_t len);

void servicePrinterDiag() {
  // F49Z8: durante cassette reducimos ruido UART hacia el Master.
  // La reproducción CAS usa ACK/status frecuentes; los diagnósticos de impresora
  // no aportan durante C: y aumentaban la probabilidad de frames mezclados.
  if (g_casPlaying || g_casUartActive || casRecordBusy()) {
    return;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - g_prnLastDiagMs) >= 15000) {
    g_prnLastDiagMs = now;
    sendPrinterDiagToMaster();
  }
}

void casApplyPlaybackBaud(uint16_t baud) {
  if (baud < 300) baud = 300;
  if (baud > 6000) baud = 6000;

  // Cambiar la UART SIO a velocidad cassette solo durante reproducción C:.
  // Se reconfigura de forma explícita para evitar quedar mezclando 19200 con velocidades cassette/turbo.
  SerialSIO.flush();
  SerialSIO.end();
  delay(2);
  SerialSIO.begin((uint32_t)baud);
  gpio_set_function(PIN_SIO_TX, GPIO_FUNC_UART);
  gpio_set_function(PIN_SIO_RX, GPIO_FUNC_UART);

  g_casBaud = baud;
  g_casUartActive = true;
  Serial.print(F("[CAS] SIO baud cassette="));
  Serial.println(baud);
}

void casRestoreDiskBaud() {
  // Restaurar la velocidad normal de SIO usada por discos/impresora.
  SerialSIO.flush();
  SerialSIO.end();
  delay(2);
  SerialSIO.begin((uint32_t)(g_sioBaud ? g_sioBaud : 19200));
  gpio_set_function(PIN_SIO_TX, GPIO_FUNC_UART);
  gpio_set_function(PIN_SIO_RX, GPIO_FUNC_UART);

  g_casUartActive = false;
  Serial.print(F("[CAS] SIO baud restaurado="));
  Serial.println((uint32_t)(g_sioBaud ? g_sioBaud : 19200));
}

void handleCasControlFromMaster(const uint8_t* data, uint8_t len) {
  if (!data || len < 9) return;

  uint8_t cmd = data[0];
  uint32_t baud32 = getLE32(data + 1);
  uint32_t totalBytes = getLE32(data + 5);
  uint16_t baud = (uint16_t)baud32;
  if (baud < 300) baud = 300;
  if (baud > 6000) baud = 6000;

  switch (cmd) {
    case 1: // Play CLOAD
    case 5: // Play Boot cassette
      casRingClear();
      casRecordClear();
      g_casPlaying = true;
      g_casBootMode = (cmd == 5);
      g_casEofFromMaster = false;
      g_casOverflow = false;
      g_casBytesPlayed = 0;
      g_casBytesRx = 0;
      g_casPauseUntilMs = 0;
      g_casRecordEndTarget = 0;
      g_casRecordFlushPending = false;
      g_casHaveLastSeq = false;
      g_casLastSeq = 0;
      g_casAckSent = 0;
      g_casAckDuplicate = 0;
      g_casAckNak = 0;
      casApplyPlaybackBaud(baud);
      Serial.print(F("[CAS] PLAY "));
      Serial.print(g_casBootMode ? F("BOOT") : F("CLOAD"));
      Serial.print(F(" baud=")); Serial.print(baud);
      Serial.print(F(" total=")); Serial.println(totalBytes);
      sendCasStatusToMaster(true);
      break;

    case 2: // Stop
      g_casPlaying = false;
      g_casEofFromMaster = false;
      g_casRecordFlushPending = false;
      g_casRecordEndTarget = 0;
      g_casPauseUntilMs = 0;
      casRingClear();
      casRecordClear();
      casRestoreDiskBaud();
      Serial.println(F("[CAS] STOP"));
      sendCasStatusToMaster(true);
      break;

    case 3: // Rewind lógico: limpiar buffer/contadores, mantener modo listo
      casRingClear();
      casRecordClear();
      g_casBytesPlayed = 0;
      g_casBytesRx = 0;
      g_casEofFromMaster = false;
      g_casOverflow = false;
      g_casPauseUntilMs = 0;
      g_casRecordEndTarget = 0;
      g_casRecordFlushPending = false;
      g_casHaveLastSeq = false;
      g_casLastSeq = 0;
      Serial.println(F("[CAS] REWIND"));
      sendCasStatusToMaster(true);
      break;

    case 4: // EOF enviado por Master
      g_casEofFromMaster = true;
      Serial.println(F("[CAS] EOF desde MASTER"));
      sendCasStatusToMaster(true);
      break;

    default:
      Serial.print(F("[CAS] control desconocido cmd="));
      Serial.println(cmd);
      sendCasStatusToMaster(true);
      break;
  }
}

void sendCasAckToMaster(uint16_t seq, bool ok) {
  uint8_t payload[10];
  putLE16(payload + 0, seq);
  payload[2] = ok ? 1 : 0;
  putLE32(payload + 3, (uint32_t)g_casBytesRx);
  putLE16(payload + 7, casRecordFree());
  payload[9] = (g_casUartActive ? 0x01 : 0x00) |
               (g_casOverflow ? 0x02 : 0x00) |
               (g_casEofFromMaster ? 0x04 : 0x00) |
               (g_casBootMode ? 0x08 : 0x00) |
               ((casRecordBusy()) ? 0x10 : 0x00);
  uartSendFrame(TYPE_CAS_ACK, payload, sizeof(payload));
  if (ok) g_casAckSent++; else g_casAckNak++;
}

void handleCasDataFromMaster(const uint8_t* data, uint8_t len) {
  if (!data || len < 7) return;
  uint16_t seq = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  uint16_t baud = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
  uint16_t gapMs = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
  uint8_t flags = data[6];
  const uint8_t* payload = data + 7;
  uint16_t dlen = (uint16_t)len - 7;

  // ACK idempotente: si el ESP32 reintenta el mismo fragmento porque se perdió
  // el ACK, no duplicamos bytes en el registro. Solo re-confirmamos.
  if (g_casHaveLastSeq && seq == g_casLastSeq) {
    g_casAckDuplicate++;
    sendCasAckToMaster(seq, true);
    sendCasStatusToMaster(true);
    return;
  }

  if (!g_casPlaying) {
    sendCasAckToMaster(seq, false);
    sendCasStatusToMaster(true);
    return;
  }

  bool startRecord = (flags & 0x01) != 0;
  bool endRecord = (flags & 0x02) != 0;

  if (startRecord) {
    // No aceptar un nuevo registro mientras el anterior está listo/en transmisión.
    // Esto replica la disciplina de SDrive-MAX: gap -> registro completo -> flush.
    if (g_casRecordReady || g_casRecordSending || g_casRecordFlushPending) {
      sendCasAckToMaster(seq, false);
      sendCasStatusToMaster(true);
      return;
    }
    g_casRecordLen = 0;
    g_casRecordPos = 0;
    g_casRecordGapMs = gapMs > 60000 ? 60000 : gapMs;
    g_casRecordBaud = (baud >= 300 && baud <= 6000) ? baud : (g_casBaud ? g_casBaud : 600);
    g_casRecordAccum = true;
    g_casRecordReady = false;
    g_casRecordSending = false;
    g_casRecordGapStarted = false;
  } else if (!g_casRecordAccum) {
    // Tolerancia: si por algún motivo el primer fragmento llegó sin START,
    // iniciamos registro sin gap para no perder la cinta.
    if (g_casRecordReady || g_casRecordSending || g_casRecordFlushPending) {
      sendCasAckToMaster(seq, false);
      sendCasStatusToMaster(true);
      return;
    }
    g_casRecordLen = 0;
    g_casRecordPos = 0;
    g_casRecordGapMs = 0;
    g_casRecordBaud = (baud >= 300 && baud <= 6000) ? baud : (g_casBaud ? g_casBaud : 600);
    g_casRecordAccum = true;
    g_casRecordGapStarted = false;
  }

  if ((uint32_t)g_casRecordLen + (uint32_t)dlen > CAS_RECORD_BUFFER_SIZE) {
    g_casOverflow = true;
    g_casRecordOverflow++;
    sendCasAckToMaster(seq, false);
    sendCasStatusToMaster(true);
    return;
  }

  for (uint16_t i = 0; i < dlen; i++) {
    g_casRecordBuf[g_casRecordLen++] = payload[i];
    g_casBytesRx++;
  }

  if (endRecord) {
    g_casRecordAccum = false;
    g_casRecordReady = true;
    g_casRecordSending = false;
    g_casRecordPos = 0;
    g_casRecordGapStarted = false;
    g_casRecordsRx++;
  }

  g_casLastSeq = seq;
  g_casHaveLastSeq = true;
  sendCasAckToMaster(seq, true);
  sendCasStatusToMaster(true);
}


void serviceCassettePlayback() {
  if (!g_casPlaying) return;
  if (!g_casUartActive) casApplyPlaybackBaud(g_casBaud ? g_casBaud : 600);

  uint32_t now = millis();

  if (g_casRecordReady) {
    if (!g_casRecordSending) {
      if (g_casRecordBaud >= 300 && g_casRecordBaud <= 6000 && g_casRecordBaud != g_casBaud) {
        casApplyPlaybackBaud(g_casRecordBaud);
      }
      if (!g_casRecordGapStarted) {
        g_casRecordGapStarted = true;
        if (g_casRecordGapMs > 0) {
          g_casPauseUntilMs = now + (uint32_t)g_casRecordGapMs;
          sendCasStatusToMaster(true);
          return;
        }
      }
      g_casRecordSending = true;
      g_casRecordFlushPending = true;
      g_casRecordEndTarget = g_casBytesPlayed + g_casRecordLen;
    }
  }

  if (g_casPauseUntilMs && (int32_t)(now - g_casPauseUntilMs) < 0) return;
  g_casPauseUntilMs = 0;

  if (g_casRecordReady && g_casRecordSending) {
    uint8_t sent = 0;
    while (g_casRecordPos < g_casRecordLen && SerialSIO.availableForWrite() > 0) {
      SerialSIO.write(g_casRecordBuf[g_casRecordPos++]);
      g_casBytesPlayed++;
      sent++;
      if (sent >= 32) break;
    }

    if (sent > 0 && (sent >= 16 || (uint32_t)(now - g_casLastStatusMs) >= 100)) {
      sendCasStatusToMaster(true);
    }

    if (g_casRecordPos >= g_casRecordLen) {
      SerialSIO.flush();
      g_casRecordReady = false;
      g_casRecordSending = false;
      g_casRecordGapStarted = false;
      g_casRecordLen = 0;
      g_casRecordPos = 0;
      g_casRecordFlushPending = false;
      g_casRecordEndTarget = 0;
      g_casRecordsDone++;
      sendCasStatusToMaster(true);
    }
  }

  if (g_casEofFromMaster && !casRecordBusy() && casRingUsed() == 0) {
    SerialSIO.flush();
    g_casPlaying = false;
    g_casEofFromMaster = false;
    casRestoreDiskBaud();
    Serial.println(F("[CAS] reproducción terminada"));
    sendCasStatusToMaster(true);
  }
}


void sendCasStatusToMaster(bool force) {
  uint32_t now = millis();
  if (!force && (uint32_t)(now - g_casLastStatusMs) < 200) return;
  g_casLastStatusMs = now;

  uint8_t payload[16];
  payload[0] = g_casPlaying ? 1 : 0;
  putLE16(payload + 1, casRecordUsed());
  putLE16(payload + 3, casRecordFree());
  putLE32(payload + 5, (uint32_t)g_casBytesPlayed);
  putLE32(payload + 9, (uint32_t)g_casBytesRx);
  putLE16(payload + 13, (uint16_t)g_casBaud);
  payload[15] = (g_casUartActive ? 0x01 : 0x00) |
                (g_casOverflow ? 0x02 : 0x00) |
                (g_casEofFromMaster ? 0x04 : 0x00) |
                (g_casBootMode ? 0x08 : 0x00) |
                ((casRecordBusy()) ? 0x10 : 0x00);
  uartSendFrame(TYPE_CAS_STATUS, payload, sizeof(payload));
}

void sendPrinterLineToMaster(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2, const uint8_t* data, uint8_t len);
bool enqueuePrinterLineForMaster(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2, const uint8_t* data, uint8_t len);
void servicePrinterQueueToMaster();
void sendPrinterDiagToMaster();
void servicePrinterDiag();
void handleSioCommand();
void sioSendError(uint8_t dev, uint8_t cmd, uint16_t sec, const __FlashStringHelper* motivo);

void initGeometry();
int getLocalSectorSizeForWrite(uint16_t sec, uint8_t cmd);
void updateGeometryFromPercom(uint8_t dev, const uint8_t percom[PERCOM_BLOCK_LEN]);
static int devIndex(uint8_t dev);
static void applyGeometryForDev(uint8_t dev);
static void saveGeometryForDev(uint8_t dev, uint16_t bytesPerSector, bool isDD);
static uint16_t bytesPerSectorForDev(uint8_t dev);
static bool isDDActiveForDev(uint8_t dev);
static inline bool sioLooksLikeDeviceId(uint8_t b);
static inline void sioPushbackByte(uint8_t b);

static inline bool isCmdDD(uint8_t cmd) { return (cmd & 0x80) != 0; }

static inline uint32_t fullMask32(uint8_t count) {
  if (count == 0) return 0;
  if (count >= 32) return 0xFFFFFFFFUL;
  return (1UL << count) - 1UL;
}

void printStats() {
  Serial.println(F("\n=== ESTADÍSTICAS RP2040 ==="));
  Serial.print(F("Total READs:  "));   Serial.println(g_totalReads);
  Serial.print(F("Total WRITEs: "));   Serial.println(g_totalWrites);
  Serial.print(F("READ errors:  "));   Serial.println(g_readErrors);
  Serial.print(F("WRITE errors: "));   Serial.println(g_writeErrors);

  Serial.print(F("UART RX bad frames (chk): ")); Serial.println((uint32_t)g_uart_bad_chk);
  Serial.print(F("UART RX bad len:          ")); Serial.println((uint32_t)g_uart_bad_len);

  if (g_lastErrorSector > 0) {
    Serial.print(F("Último sector error: ")); Serial.println(g_lastErrorSector);
  }
  Serial.println(F("===========================\n"));
}

// ================== Utilidades ==================

uint8_t calcChecksumSIO(const uint8_t *buf, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++) {
    s += buf[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;
  }
  return (uint8_t)(s & 0xFF);
}

bool readByteWithTimeoutSIO(uint8_t &outByte, uint16_t timeoutMs) {
  if (g_sioPushbackValid) {
    outByte = g_sioPushbackByte;
    g_sioPushbackValid = false;
    return true;
  }

  unsigned long t0 = millis();
  while (!SerialSIO.available()) {
    if (millis() - t0 >= timeoutMs) return false;
  }
  outByte = (uint8_t)SerialSIO.read();
  return true;
}

void dumpCommandFrame(const uint8_t *buf) {
  uint8_t dev  = buf[0];
  uint8_t cmd  = buf[1];
  uint8_t aux1 = buf[2];
  uint8_t aux2 = buf[3];
  uint8_t chk  = buf[4];
  uint8_t calc = calcChecksumSIO(buf, 4);

  Serial.print(F("[SIO] Frame CMD: DEV=0x"));
  if (dev < 0x10) Serial.print('0');
  Serial.print(dev, HEX);

  Serial.print(F(" CMD=0x"));
  if (cmd < 0x10) Serial.print('0');
  Serial.print(cmd, HEX);

  Serial.print(F(" AUX1=0x"));
  if (aux1 < 0x10) Serial.print('0');
  Serial.print(aux1, HEX);

  Serial.print(F(" AUX2=0x"));
  if (aux2 < 0x10) Serial.print('0');
  Serial.print(aux2, HEX);

  Serial.print(F(" CHK=0x"));
  if (chk < 0x10) Serial.print('0');
  Serial.print(chk, HEX);

  Serial.print(F(" (calc=0x"));
  if (calc < 0x10) Serial.print('0');
  Serial.print(calc, HEX);
  Serial.println(F(")"));

  if (dev >= SIO_DEV_MIN && dev <= SIO_DEV_MAX) {
    Serial.print(F("[SIO] → Comando dirigido a D"));
    Serial.println(dev - 0x30);
  }
}

// ================== UART → MASTER ==================

void uartSendByte(uint8_t b) {
  uart_putc_raw(UART_ESP, b);
}

void uartSendFrame(uint8_t type, const uint8_t *payload, uint8_t payloadLen) {
  uint8_t len = 1 + payloadLen;

  uint8_t sum = type;
  for (uint8_t i = 0; i < payloadLen; i++) sum += payload[i];

  // F49Z8: esta UART la usan ambos cores. El frame debe salir atómico
  // para que el Master no vea bytes intercalados y marque checksum inválido.
  uartEspTxLock();
  uartSendByte(0x55);
  uartSendByte(len);
  uartSendByte(type);
  if (payloadLen > 0) uart_write_blocking(UART_ESP, payload, payloadLen);
  uartSendByte(sum);
  uartEspTxUnlock();
}

void sendSectorChunkToMaster(uint8_t dev, uint16_t sec, const uint8_t *data, uint16_t dataLen) {
  if (dataLen == 0) {
    dataLen = (uint16_t)g_bytesPerSector;
    if (dataLen == 0 || dataLen > 256) dataLen = 128;
  }
  if (dataLen > 256) dataLen = 256;

  const uint8_t UART_CHUNK_MAX = (uint8_t)CHUNK_PAYLOAD;

  uint16_t total = dataLen;
  uint16_t sent  = 0;
  uint8_t  idx   = 0;
  uint8_t  count = (total + UART_CHUNK_MAX - 1) / UART_CHUNK_MAX;
  if (count == 0) count = 1;

  while (sent < total) {
    uint16_t thisLen = total - sent;
    if (thisLen > UART_CHUNK_MAX) thisLen = UART_CHUNK_MAX;

    uint8_t chunk[5 + CHUNK_PAYLOAD];
    chunk[0] = dev;
    chunk[1] = (uint8_t)(sec & 0xFF);
    chunk[2] = (uint8_t)(sec >> 8);
    chunk[3] = idx;
    chunk[4] = count;

    if (data && thisLen) memcpy(chunk + 5, data + sent, thisLen);

    uint8_t payloadLen = (uint8_t)(5 + thisLen);
    uartSendFrame(TYPE_SECTOR_CHUNK, chunk, payloadLen);

    sent += thisLen;
    idx++;
  }
}

// ================== Procesar frame recibido del MASTER (core1) ==================

void onMasterFrame(uint8_t type, const uint8_t *data, uint8_t len) {
  switch (type) {

    case TYPE_CAS_CONTROL: {
      handleCasControlFromMaster(data, len);
    } break;

    case TYPE_CAS_DATA: {
      handleCasDataFromMaster(data, len);
    } break;

    case TYPE_PRINTER_CFG_UPDATE: {
      if (len < 2) break;

      bool en = (data[0] != 0);
      uint8_t dev = data[1];
      if (dev < 0x40 || dev > 0x43) dev = SIO_PRINTER_DEFAULT_DEV;

      g_printerEnabled = en;
      g_printerSioDev = dev;

      uint8_t ack[2];
      ack[0] = g_printerEnabled ? 1 : 0;
      ack[1] = g_printerSioDev;
      uartSendFrame(TYPE_PRINTER_CFG_ACK, ack, sizeof(ack));

      Serial.print(F("[RP2040] PRINTER_CFG_UPDATE recibido enabled="));
      Serial.print(g_printerEnabled ? 1 : 0);
      Serial.print(F(" dev=0x"));
      if (g_printerSioDev < 0x10) Serial.print('0');
      Serial.println(g_printerSioDev, HEX);
    } break;

    case TYPE_CFG_UPDATE: {
      if (len < 8) break;

      uint32_t newUart = getLE32(&data[0]);
      uint32_t newSio  = getLE32(&data[4]);

      // clamps seguros
      if (newUart < 57600) newUart = 57600;
      if (newUart > 2000000) newUart = 2000000;

      if (newSio < 9600) newSio = 9600;
      if (newSio > 115200) newSio = 115200;

      // responde ACK antes de cambiar baud
      uint8_t ack[9];
      ack[0] = 1; // ok
      putLE32(&ack[1], newUart);
      putLE32(&ack[5], newSio);
      uartSendFrame(TYPE_CFG_ACK, ack, sizeof(ack));

      g_cfgPending  = true;
      g_newUartBaud = newUart;
      g_newSioBaud  = newSio;
      g_applyCfgAtMs = millis() + 50;

      Serial.print(F("[RP2040] CFG_UPDATE recibido uart="));
      Serial.print(newUart);
      Serial.print(F(" sio="));
      Serial.println(newSio);
    } break;

    case TYPE_TIMING_UPDATE: {
      if (len < 8) {
        Serial.println(F("[RP2040] TIMING_UPDATE demasiado corto"));
        break;
      }

      uint16_t ack2comp  = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
      uint16_t comp2data = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
      uint16_t data2chk  = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
      uint16_t chDelay   = (uint16_t)data[6] | ((uint16_t)data[7] << 8);

      auto clamp16 = [](uint16_t v, uint16_t mn, uint16_t mx) -> uint16_t {
        if (v < mn) return mn;
        if (v > mx) return mx;
        return v;
      };

      T_ACK_TO_COMPLETE  = clamp16(ack2comp,  200, 5000);
      T_COMPLETE_TO_DATA = clamp16(comp2data, 200, 5000);
      T_DATA_TO_CHK      = clamp16(data2chk,   20, 2000);
      T_CHUNK_DELAY      = clamp16(chDelay,   100, 8000);

      Serial.print(F("[RP2040] TIMINGS desde MASTER -> "));
      Serial.print(F("ack2comp="));  Serial.print(T_ACK_TO_COMPLETE);
      Serial.print(F(" comp2data=")); Serial.print(T_COMPLETE_TO_DATA);
      Serial.print(F(" data2chk="));  Serial.print(T_DATA_TO_CHK);
      Serial.print(F(" chunkDelay="));Serial.println(T_CHUNK_DELAY);
    } break;

    case TYPE_HELLO: {
      if (len >= 2) {
        uint8_t devId  = data[0];
        uint8_t sup256 = data[1];
        Serial.print(F("[RP2040] HELLO SLAVE DEV=0x"));
        if (devId < 0x10) Serial.print('0');
        Serial.print(devId, HEX);
        Serial.print(F(" DD="));
        Serial.println(sup256 ? 1 : 0);
      }
    } break;

    case TYPE_ACK: {
      Serial.println(F("[RP2040] ACK desde MASTER"));
      if (g_currentOp == OP_WRITE) {
        g_writeDone    = true;
        g_writeSuccess = true;
        mb();
        g_currentOp    = OP_NONE;
      }
    } break;

    case TYPE_NAK: {
      Serial.println(F("[RP2040] NAK desde MASTER"));

      if (g_currentOp == OP_READ) {
        g_readDone    = true;
        g_readSuccess = false;
        mb();
        g_currentOp   = OP_NONE;
        g_readErrors++;
        g_lastErrorSector = (uint16_t)g_readSec;
        Serial.println(F("[RP2040] ❌ NAK en READ"));
        return;
      } else {
        if (g_currentOp == OP_STATUS) {
          g_statusDone    = true;
          g_statusSuccess = false;
        } else if (g_currentOp == OP_FORMAT) {
          g_formatDone    = true;
          g_formatSuccess = false;
        } else if (g_currentOp == OP_PERCOM) {
          g_percomDone    = true;
          g_percomSuccess = false;
        } else if (g_currentOp == OP_WRITE) {
          g_writeDone    = true;
          g_writeSuccess = false;
          g_writeErrors++;
          g_lastErrorSector = (uint16_t)g_writeSec;
        }
        mb();
        g_currentOp = OP_NONE;
      }
    } break;

    case TYPE_SECTOR_CHUNK: {
      if (len < 5) {
        Serial.println(F("[RP2040] SECTOR_CHUNK demasiado corto"));
        return;
      }

      uint8_t dev    = data[0];
      uint16_t sec   = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
      uint8_t idx    = data[3];
      uint8_t count  = data[4];
      int     dlen   = (int)len - 5;
      const uint8_t *payload = data + 5;

      // STATUS
      if (g_currentOp == OP_STATUS && sec == 0) {
        int copyLen = (dlen > 4) ? 4 : dlen;
        memcpy(g_statusData, payload, copyLen);
        g_statusDone    = true;
        g_statusSuccess = (copyLen == 4);
        mb();
        g_currentOp     = OP_NONE;
        if (!g_statusSuccess) Serial.println(F("[RP2040] STATUS: longitud != 4"));
        return;
      }

      // ✅✅✅ READ (FIX robusto) ✅✅✅
      if (g_currentOp == OP_READ && sec == g_readSec) {
        g_readLastChunkMs = millis();

        if (!g_readStarted) {
          g_readStarted     = true;
          g_readChunkCount  = (count == 0) ? 1 : count;
          g_readChunkMask32 = 0;
          g_readLen         = 0;
          memset(g_readBuf, 0, sizeof(g_readBuf));

          Serial.print(F("[RP2040] READ iniciado sec="));
          Serial.print(sec);
          Serial.print(F(" esperando "));
          Serial.print(g_readChunkCount);
          Serial.println(F(" chunks"));
        }

        // Copiar por offset
        int off = (int)idx * (int)CHUNK_PAYLOAD;
        if (off < 0) off = 0;
        if (off >= (int)sizeof(g_readBuf)) off = (int)sizeof(g_readBuf);

        int room = (int)sizeof(g_readBuf) - off;
        if (dlen > room) dlen = room;
        if (dlen > 0) memcpy(g_readBuf + off, payload, dlen);

        int endPos = off + dlen;
        if (endPos > g_readLen) g_readLen = endPos;

        // Marcar chunk
        if (idx < 32) {
          uint32_t bit = (1UL << idx);
          g_readChunkMask32 |= bit;
        }

        uint32_t need = fullMask32(g_readChunkCount);
        if (need != 0 && ((g_readChunkMask32 & need) == need)) {
          g_readDone    = true;
          g_readSuccess = true;
          mb();
          g_currentOp   = OP_NONE;

          Serial.print(F("[RP2040] ✅ READ completo sec="));
          Serial.print(sec);
          Serial.print(F(" len="));
          Serial.println((int)g_readLen);
        }
        return;
      }

      // FORMAT
      if (g_currentOp == OP_FORMAT && sec == 0) {
        int copyLen = (dlen > (int)sizeof(g_formatBuf)) ? (int)sizeof(g_formatBuf) : dlen;
        memcpy(g_formatBuf, payload, copyLen);
        g_formatLen      = copyLen;
        g_formatDone     = true;
        g_formatSuccess  = (copyLen > 0);
        mb();
        g_currentOp      = OP_NONE;

        if (!g_formatSuccess) {
          Serial.println(F("[RP2040] FORMAT: longitud 0"));
        } else {
          Serial.print(F("[RP2040] FORMAT resultado len="));
          Serial.println((int)g_formatLen);
        }
        return;
      }

      // PERCOM READ
      if (g_currentOp == OP_PERCOM && sec == PERCOM_SEC_MAGIC) {
        int copyLen = (dlen > PERCOM_BLOCK_LEN) ? PERCOM_BLOCK_LEN : dlen;
        memcpy(g_percomBuf, payload, copyLen);
        g_percomDone     = true;
        g_percomSuccess  = (copyLen == PERCOM_BLOCK_LEN);
        mb();
        g_currentOp      = OP_NONE;

        if (!g_percomSuccess) Serial.println(F("[RP2040] PERCOM: longitud incorrecta"));
        return;
      }
    } break;

    default:
      break;
  }
}

// ================== FSM UART desde MASTER (CORE1) ==================

void serviceUartFromMaster() {
  while (uart_is_readable(UART_ESP)) {
    uint8_t b = (uint8_t)uart_getc(UART_ESP);

    switch (uartState) {
      case 0:
        if (b == 0x55) uartState = 1;
        break;

      case 1:
        uartLen = b;
        uartIdx = 0;
        if (uartLen == 0 || uartLen >= sizeof(uartBuf)) {
          uartState = 0;
          g_uart_bad_len++;
          break;
        }
        uartState = 2;
        break;

      case 2:
        uartBuf[uartIdx++] = b;
        if (uartIdx >= uartLen) uartState = 3;
        break;

      case 3: {
        uint8_t chk = b;

        uint8_t sum = 0;
        for (uint8_t i = 0; i < uartLen; i++) sum += uartBuf[i];

        if (sum == chk) {
          uint8_t type  = uartBuf[0];
          uint8_t *pl   = &uartBuf[1];
          uint8_t plen  = uartLen - 1;
          onMasterFrame(type, pl, plen);
        } else {
          g_uart_bad_chk++;
          // (evitar prints masivos aquí; ya tienes contadores)
        }

        uartState = 0;
      } break;

      default:
        uartState = 0;
        break;
    }
  }
}

// ================== SIO → Atari (DATA FRAME) ==================
// Envía: COMPLETE + DATA + CHK usando SerialSIO (seguro para cualquier mapeo UART del core)
// Si falla algún remoto, el handler enviará SIO_ERROR para no dejar al Atari esperando.

static inline void drainSioRx(uint16_t msMax) {
  unsigned long t0 = millis();
  while (SerialSIO.available()) (void)SerialSIO.read();
  while ((millis() - t0) < msMax) {
    if (SerialSIO.available()) {
      while (SerialSIO.available()) (void)SerialSIO.read();
      break;
    }
    delay(0);
  }
}

bool sendAtariDataFrame(const uint8_t *buf, int len, bool isFormat) {
  g_lastAtariPostDataResp = 0;
  g_lastAtariPostDataFlags = DISK_DIAG_FLAG_NO_RESP;

  // Drenar basura previa (importante después de operaciones largas),
  // pero SOLO antes de comenzar la respuesta. Después del checksum no se debe
  // drenar el bus: algunos loaders arrancan el siguiente comando inmediatamente
  // y el primer byte puede ser DEV=0x31..0x34.
  drainSioRx(1);

  // COMPLETE
  delayMicroseconds(T_ACK_TO_COMPLETE);
  SerialSIO.write(SIO_COMPLETE);
  SerialSIO.flush();

  // DATA
  delayMicroseconds(T_COMPLETE_TO_DATA);
  SerialSIO.write(buf, len);
  SerialSIO.flush();

  // CHECKSUM
  delayMicroseconds(T_DATA_TO_CHK);
  uint8_t chk = calcChecksumSIO(buf, len);
  SerialSIO.write(chk);
  SerialSIO.flush();

  delayMicroseconds(T_CHUNK_DELAY);

#if DISK_POST_DATA_POLL
  // V3/V4: ventana post-DATA corta. En V5 queda desactivada por defecto.
  // Si se re-habilita, solo consume ACK/NAK o hace pushback de DEV real.
  uint8_t resp;
  unsigned long t0 = micros();
  while ((uint32_t)(micros() - t0) < 2500UL) {
    if (SerialSIO.available()) {
      resp = (uint8_t)SerialSIO.read();
      if (resp == SIO_ACK) {
        g_lastAtariPostDataResp = resp;
        g_lastAtariPostDataFlags = DISK_DIAG_FLAG_NONE;
        return true;
      }
      if (resp == SIO_NAK) {
        g_lastAtariPostDataResp = resp;
        g_lastAtariPostDataFlags = DISK_DIAG_FLAG_NONE;
        return false;
      }
      if (sioLooksLikeDeviceId(resp)) {
        g_lastAtariPostDataResp = resp;
        g_lastAtariPostDataFlags = DISK_DIAG_FLAG_PUSHBACK;
        sioPushbackByte(resp);
        return true;
      }
      g_lastAtariPostDataResp = resp;
      g_lastAtariPostDataFlags = DISK_DIAG_FLAG_IGNORED;
      return true;
    }
    delayMicroseconds(50);
  }
#else
  // V5: no tocar RX post-DATA. Si el Atari dejó ACK atrasado o ya empezó
  // un frame nuevo, se queda en el buffer y readSioCommandFrame() lo filtra.
  g_lastAtariPostDataResp = 0;
  g_lastAtariPostDataFlags = DISK_DIAG_FLAG_NO_RESP;
#endif

  // Si no llegó nada / no se leyó post-DATA, asumimos OK.
  return true;
}

// ================== Operaciones remotas ==================

bool doRemoteStatus(uint8_t dev, uint8_t aux1, uint8_t aux2) {
  uint8_t payload[6];
  payload[0] = 0x53;
  payload[1] = dev;
  payload[2] = aux1;
  payload[3] = aux2;
  payload[4] = 0;
  payload[5] = 0;

  g_currentOp     = OP_STATUS;
  g_statusDone    = false;
  g_statusSuccess = false;

  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 2000;
  while (!g_statusDone && (millis() - t0) < TIMEOUT_MS) {
    delay(0); // core1 procesa UART
  }
  g_currentOp = OP_NONE;

  if (!g_statusDone || !g_statusSuccess) {
    Serial.println(F("[RP2040] STATUS remoto FALLÓ"));
    return false;
  }

  mb();
  sendAtariDataFrame(g_statusData, 4, false);
  return true;
}

// ✅✅✅ doRemoteRead con validación estricta + rescate DD para loaders ✅✅✅
bool doRemoteRead(uint8_t dev, uint16_t sec, uint8_t cmd) {
  applyGeometryForDev(dev);

  bool reqDD = isCmdDD(cmd) || isDDActiveForDev(dev);
  uint16_t devBytes = bytesPerSectorForDev(dev);

  int expectedLen = 128;
  if (reqDD && !(sec >= 1 && sec <= 3)) {
    expectedLen = (devBytes > 128) ? (int)devBytes : 256;
    if (expectedLen > 256) expectedLen = 256;
  }
  if (expectedLen < 128) expectedLen = 128;
  g_readExpectedLen = expectedLen;

  bool finalRemoteOk = false;
  bool forceDDRetry = false;

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    bool sendDD = reqDD || forceDDRetry;

    uint8_t payload[6];
    payload[0] = 0x52;
    payload[1] = dev;
    payload[2] = (uint8_t)(sec & 0xFF);
    payload[3] = (uint8_t)(sec >> 8);
    payload[4] = (sendDD ? 1 : 0);
    payload[5] = 0;

    g_currentOp      = OP_READ;
    g_readDone       = false;
    g_readSuccess    = false;
    g_readSec        = sec;
    g_readLen        = 0;

    g_readStarted     = false;
    g_readChunkCount  = 0;
    g_readChunkMask32 = 0;
    g_readLastChunkMs = millis();
    memset(g_readBuf, 0, sizeof(g_readBuf));

    Serial.print(F("[RP2040] >>> READ sec="));
    Serial.print(sec);
    Serial.print(F(" dens="));
    Serial.print(sendDD ? F("DD") : F("SD/ED"));
    Serial.print(F(" esperado="));
    Serial.print(expectedLen);
    Serial.print(F(" bytes intento="));
    Serial.println((unsigned)(attempt + 1));

    uartSendFrame(TYPE_CMD_FRAME, payload, 6);

    unsigned long t0 = millis();
    const unsigned long TIMEOUT_MS = 20000;

    while (!g_readDone && (millis() - t0) < TIMEOUT_MS) {
      delay(0);

      if (g_readStarted && (millis() - g_readLastChunkMs) > 4000) {
        Serial.println(F("[RP2040] ❌ READ: timeout chunks (gap > 4s)"));
        break;
      }
    }
    g_currentOp = OP_NONE;

    if (!g_readDone || !g_readSuccess) {
      Serial.println(F("[RP2040] ❌ READ remoto FALLÓ"));
      finalRemoteOk = false;
      break;
    }

    mb();

    // Si esperábamos 256 pero llegó 128, no rellenar con ceros: eso corrompe loaders/juegos.
    // Primero forzamos un reintento DD real hacia el SLAVE/XF551.
    if (expectedLen == 256 && g_readLen == 128 && !(sec >= 1 && sec <= 3) && !forceDDRetry) {
      Serial.print(F("[READ_LOADER_V4] ⚠️ mismatch 256->128 sec="));
      Serial.print(sec);
      Serial.println(F("; forzando DD y reintentando lectura"));
      forceDDRetry = true;
      saveGeometryForDev(dev, 256, true);
      applyGeometryForDev(dev);
      continue;
    }

    finalRemoteOk = true;
    break;
  }

  if (!finalRemoteOk) {
    g_readErrors++;
    g_lastErrorSector = sec;
    return false;
  }

  // ✅ VALIDACIÓN ESTRICTA
  if (g_readLen != expectedLen) {
    bool got128instead256 = (expectedLen == 256 && g_readLen == 128);

#if BRIDGE_READ_MISMATCH_ZERO_FILL
    if (got128instead256) {
      Serial.print(F("[RP2040] ⚠️ READ: esperaba 256, recibió 128. Rellenando por compat. sec="));
      Serial.println(sec);
      memset(g_readBuf + 128, 0, 128);
      g_readLen = 256;
    } else
#endif
    if (got128instead256) {
      Serial.print(F("[READ_LOADER_V4] ❌ READ mismatch persistente: esperaba 256, recibió 128. sec="));
      Serial.println(sec);
      g_readErrors++;
      g_lastErrorSector = sec;
      return false;
    } else if (g_readLen < expectedLen) {
      Serial.print(F("[RP2040] ❌ READ incompleto: len="));
      Serial.print((int)g_readLen);
      Serial.print(F(" esperado="));
      Serial.println(expectedLen);
      g_readErrors++;
      g_lastErrorSector = sec;
      return false;
    } else if (g_readLen > expectedLen) {
      Serial.print(F("[RP2040] ⚠️ READ excedido: len="));
      Serial.print((int)g_readLen);
      Serial.print(F(" esperado="));
      Serial.println(expectedLen);
      g_readLen = expectedLen;
    }
  }

  // ✅ Verificar chunks completos
  uint32_t expectedMask = fullMask32(g_readChunkCount);
  if ((g_readChunkMask32 & expectedMask) != expectedMask) {
    Serial.print(F("[RP2040] ❌ READ: chunks incompletos. Máscara=0x"));
    Serial.print((uint32_t)g_readChunkMask32, HEX);
    Serial.print(F(" esperada=0x"));
    Serial.println(expectedMask, HEX);
    g_readErrors++;
    g_lastErrorSector = sec;
    return false;
  }

  g_totalReads++;
  Serial.print(F("[RP2040] ✅ READ OK sec="));
  Serial.print(sec);
  Serial.print(F(" len="));
  Serial.println((int)g_readLen);

  {
    uint8_t dataChk = calcChecksumSIO(g_readBuf, (int)g_readLen);
    bool atariAccepted = sendAtariDataFrame(g_readBuf, (int)g_readLen, false);
    sendDiskDiagToMaster(atariAccepted ? DISK_DIAG_READ_SENT : DISK_DIAG_READ_NAK,
                         dev, cmd, sec, (uint16_t)g_readLen, dataChk,
                         g_lastAtariPostDataResp, g_lastAtariPostDataFlags);
    if (!atariAccepted) {
      Serial.println(F("[RP2040] ⚠️ Atari respondió NAK a DATA (READ)"));
    }
  }
  return true;
}

static bool doRemoteFormatCommon(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  applyGeometryForDev(dev);
  uint8_t payload[6];
  payload[0] = cmd;
  payload[1] = dev;
  payload[2] = aux1;
  payload[3] = aux2;

  uint8_t base = (cmd & 0x7F);
  bool isDD = g_isDDActive || (base == 0x22) || ((cmd & 0x80) != 0);
  payload[4] = isDD ? 1 : 0;
  payload[5] = 0;

  g_currentOp     = OP_FORMAT;
  g_formatDone    = false;
  g_formatSuccess = false;
  g_formatLen     = 0;

  Serial.print(F("[RP2040] FORMAT "));
  Serial.print(isDD ? F("DD") : F("SD"));
  Serial.print(F(" cmd=0x"));
  if (cmd < 0x10) Serial.print('0');
  Serial.print(cmd, HEX);
  Serial.print(F(" g_isDDActive="));
  Serial.print((bool)g_isDDActive);
  Serial.print(F(" payload[4]="));
  Serial.println(payload[4]);

  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  // Timeout mas largo para DD
  unsigned long t0 = millis();
  unsigned long TIMEOUT_MS = isDD ? 360000UL : 180000UL;
  unsigned long lastProgress = 0;

  while (!g_formatDone && (millis() - t0) < TIMEOUT_MS) {
    delay(0);

    if (millis() - lastProgress > 15000) {
      unsigned long elapsed = (millis() - t0) / 1000;
      Serial.print(F("[RP2040] FORMAT en progreso... "));
      Serial.print(elapsed);
      Serial.println(F("s"));
      lastProgress = millis();
    }
  }
  g_currentOp = OP_NONE;

  if (!g_formatDone || !g_formatSuccess || g_formatLen <= 0) {
    Serial.println(F("[RP2040] FORMAT remoto FALLO."));
    return false;
  }

  mb();

  Serial.print(F("[RP2040] FORMAT OK en "));
  Serial.print((millis() - t0) / 1000);
  Serial.println(F("s"));

  // Enviar respuesta al Atari (COMPLETE + 128 bytes + chk)
  bool ok = sendAtariDataFrame(g_formatBuf, (int)g_formatLen, true);
  if (!ok) Serial.println(F("[RP2040] ⚠️ Atari respondió NAK a DATA (FORMAT)"));

  return true;
}

bool doRemoteFormatSD(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  Serial.println(F("[RP2040] FORMAT SD (base 0x21) solicitado"));
  return doRemoteFormatCommon(dev, cmd, aux1, aux2);
}

bool doRemoteFormatDD(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  Serial.println(F("[RP2040] FORMAT DD/ED (base 0x22) solicitado"));
  Serial.print(F("[RP2040] g_bytesPerSector="));
  Serial.print((uint16_t)g_bytesPerSector);
  Serial.print(F(" g_isDDActive="));
  Serial.println((bool)g_isDDActive);
  return doRemoteFormatCommon(dev, cmd, aux1, aux2);
}

bool doRemotePercomRead(uint8_t dev) {
  uint8_t payload[6];
  payload[0] = 0x4E;
  payload[1] = dev;
  payload[2] = (uint8_t)(PERCOM_SEC_MAGIC & 0xFF);
  payload[3] = (uint8_t)(PERCOM_SEC_MAGIC >> 8);
  payload[4] = 0;
  payload[5] = 0;

  g_currentOp     = OP_PERCOM;
  g_percomDone    = false;
  g_percomSuccess = false;

  Serial.println(F("[RP2040] Enviando CMD_FRAME READ PERCOM a ESP32..."));
  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 8000;
  while (!g_percomDone && (millis() - t0) < TIMEOUT_MS) {
    delay(0);
  }
  g_currentOp = OP_NONE;

  if (!g_percomDone || !g_percomSuccess) {
    Serial.println(F("[RP2040] READ PERCOM remoto FALLÓ."));
    return false;
  }

  mb();

#if BRIDGE_PERCOM_TRANSPARENTE
  Serial.println(F("[RP2040] READ PERCOM remoto OK (modo transparente)."));
#else
  Serial.println(F("[RP2040] READ PERCOM remoto OK, actualizando geometría..."));
  updateGeometryFromPercom(dev, g_percomBuf);
#endif

  sendAtariDataFrame(g_percomBuf, PERCOM_BLOCK_LEN, false);
  return true;
}

bool doRemotePercomWrite(uint8_t dev, const uint8_t *data) {
  uint8_t payload[6];
  payload[0] = 0x4F;
  payload[1] = dev;
  payload[2] = (uint8_t)(PERCOM_SEC_MAGIC & 0xFF);
  payload[3] = (uint8_t)(PERCOM_SEC_MAGIC >> 8);
  payload[4] = 0;
  payload[5] = 0;

  g_currentOp    = OP_WRITE;
  g_writeDone    = false;
  g_writeSuccess = false;
  g_writeSec     = PERCOM_SEC_MAGIC;

  Serial.println(F("[RP2040] Enviando CMD_FRAME WRITE PERCOM a ESP32..."));
  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  sendSectorChunkToMaster(dev, PERCOM_SEC_MAGIC, data, PERCOM_BLOCK_LEN);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 12000;
  while (!g_writeDone && (millis() - t0) < TIMEOUT_MS) {
    delay(0);
  }
  g_currentOp = OP_NONE;

  if (!g_writeDone || !g_writeSuccess) {
    Serial.println(F("[RP2040] WRITE PERCOM remoto FALLÓ."));
    return false;
  }

  Serial.println(F("[RP2040] WRITE PERCOM remoto OK."));
  return true;
}

bool doRemoteWrite(uint8_t dev, uint16_t sec, uint8_t cmd, bool dd, const uint8_t *data, int len) {
  applyGeometryForDev(dev);
  bool geomDD = (g_bytesPerSector >= 256) || g_isDDActive;
  bool isDD   = dd || geomDD || ((cmd & 0x80) != 0);
  uint8_t densFlag = isDD ? 1 : 0;

  uint8_t payload[6];
  payload[0] = cmd;
  payload[1] = dev;
  payload[2] = (uint8_t)(sec & 0xFF);
  payload[3] = (uint8_t)(sec >> 8);
  payload[4] = densFlag;
  payload[5] = 0;

  int expectedLen = len;
  if (expectedLen <= 0 || expectedLen > 256) {
    expectedLen = (int)g_bytesPerSector;
    if (expectedLen <= 0 || expectedLen > 256) expectedLen = 128;
  }

  if (isDD && sec >= 1 && sec <= 3 && expectedLen > 128) expectedLen = 128;
  if (isDD && expectedLen < 256 && !(sec >= 1 && sec <= 3)) expectedLen = 256;
  if (expectedLen > 256) expectedLen = 256;

  g_currentOp    = OP_WRITE;
  g_writeDone    = false;
  g_writeSuccess = false;
  g_writeSec     = sec;

  Serial.print(F("[RP2040] >>> WRITE sec="));
  Serial.print(sec);
  Serial.print(F(" cmd=0x"));
  if (cmd < 0x10) Serial.print('0');
  Serial.print(cmd, HEX);
  Serial.print(F(" dens="));
  Serial.print(isDD ? F("DD") : F("SD"));
  Serial.print(F(" expectedLen="));
  Serial.println(expectedLen);

  uartSendFrame(TYPE_CMD_FRAME, payload, sizeof(payload));
  sendSectorChunkToMaster(dev, sec, data, (uint16_t)expectedLen);

  const unsigned long TIMEOUT_MS = 15000;
  unsigned long t0 = millis();
  while (!g_writeDone && (millis() - t0) < TIMEOUT_MS) {
    delay(0);
  }
  g_currentOp = OP_NONE;

  if (!g_writeDone) {
    Serial.println(F("[RP2040] ❌ WRITE remoto: timeout"));
    g_writeErrors++;
    g_lastErrorSector = sec;
    return false;
  }
  if (!g_writeSuccess) {
    Serial.println(F("[RP2040] ❌ WRITE remoto FALLÓ (NAK)"));
    g_writeErrors++;
    g_lastErrorSector = sec;
    return false;
  }

  g_totalWrites++;
  Serial.println(F("[RP2040] ✅ WRITE remoto OK"));
  return true;
}

// ================== SIO: lectura del frame ==================

static inline bool sioLooksLikeDeviceId(uint8_t b) {
  // V5: solo IDs reales de comando.
  // D1..D7 = 0x31..0x34. P: permitido solo en rango Atari printer 0x40..0x43.
  // Esto evita aceptar basura/ASCII como 0x4F ('O') como si fuera DEV.
  if (b >= SIO_DEV_MIN && b <= SIO_DEV_MAX) return true;
  if (g_printerSioDev >= 0x40 && g_printerSioDev <= 0x43 && b == g_printerSioDev) return true;
  return false;
}

static inline bool sioIsLateHandshakeByte(uint8_t b) {
  return (b == SIO_ACK || b == SIO_NAK || b == SIO_COMPLETE || b == SIO_ERROR);
}

bool readSioCommandFrame(uint8_t buf[5]) {
  // V6: sincronización estricta para loaders/juegos.
  // Solo se permite saltar bytes de handshake atrasados (ACK/NAK/COMPLETE/ERROR).
  // Si el primer byte real no es DEV D1..D7/P:, abortamos ese flanco CMD completo.
  // Esto evita convertir datos residuales/ruido como 0x4F,0x40,0x4F,0x4F en comandos válidos.
  uint8_t b = 0;
  bool skippedLateHandshake = false;
  uint8_t skippedHandshakeCount = 0;

  while (true) {
    if (!readByteWithTimeoutSIO(b, 12)) {
      Serial.println(F("[SIO] Timeout esperando DEV"));
      unsigned long t0 = millis();
      while (SerialSIO.available() > 0 && (millis() - t0) < 2) (void)SerialSIO.read();
      return false;
    }

    if (sioLooksLikeDeviceId(b)) {
      buf[0] = b;
      break;
    }

    if (sioIsLateHandshakeByte(b)) {
      skippedLateHandshake = true;
      skippedHandshakeCount++;
      // Saltamos como máximo unos pocos handshakes atrasados; no hacemos resync con datos arbitrarios.
      if (skippedHandshakeCount <= 4) continue;
    }

    Serial.print(F("[SIO-SYNC] Byte inicial no DEV ignorado=0x"));
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
    if (skippedLateHandshake) Serial.print(F(" después de handshake"));
    Serial.println(F("; abortando flanco CMD"));

    // No responder al Atari en frames fuera de rango: si fue ruido, no hacemos nada;
    // si fue un comando real corrupto, el host lo reintentará.
    unsigned long t0 = millis();
    while (digitalRead(PIN_CMD) == LOW && (millis() - t0) < 20) {
      while (SerialSIO.available() > 0) (void)SerialSIO.read();
      delay(0);
    }
    return false;
  }

  for (int i = 1; i < 5; i++) {
    if (!readByteWithTimeoutSIO(buf[i], 12)) {
      Serial.print(F("[SIO] Timeout leyendo byte "));
      Serial.println(i);

      unsigned long t0 = millis();
      while (SerialSIO.available() > 0 && (millis() - t0) < 2) (void)SerialSIO.read();
      return false;
    }
  }

  uint8_t calc = calcChecksumSIO(buf, 4);
  if (calc != buf[4]) {
    Serial.print(F("[SIO] Checksum CMD inválido: recv=0x"));
    if (buf[4] < 0x10) Serial.print('0');
    Serial.print(buf[4], HEX);
    Serial.print(F(" calc=0x"));
    if (calc < 0x10) Serial.print('0');
    Serial.println(calc, HEX);
    dumpCommandFrame(buf);
    return false;
  }

  dumpCommandFrame(buf);
  return true;
}

static inline void sioPushbackByte(uint8_t b) {
  g_sioPushbackByte = b;
  g_sioPushbackValid = true;
}

// ================== Printer 820 / Atari P: ==================

bool enqueuePrinterLineForMaster(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2, const uint8_t* data, uint8_t len) {
  if (len > 40) len = 40;

  uint8_t head = g_prnQHead;
  uint8_t next = (uint8_t)((head + 1) % PRN_QUEUE_SIZE);

  if (next == g_prnQTail) {
    g_prnQDrop++;
    return false;
  }

  PrinterQueueItem &it = g_prnQ[head];
  it.dev = dev;
  it.cmd = cmd;
  it.aux1 = aux1;
  it.aux2 = aux2;
  it.len = len;

  if (data && len > 0) memcpy(it.data, data, len);

  g_prnQHead = next;
  return true;
}

void servicePrinterQueueToMaster() {
  if (g_prnQTail == g_prnQHead) return;

  uint32_t now = millis();
  if ((uint32_t)(now - g_prnLastUartLineMs) < PRN_UART_LINE_GAP_MS) {
    return;
  }

  uint8_t tail = g_prnQTail;
  PrinterQueueItem it = g_prnQ[tail];
  g_prnQTail = (uint8_t)((tail + 1) % PRN_QUEUE_SIZE);

  sendPrinterLineToMaster(it.dev, it.cmd, it.aux1, it.aux2, it.data, it.len);
  g_prnLastUartLineMs = millis();
  g_prnQSent++;
}

void sendPrinterLineToMaster(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2, const uint8_t* data, uint8_t len) {
  uint8_t payload[5 + 40];

  if (len > 40) len = 40;

  payload[0] = dev;
  payload[1] = cmd;
  payload[2] = aux1;
  payload[3] = aux2;
  payload[4] = len;

  if (data && len > 0) memcpy(&payload[5], data, len);

  uartSendFrame(TYPE_PRINTER_LINE, payload, (uint8_t)(5 + len));
}

bool handlePrinter820Command(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  uint8_t base = cmd & 0x7F;

  // STATUS $53
  if (base == PRN_CMD_STATUS) {
    g_prnStatusCount++;
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();

    // Atari 820 status simple: listo/sin error.
    // Usamos tiempos propios de P: para no depender del perfil de discos.
    uint8_t st[4] = { 0x00, 0x00, 0x00, 0x00 };

    delayMicroseconds(PRN_STATUS_ACK_TO_COMPLETE_US);
    SerialSIO.write(SIO_COMPLETE);
    SerialSIO.flush();

    delayMicroseconds(PRN_STATUS_COMPLETE_TO_DATA_US);
    SerialSIO.write(st, 4);
    SerialSIO.flush();

    delayMicroseconds(PRN_STATUS_DATA_TO_CHK_US);
    SerialSIO.write(calcChecksumSIO(st, 4));
    SerialSIO.flush();

    return true;
  }

  // WRITE $57: Print Line. AUX1=$4E normal (40 chars), AUX1=$53 sideways (29 chars).
  if (base == PRN_CMD_WRITE) {
    g_prnWriteCount++;
    uint8_t lineLen = (aux1 == PRN_AUX_SIDEWAYS) ? 29 : 40;
    uint8_t line[40];

    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();

    // Sin Serial.print() aquí: el Atari puede enviar DATA muy rápido después del ACK.
    // Dejamos margen para turnaround del Atari, los bytes se acumulan en UART.
    delayMicroseconds(PRN_CMD_ACK_TO_DATA_US);

    for (int i = 0; i < lineLen; i++) {
      if (!readByteWithTimeoutSIO(line[i], 120)) {
        g_prnTimeoutData++;
        Serial.print(F("[PRN] Timeout leyendo byte "));
        Serial.println(i);
        SerialSIO.write(SIO_ERROR);
        SerialSIO.flush();
        return false;
      }
    }

    uint8_t chkRecv;
    if (!readByteWithTimeoutSIO(chkRecv, 120)) {
      g_prnTimeoutChk++;
      Serial.println(F("[PRN] Timeout leyendo checksum"));
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      return false;
    }

    uint8_t chkCalc = calcChecksumSIO(line, lineLen);
    if (chkRecv != chkCalc) {
      g_prnChecksumErr++;
      Serial.print(F("[PRN] Checksum inválido P: recv=0x"));
      if (chkRecv < 0x10) Serial.print('0');
      Serial.print(chkRecv, HEX);
      Serial.print(F(" calc=0x"));
      if (chkCalc < 0x10) Serial.print('0');
      Serial.println(chkCalc, HEX);

      // v29:
      // Si el byte leído como checksum parece ser un DEV SIO (0x31..0x43),
      // probablemente ya consumimos el primer byte del siguiente comando.
      // Lo devolvemos para que readSioCommandFrame() pueda leer ese frame bien.
      if (sioLooksLikeDeviceId(chkRecv)) {
        sioPushbackByte(chkRecv);
        Serial.print(F("[PRN] Pushback SIO byte=0x"));
        if (chkRecv < 0x10) Serial.print('0');
        Serial.println(chkRecv, HEX);
      }

      // Heurística de recuperación:
      // Si el último byte leído como dato coincide con el checksum de los primeros
      // lineLen-1 bytes, el chkRecv pertenece al próximo comando. Aceptamos lineLen-1.
      uint8_t chkMinusOne = calcChecksumSIO(line, (uint8_t)(lineLen - 1));
      if (lineLen > 1 && line[lineLen - 1] == chkMinusOne) {
        g_prnWriteOk++;

        delayMicroseconds(PRN_CHK_TO_DATA_ACK_US);
        SerialSIO.write(SIO_ACK);
        SerialSIO.flush();

        delayMicroseconds(PRN_DATA_ACK_TO_COMPLETE_US);
        SerialSIO.write(SIO_COMPLETE);
        SerialSIO.flush();

        delayMicroseconds(300);
        enqueuePrinterLineForMaster(dev, cmd, aux1, aux2, line, (uint8_t)(lineLen - 1));

        Serial.println(F("[PRN] Línea P: recuperada con len-1"));
        return true;
      }

#if PRN_TOLERANT_CHECKSUM
      // Modo tolerante solo para impresora P:
      // NO imprimimos esta línea corrupta, pero respondemos OK para que BASIC siga.
      g_prnWriteOk++;

      delayMicroseconds(PRN_CHK_TO_DATA_ACK_US);

      SerialSIO.write(SIO_ACK);
      SerialSIO.flush();

      delayMicroseconds(PRN_DATA_ACK_TO_COMPLETE_US);
      SerialSIO.write(SIO_COMPLETE);
      SerialSIO.flush();

      return true;
#else
      SerialSIO.write(SIO_NAK);
      SerialSIO.flush();
      return false;
#endif
    }

    g_prnWriteOk++;

    // v25: margen checksum -> ACK para que Atari alcance a cambiar TX->RX.
    delayMicroseconds(PRN_CHK_TO_DATA_ACK_US);

    // ACK del data frame.
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();

    // COMPLETE primero: el Atari no debe esperar por UART/Wi-Fi/HTTP.
    // v20: usar margen propio de impresora para que el Atari alcance a cambiar TX->RX.
    delayMicroseconds(PRN_DATA_ACK_TO_COMPLETE_US);
    SerialSIO.write(SIO_COMPLETE);
    SerialSIO.flush();

    // Pequeño margen para que el host termine la transacción SIO antes de actividad interna.
    delayMicroseconds(300);

    // Encolar después del COMPLETE. Core1 enviará al MASTER.
    enqueuePrinterLineForMaster(dev, cmd, aux1, aux2, line, lineLen);
    return true;
  }

  g_prnUnsupportedCmd++;
  Serial.print(F("[PRN] CMD no soportado base=0x"));
  if (base < 0x10) Serial.print('0');
  Serial.println(base, HEX);

  SerialSIO.write(SIO_NAK);
  SerialSIO.flush();
  return false;
}

// ================== Manejo del comando SIO ==================

void handleSioCommand() {
  if (!readSioCommandFrame(cmdBuf)) return;

  uint8_t dev  = cmdBuf[0];
  uint8_t cmd  = cmdBuf[1];
  uint8_t aux1 = cmdBuf[2];
  uint8_t aux2 = cmdBuf[3];

  // Printer 820 / Atari P: vive fuera del rango D1..D7, por eso se captura antes
  // del filtro de disqueteras. Por defecto usa DEV 0x40.
  if (dev == g_printerSioDev) {
    // v18:
    // El handler P: de Atari hace STATUS durante OPEN. Si no respondemos,
    // BASIC termina con error 138 en la línea del OPEN.
    // Por seguridad, DEV=0x40 siempre se atiende como impresora 820.
    // La opción web "Habilitar P:" queda para controlar el envío real en el MASTER,
    // no para romper el protocolo SIO.
    if (!g_printerEnabled) {
      Serial.println(F("[PRN] P: cfg disabled, pero respondiendo SIO para evitar ERROR 138"));
    }
    handlePrinter820Command(dev, cmd, aux1, aux2);
    return;
  }

  if (dev < SIO_DEV_MIN || dev > SIO_DEV_MAX) {
    Serial.print(F("[RP2040] DEV fuera de rango D1..D7: 0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.println(dev, HEX);
    return;
  }

  uint8_t base = cmd & 0x7F;
  uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

  // STATUS
  if (base == 0x53) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.println(F("[SIO] ACK enviado (STATUS)"));
    delayMicroseconds(800);
    if (!doRemoteStatus(dev, aux1, aux2)) {
      sioSendError(dev, cmd, sec, F("remote STATUS fail"));
    }
    return;
  }

  // READ PERCOM
  if (base == 0x4E) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.println(F("[SIO] ACK enviado (READ PERCOM)"));
    delayMicroseconds(800);
    if (!doRemotePercomRead(dev)) {
      sioSendError(dev, cmd, sec, F("remote READ PERCOM fail"));
    }
    return;
  }

  // WRITE PERCOM
  if (base == 0x4F) {
    uint8_t percom[PERCOM_BLOCK_LEN];

    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.println(F("[SIO] ACK enviado (WRITE PERCOM)"));

    delayMicroseconds(800);

    for (int i = 0; i < PERCOM_BLOCK_LEN; i++) {
      if (!readByteWithTimeoutSIO(percom[i], 100)) {
        Serial.print(F("[SIO] Timeout leyendo DATA de WRITE PERCOM en byte "));
        Serial.println(i);
        sioSendError(dev, cmd, sec, F("timeout DATA WRITE PERCOM"));
        return;
      }
    }

    uint8_t chkRecv;
    if (!readByteWithTimeoutSIO(chkRecv, 100)) {
      Serial.println(F("[SIO] Timeout leyendo CHK de WRITE PERCOM"));
      sioSendError(dev, cmd, sec, F("timeout CHK WRITE PERCOM"));
      return;
    }

    uint8_t chkCalc = calcChecksumSIO(percom, PERCOM_BLOCK_LEN);
    if (chkRecv != chkCalc) {
      Serial.println(F("[SIO] Checksum WRITE PERCOM inválido, enviando ERROR"));
      sioSendError(dev, cmd, sec, F("checksum WRITE PERCOM"));
      return;
    }

    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.println(F("[SIO] ACK enviado (DATA WRITE PERCOM OK)"));

    updateGeometryFromPercom(dev, percom);

    if (doRemotePercomWrite(dev, percom)) {
      delayMicroseconds(T_ACK_TO_COMPLETE);
      SerialSIO.write(SIO_COMPLETE);
      SerialSIO.flush();
      Serial.println(F("[SIO] COMPLETE enviado (WRITE PERCOM OK)"));
      delayMicroseconds(T_CHUNK_DELAY);
    } else {
      Serial.println(F("[SIO] WRITE PERCOM remoto FALLÓ, enviando ERROR"));
      sioSendError(dev, cmd, sec, F("remote WRITE PERCOM fail"));
    }
    return;
  }

  // READ SECTOR
  if (base == 0x52) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.print(F("[SIO] ACK enviado (READ) dev=0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.print(dev, HEX);
    Serial.print(F(" sec="));
    Serial.println(sec);
    delayMicroseconds(650);

    if (!doRemoteRead(dev, sec, cmd)) {
      Serial.println(F("[SIO] READ remoto FALLÓ, enviando ERROR al Atari"));
      sioSendError(dev, cmd, sec, F("remote READ fail"));
    }
    return;
  }

  // FORMAT SD
  if (base == 0x21) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.print(F("[SIO] ACK enviado (FORMAT SD cmd=0x"));
    if (cmd < 0x10) Serial.print('0');
    Serial.print(cmd, HEX);
    Serial.println(F(")"));
    delayMicroseconds(800);

    if (!doRemoteFormatSD(dev, cmd, aux1, aux2)) {
      sioSendError(dev, cmd, sec, F("remote FORMAT fail"));
    }
    return;
  }

  // FORMAT DD
  if (base == 0x22) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.print(F("[SIO] ACK enviado (FORMAT DD cmd=0x"));
    if (cmd < 0x10) Serial.print('0');
    Serial.print(cmd, HEX);
    Serial.println(F(")"));
    delayMicroseconds(800);

    if (!doRemoteFormatDD(dev, cmd, aux1, aux2)) {
      sioSendError(dev, cmd, sec, F("remote FORMAT fail"));
    }
    return;
  }

  // WRITE SECTOR (0x50 / 0x57)
  if (base == 0x50 || base == 0x57) {
    int expectedLen = getLocalSectorSizeForWrite(sec, cmd);
    uint8_t dataBuf[256];

    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();

    Serial.print(F("[SIO] ACK enviado (WRITE cmd=0x"));
    if (cmd < 0x10) Serial.print('0');
    Serial.print(cmd, HEX);
    Serial.print(F(") dev=0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.print(dev, HEX);
    Serial.print(F(" sec="));
    Serial.print(sec);
    Serial.print(F(" expectedLen="));
    Serial.println(expectedLen);

    delayMicroseconds(650);

    for (int i = 0; i < expectedLen; i++) {
      if (!readByteWithTimeoutSIO(dataBuf[i], 120)) {
        Serial.print(F("[SIO] Timeout leyendo DATA de WRITE en byte "));
        Serial.println(i);
        sioSendError(dev, cmd, sec, F("timeout DATA WRITE"));
        return;
      }
    }

    uint8_t chkRecv;
    if (!readByteWithTimeoutSIO(chkRecv, 120)) {
      Serial.println(F("[SIO] Timeout leyendo CHK de WRITE"));
      sioSendError(dev, cmd, sec, F("timeout CHK WRITE"));
      return;
    }

    uint8_t chkCalc = calcChecksumSIO(dataBuf, expectedLen);
    if (chkRecv != chkCalc) {
      Serial.println(F("[SIO] Checksum WRITE inválido -> NAK para que Atari reintente"));
      SerialSIO.write(SIO_NAK);
      SerialSIO.flush();
      return;
    }

    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.println(F("[SIO] ACK enviado (DATA WRITE OK)"));

    SerialSIO.write(SIO_COMPLETE);
    SerialSIO.flush();
    Serial.println(F("[SIO] COMPLETE enviado"));

    delayMicroseconds(500);

    bool ddReq = isCmdDD(cmd);

    if (doRemoteWrite(dev, sec, cmd, ddReq, dataBuf, expectedLen)) {
      delayMicroseconds(T_ACK_TO_COMPLETE);
      SerialSIO.write(SIO_COMPLETE);
      SerialSIO.flush();
      Serial.println(F("[SIO] COMPLETE enviado (WRITE OK)"));
      delayMicroseconds(T_CHUNK_DELAY);
    } else {
      Serial.println(F("[SIO] WRITE remoto FALLÓ, enviando ERROR"));
      sioSendError(dev, cmd, sec, F("remote WRITE fail"));
    }
    return;
  }

  // SuperDOS/XF551: sondas de capacidad/diagnóstico. Mejor responder ACK+ERROR que NAK.
  if (base == 0x48 || base == 0x3F) {
    delayMicroseconds(600);
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    delayMicroseconds(400);
    SerialSIO.write(SIO_ERROR);
    SerialSIO.flush();

    uint8_t hint[2] = { dev, base };
    uartSendFrame(TYPE_SUPERDOS_HINT, hint, sizeof(hint));

    Serial.print(F("[SIO] Probe cmd 0x"));
    if (base < 0x10) Serial.print('0');
    Serial.print(base, HEX);
    Serial.println(F(" -> ACK + ERROR (+HINT)"));
    return;
  }

  // Otros comandos – NAK
  delayMicroseconds(800);
  SerialSIO.write(SIO_NAK);
  SerialSIO.flush();
  Serial.print(F("[SIO] CMD no soportado base=0x"));
  if (base < 0x10) Serial.print('0');
  Serial.println(base, HEX);
}

void sioSendError(uint8_t dev, uint8_t cmd, uint16_t sec, const __FlashStringHelper* motivo) {
  Serial.print(F("[SIO] ERROR → Atari dev=0x"));
  if (dev < 0x10) Serial.print('0');
  Serial.print(dev, HEX);
  Serial.print(F(" cmd=0x"));
  if (cmd < 0x10) Serial.print('0');
  Serial.print(cmd, HEX);
  Serial.print(F(" sec="));
  Serial.print(sec);
  Serial.print(F("  Motivo: "));
  Serial.println(motivo);

  SerialSIO.write(SIO_ERROR);
  SerialSIO.flush();
}

static int devIndex(uint8_t dev) {
  if (dev < SIO_DEV_MIN || dev > SIO_DEV_MAX) return -1;
  return (int)(dev - SIO_DEV_MIN);
}

static void saveGeometryForDev(uint8_t dev, uint16_t bytesPerSector, bool isDD) {
  int idx = devIndex(dev);
  if (idx < 0) return;
  if (bytesPerSector < 128 || bytesPerSector > 256) bytesPerSector = 128;
  g_driveGeom[idx].bytesPerSector = bytesPerSector;
  g_driveGeom[idx].isDDActive = isDD || (bytesPerSector > 128);
  g_driveGeom[idx].valid = true;
  g_driveGeom[idx].lastPercomMs = millis();
}

static void applyGeometryForDev(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx < 0 || !g_driveGeom[idx].valid) {
    g_bytesPerSector = 128;
    g_isDDActive = false;
    return;
  }
  g_bytesPerSector = g_driveGeom[idx].bytesPerSector;
  g_isDDActive = g_driveGeom[idx].isDDActive;
}

static uint16_t bytesPerSectorForDev(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx >= 0 && g_driveGeom[idx].valid) return g_driveGeom[idx].bytesPerSector;
  return g_bytesPerSector ? g_bytesPerSector : 128;
}

static bool isDDActiveForDev(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx >= 0 && g_driveGeom[idx].valid) return g_driveGeom[idx].isDDActive;
  return g_isDDActive;
}

void initGeometry() {
  for (int i = 0; i < 7; i++) {
    g_driveGeom[i].bytesPerSector = 128;
    g_driveGeom[i].isDDActive = false;
    g_driveGeom[i].valid = false;
    g_driveGeom[i].lastPercomMs = 0;
  }
#if BRIDGE_FIXED_SD_GEOMETRY
  g_bytesPerSector = 128;
  g_isDDActive     = false;
#endif
}

int getLocalSectorSizeForWrite(uint16_t sec, uint8_t cmd) {
  if (sec >= 1 && sec <= 3) return 128;
  if (g_bytesPerSector == 256) return 256;
  return 128;
}

void updateGeometryFromPercom(uint8_t dev, const uint8_t percom[PERCOM_BLOCK_LEN]) {
  uint8_t  tracks          = percom[0];
  uint16_t sectorsPerTrack = ((uint16_t)percom[2] << 8) | (uint16_t)percom[3];
  uint8_t  sides           = (percom[4] & 0x01) ? 2 : 1;
  uint16_t bytesPerSector  = ((uint16_t)percom[6] << 8) | (uint16_t)percom[7];

  if (tracks == 0 || sectorsPerTrack == 0 || bytesPerSector == 0) {
    Serial.println(F("[SIO] PERCOM inválido, usando SD 128b."));
    tracks          = 40;
    sectorsPerTrack = 18;
    sides           = 1;
    bytesPerSector  = 128;
  }

  if (bytesPerSector > 256) bytesPerSector = 256;
  if (bytesPerSector < 128) bytesPerSector = 128;

  Serial.print(F("[SIO] PERCOM decodificado D"));
  Serial.print((dev >= SIO_DEV_MIN && dev <= SIO_DEV_MAX) ? (int)(dev - 0x30) : 0);
  Serial.print(F(": tracks="));
  Serial.print(tracks);
  Serial.print(F(" spt="));
  Serial.print(sectorsPerTrack);
  Serial.print(F(" sides="));
  Serial.print(sides);
  Serial.print(F(" bytes/sector="));
  Serial.println(bytesPerSector);

#if BRIDGE_PERCOM_TRANSPARENTE
  Serial.println(F("[SIO] BRIDGE_PERCOM_TRANSPARENTE=1 -> NO se cambia geometría local."));
  return;
#endif

  bool isDD = (bytesPerSector > 128);
  saveGeometryForDev(dev, bytesPerSector, isDD);
  applyGeometryForDev(dev);

  Serial.print(F("[SIO] Geometría actualizada D"));
  Serial.print((dev >= SIO_DEV_MIN && dev <= SIO_DEV_MAX) ? (int)(dev - 0x30) : 0);
  Serial.print(F(": bytes/sector="));
  Serial.print((uint16_t)g_bytesPerSector);
  Serial.print(F(" DD="));
  Serial.println((bool)g_isDDActive ? 1 : 0);
}


// ================== SETUP / LOOP (CORE0) ==================

void setup() {
  initGeometry();
  uartEspTxLockInit();

  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  Serial.begin(115200);
  //while (!Serial) { delay(10); }

  Serial.println(F("\n=== RP2040 SIO Bridge v2.2 - DUAL CORE + WEB CFG ==="));
  Serial.println(F("Core0=SIO, Core1=UART RX"));

  // Default SIO
  g_sioBaud = 19200;
  SerialSIO.begin((uint32_t)g_sioBaud);
  gpio_set_function(PIN_SIO_TX, GPIO_FUNC_UART);
  gpio_set_function(PIN_SIO_RX, GPIO_FUNC_UART);

  pinMode(PIN_CMD, INPUT_PULLUP);

  // Default UART a Master
  g_uartBaud = UART_ESP_BAUD;
  uart_init(UART_ESP, (uint32_t)g_uartBaud);
  gpio_set_function(PIN_ESP_TX, GPIO_FUNC_UART);
  gpio_set_function(PIN_ESP_RX, GPIO_FUNC_UART);

  lastCmdState = digitalRead(PIN_CMD);
  digitalWrite(LED_STATUS, HIGH);

  g_core0Ready = true; // habilita core1
  Serial.println(F("✅ RP2040 listo. Esperando comandos SIO..."));
  Serial.print(F("[BUILD] RP="));
  Serial.println(RP_BUILD);
}

void loop() {
  static unsigned long lastStats = 0;

  serviceCassettePlayback();
  if (g_casPlaying) {
    lastCmdState = digitalRead(PIN_CMD);
    delay(0);
    return;
  }

  int cmdState = digitalRead(PIN_CMD);
  if (lastCmdState == HIGH && cmdState == LOW) {
    // V5: debounce del COMMAND. Un falso flanco puede hacer que bytes de ruido
    // parezcan frames válidos como 0x4F/0x40. El Atari mantiene COMMAND bajo
    // durante todo el frame, por lo que 80us es seguro a 19200 baudios.
    delayMicroseconds(SIO_CMD_FALL_DEBOUNCE_US);
    if (digitalRead(PIN_CMD) == LOW) {
      Serial.println(F("[SIO] CMD ↓ detectado, leyendo frame de comando..."));
      handleSioCommand();
      // V6: no rearmar detector hasta que COMMAND vuelva a alto.
      unsigned long rel0 = millis();
      while (digitalRead(PIN_CMD) == LOW && (millis() - rel0) < 25) delayMicroseconds(20);
    } else {
      Serial.println(F("[SIO] Falso flanco CMD ignorado"));
    }
  }
  lastCmdState = cmdState;

  // Si CMD está alto, drenamos (evita basura acumulada)
  if (cmdState == HIGH) {
    while (SerialSIO.available() > 0) (void)SerialSIO.read();
  }

  // aplica CFG cuando toca (después de mandar CFG_ACK)
  if (g_cfgPending && (int32_t)(millis() - g_applyCfgAtMs) >= 0) {
    if (g_newSioBaud && g_newSioBaud != g_sioBaud) {
      SerialSIO.begin((uint32_t)g_newSioBaud);
      g_sioBaud = g_newSioBaud;
      Serial.print(F("[RP2040] ✅ SIO Atari baud="));
      Serial.println((uint32_t)g_sioBaud);
    }

    if (g_newUartBaud && g_newUartBaud != g_uartBaud) {
      uart_set_baudrate(UART_ESP, (uint32_t)g_newUartBaud);
      g_uartBaud = g_newUartBaud;
      Serial.print(F("[RP2040] ✅ UART MASTER baud="));
      Serial.println((uint32_t)g_uartBaud);
    }

    g_cfgPending = false;
  }

  // Estadísticas cada 60s
  if (millis() - lastStats > 60000) {
    printStats();
    lastStats = millis();
  }

  delay(1);
}

// ================== CORE1: UART RX TASK ==================
// En el core1 solo hacemos RX/parsing UART para no bloquear SIO.
void setup1() {
  // esperar a que core0 inicialice UART/Serial
  while (!g_core0Ready) { delay(1); }
  delay(10);
}

void loop1() {
  serviceUartFromMaster();
  if (!(g_casPlaying || g_casUartActive || casRecordBusy())) {
    servicePrinterQueueToMaster();
  }
  servicePrinterDiag();
  sendCasStatusToMaster(false);
  // no delays largos aquí (460800)
  delay(0);
}