// ⚠️ ESTE ARCHIVO REEMPLAZA COMPLETAMENTE TU rp2040_sio_bridge.ino
// Versión DUAL-CORE:
// - Core0: SIO Atari (CMD/frames) + ejecutar operaciones
// - Core1: RX UART desde MASTER (ESP32) + onMasterFrame()
// Mantiene funcionalidades existentes + CFG_UPDATE + validación READ
// Fix importante: FORMAT vuelve a TX por SerialSIO (evita que el Atari “quede esperando” en boards donde Serial1 != uart0)

#include <Arduino.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>

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

// Códigos SIO
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

#define SIO_DEV_MIN 0x31
#define SIO_DEV_MAX 0x34

#define PERCOM_BLOCK_LEN 12
#define PERCOM_SEC_MAGIC 0xFFFF

#define BRIDGE_PERCOM_TRANSPARENTE  0
#define BRIDGE_FIXED_SD_GEOMETRY    0

static const uint16_t CHUNK_PAYLOAD = 240;

// Timings SIO - OPTIMIZADOS
uint16_t T_ACK_TO_COMPLETE   = 120;
uint16_t T_COMPLETE_TO_DATA  = 80;
uint16_t T_DATA_TO_CHK       = 15;
uint16_t T_CHUNK_DELAY       = 20;

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
static uint8_t           g_formatBuf[256];
static volatile int      g_formatLen       = 0;
static volatile bool     g_formatStarted     = false;
static volatile uint8_t  g_formatChunkCount  = 0;
static volatile uint32_t g_formatChunkMask32 = 0;
static volatile uint32_t g_formatLastChunkMs = 0;

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

static volatile uint32_t g_uartBaud = UART_ESP_BAUD;
static volatile uint32_t g_sioBaud  = 19200;

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
void handleSioCommand();
void sioSendError(uint8_t dev, uint8_t cmd, uint16_t sec, const __FlashStringHelper* motivo);

void initGeometry();
int getLocalSectorSizeForWrite(uint16_t sec, uint8_t cmd);
void updateGeometryFromPercom(const uint8_t percom[PERCOM_BLOCK_LEN]);

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

  uartSendByte(0x55);
  uartSendByte(len);
  uartSendByte(type);
  if (payloadLen > 0) uart_write_blocking(UART_ESP, payload, payloadLen);
  uartSendByte(sum);
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

      T_ACK_TO_COMPLETE  = clamp16(ack2comp,   60, 5000);
      T_COMPLETE_TO_DATA = clamp16(comp2data,  40, 5000);
      T_DATA_TO_CHK      = clamp16(data2chk,   10, 2000);
      T_CHUNK_DELAY      = clamp16(chDelay,     0, 8000);

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
        if (!g_formatStarted) {
          g_formatStarted     = true;
          g_formatChunkCount  = (count == 0) ? 1 : count;
          g_formatChunkMask32 = 0;
          g_formatLen         = 0;
          g_formatLastChunkMs = millis();
          memset(g_formatBuf, 0, sizeof(g_formatBuf));

          Serial.print(F("[RP2040] FORMAT iniciado esperando "));
          Serial.print((int)g_formatChunkCount);
          Serial.println(F(" chunks"));
        }

        int off = (int)idx * (int)CHUNK_PAYLOAD;
        if (off < 0) off = 0;
        if (off >= (int)sizeof(g_formatBuf)) off = (int)sizeof(g_formatBuf);

        int room = (int)sizeof(g_formatBuf) - off;
        if (dlen > room) dlen = room;
        if (dlen > 0) memcpy(g_formatBuf + off, payload, dlen);

        int endPos = off + dlen;
        if (endPos > g_formatLen) g_formatLen = endPos;

        if (idx < 32) {
          uint32_t bit = (1UL << idx);
          g_formatChunkMask32 |= bit;
        }

        g_formatLastChunkMs = millis();

        uint32_t need = fullMask32(g_formatChunkCount);
        if (need != 0 && ((g_formatChunkMask32 & need) == need)) {
          g_formatDone    = true;
          g_formatSuccess = (g_formatLen > 0);
          mb();
          g_currentOp     = OP_NONE;

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
  // Drenar basura previa (importante después de operaciones largas)
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

  // (Opcional) esperar ACK/NAK del Atari (si llega).
  // No bloqueamos mucho para no perder el siguiente comando.
  uint8_t resp;
  unsigned long t0 = millis();
  while (millis() - t0 < 20) {
    if (SerialSIO.available()) {
      resp = (uint8_t)SerialSIO.read();
      if (resp == SIO_ACK) return true;
      if (resp == SIO_NAK) return false;
      // cualquier otro byte: drenar y salir
      drainSioRx(1);
      return true;
    }
    delay(0);
  }

  // si no llegó nada, asumimos OK (muchos DOS no envían ACK aquí siempre)
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

// ✅✅✅ doRemoteRead con validación estricta ✅✅✅
bool doRemoteRead(uint8_t dev, uint16_t sec, uint8_t cmd) {
  bool reqDD = isCmdDD(cmd) || g_isDDActive;

  int expectedLen = 128;
  if (reqDD && !(sec >= 1 && sec <= 3)) {
    expectedLen = (g_bytesPerSector > 128) ? (int)g_bytesPerSector : 256;
    if (expectedLen > 256) expectedLen = 256;
  }
  if (expectedLen < 128) expectedLen = 128;
  g_readExpectedLen = expectedLen;

  uint8_t payload[6];
  payload[0] = 0x52;
  payload[1] = dev;
  payload[2] = (uint8_t)(sec & 0xFF);
  payload[3] = (uint8_t)(sec >> 8);
  payload[4] = (reqDD ? 1 : 0);
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
  Serial.print(reqDD ? F("DD") : F("SD"));
  Serial.print(F(" esperando "));
  Serial.print(expectedLen);
  Serial.println(F(" bytes"));

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
    g_readErrors++;
    g_lastErrorSector = sec;
    return false;
  }

  mb();

  // ✅ VALIDACIÓN ESTRICTA
  if (g_readLen != expectedLen) {
    bool got128instead256 = (expectedLen == 256 && g_readLen == 128);

    if (got128instead256) {
      Serial.print(F("[RP2040] ⚠️ READ: esperaba 256, recibió 128. Rellenando. sec="));
      Serial.println(sec);
      memset(g_readBuf + 128, 0, 128);
      g_readLen = 256;
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

  if (!sendAtariDataFrame(g_readBuf, (int)g_readLen, false)) {
    Serial.println(F("[RP2040] ⚠️ Atari respondió NAK a DATA (READ)"));
  }
  return true;
}

static bool doRemoteFormatCommon(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  uint8_t payload[6];
  payload[0] = cmd;
  payload[1] = dev;
  payload[2] = aux1;
  payload[3] = aux2;

  uint8_t base = (cmd & 0x7F);
  bool isDD = (g_bytesPerSector > 128) || (base == 0x22) || ((cmd & 0x80) != 0);
  payload[4] = isDD ? 1 : 0;
  payload[5] = 0;

  g_currentOp     = OP_FORMAT;
  g_formatDone    = false;
  g_formatSuccess = false;
  g_formatLen     = 0;
  g_formatStarted     = false;
  g_formatChunkCount  = 0;
  g_formatChunkMask32 = 0;
  g_formatLastChunkMs = millis();
  memset(g_formatBuf, 0, sizeof(g_formatBuf));

  Serial.print(F("[RP2040] FORMAT cmd=0x"));
  if (cmd < 0x10) Serial.print('0');
  Serial.print(cmd, HEX);
  Serial.print(F(" base=0x"));
  if (base < 0x10) Serial.print('0');
  Serial.print(base, HEX);
  Serial.print(F(" bytesPerSector="));
  Serial.print((uint16_t)g_bytesPerSector);
  Serial.print(F(" payload[4]="));
  Serial.println(payload[4]);

  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  // Timeout mas largo para DD
  unsigned long t0 = millis();
  unsigned long TIMEOUT_MS = isDD ? 360000UL : 180000UL;
  unsigned long lastProgress = 0;

  while (!g_formatDone && (millis() - t0) < TIMEOUT_MS) {
    delay(0);

    if (g_formatStarted && (millis() - g_formatLastChunkMs) > 5000) {
      Serial.println(F("[RP2040] FORMAT: timeout entre chunks"));
      break;
    }

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
  Serial.println(F("[RP2040] FORMAT solicitado (wrapper SD/base 0x21)"));
  return doRemoteFormatCommon(dev, cmd, aux1, aux2);
}

bool doRemoteFormatDD(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  Serial.println(F("[RP2040] FORMAT solicitado (wrapper DD/ED/base 0x22)"));
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
  updateGeometryFromPercom(g_percomBuf);
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

bool readSioCommandFrame(uint8_t buf[5]) {
  for (int i = 0; i < 5; i++) {
    if (!readByteWithTimeoutSIO(buf[i], 10)) { // 10ms por byte (más robusto)
      Serial.print(F("[SIO] Timeout leyendo byte "));
      Serial.println(i);

      unsigned long t0 = millis();
      while (SerialSIO.available() > 0 && (millis() - t0) < 2) (void)SerialSIO.read();
      return false;
    }
  }
  dumpCommandFrame(buf);
  return true;
}

// ================== Manejo del comando SIO ==================

void handleSioCommand() {
  if (!readSioCommandFrame(cmdBuf)) return;

  uint8_t dev  = cmdBuf[0];
  uint8_t cmd  = cmdBuf[1];
  uint8_t aux1 = cmdBuf[2];
  uint8_t aux2 = cmdBuf[3];

  if (dev < SIO_DEV_MIN || dev > SIO_DEV_MAX) {
    Serial.print(F("[RP2040] DEV fuera de rango D1..D4: 0x"));
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

    updateGeometryFromPercom(percom);

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

void initGeometry() {
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

void updateGeometryFromPercom(const uint8_t percom[PERCOM_BLOCK_LEN]) {
  uint8_t  tracks        = percom[0];
  uint16_t bytesPerTrack = (uint16_t)percom[2] | ((uint16_t)percom[3] << 8);
  uint8_t  sideCode      = percom[4];
  uint8_t  sides         = (sideCode & 0x01) ? 2 : 1;
  uint8_t  bpsCode       = percom[6] & 0x03;

  uint16_t bytesPerSector;
  switch (bpsCode) {
    case 0: bytesPerSector = 128; break;
    case 1: bytesPerSector = 256; break;
    case 2: bytesPerSector = 512; break;
    default:
      Serial.print(F("[SIO] PERCOM bpsCode desconocido="));
      Serial.print(bpsCode);
      Serial.println(F(" -> forzando 128 bytes/sector"));
      bytesPerSector = 128;
      break;
  }

  if (bytesPerTrack == 0 || bytesPerSector == 0) {
    Serial.println(F("[SIO] PERCOM inválido, usando SD 128b."));
    bytesPerTrack  = 128 * 18;
    bytesPerSector = 128;
    sides          = 1;
    tracks         = 40;
  }

  uint8_t spt = (uint8_t)(bytesPerTrack / bytesPerSector);

  Serial.print(F("[SIO] PERCOM decodificado: tracks="));
  Serial.print(tracks);
  Serial.print(F(" bytesTrack="));
  Serial.print(bytesPerTrack);
  Serial.print(F(" sides="));
  Serial.print(sides);
  Serial.print(F(" bpsCode="));
  Serial.print(bpsCode);
  Serial.print(F(" -> bytes/sector="));
  Serial.print(bytesPerSector);
  Serial.print(F(" spt="));
  Serial.println(spt);

#if BRIDGE_PERCOM_TRANSPARENTE
  Serial.println(F("[SIO] BRIDGE_PERCOM_TRANSPARENTE=1 -> NO se cambia geometría local."));
  return;
#endif

  g_bytesPerSector = bytesPerSector;
  g_isDDActive     = (bytesPerSector > 128);

  Serial.print(F("[SIO] Geometría actualizada: bytes/sector="));
  Serial.print((uint16_t)g_bytesPerSector);
  Serial.print(F(" DD="));
  Serial.println((bool)g_isDDActive ? 1 : 0);
}

// ================== SETUP / LOOP (CORE0) ==================

void setup() {
  initGeometry();

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
}

void loop() {
  static unsigned long lastStats = 0;

  int cmdState = digitalRead(PIN_CMD);
  if (lastCmdState == HIGH && cmdState == LOW) {
    Serial.println(F("[SIO] CMD ↓ detectado, leyendo frame de comando..."));
    handleSioCommand();
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
  // no delays largos aquí (460800)
  delay(0);
}