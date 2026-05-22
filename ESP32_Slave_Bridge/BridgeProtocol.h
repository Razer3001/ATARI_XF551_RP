#pragma once
#include <Arduino.h>

// Protocolo interno RP2040 <-> MASTER/SLAVE
#define TYPE_CMD_FRAME     0x01
#define TYPE_SECTOR_CHUNK  0x10
#define TYPE_ACK           0x11
#define TYPE_NAK           0x12
#define TYPE_HELLO         0x20
#define TYPE_TIMING_UPDATE 0x30
#define TYPE_CFG_UPDATE    0x40
#define TYPE_CFG_ACK       0x41
#define TYPE_SUPERDOS_HINT 0x42

// Printer 820 / Atari P: (SIO DEV 0x40..0x43)
#define TYPE_PRINTER_CFG_UPDATE 0x70  // MASTER -> RP2040
#define TYPE_PRINTER_LINE       0x71  // RP2040 -> MASTER
#define TYPE_PRINTER_CFG_ACK    0x72
#define TYPE_PRINTER_DIAG       0x73  // RP2040 -> MASTER printer diagnostic counters  // RP2040 -> MASTER
#define TYPE_DISK_DIAG          0x74  // RP2040 -> MASTER disk READ/DATA diagnostic

// Cassette manual C: (F49)
#define TYPE_CAS_CONTROL       0x80  // MASTER -> RP2040: play/stop/rewind/end
#define TYPE_CAS_DATA          0x81  // MASTER -> RP2040: cassette data bytes
#define TYPE_CAS_STATUS        0x82  // RP2040 -> MASTER: playback/buffer status
#define TYPE_CAS_ACK           0x83  // RP2040 -> MASTER: ACK de fragmento cassette

// Códigos SIO Atari
#define SIO_ACK        0x41
#define SIO_NAK        0x4E
#define SIO_COMPLETE   0x43
#define SIO_ERROR      0x45

// Rango de device IDs del bridge (D1..D4)
#define SIO_DEV_MIN    0x31
#define SIO_DEV_MAX    0x34

// Layout del chunk
#define CHUNK_PAYLOAD      240

// PERCOM
#define PERCOM_BLOCK_LEN   12
#define PERCOM_SEC_MAGIC   0xFFFF

static inline uint16_t percomTracks(const uint8_t p[PERCOM_BLOCK_LEN]) {
  return (uint16_t)p[0];
}

static inline uint16_t percomSectorsPerTrack(const uint8_t p[PERCOM_BLOCK_LEN]) {
  return ((uint16_t)p[2] << 8) | (uint16_t)p[3];
}

static inline uint8_t percomSides(const uint8_t p[PERCOM_BLOCK_LEN]) {
  return (uint8_t)((p[4] & 0x01) ? 2 : 1);
}

static inline uint16_t percomBytesPerSector(const uint8_t p[PERCOM_BLOCK_LEN]) {
  return ((uint16_t)p[6] << 8) | (uint16_t)p[7];
}
