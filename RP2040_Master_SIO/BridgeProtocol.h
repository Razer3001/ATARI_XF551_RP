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
