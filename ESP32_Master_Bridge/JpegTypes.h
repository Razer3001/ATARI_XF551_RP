// ================================================================
// VERSION: F49Z77_JPEG_FILE_WRITER_HEADER_FIX_2026-05-20
// ARCHIVO: JpegTypes.h
// ROL: Tipos JPEG usados antes de la generación automática de prototipos Arduino
// ================================================================
#pragma once

#include <Arduino.h>
#include <FS.h>
#include <vector>

struct JpegHuffSymbol {
  uint16_t code;
  uint8_t size;
};

struct JpegBitWriter {
  std::vector<uint8_t>& out;
  uint16_t bitBuf;
  uint8_t bitCnt;

  JpegBitWriter(std::vector<uint8_t>& o) : out(o), bitBuf(0), bitCnt(0) {}

  void emitByte(uint8_t b) {
    out.push_back(b);
    if (b == 0xFF) out.push_back(0x00);
  }

  void putBits(uint16_t bits, uint8_t count) {
    if (count == 0) return;
    for (int8_t i = count - 1; i >= 0; i--) {
      bitBuf = (uint16_t)((bitBuf << 1) | ((bits >> i) & 1));
      bitCnt++;
      if (bitCnt == 8) {
        emitByte((uint8_t)(bitBuf & 0xFF));
        bitBuf = 0;
        bitCnt = 0;
      }
    }
  }

  void finish() {
    if (bitCnt > 0) {
      bitBuf <<= (8 - bitCnt);
      bitBuf |= (uint16_t)((1U << (8 - bitCnt)) - 1U); // padding con 1s
      emitByte((uint8_t)(bitBuf & 0xFF));
      bitBuf = 0;
      bitCnt = 0;
    }
  }
};


struct JpegFileBitWriter {
  File& out;
  uint16_t bitBuf;
  uint8_t bitCnt;
  size_t& totalBytes;
  bool ok;

  JpegFileBitWriter(File& f, size_t& total) : out(f), bitBuf(0), bitCnt(0), totalBytes(total), ok(true) {}

  void emitByte(uint8_t b) {
    if (!ok) return;
    size_t w = out.write(&b, 1);
    if (w != 1) { ok = false; return; }
    totalBytes++;
    if (b == 0xFF) {
      const uint8_t stuffed = 0x00;
      w = out.write(&stuffed, 1);
      if (w != 1) { ok = false; return; }
      totalBytes++;
    }
  }

  void putBits(uint16_t bits, uint8_t count) {
    if (!ok || count == 0) return;
    for (int8_t i = count - 1; i >= 0; i--) {
      bitBuf = (uint16_t)((bitBuf << 1) | ((bits >> i) & 1));
      bitCnt++;
      if (bitCnt == 8) {
        emitByte((uint8_t)(bitBuf & 0xFF));
        bitBuf = 0;
        bitCnt = 0;
      }
    }
  }

  void finish() {
    if (!ok) return;
    if (bitCnt > 0) {
      bitBuf <<= (8 - bitCnt);
      bitBuf |= (uint16_t)((1U << (8 - bitCnt)) - 1U);
      emitByte((uint8_t)(bitBuf & 0xFF));
      bitBuf = 0;
      bitCnt = 0;
    }
  }
};
