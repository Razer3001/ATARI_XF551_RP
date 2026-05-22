#pragma once
#include <Arduino.h>

struct WebAtrMeta {
  bool valid;
  uint32_t fileSize;
  uint32_t dataBytes;
  uint16_t sectorSize;
  uint32_t totalSectors;
};
