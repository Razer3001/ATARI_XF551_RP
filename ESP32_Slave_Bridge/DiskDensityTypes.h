#pragma once

#include <Arduino.h>

enum DiskDensity : uint8_t {
  DENS_UNKNOWN = 0,
  DENS_SD,
  DENS_ED,
  DENS_DD
};

struct DiskGeometry {
  uint8_t tracks;
  uint16_t sectorsPerTrack;
  uint8_t sides;
  uint16_t bytesPerSector;
  DiskDensity density;
  bool valid;
};
