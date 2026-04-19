#pragma once

#include <Arduino.h>

#ifndef RX_PKT_MAX
#define RX_PKT_MAX 256
#endif

struct PendingEspNowPacket {
  bool valid;
  uint8_t src[6];
  bool hasSrc;
  int len;
  uint8_t data[RX_PKT_MAX];
};
