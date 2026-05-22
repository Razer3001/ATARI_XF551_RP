#ifndef TNFS_TYPES_H
#define TNFS_TYPES_H

#include <Arduino.h>

struct TnfsParsedUrl {
  String raw;
  String host;
  String path;
  uint16_t port;
};

#endif // TNFS_TYPES_H
