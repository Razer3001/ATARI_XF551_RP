#pragma once
#include <Arduino.h>

// F36: tipo usado por el índice de Biblioteca.
// Va en header para que el preprocesador de Arduino conozca el tipo
// antes de generar prototipos automáticos de funciones que lo usan.
struct WebLibraryScanEntry {
  String name;
  String path;
};
