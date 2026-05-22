#pragma once
#include <Arduino.h>

// F41B: tipo usado por los overrides manuales de perfil CAS.
// Va en header para que el preprocesador de Arduino conozca el tipo
// antes de generar prototipos automaticos de funciones que lo usan.
struct CasProfileOverride {
  String file;
  String profile;
};
