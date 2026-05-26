// ================================================================
// ATARI_XF551_RP - ESP32 MASTER
// BUILD: F43Y_LIBRARY_DOT_MENU_CENTER_CARD_2026-05-25_2055
// DATE : 2026-05-25
// TIME : 18:45
// ARCHIVO: ESP32_Master_Bridge.ino
//
// CAMBIOS VIGENTES:
// - Base funcional F22 conservada sin cambios de logica runtime.
// - Limpieza segura de archivos historicos/demo/tools que no participan en ejecucion.
// - Limpieza binaria paso 1: eliminadas funciones C++ candidatas de alta confianza sin referencias directas.
// - Biblioteca mantiene busqueda, filtros por tipo, conteos, paginacion y montaje exclusivo por unidad.
// - Solo el boton Refrescar biblioteca ejecuta /api/library?refresh=1.
// - Cargas normales usan /api/library?refresh=0; no re-escanean SD.
// - WEB-ATR sincroniza estado con Biblioteca y muestra totales ATR/XEX/COM/EXE.
// - Mantiene data liviana para evitar SPIFFS lleno; ATR/CAS van en SD.
// - F25: Gateway Windows recibe ATASCII crudo + glifos 8x8 para imprimir caracteres especiales.
// - F26: CAS Turbo Software/Zybex 600->800 tolerante a baud 605/802 y FSK final no bloqueante.
// - F27: TNFS permite descargar .CAS a /CAS y preparar C: directamente desde la pantalla TNFS.
// - F28: TNFS conserva .XEX/.COM/.EXE/.BAS crudos con su extension original; Biblioteca elimina contenedor antiguo.
// - F29: CAS River Raid Cold Prism detectado como 600->4000 bps, sin confundirlo con FSK especial.
// - F40: expone perfil CAS detectado en Biblioteca desde cache /CAS/.cache, sin cambiar aun reproduccion/carga real.
// - F41: agrega UI/endpoint para overrides de perfil CAS en /CONFIG/cas_profiles.json.
// - F41B: mueve CasProfileOverride a header para compilar correctamente en Arduino IDE.
// - F42: Play CAS usa perfil efectivo (override/cache) para baud/gaps con fallback seguro.
// - F42F: /api/library emite files por streaming seguro, sin buffer gigante y sin comas dobles.
// - F42H: rebuild automático nunca puede reducir/limpiar el índice vigente; solo refresh manual confirmado puede hacerlo.
// - F42I: guardado transaccional del índice: escribe TMP, verifica y recién entonces reemplaza library_index.json.
// - F42J: navegación Biblioteca en modo solo lectura; refresh no guarda SD salvo commit=1; recuperación automática desde .bak.
// - F42L: /api/library no declara Content-Length; responde chunked cerrado correctamente para evitar ERR_CONTENT_LENGTH_MISMATCH.
// - F42M: refresh manual ya no responde 500; si el re-escaneo falla, conserva y sirve índice/backup existente.
// - F42N: commit=1 devuelve respuesta compacta sin streaming de files[] para evitar ERR_INCOMPLETE_CHUNKED_ENCODING.
// - F42P: diagnóstico de SD por consola; lista carpetas escaneadas y archivos montables encontrados.
// - F42S: rebuild/commit usa escaneo seguro y evita copiar String grande antes de guardar.
// - F42Y: /api/library lista files[] desde escaneo seguro paginado, sin depender de RAM/index JSON.
// - F43B: cache RAM extendida, payload reducido por defecto y cache de página JSON visible.
// - F43C: escaneo SD de Biblioteca deshabilitado en navegación/status/cassette; solo botón manual refresca.
// - F43D: commit=1 también devuelve la página visible files[] usando cache live del botón.
// - F43E: elimina límites duros de 100 en Biblioteca; el escaneo guarda/lista todos los archivos y la paginación es solo visual.
// - F43F: acelera Biblioteca usando fileSize del escaneo, evita abrir cada ATR al listar y difiere covers.
// - F43G: TNFS streaming cierra NDJSON correctamente y agrega fallback para XEX sin evento final.
// - F43H: restaura montaje Web-ATR para XEX/COM/EXE aunque el payload rápido no incluya sectorSize/totalSectors.
// - F43I: lectura/listado/escritura Biblioteca cerrados: solo botón manual escanea; commit guarda y devuelve files visibles.
// - F43J: índice persistente NDJSON en /CONFIG/library_index.ndjson; lectura línea a línea sin JSON gigante en RAM.
// - F43L: corrige orden de constantes WEB_LIBRARY_INDEX_* en respuesta streaming para compilar en Arduino.
// - F43K: respuestas grandes files[] se envían por streaming seguro; evita String gigante y coma final },]}.
// - F43M: agrega cache de listado en navegador y mejora carga diferida/persistente de carátulas.
// - F43N: refresh=0 lee primero /CONFIG/library_index.ndjson; RAM/live cache queda solo como fallback sin escaneo.
// - F43P: Biblioteca usa paginación HTTP segura de 50, sin límite total de archivos y sin fallback legacy JSON.
// - F43Q: agrega botón Actualizar imágenes en Biblioteca; limpia cache local/missing y recarga carátulas visibles.
// - F43R: restaura acciones de imagen por card en Biblioteca: Agregar URL, Subir imagen, Abrir Libretro, Guardar proxy SD y Quitar manual.
// - F43S: al montar ATR o preparar C: la Biblioteca actualiza solo la card/estado; no reconstruye la grilla ni recarga imágenes.
// - F43T: acciones de imagen por card vuelven a combobox: URL, subir, Libretro, proxy SD y quitar manual.
// - F43U: acciones de imagen por card vuelven al botón compacto "." con menú desplegable.
// ================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>
#include <vector>
#include "JpegTypes.h"
#include <algorithm>
#include <pgmspace.h>
#include <math.h>
#include "BridgeProtocol.h"
#include "WebAtrTypes.h"
#include "TnfsTypes.h"
#include "WebLibraryTypes.h"
#include "CasProfileTypes.h"

// ================================================================
// F7_SD_STORAGE_ESP32_WROOM
// Almacenamiento principal para Biblioteca/WEB-ATR/Glifos/Caratulas.
// 1 = usar microSD por SPI. 0 = volver a SPIFFS interno.
// Pines recomendados para ESP32-WROOM:
//   microSD VCC  -> 3.3V
//   microSD GND  -> GND
//   microSD CS   -> GPIO5
//   microSD SCK  -> GPIO14
//   microSD MOSI -> GPIO13
//   microSD MISO -> GPIO27
// Formato recomendado de tarjeta: FAT32.
// ================================================================
#ifndef WEB_STORAGE_USE_SD
#define WEB_STORAGE_USE_SD 1
#endif

#if WEB_STORAGE_USE_SD
#define WEB_STORAGE_NAME "SD"
static const int WEB_SD_CS   = 5;
static const int WEB_SD_SCK  = 14;
static const int WEB_SD_MOSI = 13;
static const int WEB_SD_MISO = 27;
static const uint32_t WEB_SD_FREQ_HZ = 10000000UL;

#if defined(CONFIG_IDF_TARGET_ESP32S3)
static SPIClass WebStorageSPI(FSPI);
#else
static SPIClass WebStorageSPI(VSPI);
#endif

class WebStorageAdapter {
public:
  bool ready = false;

  bool begin(bool formatOnFail = true) {
    (void)formatOnFail;
    if (ready) return true;

    WebStorageSPI.begin(WEB_SD_SCK, WEB_SD_MISO, WEB_SD_MOSI, WEB_SD_CS);
    ready = SD.begin(WEB_SD_CS, WebStorageSPI, WEB_SD_FREQ_HZ);
    if (!ready) {
      // Segundo intento aún más conservador para cableados largos o módulos sensibles.
      ready = SD.begin(WEB_SD_CS, WebStorageSPI, 4000000UL);
    }
    if (ready) ensureBaseDirs();
    return ready;
  }

  File open(const char* path, const char* mode = "r") {
    return SD.open(normalizePath(path).c_str(), mode);
  }
  File open(const String& path, const char* mode = "r") {
    return SD.open(normalizePath(path).c_str(), mode);
  }
  bool exists(const char* path) { return SD.exists(normalizePath(path).c_str()); }
  bool exists(const String& path) { return SD.exists(normalizePath(path).c_str()); }
  bool remove(const char* path) { return SD.remove(normalizePath(path).c_str()); }
  bool remove(const String& path) { return SD.remove(normalizePath(path).c_str()); }
  bool rename(const String& from, const String& to) {
    return SD.rename(normalizePath(from).c_str(), normalizePath(to).c_str());
  }
  bool rename(const char* from, const char* to) {
    return SD.rename(normalizePath(from).c_str(), normalizePath(to).c_str());
  }
  bool mkdir(const char* path) { return SD.mkdir(normalizePath(path).c_str()); }
  bool mkdir(const String& path) { return SD.mkdir(normalizePath(path).c_str()); }

  uint64_t totalBytes() {
    uint64_t t = SD.totalBytes();
    if (t == 0) t = SD.cardSize();
    return t;
  }
  uint64_t usedBytes() { return SD.usedBytes(); }

private:
  String normalizePath(const char* p) {
    if (!p || !p[0]) return "/";
    String s(p);
    return normalizePath(s);
  }
  String normalizePath(String s) {
    s.trim();
    s.replace("\\", "/");
    if (!s.length()) s = "/";
    if (!s.startsWith("/")) s = "/" + s;
    return s;
  }
  void ensureDir(const char* path) {
    String p = normalizePath(path);
    File d = SD.open(p.c_str(), "r");
    if (d) {
      bool isDir = d.isDirectory();
      d.close();
      if (isDir) return;
    }
    // Si existe como archivo (por ejemplo /CONFIG creado accidentalmente),
    // se elimina para poder crear la carpeta real.
    if (SD.exists(p.c_str())) SD.remove(p.c_str());
    SD.mkdir(p.c_str());
  }
  void ensureBaseDirs() {
    ensureDir("/ATR");
    ensureDir("/LIBRARY");
    ensureDir("/PRINT");
    ensureDir("/GLYPHS");
    ensureDir("/CONFIG");
    ensureDir("/TMP");
    ensureDir("/COVERS");
    ensureDir("/MINI_COVERS");
    ensureDir("/CAS");
  }
};

static WebStorageAdapter WebStorage;
// Compatibilidad: el código existente sigue llamando SPIFFS.open/exists/etc.,
// pero en F7 esas llamadas se redirigen a la microSD.
#define SPIFFS WebStorage
#else
#define WEB_STORAGE_NAME "SPIFFS"
#endif

#ifndef TYPE_SUPERDOS_HINT
#define TYPE_SUPERDOS_HINT 0x42
#endif

#ifndef TYPE_DISK_DIAG
#define TYPE_DISK_DIAG 0x74
#endif

#ifndef ESP_IDF_VERSION_MAJOR
#define ESP_IDF_VERSION_MAJOR 4
#endif

Preferences prefs;

static const char MASTER_BUILD[] = "F43Y_LIBRARY_DOT_MENU_CENTER_CARD_2026-05-25_2055";

// ===== Debug =====
#define MASTER_UART_BYTE_DEBUG 0   // 1 = logea cada byte UART (NO recomendado)

// ===== Protocol =====
#define UART_SYNC 0x55

// D1..D4 físicos/compatibilidad base.
#define DEV_MIN 0x31
#define DEV_MAX 0x34

// Unidades virtuales dinámicas para WEB-ATR / BT-SIO2PC.
// D1-D4 siempre visibles; D5-D7 se agregan desde la web cuando el usuario las necesita.
#define DRIVE_UI_MAX_UNITS 7
#define DRIVE_UI_DEFAULT_VISIBLE_MASK 0x0F
#define DRIVE_UI_MAX_MASK 0x7F

// Prefetch máximo (UI puede mandar más, acá se clampa)
#define MAX_PREFETCH_SECTORS 4

// Broadcast MAC
const uint8_t BCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Canal WiFi fijo (AP + ESP-NOW). Debe coincidir en SLAVES.
static const uint8_t WIFI_CHANNEL = 1;

// IMPORTANTÍSIMO:
// El SoftAP del ESP32 por defecto usa 192.168.4.1/24.
// Si tu router/impresora también está en 192.168.4.x, el MASTER queda con dos interfaces
// en la misma subred y el socket RAW 9100 puede salir por el AP equivocado.
// Lo movemos a 192.168.50.1/24 para no chocar con la red STA de la impresora.
static const IPAddress MASTER_AP_IP(192, 168, 50, 1);
static const IPAddress MASTER_AP_GW(192, 168, 50, 1);
static const IPAddress MASTER_AP_MASK(255, 255, 255, 0);

static const char MASTER_AP_SSID[] = "XF551_MASTER";
static const char MASTER_AP_PASS[] = "xf551wifi";
static const char MASTER_MDNS_HOST[] = "atari-sio";   // http://atari-sio.local cuando STA conecta
static bool g_staConnectDeferred = false;
static uint32_t g_staConnectAfterMs = 0;
static bool g_mdnsStarted = false;

// Último SLAVE conocido (fallback)
uint8_t g_lastSlave[6] = { 0 };
bool g_haveSlave = false;

// UART2 (con RP2040)
const int PIN_RP_RX = 16;  // RX2
const int PIN_RP_TX = 17;  // TX2

// ===== BT-DISK-GATEWAY V9 =====
// El MASTER queda como pasarela: D1..D4 pueden ir por BT o por la disquetera física.
// En ESP32-S3 se usa un módulo externo HC-05/HC-06 por UART TTL.
// Cableado por defecto:
//   HC TXD -> ESP32 MASTER GPIO18 (RX)
//   HC RXD -> ESP32 MASTER GPIO19 (TX) con divisor de nivel si corresponde
//   GND común.
#define BT_DISK_BRIDGE_COMPILED 1
const int PIN_BT_RX = 18;
const int PIN_BT_TX = 19;
static uint32_t BT_DISK_UART_BAUD_CFG = 115200;
static bool     BT_DISK_ENABLED = false;
static uint8_t  BT_DISK_DEV_MASK = 0x01;   // bit0=D1, bit1=D2, bit2=D3, bit3=D4
// FORCE_MASK deshabilita fallback a disquetera física para esas unidades.
// Ej: force=1 => D1 queda solo por BT; si host BT no está online, se responde NAK al RP/Atari.
static uint8_t  BT_DISK_FORCE_MASK = 0x00;
HardwareSerial SerialBTDisk(1);

// ===== BT-SIO2PC directo compatible RespeQt/AspeQt (experimental) =====
// Usa el mismo HC-05/HC-06 por UART, pero en modo SIO crudo.
// MASTER envia a RespeQt/AspeQt el frame SIO real: DEV,CMD,AUX1,AUX2,CHK.
#define BT_SIO2PC_COMPILED 1
static bool     BT_SIO2PC_ENABLED = false;
static uint8_t  BT_SIO2PC_DEV_MASK = 0x00;
static uint8_t  BT_SIO2PC_FORCE_MASK = 0x00;
static uint32_t g_btSio2pcCmdRouted = 0;
static uint32_t g_btSio2pcReadOk = 0;
static uint32_t g_btSio2pcStatusOk = 0;
static uint32_t g_btSio2pcPercomOk = 0;
static uint32_t g_btSio2pcNak = 0;
static uint32_t g_btSio2pcTimeout = 0;
static uint32_t g_btSio2pcBadChk = 0;
static uint8_t  g_btSio2pcLastCmd = 0;
static uint8_t  g_btSio2pcLastDev = 0;
static uint16_t g_btSio2pcLastSec = 0;
static uint32_t g_btSio2pcLastMs = 0;
// Perfiles de velocidad BT-SIO2PC.
// stable: tolerante a latencia Bluetooth; fast/loader: menos espera; turbo: minima latencia, requiere enlace limpio.
static uint8_t  BT_SIO2PC_PROFILE = 0;  // F47: estable por defecto; perfiles rápidos solo bajo prueba
static uint8_t  BT_SIO2PC_UNIT_PROFILE[DRIVE_UI_MAX_UNITS] = {0, 0, 0, 0, 0, 0, 0}; // D1..D7: 0=stable, 1=fast, 2=loader, 3=turbo
static bool     BT_SIO2PC_QUIET_LOG = true;
static uint16_t BT_SIO2PC_ACK_TIMEOUT_MS = 1200;
static uint16_t BT_SIO2PC_COMPLETE_TIMEOUT_MS = 2200;
static uint16_t BT_SIO2PC_DATA_TOTAL_TIMEOUT_MS = 3500;
static uint16_t BT_SIO2PC_DATA_IDLE_TIMEOUT_MS = 160;
static uint16_t BT_SIO2PC_CHK_TIMEOUT_MS = 900;
static uint16_t BT_SIO2PC_PRE_DRAIN_MS = 1;
static uint8_t  BT_SIO2PC_MAX_RETRY = 0;
static bool     BT_SIO2PC_WEB_YIELD_DURING_IO = true;
static uint16_t BT_SIO2PC_WEB_YIELD_INTERVAL_MS = 40;
static uint32_t g_btSio2pcBurstReads = 0;
static uint32_t g_btSio2pcBytesRx = 0;
static uint32_t g_btSio2pcLastLogMs = 0;
static uint32_t g_btSio2pcTotalMs = 0;
static uint32_t g_btSio2pcMaxMs = 0;
static uint32_t g_btSio2pcAvgMs = 0;
static uint32_t g_btSio2pcRetry = 0;
static uint32_t g_btSio2pcDrainBytes = 0;
static uint32_t g_btSio2pcNoWebLoops = 0;
static char     g_btSio2pcLastErr[40] = "";

// F46: Buffers y caché para BT-SIO2PC.
// No cambia el protocolo SIO: solo reduce latencia local y evita pedir de nuevo sectores recientes.
#define BT_SIO2PC_UART_RX_BUFFER 8192
#define BT_SIO2PC_UART_TX_BUFFER 2048
#define BT_SIO2PC_CACHE_SLOTS 32
#define BT_SIO2PC_CACHE_TTL_MS 300000UL

struct BtSio2pcCacheEntry {
  bool valid;
  uint8_t dev;
  uint16_t sec;
  uint16_t len;
  uint32_t lastUseMs;
  uint8_t data[256];
};

static BtSio2pcCacheEntry g_btSio2pcCache[BT_SIO2PC_CACHE_SLOTS];
static uint32_t g_btSio2pcCacheHit = 0;
static uint32_t g_btSio2pcCacheMiss = 0;
static uint32_t g_btSio2pcCacheStore = 0;
static uint32_t g_btSio2pcCacheClear = 0;
static uint32_t g_btSio2pcCacheBypass = 0;
static uint32_t g_btSio2pcSioPrioritySkips = 0;

// F47: por estabilidad, la caché BT-SIO2PC queda apagada por defecto.
// Se puede activar después solo para pruebas controladas si el host y el ATR son de solo lectura.
static bool     BT_SIO2PC_CACHE_ENABLED = false;
static uint32_t g_btSio2pcStaleDrains = 0;
static uint32_t g_btSio2pcLateBytesAfterError = 0;

// F47: Serial2.write() ya transmite por interrupción; no bloquear esperando cada frame.
// Si necesitas volver al comportamiento anterior, cambia a 1.
#ifndef MASTER_UART_FLUSH_EACH_FRAME
#define MASTER_UART_FLUSH_EACH_FRAME 0
#endif


// ===== WEB-ATR FLASH MULTI V11 =====
// Permite subir varios ATR a la flash interna del MASTER y montarlos por unidad D1..D4.
// Prioridad de rutas: WEB-ATR local -> BT-DISK -> ESP-NOW/XF551 física.
#define WEB_ATR_GATEWAY_COMPILED 1
#define WEB_ATR_MAX_UNITS DRIVE_UI_MAX_UNITS
static const uint8_t WEB_ATR_MOUNT_PROFILE = 2;
static uint8_t  DRIVE_VISIBLE_MASK = DRIVE_UI_DEFAULT_VISIBLE_MASK;
static bool     WEB_ATR_ENABLED = false;
static uint8_t  WEB_ATR_DEV_MASK = 0x00;    // bit0=D1, bit1=D2, bit2=D3, bit3=D4
static uint8_t  WEB_ATR_FORCE_MASK = 0x00;  // si está forzado y no hay ATR válido => NAK sin fallback físico
#if WEB_STORAGE_USE_SD
static const char WEB_ATR_PREFIX[] = "/ATR/";
#else
static const char WEB_ATR_PREFIX[] = "/atr_";
#endif
#if WEB_STORAGE_USE_SD
static const char WEB_ATR_TMP_PATH[] = "/TMP/atr_upload.tmp";
#else
static const char WEB_ATR_TMP_PATH[] = "/atr_upload.tmp";
#endif
static String   g_webAtrMountedName[WEB_ATR_MAX_UNITS] = { "", "", "", "", "", "", "" };
// F17: ruta física y metadata cacheadas por unidad. Evita escanear /ATR, /LIBRARY y raíz en cada sector leído.
static String   g_webAtrMountedPath[WEB_ATR_MAX_UNITS] = { "", "", "", "", "", "", "" };
static WebAtrMeta g_webAtrMountedMeta[WEB_ATR_MAX_UNITS];
static bool     g_webAtrSlotPresent[WEB_ATR_MAX_UNITS] = { false, false, false, false, false, false, false };
static uint32_t g_webAtrReadsByUnit[WEB_ATR_MAX_UNITS] = { 0, 0, 0, 0, 0, 0, 0 };
static uint32_t g_webAtrStatusByUnit[WEB_ATR_MAX_UNITS] = { 0, 0, 0, 0, 0, 0, 0 };
static uint32_t g_webAtrPercomByUnit[WEB_ATR_MAX_UNITS] = { 0, 0, 0, 0, 0, 0, 0 };
static bool     g_webAtrFilePresent = false;
static uint32_t g_webAtrReads = 0;
static uint32_t g_webAtrStatus = 0;
static uint32_t g_webAtrPercom = 0;
static uint32_t g_webAtrNak = 0;
static String   g_webAtrUploadError;
static File     g_webAtrUploadFile;
static uint32_t g_webAtrUploadBytes = 0;
static String   g_webAtrPendingName = "";
static String   g_webAtrPendingStoredName = "";
static String   g_webAtrTmpPath = "";
static bool     g_webAtrPendingIsXex = false;
static bool     g_webAtrPendingIsCas = false;

// ===== CASSETTE MANUAL PLAY F49 =====
// Biblioteca + montaje lógico C: + Play/Stop/Rewind manual para archivos .CAS.
// No requiere cable MOTOR pin 8: el usuario sincroniza CLOAD + Play desde la web.
#define CASSETTE_LIBRARY_COMPILED 1
#if WEB_STORAGE_USE_SD
static const char WEB_CAS_PREFIX[] = "/CAS/";
#else
static const char WEB_CAS_PREFIX[] = "/cas_";
#endif
#if WEB_STORAGE_USE_SD
static const char WEB_CAS_TMP_PATH[] = "/TMP/cas_upload.tmp";
#else
static const char WEB_CAS_TMP_PATH[] = "/cas_upload.tmp";
#endif
static String   g_casMountedName = "";
static String   g_casMountedPath = "";
static bool     g_casMounted = false;
static uint32_t g_casUploads = 0;
static uint32_t g_casMounts = 0;
static uint32_t g_casUnmounts = 0;
static uint32_t g_casBytesLast = 0;
static char     g_casLastError[80] = "";

// ===== CASSETTE MANUAL PLAY F49 =====
// Reproduce .CAS sin cable MOTOR pin 8: el usuario sincroniza manualmente CLOAD + Play.
#define CASSETTE_MANUAL_PLAY_COMPILED 1
#define CAS_PLAY_CHUNK_BYTES 96
#define CAS_ASPEQT_FRAGMENT_BYTES 10
#define CAS_RP_BUFFER_SAFE_FREE 384

static File     g_casPlayFile;
static bool     g_casPlaying = false;
static bool     g_casPaused = false;       // F49Z70: pausa lógica estilo casetera; no avanza nuevos bloques DATA.
static bool     g_casEof = false;
static bool     g_casRawMode = false;
static bool     g_casFujiFormat = false;
static uint32_t g_casParserStartOffset = 0;
static bool     g_casPendingBlockDelay = false;
static uint16_t g_casPendingBlockDelayMs = 0;
// F49Q: modo compatible boot cassette. Sin cable MOTOR; respeta IRG sin bloquear por rpState.
// entre registros y esperando a que el RP2040 termine el registro anterior.
// Esto evita que el OS lea registros pegados y caiga en BOOT ERROR.
static bool     g_casSdriveStreamMode = false;
static bool     g_casRecordDrainPending = false;
static uint16_t g_casLastIrgAppliedMs = 0;
static uint32_t g_casTimingWaitUntilMs = 0;
static uint32_t g_casDrainWaitStartMs = 0;
#define CAS_IRG_MAX_MS 60000
#define CAS_IRG_BOOT_SHORT_MS 260
#define CAS_IRG_BOOT_FIRST_MS 600
#define CAS_IRG_CLOAD_SHORT_MS 260
#define CAS_UART_SETTLE_MS 0
static uint32_t g_casDrainTargetBytes = 0;
static uint32_t g_casDrainDeadlineMs = 0;      // F49X: deadline dinamico = IRG + tiempo serial + margen.
static uint32_t g_casRecordExpectedMs = 0;     // F49X: duracion estimada del registro actual.
static uint32_t g_casTransportTargetRx = 0;    // F49X: confirma que RP recibio el ultimo fragmento antes del siguiente.
static uint32_t g_casTransportWaitSkips = 0;
static uint32_t g_casTransportTimeouts = 0;
static uint32_t g_casDrainTimeouts = 0;
// F49Z6: drenaje estricto por registro. No avanzar al siguiente registro
// mientras el RP2040 siga ocupado, aunque el deadline dinamico haya vencido.
#define CAS_STRICT_DRAIN_HARD_MS 120000UL
static bool     g_casStrictRecordGate = true;
static uint32_t g_casDrainHoldMs = 0;
static uint32_t g_casDrainHardTimeouts = 0;
static uint32_t g_casPostRecordSettleUntilMs = 0;
static uint32_t g_casRecordsCompleted = 0;
static uint32_t g_casRecordsStarted = 0;
static bool     g_casBootFirstIrgSkipped = false;
static bool     g_casSdriveExactMode = false;
static bool     g_casIronTurboCompat = false;
static bool     g_casIronTurboDetected = false;
static uint16_t g_casIronTurboTurboBaud = 600;
static uint32_t g_casIronTurboGapFixes = 0;
static uint8_t  g_casPrevCompletedRecordMarker = 0;
#define CAS_IRON_REOPEN_GAP_MS 12000
static bool     g_casSdriveBlockExactMode = true; // F49Z5: registro lógico completo reconstruido en RP2040 antes de reproducir.
static bool     g_casSdriveEndGapPending = false;
static uint16_t g_casSdriveEndGapMs = 0;
static uint32_t g_casSdriveEndMarkers = 0;
static uint8_t  g_casCurrentRecordMarker = 0;
static uint16_t g_casCurrentRecordLen = 0;
static uint32_t g_casRpRecordBusySkips = 0;
static uint16_t g_casShortIrgMs = CAS_IRG_BOOT_SHORT_MS;
static uint32_t g_casTimingSkips = 0;
static char     g_casLastChunkType[5] = "";
static uint16_t g_casLastChunkLen = 0;
static uint16_t g_casLastChunkAux = 0;
static uint32_t g_casParserErrors = 0;
// F49Y: modo RAW/BIN estilo SDrive-MAX. Si un archivo no es FUJI-CAS,
// no se transmite crudo; se empaqueta en registros cassette Atari:
// 0x55 0x55 + marcador 0xFC/0xFA/0xFE + 128 bytes + checksum.
static bool     g_casSdriveRawRecordMode = false;
static uint8_t  g_casRawRecord[132];
static uint16_t g_casRawRecordLen = 0;
static uint16_t g_casRawRecordPos = 0;
static bool     g_casRawEndRecordQueued = false;
static uint32_t g_casRawRecordsBuilt = 0;
static uint8_t  g_casRawLastMarker = 0;
static bool     g_casInDataChunk = false;
static uint16_t g_casChunkRemain = 0;
static uint16_t g_casChunkGapMs = 0;
static bool     g_casChunkGapSent = false;
static uint16_t g_casCurrentBaud = 600;
static uint32_t g_casPlayStartedMs = 0;
static uint32_t g_casPlaySize = 0;
static uint32_t g_casPlayPos = 0;
static uint32_t g_casBytesQueued = 0;
static uint32_t g_casChunksQueued = 0;
static uint32_t g_casDataBlocks = 0;
static uint32_t g_casSkippedChunks = 0;
static uint32_t g_casFskChunks = 0;
static uint32_t g_casLastSendMs = 0;
static uint16_t g_casUserBaseBaud = 600;
static uint16_t g_casTurboMultiplierX100 = 100;
static bool     g_casTurboEnabled = false;
// F42: perfil CAS efectivo aplicado al Play.
static char     g_casEffectiveProfile[32] = "AUTO";
static bool     g_casEffectiveProfileManual = false;
static bool     g_casProfileLockBaud = false;
static uint16_t g_casProfileForcedBaud = 0;
static uint16_t g_casInitialDelaySec = 0;
static char     g_casLoadMode[8] = "cload";   // cload | boot | auto
static bool     g_casBootModeActive = false;
static uint32_t g_casDelayUntilMs = 0;
static uint32_t g_casPlayCount = 0;
static uint32_t g_casStopCount = 0;
static uint32_t g_casRewindCount = 0;
static uint32_t g_casPauseCount = 0;       // F49Z70
static uint32_t g_casResumeCount = 0;      // F49Z70
static uint32_t g_casSeekBackCount = 0;    // F49Z70
static uint8_t  g_casLastSeekBackRecords = 0; // F49Z70
static uint8_t  g_casRpState = 0;
static uint16_t g_casRpBufferUsed = 0;
static uint16_t g_casRpBufferFree = 4096;
static uint32_t g_casRpBytesPlayed = 0;
static uint32_t g_casRpBytesRx = 0;
static uint16_t g_casRpBaud = 600;
static uint8_t  g_casRpFlags = 0;
static uint32_t g_casRpLastStatusMs = 0;
static uint16_t g_casTxSeq = 0;
static volatile uint16_t g_casAckSeq = 0;
static volatile bool     g_casAckOk = false;
static volatile uint32_t g_casAckBytesRx = 0;
static uint32_t g_casAckRetries = 0;
static uint32_t g_casAckTimeouts = 0;
static uint32_t g_casAckNak = 0;
static uint32_t g_casAckWaitMsLast = 0;
// F49Z4: modo AspeQt y compatibilidad manual removidos de la UI.
// Se mantiene un flujo único SDrive/FujiNet por registros + ACK.
static bool     g_casAspeqtCompat = false;
static uint16_t g_casAspeqtFragmentBytes = CAS_ASPEQT_FRAGMENT_BYTES;
static uint16_t g_casEmbeddedBaud = 600;
static uint16_t g_casAspeqtEffectiveBaud = 600;
static uint32_t g_casAspeqtFragments = 0;
static uint32_t g_casAspeqtFinalWaitMs = 500;

static bool casIsKnownChunkType(const char* typ);
static uint16_t casClampBaud(uint32_t baud);
static void casAutoResetAnalysis();
static bool casAnalyzeCasFile(const String& path);
static bool webCasLooksLikeName(String name);
static String webCasPathForName(const String& storedName);
static String webAtrSanitizeFileName(const String& in);
static bool webLibraryIndexRebuild(bool saveToFs, bool allowShrink);

// ===== CASSETTE AUTO DETECT / CHUNK DIAG F49Z39 =====
// Analiza FUJI-CAS/A8CAS antes de reproducir para diagnosticar chunks data/baud/fsk/pwm*.
// Mantiene cassette sin cable MOTOR: la auto-pausa por gaps largos queda preparada pero no activa.
struct CasAutoAnalysis {
  bool valid;
  bool fujiFormat;
  bool rawFormat;
  bool hasBaudChunks;
  bool hasFskChunks;
  bool hasPwmChunks;
  bool usesStandardRecords;
  bool hasLongGap;
  bool hasShortGap;
  bool textIron;
  bool textTurbo;
  bool textT2000;
  bool textKso;
  bool textChaos;
  bool textInjektor;
  bool textColdPrism;
  bool textStac;
  bool textCain;
  uint16_t initialBaud;
  uint16_t turboBaud;
  uint16_t firstDataBaud;
  uint16_t maxBaud;
  uint16_t lastBaud;
  uint16_t totalChunks;
  uint16_t dataBlocks;
  uint16_t baudChunks;
  uint16_t fskChunks;
  uint16_t pwmChunks;
  uint16_t pwmStateChunks;   // pwms
  uint16_t pwmControlChunks; // pwmc
  uint16_t pwmDataChunks;    // pwmd
  uint16_t pwmLongChunks;    // pwml
  uint16_t standard132Blocks;
  uint16_t stacSync55Blocks;  // bloques DATA que comienzan con 0x55 0x55
  uint16_t cain217Blocks;     // bloques DATA de 217 bytes
  uint16_t longGaps;         // IRG >= 10000 ms
  uint16_t shortGaps;        // IRG < 80 ms
  uint16_t minIrgMs;
  uint16_t maxIrgMs;
  uint16_t lastIrgMs;
  uint16_t autoPauseCandidates; // base futura: IRG >= 3000 ms, sin MOTOR
  uint32_t fileBytes;          // tamaño real del .CAS en almacenamiento
  uint32_t payloadBytes;       // bytes reproducibles enviados al RP2040 (data/raw)
  uint32_t overheadBytes;      // headers/chunks/metadata/baud que no son audio/data
  char profile[24];
  char confidence[8];
  char suggestedMode[8];
  char chunkSummary[120];
  char notes[300];
};
static CasAutoAnalysis g_casAutoAnalysis;

// F49Z42: cache preanalizada de CAS. Se guarda binario para carga rápida y JSON
// resumido para diagnóstico humano en /CAS/.cache.
#define CAS_AUTO_CACHE_MAGIC 0x32415343UL  // "CAS2" little-endian
#define CAS_AUTO_CACHE_VERSION 131  // F39: invalida cache CAS; corrige falsos positivos STAC por 0x55 0x55
static const char CAS_AUTO_CACHE_DIR[] = "/CAS/.cache";
static bool webAtrFsReady();
String jsonEscape(const String& in);

struct CasAutoCacheBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t structSize;
  uint32_t sourceSize;
  CasAutoAnalysis analysis;
};

static bool casAutoEnsureCacheDir() {
  if (!webAtrFsReady()) return false;
  if (!SPIFFS.exists("/CAS")) SPIFFS.mkdir("/CAS");
  if (SPIFFS.exists(CAS_AUTO_CACHE_DIR)) return true;
  return SPIFFS.mkdir(CAS_AUTO_CACHE_DIR);
}

static String casAutoCacheBaseName(const String& path) {
  String name = path;
  name.replace("\\", "/");
  int slash = name.lastIndexOf('/');
  if (slash >= 0) name = name.substring(slash + 1);
  if (name.length() == 0) name = "cas";
  for (uint16_t i = 0; i < name.length(); i++) {
    char c = name[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-';
    if (!ok) name.setCharAt(i, '_');
  }
  return String(CAS_AUTO_CACHE_DIR) + "/" + name;
}

static bool casAutoLoadCache(const String& path) {
  if (!webAtrFsReady() || path.length() == 0 || !SPIFFS.exists(path)) return false;
  String binPath = casAutoCacheBaseName(path) + ".bin";
  if (!SPIFFS.exists(binPath)) return false;
  File src = SPIFFS.open(path, "r");
  uint32_t sourceSize = src ? (uint32_t)src.size() : 0;
  if (src) src.close();
  File f = SPIFFS.open(binPath, "r");
  if (!f) return false;
  CasAutoCacheBlob blob;
  if (f.read((uint8_t*)&blob, sizeof(blob)) != (int)sizeof(blob)) { f.close(); return false; }
  f.close();
  if (blob.magic != CAS_AUTO_CACHE_MAGIC || blob.version != CAS_AUTO_CACHE_VERSION || blob.structSize != sizeof(CasAutoAnalysis)) return false;
  if (blob.sourceSize != sourceSize || blob.analysis.fileBytes != sourceSize) return false;
  memcpy(&g_casAutoAnalysis, &blob.analysis, sizeof(CasAutoAnalysis));
  g_casAutoAnalysis.valid = true;
  return true;
}


// F40: lectura local de cache CAS para Biblioteca.
// No modifica g_casAutoAnalysis; solo expone metadata ya analizada.
static bool casAutoLoadCacheForPath(const String& path, CasAutoAnalysis& out) {
  memset(&out, 0, sizeof(out));
  if (!webAtrFsReady() || path.length() == 0 || !SPIFFS.exists(path)) return false;
  String binPath = casAutoCacheBaseName(path) + ".bin";
  if (!SPIFFS.exists(binPath)) return false;

  File src = SPIFFS.open(path, "r");
  uint32_t sourceSize = src ? (uint32_t)src.size() : 0;
  if (src) src.close();

  File f = SPIFFS.open(binPath, "r");
  if (!f) return false;
  CasAutoCacheBlob blob;
  int n = f.read((uint8_t*)&blob, sizeof(blob));
  f.close();
  if (n != (int)sizeof(blob)) return false;
  if (blob.magic != CAS_AUTO_CACHE_MAGIC || blob.version != CAS_AUTO_CACHE_VERSION || blob.structSize != sizeof(CasAutoAnalysis)) return false;
  if (blob.sourceSize != sourceSize || blob.analysis.fileBytes != sourceSize) return false;
  memcpy(&out, &blob.analysis, sizeof(CasAutoAnalysis));
  out.valid = true;
  return true;
}

static void casAutoSaveCache(const String& path) {
  if (!g_casAutoAnalysis.valid || !casAutoEnsureCacheDir()) return;
  String base = casAutoCacheBaseName(path);
  String binPath = base + ".bin";
  CasAutoCacheBlob blob;
  memset(&blob, 0, sizeof(blob));
  blob.magic = CAS_AUTO_CACHE_MAGIC;
  blob.version = CAS_AUTO_CACHE_VERSION;
  blob.structSize = sizeof(CasAutoAnalysis);
  blob.sourceSize = g_casAutoAnalysis.fileBytes;
  memcpy(&blob.analysis, &g_casAutoAnalysis, sizeof(CasAutoAnalysis));
  File f = SPIFFS.open(binPath, "w");
  if (f) { f.write((const uint8_t*)&blob, sizeof(blob)); f.close(); }

  String jsonPath = base + ".json";
  File jf = SPIFFS.open(jsonPath, "w");
  if (jf) {
    jf.print("{\"version\":42");
    jf.print(",\"fileBytes\":"); jf.print(g_casAutoAnalysis.fileBytes);
    jf.print(",\"payloadBytes\":"); jf.print(g_casAutoAnalysis.payloadBytes);
    jf.print(",\"overheadBytes\":"); jf.print(g_casAutoAnalysis.overheadBytes);
    jf.print(",\"chunks\":"); jf.print(g_casAutoAnalysis.totalChunks);
    jf.print(",\"dataBlocks\":"); jf.print(g_casAutoAnalysis.dataBlocks);
    jf.print(",\"baudChunks\":"); jf.print(g_casAutoAnalysis.baudChunks);
    jf.print(",\"fskChunks\":"); jf.print(g_casAutoAnalysis.fskChunks);
    jf.print(",\"pwmChunks\":"); jf.print(g_casAutoAnalysis.pwmChunks);
    jf.print(",\"maxBaud\":"); jf.print(g_casAutoAnalysis.maxBaud);
    jf.print(",\"stacSync55Blocks\":"); jf.print(g_casAutoAnalysis.stacSync55Blocks);
    jf.print(",\"cain217Blocks\":"); jf.print(g_casAutoAnalysis.cain217Blocks);
    jf.print(",\"profile\":\""); jf.print(jsonEscape(String(g_casAutoAnalysis.profile))); jf.print("\"");
    jf.print(",\"confidence\":\""); jf.print(jsonEscape(String(g_casAutoAnalysis.confidence))); jf.print("\"");
    jf.print("}");
    jf.close();
  }
}

// ===== F41: CAS profile manual overrides for Biblioteca =====
// Archivo persistente: /CONFIG/cas_profiles.json
// Formato simple y legible:
// {"version":1,"profiles":{"Archivo.cas":"NORMAL_600"}}
static const char CAS_PROFILE_OVERRIDES_PATH[] = "/CONFIG/cas_profiles.json";

static bool casProfileEnsureConfigDir() {
  if (!webAtrFsReady()) return false;
  File d = SPIFFS.open("/CONFIG", "r");
  if (d) {
    bool ok = d.isDirectory();
    d.close();
    if (ok) return true;
    SPIFFS.remove("/CONFIG");
  } else if (SPIFFS.exists("/CONFIG")) {
    SPIFFS.remove("/CONFIG");
  }
  if (!SPIFFS.mkdir("/CONFIG")) {
    delay(10);
    if (!SPIFFS.mkdir("/CONFIG")) return false;
  }
  File d2 = SPIFFS.open("/CONFIG", "r");
  bool ok = d2 && d2.isDirectory();
  if (d2) d2.close();
  return ok;
}

static String casProfileNormalize(String profile) {
  profile.trim();
  profile.toUpperCase();
  profile.replace("-", "_");
  profile.replace(" ", "_");
  if (!profile.length()) profile = "AUTO";
  return profile;
}

static bool casProfileAllowed(const String& profileIn) {
  String p = casProfileNormalize(profileIn);
  return p == "AUTO" ||
         p == "NORMAL_600" ||
         p == "IRON_STD_600" ||
         p == "TURBO_SOFTWARE" ||
         p == "TURBO_SOFTWARE_V2_800" ||
         p == "STAC_PROBABLE" ||
         p == "CAIN_217_PROBABLE" ||
         p == "INJEKTOR_4000" ||
         p == "INJEKTOR_6000" ||
         p == "TURBO_2000" ||
         p == "TURBO_2600" ||
         p == "KSO_T2000F" ||
         p == "COLD_PRISM_4000" ||
         p == "TURBO_6000_CHAOS" ||
         p == "PWM_TURBO" ||
         p == "FSK_ESPECIAL" ||
         p == "RAW_BIN" ||
         p == "UNKNOWN_TURBO" ||
         p == "PERSONALIZADO";
}

static int casProfileFindMatchingBrace(const String& s, int openPos) {
  if (openPos < 0 || openPos >= (int)s.length() || s[openPos] != '{') return -1;
  bool inString = false;
  bool esc = false;
  int level = 0;
  for (int i = openPos; i < (int)s.length(); i++) {
    char c = s[i];
    if (inString) {
      if (esc) { esc = false; continue; }
      if (c == '\\') { esc = true; continue; }
      if (c == '"') inString = false;
      continue;
    }
    if (c == '"') { inString = true; continue; }
    if (c == '{') level++;
    else if (c == '}') {
      level--;
      if (level == 0) return i;
      if (level < 0) return -1;
    }
  }
  return -1;
}

static void casProfileSkipWs(const String& s, int& i) {
  while (i < (int)s.length()) {
    char c = s[i];
    if (c == ' ' || c == '\r' || c == '\n' || c == '\t' || c == ',') i++;
    else break;
  }
}

static bool casProfileReadJsonStringAt(const String& s, int& i, String& out) {
  out = "";
  casProfileSkipWs(s, i);
  if (i >= (int)s.length() || s[i] != '"') return false;
  i++;
  bool esc = false;
  for (; i < (int)s.length(); i++) {
    char c = s[i];
    if (esc) {
      if (c == 'n') out += '\n';
      else if (c == 'r') out += '\r';
      else if (c == 't') out += '\t';
      else out += c;
      esc = false;
      continue;
    }
    if (c == '\\') { esc = true; continue; }
    if (c == '"') { i++; return true; }
    out += c;
  }
  return false;
}

static bool casProfileOverrideLoadAll(std::vector<CasProfileOverride>& list) {
  list.clear();
  if (!webAtrFsReady() || !SPIFFS.exists(CAS_PROFILE_OVERRIDES_PATH)) return true;
  File f = SPIFFS.open(CAS_PROFILE_OVERRIDES_PATH, "r");
  if (!f) return false;
  String txt = f.readString();
  f.close();
  txt.trim();
  if (!txt.length()) return true;

  int p = txt.indexOf("\"profiles\"");
  if (p < 0) return true;
  int open = txt.indexOf('{', p);
  int close = casProfileFindMatchingBrace(txt, open);
  if (open < 0 || close <= open) return false;

  int i = open + 1;
  while (i < close) {
    casProfileSkipWs(txt, i);
    if (i >= close) break;
    String file, profile;
    if (!casProfileReadJsonStringAt(txt, i, file)) break;
    casProfileSkipWs(txt, i);
    if (i >= close || txt[i] != ':') break;
    i++;
    if (!casProfileReadJsonStringAt(txt, i, profile)) break;
    file = webAtrSanitizeFileName(file);
    profile = casProfileNormalize(profile);
    if (file.length() && webCasLooksLikeName(file) && casProfileAllowed(profile) && profile != "AUTO") {
      CasProfileOverride o;
      o.file = file;
      o.profile = profile;
      list.push_back(o);
    }
  }
  return true;
}

static bool casProfileOverrideSaveAll(const std::vector<CasProfileOverride>& list) {
  if (!casProfileEnsureConfigDir()) return false;
  File f = SPIFFS.open(CAS_PROFILE_OVERRIDES_PATH, "w");
  if (!f) return false;
  f.print("{\"version\":1,\"build\":\"");
  f.print(jsonEscape(String(MASTER_BUILD)));
  f.print("\",\"profiles\":{");
  bool first = true;
  for (const CasProfileOverride& o : list) {
    if (!o.file.length() || !o.profile.length() || o.profile == "AUTO") continue;
    if (!first) f.print(',');
    first = false;
    f.print('"'); f.print(jsonEscape(o.file)); f.print("\":\"");
    f.print(jsonEscape(casProfileNormalize(o.profile)));
    f.print('"');
  }
  f.print("}}\n");
  f.flush();
  f.close();
  return true;
}

static bool casProfileOverrideGet(const String& rawFile, String& profileOut) {
  profileOut = "";
  String file = webAtrSanitizeFileName(rawFile);
  if (!file.length() || !webCasLooksLikeName(file)) return false;
  std::vector<CasProfileOverride> list;
  if (!casProfileOverrideLoadAll(list)) return false;
  for (const CasProfileOverride& o : list) {
    if (o.file.equalsIgnoreCase(file)) {
      profileOut = casProfileNormalize(o.profile);
      return profileOut.length() && profileOut != "AUTO";
    }
  }
  return false;
}

static bool casProfileOverrideSet(const String& rawFile, const String& profileIn, String& err, String& effectiveProfile) {
  err = "";
  effectiveProfile = "AUTO";
  String file = webAtrSanitizeFileName(rawFile);
  if (!file.length() || !webCasLooksLikeName(file)) { err = "Archivo CAS inválido"; return false; }
  String profile = casProfileNormalize(profileIn);
  if (!casProfileAllowed(profile)) { err = "Perfil CAS no permitido"; return false; }

  std::vector<CasProfileOverride> list;
  if (!casProfileOverrideLoadAll(list)) { err = "No se pudo leer cas_profiles.json"; return false; }

  int found = -1;
  for (int i = 0; i < (int)list.size(); i++) {
    if (list[i].file.equalsIgnoreCase(file)) { found = i; break; }
  }

  if (profile == "AUTO") {
    if (found >= 0) list.erase(list.begin() + found);
  } else {
    if (found >= 0) list[found].profile = profile;
    else {
      CasProfileOverride o;
      o.file = file;
      o.profile = profile;
      list.push_back(o);
    }
    effectiveProfile = profile;
  }

  if (!casProfileOverrideSaveAll(list)) { err = "No se pudo guardar /CONFIG/cas_profiles.json"; return false; }
  return true;
}

static String casProfileEffectiveForFile(const String& file, const CasAutoAnalysis* cached) {
  String overrideProfile;
  if (casProfileOverrideGet(file, overrideProfile)) return overrideProfile;
  if (cached && cached->valid && cached->profile[0]) return String(cached->profile);
  return String("AUTO");
}

static void casSetEffectiveProfileRuntime(const String& profile, bool manualOverride) {
  String p = casProfileNormalize(profile);
  if (!casProfileAllowed(p)) p = "AUTO";
  memset(g_casEffectiveProfile, 0, sizeof(g_casEffectiveProfile));
  strncpy(g_casEffectiveProfile, p.c_str(), sizeof(g_casEffectiveProfile) - 1);
  g_casEffectiveProfileManual = manualOverride && p != "AUTO";
  g_casProfileLockBaud = false;
  g_casProfileForcedBaud = 0;
}

static void casApplyEffectiveProfileForPlayback(const String& profile, bool manualOverride) {
  String p = casProfileNormalize(profile);
  casSetEffectiveProfileRuntime(p, manualOverride);

  g_casTurboMultiplierX100 = 100;
  g_casTurboEnabled = false;

  if (p == "NORMAL_600") {
    g_casUserBaseBaud = 600;
    g_casProfileLockBaud = true;
    g_casProfileForcedBaud = 600;
    return;
  }

  if (p == "IRON_STD_600") {
    g_casUserBaseBaud = 600;
    g_casProfileLockBaud = true;
    g_casProfileForcedBaud = 600;
    g_casIronTurboCompat = true;
    g_casIronTurboTurboBaud = 600;
    return;
  }

  if (p == "TURBO_SOFTWARE_V2_800" || p == "TURBO_2000" || p == "TURBO_2600" || p == "KSO_T2000F" || p == "COLD_PRISM_4000") {
    uint16_t b = g_casAutoAnalysis.initialBaud ? g_casAutoAnalysis.initialBaud : 600;
    g_casUserBaseBaud = casClampBaud(b);
    return;
  }

  if (p == "INJEKTOR_6000" || p == "TURBO_6000_CHAOS") {
    g_casUserBaseBaud = 6000;
    g_casProfileLockBaud = true;
    g_casProfileForcedBaud = 6000;
    return;
  }

  if (p == "STAC_PROBABLE" || p == "CAIN_217_PROBABLE" || p == "PWM_TURBO" || p == "FSK_ESPECIAL" || p == "RAW_BIN" || p == "UNKNOWN_TURBO" || p == "PERSONALIZADO") {
    uint16_t b = g_casAutoAnalysis.initialBaud ? g_casAutoAnalysis.initialBaud : 600;
    g_casUserBaseBaud = casClampBaud(b);
    return;
  }

  g_casUserBaseBaud = 600;
}



// ===== WEB-ATR FAST LOAD V28 =====
// Optimización encapsulada para lectura ATR -> Atari.
// No modifica ESP-NOW ni los timings SIO del RP2040.
#define WEB_ATR_FAST_LOAD_COMPILED 1
#define WEB_ATR_SECTOR_CACHE_COUNT 24
#define WEB_ATR_FAST_LOAD_DEFAULT true
#define WEB_ATR_READAHEAD_DEFAULT true

static bool     g_webAtrFastLoadEnabled = WEB_ATR_FAST_LOAD_DEFAULT;
static bool     g_webAtrReadAheadEnabled = false;
static uint32_t g_webAtrCacheClock = 0;
static uint32_t g_webAtrCacheHit = 0;
static uint32_t g_webAtrCacheMiss = 0;
static uint32_t g_webAtrCacheStore = 0;
static uint32_t g_webAtrReadAhead = 0;
static uint32_t g_webAtrReadAheadHit = 0;
static uint32_t g_webAtrReadAheadFail = 0;
static uint32_t g_webAtrSdReadUsLast = 0;
static uint32_t g_webAtrSdReadUsMax = 0;

struct WebAtrSectorCacheEntry {
  bool valid;
  bool readAhead;
  uint8_t dev;
  uint16_t sec;
  uint16_t len;
  uint32_t stamp;
  char name[64];
  uint8_t data[256];
};

static WebAtrSectorCacheEntry g_webAtrCache[WEB_ATR_SECTOR_CACHE_COUNT];


// ===== WEB-ATR LOAD PRIORITY V30 =====
// Mantiene el ATR abierto por unidad para evitar open/close por sector.
// También guarda metadata por unidad y expone diagnóstico fino.
#define WEB_ATR_READAHEAD_MAX_COUNT 4
#define WEB_ATR_READAHEAD_COUNT_DEFAULT 0
#define WEB_ATR_PRIORITY_WINDOW_MS 3000

static File     g_webAtrOpenFile[WEB_ATR_MAX_UNITS];
static bool     g_webAtrOpenValid[WEB_ATR_MAX_UNITS] = { false, false, false, false, false, false, false };
static String   g_webAtrOpenName[WEB_ATR_MAX_UNITS];
static String   g_webAtrOpenPath[WEB_ATR_MAX_UNITS];
static WebAtrMeta g_webAtrOpenMeta[WEB_ATR_MAX_UNITS];

static bool     g_webAtrPriorityMode = true;
static uint8_t  g_webAtrReadAheadCount = WEB_ATR_READAHEAD_COUNT_DEFAULT;
static uint32_t g_webAtrOpenReuse = 0;
static uint32_t g_webAtrOpenCount = 0;
static uint32_t g_webAtrOpenFail = 0;
static uint32_t g_webAtrCloseCount = 0;
static uint32_t g_webAtrOpenUsLast = 0;
static uint32_t g_webAtrSeekUsLast = 0;
static uint32_t g_webAtrDataReadUsLast = 0;
static uint32_t g_webAtrTotalReadUsLast = 0;
static uint32_t g_webAtrTotalReadUsMax = 0;
static uint32_t g_webAtrLastReadMs = 0;





// Reduce ruido de diagnóstico de impresora durante cargas WEB-ATR/BT/DISCO.
// Los contadores siguen disponibles, pero el log serial se limita para no ensuciar pruebas de lectura.
static unsigned long g_prnDiagLastLogMs = 0;
static uint32_t g_prnDiagLastStatus = 0;
static uint32_t g_prnDiagLastWrite = 0;
static uint32_t g_prnDiagLastWriteOk = 0;
static uint32_t g_prnDiagLastQSent = 0;
static uint32_t g_prnDiagLastQDrop = 0;
static uint32_t g_prnDiagLastTimeoutData = 0;
static uint32_t g_prnDiagLastTimeoutChk = 0;
static uint32_t g_prnDiagLastChecksumErr = 0;
static uint32_t g_prnDiagLastUnsupported = 0;

// Prototipos usados por el gateway BT/WEB-ATR antes de la implementación real.
void sendUartFrameToRP(const uint8_t* payload, uint8_t len);
static bool handleBtSio2pcFrame(uint8_t dev, const uint8_t* payload, uint8_t len);
void handleBtSio2pcSet();
void handleCasStatus();
void handleCasAnalyze();
void handleCasProfile();
void handleCasMount();
void handleCasUnmount();
void handleCasPlay();
void handleCasStop();
void handleCasRewind();
void handleCasDownload();
void serviceCasManualPlayback();
void handleCasStatusFromRP(const uint8_t* p, uint8_t len);
void handleCasAckFromRP(const uint8_t* p, uint8_t len);
void pollUartFromRP();
void casSendControlToRP(uint8_t cmd, uint32_t baud, uint32_t totalBytes);
static void casClosePlaybackFile();
void handleDriveVisibleSet();
void handleDriveVisibleStatus();
void handleDriveVisibleAdd();
void handleDriveVisibleRemove();
void saveWebAtrConfigToNvs();
void saveBtDiskConfigToNvs();
void sendTimingUpdateToRPThrottled(bool force = false);
static void serviceDeferredConfigSaves();

// F16: montaje instantáneo. Se responde al navegador primero y el guardado
// persistente en NVS se ejecuta después desde loop(), agrupando cambios rápidos.
static bool     g_webAtrConfigSavePending = false;
static bool     g_btDiskConfigSavePending = false;
static bool     g_casConfigSavePending = false;
static uint32_t g_deferredConfigSaveAtMs = 0;
static const uint32_t DEFERRED_CONFIG_SAVE_DELAY_MS = 2200UL;

static void markDeferredConfigSave(bool webAtr, bool btDisk, bool cas = false) {
  if (webAtr) g_webAtrConfigSavePending = true;
  if (btDisk) g_btDiskConfigSavePending = true;
  if (cas) g_casConfigSavePending = true;
  g_deferredConfigSaveAtMs = millis() + DEFERRED_CONFIG_SAVE_DELAY_MS;
}
static bool webAtrFsReady();
static void clearLastMasterOpLocal();
static bool webAtrReadMetaFromPath(const String& path, WebAtrMeta &m);
static String webAtrResolvedPathForIndex(int idx);
static bool webAtrReadMetaForIndex(int idx, WebAtrMeta &m, bool forceRefresh = false);
static void webAtrResetResolvedSlot(int idx);
static bool webAtrReadMetaForDev(uint8_t dev, WebAtrMeta &m);
static void webAtrRefreshPresence();
static void webAtrRefreshPresenceMask(uint8_t unitMask);
static bool webAtrOffsetForSector(uint16_t sec, const WebAtrMeta &m, uint32_t &off, uint16_t &len);
static bool webAtrReadSector(uint8_t dev, uint16_t sec, uint8_t *buf, uint16_t &len);
void handleAtrFastLoadSet();
void webAtrCloseOpenFileDev(uint8_t dev);
void webAtrCloseOpenFileIndex(int idx);
bool webAtrEnsureOpen(uint8_t dev, const String& mountedName, File **outFile, WebAtrMeta **outMeta);
bool webAtrAtariBusy();


static void webAtrBuildPercom(uint8_t dev, uint8_t out[PERCOM_BLOCK_LEN]);
static bool sendLocalNakToRP(uint8_t dev, const char* reason);
static bool sendLocalAckToRP(uint8_t dev, const char* src);
static void sendLocalSectorChunkToRP(uint8_t dev, uint16_t sec, const uint8_t* buf, uint16_t len);

// ========= Estado de SLAVES =========
struct SlaveInfo {
  bool present;
  bool supports256;
  uint8_t mac[6];
  unsigned long lastSeen;
  uint8_t physicalDev;
};

SlaveInfo slaves[4];


// ========= MAPEO MAC -> UNIDAD LOGICA (persistente) =========
// Selecciona en la WEB qué ESP32 SLAVE (por MAC) corresponde a D1..D4.
// Se guarda en NVS (namespace "xf551map").
#define MAX_DEVICES 8

struct DeviceEntry {
  bool used;
  bool present;
  bool supports256;
  uint8_t mac[6];
  unsigned long lastSeen;
  uint8_t physicalDev;   // DEV_MIN..DEV_MAX
};

struct MapEntry {
  uint8_t mac[6];
  uint8_t logic;         // DEV_MIN..DEV_MAX
};

static DeviceEntry g_devices[MAX_DEVICES];
static MapEntry    g_map[MAX_DEVICES];
static uint8_t     g_mapCount = 0;

// Cache rapido: lógico -> mac
static uint8_t g_logicMac[4][6];
static bool    g_logicMacValid[4] = { false, false, false, false };

static inline int devIdx(uint8_t dev) {
  if (dev < DEV_MIN || dev > DEV_MAX) return -1;
  return (int)(dev - DEV_MIN);
}

static void rebuildLogicMac() {
  for (int i = 0; i < 4; i++) {
    g_logicMacValid[i] = false;
    memset(g_logicMac[i], 0, 6);
  }
  for (int i = 0; i < (int)g_mapCount; i++) {
    int idx = devIdx(g_map[i].logic);
    if (idx >= 0) {
      memcpy(g_logicMac[idx], g_map[i].mac, 6);
      g_logicMacValid[idx] = true;
    }
  }
}

// ===== Helpers: evitar choques D1..D4 =====
// ¿El lógico 'logic' está reservado por otra MAC (por mapeo persistente)?
static bool logicReservedByOther(uint8_t logic, const uint8_t mac[6]) {
  if (logic < DEV_MIN || logic > DEV_MAX) return false;
  for (int i = 0; i < (int)g_mapCount; i++) {
    if (g_map[i].logic == logic) {
      if (memcmp(g_map[i].mac, mac, 6) != 0) return true;
    }
  }
  return false;
}

// Busca en slots D1..D4 qué slot tiene esta MAC (si está aplicada)
// [F24] Eliminado helper no usado: findSlaveSlotByMac
// Cuenta cuántos dispositivos están presentes (según tabla g_devices)
static int countPresentDevices() {
  int c = 0;
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (g_devices[i].used && g_devices[i].present) c++;
  }
  return c;
}

// Si hay 1 solo dispositivo presente, devuelve su MAC
static bool getOnlyPresentDeviceMac(uint8_t out[6]) {
  int found = -1;
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (g_devices[i].used && g_devices[i].present) {
      if (found != -1) return false;
      found = i;
    }
  }
  if (found == -1) return false;
  memcpy(out, g_devices[found].mac, 6);
  return true;
}



static bool parseMacString(const String& s, uint8_t out[6]) {
  int v[6];
  if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) != 6) return false;
  for (int i=0;i<6;i++) out[i] = (uint8_t)v[i];
  return true;
}

static uint8_t devFromStr(const String& s) {
  if (s.length()==2 && (s[0]=='D' || s[0]=='d') && s[1]>='1' && s[1]<='4') {
    return (uint8_t)(0x30 + (s[1]-'0')); // '1'->0x31 ...
  }
  return 0;
}

static int findDeviceSlotByMac(const uint8_t mac[6]) {
  for (int i=0;i<MAX_DEVICES;i++) {
    if (g_devices[i].used && memcmp(g_devices[i].mac, mac, 6)==0) return i;
  }
  return -1;
}

static int allocDeviceSlot(const uint8_t mac[6]) {
  int idx = findDeviceSlotByMac(mac);
  if (idx >= 0) return idx;

  for (int i=0;i<MAX_DEVICES;i++) {
    if (!g_devices[i].used) {
      g_devices[i].used = true;
      memcpy(g_devices[i].mac, mac, 6);
      g_devices[i].present = true;
      g_devices[i].supports256 = false;
      g_devices[i].lastSeen = millis();
      g_devices[i].physicalDev = 0;
      return i;
    }
  }
  return -1;
}

static int findMapIndex(const uint8_t mac[6]) {
  for (int i=0;i<(int)g_mapCount;i++) {
    if (memcmp(g_map[i].mac, mac, 6)==0) return i;
  }
  return -1;
}

static uint8_t mapGetLogic(const uint8_t mac[6]) {
  int i = findMapIndex(mac);
  if (i < 0) return 0;
  uint8_t d = g_map[i].logic;
  if (d < DEV_MIN || d > DEV_MAX) return 0;
  return d;
}

static void mapRemoveLogic(uint8_t logic) {
  for (int i=0;i<(int)g_mapCount;i++) {
    if (g_map[i].logic == logic) g_map[i].logic = 0; // desasignar
  }
}

static void saveMapToNvs() {
  if (!prefs.begin("xf551map", false)) return;
  prefs.putUChar("n", g_mapCount);
  prefs.putBytes("m", g_map, g_mapCount * sizeof(MapEntry));
  prefs.end();
}

static void loadMapFromNvs() {
  if (!prefs.begin("xf551map", true)) return;
  uint8_t n = prefs.getUChar("n", 0);
  if (n > MAX_DEVICES) n = MAX_DEVICES;
  g_mapCount = n;
  if (g_mapCount > 0) {
    size_t need = g_mapCount * sizeof(MapEntry);
    size_t got  = prefs.getBytes("m", g_map, need);
    if (got != need) g_mapCount = 0;
  }
  prefs.end();
  rebuildLogicMac();
}

static bool mapSetLogicForMac(const uint8_t mac[6], uint8_t logic) {
  if (logic < DEV_MIN || logic > DEV_MAX) return false;

  // si otra MAC ya ocupa ese lógico, la desasignamos
  mapRemoveLogic(logic);

  int idx = findMapIndex(mac);
  if (idx < 0) {
    if (g_mapCount >= MAX_DEVICES) return false;
    idx = g_mapCount++;
    memcpy(g_map[idx].mac, mac, 6);
  }
  g_map[idx].logic = logic;

  saveMapToNvs();
  rebuildLogicMac();
  return true;
}

static void clearSlaveSlot(int slotIdx) {
  slaves[slotIdx].present = false;
  slaves[slotIdx].supports256 = false;
  memset(slaves[slotIdx].mac, 0, 6);
  slaves[slotIdx].lastSeen = 0;
  slaves[slotIdx].physicalDev = 0;
}

static void clearSlotsByMac(const uint8_t mac[6]) {
  for (int i=0;i<4;i++) {
    if (memcmp(slaves[i].mac, mac, 6)==0) clearSlaveSlot(i);
  }
}

static void applyDeviceToLogicalSlot(const uint8_t mac[6], uint8_t logic, bool supports256, uint8_t physDev) {
  int s = devIdx(logic);
  if (s < 0) return;

  slaves[s].present = true;
  slaves[s].supports256 = supports256;
  memcpy(slaves[s].mac, mac, 6);
  slaves[s].lastSeen = millis();
  slaves[s].physicalDev = physDev;
}


// Prefetch configurado por unidad - 1 sector es óptimo para XF551
uint8_t prefetchCfg[4] = { 3, 3, 3, 3 };

// ========= Tiempos SIO (µs) – ULTRA OPTIMIZADOS =========
uint16_t T_ACK_TO_COMPLETE = 220;    // perfil más ágil por defecto
uint16_t T_COMPLETE_TO_DATA = 140;   // perfil más ágil por defecto
uint16_t T_DATA_TO_CHK = 25;         // perfil más ágil por defecto
uint16_t T_CHUNK_DELAY = 80;         // perfil más ágil por defecto

// ========= Tiempos medidos por disquetera (ACK/NAK) =========
struct DriveTiming {
  uint32_t lastAckMs;
  uint32_t avgAckMs;
  bool autoEnabled;
};
DriveTiming g_driveTiming[4];

int g_autoProfile = 2;

// ========= WebServer & NVS =========
WebServer server(80);
// Preferences prefs;  // (declarado arriba)
const uint32_t CFG_MAGIC = 0xCAFEBABF;

// ========= Comm/Verify CFG (NVS/UI) =========
static const uint32_t BOOT_UART_BAUD = 460800;     // RP y Master arrancan acá
uint32_t CFG_UART_BAUD    = 460800;                // UART RP<->MASTER (configurable)
uint32_t CFG_RP_SIO_BAUD  = 19200;                 // SIO Atari<->RP (configurable)
uint32_t CFG_XF_SIO_BAUD  = 19200;                 // SIO ESP32<->XF551 (configurable)
uint32_t CFG_NET_DELAY_US = 0;                     // throttle ESP-NOW (µs)
uint8_t  CFG_VERIFY_FLAGS = 0b1110;                // all=0 boot=1 vtoc=1 verify57=1

// ========= Printer 820 / Atari P: CFG (NVS/UI) =========
enum PrinterOutputMode : uint8_t {
  PRN_MODE_RAW9100 = 1,
  PRN_MODE_HTTP    = 2,   // Gateway Windows HTTP
  PRN_MODE_BT      = 3,
  PRN_MODE_IPP_JPEG = 4   // IPP directo usando JPEG generado desde ATASCII
};

enum VirtualPrinterProfileId : uint8_t {
  VP_ATARI_820_40 = 0,
  VP_ATASCII_80   = 1,
  VP_ATASCII_132  = 2
};

enum VirtualPrinterFontId : uint8_t {
  VF_ATASCII_ORIGINAL_8X8   = 0,
  VF_ATASCII_MEJORADA_12X16 = 1  // legacy: se migra a 8x8 en V64
};

enum VirtualPrinterOrientation : uint8_t {
  VP_ORIENT_AUTO      = 0,
  VP_ORIENT_PORTRAIT  = 1,
  VP_ORIENT_LANDSCAPE = 2
};

enum VirtualPrinterPaperSize : uint8_t {
  VP_PAPER_A4 = 0,
  VP_PAPER_LETTER = 1
};

enum PrinterCompositionMode : uint8_t {
  PRN_COMPOSE_FIEL_820 = 0,      // cada WRITE recibido desde P: = una línea física
  PRN_COMPOSE_EXTENDIDA = 1      // une registros de 40/29 hasta completar ancho útil
};

static const char* printerCompositionModeName(uint8_t id) {
  switch (id) {
    case PRN_COMPOSE_FIEL_820: return "Atari 820 fiel";
    case PRN_COMPOSE_EXTENDIDA:
    default: return "Composición extendida";
  }
}



enum PrinterRenderQuality : uint8_t {
  PRN_RENDER_TEXT_SHARP = 0,   // Legacy: no usado en V120 JPEG_ONLY
  PRN_RENDER_COMPAT_JPEG = 1   // JPEG estable para compatibilidad
};

static const char* printerRenderQualityName(uint8_t q) {
  return "Compatibilidad JPEG estable";
}

static const char* virtualFontName(uint8_t id) {
  switch (id) {
    case VF_ATASCII_MEJORADA_12X16: return "ATASCII matriz 8x8";
    case VF_ATASCII_ORIGINAL_8X8:
    default: return "ATASCII matriz 8x8";
  }
}

struct VirtualPrinterProfile {
  uint8_t id;
  const char* name;
  uint16_t columns;
  uint16_t rows;
  uint8_t scaleX;
  uint8_t scaleY;
};

static const VirtualPrinterProfile VIRTUAL_PRINTER_PROFILES[] = {
  // Perfiles pensados para la impresora actual en hoja A4.
  // Las columnas/filas efectivas se calculan automáticamente según orientación y tamaño de letra.
  { VP_ATARI_820_40, "Legible",    43, 61, 2, 2 },
  { VP_ATASCII_80,   "Compacto",   87, 61, 1, 2 },
  { VP_ATASCII_132,  "Condensado", 129, 84, 1, 1 }
};

static const uint8_t VIRTUAL_PRINTER_PROFILE_COUNT = sizeof(VIRTUAL_PRINTER_PROFILES) / sizeof(VIRTUAL_PRINTER_PROFILES[0]);
static const uint16_t PRN_VIRTUAL_MAX_RENDER_ROWS = 160; // V81: permite hoja completa y auto-fit a 8pt
static const uint16_t PRN_VIRTUAL_MAX_FLUSH_ROWS  = 160; // V81: no cortar prematuramente; fallback solo si realmente falta memoria


// IMPORTANTE Arduino IDE:
// Evitamos que una función retorne VirtualPrinterProfile& porque el preprocesador
// de Arduino puede generar prototipos antes de ver el struct y falla con:
// 'VirtualPrinterProfile' does not name a type.
// Esta función retorna solo el índice; luego se accede al perfil desde la tabla.
static uint8_t getVirtualPrinterProfileIndex(uint8_t id) {
  for (uint8_t i = 0; i < VIRTUAL_PRINTER_PROFILE_COUNT; i++) {
    if (VIRTUAL_PRINTER_PROFILES[i].id == id) return i;
  }
  return 0;
}

// V69:
// El corte de página ya no es manual: se calcula automáticamente con el alto útil de la hoja.
// [F24] Eliminado helper no usado: getVirtualPrinterFlushRows
struct PrinterConfig {
  bool enabled;
  uint8_t sioDev;       // 0x40..0x43
  uint8_t mode;         // PrinterOutputMode
  char ip[32];
  uint16_t port;
  char gateway[128];
  char name[48];
  bool atasciiToAscii;
  bool appendCrLf;
  bool cut40;
  uint8_t virtualProfile; // 0=40 columnas, 1=80 columnas, 2=132 columnas
  uint8_t virtualFont;    // 0=ATASCII matriz 8x8
  uint16_t customColumns; // legado V68: se conserva solo para migración NVS; no se usa en V69
  uint16_t customRows;    // legado V68: se conserva solo para migración NVS; no se usa en V69
  uint8_t fontScale;      // 0=auto por perfil, o tamaño de letra 8/10/12/14/16 pt
  uint8_t pageOrientation;// 0=auto, 1=vertical, 2=horizontal
  uint8_t paperSize;      // 0=A4, 1=Carta/Letter
  uint8_t composeMode;    // 0=Atari 820 fiel, 1=composición extendida (default V73)
  uint8_t renderQuality;  // V120: fijo en compatibilidad JPEG estable
  bool autoPrintSpool;    // V80: imprimir automaticamente al quedar inactivo el spool
  uint16_t autoPrintDelayMs; // V80: tiempo sin datos para considerar fin de documento

  // Para impresoras que aceptan RAW 9100, pero no expulsan la hoja
  // hasta recibir Form Feed (0x0C). Activado por defecto solo para prueba.
  bool rawTestFormFeed;
  uint16_t rawCloseDelayMs;

  // Wi-Fi STA opcional para llegar a la impresora en la red local.
  // Se mantiene separado del SoftAP XF551_MASTER.
  bool staEnabled;
  char staSsid[33];
  char staPass[65];
};

PrinterConfig PRN_CFG = {
  true,
  0x40,
  PRN_MODE_HTTP,
  "192.168.1.120",
  631,
  "http://192.168.1.50:7077",
  "Brother DCP-T720DW",
  true,
  true,
  true,
  VP_ATARI_820_40,
  VF_ATASCII_ORIGINAL_8X8,
  0,        // customColumns: legado, sin uso en V69
  0,        // customRows: legado, sin uso en V69
  12,       // fontScale: 12 pt por defecto
  VP_ORIENT_AUTO,
  VP_PAPER_A4,    // paperSize: A4 por defecto
  PRN_COMPOSE_EXTENDIDA, // composeMode: default V73
  PRN_RENDER_COMPAT_JPEG, // renderQuality: V120 JPEG_ONLY
  true,     // autoPrintSpool: default V80
  5000,     // autoPrintDelayMs: 5 segundos
  true,     // rawTestFormFeed
  500,      // rawCloseDelayMs
  false,
  "",
  ""
};

static uint32_t g_prnLinesRx  = 0;
static uint32_t g_prnLinesOk  = 0;
static uint32_t g_prnLinesErr = 0;
static String   g_prnLastText = "";
static String   g_prnLastError = "";
static uint32_t g_prnLastBytes = 0;

// v19: buffer de página para Gateway HTTP.
// Evita imprimir una hoja por cada PRINT #.
static String   g_prnHttpBuffer = "";
static uint32_t g_prnHttpBufferLines = 0;
static uint32_t g_prnHttpLastLineMs = 0;
static bool     g_prnHttpPending = false;
static const uint32_t PRN_HTTP_FLUSH_IDLE_MS = 50000;   // v28: margen por tiempo
static const uint32_t PRN_HTTP_FLUSH_MAX_LINES = 5; // v28: flush inmediato al juntar 5 líneas

// Spool lógico ATASCII para modo IPP/JPEG.
// V80: primero se almacena el trabajo; luego la impresión consume una página, genera JPEG, imprime, libera y continúa.
static std::vector<uint8_t> g_prnVirtualBuffer;
static uint32_t g_prnVirtualLines = 0;
static uint32_t g_prnVirtualLastMs = 0;
static bool     g_prnVirtualPending = false;
static bool     g_prnVirtualPageFullLatched = false;
static const uint32_t PRN_VIRTUAL_FLUSH_IDLE_MS = 5000;
// V52: para probar datos reales desde Atari P:, no imprimimos por timeout.
// La hoja se libera con el botón web "Imprimir buffer Atari" o cuando se llena la página.
static const bool PRN_VIRTUAL_AUTO_FLUSH_IDLE = false;

// V71: impresión manual encolada. El botón web responde primero y el render JPEG
// se ejecuta después en loop(), liberando el cliente HTTP/página antes de usar heap.
static volatile bool g_prnManualPrintRequested = false;
static volatile bool g_prnAutoPrintRequested = false;
static bool g_prnAutoPrintBlockedAfterError = false; // V90: evita loop infinito si el auto-print falla hasta que llegue texto nuevo o impresión manual
static bool g_prnManualPrintBusy = false;
static uint16_t g_prnManualLastPages = 0;
static uint32_t g_prnSpoolJobs = 0;
static uint32_t g_prnSpoolPagesPrinted = 0;
static uint16_t g_prnJobPageIndex = 0;          // V96: pagina actual del trabajo
// V97: paginación fija; spool Atari fuerza JPEG estable en todas las páginas.
// Todas las páginas usan exactamente el snapshot elegido al inicio del trabajo.
static uint32_t g_prnSpoolLastPageBytes = 0;
static uint32_t g_prnSpoolLastJpegBytes = 0;

static uint32_t g_prnDiagStatus = 0;
static uint32_t g_prnDiagWrite = 0;
static uint32_t g_prnDiagWriteOk = 0;
static uint32_t g_prnDiagQSent = 0;
static uint32_t g_prnDiagQDrop = 0;
static uint32_t g_prnDiagTimeoutData = 0;
static uint32_t g_prnDiagTimeoutChk = 0;
static uint32_t g_prnDiagChecksumErr = 0;
static uint32_t g_prnDiagUnsupported = 0;

static String printerWifiStatusText() {
  wl_status_t st = WiFi.status();
  switch (st) {
    case WL_CONNECTED: return "CONNECTED";
    case WL_NO_SSID_AVAIL: return "NO_SSID";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    case WL_IDLE_STATUS: return "IDLE";
    default: return String((int)st);
  }
}

static uint32_t g_currentUartBaud = BOOT_UART_BAUD;

// ===== Estado BT-DISK-GATEWAY =====
static bool     g_btDiskUartReady = false;
static bool     g_btDiskOnline = false;
static uint32_t g_btDiskLastSeenMs = 0;
static uint32_t g_btDiskFramesRx = 0;
static uint32_t g_btDiskFramesTx = 0;
static uint32_t g_btDiskBadChk = 0;
static uint32_t g_btDiskBadLen = 0;
static uint32_t g_btDiskCmdRouted = 0;
static uint32_t g_btDiskChunksRx = 0;
static uint32_t g_btDiskLastCmdMs = 0;
static uint8_t  g_btDiskLastCmd = 0;
static uint8_t  g_btDiskLastDev = 0;
static uint16_t g_btDiskLastSec = 0;
static uint32_t g_btDiskLastHelloLogMs = 0;
static uint32_t g_btDiskHelloIgnoredDuringOp = 0;
static uint8_t  BT_FAKE_MAC[6] = { 0x42, 0x54, 0x2D, 0x44, 0x31, 0x00 };

// ACK de CFG desde RP
static volatile bool g_rpCfgAck = false;
static uint8_t  g_rpCfgAck_ok = 0;
static uint32_t g_rpCfgAck_uart = 0;
static uint32_t g_rpCfgAck_sio  = 0;

// ========= HTML UI =========
// V64: HTML/JS vive en IndexHtml.h y tipos JPEG viven en JpegTypes.h para evitar errores del preprocesador Arduino.
#include "IndexHtml.h"


// ========= Utils =========
void logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

uint8_t calcChecksum(const uint8_t* buf, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++) s += buf[i];
  return (uint8_t)s;
}

uint8_t calcSioChecksum(const uint8_t* buf, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++) {
    s += buf[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;
  }
  return (uint8_t)(s & 0xFF);
}

void ensurePeer(const uint8_t* mac) {
  if (!mac) return;
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CHANNEL;
  p.encrypt = false;
  esp_err_t e = esp_now_add_peer(&p);
  if (e != ESP_OK) {
    logf("[ESPNOW] esp_now_add_peer error=%d", (int)e);
  }
}

int devIndex(uint8_t dev) {
  if (dev < DEV_MIN || dev > DEV_MAX) return -1;
  return dev - DEV_MIN;
}

static int driveUiIndex(uint8_t dev) {
  if (dev < 0x31 || dev > (0x31 + DRIVE_UI_MAX_UNITS - 1)) return -1;
  return (int)(dev - 0x31);
}

static bool driveUiVisibleIndex(int idx) {
  if (idx < 0 || idx >= DRIVE_UI_MAX_UNITS) return false;
  if (idx < 4) return true;  // D1-D4 siempre visibles
  return (DRIVE_VISIBLE_MASK & (1u << idx)) != 0;
}

// [F24] Eliminado helper no usado: driveUiVisibleDev
static uint8_t driveUiClampMask(uint8_t m) {
  // D1-D4 nunca se ocultan. D5-D7 son opcionales.
  return (uint8_t)((m | DRIVE_UI_DEFAULT_VISIBLE_MASK) & DRIVE_UI_MAX_MASK);
}

static bool optionalDriveBusyIndex(int idx) {
  if (idx < 4 || idx >= DRIVE_UI_MAX_UNITS) return true;
  const uint8_t bit = (uint8_t)(1u << idx);
  return ((WEB_ATR_DEV_MASK & bit) != 0) ||
         ((BT_SIO2PC_DEV_MASK & bit) != 0) ||
         (g_webAtrMountedName[idx].length() > 0);
}

static String optionalDriveBusyReasonIndex(int idx) {
  if (idx < 4 || idx >= DRIVE_UI_MAX_UNITS) return String("unidad fija");
  const uint8_t bit = (uint8_t)(1u << idx);
  if ((WEB_ATR_DEV_MASK & bit) != 0 || g_webAtrMountedName[idx].length() > 0) return String("WEB-ATR montado o activo");
  if ((BT_SIO2PC_DEV_MASK & bit) != 0) return String("BT-SIO2PC activo");
  return String("");
}

static int nextHiddenOptionalDriveUnit() {
  for (int idx = 4; idx < DRIVE_UI_MAX_UNITS; idx++) {
    if (!driveUiVisibleIndex(idx)) return idx + 1;
  }
  return 0;
}

static int lastRemovableOptionalDriveUnit() {
  for (int idx = DRIVE_UI_MAX_UNITS - 1; idx >= 4; idx--) {
    if (driveUiVisibleIndex(idx) && !optionalDriveBusyIndex(idx)) return idx + 1;
  }
  return 0;
}

static void persistDriveUiState() {
  saveWebAtrConfigToNvs();
  saveBtDiskConfigToNvs();
}

const char* devName(uint8_t dev) {
  static const char* names[] = { "D1", "D2", "D3", "D4", "D5", "D6", "D7" };
  int idx = driveUiIndex(dev);
  return (idx >= 0) ? names[idx] : "UNK";
}

uint8_t prefetchForDev(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx < 0) return 0;
  return prefetchCfg[idx];
}

String formatMac(const uint8_t mac[6]) {
  char buf[32];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

const uint8_t* slaveMac() {
  return g_haveSlave ? g_lastSlave : BCAST_MAC;
}

const uint8_t* macForDev(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx < 0) return slaveMac();

  // 1) Si hay mapeo lógico->MAC, SIEMPRE usamos esa MAC
  if (g_logicMacValid[idx]) return g_logicMac[idx];

  // 2) fallback: lo último visto en ese slot lógico
  if (slaves[idx].present) return slaves[idx].mac;

  // 3) si hay 1 solo equipo presente, mejor unicast que broadcast
  static uint8_t only[6];
  if (getOnlyPresentDeviceMac(only)) return only;

  // 4) último slave conocido o broadcast
  return slaveMac();
}


static inline void throttleNet() {
  if (CFG_NET_DELAY_US > 0) {
    uint32_t d = CFG_NET_DELAY_US;
    if (d > 20000) d = 20000;
    delayMicroseconds(d);
  }
}

bool sendEspNow(const uint8_t* mac, const uint8_t* data, int len) {
  ensurePeer(mac);
  bool ok = (esp_now_send(mac, data, len) == ESP_OK);
  throttleNet();
  return ok;
}

bool sendEspToSlave(uint8_t dev, const uint8_t* data, int len) {
  const uint8_t* mac = macForDev(dev);
  return sendEspNow(mac, data, len);
}

static inline bool btDiskDevSelected(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx < 0) return false;
  return BT_DISK_ENABLED && ((BT_DISK_DEV_MASK & (1u << idx)) != 0);
}

static inline bool btDiskDevForced(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx < 0) return false;
  return BT_DISK_ENABLED && ((BT_DISK_FORCE_MASK & (1u << idx)) != 0);
}

static inline bool btDiskDevEnabled(uint8_t dev) {
  return btDiskDevSelected(dev) && g_btDiskOnline;
}

static inline bool btSio2pcDevSelected(uint8_t dev) {
  int idx = driveUiIndex(dev);
  if (idx < 0) return false;
  return driveUiVisibleIndex(idx) && BT_SIO2PC_ENABLED && ((BT_SIO2PC_DEV_MASK & (1u << idx)) != 0);
}

static inline bool btSio2pcDevForced(uint8_t dev) {
  int idx = driveUiIndex(dev);
  if (idx < 0) return false;
  return driveUiVisibleIndex(idx) && BT_SIO2PC_ENABLED && ((BT_SIO2PC_FORCE_MASK & (1u << idx)) != 0);
}

static inline bool btSio2pcDevEnabled(uint8_t dev) {
  return btSio2pcDevSelected(dev) && g_btDiskUartReady;
}

bool sendBtDiskFrame(const uint8_t* payload, uint8_t len) {
  if (!g_btDiskUartReady || !payload || len == 0) return false;
  uint8_t chk = calcChecksum(payload, len);
  SerialBTDisk.write(UART_SYNC);
  SerialBTDisk.write(len);
  SerialBTDisk.write(payload, len);
  SerialBTDisk.write(chk);
  SerialBTDisk.flush();
  g_btDiskFramesTx++;
  return true;
}

static void markBtDiskOnline(uint8_t dev, bool supports256) {
  (void)supports256;
  int idx = devIndex(dev);
  if (idx < 0) idx = 0;
  g_btDiskOnline = true;
  g_btDiskLastSeenMs = millis();
  BT_FAKE_MAC[4] = (uint8_t)('1' + idx);
  // Importante: NO tocamos slaves[] aquí.
  // Si se escribiera una MAC falsa en slaves[], el fallback físico podría intentar ESP-NOW
  // hacia una MAC inexistente cuando el host BT se desconecta. La ruta BT se decide
  // solamente por BT_DISK_* + g_btDiskOnline.
}



static int webAtrUnitIndex(uint8_t dev) {
  return driveUiIndex(dev);
}

static bool webAtrSelected(uint8_t dev) {
  int idx = webAtrUnitIndex(dev);
  if (idx < 0) return false;
  return driveUiVisibleIndex(idx) && WEB_ATR_ENABLED && ((WEB_ATR_DEV_MASK & (1u << idx)) != 0);
}

static bool webAtrForced(uint8_t dev) {
  int idx = webAtrUnitIndex(dev);
  if (idx < 0) return false;
  return driveUiVisibleIndex(idx) && WEB_ATR_ENABLED && ((WEB_ATR_FORCE_MASK & (1u << idx)) != 0);
}

static void normalizeDriveMasks() {
  DRIVE_VISIBLE_MASK = driveUiClampMask(DRIVE_VISIBLE_MASK);
  WEB_ATR_DEV_MASK &= DRIVE_UI_MAX_MASK;
  WEB_ATR_FORCE_MASK &= DRIVE_UI_MAX_MASK;
  BT_SIO2PC_DEV_MASK &= DRIVE_UI_MAX_MASK;
  BT_SIO2PC_FORCE_MASK &= DRIVE_UI_MAX_MASK;
  BT_SIO2PC_FORCE_MASK &= BT_SIO2PC_DEV_MASK;
  WEB_ATR_FORCE_MASK &= WEB_ATR_DEV_MASK;
  if ((BT_SIO2PC_DEV_MASK & DRIVE_UI_MAX_MASK) == 0) BT_SIO2PC_ENABLED = false;
  if ((WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK) == 0) WEB_ATR_ENABLED = false;
}

static void clearBtSio2pcUnits(uint8_t mask) {
  mask &= DRIVE_UI_MAX_MASK;
  BT_SIO2PC_DEV_MASK &= (uint8_t)~mask;
  BT_SIO2PC_FORCE_MASK &= (uint8_t)~mask;
  if ((BT_SIO2PC_DEV_MASK & DRIVE_UI_MAX_MASK) == 0) BT_SIO2PC_ENABLED = false;
}

static void clearWebAtrUnits(uint8_t mask, bool clearMountedName) {
  mask &= DRIVE_UI_MAX_MASK;
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    if ((mask & (1u << i)) && clearMountedName) { g_webAtrMountedName[i] = ""; webAtrResetResolvedSlot(i); webAtrCacheInvalidateDev((uint8_t)(0x31 + i)); }
  }
  WEB_ATR_DEV_MASK &= (uint8_t)~mask;
  WEB_ATR_FORCE_MASK &= (uint8_t)~mask;
  if ((WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK) == 0) WEB_ATR_ENABLED = false;
}


static String webAtrFsDiagString() {
  uint64_t total = SPIFFS.totalBytes();
  uint64_t used  = SPIFFS.usedBytes();
  uint64_t freeB = (total > used) ? (total - used) : 0;

  String out;
  out.reserve(96);
  out += "STORAGE total=" + String((unsigned long)total);
  out += " used=" + String((unsigned long)used);
  out += " free=" + String((unsigned long)freeB);
  return out;
}

static uint32_t webAtrFsFreeBytes() {
  uint64_t total = SPIFFS.totalBytes();
  uint64_t used  = SPIFFS.usedBytes();
  return (total > used) ? (uint32_t)(total - used) : 0;
}


static String webAtrSanitizeFileName(const String& in) {
  String base = in;
  base.replace("\\", "/");
  int slash = base.lastIndexOf('/');
  if (slash >= 0) base = base.substring(slash + 1);
  String out;
  out.reserve(48);
  for (size_t i = 0; i < base.length(); i++) {
    char c = base[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
        c == '.' || c == '_' || c == '-') {
      out += c;
    } else if (c == ' ') {
      out += '_';
    }
  }
  out.trim();
  if (out.length() == 0) out = "ATR.ATR";

  // V12 ORIGINAL_NAMES:
  // - Los ATR se guardan como .ATR/.atr.
  // - Los XEX/COM/EXE se conservan con su extension original visible en biblioteca.
  //   Internamente el contenido guardado es una imagen ATR DOS 2.x/2.5 montable,
  //   pero el usuario ve y monta el nombre original: JUEGO.XEX, no JUEGO.XEX.ATR.
  String lower = out;
  lower.toLowerCase();
  bool knownExt = lower.endsWith(".atr") || lower.endsWith(".xex") || lower.endsWith(".com") || lower.endsWith(".exe") || lower.endsWith(".bas") || lower.endsWith(".cas") || lower.endsWith(".sec");
  if (!knownExt) out += ".ATR";

  const size_t maxStoredLen = 25;
  if (out.length() > maxStoredLen) {
    int dot = out.lastIndexOf('.');
    String ext = (dot >= 0) ? out.substring(dot) : String(".ATR");
    String stem = (dot >= 0) ? out.substring(0, dot) : out;
    size_t keep = (maxStoredLen > ext.length()) ? (maxStoredLen - ext.length()) : maxStoredLen;
    out = stem.substring(0, keep) + ext;
  }
  return out;
}

static String webAtrFindExistingInDir(const char* dirPath, const String& storedName) {
  String wanted = webAtrSanitizeFileName(storedName);
  File root = SPIFFS.open(dirPath, "r");
  if (!root) return String("");
  if (!root.isDirectory()) { root.close(); return String(""); }

  File f = root.openNextFile();
  while (f) {
    String path = f.name();
    bool isDir = f.isDirectory();
    f.close();

    if (!isDir) {
      path.replace("\\", "/");
      int slash = path.lastIndexOf('/');
      String actual = (slash >= 0) ? path.substring(slash + 1) : path;
      String actualClean = webAtrSanitizeFileName(actual);
      if (actual.equalsIgnoreCase(storedName) || actualClean.equalsIgnoreCase(wanted)) {
        root.close();
        String base(dirPath);
        if (base == "/") return String("/") + actual;
        if (!base.endsWith("/")) base += "/";
        return base + actual;
      }
    }
    f = root.openNextFile();
  }
  root.close();
  return String("");
}


static bool webAtrLibraryScanSkipDir(String path) {
  path.replace("\\", "/");
  if (!path.length()) return true;
  if (!path.startsWith("/")) path = String("/") + path;

  String up = path;
  up.toUpperCase();
  if (up == "/") return false;

  // F42Q: no recorrer carpetas ocultas/cache. En F42P el diagnóstico entró a
  // /CAS/.cache y abortó durante el recorrido. Para Biblioteca solo deben
  // considerarse carpetas de juegos/ATR/CAS, no cachés ni carpetas del sistema.
  int start = 1;
  while (start < path.length()) {
    int slash = path.indexOf('/', start);
    String part = (slash >= 0) ? path.substring(start, slash) : path.substring(start);
    if (part.length()) {
      String partUp = part;
      partUp.toUpperCase();
      if (part[0] == '.') return true;          // .cache, .Trash, .Spotlight...
      if (partUp == "$RECYCLE.BIN") return true;
      if (partUp.startsWith("FOUND.")) return true;
      if (partUp == "RECYCLED") return true;
      if (partUp == "TRASH") return true;
    }
    if (slash < 0) break;
    start = slash + 1;
  }

  if (up == "/CONFIG" || up.startsWith("/CONFIG/")) return true;
  if (up == "/GLYPHS" || up.startsWith("/GLYPHS/")) return true;
  if (up == "/PRINT" || up.startsWith("/PRINT/")) return true;
  if (up == "/TMP" || up.startsWith("/TMP/")) return true;
  if (up == "/COVERS" || up.startsWith("/COVERS/")) return true;
  if (up == "/MINI_COVERS" || up.startsWith("/MINI_COVERS/")) return true;
  if (up == "/SYSTEM VOLUME INFORMATION" || up.startsWith("/SYSTEM VOLUME INFORMATION/")) return true;
  return false;
}

static String webAtrFindExistingRecursive(const char* dirPath, const String& storedName, uint8_t depth = 0) {
  if (depth > 6) return String("");
  String baseDir(dirPath);
  if (baseDir.length() == 0) baseDir = "/";
  if (webAtrLibraryScanSkipDir(baseDir)) return String("");
  String wanted = webAtrSanitizeFileName(storedName);
  File root = SPIFFS.open(baseDir, "r");
  if (!root) return String("");
  if (!root.isDirectory()) { root.close(); return String(""); }

  File f = root.openNextFile();
  while (f) {
    String path = f.name();
    path.replace("\\", "/");
    bool isDir = f.isDirectory();
    uint32_t fSize = isDir ? 0 : (uint32_t)f.size();
    f.close();

    if (path.length() && !path.startsWith("/")) {
      String prefix = baseDir;
      if (!prefix.endsWith("/")) prefix += "/";
      path = prefix + path;
    }

    if (isDir) {
      if (!webAtrLibraryScanSkipDir(path)) {
        String found = webAtrFindExistingRecursive(path.c_str(), storedName, depth + 1);
        if (found.length()) { root.close(); return found; }
      }
    } else {
      int slash = path.lastIndexOf('/');
      String actual = (slash >= 0) ? path.substring(slash + 1) : path;
      String actualClean = webAtrSanitizeFileName(actual);
      if (actual.equalsIgnoreCase(storedName) || actualClean.equalsIgnoreCase(wanted)) {
        root.close();
        return path;
      }
    }
    f = root.openNextFile();
  }
  root.close();
  return String("");
}

static String webAtrPathForName(const String& storedName) {
  if (storedName.length() == 0) return String("");
  if (storedName.startsWith("/")) return storedName;
#if WEB_STORAGE_USE_SD
  // En SD los ATR deben quedar principalmente en /ATR, pero también
  // aceptamos /LIBRARY y raíz para archivos copiados manualmente.
  // Además tolera archivos con espacios copiados desde PC, comparando
  // contra el nombre sanitizado que usa la web.
  String p;
  String lowerStored = storedName;
  lowerStored.toLowerCase();
  if (lowerStored.endsWith(".cas")) {
    p = webAtrFindExistingInDir("/CAS", storedName);
    if (p.length()) return p;
    p = webAtrFindExistingInDir("/LIBRARY", storedName);
    if (p.length()) return p;
    p = webAtrFindExistingInDir("/", storedName);
    if (p.length()) return p;
    p = webAtrFindExistingRecursive("/", storedName);
    if (p.length()) return p;
    return String("/CAS/") + webAtrSanitizeFileName(storedName);
  }
  p = webAtrFindExistingInDir("/ATR", storedName);
  if (p.length()) return p;
  p = webAtrFindExistingInDir("/LIBRARY", storedName);
  if (p.length()) return p;
  p = webAtrFindExistingInDir("/", storedName);
  if (p.length()) return p;
  p = webAtrFindExistingRecursive("/", storedName);
  if (p.length()) return p;
  return String("/ATR/") + webAtrSanitizeFileName(storedName);
#else
  return String(WEB_ATR_PREFIX) + storedName;
#endif
}

static void webAtrResetResolvedSlot(int idx) {
  if (idx < 0 || idx >= WEB_ATR_MAX_UNITS) return;
  g_webAtrMountedPath[idx] = "";
  memset(&g_webAtrMountedMeta[idx], 0, sizeof(g_webAtrMountedMeta[idx]));
  g_webAtrSlotPresent[idx] = false;
}

static String webAtrResolvedPathForIndex(int idx) {
  if (idx < 0 || idx >= WEB_ATR_MAX_UNITS) return String("");
  if (g_webAtrMountedName[idx].length() == 0) {
    webAtrResetResolvedSlot(idx);
    return String("");
  }
  if (g_webAtrMountedPath[idx].length() == 0) {
    g_webAtrMountedPath[idx] = webAtrPathForName(g_webAtrMountedName[idx]);
  }
  return g_webAtrMountedPath[idx];
}

static bool webAtrReadMetaForIndex(int idx, WebAtrMeta &m, bool forceRefresh) {
  memset(&m, 0, sizeof(m));
  if (idx < 0 || idx >= WEB_ATR_MAX_UNITS) return false;
  if (g_webAtrMountedName[idx].length() == 0) {
    webAtrResetResolvedSlot(idx);
    return false;
  }
  if (!forceRefresh && g_webAtrSlotPresent[idx] && g_webAtrMountedMeta[idx].valid) {
    m = g_webAtrMountedMeta[idx];
    return true;
  }
  String path = webAtrResolvedPathForIndex(idx);
  bool ok = webAtrReadMetaFromPath(path, m);
  g_webAtrSlotPresent[idx] = ok;
  if (ok) g_webAtrMountedMeta[idx] = m;
  else memset(&g_webAtrMountedMeta[idx], 0, sizeof(g_webAtrMountedMeta[idx]));
  return ok;
}

static bool webAtrNameEndsWith(String name, const char* extLower) {
  name.toLowerCase();
  return name.endsWith(extLower);
}

static bool webAtrLooksLikeXexUpload(const String& originalName) {
  return webAtrNameEndsWith(originalName, ".xex") || webAtrNameEndsWith(originalName, ".com") || webAtrNameEndsWith(originalName, ".exe");
}

static bool webCasLooksLikeName(String name) {
  name.trim();
  name.toLowerCase();
  return name.endsWith(".cas");
}

static bool webCasLooksLikeUpload(const String& originalName) {
  return webAtrNameEndsWith(originalName, ".cas");
}

static String webCasPathForName(const String& storedName) {
  if (storedName.length() == 0) return String("");
  if (storedName.startsWith("/")) return storedName;
#if WEB_STORAGE_USE_SD
  String p;
  p = webAtrFindExistingInDir("/CAS", storedName);
  if (p.length()) return p;
  p = webAtrFindExistingInDir("/LIBRARY", storedName);
  if (p.length()) return p;
  p = webAtrFindExistingInDir("/", storedName);
  if (p.length()) return p;
  p = webAtrFindExistingRecursive("/", storedName);
  if (p.length()) return p;
  return String(WEB_CAS_PREFIX) + webAtrSanitizeFileName(storedName);
#else
  return String(WEB_CAS_PREFIX) + webAtrSanitizeFileName(storedName);
#endif
}

static void casSetLastError(const String& err) {
  size_t i = 0;
  for (; i < sizeof(g_casLastError) - 1 && i < err.length(); i++) g_casLastError[i] = err[i];
  g_casLastError[i] = '\0';
}

static void saveCasConfigToNvs() {
  if (!prefs.begin("xf551cfg", false)) return;
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putString("casName", g_casMountedName);
  prefs.putBool("casMounted", g_casMounted);
  prefs.end();
}

static void loadCasConfigFromNvs() {
  // F49Z35: el cassette debe partir siempre detenido y sin C: montado.
  // No restauramos el último CAS desde NVS; equivalente a "Soltar C:" en cada arranque.
  g_casPlaying = false;
  g_casPaused = false;
  g_casEof = false;
  g_casBootModeActive = false;
  g_casMounted = false;
  g_casMountedName = "";
  g_casMountedPath = "";
  g_casBytesQueued = 0;
  g_casChunksQueued = 0;
  casClosePlaybackFile();
  casAutoResetAnalysis();
  webAtrPatchCasFlagsInFilesCache("", false, false);

  // Limpia la persistencia para que futuros reinicios tampoco recuperen C:.
  if (prefs.begin("xf551cfg", false)) {
    prefs.putUInt("magic", CFG_MAGIC);
    prefs.putString("casName", "");
    prefs.putBool("casMounted", false);
    prefs.end();
  }
}

static String webAtrStoredTypeForName(const String& storedName) {
  // F49Z50: el tipo visible/filtro de Biblioteca debe venir de la extensión real,
  // no de agrupaciones internas. Antes .COM/.EXE se mostraban como XEX y los
  // botones filtraban archivos que no correspondían.
  String n = storedName;
  n.toLowerCase();
  if (n.endsWith(".cas")) return String("CAS");
  if (n.endsWith(".atr")) return String("ATR");
  if (n.endsWith(".xex")) return String("XEX");
  if (n.endsWith(".com")) return String("COM");
  if (n.endsWith(".exe")) return String("EXE");
  if (n.endsWith(".bas")) return String("BAS");
  if (n.endsWith(".sec")) return String("SEC");
  return String("OTHER");
}

static String webAtrBaseNameForDos83(String storedName) {
  storedName.replace("\\", "/");
  int slash = storedName.lastIndexOf('/');
  if (slash >= 0) storedName = storedName.substring(slash + 1);
  String lower = storedName;
  lower.toLowerCase();
  if (lower.endsWith(".atr")) storedName = storedName.substring(0, storedName.length() - 4);
  lower = storedName;
  lower.toLowerCase();
  if (lower.endsWith(".xex") || lower.endsWith(".com") || lower.endsWith(".exe")) storedName = storedName.substring(0, storedName.length() - 4);
  storedName.trim();
  if (storedName.length() == 0) storedName = "GAME";
  return storedName;
}

static uint16_t webAtrDosDataSectorForIndex(uint16_t idx, uint16_t totalSectors) {
  // Sectores libres estilo DOS 2.x/2.5: 1-3 boot, 360 VTOC, 361-368 directorio.
  // En ED evitamos 1024 (VTOC2) y no usamos >1023 para mantener enlaces DOS de 10 bits.
  if (idx < 356) return (uint16_t)(4 + idx);            // 4..359
  idx -= 356;
  uint16_t s = (uint16_t)(369 + idx);                   // 369..
  if (totalSectors > 720 && s >= 1024) return 0;        // no usar 1024+ en este generador simple
  if (s > totalSectors) return 0;
  return s;
}

static uint16_t webAtrDosMaxDataSectors(uint16_t totalSectors) {
  uint16_t n = 0;
  while (webAtrDosDataSectorForIndex(n, totalSectors) != 0) n++;
  return n;
}

static void webAtrFillDosName83(const String& base, const String& sourceName, uint8_t out8[8], uint8_t out3[3]) {
  memset(out8, ' ', 8);
  memset(out3, ' ', 3);
  String b = base;
  b.toUpperCase();
  int p = 0;
  for (size_t i = 0; i < b.length() && p < 8; i++) {
    char c = b[i];
    if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') out8[p++] = (uint8_t)c;
  }

  String n = sourceName;
  n.toLowerCase();
  const char* ext = "XEX";
  if (n.endsWith(".com")) ext = "COM";
  else if (n.endsWith(".exe")) ext = "EXE";
  out3[0] = ext[0]; out3[1] = ext[1]; out3[2] = ext[2];
}

static bool webAtrConvertXexToDosAtr(const String& xexTmpPath, const String& atrDestPath, const String& storedName, String &err) {
  if (!webAtrFsReady() || xexTmpPath.length() == 0 || atrDestPath.length() == 0) { err = "Almacenamiento no disponible"; return false; }
  File src = SPIFFS.open(xexTmpPath, "r");
  if (!src) { err = "No se pudo abrir XEX temporal"; return false; }
  uint32_t xexSize = (uint32_t)src.size();
  if (xexSize == 0) { src.close(); err = "XEX vacío"; return false; }

  uint32_t fileSectors32 = (xexSize + 124UL) / 125UL;
  uint16_t totalSectors = 720;
  if (fileSectors32 > webAtrDosMaxDataSectors(720)) totalSectors = 1040;
  uint16_t maxSectors = webAtrDosMaxDataSectors(totalSectors);
  if (fileSectors32 == 0 || fileSectors32 > maxSectors) {
    src.close();
    err = "XEX demasiado grande para conversión DOS 2.x/2.5 en flash";
    return false;
  }
  uint16_t fileSectors = (uint16_t)fileSectors32;

  if (SPIFFS.exists(atrDestPath)) SPIFFS.remove(atrDestPath);
  File out = SPIFFS.open(atrDestPath, "w");
  if (!out) { src.close(); err = "No se pudo crear imagen montable para XEX"; return false; }

  uint8_t hdr[16];
  memset(hdr, 0, sizeof(hdr));
  hdr[0] = 0x96; hdr[1] = 0x02;
  uint32_t dataBytes = (uint32_t)totalSectors * 128UL;
  uint32_t pars = dataBytes / 16UL;
  hdr[2] = (uint8_t)(pars & 0xFF);
  hdr[3] = (uint8_t)((pars >> 8) & 0xFF);
  hdr[4] = 128; hdr[5] = 0;
  hdr[6] = (uint8_t)((pars >> 16) & 0xFF);
  hdr[7] = (uint8_t)((pars >> 24) & 0xFF);
  if (out.write(hdr, sizeof(hdr)) != sizeof(hdr)) { src.close(); out.close(); err = "Error escribiendo cabecera ATR"; return false; }

  uint8_t sec[128];
  uint8_t name8[8], ext3[3];
  webAtrFillDosName83(webAtrBaseNameForDos83(storedName), storedName, name8, ext3);

  uint16_t dataIndex = 0;
  for (uint16_t secNo = 1; secNo <= totalSectors; secNo++) {
    memset(sec, 0, sizeof(sec));

    if (secNo == 360) {
      sec[0] = 0x02; // VTOC DOS 2.x compatible (mínimo para lectura)
      uint16_t freeSectors = (uint16_t)(totalSectors > (fileSectors + 12) ? totalSectors - fileSectors - 12 : 0);
      sec[1] = (uint8_t)(freeSectors & 0xFF);
      sec[2] = (uint8_t)(freeSectors >> 8);
    } else if (secNo == 361) {
      sec[0] = 0x42; // entrada activa DOS 2.x
      sec[1] = (uint8_t)(fileSectors & 0xFF);
      sec[2] = (uint8_t)(fileSectors >> 8);
      uint16_t firstSec = webAtrDosDataSectorForIndex(0, totalSectors);
      sec[3] = (uint8_t)(firstSec & 0xFF);
      sec[4] = (uint8_t)(firstSec >> 8);
      memcpy(sec + 5, name8, 8);
      memcpy(sec + 13, ext3, 3);
    }

    // Sectores de datos del XEX: 125 bytes + 3 bytes link DOS 2.x.
    uint16_t expectedSec = (dataIndex < fileSectors) ? webAtrDosDataSectorForIndex(dataIndex, totalSectors) : 0;
    if (expectedSec != 0 && secNo == expectedSec) {
      int n = src.read(sec, 125);
      if (n < 0) n = 0;
      if (n < 125) memset(sec + n, 0, 125 - n);
      uint16_t nextSec = (dataIndex + 1 < fileSectors) ? webAtrDosDataSectorForIndex(dataIndex + 1, totalSectors) : 0;
      uint8_t fileNum = 0;
      sec[125] = (uint8_t)((fileNum << 2) | ((nextSec >> 8) & 0x03));
      sec[126] = (uint8_t)(nextSec & 0xFF);
      sec[127] = (uint8_t)(n == 0 ? 125 : n);
      dataIndex++;
    }

    if (out.write(sec, sizeof(sec)) != sizeof(sec)) { src.close(); out.close(); err = "Error escribiendo sector ATR"; return false; }
    if ((secNo & 0x3F) == 0) yield();
  }

  src.close();
  out.close();

  WebAtrMeta m;
  if (!webAtrReadMetaFromPath(atrDestPath, m)) { err = "Imagen montable XEX inválida"; return false; }
  logf("[WEB-ATR] XEX montable generado name=%s xexBytes=%lu imageBytes=%lu sectors=%lu fileSectors=%u",
       storedName.c_str(), (unsigned long)xexSize, (unsigned long)m.fileSize,
       (unsigned long)m.totalSectors, (unsigned)fileSectors);
  return true;
}

static bool webAtrReadMetaFromPath(const String& path, WebAtrMeta &m) {
  memset(&m, 0, sizeof(m));
  if (!webAtrFsReady() || path.length() == 0 || !SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  uint8_t h[16];
  if (f.read(h, sizeof(h)) != (int)sizeof(h)) { f.close(); return false; }
  m.fileSize = (uint32_t)f.size();
  f.close();
  if (h[0] != 0x96 || h[1] != 0x02 || m.fileSize <= 16) return false;
  m.sectorSize = (uint16_t)h[4] | ((uint16_t)h[5] << 8);
  if (m.sectorSize == 0) m.sectorSize = 128;
  if (!(m.sectorSize == 128 || m.sectorSize == 256)) return false;
  m.dataBytes = m.fileSize - 16;
  if (m.sectorSize == 256) {
    if (m.dataBytes < 384) return false;
    m.totalSectors = 3 + ((m.dataBytes - 384) / 256);
  } else {
    m.totalSectors = m.dataBytes / 128;
  }
  m.valid = (m.totalSectors > 0);
  return m.valid;
}

static bool webAtrReadMetaForDev(uint8_t dev, WebAtrMeta &m) {
  int idx = webAtrUnitIndex(dev);
  if (idx < 0) { memset(&m, 0, sizeof(m)); return false; }
  return webAtrReadMetaForIndex(idx, m, false);
}

static void webAtrRefreshPresence() {
  g_webAtrFilePresent = false;
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    WebAtrMeta m;
    bool ok = webAtrReadMetaForIndex(i, m, true);
    if (ok) g_webAtrFilePresent = true;
  }
}

// F16/F17: para montar desde la web no se debe validar la metadata de todas las unidades.
// Validamos solo los slots realmente cambiados, resolvemos ruta una vez y recomputamos el estado global.
static void webAtrRefreshPresenceMask(uint8_t unitMask) {
  unitMask &= DRIVE_UI_MAX_MASK;
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    if (unitMask & (1u << i)) {
      WebAtrMeta m;
      (void)webAtrReadMetaForIndex(i, m, true);
    }
  }
  g_webAtrFilePresent = false;
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    if (g_webAtrSlotPresent[i]) { g_webAtrFilePresent = true; break; }
  }
}

static bool webAtrOffsetForSector(uint16_t sec, const WebAtrMeta &m, uint32_t &off, uint16_t &len) {
  if (!m.valid || sec == 0 || sec > m.totalSectors) return false;
  if (m.sectorSize == 256) {
    if (sec <= 3) {
      len = 128;
      off = 16 + ((uint32_t)(sec - 1) * 128u);
    } else {
      len = 256;
      off = 16 + 384u + ((uint32_t)(sec - 4) * 256u);
    }
  } else {
    len = 128;
    off = 16 + ((uint32_t)(sec - 1) * 128u);
  }
  return (off + len <= m.fileSize);
}

static bool webAtrReadSector(uint8_t dev, uint16_t sec, uint8_t *buf, uint16_t &len) {
  int idx = webAtrUnitIndex(dev);
  if (idx < 0) return false;

  String mountedName = g_webAtrMountedName[idx];
  if (mountedName.length() == 0) return false;

  g_webAtrLastReadMs = millis();

  if (webAtrCacheGet(dev, sec, mountedName, buf, len)) {
    return true;
  }

  if (!webAtrReadSectorRaw(dev, sec, buf, len, mountedName)) {
    return false;
  }

  webAtrCachePut(dev, sec, mountedName, buf, len, false);
  return true;
}

static void webAtrBuildPercom(uint8_t dev, uint8_t out[PERCOM_BLOCK_LEN]) {
  memset(out, 0, PERCOM_BLOCK_LEN);
  WebAtrMeta m;
  bool ok = webAtrReadMetaForDev(dev, m);
  uint16_t bps = ok ? m.sectorSize : 128;
  uint32_t total32 = ok ? m.totalSectors : 720;
  uint16_t total = (uint16_t)((total32 > 65535) ? 65535 : total32);
  uint16_t spt = 18;
  uint8_t sides = 1;
  uint16_t tracks = 40;

  if (bps == 128 && total == 1040) { tracks = 40; spt = 26; sides = 1; }
  else if (total == 720) { tracks = 40; spt = 18; sides = 1; }
  else if (total == 1440) { tracks = 40; spt = 18; sides = 2; }
  else if (total == 2880) { tracks = 80; spt = 18; sides = 2; }
  else {
    spt = 18;
    sides = 1;
    tracks = (uint16_t)((total + spt - 1) / spt);
    if (tracks == 0) tracks = 40;
    if (tracks > 255) tracks = 255;
  }

  out[0] = (uint8_t)tracks;
  out[1] = 0x04;
  out[2] = (uint8_t)(spt >> 8);
  out[3] = (uint8_t)(spt & 0xFF);
  out[4] = (uint8_t)((sides > 1) ? 1 : 0);
  out[5] = (uint8_t)((bps >= 256) ? 0x04 : 0x00);
  out[6] = (uint8_t)(bps >> 8);
  out[7] = (uint8_t)(bps & 0xFF);
  out[8] = 0x01;
  out[9] = 0x41;
}


static void beginBtDiskUart() {
  // F46: ampliar buffers antes de iniciar la UART evita pérdidas en ráfagas BT/HC-05.
  SerialBTDisk.setRxBufferSize(BT_SIO2PC_UART_RX_BUFFER);
  SerialBTDisk.setTxBufferSize(BT_SIO2PC_UART_TX_BUFFER);
  SerialBTDisk.setTimeout(20);
  SerialBTDisk.begin(BT_DISK_UART_BAUD_CFG, SERIAL_8N1, PIN_BT_RX, PIN_BT_TX);
}

static uint16_t btSio2pcCacheSecKey(uint8_t base, uint16_t sec) {
  if (base == 0x53) return 0;
  if (base == 0x4E) return PERCOM_SEC_MAGIC;
  return sec;
}

static void btSio2pcCacheClear() {
  for (uint8_t i = 0; i < BT_SIO2PC_CACHE_SLOTS; ++i) g_btSio2pcCache[i].valid = false;
  g_btSio2pcCacheClear++;
}

static bool btSio2pcCacheGet(uint8_t dev, uint16_t secKey, uint16_t expectedLen, uint8_t* out, uint16_t &outLen) {
  if (!BT_SIO2PC_CACHE_ENABLED) { g_btSio2pcCacheBypass++; return false; }
  if (!out || expectedLen == 0 || expectedLen > 256) return false;
  uint32_t now = millis();
  for (uint8_t i = 0; i < BT_SIO2PC_CACHE_SLOTS; ++i) {
    BtSio2pcCacheEntry &e = g_btSio2pcCache[i];
    if (!e.valid) continue;
    if (e.dev != dev || e.sec != secKey || e.len != expectedLen) continue;
    if ((now - e.lastUseMs) > BT_SIO2PC_CACHE_TTL_MS) { e.valid = false; break; }
    memcpy(out, e.data, e.len);
    outLen = e.len;
    e.lastUseMs = now;
    g_btSio2pcCacheHit++;
    return true;
  }
  g_btSio2pcCacheMiss++;
  return false;
}

static void btSio2pcCachePut(uint8_t dev, uint16_t secKey, const uint8_t* data, uint16_t len) {
  if (!BT_SIO2PC_CACHE_ENABLED) return;
  if (!data || len == 0 || len > 256) return;
  uint32_t now = millis();
  int slot = -1;
  uint32_t oldest = 0xFFFFFFFFUL;
  for (uint8_t i = 0; i < BT_SIO2PC_CACHE_SLOTS; ++i) {
    BtSio2pcCacheEntry &e = g_btSio2pcCache[i];
    if (e.valid && e.dev == dev && e.sec == secKey) { slot = i; break; }
    if (!e.valid) { slot = i; break; }
    if (e.lastUseMs < oldest) { oldest = e.lastUseMs; slot = i; }
  }
  if (slot < 0) return;
  BtSio2pcCacheEntry &e = g_btSio2pcCache[slot];
  e.valid = true;
  e.dev = dev;
  e.sec = secKey;
  e.len = len;
  e.lastUseMs = now;
  memcpy(e.data, data, len);
  g_btSio2pcCacheStore++;
}

static bool btSio2pcPreferSioOverWeb() {
  if (!BT_SIO2PC_ENABLED) return false;
  if (BT_SIO2PC_PROFILE == 0) return false; // stable mantiene la web más respirable
  uint32_t last = g_btSio2pcLastMs;
  if (last == 0) return false;
  return (millis() - last) < 1200;
}

static const char* btSio2pcProfileName(uint8_t p) {
  switch (p) {
    case 0: return "stable";
    case 1: return "fast";
    case 2: return "loader";
    case 3: return "turbo";
    default: return "fast";
  }
}

static uint8_t btSio2pcParseProfile(const String &raw) {
  String prof = raw;
  prof.toLowerCase();
  if (prof == "stable" || prof == "0") return 0;
  if (prof == "loader" || prof == "2") return 2;
  if (prof == "turbo"  || prof == "3") return 3;
  return 1;
}

static void btSio2pcSetLastErr(const char* err) {
  if (!err) err = "";
  size_t i = 0;
  for (; i < sizeof(g_btSio2pcLastErr) - 1 && err[i]; ++i) g_btSio2pcLastErr[i] = err[i];
  g_btSio2pcLastErr[i] = '\0';
}

static void btSio2pcRecordMs(uint32_t ms) {
  g_btSio2pcTotalMs += ms;
  uint32_t ok = g_btSio2pcReadOk + g_btSio2pcStatusOk + g_btSio2pcPercomOk + 1;
  g_btSio2pcAvgMs = ok ? (g_btSio2pcTotalMs / ok) : 0;
  if (ms > g_btSio2pcMaxMs) g_btSio2pcMaxMs = ms;
}

static void btSio2pcApplyProfile(uint8_t p) {
  if (p > 3) p = 1;
  BT_SIO2PC_PROFILE = p;
  if (p == 0) { // stable: tolerante a latencia Bluetooth, sin reintentos que desincronicen
    BT_SIO2PC_ACK_TIMEOUT_MS = 3000;
    BT_SIO2PC_COMPLETE_TIMEOUT_MS = 5000;
    BT_SIO2PC_DATA_TOTAL_TIMEOUT_MS = 14000;
    BT_SIO2PC_DATA_IDLE_TIMEOUT_MS = 900;
    BT_SIO2PC_CHK_TIMEOUT_MS = 2500;
    BT_SIO2PC_PRE_DRAIN_MS = 0;
    BT_SIO2PC_MAX_RETRY = 0;
    BT_SIO2PC_WEB_YIELD_DURING_IO = true;
    BT_SIO2PC_WEB_YIELD_INTERVAL_MS = 25;
  } else if (p == 2) { // loader: menor espera, silencioso, sin retry largo
    BT_SIO2PC_ACK_TIMEOUT_MS = 800;
    BT_SIO2PC_COMPLETE_TIMEOUT_MS = 1400;
    BT_SIO2PC_DATA_TOTAL_TIMEOUT_MS = 2400;
    BT_SIO2PC_DATA_IDLE_TIMEOUT_MS = 90;
    BT_SIO2PC_CHK_TIMEOUT_MS = 700;
    BT_SIO2PC_PRE_DRAIN_MS = 0;
    BT_SIO2PC_MAX_RETRY = 0;
    BT_SIO2PC_WEB_YIELD_DURING_IO = false;
    BT_SIO2PC_WEB_YIELD_INTERVAL_MS = 80;
  } else if (p == 3) { // turbo: solo para enlaces HC-05/PC muy estables
    BT_SIO2PC_ACK_TIMEOUT_MS = 500;
    BT_SIO2PC_COMPLETE_TIMEOUT_MS = 900;
    BT_SIO2PC_DATA_TOTAL_TIMEOUT_MS = 1600;
    BT_SIO2PC_DATA_IDLE_TIMEOUT_MS = 45;
    BT_SIO2PC_CHK_TIMEOUT_MS = 450;
    BT_SIO2PC_PRE_DRAIN_MS = 0;
    BT_SIO2PC_MAX_RETRY = 0;
    BT_SIO2PC_WEB_YIELD_DURING_IO = false;
    BT_SIO2PC_WEB_YIELD_INTERVAL_MS = 120;
  } else { // fast: punto medio; sigue siendo más tolerante que F46
    BT_SIO2PC_ACK_TIMEOUT_MS = 1400;
    BT_SIO2PC_COMPLETE_TIMEOUT_MS = 2400;
    BT_SIO2PC_DATA_TOTAL_TIMEOUT_MS = 4200;
    BT_SIO2PC_DATA_IDLE_TIMEOUT_MS = 220;
    BT_SIO2PC_CHK_TIMEOUT_MS = 1000;
    BT_SIO2PC_PRE_DRAIN_MS = 0;
    BT_SIO2PC_MAX_RETRY = 0;
    BT_SIO2PC_WEB_YIELD_DURING_IO = true;
    BT_SIO2PC_WEB_YIELD_INTERVAL_MS = 45;
  }
}

static void btSio2pcApplyUnitProfileForDev(uint8_t dev) {
  int idx = driveUiIndex(dev);
  if (idx < 0) return;
  uint8_t p = BT_SIO2PC_UNIT_PROFILE[idx];
  if (p > 3) p = 1;
  btSio2pcApplyProfile(p);
}

static void btSio2pcServiceWhileWaiting(uint32_t now, uint32_t &lastWebYield) {
  if (BT_SIO2PC_WEB_YIELD_DURING_IO && (now - lastWebYield) >= BT_SIO2PC_WEB_YIELD_INTERVAL_MS) {
    server.handleClient();
    lastWebYield = now;
  } else if (!BT_SIO2PC_WEB_YIELD_DURING_IO) {
    g_btSio2pcNoWebLoops++;
  }
  delay(0);
}

static bool btSio2pcReadByte(uint8_t &out, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint32_t lastWeb = t0;
  while ((millis() - t0) < timeoutMs) {
    if (SerialBTDisk.available() > 0) {
      int c = SerialBTDisk.read();
      if (c >= 0) { out = (uint8_t)c; g_btSio2pcBytesRx++; return true; }
    }
    btSio2pcServiceWhileWaiting(millis(), lastWeb);
  }
  return false;
}

static bool btSio2pcReadBytes(uint8_t* out, uint16_t len, uint32_t totalTimeoutMs, uint32_t idleTimeoutMs) {
  if (!out || len == 0) return false;
  uint16_t got = 0;
  uint32_t t0 = millis();
  uint32_t lastRx = t0;
  uint32_t lastWeb = t0;
  while (got < len) {
    int avail = SerialBTDisk.available();
    if (avail > 0) {
      int want = (int)len - (int)got;
      if (avail > want) avail = want;
      int actual = 0;
      for (int i = 0; i < avail; ++i) {
        int c = SerialBTDisk.read();
        if (c < 0) break;
        out[got++] = (uint8_t)c;
        actual++;
      }
      g_btSio2pcBytesRx += actual;
      lastRx = millis();
      continue;
    }
    uint32_t now = millis();
    if ((now - t0) >= totalTimeoutMs) return false;
    if (got > 0 && (now - lastRx) >= idleTimeoutMs) return false;
    btSio2pcServiceWhileWaiting(now, lastWeb);
  }
  g_btSio2pcBurstReads++;
  return true;
}

static void btSio2pcDrain(uint16_t msMax) {
  if (msMax == 0) return;
  g_btSio2pcStaleDrains++;
  uint32_t t0 = millis();
  while (SerialBTDisk.available() > 0) { (void)SerialBTDisk.read(); g_btSio2pcDrainBytes++; }
  while ((millis() - t0) < msMax) {
    if (SerialBTDisk.available() > 0) {
      while (SerialBTDisk.available() > 0) { (void)SerialBTDisk.read(); g_btSio2pcDrainBytes++; }
      break;
    }
    delay(0);
  }
}

static void btSio2pcMarkTransportError(const char* err) {
  btSio2pcSetLastErr(err);
  // F47: si falló un comando, pueden llegar bytes tarde por Bluetooth.
  // No los drenamos antes de cada comando, solo marcamos el error y el próximo ciclo
  // los contadores permitirán verlo.
  if (SerialBTDisk.available() > 0) g_btSio2pcLateBytesAfterError += (uint32_t)SerialBTDisk.available();
}

static uint16_t btSio2pcExpectedLen(uint8_t cmd, uint16_t sec, uint8_t ddHint) {
  uint8_t base = cmd & 0x7F;
  if (base == 0x53) return 4;
  if (base == 0x4E) return PERCOM_BLOCK_LEN;
  if (base == 0x52) {
    if (ddHint && !(sec >= 1 && sec <= 3)) return 256;
    return 128;
  }
  return 0;
}

static bool handleBtSio2pcFrame(uint8_t dev, const uint8_t* payload, uint8_t len) {
  if (!g_btDiskUartReady || !payload || len < 7) return sendLocalNakToRP(dev, "BT_SIO2PC_UART_NOT_READY");

  uint8_t cmd = payload[1];
  uint8_t aux1 = payload[3];
  uint8_t aux2 = payload[4];
  uint8_t ddHint = payload[5];
  uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);
  uint8_t base = cmd & 0x7F;
  btSio2pcApplyUnitProfileForDev(dev);
  uint16_t expectedLen = btSio2pcExpectedLen(cmd, sec, ddHint);

  g_btSio2pcCmdRouted++;
  g_btSio2pcLastCmd = cmd;
  g_btSio2pcLastDev = dev;
  g_btSio2pcLastSec = sec;
  g_btSio2pcLastMs = millis();

  if (expectedLen == 0) { g_btSio2pcNak++; return sendLocalNakToRP(dev, "BT_SIO2PC_UNSUPPORTED_CMD"); }

  uint8_t sioCmd[5] = { dev, cmd, aux1, aux2, 0 };
  sioCmd[4] = calcSioChecksum(sioCmd, 4);

  uint8_t buf[256];
  if (expectedLen > sizeof(buf)) expectedLen = sizeof(buf);
  uint16_t secKey = btSio2pcCacheSecKey(base, sec);
  uint16_t cachedLen = 0;

  // F47: cache opcional y apagada por defecto. Solo se usa en READ normal, no STATUS/PERCOM.
  if (base == 0x52 && btSio2pcCacheGet(dev, secKey, expectedLen, buf, cachedLen)) {
    btSio2pcSetLastErr("");
    sendLocalAckToRP(dev, nullptr);
    sendLocalSectorChunkToRP(dev, secKey, buf, cachedLen);
    if (base == 0x53) g_btSio2pcStatusOk++;
    else if (base == 0x4E) g_btSio2pcPercomOk++;
    else g_btSio2pcReadOk++;
    clearLastMasterOpLocal();
    sendTimingUpdateToRPThrottled();
    if (!BT_SIO2PC_QUIET_LOG || (millis() - g_btSio2pcLastLogMs) > 1500 || base != 0x52) {
      logf("[BT-SIO2PC] CACHE HIT cmd=0x%02X dev=%s sec=%u len=%u profile=%s",
           (unsigned)cmd, devName(dev), (unsigned)sec, (unsigned)cachedLen, btSio2pcProfileName(BT_SIO2PC_PROFILE));
      g_btSio2pcLastLogMs = millis();
    }
    return true;
  }

  uint8_t rxChk = 0;
  bool okFrame = false;
  const uint8_t maxAttempts = (uint8_t)(1 + BT_SIO2PC_MAX_RETRY);

  for (uint8_t attempt = 0; attempt < maxAttempts && !okFrame; ++attempt) {
    if (attempt > 0) { g_btSio2pcRetry++; btSio2pcDrain(2); }
    else btSio2pcDrain(BT_SIO2PC_PRE_DRAIN_MS);

    SerialBTDisk.write(sioCmd, sizeof(sioCmd));
    // F47: no forzar flush por comando; el UART transmite por interrupción.
    // flush() bloquea y en Bluetooth puede amplificar la latencia.
    if (false) SerialBTDisk.flush();

    uint8_t b = 0;
    if (!btSio2pcReadByte(b, BT_SIO2PC_ACK_TIMEOUT_MS)) { btSio2pcMarkTransportError("ACK_TIMEOUT"); break; }
    if (b == SIO_NAK || b == SIO_ERROR) { g_btSio2pcNak++; btSio2pcSetLastErr("HOST_NAK"); return sendLocalNakToRP(dev, "BT_SIO2PC_HOST_NAK"); }
    if (b != SIO_ACK) { g_btSio2pcNak++; btSio2pcSetLastErr("NO_ACK"); return sendLocalNakToRP(dev, "BT_SIO2PC_NO_ACK"); }

    if (!btSio2pcReadByte(b, BT_SIO2PC_COMPLETE_TIMEOUT_MS)) { btSio2pcMarkTransportError("COMPLETE_TIMEOUT"); break; }
    if (b == SIO_ERROR || b == SIO_NAK) { g_btSio2pcNak++; btSio2pcSetLastErr("HOST_ERROR"); return sendLocalNakToRP(dev, "BT_SIO2PC_HOST_ERROR"); }
    if (b != SIO_COMPLETE) { g_btSio2pcNak++; btSio2pcSetLastErr("NO_COMPLETE"); return sendLocalNakToRP(dev, "BT_SIO2PC_NO_COMPLETE"); }

    if (!btSio2pcReadBytes(buf, expectedLen, BT_SIO2PC_DATA_TOTAL_TIMEOUT_MS, BT_SIO2PC_DATA_IDLE_TIMEOUT_MS)) { btSio2pcMarkTransportError("DATA_TIMEOUT"); break; }
    if (!btSio2pcReadByte(rxChk, BT_SIO2PC_CHK_TIMEOUT_MS)) { btSio2pcMarkTransportError("CHK_TIMEOUT"); break; }

    uint8_t calc = calcSioChecksum(buf, expectedLen);
    if (rxChk != calc) {
      g_btSio2pcBadChk++;
      btSio2pcSetLastErr("BAD_DATA_CHK");
      logf("[BT-SIO2PC] checksum DATA invalido cmd=0x%02X dev=%s sec=%u rx=0x%02X calc=0x%02X len=%u", (unsigned)cmd, devName(dev), (unsigned)sec, (unsigned)rxChk, (unsigned)calc, (unsigned)expectedLen);
      return sendLocalNakToRP(dev, "BT_SIO2PC_BAD_DATA_CHK");
    }
    okFrame = true;
  }

  if (!okFrame) {
    g_btSio2pcTimeout++;
    return sendLocalNakToRP(dev, g_btSio2pcLastErr[0] ? g_btSio2pcLastErr : "BT_SIO2PC_TIMEOUT");
  }

  btSio2pcSetLastErr("");
  btSio2pcRecordMs(millis() - g_btSio2pcLastMs);
  if (base == 0x52) btSio2pcCachePut(dev, secKey, buf, expectedLen);
  sendLocalAckToRP(dev, nullptr);
  if (base == 0x53) { sendLocalSectorChunkToRP(dev, 0, buf, expectedLen); g_btSio2pcStatusOk++; }
  else if (base == 0x4E) { sendLocalSectorChunkToRP(dev, PERCOM_SEC_MAGIC, buf, expectedLen); g_btSio2pcPercomOk++; }
  else { sendLocalSectorChunkToRP(dev, sec, buf, expectedLen); g_btSio2pcReadOk++; }
  clearLastMasterOpLocal();
  sendTimingUpdateToRPThrottled();
  if (!BT_SIO2PC_QUIET_LOG || (millis() - g_btSio2pcLastLogMs) > 1500 || base != 0x52) {
    logf("[BT-SIO2PC] ACK cmd=0x%02X dev=%s sec=%u len=%u profile=%s (%lu ms)",
         (unsigned)cmd, devName(dev), (unsigned)sec, (unsigned)expectedLen,
         btSio2pcProfileName(BT_SIO2PC_PROFILE), (unsigned long)(millis() - g_btSio2pcLastMs));
    g_btSio2pcLastLogMs = millis();
  }
  return true;
}

static bool sendLocalAckToRP(uint8_t dev, const char* src) {
  uint8_t p[2] = { TYPE_ACK, dev };
  sendUartFrameToRP(p, sizeof(p));
  if (src) logf("[%s] ACK local dev=%s", src, devName(dev));
  return true;
}

static void sendLocalSectorChunkToRP(uint8_t dev, uint16_t sec, const uint8_t* buf, uint16_t len) {
  if (!buf || len == 0) return;
  uint8_t pkt[6 + CHUNK_PAYLOAD];
  uint8_t count = (uint8_t)((len + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD);
  if (count == 0) count = 1;
  for (uint8_t i = 0; i < count; i++) {
    uint16_t off = (uint16_t)i * CHUNK_PAYLOAD;
    uint16_t n = len - off;
    if (n > CHUNK_PAYLOAD) n = CHUNK_PAYLOAD;
    pkt[0] = TYPE_SECTOR_CHUNK;
    pkt[1] = dev;
    pkt[2] = (uint8_t)(sec & 0xFF);
    pkt[3] = (uint8_t)(sec >> 8);
    pkt[4] = i;
    pkt[5] = count;
    memcpy(pkt + 6, buf + off, n);
    sendUartFrameToRP(pkt, (uint8_t)(6 + n));
    yield();
  }
}

static bool webAtrDevEnabled(uint8_t dev) {
  int idx = webAtrUnitIndex(dev);
  return (idx >= 0) && webAtrSelected(dev) && g_webAtrSlotPresent[idx];
}

static bool handleWebAtrFrame(uint8_t dev, const uint8_t* payload, uint8_t len) {
  if (!payload || len < 7 || payload[0] != TYPE_CMD_FRAME) return false;
  int idx = webAtrUnitIndex(dev);
  if (idx < 0) return false;
  uint8_t cmd = payload[1];
  uint8_t base = cmd & 0x7F;
  uint16_t sec = (uint16_t)payload[3] | ((uint16_t)payload[4] << 8);
  uint32_t t0 = millis();

  if (base == 0x53) {
    uint8_t st[4] = { 0x00, 0xFF, 0xFE, 0x00 };
    sendLocalAckToRP(dev, nullptr);
    sendLocalSectorChunkToRP(dev, 0, st, sizeof(st));
    g_webAtrStatus++;
    g_webAtrStatusByUnit[idx]++;
    clearLastMasterOpLocal();
    sendTimingUpdateToRPThrottled();
    logf("[WEB-ATR] ACK STATUS dev=%s file=%s (%lu ms)", devName(dev), g_webAtrMountedName[idx].c_str(), (unsigned long)(millis() - t0));
    return true;
  }

  if (base == 0x4E) {
    uint8_t percom[PERCOM_BLOCK_LEN];
    webAtrBuildPercom(dev, percom);
    sendLocalAckToRP(dev, nullptr);
    sendLocalSectorChunkToRP(dev, PERCOM_SEC_MAGIC, percom, PERCOM_BLOCK_LEN);
    g_webAtrPercom++;
    g_webAtrPercomByUnit[idx]++;
    clearLastMasterOpLocal();
    sendTimingUpdateToRPThrottled();
    logf("[WEB-ATR] ACK READ PERCOM dev=%s file=%s bps=%u spt=%u sides=%u (%lu ms)",
         devName(dev), g_webAtrMountedName[idx].c_str(), (unsigned)percomBytesPerSector(percom),
         (unsigned)percomSectorsPerTrack(percom), (unsigned)percomSides(percom),
         (unsigned long)(millis() - t0));
    return true;
  }

  if (base == 0x52) {
    uint8_t buf[256];
    uint16_t slen = 0;
    if (!webAtrReadSector(dev, sec, buf, slen)) {
      g_webAtrNak++;
      return sendLocalNakToRP(dev, "WEB_ATR_READ_FAIL");
    }
    sendLocalAckToRP(dev, nullptr);
    sendLocalSectorChunkToRP(dev, sec, buf, slen);
    g_webAtrReads++;
    g_webAtrReadsByUnit[idx]++;
    clearLastMasterOpLocal();
    sendTimingUpdateToRPThrottled();

    // V28: precarga del siguiente sector después de entregar el sector actual.
    // No toca ESP-NOW ni timings del RP2040.
    webAtrPrefetchNext(dev, sec);

    logf("[WEB-ATR] ACK READ dev=%s file=%s sec=%u len=%u cacheHit=%lu cacheMiss=%lu sdLastUs=%lu (%lu ms)",
         devName(dev), g_webAtrMountedName[idx].c_str(), (unsigned)sec, (unsigned)slen,
         (unsigned long)g_webAtrCacheHit, (unsigned long)g_webAtrCacheMiss,
         (unsigned long)g_webAtrSdReadUsLast, (unsigned long)(millis() - t0));
    return true;
  }

  g_webAtrNak++;
  return sendLocalNakToRP(dev, "WEB_ATR_READ_ONLY");
}
static bool sendLocalNakToRP(uint8_t dev, const char* reason) {
  uint8_t p[2] = { TYPE_NAK, dev };
  sendUartFrameToRP(p, sizeof(p));
  logf("[BT-GW] NAK local dev=%s reason=%s", devName(dev), reason ? reason : "");
  return false;
}

static const char* routeNameForDev(uint8_t dev) {
  if (webAtrDevEnabled(dev)) return "WEB-ATR";
  if (webAtrSelected(dev) && webAtrForced(dev)) return "WEB-ATR-FORCED-NOFILE-NAK";
  if (webAtrSelected(dev)) return "WEB-ATR-FALLBACK";
  if (btSio2pcDevEnabled(dev)) return "BT-SIO2PC";
  if (btSio2pcDevSelected(dev) && btSio2pcDevForced(dev)) return "BT-SIO2PC-FORCED-NAK";
  if (btSio2pcDevSelected(dev)) return "ESPNOW-FALLBACK";
  if (btDiskDevEnabled(dev)) return "BT";
  if (btDiskDevSelected(dev) && btDiskDevForced(dev)) return "BT-FORCED-OFFLINE-NAK";
  if (btDiskDevSelected(dev)) return "ESPNOW-FALLBACK";
  return "ESPNOW";
}

static bool routeFrameToBtOrEsp(uint8_t dev, const uint8_t* payload, uint8_t len) {
  if (webAtrDevEnabled(dev)) {
    return handleWebAtrFrame(dev, payload, len);
  }

  if (webAtrSelected(dev) && webAtrForced(dev)) {
    return sendLocalNakToRP(dev, "WEB_ATR_NOT_READY");
  }

  if (btSio2pcDevEnabled(dev)) {
    bool ok = handleBtSio2pcFrame(dev, payload, len);
    if (ok) return true;
    if (btSio2pcDevForced(dev)) return false;
    logf("[BT-SIO2PC] fallo dev=%s; fallback ESPNOW", devName(dev));
  }

  if (btSio2pcDevSelected(dev) && btSio2pcDevForced(dev)) {
    return sendLocalNakToRP(dev, "BT_SIO2PC_UNAVAILABLE");
  }

  if (btDiskDevEnabled(dev)) {
    g_btDiskCmdRouted++;
    if (sendBtDiskFrame(payload, len)) return true;
    if (btDiskDevForced(dev)) return sendLocalNakToRP(dev, "BT_UART_SEND_FAIL");
    logf("[BT-GW] BT send fail dev=%s; fallback ESPNOW", devName(dev));
  }

  if (btDiskDevSelected(dev) && btDiskDevForced(dev)) {
    return sendLocalNakToRP(dev, g_btDiskOnline ? "BT_UNAVAILABLE" : "BT_HOST_OFFLINE");
  }

  return sendEspToSlave(dev, payload, len);
}

// ========= Pendiente de WRITE desde el RP =========
struct PendingWriteFromRP {
  bool active;
  uint8_t dev;
  uint16_t sec;
  bool toBt;
};
PendingWriteFromRP g_pendingWriteRP = { false, 0, 0, false };

// ========= Último comando importante =========
struct LastMasterOp {
  bool active;
  uint8_t cmd;
  uint8_t dev;
  uint16_t sec;
  uint32_t sentMs;
};
LastMasterOp g_lastMasterOp = { false, 0, 0, 0, 0 };

static void clearLastMasterOpLocal() {
  g_lastMasterOp.active = false;
}

// ====== helpers auto-ajuste ======
static uint16_t clampU16(uint32_t v, uint16_t minV, uint16_t maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return (uint16_t)v;
}
static uint32_t clampU32(uint32_t v, uint32_t mn, uint32_t mx) {
  if (v < mn) return mn;
  if (v > mx) return mx;
  return v;
}

static const uint32_t UART_BAUD_OPTIONS[] = {
  115200, 230400, 460800, 921600, 1500000, 2000000
};

uint32_t pickSupportedUartBaud(uint32_t v) {
  // si viene exacto, ok; si no, elegimos el más cercano
  uint32_t best = UART_BAUD_OPTIONS[0];
  uint32_t bestDiff = (v > best) ? (v - best) : (best - v);

  for (size_t i = 0; i < sizeof(UART_BAUD_OPTIONS)/sizeof(UART_BAUD_OPTIONS[0]); i++) {
    uint32_t opt = UART_BAUD_OPTIONS[i];
    uint32_t diff = (v > opt) ? (v - opt) : (opt - v);
    if (diff < bestDiff) { best = opt; bestDiff = diff; }
    if (diff == 0) return opt;
  }
  return best;
}

void saveTimingConfigToNvs();
void savePrefetchConfigToNvs();
void saveAutoConfigToNvs();
void saveCommConfigToNvs();
void saveVerifyConfigToNvs();
void saveBtDiskConfigToNvs();
void savePrinterConfigToNvs();
void loadPrinterConfigFromNvs();
void connectPrinterStaWifi(bool waitForConnection);
static void startMdnsIfStaReady();
String jsonEscape(const String& in);
static String buildWifiStatusJson(bool includeOk);
void handleApiWifiStatus();
void handleApiWifiSet();
void sendPrinterCfgToRP();
void printerAppendHttpBuffer(const String& text);
bool printerFlushHttpBuffer(const char* reason);
void servicePrinterHttpBuffer();
void servicePrinterVirtualBuffer();
void servicePrinterManualPrintRequest();
bool printerFlushVirtualBuffer(const char* reason);
bool printerAppendVirtualAtasciiLine(const uint8_t* data, uint8_t len);
bool printerSendIppJpegRenderFileJob(const String& jpgPath, size_t jpgBytes, const char* jobName);

void autoTuneTimingsFromAck() {
  bool anyAuto = false;
  for (int i = 0; i < 4; i++) {
    if (g_driveTiming[i].autoEnabled) { anyAuto = true; break; }
  }
  if (!anyAuto) return;

  uint32_t bestAckMs = 0;
  bool have = false;

  for (int i = 0; i < 4; i++) {
    if (!g_driveTiming[i].autoEnabled) continue;
    uint32_t m = g_driveTiming[i].avgAckMs;
    if (m == 0) continue;
    if (!have || m < bestAckMs) { bestAckMs = m; have = true; }
  }
  if (!have) return;

  // Importante: el ACK puede incluir latencia real del disco/caché.
  // Si usamos ACKs altos para empujar timings SIO, el sistema completo se vuelve lento.
  // Solo autoajustamos cuando el ACK ya es claramente rápido (camino caliente).
  if (bestAckMs > 20) return;

  uint32_t ackUs = bestAckMs * 1000UL;
  float scale = 1.0f;
  if (g_autoProfile == 0) scale = 1.20f;      // seguro
  else if (g_autoProfile == 2) scale = 0.85f; // agresivo
  ackUs = (uint32_t)(ackUs * scale);

  uint16_t newAckToComplete  = clampU16((uint16_t)(ackUs / 8), 120, 600);
  uint16_t newCompleteToData = clampU16((uint16_t)(ackUs / 12), 80, 350);
  uint16_t newChunkDelay     = clampU16((uint16_t)(ackUs / 10), 60, 500);

  bool changed = false;
  if (abs((int)newAckToComplete - (int)T_ACK_TO_COMPLETE) > 25) {
    T_ACK_TO_COMPLETE = newAckToComplete; changed = true;
  }
  if (abs((int)newCompleteToData - (int)T_COMPLETE_TO_DATA) > 20) {
    T_COMPLETE_TO_DATA = newCompleteToData; changed = true;
  }
  if (T_DATA_TO_CHK == 0) { T_DATA_TO_CHK = 25; changed = true; }
  if (abs((int)newChunkDelay - (int)T_CHUNK_DELAY) > 30) {
    T_CHUNK_DELAY = newChunkDelay; changed = true;
  }

  if (changed) {
    logf("[AUTO] ACK caliente=%lu ms perfil=%d -> ackToComp=%u compToData=%u chunkDelay=%u",
         (unsigned long)bestAckMs, g_autoProfile,
         (unsigned)T_ACK_TO_COMPLETE, (unsigned)T_COMPLETE_TO_DATA, (unsigned)T_CHUNK_DELAY);
    saveTimingConfigToNvs();
  }
}

// ========= NVS =========
void saveTimingConfigToNvs() {
  if (!prefs.begin("xf551cfg", false)) { logf("[NVS] Error al abrir NVS tiempos"); return; }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUShort("t_ack", T_ACK_TO_COMPLETE);
  prefs.putUShort("t_comp", T_COMPLETE_TO_DATA);
  prefs.putUShort("t_chk", T_DATA_TO_CHK);
  prefs.putUShort("t_chd", T_CHUNK_DELAY);
  prefs.putUChar("autoProf", (uint8_t)g_autoProfile);
  prefs.end();
  logf("[NVS] Tiempos + perfil guardados");
}

void savePrefetchConfigToNvs() {
  if (!prefs.begin("xf551cfg", false)) { logf("[NVS] Error al abrir NVS prefetch"); return; }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUChar("pf1", prefetchCfg[0]);
  prefs.putUChar("pf2", prefetchCfg[1]);
  prefs.putUChar("pf3", prefetchCfg[2]);
  prefs.putUChar("pf4", prefetchCfg[3]);
  prefs.end();
  logf("[NVS] Prefetch guardado");
}

void saveAutoConfigToNvs() {
  if (!prefs.begin("xf551cfg", false)) { logf("[NVS] Error al abrir NVS auto"); return; }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUChar("auto1", g_driveTiming[0].autoEnabled ? 1 : 0);
  prefs.putUChar("auto2", g_driveTiming[1].autoEnabled ? 1 : 0);
  prefs.putUChar("auto3", g_driveTiming[2].autoEnabled ? 1 : 0);
  prefs.putUChar("auto4", g_driveTiming[3].autoEnabled ? 1 : 0);
  prefs.end();
  logf("[NVS] Auto-ajuste por unidad guardado");
}

void saveCommConfigToNvs() {
  if (!prefs.begin("xf551cfg", false)) { logf("[NVS] Error NVS comm"); return; }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUInt("uartBd", CFG_UART_BAUD);
  prefs.putUInt("sioRP",  CFG_RP_SIO_BAUD);
  prefs.putUInt("sioXF",  CFG_XF_SIO_BAUD);
  prefs.putUInt("netDly", CFG_NET_DELAY_US);
  prefs.end();
  logf("[NVS] Comm guardado");
}

void saveVerifyConfigToNvs() {
  if (!prefs.begin("xf551cfg", false)) { logf("[NVS] Error NVS verify"); return; }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUChar("vflg", CFG_VERIFY_FLAGS);
  prefs.end();
  logf("[NVS] Verify flags guardados");
}

void saveBtDiskConfigToNvs() {
  if (!prefs.begin("xf551cfg", false)) { logf("[NVS] Error NVS BT-DISK"); return; }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putBool("btEn", BT_DISK_ENABLED);
  prefs.putUChar("btMask", BT_DISK_DEV_MASK & 0x0F);
  prefs.putUChar("btForce", BT_DISK_FORCE_MASK & 0x0F);
  prefs.putUChar("drvMask", DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK);
  prefs.putUInt("btBaud", BT_DISK_UART_BAUD_CFG);
  prefs.putBool("bsioEn", BT_SIO2PC_ENABLED);
  prefs.putUChar("bsioMask", BT_SIO2PC_DEV_MASK & DRIVE_UI_MAX_MASK);
  prefs.putUChar("bsioForce", BT_SIO2PC_FORCE_MASK & DRIVE_UI_MAX_MASK);
  prefs.putUChar("bsioProf", BT_SIO2PC_PROFILE);
  for (int i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
    char k[10];
    snprintf(k, sizeof(k), "bsioP%d", i + 1);
    prefs.putUChar(k, BT_SIO2PC_UNIT_PROFILE[i]);
  }
  prefs.putBool("bsioQuiet", BT_SIO2PC_QUIET_LOG);
  prefs.putBool("bsioCache", BT_SIO2PC_CACHE_ENABLED);
  prefs.end();
  logf("[NVS] BT-DISK guardado en=%u mask=0x%02X force=0x%02X baud=%lu",
       BT_DISK_ENABLED ? 1 : 0,
       (unsigned)(BT_DISK_DEV_MASK & 0x0F),
       (unsigned)(BT_DISK_FORCE_MASK & 0x0F),
       (unsigned long)BT_DISK_UART_BAUD_CFG);
}

void saveWebAtrConfigToNvs() {
  if (!prefs.begin("xf551cfg", false)) { logf("[NVS] Error NVS WEB-ATR"); return; }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putBool("atrEn", WEB_ATR_ENABLED);
  prefs.putUChar("atrProfile", WEB_ATR_MOUNT_PROFILE);
  prefs.putUChar("drvMask", DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK);
  prefs.putUChar("atrMask", WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK);
  prefs.putUChar("atrForce", WEB_ATR_FORCE_MASK & DRIVE_UI_MAX_MASK);
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    char k[8];
    snprintf(k, sizeof(k), "atrD%d", i + 1);
    prefs.putString(k, g_webAtrMountedName[i]);
  }
  prefs.end();
  logf("[NVS] WEB-ATR guardado en=%u mask=0x%02X force=0x%02X visible=0x%02X",
       WEB_ATR_ENABLED ? 1 : 0,
       (unsigned)(WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK),
       (unsigned)(WEB_ATR_FORCE_MASK & DRIVE_UI_MAX_MASK),
       (unsigned)(DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK));
}



static void serviceDeferredConfigSaves() {
  if (!g_webAtrConfigSavePending && !g_btDiskConfigSavePending && !g_casConfigSavePending) return;
  int32_t remain = (int32_t)(g_deferredConfigSaveAtMs - millis());
  if (remain > 0) return;

  bool saveAtr = g_webAtrConfigSavePending;
  bool saveBt  = g_btDiskConfigSavePending;
  bool saveCas = g_casConfigSavePending;
  g_webAtrConfigSavePending = false;
  g_btDiskConfigSavePending = false;
  g_casConfigSavePending = false;

  if (saveAtr) saveWebAtrConfigToNvs();
  if (saveBt) saveBtDiskConfigToNvs();
  if (saveCas) saveCasConfigToNvs();
}


void savePrinterConfigToNvs() {
  if (!prefs.begin("printer", false)) {
    logf("[PRN] Error NVS printer");
    return;
  }

  prefs.putBool("en", PRN_CFG.enabled);
  prefs.putUChar("dev", PRN_CFG.sioDev);
  prefs.putUChar("mode", PRN_CFG.mode);
  prefs.putString("ip", PRN_CFG.ip);
  prefs.putUShort("port", PRN_CFG.port);
  prefs.putString("gw", PRN_CFG.gateway);
  prefs.putString("name", PRN_CFG.name);
  prefs.putBool("atascii", PRN_CFG.atasciiToAscii);
  prefs.putBool("crlf", PRN_CFG.appendCrLf);
  prefs.putBool("cut40", PRN_CFG.cut40);
  prefs.putUChar("vprof", PRN_CFG.virtualProfile);
  prefs.putUChar("vfont", PRN_CFG.virtualFont);
  prefs.putUShort("custCols", PRN_CFG.customColumns);
  prefs.putUShort("custRows", PRN_CFG.customRows);
  prefs.putUChar("fontScale", PRN_CFG.fontScale);
  prefs.putUChar("orient", PRN_CFG.pageOrientation);
  prefs.putUChar("paper", PRN_CFG.paperSize);
  prefs.putUChar("compose", PRN_CFG.composeMode);
  prefs.putUChar("rendQ", PRN_CFG.renderQuality);
  prefs.putBool("autoPrn", PRN_CFG.autoPrintSpool);
  prefs.putUShort("autoMs", PRN_CFG.autoPrintDelayMs);
  prefs.putBool("rawFF", PRN_CFG.rawTestFormFeed);
  prefs.putUShort("rawDelay", PRN_CFG.rawCloseDelayMs);
  prefs.putBool("staEn", PRN_CFG.staEnabled);
  prefs.putString("staSsid", PRN_CFG.staSsid);
  prefs.putString("staPass", PRN_CFG.staPass);
  prefs.end();

  logf("[PRN] Config impresora guardada");
}

void loadPrinterConfigFromNvs() {
  if (!prefs.begin("printer", true)) {
    logf("[PRN] No se pudo abrir NVS printer");
    return;
  }

  PRN_CFG.enabled = prefs.getBool("en", PRN_CFG.enabled);
  PRN_CFG.sioDev  = prefs.getUChar("dev", PRN_CFG.sioDev);
  PRN_CFG.mode    = prefs.getUChar("mode", PRN_CFG.mode);

  String ip = prefs.getString("ip", PRN_CFG.ip);
  ip.toCharArray(PRN_CFG.ip, sizeof(PRN_CFG.ip));

  PRN_CFG.port = prefs.getUShort("port", PRN_CFG.port);

  String gw = prefs.getString("gw", PRN_CFG.gateway);
  gw.toCharArray(PRN_CFG.gateway, sizeof(PRN_CFG.gateway));

  String nm = prefs.getString("name", PRN_CFG.name);
  nm.toCharArray(PRN_CFG.name, sizeof(PRN_CFG.name));

  PRN_CFG.atasciiToAscii = prefs.getBool("atascii", PRN_CFG.atasciiToAscii);
  PRN_CFG.appendCrLf       = prefs.getBool("crlf", PRN_CFG.appendCrLf);
  PRN_CFG.cut40            = prefs.getBool("cut40", PRN_CFG.cut40);
  PRN_CFG.virtualProfile   = prefs.getUChar("vprof", PRN_CFG.virtualProfile);
  PRN_CFG.virtualFont      = prefs.getUChar("vfont", PRN_CFG.virtualFont);
  PRN_CFG.customColumns    = prefs.getUShort("custCols", PRN_CFG.customColumns);
  PRN_CFG.customRows       = prefs.getUShort("custRows", PRN_CFG.customRows);
  PRN_CFG.fontScale        = prefs.getUChar("fontScale", PRN_CFG.fontScale);
  PRN_CFG.pageOrientation  = prefs.getUChar("orient", PRN_CFG.pageOrientation);
  PRN_CFG.paperSize        = prefs.getUChar("paper", PRN_CFG.paperSize);
  PRN_CFG.composeMode      = prefs.getUChar("compose", PRN_CFG.composeMode);
  PRN_CFG.renderQuality    = prefs.getUChar("rendQ", PRN_CFG.renderQuality);
  PRN_CFG.autoPrintSpool   = prefs.getBool("autoPrn", PRN_CFG.autoPrintSpool);
  PRN_CFG.autoPrintDelayMs = prefs.getUShort("autoMs", PRN_CFG.autoPrintDelayMs);
  PRN_CFG.rawTestFormFeed  = prefs.getBool("rawFF", PRN_CFG.rawTestFormFeed);
  PRN_CFG.rawCloseDelayMs  = prefs.getUShort("rawDelay", PRN_CFG.rawCloseDelayMs);
  PRN_CFG.staEnabled       = prefs.getBool("staEn", PRN_CFG.staEnabled);

  String ssid = prefs.getString("staSsid", PRN_CFG.staSsid);
  ssid.toCharArray(PRN_CFG.staSsid, sizeof(PRN_CFG.staSsid));

  String pass = prefs.getString("staPass", PRN_CFG.staPass);
  pass.toCharArray(PRN_CFG.staPass, sizeof(PRN_CFG.staPass));

  prefs.end();

  if (PRN_CFG.sioDev < 0x40 || PRN_CFG.sioDev > 0x43) PRN_CFG.sioDev = 0x40;
  if (PRN_CFG.mode < PRN_MODE_RAW9100 || PRN_CFG.mode > PRN_MODE_IPP_JPEG) PRN_CFG.mode = PRN_MODE_RAW9100;

  // V121: mode=2 vuelve a ser Gateway Windows real.
  // No migrar a IPP/JPEG: el spool Atari se envía al PC por HTTP.

  if (PRN_CFG.virtualProfile >= VIRTUAL_PRINTER_PROFILE_COUNT) PRN_CFG.virtualProfile = VP_ATARI_820_40;
  if (PRN_CFG.virtualFont > VF_ATASCII_MEJORADA_12X16) PRN_CFG.virtualFont = VF_ATASCII_ORIGINAL_8X8;
  PRN_CFG.customColumns = 0; // V69: columnas manuales eliminadas
  PRN_CFG.customRows = 0;    // V69: filas manuales eliminadas
  if (!(PRN_CFG.fontScale == 8 || PRN_CFG.fontScale == 10 || PRN_CFG.fontScale == 12 || PRN_CFG.fontScale == 14 || PRN_CFG.fontScale == 16)) PRN_CFG.fontScale = 12; // V78: migra NVS vieja 0/1/2/3/4 a 12pt
  if (PRN_CFG.pageOrientation > VP_ORIENT_LANDSCAPE) PRN_CFG.pageOrientation = VP_ORIENT_AUTO;
  if (PRN_CFG.paperSize > VP_PAPER_LETTER) PRN_CFG.paperSize = VP_PAPER_A4;
  if (PRN_CFG.composeMode > PRN_COMPOSE_EXTENDIDA) PRN_CFG.composeMode = PRN_COMPOSE_EXTENDIDA;
  if (PRN_CFG.renderQuality > PRN_RENDER_COMPAT_JPEG) PRN_CFG.renderQuality = PRN_RENDER_COMPAT_JPEG;
  // V120: solo JPEG estable. Ignora valores antiguos NVS que seleccionaban PWG/URF LAB.
  PRN_CFG.renderQuality = PRN_RENDER_COMPAT_JPEG;
  if (PRN_CFG.autoPrintDelayMs < 1000) PRN_CFG.autoPrintDelayMs = 5000;
  if (PRN_CFG.autoPrintDelayMs > 30000) PRN_CFG.autoPrintDelayMs = 30000;
  if (PRN_CFG.port == 0) PRN_CFG.port = 9100;
  if (PRN_CFG.rawCloseDelayMs > 5000) PRN_CFG.rawCloseDelayMs = 500;

  logf("[PRN] Config cargada enabled=%u dev=0x%02X mode=%u ip=%s port=%u",
       PRN_CFG.enabled ? 1 : 0,
       PRN_CFG.sioDev,
       PRN_CFG.mode,
       PRN_CFG.ip,
       PRN_CFG.port);
}

void loadConfigFromNvs() {
  if (!prefs.begin("xf551cfg", false)) { logf("[NVS] No se pudo abrir NVS"); return; }

  uint32_t magic = prefs.getUInt("magic", 0);
  if (magic != CFG_MAGIC) {
    prefs.putUInt("magic", CFG_MAGIC);
    prefs.putUShort("t_ack", T_ACK_TO_COMPLETE);
    prefs.putUShort("t_comp", T_COMPLETE_TO_DATA);
    prefs.putUShort("t_chk", T_DATA_TO_CHK);
    prefs.putUShort("t_chd", T_CHUNK_DELAY);
    prefs.putUChar("pf1", prefetchCfg[0]);
    prefs.putUChar("pf2", prefetchCfg[1]);
    prefs.putUChar("pf3", prefetchCfg[2]);
    prefs.putUChar("pf4", prefetchCfg[3]);
    prefs.putUChar("autoProf", (uint8_t)g_autoProfile);
    prefs.putUChar("auto1", 0);
    prefs.putUChar("auto2", 0);
    prefs.putUChar("auto3", 0);
    prefs.putUChar("auto4", 0);

    // defaults comm/verify
    prefs.putUInt("uartBd", CFG_UART_BAUD);
    prefs.putUInt("sioRP",  CFG_RP_SIO_BAUD);
    prefs.putUInt("sioXF",  CFG_XF_SIO_BAUD);
    prefs.putUInt("netDly", CFG_NET_DELAY_US);
    prefs.putUChar("vflg",  CFG_VERIFY_FLAGS);
    prefs.putBool("btEn", BT_DISK_ENABLED);
    prefs.putUChar("drvMask", DRIVE_VISIBLE_MASK);
    prefs.putUChar("btMask", BT_DISK_DEV_MASK);
    prefs.putUChar("btForce", BT_DISK_FORCE_MASK);
    prefs.putUInt("btBaud", BT_DISK_UART_BAUD_CFG);
    prefs.putBool("bsioEn", BT_SIO2PC_ENABLED);
    prefs.putUChar("bsioMask", BT_SIO2PC_DEV_MASK);
    prefs.putUChar("bsioForce", BT_SIO2PC_FORCE_MASK);
    prefs.putUChar("bsioProf", BT_SIO2PC_PROFILE);
    for (int i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
      char k[10];
      snprintf(k, sizeof(k), "bsioP%d", i + 1);
      prefs.putUChar(k, BT_SIO2PC_UNIT_PROFILE[i]);
    }
    prefs.putBool("bsioQuiet", BT_SIO2PC_QUIET_LOG);
    prefs.putBool("bsioCache", BT_SIO2PC_CACHE_ENABLED);
    prefs.putUChar("bsioSafeV", 2);
    prefs.putBool("atrEn", false);
    prefs.putUChar("atrProfile", WEB_ATR_MOUNT_PROFILE);
    prefs.putUChar("atrMask", 0);
    prefs.putUChar("atrForce", 0);
    for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
      char k[8]; snprintf(k, sizeof(k), "atrD%d", i + 1); prefs.putString(k, "");
    }

    prefs.end();
    logf("[NVS] Config inicial grabada");
    return;
  }

  T_ACK_TO_COMPLETE   = clampU16(prefs.getUShort("t_ack",  T_ACK_TO_COMPLETE), 120, 1200);
  T_COMPLETE_TO_DATA  = clampU16(prefs.getUShort("t_comp", T_COMPLETE_TO_DATA), 80, 900);
  T_DATA_TO_CHK       = clampU16(prefs.getUShort("t_chk",  T_DATA_TO_CHK), 15, 200);
  T_CHUNK_DELAY       = clampU16(prefs.getUShort("t_chd",  T_CHUNK_DELAY), 60, 1200);

  prefetchCfg[0] = prefs.getUChar("pf1", prefetchCfg[0]);
  prefetchCfg[1] = prefs.getUChar("pf2", prefetchCfg[1]);
  prefetchCfg[2] = prefs.getUChar("pf3", prefetchCfg[2]);
  prefetchCfg[3] = prefs.getUChar("pf4", prefetchCfg[3]);

  g_autoProfile = prefs.getUChar("autoProf", (uint8_t)g_autoProfile);

  g_driveTiming[0].autoEnabled = prefs.getUChar("auto1", 0) != 0;
  g_driveTiming[1].autoEnabled = prefs.getUChar("auto2", 0) != 0;
  g_driveTiming[2].autoEnabled = prefs.getUChar("auto3", 0) != 0;
  g_driveTiming[3].autoEnabled = prefs.getUChar("auto4", 0) != 0;

  // comm/verify
  CFG_UART_BAUD    = prefs.getUInt("uartBd", CFG_UART_BAUD);
  CFG_RP_SIO_BAUD  = prefs.getUInt("sioRP",  CFG_RP_SIO_BAUD);
  CFG_XF_SIO_BAUD  = prefs.getUInt("sioXF",  CFG_XF_SIO_BAUD);
  CFG_NET_DELAY_US = prefs.getUInt("netDly", CFG_NET_DELAY_US);
  CFG_VERIFY_FLAGS = prefs.getUChar("vflg",  CFG_VERIFY_FLAGS);

  BT_DISK_ENABLED = prefs.getBool("btEn", BT_DISK_ENABLED);
  BT_DISK_DEV_MASK = prefs.getUChar("btMask", BT_DISK_DEV_MASK) & 0x0F;
  BT_DISK_FORCE_MASK = prefs.getUChar("btForce", BT_DISK_FORCE_MASK) & 0x0F;
  BT_DISK_UART_BAUD_CFG = prefs.getUInt("btBaud", BT_DISK_UART_BAUD_CFG);
  if (BT_DISK_UART_BAUD_CFG < 9600 || BT_DISK_UART_BAUD_CFG > 921600) BT_DISK_UART_BAUD_CFG = 115200;
  BT_SIO2PC_ENABLED = prefs.getBool("bsioEn", BT_SIO2PC_ENABLED);
  DRIVE_VISIBLE_MASK = driveUiClampMask(prefs.getUChar("drvMask", DRIVE_UI_DEFAULT_VISIBLE_MASK));
  BT_SIO2PC_DEV_MASK = prefs.getUChar("bsioMask", BT_SIO2PC_DEV_MASK) & DRIVE_UI_MAX_MASK;
  BT_SIO2PC_FORCE_MASK = prefs.getUChar("bsioForce", BT_SIO2PC_FORCE_MASK) & DRIVE_UI_MAX_MASK;
  btSio2pcApplyProfile(prefs.getUChar("bsioProf", BT_SIO2PC_PROFILE));
  for (int i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
    char k[10];
    snprintf(k, sizeof(k), "bsioP%d", i + 1);
    BT_SIO2PC_UNIT_PROFILE[i] = prefs.getUChar(k, BT_SIO2PC_UNIT_PROFILE[i]);
    if (BT_SIO2PC_UNIT_PROFILE[i] > 3) BT_SIO2PC_UNIT_PROFILE[i] = 1;
  }
  BT_SIO2PC_QUIET_LOG = prefs.getBool("bsioQuiet", BT_SIO2PC_QUIET_LOG);
  BT_SIO2PC_CACHE_ENABLED = prefs.getBool("bsioCache", BT_SIO2PC_CACHE_ENABLED);

  // F47: una sola migración de seguridad para instalaciones que venían con fast/loader/turbo desde F46.
  // Evita que NVS viejo mantenga perfiles agresivos luego de flashear esta versión.
  uint8_t bsioSafeV = prefs.getUChar("bsioSafeV", 0);
  if (bsioSafeV < 2) {
    BT_SIO2PC_PROFILE = 0;
    for (int i = 0; i < DRIVE_UI_MAX_UNITS; i++) BT_SIO2PC_UNIT_PROFILE[i] = 0;
    BT_SIO2PC_CACHE_ENABLED = false;
    btSio2pcApplyProfile(0);
    prefs.putUChar("bsioProf", 0);
    for (int i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
      char k[10];
      snprintf(k, sizeof(k), "bsioP%d", i + 1);
      prefs.putUChar(k, 0);
    }
    prefs.putBool("bsioCache", false);
    prefs.putUChar("bsioSafeV", 2);
    logf("[BT-SIO2PC] F47 migración segura: perfil stable y caché OFF");
  }

  uint8_t atrProfile = prefs.getUChar("atrProfile", 0);
  if (atrProfile != WEB_ATR_MOUNT_PROFILE) {
    // Primera ejecución de esta versión: limpiar montajes heredados.
    // Los ATR ya subidos en la biblioteca se conservan, pero D1..D4 parten apagadas y sin disco montado.
    WEB_ATR_ENABLED = false;
    WEB_ATR_DEV_MASK = 0;
    WEB_ATR_FORCE_MASK = 0;
    for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) { g_webAtrMountedName[i] = ""; webAtrResetResolvedSlot(i); webAtrCacheInvalidateDev((uint8_t)(0x31 + i)); }
    prefs.putBool("atrEn", false);
    prefs.putUChar("atrProfile", WEB_ATR_MOUNT_PROFILE);
    prefs.putUChar("atrMask", 0);
    prefs.putUChar("atrForce", 0);
    for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
      char k[8]; snprintf(k, sizeof(k), "atrD%d", i + 1); prefs.putString(k, "");
    }
    logf("[WEB-ATR] Montaje inicial limpio: D1-D4 deshabilitadas y sin ATR montado");
  }

  WEB_ATR_ENABLED = prefs.getBool("atrEn", WEB_ATR_ENABLED);
  WEB_ATR_DEV_MASK = prefs.getUChar("atrMask", WEB_ATR_DEV_MASK) & DRIVE_UI_MAX_MASK;
  WEB_ATR_FORCE_MASK = prefs.getUChar("atrForce", WEB_ATR_FORCE_MASK) & DRIVE_UI_MAX_MASK;
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    char k[8];
    snprintf(k, sizeof(k), "atrD%d", i + 1);
    String v = prefs.getString(k, "");
    webAtrResetResolvedSlot(i);
    g_webAtrMountedName[i] = v.length() ? webAtrSanitizeFileName(v) : String("");
  }
  normalizeDriveMasks();

  prefs.end();
}

// ===== UART TX -> RP2040 =====
void sendUartFrameToRP(const uint8_t* payload, uint8_t len) {
  if (!payload || len == 0) return;
  uint8_t chk = calcChecksum(payload, len);

  Serial2.write(UART_SYNC);
  Serial2.write(len);
  Serial2.write(payload, len);
  Serial2.write(chk);
#if MASTER_UART_FLUSH_EACH_FRAME
  Serial2.flush();
#endif
}

static inline void putLE16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline uint16_t getLE16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void putLE32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline uint32_t getLE32(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

// Envia tiempos actuales al RP2040 (para que aplique delays SIO)
void sendTimingUpdateToRP() {
  uint8_t p[1 + 8];
  p[0] = TYPE_TIMING_UPDATE;

  putLE16(&p[1], T_ACK_TO_COMPLETE);
  putLE16(&p[3], T_COMPLETE_TO_DATA);
  putLE16(&p[5], T_DATA_TO_CHK);
  putLE16(&p[7], T_CHUNK_DELAY);

  sendUartFrameToRP(p, (uint8_t)sizeof(p));

  logf("[MASTER] TIMING_UPDATE -> RP ack2comp=%u comp2data=%u data2chk=%u chunk=%u",
       (unsigned)T_ACK_TO_COMPLETE, (unsigned)T_COMPLETE_TO_DATA,
       (unsigned)T_DATA_TO_CHK, (unsigned)T_CHUNK_DELAY);
}

void sendTimingUpdateToRPThrottled(bool force) {
  static uint32_t lastSentMs = 0;
  static uint16_t lastAck = 0, lastComp = 0, lastChk = 0, lastChunk = 0;
  bool changed = (lastAck != T_ACK_TO_COMPLETE) ||
                 (lastComp != T_COMPLETE_TO_DATA) ||
                 (lastChk != T_DATA_TO_CHK) ||
                 (lastChunk != T_CHUNK_DELAY);
  if (!force && !changed && (millis() - lastSentMs) < 3000) return;

  sendTimingUpdateToRP();
  lastAck = T_ACK_TO_COMPLETE;
  lastComp = T_COMPLETE_TO_DATA;
  lastChk = T_DATA_TO_CHK;
  lastChunk = T_CHUNK_DELAY;
  lastSentMs = millis();
}

// ===== NUEVO: CFG -> RP / SLAVES =====
void sendCfgUpdateToRP(uint32_t uartBaud, uint32_t rpSioBaud) {
  uint8_t p[1 + 8];
  p[0] = TYPE_CFG_UPDATE;
  putLE32(&p[1], uartBaud);
  putLE32(&p[5], rpSioBaud);
  sendUartFrameToRP(p, (uint8_t)sizeof(p));

  logf("[MASTER] CFG_UPDATE -> RP uart=%lu rpSio=%lu",
       (unsigned long)uartBaud, (unsigned long)rpSioBaud);
}


void sendPrinterCfgToRP() {
  uint8_t p[3];
  p[0] = TYPE_PRINTER_CFG_UPDATE;
  p[1] = PRN_CFG.enabled ? 1 : 0;
  p[2] = PRN_CFG.sioDev;

  sendUartFrameToRP(p, sizeof(p));

  logf("[PRN] CFG -> RP enabled=%u dev=0x%02X",
       PRN_CFG.enabled ? 1 : 0,
       PRN_CFG.sioDev);
}

void broadcastSlaveCfg() {
  static uint32_t lastSentMs = 0;
  static uint32_t lastSio = 0xFFFFFFFFUL;
  static uint32_t lastNet = 0xFFFFFFFFUL;
  static uint8_t  lastFlags = 0xFF;

  bool changed = (lastSio != CFG_XF_SIO_BAUD) ||
                 (lastNet != CFG_NET_DELAY_US) ||
                 (lastFlags != CFG_VERIFY_FLAGS);
  if (!changed && (uint32_t)(millis() - lastSentMs) < 3000UL) {
    return;
  }

  uint8_t p[11];
  p[0] = TYPE_CFG_UPDATE;
  p[1] = 0x00; // todos
  putLE32(&p[2], CFG_XF_SIO_BAUD);
  putLE32(&p[6], CFG_NET_DELAY_US);
  p[10] = CFG_VERIFY_FLAGS;

  sendEspNow(BCAST_MAC, p, sizeof(p));
  lastSio = CFG_XF_SIO_BAUD;
  lastNet = CFG_NET_DELAY_US;
  lastFlags = CFG_VERIFY_FLAGS;
  lastSentMs = millis();

  logf("[MASTER] CFG_UPDATE -> SLAVES xfSio=%lu netDelayUs=%lu vflg=0x%02X",
       (unsigned long)CFG_XF_SIO_BAUD, (unsigned long)CFG_NET_DELAY_US,
       (unsigned)CFG_VERIFY_FLAGS);
}

bool pushCfgToRPAndMaybeSwitchUart(); // fwd


void connectPrinterStaWifi(bool waitForConnection) {
  if (!PRN_CFG.staEnabled) {
    return;
  }

  if (strlen(PRN_CFG.staSsid) == 0) {
    g_prnLastError = "WiFi STA habilitado, pero SSID vacío";
    logf("[WIFI-LAN] %s", g_prnLastError.c_str());
    return;
  }

  if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == String(PRN_CFG.staSsid)) {
    startMdnsIfStaReady();
    return;
  }

  // El MASTER mantiene AP + STA: AP 192.168.50.1 queda como respaldo,
  // y STA permite acceder al panel desde la red de casa.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  esp_wifi_set_ps(WIFI_PS_NONE);

  logf("[WIFI-LAN] Conectando STA a SSID '%s'", PRN_CFG.staSsid);
  WiFi.begin(PRN_CFG.staSsid, PRN_CFG.staPass);

  if (!waitForConnection) return;

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
    delay(250);
    server.handleClient();
  }

  if (WiFi.status() == WL_CONNECTED) {
    logf("[WIFI-LAN] STA conectado ip=%s ssid=%s mdns=http://%s.local status=%s",
         WiFi.localIP().toString().c_str(),
         WiFi.SSID().c_str(),
         MASTER_MDNS_HOST,
         printerWifiStatusText().c_str());
    startMdnsIfStaReady();
  } else {
    g_prnLastError = "No conecta STA a WiFi LAN: " + printerWifiStatusText();
    logf("[WIFI-LAN] %s", g_prnLastError.c_str());
  }
}

static void startMdnsIfStaReady() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (g_mdnsStarted) return;

  if (MDNS.begin(MASTER_MDNS_HOST)) {
    g_mdnsStarted = true;
    MDNS.addService("http", "tcp", 80);
    logf("[MDNS] Disponible como http://%s.local", MASTER_MDNS_HOST);
  } else {
    logf("[MDNS] No se pudo iniciar mDNS host=%s", MASTER_MDNS_HOST);
  }
}

static String buildWifiStatusJson(bool includeOk) {
  const bool staConnected = (WiFi.status() == WL_CONNECTED);
  String json = "{";
  if (includeOk) json += "\"ok\":1,";
  json += "\"mode\":\"AP_STA\",";
  json += "\"apSsid\":\"" + jsonEscape(String(MASTER_AP_SSID)) + "\",";
  json += "\"apIp\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\",";
  json += "\"apUrl\":\"" + jsonEscape(String("http://") + WiFi.softAPIP().toString()) + "\",";
  json += "\"staEnabled\":" + String(PRN_CFG.staEnabled ? 1 : 0) + ",";
  json += "\"staSsid\":\"" + jsonEscape(String(PRN_CFG.staSsid)) + "\",";
  json += "\"staConnected\":" + String(staConnected ? 1 : 0) + ",";
  json += "\"staIp\":\"" + jsonEscape(staConnected ? WiFi.localIP().toString() : String("")) + "\",";
  json += "\"staUrl\":\"" + jsonEscape(staConnected ? String("http://") + WiFi.localIP().toString() : String("")) + "\",";
  json += "\"mdnsHost\":\"" + jsonEscape(String(MASTER_MDNS_HOST)) + "\",";
  json += "\"mdnsUrl\":\"" + jsonEscape(String("http://") + String(MASTER_MDNS_HOST) + String(".local")) + "\",";
  json += "\"wifiStatus\":\"" + jsonEscape(printerWifiStatusText()) + "\",";
  json += "\"deferred\":" + String(g_staConnectDeferred ? 1 : 0);
  json += "}";
  return json;
}

void handleApiWifiStatus() {
  server.send(200, "application/json", buildWifiStatusJson(true));
}

void handleApiWifiSet() {
  if (server.hasArg("enabled")) PRN_CFG.staEnabled = server.arg("enabled").toInt() != 0;
  if (server.hasArg("staEnabled")) PRN_CFG.staEnabled = server.arg("staEnabled").toInt() != 0;
  if (server.hasArg("ssid")) server.arg("ssid").toCharArray(PRN_CFG.staSsid, sizeof(PRN_CFG.staSsid));
  if (server.hasArg("staSsid")) server.arg("staSsid").toCharArray(PRN_CFG.staSsid, sizeof(PRN_CFG.staSsid));
  if (server.hasArg("pass") && server.arg("pass").length() > 0) server.arg("pass").toCharArray(PRN_CFG.staPass, sizeof(PRN_CFG.staPass));
  if (server.hasArg("staPass") && server.arg("staPass").length() > 0) server.arg("staPass").toCharArray(PRN_CFG.staPass, sizeof(PRN_CFG.staPass));

  savePrinterConfigToNvs();

  if (PRN_CFG.staEnabled && strlen(PRN_CFG.staSsid) > 0) {
    connectPrinterStaWifi(true);
  } else {
    WiFi.disconnect(false, false); // mantiene SoftAP de respaldo
    g_mdnsStarted = false;
  }

  server.send(200, "application/json", buildWifiStatusJson(true));
}

String atasciiToAsciiLine(const uint8_t* data, uint8_t len) {
  String out;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = data[i];

    if (b == 0x9B || b == 0x0D || b == 0x0A) {
      out += '\n';
      continue;
    }

    // Rango ASCII imprimible básico.
    if (b >= 32 && b <= 126) {
      out += (char)b;
      continue;
    }

    // Inverse video / ATASCII alto: degradación simple a ASCII imprimible.
    if (b >= 160 && b <= 254) {
      out += (char)(b - 128);
      continue;
    }

    // Otros controles: espacio para no ensuciar salida moderna.
    out += ' ';
  }
  out.trim();
  return out;
}

// ================== ATASCII -> JPEG virtual printer ==================
// F49Z82: el fallback aproximado 5x7/charset interno anterior fue reemplazado
// por ATASCII_BUILTIN_DEFAULT_8X8 en PROGMEM.

// ================== V64: ATASCII glyph JSON / editor web 8x8 + web 3 paginas ==================
// Archivo esperado en almacenamiento principal: /atascii_glyphs.json
// - ATASCII_ORIGINAL_8X8: rows o rowsHex con 8 filas de 8 bits.
// - Legacy 9x9/12x16: se acepta para migrar, pero el renderer usa 8x8.
// Si no existe JSON o falta un glifo, se usa fallback interno Atari 8x8.
#if WEB_STORAGE_USE_SD
static const char* ATASCII_GLYPH_JSON_PATH = "/GLYPHS/atascii_glyphs.json";
#else
static const char* ATASCII_GLYPH_JSON_PATH = "/atascii_glyphs.json";
#endif
static bool g_fontFsReady = false;
static uint32_t g_fontLastLoadMs = 0;
static uint32_t g_fontReloadRequests = 0;
// F49Z76: evita que una recarga inmediata post-import pise la RAM recién importada
// con un archivo antiguo de SD/raíz mientras aún hay callbacks JS pendientes.
static uint32_t g_fontLastPersistMs = 0;
static uint32_t g_fontLastImportMs = 0;
static uint16_t g_fontLastImportCount = 0;
static const uint32_t GLYPH_RELOAD_GUARD_MS = 15000UL;

static bool webAtrFsReady() {
  if (g_fontFsReady) return true;
  g_fontFsReady = SPIFFS.begin(true);
  if (!g_fontFsReady) logf("[WEB-ATR] Almacenamiento no disponible");
  return g_fontFsReady;
}
static bool g_fontFilePresent = false;
static String g_fontLoadStatus = "sin JSON";
static uint16_t g_customFont8[128][8]; // V64: matriz ATASCII 8x8 real
static bool g_customFont8Valid[128];
static uint16_t g_customFont12[128][16];
static bool g_customFont12Valid[128];

static const uint8_t ATASCII_BUILTIN_DEFAULT_8X8[128][8] PROGMEM = {
  { 0x00, 0x36, 0x7F, 0x7F, 0x3E, 0x1C, 0x08, 0x00 }, // 0x00 ATASCII_0x00
  { 0x18, 0x18, 0x18, 0x1F, 0x1F, 0x18, 0x18, 0x18 }, // 0x01 ATASCII_0x01
  { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 }, // 0x02 ATASCII_0x02
  { 0x18, 0x18, 0x18, 0xF8, 0xF8, 0x00, 0x00, 0x00 }, // 0x03 ATASCII_0x03
  { 0x18, 0x18, 0x18, 0xF8, 0xF8, 0x18, 0x18, 0x18 }, // 0x04 ATASCII_0x04
  { 0x00, 0x00, 0x00, 0xF8, 0xF8, 0x18, 0x18, 0x18 }, // 0x05 ATASCII_0x05
  { 0x03, 0x07, 0x0E, 0x1C, 0x38, 0x70, 0xE0, 0xC0 }, // 0x06 ATASCII_0x06
  { 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03 }, // 0x07 ATASCII_0x07
  { 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF }, // 0x08 ATASCII_0x08
  { 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F }, // 0x09 ATASCII_0x09
  { 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF }, // 0x0A ATASCII_0x0A
  { 0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00 }, // 0x0B ATASCII_0x0B
  { 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00 }, // 0x0C ATASCII_0x0C
  { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x0D ATASCII_0x0D
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF }, // 0x0E ATASCII_0x0E
  { 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0 }, // 0x0F ATASCII_0x0F
  { 0x00, 0x1C, 0x1C, 0x77, 0x77, 0x08, 0x1C, 0x00 }, // 0x10 ATASCII_0x10
  { 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x18, 0x18, 0x18 }, // 0x11 ATASCII_0x11
  { 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00 }, // 0x12 ATASCII_0x12
  { 0x18, 0x18, 0x18, 0xFF, 0xFF, 0x18, 0x18, 0x18 }, // 0x13 ATASCII_0x13
  { 0x00, 0x00, 0x3C, 0x7E, 0x7E, 0x7E, 0x3C, 0x00 }, // 0x14 ATASCII_0x14
  { 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00 }, // 0x15 ATASCII_0x15
  { 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0 }, // 0x16 ATASCII_0x16
  { 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x18, 0x18, 0x18 }, // 0x17 ATASCII_0x17
  { 0x18, 0x18, 0x18, 0xFF, 0xFF, 0x00, 0x00, 0x00 }, // 0x18 ATASCII_0x18
  { 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0 }, // 0x19 ATASCII_0x19
  { 0x18, 0x18, 0x18, 0x1F, 0x1F, 0x00, 0x00, 0x00 }, // 0x1A ATASCII_0x1A
  { 0x78, 0x40, 0x78, 0x40, 0x7E, 0x18, 0x1E, 0x00 }, // 0x1B ATASCII_0x1B
  { 0x00, 0x18, 0x3C, 0x7E, 0x18, 0x18, 0x18, 0x00 }, // 0x1C ATASCII_0x1C
  { 0x00, 0x18, 0x18, 0x18, 0x7E, 0x3C, 0x18, 0x00 }, // 0x1D ATASCII_0x1D
  { 0x00, 0x18, 0x30, 0x7E, 0x30, 0x18, 0x00, 0x00 }, // 0x1E ATASCII_0x1E
  { 0x00, 0x18, 0x0C, 0x7E, 0x0C, 0x18, 0x00, 0x00 }, // 0x1F ATASCII_0x1F
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x20 ATASCII_0x20
  { 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18, 0x00 }, // 0x21 !
  { 0x00, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00 }, // 0x22 ATASCII_0x22
  { 0x00, 0x66, 0xFF, 0x66, 0x66, 0xFF, 0x66, 0x00 }, // 0x23 #
  { 0x18, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x18, 0x00 }, // 0x24 $
  { 0x00, 0x66, 0x6C, 0x18, 0x30, 0x66, 0x46, 0x00 }, // 0x25 %
  { 0x1C, 0x36, 0x1C, 0x38, 0x6F, 0x66, 0x3B, 0x00 }, // 0x26 &
  { 0x00, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00 }, // 0x27 '
  { 0x00, 0x0E, 0x1C, 0x18, 0x18, 0x1C, 0x0E, 0x00 }, // 0x28 (
  { 0x00, 0x70, 0x38, 0x18, 0x18, 0x38, 0x70, 0x00 }, // 0x29 )
  { 0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00 }, // 0x2A *
  { 0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x00, 0x00 }, // 0x2B +
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30 }, // 0x2C ,
  { 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00 }, // 0x2D -
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00 }, // 0x2E .
  { 0x00, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x40, 0x00 }, // 0x2F /
  { 0x00, 0x3C, 0x66, 0x6E, 0x76, 0x66, 0x3C, 0x00 }, // 0x30 0
  { 0x00, 0x18, 0x38, 0x18, 0x18, 0x18, 0x7E, 0x00 }, // 0x31 1
  { 0x00, 0x3C, 0x66, 0x0C, 0x18, 0x30, 0x7E, 0x00 }, // 0x32 2
  { 0x00, 0x7E, 0x0C, 0x18, 0x0C, 0x66, 0x3C, 0x00 }, // 0x33 3
  { 0x00, 0x0C, 0x1C, 0x3C, 0x6C, 0x7E, 0x0C, 0x00 }, // 0x34 4
  { 0x00, 0x7E, 0x60, 0x7C, 0x06, 0x66, 0x3C, 0x00 }, // 0x35 5
  { 0x00, 0x3C, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00 }, // 0x36 6
  { 0x00, 0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x00 }, // 0x37 7
  { 0x00, 0x3C, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00 }, // 0x38 8
  { 0x00, 0x3C, 0x66, 0x3E, 0x06, 0x0C, 0x38, 0x00 }, // 0x39 9
  { 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00 }, // 0x3A :
  { 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x30 }, // 0x3B ;
  { 0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00 }, // 0x3C <
  { 0x00, 0x00, 0x7E, 0x00, 0x00, 0x7E, 0x00, 0x00 }, // 0x3D =
  { 0x60, 0x30, 0x18, 0x0C, 0x18, 0x30, 0x60, 0x00 }, // 0x3E >
  { 0x00, 0x3C, 0x66, 0x0C, 0x18, 0x00, 0x18, 0x00 }, // 0x3F ?
  { 0x00, 0x3C, 0x66, 0x6E, 0x6E, 0x60, 0x3E, 0x00 }, // 0x40 @
  { 0x00, 0x18, 0x3C, 0x66, 0x66, 0x7E, 0x66, 0x00 }, // 0x41 A
  { 0x00, 0x7C, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00 }, // 0x42 B
  { 0x00, 0x3C, 0x66, 0x60, 0x60, 0x66, 0x3C, 0x00 }, // 0x43 C
  { 0x00, 0x78, 0x6C, 0x66, 0x66, 0x6C, 0x78, 0x00 }, // 0x44 D
  { 0x00, 0x7E, 0x60, 0x7C, 0x60, 0x60, 0x7E, 0x00 }, // 0x45 E
  { 0x00, 0x7E, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x00 }, // 0x46 F
  { 0x00, 0x3E, 0x60, 0x60, 0x6E, 0x66, 0x3E, 0x00 }, // 0x47 G
  { 0x00, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00 }, // 0x48 H
  { 0x00, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00 }, // 0x49 I
  { 0x00, 0x06, 0x06, 0x06, 0x06, 0x66, 0x3C, 0x00 }, // 0x4A J
  { 0x00, 0x66, 0x6C, 0x78, 0x78, 0x6C, 0x66, 0x00 }, // 0x4B K
  { 0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00 }, // 0x4C L
  { 0x00, 0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x00 }, // 0x4D M
  { 0x00, 0x66, 0x76, 0x7E, 0x6E, 0x66, 0x66, 0x00 }, // 0x4E N
  { 0x00, 0x3C, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00 }, // 0x4F O
  { 0x00, 0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x00 }, // 0x50 P
  { 0x00, 0x3C, 0x66, 0x66, 0x66, 0x6C, 0x36, 0x00 }, // 0x51 Q
  { 0x00, 0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x00 }, // 0x52 R
  { 0x00, 0x3C, 0x60, 0x3C, 0x06, 0x06, 0x3C, 0x00 }, // 0x53 S
  { 0x00, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00 }, // 0x54 T
  { 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7E, 0x00 }, // 0x55 U
  { 0x00, 0x66, 0x66, 0x66, 0x66, 0x24, 0x18, 0x00 }, // 0x56 V
  { 0x00, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00 }, // 0x57 W
  { 0x00, 0x66, 0x7E, 0x3C, 0x3C, 0x66, 0x66, 0x00 }, // 0x58 X
  { 0x00, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x00 }, // 0x59 Y
  { 0x00, 0x7E, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00 }, // 0x5A Z
  { 0x00, 0x1E, 0x18, 0x18, 0x18, 0x18, 0x1E, 0x00 }, // 0x5B [
  { 0x00, 0x40, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x00 }, // 0x5C ATASCII_0x5C
  { 0x00, 0x78, 0x18, 0x18, 0x18, 0x18, 0x78, 0x00 }, // 0x5D ]
  { 0x00, 0x08, 0x1C, 0x36, 0x63, 0x00, 0x00, 0x00 }, // 0x5E ^
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00 }, // 0x5F _
  { 0x00, 0x18, 0x3C, 0x7E, 0x7E, 0x3C, 0x18, 0x00 }, // 0x60 `
  { 0x00, 0x00, 0x3C, 0x06, 0x3E, 0x66, 0x3E, 0x00 }, // 0x61 a
  { 0x00, 0x60, 0x60, 0x7C, 0x66, 0x66, 0x7C, 0x00 }, // 0x62 b
  { 0x00, 0x00, 0x3C, 0x60, 0x60, 0x60, 0x3C, 0x00 }, // 0x63 c
  { 0x00, 0x06, 0x06, 0x3E, 0x66, 0x66, 0x3E, 0x00 }, // 0x64 d
  { 0x00, 0x00, 0x3C, 0x66, 0x7E, 0x60, 0x3C, 0x00 }, // 0x65 e
  { 0x00, 0x0E, 0x18, 0x3E, 0x18, 0x18, 0x18, 0x00 }, // 0x66 f
  { 0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x7C }, // 0x67 g
  { 0x00, 0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x00 }, // 0x68 h
  { 0x00, 0x18, 0x00, 0x38, 0x18, 0x18, 0x3C, 0x00 }, // 0x69 i
  { 0x00, 0x06, 0x00, 0x06, 0x06, 0x06, 0x06, 0x3C }, // 0x6A j
  { 0x00, 0x60, 0x60, 0x6C, 0x78, 0x6C, 0x66, 0x00 }, // 0x6B k
  { 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00 }, // 0x6C l
  { 0x00, 0x00, 0x66, 0x7F, 0x7F, 0x6B, 0x63, 0x00 }, // 0x6D m
  { 0x00, 0x00, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00 }, // 0x6E n
  { 0x00, 0x00, 0x3C, 0x66, 0x66, 0x66, 0x3C, 0x00 }, // 0x6F o
  { 0x00, 0x00, 0x7C, 0x7E, 0x66, 0x7C, 0x60, 0x60 }, // 0x70 p
  { 0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x06 }, // 0x71 q
  { 0x00, 0x00, 0x7C, 0x66, 0x60, 0x60, 0x60, 0x00 }, // 0x72 r
  { 0x00, 0x00, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x00 }, // 0x73 s
  { 0x00, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x0E, 0x00 }, // 0x74 t
  { 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3E, 0x00 }, // 0x75 u
  { 0x00, 0x00, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00 }, // 0x76 v
  { 0x00, 0x00, 0x63, 0x6B, 0x7F, 0x3E, 0x36, 0x00 }, // 0x77 w
  { 0x00, 0x00, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x00 }, // 0x78 x
  { 0x00, 0x00, 0x66, 0x66, 0x66, 0x3E, 0x0C, 0x78 }, // 0x79 y
  { 0x00, 0x00, 0x7E, 0x0C, 0x18, 0x30, 0x7E, 0x00 }, // 0x7A z
  { 0x00, 0x18, 0x3C, 0x7E, 0x7E, 0x18, 0x3C, 0x00 }, // 0x7B {
  { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 }, // 0x7C |
  { 0x00, 0x7E, 0x78, 0x7C, 0x6E, 0x66, 0x06, 0x00 }, // 0x7D }
  { 0x08, 0x18, 0x38, 0x78, 0x38, 0x18, 0x08, 0x00 }, // 0x7E ~
  { 0x10, 0x18, 0x1C, 0x1E, 0x1C, 0x18, 0x10, 0x00 }, // 0x7F ATASCII_0x7F
};


static const char* ATASCII_GLYPH_DEFAULT_MARKER_PATH = "/GLYPHS/default_f49z82.marker";

static void clearCustomFonts();

static uint8_t builtinDefaultGlyphRow8(uint8_t code, uint8_t y) {
  if (code >= 128 || y >= 8) return 0;
  return pgm_read_byte(&ATASCII_BUILTIN_DEFAULT_8X8[code & 0x7F][y & 0x07]);
}

static void loadBuiltInDefaultGlyphsToRam(const char* reason) {
  clearCustomFonts();
  for (uint8_t code = 0; code < 128; code++) {
    for (uint8_t y = 0; y < 8; y++) {
      g_customFont8[code][y] = builtinDefaultGlyphRow8(code, y);
    }
    g_customFont8Valid[code] = true;
  }
  g_fontFilePresent = true;
  g_fontLastLoadMs = millis();
  g_fontLoadStatus = String("APP_DEFAULT_F49Z82 8x8=128 reason=") + (reason ? reason : "default");
}

static void writeBuiltInDefaultMarker() {
  if (!g_fontFsReady) return;
  File f = SPIFFS.open(ATASCII_GLYPH_DEFAULT_MARKER_PATH, "w");
  if (f) {
    f.print("F49Z82_ATASCII_ORIGINAL_8X8\n");
    f.print("sha256=3fccbe4a0e990ded9927917938610880439f3af9f6451665a4c9a92dea05bd09\n");
    f.close();
  }
}

static uint8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'a' && c <= 'f') return (uint8_t)(10 + c - 'a');
  if (c >= 'A' && c <= 'F') return (uint8_t)(10 + c - 'A');
  return 0xFF;
}

static uint16_t parseHex16Str(const String& v) {
  uint16_t out = 0;
  for (uint16_t i = 0; i < v.length(); i++) {
    uint8_t h = hexVal(v[i]);
    if (h <= 0x0F) out = (uint16_t)((out << 4) | h);
  }
  return out;
}

static uint16_t parseBin16Str(const String& v) {
  uint16_t out = 0;
  for (uint16_t i = 0; i < v.length(); i++) {
    if (v[i] == '0' || v[i] == '1') out = (uint16_t)((out << 1) | (v[i] == '1' ? 1 : 0));
  }
  return out;
}

static bool jsonExtractQuotedAt(const String& s, int start, String& out, int& after) {
  int q1 = s.indexOf('"', start);
  if (q1 < 0) return false;
  int q2 = s.indexOf('"', q1 + 1);
  if (q2 < 0) return false;
  out = s.substring(q1 + 1, q2);
  after = q2 + 1;
  return true;
}

static void clearCustomFonts() {
  memset(g_customFont8, 0, sizeof(g_customFont8));
  memset(g_customFont8Valid, 0, sizeof(g_customFont8Valid));
  memset(g_customFont12, 0, sizeof(g_customFont12));
  memset(g_customFont12Valid, 0, sizeof(g_customFont12Valid));
}

static bool parseRowTokenV53(const String& raw, bool hexRows, uint16_t& out) {
  String t = raw;
  t.trim();
  if (t.length() == 0) return false;
  if (t.startsWith("\"") && t.endsWith("\"") && t.length() >= 2) t = t.substring(1, t.length() - 1);
  t.trim();
  if (t.startsWith("0b") || t.startsWith("0B")) {
    out = parseBin16Str(t.substring(2));
    return true;
  }
  bool only01 = true;
  for (uint16_t i = 0; i < t.length(); i++) if (t[i] != '0' && t[i] != '1') { only01 = false; break; }
  if (!hexRows && only01 && t.length() >= 8) {
    out = parseBin16Str(t);
    return true;
  }
  if (t.startsWith("0x") || t.startsWith("0X")) {
    out = parseHex16Str(t.substring(2));
    return true;
  }
  bool onlyDigits = true;
  for (uint16_t i = 0; i < t.length(); i++) if (t[i] < '0' || t[i] > '9') { onlyDigits = false; break; }
  if (!hexRows && onlyDigits && t.length() >= 3) {
    long v = t.toInt();
    if (v < 0) v = 0;
    if (v > 0xFFFF) v = 0xFFFF;
    out = (uint16_t)v;
    return true;
  }
  out = parseHex16Str(t);
  return true;
}

static int parseGlyphRowsArray(const String& body, int from, int limit, uint8_t wantedRows, bool hexRows, uint16_t* rowsOut) {
  const char* key = hexRows ? "\"rowsHex\"" : "\"rows\"";
  int p = body.indexOf(key, from);
  if (p < 0 || p > limit) return 0;
  int lb = body.indexOf('[', p);
  if (lb < 0 || lb > limit) return 0;
  int rb = body.indexOf(']', lb + 1);
  if (rb < 0 || rb > limit) return 0;
  int pos = lb + 1;
  uint8_t count = 0;
  while (count < wantedRows && pos < rb) {
    while (pos < rb && (body[pos] == ' ' || body[pos] == '\r' || body[pos] == '\n' || body[pos] == '\t' || body[pos] == ',')) pos++;
    if (pos >= rb) break;
    String token;
    if (body[pos] == '\"') {
      int q2 = body.indexOf('"', pos + 1);
      if (q2 < 0 || q2 > rb) break;
      token = body.substring(pos + 1, q2);
      pos = q2 + 1;
    } else {
      int end = pos;
      while (end < rb && body[end] != ',' && body[end] != ']' && body[end] != '\r' && body[end] != '\n' && body[end] != '\t' && body[end] != ' ') end++;
      token = body.substring(pos, end);
      pos = end + 1;
    }
    uint16_t v = 0;
    if (parseRowTokenV53(token, hexRows, v)) rowsOut[count++] = v;
  }
  return count;
}

static uint8_t parseOneFontFromJson(const String& body, const char* fontId, uint8_t width, uint8_t height) {
  int fpos = body.indexOf(fontId);
  if (fpos < 0) return 0;
  int nextFont = body.indexOf("\"id\"", fpos + 8);
  if (nextFont < 0) nextFont = body.length();
  int pos = body.indexOf("\"code\"", fpos);
  uint8_t loaded = 0;
  while (pos >= 0 && pos < nextFont) {
    String codeStr;
    int afterCode = 0;
    int colon = body.indexOf(':', pos);
    if (colon < 0 || colon > nextFont || !jsonExtractQuotedAt(body, colon + 1, codeStr, afterCode)) break;
    int nextCode = body.indexOf("\"code\"", afterCode);
    int glyphLimit = (nextCode > 0 && nextCode < nextFont) ? nextCode : nextFont;
    uint8_t code = (uint8_t)(parseHex16Str(codeStr) & 0x7F);
    uint16_t rows16[16];
    memset(rows16, 0, sizeof(rows16));
    int n = parseGlyphRowsArray(body, afterCode, glyphLimit, height, true, rows16);
    if (n != height) n = parseGlyphRowsArray(body, afterCode, glyphLimit, height, false, rows16);
    if (n == height) {
      if (width == 8 && height == 8) {
        for (uint8_t y = 0; y < 8; y++) g_customFont8[code][y] = (uint16_t)(rows16[y] & 0x00FF);
        g_customFont8Valid[code] = true;
        loaded++;
      } else if (width == 9 && height == 9) {
        // Migración legacy: 9x9 -> 8x8, recupera la matriz anterior desplazada y descarta fila/columna extra.
        for (uint8_t y = 0; y < 8; y++) g_customFont8[code][y] = (uint16_t)((rows16[y] >> 1) & 0x00FF);
        g_customFont8Valid[code] = true;
        loaded++;
      } else if (width == 12 && height == 16) {
        for (uint8_t y = 0; y < 16; y++) g_customFont12[code][y] = (uint16_t)(rows16[y] & 0x0FFF);
        g_customFont12Valid[code] = true;
        loaded++;
      }
    }
    pos = (nextCode > 0) ? nextCode : -1;
  }
  return loaded;
}

static bool parseAtasciiGlyphJson(const String& body) {
  clearCustomFonts();
  uint8_t n9 = parseOneFontFromJson(body, "ATASCII_ORIGINAL_8X8", 8, 8);
  if (n9 == 0) n9 = parseOneFontFromJson(body, "ATASCII_ORIGINAL_9X9", 9, 9); // legacy V61-V63
  uint8_t n12 = parseOneFontFromJson(body, "ATASCII_MEJORADA_12X16", 12, 16); // legacy, no usado por renderer V64
  g_fontLoadStatus = "8x8=" + String((int)n9) + " legacy12=" + String((int)n12);
  return (n9 > 0 || n12 > 0);
}


static uint8_t countCustomFont8Valid() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 128; i++) if (g_customFont8Valid[i]) n++;
  return n;
}

static uint8_t countCustomFont12Valid() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 128; i++) if (g_customFont12Valid[i]) n++;
  return n;
}

static void refreshGlyphLoadStatusFromRam() {
  g_fontLoadStatus = "8x8=" + String((int)countCustomFont8Valid()) + " legacy12=" + String((int)countCustomFont12Valid());
}

static String hexByte2(uint8_t v) {
  char b[3];
  snprintf(b, sizeof(b), "%02X", (unsigned int)v);
  return String(b);
}


// [F24] Eliminado helper no usado: glyphRowsAllZeroF49Z62
static uint8_t parseGlyphCodeArgF49Z62(const String& raw) {
  // F49Z79: en Glifos el code visible es SIEMPRE hexadecimal.
  // Ejemplos requeridos: "37" => 0x37 => carácter '7', "3D" => '='.
  // Si alguna herramienta antigua necesita decimal, puede usar "dec:55".
  String s = raw;
  s.trim();
  if (s.length() == 0) return 0;
  if (s.startsWith("dec:") || s.startsWith("DEC:")) {
    long n = s.substring(4).toInt();
    if (n < 0) n = 0;
    return (uint8_t)(n & 0x7F);
  }
  if (s.startsWith("0x") || s.startsWith("0X")) return (uint8_t)(parseHex16Str(s.substring(2)) & 0x7F);
  bool allHex = true;
  for (uint16_t i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) {
      allHex = false;
      break;
    }
  }
  if (allHex) return (uint8_t)(parseHex16Str(s) & 0x7F);
  long n = s.toInt();
  if (n < 0) n = 0;
  return (uint8_t)(n & 0x7F);
}


static String hexWord3(uint16_t v) {
  char b[4];
  snprintf(b, sizeof(b), "%03X", (unsigned int)(v & 0x0FFF));
  return String(b);
}

String jsonEscape(const String& in);

static String glyphDisplayName(uint8_t code) {
  if (code >= 0x20 && code <= 0x7E) {
    String out = "";
    out += (char)code;
    return out;
  }
  return "ATASCII_0x" + hexByte2(code);
}

static String hexGlyph8(uint16_t v) {
  char b[3];
  snprintf(b, sizeof(b), "%02X", (unsigned int)(v & 0x00FF));
  return String(b);
}

static void appendGlyphRowsJson8(String& out, uint8_t code, bool& first) {
  if (!g_customFont8Valid[code]) return;
  if (!first) out += ",\n";
  first = false;
  out += "        { \"code\": \"" + hexByte2(code) + "\", \"name\": \"" + jsonEscape(glyphDisplayName(code)) + "\", \"rowsHex\": [";
  for (uint8_t y = 0; y < 8; y++) {
    if (y) out += ",";
    out += "\"" + hexGlyph8(g_customFont8[code][y]) + "\"";
  }
  out += "] }";
}

// [F24] Eliminado helper no usado: appendGlyphRowsJson12
static String buildCurrentGlyphJson() {
  refreshGlyphLoadStatusFromRam();
  String out;
  out.reserve(42000);
  out += "{\n";
  out += "  \"version\": 3,\n";
  out += "  \"defaultFont\": \"ATASCII_ORIGINAL_8X8\",\n";
  out += "  \"generatedBy\": \"V71_WEB_FS_QUEUED_PRINT_ATASCII_8X8\",\n";
  out += "  \"fonts\": [\n";
  out += "    {\n";
  out += "      \"id\": \"ATASCII_ORIGINAL_8X8\",\n";
  out += "      \"name\": \"ATASCII matriz 8x8\",\n";
  out += "      \"width\": 8,\n";
  out += "      \"height\": 8,\n";
  out += "      \"cellWidth\": 8,\n";
  out += "      \"cellHeight\": 8,\n";
  out += "      \"glyphs\": [\n";
  bool first = true;
  for (uint8_t code = 0; code < 128; code++) appendGlyphRowsJson8(out, code, first);
  out += "\n      ]\n";
  out += "    }\n";
  out += "  ]\n";
  out += "}\n";
  return out;
}


static String buildAllGlyphJsonF49Z60() {
  // F49Z79: exportar la RAM cargada desde SD como fuente principal.
  // Si un código existe en g_customFont8Valid[], sus rowsHex se envían crudos,
  // incluso cuando las 8 filas son "00". Solo si falta el glifo se usa fallback.
  String out;
  out.reserve(46000);
  out += "{\n";
  out += "  \"version\": 82,\n";
  out += "  \"defaultFont\": \"ATASCII_ORIGINAL_8X8\",\n";
  out += "  \"generatedBy\": \"F49Z79_GLYPH_SD_SINGLE_SOURCE\",\n";
  out += "  \"storagePath\": \"" + jsonEscape(String(ATASCII_GLYPH_JSON_PATH)) + "\",\n";
  out += "  \"loadedFromStorage\": " + String(g_fontFilePresent ? "true" : "false") + ",\n";
  out += "  \"valid8x8\": " + String((int)countCustomFont8Valid()) + ",\n";
  out += "  \"fonts\": [\n";
  out += "    {\n";
  out += "      \"id\": \"ATASCII_ORIGINAL_8X8\",\n";
  out += "      \"name\": \"ATASCII matriz 8x8 por code desde SD/RAM\",\n";
  out += "      \"width\": 8,\n";
  out += "      \"height\": 8,\n";
  out += "      \"cellWidth\": 8,\n";
  out += "      \"cellHeight\": 8,\n";
  out += "      \"glyphs\": [\n";
  for (uint8_t code = 0; code < 128; code++) {
    if (code) out += ",\n";
    out += "        { \"code\": \"" + hexByte2(code) + "\", \"id\": \"" + hexByte2(code) + "\", \"name\": \"" + jsonEscape(glyphDisplayName(code)) + "\", \"source\": \"";
    out += g_customFont8Valid[code] ? "SD/RAM" : "fallback/base";
    out += "\", \"rowsHex\": ";
    if (g_customFont8Valid[code]) {
      out += "[";
      for (uint8_t y = 0; y < 8; y++) {
        if (y) out += ",";
        out += "\"" + hexGlyph8(g_customFont8[code][y]) + "\"";
      }
      out += "]";
    } else {
      out += rowsHexForGlyphJson(VF_ATASCII_ORIGINAL_8X8, code);
    }
    out += " }";
  }
  out += "\n      ]\n";
  out += "    }\n";
  out += "  ]\n";
  out += "}\n";
  return out;
}


static bool glyphEnsureDirForPath(const String& path) {
  int slash = path.lastIndexOf('/');
  if (slash <= 0) return true;
  String dir = path.substring(0, slash);
  if (dir.length() == 0 || dir == "/") return true;
  if (SPIFFS.exists(dir)) return true;
  return SPIFFS.mkdir(dir);
}

static uint32_t fnv1aUpdate(uint32_t h, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    h ^= data[i];
    h *= 16777619UL;
  }
  return h;
}

static bool writeGlyphTextAtomicVerified(const String& path, const String& body, String& err) {
  if (!g_fontFsReady) { err = "almacenamiento no listo"; return false; }
  if (!glyphEnsureDirForPath(path)) { err = "no crea carpeta destino"; return false; }
  String tmp = path + ".tmp";
  if (SPIFFS.exists(tmp)) SPIFFS.remove(tmp);

  uint32_t expectedHash = fnv1aUpdate(2166136261UL, (const uint8_t*)body.c_str(), body.length());
  File f = SPIFFS.open(tmp, "w");
  if (!f) { err = "no abre temporal"; return false; }
  size_t written = f.print(body);
  f.flush();
  f.close();
  if (written != body.length()) {
    SPIFFS.remove(tmp);
    err = "write incompleto " + String((unsigned long)written) + "/" + String((unsigned long)body.length());
    return false;
  }

  File r = SPIFFS.open(tmp, "r");
  if (!r) { SPIFFS.remove(tmp); err = "no reabre temporal"; return false; }
  if ((uint32_t)r.size() != (uint32_t)body.length()) {
    uint32_t got = (uint32_t)r.size();
    r.close();
    SPIFFS.remove(tmp);
    err = "size temporal " + String((unsigned long)got) + " != " + String((unsigned long)body.length());
    return false;
  }
  uint8_t buf[256];
  uint32_t gotHash = 2166136261UL;
  while (r.available()) {
    int n = r.read(buf, sizeof(buf));
    if (n <= 0) break;
    gotHash = fnv1aUpdate(gotHash, buf, (size_t)n);
  }
  r.close();
  if (gotHash != expectedHash) {
    SPIFFS.remove(tmp);
    err = "hash temporal no coincide";
    return false;
  }

  if (SPIFFS.exists(path)) SPIFFS.remove(path);
  if (!SPIFFS.rename(tmp, path)) {
    // Algunos FS/SD fallan al renombrar; copia manual como respaldo.
    File src = SPIFFS.open(tmp, "r");
    File dst = SPIFFS.open(path, "w");
    if (!src || !dst) {
      if (src) src.close();
      if (dst) dst.close();
      SPIFFS.remove(tmp);
      err = "rename/copia fallo";
      return false;
    }
    while (src.available()) {
      int n = src.read(buf, sizeof(buf));
      if (n > 0) dst.write(buf, (size_t)n);
    }
    src.close();
    dst.flush();
    dst.close();
    SPIFFS.remove(tmp);
  }

  File finalFile = SPIFFS.open(path, "r");
  if (!finalFile) { err = "no reabre final"; return false; }
  bool ok = ((uint32_t)finalFile.size() == (uint32_t)body.length());
  finalFile.close();
  if (!ok) { err = "size final invalido"; return false; }
  return true;
}

static bool persistCurrentGlyphJson() {
  if (!g_fontFsReady) return false;
  String body = buildCurrentGlyphJson();
  if (body.length() > 70000) {
    g_fontLoadStatus = "JSON generado demasiado grande";
    return false;
  }
  String err;
  if (!writeGlyphTextAtomicVerified(String(ATASCII_GLYPH_JSON_PATH), body, err)) {
    g_fontLoadStatus = "no persiste JSON: " + err;
    return false;
  }
#if WEB_STORAGE_USE_SD
  // Espejo de compatibilidad por si quedó un firmware anterior leyendo raíz.
  String mirrorErr;
  writeGlyphTextAtomicVerified(String("/atascii_glyphs.json"), body, mirrorErr);
#endif
  g_fontFilePresent = (countCustomFont8Valid() > 0 || countCustomFont12Valid() > 0);
  refreshGlyphLoadStatusFromRam();
  g_fontLoadStatus += " default=" + String(ATASCII_GLYPH_JSON_PATH) + " bytes=" + String((unsigned long)body.length());
  g_fontLastPersistMs = millis();
  logf("[ATASCII-FONT] DEFAULT persist OK path=%s bytes=%lu", ATASCII_GLYPH_JSON_PATH, (unsigned long)body.length());
  return true;
}

// Forward declarations used by the web glyph editor helpers.
static uint8_t virtualGlyphW(uint8_t fontId);
static uint8_t virtualGlyphH(uint8_t fontId);
static bool atasciiGlyphPixel(uint8_t c, uint8_t fontId, uint8_t gx, uint8_t gy);

static String rowsHexForGlyphJson(uint8_t fontId, uint8_t code) {
  uint8_t w = virtualGlyphW(fontId);
  uint8_t h = virtualGlyphH(fontId);
  uint8_t hexLen = (uint8_t)((w + 3) / 4);
  String out = "[";
  for (uint8_t y = 0; y < h; y++) {
    uint16_t row = 0;
    for (uint8_t x = 0; x < w; x++) {
      if (atasciiGlyphPixel(code & 0x7F, fontId, x, y)) row |= (uint16_t)(1U << (w - 1 - x));
    }
    if (y) out += ",";
    char b[5];
    if (hexLen <= 2) snprintf(b, sizeof(b), "%02X", (unsigned int)(row & 0xFF));
    else snprintf(b, sizeof(b), "%03X", (unsigned int)(row & 0x0FFF));
    out += "\"";
    out += b;
    out += "\"";
  }
  out += "]";
  return out;
}


static bool loadAtasciiGlyphJsonFromPath(const char* path) {
  if (!path || !SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) { g_fontLoadStatus = String("no abre JSON ") + path; return false; }
  if (f.size() > 70000) { f.close(); g_fontLoadStatus = String("JSON demasiado grande ") + path; return false; }
  String body = f.readString();
  f.close();
  bool ok = parseAtasciiGlyphJson(body);
  if (ok) g_fontLoadStatus += String(" desde ") + path + " bytes=" + String((unsigned long)body.length());
  return ok;
}

static void loadAtasciiGlyphJsonFromFs() {
  g_fontFilePresent = false;
  g_fontLoadStatus = "sin JSON";
  clearCustomFonts();
  if (!g_fontFsReady) return;
  g_fontFilePresent = loadAtasciiGlyphJsonFromPath(ATASCII_GLYPH_JSON_PATH);
#if WEB_STORAGE_USE_SD
  if (!g_fontFilePresent) g_fontFilePresent = loadAtasciiGlyphJsonFromPath("/atascii_glyphs.json");
#endif
  g_fontLastLoadMs = millis();
  logf("[ATASCII-FONT] JSON load present=%u %s", g_fontFilePresent ? 1 : 0, g_fontLoadStatus.c_str());
}

// F49Z74: se corrige carrera de Glifos; el default no se recarga mientras hay import/guardar en curso.
// F49Z73: se eliminan bloques de encabezado/descripción solicitados sin cambiar template.
// Si la SD no estuvo lista durante el boot inicial, los endpoints de Glifos
// reintentan abrir el almacenamiento y recargan /GLYPHS/atascii_glyphs.json.
static bool ensureAtasciiGlyphDefaultLoaded(bool forceReload) {
  if (!g_fontFsReady) {
    g_fontFsReady = SPIFFS.begin(true);
  }
  if (!g_fontFsReady) {
    g_fontLoadStatus = "STORAGE ERROR";
    return false;
  }
  if (!SPIFFS.exists(ATASCII_GLYPH_DEFAULT_MARKER_PATH) || !SPIFFS.exists(ATASCII_GLYPH_JSON_PATH)) {
    loadBuiltInDefaultGlyphsToRam("lazy-install-default");
    if (persistCurrentGlyphJson()) writeBuiltInDefaultMarker();
    return true;
  }

  const uint32_t nowMs = millis();
  const bool recentlyPersisted = (g_fontLastPersistMs != 0 && (uint32_t)(nowMs - g_fontLastPersistMs) < GLYPH_RELOAD_GUARD_MS);
  const bool ramHasGlyphs = (g_fontFilePresent && countCustomFont8Valid() > 0);
  if (forceReload && recentlyPersisted && ramHasGlyphs) {
    // F49Z76: durante unos segundos tras importar/guardar, la fuente de verdad es RAM.
    // Esto evita que callbacks antiguos de la página pidan reload y repinten con un JSON viejo.
    refreshGlyphLoadStatusFromRam();
    g_fontLoadStatus += " RAM_GUARD post-import bytes=mem";
    return true;
  }
  if (forceReload || !g_fontFilePresent || countCustomFont8Valid() == 0) {
    g_fontReloadRequests++;
    loadAtasciiGlyphJsonFromFs();
  }
  return g_fontFilePresent;
}

static void initAtasciiGlyphFs() {
  g_fontFsReady = SPIFFS.begin(true);
  if (!g_fontFsReady) {
    g_fontLoadStatus = "STORAGE ERROR inicial; se reintentara al abrir Glifos";
    logf("[ATASCII-FONT] STORAGE begin ERROR inicial; lazy reload activo");
    return;
  }

  // F49Z82: estos glifos quedan como default del app.
  // En el primer arranque de esta versión, si no existe el marcador, se inicializa
  // /GLYPHS/atascii_glyphs.json con el ATASCII_ORIGINAL_8X8 embebido.
  if (!SPIFFS.exists(ATASCII_GLYPH_DEFAULT_MARKER_PATH) || !SPIFFS.exists(ATASCII_GLYPH_JSON_PATH)) {
    loadBuiltInDefaultGlyphsToRam("install-default-on-boot");
    if (persistCurrentGlyphJson()) {
      writeBuiltInDefaultMarker();
      logf("[ATASCII-FONT] APP DEFAULT instalado en SD path=%s", ATASCII_GLYPH_JSON_PATH);
      return;
    }
  }

  loadAtasciiGlyphJsonFromFs();
  if (!g_fontFilePresent || countCustomFont8Valid() == 0) {
    loadBuiltInDefaultGlyphsToRam("fallback-after-load-fail");
    if (persistCurrentGlyphJson()) writeBuiltInDefaultMarker();
  }
}

static uint8_t effectiveFontForProfile(uint8_t profileId) {
  (void)profileId;
  // V64: trabajamos solo con matriz ATASCII 8x8.
  return VF_ATASCII_ORIGINAL_8X8;
}

static uint8_t virtualGlyphW(uint8_t fontId) { (void)fontId; return 8; }
static uint8_t virtualGlyphH(uint8_t fontId) { (void)fontId; return 8; }
static uint8_t printerFontPointCell() {
  switch (PRN_CFG.fontScale) {
    case 8: return 8;
    case 10: return 10;
    case 12: return 12;
    case 14: return 14;
    case 16: return 16;
    default: return 8;
  }
}

static uint8_t virtualCellW(uint8_t profileId, uint8_t fontId) {
  (void)profileId; (void)fontId;
  // V76: glifo base 8x8 y celda configurable aproximada en puntos: 8/10/12/14/16.
  return printerFontPointCell();
}
static uint8_t virtualCellH(uint8_t profileId, uint8_t fontId) {
  (void)profileId; (void)fontId;
  return printerFontPointCell();
}
static uint8_t virtualRenderScaleX(uint8_t profileId, uint8_t fontId) {
  if (fontId == VF_ATASCII_MEJORADA_12X16) return 1;
  return VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(profileId)].scaleX;
}
static uint8_t virtualRenderScaleY(uint8_t profileId, uint8_t fontId) {
  if (fontId == VF_ATASCII_MEJORADA_12X16) return 1;
  return VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(profileId)].scaleY;
}

static uint8_t getVirtualPrinterEffectiveScaleX(uint8_t profileId, uint8_t fontId);
static uint8_t getVirtualPrinterEffectiveScaleY(uint8_t profileId, uint8_t fontId);
static void getVirtualPrinterPageSize(uint8_t profileId, uint16_t* pageW, uint16_t* pageH);

static const char* virtualOrientationName(uint8_t orientation) {
  switch (orientation) {
    case VP_ORIENT_PORTRAIT: return "Vertical";
    case VP_ORIENT_LANDSCAPE: return "Horizontal";
    case VP_ORIENT_AUTO:
    default: return "Auto";
  }
}

static const char* virtualPaperName(uint8_t paper) {
  switch (paper) {
    case VP_PAPER_LETTER: return "Carta / Letter";
    case VP_PAPER_A4:
    default: return "A4";
  }
}

static const char* virtualPaperMediaKeyword(uint8_t paper) {
  switch (paper) {
    case VP_PAPER_LETTER: return "na_letter_8.5x11in";
    case VP_PAPER_A4:
    default: return "iso_a4_210x297mm";
  }
}

static uint16_t getVirtualPrinterUsableWidth(uint8_t profileId) {
  uint16_t pageW = 744, pageH = 1056;
  getVirtualPrinterPageSize(profileId, &pageW, &pageH);
  (void)pageH;
  return (pageW > 48) ? (uint16_t)(pageW - 48) : pageW;
}

static uint16_t getVirtualPrinterUsableHeight(uint8_t profileId) {
  uint16_t pageW = 744, pageH = 1056;
  getVirtualPrinterPageSize(profileId, &pageW, &pageH);
  (void)pageW;
  return (pageH > 72) ? (uint16_t)(pageH - 72) : pageH;
}

static uint16_t getVirtualPrinterEffectiveColumns(uint8_t profileId) {
  const uint8_t fontId = effectiveFontForProfile(profileId);
  const uint8_t scaleX = getVirtualPrinterEffectiveScaleX(profileId, fontId);
  uint16_t cellPx = (uint16_t)virtualCellW(profileId, fontId) * (uint16_t)scaleX;
  if (cellPx == 0) cellPx = 1;
  uint16_t cols = getVirtualPrinterUsableWidth(profileId) / cellPx;
  if (cols < 20) cols = 20;
  if (cols > 220) cols = 220;
  return cols;
}

static uint16_t getVirtualPrinterEffectiveRows(uint8_t profileId) {
  const uint8_t fontId = effectiveFontForProfile(profileId);
  const uint8_t scaleY = getVirtualPrinterEffectiveScaleY(profileId, fontId);
  uint16_t cellPx = (uint16_t)virtualCellH(profileId, fontId) * (uint16_t)scaleY;
  if (cellPx == 0) cellPx = 1;
  uint16_t rows = getVirtualPrinterUsableHeight(profileId) / cellPx;
  if (rows < 10) rows = 10;
  if (rows > 240) rows = 240;
  if (rows > PRN_VIRTUAL_MAX_RENDER_ROWS) rows = PRN_VIRTUAL_MAX_RENDER_ROWS;
  return rows;
}

static uint16_t getVirtualPrinterEffectiveFlushRows(uint8_t profileId) {
  uint16_t rows = getVirtualPrinterEffectiveRows(profileId);
  if (rows > PRN_VIRTUAL_MAX_FLUSH_ROWS) rows = PRN_VIRTUAL_MAX_FLUSH_ROWS;
  return rows;
}

static uint8_t getVirtualPrinterEffectiveScaleX(uint8_t profileId, uint8_t fontId) {
  if (PRN_CFG.fontScale == 8 || PRN_CFG.fontScale == 10 || PRN_CFG.fontScale == 12 || PRN_CFG.fontScale == 14 || PRN_CFG.fontScale == 16) return 1;
  return virtualRenderScaleX(profileId, fontId);
}

static uint8_t getVirtualPrinterEffectiveScaleY(uint8_t profileId, uint8_t fontId) {
  if (PRN_CFG.fontScale == 8 || PRN_CFG.fontScale == 10 || PRN_CFG.fontScale == 12 || PRN_CFG.fontScale == 14 || PRN_CFG.fontScale == 16) return 1;
  return virtualRenderScaleY(profileId, fontId);
}

static uint8_t getVirtualPrinterEffectiveOrientation(uint8_t profileId) {
  if (PRN_CFG.pageOrientation == VP_ORIENT_PORTRAIT || PRN_CFG.pageOrientation == VP_ORIENT_LANDSCAPE) {
    return PRN_CFG.pageOrientation;
  }
  // V69: el perfil condensado abre por defecto en horizontal; los demás en vertical.
  return (profileId == VP_ATASCII_132) ? VP_ORIENT_LANDSCAPE : VP_ORIENT_PORTRAIT;
}

static void getVirtualPrinterPageSize(uint8_t profileId, uint16_t* pageW, uint16_t* pageH) {
  const uint8_t orient = getVirtualPrinterEffectiveOrientation(profileId);
  // V74: tamaño lógico del JPEG. IPP usa print-scaling=fit para ajustar a papel real.
  // Mantenemos resolución moderada para no agotar memoria en ESP32 sin PSRAM.
  uint16_t w = 744;
  uint16_t h = (PRN_CFG.paperSize == VP_PAPER_LETTER) ? 963 : 1056; // Carta/Letter vs A4
  if (orient == VP_ORIENT_LANDSCAPE) {
    if (pageW) *pageW = h;
    if (pageH) *pageH = w;
  } else {
    if (pageW) *pageW = w;
    if (pageH) *pageH = h;
  }
}

static bool glyph8PixelFallback(uint8_t base, uint8_t gx, uint8_t gy) {
  // Nombre legacy: en V64 devuelve pixeles de la matriz 8x8.
  uint16_t rowBits = 0;
  if (base < 128 && g_customFont8Valid[base]) {
    // F49Z79: si el JSON de SD define un glifo como 00, se respeta.
    // Antes los glifos custom todo-cero se reemplazaban por el fallback antiguo.
    rowBits = (uint16_t)(g_customFont8[base][gy % 8] & 0x00FF);
  } else {
    // F49Z82: el fallback/base del app ya no es aproximado 5x7;
    // usa el ATASCII original 8x8 embebido en PROGMEM.
    rowBits = (uint16_t)builtinDefaultGlyphRow8(base & 0x7F, gy % 8);
  }
  return (rowBits & (uint16_t)(0x0080 >> (gx % 8))) != 0;
}

static bool atasciiGlyphPixel(uint8_t c, uint8_t fontId, uint8_t gx, uint8_t gy) {
  bool inverse = (c & 0x80) != 0;
  uint8_t base = c & 0x7F;
  bool on = false;
  (void)fontId;
  on = glyph8PixelFallback(base, gx, gy);
  return inverse ? !on : on;
}

static uint16_t countAtasciiCellBlackPixelsEx(uint8_t c, uint8_t fontId, uint8_t scaleX, uint8_t scaleY) {
  uint16_t bitsOn = 0;
  uint8_t w = virtualGlyphW(fontId);
  uint8_t h = virtualGlyphH(fontId);
  for (uint8_t gy = 0; gy < h; gy++) {
    for (uint8_t gx = 0; gx < w; gx++) if (atasciiGlyphPixel(c, fontId, gx, gy)) bitsOn++;
  }
  return (uint16_t)(bitsOn * scaleX * scaleY);
}

static void rasterSetBlack(std::vector<uint8_t>& raster, uint16_t width, uint16_t height, uint16_t x, uint16_t y) {
  if (x >= width || y >= height) return;
  uint16_t bpl = (uint16_t)((width + 7) / 8);
  size_t idx = (size_t)y * bpl + (x >> 3);
  raster[idx] &= (uint8_t)~(0x80 >> (x & 7)); // CUPS_CSPACE_W 1bpp: 1=blanco, 0=negro
}

// [F24] Eliminado helper no usado: rasterDrawCellPixel
static void pwgSetU32(std::vector<uint8_t>& h, size_t off, uint32_t v) {
  h[off + 0] = (uint8_t)(v >> 24);
  h[off + 1] = (uint8_t)(v >> 16);
  h[off + 2] = (uint8_t)(v >> 8);
  h[off + 3] = (uint8_t)v;
}

static void pwgSetFloat(std::vector<uint8_t>& h, size_t off, float f) {
  union { float f; uint8_t b[4]; } u;
  u.f = f;
  h[off + 0] = u.b[3]; h[off + 1] = u.b[2]; h[off + 2] = u.b[1]; h[off + 3] = u.b[0];
}

static void pwgSetString(std::vector<uint8_t>& h, size_t off, size_t maxLen, const char* s) {
  size_t n = strlen(s);
  if (n >= maxLen) n = maxLen - 1;
  memcpy(&h[off], s, n);
}

static void pwgAppendLinePackBits(std::vector<uint8_t>& out, const uint8_t* line, uint16_t n) {
  // PWG/CUPS raster usa PackBits:
  //   0..127   => copiar N+1 bytes literales
  //   129..255 => repetir el siguiente byte 257-N veces
  //   128      => NOP
  // En V40 estaba invertido; por eso la Brother aceptaba el job pero no salía hoja.
  uint16_t pos = 0;
  while (pos < n) {
    uint16_t run = 1;
    while ((pos + run) < n && run < 128 && line[pos + run] == line[pos]) run++;

    if (run >= 3) {
      // Repetición de 3..128 bytes: control = 257 - run.
      out.push_back((uint8_t)(257 - run));
      out.push_back(line[pos]);
      pos += run;
    } else {
      uint16_t start = pos;
      pos++;
      while (pos < n && (pos - start) < 128) {
        uint16_t nextRun = 1;
        while ((pos + nextRun) < n && nextRun < 128 && line[pos + nextRun] == line[pos]) nextRun++;
        if (nextRun >= 3) break;
        pos++;
      }

      uint16_t count = pos - start;
      // Literal de 1..128 bytes: control = count - 1.
      out.push_back((uint8_t)(count - 1));
      out.insert(out.end(), line + start, line + start + count);
    }
  }
}

// [F24] Eliminado helper no usado: pwgAppendCompressedBitmap
static void pwgAppendHeader1bpp(std::vector<uint8_t>& out, uint16_t width, uint16_t height, uint16_t bytesPerLine) {
  // PWG Raster es CUPS Raster v2 big-endian: magic RaS2 + cups_page_header2_t (1796 bytes).
  // Usamos 1bpp blanco/negro para eliminar totalmente el halo gris del JPEG.
  out.push_back('R'); out.push_back('a'); out.push_back('S'); out.push_back('2');

  const size_t hdrStart = out.size();
  out.resize(out.size() + 1796, 0);
  std::vector<uint8_t> h(1796, 0);

  const bool landscape = (getVirtualPrinterEffectiveOrientation(PRN_CFG.virtualProfile) == VP_ORIENT_LANDSCAPE);
  const bool letter = (PRN_CFG.paperSize == VP_PAPER_LETTER);
  uint32_t pagePtW = letter ? 612 : 595;
  uint32_t pagePtH = letter ? 792 : 842;
  if (landscape) { uint32_t t = pagePtW; pagePtW = pagePtH; pagePtH = t; }

  // V85: algunas impresoras IPP rechazan el PWG si MediaClass no es exactamente PwgRaster.
  // Esto es parte del perfil image/pwg-raster y evita client-error-document-format-error 0x0411.
  pwgSetString(h, 0,   64, "PwgRaster");             // MediaClass obligatorio para PWG Raster
  pwgSetString(h, 64,  64, "white");                 // MediaColor
  pwgSetString(h, 128, 64, "stationery");            // MediaType
  pwgSetString(h, 192, 64, "Automatic");            // OutputType
  pwgSetU32(h, 276, 90);                             // HWResolution X
  pwgSetU32(h, 280, 90);                             // HWResolution Y
  pwgSetU32(h, 284, 0);                              // ImagingBoundingBox left
  pwgSetU32(h, 288, 0);                              // ImagingBoundingBox bottom
  pwgSetU32(h, 292, pagePtW);                        // ImagingBoundingBox right
  pwgSetU32(h, 296, pagePtH);                        // ImagingBoundingBox top
  pwgSetU32(h, 340, 1);                              // NumCopies
  pwgSetU32(h, 344, landscape ? 4 : 3);              // Orientation
  pwgSetU32(h, 352, pagePtW);                        // PageSize X in points
  pwgSetU32(h, 356, pagePtH);                        // PageSize Y in points
  pwgSetU32(h, 372, width);                          // cupsWidth
  pwgSetU32(h, 376, height);                         // cupsHeight
  pwgSetU32(h, 384, 1);                              // cupsBitsPerColor
  pwgSetU32(h, 388, 1);                              // cupsBitsPerPixel
  pwgSetU32(h, 392, bytesPerLine);                   // cupsBytesPerLine
  pwgSetU32(h, 396, 0);                              // cupsColorOrder = chunky
  pwgSetU32(h, 400, 3);                              // cupsColorSpace = W, 1=white 0=black
  pwgSetU32(h, 404, 1);                              // cupsCompression = PackBits/RLE
  pwgSetU32(h, 420, 1);                              // cupsNumColors
  pwgSetFloat(h, 424, 1.0f);                         // cupsBorderlessScalingFactor
  pwgSetString(h, 428, 64, virtualPaperMediaKeyword(PRN_CFG.paperSize));
  pwgSetString(h, 1660, 64, "black_1");              // cupsMarkerType
  pwgSetString(h, 1724, 64, "text");                 // cupsRenderingIntent
  pwgSetFloat(h, 1788, (float)pagePtW);
  pwgSetFloat(h, 1792, (float)pagePtH);

  memcpy(&out[hdrStart], h.data(), h.size());
}

static void pwgSetBlackOnLine(std::vector<uint8_t>& line, uint16_t x) {
  size_t idx = (size_t)(x >> 3);
  if (idx >= line.size()) return;
  line[idx] &= (uint8_t)~(0x80 >> (x & 7));
}

static bool buildAtasciiPwgRender(const uint8_t* data, size_t len, uint8_t profileId, std::vector<uint8_t>& out, uint16_t rowsOverride = 0) {
  const VirtualPrinterProfile& prof = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(profileId)];
  const uint8_t fontId = effectiveFontForProfile(profileId);
  const uint8_t glyphW = virtualGlyphW(fontId);
  const uint8_t glyphH = virtualGlyphH(fontId);
  const uint8_t scaleX = getVirtualPrinterEffectiveScaleX(profileId, fontId);
  const uint8_t scaleY = getVirtualPrinterEffectiveScaleY(profileId, fontId);
  uint16_t pageW = 744;
  uint16_t pageH = 1056;
  getVirtualPrinterPageSize(profileId, &pageW, &pageH);
  const uint8_t cellW = virtualCellW(profileId, fontId);
  const uint8_t cellH = virtualCellH(profileId, fontId);
  const uint16_t cols = getVirtualPrinterEffectiveColumns(profileId);
  const uint16_t fullRows = getVirtualPrinterEffectiveRows(profileId);
  uint16_t rows = fullRows;
  if (rowsOverride > 0 && rowsOverride < rows) rows = rowsOverride;
  if (rows < 10) rows = 10;

  const uint32_t textW = (uint32_t)cols * (uint32_t)cellW * (uint32_t)scaleX;
  const uint16_t marginX = ((uint32_t)pageW > textW) ? (uint16_t)(((uint32_t)pageW - textW) / 2U) : 12;
  const uint16_t marginY = 36;
  const size_t gridBytes = (size_t)cols * (size_t)rows;
  const uint16_t bytesPerLine = (uint16_t)((pageW + 7) / 8);
  uint32_t blackPixels = 0;

  logf("[ATASCII-PWG] build V120 S3_JPEG_ONLY_WIFI_FIX perfil=%s cols=%u rows=%u orient=%s glyph=%ux%u cell=%ux%u page=%ux%u grid=%lu bpl=%u freeHeap=%lu maxAlloc=%lu",
       prof.name, cols, rows, virtualOrientationName(getVirtualPrinterEffectiveOrientation(profileId)),
       glyphW, glyphH, cellW, cellH, pageW, pageH, (unsigned long)gridBytes,
       (unsigned int)bytesPerLine, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
  if (rowsOverride > 0 && rowsOverride < fullRows) {
    logf("[ATASCII-PWG] rowsOverride=%u fullRows=%u", (unsigned int)rows, (unsigned int)fullRows);
  }

  std::vector<uint8_t> cells;
  try {
    cells.assign(gridBytes, 0x20);
  } catch (...) {
    g_prnLastError = "Sin memoria creando grilla ATASCII PWG";
    return false;
  }

  uint16_t col = 0;
  uint16_t row = 0;
  bool justAutoWrapped = false;
  auto newLine = [&]() { col = 0; row++; };
  auto consumeEol = [&]() {
    if (justAutoWrapped && col == 0) { justAutoWrapped = false; return; }
    newLine(); justAutoWrapped = false;
  };
  auto putCell = [&](uint8_t c) {
    if (row >= rows) return;
    size_t idx = (size_t)row * cols + col;
    if (idx < cells.size()) cells[idx] = c;
    col++;
    if (col >= cols) { newLine(); justAutoWrapped = true; }
    else justAutoWrapped = false;
  };
  for (size_t i = 0; i < len && row < rows; i++) {
    uint8_t c = data[i];
    if (c == 0x9B || c == 0x0D || c == 0x0A) { consumeEol(); continue; }
    putCell(c);
  }

  const uint16_t charW = (uint16_t)(cellW * scaleX);
  const uint16_t charH = (uint16_t)(cellH * scaleY);

  try {
    out.clear();
    out.reserve(4 + 1796 + (size_t)pageH * 8);
    pwgAppendHeader1bpp(out, pageW, pageH, bytesPerLine);

    std::vector<uint8_t> line(bytesPerLine, 0xFF);
    for (uint16_t y = 0; y < pageH; y++) {
      std::fill(line.begin(), line.end(), 0xFF);

      if (y >= marginY) {
        uint16_t localY = (uint16_t)(y - marginY);
        uint16_t cellRow = (charH > 0) ? (localY / charH) : 0;
        if (cellRow < rows) {
          uint16_t inCellY = (charH > 0) ? (localY % charH) : 0;
          uint8_t gy = (scaleY > 0) ? (uint8_t)(inCellY / scaleY) : 0;
          if (gy < glyphH) {
            for (uint16_t cc = 0; cc < cols; cc++) {
              size_t idx = (size_t)cellRow * cols + cc;
              uint8_t ch = (idx < cells.size()) ? cells[idx] : 0x20;
              if (ch == 0x20) continue;
              uint16_t baseX = (uint16_t)(marginX + (uint32_t)cc * charW);
              for (uint8_t gx = 0; gx < glyphW; gx++) {
                if (!atasciiGlyphPixel(ch, fontId, gx, gy)) continue;
                for (uint8_t sx = 0; sx < scaleX; sx++) {
                  uint16_t px = (uint16_t)(baseX + (uint16_t)gx * scaleX + sx);
                  if (px < pageW) { pwgSetBlackOnLine(line, px); blackPixels++; }
                }
              }
            }
          }
        }
      }

      out.push_back(0); // repetir esta línea 1 vez
      pwgAppendLinePackBits(out, line.data(), bytesPerLine);
      delay(0);
    }
  } catch (...) {
    g_prnLastError = "Sin memoria generando PWG Raster";
    return false;
  }

  float coverage = ((float)blackPixels * 100.0f) / ((float)pageW * (float)pageH);
  logf("[ATASCII-PWG] OK pwg=%lu grid=%lu blackPixels=%lu coverage=%.2f%% freeHeap=%lu maxAlloc=%lu",
       (unsigned long)out.size(), (unsigned long)gridBytes, (unsigned long)blackPixels, coverage,
       (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
  return true;
}


// ================== ATASCII -> JPEG virtual printer ==================
// V56: evita el raster temporal grande; codifica el JPEG desde grilla ATASCII.
// Mantiene 64 KB para evitar fragmentación; página fija + split 32 líneas.
static const size_t ATASCII_JPEG_RESERVE_BYTES = 98304; // V90: base segura; la reserva real se elige adaptativamente según maxAlloc
static size_t g_prnActiveJpegReserveBytes = ATASCII_JPEG_RESERVE_BYTES;
static const uint8_t ATASCII_JPEG_QUALITY = 48; // F49Z75: calidad inicial mayor; JPEG se escribe a SD, no necesita vector gigante en heap.
static const uint8_t ATASCII_JPEG_QUALITY_FALLBACKS[] = { 48, 44, 40, 38, 32, 28, 24, 18, 12, 10, 8, 6 }; // F49Z75: baja gradual si la SD/codificador falla.
static const char* PRN_JPEG_SPOOL_DIR = "/PRINT";
static const char* PRN_JPEG_TMP_PATH  = "/PRINT/atari_print_tmp.jpg";
static const char* PRN_JPEG_JOB_PATH  = "/PRINT/atari_print_job.jpg";
static uint32_t g_prnSdSpoolJobs = 0;
static uint32_t g_prnSdSpoolOk = 0;
static uint32_t g_prnSdSpoolErr = 0;
static uint32_t g_prnSdSpoolLastBytes = 0;
static uint8_t  g_prnSdSpoolLastQuality = 0;
static String   g_prnSdSpoolLastPath = "";

static size_t printerChooseJpegReserveBytes(size_t preferred) {
  size_t freeHeap = ESP.getFreeHeap();
  size_t maxAlloc = ESP.getMaxAllocHeap();
  const size_t safety = 12288;
  size_t usableByFree = (freeHeap > safety) ? (freeHeap - safety) : 0;
  size_t usableByBlock = (maxAlloc > safety) ? (maxAlloc - safety) : 0;
  size_t usable = usableByFree < usableByBlock ? usableByFree : usableByBlock;
  if (usable == 0) return 0;
  if (preferred > usable) preferred = usable;

  // F49Z67: antes el mínimo era 49 KB. Si el heap contiguo real era menor,
  // fallaba antes de probar rowsOverride/calidad. Ahora la reserva puede ser
  // pequeña o incluso 0; el vector crecerá gradualmente durante la codificación.
  const size_t candidates[] = { 114688, 106496, 98304, 90112, 81920, 73728, 65536, 49152, 32768, 24576, 16384, 8192, 4096 };
  for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
    if (candidates[i] <= preferred && candidates[i] <= usable) return candidates[i];
  }
  return 0;
}

// JPEG grayscale. Se usa JPEG porque el botón "Probar IPP directo" ya fue
// validado físicamente en la Brother DCP-T720DW.
static const uint8_t JPEG_ZIGZAG[64] = {
  0, 1, 8,16, 9, 2, 3,10,
 17,24,32,25,18,11, 4, 5,
 12,19,26,33,40,48,41,34,
 27,20,13, 6, 7,14,21,28,
 35,42,49,56,57,50,43,36,
 29,22,15,23,30,37,44,51,
 58,59,52,45,38,31,39,46,
 53,60,61,54,47,55,62,63
};

static const uint8_t JPEG_STD_LUMA_QT[64] = {
  16,11,10,16,24,40,51,61,
  12,12,14,19,26,58,60,55,
  14,13,16,24,40,57,69,56,
  14,17,22,29,51,87,80,62,
  18,22,37,56,68,109,103,77,
  24,35,55,64,81,104,113,92,
  49,64,78,87,103,121,120,101,
  72,92,95,98,112,100,103,99
};

static const uint8_t JPEG_DC_BITS[17] = { 0,0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0 };
static const uint8_t JPEG_DC_VALS[12] = { 0,1,2,3,4,5,6,7,8,9,10,11 };
static const uint8_t JPEG_AC_BITS[17] = { 0,0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d };
static const uint8_t JPEG_AC_VALS[162] = {
  0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
  0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,
  0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,
  0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
  0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
  0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
  0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,
  0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,
  0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,
  0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
  0xF9,0xFA
};

static void jpegAppendU16(std::vector<uint8_t>& out, uint16_t v) {
  out.push_back((uint8_t)(v >> 8));
  out.push_back((uint8_t)(v & 0xFF));
}

static void jpegAppendMarker(std::vector<uint8_t>& out, uint8_t marker) {
  out.push_back(0xFF);
  out.push_back(marker);
}

static bool printerEnsurePrintSpoolDir() {
  if (!webAtrFsReady() && !SPIFFS.begin(true)) {
    g_prnLastError = "SD no disponible para spool de impresion";
    return false;
  }
  if (!SPIFFS.exists(PRN_JPEG_SPOOL_DIR)) {
    if (!SPIFFS.mkdir(PRN_JPEG_SPOOL_DIR)) {
      g_prnLastError = "No se pudo crear /PRINT en SD";
      return false;
    }
  }
  return true;
}

static bool jpegFileWriteByte(File& f, uint8_t b, size_t& totalBytes) {
  size_t w = f.write(&b, 1);
  if (w != 1) return false;
  totalBytes++;
  return true;
}

static bool jpegFileAppendU16(File& f, uint16_t v, size_t& totalBytes) {
  return jpegFileWriteByte(f, (uint8_t)(v >> 8), totalBytes) && jpegFileWriteByte(f, (uint8_t)(v & 0xFF), totalBytes);
}

static bool jpegFileAppendMarker(File& f, uint8_t marker, size_t& totalBytes) {
  return jpegFileWriteByte(f, 0xFF, totalBytes) && jpegFileWriteByte(f, marker, totalBytes);
}

static bool jpegFileAppendDqt(File& f, const uint8_t qt[64], size_t& totalBytes) {
  if (!jpegFileAppendMarker(f, 0xDB, totalBytes)) return false;
  if (!jpegFileAppendU16(f, 67, totalBytes)) return false;
  if (!jpegFileWriteByte(f, 0x00, totalBytes)) return false;
  for (uint8_t i = 0; i < 64; i++) {
    if (!jpegFileWriteByte(f, qt[JPEG_ZIGZAG[i]], totalBytes)) return false;
  }
  return true;
}

static bool jpegFileAppendDht(File& f, uint8_t tcTh, const uint8_t* bits, const uint8_t* vals, uint16_t valCount, size_t& totalBytes) {
  if (!jpegFileAppendMarker(f, 0xC4, totalBytes)) return false;
  if (!jpegFileAppendU16(f, (uint16_t)(19 + valCount), totalBytes)) return false;
  if (!jpegFileWriteByte(f, tcTh, totalBytes)) return false;
  for (uint8_t i = 1; i <= 16; i++) {
    if (!jpegFileWriteByte(f, bits[i], totalBytes)) return false;
  }
  for (uint16_t i = 0; i < valCount; i++) {
    if (!jpegFileWriteByte(f, vals[i], totalBytes)) return false;
  }
  return true;
}


static void jpegBuildHuffman(const uint8_t* bits, const uint8_t* vals, uint16_t valCount, JpegHuffSymbol table[256]) {
  for (int i = 0; i < 256; i++) { table[i].code = 0; table[i].size = 0; }
  uint16_t code = 0;
  uint16_t k = 0;
  for (uint8_t len = 1; len <= 16; len++) {
    for (uint8_t j = 0; j < bits[len] && k < valCount; j++) {
      table[vals[k]].code = code;
      table[vals[k]].size = len;
      code++;
      k++;
    }
    code <<= 1;
  }
}

static uint8_t jpegCategory(int v) {
  if (v == 0) return 0;
  int a = v < 0 ? -v : v;
  uint8_t n = 0;
  while (a) { n++; a >>= 1; }
  return n;
}

static uint16_t jpegValueBits(int v, uint8_t cat) {
  if (cat == 0) return 0;
  if (v >= 0) return (uint16_t)v;
  return (uint16_t)(((1 << cat) - 1) + v);
}

static void jpegMakeQualityTable(uint8_t qt[64], uint8_t quality) {
  if (quality < 1) quality = 1;
  if (quality > 100) quality = 100;
  int scale = (quality < 50) ? (5000 / quality) : (200 - quality * 2);
  for (int i = 0; i < 64; i++) {
    int q = (JPEG_STD_LUMA_QT[i] * scale + 50) / 100;
    if (q < 1) q = 1;
    if (q > 255) q = 255;
    qt[i] = (uint8_t)q;
  }
}

static void jpegAppendDqt(std::vector<uint8_t>& out, const uint8_t qt[64]) {
  jpegAppendMarker(out, 0xDB);
  jpegAppendU16(out, 67);
  out.push_back(0x00); // Pq=0, Tq=0
  for (uint8_t i = 0; i < 64; i++) out.push_back(qt[JPEG_ZIGZAG[i]]);
}

static void jpegAppendDht(std::vector<uint8_t>& out, uint8_t tcTh, const uint8_t* bits, const uint8_t* vals, uint16_t valCount) {
  jpegAppendMarker(out, 0xC4);
  jpegAppendU16(out, (uint16_t)(19 + valCount));
  out.push_back(tcTh);
  for (uint8_t i = 1; i <= 16; i++) out.push_back(bits[i]);
  for (uint16_t i = 0; i < valCount; i++) out.push_back(vals[i]);
}

static bool jpegSampleMono(const std::vector<uint8_t>& raster, uint16_t bpl, uint16_t width, uint16_t height, uint16_t x, uint16_t y) {
  if (x >= width || y >= height) return true; // fuera de imagen = blanco
  size_t idx = (size_t)y * bpl + (x >> 3);
  if (idx >= raster.size()) return true;
  return (raster[idx] & (uint8_t)(0x80 >> (x & 7))) != 0; // true=blanco
}

static void jpegEncodeOneBlock(JpegBitWriter& bw, const std::vector<uint8_t>& raster,
                               uint16_t bpl, uint16_t width, uint16_t height,
                               uint16_t bx, uint16_t by, const uint8_t qt[64],
                               JpegHuffSymbol dcTable[256], JpegHuffSymbol acTable[256],
                               int& prevDC) {
  static float cosT[8][8];
  static bool cosReady = false;
  if (!cosReady) {
    const float PI_F = 3.14159265358979323846f;
    for (uint8_t u = 0; u < 8; u++) {
      for (uint8_t x = 0; x < 8; x++) {
        cosT[u][x] = cosf(((2.0f * x + 1.0f) * u * PI_F) / 16.0f);
      }
    }
    cosReady = true;
  }

  float block[64];
  bool uniform = true;
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 8; x++) {
      bool white = jpegSampleMono(raster, bpl, width, height, bx + x, by + y);
      float v = (white ? 255.0f : 0.0f) - 128.0f;
      block[y * 8 + x] = v;
      if (y != 0 || x != 0) {
        if (v != block[0]) uniform = false;
      }
    }
  }

  int coeff[64];
  int zz[64];
  for (uint8_t i = 0; i < 64; i++) coeff[i] = 0;

  if (uniform) {
    // Para la mayoría de la página, el bloque es blanco completo.
    // El DCT de un bloque uniforme solo tiene componente DC.
    float dc = 8.0f * block[0];
    float qf = dc / (float)qt[0];
    coeff[0] = (qf >= 0.0f) ? (int)(qf + 0.5f) : (int)(qf - 0.5f);
  } else {
    for (uint8_t v = 0; v < 8; v++) {
      for (uint8_t u = 0; u < 8; u++) {
        float sum = 0.0f;
        for (uint8_t y = 0; y < 8; y++) {
          for (uint8_t x = 0; x < 8; x++) {
            sum += block[y * 8 + x] * cosT[u][x] * cosT[v][y];
          }
        }
        float cu = (u == 0) ? 0.70710678118f : 1.0f;
        float cv = (v == 0) ? 0.70710678118f : 1.0f;
        float dct = 0.25f * cu * cv * sum;
        uint8_t idx = v * 8 + u;
        float qf = dct / (float)qt[idx];
        coeff[idx] = (qf >= 0.0f) ? (int)(qf + 0.5f) : (int)(qf - 0.5f);
      }
    }
  }
  for (uint8_t i = 0; i < 64; i++) zz[i] = coeff[JPEG_ZIGZAG[i]];

  int diff = zz[0] - prevDC;
  prevDC = zz[0];
  uint8_t cat = jpegCategory(diff);
  bw.putBits(dcTable[cat].code, dcTable[cat].size);
  bw.putBits(jpegValueBits(diff, cat), cat);

  uint8_t zeroRun = 0;
  for (uint8_t i = 1; i < 64; i++) {
    int ac = zz[i];
    if (ac == 0) {
      zeroRun++;
      continue;
    }
    while (zeroRun >= 16) {
      bw.putBits(acTable[0xF0].code, acTable[0xF0].size);
      zeroRun -= 16;
    }
    cat = jpegCategory(ac);
    uint8_t sym = (uint8_t)((zeroRun << 4) | cat);
    bw.putBits(acTable[sym].code, acTable[sym].size);
    bw.putBits(jpegValueBits(ac, cat), cat);
    zeroRun = 0;
  }
  if (zeroRun > 0) bw.putBits(acTable[0x00].code, acTable[0x00].size); // EOB
}


// [F24] Eliminado helper no usado: countBits8
// [F24] Eliminado helper no usado: countAtasciiCellBlackPixels
static bool jpegSampleAtasciiGrid(const std::vector<uint8_t>& cells,
                                  uint16_t cols, uint16_t rows,
                                  uint8_t profileId, uint8_t fontId, uint8_t scaleX, uint8_t scaleY,
                                  uint16_t pageW, uint16_t pageH,
                                  uint16_t marginX, uint16_t marginY,
                                  uint16_t x, uint16_t y) {
  if (x >= pageW || y >= pageH) return true;
  if (x < marginX || y < marginY) return true;

  const uint16_t charW = (uint16_t)(virtualCellW(profileId, fontId) * scaleX);
  const uint16_t charH = (uint16_t)(virtualCellH(profileId, fontId) * scaleY);
  uint16_t localX = (uint16_t)(x - marginX);
  uint16_t localY = (uint16_t)(y - marginY);

  uint16_t col = localX / charW;
  uint16_t row = localY / charH;
  if (col >= cols || row >= rows) return true;

  size_t idx = (size_t)row * cols + col;
  if (idx >= cells.size()) return true;

  uint8_t c = cells[idx];
  uint8_t gx = (uint8_t)((localX % charW) / scaleX);
  uint8_t gy = (uint8_t)((localY % charH) / scaleY);
  // Si la celda tiene espacio adicional, queda blanco para separar caracteres.
  if (gx >= virtualGlyphW(fontId) || gy >= virtualGlyphH(fontId)) return true;
  bool on = atasciiGlyphPixel(c, fontId, gx, gy);

  return !on; // true=blanco, false=negro
}

static void jpegEncodeOneBlockAtasciiGrid(JpegBitWriter& bw,
                               const std::vector<uint8_t>& cells,
                               uint16_t cols, uint16_t rows,
                               uint8_t profileId, uint8_t fontId, uint8_t scaleX, uint8_t scaleY,
                               uint16_t pageW, uint16_t pageH,
                               uint16_t marginX, uint16_t marginY,
                               uint16_t bx, uint16_t by, const uint8_t qt[64],
                               JpegHuffSymbol dcTable[256], JpegHuffSymbol acTable[256],
                               int& prevDC) {
  static float cosT[8][8];
  static bool cosReady = false;
  if (!cosReady) {
    const float PI_F = 3.14159265358979323846f;
    for (uint8_t u = 0; u < 8; u++) {
      for (uint8_t x = 0; x < 8; x++) {
        cosT[u][x] = cosf(((2.0f * x + 1.0f) * u * PI_F) / 16.0f);
      }
    }
    cosReady = true;
  }

  float block[64];
  bool uniform = true;
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 8; x++) {
      bool white = jpegSampleAtasciiGrid(cells, cols, rows, profileId, fontId, scaleX, scaleY, pageW, pageH, marginX, marginY, bx + x, by + y);
      float v = (white ? 255.0f : 0.0f) - 128.0f;
      block[y * 8 + x] = v;
      if (y != 0 || x != 0) {
        if (v != block[0]) uniform = false;
      }
    }
  }

  int coeff[64];
  int zz[64];
  for (uint8_t i = 0; i < 64; i++) coeff[i] = 0;

  if (uniform) {
    float dc = 8.0f * block[0];
    float qf = dc / (float)qt[0];
    coeff[0] = (qf >= 0.0f) ? (int)(qf + 0.5f) : (int)(qf - 0.5f);
  } else {
    for (uint8_t v = 0; v < 8; v++) {
      for (uint8_t u = 0; u < 8; u++) {
        float sum = 0.0f;
        for (uint8_t y = 0; y < 8; y++) {
          for (uint8_t x = 0; x < 8; x++) {
            sum += block[y * 8 + x] * cosT[u][x] * cosT[v][y];
          }
        }
        float cu = (u == 0) ? 0.70710678118f : 1.0f;
        float cv = (v == 0) ? 0.70710678118f : 1.0f;
        float dct = 0.25f * cu * cv * sum;
        uint8_t idx = v * 8 + u;
        float qf = dct / (float)qt[idx];
        coeff[idx] = (qf >= 0.0f) ? (int)(qf + 0.5f) : (int)(qf - 0.5f);
      }
    }
  }
  for (uint8_t i = 0; i < 64; i++) zz[i] = coeff[JPEG_ZIGZAG[i]];

  int diff = zz[0] - prevDC;
  prevDC = zz[0];
  uint8_t cat = jpegCategory(diff);
  bw.putBits(dcTable[cat].code, dcTable[cat].size);
  bw.putBits(jpegValueBits(diff, cat), cat);

  uint8_t zeroRun = 0;
  for (uint8_t i = 1; i < 64; i++) {
    int ac = zz[i];
    if (ac == 0) {
      zeroRun++;
      continue;
    }
    while (zeroRun >= 16) {
      bw.putBits(acTable[0xF0].code, acTable[0xF0].size);
      zeroRun -= 16;
    }
    cat = jpegCategory(ac);
    uint8_t sym = (uint8_t)((zeroRun << 4) | cat);
    bw.putBits(acTable[sym].code, acTable[sym].size);
    bw.putBits(jpegValueBits(ac, cat), cat);
    zeroRun = 0;
  }
  if (zeroRun > 0) bw.putBits(acTable[0x00].code, acTable[0x00].size);
}

static void jpegEncodeOneBlockAtasciiGridFile(JpegFileBitWriter& bw,
                               const std::vector<uint8_t>& cells,
                               uint16_t cols, uint16_t rows,
                               uint8_t profileId, uint8_t fontId, uint8_t scaleX, uint8_t scaleY,
                               uint16_t pageW, uint16_t pageH,
                               uint16_t marginX, uint16_t marginY,
                               uint16_t bx, uint16_t by, const uint8_t qt[64],
                               JpegHuffSymbol dcTable[256], JpegHuffSymbol acTable[256],
                               int& prevDC) {
  static float cosT[8][8];
  static bool cosReady = false;
  if (!cosReady) {
    const float PI_F = 3.14159265358979323846f;
    for (uint8_t u = 0; u < 8; u++) {
      for (uint8_t x = 0; x < 8; x++) {
        cosT[u][x] = cosf(((2.0f * x + 1.0f) * u * PI_F) / 16.0f);
      }
    }
    cosReady = true;
  }

  float block[64];
  bool uniform = true;
  for (uint8_t y = 0; y < 8; y++) {
    for (uint8_t x = 0; x < 8; x++) {
      bool white = jpegSampleAtasciiGrid(cells, cols, rows, profileId, fontId, scaleX, scaleY, pageW, pageH, marginX, marginY, bx + x, by + y);
      float v = (white ? 255.0f : 0.0f) - 128.0f;
      block[y * 8 + x] = v;
      if (y != 0 || x != 0) {
        if (v != block[0]) uniform = false;
      }
    }
  }

  int coeff[64];
  int zz[64];
  for (uint8_t i = 0; i < 64; i++) coeff[i] = 0;

  if (uniform) {
    float dc = 8.0f * block[0];
    float qf = dc / (float)qt[0];
    coeff[0] = (qf >= 0.0f) ? (int)(qf + 0.5f) : (int)(qf - 0.5f);
  } else {
    for (uint8_t v = 0; v < 8; v++) {
      for (uint8_t u = 0; u < 8; u++) {
        float sum = 0.0f;
        for (uint8_t y = 0; y < 8; y++) {
          for (uint8_t x = 0; x < 8; x++) {
            sum += block[y * 8 + x] * cosT[u][x] * cosT[v][y];
          }
        }
        float cu = (u == 0) ? 0.70710678118f : 1.0f;
        float cv = (v == 0) ? 0.70710678118f : 1.0f;
        float dct = 0.25f * cu * cv * sum;
        uint8_t idx = v * 8 + u;
        float qf = dct / (float)qt[idx];
        coeff[idx] = (qf >= 0.0f) ? (int)(qf + 0.5f) : (int)(qf - 0.5f);
      }
    }
  }
  for (uint8_t i = 0; i < 64; i++) zz[i] = coeff[JPEG_ZIGZAG[i]];

  int diff = zz[0] - prevDC;
  prevDC = zz[0];
  uint8_t cat = jpegCategory(diff);
  bw.putBits(dcTable[cat].code, dcTable[cat].size);
  bw.putBits(jpegValueBits(diff, cat), cat);

  uint8_t zeroRun = 0;
  for (uint8_t i = 1; i < 64; i++) {
    int ac = zz[i];
    if (ac == 0) {
      zeroRun++;
      continue;
    }
    while (zeroRun >= 16) {
      bw.putBits(acTable[0xF0].code, acTable[0xF0].size);
      zeroRun -= 16;
    }
    cat = jpegCategory(ac);
    uint8_t sym = (uint8_t)((zeroRun << 4) | cat);
    bw.putBits(acTable[sym].code, acTable[sym].size);
    bw.putBits(jpegValueBits(ac, cat), cat);
    zeroRun = 0;
  }
  if (zeroRun > 0) bw.putBits(acTable[0x00].code, acTable[0x00].size);
}

static bool jpegEncodeGrayscaleFromAtasciiGrid(const std::vector<uint8_t>& cells,
                                               uint16_t cols, uint16_t rows,
                                               uint8_t profileId, uint8_t fontId, uint8_t scaleX, uint8_t scaleY,
                                               uint16_t width, uint16_t height,
                                               uint16_t marginX, uint16_t marginY,
                                               std::vector<uint8_t>& jpg, uint8_t quality) {
  uint8_t qt[64];
  jpegMakeQualityTable(qt, quality);

  JpegHuffSymbol dcTable[256];
  JpegHuffSymbol acTable[256];
  jpegBuildHuffman(JPEG_DC_BITS, JPEG_DC_VALS, sizeof(JPEG_DC_VALS), dcTable);
  jpegBuildHuffman(JPEG_AC_BITS, JPEG_AC_VALS, sizeof(JPEG_AC_VALS), acTable);

  try {
    jpg.clear();
    if (g_prnActiveJpegReserveBytes > 0 && jpg.capacity() < g_prnActiveJpegReserveBytes) {
      jpg.reserve(g_prnActiveJpegReserveBytes);
    }

    jpegAppendMarker(jpg, 0xD8); // SOI

    jpegAppendMarker(jpg, 0xE0); // APP0 JFIF
    jpegAppendU16(jpg, 16);
    jpg.push_back('J'); jpg.push_back('F'); jpg.push_back('I'); jpg.push_back('F'); jpg.push_back(0);
    jpg.push_back(1); jpg.push_back(1);
    jpg.push_back(1); // dpi
    jpegAppendU16(jpg, 90); jpegAppendU16(jpg, 90);
    jpg.push_back(0); jpg.push_back(0);

    jpegAppendDqt(jpg, qt);

    jpegAppendMarker(jpg, 0xC0);
    jpegAppendU16(jpg, 11);
    jpg.push_back(8);
    jpegAppendU16(jpg, height);
    jpegAppendU16(jpg, width);
    jpg.push_back(1);
    jpg.push_back(1);
    jpg.push_back(0x11);
    jpg.push_back(0);

    jpegAppendDht(jpg, 0x00, JPEG_DC_BITS, JPEG_DC_VALS, sizeof(JPEG_DC_VALS));
    jpegAppendDht(jpg, 0x10, JPEG_AC_BITS, JPEG_AC_VALS, sizeof(JPEG_AC_VALS));

    jpegAppendMarker(jpg, 0xDA);
    jpegAppendU16(jpg, 8);
    jpg.push_back(1);
    jpg.push_back(1);
    jpg.push_back(0x00);
    jpg.push_back(0); jpg.push_back(63); jpg.push_back(0);

    JpegBitWriter bw(jpg);
    int prevDC = 0;
    for (uint16_t by = 0; by < height; by += 8) {
      for (uint16_t bx = 0; bx < width; bx += 8) {
        jpegEncodeOneBlockAtasciiGrid(bw, cells, cols, rows, profileId, fontId, scaleX, scaleY, width, height, marginX, marginY, bx, by, qt, dcTable, acTable, prevDC);
        delay(0);
      }
    }
    bw.finish();
    jpegAppendMarker(jpg, 0xD9);
  } catch (...) {
    g_prnLastError = "Sin memoria codificando JPEG desde grilla";
    return false;
  }
  return true;
}

static bool jpegEncodeGrayscaleFromAtasciiGridToFile(const std::vector<uint8_t>& cells,
                                                     uint16_t cols, uint16_t rows,
                                                     uint8_t profileId, uint8_t fontId, uint8_t scaleX, uint8_t scaleY,
                                                     uint16_t width, uint16_t height,
                                                     uint16_t marginX, uint16_t marginY,
                                                     const String& finalPath,
                                                     size_t* outBytes,
                                                     uint8_t quality) {
  if (outBytes) *outBytes = 0;
  if (!printerEnsurePrintSpoolDir()) return false;

  uint8_t qt[64];
  jpegMakeQualityTable(qt, quality);

  JpegHuffSymbol dcTable[256];
  JpegHuffSymbol acTable[256];
  jpegBuildHuffman(JPEG_DC_BITS, JPEG_DC_VALS, sizeof(JPEG_DC_VALS), dcTable);
  jpegBuildHuffman(JPEG_AC_BITS, JPEG_AC_VALS, sizeof(JPEG_AC_VALS), acTable);

  String tmpPath = String(PRN_JPEG_TMP_PATH);
  if (SPIFFS.exists(tmpPath)) SPIFFS.remove(tmpPath);
  File f = SPIFFS.open(tmpPath, "w");
  if (!f) {
    g_prnLastError = "No se pudo abrir JPEG temporal en SD";
    return false;
  }

  size_t bytes = 0;
  bool ok = true;
  auto wb = [&](uint8_t b) { if (ok) ok = jpegFileWriteByte(f, b, bytes); };

  ok = ok && jpegFileAppendMarker(f, 0xD8, bytes); // SOI

  ok = ok && jpegFileAppendMarker(f, 0xE0, bytes); // APP0 JFIF
  ok = ok && jpegFileAppendU16(f, 16, bytes);
  wb('J'); wb('F'); wb('I'); wb('F'); wb(0);
  wb(1); wb(1); wb(1);
  ok = ok && jpegFileAppendU16(f, 90, bytes);
  ok = ok && jpegFileAppendU16(f, 90, bytes);
  wb(0); wb(0);

  ok = ok && jpegFileAppendDqt(f, qt, bytes);

  ok = ok && jpegFileAppendMarker(f, 0xC0, bytes);
  ok = ok && jpegFileAppendU16(f, 11, bytes);
  wb(8);
  ok = ok && jpegFileAppendU16(f, height, bytes);
  ok = ok && jpegFileAppendU16(f, width, bytes);
  wb(1); wb(1); wb(0x11); wb(0);

  ok = ok && jpegFileAppendDht(f, 0x00, JPEG_DC_BITS, JPEG_DC_VALS, sizeof(JPEG_DC_VALS), bytes);
  ok = ok && jpegFileAppendDht(f, 0x10, JPEG_AC_BITS, JPEG_AC_VALS, sizeof(JPEG_AC_VALS), bytes);

  ok = ok && jpegFileAppendMarker(f, 0xDA, bytes);
  ok = ok && jpegFileAppendU16(f, 8, bytes);
  wb(1); wb(1); wb(0x00); wb(0); wb(63); wb(0);

  if (ok) {
    JpegFileBitWriter bw(f, bytes);
    int prevDC = 0;
    for (uint16_t by = 0; by < height && bw.ok; by += 8) {
      for (uint16_t bx = 0; bx < width && bw.ok; bx += 8) {
        jpegEncodeOneBlockAtasciiGridFile(bw, cells, cols, rows, profileId, fontId, scaleX, scaleY, width, height, marginX, marginY, bx, by, qt, dcTable, acTable, prevDC);
        delay(0);
      }
    }
    bw.finish();
    ok = bw.ok;
  }

  ok = ok && jpegFileAppendMarker(f, 0xD9, bytes); // EOI
  f.flush();
  f.close();

  if (!ok || bytes == 0) {
    SPIFFS.remove(tmpPath);
    g_prnLastError = "Error escribiendo JPEG temporal en SD";
    return false;
  }

  if (SPIFFS.exists(finalPath)) SPIFFS.remove(finalPath);
  if (!SPIFFS.rename(tmpPath, finalPath)) {
    // Fallback para SD/librerías que no permitan rename cruzado aunque sea mismo FS.
    File src = SPIFFS.open(tmpPath, "r");
    File dst = SPIFFS.open(finalPath, "w");
    if (!src || !dst) {
      if (src) src.close();
      if (dst) dst.close();
      SPIFFS.remove(tmpPath);
      g_prnLastError = "No se pudo mover JPEG temporal a final";
      return false;
    }
    uint8_t buf[512];
    while (src.available()) {
      int n = src.read(buf, sizeof(buf));
      if (n <= 0) break;
      if (dst.write(buf, n) != (size_t)n) { ok = false; break; }
      delay(0);
    }
    src.close();
    dst.flush();
    dst.close();
    SPIFFS.remove(tmpPath);
    if (!ok) {
      SPIFFS.remove(finalPath);
      g_prnLastError = "Error copiando JPEG temporal a final";
      return false;
    }
  }

  File check = SPIFFS.open(finalPath, "r");
  if (!check) {
    g_prnLastError = "JPEG final no se puede reabrir desde SD";
    return false;
  }
  size_t finalBytes = check.size();
  check.close();
  if (finalBytes == 0) {
    SPIFFS.remove(finalPath);
    g_prnLastError = "JPEG final vacío en SD";
    return false;
  }

  if (outBytes) *outBytes = finalBytes;
  return true;
}


// [F24] Eliminado helper no usado: jpegEncodeGrayscaleFromMonoRaster
static bool buildAtasciiJpegRender(const uint8_t* data, size_t len, uint8_t profileId, std::vector<uint8_t>& out, uint16_t rowsOverride = 0) {
  const VirtualPrinterProfile& prof = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(profileId)];

  // V56: layout fijo igual a V48/V50.
  // 40/80 columnas siempre salen como hoja vertical A4 744x1056.
  // 132 columnas sale horizontal 1056x744.
  // No usamos altura dinámica, porque algunas impresoras escalan una imagen baja
  // hasta llenar la hoja, provocando letra gigante y rotación/orientación no deseada.
  const uint8_t fontId = effectiveFontForProfile(profileId);
  const uint8_t glyphW = virtualGlyphW(fontId);
  const uint8_t glyphH = virtualGlyphH(fontId);
  const uint8_t scaleX = getVirtualPrinterEffectiveScaleX(profileId, fontId);
  const uint8_t scaleY = getVirtualPrinterEffectiveScaleY(profileId, fontId);
  uint16_t pageW = 744;
  uint16_t pageH = 1056;
  getVirtualPrinterPageSize(profileId, &pageW, &pageH);
  const uint8_t cellW = virtualCellW(profileId, fontId);
  const uint8_t cellH = virtualCellH(profileId, fontId);
  const uint16_t cols = getVirtualPrinterEffectiveColumns(profileId);
  const uint16_t fullRows = getVirtualPrinterEffectiveRows(profileId);
  uint16_t rows = fullRows;
  if (rowsOverride > 0 && rowsOverride < rows) rows = rowsOverride;
  if (rows < 10) rows = 10;
  const uint32_t textW = (uint32_t)cols * (uint32_t)cellW * (uint32_t)scaleX;
  const uint16_t marginX = ((uint32_t)pageW > textW) ? (uint16_t)(((uint32_t)pageW - textW) / 2U) : 12;
  const uint16_t marginY = 36;
  const size_t gridBytes = (size_t)cols * (size_t)rows;
  uint32_t blackPixels = 0;

  logf("[ATASCII-JPEG] build V90 V81_BASE_JPEG_ADAPTIVE_SAFE perfil=%s fuente=%s cols=%u rows=%u orient=%s glyph=%ux%u cell=%ux%u scale=%ux%u page=%ux%u grid=%lu freeHeap=%lu",
       prof.name,
       virtualFontName(fontId),
       cols,
       rows,
       virtualOrientationName(getVirtualPrinterEffectiveOrientation(profileId)),
       glyphW,
       glyphH,
       cellW,
       cellH,
       scaleX,
       scaleY,
       pageW,
       pageH,
       (unsigned long)gridBytes,
       (unsigned long)ESP.getFreeHeap());
  if (rowsOverride > 0 && rowsOverride < fullRows) {
    logf("[ATASCII-JPEG] rowsOverride=%u fullRows=%u", (unsigned int)rows, (unsigned int)fullRows);
  }

  try {
    out.clear();
    g_prnActiveJpegReserveBytes = printerChooseJpegReserveBytes(114688);
    if (g_prnActiveJpegReserveBytes > 0 && out.capacity() < g_prnActiveJpegReserveBytes) {
      out.reserve(g_prnActiveJpegReserveBytes);
    }
    logf("[ATASCII-JPEG] reserve jpgCap=%lu q=%u freeHeap=%lu maxAlloc=%lu",
         (unsigned long)out.capacity(),
         (unsigned int)ATASCII_JPEG_QUALITY,
         (unsigned long)ESP.getFreeHeap(),
         (unsigned long)ESP.getMaxAllocHeap());
  } catch (...) {
    g_prnLastError = "Sin memoria reservando buffer JPEG";
    logf("[ATASCII-JPEG] ERROR reserve jpgCap=%lu freeHeap=%lu maxAlloc=%lu",
         (unsigned long)g_prnActiveJpegReserveBytes,
         (unsigned long)ESP.getFreeHeap(),
         (unsigned long)ESP.getMaxAllocHeap());
    return false;
  }

  std::vector<uint8_t> cells;
  try {
    cells.assign(gridBytes, 0x20); // grilla de caracteres; 0x20 = espacio ATASCII
  } catch (...) {
    g_prnLastError = "Sin memoria creando grilla ATASCII";
    logf("[ATASCII-JPEG] ERROR sin memoria grid=%lu freeHeap=%lu", (unsigned long)gridBytes, (unsigned long)ESP.getFreeHeap());
    return false;
  }

  uint16_t col = 0;
  uint16_t row = 0;
  bool justAutoWrapped = false;
  // V68: ESC (0x1B) se renderiza como glifo ATASCII si llega dentro de la línea.
  // 0x9B se conserva como EOL. Si la línea terminó justo en el ancho configurado,
  // el wrap automático ya avanzó de fila; el EOL no debe crear una línea vacía extra.

  auto newLine = [&]() {
    col = 0;
    row++;
  };

  auto consumeEol = [&]() {
    if (justAutoWrapped && col == 0) {
      justAutoWrapped = false;
      return;
    }
    newLine();
    justAutoWrapped = false;
  };

  auto putCell = [&](uint8_t c) {
    if (row >= rows) return;
    size_t idx = (size_t)row * cols + col;
    if (idx < cells.size()) cells[idx] = c;
    col++;
    if (col >= cols) {
      newLine();
      justAutoWrapped = true;
    } else {
      justAutoWrapped = false;
    }
  };

  for (size_t i = 0; i < len && row < rows; i++) {
    uint8_t c = data[i];
    if (c == 0x9B) { consumeEol(); continue; }
    if (c == 0x0D || c == 0x0A) { consumeEol(); continue; }
    putCell(c);
  }

  for (size_t i = 0; i < cells.size(); i++) {
    if (cells[i] != 0x20) {
      blackPixels += countAtasciiCellBlackPixelsEx(cells[i], fontId, scaleX, scaleY);
    }
  }

  logf("[ATASCII-JPEG] fixedPage page=%ux%u cols=%u rows=%u reserve=%lu qStart=%u",
       pageW, pageH, cols, rows, (unsigned long)g_prnActiveJpegReserveBytes, (unsigned int)ATASCII_JPEG_QUALITY);

  bool ok = false;
  uint8_t usedQuality = 0;
  float coverage = ((float)blackPixels * 100.0f) / ((float)pageW * (float)pageH);

  // V90: partiendo desde V81/V88, usamos reserva adaptativa según maxAlloc.
  // No forzamos 128 KB porque en ESP32 se fragmenta y falla antes de codificar.
  // La meta es sostener q=24/32 cuando el bloque contiguo real lo permite.
  for (uint8_t qi = 0; qi < (uint8_t)(sizeof(ATASCII_JPEG_QUALITY_FALLBACKS) / sizeof(ATASCII_JPEG_QUALITY_FALLBACKS[0])); qi++) {
    uint8_t qTry = ATASCII_JPEG_QUALITY_FALLBACKS[qi];
    g_prnLastError = "";
    out.clear();
    ok = jpegEncodeGrayscaleFromAtasciiGrid(cells,
                                             cols,
                                             rows,
                                             profileId,
                                             fontId,
                                             scaleX,
                                             scaleY,
                                             pageW,
                                             pageH,
                                             marginX,
                                             marginY,
                                             out,
                                             qTry);
    if (ok) {
      usedQuality = qTry;
      break;
    }
    logf("[ATASCII-JPEG] retry calidad q=%u fallo jpg=%lu cap=%lu err='%s' freeHeap=%lu",
         (unsigned int)qTry,
         (unsigned long)out.size(),
         (unsigned long)out.capacity(),
         g_prnLastError.c_str(),
         (unsigned long)ESP.getFreeHeap());
    out.clear();
    std::vector<uint8_t>().swap(out); // F49Z67: libera capacidad entre intentos para recuperar bloque contiguo
    delay(0);
  }

  if (!ok) {
    logf("[ATASCII-JPEG] ERROR encode jpg=%lu cap=%lu freeHeap=%lu", (unsigned long)out.size(), (unsigned long)out.capacity(), (unsigned long)ESP.getFreeHeap());
    return false;
  }

  g_prnLastError = "";
  logf("[ATASCII-JPEG] OK jpg=%lu grid=%lu blackPixels=%lu coverage=%.2f%% q=%u freeHeap=%lu",
       (unsigned long)out.size(),
       (unsigned long)gridBytes,
       (unsigned long)blackPixels,
       coverage,
       (unsigned int)usedQuality,
       (unsigned long)ESP.getFreeHeap());

  return true;
}

static bool buildAtasciiJpegRenderToFile(const uint8_t* data, size_t len, uint8_t profileId, const String& outPath, size_t* outBytes, uint16_t rowsOverride = 0) {
  if (outBytes) *outBytes = 0;
  const VirtualPrinterProfile& prof = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(profileId)];
  const uint8_t fontId = effectiveFontForProfile(profileId);
  const uint8_t glyphW = virtualGlyphW(fontId);
  const uint8_t glyphH = virtualGlyphH(fontId);
  const uint8_t scaleX = getVirtualPrinterEffectiveScaleX(profileId, fontId);
  const uint8_t scaleY = getVirtualPrinterEffectiveScaleY(profileId, fontId);
  uint16_t pageW = 744;
  uint16_t pageH = 1056;
  getVirtualPrinterPageSize(profileId, &pageW, &pageH);
  const uint8_t cellW = virtualCellW(profileId, fontId);
  const uint8_t cellH = virtualCellH(profileId, fontId);
  const uint16_t cols = getVirtualPrinterEffectiveColumns(profileId);
  const uint16_t fullRows = getVirtualPrinterEffectiveRows(profileId);
  uint16_t rows = fullRows;
  if (rowsOverride > 0 && rowsOverride < rows) rows = rowsOverride;
  if (rows < 10) rows = 10;
  const uint32_t textW = (uint32_t)cols * (uint32_t)cellW * (uint32_t)scaleX;
  const uint16_t marginX = ((uint32_t)pageW > textW) ? (uint16_t)(((uint32_t)pageW - textW) / 2U) : 12;
  const uint16_t marginY = 36;
  const size_t gridBytes = (size_t)cols * (size_t)rows;
  uint32_t blackPixels = 0;

  logf("[ATASCII-JPEG-SD] build perfil=%s fuente=%s cols=%u rows=%u orient=%s glyph=%ux%u cell=%ux%u scale=%ux%u page=%ux%u grid=%lu freeHeap=%lu maxAlloc=%lu path=%s",
       prof.name, virtualFontName(fontId), cols, rows,
       virtualOrientationName(getVirtualPrinterEffectiveOrientation(profileId)),
       glyphW, glyphH, cellW, cellH, scaleX, scaleY, pageW, pageH,
       (unsigned long)gridBytes, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap(), outPath.c_str());

  std::vector<uint8_t> cells;
  try {
    cells.assign(gridBytes, 0x20);
  } catch (...) {
    g_prnLastError = "Sin memoria creando grilla ATASCII para JPEG SD";
    return false;
  }

  uint16_t col = 0;
  uint16_t row = 0;
  bool justAutoWrapped = false;
  auto newLine = [&]() { col = 0; row++; };
  auto consumeEol = [&]() {
    if (justAutoWrapped && col == 0) { justAutoWrapped = false; return; }
    newLine();
    justAutoWrapped = false;
  };
  auto putCell = [&](uint8_t c) {
    if (row >= rows) return;
    size_t idx = (size_t)row * cols + col;
    if (idx < cells.size()) cells[idx] = c;
    col++;
    if (col >= cols) { newLine(); justAutoWrapped = true; }
    else justAutoWrapped = false;
  };

  for (size_t i = 0; i < len && row < rows; i++) {
    uint8_t c = data[i];
    if (c == 0x9B || c == 0x0D || c == 0x0A) { consumeEol(); continue; }
    putCell(c);
  }

  for (size_t i = 0; i < cells.size(); i++) {
    if (cells[i] != 0x20) blackPixels += countAtasciiCellBlackPixelsEx(cells[i], fontId, scaleX, scaleY);
  }
  float coverage = ((float)blackPixels * 100.0f) / ((float)pageW * (float)pageH);

  bool ok = false;
  uint8_t usedQuality = 0;
  size_t jpgBytes = 0;
  for (uint8_t qi = 0; qi < (uint8_t)(sizeof(ATASCII_JPEG_QUALITY_FALLBACKS) / sizeof(ATASCII_JPEG_QUALITY_FALLBACKS[0])); qi++) {
    uint8_t qTry = ATASCII_JPEG_QUALITY_FALLBACKS[qi];
    g_prnLastError = "";
    jpgBytes = 0;
    if (SPIFFS.exists(outPath)) SPIFFS.remove(outPath);
    ok = jpegEncodeGrayscaleFromAtasciiGridToFile(cells, cols, rows, profileId, fontId, scaleX, scaleY,
                                                  pageW, pageH, marginX, marginY, outPath, &jpgBytes, qTry);
    if (ok) { usedQuality = qTry; break; }
    logf("[ATASCII-JPEG-SD] retry q=%u fallo err='%s' freeHeap=%lu maxAlloc=%lu",
         (unsigned int)qTry, g_prnLastError.c_str(), (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
    delay(0);
  }

  if (!ok) {
    g_prnSdSpoolErr++;
    if (g_prnLastError.length() == 0) g_prnLastError = "No se pudo generar JPEG en SD";
    return false;
  }

  g_prnSdSpoolJobs++;
  g_prnSdSpoolOk++;
  g_prnSdSpoolLastBytes = (uint32_t)jpgBytes;
  g_prnSdSpoolLastQuality = usedQuality;
  g_prnSdSpoolLastPath = outPath;
  if (outBytes) *outBytes = jpgBytes;
  g_prnLastError = "";
  logf("[ATASCII-JPEG-SD] OK jpg=%lu grid=%lu blackPixels=%lu coverage=%.2f%% q=%u path=%s freeHeap=%lu",
       (unsigned long)jpgBytes, (unsigned long)gridBytes, (unsigned long)blackPixels, coverage,
       (unsigned int)usedQuality, outPath.c_str(), (unsigned long)ESP.getFreeHeap());
  return true;
}


// [F24] Eliminado helper no usado: buildAtasciiTableJpegRender
static bool buildAtasciiTableJpegRenderToFile(const String& outPath, size_t* outBytes, bool inversePage) {
  if (outBytes) *outBytes = 0;
  const uint8_t profileId = PRN_CFG.virtualProfile;
  const VirtualPrinterProfile& prof = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(profileId)];
  const uint16_t cols = getVirtualPrinterEffectiveColumns(profileId);
  const uint16_t rows = getVirtualPrinterEffectiveRows(profileId);
  uint16_t pageW = 744;
  uint16_t pageH = 1056;
  getVirtualPrinterPageSize(profileId, &pageW, &pageH);
  const uint8_t fontId = effectiveFontForProfile(profileId);
  const uint8_t scaleX = getVirtualPrinterEffectiveScaleX(profileId, fontId);
  const uint8_t scaleY = getVirtualPrinterEffectiveScaleY(profileId, fontId);
  const uint16_t textW = (uint16_t)((uint32_t)cols * (uint32_t)virtualCellW(profileId, fontId) * (uint32_t)scaleX);
  const uint16_t marginX = ((uint32_t)pageW > textW) ? (uint16_t)(((uint32_t)pageW - textW) / 2U) : 12;
  const uint16_t marginY = 36;
  const size_t gridBytes = (size_t)cols * (size_t)rows;
  uint32_t blackPixels = 0;
  const uint8_t tableQuality = ATASCII_JPEG_QUALITY;

  logf("[ATASCII-TABLE-SD] build page=%s perfil=%s cols=%u rows=%u orient=%s scale=%ux%u page=%ux%u grid=%lu freeHeap=%lu path=%s",
       inversePage ? "80-FF INVERSO" : "00-7F NORMAL", prof.name, cols, rows,
       virtualOrientationName(getVirtualPrinterEffectiveOrientation(profileId)), scaleX, scaleY,
       pageW, pageH, (unsigned long)gridBytes, (unsigned long)ESP.getFreeHeap(), outPath.c_str());

  std::vector<uint8_t> cells;
  try {
    cells.assign(gridBytes, 0x20);
  } catch (...) {
    g_prnLastError = "Sin memoria creando grilla tabla ATASCII SD";
    return false;
  }

  auto putCellAt = [&](uint16_t row, uint16_t col, uint8_t c) {
    if (row >= rows || col >= cols) return;
    cells[(size_t)row * cols + col] = c;
  };
  auto putText = [&](uint16_t row, uint16_t col, const char* t) {
    while (*t && col < cols) putCellAt(row, col++, (uint8_t)*t++);
  };
  auto hexNib = [](uint8_t v) -> uint8_t { return (uint8_t)((v < 10) ? ('0' + v) : ('A' + (v - 10))); };
  auto putHex2 = [&](uint16_t row, uint16_t col, uint8_t v) {
    putCellAt(row, col, hexNib((uint8_t)(v >> 4)));
    putCellAt(row, (uint16_t)(col + 1), hexNib((uint8_t)(v & 0x0F)));
  };

  uint16_t r = 0;
  putText(r++, 0, inversePage ? "ATASCII GLYPH TABLE - VIDEO INVERSO 80-FF" : "ATASCII GLYPH TABLE - NORMAL 00-7F");
  putText(r++, 0, "Cada codigo se dibuja como glifo Atari real en JPEG/IPP.");
  putText(r++, 0, inversePage ? "Pagina 2/2: bytes 80-FF = mismo glifo con video inverso." : "Pagina 1/2: bytes 00-7F = glifos normales.");
  r++;

  const uint8_t perGroup = (cols >= 72) ? 16 : ((cols >= 40) ? 8 : 4);
  auto putGroup = [&](uint16_t base, uint8_t count) {
    if ((r + 1) >= rows) return;
    putText(r, 0, inversePage ? "INV:" : "HEX:");
    for (uint8_t i = 0; i < count; i++) putHex2(r, (uint16_t)(5 + i * 4), (uint8_t)(base + i));
    r++;
    putText(r, 0, "GLF:");
    for (uint8_t i = 0; i < count; i++) putCellAt(r, (uint16_t)(5 + i * 4), (uint8_t)(base + i));
    r++;
  };

  const uint16_t startCode = inversePage ? 0x80 : 0x00;
  const uint16_t endCode   = inversePage ? 0x100 : 0x80;
  for (uint16_t base = startCode; base < endCode && (r + 1) < rows; base += perGroup) {
    uint8_t count = (uint8_t)((base + perGroup <= endCode) ? perGroup : (endCode - base));
    putGroup(base, count);
  }

  for (size_t i = 0; i < cells.size(); i++) {
    if (cells[i] != 0x20) blackPixels += countAtasciiCellBlackPixelsEx(cells[i], fontId, scaleX, scaleY);
  }

  size_t jpgBytes = 0;
  bool ok = jpegEncodeGrayscaleFromAtasciiGridToFile(cells, cols, rows, profileId, fontId, scaleX, scaleY,
                                                     pageW, pageH, marginX, marginY, outPath, &jpgBytes, tableQuality);
  float coverage = ((float)blackPixels * 100.0f) / ((float)pageW * (float)pageH);
  if (!ok) {
    logf("[ATASCII-TABLE-SD] ERROR page=%s err='%s' freeHeap=%lu",
         inversePage ? "80-FF" : "00-7F", g_prnLastError.c_str(), (unsigned long)ESP.getFreeHeap());
    return false;
  }

  g_prnSdSpoolJobs++;
  g_prnSdSpoolOk++;
  g_prnSdSpoolLastBytes = (uint32_t)jpgBytes;
  g_prnSdSpoolLastQuality = tableQuality;
  g_prnSdSpoolLastPath = outPath;
  if (outBytes) *outBytes = jpgBytes;
  logf("[ATASCII-TABLE-SD] OK page=%s jpg=%lu blackPixels=%lu coverage=%.2f%% q=%u freeHeap=%lu",
       inversePage ? "80-FF" : "00-7F", (unsigned long)jpgBytes, (unsigned long)blackPixels, coverage,
       (unsigned int)tableQuality, (unsigned long)ESP.getFreeHeap());
  return true;
}


bool printerEnsureStaReady() {
  if (WiFi.status() != WL_CONNECTED) {
    if (PRN_CFG.staEnabled) {
      connectPrinterStaWifi(true);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    g_prnLastError = "MASTER sin WiFi STA conectado. Active 'Conectar MASTER a red Wi-Fi' y configure SSID/clave de la red donde está la Brother.";
    return false;
  }

  if (strlen(PRN_CFG.ip) == 0) {
    g_prnLastError = "IP de impresora vacía";
    return false;
  }

  return true;
}

bool printerSendRaw9100Job(const String& text, bool sendFormFeed) {
  WiFiClient client;
  g_prnLastBytes = 0;

  if (!printerEnsureStaReady()) return false;

  if (!client.connect(PRN_CFG.ip, PRN_CFG.port)) {
    g_prnLastError = "No conecta a impresora RAW " + String(PRN_CFG.ip) + ":" + String(PRN_CFG.port) + " desde STA " + WiFi.localIP().toString();
    return false;
  }

  // El puerto 9100 puede aceptar el socket, pero la hoja puede quedar
  // pendiente si no recibe fin de página. La prueba envía todo en una
  // sola conexión y opcionalmente agrega Form Feed (0x0C).
  size_t sent = 0;
  sent += client.print(text);
  if (sendFormFeed) {
    uint8_t ff = 0x0C;
    sent += client.write(&ff, 1);
  }

  client.flush();
  if (PRN_CFG.rawCloseDelayMs > 0) delay(PRN_CFG.rawCloseDelayMs);
  client.stop();

  g_prnLastBytes = (uint32_t)sent;
  g_prnLastError = "";
  logf("[PRN] RAW9100 enviado a %s:%u bytes=%lu ff=%u",
       PRN_CFG.ip,
       PRN_CFG.port,
       (unsigned long)g_prnLastBytes,
       sendFormFeed ? 1 : 0);
  return sent > 0;
}

bool printerSendRaw9100(const String& text) {
  String job = text;
  if (PRN_CFG.appendCrLf) job += "\r\n";
  return printerSendRaw9100Job(job, false);
}

// [F24] Eliminado helper no usado: urlEncodeQuery
static String jsonEscapeString(const String& value) {
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)c < 0x20) {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04X", (unsigned int)(uint8_t)c);
          out += buf;
        } else {
          out += c;
        }
        break;
    }
  }
  return out;
}

static String base64EncodeBytes(const uint8_t* data, size_t len) {
  static const char tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out;
  out.reserve(((len + 2) / 3) * 4 + 4);
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = ((uint32_t)data[i]) << 16;
    bool have2 = (i + 1 < len);
    bool have3 = (i + 2 < len);
    if (have2) v |= ((uint32_t)data[i + 1]) << 8;
    if (have3) v |= data[i + 2];
    out += tbl[(v >> 18) & 0x3F];
    out += tbl[(v >> 12) & 0x3F];
    out += have2 ? tbl[(v >> 6) & 0x3F] : '=';
    out += have3 ? tbl[v & 0x3F] : '=';
  }
  return out;
}

static String printerGatewayJobUrl() {
  String base = printerGatewayNormalizeBase();
  if (base.length() == 0) return "";
  // F25: endpoint RAW ATASCII. El Gateway v31 renderiza bytes ATASCII con matriz 8x8,
  // preservando caracteres especiales y video inverso. /print queda como fallback texto.
  return base + "/api/print/job";
}

static String printerGatewayNormalizeBase() {
  String base = String(PRN_CFG.gateway);
  base.trim();
  while (base.endsWith("/")) base.remove(base.length() - 1);
  if (base.endsWith("/api/print/job")) {
    base.remove(base.length() - String("/api/print/job").length());
  } else if (base.endsWith("/api/print/text")) {
    base.remove(base.length() - String("/api/print/text").length());
  } else if (base.endsWith("/print")) {
    base.remove(base.length() - String("/print").length());
  } else if (base.endsWith("/status")) {
    base.remove(base.length() - String("/status").length());
  } else if (base.endsWith("/test")) {
    base.remove(base.length() - String("/test").length());
  } else if (base.endsWith("/health")) {
    base.remove(base.length() - String("/health").length());
  }
  return base;
}

static String printerGatewayHealthUrl() {
  String base = printerGatewayNormalizeBase();
  if (base.length() == 0) return "";
  return base + "/status";
}

static String printerGatewayTextUrl() {
  String base = printerGatewayNormalizeBase();
  if (base.length() == 0) return "";

  // F49Z67: Gateway PowerShell v29 escucha POST /print.
  // Mantenerlo simple evita 404 por /api/print/text.
  return base + "/print";
}


static String printerBuildGatewayGlyphRowsJson() {
  String out;
  out.reserve(128 * 8 * 5 + 512);
  out += "[";
  for (uint16_t code = 0; code < 128; code++) {
    if (code > 0) out += ",";
    out += "[";
    for (uint8_t y = 0; y < 8; y++) {
      if (y > 0) out += ",";
      uint8_t row = 0;
      if (g_customFont8Valid[code]) row = (uint8_t)(g_customFont8[code][y] & 0x00FF);
      else row = builtinDefaultGlyphRow8((uint8_t)code, y);
      out += "\"";
      out += hexGlyph8(row);
      out += "\"";
    }
    out += "]";
  }
  out += "]";
  return out;
}

static String printerBuildGatewayAtasciiJson(const uint8_t* data, size_t len, const char* reason) {
  String raw64 = base64EncodeBytes(data, len);
  String glyphs = printerBuildGatewayGlyphRowsJson();
  uint16_t cols = getVirtualPrinterEffectiveColumns(PRN_CFG.virtualProfile);
  if (cols == 0) cols = 40;

  String json;
  json.reserve(raw64.length() + glyphs.length() + 512);
  json += "{";
  json += "\"format\":\"ATASCII_RAW_V1\"";
  json += ",\"reason\":\"" + jsonEscape(String(reason ? reason : "manual")) + "\"";
  json += ",\"rawBase64\":\"" + raw64 + "\"";
  json += ",\"rawBytes\":" + String((unsigned long)len);
  json += ",\"columns\":" + String((unsigned int)cols);
  json += ",\"fontScale\":" + String((unsigned int)PRN_CFG.fontScale);
  json += ",\"profile\":\"" + jsonEscape(String(VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(PRN_CFG.virtualProfile)].name)) + "\"";
  json += ",\"paper\":\"" + jsonEscape(String(virtualPaperName(PRN_CFG.paperSize))) + "\"";
  json += ",\"orientation\":\"" + jsonEscape(String(virtualOrientationName(getVirtualPrinterEffectiveOrientation(PRN_CFG.virtualProfile)))) + "\"";
  json += ",\"glyphRowsHex\":" + glyphs;
  json += "}";
  return json;
}

static bool printerSendHttpGatewayAtasciiRawJob(const uint8_t* data, size_t len, const char* reason) {
  g_prnLastBytes = 0;

  if (!data || len == 0) {
    g_prnLastError = "Buffer ATASCII vacío para Gateway";
    return false;
  }
  if (strlen(PRN_CFG.gateway) == 0) {
    g_prnLastError = "Gateway Windows vacío. Configure, por ejemplo: http://IP_DEL_PC:8080";
    return false;
  }

  if (PRN_CFG.staEnabled && WiFi.status() != WL_CONNECTED) {
    connectPrinterStaWifi(false);
  }

  String url = printerGatewayJobUrl();
  if (url.length() == 0) {
    g_prnLastError = "URL Gateway Windows inválida";
    return false;
  }

  String json = printerBuildGatewayAtasciiJson(data, len, reason);

  HTTPClient http;
  http.setReuse(false);
  http.setConnectTimeout(15000);
  http.setTimeout(60000);

  if (!http.begin(url)) {
    g_prnLastError = "No se pudo iniciar Gateway RAW ATASCII: " + url;
    return false;
  }

  http.addHeader("Content-Type", "application/json; charset=utf-8");
  http.addHeader("X-Atari-Printer", "P:");
  http.addHeader("X-Atari-Printer-Device", String(PRN_CFG.sioDev, HEX));
  http.addHeader("X-Atari-Gateway-Mode", "atascii-raw-v1");
  http.addHeader("Connection", "close");

  logf("[PRN-GW-RAW] POST %s rawBytes=%lu jsonBytes=%lu profile=%s paper=%s orient=%s",
       url.c_str(),
       (unsigned long)len,
       (unsigned long)json.length(),
       VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(PRN_CFG.virtualProfile)].name,
       virtualPaperName(PRN_CFG.paperSize),
       virtualOrientationName(getVirtualPrinterEffectiveOrientation(PRN_CFG.virtualProfile)));

  int code = http.POST((uint8_t*)json.c_str(), json.length());
  String resp = http.getString();
  http.end();

  g_prnLastBytes = (uint32_t)len;

  if (code < 200 || code >= 300) {
    if (code == 404) {
      g_prnLastError = "Gateway no soporta /api/print/job RAW ATASCII. Use AtariPrintGateway v31. resp=" + resp;
    } else if (code < 0) {
      g_prnLastError = "Gateway RAW ATASCII sin respuesta code=" + String(code) +
        " url=" + url +
        " staIp=" + WiFi.localIP().toString() +
        " apIp=" + WiFi.softAPIP().toString() +
        " resp=" + resp;
    } else {
      g_prnLastError = "Gateway RAW ATASCII error code=" + String(code) + " resp=" + resp;
    }
    logf("[PRN-GW-RAW] ERROR code=%d err=%s", code, g_prnLastError.c_str());
    return false;
  }

  g_prnLastError = "";
  logf("[PRN-GW-RAW] OK gateway=%s rawBytes=%lu jsonBytes=%lu code=%d resp=%s",
       url.c_str(),
       (unsigned long)len,
       (unsigned long)json.length(),
       code,
       resp.substring(0, 160).c_str());
  return true;
}

bool printerSendHttpGatewayJob(const String& text) {
  g_prnLastBytes = 0;

  if (strlen(PRN_CFG.gateway) == 0) {
    g_prnLastError = "Gateway Windows vacío. Configure, por ejemplo: http://IP_DEL_PC:7077";
    return false;
  }

  // V121: El Gateway puede estar en la red STA o conectado al AP XF551_MASTER.
  // Si STA está activado, intentamos conectarla, pero no bloqueamos el envío
  // para permitir gateway en 192.168.50.x vía SoftAP.
  if (PRN_CFG.staEnabled && WiFi.status() != WL_CONNECTED) {
    connectPrinterStaWifi(false);
  }

  String url = printerGatewayTextUrl();
  if (url.length() == 0) {
    g_prnLastError = "URL Gateway Windows inválida";
    return false;
  }

  logf("[PRN-GW] NET sta=%d staIp=%s apIp=%s gwHealth=%s",
       (int)WiFi.status(),
       WiFi.localIP().toString().c_str(),
       WiFi.softAPIP().toString().c_str(),
       printerGatewayHealthUrl().c_str());

  HTTPClient http;
  http.setReuse(false);
  http.setConnectTimeout(15000);
  http.setTimeout(60000);

  if (!http.begin(url)) {
    g_prnLastError = "No se pudo iniciar Gateway Windows: " + url;
    return false;
  }

  http.addHeader("Content-Type", "text/plain; charset=utf-8");
  http.addHeader("X-Atari-Printer", "P:");
  http.addHeader("X-Atari-Printer-Device", String(PRN_CFG.sioDev, HEX));
  http.addHeader("X-Atari-Gateway-Mode", "windows-driver");
  http.addHeader("Connection", "close");

  logf("[PRN-GW] POST %s bytes=%lu profile=%s font=%u paper=%s orient=%s",
       url.c_str(),
       (unsigned long)text.length(),
       VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(PRN_CFG.virtualProfile)].name,
       (unsigned int)PRN_CFG.fontScale,
       virtualPaperName(PRN_CFG.paperSize),
       virtualOrientationName(getVirtualPrinterEffectiveOrientation(PRN_CFG.virtualProfile)));

  int code = http.POST((uint8_t*)text.c_str(), text.length());
  String resp = http.getString();
  http.end();

  g_prnLastBytes = (uint32_t)text.length();

  if (code < 200 || code >= 300) {
    if (code < 0) {
      g_prnLastError = "Gateway Windows sin respuesta code=" + String(code) +
        " url=" + url +
        " staIp=" + WiFi.localIP().toString() +
        " apIp=" + WiFi.softAPIP().toString() +
        " resp=" + resp;
    } else {
      g_prnLastError = "Gateway Windows error code=" + String(code) + " resp=" + resp;
    }
    return false;
  }

  g_prnLastError = "";
  logf("[PRN-GW] OK gateway=%s bytes=%lu code=%d resp=%s",
       url.c_str(),
       (unsigned long)g_prnLastBytes,
       code,
       resp.substring(0, 160).c_str());
  return true;
}

void printerAppendHttpBuffer(const String& text) {
  String line = text;
  if (PRN_CFG.appendCrLf) line += "\r\n";
  else line += "\n";

  g_prnHttpBuffer += line;
  g_prnHttpBufferLines++;
  g_prnHttpLastLineMs = millis();
  g_prnHttpPending = true;

  g_prnLastBytes = (uint32_t)g_prnHttpBuffer.length();

  logf("[PRN] HTTP buffer add lines=%lu bytes=%lu text='%s'",
       (unsigned long)g_prnHttpBufferLines,
       (unsigned long)g_prnHttpBuffer.length(),
       text.c_str());
}

bool printerFlushHttpBuffer(const char* reason) {
  if (!g_prnHttpPending || g_prnHttpBuffer.length() == 0) return true;

  String job = g_prnHttpBuffer;
  uint32_t lines = g_prnHttpBufferLines;

  g_prnHttpBuffer = "";
  g_prnHttpBufferLines = 0;
  g_prnHttpPending = false;

  logf("[PRN] HTTP FLUSH reason=%s lines=%lu bytes=%lu idleMs=%lu maxLines=%lu",
       reason ? reason : "?",
       (unsigned long)lines,
       (unsigned long)job.length(),
       (unsigned long)PRN_HTTP_FLUSH_IDLE_MS,
       (unsigned long)PRN_HTTP_FLUSH_MAX_LINES);

  bool ok = printerSendHttpGatewayJob(job);
  if (!ok) {
    logf("[PRN] HTTP FLUSH ERROR: %s", g_prnLastError.c_str());
  }
  return ok;
}

void servicePrinterHttpBuffer() {
  // V52: deshabilitado para el flujo Atari P:.
  // La impresión real desde Atari se acumula en g_prnVirtualBuffer y se libera
  // manualmente con "Imprimir buffer Atari" o automáticamente al llenar página.
  return;
}


// ================== IPP directo experimental ==================
// Tu Brother DCP-T720DW anuncia IPP en puerto 631, ruta /ipp/print,
// y pdl=image/jpeg,image/urf,image/pwg-raster,application/vnd.brother-hbp.
// Esta prueba envía un JPEG fijo como document-format=image/jpeg.
// NO reemplaza todavía el flujo P: completo; sirve para validar IPP directo desde ESP32.

static const uint8_t IPP_TEST_JPEG[] PROGMEM = {
  0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01,
  0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43, 0x00, 0x03, 0x02, 0x02, 0x03, 0x02, 0x02, 0x03,
  0x03, 0x03, 0x03, 0x04, 0x03, 0x03, 0x04, 0x05, 0x08, 0x05, 0x05, 0x04, 0x04, 0x05, 0x0A, 0x07,
  0x07, 0x06, 0x08, 0x0C, 0x0A, 0x0C, 0x0C, 0x0B, 0x0A, 0x0B, 0x0B, 0x0D, 0x0E, 0x12, 0x10, 0x0D,
  0x0E, 0x11, 0x0E, 0x0B, 0x0B, 0x10, 0x16, 0x10, 0x11, 0x13, 0x14, 0x15, 0x15, 0x15, 0x0C, 0x0F,
  0x17, 0x18, 0x16, 0x14, 0x18, 0x12, 0x14, 0x15, 0x14, 0xFF, 0xDB, 0x00, 0x43, 0x01, 0x03, 0x04,
  0x04, 0x05, 0x04, 0x05, 0x09, 0x05, 0x05, 0x09, 0x14, 0x0D, 0x0B, 0x0D, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0xFF, 0xC0,
  0x00, 0x11, 0x08, 0x01, 0x40, 0x02, 0x80, 0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11,
  0x01, 0xFF, 0xC4, 0x00, 0x1F, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
  0x0A, 0x0B, 0xFF, 0xC4, 0x00, 0xB5, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05,
  0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21,
  0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08, 0x23,
  0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16, 0x17,
  0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A,
  0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A,
  0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A,
  0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
  0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
  0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5,
  0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF1,
  0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xC4, 0x00, 0x1F, 0x01, 0x00, 0x03,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
  0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00, 0xB5, 0x11, 0x00,
  0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00,
  0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13,
  0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0, 0x15,
  0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26, 0x27,
  0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
  0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
  0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
  0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6,
  0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4,
  0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE2,
  0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9,
  0xFA, 0xFF, 0xDA, 0x00, 0x0C, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00, 0x3F, 0x00, 0xFD,
  0x53, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0xF9, 0x1F, 0xFE, 0x0A, 0x5F,
  0xE2, 0xAB, 0xE5, 0xF8, 0x13, 0xA5, 0xFC, 0x39, 0xD0, 0xE5, 0xDB, 0xE2, 0x3F, 0x89, 0x3A, 0xF5,
  0x97, 0x86, 0x6D, 0x15, 0x7E, 0xF0, 0x8D, 0xE5, 0x0D, 0x2B, 0x63, 0xFB, 0xB8, 0x55, 0x46, 0xF6,
  0x96, 0x80, 0x3E, 0xB8, 0xA2, 0xBE, 0x2C, 0xFF, 0x00, 0x82, 0x75, 0xAB, 0xFC, 0x27, 0xD7, 0x3E,
  0x33, 0x7C, 0x01, 0xBB, 0xB8, 0x96, 0x56, 0xF0, 0x27, 0x88, 0x9A, 0xEF, 0x4A, 0x37, 0x07, 0xF7,
  0x8F, 0xA6, 0xDD, 0xAE, 0xF8, 0x4F, 0xFE, 0x3A, 0x1C, 0xE3, 0xBC, 0xD5, 0xE9, 0x9F, 0x11, 0x3F,
  0xE0, 0x9F, 0x7F, 0x00, 0xBE, 0x2B, 0xF8, 0xD7, 0x55, 0xF1, 0x6F, 0x8A, 0xBC, 0x05, 0xFD, 0xA9,
  0xE2, 0x0D, 0x52, 0x51, 0x35, 0xE5, 0xE7, 0xF6, 0xC6, 0xA1, 0x0F, 0x9A, 0xE1, 0x42, 0x83, 0xB2,
  0x39, 0xD5, 0x07, 0x0A, 0x07, 0x00, 0x74, 0xA0, 0x0F, 0xA1, 0xE8, 0xAF, 0xCA, 0x4F, 0xF8, 0x26,
  0x9F, 0xEC, 0x53, 0xF0, 0x63, 0xF6, 0x80, 0xF8, 0x03, 0xAA, 0xF8, 0x93, 0xC7, 0xDE, 0x0D, 0xFE,
  0xDE, 0xD6, 0xA0, 0xF1, 0x15, 0xD5, 0x84, 0x77, 0x3F, 0xDA, 0x97, 0xB6, 0xDB, 0x60, 0x48, 0x60,
  0x65, 0x4D, 0xB0, 0xCC, 0x8B, 0xC1, 0x76, 0x39, 0xC6, 0x79, 0xEB, 0xD2, 0xBF, 0x43, 0x3C, 0x6B,
  0xAB, 0xF8, 0x77, 0xF6, 0x5C, 0xFD, 0x9D, 0xB5, 0x8B, 0xFD, 0x2A, 0xCD, 0x6C, 0x3C, 0x3D, 0xE0,
  0xBD, 0x06, 0x43, 0x61, 0x62, 0xF2, 0xBC, 0x80, 0x2C, 0x31, 0x62, 0x18, 0x77, 0xBB, 0x16, 0x62,
  0xCC, 0x11, 0x01, 0x62, 0x49, 0x27, 0x93, 0x40, 0x1E, 0x9F, 0x45, 0x7E, 0x54, 0xFE, 0xC2, 0x9E,
  0x1F, 0xD6, 0xFF, 0x00, 0x66, 0xBF, 0x8F, 0x7F, 0x0A, 0x6F, 0xBC, 0x45, 0x79, 0x3C, 0xB6, 0xFF,
  0x00, 0x1C, 0xFC, 0x27, 0x73, 0x7D, 0x73, 0x25, 0xC9, 0xC0, 0xFE, 0xD2, 0x59, 0xDE, 0xEA, 0x23,
  0xF5, 0xFB, 0x3B, 0xC2, 0x39, 0xFE, 0x2B, 0x86, 0xF6, 0xAF, 0x63, 0xFF, 0x00, 0x82, 0xC5, 0x5A,
  0x45, 0x7F, 0xF0, 0x07, 0xE1, 0xFD, 0xB4, 0xE9, 0xE6, 0x41, 0x37, 0x8E, 0xEC, 0x63, 0x91, 0x72,
  0x46, 0x54, 0xDA, 0xDE, 0x02, 0x32, 0x39, 0xE8, 0x68, 0x03, 0xEF, 0x3A, 0x2B, 0xF3, 0xF3, 0xF6,
  0xA6, 0xFD, 0x85, 0x3E, 0x1A, 0x7C, 0x02, 0xF8, 0x1D, 0xE2, 0x8F, 0x88, 0xFF, 0x00, 0x08, 0x46,
  0xAF, 0xF0, 0xCB, 0xC6, 0x3E, 0x17, 0xB7, 0x1A, 0x9D, 0xB6, 0xA1, 0xA5, 0xEB, 0x97, 0x8E, 0xB3,
  0x6C, 0x65, 0xDD, 0x14, 0x8B, 0x2C, 0xAF, 0x90, 0xCA, 0x48, 0x18, 0xC7, 0x38, 0xCE, 0x46, 0x41,
  0xFA, 0x07, 0xE1, 0xA6, 0x9B, 0xA2, 0xFE, 0xDA, 0x3F, 0xB2, 0x17, 0x82, 0xDF, 0xE2, 0x86, 0x8C,
  0xBA, 0xBD, 0xA7, 0x88, 0xB4, 0xCB, 0x5B, 0xBD, 0x4A, 0xCD, 0x26, 0x96, 0xD5, 0x66, 0xB8, 0x8D,
  0x81, 0xF3, 0x01, 0x85, 0x91, 0x94, 0x17, 0x4D, 0xE0, 0x03, 0x8C, 0x1C, 0x72, 0x28, 0x03, 0xE8,
  0x2A, 0x2B, 0xF2, 0x93, 0xE1, 0x8F, 0xEC, 0x53, 0xF0, 0x63, 0xC4, 0x5F, 0xF0, 0x51, 0x3F, 0x8C,
  0x1F, 0x0C, 0x75, 0x0F, 0x06, 0xFD, 0xA3, 0xC0, 0xDA, 0x07, 0x87, 0xEC, 0xEF, 0xB4, 0xDD, 0x2B,
  0xFB, 0x52, 0xF5, 0x7C, 0x89, 0x9E, 0x1B, 0x16, 0x76, 0xF3, 0x56, 0x61, 0x23, 0x64, 0xCD, 0x29,
  0xC3, 0x31, 0x1F, 0x37, 0x03, 0x81, 0x8F, 0xD4, 0x4F, 0x08, 0x78, 0x4F, 0x4A, 0xF0, 0x1F, 0x85,
  0x34, 0x6F, 0x0D, 0x68, 0x56, 0xBF, 0x61, 0xD1, 0x34, 0x7B, 0x38, 0xAC, 0x2C, 0x6D, 0x7C, 0xC7,
  0x93, 0xC9, 0x82, 0x24, 0x09, 0x1A, 0x6E, 0x72, 0x59, 0xB0, 0xAA, 0x06, 0x58, 0x92, 0x71, 0xC9,
  0x34, 0x01, 0xAF, 0x45, 0x15, 0x0D, 0xDD, 0xAC, 0x57, 0xD6, 0xB3, 0x5B, 0x4E, 0xBB, 0xE1, 0x99,
  0x1A, 0x37, 0x5C, 0x91, 0x95, 0x23, 0x04, 0x64, 0x73, 0xD0, 0xD0, 0x04, 0xD4, 0x57, 0xE5, 0x27,
  0xED, 0x7F, 0xFB, 0x14, 0xFC, 0x18, 0xF8, 0x5B, 0xFB, 0x42, 0x7E, 0xCD, 0x1E, 0x19, 0xF0, 0xC7,
  0x83, 0x7F, 0xB3, 0x34, 0x4F, 0x18, 0x78, 0x82, 0x7B, 0x1D, 0x72, 0xD7, 0xFB, 0x52, 0xF6, 0x5F,
  0xB5, 0xC2, 0xB2, 0xD9, 0x2A, 0xAE, 0xE9, 0x26, 0x66, 0x8F, 0x02, 0x69, 0x39, 0x42, 0xA7, 0xE6,
  0xEB, 0xC0, 0xC7, 0xE8, 0xCF, 0xC1, 0x4F, 0x80, 0x3E, 0x03, 0xFD, 0x9D, 0x7C, 0x33, 0x79, 0xE1,
  0xEF, 0x87, 0xBA, 0x17, 0xFC, 0x23, 0xFA, 0x3D, 0xDD, 0xE3, 0x5F, 0xCD, 0x6D, 0xF6, 0xCB, 0x8B,
  0x9D, 0xF3, 0xB2, 0x22, 0x17, 0xDD, 0x34, 0x8E, 0xC3, 0xE5, 0x8D, 0x06, 0x01, 0xC7, 0x1D, 0x32,
  0x4D, 0x00, 0x7A, 0x15, 0x15, 0xF9, 0x65, 0xFB, 0x16, 0xFE, 0xC8, 0xBF, 0x09, 0xFF, 0x00, 0x68,
  0xEF, 0x17, 0x7C, 0x7B, 0xD5, 0x3E, 0x21, 0xF8, 0x54, 0xEB, 0xFA, 0x86, 0x9D, 0xE3, 0x8B, 0xCB,
  0x6B, 0x49, 0xD7, 0x52, 0xBB, 0xB5, 0x31, 0x46, 0x64, 0x76, 0x23, 0x10, 0xCA, 0x80, 0xF3, 0xCE,
  0x48, 0x26, 0xBD, 0x33, 0xF6, 0x77, 0x4D, 0x53, 0xF6, 0x68, 0xFD, 0xBE, 0x75, 0xDF, 0x80, 0xFA,
  0x27, 0x88, 0xB5, 0x4D, 0x73, 0xE1, 0xAE, 0xA9, 0xA0, 0x7F, 0x6D, 0x58, 0x69, 0x9A, 0xA5, 0xD3,
  0x5C, 0xB6, 0x91, 0x28, 0x01, 0xB6, 0xA3, 0xB7, 0x2A, 0xBC, 0x38, 0xC7, 0x19, 0x0F, 0x19, 0x39,
  0x2B, 0x92, 0x01, 0xFA, 0x03, 0x45, 0x7E, 0x64, 0x78, 0x23, 0xF6, 0x6E, 0xF8, 0x73, 0xFB, 0x47,
  0xFF, 0x00, 0xC1, 0x43, 0xFF, 0x00, 0x68, 0xED, 0x3F, 0xE2, 0x2F, 0x87, 0x7F, 0xE1, 0x21, 0xB3,
  0xD3, 0x05, 0x94, 0xF6, 0x91, 0xFD, 0xBA, 0xE6, 0xD7, 0xCB, 0x76, 0x8E, 0x35, 0x63, 0x98, 0x24,
  0x42, 0x72, 0x07, 0x42, 0x48, 0xAE, 0x87, 0xFE, 0x11, 0x46, 0xFD, 0x88, 0xBF, 0x6E, 0x8F, 0x85,
  0x5E, 0x0D, 0xF8, 0x77, 0xAD, 0x6A, 0xA3, 0xE1, 0xDF, 0xC4, 0x18, 0x66, 0x82, 0xFB, 0xC2, 0x37,
  0xD7, 0xD2, 0x5D, 0xC3, 0x69, 0x22, 0xE5, 0x44, 0xD1, 0x79, 0x84, 0x95, 0xE7, 0x63, 0x02, 0x72,
  0xDF, 0x2C, 0x80, 0xB1, 0x07, 0x00, 0x03, 0xF4, 0x5A, 0x8A, 0x28, 0xA0, 0x02, 0x8A, 0xF9, 0xE3,
  0xE2, 0x27, 0xFC, 0x13, 0xEF, 0xE0, 0x17, 0xC5, 0x7F, 0x1A, 0xEA, 0xBE, 0x2D, 0xF1, 0x57, 0x80,
  0xBF, 0xB5, 0x3C, 0x41, 0xAA, 0x4A, 0x26, 0xBC, 0xBC, 0xFE, 0xD8, 0xD4, 0x21, 0xF3, 0x5C, 0x28,
  0x50, 0x76, 0x47, 0x3A, 0xA0, 0xE1, 0x40, 0xE0, 0x0E, 0x95, 0xF1, 0x5F, 0xFC, 0x13, 0x4F, 0xF6,
  0x29, 0xF8, 0x31, 0xFB, 0x40, 0x7C, 0x01, 0xD5, 0x7C, 0x49, 0xE3, 0xEF, 0x06, 0xFF, 0x00, 0x6F,
  0x6B, 0x50, 0x78, 0x8A, 0xEA, 0xC2, 0x3B, 0x9F, 0xED, 0x4B, 0xDB, 0x6D, 0xB0, 0x24, 0x30, 0x32,
  0xA6, 0xD8, 0x66, 0x45, 0xE0, 0xBB, 0x1C, 0xE3, 0x3C, 0xF5, 0xE9, 0x40, 0x1F, 0xAB, 0x74, 0x57,
  0xE6, 0xCF, 0xED, 0x9B, 0xF0, 0x3B, 0xC1, 0x3A, 0x87, 0xED, 0x41, 0xFB, 0x27, 0xFC, 0x2D, 0xB8,
  0xD1, 0x7C, 0xCF, 0x02, 0x2D, 0xB5, 0xDE, 0x96, 0x34, 0x9F, 0xB5, 0xCE, 0x3F, 0xD1, 0x90, 0x46,
  0x16, 0x3F, 0x34, 0x3F, 0x9B, 0xC6, 0xD1, 0xCE, 0xFC, 0xF1, 0xD6, 0xBE, 0x80, 0xB7, 0xFF, 0x00,
  0x82, 0x60, 0x7E, 0xCC, 0xD6, 0xB7, 0x11, 0x4F, 0x17, 0xC3, 0x4D, 0xB2, 0xC6, 0xC1, 0xD1, 0xBF,
  0xB7, 0xB5, 0x33, 0x82, 0x0E, 0x41, 0xFF, 0x00, 0x8F, 0x9A, 0x00, 0xFA, 0x9A, 0x8A, 0xF8, 0xB7,
  0xFE, 0x0A, 0x27, 0xF1, 0x1B, 0x5B, 0xB0, 0xF1, 0x1F, 0xC1, 0x4F, 0x86, 0x16, 0x7E, 0x28, 0xBB,
  0xF0, 0x3F, 0x86, 0xFE, 0x20, 0x6B, 0x92, 0x59, 0xEB, 0xBE, 0x20, 0xB0, 0x9B, 0xC8, 0x9E, 0x2B,
  0x68, 0xDA, 0x05, 0xF2, 0x52, 0x5F, 0xF9, 0x67, 0xBF, 0xCF, 0x39, 0x3F, 0xEC, 0x8C, 0xE5, 0x77,
  0x03, 0xAF, 0x7D, 0xFF, 0x00, 0x04, 0xC5, 0xF8, 0x49, 0x61, 0x63, 0x1C, 0xFE, 0x0B, 0xBC, 0xF1,
  0x4F, 0x81, 0x3C, 0x51, 0x11, 0x12, 0x47, 0xE2, 0x3D, 0x2B, 0x5E, 0xBA, 0x7B, 0x87, 0x90, 0x73,
  0xBA, 0x45, 0x79, 0x0A, 0xB0, 0x27, 0xA8, 0x50, 0x84, 0xF6, 0x22, 0x80, 0x3E, 0xBB, 0xA2, 0xBC,
  0x8B, 0xE3, 0x87, 0xEC, 0x9B, 0xF0, 0xA7, 0xF6, 0x90, 0xBF, 0xD2, 0xAF, 0x7E, 0x22, 0xF8, 0x57,
  0xFE, 0x12, 0x2B, 0x9D, 0x2E, 0x27, 0x86, 0xCD, 0xFF, 0x00, 0xB4, 0x6E, 0xED, 0x7C, 0xA4, 0x72,
  0x0B, 0x0C, 0x41, 0x2A, 0x03, 0x92, 0xA3, 0xAE, 0x7A, 0x57, 0xC0, 0x3F, 0x0D, 0x3F, 0x62, 0x9F,
  0x83, 0x1E, 0x20, 0xFF, 0x00, 0x82, 0x8B, 0x7C, 0x5C, 0xF8, 0x61, 0x7F, 0xE0, 0xDF, 0xB4, 0x78,
  0x1B, 0x42, 0xF0, 0xED, 0xA5, 0xFE, 0x9D, 0xA5, 0x7F, 0x6A, 0x5E, 0xAF, 0x91, 0x3B, 0xC5, 0x62,
  0xCC, 0xFE, 0x68, 0x98, 0x48, 0xD9, 0x33, 0x4B, 0xC3, 0x31, 0x1F, 0x37, 0x4E, 0x06, 0x00, 0x3F,
  0x56, 0xE8, 0xAF, 0x3A, 0xF8, 0x25, 0xFB, 0x3D, 0xF8, 0x03, 0xF6, 0x74, 0xF0, 0xF5, 0xF6, 0x87,
  0xF0, 0xF3, 0x40, 0xFF, 0x00, 0x84, 0x7B, 0x4B, 0xBD, 0xBA, 0x37, 0xB7, 0x10, 0x7D, 0xB2, 0xE2,
  0xE7, 0x7C, 0xDB, 0x15, 0x37, 0x6E, 0x9A, 0x47, 0x61, 0xF2, 0xA2, 0x8C, 0x02, 0x07, 0x1D, 0x2B,
  0xD1, 0x68, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2,
  0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2,
  0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2,
  0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2,
  0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2,
  0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2,
  0x8A, 0x28, 0x00, 0xAF, 0xCE, 0xEF, 0xDA, 0x3B, 0xC6, 0x7E, 0x36, 0xF1, 0xE7, 0xFC, 0x14, 0x2F,
  0xC2, 0x96, 0x9E, 0x01, 0xF8, 0x7C, 0xDF, 0x14, 0x47, 0xC2, 0x5D, 0x1D, 0xB5, 0x0B, 0xAD, 0x13,
  0xFB, 0x66, 0xDF, 0x4B, 0x48, 0xEF, 0x2F, 0x13, 0x89, 0x1A, 0x69, 0xB2, 0xB9, 0x54, 0x6B, 0x66,
  0x0A, 0x01, 0x39, 0x53, 0xD8, 0x1A, 0xFD, 0x11, 0xAF, 0x0B, 0xFD, 0x9F, 0x7F, 0x66, 0x37, 0xF8,
  0x29, 0xF1, 0x1B, 0xE2, 0xC7, 0x8D, 0xB5, 0x1F, 0x13, 0xFF, 0x00, 0xC2, 0x53, 0xAD, 0xF8, 0xFB,
  0x55, 0x5B, 0xF9, 0x64, 0xFB, 0x07, 0xD9, 0x45, 0x94, 0x28, 0xD2, 0x18, 0xED, 0xD7, 0xF7, 0xB2,
  0x6E, 0x0A, 0x25, 0xC6, 0xEF, 0x97, 0x21, 0x57, 0x8E, 0x28, 0x03, 0xE4, 0xFB, 0x0F, 0x88, 0x5F,
  0x11, 0xBC, 0x11, 0xFF, 0x00, 0x05, 0x0E, 0xF8, 0x7D, 0xE3, 0xBF, 0x88, 0x7F, 0x0A, 0xDB, 0xE1,
  0x36, 0x9F, 0xF1, 0x0B, 0x4D, 0x7F, 0x07, 0xCF, 0x0F, 0xFC, 0x24, 0x36, 0xBA, 0xBA, 0x5E, 0xDC,
  0x21, 0x0F, 0x0C, 0x85, 0xA0, 0x03, 0x63, 0x6F, 0x36, 0xB1, 0x80, 0xC3, 0x90, 0x0E, 0x09, 0xE4,
  0x57, 0xE9, 0x25, 0x78, 0x67, 0xED, 0x6D, 0xFB, 0x31, 0xFF, 0x00, 0xC3, 0x4F, 0xF8, 0x37, 0xC3,
  0xBA, 0x65, 0xAF, 0x89, 0x9B, 0xC1, 0xDA, 0xDE, 0x81, 0xAD, 0xC1, 0xAE, 0x69, 0xFA, 0xD2, 0x58,
  0xFD, 0xAD, 0xA1, 0x92, 0x35, 0x71, 0xB7, 0x67, 0x99, 0x1F, 0x07, 0x70, 0x39, 0xDD, 0xD5, 0x17,
  0xAD, 0x7B, 0x8C, 0x61, 0xC4, 0x68, 0x24, 0x65, 0x69, 0x00, 0x1B, 0x99, 0x57, 0x00, 0x9E, 0xE4,
  0x0C, 0x9C, 0x7E, 0x66, 0x80, 0x3E, 0x15, 0xFF, 0x00, 0x82, 0x37, 0xFF, 0x00, 0xC9, 0xAB, 0xEB,
  0x7F, 0xF6, 0x36, 0xDE, 0xFF, 0x00, 0xE8, 0x8B, 0x5A, 0xBD, 0xFF, 0x00, 0x05, 0x51, 0xF1, 0x1E,
  0xB1, 0xAD, 0x7C, 0x32, 0xF0, 0x57, 0xC2, 0x1F, 0x0B, 0xD9, 0x49, 0xAB, 0x78, 0x97, 0xE2, 0x1E,
  0xBB, 0x15, 0xB2, 0x69, 0x90, 0xCE, 0x90, 0x3D, 0xCD, 0xB5, 0xB9, 0x59, 0x64, 0x51, 0x23, 0x9D,
  0xA9, 0xFB, 0xC3, 0x6F, 0xF3, 0x37, 0xCA, 0x06, 0x49, 0xE9, 0x5E, 0xD1, 0xFB, 0x1B, 0xFE, 0xCB,
  0xFF, 0x00, 0xF0, 0xC9, 0x5F, 0x0A, 0xEF, 0xBC, 0x19, 0xFF, 0x00, 0x09, 0x37, 0xFC, 0x25, 0x5F,
  0x69, 0xD5, 0xE7, 0xD5, 0x7E, 0xDB, 0xF6, 0x0F, 0xB1, 0xED, 0xF3, 0x23, 0x89, 0x3C, 0xBD, 0x9E,
  0x6C, 0x99, 0xC7, 0x95, 0x9D, 0xDB, 0xB9, 0xDD, 0xD3, 0x8E, 0x5D, 0xE2, 0xBF, 0xD9, 0x8D, 0xFC,
  0x65, 0xFB, 0x58, 0xF8, 0x3B, 0xE3, 0x36, 0xA3, 0xE2, 0x7F, 0x32, 0xCF, 0xC2, 0xBA, 0x54, 0xD6,
  0x1A, 0x7F, 0x86, 0x85, 0x87, 0xCA, 0xB3, 0x4A, 0xB2, 0xAB, 0xDC, 0x19, 0xFC, 0xDE, 0xA4, 0x4B,
  0x8D, 0xBE, 0x5F, 0xF0, 0x2F, 0x3C, 0x50, 0x07, 0xC6, 0x1F, 0xB6, 0x67, 0x8B, 0x7E, 0x39, 0x5D,
  0x7C, 0x3A, 0xF0, 0x4F, 0x8B, 0xF5, 0x1F, 0xD9, 0xCB, 0xFE, 0x15, 0xBD, 0x9F, 0xC2, 0xED, 0x66,
  0xCF, 0x5B, 0xB5, 0xD6, 0x61, 0xF1, 0x9D, 0x8E, 0xA4, 0x20, 0x86, 0x32, 0xB1, 0xF9, 0x1E, 0x44,
  0x4A, 0x1C, 0xA3, 0x37, 0x93, 0x92, 0x33, 0x80, 0x9C, 0x8C, 0x64, 0x8E, 0xF3, 0xFE, 0x0A, 0xBF,
  0xE2, 0x1B, 0x6F, 0x1D, 0x7E, 0xCB, 0x1F, 0x09, 0xF5, 0xCD, 0x12, 0xEF, 0x6D, 0x9E, 0xB1, 0xE2,
  0xED, 0x2E, 0xF6, 0xC6, 0xEF, 0x68, 0x6C, 0x24, 0xB6, 0x37, 0x4F, 0x1B, 0xE3, 0xA1, 0xE1, 0x81,
  0xC5, 0x7D, 0xB9, 0xF1, 0x5F, 0xE1, 0xED, 0x9F, 0xC5, 0x9F, 0x86, 0x3E, 0x2A, 0xF0, 0x5D, 0xFC,
  0x9E, 0x45, 0xA6, 0xBF, 0xA6, 0x5C, 0x69, 0xCF, 0x3E, 0xCD, 0xE6, 0x1F, 0x36, 0x32, 0x82, 0x40,
  0xB9, 0x19, 0x2A, 0x48, 0x60, 0x32, 0x39, 0x03, 0x9A, 0xF9, 0xAB, 0xC6, 0x3F, 0xB0, 0x1D, 0xFF,
  0x00, 0x8D, 0xFF, 0x00, 0x65, 0xCF, 0x86, 0xBF, 0x07, 0xF5, 0x0F, 0x89, 0x3B, 0xE5, 0xF0, 0x56,
  0xB1, 0x16, 0xA5, 0x1E, 0xBC, 0x74, 0x4C, 0x9B, 0xA8, 0x62, 0x17, 0x0B, 0x1C, 0x06, 0x1F, 0xB4,
  0x7C, 0x9B, 0x52, 0x75, 0x5D, 0xDB, 0xCF, 0x11, 0x8F, 0x97, 0x9E, 0x00, 0x3C, 0x5F, 0xF6, 0xCA,
  0xF8, 0x2F, 0xF1, 0xCF, 0xC2, 0x3F, 0x0C, 0xE3, 0xD7, 0xFE, 0x24, 0x7C, 0x59, 0xBE, 0xF8, 0xDB,
  0xF0, 0xAF, 0x4D, 0xBC, 0x86, 0x7F, 0x13, 0x78, 0x5F, 0x4A, 0xD2, 0x6D, 0xBC, 0x39, 0x71, 0x2D,
  0xA8, 0x90, 0x7C, 0xFB, 0xED, 0xC3, 0xF9, 0xA8, 0xAD, 0xB5, 0x8A, 0x92, 0x31, 0x80, 0xDC, 0x60,
  0xB2, 0xFD, 0xF7, 0xF0, 0x97, 0x54, 0xF0, 0xB6, 0xB5, 0xF0, 0xBF, 0xC2, 0x97, 0xBE, 0x08, 0x10,
  0x27, 0x83, 0xE6, 0xD3, 0x2D, 0xDB, 0x49, 0x8E, 0xD9, 0x76, 0xC7, 0x1D, 0xAF, 0x96, 0x04, 0x68,
  0x17, 0xF8, 0x76, 0xA8, 0x0A, 0x41, 0xE4, 0x10, 0x41, 0xE4, 0x56, 0xFE, 0xBB, 0xA2, 0x58, 0x78,
  0x9B, 0x44, 0xD4, 0x34, 0x7D, 0x52, 0xD6, 0x3B, 0xED, 0x33, 0x50, 0xB7, 0x92, 0xD2, 0xEA, 0xDA,
  0x51, 0x94, 0x9A, 0x27, 0x52, 0xAE, 0x8C, 0x3D, 0x0A, 0x92, 0x3F, 0x1A, 0xF9, 0xBF, 0xE1, 0x27,
  0xEC, 0x9B, 0xE3, 0xCF, 0x81, 0x9F, 0x02, 0x35, 0xAF, 0x86, 0xDE, 0x0B, 0xF8, 0xCD, 0x26, 0x9A,
  0xC7, 0x53, 0x37, 0x9A, 0x06, 0xB5, 0x71, 0xE1, 0xB8, 0xAE, 0xA5, 0xD2, 0x2D, 0xDA, 0x40, 0xF2,
  0x5B, 0x18, 0xA5, 0x99, 0x92, 0x60, 0xDF, 0x37, 0xCD, 0x84, 0xC1, 0x77, 0x20, 0x72, 0x00, 0x00,
  0xF3, 0x1F, 0x82, 0xDF, 0xF2, 0x96, 0xFF, 0x00, 0xDA, 0x03, 0xFE, 0xC5, 0x5D, 0x3F, 0xFF, 0x00,
  0x49, 0xF4, 0xDA, 0xFB, 0xB2, 0xBE, 0x1A, 0xD1, 0x7F, 0x60, 0x6F, 0x8C, 0xFE, 0x1D, 0xF8, 0xB9,
  0xE2, 0x0F, 0x89, 0xDA, 0x7F, 0xED, 0x3D, 0xF6, 0x7F, 0x1C, 0xEB, 0xF6, 0x91, 0xD8, 0xEA, 0x5A,
  0xAF, 0xFC, 0x20, 0x16, 0x4D, 0xE7, 0xC2, 0x8B, 0x12, 0xA2, 0xF9, 0x4D, 0x39, 0x8D, 0x70, 0x21,
  0x88, 0x65, 0x54, 0x1F, 0x97, 0x93, 0xC9, 0xCF, 0xD9, 0xDE, 0x10, 0xD3, 0x75, 0x5D, 0x1B, 0xC2,
  0x9A, 0x35, 0x86, 0xBB, 0xAC, 0xFF, 0x00, 0xC2, 0x45, 0xAD, 0xDA, 0xD9, 0xC5, 0x0D, 0xF6, 0xAF,
  0xF6, 0x54, 0xB5, 0xFB, 0x6C, 0xEA, 0x80, 0x49, 0x37, 0x92, 0x84, 0xAC, 0x7B, 0xD8, 0x16, 0xD8,
  0xBC, 0x0C, 0xE0, 0x74, 0xA0, 0x0D, 0x7A, 0x28, 0xA8, 0x6E, 0xE3, 0x96, 0x6B, 0x59, 0xA3, 0x82,
  0x6F, 0xB3, 0xCC, 0xE8, 0xCA, 0x93, 0x6D, 0x0D, 0xB1, 0x88, 0xE1, 0xB0, 0x78, 0x38, 0x3C, 0xE2,
  0x80, 0x3E, 0x19, 0xFD, 0xBF, 0xBF, 0xE4, 0xEC, 0x3F, 0x63, 0x5F, 0xFB, 0x1A, 0xAE, 0xBF, 0xF4,
  0x7E, 0x9D, 0x5F, 0x76, 0x57, 0xC3, 0x5F, 0x10, 0x3F, 0x60, 0x6F, 0x8C, 0xFF, 0x00, 0x14, 0xBC,
  0x59, 0xE0, 0xDF, 0x13, 0x78, 0x9F, 0xF6, 0x9E, 0xFE, 0xD3, 0xD6, 0xFC, 0x1F, 0x76, 0xD7, 0xDA,
  0x1D, 0xD7, 0xFC, 0x20, 0x16, 0x51, 0x7D, 0x92, 0x66, 0x68, 0xD9, 0x9B, 0x6C, 0x73, 0xAA, 0xC9,
  0x93, 0x0C, 0x7C, 0x38, 0x61, 0xF2, 0xF4, 0xE4, 0xE7, 0xEA, 0x3F, 0x82, 0x9E, 0x0B, 0xF1, 0xE7,
  0x81, 0xBC, 0x33, 0x79, 0x63, 0xF1, 0x0B, 0xE2, 0x3F, 0xFC, 0x2C, 0xED, 0x62, 0x5B, 0xC6, 0x9A,
  0x1D, 0x57, 0xFB, 0x0A, 0xDF, 0x49, 0xF2, 0x60, 0x28, 0x80, 0x43, 0xE5, 0x42, 0x4A, 0xB6, 0x19,
  0x5D, 0xB7, 0x9E, 0x4E, 0xFC, 0x74, 0x02, 0x80, 0x3F, 0x3D, 0x3F, 0x62, 0xDF, 0x86, 0x3F, 0x16,
  0x3C, 0x7B, 0xE2, 0xEF, 0x8F, 0x73, 0xFC, 0x3C, 0xF8, 0xD2, 0x7E, 0x17, 0xD8, 0x43, 0xE3, 0x8B,
  0xC8, 0xEE, 0xEC, 0x87, 0x85, 0xAD, 0x35, 0x63, 0x75, 0x21, 0x91, 0xC8, 0x93, 0xCC, 0x99, 0x81,
  0x4C, 0x0E, 0x30, 0x38, 0xEF, 0x5F, 0x63, 0xFE, 0xCE, 0x7F, 0xB1, 0xD6, 0x97, 0xF0, 0x37, 0xC6,
  0x9E, 0x22, 0xF1, 0xF6, 0xB7, 0xE2, 0xAD, 0x57, 0xE2, 0x27, 0xC4, 0xAD, 0x7E, 0x31, 0x05, 0xF7,
  0x89, 0xB5, 0x85, 0x58, 0xD8, 0x45, 0x95, 0x3E, 0x5C, 0x51, 0x29, 0x22, 0x35, 0x3B, 0x13, 0x8C,
  0x9C, 0x04, 0x50, 0x36, 0x81, 0x8A, 0xD0, 0xFD, 0x97, 0xBF, 0x65, 0xFF, 0x00, 0xF8, 0x66, 0xDB,
  0xCF, 0x88, 0xD3, 0xFF, 0x00, 0xC2, 0x4B, 0xFF, 0x00, 0x09, 0x17, 0xFC, 0x26, 0x1A, 0xFC, 0xBA,
  0xE6, 0xDF, 0xB0, 0x7D, 0x97, 0xEC, 0x9B, 0xCB, 0x1F, 0x2B, 0xFD, 0x6B, 0xEF, 0xC6, 0xEF, 0xBD,
  0xF2, 0xF4, 0xE9, 0x5E, 0xED, 0x40, 0x1F, 0x99, 0x1E, 0x08, 0xF0, 0x1F, 0xC4, 0x6F, 0x1D, 0xFF,
  0x00, 0xC1, 0x43, 0xFF, 0x00, 0x68, 0xE8, 0x3E, 0x1D, 0x7C, 0x52, 0xFF, 0x00, 0x85, 0x5D, 0x79,
  0x6E, 0x2C, 0x9E, 0xEE, 0xEB, 0xFE, 0x11, 0xEB, 0x6D, 0x5F, 0xED, 0x68, 0x63, 0x8C, 0x2A, 0x6D,
  0x9D, 0x80, 0x4C, 0x1E, 0x72, 0x39, 0x39, 0xAF, 0xA7, 0x3E, 0x0F, 0x7E, 0xC5, 0x0B, 0xE0, 0xFF,
  0x00, 0x8C, 0x2B, 0xF1, 0x67, 0xE2, 0x2F, 0xC4, 0x0D, 0x5B, 0xE2, 0xC7, 0xC4, 0x48, 0x2D, 0xDA,
  0xD6, 0xC7, 0x51, 0xD4, 0x6D, 0x63, 0xB3, 0xB6, 0xB0, 0x8C, 0x86, 0x53, 0xE4, 0xDB, 0xC6, 0x48,
  0x43, 0x87, 0x71, 0xC1, 0xDA, 0x37, 0xB1, 0xDB, 0x93, 0x9A, 0xE9, 0x7E, 0x16, 0x7E, 0xCB, 0xFF,
  0x00, 0xF0, 0xAD, 0x3F, 0x69, 0x1F, 0x8A, 0x3F, 0x16, 0x3F, 0xE1, 0x25, 0xFE, 0xD2, 0xFF, 0x00,
  0x84, 0xDE, 0x3B, 0x74, 0xFE, 0xC8, 0xFB, 0x07, 0x95, 0xF6, 0x2F, 0x28, 0x28, 0xCF, 0x9D, 0xE6,
  0xB7, 0x99, 0x9D, 0xBF, 0xDC, 0x5C, 0x67, 0xBD, 0x7B, 0xB5, 0x00, 0x14, 0x51, 0x45, 0x00, 0x15,
  0xF0, 0x9F, 0xFC, 0x11, 0xBF, 0xFE, 0x4D, 0x5F, 0x5B, 0xFF, 0x00, 0xB1, 0xB6, 0xF7, 0xFF, 0x00,
  0x44, 0x5A, 0xD7, 0xDD, 0x95, 0xE1, 0x3F, 0xB1, 0xBF, 0xEC, 0xBF, 0xFF, 0x00, 0x0C, 0x95, 0xF0,
  0xAE, 0xFB, 0xC1, 0x9F, 0xF0, 0x93, 0x7F, 0xC2, 0x55, 0xF6, 0x9D, 0x5E, 0x7D, 0x57, 0xED, 0xBF,
  0x60, 0xFB, 0x1E, 0xDF, 0x32, 0x38, 0x93, 0xCB, 0xD9, 0xE6, 0xC9, 0x9C, 0x79, 0x59, 0xDD, 0xBB,
  0x9D, 0xDD, 0x38, 0xE4, 0x03, 0xE6, 0x9F, 0xF8, 0x28, 0x0F, 0x80, 0x34, 0x1F, 0x8A, 0x7F, 0xB6,
  0x6F, 0xEC, 0xCD, 0xE1, 0x3F, 0x14, 0x58, 0x7F, 0x69, 0xE8, 0x1A, 0xA9, 0xBF, 0x82, 0xF2, 0xCF,
  0xCE, 0x92, 0x1F, 0x35, 0x33, 0x19, 0xC6, 0xF8, 0xD9, 0x59, 0x79, 0x03, 0x90, 0x41, 0xAF, 0x6F,
  0xD1, 0x3F, 0xE0, 0x9A, 0x3F, 0xB3, 0x77, 0x87, 0x75, 0x9B, 0x0D, 0x5B, 0x4E, 0xF8, 0x71, 0xF6,
  0x7D, 0x42, 0xC2, 0xE2, 0x3B, 0xAB, 0x79, 0xBF, 0xB7, 0x35, 0x26, 0xD9, 0x22, 0x30, 0x64, 0x6C,
  0x35, 0xC9, 0x07, 0x04, 0x03, 0x82, 0x08, 0xA9, 0xBF, 0x6A, 0x1F, 0xD9, 0x17, 0x5E, 0xF8, 0xF5,
  0xF1, 0x37, 0xE1, 0xF7, 0x8E, 0xFC, 0x2F, 0xF1, 0x27, 0xFE, 0x15, 0xE6, 0xBF, 0xE0, 0xC5, 0x9F,
  0xEC, 0x73, 0xFF, 0x00, 0x61, 0x47, 0xA9, 0xEE, 0x79, 0x0A, 0xFC, 0xFB, 0x64, 0x95, 0x14, 0x60,
  0x29, 0x18, 0x2A, 0xC0, 0xE7, 0xB6, 0x2B, 0x33, 0xFE, 0x19, 0xE3, 0xF6, 0x9D, 0xFF, 0x00, 0xA3,
  0xB9, 0xFF, 0x00, 0xCC, 0x6B, 0xA5, 0xFF, 0x00, 0xF1, 0xCA, 0x00, 0xF6, 0xAF, 0x8D, 0x9F, 0x01,
  0xBC, 0x0F, 0xFB, 0x43, 0xF8, 0x35, 0xBC, 0x31, 0xE3, 0xCD, 0x0E, 0x3D, 0x67, 0x4C, 0x12, 0x09,
  0xA1, 0x3B, 0xDA, 0x39, 0xAD, 0xA5, 0x00, 0x81, 0x24, 0x52, 0x29, 0x0C, 0x8D, 0x82, 0x47, 0x07,
  0x04, 0x12, 0x08, 0x20, 0xE2, 0xBE, 0x25, 0xF8, 0xEB, 0xFB, 0x34, 0xF8, 0xD7, 0xF6, 0x1A, 0xF8,
  0x63, 0xA9, 0xFC, 0x4E, 0xF8, 0x37, 0xF1, 0x97, 0xC5, 0x6B, 0xA5, 0x78, 0x6C, 0xC3, 0x35, 0xC7,
  0x84, 0x3C, 0x51, 0x72, 0xB7, 0xD6, 0x13, 0xC0, 0xD2, 0xA4, 0x65, 0x11, 0x70, 0xAA, 0xB8, 0xDE,
  0x3F, 0x87, 0x76, 0x33, 0x87, 0x07, 0x06, 0xBE, 0xB3, 0xFD, 0xA1, 0x3F, 0x67, 0xBD, 0x53, 0xE3,
  0x54, 0x9E, 0x18, 0xD4, 0x74, 0x2F, 0x89, 0x1E, 0x21, 0xF8, 0x71, 0xE2, 0x3F, 0x0F, 0x1B, 0x83,
  0x6B, 0xA9, 0x68, 0x65, 0x4A, 0x4C, 0x26, 0x11, 0xEF, 0x59, 0xE2, 0x38, 0xF3, 0x17, 0xF7, 0x4A,
  0x42, 0xEE, 0x03, 0xAE, 0x41, 0xE3, 0x1E, 0x47, 0xE2, 0x1F, 0xD8, 0x53, 0xC7, 0x5F, 0x17, 0x45,
  0xAE, 0x97, 0xF1, 0x8F, 0xF6, 0x84, 0xD7, 0xFC, 0x7F, 0xE0, 0xE8, 0x67, 0x49, 0xE4, 0xF0, 0xF6,
  0x9F, 0xA2, 0x5B, 0x68, 0xB1, 0xDD, 0x94, 0x21, 0x95, 0x67, 0x78, 0x59, 0x8B, 0xAE, 0x40, 0x3C,
  0x8C, 0xF7, 0x04, 0x1C, 0x10, 0x01, 0xF4, 0xDF, 0xC2, 0xCF, 0x1A, 0xFF, 0x00, 0xC2, 0xC9, 0xF8,
  0x63, 0xE1, 0x0F, 0x17, 0x7D, 0x98, 0xD9, 0x7F, 0x6F, 0xE8, 0xF6, 0x7A, 0xAF, 0xD9, 0x89, 0xC9,
  0x8B, 0xCF, 0x85, 0x25, 0xD9, 0x9F, 0x6D, 0xF8, 0xFC, 0x2B, 0xE3, 0xBF, 0x83, 0x7F, 0xF2, 0x97,
  0x2F, 0x8F, 0x5F, 0xF6, 0x29, 0x58, 0x7F, 0xE8, 0x9D, 0x32, 0xBE, 0xE3, 0xD3, 0x34, 0xDB, 0x5D,
  0x1B, 0x4D, 0xB4, 0xD3, 0xEC, 0x6D, 0xE3, 0xB4, 0xB2, 0xB4, 0x89, 0x20, 0x82, 0xDE, 0x25, 0xDA,
  0x91, 0x46, 0xA0, 0x2A, 0xAA, 0x8E, 0xC0, 0x00, 0x00, 0x1E, 0xD5, 0xE2, 0x5E, 0x0D, 0xFD, 0x97,
  0xFF, 0x00, 0xE1, 0x12, 0xFD, 0xAE, 0x7C, 0x7B, 0xF1, 0xC3, 0xFE, 0x12, 0x6F, 0xB5, 0xFF, 0x00,
  0xC2, 0x55, 0xA4, 0x41, 0xA5, 0x7F, 0x61, 0x7D, 0x83, 0x67, 0xD9, 0x7C, 0xB4, 0xB6, 0x5F, 0x33,
  0xCF, 0xF3, 0x4E, 0xFC, 0xFD, 0x9B, 0x3B, 0x7C, 0xB5, 0xC6, 0xFE, 0xA7, 0x1C, 0x80, 0x7B, 0xB5,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05,
  0x14, 0x51, 0x40, 0x05, 0x7C, 0x95, 0xF1, 0x1F, 0xF6, 0xD3, 0xF1, 0xD6, 0x8D, 0xFB, 0x4B, 0xF8,
  0x83, 0xE0, 0xCF, 0xC3, 0xEF, 0x82, 0xBF, 0xF0, 0xB1, 0x75, 0xAD, 0x1F, 0x4C, 0x87, 0x56, 0x96,
  0xE3, 0xFE, 0x12, 0xA8, 0x34, 0xBD, 0xD0, 0x3A, 0xC4, 0x59, 0xB6, 0xCD, 0x09, 0x5F, 0x95, 0xA7,
  0x45, 0xC0, 0x72, 0x4E, 0x73, 0x8E, 0xB8, 0xFA, 0xD6, 0xBF, 0x34, 0xFC, 0x49, 0xE3, 0x2F, 0x1D,
  0x78, 0x1F, 0xFE, 0x0A, 0xB3, 0xF1, 0x22, 0xFF, 0x00, 0xE1, 0xF7, 0xC3, 0xBF, 0xF8, 0x59, 0x9A,
  0xD4, 0x9E, 0x0D, 0xB6, 0x86, 0x5D, 0x23, 0xFB, 0x6E, 0x0D, 0x27, 0xCA, 0x80, 0xAD, 0x89, 0x69,
  0xFC, 0xE9, 0x81, 0x56, 0xC3, 0x2A, 0x2E, 0xC1, 0xC9, 0xF3, 0x33, 0xFC, 0x26, 0x80, 0x3E, 0x8B,
  0xF0, 0x0F, 0xED, 0xA9, 0xA8, 0xB7, 0xC5, 0xDD, 0x0F, 0xE1, 0x9F, 0xC5, 0xAF, 0x85, 0xDA, 0xAF,
  0xC2, 0x3F, 0x15, 0x78, 0x81, 0x19, 0xB4, 0x66, 0xB8, 0xD4, 0xA0, 0xD4, 0xEC, 0x6F, 0x99, 0x7A,
  0xC6, 0xB7, 0x31, 0x00, 0xA1, 0xFD, 0x06, 0x0F, 0x24, 0x02, 0x41, 0x65, 0x07, 0xEA, 0x1A, 0xFC,
  0xD9, 0xF1, 0x07, 0x8A, 0xFC, 0x53, 0xF1, 0x8F, 0xF6, 0xDD, 0xF8, 0x2E, 0x7E, 0x3D, 0x78, 0x5D,
  0xFE, 0x09, 0xC3, 0xE1, 0xF9, 0xA6, 0xB8, 0xF0, 0xA6, 0x92, 0x66, 0x5D, 0x4A, 0x3D, 0x66, 0xFD,
  0x9A, 0x32, 0x63, 0x37, 0xF1, 0xE2, 0x20, 0x77, 0x47, 0x07, 0xC9, 0xB7, 0x3C, 0x05, 0xEB, 0x20,
  0x35, 0xF4, 0x6F, 0xFC, 0x14, 0x8B, 0xC5, 0xBA, 0xE7, 0x81, 0x7F, 0x63, 0x4F, 0x1F, 0x6B, 0x7E,
  0x1B, 0xD6, 0xB5, 0x0F, 0x0F, 0x6B, 0x36, 0xCD, 0xA7, 0xF9, 0x1A, 0x8E, 0x95, 0x75, 0x25, 0xB5,
  0xC4, 0x5B, 0xB5, 0x0B, 0x74, 0x6D, 0xB2, 0x21, 0x0C, 0xB9, 0x56, 0x65, 0x38, 0x3C, 0x82, 0x47,
  0x43, 0x40, 0x1F, 0x4C, 0xD1, 0x5F, 0x1A, 0xF8, 0x37, 0xF6, 0x7B, 0xF8, 0xA1, 0xF1, 0xEF, 0xE1,
  0xC6, 0x81, 0xE3, 0x5F, 0x13, 0x7C, 0x72, 0xF1, 0xCF, 0x81, 0xF5, 0xAD, 0x47, 0x4C, 0xB7, 0xBA,
  0xD2, 0xF4, 0x6F, 0x0B, 0x5E, 0x2C, 0x16, 0x7A, 0x7C, 0x4D, 0x12, 0x98, 0x85, 0xD0, 0x2A, 0x5E,
  0xEE, 0x52, 0xA4, 0x3C, 0x8C, 0xCC, 0xB9, 0x66, 0x60, 0x30, 0x00, 0xA9, 0x7F, 0x6B, 0xCF, 0x8B,
  0xFF, 0x00, 0x12, 0x3E, 0x19, 0x78, 0x4F, 0xE0, 0xBF, 0xC2, 0x8F, 0x0A, 0xEB, 0xD0, 0xC5, 0xF1,
  0x43, 0xE2, 0x05, 0xDC, 0x1A, 0x2D, 0xC7, 0x8A, 0x4C, 0x0A, 0xC2, 0x01, 0x12, 0x42, 0x97, 0x77,
  0x48, 0x84, 0x60, 0x33, 0x3C, 0xAA, 0xC3, 0x8E, 0x17, 0x7E, 0x00, 0x6C, 0x10, 0x01, 0xF6, 0x2D,
  0x15, 0xF0, 0x07, 0xED, 0x09, 0xF0, 0xC3, 0xE2, 0xA7, 0xEC, 0x69, 0xF0, 0xD5, 0xBE, 0x2E, 0xF8,
  0x3B, 0xE3, 0x7F, 0x8E, 0x3C, 0x73, 0x36, 0x81, 0x3D, 0xBC, 0x9A, 0xE6, 0x81, 0xE3, 0x5B, 0xE1,
  0x7D, 0x63, 0x7F, 0x6F, 0x24, 0xA9, 0x13, 0xF9, 0x49, 0xB4, 0x79, 0x3F, 0x33, 0x83, 0xC1, 0x24,
  0x2E, 0x48, 0x60, 0x47, 0x3F, 0x4F, 0xF8, 0xD3, 0xC0, 0xF6, 0x9F, 0xB5, 0x47, 0xC2, 0x8F, 0x08,
  0xEA, 0x36, 0x1E, 0x37, 0xF1, 0xAF, 0x80, 0x6C, 0x75, 0x08, 0x6D, 0xB5, 0xCB, 0x7B, 0xCF, 0x06,
  0x6A, 0xFF, 0x00, 0xD9, 0xD7, 0x72, 0xC7, 0x2C, 0x1B, 0x96, 0x29, 0x5F, 0x63, 0x6E, 0x4C, 0x48,
  0x09, 0x5C, 0x7D, 0xE5, 0x07, 0xB5, 0x00, 0x5F, 0xF8, 0x29, 0xFB, 0x43, 0x78, 0x73, 0xE3, 0xC6,
  0xA3, 0xE3, 0xAB, 0x3D, 0x02, 0xCB, 0x54, 0xB3, 0x97, 0xC1, 0xDA, 0xE4, 0xFE, 0x1F, 0xBF, 0x3A,
  0x94, 0x51, 0xA2, 0xCB, 0x71, 0x11, 0x21, 0x9A, 0x2D, 0x92, 0x3E, 0x50, 0xE3, 0x82, 0xDB, 0x4F,
  0xB0, 0xAE, 0x53, 0xF6, 0xE1, 0xF8, 0xF9, 0xE2, 0x0F, 0xD9, 0xA3, 0xF6, 0x7A, 0xD6, 0x3C, 0x77,
  0xE1, 0x8B, 0x3D, 0x32, 0xFF, 0x00, 0x57, 0xB3, 0xBB, 0xB5, 0x82, 0x38, 0x75, 0x78, 0xA4, 0x92,
  0xDC, 0xAC, 0xB2, 0xAA, 0x31, 0x2B, 0x1C, 0x88, 0xD9, 0xC1, 0xE3, 0xE6, 0xEB, 0xEB, 0x5F, 0x18,
  0x7E, 0xC5, 0x1F, 0xB2, 0xA7, 0xFC, 0x2C, 0x6F, 0x10, 0xFC, 0x74, 0x83, 0xFE, 0x17, 0x0F, 0xC5,
  0x7F, 0x0B, 0xFF, 0x00, 0x61, 0x78, 0xE6, 0xF7, 0x4C, 0xF3, 0x7C, 0x37, 0xE2, 0x7F, 0xB2, 0x3E,
  0xA1, 0xB1, 0xD8, 0x7D, 0xA2, 0xEC, 0xF9, 0x4D, 0xE6, 0xCE, 0xD8, 0xF9, 0x9F, 0x8C, 0xE4, 0xF1,
  0x5E, 0xDB, 0xFF, 0x00, 0x05, 0x37, 0xF0, 0xE7, 0xFC, 0x22, 0x1F, 0xF0, 0x4F, 0xFB, 0xDD, 0x0F,
  0xFB, 0x53, 0x52, 0xD6, 0xFF, 0x00, 0xB3, 0x65, 0xD2, 0xAD, 0x3F, 0xB4, 0xB5, 0x8B, 0x8F, 0xB4,
  0x5E, 0x5D, 0x6C, 0x9A, 0x35, 0xF3, 0x67, 0x93, 0x03, 0x7C, 0x8D, 0x8C, 0xB3, 0x60, 0x64, 0x92,
  0x71, 0x40, 0x1F, 0x6A, 0x69, 0x57, 0x4F, 0x7D, 0xA5, 0xD9, 0xDC, 0xC8, 0x14, 0x3C, 0xD0, 0xA4,
  0x8C, 0x17, 0xA0, 0x25, 0x41, 0x38, 0xFC, 0xEA, 0xD5, 0x7C, 0x07, 0xF1, 0xAB, 0xE0, 0x97, 0xC5,
  0xFF, 0x00, 0x0C, 0x7E, 0xCF, 0x7A, 0x8F, 0xC6, 0x05, 0xF8, 0xF3, 0xE3, 0x2D, 0x3B, 0xC7, 0xFA,
  0x2E, 0x90, 0x35, 0xC9, 0x74, 0x2D, 0x3E, 0xE1, 0x21, 0xF0, 0xFC, 0x49, 0x1A, 0x09, 0x1E, 0xD5,
  0x2D, 0x02, 0xFC, 0xC1, 0x10, 0x15, 0x0F, 0x23, 0x3E, 0xF2, 0xB9, 0x61, 0xC9, 0xAF, 0x63, 0xF1,
  0x27, 0xED, 0x73, 0x2F, 0x83, 0xBF, 0x61, 0x2B, 0x0F, 0x8E, 0x77, 0xB6, 0x10, 0xDC, 0x6B, 0x17,
  0x1E, 0x1F, 0xB3, 0xBB, 0x5B, 0x01, 0x95, 0x8A, 0x4B, 0xF9, 0xF6, 0x44, 0x17, 0x83, 0x9F, 0x2C,
  0x4A, 0xF9, 0x23, 0x39, 0xDA, 0x0F, 0x39, 0xA0, 0x0F, 0xA6, 0x68, 0xAF, 0x87, 0xBE, 0x1D, 0xFE,
  0xCB, 0xBF, 0x18, 0xFE, 0x2C, 0x7C, 0x32, 0xD3, 0x3C, 0x79, 0xE2, 0xCF, 0xDA, 0x23, 0xC7, 0x9E,
  0x1F, 0xF8, 0x85, 0xAD, 0x59, 0xA6, 0xA7, 0x6D, 0xA7, 0x68, 0x77, 0x49, 0x6F, 0xA3, 0x69, 0xE6,
  0x45, 0x0F, 0x14, 0x32, 0x5A, 0x2A, 0xE2, 0x5D, 0xA0, 0xA8, 0x6E, 0x57, 0x9C, 0x8E, 0x71, 0x93,
  0xEB, 0x9F, 0xB0, 0xDF, 0xC7, 0xAD, 0x73, 0xE3, 0xE7, 0xC1, 0x79, 0xAE, 0xFC, 0x5B, 0x1C, 0x29,
  0xE3, 0x4F, 0x0E, 0x6A, 0xD7, 0x3E, 0x1C, 0xD7, 0x1A, 0xDD, 0x42, 0xC7, 0x35, 0xD4, 0x1B, 0x49,
  0x95, 0x54, 0x70, 0x37, 0x2B, 0xA1, 0x20, 0x00, 0x37, 0x6E, 0xC0, 0x03, 0x00, 0x00, 0x7D, 0x0B,
  0x45, 0x7E, 0x7B, 0x7F, 0x6F, 0x7C, 0x59, 0xF8, 0x91, 0xFF, 0x00, 0x05, 0x08, 0xF8, 0xDF, 0xF0,
  0xC7, 0xC3, 0x5E, 0x3E, 0xD6, 0x3C, 0x37, 0xE1, 0xB1, 0x61, 0xA6, 0x5C, 0x4D, 0x7D, 0xF6, 0x86,
  0xB8, 0x5D, 0x22, 0xD8, 0x5A, 0x5B, 0x34, 0x8B, 0x63, 0x04, 0x84, 0xC7, 0x14, 0xF3, 0x49, 0x28,
  0x1E, 0x66, 0xDF, 0x94, 0x79, 0xAD, 0x82, 0xD8, 0xAF, 0xA6, 0x3E, 0x0A, 0x7E, 0xCF, 0x7E, 0x28,
  0xF8, 0x31, 0xE3, 0x9B, 0xEB, 0xC9, 0x3E, 0x2E, 0xF8, 0xC3, 0xC7, 0xFE, 0x14, 0xBD, 0xD3, 0xDA,
  0x26, 0xD2, 0xFC, 0x67, 0x78, 0x2F, 0x67, 0xB7, 0xBB, 0x12, 0x23, 0x2C, 0xD1, 0x4C, 0x15, 0x70,
  0x85, 0x3C, 0xC5, 0x29, 0x81, 0x8C, 0xAF, 0x5F, 0xE1, 0x00, 0xF7, 0x1A, 0x28, 0xA2, 0x80, 0x3E,
  0x2B, 0xF0, 0xF7, 0xED, 0xE5, 0xF1, 0x37, 0xE2, 0x0F, 0x8E, 0x7E, 0x21, 0x68, 0x9E, 0x04, 0xFD,
  0x9D, 0xE7, 0xF1, 0x86, 0x9F, 0xE0, 0xAD, 0x6A, 0xE3, 0x46, 0xBF, 0xD4, 0x2D, 0xBC, 0x61, 0x6D,
  0x6F, 0x23, 0x34, 0x72, 0xC8, 0x8A, 0xE2, 0x09, 0x61, 0x56, 0x25, 0x84, 0x65, 0xB6, 0xA9, 0x6C,
  0x74, 0xCD, 0x7B, 0x47, 0xEC, 0xEB, 0xFB, 0x57, 0x78, 0x67, 0xF6, 0x86, 0xB8, 0xD7, 0x74, 0x68,
  0x34, 0xDD, 0x53, 0xC2, 0x3E, 0x36, 0xF0, 0xFB, 0x88, 0xF5, 0x8F, 0x0A, 0x6B, 0xF0, 0xF9, 0x37,
  0xB6, 0x99, 0xE0, 0x38, 0x19, 0xC3, 0xA1, 0x3C, 0x6E, 0x1C, 0x8C, 0x8C, 0x81, 0xB9, 0x73, 0xF3,
  0xF7, 0xFC, 0x13, 0x5F, 0xFE, 0x4A, 0xCF, 0xED, 0x5B, 0xFF, 0x00, 0x63, 0xF5, 0xC7, 0xFE, 0x8F,
  0xBA, 0xAA, 0xDF, 0x12, 0x5B, 0xFE, 0x10, 0xDF, 0xF8, 0x2B, 0xF7, 0xC2, 0xC9, 0xF4, 0xC1, 0xE4,
  0x7F, 0xC2, 0x55, 0xE0, 0xFB, 0x8B, 0x7D, 0x51, 0x63, 0xE0, 0xCE, 0x12, 0x3B, 0xD2, 0xAC, 0xDE,
  0xB8, 0xFB, 0x34, 0x1D, 0x7F, 0xE7, 0x90, 0xF4, 0xA0, 0x0F, 0xA3, 0x3F, 0x68, 0xDF, 0xDA, 0xA7,
  0xC3, 0x7F, 0xB3, 0xB2, 0xE8, 0x5A, 0x6D, 0xC6, 0x9B, 0xA9, 0x78, 0xAF, 0xC6, 0x7E, 0x21, 0x94,
  0xC1, 0xA2, 0xF8, 0x57, 0x43, 0x88, 0x4B, 0x79, 0x7A, 0xC0, 0x80, 0x5B, 0x1D, 0x11, 0x01, 0x20,
  0x16, 0x3E, 0xF8, 0x07, 0x07, 0x1C, 0x5F, 0x85, 0xBF, 0x6A, 0xAF, 0x8A, 0x09, 0xE3, 0x4F, 0x0E,
  0x69, 0x1E, 0x3F, 0xFD, 0x9D, 0x3C, 0x41, 0xE0, 0xAD, 0x23, 0x5D, 0xBD, 0x8A, 0xC6, 0x0D, 0x6E,
  0xD3, 0x5A, 0xB6, 0xD5, 0x61, 0xB7, 0x92, 0x42, 0x15, 0x0D, 0xCA, 0xC4, 0xAB, 0xE4, 0xA9, 0x24,
  0x0C, 0xB1, 0xEA, 0x40, 0x00, 0x9E, 0x2B, 0xCA, 0xBE, 0x1F, 0xE3, 0xC7, 0x1F, 0xF0, 0x57, 0xCF,
  0x89, 0x73, 0x6A, 0xA3, 0xED, 0x1F, 0xF0, 0x87, 0x78, 0x3E, 0xDE, 0x0D, 0x21, 0x64, 0xE4, 0xC1,
  0xE6, 0x47, 0x66, 0xCC, 0xCB, 0xE9, 0xFF, 0x00, 0x1F, 0x77, 0x03, 0x8F, 0xF9, 0xE8, 0x6B, 0xEF,
  0x3A, 0x00, 0xF9, 0xA3, 0xF6, 0xA1, 0xFD, 0xAE, 0xB5, 0xEF, 0x80, 0xBF, 0x13, 0x7E, 0x1F, 0x78,
  0x13, 0xC2, 0xFF, 0x00, 0x0D, 0xBF, 0xE1, 0x61, 0xEB, 0xFE, 0x33, 0x59, 0xFE, 0xC7, 0x07, 0xF6,
  0xEC, 0x7A, 0x66, 0xD7, 0x8C, 0xAF, 0xC9, 0xBA, 0x48, 0x9D, 0x4E, 0x43, 0x13, 0x92, 0xCA, 0x06,
  0x3B, 0xE6, 0xB9, 0x3D, 0x5B, 0xF6, 0xF5, 0xF1, 0x47, 0xC2, 0x2D, 0x73, 0x44, 0x87, 0xE3, 0xA7,
  0xC0, 0xFD, 0x5F, 0xE1, 0x5E, 0x81, 0xAB, 0xDC, 0xAD, 0x9C, 0x3E, 0x22, 0xB7, 0xD6, 0xED, 0xB5,
  0xAB, 0x38, 0x65, 0x3C, 0x81, 0x33, 0x42, 0xAB, 0xB0, 0x60, 0x13, 0xD4, 0xB6, 0x15, 0x88, 0x53,
  0x83, 0x5E, 0x6D, 0xFF, 0x00, 0x05, 0x01, 0xF1, 0xFE, 0x83, 0xF0, 0xB3, 0xF6, 0xCD, 0xFD, 0x99,
  0xBC, 0x59, 0xE2, 0x8B, 0xFF, 0x00, 0xEC, 0xCD, 0x03, 0x4A, 0x37, 0xF3, 0xDE, 0x5E, 0x79, 0x32,
  0x4D, 0xE5, 0x26, 0x63, 0x19, 0xD9, 0x1A, 0xB3, 0x37, 0x24, 0x70, 0x01, 0x35, 0x81, 0xFB, 0x67,
  0xFE, 0xD3, 0x5E, 0x17, 0xFD, 0xB6, 0x7E, 0x19, 0xD9, 0x7C, 0x20, 0xF8, 0x17, 0x65, 0xAA, 0x7C,
  0x44, 0xD7, 0xF5, 0xAD, 0x52, 0xDA, 0x49, 0xAF, 0xA1, 0xD2, 0xAE, 0x6D, 0xAD, 0x34, 0xE8, 0xA3,
  0x7D, 0xC6, 0x49, 0x24, 0x9A, 0x34, 0xDB, 0xCE, 0x01, 0x38, 0xDA, 0x14, 0xB9, 0x24, 0x70, 0x08,
  0x07, 0xE9, 0x4A, 0x3A, 0xCA, 0x8A, 0xE8, 0xC1, 0xD1, 0x86, 0x55, 0x94, 0xE4, 0x11, 0xEA, 0x29,
  0xD5, 0xC8, 0xC5, 0xA9, 0xE8, 0x9F, 0x07, 0x7E, 0x1B, 0x69, 0x8D, 0xE2, 0x8F, 0x10, 0xE9, 0xFA,
  0x3E, 0x93, 0xA3, 0x59, 0x5B, 0x59, 0xDC, 0x6A, 0xFA, 0xB5, 0xD2, 0x5A, 0xDB, 0x82, 0xAA, 0x91,
  0x2B, 0x34, 0x92, 0x10, 0xAB, 0xB9, 0xB0, 0x06, 0x4F, 0x25, 0x80, 0xA8, 0x3C, 0x15, 0xF1, 0xBB,
  0xE1, 0xD7, 0xC4, 0xAD, 0x56, 0x5D, 0x33, 0xC2, 0x1E, 0x3E, 0xF0, 0xBF, 0x8A, 0xB5, 0x28, 0xA1,
  0x37, 0x32, 0x59, 0xE8, 0x9A, 0xCD, 0xB5, 0xE4, 0xC9, 0x10, 0x65, 0x53, 0x21, 0x48, 0xDD, 0x88,
  0x50, 0xCE, 0xA3, 0x71, 0x18, 0xCB, 0x01, 0xDC, 0x50, 0x07, 0x05, 0xF1, 0x8B, 0xE2, 0xDF, 0xC6,
  0x9F, 0x05, 0xF8, 0xC3, 0xFB, 0x3B, 0xC0, 0x7F, 0x00, 0xFF, 0x00, 0xE1, 0x63, 0xE8, 0x3F, 0x67,
  0x49, 0x3F, 0xB6, 0xBF, 0xE1, 0x32, 0xB2, 0xD2, 0xFF, 0x00, 0x7A, 0x73, 0xBA, 0x3F, 0x26, 0x64,
  0x2D, 0xF2, 0xE0, 0x7C, 0xD9, 0xC1, 0xCF, 0xB5, 0x78, 0x67, 0xC3, 0xBF, 0xDB, 0xE7, 0xE3, 0x3F,
  0xC5, 0x7F, 0x12, 0x78, 0xC7, 0x41, 0xF0, 0xAF, 0xEC, 0xC3, 0xFD, 0xA9, 0xAB, 0x78, 0x42, 0xFB,
  0xFB, 0x37, 0x5B, 0xB7, 0xFF, 0x00, 0x84, 0xFE, 0xCA, 0x1F, 0xB2, 0x5C, 0x6E, 0x91, 0x36, 0x6E,
  0x92, 0x05, 0x57, 0xF9, 0xA2, 0x90, 0x65, 0x0B, 0x0F, 0x97, 0xAF, 0x23, 0x3F, 0x72, 0xD7, 0xC2,
  0x7F, 0xF0, 0x4F, 0x0F, 0xF9, 0x38, 0xFF, 0x00, 0xDB, 0x13, 0xFE, 0xC7, 0x96, 0xFF, 0x00, 0xD2,
  0xAB, 0xFA, 0x00, 0xF7, 0xBD, 0x53, 0xE2, 0xD7, 0xC6, 0x8B, 0x4F, 0x84, 0x5A, 0x37, 0x88, 0x6C,
  0xBE, 0x02, 0x7D, 0xBF, 0xC7, 0x37, 0x57, 0xCF, 0x6F, 0x7D, 0xE0, 0xDF, 0xF8, 0x4C, 0x6C, 0xA3,
  0xFB, 0x15, 0xB8, 0xF3, 0x36, 0xDC, 0x7D, 0xB0, 0xA7, 0x97, 0x26, 0x76, 0xC7, 0xF2, 0x28, 0xC8,
  0xF3, 0x7F, 0xD9, 0x35, 0xE4, 0x3F, 0x11, 0xFF, 0x00, 0x6C, 0xFF, 0x00, 0x8F, 0x5F, 0x08, 0xFC,
  0x13, 0xAA, 0xF8, 0xBB, 0xC5, 0xDF, 0xB2, 0xD2, 0x68, 0xDE, 0x1D, 0xD2, 0xE3, 0x59, 0x2E, 0xEF,
  0x64, 0xF8, 0x8D, 0xA7, 0xC8, 0x23, 0x0C, 0xC1, 0x17, 0xE5, 0x48, 0x99, 0x98, 0x96, 0x65, 0x00,
  0x28, 0x24, 0x92, 0x38, 0xAF, 0xB3, 0xEB, 0xE1, 0x7F, 0xF8, 0x28, 0x45, 0xF4, 0xFF, 0x00, 0x1B,
  0x3E, 0x2A, 0xFC, 0x1C, 0xFD, 0x9B, 0x74, 0xB9, 0x9F, 0x1E, 0x26, 0xD4, 0xD7, 0x5C, 0xF1, 0x17,
  0x92, 0xD8, 0x68, 0xB4, 0xD8, 0x0B, 0x1C, 0x1F, 0x66, 0x09, 0x3B, 0x0C, 0xE3, 0xE6, 0x85, 0x3D,
  0x68, 0x03, 0xDE, 0xF4, 0xEF, 0xDA, 0x46, 0xE6, 0x3F, 0xD9, 0x63, 0x4E, 0xF8, 0xC9, 0xAC, 0xF8,
  0x23, 0x54, 0xB6, 0x9E, 0xF3, 0x4E, 0x83, 0x50, 0x5F, 0x0B, 0x69, 0x52, 0x0B, 0xDB, 0xC6, 0x13,
  0xCA, 0xA9, 0x02, 0x46, 0xC5, 0x63, 0xDE, 0x5C, 0x49, 0x1B, 0x7D, 0xD0, 0x40, 0x62, 0x30, 0x48,
  0xE7, 0xC9, 0xF5, 0x9F, 0xDB, 0x73, 0xE2, 0xC7, 0x83, 0xBC, 0x3B, 0x27, 0x8B, 0xBC, 0x53, 0xFB,
  0x2E, 0x78, 0x9F, 0x47, 0xF0, 0x2D, 0xBA, 0x79, 0xF7, 0x7A, 0x94, 0x5A, 0xED, 0xAC, 0xF7, 0xB6,
  0xD0, 0x0E, 0x5A, 0x47, 0xB2, 0xD8, 0xAE, 0xA1, 0x47, 0x27, 0x71, 0x50, 0x00, 0x39, 0x20, 0x0C,
  0xD7, 0xD7, 0xF6, 0x76, 0x70, 0x69, 0xF6, 0x90, 0x5A, 0xDB, 0x44, 0x96, 0xF6, 0xD0, 0x22, 0xC5,
  0x14, 0x51, 0xAE, 0xD5, 0x44, 0x51, 0x80, 0xA0, 0x76, 0x00, 0x00, 0x2B, 0xC0, 0xFF, 0x00, 0x6D,
  0xFF, 0x00, 0x8F, 0x3A, 0x3F, 0xC1, 0x1F, 0x81, 0x5A, 0xE4, 0x53, 0xA7, 0xF6, 0x97, 0x89, 0xBC,
  0x4D, 0x6D, 0x36, 0x89, 0xA0, 0x68, 0x70, 0xAF, 0x99, 0x3D, 0xFD, 0xD4, 0xC8, 0x63, 0x1B, 0x63,
  0x1C, 0xB2, 0xA6, 0xF0, 0xCD, 0xF8, 0x2F, 0xDE, 0x65, 0x04, 0x03, 0xD6, 0xBE, 0x1A, 0x7C, 0x4A,
  0xF0, 0xF7, 0xC5, 0xDF, 0x00, 0xE8, 0xDE, 0x33, 0xF0, 0xBD, 0xFA, 0xEA, 0x1E, 0x1F, 0xD5, 0xAD,
  0xFE, 0xD1, 0x6D, 0x71, 0x8D, 0xA7, 0x19, 0x21, 0x95, 0x81, 0xFB, 0xAC, 0xAC, 0x19, 0x58, 0x1E,
  0x85, 0x48, 0xED, 0x5F, 0x32, 0x9F, 0xDB, 0xE3, 0xC4, 0x5E, 0x3A, 0xD4, 0x3C, 0x47, 0x77, 0xF0,
  0x7F, 0xE0, 0x86, 0xBD, 0xF1, 0x4F, 0xC1, 0x7E, 0x1F, 0xB9, 0x7B, 0x4B, 0xDF, 0x13, 0x5B, 0xEA,
  0x90, 0xD9, 0x47, 0x34, 0x88, 0x32, 0xE2, 0xD2, 0x27, 0x56, 0x6B, 0x8C, 0x02, 0x08, 0x0B, 0xF3,
  0x10, 0x47, 0x03, 0x23, 0x35, 0xBC, 0x03, 0xF0, 0xC3, 0xC4, 0x1F, 0xB2, 0xD7, 0xFC, 0x13, 0x23,
  0xC4, 0x7A, 0x16, 0xA3, 0x21, 0x87, 0xC5, 0x3A, 0x7F, 0x84, 0x75, 0x8D, 0x42, 0xE1, 0x63, 0x7C,
  0xFD, 0x92, 0xE2, 0x68, 0xE6, 0x94, 0x20, 0x23, 0x8C, 0xC7, 0xBD, 0x41, 0x20, 0xE0, 0xB2, 0x92,
  0x0E, 0x0D, 0x72, 0xFF, 0x00, 0xB1, 0xAF, 0xC4, 0xAD, 0x7B, 0xE0, 0xE7, 0xEC, 0xCF, 0xFB, 0x39,
  0xF8, 0x73, 0xC3, 0x5F, 0x0B, 0x35, 0xBF, 0x1A, 0x69, 0xBE, 0x2D, 0x7B, 0x93, 0xAB, 0x6B, 0xBA,
  0x53, 0x01, 0x0E, 0x91, 0xBE, 0xF1, 0xC9, 0x9A, 0x7F, 0x91, 0xB3, 0xC3, 0x9F, 0xBE, 0xC8, 0x36,
  0xC7, 0x80, 0xC4, 0xF1, 0x40, 0x1F, 0x53, 0x7C, 0x09, 0xFD, 0xA0, 0xFC, 0x2D, 0xFB, 0x44, 0x7C,
  0x29, 0xB6, 0xF1, 0xE7, 0x84, 0xDE, 0x67, 0xB0, 0x7F, 0x32, 0x39, 0xEC, 0xEE, 0x94, 0x25, 0xC5,
  0xA5, 0xC4, 0x63, 0x2F, 0x04, 0xA0, 0x12, 0x03, 0x0C, 0x83, 0xC1, 0x20, 0x86, 0x52, 0x32, 0x08,
  0xAF, 0x99, 0x7E, 0x12, 0x7F, 0xC1, 0x41, 0xFE, 0x2B, 0x7C, 0x66, 0xF0, 0x53, 0xF8, 0xCF, 0xC3,
  0x3F, 0xB3, 0x1E, 0xA1, 0xAC, 0xF8, 0x4E, 0x2B, 0x89, 0x2D, 0xA5, 0xBD, 0xD2, 0xFC, 0x5F, 0x6D,
  0x35, 0xC0, 0x74, 0x00, 0xBA, 0xA5, 0xB3, 0x42, 0x92, 0x39, 0x01, 0x87, 0x41, 0xCE, 0x78, 0xAA,
  0x1F, 0xB0, 0x21, 0xFE, 0xC3, 0xF8, 0xDB, 0xFB, 0x62, 0x78, 0x56, 0xD3, 0xF7, 0x7A, 0x36, 0x9D,
  0xE2, 0xBF, 0xB5, 0x5A, 0xDB, 0xA7, 0x09, 0x13, 0x4D, 0x25, 0xE8, 0x60, 0x07, 0x6F, 0x96, 0x18,
  0xC7, 0xD1, 0x45, 0x6A, 0x7F, 0xC1, 0x1D, 0xBF, 0xE4, 0xD0, 0x4F, 0xFD, 0x8C, 0x57, 0xBF, 0xFA,
  0x0C, 0x34, 0x01, 0xEE, 0x1F, 0x0D, 0xBF, 0x6B, 0x5F, 0x0B, 0xFC, 0x65, 0xF8, 0x2D, 0xE2, 0xCF,
  0x1D, 0x78, 0x32, 0xCE, 0xEE, 0xEF, 0x52, 0xF0, 0xDD, 0xAD, 0xD3, 0x5F, 0xF8, 0x5F, 0x54, 0x1F,
  0x63, 0xBE, 0xB7, 0xBB, 0x86, 0x26, 0x71, 0x6D, 0x30, 0xC3, 0xEC, 0xDC, 0x57, 0x01, 0xC0, 0x61,
  0xD7, 0xA9, 0x56, 0x51, 0xD0, 0x7E, 0xCC, 0x5F, 0x1C, 0xE1, 0xFD, 0xA4, 0xBE, 0x07, 0x78, 0x67,
  0xE2, 0x2C, 0x3A, 0x4F, 0xF6, 0x10, 0xD6, 0x56, 0x7D, 0xDA, 0x6F, 0xDA, 0x7E, 0xD2, 0x6D, 0xDA,
  0x2B, 0x89, 0x21, 0x2B, 0xE6, 0x6C, 0x4D, 0xDC, 0xC7, 0x9F, 0xBA, 0x3A, 0xD7, 0xCA, 0xDF, 0x05,
  0x6D, 0xA2, 0xF0, 0xFF, 0x00, 0xFC, 0x15, 0x63, 0xE3, 0xFF, 0x00, 0x85, 0x2D, 0x63, 0x0B, 0xA1,
  0x6B, 0xDE, 0x18, 0x83, 0x54, 0xBC, 0xB4, 0x4E, 0x11, 0xA7, 0xD9, 0x65, 0xB9, 0xD8, 0x7A, 0x93,
  0x73, 0x39, 0xCF, 0xAC, 0x87, 0xD6, 0xBC, 0x87, 0xE0, 0xAF, 0xC5, 0x7F, 0x1E, 0x7C, 0x20, 0xFD,
  0x87, 0x22, 0xF0, 0x2F, 0xC3, 0xBD, 0x40, 0x58, 0x7C, 0x48, 0x3F, 0x14, 0xA6, 0xF0, 0x26, 0x97,
  0x33, 0xC1, 0x14, 0xC6, 0x29, 0x1A, 0x41, 0x3C, 0x8C, 0x52, 0x54, 0x64, 0xDB, 0x82, 0xC0, 0x92,
  0xA7, 0x1B, 0xF3, 0xC1, 0xC5, 0x00, 0x7D, 0xD3, 0xFB, 0x57, 0x7E, 0xD2, 0xED, 0xFB, 0x33, 0x78,
  0x63, 0xC2, 0x5A, 0x8D, 0xBF, 0x86, 0xBF, 0xE1, 0x2B, 0xD4, 0x3C, 0x47, 0xE2, 0x2B, 0x6F, 0x0F,
  0xDB, 0xE9, 0xFF, 0x00, 0x6F, 0xFB, 0x1E, 0xD6, 0x99, 0x24, 0x6F, 0x37, 0x7F, 0x97, 0x26, 0x70,
  0x51, 0x46, 0xDC, 0x73, 0xBC, 0x72, 0x31, 0x5E, 0xE1, 0x5F, 0x9A, 0x1E, 0x21, 0xF8, 0x9F, 0x7D,
  0xFB, 0x4F, 0x4F, 0xFB, 0x07, 0x9D, 0x4E, 0x71, 0x7D, 0xAB, 0xEA, 0x9A, 0xC5, 0xC6, 0xB1, 0xAB,
  0x91, 0x1A, 0xA7, 0x99, 0x73, 0xA6, 0x79, 0x62, 0x67, 0x64, 0x50, 0x15, 0x77, 0x3C, 0x73, 0x1D,
  0xA0, 0x00, 0x39, 0x00, 0x01, 0x5F, 0xA5, 0xF4, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14,
  0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14,
  0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14,
  0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14,
  0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14,
  0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x57, 0xE6, 0x9F, 0x89, 0x3E, 0x3C, 0xF8,
  0x17, 0xF6, 0x78, 0xFF, 0x00, 0x82, 0xAC, 0xFC, 0x48, 0xF1, 0x27, 0xC4, 0x1D, 0x73, 0xFE, 0x11,
  0xFD, 0x16, 0xE7, 0xC1, 0xB6, 0xDA, 0x7C, 0x57, 0x5F, 0x64, 0x9E, 0xE7, 0x74, 0xEC, 0xB6, 0x2E,
  0xA9, 0xB6, 0x14, 0x76, 0x19, 0x58, 0x9C, 0xE4, 0x8C, 0x71, 0xD7, 0x91, 0x5F, 0xA5, 0x94, 0x50,
  0x07, 0xE6, 0xEF, 0xED, 0x0B, 0xF1, 0x63, 0x4C, 0xFD, 0xBE, 0x3E, 0x23, 0x7C, 0x1E, 0xF0, 0x9F,
  0xC1, 0x7B, 0x1D, 0x5F, 0xC4, 0x1A, 0x7E, 0x83, 0xE2, 0x58, 0x75, 0xCD, 0x5F, 0xC5, 0xCF, 0xA6,
  0x4F, 0x69, 0x65, 0xA7, 0xC1, 0x19, 0x19, 0x01, 0xE5, 0x45, 0x3B, 0x88, 0xDC, 0x71, 0x81, 0x92,
  0xA8, 0x06, 0x49, 0xE3, 0xDD, 0x3F, 0xE0, 0xA9, 0x9F, 0xF2, 0x62, 0xFF, 0x00, 0x11, 0xFF, 0x00,
  0xDE, 0xD3, 0x3F, 0xF4, 0xE5, 0x6B, 0x5F, 0x57, 0x51, 0x40, 0x1C, 0xAF, 0xC2, 0x81, 0x8F, 0x85,
  0xBE, 0x0E, 0x03, 0x81, 0xFD, 0x8D, 0x67, 0xFF, 0x00, 0xA2, 0x12, 0xBE, 0x5F, 0xFF, 0x00, 0x82,
  0x84, 0xF8, 0x13, 0xC5, 0x16, 0xBA, 0xF7, 0xC1, 0xBF, 0x8C, 0xBE, 0x16, 0xD0, 0xAE, 0xFC, 0x4E,
  0xFF, 0x00, 0x0D, 0x75, 0xC7, 0xBC, 0xD4, 0xF4, 0x8D, 0x3A, 0x33, 0x25, 0xCC, 0xD6, 0x32, 0x98,
  0x4C, 0xAE, 0x8A, 0x39, 0x25, 0x44, 0x18, 0xE3, 0x38, 0xF3, 0x37, 0x1E, 0x14, 0x91, 0xF6, 0x55,
  0x14, 0x01, 0xF9, 0xDB, 0xFB, 0x54, 0x7E, 0xD8, 0x1E, 0x19, 0xFD, 0xAD, 0x7E, 0x0D, 0x5E, 0x7C,
  0x21, 0xF8, 0x25, 0x61, 0xAD, 0xF8, 0xCB, 0xC6, 0xBE, 0x2C, 0x96, 0xDA, 0xD2, 0xE2, 0xDF, 0xFB,
  0x26, 0xE2, 0xDA, 0x3D, 0x26, 0x25, 0x99, 0x24, 0x91, 0xEE, 0x24, 0x91, 0x02, 0xAE, 0x36, 0x6D,
  0x24, 0x12, 0xA3, 0x24, 0x96, 0xE3, 0x9F, 0xBB, 0x3E, 0x18, 0x78, 0x31, 0x7E, 0x1C, 0xFC, 0x35,
  0xF0, 0x97, 0x84, 0xD2, 0x61, 0x70, 0x9A, 0x0E, 0x91, 0x69, 0xA5, 0xAC, 0xC0, 0x63, 0x78, 0x82,
  0x14, 0x88, 0x37, 0xE3, 0xB3, 0x35, 0xD3, 0x51, 0x40, 0x1F, 0x9C, 0x1F, 0x02, 0x7F, 0x68, 0x1F,
  0x08, 0x7E, 0xC5, 0x9F, 0x1B, 0x3F, 0x68, 0x0F, 0x0C, 0xFC, 0x61, 0x9B, 0x51, 0xF0, 0x7A, 0xEB,
  0xFE, 0x31, 0xB9, 0xD7, 0xF4, 0x7D, 0x4A, 0x5D, 0x2E, 0xE6, 0xE2, 0xDA, 0xFA, 0xDA, 0x67, 0x76,
  0x52, 0x8D, 0x0C, 0x6E, 0x7E, 0xE9, 0x43, 0xD3, 0x19, 0x24, 0x67, 0x2A, 0x45, 0x7A, 0x0F, 0xFC,
  0x14, 0xDF, 0xC5, 0xBA, 0x4F, 0x8F, 0xBF, 0xE0, 0x9F, 0xF7, 0xBE, 0x25, 0xD0, 0x6E, 0xFE, 0xDF,
  0xA2, 0x6A, 0xF2, 0xE9, 0x57, 0xF6, 0x37, 0x5E, 0x5B, 0xC7, 0xE6, 0xC1, 0x24, 0xD1, 0xBA, 0x3E,
  0xD7, 0x01, 0x97, 0x2A, 0x41, 0xC3, 0x00, 0x47, 0x70, 0x2B, 0xED, 0xEA, 0x28, 0x03, 0xC3, 0x3F,
  0x6A, 0x7F, 0xF9, 0x32, 0xEF, 0x89, 0xDF, 0xF6, 0x25, 0x5E, 0xFF, 0x00, 0xE9, 0x23, 0x57, 0x86,
  0x69, 0x9F, 0x04, 0x35, 0x0F, 0xDA, 0x1F, 0xFE, 0x09, 0x51, 0xE1, 0x6F, 0x04, 0xE8, 0xEC, 0x8B,
  0xAD, 0xDD, 0x78, 0x6A, 0xCA, 0xE7, 0x4F, 0x12, 0xB0, 0x55, 0x79, 0xE1, 0x91, 0x65, 0x58, 0xC9,
  0x3C, 0x0D, 0xFB, 0x0A, 0x64, 0xF0, 0x37, 0xE4, 0xF4, 0xAF, 0xB9, 0xA8, 0xA0, 0x0F, 0xC9, 0x5F,
  0x0A, 0x78, 0x7F, 0xF6, 0x1E, 0xD1, 0xBC, 0x3B, 0xA7, 0xE9, 0x7F, 0x13, 0xBE, 0x15, 0xF8, 0x93,
  0xC1, 0x3F, 0x11, 0xE0, 0x81, 0x20, 0xD4, 0x3C, 0x39, 0x7C, 0xBA, 0xFB, 0xCF, 0x2D, 0xD0, 0x50,
  0x1F, 0xC9, 0x31, 0x4A, 0xC8, 0xC8, 0xCC, 0x09, 0x52, 0x4A, 0xE4, 0x11, 0xC0, 0xAF, 0xBB, 0xBF,
  0x63, 0x1F, 0x02, 0x78, 0x33, 0xC1, 0x7F, 0x0B, 0x6F, 0x26, 0xF0, 0x37, 0xC3, 0x3D, 0x6B, 0xE1,
  0x66, 0x87, 0xAA, 0xEA, 0x52, 0x5D, 0xC7, 0xA4, 0xF8, 0x82, 0xE2, 0x67, 0xBC, 0xB8, 0x01, 0x11,
  0x16, 0xE5, 0xD2, 0x59, 0x64, 0x68, 0x77, 0x84, 0x00, 0x46, 0x5B, 0xA2, 0x83, 0xFC, 0x55, 0xEF,
  0x94, 0x50, 0x07, 0xC4, 0xFF, 0x00, 0x00, 0x87, 0xFC, 0x6C, 0xEB, 0xF6, 0x99, 0x3D, 0xFF, 0x00,
  0xB2, 0x34, 0x5F, 0xFD, 0x24, 0xB6, 0xAF, 0xB6, 0x28, 0xA2, 0x80, 0x0A, 0x28, 0xA2, 0x80, 0x3F,
  0x2D, 0x7F, 0x64, 0xBF, 0xDA, 0xBF, 0xE1, 0x8F, 0xEC, 0xE3, 0xF1, 0x97, 0xF6, 0x99, 0xB2, 0xF1,
  0xE6, 0xBD, 0x3E, 0x99, 0xA8, 0x6A, 0xBE, 0x39, 0xBB, 0x9A, 0xC6, 0xD2, 0xDF, 0x4D, 0xB9, 0xBB,
  0x92, 0xE1, 0x52, 0xE2, 0xE4, 0x36, 0x3C, 0xA8, 0xD9, 0x41, 0xCB, 0x01, 0xF3, 0x11, 0xD6, 0xBD,
  0x8F, 0xE0, 0x2F, 0x85, 0xFC, 0x4D, 0xFB, 0x4C, 0x7E, 0xD9, 0xB7, 0x5F, 0xB4, 0x56, 0xB3, 0xE1,
  0x7D, 0x53, 0xC2, 0x3E, 0x07, 0xD0, 0x34, 0x63, 0xA1, 0xF8, 0x4A, 0xDB, 0x5C, 0x83, 0xEC, 0xF7,
  0xB7, 0xBB, 0x84, 0x81, 0xEE, 0x0C, 0x47, 0x95, 0x4C, 0x4F, 0x71, 0xCF, 0x7F, 0x31, 0x00, 0x27,
  0x6B, 0x63, 0xEE, 0x7A, 0x28, 0x03, 0xE1, 0x9F, 0xDA, 0x17, 0xC3, 0x1E, 0x24, 0xFD, 0x9A, 0x3F,
  0x6C, 0x4D, 0x37, 0xF6, 0x8E, 0xD0, 0xFC, 0x37, 0xA9, 0x78, 0xAB, 0xC1, 0x5A, 0xD6, 0x91, 0xFD,
  0x85, 0xE3, 0x0B, 0x4D, 0x1A, 0x03, 0x3D, 0xE5, 0xA2, 0xAE, 0xC0, 0x97, 0x22, 0x31, 0xCB, 0x26,
  0x22, 0x80, 0x9E, 0xC3, 0xC9, 0x60, 0x48, 0xDE, 0xB5, 0xE9, 0x1E, 0x16, 0xFF, 0x00, 0x82, 0x86,
  0xFC, 0x2E, 0xF8, 0x97, 0xE3, 0x3F, 0x0E, 0x78, 0x5B, 0xE1, 0xFC, 0x7A, 0xFF, 0x00, 0x8C, 0xF5,
  0x7D, 0x5A, 0xF6, 0x2B, 0x59, 0xFE, 0xC9, 0xA3, 0x5C, 0xC1, 0x0E, 0x99, 0x1B, 0x11, 0xBE, 0x6B,
  0x87, 0x96, 0x35, 0xDA, 0xA8, 0x32, 0x7E, 0x50, 0xDC, 0xE3, 0x24, 0x0E, 0x47, 0xD3, 0xD4, 0x50,
  0x07, 0xC2, 0x7F, 0xB6, 0x0F, 0xFC, 0xA4, 0x07, 0xF6, 0x51, 0xFF, 0x00, 0xAE, 0xF7, 0xDF, 0xFB,
  0x25, 0x7D, 0xD9, 0x45, 0x14, 0x01, 0x91, 0xE2, 0xAF, 0x08, 0x68, 0x3E, 0x3B, 0xD0, 0xEE, 0x34,
  0x5F, 0x12, 0xE8, 0x9A, 0x77, 0x88, 0x74, 0x6B, 0x82, 0xA6, 0x6D, 0x3B, 0x55, 0xB4, 0x8E, 0xEA,
  0xDE, 0x52, 0xAC, 0x19, 0x77, 0x47, 0x20, 0x2A, 0x70, 0xC0, 0x11, 0x91, 0xC1, 0x00, 0xF6, 0xAC,
  0x1F, 0x05, 0x7C, 0x11, 0xF8, 0x75, 0xF0, 0xD7, 0x55, 0x97, 0x53, 0xF0, 0x87, 0x80, 0x7C, 0x2F,
  0xE1, 0x5D, 0x4A, 0x58, 0x4D, 0xB4, 0x97, 0x9A, 0x26, 0x8D, 0x6D, 0x67, 0x33, 0xC4, 0x59, 0x58,
  0xC6, 0x5E, 0x34, 0x52, 0x54, 0xB2, 0x29, 0xDA, 0x4E, 0x32, 0xA0, 0xF6, 0x15, 0xDA, 0xD1, 0x40,
  0x05, 0x7C, 0x27, 0xFF, 0x00, 0x04, 0xF0, 0xFF, 0x00, 0x93, 0x8F, 0xFD, 0xB1, 0x3F, 0xEC, 0x79,
  0x6F, 0xFD, 0x2A, 0xBF, 0xAF, 0xBB, 0x28, 0xA0, 0x06, 0x4B, 0x2A, 0x41, 0x13, 0xCB, 0x2B, 0xAC,
  0x71, 0xA2, 0x96, 0x67, 0x73, 0x80, 0xA0, 0x75, 0x24, 0xF6, 0x15, 0xF0, 0x8F, 0xEC, 0x22, 0x8F,
  0xFB, 0x42, 0x7E, 0xD1, 0x5F, 0x19, 0xFF, 0x00, 0x68, 0xDB, 0xD4, 0x69, 0x34, 0xDB, 0x9B, 0xCF,
  0xF8, 0x45, 0xFC, 0x2E, 0x64, 0x1C, 0x2D, 0x94, 0x5B, 0x0B, 0xBA, 0x83, 0xD3, 0x72, 0xAD, 0xB9,
  0xC8, 0xFE, 0x27, 0x94, 0x7A, 0xD7, 0xDE, 0x34, 0x50, 0x07, 0x03, 0xF1, 0xE3, 0xE3, 0x1E, 0x91,
  0xFB, 0x3F, 0xFC, 0x22, 0xF1, 0x2F, 0xC4, 0x0D, 0x72, 0x0B, 0x8B, 0xAD, 0x37, 0x44, 0xB7, 0x59,
  0x5A, 0xDE, 0xD4, 0x66, 0x49, 0x9D, 0xE4, 0x58, 0xE3, 0x41, 0x9E, 0x06, 0xE9, 0x1D, 0x17, 0x27,
  0x81, 0x9C, 0x9E, 0x95, 0xF9, 0xC7, 0xFB, 0x3C, 0xFE, 0xD6, 0x5F, 0x03, 0xBC, 0x41, 0xF1, 0x32,
  0xE7, 0xE3, 0x97, 0xC7, 0x8F, 0x89, 0x36, 0xF7, 0x3F, 0x12, 0x66, 0xDD, 0x0E, 0x89, 0xE1, 0xC8,
  0xB4, 0x6D, 0x46, 0x7B, 0x2F, 0x0C, 0x5A, 0x64, 0x85, 0x8E, 0x22, 0xB6, 0xE5, 0x5A, 0x52, 0x09,
  0xCB, 0x82, 0x71, 0xB8, 0x9C, 0x96, 0x62, 0x47, 0xEA, 0xD5, 0x14, 0x01, 0xE5, 0x1A, 0x3F, 0x8C,
  0xFC, 0x0B, 0xFB, 0x5F, 0x7C, 0x0B, 0xF1, 0x17, 0xFC, 0x22, 0x3A, 0xD9, 0xD6, 0x3C, 0x2B, 0xE2,
  0x2B, 0x2B, 0xDD, 0x11, 0xEF, 0xC5, 0xA4, 0xD0, 0x10, 0x5E, 0x36, 0x8A, 0x4F, 0x92, 0x64, 0x46,
  0xE3, 0x7F, 0xA6, 0x0F, 0x6A, 0xF8, 0xFF, 0x00, 0xF6, 0x5F, 0xFD, 0xAD, 0x74, 0x6F, 0xD8, 0xDF,
  0xE1, 0x01, 0xF8, 0x39, 0xF1, 0xAB, 0x4C, 0xD7, 0x34, 0x0F, 0x1A, 0x78, 0x4A, 0xE2, 0xEA, 0xDF,
  0x4D, 0x82, 0xDB, 0x4A, 0x9E, 0xE5, 0x35, 0xC8, 0x1E, 0x67, 0x96, 0x23, 0x6D, 0x22, 0x2E, 0xC6,
  0xC9, 0x76, 0x50, 0x58, 0xAA, 0xE0, 0x29, 0xDD, 0x9C, 0x81, 0xFA, 0x2F, 0x45, 0x00, 0x7C, 0x6F,
  0xFB, 0x00, 0x7C, 0x28, 0xF1, 0x47, 0x86, 0xBC, 0x0B, 0xF1, 0x43, 0xE2, 0x5F, 0x8D, 0xF4, 0x99,
  0xB4, 0x0F, 0x13, 0xFC, 0x4C, 0xD6, 0xAE, 0x75, 0xD7, 0xD2, 0xAE, 0x14, 0xAC, 0xB6, 0xB6, 0xC4,
  0xC8, 0xD0, 0xAB, 0xA9, 0x00, 0xAB, 0x6E, 0x9A, 0x53, 0xB4, 0x80, 0x42, 0x95, 0xC8, 0x07, 0x20,
  0x7C, 0xDF, 0xFF, 0x00, 0x04, 0xE0, 0xFD, 0xB5, 0xFE, 0x14, 0x7C, 0x05, 0xFD, 0x9A, 0xA4, 0xF0,
  0xAF, 0x89, 0xB5, 0x7D, 0x42, 0x5F, 0x16, 0x7F, 0x6C, 0x5D, 0x5D, 0xC5, 0xA1, 0xE9, 0x7A, 0x4D,
  0xD5, 0xD4, 0xF3, 0x23, 0xAC, 0x61, 0x36, 0x32, 0xC7, 0xE5, 0xE4, 0x95, 0x23, 0x05, 0xC7, 0xBE,
  0x2B, 0xF5, 0x62, 0x8A, 0x00, 0xF8, 0xC3, 0xF6, 0x29, 0xF8, 0x61, 0xE2, 0xFF, 0x00, 0x14, 0xFC,
  0x6F, 0xF8, 0xA9, 0xFB, 0x45, 0xF8, 0xEB, 0xC3, 0xD7, 0x7E, 0x11, 0xBD, 0xF1, 0x9F, 0x97, 0xA7,
  0xE8, 0x7A, 0x1E, 0xA2, 0xBB, 0x2F, 0x20, 0xD3, 0xA3, 0x11, 0x80, 0xD3, 0x27, 0x54, 0x2C, 0xB0,
  0x5B, 0x80, 0x0E, 0x0E, 0x51, 0x8E, 0x30, 0x54, 0x9F, 0x0D, 0xF8, 0x23, 0xE0, 0x6D, 0x42, 0xEF,
  0xFE, 0x0A, 0x83, 0xE3, 0x3F, 0x07, 0x49, 0x0E, 0x3C, 0x3D, 0xE1, 0x4F, 0x10, 0x6A, 0xDE, 0x3F,
  0x78, 0xCF, 0xDD, 0x69, 0xEF, 0xAD, 0xAD, 0xD2, 0x22, 0x47, 0x62, 0xA6, 0xE1, 0x59, 0x7F, 0x1F,
  0x4A, 0xFD, 0x41, 0xA2, 0x80, 0x3F, 0x2F, 0xBF, 0x66, 0x8F, 0x03, 0x6A, 0x1A, 0x3F, 0xFC, 0x14,
  0xD7, 0x5F, 0xF0, 0x45, 0xC4, 0x3B, 0x74, 0x8F, 0x87, 0xBF, 0xDB, 0xDA, 0xE6, 0x94, 0xBD, 0xA1,
  0xB7, 0xD4, 0x5A, 0x06, 0x44, 0x1F, 0x45, 0xBB, 0xFC, 0x72, 0x7D, 0x2B, 0xF5, 0x06, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xA2, 0x8A, 0x28,
  0x00, 0xA2, 0x8A, 0xC4, 0xF1, 0x0F, 0x8E, 0x3C, 0x39, 0xE1, 0x2B, 0xDD, 0x26, 0xCF, 0x5D, 0xF1,
  0x06, 0x97, 0xA2, 0xDD, 0xEA, 0xF7, 0x22, 0xCF, 0x4D, 0x83, 0x50, 0xBD, 0x8E, 0x09, 0x2F, 0x67,
  0x24, 0x01, 0x14, 0x2A, 0xEC, 0x0C, 0x8E, 0x49, 0x00, 0x2A, 0xE4, 0xF2, 0x38, 0xE6, 0x80, 0x36,
  0xE8, 0xA2, 0x8A, 0x00, 0x28, 0xA2, 0x8A, 0x00, 0x28, 0xA2, 0x8A, 0x00, 0x28, 0xA2, 0x8A, 0x00,
  0x28, 0xA2, 0x8A, 0x00, 0x28, 0xA2, 0x8A, 0x00, 0x28, 0xAC, 0x4B, 0x4F, 0x1C, 0x78, 0x73, 0x50,
  0xF1, 0x5D, 0xF7, 0x85, 0xED, 0x7C, 0x41, 0xA5, 0xDC, 0xF8, 0x96, 0xC2, 0x15, 0xB8, 0xBB, 0xD1,
  0xA1, 0xBD, 0x8D, 0xEF, 0x2D, 0xE2, 0x6C, 0x6D, 0x79, 0x21, 0x0D, 0xBD, 0x14, 0xEE, 0x5C, 0x12,
  0x00, 0x3B, 0x87, 0xAD, 0x6D, 0xD0, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45,
  0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45,
  0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45,
  0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x45,
  0x14, 0x50, 0x01, 0x45, 0x14, 0x50, 0x01, 0x5F, 0x9B, 0xBF, 0x1B, 0xBE, 0x0B, 0xF8, 0x37, 0xE3,
  0xD7, 0xFC, 0x15, 0x46, 0xC7, 0xC2, 0xBE, 0x3B, 0xD1, 0xFF, 0x00, 0xB7, 0x74, 0x07, 0xF0, 0x3A,
  0xDC, 0xB5, 0xA7, 0xDA, 0xA6, 0xB7, 0xCC, 0x88, 0xD2, 0x6D, 0x6D, 0xF0, 0xBA, 0x37, 0x19, 0x3C,
  0x67, 0x15, 0xFA, 0x45, 0x5F, 0x9B, 0xBF, 0x1B, 0xBE, 0x0B, 0xF8, 0x37, 0xE3, 0xD7, 0xFC, 0x15,
  0x46, 0xC7, 0xC2, 0xBE, 0x3B, 0xD1, 0xFF, 0x00, 0xB7, 0x74, 0x07, 0xF0, 0x3A, 0xDC, 0xB5, 0xA7,
  0xDA, 0xA6, 0xB7, 0xCC, 0x88, 0xD2, 0x6D, 0x6D, 0xF0, 0xBA, 0x37, 0x19, 0x3C, 0x67, 0x14, 0x01,
  0xF4, 0x8F, 0x83, 0x3F, 0xE0, 0x9D, 0x3F, 0xB3, 0xCF, 0xC3, 0xDF, 0x16, 0xE8, 0xFE, 0x26, 0xF0,
  0xFF, 0x00, 0xC3, 0xEF, 0xB0, 0x6B, 0x9A, 0x45, 0xD4, 0x77, 0xB6, 0x57, 0x5F, 0xDB, 0x5A, 0x8C,
  0x9E, 0x54, 0xD1, 0xB0, 0x64, 0x6D, 0xAF, 0x70, 0x55, 0xB0, 0x40, 0x38, 0x60, 0x47, 0xA8, 0xAF,
  0x26, 0xFD, 0xAE, 0xF5, 0xDB, 0xEF, 0x8A, 0xFF, 0x00, 0xB6, 0x57, 0xC2, 0xEF, 0x80, 0x3A, 0xC7,
  0x8A, 0x75, 0x1F, 0x08, 0xFC, 0x3C, 0xD5, 0xF4, 0x69, 0x35, 0x8D, 0x47, 0xFB, 0x2E, 0xEB, 0xEC,
  0x92, 0xEB, 0x13, 0xEE, 0xB8, 0x58, 0xED, 0x0C, 0xBF, 0xDD, 0xFD, 0xC0, 0xF9, 0x7B, 0xEF, 0x3C,
  0x6E, 0xD8, 0x47, 0xB1, 0x7C, 0x3C, 0xFF, 0x00, 0x82, 0x7D, 0x7C, 0x01, 0xF8, 0x53, 0xE3, 0x4D,
  0x2B, 0xC5, 0xBE, 0x16, 0xF0, 0x17, 0xF6, 0x5F, 0x88, 0x34, 0xA9, 0x7C, 0xEB, 0x3B, 0xCF, 0xED,
  0x9D, 0x42, 0x6F, 0x29, 0xF6, 0x95, 0xCE, 0xC9, 0x27, 0x65, 0x3C, 0x31, 0xEA, 0x0D, 0x75, 0x9F,
  0xB4, 0x2F, 0xEC, 0xAD, 0xF0, 0xE7, 0xF6, 0x9E, 0xD1, 0xAC, 0xEC, 0x7C, 0x75, 0xA3, 0x35, 0xD5,
  0xC5, 0x81, 0x63, 0x63, 0xA9, 0xD9, 0xCC, 0x60, 0xBC, 0xB4, 0x2D, 0x8D, 0xDB, 0x24, 0x1D, 0x8E,
  0x06, 0x55, 0x83, 0x2E, 0x40, 0x38, 0xC8, 0x06, 0x80, 0x3C, 0xE3, 0xC3, 0xFF, 0x00, 0xF0, 0x4E,
  0xEF, 0x86, 0x3F, 0x0E, 0xFC, 0x5F, 0xE1, 0xBF, 0x12, 0xFC, 0x3B, 0xBB, 0xF1, 0x1F, 0x80, 0xF5,
  0x1D, 0x23, 0x50, 0x82, 0xF2, 0x68, 0xAC, 0x35, 0xAB, 0x99, 0xAD, 0xF5, 0x28, 0xD2, 0x40, 0xD2,
  0x41, 0x71, 0x1C, 0xB2, 0x36, 0xE5, 0x75, 0x05, 0x4E, 0xD2, 0x07, 0x3C, 0x82, 0x38, 0x3E, 0x57,
  0xFF, 0x00, 0x05, 0x6E, 0xD2, 0x6D, 0x35, 0xFF, 0x00, 0x08, 0xFC, 0x18, 0xD3, 0x2F, 0xE2, 0xF3,
  0xEC, 0x6F, 0x7C, 0x71, 0x6D, 0x6D, 0x71, 0x16, 0xE2, 0xBB, 0xE3, 0x78, 0xDD, 0x59, 0x72, 0x08,
  0x23, 0x20, 0x91, 0x90, 0x41, 0xAE, 0x43, 0xE3, 0x97, 0x83, 0xBE, 0x21, 0x7F, 0xC1, 0x37, 0xAC,
  0x3C, 0x37, 0xF1, 0x07, 0xC1, 0xBF, 0x16, 0x7C, 0x4D, 0xE3, 0x5F, 0x00, 0x1D, 0x62, 0x0D, 0x33,
  0x53, 0xF0, 0x77, 0x8B, 0xA7, 0x17, 0x64, 0xC3, 0x20, 0x66, 0xCC, 0x32, 0x70, 0x14, 0xE2, 0x32,
  0x3E, 0x45, 0x52, 0x0E, 0xD3, 0xF3, 0x0C, 0x8A, 0xEB, 0xFF, 0x00, 0xE0, 0xAD, 0xD6, 0xD7, 0x77,
  0xBE, 0x11, 0xF8, 0x31, 0x6F, 0x61, 0x7B, 0xFD, 0x9D, 0x7D, 0x2F, 0x8E, 0x2D, 0xA3, 0xB7, 0xBC,
  0xF2, 0x84, 0xBE, 0x44, 0x86, 0x37, 0x0B, 0x26, 0xC3, 0xC3, 0x6D, 0x38, 0x3B, 0x4F, 0x07, 0x18,
  0xA0, 0x0A, 0x7F, 0xB5, 0x37, 0xEC, 0x21, 0xF0, 0xCB, 0xE0, 0x3F, 0xC0, 0xFF, 0x00, 0x14, 0x7C,
  0x46, 0xF8, 0x47, 0xFD, 0xAF, 0xF0, 0xC7, 0xC6, 0x3E, 0x17, 0xB6, 0xFE, 0xD3, 0xB6, 0xD4, 0x34,
  0xCD, 0x72, 0xF1, 0xC4, 0xDB, 0x18, 0x66, 0x27, 0x12, 0xCA, 0xFC, 0x30, 0xC8, 0x1B, 0x71, 0xC9,
  0x19, 0xC8, 0xC8, 0x3E, 0xF7, 0xF0, 0xE2, 0xC5, 0xFF, 0x00, 0x6C, 0x7F, 0xD9, 0x17, 0xC0, 0x57,
  0x7E, 0x30, 0xD4, 0xF5, 0x8D, 0x1E, 0x7D, 0x73, 0x4F, 0xB5, 0xBC, 0xD4, 0xA5, 0xF0, 0xFD, 0xEB,
  0x58, 0x4B, 0x73, 0x22, 0x65, 0x5C, 0x16, 0x41, 0x91, 0x1C, 0x8C, 0x0B, 0x15, 0x18, 0xE0, 0x81,
  0x9E, 0x2B, 0x80, 0xF1, 0x77, 0xEC, 0x37, 0xF1, 0x37, 0xE3, 0x35, 0x84, 0x5A, 0x17, 0xC5, 0xCF,
  0xDA, 0x4B, 0x5A, 0xF1, 0x8F, 0x83, 0x8C, 0xC9, 0x2D, 0xCE, 0x89, 0xA4, 0x78, 0x66, 0xCF, 0x45,
  0xFB, 0x56, 0xD6, 0x0C, 0x16, 0x49, 0x22, 0x66, 0xDC, 0x32, 0x01, 0xC3, 0x29, 0xC1, 0x00, 0x8C,
  0x10, 0x0D, 0x7D, 0x67, 0xE1, 0x7F, 0x0C, 0xE9, 0x7E, 0x0B, 0xF0, 0xD6, 0x95, 0xE1, 0xFD, 0x12,
  0xCE, 0x3D, 0x3F, 0x47, 0xD2, 0xED, 0x63, 0xB3, 0xB3, 0xB4, 0x8B, 0x3B, 0x61, 0x86, 0x35, 0x0A,
  0x8A, 0x33, 0xCF, 0x00, 0x01, 0xCF, 0x34, 0x01, 0xF9, 0xF3, 0xFB, 0x60, 0xFE, 0xC9, 0xBF, 0x08,
  0x3F, 0x65, 0xAF, 0x82, 0xFA, 0xC7, 0xC4, 0x2F, 0x87, 0xDA, 0xCE, 0xA9, 0xF0, 0xD7, 0xE2, 0x26,
  0x98, 0x62, 0x9F, 0x4B, 0xBE, 0xB7, 0xF1, 0x15, 0xD3, 0xCF, 0xA8, 0xCF, 0xE6, 0x28, 0x30, 0xB4,
  0x72, 0xCA, 0xDE, 0x66, 0xE5, 0x2D, 0x90, 0xA0, 0x63, 0x19, 0x39, 0x50, 0xC0, 0xFD, 0x2D, 0x7F,
  0xF0, 0xCF, 0x5C, 0xFD, 0xAB, 0x7F, 0x66, 0x2F, 0x87, 0x96, 0x5E, 0x33, 0xD7, 0xB5, 0xCF, 0x04,
  0x6A, 0x9A, 0x8D, 0x9E, 0x9D, 0xAA, 0xEB, 0xC3, 0x41, 0x90, 0xD9, 0xDC, 0xCE, 0xC6, 0xDC, 0x99,
  0x6D, 0xCF, 0xFC, 0xF3, 0x56, 0x77, 0x0C, 0x54, 0xA9, 0xC6, 0xDC, 0x63, 0x3C, 0x8F, 0x14, 0xFD,
  0xA7, 0x7F, 0x60, 0x9F, 0x85, 0xBF, 0x0C, 0x7E, 0x1B, 0x78, 0xBB, 0xE2, 0xCF, 0xC3, 0x9B, 0x3B,
  0x8F, 0x02, 0xF8, 0xFF, 0x00, 0xC2, 0xD6, 0xB7, 0x3E, 0x24, 0xB5, 0xD4, 0xD2, 0xFA, 0x5B, 0xC8,
  0xA6, 0x9A, 0x25, 0x69, 0x59, 0x24, 0x86, 0xE5, 0xA4, 0x43, 0xBF, 0x0C, 0xA3, 0x68, 0x18, 0x66,
  0x5E, 0xA0, 0x60, 0xFD, 0x29, 0xFB, 0x32, 0x7C, 0x62, 0xB9, 0xF8, 0xBF, 0xFB, 0x37, 0x78, 0x2F,
  0xE2, 0x1F, 0x88, 0x56, 0xDF, 0x4B, 0xBB, 0xD4, 0x74, 0xBF, 0xB5, 0x6A, 0x0E, 0x4F, 0x95, 0x02,
  0x3C, 0x65, 0x96, 0x59, 0x39, 0x3F, 0x2A, 0x12, 0x8C, 0xFC, 0x9E, 0x01, 0xEB, 0xC6, 0x68, 0x03,
  0xE4, 0xEF, 0xDA, 0xAB, 0xF6, 0x2B, 0xF8, 0x7B, 0xFB, 0x33, 0xFC, 0x0A, 0xF1, 0x07, 0xC5, 0x1F,
  0x85, 0x7A, 0x8E, 0xBD, 0xF0, 0xFB, 0xC6, 0xFE, 0x18, 0x48, 0x6F, 0x21, 0xD5, 0xED, 0xF5, 0xBB,
  0x99, 0x7E, 0xD8, 0xDE, 0x72, 0x21, 0x8A, 0x65, 0x95, 0xD9, 0x58, 0x3E, 0xFC, 0x00, 0x00, 0x04,
  0x90, 0x08, 0x23, 0x20, 0xFD, 0x99, 0xFB, 0x3E, 0xF8, 0xEB, 0x52, 0xF8, 0x9F, 0xF0, 0x33, 0xC0,
  0x1E, 0x2E, 0xD6, 0x6D, 0xD6, 0xD7, 0x56, 0xD6, 0xF4, 0x3B, 0x3B, 0xFB, 0xA8, 0xD1, 0x76, 0xAF,
  0x9B, 0x24, 0x2A, 0xCC, 0xCA, 0x3B, 0x29, 0x27, 0x20, 0x7A, 0x11, 0x5F, 0x22, 0xEB, 0xB2, 0x6A,
  0x5F, 0xF0, 0x52, 0xFF, 0x00, 0x89, 0x71, 0x69, 0x3A, 0x79, 0xB9, 0xD3, 0xBF, 0x66, 0x9F, 0x09,
  0xEA, 0x01, 0xEF, 0x75, 0x11, 0xBA, 0x27, 0xF1, 0x65, 0xEC, 0x67, 0xFD, 0x5C, 0x47, 0x82, 0x20,
  0x5F, 0x5F, 0x7C, 0xFD, 0xE2, 0xBE, 0x5F, 0xDE, 0x76, 0x56, 0x56, 0xFA, 0x6D, 0x9C, 0x16, 0x96,
  0x90, 0x47, 0x6D, 0x6B, 0x6F, 0x1A, 0xC5, 0x0C, 0x11, 0x28, 0x54, 0x8D, 0x14, 0x61, 0x55, 0x40,
  0xE0, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x9E, 0x8A, 0x28, 0xA0, 0x02, 0xB2, 0x7C, 0x5B, 0xAB, 0xDD,
  0xF8, 0x7B, 0xC2, 0xBA, 0xCE, 0xA9, 0x61, 0xA6, 0x4D, 0xAD, 0x5F, 0x58, 0xD9, 0x4D, 0x73, 0x6F,
  0xA6, 0x5B, 0x9C, 0x49, 0x77, 0x22, 0x46, 0x59, 0x61, 0x53, 0x83, 0x86, 0x72, 0x02, 0x8E, 0x0F,
  0x27, 0xA1, 0xAF, 0x13, 0xF8, 0x9B, 0xFF, 0x00, 0x0D, 0x55, 0xFF, 0x00, 0x09, 0xCE, 0xA9, 0xFF,
  0x00, 0x0A, 0xF3, 0xFE, 0x14, 0xEF, 0xFC, 0x21, 0xDB, 0x93, 0xFB, 0x3F, 0xFE, 0x12, 0x7F, 0xED,
  0x5F, 0xED, 0x0C, 0x79, 0x6B, 0xBF, 0xCD, 0xF2, 0x7F, 0x77, 0x9F, 0x33, 0x7E, 0x36, 0xFF, 0x00,
  0x0E, 0xDC, 0xF3, 0x9A, 0xF6, 0x7D, 0x02, 0xF7, 0x54, 0xD3, 0xBC, 0x15, 0xA6, 0xDD, 0xF8, 0xC6,
  0x7D, 0x2E, 0xD7, 0x5A, 0x83, 0x4F, 0x8E, 0x5D, 0x66, 0x7D, 0x3D, 0xD9, 0x2C, 0x23, 0x9D, 0x63,
  0x06, 0xE1, 0xA2, 0x69, 0x7E, 0x61, 0x08, 0x60, 0xE5, 0x4B, 0xF2, 0x17, 0x19, 0xE7, 0x34, 0x01,
  0xF3, 0xEF, 0xC2, 0xCF, 0xF8, 0x28, 0x77, 0xC2, 0x6F, 0x1F, 0x6A, 0xE3, 0xC3, 0xBE, 0x22, 0xBB,
  0xBE, 0xF8, 0x5D, 0xE3, 0x24, 0x21, 0x26, 0xF0, 0xFF, 0x00, 0x8D, 0xED, 0xCE, 0x9F, 0x2A, 0xBF,
  0xA0, 0x91, 0xBF, 0x76, 0x72, 0x78, 0x00, 0xB2, 0xB1, 0xFE, 0xED, 0x7D, 0x35, 0x14, 0xA9, 0x3C,
  0x49, 0x2C, 0x4E, 0xB2, 0x46, 0xEA, 0x19, 0x5D, 0x0E, 0x43, 0x03, 0xD0, 0x83, 0xDC, 0x57, 0xC4,
  0x7F, 0xB4, 0x2F, 0xED, 0x65, 0xFB, 0x3E, 0x7C, 0x59, 0x99, 0xFC, 0x0F, 0x61, 0xE0, 0x69, 0x7F,
  0x68, 0xDF, 0x12, 0x60, 0xA4, 0x3A, 0x5F, 0x86, 0xF4, 0xBF, 0xB5, 0xAC, 0x04, 0xFF, 0x00, 0x10,
  0xBC, 0xC6, 0x23, 0x5F, 0x59, 0x21, 0x2D, 0x8E, 0xF5, 0x99, 0xFB, 0x0E, 0xFE, 0xCC, 0x5F, 0x18,
  0xFE, 0x16, 0x7C, 0x49, 0xB8, 0xF1, 0x2E, 0xA9, 0x33, 0xFC, 0x37, 0xF8, 0x5F, 0x3C, 0x52, 0xFD,
  0x9F, 0xE1, 0x83, 0xEB, 0xD2, 0xEB, 0x0D, 0x0B, 0x32, 0xFC, 0x84, 0xBB, 0x65, 0x13, 0x04, 0x86,
  0xCA, 0xB6, 0xE3, 0x8C, 0x10, 0x28, 0x03, 0xEF, 0x2A, 0x28, 0xA2, 0x80, 0x3E, 0x51, 0xFD, 0xB9,
  0xBF, 0x65, 0x0F, 0x85, 0x7F, 0x12, 0xBE, 0x19, 0xFC, 0x46, 0xF8, 0x95, 0xE2, 0x3F, 0x0B, 0x7F,
  0x68, 0xF8, 0xD7, 0x45, 0xF0, 0x85, 0xFC, 0x96, 0x1A, 0x9F, 0xF6, 0x85, 0xD4, 0x5E, 0x4B, 0x5B,
  0xDB, 0x4D, 0x2C, 0x27, 0xCA, 0x49, 0x56, 0x36, 0xDA, 0xE4, 0x9F, 0x99, 0x4E, 0x7A, 0x1C, 0x8E,
  0x2B, 0xCE, 0x7F, 0xE0, 0x9A, 0x5F, 0xB2, 0x87, 0xC2, 0xB8, 0x7E, 0x09, 0x7C, 0x2F, 0xF8, 0xC2,
  0x9E, 0x16, 0xC7, 0xC4, 0x63, 0x05, 0xE4, 0xBF, 0xDB, 0x3F, 0xDA, 0x17, 0x5F, 0x78, 0xCB, 0x71,
  0x6E, 0x4F, 0x93, 0xE6, 0xF9, 0x5F, 0xEA, 0x89, 0x5F, 0xB9, 0xEF, 0xD7, 0x9A, 0xFA, 0x87, 0xF6,
  0xAC, 0xFF, 0x00, 0x93, 0x5D, 0xF8, 0xC3, 0xFF, 0x00, 0x62, 0x6E, 0xB1, 0xFF, 0x00, 0xA4, 0x53,
  0x57, 0x9C, 0xFF, 0x00, 0xC1, 0x35, 0xBF, 0xE4, 0xC8, 0x7E, 0x17, 0x7F, 0xD7, 0xAD, 0xD7, 0xFE,
  0x96, 0xDC, 0x50, 0x07, 0x86, 0x7F, 0xC1, 0x40, 0x7C, 0x01, 0xA0, 0xFC, 0x53, 0xFD, 0xB3, 0x7F,
  0x66, 0x6F, 0x09, 0xF8, 0xA2, 0xC3, 0xFB, 0x4F, 0x40, 0xD5, 0x4D, 0xFC, 0x17, 0x96, 0x7E, 0x74,
  0x90, 0xF9, 0xA9, 0x98, 0xCE, 0x37, 0xC6, 0xCA, 0xCB, 0xC8, 0x1C, 0x82, 0x0D, 0x60, 0xFE, 0xD9,
  0xDF, 0xB3, 0x37, 0x85, 0xFF, 0x00, 0x62, 0x6F, 0x86, 0x96, 0x5F, 0x17, 0xFE, 0x05, 0xDE, 0xEA,
  0x9F, 0x0E, 0xF5, 0xFD, 0x17, 0x54, 0xB6, 0x8E, 0x6B, 0x18, 0x75, 0x5B, 0x9B, 0x9B, 0x4D, 0x46,
  0x29, 0x1F, 0x69, 0x8E, 0x48, 0xE6, 0x91, 0xF7, 0x73, 0x82, 0x46, 0x76, 0x95, 0x0C, 0x08, 0xE8,
  0x46, 0xF7, 0xFC, 0x14, 0x07, 0x45, 0xD7, 0xBC, 0x45, 0xFB, 0x66, 0xFE, 0xCC, 0xDA, 0x6F, 0x85,
  0xFC, 0x49, 0xFF, 0x00, 0x08, 0x86, 0xBF, 0x72, 0x6F, 0xD2, 0xCF, 0x5C, 0xFB, 0x0C, 0x77, 0xBF,
  0x63, 0x7C, 0xC7, 0xF3, 0xF9, 0x12, 0x10, 0xB2, 0x71, 0x91, 0x82, 0x71, 0xCD, 0x7A, 0x46, 0xAF,
  0xFB, 0x04, 0xF8, 0xA3, 0xE2, 0xEE, 0xB9, 0xA2, 0x4D, 0xF1, 0xD3, 0xE3, 0x8E, 0xAF, 0xF1, 0x53,
  0x40, 0xD2, 0x2E, 0x56, 0xF2, 0x1F, 0x0E, 0xDB, 0x68, 0x76, 0xDA, 0x2D, 0x9C, 0xD2, 0x8E, 0x01,
  0x99, 0x61, 0x66, 0xDE, 0x30, 0x48, 0xE8, 0x1B, 0x0C, 0xC0, 0x30, 0xC9, 0xA0, 0x0F, 0x20, 0xFD,
  0xB4, 0xBC, 0x39, 0xA1, 0xFC, 0x7A, 0xFD, 0xA8, 0x7F, 0x64, 0xCD, 0x33, 0xC5, 0x7A, 0x51, 0xBA,
  0xD0, 0xBC, 0x4B, 0x67, 0x71, 0x25, 0xF6, 0x9A, 0x66, 0x92, 0x2C, 0xA4, 0x89, 0x14, 0x85, 0x37,
  0xC6, 0xCA, 0xE3, 0x07, 0x1C, 0x82, 0x0F, 0x15, 0x07, 0xED, 0x9D, 0xFB, 0x33, 0x78, 0x5F, 0xF6,
  0x26, 0xF8, 0x69, 0x65, 0xF1, 0x7F, 0xE0, 0x5D, 0xEE, 0xA9, 0xF0, 0xEF, 0x5F, 0xD1, 0x75, 0x4B,
  0x68, 0xE6, 0xB1, 0x87, 0x55, 0xB9, 0xB9, 0xB4, 0xD4, 0x62, 0x91, 0xF6, 0x98, 0xE4, 0x8E, 0x69,
  0x1F, 0x77, 0x38, 0x24, 0x67, 0x69, 0x50, 0xC0, 0x8E, 0x84, 0x6E, 0x7E, 0xDF, 0xDA, 0x0E, 0xB9,
  0xAE, 0xFE, 0xD9, 0x5F, 0xB3, 0x26, 0x95, 0xE1, 0x4F, 0x11, 0x0F, 0x06, 0xEB, 0xB3, 0xFD, 0xBA,
  0x2B, 0x1D, 0x69, 0x6C, 0x23, 0xBD, 0x16, 0x4F, 0x98, 0xF0, 0xE2, 0x09, 0x08, 0x47, 0xE0, 0x11,
  0xB4, 0xF1, 0xCD, 0x7A, 0x56, 0xAF, 0xFB, 0x04, 0xF8, 0xA3, 0xE2, 0xEE, 0xB9, 0xA2, 0x4D, 0xF1,
  0xD3, 0xE3, 0x8E, 0xAF, 0xF1, 0x53, 0x40, 0xD2, 0x2E, 0x56, 0xF2, 0x1F, 0x0E, 0xDB, 0x68, 0x76,
  0xDA, 0x2D, 0x9C, 0xD2, 0x8E, 0x01, 0x99, 0x61, 0x66, 0xDE, 0x30, 0x48, 0xE8, 0x1B, 0x0C, 0xC0,
  0x30, 0xC9, 0xA0, 0x0F, 0xAB, 0x7C, 0x21, 0xAE, 0x37, 0x89, 0xFC, 0x27, 0xA2, 0x6B, 0x2F, 0x01,
  0xB6, 0x7D, 0x46, 0xC6, 0x0B, 0xB6, 0x80, 0xFF, 0x00, 0xCB, 0x33, 0x24, 0x6A, 0xFB, 0x7F, 0x0C,
  0xE2, 0xB5, 0xE9, 0xB1, 0xC6, 0xB1, 0x22, 0xA2, 0x28, 0x44, 0x50, 0x02, 0xAA, 0x8C, 0x00, 0x3D,
  0x05, 0x3A, 0x80, 0x0A, 0xF9, 0x17, 0xFE, 0x0A, 0x27, 0xE3, 0xDD, 0x5A, 0xF3, 0xC1, 0x5E, 0x18,
  0xF8, 0x27, 0xE1, 0x09, 0xB6, 0xF8, 0xD3, 0xE2, 0x9E, 0xA0, 0xBA, 0x32, 0x14, 0xE4, 0xDB, 0x69,
  0xE0, 0x83, 0x77, 0x33, 0x63, 0xA2, 0xED, 0x21, 0x4F, 0xAA, 0x34, 0x98, 0xFB, 0xB5, 0xF5, 0xC3,
  0xBA, 0xC6, 0xA5, 0x98, 0x85, 0x55, 0x19, 0x24, 0x9C, 0x00, 0x2B, 0xE2, 0x5F, 0xD9, 0x39, 0x5B,
  0xF6, 0x9C, 0xFD, 0xA8, 0x7E, 0x23, 0x7E, 0xD0, 0xD7, 0x80, 0xCF, 0xE1, 0x9D, 0x1D, 0x9B, 0xC1,
  0xFE, 0x09, 0xDE, 0x32, 0xA6, 0x08, 0xF9, 0xB8, 0xBA, 0x4C, 0xFF, 0x00, 0x7C, 0xB1, 0xC1, 0xFF,
  0x00, 0xA6, 0xF2, 0x2F, 0xF0, 0xD0, 0x07, 0xD7, 0x3F, 0x0E, 0x7C, 0x05, 0xA4, 0xFC, 0x2D, 0xF0,
  0x17, 0x87, 0xFC, 0x23, 0xA1, 0x43, 0xE4, 0x69, 0x1A, 0x2D, 0x94, 0x56, 0x36, 0xC8, 0x7A, 0x94,
  0x45, 0x0B, 0xB9, 0xBD, 0x59, 0xB0, 0x58, 0x9E, 0xE4, 0x93, 0xDE, 0xBA, 0x3A, 0x28, 0xA0, 0x02,
  0xBE, 0x25, 0xF8, 0x95, 0xFB, 0x4D, 0x7C, 0x2A, 0xF1, 0xF7, 0xC6, 0x5F, 0x86, 0x1E, 0x1F, 0xF8,
  0x8B, 0xF0, 0x57, 0xE2, 0x9E, 0x97, 0xAD, 0xDA, 0x78, 0xB2, 0x0B, 0x7F, 0x0A, 0xEB, 0x3A, 0xEE,
  0x90, 0xDA, 0x75, 0x82, 0xEA, 0x26, 0x74, 0x44, 0x99, 0x1C, 0x5D, 0x2F, 0x9C, 0x81, 0x82, 0x36,
  0x0A, 0xBF, 0xCB, 0xCE, 0xD3, 0x9C, 0x1F, 0xB6, 0xAB, 0xE2, 0x8F, 0xF8, 0x28, 0x37, 0xFC, 0x96,
  0x5F, 0xD9, 0x33, 0xFE, 0xCA, 0x2D, 0xA7, 0xFE, 0x8E, 0xB7, 0xA0, 0x0F, 0xB1, 0xBC, 0x45, 0xAE,
  0x41, 0xE1, 0x9F, 0x0F, 0xEA, 0x7A, 0xC5, 0xD2, 0x49, 0x25, 0xB6, 0x9F, 0x6B, 0x2D, 0xDC, 0xA9,
  0x08, 0x05, 0xD9, 0x23, 0x42, 0xCC, 0x14, 0x12, 0x06, 0x70, 0x0E, 0x32, 0x47, 0xD6, 0xBE, 0x4C,
  0xD2, 0x3F, 0xE0, 0xA9, 0xBF, 0x07, 0xF5, 0x0B, 0x2F, 0x0E, 0xEA, 0x97, 0xBA, 0x4F, 0x8D, 0x74,
  0x1F, 0x0D, 0xEB, 0x52, 0x35, 0xBA, 0x78, 0x8F, 0x53, 0xD0, 0xF6, 0xE9, 0xB6, 0xF7, 0x00, 0xB6,
  0x60, 0x92, 0x74, 0x91, 0x83, 0x38, 0x0B, 0x92, 0x23, 0xDE, 0x00, 0x23, 0x24, 0x61, 0xB1, 0xF4,
  0xC7, 0xC5, 0xCF, 0xF9, 0x25, 0x3E, 0x34, 0xFF, 0x00, 0xB0, 0x2D, 0xEF, 0xFE, 0x88, 0x7A, 0xFC,
  0xC6, 0xF1, 0x46, 0x9B, 0x6D, 0x37, 0xFC, 0x12, 0x67, 0xE0, 0x4C, 0x4F, 0x0A, 0x34, 0x67, 0xC5,
  0x76, 0xAC, 0x54, 0x8E, 0x32, 0xD7, 0xD7, 0x81, 0xBF, 0x3D, 0xCD, 0xF9, 0xD0, 0x07, 0xDD, 0xFF,
  0x00, 0x03, 0xFF, 0x00, 0x6C, 0x5F, 0x06, 0xFC, 0x75, 0xF1, 0xEE, 0xAD, 0xE0, 0xCB, 0x0D, 0x1B,
  0xC5, 0x1E, 0x17, 0xF1, 0x25, 0x85, 0x92, 0xEA, 0x69, 0xA7, 0xF8, 0xAF, 0x4A, 0x36, 0x12, 0x5E,
  0x59, 0x96, 0x0A, 0x2E, 0x21, 0x52, 0xC4, 0x94, 0xCB, 0x27, 0xDE, 0x0A, 0x4E, 0xE0, 0x40, 0xEB,
  0x8A, 0xBF, 0x13, 0xBF, 0x6C, 0xFF, 0x00, 0x0C, 0xFC, 0x38, 0xF1, 0xD6, 0xA9, 0xE0, 0xFB, 0x3F,
  0x04, 0xFC, 0x41, 0xF1, 0xEE, 0xBD, 0xA5, 0x79, 0x62, 0xFE, 0xDF, 0xC1, 0xDE, 0x1A, 0x96, 0xF9,
  0x6D, 0x99, 0xE3, 0x49, 0x50, 0x34, 0x84, 0xA2, 0x92, 0x51, 0xD4, 0xFC, 0xA4, 0xE3, 0x38, 0xEB,
  0xC5, 0x79, 0xDF, 0x8D, 0xCF, 0x93, 0xFF, 0x00, 0x05, 0x50, 0xF8, 0x74, 0xC9, 0xF2, 0xB4, 0xDF,
  0x0E, 0xEE, 0xE3, 0x90, 0x8F, 0xE2, 0x51, 0x73, 0x3B, 0x00, 0x7F, 0x1E, 0x6B, 0x1F, 0x46, 0xF8,
  0xA9, 0xF1, 0xCF, 0xF6, 0xB1, 0xF8, 0xB9, 0xF1, 0x2B, 0x48, 0xF8, 0x67, 0xE3, 0x6D, 0x1F, 0xE1,
  0x37, 0x81, 0x7C, 0x0F, 0xAC, 0x49, 0xA0, 0x36, 0xA7, 0x2E, 0x8B, 0x1E, 0xAD, 0xA8, 0x6A, 0x17,
  0x51, 0x96, 0x59, 0x1B, 0xCB, 0x94, 0x84, 0x54, 0xCA, 0x92, 0x3E, 0xE9, 0xC3, 0x2F, 0xDE, 0x39,
  0xC0, 0x07, 0xBB, 0xFC, 0x0A, 0xFD, 0xAC, 0x3C, 0x0B, 0xFB, 0x40, 0x6A, 0xDA, 0xBE, 0x87, 0xA2,
  0x8D, 0x5B, 0x42, 0xF1, 0x66, 0x90, 0x82, 0x5D, 0x43, 0xC3, 0x1E, 0x25, 0xB0, 0x6B, 0x1D, 0x4A,
  0xDA, 0x32, 0x40, 0x0E, 0xD1, 0x36, 0x41, 0x5C, 0xB2, 0x82, 0x54, 0x9C, 0x6E, 0x5C, 0xE3, 0x70,
  0xCE, 0x1F, 0xC6, 0x0F, 0xDB, 0x67, 0xC0, 0xBF, 0x09, 0x3E, 0x21, 0xAF, 0x80, 0xE1, 0xD2, 0x3C,
  0x55, 0xE3, 0xEF, 0x1A, 0xAC, 0x22, 0xE6, 0xE7, 0x42, 0xF0, 0x4E, 0x92, 0x75, 0x1B, 0x9B, 0x38,
  0x88, 0x04, 0x34, 0xA3, 0x72, 0x85, 0xC8, 0x20, 0xE0, 0x12, 0x40, 0x20, 0x90, 0x03, 0x0C, 0xFC,
  0xC1, 0xF0, 0xBB, 0xC3, 0xDE, 0x30, 0xF0, 0x57, 0xFC, 0x15, 0x43, 0x4B, 0xD2, 0xBC, 0x69, 0xF1,
  0x21, 0x3E, 0x26, 0x78, 0x80, 0xF8, 0x1A, 0x5F, 0xB4, 0xEA, 0xB1, 0xE8, 0x96, 0xFA, 0x49, 0x89,
  0x0B, 0xBB, 0x25, 0xBB, 0xC3, 0x01, 0x2A, 0xC4, 0x00, 0xAF, 0xBC, 0xF2, 0x43, 0xA8, 0xE8, 0xA2,
  0xBB, 0xCF, 0xF8, 0x27, 0x2C, 0x71, 0xEA, 0x1F, 0x15, 0x7F, 0x6A, 0x3D, 0x5F, 0x52, 0x01, 0xBC,
  0x56, 0xFE, 0x3C, 0xB8, 0xB6, 0xBB, 0x69, 0x39, 0x92, 0x38, 0x12, 0x49, 0xBC, 0xA4, 0xF5, 0x0A,
  0x0F, 0x98, 0x00, 0xE9, 0xF2, 0x8F, 0x4A, 0x00, 0xF6, 0x0F, 0x84, 0x5F, 0xB7, 0x1F, 0xC3, 0xAF,
  0x8D, 0xBF, 0x16, 0x22, 0xF8, 0x7B, 0xE1, 0xA8, 0x35, 0xC6, 0xD7, 0x7F, 0xB2, 0x64, 0xD5, 0xAE,
  0x1A, 0xFA, 0xC9, 0x6D, 0xD2, 0xCF, 0xCB, 0x97, 0xCA, 0x92, 0xDA, 0x65, 0x67, 0xF3, 0x52, 0x75,
  0x6E, 0xAA, 0x53, 0x18, 0x20, 0x86, 0x20, 0x8C, 0xF4, 0x7F, 0x1E, 0xFF, 0x00, 0x6A, 0xBF, 0x00,
  0xFE, 0xCE, 0xB2, 0xE9, 0x36, 0x3E, 0x24, 0xB9, 0xBE, 0xD4, 0x7C, 0x45, 0xAB, 0x9C, 0x69, 0xBE,
  0x1C, 0xD0, 0xAD, 0x1A, 0xF3, 0x51, 0xBC, 0xE7, 0x19, 0x48, 0x97, 0x18, 0x19, 0xC8, 0x05, 0x88,
  0x04, 0x82, 0x06, 0x48, 0x35, 0xF3, 0x96, 0x83, 0xA7, 0x69, 0x76, 0x1F, 0xF0, 0x58, 0x1F, 0x10,
  0x3E, 0x9C, 0x91, 0x25, 0xC5, 0xCF, 0x80, 0x84, 0xF7, 0xE2, 0x30, 0x01, 0x33, 0xEF, 0x85, 0x72,
  0xDE, 0xE6, 0x35, 0x8B, 0xF4, 0xAF, 0x3B, 0x6F, 0x12, 0x7C, 0x46, 0xD1, 0xFF, 0x00, 0xE0, 0xA6,
  0xDF, 0x19, 0x75, 0xEF, 0x0C, 0xFC, 0x28, 0x1F, 0x16, 0x35, 0xCD, 0x23, 0x4A, 0xB2, 0xB2, 0xB1,
  0xB4, 0x9B, 0xC4, 0x36, 0xDA, 0x4F, 0xF6, 0x65, 0xA4, 0x96, 0xF0, 0x1F, 0x36, 0x23, 0x70, 0xA4,
  0x36, 0xFF, 0x00, 0x98, 0x1D, 0x9D, 0x3C, 0xC7, 0xCF, 0xDE, 0xA0, 0x0F, 0xAE, 0x7E, 0x0D, 0x7E,
  0xD9, 0x7E, 0x08, 0xF8, 0xC5, 0xE3, 0x89, 0x7C, 0x14, 0xDA, 0x57, 0x89, 0xFC, 0x09, 0xE3, 0x51,
  0x01, 0xBA, 0x8B, 0xC3, 0xBE, 0x36, 0xD2, 0x5B, 0x4D, 0xBC, 0xB9, 0x84, 0x64, 0x97, 0x89, 0x4B,
  0x30, 0x70, 0x00, 0x27, 0x19, 0xCE, 0x01, 0x38, 0xC0, 0x24, 0x7B, 0xCD, 0x7E, 0x7E, 0x7C, 0x56,
  0xD1, 0xFF, 0x00, 0x68, 0xBF, 0x8F, 0x3F, 0x18, 0x7E, 0x0A, 0xF8, 0x96, 0xE7, 0xF6, 0x79, 0x1E,
  0x01, 0x97, 0xC1, 0x7E, 0x25, 0x86, 0xF6, 0xE3, 0x5A, 0x5F, 0x1A, 0x69, 0xB7, 0xEE, 0xD6, 0x2F,
  0x22, 0x0B, 0x98, 0x8A, 0x29, 0x8D, 0xB6, 0x94, 0x52, 0x48, 0x1B, 0x89, 0xE4, 0x01, 0xCD, 0x7D,
  0x8B, 0xF1, 0xEB, 0xE2, 0xF6, 0x99, 0xF0, 0x1B, 0xE0, 0xFF, 0x00, 0x8A, 0xBC, 0x79, 0xAB, 0x6D,
  0x6B, 0x6D, 0x16, 0xC9, 0xE7, 0x48, 0x59, 0xB1, 0xE7, 0xCC, 0x7E, 0x58, 0x61, 0x07, 0xD5, 0xE4,
  0x64, 0x4F, 0xF8, 0x15, 0x00, 0x7C, 0xC1, 0xF1, 0x73, 0xFE, 0x32, 0xB3, 0xF6, 0xE7, 0xF0, 0x97,
  0xC3, 0x38, 0xBF, 0xD2, 0xBC, 0x07, 0xF0, 0xA9, 0x13, 0xC4, 0xFE, 0x23, 0x03, 0x98, 0xAE, 0x35,
  0x36, 0x03, 0xEC, 0x96, 0xED, 0xD8, 0xED, 0x05, 0x4E, 0x3B, 0x86, 0x9D, 0x4F, 0x4A, 0xFA, 0xC3,
  0xE2, 0xAF, 0xC4, 0x2F, 0xF8, 0x55, 0x9E, 0x06, 0xBF, 0xF1, 0x2F, 0xFC, 0x23, 0x3E, 0x22, 0xF1,
  0x7F, 0xD9, 0x1A, 0x25, 0xFE, 0xC7, 0xF0, 0xAD, 0x87, 0xDB, 0x75, 0x09, 0xB7, 0xC8, 0xA9, 0x98,
  0xE1, 0xDC, 0xBB, 0x82, 0xEE, 0xDC, 0xDC, 0xF0, 0xAA, 0xC7, 0xB5, 0x78, 0x67, 0xFC, 0x13, 0xDF,
  0xE1, 0x16, 0xA7, 0xE0, 0x0F, 0x82, 0x4F, 0xE2, 0xEF, 0x15, 0x6E, 0x9B, 0xC7, 0xDF, 0x10, 0xEF,
  0x1F, 0xC5, 0x1A, 0xE5, 0xC4, 0xAB, 0x89, 0x33, 0x39, 0x2D, 0x0C, 0x47, 0xB8, 0xDA, 0x8D, 0xBB,
  0x69, 0xFB, 0xAD, 0x23, 0x8A, 0xFA, 0x82, 0x80, 0x3E, 0x4A, 0xFD, 0x9C, 0x3E, 0x32, 0x7C, 0x2A,
  0xF8, 0xBD, 0xFB, 0x4F, 0x78, 0xE6, 0xFB, 0x47, 0xF8, 0x67, 0xE3, 0x7F, 0x02, 0xFC, 0x5A, 0x3A,
  0x24, 0x0D, 0xAD, 0xCF, 0xE3, 0x1B, 0x46, 0xB2, 0x92, 0x4B, 0x35, 0x30, 0xAC, 0x49, 0xF6, 0x7F,
  0xB4, 0xB8, 0x43, 0x8F, 0x28, 0x82, 0x23, 0x5C, 0x81, 0x9C, 0x9C, 0xF3, 0xED, 0x1F, 0xB4, 0x57,
  0xED, 0x03, 0xE1, 0xEF, 0xD9, 0x97, 0xE1, 0x8D, 0xDF, 0x8E, 0xBC, 0x51, 0x67, 0xA9, 0xDF, 0xE9,
  0x16, 0xD7, 0x10, 0xDB, 0x3C, 0x3A, 0x44, 0x51, 0xC9, 0x39, 0x69, 0x1B, 0x6A, 0x90, 0xB2, 0x48,
  0x8B, 0x8C, 0xF5, 0xF9, 0xBF, 0x3A, 0xF9, 0xD7, 0xE1, 0x2F, 0xFC, 0xA5, 0x5B, 0xE3, 0x8F, 0xFD,
  0x8A, 0x1A, 0x77, 0xFE, 0x81, 0x67, 0x56, 0xBF, 0xE0, 0xAE, 0x1F, 0xF2, 0x66, 0x1A, 0xE7, 0xFD,
  0x85, 0x74, 0xFF, 0x00, 0xFD, 0x1C, 0x28, 0x03, 0x7F, 0x56, 0xFF, 0x00, 0x82, 0x93, 0xFC, 0x2E,
  0xD1, 0xE6, 0x5B, 0xD9, 0x74, 0x2F, 0x1C, 0xCB, 0xE0, 0xA3, 0x70, 0x2D, 0x8F, 0x8E, 0xE1, 0xF0,
  0xEC, 0x8D, 0xA0, 0x87, 0xDD, 0xB4, 0xE2, 0xE3, 0x76, 0x5C, 0x06, 0xC8, 0xCA, 0x23, 0x67, 0x1C,
  0x67, 0x8A, 0xFA, 0x6E, 0x0F, 0x13, 0xE9, 0x17, 0x3E, 0x1A, 0x8F, 0xC4, 0x31, 0x6A, 0x76, 0x8F,
  0xA0, 0xC9, 0x68, 0x2F, 0xD7, 0x53, 0x13, 0x2F, 0xD9, 0xCD, 0xB9, 0x4D, 0xE2, 0x6F, 0x33, 0x3B,
  0x76, 0x6D, 0xF9, 0xB7, 0x67, 0x18, 0xE6, 0xBC, 0xBF, 0xC7, 0xDE, 0x1D, 0xF0, 0xC2, 0xFE, 0xC8,
  0x9A, 0xFE, 0x8E, 0x21, 0xB6, 0x1E, 0x11, 0x8B, 0xC1, 0x53, 0x45, 0x1A, 0x00, 0x3C, 0xA5, 0xB5,
  0x5B, 0x23, 0xB1, 0x87, 0x61, 0x85, 0x00, 0x83, 0xDB, 0x00, 0xD7, 0xC2, 0xD7, 0x5A, 0xFF, 0x00,
  0x89, 0x53, 0xFE, 0x08, 0x8B, 0x6D, 0x3C, 0x72, 0x4E, 0x27, 0x68, 0xBE, 0xC5, 0x24, 0xA0, 0x9D,
  0xE2, 0xC7, 0xFB, 0x65, 0xA3, 0x03, 0xFD, 0xDD, 0x9B, 0x53, 0xFD, 0xC3, 0xE9, 0x40, 0x1F, 0x4C,
  0x5D, 0x7F, 0xC1, 0x4B, 0xBE, 0x19, 0x31, 0xBF, 0xBE, 0xD1, 0xBC, 0x2F, 0xF1, 0x07, 0xC5, 0x3E,
  0x13, 0xD3, 0xE4, 0x68, 0xEE, 0xFC, 0x61, 0xA2, 0x78, 0x6A, 0x49, 0xB4, 0x88, 0x36, 0xFD, 0xE6,
  0x69, 0x99, 0x95, 0x82, 0x8E, 0xB9, 0xDB, 0xD3, 0x91, 0x9A, 0xFA, 0x5B, 0xC0, 0xFE, 0x3B, 0xD0,
  0x3E, 0x24, 0x78, 0x3F, 0x4C, 0xF1, 0x4F, 0x86, 0xB5, 0x48, 0x35, 0x7F, 0x0F, 0xEA, 0x50, 0x0B,
  0x8B, 0x5B, 0xE8, 0x0F, 0xC9, 0x22, 0x72, 0x0F, 0x5C, 0x15, 0x20, 0x82, 0x0A, 0x90, 0x0A, 0x90,
  0x41, 0x00, 0x82, 0x2B, 0xE3, 0xDF, 0x83, 0x9F, 0x14, 0x7E, 0x3E, 0x78, 0x1B, 0xE0, 0xF7, 0x84,
  0x7C, 0x33, 0xE1, 0xDF, 0xD9, 0x1A, 0x0B, 0xAF, 0x0E, 0xD9, 0x69, 0x16, 0xF6, 0xF6, 0xD2, 0x27,
  0xC4, 0x4D, 0x2D, 0x12, 0xE6, 0x3F, 0x2C, 0x7E, 0xF4, 0xAF, 0x96, 0x79, 0x93, 0x25, 0xCE, 0x72,
  0x49, 0x73, 0x9C, 0xE6, 0xB9, 0x9F, 0x83, 0xFF, 0x00, 0x0D, 0xFE, 0x28, 0xFC, 0x08, 0xFD, 0x80,
  0x3F, 0x68, 0x1D, 0x27, 0xC4, 0xFE, 0x1A, 0x93, 0xC1, 0xB7, 0xE1, 0x75, 0xCD, 0x4B, 0x43, 0xD3,
  0x23, 0xD4, 0x60, 0xBD, 0x36, 0x76, 0x52, 0xDA, 0x2B, 0x6D, 0x49, 0x20, 0x76, 0x1F, 0x23, 0x09,
  0x48, 0xFB, 0xA7, 0x39, 0x38, 0x19, 0x14, 0x01, 0xEB, 0x97, 0x1F, 0xF0, 0x52, 0x0F, 0x85, 0xEF,
  0xAE, 0x6A, 0xB0, 0x69, 0x1A, 0x27, 0x8D, 0xFC, 0x51, 0xE1, 0xBD, 0x22, 0x76, 0x83, 0x52, 0xF1,
  0x96, 0x83, 0xE1, 0xE9, 0x2E, 0xF4, 0x5B, 0x32, 0xBF, 0x7D, 0xA4, 0x9D, 0x5B, 0x76, 0xD5, 0xEB,
  0xB9, 0x50, 0x82, 0x39, 0x19, 0x04, 0x13, 0xEA, 0x1F, 0xB3, 0xC7, 0xED, 0x35, 0xE1, 0x1F, 0xDA,
  0x6F, 0x4B, 0xF1, 0x36, 0xA5, 0xE0, 0xE8, 0xB5, 0x01, 0x61, 0xA0, 0xEA, 0xF2, 0x68, 0xF2, 0xDC,
  0x5E, 0xA4, 0x4A, 0x97, 0x32, 0x22, 0xAB, 0x79, 0xB0, 0x98, 0xE4, 0x7D, 0xD1, 0x30, 0x60, 0x43,
  0x1D, 0xA4, 0xFA, 0x0A, 0xE3, 0x7F, 0xE0, 0x9F, 0x1A, 0x4E, 0x91, 0x65, 0xFB, 0x17, 0xFC, 0x2F,
  0x87, 0x4B, 0x8A, 0x13, 0x6B, 0x71, 0xA5, 0x19, 0x6E, 0x02, 0x28, 0x22, 0x49, 0xDE, 0x47, 0x33,
  0xEE, 0xF5, 0x3B, 0xCB, 0x83, 0x9F, 0x4C, 0x76, 0xAF, 0x1F, 0xFF, 0x00, 0x82, 0x50, 0x59, 0x69,
  0xDA, 0x6F, 0x84, 0xBE, 0x35, 0x5A, 0x68, 0xE1, 0x06, 0x91, 0x6F, 0xE3, 0xFB, 0xD8, 0xAC, 0xC4,
  0x7F, 0x77, 0xC8, 0x58, 0xE3, 0x11, 0xE3, 0xDB, 0x68, 0x14, 0x01, 0xF7, 0x65, 0x14, 0x51, 0x40,
  0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40,
  0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40,
  0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40, 0x05, 0x14, 0x51, 0x40,
  0x05, 0x7C, 0xA7, 0xF1, 0x9B, 0xF6, 0x34, 0xF1, 0x97, 0x8E, 0x7F, 0x69, 0x08, 0xBE, 0x31, 0xF8,
  0x13, 0xE3, 0x07, 0xFC, 0x2B, 0x8D, 0x7E, 0x3D, 0x1D, 0x34, 0x65, 0x8F, 0xFE, 0x11, 0x88, 0x75,
  0x4F, 0xDD, 0x82, 0xC5, 0x8E, 0x66, 0x98, 0x2F, 0xCD, 0xB8, 0x71, 0xB3, 0x23, 0x1D, 0x79, 0xAF,
  0xAB, 0x28, 0xA0, 0x0F, 0x9A, 0x7C, 0x2D, 0xF0, 0x1F, 0xF6, 0x8B, 0xD2, 0x7C, 0x4F, 0xA4, 0x5F,
  0x6B, 0x5F, 0xB5, 0x1F, 0xF6, 0xF6, 0x8F, 0x6D, 0x79, 0x0C, 0xD7, 0xBA, 0x57, 0xFC, 0x2B, 0xED,
  0x3A, 0xDB, 0xED, 0xB0, 0x2B, 0x83, 0x24, 0x3E, 0x6A, 0xBE, 0xE8, 0xF7, 0xA8, 0x2B, 0xBD, 0x79,
  0x5C, 0xE4, 0x72, 0x2B, 0x6F, 0xE3, 0xA7, 0xEC, 0xD3, 0xE2, 0x4F, 0x89, 0x9E, 0x3F, 0xD3, 0xBC,
  0x6D, 0xE0, 0xCF, 0x8B, 0xBE, 0x23, 0xF8, 0x63, 0xE2, 0x0B, 0x4D, 0x3D, 0x74, 0xD7, 0x4D, 0x3A,
  0x28, 0xEE, 0xAC, 0x6E, 0x62, 0x59, 0x24, 0x91, 0x4C, 0xB6, 0xCE, 0x40, 0x76, 0x06, 0x56, 0xE5,
  0x89, 0x18, 0xC6, 0x00, 0xE7, 0x3E, 0xF7, 0x45, 0x00, 0x7C, 0x91, 0x6F, 0xFB, 0x0A, 0xEB, 0xDE,
  0x3D, 0xF1, 0x97, 0x87, 0xF5, 0xCF, 0x8D, 0xDF, 0x19, 0xB5, 0x9F, 0x8B, 0x76, 0x7A, 0x05, 0xD2,
  0xDF, 0x58, 0x68, 0x5F, 0xD9, 0x30, 0x69, 0x3A, 0x77, 0x9E, 0xBF, 0x75, 0xE5, 0x86, 0x16, 0x61,
  0x26, 0x3F, 0x02, 0x79, 0x04, 0x95, 0x24, 0x1F, 0x41, 0xFD, 0xAA, 0xFF, 0x00, 0x65, 0xFF, 0x00,
  0xF8, 0x69, 0xBB, 0x3F, 0x02, 0xC1, 0xFF, 0x00, 0x09, 0x2F, 0xFC, 0x23, 0x7F, 0xF0, 0x8B, 0xEB,
  0xF0, 0xEB, 0x9B, 0xBE, 0xC1, 0xF6, 0xAF, 0xB4, 0xF9, 0x60, 0x8F, 0x2B, 0xFD, 0x6A, 0x6C, 0xCE,
  0x7E, 0xF7, 0xCD, 0xF4, 0xAF, 0x76, 0xA2, 0x80, 0x0A, 0xF3, 0xDF, 0x8D, 0x7F, 0x0C, 0x75, 0x9F,
  0x8A, 0x5E, 0x1B, 0xB2, 0xB2, 0xD0, 0x3C, 0x7D, 0xAE, 0xFC, 0x3B, 0xD5, 0x6C, 0xAE, 0xC5, 0xE4,
  0x5A, 0x9E, 0x86, 0x51, 0x8C, 0x84, 0x23, 0xAF, 0x95, 0x34, 0x6E, 0x0A, 0xCB, 0x19, 0xDF, 0xBB,
  0x69, 0xC7, 0x2A, 0xA7, 0x3C, 0x57, 0xA1, 0x51, 0x40, 0x1F, 0x1E, 0x78, 0xA7, 0xF6, 0x1D, 0xF8,
  0x93, 0xF1, 0x6E, 0xC5, 0x34, 0x0F, 0x8A, 0x7F, 0xB4, 0x8F, 0x88, 0x3C, 0x5D, 0xE0, 0xA6, 0x91,
  0x5E, 0xE7, 0x43, 0xD3, 0x3C, 0x3F, 0x69, 0xA4, 0x35, 0xD2, 0xAB, 0x06, 0x09, 0x2C, 0xD1, 0x33,
  0x6F, 0x5C, 0x80, 0x70, 0xCA, 0x7A, 0x03, 0xC1, 0x00, 0xD7, 0xA2, 0xFE, 0xD1, 0x7F, 0xB2, 0xA9,
  0xF8, 0xC3, 0xFB, 0x3D, 0xD8, 0x7C, 0x23, 0xF0, 0x6F, 0x8A, 0xDF, 0xE1, 0x8E, 0x83, 0x6C, 0xD6,
  0xF0, 0x31, 0xB2, 0xB2, 0x37, 0x4B, 0x2D, 0x8C, 0x51, 0xB2, 0x8B, 0x46, 0x5F, 0x36, 0x33, 0xB4,
  0x9F, 0x2D, 0x89, 0x2C, 0x49, 0xF2, 0xF0, 0x41, 0xDC, 0x6B, 0xDF, 0xA8, 0xA0, 0x0F, 0x8F, 0xBC,
  0x25, 0xFB, 0x25, 0x7E, 0xD0, 0x9E, 0x03, 0xF0, 0xCE, 0x9B, 0xE1, 0xEF, 0x0F, 0x7E, 0xD4, 0xF6,
  0xFA, 0x46, 0x89, 0xA6, 0xC0, 0xB6, 0xF6, 0x96, 0x56, 0xBF, 0x0C, 0xB4, 0xB5, 0x8E, 0x28, 0xC7,
  0x40, 0x07, 0x99, 0xF8, 0x92, 0x79, 0x24, 0x92, 0x72, 0x4D, 0x7D, 0x43, 0xE0, 0x1D, 0x1B, 0x5D,
  0xF0, 0xF7, 0x83, 0x74, 0x9D, 0x37, 0xC4, 0xDE, 0x22, 0xFF, 0x00, 0x84, 0xB7, 0x5F, 0xB6, 0x80,
  0x25, 0xE6, 0xB7, 0xF6, 0x18, 0xEC, 0xBE, 0xD9, 0x26, 0x4E, 0x64, 0xF2, 0x23, 0x25, 0x63, 0xED,
  0xC0, 0x38, 0xE2, 0xBA, 0x0A, 0x28, 0x00, 0xA2, 0x8A, 0x28, 0x00, 0xAC, 0xFF, 0x00, 0x10, 0xE8,
  0x36, 0x1E, 0x2A, 0xD0, 0x35, 0x2D, 0x17, 0x55, 0xB7, 0x17, 0x7A, 0x5E, 0xA5, 0x6D, 0x2D, 0x9D,
  0xDD, 0xB9, 0x62, 0xA2, 0x58, 0x64, 0x42, 0x8E, 0xB9, 0x52, 0x08, 0xCA, 0xB1, 0x19, 0x04, 0x1E,
  0x6B, 0x42, 0x8A, 0x00, 0xE6, 0xBC, 0x03, 0xF0, 0xD3, 0xC2, 0x7F, 0x0A, 0xF4, 0x24, 0xD1, 0xBC,
  0x1F, 0xE1, 0xCD, 0x33, 0xC3, 0x5A, 0x5A, 0x60, 0xFD, 0x9B, 0x4C, 0xB5, 0x48, 0x55, 0x8F, 0xF7,
  0x9B, 0x68, 0xCB, 0x37, 0xFB, 0x4D, 0x92, 0x7D, 0x6B, 0xA5, 0xA2, 0x8A, 0x00, 0x28, 0xA2, 0x8A,
  0x00, 0xF9, 0xEB, 0xF6, 0x90, 0xFD, 0x9F, 0xBE, 0x2A, 0x7C, 0x6C, 0x9B, 0x55, 0xD3, 0x7C, 0x39,
  0xF1, 0xC3, 0xFE, 0x10, 0x3F, 0x05, 0x6A, 0xDA, 0x4B, 0xE9, 0x57, 0xFE, 0x1D, 0xFF, 0x00, 0x84,
  0x4A, 0xD7, 0x51, 0xF3, 0xD6, 0x45, 0x74, 0x99, 0xBE, 0xD0, 0xF2, 0x2C, 0x8B, 0xBD, 0x1C, 0x2E,
  0x14, 0x8D, 0xBB, 0x72, 0x0E, 0x4D, 0x73, 0x9F, 0xB3, 0x37, 0xEC, 0x9F, 0xF1, 0x53, 0xF6, 0x76,
  0x6F, 0x0B, 0xE8, 0x8F, 0xF1, 0xEF, 0xFE, 0x12, 0x3F, 0x87, 0x3A, 0x22, 0xCE, 0x9F, 0xF0, 0x89,
  0xFF, 0x00, 0xC2, 0x1D, 0x6B, 0x6B, 0xE7, 0x2C, 0x82, 0x46, 0x03, 0xED, 0x7E, 0x6B, 0xCA, 0xB8,
  0x96, 0x41, 0x27, 0x53, 0x9D, 0xBB, 0x7A, 0x1A, 0xFA, 0xA2, 0x8A, 0x00, 0xF0, 0x9F, 0x8C, 0x1F,
  0xB2, 0xFF, 0x00, 0xFC, 0x2D, 0x7F, 0xDA, 0x03, 0xE1, 0x47, 0xC4, 0xDF, 0xF8, 0x49, 0x7F, 0xB2,
  0xFF, 0x00, 0xE1, 0x04, 0x92, 0x77, 0xFE, 0xCB, 0xFB, 0x07, 0x9D, 0xF6, 0xEF, 0x33, 0x1C, 0x79,
  0xBE, 0x6A, 0xF9, 0x78, 0xC7, 0xF7, 0x5B, 0x3E, 0xD5, 0xEE, 0xD4, 0x51, 0x40, 0x1E, 0x13, 0xF1,
  0x83, 0xF6, 0x5F, 0xFF, 0x00, 0x85, 0xAF, 0xFB, 0x40, 0x7C, 0x28, 0xF8, 0x9B, 0xFF, 0x00, 0x09,
  0x2F, 0xF6, 0x5F, 0xFC, 0x20, 0x92, 0x4E, 0xFF, 0x00, 0xD9, 0x7F, 0x60, 0xF3, 0xBE, 0xDD, 0xE6,
  0x63, 0x8F, 0x37, 0xCD, 0x5F, 0x2F, 0x18, 0xFE, 0xEB, 0x67, 0xDA, 0xBD, 0xDA, 0x8A, 0x28, 0x00,
  0xA2, 0x8A, 0x28, 0x03, 0xE7, 0x7F, 0xDB, 0xB7, 0xC4, 0x3E, 0x38, 0xB2, 0xF8, 0x07, 0xA8, 0xF8,
  0x73, 0xE1, 0xCF, 0x87, 0xF5, 0x6D, 0x77, 0xC5, 0xBE, 0x2E, 0x9D, 0x34, 0x08, 0x24, 0xD3, 0x2D,
  0x25, 0x99, 0x6C, 0x21, 0x9F, 0x2B, 0x35, 0xC4, 0xCE, 0x8A, 0x44, 0x48, 0x23, 0xDC, 0xBB, 0xD8,
  0x8D, 0xA5, 0xC1, 0xFE, 0x13, 0x5E, 0x9B, 0xF0, 0x2F, 0xE1, 0x2E, 0x97, 0xF0, 0x2B, 0xE1, 0x17,
  0x85, 0xBC, 0x07, 0xA4, 0x00, 0x6C, 0xF4, 0x4B, 0x24, 0xB7, 0x69, 0x82, 0xED, 0x33, 0xCB, 0xF7,
  0xA5, 0x98, 0x8F, 0x57, 0x91, 0x9D, 0xCF, 0xBB, 0x57, 0x77, 0x45, 0x00, 0x14, 0x51, 0x45, 0x00,
  0x53, 0xD6, 0x6D, 0x6E, 0xEF, 0x74, 0x7B, 0xEB, 0x7B, 0x0B, 0xDF, 0xEC, 0xDB, 0xF9, 0xA0, 0x92,
  0x3B, 0x7B, 0xDF, 0x28, 0x4B, 0xF6, 0x79, 0x0A, 0x90, 0xB2, 0x6C, 0x3C, 0x36, 0xD3, 0x83, 0xB4,
  0xF0, 0x71, 0x8A, 0xF8, 0xAF, 0xE2, 0x0F, 0xEC, 0x23, 0xF1, 0xB3, 0xE2, 0xA6, 0xB3, 0xE1, 0x2D,
  0x5B, 0xC5, 0x1F, 0xB5, 0x0F, 0xF6, 0xA6, 0xA1, 0xE1, 0x4D, 0x45, 0x35, 0x6D, 0x1A, 0x6F, 0xF8,
  0x57, 0xF6, 0x50, 0xFD, 0x96, 0xE9, 0x59, 0x59, 0x64, 0xC4, 0x73, 0xA8, 0x7C, 0x14, 0x5F, 0x95,
  0xC3, 0x2F, 0x1D, 0x2B, 0xEE, 0x0A, 0x28, 0x03, 0xC5, 0x7E, 0x1D, 0x7C, 0x1E, 0xF8, 0x9B, 0xA5,
  0xF8, 0x47, 0xC6, 0x9A, 0x27, 0xC4, 0x7F, 0x8C, 0x5F, 0xF0, 0xB2, 0x9B, 0x5C, 0xB3, 0xFB, 0x1D,
  0x8D, 0xC7, 0xFC, 0x23, 0x16, 0xBA, 0x57, 0xF6, 0x6E, 0xE8, 0xE5, 0x49, 0x1B, 0x6C, 0x0F, 0xFB,
  0xED, 0xDB, 0xD0, 0xE1, 0x88, 0xC7, 0x97, 0xC1, 0xF9, 0x8D, 0x79, 0xDE, 0xA5, 0xFB, 0x09, 0xFF,
  0x00, 0x68, 0x7E, 0xC9, 0xFE, 0x04, 0xF8, 0x2B, 0xFF, 0x00, 0x09, 0xBF, 0x97, 0xFF, 0x00, 0x08,
  0xBE, 0xA9, 0x0E, 0xA5, 0xFD, 0xB9, 0xFD, 0x93, 0x9F, 0xB4, 0xF9, 0x73, 0xCB, 0x36, 0xCF, 0x23,
  0xCF, 0xF9, 0x33, 0xE6, 0xE3, 0x3B, 0xDB, 0x1B, 0x73, 0x8E, 0x70, 0x3E, 0xAE, 0xA2, 0x80, 0x3C,
  0x77, 0x5E, 0xFD, 0x9E, 0xBF, 0xB6, 0xFF, 0x00, 0x6A, 0x7F, 0x0C, 0xFC, 0x65, 0xFE, 0xDF, 0xF2,
  0x7F, 0xB1, 0x7C, 0x3F, 0x36, 0x85, 0xFD, 0x8B, 0xF6, 0x3D, 0xDE, 0x77, 0x99, 0x23, 0xBF, 0x9B,
  0xE7, 0x79, 0x83, 0x6E, 0x37, 0xE3, 0x6E, 0xC3, 0x9C, 0x75, 0xAF, 0x33, 0xF1, 0x17, 0xEC, 0x49,
  0xE2, 0x2D, 0x0F, 0xE2, 0xBF, 0x8A, 0x7C, 0x71, 0xF0, 0x7F, 0xE3, 0x1E, 0xAB, 0xF0, 0xA2, 0x4F,
  0x16, 0x4F, 0xF6, 0xBD, 0x77, 0x4B, 0x4D, 0x1E, 0xDF, 0x54, 0xB5, 0xB9, 0xB8, 0x24, 0x96, 0x96,
  0x34, 0x99, 0x80, 0x8D, 0x98, 0xB3, 0x31, 0x38, 0x63, 0x96, 0x6C, 0x10, 0x0E, 0x07, 0xD5, 0xD4,
  0x50, 0x07, 0xCA, 0x5F, 0x0A, 0x3F, 0x60, 0xBB, 0x5F, 0x85, 0x7F, 0xB4, 0x6E, 0x9F, 0xF1, 0x71,
  0x3C, 0x7B, 0xAA, 0x78, 0x8B, 0x55, 0xFE, 0xC7, 0x9A, 0xC7, 0x56, 0x1A, 0xCD, 0xBF, 0x9B, 0x73,
  0xA9, 0xDD, 0xC8, 0xEE, 0x5A, 0xE9, 0xA7, 0x12, 0x01, 0x18, 0x08, 0x63, 0x8D, 0x62, 0x58, 0xF0,
  0xAB, 0x12, 0x8D, 0xDD, 0x6A, 0x2F, 0x88, 0x3F, 0xB2, 0xAC, 0x5A, 0x87, 0xC7, 0xAD, 0x77, 0xC6,
  0xFF, 0x00, 0x08, 0xFE, 0x33, 0x4B, 0xF0, 0x97, 0xC7, 0xBA, 0x9C, 0x31, 0x2F, 0x88, 0xF4, 0xEB,
  0x3B, 0x5B, 0x6D, 0x4E, 0x1B, 0xC0, 0x17, 0x29, 0x2C, 0x96, 0x72, 0xB8, 0x08, 0xE5, 0x79, 0xDC,
  0x41, 0x07, 0x25, 0x80, 0x05, 0x98, 0xB7, 0xD6, 0x55, 0xE3, 0x5F, 0x18, 0x3F, 0x63, 0xAF, 0x83,
  0x7F, 0x1E, 0xF5, 0xC1, 0xAC, 0xF8, 0xEB, 0xC0, 0x96, 0x3A, 0xDE, 0xAE, 0x11, 0x63, 0x37, 0xEB,
  0x34, 0xD6, 0xB3, 0xBA, 0xAF, 0x0A, 0x19, 0xE1, 0x74, 0x2C, 0x00, 0xE0, 0x6E, 0x27, 0x03, 0x8A,
  0x00, 0xF9, 0x2B, 0xF6, 0x5A, 0xF8, 0x79, 0x69, 0xA6, 0xFF, 0x00, 0xC1, 0x48, 0xFC, 0x6F, 0x75,
  0xA3, 0x78, 0xA2, 0xFB, 0xC7, 0xFF, 0x00, 0xD8, 0x9E, 0x10, 0x10, 0xF8, 0x93, 0xC5, 0x77, 0x92,
  0xAC, 0xA6, 0xEB, 0x56, 0x9E, 0x78, 0xCE, 0xCC, 0xA7, 0xC8, 0x98, 0x8D, 0x76, 0x2C, 0x6B, 0xC2,
  0x08, 0x0A, 0xF5, 0x53, 0x5F, 0x51, 0x7C, 0x72, 0xFD, 0x94, 0x6D, 0x3E, 0x29, 0x78, 0xDB, 0x4E,
  0xF1, 0xFF, 0x00, 0x85, 0xBC, 0x5D, 0xAB, 0xFC, 0x33, 0xF8, 0x93, 0xA7, 0xDB, 0x1B, 0x18, 0xFC,
  0x47, 0xA2, 0x2A, 0x48, 0x2E, 0x6D, 0xB3, 0xBB, 0xC9, 0xB9, 0x81, 0xFE, 0x59, 0x90, 0x1E, 0x40,
  0x24, 0x76, 0xC9, 0x20, 0x00, 0x3D, 0x13, 0xE1, 0x57, 0xC1, 0x9F, 0x04, 0x7C, 0x0F, 0xF0, 0xD9,
  0xD0, 0x7C, 0x0B, 0xE1, 0xAB, 0x1F, 0x0D, 0x69, 0x6C, 0xFE, 0x6C, 0x90, 0xD9, 0xA1, 0xDD, 0x2B,
  0xE3, 0x1B, 0xE4, 0x76, 0x25, 0xA4, 0x6C, 0x00, 0x32, 0xC4, 0x9C, 0x00, 0x2B, 0xB4, 0xA0, 0x0F,
  0x9C, 0xBC, 0x0B, 0xFB, 0x2E, 0x78, 0xD9, 0x3C, 0x63, 0xA4, 0x78, 0x8B, 0xE2, 0x87, 0xC7, 0x2F,
  0x11, 0xFC, 0x49, 0x7D, 0x1E, 0xE1, 0x6E, 0xAC, 0x34, 0xAB, 0x6B, 0x18, 0x34, 0x4D, 0x3B, 0xCD,
  0x5F, 0xB8, 0xF3, 0xC1, 0x6E, 0x4F, 0x9C, 0x54, 0xF2, 0x37, 0x1C, 0x67, 0xA8, 0x35, 0xC5, 0x7E,
  0xD6, 0x9E, 0x07, 0xF1, 0x2F, 0xED, 0x1F, 0xF1, 0xFF, 0x00, 0xE1, 0x67, 0xC2, 0x61, 0xE1, 0xFD,
  0x55, 0x7E, 0x17, 0xE9, 0xD3, 0xFF, 0x00, 0xC2, 0x51, 0xE2, 0xAD, 0x65, 0xEC, 0xA5, 0x5B, 0x0B,
  0xA3, 0x16, 0x44, 0x16, 0x22, 0x62, 0xA1, 0x19, 0x89, 0xCE, 0xE4, 0x04, 0x9C, 0x4A, 0xAD, 0xD5,
  0x0E, 0x3E, 0xC2, 0xA2, 0x80, 0x11, 0x54, 0x2A, 0x85, 0x50, 0x00, 0x03, 0x00, 0x0E, 0xD4, 0xB4,
  0x57, 0x9A, 0xFC, 0x6F, 0xFD, 0x9C, 0xBE, 0x1D, 0xFE, 0xD1, 0xDA, 0x46, 0x9B, 0xA6, 0x7C, 0x44,
  0xF0, 0xF7, 0xFC, 0x24, 0x36, 0x3A, 0x74, 0xED, 0x73, 0x6B, 0x17, 0xDB, 0x6E, 0x2D, 0x7C, 0xB9,
  0x0A, 0xED, 0x2D, 0x98, 0x24, 0x42, 0x78, 0xE3, 0x04, 0x91, 0x40, 0x1F, 0x35, 0xFC, 0x07, 0xBE,
  0xB7, 0xF1, 0x3F, 0xFC, 0x14, 0xF7, 0xF6, 0x80, 0xD5, 0xB4, 0xA9, 0x92, 0xFF, 0x00, 0x4C, 0xB1,
  0xF0, 0xEE, 0x9B, 0xA7, 0x4F, 0x77, 0x6E, 0xC1, 0xE3, 0x4B, 0x8D, 0x96, 0xF9, 0x8F, 0x70, 0xE3,
  0x70, 0x31, 0x48, 0x08, 0xF5, 0x46, 0x1D, 0xAB, 0x47, 0xFE, 0x0A, 0xE1, 0xFF, 0x00, 0x26, 0x61,
  0xAE, 0x7F, 0xD8, 0x57, 0x4F, 0xFF, 0x00, 0xD1, 0xC2, 0xBE, 0x91, 0xF8, 0x3D, 0xF0, 0x27, 0xC0,
  0x5F, 0x00, 0xBC, 0x3B, 0x2E, 0x87, 0xE0, 0x0F, 0x0C, 0xDA, 0x78, 0x6F, 0x4E, 0x9A, 0x4F, 0x3A,
  0x64, 0x80, 0xBC, 0x92, 0x4C, 0xF8, 0xC0, 0x69, 0x25, 0x91, 0x99, 0xDC, 0x81, 0xC0, 0xDC, 0xC7,
  0x1D, 0xAA, 0xCF, 0xC5, 0xCF, 0x83, 0xBE, 0x10, 0xF8, 0xED, 0xE0, 0xB9, 0xFC, 0x27, 0xE3, 0x9D,
  0x23, 0xFB, 0x73, 0xC3, 0xF3, 0xCB, 0x1C, 0xF2, 0x59, 0xFD, 0xA6, 0x6B, 0x7D, 0xCE, 0x87, 0x72,
  0x1D, 0xF1, 0x3A, 0x37, 0x07, 0xDE, 0x80, 0x3E, 0x64, 0xD4, 0x7F, 0x60, 0xBF, 0x16, 0xF8, 0x8B,
  0xC1, 0xD6, 0xFE, 0x04, 0x5F, 0xDA, 0x03, 0xC5, 0x16, 0xBF, 0x07, 0x64, 0x89, 0x23, 0xFF, 0x00,
  0x84, 0x44, 0x69, 0xD6, 0xEF, 0x74, 0x96, 0xBC, 0x11, 0x6A, 0xBA, 0x81, 0x3E, 0x67, 0x94, 0x07,
  0xCA, 0x15, 0x95, 0x80, 0x50, 0x01, 0x07, 0x15, 0xF4, 0xED, 0xBF, 0xC2, 0x0F, 0x08, 0x5B, 0xFC,
  0x29, 0x4F, 0x86, 0xC3, 0x44, 0xB7, 0x7F, 0x05, 0x2E, 0x98, 0x34, 0x8F, 0xEC, 0xA7, 0xC9, 0x43,
  0x6D, 0xB3, 0x66, 0xD2, 0x73, 0x92, 0x71, 0xCE, 0xEC, 0xEE, 0xCF, 0x39, 0xCF, 0x35, 0xD7, 0x43,
  0x12, 0xC1, 0x12, 0x46, 0x83, 0x6A, 0x22, 0x85, 0x51, 0xE8, 0x07, 0x4A, 0x7D, 0x00, 0x7C, 0x93,
  0xA4, 0x7E, 0xC6, 0x3F, 0x13, 0xFC, 0x03, 0x63, 0x1F, 0x87, 0x7E, 0x1F, 0xFE, 0xD2, 0xDE, 0x26,
  0xF0, 0xDF, 0x81, 0x61, 0x1E, 0x5D, 0xB6, 0x8B, 0xA8, 0x68, 0x76, 0x9A, 0x9D, 0xC5, 0xAC, 0x3D,
  0xA2, 0x86, 0xEE, 0x42, 0x19, 0x15, 0x47, 0x0B, 0x85, 0xF9, 0x40, 0x15, 0xEE, 0xFF, 0x00, 0x0B,
  0x3E, 0x0D, 0x69, 0x7F, 0x0C, 0x3C, 0x03, 0x3F, 0x85, 0x9B, 0x52, 0xD5, 0xBC, 0x59, 0x1D, 0xEC,
  0x92, 0xCD, 0xA9, 0x6A, 0x3E, 0x25, 0xBB, 0x37, 0x97, 0x5A, 0x84, 0x92, 0x80, 0xB2, 0x34, 0xAC,
  0xC3, 0x04, 0x10, 0x02, 0xED, 0x00, 0x00, 0x06, 0x31, 0x5D, 0xFD, 0x14, 0x01, 0xF0, 0xE7, 0x86,
  0xBF, 0x64, 0x73, 0xF0, 0xF7, 0xC3, 0xFA, 0xE5, 0xBF, 0xC3, 0xAF, 0xDA, 0x7B, 0x52, 0xF0, 0x87,
  0xC1, 0x49, 0xA5, 0x9A, 0xE2, 0xF3, 0x45, 0xB6, 0x4B, 0x3B, 0xA1, 0x63, 0x1B, 0x12, 0x66, 0x48,
  0x35, 0x17, 0x72, 0x6D, 0xC7, 0xDE, 0xE4, 0x2E, 0x47, 0x52, 0x59, 0xB2, 0x4A, 0xFF, 0x00, 0xC1,
  0x26, 0x3C, 0x31, 0x6D, 0xA4, 0x7C, 0x1A, 0xF8, 0x81, 0xAB, 0x69, 0x36, 0xD2, 0x5A, 0xF8, 0x5F,
  0x58, 0xF1, 0xA5, 0xF4, 0xDA, 0x10, 0x97, 0x76, 0x64, 0xB1, 0x8D, 0x63, 0x8E, 0x37, 0x05, 0xB9,
  0x23, 0x2A, 0xCB, 0x93, 0xCE, 0x50, 0xE6, 0xBD, 0x67, 0xC4, 0x1F, 0xF0, 0x4F, 0x4F, 0xD9, 0xDF,
  0xC5, 0x1E, 0x25, 0x97, 0x5E, 0xD4, 0x3E, 0x17, 0xE9, 0x8D, 0xA8, 0xCB, 0x29, 0x9A, 0x43, 0x6F,
  0x73, 0x73, 0x6F, 0x0B, 0xB9, 0x39, 0x24, 0xC3, 0x1C, 0xAB, 0x19, 0xC9, 0xEA, 0x36, 0xD7, 0xBD,
  0x68, 0x3A, 0x06, 0x99, 0xE1, 0x6D, 0x16, 0xCB, 0x48, 0xD1, 0xB4, 0xFB, 0x6D, 0x2B, 0x4A, 0xB2,
  0x89, 0x60, 0xB6, 0xB2, 0xB3, 0x89, 0x62, 0x86, 0x18, 0xC0, 0xC0, 0x54, 0x45, 0x00, 0x28, 0x1E,
  0x82, 0x80, 0x2F, 0xD1, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51,
  0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x00, 0x51, 0x45, 0x14, 0x01, 0xFF,
  0xD9,
};
static const size_t IPP_TEST_JPEG_LEN = sizeof(IPP_TEST_JPEG);

static void ippAppendU8(std::vector<uint8_t>& v, uint8_t b) {
  v.push_back(b);
}

static void ippAppendU16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back((uint8_t)((x >> 8) & 0xFF));
  v.push_back((uint8_t)(x & 0xFF));
}

static void ippAppendU32(std::vector<uint8_t>& v, uint32_t x) {
  v.push_back((uint8_t)((x >> 24) & 0xFF));
  v.push_back((uint8_t)((x >> 16) & 0xFF));
  v.push_back((uint8_t)((x >> 8) & 0xFF));
  v.push_back((uint8_t)(x & 0xFF));
}

// [F24] Eliminado helper no usado: ippAppendBytes
static void ippAppendStringRaw(std::vector<uint8_t>& v, const String& s) {
  for (size_t i = 0; i < s.length(); i++) v.push_back((uint8_t)s[i]);
}

static void ippAppendAttr(std::vector<uint8_t>& v, uint8_t tag, const char* name, const String& value) {
  ippAppendU8(v, tag);
  uint16_t nameLen = (uint16_t)strlen(name);
  ippAppendU16(v, nameLen);
  ippAppendStringRaw(v, String(name));
  ippAppendU16(v, (uint16_t)value.length());
  ippAppendStringRaw(v, value);
}

static void ippAppendAttrEnum(std::vector<uint8_t>& v, const char* name, uint16_t value) {
  ippAppendU8(v, 0x23); // enum
  uint16_t nameLen = (uint16_t)strlen(name);
  ippAppendU16(v, nameLen);
  ippAppendStringRaw(v, String(name));
  ippAppendU16(v, 4);
  ippAppendU32(v, (uint32_t)value);
}

static uint16_t ippOrientationRequestedValue(uint8_t profileId) {
  return (getVirtualPrinterEffectiveOrientation(profileId) == VP_ORIENT_LANDSCAPE) ? 4 : 3;
}

bool printerSendIppDocumentJob(const std::vector<uint8_t>& doc, const char* jobName, const char* documentFormat, const char* logTag) {
  g_prnLastBytes = 0;
  g_prnLastError = "";

  if (doc.empty()) {
    g_prnLastError = "Documento de impresión vacío";
    return false;
  }
  if (!printerEnsureStaReady()) return false;

  String host = String(PRN_CFG.ip);
  uint16_t port = PRN_CFG.port;
  if (port == 0 || port == 9100) port = 631;

  String path = "/ipp/print";
  String printerUri = "ipp://" + host + ":" + String(port) + path;

  std::vector<uint8_t> ipp;
  try {
    ipp.reserve(512);
  } catch (...) {
    g_prnLastError = "Sin memoria reservando header IPP";
    return false;
  }

  ippAppendU8(ipp, 0x02);
  ippAppendU8(ipp, 0x00);
  ippAppendU16(ipp, 0x0002); // Print-Job
  ippAppendU32(ipp, 4);

  ippAppendU8(ipp, 0x01); // operation-attributes-tag
  ippAppendAttr(ipp, 0x47, "attributes-charset", "utf-8");
  ippAppendAttr(ipp, 0x48, "attributes-natural-language", "en");
  ippAppendAttr(ipp, 0x45, "printer-uri", printerUri);
  ippAppendAttr(ipp, 0x42, "requesting-user-name", "esp32-atari");
  ippAppendAttr(ipp, 0x42, "job-name", String(jobName ? jobName : "ATARI ATASCII PRINT"));
  ippAppendAttr(ipp, 0x49, "document-format", String(documentFormat ? documentFormat : "image/jpeg"));
  ippAppendAttr(ipp, 0x44, "media", virtualPaperMediaKeyword(PRN_CFG.paperSize));
  ippAppendAttr(ipp, 0x44, "print-scaling", "fit");
  ippAppendAttrEnum(ipp, "orientation-requested", ippOrientationRequestedValue(PRN_CFG.virtualProfile));
  ippAppendU8(ipp, 0x03); // end-of-attributes-tag

  const size_t contentLen = ipp.size() + doc.size();
  logf("[%s] stream connect host=%s port=%u ipp=%lu doc=%lu total=%lu format=%s freeHeap=%lu maxAlloc=%lu",
       logTag ? logTag : "IPP-DOC",
       host.c_str(), port,
       (unsigned long)ipp.size(),
       (unsigned long)doc.size(),
       (unsigned long)contentLen,
       documentFormat ? documentFormat : "?",
       (unsigned long)ESP.getFreeHeap(),
       (unsigned long)ESP.getMaxAllocHeap());

  WiFiClient client;
  client.setTimeout(45000);
  if (!client.connect(host.c_str(), port)) {
    g_prnLastError = "No conecta IPP a " + host + ":" + String(port) + " desde STA " + WiFi.localIP().toString();
    return false;
  }

  client.print("POST " + path + " HTTP/1.1\r\n");
  client.print("Host: " + host + ":" + String(port) + "\r\n");
  client.print("Content-Type: application/ipp\r\n");
  client.print("Content-Length: " + String((unsigned long)contentLen) + "\r\n");
  client.print("Connection: close\r\n\r\n");

  size_t written = 0;
  size_t w = client.write(ipp.data(), ipp.size());
  written += w;
  if (w != ipp.size()) {
    client.stop();
    g_prnLastError = "Fallo enviando header IPP";
    logf("[%s] ERROR write ipp %lu/%lu", logTag ? logTag : "IPP-DOC", (unsigned long)w, (unsigned long)ipp.size());
    return false;
  }

  const size_t CHUNK = 1024;
  for (size_t pos = 0; pos < doc.size(); pos += CHUNK) {
    size_t n = doc.size() - pos;
    if (n > CHUNK) n = CHUNK;
    w = client.write(&doc[pos], n);
    written += w;
    if (w != n) {
      client.stop();
      g_prnLastError = "Fallo enviando documento IPP";
      logf("[%s] ERROR write doc pos=%lu wrote=%lu/%lu totalWritten=%lu",
           logTag ? logTag : "IPP-DOC", (unsigned long)pos, (unsigned long)w, (unsigned long)n, (unsigned long)written);
      return false;
    }
    delay(0);
  }

  unsigned long t0 = millis();
  while (client.connected() && !client.available() && (millis() - t0 < 45000UL)) delay(10);

  if (!client.available()) {
    client.stop();
    g_prnLastBytes = (uint32_t)contentLen;
    g_prnLastError = "Timeout esperando respuesta IPP";
    logf("[%s] ERROR timeout bytes=%lu", logTag ? logTag : "IPP-DOC", (unsigned long)contentLen);
    return false;
  }

  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  int code = 0;
  int sp = statusLine.indexOf(' ');
  if (sp >= 0 && statusLine.length() >= sp + 4) code = statusLine.substring(sp + 1, sp + 4).toInt();

  String respSnippet = "";
  unsigned long t1 = millis();
  while ((client.connected() || client.available()) && (millis() - t1 < 3000UL)) {
    if (!client.available()) { delay(1); continue; }
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) break;
    if (respSnippet.length() < 180) { respSnippet += line; respSnippet += " | "; }
  }

  uint8_t ippHead[8];
  size_t ippHeadLen = 0;
  t1 = millis();
  while (ippHeadLen < sizeof(ippHead) && (client.connected() || client.available()) && (millis() - t1 < 3000UL)) {
    while (client.available() && ippHeadLen < sizeof(ippHead)) ippHead[ippHeadLen++] = (uint8_t)client.read();
    if (ippHeadLen >= sizeof(ippHead)) break;
    delay(1);
  }

  int ippStatus = -1;
  if (ippHeadLen >= 4) ippStatus = ((int)ippHead[2] << 8) | ippHead[3];

  t1 = millis();
  while ((client.connected() || client.available()) && (millis() - t1 < 500UL)) {
    while (client.available()) client.read();
    delay(1);
  }
  client.stop();

  g_prnLastBytes = (uint32_t)contentLen;

  if (code < 200 || code >= 300) {
    g_prnLastError = "IPP HTTP status='" + statusLine + "' resp=" + respSnippet;
    logf("[%s] ERROR status=%s bytes=%lu resp=%s", logTag ? logTag : "IPP-DOC", statusLine.c_str(), (unsigned long)contentLen, respSnippet.c_str());
    return false;
  }

  if (ippStatus >= 0x0400) {
    g_prnLastError = "IPP rechazado ippStatus=0x" + String(ippStatus, HEX) + " http='" + statusLine + "' resp=" + respSnippet;
    logf("[%s] ERROR ippStatus=0x%04X http=%s bytes=%lu resp=%s", logTag ? logTag : "IPP-DOC", (unsigned int)ippStatus, statusLine.c_str(), (unsigned long)contentLen, respSnippet.c_str());
    return false;
  }

  g_prnLastError = "";
  if (ippStatus >= 0) logf("[%s] OK status=%s ippStatus=0x%04X bytes=%lu", logTag ? logTag : "IPP-DOC", statusLine.c_str(), (unsigned int)ippStatus, (unsigned long)contentLen);
  else logf("[%s] OK status=%s ippStatus=? bytes=%lu", logTag ? logTag : "IPP-DOC", statusLine.c_str(), (unsigned long)contentLen);
  return true;
}

bool printerSendIppPwgRenderJob(const std::vector<uint8_t>& pwg, const char* jobName) {
  return printerSendIppDocumentJob(pwg, jobName ? jobName : "ATARI ATASCII LEGACY", "image/pwg-raster", "IPP-PWG");
}

bool printerSendIppJpegTest() {
  g_prnLastBytes = 0;
  g_prnLastError = "";

  if (!printerEnsureStaReady()) return false;

  String host = String(PRN_CFG.ip);
  uint16_t port = PRN_CFG.port;
  if (port == 0 || port == 9100) port = 631;

  String path = "/ipp/print";
  String url = "http://" + host + ":" + String(port) + path;
  String printerUri = "ipp://" + host + ":" + String(port) + path;

  std::vector<uint8_t> body;
  body.reserve(512 + IPP_TEST_JPEG_LEN);

  // IPP header: version 2.0, operation Print-Job (0x0002), request-id=1
  ippAppendU8(body, 0x02);
  ippAppendU8(body, 0x00);
  ippAppendU16(body, 0x0002);
  ippAppendU32(body, 1);

  // operation-attributes-tag
  ippAppendU8(body, 0x01);
  ippAppendAttr(body, 0x47, "attributes-charset", "utf-8");
  ippAppendAttr(body, 0x48, "attributes-natural-language", "en");
  ippAppendAttr(body, 0x45, "printer-uri", printerUri);
  ippAppendAttr(body, 0x42, "requesting-user-name", "esp32-atari");
  ippAppendAttr(body, 0x42, "job-name", "ATARI IPP TEST");
  ippAppendAttr(body, 0x49, "document-format", "image/jpeg");

  // end-of-attributes-tag
  ippAppendU8(body, 0x03);

  // JPEG document
  for (size_t i = 0; i < IPP_TEST_JPEG_LEN; i++) {
    body.push_back(pgm_read_byte(&IPP_TEST_JPEG[i]));
  }

  HTTPClient http;
  http.setTimeout(30000);
  if (!http.begin(url)) {
    g_prnLastError = "No se pudo iniciar IPP: " + url;
    return false;
  }

  http.addHeader("Content-Type", "application/ipp");
  http.addHeader("Connection", "close");

  int code = http.POST(body.data(), body.size());
  String resp = http.getString();
  http.end();

  g_prnLastBytes = (uint32_t)body.size();

  if (code < 200 || code >= 300) {
    g_prnLastError = "IPP HTTP code=" + String(code) + " resp=" + resp;
    logf("[IPP] ERROR url=%s code=%d bytes=%lu resp=%s", url.c_str(), code, (unsigned long)body.size(), resp.c_str());
    return false;
  }

  g_prnLastError = "";
  logf("[IPP] OK url=%s bytes=%lu code=%d", url.c_str(), (unsigned long)body.size(), code);
  return true;
}



bool printerSendIppJpegRenderJob(const std::vector<uint8_t>& jpg, const char* jobName) {
  g_prnLastBytes = 0;
  g_prnLastError = "";

  if (jpg.empty()) {
    g_prnLastError = "JPEG vacío";
    return false;
  }
  if (!printerEnsureStaReady()) return false;

  String host = String(PRN_CFG.ip);
  uint16_t port = PRN_CFG.port;
  if (port == 0 || port == 9100) port = 631;

  String path = "/ipp/print";
  String printerUri = "ipp://" + host + ":" + String(port) + path;

  std::vector<uint8_t> ipp;
  try {
    ipp.reserve(512);
  } catch (...) {
    g_prnLastError = "Sin memoria reservando header IPP/JPEG";
    return false;
  }

  ippAppendU8(ipp, 0x02);
  ippAppendU8(ipp, 0x00);
  ippAppendU16(ipp, 0x0002); // Print-Job
  ippAppendU32(ipp, 3);

  ippAppendU8(ipp, 0x01); // operation-attributes-tag
  ippAppendAttr(ipp, 0x47, "attributes-charset", "utf-8");
  ippAppendAttr(ipp, 0x48, "attributes-natural-language", "en");
  ippAppendAttr(ipp, 0x45, "printer-uri", printerUri);
  ippAppendAttr(ipp, 0x42, "requesting-user-name", "esp32-atari");
  ippAppendAttr(ipp, 0x42, "job-name", String(jobName ? jobName : "ATARI ATASCII JPEG"));
  ippAppendAttr(ipp, 0x49, "document-format", "image/jpeg");
  ippAppendAttr(ipp, 0x44, "media", virtualPaperMediaKeyword(PRN_CFG.paperSize));
  // V74: clave para que la Brother no imprima el JPEG como miniatura/cuarto de hoja.
  ippAppendAttr(ipp, 0x44, "print-scaling", "fit");
  ippAppendAttrEnum(ipp, "orientation-requested", ippOrientationRequestedValue(PRN_CFG.virtualProfile));
  // V89: pedir a la impresora que trate el trabajo como texto monocromo de alta calidad.
  // Ayuda a evitar que la Brother use mezcla de color/dithering sobre zonas blancas.
  ippAppendAttr(ipp, 0x44, "print-color-mode", "monochrome");
  ippAppendAttrEnum(ipp, "print-quality", 5); // 3=draft, 4=normal, 5=high
  // V89: sugerencia IPP para que el driver optimice como texto, no como foto.
  ippAppendAttr(ipp, 0x44, "print-content-optimize", "text");
  ippAppendU8(ipp, 0x03); // end-of-attributes-tag

  const size_t contentLen = ipp.size() + jpg.size();
  logf("[IPP-JPEG] stream connect host=%s port=%u ipp=%lu jpg=%lu total=%lu freeHeap=%lu",
       host.c_str(),
       port,
       (unsigned long)ipp.size(),
       (unsigned long)jpg.size(),
       (unsigned long)contentLen,
       (unsigned long)ESP.getFreeHeap());

  WiFiClient client;
  client.setTimeout(45000);
  if (!client.connect(host.c_str(), port)) {
    g_prnLastError = "No conecta IPP/JPEG a " + host + ":" + String(port) + " desde STA " + WiFi.localIP().toString();
    return false;
  }

  client.print("POST " + path + " HTTP/1.1\r\n");
  client.print("Host: " + host + ":" + String(port) + "\r\n");
  client.print("Content-Type: application/ipp\r\n");
  client.print("Content-Length: " + String((unsigned long)contentLen) + "\r\n");
  client.print("Connection: close\r\n\r\n");

  size_t written = 0;
  size_t w = client.write(ipp.data(), ipp.size());
  written += w;
  if (w != ipp.size()) {
    client.stop();
    g_prnLastError = "Fallo enviando header IPP/JPEG";
    logf("[IPP-JPEG] ERROR write ipp %lu/%lu", (unsigned long)w, (unsigned long)ipp.size());
    return false;
  }

  const size_t CHUNK = 1024;
  for (size_t pos = 0; pos < jpg.size(); pos += CHUNK) {
    size_t n = jpg.size() - pos;
    if (n > CHUNK) n = CHUNK;
    w = client.write(&jpg[pos], n);
    written += w;
    if (w != n) {
      client.stop();
      g_prnLastError = "Fallo enviando JPEG";
      logf("[IPP-JPEG] ERROR write jpg pos=%lu wrote=%lu/%lu totalWritten=%lu",
           (unsigned long)pos, (unsigned long)w, (unsigned long)n, (unsigned long)written);
      return false;
    }
    delay(0);
  }

  unsigned long t0 = millis();
  while (client.connected() && !client.available() && (millis() - t0 < 45000UL)) delay(10);

  if (!client.available()) {
    client.stop();
    g_prnLastBytes = (uint32_t)contentLen;
    g_prnLastError = "Timeout esperando respuesta IPP/JPEG";
    logf("[IPP-JPEG] ERROR timeout bytes=%lu", (unsigned long)contentLen);
    return false;
  }

  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  int code = 0;
  int sp = statusLine.indexOf(' ');
  if (sp >= 0 && statusLine.length() >= sp + 4) code = statusLine.substring(sp + 1, sp + 4).toInt();

  String respSnippet = "";
  unsigned long t1 = millis();
  while ((client.connected() || client.available()) && (millis() - t1 < 3000UL)) {
    if (!client.available()) { delay(1); continue; }
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) break;
    if (respSnippet.length() < 180) { respSnippet += line; respSnippet += " | "; }
  }

  uint8_t ippHead[8];
  size_t ippHeadLen = 0;
  t1 = millis();
  while (ippHeadLen < sizeof(ippHead) && (client.connected() || client.available()) && (millis() - t1 < 3000UL)) {
    while (client.available() && ippHeadLen < sizeof(ippHead)) ippHead[ippHeadLen++] = (uint8_t)client.read();
    if (ippHeadLen >= sizeof(ippHead)) break;
    delay(1);
  }

  int ippStatus = -1;
  if (ippHeadLen >= 4) ippStatus = ((int)ippHead[2] << 8) | ippHead[3];

  t1 = millis();
  while ((client.connected() || client.available()) && (millis() - t1 < 500UL)) {
    while (client.available()) client.read();
    delay(1);
  }
  client.stop();

  g_prnLastBytes = (uint32_t)contentLen;

  if (code < 200 || code >= 300) {
    g_prnLastError = "IPP/JPEG HTTP status='" + statusLine + "' resp=" + respSnippet;
    logf("[IPP-JPEG] ERROR status=%s bytes=%lu resp=%s", statusLine.c_str(), (unsigned long)contentLen, respSnippet.c_str());
    return false;
  }

  if (ippStatus >= 0x0400) {
    g_prnLastError = "IPP/JPEG rechazado ippStatus=0x" + String(ippStatus, HEX) + " http='" + statusLine + "' resp=" + respSnippet;
    logf("[IPP-JPEG] ERROR ippStatus=0x%04X http=%s bytes=%lu resp=%s", (unsigned int)ippStatus, statusLine.c_str(), (unsigned long)contentLen, respSnippet.c_str());
    return false;
  }

  g_prnLastError = "";
  if (ippStatus >= 0) logf("[IPP-JPEG] OK status=%s ippStatus=0x%04X bytes=%lu", statusLine.c_str(), (unsigned int)ippStatus, (unsigned long)contentLen);
  else logf("[IPP-JPEG] OK status=%s ippStatus=? bytes=%lu", statusLine.c_str(), (unsigned long)contentLen);
  return true;
}

static size_t printerVirtualBytesForRows(uint32_t maxRows, uint32_t* outRows) {
  uint32_t occupiedRows = 0;
  size_t pos = 0;
  if (maxRows == 0) maxRows = 1;
  if (g_prnVirtualBuffer.empty()) {
    if (outRows) *outRows = 0;
    return 0;
  }

  uint16_t cols = getVirtualPrinterEffectiveColumns(PRN_CFG.virtualProfile);
  if (cols == 0) cols = 40;
  uint16_t col = 0;
  bool currentRowHasContent = false;
  bool justAutoWrapped = false;

  while (pos < g_prnVirtualBuffer.size()) {
    uint8_t c = g_prnVirtualBuffer[pos];

    if (c == 0x9B || c == 0x0D || c == 0x0A) {
      if (justAutoWrapped && !currentRowHasContent) {
        justAutoWrapped = false;
        pos++;
        continue;
      }
      if (!currentRowHasContent && occupiedRows >= maxRows) break;
      pos++;
      if (!currentRowHasContent) occupiedRows++;
      col = 0;
      currentRowHasContent = false;
      justAutoWrapped = false;
      continue;
    }

    if (!currentRowHasContent && occupiedRows >= maxRows) break;
    pos++;
    if (!currentRowHasContent) {
      occupiedRows++;
      currentRowHasContent = true;
    }
    col++;
    if (col >= cols) {
      col = 0;
      currentRowHasContent = false;
      justAutoWrapped = true;
    } else {
      justAutoWrapped = false;
    }
  }

  if (outRows) *outRows = occupiedRows;
  return pos;
}

static uint32_t printerVirtualRenderedRows() {
  uint32_t rows = 0;
  printerVirtualBytesForRows(0xFFFFFFFFu, &rows);
  return rows;
}

static bool printerVirtualIsEol(uint8_t c) {
  return (c == 0x9B || c == 0x0D || c == 0x0A);
}

static bool printerVirtualLastIsEol() {
  return (!g_prnVirtualBuffer.empty() && printerVirtualIsEol(g_prnVirtualBuffer.back()));
}

static void printerVirtualAppendEol() {
  if (!printerVirtualLastIsEol()) g_prnVirtualBuffer.push_back(0x9B);
}

static uint16_t printerVirtualCurrentColumn() {
  const uint16_t cols = getVirtualPrinterEffectiveColumns(PRN_CFG.virtualProfile);
  uint16_t col = 0;
  size_t start = 0;
  for (size_t i = g_prnVirtualBuffer.size(); i > 0; i--) {
    if (printerVirtualIsEol(g_prnVirtualBuffer[i - 1])) {
      start = i;
      break;
    }
  }
  for (size_t i = start; i < g_prnVirtualBuffer.size(); i++) {
    if (printerVirtualIsEol(g_prnVirtualBuffer[i])) {
      col = 0;
    } else {
      col++;
      if (cols > 0 && col >= cols) col = 0;
    }
  }
  return col;
}

static void printerAppendVirtualLineFiel820(const uint8_t* data, uint8_t len, uint8_t effectiveLen) {
  for (uint8_t i = 0; i < effectiveLen; i++) g_prnVirtualBuffer.push_back(data[i]);
  printerVirtualAppendEol();
}

static void printerAppendVirtualLineExtendida(const uint8_t* data, uint8_t len, uint8_t effectiveLen) {
  const bool sourceHadEol = (effectiveLen < len);
  const uint16_t cols = getVirtualPrinterEffectiveColumns(PRN_CFG.virtualProfile);

  if (effectiveLen == 0) {
    // Línea vacía o solo espacios: conserva salto de línea real.
    printerVirtualAppendEol();
    return;
  }

  uint8_t pos = 0;
  while (pos < effectiveLen) {
    uint16_t col = printerVirtualCurrentColumn();
    if (cols > 0 && col >= cols) {
      printerVirtualAppendEol();
      col = 0;
    }

    uint16_t room = (cols > col) ? (uint16_t)(cols - col) : cols;
    if (room == 0) room = cols ? cols : 40;

    uint16_t remain = (uint16_t)(effectiveLen - pos);
    uint8_t chunk = (uint8_t)((remain < room) ? remain : room);
    for (uint8_t i = 0; i < chunk; i++) g_prnVirtualBuffer.push_back(data[pos + i]);
    pos += chunk;

    // Si todavía queda contenido, se parte al ancho útil calculado.
    if (pos < effectiveLen) printerVirtualAppendEol();
  }

  // En modo extendido, el EOL se infiere por el blank-fill del handler Atari:
  // si el registro físico llegó con espacios al final, se considera fin de línea lógica.
  if (sourceHadEol) printerVirtualAppendEol();
}

bool printerAppendVirtualAtasciiLine(const uint8_t* data, uint8_t len) {
  if (!PRN_CFG.enabled) {
    g_prnLastError = "Impresora deshabilitada";
    return true;
  }
  if (!data && len) return false;

  // V73 default: composición extendida.
  // La Atari 820 recibe registros físicos de 40 caracteres normal / 29 sideways.
  // Según el manual del OS, el handler acumula caracteres y envía registros
  // correspondientes a una línea de impresión; al ver EOL hace blank-fill.
  // Por eso aquí recortamos espacios finales y usamos ese blank-fill como señal
  // de fin de línea lógica. Si no hay espacios finales, se permite unir el
  // siguiente registro para aprovechar 80/132/ancho A4.
  uint8_t effectiveLen = len;
  while (effectiveLen > 0 && data[effectiveLen - 1] == 0x20) effectiveLen--;

  if (PRN_CFG.composeMode == PRN_COMPOSE_FIEL_820) {
    printerAppendVirtualLineFiel820(data, len, effectiveLen);
  } else {
    printerAppendVirtualLineExtendida(data, len, effectiveLen);
  }

  g_prnVirtualLines = printerVirtualRenderedRows();
  g_prnVirtualLastMs = millis();
  g_prnVirtualPending = !g_prnVirtualBuffer.empty();
  g_prnAutoPrintBlockedAfterError = false; // V90: texto nuevo desbloquea auto-print

  const VirtualPrinterProfile& prof = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(PRN_CFG.virtualProfile)];
  const uint16_t flushRows = getVirtualPrinterEffectiveFlushRows(prof.id);
  logf("[PRN-VIRT] add AtariLine mode=%s len=%u trim=%u rows=%lu bytes=%lu perfil=%s cols=%u rows=%u flushRows=%u orient=%s fontPt=%u",
       printerCompositionModeName(PRN_CFG.composeMode),
       (unsigned int)len,
       (unsigned int)effectiveLen,
       (unsigned long)g_prnVirtualLines,
       (unsigned long)g_prnVirtualBuffer.size(),
       prof.name,
       getVirtualPrinterEffectiveColumns(prof.id),
       getVirtualPrinterEffectiveRows(prof.id),
       flushRows,
       virtualOrientationName(getVirtualPrinterEffectiveOrientation(prof.id)),
       (unsigned int)PRN_CFG.fontScale);

  // V80: no renderizar mientras el Atari todavía está enviando datos.
  // Solo dejamos latcheado que ya hay al menos una página lista dentro del spool.
  // El render/IPP se ejecuta después desde servicePrinterManualPrintRequest(),
  // una página a la vez, liberando memoria entre páginas.
  if (g_prnVirtualLines >= flushRows && !g_prnVirtualPageFullLatched) {
    g_prnVirtualPageFullLatched = true;
    logf("[PRN-SPOOL] pageReady rows=%lu flushRows=%u bytes=%lu; esperando impresión manual/cola",
         (unsigned long)g_prnVirtualLines,
         (unsigned int)flushRows,
         (unsigned long)g_prnVirtualBuffer.size());
  }
  return true;
}

bool printerFlushVirtualBuffer(const char* reason) {
  if (!g_prnVirtualPending || g_prnVirtualBuffer.empty()) return true;

  const VirtualPrinterProfile& prof = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(PRN_CFG.virtualProfile)];
  uint16_t flushRows = getVirtualPrinterEffectiveFlushRows(prof.id);

  // V97: NO ajustar fuente, NO ONE_PAGE_FIT y NO mezclar formatos en el spool.
  // Si el documento ocupa varias hojas, se imprimen TODAS con la misma
  // configuración capturada al inicio del trabajo.
  const uint8_t savedFontPt = PRN_CFG.fontScale;
  bool autoFitFont8 = false;

  uint32_t pageLines = 0;
  size_t pageBytes = 0;
  uint16_t attemptRows = flushRows;
  std::vector<uint8_t> pageData;
  std::vector<uint8_t> doc; // solo para flujos legacy/PWG; JPEG F49Z75 se spolea a SD
  String docPath = String(PRN_JPEG_JOB_PATH);
  size_t docBytes = 0;
  bool built = false;
  // V97: spool Atari siempre JPEG estable para TODAS las páginas del trabajo.
  const bool usePwg = false;

  while (attemptRows >= 16 && !built) {
    pageLines = 0;
    pageBytes = printerVirtualBytesForRows(attemptRows, &pageLines);
    if (pageBytes == 0 || pageLines == 0) { if (autoFitFont8) PRN_CFG.fontScale = savedFontPt; return true; }

    try {
      pageData.assign(g_prnVirtualBuffer.begin(), g_prnVirtualBuffer.begin() + pageBytes);
    } catch (...) {
      g_prnLastError = "Sin memoria separando pagina ATASCII";
      logf("[PRN-VIRT] ERROR split attemptRows=%u pageBytes=%lu freeHeap=%lu",
           (unsigned int)attemptRows, (unsigned long)pageBytes, (unsigned long)ESP.getFreeHeap());
      if (autoFitFont8) PRN_CFG.fontScale = savedFontPt;
      return false;
    }

    g_prnLastError = "";
    docBytes = 0;
    built = usePwg
      ? buildAtasciiPwgRender(pageData.data(), pageData.size(), PRN_CFG.virtualProfile, doc, attemptRows)
      : buildAtasciiJpegRenderToFile(pageData.data(), pageData.size(), PRN_CFG.virtualProfile, docPath, &docBytes, attemptRows);
    if (!built) {
      logf("[PRN-VIRT] %s retry reason=%s attemptRows=%u pageLines=%lu pageBytes=%lu err='%s' freeHeap=%lu",
           usePwg ? "PWG" : "JPEG", reason ? reason : "?", (unsigned int)attemptRows, (unsigned long)pageLines,
           (unsigned long)pageBytes, g_prnLastError.c_str(), (unsigned long)ESP.getFreeHeap());
      pageData.clear();
      doc.clear();
      if (!usePwg && SPIFFS.exists(docPath)) SPIFFS.remove(docPath);
      if (attemptRows > 128) attemptRows = 128;
      else if (attemptRows > 112) attemptRows = 112;
      else if (attemptRows > 96) attemptRows = 96;
      else if (attemptRows > 72) attemptRows = 72;
      else if (attemptRows > 56) attemptRows = 56;
      else if (attemptRows > 32) attemptRows = 32;
      else if (attemptRows > 16) attemptRows = 16;
      else break;
      delay(0);
    }
  }

  if (!built) {
    if (g_prnLastError.length() == 0) g_prnLastError = usePwg ? "No se pudo generar PWG ATASCII incluso con página reducida" : "No se pudo generar JPEG ATASCII incluso con página reducida";
    logf("[PRN-VIRT] ERROR build %s final reason=%s pageLines=%lu pageBytes=%lu bufferSeMantiene=1",
         usePwg ? "PWG" : "JPEG",
         reason ? reason : "?", (unsigned long)pageLines, (unsigned long)pageBytes);
    if (autoFitFont8) PRN_CFG.fontScale = savedFontPt;
    return false;
  }

  logf("[PRN-SPOOL] PRINT_PAGE reason=%s pageLines=%lu totalLines=%lu pageRaw=%lu totalRaw=%lu doc=%lu perfil=%s quality=%s format=%s",
       reason ? reason : "?",
       (unsigned long)pageLines,
       (unsigned long)g_prnVirtualLines,
       (unsigned long)pageBytes,
       (unsigned long)g_prnVirtualBuffer.size(),
       (unsigned long)(usePwg ? doc.size() : docBytes),
       prof.name,
       printerRenderQualityName(PRN_CFG.renderQuality),
       usePwg ? "image/pwg-raster" : "image/jpeg");

  bool sent = usePwg ? printerSendIppPwgRenderJob(doc, "ATARI ATASCII VIRTUAL PRINTER") : printerSendIppJpegRenderFileJob(docPath, docBytes, "ATARI ATASCII VIRTUAL PRINTER");
  if (!sent && usePwg && g_prnLastError.indexOf("0x411") >= 0) {
    String pwgErr = g_prnLastError;
    const size_t originalPwgBytes = doc.size();
    logf("[PRN-QUALITY] PWG rechazado por formato 0x0411; liberando PWG y reintentando JPEG seguro");

    // V94: liberar completamente el documento PWG antes de construir JPEG.
    // En V93 quedaba el vector PWG ocupando heap y el fallback JPEG fallaba por memoria/capacidad.
    std::vector<uint8_t>().swap(doc);
    delay(10);

    bool fallbackPrinted = false;
    uint16_t fbRowsList[7];
    uint8_t fbCount = 0;
    fbRowsList[fbCount++] = attemptRows;
    if (attemptRows > 72) fbRowsList[fbCount++] = 72;
    if (attemptRows > 56) fbRowsList[fbCount++] = 56;
    if (attemptRows > 40) fbRowsList[fbCount++] = 40;
    if (attemptRows > 32) fbRowsList[fbCount++] = 32;
    if (attemptRows > 24) fbRowsList[fbCount++] = 24;
    if (attemptRows > 16) fbRowsList[fbCount++] = 16;

    for (uint8_t i = 0; i < fbCount && !fallbackPrinted; ++i) {
      uint16_t fbRows = fbRowsList[i];
      uint32_t fbLines = 0;
      size_t fbBytes = printerVirtualBytesForRows(fbRows, &fbLines);
      if (fbBytes == 0 || fbLines == 0) continue;

      std::vector<uint8_t> fbPageData;
      try {
        fbPageData.assign(g_prnVirtualBuffer.begin(), g_prnVirtualBuffer.begin() + fbBytes);
      } catch (...) {
        g_prnLastError = "Sin memoria separando pagina fallback JPEG";
        logf("[PRN-QUALITY] fallback JPEG split fail rows=%u bytes=%lu freeHeap=%lu maxAlloc=%lu",
             (unsigned int)fbRows, (unsigned long)fbBytes, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
        continue;
      }

      std::vector<uint8_t> jpgFallback;
      g_prnLastError = "";
      bool builtFallback = buildAtasciiJpegRender(fbPageData.data(), fbPageData.size(), PRN_CFG.virtualProfile, jpgFallback, fbRows);
      if (!builtFallback) {
        logf("[PRN-QUALITY] fallback JPEG build fail rows=%u lines=%lu bytes=%lu err='%s' freeHeap=%lu maxAlloc=%lu",
             (unsigned int)fbRows, (unsigned long)fbLines, (unsigned long)fbBytes, g_prnLastError.c_str(),
             (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
        continue;
      }

      logf("[PRN-SPOOL] FALLBACK_PAGE reason=%s fbRows=%u pageLines=%lu jpeg=%lu originalPwg=%lu perfil=%s format=image/jpeg",
           reason ? reason : "?", (unsigned int)fbRows, (unsigned long)fbLines,
           (unsigned long)jpgFallback.size(), (unsigned long)originalPwgBytes, prof.name);
      sent = printerSendIppJpegRenderJob(jpgFallback, "ATARI ATASCII VIRTUAL PRINTER JPEG FALLBACK");
      if (sent) {
        doc.swap(jpgFallback);
        pageData.swap(fbPageData);
        pageLines = fbLines;
        pageBytes = fbBytes;
        attemptRows = fbRows;
        g_prnLastError = "";
        fallbackPrinted = true;
      } else {
        logf("[PRN-QUALITY] fallback JPEG send fail rows=%u err='%s'", (unsigned int)fbRows, g_prnLastError.c_str());
      }
    }

    if (!fallbackPrinted && g_prnLastError.length() == 0) {
      g_prnLastError = pwgErr + " | fallback JPEG no generado/enviado";
    }
  }
  if (sent) {
    g_prnSpoolPagesPrinted++;
    g_prnSpoolLastPageBytes = (uint32_t)pageBytes;
    g_prnSpoolLastJpegBytes = (uint32_t)(usePwg ? doc.size() : docBytes);
    pageData.clear();
    doc.clear();
    g_prnVirtualBuffer.erase(g_prnVirtualBuffer.begin(), g_prnVirtualBuffer.begin() + pageBytes);
    // V98: mantener explícitamente el tamaño de fuente bloqueado del trabajo.
    PRN_CFG.fontScale = savedFontPt;
    g_prnVirtualLines = printerVirtualRenderedRows();

    g_prnVirtualPending = !g_prnVirtualBuffer.empty();
    g_prnVirtualPageFullLatched = false;
    if (!g_prnVirtualPending) {
      g_prnVirtualLastMs = 0;
      std::vector<uint8_t>().swap(g_prnVirtualBuffer); // V80: liberar heap del spool ya consumido
    }
  } else {
    logf("[PRN-VIRT] ERROR send %s reason=%s bufferSeMantiene=1", usePwg ? "PWG" : "JPEG", reason ? reason : "?");
  }
  if (autoFitFont8) PRN_CFG.fontScale = savedFontPt;
  return sent;
}

void servicePrinterVirtualBuffer() {
  if (!g_prnVirtualPending || g_prnVirtualBuffer.empty()) return;
  if (!PRN_CFG.autoPrintSpool) return;
  if (g_prnAutoPrintBlockedAfterError) return;
  if (g_prnManualPrintBusy || g_prnManualPrintRequested || g_prnAutoPrintRequested) return;
  if (g_prnVirtualLastMs == 0) return;

  const uint32_t idleMs = (uint32_t)(millis() - g_prnVirtualLastMs);
  if (idleMs >= PRN_CFG.autoPrintDelayMs) {
    g_prnAutoPrintRequested = true;
    logf("[PRN-SPOOL] AUTO request idleMs=%lu delayMs=%u lines=%lu bytes=%lu freeHeap=%lu",
         (unsigned long)idleMs,
         (unsigned int)PRN_CFG.autoPrintDelayMs,
         (unsigned long)g_prnVirtualLines,
         (unsigned long)g_prnVirtualBuffer.size(),
         (unsigned long)ESP.getFreeHeap());
  }
}

void handlePrinterVirtualTest() {
  logf("[PRN-VIRT] TEST inicio V121 GATEWAY_WINDOWS_JPEG_STABLE freeHeap=%lu", (unsigned long)ESP.getFreeHeap());
  g_prnLinesRx++;
  g_prnLastText = "ATASCII/JPEG virtual test layout";
  g_prnLastError = "";
  g_prnLastBytes = 0;

  const uint8_t profileId = PRN_CFG.virtualProfile;
  const VirtualPrinterProfile& prof = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(profileId)];
  const uint16_t cols = getVirtualPrinterEffectiveColumns(profileId);
  const uint16_t rows = getVirtualPrinterEffectiveRows(profileId);
  const uint16_t flushRows = getVirtualPrinterEffectiveFlushRows(profileId);
  const uint8_t orient = getVirtualPrinterEffectiveOrientation(profileId);
  const uint8_t fontId = effectiveFontForProfile(profileId);
  const uint8_t scaleX = getVirtualPrinterEffectiveScaleX(profileId, fontId);
  const uint8_t scaleY = getVirtualPrinterEffectiveScaleY(profileId, fontId);

  std::vector<uint8_t> data;
  try {
    data.reserve((size_t)cols * (size_t)rows + (size_t)rows + 128);
  } catch (...) {
    g_prnLastError = "Sin memoria preparando prueba layout";
    server.send(500, "text/plain", g_prnLastError);
    return;
  }

  auto addText = [&](const char* s) {
    while (*s) data.push_back((uint8_t)*s++);
    data.push_back(0x9B);
  };

  auto addString = [&](const String& s) {
    for (size_t i = 0; i < s.length(); i++) data.push_back((uint8_t)s[i]);
    data.push_back(0x9B);
  };

  auto addExactWidthLine = [&](uint16_t lineNo) {
    char prefix[12];
    snprintf(prefix, sizeof(prefix), "%03u|", (unsigned int)(lineNo % 1000));
    const uint8_t prefixLen = 4;
    for (uint16_t c = 0; c < cols; c++) {
      if (c < prefixLen) data.push_back((uint8_t)prefix[c]);
      else data.push_back((uint8_t)('0' + ((c + 1) % 10)));
    }
    data.push_back(0x9B);
  };

  addText("ATARI VIRTUAL PRINTER - ATASCII / JPEG RENDER");
  addString(String("PERFIL: ") + prof.name);
  addString(String("LAYOUT: cols=") + String((int)cols) +
            " rows=" + String((int)rows) +
            " flush=" + String((int)flushRows) +
            " orient=" + virtualOrientationName(orient) +
            " scale=" + String((int)scaleX) + "x" + String((int)scaleY));

  // Línea de regla exactamente del ancho elegido por el usuario.
  // V70: línea de prueba usa el ancho efectivo calculado, con corte JPEG seguro.
  if (rows > 3) {
    addExactWidthLine(1);
  }

  // Completa la hoja con líneas exactamente del ancho configurado para comprobar filas por hoja.
  uint16_t generatedLines = 4;
  for (uint16_t r = 2; r <= rows && generatedLines < rows; r++, generatedLines++) {
    addExactWidthLine(r);
  }

  std::vector<uint8_t> doc; // legacy/PWG; JPEG F49Z75 se genera como archivo en SD
  String docPath = String(PRN_JPEG_JOB_PATH);
  size_t docBytes = 0;
  bool built = false;
  const bool usePwg = false; // V120 JPEG_ONLY
  uint16_t attemptRows = rows;
  while (attemptRows >= 16 && !built) {
    g_prnLastError = "";
    doc.clear();
    docBytes = 0;
    if (!usePwg && SPIFFS.exists(docPath)) SPIFFS.remove(docPath);
    built = usePwg
      ? buildAtasciiPwgRender(data.data(), data.size(), profileId, doc, attemptRows)
      : buildAtasciiJpegRenderToFile(data.data(), data.size(), profileId, docPath, &docBytes, attemptRows);
    logf("[PRN-VIRT] TEST V120 attemptRows=%u built=%d cols=%u rows=%u data=%lu doc=%lu format=%s freeHeap=%lu",
         (unsigned int)attemptRows, built ? 1 : 0, cols, rows,
         (unsigned long)data.size(), (unsigned long)(usePwg ? doc.size() : docBytes),
         usePwg ? "image/pwg-raster" : "image/jpeg",
         (unsigned long)ESP.getFreeHeap());
    if (!built) {
      if (attemptRows > 128) attemptRows = 128;
      else if (attemptRows > 112) attemptRows = 112;
      else if (attemptRows > 96) attemptRows = 96;
      else if (attemptRows > 72) attemptRows = 72;
      else if (attemptRows > 56) attemptRows = 56;
      else if (attemptRows > 32) attemptRows = 32;
      else if (attemptRows > 16) attemptRows = 16;
      else break;
      delay(0);
    }
  }
  bool ok = built && (usePwg ? printerSendIppPwgRenderJob(doc, "ATARI ATASCII TEST LAYOUT") : printerSendIppJpegRenderFileJob(docPath, docBytes, "ATARI ATASCII TEST LAYOUT"));
  logf("[PRN-VIRT] TEST V120 sent ok=%d format=%s lastErr='%s' freeHeap=%lu", ok ? 1 : 0, usePwg ? "PWG" : "JPEG", g_prnLastError.c_str(), (unsigned long)ESP.getFreeHeap());

  if (ok) {
    g_prnLinesOk++;
    server.send(200, "text/plain", String("OK IPP/") + (usePwg ? "PWG" : "JPEG") + " perfil=" + String(prof.name) +
                " cols=" + String((int)cols) +
                " rows=" + String((int)rows) +
                " bytes=" + String((unsigned long)g_prnLastBytes));
  } else {
    g_prnLinesErr++;
    if (!built && g_prnLastError.length() == 0) g_prnLastError = usePwg ? "No se pudo generar PWG" : "No se pudo generar JPEG";
    server.send(500, "text/plain", g_prnLastError);
  }
}

void handlePrinterAtasciiTableTest() {
  logf("[PRN-VIRT] ATASCII TABLE TEST inicio V81 one-page-fit freeHeap=%lu", (unsigned long)ESP.getFreeHeap());
  g_prnLinesRx++;
  g_prnLastText = "ATASCII glyph table test split";
  g_prnLastError = "";
  g_prnLastBytes = 0;

  bool allOk = true;
  unsigned long totalBytes = 0;

  for (uint8_t page = 0; page < 2; page++) {
    const bool inversePage = (page == 1);
    String jpgPath = inversePage ? String("/PRINT/atascii_table_80_ff.jpg") : String("/PRINT/atascii_table_00_7f.jpg");
    size_t jpgBytes = 0;
    if (SPIFFS.exists(jpgPath)) SPIFFS.remove(jpgPath);
    bool built = buildAtasciiTableJpegRenderToFile(jpgPath, &jpgBytes, inversePage);
    logf("[PRN-VIRT] ATASCII TABLE page=%u built=%d jpg=%lu freeHeap=%lu",
         (unsigned int)(page + 1), built ? 1 : 0, (unsigned long)jpgBytes, (unsigned long)ESP.getFreeHeap());

    String jobName = inversePage ? "ATARI ATASCII GLYPH TABLE 80-FF" : "ATARI ATASCII GLYPH TABLE 00-7F";
    bool ok = built && printerSendIppJpegRenderFileJob(jpgPath, jpgBytes, jobName.c_str());
    logf("[PRN-VIRT] ATASCII TABLE page=%u sent ok=%d lastErr='%s' freeHeap=%lu",
         (unsigned int)(page + 1), ok ? 1 : 0, g_prnLastError.c_str(), (unsigned long)ESP.getFreeHeap());

    if (!ok) {
      allOk = false;
      if (!built && g_prnLastError.length() == 0) g_prnLastError = "No se pudo generar tabla ATASCII JPEG";
      break;
    }
    totalBytes += (unsigned long)g_prnLastBytes;
    // Mantener el ultimo JPEG en /PRINT ayuda al diagnóstico; se sobrescribe en la siguiente prueba.
    delay(300);
  }

  if (allOk) {
    g_prnLinesOk++;
    server.send(200, "text/plain", "OK ATASCII TABLE SPLIT pages=2 bytes=" + String(totalBytes));
  } else {
    g_prnLinesErr++;
    server.send(500, "text/plain", g_prnLastError);
  }
}


bool printerSendIppJpegRenderFileJob(const String& jpgPath, size_t jpgBytes, const char* jobName) {
  g_prnLastBytes = 0;
  g_prnLastError = "";

  if (jpgPath.length() == 0 || jpgBytes == 0 || !SPIFFS.exists(jpgPath)) {
    g_prnLastError = "JPEG SD no existe o está vacío";
    return false;
  }
  if (!printerEnsureStaReady()) return false;

  File jpg = SPIFFS.open(jpgPath, "r");
  if (!jpg) {
    g_prnLastError = "No se pudo abrir JPEG SD para enviar";
    return false;
  }
  jpgBytes = jpg.size();
  if (jpgBytes == 0) {
    jpg.close();
    g_prnLastError = "JPEG SD vacío al enviar";
    return false;
  }

  String host = String(PRN_CFG.ip);
  uint16_t port = PRN_CFG.port;
  if (port == 0 || port == 9100) port = 631;
  String path = "/ipp/print";
  String printerUri = "ipp://" + host + ":" + String(port) + path;

  std::vector<uint8_t> ipp;
  try {
    ipp.reserve(512);
  } catch (...) {
    jpg.close();
    g_prnLastError = "Sin memoria reservando header IPP/JPEG SD";
    return false;
  }

  ippAppendU8(ipp, 0x02);
  ippAppendU8(ipp, 0x00);
  ippAppendU16(ipp, 0x0002);
  ippAppendU32(ipp, 5);
  ippAppendU8(ipp, 0x01);
  ippAppendAttr(ipp, 0x47, "attributes-charset", "utf-8");
  ippAppendAttr(ipp, 0x48, "attributes-natural-language", "en");
  ippAppendAttr(ipp, 0x45, "printer-uri", printerUri);
  ippAppendAttr(ipp, 0x42, "requesting-user-name", "esp32-atari");
  ippAppendAttr(ipp, 0x42, "job-name", String(jobName ? jobName : "ATARI ATASCII JPEG SD"));
  ippAppendAttr(ipp, 0x49, "document-format", "image/jpeg");
  ippAppendAttr(ipp, 0x44, "media", virtualPaperMediaKeyword(PRN_CFG.paperSize));
  ippAppendAttr(ipp, 0x44, "print-scaling", "fit");
  ippAppendAttrEnum(ipp, "orientation-requested", ippOrientationRequestedValue(PRN_CFG.virtualProfile));
  ippAppendAttr(ipp, 0x44, "print-color-mode", "monochrome");
  ippAppendAttrEnum(ipp, "print-quality", 5);
  ippAppendAttr(ipp, 0x44, "print-content-optimize", "text");
  ippAppendU8(ipp, 0x03);

  const size_t contentLen = ipp.size() + jpgBytes;
  logf("[IPP-JPEG-SD] stream connect host=%s port=%u ipp=%lu jpgFile=%lu total=%lu path=%s freeHeap=%lu maxAlloc=%lu",
       host.c_str(), port, (unsigned long)ipp.size(), (unsigned long)jpgBytes,
       (unsigned long)contentLen, jpgPath.c_str(), (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());

  WiFiClient client;
  client.setTimeout(45000);
  if (!client.connect(host.c_str(), port)) {
    jpg.close();
    g_prnLastError = "No conecta IPP/JPEG SD a " + host + ":" + String(port) + " desde STA " + WiFi.localIP().toString();
    return false;
  }

  client.print("POST " + path + " HTTP/1.1\r\n");
  client.print("Host: " + host + ":" + String(port) + "\r\n");
  client.print("Content-Type: application/ipp\r\n");
  client.print("Content-Length: " + String((unsigned long)contentLen) + "\r\n");
  client.print("Connection: close\r\n\r\n");

  size_t written = 0;
  size_t w = client.write(ipp.data(), ipp.size());
  written += w;
  if (w != ipp.size()) {
    jpg.close();
    client.stop();
    g_prnLastError = "Fallo enviando header IPP/JPEG SD";
    return false;
  }

  uint8_t buf[1024];
  while (jpg.available()) {
    int n = jpg.read(buf, sizeof(buf));
    if (n <= 0) break;
    w = client.write(buf, (size_t)n);
    written += w;
    if (w != (size_t)n) {
      jpg.close();
      client.stop();
      g_prnLastError = "Fallo enviando JPEG desde SD";
      logf("[IPP-JPEG-SD] ERROR write pos=%lu wrote=%lu/%d totalWritten=%lu",
           (unsigned long)jpg.position(), (unsigned long)w, n, (unsigned long)written);
      return false;
    }
    delay(0);
  }
  jpg.close();

  unsigned long t0 = millis();
  while (client.connected() && !client.available() && (millis() - t0 < 45000UL)) delay(10);
  if (!client.available()) {
    client.stop();
    g_prnLastBytes = (uint32_t)contentLen;
    g_prnLastError = "Timeout esperando respuesta IPP/JPEG SD";
    return false;
  }

  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  int code = 0;
  int sp = statusLine.indexOf(' ');
  if (sp >= 0 && statusLine.length() >= sp + 4) code = statusLine.substring(sp + 1, sp + 4).toInt();

  String respSnippet = "";
  unsigned long t1 = millis();
  while ((client.connected() || client.available()) && (millis() - t1 < 3000UL)) {
    if (!client.available()) { delay(1); continue; }
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) break;
    if (respSnippet.length() < 180) { respSnippet += line; respSnippet += " | "; }
  }

  uint8_t ippHead[8];
  size_t ippHeadLen = 0;
  t1 = millis();
  while (ippHeadLen < sizeof(ippHead) && (client.connected() || client.available()) && (millis() - t1 < 3000UL)) {
    while (client.available() && ippHeadLen < sizeof(ippHead)) ippHead[ippHeadLen++] = (uint8_t)client.read();
    if (ippHeadLen >= sizeof(ippHead)) break;
    delay(1);
  }
  int ippStatus = -1;
  if (ippHeadLen >= 4) ippStatus = ((int)ippHead[2] << 8) | ippHead[3];

  t1 = millis();
  while ((client.connected() || client.available()) && (millis() - t1 < 500UL)) {
    while (client.available()) client.read();
    delay(1);
  }
  client.stop();

  g_prnLastBytes = (uint32_t)contentLen;
  if (code < 200 || code >= 300) {
    g_prnLastError = "IPP/JPEG SD HTTP status='" + statusLine + "' resp=" + respSnippet;
    logf("[IPP-JPEG-SD] ERROR status=%s bytes=%lu resp=%s", statusLine.c_str(), (unsigned long)contentLen, respSnippet.c_str());
    return false;
  }
  if (ippStatus >= 0x0400) {
    g_prnLastError = "IPP/JPEG SD rechazado ippStatus=0x" + String(ippStatus, HEX) + " http='" + statusLine + "' resp=" + respSnippet;
    logf("[IPP-JPEG-SD] ERROR ippStatus=0x%04X http=%s bytes=%lu resp=%s", (unsigned int)ippStatus, statusLine.c_str(), (unsigned long)contentLen, respSnippet.c_str());
    return false;
  }

  g_prnLastError = "";
  if (ippStatus >= 0) logf("[IPP-JPEG-SD] OK status=%s ippStatus=0x%04X bytes=%lu", statusLine.c_str(), (unsigned int)ippStatus, (unsigned long)contentLen);
  else logf("[IPP-JPEG-SD] OK status=%s ippStatus=? bytes=%lu", statusLine.c_str(), (unsigned long)contentLen);
  return true;
}


String printerVirtualBufferToGatewayText() {
  String out;
  out.reserve(g_prnVirtualBuffer.size() + 256);

  size_t start = 0;
  for (size_t i = 0; i <= g_prnVirtualBuffer.size(); i++) {
    bool eol = (i >= g_prnVirtualBuffer.size()) || printerVirtualIsEol(g_prnVirtualBuffer[i]);
    if (!eol) continue;

    if (i > start) {
      size_t len = i - start;
      while (len > 0) {
        uint8_t chunk = (uint8_t)min((size_t)240, len);
        out += atasciiToAsciiLine(&g_prnVirtualBuffer[start], chunk);
        start += chunk;
        len -= chunk;
      }
    }
    out += "\r\n";
    start = i + 1;
  }

  return out;
}

bool printerFlushVirtualBufferToGateway(const char* reason) {
  if (!g_prnVirtualPending || g_prnVirtualBuffer.empty()) return true;

  // F25: NO convertir a ASCII antes de enviar al Gateway.
  // El texto plano perdía gráficos ATASCII/control/inverso. Enviamos bytes crudos + glifos 8x8.
  logf("[PRN-GW-RAW] FLUSH ATASCII reason=%s lines=%lu rawBytes=%lu gateway=%s",
       reason ? reason : "?",
       (unsigned long)g_prnVirtualLines,
       (unsigned long)g_prnVirtualBuffer.size(),
       PRN_CFG.gateway);

  bool ok = printerSendHttpGatewayAtasciiRawJob(g_prnVirtualBuffer.data(), g_prnVirtualBuffer.size(), reason);
  if (ok) {
    g_prnSpoolPagesPrinted++;
    g_prnSpoolLastPageBytes = (uint32_t)g_prnVirtualBuffer.size();
    g_prnSpoolLastJpegBytes = 0;
    g_prnVirtualBuffer.clear();
    std::vector<uint8_t>().swap(g_prnVirtualBuffer);
    g_prnVirtualLines = 0;
    g_prnVirtualPending = false;
    g_prnVirtualPageFullLatched = false;
    g_prnVirtualLastMs = 0;
  }
  return ok;
}

void servicePrinterManualPrintRequest() {
  if ((!g_prnManualPrintRequested && !g_prnAutoPrintRequested) || g_prnManualPrintBusy) return;
  const bool autoJob = g_prnAutoPrintRequested && !g_prnManualPrintRequested;
  g_prnManualPrintRequested = false;
  g_prnAutoPrintRequested = false;
  g_prnManualPrintBusy = true;
  g_prnManualLastPages = 0;

  g_prnSpoolJobs++;
  logf("[PRN-SPOOL] JOB begin id=%lu source=%s pending=%u lines=%lu bytes=%lu freeHeap=%lu",
       (unsigned long)g_prnSpoolJobs,
       autoJob ? "autoIdle" : "manual",
       g_prnVirtualPending ? 1 : 0,
       (unsigned long)g_prnVirtualLines,
       (unsigned long)g_prnVirtualBuffer.size(),
       (unsigned long)ESP.getFreeHeap());

  if (!g_prnVirtualPending || g_prnVirtualBuffer.empty()) {
    g_prnLastError = "";
    g_prnManualPrintBusy = false;
    return;
  }


  // V121: Gateway Windows usa el spool ATASCII crudo y lo renderiza en Windows como matriz 8x8
  // y lo envía completo al PC. Windows/driver Brother se encarga de paginar e imprimir.
  if (PRN_CFG.mode == PRN_MODE_HTTP) {
    bool okGw = printerFlushVirtualBufferToGateway(autoJob ? "autoIdleGateway" : "manualGateway");
    if (okGw) {
      g_prnLinesOk++;
      g_prnManualLastPages = 1;
      logf("[PRN-GW] JOB OK gateway=%s bytesLast=%lu freeHeap=%lu",
           PRN_CFG.gateway,
           (unsigned long)g_prnLastBytes,
           (unsigned long)ESP.getFreeHeap());
    } else {
      g_prnLinesErr++;
      if (autoJob) g_prnAutoPrintBlockedAfterError = true;
      if (g_prnLastError.length() == 0) g_prnLastError = "No se pudo enviar al Gateway Windows";
      logf("[PRN-GW] JOB ERROR err='%s' freeHeap=%lu", g_prnLastError.c_str(), (unsigned long)ESP.getFreeHeap());
    }
    g_prnJobPageIndex = 0;
    g_prnManualPrintBusy = false;
    return;
  }

  // V93: congelar la configuración de layout al inicio del trabajo.
  // Antes cada página usaba PRN_CFG en vivo; si la web guardaba cambios,
  // si el fallback modificaba temporalmente fontScale, o si se recalculaba
  // el spool con otro estado, la segunda página podía salir con fuente/papel distinto.
  const uint8_t jobProfile       = PRN_CFG.virtualProfile;
  const uint8_t jobPaperSize     = PRN_CFG.paperSize;
  const uint8_t jobFontScale     = PRN_CFG.fontScale;
  // V97: el flujo real del spool Atari queda fijado a JPEG estable para evitar pérdida de páginas al mezclar PWG/JPEG.
  const uint8_t jobRenderQuality = PRN_RENDER_COMPAT_JPEG;
  const uint8_t jobComposeMode   = PRN_CFG.composeMode;
  const uint8_t jobOrientation   = PRN_CFG.pageOrientation;

  // V120: snapshot absoluto. La orientación también queda congelada,
  // ya sea Vertical u Horizontal. Todas las hojas usan el mismo layout.
  PRN_CFG.virtualProfile   = jobProfile;
  PRN_CFG.paperSize        = jobPaperSize;
  PRN_CFG.fontScale        = jobFontScale;
  PRN_CFG.renderQuality    = jobRenderQuality;
  PRN_CFG.composeMode      = jobComposeMode;
  PRN_CFG.pageOrientation  = jobOrientation;

  logf("[PRN-SPOOL] JOB LOCK profile=%s fontPt=%u paper=%s orient=%s quality=%s compose=%s",
       VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(jobProfile)].name,
       (unsigned int)jobFontScale,
       virtualPaperName(jobPaperSize),
       virtualOrientationName(getVirtualPrinterEffectiveOrientation(jobProfile)),
       printerRenderQualityName(jobRenderQuality),
       printerCompositionModeName(jobComposeMode));
  logf("[PRN-SPOOL] JOB SNAPSHOT_ABSOLUTE profile=%u fontPt=%u paper=%u orientationCfg=%u orientationEffective=%s compose=%u format=image/jpeg",
       (unsigned int)jobProfile,
       (unsigned int)jobFontScale,
       (unsigned int)jobPaperSize,
       (unsigned int)jobOrientation,
       virtualOrientationName(getVirtualPrinterEffectiveOrientation(jobProfile)),
       (unsigned int)jobComposeMode);

  // V98: bloquear layout absoluto, deshabilitar totalmente ONE_PAGE_FIT y forzar un solo formato de salida: JPEG.
  // No se verifica si todo cabe en una hoja ni se baja de 12/10/14/16 a 8 pt.
  PRN_CFG.virtualProfile   = jobProfile;
  PRN_CFG.paperSize        = jobPaperSize;
  PRN_CFG.fontScale        = jobFontScale;
  PRN_CFG.renderQuality    = jobRenderQuality;
  PRN_CFG.composeMode      = jobComposeMode;
  PRN_CFG.pageOrientation  = jobOrientation;
  uint16_t jobFlushRows = getVirtualPrinterEffectiveFlushRows(jobProfile);
  logf("[PRN-SPOOL] JOB LAYOUT_LOCK rows=%u totalRows=%lu onePageFit=0 fixedFontPt=%u",
       (unsigned int)jobFlushRows,
       (unsigned long)g_prnVirtualLines,
       (unsigned int)jobFontScale);
  logf("[PRN-SPOOL] JOB FORMAT_LOCK format=image/jpeg reason=V121_gateway_windows_or_jpeg_stable same_profile_font_paper_orientation_compose");

  bool ok = true;
  uint16_t pages = 0;
  while (g_prnVirtualPending && !g_prnVirtualBuffer.empty() && pages < 64) {
    PRN_CFG.virtualProfile   = jobProfile;
    PRN_CFG.paperSize        = jobPaperSize;
    PRN_CFG.fontScale        = jobFontScale;
    PRN_CFG.renderQuality    = jobRenderQuality;
    PRN_CFG.composeMode      = jobComposeMode;
    PRN_CFG.pageOrientation  = jobOrientation;
    g_prnJobPageIndex = pages;

    ok = printerFlushVirtualBuffer(autoJob ? "autoIdle" : "manualQueued");
    // V93: reforzar después de cada página. printerFlushVirtualBuffer puede usar
    // ajustes temporales; la página siguiente debe partir desde el mismo snapshot.
    PRN_CFG.virtualProfile   = jobProfile;
    PRN_CFG.paperSize        = jobPaperSize;
    PRN_CFG.fontScale        = jobFontScale;
    PRN_CFG.renderQuality    = jobRenderQuality;
    PRN_CFG.composeMode      = jobComposeMode;
    PRN_CFG.pageOrientation  = jobOrientation;

    if (!ok) break;
    pages++;
    g_prnManualLastPages = pages;
    delay(60);
  }

  if (ok) {
    g_prnLinesOk++;
    logf("[PRN-SPOOL] JOB OK pages=%u pending=%u bytesLast=%lu freeHeap=%lu",
         (unsigned int)pages,
         g_prnVirtualPending ? 1 : 0,
         (unsigned long)g_prnLastBytes,
         (unsigned long)ESP.getFreeHeap());
  } else {
    g_prnLinesErr++;
    if (autoJob) {
      g_prnAutoPrintBlockedAfterError = true;
      logf("[PRN-SPOOL] AUTO blocked after error; se desbloquea con texto nuevo o impresión manual");
    }
    if (g_prnLastError.length() == 0) g_prnLastError = "No se pudo imprimir buffer Atari";
    logf("[PRN-SPOOL] JOB ERROR pages=%u err='%s' freeHeap=%lu",
         (unsigned int)pages,
         g_prnLastError.c_str(),
         (unsigned long)ESP.getFreeHeap());
  }

  g_prnJobPageIndex = 0;
  g_prnManualPrintBusy = false;
}

void handlePrinterBufferPrint() {
  logf("[PRN-SPOOL] manual print request queued id=%lu pending=%u lines=%lu bytes=%lu busy=%u freeHeap=%lu",
       (unsigned long)g_prnSpoolJobs,
       g_prnVirtualPending ? 1 : 0,
       (unsigned long)g_prnVirtualLines,
       (unsigned long)g_prnVirtualBuffer.size(),
       g_prnManualPrintBusy ? 1 : 0,
       (unsigned long)ESP.getFreeHeap());

  if (!g_prnVirtualPending || g_prnVirtualBuffer.empty()) {
    g_prnLastError = "";
    server.send(200, "text/plain", "BUFFER VACIO");
    return;
  }
  if (g_prnManualPrintBusy || g_prnManualPrintRequested || g_prnAutoPrintRequested) {
    server.send(200, "text/plain", "IMPRESION YA EN CURSO");
    return;
  }

  g_prnAutoPrintBlockedAfterError = false;
  g_prnManualPrintRequested = true;
  // Respondemos antes de crear JPEG para liberar el cliente HTTP y ahorrar heap.
  server.send(202, "text/plain", "IMPRESION PROGRAMADA; revise el log/estado para resultado");
}

void handlePrinterIppTest() {
  g_prnLinesRx++;
  g_prnLastText = "ATARI IPP TEST JPEG";
  bool ok = printerSendIppJpegTest();

  if (ok) {
    g_prnLinesOk++;
    server.send(200, "text/plain", "OK IPP JPEG bytes=" + String((unsigned long)g_prnLastBytes));
  } else {
    g_prnLinesErr++;
    server.send(500, "text/plain", g_prnLastError);
  }
}

bool printerPrintLine(const String& text) {
  if (!PRN_CFG.enabled) {
    g_prnLastError = "Impresora deshabilitada";
    return true;
  }

  if (PRN_CFG.mode == PRN_MODE_RAW9100) {
    return printerSendRaw9100(text);
  }

  if (PRN_CFG.mode == PRN_MODE_HTTP) {
    // V121: Gateway Windows real. Acumula texto y se envía por idle/manual.
    printerAppendHttpBuffer(text);
    return true;
  }

  if (PRN_CFG.mode == PRN_MODE_BT) {
    g_prnLastError = "Bluetooth SPP pendiente de activar";
    return true;
  }

  if (PRN_CFG.mode == PRN_MODE_IPP_JPEG) {
    std::vector<uint8_t> tmp;
    for (size_t i = 0; i < text.length(); i++) tmp.push_back((uint8_t)text[i]);
    return printerAppendVirtualAtasciiLine(tmp.data(), (uint8_t)min((size_t)240, tmp.size()));
  }

  return true;
}

void handlePrinterLineFromRP(const uint8_t* p, uint8_t len) {
  if (!p || len < 5) return;

  uint8_t dev     = p[0];
  uint8_t cmd     = p[1];
  uint8_t aux1    = p[2];
  uint8_t aux2    = p[3];
  uint8_t lineLen = p[4];

  (void)cmd;
  (void)aux2;

  if (len < (uint8_t)(5 + lineLen)) return;

  const uint8_t* line = &p[5];
  g_prnLinesRx++;

  // En modo IPP/JPEG o Gateway Windows usamos el spool ATASCII virtual.
  // IPP/JPEG renderiza a imagen; Gateway Windows lo convierte a texto al finalizar el trabajo.
  if (PRN_CFG.mode == PRN_MODE_IPP_JPEG || PRN_CFG.mode == PRN_MODE_HTTP) {
    String preview = atasciiToAsciiLine(line, lineLen);
    g_prnLastText = preview;
    g_prnLastError = "";
    logf("[PRN] VIRTUAL LINE dev=0x%02X aux1=0x%02X len=%u preview='%s'", dev, aux1, lineLen, preview.c_str());
    bool ok = printerAppendVirtualAtasciiLine(line, lineLen);
    if (ok) g_prnLinesOk++;
    else {
      g_prnLinesErr++;
      logf("[PRN] VIRTUAL ERROR: %s", g_prnLastError.c_str());
    }
    return;
  }

  String text = PRN_CFG.atasciiToAscii ? atasciiToAsciiLine(line, lineLen) : "";
  if (!PRN_CFG.atasciiToAscii) {
    for (uint8_t i = 0; i < lineLen; i++) text += (char)line[i];
    text.trim();
  }

  if (PRN_CFG.cut40) {
    const VirtualPrinterProfile& prof = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(PRN_CFG.virtualProfile)];
    if (text.length() > prof.columns) text = text.substring(0, prof.columns);
  }

  g_prnLastText = text;
  g_prnLastError = "";

  logf("[PRN] LINE dev=0x%02X aux1=0x%02X len=%u text='%s'",
       dev, aux1, lineLen, text.c_str());

  bool ok = printerPrintLine(text);
  if (ok) g_prnLinesOk++;
  else {
    g_prnLinesErr++;
    logf("[PRN] ERROR: %s", g_prnLastError.c_str());
  }
}

// ===== UART RX <- RP2040 FSM =====
static uint8_t uartState = 0;
static uint8_t uartLen = 0;
static uint8_t uartIdx = 0;
static uint32_t uartLastTime = 0;
static uint8_t uartBuf[260];

static uint32_t prnReadU32LE(const uint8_t* p) {
  return ((uint32_t)p[0]) |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

void handlePrinterDiagFromRP(const uint8_t* p, uint8_t len) {
  if (!p || len < 36) return;

  g_prnDiagStatus      = prnReadU32LE(p + 0);
  g_prnDiagWrite       = prnReadU32LE(p + 4);
  g_prnDiagWriteOk     = prnReadU32LE(p + 8);
  g_prnDiagQSent       = prnReadU32LE(p + 12);
  g_prnDiagQDrop       = prnReadU32LE(p + 16);
  g_prnDiagTimeoutData = prnReadU32LE(p + 20);
  g_prnDiagTimeoutChk  = prnReadU32LE(p + 24);
  g_prnDiagChecksumErr = prnReadU32LE(p + 28);
  g_prnDiagUnsupported = prnReadU32LE(p + 32);

  const bool countersChanged =
      g_prnDiagStatus      != g_prnDiagLastStatus ||
      g_prnDiagWrite       != g_prnDiagLastWrite ||
      g_prnDiagWriteOk     != g_prnDiagLastWriteOk ||
      g_prnDiagQSent       != g_prnDiagLastQSent ||
      g_prnDiagQDrop       != g_prnDiagLastQDrop ||
      g_prnDiagTimeoutData != g_prnDiagLastTimeoutData ||
      g_prnDiagTimeoutChk  != g_prnDiagLastTimeoutChk ||
      g_prnDiagChecksumErr != g_prnDiagLastChecksumErr ||
      g_prnDiagUnsupported != g_prnDiagLastUnsupported;

  const bool hasError =
      (g_prnDiagQDrop || g_prnDiagTimeoutData || g_prnDiagTimeoutChk ||
       g_prnDiagChecksumErr || g_prnDiagUnsupported);

  const unsigned long now = millis();
  const unsigned long minInterval = WEB_ATR_ENABLED ? 30000UL : 10000UL;

  // Si todo sigue en cero, no repetir el mismo diagnóstico mientras se prueban discos.
  bool shouldLog = false;
  if (hasError && countersChanged) shouldLog = true;
  else if (countersChanged && (now - g_prnDiagLastLogMs >= minInterval)) shouldLog = true;
  else if (!g_prnDiagLastLogMs && !WEB_ATR_ENABLED) shouldLog = true;

  if (shouldLog) {
    logf("[PRN-DIAG] status=%lu write=%lu ok=%lu qSent=%lu qDrop=%lu toData=%lu toChk=%lu chkErr=%lu unsup=%lu",
         (unsigned long)g_prnDiagStatus,
         (unsigned long)g_prnDiagWrite,
         (unsigned long)g_prnDiagWriteOk,
         (unsigned long)g_prnDiagQSent,
         (unsigned long)g_prnDiagQDrop,
         (unsigned long)g_prnDiagTimeoutData,
         (unsigned long)g_prnDiagTimeoutChk,
         (unsigned long)g_prnDiagChecksumErr,
         (unsigned long)g_prnDiagUnsupported);
    g_prnDiagLastLogMs = now;
  }

  g_prnDiagLastStatus      = g_prnDiagStatus;
  g_prnDiagLastWrite       = g_prnDiagWrite;
  g_prnDiagLastWriteOk     = g_prnDiagWriteOk;
  g_prnDiagLastQSent       = g_prnDiagQSent;
  g_prnDiagLastQDrop       = g_prnDiagQDrop;
  g_prnDiagLastTimeoutData = g_prnDiagTimeoutData;
  g_prnDiagLastTimeoutChk  = g_prnDiagTimeoutChk;
  g_prnDiagLastChecksumErr = g_prnDiagChecksumErr;
  g_prnDiagLastUnsupported = g_prnDiagUnsupported;
}


static const char* diskDiagEventName(uint8_t ev) {
  switch (ev) {
    case 1: return "READ_SENT";
    case 2: return "READ_NAK";
    case 3: return "READ_FAIL";
    default: return "EV";
  }
}

static const char* diskDiagPostName(uint8_t r, uint8_t flags) {
  if (flags & 0x02) return "NEXT_CMD_PUSHBACK";
  if (flags & 0x04) return "IGNORED";
  if (flags & 0x01) return "NO_RESP";
  if (r == 0x41) return "ATARI_ACK";
  if (r == 0x4E) return "ATARI_NAK";
  if (r == 0x43) return "ATARI_COMPLETE";
  if (r == 0x45) return "ATARI_ERROR";
  if (r == 0x00) return "NONE";
  return "OTHER";
}

void handleDiskDiagFromRP(const uint8_t* p, uint8_t len) {
  if (!p || len < 10) return;
  uint8_t ev = p[0];
  uint8_t dev = p[1];
  uint8_t cmd = p[2];
  uint16_t sec = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
  uint16_t dataLen = (uint16_t)p[5] | ((uint16_t)p[6] << 8);
  uint8_t dataChk = p[7];
  uint8_t post = p[8];
  uint8_t flags = p[9];

  logf("[RP-DISK] %s dev=D%u cmd=0x%02X sec=%u len=%u dataChk=0x%02X post=%s(0x%02X) flags=0x%02X",
       diskDiagEventName(ev),
       (unsigned)(dev >= 0x31 && dev <= 0x34 ? dev - 0x30 : 0),
       (unsigned)cmd,
       (unsigned)sec,
       (unsigned)dataLen,
       (unsigned)dataChk,
       diskDiagPostName(post, flags),
       (unsigned)post,
       (unsigned)flags);
}

static uint8_t btState = 0, btLen = 0, btIdx = 0;
static uint8_t btBuf[260];
static uint32_t btLastByteMs = 0;

void handleBtDiskPayload(const uint8_t* data, uint8_t len) {
  if (!data || len == 0) return;
  uint8_t type = data[0];
  g_btDiskFramesRx++;
  g_btDiskLastSeenMs = millis();

  if (type == TYPE_HELLO && len >= 3) {
    uint8_t dev = data[1];
    if (dev < DEV_MIN || dev > DEV_MAX) dev = DEV_MIN;
    bool supports256 = (data[2] != 0);
    markBtDiskOnline(dev, supports256);

    // FIX1: HELLO es solo keepalive del host BT. No se reenvía al RP2040.
    // En la V9 original, HELLO repetidos podían intercalarse durante READ y llegar al RP
    // como frames extra mientras el Atari esperaba DATA, provocando NAK/pausas.
    if (g_lastMasterOp.active) {
      g_btDiskHelloIgnoredDuringOp++;
      return;
    }

    uint32_t now = millis();
    if (now - g_btDiskLastHelloLogMs > 3000) {
      g_btDiskLastHelloLogMs = now;
      logf("[BT-DISK] HELLO host dev=%s 256=%u mask=0x%02X force=0x%02X enabled=%u ignoredOp=%lu",
           devName(dev), supports256 ? 1 : 0, (unsigned)BT_DISK_DEV_MASK,
           (unsigned)BT_DISK_FORCE_MASK, BT_DISK_ENABLED ? 1 : 0,
           (unsigned long)g_btDiskHelloIgnoredDuringOp);
    }
    return;
  }

  if (!BT_DISK_ENABLED) {
    // Host conectado pero gateway deshabilitado: conservar estado online, no inyectar frames al RP.
    return;
  }

  if (!g_btDiskOnline) {
    g_btDiskOnline = true;
    g_btDiskLastSeenMs = millis();
  }

  if (type == TYPE_ACK || type == TYPE_NAK) {
    const char* kind = (type == TYPE_ACK) ? "ACK" : "NAK";
    if (g_lastMasterOp.active) {
      uint32_t dt = millis() - g_lastMasterOp.sentMs;
      logf("[BT-DISK] %s cmd=0x%02X dev=%s sec=%u (%lu ms)",
           kind, (unsigned)g_lastMasterOp.cmd, devName(g_lastMasterOp.dev),
           (unsigned)g_lastMasterOp.sec, (unsigned long)dt);
    }
    clearLastMasterOpLocal();
    sendTimingUpdateToRPThrottled();
    sendUartFrameToRP(data, len);
    return;
  }

  if (type == TYPE_SECTOR_CHUNK) {
    if (len >= 6) g_btDiskChunksRx++;
    sendUartFrameToRP(data, len);
    return;
  }

  sendUartFrameToRP(data, len);
}

void pollBtDiskBridge() {
  if (!g_btDiskUartReady) return;
  while (SerialBTDisk.available() > 0) {
    uint8_t b = (uint8_t)SerialBTDisk.read();
    btLastByteMs = millis();
    switch (btState) {
      case 0: if (b == UART_SYNC) btState = 1; break;
      case 1:
        btLen = b;
        if (btLen == 0 || btLen >= sizeof(btBuf)) { btState = 0; btIdx = 0; btLen = 0; g_btDiskBadLen++; }
        else { btIdx = 0; btState = 2; }
        break;
      case 2:
        btBuf[btIdx++] = b;
        if (btIdx >= btLen) btState = 3;
        break;
      case 3: {
        uint8_t chk = b;
        uint8_t sum = calcChecksum(btBuf, btLen);
        if (chk != sum) {
          g_btDiskBadChk++;
          logf("[BT-DISK] checksum invalido rx=0x%02X calc=0x%02X len=%u", chk, sum, (unsigned)btLen);
        } else {
          handleBtDiskPayload(btBuf, btLen);
        }
        btState = 0; btIdx = 0; btLen = 0;
      } break;
    }
  }

  if (btState != 0 && (millis() - btLastByteMs > 80)) {
    btState = 0; btIdx = 0; btLen = 0; g_btDiskBadLen++;
  }
  if (g_btDiskOnline && (millis() - g_btDiskLastSeenMs > 15000)) {
    g_btDiskOnline = false;
    logf("[BT-DISK] host offline por timeout");
  }
}

void pollUartFromRP() {
  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();
    uartLastTime = millis();

#if MASTER_UART_BYTE_DEBUG
    Serial.print(F("[MASTER UART FSM] state="));
    Serial.print(uartState);
    Serial.print(F(" byte=0x"));
    if (b < 0x10) Serial.print('0');
    Serial.println(b, HEX);
#endif

    switch (uartState) {
      case 0:
        if (b == UART_SYNC) uartState = 1;
        break;

      case 1:
        uartLen = b;
        if (uartLen == 0 || uartLen >= sizeof(uartBuf)) {
          uartState = 0;
          uartIdx = 0;
          uartLen = 0;
        } else {
          uartIdx = 0;
          uartState = 2;
        }
        break;

      case 2:
        uartBuf[uartIdx++] = b;
        if (uartIdx >= uartLen) uartState = 3;
        break;

      case 3: {
        uint8_t chk = b;
        uint8_t sum = calcChecksum(uartBuf, uartLen);
        if (chk != sum) {
          Serial.println(F("[MASTER] UART: checksum inválido, descartando frame."));
        } else {
          uint8_t type = uartBuf[0];

          switch (type) {

            case TYPE_CFG_ACK: {
              if (uartLen >= 10) {
                g_rpCfgAck_ok   = uartBuf[1];
                g_rpCfgAck_uart = getLE32(&uartBuf[2]);
                g_rpCfgAck_sio  = getLE32(&uartBuf[6]);
                g_rpCfgAck = true;

                logf("[MASTER] CFG_ACK <- RP ok=%u uart=%lu rpSio=%lu",
                     (unsigned)g_rpCfgAck_ok,
                     (unsigned long)g_rpCfgAck_uart,
                     (unsigned long)g_rpCfgAck_sio);
              }
            } break;

            case TYPE_PRINTER_CFG_ACK: {
              if (uartLen >= 3) {
                logf("[PRN] CFG_ACK <- RP enabled=%u dev=0x%02X",
                     (unsigned)uartBuf[1],
                     (unsigned)uartBuf[2]);
              }
            } break;

            case TYPE_PRINTER_LINE: {
              if (uartLen >= 6) {
                handlePrinterLineFromRP(&uartBuf[1], uartLen - 1);
              }
            } break;

            case TYPE_PRINTER_DIAG: {
              if (uartLen >= 37) {
                handlePrinterDiagFromRP(&uartBuf[1], uartLen - 1);
              }
            } break;

            case TYPE_DISK_DIAG: {
              if (uartLen >= 11) {
                handleDiskDiagFromRP(&uartBuf[1], uartLen - 1);
              }
            } break;

            case TYPE_CAS_STATUS: {
              if (uartLen >= 17) {
                handleCasStatusFromRP(&uartBuf[1], uartLen - 1);
              }
            } break;

            case TYPE_CAS_ACK: {
              if (uartLen >= 11) {
                handleCasAckFromRP(&uartBuf[1], uartLen - 1);
              }
            } break;

            case TYPE_TIMING_UPDATE: {
              if (uartLen < (1 + 8)) {
                logf("[MASTER] TIMING_UPDATE <- RP inválido (uartLen=%u)", (unsigned)uartLen);
                break;
              }

              const uint8_t* p = &uartBuf[1];
              uint16_t newAck2Comp  = getLE16(&p[0]);
              uint16_t newComp2Data = getLE16(&p[2]);
              uint16_t newData2Chk  = getLE16(&p[4]);
              uint16_t newChunk     = getLE16(&p[6]);

              T_ACK_TO_COMPLETE  = clampU16(newAck2Comp,  120, 1200);
              T_COMPLETE_TO_DATA = clampU16(newComp2Data, 80, 900);
              T_DATA_TO_CHK      = clampU16(newData2Chk,  15, 200);
              T_CHUNK_DELAY      = clampU16(newChunk,     60, 1200);

              logf("[MASTER] TIMING_UPDATE <- RP ack2comp=%u comp2data=%u data2chk=%u chunk=%u",
                  (unsigned)T_ACK_TO_COMPLETE, (unsigned)T_COMPLETE_TO_DATA,
                  (unsigned)T_DATA_TO_CHK, (unsigned)T_CHUNK_DELAY);

              saveTimingConfigToNvs();
            } break;

            case TYPE_CMD_FRAME: {
              if (uartLen >= 7) {
                uint8_t cmd = uartBuf[1];
                uint8_t dev = uartBuf[2];
                uint8_t base = cmd & 0x7F;
                uint16_t sec = (uint16_t)uartBuf[3] | ((uint16_t)uartBuf[4] << 8);

                uint8_t pf = prefetchForDev(dev);
                if (pf > MAX_PREFETCH_SECTORS) pf = MAX_PREFETCH_SECTORS;
                if (pf > 0) uartBuf[6] = pf;

                bool isWriteSector = (base == 0x50 || base == 0x57);
                bool isWritePercom = (base == 0x4F);

                bool toBt = btDiskDevEnabled(dev) || btDiskDevForced(dev);

                if (isWriteSector || isWritePercom) {
                  uint16_t opSec = isWritePercom ? 0xFFFF : sec;
                  g_pendingWriteRP.active = true;
                  g_pendingWriteRP.dev = dev;
                  g_pendingWriteRP.sec = opSec;
                  g_pendingWriteRP.toBt = toBt;
                }

                bool isRead = (base == 0x52);
                bool isFormatSD = (base == 0x21);
                bool isFormatDD = (base == 0x22);
                bool isStatus = (base == 0x53);
                bool isReadPercom = (base == 0x4E);

                g_lastMasterOp.active = (isRead || isWriteSector || isWritePercom || isFormatSD || isFormatDD || isStatus || isReadPercom);
                if (g_lastMasterOp.active) {
                  g_lastMasterOp.cmd = cmd;
                  g_lastMasterOp.dev = dev;
                  if (isRead || isWriteSector) g_lastMasterOp.sec = sec;
                  else if (isWritePercom || isReadPercom) g_lastMasterOp.sec = 0xFFFF;
                  else g_lastMasterOp.sec = 0;
                  g_lastMasterOp.sentMs = millis();
                }

                logf("[MASTER] CMD 0x%02X dev=%s sec=%u pf=%u route=%s",
                     (unsigned)cmd, devName(dev), (unsigned)sec, (unsigned)uartBuf[6],
                     routeNameForDev(dev));
                if (btDiskDevEnabled(dev)) {
                  g_btDiskLastCmd = cmd;
                  g_btDiskLastDev = dev;
                  g_btDiskLastSec = sec;
                  g_btDiskLastCmdMs = millis();
                }
              }

              uint8_t dev = uartBuf[2];
              routeFrameToBtOrEsp(dev, uartBuf, uartLen);
            } break;

            case TYPE_SECTOR_CHUNK: {
              if (uartLen < 6) break;

              uint8_t dev   = uartBuf[1];
              uint16_t sec  = (uint16_t)uartBuf[2] | ((uint16_t)uartBuf[3] << 8);
              uint8_t idx   = uartBuf[4];
              uint8_t count = uartBuf[5];

              bool isPercom = (sec == 0xFFFF);
              if (isPercom) {
                if (g_pendingWriteRP.toBt) sendBtDiskFrame(uartBuf, uartLen);
                else sendEspToSlave(dev, uartBuf, uartLen);
                g_pendingWriteRP.active = false;
                break;
              }

              if (!g_pendingWriteRP.active) break;
              if (dev != g_pendingWriteRP.dev || sec != g_pendingWriteRP.sec) break;

              if (g_pendingWriteRP.toBt) sendBtDiskFrame(uartBuf, uartLen);
              else sendEspToSlave(dev, uartBuf, uartLen);

              if ((uint8_t)(idx + 1) >= count) {
                g_pendingWriteRP.active = false;
              }
            } break;

            case TYPE_SUPERDOS_HINT: {
              // v15: ignorar hint en MASTER para mantener S/E lo mas parecido a v6
              break;
            }
          }
        }

        uartState = 0;
        uartIdx = 0;
        uartLen = 0;
      } break;
    }
  }

  if (uartState != 0 && (millis() - uartLastTime > 50)) {
    uartState = 0;
    uartIdx = 0;
    uartLen = 0;
    Serial.println(F("[MASTER] UART: timeout mid-frame, reseteando FSM."));
  }
}

// ===== ESP-NOW callbacks =====
#if ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* src = info ? info->src_addr : nullptr;
#else
void onDataRecv(const uint8_t* src, const uint8_t* data, int len) {
#endif
  if (!data || len <= 0) return;

  bool srcIsBcast = false;
  if (src) {
    srcIsBcast = true;
    for (int i = 0; i < 6; i++) {
      if (src[i] != 0xFF) { srcIsBcast = false; break; }
    }
    if (!srcIsBcast) {
      memcpy(g_lastSlave, src, 6);
      g_haveSlave = true;
      ensurePeer(g_lastSlave);
    }
  }

  uint8_t type = data[0];

    // Actualiza tabla de dispositivos por MAC (solo si no es broadcast)
  if (src && !srcIsBcast) {
    int di = allocDeviceSlot(src);
    if (di >= 0) {
      g_devices[di].present = true;
      g_devices[di].lastSeen = millis();
    }
  }

  // HELLO: el SLAVE anuncia unidad FISICA y si soporta 256B
  if (type == TYPE_HELLO && len >= 3 && src && !srcIsBcast) {
    // En el HELLO el SLAVE puede mandar:
    // [0]=TYPE_HELLO [1]=dev_payload (no confiable) [2]=supports256 [3]=physicalDev (opcional)
    uint8_t devPayload = data[1];
    uint8_t physDev = (len >= 4) ? data[3] : devPayload;
    bool supports256 = (data[2] != 0);

    int di = allocDeviceSlot(src);
    if (di >= 0) {
      g_devices[di].supports256 = supports256;
      g_devices[di].physicalDev = physDev;
      g_devices[di].present = true;
      g_devices[di].lastSeen = millis();
    }

    // Lógico = lo que está configurado por MAC (si existe)
    uint8_t logicDev = mapGetLogic(src);
    bool hasMapping = (logicDev >= DEV_MIN && logicDev <= DEV_MAX);

    // Si no hay mapeo y hay más de 1 equipo, NO aplicamos el HELLO a D1..D4 para evitar choques.
    // Igual queda listado en "devices" para que lo asignes desde la WEB.
    if (!hasMapping) {
      int presentCount = countPresentDevices();

      // "Modo 1 solo equipo": si hay 1 dispositivo presente, aceptamos su devPayload como lógico temporal
      // siempre que no esté reservado por otra MAC.
      if (presentCount <= 1 && devPayload >= DEV_MIN && devPayload <= DEV_MAX &&
          !logicReservedByOther(devPayload, src)) {
        logicDev = devPayload;
      } else {
        // No forward al RP (evita que el RP "vea" un D1 duplicado)
        return;
      }
    }

    // Aplica a slots lógicos
    clearSlotsByMac(src);
    applyDeviceToLogicalSlot(src, logicDev, supports256, physDev);
    ensurePeer(src);

    // Forward al RP, pero parcheando el byte DEV para que refleje el lógico real
    uint8_t helloBuf[8];
    int sendLen = len;
    if (sendLen > (int)sizeof(helloBuf)) sendLen = (int)sizeof(helloBuf);
    memcpy(helloBuf, data, sendLen);
    if (sendLen >= 2) helloBuf[1] = logicDev;
    if (sendLen >= 4) helloBuf[3] = physDev;

    sendUartFrameToRP(helloBuf, (uint8_t)sendLen);
    return;
  }

  // Para ACK/NAK/SECTOR: refresca presencia.
// Solo aplicamos a slots D1..D4 si:
// - existe mapeo por MAC, o
// - hay 1 solo equipo presente (modo simple), y el devPayload no está reservado por otra MAC.
  if ((type == TYPE_ACK || type == TYPE_NAK || type == TYPE_SECTOR_CHUNK) && len >= 2 && src && !srcIsBcast) {
    uint8_t devPayload = data[1];

    int di = allocDeviceSlot(src);
    bool supports256 = (di >= 0) ? g_devices[di].supports256 : false;
    uint8_t physDev  = (di >= 0) ? g_devices[di].physicalDev : 0;

    uint8_t logicDev = mapGetLogic(src);
    bool hasMapping = (logicDev >= DEV_MIN && logicDev <= DEV_MAX);

    if (!hasMapping) {
      bool oneOnly = (countPresentDevices() <= 1);
      if (oneOnly && devPayload >= DEV_MIN && devPayload <= DEV_MAX && !logicReservedByOther(devPayload, src)) {
        logicDev = devPayload;
      } else {
        // No tocar slots lógicos: el equipo queda disponible en "devices" para asignarlo desde la web.
        return;
      }
    }

    clearSlotsByMac(src);
    applyDeviceToLogicalSlot(src, logicDev, supports256, physDev);
    ensurePeer(src);
  }

  if (type == TYPE_SECTOR_CHUNK) {
    sendUartFrameToRP(data, (uint8_t)len);
    return;
  }

  if (type == TYPE_ACK || type == TYPE_NAK) {
    const char* kind = (type == TYPE_ACK) ? "ACK" : "NAK";

    if (g_lastMasterOp.active) {
      uint32_t dt = millis() - g_lastMasterOp.sentMs;
      int idx = devIndex(g_lastMasterOp.dev);
      if (idx >= 0) {
        g_driveTiming[idx].lastAckMs = dt;
        if (g_driveTiming[idx].avgAckMs == 0) g_driveTiming[idx].avgAckMs = dt;
        else g_driveTiming[idx].avgAckMs = (g_driveTiming[idx].avgAckMs * 3 + dt) / 4;
      }

      logf("[MASTER] %s cmd=0x%02X dev=%s sec=%u (%lu ms)",
           kind, (unsigned)g_lastMasterOp.cmd, devName(g_lastMasterOp.dev),
           (unsigned)g_lastMasterOp.sec, (unsigned long)dt);
    }

    clearLastMasterOpLocal();
    autoTuneTimingsFromAck();
    // V5: antes enviábamos TIMING_UPDATE después de cada ACK. Eso mete ruido
    // UART/Serial durante loaders rápidos. Ahora solo si cambió o cada 1.5s.
    sendTimingUpdateToRPThrottled();
    sendUartFrameToRP(data, (uint8_t)len);
    return;
  }
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t s) {
  (void)info;
#else
void onDataSent(const uint8_t* mac, esp_now_send_status_t s) {
  (void)mac;
#endif
  (void)s;
}

// ========= Web Handlers =========
static String webAtrLowerName(const String &v) {
  String out = v;
  out.toLowerCase();
  return out;
}

static bool webAtrIsBundledProtectedName(const String &name) {
  String n = webAtrSanitizeFileName(name);
  n.toLowerCase();
  return n == "dos_2.5pls.atr" || n == "mydos_4.55_4.atr" || n == "spartados_3.2g.atr";
}

static bool webAtrIsMountableVisibleName(String name) {
  name.trim();
  name.toLowerCase();
  return name.endsWith(".atr") || name.endsWith(".xex") || name.endsWith(".com") || name.endsWith(".exe") || name.endsWith(".bas") || name.endsWith(".cas") || name.endsWith(".sec");
}

static bool webAtrVectorContains(const std::vector<String>& names, const String& name) {
  for (const String& n : names) {
    if (n.equalsIgnoreCase(name)) return true;
  }
  return false;
}

static void webAtrPushNameUnique(std::vector<String>& names, String rawName) {
  rawName.replace("\\", "/");
  int slash = rawName.lastIndexOf('/');
  if (slash >= 0) rawName = rawName.substring(slash + 1);
  rawName.trim();
  if (!webAtrIsMountableVisibleName(rawName)) return;
  String clean = webAtrSanitizeFileName(rawName);
  if (clean.length() == 0) return;
  if (!webAtrVectorContains(names, clean)) names.push_back(clean);
}


// F42P: diagnóstico opcional del escaneo SD por consola.
// Se activa con /api/library/scan_console o con /api/library?refresh=1&manual=1&console=1.
// No cambia el índice ni escribe la SD por sí solo; solo imprime lo encontrado.
static bool     g_webLibraryConsoleScan = false;
static uint32_t g_webLibraryConsoleScanFound = 0;

// F34: índice de Biblioteca por ruta directa.
// El diagnóstico /api/fs confirma que /ATR y /CAS sí ven archivos. Para evitar
// volver a buscar cada nombre desde raíz durante el armado del índice, el escaneo
// conserva la ruta física encontrada y la usa directamente en el JSON.
static bool webLibraryScanEntryContains(const std::vector<WebLibraryScanEntry>& entries, const String& name) {
  for (const WebLibraryScanEntry& e : entries) {
    if (e.name.equalsIgnoreCase(name)) return true;
  }
  return false;
}

static void webLibraryPushEntryUnique(std::vector<WebLibraryScanEntry>& entries, String rawPath, uint32_t rawSize = 0) {
  rawPath.replace("\\", "/");
  rawPath.trim();
  if (!rawPath.length()) return;

  int slash = rawPath.lastIndexOf('/');
  String rawName = (slash >= 0) ? rawPath.substring(slash + 1) : rawPath;
  rawName.trim();
  if (!webAtrIsMountableVisibleName(rawName)) return;

  String clean = webAtrSanitizeFileName(rawName);
  if (!clean.length()) return;
  if (webLibraryScanEntryContains(entries, clean)) return;

  if (!rawPath.startsWith("/")) rawPath = String("/") + rawPath;

  WebLibraryScanEntry e;
  e.name = clean;
  e.path = rawPath;
  e.fileSize = rawSize;
  entries.push_back(e);
  if (g_webLibraryConsoleScan) {
    g_webLibraryConsoleScanFound++;
    String type = webAtrStoredTypeForName(clean);
    logf("[LIB-SD] FOUND #%lu type=%s name=%s path=%s",
      (unsigned long)g_webLibraryConsoleScanFound,
      type.c_str(), clean.c_str(), rawPath.c_str());
  }
}

static void webLibraryCollectEntriesRecursive(std::vector<WebLibraryScanEntry>& entries, const char* dirPath, uint8_t depth = 0) {
  if (!webAtrFsReady() || depth > 6) {
    if (g_webLibraryConsoleScan) logf("[LIB-SD] SKIP dir=%s ready=%u depth=%u", dirPath, (unsigned)webAtrFsReady(), (unsigned)depth);
    return;
  }
  String baseDir(dirPath);
  if (!baseDir.length()) baseDir = "/";
  baseDir.replace("\\", "/");
  if (!baseDir.startsWith("/")) baseDir = String("/") + baseDir;
  if (webAtrLibraryScanSkipDir(baseDir)) {
    if (g_webLibraryConsoleScan) logf("[LIB-SD] SKIP legacy dir=%s", baseDir.c_str());
    return;
  }
  if (g_webLibraryConsoleScan) logf("[LIB-SD] SCAN recursive depth=%u dir=%s", (unsigned)depth, baseDir.c_str());

  File root = SPIFFS.open(baseDir, "r");
  if (!root) {
    if (g_webLibraryConsoleScan) logf("[LIB-SD] OPEN FAIL dir=%s", baseDir.c_str());
    return;
  }
  if (!root.isDirectory()) {
    if (g_webLibraryConsoleScan) logf("[LIB-SD] NOT DIR path=%s", baseDir.c_str());
    root.close(); return;
  }

  File f = root.openNextFile();
  while (f) {
    String path = f.name();
    path.replace("\\", "/");
    bool isDir = f.isDirectory();
    uint32_t fSize = isDir ? 0 : (uint32_t)f.size();
    f.close();

    if (path.length() && !path.startsWith("/")) {
      String prefix = baseDir;
      if (!prefix.endsWith("/")) prefix += "/";
      path = prefix + path;
    }

    if (isDir) {
      if (!webAtrLibraryScanSkipDir(path)) webLibraryCollectEntriesRecursive(entries, path.c_str(), depth + 1);
    } else {
      webLibraryPushEntryUnique(entries, path, fSize);
    }
    f = root.openNextFile();
  }
  root.close();
}

static void webLibraryCollectEntriesShallow(std::vector<WebLibraryScanEntry>& entries, const char* dirPath) {
  if (!webAtrFsReady()) {
    if (g_webLibraryConsoleScan) logf("[LIB-SD] SHALLOW skip, storage not ready dir=%s", dirPath);
    return;
  }
  String baseDir(dirPath);
  if (!baseDir.length()) baseDir = "/";
  baseDir.replace("\\", "/");
  if (!baseDir.startsWith("/")) baseDir = String("/") + baseDir;
  if (webAtrLibraryScanSkipDir(baseDir)) return;
  if (g_webLibraryConsoleScan) logf("[LIB-SD] SCAN shallow dir=%s", baseDir.c_str());

  File root = SPIFFS.open(baseDir, "r");
  if (!root) {
    if (g_webLibraryConsoleScan) logf("[LIB-SD] OPEN FAIL shallow dir=%s", baseDir.c_str());
    return;
  }
  if (!root.isDirectory()) {
    if (g_webLibraryConsoleScan) logf("[LIB-SD] NOT DIR shallow path=%s", baseDir.c_str());
    root.close(); return;
  }

  File f = root.openNextFile();
  while (f) {
    String path = f.name();
    path.replace("\\", "/");
    bool isDir = f.isDirectory();
    uint32_t fSize = isDir ? 0 : (uint32_t)f.size();
    f.close();
    if (!isDir) {
      if (path.length() && !path.startsWith("/")) {
        String prefix = baseDir;
        if (!prefix.endsWith("/")) prefix += "/";
        path = prefix + path;
      }
      webLibraryPushEntryUnique(entries, path, fSize);
    }
    f = root.openNextFile();
  }
  root.close();
}

static void webLibraryCollectEntries(std::vector<WebLibraryScanEntry>& entries) {
  if (g_webLibraryConsoleScan) {
    g_webLibraryConsoleScanFound = 0;
    logf("[LIB-SD] ===== INICIO ESCANEO SD Biblioteca =====");
    logf("[LIB-SD] ready=%u backend=%s", (unsigned)webAtrFsReady(), WEB_STORAGE_NAME);
  }
#if WEB_STORAGE_USE_SD
  webLibraryCollectEntriesRecursive(entries, "/ATR");
  webLibraryCollectEntriesRecursive(entries, "/CAS");
  webLibraryCollectEntriesRecursive(entries, "/LIBRARY");
  webLibraryCollectEntriesRecursive(entries, "/SD_CARD_CONTENT/ATR");
  webLibraryCollectEntriesRecursive(entries, "/SD_CARD_CONTENT/CAS");
  webLibraryCollectEntriesShallow(entries, "/");
#else
  std::vector<String> names;
  webAtrCollectFiles(names);
  for (const String& name : names) {
    WebLibraryScanEntry e;
    e.name = name;
    e.path = webAtrPathForName(name);
    File nf = SPIFFS.open(e.path, "r");
    e.fileSize = nf ? (uint32_t)nf.size() : 0;
    if (nf) nf.close();
    entries.push_back(e);
  }
#endif
  if (g_webLibraryConsoleScan) {
    logf("[LIB-SD] ===== FIN ESCANEO SD Biblioteca: encontrados=%lu entries=%lu =====",
      (unsigned long)g_webLibraryConsoleScanFound,
      (unsigned long)entries.size());
  }
}

// [F24] Eliminado helper no usado: webAtrCollectFilesFromDir
static void webAtrCollectFilesRecursive(std::vector<String>& names, const char* dirPath, uint8_t depth = 0) {
  if (!webAtrFsReady() || depth > 6) return;
  String baseDir(dirPath);
  if (baseDir.length() == 0) baseDir = "/";
  if (webAtrLibraryScanSkipDir(baseDir)) return;
  File root = SPIFFS.open(baseDir, "r");
  if (!root) return;
  if (!root.isDirectory()) { root.close(); return; }

  File f = root.openNextFile();
  while (f) {
    String path = f.name();
    path.replace("\\", "/");
    bool isDir = f.isDirectory();
    uint32_t fSize = isDir ? 0 : (uint32_t)f.size();
    f.close();
    if (path.length() && !path.startsWith("/")) {
      String prefix = baseDir;
      if (!prefix.endsWith("/")) prefix += "/";
      path = prefix + path;
    }
    if (isDir) {
      if (!webAtrLibraryScanSkipDir(path)) webAtrCollectFilesRecursive(names, path.c_str(), depth + 1);
    } else {
      webAtrPushNameUnique(names, path);
    }
    f = root.openNextFile();
  }
  root.close();
}

static void webAtrCollectFilesShallow(std::vector<String>& names, const char* dirPath) {
  if (!webAtrFsReady()) return;
  String baseDir(dirPath);
  if (baseDir.length() == 0) baseDir = "/";
  if (webAtrLibraryScanSkipDir(baseDir)) return;
  File root = SPIFFS.open(baseDir, "r");
  if (!root) return;
  if (!root.isDirectory()) { root.close(); return; }

  File f = root.openNextFile();
  while (f) {
    String path = f.name();
    path.replace("\\", "/");
    bool isDir = f.isDirectory();
    uint32_t fSize = isDir ? 0 : (uint32_t)f.size();
    f.close();
    if (!isDir) {
      if (path.length() && !path.startsWith("/")) {
        String prefix = baseDir;
        if (!prefix.endsWith("/")) prefix += "/";
        path = prefix + path;
      }
      webAtrPushNameUnique(names, path);
    }
    f = root.openNextFile();
  }
  root.close();
}

static void webAtrCollectFiles(std::vector<String>& names) {
#if WEB_STORAGE_USE_SD
  // F33: no recorremos toda la SD desde raíz porque en tarjetas grandes puede
  // tardar decenas de segundos y terminar sin datos útiles. Escaneamos primero
  // las carpetas oficiales y también toleramos el caso común de haber copiado
  // la carpeta SD_CARD_CONTENT completa dentro de la tarjeta.
  webAtrCollectFilesRecursive(names, "/ATR");
  webAtrCollectFilesRecursive(names, "/CAS");
  webAtrCollectFilesRecursive(names, "/LIBRARY");
  webAtrCollectFilesRecursive(names, "/SD_CARD_CONTENT/ATR");
  webAtrCollectFilesRecursive(names, "/SD_CARD_CONTENT/CAS");
  webAtrCollectFilesShallow(names, "/");
#else
  File root = SPIFFS.open("/");
  if (root) {
    File f = root.openNextFile();
    while (f) {
      String path = f.name();
      if (!path.startsWith("/")) path = "/" + path;
      if (path.startsWith(WEB_ATR_PREFIX)) names.push_back(path.substring(strlen(WEB_ATR_PREFIX)));
      f.close();
      f = root.openNextFile();
    }
    root.close();
  }
#endif
}

static bool webAtrMoveFile(const String& from, const String& to, String& err) {
  if (from.length() == 0 || to.length() == 0) { err = "Ruta origen/destino inválida"; return false; }
  if (SPIFFS.exists(to)) SPIFFS.remove(to);
  if (SPIFFS.rename(from, to)) return true;

  // Fallback para SD/FATFS: algunos módulos fallan con rename entre carpetas.
  File src = SPIFFS.open(from, "r");
  if (!src) { err = "No se pudo abrir archivo temporal para mover"; return false; }
  File dst = SPIFFS.open(to, "w");
  if (!dst) { src.close(); err = "No se pudo crear destino en /ATR"; return false; }

  uint8_t buf[1024];
  bool ok = true;
  while (src.available()) {
    int n = src.read(buf, sizeof(buf));
    if (n <= 0) break;
    if (dst.write(buf, n) != (size_t)n) { ok = false; break; }
  }
  src.close();
  dst.close();
  if (!ok) {
    if (SPIFFS.exists(to)) SPIFFS.remove(to);
    err = "No se pudo copiar archivo temporal a /ATR";
    return false;
  }
  SPIFFS.remove(from);
  return true;
}



// F43: cache RAM temporal del escaneo vivo de Biblioteca.
// F42Y resolvió la estabilidad listando desde SD, pero la web puede pedir page 0/1/2
// varias veces al entrar. Reusar la lista ya escaneada baja la carga de 3-5 s por página
// a milisegundos mientras la SD no cambie. No reemplaza al índice persistente ni escribe SD.
static std::vector<WebLibraryScanEntry> g_webLibraryLiveScanCache;
static bool     g_webLibraryLiveScanCacheValid = false;
static uint32_t g_webLibraryLiveScanCacheAtMs = 0;
static uint32_t g_webLibraryLiveScanCacheBuildMs = 0;
static const uint32_t WEB_LIBRARY_LIVE_CACHE_TTL_MS = 60000;

// F43B: cache del JSON de página visible. Evita reconstruir files[] si la web
// pide varias veces la misma página al entrar/salir de Biblioteca.
static bool     g_webLibraryPageCacheValid = false;
static String   g_webLibraryPageCacheKey;
static String   g_webLibraryPageCacheEntries;
static uint32_t g_webLibraryPageCacheTotal = 0;
static uint32_t g_webLibraryPageCacheAll = 0;
static uint32_t g_webLibraryPageCacheAtr = 0;
static uint32_t g_webLibraryPageCacheXex = 0;
static uint32_t g_webLibraryPageCacheCom = 0;
static uint32_t g_webLibraryPageCacheExe = 0;
static uint32_t g_webLibraryPageCacheBas = 0;
static uint32_t g_webLibraryPageCacheCas = 0;
static uint32_t g_webLibraryPageCacheSec = 0;
static uint32_t g_webLibraryPageCacheOther = 0;
static uint32_t g_webLibraryPageCacheAtMs = 0;
static uint32_t g_webLibraryPageCacheScanMs = 0;
static const uint32_t WEB_LIBRARY_PAGE_CACHE_TTL_MS = 60000;
// F43E: límite máximo de objetos devueltos en UNA respuesta HTTP de Biblioteca.
// Esto NO limita el escaneo ni el total encontrado en SD; solo evita respuestas gigantes.
static const int WEB_LIBRARY_SAFE_HTTP_PAGE_SIZE = 50;
static const int WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX = WEB_LIBRARY_SAFE_HTTP_PAGE_SIZE;

static void webLibraryLiveScanCacheInvalidate(const char* reason) {
  g_webLibraryLiveScanCache.clear();
  g_webLibraryLiveScanCacheValid = false;
  g_webLibraryLiveScanCacheAtMs = 0;
  g_webLibraryLiveScanCacheBuildMs = 0;
  g_webLibraryPageCacheValid = false;
  g_webLibraryPageCacheKey = "";
  g_webLibraryPageCacheEntries = "";
  if (reason && reason[0]) logf("[LIB-IDX] F43G live/page cache invalidado: %s", reason);
}

static bool webLibraryLiveScanCacheGet(std::vector<WebLibraryScanEntry>& out, bool forceRefresh,
                                       bool& cacheHit, uint32_t& ageMs, uint32_t& buildMs) {
  cacheHit = false;
  ageMs = 0;
  buildMs = 0;
  if (!webAtrFsReady()) return false;

  uint32_t now = millis();
  if (g_webLibraryLiveScanCacheValid && !g_webLibraryLiveScanCache.empty() && !forceRefresh) {
    // F43C: navegación/status/Cassette NO re-escanean SD, aunque el cache
    // supere el TTL. El cache vive hasta que el usuario presiona Refrescar.
    ageMs = now - g_webLibraryLiveScanCacheAtMs;
    out = g_webLibraryLiveScanCache;
    buildMs = g_webLibraryLiveScanCacheBuildMs;
    cacheHit = true;
    return true;
  }

  if (!forceRefresh) {
    logf("[LIB-IDX] F43G auto scan bloqueado: cacheValid=%u entries=%lu",
         (unsigned)g_webLibraryLiveScanCacheValid,
         (unsigned long)(g_webLibraryLiveScanCacheValid ? g_webLibraryLiveScanCache.size() : 0));
    return false;
  }

  uint32_t t0 = millis();
  std::vector<WebLibraryScanEntry> entries;
  webLibraryCollectEntries(entries);
  std::sort(entries.begin(), entries.end(), [](const WebLibraryScanEntry &a, const WebLibraryScanEntry &b) {
    String aa = webAtrLowerName(a.name);
    String bb = webAtrLowerName(b.name);
    int c = aa.compareTo(bb);
    if (c == 0) return a.path.compareTo(b.path) < 0;
    return c < 0;
  });

  buildMs = millis() - t0;
  if (entries.empty()) {
    if (g_webLibraryLiveScanCacheValid && !g_webLibraryLiveScanCache.empty()) {
      out = g_webLibraryLiveScanCache;
      ageMs = now - g_webLibraryLiveScanCacheAtMs;
      buildMs = g_webLibraryLiveScanCacheBuildMs;
      cacheHit = true;
      logf("[LIB-IDX] F43F live scan manual devolvio 0; se conserva cache entries=%lu", (unsigned long)out.size());
      return true;
    }
    return false;
  }

  g_webLibraryLiveScanCache = entries;
  g_webLibraryLiveScanCacheValid = true;
  g_webLibraryLiveScanCacheAtMs = millis();
  g_webLibraryLiveScanCacheBuildMs = buildMs;
  out = entries;
  ageMs = 0;
  cacheHit = false;
  return true;
}

// F42Y: helper único para convertir una entrada de escaneo SD a objeto JSON de Biblioteca.
// Se usa para responder /api/library desde escaneo seguro paginado, sin depender
// de g_webLibraryIndexJson ni de cargar /CONFIG/library_index.json completo a RAM.
static void webLibraryAppendScanEntryJson(String &j, const WebLibraryScanEntry &entry) {
  // F43B: payload reducido por defecto. La página de Biblioteca no necesita toda
  // la metadata CAS ni leer cabeceras ATR en cada render. Si se requiere diagnóstico
  // completo se puede pedir full=1.
  const bool full = server.hasArg("full") && server.arg("full") == "1";
  String name = entry.name;
  String path = entry.path;
  String type = webAtrStoredTypeForName(name);
  bool isCas = (type == "CAS");
  bool isAtr = (type == "ATR");
  bool casThisMounted = isCas && g_casMounted && g_casMountedName.equalsIgnoreCase(name);

  // F43F: el listado normal debe ser rápido. El tamaño viene del escaneo SD
  // (f.size()) y NO se abre cada archivo para leer cabecera ATR. Abrir 50 ATR
  // por página costaba ~900-1200 ms incluso con cache de lista.
  uint32_t outFileSize = entry.fileSize;
  uint16_t outSectorSize = 0;
  uint32_t outTotalSectors = 0;
  bool ok = true;

  if (full) {
    File f2 = SPIFFS.open(path, "r");
    uint32_t rawSize = f2 ? (uint32_t)f2.size() : outFileSize;
    if (f2) f2.close();
    if (isAtr) {
      WebAtrMeta m;
      memset(&m, 0, sizeof(m));
      ok = webAtrReadMetaFromPath(path, m);
      outFileSize = ok ? m.fileSize : rawSize;
      outSectorSize = ok ? m.sectorSize : 0;
      outTotalSectors = ok ? m.totalSectors : 0;
    } else {
      outFileSize = rawSize;
      ok = rawSize > 0;
    }
  }

  j += "{\"name\":\"" + jsonEscape(name) + "\"";
  j += ",\"type\":\"" + type + "\"";
  j += ",\"protected\":" + String(webAtrIsBundledProtectedName(name) ? 1 : 0);
  j += ",\"path\":\"" + jsonEscape(path) + "\"";
  j += ",\"valid\":" + String(ok ? 1 : 0);
  bool webAtrMountable = false;
  if (type == "ATR") webAtrMountable = ok;
  else if ((type == "XEX" || type == "COM" || type == "EXE") && ok && outFileSize > 0) webAtrMountable = true;
  j += ",\"mountable\":" + String(webAtrMountable ? 1 : 0);
  j += ",\"fileSize\":" + String(outFileSize);
  j += ",\"sectorSize\":" + String(outSectorSize);
  j += ",\"totalSectors\":" + String(outTotalSectors);
  uint8_t mm = 0;
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) if (g_webAtrMountedName[i] == name) mm |= (1u << i);
  j += ",\"mountedMask\":" + String(mm);
  j += ",\"casMounted\":" + String(casThisMounted ? 1 : 0);
  j += ",\"casPlaying\":" + String((casThisMounted && g_casPlaying) ? 1 : 0);
  if (isCas) {
    String casOverrideProfile;
    bool hasCasOverride = casProfileOverrideGet(name, casOverrideProfile);
    String casEffectiveProfile = hasCasOverride ? casOverrideProfile : String("AUTO");
    bool casCached = false;
    CasAutoAnalysis cachedCas;
    if (full) {
      casCached = casAutoLoadCacheForPath(path, cachedCas);
      if (!hasCasOverride && casCached) casEffectiveProfile = String(cachedCas.profile);
    }
    j += ",\"casAnalyzed\":" + String(casCached ? 1 : 0);
    j += ",\"casOverride\":" + String(hasCasOverride ? 1 : 0);
    j += ",\"casOverrideProfile\":\"" + jsonEscape(hasCasOverride ? casOverrideProfile : String("AUTO")) + "\"";
    j += ",\"casEffectiveProfile\":\"" + jsonEscape(casEffectiveProfile) + "\"";
    if (full && casCached) {
      j += ",\"casProfile\":\"" + jsonEscape(String(cachedCas.profile)) + "\"";
      j += ",\"casAutoProfile\":\"" + jsonEscape(String(cachedCas.profile)) + "\"";
      j += ",\"casConfidence\":\"" + jsonEscape(String(cachedCas.confidence)) + "\"";
      j += ",\"casSuggestedMode\":\"" + jsonEscape(String(cachedCas.suggestedMode)) + "\"";
      j += ",\"casInitialBaud\":" + String((unsigned)cachedCas.initialBaud);
      j += ",\"casFirstDataBaud\":" + String((unsigned)cachedCas.firstDataBaud);
      j += ",\"casTurboBaud\":" + String((unsigned)cachedCas.turboBaud);
      j += ",\"casMaxBaud\":" + String((unsigned)cachedCas.maxBaud);
      j += ",\"casDataBlocks\":" + String((unsigned)cachedCas.dataBlocks);
      j += ",\"casBaudChunks\":" + String((unsigned)cachedCas.baudChunks);
      j += ",\"casFskChunks\":" + String((unsigned)cachedCas.fskChunks);
      j += ",\"casPwmChunks\":" + String((unsigned)cachedCas.pwmChunks);
    }
  }
  j += "}";
}
static bool webLibraryEntryMatchesLive(const WebLibraryScanEntry& entry, const String& qLower, const String& typeUpper) {
  String type = webAtrStoredTypeForName(entry.name);
  if (typeUpper.length() && typeUpper != "ALL" && type != typeUpper) return false;
  if (!qLower.length()) return true;
  String n = entry.name; n.toLowerCase();
  String p = entry.path; p.toLowerCase();
  return n.indexOf(qLower) >= 0 || p.indexOf(qLower) >= 0;
}

static bool webLibraryEntryMatchesLiveSearchOnly(const WebLibraryScanEntry& entry, const String& qLower) {
  if (!qLower.length()) return true;
  String n = entry.name; n.toLowerCase();
  String p = entry.path; p.toLowerCase();
  return n.indexOf(qLower) >= 0 || p.indexOf(qLower) >= 0;
}

// F42Y: respuesta visible de /api/library por escaneo seguro paginado.
// No lee /CONFIG/library_index.json, no toca g_webLibraryIndexJson y no escribe SD.
static bool webLibraryBuildLiveVisiblePage(int page, int pageSize, const String& qLower, const String& typeUpper,
                                           String& pageEntries, uint32_t& totalMatches,
                                           uint32_t& all, uint32_t& atr, uint32_t& xex, uint32_t& com, uint32_t& exe, uint32_t& bas, uint32_t& cas, uint32_t& sec, uint32_t& other,
                                           uint32_t& scanMs) {
  pageEntries = "";
  totalMatches = 0;
  all = atr = xex = com = exe = bas = cas = sec = other = 0;
  scanMs = 0;
  if (!webAtrFsReady()) return false;
  if (page < 0) page = 0;
  if (pageSize <= 0) pageSize = 50;
  if (pageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) pageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX; // F43E: solo paginación visual, no límite de escaneo

  uint32_t t0 = millis();
  const bool full = server.hasArg("full") && server.arg("full") == "1";
  String pageCacheKey = String(page) + "|" + String(pageSize) + "|" + typeUpper + "|" + qLower + "|" + String(full ? 1 : 0);
  bool forceScan = (server.hasArg("nocache") && server.arg("nocache") == "1") ||
                   ((server.hasArg("refresh") && server.arg("refresh") == "1") &&
                    ((server.hasArg("manual") && server.arg("manual") == "1") || (server.hasArg("confirm") && server.arg("confirm") == "1")) &&
                    !(server.hasArg("commit") && server.arg("commit") == "1"));
  if (!forceScan && g_webLibraryPageCacheValid && g_webLibraryPageCacheKey == pageCacheKey &&
      (millis() - g_webLibraryPageCacheAtMs) <= WEB_LIBRARY_PAGE_CACHE_TTL_MS) {
    pageEntries = g_webLibraryPageCacheEntries;
    totalMatches = g_webLibraryPageCacheTotal;
    all = g_webLibraryPageCacheAll;
    atr = g_webLibraryPageCacheAtr;
    xex = g_webLibraryPageCacheXex;
    com = g_webLibraryPageCacheCom;
    exe = g_webLibraryPageCacheExe;
    bas = g_webLibraryPageCacheBas;
    cas = g_webLibraryPageCacheCas;
    sec = g_webLibraryPageCacheSec;
    other = g_webLibraryPageCacheOther;
    scanMs = millis() - t0;
    logf("[LIB-IDX] F43G page json cache hit page=%d pageSize=%d pageBytes=%u ms=%lu",
         page, pageSize, (unsigned)pageEntries.length(), (unsigned long)scanMs);
    return true;
  }

  std::vector<WebLibraryScanEntry> entries;
  bool cacheHit = false;
  uint32_t cacheAgeMs = 0;
  uint32_t cacheBuildMs = 0;
  if (!webLibraryLiveScanCacheGet(entries, forceScan, cacheHit, cacheAgeMs, cacheBuildMs)) return false;

  uint32_t startWanted = (uint32_t)page * (uint32_t)pageSize;
  uint32_t added = 0;
  pageEntries.reserve(pageSize <= 50 ? 16384 : (pageSize <= 100 ? 32768 : 49152));

  for (const WebLibraryScanEntry& entry : entries) {
    String type = webAtrStoredTypeForName(entry.name);
    if (webLibraryEntryMatchesLiveSearchOnly(entry, qLower)) {
      all++;
      if (type == "ATR") atr++;
      else if (type == "XEX") xex++;
      else if (type == "COM") com++;
      else if (type == "EXE") exe++;
      else if (type == "BAS") bas++;
      else if (type == "CAS") cas++;
      else if (type == "SEC") sec++;
      else other++;
    }
    if (webLibraryEntryMatchesLive(entry, qLower, typeUpper)) {
      if (totalMatches >= startWanted && added < (uint32_t)pageSize) {
        if (pageEntries.length()) pageEntries += ",";
        webLibraryAppendScanEntryJson(pageEntries, entry);
        added++;
      }
      totalMatches++;
    }
    if ((totalMatches & 0x0F) == 0) yield();
  }
  scanMs = millis() - t0;
  g_webLibraryPageCacheValid = true;
  g_webLibraryPageCacheKey = pageCacheKey;
  g_webLibraryPageCacheEntries = pageEntries;
  g_webLibraryPageCacheTotal = totalMatches;
  g_webLibraryPageCacheAll = all;
  g_webLibraryPageCacheAtr = atr;
  g_webLibraryPageCacheXex = xex;
  g_webLibraryPageCacheCom = com;
  g_webLibraryPageCacheExe = exe;
  g_webLibraryPageCacheBas = bas;
  g_webLibraryPageCacheCas = cas;
  g_webLibraryPageCacheSec = sec;
  g_webLibraryPageCacheOther = other;
  g_webLibraryPageCacheAtMs = millis();
  g_webLibraryPageCacheScanMs = scanMs;
  logf("[LIB-IDX] F43G live page ok listCache=%u age=%lu entries=%lu total=%lu page=%d pageSize=%d pageBytes=%u ms=%lu scanBuildMs=%lu full=%u",
       (unsigned)(cacheHit ? 1 : 0), (unsigned long)cacheAgeMs,
       (unsigned long)entries.size(), (unsigned long)totalMatches, page, pageSize,
       (unsigned)pageEntries.length(), (unsigned long)scanMs, (unsigned long)cacheBuildMs, (unsigned)(full ? 1 : 0));

  return true;
}

// F43K/F43P: enviar /api/library por streaming y mantener páginas HTTP chicas.
// Evita construir un String gigante con files[]; en F43J con pageSize alto podía
// fragmentarse y terminar en JSON inválido: },]} o en objetos truncados.
static bool webLibraryStreamApiResponseFromEntries(const std::vector<WebLibraryScanEntry>& entries,
                                                   const String& source,
                                                   bool okFlag,
                                                   bool refreshed,
                                                   bool manualRefreshFlag,
                                                   bool indexCommitFlag,
                                                   bool ndjsonFlag,
                                                   bool commitReturnedFilesFlag,
                                                   int page,
                                                   int pageSize,
                                                   const String& qLower,
                                                   const String& typeUpper,
                                                   uint32_t indexBuildMs,
                                                   const String& extraError = String("")) {
  if (page < 0) page = 0;
  if (pageSize <= 0) pageSize = 50;
  if (pageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) pageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX;

  uint32_t all = 0, atr = 0, xex = 0, com = 0, exe = 0, bas = 0, cas = 0, sec = 0, other = 0;
  uint32_t totalMatches = 0;
  for (const WebLibraryScanEntry& entry : entries) {
    String type = webAtrStoredTypeForName(entry.name);
    if (webLibraryEntryMatchesLiveSearchOnly(entry, qLower)) {
      all++;
      if (type == "ATR") atr++;
      else if (type == "XEX") xex++;
      else if (type == "COM") com++;
      else if (type == "EXE") exe++;
      else if (type == "BAS") bas++;
      else if (type == "CAS") cas++;
      else if (type == "SEC") sec++;
      else other++;
    }
    if (webLibraryEntryMatchesLive(entry, qLower, typeUpper)) totalMatches++;
    if ((all & 0x1F) == 0) yield();
  }

  String head;
  head.reserve(4096);
  head = "{\"ok\":" + String(okFlag ? "true" : "false");
  head += ",\"source\":\"" + jsonEscape(source) + "\"";
  head += ",\"refreshed\":" + String(refreshed ? "true" : "false");
  head += ",\"manualRefresh\":" + String(manualRefreshFlag ? "true" : "false");
  head += ",\"indexCommit\":" + String(indexCommitFlag ? "true" : "false");
  head += ",\"ndjson\":" + String(ndjsonFlag ? "true" : "false");
  head += ",\"autoScanDisabled\":true";
  if (indexCommitFlag) head += ",\"commitReturnedFiles\":" + String(commitReturnedFilesFlag ? "true" : "false");
  head += ",\"streamedFiles\":true";
  head += ",\"indexPath\":\"" + jsonEscape(String("/CONFIG/library_index.ndjson")) + "\"";
  head += ",\"legacyJsonPathDisabled\":\"/CONFIG/library_index.json\"";
  head += ",\"indexCount\":" + String((unsigned)entries.size());
  head += ",\"indexBuildMs\":" + String(indexBuildMs);
  head += ",\"indexLoadMs\":0";
  if (extraError.length()) head += ",\"streamError\":\"" + jsonEscape(extraError) + "\"";
  webLibraryAppendSaveStatusJson(head);
  head += ",\"page\":" + String(page);
  head += ",\"pageSize\":" + String(pageSize);
  head += ",\"total\":" + String(totalMatches);
  head += ",\"typeCounts\":{";
  head += "\"ALL\":" + String(all);
  head += ",\"ATR\":" + String(atr);
  head += ",\"XEX\":" + String(xex);
  head += ",\"COM\":" + String(com);
  head += ",\"EXE\":" + String(exe);
  head += ",\"BAS\":" + String(bas);
  head += ",\"CAS\":" + String(cas);
  head += ",\"SEC\":" + String(sec);
  head += ",\"OTHER\":" + String(other);
  head += "},\"files\":[";

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.sendHeader("Connection", "close");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent(head);

  uint32_t startWanted = (uint32_t)page * (uint32_t)pageSize;
  uint32_t seenMatches = 0;
  uint32_t added = 0;
  bool first = true;
  for (const WebLibraryScanEntry& entry : entries) {
    if (webLibraryEntryMatchesLive(entry, qLower, typeUpper)) {
      if (seenMatches >= startWanted && added < (uint32_t)pageSize) {
        if (!first) server.sendContent(",");
        String obj;
        obj.reserve(768);
        webLibraryAppendScanEntryJson(obj, entry);
        server.sendContent(obj);
        first = false;
        added++;
        if ((added & 0x07) == 0) yield();
      }
      seenMatches++;
    }
  }
  server.sendContent("]}");
  server.sendContent("");
  logf("[LIB-IDX] F43K streamed library response source=%s entries=%lu total=%lu page=%d pageSize=%d sent=%lu buildMs=%lu",
       source.c_str(), (unsigned long)entries.size(), (unsigned long)totalMatches,
       page, pageSize, (unsigned long)added, (unsigned long)indexBuildMs);
  return true;
}

static void webAtrAppendFileListJson(String &j) {
  std::vector<WebLibraryScanEntry> entries;
  webLibraryCollectEntries(entries);

  std::sort(entries.begin(), entries.end(), [](const WebLibraryScanEntry &a, const WebLibraryScanEntry &b) {
    String aa = webAtrLowerName(a.name);
    String bb = webAtrLowerName(b.name);
    int c = aa.compareTo(bb);
    if (c == 0) return a.name.compareTo(b.name) < 0;
    return c < 0;
  });

  bool first = true;
  for (const WebLibraryScanEntry &entry : entries) {
    String name = entry.name;
    String path = entry.path;
    String type = webAtrStoredTypeForName(name);
    bool isCas = (type == "CAS");
    bool isAtr = (type == "ATR");

    File f2 = SPIFFS.open(path, "r");
    uint32_t rawSize = f2 ? (uint32_t)f2.size() : 0;
    if (f2) f2.close();

    WebAtrMeta m;
    memset(&m, 0, sizeof(m));
    // F35: sólo los ATR necesitan leer cabecera/metadata. Para XEX/COM/EXE/BAS/CAS
    // basta el tamaño del archivo; intentar parsearlos como ATR ralentizaba mucho el índice.
    bool ok = isAtr ? webAtrReadMetaFromPath(path, m) : (rawSize > 0);
    uint32_t outFileSize = isAtr ? (ok ? m.fileSize : rawSize) : rawSize;
    uint16_t outSectorSize = isAtr ? (ok ? m.sectorSize : 0) : 0;
    uint32_t outTotalSectors = isAtr ? (ok ? m.totalSectors : 0) : 0;
    bool casThisMounted = isCas && g_casMounted && g_casMountedName.equalsIgnoreCase(name);
    if (!first) j += ",";
    first = false;
    j += "{\"name\":\"" + jsonEscape(name) + "\"";
    j += ",\"type\":\"" + type + "\"";
    j += ",\"protected\":" + String(webAtrIsBundledProtectedName(name) ? 1 : 0);
    j += ",\"path\":\"" + jsonEscape(path) + "\"";
    j += ",\"valid\":" + String(ok ? 1 : 0);
    j += ",\"fileSize\":" + String(outFileSize);
    j += ",\"sectorSize\":" + String(outSectorSize);
    j += ",\"totalSectors\":" + String(outTotalSectors);
    uint8_t mm = 0;
    for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) if (g_webAtrMountedName[i] == name) mm |= (1u << i);
    j += ",\"mountedMask\":" + String(mm);
    j += ",\"casMounted\":" + String(casThisMounted ? 1 : 0);
    j += ",\"casPlaying\":" + String((casThisMounted && g_casPlaying) ? 1 : 0);
    if (isCas) {
      CasAutoAnalysis cachedCas;
      bool casCached = casAutoLoadCacheForPath(path, cachedCas);
      String casOverrideProfile;
      bool hasCasOverride = casProfileOverrideGet(name, casOverrideProfile);
      String casEffectiveProfile = hasCasOverride ? casOverrideProfile : (casCached ? String(cachedCas.profile) : String("AUTO"));
      j += ",\"casAnalyzed\":" + String(casCached ? 1 : 0);
      j += ",\"casOverride\":" + String(hasCasOverride ? 1 : 0);
      j += ",\"casOverrideProfile\":\"" + jsonEscape(hasCasOverride ? casOverrideProfile : String("AUTO")) + "\"";
      j += ",\"casEffectiveProfile\":\"" + jsonEscape(casEffectiveProfile) + "\"";
      if (casCached) {
        j += ",\"casProfile\":\"" + jsonEscape(String(cachedCas.profile)) + "\"";
        j += ",\"casAutoProfile\":\"" + jsonEscape(String(cachedCas.profile)) + "\"";
        j += ",\"casConfidence\":\"" + jsonEscape(String(cachedCas.confidence)) + "\"";
        j += ",\"casSuggestedMode\":\"" + jsonEscape(String(cachedCas.suggestedMode)) + "\"";
        j += ",\"casInitialBaud\":" + String((unsigned)cachedCas.initialBaud);
        j += ",\"casFirstDataBaud\":" + String((unsigned)cachedCas.firstDataBaud);
        j += ",\"casTurboBaud\":" + String((unsigned)cachedCas.turboBaud);
        j += ",\"casMaxBaud\":" + String((unsigned)cachedCas.maxBaud);
        j += ",\"casDataBlocks\":" + String((unsigned)cachedCas.dataBlocks);
        j += ",\"casBaudChunks\":" + String((unsigned)cachedCas.baudChunks);
        j += ",\"casFskChunks\":" + String((unsigned)cachedCas.fskChunks);
        j += ",\"casPwmChunks\":" + String((unsigned)cachedCas.pwmChunks);
      }
    }
    j += "}";
  }
}// F13: cache liviana para la lista de ATR/XEX.
// Escanear /ATR, /LIBRARY y raíz en cada /api/atr/status bloquea el WebServer
// cuando hay muchos archivos o la microSD responde lento. La cache se invalida
// al subir, borrar, montar o desmontar, y además expira sola por seguridad.
static String   g_webAtrFilesJsonCache;
static uint32_t g_webAtrFilesJsonCacheMs = 0;
static uint32_t g_webAtrFilesJsonBuildMs = 0;
static uint32_t g_webAtrFilesJsonHits = 0;
static uint32_t g_webAtrFilesJsonCount = 0;
static bool     g_webAtrFilesJsonValid = false;
static const uint32_t WEB_ATR_FILES_CACHE_TTL_MS = 30000UL; // F49Z24: menos escaneos SD al navegar/refrescar

// F49Z42: índice persistente de Biblioteca. Guarda la lista ya armada en SD
// para evitar escanear /ATR, /CAS, /LIBRARY y raíz cada vez que se abre la web/app.
static const char WEB_LIBRARY_INDEX_DIR[] = "/CONFIG";
static const char WEB_LIBRARY_INDEX_PATH[] = "/CONFIG/library_index.json";
static const char WEB_LIBRARY_INDEX_BAK_PATH[] = "/CONFIG/library_index.bak.json";
static const char WEB_LIBRARY_INDEX_TMP_PATH[] = "/CONFIG/library_index.tmp";
static const char WEB_LIBRARY_INDEX_PREV_PATH[] = "/CONFIG/library_index.prev.json";
// F43J: índice persistente en NDJSON. Cada línea es un objeto JSON independiente.
// Esto evita cargar un JSON gigante completo a RAM y elimina problemas de coma final/len=0.
static const char WEB_LIBRARY_INDEX_NDJSON_PATH[] = "/CONFIG/library_index.ndjson";
static const char WEB_LIBRARY_INDEX_NDJSON_BAK_PATH[] = "/CONFIG/library_index.bak.ndjson";
static const char WEB_LIBRARY_INDEX_NDJSON_TMP_PATH[] = "/CONFIG/library_index.tmp.ndjson";
static const char WEB_LIBRARY_INDEX_NDJSON_PREV_PATH[] = "/CONFIG/library_index.prev.ndjson";
static String   g_webLibraryIndexJson;
static bool     g_webLibraryIndexLoaded = false;
static bool     g_webLibraryIndexDirty = false;
static uint32_t g_webLibraryIndexBuildMs = 0;
static uint32_t g_webLibraryIndexLoadMs = 0;
static uint32_t g_webLibraryIndexCount = 0;
static uint32_t g_webLibraryIndexHits = 0;
static bool     g_webLibraryIndexLastSaveOk = false;
static bool     g_webLibraryIndexLastVerifyOk = false;
static uint32_t g_webLibraryIndexLastSaveMs = 0;
static uint32_t g_webLibraryIndexLastSavedBytes = 0;
static uint32_t g_webLibraryIndexLastVerifyCount = 0;
static String   g_webLibraryIndexLastSaveError;

static bool webLibraryConsoleScanArgEnabled() {
  return (server.hasArg("console") && server.arg("console") == "1") ||
         (server.hasArg("log") && server.arg("log") == "1") ||
         (server.hasArg("serial") && server.arg("serial") == "1");
}

static uint32_t webLibraryCountObjects(const String& entries) {
  uint32_t count = 0;
  int p = 0;
  while ((p = entries.indexOf("\"name\"", p)) >= 0) { count++; p += 8; }
  return count;
}

// F42B: defensa final contra JSON de biblioteca con coma colgante.
// Algunas rutas legacy/F41 podían dejar el índice interno o persistido como:
//   { ... },]
// Eso rompe JSON.parse() en la web. Normalizamos SIEMPRE la parte interna
// del array antes de guardar, validar o servir /api/library.
static void webLibraryNormalizeEntriesJson(String& entries) {
  entries.trim();
  if (entries.startsWith("[")) entries = entries.substring(1);
  if (entries.endsWith("]")) entries = entries.substring(0, entries.length() - 1);
  entries.trim();

  // F42F: corregir separadores dobles entre objetos generados por rutas
  // intermedias/streaming anteriores:  {...},,{...}
  // No hacemos replace global de ",," para no tocar texto dentro de strings.
  int guard = 0;
  while (entries.indexOf("},,{") >= 0 && guard++ < 32) {
    entries.replace("},,{", "},{");
  }

  while (entries.endsWith(",")) {
    entries.remove(entries.length() - 1);
    entries.trim();
  }
}

static String webLibraryNormalizedEntriesCopy(const String& in) {
  String out = in;
  webLibraryNormalizeEntriesJson(out);
  return out;
}

// F49Z44: evita servir un indice corrupto creado por una version anterior.
// Valida la secuencia de objetos interna: {..},{..}. Ignora llaves dentro
// de strings JSON escapados. Si falla, se reconstruye desde la SD.
static bool webLibraryEntriesLooksValid(const String& entriesIn) {
  String entries = entriesIn;
  entries.trim();
  if (entries.length() == 0) return true;
  if (entries.startsWith("[") || entries.endsWith("]")) return false;
  if (entries.endsWith(",")) return false;

  bool inString = false;
  bool esc = false;
  int level = 0;
  bool sawObject = false;
  for (int i = 0; i < (int)entries.length(); i++) {
    char c = entries[i];
    if (inString) {
      if (esc) { esc = false; continue; }
      if (c == '\\') { esc = true; continue; }
      if (c == '"') inString = false;
      continue;
    }
    if (c == '"') { inString = true; continue; }
    if (c == '{') { level++; sawObject = true; continue; }
    if (c == '}') {
      level--;
      if (level < 0) return false;
      continue;
    }
  }
  if (inString || level != 0) return false;
  if (!sawObject) return entries.length() == 0;
  // F37: no usar busquedas crudas tipo ",," o "},}" porque pueden aparecer
  // dentro de nombres/rutas escapadas y provocar falsos negativos. La validacion
  // importante aqui es que no venga con [] externos, que no termine en coma y que
  // las llaves esten balanceadas ignorando strings JSON.
  return true;
}

static bool webLibraryDirExists(const char* path) {
  if (!webAtrFsReady()) return false;
  File d = SPIFFS.open(path, "r");
  if (!d) return false;
  bool isDir = d.isDirectory();
  d.close();
  return isDir;
}

static bool webLibraryEnsureConfigDir() {
  if (!webAtrFsReady()) return false;

  // En SD.exists() no basta: puede existir un archivo llamado /CONFIG,
  // lo que impide crear /CONFIG/library_index.json. Verificamos que sea carpeta.
  if (webLibraryDirExists(WEB_LIBRARY_INDEX_DIR)) return true;

  if (SPIFFS.exists(WEB_LIBRARY_INDEX_DIR)) {
    SPIFFS.remove(WEB_LIBRARY_INDEX_DIR);
    delay(5);
  }

  if (!SPIFFS.mkdir(WEB_LIBRARY_INDEX_DIR)) {
    delay(25);
    if (!SPIFFS.mkdir(WEB_LIBRARY_INDEX_DIR) && !webLibraryDirExists(WEB_LIBRARY_INDEX_DIR)) return false;
  }

  return webLibraryDirExists(WEB_LIBRARY_INDEX_DIR);
}

static bool webLibraryLoadEntriesFromPath(const char* path, String& out, uint32_t* countOut = nullptr) {
  out = "";
  if (countOut) *countOut = 0;
  if (!webAtrFsReady() || !SPIFFS.exists(path)) return false;

  File f = SPIFFS.open(path, "r");
  if (!f) return false;

  // F42V: no usar File.readString() NI depender de f.available().
  // En esta placa se observo: save/verify OK bytes=36686, pero luego
  // load count=0 len=0. La causa probable es que available() puede iniciar en 0
  // para FAT/SD aunque f.size() sea correcto. Leemos por tamaño real.
  uint32_t expectedSize = (uint32_t)f.size();
  if (expectedSize > 0 && expectedSize < 262144UL) out.reserve(expectedSize + 1);

  char buf[257];
  uint32_t readBytesTotal = 0;
  uint16_t idleLoops = 0;
  if (expectedSize > 0) {
    f.seek(0);
    while (readBytesTotal < expectedSize) {
      uint32_t remain = expectedSize - readBytesTotal;
      size_t want = remain > 256 ? 256 : (size_t)remain;
      int n = f.read((uint8_t*)buf, want);
      if (n <= 0) {
        if (++idleLoops > 80) break;
        delay(1);
        yield();
        continue;
      }
      idleLoops = 0;
      buf[n] = 0;
      out += buf;
      readBytesTotal += (uint32_t)n;
      if ((readBytesTotal & 0x0FFF) == 0) yield();
    }
  } else {
    // Fallback para FS que no informan size().
    while (f.available()) {
      int n = f.read((uint8_t*)buf, 256);
      if (n <= 0) {
        if (++idleLoops > 80) break;
        delay(1);
        yield();
        continue;
      }
      idleLoops = 0;
      buf[n] = 0;
      out += buf;
      readBytesTotal += (uint32_t)n;
      if ((readBytesTotal & 0x0FFF) == 0) yield();
    }
  }
  f.close();

  if (expectedSize > 0 && readBytesTotal < expectedSize) {
    logf("[LIB-IDX] load parcial path=%s read=%lu expected=%lu len=%u", path, (unsigned long)readBytesTotal, (unsigned long)expectedSize, (unsigned)out.length());
  }

  webLibraryNormalizeEntriesJson(out);
  if (!webLibraryEntriesLooksValid(out)) {
    logf("[LIB-IDX] load invalido path=%s len=%u", path, (unsigned)out.length());
    return false;
  }

  uint32_t c = webLibraryCountObjects(out);
  if (countOut) *countOut = c;
  if (c == 0) {
    logf("[LIB-IDX] load count=0 path=%s len=%u", path, (unsigned)out.length());
    return false;
  }

  logf("[LIB-IDX] load ok path=%s count=%lu bytes=%lu/%lu", path, (unsigned long)c, (unsigned long)readBytesTotal, (unsigned long)expectedSize);
  return true;
}

static bool webLibraryIndexCopyFile(const char* from, const char* to);

static bool webLibraryIndexLoadBestFromFs(String& bestEntries, uint32_t& bestCount, String* bestSource = nullptr) {
  bestEntries = "";
  bestCount = 0;
  if (bestSource) *bestSource = "";

  const char* paths[] = {
    WEB_LIBRARY_INDEX_PATH,
    WEB_LIBRARY_INDEX_BAK_PATH,
    WEB_LIBRARY_INDEX_PREV_PATH
  };
  const char* names[] = { "main", "bak", "prev" };

  for (int i = 0; i < 3; i++) {
    String e;
    uint32_t c = 0;
    if (webLibraryLoadEntriesFromPath(paths[i], e, &c) && c > bestCount) {
      bestEntries = e;
      bestCount = c;
      if (bestSource) *bestSource = names[i];
    }
  }

  if (g_webLibraryIndexJson.length() && webLibraryEntriesLooksValid(g_webLibraryIndexJson)) {
    String e = g_webLibraryIndexJson;
    webLibraryNormalizeEntriesJson(e);
    uint32_t c = webLibraryCountObjects(e);
    if (c > bestCount) {
      bestEntries = e;
      bestCount = c;
      if (bestSource) *bestSource = "ram";
    }
  }
  return bestCount > 0;
}

static bool webLibraryIndexLoadFromFs() {
  uint32_t t0 = millis();
  String entries;
  uint32_t count = 0;
  String source;
  bool ok = webLibraryIndexLoadBestFromFs(entries, count, &source);

  if (!ok) {
    g_webLibraryIndexJson = "";
    g_webLibraryIndexCount = 0;
    g_webLibraryIndexLoaded = false;
    g_webLibraryIndexDirty = false;
    g_webLibraryIndexLoadMs = millis() - t0;
    return false;
  }

  g_webLibraryIndexJson = entries;
  g_webLibraryIndexCount = count;
  g_webLibraryIndexLoaded = true;
  g_webLibraryIndexDirty = false;
  g_webLibraryIndexLoadMs = millis() - t0;

  // F42J: si el índice principal fue dañado/vaciado pero el backup está bueno,
  // restaurar el principal desde la mejor fuente. Nunca se restaura desde 0.
  if (source != "main" && count > 0 && webLibraryEnsureConfigDir()) {
    const char* src = (source == "bak") ? WEB_LIBRARY_INDEX_BAK_PATH : ((source == "prev") ? WEB_LIBRARY_INDEX_PREV_PATH : nullptr);
    if (src) {
      bool restored = webLibraryIndexCopyFile(src, WEB_LIBRARY_INDEX_PATH);
      logf("[LIB-IDX] main restaurado desde %s count=%u ok=%u", source.c_str(), (unsigned)count, (unsigned)restored);
    }
  }

  return true;
}

static bool webLibraryIndexVerifyFileStreamAt(const char* path, uint32_t expectedCount, uint32_t expectedBytes, bool& bracketOk, uint32_t& verifyCount, uint32_t& fileBytes) {
  bracketOk = false;
  verifyCount = 0;
  fileBytes = 0;
  if (!webAtrFsReady() || !SPIFFS.exists(path)) return false;

  File vf = SPIFFS.open(path, "r");
  if (!vf) return false;
  fileBytes = (uint32_t)vf.size();

  const char* pat = "\"name\"";
  const uint8_t patLen = 6;
  uint8_t mi = 0;
  char first = 0;
  char last = 0;

  while (vf.available()) {
    char c = (char)vf.read();
    if (c > ' ') {
      if (!first) first = c;
      last = c;
    }
    if (c == pat[mi]) {
      mi++;
      if (mi >= patLen) { verifyCount++; mi = 0; }
    } else {
      mi = (c == pat[0]) ? 1 : 0;
    }
  }
  vf.close();

  bracketOk = (first == '[' && last == ']');
  bool sizeOk = (expectedBytes == 0) || (fileBytes == expectedBytes);
  bool countOk = (verifyCount == expectedCount);
  return bracketOk && sizeOk && countOk;
}

static bool webLibraryIndexVerifyFileStream(uint32_t expectedCount, uint32_t expectedBytes, bool& bracketOk, uint32_t& verifyCount, uint32_t& fileBytes) {
  return webLibraryIndexVerifyFileStreamAt(WEB_LIBRARY_INDEX_PATH, expectedCount, expectedBytes, bracketOk, verifyCount, fileBytes);
}

static bool webLibraryIndexWriteBracketedFile(const char* path, const String& entries, uint32_t& writtenOut) {
  writtenOut = 0;
  if (SPIFFS.exists(path)) SPIFFS.remove(path);
  File f = SPIFFS.open(path, "w");
  if (!f) return false;

  bool ok = true;
  size_t w = f.write((const uint8_t*)"[", 1);
  if (w != 1) ok = false;
  writtenOut += (uint32_t)w;

  const char* data = entries.c_str();
  size_t len = entries.length();
  const size_t CHUNK = 512;
  for (size_t off = 0; ok && off < len; off += CHUNK) {
    size_t n = len - off;
    if (n > CHUNK) n = CHUNK;
    w = f.write((const uint8_t*)(data + off), n);
    writtenOut += (uint32_t)w;
    if (w != n) ok = false;
    yield();
  }

  w = f.write((const uint8_t*)"]", 1);
  if (w != 1) ok = false;
  writtenOut += (uint32_t)w;
  f.flush();
  f.close();

  if (!ok) {
    if (SPIFFS.exists(path)) SPIFFS.remove(path);
    return false;
  }
  return true;
}

static bool webLibraryIndexCopyFile(const char* from, const char* to) {
  File src = SPIFFS.open(from, "r");
  if (!src) return false;
  if (SPIFFS.exists(to)) SPIFFS.remove(to);
  File dst = SPIFFS.open(to, "w");
  if (!dst) { src.close(); return false; }
  uint8_t buf[512];
  bool ok = true;
  while (src.available()) {
    int n = src.read(buf, sizeof(buf));
    if (n <= 0) break;
    if (dst.write(buf, n) != (size_t)n) { ok = false; break; }
    yield();
  }
  src.close();
  dst.flush();
  dst.close();
  if (!ok && SPIFFS.exists(to)) SPIFFS.remove(to);
  return ok;
}

static bool webLibraryIndexSaveToFsFromEntries(const String& entriesIn, uint32_t expectedCount) {
  uint32_t t0 = millis();
  g_webLibraryIndexLastSaveOk = false;
  g_webLibraryIndexLastVerifyOk = false;
  g_webLibraryIndexLastSavedBytes = 0;
  g_webLibraryIndexLastVerifyCount = 0;
  g_webLibraryIndexLastSaveError = "";

  if (!webLibraryEnsureConfigDir()) {
    g_webLibraryIndexLastSaveError = "No se pudo crear/acceder /CONFIG";
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }

  // F42S: evitar duplicar un String grande (~30-40 KB) antes de guardar.
  // En F42N/O el escaneo encontraba 148 archivos, pero la copia local podia
  // quedar vacia por fragmentacion/heap y fallaba con local=0 expected=148.
  // Si entriesIn ya viene como lista interna normalizada ({...},{...}), se usa
  // directamente por referencia. Solo se crea copia si viene con [ ] externos
  // o con coma colgante heredada.
  String entriesNormalizedCopy;
  const String* entriesPtr = &entriesIn;
  bool needsNormalizeCopy = entriesIn.startsWith("[") || entriesIn.endsWith("]") || entriesIn.endsWith(",") || (entriesIn.indexOf("},,{") >= 0);
  if (needsNormalizeCopy) {
    entriesNormalizedCopy = entriesIn;
    webLibraryNormalizeEntriesJson(entriesNormalizedCopy);
    entriesPtr = &entriesNormalizedCopy;
  }
  const String& entries = *entriesPtr;

  bool entriesValidForSave = webLibraryEntriesLooksValid(entries);
  uint32_t localCount = webLibraryCountObjects(entries);
  if (!entriesValidForSave) {
    g_webLibraryIndexLastSaveError = "JSON interno inválido antes de guardar índice";
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }
  if (expectedCount > 0 && localCount != expectedCount) {
    g_webLibraryIndexLastSaveError = String("RAM/index mismatch antes de guardar: local=") + String(localCount) + " expected=" + String(expectedCount);
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }
  if (expectedCount > 0 && entries.length() < 4) {
    g_webLibraryIndexLastSaveError = "Protegido: no se escribe índice vacío/casi vacío cuando el conteo esperado es mayor que 0";
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }

  // F42I: guardado transaccional. Primero escribir TMP, verificar TMP,
  // y solo entonces reemplazar el índice real. Si falla, el índice anterior queda intacto.
  uint32_t written = 0;
  if (!webLibraryIndexWriteBracketedFile(WEB_LIBRARY_INDEX_TMP_PATH, entries, written)) {
    g_webLibraryIndexLastSavedBytes = written;
    g_webLibraryIndexLastSaveError = "No se pudo escribir library_index.tmp completo";
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }
  g_webLibraryIndexLastSavedBytes = written;

  delay(10);

  bool bracketOk = false;
  uint32_t verifyCount = 0;
  uint32_t fileBytes = 0;
  bool tmpValid = webLibraryIndexVerifyFileStreamAt(WEB_LIBRARY_INDEX_TMP_PATH, expectedCount, g_webLibraryIndexLastSavedBytes, bracketOk, verifyCount, fileBytes);
  if (!tmpValid) {
    g_webLibraryIndexLastVerifyCount = verifyCount;
    g_webLibraryIndexLastVerifyOk = false;
    g_webLibraryIndexLastSaveOk = false;
    g_webLibraryIndexLastSaveMs = millis() - t0;
    g_webLibraryIndexLastSaveError = String("TMP inválido: bracket=") + String(bracketOk ? 1 : 0) +
      " count=" + String(verifyCount) + " expected=" + String(expectedCount) +
      " bytes=" + String(fileBytes) + "/" + String(g_webLibraryIndexLastSavedBytes) +
      "; índice real conservado";
    if (SPIFFS.exists(WEB_LIBRARY_INDEX_TMP_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_TMP_PATH);
    return false;
  }

  // Reemplazo con posibilidad de rollback simple.
  if (SPIFFS.exists(WEB_LIBRARY_INDEX_PREV_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_PREV_PATH);
  bool hadMain = SPIFFS.exists(WEB_LIBRARY_INDEX_PATH);
  if (hadMain) {
    if (!SPIFFS.rename(WEB_LIBRARY_INDEX_PATH, WEB_LIBRARY_INDEX_PREV_PATH)) {
      // Si rename falla en alguna SD, intentamos al menos copiar el índice anterior.
      webLibraryIndexCopyFile(WEB_LIBRARY_INDEX_PATH, WEB_LIBRARY_INDEX_PREV_PATH);
      SPIFFS.remove(WEB_LIBRARY_INDEX_PATH);
    }
  }

  bool promoted = SPIFFS.rename(WEB_LIBRARY_INDEX_TMP_PATH, WEB_LIBRARY_INDEX_PATH);
  if (!promoted) promoted = webLibraryIndexCopyFile(WEB_LIBRARY_INDEX_TMP_PATH, WEB_LIBRARY_INDEX_PATH);

  bool finalBracketOk = false;
  uint32_t finalVerifyCount = 0;
  uint32_t finalFileBytes = 0;
  bool finalValid = promoted && webLibraryIndexVerifyFileStreamAt(WEB_LIBRARY_INDEX_PATH, expectedCount, g_webLibraryIndexLastSavedBytes, finalBracketOk, finalVerifyCount, finalFileBytes);

  if (!finalValid) {
    if (SPIFFS.exists(WEB_LIBRARY_INDEX_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_PATH);
    if (SPIFFS.exists(WEB_LIBRARY_INDEX_PREV_PATH)) SPIFFS.rename(WEB_LIBRARY_INDEX_PREV_PATH, WEB_LIBRARY_INDEX_PATH);
    g_webLibraryIndexLastVerifyCount = finalVerifyCount;
    g_webLibraryIndexLastVerifyOk = false;
    g_webLibraryIndexLastSaveOk = false;
    g_webLibraryIndexLastSaveMs = millis() - t0;
    g_webLibraryIndexLastSaveError = String("Promoción fallida/rollback: bracket=") + String(finalBracketOk ? 1 : 0) +
      " count=" + String(finalVerifyCount) + " expected=" + String(expectedCount) +
      " bytes=" + String(finalFileBytes) + "/" + String(g_webLibraryIndexLastSavedBytes);
    return false;
  }

  if (SPIFFS.exists(WEB_LIBRARY_INDEX_TMP_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_TMP_PATH);
  if (SPIFFS.exists(WEB_LIBRARY_INDEX_PREV_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_PREV_PATH);

  g_webLibraryIndexJson = entries;
  g_webLibraryIndexCount = expectedCount;
  g_webLibraryIndexLastVerifyCount = finalVerifyCount;
  g_webLibraryIndexLastVerifyOk = true;
  g_webLibraryIndexLastSaveOk = true;
  g_webLibraryIndexLastSaveMs = millis() - t0;
  g_webLibraryIndexLastSaveError = "";

  // F42G/F42I: backup solo desde un índice final verificado y con archivos.
  if (expectedCount > 0) {
    uint32_t bakWritten = 0;
    webLibraryIndexWriteBracketedFile(WEB_LIBRARY_INDEX_BAK_PATH, entries, bakWritten);
  }

  logf("[LIB-IDX] save ok=%u verify=%u bytes=%lu count=%lu/%lu ms=%lu err=%s",
    (unsigned)g_webLibraryIndexLastSaveOk,
    (unsigned)g_webLibraryIndexLastVerifyOk,
    (unsigned long)g_webLibraryIndexLastSavedBytes,
    (unsigned long)g_webLibraryIndexLastVerifyCount,
    (unsigned long)expectedCount,
    (unsigned long)g_webLibraryIndexLastSaveMs,
    g_webLibraryIndexLastSaveError.c_str());

  return g_webLibraryIndexLastSaveOk;
}

static bool webLibraryIndexSaveToFs() {
  return webLibraryIndexSaveToFsFromEntries(g_webLibraryIndexJson, g_webLibraryIndexCount);
}

static bool webLibraryIndexRebuild(bool saveToFs = true, bool allowShrink = false) {
  if (!webAtrFsReady()) return false;
  uint32_t t0 = millis();
  String tmp;
  tmp.reserve(g_webAtrFilesJsonCache.length() ? g_webAtrFilesJsonCache.length() : 4096);
  // F42S: webAtrAppendFileListJson() usa webLibraryCollectEntries(), que ahora
  // comparte el skip seguro validado por /api/library/scan_console: no entra
  // a .cache, /CONFIG, /TMP, COVERS ni carpetas del sistema.
  webAtrAppendFileListJson(tmp);
  webLibraryNormalizeEntriesJson(tmp);

  bool entriesValid = webLibraryEntriesLooksValid(tmp);
  uint32_t newCount = webLibraryCountObjects(tmp);

  // F42H: capturar índice vigente/backup ANTES de decidir reemplazar.
  // Cualquier reconstrucción no manual que devuelva 0 o menos archivos que
  // el índice vigente se considera parcial/accidental y no reemplaza RAM ni SD.
  String oldEntriesF42H;
  uint32_t oldCountF42H = 0;
  if (g_webLibraryIndexLoaded && g_webLibraryIndexJson.length() && webLibraryEntriesLooksValid(g_webLibraryIndexJson)) {
    oldEntriesF42H = g_webLibraryIndexJson;
    oldCountF42H = webLibraryCountObjects(oldEntriesF42H);
  }
  if (oldCountF42H == 0) webLibraryLoadEntriesFromPath(WEB_LIBRARY_INDEX_PATH, oldEntriesF42H, &oldCountF42H);
  if (oldCountF42H == 0) webLibraryLoadEntriesFromPath(WEB_LIBRARY_INDEX_BAK_PATH, oldEntriesF42H, &oldCountF42H);

  // F42G: jamás reemplazar en RAM ni en SD un índice bueno por [] durante
  // navegación normal, salida de Cassette o refresh accidental. Si el escaneo
  // devuelve cero, se conserva RAM -> índice principal -> backup.
  if (newCount == 0) {
    if (oldCountF42H > 0) {
      g_webLibraryIndexJson = oldEntriesF42H;
      g_webLibraryIndexCount = oldCountF42H;
      g_webLibraryIndexBuildMs = millis() - t0;
      g_webLibraryIndexLoaded = true;
      g_webLibraryIndexDirty = false;
      g_webLibraryIndexLastSaveOk = true;
      g_webLibraryIndexLastVerifyOk = true;
      g_webLibraryIndexLastVerifyCount = g_webLibraryIndexCount;
      g_webLibraryIndexLastSavedBytes = g_webLibraryIndexJson.length() + 2;
      g_webLibraryIndexLastSaveError = "Escaneo devolvio 0; se conserva indice previo/backup";
      logf("[LIB-IDX] rebuild scan=0; conservando indice count=%u", (unsigned)g_webLibraryIndexCount);
      return true;
    }

    g_webLibraryIndexJson = "";
    g_webLibraryIndexCount = 0;
    g_webLibraryIndexBuildMs = millis() - t0;
    g_webLibraryIndexLoaded = false;
    g_webLibraryIndexDirty = false;
    g_webLibraryIndexLastSaveOk = false;
    g_webLibraryIndexLastVerifyOk = false;
    g_webLibraryIndexLastSaveError = "Escaneo devolvio 0 archivos; no se guarda indice vacio";
    logf("[LIB-IDX] rebuild scan=0 sin indice previo; no se guarda []");
    return false;
  }

  if (!allowShrink && oldCountF42H > 0 && newCount < oldCountF42H) {
    g_webLibraryIndexJson = oldEntriesF42H;
    g_webLibraryIndexCount = oldCountF42H;
    g_webLibraryIndexBuildMs = millis() - t0;
    g_webLibraryIndexLoaded = true;
    g_webLibraryIndexDirty = false;
    g_webLibraryIndexLastSaveOk = true;
    g_webLibraryIndexLastVerifyOk = true;
    g_webLibraryIndexLastVerifyCount = g_webLibraryIndexCount;
    g_webLibraryIndexLastSavedBytes = g_webLibraryIndexJson.length() + 2;
    g_webLibraryIndexLastSaveError = String("Rebuild automatico devolvio menos archivos (") + String(newCount) + "/" + String(oldCountF42H) + "); se conserva indice previo";
    logf("[LIB-IDX] rebuild shrink bloqueado new=%u old=%u allowShrink=0", (unsigned)newCount, (unsigned)oldCountF42H);
    return true;
  }

  g_webLibraryIndexJson = tmp;
  g_webLibraryIndexCount = newCount;

  // F37: no responder 500 si el escaneo produjo entradas. En F36 el validador
  // podia fallar por falsos positivos o por fragmentacion de String, aunque el
  // escaneo hubiera encontrado archivos. Si hay objetos "name", mantenemos el
  // indice en RAM y dejamos el estado real en indexSaveError/indexVerifyOk.
  if (!entriesValid) {
    logf("[LIB-IDX] rebuild validacion tolerante: valid=0 count=%lu len=%lu; se conserva si count>0",
      (unsigned long)g_webLibraryIndexCount,
      (unsigned long)g_webLibraryIndexJson.length());
    if (g_webLibraryIndexCount == 0) {
      g_webLibraryIndexJson = "";
      g_webLibraryIndexLoaded = false;
      g_webLibraryIndexDirty = true;
      g_webLibraryIndexLastSaveOk = false;
      g_webLibraryIndexLastVerifyOk = false;
      g_webLibraryIndexLastSaveError = "No se pudo generar JSON valido del indice";
      return false;
    }
    g_webLibraryIndexLastSaveError = "Indice generado en modo tolerante; revisar si la verificacion de archivo falla";
  }

  g_webLibraryIndexBuildMs = millis() - t0;
  g_webLibraryIndexLoaded = true;
  g_webLibraryIndexDirty = false;
  bool savedOk = true;
  if (saveToFs) {
    savedOk = webLibraryIndexSaveToFsFromEntries(tmp, newCount);
  } else {
    g_webLibraryIndexLastSaveOk = false;
    g_webLibraryIndexLastVerifyOk = false;
    g_webLibraryIndexLastSavedBytes = 0;
    g_webLibraryIndexLastVerifyCount = 0;
    g_webLibraryIndexLastSaveMs = 0;
    g_webLibraryIndexLastSaveError = "Modo seguro: índice reconstruido en RAM; SD no escrita sin commit=1";
  }
  // F49Z47: despues de reconstruir el indice, sincronizar tambien
  // la cache transitoria usada por /api/atr/status?files=1. Asi ningun
  // refresco diferido puede volver a pintar una lista vieja o parcial.
  g_webAtrFilesJsonCache = g_webLibraryIndexJson;
  g_webAtrFilesJsonCacheMs = millis();
  g_webAtrFilesJsonBuildMs = g_webLibraryIndexBuildMs;
  g_webAtrFilesJsonCount = g_webLibraryIndexCount;
  g_webAtrFilesJsonValid = true;
  logf("[LIB-IDX] rebuild count=%u buildMs=%lu save=%u savedOk=%u nonBlocking=1", (unsigned)g_webLibraryIndexCount, (unsigned long)g_webLibraryIndexBuildMs, (unsigned)saveToFs, (unsigned)savedOk);
  // F18: el escaneo/listado es valido aunque la verificacion de guardado JSON falle.
  // indexSaved/indexVerifyOk/indexSaveError informan el estado real del archivo.
  return true;
}

// [F24] Eliminado helper no usado: webLibraryIndexInvalidate
static bool webLibraryIndexRamUsable(bool repairCount = true) {
  if (!g_webLibraryIndexLoaded || g_webLibraryIndexCount == 0) return false;
  if (!g_webLibraryIndexJson.length()) {
    logf("[LIB-IDX] RAM invalida: count=%u pero json vacio; se fuerza lectura desde SD", (unsigned)g_webLibraryIndexCount);
    return false;
  }

  String e = g_webLibraryIndexJson;
  webLibraryNormalizeEntriesJson(e);
  if (!webLibraryEntriesLooksValid(e)) {
    logf("[LIB-IDX] RAM invalida: entries no validas len=%u count=%u; se fuerza lectura desde SD", (unsigned)e.length(), (unsigned)g_webLibraryIndexCount);
    return false;
  }

  uint32_t c = webLibraryCountObjects(e);
  if (c == 0) {
    logf("[LIB-IDX] RAM invalida: count=%u pero objects=0 len=%u; se fuerza lectura desde SD", (unsigned)g_webLibraryIndexCount, (unsigned)e.length());
    return false;
  }

  // F42T: si el contador y el JSON real no coinciden, manda el JSON real.
  // El contador solo es metadata; la fuente de verdad para listar es el array.
  if (repairCount && c != g_webLibraryIndexCount) {
    logf("[LIB-IDX] RAM count corregido: meta=%u real=%u", (unsigned)g_webLibraryIndexCount, (unsigned)c);
    g_webLibraryIndexCount = c;
  }

  g_webLibraryIndexJson = e;
  return true;
}

static bool webLibraryIndexEnsure(bool forceRefresh, bool allowShrink = false) {
  // F42G/F49Z46: solo refresh manual confirmado debe escanear la SD y actualizar
  // /CONFIG/library_index.json. Navegación normal, Cassette y /api/atr/status
  // solo leen RAM/FS/backup y nunca reconstruyen ni guardan [].
  if (forceRefresh) return webLibraryIndexRebuild(false, allowShrink);

  // F42T: no basta con g_webLibraryIndexCount > 0. Vimos casos reales donde
  // indexCount quedaba en 147/148 pero el String global de entries quedaba vacio
  // o inutilizable; eso producia total=0 y files=[]. Si RAM no es usable,
  // recargamos desde /CONFIG/library_index.json o backup.
  if (webLibraryIndexRamUsable(true)) { g_webLibraryIndexHits++; return true; }

  g_webLibraryIndexLoaded = false;
  g_webLibraryIndexJson = "";
  g_webLibraryIndexCount = 0;

  if (webLibraryIndexLoadFromFs() && webLibraryIndexRamUsable(true)) { g_webLibraryIndexHits++; return true; }
  return false;
}

static String webLibraryJsonStringField(const String& obj, const char* key) {
  String needle = String("\"") + key + "\":\"";
  int p = obj.indexOf(needle);
  if (p < 0) return String("");
  p += needle.length();
  String out;
  bool esc = false;
  for (int i = p; i < (int)obj.length(); i++) {
    char c = obj[i];
    if (esc) { out += c; esc = false; continue; }
    if (c == '\\') { esc = true; continue; }
    if (c == '"') break;
    out += c;
  }
  return out;
}

static String webLibraryTypeFromObject(const String& obj) {
  // F49Z50: para filtrar y contar se usa primero la extensión del nombre.
  // Esto también corrige índices persistentes antiguos donde .COM/.EXE quedaron
  // guardados como tipo XEX.
  String name = webLibraryJsonStringField(obj, "name");
  int dot = name.lastIndexOf('.');
  if (dot >= 0 && dot < (int)name.length() - 1) {
    String ext = name.substring(dot + 1);
    ext.toUpperCase();
    if (ext == "ATR" || ext == "XEX" || ext == "COM" || ext == "EXE" || ext == "BAS" || ext == "CAS" || ext == "SEC") return ext;
  }
  String type = webLibraryJsonStringField(obj, "type");
  type.trim();
  type.replace(".", "");
  type.toUpperCase();
  if (type == "ATR" || type == "XEX" || type == "COM" || type == "EXE" || type == "BAS" || type == "CAS" || type == "SEC") return type;
  if (type.length()) return String("OTHER");
  return String("OTHER");
}

static bool webLibraryObjectMatchesSearchOnly(const String& obj, const String& qLower) {
  if (!qLower.length()) return true;
  String name = webLibraryJsonStringField(obj, "name");
  String path = webLibraryJsonStringField(obj, "path");
  String type = webLibraryTypeFromObject(obj);
  String nlow = name; nlow.toLowerCase();
  String plow = path; plow.toLowerCase();
  String tlow = type; tlow.toLowerCase();
  return nlow.indexOf(qLower) >= 0 || plow.indexOf(qLower) >= 0 || tlow.indexOf(qLower) >= 0;
}

static bool webLibraryObjectMatches(const String& obj, const String& qLower, const String& typeUpper) {
  if (!webLibraryObjectMatchesSearchOnly(obj, qLower)) return false;
  if (typeUpper.length() && typeUpper != "ALL") {
    String type = webLibraryTypeFromObject(obj);
    if (type != typeUpper) return false;
  }
  return true;
}


// F43J: guardar índice en NDJSON, una entrada por línea.
// No se usa un array JSON gigante ni se carga el archivo completo a RAM.
static bool webLibraryNdjsonVerifyFileAt(const char* path, uint32_t expectedCount, uint32_t expectedBytes,
                                         uint32_t& verifyCount, uint32_t& fileBytes) {
  verifyCount = 0;
  fileBytes = 0;
  if (!webAtrFsReady() || !SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  fileBytes = (uint32_t)f.size();
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    if (line.startsWith("{") && line.endsWith("}") && line.indexOf("\"name\"") >= 0 && line.indexOf("\"path\"") >= 0) verifyCount++;
    yield();
  }
  f.close();
  bool countOk = (expectedCount == 0) || (verifyCount == expectedCount);
  bool bytesOk = (expectedBytes == 0) || (fileBytes == expectedBytes);
  return countOk && bytesOk && verifyCount > 0;
}

static bool webLibraryNdjsonCopyFile(const char* from, const char* to) {
  if (!webAtrFsReady() || !SPIFFS.exists(from)) return false;
  File src = SPIFFS.open(from, "r");
  if (!src) return false;
  if (SPIFFS.exists(to)) SPIFFS.remove(to);
  File dst = SPIFFS.open(to, "w");
  if (!dst) { src.close(); return false; }
  uint8_t buf[512];
  bool ok = true;
  while (src.available()) {
    int n = src.read(buf, sizeof(buf));
    if (n <= 0) break;
    if (dst.write(buf, n) != (size_t)n) { ok = false; break; }
    yield();
  }
  src.close();
  dst.flush();
  dst.close();
  if (!ok && SPIFFS.exists(to)) SPIFFS.remove(to);
  return ok;
}

static bool webLibraryNdjsonSaveFromEntries(const std::vector<WebLibraryScanEntry>& entries) {
  uint32_t t0 = millis();
  g_webLibraryIndexLastSaveOk = false;
  g_webLibraryIndexLastVerifyOk = false;
  g_webLibraryIndexLastSavedBytes = 0;
  g_webLibraryIndexLastVerifyCount = 0;
  g_webLibraryIndexLastSaveError = "";

  if (!webLibraryEnsureConfigDir()) {
    g_webLibraryIndexLastSaveError = "No se pudo crear/acceder /CONFIG para NDJSON";
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }
  if (entries.empty()) {
    g_webLibraryIndexLastSaveError = "Protegido: no se escribe índice NDJSON vacío";
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }

  if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH);
  File f = SPIFFS.open(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH, "w");
  if (!f) {
    g_webLibraryIndexLastSaveError = "No se pudo abrir library_index.tmp.ndjson";
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }

  uint32_t written = 0;
  uint32_t count = 0;
  bool ok = true;
  for (const WebLibraryScanEntry& entry : entries) {
    String obj;
    obj.reserve(768);
    webLibraryAppendScanEntryJson(obj, entry);
    size_t w1 = f.print(obj);
    size_t w2 = f.print('\n');
    written += (uint32_t)(w1 + w2);
    if (w1 != obj.length() || w2 != 1) { ok = false; break; }
    count++;
    if ((count & 0x0F) == 0) yield();
  }
  f.flush();
  f.close();

  g_webLibraryIndexLastSavedBytes = written;
  if (!ok || count != entries.size()) {
    if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH);
    g_webLibraryIndexLastSaveError = String("Escritura NDJSON incompleta count=") + String(count) + " expected=" + String((unsigned)entries.size());
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }

  uint32_t verifyCount = 0;
  uint32_t fileBytes = 0;
  bool tmpOk = webLibraryNdjsonVerifyFileAt(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH, count, written, verifyCount, fileBytes);
  if (!tmpOk) {
    if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH);
    g_webLibraryIndexLastVerifyCount = verifyCount;
    g_webLibraryIndexLastSaveError = String("TMP NDJSON inválido count=") + String(verifyCount) + " expected=" + String(count) +
      " bytes=" + String(fileBytes) + "/" + String(written);
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }

  if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_PREV_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_NDJSON_PREV_PATH);
  if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_PATH)) {
    if (!SPIFFS.rename(WEB_LIBRARY_INDEX_NDJSON_PATH, WEB_LIBRARY_INDEX_NDJSON_PREV_PATH)) {
      webLibraryNdjsonCopyFile(WEB_LIBRARY_INDEX_NDJSON_PATH, WEB_LIBRARY_INDEX_NDJSON_PREV_PATH);
      SPIFFS.remove(WEB_LIBRARY_INDEX_NDJSON_PATH);
    }
  }

  bool promoted = SPIFFS.rename(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH, WEB_LIBRARY_INDEX_NDJSON_PATH);
  if (!promoted) promoted = webLibraryNdjsonCopyFile(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH, WEB_LIBRARY_INDEX_NDJSON_PATH);

  uint32_t finalCount = 0;
  uint32_t finalBytes = 0;
  bool finalOk = promoted && webLibraryNdjsonVerifyFileAt(WEB_LIBRARY_INDEX_NDJSON_PATH, count, written, finalCount, finalBytes);
  if (!finalOk) {
    if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_NDJSON_PATH);
    if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_PREV_PATH)) SPIFFS.rename(WEB_LIBRARY_INDEX_NDJSON_PREV_PATH, WEB_LIBRARY_INDEX_NDJSON_PATH);
    g_webLibraryIndexLastVerifyCount = finalCount;
    g_webLibraryIndexLastSaveError = String("Promoción NDJSON fallida count=") + String(finalCount) + " expected=" + String(count) +
      " bytes=" + String(finalBytes) + "/" + String(written);
    g_webLibraryIndexLastSaveMs = millis() - t0;
    return false;
  }

  if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_NDJSON_TMP_PATH);
  if (SPIFFS.exists(WEB_LIBRARY_INDEX_NDJSON_PREV_PATH)) SPIFFS.remove(WEB_LIBRARY_INDEX_NDJSON_PREV_PATH);
  webLibraryNdjsonCopyFile(WEB_LIBRARY_INDEX_NDJSON_PATH, WEB_LIBRARY_INDEX_NDJSON_BAK_PATH);

  g_webLibraryIndexLastVerifyCount = finalCount;
  g_webLibraryIndexLastVerifyOk = true;
  g_webLibraryIndexLastSaveOk = true;
  g_webLibraryIndexLastSaveMs = millis() - t0;
  g_webLibraryIndexLastSaveError = "";
  g_webLibraryIndexCount = count;
  logf("[LIB-NDJSON] save ok=1 bytes=%lu count=%lu ms=%lu path=%s",
       (unsigned long)finalBytes, (unsigned long)finalCount, (unsigned long)g_webLibraryIndexLastSaveMs,
       WEB_LIBRARY_INDEX_NDJSON_PATH);
  return true;
}

static bool webLibraryNdjsonPageFromPath(const char* path, int page, int pageSize, const String& qLower, const String& typeUpper,
                                         String& pageEntries, uint32_t& totalMatches,
                                         uint32_t& all, uint32_t& atr, uint32_t& xex, uint32_t& com, uint32_t& exe, uint32_t& bas, uint32_t& cas, uint32_t& sec, uint32_t& other,
                                         uint32_t& fileBytesOut) {
  pageEntries = "";
  totalMatches = 0;
  all = atr = xex = com = exe = bas = cas = sec = other = 0;
  fileBytesOut = 0;
  if (!webAtrFsReady() || !SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  fileBytesOut = (uint32_t)f.size();
  if (page < 0) page = 0;
  if (pageSize <= 0) pageSize = 50;
  if (pageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) pageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX;
  uint32_t startWanted = (uint32_t)page * (uint32_t)pageSize;
  uint32_t added = 0;
  pageEntries.reserve(pageSize <= 50 ? 16384 : (pageSize <= 100 ? 32768 : 49152));

  while (f.available()) {
    String obj = f.readStringUntil('\n');
    obj.trim();
    if (!obj.length()) continue;
    if (!obj.startsWith("{") || !obj.endsWith("}")) continue;

    if (webLibraryObjectMatchesSearchOnly(obj, qLower)) {
      all++;
      String tt = webLibraryTypeFromObject(obj);
      if (tt == "ATR") atr++;
      else if (tt == "XEX") xex++;
      else if (tt == "COM") com++;
      else if (tt == "EXE") exe++;
      else if (tt == "BAS") bas++;
      else if (tt == "CAS") cas++;
      else if (tt == "SEC") sec++;
      else other++;
    }

    if (webLibraryObjectMatches(obj, qLower, typeUpper)) {
      if (totalMatches >= startWanted && added < (uint32_t)pageSize) {
        if (pageEntries.length()) pageEntries += ",";
        pageEntries += obj;
        added++;
      }
      totalMatches++;
    }
    if ((all & 0x0F) == 0) yield();
  }
  f.close();
  bool ok = (all > 0) || qLower.length() || (typeUpper.length() && typeUpper != "ALL");
  logf("[LIB-NDJSON] read page path=%s ok=%u bytes=%lu all=%lu total=%lu page=%d pageSize=%d pageBytes=%u",
       path, (unsigned)(ok ? 1 : 0), (unsigned long)fileBytesOut, (unsigned long)all, (unsigned long)totalMatches,
       page, pageSize, (unsigned)pageEntries.length());
  return ok;
}

static bool webLibraryNdjsonBestPage(int page, int pageSize, const String& qLower, const String& typeUpper,
                                     String& pageEntries, uint32_t& totalMatches,
                                     uint32_t& all, uint32_t& atr, uint32_t& xex, uint32_t& com, uint32_t& exe, uint32_t& bas, uint32_t& cas, uint32_t& sec, uint32_t& other,
                                     String& sourceOut, uint32_t& fileBytesOut) {
  const char* paths[] = { WEB_LIBRARY_INDEX_NDJSON_PATH, WEB_LIBRARY_INDEX_NDJSON_BAK_PATH, WEB_LIBRARY_INDEX_NDJSON_PREV_PATH };
  const char* names[] = { "ndjson", "ndjson_bak", "ndjson_prev" };
  for (int i = 0; i < 3; i++) {
    if (webLibraryNdjsonPageFromPath(paths[i], page, pageSize, qLower, typeUpper, pageEntries, totalMatches,
                                     all, atr, xex, com, exe, bas, cas, sec, other, fileBytesOut)) {
      sourceOut = names[i];
      return true;
    }
  }
  sourceOut = "";
  fileBytesOut = 0;
  return false;
}

static bool webLibraryNdjsonReadAllEntries(String& entries, uint32_t& count, String& sourceOut, uint32_t& fileBytesOut) {
  entries = "";
  count = 0;
  sourceOut = "";
  fileBytesOut = 0;
  if (!webAtrFsReady()) return false;
  const char* paths[] = { WEB_LIBRARY_INDEX_NDJSON_PATH, WEB_LIBRARY_INDEX_NDJSON_BAK_PATH, WEB_LIBRARY_INDEX_NDJSON_PREV_PATH };
  const char* names[] = { "ndjson", "ndjson_bak", "ndjson_prev" };
  for (int i = 0; i < 3; i++) {
    const char* path = paths[i];
    if (!SPIFFS.exists(path)) continue;
    File f = SPIFFS.open(path, "r");
    if (!f) continue;
    entries = "";
    count = 0;
    fileBytesOut = (uint32_t)f.size();
    entries.reserve(fileBytesOut > 0 ? (fileBytesOut + 8) : 4096);
    while (f.available()) {
      String obj = f.readStringUntil('\n');
      obj.trim();
      if (!obj.length()) continue;
      if (!obj.startsWith("{") || !obj.endsWith("}")) continue;
      if (entries.length()) entries += ",";
      entries += obj;
      count++;
      if ((count & 0x0F) == 0) yield();
    }
    f.close();
    if (count > 0) {
      sourceOut = names[i];
      logf("[LIB-NDJSON] read all path=%s ok=1 bytes=%lu count=%lu pageBytes=%u",
           path, (unsigned long)fileBytesOut, (unsigned long)count, (unsigned)entries.length());
      return true;
    }
  }
  entries = "";
  count = 0;
  fileBytesOut = 0;
  return false;
}


// F42F: contar matches sin construir un String gigante con todos los archivos.
static uint32_t webLibraryCountMatchingEntries(const String& entries, const String& qLower, const String& typeUpper) {
  uint32_t total = 0;
  int p = 0;
  while (p < (int)entries.length()) {
    int a = entries.indexOf('{', p);
    if (a < 0) break;
    int b = entries.indexOf('}', a);
    if (b < 0) break;
    String obj = entries.substring(a, b + 1);
    if (webLibraryObjectMatches(obj, qLower, typeUpper)) total++;
    p = b + 1;
  }
  return total;
}

// F42F: enviar files[] directo al cliente con separador controlado.
// Esto evita tres problemas vistos en F42D/F42E:
// 1) doble coma: },,{
// 2) String pageEntries gigante y fragmentado
// 3) respuesta chunked cortada antes de cerrar ]}
static void webLibraryStreamPagedEntries(const String& entries, int page, int pageSize, const String& qLower, const String& typeUpper) {
  if (page < 0) page = 0;
  if (pageSize <= 0) pageSize = 10000;
  if (pageSize > 10000) pageSize = 10000;

  int startWanted = page * pageSize;
  int added = 0;
  int seen = 0;
  bool first = true;
  int p = 0;

  String buf;
  buf.reserve(2048);

  while (p < (int)entries.length()) {
    int a = entries.indexOf('{', p);
    if (a < 0) break;
    int b = entries.indexOf('}', a);
    if (b < 0) break;

    String obj = entries.substring(a, b + 1);
    if (webLibraryObjectMatches(obj, qLower, typeUpper)) {
      if (seen >= startWanted && added < pageSize) {
        if (!first) buf += ",";
        buf += obj;
        first = false;
        added++;

        if (buf.length() >= 2048) {
          server.sendContent(buf);
          buf = "";
          delay(0);
        }
      }
      seen++;
      if (added >= pageSize) {
        // Se puede cortar aquí porque totalMatches ya fue calculado antes.
        break;
      }
    }
    p = b + 1;
  }

  if (buf.length()) {
    server.sendContent(buf);
    delay(0);
  }
}

// F49Z48: conteos por tipo calculados desde el indice completo, no desde
// la pagina visible. Esto evita que los botones ATR/XEX/CAS muestren solo
// los 20 elementos de la pagina actual.
static void webLibraryAppendTypeCountsJson(String& json, const String& entries, const String& qLower) {
  uint32_t all = 0, atr = 0, xex = 0, com = 0, exe = 0, bas = 0, cas = 0, sec = 0, other = 0;
  int p = 0;
  while (p < (int)entries.length()) {
    int a = entries.indexOf('{', p);
    if (a < 0) break;
    int b = entries.indexOf('}', a);
    if (b < 0) break;
    String obj = entries.substring(a, b + 1);
    if (webLibraryObjectMatchesSearchOnly(obj, qLower)) {
      all++;
      String type = webLibraryTypeFromObject(obj);
      if (type == "ATR") atr++;
      else if (type == "XEX") xex++;
      else if (type == "COM") com++;
      else if (type == "EXE") exe++;
      else if (type == "BAS") bas++;
      else if (type == "CAS") cas++;
      else if (type == "SEC") sec++;
      else other++;
    }
    p = b + 1;
  }
  json += ",\"typeCounts\":{";
  json += "\"ALL\":" + String(all);
  json += ",\"ATR\":" + String(atr);
  json += ",\"XEX\":" + String(xex);
  json += ",\"COM\":" + String(com);
  json += ",\"EXE\":" + String(exe);
  json += ",\"BAS\":" + String(bas);
  json += ",\"CAS\":" + String(cas);
  json += ",\"SEC\":" + String(sec);
  json += ",\"OTHER\":" + String(other);
  json += "}";
}

// F42X: paginacion directa desde /CONFIG/library_index.json.
// Motivo: el archivo en SD puede estar correcto (36KB/148 objetos), pero cargarlo
// completo a String para luego servir /api/library puede quedar en RAM vacia en
// algunos ciclos. Esta ruta lee el JSON como stream, extrae objeto por objeto y
// arma solo la pagina visible de files[]. No escribe SD ni reconstruye indice.
static bool webLibraryStreamIndexFilePageAt(const char* path, int page, int pageSize, const String& qLower, const String& typeUpper,
                                            String& pageEntries, uint32_t& totalMatches,
                                            uint32_t& all, uint32_t& atr, uint32_t& xex, uint32_t& com, uint32_t& exe, uint32_t& bas, uint32_t& cas, uint32_t& sec, uint32_t& other,
                                            uint32_t& fileBytesOut) {
  pageEntries = "";
  totalMatches = 0;
  all = atr = xex = com = exe = bas = cas = sec = other = 0;
  fileBytesOut = 0;

  if (!webAtrFsReady() || !SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  fileBytesOut = (uint32_t)f.size();

  if (page < 0) page = 0;
  if (pageSize <= 0) pageSize = 50;
  if (pageSize > 250) pageSize = 250; // F42X: respuesta visible acotada para heap estable
  uint32_t startWanted = (uint32_t)page * (uint32_t)pageSize;
  uint32_t added = 0;

  bool inString = false;
  bool esc = false;
  int level = 0;
  bool capturing = false;
  String obj;
  obj.reserve(1536);

  uint32_t bytesRead = 0;
  while (true) {
    int ri = f.read();
    if (ri < 0) break;
    char c = (char)ri;
    bytesRead++;

    if (!capturing) {
      if (c == '{') {
        capturing = true;
        level = 1;
        inString = false;
        esc = false;
        obj = "{";
      }
      continue;
    }

    obj += c;

    if (inString) {
      if (esc) { esc = false; }
      else if (c == '\\') { esc = true; }
      else if (c == '"') { inString = false; }
    } else {
      if (c == '"') inString = true;
      else if (c == '{') level++;
      else if (c == '}') level--;
    }

    if (capturing && level == 0) {
      if (webLibraryObjectMatchesSearchOnly(obj, qLower)) {
        all++;
        String tt = webLibraryTypeFromObject(obj);
        if (tt == "ATR") atr++;
        else if (tt == "XEX") xex++;
        else if (tt == "COM") com++;
        else if (tt == "EXE") exe++;
        else if (tt == "BAS") bas++;
        else if (tt == "CAS") cas++;
        else if (tt == "SEC") sec++;
        else other++;
      }

      if (webLibraryObjectMatches(obj, qLower, typeUpper)) {
        if (totalMatches >= startWanted && added < (uint32_t)pageSize) {
          if (pageEntries.length()) pageEntries += ",";
          pageEntries += obj;
          added++;
        }
        totalMatches++;
      }

      obj = "";
      capturing = false;
      inString = false;
      esc = false;
      level = 0;
      if ((bytesRead & 0x0FFF) == 0) yield();
    }

    if (obj.length() > 8192) {
      // Defensa contra JSON dañado: no dejamos crecer un objeto indefinidamente.
      logf("[LIB-IDX] direct page abort: objeto demasiado grande path=%s len=%u", path, (unsigned)obj.length());
      f.close();
      return false;
    }
  }
  f.close();

  bool ok = (all > 0 || totalMatches > 0 || qLower.length() || (typeUpper.length() && typeUpper != "ALL"));
  logf("[LIB-IDX] direct page path=%s ok=%u bytes=%lu total=%lu all=%lu pageBytes=%u",
       path, (unsigned)ok, (unsigned long)fileBytesOut, (unsigned long)totalMatches, (unsigned long)all, (unsigned)pageEntries.length());
  return ok;
}

static bool webLibraryStreamIndexBestFilePage(int page, int pageSize, const String& qLower, const String& typeUpper,
                                              String& pageEntries, uint32_t& totalMatches,
                                              uint32_t& all, uint32_t& atr, uint32_t& xex, uint32_t& com, uint32_t& exe, uint32_t& bas, uint32_t& cas, uint32_t& sec, uint32_t& other,
                                              String& sourceOut, uint32_t& fileBytesOut) {
  const char* paths[] = { WEB_LIBRARY_INDEX_PATH, WEB_LIBRARY_INDEX_BAK_PATH, WEB_LIBRARY_INDEX_PREV_PATH };
  const char* names[] = { "main", "bak", "prev" };

  for (int i = 0; i < 3; i++) {
    String pe;
    uint32_t tm = 0, a = 0, at = 0, xx = 0, co = 0, ex = 0, ba = 0, ca = 0, se = 0, ot = 0, fb = 0;
    if (webLibraryStreamIndexFilePageAt(paths[i], page, pageSize, qLower, typeUpper, pe, tm, a, at, xx, co, ex, ba, ca, se, ot, fb) && a > 0) {
      pageEntries = pe;
      totalMatches = tm;
      all = a; atr = at; xex = xx; com = co; exe = ex; bas = ba; cas = ca; sec = se; other = ot;
      sourceOut = names[i];
      fileBytesOut = fb;
      return true;
    }
  }
  return false;
}


static void webLibraryAppendPagedEntries(String& out, const String& entries, int page, int pageSize, const String& qLower, const String& typeUpper, uint32_t& totalMatches) {
  if (page < 0) page = 0;
  if (pageSize <= 0) pageSize = 10000;
  if (pageSize > 10000) pageSize = 10000;
  int startWanted = page * pageSize;
  int added = 0;
  int seen = 0;
  bool first = true;
  int p = 0;
  while (p < (int)entries.length()) {
    int a = entries.indexOf('{', p);
    if (a < 0) break;
    int b = entries.indexOf('}', a);
    if (b < 0) break;
    String obj = entries.substring(a, b + 1);
    if (webLibraryObjectMatches(obj, qLower, typeUpper)) {
      totalMatches++;
      if (seen >= startWanted && added < pageSize) {
        if (!first) out += ",";
        out += obj;
        first = false;
        added++;
      }
      seen++;
    }
    p = b + 1;
  }
}

static void webLibraryPatchCasFlagsInIndex(const String& mountedName, bool mounted, bool playing) {
  if (!g_webLibraryIndexLoaded || g_webLibraryIndexJson.length() == 0) return;
  g_webLibraryIndexJson.replace(",\"casMounted\":1", ",\"casMounted\":0");
  g_webLibraryIndexJson.replace(",\"casPlaying\":1", ",\"casPlaying\":0");
  if (!mounted || mountedName.length() == 0) return;
  String needle = "\"name\":\"" + jsonEscape(mountedName) + "\"";
  int p = g_webLibraryIndexJson.indexOf(needle);
  if (p < 0) return;
  int e = g_webLibraryIndexJson.indexOf("}", p);
  if (e < 0) return;
  int m = g_webLibraryIndexJson.indexOf(",\"casMounted\":0", p);
  if (m >= 0 && m < e) g_webLibraryIndexJson = g_webLibraryIndexJson.substring(0, m) + ",\"casMounted\":1" + g_webLibraryIndexJson.substring(m + 15);
  if (playing) {
    int q = g_webLibraryIndexJson.indexOf(",\"casPlaying\":0", p);
    if (q >= 0 && q < e) g_webLibraryIndexJson = g_webLibraryIndexJson.substring(0, q) + ",\"casPlaying\":1" + g_webLibraryIndexJson.substring(q + 15);
  }
}

static void webAtrInvalidateFilesCache() {
  g_webAtrFilesJsonValid = false;
  g_webAtrFilesJsonCache = "";
  g_webAtrFilesJsonCacheMs = 0;
  g_webAtrFilesJsonBuildMs = 0;
  g_webAtrFilesJsonCount = 0;
  // F49Z46: no invalidar ni reconstruir el indice persistente automaticamente.
  // El usuario decide cuando actualizar archivos + JSON usando "Refrescar biblioteca".
}

static void webAtrPatchCasFlagsInFilesCache(const String& mountedName, bool mounted, bool playing) {
  webLibraryPatchCasFlagsInIndex(mountedName, mounted, playing);
  if (!g_webAtrFilesJsonValid || g_webAtrFilesJsonCache.length() == 0) return;

  g_webAtrFilesJsonCache.replace(",\"casMounted\":1", ",\"casMounted\":0");
  g_webAtrFilesJsonCache.replace(",\"casPlaying\":1", ",\"casPlaying\":0");

  if (!mounted || mountedName.length() == 0) return;

  String needle = "\"name\":\"" + jsonEscape(mountedName) + "\"";
  int p = g_webAtrFilesJsonCache.indexOf(needle);
  if (p < 0) return;
  int e = g_webAtrFilesJsonCache.indexOf("}", p);
  if (e < 0) return;

  int m = g_webAtrFilesJsonCache.indexOf(",\"casMounted\":0", p);
  if (m >= 0 && m < e) g_webAtrFilesJsonCache = g_webAtrFilesJsonCache.substring(0, m) + ",\"casMounted\":1" + g_webAtrFilesJsonCache.substring(m + 15);

  if (playing) {
    int q = g_webAtrFilesJsonCache.indexOf(",\"casPlaying\":0", p);
    if (q >= 0 && q < e) g_webAtrFilesJsonCache = g_webAtrFilesJsonCache.substring(0, q) + ",\"casPlaying\":1" + g_webAtrFilesJsonCache.substring(q + 15);
  }
}

static void webAtrAppendFileListJsonCached(String &j, bool forceRefresh) {
  uint32_t now = millis();
  if (!forceRefresh && g_webAtrFilesJsonValid && (now - g_webAtrFilesJsonCacheMs) < WEB_ATR_FILES_CACHE_TTL_MS) {
    g_webAtrFilesJsonHits++;
    j += g_webAtrFilesJsonCache;
    return;
  }

  uint32_t t0 = millis();
  // F43C: esta función es usada por status/compatibilidad. No puede disparar
  // webLibraryIndexEnsure(), porque eso re-escanea desde Cassette/Web-ATR.
  // Solo sirve cache ya existente; si no existe, devuelve lista vacía.
  if (g_webLibraryIndexLoaded && g_webLibraryIndexJson.length() > 2) {
    g_webAtrFilesJsonCache = g_webLibraryIndexJson;
    g_webAtrFilesJsonBuildMs = g_webLibraryIndexBuildMs ? g_webLibraryIndexBuildMs : (millis() - t0);
    g_webAtrFilesJsonCount = g_webLibraryIndexCount;
  } else {
    g_webAtrFilesJsonBuildMs = millis() - t0;
    g_webAtrFilesJsonCache = "";
    g_webAtrFilesJsonCount = 0;
    logf("[LIB-IDX] F43C files cache pedido por status sin cache; no se escanea SD");
  }
  g_webAtrFilesJsonCacheMs = now;
  g_webAtrFilesJsonValid = true;
  j += g_webAtrFilesJsonCache;
}

// [F24] Eliminado helper no usado: handleAtrPage
void handleAtrStatus() {
  bool fastStatus = server.hasArg("fast") && server.arg("fast") == "1";
  if (!fastStatus) webAtrRefreshPresence();
  // F43C: /api/atr/status, usado al montar/desmontar y al salir de Cassette,
  // NO debe incluir ni reconstruir listados por defecto. La Biblioteca se lista
  // desde /api/library y el escaneo lo hace únicamente el botón manual.
  bool includeFiles = false;
  if (server.hasArg("files")) {
    String fv = server.arg("files");
    fv.toLowerCase();
    includeFiles = (fv == "1" || fv == "true" || fv == "yes");
  }
  bool forceFilesRefresh = false;
  bool manualFilesRefresh = false;
  String j = "{";
  j += "\"compiled\":true";
  j += ",\"mode\":\"flash-multi\"";
  j += ",\"fastStatus\":" + String(fastStatus ? 1 : 0);
  j += ",\"enabled\":" + String(WEB_ATR_ENABLED ? 1 : 0);
  j += ",\"mask\":" + String(WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"forceMask\":" + String(WEB_ATR_FORCE_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"driveVisibleMask\":" + String(DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"maxUnits\":" + String(DRIVE_UI_MAX_UNITS);
  j += ",\"filePresent\":" + String(g_webAtrFilePresent ? 1 : 0);
  j += ",\"casMounted\":" + String(g_casMounted ? 1 : 0);
  j += ",\"casPlaying\":" + String(g_casPlaying ? 1 : 0);
  j += ",\"casEof\":" + String(g_casEof ? 1 : 0);
  j += ",\"casName\":\"" + jsonEscape(g_casMountedName) + "\"";
  j += ",\"casPath\":\"" + jsonEscape(g_casMountedPath) + "\"";
  j += ",\"casPlayPos\":" + String((unsigned long)g_casPlayPos);
  j += ",\"casPlaySize\":" + String((unsigned long)g_casPlaySize);
  j += ",\"casBytesQueued\":" + String((unsigned long)g_casBytesQueued);
  j += ",\"casBaud\":" + String((unsigned long)g_casCurrentBaud);
  j += ",\"casRpBufferUsed\":" + String((unsigned long)g_casRpBufferUsed);
  j += ",\"casRpBufferFree\":" + String((unsigned long)g_casRpBufferFree);
  j += ",\"casRpBytesPlayed\":" + String((unsigned long)g_casRpBytesPlayed);
  j += ",\"casLastError\":\"" + jsonEscape(String(g_casLastError)) + "\"";
  j += ",\"fsTotal\":" + String(webAtrFsReady() ? SPIFFS.totalBytes() : 0);
  j += ",\"fsUsed\":" + String(webAtrFsReady() ? SPIFFS.usedBytes() : 0);
  j += ",\"fsFree\":" + String(webAtrFsReady() ? (SPIFFS.totalBytes() - SPIFFS.usedBytes()) : 0);
  j += ",\"reads\":" + String(g_webAtrReads);
  j += ",\"status\":" + String(g_webAtrStatus);
  j += ",\"percom\":" + String(g_webAtrPercom);
  j += ",\"nak\":" + String(g_webAtrNak);
  j += ",\"fastLoad\":" + String(g_webAtrFastLoadEnabled ? 1 : 0);
  j += ",\"readAhead\":" + String(g_webAtrReadAheadEnabled ? 1 : 0);
  j += ",\"cacheHit\":" + String(g_webAtrCacheHit);
  j += ",\"cacheMiss\":" + String(g_webAtrCacheMiss);
  j += ",\"cacheStore\":" + String(g_webAtrCacheStore);
  j += ",\"readAheadCount\":" + String(g_webAtrReadAhead);
  j += ",\"readAheadHit\":" + String(g_webAtrReadAheadHit);
  j += ",\"readAheadFail\":" + String(g_webAtrReadAheadFail);
  j += ",\"sdReadUsLast\":" + String(g_webAtrSdReadUsLast);
  j += ",\"sdReadUsMax\":" + String(g_webAtrSdReadUsMax);
  j += ",\"priorityMode\":" + String(g_webAtrPriorityMode ? 1 : 0);
  j += ",\"atariBusy\":" + String(webAtrAtariBusy() ? 1 : 0);
  j += ",\"readAheadCountCfg\":" + String(g_webAtrReadAheadCount);
  j += ",\"openReuse\":" + String(g_webAtrOpenReuse);
  j += ",\"openCount\":" + String(g_webAtrOpenCount);
  j += ",\"openFail\":" + String(g_webAtrOpenFail);
  j += ",\"closeCount\":" + String(g_webAtrCloseCount);
  j += ",\"openUsLast\":" + String(g_webAtrOpenUsLast);
  j += ",\"seekUsLast\":" + String(g_webAtrSeekUsLast);
  j += ",\"dataReadUsLast\":" + String(g_webAtrDataReadUsLast);
  j += ",\"totalReadUsLast\":" + String(g_webAtrTotalReadUsLast);
  j += ",\"totalReadUsMax\":" + String(g_webAtrTotalReadUsMax);
  j += ",\"slots\":[";
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    if (i) j += ",";
    WebAtrMeta m;
    bool ok = false;
    if (fastStatus) {
      ok = g_webAtrSlotPresent[i];
      memset(&m, 0, sizeof(m));
    } else {
      ok = webAtrReadMetaForIndex(i, m, false);
    }
    j += "{\"unit\":" + String(i + 1);
    j += ",\"visible\":" + String(driveUiVisibleIndex(i) ? 1 : 0);
    j += ",\"name\":\"" + jsonEscape(g_webAtrMountedName[i]) + "\"";
    j += ",\"enabled\":" + String((WEB_ATR_ENABLED && (WEB_ATR_DEV_MASK & (1u << i))) ? 1 : 0);
    j += ",\"forced\":" + String((WEB_ATR_ENABLED && (WEB_ATR_FORCE_MASK & (1u << i))) ? 1 : 0);
    j += ",\"present\":" + String(ok ? 1 : 0);
    j += ",\"fileSize\":" + String(ok ? m.fileSize : 0);
    j += ",\"sectorSize\":" + String(ok ? m.sectorSize : 0);
    j += ",\"totalSectors\":" + String(ok ? m.totalSectors : 0);
    j += ",\"reads\":" + String(g_webAtrReadsByUnit[i]);
    j += "}";
  }
  j += "]";
  j += ",\"filesCache\":{";
  j += "\"enabled\":" + String(g_webAtrFilesJsonValid ? 1 : 0);
  j += ",\"ageMs\":" + String(g_webAtrFilesJsonValid ? (millis() - g_webAtrFilesJsonCacheMs) : 0);
  j += ",\"buildMs\":" + String(g_webAtrFilesJsonBuildMs);
  j += ",\"hits\":" + String(g_webAtrFilesJsonHits);
  j += ",\"count\":" + String(g_webAtrFilesJsonCount);
  j += ",\"indexLoaded\":" + String(g_webLibraryIndexLoaded ? 1 : 0);
  j += ",\"indexDirty\":" + String(g_webLibraryIndexDirty ? 1 : 0);
  j += ",\"indexBuildMs\":" + String(g_webLibraryIndexBuildMs);
  j += ",\"indexLoadMs\":" + String(g_webLibraryIndexLoadMs);
  j += ",\"indexHits\":" + String(g_webLibraryIndexHits);
  j += "}";
  j += ",\"filesCount\":" + String(g_webAtrFilesJsonCount ? g_webAtrFilesJsonCount : g_webLibraryIndexCount);
  if (includeFiles) {
    bool paged = server.hasArg("page") || server.hasArg("pageSize") || server.hasArg("limit") || server.hasArg("offset") || server.hasArg("q") || server.hasArg("type");
    if (paged && webLibraryIndexEnsure(forceFilesRefresh)) {
      int pageSize = server.hasArg("pageSize") ? server.arg("pageSize").toInt() : (server.hasArg("limit") ? server.arg("limit").toInt() : 20);
      if (pageSize <= 0) pageSize = 20;
      if (pageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) pageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX;
      int page = server.hasArg("page") ? server.arg("page").toInt() : 0;
      if (server.hasArg("offset")) page = server.arg("offset").toInt() / pageSize;
      String q = server.hasArg("q") ? server.arg("q") : String(""); q.toLowerCase();
      String type = server.hasArg("type") ? server.arg("type") : String(""); type.toUpperCase();
      uint32_t totalMatches = 0;
      String pageEntries;
      pageEntries.reserve(4096);
      webLibraryAppendPagedEntries(pageEntries, g_webLibraryIndexJson, page, pageSize, q, type, totalMatches);
      j += ",\"libraryIndex\":1";
      j += ",\"page\":" + String(page);
      j += ",\"pageSize\":" + String(pageSize);
      j += ",\"filesTotal\":" + String(totalMatches);
      j += ",\"files\":[";
      j += pageEntries;
      j += "]";
    } else {
      j += ",\"files\":[";
      webAtrAppendFileListJsonCached(j, forceFilesRefresh);
      j += "]";
    }
  }
  j += "}";
  server.send(200, "application/json", j);
}

static void webAtrSendLightStatusJson(const char* action) {
  // F16: respuesta ultra-liviana. No abre ATRs ni calcula metadata completa;
  // eso se hace luego con /api/atr/status en refresco diferido.
  uint32_t t0 = millis();
  String j;
  j.reserve(900);
  j = "{";
  j += "\"ok\":1";
  j += ",\"action\":\"" + jsonEscape(String(action ? action : "webatr")) + "\"";
  j += ",\"compiled\":true";
  j += ",\"mode\":\"flash-multi\"";
  j += ",\"fastStatus\":1";
  j += ",\"enabled\":" + String(WEB_ATR_ENABLED ? 1 : 0);
  j += ",\"mask\":" + String(WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"forceMask\":" + String(WEB_ATR_FORCE_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"driveVisibleMask\":" + String(DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"maxUnits\":" + String(DRIVE_UI_MAX_UNITS);
  j += ",\"deferredSave\":" + String(g_webAtrConfigSavePending ? 1 : 0);
  j += ",\"slots\":[";
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    if (i) j += ",";
    j += "{\"unit\":" + String(i + 1);
    j += ",\"visible\":" + String(driveUiVisibleIndex(i) ? 1 : 0);
    j += ",\"name\":\"" + jsonEscape(g_webAtrMountedName[i]) + "\"";
    j += ",\"enabled\":" + String((WEB_ATR_ENABLED && (WEB_ATR_DEV_MASK & (1u << i))) ? 1 : 0);
    j += ",\"forced\":" + String((WEB_ATR_ENABLED && (WEB_ATR_FORCE_MASK & (1u << i))) ? 1 : 0);
    j += ",\"present\":" + String(g_webAtrSlotPresent[i] ? 1 : 0);
    j += "}";
  }
  j += "]";
  j += ",\"elapsedMs\":" + String(millis() - t0);
  j += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", j);
}
void handleSetWebAtr() {
  // F16: montar/desmontar desde la web debe ser instantáneo.
  // No escaneamos biblioteca, no validamos todas las unidades y no guardamos NVS antes de responder.
  uint32_t t0 = millis();
  uint8_t changedMask = 0;
  bool btChanged = false;

  if (server.hasArg("en")) {
    bool newEn = server.arg("en").toInt() != 0;
    if (WEB_ATR_ENABLED != newEn) WEB_ATR_ENABLED = newEn;
  }
  if (server.hasArg("mask")) {
    uint8_t newMask = (uint8_t)(server.arg("mask").toInt() & DRIVE_UI_MAX_MASK);
    if ((WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK) != newMask) WEB_ATR_DEV_MASK = newMask;
  }
  if (server.hasArg("force")) {
    uint8_t newForce = (uint8_t)(server.arg("force").toInt() & DRIVE_UI_MAX_MASK);
    if ((WEB_ATR_FORCE_MASK & DRIVE_UI_MAX_MASK) != newForce) WEB_ATR_FORCE_MASK = newForce;
  }

  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    char argn[4];
    snprintf(argn, sizeof(argn), "d%d", i + 1);
    if (server.hasArg(argn)) {
      String raw = server.arg(argn); raw.trim();
      String clean = raw.length() ? webAtrSanitizeFileName(raw) : String("");
      if (g_webAtrMountedName[i] != clean) {
        g_webAtrMountedName[i] = clean;
        webAtrResetResolvedSlot(i);
        changedMask |= (uint8_t)(1u << i);
        webAtrCacheInvalidateDev((uint8_t)(0x31 + i));
      }
    }
  }
  if (server.hasArg("unit") && server.hasArg("file")) {
    int u = server.arg("unit").toInt();
    if (u >= 1 && u <= WEB_ATR_MAX_UNITS) {
      String raw = server.arg("file"); raw.trim();
      String clean = raw.length() ? webAtrSanitizeFileName(raw) : String("");
      if (g_webAtrMountedName[u - 1] != clean) {
        g_webAtrMountedName[u - 1] = clean;
        webAtrResetResolvedSlot(u - 1);
        changedMask |= (uint8_t)(1u << (u - 1));
        webAtrCacheInvalidateDev((uint8_t)(0x30 + u));
      }
    }
  }

  normalizeDriveMasks();

  // Si WEB-ATR toma una unidad, se desmonta/desactiva BT-SIO2PC para esa unidad.
  uint8_t btBeforeMask = BT_SIO2PC_DEV_MASK;
  uint8_t btBeforeForce = BT_SIO2PC_FORCE_MASK;
  bool btBeforeEnabled = BT_SIO2PC_ENABLED;
  if (WEB_ATR_ENABLED && (WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK)) {
    clearBtSio2pcUnits(WEB_ATR_DEV_MASK);
  }
  btChanged = btChanged || btBeforeMask != BT_SIO2PC_DEV_MASK || btBeforeForce != BT_SIO2PC_FORCE_MASK || btBeforeEnabled != BT_SIO2PC_ENABLED;

  // Validación mínima: solo ATR/XEX de la unidad modificada. Esto mantiene operativo el SIO
  // sin abrir metadata de todas las unidades en cada click.
  if (changedMask) webAtrRefreshPresenceMask(changedMask);

  markDeferredConfigSave(true, btChanged);
  logf("[WEB-ATR] set fast changed=0x%02X elapsed=%lu ms deferredNVS=1",
       (unsigned)changedMask, (unsigned long)(millis() - t0));

  if (server.hasArg("reply") && server.arg("reply") == "full") handleAtrStatus();
  else webAtrSendLightStatusJson("set_webatr");
}
void handleAtrDelete() {
  String name = server.hasArg("file") ? webAtrSanitizeFileName(server.arg("file")) : String("");
  if (name.length() == 0) {
    server.send(400, "text/plain", "Falta parametro file");
    return;
  }
  if (webAtrIsBundledProtectedName(name)) {
    server.send(403, "text/plain", "ATR incluido protegido: no se puede eliminar desde la web");
    return;
  }
  bool isCas = webCasLooksLikeName(name);
  String path = isCas ? webCasPathForName(name) : webAtrPathForName(name);
  if (webAtrFsReady() && SPIFFS.exists(path)) SPIFFS.remove(path);
  webLibraryLiveScanCacheInvalidate("delete file");
  if (isCas) {
    String cacheBase = casAutoCacheBaseName(path);
    if (SPIFFS.exists(cacheBase + ".bin")) SPIFFS.remove(cacheBase + ".bin");
    if (SPIFFS.exists(cacheBase + ".json")) SPIFFS.remove(cacheBase + ".json");
  }
  webAtrInvalidateFilesCache();
  if (isCas && g_casMountedName == name) { g_casMounted = false; g_casMountedName = ""; g_casMountedPath = ""; saveCasConfigToNvs(); }
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) if (g_webAtrMountedName[i] == name) { g_webAtrMountedName[i] = ""; webAtrResetResolvedSlot(i); webAtrCacheInvalidateDev((uint8_t)(0x31 + i)); }
  webAtrRefreshPresence();
  saveWebAtrConfigToNvs();
  if (server.hasArg("reply") && server.arg("reply") == "full") handleAtrStatus();
  else webAtrSendLightStatusJson("delete");
}

void handleAtrUnmount() {
  int u = server.hasArg("unit") ? server.arg("unit").toInt() : 0;
  if (u < 1 || u > WEB_ATR_MAX_UNITS) {
    server.send(400, "text/plain", "Unidad inválida");
    return;
  }
  uint8_t bit = (uint8_t)(1u << (u - 1));
  g_webAtrMountedName[u - 1] = "";
  webAtrResetResolvedSlot(u - 1);
  webAtrCacheInvalidateDev((uint8_t)(0x30 + u));
  WEB_ATR_DEV_MASK &= (uint8_t)~bit;
  WEB_ATR_FORCE_MASK &= (uint8_t)~bit;
  if ((WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK) == 0) WEB_ATR_ENABLED = false;
  g_webAtrFilePresent = false;
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) if (g_webAtrSlotPresent[i]) { g_webAtrFilePresent = true; break; }
  markDeferredConfigSave(true, false);
  if (server.hasArg("reply") && server.arg("reply") == "full") handleAtrStatus();
  else webAtrSendLightStatusJson("unmount");
}
void handleAtrUploadDone() {
  if (g_webAtrUploadFile) g_webAtrUploadFile.close();
  const bool ajax = server.hasHeader("X-Requested-With") || server.hasHeader("X-Upload-Mode") || (server.hasArg("json") && server.arg("json") == "1");
  const bool wasCasUpload = g_webAtrPendingIsCas;
  String uploadedName = g_webAtrPendingStoredName;
  uint32_t uploadedBytes = g_webAtrUploadBytes;

  if (g_webAtrUploadError.length() > 0) {
    if (webAtrFsReady() && g_webAtrTmpPath.length() && SPIFFS.exists(g_webAtrTmpPath)) SPIFFS.remove(g_webAtrTmpPath);
    String err = g_webAtrUploadError;
    g_webAtrUploadError = "";
    g_webAtrTmpPath = "";
    g_webAtrPendingIsXex = false;
    g_webAtrPendingIsCas = false;
    if (ajax) {
      server.sendHeader("Cache-Control", "no-store");
      server.send(500, "application/json", String("{\"ok\":0,\"error\":\"") + jsonEscape(err) + "\"}");
    } else {
      server.send(500, "text/plain", err);
    }
    return;
  }

  // F49Z41: para CAS no es necesario reabrir ATRs ni recalcular presencia de disqueteras.
  webAtrInvalidateFilesCache();
  if (!wasCasUpload) webAtrRefreshPresence();
  markDeferredConfigSave(true, false, true);

  g_webAtrUploadError = "";
  g_webAtrTmpPath = "";
  g_webAtrPendingIsXex = false;
  g_webAtrPendingIsCas = false;

  if (ajax) {
    String j = "{";
    j += "\"ok\":1";
    j += ",\"name\":\"" + jsonEscape(uploadedName) + "\"";
    j += ",\"bytes\":" + String((unsigned long)uploadedBytes);
    j += ",\"cas\":" + String(wasCasUpload ? 1 : 0);
    j += ",\"deferredSave\":1";
    j += "}";
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", j);
    return;
  }

  server.sendHeader("Location", "/library");
  server.send(303, "text/plain", "Archivo cargado en biblioteca. ATR/XEX/COM/EXE se montan como D:, CAS queda preparado para C: cassette.");
}

void handleAtrUploadStream() {
  if (!webAtrFsReady()) { g_webAtrUploadError = "Almacenamiento no disponible"; return; }
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    if (g_webAtrUploadError.length() > 0) return;
    g_webAtrUploadBytes = 0;
    g_webAtrPendingIsXex = webAtrLooksLikeXexUpload(up.filename);
    g_webAtrPendingIsCas = webCasLooksLikeUpload(up.filename);
    g_webAtrPendingName = webAtrSanitizeFileName(up.filename);
    g_webAtrPendingStoredName = g_webAtrPendingName;
    String dest = g_webAtrPendingIsCas ? webCasPathForName(g_webAtrPendingStoredName) : webAtrPathForName(g_webAtrPendingStoredName);
    g_webAtrTmpPath = String(g_webAtrPendingIsCas ? WEB_CAS_TMP_PATH : WEB_ATR_TMP_PATH) + "_" + g_webAtrPendingStoredName + ".tmp";
    if (SPIFFS.exists(g_webAtrTmpPath)) SPIFFS.remove(g_webAtrTmpPath);

    // Para ahorrar flash interna al reemplazar un ATR existente, liberamos primero el destino.
    // Si la subida falla, el archivo anterior se habrá eliminado; esto permite ATRs más grandes.
    if (dest.length() && SPIFFS.exists(dest)) SPIFFS.remove(dest);

    g_webAtrUploadFile = SPIFFS.open(g_webAtrTmpPath, "w");
    if (!g_webAtrUploadFile) g_webAtrUploadError = "No se pudo crear archivo temporal";
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_webAtrUploadError.length() == 0 && g_webAtrUploadFile) {
      g_webAtrUploadBytes += up.currentSize;
      uint32_t total = SPIFFS.totalBytes();
      uint32_t used = SPIFFS.usedBytes();
      const uint32_t reserve = 16UL * 1024UL;
      if (total > reserve && used + up.currentSize + reserve > total) {
        g_webAtrUploadError = String("No hay espacio suficiente en el almacenamiento para este archivo. ") + webAtrFsDiagString();
      } else {
        size_t w = g_webAtrUploadFile.write(up.buf, up.currentSize);
        if (w != up.currentSize) g_webAtrUploadError = String("Error escribiendo archivo en almacenamiento. ") + webAtrFsDiagString() + String(" chunk=") + String((unsigned long)up.currentSize);
      }
    }
  } else if (up.status == UPLOAD_FILE_END || up.status == UPLOAD_FILE_ABORTED) {
    if (g_webAtrUploadFile) g_webAtrUploadFile.close();
    if (up.status == UPLOAD_FILE_ABORTED) {
      if (g_webAtrUploadError.length() == 0) g_webAtrUploadError = "Upload abortado";
      if (g_webAtrTmpPath.length() && SPIFFS.exists(g_webAtrTmpPath)) SPIFFS.remove(g_webAtrTmpPath);
      return;
    }
    if (g_webAtrUploadError.length() == 0) {
      String dest = g_webAtrPendingIsCas ? webCasPathForName(g_webAtrPendingStoredName) : webAtrPathForName(g_webAtrPendingStoredName);
      WebAtrMeta m;
      if (g_webAtrTmpPath.length() == 0 || !SPIFFS.exists(g_webAtrTmpPath)) {
        g_webAtrUploadError = "No se recibió archivo";
      } else if (g_webAtrPendingIsCas) {
        String moveErr;
        if (dest.length() == 0 || !webAtrMoveFile(g_webAtrTmpPath, dest, moveErr)) {
          if (g_webAtrTmpPath.length() && SPIFFS.exists(g_webAtrTmpPath)) SPIFFS.remove(g_webAtrTmpPath);
          g_webAtrUploadError = moveErr.length() ? moveErr : String("No se pudo mover CAS temporal a /CAS");
        } else {
          String cacheBase = casAutoCacheBaseName(dest);
          if (SPIFFS.exists(cacheBase + ".bin")) SPIFFS.remove(cacheBase + ".bin");
          if (SPIFFS.exists(cacheBase + ".json")) SPIFFS.remove(cacheBase + ".json");
          File cf = SPIFFS.open(dest, "r");
          g_casBytesLast = cf ? (uint32_t)cf.size() : 0;
          if (cf) cf.close();
          g_casUploads++;
          casSetLastError("");
          logf("[CAS] Upload CAS OK name=%s path=%s bytes=%lu free=%lu",
               g_webAtrPendingStoredName.c_str(), dest.c_str(),
               (unsigned long)g_casBytesLast,
               (unsigned long)(SPIFFS.totalBytes() - SPIFFS.usedBytes()));
        }
      } else if (g_webAtrPendingIsXex) {
        String convErr;
        if (!webAtrConvertXexToDosAtr(g_webAtrTmpPath, dest, g_webAtrPendingStoredName, convErr)) {
          if (SPIFFS.exists(g_webAtrTmpPath)) SPIFFS.remove(g_webAtrTmpPath);
          if (dest.length() && SPIFFS.exists(dest)) SPIFFS.remove(dest);
          g_webAtrUploadError = convErr.length() ? convErr : String("No se pudo preparar XEX montable");
        } else {
          if (SPIFFS.exists(g_webAtrTmpPath)) SPIFFS.remove(g_webAtrTmpPath);
          webAtrReadMetaFromPath(dest, m);
          logf("[WEB-ATR] Upload XEX OK nombre-original=%s path=%s bytes=%lu secSize=%u sectors=%lu free=%lu",
               g_webAtrPendingStoredName.c_str(), dest.c_str(),
               (unsigned long)m.fileSize, (unsigned)m.sectorSize, (unsigned long)m.totalSectors,
               (unsigned long)(SPIFFS.totalBytes() - SPIFFS.usedBytes()));
        }
      } else if (!webAtrReadMetaFromPath(g_webAtrTmpPath, m)) {
        SPIFFS.remove(g_webAtrTmpPath);
        g_webAtrUploadError = "ATR inválido: cabecera o tamaño no reconocido";
      } else {
        String moveErr;
        if (dest.length() == 0 || !webAtrMoveFile(g_webAtrTmpPath, dest, moveErr)) {
          if (g_webAtrTmpPath.length() && SPIFFS.exists(g_webAtrTmpPath)) SPIFFS.remove(g_webAtrTmpPath);
          g_webAtrUploadError = moveErr.length() ? moveErr : String("No se pudo mover ATR temporal a /ATR");
        } else {
        logf("[WEB-ATR] Upload ATR OK name=%s path=%s bytes=%lu secSize=%u sectors=%lu free=%lu",
             g_webAtrPendingStoredName.c_str(), dest.c_str(),
             (unsigned long)m.fileSize, (unsigned)m.sectorSize, (unsigned long)m.totalSectors,
             (unsigned long)(SPIFFS.totalBytes() - SPIFFS.usedBytes()));
        }
      }
    }
    g_webAtrTmpPath = "";
    g_webAtrPendingIsXex = false;
    g_webAtrPendingIsCas = false;
  }
}


void handleCasStatus() {
  bool liteStatus = (server.hasArg("lite") && server.arg("lite") == "1") || (server.hasArg("fast") && server.arg("fast") == "1");
  // F49Z41: el status rápido no recorre el CAS. El análisis queda para diagnóstico completo
  // o para Play Auto, evitando bloqueos al entrar a Cassette.
  if (!liteStatus && g_casMounted && !g_casAutoAnalysis.valid && g_casMountedPath.length()) casAnalyzeCasFile(g_casMountedPath);
  String j = "{";
  j += "\"ok\":true";
  j += ",\"lite\":" + String(liteStatus ? "true" : "false");
  j += ",\"compiled\":true";
  j += ",\"mode\":\"manual-play\"";
  j += ",\"loadMode\":\"" + jsonEscape(String(g_casLoadMode)) + "\"";
  j += ",\"bootMode\":" + String(g_casBootModeActive ? "true" : "false");
  j += ",\"device\":\"C:\"";
  j += ",\"mounted\":" + String(g_casMounted ? "true" : "false");
  j += ",\"playing\":" + String(g_casPlaying ? "true" : "false");
  j += ",\"casPlaying\":" + String(g_casPlaying ? 1 : 0);
  j += ",\"paused\":" + String(g_casPaused ? "true" : "false");
  j += ",\"casPaused\":" + String(g_casPaused ? 1 : 0);
  j += ",\"eof\":" + String(g_casEof ? "true" : "false");
  j += ",\"rawMode\":" + String(g_casRawMode ? "true" : "false");
  j += ",\"fujiFormat\":" + String(g_casFujiFormat ? "true" : "false");
  j += ",\"format\":\"" + String(g_casFujiFormat ? "FUJI-CAS" : (g_casRawMode ? "RAW" : "CAS")) + "\"";
  j += ",\"parserStartOffset\":" + String((unsigned long)g_casParserStartOffset);
  j += ",\"sdriveStreamMode\":" + String(g_casSdriveStreamMode ? "true" : "false");
  j += ",\"timingCompat\":" + String(!g_casSdriveStreamMode ? "true" : "false");
  j += ",\"recordDrainPending\":" + String(g_casRecordDrainPending ? "true" : "false");
  j += ",\"lastIrgAppliedMs\":" + String(g_casLastIrgAppliedMs);
  j += ",\"timingWaitMs\":" + String((g_casTimingWaitUntilMs && (int32_t)(g_casTimingWaitUntilMs - millis()) > 0) ? (uint32_t)(g_casTimingWaitUntilMs - millis()) : 0);
  j += ",\"drainTargetBytes\":" + String((unsigned long)g_casDrainTargetBytes);
  j += ",\"drainDeadlineMs\":" + String((g_casDrainDeadlineMs && (int32_t)(g_casDrainDeadlineMs - millis()) > 0) ? (uint32_t)(g_casDrainDeadlineMs - millis()) : 0);
  j += ",\"recordExpectedMs\":" + String((unsigned long)g_casRecordExpectedMs);
  j += ",\"transportTargetRx\":" + String((unsigned long)g_casTransportTargetRx);
  j += ",\"transportWaitSkips\":" + String((unsigned long)g_casTransportWaitSkips);
  j += ",\"transportTimeouts\":" + String((unsigned long)g_casTransportTimeouts);
  j += ",\"drainTimeouts\":" + String((unsigned long)g_casDrainTimeouts);
  j += ",\"strictRecordGate\":" + String(g_casStrictRecordGate ? "true" : "false");
  j += ",\"drainHoldMs\":" + String((unsigned long)g_casDrainHoldMs);
  j += ",\"drainHardTimeouts\":" + String((unsigned long)g_casDrainHardTimeouts);
  j += ",\"postRecordSettleMs\":" + String((g_casPostRecordSettleUntilMs && (int32_t)(g_casPostRecordSettleUntilMs - millis()) > 0) ? (uint32_t)(g_casPostRecordSettleUntilMs - millis()) : 0);
  j += ",\"recordsStarted\":" + String((unsigned long)g_casRecordsStarted);
  j += ",\"recordsCompleted\":" + String((unsigned long)g_casRecordsCompleted);
  j += ",\"bootFirstIrgSkipped\":" + String(g_casBootFirstIrgSkipped ? "true" : "false");
  j += ",\"sdriveExactMode\":" + String(g_casSdriveExactMode ? "true" : "false");
  j += ",\"sdriveBlockExactMode\":" + String(g_casSdriveBlockExactMode ? "true" : "false");
  j += ",\"sdriveRecordBufferMode\":true";
  j += ",\"rpRecordBufferBytes\":2048";
  j += ",\"sdriveRawRecordMode\":" + String(g_casSdriveRawRecordMode ? "true" : "false");
  j += ",\"aspeqtCompat\":" + String(g_casAspeqtCompat ? "true" : "false");
  j += ",\"aspeqtFragmentBytes\":" + String((unsigned)g_casAspeqtFragmentBytes);
  j += ",\"aspeqtFragments\":" + String((unsigned long)g_casAspeqtFragments);
  j += ",\"casEmbeddedBaud\":" + String((unsigned)g_casEmbeddedBaud);
  j += ",\"aspeqtEffectiveBaud\":" + String((unsigned)g_casAspeqtEffectiveBaud);
  j += ",\"aspeqtFinalWaitMs\":" + String((unsigned long)g_casAspeqtFinalWaitMs);
  j += ",\"rawRecordsBuilt\":" + String((unsigned long)g_casRawRecordsBuilt);
  j += ",\"rawLastMarker\":" + String((unsigned)g_casRawLastMarker);
  j += ",\"wavDirectSupported\":false";
  j += ",\"ironTurboCompat\":" + String(g_casIronTurboCompat ? "true" : "false");
  j += ",\"ironTurboDetected\":" + String(g_casIronTurboDetected ? "true" : "false");
  j += ",\"ironTurboEffective\":" + String((g_casIronTurboCompat || g_casIronTurboDetected) ? "true" : "false");
  j += ",\"casEffectiveProfile\":\"" + jsonEscape(String(g_casEffectiveProfile)) + "\"";
  j += ",\"casEffectiveProfileManual\":" + String(g_casEffectiveProfileManual ? "true" : "false");
  j += ",\"casProfileLockBaud\":" + String(g_casProfileLockBaud ? "true" : "false");
  j += ",\"casProfileForcedBaud\":" + String((unsigned)g_casProfileForcedBaud);
  j += ",\"ironTurboTurboBaud\":" + String((unsigned)g_casIronTurboTurboBaud);
  j += ",\"ironTurboGapFixes\":" + String((unsigned long)g_casIronTurboGapFixes);
  j += ",\"prevCompletedRecordMarker\":" + String((unsigned)g_casPrevCompletedRecordMarker);
  j += ",\"sdriveEndMarkers\":" + String(g_casSdriveEndMarkers);
  j += ",\"sdriveEndGapPending\":" + String(g_casSdriveEndGapPending ? "true" : "false");
  j += ",\"sdriveEndGapMs\":" + String(g_casSdriveEndGapMs);
  j += ",\"currentRecordMarker\":" + String((unsigned)g_casCurrentRecordMarker);
  j += ",\"currentRecordLen\":" + String((unsigned)g_casCurrentRecordLen);
  j += ",\"rpRecordBusy\":" + String((g_casRpFlags & 0x10) ? "true" : "false");
  j += ",\"rpRecordBusySkips\":" + String((unsigned long)g_casRpRecordBusySkips);
  j += ",\"shortIrgMs\":" + String(g_casShortIrgMs);
  j += ",\"timingSkips\":" + String(g_casTimingSkips);
  j += ",\"lastChunkType\":\"" + jsonEscape(String(g_casLastChunkType)) + "\"";
  j += ",\"lastChunkLen\":" + String(g_casLastChunkLen);
  j += ",\"lastChunkAuxMs\":" + String(g_casLastChunkAux);
  j += ",\"parserErrors\":" + String(g_casParserErrors);
  j += ",\"d1RecommendedEmpty\":true";
  j += ",\"name\":\"" + jsonEscape(g_casMountedName) + "\"";
  j += ",\"path\":\"" + jsonEscape(g_casMountedPath) + "\"";
  j += ",\"uploads\":" + String(g_casUploads);
  j += ",\"mounts\":" + String(g_casMounts);
  j += ",\"unmounts\":" + String(g_casUnmounts);
  j += ",\"playCount\":" + String(g_casPlayCount);
  j += ",\"stopCount\":" + String(g_casStopCount);
  j += ",\"rewindCount\":" + String(g_casRewindCount);
  j += ",\"lastBytes\":" + String(g_casBytesLast);
  j += ",\"playSize\":" + String(g_casPlaySize);
  j += ",\"casFileBytes\":" + String((unsigned long)(g_casAutoAnalysis.fileBytes ? g_casAutoAnalysis.fileBytes : (g_casPlaySize ? g_casPlaySize : g_casBytesLast)));
  j += ",\"casPayloadBytes\":" + String((unsigned long)g_casAutoAnalysis.payloadBytes);
  j += ",\"casOverheadBytes\":" + String((unsigned long)g_casAutoAnalysis.overheadBytes);
  j += ",\"playPos\":" + String(g_casPlayPos);
  j += ",\"tapeCounter\":" + String((unsigned long)(g_casPlayPos / 128UL));
  j += ",\"casTapeCounter\":" + String((unsigned long)(g_casPlayPos / 128UL));
  j += ",\"pauseCount\":" + String((unsigned long)g_casPauseCount);
  j += ",\"resumeCount\":" + String((unsigned long)g_casResumeCount);
  j += ",\"seekBackCount\":" + String((unsigned long)g_casSeekBackCount);
  j += ",\"lastSeekBackRecords\":" + String((unsigned)g_casLastSeekBackRecords);
  j += ",\"baud\":" + String(g_casCurrentBaud);
  j += ",\"bytesQueued\":" + String(g_casBytesQueued);
  j += ",\"chunksQueued\":" + String(g_casChunksQueued);
  j += ",\"dataBlocks\":" + String(g_casDataBlocks);
  j += ",\"skippedChunks\":" + String(g_casSkippedChunks);
  j += ",\"fskChunks\":" + String(g_casFskChunks);
  j += ",\"rpState\":" + String(g_casRpState);
  j += ",\"rpBufferUsed\":" + String(g_casRpBufferUsed);
  j += ",\"rpBufferFree\":" + String(g_casRpBufferFree);
  j += ",\"rpBytesPlayed\":" + String(g_casRpBytesPlayed);
  j += ",\"rpBytesRx\":" + String(g_casRpBytesRx);
  j += ",\"rpBaud\":" + String(g_casRpBaud);
  j += ",\"rpFlags\":" + String(g_casRpFlags);
  j += ",\"rpLastStatusMs\":" + String((unsigned long)g_casRpLastStatusMs);
  j += ",\"lastError\":\"" + jsonEscape(String(g_casLastError)) + "\"";
  j += ",\"casAutoValid\":" + String(g_casAutoAnalysis.valid ? "true" : "false");
  j += ",\"casAutoProfile\":\"" + jsonEscape(String(g_casAutoAnalysis.profile)) + "\"";
  j += ",\"casAutoConfidence\":\"" + jsonEscape(String(g_casAutoAnalysis.confidence)) + "\"";
  j += ",\"casAutoSuggestedMode\":\"" + jsonEscape(String(g_casAutoAnalysis.suggestedMode)) + "\"";
  j += ",\"casAutoInitialBaud\":" + String((unsigned)g_casAutoAnalysis.initialBaud);
  j += ",\"casAutoTurboBaud\":" + String((unsigned)g_casAutoAnalysis.turboBaud);
  j += ",\"casAutoFirstDataBaud\":" + String((unsigned)g_casAutoAnalysis.firstDataBaud);
  j += ",\"casAutoMaxBaud\":" + String((unsigned)g_casAutoAnalysis.maxBaud);
  j += ",\"casAutoDataBlocks\":" + String((unsigned)g_casAutoAnalysis.dataBlocks);
  j += ",\"casAutoBaudChunks\":" + String((unsigned)g_casAutoAnalysis.baudChunks);
  j += ",\"casAutoTotalChunks\":" + String((unsigned)g_casAutoAnalysis.totalChunks);
  j += ",\"casAutoFskChunks\":" + String((unsigned)g_casAutoAnalysis.fskChunks);
  j += ",\"casAutoPwmChunks\":" + String((unsigned)g_casAutoAnalysis.pwmChunks);
  j += ",\"casAutoPwmStateChunks\":" + String((unsigned)g_casAutoAnalysis.pwmStateChunks);
  j += ",\"casAutoPwmControlChunks\":" + String((unsigned)g_casAutoAnalysis.pwmControlChunks);
  j += ",\"casAutoPwmDataChunks\":" + String((unsigned)g_casAutoAnalysis.pwmDataChunks);
  j += ",\"casAutoPwmLongChunks\":" + String((unsigned)g_casAutoAnalysis.pwmLongChunks);
  j += ",\"casAutoStandard132Blocks\":" + String((unsigned)g_casAutoAnalysis.standard132Blocks);
  j += ",\"casAutoStacSync55Blocks\":" + String((unsigned)g_casAutoAnalysis.stacSync55Blocks);
  j += ",\"casAutoCain217Blocks\":" + String((unsigned)g_casAutoAnalysis.cain217Blocks);
  j += ",\"casAutoLongGaps\":" + String((unsigned)g_casAutoAnalysis.longGaps);
  j += ",\"casAutoShortGaps\":" + String((unsigned)g_casAutoAnalysis.shortGaps);
  j += ",\"casAutoMinIrgMs\":" + String((unsigned)g_casAutoAnalysis.minIrgMs);
  j += ",\"casAutoMaxIrgMs\":" + String((unsigned)g_casAutoAnalysis.maxIrgMs);
  j += ",\"casAutoLastIrgMs\":" + String((unsigned)g_casAutoAnalysis.lastIrgMs);
  j += ",\"casAutoAutoPauseCandidates\":" + String((unsigned)g_casAutoAnalysis.autoPauseCandidates);
  j += ",\"casAutoPauseThresholdMs\":3000";
  j += ",\"casAutoPauseActive\":false";
  j += ",\"casAutoMotorMode\":\"manual-no-motor\"";
  j += ",\"casAutoFuji\":" + String(g_casAutoAnalysis.fujiFormat ? "true" : "false");
  j += ",\"casAutoRaw\":" + String(g_casAutoAnalysis.rawFormat ? "true" : "false");
  j += ",\"casAutoPwmTurbo\":" + String(g_casAutoAnalysis.hasPwmChunks ? "true" : "false");
  j += ",\"casAutoFsk\":" + String(g_casAutoAnalysis.hasFskChunks ? "true" : "false");
  j += ",\"casAutoTextIron\":" + String(g_casAutoAnalysis.textIron ? "true" : "false");
  j += ",\"casAutoTextTurbo\":" + String(g_casAutoAnalysis.textTurbo ? "true" : "false");
  j += ",\"casAutoTextStac\":" + String(g_casAutoAnalysis.textStac ? "true" : "false");
  j += ",\"casAutoTextCain\":" + String(g_casAutoAnalysis.textCain ? "true" : "false");
  j += ",\"casAutoChunkSummary\":\"" + jsonEscape(String(g_casAutoAnalysis.chunkSummary)) + "\"";
  j += ",\"casAutoNotes\":\"" + jsonEscape(String(g_casAutoAnalysis.notes)) + "\"";
  j += ",\"note\":\"F49Z40: metricas CAS separan archivo vs datos reproducibles.\"";
  j += "}";
  server.send(200, "application/json", j);
}

void handleCasAnalyze() {
  String name = "";
  String path = "";

  if (server.hasArg("file")) {
    name = webAtrSanitizeFileName(server.arg("file"));
    if (!webCasLooksLikeName(name)) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"Parametro file debe ser .CAS\"}");
      return;
    }
    path = webCasPathForName(name);
  } else if (g_casMounted && g_casMountedPath.length()) {
    name = g_casMountedName;
    path = g_casMountedPath;
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"Falta file o CAS montado en C:\"}");
    return;
  }

  bool force = server.hasArg("force") && server.arg("force").toInt() != 0;
  if (force) {
    String cacheBase = casAutoCacheBaseName(path);
    if (SPIFFS.exists(cacheBase + ".bin")) SPIFFS.remove(cacheBase + ".bin");
    if (SPIFFS.exists(cacheBase + ".json")) SPIFFS.remove(cacheBase + ".json");
    casAutoResetAnalysis();
  }

  bool ok = casAnalyzeCasFile(path);
  // F41: si se analiza desde Biblioteca, la cache queda persistida y se reconstruye
  // el índice para que /api/library exponga casAnalyzed/casProfile sin esperar otro refresco manual.
  bool indexRebuilt = false;
  // F42J: analizar CAS no reconstruye ni reemplaza el índice de Biblioteca.
  // La tarjeta se actualiza en memoria del navegador; el índice persistente queda protegido.
  String casOverrideProfile;
  bool hasCasOverride = casProfileOverrideGet(name, casOverrideProfile);
  String casEffectiveProfile = hasCasOverride ? casOverrideProfile : String(g_casAutoAnalysis.profile);
  String j = "{";
  j += "\"ok\":" + String(ok ? "true" : "false");
  j += ",\"build\":\"" + jsonEscape(String(MASTER_BUILD)) + "\"";
  j += ",\"file\":\"" + jsonEscape(name) + "\"";
  j += ",\"path\":\"" + jsonEscape(path) + "\"";
  j += ",\"profile\":\"" + jsonEscape(String(g_casAutoAnalysis.profile)) + "\"";
  j += ",\"confidence\":\"" + jsonEscape(String(g_casAutoAnalysis.confidence)) + "\"";
  j += ",\"suggestedMode\":\"" + jsonEscape(String(g_casAutoAnalysis.suggestedMode)) + "\"";
  j += ",\"indexRebuilt\":" + String(indexRebuilt ? "true" : "false");
  j += ",\"casOverride\":" + String(hasCasOverride ? "true" : "false");
  j += ",\"casOverrideProfile\":\"" + jsonEscape(hasCasOverride ? casOverrideProfile : String("AUTO")) + "\"";
  j += ",\"casEffectiveProfile\":\"" + jsonEscape(casEffectiveProfile) + "\"";
  j += ",\"fuji\":" + String(g_casAutoAnalysis.fujiFormat ? "true" : "false");
  j += ",\"raw\":" + String(g_casAutoAnalysis.rawFormat ? "true" : "false");
  j += ",\"fileBytes\":" + String((unsigned long)g_casAutoAnalysis.fileBytes);
  j += ",\"payloadBytes\":" + String((unsigned long)g_casAutoAnalysis.payloadBytes);
  j += ",\"chunks\":" + String((unsigned)g_casAutoAnalysis.totalChunks);
  j += ",\"dataBlocks\":" + String((unsigned)g_casAutoAnalysis.dataBlocks);
  j += ",\"baudChunks\":" + String((unsigned)g_casAutoAnalysis.baudChunks);
  j += ",\"fskChunks\":" + String((unsigned)g_casAutoAnalysis.fskChunks);
  j += ",\"pwmChunks\":" + String((unsigned)g_casAutoAnalysis.pwmChunks);
  j += ",\"initialBaud\":" + String((unsigned)g_casAutoAnalysis.initialBaud);
  j += ",\"firstDataBaud\":" + String((unsigned)g_casAutoAnalysis.firstDataBaud);
  j += ",\"maxBaud\":" + String((unsigned)g_casAutoAnalysis.maxBaud);
  j += ",\"turboBaud\":" + String((unsigned)g_casAutoAnalysis.turboBaud);
  j += ",\"standard132Blocks\":" + String((unsigned)g_casAutoAnalysis.standard132Blocks);
  j += ",\"stacSync55Blocks\":" + String((unsigned)g_casAutoAnalysis.stacSync55Blocks);
  j += ",\"cain217Blocks\":" + String((unsigned)g_casAutoAnalysis.cain217Blocks);
  j += ",\"longGaps\":" + String((unsigned)g_casAutoAnalysis.longGaps);
  j += ",\"shortGaps\":" + String((unsigned)g_casAutoAnalysis.shortGaps);
  j += ",\"textFlags\":{\"iron\":" + String(g_casAutoAnalysis.textIron ? "true" : "false");
  j += ",\"turbo\":" + String(g_casAutoAnalysis.textTurbo ? "true" : "false");
  j += ",\"t2000\":" + String(g_casAutoAnalysis.textT2000 ? "true" : "false");
  j += ",\"injektor\":" + String(g_casAutoAnalysis.textInjektor ? "true" : "false");
  j += ",\"stac\":" + String(g_casAutoAnalysis.textStac ? "true" : "false");
  j += ",\"cain\":" + String(g_casAutoAnalysis.textCain ? "true" : "false");
  j += "}";
  j += ",\"chunkSummary\":\"" + jsonEscape(String(g_casAutoAnalysis.chunkSummary)) + "\"";
  j += ",\"notes\":\"" + jsonEscape(String(g_casAutoAnalysis.notes)) + "\"";
  j += "}";
  server.send(200, "application/json", j);
}


void handleCasProfile() {
  String name = server.hasArg("file") ? webAtrSanitizeFileName(server.arg("file")) : String("");
  if (!name.length() || !webCasLooksLikeName(name)) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"Falta archivo .CAS\"}");
    return;
  }

  String profile = server.hasArg("profile") ? server.arg("profile") : String("");
  bool doSet = server.hasArg("profile") || (server.hasArg("set") && server.arg("set").toInt() != 0);

  if (doSet) {
    profile = casProfileNormalize(profile);
    String err, effective;
    bool ok = casProfileOverrideSet(name, profile, err, effective);
    bool indexRebuilt = false;
    // F42J: cambiar perfil CAS no reconstruye ni reemplaza el índice de Biblioteca.

    String autoProfile = "";
    CasAutoAnalysis cachedCas;
    bool casCached = casAutoLoadCacheForPath(webCasPathForName(name), cachedCas);
    if (casCached) autoProfile = String(cachedCas.profile);
    String outEffective = (profile == "AUTO") ? (casCached ? autoProfile : String("AUTO")) : effective;

    String j = "{";
    j += "\"ok\":" + String(ok ? "true" : "false");
    j += ",\"build\":\"" + jsonEscape(String(MASTER_BUILD)) + "\"";
    j += ",\"file\":\"" + jsonEscape(name) + "\"";
    j += ",\"profile\":\"" + jsonEscape(profile) + "\"";
    j += ",\"override\":" + String((ok && profile != "AUTO") ? "true" : "false");
    j += ",\"overrideProfile\":\"" + jsonEscape((ok && profile != "AUTO") ? profile : String("AUTO")) + "\"";
    j += ",\"autoProfile\":\"" + jsonEscape(casCached ? autoProfile : String("")) + "\"";
    j += ",\"effectiveProfile\":\"" + jsonEscape(outEffective) + "\"";
    j += ",\"indexRebuilt\":" + String(indexRebuilt ? "true" : "false");
    if (!ok) j += ",\"error\":\"" + jsonEscape(err) + "\"";
    j += "}";
    server.send(ok ? 200 : 500, "application/json", j);
    return;
  }

  String overrideProfile;
  bool hasOverride = casProfileOverrideGet(name, overrideProfile);
  CasAutoAnalysis cachedCas;
  bool casCached = casAutoLoadCacheForPath(webCasPathForName(name), cachedCas);
  String effective = hasOverride ? overrideProfile : (casCached ? String(cachedCas.profile) : String("AUTO"));

  String j = "{";
  j += "\"ok\":true";
  j += ",\"build\":\"" + jsonEscape(String(MASTER_BUILD)) + "\"";
  j += ",\"file\":\"" + jsonEscape(name) + "\"";
  j += ",\"override\":" + String(hasOverride ? "true" : "false");
  j += ",\"overrideProfile\":\"" + jsonEscape(hasOverride ? overrideProfile : String("AUTO")) + "\"";
  j += ",\"autoProfile\":\"" + jsonEscape(casCached ? String(cachedCas.profile) : String("")) + "\"";
  j += ",\"effectiveProfile\":\"" + jsonEscape(effective) + "\"";
  j += "}";
  server.send(200, "application/json", j);
}

void handleCasMount() {
  String name = server.hasArg("file") ? webAtrSanitizeFileName(server.arg("file")) : String("");
  bool fastReply = server.hasArg("fast") && server.arg("fast").toInt() != 0;
  if (name.length() == 0 || !webCasLooksLikeName(name)) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"Falta archivo .CAS\"}");
    return;
  }
  String path = webCasPathForName(name);
  if (!webAtrFsReady() || !SPIFFS.exists(path)) {
    casSetLastError(String("CAS no encontrado: ") + name);
    server.send(404, "application/json", "{\"ok\":false,\"error\":\"CAS no encontrado\"}");
    return;
  }
  File f = SPIFFS.open(path, "r");
  g_casBytesLast = f ? (uint32_t)f.size() : 0;
  if (f) f.close();
  g_casMountedName = name;
  g_casMountedPath = path;
  g_casMounted = true;
  g_casMounts++;

  // F49Z13: Biblioteca debe responder rápido al botón Preparar C:.
  // El análisis completo CAS/FUJI puede recorrer chunks y abrir la flash varias veces;
  // lo diferimos para /cassette o para un status completo, donde sí se necesita.
  casAutoResetAnalysis();
  if (!fastReply) casAnalyzeCasFile(path);

  casSetLastError("");
  webAtrPatchCasFlagsInFilesCache(g_casMountedName, true, false);

  if (fastReply) {
    // F49Z14: no escribimos NVS ni reconstruimos cache de Biblioteca antes de responder.
    // La UI actualiza la tarjeta localmente y el guardado persistente se hace luego desde loop().
    markDeferredConfigSave(false, false, true);
    String j = "{";
    j += "\"ok\":true";
    j += ",\"fast\":true";
    j += ",\"mounted\":true";
    j += ",\"casMounted\":1";
    j += ",\"playing\":false";
    j += ",\"casPlaying\":0";
    j += ",\"name\":\"" + jsonEscape(g_casMountedName) + "\"";
    j += ",\"casName\":\"" + jsonEscape(g_casMountedName) + "\"";
    j += ",\"path\":\"" + jsonEscape(g_casMountedPath) + "\"";
    j += ",\"lastBytes\":" + String((unsigned long)g_casBytesLast);
    j += ",\"casAutoValid\":false";
    j += ",\"note\":\"F49Z14: montaje CAS rapido; analisis y NVS diferidos\"";
    j += "}";
    server.send(200, "application/json", j);
    return;
  }

  saveCasConfigToNvs();
  webAtrPatchCasFlagsInFilesCache(g_casMountedName, true, g_casPlaying);
  handleCasStatus();
}

void handleCasDownload() {
  String name = server.hasArg("file") ? webAtrSanitizeFileName(server.arg("file")) : String("");
  if (name.length() == 0 || !webCasLooksLikeName(name)) {
    server.send(400, "text/plain", "Falta archivo .CAS");
    return;
  }
  String path = webCasPathForName(name);
  if (!webAtrFsReady() || !SPIFFS.exists(path)) {
    server.send(404, "text/plain", "CAS no encontrado");
    return;
  }
  File f = SPIFFS.open(path, "r");
  if (!f) {
    server.send(500, "text/plain", "No se pudo abrir CAS");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");
  server.streamFile(f, "application/octet-stream");
  f.close();
}


void handleCasUnmount() {
  bool fastReply = server.hasArg("fast") && server.arg("fast").toInt() != 0;
  String oldName = g_casMountedName;
  if (g_casPlaying) {
    g_casPlaying = false;
    g_casPaused = false;
    g_casEof = false;
    casClosePlaybackFile();
    casSendControlToRP(2, g_casCurrentBaud, g_casBytesQueued);
  }
  g_casBootModeActive = false;
  strncpy(g_casLoadMode, "cload", sizeof(g_casLoadMode));
  g_casMounted = false;
  g_casMountedName = "";
  g_casMountedPath = "";
  casAutoResetAnalysis();
  g_casUnmounts++;

  // F49Z16: soltar C: no cambia archivos de Biblioteca. No invalidar la cache,
  // solo limpiar flags CAS dentro del cache existente para conservar el listado cargado.
  webAtrPatchCasFlagsInFilesCache("", false, false);
  if (fastReply) {
    markDeferredConfigSave(false, false, true);
    String j = "{";
    j += "\"ok\":true";
    j += ",\"fast\":true";
    j += ",\"mounted\":false";
    j += ",\"casMounted\":0";
    j += ",\"playing\":false";
    j += ",\"casPlaying\":0";
    j += ",\"oldName\":\"" + jsonEscape(oldName) + "\"";
    j += ",\"note\":\"F49Z16: C liberado sin invalidar Biblioteca\"";
    j += "}";
    server.send(200, "application/json", j);
    return;
  }
  saveCasConfigToNvs();
  handleCasStatus();
}

void casSendControlToRP(uint8_t cmd, uint32_t baud, uint32_t totalBytes) {
  uint8_t frame[10];
  frame[0] = TYPE_CAS_CONTROL;
  frame[1] = cmd;
  putLE32(&frame[2], baud);
  putLE32(&frame[6], totalBytes);
  sendUartFrameToRP(frame, sizeof(frame));
}

static void casClosePlaybackFile() {
  if (g_casPlayFile) g_casPlayFile.close();
  g_casInDataChunk = false;
  g_casChunkRemain = 0;
  g_casChunkGapMs = 0;
  g_casChunkGapSent = false;
  g_casPendingBlockDelay = false;
  g_casPendingBlockDelayMs = 0;
  g_casRecordDrainPending = false;
  g_casLastIrgAppliedMs = 0;
  g_casTimingWaitUntilMs = 0;
  g_casDrainWaitStartMs = 0;
  g_casDrainTargetBytes = 0;
  g_casDrainDeadlineMs = 0;
  g_casRecordExpectedMs = 0;
  g_casTransportTargetRx = 0;
  g_casPostRecordSettleUntilMs = 0;
  g_casSdriveEndGapPending = false;
  g_casSdriveEndGapMs = 0;
  g_casTransportTargetRx = 0;
  g_casAckSeq = 0;
  g_casAckOk = false;
  g_casAckBytesRx = 0;
  g_casCurrentRecordMarker = 0;
  g_casCurrentRecordLen = 0;
  g_casAspeqtFragments = 0;
  g_casSdriveRawRecordMode = false;
  g_casRawRecordLen = 0;
  g_casRawRecordPos = 0;
  g_casRawEndRecordQueued = false;
}


static bool casReadExact(uint8_t* out, size_t n) {
  if (!g_casPlayFile || !out) return false;
  return g_casPlayFile.read(out, n) == (int)n;
}

static bool casSkipBytes(uint32_t n) {
  if (!g_casPlayFile) return false;
  uint32_t pos = g_casPlayFile.position();
  return g_casPlayFile.seek(pos + n);
}

static uint16_t casClampBaud(uint32_t baud) {
  if (baud < 300) baud = 300;
  if (baud > 6000) baud = 6000;
  return (uint16_t)baud;
}

static void casAutoResetAnalysis() {
  memset(&g_casAutoAnalysis, 0, sizeof(g_casAutoAnalysis));
  g_casAutoAnalysis.initialBaud = 600;
  g_casAutoAnalysis.turboBaud = 600;
  g_casAutoAnalysis.firstDataBaud = 600;
  g_casAutoAnalysis.maxBaud = 600;
  g_casAutoAnalysis.lastBaud = 600;
  g_casAutoAnalysis.fileBytes = 0;
  g_casAutoAnalysis.payloadBytes = 0;
  g_casAutoAnalysis.overheadBytes = 0;
  strncpy(g_casAutoAnalysis.profile, "NO_ANALIZADO", sizeof(g_casAutoAnalysis.profile) - 1);
  strncpy(g_casAutoAnalysis.confidence, "BAJA", sizeof(g_casAutoAnalysis.confidence) - 1);
  strncpy(g_casAutoAnalysis.suggestedMode, "boot", sizeof(g_casAutoAnalysis.suggestedMode) - 1);
  strncpy(g_casAutoAnalysis.chunkSummary, "sin chunks", sizeof(g_casAutoAnalysis.chunkSummary) - 1);
  strncpy(g_casAutoAnalysis.notes, "Sin analisis CAS aun", sizeof(g_casAutoAnalysis.notes) - 1);
}

static void casAutoCopy(char* dst, size_t dstSize, const char* src) {
  if (!dst || dstSize == 0) return;
  dst[0] = 0;
  if (!src) return;
  strncpy(dst, src, dstSize - 1);
  dst[dstSize - 1] = 0;
}

static void casAutoAppendNote(const char* note) {
  if (!note || !note[0]) return;
  size_t cur = strlen(g_casAutoAnalysis.notes);
  if (cur == 0 || strcmp(g_casAutoAnalysis.notes, "Sin analisis CAS aun") == 0) {
    g_casAutoAnalysis.notes[0] = 0;
    cur = 0;
  }
  if (cur > 0 && cur + 3 < sizeof(g_casAutoAnalysis.notes)) {
    strncat(g_casAutoAnalysis.notes, "; ", sizeof(g_casAutoAnalysis.notes) - strlen(g_casAutoAnalysis.notes) - 1);
  }
  strncat(g_casAutoAnalysis.notes, note, sizeof(g_casAutoAnalysis.notes) - strlen(g_casAutoAnalysis.notes) - 1);
}

static void casAutoTrackIrg(uint16_t irgMs) {
  if (irgMs == 0) return;
  CasAutoAnalysis &a = g_casAutoAnalysis;
  if (a.minIrgMs == 0 || irgMs < a.minIrgMs) a.minIrgMs = irgMs;
  if (irgMs > a.maxIrgMs) a.maxIrgMs = irgMs;
  a.lastIrgMs = irgMs;
  if (irgMs >= 10000) { a.hasLongGap = true; a.longGaps++; }
  if (irgMs >= 3000) { a.autoPauseCandidates++; }
  if (irgMs > 0 && irgMs < 80) { a.hasShortGap = true; a.shortGaps++; }
}

static void casAutoBuildChunkSummary() {
  CasAutoAnalysis &a = g_casAutoAnalysis;
  snprintf(a.chunkSummary, sizeof(a.chunkSummary),
           "chunks=%u · data=%u · baud=%u · fsk=%u · pwm=%u",
           (unsigned)a.totalChunks, (unsigned)a.dataBlocks, (unsigned)a.baudChunks,
           (unsigned)a.fskChunks, (unsigned)a.pwmChunks);
}

static bool casAutoContainsText(const uint8_t* data, uint16_t len, const char* needle) {
  if (!data || !needle || !needle[0]) return false;
  uint8_t n = (uint8_t)strlen(needle);
  if (n == 0 || len < n) return false;
  for (uint16_t i = 0; i <= (uint16_t)(len - n); i++) {
    bool ok = true;
    for (uint8_t j = 0; j < n; j++) {
      char a = (char)data[i + j];
      char b = needle[j];
      if (a >= 'a' && a <= 'z') a = (char)(a - 32);
      if (b >= 'a' && b <= 'z') b = (char)(b - 32);
      if (a != b) { ok = false; break; }
    }
    if (ok) return true;
  }
  return false;
}

static bool casAutoBaudNear(uint16_t value, uint16_t target) {
  int32_t d = (int32_t)value - (int32_t)target;
  if (d < 0) d = -d;
  return d <= 60;
}

static const char* casAutoProfileForBaud(uint16_t baud) {
  if (casAutoBaudNear(baud, 600)) return "NORMAL_600";
  if (casAutoBaudNear(baud, 800)) return "TURBO_SOFTWARE";
  if (casAutoBaudNear(baud, 1200)) return "IRON_TURBO";
  if (casAutoBaudNear(baud, 2270)) return "TURBO_2000";
  if (casAutoBaudNear(baud, 2600)) return "TURBO_2600";
  if (casAutoBaudNear(baud, 3000)) return "KSO_T2000F";
  if (casAutoBaudNear(baud, 3995)) return "INJEKTOR";
  if (casAutoBaudNear(baud, 6000)) return "INJEKTOR_6000";
  return "PERSONALIZADO";
}

static void casAutoFinalizeAnalysis() {
  CasAutoAnalysis &a = g_casAutoAnalysis;
  a.valid = true;
  if (a.firstDataBaud < 300) a.firstDataBaud = 600;
  if (a.maxBaud < 300) a.maxBaud = 600;
  a.initialBaud = a.firstDataBaud ? a.firstDataBaud : 600;
  a.turboBaud = a.maxBaud ? a.maxBaud : a.initialBaud;
  casAutoCopy(a.suggestedMode, sizeof(a.suggestedMode), "boot");
  casAutoBuildChunkSummary();
  if (a.fileBytes > 0 && a.payloadBytes == 0 && a.rawFormat) a.payloadBytes = a.fileBytes;
  a.overheadBytes = (a.fileBytes > a.payloadBytes) ? (a.fileBytes - a.payloadBytes) : 0;
  // F49Z39: se conserva autoPauseCandidates internamente, pero no se agrega nota visible en UI.

  if (a.rawFormat) {
    a.initialBaud = 600;
    a.turboBaud = 600;
    casAutoCopy(a.profile, sizeof(a.profile), "RAW_BIN");
    casAutoCopy(a.confidence, sizeof(a.confidence), "BAJA");
    casAutoAppendNote("Archivo no FUJI-CAS: se usara empaquetado RAW/BIN a 600 baud");
    return;
  }

  if (a.hasPwmChunks) {
    casAutoCopy(a.profile, sizeof(a.profile), "PWM_TURBO");
    casAutoCopy(a.confidence, sizeof(a.confidence), "MEDIA");
    casAutoAppendNote("Chunks PWM detectados: requiere soporte turbo avanzado/parcial");
    return;
  }

  // F26/F39: Turbo Software/Zybex suele venir como loader normal cercano a 600 bps
  // y luego cambia a ~800 bps (por ejemplo baud 605 -> 802). Debe evaluarse
  // antes que STAC porque los bloques normales y turbo tambien pueden comenzar
  // con 0x55 0x55. Algunos dumps agregan chunks FSK al final; esos FSK se
  // conservan como diagnostico, pero no deben reclasificar el CAS como FSK_ESPECIAL
  // si hay DATA reproducibles y baud real ~800.
  if (a.hasBaudChunks && a.dataBlocks > 0 && casAutoBaudNear(a.maxBaud, 800)) {
    casAutoCopy(a.profile, sizeof(a.profile), "TURBO_SOFTWARE_V2_800");
    casAutoCopy(a.confidence, sizeof(a.confidence), "ALTA");
    a.initialBaud = a.firstDataBaud ? a.firstDataBaud : 600;
    a.turboBaud = a.maxBaud ? a.maxBaud : 800;
    casAutoAppendNote("Turbo Software/Zybex detectado: loader ~600 y DATA ~800");
    if (a.hasFskChunks) casAutoAppendNote("FSK final tratado como diagnostico; no bloquea DATA/baud");
    if (a.stacSync55Blocks) casAutoAppendNote("0x55 0x55 observado como sincronismo de registro; no reclasifica como STAC");
    return;
  }

  // CAIN se reconoce como probable cuando aparecen bloques DATA de 217 bytes.
  if ((a.cain217Blocks >= 2) || a.textCain) {
    casAutoCopy(a.profile, sizeof(a.profile), "CAIN_217_PROBABLE");
    casAutoCopy(a.confidence, sizeof(a.confidence), (a.cain217Blocks >= 2) ? "MEDIA" : "BAJA");
    a.turboBaud = a.maxBaud ? a.maxBaud : 600;
    casAutoAppendNote("CAIN probable: bloques DATA de 217 bytes detectados");
    if (a.stacSync55Blocks) casAutoAppendNote("0x55 0x55 tambien puede aparecer como sincronismo estandar");
    return;
  }

  // F39: no usar 0x55 0x55 solo como firma STAC. En FUJI-CAS estándar cada
  // registro de cassette suele comenzar con 0x55 0x55, por lo que F38 marcaba
  // falsamente River Raid, Karateka, International Karate, Airwolf, etc. como STAC.
  // STAC se sugiere solo si hay texto STAC o si, ademas del sync, hay evidencia
  // de velocidad no estandar y no todos los bloques son registros de 132 bytes.
  bool allDataLooksStandard600 = (a.dataBlocks > 0 && a.standard132Blocks == a.dataBlocks && casAutoBaudNear(a.maxBaud, 600) && !a.hasFskChunks && !a.hasPwmChunks);
  bool stacByEvidence = false;
  if (a.textStac) stacByEvidence = true;
  else if (a.stacSync55Blocks >= 2 && !allDataLooksStandard600 && !casAutoBaudNear(a.maxBaud, 600) && !casAutoBaudNear(a.maxBaud, 800)) stacByEvidence = true;
  if (stacByEvidence) {
    casAutoCopy(a.profile, sizeof(a.profile), "STAC_PROBABLE");
    casAutoCopy(a.confidence, sizeof(a.confidence), a.textStac ? "MEDIA" : "BAJA");
    a.turboBaud = a.maxBaud ? a.maxBaud : 600;
    casAutoAppendNote("STAC probable solo por evidencia adicional; 0x55 0x55 aislado no basta");
    return;
  }

  if (a.hasFskChunks) {
    casAutoCopy(a.profile, sizeof(a.profile), "FSK_ESPECIAL");
    casAutoCopy(a.confidence, sizeof(a.confidence), "MEDIA");
    casAutoAppendNote("Chunks FSK detectados: senal no estandar o protegida");
    return;
  }

  if (a.textIron && a.usesStandardRecords && casAutoBaudNear(a.maxBaud, 600) && !a.hasFskChunks && !a.hasPwmChunks) {
    // Algunos CAS rotulados como "Iron Turbo" contienen solo registros C: estándar
    // de 132 bytes a 600 bps. Forzar 1200 aquí rompe esos archivos; se mantiene
    // el baud real del CAS y solo se protege el gap de reapertura de C:.
    casAutoCopy(a.profile, sizeof(a.profile), "IRON_STD_600");
    casAutoCopy(a.confidence, sizeof(a.confidence), "ALTA");
    a.initialBaud = 600;
    a.turboBaud = 600;
    casAutoAppendNote("IRON/TURBO textual, pero CAS real usa registros estandar 600 bps");
    casAutoAppendNote("Se conserva 600 bps y se asegura gap largo si el loader reabre C:");
    return;
  }

  if (a.textIron || (a.textTurbo && casAutoBaudNear(a.maxBaud, 1200))) {
    casAutoCopy(a.profile, sizeof(a.profile), "IRON_TURBO");
    casAutoCopy(a.confidence, sizeof(a.confidence), (a.hasBaudChunks && casAutoBaudNear(a.maxBaud, 1200)) ? "ALTA" : "MEDIA");
    a.initialBaud = 600;
    a.turboBaud = 1200;
    casAutoAppendNote("IRON_TURBO real requiere baud turbo explicito o perfil manual");
    return;
  }

  if (a.textT2000 || casAutoBaudNear(a.maxBaud, 2270)) {
    casAutoCopy(a.profile, sizeof(a.profile), "TURBO_2000");
    casAutoCopy(a.confidence, sizeof(a.confidence), a.hasBaudChunks ? "ALTA" : "MEDIA");
    a.initialBaud = a.hasBaudChunks ? a.firstDataBaud : 2270;
    a.turboBaud = 2270;
    casAutoAppendNote("Perfil Turbo 2000 sugerido");
    return;
  }

  if (a.textKso || casAutoBaudNear(a.maxBaud, 3000)) {
    casAutoCopy(a.profile, sizeof(a.profile), "KSO_T2000F");
    casAutoCopy(a.confidence, sizeof(a.confidence), a.hasBaudChunks ? "ALTA" : "MEDIA");
    a.initialBaud = a.hasBaudChunks ? a.firstDataBaud : 3000;
    a.turboBaud = 3000;
    casAutoAppendNote("Perfil KSO/Turbo 2000F sugerido");
    return;
  }

  // F29: River Raid Cold Prism / turbo 4000. El patron observado es:
  // baud 600, tres bloques de loader, baud ~3995/4000, un bloque grande y
  // muchos bloques de 257 bytes con gaps ~235 ms. Se mantiene reproduccion por
  // chunks DATA/baud reales; solo cambia perfil/diagnostico.
  if (a.textColdPrism && a.hasBaudChunks && a.dataBlocks > 0 && casAutoBaudNear(a.firstDataBaud, 600) && casAutoBaudNear(a.maxBaud, 3995)) {
    casAutoCopy(a.profile, sizeof(a.profile), "COLD_PRISM_4000");
    casAutoCopy(a.confidence, sizeof(a.confidence), "ALTA");
    a.initialBaud = a.firstDataBaud ? a.firstDataBaud : 600;
    a.turboBaud = a.maxBaud ? a.maxBaud : 3995;
    casAutoAppendNote("Cold Prism detectado: loader 600 bps y DATA turbo ~4000 bps");
    casAutoAppendNote("Se respetan baud/gaps reales del FUJI-CAS");
    return;
  }

  if (a.textInjektor || casAutoBaudNear(a.maxBaud, 6000) || casAutoBaudNear(a.maxBaud, 3995)) {
    if (a.textChaos && casAutoBaudNear(a.maxBaud, 6000)) {
      casAutoCopy(a.profile, sizeof(a.profile), "TURBO_6000_CHAOS");
      casAutoAppendNote("Perfil Turbo 6000/Chaos sugerido");
    } else {
      casAutoCopy(a.profile, sizeof(a.profile), casAutoBaudNear(a.maxBaud, 6000) ? "INJEKTOR_6000" : "INJEKTOR_4000");
      casAutoAppendNote("Perfil Injektor sugerido; 6000 bps si el CAS lo declara, 4000 bps si viene como perfil moderno");
    }
    casAutoCopy(a.confidence, sizeof(a.confidence), a.hasBaudChunks ? "ALTA" : "MEDIA");
    a.initialBaud = a.hasBaudChunks ? a.firstDataBaud : (casAutoBaudNear(a.maxBaud, 6000) ? 6000 : 3995);
    a.turboBaud = casAutoBaudNear(a.maxBaud, 6000) ? 6000 : 3995;
    return;
  }

  if (a.textChaos || casAutoBaudNear(a.maxBaud, 6000)) {
    casAutoCopy(a.profile, sizeof(a.profile), "TURBO_6000_CHAOS");
    casAutoCopy(a.confidence, sizeof(a.confidence), a.hasBaudChunks ? "ALTA" : "MEDIA");
    a.initialBaud = a.hasBaudChunks ? a.firstDataBaud : 6000;
    a.turboBaud = 6000;
    casAutoAppendNote("Perfil Turbo 6000/Chaos sugerido");
    return;
  }

  if (casAutoBaudNear(a.maxBaud, 2600)) {
    casAutoCopy(a.profile, sizeof(a.profile), "TURBO_2600");
    casAutoCopy(a.confidence, sizeof(a.confidence), "ALTA");
    a.initialBaud = a.firstDataBaud;
    a.turboBaud = 2600;
    casAutoAppendNote("Baud 2600 detectado en CAS");
    return;
  }

  if (casAutoBaudNear(a.maxBaud, 800)) {
    casAutoCopy(a.profile, sizeof(a.profile), "TURBO_SOFTWARE");
    casAutoCopy(a.confidence, sizeof(a.confidence), "ALTA");
    a.initialBaud = a.firstDataBaud;
    a.turboBaud = 800;
    casAutoAppendNote("Baud 800 detectado en CAS");
    return;
  }

  if (a.usesStandardRecords && casAutoBaudNear(a.maxBaud, 600)) {
    casAutoCopy(a.profile, sizeof(a.profile), "NORMAL_600");
    casAutoCopy(a.confidence, sizeof(a.confidence), "ALTA");
    a.initialBaud = 600;
    a.turboBaud = 600;
    casAutoAppendNote("Registros cassette estandar de 132 bytes a 600 baud");
    if (a.stacSync55Blocks) casAutoAppendNote("0x55 0x55 tratado como sincronismo normal, no como STAC");
    return;
  }

  casAutoCopy(a.profile, sizeof(a.profile), casAutoProfileForBaud(a.maxBaud));
  casAutoCopy(a.confidence, sizeof(a.confidence), a.hasBaudChunks ? "MEDIA" : "BAJA");
  casAutoAppendNote("No se pudo clasificar con certeza; usar Auto o perfil manual");
}

static bool casAnalyzeCasFile(const String& path) {
  casAutoResetAnalysis();
  if (casAutoLoadCache(path)) {
    logf("[CAS-AUTO] cache hit path=%s profile=%s chunks=%u payload=%lu", path.c_str(), g_casAutoAnalysis.profile, (unsigned)g_casAutoAnalysis.totalChunks, (unsigned long)g_casAutoAnalysis.payloadBytes);
    return true;
  }
  if (!webAtrFsReady() || path.length() == 0 || !SPIFFS.exists(path)) {
    casAutoCopy(g_casAutoAnalysis.notes, sizeof(g_casAutoAnalysis.notes), "CAS no encontrado para analisis");
    return false;
  }

  File f = SPIFFS.open(path, "r");
  if (!f) {
    casAutoCopy(g_casAutoAnalysis.notes, sizeof(g_casAutoAnalysis.notes), "No se pudo abrir CAS para analisis");
    return false;
  }
  g_casAutoAnalysis.fileBytes = (uint32_t)f.size();

  // F29: algunos dumps de River Raid "Cold Prism" vienen como FUJI-CAS con
  // loader 600 bps y luego baud ~3995/4000. La reproduccion debe respetar los
  // chunks DATA/baud; el perfil solo evita diagnosticos confusos.
  {
    String casPathLower = path;
    casPathLower.toLowerCase();
    if (casPathLower.indexOf("cold") >= 0 || casPathLower.indexOf("prism") >= 0) {
      g_casAutoAnalysis.textColdPrism = true;
    }
    if (casPathLower.indexOf("stac") >= 0) g_casAutoAnalysis.textStac = true;
    if (casPathLower.indexOf("cain") >= 0) g_casAutoAnalysis.textCain = true;
  }

  uint8_t magic[4] = {0,0,0,0};
  bool fuji = (f.read(magic, 4) == 4 && memcmp(magic, "FUJI", 4) == 0);
  f.seek(0);
  g_casAutoAnalysis.fujiFormat = fuji;
  g_casAutoAnalysis.rawFormat = !fuji;

  if (!fuji) {
    g_casAutoAnalysis.payloadBytes = g_casAutoAnalysis.fileBytes;
    f.close();
    casAutoFinalizeAnalysis();
    casAutoSaveCache(path);
    logf("[CAS-AUTO] path=%s profile=%s conf=%s initial=%u turbo=%u notes=%s",
         path.c_str(), g_casAutoAnalysis.profile, g_casAutoAnalysis.confidence,
         (unsigned)g_casAutoAnalysis.initialBaud, (unsigned)g_casAutoAnalysis.turboBaud,
         g_casAutoAnalysis.notes);
    return true;
  }

  uint16_t currentBaud = 600;
  g_casAutoAnalysis.firstDataBaud = 0;
  g_casAutoAnalysis.maxBaud = 600;
  g_casAutoAnalysis.lastBaud = 600;
  uint16_t chunks = 0;
  while (f.available() > 0 && chunks < 512) {
    chunks++;
    g_casAutoAnalysis.totalChunks = chunks;
    uint8_t h[8];
    if (f.read(h, sizeof(h)) != (int)sizeof(h)) break;
    char typ[5] = {(char)h[0], (char)h[1], (char)h[2], (char)h[3], 0};
    uint16_t chunkLen = (uint16_t)h[4] | ((uint16_t)h[5] << 8);
    uint16_t aux = (uint16_t)h[6] | ((uint16_t)h[7] << 8);
    uint32_t remain = (uint32_t)f.available();
    if (!casIsKnownChunkType(typ) || chunkLen > remain) {
      casAutoAppendNote("Header CAS invalido durante analisis");
      break;
    }

    if (memcmp(typ, "baud", 4) == 0) {
      if (aux >= 300 && aux <= 6000) {
        currentBaud = aux;
        g_casAutoAnalysis.hasBaudChunks = true;
        g_casAutoAnalysis.baudChunks++;
        g_casAutoAnalysis.lastBaud = aux;
        if (aux > g_casAutoAnalysis.maxBaud) g_casAutoAnalysis.maxBaud = aux;
      }
      if (chunkLen) f.seek(f.position() + chunkLen);
      continue;
    }

    if (memcmp(typ, "data", 4) == 0) {
      g_casAutoAnalysis.dataBlocks++;
      g_casAutoAnalysis.payloadBytes += chunkLen;
      if (!g_casAutoAnalysis.firstDataBaud) g_casAutoAnalysis.firstDataBaud = currentBaud;
      if (currentBaud > g_casAutoAnalysis.maxBaud) g_casAutoAnalysis.maxBaud = currentBaud;
      if (chunkLen >= 128 && chunkLen <= 132) {
        g_casAutoAnalysis.standard132Blocks++;
        g_casAutoAnalysis.usesStandardRecords = true;
      }
      if (chunkLen == 217) g_casAutoAnalysis.cain217Blocks++;
      casAutoTrackIrg(aux);

      uint16_t scanLen = chunkLen;
      if (scanLen > 256) scanLen = 256;
      uint8_t scan[256];
      if (scanLen && f.read(scan, scanLen) == (int)scanLen) {
        if (scanLen >= 2 && scan[0] == 0x55 && scan[1] == 0x55) g_casAutoAnalysis.stacSync55Blocks++;
        if (casAutoContainsText(scan, scanLen, "IRON")) g_casAutoAnalysis.textIron = true;
        if (casAutoContainsText(scan, scanLen, "TURBO")) g_casAutoAnalysis.textTurbo = true;
        if (casAutoContainsText(scan, scanLen, "T2000") || casAutoContainsText(scan, scanLen, "TURBO 2000")) g_casAutoAnalysis.textT2000 = true;
        if (casAutoContainsText(scan, scanLen, "KSO")) g_casAutoAnalysis.textKso = true;
        if (casAutoContainsText(scan, scanLen, "CHAOS")) g_casAutoAnalysis.textChaos = true;
        if (casAutoContainsText(scan, scanLen, "INJEKTOR")) g_casAutoAnalysis.textInjektor = true;
        if (casAutoContainsText(scan, scanLen, "STAC")) g_casAutoAnalysis.textStac = true;
        if (casAutoContainsText(scan, scanLen, "CAIN")) g_casAutoAnalysis.textCain = true;
      }
      if (chunkLen > scanLen) f.seek(f.position() + (chunkLen - scanLen));
      continue;
    }

    if (memcmp(typ, "fsk ", 4) == 0) {
      g_casAutoAnalysis.hasFskChunks = true;
      g_casAutoAnalysis.fskChunks++;
      casAutoTrackIrg(aux);
      if (chunkLen) f.seek(f.position() + chunkLen);
      continue;
    }

    if (memcmp(typ, "pwms", 4) == 0 || memcmp(typ, "pwmc", 4) == 0 || memcmp(typ, "pwmd", 4) == 0 || memcmp(typ, "pwml", 4) == 0) {
      g_casAutoAnalysis.hasPwmChunks = true;
      g_casAutoAnalysis.pwmChunks++;
      if (memcmp(typ, "pwms", 4) == 0) g_casAutoAnalysis.pwmStateChunks++;
      else if (memcmp(typ, "pwmc", 4) == 0) g_casAutoAnalysis.pwmControlChunks++;
      else if (memcmp(typ, "pwmd", 4) == 0) g_casAutoAnalysis.pwmDataChunks++;
      else if (memcmp(typ, "pwml", 4) == 0) g_casAutoAnalysis.pwmLongChunks++;
      if (chunkLen) f.seek(f.position() + chunkLen);
      continue;
    }

    if (chunkLen) f.seek(f.position() + chunkLen);
  }

  f.close();
  if (!g_casAutoAnalysis.firstDataBaud) g_casAutoAnalysis.firstDataBaud = 600;
  // F49Z39: hasLongGap queda disponible en JSON/diagnostico, sin nota textual larga.
  if (g_casAutoAnalysis.hasShortGap) casAutoAppendNote("Gaps muy cortos detectados; revisar si falla entre bloques");
  casAutoFinalizeAnalysis();
  casAutoSaveCache(path);
  logf("[CAS-AUTO] path=%s profile=%s conf=%s initial=%u turbo=%u chunks=%u blocks=%u baudChunks=%u pwm=%u fsk=%u notes=%s",
       path.c_str(), g_casAutoAnalysis.profile, g_casAutoAnalysis.confidence,
       (unsigned)g_casAutoAnalysis.initialBaud, (unsigned)g_casAutoAnalysis.turboBaud,
       (unsigned)g_casAutoAnalysis.totalChunks,
       (unsigned)g_casAutoAnalysis.dataBlocks, (unsigned)g_casAutoAnalysis.baudChunks,
       (unsigned)g_casAutoAnalysis.pwmChunks, (unsigned)g_casAutoAnalysis.fskChunks,
       g_casAutoAnalysis.notes);
  return true;
}

static uint8_t casAtariChecksum(const uint8_t* data, uint16_t len) {
  // Checksum SIO/Atari: suma con carry end-around en 8 bits.
  // Usado aquí para generar registros RAW/BIN al estilo SDrive-MAX.
  uint16_t sum = 0;
  for (uint16_t i = 0; i < len; i++) {
    sum += data[i];
    if (sum > 0xFF) sum = (sum & 0xFF) + 1;
  }
  return (uint8_t)(sum & 0xFF);
}

static bool casBuildNextRawSdriveRecord() {
  if (!g_casPlayFile) return false;
  memset(g_casRawRecord, 0, sizeof(g_casRawRecord));
  g_casRawRecord[0] = 0x55;
  g_casRawRecord[1] = 0x55;

  if (g_casRawEndRecordQueued || g_casPlayFile.position() >= g_casPlaySize) {
    g_casRawRecord[2] = 0xFE; // end record
    g_casRawRecord[131] = casAtariChecksum(g_casRawRecord, 131);
    g_casRawRecordLen = 132;
    g_casRawRecordPos = 0;
    g_casRawEndRecordQueued = false;
    g_casRawLastMarker = 0xFE;
    g_casRawRecordsBuilt++;
    return true;
  }

  uint8_t data[128];
  memset(data, 0, sizeof(data));
  int r = g_casPlayFile.read(data, sizeof(data));
  if (r < 0) r = 0;

  if (r >= 128) {
    g_casRawRecord[2] = 0xFC; // full record
  } else {
    g_casRawRecord[2] = 0xFA; // partial record
    g_casRawRecord[130] = (uint8_t)r;
    g_casRawEndRecordQueued = true;
  }
  memcpy(g_casRawRecord + 3, data, 128);
  // En registro parcial, SDrive-MAX deja el tamaño real en el último byte de datos.
  if (r < 128) g_casRawRecord[130] = (uint8_t)r;
  g_casRawRecord[131] = casAtariChecksum(g_casRawRecord, 131);
  g_casRawRecordLen = 132;
  g_casRawRecordPos = 0;
  g_casRawLastMarker = g_casRawRecord[2];
  g_casRawRecordsBuilt++;
  return true;
}

static uint16_t casEffectiveBaud(uint16_t sourceBaud) {
  if (g_casProfileLockBaud && g_casProfileForcedBaud > 0) {
    g_casAspeqtEffectiveBaud = casClampBaud(g_casProfileForcedBaud);
    return g_casAspeqtEffectiveBaud;
  }

  uint32_t b = sourceBaud ? sourceBaud : 600;
  if (g_casUserBaseBaud > b) b = g_casUserBaseBaud;
  if (g_casTurboEnabled) b = (b * (uint32_t)g_casTurboMultiplierX100) / 100UL;

  // F42: NORMAL_600/IRON_STD_600 bloquean el baud para que chunks baud
  // posteriores no suban la velocidad por accidente.
  g_casAspeqtEffectiveBaud = casClampBaud(b);
  return g_casAspeqtEffectiveBaud;
}

static uint16_t casTransportFrameLimit() {
  // F49Z4: modo AspeQt removido; transporte seguro estándar con ACK.
  return CAS_PLAY_CHUNK_BYTES;
}

static void casScanDataForKnownLoaders(const uint8_t* data, uint16_t len) {
  if (!data || len == 0) return;
  for (uint16_t i = 0; i < len; i++) {
    char c = (char)data[i];
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    // Deteccion simple y no invasiva. Sirve solo para diagnostico/UI.
    if (i + 3 < len) {
      char a[5];
      for (uint8_t j = 0; j < 4; j++) {
        char x = (char)data[i + j];
        if (x >= 'a' && x <= 'z') x = (char)(x - 32);
        a[j] = x;
      }
      a[4] = 0;
      if (memcmp(a, "IRON", 4) == 0) g_casIronTurboDetected = true;
    }
    if (i + 4 < len) {
      char a[6];
      for (uint8_t j = 0; j < 5; j++) {
        char x = (char)data[i + j];
        if (x >= 'a' && x <= 'z') x = (char)(x - 32);
        a[j] = x;
      }
      a[5] = 0;
      if (memcmp(a, "TURBO", 5) == 0) g_casIronTurboDetected = true;
    }
  }
}

static void casStartPlaybackState(bool rawMode, uint32_t fileSize, bool bootMode) {
  g_casRawMode = rawMode;
  g_casBootModeActive = bootMode;
  g_casPlaying = true;
  g_casPaused = false;
  g_casEof = false;
  g_casCurrentBaud = casEffectiveBaud(g_casUserBaseBaud);
  g_casPlayStartedMs = millis();
  g_casDelayUntilMs = g_casInitialDelaySec ? (millis() + ((uint32_t)g_casInitialDelaySec * 1000UL)) : 0;
  g_casPlaySize = fileSize;
  g_casPlayPos = g_casParserStartOffset;
  g_casBytesQueued = 0;
  g_casChunksQueued = 0;
  g_casDataBlocks = 0;
  g_casSkippedChunks = 0;
  g_casFskChunks = 0;
  g_casLastSendMs = 0;
  g_casInDataChunk = false;
  g_casChunkRemain = 0;
  g_casChunkGapMs = 0;
  g_casChunkGapSent = false;
  g_casPendingBlockDelay = false;
  g_casPendingBlockDelayMs = 0;
  g_casRecordDrainPending = false;
  g_casLastIrgAppliedMs = 0;
  g_casTimingWaitUntilMs = 0;
  g_casDrainWaitStartMs = 0;
  g_casDrainTargetBytes = 0;
  g_casDrainDeadlineMs = 0;
  g_casRecordExpectedMs = 0;
  g_casTransportTargetRx = 0;
  g_casPostRecordSettleUntilMs = 0;
  g_casRecordsCompleted = 0;
  g_casRecordsStarted = 0;
  g_casBootFirstIrgSkipped = false;
  g_casSdriveExactMode = bootMode;
  g_casIronTurboDetected = false;
  g_casIronTurboGapFixes = 0;
  g_casPrevCompletedRecordMarker = 0;
  g_casSdriveBlockExactMode = true; // F49Z5: registro lógico completo reconstruido en RP2040 antes de reproducir.
  g_casSdriveEndGapPending = false;
  g_casSdriveEndGapMs = 0;
  g_casSdriveEndMarkers = 0;
  g_casCurrentRecordMarker = 0;
  g_casCurrentRecordLen = 0;
  g_casSdriveRawRecordMode = rawMode;
  g_casRawRecordLen = 0;
  g_casRawRecordPos = 0;
  g_casRawEndRecordQueued = false;
  g_casRawRecordsBuilt = 0;
  g_casRawLastMarker = 0;
  g_casRpRecordBusySkips = 0;
  g_casShortIrgMs = bootMode ? CAS_IRG_BOOT_SHORT_MS : CAS_IRG_CLOAD_SHORT_MS;
  g_casTimingSkips = 0;
  g_casLastChunkType[0] = 0;
  g_casLastChunkLen = 0;
  g_casLastChunkAux = 0;
  g_casParserErrors = 0;
  g_casSdriveStreamMode = false;
  g_casRpBufferFree = 4096;
  g_casEmbeddedBaud = g_casCurrentBaud;
  g_casAspeqtEffectiveBaud = g_casCurrentBaud;
  g_casAspeqtCompat = false;
  g_casAspeqtFragmentBytes = CAS_PLAY_CHUNK_BYTES;
  g_casAspeqtFragments = 0;
  g_casTxSeq = 0;
  g_casAckSeq = 0;
  g_casAckOk = false;
  g_casAckBytesRx = 0;
  g_casPlayCount++;
}

static bool casWaitCasAck(uint16_t seq, uint32_t timeoutMs) {
  uint32_t start = millis();
  g_casAckSeq = 0;
  g_casAckOk = false;
  g_casAckBytesRx = 0;
  while ((uint32_t)(millis() - start) < timeoutMs) {
    pollUartFromRP();
    if (g_casAckSeq == seq) {
      g_casAckWaitMsLast = (uint32_t)(millis() - start);
      return g_casAckOk;
    }
    delay(1);
    yield();
  }
  g_casAckWaitMsLast = timeoutMs;
  return false;
}

static bool casSendDataToRP(const uint8_t* data, uint16_t len, uint16_t baud, uint16_t gapMs, uint8_t flags) {
  if (!data || len == 0) return false;
  if (len > CAS_PLAY_CHUNK_BYTES) len = CAS_PLAY_CHUNK_BYTES;

  uint16_t seq = ++g_casTxSeq;
  if (seq == 0) seq = ++g_casTxSeq;

  uint8_t payload[1 + 2 + 2 + 2 + 1 + CAS_PLAY_CHUNK_BYTES];
  payload[0] = TYPE_CAS_DATA;
  payload[1] = (uint8_t)(seq & 0xFF);
  payload[2] = (uint8_t)(seq >> 8);
  payload[3] = (uint8_t)(baud & 0xFF);
  payload[4] = (uint8_t)(baud >> 8);
  payload[5] = (uint8_t)(gapMs & 0xFF);
  payload[6] = (uint8_t)(gapMs >> 8);
  payload[7] = flags;
  memcpy(payload + 8, data, len);

  bool ok = false;
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    sendUartFrameToRP(payload, (uint8_t)(8 + len));
    g_casLastSendMs = millis();
    if (casWaitCasAck(seq, 2200UL)) { ok = true; break; }
    g_casAckRetries++;
  }

  if (!ok) {
    g_casAckTimeouts++;
    g_casTransportTimeouts++;
    snprintf(g_casLastError, sizeof(g_casLastError), "ACK CAS timeout seq=%u len=%u", (unsigned)seq, (unsigned)len);
    return false;
  }

  if (!g_casAckOk) {
    g_casAckNak++;
    snprintf(g_casLastError, sizeof(g_casLastError), "ACK CAS NAK seq=%u len=%u", (unsigned)seq, (unsigned)len);
    return false;
  }

  g_casBytesQueued += len;
  g_casTransportTargetRx = g_casBytesQueued;
  g_casChunksQueued++;
  // F49Z4: AspeQt removido; no se contabilizan fragmentos AspeQt.
  g_casLastSendMs = millis();
  return true;
}


static uint32_t casEstimateRecordMs(uint16_t gapMs, uint16_t recordBytes, uint16_t baud) {
  // UART 8N1: 1 start + 8 data + 1 stop = 10 bits/byte.
  uint32_t b = baud ? baud : 600;
  uint32_t serialMs = ((uint32_t)recordBytes * 10UL * 1000UL + b - 1UL) / b;
  // Margen amplio porque el RP2040 reporta estado cada ~200 ms y el Atari puede pausar IRQs.
  return (uint32_t)gapMs + serialMs + 5000UL;
}

static bool casIsKnownChunkType(const char* typ) {
  if (!typ) return false;
  return memcmp(typ, "FUJI", 4) == 0 ||
         memcmp(typ, "baud", 4) == 0 ||
         memcmp(typ, "data", 4) == 0 ||
         memcmp(typ, "fsk ", 4) == 0 ||
         memcmp(typ, "pwms", 4) == 0 ||
         memcmp(typ, "pwmc", 4) == 0 ||
         memcmp(typ, "pwmd", 4) == 0 ||
         memcmp(typ, "pwml", 4) == 0;
}

static bool casTryResyncToNextChunk() {
  if (!g_casPlayFile) return false;
  uint32_t start = g_casPlayFile.position();
  uint32_t sz = (uint32_t)g_casPlayFile.size();
  if (start >= sz) return false;

  uint32_t scanLimit = sz - start;
  if (scanLimit > 4096) scanLimit = 4096;

  uint8_t win[4] = {0, 0, 0, 0};
  for (uint32_t i = 0; i < scanLimit && g_casPlayFile.available() > 0; i++) {
    int c = g_casPlayFile.read();
    if (c < 0) break;
    win[0] = win[1]; win[1] = win[2]; win[2] = win[3]; win[3] = (uint8_t)c;
    char typ[5] = {(char)win[0], (char)win[1], (char)win[2], (char)win[3], 0};
    if (casIsKnownChunkType(typ)) {
      uint32_t pos = g_casPlayFile.position() - 4;
      g_casPlayFile.seek(pos);
      casSetLastError(String("CAS resincronizado en chunk ") + typ + " pos=" + String(pos));
      return true;
    }
  }

  g_casPlayFile.seek(start);
  return false;
}

static bool casPrepareNextDataChunk() {
  if (!g_casPlayFile || g_casEof) return false;

  while (g_casPlayFile && g_casPlayFile.available() > 0) {
    uint32_t headerPos = g_casPlayFile.position();
    uint8_t h[8];
    if (!casReadExact(h, sizeof(h))) { g_casEof = true; return false; }
    g_casPlayPos = g_casPlayFile.position();
    char typ[5];
    typ[0] = (char)h[0]; typ[1] = (char)h[1]; typ[2] = (char)h[2]; typ[3] = (char)h[3]; typ[4] = 0;
    uint16_t chunkLen = (uint16_t)h[4] | ((uint16_t)h[5] << 8);
    uint16_t aux = (uint16_t)h[6] | ((uint16_t)h[7] << 8);
    strncpy(g_casLastChunkType, typ, sizeof(g_casLastChunkType) - 1);
    g_casLastChunkType[sizeof(g_casLastChunkType) - 1] = 0;
    g_casLastChunkLen = chunkLen;
    g_casLastChunkAux = aux;

    uint32_t remainAfterHeader = (uint32_t)g_casPlayFile.available();
    if (!casIsKnownChunkType(typ) || chunkLen > remainAfterHeader) {
      g_casParserErrors++;
      String err = String("Chunk CAS invalido/desalineado: ") + typ + " pos=" + String(headerPos) + " len=" + String(chunkLen) + " remain=" + String(remainAfterHeader);
      casSetLastError(err);
      if (casTryResyncToNextChunk()) continue;
      g_casEof = true;
      return false;
    }

    if (memcmp(typ, "baud", 4) == 0) {
      if (aux >= 300 && aux <= 6000) {
        g_casEmbeddedBaud = aux;
        g_casCurrentBaud = casEffectiveBaud(aux);
      }
      if (chunkLen) casSkipBytes(chunkLen);
      continue;
    }
    if (memcmp(typ, "data", 4) == 0) {
      if (chunkLen == 0) continue;
      g_casInDataChunk = true;
      g_casChunkRemain = chunkLen;
      g_casCurrentRecordLen = chunkLen;
      uint16_t effectiveGapMs = aux;
      // F49Z69: si un registro anterior terminó con 0xFE (EOF cassette)
      // y aun viene otro bloque DATA, tratamos el archivo como multi-etapa.
      // El Atari normalmente reabre C: y necesita leader/gap largo antes del
      // siguiente bloque. Esto cubre Blue Max / Iron Turbo 600 y también otros
      // CAS estándar que no traen el gap suficientemente largo.
      // No se fuerza baud ni se altera el template: solo se corrige el IRG corto.
      if (chunkLen >= 128 && chunkLen <= 132 &&
          g_casPrevCompletedRecordMarker == 0xFE &&
          effectiveGapMs < CAS_IRON_REOPEN_GAP_MS) {
        effectiveGapMs = CAS_IRON_REOPEN_GAP_MS;
        g_casIronTurboGapFixes++;
      }
      g_casChunkGapMs = effectiveGapMs;
      // F49Q: cada chunk data es un registro cassette. Antes de enviarlo,
      // aplicamos IRG compatible y luego enviamos el registro completo por partes.
      g_casChunkGapSent = false;
      g_casPendingBlockDelay = true;
      g_casPendingBlockDelayMs = effectiveGapMs;
      g_casDataBlocks++;
      g_casRecordsStarted++;
      return true;
    }
    if (memcmp(typ, "fsk ", 4) == 0 || memcmp(typ, "pwms", 4) == 0 || memcmp(typ, "pwmc", 4) == 0 || memcmp(typ, "pwmd", 4) == 0 || memcmp(typ, "pwml", 4) == 0) {
      g_casFskChunks++;
      g_casSkippedChunks++;
      casSkipBytes(chunkLen);
      continue;
    }
    // FUJI/descripciones: es un chunk completo, no una firma suelta de 4 bytes.
    g_casSkippedChunks++;
    casSkipBytes(chunkLen);
  }
  g_casEof = true;
  return false;
}

void serviceCasManualPlayback() {
  if (!g_casPlaying || g_casPaused || g_casEof) return;
  if (!g_casPlayFile) { g_casPlaying = false; return; }

  uint32_t now = millis();

  // Delay inicial configurado desde la UI. Sirve para alcanzar a presionar RETURN/START.
  if (g_casDelayUntilMs && (int32_t)(now - g_casDelayUntilMs) < 0) {
    g_casTimingSkips++;
    return;
  }
  g_casDelayUntilMs = 0;

  // F49X: transporte seguro por confirmación. Si enviamos un fragmento al RP2040,
  // esperamos a que el status confirme rpBytesRx >= bytesQueued antes de enviar más.
  // Esto evita que se adelanten fragmentos/registros cuando hay IRG largos.
  if (g_casTransportTargetRx > 0 && g_casRpBytesRx < g_casTransportTargetRx) {
    uint32_t age = (uint32_t)(now - g_casLastSendMs);
    if (age < 2500UL) {
      g_casTransportWaitSkips++;
      return;
    }
    // No abortar: si un status se perdió, dejamos que el drenaje del registro decida.
    g_casTransportTimeouts++;
    g_casTransportTargetRx = 0;
  }
  if (g_casTransportTargetRx > 0 && g_casRpBytesRx >= g_casTransportTargetRx) {
    g_casTransportTargetRx = 0;
  }

  // F49X: entre registros cassette, esperar que el RP2040 haya recibido y
  // consumido todos los bytes enviados. El timeout ya no es fijo; se calcula
  // con IRG real + tiempo serial del registro + margen, como flujo tipo SDrive.
  if (g_casRecordDrainPending) {
    if (!g_casDrainWaitStartMs) g_casDrainWaitStartMs = now;
    bool rxAll = (g_casDrainTargetBytes == 0) || (g_casRpBytesRx >= g_casDrainTargetBytes);
    bool playedAll = (g_casDrainTargetBytes == 0) || (g_casRpBytesPlayed >= g_casDrainTargetBytes);
    bool rpRecordBusy = (g_casRpFlags & 0x10) != 0; // RP2040 aún vacía/flush del registro.
    uint32_t elapsedDrain = (uint32_t)(now - g_casDrainWaitStartMs);

    // F49Z6: modo estricto. Con el buffer de registro completo, el siguiente
    // registro NO debe enviarse hasta que el RP2040 informe que terminó el
    // registro anterior. El deadline dinamico queda solo como diagnostico;
    // no libera el flujo si el RP sigue ocupado.
    if (!rxAll || !playedAll || rpRecordBusy) {
      g_casTimingSkips++;
      g_casDrainHoldMs = elapsedDrain;
      if (rpRecordBusy) g_casRpRecordBusySkips++;
      if (elapsedDrain < CAS_STRICT_DRAIN_HARD_MS) {
        return;
      }
      // Recuperacion de emergencia: evita quedar bloqueado para siempre si el
      // RP2040 deja de reportar estado. Esto debería ser raro y queda medido.
      g_casDrainHardTimeouts++;
      g_casDrainTimeouts++;
    }

    g_casDrainHoldMs = 0;
    g_casRecordDrainPending = false;
    g_casDrainWaitStartMs = 0;
    g_casDrainDeadlineMs = 0;
    g_casRecordExpectedMs = 0;
    uint32_t settle = CAS_UART_SETTLE_MS;
    if (g_casSdriveEndGapPending && g_casSdriveEndGapMs > 0) settle += g_casSdriveEndGapMs;
    g_casPostRecordSettleUntilMs = now + settle;
    g_casSdriveEndGapPending = false;
    g_casSdriveEndGapMs = 0;
    g_casRecordsCompleted++;
  }

  if (g_casPostRecordSettleUntilMs && (int32_t)(now - g_casPostRecordSettleUntilMs) < 0) {
    g_casTimingSkips++;
    return;
  }
  g_casPostRecordSettleUntilMs = 0;

  // IRG/leader antes del próximo registro. En modo SDrive/FUJI se manda
  // al RP2040 como gap real del chunk data; no se espera aquí en el ESP32.
  if (g_casTimingWaitUntilMs && (int32_t)(now - g_casTimingWaitUntilMs) < 0) {
    g_casTimingSkips++;
    return;
  }
  g_casTimingWaitUntilMs = 0;

  if ((now - g_casLastSendMs) < 8) return;
  if (g_casRpBufferFree < CAS_PLAY_CHUNK_BYTES) return;

  uint8_t buf[CAS_PLAY_CHUNK_BYTES];
  uint16_t n = 0;
  uint16_t gapMs = 0;
  uint8_t flags = 0;
  bool recordEndedAfterThisSend = false;

  if (g_casRawMode) {
    // F49Y: RAW/BIN compatible con SDrive-MAX. En vez de mandar el archivo
    // crudo, generamos registros cassette Atari de 132 bytes:
    // 55 55 + marcador FC/FA/FE + 128 bytes + checksum.
    if (g_casRawRecordPos >= g_casRawRecordLen) {
      if (!casBuildNextRawSdriveRecord()) {
        g_casEof = true;
        casSendControlToRP(4, g_casCurrentBaud, g_casBytesQueued);
        return;
      }
      g_casDataBlocks++;
      g_casRecordsStarted++;
      g_casCurrentRecordMarker = g_casRawLastMarker;
      g_casCurrentRecordLen = g_casRawRecordLen;
      g_casPendingBlockDelay = true;
      g_casPendingBlockDelayMs = (g_casRawRecordsBuilt <= 1) ? 0 : 300;
      g_casChunkGapSent = false;
    }

    uint16_t remain = (uint16_t)(g_casRawRecordLen - g_casRawRecordPos);
    uint16_t frameLimit = casTransportFrameLimit();
    n = remain > frameLimit ? frameLimit : remain;
    memcpy(buf, g_casRawRecord + g_casRawRecordPos, n);
    g_casRawRecordPos += n;
    recordEndedAfterThisSend = (g_casRawRecordPos >= g_casRawRecordLen);
    if (recordEndedAfterThisSend && g_casRawLastMarker == 0xFE) {
      g_casEof = true;
    }
  } else {
    if (!g_casInDataChunk && !casPrepareNextDataChunk()) {
      g_casEof = true;
      casSendControlToRP(4, g_casCurrentBaud, g_casBytesQueued);
      return;
    }

    // F49U estilo SDrive-MAX:
    // El Master no espera el IRG con millis(). Envia el gap al RP2040 en el
    // primer fragmento del registro, para que el RP mantenga la linea idle,
    // espere el IRG y despues transmita el registro completo por UART.
    // Esto imita mejor a SDrive-MAX: gap -> bloque data, y evita jitter web/ESP32.

    uint16_t want = g_casChunkRemain;
    uint16_t frameLimit = casTransportFrameLimit();
    if (want > frameLimit) want = frameLimit;
    n = (uint16_t)g_casPlayFile.read(buf, want);
    if (n == 0) { g_casEof = true; casSendControlToRP(4, g_casCurrentBaud, g_casBytesQueued); return; }

    // F49X: en FUJI/data el registro normalmente mide 132 bytes.
    // Lo mantenemos como registro logico, pero lo transportamos en frames pequenos
    // ESP32->RP2040 para no perder datos en la UART interna. El flag 0x02 solo
    // se envia en el ultimo fragmento, por lo que el RP2040 hace flush de registro
    // al final igual que en el modelo SDrive.
    if (!g_casChunkGapSent && n >= 3) g_casCurrentRecordMarker = buf[2];
    casScanDataForKnownLoaders(buf, n);

    if (n >= g_casChunkRemain) {
      g_casChunkRemain = 0;
      g_casInDataChunk = false;
      // Acabamos un registro: después de enviar este último fragmento,
      // fijamos el objetivo de bytes para que el RP2040 lo consuma completo.
      recordEndedAfterThisSend = true;
    } else {
      g_casChunkRemain -= n;
    }
  }

  // F49U: flags de registro hacia RP2040.
  // 0x01 = aplicar gap antes del primer byte del registro.
  // 0x02 = este fragmento termina el registro; RP2040 hace flush real del UART.
  if (g_casPendingBlockDelay) {
    gapMs = g_casPendingBlockDelayMs;
    if (!g_casBootModeActive) {
      // Para CLOAD BASIC dejamos un minimo razonable cuando el CAS trae 0 o gaps enormes.
      if (gapMs == 0 || gapMs > 1000) gapMs = g_casShortIrgMs;
    }
    if (gapMs > CAS_IRG_MAX_MS) gapMs = CAS_IRG_MAX_MS;
    flags |= 0x01;
    g_casLastIrgAppliedMs = gapMs;
    g_casPendingBlockDelay = false;
    g_casChunkGapSent = true;
  }
  if (recordEndedAfterThisSend) flags |= 0x02;

  if (recordEndedAfterThisSend && g_casCurrentRecordMarker == 0xFE) {
    // SDrive-MAX no detiene la cinta al encontrar 0xFE dentro de FUJI/data:
    // muchos loaders multi-etapa se reinician o vuelven a abrir C:. Agregamos
    // un pequeño gap de seguridad y dejamos correr. Para RAW/BIN, 0xFE sí marca
    // fin real y se manda EOF al RP después de transmitirlo.
    g_casSdriveEndGapPending = true;
    g_casSdriveEndGapMs = 200;
    g_casSdriveEndMarkers++;
  }

  if (casSendDataToRP(buf, n, g_casCurrentBaud, gapMs, flags) && recordEndedAfterThisSend) {
    g_casPrevCompletedRecordMarker = g_casCurrentRecordMarker;
    g_casRecordDrainPending = true;
    g_casDrainWaitStartMs = 0;
    g_casDrainTargetBytes = g_casBytesQueued;
    g_casRecordExpectedMs = casEstimateRecordMs(gapMs, g_casCurrentRecordLen ? g_casCurrentRecordLen : n, g_casCurrentBaud);
    g_casDrainDeadlineMs = millis() + g_casRecordExpectedMs;
    if (g_casRawMode && g_casEof) {
      casSendControlToRP(4, g_casCurrentBaud, g_casBytesQueued);
    }
  }
  g_casPlayPos = g_casPlayFile.position();
}

void handleCasStatusFromRP(const uint8_t* p, uint8_t len) {
  if (!p || len < 16) return;
  g_casRpState = p[0];
  g_casRpBufferUsed = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
  g_casRpBufferFree = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
  g_casRpBytesPlayed = prnReadU32LE(p + 5);
  g_casRpBytesRx = prnReadU32LE(p + 9);
  g_casRpBaud = (uint16_t)p[13] | ((uint16_t)p[14] << 8);
  g_casRpFlags = p[15];
  g_casRpLastStatusMs = millis();
  if (g_casEof && g_casRpState == 0) {
    g_casPlaying = false;
    casClosePlaybackFile();
  }
}

void handleCasAckFromRP(const uint8_t* p, uint8_t len) {
  if (!p || len < 10) return;
  g_casAckSeq = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  g_casAckOk = (p[2] != 0);
  g_casAckBytesRx = prnReadU32LE(p + 3);
  g_casRpBytesRx = g_casAckBytesRx;
  g_casRpBufferFree = (uint16_t)p[7] | ((uint16_t)p[8] << 8);
  g_casRpFlags = p[9];
  g_casRpLastStatusMs = millis();
}

void handleCasPlay() {
  if (!g_casMounted || g_casMountedName.length() == 0) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"No hay CAS montado en C:\"}");
    return;
  }
  String path = webCasPathForName(g_casMountedName);
  if (!webAtrFsReady() || !SPIFFS.exists(path)) {
    casSetLastError(String("CAS no encontrado: ") + g_casMountedName);
    server.send(404, "application/json", "{\"ok\":false,\"error\":\"CAS no encontrado\"}");
    return;
  }

  casClosePlaybackFile();
  g_casPlayFile = SPIFFS.open(path, "r");
  if (!g_casPlayFile) {
    casSetLastError("No se pudo abrir CAS");
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"No se pudo abrir CAS\"}");
    return;
  }

  uint32_t sz = (uint32_t)g_casPlayFile.size();
  String reqMode = server.hasArg("mode") ? server.arg("mode") : String("boot");
  reqMode.toLowerCase();
  if (!(reqMode == "cload" || reqMode == "boot" || reqMode == "auto")) reqMode = "boot";
  strncpy(g_casLoadMode, reqMode.c_str(), sizeof(g_casLoadMode) - 1);
  g_casLoadMode[sizeof(g_casLoadMode) - 1] = '\0';
  bool bootMode = (reqMode == "boot") || (reqMode == "auto");
  g_casSdriveStreamMode = false;
  g_casSdriveExactMode = bootMode; // F49U: FUJI/data con gap aplicado por RP2040, estilo SDrive-MAX.

  // F42: resolver perfil efectivo antes del Play. Prioridad:
  // override manual -> cache de analisis -> AUTO. Si no hay cache, se analiza una vez.
  String overrideProfile;
  bool hasProfileOverride = casProfileOverrideGet(g_casMountedName, overrideProfile);
  CasAutoAnalysis cachedCas;
  bool casCached = casAutoLoadCacheForPath(path, cachedCas);
  if (casCached) {
    memcpy(&g_casAutoAnalysis, &cachedCas, sizeof(CasAutoAnalysis));
    g_casAutoAnalysis.valid = true;
  }
  if (!casCached && !g_casAutoAnalysis.valid) {
    casAnalyzeCasFile(path);
  }
  String effectiveProfile = hasProfileOverride ? overrideProfile : (g_casAutoAnalysis.valid ? String(g_casAutoAnalysis.profile) : String("AUTO"));
  bool profileDriven = effectiveProfile.length() && effectiveProfile != "AUTO" && effectiveProfile != "NO_ANALIZADO";
  bool autoSpeed = profileDriven || (reqMode == "auto") || (server.hasArg("auto") && server.arg("auto").toInt() != 0);

  g_casIronTurboCompat = false;
  g_casIronTurboTurboBaud = 600;
  g_casAspeqtCompat = false;
  if (autoSpeed) {
    casApplyEffectiveProfileForPlayback(effectiveProfile, hasProfileOverride);
    if (effectiveProfile == "AUTO" || effectiveProfile == "NO_ANALIZADO") {
      uint16_t autoBaud = g_casAutoAnalysis.initialBaud ? g_casAutoAnalysis.initialBaud : 600;
      g_casUserBaseBaud = casClampBaud(autoBaud);
      casSetEffectiveProfileRuntime(effectiveProfile, false);
    }
    if ((strncmp(g_casAutoAnalysis.profile, "IRON", 4) == 0) || g_casAutoAnalysis.textIron || effectiveProfile == "IRON_STD_600") {
      g_casIronTurboCompat = true;
      g_casIronTurboTurboBaud = g_casAutoAnalysis.turboBaud ? casClampBaud(g_casAutoAnalysis.turboBaud) : 600;
    }
    logf("[CAS-F42] Play profile=%s manual=%u conf=%s base=%u lock=%u forced=%u turbo=%u",
         g_casEffectiveProfile, (unsigned)g_casEffectiveProfileManual, g_casAutoAnalysis.confidence,
         (unsigned)g_casUserBaseBaud, (unsigned)g_casProfileLockBaud, (unsigned)g_casProfileForcedBaud,
         (unsigned)g_casAutoAnalysis.turboBaud);
  } else {
    casSetEffectiveProfileRuntime("AUTO", false);
    if (server.hasArg("baud")) g_casUserBaseBaud = casClampBaud(server.arg("baud").toInt());
    else g_casUserBaseBaud = 600;
    if (server.hasArg("mult")) {
      uint16_t m = (uint16_t)server.arg("mult").toInt();
      if (!(m == 100 || m == 150 || m == 200 || m == 400)) m = 100;
      g_casTurboMultiplierX100 = m;
    } else g_casTurboMultiplierX100 = 100;
    g_casTurboEnabled = server.hasArg("turbo") && server.arg("turbo").toInt() != 0;
  }
  if (server.hasArg("delay")) {
    int d = server.arg("delay").toInt();
    if (d < 0) d = 0; if (d > 10) d = 10;
    g_casInitialDelaySec = (uint16_t)d;
  } else g_casInitialDelaySec = bootMode ? 0 : 2; // F49U: Boot parte sin delay artificial; el IRG CAS lo aplica el RP2040 antes del registro.
  uint8_t magic[4] = {0,0,0,0};
  bool rawMode = true;
  g_casFujiFormat = false;
  g_casParserStartOffset = 0;
  if (g_casPlayFile.read(magic, 4) == 4 && memcmp(magic, "FUJI", 4) == 0) {
    // Formato CAS/FUJI: FUJI es el primer chunk completo del archivo.
    // Estructura: 4 bytes tipo + 2 bytes largo + 2 bytes aux + datos.
    // No se debe saltar solo la palabra FUJI, porque eso desalinearía el parser.
    rawMode = false;
    g_casFujiFormat = true;
    g_casParserStartOffset = 0;
  }
  g_casPlayFile.seek(g_casParserStartOffset);

  uint16_t startBaud = casEffectiveBaud(g_casUserBaseBaud);
  casSendControlToRP(2, startBaud, 0); // stop limpio previo
  delay(5);
  casSendControlToRP(bootMode ? 5 : 1, startBaud, sz);
  casStartPlaybackState(rawMode, sz, bootMode);
  casSetLastError("");
  handleCasStatus();
}


static bool casFindFujiDataHeaderBack(uint8_t recordsBack, uint32_t currentPos, uint32_t& targetPos) {
  if (!g_casPlayFile) return false;
  if (recordsBack == 0) recordsBack = 1;
  if (recordsBack > 40) recordsBack = 40;
  const uint8_t HIST = 48;
  uint32_t hist[HIST];
  uint8_t used = 0;
  uint32_t saved = g_casPlayFile.position();
  if (!g_casPlayFile.seek(g_casParserStartOffset)) return false;

  while (g_casPlayFile.available()) {
    uint32_t headerPos = g_casPlayFile.position();
    if (headerPos >= currentPos) break;
    char typ[4];
    if (g_casPlayFile.read((uint8_t*)typ, 4) != 4) break;
    int lo = g_casPlayFile.read();
    int hi = g_casPlayFile.read();
    int alo = g_casPlayFile.read();
    int ahi = g_casPlayFile.read();
    if (lo < 0 || hi < 0 || alo < 0 || ahi < 0) break;
    uint16_t chunkLen = (uint16_t)(lo | (hi << 8));
    uint32_t nextPos = headerPos + 8UL + (uint32_t)chunkLen;
    if (memcmp(typ, "data", 4) == 0) {
      if (used < HIST) hist[used++] = headerPos;
      else {
        memmove(hist, hist + 1, sizeof(uint32_t) * (HIST - 1));
        hist[HIST - 1] = headerPos;
      }
    }
    if (nextPos <= headerPos || !g_casPlayFile.seek(nextPos)) break;
  }

  g_casPlayFile.seek(saved);
  if (used == 0) return false;
  int idx = (int)used - 1 - (int)recordsBack;
  if (idx < 0) idx = 0;
  targetPos = hist[idx];
  return true;
}

static void casResetStreamAfterCounterSeek(uint32_t targetPos) {
  g_casPlayFile.seek(targetPos);
  g_casPlayPos = targetPos;
  g_casPaused = false;
  g_casEof = false;
  g_casInDataChunk = false;
  g_casChunkRemain = 0;
  g_casChunkGapMs = 0;
  g_casChunkGapSent = false;
  g_casPendingBlockDelay = false;
  g_casPendingBlockDelayMs = 0;
  g_casRecordDrainPending = false;
  g_casLastIrgAppliedMs = 0;
  g_casTimingWaitUntilMs = 0;
  g_casDrainWaitStartMs = 0;
  g_casDrainTargetBytes = 0;
  g_casDrainDeadlineMs = 0;
  g_casRecordExpectedMs = 0;
  g_casTransportTargetRx = 0;
  g_casPostRecordSettleUntilMs = 0;
  g_casSdriveEndGapPending = false;
  g_casSdriveEndGapMs = 0;
  g_casCurrentRecordMarker = 0;
  g_casCurrentRecordLen = 0;
  g_casRawRecordLen = 0;
  g_casRawRecordPos = 0;
  g_casRawEndRecordQueued = false;
  g_casBytesQueued = 0;
  g_casChunksQueued = 0;
  g_casDataBlocks = 0;
  g_casSkippedChunks = 0;
  g_casFskChunks = 0;
  g_casRecordsCompleted = 0;
  g_casRecordsStarted = 0;
  g_casPrevCompletedRecordMarker = 0;
  g_casRpBufferFree = 4096;
  g_casRpBytesRx = 0;
  g_casRpBytesPlayed = 0;
  g_casCurrentBaud = casEffectiveBaud(g_casUserBaseBaud);
  casSendControlToRP(3, g_casCurrentBaud, g_casPlaySize);
  delay(5);
  casSendControlToRP(g_casBootModeActive ? 5 : 1, g_casCurrentBaud, g_casPlaySize);
}

void handleCasPause() {
  if (!g_casMounted) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"No hay CAS montado en C:\"}");
    return;
  }
  bool pause = true;
  if (server.hasArg("state")) pause = server.arg("state").toInt() != 0;
  if (server.hasArg("resume") && server.arg("resume").toInt() != 0) pause = false;
  if (!g_casPlaying && pause) {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"El cassette no esta reproduciendo\"}");
    return;
  }
  g_casPaused = pause;
  if (pause) g_casPauseCount++; else g_casResumeCount++;
  handleCasStatus();
}

void handleCasResume() {
  g_casPaused = false;
  g_casResumeCount++;
  handleCasStatus();
}

void handleCasSeekBack() {
  if (!g_casMounted || !g_casPlayFile || !g_casPlaying) {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"Para volver contador, primero inicia Play del CAS\"}");
    return;
  }
  int turns = server.hasArg("turns") ? server.arg("turns").toInt() : 3;
  if (turns < 1) turns = 1;
  if (turns > 20) turns = 20;

  uint32_t currentPos = g_casPlayFile.position();
  uint32_t targetPos = g_casParserStartOffset;
  bool ok = false;
  if (g_casRawMode) {
    uint32_t logical = currentPos > g_casParserStartOffset ? (currentPos - g_casParserStartOffset) : 0;
    uint32_t records = logical / 132UL;
    uint32_t back = (uint32_t)turns;
    uint32_t newRecords = (records > back) ? (records - back) : 0;
    targetPos = g_casParserStartOffset + (newRecords * 132UL);
    ok = true;
  } else {
    ok = casFindFujiDataHeaderBack((uint8_t)turns, currentPos, targetPos);
  }
  if (!ok) {
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"No se encontro bloque DATA anterior seguro\"}");
    return;
  }

  casResetStreamAfterCounterSeek(targetPos);
  g_casSeekBackCount++;
  g_casLastSeekBackRecords = (uint8_t)turns;
  handleCasStatus();
}

void handleCasStop() {
  g_casPlaying = false;
  g_casPaused = false;
  g_casEof = false;
  casClosePlaybackFile();
  casSendControlToRP(2, g_casCurrentBaud, g_casBytesQueued);
  g_casStopCount++;
  handleCasStatus();
}

void handleCasRewind() {
  g_casPaused = false;
  g_casRewindCount++;
  if (g_casPlaying && g_casPlayFile) {
    g_casPlayFile.seek(g_casParserStartOffset);
    g_casEof = false;
    g_casInDataChunk = false;
    g_casChunkRemain = 0;
    g_casBytesQueued = 0;
    g_casChunksQueued = 0;
    g_casDataBlocks = 0;
    g_casSkippedChunks = 0;
    g_casFskChunks = 0;
    g_casCurrentBaud = casEffectiveBaud(g_casUserBaseBaud);
    g_casRecordDrainPending = false;
    g_casLastIrgAppliedMs = 0;
    g_casTimingWaitUntilMs = 0;
    g_casDrainWaitStartMs = 0;
    g_casDrainTargetBytes = 0;
    g_casDrainDeadlineMs = 0;
    g_casRecordExpectedMs = 0;
    g_casTransportTargetRx = 0;
    g_casPostRecordSettleUntilMs = 0;
    g_casRecordsCompleted = 0;
    g_casRecordsStarted = 0;
    g_casBootFirstIrgSkipped = false;
    g_casSdriveExactMode = g_casBootModeActive;
    g_casRpRecordBusySkips = 0;
    casSendControlToRP(3, g_casCurrentBaud, g_casPlaySize);
    delay(5);
    casSendControlToRP(g_casBootModeActive ? 5 : 1, g_casCurrentBaud, g_casPlaySize);
  } else {
    casSendControlToRP(3, casEffectiveBaud(g_casUserBaseBaud), g_casPlaySize);
  }
  handleCasStatus();
}

void handleRoot() {
  // V30: servir siempre el HTML embebido y sin caché para evitar que un /index.html
  // antiguo en SPIFFS o en el navegador deje visible una pantalla anterior.
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Vary", "Accept-Encoding");
  server.send_P(200, "text/html; charset=utf-8", (const char*)INDEX_HTML_GZ, INDEX_HTML_GZ_LEN);
}

String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in.charAt(i);
    if (c == '\\') out += "\\\\";
    else if (c == '"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if ((uint8_t)c < 32) out += ' ';
    else out += c;
  }
  return out;
}

void handleApiStatus() {
  String json = String("{\"build\":\"") + MASTER_BUILD + "\",";
  json += "\"drives\":[";
  bool first = true;

  for (int i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
    if (!driveUiVisibleIndex(i)) continue;
    uint8_t devCode = 0x31 + i;
    const char* name = devName(devCode);

    if (!first) json += ",";
    first = false;

    bool physicalSlot = (i < 4);
    bool present = physicalSlot ? slaves[i].present : false;
    bool supports256 = physicalSlot ? slaves[i].supports256 : false;
    uint8_t pf = physicalSlot ? prefetchCfg[i] : 0;
    String macStr = physicalSlot ? formatMac(slaves[i].mac) : String("—");
    unsigned long lastSeen = physicalSlot ? slaves[i].lastSeen : 0;

    uint8_t physDev = physicalSlot ? slaves[i].physicalDev : 0;
    String physName = "-";
    if (physDev >= DEV_MIN && physDev <= DEV_MAX) physName = String(devName(physDev));
    if (!physicalSlot) physName = "virtual";

    uint32_t avgAck = physicalSlot ? g_driveTiming[i].avgAckMs : 0;
    bool autoEn = physicalSlot ? g_driveTiming[i].autoEnabled : false;

    String source = routeNameForDev(devCode);
    bool webActive = webAtrSelected(devCode);
    bool btActive = btSio2pcDevSelected(devCode);

    json += "{";
    json += "\"dev\":\"" + String(name) + "\",";
    json += "\"visible\":" + String(driveUiVisibleIndex(i) ? "true" : "false") + ",";
    json += "\"virtual\":" + String(physicalSlot ? "false" : "true") + ",";
    json += "\"source\":\"" + jsonEscape(source) + "\",";
    json += "\"webAtr\":" + String(webActive ? "true" : "false") + ",";
    json += "\"btSio2pc\":" + String(btActive ? "true" : "false") + ",";
    json += "\"present\":" + String(present ? "true" : "false") + ",";
    json += "\"supports256\":" + String(supports256 ? "true" : "false") + ",";
    json += "\"prefetch\":" + String(pf > 0 ? "true" : "false") + ",";
    json += "\"prefetchSectors\":" + String((int)pf) + ",";
    json += "\"mac\":\"" + macStr + "\",";
    json += "\"physical\":\"" + physName + "\",";
    json += "\"avgAckMs\":" + String((unsigned long)avgAck) + ",";
    json += "\"autoEnabled\":" + String(autoEn ? "true" : "false") + ",";
    json += "\"lastSeen\":" + String(lastSeen);
    json += "}";
  }
  json += "],";
  json += "\"driveVisibleMask\":" + String((int)(DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK)) + ",";
  json += "\"maxUnits\":" + String((int)DRIVE_UI_MAX_UNITS) + ",";

  json += "\"timings\":{";
  json += "\"ackToComplete\":" + String((int)T_ACK_TO_COMPLETE) + ",";
  json += "\"completeToData\":" + String((int)T_COMPLETE_TO_DATA) + ",";
  json += "\"dataToChk\":" + String((int)T_DATA_TO_CHK) + ",";
  json += "\"chunkDelay\":" + String((int)T_CHUNK_DELAY) + ",";
  json += "\"autoProfile\":" + String((int)g_autoProfile);
  json += "}";

  json += ",\"comm\":{";
  json += "\"uartBaud\":" + String((unsigned long)CFG_UART_BAUD) + ",";
  json += "\"rpSioBaud\":" + String((unsigned long)CFG_RP_SIO_BAUD) + ",";
  json += "\"xfSioBaud\":" + String((unsigned long)CFG_XF_SIO_BAUD) + ",";
  json += "\"netDelayUs\":" + String((unsigned long)CFG_NET_DELAY_US);
  json += "}";

  json += ",\"verify\":{";
  json += "\"all\":"  + String((CFG_VERIFY_FLAGS & 0x01) ? 1 : 0) + ",";
  json += "\"boot\":" + String((CFG_VERIFY_FLAGS & 0x02) ? 1 : 0) + ",";
  json += "\"vtoc\":" + String((CFG_VERIFY_FLAGS & 0x04) ? 1 : 0) + ",";
  json += "\"w57\":"  + String((CFG_VERIFY_FLAGS & 0x08) ? 1 : 0);
  json += "}";

  json += ",\"wifi\":";
  json += buildWifiStatusJson(false);

  json += ",\"printer\":{";
  json += "\"enabled\":" + String(PRN_CFG.enabled ? 1 : 0) + ",";
  json += "\"sioDev\":" + String((int)PRN_CFG.sioDev) + ",";
  json += "\"mode\":" + String((int)PRN_CFG.mode) + ",";
  json += "\"ip\":\"" + jsonEscape(String(PRN_CFG.ip)) + "\",";
  json += "\"port\":" + String((int)PRN_CFG.port) + ",";
  json += "\"gateway\":\"" + jsonEscape(String(PRN_CFG.gateway)) + "\",";
  json += "\"name\":\"" + jsonEscape(String(PRN_CFG.name)) + "\",";
  const VirtualPrinterProfile& vp = VIRTUAL_PRINTER_PROFILES[getVirtualPrinterProfileIndex(PRN_CFG.virtualProfile)];
  json += "\"atascii\":" + String(PRN_CFG.atasciiToAscii ? 1 : 0) + ",";
  json += "\"crlf\":" + String(PRN_CFG.appendCrLf ? 1 : 0) + ",";
  json += "\"cut40\":" + String(PRN_CFG.cut40 ? 1 : 0) + ",";
  json += "\"virtualProfile\":" + String((int)PRN_CFG.virtualProfile) + ",";
  json += "\"virtualFont\":" + String((int)PRN_CFG.virtualFont) + ",";
  json += "\"customColumns\":" + String((int)PRN_CFG.customColumns) + ",";
  json += "\"customRows\":" + String((int)PRN_CFG.customRows) + ",";
  json += "\"fontScale\":" + String((int)PRN_CFG.fontScale) + ",";
  json += "\"pageOrientation\":" + String((int)PRN_CFG.pageOrientation) + ",";
  json += "\"paperSize\":" + String((int)PRN_CFG.paperSize) + ",";
  json += "\"paperName\":\"" + jsonEscape(String(virtualPaperName(PRN_CFG.paperSize))) + "\",";
  json += "\"paperMedia\":\"" + jsonEscape(String(virtualPaperMediaKeyword(PRN_CFG.paperSize))) + "\",";
  json += "\"composeMode\":" + String((int)PRN_CFG.composeMode) + ",";
  json += "\"composeModeName\":\"" + jsonEscape(String(printerCompositionModeName(PRN_CFG.composeMode))) + "\",";
  json += "\"renderQuality\":" + String((int)PRN_CFG.renderQuality) + ",";
  json += "\"renderQualityName\":\"" + jsonEscape(String(printerRenderQualityName(PRN_CFG.renderQuality))) + "\",";
  json += "\"autoPrintSpool\":" + String(PRN_CFG.autoPrintSpool ? 1 : 0) + ",";
  json += "\"autoPrintDelayMs\":" + String((int)PRN_CFG.autoPrintDelayMs) + ",";
  json += "\"effectiveColumns\":" + String((int)getVirtualPrinterEffectiveColumns(PRN_CFG.virtualProfile)) + ",";
  json += "\"effectiveRows\":" + String((int)getVirtualPrinterEffectiveRows(PRN_CFG.virtualProfile)) + ",";
  json += "\"effectiveFlushRows\":" + String((int)getVirtualPrinterEffectiveFlushRows(PRN_CFG.virtualProfile)) + ",";
  json += "\"orientationName\":\"" + jsonEscape(String(virtualOrientationName(getVirtualPrinterEffectiveOrientation(PRN_CFG.virtualProfile)))) + "\",";
  json += "\"fontName\":\"" + jsonEscape(String(virtualFontName(PRN_CFG.virtualFont))) + "\",";
  json += "\"glyphJsonLoaded\":" + String(g_fontFilePresent ? 1 : 0) + ",";
  json += "\"glyphJsonStatus\":\"" + jsonEscape(g_fontLoadStatus) + "\",";
  json += "\"glyphJsonDefaultPath\":\"" + jsonEscape(String(ATASCII_GLYPH_JSON_PATH)) + "\",";
  json += "\"glyphJsonReloads\":" + String((unsigned long)g_fontReloadRequests) + ",";
  json += "\"glyphJsonLoadedAtMs\":" + String((unsigned long)g_fontLastLoadMs) + ",";
  json += "\"glyphJsonLastPersistMs\":" + String((unsigned long)g_fontLastPersistMs) + ",";
  json += "\"glyphJsonLastImportMs\":" + String((unsigned long)g_fontLastImportMs) + ",";
  json += "\"glyphJsonLastImportCount\":" + String((unsigned int)g_fontLastImportCount) + ",";
  json += "\"glyphBuiltinDefault\":\"ATASCII_ORIGINAL_8X8_F49Z82\",";
  json += "\"profileName\":\"" + jsonEscape(String(vp.name)) + "\",";
  json += "\"profileColumns\":" + String((int)vp.columns) + ",";
  json += "\"profileRows\":" + String((int)vp.rows) + ",";
  json += "\"bufferPending\":" + String(g_prnVirtualPending ? 1 : 0) + ",";
  json += "\"bufferLines\":" + String((unsigned long)g_prnVirtualLines) + ",";
  json += "\"bufferBytes\":" + String((unsigned long)g_prnVirtualBuffer.size()) + ",";
  json += "\"spoolJobs\":" + String((unsigned long)g_prnSpoolJobs) + ",";
  json += "\"spoolPagesPrinted\":" + String((unsigned long)g_prnSpoolPagesPrinted) + ",";
  json += "\"spoolLastPageBytes\":" + String((unsigned long)g_prnSpoolLastPageBytes) + ",";
  json += "\"spoolLastJpegBytes\":" + String((unsigned long)g_prnSpoolLastJpegBytes) + ",";
  json += "\"manualPrintBusy\":" + String(g_prnManualPrintBusy ? 1 : 0) + ",";
  json += "\"manualPrintRequested\":" + String(g_prnManualPrintRequested ? 1 : 0) + ",";
  json += "\"autoPrintRequested\":" + String(g_prnAutoPrintRequested ? 1 : 0) + ",";
  json += "\"manualLastPages\":" + String((int)g_prnManualLastPages) + ",";
  json += "\"rawTestFormFeed\":" + String(PRN_CFG.rawTestFormFeed ? 1 : 0) + ",";
  json += "\"rawCloseDelayMs\":" + String((int)PRN_CFG.rawCloseDelayMs) + ",";
  json += "\"staEnabled\":" + String(PRN_CFG.staEnabled ? 1 : 0) + ",";
  json += "\"staSsid\":\"" + jsonEscape(String(PRN_CFG.staSsid)) + "\",";
  json += "\"staConnected\":" + String(WiFi.status() == WL_CONNECTED ? 1 : 0) + ",";
  json += "\"staIp\":\"" + jsonEscape(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("")) + "\",";
  json += "\"wifiStatus\":\"" + jsonEscape(printerWifiStatusText()) + "\",";
  json += "\"linesRx\":" + String((unsigned long)g_prnLinesRx) + ",";
  json += "\"linesOk\":" + String((unsigned long)g_prnLinesOk) + ",";
  json += "\"linesErr\":" + String((unsigned long)g_prnLinesErr) + ",";
  json += "\"lastBytes\":" + String((unsigned long)g_prnLastBytes) + ",";
  json += "\"lastText\":\"" + jsonEscape(g_prnLastText) + "\",";
  json += "\"lastError\":\"" + jsonEscape(g_prnLastError) + "\"";
  json += "}";

  json += ",\"devices\":[";
  bool firstDev = true;
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (!g_devices[i].used) continue;
    if (!firstDev) json += ",";
    firstDev = false;

    String macStr = formatMac(g_devices[i].mac);
    String physStr = devName(g_devices[i].physicalDev);
    uint8_t logic = mapGetLogic(g_devices[i].mac);
    String logicStr = logic ? devName(logic) : String("-");

    json += "{";
    json += "\"mac\":\"" + macStr + "\",";
    json += "\"physical\":\"" + physStr + "\",";
    json += "\"logical\":\"" + logicStr + "\",";
    json += "\"present\":" + String(g_devices[i].present ? 1 : 0) + ",";
    json += "\"supports256\":" + String(g_devices[i].supports256 ? 1 : 0);
    json += "}";
  }
  json += "]";

  json += "}";

  server.send(200, "application/json", json);
}

void handleSetTiming() {
  if (server.hasArg("t_ack"))   T_ACK_TO_COMPLETE = (uint16_t)server.arg("t_ack").toInt();
  if (server.hasArg("t_comp"))  T_COMPLETE_TO_DATA = (uint16_t)server.arg("t_comp").toInt();
  if (server.hasArg("t_chk"))   T_DATA_TO_CHK = (uint16_t)server.arg("t_chk").toInt();
  if (server.hasArg("t_chunk")) T_CHUNK_DELAY = (uint16_t)server.arg("t_chunk").toInt();
  saveTimingConfigToNvs();
  sendTimingUpdateToRP();
  server.send(200, "text/plain", "OK");
}

void handleSetPrefetch() {
  int pf1 = server.hasArg("pf1") ? server.arg("pf1").toInt() : prefetchCfg[0];
  int pf2 = server.hasArg("pf2") ? server.arg("pf2").toInt() : prefetchCfg[1];
  int pf3 = server.hasArg("pf3") ? server.arg("pf3").toInt() : prefetchCfg[2];
  int pf4 = server.hasArg("pf4") ? server.arg("pf4").toInt() : prefetchCfg[3];

  auto clampPf = [](int v) -> uint8_t {
    if (v < 0) v = 0;
    if (v > MAX_PREFETCH_SECTORS) v = MAX_PREFETCH_SECTORS;
    return (uint8_t)v;
  };

  prefetchCfg[0] = clampPf(pf1);
  prefetchCfg[1] = clampPf(pf2);
  prefetchCfg[2] = clampPf(pf3);
  prefetchCfg[3] = clampPf(pf4);

  savePrefetchConfigToNvs();
  server.send(200, "text/plain", "OK");
}

void handleSetAuto() {
  int a1 = server.hasArg("d1") ? server.arg("d1").toInt() : (g_driveTiming[0].autoEnabled ? 1 : 0);
  int a2 = server.hasArg("d2") ? server.arg("d2").toInt() : (g_driveTiming[1].autoEnabled ? 1 : 0);
  int a3 = server.hasArg("d3") ? server.arg("d3").toInt() : (g_driveTiming[2].autoEnabled ? 1 : 0);
  int a4 = server.hasArg("d4") ? server.arg("d4").toInt() : (g_driveTiming[3].autoEnabled ? 1 : 0);

  g_driveTiming[0].autoEnabled = (a1 != 0);
  g_driveTiming[1].autoEnabled = (a2 != 0);
  g_driveTiming[2].autoEnabled = (a3 != 0);
  g_driveTiming[3].autoEnabled = (a4 != 0);

  if (server.hasArg("profile")) {
    int p = server.arg("profile").toInt();
    if (p < 0) p = 0;
    if (p > 2) p = 2;
    g_autoProfile = p;
  }

  saveAutoConfigToNvs();
  saveTimingConfigToNvs();
  server.send(200, "text/plain", "OK");
}

bool pushCfgToRPAndMaybeSwitchUart() {
  g_rpCfgAck = false;
  g_rpCfgAck_ok = 0;
  g_rpCfgAck_uart = 0;
  g_rpCfgAck_sio  = 0;

  sendCfgUpdateToRP(CFG_UART_BAUD, CFG_RP_SIO_BAUD);

  uint32_t t0 = millis();
  while (!g_rpCfgAck && (millis() - t0) < 500) {
    pollUartFromRP();
    delay(1);
  }
  if (!g_rpCfgAck || !g_rpCfgAck_ok) {
    logf("[MASTER] ❌ No llegó CFG_ACK desde RP (uart actual=%lu)", (unsigned long)g_currentUartBaud);
    return false;
  }

  if (CFG_UART_BAUD != g_currentUartBaud) {
    delay(60);
    Serial2.updateBaudRate(CFG_UART_BAUD);
    g_currentUartBaud = CFG_UART_BAUD;
    logf("[MASTER] ✅ UART RP<->MASTER ahora %lu", (unsigned long)g_currentUartBaud);
  }
  return true;
}

void handleSetComm() {
  if (server.hasArg("uart")) { uint32_t req = (uint32_t)server.arg("uart").toInt(); CFG_UART_BAUD = pickSupportedUartBaud(req);}
  if (server.hasArg("sio_rp")) CFG_RP_SIO_BAUD  = clampU32((uint32_t)server.arg("sio_rp").toInt(), 9600, 115200);
  if (server.hasArg("sio_xf")) CFG_XF_SIO_BAUD  = clampU32((uint32_t)server.arg("sio_xf").toInt(), 9600, 115200);
  if (server.hasArg("net_us")) CFG_NET_DELAY_US = clampU32((uint32_t)server.arg("net_us").toInt(), 0, 20000);

  saveCommConfigToNvs();
  broadcastSlaveCfg();
  pushCfgToRPAndMaybeSwitchUart();
  sendTimingUpdateToRP();
  sendPrinterCfgToRP();

  server.send(200, "text/plain", "OK");
}

void handleSetVerify() {
  int a = server.hasArg("v_all")  ? server.arg("v_all").toInt()  : ((CFG_VERIFY_FLAGS & 0x01) ? 1 : 0);
  int b = server.hasArg("v_boot") ? server.arg("v_boot").toInt() : ((CFG_VERIFY_FLAGS & 0x02) ? 1 : 0);
  int c = server.hasArg("v_vtoc") ? server.arg("v_vtoc").toInt() : ((CFG_VERIFY_FLAGS & 0x04) ? 1 : 0);
  int d = server.hasArg("v_57")   ? server.arg("v_57").toInt()   : ((CFG_VERIFY_FLAGS & 0x08) ? 1 : 0);

  CFG_VERIFY_FLAGS = 0;
  if (a) CFG_VERIFY_FLAGS |= 0x01;
  if (b) CFG_VERIFY_FLAGS |= 0x02;
  if (c) CFG_VERIFY_FLAGS |= 0x04;
  if (d) CFG_VERIFY_FLAGS |= 0x08;

  saveVerifyConfigToNvs();
  broadcastSlaveCfg();

  server.send(200, "text/plain", "OK");
}

// Cambia el mapeo MAC -> Unidad lógica (D1..D4)
void handleSetLogic() {
  if (!server.hasArg("mac") || !server.hasArg("to")) {
    server.send(400, "text/plain", "Falta mac o to");
    return;
  }
  uint8_t mac[6];
  if (!parseMacString(server.arg("mac"), mac)) {
    server.send(400, "text/plain", "MAC inválida");
    return;
  }
  uint8_t logic = devFromStr(server.arg("to"));
  if (logic < DEV_MIN || logic > DEV_MAX) {
    server.send(400, "text/plain", "Unidad lógica inválida");
    return;
  }

  if (!mapSetLogicForMac(mac, logic)) {
    server.send(500, "text/plain", "No se pudo guardar");
    return;
  }

  // reflejar en slots inmediatamente (si conocemos el equipo)
  int di = findDeviceSlotByMac(mac);
  bool supports256 = (di >= 0) ? g_devices[di].supports256 : false;
  uint8_t physDev  = (di >= 0) ? g_devices[di].physicalDev : 0;

  int slot = devIdx(logic);
  if (slot >= 0) clearSlaveSlot(slot);
  clearSlotsByMac(mac);
  applyDeviceToLogicalSlot(mac, logic, supports256, physDev);

  server.send(200, "text/plain", "OK");
}


void handleSetPrinter() {
  if (server.hasArg("enabled")) PRN_CFG.enabled = server.arg("enabled").toInt() != 0;

  if (server.hasArg("dev")) {
    int d = server.arg("dev").toInt();
    if (d >= 0x40 && d <= 0x43) PRN_CFG.sioDev = (uint8_t)d;
  }

  if (server.hasArg("mode")) {
    int m = server.arg("mode").toInt();
    if (m >= PRN_MODE_RAW9100 && m <= PRN_MODE_IPP_JPEG) PRN_CFG.mode = (uint8_t)m;
  }

  if (server.hasArg("ip")) server.arg("ip").toCharArray(PRN_CFG.ip, sizeof(PRN_CFG.ip));

  if (server.hasArg("port")) {
    int prt = server.arg("port").toInt();
    if (prt > 0 && prt <= 65535) PRN_CFG.port = (uint16_t)prt;
  }

  if (server.hasArg("gateway")) server.arg("gateway").toCharArray(PRN_CFG.gateway, sizeof(PRN_CFG.gateway));
  if (server.hasArg("name")) server.arg("name").toCharArray(PRN_CFG.name, sizeof(PRN_CFG.name));

  if (server.hasArg("atascii")) PRN_CFG.atasciiToAscii = server.arg("atascii").toInt() != 0;
  if (server.hasArg("crlf"))    PRN_CFG.appendCrLf       = server.arg("crlf").toInt() != 0;
  if (server.hasArg("cut40"))   PRN_CFG.cut40            = server.arg("cut40").toInt() != 0;
  if (server.hasArg("profile")) {
    int prof = server.arg("profile").toInt();
    if (prof < 0) prof = 0;
    if (prof >= (int)VIRTUAL_PRINTER_PROFILE_COUNT) prof = 0;
    PRN_CFG.virtualProfile = (uint8_t)prof;
  }
  if (server.hasArg("font")) {
    int f = server.arg("font").toInt();
    if (f < 0 || f > VF_ATASCII_MEJORADA_12X16) f = VF_ATASCII_ORIGINAL_8X8;
    PRN_CFG.virtualFont = (uint8_t)f;
  }
  PRN_CFG.customColumns = 0; // V69: columnas manuales eliminadas
  PRN_CFG.customRows = 0;    // V69: filas manuales eliminadas
  if (server.hasArg("fontScale")) {
    int fs = server.arg("fontScale").toInt();
    if (!(fs == 8 || fs == 10 || fs == 12 || fs == 14 || fs == 16)) fs = 12;
    PRN_CFG.fontScale = (uint8_t)fs;
  }
  if (server.hasArg("orientation")) {
    int o = server.arg("orientation").toInt();
    if (o < VP_ORIENT_AUTO || o > VP_ORIENT_LANDSCAPE) o = VP_ORIENT_AUTO;
    PRN_CFG.pageOrientation = (uint8_t)o;
  }
  if (server.hasArg("paper")) {
    int p = server.arg("paper").toInt();
    if (p < VP_PAPER_A4 || p > VP_PAPER_LETTER) p = VP_PAPER_A4;
    PRN_CFG.paperSize = (uint8_t)p;
  }
  if (server.hasArg("compose")) {
    int cm = server.arg("compose").toInt();
    if (cm < PRN_COMPOSE_FIEL_820 || cm > PRN_COMPOSE_EXTENDIDA) cm = PRN_COMPOSE_EXTENDIDA;
    PRN_CFG.composeMode = (uint8_t)cm;
  }
  // V120: la interfaz ya no expone calidad experimental; siempre queda JPEG estable.
  PRN_CFG.renderQuality = PRN_RENDER_COMPAT_JPEG;
  if (server.hasArg("rawFF"))   PRN_CFG.rawTestFormFeed  = server.arg("rawFF").toInt() != 0;
  if (server.hasArg("rawDelay")) {
    int rd = server.arg("rawDelay").toInt();
    if (rd < 0) rd = 0;
    if (rd > 5000) rd = 5000;
    PRN_CFG.rawCloseDelayMs = (uint16_t)rd;
  }
  if (server.hasArg("staEnabled")) PRN_CFG.staEnabled = server.arg("staEnabled").toInt() != 0;
  if (server.hasArg("staSsid")) server.arg("staSsid").toCharArray(PRN_CFG.staSsid, sizeof(PRN_CFG.staSsid));
  if (server.hasArg("staPass") && server.arg("staPass").length() > 0) {
    server.arg("staPass").toCharArray(PRN_CFG.staPass, sizeof(PRN_CFG.staPass));
  }

  savePrinterConfigToNvs();
  if (PRN_CFG.staEnabled) connectPrinterStaWifi(false);
  sendPrinterCfgToRP();

  server.send(200, "text/plain", "OK");
}


void handleGlyphsExport() {
  // F49Z79: endpoint único para la grilla. Al pedir reload/default, recarga SD a RAM;
  // luego exporta esa RAM, sin usar el JSON viejo embebido como fuente primaria.
  bool forceReload = server.hasArg("reload") || server.hasArg("verify") || server.hasArg("default") || server.hasArg("source");
  ensureAtasciiGlyphDefaultLoaded(forceReload);
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "application/json", buildAllGlyphJsonF49Z60());
}

void handleGlyphsImport() {
  if (!g_fontFsReady) {
    server.send(500, "text/plain", "Almacenamiento no disponible");
    return;
  }
  String body = server.arg("plain");
  if (body.length() < 10) {
    server.send(400, "text/plain", "JSON vacío");
    return;
  }
  if (body.length() > 70000) {
    server.send(413, "text/plain", "JSON demasiado grande");
    return;
  }
  if (!parseAtasciiGlyphJson(body)) {
    server.send(400, "text/plain", "JSON sin glifos válidos: " + g_fontLoadStatus);
    return;
  }
  if (!persistCurrentGlyphJson()) {
    server.send(500, "text/plain", "No se pudo persistir JSON importado: " + g_fontLoadStatus);
    return;
  }
  g_fontFilePresent = true;
  g_fontLastImportMs = millis();
  g_fontLastImportCount = countCustomFont8Valid();
  logf("[ATASCII-FONT] JSON import OK default=%s status=%s", ATASCII_GLYPH_JSON_PATH, g_fontLoadStatus.c_str());
  server.send(200, "text/plain", "OK default actualizado en MASTER: " + g_fontLoadStatus);
}


// V67: importador compacto y tolerante.
// El navegador extrae solo code + rowsHex y manda lineas pequenas:
// 41:00,18,3C,66,7E,66,66,00
// Evita errores "Failed to fetch" por POST JSON grande o JSON con nombres no escapados.
void handleGlyphsImportCompact() {
  if (!g_fontFsReady) {
    server.sendHeader("Connection", "close");
    server.send(500, "text/plain", "Almacenamiento no disponible");
    return;
  }

  String body = server.arg("plain");
  body.trim();
  if (body.length() < 8) {
    server.sendHeader("Connection", "close");
    server.send(400, "text/plain", "Import compacto vacío");
    return;
  }
  if (body.length() > 12000) {
    server.sendHeader("Connection", "close");
    server.send(413, "text/plain", "Import compacto demasiado grande");
    return;
  }

  clearCustomFonts();

  uint16_t loaded = 0;
  int pos = 0;
  while (pos < (int)body.length()) {
    int eol = body.indexOf('\n', pos);
    String line = (eol >= 0) ? body.substring(pos, eol) : body.substring(pos);
    line.trim();
    pos = (eol >= 0) ? (eol + 1) : body.length();

    if (line.length() < 19) continue;
    int sep = line.indexOf(':');
    if (sep < 0) sep = line.indexOf('=');
    if (sep <= 0) continue;

    String codeStr = line.substring(0, sep);
    codeStr.trim();
    uint8_t code = (uint8_t)(parseHex16Str(codeStr) & 0x7F);

    uint16_t rows[8];
    memset(rows, 0, sizeof(rows));
    uint8_t count = 0;
    int rpos = sep + 1;
    while (count < 8 && rpos <= (int)line.length()) {
      int comma = line.indexOf(',', rpos);
      String part = (comma >= 0) ? line.substring(rpos, comma) : line.substring(rpos);
      part.trim();
      if (part.length() > 0) rows[count++] = (uint16_t)(parseHex16Str(part) & 0x00FF);
      if (comma < 0) break;
      rpos = comma + 1;
    }

    if (count == 8) {
      for (uint8_t y = 0; y < 8; y++) g_customFont8[code][y] = rows[y];
      g_customFont8Valid[code] = true;
      loaded++;
    }
  }

  if (loaded == 0) {
    g_fontLoadStatus = "compacto sin glifos válidos";
    server.sendHeader("Connection", "close");
    server.send(400, "text/plain", g_fontLoadStatus);
    return;
  }

  if (!persistCurrentGlyphJson()) {
    server.sendHeader("Connection", "close");
    server.send(500, "text/plain", "No se pudo persistir JSON: " + g_fontLoadStatus);
    return;
  }

  g_fontFilePresent = true;
  g_fontLastImportMs = millis();
  g_fontLastImportCount = loaded;
  refreshGlyphLoadStatusFromRam();
  logf("[ATASCII-FONT] COMPACT import OK default=%s loaded=%u status=%s", ATASCII_GLYPH_JSON_PATH, (unsigned int)loaded, g_fontLoadStatus.c_str());
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", "OK default actualizado en MASTER: " + g_fontLoadStatus + " loaded=" + String((unsigned int)loaded));
}


void handleGlyphGet() {
  ensureAtasciiGlyphDefaultLoaded(server.hasArg("reload") || server.hasArg("default"));
  uint8_t font = VF_ATASCII_ORIGINAL_8X8;
  uint8_t code = parseGlyphCodeArgF49Z62(server.arg("code"));
  const bool valid = g_customFont8Valid[code];
  String json = "{\n";
  json += "\"font\":" + String((int)font) + ",";
  json += "\"fontName\":\"" + jsonEscape(String(virtualFontName(font))) + "\",";
  json += "\"code\":" + String((int)code) + ",";
  json += "\"codeHex\":\"" + hexByte2(code) + "\",";
  json += "\"name\":\"" + jsonEscape(glyphDisplayName(code)) + "\",";
  json += "\"width\":" + String((int)virtualGlyphW(font)) + ",";
  json += "\"height\":" + String((int)virtualGlyphH(font)) + ",";
  json += "\"valid\":" + String(valid ? 1 : 0) + ",";
  json += "\"source\":\"" + String(valid ? "JSON/custom" : "fallback/base") + "\",";
  json += "\"rowsHex\":" + rowsHexForGlyphJson(font, code);
  json += "\n}";
  server.send(200, "application/json", json);
}

void handleGlyphSet() {
  if (!g_fontFsReady) {
    server.send(500, "text/plain", "Almacenamiento no disponible");
    return;
  }
  uint8_t font = VF_ATASCII_ORIGINAL_8X8;
  uint8_t code = parseGlyphCodeArgF49Z62(server.arg("code"));
  String rows = server.arg("rows");
  rows.replace("%2C", ",");
  rows.replace("%2c", ",");
  uint8_t wanted = 8;
  uint16_t parsed[16];
  memset(parsed, 0, sizeof(parsed));
  uint8_t count = 0;
  int start = 0;
  while (count < wanted && start <= (int)rows.length()) {
    int comma = rows.indexOf(',', start);
    String part = (comma >= 0) ? rows.substring(start, comma) : rows.substring(start);
    part.trim();
    if (part.length() > 0) parsed[count++] = parseHex16Str(part);
    if (comma < 0) break;
    start = comma + 1;
  }
  if (count != wanted) {
    server.send(400, "text/plain", "Cantidad de filas inválida");
    return;
  }
  for (uint8_t y = 0; y < 8; y++) g_customFont8[code][y] = (uint16_t)(parsed[y] & 0x00FF);
  g_customFont8Valid[code] = true;
  if (!persistCurrentGlyphJson()) {
    server.send(500, "text/plain", "No se pudo persistir JSON: " + g_fontLoadStatus);
    return;
  }
  logf("[ATASCII-FONT] GLYPH_SAVE font=%u code=0x%02X %s", font, code, g_fontLoadStatus.c_str());
  server.send(200, "text/plain", "OK code=0x" + hexByte2(code) + " " + g_fontLoadStatus);
}

void handleGlyphRestore() {
  if (!g_fontFsReady) {
    server.send(500, "text/plain", "Almacenamiento no disponible");
    return;
  }
  uint8_t font = VF_ATASCII_ORIGINAL_8X8;
  uint8_t code = parseGlyphCodeArgF49Z62(server.arg("code"));
  // F49Z82: restaurar vuelve al default real del app, no a un glifo inválido
  // ni al fallback aproximado antiguo.
  for (uint8_t y = 0; y < 8; y++) g_customFont8[code][y] = builtinDefaultGlyphRow8(code, y);
  g_customFont8Valid[code] = true;
  if (!persistCurrentGlyphJson()) {
    server.send(500, "text/plain", "No se pudo persistir JSON: " + g_fontLoadStatus);
    return;
  }
  logf("[ATASCII-FONT] GLYPH_RESTORE font=%u code=0x%02X %s", font, code, g_fontLoadStatus.c_str());
  server.send(200, "text/plain", "OK code=0x" + hexByte2(code) + " restaurado");
}

void handlePrinterTest() {
  g_prnLastText = "=== ATARI SIO PRINTER TEST ===";
  g_prnLastError = "";
  g_prnLastBytes = 0;
  g_prnLinesRx++;

  String job;
  job += "=== ATARI SIO PRINTER TEST ===\r\n";
  job += "MASTER ESP32 OK\r\n";
  job += "P: DEVICE READY\r\n";
  job += "Brother DCP-T720DW RAW 9100\r\n";
  job += "STA: " + WiFi.localIP().toString() + "\r\n";
  job += "PRN: " + String(PRN_CFG.ip) + ":" + String(PRN_CFG.port) + "\r\n";

  bool ok = false;
  if (PRN_CFG.enabled && PRN_CFG.mode == PRN_MODE_RAW9100) {
    ok = printerSendRaw9100Job(job, PRN_CFG.rawTestFormFeed);
  } else if (PRN_CFG.enabled && PRN_CFG.mode == PRN_MODE_HTTP) {
    ok = printerSendHttpGatewayJob(job);
  } else {
    ok = printerPrintLine("=== ATARI SIO PRINTER TEST ===");
  }

  if (ok) {
    g_prnLinesOk++;
    String modeName = (PRN_CFG.mode == PRN_MODE_HTTP) ? "HTTP" : "RAW";
    server.send(200, "text/plain", String("OK ") + modeName + " bytes=" + String((unsigned long)g_prnLastBytes) + " ff=" + String(PRN_CFG.rawTestFormFeed ? 1 : 0));
  } else {
    g_prnLinesErr++;
    if (g_prnLastError.length() == 0) g_prnLastError = "Fallo prueba impresora";
    server.send(500, "text/plain", g_prnLastError);
  }
}

void handlePrinterClear() {
  g_prnLinesRx = 0;
  g_prnLinesOk = 0;
  g_prnLinesErr = 0;
  g_prnLastText = "";
  g_prnLastError = "";
  g_prnLastBytes = 0;
  g_prnHttpBuffer = "";
  g_prnHttpBufferLines = 0;
  g_prnHttpPending = false;
  g_prnVirtualBuffer.clear();
  g_prnVirtualLines = 0;
  g_prnVirtualPending = false;
  g_prnVirtualPageFullLatched = false;
  g_prnAutoPrintBlockedAfterError = false;
  g_prnSpoolLastPageBytes = 0;
  g_prnSpoolLastJpegBytes = 0;
  server.send(200, "text/plain", "OK");
}


// ===== WiFi S3 Safe Boot =====
static bool startMasterSoftApS3Safe() {
  bool ok = false;
  IPAddress ip(0, 0, 0, 0);

  logf("[WIFI] S3 safe init begin channel=%u", (unsigned)WIFI_CHANNEL);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(true, true);
  WiFi.softAPdisconnect(true);
  delay(250);

  WiFi.mode(WIFI_OFF);
  delay(250);

  WiFi.mode(WIFI_AP_STA);
  delay(250);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  bool cfgOk = WiFi.softAPConfig(MASTER_AP_IP, MASTER_AP_GW, MASTER_AP_MASK);
  ok = WiFi.softAP(MASTER_AP_SSID, MASTER_AP_PASS, WIFI_CHANNEL, 0, 4);
  delay(600);
  ip = WiFi.softAPIP();

  logf("[WIFI] AP try1 cfg=%d ok=%d ip=%s mac=%s mode=%d ch=%u",
       cfgOk ? 1 : 0,
       ok ? 1 : 0,
       ip.toString().c_str(),
       WiFi.softAPmacAddress().c_str(),
       (int)WiFi.getMode(),
       (unsigned)WiFi.channel());

  if (!ok || ip == IPAddress(0, 0, 0, 0)) {
    logf("[WIFI] AP try1 fallo; reintentando en modo AP_STA limpio");
    WiFi.softAPdisconnect(true);
    delay(300);
    WiFi.mode(WIFI_OFF);
    delay(300);
    WiFi.mode(WIFI_AP_STA);
    delay(300);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    cfgOk = WiFi.softAPConfig(MASTER_AP_IP, MASTER_AP_GW, MASTER_AP_MASK);
    ok = WiFi.softAP(MASTER_AP_SSID, MASTER_AP_PASS, WIFI_CHANNEL, 0, 4);
    delay(700);
    ip = WiFi.softAPIP();
    logf("[WIFI] AP try2 cfg=%d ok=%d ip=%s mac=%s mode=%d ch=%u",
         cfgOk ? 1 : 0,
         ok ? 1 : 0,
         ip.toString().c_str(),
         WiFi.softAPmacAddress().c_str(),
         (int)WiFi.getMode(),
         (unsigned)WiFi.channel());
  }

  if (ok && ip != IPAddress(0, 0, 0, 0)) {
    logf("[WIFI] AP READY ssid=%s pass=%s ip=%s", MASTER_AP_SSID, MASTER_AP_PASS, ip.toString().c_str());
    return true;
  }

  logf("[WIFI] AP ERROR: no se pudo levantar SoftAP");
  return false;
}

static void deferStaConnectAfterWebStart() {
  if (!PRN_CFG.staEnabled) return;
  if (strlen(PRN_CFG.staSsid) == 0) {
    logf("[WIFI] STA diferido omitido: SSID vacio");
    return;
  }
  g_staConnectDeferred = true;
  g_staConnectAfterMs = millis() + 3500;
  logf("[WIFI-LAN] STA diferido: AP/web primero, luego SSID='%s'", PRN_CFG.staSsid);
}

static void serviceDeferredStaConnect() {
  if (!g_staConnectDeferred) return;
  if ((int32_t)(millis() - g_staConnectAfterMs) < 0) return;
  g_staConnectDeferred = false;
  logf("[WIFI-LAN] STA inicio diferido ahora");
  connectPrinterStaWifi(false);
}

void handleBtDiskSet() {
  if (server.hasArg("en")) BT_DISK_ENABLED = server.arg("en").toInt() != 0;
  if (server.hasArg("mask")) {
    int m = server.arg("mask").toInt();
    if (m < 0) m = 0;
    if (m > 15) m = 15;
    BT_DISK_DEV_MASK = (uint8_t)m;
  }
  if (server.hasArg("force")) {
    int f = server.arg("force").toInt();
    if (f < 0) f = 0;
    if (f > 15) f = 15;
    BT_DISK_FORCE_MASK = (uint8_t)f;
  }
  // Conveniencia: phys=0 deshabilita física para las unidades de mask; phys=1 re-habilita fallback físico.
  if (server.hasArg("phys")) {
    bool physEnabled = server.arg("phys").toInt() != 0;
    if (physEnabled) BT_DISK_FORCE_MASK &= (uint8_t)~BT_DISK_DEV_MASK;
    else BT_DISK_FORCE_MASK |= (BT_DISK_DEV_MASK & 0x0F);
    BT_DISK_FORCE_MASK &= 0x0F;
  }
  if (server.hasArg("baud")) {
    uint32_t b = (uint32_t)server.arg("baud").toInt();
    if (b >= 9600 && b <= 921600 && b != BT_DISK_UART_BAUD_CFG) {
      BT_DISK_UART_BAUD_CFG = b;
      if (g_btDiskUartReady) {
        SerialBTDisk.end();
        delay(20);
        beginBtDiskUart();
      }
    }
  }
  markDeferredConfigSave(false, true);
  String json = String("{\"ok\":1,\"deferredSave\":1,\"enabled\":") + (BT_DISK_ENABLED ? "1" : "0") +
                ",\"mask\":" + String((int)BT_DISK_DEV_MASK) +
                ",\"forceMask\":" + String((int)BT_DISK_FORCE_MASK) +
                ",\"baud\":" + String((unsigned long)BT_DISK_UART_BAUD_CFG) +
                ",\"online\":" + String(g_btDiskOnline ? 1 : 0) + "}";
  server.send(200, "application/json", json);
}


void handleBtSio2pcSet() {
  if (server.hasArg("en")) BT_SIO2PC_ENABLED = server.arg("en").toInt() != 0;
  if (server.hasArg("mask")) {
    int m = server.arg("mask").toInt();
    if (m < 0) m = 0;
    if (m > DRIVE_UI_MAX_MASK) m = DRIVE_UI_MAX_MASK;
    BT_SIO2PC_DEV_MASK = (uint8_t)m;
  }
  if (server.hasArg("force")) {
    int f = server.arg("force").toInt();
    if (f < 0) f = 0;
    if (f > DRIVE_UI_MAX_MASK) f = DRIVE_UI_MAX_MASK;
    BT_SIO2PC_FORCE_MASK = (uint8_t)f;
  }
  if (server.hasArg("phys")) {
    bool physEnabled = server.arg("phys").toInt() != 0;
    if (physEnabled) BT_SIO2PC_FORCE_MASK &= (uint8_t)~BT_SIO2PC_DEV_MASK;
    else BT_SIO2PC_FORCE_MASK |= (BT_SIO2PC_DEV_MASK & DRIVE_UI_MAX_MASK);
    BT_SIO2PC_FORCE_MASK &= DRIVE_UI_MAX_MASK;
  }
  if (server.hasArg("baud")) {
    uint32_t b = (uint32_t)server.arg("baud").toInt();
    if (b >= 9600 && b <= 921600 && b != BT_DISK_UART_BAUD_CFG) {
      BT_DISK_UART_BAUD_CFG = b;
      if (g_btDiskUartReady) { SerialBTDisk.end(); delay(20); beginBtDiskUart(); }
    }
  }
  if (server.hasArg("profile")) {
    uint8_t p = btSio2pcParseProfile(server.arg("profile"));
    btSio2pcApplyProfile(p);
    // Compatibilidad con URLs antiguas: aplica el perfil a las unidades seleccionadas.
    uint8_t targetMask = BT_SIO2PC_DEV_MASK ? BT_SIO2PC_DEV_MASK : DRIVE_UI_DEFAULT_VISIBLE_MASK;
    for (uint8_t i = 0; i < DRIVE_UI_MAX_UNITS; i++) if (targetMask & (1u << i)) BT_SIO2PC_UNIT_PROFILE[i] = p;
  }
  for (uint8_t i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
    char keyA[4]; snprintf(keyA, sizeof(keyA), "p%u", (unsigned)(i + 1));
    char keyB[10]; snprintf(keyB, sizeof(keyB), "profile%u", (unsigned)(i + 1));
    if (server.hasArg(keyA)) BT_SIO2PC_UNIT_PROFILE[i] = btSio2pcParseProfile(server.arg(keyA));
    if (server.hasArg(keyB)) BT_SIO2PC_UNIT_PROFILE[i] = btSio2pcParseProfile(server.arg(keyB));
    if (BT_SIO2PC_UNIT_PROFILE[i] > 3) BT_SIO2PC_UNIT_PROFILE[i] = 1;
  }
  if (server.hasArg("quiet")) BT_SIO2PC_QUIET_LOG = server.arg("quiet").toInt() != 0;
  if (server.hasArg("cache")) BT_SIO2PC_CACHE_ENABLED = server.arg("cache").toInt() != 0;
  if (server.hasArg("safe")) {
    BT_SIO2PC_CACHE_ENABLED = false;
    btSio2pcApplyProfile(0);
    for (uint8_t i = 0; i < DRIVE_UI_MAX_UNITS; i++) BT_SIO2PC_UNIT_PROFILE[i] = 0;
  }

  normalizeDriveMasks();
  // F46: cualquier cambio de ruta/perfil/baud invalida caché para evitar lecturas antiguas.
  btSio2pcCacheClear();
  bool webAtrChangedByBt = false;
  if (BT_SIO2PC_ENABLED) {
    BT_DISK_ENABLED = false;
    // Regla de exclusión: si BT-SIO2PC toma una unidad, desmonta WEB-ATR en esa unidad.
    uint8_t beforeWebMask = WEB_ATR_DEV_MASK;
    uint8_t beforeWebForce = WEB_ATR_FORCE_MASK;
    bool beforeWebEnabled = WEB_ATR_ENABLED;
    clearWebAtrUnits(BT_SIO2PC_DEV_MASK, true);
    webAtrChangedByBt = (beforeWebMask != WEB_ATR_DEV_MASK) || (beforeWebForce != WEB_ATR_FORCE_MASK) || (beforeWebEnabled != WEB_ATR_ENABLED);
  }
  markDeferredConfigSave(webAtrChangedByBt, true);

  String json = String("{\"ok\":1,\"deferredSave\":1,\"sio2pcEnabled\":") + (BT_SIO2PC_ENABLED ? "1" : "0") +
                ",\"sio2pcMask\":" + String((int)BT_SIO2PC_DEV_MASK) +
                ",\"sio2pcForceMask\":" + String((int)BT_SIO2PC_FORCE_MASK) +
                ",\"baud\":" + String((unsigned long)BT_DISK_UART_BAUD_CFG) +
                ",\"profile\":\"" + String(btSio2pcProfileName(BT_SIO2PC_PROFILE)) + "\"" +
                ",\"unitProfiles\":[";
  for (uint8_t i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
    if (i) json += ",";
    json += "\"" + String(btSio2pcProfileName(BT_SIO2PC_UNIT_PROFILE[i])) + "\"";
  }
  json += "]";
  json += ",\"quiet\":" + String(BT_SIO2PC_QUIET_LOG ? 1 : 0);
  json += ",\"cache\":" + String(BT_SIO2PC_CACHE_ENABLED ? 1 : 0) + "}";
  server.send(200, "application/json", json);
}

void handleBtDiskStatus() {
  String json = "{";
  json += "\"build\":\"" + jsonEscape(String(MASTER_BUILD)) + "\",";
  json += "\"enabled\":" + String(BT_DISK_ENABLED ? 1 : 0) + ",";
  json += "\"online\":" + String(g_btDiskOnline ? 1 : 0) + ",";
  json += "\"mask\":" + String((int)BT_DISK_DEV_MASK) + ",";
  json += "\"forceMask\":" + String((int)BT_DISK_FORCE_MASK) + ",";
  json += "\"d1PhysicalDisabled\":" + String((BT_DISK_FORCE_MASK & 0x01) ? 1 : 0) + ",";
  json += "\"baud\":" + String((unsigned long)BT_DISK_UART_BAUD_CFG) + ",";
  json += "\"sio2pcEnabled\":" + String(BT_SIO2PC_ENABLED ? 1 : 0) + ",";
  json += "\"sio2pcMask\":" + String((int)BT_SIO2PC_DEV_MASK) + ",";
  json += "\"sio2pcForceMask\":" + String((int)BT_SIO2PC_FORCE_MASK) + ",";
  json += "\"driveVisibleMask\":" + String((int)(DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK)) + ",";
  json += "\"maxUnits\":" + String((int)DRIVE_UI_MAX_UNITS) + ",";
  json += "\"sio2pcProfile\":\"" + String(btSio2pcProfileName(BT_SIO2PC_PROFILE)) + "\",";
  json += "\"sio2pcUnitProfiles\":[";
  for (uint8_t i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
    if (i) json += ",";
    json += "\"" + String(btSio2pcProfileName(BT_SIO2PC_UNIT_PROFILE[i])) + "\"";
  }
  json += "],";
  json += "\"sio2pcQuietLog\":" + String(BT_SIO2PC_QUIET_LOG ? 1 : 0) + ",";
  json += "\"sio2pcCacheEnabled\":" + String(BT_SIO2PC_CACHE_ENABLED ? 1 : 0) + ",";
  json += "\"sio2pcBurstReads\":" + String((unsigned long)g_btSio2pcBurstReads) + ",";
  json += "\"sio2pcBytesRx\":" + String((unsigned long)g_btSio2pcBytesRx) + ",";
  json += "\"sio2pcAvgMs\":" + String((unsigned long)g_btSio2pcAvgMs) + ",";
  json += "\"sio2pcMaxMs\":" + String((unsigned long)g_btSio2pcMaxMs) + ",";
  json += "\"sio2pcRetry\":" + String((unsigned long)g_btSio2pcRetry) + ",";
  json += "\"sio2pcDrainBytes\":" + String((unsigned long)g_btSio2pcDrainBytes) + ",";
  json += "\"sio2pcNoWebLoops\":" + String((unsigned long)g_btSio2pcNoWebLoops) + ",";
  json += "\"sio2pcCacheHit\":" + String((unsigned long)g_btSio2pcCacheHit) + ",";
  json += "\"sio2pcCacheMiss\":" + String((unsigned long)g_btSio2pcCacheMiss) + ",";
  json += "\"sio2pcCacheStore\":" + String((unsigned long)g_btSio2pcCacheStore) + ",";
  json += "\"sio2pcCacheClear\":" + String((unsigned long)g_btSio2pcCacheClear) + ",";
  json += "\"sio2pcCacheBypass\":" + String((unsigned long)g_btSio2pcCacheBypass) + ",";
  json += "\"sio2pcSioPrioritySkips\":" + String((unsigned long)g_btSio2pcSioPrioritySkips) + ",";
  json += "\"sio2pcStaleDrains\":" + String((unsigned long)g_btSio2pcStaleDrains) + ",";
  json += "\"sio2pcLateBytesAfterError\":" + String((unsigned long)g_btSio2pcLateBytesAfterError) + ",";
  json += "\"masterUartFlushEachFrame\":" + String((int)MASTER_UART_FLUSH_EACH_FRAME) + ",";
  json += "\"sio2pcRxBuffer\":" + String((unsigned long)BT_SIO2PC_UART_RX_BUFFER) + ",";
  json += "\"sio2pcTxBuffer\":" + String((unsigned long)BT_SIO2PC_UART_TX_BUFFER) + ",";
  json += "\"sio2pcLastErr\":\"" + jsonEscape(String(g_btSio2pcLastErr)) + "\",";
  json += "\"sio2pcTimeoutsMs\":{\"ack\":" + String(BT_SIO2PC_ACK_TIMEOUT_MS) + ",\"complete\":" + String(BT_SIO2PC_COMPLETE_TIMEOUT_MS) + ",\"dataTotal\":" + String(BT_SIO2PC_DATA_TOTAL_TIMEOUT_MS) + ",\"dataIdle\":" + String(BT_SIO2PC_DATA_IDLE_TIMEOUT_MS) + ",\"chk\":" + String(BT_SIO2PC_CHK_TIMEOUT_MS) + ",\"preDrain\":" + String(BT_SIO2PC_PRE_DRAIN_MS) + ",\"retry\":" + String(BT_SIO2PC_MAX_RETRY) + "},";
  json += "\"sio2pcCmdRouted\":" + String((unsigned long)g_btSio2pcCmdRouted) + ",";
  json += "\"sio2pcReadOk\":" + String((unsigned long)g_btSio2pcReadOk) + ",";
  json += "\"sio2pcStatusOk\":" + String((unsigned long)g_btSio2pcStatusOk) + ",";
  json += "\"sio2pcPercomOk\":" + String((unsigned long)g_btSio2pcPercomOk) + ",";
  json += "\"sio2pcNak\":" + String((unsigned long)g_btSio2pcNak) + ",";
  json += "\"sio2pcTimeout\":" + String((unsigned long)g_btSio2pcTimeout) + ",";
  json += "\"sio2pcBadChk\":" + String((unsigned long)g_btSio2pcBadChk) + ",";
  json += "\"pins\":{\"rx\":" + String(PIN_BT_RX) + ",\"tx\":" + String(PIN_BT_TX) + "},";
  json += "\"framesRx\":" + String((unsigned long)g_btDiskFramesRx) + ",";
  json += "\"framesTx\":" + String((unsigned long)g_btDiskFramesTx) + ",";
  json += "\"badChk\":" + String((unsigned long)g_btDiskBadChk) + ",";
  json += "\"badLen\":" + String((unsigned long)g_btDiskBadLen) + ",";
  json += "\"cmdRouted\":" + String((unsigned long)g_btDiskCmdRouted) + ",";
  json += "\"chunksRx\":" + String((unsigned long)g_btDiskChunksRx) + ",";
  json += "\"helloIgnoredDuringOp\":" + String((unsigned long)g_btDiskHelloIgnoredDuringOp) + ",";
  json += "\"lastDev\":" + String((int)g_btDiskLastDev) + ",";
  json += "\"lastCmd\":" + String((int)g_btDiskLastCmd) + ",";
  json += "\"lastSec\":" + String((unsigned)g_btDiskLastSec);
  json += "}";
  server.send(200, "application/json", json);
}


void handleDriveVisibleStatus() {
  normalizeDriveMasks();
  const int nextAdd = nextHiddenOptionalDriveUnit();
  const int nextRemove = lastRemovableOptionalDriveUnit();
  String json = "{";
  json += "\"ok\":1";
  json += ",\"visibleMask\":" + String((int)(DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK));
  json += ",\"maxUnits\":" + String((int)DRIVE_UI_MAX_UNITS);
  json += ",\"nextAddUnit\":" + String(nextAdd);
  json += ",\"nextRemoveUnit\":" + String(nextRemove);
  json += ",\"units\":[";
  for (int i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
    const bool visible = driveUiVisibleIndex(i);
    const bool fixed = i < 4;
    const bool busy = optionalDriveBusyIndex(i);
    const bool removable = visible && !fixed && !busy;
    if (i) json += ",";
    json += "{\"unit\":" + String(i + 1);
    json += ",\"dev\":\"" + String(devName(0x31 + i)) + "\"";
    json += ",\"visible\":" + String(visible ? 1 : 0);
    json += ",\"fixed\":" + String(fixed ? 1 : 0);
    json += ",\"removable\":" + String(removable ? 1 : 0);
    json += ",\"busy\":" + String((!fixed && busy) ? 1 : 0);
    json += ",\"busyReason\":\"" + jsonEscape(optionalDriveBusyReasonIndex(i)) + "\"";
    json += "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleDriveVisibleSet() {
  int u = server.hasArg("unit") ? server.arg("unit").toInt() : 0;
  int visible = server.hasArg("visible") ? server.arg("visible").toInt() : 1;
  if (u < 5 || u > DRIVE_UI_MAX_UNITS) {
    server.send(400, "text/plain", "Solo se pueden agregar/quitar D5-D7. D1-D4 son fijas.");
    return;
  }
  const int idx = u - 1;
  const uint8_t bit = (uint8_t)(1u << idx);
  const bool currentlyVisible = (DRIVE_VISIBLE_MASK & bit) != 0;

  if (visible) {
    if (!currentlyVisible) DRIVE_VISIBLE_MASK |= bit;
  } else {
    if (optionalDriveBusyIndex(idx)) {
      String reason = optionalDriveBusyReasonIndex(idx);
      server.send(409, "text/plain", String("Primero desmonta/desactiva D") + String(u) + (reason.length() ? String(": ") + reason : String("")));
      return;
    }
    if (currentlyVisible) DRIVE_VISIBLE_MASK &= (uint8_t)~bit;
  }

  normalizeDriveMasks();
  persistDriveUiState();
  handleDriveVisibleStatus();
}

void handleDriveVisibleAdd() {
  int u = nextHiddenOptionalDriveUnit();
  if (!u) {
    server.send(409, "text/plain", "Ya están visibles D1-D7.");
    return;
  }
  DRIVE_VISIBLE_MASK |= (uint8_t)(1u << (u - 1));
  normalizeDriveMasks();
  persistDriveUiState();
  handleDriveVisibleStatus();
}

void handleDriveVisibleRemove() {
  int u = server.hasArg("unit") ? server.arg("unit").toInt() : 0;
  if (u == 0) u = lastRemovableOptionalDriveUnit();
  if (u < 5 || u > DRIVE_UI_MAX_UNITS) {
    server.send(409, "text/plain", "No hay unidades opcionales libres para quitar.");
    return;
  }
  const int idx = u - 1;
  if (!driveUiVisibleIndex(idx)) {
    server.send(409, "text/plain", String("D") + String(u) + " no está visible.");
    return;
  }
  if (optionalDriveBusyIndex(idx)) {
    String reason = optionalDriveBusyReasonIndex(idx);
    server.send(409, "text/plain", String("Primero desmonta/desactiva D") + String(u) + (reason.length() ? String(": ") + reason : String("")));
    return;
  }
  DRIVE_VISIBLE_MASK &= (uint8_t)~(1u << idx);
  normalizeDriveMasks();
  persistDriveUiState();
  handleDriveVisibleStatus();
}

// ===== Android REST API V14 =====
// Capa REST estable para app Android nativa. Mantiene compatibilidad con la web existente
// y reutiliza el mismo estado interno: WEB-ATR, BT-SIO2PC y unidades dinámicas D5-D7.

static void apiSendJsonError(int code, const String& message) {
  String json = String("{\"ok\":false,\"error\":\"") + jsonEscape(message) + "\"}";
  server.send(code, "application/json", json);
}

static String apiJsonStringField(const String& body, const char* key) {
  String pattern = String("\"") + key + "\"";
  int k = body.indexOf(pattern);
  if (k < 0) return String("");
  int colon = body.indexOf(':', k + pattern.length());
  if (colon < 0) return String("");
  int i = colon + 1;
  while (i < (int)body.length() && (body[i] == ' ' || body[i] == '\t' || body[i] == '\r' || body[i] == '\n')) i++;
  if (i >= (int)body.length()) return String("");

  if (body[i] == '"') {
    i++;
    String out;
    while (i < (int)body.length()) {
      char c = body[i++];
      if (c == '\\' && i < (int)body.length()) {
        char e = body[i++];
        if (e == 'n') out += '\n';
        else if (e == 'r') out += '\r';
        else if (e == 't') out += '\t';
        else out += e;
      } else if (c == '"') {
        break;
      } else {
        out += c;
      }
    }
    out.trim();
    return out;
  }

  int end = i;
  while (end < (int)body.length() && body[end] != ',' && body[end] != '}') end++;
  String out = body.substring(i, end);
  out.trim();
  return out;
}

static int apiValidateDriveUnit(int unit) {
  if (unit < 1 || unit > DRIVE_UI_MAX_UNITS) return 0;
  return unit;
}

static void apiPersistStorageConfig() {
  normalizeDriveMasks();
  webAtrRefreshPresence();
  saveWebAtrConfigToNvs();
  saveBtDiskConfigToNvs();
}

static void apiAppendDriveJson(String& json, int idx) {
  const uint8_t bit = (uint8_t)(1u << idx);
  const uint8_t dev = (uint8_t)(0x31 + idx);
  const bool visible = driveUiVisibleIndex(idx);
  const bool dynamic = idx >= 4;
  const bool webSelected = WEB_ATR_ENABLED && ((WEB_ATR_DEV_MASK & bit) != 0);
  const bool webMounted = webSelected && g_webAtrMountedName[idx].length() > 0 && g_webAtrSlotPresent[idx];
  const bool btSelected = BT_SIO2PC_ENABLED && ((BT_SIO2PC_DEV_MASK & bit) != 0);
  const bool removable = visible && dynamic && !optionalDriveBusyIndex(idx);

  String source = "NONE";
  if (webSelected) source = "WEB_ATR";
  else if (btSelected) source = "BT_SIO2PC";

  WebAtrMeta m;
  bool metaOk = false;
  if (g_webAtrMountedName[idx].length() > 0) metaOk = webAtrReadMetaForIndex(idx, m, false);

  json += "{";
  json += "\"id\":\"" + String(devName(dev)) + "\"";
  json += ",\"unit\":" + String(idx + 1);
  json += ",\"visible\":" + String(visible ? "true" : "false");
  json += ",\"dynamic\":" + String(dynamic ? "true" : "false");
  json += ",\"removable\":" + String(removable ? "true" : "false");
  json += ",\"source\":\"" + source + "\"";
  json += ",\"route\":\"" + jsonEscape(String(routeNameForDev(dev))) + "\"";
  json += ",\"file\":\"" + jsonEscape(g_webAtrMountedName[idx]) + "\"";
  json += ",\"btEnabled\":" + String(btSelected ? "true" : "false");
  json += ",\"btForced\":" + String((BT_SIO2PC_FORCE_MASK & bit) ? "true" : "false");
  json += ",\"mounted\":" + String(webMounted ? "true" : "false");
  json += ",\"webAtrEnabled\":" + String(webSelected ? "true" : "false");
  json += ",\"webAtrForced\":" + String((WEB_ATR_FORCE_MASK & bit) ? "true" : "false");
  json += ",\"present\":" + String(metaOk ? "true" : "false");
  json += ",\"sectorSize\":" + String(metaOk ? m.sectorSize : 0);
  json += ",\"totalSectors\":" + String(metaOk ? m.totalSectors : 0);
  json += ",\"dirty\":false";
  json += "}";
}

void handleApiDrives() {
  normalizeDriveMasks();
  webAtrRefreshPresence();

  String json = "{";
  json += "\"ok\":true";
  json += ",\"build\":\"" + jsonEscape(String(MASTER_BUILD)) + "\"";
  json += ",\"driveVisibleMask\":" + String((int)(DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK));
  json += ",\"maxUnits\":" + String((int)DRIVE_UI_MAX_UNITS);
  json += ",\"btSio2pcEnabled\":" + String(BT_SIO2PC_ENABLED ? "true" : "false");
  json += ",\"webAtrEnabled\":" + String(WEB_ATR_ENABLED ? "true" : "false");
  json += ",\"drives\":[";
  for (int i = 0; i < DRIVE_UI_MAX_UNITS; i++) {
    if (i) json += ",";
    apiAppendDriveJson(json, i);
  }
  json += "]}";
  server.send(200, "application/json", json);
}


static void webLibraryAppendSaveStatusJson(String& json) {
  json += ",\"indexSaved\":" + String(g_webLibraryIndexLastSaveOk ? "true" : "false");
  json += ",\"indexVerifyOk\":" + String(g_webLibraryIndexLastVerifyOk ? "true" : "false");
  json += ",\"indexSaveMs\":" + String(g_webLibraryIndexLastSaveMs);
  json += ",\"indexSavedBytes\":" + String(g_webLibraryIndexLastSavedBytes);
  json += ",\"indexVerifyCount\":" + String(g_webLibraryIndexLastVerifyCount);
  json += ",\"indexSaveError\":\"" + jsonEscape(g_webLibraryIndexLastSaveError) + "\"";
  json += ",\"indexBuildOk\":true";
  json += ",\"indexSaveBlocking\":false";
}

void handleApiLibraryIndexJson() {
  // F43P: endpoint legacy mantenido solo por compatibilidad.
  // Ya no lee /CONFIG/library_index.json ni .bak.json porque esos archivos viejos
  // pueden estar truncados/inválidos. La fuente persistente real es NDJSON.
  if (!webAtrFsReady()) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", "[]");
    return;
  }

  String entries;
  uint32_t total = 0;
  String source;
  uint32_t bytes = 0;
  bool ok = webLibraryNdjsonReadAllEntries(entries, total, source, bytes);

  String out;
  if (ok) {
    out.reserve(entries.length() + 2);
    out = "[";
    out += entries;
    out += "]";
  } else {
    out = "[]";
  }

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "application/json", out);
}


// F42V: fallback de servicio. Si el archivo index fue verificado pero la carga
// desde SD/RAM queda vacia, reconstruimos EN RAM usando el escaneo seguro y
// servimos esa pagina sin tocar SD. Evita total=0/files=[] en navegacion normal.
static bool webLibraryBuildLiveIndexForServe(const char* reason) {
  if (!webAtrFsReady()) return false;
  uint32_t t0 = millis();
  String live;
  live.reserve(49152);
  webAtrAppendFileListJson(live);
  webLibraryNormalizeEntriesJson(live);
  uint32_t c = webLibraryCountObjects(live);
  bool valid = webLibraryEntriesLooksValid(live);
  if (c == 0 || !valid) {
    logf("[LIB-IDX] live serve fallback fallo reason=%s count=%lu valid=%u len=%lu", reason ? reason : "", (unsigned long)c, valid ? 1 : 0, (unsigned long)live.length());
    return false;
  }
  g_webLibraryIndexJson = live;
  g_webLibraryIndexCount = c;
  g_webLibraryIndexLoaded = true;
  g_webLibraryIndexDirty = false;
  g_webLibraryIndexBuildMs = millis() - t0;
  g_webLibraryIndexLastSaveOk = false;
  g_webLibraryIndexLastVerifyOk = false;
  g_webLibraryIndexLastSaveError = String("Fallback live serve RAM; SD no escrita. reason=") + String(reason ? reason : "");
  logf("[LIB-IDX] live serve fallback OK reason=%s count=%lu bytes=%lu ms=%lu", reason ? reason : "", (unsigned long)c, (unsigned long)g_webLibraryIndexJson.length(), (unsigned long)g_webLibraryIndexBuildMs);
  return true;
}

void handleApiLibrary() {
  if (!webAtrFsReady()) {
    apiSendJsonError(500, "Almacenamiento no disponible");
    return;
  }

  bool refreshArg = (server.hasArg("refresh") && server.arg("refresh") == "1") || (server.hasArg("rebuild") && server.arg("rebuild") == "1");
  bool manualRefresh = refreshArg && (
    (server.hasArg("manual") && server.arg("manual") == "1") ||
    (server.hasArg("confirm") && server.arg("confirm") == "1")
  );
  // F42J: refresh manual reconstruye RAM, pero NO escribe SD salvo commit=1.
  // Esto evita que entrar/salir de Cassette o un refresh web deje library_index.json en [].
  bool commitRefresh = manualRefresh && server.hasArg("commit") && server.arg("commit") == "1";
  bool forceRefresh = manualRefresh;
  if (refreshArg && !manualRefresh) {
    logf("[LIB-IDX] refresh ignorado por no venir con manual=1; usando indice existente");
  }

  // F43I: el botón Refrescar Biblioteca sin commit es la única ruta de
  // escaneo manual sin escritura. Debe escanear, poblar cache RAM y devolver
  // files[] visibles. No toca /CONFIG/library_index.json y no cae en el flujo
  // F43K: refresh manual sin commit escanea únicamente porque viene del botón,
  // actualiza cache RAM y responde files[] por streaming seguro.
  if (manualRefresh && !commitRefresh) {
    webLibraryLiveScanCacheInvalidate("manual refresh button ndjson stream");

    int rPageSize = server.hasArg("pageSize") ? server.arg("pageSize").toInt() : (server.hasArg("limit") ? server.arg("limit").toInt() : 50);
    if (rPageSize <= 0) rPageSize = 50;
    if (rPageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) rPageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX;
    int rPage = server.hasArg("page") ? server.arg("page").toInt() : 0;
    if (server.hasArg("offset")) rPage = server.arg("offset").toInt() / rPageSize;
    if (rPage < 0) rPage = 0;
    String rQ = server.hasArg("q") ? server.arg("q") : String(""); rQ.toLowerCase();
    String rType = server.hasArg("type") ? server.arg("type") : String(""); rType.toUpperCase();

    std::vector<WebLibraryScanEntry> liveEntries;
    bool cacheHit = false;
    uint32_t cacheAge = 0;
    uint32_t buildMs = 0;
    bool scanOk = webLibraryLiveScanCacheGet(liveEntries, true, cacheHit, cacheAge, buildMs);

    g_webLibraryIndexCount = liveEntries.size();
    g_webLibraryIndexBuildMs = buildMs;
    g_webLibraryIndexLastSaveOk = false;
    g_webLibraryIndexLastVerifyOk = false;
    g_webLibraryIndexLastSaveMs = 0;
    g_webLibraryIndexLastSavedBytes = 0;
    g_webLibraryIndexLastVerifyCount = 0;
    g_webLibraryIndexLastSaveError = scanOk ? String("Refresco manual sin commit: cache RAM actualizada; SD no escrita") : String("Refresco manual falló: no se pudo escanear SD");

    if (!scanOk) {
      String err = "{\"ok\":false,\"source\":\"library_manual_live_scan\",\"refreshed\":true,\"manualRefresh\":true,\"indexCommit\":false,\"autoScanDisabled\":true,\"error\":\"SCAN_FAILED\",\"files\":[]}";
      server.sendHeader("Cache-Control", "no-store");
      server.send(500, "application/json", err);
      return;
    }

    webLibraryStreamApiResponseFromEntries(liveEntries,
      String("library_manual_live_scan"), true, true, true, false, true, false,
      rPage, rPageSize, rQ, rType, buildMs);
    return;
  }

  // F43K: commit explícito con NDJSON. Escanea desde botón, guarda/verifica línea por línea
  // y responde files[] por streaming seguro para evitar JSON truncado con pageSize alto.
  if (commitRefresh) {
    webLibraryLiveScanCacheInvalidate("manual commit ndjson stream");
    std::vector<WebLibraryScanEntry> commitEntries;
    bool commitCacheHit = false;
    uint32_t commitCacheAge = 0;
    uint32_t commitCacheBuild = 0;
    bool commitCacheOk = webLibraryLiveScanCacheGet(commitEntries, true, commitCacheHit, commitCacheAge, commitCacheBuild);
    bool ndjsonSaved = commitCacheOk && webLibraryNdjsonSaveFromEntries(commitEntries);

    int cPageSize = server.hasArg("pageSize") ? server.arg("pageSize").toInt() : (server.hasArg("limit") ? server.arg("limit").toInt() : 50);
    if (cPageSize <= 0) cPageSize = 50;
    if (cPageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) cPageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX;
    int cPage = server.hasArg("page") ? server.arg("page").toInt() : 0;
    if (server.hasArg("offset")) cPage = server.arg("offset").toInt() / cPageSize;
    if (cPage < 0) cPage = 0;
    String cQ = server.hasArg("q") ? server.arg("q") : String(""); cQ.toLowerCase();
    String cType = server.hasArg("type") ? server.arg("type") : String(""); cType.toUpperCase();

    g_webLibraryIndexBuildMs = commitCacheBuild;
    g_webLibraryIndexCount = commitEntries.size();

    if (!commitCacheOk) {
      String err = "{\"ok\":false,\"source\":\"library_ndjson_commit\",\"refreshed\":true,\"manualRefresh\":true,\"indexCommit\":true,\"ndjson\":true,\"error\":\"SCAN_FAILED\",\"files\":[]}";
      server.sendHeader("Cache-Control", "no-store");
      server.send(500, "application/json", err);
      return;
    }

    webLibraryStreamApiResponseFromEntries(commitEntries,
      String("library_ndjson_commit"), (commitCacheOk && ndjsonSaved), true, true, true, true, true,
      cPage, cPageSize, cQ, cType, commitCacheBuild,
      ndjsonSaved ? String("") : g_webLibraryIndexLastSaveError);
    return;
  }

  bool indexOk = false;
  bool refreshFailed = false;
  String refreshError = "";

  // F43I: commit=1 usa rebuild/guardado; refresh manual sin commit ya respondió arriba.
  // F43B: refresh=1&manual=1 sin commit ya no pasa por el rebuild antiguo.
  // Solo invalida cache y lista por live-scan paginado, evitando count parcial
  // y RAM/index JSON vacio. Para escribir SD se exige commit=1.
  if (manualRefresh) {
    webLibraryLiveScanCacheInvalidate(commitRefresh ? "manual commit" : "manual refresh");
    bool prevConsoleScan = g_webLibraryConsoleScan;
    g_webLibraryConsoleScan = webLibraryConsoleScanArgEnabled();
    indexOk = webLibraryIndexRebuild(commitRefresh, true);
    g_webLibraryConsoleScan = prevConsoleScan;
    if (!indexOk) {
      // F42M: el refresco manual no debe tumbar la web ni devolver 500.
      // Si el re-escaneo SD falla o devuelve 0, se conserva/recupera el mejor
      // índice disponible desde RAM, library_index.json, .bak o .prev.
      refreshFailed = true;
      refreshError = g_webLibraryIndexLastSaveError;
      if (!refreshError.length()) refreshError = "Rebuild manual falló; se conserva índice existente/backup";
      logf("[LIB-IDX] refresh manual fallo; fallback a indice existente: %s", refreshError.c_str());
      indexOk = webLibraryIndexEnsure(false, false);
    }
  } else {
    // F42Y: navegación normal NO debe tocar el índice persistente ni RAM.
    // El botón Biblioteca debe ir directo al escaneo seguro paginado más abajo.
    // Esto evita el ciclo observado: load len=0 -> RAM inválida -> fallback -> abort().
    indexOk = true;
  }

  if (!indexOk) {
    // F42V: si cargar desde SD falla pero la SD esta lista, intentamos servir
    // desde escaneo seguro en RAM, sin guardar en SD.
    if (!commitRefresh && webLibraryBuildLiveIndexForServe("indexOk=false")) {
      indexOk = true;
    }
  }

  if (!indexOk) {
    // F42M: nunca responder 500 en /api/library por falta de índice. La UI debe
    // poder renderizar estado vacío y permitir recuperación manual sin caer.
    String json = "{\"ok\":true";
    json += ",\"source\":\"library_index\"";
    json += ",\"indexMissing\":true";
    json += ",\"needsRefresh\":true";
    json += ",\"refreshFailed\":" + String(refreshFailed ? "true" : "false");
    if (refreshError.length()) json += ",\"refreshError\":\"" + jsonEscape(refreshError) + "\"";
    json += ",\"indexPath\":\"" + jsonEscape(String(WEB_LIBRARY_INDEX_PATH)) + "\"";
    json += ",\"page\":0,\"pageSize\":50,\"total\":0,\"typeCounts\":{\"ALL\":0,\"ATR\":0,\"XEX\":0,\"COM\":0,\"EXE\":0,\"BAS\":0,\"CAS\":0,\"SEC\":0,\"OTHER\":0},\"files\":[]}";
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    server.send(200, "application/json", json);
    return;
  }

  // F42N: commit=1 debe ser una operación de guardado/confirmación, no una
  // descarga del listado completo. En F42M esta misma ruta seguía cayendo al
  // flujo streaming de files[] y en algunos ESP32/WebServer se cortaba con
  // ERR_INCOMPLETE_CHUNKED_ENCODING. Respondemos un JSON pequeño y después la
  // web/app puede pedir /api/library?refresh=0 para listar paginado.
  if (commitRefresh) {
    // F43D: commit=1 sí escanea y guarda; además devuelve la página visible.
    // En F43C la respuesta era compacta con files:[], lo que hacía parecer que
    // no había escaneo. Ahora el commit deja cache RAM poblado y responde page/files.
    int cPageSize = server.hasArg("pageSize") ? server.arg("pageSize").toInt() : (server.hasArg("limit") ? server.arg("limit").toInt() : 50);
    if (cPageSize <= 0) cPageSize = 50;
    if (cPageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) cPageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX;
    int cPage = server.hasArg("page") ? server.arg("page").toInt() : 0;
    if (server.hasArg("offset")) cPage = server.arg("offset").toInt() / cPageSize;
    if (cPage < 0) cPage = 0;
    String cQ = server.hasArg("q") ? server.arg("q") : String(""); cQ.toLowerCase();
    String cType = server.hasArg("type") ? server.arg("type") : String(""); cType.toUpperCase();

    // Poblar cache vivo desde el mismo botón/acción manual. Esta es la única
    // ruta, junto a refresh=1&manual=1 sin commit, que puede escanear la SD.
    std::vector<WebLibraryScanEntry> commitEntries;
    bool commitCacheHit = false;
    uint32_t commitCacheAge = 0;
    uint32_t commitCacheBuild = 0;
    bool commitCacheOk = webLibraryLiveScanCacheGet(commitEntries, true, commitCacheHit, commitCacheAge, commitCacheBuild);

    String pageEntries;
    uint32_t totalMatches = 0;
    uint32_t lcAll = 0, lcAtr = 0, lcXex = 0, lcCom = 0, lcExe = 0, lcBas = 0, lcCas = 0, lcSec = 0, lcOther = 0;
    uint32_t liveMs = 0;
    bool pageOk = commitCacheOk && webLibraryBuildLiveVisiblePage(cPage, cPageSize, cQ, cType, pageEntries, totalMatches,
                                                                  lcAll, lcAtr, lcXex, lcCom, lcExe, lcBas, lcCas, lcSec, lcOther,
                                                                  liveMs);

    String json;
    json.reserve(4096 + pageEntries.length());
    json = "{\"ok\":true";
    json += ",\"source\":\"library_commit_live_page\"";
    json += ",\"refreshed\":true";
    json += ",\"indexCommit\":true";
    json += ",\"commitOnly\":false";
    json += ",\"commitReturnedFiles\":" + String(pageOk ? "true" : "false");
    json += ",\"indexPath\":\"" + jsonEscape(String(WEB_LIBRARY_INDEX_PATH)) + "\"";
    json += ",\"indexCount\":" + String(g_webLibraryIndexCount);
    json += ",\"indexBuildMs\":" + String(g_webLibraryIndexBuildMs);
    json += ",\"indexLoadMs\":" + String(g_webLibraryIndexLoadMs);
    webLibraryAppendSaveStatusJson(json);
    json += ",\"page\":" + String(cPage);
    json += ",\"pageSize\":" + String(pageOk ? cPageSize : 0);
    json += ",\"total\":" + String(pageOk ? totalMatches : g_webLibraryIndexCount);
    json += ",\"typeCounts\":{";
    json += "\"ALL\":" + String(pageOk ? lcAll : g_webLibraryIndexCount);
    json += ",\"ATR\":" + String(pageOk ? lcAtr : 0);
    json += ",\"XEX\":" + String(pageOk ? lcXex : 0);
    json += ",\"COM\":" + String(pageOk ? lcCom : 0);
    json += ",\"EXE\":" + String(pageOk ? lcExe : 0);
    json += ",\"BAS\":" + String(pageOk ? lcBas : 0);
    json += ",\"CAS\":" + String(pageOk ? lcCas : 0);
    json += ",\"SEC\":" + String(pageOk ? lcSec : 0);
    json += ",\"OTHER\":" + String(pageOk ? lcOther : 0);
    json += "}";
    json += ",\"files\":[";
    if (pageOk) json += pageEntries;
    json += "]}";
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "0");
    server.send(200, "application/json", json);
    return;
  }

  int pageSize = server.hasArg("pageSize") ? server.arg("pageSize").toInt() : (server.hasArg("limit") ? server.arg("limit").toInt() : 0);
  // F43P: /api/library nunca devuelve una página gigante.
  // Esto NO limita el total: total/indexCount siguen informando todos los archivos encontrados,
  // pero files[] se entrega en páginas de 50 para no fragmentar heap ni disparar reinicios.
  if (pageSize <= 0) pageSize = WEB_LIBRARY_SAFE_HTTP_PAGE_SIZE;
  if (pageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) pageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX;

  int page = server.hasArg("page") ? server.arg("page").toInt() : 0;
  if (server.hasArg("offset")) page = server.arg("offset").toInt() / pageSize;
  if (page < 0) page = 0;
  String q = server.hasArg("q") ? server.arg("q") : String(""); q.toLowerCase();
  String type = server.hasArg("type") ? server.arg("type") : String(""); type.toUpperCase();

  // F43N: ruta principal para navegación normal.
  // Regla final: refresh=0 NO escanea SD y NO usa library_index.json legacy.
  // Primero lee /CONFIG/library_index.ndjson línea por línea. Si no existe,
  // usa la cache RAM viva ya cargada por el botón manual. Si tampoco existe,
  // pide refresco manual. Esto elimina los logs legacy:
  //   load count=0 path=/CONFIG/library_index.json len=0
  // y deja claro que la fuente persistente real es NDJSON.
  if (!manualRefresh) {
    if (pageSize > WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX) pageSize = WEB_LIBRARY_VISIBLE_PAGE_SIZE_MAX;

    String ndEntries;
    uint32_t ndTotal = 0;
    uint32_t ndAll = 0, ndAtr = 0, ndXex = 0, ndCom = 0, ndExe = 0, ndBas = 0, ndCas = 0, ndSec = 0, ndOther = 0;
    String ndSource;
    uint32_t ndBytes = 0;

    if (webLibraryNdjsonBestPage(page, pageSize, q, type, ndEntries, ndTotal,
                                 ndAll, ndAtr, ndXex, ndCom, ndExe, ndBas, ndCas, ndSec, ndOther,
                                 ndSource, ndBytes)) {
      g_webLibraryIndexCount = ndAll;
      String json;
      json.reserve(4096 + ndEntries.length());
      json = "{\"ok\":true";
      json += ",\"source\":\"library_ndjson_index\"";
      json += ",\"indexSource\":\"" + jsonEscape(ndSource) + "\"";
      json += ",\"refreshed\":false";
      json += ",\"indexCommit\":false";
      json += ",\"autoScanDisabled\":true";
      json += ",\"ndjson\":true";
      json += ",\"usedLegacyJson\":false";
      json += ",\"indexPath\":\"" + jsonEscape(String(WEB_LIBRARY_INDEX_NDJSON_PATH)) + "\"";
      json += ",\"legacyJsonPathDisabled\":\"/CONFIG/library_index.json\"";
      json += ",\"indexCount\":" + String(ndAll);
      json += ",\"indexFileBytes\":" + String(ndBytes);
      json += ",\"indexBuildMs\":" + String(g_webLibraryIndexBuildMs);
      json += ",\"indexLoadMs\":0";
      webLibraryAppendSaveStatusJson(json);
      json += ",\"page\":" + String(page);
      json += ",\"pageSize\":" + String(pageSize);
      json += ",\"total\":" + String(ndTotal);
      json += ",\"typeCounts\":{";
      json += "\"ALL\":" + String(ndAll);
      json += ",\"ATR\":" + String(ndAtr);
      json += ",\"XEX\":" + String(ndXex);
      json += ",\"COM\":" + String(ndCom);
      json += ",\"EXE\":" + String(ndExe);
      json += ",\"BAS\":" + String(ndBas);
      json += ",\"CAS\":" + String(ndCas);
      json += ",\"SEC\":" + String(ndSec);
      json += ",\"OTHER\":" + String(ndOther);
      json += "},\"files\":[";
      json += ndEntries;
      json += "]}";
      server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
      server.sendHeader("Pragma", "no-cache");
      server.sendHeader("Expires", "0");
      server.send(200, "application/json", json);
      return;
    }

    // Fallback permitido: cache RAM creada por el botón manual. No escanea SD.
    String liveEntries;
    uint32_t liveTotal = 0;
    uint32_t lcAll = 0, lcAtr = 0, lcXex = 0, lcCom = 0, lcExe = 0, lcBas = 0, lcCas = 0, lcSec = 0, lcOther = 0;
    uint32_t liveMs = 0;
    if (webLibraryBuildLiveVisiblePage(page, pageSize, q, type, liveEntries, liveTotal,
                                       lcAll, lcAtr, lcXex, lcCom, lcExe, lcBas, lcCas, lcSec, lcOther, liveMs)) {
      String json;
      json.reserve(4096 + liveEntries.length());
      json = "{\"ok\":true";
      json += ",\"source\":\"library_ram_cache_fallback\"";
      json += ",\"refreshed\":false";
      json += ",\"indexCommit\":false";
      json += ",\"autoScanDisabled\":true";
      json += ",\"ndjson\":false";
      json += ",\"ndjsonMissing\":true";
      json += ",\"message\":\"NDJSON no disponible; usando cache RAM creada por refresco manual\"";
      json += ",\"indexPath\":\"" + jsonEscape(String(WEB_LIBRARY_INDEX_NDJSON_PATH)) + "\"";
      json += ",\"indexCount\":" + String(lcAll);
      json += ",\"indexBuildMs\":" + String(liveMs);
      json += ",\"indexLoadMs\":0";
      webLibraryAppendSaveStatusJson(json);
      json += ",\"page\":" + String(page);
      json += ",\"pageSize\":" + String(pageSize);
      json += ",\"total\":" + String(liveTotal);
      json += ",\"typeCounts\":{";
      json += "\"ALL\":" + String(lcAll);
      json += ",\"ATR\":" + String(lcAtr);
      json += ",\"XEX\":" + String(lcXex);
      json += ",\"COM\":" + String(lcCom);
      json += ",\"EXE\":" + String(lcExe);
      json += ",\"BAS\":" + String(lcBas);
      json += ",\"CAS\":" + String(lcCas);
      json += ",\"SEC\":" + String(lcSec);
      json += ",\"OTHER\":" + String(lcOther);
      json += "},\"files\":[";
      json += liveEntries;
      json += "]}";
      server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
      server.sendHeader("Pragma", "no-cache");
      server.sendHeader("Expires", "0");
      server.send(200, "application/json", json);
      return;
    }

    String json = "{\"ok\":true";
    json += ",\"source\":\"library_ndjson_required\"";
    json += ",\"refreshed\":false";
    json += ",\"autoScanDisabled\":true";
    json += ",\"ndjson\":true";
    json += ",\"needsManualRefresh\":true";
    json += ",\"message\":\"Presiona Refrescar biblioteca para escanear la SD y crear /CONFIG/library_index.ndjson\"";
    json += ",\"indexPath\":\"" + jsonEscape(String(WEB_LIBRARY_INDEX_NDJSON_PATH)) + "\"";
    json += ",\"page\":" + String(page);
    json += ",\"pageSize\":" + String(pageSize);
    json += ",\"total\":0";
    json += ",\"typeCounts\":{\"ALL\":0,\"ATR\":0,\"XEX\":0,\"COM\":0,\"EXE\":0,\"BAS\":0,\"CAS\":0,\"SEC\":0,\"OTHER\":0}";
    json += ",\"files\":[]}";
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    server.send(200, "application/json", json);
    return;
  }

  // F42X: para navegacion normal servimos directo desde /CONFIG/library_index.json
  // sin cargar el array completo a RAM. El archivo que queda en SD es la fuente
  // de verdad; si existe y tiene objetos, generamos total/typeCounts/files[] por streaming.
  if (!manualRefresh) {
    if (pageSize > 250) pageSize = 250;
    String directEntries;
    directEntries.reserve(pageSize <= 50 ? 24576 : 32768);
    uint32_t directTotal = 0;
    uint32_t dcAll = 0, dcAtr = 0, dcXex = 0, dcCom = 0, dcExe = 0, dcBas = 0, dcCas = 0, dcSec = 0, dcOther = 0;
    String directSource;
    uint32_t directBytes = 0;
    if (webLibraryStreamIndexBestFilePage(page, pageSize, q, type, directEntries, directTotal,
                                          dcAll, dcAtr, dcXex, dcCom, dcExe, dcBas, dcCas, dcSec, dcOther,
                                          directSource, directBytes)) {
      g_webLibraryIndexCount = dcAll;
      g_webLibraryIndexLoaded = true;
      g_webLibraryIndexDirty = false;
      String json;
      json.reserve(4096 + directEntries.length());
      json = "{\"ok\":true";
      json += ",\"source\":\"library_index_file_direct\"";
      json += ",\"indexSource\":\"" + jsonEscape(directSource) + "\"";
      json += ",\"refreshed\":false";
      json += ",\"refreshFailed\":false";
      json += ",\"indexCommit\":false";
      json += ",\"indexPath\":\"" + jsonEscape(String(WEB_LIBRARY_INDEX_PATH)) + "\"";
      json += ",\"indexCount\":" + String(dcAll);
      json += ",\"indexFileBytes\":" + String(directBytes);
      json += ",\"indexBuildMs\":" + String(g_webLibraryIndexBuildMs);
      json += ",\"indexLoadMs\":" + String(g_webLibraryIndexLoadMs);
      webLibraryAppendSaveStatusJson(json);
      json += ",\"page\":" + String(page);
      json += ",\"pageSize\":" + String(pageSize);
      json += ",\"total\":" + String(directTotal);
      json += ",\"typeCounts\":{";
      json += "\"ALL\":" + String(dcAll);
      json += ",\"ATR\":" + String(dcAtr);
      json += ",\"XEX\":" + String(dcXex);
      json += ",\"COM\":" + String(dcCom);
      json += ",\"EXE\":" + String(dcExe);
      json += ",\"BAS\":" + String(dcBas);
      json += ",\"CAS\":" + String(dcCas);
      json += ",\"SEC\":" + String(dcSec);
      json += ",\"OTHER\":" + String(dcOther);
      json += "}";
      json += ",\"files\":[";
      json += directEntries;
      json += "]}";
      server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
      server.sendHeader("Pragma", "no-cache");
      server.sendHeader("Expires", "0");
      server.send(200, "application/json", json);
      return;
    }
    logf("[LIB-IDX] direct file page fallo; se usa fallback RAM/SD");
  }

  // F42F/F42T: saneamos RAM antes de contar/servir y evitamos construir pageEntries completo.
  // Si la metadata dice que hay archivos pero el String global no sirve, recargamos
  // desde SD/backup antes de responder para evitar total=0 files=[].
  if (!webLibraryIndexRamUsable(true)) {
    logf("[LIB-IDX] serve detecto RAM no usable; recargando indice desde SD");
    webLibraryIndexLoadFromFs();
    webLibraryIndexRamUsable(true);
  }
  if (!webLibraryIndexRamUsable(true)) {
    webLibraryBuildLiveIndexForServe("serve RAM/SD unusable");
  }
  webLibraryNormalizeEntriesJson(g_webLibraryIndexJson);
  uint32_t realObjectCount = webLibraryCountObjects(g_webLibraryIndexJson);
  if (realObjectCount != g_webLibraryIndexCount && realObjectCount > 0) {
    logf("[LIB-IDX] serve count corregido: meta=%u real=%u", (unsigned)g_webLibraryIndexCount, (unsigned)realObjectCount);
    g_webLibraryIndexCount = realObjectCount;
  }
  // F42W: volver a construir files[] para la pagina visible en un String pequeño.
  // F42L/V usaban respuesta chunked; en algunos navegadores/ESP32 se veia total,
  // pero files quedaba vacio o la respuesta se cortaba. Para pageSize normal (50)
  // armamos un JSON completo y lo enviamos con server.send().
  // El listado completo sigue estando paginado para no saturar heap.
  uint32_t totalMatches = 0;
  String pageEntries;
  pageEntries.reserve(pageSize <= 50 ? 24576 : 32768);
  webLibraryAppendPagedEntries(pageEntries, g_webLibraryIndexJson, page, pageSize, q, type, totalMatches);

  if (totalMatches == 0 && realObjectCount > 0 && q.length() == 0 && (type.length() == 0 || type == "ALL")) {
    logf("[LIB-IDX] serve total=0 con realObjectCount=%u; intentando live fallback", (unsigned)realObjectCount);
    if (webLibraryBuildLiveIndexForServe("serve total=0 despite realObjectCount")) {
      totalMatches = 0;
      pageEntries = "";
      webLibraryAppendPagedEntries(pageEntries, g_webLibraryIndexJson, page, pageSize, q, type, totalMatches);
    }
  }

  String json;
  json.reserve(24576 + pageEntries.length());
  json = "{\"ok\":true";
  json += ",\"source\":\"library_index\"";
  json += ",\"refreshed\":" + String(forceRefresh ? "true" : "false");
  json += ",\"refreshFailed\":" + String(refreshFailed ? "true" : "false");
  if (refreshError.length()) json += ",\"refreshError\":\"" + jsonEscape(refreshError) + "\"";
  json += ",\"indexCommit\":" + String(commitRefresh ? "true" : "false");
  json += ",\"indexPath\":\"" + jsonEscape(String(WEB_LIBRARY_INDEX_PATH)) + "\"";
  json += ",\"indexCount\":" + String(g_webLibraryIndexCount);
  json += ",\"indexBuildMs\":" + String(g_webLibraryIndexBuildMs);
  json += ",\"indexLoadMs\":" + String(g_webLibraryIndexLoadMs);
  webLibraryAppendSaveStatusJson(json);
  json += ",\"page\":" + String(page);
  json += ",\"pageSize\":" + String(pageSize);
  json += ",\"total\":" + String(totalMatches);
  webLibraryAppendTypeCountsJson(json, g_webLibraryIndexJson, q);
  json += ",\"files\":[";
  json += pageEntries;
  json += "]}";

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  // F42W: respuesta normal con JSON completo de la pagina visible.
  // No se usa Content-Length manual ni streaming chunked para evitar:
  // - total visible pero files vacio
  // - ERR_INCOMPLETE_CHUNKED_ENCODING
  // - ERR_CONTENT_LENGTH_MISMATCH por cierre de chunk incompleto.
  server.send(200, "application/json", json);

}

// F42R: escaneo de consola seguro y sin structs custom en firmas.
// Motivo: Arduino IDE genera prototipos antes de algunas declaraciones del .ino;
// usando solo contadores globales evitamos errores tipo "WebLibraryConsoleScanStats no declarado".
static uint32_t g_libConsoleScanAll = 0;
static uint32_t g_libConsoleScanAtr = 0;
static uint32_t g_libConsoleScanXex = 0;
static uint32_t g_libConsoleScanCas = 0;
static uint32_t g_libConsoleScanOther = 0;
static uint32_t g_libConsoleScanDirs = 0;
static uint32_t g_libConsoleScanSkippedDirs = 0;

static void webLibraryConsoleScanResetCounters() {
  g_libConsoleScanAll = 0;
  g_libConsoleScanAtr = 0;
  g_libConsoleScanXex = 0;
  g_libConsoleScanCas = 0;
  g_libConsoleScanOther = 0;
  g_libConsoleScanDirs = 0;
  g_libConsoleScanSkippedDirs = 0;
}

static void webLibraryConsoleScanCountType(const String& type) {
  g_libConsoleScanAll++;
  if (type == "ATR") g_libConsoleScanAtr++;
  else if (type == "XEX") g_libConsoleScanXex++;
  else if (type == "CAS") g_libConsoleScanCas++;
  else g_libConsoleScanOther++;
}

static void webLibraryConsoleScanDir(const char* dirPath, uint8_t depth) {
  if (!webAtrFsReady()) {
    logf("[LIB-SD] SKIP dir=%s ready=0", dirPath);
    return;
  }
  if (depth > 4) {
    logf("[LIB-SD] SKIP dir=%s depth=%u maxDepth", dirPath, (unsigned)depth);
    return;
  }

  String baseDir(dirPath ? dirPath : "/");
  baseDir.replace("\\", "/");
  if (!baseDir.length()) baseDir = "/";
  if (!baseDir.startsWith("/")) baseDir = String("/") + baseDir;
  if (webAtrLibraryScanSkipDir(baseDir)) {
    g_libConsoleScanSkippedDirs++;
    logf("[LIB-SD] SKIP dir=%s", baseDir.c_str());
    return;
  }

  logf("[LIB-SD] SCAN depth=%u dir=%s", (unsigned)depth, baseDir.c_str());
  File root = SPIFFS.open(baseDir, "r");
  if (!root) {
    logf("[LIB-SD] OPEN FAIL dir=%s", baseDir.c_str());
    return;
  }
  if (!root.isDirectory()) {
    logf("[LIB-SD] NOT DIR path=%s", baseDir.c_str());
    root.close();
    return;
  }

  File f = root.openNextFile();
  while (f) {
    String path = f.name();
    path.replace("\\", "/");
    bool isDir = f.isDirectory();
    uint32_t fSize = isDir ? 0 : (uint32_t)f.size();
    f.close();

    if (path.length() && !path.startsWith("/")) {
      String prefix = baseDir;
      if (!prefix.endsWith("/")) prefix += "/";
      path = prefix + path;
    }

    if (isDir) {
      if (webAtrLibraryScanSkipDir(path)) {
        g_libConsoleScanSkippedDirs++;
        logf("[LIB-SD] SKIP dir=%s", path.c_str());
      } else {
        g_libConsoleScanDirs++;
        webLibraryConsoleScanDir(path.c_str(), depth + 1);
      }
    } else {
      int slash = path.lastIndexOf('/');
      String rawName = (slash >= 0) ? path.substring(slash + 1) : path;
      String clean = webAtrSanitizeFileName(rawName);
      if (webAtrIsMountableVisibleName(rawName) && clean.length()) {
        String type = webAtrStoredTypeForName(clean);
        webLibraryConsoleScanCountType(type);
        logf("[LIB-SD] FOUND #%lu type=%s name=%s path=%s",
             (unsigned long)g_libConsoleScanAll, type.c_str(), clean.c_str(), path.c_str());
        if ((g_libConsoleScanAll % 20) == 0) delay(1);
      }
    }
    f = root.openNextFile();
  }
  root.close();
}

static void webLibraryConsoleScanRootShallow() {
  logf("[LIB-SD] SCAN shallow dir=/");
  File root = SPIFFS.open("/", "r");
  if (!root) { logf("[LIB-SD] OPEN FAIL shallow dir=/"); return; }
  if (!root.isDirectory()) { logf("[LIB-SD] NOT DIR shallow path=/"); root.close(); return; }
  File f = root.openNextFile();
  while (f) {
    String path = f.name();
    path.replace("\\", "/");
    bool isDir = f.isDirectory();
    uint32_t fSize = isDir ? 0 : (uint32_t)f.size();
    f.close();
    if (!isDir) {
      if (path.length() && !path.startsWith("/")) path = String("/") + path;
      int slash = path.lastIndexOf('/');
      String rawName = (slash >= 0) ? path.substring(slash + 1) : path;
      String clean = webAtrSanitizeFileName(rawName);
      if (webAtrIsMountableVisibleName(rawName) && clean.length()) {
        String type = webAtrStoredTypeForName(clean);
        webLibraryConsoleScanCountType(type);
        logf("[LIB-SD] FOUND #%lu type=%s name=%s path=%s",
             (unsigned long)g_libConsoleScanAll, type.c_str(), clean.c_str(), path.c_str());
      }
    }
    f = root.openNextFile();
  }
  root.close();
}

static void webLibraryConsoleScanKnownDirs() {
  webLibraryConsoleScanResetCounters();
  logf("[LIB-SD] ===== INICIO ESCANEO SD Biblioteca SEGURO =====");
  logf("[LIB-SD] ready=%u backend=%s heap=%lu maxAlloc=%lu",
       (unsigned)webAtrFsReady(), WEB_STORAGE_NAME,
       (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
#if WEB_STORAGE_USE_SD
  webLibraryConsoleScanDir("/ATR", 0);
  webLibraryConsoleScanDir("/CAS", 0);
  webLibraryConsoleScanDir("/LIBRARY", 0);
  webLibraryConsoleScanDir("/SD_CARD_CONTENT/ATR", 0);
  webLibraryConsoleScanDir("/SD_CARD_CONTENT/CAS", 0);
  webLibraryConsoleScanRootShallow();
#endif
  logf("[LIB-SD] ===== FIN ESCANEO SD Biblioteca: total=%lu ATR=%lu XEX=%lu CAS=%lu OTHER=%lu dirs=%lu skipped=%lu heap=%lu =====",
       (unsigned long)g_libConsoleScanAll, (unsigned long)g_libConsoleScanAtr, (unsigned long)g_libConsoleScanXex,
       (unsigned long)g_libConsoleScanCas, (unsigned long)g_libConsoleScanOther, (unsigned long)g_libConsoleScanDirs,
       (unsigned long)g_libConsoleScanSkippedDirs, (unsigned long)ESP.getFreeHeap());
}

void handleApiLibraryScanConsole() {
  if (!webAtrFsReady()) {
    apiSendJsonError(500, "Almacenamiento no disponible");
    return;
  }

  bool save = server.hasArg("commit") && server.arg("commit") == "1";
  webLibraryConsoleScanKnownDirs();
  bool saveOk = false;
  if (save) {
    // F42S: permite validar por consola y luego reconstruir/guardar con el mismo
    // camino seguro de Biblioteca. La respuesta sigue siendo pequeña.
    bool prevConsoleScan = g_webLibraryConsoleScan;
    g_webLibraryConsoleScan = false;
    saveOk = webLibraryIndexRebuild(true, true);
    g_webLibraryConsoleScan = prevConsoleScan;
    logf("[LIB-SD] scan_console commit=1 rebuild/save ok=%u count=%lu err=%s",
         (unsigned)saveOk, (unsigned long)g_webLibraryIndexCount, g_webLibraryIndexLastSaveError.c_str());
  }

  String json = "{\"ok\":true";
  json += ",\"action\":\"scan_console\"";
  json += ",\"build\":\"" + jsonEscape(String(MASTER_BUILD)) + "\"";
  json += ",\"saved\":" + String((save && saveOk) ? "true" : "false");
  json += ",\"commitIgnored\":false";
  json += ",\"scanPrinted\":" + String(g_libConsoleScanAll);
  json += ",\"atr\":" + String(g_libConsoleScanAtr);
  json += ",\"xex\":" + String(g_libConsoleScanXex);
  json += ",\"cas\":" + String(g_libConsoleScanCas);
  json += ",\"other\":" + String(g_libConsoleScanOther);
  json += ",\"skippedDirs\":" + String(g_libConsoleScanSkippedDirs);
  json += ",\"indexCount\":" + String(g_webLibraryIndexCount);
  json += ",\"indexSaved\":" + String(g_webLibraryIndexLastSaveOk ? "true" : "false");
  json += ",\"indexVerifyOk\":" + String(g_webLibraryIndexLastVerifyOk ? "true" : "false");
  json += ",\"indexVerifyCount\":" + String(g_webLibraryIndexLastVerifyCount);
  json += ",\"indexSaveError\":\"" + jsonEscape(g_webLibraryIndexLastSaveError) + "\"";
  json += ",\"message\":\"Revisa el Monitor Serie: lineas [LIB-SD] con carpetas y archivos encontrados\"}";
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send(200, "application/json", json);
}

static bool apiMountWebAtrUnit(int unit, const String& requestedFile, String& err) {
  if (!apiValidateDriveUnit(unit)) { err = "Unidad inválida"; return false; }
  String name = webAtrSanitizeFileName(requestedFile);
  if (name.length() == 0) { err = "Falta archivo"; return false; }
  String path = webAtrPathForName(name);
  WebAtrMeta m;
  if (!webAtrReadMetaFromPath(path, m)) {
    err = String("ATR/XEX no encontrado o inválido: ") + name;
    return false;
  }

  int idx = unit - 1;
  uint8_t bit = (uint8_t)(1u << idx);
  if (idx >= 4) DRIVE_VISIBLE_MASK |= bit;

  // Regla central: WEB-ATR y BT-SIO2PC se excluyen automáticamente por unidad.
  clearBtSio2pcUnits(bit);
  BT_DISK_ENABLED = false;

  webAtrInvalidateFilesCache();
  g_webAtrMountedName[idx] = name;
  g_webAtrMountedPath[idx] = path;
  g_webAtrMountedMeta[idx] = m;
  g_webAtrSlotPresent[idx] = true;
  webAtrCacheInvalidateDev((uint8_t)(0x31 + idx));
  WEB_ATR_ENABLED = true;
  WEB_ATR_DEV_MASK |= bit;
  WEB_ATR_FORCE_MASK |= bit;
  apiPersistStorageConfig();
  logf("[ANDROID-API] mount %s file=%s", devName(0x31 + idx), name.c_str());
  return true;
}

static bool apiUnmountWebAtrUnit(int unit, String& err) {
  if (!apiValidateDriveUnit(unit)) { err = "Unidad inválida"; return false; }
  int idx = unit - 1;
  uint8_t bit = (uint8_t)(1u << idx);
  webAtrInvalidateFilesCache();
  g_webAtrMountedName[idx] = "";
  webAtrResetResolvedSlot(idx);
  webAtrCacheInvalidateDev((uint8_t)(0x31 + idx));
  WEB_ATR_DEV_MASK &= (uint8_t)~bit;
  WEB_ATR_FORCE_MASK &= (uint8_t)~bit;
  if ((WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK) == 0) WEB_ATR_ENABLED = false;
  apiPersistStorageConfig();
  logf("[ANDROID-API] unmount %s", devName(0x31 + idx));
  return true;
}

static bool apiEnableBtSio2pcUnit(int unit, String& err) {
  if (!apiValidateDriveUnit(unit)) { err = "Unidad inválida"; return false; }
  int idx = unit - 1;
  uint8_t bit = (uint8_t)(1u << idx);
  if (idx >= 4) DRIVE_VISIBLE_MASK |= bit;

  // Regla central: si BT-SIO2PC toma la unidad, WEB-ATR se desmonta en esa unidad.
  clearWebAtrUnits(bit, true);
  BT_DISK_ENABLED = false;
  BT_SIO2PC_ENABLED = true;
  BT_SIO2PC_DEV_MASK |= bit;
  BT_SIO2PC_FORCE_MASK |= bit;
  btSio2pcCacheClear();
  apiPersistStorageConfig();
  logf("[ANDROID-API] bt enable %s", devName(0x31 + idx));
  return true;
}

static bool apiDisableBtSio2pcUnit(int unit, String& err) {
  if (!apiValidateDriveUnit(unit)) { err = "Unidad inválida"; return false; }
  int idx = unit - 1;
  uint8_t bit = (uint8_t)(1u << idx);
  BT_SIO2PC_DEV_MASK &= (uint8_t)~bit;
  BT_SIO2PC_FORCE_MASK &= (uint8_t)~bit;
  if ((BT_SIO2PC_DEV_MASK & DRIVE_UI_MAX_MASK) == 0) BT_SIO2PC_ENABLED = false;
  btSio2pcCacheClear();
  apiPersistStorageConfig();
  logf("[ANDROID-API] bt disable %s", devName(0x31 + idx));
  return true;
}

// [F24] Eliminado helper no usado: handleApiDriveMountUnit
// [F24] Eliminado helper no usado: handleApiDriveUnmountUnit
// [F24] Eliminado helper no usado: handleApiDriveBtEnableUnit
// [F24] Eliminado helper no usado: handleApiDriveBtDisableUnit
// [F24] Eliminado helper no usado: handleApiDriveRemoveUnit
void handleApiDriveAdd() {
  int u = nextHiddenOptionalDriveUnit();
  if (!u) {
    apiSendJsonError(409, "Ya están visibles D1-D7");
    return;
  }
  DRIVE_VISIBLE_MASK |= (uint8_t)(1u << (u - 1));
  apiPersistStorageConfig();
  logf("[ANDROID-API] add D%d", u);
  handleApiDrives();
}


static void handleApiDynamicRouter() {
  String uri = server.uri();
  if (!uri.startsWith("/api/drives/")) {
    server.send(404, "application/json", "{\"ok\":false,\"error\":\"not found\"}");
    return;
  }
  if (server.method() != HTTP_POST) {
    apiSendJsonError(405, "Método no permitido");
    return;
  }

  String rest = uri.substring(12); // after /api/drives/
  int p = rest.indexOf('/');
  if (p < 2 || rest[0] != 'D') {
    apiSendJsonError(400, "Ruta de unidad inválida");
    return;
  }

  int unit = rest.substring(1, p).toInt();
  String action = rest.substring(p + 1);
  String err;

  if (action == "mount") {
    String body = server.hasArg("plain") ? server.arg("plain") : String("");
    String file = server.hasArg("file") ? server.arg("file") : apiJsonStringField(body, "file");
    if (!apiMountWebAtrUnit(unit, file, err)) { apiSendJsonError(400, err); return; }
  } else if (action == "unmount") {
    if (!apiUnmountWebAtrUnit(unit, err)) { apiSendJsonError(400, err); return; }
  } else if (action == "bt/enable") {
    if (!apiEnableBtSio2pcUnit(unit, err)) { apiSendJsonError(400, err); return; }
  } else if (action == "bt/disable") {
    if (!apiDisableBtSio2pcUnit(unit, err)) { apiSendJsonError(400, err); return; }
  } else if (action == "remove") {
    if (unit < 5 || unit > DRIVE_UI_MAX_UNITS) { apiSendJsonError(400, "Solo D5-D7 se pueden eliminar"); return; }
    int idx = unit - 1;
    if (!driveUiVisibleIndex(idx)) { apiSendJsonError(409, String("D") + String(unit) + " no está visible"); return; }
    if (optionalDriveBusyIndex(idx)) { apiSendJsonError(409, String("Primero desmonta/desactiva D") + String(unit)); return; }
    DRIVE_VISIBLE_MASK &= (uint8_t)~(1u << idx);
    apiPersistStorageConfig();
    logf("[API] remove D%d", unit);
  } else {
    apiSendJsonError(404, "Acción no encontrada");
    return;
  }
  handleApiDrives();
}




struct LibretroCoverEntry {
  const char* key;
  const char* title;
  const char* image;
};

static const LibretroCoverEntry LIBRETRO_COVERS[] PROGMEM = {
  { "montezuma revenge", "Montezuma's Revenge", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Montezuma%27s%20Revenge%20%281984%29%28Parker%20Brothers%29%28US%29.png" },
  { "moon patrol", "Moon Patrol", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Moon%20Patrol%20%281983%29%28Atari%29%28US%29.png" },
  { "buck rodgers", "Buck Rodgers", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Buck%20Rodgers%20%281983%29%28Sega%29%28US%29.png" },
  { "buck rogers", "Buck Rogers", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Buck%20Rogers%20%281983%29%28SEGA%29%28US%29.png" },
  { "draconus", "Draconus", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Draconus%20%281988%29%28Cognito%29%28GB%29.png" },
  { "hero", "H.E.R.O.", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/H.E.R.O.%20%281984%29%28Activision%29%28US%29.png" },
  { "h e r o", "H.E.R.O.", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/H.E.R.O.%20%281984%29%28Activision%29%28US%29.png" },
  { "pitstop", "Pitstop", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Pitstop%20%281983%29%28Epyx%29%28US%29.png" },
  { "pitstop ii", "Pitstop II", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Pitstop%20II%20%281984%29%28Epyx%29%28US%29.png" },
  { "zybex", "Zybex", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Zybex%20%281988%29%28Zeppelin%20Games%29%28GB%29.png" },
  { "zorro", "Zorro", "https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/Zorro%20%281985%29%28Datasoft%29%28US%29.png" }
};

static String coverNormalizeKey(String v) {
  v.toLowerCase();
  v.replace("atr_", " ");
  v.replace(".atr", " ");
  v.replace(".xex", " ");
  v.replace(".com", " ");
  v.replace(".exe", " ");
  v.replace("_", " ");
  v.replace("-", " ");
  v.replace(".", " ");

  String out;
  out.reserve(v.length());
  bool lastSpace = true;
  for (size_t i = 0; i < v.length(); i++) {
    char c = v[i];
    bool keep = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    if (keep) {
      out += c;
      lastSpace = false;
    } else {
      if (!lastSpace) {
        out += ' ';
        lastSpace = true;
      }
    }
  }
  out.trim();

  // Limpieza específica para nombres de archivos ATR comunes.
  out.replace(" arcade", "");
  out.replace(" disk", "");
  out.replace(" disc", "");
  out.replace(" side a", "");
  out.replace(" side b", "");
  out.replace(" atari", "");
  out.replace(" xe", "");
  out.replace(" xl", "");
  out.replace("  ", " ");
  out.trim();

  if (out == "montezuma" || out == "montezumas revenge") out = "montezuma revenge";
  if (out == "moon patrol arcade") out = "moon patrol";
  if (out == "zorros") out = "zorro";
  if (out == "h e r o" || out == "h e r o ") out = "hero";

  return out;
}

static String coverJsonEntry(const String& key, const String& title, const String& image) {
  String j = "{";
  j += "\"ok\":1";
  j += ",\"key\":\"" + jsonEscape(key) + "\"";
  j += ",\"title\":\"" + jsonEscape(title) + "\"";
  j += ",\"image\":\"" + jsonEscape(image) + "\"";
  j += ",\"page\":\"https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/\"";
  j += ",\"source\":\"Libretro Named_Boxarts\"";
  j += "}";
  return j;
}

static bool coverFindLibretro(const String& query, String& outKey, String& outTitle, String& outImage) {
  String q = coverNormalizeKey(query);
  if (!q.length()) return false;

  const size_t n = sizeof(LIBRETRO_COVERS) / sizeof(LIBRETRO_COVERS[0]);
  for (size_t i = 0; i < n; i++) {
    String key = String((const char*)pgm_read_ptr(&LIBRETRO_COVERS[i].key));
    if (q == key || q.indexOf(key) >= 0 || key.indexOf(q) >= 0) {
      outKey = key;
      outTitle = String((const char*)pgm_read_ptr(&LIBRETRO_COVERS[i].title));
      outImage = String((const char*)pgm_read_ptr(&LIBRETRO_COVERS[i].image));
      return true;
    }
  }
  return false;
}


// ===== Shared manual covers for Web + Android =====
// Storage format: /covers_user.json
// {
//   "items": {
//     "zorro": {"title":"Zorro","image":"https://...png","source":"manual"}
//   }
// }

static const char* USER_COVERS_PATH = "/covers_user.json";

static const char* USER_COVERS_DIR = "/COVERS";
static const char* USER_MINI_COVERS_DIR = "/MINI_COVERS";
static File g_coverMiniUploadFile;
static String g_coverMiniUploadTmpPath;
static String g_coverMiniUploadFinalPath;
static String g_coverMiniUploadKey;
static String g_coverMiniUploadError;
static File g_coverUploadFile;
static String g_coverUploadTmpPath;
static String g_coverUploadFinalPath;
static String g_coverUploadName;
static String g_coverUploadTitle;
static String g_coverUploadKey;
static String g_coverUploadError;
static String g_coverUploadContentType;

static String coverSanitizeKeyForFile(String key) {
  key = coverNormalizeKey(key);
  if (!key.length()) key = "cover";
  String out;
  out.reserve(key.length() + 8);
  for (size_t i = 0; i < key.length(); i++) {
    char c = key[i];
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) out += c;
    else if (c == ' ' || c == '_' || c == '-') out += '_';
  }
  while (out.indexOf("__") >= 0) out.replace("__", "_");
  out.trim();
  if (!out.length()) out = "cover";
  return out;
}

static bool coverEnsureDir() {
  if (!g_fontFsReady && !SPIFFS.begin(true)) return false;
  if (!SPIFFS.exists(USER_COVERS_DIR)) SPIFFS.mkdir(USER_COVERS_DIR);
  if (!SPIFFS.exists(USER_MINI_COVERS_DIR)) SPIFFS.mkdir(USER_MINI_COVERS_DIR);
  return SPIFFS.exists(USER_COVERS_DIR);
}

static bool coverEnsureMiniDir() {
  if (!g_fontFsReady && !SPIFFS.begin(true)) return false;
  if (!SPIFFS.exists(USER_MINI_COVERS_DIR)) SPIFFS.mkdir(USER_MINI_COVERS_DIR);
  return SPIFFS.exists(USER_MINI_COVERS_DIR);
}

static String coverExtFromMimeOrName(String mime, String name) {
  mime.toLowerCase();
  name.toLowerCase();
  if (mime.indexOf("jpeg") >= 0 || mime.indexOf("jpg") >= 0 || name.endsWith(".jpg") || name.endsWith(".jpeg")) return ".jpg";
  if (mime.indexOf("webp") >= 0 || name.endsWith(".webp")) return ".webp";
  if (mime.indexOf("gif") >= 0 || name.endsWith(".gif")) return ".gif";
  if (mime.indexOf("bmp") >= 0 || name.endsWith(".bmp")) return ".bmp";
  return ".png";
}

static String coverContentTypeFromPath(const String& path) {
  String p = path;
  p.toLowerCase();
  if (p.endsWith(".jpg") || p.endsWith(".jpeg")) return "image/jpeg";
  if (p.endsWith(".webp")) return "image/webp";
  if (p.endsWith(".gif")) return "image/gif";
  if (p.endsWith(".bmp")) return "image/bmp";
  return "image/png";
}

static bool coverLocalFindPathForKey(const String& keyIn, String& outPath) {
  String base = coverSanitizeKeyForFile(keyIn);
  const char* exts[] = { ".png", ".jpg", ".jpeg", ".webp", ".gif", ".bmp" };
  for (size_t i = 0; i < sizeof(exts)/sizeof(exts[0]); i++) {
    String p = String(USER_COVERS_DIR) + "/" + base + exts[i];
    if (SPIFFS.exists(p)) { outPath = p; return true; }
  }
  return false;
}

static bool coverIsLocalPath(String path);

static bool coverMiniFindPathForKey(const String& keyIn, String& outPath) {
  String base = coverSanitizeKeyForFile(keyIn);
  const char* exts[] = { ".jpg", ".jpeg", ".webp", ".png" };
  for (size_t i = 0; i < sizeof(exts)/sizeof(exts[0]); i++) {
    String p = String(USER_MINI_COVERS_DIR) + "/" + base + exts[i];
    if (SPIFFS.exists(p)) { outPath = p; return true; }
  }
  return false;
}

static bool coverIsMiniLocalPath(String path) {
  path.trim();
  path.replace("\\", "/");
  return path.startsWith(String(USER_MINI_COVERS_DIR) + "/");
}

static void coverAddImageCacheHeaders(uint32_t maxAgeSeconds = 604800UL) {
  // F49Z41: carátulas locales son archivos versionados por nombre; mantenerlas cacheadas
  // baja mucho el tráfico HTTP al navegar Biblioteca/Discos.
  server.sendHeader("Cache-Control", String("public, max-age=") + String(maxAgeSeconds) + ", immutable");
  server.sendHeader("Connection", "close");
}

static void coverSendLocalNotFound(const char* reason = "NO_LOCAL_COVER") {
  // F49Z41: cachear brevemente misses de miniaturas evita cientos de 404 repetidos
  // cuando la Biblioteca tiene muchos títulos sin carátula local.
  server.sendHeader("Cache-Control", "public, max-age=300");
  server.sendHeader("Connection", "close");
  server.send(404, "text/plain", reason ? reason : "NO_LOCAL_COVER");
}

static bool coverStreamAnyLocalPath(String path) {
  path.trim();
  path.replace("\\", "/");
  if (!(coverIsLocalPath(path) || coverIsMiniLocalPath(path)) || !SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  coverAddImageCacheHeaders();
  server.streamFile(f, coverContentTypeFromPath(path));
  f.close();
  return true;
}

static void coverRemoveMiniFilesForKey(const String& keyIn) {
  String base = coverSanitizeKeyForFile(keyIn);
  const char* exts[] = { ".jpg", ".jpeg", ".webp", ".png", ".tmp" };
  for (size_t i = 0; i < sizeof(exts)/sizeof(exts[0]); i++) {
    String p = String(USER_MINI_COVERS_DIR) + "/" + base + exts[i];
    if (SPIFFS.exists(p)) SPIFFS.remove(p);
  }
}

static bool coverIsLocalPath(String path) {
  path.trim();
  path.replace("\\", "/");
  return path.startsWith(String(USER_COVERS_DIR) + "/");
}

static bool coverStreamLocalPath(String path) {
  path.trim();
  path.replace("\\", "/");
  if (!coverIsLocalPath(path) || !SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  coverAddImageCacheHeaders();
  server.streamFile(f, coverContentTypeFromPath(path));
  f.close();
  return true;
}

static void coverRemoveLocalFilesForKey(const String& keyIn) {
  coverRemoveMiniFilesForKey(keyIn);
  String base = coverSanitizeKeyForFile(keyIn);
  const char* exts[] = { ".png", ".jpg", ".jpeg", ".webp", ".gif", ".bmp", ".tmp" };
  for (size_t i = 0; i < sizeof(exts)/sizeof(exts[0]); i++) {
    String p = String(USER_COVERS_DIR) + "/" + base + exts[i];
    if (SPIFFS.exists(p)) SPIFFS.remove(p);
  }
}

static bool coverDownloadUrlToSd(const String& keyIn, const String& urlIn, String& outLocalPath, String& outContentType, String& err) {
  if (!coverEnsureDir()) { err = "COVERS_DIR_NOT_READY"; return false; }
  String url = urlIn;
  url.trim();
  url.replace(" ", "%20");
  if (!(url.startsWith("http://") || url.startsWith("https://"))) { err = "INVALID_URL"; return false; }
  if (WiFi.status() != WL_CONNECTED) { err = "STA_WIFI_NOT_CONNECTED"; return false; }

  HTTPClient http;
  WiFiClientSecure secureClient;
  WiFiClient plainClient;
  bool https = url.startsWith("https://");
  if (https) secureClient.setInsecure();
  bool begun = https ? http.begin(secureClient, url) : http.begin(plainClient, url);
  if (!begun) { err = "HTTP_BEGIN_FAILED"; return false; }

  http.setTimeout(7000); // F49Z41: no bloquear web por carátulas remotas lentas
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    err = String("HTTP_GET_FAILED_") + String(code);
    http.end();
    return false;
  }

  String contentType = http.header("Content-Type");
  if (!contentType.length()) contentType = "image/png";
  int len = http.getSize();
  const int MAX_COVER_BYTES = 2 * 1024 * 1024;
  if (len > MAX_COVER_BYTES) {
    err = "IMAGE_TOO_LARGE";
    http.end();
    return false;
  }

  String base = coverSanitizeKeyForFile(keyIn);
  String ext = coverExtFromMimeOrName(contentType, url);
  String tmpPath = String(USER_COVERS_DIR) + "/" + base + ".tmp";
  String finalPath = String(USER_COVERS_DIR) + "/" + base + ext;

  coverRemoveLocalFilesForKey(keyIn);
  if (SPIFFS.exists(tmpPath)) SPIFFS.remove(tmpPath);
  File f = SPIFFS.open(tmpPath, "w");
  if (!f) {
    err = "OPEN_TMP_FAILED";
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[1024];
  int remaining = len;
  int total = 0;
  unsigned long lastData = millis();
  bool ok = true;

  while (http.connected()) {
    size_t avail = stream->available();
    if (avail) {
      size_t n = avail;
      if (n > sizeof(buf)) n = sizeof(buf);
      if (remaining > 0 && (int)n > remaining) n = remaining;
      int r = stream->readBytes(buf, n);
      if (r > 0) {
        size_t w = f.write(buf, r);
        if (w != (size_t)r) { err = "WRITE_FAILED"; ok = false; break; }
        total += r;
        if (total > MAX_COVER_BYTES) { err = "IMAGE_TOO_LARGE"; ok = false; break; }
        lastData = millis();
        if (remaining > 0) {
          remaining -= r;
          if (remaining <= 0) break;
        }
      }
    } else {
      if (remaining == 0) break;
      if (millis() - lastData > 7000) { err = "HTTP_TIMEOUT"; ok = false; break; }
      delay(1);
    }
  }

  f.close();
  http.end();

  if (!ok || total <= 0) {
    if (!err.length()) err = "EMPTY_IMAGE";
    if (SPIFFS.exists(tmpPath)) SPIFFS.remove(tmpPath);
    return false;
  }

  if (SPIFFS.exists(finalPath)) SPIFFS.remove(finalPath);
  if (!SPIFFS.rename(tmpPath, finalPath)) {
    // Fallback copy/remove para algunos FATFS que fallan con rename.
    File in = SPIFFS.open(tmpPath, "r");
    File out = SPIFFS.open(finalPath, "w");
    if (!in || !out) {
      if (in) in.close();
      if (out) out.close();
      err = "RENAME_FAILED";
      if (SPIFFS.exists(tmpPath)) SPIFFS.remove(tmpPath);
      return false;
    }
    while (in.available()) {
      size_t r = in.read(buf, sizeof(buf));
      if (r && out.write(buf, r) != r) { ok = false; err = "COPY_FAILED"; break; }
    }
    in.close(); out.close();
    SPIFFS.remove(tmpPath);
    if (!ok) { if (SPIFFS.exists(finalPath)) SPIFFS.remove(finalPath); return false; }
  }

  outLocalPath = finalPath;
  outContentType = contentType;
  logf("[COVERS] cached key=%s path=%s bytes=%d type=%s", coverSanitizeKeyForFile(keyIn).c_str(), finalPath.c_str(), total, contentType.c_str());
  return true;
}

static String coverUserEmptyJson() {
  return "{\"items\":{}}";
}

static String coverUserLoadJson() {
  if (!SPIFFS.exists(USER_COVERS_PATH)) return coverUserEmptyJson();
  File f = SPIFFS.open(USER_COVERS_PATH, "r");
  if (!f) return coverUserEmptyJson();
  String body = f.readString();
  f.close();
  body.trim();
  if (!body.length()) return coverUserEmptyJson();
  return body;
}

static bool coverUserSaveJson(const String& body) {
  File f = SPIFFS.open(USER_COVERS_PATH, "w");
  if (!f) return false;
  size_t w = f.print(body);
  f.close();
  return w == body.length();
}

static int coverUserFindKeyPos(const String& body, const String& key) {
  String needle = "\"" + key + "\"";
  return body.indexOf(needle);
}

static bool coverUserResolve(const String& name, String& outKey, String& outTitle, String& outImage) {
  String key = coverNormalizeKey(name);
  if (!key.length()) return false;

  String body = coverUserLoadJson();
  int p = coverUserFindKeyPos(body, key);
  if (p < 0) return false;

  int objStart = body.indexOf('{', p);
  if (objStart < 0) return false;
  int objEnd = body.indexOf('}', objStart);
  if (objEnd < 0) return false;

  String obj = body.substring(objStart, objEnd + 1);

  auto getJsonStringField = [](const String& objIn, const char* field) -> String {
    String needle = String("\"") + field + "\":\"";
    int a = objIn.indexOf(needle);
    if (a < 0) return "";
    a += needle.length();
    int b = a;
    String out;
    bool esc = false;
    while (b < (int)objIn.length()) {
      char c = objIn[b++];
      if (esc) {
        out += c;
        esc = false;
      } else if (c == '\\') {
        esc = true;
      } else if (c == '"') {
        break;
      } else {
        out += c;
      }
    }
    return out;
  };

  outKey = key;
  outTitle = getJsonStringField(obj, "title");
  outImage = getJsonStringField(obj, "image");
  if (!outTitle.length()) outTitle = key;
  return outImage.length() > 0;
}

static String coverUserUpsertJson(const String& bodyIn, const String& key, const String& title, const String& image) {
  String body = bodyIn;
  body.trim();
  if (!body.length() || body.indexOf("\"items\"") < 0) body = coverUserEmptyJson();

  String entry = "\"" + jsonEscape(key) + "\":{";
  entry += "\"title\":\"" + jsonEscape(title.length() ? title : key) + "\",";
  entry += "\"image\":\"" + jsonEscape(image) + "\",";
  entry += "\"source\":\"manual\",";
  entry += "\"savedAt\":\"" + String(millis()) + "\"";
  entry += "}";

  int existing = coverUserFindKeyPos(body, key);
  if (existing >= 0) {
    int objStart = body.indexOf('{', existing);
    int objEnd = body.indexOf('}', objStart);
    if (objStart >= 0 && objEnd > objStart) {
      int keyStart = body.lastIndexOf('"', existing - 1);
      if (keyStart < 0) keyStart = existing;
      return body.substring(0, keyStart) + entry + body.substring(objEnd + 1);
    }
  }

  int itemsStart = body.indexOf("\"items\"");
  int braceStart = body.indexOf('{', itemsStart);
  int braceEnd = body.lastIndexOf('}');
  int itemsEnd = body.lastIndexOf('}', braceEnd - 1);
  if (braceStart < 0 || itemsEnd < braceStart) return "{\"items\":{" + entry + "}}";

  String inside = body.substring(braceStart + 1, itemsEnd);
  inside.trim();

  String out = body.substring(0, braceStart + 1);
  if (inside.length()) out += inside + ",";
  out += entry;
  out += body.substring(itemsEnd);
  return out;
}

static String coverUserDeleteJson(const String& bodyIn, const String& key) {
  String body = bodyIn;
  int existing = coverUserFindKeyPos(body, key);
  if (existing < 0) return body;

  int keyStart = body.lastIndexOf('"', existing - 1);
  if (keyStart < 0) keyStart = existing;
  int objStart = body.indexOf('{', existing);
  int objEnd = body.indexOf('}', objStart);
  if (objStart < 0 || objEnd < 0) return body;

  int delStart = keyStart;
  int delEnd = objEnd + 1;

  // Remove a leading comma if present, otherwise trailing comma.
  int scan = delStart - 1;
  while (scan >= 0 && isspace((unsigned char)body[scan])) scan--;
  if (scan >= 0 && body[scan] == ',') {
    delStart = scan;
  } else {
    scan = delEnd;
    while (scan < (int)body.length() && isspace((unsigned char)body[scan])) scan++;
    if (scan < (int)body.length() && body[scan] == ',') {
      delEnd = scan + 1;
    }
  }

  return body.substring(0, delStart) + body.substring(delEnd);
}

void handleApiCoversUser() {
  String body = coverUserLoadJson();
  server.send(200, "application/json", body);
}

void handleApiCoverSet() {
  String name = server.hasArg("name") ? server.arg("name") : server.arg("key");
  String url  = server.hasArg("url") ? server.arg("url") : server.arg("image");
  String title = server.hasArg("title") ? server.arg("title") : name;

  name.trim();
  url.trim();
  title.trim();

  String key = coverNormalizeKey(name);
  if (!key.length() || !url.length()) {
    server.send(400, "application/json", "{\"ok\":0,\"reason\":\"MISSING_NAME_OR_URL\"}");
    return;
  }

  String imageToStore = url;
  String sourceToStore = "manual";
  String contentType;
  String cacheErr;

  if (coverIsLocalPath(url)) {
    if (!SPIFFS.exists(url)) {
      server.send(404, "application/json", "{\"ok\":0,\"reason\":\"LOCAL_COVER_NOT_FOUND\"}");
      return;
    }
    imageToStore = url;
    sourceToStore = "manual-local";
  } else {
    if (!(url.startsWith("http://") || url.startsWith("https://"))) {
      server.send(400, "application/json", "{\"ok\":0,\"reason\":\"INVALID_URL\"}");
      return;
    }
    String localPath;
    if (!coverDownloadUrlToSd(key, url, localPath, contentType, cacheErr)) {
      // F49Z37: no fallar con HTTP 502 cuando el ESP32 no puede descargar/cachear
      // la imagen remota. Se guarda igualmente la URL manual en covers_user.json
      // para que no quede solo en el navegador. La cache SD se podrá intentar después.
      imageToStore = url;
      sourceToStore = "manual-remote";
      logf("[COVERS] manual url saved without SD cache key=%s err=%s", key.c_str(), cacheErr.c_str());
    } else {
      imageToStore = localPath;
      sourceToStore = "manual-sd";
    }
  }

  String body = coverUserLoadJson();
  String next = coverUserUpsertJson(body, key, title, imageToStore);
  bool ok = coverUserSaveJson(next);

  String j = "{";
  j += "\"ok\":" + String(ok ? 1 : 0);
  j += ",\"key\":\"" + jsonEscape(key) + "\"";
  j += ",\"title\":\"" + jsonEscape(title.length() ? title : key) + "\"";
  j += ",\"image\":\"" + jsonEscape(imageToStore) + "\"";
  j += ",\"source\":\"" + jsonEscape(sourceToStore) + "\"";
  j += ",\"cached\":" + String(coverIsLocalPath(imageToStore) ? 1 : 0);
  if (contentType.length()) j += ",\"contentType\":\"" + jsonEscape(contentType) + "\"";
  if (sourceToStore == "manual-remote" && cacheErr.length()) {
    j += ",\"cacheWarning\":1";
    j += ",\"cacheError\":\"" + jsonEscape(cacheErr) + "\"";
  }
  if (!ok) j += ",\"reason\":\"WRITE_FAILED\"";
  j += "}";
  server.send(ok ? 200 : 500, "application/json", j);
}

void handleApiCoverDelete() {
  String name = server.hasArg("name") ? server.arg("name") : server.arg("key");
  String key = coverNormalizeKey(name);
  if (!key.length()) {
    server.send(400, "application/json", "{\"ok\":0,\"reason\":\"MISSING_NAME\"}");
    return;
  }

  String body = coverUserLoadJson();
  String next = coverUserDeleteJson(body, key);
  bool ok = coverUserSaveJson(next);
  coverRemoveLocalFilesForKey(key);

  String j = "{";
  j += "\"ok\":" + String(ok ? 1 : 0);
  j += ",\"key\":\"" + jsonEscape(key) + "\"";
  j += ",\"localDeleted\":1";
  if (!ok) j += ",\"reason\":\"WRITE_FAILED\"";
  j += "}";
  server.send(ok ? 200 : 500, "application/json", j);
}


void handleCoverCatalog() {
  const char* path = "/covers_libretro.json";
  if (g_fontFsReady && SPIFFS.exists(path)) {
    File f = SPIFFS.open(path, "r");
    if (f) {
      server.streamFile(f, "application/json");
      f.close();
      return;
    }
  }
  server.send(200, "application/json", "{\"source\":\"Libretro Atari - 8-bit Named_Boxarts\",\"items\":{},\"aliases\":{}}");
}

void handleApiCoverResolve() {
  String name = server.hasArg("name") ? server.arg("name") : server.arg("q");
  
  String keyUser, titleUser, imageUser;
  if (coverUserResolve(name, keyUser, titleUser, imageUser)) {
    String j = coverJsonEntry(keyUser, titleUser, imageUser);
    j.replace("\"source\":\"Libretro Named_Boxarts\"", "\"source\":\"manual\"");
    server.send(200, "application/json", j);
    return;
  }

String key, title, image;
  if (coverFindLibretro(name, key, title, image)) {
    server.send(200, "application/json", coverJsonEntry(key, title, image));
    return;
  }

  String j = "{";
  j += "\"ok\":0";
  j += ",\"key\":\"" + jsonEscape(coverNormalizeKey(name)) + "\"";
  j += ",\"reason\":\"NOT_FOUND\"";
  j += ",\"source\":\"Libretro Named_Boxarts\"";
  j += ",\"page\":\"https://thumbnails.libretro.com/Atari%20-%208-bit/Named_Boxarts/\"";
  j += "}";
  server.send(200, "application/json", j);
}

void handleApiCoverSearch() {
  String q = server.hasArg("q") ? server.arg("q") : server.arg("name");
  String norm = coverNormalizeKey(q);
  String j = "{";
  j += "\"ok\":1";
  j += ",\"q\":\"" + jsonEscape(q) + "\"";
  j += ",\"key\":\"" + jsonEscape(norm) + "\"";
  j += ",\"source\":\"Libretro Named_Boxarts\"";
  j += ",\"items\":[";

  bool first = true;
  const size_t n = sizeof(LIBRETRO_COVERS) / sizeof(LIBRETRO_COVERS[0]);
  for (size_t i = 0; i < n; i++) {
    String key = String((const char*)pgm_read_ptr(&LIBRETRO_COVERS[i].key));
    if (norm.length() == 0 || key.indexOf(norm) >= 0 || norm.indexOf(key) >= 0) {
      String title = String((const char*)pgm_read_ptr(&LIBRETRO_COVERS[i].title));
      String image = String((const char*)pgm_read_ptr(&LIBRETRO_COVERS[i].image));
      if (!first) j += ",";
      first = false;
      j += coverJsonEntry(key, title, image);
    }
  }
  j += "]}";
  server.send(200, "application/json", j);
}


// ===== F18 TNFS BROWSER INITIAL =====
// Pantalla estilo FujiNet para navegar servidores TNFS desde la web del MASTER.
// Implementa solo listado/navegación de directorios. Montaje/carga remota queda para etapa siguiente.
static const uint16_t TNFS_DEFAULT_PORT = 16384;
static const uint16_t TNFS_TIMEOUT_MS = 1400;
static const uint8_t  TNFS_MAX_RETRIES = 2;
static const uint8_t  TNFS_CMD_MOUNT = 0x00;
static const uint8_t  TNFS_CMD_UMOUNT = 0x01;
static const uint8_t  TNFS_CMD_OPENDIR = 0x10;
static const uint8_t  TNFS_CMD_READDIR = 0x11;
static const uint8_t  TNFS_CMD_CLOSEDIR = 0x12;
static const uint8_t  TNFS_CMD_READ = 0x21;
static const uint8_t  TNFS_CMD_CLOSE = 0x23;
static const uint8_t  TNFS_CMD_OPEN = 0x29;
static const uint8_t  TNFS_ERR_EOF = 0x21;
static const uint16_t TNFS_OPEN_RDONLY = 0x0001;
static const uint16_t TNFS_READ_CHUNK = 512;
static const uint32_t TNFS_DOWNLOAD_MAX_BYTES = 16UL * 1024UL * 1024UL;

static uint8_t g_tnfsSeq = 1;

static const char* TNFS_DEFAULT_URLS[] = {
  "tnfs://tnfs.fujinet.online/",
  "tnfs://apps.irata.online/",
  "tnfs://fujinet.abbuc.de/",
  "tnfs://fujinet.diller.org/",
  "tnfs://fujinet.pl/",
  "tnfs://ec.tnfs.io/",
  "tnfs://atarionline.eu/",
  "tnfs://fujinet.skdev.org/",
  "tnfs://fuji.buriedbits.org/"
};

static String tnfsStatusText(uint8_t code) {
  if (code == 0x00) return F("OK");
  if (code == TNFS_ERR_EOF) return F("EOF");
  if (code == 0x1F) return F("TNFS_ERROR");
  if (code == 0x20) return F("NOT_FOUND_OR_INVALID");
  if (code == 0x22) return F("NO_MORE_HANDLES");
  char tmp[12];
  snprintf(tmp, sizeof(tmp), "0x%02X", code);
  return String(tmp);
}

static bool tnfsParseUrl(String in, TnfsParsedUrl& out) {
  in.trim();
  if (!in.length()) return false;
  in.replace("\\", "/");
  if (in.startsWith("tnfs://")) in = in.substring(7);
  else if (in.startsWith("TNFS://")) in = in.substring(7);
  else if (in.startsWith("http://")) in = in.substring(7);
  else if (in.startsWith("https://")) in = in.substring(8);
  while (in.startsWith("/")) in = in.substring(1);

  int slash = in.indexOf('/');
  String hostPort = slash >= 0 ? in.substring(0, slash) : in;
  String path = slash >= 0 ? in.substring(slash) : String("/");
  hostPort.trim();
  path.trim();
  if (!hostPort.length()) return false;
  if (!path.length()) path = "/";
  if (!path.startsWith("/")) path = "/" + path;

  uint16_t port = TNFS_DEFAULT_PORT;
  int colon = hostPort.lastIndexOf(':');
  if (colon > 0) {
    String ps = hostPort.substring(colon + 1);
    int pi = ps.toInt();
    if (pi > 0 && pi <= 65535) {
      port = (uint16_t)pi;
      hostPort = hostPort.substring(0, colon);
    }
  }
  hostPort.trim();
  if (!hostPort.length()) return false;
  out.host = hostPort;
  out.path = path;
  out.port = port;
  out.raw = String("tnfs://") + out.host + (out.port == TNFS_DEFAULT_PORT ? String("") : String(":") + String(out.port)) + out.path;
  return true;
}

static String tnfsCustomUrlsLoad() {
  String v;
  if (prefs.begin("tnfsui", true)) {
    v = prefs.getString("hosts", "");
    prefs.end();
  }
  return v;
}

static void tnfsCustomUrlsSave(const String& v) {
  if (prefs.begin("tnfsui", false)) {
    prefs.putString("hosts", v);
    prefs.end();
  }
}

static String tnfsHiddenUrlsLoad() {
  String v;
  if (prefs.begin("tnfsui", true)) {
    v = prefs.getString("hidden", "");
    prefs.end();
  }
  return v;
}

static void tnfsHiddenUrlsSave(const String& v) {
  if (prefs.begin("tnfsui", false)) {
    prefs.putString("hidden", v);
    prefs.end();
  }
}

static bool tnfsUrlLineExists(const String& list, const String& url) {
  int start = 0;
  while (start < (int)list.length()) {
    int end = list.indexOf('\n', start);
    if (end < 0) end = list.length();
    String line = list.substring(start, end);
    line.trim();
    if (line.equalsIgnoreCase(url)) return true;
    start = end + 1;
  }
  return false;
}

static String tnfsUrlLineRemove(const String& list, const String& url, bool& removed) {
  String out;
  removed = false;
  int start = 0;
  while (start < (int)list.length()) {
    int end = list.indexOf('\n', start);
    if (end < 0) end = list.length();
    String line = list.substring(start, end);
    line.trim();
    if (line.length()) {
      if (line.equalsIgnoreCase(url)) {
        removed = true;
      } else {
        out += line;
        out += "\n";
      }
    }
    start = end + 1;
  }
  return out;
}

static bool tnfsUrlIsDefault(const String& url) {
  for (size_t i = 0; i < sizeof(TNFS_DEFAULT_URLS) / sizeof(TNFS_DEFAULT_URLS[0]); i++) {
    if (url.equalsIgnoreCase(String(TNFS_DEFAULT_URLS[i]))) return true;
  }
  return false;
}

static String tnfsAllServersJson() {
  String hidden = tnfsHiddenUrlsLoad();
  String json = "{\"ok\":1,\"protocol\":\"TNFS\",\"port\":" + String(TNFS_DEFAULT_PORT) + ",\"servers\":[";
  bool first = true;
  for (size_t i = 0; i < sizeof(TNFS_DEFAULT_URLS) / sizeof(TNFS_DEFAULT_URLS[0]); i++) {
    String url = String(TNFS_DEFAULT_URLS[i]);
    if (tnfsUrlLineExists(hidden, url)) continue;
    if (!first) json += ",";
    first = false;
    json += "{\"url\":\"" + jsonEscape(url) + "\",\"source\":\"default\",\"deletable\":1}";
  }
  String custom = tnfsCustomUrlsLoad();
  int start = 0;
  while (start < (int)custom.length()) {
    int end = custom.indexOf('\n', start);
    if (end < 0) end = custom.length();
    String line = custom.substring(start, end);
    line.trim();
    if (line.length() && !tnfsUrlLineExists(hidden, line)) {
      bool dup = false;
      for (size_t i = 0; i < sizeof(TNFS_DEFAULT_URLS) / sizeof(TNFS_DEFAULT_URLS[0]); i++) {
        if (line.equalsIgnoreCase(String(TNFS_DEFAULT_URLS[i]))) { dup = true; break; }
      }
      if (!dup) {
        if (!first) json += ",";
        first = false;
        json += "{\"url\":\"" + jsonEscape(line) + "\",\"source\":\"custom\",\"deletable\":1}";
      }
    }
    start = end + 1;
  }
  json += "]}";
  return json;
}

static int tnfsSendRecv(WiFiUDP& udp, const IPAddress& ip, uint16_t port,
                        const uint8_t* req, size_t reqLen,
                        uint8_t* resp, size_t respMax,
                        uint8_t seq, uint8_t cmd,
                        String& err) {
  for (uint8_t attempt = 0; attempt < TNFS_MAX_RETRIES; attempt++) {
    if (!udp.beginPacket(ip, port)) {
      err = F("UDP_BEGIN_PACKET_FAILED");
      return -1;
    }
    udp.write(req, reqLen);
    if (!udp.endPacket()) {
      err = F("UDP_SEND_FAILED");
      return -1;
    }

    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < TNFS_TIMEOUT_MS) {
      int packetSize = udp.parsePacket();
      if (packetSize > 0) {
        int n = udp.read(resp, respMax);
        if (n >= 4 && resp[2] == seq && resp[3] == cmd) return n;
      }
      delay(2);
      yield();
    }
  }
  err = F("TNFS_TIMEOUT");
  return -1;
}

static void tnfsPutHeader(uint8_t* buf, uint16_t conn, uint8_t seq, uint8_t cmd) {
  buf[0] = (uint8_t)(conn & 0xFF);
  buf[1] = (uint8_t)((conn >> 8) & 0xFF);
  buf[2] = seq;
  buf[3] = cmd;
}


static uint8_t tnfsNextSeq() {
  uint8_t seq = ++g_tnfsSeq;
  if (g_tnfsSeq == 0) { g_tnfsSeq = 1; seq = 1; }
  return seq;
}

static void tnfsAppendCString(uint8_t* buf, size_t& pos, size_t maxLen, const String& v) {
  for (size_t i = 0; i < (size_t)v.length() && pos < maxLen - 1; i++) buf[pos++] = (uint8_t)v[i];
  if (pos < maxLen) buf[pos++] = 0x00;
}

static String tnfsNormalizeRemotePath(String p) {
  p.trim();
  p.replace("\\", "/");
  if (!p.length()) p = "/";
  if (!p.startsWith("/")) p = "/" + p;
  while (p.indexOf("//") >= 0) p.replace("//", "/");
  return p;
}

static String tnfsBaseNameFromPath(String p) {
  p = tnfsNormalizeRemotePath(p);
  if (p.endsWith("/") && p.length() > 1) p.remove(p.length() - 1);
  int slash = p.lastIndexOf('/');
  return (slash >= 0) ? p.substring(slash + 1) : p;
}

static bool tnfsNameIsAtr(String name) {
  name.toLowerCase();
  return name.endsWith(".atr");
}

static bool tnfsNameIsXexLike(String name) {
  name.toLowerCase();
  return name.endsWith(".xex") || name.endsWith(".com") || name.endsWith(".exe") || name.endsWith(".bas");
}

static bool tnfsNameIsCas(String name) {
  name.toLowerCase();
  return name.endsWith(".cas");
}

static String tnfsRawDirForName(String name) {
  name.toLowerCase();
  if (name.endsWith(".xex")) return String("/XEX");
  if (name.endsWith(".com")) return String("/COM");
  if (name.endsWith(".exe")) return String("/EXE");
  if (name.endsWith(".bas")) return String("/BAS");
  return String("/FILES");
}

static bool tnfsEnsureDir(const String& dir) {
  if (dir.length() == 0) return false;
  if (SPIFFS.exists(dir)) return true;
  return SPIFFS.mkdir(dir);
}

static bool tnfsMoveTmpToFinal(const String& tmpPath, const String& localPath, String& err) {
  if (tmpPath.length() == 0 || localPath.length() == 0) { err = F("LOCAL_PATH_INVALID"); return false; }
  if (SPIFFS.exists(localPath)) SPIFFS.remove(localPath);
  if (SPIFFS.rename(tmpPath, localPath)) return true;
  File src = SPIFFS.open(tmpPath, "r");
  File dst = SPIFFS.open(localPath, "w");
  if (!src || !dst) {
    if (src) src.close();
    if (dst) dst.close();
    if (SPIFFS.exists(tmpPath)) SPIFFS.remove(tmpPath);
    err = F("LOCAL_RENAME_FAILED");
    return false;
  }
  uint8_t buf[512];
  while (src.available()) {
    int n = src.read(buf, sizeof(buf));
    if (n > 0 && dst.write(buf, n) != (size_t)n) { err = F("LOCAL_COPY_FAILED"); break; }
    yield();
  }
  src.close(); dst.close();
  SPIFFS.remove(tmpPath);
  if (err.length()) { if (SPIFFS.exists(localPath)) SPIFFS.remove(localPath); return false; }
  return true;
}

static String tnfsRawDownloadDoneJson(const String& remotePath, const String& localName, const String& localPath,
                                      bool skipped, uint32_t bytes, uint32_t elapsedMs) {
  String j;
  j.reserve(900);
  j = "{";
  j += "\"ok\":true";
  j += ",\"action\":\"tnfs_download_raw_file\"";
  j += ",\"tnfs\":1";
  j += ",\"downloaded\":" + String(skipped ? 0 : 1);
  j += ",\"skipped\":" + String(skipped ? 1 : 0);
  j += ",\"bytes\":" + String((unsigned long)bytes);
  j += ",\"remotePath\":\"" + jsonEscape(remotePath) + "\"";
  j += ",\"localName\":\"" + jsonEscape(localName) + "\"";
  j += ",\"localPath\":\"" + jsonEscape(localPath) + "\"";
  j += ",\"type\":\"" + jsonEscape(webAtrStoredTypeForName(localName)) + "\"";
  j += ",\"mounted\":false";
  j += ",\"preparedC\":0";
  j += ",\"elapsedMs\":" + String((unsigned long)elapsedMs);
  j += ",\"note\":\"F28: archivo TNFS guardado crudo con extension original; no convertido a ATR\"";
  j += "}";
  return j;
}

// [F24] Eliminado helper no usado: tnfsNameIsMountable
static String webAtrLightStatusJsonWithTnfs(const char* action, const String& remotePath, const String& localName,
                                           const String& localPath, int unit, bool skipped, uint32_t bytes,
                                           uint32_t elapsedMs) {
  String j;
  j.reserve(1200);
  j = "{";
  j += "\"ok\":1";
  j += ",\"action\":\"" + jsonEscape(String(action ? action : "tnfs_mount")) + "\"";
  j += ",\"compiled\":true";
  j += ",\"mode\":\"flash-multi\"";
  j += ",\"fastStatus\":1";
  j += ",\"tnfs\":1";
  j += ",\"unit\":" + String(unit);
  j += ",\"downloaded\":" + String(skipped ? 0 : 1);
  j += ",\"skipped\":" + String(skipped ? 1 : 0);
  j += ",\"bytes\":" + String((unsigned long)bytes);
  j += ",\"remotePath\":\"" + jsonEscape(remotePath) + "\"";
  j += ",\"localName\":\"" + jsonEscape(localName) + "\"";
  j += ",\"localPath\":\"" + jsonEscape(localPath) + "\"";
  j += ",\"enabled\":" + String(WEB_ATR_ENABLED ? 1 : 0);
  j += ",\"mask\":" + String(WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"forceMask\":" + String(WEB_ATR_FORCE_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"driveVisibleMask\":" + String(DRIVE_VISIBLE_MASK & DRIVE_UI_MAX_MASK);
  j += ",\"maxUnits\":" + String(DRIVE_UI_MAX_UNITS);
  j += ",\"deferredSave\":" + String(g_webAtrConfigSavePending ? 1 : 0);
  j += ",\"slots\":[";
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    if (i) j += ",";
    j += "{\"unit\":" + String(i + 1);
    j += ",\"visible\":" + String(driveUiVisibleIndex(i) ? 1 : 0);
    j += ",\"name\":\"" + jsonEscape(g_webAtrMountedName[i]) + "\"";
    j += ",\"enabled\":" + String((WEB_ATR_ENABLED && (WEB_ATR_DEV_MASK & (1u << i))) ? 1 : 0);
    j += ",\"forced\":" + String((WEB_ATR_ENABLED && (WEB_ATR_FORCE_MASK & (1u << i))) ? 1 : 0);
    j += ",\"present\":" + String(g_webAtrSlotPresent[i] ? 1 : 0);
    j += "}";
  }
  j += "]";
  j += ",\"elapsedMs\":" + String((unsigned long)elapsedMs);
  j += "}";
  return j;
}

static bool tnfsMountRootSession(WiFiUDP& udp, const IPAddress& ip, const TnfsParsedUrl& u, uint16_t& conn, String& err) {
  uint8_t req[320];
  uint8_t resp[700];
  uint8_t seq = tnfsNextSeq();
  size_t pos = 0;
  tnfsPutHeader(req, 0, seq, TNFS_CMD_MOUNT); pos = 4;
  req[pos++] = 0x02; req[pos++] = 0x01; // protocolo 1.2, little endian minor/major.
  req[pos++] = '/'; req[pos++] = 0x00;
  req[pos++] = 0x00; // usuario anonimo
  req[pos++] = 0x00; // password anonimo
  int n = tnfsSendRecv(udp, ip, u.port, req, pos, resp, sizeof(resp), seq, TNFS_CMD_MOUNT, err);
  if (n < 6) { if (!err.length()) err = F("MOUNT_FAILED"); return false; }
  conn = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);
  if (conn == 0) { err = (n > 4) ? tnfsStatusText(resp[4]) : String("MOUNT_DENIED"); return false; }
  return true;
}

static void tnfsUmountSession(WiFiUDP& udp, const IPAddress& ip, uint16_t port, uint16_t conn) {
  if (!conn) return;
  uint8_t req[8];
  uint8_t resp[64];
  String err;
  uint8_t seq = tnfsNextSeq();
  tnfsPutHeader(req, conn, seq, TNFS_CMD_UMOUNT);
  tnfsSendRecv(udp, ip, port, req, 4, resp, sizeof(resp), seq, TNFS_CMD_UMOUNT, err);
}

static void tnfsCloseFileBestEffort(WiFiUDP& udp, const IPAddress& ip, uint16_t port, uint16_t conn, uint8_t fd) {
  uint8_t req[8];
  uint8_t resp[64];
  String err;
  uint8_t seq = tnfsNextSeq();
  tnfsPutHeader(req, conn, seq, TNFS_CMD_CLOSE);
  req[4] = fd;
  tnfsSendRecv(udp, ip, port, req, 5, resp, sizeof(resp), seq, TNFS_CMD_CLOSE, err);
}

static void tnfsStreamJsonLine(const String& line) {
  server.sendContent(line);
  server.sendContent("\n");
}

static void tnfsStreamFinish() {
  // Cierra correctamente la respuesta chunked/NDJSON.
  // Sin esto, algunos clientes terminan sin recibir el evento final y reportan
  // TNFS_STREAM_WITHOUT_FINAL aunque el archivo XEX/COM/EXE/BAS ya se haya guardado.
  server.sendContent("");
}

static void tnfsStreamEvent(const String& phase, const String& message, uint32_t bytes, uint32_t elapsedMs, bool ok = true) {
  String j = "{";
  j += "\"type\":\"progress\"";
  j += ",\"ok\":" + String(ok ? 1 : 0);
  j += ",\"phase\":\"" + jsonEscape(phase) + "\"";
  j += ",\"message\":\"" + jsonEscape(message) + "\"";
  j += ",\"bytes\":" + String((unsigned long)bytes);
  j += ",\"elapsedMs\":" + String((unsigned long)elapsedMs);
  j += "}";
  tnfsStreamJsonLine(j);
}

static String tnfsAsDoneJson(String json) {
  json.trim();
  if (!json.startsWith("{")) return String("{\"type\":\"done\",\"ok\":0,\"error\":\"INVALID_FINAL_JSON\"}");
  json = json.substring(0, 1) + "\"type\":\"done\"," + json.substring(1);
  return json;
}

static bool tnfsDownloadFileToPath(const TnfsParsedUrl& u, const String& remotePath, const String& localTmpPath,
                                   uint32_t& bytesOut, String& err, bool streamProgress = false, uint32_t streamStartMs = 0) {
  bytesOut = 0;
  if (WiFi.status() != WL_CONNECTED) { err = F("WIFI_STA_NOT_CONNECTED"); return false; }
  if (!webAtrFsReady()) { err = F("STORAGE_NOT_READY"); return false; }

  IPAddress ip;
  if (!WiFi.hostByName(u.host.c_str(), ip)) { err = F("DNS_FAILED"); return false; }

  WiFiUDP udp;
  if (!udp.begin(0)) { err = F("UDP_BEGIN_FAILED"); return false; }

  uint16_t conn = 0;
  if (streamProgress) tnfsStreamEvent("connect", "Conectando a TNFS...", 0, millis() - streamStartMs);
  if (!tnfsMountRootSession(udp, ip, u, conn, err)) { udp.stop(); return false; }
  if (streamProgress) tnfsStreamEvent("open", "Abriendo archivo remoto...", 0, millis() - streamStartMs);

  uint8_t req[360];
  uint8_t resp[1100];
  uint8_t fd = 0;
  uint8_t seq = tnfsNextSeq();
  size_t pos = 0;
  tnfsPutHeader(req, conn, seq, TNFS_CMD_OPEN); pos = 4;
  req[pos++] = (uint8_t)(TNFS_OPEN_RDONLY & 0xFF);
  req[pos++] = (uint8_t)((TNFS_OPEN_RDONLY >> 8) & 0xFF);
  req[pos++] = 0x00; req[pos++] = 0x00; // mode 0000, no aplica en read-only
  tnfsAppendCString(req, pos, sizeof(req), remotePath);
  int n = tnfsSendRecv(udp, ip, u.port, req, pos, resp, sizeof(resp), seq, TNFS_CMD_OPEN, err);
  if (n < 6 || resp[4] != 0x00) {
    err = String("OPEN_FAILED:") + ((n > 4) ? tnfsStatusText(resp[4]) : err);
    tnfsUmountSession(udp, ip, u.port, conn);
    udp.stop();
    return false;
  }
  fd = resp[5];

  if (SPIFFS.exists(localTmpPath)) SPIFFS.remove(localTmpPath);
  File out = SPIFFS.open(localTmpPath, "w");
  if (!out) {
    err = F("LOCAL_OPEN_FAILED");
    tnfsCloseFileBestEffort(udp, ip, u.port, conn, fd);
    tnfsUmountSession(udp, ip, u.port, conn);
    udp.stop();
    return false;
  }

  bool ok = true;
  while (ok) {
    seq = tnfsNextSeq();
    tnfsPutHeader(req, conn, seq, TNFS_CMD_READ); pos = 4;
    req[pos++] = fd;
    req[pos++] = (uint8_t)(TNFS_READ_CHUNK & 0xFF);
    req[pos++] = (uint8_t)((TNFS_READ_CHUNK >> 8) & 0xFF);
    n = tnfsSendRecv(udp, ip, u.port, req, pos, resp, sizeof(resp), seq, TNFS_CMD_READ, err);
    if (n < 5) { err = err.length() ? err : String("READ_FAILED"); ok = false; break; }
    if (resp[4] == TNFS_ERR_EOF) break;
    if (resp[4] != 0x00) { err = String("READ_FAILED:") + tnfsStatusText(resp[4]); ok = false; break; }
    if (n < 7) { err = F("READ_SHORT_RESPONSE"); ok = false; break; }
    uint16_t got = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
    if (got == 0) break;
    if (7 + got > (uint16_t)n) { err = F("READ_SIZE_MISMATCH"); ok = false; break; }
    if (bytesOut + got > TNFS_DOWNLOAD_MAX_BYTES) { err = F("FILE_TOO_LARGE"); ok = false; break; }
    size_t wr = out.write(resp + 7, got);
    if (wr != got) { err = F("LOCAL_WRITE_FAILED"); ok = false; break; }
    bytesOut += got;
    if ((bytesOut & 0x1FFF) == 0) { out.flush(); if (streamProgress) tnfsStreamEvent("download", "Descargando desde TNFS...", bytesOut, millis() - streamStartMs); yield(); }
    delay(0);
    yield();
  }

  if (streamProgress) tnfsStreamEvent("download", "Descarga TNFS recibida. Cerrando archivo...", bytesOut, millis() - streamStartMs);
  out.flush();
  out.close();
  tnfsCloseFileBestEffort(udp, ip, u.port, conn, fd);
  tnfsUmountSession(udp, ip, u.port, conn);
  udp.stop();

  if (!ok || bytesOut == 0) {
    if (SPIFFS.exists(localTmpPath)) SPIFFS.remove(localTmpPath);
    if (!err.length()) err = F("EMPTY_OR_FAILED_DOWNLOAD");
    return false;
  }
  return true;
}

static void webAtrMountLocalNameOnUnit(const String& localName, const String& localPath, int unit) {
  if (unit < 1 || unit > WEB_ATR_MAX_UNITS) return;
  int idx = unit - 1;
  uint8_t bit = (uint8_t)(1u << idx);
  g_webAtrMountedName[idx] = localName;
  webAtrResetResolvedSlot(idx);
  g_webAtrMountedPath[idx] = localPath;
  WEB_ATR_ENABLED = true;
  WEB_ATR_DEV_MASK |= bit;
  WEB_ATR_FORCE_MASK |= bit;
  DRIVE_VISIBLE_MASK |= bit;
  clearBtSio2pcUnits(bit);
  normalizeDriveMasks();
  webAtrCacheInvalidateDev((uint8_t)(0x30 + unit));
  webAtrRefreshPresenceMask(bit);
  markDeferredConfigSave(true, true);
}

static bool tnfsPrepareCasLocalFast(const String& localName, const String& localPath,
                                    const String& remotePath, bool skipped, uint32_t bytes,
                                    uint32_t elapsedMs, String& jsonOut, String& errOut) {
  if (!localName.length() || !webCasLooksLikeName(localName)) { errOut = F("INVALID_CAS_NAME"); return false; }
  if (!webAtrFsReady() || !SPIFFS.exists(localPath)) {
    errOut = String("CAS_NOT_FOUND:") + localPath;
    casSetLastError(errOut);
    return false;
  }
  File f = SPIFFS.open(localPath, "r");
  g_casBytesLast = f ? (uint32_t)f.size() : 0;
  if (f) f.close();

  g_casMountedName = localName;
  g_casMountedPath = localPath;
  g_casMounted = true;
  g_casPlaying = false;
  g_casPaused = false;
  g_casEof = false;
  g_casBootModeActive = false;
  g_casMounts++;

  casAutoResetAnalysis();
  casSetLastError("");
  webAtrPatchCasFlagsInFilesCache(g_casMountedName, true, false);
  markDeferredConfigSave(false, false, true);

  String j;
  j.reserve(1200);
  j = "{";
  j += "\"ok\":true";
  j += ",\"action\":\"tnfs_cas_prepare_c\"";
  j += ",\"tnfs\":1";
  j += ",\"downloaded\":" + String(skipped ? 0 : 1);
  j += ",\"skipped\":" + String(skipped ? 1 : 0);
  j += ",\"bytes\":" + String((unsigned long)bytes);
  j += ",\"remotePath\":\"" + jsonEscape(remotePath) + "\"";
  j += ",\"localName\":\"" + jsonEscape(localName) + "\"";
  j += ",\"localPath\":\"" + jsonEscape(localPath) + "\"";
  j += ",\"preparedC\":1";
  j += ",\"mounted\":true";
  j += ",\"casMounted\":1";
  j += ",\"playing\":false";
  j += ",\"casPlaying\":0";
  j += ",\"name\":\"" + jsonEscape(g_casMountedName) + "\"";
  j += ",\"casName\":\"" + jsonEscape(g_casMountedName) + "\"";
  j += ",\"path\":\"" + jsonEscape(g_casMountedPath) + "\"";
  j += ",\"lastBytes\":" + String((unsigned long)g_casBytesLast);
  j += ",\"elapsedMs\":" + String((unsigned long)elapsedMs);
  j += ",\"note\":\"F27: CAS descargado desde TNFS a /CAS y preparado en C:; analisis CAS diferido\"";
  j += "}";
  jsonOut = j;
  return true;
}


static void handleApiTnfsFetchMountStream() {
  uint32_t t0 = millis();
  String url = server.hasArg("url") ? server.arg("url") : String("");
  String pathArg = server.hasArg("path") ? server.arg("path") : String("");
  int unit = server.hasArg("unit") ? server.arg("unit").toInt() : 1;

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/x-ndjson", "");

  auto sendDoneError = [&](const String& error, const String& hint = String("")) {
    String j = "{\"type\":\"done\",\"ok\":0,\"error\":\"" + jsonEscape(error) + "\"";
    if (hint.length()) j += ",\"hint\":\"" + jsonEscape(hint) + "\"";
    j += ",\"elapsedMs\":" + String((unsigned long)(millis() - t0)) + "}";
    tnfsStreamJsonLine(j);
    tnfsStreamFinish();
  };

  TnfsParsedUrl u;
  if (!tnfsParseUrl(url, u)) { sendDoneError("INVALID_TNFS_URL"); return; }
  if (pathArg.length()) u.path = tnfsNormalizeRemotePath(pathArg);
  String remotePath = tnfsNormalizeRemotePath(u.path);
  String remoteName = tnfsBaseNameFromPath(remotePath);
  bool remoteIsAtr = tnfsNameIsAtr(remoteName);
  bool remoteIsRaw = tnfsNameIsXexLike(remoteName); // .XEX/.COM/.EXE/.BAS crudos desde F28
  bool remoteIsCas = tnfsNameIsCas(remoteName);
  if (!remoteIsAtr && !remoteIsRaw && !remoteIsCas) {
    sendDoneError("UNSUPPORTED_FILE_TYPE", "Se admite .CAS para preparar C:, .ATR directo y .XEX/.COM/.EXE/.BAS como descarga cruda sin convertir.");
    return;
  }
  if (remoteIsAtr && (unit < 1 || unit > WEB_ATR_MAX_UNITS)) { sendDoneError("INVALID_UNIT"); return; }
  if (!webAtrFsReady()) { sendDoneError("STORAGE_NOT_READY"); return; }
#if WEB_STORAGE_USE_SD
  tnfsEnsureDir("/ATR");
  tnfsEnsureDir("/CAS");
  tnfsEnsureDir("/TMP");
  if (remoteIsRaw) tnfsEnsureDir(tnfsRawDirForName(remoteName));
#endif

  String localName = webAtrSanitizeFileName(remoteName);
  String localDir = remoteIsCas ? String("/CAS") : (remoteIsAtr ? String("/ATR") : tnfsRawDirForName(localName));
  String tnfsTargetText = remoteIsCas ? String(" para C:...") : (remoteIsAtr ? (String(" para D") + String(unit) + "...") : String(" para Biblioteca..."));
  tnfsStreamEvent("start", String("Preparando ") + remoteName + tnfsTargetText, 0, millis() - t0);

  String localPath = webAtrFindExistingInDir(localDir.c_str(), localName);
  if (!localPath.length()) localPath = webAtrFindExistingRecursive("/", localName);
  bool skipped = localPath.length() > 0;
  uint32_t bytes = 0;
  String err;

  if (skipped) {
    tnfsStreamEvent("skip", String("El archivo ya existe en ") + localDir + ". Se omite descarga.", 0, millis() - t0);
  } else {
    localPath = localDir + "/" + localName;
    String tmpPath = String("/TMP/tnfs_") + localName + ".part";
    if (SPIFFS.exists(tmpPath)) SPIFFS.remove(tmpPath);
    bool ok = tnfsDownloadFileToPath(u, remotePath, tmpPath, bytes, err, true, t0);
    if (!ok) { sendDoneError(err.length() ? err : String("DOWNLOAD_FAILED")); return; }
    tnfsStreamEvent("save", String("Guardando en ") + localDir + " sin convertir...", bytes, millis() - t0);
    if (!tnfsMoveTmpToFinal(tmpPath, localPath, err)) {
      sendDoneError(err.length() ? err : String("LOCAL_SAVE_FAILED"));
      return;
    }
  }

  if (remoteIsCas) {
    tnfsStreamEvent("mount", "Preparando cassette en C:...", bytes, millis() - t0);
    String finalJson, casErr;
    if (!tnfsPrepareCasLocalFast(localName, localPath, remotePath, skipped, bytes, (uint32_t)(millis() - t0), finalJson, casErr)) {
      if (!skipped && SPIFFS.exists(localPath)) SPIFFS.remove(localPath);
      sendDoneError(casErr.length() ? casErr : String("CAS_PREPARE_FAILED"));
      return;
    }
    webAtrInvalidateFilesCache();
    logf("[TNFS-CAS] %s %s -> %s C: bytes=%lu elapsed=%lu ms", skipped ? "SKIP" : "DOWNLOAD", remotePath.c_str(), localPath.c_str(), (unsigned long)bytes, (unsigned long)(millis() - t0));
    tnfsStreamJsonLine(tnfsAsDoneJson(finalJson));
    tnfsStreamFinish();
    return;
  }

  if (remoteIsRaw) {
    webAtrInvalidateFilesCache();
    logf("[TNFS-RAW] %s %s -> %s bytes=%lu elapsed=%lu ms", skipped ? "SKIP" : "DOWNLOAD", remotePath.c_str(), localPath.c_str(), (unsigned long)bytes, (unsigned long)(millis() - t0));
    String finalJson = tnfsRawDownloadDoneJson(remotePath, localName, localPath, skipped, bytes, (uint32_t)(millis() - t0));
    tnfsStreamJsonLine(tnfsAsDoneJson(finalJson));
    tnfsStreamFinish();
    return;
  }

  tnfsStreamEvent("mount", String("Montando en D") + String(unit) + "...", bytes, millis() - t0);
  WebAtrMeta meta;
  if (!webAtrReadMetaFromPath(localPath, meta)) {
    if (!skipped && SPIFFS.exists(localPath)) SPIFFS.remove(localPath);
    sendDoneError("DOWNLOADED_FILE_NOT_VALID_MOUNT_IMAGE");
    return;
  }

  webAtrMountLocalNameOnUnit(localName, localPath, unit);
  webAtrInvalidateFilesCache();
  logf("[TNFS] %s %s -> %s D%d bytes=%lu elapsed=%lu ms", skipped ? "SKIP" : "DOWNLOAD", remotePath.c_str(), localPath.c_str(), unit, (unsigned long)bytes, (unsigned long)(millis() - t0));

  String finalJson = webAtrLightStatusJsonWithTnfs(skipped ? "tnfs_skip_mount" : "tnfs_download_mount", remotePath, localName, localPath, unit, skipped, bytes, (uint32_t)(millis() - t0));
  tnfsStreamJsonLine(tnfsAsDoneJson(finalJson));
  tnfsStreamFinish();
}

static void handleApiTnfsFetchMount() {
  if (server.hasArg("stream") && server.arg("stream") == "1") {
    handleApiTnfsFetchMountStream();
    return;
  }
  uint32_t t0 = millis();
  String url = server.hasArg("url") ? server.arg("url") : String("");
  String pathArg = server.hasArg("path") ? server.arg("path") : String("");
  int unit = server.hasArg("unit") ? server.arg("unit").toInt() : 1;

  TnfsParsedUrl u;
  if (!tnfsParseUrl(url, u)) { server.send(400, "application/json", "{\"ok\":0,\"error\":\"INVALID_TNFS_URL\"}"); return; }
  if (pathArg.length()) u.path = tnfsNormalizeRemotePath(pathArg);
  String remotePath = tnfsNormalizeRemotePath(u.path);
  String remoteName = tnfsBaseNameFromPath(remotePath);
  bool remoteIsAtr = tnfsNameIsAtr(remoteName);
  bool remoteIsRaw = tnfsNameIsXexLike(remoteName); // .XEX/.COM/.EXE/.BAS crudos desde F28
  bool remoteIsCas = tnfsNameIsCas(remoteName);
  if (!remoteIsAtr && !remoteIsRaw && !remoteIsCas) {
    String json = "{\"ok\":0,\"error\":\"UNSUPPORTED_FILE_TYPE\",\"hint\":\"Se descarga .CAS a /CAS y prepara C:, .ATR directo, y .XEX/.COM/.EXE/.BAS como archivo crudo sin convertir.\",\"name\":\"" + jsonEscape(remoteName) + "\"}";
    server.send(200, "application/json", json);
    return;
  }
  if (remoteIsAtr && (unit < 1 || unit > WEB_ATR_MAX_UNITS)) { server.send(400, "application/json", "{\"ok\":0,\"error\":\"INVALID_UNIT\"}"); return; }
  if (!webAtrFsReady()) { server.send(200, "application/json", "{\"ok\":0,\"error\":\"STORAGE_NOT_READY\"}"); return; }
#if WEB_STORAGE_USE_SD
  tnfsEnsureDir("/ATR");
  tnfsEnsureDir("/CAS");
  tnfsEnsureDir("/TMP");
  if (remoteIsRaw) tnfsEnsureDir(tnfsRawDirForName(remoteName));
#endif

  String localName = webAtrSanitizeFileName(remoteName);
  String localDir = remoteIsCas ? String("/CAS") : (remoteIsAtr ? String("/ATR") : tnfsRawDirForName(localName));
  String localPath = webAtrFindExistingInDir(localDir.c_str(), localName);
  if (!localPath.length()) localPath = webAtrFindExistingRecursive("/", localName);
  bool skipped = localPath.length() > 0;
  uint32_t bytes = 0;
  String err;

  if (!skipped) {
    localPath = localDir + "/" + localName;
    String tmpPath = String("/TMP/tnfs_") + localName + ".part";
    if (SPIFFS.exists(tmpPath)) SPIFFS.remove(tmpPath);
    bool ok = tnfsDownloadFileToPath(u, remotePath, tmpPath, bytes, err);
    if (!ok) {
      String json = "{\"ok\":0,\"error\":\"" + jsonEscape(err.length() ? err : String("DOWNLOAD_FAILED")) + "\",\"remotePath\":\"" + jsonEscape(remotePath) + "\",\"elapsedMs\":" + String((unsigned long)(millis() - t0)) + "}";
      server.send(200, "application/json", json);
      return;
    }
    if (!tnfsMoveTmpToFinal(tmpPath, localPath, err)) {
      String json = "{\"ok\":0,\"error\":\"" + jsonEscape(err.length() ? err : String("LOCAL_SAVE_FAILED")) + "\",\"localPath\":\"" + jsonEscape(localPath) + "\"}";
      server.send(200, "application/json", json);
      return;
    }
  }

  if (remoteIsCas) {
    String json, casErr;
    if (!tnfsPrepareCasLocalFast(localName, localPath, remotePath, skipped, bytes, (uint32_t)(millis() - t0), json, casErr)) {
      if (!skipped && SPIFFS.exists(localPath)) SPIFFS.remove(localPath);
      String e = casErr.length() ? casErr : String("CAS_PREPARE_FAILED");
      server.send(200, "application/json", "{\"ok\":0,\"error\":\"" + jsonEscape(e) + "\"}");
      return;
    }
    webAtrInvalidateFilesCache();
    logf("[TNFS-CAS] %s %s -> %s C: bytes=%lu elapsed=%lu ms", skipped ? "SKIP" : "DOWNLOAD", remotePath.c_str(), localPath.c_str(), (unsigned long)bytes, (unsigned long)(millis() - t0));
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", json);
    return;
  }

  if (remoteIsRaw) {
    webAtrInvalidateFilesCache();
    logf("[TNFS-RAW] %s %s -> %s bytes=%lu elapsed=%lu ms", skipped ? "SKIP" : "DOWNLOAD", remotePath.c_str(), localPath.c_str(), (unsigned long)bytes, (unsigned long)(millis() - t0));
    String json = tnfsRawDownloadDoneJson(remotePath, localName, localPath, skipped, bytes, (uint32_t)(millis() - t0));
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", json);
    return;
  }

  WebAtrMeta meta;
  if (!webAtrReadMetaFromPath(localPath, meta)) {
    if (!skipped && SPIFFS.exists(localPath)) SPIFFS.remove(localPath);
    String json = "{\"ok\":0,\"error\":\"DOWNLOADED_FILE_NOT_VALID_MOUNT_IMAGE\",\"localPath\":\"" + jsonEscape(localPath) + "\"}";
    server.send(200, "application/json", json);
    return;
  }

  webAtrMountLocalNameOnUnit(localName, localPath, unit);
  webAtrInvalidateFilesCache();
  logf("[TNFS] %s %s -> %s D%d bytes=%lu elapsed=%lu ms", skipped ? "SKIP" : "DOWNLOAD", remotePath.c_str(), localPath.c_str(), unit, (unsigned long)bytes, (unsigned long)(millis() - t0));

  String json = webAtrLightStatusJsonWithTnfs(skipped ? "tnfs_skip_mount" : "tnfs_download_mount", remotePath, localName, localPath, unit, skipped, bytes, (uint32_t)(millis() - t0));
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", json);
}

static void handleApiTnfsServers() {
  server.send(200, "application/json", tnfsAllServersJson());
}

static void handleApiTnfsAdd() {
  String url = server.hasArg("url") ? server.arg("url") : String("");
  TnfsParsedUrl u;
  if (!tnfsParseUrl(url, u)) {
    server.send(400, "application/json", "{\"ok\":0,\"error\":\"INVALID_TNFS_URL\"}");
    return;
  }

  String hidden = tnfsHiddenUrlsLoad();
  bool unhidden = false;
  String hidden2 = tnfsUrlLineRemove(hidden, u.raw, unhidden);
  if (unhidden) tnfsHiddenUrlsSave(hidden2);

  String custom = tnfsCustomUrlsLoad();
  bool exists = tnfsUrlLineExists(custom, u.raw) || tnfsUrlIsDefault(u.raw);
  if (!exists) {
    if (custom.length() && !custom.endsWith("\n")) custom += "\n";
    custom += u.raw;
    custom += "\n";
    tnfsCustomUrlsSave(custom);
  }
  String json = "{\"ok\":1,\"added\":" + String(exists ? 0 : 1) + ",\"unhidden\":" + String(unhidden ? 1 : 0) + ",\"url\":\"" + jsonEscape(u.raw) + "\"}";
  server.send(200, "application/json", json);
}

static void handleApiTnfsDelete() {
  String url = server.hasArg("url") ? server.arg("url") : String("");
  TnfsParsedUrl u;
  if (!tnfsParseUrl(url, u)) {
    server.send(400, "application/json", "{\"ok\":0,\"error\":\"INVALID_TNFS_URL\"}");
    return;
  }

  bool customRemoved = false;
  String custom = tnfsCustomUrlsLoad();
  String custom2 = tnfsUrlLineRemove(custom, u.raw, customRemoved);
  if (customRemoved) tnfsCustomUrlsSave(custom2);

  bool hiddenAdded = false;
  bool isDefault = tnfsUrlIsDefault(u.raw);
  if (isDefault) {
    String hidden = tnfsHiddenUrlsLoad();
    if (!tnfsUrlLineExists(hidden, u.raw)) {
      if (hidden.length() && !hidden.endsWith("\n")) hidden += "\n";
      hidden += u.raw;
      hidden += "\n";
      tnfsHiddenUrlsSave(hidden);
      hiddenAdded = true;
    }
  }

  String json = "{\"ok\":1,\"url\":\"" + jsonEscape(u.raw) + "\",\"removed\":" + String((customRemoved || hiddenAdded) ? 1 : 0) + ",\"customRemoved\":" + String(customRemoved ? 1 : 0) + ",\"defaultHidden\":" + String((isDefault && (hiddenAdded || !customRemoved)) ? 1 : 0) + "}";
  server.send(200, "application/json", json);
}

static void handleApiTnfsList() {
  uint32_t t0 = millis();
  String url = server.hasArg("url") ? server.arg("url") : String("");
  String pathArg = server.hasArg("path") ? server.arg("path") : String("");
  int limit = server.hasArg("limit") ? server.arg("limit").toInt() : 80;
  int offset = server.hasArg("offset") ? server.arg("offset").toInt() : 0;
  if (limit < 1) limit = 1;
  if (limit > 160) limit = 160;
  if (offset < 0) offset = 0;
  // Evita que una página muy profunda bloquee demasiado al ESP32.
  // Si se necesita más, se puede subir este tope después de medir tiempos reales.
  if (offset > 5000) offset = 5000;

  TnfsParsedUrl u;
  if (!tnfsParseUrl(url, u)) {
    server.send(400, "application/json", "{\"ok\":0,\"error\":\"INVALID_TNFS_URL\"}");
    return;
  }
  if (pathArg.length()) {
    pathArg.trim();
    pathArg.replace("\\", "/");
    if (!pathArg.startsWith("/")) pathArg = "/" + pathArg;
    u.path = pathArg;
  }

  if (WiFi.status() != WL_CONNECTED) {
    String json = "{\"ok\":0,\"error\":\"WIFI_STA_NOT_CONNECTED\",\"hint\":\"Configura WiFi STA para acceder a servidores TNFS externos.\",\"url\":\"" + jsonEscape(u.raw) + "\"}";
    server.send(200, "application/json", json);
    return;
  }

  IPAddress ip;
  if (!WiFi.hostByName(u.host.c_str(), ip)) {
    String json = "{\"ok\":0,\"error\":\"DNS_FAILED\",\"host\":\"" + jsonEscape(u.host) + "\"}";
    server.send(200, "application/json", json);
    return;
  }

  WiFiUDP udp;
  if (!udp.begin(0)) {
    server.send(200, "application/json", "{\"ok\":0,\"error\":\"UDP_BEGIN_FAILED\"}");
    return;
  }

  uint8_t req[320];
  uint8_t resp[700];
  String err;
  uint8_t seq = ++g_tnfsSeq;
  if (g_tnfsSeq == 0) g_tnfsSeq = 1;

  // MOUNT: version 1.2, mount path "/", anonymous.
  size_t pos = 0;
  tnfsPutHeader(req, 0, seq, TNFS_CMD_MOUNT); pos = 4;
  req[pos++] = 0x02; req[pos++] = 0x01;
  req[pos++] = '/'; req[pos++] = 0x00;
  req[pos++] = 0x00; // user
  req[pos++] = 0x00; // password
  int n = tnfsSendRecv(udp, ip, u.port, req, pos, resp, sizeof(resp), seq, TNFS_CMD_MOUNT, err);
  if (n < 6) {
    udp.stop();
    String json = "{\"ok\":0,\"error\":\"" + jsonEscape(err.length() ? err : String("MOUNT_FAILED")) + "\",\"host\":\"" + jsonEscape(u.host) + "\"}";
    server.send(200, "application/json", json);
    return;
  }
  uint16_t conn = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);
  if (conn == 0) {
    String status = (n > 4) ? tnfsStatusText(resp[4]) : String("MOUNT_DENIED");
    udp.stop();
    String json = "{\"ok\":0,\"error\":\"MOUNT_FAILED\",\"status\":\"" + jsonEscape(status) + "\"}";
    server.send(200, "application/json", json);
    return;
  }

  // OPENDIR path solicitado.
  seq = ++g_tnfsSeq; if (g_tnfsSeq == 0) g_tnfsSeq = 1;
  tnfsPutHeader(req, conn, seq, TNFS_CMD_OPENDIR); pos = 4;
  for (size_t i = 0; i < (size_t)u.path.length() && pos < sizeof(req) - 1; i++) req[pos++] = (uint8_t)u.path[i];
  req[pos++] = 0x00;
  n = tnfsSendRecv(udp, ip, u.port, req, pos, resp, sizeof(resp), seq, TNFS_CMD_OPENDIR, err);
  if (n < 6 || resp[4] != 0x00) {
    uint8_t code = (n > 4) ? resp[4] : 0xFF;
    // UMOUNT best-effort.
    seq = ++g_tnfsSeq; if (g_tnfsSeq == 0) g_tnfsSeq = 1;
    tnfsPutHeader(req, conn, seq, TNFS_CMD_UMOUNT);
    tnfsSendRecv(udp, ip, u.port, req, 4, resp, sizeof(resp), seq, TNFS_CMD_UMOUNT, err);
    udp.stop();
    String json = "{\"ok\":0,\"error\":\"OPENDIR_FAILED\",\"status\":\"" + jsonEscape(tnfsStatusText(code)) + "\",\"path\":\"" + jsonEscape(u.path) + "\"}";
    server.send(200, "application/json", json);
    return;
  }
  uint8_t handle = resp[5];

  String items = "[";
  bool first = true;
  int count = 0;
  int seen = 0;
  int skipped = 0;
  bool eof = false;
  bool hasMore = false;

  while (true) {
    seq = ++g_tnfsSeq; if (g_tnfsSeq == 0) g_tnfsSeq = 1;
    tnfsPutHeader(req, conn, seq, TNFS_CMD_READDIR); pos = 4;
    req[pos++] = handle;
    n = tnfsSendRecv(udp, ip, u.port, req, pos, resp, sizeof(resp), seq, TNFS_CMD_READDIR, err);
    if (n < 5) break;
    if (resp[4] == TNFS_ERR_EOF) { eof = true; break; }
    if (resp[4] != 0x00) break;

    String name;
    for (int i = 5; i < n && resp[i] != 0; i++) {
      char c = (char)resp[i];
      if (c >= 32) name += c;
    }
    if (!name.length()) continue;

    if (seen < offset) {
      seen++;
      skipped++;
      continue;
    }

    if (count >= limit) {
      hasMore = true;
      break;
    }

    if (!first) items += ",";
    first = false;
    items += "{\"name\":\"" + jsonEscape(name) + "\",\"type\":\"unknown\"}";
    count++;
    seen++;
  }
  items += "]";

  // CLOSEDIR best-effort.
  seq = ++g_tnfsSeq; if (g_tnfsSeq == 0) g_tnfsSeq = 1;
  tnfsPutHeader(req, conn, seq, TNFS_CMD_CLOSEDIR); pos = 4;
  req[pos++] = handle;
  tnfsSendRecv(udp, ip, u.port, req, pos, resp, sizeof(resp), seq, TNFS_CMD_CLOSEDIR, err);

  // UMOUNT best-effort.
  seq = ++g_tnfsSeq; if (g_tnfsSeq == 0) g_tnfsSeq = 1;
  tnfsPutHeader(req, conn, seq, TNFS_CMD_UMOUNT);
  tnfsSendRecv(udp, ip, u.port, req, 4, resp, sizeof(resp), seq, TNFS_CMD_UMOUNT, err);
  udp.stop();

  String currentUrl = String("tnfs://") + u.host + (u.port == TNFS_DEFAULT_PORT ? String("") : String(":") + String(u.port)) + u.path;
  String json = "{\"ok\":1";
  json += ",\"host\":\"" + jsonEscape(u.host) + "\"";
  json += ",\"port\":" + String(u.port);
  json += ",\"path\":\"" + jsonEscape(u.path) + "\"";
  json += ",\"url\":\"" + jsonEscape(currentUrl) + "\"";
  json += ",\"ip\":\"" + jsonEscape(ip.toString()) + "\"";
  json += ",\"count\":" + String(count);
  json += ",\"limit\":" + String(limit);
  json += ",\"offset\":" + String(offset);
  json += ",\"skipped\":" + String(skipped);
  json += ",\"nextOffset\":" + String(offset + count);
  json += ",\"prevOffset\":" + String(offset > limit ? offset - limit : 0);
  json += ",\"hasMore\":" + String(hasMore ? 1 : 0);
  json += ",\"eof\":" + String(eof ? 1 : 0);
  json += ",\"elapsedMs\":" + String((unsigned long)(millis() - t0));
  json += ",\"items\":" + items;
  json += "}";
  server.send(200, "application/json", json);
}



// ===== SETUP / LOOP =====


// ===== WEB-ATR FAST LOAD FIX2 DEFINITIONS =====
// Definiciones ubicadas antes de setup() para evitar errores de link en Arduino.
void webAtrCacheInvalidateAll() {
  for (int i = 0; i < WEB_ATR_SECTOR_CACHE_COUNT; i++) {
    g_webAtrCache[i].valid = false;
    g_webAtrCache[i].readAhead = false;
    g_webAtrCache[i].dev = 0;
    g_webAtrCache[i].sec = 0;
    g_webAtrCache[i].len = 0;
    g_webAtrCache[i].stamp = 0;
    g_webAtrCache[i].name[0] = '\0';
  }
  for (int i = 0; i < WEB_ATR_MAX_UNITS; i++) {
    webAtrCloseOpenFileIndex(i);
  }
}

void webAtrCacheInvalidateDev(uint8_t dev) {
  for (int i = 0; i < WEB_ATR_SECTOR_CACHE_COUNT; i++) {
    if (g_webAtrCache[i].valid && g_webAtrCache[i].dev == dev) {
      g_webAtrCache[i].valid = false;
    }
  }
  webAtrCloseOpenFileDev(dev);
}

/* F49Z43: se elimina helper con WebAtrSectorCacheEntry en la firma.
   Arduino genera prototipos antes de los structs y eso rompe la compilacion. */
#define WEB_ATR_CACHE_NAME_EQ(e, name) ((e).valid && strncmp((e).name, (name).c_str(), sizeof((e).name)) == 0)

bool webAtrCacheGet(uint8_t dev, uint16_t sec, const String& name, uint8_t* buf, uint16_t &len) {
  if (!g_webAtrFastLoadEnabled || !buf) return false;

  for (int i = 0; i < WEB_ATR_SECTOR_CACHE_COUNT; i++) {
    WebAtrSectorCacheEntry& e = g_webAtrCache[i];
    if (e.valid && e.dev == dev && e.sec == sec && WEB_ATR_CACHE_NAME_EQ(e, name)) {
      len = e.len;
      memcpy(buf, e.data, len);
      e.stamp = ++g_webAtrCacheClock;
      g_webAtrCacheHit++;
      if (e.readAhead) {
        g_webAtrReadAheadHit++;
        e.readAhead = false;
      }
      return true;
    }
  }

  g_webAtrCacheMiss++;
  return false;
}

void webAtrCachePut(uint8_t dev, uint16_t sec, const String& name, const uint8_t* buf, uint16_t len, bool readAhead) {
  if (!g_webAtrFastLoadEnabled || !buf || len == 0 || len > 256) return;

  int slot = -1;
  uint32_t oldest = 0xFFFFFFFFUL;

  for (int i = 0; i < WEB_ATR_SECTOR_CACHE_COUNT; i++) {
    WebAtrSectorCacheEntry& e = g_webAtrCache[i];

    if (e.valid && e.dev == dev && e.sec == sec && WEB_ATR_CACHE_NAME_EQ(e, name)) {
      slot = i;
      break;
    }

    if (!e.valid) {
      slot = i;
      break;
    }

    if (e.stamp < oldest) {
      oldest = e.stamp;
      slot = i;
    }
  }

  if (slot < 0) return;

  WebAtrSectorCacheEntry& e = g_webAtrCache[slot];
  e.valid = true;
  e.readAhead = readAhead;
  e.dev = dev;
  e.sec = sec;
  e.len = len;
  e.stamp = ++g_webAtrCacheClock;
  strncpy(e.name, name.c_str(), sizeof(e.name) - 1);
  e.name[sizeof(e.name) - 1] = '\0';
  memcpy(e.data, buf, len);
  g_webAtrCacheStore++;
}


bool webAtrAtariBusy() {
  if (!g_webAtrPriorityMode) return false;
  uint32_t now = millis();
  return g_webAtrLastReadMs != 0 && (uint32_t)(now - g_webAtrLastReadMs) < WEB_ATR_PRIORITY_WINDOW_MS;
}

void webAtrCloseOpenFileIndex(int idx) {
  if (idx < 0 || idx >= WEB_ATR_MAX_UNITS) return;
  if (g_webAtrOpenValid[idx]) {
    g_webAtrOpenFile[idx].close();
    g_webAtrOpenValid[idx] = false;
    g_webAtrOpenName[idx] = "";
    g_webAtrOpenPath[idx] = "";
    memset(&g_webAtrOpenMeta[idx], 0, sizeof(g_webAtrOpenMeta[idx]));
    g_webAtrCloseCount++;
  }
}

void webAtrCloseOpenFileDev(uint8_t dev) {
  int idx = webAtrUnitIndex(dev);
  if (idx < 0) return;
  webAtrCloseOpenFileIndex(idx);
}

bool webAtrEnsureOpen(uint8_t dev, const String& mountedName, File **outFile, WebAtrMeta **outMeta) {
  if (outFile) *outFile = nullptr;
  if (outMeta) *outMeta = nullptr;

  int idx = webAtrUnitIndex(dev);
  if (idx < 0 || mountedName.length() == 0) return false;

  String path = (g_webAtrMountedName[idx] == mountedName) ? webAtrResolvedPathForIndex(idx) : webAtrPathForName(mountedName);

  if (g_webAtrOpenValid[idx] &&
      g_webAtrOpenName[idx] == mountedName &&
      g_webAtrOpenPath[idx] == path &&
      g_webAtrOpenFile[idx]) {
    g_webAtrOpenReuse++;
    if (outFile) *outFile = &g_webAtrOpenFile[idx];
    if (outMeta) *outMeta = &g_webAtrOpenMeta[idx];
    return true;
  }

  webAtrCloseOpenFileIndex(idx);

  uint32_t t0 = micros();

  WebAtrMeta m;
  if (g_webAtrMountedName[idx] == mountedName) {
    if (!webAtrReadMetaForIndex(idx, m, false)) {
      g_webAtrOpenFail++;
      return false;
    }
  } else if (!webAtrReadMetaFromPath(path, m)) {
    g_webAtrOpenFail++;
    return false;
  }

  File f = SPIFFS.open(path, "r");
  if (!f) {
    g_webAtrOpenFail++;
    return false;
  }

  g_webAtrOpenFile[idx] = f;
  g_webAtrOpenMeta[idx] = m;
  g_webAtrOpenName[idx] = mountedName;
  g_webAtrOpenPath[idx] = path;
  g_webAtrOpenValid[idx] = true;
  g_webAtrOpenCount++;
  g_webAtrOpenUsLast = micros() - t0;

  if (outFile) *outFile = &g_webAtrOpenFile[idx];
  if (outMeta) *outMeta = &g_webAtrOpenMeta[idx];
  return true;
}


bool webAtrReadSectorRaw(uint8_t dev, uint16_t sec, uint8_t *buf, uint16_t &len, const String& mountedName) {
  int idx = webAtrUnitIndex(dev);
  if (idx < 0) return false;

  uint32_t tTotal0 = micros();

  File *pf = nullptr;
  WebAtrMeta *pm = nullptr;
  if (!webAtrEnsureOpen(dev, mountedName, &pf, &pm) || !pf || !pm) {
    return false;
  }

  uint32_t off = 0;
  if (!webAtrOffsetForSector(sec, *pm, off, len)) return false;

  uint32_t tSeek0 = micros();
  if (!pf->seek(off, SeekSet)) {
    webAtrCloseOpenFileIndex(idx);
    return false;
  }
  g_webAtrSeekUsLast = micros() - tSeek0;

  uint32_t tRead0 = micros();
  int n = pf->read(buf, len);
  g_webAtrDataReadUsLast = micros() - tRead0;

  uint32_t dt = micros() - tTotal0;
  g_webAtrSdReadUsLast = dt;
  g_webAtrTotalReadUsLast = dt;
  if (dt > g_webAtrSdReadUsMax) g_webAtrSdReadUsMax = dt;
  if (dt > g_webAtrTotalReadUsMax) g_webAtrTotalReadUsMax = dt;

  return n == (int)len;
}

void webAtrPrefetchNext(uint8_t dev, uint16_t sec) {
  if (!g_webAtrFastLoadEnabled || !g_webAtrReadAheadEnabled) return;
  if (g_webAtrReadAheadCount == 0) return;

  int idx = webAtrUnitIndex(dev);
  if (idx < 0) return;

  String mountedName = g_webAtrMountedName[idx];
  if (mountedName.length() == 0) return;

  uint8_t count = g_webAtrReadAheadCount;
  if (count > WEB_ATR_READAHEAD_MAX_COUNT) count = WEB_ATR_READAHEAD_MAX_COUNT;

  for (uint8_t i = 1; i <= count; i++) {
    uint16_t nextSec = sec + i;
    uint8_t tmp[256];
    uint16_t len = 0;

    if (webAtrCacheGet(dev, nextSec, mountedName, tmp, len)) {
      continue;
    }

    if (webAtrReadSectorRaw(dev, nextSec, tmp, len, mountedName)) {
      webAtrCachePut(dev, nextSec, mountedName, tmp, len, true);
      g_webAtrReadAhead++;
    } else {
      g_webAtrReadAheadFail++;
      break;
    }

    // Ceder mínimo si se precargan varios sectores.
    if (count > 1) yield();
  }
}

void handleAtrFastLoadSet() {
  if (server.hasArg("enabled")) {
    String v = server.arg("enabled");
    v.toLowerCase();
    g_webAtrFastLoadEnabled = !(v == "0" || v == "false" || v == "off");
  }

  if (server.hasArg("readahead")) {
    String v = server.arg("readahead");
    v.toLowerCase();
    g_webAtrReadAheadEnabled = !(v == "0" || v == "false" || v == "off");
  }

  if (server.hasArg("ra")) {
    int n = server.arg("ra").toInt();
    if (n < 0) n = 0;
    if (n > WEB_ATR_READAHEAD_MAX_COUNT) n = WEB_ATR_READAHEAD_MAX_COUNT;
    g_webAtrReadAheadCount = (uint8_t)n;
    g_webAtrReadAheadEnabled = (g_webAtrReadAheadCount > 0);
  }

  if (server.hasArg("priority")) {
    String v = server.arg("priority");
    v.toLowerCase();
    g_webAtrPriorityMode = !(v == "0" || v == "false" || v == "off");
  }

  if (server.hasArg("clear") && server.arg("clear") == "1") {
    webAtrCacheInvalidateAll();
    g_webAtrCacheHit = 0;
    g_webAtrCacheMiss = 0;
    g_webAtrCacheStore = 0;
    g_webAtrReadAhead = 0;
    g_webAtrReadAheadHit = 0;
    g_webAtrReadAheadFail = 0;
    g_webAtrSdReadUsLast = 0;
    g_webAtrSdReadUsMax = 0;
    g_webAtrOpenReuse = 0;
    g_webAtrOpenCount = 0;
    g_webAtrOpenFail = 0;
    g_webAtrCloseCount = 0;
    g_webAtrOpenUsLast = 0;
    g_webAtrSeekUsLast = 0;
    g_webAtrDataReadUsLast = 0;
    g_webAtrTotalReadUsLast = 0;
    g_webAtrTotalReadUsMax = 0;
  }

  handleAtrStatus();
}


void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("============================================================"));
  Serial.println(F("[BOOT] Atari SIO Controller F12 SD - entrada a setup()"));
  Serial.println(F("[BOOT] Serial OK, inicializando almacenamiento/NVS/WiFi..."));
  Serial.println(F("============================================================"));
  initAtasciiGlyphFs();

  Serial.println(F("\n=== ESP32-S3 MASTER BRIDGE JPEG_ONLY (RP2040 <-> SLAVE XF551) + WEB + AUTO + CFG ==="));
  Serial.print(F("[BUILD] MASTER="));
  Serial.println(MASTER_BUILD);
#if defined(ESP32)
  bool psramOk = psramFound();
  Serial.printf("[MEM] heap=%lu maxAlloc=%lu psramFound=%d psramTotal=%lu freePsram=%lu\n",
                (unsigned long)ESP.getFreeHeap(),
                (unsigned long)ESP.getMaxAllocHeap(),
                psramOk ? 1 : 0,
                (unsigned long)ESP.getPsramSize(),
                (unsigned long)ESP.getFreePsram());
#endif
  Serial.printf("[STORAGE] Backend=%s SD_CS=%d SCK=%d MOSI=%d MISO=%d ready=%d total=%lu used=%lu free=%lu\n",
                WEB_STORAGE_NAME,
#if WEB_STORAGE_USE_SD
                WEB_SD_CS, WEB_SD_SCK, WEB_SD_MOSI, WEB_SD_MISO,
#else
                -1, -1, -1, -1,
#endif
                webAtrFsReady() ? 1 : 0,
                (unsigned long)SPIFFS.totalBytes(),
                (unsigned long)SPIFFS.usedBytes(),
                (unsigned long)(SPIFFS.totalBytes() > SPIFFS.usedBytes() ? SPIFFS.totalBytes() - SPIFFS.usedBytes() : 0));

  // init arrays
  for (int i = 0; i < 4; i++) {
    slaves[i].present = false;
    slaves[i].supports256 = false;
    memset(slaves[i].mac, 0, 6);
    slaves[i].lastSeen = 0;
    slaves[i].physicalDev = 0;

    g_driveTiming[i].lastAckMs = 0;
    g_driveTiming[i].avgAckMs = 0;
    g_driveTiming[i].autoEnabled = true;
  }
  clearLastMasterOpLocal();
  casAutoResetAnalysis();

  loadConfigFromNvs();
  loadCasConfigFromNvs();
  loadPrinterConfigFromNvs();
  webAtrRefreshPresence();
  logf("[WEB-ATR] init enabled=%u mask=0x%02X force=0x%02X file=%u fsFree=%lu",
       WEB_ATR_ENABLED ? 1 : 0, (unsigned)(WEB_ATR_DEV_MASK & DRIVE_UI_MAX_MASK),
       (unsigned)(WEB_ATR_FORCE_MASK & DRIVE_UI_MAX_MASK), g_webAtrFilePresent ? 1 : 0,
       (unsigned long)webAtrFsFreeBytes());
  loadMapFromNvs();

  // UART con RP siempre arranca BOOT
  Serial2.begin(BOOT_UART_BAUD, SERIAL_8N1, PIN_RP_RX, PIN_RP_TX);
  g_currentUartBaud = BOOT_UART_BAUD;

  // F49Z35: asegura que el RP2040 quede en STOP al arrancar el MASTER.
  casSendControlToRP(2, 600, 0);

  beginBtDiskUart();
  g_btDiskUartReady = true;
  logf("[BT] UART listo baud=%lu rx=%d tx=%d btDisk=%u mask=0x%02X force=0x%02X sio2pc=%u mask=0x%02X force=0x%02X",
       (unsigned long)BT_DISK_UART_BAUD_CFG, PIN_BT_RX, PIN_BT_TX,
       BT_DISK_ENABLED ? 1 : 0, (unsigned)BT_DISK_DEV_MASK, (unsigned)BT_DISK_FORCE_MASK,
       BT_SIO2PC_ENABLED ? 1 : 0, (unsigned)BT_SIO2PC_DEV_MASK, (unsigned)BT_SIO2PC_FORCE_MASK);

  // V120: arranque WiFi robusto para ESP32-S3.
  // Primero levanta el AP local, después se intenta STA de forma diferida.
  startMasterSoftApS3Safe();

  Serial.print("[WIFI] AP IP: ");
  Serial.println(WiFi.softAPIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[MASTER] ERROR: esp_now_init fallo"));
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);

  // aplicar cfg a slaves + rp al inicio
  broadcastSlaveCfg();
  pushCfgToRPAndMaybeSwitchUart();
  sendTimingUpdateToRP();
  sendPrinterCfgToRP();

  server.on("/", handleRoot);
  server.on("/disk", handleRoot);
  server.on("/printer", handleRoot);
  server.on("/upload", handleRoot);
  server.on("/library", handleRoot);
  server.on("/glyphs", handleRoot);
  server.on("/tnfs", handleRoot);
  // F33: aliases para Apariencia / Configuración visual.
  // Sin estos handlers, /settings caía al router dinámico y devolvía {ok:false,error:"not found"}.
  server.on("/settings", handleRoot);
  server.on("/appearance", handleRoot);
  server.on("/apariencia", handleRoot);

  // V28: aliases compatibles con la ruta anterior /editor/*.
  // Sin esto, /editor/glyphs podia caer al router API/notFound y no cargaba la pantalla nueva.
  server.on("/editor/disks", handleRoot);
  server.on("/editor/disk", handleRoot);
  server.on("/editor/web-atr", handleRoot);
  server.on("/editor/webatr", handleRoot);
  server.on("/editor/library", handleRoot);
  server.on("/editor/printer", handleRoot);
  server.on("/editor/bt", handleRoot);
  server.on("/editor/glyphs", handleRoot);
  server.on("/editor/glifos", handleRoot);
  server.on("/editor/tnfs", handleRoot);
  server.on("/editor/cassette", handleRoot);
  server.on("/editor/cas", handleRoot);
  server.on("/editor/settings", handleRoot);
  server.on("/editor/appearance", handleRoot);
  server.on("/editor/apariencia", handleRoot);

  server.on("/config", [](){ server.sendHeader("Location", "/disk", true); server.send(302, "text/plain", ""); });
  server.on("/favicon.ico", HTTP_GET, [](){
    server.sendHeader("Cache-Control", "max-age=86400");
    server.send(204, "image/x-icon", "");
  });
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/fs", HTTP_GET, handleApiFsDiag);
  server.on("/covers.json", HTTP_GET, handleCoverCatalog);
  server.on("/covers_libretro.json", HTTP_GET, handleCoverCatalog);
  server.on("/api/covers/resolve", HTTP_GET, handleApiCoverResolve);
  server.on("/api/covers/search", HTTP_GET, handleApiCoverSearch);
  server.on("/api/covers/user", HTTP_GET, handleApiCoversUser);
  server.on("/api/covers/set", HTTP_ANY, handleApiCoverSet);
  server.on("/api/covers/delete", HTTP_ANY, handleApiCoverDelete);
  server.on("/api/covers/upload", HTTP_POST, handleApiCoverUploadDone, handleApiCoverUploadStream);
  server.on("/api/covers/mini_upload", HTTP_POST, handleApiCoverMiniUploadDone, handleApiCoverMiniUploadStream);
  server.on("/api/covers/thumb", HTTP_GET, handleApiCoverThumb);
  server.on("/api/covers/image", HTTP_GET, handleApiCoverImage);
  server.on("/api/wifi", HTTP_GET, handleApiWifiStatus);
  server.on("/api/tnfs/servers", HTTP_GET, handleApiTnfsServers);
  server.on("/api/tnfs/add", HTTP_ANY, handleApiTnfsAdd);
  server.on("/api/tnfs/delete", HTTP_ANY, handleApiTnfsDelete);
  server.on("/api/tnfs/list", HTTP_GET, handleApiTnfsList);
  server.on("/api/tnfs/fetch_mount", HTTP_GET, handleApiTnfsFetchMount);
  server.on("/api/wifi", HTTP_POST, handleApiWifiSet);
  server.on("/set_wifi", HTTP_GET, handleApiWifiSet);
  // Android REST API V14 SLIM
  server.on("/api/drives", HTTP_GET, handleApiDrives);
  server.on("/api/library", HTTP_GET, handleApiLibrary);
  server.on("/api/library/scan_console", HTTP_GET, handleApiLibraryScanConsole);
  server.on("/api/library_index_json", HTTP_GET, handleApiLibraryIndexJson);
  server.on("/api/drives/add", HTTP_POST, handleApiDriveAdd);
  server.onNotFound(handleApiDynamicRouter);

  server.on("/api/bt/status", HTTP_GET, handleBtDiskStatus);
  server.on("/api/drives/visible", HTTP_GET, handleDriveVisibleStatus);
  server.on("/set_drive_visible", HTTP_GET, handleDriveVisibleSet);
  server.on("/add_drive_visible", HTTP_GET, handleDriveVisibleAdd);
  server.on("/remove_drive_visible", HTTP_GET, handleDriveVisibleRemove);
  server.on("/set_bt_disk", HTTP_GET, handleBtDiskSet);
  server.on("/set_bt_sio2pc", HTTP_GET, handleBtSio2pcSet);
  server.on("/atr", handleRoot);
  server.on("/cassette", handleRoot);
  server.on("/cas", handleRoot);
  server.on("/bt", handleRoot);
  server.on("/api/atr/status", HTTP_GET, handleAtrStatus);
  server.on("/api/cas/status", HTTP_GET, handleCasStatus);
  server.on("/api/cas/analyze", HTTP_GET, handleCasAnalyze);
  server.on("/api/cas/profile", HTTP_GET, handleCasProfile);
  server.on("/cas/profile", HTTP_GET, handleCasProfile);
  server.on("/cas/mount", HTTP_GET, handleCasMount);
  server.on("/cas/unmount", HTTP_GET, handleCasUnmount);
  server.on("/cas/play", HTTP_GET, handleCasPlay);
  server.on("/cas/stop", HTTP_GET, handleCasStop);
  server.on("/cas/rewind", HTTP_GET, handleCasRewind);
  server.on("/cas/pause", HTTP_GET, handleCasPause);
  server.on("/cas/resume", HTTP_GET, handleCasResume);
  server.on("/cas/seek_back", HTTP_GET, handleCasSeekBack);
  server.on("/cas/download", HTTP_GET, handleCasDownload);
  server.on("/atr/fastload", HTTP_ANY, handleAtrFastLoadSet);
  server.on("/set_webatr", HTTP_GET, handleSetWebAtr);
  server.on("/atr/delete", HTTP_GET, handleAtrDelete);
  server.on("/atr/unmount", HTTP_GET, handleAtrUnmount);
  server.on("/atr/upload", HTTP_POST, handleAtrUploadDone, handleAtrUploadStream);
  server.on("/set_timing", HTTP_GET, handleSetTiming);
  server.on("/set_prefetch", HTTP_GET, handleSetPrefetch);
  server.on("/set_auto", HTTP_GET, handleSetAuto);
  server.on("/set_comm", HTTP_GET, handleSetComm);
  server.on("/set_verify", HTTP_GET, handleSetVerify);
  server.on("/set_logic", HTTP_GET, handleSetLogic);
  server.on("/set_printer", HTTP_GET, handleSetPrinter);
  server.on("/printer_test", HTTP_GET, handlePrinterTest);
  server.on("/printer_ipp_test", HTTP_GET, handlePrinterIppTest);
  server.on("/printer_virtual_test", HTTP_GET, handlePrinterVirtualTest);
  server.on("/printer_atascii_table_test", HTTP_GET, handlePrinterAtasciiTableTest);
  server.on("/printer_buffer_print", HTTP_GET, handlePrinterBufferPrint);
  server.on("/printer_clear", HTTP_GET, handlePrinterClear);
  server.on("/glyphs_export", HTTP_GET, handleGlyphsExport);
  server.on("/glyphs_import", HTTP_POST, handleGlyphsImport);
  server.on("/glyphs_import_compact", HTTP_POST, handleGlyphsImportCompact);
  server.on("/glyph_get", HTTP_GET, handleGlyphGet);
  server.on("/glyph_set", HTTP_GET, handleGlyphSet);
  server.on("/glyph_restore", HTTP_GET, handleGlyphRestore);
  server.begin();
  Serial.println("[WEB] Servidor HTTP iniciado en puerto 80");

  // V120: no bloquear ni matar el AP durante el arranque.
  // La conexión STA hacia la red de la impresora se inicia unos segundos después.
  deferStaConnectAfterWebStart();
}

void loop() {
  // F46: durante ráfagas BT-SIO2PC priorizamos el polling del RP para reducir latencia entre sectores.
  if (btSio2pcPreferSioOverWeb()) {
    g_btSio2pcSioPrioritySkips++;
  } else {
    server.handleClient();
  }
  serviceDeferredConfigSaves();
  serviceDeferredStaConnect();
  startMdnsIfStaReady();
  servicePrinterManualPrintRequest();
  serviceCasManualPlayback();

  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
    if (slaves[i].present && (now - slaves[i].lastSeen > 60000)) {
      slaves[i].present = false;
      int di = findDeviceSlotByMac(slaves[i].mac);
      if (di >= 0) g_devices[di].present = false;
    }
  }

  if (!BT_SIO2PC_ENABLED) pollBtDiskBridge();
  pollUartFromRP();
  servicePrinterHttpBuffer();
  servicePrinterVirtualBuffer();
  delay(0);
}


static void fsDiagAppendDirListingJson(String& j, const char* field, const char* dirPath, uint8_t maxItems) {
  j += ",\"";
  j += field;
  j += "\":[";

  bool first = true;
  uint8_t count = 0;
  File root = SPIFFS.open(dirPath, "r");
  if (root && root.isDirectory()) {
    File f = root.openNextFile();
    while (f && count < maxItems) {
      String name = f.name();
      name.replace("\\", "/");
      if (!first) j += ",";
      first = false;
      j += "{\"name\":\"" + jsonEscape(name) + "\"";
      j += ",\"dir\":" + String(f.isDirectory() ? 1 : 0);
      j += ",\"size\":" + String((unsigned long)f.size());
      j += "}";
      count++;
      f.close();
      f = root.openNextFile();
    }
  }
  if (root) root.close();
  j += "]";
}

static bool fsDiagWriteTest(String& err) {
  err = "";
  if (!webAtrFsReady()) { err = "STORAGE_NOT_READY"; return false; }
  if (!webLibraryEnsureConfigDir()) { err = "CONFIG_DIR_FAIL"; return false; }

  const char* testPath = "/CONFIG/_write_test.tmp";
  File f = SPIFFS.open(testPath, "w");
  if (!f) { err = "OPEN_WRITE_FAIL"; return false; }
  size_t w = f.print("OK");
  f.flush();
  f.close();
  if (w != 2) { err = "WRITE_SIZE_FAIL"; return false; }
  if (!SPIFFS.exists(testPath)) { err = "WRITE_VERIFY_EXISTS_FAIL"; return false; }
  SPIFFS.remove(testPath);
  return true;
}

static void handleApiFsDiag() {
  bool ready = webAtrFsReady();
  uint64_t total = ready ? SPIFFS.totalBytes() : 0;
  uint64_t used  = ready ? SPIFFS.usedBytes() : 0;
  uint64_t freeB = (total > used) ? (total - used) : 0;

  bool rootDir = false;
  bool atrDir = false;
  bool casDir = false;
  bool configDirBefore = false;
  bool configMkdirOk = false;
  bool configDirAfter = false;
  bool writeTestOk = false;
  String writeTestErr;

  if (ready) {
    File root = SPIFFS.open("/", "r");
    rootDir = root && root.isDirectory();
    if (root) root.close();
    atrDir = webLibraryDirExists("/ATR");
    casDir = webLibraryDirExists("/CAS");
    configDirBefore = webLibraryDirExists("/CONFIG");
    configMkdirOk = webLibraryEnsureConfigDir();
    configDirAfter = webLibraryDirExists("/CONFIG");
    writeTestOk = fsDiagWriteTest(writeTestErr);
  }

  String j = "{";
  j += "\"ok\":1";
  j += ",\"fs\":\"" + String(WEB_STORAGE_NAME) + "\"";
  j += ",\"ready\":" + String(ready ? 1 : 0);
  j += ",\"rootDir\":" + String(rootDir ? 1 : 0);
  j += ",\"atrDir\":" + String(atrDir ? 1 : 0);
  j += ",\"casDir\":" + String(casDir ? 1 : 0);
  j += ",\"configDirBefore\":" + String(configDirBefore ? 1 : 0);
  j += ",\"configMkdirOk\":" + String(configMkdirOk ? 1 : 0);
  j += ",\"configDirAfter\":" + String(configDirAfter ? 1 : 0);
  j += ",\"writeTestOk\":" + String(writeTestOk ? 1 : 0);
  j += ",\"writeTestErr\":\"" + jsonEscape(writeTestErr) + "\"";
  j += ",\"total\":" + String((unsigned long)total);
  j += ",\"used\":" + String((unsigned long)used);
  j += ",\"free\":" + String((unsigned long)freeB);
  j += ",\"build\":\"" + String(MASTER_BUILD) + "\"";
  if (ready) {
    fsDiagAppendDirListingJson(j, "root", "/", 24);
    fsDiagAppendDirListingJson(j, "atr", "/ATR", 24);
    fsDiagAppendDirListingJson(j, "cas", "/CAS", 24);
    fsDiagAppendDirListingJson(j, "config", "/CONFIG", 24);
  }
  j += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}


void handleApiCoverImage() {
  String url = server.hasArg("url") ? server.arg("url") : "";
  String name = server.hasArg("name") ? server.arg("name") : "";
  String cacheKey = coverNormalizeKey(name.length() ? name : url);

  // 1) Si existe una carátula local en SD para ese nombre, se sirve directamente.
  if (name.length()) {
    String localPath;
    if (coverLocalFindPathForKey(cacheKey, localPath) && coverStreamLocalPath(localPath)) return;
  }

  if (!url.length() && name.length()) {
    String keyUser, titleUser, imageUser;
    if (coverUserResolve(name, keyUser, titleUser, imageUser)) {
      url = imageUser;
      cacheKey = keyUser;
    } else {
      String key, title, image;
      if (coverFindLibretro(name, key, title, image)) {
        url = image;
        cacheKey = key;
      }
    }
  }

  url.trim();
  url.replace(" ", "%20");

  // 2) Si la URL resuelta ya apunta a /COVERS, se carga desde SD.
  if (coverIsLocalPath(url)) {
    if (coverStreamAnyLocalPath(url)) return;
    server.send(404, "text/plain", "LOCAL_COVER_NOT_FOUND");
    return;
  }

  if (!(url.startsWith("https://") || url.startsWith("http://"))) {
    server.send(400, "text/plain", "INVALID_URL");
    return;
  }

  // 3) Si no existe local, se descarga a /COVERS y se sirve desde SD.
  //    Esto hace caché automático de Libretro/proxy cuando hay STA con internet.
  if (cacheKey.length()) {
    String localPath, contentType, err;
    if (coverDownloadUrlToSd(cacheKey, url, localPath, contentType, err)) {
      coverStreamLocalPath(localPath);
      return;
    }
    // Si no se pudo cachear, seguimos con proxy directo para no romper la visualización mientras haya internet.
    logf("[COVERS] cache miss/fail key=%s err=%s", cacheKey.c_str(), err.c_str());
  }

  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "text/plain", "STA_WIFI_NOT_CONNECTED");
    return;
  }

  HTTPClient http;
  WiFiClientSecure secureClient;
  WiFiClient plainClient;
  bool https = url.startsWith("https://");

  if (https) {
    secureClient.setInsecure();
    if (!http.begin(secureClient, url)) {
      server.send(500, "text/plain", "HTTP_BEGIN_FAILED");
      return;
    }
  } else {
    if (!http.begin(plainClient, url)) {
      server.send(500, "text/plain", "HTTP_BEGIN_FAILED");
      return;
    }
  }

  http.setTimeout(7000); // F49Z41
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();

  if (code != HTTP_CODE_OK) {
    String msg = "HTTP_GET_FAILED code=" + String(code);
    http.end();
    server.send(502, "text/plain", msg);
    return;
  }

  String contentType = http.header("Content-Type");
  if (!contentType.length()) contentType = "image/png";
  int len = http.getSize();

  if (len > 0) server.setContentLength(len);
  else server.setContentLength(CONTENT_LENGTH_UNKNOWN);

  server.send(200, contentType, "");

  WiFiClient* stream = http.getStreamPtr();
  WiFiClient out = server.client();

  uint8_t buf[1024];
  int remaining = len;
  unsigned long lastData = millis();

  while (http.connected()) {
    size_t avail = stream->available();
    if (avail) {
      size_t n = avail;
      if (n > sizeof(buf)) n = sizeof(buf);
      if (remaining > 0 && (int)n > remaining) n = remaining;
      int r = stream->readBytes(buf, n);
      if (r > 0) {
        out.write(buf, r);
        lastData = millis();
        if (remaining > 0) {
          remaining -= r;
          if (remaining <= 0) break;
        }
      }
    } else {
      if (remaining == 0) break;
      if (millis() - lastData > 7000) break;
      delay(1);
    }
  }

  http.end();
}

void handleApiCoverThumb() {
  String name = server.hasArg("name") ? server.arg("name") : server.arg("key");
  String url = server.hasArg("url") ? server.arg("url") : "";
  String key = coverNormalizeKey(name.length() ? name : url);

  // F12 performance:
  // El endpoint de miniaturas NO descarga imágenes remotas ni hace proxy HTTP.
  // Solo sirve /MINI_COVERS o /COVERS locales desde la SD. Esto evita que la página
  // se quede pegada cuando hay muchos juegos o una red externa lenta.
  if (key.length()) {
    String miniPath;
    if (coverMiniFindPathForKey(key, miniPath) && coverStreamAnyLocalPath(miniPath)) return;

    String localPath;
    if (coverLocalFindPathForKey(key, localPath) && coverStreamAnyLocalPath(localPath)) return;
  }

  // Si la URL ya es local, también se permite servirla desde SD.
  if (url.length()) {
    url.trim();
    url.replace("\\", "/");
    if (coverIsMiniLocalPath(url) || coverIsLocalPath(url)) {
      if (coverStreamAnyLocalPath(url)) return;
    }
  }

  coverSendLocalNotFound("NO_LOCAL_THUMB");
}

void handleApiCoverMiniUploadDone() {
  if (g_coverMiniUploadFile) g_coverMiniUploadFile.close();

  if (g_coverMiniUploadError.length()) {
    if (g_coverMiniUploadTmpPath.length() && SPIFFS.exists(g_coverMiniUploadTmpPath)) SPIFFS.remove(g_coverMiniUploadTmpPath);
    String err = g_coverMiniUploadError;
    g_coverMiniUploadError = "";
    g_coverMiniUploadTmpPath = "";
    g_coverMiniUploadFinalPath = "";
    server.send(500, "application/json", String("{\"ok\":0,\"reason\":\"") + jsonEscape(err) + "\"}");
    return;
  }

  if (!g_coverMiniUploadFinalPath.length() || !SPIFFS.exists(g_coverMiniUploadFinalPath)) {
    server.send(500, "application/json", "{\"ok\":0,\"reason\":\"MINI_UPLOAD_NOT_SAVED\"}");
    return;
  }

  String j = "{";
  j += "\"ok\":1";
  j += ",\"key\":\"" + jsonEscape(g_coverMiniUploadKey) + "\"";
  j += ",\"thumb\":\"" + jsonEscape(g_coverMiniUploadFinalPath) + "\"";
  j += ",\"source\":\"mini-sd\"";
  j += "}";

  g_coverMiniUploadTmpPath = "";
  g_coverMiniUploadFinalPath = "";
  g_coverMiniUploadKey = "";
  server.send(200, "application/json", j);
}

void handleApiCoverMiniUploadStream() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_coverMiniUploadError = "";
    String name = server.hasArg("name") ? server.arg("name") : server.arg("key");
    g_coverMiniUploadKey = coverNormalizeKey(name.length() ? name : up.filename);
    if (!g_coverMiniUploadKey.length()) g_coverMiniUploadKey = coverNormalizeKey(up.filename);
    if (!coverEnsureMiniDir()) { g_coverMiniUploadError = "MINI_COVERS_DIR_NOT_READY"; return; }

    String ext = coverExtFromMimeOrName(up.type, up.filename);
    // Las miniaturas generadas por navegador normalmente son JPEG/WebP.
    if (!(ext == ".jpg" || ext == ".jpeg" || ext == ".webp" || ext == ".png")) ext = ".jpg";
    String base = coverSanitizeKeyForFile(g_coverMiniUploadKey);
    g_coverMiniUploadTmpPath = String(USER_MINI_COVERS_DIR) + "/" + base + ".tmp";
    g_coverMiniUploadFinalPath = String(USER_MINI_COVERS_DIR) + "/" + base + ext;
    coverRemoveMiniFilesForKey(g_coverMiniUploadKey);
    if (SPIFFS.exists(g_coverMiniUploadTmpPath)) SPIFFS.remove(g_coverMiniUploadTmpPath);
    g_coverMiniUploadFile = SPIFFS.open(g_coverMiniUploadTmpPath, "w");
    if (!g_coverMiniUploadFile) g_coverMiniUploadError = "OPEN_MINI_TMP_FAILED";
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (!g_coverMiniUploadError.length() && g_coverMiniUploadFile) {
      // Limite duro para miniaturas: 256 KB.
      if (g_coverMiniUploadFile.size() + up.currentSize > 262144) {
        g_coverMiniUploadError = "MINI_TOO_LARGE";
        return;
      }
      size_t w = g_coverMiniUploadFile.write(up.buf, up.currentSize);
      if (w != up.currentSize) g_coverMiniUploadError = "MINI_WRITE_FAILED";
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_coverMiniUploadFile) g_coverMiniUploadFile.close();
    if (!g_coverMiniUploadError.length()) {
      if (SPIFFS.exists(g_coverMiniUploadFinalPath)) SPIFFS.remove(g_coverMiniUploadFinalPath);
      if (!SPIFFS.rename(g_coverMiniUploadTmpPath, g_coverMiniUploadFinalPath)) {
        File in = SPIFFS.open(g_coverMiniUploadTmpPath, "r");
        File out = SPIFFS.open(g_coverMiniUploadFinalPath, "w");
        uint8_t buf[1024];
        bool ok = in && out;
        while (ok && in.available()) {
          size_t r = in.read(buf, sizeof(buf));
          if (r && out.write(buf, r) != r) ok = false;
        }
        if (in) in.close();
        if (out) out.close();
        if (SPIFFS.exists(g_coverMiniUploadTmpPath)) SPIFFS.remove(g_coverMiniUploadTmpPath);
        if (!ok) g_coverMiniUploadError = "MINI_RENAME_FAILED";
      }
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (g_coverMiniUploadFile) g_coverMiniUploadFile.close();
    if (g_coverMiniUploadTmpPath.length() && SPIFFS.exists(g_coverMiniUploadTmpPath)) SPIFFS.remove(g_coverMiniUploadTmpPath);
    g_coverMiniUploadError = "MINI_UPLOAD_ABORTED";
  }
}

void handleApiCoverUploadDone() {
  if (g_coverUploadFile) g_coverUploadFile.close();

  if (g_coverUploadError.length()) {
    if (g_coverUploadTmpPath.length() && SPIFFS.exists(g_coverUploadTmpPath)) SPIFFS.remove(g_coverUploadTmpPath);
    String err = g_coverUploadError;
    g_coverUploadError = "";
    g_coverUploadTmpPath = "";
    g_coverUploadFinalPath = "";
    server.send(500, "application/json", String("{\"ok\":0,\"reason\":\"") + jsonEscape(err) + "\"}");
    return;
  }

  if (!g_coverUploadFinalPath.length() || !SPIFFS.exists(g_coverUploadFinalPath)) {
    server.send(500, "application/json", "{\"ok\":0,\"reason\":\"UPLOAD_NOT_SAVED\"}");
    return;
  }

  String body = coverUserLoadJson();
  String next = coverUserUpsertJson(body, g_coverUploadKey, g_coverUploadTitle.length() ? g_coverUploadTitle : g_coverUploadName, g_coverUploadFinalPath);
  bool ok = coverUserSaveJson(next);

  String j = "{";
  j += "\"ok\":" + String(ok ? 1 : 0);
  j += ",\"key\":\"" + jsonEscape(g_coverUploadKey) + "\"";
  j += ",\"title\":\"" + jsonEscape(g_coverUploadTitle.length() ? g_coverUploadTitle : g_coverUploadName) + "\"";
  j += ",\"image\":\"" + jsonEscape(g_coverUploadFinalPath) + "\"";
  j += ",\"source\":\"upload-sd\"";
  j += ",\"cached\":1";
  if (!ok) j += ",\"reason\":\"WRITE_FAILED\"";
  j += "}";

  g_coverUploadTmpPath = "";
  g_coverUploadFinalPath = "";
  g_coverUploadName = "";
  g_coverUploadTitle = "";
  g_coverUploadKey = "";
  server.send(ok ? 200 : 500, "application/json", j);
}

void handleApiCoverUploadStream() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_coverUploadError = "";
    g_coverUploadName = server.hasArg("name") ? server.arg("name") : "";
    g_coverUploadTitle = server.hasArg("title") ? server.arg("title") : g_coverUploadName;
    g_coverUploadKey = coverNormalizeKey(g_coverUploadName.length() ? g_coverUploadName : up.filename);
    g_coverUploadContentType = up.type;
    if (!g_coverUploadKey.length()) g_coverUploadKey = coverNormalizeKey(up.filename);
    if (!coverEnsureDir()) { g_coverUploadError = "COVERS_DIR_NOT_READY"; return; }

    String ext = coverExtFromMimeOrName(up.type, up.filename);
    String base = coverSanitizeKeyForFile(g_coverUploadKey);
    g_coverUploadTmpPath = String(USER_COVERS_DIR) + "/" + base + ".tmp";
    g_coverUploadFinalPath = String(USER_COVERS_DIR) + "/" + base + ext;
    coverRemoveLocalFilesForKey(g_coverUploadKey);
    if (SPIFFS.exists(g_coverUploadTmpPath)) SPIFFS.remove(g_coverUploadTmpPath);
    g_coverUploadFile = SPIFFS.open(g_coverUploadTmpPath, "w");
    if (!g_coverUploadFile) g_coverUploadError = "OPEN_TMP_FAILED";
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (!g_coverUploadError.length() && g_coverUploadFile) {
      size_t w = g_coverUploadFile.write(up.buf, up.currentSize);
      if (w != up.currentSize) g_coverUploadError = "WRITE_FAILED";
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_coverUploadFile) g_coverUploadFile.close();
    if (!g_coverUploadError.length()) {
      if (SPIFFS.exists(g_coverUploadFinalPath)) SPIFFS.remove(g_coverUploadFinalPath);
      if (!SPIFFS.rename(g_coverUploadTmpPath, g_coverUploadFinalPath)) {
        File in = SPIFFS.open(g_coverUploadTmpPath, "r");
        File out = SPIFFS.open(g_coverUploadFinalPath, "w");
        uint8_t buf[1024];
        bool ok = in && out;
        while (ok && in.available()) {
          size_t r = in.read(buf, sizeof(buf));
          if (r && out.write(buf, r) != r) ok = false;
        }
        if (in) in.close();
        if (out) out.close();
        if (SPIFFS.exists(g_coverUploadTmpPath)) SPIFFS.remove(g_coverUploadTmpPath);
        if (!ok) g_coverUploadError = "RENAME_FAILED";
      }
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (g_coverUploadFile) g_coverUploadFile.close();
    if (g_coverUploadTmpPath.length() && SPIFFS.exists(g_coverUploadTmpPath)) SPIFFS.remove(g_coverUploadTmpPath);
    g_coverUploadError = "UPLOAD_ABORTED";
  }
}

