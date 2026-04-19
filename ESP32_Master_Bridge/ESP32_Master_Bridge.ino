#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <stdarg.h>

#ifndef ESP_IDF_VERSION_MAJOR
#define ESP_IDF_VERSION_MAJOR 4
#endif

Preferences prefs;

// ===== Debug =====
#define MASTER_UART_BYTE_DEBUG 0   // 1 = logea cada byte UART (NO recomendado)

// ===== Protocol =====
#define TYPE_CMD_FRAME      0x01
#define TYPE_SECTOR_CHUNK   0x10
#define TYPE_ACK            0x11
#define TYPE_NAK            0x12
#define TYPE_HELLO          0x20
#define TYPE_TIMING_UPDATE  0x30

// NUEVO CFG
#define TYPE_CFG_UPDATE     0x40
#define TYPE_CFG_ACK        0x41

#define UART_SYNC 0x55

// D1..D4
#define DEV_MIN 0x31
#define DEV_MAX 0x34

// Prefetch máximo (UI puede mandar más, acá se clampa)
#define MAX_PREFETCH_SECTORS 4

// Broadcast MAC
const uint8_t BCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Canal WiFi fijo (AP + ESP-NOW). Debe coincidir en SLAVES.
static const uint8_t WIFI_CHANNEL = 1;

// Último SLAVE conocido (fallback)
uint8_t g_lastSlave[6] = { 0 };
bool g_haveSlave = false;

// UART2 (con RP2040)
const int PIN_RP_RX = 16;  // RX2
const int PIN_RP_TX = 17;  // TX2

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
static int findSlaveSlotByMac(const uint8_t mac[6]) {
  for (int i = 0; i < 4; i++) {
    if (slaves[i].present && memcmp(slaves[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

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
uint8_t prefetchCfg[4] = { 1, 1, 1, 1 };

// ========= Tiempos SIO (µs) – ULTRA OPTIMIZADOS =========
uint16_t T_ACK_TO_COMPLETE = 120;
uint16_t T_COMPLETE_TO_DATA = 80;
uint16_t T_DATA_TO_CHK = 15;
uint16_t T_CHUNK_DELAY = 20;

static const bool g_lockFastTimings = true;
static const uint16_t kFastAckToComplete = 120;
static const uint16_t kFastCompleteToData = 80;
static const uint16_t kFastDataToChk = 15;
static const uint16_t kFastChunkDelay = 20;

// ========= Tiempos medidos por disquetera (ACK/NAK) =========
struct DriveTiming {
  uint32_t lastAckMs;
  uint32_t avgAckMs;
  bool autoEnabled;
};
DriveTiming g_driveTiming[4];

int g_autoProfile = 1;

static inline void applyFastTimingPreset() {
  T_ACK_TO_COMPLETE = kFastAckToComplete;
  T_COMPLETE_TO_DATA = kFastCompleteToData;
  T_DATA_TO_CHK = kFastDataToChk;
  T_CHUNK_DELAY = kFastChunkDelay;
}

// ========= WebServer & NVS =========
WebServer server(80);
// Preferences prefs;  // (declarado arriba)
const uint32_t CFG_MAGIC = 0xCAFEBABE;

// ========= Comm/Verify CFG (NVS/UI) =========
static const uint32_t BOOT_UART_BAUD = 460800;     // RP y Master arrancan acá
uint32_t CFG_UART_BAUD    = 460800;                // UART RP<->MASTER (configurable)
uint32_t CFG_RP_SIO_BAUD  = 19200;                 // SIO Atari<->RP (configurable)
uint32_t CFG_XF_SIO_BAUD  = 19200;                 // SIO ESP32<->XF551 (configurable)
uint32_t CFG_NET_DELAY_US = 0;                     // throttle ESP-NOW (µs)
uint8_t  CFG_VERIFY_FLAGS = 0b1110;                // all=0 boot=1 vtoc=1 verify57=1

static uint32_t g_currentUartBaud = BOOT_UART_BAUD;

// ACK de CFG desde RP
static volatile bool g_rpCfgAck = false;
static uint8_t  g_rpCfgAck_ok = 0;
static uint32_t g_rpCfgAck_uart = 0;
static uint32_t g_rpCfgAck_sio  = 0;

// ========= HTML UI =========
extern const char INDEX_HTML[] PROGMEM;
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
 <meta charset="UTF-8" />
 <meta name="viewport" content="width=device-width, initial-scale=1" />
 <title>XF551 WiFi - Panel</title>
 <style>
 :root {
 --bg: #020617;
 --accent: #38bdf8;
 --accent-soft: rgba(56, 189, 248, 0.15);
 --accent-strong: rgba(56, 189, 248, 0.45);
 --text: #e5e7eb;
 --text-soft: #9ca3af;
 --border: #1f2937;
 --danger: #f97373;
 --ok: #4ade80;
 }
 * { box-sizing: border-box; }
 body {
 margin: 0;
 font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
 background: radial-gradient(circle at top left, #0f172a 0, #020617 40%, #000 100%);
 color: var(--text);
 }
 .page { min-height: 100vh; padding: 12px; display: flex; flex-direction: column; align-items: center; }
 .app { width: 100%; max-width: 960px; }
 header { margin-bottom: 10px; text-align: center; }
 header h1 { font-size: 1.4rem; margin: 4px 0; }
 header p { font-size: 0.85rem; color: var(--text-soft); margin: 0; }
 .grid { display: grid; grid-template-columns: 1fr; gap: 10px; }
 @media (min-width: 700px) {
 .grid { grid-template-columns: minmax(0, 1.2fr) minmax(0, 1fr); }
 }
 .card {
 background: linear-gradient(135deg, rgba(15,23,42,0.96), rgba(15,23,42,0.9));
 border-radius: 14px;
 border: 1px solid var(--border);
 box-shadow: 0 16px 40px rgba(15,23,42,0.8);
 padding: 12px;
 }
 .card h2 { font-size: 1.1rem; margin: 0 0 8px; display: flex; align-items: center; gap: 6px; }
 .card h2 span.icon {
 display: inline-flex; width: 20px; height: 20px; border-radius: 999px;
 align-items: center; justify-content: center;
 background: var(--accent-soft); color: var(--accent); font-size: 0.9rem;
 }
 .card small { display: block; font-size: 0.75rem; color: var(--text-soft); margin-bottom: 6px; }
 .drives-list { display: flex; flex-direction: column; gap: 6px; }
 .drive-item {
 border-radius: 10px; border: 1px solid var(--border);
 padding: 8px; background: radial-gradient(circle at top left, rgba(15,23,42,0.9), #020617);
 display: flex; flex-direction: column; gap: 4px;
 }
 .drive-header { display: flex; justify-content: space-between; align-items: center; gap: 6px; }
 .drive-name { font-weight: 600; font-size: 0.95rem; }
 .pill {
 padding: 2px 8px; border-radius: 999px; font-size: 0.7rem;
 border: 1px solid var(--border); background: rgba(15,23,42,0.9);
 color: var(--text-soft); white-space: nowrap;
 }
 .pill.ok {
 color: var(--ok); border-color: rgba(74,222,128,0.5); background: rgba(34,197,94,0.12);
 }
 .pill.bad {
 color: var(--danger); border-color: rgba(248,113,113,0.6); background: rgba(248,113,113,0.12);
 }
 .drive-body { display: flex; flex-wrap: wrap; gap: 4px 10px; font-size: 0.75rem; color: var(--text-soft); }
 .drive-body span.label { color: var(--text-soft); }
 .drive-body span.value { color: var(--text); font-weight: 500; }
 .drive-prefetch {
 margin-top: 4px; display: flex; align-items: center; justify-content: space-between;
 gap: 6px; font-size: 0.75rem;
 flex-wrap: wrap;
 }
 .switch { position: relative; display: inline-flex; align-items: center; cursor: pointer; gap: 4px; font-size: 0.75rem; }
 .switch input { opacity: 0; width: 0; height: 0; position: absolute; }
 .switch-slider {
 width: 32px; height: 18px; background-color: #111827;
 border-radius: 999px; border: 1px solid var(--border);
 position: relative; transition: background-color 0.15s ease, border-color 0.15s ease;
 }
 .switch-slider::before {
 content: ""; position: absolute; width: 14px; height: 14px; border-radius: 999px;
 background-color: #f9fafb; left: 1px; top: 1px; transition: transform 0.15s ease;
 box-shadow: 0 1px 2px rgba(0,0,0,0.5);
 }
 .switch input:checked + .switch-slider {
 background-color: var(--accent-strong); border-color: rgba(56,189,248,0.8);
 }
 .switch input:checked + .switch-slider::before { transform: translateX(14px); }
 .small-input {
 width: 120px; padding: 4px 6px; border-radius: 6px;
 border: 1px solid var(--border); background: #020617;
 color: var(--text); font-size: 0.75rem;
 }
 .small-input:focus {
 outline: none; border-color: var(--accent); box-shadow: 0 0 0 1px rgba(56,189,248,0.5);
 }
 .timing-grid {
 display: grid; grid-template-columns: repeat(2, minmax(0, 1fr));
 gap: 6px; margin-top: 8px;
 }
 @media (min-width: 700px) {
 .timing-grid { grid-template-columns: repeat(4, minmax(0, 1fr)); }
 }
 .timing-item {
 background: rgba(15,23,42,0.85); border-radius: 8px;
 border: 1px solid var(--border); padding: 6px; font-size: 0.75rem;
 }
 .timing-item label { display: block; margin-bottom: 2px; color: var(--text-soft); }
 .timing-item input {
 width: 100%; padding: 4px 6px; border-radius: 6px;
 border: 1px solid var(--border); background: #020617;
 color: var(--text); font-size: 0.8rem;
 }
 .timing-item select{
 width: 100%; padding: 4px 6px; border-radius: 6px;
 border: 1px solid var(--border); background: #020617;
 color: var(--text); font-size: 0.8rem;
 }
 .timing-item input:focus, .timing-item select:focus {
 outline: none; border-color: var(--accent); box-shadow: 0 0 0 1px rgba(56,189,248,0.5);
 }
 .actions { margin-top: 10px; display: flex; flex-wrap: wrap; gap: 6px; }
 button {
 border: none; border-radius: 999px; padding: 8px 14px;
 font-size: 0.8rem; font-weight: 500; cursor: pointer;
 display: inline-flex; align-items: center; gap: 6px;
 background: radial-gradient(circle at top left, #0ea5e9, #0369a1);
 color: #f9fafb; box-shadow: 0 10px 25px rgba(8,47,73,0.9);
 transition: transform 0.12s ease, box-shadow 0.12s ease, filter 0.12s ease;
 }
 button:hover {
 transform: translateY(-1px); filter: brightness(1.05);
 box-shadow: 0 12px 28px rgba(8,47,73,0.9);
 }
 button:active { transform: translateY(0); box-shadow: 0 6px 18px rgba(8,47,73,0.9); }
 button.secondary {
 background: radial-gradient(circle at top left, #111827, #020617);
 color: var(--text-soft); box-shadow: none; border: 1px solid var(--border);
 }
 .status-line {
 margin-top: 8px; font-size: 0.75rem; color: var(--text-soft);
 display: flex; flex-wrap: wrap; gap: 6px; align-items: center;
 }
 .status-dot {
 width: 8px; height: 8px; border-radius: 999px;
 background: #4ade80; box-shadow: 0 0 0 5px rgba(74,222,128,0.15);
 }
 .status-dot.err {
 background: #f97373; box-shadow: 0 0 0 5px rgba(248,113,113,0.18);
 }
 .status-msg { flex: 1; min-width: 0; }
 .footer { margin-top: 10px; text-align: center; font-size: 0.7rem; color: var(--text-soft); opacity: 0.7; }
 </style>
</head>
<body>
 <div class="page">
 <div class="app">
 <header>
 <h1>XF551 WiFi · Panel Maestro</h1>
 <p>Estado de disqueteras · Tiempos SIO · Prefetch & Auto-ajuste</p>
 </header>

 <div class="grid">
 <!-- Columna izquierda: Disqueteras -->
 <section class="card">
 <h2><span class="icon">💾</span> Disqueteras</h2>
 <small>Toca una unidad para ver su MAC, densidad, prefetch y auto-ajuste.</small>
 <div id="drives" class="drives-list"></div>

 <div style="margin-top:10px;padding-top:8px;border-top:1px solid rgba(255,255,255,0.08);">
   <h3 style="margin:6px 0 2px 0; font-size:0.95rem;">Asignación de unidad lógica (por MAC)</h3>
   <small style="display:block;margin-bottom:6px;">Elige qué ESP32 (por MAC) será D1..D4. Se guarda en el master.</small>
   <div id="devices" class="drives-list"></div>
 </div>

 </section>

 <!-- Columna derecha: Configuración -->
 <section class="card">
 <h2><span class="icon">⚙️</span> Configuración avanzada</h2>
 <small>Tiempos SIO (NVS), prefetch y auto-ajuste por unidad.</small>

 <div class="timing-grid">
   <div class="timing-item">
     <label for="ackToComplete">ACK → COMPLETE (µs)</label>
     <input type="number" id="ackToComplete" min="200" step="50" />
   </div>
   <div class="timing-item">
     <label for="completeToData">COMPLETE → DATA (µs)</label>
     <input type="number" id="completeToData" min="200" step="50" />
   </div>
   <div class="timing-item">
     <label for="dataToChk">DATA → CHK (µs)</label>
     <input type="number" id="dataToChk" min="50" step="50" />
   </div>
   <div class="timing-item">
     <label for="chunkDelay">Delay entre sectores (µs)</label>
     <input type="number" id="chunkDelay" min="200" step="50" />
   </div>
 </div>

 <div style="margin-top:8px; font-size:0.8rem;">
   <span style="display:block;margin-bottom:4px;">Perfil global de auto-ajuste:</span>
   <select id="autoProfile" class="small-input">
     <option value="0">Conservador (más margen)</option>
     <option value="1">Normal</option>
     <option value="2">Agresivo (más rápido)</option>
   </select>
   <div style="font-size:0.7rem; color:var(--text-soft); margin-top:3px;">
     Cada disquetera puede activar/desactivar auto-ajuste en su tarjeta.
   </div>
 </div>

 <!-- ===== NUEVO: Comunicaciones ===== -->
 <div style="margin-top:12px; font-size:0.8rem;">
   <span style="display:block;margin-bottom:4px;">Comunicaciones</span>
 </div>

 <div class="timing-grid">
   <div class="timing-item">
     <label for="uartBaud">UART RP ↔ Master (baud)</label>
      <select id="uartBaud">
        <option value="115200">115200</option>
        <option value="230400">230400</option>
        <option value="460800">460800</option>
        <option value="921600">921600</option>
        <option value="1500000">1500000</option>
        <option value="2000000">2000000</option>
      </select>
   </div>

   <div class="timing-item">
     <label for="rpSioBaud">SIO Atari ↔ RP (baud)</label>
     <select id="rpSioBaud">
       <option value="19200">19200</option>
       <option value="38400">38400</option>
       <option value="57600">57600</option>
     </select>
   </div>

   <div class="timing-item">
     <label for="xfSioBaud">SIO ESP32 ↔ XF551 (baud)</label>
     <select id="xfSioBaud">
       <option value="19200">19200</option>
       <option value="38400">38400</option>
       <option value="57600">57600</option>
     </select>
   </div>

   <div class="timing-item">
     <label for="netDelayUs">Delay ESP-NOW (µs)</label>
     <input type="number" id="netDelayUs" min="0" step="50" />
   </div>
 </div>

 <!-- ===== NUEVO: VERIFY WRITE (SLAVE) ===== -->
 <div style="margin-top:12px; font-size:0.8rem;">
   <span style="display:block;margin-bottom:4px;">Verificación WRITE (SLAVE)</span>
 </div>

 <div style="display:flex; flex-wrap:wrap; gap:10px; align-items:center;">
   <div style="display:flex; gap:6px; align-items:center;">
     <label class="switch">
       <input type="checkbox" id="verAll">
       <span class="switch-slider"></span>
     </label>
     <span>VERIFY_ALL_WRITES</span>
   </div>

   <div style="display:flex; gap:6px; align-items:center;">
     <label class="switch">
       <input type="checkbox" id="verBoot">
       <span class="switch-slider"></span>
     </label>
     <span>VERIFY_BOOT_SECTORS</span>
   </div>

   <div style="display:flex; gap:6px; align-items:center;">
     <label class="switch">
       <input type="checkbox" id="verVtoc">
       <span class="switch-slider"></span>
     </label>
     <span>VERIFY_VTOC_DIR</span>
   </div>

   <div style="display:flex; gap:6px; align-items:center;">
     <label class="switch">
       <input type="checkbox" id="ver57">
       <span class="switch-slider"></span>
     </label>
     <span>VERIFY_WRITE_WITH_VERIFY (0x57)</span>
   </div>
 </div>

 <div class="actions">
   <button id="btnSave">💾 Guardar configuración</button>
   <button id="btnReload" class="secondary">🔄 Recargar estado</button>
 </div>

 <div class="status-line">
 <div id="statusDot" class="status-dot"></div>
 <div id="statusMsg" class="status-msg">Listo.</div>
 </div>
 </section>
 </div>

 <div class="footer">
 ESP32 Master Bridge · XF551 WiFi · V1.3
 </div>
 </div>
 </div>

 <script>
 function $(id) { return document.getElementById(id); }

 function setStatus(msg, ok = true) {
   const dot = $("statusDot");
   const text = $("statusMsg");
   text.textContent = msg;
   if (ok) dot.classList.remove("err");
   else dot.classList.add("err");
 }

 function renderDrives(drives) {
   const container = $("drives");
   container.innerHTML = "";
   if (!drives || drives.length === 0) {
     container.innerHTML = "<small>No hay disqueteras registradas aún.</small>";
     return;
   }
   drives.forEach(d => {
     const div = document.createElement("div");
     div.className = "drive-item";

     const present         = !!d.present;
     const supports256     = !!d.supports256;
     const prefetch        = !!d.prefetch;
     const prefetchSectors = d.prefetchSectors || 0;
     const mac             = d.mac || "—";
     const physical        = d.physical || "—";
     const avgAckMs        = (typeof d.avgAckMs === "number" && d.avgAckMs > 0)
                               ? d.avgAckMs.toFixed(1)
                               : null;
     const autoEnabled     = (typeof d.autoEnabled === "boolean") ? d.autoEnabled : true;

     div.innerHTML = `
       <div class="drive-header">
         <div class="drive-name">${d.dev || "DX"}</div>
         <div style="display:flex; gap:4px; align-items:center;">
           <span class="pill ${present ? "ok" : "bad"}">${present ? "ONLINE" : "OFFLINE"}</span>
           <span class="pill">${supports256 ? "DD 256B" : "SD 128B"}</span>
         </div>
       </div>
       <div class="drive-body">
         <div><span class="label">MAC: </span><span class="value">${mac}</span></div>
         <div><span class="label">Prefetch: </span><span class="value">${prefetch ? "Sí" : "No"}</span></div>
         <div><span class="label">Sectores pref.: </span><span class="value">${prefetchSectors}</span></div>
         <div><span class="label">Unidad física XF: </span><span class="value">${physical}</span></div>
         <div><span class="label">Tiempo medio ACK: </span><span class="value">${
           avgAckMs ? (avgAckMs + " ms") : "Sin datos aún"
         }</span></div>
         <div><span class="label">Auto-ajuste: </span><span class="value">${autoEnabled ? "ON" : "OFF"}</span></div>
       </div>

       <div class="drive-prefetch">
         <div style="display:flex; align-items:center; gap:4px;">
           <label class="switch">
             <input type="checkbox" data-dev="${d.dev}" class="pf-toggle" ${prefetch ? "checked" : ""}>
             <span class="switch-slider"></span>
           </label>
           <span style="font-size:0.75rem;">Prefetch</span>
         </div>
         <div style="display:flex; align-items:center; gap:4px;">
           <span style="font-size:0.75rem;">Sectores:</span>
           <input
             type="number"
             min="0"
             max="16"
             step="1"
             value="${prefetchSectors}"
             class="small-input pf-count"
             data-dev="${d.dev}"
           />
         </div>
       </div>

       <div class="drive-prefetch" style="margin-top:2px;">
         <div style="display:flex; align-items:center; gap:4px;">
           <label class="switch">
             <input type="checkbox" data-dev="${d.dev}" class="auto-toggle" ${autoEnabled ? "checked" : ""}>
             <span class="switch-slider"></span>
           </label>
           <span style="font-size:0.75rem;">Auto-ajuste ON/OFF</span>
         </div>
       </div>

       ${(mac && mac !== "—" && mac !== "00:00:00:00:00:00") ? `
       <div class="drive-prefetch" style="margin-top:2px;">
         <div style="display:flex; align-items:center; gap:6px;">
           <span style="font-size:0.75rem;">Asignar este equipo a:</span>
           <select class="small-input logic-sel" data-mac="${mac}">
             <option value="D1">D1</option>
             <option value="D2">D2</option>
             <option value="D3">D3</option>
             <option value="D4">D4</option>
           </select>
           <button class="secondary btnLogic" data-mac="${mac}" style="padding:8px 10px;">Asignar</button>
         </div>
       </div>
       ` : ``}
     `;
     container.appendChild(div);

     // Prefijar selector y bind de mapeo lógico para este equipo
     const btn = div.querySelector(".btnLogic");
     const sel = div.querySelector(".logic-sel");
     if (sel && d.dev) sel.value = d.dev;
     if (btn && sel) {
       btn.addEventListener("click", async () => {
         try {
           const to = sel.value;
           setStatus("Guardando mapeo lógico...", true);
           const r = await fetch(`/set_logic?mac=${encodeURIComponent(btn.dataset.mac)}&to=${encodeURIComponent(to)}`);
           if (!r.ok) {
             const t = await r.text();
             throw new Error(t || `HTTP ${r.status}`);
           }
           await loadStatus();
           setStatus("Mapeo lógico actualizado.", true);
         } catch (e) {
           setStatus("Error al mapear: " + e.message, false);
         }
       });
     }
   });
 }

 function renderDevices(devices) {
   const container = $("devices");
   if (!container) return;
   container.innerHTML = "";

   if (!devices || devices.length === 0) {
     container.innerHTML = "<small>No hay equipos detectados aún.</small>";
     return;
   }

   devices.forEach(dev => {
     const div = document.createElement("div");
     div.className = "drive-item device-item";

     const mac = dev.mac || "—";
     const physical = dev.physical || "—";
     const logical = dev.logical || "—";
     const present = !!dev.present;
     const supports256 = !!dev.supports256;

     div.innerHTML = `
       <div class="drive-header">
         <div class="drive-name" style="font-size:0.9rem;">${mac}</div>
         <div style="display:flex; gap:4px; align-items:center;">
           <span class="pill ${present ? "ok" : "bad"}">${present ? "ONLINE" : "OFFLINE"}</span>
           <span class="pill">${supports256 ? "DD 256B" : "SD 128B"}</span>
         </div>
       </div>
       <div class="drive-body">
         <div><span class="label">Unidad física XF: </span><span class="value">${physical}</span></div>
         <div><span class="label">Unidad lógica asignada: </span><span class="value">${logical}</span></div>
       </div>
       <div class="drive-prefetch">
         <span style="font-size:0.75rem;">Asignar como:</span>
         <select class="small-input" data-mac="${mac}">
           <option value="D1">D1</option>
           <option value="D2">D2</option>
           <option value="D3">D3</option>
           <option value="D4">D4</option>
         </select>
         <button class="secondary" style="padding:8px 10px;">Asignar</button>
       </div>
     `;

     const sel = div.querySelector("select");
     const btn = div.querySelector("button");
     if (sel && logical && logical !== "—") sel.value = logical;
     if (btn && sel) {
       btn.addEventListener("click", async () => {
         try {
           const to = sel.value;
           setStatus("Guardando mapeo lógico...", true);
           const r = await fetch(`/set_logic?mac=${encodeURIComponent(mac)}&to=${encodeURIComponent(to)}`);
           if (!r.ok) {
             const t = await r.text();
             throw new Error(t || `HTTP ${r.status}`);
           }
           await loadStatus();
           setStatus("Mapeo lógico actualizado.", true);
         } catch (e) {
           setStatus("Error al mapear: " + e.message, false);
         }
       });
     }

     container.appendChild(div);
   });
 }


 async function loadStatus() {
   try {
     setStatus("Cargando estado...", true);
     const res = await fetch("/api/status");
     if (!res.ok) throw new Error("HTTP " + res.status);
     const data = await res.json();

     renderDrives(data.drives || []);

     if (data.timings) {
       $("ackToComplete").value  = data.timings.ackToComplete ?? "";
       $("completeToData").value = data.timings.completeToData ?? "";
       $("dataToChk").value      = data.timings.dataToChk ?? "";
       $("chunkDelay").value     = data.timings.chunkDelay ?? "";

       if (typeof data.timings.autoProfile !== "undefined") {
         $("autoProfile").value = String(data.timings.autoProfile);
       }
     }

     if (data.comm) {
       $("uartBaud").value = String(data.comm.uartBaud ?? "460800");
       $("rpSioBaud").value  = String(data.comm.rpSioBaud ?? "19200");
       $("xfSioBaud").value  = String(data.comm.xfSioBaud ?? "19200");
       $("netDelayUs").value = data.comm.netDelayUs ?? "0";
     }

     if (data.verify) {
       $("verAll").checked  = !!data.verify.all;
       $("verBoot").checked = !!data.verify.boot;
       $("verVtoc").checked = !!data.verify.vtoc;
       $("ver57").checked   = !!data.verify.w57;
     }

     setStatus("Estado actualizado.");
   } catch (e) {
     console.error(e);
     setStatus("Error al obtener estado: " + e.message, false);
   }
 }

 function collectConfig() {
   const drives = [];
   document.querySelectorAll("#drives .drive-item").forEach(div => {
     const devName = div.querySelector(".drive-name").textContent.trim(); // "D1"
     const togglePf = div.querySelector(".pf-toggle");
     const inputCount = div.querySelector(".pf-count");
     const autoToggle = div.querySelector(".auto-toggle");

     let sectors = inputCount ? Number(inputCount.value) || 0 : 0;
     const pfEnabled = togglePf ? togglePf.checked : false;
     if (!pfEnabled) sectors = 0;

     const autoEnabled = autoToggle ? autoToggle.checked : true;

     drives.push({
       dev: devName,
       prefetch: sectors > 0,
       prefetchSectors: sectors,
       autoEnabled: autoEnabled
     });
   });

   const timings = {
     ackToComplete:  Number($("ackToComplete").value)  || 0,
     completeToData: Number($("completeToData").value) || 0,
     dataToChk:      Number($("dataToChk").value)      || 0,
     chunkDelay:     Number($("chunkDelay").value)     || 0
   };

   const autoProfile = Number($("autoProfile").value) || 1;

   const comm = {
     uart:   Number($("uartBaud").value) || 0,
     sio_rp: Number($("rpSioBaud").value) || 0,
     sio_xf: Number($("xfSioBaud").value) || 0,
     net_us: Number($("netDelayUs").value) || 0
   };

   const verify = {
     v_all:  $("verAll").checked ? 1 : 0,
     v_boot: $("verBoot").checked ? 1 : 0,
     v_vtoc: $("verVtoc").checked ? 1 : 0,
     v_57:   $("ver57").checked ? 1 : 0
   };

   return { drives, timings, autoProfile, comm, verify };
 }

 function mapPrefetchByDev(drives) {
   const out = { D1: 0, D2: 0, D3: 0, D4: 0 };
   drives.forEach(d => {
     const dev = d.dev;
     if (out.hasOwnProperty(dev)) {
       out[dev] = d.prefetchSectors || 0;
     }
   });
   return out;
 }

 function mapAutoByDev(drives) {
   const out = { D1: 1, D2: 1, D3: 1, D4: 1 };
   drives.forEach(d => {
     const dev = d.dev;
     if (out.hasOwnProperty(dev)) {
       out[dev] = d.autoEnabled ? 1 : 0;
     }
   });
   return out;
 }

 async function saveConfig() {
   try {
     const cfg = collectConfig();
     const pf  = mapPrefetchByDev(cfg.drives);
     const auto = mapAutoByDev(cfg.drives);

     setStatus("Guardando configuración...", true);

     const urlTiming =
       `/set_timing?t_ack=${cfg.timings.ackToComplete}` +
       `&t_comp=${cfg.timings.completeToData}` +
       `&t_chk=${cfg.timings.dataToChk}` +
       `&t_chunk=${cfg.timings.chunkDelay}`;

     const urlPrefetch =
       `/set_prefetch?pf1=${pf.D1}&pf2=${pf.D2}&pf3=${pf.D3}&pf4=${pf.D4}`;

     const urlAuto =
       `/set_auto?d1=${auto.D1}&d2=${auto.D2}&d3=${auto.D3}&d4=${auto.D4}` +
       `&profile=${cfg.autoProfile}`;

     const urlComm =
       `/set_comm?uart=${cfg.comm.uart}` +
       `&sio_rp=${cfg.comm.sio_rp}` +
       `&sio_xf=${cfg.comm.sio_xf}` +
       `&net_us=${cfg.comm.net_us}`;

     const urlVerify =
       `/set_verify?v_all=${cfg.verify.v_all}` +
       `&v_boot=${cfg.verify.v_boot}` +
       `&v_vtoc=${cfg.verify.v_vtoc}` +
       `&v_57=${cfg.verify.v_57}`;

     const r1 = await fetch(urlTiming);
     if (!r1.ok) throw new Error("Error en /set_timing (" + r1.status + ")");

     const r2 = await fetch(urlPrefetch);
     if (!r2.ok) throw new Error("Error en /set_prefetch (" + r2.status + ")");

     const r3 = await fetch(urlAuto);
     if (!r3.ok) throw new Error("Error en /set_auto (" + r3.status + ")");

     const r4 = await fetch(urlComm);
     if (!r4.ok) throw new Error("Error en /set_comm (" + r4.status + ")");

     const r5 = await fetch(urlVerify);
     if (!r5.ok) throw new Error("Error en /set_verify (" + r5.status + ")");

     setStatus("Configuración guardada. Recargando estado...");
     await loadStatus();
   } catch (e) {
     console.error(e);
     setStatus("Error al guardar configuración: " + e.message, false);
   }
 }

 document.addEventListener("DOMContentLoaded", () => {
   $("btnReload").addEventListener("click", loadStatus);
   $("btnSave").addEventListener("click", saveConfig);
   loadStatus();
 });
 </script>
</body>
</html>
)rawliteral";

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

const char* devName(uint8_t dev) {
  static const char* names[] = { "D1", "D2", "D3", "D4" };
  int idx = devIndex(dev);
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

// ========= Pendiente de WRITE desde el RP =========
struct PendingWriteFromRP {
  bool active;
  uint8_t dev;
  uint16_t sec;
};
PendingWriteFromRP g_pendingWriteRP = { false, 0, 0 };

// ========= Último comando importante =========
struct LastMasterOp {
  bool active;
  uint8_t cmd;
  uint8_t dev;
  uint16_t sec;
  uint32_t sentMs;
};
LastMasterOp g_lastMasterOp = { false, 0, 0, 0, 0 };

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

void autoTuneTimingsFromAck() {
  if (g_lockFastTimings) return;
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

  uint32_t ackUs = bestAckMs * 1000UL;

  float scale = 1.0f;
  if (g_autoProfile == 0) scale = 1.5f;
  else if (g_autoProfile == 2) scale = 0.7f;

  ackUs = (uint32_t)(ackUs * scale);

  uint16_t newAckToComplete   = clampU16(ackUs / 4, 300, 2000);
  uint16_t newCompleteToData  = clampU16(ackUs / 6, 250, 1500);
  uint16_t newChunkDelay      = clampU16(ackUs / 3, 100, 4000);

  bool changed = false;

  if (abs((int)newAckToComplete - (int)T_ACK_TO_COMPLETE) > 50) {
    T_ACK_TO_COMPLETE = newAckToComplete; changed = true;
  }
  if (abs((int)newCompleteToData - (int)T_COMPLETE_TO_DATA) > 50) {
    T_COMPLETE_TO_DATA = newCompleteToData; changed = true;
  }
  if (T_DATA_TO_CHK == 0) { T_DATA_TO_CHK = 80; changed = true; }
  if (abs((int)newChunkDelay - (int)T_CHUNK_DELAY) > 100) {
    T_CHUNK_DELAY = newChunkDelay; changed = true;
  }

  if (changed) {
    logf("[AUTO] ACK=%lu ms perfil=%d -> ackToComp=%u compToData=%u chunkDelay=%u",
         (unsigned long)bestAckMs, g_autoProfile,
         (unsigned)T_ACK_TO_COMPLETE, (unsigned)T_COMPLETE_TO_DATA, (unsigned)T_CHUNK_DELAY);
    saveTimingConfigToNvs();
    // sendTimingUpdateToRP() está más abajo
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
    prefs.putUChar("auto1", 1);
    prefs.putUChar("auto2", 1);
    prefs.putUChar("auto3", 1);
    prefs.putUChar("auto4", 1);

    // defaults comm/verify
    prefs.putUInt("uartBd", CFG_UART_BAUD);
    prefs.putUInt("sioRP",  CFG_RP_SIO_BAUD);
    prefs.putUInt("sioXF",  CFG_XF_SIO_BAUD);
    prefs.putUInt("netDly", CFG_NET_DELAY_US);
    prefs.putUChar("vflg",  CFG_VERIFY_FLAGS);

    prefs.end();
    logf("[NVS] Config inicial grabada");
    return;
  }

  T_ACK_TO_COMPLETE   = prefs.getUShort("t_ack",  T_ACK_TO_COMPLETE);
  T_COMPLETE_TO_DATA  = prefs.getUShort("t_comp", T_COMPLETE_TO_DATA);
  T_DATA_TO_CHK       = prefs.getUShort("t_chk",  T_DATA_TO_CHK);
  T_CHUNK_DELAY       = prefs.getUShort("t_chd",  T_CHUNK_DELAY);
  if (g_lockFastTimings) {
    applyFastTimingPreset();
  }

  prefetchCfg[0] = prefs.getUChar("pf1", prefetchCfg[0]);
  prefetchCfg[1] = prefs.getUChar("pf2", prefetchCfg[1]);
  prefetchCfg[2] = prefs.getUChar("pf3", prefetchCfg[2]);
  prefetchCfg[3] = prefs.getUChar("pf4", prefetchCfg[3]);

  g_autoProfile = prefs.getUChar("autoProf", (uint8_t)g_autoProfile);

  g_driveTiming[0].autoEnabled = prefs.getUChar("auto1", 1) != 0;
  g_driveTiming[1].autoEnabled = prefs.getUChar("auto2", 1) != 0;
  g_driveTiming[2].autoEnabled = prefs.getUChar("auto3", 1) != 0;
  g_driveTiming[3].autoEnabled = prefs.getUChar("auto4", 1) != 0;
  if (g_lockFastTimings) {
    g_autoProfile = 0;
    for (int i = 0; i < 4; ++i) g_driveTiming[i].autoEnabled = false;
  }

  // comm/verify
  CFG_UART_BAUD    = prefs.getUInt("uartBd", CFG_UART_BAUD);
  CFG_RP_SIO_BAUD  = prefs.getUInt("sioRP",  CFG_RP_SIO_BAUD);
  CFG_XF_SIO_BAUD  = prefs.getUInt("sioXF",  CFG_XF_SIO_BAUD);
  CFG_NET_DELAY_US = prefs.getUInt("netDly", CFG_NET_DELAY_US);
  CFG_VERIFY_FLAGS = prefs.getUChar("vflg",  CFG_VERIFY_FLAGS);

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
  Serial2.flush();
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

void broadcastSlaveCfg() {
  uint8_t p[11];
  p[0] = TYPE_CFG_UPDATE;
  p[1] = 0x00; // todos
  putLE32(&p[2], CFG_XF_SIO_BAUD);
  putLE32(&p[6], CFG_NET_DELAY_US);
  p[10] = CFG_VERIFY_FLAGS;

  sendEspNow(BCAST_MAC, p, sizeof(p));
  logf("[MASTER] CFG_UPDATE -> SLAVES xfSio=%lu netDelayUs=%lu vflg=0x%02X",
       (unsigned long)CFG_XF_SIO_BAUD, (unsigned long)CFG_NET_DELAY_US,
       (unsigned)CFG_VERIFY_FLAGS);
}

bool pushCfgToRPAndMaybeSwitchUart(); // fwd

// ===== UART RX <- RP2040 FSM =====
static uint8_t uartState = 0;
static uint8_t uartLen = 0;
static uint8_t uartIdx = 0;
static uint32_t uartLastTime = 0;
static uint8_t uartBuf[260];

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

              if (g_lockFastTimings) {
                applyFastTimingPreset();
                logf("[MASTER] TIMING_UPDATE <- RP ignorado; usando preset rápido ack2comp=%u comp2data=%u data2chk=%u chunk=%u",
                    (unsigned)T_ACK_TO_COMPLETE, (unsigned)T_COMPLETE_TO_DATA,
                    (unsigned)T_DATA_TO_CHK, (unsigned)T_CHUNK_DELAY);
              } else {
                T_ACK_TO_COMPLETE  = clampU16(newAck2Comp,  200, 8000);
                T_COMPLETE_TO_DATA = clampU16(newComp2Data, 100, 8000);
                T_DATA_TO_CHK      = clampU16(newData2Chk,   10, 2000);
                T_CHUNK_DELAY      = clampU16(newChunk,     100, 20000);

                logf("[MASTER] TIMING_UPDATE <- RP ack2comp=%u comp2data=%u data2chk=%u chunk=%u",
                    (unsigned)T_ACK_TO_COMPLETE, (unsigned)T_COMPLETE_TO_DATA,
                    (unsigned)T_DATA_TO_CHK, (unsigned)T_CHUNK_DELAY);

                saveTimingConfigToNvs();
              }
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

                if (isWriteSector || isWritePercom) {
                  uint16_t opSec = isWritePercom ? 0xFFFF : sec;
                  g_pendingWriteRP.active = true;
                  g_pendingWriteRP.dev = dev;
                  g_pendingWriteRP.sec = opSec;
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

                logf("[MASTER] CMD 0x%02X dev=%s sec=%u pf=%u",
                     (unsigned)cmd, devName(dev), (unsigned)sec, (unsigned)uartBuf[6]);
              }

              uint8_t dev = uartBuf[2];
              sendEspToSlave(dev, uartBuf, uartLen);
            } break;

            case TYPE_SECTOR_CHUNK: {
              if (uartLen < 6) break;

              uint8_t dev   = uartBuf[1];
              uint16_t sec  = (uint16_t)uartBuf[2] | ((uint16_t)uartBuf[3] << 8);
              uint8_t idx   = uartBuf[4];
              uint8_t count = uartBuf[5];

              bool isPercom = (sec == 0xFFFF);
              if (isPercom) {
                sendEspToSlave(dev, uartBuf, uartLen);
                g_pendingWriteRP.active = false;
                break;
              }

              if (!g_pendingWriteRP.active) break;
              if (dev != g_pendingWriteRP.dev || sec != g_pendingWriteRP.sec) break;

              sendEspToSlave(dev, uartBuf, uartLen);

              if ((uint8_t)(idx + 1) >= count) {
                g_pendingWriteRP.active = false;
              }
            } break;

            default:
              break;
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

    g_lastMasterOp.active = false;
    autoTuneTimingsFromAck();
    sendTimingUpdateToRP();
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
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleApiStatus() {
  String json = "{";
  json += "\"drives\":[";
  bool first = true;

  for (int i = 0; i < 4; i++) {
    uint8_t devCode = DEV_MIN + i;
    const char* name = devName(devCode);

    if (!first) json += ",";
    first = false;

    bool present = slaves[i].present;
    bool supports256 = slaves[i].supports256;
    uint8_t pf = prefetchCfg[i];
    String macStr = formatMac(slaves[i].mac);
    unsigned long lastSeen = slaves[i].lastSeen;

    uint8_t physDev = slaves[i].physicalDev;
    String physName = "-";
    if (physDev >= DEV_MIN && physDev <= DEV_MAX) physName = String(devName(physDev));

    uint32_t avgAck = g_driveTiming[i].avgAckMs;
    bool autoEn = g_driveTiming[i].autoEnabled;

    json += "{";
    json += "\"dev\":\"" + String(name) + "\",";
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

// ===== SETUP / LOOP =====
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("\n=== ESP32 MASTER BRIDGE (RP2040 <-> SLAVE XF551) + WEB + AUTO + CFG ==="));

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
  g_lastMasterOp.active = false;

  loadConfigFromNvs();
  if (g_lockFastTimings) applyFastTimingPreset();
  loadMapFromNvs();

  // UART con RP siempre arranca BOOT
  Serial2.begin(BOOT_UART_BAUD, SERIAL_8N1, PIN_RP_RX, PIN_RP_TX);
  g_currentUartBaud = BOOT_UART_BAUD;

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  WiFi.softAP("XF551_MASTER", "xf551wifi", WIFI_CHANNEL);

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

  server.on("/", handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/set_timing", HTTP_GET, handleSetTiming);
  server.on("/set_prefetch", HTTP_GET, handleSetPrefetch);
  server.on("/set_auto", HTTP_GET, handleSetAuto);
  server.on("/set_comm", HTTP_GET, handleSetComm);
  server.on("/set_verify", HTTP_GET, handleSetVerify);
  server.on("/set_logic", HTTP_GET, handleSetLogic);
  server.begin();
  Serial.println("[WEB] Servidor HTTP iniciado en puerto 80");
}

void loop() {
  server.handleClient();

  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
    if (slaves[i].present && (now - slaves[i].lastSeen > 60000)) {
      slaves[i].present = false;
      int di = findDeviceSlotByMac(slaves[i].mac);
      if (di >= 0) g_devices[di].present = false;
    }
  }

  pollUartFromRP();
  delay(0);
}
