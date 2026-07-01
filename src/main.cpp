/*
 ******************************************************************************
 * Humidity Controller Remote Display  v1.0
 ******************************************************************************
 *
 *  TARGET BOARD:
 *    Freenove ESP32-S3 Display FNK0104A
 *    ILI9341 2.8" IPS  240×320 px
 *
 *  DISPLAY SPI PINS (from official Freenove FNK0104AB User_Setup):
 *    MOSI (SDA) → GPIO 11
 *    MISO (SDO) → GPIO 13
 *    CLK  (SCL) → GPIO 12
 *    CS         → GPIO 10
 *    DC         → GPIO 46
 *    RST        → -1      (tied to chip RESET via CHIP_PU)
 *    BL         → GPIO 45 (TFT_eSPI controls, HIGH = on)
 *
 *  FUNCTION:
 *    Polls the Humidity Controller's /status JSON endpoint over WiFi and
 *    renders live humidity, temperature, and machine state on the TFT display.
 *    The controller IP/hostname is configured through the web UI or AP portal
 *    and persisted in NVS.
 *
 *  WiFi SETUP:
 *    On first boot (no saved credentials) the device starts an AP named
 *    "ESP32-Display".  Connect to it, enter your WiFi credentials and the
 *    Humidity Controller's IP, and the device restarts in station mode.
 *
 *  OTA:
 *    ElegantOTA  → http://<device>.local/update  (user: admin / pass: pw)
 *    ArduinoOTA  → PlatformIO env:freenove-s3-ota  (port 3232)
 *
 ******************************************************************************/

#define FIRMWARE_VERSION  "v1.0"

// ============================================================
//  SPI pins and backlight (GPIO 45) are set via build_flags in platformio.ini.
//  TFT_eSPI turns the backlight on automatically in tft.init().

// ============================================================
//  AP PORTAL
// ============================================================
#define AP_SSID  "ESP32-Display"
#define AP_PASS  ""

// ============================================================
//  OTA CREDENTIALS
// ============================================================
#define OTA_USER  "admin"
#define OTA_PASS  "pw"

// ============================================================
//  DEFAULTS
// ============================================================
#define DEFAULT_DEVICE_NAME   "HumidityDisplay"
#define DEFAULT_CTRL_HOST     "192.168.1.169"   // Humidity Controller IP
#define DEFAULT_PLANT_HOST    ""                // Plant Monitor IP (empty = disabled)

// ============================================================
//  TIMING
// ============================================================
#define POLL_INTERVAL_DRYING_MS    10000UL   // 10 s — fast while drying
#define POLL_INTERVAL_STORAGE_MS   60000UL   // 60 s — slow in storage mode
#define PLANT_POLL_INTERVAL_MS     30000UL   // 30 s — plant data
#define WIFI_RETRY_INTERVAL        30000UL
#define STALE_DATA_MS             120000UL   // 2 min — flag stale data on display
#define DISPLAY_REFRESH_MS         10000UL   // refresh "updated X ago" text

// ============================================================
//  INCLUDES
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>

// ============================================================
//  OBJECTS
// ============================================================
TFT_eSPI    tft;
WebServer   server(80);
Preferences prefs;

// ============================================================
//  THEME COLORS  (RGB565 — initialised in setup via tft.color565)
// ============================================================
uint16_t COL_BG, COL_CARD, COL_BORDER,
         COL_ACCENT, COL_GREEN, COL_RED, COL_WARN,
         COL_DIM, COL_TEXT, COL_CYAN;

// ============================================================
//  STATUS SNAPSHOTS
// ============================================================
struct CtrlStatus {
  bool    valid        = false;
  float   humidity     = NAN;
  float   temperature  = NAN;
  float   rhThreshold  = 20.0f;
  String  appState     = "";
  bool    heaterOn     = false;
  bool    fault        = false;
  String  faultReason  = "";
  String  lastRead     = "--";
  String  device       = "";
};

struct PlantStatus {
  bool    valid        = false;
  float   moisture     = NAN;   // soil moisture (soilMoisture or moisture key)
  float   humidity     = NAN;   // ambient humidity (humidity key)
  float   temperature  = NAN;
  String  appState     = "";
  bool    pumpOn       = false;
  bool    fault        = false;
  bool    sleepEnabled = true;
  long    nextReadIn   = 0;     // seconds until next wake (from Plant Monitor JSON)
  String  faultReason  = "";
  String  lastRead     = "--";
  String  device       = "";
};

CtrlStatus  ctrlStatus;
PlantStatus plantStatus;

// ============================================================
//  RUNTIME STATE
// ============================================================
String deviceName;
String ctrlHost;
String plantHost;

bool          mdnsRunning        = false;
bool          inAPMode           = false;
bool          otaRunning         = false;
unsigned long lastSuccessfulPoll = 0;
unsigned long nextPollAt         = 0;
unsigned long lastSuccessfulPlantPoll = 0;
unsigned long nextPlantPollAt    = 0;
unsigned long lastWiFiRetry      = 0;
unsigned long bootMillis         = 0;
unsigned long lastDisplayRefresh = 0;
bool          needsFullRedraw    = true;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
bool   connectWiFi();
void   startAPPortal();
void   startMDNS();
void   setupRoutes();
bool   pollController();
bool   pollPlant();
void   initColors();
void   drawBootScreen();
void   drawAPScreen();
void   drawMainScreen();
void   drawHumidityPanel();
void   drawPlantPanel();
String sanitizeMDNS(const String& s);
String wifiQuality(int rssi);
String buildStatusJson();
void   loadPrefs();
void   saveSettings();
void   saveWiFiCreds(const String& ssid, const String& pass);
void   loadWiFiCreds(String& ssid, String& pass);
void   clearWiFiCreds();
void   setupArduinoOTA();
void   setupPowerManagement();

// ============================================================
//  NVS HELPERS
// ============================================================
void loadPrefs() {
  prefs.begin("system", true);
  deviceName = prefs.getString("name",       DEFAULT_DEVICE_NAME);
  ctrlHost   = prefs.getString("ctrl_host",  DEFAULT_CTRL_HOST);
  plantHost  = prefs.getString("plant_host", DEFAULT_PLANT_HOST);
  prefs.end();
}

void saveSettings() {
  prefs.begin("system", false);
  prefs.putString("name",       deviceName);
  prefs.putString("ctrl_host",  ctrlHost);
  prefs.putString("plant_host", plantHost);
  prefs.end();
}

void saveWiFiCreds(const String& ssid, const String& pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void loadWiFiCreds(String& ssid, String& pass) {
  prefs.begin("wifi", true);
  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");
  prefs.end();
}

void clearWiFiCreds() {
  prefs.begin("wifi", false);
  prefs.remove("ssid");
  prefs.remove("pass");
  prefs.end();
}

// ============================================================
//  mDNS
// ============================================================
String sanitizeMDNS(const String& input) {
  String out = "";
  for (int i = 0; i < (int)input.length(); i++) {
    char c = tolower((unsigned char)input[i]);
    if (isalnum((unsigned char)c))  out += c;
    else if (c == '-' || c == ' ')  out += '-';
  }
  while (out.startsWith("-")) out = out.substring(1);
  while (out.endsWith("-"))   out = out.substring(0, out.length() - 1);
  if (out.isEmpty())          out = DEFAULT_DEVICE_NAME;
  return out;
}

void startMDNS() {
  if (mdnsRunning) MDNS.end();
  String host = sanitizeMDNS(deviceName);
  if (MDNS.begin(host.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsRunning = true;
    Serial.printf("[mDNS] http://%s.local\n", host.c_str());
  } else {
    mdnsRunning = false;
    Serial.println("[mDNS] Failed to start");
  }
}

// ============================================================
//  POWER MANAGEMENT
// ============================================================
void setupPowerManagement() {
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.println("[Power] Always-on mode");
}

// ============================================================
//  WIFI SIGNAL STRING
// ============================================================
String wifiQuality(int rssi) {
  if (!WiFi.isConnected()) return "Disconnected";
  if (rssi > -50) return "Excellent";
  if (rssi > -60) return "Good";
  if (rssi > -70) return "Fair";
  return "Weak";
}

// ============================================================
//  THEME COLORS
// ============================================================
void initColors() {
  COL_BG     = tft.color565(0x0D, 0x0F, 0x1A);
  COL_CARD   = tft.color565(0x15, 0x17, 0x24);
  COL_BORDER = tft.color565(0x25, 0x28, 0x40);
  COL_ACCENT = tft.color565(0x4F, 0x8E, 0xF7);
  COL_GREEN  = tft.color565(0x3E, 0xCF, 0x8E);
  COL_RED    = tft.color565(0xF7, 0x6E, 0x6E);
  COL_WARN   = tft.color565(0xF7, 0xB8, 0x4F);
  COL_DIM    = tft.color565(0x88, 0x92, 0xA4);
  COL_TEXT   = tft.color565(0xDD, 0xE3, 0xF0);
  COL_CYAN   = tft.color565(0x4F, 0xCF, 0xEF);
}

// ============================================================
//  DISPLAY — BOOT SCREEN
// ============================================================
void drawBootScreen() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.setTextFont(4);
  tft.drawString("Humidity Display", 120, 120);
  tft.setTextFont(2);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString(FIRMWARE_VERSION, 120, 160);
  tft.drawString("Connecting...", 120, 185);
}

// ============================================================
//  DISPLAY — AP MODE SCREEN
// ============================================================
void drawAPScreen() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_WARN, COL_BG);
  tft.setTextFont(4);
  tft.drawString("WiFi Setup", 120, 10);
  tft.setTextFont(2);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Connect your phone to:", 120, 65);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextFont(4);
  tft.drawString(AP_SSID, 120, 90);
  tft.setTextFont(2);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Then open:", 120, 140);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.drawString("http://192.168.4.1", 120, 162);
  tft.drawFastHLine(10, 200, 220, COL_BORDER);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Set WiFi + device IPs", 120, 212);
  tft.drawString("then Save to connect.", 120, 232);
}

// ============================================================
//  DISPLAY — BANNER HELPER
//  Draws a colored state banner in the given rect.
// ============================================================
static void drawStateBanner(int y, int h, const String& state, bool valid, bool fault) {
  const int W = tft.width();
  uint16_t bg, fg;
  String   text;
  if (!valid) {
    bg = COL_CARD; fg = COL_DIM; text = "CONNECTING...";
  } else if (fault) {
    bg = tft.color565(0x1A,0x00,0x00); fg = COL_RED;  text = "FAULT";
  // ── Humidity Controller states ──
  } else if (state == "Drying") {
    bg = tft.color565(0x1A,0x0F,0x00); fg = COL_WARN; text = "DRYING";
  } else if (state == "Storage") {
    bg = tft.color565(0x0A,0x1A,0x0D); fg = COL_GREEN; text = "STORAGE";
  } else if (state == "Stopped") {
    bg = tft.color565(0x1A,0x00,0x00); fg = COL_RED;  text = "STOPPED";
  // ── Plant Monitor states ──
  } else if (state == "Sleeping") {
    bg = tft.color565(0x05,0x10,0x1A); fg = COL_ACCENT; text = "RESTING";
  } else if (state == "Reading") {
    bg = tft.color565(0x0A,0x1A,0x0D); fg = COL_GREEN; text = "READING";
  } else if (state == "Pump running") {
    bg = tft.color565(0x00,0x0F,0x1A); fg = COL_CYAN; text = "WATERING";
  } else if (state == "Cooldown") {
    bg = tft.color565(0x0A,0x12,0x1A); fg = COL_DIM;  text = "COOLDOWN";
  // ── Generic fallback ──
  } else if (state == "Idle") {
    bg = tft.color565(0x0A,0x1A,0x0D); fg = COL_GREEN; text = "IDLE";
  } else {
    bg = COL_CARD; fg = COL_DIM; text = state.isEmpty() ? "--" : state;
  }
  tft.fillRect(0, y, W, h, bg);
  tft.drawRect(0, y, W, h, COL_BORDER);
  tft.setTextColor(fg, bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(2);
  tft.drawString(text.c_str(), W / 2, y + h / 2);
}

// ── shared title bar helper (Font 4 bold-effect + Font 1 version) ────────────
static void drawPanelTitle(int y, const String& title, uint16_t col) {
  const int W = tft.width();
  tft.fillRect(0, y, W, 26, COL_BG);
  // Device name — Font 4, draw twice for bold effect
  tft.setTextFont(4);
  tft.setTextColor(col, COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(title.c_str(), W / 2,     y + 2);
  tft.drawString(title.c_str(), W / 2 + 1, y + 2);
  // Firmware version — Font 1, right edge, not bold
  tft.setTextFont(1);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(FIRMWARE_VERSION, W - 4, y + 18);
  tft.setTextDatum(TL_DATUM);
}

// ============================================================
//  DISPLAY — TOP PANEL  (Humidity Controller, y 0–153)
//
//  y=  0  Title / device name         Font 4 (bold), 26px
//  y= 26  State banner                Font 2, h=18
//  y= 44  Humidity row  "HUM  XX.X %" Font 4 (26px)
//  y= 70  Temp row      "TEMP XX.X F" Font 4 (26px)
//  y= 96  Heater + threshold          Font 2 (16px)
//  y=112  Last read time              Font 1 (8px)
// ============================================================
void drawHumidityPanel() {
  const int W  = tft.width();

  // Title
  String title = ctrlStatus.device.isEmpty() ? "HUMIDITY CTRL"
                                              : ctrlStatus.device.c_str();
  drawPanelTitle(0, title, COL_ACCENT);

  // State banner
  drawStateBanner(26, 18, ctrlStatus.appState, ctrlStatus.valid, ctrlStatus.fault);

  // Humidity row (Font 4, same height as temp)
  tft.fillRect(0, 44, W, 26, COL_BG);
  {
    uint16_t rhCol;
    String   rhStr;
    if (!ctrlStatus.valid || isnan(ctrlStatus.humidity)) {
      rhCol = COL_DIM; rhStr = "--";
    } else {
      rhStr  = String(ctrlStatus.humidity, 1);
      float rh = ctrlStatus.humidity, thr = ctrlStatus.rhThreshold;
      rhCol = (rh > thr) ? COL_RED : (rh > thr * 0.85f) ? COL_WARN : COL_GREEN;
    }
    tft.setTextFont(1);
    tft.setTextColor(COL_DIM, COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("HUM", 8, 45);
    tft.setTextFont(4);
    tft.setTextColor(rhCol, COL_BG);
    tft.setTextDatum(TR_DATUM);
    tft.drawString((rhStr + " %").c_str(), W - 8, 44);
  }

  // Temp row
  tft.fillRect(0, 70, W, 26, COL_BG);
  uint16_t tCol;
  String   tStr;
  if (!ctrlStatus.valid || isnan(ctrlStatus.temperature)) {
    tCol = COL_DIM; tStr = "--";
  } else {
    tStr = String(ctrlStatus.temperature, 1);
    float t = ctrlStatus.temperature;
    tCol = (t >= 155.0f) ? COL_RED
         : (t >= 153.0f) ? COL_WARN
         : ctrlStatus.heaterOn ? COL_CYAN : COL_TEXT;
  }
  tft.setTextFont(1);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("TEMP", 8, 71);
  tft.setTextFont(4);
  tft.setTextColor(tCol, COL_BG);
  tft.setTextDatum(TR_DATUM);
  tft.drawString((tStr + " F").c_str(), W - 8, 70);

  // Heater + threshold row
  tft.fillRect(0, 96, W, 16, COL_BG);
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Heater:", 8, 96);
  if (!ctrlStatus.valid) {
    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawString("--", 68, 96);
  } else {
    tft.setTextColor(ctrlStatus.heaterOn ? COL_CYAN : COL_DIM, COL_BG);
    tft.drawString(ctrlStatus.heaterOn ? "ON" : "OFF", 68, 96);
  }
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Thr:", 130, 96);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString(ctrlStatus.valid
    ? (String(ctrlStatus.rhThreshold, 0) + "%") : "--", 162, 96);

  // Last read
  tft.fillRect(0, 112, W, 11, COL_BG);
  tft.setTextFont(1);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Last:", 8, 113);
  if (ctrlStatus.valid && ctrlStatus.lastRead != "--") {
    String t = ctrlStatus.lastRead;
    int sp = t.indexOf(' ');
    if (sp >= 0 && sp < (int)t.length() - 1) t = t.substring(sp + 1);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString(t.c_str(), 42, 113);
  } else {
    tft.drawString("--", 42, 113);
  }
}

// ============================================================
//  DISPLAY — BOTTOM PANEL  (Plant Monitor, y 158–318)
//  BASE+0   Title / device name        Font 4 (bold), 26px
//  BASE+26  State banner               Font 2, h=18
//  BASE+44  Soil moisture row  "SOIL %" Font 4 (26px)
//  BASE+70  Temp row                   Font 4 (26px)
//  BASE+96  XIAO Humidity row  "HUM %" Font 4 (26px)
//  BASE+122 Pump + Next countdown      Font 2 + Font 1
//  BASE+138 Last read time             Font 1
// ============================================================
void drawPlantPanel() {
  const int W    = tft.width();
  const int BASE = 158;

  if (plantHost.isEmpty()) {
    tft.fillRect(0, BASE, W, 320 - BASE, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextFont(2);
    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawString("PLANT MONITOR", W / 2, BASE + 6);
    tft.setTextFont(1);
    tft.drawString("Set IP in web config", W / 2, BASE + 32);
    tft.drawString(("http://" + sanitizeMDNS(deviceName) + ".local").c_str(), W / 2, BASE + 44);
    return;
  }

  // Title
  String title = plantStatus.device.isEmpty() ? "PLANT MONITOR"
                                              : plantStatus.device.c_str();
  drawPanelTitle(BASE, title, COL_GREEN);

  // State banner
  drawStateBanner(BASE + 26, 18,
                  plantStatus.appState, plantStatus.valid, plantStatus.fault);

  // Soil moisture row (Font 4)
  tft.fillRect(0, BASE + 44, W, 26, COL_BG);
  {
    uint16_t mCol;
    String   mStr;
    if (!plantStatus.valid || isnan(plantStatus.moisture)) {
      mCol = COL_DIM; mStr = "--";
    } else {
      mStr = String(plantStatus.moisture, 1);
      float m = plantStatus.moisture;
      mCol = (m < 20.0f) ? COL_RED : (m < 35.0f) ? COL_WARN : COL_GREEN;
    }
    tft.setTextFont(1);
    tft.setTextColor(COL_DIM, COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("SOIL", 8, BASE + 45);
    tft.setTextFont(4);
    tft.setTextColor(mCol, COL_BG);
    tft.setTextDatum(TR_DATUM);
    tft.drawString((mStr + " %").c_str(), W - 8, BASE + 44);
  }

  // Temp row
  tft.fillRect(0, BASE + 70, W, 26, COL_BG);
  uint16_t ptCol;
  String   ptStr;
  if (!plantStatus.valid || isnan(plantStatus.temperature)) {
    ptCol = COL_DIM; ptStr = "--";
  } else {
    ptStr = String(plantStatus.temperature, 1);
    ptCol = COL_TEXT;
  }
  tft.setTextFont(1);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("TEMP", 8, BASE + 71);
  tft.setTextFont(4);
  tft.setTextColor(ptCol, COL_BG);
  tft.setTextDatum(TR_DATUM);
  tft.drawString((ptStr + " F").c_str(), W - 8, BASE + 70);

  // Plant humidity row (Font 4)
  tft.fillRect(0, BASE + 96, W, 26, COL_BG);
  {
    uint16_t rhCol;
    String   rhStr;
    if (!plantStatus.valid || isnan(plantStatus.humidity)) {
      rhCol = COL_DIM; rhStr = "--";
    } else {
      rhStr = String(plantStatus.humidity, 1);
      float rh = plantStatus.humidity, thr = ctrlStatus.rhThreshold;
      rhCol = (rh > thr) ? COL_RED : (rh > thr * 0.85f) ? COL_WARN : COL_GREEN;
    }
    tft.setTextFont(1);
    tft.setTextColor(COL_DIM, COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("HUM", 8, BASE + 97);
    tft.setTextFont(4);
    tft.setTextColor(rhCol, COL_BG);
    tft.setTextDatum(TR_DATUM);
    tft.drawString((rhStr + " %").c_str(), W - 8, BASE + 96);
  }

  // Pump row
  tft.fillRect(0, BASE + 122, W, 16, COL_BG);
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Pump:", 8, BASE + 122);
  if (!plantStatus.valid) {
    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawString("--", 68, BASE + 122);
  } else {
    tft.setTextColor(plantStatus.pumpOn ? COL_CYAN : COL_DIM, COL_BG);
    tft.drawString(plantStatus.pumpOn ? "ON" : "OFF", 68, BASE + 122);
  }
  tft.setTextFont(1);
  tft.setTextColor(COL_DIM, COL_BG);
  if (plantStatus.valid && plantStatus.appState == "Sleeping" && plantStatus.nextReadIn > 0) {
    long rem = plantStatus.nextReadIn;
    String cd = rem >= 60 ? String(rem / 60) + "m " + String(rem % 60) + "s"
                          : String(rem) + "s";
    tft.drawString("Next:", 130, BASE + 123);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString(cd.c_str(), 162, BASE + 123);
  }

  // Last read
  tft.fillRect(0, BASE + 138, W, 11, COL_BG);
  tft.setTextFont(1);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Last:", 8, BASE + 139);
  if (plantStatus.valid && plantStatus.lastRead != "--") {
    String t = plantStatus.lastRead;
    int sp = t.indexOf(' ');
    if (sp >= 0 && sp < (int)t.length() - 1) t = t.substring(sp + 1);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString(t.c_str(), 42, BASE + 139);
  } else {
    tft.drawString("--", 42, BASE + 139);
  }
}

// ============================================================
//  DISPLAY — MAIN SCREEN  (full repaint, split layout)
//
//  Portrait 240×320:
//    y=  0–153  Humidity Controller panel
//    y=154–157  Divider
//    y=158–306  Plant Monitor panel
//    y=307–319  IP + mDNS footer
// ============================================================
void drawMainScreen() {
  const int W = tft.width();
  tft.fillScreen(COL_BG);
  drawHumidityPanel();
  tft.fillRect(0, 154, W, 4, COL_BORDER);
  drawPlantPanel();
  // IP + mDNS footer
  tft.setTextFont(1);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(WiFi.localIP().toString().c_str(), 4, 309);
  tft.setTextDatum(TR_DATUM);
  tft.drawString((sanitizeMDNS(deviceName) + ".local").c_str(), W - 4, 309);
  needsFullRedraw = false;
}

// ============================================================
//  HTTP POLL — fetch /status from Humidity Controller
// ============================================================
bool pollController() {
  if (!WiFi.isConnected() || ctrlHost.isEmpty()) return false;

  String url = "http://" + ctrlHost + "/status";
  Serial.printf("[Poll] GET %s\n", url.c_str());

  HTTPClient http;
  http.begin(url);
  http.setTimeout(5000);

  int code = http.GET();

  if (code != 200) {
    Serial.printf("[Poll] HTTP %d\n", code);
    http.end();
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, http.getStream());
  http.end();

  if (err) {
    Serial.printf("[Poll] JSON error: %s\n", err.c_str());
    return false;
  }

  ctrlStatus.valid       = true;
  ctrlStatus.humidity    = doc["humidity"].isNull()
                             ? NAN : doc["humidity"].as<float>();
  ctrlStatus.temperature = doc["temperature"].isNull()
                             ? NAN : doc["temperature"].as<float>();
  ctrlStatus.rhThreshold = doc["rhThreshold"] | 20.0f;
  ctrlStatus.appState    = doc["appState"].as<String>();
  ctrlStatus.heaterOn    = doc["heaterOn"]    | false;
  ctrlStatus.fault       = doc["fault"]       | false;
  ctrlStatus.faultReason = doc["faultReason"].as<String>();
  ctrlStatus.lastRead    = doc["lastRead"].as<String>();
  ctrlStatus.device      = doc["device"].as<String>();

  lastSuccessfulPoll = millis();

  unsigned long interval = (ctrlStatus.appState == "Drying")
                             ? POLL_INTERVAL_DRYING_MS
                             : POLL_INTERVAL_STORAGE_MS;
  nextPollAt = millis() + interval;

  Serial.printf("[Poll] OK  RH:%.1f%%  T:%.1fF  State:%s\n",
                ctrlStatus.humidity,
                ctrlStatus.temperature,
                ctrlStatus.appState.c_str());
  return true;
}

// ============================================================
//  HTTP POLL — fetch /status from Plant Monitor
// ============================================================
bool pollPlant() {
  if (!WiFi.isConnected() || plantHost.isEmpty()) return false;

  String url = "http://" + plantHost + "/status";
  Serial.printf("[Plant] GET %s\n", url.c_str());

  HTTPClient http;
  http.begin(url);
  http.setTimeout(5000);
  int code = http.GET();

  if (code != 200) {
    Serial.printf("[Plant] HTTP %d\n", code);
    http.end();
    nextPlantPollAt = millis() + 15000UL;
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, http.getStream());
  http.end();

  if (err) {
    Serial.printf("[Plant] JSON error: %s\n", err.c_str());
    nextPlantPollAt = millis() + 15000UL;
    return false;
  }

  plantStatus.valid   = true;
  plantStatus.device  = doc["device"].as<String>();
  plantStatus.appState= doc["appState"].as<String>();
  // Soil moisture: prefer "soilMoisture", then "moisture"
  if (!doc["soilMoisture"].isNull())
    plantStatus.moisture = doc["soilMoisture"].as<float>();
  else if (!doc["moisture"].isNull())
    plantStatus.moisture = doc["moisture"].as<float>();
  else
    plantStatus.moisture = NAN;
  // Ambient humidity: Plant Monitor uses "ambientHum", fall back to "humidity"
  if (!doc["ambientHum"].isNull())
    plantStatus.humidity = doc["ambientHum"].as<float>();
  else if (!doc["humidity"].isNull())
    plantStatus.humidity = doc["humidity"].as<float>();
  else
    plantStatus.humidity = NAN;
  plantStatus.temperature = doc["ambientTemp"].isNull() ? NAN : doc["ambientTemp"].as<float>();
  plantStatus.pumpOn      = doc["pumpOn"] | doc["pump"] | false;
  plantStatus.fault       = doc["fault"] | false;
  plantStatus.sleepEnabled= doc["sleepEnabled"] | true;
  plantStatus.nextReadIn  = doc["nextReadIn"] | 0L;
  plantStatus.faultReason = doc["faultReason"].as<String>();
  plantStatus.lastRead    = doc["lastRead"].as<String>();
  if (plantStatus.lastRead.isEmpty()) plantStatus.lastRead = "--";

  lastSuccessfulPlantPoll = millis();
  nextPlantPollAt         = millis() + PLANT_POLL_INTERVAL_MS;

  Serial.printf("[Plant] OK  Moisture:%.1f%%  T:%.1fF  State:%s\n",
                plantStatus.moisture,
                plantStatus.temperature,
                plantStatus.appState.c_str());
  return true;
}

// ============================================================
//  STATUS JSON  (this device's status for the web UI)
// ============================================================
String buildStatusJson() {
  unsigned long nowMs     = millis();
  unsigned long uptimeSec = (nowMs - bootMillis) / 1000UL;

  JsonDocument doc;
  doc["firmware"]  = FIRMWARE_VERSION;
  doc["device"]    = deviceName;
  doc["mdns"]      = sanitizeMDNS(deviceName);
  doc["uptime"]    = uptimeSec;
  doc["ctrlHost"]  = ctrlHost;
  doc["plantHost"] = plantHost;

  doc["ctrlValid"]       = ctrlStatus.valid;
  if (!isnan(ctrlStatus.humidity))    doc["ctrlHumidity"]    = ctrlStatus.humidity;
  if (!isnan(ctrlStatus.temperature)) doc["ctrlTemperature"] = ctrlStatus.temperature;
  doc["ctrlState"]       = ctrlStatus.appState;
  doc["ctrlHeaterOn"]    = ctrlStatus.heaterOn;
  doc["ctrlFault"]       = ctrlStatus.fault;
  doc["ctrlLastRead"]    = ctrlStatus.lastRead;
  doc["lastPollAgoSec"]  = lastSuccessfulPoll == 0
                             ? -1 : (long)((nowMs - lastSuccessfulPoll) / 1000UL);

  doc["plantValid"]      = plantStatus.valid;
  if (!isnan(plantStatus.moisture))    doc["plantMoisture"]    = plantStatus.moisture;
  if (!isnan(plantStatus.temperature)) doc["plantTemperature"] = plantStatus.temperature;
  doc["plantState"]      = plantStatus.appState;
  doc["plantPumpOn"]     = plantStatus.pumpOn;
  doc["plantFault"]      = plantStatus.fault;
  doc["plantLastRead"]   = plantStatus.lastRead;
  doc["lastPlantPollAgoSec"] = lastSuccessfulPlantPoll == 0
                             ? -1 : (long)((nowMs - lastSuccessfulPlantPoll) / 1000UL);

  doc["ssid"]    = WiFi.isConnected() ? WiFi.SSID()              : "Not connected";
  doc["ip"]      = WiFi.isConnected() ? WiFi.localIP().toString() : "--";
  doc["mac"]     = WiFi.macAddress();
  doc["rssi"]    = WiFi.isConnected() ? WiFi.RSSI() : 0;
  doc["quality"] = wifiQuality(WiFi.RSSI());

  String out;
  serializeJson(doc, out);
  return out;
}

// ============================================================
//  WEB ROUTE HANDLERS
// ============================================================
void handleStatus() { server.send(200, "application/json", buildStatusJson()); }
void handleHealth() { server.send(200, "text/plain", "OK"); }

void handleRename() {
  if (!server.hasArg("name") || server.arg("name").isEmpty()) {
    server.send(400, "application/json", "{\"error\":\"Missing name\"}");
    return;
  }
  String raw = server.arg("name"); raw.trim();
  deviceName = sanitizeMDNS(raw);
  saveSettings();
  startMDNS();
  server.send(200, "application/json",
              "{\"ok\":true,\"name\":\"" + deviceName + "\"}");
}

void handleSetController() {
  if (!server.hasArg("host") || server.arg("host").isEmpty()) {
    server.send(400, "application/json", "{\"error\":\"Missing host\"}");
    return;
  }
  ctrlHost = server.arg("host"); ctrlHost.trim();
  saveSettings();
  nextPollAt = 0;
  server.send(200, "application/json",
              "{\"ok\":true,\"host\":\"" + ctrlHost + "\"}");
}

void handleSetPlant() {
  // Accepts empty host to disable plant monitor
  plantHost = server.hasArg("host") ? server.arg("host") : "";
  plantHost.trim();
  saveSettings();
  nextPlantPollAt = 0;
  needsFullRedraw = true;
  server.send(200, "application/json",
              "{\"ok\":true,\"host\":\"" + plantHost + "\"}");
}

void handleWiFiReset() {
  server.send(200, "application/json",
              "{\"ok\":true,\"msg\":\"Rebooting into setup mode.\"}");
  clearWiFiCreds();
  delay(1000);
  ESP.restart();
}

void handleWiFiChange() {
  if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
    server.send(400, "application/json", "{\"error\":\"Missing SSID\"}");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.hasArg("pass") ? server.arg("pass") : "";
  saveWiFiCreds(ssid, pass);
  server.send(200, "application/json", "{\"ok\":true}");
  delay(800);
  ESP.restart();
}

void handleWiFiScanStart() {
  WiFi.scanNetworks(true);
  server.send(200, "application/json", "{\"ok\":true,\"status\":\"scanning\"}");
}

void handleWiFiScanResult() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"status\":\"scanning\"}");
    return;
  }
  JsonDocument doc;
  if (n < 0) {
    doc["status"] = "error";
  } else {
    doc["status"] = "done";
    JsonArray arr = doc["networks"].to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject net = arr.add<JsonObject>();
      net["ssid"]   = WiFi.SSID(i);
      net["rssi"]   = WiFi.RSSI(i);
      net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
  }
  WiFi.scanDelete();
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ============================================================
//  WEB DASHBOARD  (config page for the display device)
// ============================================================
void handleRoot() {
  server.send(200, "text/html", R"DASH(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Humidity Display Config</title>
<style>
:root{
  --bg:#0d0f1a;--card:#151724;--bdr:#252840;--acc:#4f8ef7;
  --grn:#3ecf8e;--red:#f76e6e;--warn:#f7b84f;--dim:#8892a4;--txt:#dde3f0;--cya:#4fcfef;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',Arial,sans-serif;
     padding:1rem;max-width:560px;margin:auto}
h1{text-align:center;color:var(--acc);font-size:1.3rem;margin:.8rem 0 .3rem}
.card{background:var(--card);border:1px solid var(--bdr);border-radius:14px;
      padding:1.2rem 1.4rem;margin-bottom:1rem}
.card h2{font-size:.78rem;text-transform:uppercase;letter-spacing:.1em;
         color:var(--dim);margin-bottom:.9rem}
.row{display:flex;justify-content:space-between;align-items:center;
     padding:.35rem 0;border-bottom:1px solid var(--bdr);font-size:.9rem}
.row:last-child{border-bottom:none}
.lbl{color:var(--dim);font-size:.88rem}.val{font-weight:600;font-size:.92rem}
.badge{display:inline-block;padding:.2rem .7rem;border-radius:999px;
       font-size:.78rem;font-weight:600;background:#1e2535}
.badge.ok{color:var(--grn)}.badge.warn{color:var(--warn)}.badge.err{color:var(--red)}
.btn-row{display:flex;gap:.6rem;flex-wrap:wrap;margin-top:.85rem}
button{padding:.5rem .95rem;border:none;border-radius:8px;font-size:.86rem;
       font-weight:600;cursor:pointer;transition:opacity .15s}
button:hover{opacity:.83}button:active{opacity:.65}
.b-blue{background:var(--acc);color:#fff}.b-grn{background:var(--grn);color:#111}
.b-red{background:var(--red);color:#fff}.b-dim{background:var(--bdr);color:var(--txt)}
.b-sm{padding:.3rem .7rem;font-size:.78rem}
.irow{display:flex;gap:.5rem;margin-top:.7rem}
input[type=text],input[type=password]{flex:1;padding:.5rem .8rem;
  background:#1e2535;border:1px solid var(--bdr);border-radius:8px;
  color:var(--txt);font-size:.9rem;outline:none}
input:focus{border-color:var(--acc)}
.ota-link{display:block;text-align:center;color:var(--acc);text-decoration:none;
          font-size:.92rem;font-weight:600;padding:.4rem 0}
.net-list{margin-top:.7rem;max-height:180px;overflow-y:auto}
.net-item{display:flex;justify-content:space-between;align-items:center;
          padding:.4rem .6rem;border-radius:.4rem;cursor:pointer;
          margin-bottom:.3rem;background:#1e2535}
.net-item:hover{background:var(--bdr)}
.lock{font-size:.72rem;margin-left:.3rem;color:var(--warn)}
#scanStatus{font-size:.82rem;color:var(--dim);margin:.4rem 0;text-align:center}
#upd{text-align:center;font-size:.73rem;color:var(--dim);margin-top:.4rem}
</style>
</head>
<body>
<h1 id="hTitle">Humidity Display</h1>

<!-- CONTROLLER STATUS -->
<div class="card">
  <h2>Controller Status</h2>
  <div class="row"><span class="lbl">State</span>
    <span id="ctrlState" class="badge">--</span></div>
  <div class="row"><span class="lbl">Humidity</span>
    <span class="val" id="ctrlHum">--</span></div>
  <div class="row"><span class="lbl">Temperature</span>
    <span class="val" id="ctrlTemp">--</span></div>
  <div class="row"><span class="lbl">Heater</span>
    <span class="val" id="ctrlHeater">--</span></div>
  <div class="row"><span class="lbl">Last read</span>
    <span class="val" id="ctrlLast">--</span></div>
  <div class="row"><span class="lbl">Data age</span>
    <span class="val" id="pollAge">--</span></div>
</div>

<!-- CONTROLLER HOST -->
<div class="card">
  <h2>Controller Connection</h2>
  <div class="row"><span class="lbl">Controller IP / hostname</span>
    <span class="val" id="ctrlHostVal">--</span></div>
  <div class="irow">
    <input type="text" id="ctrlHostInput" placeholder="e.g. 192.168.1.169 or esp32.local">
    <button class="b-blue" onclick="saveCtrl()">Save</button>
  </div>
</div>

<!-- PLANT MONITOR STATUS -->
<div class="card">
  <h2>Plant Monitor Status</h2>
  <div class="row"><span class="lbl">State</span>
    <span id="plantState" class="badge">--</span></div>
  <div class="row"><span class="lbl">Soil Moisture</span>
    <span class="val" id="plantMoist">--</span></div>
  <div class="row"><span class="lbl">Temperature</span>
    <span class="val" id="plantTemp">--</span></div>
  <div class="row"><span class="lbl">Pump</span>
    <span class="val" id="plantPump">--</span></div>
  <div class="row"><span class="lbl">Last read</span>
    <span class="val" id="plantLast">--</span></div>
  <div class="row"><span class="lbl">Data age</span>
    <span class="val" id="plantAge">--</span></div>
</div>

<!-- PLANT MONITOR HOST -->
<div class="card">
  <h2>Plant Monitor Connection</h2>
  <div class="row"><span class="lbl">Plant Monitor IP / hostname</span>
    <span class="val" id="plantHostVal">--</span></div>
  <div class="irow">
    <input type="text" id="plantHostInput" placeholder="e.g. 192.168.1.170 (leave blank to disable)">
    <button class="b-blue" onclick="savePlant()">Save</button>
  </div>
</div>

<!-- NETWORK -->
<div class="card">
  <h2>Network</h2>
  <div class="row"><span class="lbl">SSID</span><span class="val" id="ssid">--</span></div>
  <div class="row"><span class="lbl">IP address</span><span class="val" id="ip">--</span></div>
  <div class="row"><span class="lbl">MAC address</span><span class="val" id="mac">--</span></div>
  <div class="row"><span class="lbl">Signal</span><span class="val" id="quality">--</span></div>
  <div class="row"><span class="lbl">mDNS hostname</span><span class="val" id="mdnsLabel">--</span></div>

  <h2 style="margin-top:1.1rem">Change WiFi Network</h2>
  <button class="b-dim" style="width:100%;margin-top:.5rem" onclick="doScan()">Scan for networks</button>
  <div id="scanStatus"></div>
  <div class="net-list" id="netList"></div>
  <div class="irow">
    <input type="text" id="wifiSSID" placeholder="Selected or type SSID" autocomplete="off">
    <input type="password" id="wifiPass" placeholder="Password" autocomplete="new-password">
  </div>
  <div class="btn-row" style="margin-top:.7rem">
    <button class="b-blue" onclick="changeWiFi()">Connect to network</button>
    <button class="b-red"  onclick="resetWiFi()">Reset WiFi credentials</button>
  </div>

  <h2 style="margin-top:1.1rem">Change Device Name / mDNS</h2>
  <div class="irow">
    <input type="text" id="renameInput" placeholder="new-hostname">
    <button class="b-blue" onclick="renameDevice()">Save</button>
  </div>
</div>

<!-- FIRMWARE -->
<div class="card">
  <h2>Firmware  <span id="fwVer" style="font-weight:600;color:var(--txt)"></span></h2>
  <a class="ota-link" href="/update">Open OTA update page &#x2197;</a>
</div>

<div id="upd">Connecting...</div>

<script>
var $=function(id){return document.getElementById(id);};

function toast(msg,ms){
  ms=ms||2500;
  var d=document.createElement('div');d.textContent=msg;
  d.style.cssText='position:fixed;bottom:1.4rem;left:50%;transform:translateX(-50%);'
    +'background:#252840;color:#dde3f0;padding:.55rem 1.1rem;border-radius:8px;'
    +'font-size:.86rem;z-index:9999;box-shadow:0 2px 12px rgba(0,0,0,.4);opacity:1;transition:opacity .4s';
  document.body.appendChild(d);
  setTimeout(function(){d.style.opacity='0';setTimeout(function(){d.parentNode&&d.parentNode.removeChild(d);},500);},ms);
}

function update(){
  fetch('/status').then(function(r){return r.json();}).then(function(d){
    $('hTitle').textContent = d.device || 'Humidity Display';
    document.title          = d.device || 'Humidity Display';
    $('fwVer').textContent  = d.firmware || '';

    // Controller status
    var cs=$('ctrlState');
    cs.className='badge';
    if(!d.ctrlValid){cs.textContent='Offline';cs.classList.add('err');}
    else if(d.ctrlFault){cs.textContent='FAULT';cs.classList.add('err');}
    else if(d.ctrlState==='Drying'){cs.textContent='Drying';cs.classList.add('warn');}
    else if(d.ctrlState==='Storage'){cs.textContent='Storage';cs.classList.add('ok');}
    else{cs.textContent=d.ctrlState||'--';}

    $('ctrlHum').textContent  = d.ctrlHumidity!=null ? d.ctrlHumidity.toFixed(1)+'%' : '--';
    $('ctrlTemp').textContent = d.ctrlTemperature!=null ? d.ctrlTemperature.toFixed(1)+'°F' : '--';
    $('ctrlHeater').textContent = d.ctrlValid ? (d.ctrlHeaterOn ? 'ON' : 'OFF') : '--';
    $('ctrlLast').textContent = d.ctrlLastRead || '--';
    $('pollAge').textContent  = d.lastPollAgoSec<0 ? 'Never polled'
                                  : d.lastPollAgoSec+'s ago';
    $('ctrlHostVal').textContent = d.ctrlHost || '--';

    // Plant Monitor status
    var ps=$('plantState');
    ps.className='badge';
    if(!d.plantHost){ps.textContent='Disabled';ps.classList.add('err');}
    else if(!d.plantValid){ps.textContent='Offline';ps.classList.add('err');}
    else if(d.plantFault){ps.textContent='FAULT';ps.classList.add('err');}
    else if(d.plantState==='Watering'){ps.textContent='Watering';ps.classList.add('ok');}
    else if(d.plantState==='Idle'){ps.textContent='Idle';ps.classList.add('ok');}
    else{ps.textContent=d.plantState||'--';}

    $('plantMoist').textContent = d.plantMoisture!=null ? d.plantMoisture.toFixed(1)+'%' : '--';
    $('plantTemp').textContent  = d.plantTemperature!=null ? d.plantTemperature.toFixed(1)+'°F' : '--';
    $('plantPump').textContent  = d.plantValid ? (d.plantPumpOn ? 'ON' : 'OFF') : '--';
    $('plantLast').textContent  = d.plantLastRead || '--';
    $('plantAge').textContent   = d.lastPlantPollAgoSec<0 ? 'Never polled'
                                    : d.lastPlantPollAgoSec+'s ago';
    $('plantHostVal').textContent = d.plantHost || '(not set)';

    // Network
    $('ssid').textContent    = d.ssid;
    $('ip').textContent      = d.ip;
    $('mac').textContent     = d.mac;
    $('quality').textContent = d.quality + ' (' + d.rssi + ' dBm)';
    $('mdnsLabel').textContent = (d.mdns||'') + '.local';

    $('upd').textContent = 'Updated: ' + new Date().toLocaleTimeString();
  }).catch(function(){$('upd').textContent='Connection lost — retrying...';});
}

function saveCtrl(){
  var h=$('ctrlHostInput').value.trim();
  if(!h){toast('Enter a host or IP first');return;}
  fetch('/controller/set?host='+encodeURIComponent(h))
    .then(function(r){return r.json();})
    .then(function(d){if(d.ok){toast('Saved: '+d.host);$('ctrlHostInput').value='';update();}
                     else toast('Error: '+(d.error||'unknown'));});
}
function savePlant(){
  var h=$('plantHostInput').value.trim();
  fetch('/plant/set?host='+encodeURIComponent(h))
    .then(function(r){return r.json();})
    .then(function(d){if(d.ok){toast(h?'Plant host saved: '+d.host:'Plant monitor disabled');
                              $('plantHostInput').value='';update();}
                     else toast('Error: '+(d.error||'unknown'));});
}
function renameDevice(){
  var n=$('renameInput').value.trim();
  if(!n){toast('Enter a name first');return;}
  fetch('/device/rename?name='+encodeURIComponent(n))
    .then(function(r){return r.json();})
    .then(function(d){if(d.ok){toast('Renamed to: '+d.name);$('renameInput').value='';update();}
                     else toast('Error: '+(d.error||'unknown'));});
}
function changeWiFi(){
  var ssid=$('wifiSSID').value.trim(),pass=$('wifiPass').value;
  if(!ssid){toast('Enter or select a network first');return;}
  var fd=new FormData();fd.append('ssid',ssid);fd.append('pass',pass);
  fetch('/wifi/change',{method:'POST',body:fd}).then(function(){toast('Saved. Rebooting...',4000);});
}
function resetWiFi(){
  fetch('/wifi/reset').then(function(){toast('Rebooting into setup mode...',4000);});
}

var scanTimer=null;
function signalBars(r){return r>-50?'||||':r>-60?'||| ':r>-70?'||  ':'|   ';}
function doScan(){
  $('scanStatus').textContent='Starting scan...';$('netList').innerHTML='';
  fetch('/wifi/scan/start').then(function(){
    $('scanStatus').textContent='Scanning... (5-10 s)';
    clearInterval(scanTimer);scanTimer=setInterval(pollScan,2000);
  }).catch(function(){$('scanStatus').textContent='Scan request failed';});
}
function pollScan(){
  fetch('/wifi/scan/result').then(function(r){return r.json();}).then(function(d){
    if(d.status==='scanning') return;
    clearInterval(scanTimer);
    if(d.status==='error'||!d.networks){$('scanStatus').textContent='Scan failed';return;}
    $('scanStatus').textContent=d.networks.length+' network'+(d.networks.length!==1?'s':'')+' found';
    d.networks.forEach(function(n){
      var div=document.createElement('div');div.className='net-item';
      div.innerHTML='<span>'+n.ssid+(n.secure?'<span class="lock">&#x1F512;</span>':'')+'</span>'
        +'<span style="font-size:.75rem;color:var(--dim)">'+signalBars(n.rssi)+' '+n.rssi+' dBm</span>';
      div.onclick=function(){$('wifiSSID').value=n.ssid;$('wifiPass').focus();};
      $('netList').appendChild(div);
    });
  }).catch(function(){clearInterval(scanTimer);$('scanStatus').textContent='Poll failed';});
}

setInterval(update, 15000);
update();
</script>
</body>
</html>
)DASH");
}

// ============================================================
//  ROUTE REGISTRATION
// ============================================================
void setupRoutes() {
  server.on("/",                 HTTP_GET,  handleRoot);
  server.on("/status",           HTTP_GET,  handleStatus);
  server.on("/health",           HTTP_GET,  handleHealth);
  server.on("/controller/set",   HTTP_GET,  handleSetController);
  server.on("/plant/set",        HTTP_GET,  handleSetPlant);
  server.on("/device/rename",    HTTP_GET,  handleRename);
  server.on("/wifi/scan/start",  HTTP_GET,  handleWiFiScanStart);
  server.on("/wifi/scan/result", HTTP_GET,  handleWiFiScanResult);
  server.on("/wifi/change",      HTTP_POST, handleWiFiChange);
  server.on("/wifi/reset",       HTTP_GET,  handleWiFiReset);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
}

// ============================================================
//  AP SETUP PORTAL
// ============================================================
void startAPPortal() {
  inAPMode = true;
  Serial.printf("[WiFi] Starting AP: %s\n", AP_SSID);
  WiFi.mode(WIFI_AP);
  if (strlen(AP_PASS) >= 8) WiFi.softAP(AP_SSID, AP_PASS);
  else                       WiFi.softAP(AP_SSID);
  Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  drawAPScreen();

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", R"HTML(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Display Setup</title>
<style>
body{background:#111;color:#eee;font-family:Arial,sans-serif;display:flex;
  justify-content:center;align-items:center;min-height:100vh;margin:0;padding:1rem;box-sizing:border-box}
.box{background:#1e1e2e;padding:2rem;border-radius:1rem;width:100%;max-width:340px;
  text-align:center;box-shadow:0 4px 24px rgba(0,0,0,.5)}
h2{color:#4f8ef7;margin-bottom:1.5rem}
input{width:100%;padding:.6rem;margin:.4rem 0 .8rem;border-radius:.5rem;
  border:1px solid #444;background:#2a2a3e;color:#fff;font-size:1rem;box-sizing:border-box}
label{display:block;text-align:left;color:#8892a4;font-size:.82rem;margin-bottom:.1rem}
button{width:100%;padding:.7rem;background:#4f8ef7;color:#fff;border:none;
  border-radius:.5rem;font-size:1rem;cursor:pointer;margin-bottom:.6rem}
button:hover{background:#3a7de0}
.scan-btn{background:#252840}.scan-btn:hover{background:#333658}
.note{font-size:.8rem;color:#888;margin-top:.8rem}
.net-item{display:flex;justify-content:space-between;align-items:center;
  padding:.4rem .6rem;border-radius:.4rem;cursor:pointer;margin-bottom:.3rem;
  background:#252840;text-align:left}
.net-item:hover{background:#333658}
.lock{font-size:.75rem;margin-left:.4rem;color:#f7b84f}
#netList{margin-bottom:.8rem;max-height:180px;overflow-y:auto}
#scanning{color:#8892a4;font-size:.85rem;margin:.5rem 0}
</style></head><body>
<div class="box">
  <h2>Display WiFi Setup</h2>
  <button class="scan-btn" onclick="doScan()">Scan for Networks</button>
  <div id="scanning" style="display:none">Scanning...</div>
  <div id="netList"></div>
  <label>WiFi Network</label>
  <input id="ssidInput" placeholder="Select above or type SSID" autocomplete="off">
  <label>Password</label>
  <input id="passInput" placeholder="Password" type="password" autocomplete="new-password">
  <label>Humidity Controller IP</label>
  <input id="ctrlInput" placeholder="e.g. 192.168.1.169" value="192.168.1.169">
  <label>Plant Monitor IP <span style="color:#8892a4;font-size:.75rem">(optional)</span></label>
  <input id="plantInput" placeholder="e.g. 192.168.1.170">
  <button onclick="doConnect()">Save &amp; Connect</button>
  <p class="note">Device will restart and connect to your network.</p>
</div>
<script>
function doScan(){
  var list=document.getElementById('netList'),sc=document.getElementById('scanning');
  list.innerHTML='';sc.style.display='block';
  fetch('/wifi/scan').then(function(r){return r.json();}).then(function(nets){
    sc.style.display='none';
    if(!nets.length){list.innerHTML='<p style="color:#8892a4;font-size:.85rem">No networks found</p>';return;}
    nets.forEach(function(n){
      var d=document.createElement('div');d.className='net-item';
      d.innerHTML='<span>'+n.ssid+(n.secure?'<span class="lock">&#x1F512;</span>':'')+'</span>'
        +'<span style="font-size:.75rem;color:#8892a4">'+n.rssi+' dBm</span>';
      d.onclick=function(){document.getElementById('ssidInput').value=n.ssid;};
      list.appendChild(d);
    });
  }).catch(function(){sc.textContent='Scan failed — try again';});
}
function doConnect(){
  var s=document.getElementById('ssidInput').value.trim();
  var p=document.getElementById('passInput').value;
  var c=document.getElementById('ctrlInput').value.trim();
  var pl=document.getElementById('plantInput').value.trim();
  if(!s){alert('Enter or select an SSID first.');return;}
  var fd=new FormData();fd.append('ssid',s);fd.append('password',p);
  fd.append('ctrl',c);fd.append('plant',pl);
  fetch('/save',{method:'POST',body:fd}).then(function(r){return r.text();})
    .then(function(t){document.body.innerHTML=t;});
}
doScan();
</script></body></html>
)HTML");
  });

  server.on("/save", HTTP_POST, []() {
    if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
      server.send(400, "text/plain", "SSID is required");
      return;
    }
    String ssid  = server.arg("ssid"); ssid.trim();
    String pass  = server.hasArg("password") ? server.arg("password") : "";
    String ctrl  = server.hasArg("ctrl")  ? server.arg("ctrl")  : DEFAULT_CTRL_HOST;
    String plant = server.hasArg("plant") ? server.arg("plant") : "";
    ctrl.trim(); plant.trim();
    saveWiFiCreds(ssid, pass);
    ctrlHost  = ctrl;
    plantHost = plant;
    saveSettings();
    String mdnsName = sanitizeMDNS(deviceName);
    server.send(200, "text/html",
      String("<!DOCTYPE html><html><head>")
      + "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      + "<style>body{background:#111;color:#eee;font-family:Arial;text-align:center;padding-top:3rem}</style></head><body>"
      + "<h2 style='color:#3ecf8e'>Credentials Saved!</h2>"
      + "<p>Restarting...</p>"
      + "<p style='color:#888;font-size:.9rem'>Reconnect to your WiFi, then visit<br>"
      + "<strong>http://" + mdnsName + ".local</strong></p></body></html>");
    delay(1500);
    ESP.restart();
  });

  server.on("/wifi/scan", HTTP_GET, []() {
    int n = WiFi.scanNetworks();
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject net = arr.add<JsonObject>();
      net["ssid"]   = WiFi.SSID(i);
      net["rssi"]   = WiFi.RSSI(i);
      net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
    WiFi.scanDelete();
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
}

// ============================================================
//  WiFi — STATION CONNECT
// ============================================================
bool connectWiFi() {
  String ssid, pass;
  loadWiFiCreds(ssid, pass);
  if (ssid.isEmpty()) { Serial.println("[WiFi] No saved credentials"); return false; }
  Serial.printf("[WiFi] Connecting to: %s\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected  IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }
  Serial.println("[WiFi] Connection failed");
  return false;
}

// ============================================================
//  ArduinoOTA
// ============================================================
void setupArduinoOTA() {
  ArduinoOTA.setHostname(sanitizeMDNS(deviceName).c_str());
  ArduinoOTA.onStart([]() {
    tft.fillScreen(COL_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COL_ACCENT, COL_BG);
    tft.setTextFont(4);
    tft.drawString("OTA Update...", 120, 120);
    Serial.println("[ArduinoOTA] Start");
  });
  ArduinoOTA.onEnd([]() {
    tft.setTextColor(COL_GREEN, COL_BG);
    tft.setTextFont(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Done! Rebooting...", 120, 155);
    Serial.println("\n[ArduinoOTA] Done");
  });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    if (t > 0) {
      int pct = p * 100 / t;
      tft.fillRect(20, 155, 200 * pct / 100, 12, COL_ACCENT);
      tft.drawRect(20, 155, 200, 12, COL_BORDER);
      Serial.printf("[ArduinoOTA] %u%%\r", pct);
    }
  });
  ArduinoOTA.onError([](ota_error_t err) {
    const char* msgs[] = {"","Auth failed","Begin failed","Connect failed","Receive failed","End failed"};
    Serial.printf("[ArduinoOTA] Error[%u]: %s\n", err,
                  err <= 5 ? msgs[err] : "Unknown");
  });
  ArduinoOTA.begin();
  otaRunning = true;
  Serial.println("[ArduinoOTA] Ready on port 3232");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] Freenove ESP32-S3 Display " FIRMWARE_VERSION);
  bootMillis = millis();

  // Force backlight on before init — GPIO floats low after OTA reboot
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init();
  tft.setRotation(0);      // portrait; change to 2 if display is upside-down
  initColors();
  drawBootScreen();

  // NVS
  loadPrefs();
  Serial.printf("[Config] ctrlHost: %s\n", ctrlHost.c_str());

  // WiFi
  String ssid, pass;
  loadWiFiCreds(ssid, pass);
  bool connected = !ssid.isEmpty() && connectWiFi();

  if (!connected) {
    Serial.println(ssid.isEmpty()
      ? "[WiFi] No credentials — starting AP"
      : "[WiFi] Could not connect — starting AP");
    startAPPortal();
    return;
  }

  setupPowerManagement();
  startMDNS();
  setupArduinoOTA();
  setupRoutes();
  ElegantOTA.begin(&server, OTA_USER, OTA_PASS);
  server.begin();
  Serial.println("[HTTP] Server started");

  // Initial poll immediately
  nextPollAt = 0;
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  server.handleClient();
  ElegantOTA.loop();
  if (otaRunning) ArduinoOTA.handle();

  // WiFi reconnect watchdog
  if (!inAPMode
      && WiFi.getMode() == WIFI_STA
      && WiFi.status() != WL_CONNECTED
      && millis() - lastWiFiRetry >= WIFI_RETRY_INTERVAL)
  {
    lastWiFiRetry = millis();
    Serial.println("[WiFi] Reconnecting...");
    String ssid, pass;
    loadWiFiCreds(ssid, pass);
    WiFi.begin(ssid.c_str(), pass.c_str());
  }

  // Poll controller
  if (!inAPMode && millis() >= nextPollAt) {
    if (!pollController())
      nextPollAt = millis() + 15000UL;
  }

  // Poll plant monitor
  if (!inAPMode && !plantHost.isEmpty() && millis() >= nextPlantPollAt) {
    pollPlant();   // sets nextPlantPollAt internally on both success and failure
  }

  // Full screen redraw when new data arrives
  if (!inAPMode && needsFullRedraw) {
    drawMainScreen();
  }

  // Periodic partial refresh (updated-ago footer and WiFi quality)
  if (!inAPMode && !needsFullRedraw
      && millis() - lastDisplayRefresh >= DISPLAY_REFRESH_MS)
  {
    lastDisplayRefresh = millis();
    drawHumidityPanel();
    drawPlantPanel();
  }

  delay(50);
}
