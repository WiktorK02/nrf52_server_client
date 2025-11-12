#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <stdarg.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

using namespace Adafruit_LittleFS_Namespace;

extern "C" {
  #include "ble_gap.h"   // RSSI z SoftDevice
}

// ================== KONFIG ==================
#ifndef PIN_015
#define PIN_015 (15)
#endif
#define LED_PIN PIN_015

#ifndef PIN_022
#define PIN_022 (22)     // teraz: przycisk klawisza A/B (server i client)
#endif
#define IO_PIN_022 PIN_022   // pozostawione dla kompatybilności, ale nieużywane

#ifndef PIN_024
#define PIN_024 (24)     // serwer: przycisk listy
#endif
#define BTN_LIST_PIN PIN_024

#ifndef PIN_029
#define PIN_029 (29)     // przycisk wyboru strony klawiatury (LEFT/RIGHT)
#endif
#define SIDE_BTN_PIN PIN_029

#define USB_LOG   Serial
#define UART_DOCK Serial1
#define UART_BAUD 115200

// UART pins (nRF52 P0.xx numbers)
#define UART_RX_PIN  8
#define UART_TX_PIN  6

// PIN klawisza (wspólny dla servera i clienta)
#define KBD_BTN_PIN PIN_022

// ===== OLED =====
#ifndef OLED_ADDR
  #define OLED_ADDR 0x3C
#endif
#ifndef OLED_W
  #define OLED_W 128
#endif
#ifndef OLED_H
  #define OLED_H 32   // zmień na 64 jeśli masz 128x64
#endif

// ===== BATTERY SENSE =====
#ifndef BAT_CHG_PIN
  #define BAT_CHG_PIN PIN_004   // np. 4 / 31 itd. Dostosuj do swojego układu
#endif

// Przybliżone napięcia dla 1S LiPo (dostosuj do swojej baterii)
#define BATT_MIN_V 3.30f   // napięcie ~0%
#define BATT_MAX_V 4.20f   // napięcie ~100%

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
static bool g_oled_ok = false;

// —— tryb listy klientów na OLED (serwer) ——
static bool g_oled_show_list = false;

// —— wybór strony klawiatury ——
static bool g_is_left_side = true;   // true = LEFT => 'A', false = RIGHT => 'B'
static bool g_is_server    = false;  // ustawiane w setup()

// ===== STAN BATERII =====
static float   g_battVoltage  = 0.0f;
static uint8_t g_battPercent  = 0;
static bool    g_battCharging = false;

// filtrowanie napięcia
static bool  g_battFiltInit    = false;
static float g_battVoltageFilt = 0.0f;

// ===== FLASH CONFIG KLIENTA =====
struct ClientConfig { uint32_t magic; uint32_t client_id; };
static const uint32_t CFG_MAGIC = 0x52494E4B;          // 'RINK'
static const char*    CFG_PATH  = "/client_cfg.bin";
ClientConfig g_cfg;

// ===== Utils =====
static inline bool usb_mounted() { return TinyUSBDevice.mounted(); }
static inline void soft_reset()  { NVIC_SystemReset(); }

static inline void ledBlink(uint8_t n, uint16_t on=60, uint16_t off=120){
  for(uint8_t i=0;i<n;i++){
    digitalWrite(LED_PIN,HIGH); delay(on);
    digitalWrite(LED_PIN,LOW);  delay(off);
  }
}

static inline String readLine(Stream& s, uint32_t to_ms=500){
  uint32_t t0=millis(); String line;
  while(millis()-t0<to_ms){
    while(s.available()){
      char c=(char)s.read();
      if(c=='\n') return line;
      if(c!='\r') line+=c;
    }
    yield();
  }
  return "";
}

static inline String macToString(const uint8_t mac[6]){
  char b[18];
  sprintf(b,"%02X:%02X:%02X:%02X:%02X:%02X",
          mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
  return String(b);
}

// Stabilne logowanie: zawsze CRLF
static inline void LOGF(const char* fmt, ...) {
  char buf[220];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  USB_LOG.print(buf);
  USB_LOG.print("\r\n");
  USB_LOG.flush();
}

// Prawdziwy RSSI (N/A gdy brak próbki)
static inline int getRealRssi(uint16_t conn_handle) {
  int8_t rssi_value = 0;
  uint8_t ch_index = 0;
  uint32_t err = sd_ble_gap_rssi_get(conn_handle, &rssi_value, &ch_index);
  if (err == NRF_SUCCESS) return (int)rssi_value;
  return INT16_MIN; // brak danych
}

// ===== ODCZYT NAPIĘCIA BATERII (nRF52 SAADC, VDDH/5) =====
// — przeliczenie dokładnie jak w Twoim przykładzie —
float readBatteryVoltage() {
  volatile uint32_t raw_value = 0;

  // Configure SAADC
  NRF_SAADC->ENABLE = 1;
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;

  NRF_SAADC->CH[0].CONFIG =
    (SAADC_CH_CONFIG_GAIN_Gain1_4 << SAADC_CH_CONFIG_GAIN_Pos) |
    (SAADC_CH_CONFIG_MODE_SE      << SAADC_CH_CONFIG_MODE_Pos) |
    (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos);

  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDDHDIV5;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;

  // Single sample
  NRF_SAADC->RESULT.PTR    = (uint32_t)&raw_value;
  NRF_SAADC->RESULT.MAXCNT = 1;
  NRF_SAADC->TASKS_START   = 1;
  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0;
  NRF_SAADC->TASKS_SAMPLE   = 1;
  while (!NRF_SAADC->EVENTS_END);
  NRF_SAADC->EVENTS_END   = 0;
  NRF_SAADC->TASKS_STOP   = 1;
  while (!NRF_SAADC->EVENTS_STOPPED);
  NRF_SAADC->EVENTS_STOPPED = 0;
  NRF_SAADC->ENABLE = 0;

  // Force explicit double-precision calculations
  double raw_double = (double)raw_value;
  double step1 = raw_double * 2.4;     // referencja 2.4V
  double step2 = step1 / 4095.0;       // 12-bit
  double vddh  = 5.0 * step2;          // VDDH = 5 * VDDHDIV5

  return (float)vddh;
}

// Przeliczenie napięcia baterii na % (całkowite)


// Dokładny procent jako float (przed zaokrągleniem) – do histerezy
static inline float batteryVoltageToPercent(float v) {
  if (v <= BATT_MIN_V) return 0.f;
  if (v >= BATT_MAX_V) return 100.f;
  return (v - BATT_MIN_V) * 100.f / (BATT_MAX_V - BATT_MIN_V);
}

// Jedno miejsce gdzie odświeżamy globalne zmienne baterii
// + filtr + wykrywanie ładowania, żeby wartości nie skakały
static void updateBatteryStatus() {
  float vRaw = readBatteryVoltage();
  uint32_t now = millis();

  // — stan wewnętrzny
  static bool first_run = true;

  // Debounce ładowania (LOW na BAT_CHG_PIN = ładuje)
  static bool     chg_state = false;      // zaakceptowany stan
  static bool     chg_pending = false;
  static uint32_t chg_change_ms = 0;
  const  uint16_t CHG_DEBOUNCE_MS = 500;

  // Okno „fast” po starcie / zmianie ładowania (szybkie osiadanie)
  static uint32_t fast_until_ms = 0;

  // Histereza dla procenta (Schmitt)
  static uint8_t pStable = 0;             // stabilny, wyświetlany %
  static bool    pInit   = false;
  const  float   H = 1.2f;                // szerokość martwej strefy (±1.2%)

  // — 1) szybki start — natychmiastowy, dokładny %
  if (first_run) {
    float vClamped = (vRaw > BATT_MAX_V) ? BATT_MAX_V : vRaw;
    g_battVoltageFilt = vRaw;
    g_battVoltage     = vRaw;
    g_battFiltInit    = true;
    pStable           = (uint8_t)roundf(batteryVoltageToPercent(vClamped));
    pInit             = true;
    g_battPercent     = pStable;          // OD RAZU na starcie
    first_run         = false;
    fast_until_ms     = now + 1500;       // krótki boost po starcie
  }

  // — 2) stabilna detekcja ładowania (pin lub pik napięcia)
  bool pinCharging = false;
  if (BAT_CHG_PIN >= 0) pinCharging = (digitalRead(BAT_CHG_PIN) == LOW);
  bool vSuggestsCharging = (vRaw > (BATT_MAX_V + 0.05f));
  bool chg_raw = pinCharging || vSuggestsCharging;

  if (chg_raw != chg_pending) { chg_pending = chg_raw; chg_change_ms = now; }
  bool prev_chg_state = chg_state;
  if ((now - chg_change_ms) >= CHG_DEBOUNCE_MS) chg_state = chg_pending;
  g_battCharging = chg_state;

  // — 3) zbocze stanu ładowania → natychmiastowy „snap” + okno fast
  if (chg_state != prev_chg_state) {
    g_battVoltageFilt = vRaw;
    g_battVoltage     = vRaw;
    uint8_t pctNow    = (uint8_t)roundf(batteryVoltageToPercent((vRaw > BATT_MAX_V) ? BATT_MAX_V : vRaw));
    pStable           = pctNow;           // od razu prawda także W TRAKCIE ładowania
    g_battPercent     = pStable;
    fast_until_ms     = now + 2000;       // 2 s szybkiego osiadania
  }

  // — 4) filtr EMA adaptacyjny: duży skok → szybciej
  float dv    = fabsf(vRaw - g_battVoltageFilt);
  bool  big   = (dv > 0.06f);             // ~60 mV
  float alpha = ((now < fast_until_ms) || big) ? 0.80f : 0.12f;
  g_battVoltageFilt = g_battVoltageFilt + alpha * (vRaw - g_battVoltageFilt);
  g_battVoltage     = g_battVoltageFilt;

  // — 5) procent (float), clamp do MAX żeby nie wskakiwało sztuczne 100%
  float  vForPct = (g_battVoltageFilt > BATT_MAX_V) ? BATT_MAX_V : g_battVoltageFilt;
  float  pFloat  = batteryVoltageToPercent(vForPct);

  // — 6) histereza Schmitta ±1.2% (DZIAŁA ZAWSZE — także gdy ładuje)
  if (!pInit) { pStable = (uint8_t)roundf(pFloat); pInit = true; }

  if (now < fast_until_ms) {
    // w trybie fast aktualizuj natychmiast (po starcie / zmianie CHG)
    pStable = (uint8_t)roundf(pFloat);
  } else {
    // poza fast: zmiana o 1 pp dopiero po wyjściu poza martwą strefę
    float upTh   = (float)pStable + H;
    float downTh = (float)pStable - H;
    if (pFloat >= upTh  && pStable < 100) pStable++;      // rośnij o 1
    else if (pFloat <= downTh && pStable > 0)  pStable--; // malej o 1
    // w środku martwej strefy nic — brak ping-pongu
  }

  g_battPercent = pStable;  // AKTUALIZUJEMY TAKŻE PODCZAS ŁADOWANIA
}

// ===== OLED helpers =====
static inline void oledSafeDisplay() { if (g_oled_ok) display.display(); }

// Rysowanie ikonki baterii w prawym górnym rogu
static void drawBatteryIcon() {
  if (!g_oled_ok) return;

  const int iconW = 16;
  const int iconH = 8;
  int x = OLED_W - iconW - 1;
  int y = 0;

  // Ramka baterii
  display.drawRect(x, y, iconW-2, iconH, SSD1306_WHITE);
  // "czubek"
  display.drawRect(x + iconW - 3, y + 2, 2, iconH-4, SSD1306_WHITE);

  // Wypełnienie wnętrza zależnie od % (bez ramek) – tylko gdy NIE ładuje
  int innerW = iconW - 4;
  int innerH = iconH - 2;
  int fillW  = (int)((innerW * g_battPercent) / 100);

  if (fillW < 0) fillW = 0;
  if (fillW > innerW) fillW = innerW;

  if (!g_battCharging && fillW > 0) {
    display.fillRect(x + 2, y + 1, fillW, innerH, SSD1306_WHITE);
  }

  // Tekst małym fontem obok ikonki
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x - 28, y);

  if (g_battCharging) {
    display.print("CHG");
  } else {
    display.print(g_battPercent);
    display.print("%");
  }

  // Błyskawica przy ładowaniu
  if (g_battCharging) {
    int bx = x + 3;   // przesunięcie wewnątrz baterii
    int by = y + 1;

    const uint8_t boltW = 8;
    const uint8_t boltH = 5;
    const uint8_t bolt[boltH][boltW] = {
      {0,1,1,0,0,0,0,0},
      {1,1,1,0,0,0,0,0},
      {0,0,1,1,1,0,0,0},
      {0,0,0,1,1,1,0,0},
      {0,0,0,0,1,1,0,0}
    };

    for (uint8_t ry = 0; ry < boltH; ry++) {
      for (uint8_t rx = 0; rx < boltW; rx++) {
        if (bolt[ry][rx]) {
          display.drawPixel(bx + rx, by + ry, SSD1306_WHITE);
        }
      }
    }
  }
}

static void oledShowServer(const char* bleState, uint8_t conn_count, uint32_t uptime_s) {
  if (!g_oled_ok) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("SERVER ");
  display.println(g_is_left_side ? "L" : "R");
  display.print("BLE: ");
  display.print(bleState);
  display.print(" ");
  display.println(conn_count);
  display.print("up: ");
  display.print(uptime_s);
  display.println("s");

  drawBatteryIcon();
  oledSafeDisplay();
}

static void oledShowClientIdle(uint32_t uptime_s) {
  if (!g_oled_ok) return;
  display.clearDisplay();
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("CLIENT ");
  display.print(g_is_left_side ? "L" : "R");
  display.println(" ");
  display.println("Dock to assign ID");
  display.print("up: "); 
  display.print(uptime_s); 
  display.println("s");

  drawBatteryIcon();
  oledSafeDisplay();
}

static void oledShowClientBLE(uint32_t id, bool connected, uint32_t uptime_s) {
  if (!g_oled_ok) return;
  display.clearDisplay();
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("CLIENT ");
  display.print(g_is_left_side ? "L" : "R");
  display.println(" (BLE)");
  display.print("ID: "); 
  display.println((unsigned long)id);
  display.println(connected ? "State: connected" : "State: advertising");
  display.print("up: "); 
  display.print(uptime_s); 
  display.println("s");

  drawBatteryIcon();
  oledSafeDisplay();
}

// ===== KLIENT: zapis/odczyt ID =====
bool cfg_load() {
  if (!InternalFS.begin()) return false;
  File f(InternalFS.open(CFG_PATH, FILE_O_READ));
  if (!f) return false;
  if (f.size() != sizeof(ClientConfig)) { f.close(); return false; }
  f.read(&g_cfg, sizeof(ClientConfig)); f.close();
  return g_cfg.magic == CFG_MAGIC;
}
bool cfg_save() {
  if (!InternalFS.begin()) return false;
  File f(InternalFS.open(CFG_PATH, FILE_O_WRITE));
  if (!f) return false;
  g_cfg.magic = CFG_MAGIC;
  size_t n = f.write((const uint8_t*)&g_cfg, sizeof(ClientConfig));
  f.flush(); f.close();
  return n == sizeof(ClientConfig);
}

// Parser nazwy z ADV (dla 1.6.x Bluefruit)
bool adv_get_name(ble_gap_evt_adv_report_t* rpt, char* out, size_t outlen) {
  if (!rpt || !out || outlen == 0) return false;
  out[0] = 0;
  const uint8_t* p = rpt->data.p_data;
  uint8_t len = rpt->data.len;
  while (len > 2) {
    uint8_t ad_len  = p[0];
    if (ad_len == 0 || ad_len + 1 > len) break;
    uint8_t ad_type = p[1];
    if (ad_type == 0x09 || ad_type == 0x08) { // Complete/Shortened Local Name
      uint8_t name_len = ad_len - 1;
      if (name_len >= outlen) name_len = outlen - 1;
      memcpy(out, &p[2], name_len);
      out[name_len] = 0;
      return true;
    }
    uint8_t step = ad_len + 1;
    p   += step;
    len -= step;
  }
  return false;
}

// ================== SERWER: baza zajętych ID ==================
struct IdEntry { uint32_t id; uint8_t mac[6]; };
static const char* SERVER_ID_DB = "/server_ids.bin";
static const uint32_t MAX_IDS = 128;
static IdEntry g_ids[MAX_IDS];
static uint32_t g_ids_count = 0;
static uint32_t g_next_id = 1;        // start next candidate

static bool iddb_load() {
  if (!InternalFS.begin()) return false;
  File f(InternalFS.open(SERVER_ID_DB, FILE_O_READ));
  if (!f) return false;
  size_t n = f.read(g_ids, sizeof(g_ids));
  f.close();
  g_ids_count = n/sizeof(IdEntry);
  if (g_ids_count > MAX_IDS) g_ids_count = MAX_IDS;
  return true;
}
static void iddb_save() {
  if (!InternalFS.begin()) return;
  File f(InternalFS.open(SERVER_ID_DB, FILE_O_WRITE));
  if (!f) return;
  f.write((const uint8_t*)g_ids, g_ids_count*sizeof(IdEntry));
  f.flush(); f.close();
}
static bool id_is_used(uint32_t id) {
  for (uint32_t i=0;i<g_ids_count;i++) if (g_ids[i].id==id) return true;
  return false;
}

static int idx_by_mac(const uint8_t mac[6]){
  for (uint32_t i=0;i<g_ids_count;i++){
    if (memcmp(g_ids[i].mac, mac, 6)==0) return (int)i;
  }
  return -1;
}
static int idx_by_id(uint32_t id){
  for (uint32_t i=0;i<g_ids_count;i++){
    if (g_ids[i].id == id) return (int)i;
  }
  return -1;
}
static void remove_idx(int idx){
  if (idx<0 || (uint32_t)idx>=g_ids_count) return;
  g_ids[idx] = g_ids[g_ids_count-1];
  g_ids_count--;
}

static void id_set(uint32_t id, const uint8_t mac[6]) {
  int i_mac = idx_by_mac(mac);
  int i_id  = idx_by_id(id);

  if (i_mac >= 0 && i_id >= 0) {
    if (i_mac != i_id) {
      memcpy(g_ids[i_id].mac, mac, 6);
      remove_idx(i_mac);
    }
  } else if (i_id >= 0) {
    memcpy(g_ids[i_id].mac, mac, 6);
  } else if (i_mac >= 0) {
    g_ids[i_mac].id = id;
  } else if (g_ids_count < MAX_IDS) {
    g_ids[g_ids_count].id = id;
    memcpy(g_ids[g_ids_count].mac, mac, 6);
    g_ids_count++;
  }
  iddb_save();
}

static uint32_t id_next_free(uint32_t start_from=1) {
  uint32_t cand = start_from;
  while (id_is_used(cand)) cand++;
  return cand;
}

// format klientów do logów
static void format_clients_inline(char* out, size_t outlen) {
  if (!out || outlen == 0) return;
  out[0] = 0;
  if (g_ids_count == 0) { snprintf(out, outlen, "(none)"); return; }
  size_t used = 0;
  for (uint32_t i=0; i<g_ids_count; ++i) {
    char macStr[18];
    sprintf(macStr,"%02X:%02X:%02X:%02X:%02X:%02X",
            g_ids[i].mac[5], g_ids[i].mac[4], g_ids[i].mac[3],
            g_ids[i].mac[2], g_ids[i].mac[1], g_ids[i].mac[0]);
    int n = snprintf(out + used,
                     (used < outlen ? outlen - used : 0),
                     "%sID=%lu MAC=%s",
                     (i==0 ? "" : ", "),
                     (unsigned long)g_ids[i].id, macStr);
    if (n < 0) break;
    used += (size_t)n;
    if (used >= outlen) { out[outlen-1]=0; break; }
  }
}

// OLED lista klientów
static void oledShowClientsList() {
  if (!g_oled_ok) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("CLIENTS");

  uint8_t maxRows = OLED_H / 8;
  if (maxRows == 0) maxRows = 1;
  uint8_t rowsAvail = (maxRows > 1) ? (maxRows - 1) : 0;

  uint8_t shown = 0;
  for (uint32_t i = 0; i < g_ids_count && shown < rowsAvail; ++i) {
    char macStr[18];
    sprintf(macStr,"%02X:%02X:%02X:%02X:%02X:%02X",
            g_ids[i].mac[5], g_ids[i].mac[4], g_ids[i].mac[3],
            g_ids[i].mac[2], g_ids[i].mac[1], g_ids[i].mac[0]);

    display.print("ID");
    display.print((unsigned long)g_ids[i].id);
    display.print("  ");
    display.println(macStr);
    shown++;
  }

  if (g_ids_count > shown) {
    display.print("... total=");
    display.println((unsigned long)g_ids_count);
  }

  drawBatteryIcon();
  oledSafeDisplay();
}

// ===== USB HID KEYBOARD (SERWER) ==================
uint8_t const desc_hid_report[] = { TUD_HID_REPORT_DESC_KEYBOARD() };

Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report),
                          HID_ITF_PROTOCOL_KEYBOARD, 10, false);

// Tap (debug / awaryjnie)
static void hid_tap_key_immediate(uint8_t keycode) {
  if (!TinyUSBDevice.mounted()) return;

  if (TinyUSBDevice.suspended()) {
    TinyUSBDevice.remoteWakeup();
    delay(2);
  }

  {
    uint32_t t0 = millis();
    while (!usb_hid.ready() && (millis() - t0) < 30) { yield(); }
    uint8_t keys[6] = { keycode, 0,0,0,0,0 };
    usb_hid.keyboardReport(0, 0, keys);   // key down
  }

  delay(10);

  {
    uint32_t t0 = millis();
    while (!usb_hid.ready() && (millis() - t0) < 30) { yield(); }
    uint8_t empty[6] = {0,0,0,0,0,0};
    usb_hid.keyboardReport(0, 0, empty);  // key up
  }
}

// HOLD-style: naciśnij i TRZYMAJ
static void hid_key_down(uint8_t keycode) {
  if (!TinyUSBDevice.mounted()) return;

  if (TinyUSBDevice.suspended()) {
    TinyUSBDevice.remoteWakeup();
    delay(2);
  }

  uint32_t t0 = millis();
  while (!usb_hid.ready() && (millis() - t0) < 30) { yield(); }

  uint8_t keys[6] = { keycode, 0,0,0,0,0 };
  usb_hid.keyboardReport(0, 0, keys); // key held down
}

// PUŚĆ WSZYSTKO
static void hid_key_up_all() {
  if (!TinyUSBDevice.mounted()) return;

  uint32_t t0 = millis();
  while (!usb_hid.ready() && (millis() - t0) < 30) { yield(); }

  uint8_t empty[6] = {0,0,0,0,0,0};
  usb_hid.keyboardReport(0, 0, empty); // release
}

// ================== BLE ==================
BLEUart       bleuart;         // peripheral (klient BLE)
BLEClientUart clientUart;      // central (serwer)
uint16_t g_conn = BLE_CONN_HANDLE_INVALID;
static uint32_t g_active_id = 0;
static uint8_t  g_active_mac[6] = {0};

// ===== Przycisk listy (SERVER) — ISR =====
volatile bool     g_btn024_irq     = false;
volatile uint32_t g_btn024_last_us = 0;
static void btn024_isr() {
  uint32_t now = micros();
  if (now - g_btn024_last_us < 150000U) return;
  g_btn024_last_us = now;
  g_btn024_irq = true;
}

// ===== Przycisk wyboru strony (PIN_029) — ISR =====
volatile bool     g_side_btn_irq     = false;
volatile uint32_t g_side_btn_last_us = 0;
static void side_btn_isr() {
  uint32_t now = micros();
  if (now - g_side_btn_last_us < 150000U) return; // ~150 ms debounce
  g_side_btn_last_us = now;
  g_side_btn_irq = true;
}

// ===== SERVER CALLBACKS =====
void server_connect_cb(uint16_t conn_handle) {
  g_conn = conn_handle;
  ledBlink(2,60,80);

  bool ok = false;
  for (int i=0; i<3 && !ok; ++i) {
    ok = clientUart.discover(conn_handle);
    if (!ok) delay(150);
  }
  if (!ok) {
    LOGF("[SERVER] BLE discovery failed (3x), disconnecting");
    Bluefruit.disconnect(conn_handle);
    return;
  }
  clientUart.enableTXD();

  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  ble_gap_addr_t addr = conn->getPeerAddr();
  int rssi_meas = getRealRssi(conn_handle);

  char macStr[18];
  sprintf(macStr,"%02X:%02X:%02X:%02X:%02X:%02X",
          addr.addr[5], addr.addr[4], addr.addr[3],
          addr.addr[2], addr.addr[1], addr.addr[0]);

  char name[32] = {0};
  conn->getPeerName(name, sizeof(name));
  int id = 0;
  if (strncmp(name, "Client-", 7) == 0) id = atoi(name + 7);
  if (id > 0) id_set((uint32_t)id, addr.addr);

  memcpy(g_active_mac, addr.addr, 6);
  g_active_id = id;

  if (rssi_meas == INT16_MIN)
    LOGF("[SERVER] CONNECTED → ID=%d MAC=%s RSSI=N/A", id, macStr);
  else
    LOGF("[SERVER] CONNECTED → ID=%d MAC=%s RSSI=%d dBm", id, macStr, rssi_meas);

  if (g_oled_ok && !g_oled_show_list) {
    oledShowServer("connected", 1, millis()/1000);
  }
}

void server_disconnect_cb(uint16_t, uint8_t reason) {
  g_conn = BLE_CONN_HANDLE_INVALID;
  char macStr[18];
  sprintf(macStr,"%02X:%02X:%02X:%02X:%02X:%02X",
          g_active_mac[5],g_active_mac[4],g_active_mac[3],
          g_active_mac[2],g_active_mac[1],g_active_mac[0]);
  LOGF("[SERVER] DISCONNECTED → ID=%lu MAC=%s (reason=%u)",
      (unsigned long)g_active_id, macStr, reason);

  memset(g_active_mac,0,sizeof(g_active_mac));
  g_active_id = 0;
  ledBlink(1,200,200);

  if (g_oled_ok && !g_oled_show_list) {
    oledShowServer("scanning", 0, millis()/1000);
  }
}

// ===== SERVER SCAN CB =====
void server_scan_cb(ble_gap_evt_adv_report_t* report){
  if (Bluefruit.Scanner.checkReportForService(report, clientUart)) {
    Bluefruit.Scanner.stop();
    Bluefruit.Central.connect(report);
    return;
  }
  char name[32] = {0};
  if (adv_get_name(report, name, sizeof(name))) {
    if (strncmp(name, "Client-", 7) == 0) {
      LOGF("[SERVER] found by name: %s -> connecting", name);
      Bluefruit.Scanner.stop();
      Bluefruit.Central.connect(report);
      return;
    } else {
      LOGF("[SERVER] seen ADV: %s", name);
    }
  }
}

// ===== SERVER UART HELPERY (HELLO/ASSIGN/ACK) =====
void server_pollDock_andAssign() {
  enum { WAIT_HELLO, SENT_ASSIGN } static state = WAIT_HELLO;
  static uint32_t lastWHO = 0, assigned = 0, tAssign = 0;
  static uint8_t retries = 0;
  static uint8_t last_mac[6] = {0};

  if (state == WAIT_HELLO) {
    if (millis() - lastWHO > 800) {
      lastWHO = millis();
      UART_DOCK.println("WHO");
    }
    String line = readLine(UART_DOCK, 10);
    if (line.startsWith("HELLO ")) {
      memset(last_mac, 0, sizeof(last_mac));
      String macStr = line.substring(6);
      int b[6]={0};
      if (sscanf(macStr.c_str(),
                 "%02x:%02x:%02x:%02x:%02x:%02x",
                 &b[5],&b[4],&b[3],&b[2],&b[1],&b[0])==6) {
        for (int i=0;i<6;i++) last_mac[i]=(uint8_t)b[i];
      }
      int known = idx_by_mac(last_mac);
      if (known >= 0) assigned = g_ids[known].id;
      else            assigned = id_next_free(g_next_id);

      UART_DOCK.print("ASSIGN ");
      UART_DOCK.println(assigned);
      LOGF("[SERVER] assign %lu to %s (wait ack)",
           (unsigned long)assigned, macStr.c_str());
      retries = 0;
      tAssign = millis();
      state = SENT_ASSIGN;
    }
  } else { // SENT_ASSIGN
    String ack = readLine(UART_DOCK, 10);
    if (ack == "OK") {
      LOGF("[SERVER] ack OK for ID %lu", (unsigned long)assigned);
      id_set(assigned, last_mac);
      if (assigned >= g_next_id) g_next_id = assigned + 1;
      UART_DOCK.println("GO");
      ledBlink(2,120,120);
      lastWHO = millis() + 2000;
      state = WAIT_HELLO;
      return;
    }
    if (millis() - tAssign > 600) {
      if (++retries <= 3) {
        tAssign = millis();
        UART_DOCK.print("ASSIGN ");
        UART_DOCK.println(assigned);
        LOGF("[SERVER] retry %u for ID %lu",
             retries, (unsigned long)assigned);
      } else {
        LOGF("[SERVER] no-ack for ID %lu, giving up",
             (unsigned long)assigned);
        state = WAIT_HELLO;
      }
    }
  }
}

// ===== helper do wysyłania komendy z clienta =====
static void client_send_line(const char* line) {
  if (Bluefruit.connected()) {
    bleuart.write((const uint8_t*)line, strlen(line));
    LOGF("[CLIENT] SENT: %s", line);

    if (g_oled_ok) {
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 16);
      display.println(line);
      drawBatteryIcon();
      oledSafeDisplay();
    }
  } else {
    LOGF("[CLIENT] (not connected) WOULD SEND: %s", line);
  }
}

// ===== wspólny przycisk klawisza na PIN_022 (A/B) =====
static const bool KBD_BTN_ACTIVE_LOW    = true;   // guzik do GND, INPUT_PULLUP
static const uint32_t KBD_BTN_STABLE_MS = 40;

static bool     kbd_btn_raw_last          = true;
static uint32_t kbd_btn_raw_change_ms     = 0;
static bool     kbd_btn_filtered_state    = true;
static bool     kbd_btn_pressed_logical   = false;

static inline bool kbd_hw_is_pressed(bool raw_level) {
  return KBD_BTN_ACTIVE_LOW ? (!raw_level) : (raw_level);
}

static void keyboard_button_poll() {
  uint32_t now = millis();
  bool raw_now = digitalRead(KBD_BTN_PIN); // HIGH/LOW z pinu 22

  // detekcja zmiany surowej
  if (raw_now != kbd_btn_raw_last) {
    kbd_btn_raw_last      = raw_now;
    kbd_btn_raw_change_ms = now;
  }

  // sprawdzamy czy różnica jest stabilna
  if (raw_now != kbd_btn_filtered_state) {
    if ((now - kbd_btn_raw_change_ms) >= KBD_BTN_STABLE_MS) {
      kbd_btn_filtered_state = raw_now;
      bool pressed_now = kbd_hw_is_pressed(kbd_btn_filtered_state);

      uint8_t keycode   = g_is_left_side ? HID_KEY_A : HID_KEY_B;
      char    keyLetter = g_is_left_side ? 'A'        : 'B';

      // przejście: released -> pressed
      if (pressed_now && !kbd_btn_pressed_logical) {
        kbd_btn_pressed_logical = true;
        LOGF("[%s] KBD BTN DOWN (%c)",
             g_is_server ? "SERVER" : "CLIENT",
             keyLetter);

        if (g_is_server) {
          // bezpośredni HID do hosta
          hid_key_down(keycode);
        } else {
          // klient wysyła do serwera
          char line[16];
          snprintf(line, sizeof(line), "KEY %c DOWN\n", keyLetter);
          client_send_line(line);
        }
      }
      // przejście: pressed -> released
      else if (!pressed_now && kbd_btn_pressed_logical) {
        kbd_btn_pressed_logical = false;
        LOGF("[%s] KBD BTN UP (%c)",
             g_is_server ? "SERVER" : "CLIENT",
             keyLetter);

        if (g_is_server) {
          hid_key_up_all();
        } else {
          char line[16];
          snprintf(line, sizeof(line), "KEY %c UP\n", keyLetter);
          client_send_line(line);
        }
      }
    }
  }
}

// ===== SERWER most BLE <-> USB LOG + HID =====
void server_bridgeUsbBle(){
  // host USB -> BLE echo
  if (g_conn!=BLE_CONN_HANDLE_INVALID && USB_LOG.available()){
    while(USB_LOG.available()){
      char c=USB_LOG.read();
      clientUart.write(&c,1);
      if(c=='\n') break;
    }
  }

  // BLE -> HID / log / OLED
  if (g_conn!=BLE_CONN_HANDLE_INVALID && clientUart.available()){
    String s = clientUart.readStringUntil('\n');

    if (s.startsWith("KEY ")) {
      // format: "KEY X DOWN" / "KEY X UP"
      if (s.length() < 8) {
        LOGF("[SERVER] malformed KEY cmd: %s", s.c_str());
      } else {
        char which = s.charAt(4); // 'A' lub 'B'
        bool isDown = (s.indexOf("DOWN") >= 0);
        uint8_t keycode;

        if (which == 'A')      keycode = HID_KEY_A;
        else if (which == 'B') keycode = HID_KEY_B;
        else {
          LOGF("[SERVER] unknown KEY: %c", which);
          return;
        }

        if (isDown) {
          LOGF("[SERVER] KEY %c DOWN -> hold start", which);
          hid_key_down(keycode);

          if (g_oled_ok && !g_oled_show_list) {
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(80, (OLED_H >= 64) ? 40 : 24);
            display.print(which);
            display.print(":DOWN");
            drawBatteryIcon();
            oledSafeDisplay();
          }
        } else {
          LOGF("[SERVER] KEY %c UP -> hold end", which);
          hid_key_up_all();

          if (g_oled_ok && !g_oled_show_list) {
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(80, (OLED_H >= 64) ? 40 : 24);
            display.print(which);
            display.print(":UP  ");
            drawBatteryIcon();
            oledSafeDisplay();
          }
        }
      }
    }

    else {
      LOGF("[BLE] %s", s.c_str());
    }
  }
}

// ===== SERVER MAIN LOOP =====
void run_server(){
  Bluefruit.begin(0,1);
  Bluefruit.setTxPower(4);
  clientUart.begin();

  // Przycisk listy (serwer)
  pinMode(BTN_LIST_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_LIST_PIN), btn024_isr, FALLING);

  // baza ID
  iddb_load();
  for (uint32_t i=0; i<g_ids_count; ++i){
    for (uint32_t j=i+1; j<g_ids_count; ){
      if (memcmp(g_ids[i].mac, g_ids[j].mac, 6)==0 ||
          g_ids[i].id==g_ids[j].id){
        remove_idx(j);
      } else ++j;
    }
  }
  iddb_save();
  g_next_id = id_next_free(1);

  Bluefruit.Central.setConnectCallback(server_connect_cb);
  Bluefruit.Central.setDisconnectCallback(server_disconnect_cb);
  Bluefruit.Scanner.setRxCallback(server_scan_cb);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterUuid(BLEUuid(BLEUART_UUID_SERVICE));
  Bluefruit.Scanner.setInterval(160,80);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);

  LOGF("[BOOT] role=SERVER]");
  if (g_oled_ok && !g_oled_show_list) {
    oledShowServer("scanning", 0, millis()/1000);
  }

  // heartbeat LED_PIN na serwerze (mruga, non-blocking)
  bool srv_led_state = false;
  uint32_t srv_led_next = millis();
  const uint16_t SRV_LED_ON_MS  = 20;
  const uint16_t SRV_LED_OFF_MS = 380;

  unsigned long last = 0, lastRssiLog = 0;
  bool wasConnected = false;

  while (true) {

    // ---- AUTO-SWITCH: jeśli USB zniknie → reboot jako CLIENT ----
    static uint32_t usb_unplug_since = 0;
    if (!usb_mounted()) {
      if (usb_unplug_since == 0) usb_unplug_since = millis();   // start licznika
      if (millis() - usb_unplug_since > 800) {                   // debounce ~0.8 s
        LOGF("[SERVER] USB unplugged → reboot to CLIENT");
        delay(50);
        soft_reset();                                            // NVIC_SystemReset()
      }
    } else {
      usb_unplug_since = 0; // nadal podłączone – zeruj licznik
    }

    // lokalny przycisk klawisza na PIN_022
    keyboard_button_poll();

    // zmiana strony klawiatury (LEFT/RIGHT)
    if (g_side_btn_irq) {
      noInterrupts(); 
      g_side_btn_irq = false; 
      interrupts();

      g_is_left_side = !g_is_left_side;
      LOGF("[SERVER] keyboard side changed to: %s",
           g_is_left_side ? "LEFT (A)" : "RIGHT (B)");

      if (g_oled_ok && !g_oled_show_list) {
        const char* bleStateTxt = (g_conn != BLE_CONN_HANDLE_INVALID)
                                  ? "connected" : "scanning";
        uint8_t conn_count = (g_conn != BLE_CONN_HANDLE_INVALID) ? 1 : 0;
        oledShowServer(bleStateTxt, conn_count, millis()/1000);
      }
    }

    // toggle trybu wyświetlania listy klientów
    if (g_btn024_irq) {
      noInterrupts(); g_btn024_irq = false; interrupts();
      g_oled_show_list = !g_oled_show_list;

      if (g_oled_ok) {
        if (g_oled_show_list) {
          oledShowClientsList();
        } else {
          const char* bleStateTxt = (g_conn != BLE_CONN_HANDLE_INVALID)
                                    ? "connected" : "scanning";
          uint8_t conn_count = (g_conn != BLE_CONN_HANDLE_INVALID) ? 1 : 0;
          oledShowServer(bleStateTxt, conn_count, millis()/1000);
        }
      }
    }

    // heartbeat LED na serwerze bez delay()
    uint32_t now_ms = millis();
    if (now_ms >= srv_led_next) {
      srv_led_state = !srv_led_state;
      digitalWrite(LED_PIN, srv_led_state ? HIGH : LOW);
      srv_led_next = now_ms + (srv_led_state ? SRV_LED_ON_MS : SRV_LED_OFF_MS);
    }

    server_pollDock_andAssign();
    server_bridgeUsbBle();

    // heartbeat log raz na sekundę
    if (millis() - last > 1000) {
      last = millis();
      char clients[512]; format_clients_inline(clients, sizeof(clients));

      // odświeżenie stanu baterii
      updateBatteryStatus();

      const char* bleStateTxt = (g_conn != BLE_CONN_HANDLE_INVALID)
                                ? "connected" : "scanning";
      uint8_t conn_count = (g_conn != BLE_CONN_HANDLE_INVALID) ? 1 : 0;

      LOGF("[SERVER] uptime=%lus BLE=%s | batt=%.2fV (%u%%)%s | clients: %s",
           millis()/1000, bleStateTxt,
           g_battVoltage, g_battPercent,
           g_battCharging ? " [CHG]" : "",
           clients);

      if (g_oled_ok && !g_oled_show_list) {
        oledShowServer(bleStateTxt, conn_count, millis()/1000);
      }
    }

    // periodyczny RSSI
    if (g_conn != BLE_CONN_HANDLE_INVALID && millis() - lastRssiLog > 5000) {
      lastRssiLog = millis();
      int r = getRealRssi(g_conn);
      if (r != INT16_MIN) LOGF("[SERVER] RSSI=%d dBm", r);
      else LOGF("[SERVER] RSSI/N-A");
    }

    bool isConnected = (g_conn != BLE_CONN_HANDLE_INVALID);
    if (isConnected && !wasConnected) {
      BLEConnection* conn = Bluefruit.Connection(g_conn);
      char name[32] = {0};
      if (conn) conn->getPeerName(name, sizeof(name));
      int r = getRealRssi(g_conn);
      if (r == INT16_MIN)
        LOGF("[SERVER] New BLE connection: handle=%u, name=%s, RSSI=N/A",
             g_conn, (name[0]?name:"(unknown)"));
      else
        LOGF("[SERVER] New BLE connection: handle=%u, name=%s, RSSI=%d dBm",
             g_conn, (name[0]?name:"(unknown)"), r);
      wasConnected = true;
    } else if (!isConnected && wasConnected) {
      LOGF("[SERVER] BLE disconnected");
      wasConnected = false;
    }

    yield();
  }
}
// ===== KLIENT =====

// BOOT-idle/ble i HID przez BLE

void client_startAdvertising(uint32_t id){
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.setName(("Client-"+String(id)).c_str());
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32,244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

bool client_handleDockOnce_andAssign(){
  if (!UART_DOCK.available()) return false;
  uint8_t mac[6]; Bluefruit.getAddr(mac);
  UART_DOCK.print("HELLO "); UART_DOCK.println(macToString(mac));
  String line = readLine(UART_DOCK, 2000);
  if (!line.startsWith("ASSIGN ")) return false;
  uint32_t new_id = line.substring(7).toInt();
  if (!new_id) return false;
  g_cfg.client_id = new_id;
  bool ok = cfg_save();
  UART_DOCK.println(ok ? "OK" : "ERR");
  ledBlink(2,120,120);
  return ok;
}

void run_client_idle_or_ble(){
  if (!cfg_load()){ g_cfg.magic=CFG_MAGIC; g_cfg.client_id=0; cfg_save(); }

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  bleuart.begin();

  // Heartbeat LED w trybie idle (nieblokująco)
  bool cli_idle_led_state = false;
  uint32_t cli_idle_led_next = millis();
  const uint16_t CLI_IDLE_ON_MS  = 5;
  const uint16_t CLI_IDLE_OFF_MS = 995;

  if (g_cfg.client_id==0){
    LOGF("[BOOT] role=CLIENT_IDLE (no ID)");
    if (g_oled_ok) oledShowClientIdle(millis()/1000);

    uint32_t last_idle_oled_ms = 0;

    while(true){
      uint32_t now = millis();

      // heartbeat LED
      if (now >= cli_idle_led_next) {
        cli_idle_led_state = !cli_idle_led_state;
        digitalWrite(LED_PIN, cli_idle_led_state ? HIGH : LOW);
        cli_idle_led_next = now + (cli_idle_led_state ? CLI_IDLE_ON_MS : CLI_IDLE_OFF_MS);
      }

      // odświeżanie OLED
      if (now - last_idle_oled_ms > 1000) {
        last_idle_oled_ms = now;

        updateBatteryStatus();

        if (g_oled_ok) oledShowClientIdle(now/1000);
      }

      if (usb_mounted()){
        LOGF("[CLIENT_IDLE] USB mounted -> reset");
        soft_reset();
      }

      // dock assign?
      if (client_handleDockOnce_andAssign()){
        LOGF("[CLIENT_IDLE] got ID=%lu, starting BLE NOW", g_cfg.client_id);
        client_startAdvertising(g_cfg.client_id);
        digitalWrite(LED_PIN, HIGH); // LED ON = klient BLE
        if (g_oled_ok) oledShowClientBLE(g_cfg.client_id, false, millis()/1000);
        LOGF("[CLIENT] advertising started (LED ON)");
        break;
      }

      // przycisk
      keyboard_button_poll();

      // zmiana strony klawiatury
      if (g_side_btn_irq) {
        noInterrupts(); 
        g_side_btn_irq = false; 
        interrupts();

        g_is_left_side = !g_is_left_side;
        LOGF("[CLIENT_IDLE] keyboard side changed to: %s",
             g_is_left_side ? "LEFT (A)" : "RIGHT (B)");

        updateBatteryStatus();
        if (g_oled_ok) oledShowClientIdle(now/1000);
      }

      yield();
    }
  } else {
    LOGF("[BOOT] role=CLIENT_BLE (ID=%lu)", g_cfg.client_id);
    client_startAdvertising(g_cfg.client_id);
    digitalWrite(LED_PIN,HIGH); // LED ON
    if (g_oled_ok) oledShowClientBLE(g_cfg.client_id, false, millis()/1000);
  }

  unsigned long last=0;
  while(true){
    if (usb_mounted()){
      LOGF("[CLIENT] USB mounted -> reset");
      soft_reset();
    }

    if (millis()-last>1000){
      last=millis();
      bool isConn = Bluefruit.connected();

      updateBatteryStatus();

      LOGF("[CLIENT] uptime=%lus BLE=%s ID=%lu | batt=%.2fV (%u%%)%s",
           millis()/1000,
           isConn ? "connected" : "advertising",
           g_cfg.client_id,
           g_battVoltage, g_battPercent,
           g_battCharging ? " [CHG]" : "");

      if (g_oled_ok) {
        oledShowClientBLE(g_cfg.client_id, isConn, millis()/1000);
      }
    }

    // echo przez BLE
    if (Bluefruit.connected()){
      digitalWrite(LED_PIN,HIGH);
      while (bleuart.available()){
        String msg = bleuart.readStringUntil('\n');
        bleuart.printf("CLIENT %lu: %s\n", g_cfg.client_id, msg.c_str());
      }
    }

    // przycisk litery A/B
    keyboard_button_poll();

    // zmiana strony klawiatury
    if (g_side_btn_irq) {
      noInterrupts(); 
      g_side_btn_irq = false; 
      interrupts();

      g_is_left_side = !g_is_left_side;
      LOGF("[CLIENT] keyboard side changed to: %s",
           g_is_left_side ? "LEFT (A)" : "RIGHT (B)");

      if (g_oled_ok) {
        bool isConn = Bluefruit.connected();
        oledShowClientBLE(g_cfg.client_id, isConn, millis()/1000);
      }
    }

    // dock reassign?
    if (UART_DOCK.available()){
      String l = readLine(UART_DOCK,5);
      if (l.startsWith("ASSIGN ")){
        uint32_t nid = l.substring(7).toInt();
        if (nid){
          g_cfg.client_id=nid;
          if (cfg_save()){
            UART_DOCK.println("OK");
            LOGF("[CLIENT] reassign ID=%lu", nid);
          }
        }
      }
    }

    yield();
  }
}

// ===== MAIN =====
void setup(){
  // HID najpierw, żeby host widział CDC+HID
  usb_hid.begin();

  pinMode(LED_PIN, OUTPUT); 
  digitalWrite(LED_PIN, LOW);

  UART_DOCK.setPins(UART_RX_PIN, UART_TX_PIN);
  UART_DOCK.begin(UART_BAUD);
  USB_LOG.begin(115200);

  // przycisk klawisza na PIN_022 (INPUT_PULLUP)
  pinMode(KBD_BTN_PIN, INPUT_PULLUP);

  // przycisk wyboru strony na PIN_029 (INPUT_PULLUP + IRQ)
  pinMode(SIDE_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SIDE_BTN_PIN), side_btn_isr, FALLING);

  // pin ładowania baterii (jeśli używany)
  if (BAT_CHG_PIN >= 0) {
    pinMode(BAT_CHG_PIN, INPUT_PULLUP);  // w razie potrzeby zmień na INPUT
  }

  // OLED init
  Wire.begin();
  g_oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!g_oled_ok) {
    LOGF("[OLED] not found at 0x%02X", OLED_ADDR);
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Display OK");
    // pierwszy odczyt baterii — natychmiastowe % dzięki nowej funkcji
    updateBatteryStatus();
    drawBatteryIcon();
    oledSafeDisplay();
  }

  LOGF("[IO] PIN_022 configured: KBD button (A/B) INPUT_PULLUP on server & client");
  LOGF("[IO] PIN_024 configured: server=BTN list (INPUT_PULLUP)");
  LOGF("[IO] PIN_029 configured: LEFT/RIGHT selector button (INPUT_PULLUP)");

  // BOOT faza: jeśli USB mounted → serwer, inaczej klient
  uint32_t t0=millis();
  while(!usb_mounted() && millis()-t0<6000){
    LOGF("[BOOT] waiting for USB...");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
  LOGF(usb_mounted() ? "[BOOT] USB mounted" : "[BOOT] USB timeout");

  if (usb_mounted()) {
    g_is_server = true;
    run_server();
  } else {
    g_is_server = false;
    run_client_idle_or_ble();
  }
}

void loop(){}
