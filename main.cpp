#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include <stdarg.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

extern "C" {
  #include "ble_gap.h"   // RSSI z SoftDevice
}

// ================== KONFIG ==================
#ifndef PIN_015
#define PIN_015 (15)
#endif
#define LED_PIN PIN_015

#ifndef PIN_022
#define PIN_022 (22)     // przycisk klawisza A/B (server i client)
#endif
#define KBD_BTN_PIN PIN_022

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
  #define BAT_CHG_PIN PIN_004
#endif

#define BATT_MIN_V 3.30f
#define BATT_MAX_V 4.20f

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

static bool  g_battFiltInit    = false;
static float g_battVoltageFilt = 0.0f;

// ===== CLIENT GLOBAL ID (tylko EEPROM) =====
static uint32_t g_client_id = 0;

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

// ===== EEPROM AT24C256 (I2C) =====
#ifndef EEPROM_I2C_ADDR
  #define EEPROM_I2C_ADDR 0x50   // typowy adres AT24C256 (A0..A2 = GND)
#endif

// mapowanie EEPROM:
// 0x0000: 1B  side (0=LEFT,1=RIGHT)
// 0x0001: 1B  client_flag (0xA5 = ważne ID)
// 0x0002: 4B  client_id (uint32)
// 0x0100: 4B  server_db_magic
// 0x0104: 4B  server_db_count
// 0x0108: N * entries (10B każdy): [u32 id][6B mac]

#define EEPROM_ADDR_SIDE          0x0000
#define EEPROM_ADDR_CLIENT_FLAG   0x0001
#define EEPROM_ADDR_CLIENT_ID     0x0002

#define EEPROM_ADDR_SERVER_MAGIC  0x0100
#define EEPROM_ADDR_SERVER_COUNT  0x0104
#define EEPROM_ADDR_SERVER_IDS    0x0108

static bool eeprom_write_byte(uint16_t addr, uint8_t data) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((addr >> 8) & 0xFF);
  Wire.write(addr & 0xFF);
  Wire.write(data);
  if (Wire.endTransmission() != 0) {
    LOGF("[EEPROM] write error @0x%04X", addr);
    return false;
  }
  delay(5);
  return true;
}

static bool eeprom_read_byte(uint16_t addr, uint8_t* out) {
  if (!out) return false;

  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((addr >> 8) & 0xFF);
  Wire.write(addr & 0xFF);
  if (Wire.endTransmission(false) != 0) {
    LOGF("[EEPROM] addr set error @0x%04X", addr);
    return false;
  }

  if (Wire.requestFrom(EEPROM_I2C_ADDR, (uint8_t)1) != 1) {
    LOGF("[EEPROM] read error @0x%04X", addr);
    return false;
  }
  *out = Wire.read();
  return true;
}

static bool eeprom_write_u32(uint16_t addr, uint32_t value) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((addr >> 8) & 0xFF);
  Wire.write(addr & 0xFF);

  Wire.write((uint8_t)( value        & 0xFF));
  Wire.write((uint8_t)((value >> 8 ) & 0xFF));
  Wire.write((uint8_t)((value >> 16) & 0xFF));
  Wire.write((uint8_t)((value >> 24) & 0xFF));

  if (Wire.endTransmission() != 0) {
    LOGF("[EEPROM] write_u32 error @0x%04X", addr);
    return false;
  }
  delay(5);
  return true;
}

static bool eeprom_read_u32(uint16_t addr, uint32_t* out) {
  if (!out) return false;

  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((addr >> 8) & 0xFF);
  Wire.write(addr & 0xFF);
  if (Wire.endTransmission(false) != 0) {
    LOGF("[EEPROM] addr set error (u32) @0x%04X", addr);
    return false;
  }

  if (Wire.requestFrom(EEPROM_I2C_ADDR, (uint8_t)4) != 4) {
    LOGF("[EEPROM] read_u32 error @0x%04X", addr);
    return false;
  }

  uint32_t v = 0;
  v |= (uint32_t)Wire.read();
  v |= (uint32_t)Wire.read() << 8;
  v |= (uint32_t)Wire.read() << 16;
  v |= (uint32_t)Wire.read() << 24;

  *out = v;
  return true;
}

// zapis/odczyt strony klawiatury (0 = LEFT, 1 = RIGHT)
static void side_save_to_eeprom() {
  uint8_t v = g_is_left_side ? 0x00 : 0x01;
  if (eeprom_write_byte(EEPROM_ADDR_SIDE, v)) {
    LOGF("[EEPROM] saved side = %s", g_is_left_side ? "LEFT(A)" : "RIGHT(B)");
  }
}

static void side_load_from_eeprom() {
  uint8_t v = 0xFF;
  if (!eeprom_read_byte(EEPROM_ADDR_SIDE, &v)) {
    LOGF("[EEPROM] side read failed, using default LEFT(A)");
    return;
  }

  if (v == 0x00) {
    g_is_left_side = true;
  } else if (v == 0x01) {
    g_is_left_side = false;
  } else {
    LOGF("[EEPROM] side unknown 0x%02X, keeping default LEFT(A)", v);
    return;
  }

  LOGF("[EEPROM] loaded side = %s", g_is_left_side ? "LEFT(A)" : "RIGHT(B)");
}

// ===== CLIENT ID w EEPROM =====
static void client_save_id_to_eeprom(uint32_t id) {
  if (!eeprom_write_byte(EEPROM_ADDR_CLIENT_FLAG, 0xA5)) {
    LOGF("[EEPROM] save client flag failed");
    return;
  }
  if (!eeprom_write_u32(EEPROM_ADDR_CLIENT_ID, id)) {
    LOGF("[EEPROM] save client id failed");
    return;
  }
  LOGF("[EEPROM] saved client_id=%lu", (unsigned long)id);
}

static bool client_load_id_from_eeprom(uint32_t* out_id) {
  if (!out_id) return false;

  uint8_t flag = 0;
  if (!eeprom_read_byte(EEPROM_ADDR_CLIENT_FLAG, &flag)) {
    LOGF("[EEPROM] read client flag failed");
    return false;
  }
  if (flag != 0xA5) {
    LOGF("[EEPROM] client flag invalid (0x%02X)", flag);
    return false;
  }

  uint32_t id = 0;
  if (!eeprom_read_u32(EEPROM_ADDR_CLIENT_ID, &id)) {
    LOGF("[EEPROM] read client id failed");
    return false;
  }
  if (id == 0) {
    LOGF("[EEPROM] client id=0 -> ignore");
    return false;
  }

  *out_id = id;
  LOGF("[EEPROM] loaded client_id=%lu", (unsigned long)id);
  return true;
}

// ===== ODCZYT NAPIĘCIA BATERII (nRF52 SAADC, VDDH/5) =====
float readBatteryVoltage() {
  volatile uint32_t raw_value = 0;

  NRF_SAADC->ENABLE = 1;
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;

  NRF_SAADC->CH[0].CONFIG =
    (SAADC_CH_CONFIG_GAIN_Gain1_4 << SAADC_CH_CONFIG_GAIN_Pos) |
    (SAADC_CH_CONFIG_MODE_SE      << SAADC_CH_CONFIG_MODE_Pos) |
    (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos);

  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDDHDIV5;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;

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

  double raw_double = (double)raw_value;
  double step1 = raw_double * 2.4;
  double step2 = step1 / 4095.0;
  double vddh  = 5.0 * step2;

  return (float)vddh;
}

// wykrywanie 5V na VBUS (nRF52840)
static inline bool is5VPresent() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

static inline float batteryVoltageToPercent(float v) {
  if (v <= BATT_MIN_V) return 0.f;
  if (v >= BATT_MAX_V) return 100.f;
  return (v - BATT_MIN_V) * 100.f / (BATT_MAX_V - BATT_MIN_V);
}

static void updateBatteryStatus() {
  float vRaw = readBatteryVoltage();
  uint32_t now = millis();

  static bool first_run = true;

  static bool     chg_state = false;
  static bool     chg_pending = false;
  static uint32_t chg_change_ms = 0;
  const  uint16_t CHG_DEBOUNCE_MS = 500;

  static uint32_t fast_until_ms = 0;

  static uint8_t pStable = 0;
  static bool    pInit   = false;
  const  float   H = 1.2f;

  if (first_run) {
    float vClamped = (vRaw > BATT_MAX_V) ? BATT_MAX_V : vRaw;
    g_battVoltageFilt = vRaw;
    g_battVoltage     = vRaw;
    g_battFiltInit    = true;
    pStable           = (uint8_t)roundf(batteryVoltageToPercent(vClamped));
    pInit             = true;
    g_battPercent     = pStable;
    first_run         = false;
    fast_until_ms     = now + 1500;
  }

  bool pinCharging = false;
  if (BAT_CHG_PIN >= 0) {
    pinCharging = (digitalRead(BAT_CHG_PIN) == LOW);
  }

  bool vbusPresent = is5VPresent();
  bool vSuggestsCharging = false;

  bool chg_raw = vbusPresent || pinCharging || vSuggestsCharging;

  if (chg_raw != chg_pending) {
    chg_pending = chg_raw;
    chg_change_ms = now;
  }
  bool prev_chg_state = chg_state;
  if ((now - chg_change_ms) >= CHG_DEBOUNCE_MS) chg_state = chg_pending;
  g_battCharging = chg_state;

  if (chg_state != prev_chg_state) {
    g_battVoltageFilt = vRaw;
    g_battVoltage     = vRaw;
    uint8_t pctNow    = (uint8_t)roundf(batteryVoltageToPercent((vRaw > BATT_MAX_V) ? BATT_MAX_V : vRaw));
    pStable           = pctNow;
    g_battPercent     = pStable;
    fast_until_ms     = now + 2000;
  }

  float dv    = fabsf(vRaw - g_battVoltageFilt);
  bool  big   = (dv > 0.06f);
  float alpha = ((now < fast_until_ms) || big) ? 0.80f : 0.12f;
  g_battVoltageFilt = g_battVoltageFilt + alpha * (vRaw - g_battVoltageFilt);
  g_battVoltage     = g_battVoltageFilt;

  float  vForPct = (g_battVoltageFilt > BATT_MAX_V) ? BATT_MAX_V : g_battVoltageFilt;
  float  pFloat  = batteryVoltageToPercent(vForPct);

  if (!pInit) { pStable = (uint8_t)roundf(pFloat); pInit = true; }

  if (now < fast_until_ms) {
    pStable = (uint8_t)roundf(pFloat);
  } else {
    float upTh   = (float)pStable + H;
    float downTh = (float)pStable - H;
    if (pFloat >= upTh  && pStable < 100) pStable++;
    else if (pFloat <= downTh && pStable > 0) pStable--;
  }

  g_battPercent = pStable;
}

// ===== OLED helpers =====
static inline void oledSafeDisplay() { if (g_oled_ok) display.display(); }

static void drawBatteryIcon() {
  if (!g_oled_ok) return;

  const int iconW = 16;
  const int iconH = 8;
  int x = OLED_W - iconW - 1;
  int y = 0;

  display.drawRect(x, y, iconW-2, iconH, SSD1306_WHITE);
  display.drawRect(x + iconW - 3, y + 2, 2, iconH-4, SSD1306_WHITE);

  int innerW = iconW - 4;
  int innerH = iconH - 2;
  int fillW  = (int)((innerW * g_battPercent) / 100);

  if (fillW < 0) fillW = 0;
  if (fillW > innerW) fillW = innerW;

  if (!g_battCharging && fillW > 0) {
    display.fillRect(x + 2, y + 1, fillW, innerH, SSD1306_WHITE);
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x - 28, y);

  if (g_battCharging) {
    display.print("CHG");
  } else {
    display.print(g_battPercent);
    display.print("%");
  }

  if (g_battCharging) {
    int bx = x + 3;
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

// Parser nazwy z ADV
bool adv_get_name(ble_gap_evt_adv_report_t* rpt, char* out, size_t outlen) {
  if (!rpt || !out || outlen == 0) return false;
  out[0] = 0;
  const uint8_t* p = rpt->data.p_data;
  uint8_t len = rpt->data.len;
  while (len > 2) {
    uint8_t ad_len  = p[0];
    if (ad_len == 0 || ad_len + 1 > len) break;
    uint8_t ad_type = p[1];
    if (ad_type == 0x09 || ad_type == 0x08) {
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

// ================== SERWER: baza zajętych ID w EEPROM ==================
struct IdEntry { uint32_t id; uint8_t mac[6]; };
static const uint32_t MAX_IDS = 128;
static IdEntry g_ids[MAX_IDS];
static uint32_t g_ids_count = 0;

static const uint32_t SERVER_DB_MAGIC = 0x53494442; // 'SIDB'
static const uint16_t SERVER_ENTRY_SIZE = 10;        // 4B id + 6B mac

static bool iddb_load() {
  uint32_t magic = 0;
  if (!eeprom_read_u32(EEPROM_ADDR_SERVER_MAGIC, &magic)) {
    g_ids_count = 0;
    return false;
  }
  if (magic != SERVER_DB_MAGIC) {
    g_ids_count = 0;
    return false;
  }

  uint32_t count = 0;
  if (!eeprom_read_u32(EEPROM_ADDR_SERVER_COUNT, &count)) {
    g_ids_count = 0;
    return false;
  }
  if (count > MAX_IDS) count = MAX_IDS;
  g_ids_count = count;

  for (uint32_t i=0; i<g_ids_count; ++i) {
    uint16_t addr = EEPROM_ADDR_SERVER_IDS + i*SERVER_ENTRY_SIZE;
    uint32_t id;
    if (!eeprom_read_u32(addr, &id)) {
      g_ids_count = i;
      break;
    }
    g_ids[i].id = id;
    for (int j=0; j<6; ++j) {
      uint8_t b;
      if (!eeprom_read_byte(addr+4+j, &b)) {
        g_ids_count = i;
        break;
      }
      g_ids[i].mac[j] = b;
    }
  }
  LOGF("[SERVER] iddb_load: count=%lu", (unsigned long)g_ids_count);
  return true;
}

static void iddb_save() {
  eeprom_write_u32(EEPROM_ADDR_SERVER_MAGIC, SERVER_DB_MAGIC);
  eeprom_write_u32(EEPROM_ADDR_SERVER_COUNT, g_ids_count);

  for (uint32_t i=0; i<g_ids_count; ++i) {
    uint16_t addr = EEPROM_ADDR_SERVER_IDS + i*SERVER_ENTRY_SIZE;
    eeprom_write_u32(addr, g_ids[i].id);
    for (int j=0; j<6; ++j) {
      eeprom_write_byte(addr+4+j, g_ids[i].mac[j]);
    }
  }
  LOGF("[SERVER] iddb_save: count=%lu", (unsigned long)g_ids_count);
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
    usb_hid.keyboardReport(0, 0, keys);
  }

  delay(10);

  {
    uint32_t t0 = millis();
    while (!usb_hid.ready() && (millis() - t0) < 30) { yield(); }
    uint8_t empty[6] = {0,0,0,0,0,0};
    usb_hid.keyboardReport(0, 0, empty);
  }
}

static void hid_key_down(uint8_t keycode) {
  if (!TinyUSBDevice.mounted()) return;

  if (TinyUSBDevice.suspended()) {
    TinyUSBDevice.remoteWakeup();
    delay(2);
  }

  uint32_t t0 = millis();
  while (!usb_hid.ready() && (millis() - t0) < 30) { yield(); }

  uint8_t keys[6] = { keycode, 0,0,0,0,0 };
  usb_hid.keyboardReport(0, 0, keys);
}

static void hid_key_up_all() {
  if (!TinyUSBDevice.mounted()) return;

  uint32_t t0 = millis();
  while (!usb_hid.ready() && (millis() - t0) < 30) { yield(); }

  uint8_t empty[6] = {0,0,0,0,0,0};
  usb_hid.keyboardReport(0, 0, empty);
}

// ================== BLE ==================
BLEUart       bleuart;
BLEClientUart clientUart;
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
  if (now - g_side_btn_last_us < 150000U) return;
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
      else            assigned = id_next_free(1);

      UART_DOCK.print("ASSIGN ");
      UART_DOCK.println(assigned);
      LOGF("[SERVER] assign %lu to %s (wait ack)",
           (unsigned long)assigned, macStr.c_str());
      retries = 0;
      tAssign = millis();
      state = SENT_ASSIGN;
    }
  } else {
    String ack = readLine(UART_DOCK, 10);
    if (ack == "OK") {
      LOGF("[SERVER] ack OK for ID %lu", (unsigned long)assigned);
      id_set(assigned, last_mac);
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
static const bool KBD_BTN_ACTIVE_LOW    = true;
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
  bool raw_now = digitalRead(KBD_BTN_PIN);

  if (raw_now != kbd_btn_raw_last) {
    kbd_btn_raw_last      = raw_now;
    kbd_btn_raw_change_ms = now;
  }

  if (raw_now != kbd_btn_filtered_state) {
    if ((now - kbd_btn_raw_change_ms) >= KBD_BTN_STABLE_MS) {
      kbd_btn_filtered_state = raw_now;
      bool pressed_now = kbd_hw_is_pressed(kbd_btn_filtered_state);

      uint8_t keycode   = g_is_left_side ? HID_KEY_A : HID_KEY_B;
      char    keyLetter = g_is_left_side ? 'A'        : 'B';

      if (pressed_now && !kbd_btn_pressed_logical) {
        kbd_btn_pressed_logical = true;
        LOGF("[%s] KBD BTN DOWN (%c)",
             g_is_server ? "SERVER" : "CLIENT",
             keyLetter);

        if (g_is_server) {
          hid_key_down(keycode);
        } else {
          char line[16];
          snprintf(line, sizeof(line), "KEY %c DOWN\n", keyLetter);
          client_send_line(line);
        }
      }
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
  if (g_conn!=BLE_CONN_HANDLE_INVALID && USB_LOG.available()){
    while(USB_LOG.available()){
      char c=USB_LOG.read();
      clientUart.write(&c,1);
      if(c=='\n') break;
    }
  }

  if (g_conn!=BLE_CONN_HANDLE_INVALID && clientUart.available()){
    String s = clientUart.readStringUntil('\n');

    if (s.startsWith("KEY ")) {
      if (s.length() < 8) {
        LOGF("[SERVER] malformed KEY cmd: %s", s.c_str());
      } else {
        char which = s.charAt(4);
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

  pinMode(BTN_LIST_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_LIST_PIN), btn024_isr, FALLING);

  // wczytaj bazę ID z EEPROM
  iddb_load();

  // posprzątaj duplikaty w RAM
  for (uint32_t i=0; i<g_ids_count; ++i){
    for (uint32_t j=i+1; j<g_ids_count; ){
      if (memcmp(g_ids[i].mac, g_ids[j].mac, 6)==0 ||
          g_ids[i].id==g_ids[j].id){
        remove_idx(j);
      } else ++j;
    }
  }
  iddb_save();

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

  bool srv_led_state = false;
  uint32_t srv_led_next = millis();
  const uint16_t SRV_LED_ON_MS  = 20;
  const uint16_t SRV_LED_OFF_MS = 380;

  unsigned long last = 0, lastRssiLog = 0;
  bool wasConnected = false;

  while (true) {

    static uint32_t usb_unplug_since = 0;
    if (!usb_mounted()) {
      if (usb_unplug_since == 0) usb_unplug_since = millis();
      if (millis() - usb_unplug_since > 800) {
        LOGF("[SERVER] USB unplugged → reboot to CLIENT");
        delay(50);
        soft_reset();
      }
    } else {
      usb_unplug_since = 0;
    }

    keyboard_button_poll();

    if (g_side_btn_irq) {
      noInterrupts(); 
      g_side_btn_irq = false; 
      interrupts();

      g_is_left_side = !g_is_left_side;
      LOGF("[SERVER] keyboard side changed to: %s",
           g_is_left_side ? "LEFT (A)" : "RIGHT (B)");

      side_save_to_eeprom();

      if (g_oled_ok && !g_oled_show_list) {
        const char* bleStateTxt = (g_conn != BLE_CONN_HANDLE_INVALID)
                                  ? "connected" : "scanning";
        uint8_t conn_count = (g_conn != BLE_CONN_HANDLE_INVALID) ? 1 : 0;
        oledShowServer(bleStateTxt, conn_count, millis()/1000);
      }
    }

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

    uint32_t now_ms = millis();
    if (now_ms >= srv_led_next) {
      srv_led_state = !srv_led_state;
      digitalWrite(LED_PIN, srv_led_state ? HIGH : LOW);
      srv_led_next = now_ms + (srv_led_state ? SRV_LED_ON_MS : SRV_LED_OFF_MS);
    }

    server_pollDock_andAssign();
    server_bridgeUsbBle();

    if (millis() - last > 1000) {
      last = millis();
      char clients[512]; format_clients_inline(clients, sizeof(clients));

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
  g_client_id = new_id;
  client_save_id_to_eeprom(new_id);
  UART_DOCK.println("OK");
  ledBlink(2,120,120);
  LOGF("[CLIENT_IDLE] got ID=%lu from dock", (unsigned long)new_id);
  return true;
}

void run_client_idle_or_ble(){
  // spróbuj wczytać ID z EEPROM
  if (!client_load_id_from_eeprom(&g_client_id)) {
    g_client_id = 0;
  }

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  bleuart.begin();

  bool cli_idle_led_state = false;
  uint32_t cli_idle_led_next = millis();
  const uint16_t CLI_IDLE_ON_MS  = 5;
  const uint16_t CLI_IDLE_OFF_MS = 995;

  if (g_client_id==0){
    LOGF("[BOOT] role=CLIENT_IDLE (no ID)");
    if (g_oled_ok) oledShowClientIdle(millis()/1000);

    uint32_t last_idle_oled_ms = 0;

    while(true){
      uint32_t now = millis();

      if (now >= cli_idle_led_next) {
        cli_idle_led_state = !cli_idle_led_state;
        digitalWrite(LED_PIN, cli_idle_led_state ? HIGH : LOW);
        cli_idle_led_next = now + (cli_idle_led_state ? CLI_IDLE_ON_MS : CLI_IDLE_OFF_MS);
      }

      if (now - last_idle_oled_ms > 1000) {
        last_idle_oled_ms = now;

        updateBatteryStatus();

        if (g_oled_ok) oledShowClientIdle(now/1000);
      }

      if (usb_mounted()){
        LOGF("[CLIENT_IDLE] USB mounted -> reset");
        soft_reset();
      }

      if (client_handleDockOnce_andAssign()){
        LOGF("[CLIENT_IDLE] starting BLE NOW, ID=%lu", (unsigned long)g_client_id);
        client_startAdvertising(g_client_id);
        digitalWrite(LED_PIN, HIGH);
        if (g_oled_ok) oledShowClientBLE(g_client_id, false, millis()/1000);
        LOGF("[CLIENT] advertising started (LED ON)");
        break;
      }

      keyboard_button_poll();

      if (g_side_btn_irq) {
        noInterrupts(); 
        g_side_btn_irq = false; 
        interrupts();

        g_is_left_side = !g_is_left_side;
        LOGF("[CLIENT_IDLE] keyboard side changed to: %s",
             g_is_left_side ? "LEFT (A)" : "RIGHT (B)");

        side_save_to_eeprom();

        updateBatteryStatus();
        if (g_oled_ok) oledShowClientIdle(now/1000);
      }

      yield();
    }
  } else {
    LOGF("[BOOT] role=CLIENT_BLE (ID=%lu)", (unsigned long)g_client_id);
    client_startAdvertising(g_client_id);
    digitalWrite(LED_PIN,HIGH);
    if (g_oled_ok) oledShowClientBLE(g_client_id, false, millis()/1000);
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
           (unsigned long)g_client_id,
           g_battVoltage, g_battPercent,
           g_battCharging ? " [CHG]" : "");

      if (g_oled_ok) {
        oledShowClientBLE(g_client_id, isConn, millis()/1000);
      }
    }

    if (Bluefruit.connected()){
      digitalWrite(LED_PIN,HIGH);
      while (bleuart.available()){
        String msg = bleuart.readStringUntil('\n');
        bleuart.printf("CLIENT %lu: %s\n", (unsigned long)g_client_id, msg.c_str());
      }
    }

    keyboard_button_poll();

    if (g_side_btn_irq) {
      noInterrupts(); 
      g_side_btn_irq = false; 
      interrupts();

      g_is_left_side = !g_is_left_side;
      LOGF("[CLIENT] keyboard side changed to: %s",
           g_is_left_side ? "LEFT (A)" : "RIGHT (B)");

      side_save_to_eeprom();

      if (g_oled_ok) {
        bool isConn = Bluefruit.connected();
        oledShowClientBLE(g_client_id, isConn, millis()/1000);
      }
    }

    // reassign ID przez UART (dock)
    if (UART_DOCK.available()){
      String l = readLine(UART_DOCK,5);
      if (l.startsWith("ASSIGN ")){
        uint32_t nid = l.substring(7).toInt();
        if (nid){
          g_client_id=nid;
          client_save_id_to_eeprom(nid);
          UART_DOCK.println("OK");
          LOGF("[CLIENT] reassign ID=%lu", (unsigned long)nid);
        }
      }
    }

    yield();
  }
}

// ===== MAIN =====
void setup(){
  usb_hid.begin();

  pinMode(LED_PIN, OUTPUT); 
  digitalWrite(LED_PIN, LOW);

  UART_DOCK.setPins(UART_RX_PIN, UART_TX_PIN);
  UART_DOCK.begin(UART_BAUD);
  USB_LOG.begin(115200);

  pinMode(KBD_BTN_PIN, INPUT_PULLUP);

  pinMode(SIDE_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SIDE_BTN_PIN), side_btn_isr, FALLING);

  if (BAT_CHG_PIN >= 0) {
    pinMode(BAT_CHG_PIN, INPUT_PULLUP);
  }

  Wire.begin();

  // wczytaj stronę klawiatury z EEPROM – wspólne dla serwera/klienta
  side_load_from_eeprom();

  g_oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!g_oled_ok) {
    LOGF("[OLED] not found at 0x%02X", OLED_ADDR);
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Display OK");
    updateBatteryStatus();
    drawBatteryIcon();
    oledSafeDisplay();
  }

  LOGF("[IO] PIN_022: KBD button (A/B) INPUT_PULLUP");
  LOGF("[IO] PIN_024: server BTN list INPUT_PULLUP");
  LOGF("[IO] PIN_029: LEFT/RIGHT selector INPUT_PULLUP");

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
