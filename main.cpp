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
#define PIN_022 (22)     // teraz: SWITCH (client) / LED (server)
#endif
#define IO_PIN_022 PIN_022

// — Przycisk listy na PIN_024 —
#ifndef PIN_024
#define PIN_024 (24)             // przycisk trybu listy (serwer) / KEY A (klient)
#endif
#define BTN_LIST_PIN PIN_024

#define USB_LOG   Serial
#define UART_DOCK Serial1
#define UART_BAUD 115200

// <<< USTAW swoje piny UART (numery portów nRF52, np. P0.08 => 8) >>>
#define UART_RX_PIN  8
#define UART_TX_PIN  6
// ===========================================

// ===== OLED =====
#ifndef OLED_ADDR
  #define OLED_ADDR 0x3C
#endif
#ifndef OLED_W
  #define OLED_W 128
#endif
#ifndef OLED_H
  #define OLED_H 32   // jeśli masz 128x64, zmień na 64
#endif

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
static bool g_oled_ok = false;

// —— Tryb listy klientów (serwer) ——
static bool g_oled_show_list = false;

static inline void oledSafeDisplay() { if (g_oled_ok) display.display(); }

static void oledBoot(const char* msg) {
  if (!g_oled_ok) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("BOOT...");
  display.println(msg);
  oledSafeDisplay();
}

// —— Ekran główny SERWERA (bez ID/MAC) — BLE + up + liczba połączeń ——
static void oledShowServer(const char* bleState, uint8_t conn_count, uint32_t uptime_s) {
  if (!g_oled_ok) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("SERVER");
  display.print("BLE: ");
  display.print(bleState);
  display.print(" ");
  display.println(conn_count);
  display.print("up: ");
  display.print(uptime_s);
  display.println("s");
  oledSafeDisplay();
}

static void oledShowClientIdle(uint32_t uptime_s) {
  if (!g_oled_ok) return;
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("CLIENT (IDLE)");
  display.println("Dock to assign ID");
  display.print("up: "); display.print(uptime_s); display.println("s");
  oledSafeDisplay();
}

static void oledShowClientBLE(uint32_t id, bool connected, uint32_t uptime_s) {
  if (!g_oled_ok) return;
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("CLIENT (BLE)");
  display.print("ID: "); display.println((unsigned long)id);
  display.println(connected ? "State: connected" : "State: advertising");
  display.print("up: "); display.print(uptime_s); display.println("s");
  oledSafeDisplay();
}

// ===== FLASH CONFIG KLIENTA =====
struct ClientConfig { uint32_t magic; uint32_t client_id; };
static const uint32_t CFG_MAGIC = 0x52494E4B;          // 'RINK'
static const char*    CFG_PATH  = "/client_cfg.bin";
ClientConfig g_cfg;

// ===== Utils =====
static inline bool usb_mounted() { return TinyUSBDevice.mounted(); }
static inline void soft_reset()  { NVIC_SystemReset(); }

// (pozostawiamy – używane rzadko; nie wpływa na pętle główne)
static inline void ledBlink(uint8_t n, uint16_t on=60, uint16_t off=120){
  for(uint8_t i=0;i<n;i++){ digitalWrite(LED_PIN,HIGH); delay(on); digitalWrite(LED_PIN,LOW); delay(off); }
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
  sprintf(b,"%02X:%02X:%02X:%02X:%02X:%02X",mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
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

// Parser nazwy z ADV (dla 1.6.x)
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
static uint32_t g_next_id = 1;        // kandydat startowy (przesuwany po ACK)

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

// ——— deduplikacja i indeksy ———
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

// ——— główna funkcja zapisu z deduplikacją ———
static void id_set(uint32_t id, const uint8_t mac[6]) {
  int i_mac = idx_by_mac(mac);
  int i_id  = idx_by_id(id);

  if (i_mac >= 0 && i_id >= 0) {
    if (i_mac != i_id) {            // scal duplikaty
      memcpy(g_ids[i_id].mac, mac, 6);
      remove_idx(i_mac);
    }
  } else if (i_id >= 0) {           // istnieje ID → zaktualizuj MAC
    memcpy(g_ids[i_id].mac, mac, 6);
  } else if (i_mac >= 0) {          // istnieje MAC → zaktualizuj ID
    g_ids[i_mac].id = id;
  } else if (g_ids_count < MAX_IDS) { // nowy wpis
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

// 🔹 formatter inline do logów „clients: ID=.. MAC=.., ...”
static void format_clients_inline(char* out, size_t outlen) {
  if (!out || outlen == 0) return;
  out[0] = 0;
  if (g_ids_count == 0) { snprintf(out, outlen, "(none)"); return; }
  size_t used = 0;
  for (uint32_t i=0; i<g_ids_count; ++i) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            g_ids[i].mac[5], g_ids[i].mac[4], g_ids[i].mac[3],
            g_ids[i].mac[2], g_ids[i].mac[1], g_ids[i].mac[0]);
    int n = snprintf(out + used, (used < outlen ? outlen - used : 0),
                     "%sID=%lu MAC=%s",
                     (i==0 ? "" : ", "), (unsigned long)g_ids[i].id, macStr);
    if (n < 0) break;
    used += (size_t)n;
    if (used >= outlen) { out[outlen-1]=0; break; }
  }
}

// ====== OLED: lista klientów (ID + MAC, bez '=') ======
static void oledShowClientsList() {
  if (!g_oled_ok) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("CLIENTS");

  // 8 px / wiersz
  uint8_t maxRows = OLED_H / 8;
  if (maxRows == 0) maxRows = 1;
  uint8_t rowsAvail = (maxRows > 1) ? (maxRows - 1) : 0;

  uint8_t shown = 0;
  for (uint32_t i = 0; i < g_ids_count && shown < rowsAvail; ++i) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            g_ids[i].mac[5], g_ids[i].mac[4], g_ids[i].mac[3],
            g_ids[i].mac[2], g_ids[i].mac[1], g_ids[i].mac[0]);

    // Bez '=': "ID<nr>  <MAC>"
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

  oledSafeDisplay();
}

// ================== USB HID KEYBOARD (SERWER) ==================
// używamy makra BEZ argumentów
uint8_t const desc_hid_report[] = { TUD_HID_REPORT_DESC_KEYBOARD() };

Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report),
                          HID_ITF_PROTOCOL_KEYBOARD, 10, false);

// --- HID tap queue (server) ---
static volatile bool g_hid_a_pending = false;
static uint32_t g_hid_last_tap_ms = 0;

// bezpieczny „tap” klawisza
static inline void hid_tap_key(uint8_t key, uint16_t down_ms = 15, uint16_t post_ms = 2) {
  if (!TinyUSBDevice.mounted()) return;

  if (TinyUSBDevice.suspended()) {
    TinyUSBDevice.remoteWakeup();
    delay(5);
  }

  uint32_t t0 = millis();
  while (!usb_hid.ready() && (millis() - t0) < 50) { yield(); }

  uint8_t keys[6] = { key, 0,0,0,0,0 };
  usb_hid.keyboardReport(0, 0, keys);   // PRESS
  delay(down_ms);                       // >= 1 okresu pollingu (10 ms)

  uint8_t empty[6] = {0};
  usb_hid.keyboardReport(0, 0, empty);  // RELEASE
  delay(post_ms);
}

// dedykowane 'a'
static inline void hid_send_key_a_safe() {
  hid_tap_key(HID_KEY_A, 15, 2);
}

// ================== BLE ==================
BLEUart       bleuart;         // peripheral (klient BLE)
BLEClientUart clientUart;      // central (serwer)
uint16_t g_conn = BLE_CONN_HANDLE_INVALID;
static uint32_t g_active_id = 0;
static uint8_t  g_active_mac[6] = {0};

// ===== SWITCH (CLIENT) — ISR =====
volatile bool     g_sw_irq_flag = false;
volatile uint32_t g_sw_count    = 0;
volatile uint32_t g_sw_last_us  = 0;  // debounce w ISR (micros)
static void sw_isr() {
  uint32_t now = micros();
  if (now - g_sw_last_us < 150000U) return; // ~150 ms debounce
  g_sw_last_us = now;
  g_sw_irq_flag = true;
  g_sw_count++;
}

// ===== Przycisk listy (SERVER) — ISR =====
volatile bool     g_btn024_irq     = false;
volatile uint32_t g_btn024_last_us = 0;
static void btn024_isr() {
  uint32_t now = micros();
  if (now - g_btn024_last_us < 150000U) return; // ~150 ms debounce
  g_btn024_last_us = now;
  g_btn024_irq = true;
}

// ===== KLIENT: przycisk na PIN_024 → wysyłka KEY A — ISR =====
volatile bool     g_btn024_cli_irq     = false;
volatile uint32_t g_btn024_cli_last_us = 0;
static void btn024_client_isr() {
  uint32_t now = micros();
  if (now - g_btn024_cli_last_us < 200000U) return; // 200 ms debounce (stabilniej)
  g_btn024_cli_last_us = now;
  g_btn024_cli_irq = true;
}

// ===== SERVER CALLBACKS =====
void server_connect_cb(uint16_t conn_handle) {
  g_conn = conn_handle;
  ledBlink(2,60,80); // rzadkie

  bool ok = false;
  for (int i=0; i<3 && !ok; ++i) { ok = clientUart.discover(conn_handle); if (!ok) delay(150); }
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
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
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
  ledBlink(1,200,200); // rzadkie

  if (g_oled_ok && !g_oled_show_list) {
    oledShowServer("scanning", 0, millis()/1000);
  }
}

// ===== SERVER SCAN CB (UUID + nazwa fallback) =====
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
    if (millis() - lastWHO > 800) { lastWHO = millis(); UART_DOCK.println("WHO"); }
    String line = readLine(UART_DOCK, 10);
    if (line.startsWith("HELLO ")) {
      memset(last_mac, 0, sizeof(last_mac));
      String macStr = line.substring(6);
      int b[6]={0};
      if (sscanf(macStr.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
                 &b[5],&b[4],&b[3],&b[2],&b[1],&b[0])==6) {
        for (int i=0;i<6;i++) last_mac[i]=(uint8_t)b[i];
      }
      int known = idx_by_mac(last_mac);
      if (known >= 0) assigned = g_ids[known].id;
      else            assigned = id_next_free(g_next_id);

      UART_DOCK.print("ASSIGN "); UART_DOCK.println(assigned);
      LOGF("[SERVER] assign %lu to %s (wait ack)", (unsigned long)assigned, macStr.c_str());
      retries = 0; tAssign = millis(); state = SENT_ASSIGN;
    }
  } else { // SENT_ASSIGN
    String ack = readLine(UART_DOCK, 10);
    if (ack == "OK") {
      LOGF("[SERVER] ack OK for ID %lu", (unsigned long)assigned);
      id_set(assigned, last_mac);
      if (assigned >= g_next_id) g_next_id = assigned + 1;
      UART_DOCK.println("GO");
      ledBlink(2,120,120); // sporadyczne
      lastWHO = millis() + 2000;
      state = WAIT_HELLO;
      return;
    }
    if (millis() - tAssign > 600) {
      if (++retries <= 3) {
        tAssign = millis();
        UART_DOCK.print("ASSIGN "); UART_DOCK.println(assigned);
        LOGF("[SERVER] retry %u for ID %lu", retries, (unsigned long)assigned);
      } else {
        LOGF("[SERVER] no-ack for ID %lu, giving up", (unsigned long)assigned);
        state = WAIT_HELLO;
      }
    }
  }
}

void server_bridgeUsbBle(){
  if (g_conn!=BLE_CONN_HANDLE_INVALID && USB_LOG.available()){
    while(USB_LOG.available()){ char c=USB_LOG.read(); clientUart.write(&c,1); if(c=='\n') break; }
  }
  if (g_conn!=BLE_CONN_HANDLE_INVALID && clientUart.available()){
    String s = clientUart.readStringUntil('\n');
    if (s.startsWith("SW ")) {
      int state = digitalRead(IO_PIN_022);
      digitalWrite(IO_PIN_022, !state); // TOGGLE LED na serwerze
      LOGF("[SERVER] got SWITCH evt from client -> LED on PIN_022 = %s",
           (!state) ? "ON" : "OFF");

      if (g_oled_ok && !g_oled_show_list) {
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(80, (OLED_H >= 64) ? 40 : 24);
        display.print("LED: ");
        display.println((!state) ? "ON" : "OFF");
        oledSafeDisplay();
      }
    }
    else if (s.startsWith("KEY A")) {
      LOGF("[SERVER] got KEY A");
      // ustaw tylko flagę; HID zrobimy w pętli głównej
      g_hid_a_pending = true;

      if (g_oled_ok && !g_oled_show_list) {
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(80, (OLED_H >= 64) ? 40 : 24);
        display.print("HID: a");
        oledSafeDisplay();
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

  // LED na PIN_022 (serwer)
  pinMode(IO_PIN_022, OUTPUT);
  digitalWrite(IO_PIN_022, LOW);           // LED OFF

  // Przycisk listy
  pinMode(BTN_LIST_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_LIST_PIN), btn024_isr, FALLING);

  // wczytaj bazę ID, deduplikuj i ustaw startowy kandydat
  iddb_load();
  for (uint32_t i=0; i<g_ids_count; ++i){
    for (uint32_t j=i+1; j<g_ids_count; ){
      if (memcmp(g_ids[i].mac, g_ids[j].mac, 6)==0 || g_ids[i].id==g_ids[j].id){
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

  // [NB] Nieblokujący heartbeat LED_PIN
  bool srv_led_state = false;
  uint32_t srv_led_next = millis();
  const uint16_t SRV_LED_ON_MS  = 20;
  const uint16_t SRV_LED_OFF_MS = 380;

  unsigned long last = 0, lastRssiLog = 0;
  bool wasConnected = false;

  while (true) {
    // —— Toggle trybu wyświetlania po naciśnięciu BTN PIN_024 ——
    if (g_btn024_irq) {
      noInterrupts(); g_btn024_irq = false; interrupts();
      g_oled_show_list = !g_oled_show_list;

      if (g_oled_ok) {
        if (g_oled_show_list) {
          oledShowClientsList();
        } else {
          const char* bleStateTxt = (g_conn != BLE_CONN_HANDLE_INVALID) ? "connected" : "scanning";
          uint8_t conn_count = (g_conn != BLE_CONN_HANDLE_INVALID) ? 1 : 0;
          oledShowServer(bleStateTxt, conn_count, millis()/1000);
        }
      }
    }

    // Heartbeat LED bez delay
    uint32_t now = millis();
    if (now >= srv_led_next) {
      srv_led_state = !srv_led_state;
      digitalWrite(LED_PIN, srv_led_state ? HIGH : LOW);
      srv_led_next = now + (srv_led_state ? SRV_LED_ON_MS : SRV_LED_OFF_MS);
    }

    server_pollDock_andAssign();
    server_bridgeUsbBle();

    // --- HID tap z lockoutem (po stronie serwera) ---
    if (g_hid_a_pending) {
      noInterrupts(); g_hid_a_pending = false; interrupts();

      uint32_t now_ms = millis();
      if (now_ms - g_hid_last_tap_ms >= 120) { // min. odstęp 120 ms
        hid_send_key_a_safe();
        g_hid_last_tap_ms = now_ms;
      } else {
        // zbyt szybko – pomiń duplikat
      }
    }

    // heartbeat co sekundę + inline lista klientów do logów
    if (millis() - last > 1000) {
      last = millis();
      char clients[512]; format_clients_inline(clients, sizeof(clients));

      const char* bleStateTxt = (g_conn != BLE_CONN_HANDLE_INVALID) ? "connected" : "scanning";
      uint8_t conn_count = (g_conn != BLE_CONN_HANDLE_INVALID) ? 1 : 0;

      LOGF("[SERVER] uptime=%lus BLE=%s | clients: %s",
          millis()/1000, bleStateTxt, clients);

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
      if (r == INT16_MIN) LOGF("[SERVER] New BLE connection: handle=%u, name=%s, RSSI=N/A",
                               g_conn, (name[0]?name:"(unknown)"));
      else LOGF("[SERVER] New BLE connection: handle=%u, name=%s, RSSI=%d dBm",
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
  g_cfg.client_id = new_id;
  bool ok = cfg_save();
  UART_DOCK.println(ok ? "OK" : "ERR");
  ledBlink(2,120,120); // sporadyczne
  return ok;
}

void run_client_idle_or_ble(){
  if (!cfg_load()){ g_cfg.magic=CFG_MAGIC; g_cfg.client_id=0; cfg_save(); }
  Bluefruit.begin(); Bluefruit.setTxPower(4); bleuart.begin();

  // SWITCH na PIN_022 (klient) – przycisk do GND
  pinMode(IO_PIN_022, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IO_PIN_022), sw_isr, FALLING);

  // Drugi przycisk klienta na PIN_024 → wysyłka KEY A
  pinMode(PIN_024, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_024), btn024_client_isr, FALLING);

  // [NB] Nieblokujący heartbeat w trybie IDLE
  bool cli_idle_led_state = false;
  uint32_t cli_idle_led_next = millis();
  const uint16_t CLI_IDLE_ON_MS  = 5;
  const uint16_t CLI_IDLE_OFF_MS = 995;

  if (g_cfg.client_id==0){
    LOGF("[BOOT] role=CLIENT_IDLE (no ID)");
    if (g_oled_ok) oledShowClientIdle(millis()/1000);

    while(true){
      uint32_t now = millis();
      if (now >= cli_idle_led_next) {
        cli_idle_led_state = !cli_idle_led_state;
        digitalWrite(LED_PIN, cli_idle_led_state ? HIGH : LOW);
        cli_idle_led_next = now + (cli_idle_led_state ? CLI_IDLE_ON_MS : CLI_IDLE_OFF_MS);
      }

      if (usb_mounted()){ LOGF("[CLIENT_IDLE] USB mounted -> reset"); soft_reset(); }

      if (client_handleDockOnce_andAssign()){
        LOGF("[CLIENT_IDLE] got ID=%lu, starting BLE NOW", g_cfg.client_id);
        client_startAdvertising(g_cfg.client_id);
        digitalWrite(LED_PIN, HIGH); // LED ON = klient BLE
        if (g_oled_ok) oledShowClientBLE(g_cfg.client_id, false, millis()/1000);
        LOGF("[CLIENT] advertising started (LED ON)");
        break;
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
    if (usb_mounted()){ LOGF("[CLIENT] USB mounted -> reset"); soft_reset(); }

    if (millis()-last>1000){
      last=millis();
      bool isConn = Bluefruit.connected();
      LOGF("[CLIENT] uptime=%lus BLE=%s ID=%lu",
           millis()/1000,
           isConn ? "connected" : "advertising",
           g_cfg.client_id);

      if (g_oled_ok) {
        oledShowClientBLE(g_cfg.client_id, isConn, millis()/1000);
      }
    }

    if (Bluefruit.connected()){
      digitalWrite(LED_PIN,HIGH);
      while (bleuart.available()){
        String msg = bleuart.readStringUntil('\n');
        bleuart.printf("CLIENT %lu: %s\n", g_cfg.client_id, msg.c_str());
      }
    }

    // przycisk na PIN_022 (twój istniejący switch)
    if (g_sw_irq_flag) {
      noInterrupts();
      g_sw_irq_flag = false;
      uint32_t cnt = g_sw_count;
      interrupts();

      if (Bluefruit.connected()) {
        char buf[32];
        snprintf(buf, sizeof(buf), "SW %lu\n", (unsigned long)cnt);
        bleuart.write((const uint8_t*)buf, strlen(buf));
        LOGF("[CLIENT] sent SWITCH #%lu to server", (unsigned long)cnt);

        if (g_oled_ok) {
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, (OLED_H >= 64) ? 40 : 24);
          display.print("BTN cnt: ");
          display.println((unsigned long)cnt);
          oledSafeDisplay();
        }
      } else {
        LOGF("[CLIENT] switch press, but not connected – skipped");
      }
    }

    // NOWE: przycisk na PIN_024 → wyślij „KEY A”
    if (g_btn024_cli_irq) {
      noInterrupts(); g_btn024_cli_irq = false; interrupts();

      if (Bluefruit.connected()) {
        bleuart.write((const uint8_t*)"KEY A\n", 6);
        LOGF("[CLIENT] sent KEY A to server");

        if (g_oled_ok) {
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, (OLED_H >= 64) ? 24 : 16);
          display.println("KEY A sent");
          oledSafeDisplay();
        }
      } else {
        LOGF("[CLIENT] PIN_024 press, but not connected – skipped");
      }
    }

    // ewentualny reassign przez dock
    if (UART_DOCK.available()){
      String l = readLine(UART_DOCK,5);
      if (l.startsWith("ASSIGN ")){
        uint32_t nid = l.substring(7).toInt();
        if (nid){ g_cfg.client_id=nid; if (cfg_save()){ UART_DOCK.println("OK"); LOGF("[CLIENT] reassign ID=%lu", nid); } }
      }
    }

    yield();
  }
}

// ===== MAIN =====
void setup(){
  // HID zaczynamy wcześnie, żeby host widział urządzenie jako CDC+HID
  usb_hid.begin();

  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  UART_DOCK.setPins(UART_RX_PIN, UART_TX_PIN);
  UART_DOCK.begin(UART_BAUD);
  USB_LOG.begin(115200);

  // ===== OLED begin =====
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
    oledSafeDisplay();
  }

  LOGF("[IO] PIN_022 configured: server=LED (OUTPUT), client=SWITCH (INPUT_PULLUP)");
  LOGF("[IO] PIN_024 configured: server=BTN (INPUT_PULLUP) / client=KEY A button");

  // Faza BOOT (krótka)
  uint32_t t0=millis();
  while(!usb_mounted() && millis()-t0<6000){
    LOGF("[BOOT] waiting for USB...");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
  LOGF(usb_mounted() ? "[BOOT] USB mounted" : "[BOOT] USB timeout");

  if (usb_mounted()) run_server();
  else               run_client_idle_or_ble();
}
void loop(){}
