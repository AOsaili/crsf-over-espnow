#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <Preferences.h>
#include <driver/uart.h>

#define CRSF_PIN     4
#define CRSF_UART    UART_NUM_1

#define DEF_CBAUD    400000
#define DEF_INVERT   true
#define DEF_CHAN     1
#define DEF_RATE     100
#define DEF_LOG      false
#define DEF_DUMPMS   500
#define DEF_SBAUD    115200
#define DEF_CTELEM   false         // link stats -> radio: OFF by default
#define DEF_LINKMS   100

#define TX_POWER_IDX 3             // 3 = 100mW, matches ESP-NOW ~20dBm
#define RF_MODE_IDX  2
#define DEVICE_NAME  "ESP-NOW"

HardwareSerial CRSF(1);
Preferences prefs;

typedef struct {
  uint16_t ch[16];
  uint32_t frames;
  uint32_t ms;
} RcPacket;

typedef struct {
  uint8_t  magic;                 // 0x7E
  uint8_t  telemHz;
  int8_t   rssi;
  uint8_t  lq;
  uint32_t rxFrames;
  uint32_t lost;
  uint32_t uptime;
  uint16_t vbat;
} TelemPacket;

RcPacket packet;

uint8_t  defaultMac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t  receiverMac[6];

uint32_t crsfBaud    = DEF_CBAUD;
bool     crsfInvert  = DEF_INVERT;
uint8_t  wifiChannel = DEF_CHAN;
uint16_t sendHz      = DEF_RATE;
bool     logChannels = DEF_LOG;
uint16_t dumpMs      = DEF_DUMPMS;
uint32_t serialBaud  = DEF_SBAUD;
bool     crsfTelem   = DEF_CTELEM;
uint16_t linkMs      = DEF_LINKMS;

uint32_t dbgBytesIn = 0, dbgSynced = 0, dbgCrcFail = 0, dbgSendOk = 0, dbgSendFail = 0;
uint32_t dbgAcked = 0, dbgNoAck = 0, dbgPings = 0, dbgRcFrames = 0;

uint8_t frame[64];
int fidx = 0, expectedLen = 0;
bool freshFrame = false;
bool gapNow = false;
bool pingPending = false;

bool    dumpArmed = false;
uint8_t dumpBuf[64];
int     dumpIdx = 0;

// telemetry from the receiver
TelemPacket telem = {};
volatile bool     telemValid  = false;
volatile int8_t   dlRssi      = 0;
volatile uint32_t telemCount  = 0;
unsigned long     lastTelemMs = 0;
uint32_t          telemWin    = 0;
uint8_t           dlLq        = 0;

// ---------------- settings ----------------

void saveSettings() {
  prefs.begin("rctx", false);
  prefs.putULong ("cbaud", crsfBaud);
  prefs.putBool  ("inv",   crsfInvert);
  prefs.putUChar ("chan",  wifiChannel);
  prefs.putUShort("rate",  sendHz);
  prefs.putBool  ("log",   logChannels);
  prefs.putUShort("dump",  dumpMs);
  prefs.putULong ("sbaud", serialBaud);
  prefs.putBool  ("ctelem",crsfTelem);
  prefs.putUShort("linkms",linkMs);
  prefs.putBytes ("peer",  receiverMac, 6);
  prefs.end();
}

void loadSettings() {
  prefs.begin("rctx", true);
  crsfBaud    = prefs.getULong ("cbaud", DEF_CBAUD);
  crsfInvert  = prefs.getBool  ("inv",   DEF_INVERT);
  wifiChannel = prefs.getUChar ("chan",  DEF_CHAN);
  sendHz      = prefs.getUShort("rate",  DEF_RATE);
  logChannels = prefs.getBool  ("log",   DEF_LOG);
  dumpMs      = prefs.getUShort("dump",  DEF_DUMPMS);
  serialBaud  = prefs.getULong ("sbaud", DEF_SBAUD);
  crsfTelem   = prefs.getBool  ("ctelem",DEF_CTELEM);
  linkMs      = prefs.getUShort("linkms",DEF_LINKMS);
  if (prefs.getBytesLength("peer") != 6 || prefs.getBytes("peer", receiverMac, 6) != 6)
    memcpy(receiverMac, defaultMac, 6);
  prefs.end();

  if (crsfBaud < 9600 || crsfBaud > 2000000)     crsfBaud    = DEF_CBAUD;
  if (wifiChannel < 1 || wifiChannel > 13)       wifiChannel = DEF_CHAN;
  if (sendHz > 500)                              sendHz      = DEF_RATE;
  if (dumpMs < 5 || dumpMs > 5000)               dumpMs      = DEF_DUMPMS;
  if (serialBaud < 9600 || serialBaud > 2000000) serialBaud  = DEF_SBAUD;
  if (linkMs < 20 || linkMs > 1000)              linkMs      = DEF_LINKMS;
}

void clearSettings() {
  prefs.begin("rctx", false);
  prefs.clear();
  prefs.end();
}

// ---------------- crsf ----------------

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++)
    crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : crc << 1;
  return crc;
}

bool crc_ok(uint8_t *f) {
  uint8_t crc = 0;
  for (int i = 2; i < 2 + f[1] - 1; i++)
    crc = crc8_dvb_s2(crc, f[i]);
  return crc == f[2 + f[1] - 1];
}

void unpackChannels(uint8_t *p) {
  uint32_t bitbuf = 0;
  int bits = 0, byteIndex = 0;
  for (int i = 0; i < 16; i++) {
    while (bits < 11) {
      bitbuf |= ((uint32_t)p[byteIndex++]) << bits;
      bits += 8;
    }
    packet.ch[i] = bitbuf & 0x7FF;
    bitbuf >>= 11;
    bits -= 11;
  }
}

// LINK_STATISTICS (0x14) -> handset telemetry page
void sendLinkStats() {
  if (!crsfTelem) return;

  bool live = telemValid && (millis() - lastTelemMs < 1000);
  uint8_t upRssi = (live && telem.rssi < 0) ? (uint8_t)(-telem.rssi) : 0;
  uint8_t dnRssi = (dlRssi < 0) ? (uint8_t)(-dlRssi) : 0;

  uint8_t b[14];
  b[0]  = 0xEA;                       // destination: radio
  b[1]  = 0x0C;                       // type + 10 payload + crc
  b[2]  = 0x14;                       // LINK_STATISTICS
  b[3]  = upRssi;                     // uplink RSSI ant1 (-dBm)
  b[4]  = upRssi;                     // ant2 (single antenna)
  b[5]  = live ? telem.lq : 0;        // uplink LQ %
  b[6]  = 0;                          // uplink SNR (ESP-NOW gives none)
  b[7]  = 0;                          // active antenna
  b[8]  = RF_MODE_IDX;
  b[9]  = TX_POWER_IDX;
  b[10] = dnRssi;                     // downlink RSSI (-dBm)
  b[11] = dlLq;                       // downlink LQ %
  b[12] = 0;                          // downlink SNR
  uint8_t crc = 0;
  for (int i = 2; i < 13; i++) crc = crc8_dvb_s2(crc, b[i]);
  b[13] = crc;

  CRSF.write(b, 14);
}

// DEVICE_INFO (0x29) -> answers the radio's DEVICE_PING
void sendDeviceInfo() {
  if (!crsfTelem) return;

  uint8_t b[48];
  int i = 0;
  b[i++] = 0xEA;                      // destination: radio
  int lenPos = i++;                   // filled in below
  b[i++] = 0x29;                      // DEVICE_INFO
  b[i++] = 0xEA;                      // dest
  b[i++] = 0xEE;                      // origin: this module
  for (const char *p = DEVICE_NAME; *p; p++) b[i++] = (uint8_t)*p;
  b[i++] = 0;                         // name terminator
  b[i++]=0; b[i++]=0; b[i++]=0; b[i++]=1;   // serial
  b[i++]=0; b[i++]=0; b[i++]=0; b[i++]=1;   // hardware version
  b[i++]=0; b[i++]=0; b[i++]=0; b[i++]=1;   // firmware version
  b[i++] = 0;                         // parameter count
  b[i++] = 0;                         // parameter version
  uint8_t crc = 0;
  for (int k = 2; k < i; k++) crc = crc8_dvb_s2(crc, b[k]);
  b[i++] = crc;
  b[lenPos] = (uint8_t)(i - 2);

  CRSF.write(b, i);
}

// ---------------- espnow ----------------

void sendPacket() {
  packet.frames++;
  packet.ms = millis();
  esp_err_t r = esp_now_send(receiverMac, (uint8_t *)&packet, sizeof(packet));
  if (r == ESP_OK) dbgSendOk++;
  else {
    dbgSendFail++;
    if (dbgSendFail < 5) Serial.printf("[ESPNOW] send err=%d\n", r);
  }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,4,0)
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) dbgAcked++;
  else                                dbgNoAck++;
}

void onRecvTelem(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(TelemPacket)) return;
  TelemPacket t;
  memcpy(&t, data, sizeof(t));
  if (t.magic != 0x7E) return;
  telem = t;
  if (info->rx_ctrl) dlRssi = info->rx_ctrl->rssi;
  lastTelemMs = millis();
  telemCount++;
  telemValid = true;
}

void updateDlLq() {
  uint8_t expect = telem.telemHz ? telem.telemHz : 10;
  uint32_t got = telemCount - telemWin;
  telemWin = telemCount;
  if (millis() - lastTelemMs > 1500) { dlLq = 0; return; }
  uint32_t v = got * 100 / expect;
  dlLq = v > 100 ? 100 : (uint8_t)v;
}

void resetParser() { fidx = 0; expectedLen = 0; }

void readCRSF() {
  while (CRSF.available()) {
    uint8_t b = CRSF.read();
    dbgBytesIn++;

    if (dumpArmed) {
      dumpBuf[dumpIdx++] = b;
      if (dumpIdx >= 64) {
        Serial.print("[RAW]");
        for (int i = 0; i < 64; i++) Serial.printf(" %02X", dumpBuf[i]);
        Serial.println();
        dumpArmed = false;
      }
    }

    if (fidx == 0) {
      if (b != 0xEE && b != 0xC8) continue;
      frame[fidx++] = b;
      continue;
    }

    frame[fidx++] = b;

    if (fidx == 2) {
      expectedLen = frame[1] + 2;
      if (frame[1] < 2 || frame[1] > 62) { fidx = 0; continue; }
    }

    if (fidx >= expectedLen) {
      dbgSynced++;
      if (frame[2] == 0x16 && frame[1] == 24) {
        if (crc_ok(frame)) {
          dbgRcFrames++;
          unpackChannels(frame + 3);
          freshFrame = true;
          if (sendHz == 0) sendPacket();
        } else dbgCrcFail++;
      }
      else if (frame[2] == 0x28 && crc_ok(frame)) {   // DEVICE_PING
        dbgPings++;
        pingPending = true;
      }
      fidx = 0;
      gapNow = true;          // safe moment to talk back
    }
  }
}

// ---------------- runtime config ----------------

void applyUart() {
  CRSF.end();
  delay(10);

  Serial.println("[CRSF] begin...");
  CRSF.begin(crsfBaud, SERIAL_8N1, CRSF_PIN, -1, crsfInvert);   // rx-only, known good
  Serial.println("[CRSF] begin ok");

  if (crsfTelem) {
    Serial.println("[CRSF] set_pin...");
    esp_err_t e1 = uart_set_pin(CRSF_UART, CRSF_PIN, CRSF_PIN,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    Serial.printf("[CRSF] set_pin=%d\n", e1);

    Serial.println("[CRSF] set_mode...");
    esp_err_t e2 = uart_set_mode(CRSF_UART, UART_MODE_RS485_HALF_DUPLEX);
    Serial.printf("[CRSF] set_mode=%d\n", e2);
  }

  resetParser();
  Serial.printf("[CRSF] baud=%lu invert=%s pin=%d mode=%s\n",
                (unsigned long)crsfBaud, crsfInvert ? "on" : "off", CRSF_PIN,
                crsfTelem ? "half-duplex" : "rx-only");
}

void setPeer(uint8_t *mac) {
  esp_now_del_peer(receiverMac);
  memcpy(receiverMac, mac, 6);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_err_t r = esp_now_add_peer(&peer);
  Serial.printf("[ESPNOW] peer -> %02X:%02X:%02X:%02X:%02X:%02X  %s\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
                r == ESP_OK ? "OK" : "add_peer FAILED");
}

// ---------------- serial commands ----------------

void printHelp() {
  Serial.println(F(
    "\n--- commands ---\n"
    "help              this list\n"
    "stat              counters + config\n"
    "link              telemetry from the receiver\n"
    "ch                dump all 16 channels once\n"
    "raw               hex dump next 64 CRSF bytes\n"
    "baud <n>          CRSF baud (400000, 420000, 921600, 115200)\n"
    "inv on|off        CRSF inverted serial\n"
    "telem on|off      link stats -> handset (half-duplex, default off)\n"
    "linkms <20-1000>  link stats interval\n"
    "rate <0-500>      ESP-NOW send Hz (0 = every frame)\n"
    "peer <MAC>        target RX, aa:bb:cc:dd:ee:ff\n"
    "bcast             target broadcast FF:FF:FF:FF:FF:FF\n"
    "chan <1-13>       wifi channel (must match RX)\n"
    "log on|off|<ms>   periodic [CH] dump, 5-5000 ms\n"
    "sbaud <n>         console baud (reconnect monitor after!)\n"
    "mac               this board's MAC\n"
    "zero              reset counters\n"
    "defaults          erase saved settings + reboot\n"
    "reboot            restart\n"
    "  (settings are saved to flash automatically)\n"
    "----------------"));
}

void printChannels() {
  Serial.print("[CH]");
  for (int i = 0; i < 16; i++) Serial.printf(" %4u", packet.ch[i]);
  Serial.println();
}

void printLink() {
  bool live = telemValid && (millis() - lastTelemMs < 1000);
  if (!live) { Serial.println("[LINK] no telemetry from receiver"); return; }
  Serial.printf("[LINK] up: %ddBm %u%%  down: %ddBm %u%%  rxFrames=%lu lost=%lu rxUp=%lus\n",
                telem.rssi, telem.lq, dlRssi, dlLq,
                (unsigned long)telem.rxFrames, (unsigned long)telem.lost,
                (unsigned long)telem.uptime);
}

void printStat() {
  Serial.printf("[STAT] in=%lu synced=%lu rc=%lu ping=%lu crcFail=%lu q=%lu qFail=%lu "
                "ack=%lu noAck=%lu | cbaud=%lu inv=%s rate=%uHz ch=%d ctelem=%s/%ums "
                "log=%s dump=%ums sbaud=%lu peer=%02X:%02X:%02X:%02X:%02X:%02X\n",
                dbgBytesIn, dbgSynced, dbgRcFrames, dbgPings, dbgCrcFail,
                dbgSendOk, dbgSendFail, dbgAcked, dbgNoAck,
                (unsigned long)crsfBaud, crsfInvert ? "on" : "off", sendHz, WiFi.channel(),
                crsfTelem ? "on" : "off", linkMs,
                logChannels ? "on" : "off", dumpMs, (unsigned long)serialBaud,
                receiverMac[0],receiverMac[1],receiverMac[2],
                receiverMac[3],receiverMac[4],receiverMac[5]);
}

void runCommand(String c) {
  c.trim();
  c.toLowerCase();
  if (!c.length()) return;

  bool dirty = true;

  if      (c == "help" || c == "?") { printHelp();     dirty = false; }
  else if (c == "stat")             { printStat();     dirty = false; }
  else if (c == "link")             { printLink();     dirty = false; }
  else if (c == "ch")               { printChannels(); dirty = false; }
  else if (c == "mac")              { Serial.println(WiFi.macAddress()); dirty = false; }
  else if (c == "raw")              { dumpIdx = 0; dumpArmed = true;
                                      Serial.println("[RAW] armed"); dirty = false; }
  else if (c == "zero")             { dbgBytesIn = dbgSynced = dbgCrcFail = 0;
                                      dbgRcFrames = dbgPings = 0;
                                      dbgSendOk = dbgSendFail = dbgAcked = dbgNoAck = 0;
                                      Serial.println("[STAT] cleared"); dirty = false; }
  else if (c == "inv on")           { crsfInvert = true;  applyUart(); }
  else if (c == "inv off")          { crsfInvert = false; applyUart(); }
  else if (c == "telem on")         { crsfTelem = true;  applyUart(); }
  else if (c == "telem off")        { crsfTelem = false; applyUart(); }
  else if (c == "bcast")            { uint8_t b[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; setPeer(b); }
  else if (c == "reboot")           { Serial.println("[SYS] restarting..."); delay(100); ESP.restart(); }
  else if (c == "defaults") {
    clearSettings();
    Serial.println("[NVS] cleared, restarting...");
    delay(100);
    ESP.restart();
  }
  else if (c.startsWith("linkms ")) {
    int ms = c.substring(7).toInt();
    if (ms < 20 || ms > 1000) { Serial.println("[CRSF] linkms must be 20-1000"); dirty = false; }
    else { linkMs = (uint16_t)ms; Serial.printf("[CRSF] link stats every %ums\n", linkMs); }
  }
  else if (c.startsWith("baud ")) {
    uint32_t b = (uint32_t)c.substring(5).toInt();
    if (b < 9600 || b > 2000000) { Serial.println("[CRSF] baud must be 9600-2000000"); dirty = false; }
    else { crsfBaud = b; applyUart(); }
  }
  else if (c.startsWith("rate ")) {
    int hz = c.substring(5).toInt();
    if (hz < 0 || hz > 500) { Serial.println("[TX] rate must be 0-500"); dirty = false; }
    else { sendHz = (uint16_t)hz;
           Serial.printf("[TX] rate=%uHz%s\n", sendHz, sendHz ? "" : " (every frame)"); }
  }
  else if (c.startsWith("chan ")) {
    int n = c.substring(5).toInt();
    if (n < 1 || n > 13) { Serial.println("[WIFI] channel must be 1-13"); dirty = false; }
    else { wifiChannel = (uint8_t)n;
           esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
           Serial.printf("[WIFI] channel=%d (RX must match)\n", WiFi.channel()); }
  }
  else if (c.startsWith("peer ")) {
    unsigned int m[6];
    if (sscanf(c.substring(5).c_str(), "%x:%x:%x:%x:%x:%x",
               &m[0],&m[1],&m[2],&m[3],&m[4],&m[5]) == 6) {
      uint8_t mac[6];
      for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
      setPeer(mac);
    } else { Serial.println("[ESPNOW] format: peer aa:bb:cc:dd:ee:ff"); dirty = false; }
  }
  else if (c.startsWith("log ")) {
    String a = c.substring(4); a.trim();
    if      (a == "on")  { logChannels = true;  Serial.println("[LOG] on"); }
    else if (a == "off") { logChannels = false; Serial.println("[LOG] off"); }
    else {
      int ms = a.toInt();
      if (ms < 5 || ms > 5000) { Serial.println("[LOG] interval must be 5-5000 ms"); dirty = false; }
      else { dumpMs = (uint16_t)ms; logChannels = true;
             Serial.printf("[LOG] on, every %ums (%d Hz)\n", dumpMs, 1000 / dumpMs); }
    }
  }
  else if (c.startsWith("sbaud ")) {
    uint32_t b = (uint32_t)c.substring(6).toInt();
    if (b < 9600 || b > 2000000) { Serial.println("[SER] baud must be 9600-2000000"); dirty = false; }
    else {
      serialBaud = b;
      saveSettings();
      Serial.printf("[SER] switching to %lu baud - reconnect monitor now\n", (unsigned long)b);
      Serial.flush();
      delay(50);
      Serial.begin(serialBaud);
      dirty = false;
    }
  }
  else { Serial.printf("[CMD] unknown: '%s'  (type help)\n", c.c_str()); dirty = false; }

  if (dirty) saveSettings();
}

void handleSerial() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { buf[len] = 0; runCommand(String(buf)); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = ch;
  }
}

// ---------------- setup / loop ----------------

void setup() {
  loadSettings();

  Serial.begin(serialBaud);
  delay(200);
  Serial.printf("\n[BOOT] Starting RC transmitter... (settings restored, %lu baud)\n",
                (unsigned long)serialBaud);

  applyUart();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  Serial.printf("[WIFI] channel=%d\n", WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init FAILED");
    while (true);
  }
  Serial.println("[ESPNOW] init OK");
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecvTelem);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  Serial.printf("[ESPNOW] add_peer %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
                receiverMac[0],receiverMac[1],receiverMac[2],
                receiverMac[3],receiverMac[4],receiverMac[5],
                esp_now_add_peer(&peer) == ESP_OK ? "OK" : "FAILED");

  Serial.print("[BOOT] TX MAC: ");
  Serial.println(WiFi.macAddress());
  printHelp();
}

void loop() {
  handleSerial();
  readCRSF();

  // talk back only in the gap right after a frame
  static unsigned long lastLinkTx = 0;
  if (gapNow) {
    gapNow = false;
    if (crsfTelem) {
      if (pingPending) {
        pingPending = false;
        sendDeviceInfo();
      } else if (millis() - lastLinkTx >= linkMs) {
        sendLinkStats();
        lastLinkTx = millis();
      }
    } else {
      pingPending = false;
    }
  }

  if (sendHz > 0 && freshFrame) {
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= (unsigned long)(1000 / sendHz)) {
      sendPacket();
      freshFrame = false;
      lastSend = millis();
    }
  }

  static unsigned long lastLq = 0;
  if (millis() - lastLq >= 1000) { updateDlLq(); lastLq = millis(); }

  if (logChannels) {
    static unsigned long lastDump = 0;
    if (millis() - lastDump >= dumpMs) { printChannels(); lastDump = millis(); }
  } else {
    static unsigned long lastStat = 0;
    if (millis() - lastStat >= 2000) { printStat(); lastStat = millis(); }
  }
}