#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_SDA 8
#define OLED_SCL 9
#define OLED_ADDR 0x3C

#define AP_SSID "RC-Receiver"
#define AP_PASS "12345678"

#define DEF_WEB      false
#define DEF_LOG      true
#define DEF_CHAN     1
#define DEF_DUMPMS   500
#define DEF_SBAUD    115200
#define DEF_TELEMHZ  10

bool     webEnabled  = false;
bool     webOnBoot   = DEF_WEB;
bool     logChannels = DEF_LOG;
uint8_t  wifiChannel = DEF_CHAN;
uint16_t dumpMs      = DEF_DUMPMS;
uint32_t serialBaud  = DEF_SBAUD;
uint8_t  telemHz     = DEF_TELEMHZ;   // 0 = telemetry off

Preferences prefs;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
WebServer server(80);

typedef struct {
  uint16_t ch[16];
  uint32_t frames;
  uint32_t ms;
} RcPacket;                       // 40 bytes, TX -> RX

typedef struct {
  uint8_t  magic;                 // 0x7E
  uint8_t  telemHz;
  int8_t   rssi;                  // uplink RSSI measured here, dBm
  uint8_t  lq;                    // uplink link quality 0-100
  uint32_t rxFrames;
  uint32_t lost;
  uint32_t uptime;
  uint16_t vbat;                  // mV, 0 = not measured
} TelemPacket;                    // 20 bytes, RX -> TX

RcPacket rxData;
volatile bool newData = false;
unsigned long lastReceiveMs = 0;

// link measurement
volatile int8_t   lastRssi   = 0;
volatile uint32_t pktCount   = 0;
volatile uint32_t lastFrameNo = 0;
volatile uint32_t lostTotal  = 0;
uint32_t winFrame = 0, winPkts = 0;
uint8_t  uplinkLq = 0;

// learned transmitter
uint8_t  txMac[6] = {0};
bool     txMacKnown = false;
volatile bool needPeer = false;

// ---------------- settings ----------------

void saveSettings() {
  prefs.begin("rcrx", false);
  prefs.putBool  ("web",   webEnabled);
  prefs.putBool  ("log",   logChannels);
  prefs.putUChar ("chan",  wifiChannel);
  prefs.putUShort("dump",  dumpMs);
  prefs.putULong ("sbaud", serialBaud);
  prefs.putUChar ("telem", telemHz);
  prefs.end();
}

void loadSettings() {
  prefs.begin("rcrx", true);
  webOnBoot   = prefs.getBool  ("web",   DEF_WEB);
  logChannels = prefs.getBool  ("log",   DEF_LOG);
  wifiChannel = prefs.getUChar ("chan",  DEF_CHAN);
  dumpMs      = prefs.getUShort("dump",  DEF_DUMPMS);
  serialBaud  = prefs.getULong ("sbaud", DEF_SBAUD);
  telemHz     = prefs.getUChar ("telem", DEF_TELEMHZ);
  prefs.end();
  if (wifiChannel < 1 || wifiChannel > 13) wifiChannel = DEF_CHAN;
  if (dumpMs < 5 || dumpMs > 5000)         dumpMs      = DEF_DUMPMS;
  if (serialBaud < 9600 || serialBaud > 2000000) serialBaud = DEF_SBAUD;
  if (telemHz > 50)                        telemHz     = DEF_TELEMHZ;
}

void clearSettings() {
  prefs.begin("rcrx", false);
  prefs.clear();
  prefs.end();
}

// ---------------- espnow ----------------

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(RcPacket)) {
    Serial.printf("[RX] size mismatch: got %d, expected %d\n", len, (int)sizeof(RcPacket));
    return;
  }
  memcpy(&rxData, data, sizeof(RcPacket));
  lastReceiveMs = millis();
  newData = true;

  if (info->rx_ctrl) lastRssi = info->rx_ctrl->rssi;

  // learn the transmitter; peer is added from loop(), never in this callback
  if (!txMacKnown || memcmp(txMac, info->src_addr, 6) != 0) {
    memcpy(txMac, info->src_addr, 6);
    needPeer = true;
  }

  pktCount++;
  if (rxData.frames < lastFrameNo) {          // TX rebooted, counter restarted
    lastFrameNo = rxData.frames;
    lostTotal = 0;
  } else if (lastFrameNo) {
    uint32_t gap = rxData.frames - lastFrameNo;
    if (gap > 1 && gap < 1000) lostTotal += gap - 1;
  }
  lastFrameNo = rxData.frames;
}

void addTxPeer() {
  esp_now_del_peer(txMac);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, txMac, 6);
  p.channel = 0;
  p.encrypt = false;
  esp_err_t r = esp_now_add_peer(&p);
  txMacKnown = (r == ESP_OK);
  Serial.printf("[TELEM] tx peer %02X:%02X:%02X:%02X:%02X:%02X %s\n",
                txMac[0],txMac[1],txMac[2],txMac[3],txMac[4],txMac[5],
                txMacKnown ? "OK" : "FAILED");
}

void sendTelem() {
  if (!txMacKnown || telemHz == 0) return;
  TelemPacket t;
  t.magic    = 0x7E;
  t.telemHz  = telemHz;
  t.rssi     = lastRssi;
  t.lq       = uplinkLq;
  t.rxFrames = pktCount;
  t.lost     = lostTotal;
  t.uptime   = millis() / 1000;
  t.vbat     = 0;
  esp_now_send(txMac, (uint8_t *)&t, sizeof(t));
}

void updateLq() {
  uint32_t fDelta = lastFrameNo - winFrame;
  uint32_t pDelta = pktCount - winPkts;
  if (millis() - lastReceiveMs > 1000)   uplinkLq = 0;
  else if (fDelta == 0)                  uplinkLq = 0;
  else {
    uint32_t v = pDelta * 100 / fDelta;
    uplinkLq = v > 100 ? 100 : (uint8_t)v;
  }
  winFrame = lastFrameNo;
  winPkts  = pktCount;
}

// ---------------- display ----------------

void updateOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  unsigned long age = millis() - lastReceiveMs;

  display.setCursor(0, 0);
  if (age > 1000) display.print("NO LINK");
  else            display.printf("%ddB %u%%", lastRssi, uplinkLq);

  display.setCursor(86, 0);
  display.printf("L:%lu", (unsigned long)lostTotal);

  for (int i = 0; i < 16; i++) {
    display.setCursor((i % 4) * 32, 16 + (i / 4) * 12);
    display.print(rxData.ch[i]);
  }

  display.display();
}

String page() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{font-family:Arial;background:#111;color:white;padding:20px}
.ch{margin:10px 0}.bar{height:18px;background:#333;border-radius:9px;overflow:hidden}
.fill{height:100%;background:#4caf50}.v{float:right;color:#ccc}
</style>
</head>
<body>
<h2>ESP-NOW RC Receiver</h2>
<div id="s">Waiting...</div>
<div id="c"></div>
<script>
async function u(){
 let r=await fetch('/data'),d=await r.json();
 s.innerHTML=`Frames: ${d.frames} &nbsp; RSSI: ${d.rssi} dBm &nbsp; LQ: ${d.lq}%<br>Lost: ${d.lost} &nbsp; Last RX: ${d.age} ms ago`;
 let h='';
 d.ch.forEach((v,i)=>{
  let p=Math.max(0,Math.min(100,(v-172)/(1811-172)*100));
  h+=`<div class="ch">CH${i+1}<span class="v">${v}</span><div class="bar"><div class="fill" style="width:${p}%"></div></div></div>`;
 });
 c.innerHTML=h;
}
setInterval(u,100);u();
</script>
</body>
</html>
)rawliteral";
}

void handleRoot() {
  Serial.println("[HTTP] GET /");
  server.send(200, "text/html", page());
}

void handleData() {
  String json = "{";
  json += "\"frames\":" + String(rxData.frames) + ",";
  json += "\"age\":" + String(millis() - lastReceiveMs) + ",";
  json += "\"rssi\":" + String(lastRssi) + ",";
  json += "\"lq\":" + String(uplinkLq) + ",";
  json += "\"lost\":" + String(lostTotal) + ",";
  json += "\"ch\":[";
  for (int i = 0; i < 16; i++) {
    json += String(rxData.ch[i]);
    if (i < 15) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// ---------------- web on/off ----------------

void startWeb() {
  if (webEnabled) { Serial.println("[WEB] already on"); return; }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS, wifiChannel);
  server.begin();
  webEnabled = true;
  Serial.printf("[WEB] ON  ssid=%s ch=%d  http://192.168.4.1\n", AP_SSID, WiFi.channel());
}

void stopWeb() {
  if (!webEnabled) { Serial.println("[WEB] already off"); return; }
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  webEnabled = false;
  Serial.printf("[WEB] OFF ch=%d\n", WiFi.channel());
}

void setChannel(int n) {
  if (n < 1 || n > 13) { Serial.println("[WIFI] channel must be 1-13"); return; }
  wifiChannel = (uint8_t)n;
  if (webEnabled) { stopWeb(); startWeb(); }
  else            esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  Serial.printf("[WIFI] channel=%d  (TX must be on the same channel)\n", WiFi.channel());
}

// ---------------- serial commands ----------------

void printHelp() {
  Serial.println(F(
    "\n--- commands ---\n"
    "help              this list\n"
    "stat              link status + settings\n"
    "ch                dump all 16 channels once\n"
    "link              rssi / lq / lost\n"
    "web on|off        AP + http server\n"
    "chan <1-13>       wifi channel (must match TX)\n"
    "telem off|<hz>    telemetry back to TX, 1-50 Hz\n"
    "log on|off|<ms>   periodic [CH] dump, 5-5000 ms\n"
    "sbaud <n>         serial baud (reconnect monitor after!)\n"
    "mac               this board's MAC\n"
    "defaults          erase saved settings + reboot\n"
    "reboot            restart\n"
    "  (settings are saved to flash automatically)\n"
    "----------------"));
}

void printChannels() {
  Serial.print("[CH]");
  for (int i = 0; i < 16; i++) Serial.printf(" %4u", rxData.ch[i]);
  Serial.println();
}

void printLink() {
  Serial.printf("[LINK] rssi=%ddBm lq=%u%% lost=%lu rx=%lu age=%lums telem=%uHz tx=%02X:%02X:%02X:%02X:%02X:%02X\n",
                lastRssi, uplinkLq, (unsigned long)lostTotal, (unsigned long)pktCount,
                millis() - lastReceiveMs, telemHz,
                txMac[0],txMac[1],txMac[2],txMac[3],txMac[4],txMac[5]);
}

void printStat() {
  unsigned long age = millis() - lastReceiveMs;
  Serial.printf("[STAT] frames=%lu age=%lums link=%s rssi=%ddBm lq=%u%% | web=%s ch=%d "
                "telem=%uHz log=%s dump=%ums sbaud=%lu mac=%s up=%lus\n",
                (unsigned long)rxData.frames, age, age < 1000 ? "OK" : "LOST",
                lastRssi, uplinkLq,
                webEnabled ? "on" : "off", WiFi.channel(), telemHz,
                logChannels ? "on" : "off", dumpMs, (unsigned long)serialBaud,
                WiFi.macAddress().c_str(), millis() / 1000);
}

void runCommand(String c) {
  c.trim();
  c.toLowerCase();
  if (!c.length()) return;

  bool dirty = true;

  if      (c == "help" || c == "?") { printHelp();     dirty = false; }
  else if (c == "stat")             { printStat();     dirty = false; }
  else if (c == "ch")               { printChannels(); dirty = false; }
  else if (c == "link")             { printLink();     dirty = false; }
  else if (c == "mac")              { Serial.println(WiFi.macAddress()); dirty = false; }
  else if (c == "web on")             startWeb();
  else if (c == "web off")            stopWeb();
  else if (c.startsWith("chan "))     setChannel(c.substring(5).toInt());
  else if (c == "reboot")           { Serial.println("[SYS] restarting..."); delay(100); ESP.restart(); }
  else if (c == "defaults") {
    clearSettings();
    Serial.println("[NVS] cleared, restarting...");
    delay(100);
    ESP.restart();
  }
  else if (c.startsWith("telem ")) {
    String a = c.substring(6); a.trim();
    if (a == "off") { telemHz = 0; Serial.println("[TELEM] off"); }
    else {
      int hz = a.toInt();
      if (hz < 1 || hz > 50) { Serial.println("[TELEM] rate must be 1-50 Hz or 'off'"); dirty = false; }
      else { telemHz = (uint8_t)hz; Serial.printf("[TELEM] %uHz\n", telemHz); }
    }
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
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[len] = 0; runCommand(String(buf)); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = c;
  }
}

// ---------------- setup / loop ----------------

void setup() {
  loadSettings();

  Serial.begin(serialBaud);
  delay(200);
  Serial.printf("\n[BOOT] Starting RC receiver... (settings restored, %lu baud)\n",
                (unsigned long)serialBaud);

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) Serial.println("[OLED] init FAILED");
  else                                                 Serial.println("[OLED] init OK");
  display.clearDisplay();
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  Serial.printf("[WIFI] channel=%d\n", WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init FAILED");
    while (true);
  }
  Serial.println("[ESPNOW] init OK");
  esp_now_register_recv_cb(onReceive);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  if (webOnBoot) startWeb();
  else Serial.println("[WEB] OFF (type 'web on' to enable)");

  Serial.print("[BOOT] RX MAC: ");
  Serial.println(WiFi.macAddress());
  printHelp();
}

void loop() {
  handleSerial();
  if (webEnabled) server.handleClient();

  if (needPeer) { needPeer = false; addTxPeer(); }

  static unsigned long lastTelem = 0;
  if (telemHz && millis() - lastTelem >= (unsigned long)(1000 / telemHz)) {
    sendTelem();
    lastTelem = millis();
  }

  static unsigned long lastLq = 0;
  if (millis() - lastLq >= 500) { updateLq(); lastLq = millis(); }

  static unsigned long lastOled = 0;
  if (millis() - lastOled > 200) { updateOLED(); lastOled = millis(); }

  static unsigned long lastStatus = 0;
  if (logChannels && millis() - lastStatus >= dumpMs) {
    printChannels();
    lastStatus = millis();
  }
}