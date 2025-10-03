#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include "esp_camera.h"

// ===== AI Thinker ESP32-CAM pin map =====
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM   35
#define Y8_GPIO_NUM   34
#define Y7_GPIO_NUM   39
#define Y6_GPIO_NUM   36
#define Y5_GPIO_NUM   21
#define Y4_GPIO_NUM   19
#define Y3_GPIO_NUM   18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define LED_FLASH_GPIO  4

// ===== Device ID / Hostname =====
const char* HOST = "esp32cam-robot";

// ===== Defaults (під себе) =====
const char* DEF_WIFI_SSID = "5 kv";
const char* DEF_WIFI_PASS = "Nz31032020";
IPAddress DEF_IP  (192,168,1,60);
IPAddress DEF_GW  (192,168,1,1);
IPAddress DEF_MSK (255,255,255,0);
IPAddress DEF_DNS (192,168,1,1);

// ===== Fallback AP =====
const char* AP_SSID = "esp32cam-robot";
const char* AP_PASS = "12345678"; // зміни для безпеки

// ===== Wi-Fi авто-відновлення =====
const uint32_t WIFI_RECONNECT_INTERVAL_MS = 4000;
const uint32_t WIFI_REINIT_AFTER_MS       = 15000;
const uint32_t WIFI_AP_FALLBACK_MS        = 30000;
const uint32_t WIFI_BOOT_FASTAP_MS        = 12000;

// ===== Globals =====
WebServer   server(80);
WiFiUDP     udpAnn;   // для анонсу IP
WiFiUDP     udpCmd;   // для команд на MEGA
Preferences prefs;

framesize_t g_frame = FRAMESIZE_QVGA; // 320x240
int         g_q     = 26;
bool        g_psram = false;

struct NetConfig {
  String ssid, pass;
  bool   useDHCP;
  IPAddress ip, gw, msk, dns;
  String webUser, webPass;
  IPAddress mega_ip;     // куди слати команди
  uint16_t  mega_port;   // порт MEGA
};
NetConfig cfg;

bool     g_apMode        = false;
bool     g_serverStarted = false;
bool     g_mdnsStarted   = false;

uint32_t g_lastOkTs      = 0;
uint32_t g_lastTryTs     = 0;
uint32_t g_lastReinitTs  = 0;

static String ipToStr(const IPAddress& ip){
  return String(ip[0])+"."+ip[1]+"."+ip[2]+"."+ip[3];
}
static bool strToIp(const String& s, IPAddress& out){
  int a,b,c,d; char dot;
  if (sscanf(s.c_str(), "%d%c%d%c%d%c%d", &a,&dot,&b,&dot,&c,&dot,&d) == 7){
    if ((unsigned)a<=255&&(unsigned)b<=255&&(unsigned)c<=255&&(unsigned)d<=255){
      out = IPAddress(a,b,c,d); return true;
    }
  }
  return false;
}

void loadConfig(){
  prefs.begin("net", true);
  cfg.ssid    = prefs.getString("ssid", DEF_WIFI_SSID);
  cfg.pass    = prefs.getString("pass", DEF_WIFI_PASS);
  cfg.useDHCP = prefs.getBool  ("dhcp", true);
  cfg.ip      = IPAddress(prefs.getUInt("ip",  (uint32_t)DEF_IP));
  cfg.gw      = IPAddress(prefs.getUInt("gw",  (uint32_t)DEF_GW));
  cfg.msk     = IPAddress(prefs.getUInt("msk", (uint32_t)DEF_MSK));
  cfg.dns     = IPAddress(prefs.getUInt("dns", (uint32_t)DEF_DNS));
  cfg.webUser = prefs.getString("wuser", "admin");
  cfg.webPass = prefs.getString("wpass", "admin");
  cfg.mega_ip = IPAddress(prefs.getUInt("mip", (uint32_t)IPAddress(192,168,1,61)));
  cfg.mega_port = prefs.getUShort("mport", 4210);
  prefs.end();
}
void saveConfig(){
  prefs.begin("net", false);
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("pass", cfg.pass);
  prefs.putBool  ("dhcp", cfg.useDHCP);
  prefs.putUInt  ("ip",  (uint32_t)cfg.ip);
  prefs.putUInt  ("gw",  (uint32_t)cfg.gw);
  prefs.putUInt  ("msk", (uint32_t)cfg.msk);
  prefs.putUInt  ("dns", (uint32_t)cfg.dns);
  prefs.putString("wuser", cfg.webUser);
  prefs.putString("wpass", cfg.webPass);
  prefs.putUInt  ("mip",  (uint32_t)cfg.mega_ip);
  prefs.putUShort("mport", cfg.mega_port);
  prefs.end();
}

bool initCamera(){
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH); delay(10);
  digitalWrite(PWDN_GPIO_NUM, LOW);  delay(50);

  g_psram = psramFound();

  camera_config_t c{};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;

  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sscb_sda = SIOD_GPIO_NUM; c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;    c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = g_frame;
  c.jpeg_quality = g_q;

  if (g_psram) { c.fb_location = CAMERA_FB_IN_PSRAM; c.fb_count = 2; c.grab_mode = CAMERA_GRAB_LATEST; }
  else         { c.fb_location = CAMERA_FB_IN_DRAM;  c.fb_count = 1; c.grab_mode = CAMERA_GRAB_WHEN_EMPTY; }

  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_contrast(s,1);
    s->set_sharpness(s,2);
    s->set_aec2(s,1);
    s->set_dcw(s,1);
    s->set_lenc(s,1);
  }
  return true;
}

// ---- UI ----
const char INDEX_HTML[] PROGMEM =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32-CAM</title>"
"<style>body{font-family:system-ui;margin:16px}img{width:100%;max-width:480px;background:#000}"
".row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}"
"button,select,input{padding:.5rem .75rem}a.btn,button{border:1px solid #ccc;border-radius:8px;text-decoration:none}"
"small{opacity:.7}</style></head><body>"
"<h3>ESP32-CAM</h3>"
"<div class='row'>"
"<a class='btn' href='/stream' target='_blank'><button>Open MJPEG /stream</button></a>"
"<a class='btn' href='/robot'><button>Robot Control</button></a>"
"<button onclick=\"fetch('/flash?on=1')\">Flash ON</button>"
"<button onclick=\"fetch('/flash?on=0')\">Flash OFF</button>"
"<a class='btn' href='/wifi'><button>Wi-Fi Settings</button></a>"
"<a class='btn' href='/diag'><button>Diagnostics</button></a>"
"</div>"
"<div class='row'>"
"<label>Size: <select id='size'>"
"<option value='qqvga'>QQVGA</option>"
"<option value='qvga' selected>QVGA</option>"
"<option value='vga'>VGA</option>"
"</select></label>"
"<label>Quality: <input id='q' type='number' min='10' max='63' value='26' style='width:5rem'></label>"
"<button onclick='apply()'>Apply</button>"
"</div>"
"<img id='v' src='/jpg'>"
"<p><small>Auto-reconnect Wi-Fi + AP fallback enabled.</small></p>"
"<script>let ms=1000;function tick(){document.getElementById('v').src='/jpg?ts='+Date.now()}function loop(){tick();setTimeout(loop,ms)}async function apply(){const s=document.getElementById('size').value;const q=document.getElementById('q').value;try{await fetch('/cfg?size='+s+'&q='+q)}catch(e){}}loop();</script>"
"</body></html>";

const char ROBOT_HTML[] PROGMEM =
"<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Robot Control</title>"
"<style>body{font-family:system-ui;margin:16px;max-width:720px} .row{display:flex;gap:8px;flex-wrap:wrap}"
"button,input,select{padding:.6rem;border:1px solid #ccc;border-radius:10px}</style></head><body>"
"<h3>Robot Control</h3>"
"<div class='row'>"
"<button onclick=\"fetch('/rc?l=200&r=200')\">Forward</button>"
"<button onclick=\"fetch('/rc?l=-200&r=-200')\">Back</button>"
"<button onclick=\"fetch('/rc?l=-200&r=200')\">Left</button>"
"<button onclick=\"fetch('/rc?l=200&r=-200')\">Right</button>"
"<button onclick=\"fetch('/stop',{method:'POST'})\">STOP</button>"
"</div>"
"<h4>Servo</h4>"
"<div class='row'>"
"<select id='j'><option value='root'>Root</option><option value='a1'>A1</option><option value='a2'>A2</option>"
"<option value='armb'>ArmB</option><option value='wrista'>WristA</option><option value='wristb'>WristB</option>"
"<option value='grip'>Grip</option></select>"
"<input id='ang' type='number' min='0' max='180' value='90' style='width:6rem'>"
"<button onclick=\"srv()\">Set</button>"
"</div>"
"<p><a href='/'>← Camera</a> | <a href='/diag'>Wi-Fi Diag</a></p>"
"<script>function srv(){const j=document.getElementById('j').value;const a=document.getElementById('ang').value;fetch('/servo?j='+j+'&a='+a)}</script>"
"</body></html>";

void handleIndex(){ server.send_P(200,"text/html",INDEX_HTML); }
void handleRobot(){ server.send_P(200,"text/html",ROBOT_HTML); }

// ---- Pages ----
String wifiFormHtml(){
  String masked; masked.reserve(cfg.pass.length());
  for (size_t i=0;i<cfg.pass.length();++i) masked += '*';

  String h;
  h += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Wi-Fi settings</title><style>body{font-family:system-ui;margin:16px;max-width:720px}"
       "label{display:block;margin:.5rem 0 .2rem}input,select,button{padding:.5rem;border:1px solid #ccc;border-radius:8px;width:100%}"
       ".row{display:grid;grid-template-columns:1fr 1fr;gap:12px}small{opacity:.7}</style></head><body>";
  h += "<h3>Wi-Fi & Network</h3><form method='POST' action='/wifi'>";
  h += "<label>SSID</label><input name='ssid' value='" + cfg.ssid + "' required>";
  h += "<label>Password (leave empty = no change)</label><input name='pass' type='password' placeholder='" + masked + "'>";
  h += "<label>Mode</label><select name='mode'>";
  h += String("<option value='dhcp'") + (cfg.useDHCP?" selected":"") + ">DHCP</option>";
  h += String("<option value='static'") + (!cfg.useDHCP?" selected":"") + ">Static IP</option></select>";
  h += "<div class='row'><div><label>IP</label><input name='ip' value='" + ipToStr(cfg.ip) + "'></div>";
  h += "<div><label>Gateway</label><input name='gw' value='" + ipToStr(cfg.gw) + "'></div></div>";
  h += "<div class='row'><div><label>Mask</label><input name='msk' value='" + ipToStr(cfg.msk) + "'></div>";
  h += "<div><label>DNS</label><input name='dns' value='" + ipToStr(cfg.dns) + "'></div></div>";

  h += "<h4>Robot (MEGA) UDP</h4>";
  h += "<div class='row'><div><label>MEGA IP</label><input name='mip' value='" + ipToStr(cfg.mega_ip) + "'></div>";
  h += "<div><label>MEGA Port</label><input name='mport' value='" + String(cfg.mega_port) + "'></div></div>";

  h += "<h4>Web auth (opt)</h4><div class='row'><div><label>User</label><input name='wuser' value='" + cfg.webUser + "'></div>";
  h += "<div><label>Password</label><input name='wpass' type='password' placeholder='••••••'></div></div>";
  h += "<p><small>Після збереження пристрій перезапуститься з новими налаштуваннями.</small></p>";
  h += "<button type='submit'>Save & Reboot</button></form><p><a href='/'>← Back</a></p></body></html>";
  return h;
}

String wifiDiagHtml(){
  wl_status_t st = WiFi.status();
  String h;
  h += "<html><body style='font-family:system-ui'><h3>Wi-Fi Diagnostics</h3><pre>";
  h += "Configured SSID: " + cfg.ssid + "\n";
  h += "Mode: " + String(cfg.useDHCP ? "DHCP" : "Static") + "\n";
  if (!cfg.useDHCP){
    h += "IP: " + ipToStr(cfg.ip) + "  GW: " + ipToStr(cfg.gw) + "  MASK: " + ipToStr(cfg.msk) + "  DNS: " + ipToStr(cfg.dns) + "\n";
  }
  h += "Status: " + String((int)st) + " (3=WL_CONNECTED)\n";
  if (st == WL_CONNECTED){
    h += "Current IP: " + WiFi.localIP().toString() + "\n";
    h += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
    h += "MAC: " + WiFi.macAddress() + "\n";
    h += "MEGA: " + ipToStr(cfg.mega_ip) + ":" + String(cfg.mega_port) + "\n";
  }
  h += "</pre><p><a href='/'>← Back</a></p></body></html>";
  return h;
}

// ---- Camera/stream ----
void handleJpg(){
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb){ server.send(500,"text/plain","capture failed"); return; }
  server.sendHeader("Cache-Control","no-store");
  server.setContentLength(fb->len);
  server.send(200,"image/jpeg","");
  server.client().write((const uint8_t*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}
void handleStream(){
  WiFiClient client=server.client();
  client.print("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
  while(client.connected()){
    camera_fb_t* fb=esp_camera_fb_get();
    if(!fb) break;
    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",fb->len);
    client.write((const uint8_t*)fb->buf,fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
    delay(5);
  }
}
void handleFlash(){
  String on = server.hasArg("on")?server.arg("on"):"0";
  pinMode(LED_FLASH_GPIO, OUTPUT);
  digitalWrite(LED_FLASH_GPIO, (on=="1")?HIGH:LOW);
  server.send(200,"text/plain","ok");
}
framesize_t parseSize(const String&s){
  if(s=="qqvga")return FRAMESIZE_QQVGA;
  if(s=="qvga") return FRAMESIZE_QVGA;
  if(s=="vga")  return FRAMESIZE_VGA;
  return g_frame;
}
void handleCfg(){
  if(server.hasArg("size")){
    framesize_t fs=parseSize(server.arg("size"));
    sensor_t* s=esp_camera_sensor_get();
    if(s) s->set_framesize(s,fs);
  }
  if(server.hasArg("q")){
    int q=constrain(server.arg("q").toInt(),10,63);
    sensor_t* s=esp_camera_sensor_get();
    if(s) s->set_quality(s,q);
  }
  server.send(200,"text/plain","ok");
}

// ---- Wi-Fi settings ----
void handleWifiGet(){ server.send(200,"text/html",wifiFormHtml()); }
void handleWifiPost(){
  if(server.hasArg("ssid")) cfg.ssid=server.arg("ssid");
  if(server.hasArg("pass") && server.arg("pass").length()>0) cfg.pass=server.arg("pass");
  if(server.hasArg("wuser")) cfg.webUser=server.arg("wuser");
  if(server.hasArg("wpass") && server.arg("wpass").length()>0) cfg.webPass=server.arg("wpass");
  cfg.useDHCP=(server.arg("mode")=="dhcp");
  if(!cfg.useDHCP){
    IPAddress ip=cfg.ip, gw=cfg.gw, msk=cfg.msk, dns=cfg.dns;
    if(server.hasArg("ip"))  strToIp(server.arg("ip"), ip);
    if(server.hasArg("gw"))  strToIp(server.arg("gw"), gw);
    if(server.hasArg("msk")) strToIp(server.arg("msk"), msk);
    if(server.hasArg("dns")) strToIp(server.arg("dns"), dns);
    cfg.ip=ip; cfg.gw=gw; cfg.msk=msk; cfg.dns=dns;
  }
  IPAddress mip = cfg.mega_ip;
  if(server.hasArg("mip"))  strToIp(server.arg("mip"), mip);
  cfg.mega_ip = mip;
  if(server.hasArg("mport")) cfg.mega_port = constrain(server.arg("mport").toInt(), 1, 65535);

  saveConfig();
  server.send(200,"text/html","<h3>Saved. Rebooting...</h3>");
  delay(300);
  ESP.restart();
}

void handleDiag(){ server.send(200,"text/html", wifiDiagHtml()); }

void handleForget(){
  prefs.begin("net", false);
  prefs.putString("ssid", "");
  prefs.putString("pass", "");
  prefs.end();
  server.send(200,"text/plain","Wi-Fi creds cleared. Starting AP...");
  delay(200);
  WiFi.disconnect(true,true);
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192,168,4,1), apGW(192,168,4,1), apMSK(255,255,255,0);
  WiFi.softAPConfig(apIP, apGW, apMSK);
  WiFi.softAP(AP_SSID, AP_PASS);
}
void handleNuke(){
  prefs.begin("net", false);
  prefs.clear();
  prefs.end();
  server.send(200,"text/plain","NVS 'net' wiped. Rebooting...");
  delay(300);
  ESP.restart();
}

// ---- UDP proxy to MEGA ----
void sendToMega(const String& msg){
  if (WiFi.status() != WL_CONNECTED) return;
  udpCmd.beginPacket(cfg.mega_ip, cfg.mega_port);
  udpCmd.write((const uint8_t*)msg.c_str(), msg.length());
  udpCmd.endPacket();
}
void handleRC(){ // /rc?l=..&r=..
  int l = server.hasArg("l") ? server.arg("l").toInt() : 0;
  int r = server.hasArg("r") ? server.arg("r").toInt() : 0;
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
  String cmd = "DRV L=" + String(l) + " R=" + String(r);
  sendToMega(cmd);
  server.send(200, "text/plain", "ok");
}
void handleServo(){ // /servo?j=..&a=..
  String j = server.hasArg("j") ? server.arg("j") : "";
  int a = server.hasArg("a") ? server.arg("a").toInt() : 90;
  a = constrain(a, 0, 180);
  if(j.length()==0){ server.send(400,"text/plain","no joint"); return; }
  String cmd = "SRV ID=" + j + " A=" + String(a);
  sendToMega(cmd);
  server.send(200, "text/plain", "ok");
}
void handleStop(){
  sendToMega("STOP");
  server.send(200, "text/plain", "ok");
}

// ---- Safe starters ----
void startHttpServerOnce(){
  if (g_serverStarted) return;
  server.on("/",       HTTP_GET, handleIndex);
  server.on("/robot",  HTTP_GET, handleRobot);
  server.on("/jpg",    HTTP_GET, handleJpg);
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/flash",  HTTP_GET, handleFlash);
  server.on("/cfg",    HTTP_GET, handleCfg);
  server.on("/wifi",   HTTP_GET, handleWifiGet);
  server.on("/wifi",   HTTP_POST, handleWifiPost);
  server.on("/diag",   HTTP_GET, handleDiag);
  server.on("/forget", HTTP_POST, handleForget);
  server.on("/nuke",   HTTP_POST, handleNuke);

  // керування
  server.on("/rc",     HTTP_GET,  handleRC);
  server.on("/servo",  HTTP_GET,  handleServo);
  server.on("/stop",   HTTP_POST, handleStop);

  server.begin();
  g_serverStarted = true;
  Serial.println("HTTP server started.");
}
void tryStartMDNS(){
  if (g_mdnsStarted) return;
  if (MDNS.begin(HOST)) {
    MDNS.addService("http","tcp",80);
    g_mdnsStarted = true;
    Serial.println("mDNS started.");
  }
}

// ---- Announce IP (UDP) ----
void announceIp(){
  if (WiFi.status() == WL_CONNECTED){
    udpAnn.begin(31337);
    udpAnn.beginPacket("255.255.255.255",31337);
    udpAnn.printf("ESP32CAM %s %s",HOST,WiFi.localIP().toString().c_str());
    udpAnn.endPacket();
  }
}

// ---- Wi-Fi helpers ----
void startSTA_nonblocking(){
  g_apMode = false;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOST);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  if(!cfg.useDHCP) WiFi.config(cfg.ip,cfg.gw,cfg.msk,cfg.dns);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
  g_lastTryTs = millis();
}
void startAP(){
  g_apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  IPAddress apIP(192,168,4,1), apGW(192,168,4,1), apMSK(255,255,255,0);
  WiFi.softAPConfig(apIP, apGW, apMSK);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  startHttpServerOnce();
  tryStartMDNS();
}
void reinitWiFiStack(){
  WiFi.disconnect(true, true);
  delay(100);
  startSTA_nonblocking();
  g_lastReinitTs = millis();
}

// ---- Wi-Fi events ----
void onWiFiEvent(WiFiEvent_t event){
  switch(event){
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      g_lastOkTs = millis();
      Serial.print("IP: "); Serial.println(WiFi.localIP());
      announceIp();
      startHttpServerOnce();
      tryStartMDNS();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      g_mdnsStarted = false;
      g_serverStarted = false;
      break;
    default: break;
  }
}

// ---- Setup ----
void setup(){
  Serial.begin(115200);
  delay(200);
  pinMode(LED_FLASH_GPIO, OUTPUT);
  digitalWrite(LED_FLASH_GPIO, LOW);

  loadConfig();
  if(!initCamera()){
    Serial.println("Camera init FAILED");
  }
  WiFi.onEvent(onWiFiEvent);
  startSTA_nonblocking();
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_BOOT_FASTAP_MS) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED){
    // події запустять HTTP/mDNS
  } else {
    startAP();
    Serial.println("Boot: STA failed fast -> AP mode");
  }
}

// ---- Loop ----
void loop(){
  if (g_serverStarted) server.handleClient();

  const uint32_t now = millis();
  wl_status_t st = WiFi.status();

  if (!g_apMode){
    if (st == WL_CONNECTED){
      g_lastOkTs = now;
    } else {
      if (now - g_lastTryTs > WIFI_RECONNECT_INTERVAL_MS){
        WiFi.reconnect();
        g_lastTryTs = now;
      }
      if (now - g_lastOkTs > WIFI_REINIT_AFTER_MS && now - g_lastReinitTs > WIFI_REINIT_AFTER_MS){
        reinitWiFiStack();
      }
      if (now - g_lastOkTs > WIFI_AP_FALLBACK_MS){
        startAP();
        Serial.println("Wi-Fi offline too long -> AP mode");
      }
    }
  }
  delay(2);
}
