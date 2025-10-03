/* ========== MEGA + ESP-01 (AT) + NRF24 (опц.) — UDP control, діагностика ========== */

#include <Servo.h>

// --------- ТУТ ВКЛ/ВИКЛ РЕЖИМИ ---------
#define USE_NRF24       1   // 1 — використовувати NRF24, 0 — без пульта
#define FORCE_NO_PULT   0   // 1 — тимчасово ІГНОРУВАТИ пульт, навіть якщо USE_NRF24=1
#define DEBUG_RAW       1   // 1 — показувати сирі байти з ESP-01 (для діагностики)
#define DBG_STATUS_EVERY_MS 2000

// --------- Wi-Fi ---------
const char* WIFI_SSID = "MERCUSYS";
const char* WIFI_PASS = "0987996434";
const uint16_t UDP_PORT = 4210;

// --------- Мотори (L298N приклад) ---------
const int L_IN1=22, L_IN2=23, R_IN1=24, R_IN2=25;
const int L_EN=11, R_EN=12; // PWM (на МЕГА ок)

// --------- Серво ---------
const int SERVO_ROOT_PIN = 2;
const int SERVO_A1_PIN   = 3;
const int SERVO_A2_PIN   = 4;
const int SERVO_ARMB_PIN = 5;
const int SERVO_WA_PIN   = 6;
const int SERVO_WB_PIN   = 7;
const int SERVO_GRIP_PIN = 8;

// --------- NRF24 (пульт) ---------
const uint32_t RADIO_ACTIVE_MS = 1200;
const uint8_t  PULT_FORCE_PIN  = 30;   // LOW = примусовий режим пульта

#if USE_NRF24
  #include <SPI.h>
  #include <RF24.h>
  const uint8_t NRF_CE=9, NRF_CSN=10;
  RF24 radio(NRF_CE, NRF_CSN);
  const byte PIPE_ADDR[6] = "RC01";
  volatile uint32_t lastRadioTs = 0;
#endif

// --------- Глобальні ---------
Servo sRoot, sA1, sA2, sB, sWA, sWB, sGrip;
uint32_t lastCmdTs = 0;
const uint32_t CMD_TIMEOUT_MS = 3000;
uint32_t lastDbgTs = 0;

// --------- Прототипи ---------
String atReadLine(uint32_t timeout_ms=800);
bool atCmd(const String& cmd, const char* okToken="OK", uint32_t timeout_ms=2000, bool echoToUSB=true);
bool atReadUdpPacket(String& out, String* srcIp=nullptr, uint16_t* srcPort=nullptr, uint32_t timeout_ms=50);
bool udpSendTo(const String& ip, uint16_t port, const String& data);
bool wifiConnect();
bool udpStart(uint16_t port);
String getStaIp_(uint32_t total_ms=4000);
void driveLR(int L, int R);
void allStop();
void setJoint(const String& id, int ang);
void parseCommand(const String& s, const char* src);

// =================== AT helpers ===================
String atReadLine(uint32_t timeout_ms){
  String s; uint32_t t0 = millis();
  while (millis()-t0 < timeout_ms) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      s += c;
      if (c=='\n') return s;
    }
  }
  return s;
}

bool atCmd(const String& cmd, const char* okToken, uint32_t timeout_ms, bool echoToUSB){
  if (cmd.length()){
    Serial2.print(cmd); Serial2.print("\r\n");
    if (echoToUSB){ Serial.print(">> "); Serial.println(cmd); }
  }
  String buf; buf.reserve(256);
  uint32_t t0 = millis();
  while (millis()-t0 < timeout_ms) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      buf += c;
      if (echoToUSB) Serial.write(c);
      if (okToken && buf.indexOf(okToken)>=0) return true;
      if (buf.indexOf("ERROR")>=0 || buf.indexOf("FAIL")>=0) return false;
      if (buf.indexOf("ALREADY CONNECTED")>=0) return true;
      if (buf.indexOf("CONNECT")>=0 && String(okToken)=="OK") {/*ок*/}
    }
  }
  return (okToken && buf.indexOf(okToken)>=0);
}

// ======== УНІВЕРСАЛЬНИЙ парсер +IPD (MUX/CIPDINFO будь-які) ========
bool atReadUdpPacket(String& out, String* srcIp, uint16_t* srcPort, uint32_t timeout_ms) {
  static String acc;
  unsigned long t0 = millis();
  auto trim = [](String s){ s.trim(); return s; };

  while (millis() - t0 < timeout_ms) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
#if DEBUG_RAW
      Serial.write(c); // покажемо абсолютно все, що йде від ESP-01
#endif
      acc += c;

      int p = acc.indexOf("+IPD,");
      if (p < 0) {
        if ((int)acc.length() > 1024) acc.remove(0, 512);
        continue;
      }
      int colon = acc.indexOf(':', p+5);
      if (colon < 0) continue;

      String header = trim(acc.substring(p+5, colon)); // між "+IPD," і ':'

      // 1) довжина: останнє числове перед ':' або перед портом
      int lastComma = header.lastIndexOf(',');
      String lastTok = (lastComma>=0) ? header.substring(lastComma+1) : header;
      lastTok = trim(lastTok);

      auto isNum = [](const String& s){
        if (!s.length()) return false;
        for (uint16_t i=0;i<s.length();++i) if (s[i]<'0'||s[i]>'9') return false;
        return true;
      };

      int dataLen = -1;
      if (isNum(lastTok)) {
        if (lastComma >= 0) {
          int prevComma = header.lastIndexOf(',', lastComma-1);
          String prevTok = (prevComma>=0) ? header.substring(prevComma+1, lastComma) : header.substring(0,lastComma);
          prevTok = trim(prevTok);
          dataLen = isNum(prevTok) ? prevTok.toInt() : lastTok.toInt();
        } else {
          dataLen = lastTok.toInt();
        }
      } else {
        int i = header.length()-1; while (i>=0 && (header[i]<'0'||header[i]>'9')) i--;
        int endNum = i; while (i>=0 && (header[i]>='0'&&header[i]<='9')) i--;
        int startNum = i+1;
        if (endNum >= startNum) dataLen = header.substring(startNum, endNum+1).toInt();
      }
      if (dataLen < 0) { acc.remove(0, colon+1); continue; }

      // 2) IP/порт (якщо є)
      String ip=""; uint16_t port=0;
      int dot = header.indexOf('.');
      if (dot >= 0) {
        int st = dot-1; while (st>=0 && ( (header[st]>='0'&&header[st]<='9') || header[st]=='.')) st--; st++;
        int en = dot+1; while (en<header.length() && ( (header[en]>='0'&&header[en]<='9') || header[en]=='.')) en++;
        ip = header.substring(st,en);
        int k = header.indexOf(',', en);
        if (k >= 0) {
          String portTok = trim(header.substring(k+1));
          int kk = portTok.lastIndexOf(','); if (kk>=0) portTok = trim(portTok.substring(kk+1));
          int j=0; while (j<(int)portTok.length() && isdigit(portTok[j])) j++;
          if (j>0) port = (uint16_t)portTok.substring(0,j).toInt();
        }
      }
      if (srcIp)   *srcIp   = ip;
      if (srcPort) *srcPort = port;

      // 3) дочитаємо рівно dataLen байтів тіла
      String payload; payload.reserve(dataLen+2);
      while ((int)payload.length() < dataLen) {
        int n = Serial2.available();
        if (n>0) { while (n-- && (int)payload.length()<dataLen) {
          char cc=(char)Serial2.read();
#if DEBUG_RAW
          Serial.write(cc);
#endif
          payload += cc;
        } }
      }
      out = payload;
      acc.remove(0, colon + 1 + dataLen);
      return true;
    }
  }
  return false;
}

bool udpSendTo(const String& ip, uint16_t port, const String& data){
  String c1 = String("AT+CIPSTART=0,\"UDP\",\"") + ip + "\"," + String(port);
  if (!atCmd(c1, "OK", 1200)) return false;
  String c2 = String("AT+CIPSEND=0,") + String(data.length());
  if (!atCmd(c2, ">", 1200)) return false;
  Serial2.print(data);
  return atCmd("", "SEND OK", 1500, false);
}

// =================== Wi-Fi / UDP init ===================
String getStaIp_(uint32_t total_ms) {
  String resp, ip=""; unsigned long t0 = millis();
  while (millis() - t0 < total_ms) {
    Serial2.print("AT+CIFSR\r\n");
    unsigned long t1 = millis();
    while (millis() - t1 < 600) { while (Serial2.available()) {
      char c=(char)Serial2.read();
#if DEBUG_RAW
      Serial.write(c);
#endif
      resp += c;
    } }
    int pos = resp.indexOf("STAIP,\"");
    if (pos >= 0) {
      int st = pos + 7, en = resp.indexOf("\"", st);
      if (en > st) { ip = resp.substring(st, en); break; }
    }
    delay(200);
  }
  if (resp.length()) Serial.println(resp);
  return ip;
}

bool wifiConnect(){
  Serial.println("\n[WiFi] Init AT link...");
  atCmd("ATE0");
  atCmd("AT");
  atCmd("AT+CWMODE=1");
  atCmd("AT+CIPRECVMODE=0");  // push mode (на всяк)
  // MUX=1
  Serial2.print("AT+CIPMUX?\r\n");
  String q; unsigned long t0 = millis();
  while (millis()-t0 < 800) { while (Serial2.available()) {
    char c=(char)Serial2.read();
#if DEBUG_RAW
    Serial.write(c);
#endif
    q += c;
  } }
  if (q.indexOf("+CIPMUX:1") < 0) {
    if (!atCmd("AT+CIPMUX=1", "OK", 1500)) { Serial.println("[WiFi] CIPMUX=1 FAIL"); return false; }
  }
  Serial.println("[WiFi] MUX = 1");
  atCmd("AT+CIPDINFO=1");

  Serial.print("[WiFi] Join "); Serial.println(WIFI_SSID);
  String cmd = String("AT+CWJAP=\"") + WIFI_SSID + "\",\"" + WIFI_PASS + "\"";
  if (!atCmd(cmd.c_str(), "WIFI GOT IP", 20000)) {
    if (!atCmd(cmd.c_str(), "OK", 20000)) return false;
  }

  String ip = getStaIp_();
  if (ip.length()) Serial.print("[WiFi] MEGA IP = "), Serial.println(ip);
  else             Serial.println("[WiFi] CIFSR: IP not found");

  return true;
}

bool udpStart(uint16_t port){
  // переконаємось у MUX=1
  Serial2.print("AT+CIPMUX?\r\n");
  String q; unsigned long t0 = millis();
  while (millis()-t0 < 500) { while (Serial2.available()) {
    char c=(char)Serial2.read();
#if DEBUG_RAW
    Serial.write(c);
#endif
    q += c;
  } }
  if (q.indexOf("+CIPMUX:1") < 0) {
    if (!atCmd("AT+CIPMUX=1", "OK", 1500)) { Serial.println("[UDP] Cannot set CIPMUX=1"); return false; }
  }

  atCmd("AT+CIPCLOSE=0", "OK", 500); // закрити попередній лінк (якщо був)
  char tmp[96];
  snprintf(tmp, sizeof(tmp), "AT+CIPSTART=0,\"UDP\",\"0.0.0.0\",0,%u,2", port);
  bool ok = atCmd(tmp, "OK", 4000) || atCmd("", "ALREADY CONNECTED", 500, false);
  if (!ok) ok = atCmd("", "CONNECT", 1200, false);
  if (!ok) return false;

  Serial.print("[UDP] Listening on "); Serial.println(port);
  return true;
}

// =================== РУХ / СЕРВО ===================
void driveLR(int L, int R){
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);
  if (L >= 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); }
  else        { digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH); }
  if (R >= 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); }
  else        { digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH); }
  analogWrite(L_EN, abs(L));
  analogWrite(R_EN, abs(R));
}
void allStop(){ analogWrite(L_EN,0); analogWrite(R_EN,0); }

void setJoint(const String& id, int ang){
  int a = constrain(ang,0,180);
  if(id=="root")      sRoot.write(a);
  else if(id=="a1")   sA1.write(a);
  else if(id=="a2")   sA2.write(a);
  else if(id=="armb") sB.write(a);
  else if(id=="wrista") sWA.write(a);
  else if(id=="wristb") sWB.write(a);
  else if(id=="grip")   sGrip.write(a);
}

// Єдиний парсер
void parseCommand(const String& s, const char* src){
  String t=s; t.trim(); t.toLowerCase();
  if (t.length()==0) return;

  if (t.startsWith("drv")) {
    int l=0,r=0; int li=t.indexOf("l="), ri=t.indexOf("r=");
    if (li>=0){ int sp=t.indexOf(' ',li); l=t.substring(li+2, sp>0?sp:t.length()).toInt(); }
    if (ri>=0){ int sp=t.indexOf(' ',ri); r=t.substring(ri+2, sp>0?sp:t.length()).toInt(); }
    driveLR(l,r);
    lastCmdTs = millis();
    Serial.print("[CMD "); Serial.print(src); Serial.print("] "); Serial.println(t);
  } else if (t.startsWith("srv")) {
    String id=""; int a=90;
    int idi=t.indexOf("id="), ai=t.indexOf("a=");
    if (idi>=0){ int sp=t.indexOf(' ',idi); id=t.substring(idi+3, sp>0?sp:t.length()); }
    if (ai>=0){ int sp=t.indexOf(' ',ai);   a =t.substring(ai+2, sp>0?sp:t.length()).toInt(); }
    setJoint(id,a);
    lastCmdTs = millis();
    Serial.print("[CMD "); Serial.print(src); Serial.print("] "); Serial.println(t);
  } else if (t.startsWith("stop")) {
    allStop();
    lastCmdTs = millis();
    Serial.print("[CMD "); Serial.print(src); Serial.print("] "); Serial.println(t);
  }
}

// =================== SETUP / LOOP ===================
void setup(){
  Serial.begin(115200);
  Serial.println("\nMEGA + ESP-01 (AT) — UDP 4210; діагностика увімкнена");

  pinMode(PULT_FORCE_PIN, INPUT_PULLUP); // LOW = примусовий пульт

  // Мотори
  pinMode(L_IN1,OUTPUT); pinMode(L_IN2,OUTPUT);
  pinMode(R_IN1,OUTPUT); pinMode(R_IN2,OUTPUT);
  pinMode(L_EN,OUTPUT);  pinMode(R_EN,OUTPUT);
  allStop();

  // Серво
  sRoot.attach(SERVO_ROOT_PIN);
  sA1.attach(SERVO_A1_PIN);
  sA2.attach(SERVO_A2_PIN);
  sB.attach(SERVO_ARMB_PIN);
  sWA.attach(SERVO_WA_PIN);
  sWB.attach(SERVO_WB_PIN);
  sGrip.attach(SERVO_GRIP_PIN);

  // Зв'язок із ESP-01
  Serial2.begin(115200);
  delay(400);

  if (!wifiConnect()) {
    Serial.println("[WiFi] FAIL — перевір SSID/PASS і живлення ESP-01.");
  } else {
    if (!udpStart(UDP_PORT)) {
      Serial.println("[UDP] start FAIL");
    } else {
      Serial.print("[READY] UDP waiting on "); Serial.println(UDP_PORT);
    }
  }

#if USE_NRF24
  if (!FORCE_NO_PULT) {
    if (radio.begin()) {
      radio.setPALevel(RF24_PA_LOW);
      radio.setDataRate(RF24_1MBPS);
      radio.openReadingPipe(1, PIPE_ADDR);
      radio.startListening();
      Serial.println("[NRF24] Ready (listening)");
    } else {
      Serial.println("[NRF24] Init FAIL");
    }
  }
#endif
}

void loop(){
  const bool pultForced = (digitalRead(PULT_FORCE_PIN) == LOW);
  bool radioActive = false;

#if USE_NRF24
  if (!FORCE_NO_PULT) {
    if (radio.available()) {
      char buf[64]={0}; radio.read(buf, sizeof(buf)-1);
      String s(buf); s.trim();
      if (s.length()) {
        parseCommand(s, "RADIO");
        lastRadioTs = millis();
      }
    }
    radioActive = (millis() - lastRadioTs) < RADIO_ACTIVE_MS;
  }
#endif

  // UDP прийом (лише якщо пульт не примусово та не активний, або FORCE_NO_PULT=1)
  if (FORCE_NO_PULT || (!pultForced && !radioActive)) {
    String pkt, ip; uint16_t port=0;
    if (atReadUdpPacket(pkt, &ip, &port, 50)) {
      Serial.print("[UDP "); 
      if (ip.length()) { Serial.print(ip); Serial.print(":"); Serial.print(port); }
      Serial.print("] "); Serial.println(pkt);
      parseCommand(pkt, "UDP");
    }
  }

  // Статус раз на 2с
  if (millis() - lastDbgTs > DBG_STATUS_EVERY_MS) {
    lastDbgTs = millis();
    Serial.print("[STAT] pultForced="); Serial.print(pultForced);
#if USE_NRF24
    Serial.print(" radioActive="); Serial.print(radioActive);
#else
    Serial.print(" radioActive=0");
#endif
    Serial.println(FORCE_NO_PULT ? " (FORCE_NO_PULT=1)" : "");
  }

  // Безпека: STOP якщо довго немає команд
  if (millis() - lastCmdTs > CMD_TIMEOUT_MS) {
    allStop();
  }
}
