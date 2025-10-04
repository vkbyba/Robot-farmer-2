/* ===== MEGA + ESP-01 (AT) — UDP control (single connection, CIPMUX=0) =====
 * Команди (рядки):
 *   "DRV L=X R=Y"          X,Y = -255..255
 *   "SRV ID=<root|a1|a2|armb|wrista|wristb|grip> A=N"
 *   "STOP"
 *
 * Підключення MEGA <-> ESP-01:
 *   MEGA TX2 (D16) --(дільник ~2k : 3.3k)--> ESP RX
 *   MEGA RX2 (D17) <------------------------- ESP TX (напряму)
 *   GND спільна; ЖИВЛЕННЯ ESP-01 через 3.3V адаптер
 *
 * ВАЖЛИВО (один раз у терміналі AT):
 *   ATE0
 *   AT+UART_DEF=115200,8,1,0,0
 *
 * Якщо не піднімається UDP — у коді є кілька варіантів CIPSTART, спробує кожен.
 * ========================================================================= */

#include <Servo.h>

// ---------- НАЛАШТУВАННЯ Wi-Fi ----------
const char* WIFI_SSID = "5 kv";
const char* WIFI_PASS = "Nz31032020";
const uint16_t UDP_PORT = 4210;

// ---------- ПІНИ МОТОРІВ (як на твоїй схемі) ----------
const int L_IN1=22, L_IN2=24, R_IN1=26, R_IN2=28;
const int L_EN=2,  R_EN=3;    // PWM

// ---------- ПІНИ СЕРВО (як на схемі) ----------
const int SERVO_ROOT_PIN = 12;
const int SERVO_A1_PIN   = 11;
const int SERVO_A2_PIN   = 10;
const int SERVO_ARMB_PIN = 9;
const int SERVO_WA_PIN   = 8;
const int SERVO_WB_PIN   = 7;
const int SERVO_GRIP_PIN = 6;

// ---------- СЕРВІСНІ НАЛАШТУВАННЯ ----------
#define DEBUG_AT   1     // друкувати усі відповіді ESP-01
#define DEBUG_RAW  0     // сирий UART потік (вимкнено, вмикай для діагностики)
const uint32_t CMD_TIMEOUT_MS = 3000;

// ---------- ГЛОБАЛЬНІ ----------
Servo sRoot, sA1, sA2, sB, sWA, sWB, sGrip;
uint32_t lastCmdTs = 0;

// ---------- ПРОТОТИПИ ----------
bool atCmd(const String& cmd, const char* ok="OK", uint32_t to=3000, bool echo=true);
String atReadAll(uint32_t ms);
String getStaIp_(uint32_t total_ms=4000);
bool wifiConnect();
bool udpStart(uint16_t port);
bool readUdpPacket(String& payload, uint32_t timeout_ms=30);
void parseCommand(const String& s, const char* src);
void driveLR(int L, int R);
void allStop();
void initServosHome();

// ========================= AT helpers =========================
bool atCmd(const String& cmd, const char* ok, uint32_t to, bool echo){
  if (cmd.length()) { Serial2.print(cmd); Serial2.print("\r\n"); if (echo) { Serial.print(">> "); Serial.println(cmd); } }
  String buf; buf.reserve(256);
  uint32_t t0=millis();
  while (millis()-t0 < to){
    while (Serial2.available()){
      char c=(char)Serial2.read();
#if DEBUG_RAW
      Serial.write(c);
#endif
      buf += c;
    }
    if (ok && buf.indexOf(ok)>=0) { if (DEBUG_AT) Serial.print(buf); return true; }
    if (buf.indexOf("ERROR")>=0 || buf.indexOf("FAIL")>=0) { if (DEBUG_AT) Serial.print(buf); return false; }
  }
  if (DEBUG_AT && buf.length()) Serial.print(buf);
  return (ok && buf.indexOf(ok)>=0);
}

String atReadAll(uint32_t ms){
  String s; uint32_t t0=millis();
  while (millis()-t0 < ms){
    while (Serial2.available()) {
      char c=(char)Serial2.read();
#if DEBUG_RAW
      Serial.write(c);
#endif
      s += c;
    }
  }
  if (DEBUG_AT && s.length()) Serial.print(s);
  return s;
}

String getStaIp_(uint32_t total_ms){
  String resp, ip=""; uint32_t t0=millis();
  while (millis()-t0 < total_ms){
    Serial2.print("AT+CIFSR\r\n");
    resp += atReadAll(500);
    int pos = resp.indexOf("STAIP,\"");
    if (pos>=0){ int st=pos+7, en=resp.indexOf("\"", st); if (en>st){ ip=resp.substring(st,en); break; } }
    delay(150);
  }
  return ip;
}

// ========================= Wi-Fi / UDP =========================
bool wifiConnect(){
  Serial.println("\n[WiFi] Init AT link...");
  atCmd("ATE0", "OK", 1200);
  atCmd("AT", "OK", 1200);
  atCmd("AT+CWMODE=1", "OK", 1500);
  atCmd("AT+CIPRECVMODE=0", "OK", 1200);  // push mode
  // фіксуємо single connection
  atCmd("AT+CIPMUX=0", "OK", 1500);

  Serial.print("[WiFi] Join "); Serial.println(WIFI_SSID);
  String j = String("AT+CWJAP=\"")+WIFI_SSID+"\",\""+WIFI_PASS+"\"";
  if (!atCmd(j, "WIFI GOT IP", 22000)) { // не всі прошивки друкують WIFI GOT IP
    if (!atCmd(j, "OK", 22000)) return false;
  }
  String ip = getStaIp_();
  if (ip.length()) Serial.print("[WiFi] MEGA IP = "), Serial.println(ip);
  else             Serial.println("[WiFi] IP not found (CIFSR)");
  return true;
}

// Спробуємо кілька синтаксисів для single-UDP “серверу”
bool udpStart(uint16_t port){
  // закриємо, якщо було щось відкрите
  atCmd("AT+CIPCLOSE", "OK", 800);

  // ВАРІАНТ 1 (частіше працює в single): UDP, "0.0.0.0", <local>, 2
  char cmd1[64]; snprintf(cmd1,sizeof(cmd1),"AT+CIPSTART=\"UDP\",\"0.0.0.0\",%u,2", port);
  if (atCmd(cmd1, "OK", 4000) || atCmd("", "ALREADY CONNECTED", 800, false)) {
    Serial.print("[UDP] Listening on "); Serial.println(port);
    return true;
  }

  // ВАРІАНТ 2: UDP, "0", 0, <local>
  char cmd2[64]; snprintf(cmd2,sizeof(cmd2),"AT+CIPSTART=\"UDP\",\"0\",0,%u", port);
  if (atCmd(cmd2, "OK", 4000) || atCmd("", "ALREADY CONNECTED", 800, false)) {
    Serial.print("[UDP] Listening on "); Serial.println(port);
    return true;
  }

  // ВАРІАНТ 3: UDP, "", 0, <local>
  char cmd3[64]; snprintf(cmd3,sizeof(cmd3),"AT+CIPSTART=\"UDP\",\"\",0,%u", port);
  if (atCmd(cmd3, "OK", 4000) || atCmd("", "ALREADY CONNECTED", 800, false)) {
    Serial.print("[UDP] Listening on "); Serial.println(port);
    return true;
  }

  Serial.println("[UDP] start FAIL");
  return false;
}

// читаємо один UDP-пакет у CIPMUX=0: формати +IPD,<len>:payload  або +IPD,<id>,<len>:payload
bool readUdpPacket(String& payload, uint32_t timeout_ms){
  static String acc;
  uint32_t t0 = millis();

  auto getIntFromEnd = [](const String& s)->int{
    int i=s.length()-1; while(i>=0 && (s[i]<'0'||s[i]>'9')) i--;
    int e=i; while(i>=0 && (s[i]>='0'&&s[i]<='9')) i--;
    if (e>=i+1) return s.substring(i+1,e+1).toInt();
    return -1;
  };

  while (millis()-t0 < timeout_ms){
    while (Serial2.available()){
      char c=(char)Serial2.read();
#if DEBUG_RAW
      Serial.write(c);
#endif
      acc += c;

      int p = acc.indexOf("+IPD,");
      if (p<0) { if ((int)acc.length()>1024) acc.remove(0,512); continue; }

      int colon = acc.indexOf(':', p+5);
      if (colon<0) continue;

      String header = acc.substring(p+5, colon); header.trim();
      int len = getIntFromEnd(header);
      if (len <= 0) { acc.remove(0, colon+1); continue; }

      // дочитати рівно len байтів
      String data; data.reserve(len+2);
      while ((int)data.length() < len){
        int n = Serial2.available();
        if (n>0) while(n-- && (int)data.length()<len) {
          char cc=(char)Serial2.read();
#if DEBUG_RAW
          Serial.write(cc);
#endif
          data += cc;
        }
      }
      payload = data;
      acc.remove(0, colon+1+len);
      return true;
    }
  }
  return false;
}

// ========================= РУХ / СЕРВО =========================
void driveLR(int L, int R){
  L = constrain(L,-255,255); R = constrain(R,-255,255);
  if (L>=0){ digitalWrite(L_IN1,HIGH); digitalWrite(L_IN2,LOW); }  else { digitalWrite(L_IN1,LOW); digitalWrite(L_IN2,HIGH); }
  if (R>=0){ digitalWrite(R_IN1,HIGH); digitalWrite(R_IN2,LOW); }  else { digitalWrite(R_IN1,LOW); digitalWrite(R_IN2,HIGH); }
  analogWrite(L_EN,abs(L)); analogWrite(R_EN,abs(R));
}
void allStop(){ analogWrite(L_EN,0); analogWrite(R_EN,0); }

void initServosHome(){
  sRoot.write(90);
  sA1.write(90);
  sA2.write(90);
  sB.write(90);
  sWA.write(90);
  sWB.write(90);
  sGrip.write(90);
}

// Єдиний парсер команд
void parseCommand(const String& s, const char* src){
  String t=s; t.trim(); t.toLowerCase();
  if (!t.length()) return;

  if (t.startsWith("drv")){
    int l=0,r=0; int li=t.indexOf("l="), ri=t.indexOf("r=");
    if (li>=0){ int sp=t.indexOf(' ',li); l=t.substring(li+2, sp>0?sp:t.length()).toInt(); }
    if (ri>=0){ int sp=t.indexOf(' ',ri); r=t.substring(ri+2, sp>0?sp:t.length()).toInt(); }
    driveLR(l,r);
    lastCmdTs = millis();
    Serial.print("[CMD "); Serial.print(src); Serial.print("] "); Serial.println(t);
  } else if (t.startsWith("srv")){
    String id=""; int a=90;
    int idi=t.indexOf("id="), ai=t.indexOf("a=");
    if (idi>=0){ int sp=t.indexOf(' ',idi); id=t.substring(idi+3, sp>0?sp:t.length()); }
    if (ai>=0){ int sp=t.indexOf(' ',ai);   a =t.substring(ai+2, sp>0?sp:t.length()).toInt(); }
    a = constrain(a,0,180);
         if (id=="root")   sRoot.write(a);
    else if (id=="a1")     sA1.write(a);
    else if (id=="a2")     sA2.write(a);
    else if (id=="armb")   sB.write(a);
    else if (id=="wrista") sWA.write(a);
    else if (id=="wristb") sWB.write(a);
    else if (id=="grip")   sGrip.write(a);
    lastCmdTs = millis();
    Serial.print("[CMD "); Serial.print(src); Serial.print("] "); Serial.println(t);
  } else if (t.startsWith("stop")){
    allStop();
    lastCmdTs = millis();
    Serial.print("[CMD "); Serial.print(src); Serial.print("] "); Serial.println(t);
  }
}

// ========================= SETUP / LOOP =========================
void setup(){
  Serial.begin(115200);
  Serial.println(F("\nMEGA + ESP-01 (AT) — UDP single (CIPMUX=0)"));

  // Мотори
  pinMode(L_IN1,OUTPUT); pinMode(L_IN2,OUTPUT);
  pinMode(R_IN1,OUTPUT); pinMode(R_IN2,OUTPUT);
  pinMode(L_EN,OUTPUT);  pinMode(R_EN,OUTPUT);
  allStop();

  // Серво (attach і базові кути, щоб не смикались)
  sRoot.attach(SERVO_ROOT_PIN);
  sA1.attach(SERVO_A1_PIN);
  sA2.attach(SERVO_A2_PIN);
  sB.attach(SERVO_ARMB_PIN);
  sWA.attach(SERVO_WA_PIN);
  sWB.attach(SERVO_WB_PIN);
  sGrip.attach(SERVO_GRIP_PIN);
  delay(150);
  initServosHome();

  // ESP-01 лінк
  Serial2.begin(115200);
  delay(400);

  if (!wifiConnect()) {
    Serial.println("[WiFi] FAIL — перевір SSID/PASS і живлення ESP-01.");
  } else if (!udpStart(UDP_PORT)) {
    Serial.println("[UDP] FAIL — не вдалось відкрити локальний порт.");
  } else {
    Serial.print("[READY] UDP waiting on "); Serial.println(UDP_PORT);
  }
}

void loop(){
  // Прийом UDP
  String pkt;
  if (readUdpPacket(pkt, 30)){
    Serial.print("[UDP] "); Serial.println(pkt);
    parseCommand(pkt, "UDP");
  }

  // Безпека: авто-STOP, якщо довго тиша
  if (millis() - lastCmdTs > CMD_TIMEOUT_MS) {
    allStop();
  }
}
