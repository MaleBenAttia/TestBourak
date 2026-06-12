#ifndef COAP_TELEMETRY_H
#define COAP_TELEMETRY_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//  CONFIG RÉSEAU
// ══════════════════════════════════════════════════════════════
#define COAP_SERVER_IP    "192.168.100.14"   // ← IP de ton PC (ipconfig)
#define COAP_SERVER_PORT  5683
#define WIFI_SSID         "TUNISIETELECOM-4G-WWc7"
#define WIFI_PASS         "25487077"

#define TELEM_INTERVAL_MS  10   // 100Hz

static WiFiUDP   _udp;
static uint16_t  _msg_id  = 0;
static uint32_t  _last_telem_ms = 0;

static void _coapSendRaw(const char* path, const char* payload, int pl_len) {
  if (WiFi.status() != WL_CONNECTED) return;
  const uint8_t path_len = strlen(path);
  uint8_t pkt[4 + 1 + path_len + 1 + pl_len];
  uint8_t i = 0;
  _msg_id++;
  pkt[i++] = 0x50;
  pkt[i++] = 0x02;
  pkt[i++] = (_msg_id >> 8) & 0xFF;
  pkt[i++] = _msg_id & 0xFF;
  pkt[i++] = (11 << 4) | path_len;
  memcpy(&pkt[i], path, path_len); i += path_len;
  pkt[i++] = 0xFF;
  memcpy(&pkt[i], payload, pl_len); i += pl_len;
  _udp.beginPacket(COAP_SERVER_IP, COAP_SERVER_PORT);
  _udp.write(pkt, i);
  _udp.endPacket();
}

void coapInit() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[CoAP] WiFi...");
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
    delay(200); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\n[CoAP] OK -> " + WiFi.localIP().toString());
  else
    Serial.println("\n[CoAP] WiFi ECHEC (robot continue quand meme)");
  _udp.begin(0);
}

void coapSendConfig(float Kp_o, float Kd_o, float Ki_o,
                    float Kp_i, float Ki_i,
                    float maxRPM, float minRPM, float maxCor,
                    int tpt, float circ) {
  char pl[256];
  int pl_len = snprintf(pl, sizeof(pl),
    "{\"Kp_o\":%.5f,\"Kd_o\":%.5f,\"Ki_o\":%.6f,"
    "\"Kp_i\":%.5f,\"Ki_i\":%.5f,"
    "\"maxRPM\":%.1f,\"minRPM\":%.1f,\"maxCor\":%.1f,"
    "\"tpt\":%d,\"circ\":%.5f}",
    Kp_o, Kd_o, Ki_o, Kp_i, Ki_i,
    maxRPM, minRPM, maxCor, tpt, circ);
  _coapSendRaw("config", pl, pl_len);
  Serial.println("[CoAP] Config envoyee");
}

void coapSendTelem(float masafa,
                   int err, float cor,
                   float rpmL, float rpmR,
                   float tgL, float tgR,
                   int pwmL, int pwmR,
                   float base_rpm) {
  uint32_t now = millis();
  if (now - _last_telem_ms < TELEM_INTERVAL_MS) return;
  _last_telem_ms = now;
  char pl[256];
  int pl_len = snprintf(pl, sizeof(pl),
    "{\"mas\":%.4f,"
    "\"err\":%d,\"cor\":%.2f,"
    "\"rpmL\":%.1f,\"rpmR\":%.1f,"
    "\"tgL\":%.1f,\"tgR\":%.1f,"
    "\"pwmL\":%d,\"pwmR\":%d,"
    "\"base\":%.1f}",
    masafa, err, cor,
    rpmL, rpmR, tgL, tgR,
    pwmL, pwmR, base_rpm);
  _coapSendRaw("telemetry", pl, pl_len);
}
void coapSendMPU(float masafa, float angle, float target,
                 float error, float correction,
                 int pwmR, int pwmL) {
  uint32_t now = millis();
  if (now - _last_telem_ms < TELEM_INTERVAL_MS) return;
  _last_telem_ms = now;
  char pl[256];
  int pl_len = snprintf(pl, sizeof(pl),
    "{\"mas\":%.2f,"
    "\"ang\":%.2f,\"tgt\":%.2f,"
    "\"err\":%.2f,\"cor\":%.2f,"
    "\"pwmR\":%d,\"pwmL\":%d}",
    masafa, angle, target,
    error, correction,
    pwmR, pwmL);
  _coapSendRaw("mpu", pl, pl_len);
}
void coapSendVirage(uint32_t ts, float angle, long tL, long tR,
                    float rl, float rr, int pL, int pR, float err) {
    uint32_t now = millis();
    if (now - _last_telem_ms < TELEM_INTERVAL_MS) return;
    _last_telem_ms = now;
    char pl[256];
    int pl_len = snprintf(pl, sizeof(pl),
        "{\"ts\":%lu,\"ang\":%.2f,\"tL\":%ld,\"tR\":%ld,"
        "\"rpmL\":%.1f,\"rpmR\":%.1f,\"pwmL\":%d,\"pwmR\":%d,\"err\":%.2f}",
        ts, angle, tL, tR, rl, rr, pL, pR, err);
    _coapSendRaw("virage", pl, pl_len);
}
#endif
