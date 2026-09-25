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

#define TELEM_INTERVAL_MS  10   // 100Hz — fréquence max d'envoi de télémétrie

static WiFiUDP   _udp;
static uint16_t  _msg_id  = 0;
static uint32_t  _last_telem_ms = 0;

// ── ENVOI BAS NIVEAU (interne) ──────────────────────────────────
static void _coapSendRaw(
    const char* path,     // nom du "topic" CoAP, ex: "telemetry", "rpm", "virage"
    const char* payload,  // corps JSON déjà construit (string)
    int pl_len)            // longueur du payload en octets
{
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

// ── CONNEXION WIFI ───────────────────────────────────────────────
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

// ── ENVOI DES CONSTANTES PID (config) ────────────────────────────
void coapSendConfig(
    float Kp_o,    // gain proportionnel boucle EXTERNE (position ligne → RPM cible)
    float Kd_o,    // gain dérivé boucle externe
    float Ki_o,    // gain intégral boucle externe
    float Kp_i,    // gain proportionnel boucle INTERNE (RPM → PWM)
    float Ki_i,    // gain intégral boucle interne
    float maxRPM,  // RPM maximum autorisé (saturation haute du setpoint)
    float minRPM,  // RPM minimum autorisé (saturation basse du setpoint)
    float maxCor,  // correction maximale appliquée par la boucle externe
    int   tpt,     // ticks par tour de roue (constante encodeur/mécanique)
    float circ)    // circonférence de la roue (cm), sert au calcul de distance
{
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

// ── TELEMETRIE LIGNE (PID cascade complet) ───────────────────────
void coapSendTelem(
    float masafa,    // distance parcourue depuis le dernier reset (cm)
    int   err,       // erreur de position ligne (capteur), ex: position - setpoint
    float cor,       // correction calculée par la boucle externe (delta RPM L/R)
    float rpmL,      // RPM RÉEL mesuré roue gauche (via encodeur)
    float rpmR,      // RPM RÉEL mesuré roue droite
    float tgL,       // RPM CIBLE (target/setpoint) roue gauche, sortie boucle externe
    float tgR,       // RPM CIBLE roue droite
    int   pwmL,      // PWM final appliqué au moteur gauche (sortie boucle interne)
    int   pwmR,      // PWM final appliqué au moteur droit
    float base_rpm)  // vitesse de base commune (avant correction ligne)
{
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

// ── TELEMETRIE MPU (PID d'angle gyro) ────────────────────────────
void coapSendMPU(
    float masafa,      // distance parcourue depuis le dernier reset (cm)
    float angle,       // angle actuel mesuré par le MPU (degrés)
    float target,      // angle cible à atteindre/maintenir (degrés)
    float error,       // erreur d'angle = target - angle
    float correction,  // correction PID calculée à partir de l'erreur d'angle
    int   pwmR,        // PWM appliqué au moteur droit résultant de la correction
    int   pwmL)        // PWM appliqué au moteur gauche résultant de la correction
{
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

// ── TELEMETRIE VIRAGE (rotation sur angle) ───────────────────────
void coapSendVirage(
    uint32_t ts,    // timestamp (millis()) au moment de l'envoi
    float angle,    // angle actuel mesuré pendant la rotation (degrés)
    long  tL,       // ticks bruts encodeur gauche (compteur brut, pas delta)
    long  tR,       // ticks bruts encodeur droit
    float rl,       // RPM réel roue gauche pendant le virage
    float rr,       // RPM réel roue droite pendant le virage
    int   pL,       // PWM appliqué moteur gauche
    int   pR,       // PWM appliqué moteur droit
    float err)      // erreur d'angle restante (angleCible - angle actuel)
{
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

// ── TELEMETRIE RPM/PWM SEULE (légère) ────────────────────────────
void coapSendRPM(
    float rpmL,  // RPM réel mesuré roue gauche
    float rpmR,  // RPM réel mesuré roue droite
    int   pwmL,  // PWM appliqué moteur gauche
    int   pwmR)  // PWM appliqué moteur droit
{
  uint32_t now = millis();
  if (now - _last_telem_ms < TELEM_INTERVAL_MS) return;
  _last_telem_ms = now;
  char pl[128];
  int pl_len = snprintf(pl, sizeof(pl),
    "{\"rpmL\":%.1f,\"rpmR\":%.1f,"
    "\"pwmL\":%d,\"pwmR\":%d}",
    rpmL, rpmR, pwmL, pwmR);
  _coapSendRaw("rpm", pl, pl_len);
}

#endif