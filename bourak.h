#ifndef BOURAK_H
#define BOURAK_H

#include <U8g2lib.h>
#include <Wire.h>
#include "ElBourak.h"
#include "encoder.h"
#include "coap_telemetry.h"
// ═══════════════════════════════════════════════════════════════
//  CAPTEURS IR — MUX
// ═══════════════════════════════════════════════════════════════
#define S0 25
#define S1 26
#define S2 27
#define S3 14
#define SIG 36

#define NB_CAPT 16
#define SETPOINT 7500

ElBourak pid(NB_CAPT, SETPOINT, SIG, S0, S1, S2, S3);

// ═══════════════════════════════════════════════════════════════
//  MOTEURS — BTS7960
// ═══════════════════════════════════════════════════════════════
#define MOT_R_PWM1 18
#define MOT_R_PWM2 19
#define MOT_L_PWM1 5
#define MOT_L_PWM2 17

// ═══════════════════════════════════════════════════════════════
//  LED + BOUTON — GPIO2 polyvalent
//  INPUT  → lire le bouton
//  OUTPUT → allumer la LED
// ═══════════════════════════════════════════════════════════════
#define LED_BTN 2
#define LED_BTNIn digitalRead(LED_BTN)
#define LEDON digitalWrite(LED_BTN, 1)
#define LEDOFF digitalWrite(LED_BTN, 0)
// ═══════════════════════════════════════════════════════════════
//  OLED
// ═══════════════════════════════════════════════════════════════
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 21, 22);
bool oledActif = true;

// ═══════════════════════════════════════════════════════════════
//  MPU6050
// ═══════════════════════════════════════════════════════════════
#define I2CMPU Wire

int16_t accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
int16_t gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

unsigned long lastMicros = 0;
float angleZ = 0;
float gyroBiasZ = 0;
float rotZFiltered = 0;
float ang = 0, ang1 = 0;
long t = 0, t1 = 0, t2 = 0;

// ═══════════════════════════════════════════════════════════════
//  PID ANCIEN — constantes (compatibilité)
// ═══════════════════════════════════════════════════════════════
const int ConstantCount_old = 4;
static const float Kp_old[4] = { 0.10f, 0.15f, 0.065f, 0.0f };
static const float Ki_old[4] = { 0.0005f, 0.0005f, 0.0003f, 0.0f };
static const float Kd_old[4] = { 0.21f, 0.19f, 0.32f, 0.0f };

int maxspeeda = 240, maxspeedb = 240;
int basespeeda = 225, basespeedb = 225;
int P, I, D;
int lastError = 0, lastError2 = 0;
long I_accumulated = 0;

// ═══════════════════════════════════════════════════════════════
//  PID CASCADE — profils 7 cases
//50 pwm ----- 184.5rpm
//100pwm-----397.4
//150pwm---594
//200---765.4
//255-950
// ═══════════════════════════════════════════════════════════════
const int ProfileCount = 7;

float base_rpm[ProfileCount] = { 60.0f, 180.0f, 320.0f, 460.0f, 600.0f, 760.0f, 940.0f };
float Kp_outer[ProfileCount] = { 0.09f, 0.08f, 0.08f, 0.08f, 0.07f, 0.07f, 0.06f };
float Ki_outer[ProfileCount] = { 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f };
float Kd_outer[ProfileCount] = { 0.65f, 0.55f, 0.55f, 0.55f, 0.58f, 0.60f, 0.65f };
float Kp_inner[ProfileCount] = { 0.05f, 0.07f, 0.05f, 0.05f, 0.05f, 0.04f, 0.04f };
float Ki_inner[ProfileCount] = { 0.028f, 0.024f, 0.024f, 0.024f, 0.022f, 0.020f, 0.018f };
float MAX_COR[ProfileCount] = { 500.0f, 500.0f, 500.0f, 500.0f, 600.0f, 700.0f, 800.0f };

float MAX_RPM = 950.0f;
float MIN_RPM = 150.0f;
int activeProfile = 2;

// Variables internes boucle externe
int lastError_outer = 0;
int lastError2_outer = 0;
long I_outer = 0;

// Variables internes boucle interne
float integL = 0.0f;
float integR = 0.0f;

// Variables télémétrie
int g_motorspeeda = 0;
int g_motorspeedb = 0;
int g_correction = 0;
int g_position = 0;

// RPM
float rpmL = 0.0f;
float rpmR = 0.0f;
static long _prevTicksL = 0, _prevTicksR = 0;
static uint32_t _prevTimeL = 0, _prevTimeR = 0;

// Compteurs
int cAll = 0, cL = 0, cR = 0;

// ═══════════════════════════════════════════════════════════════
//  MPU PID
// ═══════════════════════════════════════════════════════════════
float Kp_mpu = 9.0f, Ki_mpu = 0.005f, Kd_mpu = 3.0f;
float mpuError = 0, mpuIntegral = 0, mpuLastError = 0;
float targetAngle = 0;
bool mpuInitialized = false;
int baseSpeed = 140, maxSpeed = 220, currentSpeed = 0;

// ═══════════════════════════════════════════════════════════════
//  MOTEURS — INIT + FAST FORWARD
// ═══════════════════════════════════════════════════════════════

// Précalcul des canaux PWM LEDC pour BTS7960
// 4 canaux : R_PWM1, R_PWM2, L_PWM1, L_PWM2
#define CH_R1 0
#define CH_R2 1
#define CH_L1 2
#define CH_L2 3
#define PWM_FREQ 20000
#define PWM_BITS 8

void initMotors() {
  ledcAttach(MOT_R_PWM1, PWM_FREQ, PWM_BITS);
  ledcAttach(MOT_R_PWM2, PWM_FREQ, PWM_BITS);
  ledcAttach(MOT_L_PWM1, PWM_FREQ, PWM_BITS);
  ledcAttach(MOT_L_PWM2, PWM_FREQ, PWM_BITS);
}

// ⚡ BTS7960 ultra-rapide — ledcWrite direct, pas de digitalRead
// pwmR > 0 = avant,  pwmR < 0 = arrière
// pwmL > 0 = avant,  pwmL < 0 = arrière
inline void forward_brake_fast(int pwmR, int pwmL) {
  if (pwmR > 0) {
    ledcWrite(MOT_R_PWM1, (uint32_t)pwmR);
    ledcWrite(MOT_R_PWM2, 0);
  } else if (pwmR < 0) {
    ledcWrite(MOT_R_PWM1, 0);
    ledcWrite(MOT_R_PWM2, (uint32_t)(-pwmR));
  } else {
    ledcWrite(MOT_R_PWM1, 0);
    ledcWrite(MOT_R_PWM2, 0);
  }
  if (pwmL > 0) {
    ledcWrite(MOT_L_PWM1, (uint32_t)pwmL);
    ledcWrite(MOT_L_PWM2, 0);
  } else if (pwmL < 0) {
    ledcWrite(MOT_L_PWM1, 0);
    ledcWrite(MOT_L_PWM2, (uint32_t)(-pwmL));
  } else {
    ledcWrite(MOT_L_PWM1, 0);
    ledcWrite(MOT_L_PWM2, 0);
  }
}

inline void stopMotors() {
  ledcWrite(MOT_R_PWM1, 0);
  ledcWrite(MOT_R_PWM2, 0);
  ledcWrite(MOT_L_PWM1, 0);
  ledcWrite(MOT_L_PWM2, 0);
}

// ═══════════════════════════════════════════════════════════════
//  OLED — FONCTIONS
// ═══════════════════════════════════════════════════════════════
void afficherTexte(String message, byte style) {
  u8g2.firstPage();
  do {
    switch (style) {
      case 1:
        u8g2.setFont(u8g2_font_10x20_tr);
        u8g2.drawStr(0, 35, message.c_str());
        break;
      case 2:
        u8g2.setFont(u8g2_font_helvR12_te);
        u8g2.drawStr(0, 40, message.c_str());
        break;
      case 3:
        u8g2.setFont(u8g2_font_8x13_tr);
        u8g2.drawFrame(0, 10, 128, 44);
        u8g2.drawStr(5, 35, message.c_str());
        break;
      default: u8g2.setFont(u8g2_font_8x13_tr); u8g2.drawStr(0, 32, message.c_str());
    }
  } while (u8g2.nextPage());
}

void afficherValeur(double valeur, byte style) {
  char buffer[16];
  dtostrf(valeur, 1, 2, buffer);
  u8g2.firstPage();
  do {
    switch (style) {
      case 1:
        u8g2.setFont(u8g2_font_10x20_tr);
        u8g2.drawStr(0, 35, buffer);
        break;
      case 2:
        u8g2.setFont(u8g2_font_helvR12_te);
        u8g2.drawStr(0, 40, buffer);
        break;
      case 3:
        u8g2.setFont(u8g2_font_8x13_tr);
        u8g2.drawFrame(0, 10, 128, 44);
        u8g2.drawStr(5, 35, buffer);
        break;
      default: u8g2.setFont(u8g2_font_8x13_tr); u8g2.drawStr(0, 32, buffer);
    }
  } while (u8g2.nextPage());
}

inline void afficherTexte1(float v, byte s) {
  afficherValeur(v, s);
}
inline void afficherTexte2(long v, byte s) {
  afficherValeur((double)v, s);
}
void eteindreOLED() {
  u8g2.setPowerSave(true);
  oledActif = false;
}
void allumerOLED() {
  u8g2.setPowerSave(false);
  oledActif = true;
}

void afficherEncodeurs() {
  long tL = TICKS_L;
  long tR = TICKS_R;
  float dist = MASAFA;
  char buf[24];
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tr);
    snprintf(buf, sizeof(buf), "L: %ld", tL);
    u8g2.drawStr(0, 15, buf);
    snprintf(buf, sizeof(buf), "R: %ld", tR);
    u8g2.drawStr(0, 33, buf);
    snprintf(buf, sizeof(buf), "D:%.2f cm", dist);
    u8g2.drawStr(0, 51, buf);
  } while (u8g2.nextPage());
}

// ═══════════════════════════════════════════════════════════════
//  MPU — FONCTIONS
// ═══════════════════════════════════════════════════════════════
void setupMPU() {
  I2CMPU.beginTransmission(0x68);
  I2CMPU.write(0x6B);
  I2CMPU.write(0x00);
  I2CMPU.endTransmission();
  I2CMPU.beginTransmission(0x68);
  I2CMPU.write(0x1B);
  I2CMPU.write(0x08);
  I2CMPU.endTransmission();
  I2CMPU.beginTransmission(0x68);
  I2CMPU.write(0x1C);
  I2CMPU.write(0x00);
  I2CMPU.endTransmission();
}

void recordMPURegisters(uint8_t startReg, uint8_t numBytes, int16_t* dX, int16_t* dY, int16_t* dZ) {
  I2CMPU.beginTransmission(0x68);
  I2CMPU.write(startReg);
  I2CMPU.endTransmission();
  I2CMPU.requestFrom(0x68, numBytes);
  unsigned long timeout = micros() + 1000;
  while (I2CMPU.available() < numBytes && micros() < timeout)
    ;
  if (I2CMPU.available() >= numBytes) {
    *dX = I2CMPU.read() << 8 | I2CMPU.read();
    *dY = I2CMPU.read() << 8 | I2CMPU.read();
    *dZ = I2CMPU.read() << 8 | I2CMPU.read();
  }
}

inline void processAccelData() {
  gForceX = accelX / 16384.0f;
  gForceY = accelY / 16384.0f;
  gForceZ = accelZ / 16384.0f;
}
inline void processGyroData() {
  rotX = gyroX / 65.5f;
  rotY = gyroY / 65.5f;
  rotZ = gyroZ / 65.5f;
}
void recordAccelRegisters() {
  recordMPURegisters(0x3B, 6, &accelX, &accelY, &accelZ);
  processAccelData();
}
void recordGyroRegisters() {
  recordMPURegisters(0x43, 6, &gyroX, &gyroY, &gyroZ);
  processGyroData();
}

void calibrateGyro() {
  long sumZ = 0;
  afficherTexte("Calibration...", 1);
  for (int i = 0; i < 1000; i++) {
    recordGyroRegisters();
    sumZ += gyroZ;
    delayMicroseconds(900);
  }
  gyroBiasZ = (float)sumZ / 1000.0f;
  lastMicros = micros();
  angleZ = 0;
  afficherTexte("Pret!", 2);
  delay(500);
}

void calcANG() {
  unsigned long now = micros();
  if (lastMicros == 0) {
    lastMicros = now;
    return;
  }
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;
  recordGyroRegisters();
  float rawZ = gyroZ - gyroBiasZ;
  if (abs(rawZ) < 40) rawZ = 0;
  rotZ = rawZ / 65.5f;
  angleZ += rotZ * dt;
  angleZ = fmod(angleZ, 360.0f);
  if (angleZ < 0) angleZ += 360.0f;
}

// ═══════════════════════════════════════════════════════════════
//  MPU PID — FONCTIONS
// ═══════════════════════════════════════════════════════════════
void initMPU_PID(int base, int maxSp, float angleOffset) {
  calcANG();
  targetAngle = angleZ + angleOffset;
  mpuIntegral = 0;
  mpuLastError = 0;
  baseSpeed = constrain(base, 0, 255);
  maxSpeed = constrain(maxSp, 0, 255);
  currentSpeed = baseSpeed;
  mpuInitialized = true;
}

void runMPU_PID() {
  if (!mpuInitialized) return;
  calcANG();
  mpuError = angleZ - targetAngle;
  if (mpuError > 180) mpuError -= 360;
  if (mpuError < -180) mpuError += 360;
  float Pterm = Kp_mpu * mpuError;
  mpuIntegral += mpuError;
  mpuIntegral = constrain(mpuIntegral, -400, 400);
  float Iterm = Ki_mpu * mpuIntegral;
  float Dterm = Kd_mpu * (mpuError - mpuLastError);
  mpuLastError = mpuError;
  float correction = constrain(Pterm + Iterm + Dterm, -180, 180);
  int sR = constrain(baseSpeed + (int)correction, 0, 255);
  int sL = constrain(baseSpeed - (int)correction, 0, 255);
  forward_brake_fast(sR, sL);
}

void stopMPU_PID() {
  stopMotors();
  mpuInitialized = false;
  currentSpeed = 0;
}
void setMPU_PID(float kp, float ki, float kd) {
  Kp_mpu = kp;
  Ki_mpu = ki;
  Kd_mpu = kd;
}

void gyroTurnPID(float angle, int maxSpd, float tolerance) {
  float Kp_t = 2.7f, Kd_t = 0.03f;
  float error, prevError = 0, output;
  calcANG();
  float tgtAngle = fmod(angleZ + angle + 360.0f, 360.0f);
  unsigned long lastTime = millis();
  while (true) {
    calcANG();
    error = tgtAngle - angleZ;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    if (abs(error) <= tolerance) break;
    unsigned long now = millis();
    float dt = max((now - lastTime) / 1000.0f, 0.001f);
    lastTime = now;
    float deriv = (error - prevError) / dt;
    prevError = error;
    output = constrain(Kp_t * error + Kd_t * deriv, -maxSpd, maxSpd);
    forward_brake_fast((int)output, -(int)output);
  }
  stopMotors();
}

// ═══════════════════════════════════════════════════════════════
//  PID ANCIEN — compatibilité
// ═══════════════════════════════════════════════════════════════
void PID_control_fast(int idx, int st, bool whiteLine) {
  int position = whiteLine ? pid.ReadLineWhiteFast() : pid.ReadLineBlackFast();
  int error = st - position;
  int d = ((error - lastError) + (lastError - lastError2)) >> 1;
  lastError2 = lastError;
  lastError = error;
  I_accumulated += error;
  if ((error > 0 && I_accumulated < 0) || (error < 0 && I_accumulated > 0)) I_accumulated >>= 1;
  I_accumulated = constrain(I_accumulated, -100000, 100000);
  I = I_accumulated / 100;
  P = error;
  D = d;
  int motorspeed = (int)(P * Kp_old[idx] + I * Ki_old[idx] + D * Kd_old[idx]);
  int mA = constrain(basespeeda + motorspeed, -80, maxspeeda);
  int mB = constrain(basespeedb - motorspeed, -80, maxspeedb);
  forward_brake_fast(mA, mB);
}
inline void PID_controlB_fast(int idx, int st) {
  PID_control_fast(idx, st, false);
}
inline void PID_controlW_fast(int idx, int st) {
  PID_control_fast(idx, st, true);
}

// ═══════════════════════════════════════════════════════════════
//  UPDATE RPM
// ═══════════════════════════════════════════════════════════════
void updateRPM() {
  uint32_t now = micros();
  float dtL = (now - _prevTimeL) / 1000000.0f;
  if (dtL >= 0.005f) {
    long tL = readEncoderL();
    rpmL = ((float)(tL - _prevTicksL) / dtL) / TICKS_PAR_TOUR_ROUE * 60.0f;
    _prevTicksL = tL;
    _prevTimeL = now;
  }
  float dtR = (now - _prevTimeR) / 1000000.0f;
  if (dtR >= 0.005f) {
    long tR = readEncoderR();
    rpmR = ((float)(tR - _prevTicksR) / dtR) / TICKS_PAR_TOUR_ROUE * 60.0f;
    _prevTicksR = tR;
    _prevTimeR = now;
  }
}

// ═══════════════════════════════════════════════════════════════
//  READ COUNTS — depuis Tab[] sans ADC supplémentaire
// ═══════════════════════════════════════════════════════════════
void readCounts() {
  cAll = 0;
  cL = 0;
  cR = 0;
  for (int i = 0; i < NB_CAPT; i++) {
    if ((int)pid.Tab[i] > pid.minValue[i] + 400) {
      cAll++;
      if (i < NB_CAPT / 2) cL++;
      else cR++;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  PID CASCADE INTERNE — commun aux deux fonctions
// ═══════════════════════════════════════════════════════════════
static inline void _pid_cascade_inner(int p, int position) {
  g_position = position;
  int error = SETPOINT - position;

  int D_out = ((error - lastError_outer) + (lastError_outer - lastError2_outer)) >> 1;
  lastError2_outer = lastError_outer;
  lastError_outer = error;

  I_outer += error;
  if ((error > 0 && I_outer < 0) || (error < 0 && I_outer > 0)) I_outer >>= 1;
  I_outer = constrain(I_outer, -100000, 100000);

  float correctionRPM = error * Kp_outer[p]
                        + (I_outer / 100.0f) * Ki_outer[p]
                        + D_out * Kd_outer[p];
  correctionRPM = constrain(correctionRPM, -MAX_COR[p], MAX_COR[p]);
  g_correction = (int)correctionRPM;

  float targetL = constrain(base_rpm[p] + correctionRPM, -50.0f, MAX_RPM);
  float targetR = constrain(base_rpm[p] - correctionRPM, -50.0f, MAX_RPM);

  updateRPM();

  float errL = targetL - rpmL;
  float errR = targetR - rpmR;

  integL += errL;
  integL = constrain(integL, -300.0f, 300.0f);
  integR += errR;
  integR = constrain(integR, -300.0f, 300.0f);
  if ((errL > 0 && integL < 0) || (errL < 0 && integL > 0)) integL = 0;
  if ((errR > 0 && integR < 0) || (errR < 0 && integR > 0)) integR = 0;

  float FF = 255.0f / MAX_RPM;
  int pwmL_out = constrain((int)(targetL * FF + Kp_inner[p] * errL + Ki_inner[p] * integL), -50, 255);
  int pwmR_out = constrain((int)(targetR * FF + Kp_inner[p] * errR + Ki_inner[p] * integR), -50, 255);

  g_motorspeeda = pwmL_out;
  g_motorspeedb = pwmR_out;

  forward_brake_fast(pwmR_out, pwmL_out);
}

// ⚡ Ligne NOIRE
inline void PID_cascadeB(int p) {
  _pid_cascade_inner(p, pid.ReadLineBlackFast());
}

// ⚡ Ligne BLANCHE
inline void PID_cascadeW(int p) {
  _pid_cascade_inner(p, pid.ReadLineWhiteFast());
}

// ═══════════════════════════════════════════════════════════════
//  COMPTEURS UTILITAIRES
// ═══════════════════════════════════════════════════════════════
inline int count() {
  pid.readDigitalAll();
  int x = 0;
  for (int i = 0; i < NB_CAPT; i++)
    if (pid.Tab1[i] == 1) x++;
  return x;
}
inline int countL() {
  pid.readDigitalAllL();
  int x = 0;
  for (int i = 0; i < NB_CAPT / 2; i++)
    if (pid.Tab1[i] == 1) x++;
  return x;
}
inline int countR() {
  pid.readDigitalAllR();
  int x = 0;
  for (int i = NB_CAPT / 2; i < NB_CAPT; i++)
    if (pid.Tab1[i] == 1) x++;
  return x;
}

float mesurerLoopHz() {
  // Les variables 'static' gardent leur valeur en mémoire à chaque tour de loop
  static unsigned long dernierTemps = 0;
  static unsigned long compteur = 0;
  static float frequenceHz = 0.0;

  // On compte un passage de plus
  compteur++;

  unsigned long tempsActuel = millis();

  // Si exactement 1 seconde (1000 millisecondes) s'est écoulée
  if (tempsActuel - dernierTemps >= 1000) {

    // Le nombre de passages en 1 seconde correspond exactement aux Hertz
    frequenceHz = (float)compteur;

    // Affichage sur le port Série
    Serial.print("Vitesse du loop : ");
    Serial.print(frequenceHz);
    Serial.println(" Hz");

    // On réinitialise pour la seconde suivante
    compteur = 0;
    dernierTemps = tempsActuel;
  }

  // Retourne la valeur en Hz si tu as besoin de l'utiliser ailleurs dans ton code
  return frequenceHz;
}

float testPWM(int pwmTarget) {

  resetEncoders();
  rpmL = 0;
  rpmR = 0;

  AKRA_MASAFA;

  // ==========================
  // 0 -> 20 cm : Anti-cabrage
  // ==========================
  while (MASAFA < 20) {
    if (pwmTarget > 130) {
      forward_brake_fast(130, 130);
    } else {
      forward_brake_fast(pwmTarget, pwmTarget);
    }
    updateRPM();
    delay(2);
  }

  // ==========================
  // 20 -> 120 cm : Mesure
  // ==========================
  float rpmSum = 0;
  int samples = 0;

  while (MASAFA < 120) {
    forward_brake_fast(pwmTarget, pwmTarget);
    updateRPM();

    float rpmAvg = (abs(rpmL) + abs(rpmR)) * 0.5f;
    rpmSum += rpmAvg;
    samples++;

    delay(5);
  }

  // ==========================
  // STOP et Calcul
  // ==========================
  stopMotors();

  // Sécurité pour éviter une division par zéro
  float rpmFinal = 0;
  if (samples > 0) {
    rpmFinal = rpmSum / samples;
  }

  // ==========================
  // Affichage Série
  // ==========================
  Serial.print("PWM ");
  Serial.print(pwmTarget);
  Serial.print(" -> ");
  Serial.print(rpmFinal, 2);
  Serial.println(" RPM");

  // ==========================
  // Affichage OLED
  // ==========================
  u8g2.firstPage();
  do {
    char txt1[25];
    char txt2[25];

    u8g2.setFont(u8g2_font_10x20_tr);
    sprintf(txt1, "PWM:%d", pwmTarget);
    sprintf(txt2, "RPM:%.1f", rpmFinal);

    u8g2.drawStr(0, 22, txt1);
    u8g2.drawStr(0, 52, txt2);

  } while (u8g2.nextPage());

  // On retourne le résultat pour le récupérer dans le loop()
  return rpmFinal;
}
// ═══════════════════════════════════════════════════════════════
//  VIRAGE CASCADE — CONSTANTES
// ═══════════════════════════════════════════════════════════════
float VIRAGE_KP_OUTER = 4.0f;
float VIRAGE_KI_OUTER = 0.0001f;
float VIRAGE_KD_OUTER = 0.08f;
float VIRAGE_KP_INNER = 0.05f;
float VIRAGE_KI_INNER = 0.020f;
float VIRAGE_RPM_MAX = 700.0f;
float VIRAGE_MAX_COR = 700.0f;
int VIRAGE_BRAKE_MS = 25;

#define VIRAGE_TICKS_PAR_DEGRE (220.0f / 90.0f)
inline long virageAnticipation(float angleCible) {
  float a = 2.535f * sqrtf(angleCible) + 6.5f;
  return (long)(a + 0.5f);  // arrondi
}

// ═══════════════════════════════════════════════════════════════
//  UTILITAIRES
// ═══════════════════════════════════════════════════════════════
inline void hardBrakeMoteurs() {
  ledcWrite(MOT_R_PWM1, 255);
  ledcWrite(MOT_R_PWM2, 255);
  ledcWrite(MOT_L_PWM1, 255);
  ledcWrite(MOT_L_PWM2, 255);
}

inline float normaliserAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static void _coapSendVirageForce(uint32_t ts, float angle, long tL, long tR,
                                 float rl, float rr, int pL, int pR, float err) {
  char pl[256];
  int pl_len = snprintf(pl, sizeof(pl),
                        "{\"ts\":%lu,\"ang\":%.2f,\"tL\":%ld,\"tR\":%ld,"
                        "\"rpmL\":%.1f,\"rpmR\":%.1f,\"pwmL\":%d,\"pwmR\":%d,\"err\":%.2f}",
                        ts, angle, tL, tR, rl, rr, pL, pR, err);
  _coapSendRaw("virage", pl, pl_len);
}

// ═══════════════════════════════════════════════════════════════
//  VIRAGE ANGLE PUR — CASCADE
//  angleCible : toujours positif
//  gauche     : true = pivot gauche, false = pivot droite
// ═══════════════════════════════════════════════════════════════
void executerVirageAnglePur(float angleCible, bool gauche) {

  // ── 1. INIT ─────────────────────────────────────────────
  resetEncoders();
  for (uint8_t i = 0; i < 5; i++) calcANG();

  const long TICKS_CIBLE = (long)(angleCible * VIRAGE_TICKS_PAR_DEGRE + 0.5f);
  const long TICKS_FREINAGE = TICKS_CIBLE - virageAnticipation(angleCible);

  Serial.print("[VIRAGE] Cible=");
  Serial.print(angleCible);
  Serial.print("°  ticks=");
  Serial.print(TICKS_CIBLE);
  Serial.print("  freinage à tick=");
  Serial.println(TICKS_FREINAGE);

  int lastErr_v = 0;
  int lastErr2_v = 0;
  long I_v = 0;
  float integLv = 0.0f;
  float integRv = 0.0f;

  // ── 2. BOUCLE CASCADE — phase normale ───────────────────
  while (true) {
    calcANG();

    long rawL = readEncoderL();
    long rawR = readEncoderR();
    long progL = gauche ? -rawL : rawL;
    long progR = gauche ? rawR : -rawR;
    long tMoy = (progL + progR) / 2;

    if (tMoy >= TICKS_FREINAGE) break;

    long erreurTicks = TICKS_CIBLE - tMoy;
    int error = (int)((float)erreurTicks / VIRAGE_TICKS_PAR_DEGRE * 100.0f);

    int D_out = ((error - lastErr_v) + (lastErr_v - lastErr2_v)) >> 1;
    lastErr2_v = lastErr_v;
    lastErr_v = error;

    I_v += error;
    if ((error > 0 && I_v < 0) || (error < 0 && I_v > 0)) I_v >>= 1;
    I_v = constrain(I_v, -100000, 100000);

    float correctionRPM = (float)error * VIRAGE_KP_OUTER
                          + (I_v / 100.0f) * VIRAGE_KI_OUTER
                          + (float)D_out * VIRAGE_KD_OUTER;
    correctionRPM = constrain(correctionRPM, -VIRAGE_MAX_COR, VIRAGE_MAX_COR);

    float targetL, targetR;
    if (gauche) {
      targetL = constrain(-fabsf(correctionRPM), -VIRAGE_RPM_MAX, 0.0f);
      targetR = constrain(fabsf(correctionRPM), 0.0f, VIRAGE_RPM_MAX);
    } else {
      targetL = constrain(fabsf(correctionRPM), 0.0f, VIRAGE_RPM_MAX);
      targetR = constrain(-fabsf(correctionRPM), -VIRAGE_RPM_MAX, 0.0f);
    }

    updateRPM();

    float errL = targetL - rpmL;
    float errR = targetR - rpmR;

    integLv += errL;
    integLv = constrain(integLv, -300.0f, 300.0f);
    integRv += errR;
    integRv = constrain(integRv, -300.0f, 300.0f);
    if ((errL > 0 && integLv < 0) || (errL < 0 && integLv > 0)) integLv = 0;
    if ((errR > 0 && integRv < 0) || (errR < 0 && integRv > 0)) integRv = 0;

    float FF = 255.0f / MAX_RPM;
    int pwmL = constrain((int)(targetL * FF + VIRAGE_KP_INNER * errL + VIRAGE_KI_INNER * integLv), -255, 255);
    int pwmR = constrain((int)(targetR * FF + VIRAGE_KP_INNER * errR + VIRAGE_KI_INNER * integRv), -255, 255);

    forward_brake_fast(pwmR, pwmL);

    float erreurDeg = (float)erreurTicks / VIRAGE_TICKS_PAR_DEGRE;
    coapSendVirage(millis(), angleZ, rawL, rawR, rpmL, rpmR, pwmL, pwmR, erreurDeg);
  }

  // ── 3. DÉCÉLÉRATION PROGRESSIVE ─────────────────────────
  // Même cascade mais rpmMax décroît linéairement 400→0
  // sur les VIRAGE_ANTICIPATION ticks restants
  while (true) {
    calcANG();

    long rawL = readEncoderL();
    long rawR = readEncoderR();
    long progL = gauche ? -rawL : rawL;
    long progR = gauche ? rawR : -rawR;
    long tMoy = (progL + progR) / 2;

    long erreurTicks = TICKS_CIBLE - tMoy;

    if (erreurTicks <= 0) break;

    // RPM max : 400 → 0 linéairement sur 25 ticks
    float ratio = constrain((float)erreurTicks / (float)virageAnticipation(angleCible), 0.0f, 1.0f);
    float rpmMax = VIRAGE_RPM_MAX * ratio;

    int error = (int)((float)erreurTicks / VIRAGE_TICKS_PAR_DEGRE * 100.0f);

    int D_out = ((error - lastErr_v) + (lastErr_v - lastErr2_v)) >> 1;
    lastErr2_v = lastErr_v;
    lastErr_v = error;

    I_v += error;
    if ((error > 0 && I_v < 0) || (error < 0 && I_v > 0)) I_v >>= 1;
    I_v = constrain(I_v, -100000, 100000);

    float correctionRPM = (float)error * VIRAGE_KP_OUTER
                          + (I_v / 100.0f) * VIRAGE_KI_OUTER
                          + (float)D_out * VIRAGE_KD_OUTER;
    correctionRPM = constrain(correctionRPM, -VIRAGE_MAX_COR, VIRAGE_MAX_COR);

    float targetL, targetR;
    if (gauche) {
      targetL = constrain(-fabsf(correctionRPM), -rpmMax, 0.0f);
      targetR = constrain(fabsf(correctionRPM), 0.0f, rpmMax);
    } else {
      targetL = constrain(fabsf(correctionRPM), 0.0f, rpmMax);
      targetR = constrain(-fabsf(correctionRPM), -rpmMax, 0.0f);
    }

    updateRPM();

    float errL = targetL - rpmL;
    float errR = targetR - rpmR;

    integLv += errL;
    integLv = constrain(integLv, -300.0f, 300.0f);
    integRv += errR;
    integRv = constrain(integRv, -300.0f, 300.0f);
    if ((errL > 0 && integLv < 0) || (errL < 0 && integLv > 0)) integLv = 0;
    if ((errR > 0 && integRv < 0) || (errR < 0 && integRv > 0)) integRv = 0;

    float FF = 255.0f / MAX_RPM;
    int pwmL = constrain((int)(targetL * FF + VIRAGE_KP_INNER * errL + VIRAGE_KI_INNER * integLv), -255, 255);
    int pwmR = constrain((int)(targetR * FF + VIRAGE_KP_INNER * errR + VIRAGE_KI_INNER * integRv), -255, 255);

    forward_brake_fast(pwmR, pwmL);

    float erreurDeg = (float)erreurTicks / VIRAGE_TICKS_PAR_DEGRE;
    coapSendVirage(millis(), angleZ, rawL, rawR, rpmL, rpmR, pwmL, pwmR, erreurDeg);
  }

  hardBrakeMoteurs();
  delay(VIRAGE_BRAKE_MS);
  stopMotors();

  // ── 4. SNAPSHOT t+0ms ───────────────────────────────────
  calcANG();
  long ticksL_stop = readEncoderL();
  long ticksR_stop = readEncoderR();
  long progL_stop = gauche ? -ticksL_stop : ticksL_stop;
  long progR_stop = gauche ? ticksR_stop : -ticksR_stop;
  float degStop = (progL_stop + progR_stop) * 0.5f / VIRAGE_TICKS_PAR_DEGRE;
  float errStop = angleCible - degStop;

  _coapSendVirageForce(millis(), angleZ, ticksL_stop, ticksR_stop,
                       0.0f, 0.0f, 0, 0, errStop);

  Serial.print("[STOP ] deg=");
  Serial.print(degStop, 1);
  Serial.print("°  err=");
  Serial.print(errStop, 2);
  Serial.println("°");

  // ── 5. SNAPSHOT t+2s (mesure glissement) ────────────────
  delay(2000);

  calcANG();
  long ticksL_2s = readEncoderL();
  long ticksR_2s = readEncoderR();
  long progL_2s = gauche ? -ticksL_2s : ticksL_2s;
  long progR_2s = gauche ? ticksR_2s : -ticksR_2s;
  float deg2s = (progL_2s + progR_2s) * 0.5f / VIRAGE_TICKS_PAR_DEGRE;
  float err2s = angleCible - deg2s;
  long glissL = ticksL_2s - ticksL_stop;
  long glissR = ticksR_2s - ticksR_stop;

  _coapSendVirageForce(millis(), angleZ, ticksL_2s, ticksR_2s,
                       (float)glissL, (float)glissR, 0, 0, err2s);

  Serial.print("[+2s  ] deg=");
  Serial.print(deg2s, 1);
  Serial.print("°  err=");
  Serial.print(err2s, 2);
  Serial.print("°  glissL=");
  Serial.print(glissL);
  Serial.print("  glissR=");
  Serial.println(glissR);

  // ── 6. SORTIE ────────────────────────────────────────────
  resetEncoders();
}
void testDoura() {
  calcANG();
  float angleAvant = angleZ;

  executerVirageAnglePur(45.0f, false);
  delay(2000);
  executerVirageAnglePur(45.0f, false);
  delay(2000);
  executerVirageAnglePur(90.0f, false);
  delay(2000);
  executerVirageAnglePur(90.0f, false);
  delay(2000);
  executerVirageAnglePur(180.0f, false);
  calcANG();
  float angleApres = angleZ;
  float angleParcouru = fabsf(normaliserAngle(angleApres - angleAvant));

  Serial.print("Avant   : ");
  Serial.println(angleAvant, 2);
  Serial.print("Apres   : ");
  Serial.println(angleApres, 2);
  Serial.print("Parcouru: ");
  Serial.println(angleParcouru, 2);
}
#endif