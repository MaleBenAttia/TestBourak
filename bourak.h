#ifndef BOURAK_H
#define BOURAK_H

#include <U8g2lib.h>
#include <Wire.h>
#include "ElBourak.h"
#include "encoder.h"
#include "coap_telemetry.h"
#include "soc/ledc_struct.h"
// ═══════════════════════════════════════════════════════════════
// CAPTEURS IR — MUX — 14 CAPTEURS
// setpoint centre = (14-1)*1000/2 = 6500
// ═══════════════════════════════════════════════════════════════
#define S0        4
#define S1        13
#define S2        14
#define S3        23
#define SIG       36
#define NB_CAPT   16
#define SETPOINT  7500

ElBourak pid(SIG, S0, S1, S2, S3);

// ═══════════════════════════════════════════════════════════════
// CORRECTION BINÔMES SOUDÉS INVERSÉS (1↔2, 3↔4, 5↔6...)
// Déjà appliquée DANS ElBourak.h au niveau de la lecture ADC
// (readRawAll / readDigitalAllL / readDigitalAllR) : pid.Tab[] et
// pid.Tab1[] sont donc DÉJÀ dans le bon ordre physique, pour TOUT
// le code existant — ReadLineWhiteFast/BlackFast, PID_control_fast,
// PID_cascadeB/W, readCounts, count/countL/countR, etc. Aucun
// changement nécessaire ailleurs, aucun appel supplémentaire à
// faire : tu peux utiliser PID_controlB_fast() et les CAPTx_D
// directement, la lecture ADC est partagée automatiquement.
// ═══════════════════════════════════════════════════════════════

// ── #define par capteur — valeurs CALIBRÉES (0-1000), déjà corrigées ──
// Utilisables directement, ex: if (CAPT3 > 500)
#define CAPT0   pid.Tab[0]
#define CAPT1   pid.Tab[1]
#define CAPT2   pid.Tab[2]
#define CAPT3   pid.Tab[3]
#define CAPT4   pid.Tab[4]
#define CAPT5   pid.Tab[5]
#define CAPT6   pid.Tab[6]
#define CAPT7   pid.Tab[7]
#define CAPT8   pid.Tab[8]
#define CAPT9   pid.Tab[9]
#define CAPT10  pid.Tab[10]
#define CAPT11  pid.Tab[11]
#define CAPT12  pid.Tab[12]
#define CAPT13  pid.Tab[13]
#define CAPT14  pid.Tab[14]
#define CAPT15  pid.Tab[15]

// ═══════════════════════════════════════════════════════════════
// SEUIL PAR CAPTEUR — milieu entre min et max (valeurs BRUTES)
// Seuil[i] = (minValue[i] + maxValue[i]) / 2, calculé APRÈS
// calibration (pid.calibrateSensors() doit avoir tourné avant).
// pid.minValue[]/maxValue[] sont déjà dans le bon ordre physique
// (même correction appliquée à la lecture), donc pas de remap ici.
// TOLERANCE : marge ajoutée/retranchée autour du seuil (0 = nulle,
// pas de tolérance). Changer juste cette valeur pour ajuster.
// ═══════════════════════════════════════════════════════════════
#define TOLERANCE 0   // ex: mettre 100 pour +/-100 autour du seuil

int Seuil[EB_NB_CAPT];

// À appeler UNE FOIS après la phase de calibration (pid.calibrateSensors()
// répétée), avant d'utiliser les #define CAPTx_D ci-dessous.
inline void calculerSeuils() {
    for (int i = 0; i < EB_NB_CAPT; i++) {
        Seuil[i] = (pid.minValue[i] + pid.maxValue[i]) / 2;
    }
}

// ── #define par capteur — DIGITAL 1/0 (valeurs BRUTES + tolérance) ──
// Retourne 1 si pid.Tab1[i] > Seuil[i] + TOLERANCE, sinon 0.
// Fonctionne avec n'importe quelle lecture ADC déjà faite dans la
// même itération (readRawAll / readCalibrated / ReadLineXFast...),
// pas besoin d'appel séparé.
#define CAPT0_D   ((pid.Tab1[0]  > Seuil[0]  + TOLERANCE) ? 1 : 0)
#define CAPT1_D   ((pid.Tab1[1]  > Seuil[1]  + TOLERANCE) ? 1 : 0)
#define CAPT2_D   ((pid.Tab1[2]  > Seuil[2]  + TOLERANCE) ? 1 : 0)
#define CAPT3_D   ((pid.Tab1[3]  > Seuil[3]  + TOLERANCE) ? 1 : 0)
#define CAPT4_D   ((pid.Tab1[4]  > Seuil[4]  + TOLERANCE) ? 1 : 0)
#define CAPT5_D   ((pid.Tab1[5]  > Seuil[5]  + TOLERANCE) ? 1 : 0)
#define CAPT6_D   ((pid.Tab1[6]  > Seuil[6]  + TOLERANCE) ? 1 : 0)
#define CAPT7_D   ((pid.Tab1[7]  > Seuil[7]  + TOLERANCE) ? 1 : 0)
#define CAPT8_D   ((pid.Tab1[8]  > Seuil[8]  + TOLERANCE) ? 1 : 0)
#define CAPT9_D   ((pid.Tab1[9]  > Seuil[9]  + TOLERANCE) ? 1 : 0)
#define CAPT10_D  ((pid.Tab1[10] > Seuil[10] + TOLERANCE) ? 1 : 0)
#define CAPT11_D  ((pid.Tab1[11] > Seuil[11] + TOLERANCE) ? 1 : 0)
#define CAPT12_D  ((pid.Tab1[12] > Seuil[12] + TOLERANCE) ? 1 : 0)
#define CAPT13_D  ((pid.Tab1[13] > Seuil[13] + TOLERANCE) ? 1 : 0)
#define CAPT14_D  ((pid.Tab1[14] > Seuil[14] + TOLERANCE) ? 1 : 0)
#define CAPT15_D  ((pid.Tab1[15] > Seuil[15] + TOLERANCE) ? 1 : 0)

// ═══════════════════════════════════════════════════════════════
// ███  MOTEURS — BTS7960 (pins + fonctions bas niveau)  ███
// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
// MOTEURS — BTS7960
// ═══════════════════════════════════════════════════════════════
#define MOT_R_PWM1  25
#define MOT_R_PWM2  26
#define MOT_L_PWM1  32
#define MOT_L_PWM2  33

#define CH_R1 0
#define CH_R2 1
#define CH_L1 2
#define CH_L2 3
#define PWM_FREQ 20000
#define PWM_BITS 8

// ═══════════════════════════════════════════════════════════════
// Précalcul des canaux PWM LEDC pour BTS7960
// 4 canaux : R_PWM1, R_PWM2, L_PWM1, L_PWM2


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

// Rotation sur place — vérifié sur le robot assemblé :
// forward_brake_fast(pwmR, pwmL) → (pwm, -pwm) tourne à DROITE
inline void tournerDroite(int pwm) { forward_brake_fast(pwm, -pwm); }
inline void tournerGauche(int pwm) { forward_brake_fast(-pwm, pwm); }
// ═══════════════════════════════════════════════════════════════
// LED + BOUTON
// ═══════════════════════════════════════════════════════════════
#define LED_BTN   2
#define LED_BTNIn digitalRead(LED_BTN)
#define LEDON     digitalWrite(LED_BTN, 1)
#define LEDOFF    digitalWrite(LED_BTN, 0)
// ═══════════════════════════════════════════════════════════════
// ███  OLED — objet + fonctions d'affichage  ███
// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
// OLED
// ═══════════════════════════════════════════════════════════════
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);
bool oledActif = true;
// ═══════════════════════════════════════════════════════════════
// OLED
// ═══════════════════════════════════════════════════════════════
void afficherTexte(String message, byte style) {
    u8g2.firstPage();
    do {
        switch (style) {
            case 1: u8g2.setFont(u8g2_font_10x20_tr); u8g2.drawStr(0, 35, message.c_str()); break;
            case 2: u8g2.setFont(u8g2_font_helvR12_te); u8g2.drawStr(0, 40, message.c_str()); break;
            case 3: u8g2.setFont(u8g2_font_8x13_tr); u8g2.drawFrame(0,10,128,44); u8g2.drawStr(5, 35, message.c_str()); break;
            default: u8g2.setFont(u8g2_font_8x13_tr); u8g2.drawStr(0, 32, message.c_str());
        }
    } while (u8g2.nextPage());
}

void afficherValeur(double valeur, byte style) {
    char buffer[16];
    dtostrf(valeur, 1, 2, buffer);
    afficherTexte(String(buffer), style);
}

inline void afficherTexte1(float v, byte s)  { afficherValeur(v, s); }
inline void afficherTexte2(long  v, byte s)  { afficherValeur((double)v, s); }
void eteindreOLED() { u8g2.setPowerSave(true);  oledActif = false; }
void allumerOLED()  { u8g2.setPowerSave(false); oledActif = true;  }

// ============================================================
// AFFICHAGE DES 16 CAPTEURS SUR OLED
//
// mode = 0 : BRUT
//   Valeurs ADC brutes : 0 → 4095
//
// mode = 1 : CALIBRE
//   Valeurs calibrées : 0 → 1000
//
// mode = 2 : DIGITAL
//   Valeurs 0 / 1 selon Seuil[i] + TOLERANCE
// ============================================================

void afficherBarresCapteurs(int mode) {

    if (mode == 0) {
        pid.readRawAll();

    } else if (mode == 1) {
        pid.readCalibrated();

    } else if (mode == 2) {
        pid.readRawAll();
    }

    u8g2.firstPage();

    do {

        u8g2.setFont(u8g2_font_6x10_tr);

        if (mode == 0)
            u8g2.drawStr(0, 10, "Mode : Brut");

        else if (mode == 1)
            u8g2.drawStr(0, 10, "Mode : Calibre");

        else if (mode == 2)
            u8g2.drawStr(0, 10, "Mode : Digital");

        for (uint8_t i = 0; i < NB_CAPT; i++) {

            uint8_t hauteurBarre = 0;

            if (mode == 0) {

                hauteurBarre = map(
                    constrain(pid.Tab1[i], 0, 4095),
                    0,
                    4095,
                    0,
                    50
                );

            } else if (mode == 1) {

                hauteurBarre = map(
                    constrain(pid.Tab[i], 0, 1000),
                    0,
                    1000,
                    0,
                    50
                );

            } else if (mode == 2) {

                bool actif =
                    pid.Tab1[i] > (Seuil[i] + TOLERANCE);

                hauteurBarre = actif ? 50 : 0;
            }

            u8g2.drawBox(
                5 + (i * 7),
                64 - hauteurBarre,
                5,
                hauteurBarre
            );

            if (i % 2 == 0) {
                u8g2.setCursor(
                    5 + (i * 7),
                    63
                );
                u8g2.print(i);
            }
        }

    } while (u8g2.nextPage());

    delay(100);
}
// ═══════════════════════════════════════════════════════════════
// ███  MPU6050 — I2C ROBUSTE + PID D'ANGLE (CAP)  ███
// Tout ce qui concerne le gyro (constantes, variables, lecture
// I2C anti-hang, auto-recovery bus) ET son PID de maintien de
// cap est regroupé ici.
// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
// MPU6050 — I2C ROBUSTE (anti-hang)
// Fix : filtre hardware DLPF (reg 0x1A) + Wire.setTimeOut() courte
// + comptage des échecs consécutifs (i2cHealthy) + auto-reset du
// bus I2C. Remplace l'ancienne lecture bloquante sans détection
// d'échec ni recovery.
// ═══════════════════════════════════════════════════════════════
#define MPU_ADDR 0x68
#define MPU_SDA  21
#define MPU_SCL  22

int16_t gyroZ;
float   rotZ;
unsigned long lastMicros   = 0;
float         angleZ       = 0;
float         gyroBiasZ    = 0;

// Diagnostics I2C
uint32_t mpuReadsOK     = 0;
uint32_t mpuReadsFailed = 0;

// Santé I2C : passe à false dès I2C_FAULT_THRESHOLD échecs consécutifs
// (seuil bas, volontairement plus agressif que le seuil de reset du bus)
// pour couper le PID d'angle avant que le robot ne roule en aveugle.
#define I2C_FAULT_THRESHOLD        3
#define I2C_MAX_CONSECUTIVE_FAILS  30
#define I2C_RESET_COOLDOWN_MS      1000
#define I2C_RESET_MAX_ATTEMPTS     6
volatile bool i2cHealthy       = true;
uint8_t       i2cResetAttempts = 0;
uint16_t      mpuConsecutiveFails = 0;
unsigned long lastI2CResetMs   = 0;

// PID de cap (garde le robot droit)
float Kp_mpu = 9.0f, Ki_mpu = 0.005f, Kd_mpu = 3.0f;
float mpuError = 0, mpuIntegral = 0, mpuLastError = 0;
float targetAngle = 0;
bool  mpuInitialized = false;

// Dernières valeurs calculées par runMPU_PID() — exposées pour la télémétrie
float g_mpu_correction = 0.0f;
int   g_mpu_pwmR = 0;
int   g_mpu_pwmL = 0;
int   baseSpeed = 140, maxSpeed = 220, currentSpeed = 0;
// ═══════════════════════════════════════════════════════════════
// MPU — FONCTIONS (I2C ROBUSTE, anti-hang)
// ═══════════════════════════════════════════════════════════════
void setupMPU() {
    // Timeout I2C court (défaut ESP32 ~50ms) : évite qu'un bus qui
    // coince ne bloque la boucle pendant des dizaines de ms.
    Wire.setTimeOut(8);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); // wake up
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); // gyro ±500°/s
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); // accel ±2g
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); // DLPF hardware 44Hz — filtre le bruit avant lecture
}

void calibrateGyro() {
    long sumZ = 0;
    afficherTexte("Calibration...", 1);
    for (int i = 0; i < 1000; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 2);
        if (Wire.available() >= 2) gyroZ = (Wire.read() << 8) | Wire.read();
        sumZ += gyroZ;
        delayMicroseconds(900);
    }
    gyroBiasZ = (float)sumZ / 1000.0f;
    lastMicros = micros();
    angleZ = 0;
    afficherTexte("Pret!", 2);
    delay(500);
}

// ── AUTO-RECUPERATION BUS I2C ────────────────────────────────────
// Un MPU6050 qui plante/reboot peut laisser SDA bloqué bas -> toutes
// les transactions échouent en boucle. On ré-initialise le bus après
// N échecs consécutifs, avec cooldown + nombre max de tentatives pour
// ne pas spammer Wire.end()/begin() (source connue de crash ESP32) si
// le capteur est simplement débranché.
void resetI2CBus() {
    if (i2cResetAttempts >= I2C_RESET_MAX_ATTEMPTS) {
        mpuConsecutiveFails = 0;
        return;   // abandon définitif jusqu'au reboot manuel
    }
    unsigned long now = millis();
    if (now - lastI2CResetMs < I2C_RESET_COOLDOWN_MS) {
        mpuConsecutiveFails = 0;
        return;
    }
    lastI2CResetMs = now;
    i2cResetAttempts++;

    Wire.end();
    Wire.begin(MPU_SDA, MPU_SCL, 100000);
    Wire.setTimeOut(8);
    mpuConsecutiveFails = 0;
    mpuInitialized = false;  // force un ré-init du PID au prochain démarrage
    Serial.print("[I2C] Bus reinitialise (tentative ");
    Serial.print(i2cResetAttempts); Serial.print("/");
    Serial.print(I2C_RESET_MAX_ATTEMPTS); Serial.println(") apres echecs repetes");
}

// calcANG robuste : lecture GyroZ uniquement (2 bytes, repeated start).
// Ne bloque jamais plus que Wire.setTimeOut(8) — si la lecture échoue,
// on NE réintègre PAS l'ancienne valeur de gyroZ (sinon angleZ dérive
// dans le vide) : on sort simplement, lastMicros est déjà avancé donc
// le dt du prochain tick réussi rattrape le temps écoulé correctement.
IRAM_ATTR void calcANG() {
    unsigned long now = micros();
    if (lastMicros == 0) { lastMicros = now; return; }
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    if (Wire.endTransmission(false) != 0) {
        mpuReadsFailed++; mpuConsecutiveFails++;
        if (mpuConsecutiveFails >= I2C_FAULT_THRESHOLD)       i2cHealthy = false;
        if (mpuConsecutiveFails >= I2C_MAX_CONSECUTIVE_FAILS) resetI2CBus();
        return;
    }
    if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2 || Wire.available() < 2) {
        mpuReadsFailed++; mpuConsecutiveFails++;
        if (mpuConsecutiveFails >= I2C_FAULT_THRESHOLD)       i2cHealthy = false;
        if (mpuConsecutiveFails >= I2C_MAX_CONSECUTIVE_FAILS) resetI2CBus();
        return;
    }

    gyroZ = (Wire.read() << 8) | Wire.read();
    mpuReadsOK++;
    mpuConsecutiveFails = 0;
    i2cHealthy       = true;
    i2cResetAttempts = 0;  // panne résolue -> on redonne droit à de nouvelles tentatives de reset

    float rawZ = gyroZ - gyroBiasZ;
    if (rawZ > -40.0f && rawZ < 40.0f) rawZ = 0;
    rotZ   = -rawZ / 65.5f;   // signe inversé : sens trigonométrique (anti-horaire = positif)
    angleZ += rotZ * dt;
    if (angleZ >= 360.0f)  angleZ -= 360.0f;
    if (angleZ <    0.0f)  angleZ += 360.0f;
}
// ═══════════════════════════════════════════════════════════════
// MPU PID
// ═══════════════════════════════════════════════════════════════
void initMPU_PID(int base, int maxSp, float angleOffset) {
    calcANG();
    targetAngle   = angleZ + angleOffset;
    mpuIntegral   = 0;
    mpuLastError  = 0;
    baseSpeed     = constrain(base,  0, 255);
    maxSpeed      = constrain(maxSp, 0, 255);
    currentSpeed  = baseSpeed;
    mpuInitialized = true;
}

IRAM_ATTR void runMPU_PID() {
    if (!mpuInitialized) return;
    calcANG();
    if (!i2cHealthy) { stopMotors(); return; }  // sécurité anti-hang : capteur mort -> on ne roule pas en aveugle

    mpuError = angleZ - targetAngle;
    if (mpuError >  180.0f) mpuError -= 360.0f;
    if (mpuError < -180.0f) mpuError += 360.0f;

    float Pterm = Kp_mpu * mpuError;
    mpuIntegral += mpuError;
    if (mpuIntegral >  400.0f) mpuIntegral =  400.0f;
    if (mpuIntegral < -400.0f) mpuIntegral = -400.0f;
    float Iterm = Ki_mpu * mpuIntegral;
    float Dterm = Kd_mpu * (mpuError - mpuLastError);
    mpuLastError = mpuError;

    float correction = Pterm + Iterm + Dterm;
    if (correction >  180.0f) correction =  180.0f;
    if (correction < -180.0f) correction = -180.0f;

    int sR = (int)(baseSpeed + correction); if (sR > 255) sR = 255; if (sR < 0) sR = 0;
    int sL = (int)(baseSpeed - correction); if (sL > 255) sL = 255; if (sL < 0) sL = 0;

    g_mpu_correction = correction;
    g_mpu_pwmR = sR;
    g_mpu_pwmL = sL;

    forward_brake_fast(sR, sL);
}

void stopMPU_PID() {
    stopMotors();
    mpuInitialized = false;
    currentSpeed   = 0;
}

void setMPU_PID(float kp, float ki, float kd) {
    Kp_mpu = kp; Ki_mpu = ki; Kd_mpu = kd;
}

void gyroTurnPID(float angle, int maxSpd, float tolerance) {
    float Kp_t = 2.7f, Kd_t = 0.03f;
    float error, prevError = 0, output;
    calcANG();
    float tgtAngle = fmod(angleZ + angle + 360.0f, 360.0f);
    unsigned long lastTime  = millis();
    unsigned long startTime = millis();
    const unsigned long GYRO_TURN_TIMEOUT_MS = 5000;  // anti-hang : abandon si le capteur ne répond plus
    while (true) {
        calcANG();
        if (!i2cHealthy || millis() - startTime > GYRO_TURN_TIMEOUT_MS) break;
        error = tgtAngle - angleZ;
        if (error >  180) error -= 360;
        if (error < -180) error += 360;
        if (fabsf(error) <= tolerance) break;
        unsigned long now = millis();
        float dt = (float)(now - lastTime) * 0.001f;
        if (dt < 0.001f) dt = 0.001f;
        lastTime = now;
        float deriv = (error - prevError) / dt;
        prevError = error;
        output = Kp_t * error + Kd_t * deriv;
        if (output >  maxSpd) output =  maxSpd;
        if (output < -maxSpd) output = -maxSpd;
        forward_brake_fast((int)output, -(int)output);
    }
    stopMotors();
}
// ═══════════════════════════════════════════════════════════════
// ███  SUIVI DE LIGNE — PID ANCIEN (compatibilité)  ███
// Constantes/variables + fonctions PID_control_fast/B/W,
// readCounts() et les compteurs digitaux count/countL/countR.
// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
// PID ANCIEN — compatibilité
// ═══════════════════════════════════════════════════════════════
const int   ConstantCount_old = 4;
static const float Kp_old[4] = { 0.05f, 0.15f, 0.065f, 0.0f };
static const float Ki_old[4] = { 0.0005f, 0.0005f, 0.0003f, 0.0f };
static const float Kd_old[4] = { 0.21f, 0.19f, 0.32f, 0.0f };
int  maxspeeda = 170, maxspeedb = 170;
int  basespeeda = 120, basespeedb = 120;
int  P, I, D;
int  lastError = 0, lastError2 = 0;
long I_accumulated = 0;
int cAll = 0, cL = 0, cR = 0;   // utilisés par readCounts() (PID ancien)

// ═══════════════════════════════════════════════════════════════
// PID ANCIEN — compatibilité
// ═══════════════════════════════════════════════════════════════
void PID_control_fast(int idx, int st, bool whiteLine) {
    int position = whiteLine ? pid.ReadLineWhiteFast() : pid.ReadLineBlackFast();
    int error    = st - position;
    int d        = ((error - lastError) + (lastError - lastError2)) >> 1;
    lastError2   = lastError;
    lastError    = error;
    I_accumulated += error;
    if ((error > 0 && I_accumulated < 0) || (error < 0 && I_accumulated > 0)) I_accumulated >>= 1;
    if (I_accumulated >  100000) I_accumulated =  100000;
    if (I_accumulated < -100000) I_accumulated = -100000;
    I = (int)(I_accumulated / 100);
    P = error;
    D = d;
    int motorspeed = (int)(P * Kp_old[idx] + I * Ki_old[idx] + D * Kd_old[idx]);
    int mA = basespeeda + motorspeed; if (mA > maxspeeda) mA = maxspeeda; if (mA < -80) mA = -80;
    int mB = basespeedb - motorspeed; if (mB > maxspeedb) mB = maxspeedb; if (mB < -80) mB = -80;
    forward_brake_fast(mA, mB);
}

inline void PID_controlB_fast(int idx, int st) { PID_control_fast(idx, st, false); }
inline void PID_controlW_fast(int idx, int st) { PID_control_fast(idx, st, true);  }
// ═══════════════════════════════════════════════════════════════
// READ COUNTS
// ═══════════════════════════════════════════════════════════════
void readCounts() {
    cAll = 0; cL = 0; cR = 0;
    for (int i = 0; i < NB_CAPT; i++) {
        if ((int)pid.Tab[i] > pid.minValue[i] + 400) {
            cAll++;
            if (i < NB_CAPT / 2) cL++; else cR++;
        }
    }
}
inline int count(bool lireADC = true) {
    if (lireADC) pid.readRawAll();
    int x = 0;
    for (int i = 0; i < NB_CAPT; i++) if (pid.Tab1[i] > Seuil[i] + TOLERANCE) x++;
    return x;
}

inline int countL(bool lireADC = true) {
    if (lireADC) pid.readRawAll();
    int x = 0;
    for (int i = 0; i < NB_CAPT / 2; i++) if (pid.Tab1[i] > Seuil[i] + TOLERANCE) x++;
    return x;
}

inline int countR(bool lireADC = true) {
    if (lireADC) pid.readRawAll();
    int x = 0;
    for (int i = NB_CAPT / 2; i < NB_CAPT; i++) if (pid.Tab1[i] > Seuil[i] + TOLERANCE) x++;
    return x;
}
// ═══════════════════════════════════════════════════════════════
// ███  SUIVI DE LIGNE — PID CASCADE (position → RPM → PWM)  ███
// Profils, état interne, updateRPM() (lecture encodeurs → RPM
// réel) et les fonctions PID_cascadeB/W.
// ═══════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════
// PID CASCADE — 7 profils
// 50pwm→184rpm | 100→397 | 150→594 | 200→765 | 255→950
// ═══════════════════════════════════════════════════════════════
const int ProfileCount = 7;
float base_rpm[ProfileCount]  = { 60.0f, 180.0f, 320.0f, 460.0f, 600.0f, 760.0f, 940.0f };
float Kp_outer[ProfileCount]  = { 0.09f,  0.08f,  0.08f,  0.08f,  0.07f,  0.07f,  0.06f };
float Ki_outer[ProfileCount]  = { 0.0001f,0.0001f,0.0001f,0.0001f,0.0001f,0.0001f,0.0001f };
float Kd_outer[ProfileCount]  = { 0.65f,  0.55f,  0.55f,  0.55f,  0.58f,  0.60f,  0.65f };
float Kp_inner[ProfileCount]  = { 0.05f,  0.07f,  0.05f,  0.05f,  0.05f,  0.04f,  0.04f };
float Ki_inner[ProfileCount]  = { 0.028f, 0.024f, 0.024f, 0.024f, 0.022f, 0.020f, 0.018f };
float MAX_COR[ProfileCount]   = { 500.0f, 500.0f, 500.0f, 500.0f, 600.0f, 700.0f, 800.0f };
float MAX_RPM = 950.0f;
float MIN_RPM = 150.0f;
int   activeProfile = 2;

int  lastError_outer = 0, lastError2_outer = 0;
long I_outer = 0;
float integL = 0.0f, integR = 0.0f;

// télémétrie
int   g_motorspeeda = 0, g_motorspeedb = 0;
int   g_correction  = 0, g_position = 0;

// RPM (partagé : updateRPM, PID cascade, testPWM)
float rpmL = 0.0f, rpmR = 0.0f;
static long     _prevTicksL = 0, _prevTicksR = 0;
static uint32_t _prevTimeL  = 0, _prevTimeR  = 0;

// ═══════════════════════════════════════════════════════════════
// UPDATE RPM
// ═══════════════════════════════════════════════════════════════
IRAM_ATTR void updateRPM() {
    uint32_t now = micros();
    float dtL = (now - _prevTimeL) * 1e-6f;
    if (dtL >= 0.005f) {
        long tL = readEncoderL();
        rpmL = ((float)(tL - _prevTicksL) / dtL) / TICKS_PAR_TOUR_ROUE * 60.0f;
        _prevTicksL = tL;
        _prevTimeL  = now;
    }
    float dtR = (now - _prevTimeR) * 1e-6f;
    if (dtR >= 0.005f) {
        long tR = readEncoderR();
        rpmR = ((float)(tR - _prevTicksR) / dtR) / TICKS_PAR_TOUR_ROUE * 60.0f;
        _prevTicksR = tR;
        _prevTimeR  = now;
    }
}
// ═══════════════════════════════════════════════════════════════
// PID CASCADE INTERNE
// ═══════════════════════════════════════════════════════════════
static IRAM_ATTR inline void _pid_cascade_inner(int p, int position) {
    g_position = position;
    int error  = SETPOINT - position;

    int D_out = ((error - lastError_outer) + (lastError_outer - lastError2_outer)) >> 1;
    lastError2_outer = lastError_outer;
    lastError_outer  = error;

    I_outer += error;
    if ((error > 0 && I_outer < 0) || (error < 0 && I_outer > 0)) I_outer >>= 1;
    if (I_outer >  100000) I_outer =  100000;
    if (I_outer < -100000) I_outer = -100000;

    float correctionRPM = error * Kp_outer[p]
                        + (I_outer * 0.01f) * Ki_outer[p]
                        + D_out * Kd_outer[p];
    float mc = MAX_COR[p];
    if (correctionRPM >  mc) correctionRPM =  mc;
    if (correctionRPM < -mc) correctionRPM = -mc;
    g_correction = (int)correctionRPM;

    float targetL = base_rpm[p] + correctionRPM;
    float targetR = base_rpm[p] - correctionRPM;
    if (targetL > MAX_RPM) targetL = MAX_RPM; if (targetL < -50.0f) targetL = -50.0f;
    if (targetR > MAX_RPM) targetR = MAX_RPM; if (targetR < -50.0f) targetR = -50.0f;

    updateRPM();

    float errL = targetL - rpmL;
    float errR = targetR - rpmR;

    integL += errL;
    if (integL >  300.0f) integL =  300.0f; if (integL < -300.0f) integL = -300.0f;
    integR += errR;
    if (integR >  300.0f) integR =  300.0f; if (integR < -300.0f) integR = -300.0f;
    if ((errL > 0 && integL < 0) || (errL < 0 && integL > 0)) integL = 0;
    if ((errR > 0 && integR < 0) || (errR < 0 && integR > 0)) integR = 0;

    const float FF = 255.0f / 950.0f;
    int pwmL_out = (int)(targetL * FF + Kp_inner[p] * errL + Ki_inner[p] * integL);
    int pwmR_out = (int)(targetR * FF + Kp_inner[p] * errR + Ki_inner[p] * integR);
    if (pwmL_out >  255) pwmL_out =  255; if (pwmL_out < -50) pwmL_out = -50;
    if (pwmR_out >  255) pwmR_out =  255; if (pwmR_out < -50) pwmR_out = -50;

    g_motorspeeda = pwmL_out;
    g_motorspeedb = pwmR_out;

    forward_brake_fast(pwmR_out, pwmL_out);
}

IRAM_ATTR inline void PID_cascadeB(int p) { _pid_cascade_inner(p, pid.ReadLineBlackFast()); }
IRAM_ATTR inline void PID_cascadeW(int p) { _pid_cascade_inner(p, pid.ReadLineWhiteFast()); }
// ═══════════════════════════════════════════════════════════════
// VIRAGE CASCADE
// ═══════════════════════════════════════════════════════════════
float VIRAGE_KP_OUTER  = 4.0f;
float VIRAGE_KI_OUTER  = 0.0001f;
float VIRAGE_KD_OUTER  = 0.08f;
float VIRAGE_KP_INNER  = 0.05f;
float VIRAGE_KI_INNER  = 0.020f;
float VIRAGE_RPM_MAX   = 700.0f;
float VIRAGE_MAX_COR   = 700.0f;
int   VIRAGE_BRAKE_MS  = 25;
#define VIRAGE_TICKS_PAR_DEGRE (220.0f / 90.0f)

inline long virageAnticipation(float angleCible) {
    float a = 2.535f * sqrtf(angleCible) + 6.5f;
    return (long)(a + 0.5f);
}

inline void hardBrakeMoteurs() {
  ledcWrite(MOT_R_PWM1, 255);
  ledcWrite(MOT_R_PWM2, 255);
  ledcWrite(MOT_L_PWM1, 255);
  ledcWrite(MOT_L_PWM2, 255);
}
inline float normaliserAngle(float a) {
    while (a >  180.0f) a -= 360.0f;
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

#define VIRAGE_TIMEOUT_MS  4000  // anti-hang : abandon si le virage ne converge pas (mauvais sens encodeur, blocage mécanique...)

void executerVirageAnglePur(float angleCible, bool gauche) {
    resetEncoders();
    for (uint8_t i = 0; i < 5; i++) calcANG();

    const long TICKS_CIBLE   = (long)(angleCible * VIRAGE_TICKS_PAR_DEGRE + 0.5f);
    const long TICKS_FREINAGE = TICKS_CIBLE - virageAnticipation(angleCible);

    Serial.print("[VIRAGE] Cible="); Serial.print(angleCible);
    Serial.print("° ticks="); Serial.print(TICKS_CIBLE);
    Serial.print(" freinage="); Serial.println(TICKS_FREINAGE);

    int   lastErr_v = 0, lastErr2_v = 0;
    long  I_v = 0;
    float integLv = 0.0f, integRv = 0.0f;
    unsigned long t_start_v = millis();
    unsigned long t_last_dbg = 0;

    // ── Phase normale ───────────────────────────────────────────
    while (true) {
        calcANG();
        long rawL = readEncoderL(), rawR = readEncoderR();
        long progL = gauche ? rawL : -rawL;   // polarité encodeur inversée sur ce robot
        long progR = gauche ? -rawR : rawR;   // polarité encodeur inversée sur ce robot
        long tMoy  = (progL + progR) / 2;
        if (tMoy >= TICKS_FREINAGE) break;

        if (millis() - t_start_v > VIRAGE_TIMEOUT_MS) {
            Serial.println("[VIRAGE] TIMEOUT phase normale — arret securite (verifier sens encodeur/gauche)");
            stopMotors();
            return;
        }
        if (millis() - t_last_dbg > 200) {
            t_last_dbg = millis();
            Serial.print("[VIRAGE] tMoy="); Serial.print(tMoy);
            Serial.print(" cible="); Serial.print(TICKS_CIBLE);
            Serial.print(" rawL="); Serial.print(rawL);
            Serial.print(" rawR="); Serial.println(rawR);
        }

        long  erreurTicks   = TICKS_CIBLE - tMoy;
        int   error         = (int)((float)erreurTicks / VIRAGE_TICKS_PAR_DEGRE * 100.0f);
        int   D_out         = ((error - lastErr_v) + (lastErr_v - lastErr2_v)) >> 1;
        lastErr2_v = lastErr_v; lastErr_v = error;
        I_v += error;
        if ((error > 0 && I_v < 0) || (error < 0 && I_v > 0)) I_v >>= 1;
        if (I_v >  100000) I_v =  100000; if (I_v < -100000) I_v = -100000;

        float correctionRPM = error * VIRAGE_KP_OUTER + (I_v * 0.01f) * VIRAGE_KI_OUTER + D_out * VIRAGE_KD_OUTER;
        if (correctionRPM >  VIRAGE_MAX_COR) correctionRPM =  VIRAGE_MAX_COR;
        if (correctionRPM < -VIRAGE_MAX_COR) correctionRPM = -VIRAGE_MAX_COR;

        float targetL = gauche ? -fabsf(correctionRPM) : fabsf(correctionRPM);
        float targetR = gauche ?  fabsf(correctionRPM) : -fabsf(correctionRPM);
        if (targetL < -VIRAGE_RPM_MAX) targetL = -VIRAGE_RPM_MAX;
        if (targetL >  VIRAGE_RPM_MAX) targetL =  VIRAGE_RPM_MAX;
        if (targetR < -VIRAGE_RPM_MAX) targetR = -VIRAGE_RPM_MAX;
        if (targetR >  VIRAGE_RPM_MAX) targetR =  VIRAGE_RPM_MAX;

        updateRPM();
        float errL = targetL - rpmL, errR = targetR - rpmR;
        integLv += errL; if (integLv > 300) integLv = 300; if (integLv < -300) integLv = -300;
        integRv += errR; if (integRv > 300) integRv = 300; if (integRv < -300) integRv = -300;
        if ((errL > 0 && integLv < 0) || (errL < 0 && integLv > 0)) integLv = 0;
        if ((errR > 0 && integRv < 0) || (errR < 0 && integRv > 0)) integRv = 0;

        float FF   = 255.0f / MAX_RPM;
        int   pwmL = (int)(targetL * FF + VIRAGE_KP_INNER * errL + VIRAGE_KI_INNER * integLv);
        int   pwmR = (int)(targetR * FF + VIRAGE_KP_INNER * errR + VIRAGE_KI_INNER * integRv);
        if (pwmL >  255) pwmL =  255; if (pwmL < -255) pwmL = -255;
        if (pwmR >  255) pwmR =  255; if (pwmR < -255) pwmR = -255;
        forward_brake_fast(pwmR, pwmL);

        float erreurDeg = (float)erreurTicks / VIRAGE_TICKS_PAR_DEGRE;
        coapSendVirage(millis(), angleZ, rawL, rawR, rpmL, rpmR, pwmL, pwmR, erreurDeg);
    }

    // ── Décélération progressive ────────────────────────────────
    while (true) {
        calcANG();
        long rawL = readEncoderL(), rawR = readEncoderR();
        long progL = gauche ? rawL : -rawL;   // polarité encodeur inversée sur ce robot
        long progR = gauche ? -rawR : rawR;   // polarité encodeur inversée sur ce robot
        long tMoy  = (progL + progR) / 2;
        long erreurTicks = TICKS_CIBLE - tMoy;
        if (erreurTicks <= 0) break;

        if (millis() - t_start_v > VIRAGE_TIMEOUT_MS) {
            Serial.println("[VIRAGE] TIMEOUT phase deceleration — arret securite");
            stopMotors();
            return;
        }

        float ratio  = (float)erreurTicks / (float)virageAnticipation(angleCible);
        if (ratio > 1.0f) ratio = 1.0f;
        float rpmMax = VIRAGE_RPM_MAX * ratio;

        int   error  = (int)((float)erreurTicks / VIRAGE_TICKS_PAR_DEGRE * 100.0f);
        int   D_out  = ((error - lastErr_v) + (lastErr_v - lastErr2_v)) >> 1;
        lastErr2_v = lastErr_v; lastErr_v = error;
        I_v += error;
        if ((error > 0 && I_v < 0) || (error < 0 && I_v > 0)) I_v >>= 1;
        if (I_v >  100000) I_v =  100000; if (I_v < -100000) I_v = -100000;

        float correctionRPM = error * VIRAGE_KP_OUTER + (I_v * 0.01f) * VIRAGE_KI_OUTER + D_out * VIRAGE_KD_OUTER;
        if (correctionRPM >  VIRAGE_MAX_COR) correctionRPM =  VIRAGE_MAX_COR;
        if (correctionRPM < -VIRAGE_MAX_COR) correctionRPM = -VIRAGE_MAX_COR;

        float targetL = gauche ? -fabsf(correctionRPM) : fabsf(correctionRPM);
        float targetR = gauche ?  fabsf(correctionRPM) : -fabsf(correctionRPM);
        if (targetL < -rpmMax) targetL = -rpmMax; if (targetL > rpmMax) targetL = rpmMax;
        if (targetR < -rpmMax) targetR = -rpmMax; if (targetR > rpmMax) targetR = rpmMax;

        updateRPM();
        float errL = targetL - rpmL, errR = targetR - rpmR;
        integLv += errL; if (integLv > 300) integLv = 300; if (integLv < -300) integLv = -300;
        integRv += errR; if (integRv > 300) integRv = 300; if (integRv < -300) integRv = -300;
        if ((errL > 0 && integLv < 0) || (errL < 0 && integLv > 0)) integLv = 0;
        if ((errR > 0 && integRv < 0) || (errR < 0 && integRv > 0)) integRv = 0;

        float FF   = 255.0f / MAX_RPM;
        int   pwmL = (int)(targetL * FF + VIRAGE_KP_INNER * errL + VIRAGE_KI_INNER * integLv);
        int   pwmR = (int)(targetR * FF + VIRAGE_KP_INNER * errR + VIRAGE_KI_INNER * integRv);
        if (pwmL >  255) pwmL =  255; if (pwmL < -255) pwmL = -255;
        if (pwmR >  255) pwmR =  255; if (pwmR < -255) pwmR = -255;
        forward_brake_fast(pwmR, pwmL);

        float erreurDeg = (float)erreurTicks / VIRAGE_TICKS_PAR_DEGRE;
        coapSendVirage(millis(), angleZ, rawL, rawR, rpmL, rpmR, pwmL, pwmR, erreurDeg);
    }

    hardBrakeMoteurs(); delay(VIRAGE_BRAKE_MS); stopMotors();

    // snapshots
    calcANG();
    long tL_s = readEncoderL(), tR_s = readEncoderR();
    long pL_s = gauche ? -tL_s : tL_s, pR_s = gauche ? tR_s : -tR_s;
    float degStop = (pL_s + pR_s) * 0.5f / VIRAGE_TICKS_PAR_DEGRE;
    _coapSendVirageForce(millis(), angleZ, tL_s, tR_s, 0,0,0,0, angleCible - degStop);
    Serial.print("[STOP ] deg="); Serial.print(degStop, 1);
    Serial.print("° err="); Serial.println(angleCible - degStop, 2);

    delay(2000); calcANG();
    long tL_2 = readEncoderL(), tR_2 = readEncoderR();
    long pL_2 = gauche ? -tL_2 : tL_2, pR_2 = gauche ? tR_2 : -tR_2;
    float deg2s = (pL_2 + pR_2) * 0.5f / VIRAGE_TICKS_PAR_DEGRE;
    _coapSendVirageForce(millis(), angleZ, tL_2, tR_2,
        (float)(tL_2-tL_s),(float)(tR_2-tR_s),0,0, angleCible - deg2s);
    Serial.print("[+2s ] deg="); Serial.print(deg2s, 1);
    Serial.print("° err="); Serial.println(angleCible - deg2s, 2);

    resetEncoders();
}

void testDoura() {
    calcANG(); float angleAvant = angleZ;
    executerVirageAnglePur( 45.0f, false); delay(2000);
    executerVirageAnglePur( 45.0f, false); delay(2000);
    executerVirageAnglePur( 90.0f, false); delay(2000);
    executerVirageAnglePur( 90.0f, false); delay(2000);
    executerVirageAnglePur(180.0f, false);
    calcANG(); float angleApres = angleZ;
    Serial.print("Parcouru: "); Serial.println(fabsf(normaliserAngle(angleApres - angleAvant)), 2);
}
// ═══════════════════════════════════════════════════════════════
// ███  UTILITAIRES / DEBUG  ███
// ═══════════════════════════════════════════════════════════════
float mesurerLoopHz() {
    static unsigned long dernierTemps = 0;
    static unsigned long compteur     = 0;
    static float         frequenceHz  = 0.0f;
    compteur++;
    unsigned long tempsActuel = millis();
    if (tempsActuel - dernierTemps >= 1000) {
        frequenceHz  = (float)compteur;
        Serial.print("Vitesse du loop : ");
        Serial.print(frequenceHz);
        Serial.println(" Hz");
        compteur    = 0;
        dernierTemps = tempsActuel;
    }
    return frequenceHz;
}
// ═══════════════════════════════════════════════════════════════
// TEST PWM → RPM
// ═══════════════════════════════════════════════════════════════
float testPWM(int pwmTarget) {
    resetEncoders(); rpmL = 0; rpmR = 0; AKRA_MASAFA;
    while (MASAFA < 20) {
        forward_brake_fast(pwmTarget > 130 ? 130 : pwmTarget, pwmTarget > 130 ? 130 : pwmTarget);
        updateRPM(); delay(2);
    }
    float rpmSum = 0; int samples = 0;
    while (MASAFA < 120) {
        forward_brake_fast(pwmTarget, pwmTarget);
        updateRPM();
        rpmSum += (fabsf(rpmL) + fabsf(rpmR)) * 0.5f;
        samples++;
        delay(5);
    }
    stopMotors();
    float rpmFinal = samples > 0 ? rpmSum / samples : 0;
    Serial.print("PWM "); Serial.print(pwmTarget);
    Serial.print(" -> "); Serial.print(rpmFinal, 2); Serial.println(" RPM");
    u8g2.firstPage();
    do {
        char t1[25], t2[25];
        u8g2.setFont(u8g2_font_10x20_tr);
        sprintf(t1, "PWM:%d",    pwmTarget);
        sprintf(t2, "RPM:%.1f", rpmFinal);
        u8g2.drawStr(0, 22, t1);
        u8g2.drawStr(0, 52, t2);
    } while (u8g2.nextPage());
    return rpmFinal;
}
#endif
