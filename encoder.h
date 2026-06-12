#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "driver/pcnt.h"

// TICKS PAR DEGRE : 2.44
// 30°  →  73 ticks
// 45°  → 110 ticks
// 90°  → 220 ticks
// 120° → 293 ticks
// 130° → 318 ticks
// 180° → 440 ticks
// ============================================================
// MOTEUR : CQRobot CQR25D — 9.68:1 Metal DC Gearmotor w/Encoder
//
// ✅ VERSION PCNT HARDWARE — ZERO TICK PERDU
// Le comptage est fait par le hardware ESP32 (PCNT),
// complètement indépendant du CPU, du ADC, de l'I2C.
// analogRead() peut bloquer les IRQ autant qu'il veut :
// le PCNT continue de compter dans son registre matériel.
//
// PCNT Unit 0 → Encodeur GAUCHE
// PCNT Unit 1 → Encodeur DROIT
//
// Mode : 1× quadrature (RISING sur A, B donne le sens)
// Ticks par tour roue : 100 (valeur calibrée empiriquement)
// ============================================================

// ==========================================
// PINS (identiques à l'ancienne version)
// ==========================================
#define ENC_L_PIN_A 35
#define ENC_L_PIN_B 33
#define ENC_R_PIN_A 34
#define ENC_R_PIN_B 32

// ==========================================
// CONSTANTES
// ==========================================
#define CPR_MOTEUR          48.0f
#define RAPPORT_REDUCTEUR   9.68f
#define DIAMETRE_ROUE_MM    48.71f
#define TICKS_PAR_TOUR_ROUE 240.0f

// Seuil overflow PCNT (16-bit signé : max 32767)
// On utilise ±30000 pour avoir de la marge
#define PCNT_OVERFLOW  30000

// ==========================================
// ACCUMULATEURS SOFTWARE (pour gérer l'overflow 16-bit du PCNT)
// Mis à jour uniquement lors du dépassement de ±30000
// ==========================================
volatile long pcntAccumL = 0;
volatile long pcntAccumR = 0;
long savedEncoderL = 0;
long savedEncoderR = 0;
float cm_par_tick = 0.0f;

// ==========================================
// ISR PCNT — appelée UNIQUEMENT sur overflow (±30000)
// Contrairement à l'ancienne ISR, elle ne s'exécute
// que ~1 fois par tour de roue → charge CPU quasi nulle
// ==========================================
static void IRAM_ATTR pcntIsrHandler(void* arg) {
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_0, &status);
    if (status & PCNT_EVT_H_LIM) pcntAccumL += PCNT_OVERFLOW;
    if (status & PCNT_EVT_L_LIM) pcntAccumL -= PCNT_OVERFLOW;

    pcnt_get_event_status(PCNT_UNIT_1, &status);
    if (status & PCNT_EVT_H_LIM) pcntAccumR += PCNT_OVERFLOW;
    if (status & PCNT_EVT_L_LIM) pcntAccumR -= PCNT_OVERFLOW;
}

// ==========================================
// LECTURE HARDWARE — inline, sans désactiver les IRQ
// ==========================================
inline long readEncoderL() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    return pcntAccumL + count;
}

inline long readEncoderR() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_1, &count);
    return pcntAccumR + count;
}

// ==========================================
// CONFIGURATION D'UN CANAL PCNT
// ==========================================
static void configurePCNT(pcnt_unit_t unit,
                           int pinA, int pinB,
                           pcnt_ctrl_mode_t lctrl,
                           pcnt_ctrl_mode_t hctrl)
{
    pcnt_config_t cfg;
    cfg.pulse_gpio_num  = pinA;
    cfg.ctrl_gpio_num   = pinB;
    cfg.unit            = unit;
    cfg.channel         = PCNT_CHANNEL_0;
    cfg.pos_mode        = PCNT_COUNT_INC;   // RISING → incrément
    cfg.neg_mode        = PCNT_COUNT_DEC;   // FALLING → ignoré (mode 1×)
    cfg.lctrl_mode      = lctrl;
    cfg.hctrl_mode      = hctrl;
    cfg.counter_h_lim   = PCNT_OVERFLOW;
    cfg.counter_l_lim   = -PCNT_OVERFLOW;

    pcnt_unit_config(&cfg);

    // Filtre anti-rebond : ignore les pulses < 10 cycles CPU (~125ns)
    pcnt_set_filter_value(unit, 10);
    pcnt_filter_enable(unit);

    // Activer les événements overflow
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// ==========================================
// SETUP — à appeler dans setup()
// Remplace exactement l'ancien setupEncoders()
// ==========================================
void setupEncoders() {
// Activer les Pull-ups internes pour l'encodeur Gauche
    pinMode(ENC_L_PIN_A, INPUT);
    pinMode(ENC_L_PIN_B, INPUT_PULLUP);
    
    // Activer les Pull-ups internes pour l'encodeur Droit
    pinMode(ENC_R_PIN_A, INPUT);
    pinMode(ENC_R_PIN_B, INPUT_PULLUP);

    // --- ENCODEUR GAUCHE (PCNT_UNIT_0) ---
    // Ancien ISR : B==LOW → +1, B==HIGH → -1
    // → quand B (ctrl) est LOW  : KEEP   (compte normalement = INC)
    // → quand B (ctrl) est HIGH : REVERSE (inverse = DEC)
    configurePCNT(PCNT_UNIT_0, ENC_L_PIN_A, ENC_L_PIN_B,
                  PCNT_MODE_KEEP,    // B=LOW  → INC
                  PCNT_MODE_REVERSE  // B=HIGH → DEC
    );

    // --- ENCODEUR DROIT (PCNT_UNIT_1) ---
    // Ancien ISR : B==HIGH → +1, B==LOW → -1  (logique inversée)
    // → quand B (ctrl) est LOW  : REVERSE (inverse = DEC)
    // → quand B (ctrl) est HIGH : KEEP    (compte normalement = INC)
    configurePCNT(PCNT_UNIT_1, ENC_R_PIN_A, ENC_R_PIN_B,
                  PCNT_MODE_REVERSE, // B=LOW  → DEC
                  PCNT_MODE_KEEP     // B=HIGH → INC
    );

    // ISR partagée pour les deux unités PCNT
    pcnt_isr_register(pcntIsrHandler, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);
    pcnt_intr_enable(PCNT_UNIT_1);

    // Calcul cm par tick
    float circonference_cm = (DIAMETRE_ROUE_MM * PI) / 10.0f;
    cm_par_tick = circonference_cm / TICKS_PAR_TOUR_ROUE;
}

// ==========================================
// SAUVEGARDER LE POINT DE DEPART
// (même interface qu'avant, compatible avec tout le code)
// ==========================================
#define AKRA_MASAFA \
  do { \
    savedEncoderL = readEncoderL(); \
    savedEncoderR = readEncoderR(); \
  } while(0)

// ==========================================
// DISTANCE DEPUIS LE POINT SAUVEGARDE
// (macros identiques à l'ancienne version)
// ==========================================

#define TICKS_L   (readEncoderL() - savedEncoderL)
#define TICKS_R   (readEncoderR() - savedEncoderR)

#define MASAFA_L  ((float)(TICKS_L) * cm_par_tick)
#define MASAFA_R  ((float)(TICKS_R) * cm_par_tick)
#define MASAFA    ((MASAFA_L + MASAFA_R) * 0.5f)

// ==========================================
// UTILITAIRES
// ==========================================

inline void getTicksSafe(long &l, long &r) {
    l = readEncoderL();
    r = readEncoderR();
}

inline void resetEncoders() {
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcntAccumL = 0;
    pcntAccumR = 0;
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);
}

inline long ticksPourDistance(float distanceCm) {
    return (long)(distanceCm / cm_par_tick);
}

// ==========================================
// DEBUG serie (identique à l'ancienne version)
// ==========================================
#define DEBUG_ENCODERS \
  do { \
    long _l = readEncoderL(), _r = readEncoderR(); \
    Serial.print("L_abs:"); Serial.print(_l); \
    Serial.print(" | R_abs:"); Serial.print(_r); \
    Serial.print(" | L_delta:"); Serial.print(TICKS_L); \
    Serial.print(" | R_delta:"); Serial.print(TICKS_R); \
    Serial.print(" | Dist_cm:"); Serial.println(MASAFA, 2); \
  } while(0)

  

#endif