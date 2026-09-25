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
// VERSION ALIGNÉE SUR pins_encoders.h (config qui fonctionne) :
// - même filtre (100)
// - même lctrl/hctrl pour les 2 unités (REVERSE / KEEP)
// - pas d'ISR overflow, pas d'accumulateur logiciel
// - counter_h_lim/l_lim sur toute la plage int16 (32767/-32768)
// L'interface (macros AKRA_MASAFA, TICKS_L/R, MASAFA, DEBUG_ENCODERS)
// est conservée à l'identique pour rester compatible avec le reste
// du code (bourak.h, etc.).
//
// PCNT Unit 0 → Encodeur GAUCHE
// PCNT Unit 1 → Encodeur DROIT
// ============================================================

// ==========================================
// PINS (identiques à l'ancienne version)
// ==========================================
#define ENC_L_PIN_A 16
#define ENC_L_PIN_B 17
#define ENC_R_PIN_A 19
#define ENC_R_PIN_B 18

// ==========================================
// CONSTANTES
// ==========================================
#define CPR_MOTEUR          48.0f
#define RAPPORT_REDUCTEUR   9.68f
#define DIAMETRE_ROUE_MM    68.0f //48.71
#define TICKS_PAR_TOUR_ROUE 230.0f
long savedEncoderL = 0;
long savedEncoderR = 0;
float cm_par_tick = 0.0f;

// ==========================================
// LECTURE HARDWARE — lecture directe du compteur 16 bits
// (pas d'accumulateur logiciel, comme pins_encoders.h)
// ==========================================
inline long readEncoderL() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    return (long)count;
}

inline long readEncoderR() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_1, &count);
    return (long)count;
}

// ==========================================
// CONFIGURATION D'UN CANAL PCNT
// Même config pour les 2 unités (comme pins_encoders.h) :
// lctrl_mode = REVERSE, hctrl_mode = KEEP
// ==========================================
static void configurePCNT(pcnt_unit_t unit, int pinA, int pinB)
{
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num  = pinA;
    cfg.ctrl_gpio_num   = pinB;
    cfg.unit            = unit;
    cfg.channel         = PCNT_CHANNEL_0;
    cfg.pos_mode        = PCNT_COUNT_INC;
    cfg.neg_mode        = PCNT_COUNT_DEC;
    cfg.lctrl_mode      = PCNT_MODE_REVERSE;
    cfg.hctrl_mode      = PCNT_MODE_KEEP;
    cfg.counter_h_lim   = 32767;
    cfg.counter_l_lim   = -32768;

    pcnt_unit_config(&cfg);

    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// ==========================================
// SETUP — à appeler dans setup()
// ==========================================
void setupEncoders() {
    pinMode(ENC_L_PIN_A, INPUT_PULLUP);
    pinMode(ENC_L_PIN_B, INPUT_PULLUP);
    pinMode(ENC_R_PIN_A, INPUT_PULLUP);
    pinMode(ENC_R_PIN_B, INPUT_PULLUP);

    configurePCNT(PCNT_UNIT_0, ENC_L_PIN_A, ENC_L_PIN_B);
    configurePCNT(PCNT_UNIT_1, ENC_R_PIN_A, ENC_R_PIN_B);

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