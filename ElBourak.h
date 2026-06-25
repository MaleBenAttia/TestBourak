#ifndef ELBOURAK_H
#define ELBOURAK_H

#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ═══════════════════════════════════════════════════════════════
// NB_CAPT fixé à 14
// ═══════════════════════════════════════════════════════════════
#define EB_NB_CAPT 14

class ElBourak {
public:
    int      Tab[EB_NB_CAPT];      // valeurs calibrées 0-1000
    int      Tab1[EB_NB_CAPT];     // valeurs brutes 0-4095
    int      minValue[EB_NB_CAPT];
    int      maxValue[EB_NB_CAPT];
    int      lastPosition;

private:
    uint8_t  _pinSIG;
    uint8_t  _pinS0, _pinS1, _pinS2, _pinS3;
    uint32_t _maskS0, _maskS1, _maskS2, _maskS3, _allS;
    uint32_t _muxMasks[EB_NB_CAPT];

    // canal ADC1 précalculé depuis le pin SIG
    adc1_channel_t _adcChannel;

    // mapping GPIO → adc1_channel_t (pins ADC1 valides sur ESP32)
    static adc1_channel_t _pinToAdc1(uint8_t pin) {
        switch (pin) {
            case 36: return ADC1_CHANNEL_0;
            case 37: return ADC1_CHANNEL_1;
            case 38: return ADC1_CHANNEL_2;
            case 39: return ADC1_CHANNEL_3;
            case 32: return ADC1_CHANNEL_4;
            case 33: return ADC1_CHANNEL_5;
            case 34: return ADC1_CHANNEL_6;
            case 35: return ADC1_CHANNEL_7;
            default: return ADC1_CHANNEL_0;
        }
    }

public:
    ElBourak(uint8_t pinSIG, uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3)
        : lastPosition(6500),
          _pinSIG(pinSIG), _pinS0(s0), _pinS1(s1), _pinS2(s2), _pinS3(s3)
    {
        _maskS0 = (1UL << s0);
        _maskS1 = (1UL << s1);
        _maskS2 = (1UL << s2);
        _maskS3 = (1UL << s3);
        _allS   = _maskS0 | _maskS1 | _maskS2 | _maskS3;

        // précalcul masques MUX pour 14 capteurs
        for (uint8_t i = 0; i < EB_NB_CAPT; i++) {
            uint32_t m = 0;
            if (i & 0x01) m |= _maskS0;
            if (i & 0x02) m |= _maskS1;
            if (i & 0x04) m |= _maskS2;
            if (i & 0x08) m |= _maskS3;
            _muxMasks[i] = m;
        }

        for (int i = 0; i < EB_NB_CAPT; i++) {
            minValue[i] = 4095;
            maxValue[i] = 0;
        }

        _adcChannel = _pinToAdc1(pinSIG);
    }

    // ── INIT ────────────────────────────────────────────────────
    void begin() {
        pinMode(_pinS0, OUTPUT);
        pinMode(_pinS1, OUTPUT);
        pinMode(_pinS2, OUTPUT);
        pinMode(_pinS3, OUTPUT);

        // ADC1 config directe — une seule fois
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(_adcChannel, ADC_ATTEN_DB_11);
    }

    // ── LECTURE BRUTE OPTIMALE ──────────────────────────────────
    // GPIO registres directs + adc1_get_raw (sans overhead Arduino)
    // ~8-12µs par capteur au lieu de ~25-40µs avec analogRead
    void readRawAll() {
        for (uint8_t i = 0; i < EB_NB_CAPT; i++) {
            GPIO.out_w1tc = _allS;
            GPIO.out_w1ts = _muxMasks[i];
            // 4 NOPs = ~50ns, suffisant pour 74HC4051 (tpd ~15ns typ)
            asm volatile("nop;nop;nop;nop;");
            Tab1[i] = adc1_get_raw(_adcChannel);
            Tab[i]  = Tab1[i];
        }
    }

    // ── CALIBRATION ─────────────────────────────────────────────
    void calibrateSensors() {
        readRawAll();
        for (int i = 0; i < EB_NB_CAPT; i++) {
            if (Tab1[i] < minValue[i]) minValue[i] = Tab1[i];
            if (Tab1[i] > maxValue[i]) maxValue[i] = Tab1[i];
        }
    }

    // ── LECTURE CALIBRÉE ─────────────────────────────────────────
    void readCalibrated() {
        readRawAll();
        for (int i = 0; i < EB_NB_CAPT; i++) {
            int range = maxValue[i] - minValue[i];
            if (range == 0) { Tab[i] = 0; continue; }
            int v = ((Tab1[i] - minValue[i]) * 1000) / range;
            Tab[i] = v < 0 ? 0 : (v > 1000 ? 1000 : v);
        }
    }

    // ── POSITION LIGNE BLANCHE — FAST ───────────────────────────
    // setpoint = (NB_CAPT-1)*1000/2 = 6500 pour 14 capteurs
    int ReadLineWhiteFast() {
        readCalibrated();
        unsigned long avg = 0, sum = 0;
        bool onLine = false;
        for (int i = 0; i < EB_NB_CAPT; i++) {
            if (Tab[i] > 200) onLine = true;
            avg += (unsigned long)Tab[i] * (i * 1000);
            sum += Tab[i];
        }
        if (!onLine) return (lastPosition < 6500) ? 0 : 13000;
        lastPosition = (int)(avg / sum);
        return lastPosition;
    }

    // ── POSITION LIGNE NOIRE — FAST ─────────────────────────────
    int ReadLineBlackFast() {
        readCalibrated();
        unsigned long avg = 0, sum = 0;
        bool onLine = false;
        for (int i = 0; i < EB_NB_CAPT; i++) {
            int val = 1000 - Tab[i];
            if (val > 200) onLine = true;
            avg += (unsigned long)val * (i * 1000);
            sum += val;
        }
        if (!onLine) return (lastPosition < 6500) ? 0 : 13000;
        lastPosition = (int)(avg / sum);
        return lastPosition;
    }

    // ── DIGITAL ─────────────────────────────────────────────────
    void readDigitalAll() {
        readRawAll();
        for (int i = 0; i < EB_NB_CAPT; i++)
            Tab1[i] = (Tab1[i] > 500) ? 1 : 0;
    }

    void readDigitalAllL() {
        // capteurs 0..6 (moitié gauche)
        for (uint8_t i = 0; i < EB_NB_CAPT / 2; i++) {
            GPIO.out_w1tc = _allS;
            GPIO.out_w1ts = _muxMasks[i];
            asm volatile("nop;nop;nop;nop;");
            Tab1[i] = (adc1_get_raw(_adcChannel) > 500) ? 1 : 0;
        }
    }

    void readDigitalAllR() {
        // capteurs 7..13 (moitié droite)
        for (uint8_t i = EB_NB_CAPT / 2; i < EB_NB_CAPT; i++) {
            GPIO.out_w1tc = _allS;
            GPIO.out_w1ts = _muxMasks[i];
            asm volatile("nop;nop;nop;nop;");
            Tab1[i] = (adc1_get_raw(_adcChannel) > 500) ? 1 : 0;
        }
    }
};

#endif