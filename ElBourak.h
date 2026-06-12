#ifndef ELBOURAK_H
#define ELBOURAK_H

#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

class ElBourak {
public:
    // Tableaux publics (compatibilité)
    int Tab[16];        // Valeurs calibrées (0-1000)
    int Tab1[16];       // Valeurs brutes (0-4095)
    int minValue[16];
    int maxValue[16];
    int lastPosition;   // Dernière position calculée

    // Paramètres internes
    int _sensorCount;
    int _threshold;      // Seuil pour la détection de ligne (ex: 7500 en position)
    uint8_t _pinSIG;
    uint8_t _pinS0, _pinS1, _pinS2, _pinS3;

    // Masques pour manipulation rapide (précalculés)
    uint32_t _maskS0;
    uint32_t _maskS1;
    uint32_t _maskS2;
    uint32_t _maskS3;
    uint32_t _allS;

    // Tableau des masques MUX précalculés
    uint32_t _muxMasks[16];

    // Constructeur principal
    ElBourak(int sensorCount, int threshold, uint8_t pinSIG,
             uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3)
        : _sensorCount(sensorCount),
          _threshold(threshold),
          _pinSIG(pinSIG),
          _pinS0(s0), _pinS1(s1), _pinS2(s2), _pinS3(s3),
          lastPosition(7500)  // valeur par défaut, peut être modifiée
    {
        // Calcul des masques pour les pins de sélection
        _maskS0 = (1UL << _pinS0);
        _maskS1 = (1UL << _pinS1);
        _maskS2 = (1UL << _pinS2);
        _maskS3 = (1UL << _pinS3);
        _allS = _maskS0 | _maskS1 | _maskS2 | _maskS3;

        // Précalcul des masques pour chaque capteur
        for (uint8_t i = 0; i < 16; i++) {
            uint32_t mask = 0;
            if (i & 0x01) mask |= _maskS0;
            if (i & 0x02) mask |= _maskS1;
            if (i & 0x04) mask |= _maskS2;
            if (i & 0x08) mask |= _maskS3;
            _muxMasks[i] = mask;
        }

        // Initialisation des tableaux
        for (int i = 0; i < 16; i++) {
            minValue[i] = 4095;
            maxValue[i] = 0;
        }
    }

    // Configuration des pins (à appeler dans setup)
    void begin() {
        pinMode(_pinS0, OUTPUT);
        pinMode(_pinS1, OUTPUT);
        pinMode(_pinS2, OUTPUT);
        pinMode(_pinS3, OUTPUT);
        pinMode(_pinSIG, INPUT);
    }

    // Lecture brute ultra-rapide (registres GPIO)
    void readRawAll() {
        for (uint8_t i = 0; i < 16; i++) {
            // Application directe aux registres GPIO
            GPIO.out_w1tc = _allS;           // Clear des 4 pins
            GPIO.out_w1ts = _muxMasks[i];    // Set des pins nécessaires

            delayMicroseconds(1);             // Stabilisation minimale

            Tab1[i] = analogRead(_pinSIG);    // Valeur brute
            Tab[i] = Tab1[i];                 // Par défaut, brut (sera écrasé en calibré)
        }
    }

    // Calibration des min/max
    void calibrateSensors() {
        readRawAll();
        for (int i = 0; i < 16; i++) {
            if (Tab1[i] < minValue[i]) minValue[i] = Tab1[i];
            if (Tab1[i] > maxValue[i]) maxValue[i] = Tab1[i];
        }
    }

    // Lecture calibrée (0-1000)
    void readCalibrated() {
        readRawAll();
        for (int i = 0; i < 16; i++) {
            if (maxValue[i] == minValue[i]) continue;
            long value = (Tab1[i] - minValue[i]) * 1000L / (maxValue[i] - minValue[i]);
            Tab[i] = constrain(value, 0, 1000);
        }
    }

    // Position de la ligne (blanche)
    int getLinePosition() {
        readCalibrated();
        unsigned long avg = 0;
        unsigned long sum = 0;
        bool onLine = false;

        for (int i = 0; i < 16; i++) {
            if (Tab[i] > 200) onLine = true;   // Seuil de détection
            avg += (unsigned long)Tab[i] * (i * 1000);
            sum += Tab[i];
        }

        if (!onLine) return (lastPosition < 7500) ? 0 : 15000;
        lastPosition = avg / sum;
        return lastPosition;
    }

    // Méthodes supplémentaires pour la ligne noire / blanche rapide
    // (à adapter selon votre logique originale)
    int ReadLineWhiteFast() {
        // Exemple : même principe que getLinePosition mais avec seuil différent
        readCalibrated();
        unsigned long avg = 0;
        unsigned long sum = 0;
        bool onLine = false;
        for (int i = 0; i < 16; i++) {
            if (Tab[i] > 200) onLine = true;
            avg += (unsigned long)Tab[i] * (i * 1000);
            sum += Tab[i];
        }
        if (!onLine) return (lastPosition < 7500) ? 0 : 15000;
        lastPosition = avg / sum;
        return lastPosition;
    }

    int ReadLineBlackFast() {
        // Pour ligne noire, on inverse la logique
        readCalibrated();
        unsigned long avg = 0;
        unsigned long sum = 0;
        bool onLine = false;
        for (int i = 0; i < 16; i++) {
            int val = 1000 - Tab[i];   // Inversion
            if (val > 200) onLine = true;
            avg += (unsigned long)val * (i * 1000);
            sum += val;
        }
        if (!onLine) return (lastPosition < 7500) ? 0 : 15000;
        lastPosition = avg / sum;
        return lastPosition;
    }

    // Méthodes numériques (digital)
    void readDigitalAll() {
        readRawAll();
        for (int i = 0; i < 16; i++) {
            Tab1[i] = (Tab1[i] > 500) ? 1 : 0;   // Seuil de basculement
        }
    }

    void readDigitalAllR() {
        readRawAll();
        for (int i = 8; i < 16; i++) {
            Tab1[i] = (Tab1[i] > 500) ? 1 : 0;
        }
    }

    void readDigitalAllL() {
        readRawAll();
        for (int i = 0; i < 8; i++) {
            Tab1[i] = (Tab1[i] > 500) ? 1 : 0;
        }
    }
};

// Pour compatibilité avec l'ancien code, on peut créer un objet global par défaut
// avec les valeurs des macros (si elles sont définies). Mais vous pouvez aussi ne pas le faire.
// Exemple :
// #ifdef S0
// ElBourak pid(16, 7500, SIG_PIN, S0, S1, S2, S3);
// #endif

#endif