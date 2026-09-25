#include "bourak.h"
#include "encoder.h"
#include "coap_telemetry.h"

// ============================================================
// TestBourak — sketch de test matériel complet
// setup() initialise TOUT (I2C robuste anti-hang, moteurs,
// encodeurs, capteurs IR, OLED, télémétrie WiFi/CoAP) pour
// valider le câblage avant d'activer la logique PID complète.
// loop() ne fait qu'un test moteur simple : forward_brake_fast(100,100).
// forward_brake_fast(100, -100); right
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  // ===== I2C (MPU6050 + OLED, bus partagé SDA=21/SCL=22) =====
  Wire.begin(MPU_SDA, MPU_SCL, 100000);
  Wire.setTimeOut(8);  // court, avant même le diagnostic ci-dessous
  u8g2.begin();

  // ===== OLED =====
  delay(1000);
  afficherTexte("TestBourak", 1);
  delay(1000);

  // ===== MPU6050 — I2C robuste (DLPF + anti-hang) =====
  setupMPU();
  calibrateGyro();  // robot IMMOBILE pendant cette étape

  // ===== Moteurs (PWM LEDC) =====
  initMotors();

  // ===== Encodeurs (PCNT hardware) =====
  setupEncoders();

  // ===== Capteurs IR (mux 14 voies) =====
  pid.begin();

// ===== Télémétrie WiFi/CoAP =====
  //coapInit();

  // ===== Calibration capteurs IR (passer le robot sur la ligne) =====
  afficherTexte("Calibration!", 1);
  /*
  unsigned long t_cal = millis();
  while (millis() - t_cal < 3000) {
    pid.calibrateSensors();
    delay(10);
  }
*/
  AKRA_MASAFA;
  resetEncoders();

  // Porte manuelle : le temps de lire le diagnostic ci-dessus avant de
  // lancer les moteurs. INPUT_PULLDOWN suppose un bouton câblé vers
  // 3,3V (repos = LOW, appui = HIGH). Bouton câblé vers GND à la place ?
  // -> mets INPUT_PULLUP et inverse la condition (== 1).
  pinMode(LED_BTN, INPUT_PULLDOWN);
  Serial.println("[SETUP] Appuie sur le bouton pour lancer le test moteur...");
  while (digitalRead(LED_BTN) == 0) {
    delay(50);
    Serial.println("ok");
  }

  afficherTexte("Pret!", 2);
  delay(3000);

  Serial.println("[SETUP] Termine — test moteur en boucle (forward_brake_fast 100,100)");
  resetEncoders();
}




bool testTermine = false;

void loop() {
  
  if (testTermine) { stopMotors(); return; }

  // Initialise le PID UNE SEULE FOIS (verrouille le cap de départ)
  if (!mpuInitialized) {
    initMPU_PID(110, 150, 0.0f);
  }

  float d = fabsf(MASAFA);

  // Ajuste juste la vitesse selon la distance, SANS reset du PID/cap
  if (d < 25.0f) {
    baseSpeed = 140; maxSpeed = 170;
  } else if (d < 50.0f) {
    baseSpeed = 190; maxSpeed = 220;
  } else {
    baseSpeed = 240; maxSpeed = 250;
  }

  if (d >= 240.0f) {
    stopMPU_PID();
    testTermine = true;
    return;
  }

  runMPU_PID();

}
