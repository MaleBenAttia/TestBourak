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
long t1 = 0;
int x = 0;
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  delay(200);
  pinMode(LED_BTN, INPUT_PULLDOWN);
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

  while (digitalRead(LED_BTN) == 0) {
    afficherBarresCapteurs(0);
    delay(10);
  }


  // ===== Calibration capteurs IR (passer le robot sur la ligne) =====
  afficherTexte("Calibration!", 1);

  unsigned long t_cal = millis();
  forward_brake_fast(80, -80);
  while (millis() - t_cal < 4000) {
    pid.calibrateSensors();
    delay(10);
  }
  forward_brake_fast(0, 0);
  calculerSeuils();
  delay(300);
  while (digitalRead(LED_BTN) == 0) {
    afficherBarresCapteurs(1);
    delay(10);
  }
  delay(300);
  while (digitalRead(LED_BTN) == 0) {
    afficherBarresCapteurs(2);
    delay(10);
  }

  AKRA_MASAFA;
  resetEncoders();
  Serial.println("[SETUP] Appuie sur le bouton pour lancer le test moteur...");
  while (digitalRead(LED_BTN) == 0) {
    delay(50);
    Serial.println("ok");
  }

  afficherTexte("Pret!", 2);
  delay(3000);
  eteindreOLED();

  resetEncoders();
  t1 = millis();
}


void loop() {

/*
  PID_controlB_fast(0, 7500);
  if ((millis() - t1) > 1000) {
    x++;
  }
  if (x == 1) {
    while (1) {
      stopMotors();
    }
  }
  mesurerLoopHz();
  */
}
