  #include "bourak.h"
  #include "encoder.h"
  #include "coap_telemetry.h"
  bool finished = false;
  int g=0;
  void setup() {
    Serial.begin(115200);

    Wire.begin(21, 22);
    u8g2.begin();
    afficherTexte("TestBourak", 1);
    delay(1000);

    setupMPU();
    calibrateGyro();

    initMotors();
    setupEncoders();
    pid.begin();

    coapInit();

    AKRA_MASAFA;
    resetEncoders();

    afficherTexte("Pret!", 2);
    delay(3000);

    initMPU_PID(130, 200, 0.0f);
  }


  
  void loop() {
    if (finished) return;

    float dist = MASAFA;
    if (g== 0 && dist >= 40.0f){
      initMPU_PID(240, 230, 0.0f);
      g++;
    }

      if (g== 1 && dist >= 90.0f){
      initMPU_PID(255, 255, 0.0f);
      g++;
    }

    if (dist >= 150.0f) {
      stopMPU_PID();
      finished = true;
      afficherTexte("STOP", 1);
      return;
    }

    runMPU_PID();
  mesurerLoopHz();
  /*
   coapSendVirage(millis(), angleZ, TICKS_L, TICKS_R,
               rpmL, rpmR, pwmL, pwmR, erreurAngle);
  
*/
  }