#include <ax12.h>

#define SERVO_ID 3
#define START_POSITION 0
#define DESIRED_POSITION 512

bool LQR = true;
float
  PREVIOUS_TIME = 0,
  X_1 = 0,
  X_2 = 0,
  X_3 = 0,
  X_4 = 0,
  X_5 = 0,
  AX_1 = 0,
  AX_2 = 0,
  AX_3 = 0,
  AX_4 = 0,
  AX_5 = 0,
  CX_SUM = 0,
  LCX_1 = 0,
  LCX_2 = 0,
  LCX_3 = 0,
  LCX_4 = 0,
  LCX_5 = 0,
  KX = 0;

void calculateLQR(float current_angle, float desired_angle) {
  // Setup for integration X_dot -> x
  float timeElapsed = (millis() - PREVIOUS_TIME) / 1000;
  float error = (desired_angle - KX);
  
  // Calculate X_dot using previous calculations
  float X_dot_1 = error + AX_1 + LCX_1;
  float X_dot_2 = AX_2 + LCX_2;
  float X_dot_3 = AX_3 + LCX_3;
  float X_dot_4 = AX_4 + LCX_4;
  float X_dot_5 = AX_5 + LCX_5;
  
  // Calculate x as integral of X_dot
  X_1 += timeElapsed * X_dot_1;
  X_2 += timeElapsed * X_dot_2;
  X_3 += timeElapsed * X_dot_3;
  X_4 += timeElapsed * X_dot_4;
  X_5 += timeElapsed * X_dot_5;
  
  // Get values for future calculations
  AX_1 = (-31.54*X_1) + (-952.9*X_2) + (-15010*X_3)
      + (-150900*X_4) + (-697100*X_5);
  AX_2 = X_1;
  AX_3 = X_2;
  AX_4 = X_3;
  AX_5 = X_4;
  CX_SUM = current_angle - ((2828*X_3) + (-2665*X_4) + (694900*X_5));
  LCX_1 = 0.00000000000012732123704458735732976732752067 * CX_SUM;
  LCX_2 = -0.0000000000000010261023036638274263905339987485 * CX_SUM;
  LCX_3 = -0.00000000000000013709883892126410487014874164629 * CX_SUM;
  LCX_4 = 0.000000000000000003681017957841670441530323011391 * CX_SUM;
  LCX_5 = -0.00000000000000000040899759673672038057085748671409 * CX_SUM;
  KX = (4.6136936680715328051860524283256 * X_1) + (106.15898292237854150243947515264 * X_2)
      + (1185.898833741578528133686631918 * X_3) + (5382.4054598786433416535146534443 * X_4)
      + (0.000071727101930481917458660989783681 * X_5);
  PREVIOUS_TIME = millis();
}

void setup() {
  Serial.begin(115200);
  ax12Init(1000000);
  // go to start position
  SetPosition(SERVO_ID, START_POSITION);
  delay(1500);
  PREVIOUS_TIME = millis();
}

void loop() {
  // Go to desired position w/o LQR
  if (!LQR) {
    SetPosition(SERVO_ID, DESIRED_POSITION);
  }
  // Go to desired position w/ LQR
  else {
    SetPosition(SERVO_ID, DESIRED_POSITION - KX);
    calculateLQR(GetPosition(SERVO_ID), DESIRED_POSITION);
  }
  // Output Data
  if( millis() % 10 == 0 && millis() <= 3000) {      
    Serial.print(millis());
    Serial.print(", ");
    Serial.println(GetPosition(SERVO_ID));
  }
}
