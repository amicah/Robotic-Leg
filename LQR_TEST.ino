#include <ax12.h>

#define SERVO_ID 1
#define START_POSITION 0
int DESIRED_POSITION = 512;
int PREVIOUS_POSITION = 0;

float
    START_TIME,                                         // Timing and delays
    PREVIOUS_TIME[3] = { 0, 0, 0 };                     // Timing used in control system
float
    X_1[3] = { 0, 0, 0 },                               // Calculated value of X = int(X_dot) (5 x 1) in the control system
    X_2[3] = { 0, 0, 0 },
    X_3[3] = { 0, 0, 0 },
    X_4[3] = { 0, 0, 0 },
    X_5[3] = { 0, 0, 0 },
    AX_1[3] = { 0, 0, 0 },                              // Calculated value of A*X (5 x 1) in the control system
    AX_2[3] = { 0, 0, 0 },
    AX_3[3] = { 0, 0, 0 },
    AX_4[3] = { 0, 0, 0 },
    AX_5[3] = { 0, 0, 0 },
    CX_SUM[3] = { 0, 0, 0 },                            // Calculated value of C(X-X_sim) in the control system
    LCX_1[3] = { 0, 0, 0 },                             // Calculated value of L*C(X-X_sim) (5 x 1) in the control system
    LCX_2[3] = { 0, 0, 0 },
    LCX_3[3] = { 0, 0, 0 },
    LCX_4[3] = { 0, 0, 0 },
    LCX_5[3] = { 0, 0, 0 },
    KX[3] = { 0, 0, 0 };                                // Calculated value of K*X in the control system

void calculateLQR(float current_angle, float desired_angle, uint8_t id) {
  // Setup for integration X_dot -> x
  float timeElapsed = (millis() - PREVIOUS_TIME[id]) / 1000;
  float error = (desired_angle - KX[id]);
  
  // Calculate X_dot using previous calculations
  float X_dot_1 = error + AX_1[id] + LCX_1[id];
  float X_dot_2 = AX_2[id] + LCX_2[id];
  float X_dot_3 = AX_3[id] + LCX_3[id];
  float X_dot_4 = AX_4[id] + LCX_4[id];
  float X_dot_5 = AX_5[id] + LCX_5[id];
  
  // Calculate x as integral of X_dot
  X_1[id] += timeElapsed * X_dot_1;
  X_2[id] += timeElapsed * X_dot_2;
  X_3[id] += timeElapsed * X_dot_3;
  X_4[id] += timeElapsed * X_dot_4;
  X_5[id] += timeElapsed * X_dot_5;
  
  // Get values for future calculations
  AX_1[id] = (-31.54*X_1[id]) + (-952.9*X_2[id]) + (-15010*X_3[id])
      + (-150900*X_4[id]) + (-697100*X_5[id]);
  AX_2[id] = X_1[id];
  AX_3[id] = X_2[id];
  AX_4[id] = X_3[id];
  AX_5[id] = X_4[id];
  CX_SUM[id] = current_angle - ((2828*X_3[id]) + (-2665*X_4[id]) + (694900*X_5[id]));
  LCX_1[id] = 0.00000000000012732123704458735732976732752067 * CX_SUM[id];
  LCX_2[id] = -0.0000000000000010261023036638274263905339987485 * CX_SUM[id];
  LCX_3[id] = -0.00000000000000013709883892126410487014874164629 * CX_SUM[id];
  LCX_4[id] = 0.000000000000000003681017957841670441530323011391 * CX_SUM[id];
  LCX_5[id] = -0.00000000000000000040899759673672038057085748671409 * CX_SUM[id];
  KX[id] = (4.6136936680715328051860524283256 * X_1[id]) + (106.15898292237854150243947515264 * X_2[id])
      + (1185.898833741578528133686631918 * X_3[id]) + (5382.4054598786433416535146534443 * X_4[id])
      + (0.000071727101930481917458660989783681 * X_5[id]);
  PREVIOUS_TIME[id] = millis();
}

void setup() {
  Serial.begin(115200);
  ax12Init(1000000);
  // go to start position
  SetPosition(SERVO_ID, START_POSITION);
  delay(1500);
  START_TIME = millis();
  PREVIOUS_TIME[0] = START_TIME;
  PREVIOUS_TIME[1] = START_TIME;
  PREVIOUS_TIME[2] = START_TIME;
}

void loop() {
  // go to desired position
  SetPosition(SERVO_ID, DESIRED_POSITION-KX[0]);  
  delay(10);
  calculateLQR(GetPosition(SERVO_ID), DESIRED_POSITION, 0);

  // output information
  if (millis() <= 3000) {
    Serial.print(millis());
    Serial.print(", ");
    Serial.println(KX[0],20);    
  }
}
