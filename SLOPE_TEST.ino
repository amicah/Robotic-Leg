

//#include <ILQR.h>
//#include <standardIncludes.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <ax12.h>
#include <Servo.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>

#endif

// ============================================= VARIABLES BEGIN =============================================
#define SERIAL_ENABLE 1                                 // Toggle Serial communications
#define INTERRUPT_PIN 2                                 // Pin 2 on Arbotix-M
#define INT_LED_PIN 0                                   // Blinks on interrupt
#define CAL_LED_PIN 18                                  // LED on when calibrating
#define BAL_LED_PIN 19                                  // LED on when robot is balanced
#define NUM_SERVOS 4                                    // The number of AX-12A servos
#define ACTUATOR_PIN 15                                 // Data pin for linear actuator
#define X_GYRO_OFFSET 50                                // Gyroscope offsets found using IMU_Zero
#define Y_GYRO_OFFSET -31
#define Z_GYRO_OFFSET 36
#define X_ACCEL_OFFSET -1328                            // Accelerometer offsets found using IMU_Zero
#define Y_ACCEL_OFFSET -4218
#define Z_ACCEL_OFFSET 1008
#define ANGLE_OFFSET 90                                 // 90 degree offset for trig. calculations
#define CLOCK_FREQUENCY 4e5                             // 400 kHz I2C Clock
#define CALIBRATION_DELAY 25e3                          // Time to let the system calibrate before balancing begins
#define LINK_LENGTH_1 375.92                            // Link length L1 in millimeters
#define LINK_LENGTH_2 396.24                            // Link length L2 in millimeters
#define SERVO2_MIN 430                                  // Servo 2 reverse angle (Hip Joint)
#define SERVO2_MAX 583                                  // Servo 2 forward angle (Hip Joint)
#define SERVO3_MIN 422                                  // Servo 3 forward angle (Knee Joint)
#define SERVO3_MAX 601                                  // Servo 3 reverse angle (Knee Joint)
#define SERVO4_MIN 232                                  // Servo 4 right-side angle (Ankle Joint)
#define SERVO4_MAX 783                                  // Servo 4 left-side angle (Ankle Joint)
#define ACTUATOR_MIN 1050                               // Linear actuator upward angle (Foot Joint)
#define ACTUATOR_MAX 2000                               // Linear actuator downward angle (Foot Joint)
#define ACTUATOR_DEGREE 17                              // Linear actuator 1 degree of movement in PWM
#define ACTUATOR_ZERO 1390                              // Linear actuator at 0 degrees (flat with floor)

Servo LINEAR;                                           // Linear Actuator class object
MPU6050 MPU;                                            // MPU6050 class object

volatile bool
    MPU_INTERRUPT = false;                              // True on MPU interrupt
bool
    BLINK_STATE,                                        // Blink LED during MPU interrupt
    CAL_DONE = false,                                   // True if calibration time is complete
    DMP_READY,                                          // Set true if DMP init was successful
    IS_BALANCED = false;                                // True if robot is upright
uint8_t
    MPU_INT_STATUS,                                     // Holds actual interrupt status byte from MPU
    DEV_STATUS,                                         // Return status after each device operation (0 = success)
    FIFO_BUFFER[64],                                    // FIFO storage buffer
    SERVO_ID[NUM_SERVOS] = { 4, 3, 2, 1 };              // Designated ID numbers for the AX-12A servos
uint16_t
    PACKET_SIZE,                                        // Expected DMP packet size (default is 42 bytes)
    FIFO_COUNT;                                         // Count of all bytes currently in FIFO
Quaternion
    Q;                                                  // [w, x, y, z] -->  Quaternion container
VectorFloat
    GRAVITY;                                            // [x, y, z]  -->  Gravity vector
float
    START_TIME = 0,                                     // Timing and delays
    PREVIOUS_TIME[3] = { 0, 0, 0 },                     // Timing used in control system
    PRINCIPAL_AXES[3],                                  // [yaw, pitch, roll] Container and gravity vector
    PITCH = 0,                                          // Anteroposterior angle
    ROLL = 0,                                           // Mediolateral angle
    PREVIOUS_PITCH = 0,                                 // The previous anteroposterior angle
    PREVIOUS_ROLL = 0,                                  // The previous mediolateral angl
    PITCH_AP_CONSTANT = 2.1185,                         // Slope between two anteroposterior balance points wrt Pitch
    ROLL_ML_CONSTANT = -2.5758,                         // Slope between two mediolateral balance points wrt Roll
    PREVIOUS_ANGLES[3] = { 0, 0, 0 },                   // Joint angles from previous iteration
    CURRENT_ANGLES[3] = { 0, 0, 0 },                    // Joint angles for current iteration
    PREVIOUS_POSITION[3] = { 0, 0, 0 },                 // End effector position from previous iteration
    CURRENT_POSITION[3] = { 0, 0, 0 },                  // End effector position for current iteration
    DESIRED_POSITION[3] = { 0, 0, 0 },                  // Desired end effector position
    THETA_DOT[3] = { 0, 0, 0 },                         // Change in angles (calculated)
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

// ============================================= FUNCTIONS BEGIN =============================================
void dmpDataReady() {
  MPU_INTERRUPT = true;
}

float sind(float value) {
  return sin(value * M_PI /180);
}

float cosd(float value) {
  return cos(value * M_PI / 180);
}

float asind(float value) {
  return (float)(asin(value) * 180 / M_PI);
}

float acosd(float value) {
  return (float)(acos(value) * 180 / M_PI);
}

void readGyroscope() {
  // Reset interrupt flag and get INT_STATUS byte
      MPU_INTERRUPT = false;
      MPU_INT_STATUS = MPU.getIntStatus();
      
      // Get current FIFO count
      FIFO_COUNT = MPU.getFIFOCount();
      
      // Check for overflow
      if ((MPU_INT_STATUS & 0x10) || FIFO_COUNT == 1024) {
        MPU.resetFIFO();
        if (SERIAL_ENABLE) Serial.println(F("FIFO overflow!"));
      }
      
      // Otherwise, check for DMP data ready interrupt
      else if (MPU_INT_STATUS & 0x02) {
        
        // Wait for correct available data length
        while (FIFO_COUNT < PACKET_SIZE) FIFO_COUNT = MPU.getFIFOCount();
        MPU.getFIFOBytes(FIFO_BUFFER, PACKET_SIZE);
        
        // Track FIFO count here in case there is > 1 packet available
        FIFO_COUNT -= PACKET_SIZE;
        
        // Display Euler angles in degrees
        MPU.dmpGetQuaternion(&Q, FIFO_BUFFER);
        MPU.dmpGetGravity(&GRAVITY, &Q);
        MPU.dmpGetYawPitchRoll(PRINCIPAL_AXES, &Q, &GRAVITY);
        // Update previous Pitch and Roll values
        PREVIOUS_PITCH = PITCH;
        PREVIOUS_ROLL = ROLL;
        
        // Get PITCH and ROLL in degrees
        ROLL = PRINCIPAL_AXES[1] * 180 / M_PI;
        PITCH = PRINCIPAL_AXES[2] * 180 / M_PI;
        
        // Blink LED to indicate activity
        BLINK_STATE = !BLINK_STATE;
        digitalWrite(INT_LED_PIN, BLINK_STATE);
      }
}

// The AX-12A servos do not move by degree, but by position.
// Positions range from 0 to 1023 and angles range from -60 to 240 degrees
// with the vertical position (512) representing ~90 degrees.
float servo2Degree(float value) {
  return (float)((300 * value) / 1023) - 60;
}

int degree2Servo(float value) {
  return (float)((value + 60) * 1023) / 300;
}

void moveActuator(uint8_t value) {
  value = ACTUATOR_ZERO + (value * ACTUATOR_DEGREE);
  if (value >= ACTUATOR_MIN && value <= ACTUATOR_MAX) {
    LINEAR.writeMicroseconds(value); // Sets the linear actuator based on PWM input
  }
}

// The Jacobian was calculated as 3DOF starting at the ankle and ending at the hip
// Therefore joints z1->z2->z3 become servo IDs 4->3->2 
void updateAngles() {
  // Set previous angles
  PREVIOUS_ANGLES[0] = CURRENT_ANGLES[0];
  PREVIOUS_ANGLES[1] = CURRENT_ANGLES[1];
  PREVIOUS_ANGLES[2] = CURRENT_ANGLES[2];
  // Get current angles
  CURRENT_ANGLES[0] = servo2Degree(GetPosition(SERVO_ID[0]));
  CURRENT_ANGLES[1] = servo2Degree(GetPosition(SERVO_ID[1]));
  CURRENT_ANGLES[2] = servo2Degree(GetPosition(SERVO_ID[2]));
}

// These equations found using transformation matrices
void updatePositions() {
  // Set previous positions
  PREVIOUS_POSITION[0] = CURRENT_POSITION[0];
  PREVIOUS_POSITION[1] = CURRENT_POSITION[1];
  PREVIOUS_POSITION[2] = CURRENT_POSITION[2];
  // Get current positions
  CURRENT_POSITION[0] = cosd(CURRENT_ANGLES[0]-ANGLE_OFFSET)*(LINK_LENGTH_1+LINK_LENGTH_2*cosd(CURRENT_ANGLES[1]-ANGLE_OFFSET));
  CURRENT_POSITION[1] = sind(CURRENT_ANGLES[0]-ANGLE_OFFSET)*(LINK_LENGTH_1+LINK_LENGTH_2*cosd(CURRENT_ANGLES[1]-ANGLE_OFFSET));
  CURRENT_POSITION[2] = LINK_LENGTH_2*sind(CURRENT_ANGLES[1]-ANGLE_OFFSET);
}

// DESIRED_POSITION will be determined by appropriate anteroposterior (AP) and mediolateral (ML) movment to find
// a suitable height (H) position where DESIRED_POSITION = [ H; ML; AP ]
void calculateDESIRED_POSITION() {
  // Find AP and ML based on current ROLL and PITCH angles
  float AP_DESIRED = PITCH_AP_CONSTANT * PITCH;
  float ML_DESIRED = ROLL_ML_CONSTANT * ROLL;
  // Solve for theta2 using ML formula
  float theta2 = asind(ML_DESIRED/LINK_LENGTH_2) + ANGLE_OFFSET;
  // Use theta2 to find theta1 using AP formula
  float theta1 = asind(AP_DESIRED/(LINK_LENGTH_1+LINK_LENGTH_2*cosd(theta2-ANGLE_OFFSET))) + ANGLE_OFFSET;
  // Use theta1 and theta2 to find H
  float H_DESIRED = cosd(theta1-ANGLE_OFFSET)*(LINK_LENGTH_1+LINK_LENGTH_2*cosd(theta2-ANGLE_OFFSET));
  // Set the DESIRED_POSITION
  DESIRED_POSITION[0] = H_DESIRED;
  DESIRED_POSITION[1] = ML_DESIRED;
  DESIRED_POSITION[2] = AP_DESIRED;
}

// These equations found using Pseudo-Inverse Jacobian
void calculateTHETA_DOT() {
  // Shorthand
  float s1 = sind(CURRENT_ANGLES[0]-ANGLE_OFFSET);
  float c1 = cosd(CURRENT_ANGLES[0]-ANGLE_OFFSET);
  float s2 = sind(CURRENT_ANGLES[1]-ANGLE_OFFSET);
  float c2 = cosd(CURRENT_ANGLES[1]-ANGLE_OFFSET);
  float L1 = LINK_LENGTH_1;
  float L2 = LINK_LENGTH_2;
  // Find values for V_DOT
  float Xv = DESIRED_POSITION[0] - CURRENT_POSITION[0];
  float Yv = DESIRED_POSITION[1] - CURRENT_POSITION[1];
  float Zv = DESIRED_POSITION[2] - CURRENT_POSITION[2];
  // Find cell values for inverse(J)
  float ij11 = (-s1*(L1 + L2*c2)) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  float ij21 = (-c1*s2) / L2;
  float ij31 = s1 / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  float ij12 = (c1*(L1 + L2*c2)) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  float ij22 = (-s1*s2) / L2;
  float ij32 = (-c1) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  float ij13 = 0;
  float ij23 = c2 / L2;
  float ij33 = 0;
  // THETA_DOT = inverse(J) * V_DOT
  THETA_DOT[0] = (ij11*Xv + ij12*Yv + ij13*Zv)*(180/M_PI); // Ankle
  THETA_DOT[1] = (ij21*Xv + ij22*Yv + ij23*Zv)*(180/M_PI); // Knee
  THETA_DOT[2] = (ij31*Xv + ij32*Yv + ij33*Zv)*(180/M_PI); // Hip
}

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

// =============================================== SETUP BEGIN ===============================================
void setup() {
  if (SERIAL_ENABLE) {
    Serial.begin(115200);
  }
  // Initialize MPU6050
  BLINK_STATE = false;
  DMP_READY = false;
  if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE) {
    Wire.begin();
    Wire.setClock(CLOCK_FREQUENCY);
  }
  // Setup gyroscope
  MPU.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  DEV_STATUS = MPU.dmpInitialize();
  MPU.setXGyroOffset(X_GYRO_OFFSET);
  MPU.setYGyroOffset(Y_GYRO_OFFSET);
  MPU.setZGyroOffset(Z_GYRO_OFFSET);
  MPU.setXAccelOffset(X_ACCEL_OFFSET);
  MPU.setYAccelOffset(Y_ACCEL_OFFSET);
  MPU.setZAccelOffset(Z_ACCEL_OFFSET);
  if (DEV_STATUS == 0) {
    MPU.setDMPEnabled(true);
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
    MPU_INT_STATUS = MPU.getIntStatus();
    DMP_READY = true;
    PACKET_SIZE = MPU.dmpGetFIFOPacketSize();
  }
  else {
    if (SERIAL_ENABLE) {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(DEV_STATUS);
      Serial.println(F(")"));
    }
  }
  // Setup AX-12A servos
  ax12Init(1000000);

    
  // Set linear actuator to 0 degree position
  LINEAR.attach(ACTUATOR_PIN, ACTUATOR_MIN, ACTUATOR_MAX);
  moveActuator(0);
  // Setup LEDs
  pinMode(INT_LED_PIN, OUTPUT);
  pinMode(CAL_LED_PIN, OUTPUT);
  pinMode(BAL_LED_PIN, OUTPUT);
  START_TIME = millis();
  PREVIOUS_TIME[0] = START_TIME;
  PREVIOUS_TIME[1] = START_TIME;
  PREVIOUS_TIME[2] = START_TIME;
  
  // ========= FOR SERVO TESTING ===========
  // Simple balance on flat surface: [ 90 90 82 90 ]
  // Uncomment for servo fixed position
  SetPosition(1, degree2Servo(90));
  SetPosition(2, degree2Servo(90));
  SetPosition(3, degree2Servo(82));
  SetPosition(4, degree2Servo(90));

  // Uncomment for servo free movment
  //Relax(1);
  //Relax(2);
  //Relax(3);
  //Relax(4);
  // ========= FOR SERVO TESTING ===========
}

// =============================================== LOOP BEGIN ================================================
void loop() {
  // ========= FOR SERVO TESTING ===========
  // Simple balance on flat surface: [ 90 90 82 90 ]
  // Uncomment for servo fixed position
  SetPosition(1, degree2Servo(90));
  SetPosition(2, degree2Servo(90));
  SetPosition(3, degree2Servo(82));
  SetPosition(4, degree2Servo(90));
  // ========= FOR SERVO TESTING ===========
  
  // Check if calibration time has completed
  CAL_DONE = millis() - START_TIME >= CALIBRATION_DELAY;
  digitalWrite(CAL_LED_PIN, !CAL_DONE);

  // --------------- Read Sensor Data ---------------
  if (DMP_READY) {
    bool mpuReady = !(!MPU_INTERRUPT && FIFO_COUNT < PACKET_SIZE);
    if (mpuReady) {
      readGyroscope();
      
  // ----------------- Debug Output -----------------
  // Only compiles if SERIAL_ENABLE is defined above
  if (SERIAL_ENABLE) {
    Serial.print("[PITCH,ROLL]=[");
    Serial.print(PITCH); Serial.print(", ");
    Serial.print(ROLL); Serial.print("];  ");
    if (CAL_DONE) {
      Serial.print("[4,3,2]=[");
      Serial.print(CURRENT_ANGLES[0]); Serial.print(", ");
      Serial.print(CURRENT_ANGLES[1]); Serial.print(", ");
      Serial.print(CURRENT_ANGLES[2]); Serial.print("];  ");
      Serial.print("[H;ML;AP]=[");
      Serial.print(CURRENT_POSITION[0]); Serial.print("; ");
      Serial.print(CURRENT_POSITION[1]); Serial.print("; ");
      Serial.print(CURRENT_POSITION[2]); Serial.println("];");      
    }
    else {
      Serial.println(" ~calibrating~ ");
    }
  }
    }
  }
  
  // ------------ Control and Positioning -----------  
  if (CAL_DONE) {
    // Control servos here
    updateAngles();
    updatePositions();
    calculateDESIRED_POSITION();
    calculateTHETA_DOT();
    /*
    for (int i = 0; i < NUM_SERVOS-1; ++i) {
      float desiredPosition = degree2Servo(CURRENT_ANGLES[i]-ANGLE_OFFSET+THETA_DOT[i]);
      SetPosition(SERVO_ID[i], desiredPosition-KX[i]);  
      delay(10);
      calculateLQR(GetPosition(SERVO_ID[i]), desiredPosition, i);
    }
    */
    // Determine if balance has been achieved
    IS_BALANCED = abs(PITCH-PREVIOUS_PITCH) <= 1 && abs(ROLL-PREVIOUS_ROLL) <= 1;
    digitalWrite(BAL_LED_PIN, IS_BALANCED);
  }
}
