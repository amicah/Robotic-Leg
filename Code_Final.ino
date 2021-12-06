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
#define SERVO_TORQUE 1023                               // Set the torque value of the servos
#define ACTUATOR_PIN 15                                 // Data pin for linear actuator
#define X_GYRO_OFFSET 50                                // Gyroscope offsets found using IMU_Zero
#define Y_GYRO_OFFSET -31
#define Z_GYRO_OFFSET 36
#define X_ACCEL_OFFSET -1328                            // Accelerometer offsets found using IMU_Zero
#define Y_ACCEL_OFFSET -4218
#define Z_ACCEL_OFFSET 1008
#define PITCH_OFFSET -5.58                              // Average pitch for flat balance
#define ROLL_OFFSET 0.8580                              // Average roll for flat balance
#define ANGLE_OFFSET_1 90                               // 90 degree offset for trig. calculations
#define ANGLE_OFFSET_2 82                               // 82 degree offset for trig
#define CLOCK_FREQUENCY 4e5                             // 400 kHz I2C Clock
#define CALIBRATION_DELAY 30e3                          // Time to let the system calibrate before balancing begins
#define LINK_LENGTH_1 375.92                            // Link length L1 in millimeters
#define LINK_LENGTH_2 396.24                            // Link length L2 in millimeters
#define ACTUATOR_MIN 1050                               // Linear actuator upward angle (Foot Joint)
#define ACTUATOR_MAX 2000                               // Linear actuator downward angle (Foot Joint)
#define ACTUATOR_DEGREE 17                              // Linear actuator 1 degree of movement in PWM
#define ACTUATOR_ZERO 1390                              // Linear actuator at 0 degrees (flat with floor)

Servo LINEAR;                                           // Linear Actuator class object
MPU6050 MPU;                                            // MPU6050 class object

volatile bool
    MPU_INTERRUPT = false,                              // True on MPU interrupt
    LOAD_BAR_A = true,                                  // Loading bar for debugging purposes
    LOAD_BAR_B = true,                                  // Loading bar for debugging purposes
    SERVO_TURN = true;                                  // Control one servo at a time
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
    PREVIOUS_ANGLES[3] = { 0, 0, 0 },                   // Joint angles from previous iteration
    CURRENT_ANGLE[3] = { 0, 0, 0 },                    // Joint angles for current iteration
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
    LQR_GAIN[3] = { 0, 0, 0 };                          // Calculated value of K*X in the control system

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
  return (double)(asin(value) * 180 / M_PI);
}

float acosd(float value) {
  return (double)(acos(value) * 180 / M_PI);
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
        MPU.resetFIFO();
        
        // Track FIFO count here in case there is > 1 packet available
        FIFO_COUNT -= PACKET_SIZE;
        
        // Display Euler angles in degrees
        MPU.dmpGetQuaternion(&Q, FIFO_BUFFER);
        MPU.dmpGetGravity(&GRAVITY, &Q);
        MPU.dmpGetYawPitchRoll(PRINCIPAL_AXES, &Q, &GRAVITY);
        // Update previous Pitch and Roll values
        PREVIOUS_ROLL = ROLL;
        PREVIOUS_PITCH = PITCH;
        
        // Get PITCH and ROLL in degrees
        ROLL = (PRINCIPAL_AXES[1] * 180 / M_PI) + ROLL_OFFSET;
        PITCH = (PRINCIPAL_AXES[2] * 180 / M_PI) + PITCH_OFFSET;

        // Avoid glitches
        ROLL = (abs(ROLL - PREVIOUS_ROLL) <= 50) ? ROLL : PREVIOUS_ROLL;
        PITCH = (abs(PITCH - PREVIOUS_PITCH) <= 50) ? PITCH : PREVIOUS_PITCH;
        
        // Blink LED to indicate activity
        BLINK_STATE = !BLINK_STATE;
        digitalWrite(INT_LED_PIN, BLINK_STATE);
      }
}

// The AX-12A servos do not move by degree, but by position.
// Positions range from 0 to 1023 and angles range from -60 to 240 degrees
// with the vertical position (512) representing ~90 degrees.
float servo2Degree(float value) {
  return (double)((300 * value) / 1023) - 60;
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
void updateCURRENT_ANGLE() {
  // Set previous angles
  PREVIOUS_ANGLES[0] = CURRENT_ANGLE[0];
  PREVIOUS_ANGLES[1] = CURRENT_ANGLE[1];
  PREVIOUS_ANGLES[2] = CURRENT_ANGLE[2];
  
  // Get current angles
  CURRENT_ANGLE[0] = servo2Degree(GetPosition(SERVO_ID[0]));
  CURRENT_ANGLE[1] = servo2Degree(GetPosition(SERVO_ID[1]));
  CURRENT_ANGLE[2] = servo2Degree(GetPosition(SERVO_ID[2]));
}

// These equations found using transformation matrices
void calculateCURRENT_POSITION() {
  // Set previous positions
  PREVIOUS_POSITION[0] = CURRENT_POSITION[0];
  PREVIOUS_POSITION[1] = CURRENT_POSITION[1];
  PREVIOUS_POSITION[2] = CURRENT_POSITION[2];
  
  // Get current positions [H; ML; AP]
  CURRENT_POSITION[0] = cosd(CURRENT_ANGLE[0]-ANGLE_OFFSET_1)*(LINK_LENGTH_1+LINK_LENGTH_2*cosd(CURRENT_ANGLE[1]-ANGLE_OFFSET_2));
  CURRENT_POSITION[1] = sind(CURRENT_ANGLE[0]-ANGLE_OFFSET_1)*(LINK_LENGTH_1+LINK_LENGTH_2*cosd(CURRENT_ANGLE[1]-ANGLE_OFFSET_2));
  CURRENT_POSITION[2] = -LINK_LENGTH_2*sind(CURRENT_ANGLE[1]-ANGLE_OFFSET_2);
}

// DESIRED_POSITION will be determined by appropriate anteroposterior (AP) and mediolateral (ML) movment to find
// a suitable height (H) position where DESIRED_POSITION = [H; ML; AP]
void calculateDESIRED_POSITION() {
  // Find AP and ML based on current ROLL and PITCH angles
  float desiredAP = 0;
  float desiredML = 0;
  if (PITCH >= 0) {
    if (ROLL >= 0) {
      // Average between Left AP and Posterior AP
      desiredAP = ((-6.8525*ROLL - 48.5574) + (-6.8985*PITCH - 48.5520)) / 2;
      // Average between Left ML and Posterior ML
      desiredML = ((-13.3730*ROLL - 9.8118) + (-13.5709*PITCH - 9.8567)) / 2;
    }
    else {
      // Average between Right AP and Posterior AP
      desiredAP = ((-6.8681*ROLL - 48.5452) + (-6.8985*PITCH - 48.5520)) / 2;
      // Average between Right ML and Posterior ML
      desiredML = ((-13.4014*ROLL - 9.8249) + (-13.5709*PITCH - 9.8567)) / 2;
    }
  }
  else {
    if (ROLL >= 0) {
      // Average between Left AP and Anterior AP
      desiredAP = ((-6.8525*ROLL - 48.5574) + (-6.7476*PITCH - 48.7425)) / 2;
      // Average between Left ML and Anterior ML
      desiredML = ((-13.3730*ROLL - 9.8118) + (-12.8560*PITCH - 9.7597)) / 2;
    }
    else {
      // Average between Right AP and Anterior AP
      desiredAP = ((-6.8681*ROLL - 48.5452) + (-6.7476*PITCH - 48.7425)) / 2;
      // Average between Right ML and Anterior ML
      desiredML = ((-13.4014*ROLL - 9.8249) + (-12.8560*PITCH - 9.7597)) / 2;
    }
  }
  
  // Solve for theta2 using AP formula
  float theta2 = asind(desiredAP/(-LINK_LENGTH_2)) + ANGLE_OFFSET_2;
  
  // Use theta2 to find theta1 using ML formula
  float theta1 = asind(desiredML/(LINK_LENGTH_1+LINK_LENGTH_2*cosd(theta2-ANGLE_OFFSET_2))) + ANGLE_OFFSET_1;
  
  // Use theta1 and theta2 to find H
  float desiredH = cosd(theta1-ANGLE_OFFSET_1)*(LINK_LENGTH_1+LINK_LENGTH_2*cosd(theta2-ANGLE_OFFSET_2));
  
  // Set the DESIRED_POSITION
  DESIRED_POSITION[0] = desiredH;
  DESIRED_POSITION[1] = desiredML;
  DESIRED_POSITION[2] = desiredAP;
}

// These equations found using Pseudo-Inverse Jacobian
void calculateTHETA_DOT() {
  // Shorthand Trigonometry
  float s1 = sind(CURRENT_ANGLE[0]-ANGLE_OFFSET_1);
  float c1 = cosd(CURRENT_ANGLE[0]-ANGLE_OFFSET_1);
  float s2 = sind(CURRENT_ANGLE[1]-ANGLE_OFFSET_2);
  float c2 = cosd(CURRENT_ANGLE[1]-ANGLE_OFFSET_2);
  float L1 = LINK_LENGTH_1;
  float L2 = LINK_LENGTH_2;
  
  // Find values for V_DOT
  float Hv = DESIRED_POSITION[0] - CURRENT_POSITION[0];
  float MLv = DESIRED_POSITION[1] - CURRENT_POSITION[1];
  float APv = DESIRED_POSITION[2] - CURRENT_POSITION[2];
  
  // Find cell values for inverse(J)
  float ij11 = (-s1*(L1 + L2*c2)) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  float ij21 = (-c1*s2) / L2;
  float ij31 = -s1 / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  float ij12 = (c1*(L1 + L2*c2)) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  float ij22 = (-s1*s2) / L2;
  float ij32 = (c1) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  float ij13 = 0;
  float ij23 = -c2 / L2;
  float ij33 = 0;
  
  // THETA_DOT = inverse(J) * V_DOT
  THETA_DOT[0] = (ij11*Hv + ij12*MLv + ij13*APv)*(180/M_PI); // Ankle/Servo 4/Theta 1
  THETA_DOT[1] = (ij21*Hv + ij22*MLv + ij23*APv)*(180/M_PI); // Knee/Servo 3/Theta 2
  THETA_DOT[2] = (ij31*Hv + ij32*MLv + ij33*APv)*(180/M_PI); // Hip/Servo 2/Theta 3
}

void calculateLQR(float current_angle, float desired_angle, uint8_t id) {
  // Setup for integration X_dot -> x
  float timeElapsed = (millis() - PREVIOUS_TIME[id]) / 1000;
  float error = (desired_angle - LQR_GAIN[id]);
  
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
  LQR_GAIN[id] = (4.6136936680715328051860524283256 * X_1[id]) + (106.15898292237854150243947515264 * X_2[id])
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
  
  // Set Servo Torque Limit
  ax12SetRegister2(1, AX_TORQUE_LIMIT_L, SERVO_TORQUE);
  ax12SetRegister2(2, AX_TORQUE_LIMIT_L, SERVO_TORQUE);
  ax12SetRegister2(3, AX_TORQUE_LIMIT_L, SERVO_TORQUE);
  ax12SetRegister2(4, AX_TORQUE_LIMIT_L, SERVO_TORQUE);
  
  // Simple balance on flat surface: [ 90 90 82 90 ]
  SetPosition(1, degree2Servo(90));
  SetPosition(2, degree2Servo(90));
  SetPosition(3, degree2Servo(82));
  SetPosition(4, degree2Servo(90));
    
  // Set linear actuator to 0 degree position
  LINEAR.attach(ACTUATOR_PIN, ACTUATOR_MIN, ACTUATOR_MAX);
  moveActuator(0);
  
  // Setup LEDs
  pinMode(INT_LED_PIN, OUTPUT);
  pinMode(CAL_LED_PIN, OUTPUT);
  pinMode(BAL_LED_PIN, OUTPUT);

  // Finish setup
  START_TIME = millis();
  PREVIOUS_TIME[0] = START_TIME;
  PREVIOUS_TIME[1] = START_TIME;
  PREVIOUS_TIME[2] = START_TIME;
  if (SERIAL_ENABLE) {
    Serial.print("Calibrating MPU-6050...");
    Serial.println("\t\t0                          100");
  }
}

// =============================================== LOOP BEGIN ================================================
void loop() {
  // Load bar for debugging purposes
  if (SERIAL_ENABLE) {
    if (LOAD_BAR_A && millis() >= 5e3 && millis() <= 10e3) {
      Serial.print("\t\t\t\t#####");
      LOAD_BAR_A = false;
    }
    if (LOAD_BAR_B && millis() >= 10e3 && millis() <= 15e3) {
      Serial.print("#####");
      LOAD_BAR_B = false;
    }
    if (!LOAD_BAR_A && millis() >= 15e3 && millis() <= 20e3) {
      Serial.print("#####");
      LOAD_BAR_A = true;
    }
    if (!LOAD_BAR_B && millis() >= 20e3 && millis() <= 25e3) {
      Serial.print("#####");
      LOAD_BAR_B = true;
    }
    if (LOAD_BAR_A && millis() >= 25e3 && millis() <= 26e3) {
      Serial.print("#####");
      LOAD_BAR_A = false;
    }
    if (LOAD_BAR_B && millis() >= 26e3 && millis() <= 27e3) {
      Serial.print("#");
      LOAD_BAR_B = false;
    }
    if (!LOAD_BAR_A && millis() >= 27e3 && millis() <= 28e3) {
      Serial.print("#");
      LOAD_BAR_A = true;
    }
    if (!LOAD_BAR_B && millis() >= 28e3 && millis() <= 29e3) {
      Serial.print("#");
      LOAD_BAR_B = true;
    }
    if (LOAD_BAR_A && millis() >= 29e3 && millis() <= 30e3) {
      Serial.print("#");
      LOAD_BAR_A = false;
    }
    if (LOAD_BAR_B && millis() >= 30e3 && millis() <= 31e3) {
      Serial.println("#");
      LOAD_BAR_B = false;
    }
  }
  
  // Check if calibration time has completed
  CAL_DONE = millis() - START_TIME >= CALIBRATION_DELAY;
  digitalWrite(CAL_LED_PIN, !CAL_DONE);
  
  // --------------- Read Sensor Data ---------------
  if (DMP_READY) {
    bool mpuReady = !(!MPU_INTERRUPT && FIFO_COUNT < PACKET_SIZE);
    if (mpuReady) {
      readGyroscope();      
      
      // ---------------- Debug Output 1 ---------------
      // Only print if SERIAL_ENABLE is true
      if (SERIAL_ENABLE) {
        //Serial.print("[PITCH,ROLL]=[");
        //Serial.print(PITCH); Serial.print(", ");
        //Serial.print(ROLL); Serial.print("]; ");
        if (CAL_DONE) {
          Serial.print("\n[4,3]=[");
          Serial.print(CURRENT_ANGLE[0]); Serial.print(", ");
          Serial.print(CURRENT_ANGLE[1]); Serial.print("];  ");
          //Serial.print("[POS]=[");
          //Serial.print(CURRENT_POSITION[0]); Serial.print("; ");
          //Serial.print(CURRENT_POSITION[1]); Serial.print("; ");
          //Serial.print(CURRENT_POSITION[2]); Serial.print("];  "); 
          //Serial.print("[DES]=[");
          //Serial.print(DESIRED_POSITION[0]); Serial.print("; ");
          //Serial.print(DESIRED_POSITION[1]); Serial.print("; ");
          //Serial.print(DESIRED_POSITION[2]); Serial.print("];  ");  
          Serial.print("TDot=[");
          Serial.print(THETA_DOT[0]); Serial.print(", ");
          Serial.print(THETA_DOT[1]); Serial.print("];  ");          
        }
      }

      // ------------ Control and Positioning -----------  
      if (CAL_DONE) {
        // Do angle/position calculations
        updateCURRENT_ANGLE();
        calculateCURRENT_POSITION();
        calculateDESIRED_POSITION();
        calculateTHETA_DOT();
        
        // Determine adjusted desired angle for this iteration
        float desiredAngle[2] = {
          degree2Servo(CURRENT_ANGLE[0] + THETA_DOT[0] - 0.8*LQR_GAIN[0]),
          degree2Servo(CURRENT_ANGLE[1] + THETA_DOT[1] - 0.8*LQR_GAIN[1])
        };
        
        // ---------------- Debug Output 2 ---------------
        if (SERIAL_ENABLE) {
          Serial.print("LQR_GAIN=[");
          Serial.print(0.8*LQR_GAIN[0]); Serial.print(", ");
          Serial.print(0.8*LQR_GAIN[1]); Serial.print("]; ");
          Serial.print("[4',3']=[");
          Serial.print(servo2Degree(desiredAngle[0])); Serial.print(", ");
          Serial.print(servo2Degree(desiredAngle[1])); Serial.print("];");      
        }
        
        // Set new Ankle and Knee servo position while ignoring extreme cases/glitches
        float balanceCondition[2] = {
          abs(CURRENT_ANGLE[0] - servo2Degree(desiredAngle[0])),
          abs(CURRENT_ANGLE[1] - servo2Degree(desiredAngle[1]))
        };
        if (balanceCondition[0] <= 10 && SERVO_TURN) {
          SetPosition(SERVO_ID[0], (int)desiredAngle[0]);
          SERVO_TURN = false;
        }
        if (balanceCondition[1] <= 10 && !SERVO_TURN) {
          SetPosition(SERVO_ID[1], (int)desiredAngle[1]);
          SERVO_TURN = true;
        }
        
        // Calculate LQR control adjustment for next iteration
        calculateLQR(servo2Degree(GetPosition(SERVO_ID[0])), CURRENT_ANGLE[0] + THETA_DOT[0], 0);
        calculateLQR(servo2Degree(GetPosition(SERVO_ID[1])), CURRENT_ANGLE[1] + THETA_DOT[1], 1);
        
        // Determine if balance has been achieved
        IS_BALANCED = abs(CURRENT_ANGLE[0]-servo2Degree(desiredAngle[0])) <= 0.5 && abs(CURRENT_ANGLE[1]-servo2Degree(desiredAngle[1])) <= 0.5;
        
        // Turn on BALANCE LED if robot is balanced
        digitalWrite(BAL_LED_PIN, IS_BALANCED);
      }    
    }
  }  
}
