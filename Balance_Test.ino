

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
#define PITCH_AP_CONSTANT 3.0537                        // Slope between two anteroposterior balance points wrt Pitch
#define ROLL_ML_CONSTANT -1.7870                        // Slope between two mediolateral balance points wrt Roll
#define CLOCK_FREQUENCY 4e5                             // 400 kHz I2C Clock
#define CALIBRATION_DELAY 10e3                          // Time to let the system calibrate before balancing begins
#define INITIAL_DESIRED_ANGLE 0                         // Desired starting point
#define LINK_LENGTH 375                                 // Link length in millimeters; L1 = L2 = LINK_LENGTH
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
    SERVO_ID[NUM_SERVOS] = { 1, 2, 3, 4 };              // Designated ID numbers for the AX-12A servos
uint16_t
    PACKET_SIZE,                                        // Expected DMP packet size (default is 42 bytes)
    FIFO_COUNT;                                         // Count of all bytes currently in FIFO
uint32_t
    START_TIME;                                         // Timing and delays
Quaternion
    Q;                                                  // [w, x, y, z] -->  Quaternion container
VectorFloat
    GRAVITY;                                            // [x, y, z]  -->  Gravity vector
float
    PRINCIPAL_AXES[3],                                  // [yaw, pitch, roll] Container and gravity vector
    PITCH = INITIAL_DESIRED_ANGLE,                      // Front-to-Back angle
    ROLL = INITIAL_DESIRED_ANGLE,                       // Side-to-Side angle
    ROLL_OFFSET = 0,                                    // Roll offset for circuit box rotation    
    PREVIOUS_ANGLES[3] = { 0, 0, 0 },                   // Joint angles from previous iteration
    CURRENT_ANGLES[3] = { 0, 0, 0 },                    // Joint angles for current iteration
    PREVIOUS_POSITION[3] = { 0, 0, 0 },                 // End effector position from previous iteration
    CURRENT_POSITION[3] = { 0, 0, 0 },                  // End effector position for current iteration
    DESIRED_POSITION[3] = { 0, 0, 0 },                  // Desired end effector position
    THETA_DOT[3] = { 0, 0, 0 };                         // Change in angles (calculated)

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
        
        // Get PITCH and ROLL in degrees
        PITCH = PRINCIPAL_AXES[1] * 180 / M_PI;
        ROLL = (PRINCIPAL_AXES[2] * 180 / M_PI) - ROLL_OFFSET;
        
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

float degree2Servo(float value) {
  return (double)((value + 60) * 1023) / 300;
}

void initializeServos() {
  for(int index = 0; index < NUM_SERVOS; ++index) {
    int id = SERVO_ID[index];
    int torqueMode = dxlGetTorqueEnable(id);
    int torqueMax = dxlGetStartupMaxTorque(id);
    int torqueLimit = dxlGetTorqueLimit(id);
    // Set servos to angle mode (not wheel mode)
    int dxMode = dxlGetMode(id);
    if (dxMode != JOINT_MODE) {
      axSetJointMode(id);
    }
    // Set Torque Enable
    if (torqueMode != 1) {
      dxlSetTorqueEnable(id, 1);
    }
    // Set Max Torque
    if (torqueMax != 1023) {
      dxlSetStartupMaxTorque(id, 1023);
    }
  }
}

void setServoPosition(uint8_t ID, float value) {
  if (value >= 0 && value <= 1023) {
    dxlSetGoalPosition(ID, value);
    dxlAction(ID);
  }
}

void moveServo(uint8_t ID, float value) {
  switch(ID) {
    case 1:
      setServoPosition(1, degree2Servo(value));
      break;
    case 2:
      if (value >= SERVO2_MIN && value <= SERVO2_MAX) {
        setServoPosition(2, degree2Servo(value)); 
      }
      break;
    case 3:
      if (value >= SERVO3_MIN && value <= SERVO3_MAX) {
        setServoPosition(3, degree2Servo(value)); 
      }
      break;
    case 4:
      if (value >= SERVO4_MIN && value <= SERVO4_MAX) {
        setServoPosition(4, degree2Servo(value)); 
      }
      break;
    default:
      break;
  }
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
  CURRENT_ANGLES[0] = servo2Degree(dxlGetPosition(SERVO_ID[3]));
  CURRENT_ANGLES[1] = servo2Degree(dxlGetPosition(SERVO_ID[2]));
  CURRENT_ANGLES[2] = servo2Degree(dxlGetPosition(SERVO_ID[1]));
}

void setAngle(int ID, int AMOUNT) {
  dxlSetGoalPosition(ID, degree2Servo(AMOUNT));
}

// These equations found using transformation matrices
void updatePositions() {
  // Set previous positions
  PREVIOUS_POSITION[0] = CURRENT_POSITION[0];
  PREVIOUS_POSITION[1] = CURRENT_POSITION[1];
  PREVIOUS_POSITION[2] = CURRENT_POSITION[2];
  // Get current positions
  CURRENT_POSITION[0] = cosd(CURRENT_ANGLES[0]-ANGLE_OFFSET)*LINK_LENGTH*(1+cosd(CURRENT_ANGLES[1]-ANGLE_OFFSET));
  CURRENT_POSITION[1] = sind(CURRENT_ANGLES[0]-ANGLE_OFFSET)*LINK_LENGTH*(1+cosd(CURRENT_ANGLES[1]-ANGLE_OFFSET));
  CURRENT_POSITION[2] = LINK_LENGTH*sind(CURRENT_ANGLES[1]-ANGLE_OFFSET);
}

// These equations found using Pseudo-Inverse Jacobian
void calculateTHETA_DOT() {
  // Shorthand
  float s1 = sind(CURRENT_ANGLES[0]-ANGLE_OFFSET);
  float c1 = cosd(CURRENT_ANGLES[0]-ANGLE_OFFSET);
  float s2 = sind(CURRENT_ANGLES[1]-ANGLE_OFFSET);
  float c2 = cosd(CURRENT_ANGLES[1]-ANGLE_OFFSET);
  float L1 = LINK_LENGTH;
  float L2 = LINK_LENGTH;
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
  THETA_DOT[0] = ij11*Xv + ij12*Yv + ij13*Zv;
  THETA_DOT[1] = ij21*Xv + ij22*Yv + ij23*Zv;
  THETA_DOT[2] = ij31*Xv + ij32*Yv + ij33*Zv;
}

// ========================================= BALANCE TEST FUNCTIONS ==========================================
float asind(float value) {
  return (double)(asin(value) * 180 / M_PI);
}

float acosd(float value) {
  return (double)(acos(value) * 180 / M_PI);
}

// DESIRED_POSITION will be determined by appropriate anteroposterior (AP) and mediolateral (ML) movment to find
// a suitable height (H) position where DESIRED_POSITION = [H; AP; ML]
void calculateDESIRED_POSITION() {
  // Find AP and ML based on current ROLL and PITCH angles
  float AP_DESIRED = PITCH_AP_CONSTANT * PITCH;
  float ML_DESIRED = ROLL_ML_CONSTANT * ROLL;
  // Solve for theta2 using ML formula
  float theta2 = asind(ML_DESIRED/LINK_LENGTH) + ANGLE_OFFSET;
  // Use theta2 to find theta1 using AP formula
  float theta1 = asind(AP_DESIRED/(LINK_LENGTH*(1+cosd(theta2-ANGLE_OFFSET)))) + ANGLE_OFFSET;
  // Use theta1 and theta2 to find H
  float H_DESIRED = cosd(theta1-ANGLE_OFFSET)*LINK_LENGTH*(1+cosd(theta2-ANGLE_OFFSET));
  // Set the DESIRED_POSITION
  DESIRED_POSITION[0] = H_DESIRED;
  DESIRED_POSITION[1] = AP_DESIRED;
  DESIRED_POSITION[2] = ML_DESIRED;
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
  dxlInit(1000000);

  //==========================REMOVE THIS=============================

  // Uncomment for servo fixed position
  dxlSetGoalPosition(1, degree2Servo(90));
  dxlSetGoalPosition(2, degree2Servo(82));
  //dxlSetGoalPosition(3, degree2Servo(86));
  dxlSetGoalPosition(4, degree2Servo(89));

  // Uncomment for servo free movment
  //ax12SetRegister(1, AX_TORQUE_ENABLE, 0);
  //ax12SetRegister(2, AX_TORQUE_ENABLE, 0);
  ax12SetRegister(3, AX_TORQUE_ENABLE, 0);
  //ax12SetRegister(4, AX_TORQUE_ENABLE, 0);

  //=======================END REMOVABLE CODE=========================
  // Set linear actuator to 0 degree position
  LINEAR.attach(ACTUATOR_PIN, ACTUATOR_MIN, ACTUATOR_MAX);
  moveActuator(0);
  // Setup LEDs
  pinMode(INT_LED_PIN, OUTPUT);
  pinMode(CAL_LED_PIN, OUTPUT);
  pinMode(BAL_LED_PIN, OUTPUT);
  START_TIME = millis();
}

// =============================================== LOOP BEGIN ================================================
void loop() {
  
  // Check if calibration time has completed
  CAL_DONE = millis() - START_TIME >= CALIBRATION_DELAY;
  digitalWrite(CAL_LED_PIN, !CAL_DONE);

  // --------------- Read Sensor Data ---------------
  // Read MPU6050 data
  if (DMP_READY) {
    bool mpuReady = !(!MPU_INTERRUPT && FIFO_COUNT < PACKET_SIZE);
    if (mpuReady) {
      readGyroscope();
      // Only compiles if SERIAL_ENABLE is defined above
      if (SERIAL_ENABLE) {
        Serial.print("GA:(");
        Serial.print(PITCH); Serial.print(", ");
        Serial.print(ROLL); Serial.print(")\t");
        Serial.print("SA:[");
        Serial.print(CURRENT_ANGLES[0]); Serial.print(", ");
        Serial.print(CURRENT_ANGLES[1]); Serial.print(", ");
        Serial.print(CURRENT_ANGLES[2]); Serial.print("]\t");
        Serial.print("P:[");
        Serial.print(CURRENT_POSITION[0]); Serial.print(", ");
        Serial.print(CURRENT_POSITION[1]); Serial.print(", ");
        Serial.print(CURRENT_POSITION[2]); Serial.print("]\t");
        Serial.print("TD:[");
        Serial.print(THETA_DOT[0]); Serial.print(", ");
        Serial.print(THETA_DOT[1]); Serial.print(", ");
        Serial.print(THETA_DOT[2]); Serial.print("]\t");
        (CAL_DONE) ? Serial.println("") : Serial.println(" ~calibrating~ ");
      }
    }
  }
  
  // ------------ Control and Positioning -----------
  
  if (CAL_DONE) {
    // Control servos here
    updateAngles();
    updatePositions();
    dxlAction(1); dxlAction(2); dxlAction(3); dxlAction(4);
    calculateTHETA_DOT();
    // Determine if balance has been achieved
    IS_BALANCED = abs(PITCH) <= 5 && abs(ROLL) <= 5;
    digitalWrite(BAL_LED_PIN, IS_BALANCED);
  }
}
