#include <ax12.h>

#define SERVOID_ANKLE 4
#define SERVOID_KNEE 3
#define SERVOID_HIP 2

int L1 = 375;
int L2 = 375;

double previousAngle[3] = { 0, 0, 0 };
double currentAngle[3] = { 0, 0, 0 };

double desiredPosition[3] = { 750, 0, 0 };

double previousPosition[3] = { 0, 0, 0 };
double currentPosition[3] = { 0, 0, 0 };
double angleOffset = 90;

double qDots[3] = { 0, 0, 0 };

// Functions

double sind(double x) {
  return sin(x * M_PI / 180);
}

double cosd(double x) {
  return cos(x * M_PI / 180);
}

// Degree-Position conversion:
// The values 0 to 1023 correspond to the degrees -60 to 240
// (or 0 to 300 if you're feeling adventurous!)
double position2Degree(double value) {
  //double temp = (300 * value)/1023;
  return (double)((300 * value) / 1023) - 60;
}

double degree2Position(double value) {
  return (double)((value + 60) * 1023) / 300;
}

// Handle Angles
void getAngle() {
  previousAngle[0] = currentAngle[0];
  previousAngle[1] = currentAngle[1];
  previousAngle[2] = currentAngle[2];
  currentAngle[0] = position2Degree(dxlGetPosition(SERVOID_ANKLE));
  currentAngle[1] = position2Degree(dxlGetPosition(SERVOID_KNEE));
  currentAngle[2] = position2Degree(dxlGetPosition(SERVOID_HIP));
  Serial.print(currentAngle[0]); Serial.print('\t');
  Serial.print(currentAngle[1]); Serial.print('\t');
  Serial.print(currentAngle[2]); Serial.print('\t');
}

void setAngle(int ID, int AMOUNT) {
  dxlSetGoalPosition(ID, degree2Position(AMOUNT));
}

// Handle Positions
void getPositions() {
  previousPosition[0] = currentPosition[0];
  previousPosition[1] = currentPosition[1];
  previousPosition[2] = currentPosition[2];

  currentPosition[0] = cosd(currentAngle[0]-angleOffset)*(L1 + L2*cosd(currentAngle[1]-angleOffset));
  currentPosition[1] = sind(currentAngle[0]-angleOffset)*(L1 + L2*cosd(currentAngle[1]-angleOffset));
  currentPosition[2] = L2 * sind(currentAngle[1]-angleOffset);
  
  Serial.print(currentPosition[0]); Serial.print('\t');
  Serial.print(currentPosition[1]); Serial.print('\t');
  Serial.print(currentPosition[2]); Serial.print('\t');
}

// Inverse Jacobian Stuff

void qDotMaths() {
  double Xv = desiredPosition[0] - currentPosition[0];
  double Yv = desiredPosition[1] - currentPosition[1];
  double Zv = desiredPosition[2] - currentPosition[2];
  double s1 = sind(currentAngle[0]-angleOffset);
  double c1 = cosd(currentAngle[0]-angleOffset);
  double s2 = sind(currentAngle[1]-angleOffset);
  double c2 = cosd(currentAngle[1]-angleOffset);

  double ij11 = (-s1*(L1 + L2*c2)) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  double ij21 = (-c1*s2) / L2;
  double ij31 = s1 / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  double ij12 = (c1*(L1 + L2*c2)) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  double ij22 = (-s1*s2) / L2;
  double ij32 = (-c1) / (L2*L2*c2*c2 + L1*L1 + 2*L1*L2*c2 + 1);
  double ij13 = 0;
  double ij23 = c2 / L2;
  double ij33 = 0;

  qDots[0] = ij11*Xv + ij12*Yv + ij13*Zv;
  qDots[1] = ij21*Xv + ij22*Yv + ij23*Zv;
  qDots[2] = ij31*Xv + ij32*Yv + ij33*Zv;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dxlInit(1000000); //start the DYNAMIXEL chain at 

}

void loop() {
  // put your main code here, to run repeatedly:
  getAngle();
  getPositions();
  dxlAction(1);
  dxlAction(2);
  dxlAction(3);
  dxlAction(4);
  qDotMaths();
  
  Serial.print(qDots[0]); Serial.print('\t');
  Serial.print(qDots[1]); Serial.print('\t');
  Serial.println(qDots[2]);
  delay(1000);
}
