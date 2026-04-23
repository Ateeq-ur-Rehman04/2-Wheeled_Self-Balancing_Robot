/*
 * Project: Self-Balancing Robot with Obstacle Avoidance
 * Hardware: Arduino Mega 2560, L298N, JGA-371 Motors, MPU6050, HC-SR04
 * Orientation: MPU6050 Pins facing BACK, Chip UP (X-Axis Forward)
 * * Integration: Prints data to Serial in format "Angle,PWM,Distance" 
 * for MATLAB/Simulink/LabVIEW/Python parsing.
 */

#include <Wire.h>

// ================= USER TUNING (PID) =================
// Start with these. If it oscillates wildly, reduce Kp. If it falls without recovering, increase Kp.
float Kp = 75.0;   // Power: Main restoring force
float Ki = 1.2;    // Stability: Fixes small drift over time
float Kd = 2.0;    // Damping: Reduces jitter/shaking
float targetAngle = 1.0; // Mechanical Zero (Adjust if robot naturally drifts forward/back)

// ================= HARDWARE PROTECTION =================
// Limits PWM to  to protect 6V motors from 11.1V battery
const float MAX_PWM = 168.132;

// ================= PIN DEFINITIONS (MEGA 2560) =================
// L298N Motor Driver
const int ENA = 6;  // PWM Left
const int IN1 = 24; // Left Dir 1
const int IN2 = 25; // Left Dir 2
const int IN3 = 26; // Right Dir 1
const int IN4 = 27; // Right Dir 2
const int ENB = 7;  // PWM Right

// Ultrasonic Sensor
const int TRIG_PIN = 22;
const int ECHO_PIN = 23;

// MPU6050 I2C Address
const int MPU_ADDR = 0x68;

// ================= GLOBAL VARIABLES =================
volatile bool runControlLoop = false; // Timer flag
float accAngle, gyroRate, currentAngle = 0;
float gyroOffset = 0;
float errorSum = 0, lastAngle = 0;
int distanceCm = 0;

// State Machine for Obstacle Avoidance
enum State { BALANCING, AVOIDING };
State robotState = BALANCING;
float turnOffset = 0; // Added to left, subtracted from right to turn

// ================= SETUP =================
void setup() {
  Serial.begin(115200); // Fast Serial for LabVIEW/MATLAB
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast I2C

  // 1. Initialize Pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // 2. Initialize MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); // Wake up
  Wire.endTransmission(true);

  // 3. Calibrate Gyro (CRITICAL: Robot must be still)
  calibrateGyro();

  // 4. Setup Timer 3 Interrupt for 200Hz Loop (5ms)
  // This ensures the PID runs exactly every 5ms regardless of code complexity
  noInterrupts();
  TCCR3A = 0; TCCR3B = 0; TCNT3 = 0;
  OCR3A = 1249; // (16MHz / 64 / 200Hz) - 1
  TCCR3B |= (1 << WGM32); // CTC Mode
  TCCR3B |= (1 << CS31) | (1 << CS30); // 64 Prescaler
  TIMSK3 |= (1 << OCIE3A); // Enable Interrupt
  interrupts();
}

// Timer Interrupt Service Routine
ISR(TIMER3_COMPA_vect) {
  runControlLoop = true;
}

// ================= MAIN LOOP =================
void loop() {
  if (runControlLoop) {
    runControlLoop = false;

    // --- A. DATA ACQUISITION ---
    readMPU(); // Updates 'currentAngle' and 'gyroRate'

    // --- B. SAFETY CHECK ---
    // If robot falls > 40 degrees, cut power immediately
    if (abs(currentAngle) > 40) {
      stopMotors();
      errorSum = 0; // Reset integral to prevent "windup"
      return;
    }

    // --- C. SLOW TASKS (Sonar) ---
    // We don't want to read sonar every 5ms (too fast for sound waves)
    static int slowLoopCounter = 0;
    if (slowLoopCounter++ > 10) { // Every ~50ms
      slowLoopCounter = 0;
      readSonar();
      checkObstacles();
    }

    // --- D. PID CALCULATION ---
    // Error: Difference between where we want to be (targetAngle) and where we are (currentAngle)
    float error = targetAngle - currentAngle;
    
    // Integral: Sum of errors over time (fixes small steady-state leans)
    errorSum += error * 0.005; 
    errorSum = constrain(errorSum, -300, 300); // Anti-windup limit

    // Derivative: Rate of change (we use Gyro Rate directly for smoother response)
    // PID Output
    float pidOutput = (Kp * error) + (Ki * errorSum) - (Kd * gyroRate);

    // --- E. MOTOR MIXING ---
    // Mix the Balance PID with the Turn command
    float speedLeft = pidOutput + turnOffset;
    float speedRight = pidOutput - turnOffset;

    driveMotors(speedLeft, speedRight);

    // --- F. DATA LOGGING (MATLAB/LabVIEW) ---
    // Format: "Angle, PWM_Left, PWM_Right, Distance"
    Serial.print(currentAngle); Serial.print(",");
    Serial.print(speedLeft); Serial.print(",");
    Serial.print(speedRight); Serial.print(",");
    Serial.println(distanceCm);
  }
}

// ================= HELPER FUNCTIONS =================

void readMPU() {
  // We need Accel X, Z and Gyro Y (Pitch Rate)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start at Accel X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = Wire.read()<<8|Wire.read();
  int16_t ay = Wire.read()<<8|Wire.read(); // Ignore
  int16_t az = Wire.read()<<8|Wire.read();
  Wire.read()<<8|Wire.read(); // Temp
  int16_t gx = Wire.read()<<8|Wire.read(); // Ignore Gyro X
  int16_t gy = Wire.read()<<8|Wire.read(); // GYRO Y IS PITCH
  
  // 1. Calculate Pitch from Accelerometer (ArcTan of X and Z gravity vectors)
  // If X points forward, tilting forward makes X positive? Check standard:
  // Usually, gravity is Down (Z). If we tilt forward, X vector increases.
  float accAng = atan2(ax, az) * 180.0 / PI;

  // 2. Convert Gyro to deg/s
  float gyroY = (gy / 131.0) - gyroOffset;

  // 3. Sensor Fusion (Complementary Filter)
  // 99.5% Gyro (Fast), 0.5% Accel (Stable)
  currentAngle = 0.995 * (currentAngle + gyroY * 0.005) + 0.005 * accAng;
  
  // Store rate for D-term in PID
  gyroRate = gyroY;
}

void checkObstacles() {
  // Logic: If obstacle < 15cm, lean back slightly and spin
  if (distanceCm > 0 && distanceCm < 15) {
    robotState = AVOIDING;
    turnOffset = 60.0;    // Spin value (adjust for speed)
    targetAngle = -3.5;   // Lean back slightly to brake/stop forward motion
  } else {
    robotState = BALANCING;
    turnOffset = 0;
    targetAngle = -1.5;   // Restore normal balance point
  }
}

void readSonar() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 4000); // 4ms timeout (~60cm) to prevent blocking
  if (duration == 0) distanceCm = 100;
  else distanceCm = duration * 0.034 / 2;
}

void driveMotors(float sL, float sR) {
  // 1. Constrain to PWM limits
  int pwmL = constrain(sL, -MAX_PWM, MAX_PWM);
  int pwmR = constrain(sR, -MAX_PWM, MAX_PWM);

  // 2. Deadzone Compensation
  // Motors often stall below PWM 40. This forces them to jump the gap.
  if (pwmL > 0 && pwmL < 40) pwmL = 40;
  if (pwmL < 0 && pwmL > -40) pwmL = -40;
  if (pwmR > 0 && pwmR < 40) pwmR = 40;
  if (pwmR < 0 && pwmR > -40) pwmR = -40;

  // 3. Send to Left Motor
  if (pwmL > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Forward
    analogWrite(ENA, pwmL);
  } else {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Reverse
    analogWrite(ENA, abs(pwmL));
  }

  // 4. Send to Right Motor
  if (pwmR > 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Forward
    analogWrite(ENB, pwmR);
  } else {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Reverse
    analogWrite(ENB, abs(pwmR));
  }
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void calibrateGyro() {
  long sum = 0;
  for(int i=0; i<500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x45); // Gyro Y Register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    int16_t gy = Wire.read()<<8|Wire.read();
    sum += gy;
    delay(3);
  }
  gyroOffset = (sum / 500.0) / 131.0;
}