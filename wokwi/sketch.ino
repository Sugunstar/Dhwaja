/*
  DHWAJA Guidance Kit Control Loop Demonstration for Wokwi (STM32F103C8 Blue Pill)
  This sketch demonstrates the sense -> estimate -> correct -> act loop
  using simulated sensors and actuators on an STM32.

  Sensors:
    - MPU6050 IMU (accelerometer and gyroscope)
    - Potentiometer (simulates target angle or disturbance)

  Estimation:
    - Complementary filter to fuse accelerometer and gyroscope data
      to estimate the current angle (pitch).

  Control:
    - PD controller to compute servo angle based on error between
      estimated angle and target angle (from potentiometer).

  Actuation:
    - Servo motor representing canard actuation.

  Fuze Simulation:
    - LED and buzzer trigger when the system remains within a deadband
      for a specified time (simulating impact fuze).

  Note: This is a simplified demonstration and does not model real
        flight dynamics, real sensor accuracies, or real hardware
        constraints of the DHWAJA system.

  Libraries required:
    - MPU6050 (by Electronic Cats)
    - Adafruit SSD1306
    - Adafruit GFX Library
    - Servo (built-in)
*/

#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// MPU6050 I2C pins (STM32F103C8: PB6=SCL, PB7=SDA)
#define I2C_SCL PB6
#define I2C_SDA PB7

// OLED dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

// Pin definitions (STM32 Arduino numbering: PA0-PA15 = 0-15, PB0-PB15 = 16-31)
#define SERVO_PIN      PA1   // Arduino pin 1
#define POT_PIN        PA0   // Arduino pin 0 (ADC)
#define BUTTON_PIN     PA2   // Arduino pin 2
#define LED_PIN        PA3   // Arduino pin 3
#define BUZZER_PIN     PA4   // Arduino pin 4

// Filter constants
#define ALPHA 0.98   // Complementary filter constant
#define DT    0.01   // Time step in seconds (10 ms)

// Control constants
#define Kp 2.0       // Proportional gain
#define Kd 0.1       // Derivative gain
#define SERVO_MIN 0  // Servo minimum angle
#define SERVO_MAX 180 // Servo maximum angle

// Fuze simulation
#define DEADBAND 5.0   // Degrees of error considered "on target"
#define FUSE_TIME 2000 // milliseconds to wait on target before triggering

// Global objects
MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Servo servo;

// Global variables
float estimatedAngle = 0.0;   // in degrees
float targetAngle = 0.0;      // in degrees
float servoAngle = 90.0;      // in degrees
unsigned long lastTime = 0;
unsigned long fuseTimer = 0;
bool fuseTriggered = false;
float lastGyroAngle = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); // SDA, SCL

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    while (1);
  }
  display.clearDisplay();
  display.display();

  // Initialize servo
  servo.attach(SERVO_PIN);
  servo.write((int)servoAngle);

  // Initialize pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initial delay for sensor stabilization
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // Convert to seconds
  if (dt < DT) {
    // Wait until the desired time step has passed
    delay((DT - dt) * 1000);
    dt = DT;
  }
  lastTime = now;

  // ===== SENSE =====
  // Read MPU6050: accelerometer and gyroscope
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to SI units
  float accelX = ax / 16384.0; // Assuming +/- 2g range
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX = gx / 131.0;    // Assuming +/- 250 deg/s range
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Calculate angle from accelerometer (for pitch and roll)
  // We'll use the Y-axis (pitch) for this example
  float accelAngle = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0 / PI;

  // Integrate gyroscope to get angle change
  float gyroAngle = lastGyroAngle + gyroY * dt;
  lastGyroAngle = gyroAngle;

  // Complementary filter to fuse accelerometer and gyroscope
  estimatedAngle = ALPHA * (estimatedAngle + gyroY * dt) + (1.0 - ALPHA) * accelAngle;

  // Read potentiometer (0-4095) and map to target angle (-90 to 90 degrees)
  int potValue = analogRead(POT_PIN);
  targetAngle = map(potValue, 0, 4095, -90, 90);

  // ===== ESTIMATE =====
  // (Already done in complementary filter)

  // ===== CORRECT =====
  // Compute error
  float error = targetAngle - estimatedAngle;

  // PD controller
  static float lastError = 0.0;
  float derivative = (error - lastError) / dt;
  float output = Kp * error + Kd * derivative;
  lastError = error;

  // Map controller output to servo angle (0-180)
  // We assume that zero error corresponds to 90 degrees (center)
  servoAngle = 90.0 + output;
  // Constrain to servo limits
  if (servoAngle < SERVO_MIN) servoAngle = SERVO_MIN;
  if (servoAngle > SERVO_MAX) servoAngle = SERVO_MAX;

  // ===== ACT =====
  servo.write((int)servoAngle);

  // ===== FUZE SIMULATION =====
  // Check if within deadband
  if (abs(error) < DEADBAND) {
    // If we have been in deadband for FUSE_TIME, trigger
    if (fuseTimer == 0) {
      fuseTimer = now; // Start timer
    } else if (now - fuseTimer > FUSE_TIME) {
      fuseTriggered = true;
    }
  } else {
    // Reset timer if outside deadband
    fuseTimer = 0;
    fuseTriggered = false;
  }

  // Update fuze outputs
  digitalWrite(LED_PIN, fuseTriggered ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, fuseTriggered ? HIGH : LOW);

  // Button to reset fuze
  if (digitalRead(BUTTON_PIN) == LOW) {
    fuseTriggered = false;
    fuseTimer = 0;
    delay(50); // Debounce
  }

  // ===== TELEMETRY (OLED) =====
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Est: ");
  display.print(estimatedAngle, 1);
  display.print(" deg");
  display.setCursor(0, 10);
  display.print("Tgt: ");
  display.print(targetAngle, 1);
  display.print(" deg");
  display.setCursor(0, 20);
  display.print("Servo: ");
  display.print((int)servoAngle);
  display.print(" deg");
  display.setCursor(0, 30);
  display.print("Error: ");
  display.print(error, 1);
  display.print(" deg");
  display.setCursor(0, 40);
  if (fuseTriggered) {
    display.print("FUSE: TRIGGERED");
  } else {
    display.print("Fuse timer: ");
    if (fuseTimer > 0) {
      display.print((now - fuseTimer) / 1000.0, 1);
      display.print("s");
    } else {
      display.print("0.0s");
    }
  }
  display.display();

  // Debug output to serial
  Serial.print("Est: ");
  Serial.print(estimatedAngle);
  Serial.print(" | Tgt: ");
  Serial.print(targetAngle);
  Serial.print(" | Err: ");
  Serial.print(error);
  Serial.print(" | Servo: ");
  Serial.println((int)servoAngle);
}