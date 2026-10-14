#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Motor number -> PCA9685 channel
int motorChannel[6] = {
  10,  // Motor 0
  11,  // Motor 1
  12,  // Motor 2
  13,  // Motor 3
  14,  // Motor 4
  15   // Motor 5
};

// Minimum pulse width for each motor
int minPulse[6] = {
  500,   // Motor 0
  500,   // Motor 1
  500,   // Motor 2
  500,   // Motor 3
  500,   // Motor 4
  1100   // Motor 5
};

// Maximum pulse width for each motor
int maxPulse[6] = {
  2900,  // Motor 0
  2700,  // Motor 1
  2900,  // Motor 2
  2900,  // Motor 3
  2900,  // Motor 4
  1600   // Motor 5
};

// Initial positions
int initialPulse[6] = {
  1700,  // Motor 0
  900,   // Motor 1
  1700,  // Motor 2
  2000,  // Motor 3
  2050,  // Motor 4
  1550   // Motor 5
};

void setup() {

  Serial.begin(115200);

  // ESP32 I2C pins
  Wire.begin(21, 22);

  // Start PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(500);

  Serial.println("================================");
  Serial.println("6 Motor Controller Ready");
  Serial.println("================================");

  // Set all motors to their initial positions
  for (int motor = 0; motor < 6; motor++) {

    pwm.writeMicroseconds(
      motorChannel[motor],
      initialPulse[motor]
    );

    Serial.print("Motor ");
    Serial.print(motor);
    Serial.print(" -> ");
    Serial.print(initialPulse[motor]);
    Serial.println(" us");
  }

  Serial.println("================================");
}

void loop() {

  if (Serial.available() > 0) {

    // Expected format:
    // motor pulseWidth
    //
    // Example:
    // 0 1700

    int motor = Serial.parseInt();
    int pulseWidth = Serial.parseInt();

    // Check motor number
    if (motor < 0 || motor > 5) {
      Serial.println("ERROR: Motor must be 0-5");
      return;
    }

    // Keep pulse width within that motor's allowed range
    pulseWidth = constrain(
      pulseWidth,
      minPulse[motor],
      maxPulse[motor]
    );

    // Send PWM command
    pwm.writeMicroseconds(
      motorChannel[motor],
      pulseWidth
    );

    // Confirmation
    Serial.print("Motor ");
    Serial.print(motor);
    Serial.print(" -> PCA ");
    Serial.print(motorChannel[motor]);
    Serial.print(" -> ");
    Serial.print(pulseWidth);
    Serial.println(" us");

    // Clear leftover serial characters
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}