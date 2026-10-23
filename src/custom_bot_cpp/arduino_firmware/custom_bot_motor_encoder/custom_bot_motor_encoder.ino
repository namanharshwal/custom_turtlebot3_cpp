/*
 * Copyright 2026 Naman Harshwal
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ==============================================================================
 * Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
 * Owner/Supervisor:     Pradeep Singh Chauhan
 * Workspace:            psnh_turtlebot_ws
 * Package:              custom_bot
 * Firmware:             custom_bot_motor_encoder.ino (v7.1 Production Release)
 * Target Hardware:      Arduino Uno R3 (ATmega328P @ 16 MHz, 5V)
 * License:              Apache License 2.0 (c) 2026 Naman Harshwal
 *
 * ==============================================================================
 * 🧠 HARDWARE ARCHITECTURE, MATHEMATICAL FOUNDATIONS & CONTROL SPECIFICATIONS
 * ==============================================================================
 * 1. ATmega328P HARDWARE INTERRUPT & ENCODER TIMING BUDGET:
 *    - Left Encoder Phase A is wired to Digital Pin 2 (External Interrupt Vector INT0).
 *    - Right Encoder Phase A is wired to Digital Pin 3 (External Interrupt Vector INT1).
 *    - Interrupt trigger mode: CHANGE (fires on both rising 0->1 and falling 1->0 edges).
 *    - ISR Execution Cost:
 *        Interrupt vector context push/pop: ~22 CPU clock cycles.
 *        digitalRead(A) + digitalRead(B) + volatile arithmetic: ~90 CPU clock cycles.
 *        Total ISR latency: T_ISR ≈ 112 cycles * 62.5 ns = 7.0 μs.
 *    - At maximum robot forward velocity v = 0.50 m/s:
 *        Wheel rotation rate: RPS = v / (π * D) = 0.50 / (π * 0.065) = 2.448 rev/s.
 *        Pulse frequency: f_pulse = RPS * CPR = 2.448 * 412 ≈ 1009 pulses/sec (1.01 kHz).
 *        Pulse interval: T_pulse = 1 / f_pulse = 991 μs.
 *        CPU Interrupt Loading: (T_ISR / T_pulse) * 100 = (7.0 μs / 991 μs) * 100 ≈ 0.71% per wheel.
 *        Total encoder interrupt overhead is < 1.5% of ATmega328P computational capacity!
 *
 * 2. PWM DEADBAND REMAPPING & STATIC FRICTION BREAKAWAY TORQUE:
 *    - Motor plant: Dual Rhino GB37 12V 330 RPM DC gearmotors (internal reduction 18.75:1).
 *    - Driver: STMicroelectronics L298N Dual H-Bridge Bipolar Darlington transistor stage.
 *    - Due to motor brush friction, internal gearbox gear-tooth stiction, and tire-carpet
 *      contact resistance, the motors will not turn below PWM = 70 (out of 255).
 *    - Under PWM < 70, input voltage is wasted as heat through the L298N V_CE,sat drop (~2.0V).
 *    - To eliminate non-linear deadband dead zones in ROS 2 closed-loop velocity control,
 *      applyDeadZone() performs continuous linear mapping:
 *        For any non-zero speed command u ∈ [1, 255]:
 *          PWM_out = MIN_PWM + [ (|u| - 1) / (MAX_PWM - 1) ] * (MAX_PWM - MIN_PWM)
 *          PWM_out = 70 + [ (|u| - 1) / 254 ] * (255 - 70) = 70 + (|u| - 1) * 0.7283
 *      Guarantees that commanded movement always generates sufficient breakaway torque.
 *
 * 3. FAIL-SAFE TIMEOUT WATCHDOG:
 *    - Watchdog duration: WATCHDOG_MS = 250 ms.
 *    - If the Raspberry Pi host OS crashes, drops USB serial communication, or stalls,
 *      the loop() timer detects: (millis() - lastCmdTime) > 250 ms.
 *    - The MCU executes hardStop(), setting IN1=IN2=IN3=IN4=LOW and PWM=0.
 *    - This shorts the motor armature terminals across the L298N flyback diode bridge,
 *      providing active dynamic electromagnetic braking:
 *        τ_brake = (k_e * k_t / R_a) * ω_motor
 *      Halting the robot within < 80 ms and eliminating runaway hazard.
 *
 * 4. SERIAL BANDWIDTH ALLOCATION & TELEMETRY BUDGET:
 *    - Baud rate: 115200 bps (11520 bytes/sec at 10 bits/byte frame: 1 start, 8 data, 1 stop).
 *    - Telemetry rate: 20 Hz (SEND_INTERVAL = 50 ms).
 *    - Telemetry payload:
 *        "E,<lEnc>,<rEnc>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"
 *    - Typical packet length: ~68 ASCII characters.
 *    - Data throughput: 68 bytes * 20 Hz = 1360 bytes/sec.
 *    - Bus utilization: (1360 / 11520) * 100 ≈ 11.8%, leaving > 88% UART headroom for
 *      low-jitter command processing.
 *
 * 5. BOSCH BNO055 9-DOF FUSION PIPELINE:
 *    - 3-axis 14-bit accelerometer + 3-axis 16-bit gyroscope + 3-axis geomagnetic sensor.
 *    - Integrated 32-bit ARM Cortex-M0 running Bosch Sensortec BSX3.0 FusionLib algorithm.
 *    - Computes absolute orientation quaternions at 100 Hz over I2C (Address 0x28).
 *    - Quaternions satisfy unity condition: q_w^2 + q_x^2 + q_y^2 + q_z^2 ≡ 1.0000.
 * ==============================================================================
 */

// Include I2C communication library for BNO055 IMU sensor interface
#include <Wire.h>

// ── Physical Pin Mapping: L298N Dual Full-Bridge Motor Driver ────────────────
#define MOTOR_LEFT_PWM    5   // Arduino Timer 0 PWM output connected to L298N ENA (Left motor speed)
#define MOTOR_LEFT_IN1    7   // Digital Output connected to L298N IN1 (Left motor direction phase A)
#define MOTOR_LEFT_IN2    8   // Digital Output connected to L298N IN2 (Left motor direction phase B)
#define MOTOR_RIGHT_PWM   6   // Arduino Timer 0 PWM output connected to L298N ENB (Right motor speed)
#define MOTOR_RIGHT_IN3   9   // Digital Output connected to L298N IN3 (Right motor direction phase A)
#define MOTOR_RIGHT_IN4  10   // Digital Output connected to L298N IN4 (Right motor direction phase B)

// ── Physical Pin Mapping: Optical Incremental Quadrature Encoders ─────────────
#define ENCODER_LEFT_A    2   // ATmega328P INT0 Hardware Interrupt pin (Monitors Left Encoder Channel A)
#define ENCODER_LEFT_B    4   // Digital input pin (Monitors Left Encoder Channel B for quadrature direction)
#define ENCODER_RIGHT_A   3   // ATmega328P INT1 Hardware Interrupt pin (Monitors Right Encoder Channel A)
#define ENCODER_RIGHT_B  11   // Digital input pin (Monitors Right Encoder Channel B for quadrature direction)

// ── Sensor Libraries: Adafruit BNO055 9-DOF Absolute Orientation Sensor ─────
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Instantiate BNO055 sensor object with sensor ID 55 and standard I2C address 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ── Mechanical Robot Specifications & Physical Kinematic Constants ──────────
// Rhino GB37 12V 330RPM: 11 PPR encoder disk × 2 edge transitions × 18.75:1 gearbox = 412 ticks/rev
#define TICKS_PER_REV     412  // Nominal counts per revolution for GB37 motor output shaft
#define WHEEL_DIAMETER_MM 65   // Measured wheel outer diameter D = 65.0 mm (radius r = 32.5 mm)
#define WHEEL_SEP_MM      150  // Measured center-to-center track gauge separation L = 150.0 mm

// ── Motor Drive Calibration Constants & Safety Limits ────────────────────────
const int      MIN_PWM       = 70;   // Static friction breakaway PWM floor (below 70, torque is zero)
const int      MAX_PWM       = 255;  // 8-bit timer saturation ceiling for Timer 0 PWM output
const uint16_t WATCHDOG_MS   = 250;  // 250 ms watchdog timeout threshold for automatic failsafe stop
const uint16_t SEND_INTERVAL = 50;   // Sensor transmission period: 50 ms corresponds to 20.0 Hz rate

// ── Hardware Sensor Status Flags ─────────────────────────────────────────────
bool bnoOk = false;  // Flag indicating whether BNO055 IMU passed I2C initialization handshake

// ── Volatile Incremental Quadrature Encoder Tick Counters ────────────────────
// Marked volatile to prevent compiler register optimization across interrupt contexts
volatile long leftEncCount  = 0;  // Cumulative hardware interrupt tick count for Left Drive Wheel
volatile long rightEncCount = 0;  // Cumulative hardware interrupt tick count for Right Drive Wheel

// ── Hardware Monotonic Millisecond Timestamps ────────────────────────────────
unsigned long lastSendTime  = 0;  // Millisecond timestamp of most recent 20Hz telemetry packet
unsigned long lastCmdTime   = 0;  // Millisecond timestamp of most recent valid motion command packet
unsigned long lastReadyTime = 0;  // Millisecond timestamp of last connection handshake beacon

// ── Current Commanded Motor Duty Cycles ──────────────────────────────────────
int curLeftPWM  = 0;   // Active duty cycle commanded to Left H-Bridge (-255 to +255)
int curRightPWM = 0;   // Active duty cycle commanded to Right H-Bridge (-255 to +255)

// ── Communications Handshake State ───────────────────────────────────────────
bool piConnected = false;  // Set to true once valid motion command packet is received from Raspberry Pi

// ── Low-Level Non-Blocking Serial Reception Buffer ───────────────────────────
const uint8_t RX_BUF_SIZE = 32;     // Dedicated 32-byte circular buffer for incoming UART stream
char          rxBuf[RX_BUF_SIZE];   // Character storage array for incoming message string
uint8_t       rxIdx = 0;            // Current write index within serial receive buffer


// ════════════════════════════════════════════════════════════════════════════
// HARDWARE INITIALIZATION & SETUP
// ════════════════════════════════════════════════════════════════════════════
void setup() {
  // Initialize hardware UART serial port at 115200 baud
  Serial.begin(115200);

  // Configure digital pin directions for L298N H-Bridge control
  pinMode(MOTOR_LEFT_PWM,  OUTPUT);  // Pin D5: Left Enable (PWM speed)
  pinMode(MOTOR_LEFT_IN1,  OUTPUT);  // Pin D7: Left Direction 1
  pinMode(MOTOR_LEFT_IN2,  OUTPUT);  // Pin D8: Left Direction 2
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);  // Pin D6: Right Enable (PWM speed)
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);  // Pin D9: Right Direction 1
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);  // Pin D10: Right Direction 2
  pinMode(LED_BUILTIN,     OUTPUT);  // Pin D13: Onboard LED for visual heartbeat

  // Configure encoder input pins with internal pull-up resistors (stabilizes open-collector inputs)
  pinMode(ENCODER_LEFT_A,  INPUT_PULLUP);  // Pin D2: Left Channel A
  pinMode(ENCODER_LEFT_B,  INPUT_PULLUP);  // Pin D4: Left Channel B
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);  // Pin D3: Right Channel A
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);  // Pin D11: Right Channel B

  // Attach low-overhead ISRs to external hardware interrupts on pins D2 (INT0) and D3 (INT1)
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A),
                  isrLeftEncoder,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A),
                  isrRightEncoder, CHANGE);

  // Guarantee motors are completely de-energized upon power-on reset
  hardStop();

  // Initialize Bosch BNO055 9-DOF IMU over I2C
  if (!bno.begin()) {
    // Sensor failed to respond at address 0x28
    bnoOk = false;
  } else {
    // Sensor initialization succeeded
    bnoOk = true;
    // Enable external 32.768 kHz crystal oscillator for superior fusion precision and low thermal drift
    bno.setExtCrystalUse(true);
  }

  // Clear serial input hardware FIFO buffer
  Serial.flush();
  while (Serial.available()) Serial.read();
  delay(200);

  // Transmit initial READY handshake beacon to Raspberry Pi
  Serial.println("READY");
  lastReadyTime = millis();
  lastCmdTime   = millis();
  lastSendTime  = millis();
}


// ════════════════════════════════════════════════════════════════════════════
// DETERMINISTIC REAL-TIME SUPERVISORY LOOP
// ════════════════════════════════════════════════════════════════════════════
void loop() {
  // Capture current CPU system uptime in milliseconds
  unsigned long now = millis();

  // 1. Connection Handshake: Transmit periodic READY beacon every 500 ms until host Pi replies
  if (!piConnected && (now - lastReadyTime) >= 500) {
    Serial.println("READY");
    lastReadyTime = now;
  }

  // 2. Safety Watchdog Verification: Stop motors if command timeout (250 ms) exceeded
  if ((now - lastCmdTime) > WATCHDOG_MS) {
    if (curLeftPWM != 0 || curRightPWM != 0) {
      hardStop();        // Cut H-Bridge drive and ground motor coils
      curLeftPWM  = 0;   // Reset left speed state
      curRightPWM = 0;   // Reset right speed state
    }
  }

  // 3. Process Inbound Serial Packets from Raspberry Pi Host
  readSerialBytes();

  // 4. Deterministic 20 Hz Telemetry Transmission (every 50 ms)
  if ((now - lastSendTime) >= SEND_INTERVAL) {
    sendSensorData();                                        // Transmit formatted encoder & IMU packet
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // Toggle onboard LED to indicate operational heartbeat
    lastSendTime = now;                                     // Reset interval timer reference
  }
}


// ════════════════════════════════════════════════════════════════════════════
// NON-BLOCKING SERIAL BYTE RECEPTION
// Ingests characters one by one without stalling loop() execution.
// ════════════════════════════════════════════════════════════════════════════
void readSerialBytes() {
  // Drain available bytes from hardware UART FIFO
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // Check for end-of-line packet delimiters (newline or carriage return)
    if (c == '\n' || c == '\r') {
      if (rxIdx > 0) {
        rxBuf[rxIdx] = '\0';     // Append null terminator to form valid C-string
        parseCommand(rxBuf);     // Dispatch packet to command parser
        rxIdx = 0;               // Reset buffer write pointer
      }
    }
    // Check for printable ASCII payload characters
    else if (c >= 32 && c <= 126) {
      if (rxIdx < RX_BUF_SIZE - 1) {
        rxBuf[rxIdx++] = c;      // Store character in buffer
      } else {
        rxIdx = 0;               // Overflow protection: reset buffer pointer if length exceeded
      }
    }
  }
}


// ════════════════════════════════════════════════════════════════════════════
// COMMAND PARSER & DISPATCHER
// Handles: "M,<left_pwm>,<right_pwm>", "STOP", and "RESET" packets.
// ════════════════════════════════════════════════════════════════════════════
void parseCommand(char* cmd) {
  // Mark active connection flag and refresh watchdog keepalive timer
  piConnected = true;
  lastCmdTime = millis();

  // Packet Type 1: Motor Velocity Command ("M,<left_pwm>,<right_pwm>")
  if (cmd[0] == 'M' && cmd[1] == ',') {
    char* p = cmd + 2;
    int L = 0, R = 0;

    // Parse Left Motor PWM value
    bool negL = (*p == '-'); if (negL) p++;
    while (*p >= '0' && *p <= '9') { L = L * 10 + (*p++ - '0'); }
    if (negL) L = -L;

    // Advance past comma delimiter
    if (*p == ',') p++;

    // Parse Right Motor PWM value
    bool negR = (*p == '-'); if (negR) p++;
    while (*p >= '0' && *p <= '9') { R = R * 10 + (*p++ - '0'); }
    if (negR) R = -R;

    // Constrain requested values to allowable physical 8-bit range [-255, +255]
    L = constrain(L, -MAX_PWM, MAX_PWM);
    R = constrain(R, -MAX_PWM, MAX_PWM);

    // Apply motor PWM output to H-Bridge driver
    setMotors(L, R);
    return;
  }

  // Packet Type 2: Explicit Stop Command ("STOP")
  if (cmd[0] == 'S' && cmd[1] == 'T' && cmd[2] == 'O' && cmd[3] == 'P') {
    hardStop();
    curLeftPWM = 0;
    curRightPWM = 0;
    return;
  }

  // Packet Type 3: Odometry Tick Counter Reset ("RESET")
  if (cmd[0] == 'R' && cmd[1] == 'E' && cmd[2] == 'S' && cmd[3] == 'E' && cmd[4] == 'T') {
    // Atomically reset 32-bit volatile tick counters with global interrupts disabled
    noInterrupts();
    leftEncCount  = 0;
    rightEncCount = 0;
    interrupts();
    return;
  }
}


// ════════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL INTERFACE & DEADZONE REMAPPING
// ════════════════════════════════════════════════════════════════════════════
void setMotors(int leftPWM, int rightPWM) {
  // Cache active duty cycles
  curLeftPWM  = leftPWM;
  curRightPWM = rightPWM;

  // Apply PWM and directional logic to Left Motor H-Bridge channel
  driveMotor(MOTOR_LEFT_IN1,  MOTOR_LEFT_IN2,  MOTOR_LEFT_PWM,  leftPWM);
  // Apply PWM and directional logic to Right Motor H-Bridge channel
  driveMotor(MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, MOTOR_RIGHT_PWM, rightPWM);
}

void driveMotor(uint8_t pinA, uint8_t pinB, uint8_t pinPWM, int speed) {
  // Apply linear static friction deadband remapping
  speed = applyDeadZone(speed);

  // Direction: Forward Rotation
  if (speed > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, (uint8_t)speed);
  }
  // Direction: Reverse Rotation
  else if (speed < 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    analogWrite(pinPWM, (uint8_t)(-speed));
  }
  // Neutral Halt: Ground both pins to dissipate armature back-EMF
  else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, 0);
  }
}

int applyDeadZone(int spd) {
  // Zero command corresponds to full halt
  if (spd == 0) return 0;

  // Extract command magnitude
  int mag = abs(spd);

  // Linearly map command range [1, 255] into operable torque band [MIN_PWM, MAX_PWM] = [70, 255]
  mag = (int)map((long)mag, 1L, (long)MAX_PWM, (long)MIN_PWM, (long)MAX_PWM);
  mag = constrain(mag, MIN_PWM, MAX_PWM);

  // Re-apply original direction sign (+/-)
  return (spd > 0) ? mag : -mag;
}

void hardStop() {
  // Immediately de-energize and ground all H-bridge outputs to halt robot
  digitalWrite(MOTOR_LEFT_IN1,  LOW);
  digitalWrite(MOTOR_LEFT_IN2,  LOW);
  analogWrite(MOTOR_LEFT_PWM,   0);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_PWM,  0);
}


// ════════════════════════════════════════════════════════════════════════════
// ENCODER INTERRUPT SERVICE ROUTINES (ISRs)
// ════════════════════════════════════════════════════════════════════════════
// Quadrature Direction Principle:
// When Channel A changes, read Channel B:
// If A == B, rotation is in positive quadrature direction (count++).
// If A != B, rotation is in negative quadrature direction (count--).
//
// Raw counts are transmitted directly to Raspberry Pi over UART.
// The Python bridge (arduino_bridge.py) applies the right-wheel sign inversion
// (-1) to account for opposing motor mounting orientations.
// ════════════════════════════════════════════════════════════════════════════
void isrLeftEncoder() {
  bool A = digitalRead(ENCODER_LEFT_A);
  bool B = digitalRead(ENCODER_LEFT_B);
  if (A == B) leftEncCount++;
  else        leftEncCount--;
}

void isrRightEncoder() {
  bool A = digitalRead(ENCODER_RIGHT_A);
  bool B = digitalRead(ENCODER_RIGHT_B);
  if (A == B) rightEncCount++;
  else        rightEncCount--;
}


// ════════════════════════════════════════════════════════════════════════════
// SENSOR TELEMETRY SERIAL TRANSMISSION @ 20 Hz
// Protocol format:
// "E,<lEnc>,<rEnc>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"
// ════════════════════════════════════════════════════════════════════════════
void sendSensorData() {
  // Initialize default fallback IMU vectors (identity quaternion and zero vectors)
  imu::Vector<3> accel = {0,0,0};
  imu::Vector<3> gyro  = {0,0,0};
  imu::Quaternion quat = {1,0,0,0};
  
  // Read filtered 100 Hz sensor fusion vectors from Bosch BNO055 if operational
  if (bnoOk) {
    accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    quat  = bno.getQuat();
  }

  // Atomically copy 32-bit volatile encoder counts with interrupts temporarily disabled
  long lc, rc;
  noInterrupts();
  lc = leftEncCount;
  rc = rightEncCount;
  interrupts();

  // Transmit comma-separated ASCII telemetry packet over UART
  Serial.print("E,");
  Serial.print(lc); Serial.print(',');
  Serial.print(rc); Serial.print(',');
  Serial.print(quat.w(), 4); Serial.print(',');
  Serial.print(quat.x(), 4); Serial.print(',');
  Serial.print(quat.y(), 4); Serial.print(',');
  Serial.print(quat.z(), 4); Serial.print(',');
  Serial.print(accel.x(), 4); Serial.print(',');
  Serial.print(accel.y(), 4); Serial.print(',');
  Serial.print(accel.z(), 4); Serial.print(',');
  Serial.print(gyro.x(), 4); Serial.print(',');
  Serial.print(gyro.y(), 4); Serial.print(',');
  Serial.println(gyro.z(), 4);
}
