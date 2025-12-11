/*
  Project: ESP32 Bluetooth Car – Dual L298N + 4 DC Motors
  File: bt_car_esp32.ino
  Author: Furkan Ege
  Board: ESP32 Dev Module (NodeMCU-32S)
  Version: 1.0
  Last Updated: 12/11/2025

  Description:
    Four-wheel drive Bluetooth car using an ESP32 (Classic Bluetooth SPP)
    and two L298N motor drivers. The car is controlled from a mobile
    "BT Car" style app that sends single-character commands over Bluetooth:
      - 'F' = forward
      - 'B' = backward
      - 'L' = turn left
      - 'R' = turn right
      - 'S' = stop

    Each TT gear motor is driven by its own half-bridge channel,
    allowing independent control of all 4 wheels.

  Features:
    - ESP32 Classic Bluetooth (BluetoothSerial)
    - 4 × 5V DC TT gear motors
    - 2 × L298N drivers (one per side, 2 motors each)
    - Simple rule-based motion commands (no PWM speed control yet)
    - Easily adaptable motor mapping if wiring changes

  Wiring:
    L298N #1 (Motors 1 & 2):
      ENA → GPIO 32
      IN1 → GPIO 25
      IN2 → GPIO 26
      ENB → GPIO 33
      IN3 → GPIO 27
      IN4 → GPIO 14

    L298N #2 (Motors 3 & 4):
      ENA → GPIO 22
      IN1 → GPIO 19
      IN2 → GPIO 18
      ENB → GPIO 23
      IN3 → GPIO 5
      IN4 → GPIO 17

    Power:
      - L298N 12V/+V → 6× AA NiMH battery pack (~7.2–8.4V)
      - ESP32 5V → USB powerbank
      - All GNDs common (battery, L298N, ESP32/powerbank)

  Libraries:
    - BluetoothSerial (built-in ESP32 Arduino core)

  Notes:
    - Speed control can be added later using LEDC PWM on EN pins.
    - Ensure all grounds are tied together for reliable control.

  License: GPL-3.0
*/

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

// ====== PINS ======
const int M1_EN  = 32;
const int M1_IN1 = 25;
const int M1_IN2 = 26;
const int M2_EN  = 33;
const int M2_IN1 = 27;
const int M2_IN2 = 14;
const int M3_EN  = 22;
const int M3_IN1 = 19;
const int M3_IN2 = 18;
const int M4_EN  = 23;
const int M4_IN1 = 5;
const int M4_IN2 = 17;

// dir:  1 = forward, -1 = backward, 0 = stop
void setMotor(int in1, int in2, int en, int dir) {
  if (dir > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(en, HIGH);
  } else if (dir < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(en, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(en, LOW);
  }
}

void stopCar() {
  setMotor(M1_IN1, M1_IN2, M1_EN, 0);
  setMotor(M2_IN1, M2_IN2, M2_EN, 0);
  setMotor(M3_IN1, M3_IN2, M3_EN, 0);
  setMotor(M4_IN1, M4_IN2, M4_EN, 0);
}

void forward() {
  setMotor(M1_IN1, M1_IN2, M1_EN, 1);
  setMotor(M3_IN1, M3_IN2, M3_EN, 1);
  setMotor(M2_IN1, M2_IN2, M2_EN, 1);
  setMotor(M4_IN1, M4_IN2, M4_EN, 1);
}

void backward() {
  setMotor(M1_IN1, M1_IN2, M1_EN, -1);
  setMotor(M3_IN1, M3_IN2, M3_EN, -1);
  setMotor(M2_IN1, M2_IN2, M2_EN, -1);
  setMotor(M4_IN1, M4_IN2, M4_EN, -1);
}

void turnLeft() {
  setMotor(M1_IN1, M1_IN2, M1_EN, -1);
  setMotor(M3_IN1, M3_IN2, M3_EN, -1);
  setMotor(M2_IN1, M2_IN2, M2_EN, 1);
  setMotor(M4_IN1, M4_IN2, M4_EN, 1);
}

void turnRight() {
  setMotor(M1_IN1, M1_IN2, M1_EN, 1);
  setMotor(M3_IN1, M3_IN2, M3_EN, 1);
  setMotor(M2_IN1, M2_IN2, M2_EN, -1);
  setMotor(M4_IN1, M4_IN2, M4_EN, -1);
}

// IMPORTANT: APP BUTTON MAPPING
void handleCommand(char c) {
  switch (c) {
    case 'F':      // app: forward button
      turnLeft();  // mapped to physical "forward" pattern
      break;
    case 'B':      // app: backward button
      turnRight(); // mapped to physical "backward" pattern
      break;
    case 'L':      // app: left button
      forward();   // mapped to physical "left" pattern
      break;
    case 'R':      // app: right button
      backward();  // mapped to physical "right" pattern
      break;
    case 'S':      // app: stop button
      stopCar();
      break;
    default:
      break;
  }
}

void setup() {
  SerialBT.begin("BT_CAR");  // Device name shown on the phone
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(M1_EN, OUTPUT);
  pinMode(M2_EN, OUTPUT);
  pinMode(M3_EN, OUTPUT);
  pinMode(M4_EN, OUTPUT);
  stopCar();
}

void loop() {
  if (SerialBT.available() > 0) {
    char c = SerialBT.read();
    handleCommand(c);
  }
}