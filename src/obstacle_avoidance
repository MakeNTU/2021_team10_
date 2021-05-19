#include <NewPing.h>        //for the Ultrasonic sensor function library.
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
SoftwareSerial BT (9, 10);


#define MotorR_I1     7 //定義 I1 接腳（右）
#define MotorR_I2     8 //定義 I2 接腳（右）8
#define MotorL_I3     4 //定義 I3 接腳（左）4
#define MotorL_I4     2 //定義 I4 接腳（左） 2
#define MotorL_PWML    6 //定義 ENA (PWM調速) 接腳
#define MotorR_PWMR    5 //定義 ENB (PWM調速) 接腳 5
#define trig_pin A1 //analog input 1
#define echo_pin A2 //analog input 2
#define maximum_distance 200
int distance = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function

void setup() {

  pinMode(MotorR_I1,   OUTPUT);
  pinMode(MotorR_I2,   OUTPUT);
  pinMode(MotorL_I3,   OUTPUT);
  pinMode(MotorL_I4,   OUTPUT);
  pinMode(MotorL_PWML, OUTPUT);
  pinMode(MotorR_PWMR, OUTPUT);
  pinMode(A6, INPUT);

  Serial.begin(9600);
  MotorWriting(0, 0);
  analogWrite(A6, 0);
  distance = readPing();
  delay(18000);
}

void loop() {
  //  Serial.print("A6 = ");
  //  Serial.println(analogRead(A6));
  moveForward();
  readPing();
  if (distance <= 50) {
    moveBackward();
    int i = random(0, 2);
    if (i == 1) {
      turnRight();
      moveStop();
    }
    else {
      turnLeft();
      moveStop();
    }
  }
  else {
    moveForward();
  }
  distance = readPing();

}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  //  Serial.println(cm);
  return cm;
}

void moveStop() {
  MotorWriting(0, 0);
  delay(200);
}

void moveForward() {
  MotorWriting(255, 255);
  delay(500);
}

void moveBackward() {
  MotorWriting(-255, -255);
  delay(400);
}

void turnRight() {
  MotorWriting(255, -255);
  delay(500);
}

void turnLeft() {
  MotorWriting(-255, 255);
  delay(500);
}

void MotorWriting(double vL, double vR) {
  if (vR > 255) vR = 255;
  if (vL > 255) vL = 255;
  if (vR < -255) vR = -255;
  if (vL < -255) vL = -255;

  if (vL < 0) {
    digitalWrite(MotorR_I1, LOW);
    digitalWrite(MotorR_I2, HIGH);
    vL = -vL;
  }
  else if (vL >= 0) {
    digitalWrite(MotorR_I1, HIGH);
    digitalWrite(MotorR_I2, LOW);
  }

  if (vR < 0) {
    digitalWrite(MotorL_I3, HIGH);
    digitalWrite(MotorL_I4, LOW);
    vR = -vR;
  }
  else if (vR >= 0) {
    digitalWrite(MotorL_I3, LOW);
    digitalWrite(MotorL_I4, HIGH);
  }
  if (analogRead(A6) > 300) {
    vL = 0; vR = 0;
  }
//  Serial.print("vL = ");
//  Serial.println(vL);
//  Serial.print("vR = ");
//  Serial.println(vR);

  analogWrite(MotorL_PWML, vL);
  analogWrite(MotorR_PWMR, vR);
}// MotorWriting
