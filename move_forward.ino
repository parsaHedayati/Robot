#include <ESP32Servo.h>

// Define servos for each leg
Servo coxaFront1, coxaFront2, coxaBack1, coxaBack2;
Servo femurFront1, femurFront2, femurBack1, femurBack2;
Servo tibiaFront1, tibiaFront2, tibiaBack1, tibiaBack2;

// Define pins for each servo
#define COXA_FRONT_1_PIN 5
#define COXA_BACK_1_PIN 27
#define COXA_BACK_2_PIN 13
#define COXA_FRONT_2_PIN 21
#define FEMUR_FRONT_1_PIN 18
#define FEMUR_FRONT_2_PIN 22
#define FEMUR_BACK_1_PIN 2
#define FEMUR_BACK_2_PIN 12 
#define TIBIA_FRONT_1_PIN 19
#define TIBIA_FRONT_2_PIN 23
#define TIBIA_BACK_1_PIN 4
#define TIBIA_BACK_2_PIN 14

void setup() {
  // Attach each servo to its corresponding pin
  attachServos();

  // Move all servos to their initial positions
  initializeServoPositions();

  // Execute the move forward sequence
  moveForward();
}

void loop() {
while (true){
  
  moveForward();
}
}

void attachServos() {
  coxaFront1.attach(COXA_FRONT_1_PIN, 200, 2500);
  coxaFront2.attach(COXA_FRONT_2_PIN, 200, 2500);
  coxaBack1.attach(COXA_BACK_1_PIN, 1000, 3000);
  coxaBack2.attach(COXA_BACK_2_PIN, 1000, 3000);

  femurFront1.attach(FEMUR_FRONT_1_PIN, 200, 2500);
  femurFront2.attach(FEMUR_FRONT_2_PIN, 200, 2500);
  femurBack1.attach(FEMUR_BACK_1_PIN, 200, 2500);
  femurBack2.attach(FEMUR_BACK_2_PIN, 200, 3000);

  tibiaFront1.attach(TIBIA_FRONT_1_PIN, 1000, 3000);
  tibiaFront2.attach(TIBIA_FRONT_2_PIN, 500, 2500);
  tibiaBack1.attach(TIBIA_BACK_1_PIN, 1000, 3000);
  tibiaBack2.attach(TIBIA_BACK_2_PIN, 500, 2500);
}

void initializeServoPositions() {
  tibiaFront1.write(30);                  
  tibiaFront2.write(90);
  tibiaBack1.write(80);
  tibiaBack2.write(125);
}

void moveForward() {
  executeMovementSequence(140, 180, 15, femurFront1);
  delay(1000);
  executeMovementSequence(0, 90, 15, coxaFront1);
  delay(1000);
  for(int i_femur_front_1 = 180; i_femur_front_1 >=140; i_femur_front_1 -=1){
      femurFront1.write(i_femur_front_1);
      delay(15);
    }
  delay(1000);
  executeMovementSequence(140, 180, 15, femurBack2);
  delay(1000);
  executeMovementSequence(46, 180, 15, coxaBack2);
  delay(1000);
  for(int i_femur_back_2 = 180; i_femur_back_2 >=140; i_femur_back_2 -=1){
  femurBack2.write(i_femur_back_2);
  delay(15);
}
  delay(1000);
  executeMovementSequence(90, 0, -15, coxaFront1);
  delay(1000);
  for (int i_coxa_back_2 = 180; i_coxa_back_2 >=46; i_coxa_back_2 -=1){
  coxaBack2.write(i_coxa_back_2);
  delay(20);
}

  delay(2000);

  for(int i_femur_back_1 = 125;i_femur_back_1>=0;i_femur_back_1 -=1){
  femurBack1.write(i_femur_back_1);
  delay(15);
}
  delay(1000);
  executeMovementSequence(70, 0, -15, coxaBack1);
  delay(1000);
  for(int i_femur_back_1 = 0;i_femur_back_1 <= 125; i_femur_back_1 +=1 ){
    femurBack1.write(i_femur_back_1);
    delay(15);
  }
  delay(1000);
  for (int i_femur_front_2 = 160;i_femur_front_2 >=0; i_femur_front_2 -=1){
  femurFront2.write(i_femur_front_2);
  delay(15);
}
  delay(1000);
  executeMovementSequence(180, 90, -15, coxaFront2);
  delay(1000);
  for (int i_femur_front_2 = 0;i_femur_front_2 <=160; i_femur_front_2 +=1){
    femurFront2.write(i_femur_front_2);
    delay(15);
}
  delay(1000);
  for(int i_coxa_back_1 = 0;i_coxa_back_1 <=70; i_coxa_back_1 +=1){
    coxaBack1.write(i_coxa_back_1);
    delay(15);
  }
  delay(1000);
  executeMovementSequence(90, 180, 15, coxaFront2);
}

void executeMovementSequence(int start, int end, int step, Servo &servo) {
  if (step > 0) {
    for (int i = start; i <= end; i += step) {
      servo.write(i);
      delay(15);
    }
  } else {
    for (int i = start; i >= end; i += step) {
      servo.write(i);
      delay(15);
    }
  }
}
