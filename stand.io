#include <ESP32Servo.h>

// Servo objects
Servo coxaFront1;
Servo coxaBack1;
Servo coxaBack2;
Servo coxaFront2;
Servo femurFront1;
Servo femurFront2;
Servo femurBack1;
Servo femurBack2;
Servo tibiaFront1;
Servo tibiaFront2;
Servo tibiaBack1;
Servo tibiaBack2;

// Servo pin assignments
#define COXA_FRONT_1_PIN 5
#define COXA_BACK_1_PIN 15
#define COXA_BACK_2_PIN 12
#define COXA_FRONT_2_PIN 21
#define FEMUR_FRONT_1_PIN 18
#define FEMUR_FRONT_2_PIN 22
#define FEMUR_BACK_1_PIN 2
#define FEMUR_BACK_2_PIN 14
#define TIBIA_FRONT_1_PIN 19
#define TIBIA_FRONT_2_PIN 23
#define TIBIA_BACK_1_PIN 4
#define TIBIA_BACK_2_PIN 27

void setup() {
  // Attach each servo to its corresponding pin
  coxaFront1.attach(COXA_FRONT_1_PIN,500,2500);
  coxaBack1.attach(COXA_BACK_1_PIN);
  coxaBack2.attach(COXA_BACK_2_PIN);
  coxaFront2.attach(COXA_FRONT_2_PIN,200,2500);
  femurFront1.attach(FEMUR_FRONT_1_PIN,500,2500);
  femurFront2.attach(FEMUR_FRONT_2_PIN,500,2500);
  femurBack1.attach(FEMUR_BACK_1_PIN,500,2500);
  femurBack2.attach(FEMUR_BACK_2_PIN,500,2500);
  tibiaFront1.attach(TIBIA_FRONT_1_PIN,500,2500);
  tibiaFront2.attach(TIBIA_FRONT_2_PIN,500,2500);
  tibiaBack1.attach(TIBIA_BACK_1_PIN,500,3000);
  tibiaBack2.attach(TIBIA_BACK_2_PIN);

  // Move all servos to 90 degrees
  setAllServosTo90();
}

void loop() {
  // Nothing to do in the loop
}

// Function to set all servos to 90 degrees
void setAllServosTo90() {
  coxaFront2.write(90);
  delay(15);
  coxaFront1.write(60);
  delay(15);
  femurFront1.write(140);  
  delay(15);
  femurFront2.write(165);
  delay(15);
  tibiaFront1.write(60);  
  delay(15);
  tibiaFront2.write(90);
  delay(15);
  femurBack1.write(125);
  delay(15);
  tibiaBack1.write(90);
  delay(15);
  femurBack2.write(140);
  delay(15);
  tibiaBack2.write(120);


}
