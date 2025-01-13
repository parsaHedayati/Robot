#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

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

// Micro-ROS variables
rcl_subscription_t subscriber;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Error handling loop
void error_loop() {
  while (1) {
    delay(100);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {}}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize servos
  attachServos();
  initializeServoPositions();

  // Initialize micro-ROS
  set_microros_transports();
  delay(2000); // Ensure transport setup

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "servo_controller_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "command"
  ));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, command_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
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
  for (int i_femur_front_1 = 180; i_femur_front_1 >= 140; i_femur_front_1 -= 1) {
    femurFront1.write(i_femur_front_1);
    delay(15);
  }
  delay(1000);
  executeMovementSequence(140, 180, 15, femurBack2);
  delay(1000);
  executeMovementSequence(46, 180, 15, coxaBack2);
  delay(1000);
  for (int i_femur_back_2 = 180; i_femur_back_2 >= 140; i_femur_back_2 -= 1) {
    femurBack2.write(i_femur_back_2);
    delay(15);
  }
  delay(1000);
  executeMovementSequence(90, 0, -15, coxaFront1);
  delay(1000);
  for (int i_coxa_back_2 = 180; i_coxa_back_2 >= 46; i_coxa_back_2 -= 1) {
    coxaBack2.write(i_coxa_back_2);
    delay(20);
  }
  delay(2000);
  for (int i_femur_back_1 = 125; i_femur_back_1 >= 0; i_femur_back_1 -= 1) {
    femurBack1.write(i_femur_back_1);
    delay(15);
  }
  delay(1000);
  executeMovementSequence(70, 0, -15, coxaBack1);
  delay(1000);
  for (int i_femur_back_1 = 0; i_femur_back_1 <= 125; i_femur_back_1 += 1) {
    femurBack1.write(i_femur_back_1);
    delay(15);
  }
  delay(1000);
  for (int i_femur_front_2 = 160; i_femur_front_2 >= 0; i_femur_front_2 -= 1) {
    femurFront2.write(i_femur_front_2);
    delay(15);
  }
  delay(1000);
  executeMovementSequence(180, 90, -15, coxaFront2);
  delay(1000);
  for (int i_femur_front_2 = 0; i_femur_front_2 <= 160; i_femur_front_2 += 1) {
    femurFront2.write(i_femur_front_2);
    delay(15);
  }
  delay(1000);
  for (int i_coxa_back_1 = 0; i_coxa_back_1 <= 70; i_coxa_back_1 += 1) {
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

void command_callback(const void * msgin) {
  const std_msgs__msg__String * received_msg = (const std_msgs__msg__String *)msgin;
  
  if (strcmp(received_msg->data.data, "stand") == 0) {
    initializeServoPositions();
  } else if (strcmp(received_msg->data.data, "move_forward") == 0) {
    moveForward();
  }
}
