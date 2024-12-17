#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Servo objects
Servo coxaFront1, coxaBack1, coxaBack2, coxaFront2;
Servo femurFront1, femurFront2, femurBack1, femurBack2;
Servo tibiaFront1, tibiaFront2, tibiaBack1, tibiaBack2;

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

// ROS 2-related variables
rcl_subscription_t subscriber;
std_msgs__msg__String recv_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Error loop for critical failures
void error_loop() {
  while (1) {
    delay(100);
  }
}

// Function to set all servos to 90 degrees (standing position)
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

// Callback function for receiving ROS 2 commands
void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  if (strcmp(msg->data.data, "stand") == 0) {
    Serial.println("Command received: stand");
    setAllServosTo90();
  } else {
    Serial.print("Unknown command: ");
    Serial.println(msg->data.data);
  }
}

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // Attach each servo to its pin
  coxaFront1.attach(COXA_FRONT_1_PIN, 500, 2500);
  coxaBack1.attach(COXA_BACK_1_PIN);
  coxaBack2.attach(COXA_BACK_2_PIN);
  coxaFront2.attach(COXA_FRONT_2_PIN, 200, 2500);
  femurFront1.attach(FEMUR_FRONT_1_PIN, 500, 2500);
  femurFront2.attach(FEMUR_FRONT_2_PIN, 500, 2500);
  femurBack1.attach(FEMUR_BACK_1_PIN, 500, 2500);
  femurBack2.attach(FEMUR_BACK_2_PIN, 500, 2500);
  tibiaFront1.attach(TIBIA_FRONT_1_PIN, 500, 2500);
  tibiaFront2.attach(TIBIA_FRONT_2_PIN, 500, 2500);
  tibiaBack1.attach(TIBIA_BACK_1_PIN, 500, 3000);
  tibiaBack2.attach(TIBIA_BACK_2_PIN);

  // micro-ROS setup
  set_microros_transports(); // Set up transport (Serial, UDP, etc.)
  delay(2000);               // Delay to stabilize connection

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create a ROS 2 node
  RCCHECK(rclc_node_init_default(&node, "servo_controller_node", "", &support));

  // Create a subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/command"));

  // Initialize executor for handling callbacks
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

  Serial.println("Servo controller ready. Waiting for commands...");
}

void loop() {
  // Spin executor to handle incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
