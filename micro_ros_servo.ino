#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>

#define NUM_SERVOS 12

Servo servos[NUM_SERVOS];
const int servoPins[NUM_SERVOS] = {12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25}; // Adjust pins as needed

rcl_subscription_t servo_subscribers[NUM_SERVOS];
std_msgs__msg__Int8 servo_msgs[NUM_SERVOS];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

int limitToMaxValue(int value, int maxLimit) {
  if (value > maxLimit) {
    return maxLimit;
  }
  return value;
}

void servo_callback(const void* msgin, int servoIndex) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  int8_t angle = msg->data;
  int servo_position = limitToMaxValue(angle, 40);
  servos[servoIndex].write(servo_position);
}

// Callback functions for each servo
void servo0_cb(const void* msgin) { servo_callback(msgin, 0); }
void servo1_cb(const void* msgin) { servo_callback(msgin, 1); }
void servo2_cb(const void* msgin) { servo_callback(msgin, 2); }
void servo3_cb(const void* msgin) { servo_callback(msgin, 3); }
void servo4_cb(const void* msgin) { servo_callback(msgin, 4); }
void servo5_cb(const void* msgin) { servo_callback(msgin, 5); }
void servo6_cb(const void* msgin) { servo_callback(msgin, 6); }
void servo7_cb(const void* msgin) { servo_callback(msgin, 7); }
void servo8_cb(const void* msgin) { servo_callback(msgin, 8); }
void servo9_cb(const void* msgin) { servo_callback(msgin, 9); }
void servo10_cb(const void* msgin) { servo_callback(msgin, 10); }
void servo11_cb(const void* msgin) { servo_callback(msgin, 11); }

void (*callbacks[])(const void*) = {
    servo0_cb, servo1_cb, servo2_cb, servo3_cb, 
    servo4_cb, servo5_cb, servo6_cb, servo7_cb,
    servo8_cb, servo9_cb, servo10_cb, servo11_cb
};

void setup() {
  set_microros_transports();

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Initialize all servos
  for(int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 1000, 2000);
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "servo_esp32", "", &support));

  // Initialize executor for all servos
  RCCHECK(rclc_executor_init(&executor, &support.context, NUM_SERVOS, &allocator));

  // Create subscribers for each servo
  char topic_name[20];
  for(int i = 0; i < NUM_SERVOS; i++) {
    sprintf(topic_name, "/servo%d", i);
    RCCHECK(rclc_subscription_init_default(
      &servo_subscribers[i],
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      topic_name));
    
    RCCHECK(rclc_executor_add_subscription(
      &executor, 
      &servo_subscribers[i], 
      &servo_msgs[i], 
      callbacks[i],
      ON_NEW_DATA));
  }
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
