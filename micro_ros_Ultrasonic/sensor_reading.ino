#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define SENSOR_PIN 34 // Pin for sensor input (ADC-capable)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

// Error handling loop
void error_loop() {
  while (1) {
    delay(100);
  }
}

// Timer callback to read sensor data and publish
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    int rawValue = analogRead(SENSOR_PIN);                // Read analog signal
    float voltage = rawValue * (3.3 / 4095.0);            // Convert ADC to voltage
    float distance = voltage * 100.0;                     // Convert voltage to distance (in cm)

    msg.data = distance;                                  // Set message data
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));     // Publish data

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);

  // Configure micro-ROS transport
  set_microros_transports();
  delay(2000); // Delay to ensure transport setup

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "distance_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/distance"));

  // Create timer (publishing at 100 ms intervals)
  const unsigned int timer_timeout = 100; // 100ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0.0; // Initialize message data
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
