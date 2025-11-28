#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/touch_sensor.h>
#include <stdio.h>

#define TIME_STEP 32
#define NUM_OBJECTS 3

int main() {
  wb_robot_init();

  // --- Motors ---
  WbDeviceTag baseMotor = wb_robot_get_device("base");
  WbDeviceTag gripperLeft = wb_robot_get_device("gripper::left");
  WbDeviceTag gripperRight = wb_robot_get_device("gripper::right");

  // --- Position sensors (optional) ---
  WbDeviceTag baseSensor = wb_robot_get_device("base_sensor");
  wb_position_sensor_enable(baseSensor, TIME_STEP);

  // --- Distance sensor on conveyor belt ---
  WbDeviceTag beltSensor = wb_robot_get_device("dsBelt");
  wb_distance_sensor_enable(beltSensor, TIME_STEP);

  // --- Touch sensors on gripper ---
  WbDeviceTag ts1 = wb_robot_get_device("ts1");
  WbDeviceTag ts2 = wb_robot_get_device("ts2");
  wb_touch_sensor_enable(ts1, TIME_STEP);
  wb_touch_sensor_enable(ts2, TIME_STEP);

  // Set motor velocities
  wb_motor_set_velocity(baseMotor, 1.0);
  wb_motor_set_velocity(gripperLeft, 0.5);
  wb_motor_set_velocity(gripperRight, 0.5);

  for (int obj = 0; obj < NUM_OBJECTS; obj++) {
    printf("Waiting for object %d on conveyor...\n", obj + 1);

    // Wait for object detection on conveyor belt
    while (wb_distance_sensor_get_value(beltSensor) < 100.0) {
      wb_robot_step(TIME_STEP);
    }
    printf("Object detected!\n");

    // Move base to pickup position (adjust radians if needed)
    wb_motor_set_position(baseMotor, 0.5);  // example angle
    for (int t = 0; t < 50; t++) wb_robot_step(TIME_STEP);

    // Open gripper
    wb_motor_set_position(gripperLeft, 0.025);
    wb_motor_set_position(gripperRight, 0.025);
    for (int t = 0; t < 20; t++) wb_robot_step(TIME_STEP);

    // Close gripper to grab object
    wb_motor_set_position(gripperLeft, 0.0);
    wb_motor_set_position(gripperRight, 0.0);
    for (int t = 0; t < 20; t++) wb_robot_step(TIME_STEP);

    // Move base to container position
    wb_motor_set_position(baseMotor, -0.5);  // example angle
    for (int t = 0; t < 50; t++) wb_robot_step(TIME_STEP);

    // Open gripper to drop object
    wb_motor_set_position(gripperLeft, 0.025);
    wb_motor_set_position(gripperRight, 0.025);
    for (int t = 0; t < 20; t++) wb_robot_step(TIME_STEP);

    // Return base to initial position
    wb_motor_set_position(baseMotor, 0.0);
    for (int t = 0; t < 50; t++) wb_robot_step(TIME_STEP);

    printf("Object %d placed in container.\n", obj + 1);
  }

  printf("All objects processed.\n");
  wb_robot_cleanup();
  return 0;
}
