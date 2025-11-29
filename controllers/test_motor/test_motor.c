#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  // === MOTOR NAMES FROM YOUR ROBOT ===
  const char *names[] = {
    "base",
    "upperarm",
    "forearm",
    "wrist",
    "rotationalwrist",
    "gripper::left",
    "gripper::right"
  };

  // === GET ALL MOTORS ===
  WbDeviceTag motors[7];
  for (int i = 0; i < 7; i++) {
    motors[i] = wb_robot_get_device(names[i]);
    if (!motors[i]) {
      printf("ERROR: Motor %s not found!\n", names[i]);
      return 1;
    }
    wb_motor_set_velocity(motors[i], 1.0);
  }

  printf("All motors loaded.\n");

  // === SIMPLE FUNCTION TO MOVE JOINTS ===
  double pick_pose[7] = {
    1.2,   // base
    -0.9,  // upperarm
    1.3,   // forearm
    -1.5,  // wrist
    0.0,   // rotational wrist
    0.04,  // left gripper close
    -0.04  // right gripper close
  };

  double place_pose[7] = {
    0.4,   // base
    -1.2,  // upperarm
    1.0,   // forearm
    -1.0,  // wrist
    0.0,   // rotational wrist
    0.04,  // left gripper close
    -0.04  // right gripper close
  };

  double home_pose[7] = {
    0.0,  // base
    -0.5, // upperarm
    0.6,  // forearm
    -1.0, // wrist
    0.0,  // rotational wrist
    0.07, // open
    -0.07
  };

  // ========================
  // MOVE TO HOME POSITION
  // ========================
  printf("Moving to home...\n");
  for (int i = 0; i < 7; i++)
    wb_motor_set_position(motors[i], home_pose[i]);
  for (int t = 0; t < 120; t++) wb_robot_step(TIME_STEP);

  // ========================
  // MOVE TO PICK POSITION
  // ========================
  printf("Moving to pick location...\n");
  for (int i = 0; i < 7; i++)
    wb_motor_set_position(motors[i], pick_pose[i]);
  for (int t = 0; t < 150; t++) wb_robot_step(TIME_STEP);

  // CLOSE GRIPPER
  printf("Closing gripper...\n");
  wb_motor_set_position(motors[5], 0.0);
  wb_motor_set_position(motors[6], 0.0);
  for (int t = 0; t < 80; t++) wb_robot_step(TIME_STEP);

  // ========================
  // MOVE TO PLACE LOCATION
  // ========================
  printf("Moving to place location...\n");
  for (int i = 0; i < 7; i++)
    wb_motor_set_position(motors[i], place_pose[i]);
  for (int t = 0; t < 150; t++) wb_robot_step(TIME_STEP);

  // OPEN GRIPPER
  printf("Releasing object...\n");
  wb_motor_set_position(motors[5], 0.07);
  wb_motor_set_position(motors[6], -0.07);
  for (int t = 0; t < 80; t++) wb_robot_step(TIME_STEP);

  // BACK TO HOME
  printf("Returning home...\n");
  for (int i = 0; i < 7; i++)
    wb_motor_set_position(motors[i], home_pose[i]);
  for (int t = 0; t < 150; t++) wb_robot_step(TIME_STEP);

  printf("Done.\n");

  wb_robot_cleanup();
  return 0;
}
