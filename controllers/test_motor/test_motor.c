// ===================== Header Files =====================
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <stdio.h>

// ===================== Defines =====================
#define TIME_STEP 32

#define CALIBRATION_STEPS 50     // steps to learn background
#define DETECTION_DROP   0.15   // how much the distance must drop to detect object

// ===================== States =====================
enum State {
  WAITING,
  GRASPING,
  ROTATING,
  RELEASING,
  ROTATING_BACK
};

// ===================== Main =====================
int main(int argc, char **argv) {

  wb_robot_init();

  int state = WAITING;
  int counter = 0;
  int i;

  double baseline_distance = 0.0;
  int calibrated = 0;

  const double target_positions[] = {-1.88, -2.14, -2.38, -1.51};

  double speed = 1.0;
  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  // ===================== Gripper Motors =====================
  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");

  // ===================== UR5 Motors =====================
  WbDeviceTag ur_motors[4];
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");

  for (i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);

  // ===================== Camera (VISION ONLY) =====================
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  // ===================== Distance Sensor =====================
  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  // ===================== Position Sensor =====================
  WbDeviceTag position_sensor =
      wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  // ===================== Main Loop =====================
  while (wb_robot_step(TIME_STEP) != -1) {

    double distance = wb_distance_sensor_get_value(distance_sensor);
    double position = wb_position_sensor_get_value(position_sensor);

    // ===================== CALIBRATION PHASE =====================
    if (!calibrated) {
      baseline_distance += distance;
      counter++;

      if (counter >= CALIBRATION_STEPS) {
        baseline_distance /= CALIBRATION_STEPS;
        calibrated = 1;
        counter = 0;
        printf("Calibration done | Baseline distance = %.3f\n",
               baseline_distance);
      }
      continue;  // do NOTHING else during calibration
    }

    // ===================== DEBUG =====================
    printf("State:%d | Dist:%.3f | Base:%.3f | Pos:%.2f\n",
           state, distance, baseline_distance, position);

    // ===================== FSM =====================
    if (counter <= 0) {

      switch (state) {

        // ===================== WAITING =====================
        case WAITING:
          if (distance < baseline_distance - DETECTION_DROP) {
            printf("Object detected -> GRASPING\n");
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.85);
            counter = 10;
            state = GRASPING;
          }
          break;

        // ===================== GRASPING =====================
        case GRASPING:
          printf("Grasp complete -> ROTATING\n");
          for (i = 0; i < 4; ++i)
            wb_motor_set_position(ur_motors[i], target_positions[i]);
          state = ROTATING;
          break;

        // ===================== ROTATING =====================
        case ROTATING:
          if (position < -2.3) {
            printf("Target reached -> RELEASING\n");
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(
                hand_motors[i],
                wb_motor_get_min_position(hand_motors[i]));
            counter = 10;
            state = RELEASING;
          }
          break;

        // ===================== RELEASING =====================
        case RELEASING:
          printf("Release done -> ROTATING_BACK\n");
          for (i = 0; i < 4; ++i)
            wb_motor_set_position(ur_motors[i], 0.0);
          state = ROTATING_BACK;
          break;

        // ===================== ROTATING_BACK =====================
        case ROTATING_BACK:
          if (position > -0.1) {
            printf("Home position -> WAITING\n");
            state = WAITING;
          }
          break;
      }
    }

    if (counter > 0)
      counter--;
  }

  wb_robot_cleanup();
  return 0;
}
