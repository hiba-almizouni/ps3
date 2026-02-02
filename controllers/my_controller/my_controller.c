//Header Files
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include<webots/camera.h>
#include <webots/camera_recognition_object.h>

#include <stdio.h>

#define TIME_STEP 32

// 5 Cases for switch statement
enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK };

int main(int argc, char **argv) {
  wb_robot_init();
  int counter = 0, i = 0;
  int state = WAITING;
  const double target_positions[] = {-1.88, -2.14, -2.38, -1.51};
  const double target_positions1[] = {-1.88, 2.14, -2.38, -1.51};
  const double target_positions2[] = {-1.88,-1.14, -2.38, -1.51};
  
  double speed = 1.0;

  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  // Gripper internal motors
  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  
  //ur5e
  WbDeviceTag ur_motors[4];
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");
  
  //Camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 2*TIME_STEP);
  wb_camera_recognition_enable(camera, 2*TIME_STEP);
  
  double a,b,c;     // For storing the recognized RGB values from camera
  
  // Initial the rotational motor speed
  for (i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);
    
  // Defining the distance sensor
  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  // Defining the position sensor
  WbDeviceTag position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);
  
  //Getting the camera recongnition object details 

  while (wb_robot_step(TIME_STEP) != -1) {
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    printf("\nRecognized %d objects.\n", number_of_objects);

    //Get and display all the objects information especially the color 
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    for (i = 0; i < number_of_objects; ++i) {
      for (int j = 0; j < objects[i].number_of_colors; ++j){
        a = objects[i].colors[3*j];
        b = objects[i].colors[3*j+1];
        c = objects[i].colors[3*j+2];
        printf("%lf %lf %lf",a,b,c);
        printf("Color : %lf %lf %lf\n", objects[i].colors[3 * j],
               objects[i].colors[3 * j + 1], objects[i].colors[3 * j + 2]);
    }
    }
    
    // Switch cases for the different states of the arm
    if (counter <= 0) {
      switch (state) {
        case WAITING:
          if (wb_distance_sensor_get_value(distance_sensor) < 500) {
            state = GRASPING;
            counter = 8;
            printf("Grasping object\n");
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.85);
          }
          break;
        case GRASPING:
          if (a == 0.666667 && b == 1.000000 && c == 0.000000)
            for (i = 0; i < 4; ++i)
               wb_motor_set_position(ur_motors[i], target_positions1[i]);
          else if (a == 1.000000 && b == 0.000000 && c == 0.000000)
            for (i = 0; i < 4; ++i)
                wb_motor_set_position(ur_motors[i], target_positions2[i]);
          else
            for (i = 0; i < 4; ++i)
                wb_motor_set_position(ur_motors[i], target_positions[i]);
          printf("Rotating arm\n");
          state = ROTATING;
          break;
        case ROTATING:
          if (wb_position_sensor_get_value(position_sensor) < -2.3) {
            counter = 8;
            printf("Releasing object\n");
            state = RELEASING;
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], wb_motor_get_min_position(hand_motors[i]));
          }
          break;
        case RELEASING:
          for (i = 0; i < 4; ++i)
            wb_motor_set_position(ur_motors[i], 0.0);
          printf("Rotating arm back\n");
          state = ROTATING_BACK;
          break;
        case ROTATING_BACK:
          if (wb_position_sensor_get_value(position_sensor) > -0.1) {
            state = WAITING;
            printf("Waiting object\n");
          }
          break;
      }
    }
    counter--;
  };

  wb_robot_cleanup();
  return 0;
}