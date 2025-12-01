#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <vector>

using namespace webots;
using namespace std;

#define TIME_STEP 32

int main() {
  Robot robot;

  // === Motors ===
  vector<string> motorNames = {"base", "upperarm", "forearm", "wrist", "rotationalwrist", "gripper::left", "gripper::right"};
  vector<Motor*> motors;

  for (const string &name : motorNames) {
    Motor *m = robot.getMotor(name);
    if (!m) {
      cerr << "ERROR: Motor " << name << " not found!" << endl;
      return 1;
    }
    m->setVelocity(1.0);
    motors.push_back(m);
  }

  // === Camera ===
  Camera *camera = robot.getCamera("camera");
  if (!camera) {
    cerr << "Camera not found!" << endl;
    return 1;
  }
  camera->enable(TIME_STEP);
  int camWidth = camera->getWidth();
  int camHeight = camera->getHeight();

  // === Distance sensor ===
  DistanceSensor *ds = robot.getDistanceSensor("ds");  // add a DistanceSensor to the robot in Webots
  if (!ds) {
      cerr << "Distance sensor not found!" << endl;
      return 1;
  }
  ds->enable(TIME_STEP);

  // === Gripper positions (positive!) ===
  double gripperOpen  = 0.07;  // fully open
  double gripperClose = 0.0;   // fully closed

  // === Predefined joint positions ===
  vector<double> homePose = {0.0, -0.5, 0.6, -1.0, 0.0, gripperOpen, gripperOpen};
  vector<double> pickPose = {0.5, -0.9, 1.0, -1.2, 0.0, gripperOpen, gripperOpen};
  vector<double> dropPose = {1.0, -1.0, 1.0, -1.2, 0.0, gripperClose, gripperClose};

  // Move to home at start
  for (int i = 0; i < 7; i++)
    motors[i]->setPosition(homePose[i]);
  for (int t = 0; t < 150; t++) robot.step(TIME_STEP);

  cout << "Robot ready, waiting for object..." << endl;

  while (robot.step(TIME_STEP) != -1) {

    // --- Check distance sensor first ---
    double distance = ds->getValue();
    double threshold = 800.0;  // adjust based on your sensor
    if (distance > threshold) {
        // no object detected
        continue;
    }

    cout << "Object detected by sensor!" << endl;

    // --- Use camera for X/Y positioning ---
    const unsigned char *image = camera->getImage();
    int xSum = 0, ySum = 0, count = 0;

    for (int y = 0; y < camHeight; y++) {
      for (int x = 0; x < camWidth; x++) {
        int r = camera->imageGetRed(image, camWidth, x, y);
        int g = camera->imageGetGreen(image, camWidth, x, y);
        int b = camera->imageGetBlue(image, camWidth, x, y);

        int brightness = (r + g + b) / 3;
        if (brightness > 150) {  // adjust threshold for your object color
          xSum += x;
          ySum += y;
          count++;
        }
      }
    }

    if (count == 0) {
        cout << "Object sensor triggered but camera cannot find it, skipping..." << endl;
        continue; // object not visible
    }

    int objX = xSum / count;
    int objY = ySum / count;
    cout << "Object position from camera: (" << objX << ", " << objY << ")" << endl;

    // --- Move to pick position ---
    for (int i = 0; i < 5; i++)
      motors[i]->setPosition(pickPose[i]);
    motors[5]->setPosition(gripperOpen);
    motors[6]->setPosition(gripperOpen);
    for (int t = 0; t < 150; t++) robot.step(TIME_STEP);

    // --- Close gripper ---
    motors[5]->setPosition(gripperClose);
    motors[6]->setPosition(gripperClose);
    for (int t = 0; t < 100; t++) robot.step(TIME_STEP);

    // --- Move to drop position ---
    for (int i = 0; i < 5; i++)
      motors[i]->setPosition(dropPose[i]);
    for (int t = 0; t < 150; t++) robot.step(TIME_STEP);

    // --- Open gripper ---
    motors[5]->setPosition(gripperOpen);
    motors[6]->setPosition(gripperOpen);
    for (int t = 0; t < 100; t++) robot.step(TIME_STEP);

    // --- Return to home ---
    for (int i = 0; i < 5; i++)
      motors[i]->setPosition(homePose[i]);
    motors[5]->setPosition(gripperOpen);
    motors[6]->setPosition(gripperOpen);
    for (int t = 0; t < 150; t++) robot.step(TIME_STEP);

    cout << "Pick and place done, waiting for next object..." << endl;
  }

  return 0;
}
