# ===================== Imports =====================
from controller import Robot
import cv2
import numpy as np
import sys

# ===================== Defines =====================
TIME_STEP = 32
CALIBRATION_STEPS = 50
DETECTION_DROP = 0.15

# ===================== States =====================
WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4

# ===================== Init =====================
robot = Robot()

state = WAITING
counter = 0
baseline_distance = 0.0
calibrated = False

target_positions = [-1.88, -2.14, -2.38, -1.51]

# ===================== Speed =====================
speed = 1.0
if len(sys.argv) == 2:
    speed = float(sys.argv[1])

# ===================== Gripper Motors =====================
hand_names = [
    "finger_1_joint_1",
    "finger_2_joint_1",
    "finger_middle_joint_1"
]
hand_motors = [robot.getDevice(n) for n in hand_names]

# ===================== UR5 Motors =====================
ur_names = [
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint"
]
ur_motors = [robot.getDevice(n) for n in ur_names]
for m in ur_motors:
    m.setVelocity(speed)

# ===================== Camera =====================
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
# ===================== Sensors =====================
distance_sensor = robot.getDevice("distance sensor")
distance_sensor.enable(TIME_STEP)

position_sensor = robot.getDevice("wrist_1_joint_sensor")
position_sensor.enable(TIME_STEP)

# Create named window with resizable property
cv2.namedWindow("UR5 Camera View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("UR5 Camera View", 600, 400)  # Reasonable size window

# ===================== Main Loop =====================
while robot.step(TIME_STEP) != -1:

    distance = distance_sensor.getValue()
    position = position_sensor.getValue()

    # ===================== CAMERA IMAGE =====================
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    if image is None:
        continue
        
    img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)  # Proper conversion

    # ===================== CALIBRATION =====================
    if not calibrated:
        baseline_distance += distance
        counter += 1
        if counter >= CALIBRATION_STEPS:
            baseline_distance /= CALIBRATION_STEPS
            calibrated = True
            counter = 0
            print(f"Calibration done | Baseline = {baseline_distance:.3f}")
        continue

    # ===================== EDGE DETECTION (ONLY) =====================
    # Convert to grayscale for edge detection
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)
    
    # Find contours from edges
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # For each significant contour, draw a bounding rectangle
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:  # Filter small contours
            x, y, w, h = cv2.boundingRect(cnt)
            # Draw red bounding box around detected edges
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)

    # ===================== COLOR DETECTION (WAITING ONLY) =====================
    if state == WAITING:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # ---- GREEN COLOR DETECTION ----
        lower = np.array([40, 40, 40])
        upper = np.array([80, 255, 255])
        # -------------------------------------------------

        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours_color, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours_color:
            area = cv2.contourArea(cnt)
            if area < 300:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # Draw red bounding box for detected objects
            cv2.rectangle(
                img,
                (x, y),
                (x + w, y + h),
                (0, 0, 255),
                3
            )

    # ===================== DISPLAY (NO SCALING NEEDED) =====================
    cv2.imshow("UR5 Camera View", img)
    cv2.waitKey(3)

    # ===================== FSM =====================
    if counter <= 0:

        if state == WAITING:
            if distance < baseline_distance - DETECTION_DROP:
                print("Object detected -> GRASPING")
                for m in hand_motors:
                    m.setPosition(0.85)
                counter = 10
                state = GRASPING

        elif state == GRASPING:
            print("Grasp complete -> ROTATING")
            for i in range(4):
                ur_motors[i].setPosition(target_positions[i])
            state = ROTATING

        elif state == ROTATING:
            if position < -2.3:
                print("Target reached -> RELEASING")
                for m in hand_motors:
                    m.setPosition(m.getMinPosition())
                counter = 10
                state = RELEASING

        elif state == RELEASING:
            print("Release done -> ROTATING_BACK")
            for m in ur_motors:
                m.setPosition(0.0)
            state = ROTATING_BACK

        elif state == ROTATING_BACK:
            if position > -0.1:
                print("Home -> WAITING")
                state = WAITING

    if counter > 0:
        counter -= 1

cv2.destroyAllWindows()