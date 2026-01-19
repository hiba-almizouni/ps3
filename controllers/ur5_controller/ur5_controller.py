# ===================== Imports =====================
from controller import Robot
import cv2
import numpy as np
import sys

# ===================== Defines =====================
TIME_STEP = 32
GRASP_DISTANCE = 28          # IR sensor threshold (tune)
GRASP_DELAY_STEPS = int(1.0 / (TIME_STEP / 1000))  # 0.5 sec ≈ 16 steps


# ===================== States =====================
WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4

# ===================== Init =====================
robot = Robot()
state = WAITING
grasp_counter = 0

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
distance_sensor = robot.getDevice("ds_sensor")  # infrared sensor
distance_sensor.enable(TIME_STEP)

position_sensor = robot.getDevice("wrist_1_joint_sensor")
position_sensor.enable(TIME_STEP)

# ===================== OpenCV Window =====================
cv2.namedWindow("UR5 Camera View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("UR5 Camera View", 600, 400)

# ===================== Main Loop =====================
while robot.step(TIME_STEP) != -1:

    distance = distance_sensor.getValue()
    position = position_sensor.getValue()

    # ===================== CAMERA IMAGE =====================
    image = camera.getImage()
    if image is None:
        continue

    width = camera.getWidth()
    height = camera.getHeight()

    img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    # ===================== EDGE DETECTION =====================
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for cnt in contours:
        if cv2.contourArea(cnt) > 100:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)

    # ===================== COLOR DETECTION (WAITING ONLY) =====================
    if state == WAITING:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([40, 40, 40])
        upper = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours_color, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours_color:
            if cv2.contourArea(cnt) > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3)

    # ===================== DISPLAY =====================
    cv2.imshow("UR5 Camera View", img)
    cv2.waitKey(3)

    # ===================== FSM =====================
    if state == WAITING:
        print(f"[WAITING] distance = {distance:.2f}")

        if distance < GRASP_DISTANCE:
            grasp_counter += 1
            print(f"  stable... {grasp_counter}/{GRASP_DELAY_STEPS}")
            print("Object stable -> GRASPING")
            
            counter = 0
            state = GRASPING
        else:
            grasp_counter = 0

    elif state == GRASPING:
        counter+=1
        for m in hand_motors:
                m.setVelocity(0.8)
                m.setPosition(0.85)
        
        
        if counter >= GRASP_DELAY_STEPS :
            print("Grasp complete -> ROTATING")
            state = ROTATING
            for m in hand_motors:
                m.setVelocity(m.getMaxVelocity())
            for i in range(4):
                ur_motors[i].setPosition(target_positions[i])

    elif state == ROTATING:
        if position < -2.3:
            print("Target reached -> RELEASING")
            for m in hand_motors:
                m.setPosition(m.getMinPosition())
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

cv2.destroyAllWindows()
