# ===================== Imports =====================
from controller import Supervisor
import cv2
import numpy as np
import sys
import struct

# ===================== Defines =====================
TIME_STEP = 32
GRASP_DISTANCE = 350       # Object detected when distance BELOW belt surface (~357)
GRASP_DELAY_STEPS = int(0.5 / (TIME_STEP / 1000))  # 0.5 sec ≈ 16 steps

# ===================== States =====================
WAITING = 0
ALIGNING = 1
GRASPING = 2
ROTATING = 3
RELEASING = 4
ROTATING_BACK = 5

# ===================== Init =====================
robot = Supervisor()
state = WAITING

object_center = None
object_angle = 0.0
object_detected = False

error_x = 0
error_y = 0
angle_error = 0

grasp_counter = 0
current_distance = 1000.0  # Initialize to max distance (no object)
sensor_ready = False       # Flag to ensure we received first sensor reading

target_positions = [-1.88, -2.14, -2.38, -1.51]

# ===================== Conveyor =====================
cb = robot.getFromDef("cb")
if cb is None:
    print("ERROR: Conveyor not found")
    exit()

cb_speed = cb.getField("speed")
CONVEYOR_RUNNING_SPEED = 0.2
cb_speed.setSFFloat(CONVEYOR_RUNNING_SPEED)  # start moving

# ===================== Receiver (to get sensor data) =====================
receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)
print("Receiver enabled, waiting for sensor data...")

# ===================== Speed =====================
speed = 1.0
if len(sys.argv) == 2:
    speed = float(sys.argv[1])

# ===================== Gripper Motors =====================
hand_names = [
    "gripper::left finger joint",
    "gripper::right finger joint"
]
hand_motors = [robot.getDevice(n) for n in hand_names]

# ===================== UR5 Motors =====================
ur_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]
ur_motors = [robot.getDevice(n) for n in ur_names]
for m in ur_motors:
    m.setVelocity(speed)

# ===================== Camera =====================
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

# ===================== Position Sensor =====================
position_sensor = robot.getDevice("wrist_1_joint_sensor")
position_sensor.enable(TIME_STEP)

# ===================== OpenCV Window =====================
cv2.namedWindow("UR5 Camera View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("UR5 Camera View", 600, 400)

# ===================== Helper Functions =====================
def get_sensor_distance():
    """Receive distance from the fixed sensor robot via receiver"""
    global current_distance, sensor_ready
    
    if receiver.getQueueLength() > 0:
        # Get the message as bytes
        message = receiver.getBytes()
        if message:
            # Unpack the float value
            distance = struct.unpack('f', message)[0]
            current_distance = distance
            sensor_ready = True  # Mark that we received valid data
        # Clear the message
        receiver.nextPacket()
    
    return current_distance

def reset_arm_to_home():
    """Reset UR5 arm to home position"""
    for m in ur_motors:
        m.setPosition(0.0)

def set_picking_position():
    """Move arm to picking position above conveyor"""
    ur_motors[0].setPosition(-2.0)      # shoulder_pan
    ur_motors[1].setPosition(-1.95)     # shoulder_lift
    ur_motors[2].setPosition(-1.0)      # elbow
    ur_motors[3].setPosition(-1.95)     # wrist_1
    ur_motors[4].setPosition(1.57)      # wrist_2
    ur_motors[5].setPosition(-1.50)     # wrist_3

def open_gripper():
    """Open the gripper"""
    for m in hand_motors:
        m.setPosition(m.getMaxPosition())

def close_gripper():
    """Close the gripper"""
    for m in hand_motors:
        m.setPosition(m.getMinPosition())

def set_target_position():
    """Move to target drop-off position"""
    for i in range(4):
        ur_motors[i].setPosition(target_positions[i])

# ===================== Initialize =====================
# Set robot to picking position at startup
set_picking_position()
open_gripper()

print("UR5 Controller initialized. Robot in picking position, waiting for objects...")

# ===================== Main Loop =====================
while robot.step(TIME_STEP) != -1:

    # Get distance from FIXED sensor via receiver
    distance = get_sensor_distance()
    
    # Get wrist position for state transitions
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

    object_detected = False

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 300:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.intp(box)

            (cx, cy), (w, h), angle = rect

            if w < h:
                angle += 90

            object_center = (int(cx), int(cy))
            object_angle = angle
            object_detected = True

            cv2.drawContours(img, [box], 0, (0, 255, 0), 2)
            cv2.circle(img, object_center, 4, (0, 0, 255), -1)

    if object_detected:
        img_cx = width // 2
        img_cy = height // 2

        error_x = object_center[0] - img_cx
        error_y = object_center[1] - img_cy
        angle_error = object_angle

    # Add distance info to image
    cv2.putText(img, f"Distance: {distance:.2f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(img, f"State: {['WAITING', 'ALIGNING', 'GRASPING', 'ROTATING', 'RELEASING', 'ROTATING_BACK'][state]}", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # ===================== DISPLAY =====================
    cv2.imshow("UR5 Camera View", img)
    cv2.waitKey(3)

    # ===================== FSM =====================
    if state == WAITING:
        # Only process if we have received valid sensor data
        if not sensor_ready:
            print("[WAITING] Waiting for sensor data...", end='\r')
        else:
            print(f"[WAITING] Fixed sensor distance = {distance:.2f}", end='\r')

            # Object detected when distance is BELOW threshold (belt surface ~357)
            if distance < GRASP_DISTANCE:
                print(f"\n[WAITING] Object detected! Distance = {distance:.2f}")
                print("Object detected by FIXED sensor → STOP conveyor")
                cb_speed.setSFFloat(0.0)   # STOP conveyor
                grasp_counter = 0
                state = GRASPING  # Go directly to grasping (already in position)
                close_gripper()
                print("[GRASPING] Closing gripper...")

    elif state == ALIGNING:
        # This state is no longer used, but kept for compatibility
        # Robot is always in picking position now
        pass

    elif state == GRASPING:
        grasp_counter += 1
        
        if grasp_counter % 5 == 0:
            print(f"[GRASPING] Closing gripper... {grasp_counter}/{GRASP_DELAY_STEPS}")
        
        if grasp_counter >= GRASP_DELAY_STEPS:
            print("[GRASPING] Grasp complete → ROTATING to target")
            set_target_position()
            state = ROTATING
            grasp_counter = 0

    elif state == ROTATING:
        if position % 0.5 < 0.1:  # Print occasionally
            print(f"[ROTATING] Moving to target... position={position:.2f}")
        
        if position < -2.3:
            print("[ROTATING] Target reached → RELEASING")
            open_gripper()
            state = RELEASING
            grasp_counter = 0

    elif state == RELEASING:
        grasp_counter += 1
        
        if grasp_counter % 5 == 0:
            print(f"[RELEASING] Opening gripper... {grasp_counter}/{GRASP_DELAY_STEPS}")
        
        if grasp_counter >= GRASP_DELAY_STEPS:
            print("[RELEASING] Release done → ROTATING_BACK to pick position")
            set_picking_position()  # Return to picking position instead of home
            state = ROTATING_BACK
            grasp_counter = 0

    elif state == ROTATING_BACK:
        if abs(position) % 0.5 < 0.1:  # Print occasionally
            print(f"[ROTATING_BACK] Returning to pick position... position={position:.2f}")
        
        # Check if returned to picking position (position close to -1.95)
        if position > -2.0 and position < -1.9:
            print("[ROTATING_BACK] Pick position reached → WAITING")
            print("Restarting conveyor\n")
            cb_speed.setSFFloat(CONVEYOR_RUNNING_SPEED)  # Restart conveyor
            state = WAITING
            grasp_counter = 0
            sensor_ready = False  # Reset for next cycle

cv2.destroyAllWindows()