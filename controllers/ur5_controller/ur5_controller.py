# ===================== Imports =====================
from controller import Supervisor
import cv2
import numpy as np
import sys

# ===================== Defines =====================
TIME_STEP = 32
GRASP_DISTANCE = 200  # Distance pour confirmer la position de grasp
GRASP_DELAY_STEPS = int(1.0 / (TIME_STEP / 1000))
STARTUP_DELAY_STEPS = int(2.0 / (TIME_STEP / 1000))  # 2 secondes au démarrage
POSITION_TOLERANCE = 0.4  # Tolérance pour considérer qu'un joint a atteint sa position

# ===================== States =====================
STARTUP = -1
WAITING = 0
ALIGNING = 1
GRASPING = 2
ROTATING = 3
RELEASING = 4
ROTATING_BACK = 5

# ===================== Init =====================
robot = Supervisor()
state = STARTUP
grasp_counter = 0
counter = 0
piece_orientation = 0.0
startup_counter = 0

# ===================== Conveyor (Supervisor Logic) =====================
cb = robot.getFromDef("cb")
if cb is None:
    print("ERROR: Conveyor 'cb' not found in world")
    exit()

cb_speed = cb.getField("speed")
CONVEYOR_RUNNING_SPEED = 0.2
cb_speed.setSFFloat(CONVEYOR_RUNNING_SPEED)  # Start conveyor

# ===================== Speed =====================
speed = 1.0
if len(sys.argv) == 2:
    speed = float(sys.argv[1])

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

# ===================== Position Sensors for all joints =====================
ur_position_sensors = []
for name in ur_names:
    sensor = robot.getDevice(name + "_sensor")
    sensor.enable(TIME_STEP)
    ur_position_sensors.append(sensor)

# ===================== INITIAL POSE =====================
INITIAL_POSE = [
    0.0,
   -0.05,
   -0.9,
   -0.65,
   -1.5,
    0.0
]

# ===================== PICKING POSITION =====================
PICKING_POSITION = [
    0.0,
   -0.8,
    1.1,
   -1.5,
   -1.5,
    0.0  # wrist_3 sera remplacé par l'angle détecté
]

# Set initial pose
for m, pos in zip(ur_motors, INITIAL_POSE):
    m.setPosition(pos)

# ===================== Robotiq 2F-85 Gripper =====================
# Le gripper Robotiq2f85 a deux moteurs pour les doigts
gripper_left = robot.getDevice("Gripper::left finger joint")
gripper_right = robot.getDevice("Gripper::right finger joint")

if gripper_left is None or gripper_right is None:
    print("ERROR: ROBOTIQ gripper motors not found")
    print("Available devices:")
    for i in range(robot.getNumberOfDevices()):
        d = robot.getDeviceByIndex(i)
        print(f"  - {d.getName()}")
    exit()

# Configuration des deux doigts
gripper_left.setVelocity(0.05)
gripper_right.setVelocity(0.05)

# Position initiale : ouvert
gripper_left.setPosition(0.0)
gripper_right.setPosition(0.0)

# ===================== Camera =====================
camera = robot.getDevice("camera")
if camera is None:
    print("ERROR: Camera not found")
    exit()
camera.enable(TIME_STEP)

# ===================== Sensors =====================
distance_sensor = robot.getDevice("ds_sensor")
distance_sensor.enable(TIME_STEP)

piece_detected = robot.getDevice("presence_piece")
piece_detected.enable(TIME_STEP)

# ===================== OpenCV Window =====================
cv2.namedWindow("UR5 Camera View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("UR5 Camera View", 600, 400)

# ===================== Target positions =====================
target_positions = [-1.88, -2.14, -2.38, -1.51]

# ===================== Zone de détection fixe =====================
DETECTION_ZONE_CENTER_X = 320
DETECTION_TOLERANCE = 50

# ===================== Helper Functions =====================
def check_position_reached(target_positions, tolerance=POSITION_TOLERANCE):
    """Vérifie si tous les joints ont atteint leur position cible"""
    for i, sensor in enumerate(ur_position_sensors):
        current_pos = sensor.getValue()
        target_pos = target_positions[i]
        if abs(current_pos - target_pos) > tolerance:
            return False
    return True

# ===================== Fonction de détection améliorée =====================
def detect_piece_orientation(img):
    """
    Détecte la pièce verte et calcule son orientation.
    Retourne: (orientation_angle, center_x, center_y, detected)
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Masque vert amélioré
    lower_green = np.array([40, 60, 60])
    upper_green = np.array([80, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Morphologie pour nettoyer le masque
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Trouver les contours DIRECTEMENT sur le masque
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 0:
        return 0.0, 0, 0, False
    
    # Trouver le plus grand contour (la pièce)
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    
    # Filtrer les petits contours (bruit)
    if area < 500:
        return 0.0, 0, 0, False
    
    # Calculer le rectangle orienté minimum
    rect = cv2.minAreaRect(largest_contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    
    # Extraire les informations
    center = rect[0]
    size = rect[1]
    angle = rect[2]
    
    # Correction de l'angle OpenCV
    width, height = size
    if width < height:
        angle = angle + 90
    
    # Dessiner le rectangle orienté
    cv2.drawContours(img, [box], 0, (0, 255, 255), 2)
    
    # Dessiner le centre
    cx, cy = int(center[0]), int(center[1])
    cv2.circle(img, (cx, cy), 5, (255, 0, 0), -1)
    
    # Dessiner l'axe d'orientation
    length = 50
    angle_rad = np.deg2rad(angle)
    end_x = int(cx + length * np.cos(angle_rad))
    end_y = int(cy + length * np.sin(angle_rad))
    cv2.line(img, (cx, cy), (end_x, end_y), (255, 0, 255), 2)
    
    return angle, cx, cy, True

# ===================== Fonction pour vérifier si pièce dans zone =====================
def is_piece_in_detection_zone(center_x):
    """Vérifie si la pièce est dans la zone de détection fixe"""
    return abs(center_x - DETECTION_ZONE_CENTER_X) < DETECTION_TOLERANCE

# ===================== Main Loop =====================
print("[STARTUP] Délai de 2 secondes avant démarrage de la détection...")

while robot.step(TIME_STEP) != -1:
    distance = distance_sensor.getValue()
    distancee = piece_detected.getValue()

    image = camera.getImage()
    if image is None:
        continue

    w = camera.getWidth()
    h = camera.getHeight()
    img = np.frombuffer(image, np.uint8).reshape((h, w, 4))
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    # ==================================================
    # DÉTECTION AMÉLIORÉE
    # ==================================================
    angle, cx, cy, detected = detect_piece_orientation(img)
    
    # Dessiner la zone de détection
    zone_x = DETECTION_ZONE_CENTER_X
    cv2.line(img, (zone_x - DETECTION_TOLERANCE, 0), 
             (zone_x - DETECTION_TOLERANCE, h), (0, 0, 255), 2)
    cv2.line(img, (zone_x + DETECTION_TOLERANCE, 0), 
             (zone_x + DETECTION_TOLERANCE, h), (0, 0, 255), 2)

    # ===================== DISPLAY (NO TEXT) =====================
    cv2.imshow("UR5 Camera View", img)
    cv2.waitKey(3)

    # ===================== FSM =====================
    
    if state == STARTUP:
        # Attendre 2 secondes avant de commencer
        startup_counter += 1
        if startup_counter % 10 == 0:
            remaining_time = (STARTUP_DELAY_STEPS - startup_counter) * TIME_STEP / 1000
            print(f"[STARTUP] Initialisation... {remaining_time:.1f}s restantes")
        
        if startup_counter >= STARTUP_DELAY_STEPS:
            print("[STARTUP] Initialisation terminée → WAITING")
            state = WAITING
            startup_counter = 0
    
    elif state == WAITING:
        print(f"[WAITING] distance = {distance:.2f}, distancee = {distancee:.2f}")
        
        # Détecter la pièce avec distancee < 1000
        if distancee < 1000:
            if grasp_counter == 0:
                print("[WAITING] distancee < 1000 → Pièce détectée, convoyeur continue 6 steps...")
            
            grasp_counter += 1
            
            # Arrêter le convoyeur après 6 steps supplémentaires
            if grasp_counter == 6:
                print("[WAITING] → STOP conveyor (après 6 steps)")
                cb_speed.setSFFloat(0.0)  # STOP conveyor
            
            # Attendre 3 steps de plus après l'arrêt (total 9 steps après détection)
            if grasp_counter >= 9:
                # Capturer l'angle de la pièce (si détectée)
                if detected:
                    piece_orientation = angle
                    print(f"========================================")
                    print(f"[PIECE] ANGLE CAPTURÉ = {piece_orientation:.2f}°")
                    print(f"========================================")
                else:
                    # Si pas détectée par vision, utiliser angle par défaut
                    piece_orientation = 0.0
                    print(f"[WAITING] Pièce non visible par caméra, angle par défaut = 0.0°")
                
                # Déplacer vers la position de picking avec wrist_3 orienté
                picking_pos = PICKING_POSITION.copy()
                picking_pos[5] = np.deg2rad(piece_orientation)  # wrist_3 = angle détecté
                
                for i, m in enumerate(ur_motors):
                    m.setPosition(picking_pos[i])
                
                print(f"[WAITING] → ALIGNING (moving to picking position)")
                state = ALIGNING
                grasp_counter = 0
        else:
            grasp_counter = 0

    elif state == ALIGNING:
        # Vérifier si la position est atteinte
        picking_pos = PICKING_POSITION.copy()
        picking_pos[5] = np.deg2rad(piece_orientation)
        
        position_reached = check_position_reached(picking_pos)
        
        # DEBUG: Afficher les positions actuelles vs cibles
        if counter % 10 == 0:
            print(f"[ALIGNING DEBUG] distance={distance:.2f}, position_reached={position_reached}")
            for i, sensor in enumerate(ur_position_sensors):
                current = sensor.getValue()
                target = picking_pos[i]
                diff = abs(current - target)
                if diff > 0.05:  # Afficher seulement les joints qui ne sont pas OK
                    print(f"  Joint {i} ({ur_names[i]}): current={current:.3f}, target={target:.3f}, diff={diff:.3f}")
        
        if position_reached:
            print(f"[ALIGNING] Position atteinte! distance={distance:.2f}")
            print(f"[ALIGNING] → GRASPING")
            state = GRASPING
            counter = 0
        
        counter += 1

    elif state == GRASPING:
        counter += 1
        
        # Fermer le gripper Robotiq (les deux doigts)
        gripper_left.setVelocity(0.8)
        gripper_right.setVelocity(0.8)
        gripper_left.setPosition(0.85)
        gripper_right.setPosition(0.85)

        if counter >= GRASP_DELAY_STEPS:
            print(f"[GRASPING] Pièce saisie → ROTATING")
            state = ROTATING
            counter = 0
            # Déplacer vers la position cible
            for i in range(4):
                ur_motors[i].setPosition(target_positions[i])

    elif state == ROTATING:
        wrist_1_position = ur_position_sensors[3].getValue()
        
        if counter % 10 == 0:
            print(f"[ROTATING] En déplacement vers target... wrist_1={wrist_1_position:.2f}")
        counter += 1
        
        if wrist_1_position < -2.3:
            print(f"[ROTATING] Position target atteinte → RELEASING")
            # Ouvrir le gripper Robotiq pour relâcher
            gripper_left.setPosition(0.0)
            gripper_right.setPosition(0.0)
            state = RELEASING
            counter = 0

    elif state == RELEASING:
        counter += 1
        
        if counter >= GRASP_DELAY_STEPS:
            print(f"[RELEASING] Pièce relâchée → ROTATING_BACK")
            # Retourner à la position initiale
            for i, m in enumerate(ur_motors):
                m.setPosition(INITIAL_POSE[i])
            state = ROTATING_BACK
            counter = 0

    elif state == ROTATING_BACK:
        position_reached = check_position_reached(INITIAL_POSE)
        
        if counter % 10 == 0:
            print(f"[ROTATING_BACK] Retour à position initiale...")
        counter += 1
        
        if position_reached:
            print("[ROTATING_BACK] Position initiale atteinte")
            print("[ROTATING_BACK] Redémarrage du convoyeur")
            cb_speed.setSFFloat(CONVEYOR_RUNNING_SPEED)  # Restart conveyor
            
            # Réinitialiser les variables
            piece_orientation = 0.0
            grasp_counter = 0
            counter = 0
            state = WAITING
            print("[ROTATING_BACK] Cycle terminé → WAITING\n")

cv2.destroyAllWindows()