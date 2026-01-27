# ===================== Imports =====================
from controller import Supervisor
import cv2
import numpy as np
import sys

# ===================== Defines =====================
TIME_STEP = 32
GRASP_DISTANCE = 28
GRASP_DELAY_STEPS = int(1.0 / (TIME_STEP / 1000))
STARTUP_DELAY_STEPS = int(2.0 / (TIME_STEP / 1000))  # 2 secondes au démarrage

# ===================== States =====================
STARTUP = -1  # Nouvel état de démarrage
WAITING = 0
DETECTING = 1
GRASPING = 2
ROTATING = 3
RELEASING = 4
ROTATING_BACK = 5

# ===================== Init =====================
robot = Supervisor()
state = STARTUP  # Commencer en mode STARTUP
grasp_counter = 0
counter = 0
piece_orientation = 0.0
startup_counter = 0  # Compteur pour le délai de démarrage

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

# ===================== INITIAL POSE =====================
INITIAL_POSE = [
    0.0,
   -0.05,
   -0.9,
   -0.65,
   -1.5,
    0.0
]
for m, pos in zip(ur_motors, INITIAL_POSE):
    m.setPosition(pos)

# ===================== Gripper Motors =====================
hand_names = [
    "finger_1_joint_1",
    "finger_2_joint_1",
    "finger_middle_joint_1"
]
hand_motors = [robot.getDevice(n) for n in hand_names]
for m in hand_motors:
    m.setVelocity(0.6)
    m.setPosition(0.0)

# ===================== Camera =====================
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

# ===================== Sensors =====================
distance_sensor = robot.getDevice("ds_sensor")
distance_sensor.enable(TIME_STEP)

piece_detected = robot.getDevice("presence_piece")
piece_detected.enable(TIME_STEP)

position_sensor = robot.getDevice("wrist_1_joint_sensor")
position_sensor.enable(TIME_STEP)

# ===================== OpenCV Window =====================
cv2.namedWindow("UR5 Camera View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("UR5 Camera View", 600, 400)

# ===================== Target positions =====================
target_positions = [-1.88, -2.14, -2.38, -1.51]

# ===================== Zone de détection fixe =====================
DETECTION_ZONE_CENTER_X = 320
DETECTION_TOLERANCE = 50

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
    position = position_sensor.getValue()

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

    # ===================== FSM AMÉLIORÉ =====================
    
    if state == STARTUP:
        # Attendre 2 secondes avant de commencer
        startup_counter += 1
        if startup_counter % 10 == 0:  # Afficher tous les 10 steps
            remaining_time = (STARTUP_DELAY_STEPS - startup_counter) * TIME_STEP / 1000
            print(f"[STARTUP] Initialisation... {remaining_time:.1f}s restantes")
        
        if startup_counter >= STARTUP_DELAY_STEPS:
            print("[STARTUP] Initialisation terminée → WAITING")
            state = WAITING
            startup_counter = 0
    
    elif state == WAITING:
        print(f"[WAITING] distance = {distance:.2f}, distancee = {distancee:.2f}")
        
        # Arrêter le convoyeur si distancee < 1000
        if distancee < 1000:
            print("[WAITING] distancee < 1000 → STOP conveyor")
            cb_speed.setSFFloat(0.0)  # STOP conveyor
            grasp_counter += 1
            
            # Attendre 5 steps après l'arrêt du convoyeur
            if grasp_counter >= 5:
                # Vérifier si la pièce est dans la zone de vision
                if detected and is_piece_in_detection_zone(cx):
                    print(f"[WAITING] Pièce détectée dans la zone! Position X: {cx}")
                    state = DETECTING
                    grasp_counter = 0
        else:
            grasp_counter = 0

    elif state == DETECTING:
        print(f"[DETECTING] Angle: {angle:.2f}°, Position: ({cx}, {cy})")
        
        # Capturer l'orientation de la pièce
        piece_orientation = angle
        
        # Attendre que la pièce soit à distance de grasp
        if distance < GRASP_DISTANCE:
            grasp_counter += 1
            if grasp_counter >= GRASP_DELAY_STEPS:
                counter = 0
                state = GRASPING
                print(f"[DETECTING] ✓ Orientation capturée: {piece_orientation:.2f}°")
                print(f"[PIECE] Angle = {piece_orientation:.2f}°")
        else:
            grasp_counter = 0

    elif state == GRASPING:
        counter += 1
        
        # Ajuster l'orientation du wrist_3 selon la pièce
        wrist_angle = np.deg2rad(piece_orientation)
        ur_motors[5].setPosition(wrist_angle)
        
        # Fermer la pince
        for m in hand_motors:
            m.setVelocity(0.8)
            m.setPosition(0.85)

        if counter >= GRASP_DELAY_STEPS:
            state = ROTATING
            # Déplacer vers la position cible
            for i in range(4):
                ur_motors[i].setPosition(target_positions[i])
            print(f"[GRASPING] Wrist_3 ajusté à: {piece_orientation:.2f}°")

    elif state == ROTATING:
        if position < -2.3:
            # Ouvrir la pince pour relâcher
            for m in hand_motors:
                m.setPosition(m.getMinPosition())
            state = RELEASING

    elif state == RELEASING:
        # Retourner à la position initiale
        for i, m in enumerate(ur_motors):
            m.setPosition(INITIAL_POSE[i])
        state = ROTATING_BACK

    elif state == ROTATING_BACK:
        if position > -0.1:
            # Redémarrer le convoyeur
            print("[ROTATING_BACK] Redémarrage du convoyeur")
            cb_speed.setSFFloat(CONVEYOR_RUNNING_SPEED)  # Restart conveyor
            
            # Réinitialiser les variables
            piece_orientation = 0.0
            grasp_counter = 0
            state = WAITING
            print("[ROTATING_BACK] Cycle terminé, retour à WAITING\n")

cv2.destroyAllWindows()