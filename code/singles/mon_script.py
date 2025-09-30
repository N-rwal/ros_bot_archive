import cv2
import numpy as np
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
import struct
import time

client = mqtt.Client()
print("attempting connect...")
client.connect("192.168.123.161", 1883, 60)
print("connected ?")
client.publish("controller/action", "walk")

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def stop():
    rotation = 0.0
    move_x = 0.0
    move_y = 0.0

# 1er param : Mouvement Gauche ou Droite
# -1 = Gauche à fond (~1m)
#  1 = Droite à fond (~1m)

# 2eme param : Rotation Gauche ou Droite
# -1 = Rotation vers la gauche (~180°)
#  1 = Rotation vers la droite (~180°)

# 3eme param : ? Pitche/Yaw (Ne fais rien)

# 4eme param : Mouvement Arrière ou Avant
# -1 = Arrière à fond (~1m)
#  1 = Avant à fond (~1m)
move_x = 0.0
move_y = 0.0
rotation = 0.0

val = 'stabilise'

# === Taille réelle du marqueur (en cm)
taille_reelle_cm = 20.0
taille_reelle_m = 0.20  # conversion en mètres

#=== Valeur X et Y Aruco
degAruco1 = np.deg2rad(-90)
Xaruco1 = 131.25
Yaruco1 = 171

degAruco2 = np.deg2rad(180)
Xaruco2 = 243
Yaruco2 = 85.5

# === Matrice caméra calibrée
camera_matrix = np.array([
    [1020.67,    0.0,     641.85],
    [   0.0,   1018.34,   355.28],
    [   0.0,     0.0,       1.0]
], dtype=np.float32)

# === Coefficients de distorsion calibrés
dist_coeffs = np.array([[-0.197, 0.133, 0.0012, 0.0008, -0.061]], dtype=np.float32)

# === Webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# === ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    gray = cv2.equalizeHist(gray)

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        for i in range(len(ids)):
            id_detecte = ids[i]
            en_rotation = False  # état mémorisé
            rot = True
            pts = corners[i][0]
            perimeter = cv2.arcLength(pts, True)
            if perimeter < 150:
                continue

            cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], taille_reelle_m, camera_matrix, dist_coeffs)

            tvec = tvecs[0][0]

            x_mesu = tvec[0] * 100
            x_corr = -1.067 * x_mesu - 53.33
            x = x_corr

            y = tvec[1] * 100
            z = tvec[2] * 100 * 0.767

            dx = z
            dy = -x
            
            

            coin = corners[i][0][0]
            x_pos, y_pos = int(coin[0]), int(coin[1])
            cv2.putText(frame, f"X:{x:.1f} Y:{y:.1f} Z:{z:.1f} cm",
                        (x_pos, y_pos - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)
            
            move_y = 0.0
            rotation = 0.0
            erreur = 80 - x  # on veut que x atteigne 80 cm
            erreur2 = - 80 - x  # on veut que x atteigne 80 cm

        



            match val:
                case 'stabilise':
                    seuil_zone_morte = z * 0.15  # zone morte dynamique = 15 % de la distance

                    if z > 100 and z < 200:
                        if z < 100:
                            move_y = 0.0
                            move_x = 0.0
                            rotation = 0.0

                        elif abs(erreur) < 8:
                            move_y = 0.0
                            move_x = 0.0
                            rotation = 0.0
                            print('is good')
                            val = 'avancer'  
                            continue         

                        elif erreur > 0:
                            move_y = 0.0
                            move_x = 0.15
                            rotation = 0.0
                        else:
                            move_y = 0.0
                            move_x = -0.15
                            rotation = 0.0

                case 'avancer':
                    move_y = 0.3
                    move_x = 0.0
                    rotation = 0.0
                    
                    if id_detecte == 21 and z <= 200:
                        val = 'stop'
                    if id_detecte == 18 and z < 300:
                        val = 'rotation'
                        continue  

                case 'rotation':
                    move_y = 0.0
                    move_x = 0.0
                    rotation = 0.35

                    if id_detecte == 20:
                        val = 'avancer'
                        continue
                case 'stop':
                    move_y = 0.0
                    move_x = 0.0
                    rotation = 0.0
                    

        


            
    cv2.imshow("Position ArUco 3D (X, Y, Z)", frame)
    payload = struct.pack('<ffff', move_x, rotation, 0.0, move_y)
    client.publish("controller/stick", payload)
    time.sleep(0.05)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()