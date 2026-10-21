import serial
import os
import time
import socket
import threading
import json
import numpy as np
import math
import csv

from arduino_driver_v2 import ArduinoIO
from gps_driver_v2 import GpsIO
from imu9_driver_v2 import Imu9IO

# Initialisation de la position gps
pos = [4825.154359,'N',428.162682,'W',0.]
my_current_pos = [0.,'N',0.,'W',0.]
u1, u2 = 0., 0.

eta = 0.4

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi 

def nmea_to_decimal(value, direction):
    # Séparer les degrés des minutes
    # Pour la latitude (DDmm.mmmm), les degrés sont les 2 premiers chiffres
    # Pour la longitude (DDDmm.mmmm), ce sont les 3 premiers
    # Astuce : on divise par 100 et on prend la partie entière
    deg = int(value / 100)
    minutes = value - (deg * 100)
    decimal = deg + (minutes / 60)
    
    # Sud ou Ouest = Négatif
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

def local_coord(gps_data, ref): # pour manip réelle
    """
    Fonction de conversion des données GPS du repère (lat, lon) dans le repère local (x, y) d'origine (ref).

    params:
    - gps_data: Ensembles des données reçues depuis le GPS (lat, N, lon, W, gps_time)
    - ref: Coordonnées GPS en (lat, lon) de l'origine du repère local
    return:
    - [xl, yl]: position (x, y) dans le repère local
    """
    # lat , lon = gps_data[0], gps_data[2]
    lat = nmea_to_decimal(gps_data[0], gps_data[1])
    lon = nmea_to_decimal(gps_data[2], gps_data[3])

    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    # lat0, lon0 = 
    # lat0, lon0 = 4825.154359, 428.162682
    # lon0, lat0 = -0.07800405 , 0.84507466
    # Conversion NMEA DDMM.MMMM -> Degrés Décimaux -> Radians
    lat0_deg = (48 + 25.154359 / 60)
    lon0_deg = -(4 + 28.162682 / 60) # Négatif pour l'Ouest (Brest)
    
    lat0 = np.radians(lat0_deg) # approx 0.8450751
    lon0 = np.radians(lon0_deg) # approx -0.0780054
    # lon0, lat0 = -0.07800520 , 0.84507476 #0.84507466


    psi0 = -0.0677 # rotation du repère locale pour avoir y selon le bord de la piscine
    rho = 6371000 # rayon de la terre en mètres

    dy = rho * (lat_rad - lat0)
    dx = rho * np.cos(lat0) * (lon_rad - lon0)

    # Matrice de rotation du repère local
    Rloc = np.array([[np.cos(psi0), -np.sin(psi0)],
                    [np.sin(psi0), np.cos(psi0)]])
    
    Xloc = Rloc@ np.array([[dx],[dy]])
    offset_x = 0
    offset_y = 0
    Xloc -= np.array([[offset_x], [offset_y]])

    
    return Xloc.flatten()

# def local_coord(gps_data, ref_lat_dec, ref_lon_dec):
#     # 1. Conversion des données actuelles en degrés décimaux
#     lat = nmea_to_decimal(gps_data[0], gps_data[1])
#     lon = nmea_to_decimal(gps_data[2], gps_data[3])
    
#     # 2. Passage en RADIANS pour les calculs mathématiques
#     lat_rad = np.radians(lat)
#     lon_rad = np.radians(lon)
#     # lat0_rad = np.radians(ref_lat_dec)
#     lat0_rad = 0
#     # lon0_rad = np.radians(ref_lon_dec)
#     lon0_rad = np.radians(ref_lon_dec)

#     rho = 6371000  # Rayon moyen de la Terre en mètres
#     psi0 = -0.0677 # Angle de la piscine en radians

#     # 3. Calcul des distances métriques (Approximation par petit angle)
#     # dy = Nord, dx = Est
#     dy = rho * (lat_rad - lat0_rad)
#     dx = rho * np.cos(lat0_rad) * (lon_rad - lon0_rad)
    
#     # 4. Matrice de rotation (Repère local)
#     Rloc = np.array([[np.cos(psi0), -np.sin(psi0)],
#                      [np.sin(psi0),  np.cos(psi0)]])
    
#     # Application de la rotation sur le vecteur [dx, dy]
#     Xloc = Rloc @ np.array([[dx], [dy]])
    
#     return Xloc.flatten()

def cap_calib(mag_raw, my_id): # pour manip réelle
    """
    Fonction de mesure du cap du robot avec les données du magnétomètre
    et ses données de calibrations.

    params:
    - mag_raw: Mesures (mx, my, mz) non calibrées directement reçues du magnétomètre
    return:
    - cap: La mesure calibrée du cap
    """
    xmag = np.array([[mag_raw[0]],
                     [mag_raw[1]],
                     [mag_raw[2]]])
    
    # invA = np.array([[-2997.61156126, -1297.53703632,  -865.2553575 ],
    #                  [-1075.49905853,  3038.1765926,    217.74686775],
    #                  [  622.80561273,   299.29145391, -3294.6197577 ]])

    # if my_id == "robot1":
    #     invA = np.array([[-3.51524342e-04, -4.29796138e-05, -4.45837274e-05],
    #                      [-4.04961156e-05,  3.55360871e-04, -1.87753845e-05],
    #                      [ 1.77942028e-05, -5.09029746e-05, -3.04295794e-04]])
        
    #     xbar = np.array([[-1043.93653822],
    #                      [ -529.24420523],
    #                      [ -478.41277581]])
        
    if my_id == "robot1":
        invA = np.array([[-2.75736695e-04, -1.24086762e-04,  6.42147392e-05],
                        [-9.32663827e-05,  2.85044041e-04,  4.33332812e-05],
                        [-6.05970361e-05,  2.43709870e-06, -2.87449687e-04]])
        
        xbar = np.array([[-1230.20955861],
                        [ -323.89942365],
                        [ -615.08824771]])
    
    if my_id == "robot2":
        invA = np.array([[-3.25486347e-04,  2.37519601e-05,  1.83248423e-05],
                         [-6.21890904e-05, -3.21356373e-04,  7.46167541e-05],
                         [ 1.84890346e-05,  7.99574712e-05,  2.85456929e-04]])
        
        xbar = np.array([[-841.75833415],
                         [-297.28519492],
                         [ 172.35021093]])
        
    if my_id == "robot3":
        invA = np.array([[-3.13521168e-04, -6.74945465e-05, -2.54489932e-05],
                         [-5.78049723e-05,  3.21248511e-04, -3.65890892e-05],
                         [ 3.76209813e-05, -3.89217381e-05, -2.90289422e-04]])
        
        xbar = np.array([[-1582.86966103],
                         [  554.28074317],
                         [   45.76492052]])
        
    

    # y = np.dot(invA,(xmag - xbar))
    y = invA@(xmag - xbar)
    cap = -np.arctan2(y[1], y[0])
    
    return cap

def f_pot(x, Xr):
    """
    Fonction générant un champ de potentiel circulaire pour des cycles continus et 
    des pôles répulsifs sur les coordonnées des autres robots.

    params:
    - x: Vecteur d'état du robot (x, y, v, theta)
    - Xr: liste contenant les coordonnées dans le repère local des autres robots
    return:
    - [vec_x, vec_y]: coordonées du vecteur champ de potentiel généré en (x1, x2)
    """
    x1, x2 = x[0,0], x[1,0]

    # Initialisation de la distance pour paramétrer la cercle
    R = 2 # rayon du cercle
    xc, yc, = 6, 12 # coordonnées du cercle
    dx = (x1 - xc)
    dy = (x2 - yc)
    dist = math.sqrt((dx)**2 + (dy)**2) # distance du point au centre du cercle
    # print(dist)

    # Champ de potentiel pour le cercle
    u_radial_x1 = -(dist - R)*(dx/dist)
    u_radial_x2 = -(dist - R)*(dy/dist)
    # print(-(dist - R)*x1)

    # Champ de potentiel pour la rotation continue sur le cercle
    u_tangent_x1 = -dy/dist
    u_tangent_x2 = dx/dist

    # Champ de potentiel pour la répulsion entre robots
    krep = 3 # facteur d'ajustement du champ de répulsion
    u_rep_x1 = 0
    u_rep_x2 = 0

    for i in range(len(Xr)):
        u_rep_x1 += krep*(x1 - Xr[i][0])/(math.sqrt((x1 - Xr[i][0])**2 + (x2 - Xr[i][1])**2)**3)
        u_rep_x2 += krep*(x2 - Xr[i][1])/(math.sqrt((x1 - Xr[i][0])**2 + (x2 - Xr[i][1])**2)**3)

    return u_radial_x1+u_tangent_x1+u_rep_x1, u_radial_x2+u_tangent_x2+u_rep_x2

def f_pot_ell(x, Xr):
    """
    Fonction générant un champ de potentiel elliptique (centré 6,10 | a=2, b=6)
    avec évitement d'obstacles.
    """
    x1, x2 = x[0,0], x[1,0]

    # Paramètres de l'ellipse
    xc, yc = 6, 10  # Centre
    a, b = 2, 6     # Demi-axes (2 en x, 6 en y)

    # Calcul de la distance normalisée à l'ellipse (r_norm = 1 sur l'ellipse)
    dx = (x1 - xc)
    dy = (x2 - yc)
    
    # On utilise une distance quadratique pondérée pour l'ellipse
    # Cela définit des courbes de niveau elliptiques
    r_norm = math.sqrt((dx/a)**2 + (dy/b)**2)

    # 1. Champ radial (Rappel vers l'ellipse)
    # Plus r_norm est loin de 1, plus la force de rappel est grande
    # On normalise les composantes pour garder une force cohérente
    u_radial_x1 = -(r_norm - 1) * (dx / (a**2 * r_norm))
    u_radial_x2 = -(r_norm - 1) * (dy / (b**2 * r_norm))

    # 2. Champ tangent (Vitesse de rotation le long de l'ellipse)
    # On dérive l'équation de l'ellipse pour trouver la tangente
    # Le vecteur [-df/dy, df/dx] donne la direction tangentielle
    u_tangent_x1 = - (dy / b**2) / r_norm
    u_tangent_x2 = (dx / a**2) / r_norm

    # 3. Champ de répulsion (Inchangé)
    krep = 3 
    u_rep_x1 = 0
    u_rep_x2 = 0

    for i in range(len(Xr)):
        dist_robot = math.sqrt((x1 - Xr[i][0])**2 + (x2 - Xr[i][1])**2)
        if dist_robot < 0.1: dist_robot = 0.1 # Éviter division par zéro
        u_rep_x1 += krep * (x1 - Xr[i][0]) / (dist_robot**3)
        u_rep_x2 += krep * (x2 - Xr[i][1]) / (dist_robot**3)

    # Gain de vitesse (optionnel : ajuster pour la vitesse de croisière souhaitée)
    kv = 1.0 
    return (u_radial_x1 + u_tangent_x1 + u_rep_x1) * kv, \
           (u_radial_x2 + u_tangent_x2 + u_rep_x2) * kv

def controle(x, Xr, f):
    """
    Contrôleur du robot sur un champ de potentiel généré par f.

    params:
    - x: Vecteur d'état du robot (x, y, v, theta)
    - Xr: liste contenant les coordonnées dans le repère local des autres robots
    - f: Fonction générant un champ de potentiel
    return:
    - u1, u2: consignes des accélérations (u1) et des rotations (u2)
    """

    v_des = np.array(f(x, Xr))
    # print(v_des)

    theta_des = np.arctan2(v_des[1], v_des[0])

    # # modif theta pour simu
    # theta_r_deg = -x[3,0]
    # theta_r = math.radians(theta_r_deg) + (math.pi / 2.0)

    # Pour réel
    theta_r = x[3,0] + (math.pi/2)

    err_theta = sawtooth(theta_des - theta_r)
    print("err_theta: ", math.degrees(err_theta))
    Kp = 2.0

    omega = Kp*err_theta
    accel = 1.*(np.linalg.norm(v_des) - x[2,0])
    
    return accel, omega


def get_positions(other_pos):
    """
    Extrait les coordonnées x, y du dictionnaire other_positions.
    Renvoie une liste de np.array([x, y]).
    """
    Xr = []
    
    # On utilise le verrou (lock) si vous appelez cette fonction 
    # depuis un thread différent de celui qui reçoit les données.
    for robot_id in other_positions:
        pos_list = other_positions[robot_id]
        # Format attendu : [x, 'N', y, 'W', heading]
        if len(pos_list) >= 3:
            x = pos_list[0]
            y = pos_list[2]
            Xr.append(np.array([float(x), float(y)]))
            
    return Xr

def luan_berger(x, u, cap, xloc, rcv):
    alpha = 0.5
    beta = 0.05
    u_total = u[0,0] + u[1,0]
    dv = -alpha * (x[2,0]**2) * np.sign(x[2,0]) + beta * (u_total**2) * np.sign(u_total)
    x[2,0] += dv * dt
    x_pred = x[0,0] + x[2,0] * np.cos(cap) * dt
    y_pred = x[1,0] + x[2,0] * np.sin(cap) * dt
    if rcv:
        x_hat = eta*x_pred + (1-eta)*xloc[0]
        y_hat = eta*y_pred + (1-eta)*xloc[1]

        return np.array([[x_hat], [y_hat], [x[2,0]], [cap]])
    else:
        return np.array([[x_pred], [y_pred], [x[2,0]], [cap]])




######################################## Communication socket entre robots########################################
# Remplacez par les vraies IP de vos robots
ROBOTS_CONFIG = {
    "robot1": {"ip": "172.20.25.206", "port": 5000, "sim_id": 1},
    "robot2": {"ip": "172.20.25.207", "port": 5000, "sim_id": 2},
    "robot3": {"ip": "172.20.25.215", "port": 5000, "sim_id": 3},
    "robot4": {"ip": "172.20.25.217", "port": 5000, "sim_id": 4}
}

# Identifiant de CE robot (à changer sur chaque machine : "robot1", "robot2" ou "robot3")
MY_ID = "robot2"
MY_IP = ROBOTS_CONFIG[MY_ID]["ip"]
MY_PORT = ROBOTS_CONFIG[MY_ID]["port"]

# Dictionnaire pour stocker les positions reçues
other_positions = {}

def position_receiver():
    """Thread qui écoute les messages entrants."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((MY_IP, MY_PORT))
        # print(f"[{MY_ID}] Écoute sur {MY_IP}:{MY_PORT}...")
        print("[{}] Écoute sur {}:{}...".format(MY_ID, MY_IP, MY_PORT))
        while True:
            data, addr = s.recvfrom(1024)
            message = json.loads(data.decode())
            sender = message["id"]
            poso = message["pos"]
            other_positions[sender] = poso
            # print(f" Reçu de {sender}: {pos}")
            print(" Reçu de {}: {}".format(sender, poso))

def position_sender():
    """Thread qui envoie ma position aux autres."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        while True:
            # # Simulation de position (remplacez par vos vraies coordonnées x, y)
            # my_current_pos = {"x": 1.2, "y": 3.4, "z": 0.0}
            
            # payload = json.dumps({"id": MY_ID, "pos": my_current_pos}).encode()

            #Format 2 pour le test de communication
            # Format: [lat_val, lat_dir, lon_val, lon_dir, timestamp/alt]
            # pos = [48.40, 'N', 4.48, 'W', 0.0] 

            # 2. Préparation du message 
            # On envoie la liste directement dans le dictionnaire
            # my_current_pos = pos 
            
            payload_dict = {
                "id": MY_ID,
                # "pos": my_current_pos # simulation unity
                "pos": pos
            }
            
            # 3. Encodage en JSON
            payload = json.dumps(payload_dict).encode()
            
            for r_id, config in ROBOTS_CONFIG.items():
                if r_id != MY_ID:
                    try:
                        s.sendto(payload, (config["ip"], config["port"]))
                    except Exception as e:
                        # print(f"Erreur d'envoi vers {r_id}: {e}")
                        print("Erreur d'envoi vers {}: {}".format(r_id, e))
            
            time.sleep(0.5)  # Envoi toutes les 500ms

######################################## Communication socket avec serveur ########################################
MY_SIM_ID = ROBOTS_CONFIG[MY_ID]["sim_id"]

UNITY_IP = "172.20.26.1" # Adresse IP du pc hébergeant le serveur unity
COMMAND_PORT = 5005
POSE_PORT = 5006
PASSWORD = " "

def unity_receiver_thread():
    """Récupère la télémétrie depuis Unity."""
    global my_current_pos
    sock_pose = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_pose.bind(("0.0.0.0", POSE_PORT))
    sock_pose.settimeout(1.0)
    
    while True:
        try:
            data, _ = sock_pose.recvfrom(1024)
            pose = json.loads(data.decode("utf-8"))
            if pose['id'] == MY_SIM_ID:
                # with lock:
                my_current_pos = [pose['X'], 'N', pose['Y'], 'W', pose['heading_deg']]
        except socket.timeout:
            pass
        except Exception as e:
            print("Erreur Unity Recv: {}".format(e))

# --- THREAD 2 : ENVOI COMMANDES UNITY ---
def unity_sender_thread():
    global u1, u2
    """Envoie les commandes de pilotage à Unity."""
    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        cmd_msg = {
            "id": MY_SIM_ID,
            "password": PASSWORD,
            "cmd_left": u1,
            "cmd_right": u2,
        }
        try:
            sock_cmd.sendto(json.dumps(cmd_msg).encode("utf-8"), (UNITY_IP, COMMAND_PORT))
        except Exception as e:
            print("Erreur Unity Send: {}".format(e))
        time.sleep(0.1) # 10Hz

######################################## Communication socket avec IHM ########################################
# Configuration réseau
DEST_IP = "172.20.26.99" # <--- METS L'IP DE TON PC DELL ICI
DEST_PORT = 5005


def sender_thread_ihm(data_getter):
    """
    data_getter: une fonction ou une variable globale qui contient 
    [cap, xloc_0, xloc_1]
    """
    sock_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        try:
            # On récupère les dernières données calculées
            cap, x, y = data_getter() 
            
            # On prépare le message en JSON
            msg = json.dumps({"cap": float(cap), "x": float(x), "y": float(y)})
            
            # Envoi UDP
            sock_sender.sendto(msg.encode(), (DEST_IP, DEST_PORT))
        except Exception as e:
            print("Erreur envoi IHM: {}".format(e))
            
        time.sleep(0.1) # 10 Hz est suffisant pour l'affichage

def get_current_ui_data():
    global cap, x
    # 'x' et 'cap' sont tes variables d'état calculées dans la boucle while
    # Il peut être nécessaire de les déclarer en global ou d'utiliser un objet
    return cap, x[0,0], x[1,0]

if __name__ == "__main__":
    # Initialisation capteurs
    ard = ArduinoIO()
    imu = Imu9IO()
    gps = GpsIO()

    # --- CONFIGURATION DU LOGGING ---
    log_filename = "mission_log_{}.csv".format(int(time.time()))
    log_file = open(log_filename, mode='w', newline='')
    log_writer = csv.writer(log_file)
    # En-tête du fichier CSV
    log_writer.writerow(['timestamp', 'lat_nmea', 'lon_nmea', 'x_local', 'y_local', 'u1', 'u2', 'cap_rad', 'mx', 'my', 'mz', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])
    #

    # Initialisation temps de simulation
    Tmax = 180
    dt = 0.1

    t0 =  time.time()
    t = t0

    xlloc = np.array([12, 10])

    threads = [
        # threading.Thread(target=unity_receiver_thread, daemon=True),
        # threading.Thread(target=unity_sender_thread, daemon=True),
        threading.Thread(target=position_receiver, daemon=True),
        threading.Thread(target=position_sender, daemon=True),
        threading.Thread(target=sender_thread_ihm, args=(get_current_ui_data,), daemon=True)

    ]
    
    for th in threads:
        th.start()
    
    print("[{}] Système multi-threads prêt.".format(MY_ID))

    # Initialisation vecteur état robot
    x = np.array([[0], [0], [0], [0]]) # x, y, v, theta

    try:
        while (t - t0) < Tmax:
            t = time.time()
            # print("current_pos ", my_current_pos)

            ############ GPS #############
            # Mise à jour de la position GPS si possible
            gll_ok, gll_pos = gps.read_gll_non_blocking()
            # if gll_ok:
            if gll_pos[0] != 0:
                pos = gll_pos
                print("pos: ", pos)
                # x, y = local_coord(pos)


            ############ IMU #############
            mag = imu.read_mag_raw()
            acc = imu.read_accel_raw()
            gyr = imu.read_gyro_raw()
            # print("mag: ", mag)
            cap = cap_calib(mag, MY_ID)
            print("cap: ", math.degrees(cap))

            ############ Socket #############
            # Xr = get_positions()

            ############ Controle #############
            Xr = get_positions(other_positions)
            # print("Xr: ", Xr)
            # uc1, uc2 = controle(x, Xr, f_pot)
            uc1, uc2 = controle(x, Xr, f_pot_ell)
            # u1 = max(-5,min((uc1 - 0.5*uc2), 5))
            # u2 = max(-5,min((uc1 + 0.5*uc2), 5))
            u1 = 1. - 0.6*uc2
            u2 = 1. + 0.6*uc2
            if MY_ID == "robot3":
                ard.send_arduino_cmd_motor(50*u2, 50*u1)
            else:    
                ard.send_arduino_cmd_motor(50*u1, 50*u2)
            u = np.array([[u1], [u2]])

            # print("uc1 ", uc1)
            # print("uc2 ", uc2)
            print("u1 ", u1)
            print("u2 ", u2)
            # x = np.array([[my_current_pos[0]], [my_current_pos[2]], [0], [my_current_pos[4]]]) # x, y, v, theta => pour simulation unity
            xloc = local_coord(pos, 0)
            # if (xlloc[0] != xloc[0]) or (xlloc[1] != xloc[1]):
            #     print("no new data")
            #     x = luan_berger(x, u, cap, xloc, True)
            #     xlloc = xloc
            # else:
            #     x = luan_berger(x, u, cap, xloc, False)
            x = luan_berger(x, u, cap, xloc, True)
            
            # print("xloc: ", xloc)
            # x = np.array([[xloc[0]], [xloc[1]], [0], [cap]]) # x, y, v, theta(rad) => pour réel

            print("état: ", x)
            log_writer.writerow([
                round(t - t0, 2), 
                pos[0], pos[2],     
                round(float(xloc[0]), 3),  # Ajout de float()
                round(float(xloc[1]), 3),  # Ajout de float()
                round(float(u1), 2),       # Ajout de float()
                round(float(u2), 2),       # Ajout de float()
                round(float(cap), 4),      # Correction ici : float(cap)
                round(float(mag[0]), 4),
                round(float(mag[1]), 4),
                round(float(mag[2]), 4),
                round(float(acc[0]), 4),
                round(float(acc[1]), 4),
                round(float(acc[2]), 4),
                round(float(gyr[0]), 4),
                round(float(gyr[1]), 4),
                round(float(gyr[2]), 4)
            ])
            # time.sleep(dt)

        # ard.send_arduino_cmd_motor(50,50)
        ard.send_arduino_cmd_motor(0,0)


    except KeyboardInterrupt:
        print("Arrêt du robot...")

    finally:
        # Fermeture propre du fichier même en cas d'erreur ou d'arrêt
        log_file.close()
        print("Données enregistrées dans {}".format(log_filename))

    