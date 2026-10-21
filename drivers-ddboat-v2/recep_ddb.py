import socket
from arduino_driver_v2 import ArduinoIO
# Config Réseau locale
UDP_IP = "0.0.0.0" # Écoute sur toutes les interfaces
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def piloter_moteurs(v, d, ard):
    # Mixage différentiel
    # Si d > 0 (droite), le moteur gauche accélère et le droit ralentit
    moteur_gauche = v + d
    moteur_droite = v - d

    # Limitation entre -100 et 100 (pour ton driver)
    moteur_gauche = max(min(moteur_gauche, 100), -100)
    moteur_droite = max(min(moteur_droite, 100), -100)

    print("Moteur G: {} | Moteur D: {}".format(moteur_gauche,moteur_droite))
    
    # --- ICI : Ajoute l'appel à ton driver spécifique ---
    ard.send_arduino_cmd_motor(moteur_gauche, moteur_droite)
    # exemple: ton_driver.set_speed(moteur_gauche, moteur_droite)

print("En attente de commandes WiFi...")
while True:
    ard = ArduinoIO()
    data, addr = sock.recvfrom(1024)
    msg = data.decode().split(',')
    v_consigne = int(msg[0])
    d_consigne = int(msg[1])
    
    piloter_moteurs(v_consigne, d_consigne, ard)