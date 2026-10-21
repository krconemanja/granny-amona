import time

from arduino_driver_v2 import ArduinoIO


if __name__ == "__main__":
    # Initialisation capteurs
    ard = ArduinoIO()

    t0 =  time.time()
    t = t0

    while (t-t0) < 2:
        t = time.time()
        ard.send_arduino_cmd_motor(50,50)
    print("out cmd 0")
    time.sleep(0.1)
    ard.send_arduino_cmd_motor(0,0)
    time.sleep(0.1)
    print("cmd 0")
