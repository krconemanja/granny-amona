import arduino_driver_v2 as ardudrv
import sys
import time

if __name__ == "__main__":
    
    timeout = 1.0    
    tloop = 1.0 # 1 Hz loop
    ard = ardudrv.ArduinoIO()
    print ("Arduino status is",ard.get_arduino_status())

    print ("Hit Ctrl+C to get out!")
    while True:
        print ("arduino motors rc : ",ard.get_arduino_cmd_motor())
        print ("arduino ESC : ",ard.get_arduino_cmd_motor_esc())
        print ("RC sel channels : ",ard.get_arduino_rc_chan())
        print ("RC 8 raw channels :",ard.get_arduino_raw_rc_chan())
        time.sleep(tloop)
