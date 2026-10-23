import sys
import os
import time

# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import gps_driver_v2 as gpddrv

gps = gpddrv.GpsIO() # create a GPS object
gps.set_filter_speed("0") # allowing GPS measures to change even if the DDBoat is not moving
cnt = 5 # takes 5 GPS measures

while True:
    gll_ok,gll_data=gps.read_gll_non_blocking()
    if gll_ok: # GPGLL message received
        print (gll_data)
        cnt -= 1
        if cnt==0:
            break
    time.sleep(0.01)