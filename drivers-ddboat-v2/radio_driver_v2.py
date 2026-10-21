import serial
import os
import time
import sys

# LORA radio driver
class RadioIO():
    def __init__(self,dev_tty=1,boat_id=200):
        self.baud_rate = 115200
        if dev_tty == 1:
            self.dev_tty = '/dev/ttyLORA1'
        if dev_tty == 2:
            self.dev_tty = '/dev/ttyLORA2'
        self.init_line()
        self.boat_id = boat_id
        self.gs_id = 224

    def init_line(self,timeout=1.0):
        self.ser = serial.Serial(self.dev_tty,self.baud_rate,timeout=timeout)

    # in principle , no need to use this function , baudrate is normally 115200 with
    # the current software version
    def set_baudrate(self,baudrate=115200):
        self.baud_rate = baudrate
        st = os.system ("stty -F %s %d"%(self.dev_tty,self.baud_rate))
        print (st)
        st = os.system ("stty -F %s"%(self.dev_tty))
        print (st)

    def close_line(self):
        self.ser.close()

    def send (self,id_src,id_dst,msg):
        l = len(msg)
        st = "%d:%d:%d:%s"%(id_src,id_dst,l,msg)
        self.ser.write(st.encode())

    def receive (self,timeout=0.01):
        self.ser.timeout=timeout
        v=""
        try:
            v=self.ser.readline()
        except:
            print ("error on receiving LORA")
        msg_ok=False
        msg=None
        if len(v)>0:
            msg_ok = True
            msg = v.decode()
        return msg_ok, msg

if __name__ == "__main__":    
    timeout_rx = 10.0
    print (sys.argv)
    try:
        boat_id = int(sys.argv[1])
    except:
        boat_id = 200
    # test with com1
    radio = RadioIO(boat_id=boat_id)
    for cnt in range(3):
        msg = "Hello server, are you awake ?"
        print ('send "%s" to ground station'%(msg))
        radio.send(radio.boat_id,radio.gs_id,msg)

        t0 = time.time()
        while True:
            msg_ok, msg = radio.receive()
            if msg_ok:
                print ("OK: get answer from ground station : ")
                print (msg)
                break
            time.sleep(0.1)
            if (time.time()-t0) > timeout_rx:
                print ("KO: timeout reached ...")
                break
        time.sleep(1.0)
    radio.close_line()

    # test with com2
    radio = RadioIO(dev_tty=2,boat_id=boat_id)
    for cnt in range(3):
        msg = "Hello server, are you awake ?"
        print ('send "%s" to ground station'%(msg))
        radio.send(radio.boat_id,radio.gs_id,msg)

        t0 = time.time()
        while True:
            msg_ok, msg = radio.receive()
            if msg_ok:
                print ("OK: get answer from ground station : ")
                print (msg)
                break
            time.sleep(0.1)
            if (time.time()-t0) > timeout_rx:
                print ("KO: timeout reached ...")
                break
        time.sleep(1.0)
    radio.close_line() 

